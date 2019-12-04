// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - AEAD Support
 *
 * This file contains the implementation of the authenticated encryption with
 * additional data algorithms supported for hardware offloading via HSE.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/crypto.h>
#include <crypto/aead.h>
#include <crypto/internal/aead.h>
#include <crypto/aes.h>
#include <crypto/gcm.h>
#include <crypto/scatterwalk.h>

#include "hse-abi.h"
#include "hse-core.h"
#include "hse-mu.h"

#define HSE_AEAD_MAX_KEY_SIZE    AES_MAX_KEY_SIZE

/**
 * hse_aead_alg - HSE AEAD algorithm data
 * @aead: generic AEAD cipher
 * @auth_mode: authenticated encryption mode
 * @key_type: type of key used
 * @dev: HSE device
 * @mu_inst: MU instance
 * @registered: algorithm registered flag
 */
struct hse_aead_alg {
	struct aead_alg aead;
	enum hse_auth_cipher_mode auth_mode;
	enum hse_key_type key_type;
	struct device *dev;
	void *mu_inst;
	bool registered;
};

/**
 * struct hse_aead_tfm_ctx - crypto transformation context
 * @srv_desc: service descriptor for setkey ops
 * @srv_desc_dma: service descriptor DMA address
 * @key_slot: current key entry in cipher key ring
 * @keyinf: key information/flags, used for import
 * @keyinf_dma: key information/flags DMA address
 * @keybuf: buffer containing current key
 * @keybuf_dma: current key DMA address
 * @keylen: size of the last imported key
 * @channel: MU channel
 */
struct hse_aead_tfm_ctx {
	struct hse_srv_desc srv_desc;
	dma_addr_t srv_desc_dma;
	struct hse_key *key_slot;
	struct hse_key_info keyinf;
	dma_addr_t keyinf_dma;
	u8 keybuf[HSE_AEAD_MAX_KEY_SIZE];
	dma_addr_t keybuf_dma;
	unsigned int keylen;
	u8 channel;
} ____cacheline_aligned;

/**
 * struct hse_aead_req_ctx - AEAD request context
 * @desc: HSE service descriptor used for encrypt/decrypt operations
 * @desc_dma: HSE service descriptor DMA address
 * @iv: initialization vector
 * @iv_dma: iv DMA address
 * @buf: linearized buffer for send/recv plaintext/ciphertext to HSE
 * @buf_dma: linear buffer DMA address
 * @buflen: length of dma mapped HSE request buffer
 * @encrypt: whether this is an encrypt or decrypt request
 */
struct hse_aead_req_ctx {
	struct hse_srv_desc desc;
	dma_addr_t desc_dma;
	u8 iv[GCM_AES_IV_SIZE];
	dma_addr_t iv_dma;
	void *buf;
	dma_addr_t buf_dma;
	u32 buflen;
	bool encrypt;
} ____cacheline_aligned;

/**
 * hse_aead_get_alg - get AEAD algorithm data from crypto transformation
 * @tfm: crypto aead transformation
 */
static inline struct hse_aead_alg *hse_aead_get_alg(struct crypto_aead *tfm)
{
	struct aead_alg *alg = crypto_aead_alg(tfm);

	return container_of(alg, struct hse_aead_alg, aead);
}

/**
 * hse_gcm_setauthsize - set authentication tag (MAC) size for GCM tfm
 * @tfm: crypto aead transformation
 * @authsize: auth tag size
 *
 * Validate authentication tag size for GCM transformations.
 */
static int hse_gcm_setauthsize(struct crypto_aead *tfm, unsigned int authsize)
{
	switch (authsize) {
	case 4:
	case 8:
	case 12:
	case 13:
	case 14:
	case 15:
	case 16:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * hse_aead_done - AEAD request done callback
 * @mu_inst: MU instance
 * @channel: service channel index
 * @aeadreq: AEAD request
 *
 * Callback called when HSE completes an AEAD request.
 */
static void hse_aead_done(void *mu_inst, u8 channel, void *aeadreq)
{
	struct aead_request *req = (struct aead_request *)aeadreq;
	struct hse_aead_req_ctx *rctx = aead_request_ctx(req);
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	u32 maclen = crypto_aead_authsize(tfm);
	u32 ivlen = crypto_aead_ivsize(tfm);
	int err, nbytes, dstlen;

	err = hse_mu_async_req_recv(mu_inst, channel);
	if (unlikely(err)) {
		dev_dbg(alg->dev, "%s: AEAD request failed: %d\n",
			__func__, err);
		goto out;
	}

	dma_unmap_single(alg->dev, rctx->buf_dma, rctx->buflen,
			 DMA_BIDIRECTIONAL);

	if (IS_ENABLED(CONFIG_VERBOSE_DEBUG))
		print_hex_dump_debug("gcm(aes) req out: ", DUMP_PREFIX_OFFSET,
				     16, 4, rctx->buf, rctx->buflen, true);

	/* copy result from linear buffer into destination SG */
	dstlen = req->assoclen + req->cryptlen +
		 (rctx->encrypt ? maclen : -maclen);
	nbytes = sg_copy_from_buffer(req->dst, sg_nents(req->dst),
				     rctx->buf, dstlen);
	if (unlikely(nbytes != dstlen))
		err = -ENODATA;

out:
	kfree(rctx->buf);
	dma_unmap_single(alg->dev, rctx->iv_dma, ivlen, DMA_TO_DEVICE);
	dma_unmap_single(alg->dev, rctx->desc_dma, sizeof(rctx->desc),
			 DMA_TO_DEVICE);

	aead_request_complete(req, err);
}

/**
 * hse_gcm_crypt - encrypt/decrypt AEAD operation
 * @req: AEAD encrypt/decrypt request
 * @encrypt: whether the request is encrypt or not
 */
static int hse_gcm_crypt(struct aead_request *req, bool encrypt)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	struct hse_aead_req_ctx *rctx = aead_request_ctx(req);
	u32 maclen = crypto_aead_authsize(tfm);
	u32 ivlen = crypto_aead_ivsize(tfm);
	struct hse_aead_srv *hse_req;
	u32 cryptlen, srclen;
	int err, nbytes;

	/* Make sure IV is located in a DMAable area before DMA mapping it */
	memcpy(&rctx->iv, req->iv, ivlen);

	/* for decrypt req->cryptlen includes the MAC length */
	cryptlen = req->cryptlen - (!encrypt ? maclen : 0);

	/* alloc linearized buffer for HSE request */
	rctx->buflen = req->assoclen + cryptlen + maclen;
	rctx->buf = kzalloc(rctx->buflen, GFP_KERNEL);
	if (IS_ERR_OR_NULL(rctx->buf))
		return -ENOMEM;

	/* copy source SG into linear buffer */
	srclen = req->assoclen + req->cryptlen;
	nbytes = sg_copy_to_buffer(req->src, sg_nents(req->src),
				   rctx->buf, srclen);
	if (nbytes != srclen) {
		err = -ENODATA;
		goto err_free_buf;
	}

	if (IS_ENABLED(CONFIG_VERBOSE_DEBUG))
		print_hex_dump_debug("gcm(aes) req in: ", DUMP_PREFIX_OFFSET,
				     16, 4, rctx->buf, rctx->buflen, true);

	/* DMA map IV and req buffer */
	rctx->iv_dma = dma_map_single(alg->dev, rctx->iv, ivlen, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->iv_dma))) {
		dev_err(alg->dev, "unable to map IV buffer\n");
		err = -ENOMEM;
		goto err_free_buf;
	}

	rctx->buf_dma = dma_map_single(alg->dev, rctx->buf, rctx->buflen,
				       DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(alg->dev, rctx->buf_dma))) {
		dev_err(alg->dev, "unable to map HSE request buffer\n");
		err = -ENOMEM;
		goto err_unmap_iv;
	}

	/* prepare HSE service descriptor */
	rctx->desc.srv_id = HSE_SRV_ID_AEAD;
	rctx->desc.priority = HSE_SRV_PRIO_MED;
	hse_req = &rctx->desc.aead_req;

	hse_req->access_mode = HSE_ACCESS_MODE_ONE_PASS;
	hse_req->auth_cipher_mode = alg->auth_mode;
	hse_req->cipher_dir =
		encrypt ? HSE_CIPHER_DIR_ENCRYPT : HSE_CIPHER_DIR_DECRYPT;
	hse_req->key_handle = tctx->key_slot->handle;
	hse_req->iv_len = ivlen;
	hse_req->iv = rctx->iv_dma;
	hse_req->aad_len = req->assoclen;
	hse_req->aad = rctx->buf_dma;
	hse_req->input_len = cryptlen;
	hse_req->input = hse_req->aad + req->assoclen;
	hse_req->tag_len = maclen;
	hse_req->tag = hse_req->input + cryptlen;
	hse_req->output = hse_req->input;

	/* DMA map service descriptor */
	rctx->desc_dma = dma_map_single(alg->dev, &rctx->desc,
					sizeof(rctx->desc), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->desc_dma))) {
		dev_err(alg->dev, "unable to map HSE service descriptor\n");
		err = -ENOMEM;
		goto err_unmap_buf;
	}

	rctx->encrypt = encrypt;
	err = hse_mu_async_req_send(alg->mu_inst, tctx->channel,
				    lower_32_bits(rctx->desc_dma),
				    req, hse_aead_done);
	if (err)
		goto err_unmap_desc;

	return -EINPROGRESS;

err_unmap_desc:
	dma_unmap_single(alg->dev, rctx->desc_dma, sizeof(rctx->desc),
			 DMA_TO_DEVICE);
err_unmap_buf:
	dma_unmap_single(alg->dev, rctx->buf_dma, rctx->buflen,
			 DMA_BIDIRECTIONAL);
err_unmap_iv:
	dma_unmap_single(alg->dev, rctx->iv_dma, ivlen, DMA_TO_DEVICE);
err_free_buf:
	kfree(rctx->buf);
	return err;
}

static int hse_gcm_encrypt(struct aead_request *req)
{
	return hse_gcm_crypt(req, true);
}

static int hse_gcm_decrypt(struct aead_request *req)
{
	return hse_gcm_crypt(req, false);
}

/**
 * hse_gcm_setkey - set key for GCM transformations
 * @tfm: crypto aead transformation
 * @key: input key
 * @keylen: input key length, in bytes
 */
static int hse_gcm_setkey(struct crypto_aead *tfm, const u8 *key,
			  unsigned int keylen)
{
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	int err;

	/* do not update the key if already imported */
	if (keylen == tctx->keylen &&
	    unlikely(!crypto_memneq(key, tctx->keybuf, keylen)))
		return 0;

	err = aes_check_keylen(keylen);
	if (err)
		return err;

	memcpy(tctx->keybuf, key, keylen);
	dma_sync_single_for_device(alg->dev, tctx->keybuf_dma, keylen,
				   DMA_TO_DEVICE);
	tctx->keylen = keylen;

	tctx->keyinf.key_flags = HSE_KF_USAGE_ENCRYPT | HSE_KF_USAGE_DECRYPT;
	tctx->keyinf.key_bit_len = keylen * BITS_PER_BYTE;
	tctx->keyinf.key_type = alg->key_type;

	dma_sync_single_for_device(alg->dev, tctx->keyinf_dma,
				   sizeof(tctx->keyinf), DMA_TO_DEVICE);

	tctx->srv_desc.srv_id = HSE_SRV_ID_IMPORT_KEY;
	tctx->srv_desc.priority = HSE_SRV_PRIO_HIGH;

	tctx->srv_desc.import_key_req.key_handle = tctx->key_slot->handle;
	tctx->srv_desc.import_key_req.key_info = tctx->keyinf_dma;
	tctx->srv_desc.import_key_req.sym.key = tctx->keybuf_dma;
	tctx->srv_desc.import_key_req.sym.keylen = keylen;
	tctx->srv_desc.import_key_req.cipher_key = HSE_INVALID_KEY_HANDLE;
	tctx->srv_desc.import_key_req.auth_key = HSE_INVALID_KEY_HANDLE;

	dma_sync_single_for_device(alg->dev, tctx->srv_desc_dma,
				   sizeof(tctx->srv_desc), DMA_TO_DEVICE);

	err = hse_mu_sync_req(alg->mu_inst, HSE_ANY_CHANNEL,
			      lower_32_bits(tctx->srv_desc_dma));
	if (unlikely(err))
		dev_dbg(alg->dev, "%s: key import request failed: %d\n",
			__func__, err);

	return err;
}

/**
 * hse_aead_init - crypto transformation init
 * @tfm: crypto aead transformation
 */
static int hse_aead_init(struct crypto_aead *tfm)
{
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	int err;

	crypto_aead_set_reqsize(tfm, sizeof(struct hse_aead_req_ctx));

	tctx->key_slot = hse_key_slot_acquire(alg->dev, alg->key_type);
	if (IS_ERR_OR_NULL(tctx->key_slot)) {
		dev_dbg(alg->dev, "%s: cannot acquire key slot\n", __func__);
		return PTR_ERR(tctx->key_slot);
	}

	/* DMA map key buffer */
	tctx->keybuf_dma = dma_map_single_attrs(alg->dev, tctx->keybuf,
						HSE_AEAD_MAX_KEY_SIZE,
						DMA_TO_DEVICE,
						DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->keybuf_dma))) {
		dev_err(alg->dev, "unable to map key buffer\n");
		err = -ENOMEM;
		goto err_release_key_slot;
	}

	/* DMA map key info */
	tctx->keyinf_dma = dma_map_single_attrs(alg->dev, &tctx->keyinf,
						sizeof(tctx->keyinf),
						DMA_TO_DEVICE,
						DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->keyinf_dma))) {
		dev_err(alg->dev, "unable to map key info\n");
		err = -ENOMEM;
		goto err_unmap_keybuf;
	}

	/* DMA map key service descriptor */
	tctx->srv_desc_dma = dma_map_single_attrs(alg->dev, &tctx->srv_desc,
						  sizeof(tctx->srv_desc),
						  DMA_TO_DEVICE,
						  DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->srv_desc_dma))) {
		dev_err(alg->dev, "unable to map key service descriptor\n");
		err = -ENOMEM;
		goto err_unmap_keyinf;
	}

	/* acquire MU shared channel */
	err = hse_mu_channel_acquire(alg->mu_inst, &tctx->channel, false);
	if (unlikely(err))
		goto err_unmap_srv_desc;

	return 0;

err_unmap_srv_desc:
	dma_unmap_single_attrs(alg->dev, tctx->srv_desc_dma,
			       sizeof(tctx->srv_desc), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_keyinf:
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_keybuf:
	dma_unmap_single_attrs(alg->dev, tctx->keybuf_dma,
			       HSE_AEAD_MAX_KEY_SIZE, DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
err_release_key_slot:
	hse_key_slot_release(alg->dev, tctx->key_slot);
	return err;
}

/**
 * hse_aead_exit - crypto transformation exit
 * @tfm: crypto aead transformation
 */
static void hse_aead_exit(struct crypto_aead *tfm)
{
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);

	hse_mu_channel_release(alg->mu_inst, tctx->channel);

	hse_key_slot_release(alg->dev, tctx->key_slot);

	dma_unmap_single_attrs(alg->dev, tctx->srv_desc_dma,
			       sizeof(tctx->srv_desc), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keybuf_dma, sizeof(tctx->keybuf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
}

/* AEAD algs to be registered */
static struct hse_aead_alg aead_algs[] = {
	{
		.aead = {
			.base = {
				.cra_name = "gcm(aes)",
				.cra_driver_name = "gcm-aes-hse",
				.cra_module = THIS_MODULE,
				.cra_priority = HSE_CRA_PRIORITY,
				.cra_flags = CRYPTO_ALG_ASYNC |
					     CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_ctxsize = sizeof(struct hse_aead_tfm_ctx),
				.cra_blocksize = 1,
			},
			.setkey = hse_gcm_setkey,
			.setauthsize = hse_gcm_setauthsize,
			.encrypt = hse_gcm_encrypt,
			.decrypt = hse_gcm_decrypt,
			.init = hse_aead_init,
			.exit = hse_aead_exit,
			.ivsize = GCM_AES_IV_SIZE,
			.maxauthsize = AES_BLOCK_SIZE,
		},
		.registered = false,
		.auth_mode = HSE_AUTH_CIPHER_MODE_GCM,
		.key_type = HSE_KEY_TYPE_AES,
	},
};

/**
 * hse_aead_register - register AEAD algorithms
 * @dev: HSE device
 */
void hse_aead_register(struct device *dev)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	int i, err = 0;

	/* register crypto algorithms the device supports */
	for (i = 0; i < ARRAY_SIZE(aead_algs); i++) {
		struct hse_aead_alg *alg = &aead_algs[i];
		const char *name = alg->aead.base.cra_name;

		alg->dev = dev;
		alg->mu_inst = drvdata->mu_inst;

		err = crypto_register_aead(&alg->aead);
		if (err) {
			dev_err(dev, "failed to register %s: %d", name, err);
		} else {
			alg->registered = true;
			dev_info(dev, "registered alg %s\n", name);
		}
	}
}

/**
 * hse_skcipher_unregister - unregister AEAD algorithms
 */
void hse_aead_unregister(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(aead_algs); i++)
		if (aead_algs[i].registered)
			crypto_unregister_aead(&aead_algs[i].aead);
}
