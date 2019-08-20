// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - AEAD Support
 *
 * This file contains the kernel crypto API implementation for the
 * AEAD algorithms supported by HSE.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <crypto/gcm.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/aead.h>

#include "hse.h"
#include "hse-abi.h"
#include "hse-mu.h"

/**
 * Maximum AEAD key size supported by HSE
 */
#define HSE_AEAD_MAX_KEY_SIZE AES_MAX_KEY_SIZE

/**
 * hse_aead_alg - HSE AEAD algorithm data
 * @aead: generic AEAD cipher
 * @registered: algorithm registered flag
 * @key_group_id: key group id from key catalog
 * @sym_keys: list of available symmetric key handles
 * @dev: HSE device
 * @mu_inst: MU instance
 */
struct hse_aead_alg {
	struct aead_alg aead;
	bool registered;
	struct list_head *sym_keys;
	struct device *dev;
	void *mu_inst;
};

/**
 * struct hse_aead_tfm_ctx - AEAD transformation object context/state
 * @key_desc: HSE service descriptor used for setkey operations
 * @key_desc_dma: DMA address of HSE key service descriptor
 * @crt_key: key handle currently in use
 * @key_info: key flags, used for import
 * @key_info_dma: key info DMA address
 * @key: buffer containing the current key
 * @key_dma: key buffer DMA address
 * @channel: MU channel
 */
struct hse_aead_tfm_ctx {
	struct hse_srv_desc key_desc;
	dma_addr_t key_desc_dma;
	struct hse_key *key_handle;
	struct hse_key_info key_info;
	dma_addr_t key_info_dma;
	u8 key_buf[HSE_AEAD_MAX_KEY_SIZE];
	dma_addr_t key_buf_dma;
	u8 channel;
};

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
};

/**
 * Get HSE AEAD algorithm data from AEAD transformation
 */
static inline struct hse_aead_alg *get_hse_aead_alg(struct crypto_aead *tfm)
{
	struct aead_alg *alg = crypto_aead_alg(tfm);

	return container_of(alg, struct hse_aead_alg, aead);
}

/**
 * hse_gcm_setkey - set key for GCM transformations
 * @tfm: AEAD transformation object
 * @key: input key
 * @keylen: input key length in bytes
 */
static int hse_gcm_setkey(struct crypto_aead *tfm, const u8 *key,
			  unsigned int keylen)
{
	struct hse_aead_alg *alg = get_hse_aead_alg(tfm);
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	int err;

	err = hse_check_aes_keylen(keylen);
	if (err) {
		crypto_aead_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return err;
	}

	if (IS_ENABLED(CONFIG_VERBOSE_DEBUG))
		print_hex_dump_debug("gcm(aes) key: ", DUMP_PREFIX_OFFSET,
				     16, 4, key, keylen, false);

	/* Make sure key is located in a DMAable area */
	memcpy(tctx->key_buf, key, keylen);

	tctx->key_info.key_flags = HSE_KF_USAGE_ENCRYPT | HSE_KF_USAGE_DECRYPT;
	tctx->key_info.key_bit_len = keylen * BITS_PER_BYTE;
	tctx->key_info.key_type = HSE_KEY_TYPE_AES;

	/* DAM sync key memory accessed by HSE */
	dma_sync_single_for_device(alg->dev, tctx->key_buf_dma, keylen,
				   DMA_TO_DEVICE);

	dma_sync_single_for_device(alg->dev, tctx->key_info_dma,
				   sizeof(tctx->key_info), DMA_TO_DEVICE);

	tctx->key_desc.srv_id = HSE_SRV_ID_IMPORT_KEY;
	tctx->key_desc.priority = HSE_SRV_PRIO_HIGH;

	tctx->key_desc.import_key_req.key_handle = tctx->key_handle->handle;
	tctx->key_desc.import_key_req.key_info = tctx->key_info_dma;
	tctx->key_desc.import_key_req.key = tctx->key_buf_dma;
	tctx->key_desc.import_key_req.key_len = keylen;
	tctx->key_desc.import_key_req.cipher_key = HSE_INVALID_KEY_HANDLE;
	tctx->key_desc.import_key_req.auth_key = HSE_INVALID_KEY_HANDLE;

	/* DMA sync service descriptor memory accessed by HSE */
	dma_sync_single_for_device(alg->dev, tctx->key_desc_dma,
				   sizeof(tctx->key_desc), DMA_TO_DEVICE);
	/*
	 * HSE accepts 32-bit srv desc address. This check should never fail
	 * on S32xx parts because DDR is mapped in first 32-bit address space.
	 */
	if (unlikely(upper_32_bits(tctx->key_desc_dma))) {
		dev_err(alg->dev, "Out of range HSE service descriptor addr\n");
		return -EFAULT;
	}

	err = hse_mu_sync_req(alg->mu_inst, HSE_ANY_CHANNEL,
			      lower_32_bits(tctx->key_desc_dma));
	if (unlikely(err))
		dev_dbg(alg->dev, "%s: key import request failed: %d\n",
			__func__, err);

	return err;
}

/**
 * hse_gcm_setauthsize - set authentication tag (MAC) size for GCM tfm
 * @tfm: AEAD GCM transformation
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
	struct hse_aead_alg *alg = get_hse_aead_alg(tfm);
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

/*
 * hse_gcm_crypt - encrypt/decrypt AEAD operation
 * @req: AEAD encrypt/decrypt request
 * @encrypt: whether the request is encrypt or not
 */
static int hse_gcm_crypt(struct aead_request *req, bool encrypt)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct hse_aead_alg *alg = get_hse_aead_alg(tfm);
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
	hse_req->auth_cipher_mode = HSE_AUTH_CIPHER_MODE_GCM;
	hse_req->cipher_dir =
		encrypt ? HSE_CIPHER_DIR_ENCRYPT : HSE_CIPHER_DIR_DECRYPT;
	hse_req->key_handle = tctx->key_handle->handle;
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
	/*
	 * HSE accepts 32-bit srv desc address. This check should never fail
	 * on S32xx parts because DDR is mapped in first 32-bit address space.
	 */
	if (unlikely(upper_32_bits(rctx->desc_dma))) {
		dev_err(alg->dev, "Out of range HSE service descriptor addr\n");
		err = -EFAULT;
		goto err_unmap_desc;
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
 * hse_aead_init - init AEAD transformation object
 * @tfm: AEAD transformation
 */
static int hse_aead_init(struct crypto_aead *tfm)
{
	struct hse_aead_alg *alg = get_hse_aead_alg(tfm);
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	struct hse_drvdata *drv = dev_get_drvdata(alg->dev);
	int err;

	crypto_aead_set_reqsize(tfm, sizeof(struct hse_aead_req_ctx));

	/* acquire key handle from key list */
	spin_lock(&drv->key_lock);
	tctx->key_handle = list_first_entry_or_null(alg->sym_keys,
						    struct hse_key, entry);
	if (unlikely(!tctx->key_handle)) {
		dev_err(alg->dev, "%s: cannot acquire key slot\n", __func__);
		spin_unlock(&drv->key_lock);
		return -ENOKEY;
	}
	list_del(&tctx->key_handle->entry);
	spin_unlock(&drv->key_lock);

	/* DMA map key buffer */
	tctx->key_buf_dma = dma_map_single_attrs(alg->dev, tctx->key_buf,
						 HSE_AEAD_MAX_KEY_SIZE,
						 DMA_TO_DEVICE,
						 DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->key_buf_dma))) {
		dev_err(alg->dev, "unable to map key buffer\n");
		err = -ENOMEM;
		goto err_free_key_slot;
	}

	/* DMA map key info */
	tctx->key_info_dma = dma_map_single_attrs(alg->dev, &tctx->key_info,
						  sizeof(tctx->key_info),
						  DMA_TO_DEVICE,
						  DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->key_info_dma))) {
		dev_err(alg->dev, "unable to map key info\n");
		err = -ENOMEM;
		goto err_unmap_key;
	}

	/* DMA map key service descriptor */
	tctx->key_desc_dma = dma_map_single_attrs(alg->dev, &tctx->key_desc,
						  sizeof(tctx->key_desc),
						  DMA_TO_DEVICE,
						  DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->key_desc_dma))) {
		dev_err(alg->dev, "unable to map key service descriptor\n");
		err = -ENOMEM;
		goto err_unmap_key_info;
	}

	/* acquire MU shared channel */
	err = hse_mu_channel_acquire(alg->mu_inst, &tctx->channel, false);
	if (unlikely(err))
		goto err_unmap_key_desc;

	return 0;

err_unmap_key_desc:
	dma_unmap_single_attrs(alg->dev, tctx->key_desc_dma,
			       sizeof(tctx->key_desc), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_key_info:
	dma_unmap_single_attrs(alg->dev, tctx->key_info_dma,
			       sizeof(tctx->key_info), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_key:
	dma_unmap_single_attrs(alg->dev, tctx->key_buf_dma,
			       HSE_AEAD_MAX_KEY_SIZE, DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
err_free_key_slot:
	list_add_tail(&tctx->key_handle->entry, alg->sym_keys);
	return err;
}

/**
 * hse_aead_exit - deinitialize AEAD transformation object
 * @tfm: AEAD transformation
 */
static void hse_aead_exit(struct crypto_aead *tfm)
{
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	struct hse_aead_alg *alg = get_hse_aead_alg(tfm);

	/* release MU shared channel */
	hse_mu_channel_release(alg->mu_inst, tctx->channel);

	/* unmap key service descriptor */
	dma_unmap_single_attrs(alg->dev, tctx->key_desc_dma,
			       sizeof(tctx->key_desc), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);

	/* unmap key info */
	dma_unmap_single_attrs(alg->dev, tctx->key_info_dma,
			       sizeof(tctx->key_info), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);

	/* unmap key buffer */
	dma_unmap_single_attrs(alg->dev, tctx->key_buf_dma,
			       HSE_AEAD_MAX_KEY_SIZE, DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);

	/* add key slot back */
	list_add_tail(&tctx->key_handle->entry, alg->sym_keys);
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
					     CRYPTO_ALG_TYPE_AEAD,
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

		/* only GCM is supported => using AES keys */
		alg->sym_keys = &drvdata->aes_keys;

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
 * hse_skcipher_free - free AEAD algs
 */
void hse_aead_unregister(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(aead_algs); i++)
		if (aead_algs[i].registered)
			crypto_unregister_aead(&aead_algs[i].aead);
}
