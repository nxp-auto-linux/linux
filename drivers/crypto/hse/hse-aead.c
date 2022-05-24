// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Authenticated Encryption and AEAD Support
 *
 * This file contains the implementation of the authenticated encryption with
 * additional data algorithms supported for hardware offloading via HSE.
 *
 * Copyright 2019-2022 NXP
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

#define HSE_AEAD_MAX_KEY_SIZE    AES_MAX_KEY_SIZE
#define HSE_AEAD_MAX_IV_SIZE     GCM_AES_IV_SIZE

/**
 * enum hse_alg_type - algorithm type
 * @HSE_ALG_TYPE_AEAD: AEAD
 * @HSE_ALG_TYPE_AUTHENC: authenticated encryption
 */
enum hse_alg_type {
	HSE_ALG_TYPE_AEAD = 0u,
	HSE_ALG_TYPE_AUTHENC = 1u,
};

/**
 * struct hse_aead_tpl - algorithm template
 * @cipher_name: cipher algorithm name
 * @cipher_drv: cipher driver name
 * @blocksize: block size
 * @ivsize: initialization vector size
 * @maxauthsize: maximum authentication tag size
 * @auth_mode: authenticated encryption mode
 * @key_type: type of key used
 * @alg_type: algorithm type
 */
struct hse_aead_tpl {
	char aead_name[CRYPTO_MAX_ALG_NAME];
	char aead_drv[CRYPTO_MAX_ALG_NAME];
	unsigned int blocksize;
	unsigned int ivsize;
	unsigned int maxauthsize;
	enum hse_auth_cipher_mode auth_mode;
	enum hse_key_type key_type;
	enum hse_alg_type alg_type;
};

/**
 * hse_aead_alg - algorithm private data
 * @aead: generic AEAD cipher
 * @entry: position in supported algorithms list
 * @auth_mode: authenticated encryption mode
 * @alg_type: algorithm type
 * @key_type: type of key used
 * @dev: HSE device
 */
struct hse_aead_alg {
	struct aead_alg aead;
	struct list_head entry;
	enum hse_auth_cipher_mode auth_mode;
	enum hse_key_type key_type;
	enum hse_alg_type alg_type;
	struct device *dev;
};

/**
 * struct hse_aead_tfm_ctx - crypto transformation context
 * @srv_desc: service descriptor for setkey ops
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
	struct hse_key *key_slot;
	struct hse_key_info keyinf;
	dma_addr_t keyinf_dma;
	u8 keybuf[HSE_AEAD_MAX_KEY_SIZE];
	dma_addr_t keybuf_dma;
	unsigned int keylen;
	u8 channel;
};

/**
 * struct hse_aead_req_ctx - crypto request context
 * @srv_desc: service descriptor for AEAD ops
 * @iv: current initialization vector
 * @iv_dma: current initialization vector DMA address
 * @buf: linearized input/output buffer
 * @buf_dma: linearized input/output buffer DMA address
 * @buflen: size of current linearized input buffer
 * @direction: encrypt/decrypt selector
 */
struct hse_aead_req_ctx {
	struct hse_srv_desc srv_desc;
	u8 iv[HSE_AEAD_MAX_IV_SIZE];
	dma_addr_t iv_dma;
	void *buf;
	dma_addr_t buf_dma;
	size_t buflen;
	enum hse_cipher_dir direction;
};

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
 * hse_aead_setauthsize - check authentication tag (MAC) size
 * @tfm: crypto aead transformation
 * @authsize: authentication tag size
 */
static int hse_aead_setauthsize(struct crypto_aead *tfm, unsigned int authsize)
{
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);

	switch (alg->auth_mode) {
	case HSE_AUTH_CIPHER_MODE_GCM:
		return crypto_gcm_check_authsize(authsize);
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * hse_aead_done - AEAD request done callback
 * @err: service response error code
 * @_req: AEAD request
 */
static void hse_aead_done(int err, void *_req)
{
	struct aead_request *req = (struct aead_request *)_req;
	struct hse_aead_req_ctx *rctx = aead_request_ctx(req);
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	unsigned int nbytes, dstlen, maclen = crypto_aead_authsize(tfm);

	dma_unmap_single(alg->dev, rctx->iv_dma, sizeof(rctx->iv),
			 DMA_TO_DEVICE);
	dma_unmap_single(alg->dev, rctx->buf_dma, rctx->buflen,
			 DMA_BIDIRECTIONAL);

	if (unlikely(err)) {
		dev_dbg(alg->dev, "%s: %s request failed: %d\n", __func__,
			crypto_tfm_alg_name(crypto_aead_tfm(tfm)), err);
		goto out_free_buf;
	}

	/* copy result from linear buffer */
	dstlen = req->assoclen + req->cryptlen +
		 (rctx->direction == HSE_CIPHER_DIR_ENCRYPT ? maclen : -maclen);
	nbytes = sg_copy_from_buffer(req->dst, sg_nents(req->dst), rctx->buf,
				     dstlen);
	if (unlikely(nbytes != dstlen))
		err = -ENODATA;

out_free_buf:
	kfree(rctx->buf);

	aead_request_complete(req, err);
}

/**
 * hse_aead_crypt - AEAD operation
 * @req: AEAD request
 * @direction: encrypt/decrypt selector
 */
static int hse_aead_crypt(struct aead_request *req,
			  enum hse_cipher_dir direction)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	struct hse_aead_req_ctx *rctx = aead_request_ctx(req);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	unsigned int maclen = crypto_aead_authsize(tfm);
	unsigned int ivsize = crypto_aead_ivsize(tfm);
	int err, nbytes, cryptlen = req->cryptlen;

	if (unlikely(!tctx->keylen))
		return -ENOKEY;

	/* for decrypt req->cryptlen includes the MAC length */
	cryptlen -= (direction == HSE_CIPHER_DIR_DECRYPT ? maclen : 0);
	rctx->buflen = req->assoclen + cryptlen + maclen;
	rctx->buf = kzalloc(rctx->buflen, GFP_KERNEL);
	if (IS_ERR_OR_NULL(rctx->buf)) {
		rctx->buflen = 0;
		return -ENOMEM;
	}

	/* copy source to linear buffer */
	nbytes = sg_copy_to_buffer(req->src, sg_nents(req->src),
				   rctx->buf, req->assoclen + req->cryptlen);
	if (nbytes != req->assoclen + req->cryptlen) {
		err = -ENODATA;
		goto err_free_buf;
	}

	rctx->buf_dma = dma_map_single(alg->dev, rctx->buf, rctx->buflen,
				       DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(alg->dev, rctx->buf_dma))) {
		err = -ENOMEM;
		goto err_free_buf;
	}

	/* copy IV to DMAable area */
	memcpy(rctx->iv, req->iv, ivsize);
	rctx->iv_dma = dma_map_single(alg->dev, rctx->iv, sizeof(rctx->iv),
				      DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->iv_dma))) {
		err = -ENOMEM;
		goto err_unmap_buf;
	}

	/* prepare HSE service descriptor */
	rctx->srv_desc.srv_id = HSE_SRV_ID_AEAD;
	rctx->srv_desc.aead_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
	rctx->srv_desc.aead_req.auth_cipher_mode = alg->auth_mode;
	rctx->srv_desc.aead_req.cipher_dir = direction;
	rctx->srv_desc.aead_req.key_handle = tctx->key_slot->handle;
	rctx->srv_desc.aead_req.iv_len = ivsize;
	rctx->srv_desc.aead_req.iv = rctx->iv_dma;
	rctx->srv_desc.aead_req.aad_len = req->assoclen;
	rctx->srv_desc.aead_req.aad = rctx->buf_dma;
	rctx->srv_desc.aead_req.sgt_opt = HSE_SGT_OPT_NONE;
	rctx->srv_desc.aead_req.input_len = cryptlen;
	rctx->srv_desc.aead_req.input = rctx->srv_desc.aead_req.aad +
					req->assoclen;
	rctx->srv_desc.aead_req.tag_len = maclen;
	rctx->srv_desc.aead_req.tag = rctx->srv_desc.aead_req.input + cryptlen;
	rctx->srv_desc.aead_req.output = rctx->srv_desc.aead_req.input;

	rctx->direction = direction;

	err = hse_srv_req_async(alg->dev, tctx->channel, &rctx->srv_desc, req,
				hse_aead_done);
	if (err)
		goto err_unmap_iv;

	return -EINPROGRESS;
err_unmap_iv:
	dma_unmap_single(alg->dev, rctx->iv_dma, sizeof(rctx->iv),
			 DMA_TO_DEVICE);
err_unmap_buf:
	dma_unmap_single(alg->dev, rctx->buf_dma, rctx->buflen,
			 DMA_BIDIRECTIONAL);
err_free_buf:
	kfree(rctx->buf);
	rctx->buflen = 0;
	return err;
}

static int hse_aead_encrypt(struct aead_request *req)
{
	return hse_aead_crypt(req, HSE_CIPHER_DIR_ENCRYPT);
}

static int hse_aead_decrypt(struct aead_request *req)
{
	return hse_aead_crypt(req, HSE_CIPHER_DIR_DECRYPT);
}

/**
 * hse_aead_setkey - AEAD setkey operation
 * @tfm: crypto aead transformation
 * @key: input key
 * @keylen: input key length, in bytes
 */
static int hse_aead_setkey(struct crypto_aead *tfm, const u8 *key,
			   unsigned int keylen)
{
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	int err;

	err = aes_check_keylen(keylen);
	if (err)
		return err;

	/* make sure key is located in a DMAable area */
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
	tctx->srv_desc.import_key_req.key_handle = tctx->key_slot->handle;
	tctx->srv_desc.import_key_req.key_info = tctx->keyinf_dma;
	tctx->srv_desc.import_key_req.sym.key = tctx->keybuf_dma;
	tctx->srv_desc.import_key_req.sym.keylen = keylen;
	tctx->srv_desc.import_key_req.cipher_key = HSE_INVALID_KEY_HANDLE;
	tctx->srv_desc.import_key_req.auth_key = HSE_INVALID_KEY_HANDLE;

	err = hse_srv_req_sync(alg->dev, HSE_CHANNEL_ANY, &tctx->srv_desc);
	if (unlikely(err))
		dev_dbg(alg->dev, "%s: setkey failed for %s: %d\n", __func__,
			crypto_tfm_alg_name(crypto_aead_tfm(tfm)), err);

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

	tctx->keyinf_dma = dma_map_single_attrs(alg->dev, &tctx->keyinf,
						sizeof(tctx->keyinf),
						DMA_TO_DEVICE,
						DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->keyinf_dma)))
		return -ENOMEM;

	tctx->keybuf_dma = dma_map_single_attrs(alg->dev, tctx->keybuf,
						sizeof(tctx->keybuf),
						DMA_TO_DEVICE,
						DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->keybuf_dma))) {
		err = -ENOMEM;
		goto err_unmap_keyinf;
	}

	tctx->keylen = 0;

	tctx->key_slot = hse_key_slot_acquire(alg->dev, alg->key_type);
	if (IS_ERR_OR_NULL(tctx->key_slot)) {
		dev_dbg(alg->dev, "%s: cannot acquire key slot\n", __func__);
		err = PTR_ERR(tctx->key_slot);
		goto err_unmap_keybuf;
	}

	err = hse_channel_acquire(alg->dev, HSE_CH_TYPE_SHARED, &tctx->channel,
				  NULL);
	if (unlikely(err))
		goto err_release_key_slot;

	return 0;
err_release_key_slot:
	hse_key_slot_release(alg->dev, tctx->key_slot);
err_unmap_keybuf:
	dma_unmap_single_attrs(alg->dev, tctx->keybuf_dma, sizeof(tctx->keybuf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_keyinf:
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
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

	hse_channel_release(alg->dev, tctx->channel);

	hse_key_slot_release(alg->dev, tctx->key_slot);

	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keybuf_dma, sizeof(tctx->keybuf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
}

static const struct hse_aead_tpl hse_aead_algs_tpl[] = {
	{
		.aead_name = "gcm(aes)",
		.aead_drv = "gcm-aes-hse",
		.blocksize = 1u,
		.ivsize = GCM_AES_IV_SIZE,
		.maxauthsize = AES_BLOCK_SIZE,
		.auth_mode = HSE_AUTH_CIPHER_MODE_GCM,
		.key_type = HSE_KEY_TYPE_AES,
		.alg_type = HSE_ALG_TYPE_AEAD,
	},
};

/**
 * hse_aead_alloc - allocate AEAD algorithm
 * @dev: HSE device
 * @tpl: AEAD algorithm template
 */
static struct hse_aead_alg *hse_aead_alloc(struct device *dev,
					   const struct hse_aead_tpl *tpl)
{
	struct hse_aead_alg *alg;
	struct crypto_alg *base;

	alg = devm_kzalloc(dev, sizeof(*alg), GFP_KERNEL);
	if (IS_ERR_OR_NULL(alg))
		return ERR_PTR(-ENOMEM);

	base = &alg->aead.base;

	alg->auth_mode = tpl->auth_mode;
	alg->key_type = tpl->key_type;
	alg->alg_type = tpl->alg_type;
	alg->dev = dev;

	alg->aead.init = hse_aead_init;
	alg->aead.exit = hse_aead_exit;
	alg->aead.setkey = hse_aead_setkey;

	switch (tpl->alg_type) {
	case HSE_ALG_TYPE_AEAD:
		alg->aead.encrypt = hse_aead_encrypt;
		alg->aead.decrypt = hse_aead_decrypt;
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	alg->aead.setauthsize = hse_aead_setauthsize;
	alg->aead.maxauthsize = tpl->maxauthsize;
	alg->aead.ivsize = tpl->ivsize;

	snprintf(base->cra_name, CRYPTO_MAX_ALG_NAME, "%s", tpl->aead_name);
	snprintf(base->cra_driver_name, CRYPTO_MAX_ALG_NAME, "%s",
		 tpl->aead_drv);

	base->cra_module = THIS_MODULE;
	base->cra_ctxsize = sizeof(struct hse_aead_tfm_ctx);
	base->cra_priority = HSE_CRA_PRIORITY;
	base->cra_blocksize = tpl->blocksize;
	base->cra_alignmask = 0u;
	base->cra_flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_KERN_DRIVER_ONLY;

	return alg;
}

/**
 * hse_aead_register - register AEAD algorithms
 * @dev: HSE device
 * @alg_list: list of registered algorithms
 */
void hse_aead_register(struct device *dev, struct list_head *alg_list)
{
	int i, err = 0;

	INIT_LIST_HEAD(alg_list);

	/* register crypto algorithms supported by device */
	for (i = 0; i < ARRAY_SIZE(hse_aead_algs_tpl); i++) {
		struct hse_aead_alg *alg;
		const struct hse_aead_tpl *tpl = &hse_aead_algs_tpl[i];

		alg = hse_aead_alloc(dev, tpl);
		if (IS_ERR(alg)) {
			dev_err(dev, "failed to allocate %s\n", tpl->aead_drv);
			continue;
		}

		err = crypto_register_aead(&alg->aead);
		if (unlikely(err)) {
			dev_err(dev, "failed to register alg %s: %d\n",
				tpl->aead_name, err);
		} else {
			list_add_tail(&alg->entry, alg_list);
			dev_info(dev, "registered alg %s\n", tpl->aead_name);
		}
	}
}

/**
 * hse_aead_unregister - unregister AEAD algorithms
 * @alg_list: list of registered algorithms
 */
void hse_aead_unregister(struct list_head *alg_list)
{
	struct hse_aead_alg *alg, *tmp;

	if (unlikely(!alg_list || !alg_list->next))
		return;

	list_for_each_entry_safe(alg, tmp, alg_list, entry) {
		crypto_unregister_aead(&alg->aead);
		list_del(&alg->entry);
	}
}
