// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - AEAD Support
 *
 * This file contains the implementation of the authenticated encryption with
 * additional data algorithms supported for hardware offloading via HSE.
 *
 * Copyright 2019-2020 NXP
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
 * hse_aead_setauthsize - set authentication tag (MAC) size
 * @tfm: crypto aead transformation
 * @authsize: auth tag size
 *
 * Validate authentication tag size for AEAD transformations.
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
 * @aeadreq: AEAD request
 */
static void hse_aead_done(int err, void *aeadreq)
{
	struct aead_request *req = (struct aead_request *)aeadreq;
	struct hse_aead_req_ctx *rctx = aead_request_ctx(req);
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	u32 maclen = crypto_aead_authsize(tfm);
	u32 ivlen = crypto_aead_ivsize(tfm);
	int nbytes, dstlen;

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
 * hse_aead_crypt - encrypt/decrypt AEAD operation
 * @req: AEAD encrypt/decrypt request
 * @encrypt: whether the request is encrypt or not
 */
static int hse_aead_crypt(struct aead_request *req, bool encrypt)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	struct hse_aead_req_ctx *rctx = aead_request_ctx(req);
	unsigned int cryptlen, srclen, ivlen = crypto_aead_ivsize(tfm);
	unsigned int maclen = crypto_aead_authsize(tfm);
	int err, nbytes;

	/* make sure IV is located in a DMAable area before DMA mapping it */
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
	rctx->desc.aead_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
	rctx->desc.aead_req.auth_cipher_mode = alg->auth_mode;
	rctx->desc.aead_req.cipher_dir =
		encrypt ? HSE_CIPHER_DIR_ENCRYPT : HSE_CIPHER_DIR_DECRYPT;
	rctx->desc.aead_req.key_handle = tctx->key_slot->handle;
	rctx->desc.aead_req.iv_len = ivlen;
	rctx->desc.aead_req.iv = rctx->iv_dma;
	rctx->desc.aead_req.aad_len = req->assoclen;
	rctx->desc.aead_req.aad = rctx->buf_dma;
	rctx->desc.aead_req.sgt_opt = HSE_SGT_OPT_NONE;
	rctx->desc.aead_req.input_len = cryptlen;
	rctx->desc.aead_req.input = rctx->desc.aead_req.aad + req->assoclen;
	rctx->desc.aead_req.tag_len = maclen;
	rctx->desc.aead_req.tag = rctx->desc.aead_req.input + cryptlen;
	rctx->desc.aead_req.output = rctx->desc.aead_req.input;

	/* DMA map service descriptor */
	rctx->desc_dma = dma_map_single(alg->dev, &rctx->desc,
					sizeof(rctx->desc), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->desc_dma))) {
		dev_err(alg->dev, "unable to map HSE service descriptor\n");
		err = -ENOMEM;
		goto err_unmap_buf;
	}

	rctx->encrypt = encrypt;

	err = hse_srv_req_async(alg->dev, tctx->channel, rctx->desc_dma,
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

static int hse_aead_encrypt(struct aead_request *req)
{
	return hse_aead_crypt(req, true);
}

static int hse_aead_decrypt(struct aead_request *req)
{
	return hse_aead_crypt(req, false);
}

/**
 * hse_aead_setkey - AEAD setkey operation
 * @tfm: crypto aead transformation
 * @key: input key
 * @keylen: input key length, in bytes
 *
 * Key buffers and information must be located in the 32-bit address range.
 */
static int hse_aead_setkey(struct crypto_aead *tfm, const u8 *key,
			   unsigned int keylen)
{
	struct hse_aead_tfm_ctx *tctx = crypto_aead_ctx(tfm);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	int err;

	if (alg->alg_type == HSE_ALG_TYPE_KEYWRAP)
		return 0;

	err = aes_check_keylen(keylen);
	if (err) {
		crypto_aead_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return err;
	}

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

	dma_sync_single_for_device(alg->dev, tctx->srv_desc_dma,
				   sizeof(tctx->srv_desc), DMA_TO_DEVICE);

	err = hse_srv_req_sync(alg->dev, HSE_CHANNEL_ANY, tctx->srv_desc_dma);
	if (unlikely(err))
		dev_dbg(alg->dev, "%s: cipher key import request failed: %d\n",
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

	tctx->srv_desc_dma = dma_map_single_attrs(alg->dev, &tctx->srv_desc,
						  sizeof(tctx->srv_desc),
						  DMA_TO_DEVICE,
						  DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->srv_desc_dma)))
		return -ENOMEM;

	tctx->keyinf_dma = dma_map_single_attrs(alg->dev, &tctx->keyinf,
						sizeof(tctx->keyinf),
						DMA_TO_DEVICE,
						DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->keyinf_dma))) {
		err = -ENOMEM;
		goto err_unmap_srv_desc;
	}

	tctx->keybuf_dma = dma_map_single_attrs(alg->dev, tctx->keybuf,
						sizeof(tctx->keybuf),
						DMA_TO_DEVICE,
						DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->keybuf_dma))) {
		err = -ENOMEM;
		goto err_unmap_keyinf;
	}

	tctx->keylen = alg->alg_type == HSE_ALG_TYPE_KEYWRAP ? 16 : 0;

	if (alg->alg_type == HSE_ALG_TYPE_KEYWRAP)
		return 0;

	tctx->key_slot = hse_key_slot_acquire(alg->dev, alg->key_type);
	if (IS_ERR_OR_NULL(tctx->key_slot)) {
		dev_dbg(alg->dev, "%s: cannot acquire key slot\n", __func__);
		err = PTR_ERR(tctx->key_slot);
		goto err_unmap_keybuf;
	}

	err = hse_channel_acquire(alg->dev, HSE_CH_TYPE_SHARED, &tctx->channel);
	if (unlikely(err))
		goto err_release_key_slot;

	return 0;
err_release_key_slot:
	hse_key_slot_release(alg->dev, tctx->key_slot);
err_unmap_keybuf:
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_keyinf:
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_srv_desc:
	dma_unmap_single_attrs(alg->dev, tctx->srv_desc_dma,
			       sizeof(tctx->srv_desc), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
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

	if (alg->alg_type != HSE_ALG_TYPE_KEYWRAP) {
		hse_channel_release(alg->dev, tctx->channel);

		hse_key_slot_release(alg->dev, tctx->key_slot);
	}

	dma_unmap_single_attrs(alg->dev, tctx->srv_desc_dma,
			       sizeof(tctx->srv_desc), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keybuf_dma, sizeof(tctx->keybuf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
}

/**
 * hse_key_wrap_done - AEAD request done callback
 * @err: service response error code
 * @aeadreq: AEAD request
 */
static void hse_key_wrap_done(int err, void *aeadreq)
{
	struct aead_request *req = (struct aead_request *)aeadreq;
	struct hse_aead_req_ctx *rctx = aead_request_ctx(req);
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	u32 maclen = crypto_aead_authsize(tfm);
	u32 ivlen = crypto_aead_ivsize(tfm);
	int nbytes, dstlen;

	if (unlikely(err)) {
		dev_dbg(alg->dev, "%s: key wrap/unwrap request failed: %d\n",
			__func__, err);
		goto out;
	}

	dma_unmap_single(alg->dev, rctx->buf_dma, rctx->buflen,
			 DMA_BIDIRECTIONAL);

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
 * hse_key_wrap_unwrap - key wrapping/unwrapping operation
 * @req: AEAD encrypt/decrypt request
 * @encrypt: whether the request is wrapping or unwrapping
 */
static int hse_key_wrap_unwrap(struct aead_request *req, bool wrap)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct hse_aead_alg *alg = hse_aead_get_alg(tfm);
	struct hse_aead_req_ctx *rctx = aead_request_ctx(req);
	unsigned int cryptlen, srclen, ivlen = crypto_aead_ivsize(tfm);
	unsigned int maclen = crypto_aead_authsize(tfm);
	int err, nbytes;

	/* make sure IV is located in a DMAable area before DMA mapping it */
	memcpy(&rctx->iv, req->iv, ivlen);

	/* for decrypt req->cryptlen includes the MAC length */
	cryptlen = req->cryptlen - (!wrap ? maclen : 0);

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

	rctx->desc.srv_id = HSE_SRV_ID_AEAD;
	rctx->desc.aead_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
	rctx->desc.aead_req.auth_cipher_mode = alg->auth_mode;
	rctx->desc.aead_req.cipher_dir =
		wrap ? HSE_CIPHER_DIR_ENCRYPT : HSE_CIPHER_DIR_DECRYPT;
	rctx->desc.aead_req.key_handle = HSE_ROM_KEY_AES256_KEY0;
	rctx->desc.aead_req.iv_len = ivlen;
	rctx->desc.aead_req.iv = rctx->iv_dma;
	rctx->desc.aead_req.aad_len = req->assoclen;
	rctx->desc.aead_req.aad = rctx->buf_dma;
	rctx->desc.aead_req.sgt_opt = HSE_SGT_OPT_NONE;
	rctx->desc.aead_req.input_len = cryptlen;
	rctx->desc.aead_req.input = rctx->desc.aead_req.aad + req->assoclen;
	rctx->desc.aead_req.tag_len = maclen;
	rctx->desc.aead_req.tag = rctx->desc.aead_req.input + cryptlen;
	rctx->desc.aead_req.output = rctx->desc.aead_req.input;

	rctx->desc_dma = dma_map_single(alg->dev, &rctx->desc,
					sizeof(rctx->desc), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->desc_dma))) {
		dev_err(alg->dev, "unable to map HSE service descriptor\n");
		err = -ENOMEM;
		goto err_unmap_buf;
	}

	rctx->encrypt = wrap;

	err = hse_srv_req_async(alg->dev, HSE_CHANNEL_ANY, rctx->desc_dma,
				req, hse_key_wrap_done);
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

static int hse_key_wrap(struct aead_request *req)
{
	return hse_key_wrap_unwrap(req, true);
}

static int hse_key_unwrap(struct aead_request *req)
{
	return hse_key_wrap_unwrap(req, false);
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
#if defined(CRYPTO_DEV_NXP_HSE_KEY_WRAPPING)
	{
		.aead_name = "wrap(key)",
		.aead_drv = "wrap-key-hse",
		.blocksize = 1u,
		.ivsize = GCM_AES_IV_SIZE,
		.maxauthsize = AES_BLOCK_SIZE,
		.auth_mode = HSE_AUTH_CIPHER_MODE_GCM,
		.key_type = HSE_KEY_TYPE_AES,
		.alg_type = HSE_ALG_TYPE_KEYWRAP,
	},
#endif
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
	case HSE_ALG_TYPE_KEYWRAP:
		alg->aead.encrypt = hse_key_wrap;
		alg->aead.decrypt = hse_key_unwrap;
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

	if (unlikely(!alg_list->next))
		return;

	list_for_each_entry_safe(alg, tmp, alg_list, entry) {
		crypto_unregister_aead(&alg->aead);
		list_del(&alg->entry);
	}
}
