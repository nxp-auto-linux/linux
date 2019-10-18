// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Symmetric Key Cipher Support
 *
 * This file contains the implementation of the symmetric key block cipher
 * algorithms that benefit from hardware acceleration via HSE.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/crypto.h>
#include <crypto/skcipher.h>
#include <crypto/internal/skcipher.h>
#include <crypto/aes.h>
#include <crypto/scatterwalk.h>

#include "hse.h"
#include "hse-abi.h"
#include "hse-mu.h"

#define  HSE_SKCIPHER_MAX_BLOCK_SIZE    AES_BLOCK_SIZE
#define  HSE_SKCIPHER_MAX_KEY_SIZE      AES_MAX_KEY_SIZE

/**
 * struct hse_skcipher_req_ctx - crypto request context
 * @srv_desc: service descriptor for skcipher ops
 * @srv_desc_dma: service descriptor DMA address
 * @iv: current initialization vector
 * @iv_dma: current initialization vector DMA address
 * @buf: linearized input/output buffer
 * @buf_dma: linearized input/output buffer DMA address
 * @direction: encrypt/decrypt selector
 */
struct hse_skcipher_req_ctx {
	struct hse_srv_desc srv_desc;
	dma_addr_t srv_desc_dma;
	u8 iv[HSE_SKCIPHER_MAX_BLOCK_SIZE];
	dma_addr_t iv_dma;
	void *buf;
	dma_addr_t buf_dma;
	u32 buflen;
	u8 direction;
} ____cacheline_aligned;

/**
 * struct hse_skcipher_tfm_ctx - crypto transformation context
 * @srv_desc: service descriptor for setkey ops
 * @srv_desc_dma: service descriptor DMA address
 * @key_slot: current key entry in cipher key ring
 * @keyinf: key information/flags, used for import
 * @keyinf_dma: key information/flags DMA address
 * @keybuf: buffer containing current key
 * @keybuf_dma: current key DMA address
 * @keylen: current key size
 */
struct hse_skcipher_tfm_ctx {
	struct hse_srv_desc srv_desc;
	dma_addr_t srv_desc_dma;
	struct hse_key *key_slot;
	struct hse_key_info keyinf;
	dma_addr_t keyinf_dma;
	u8 keybuf[HSE_SKCIPHER_MAX_KEY_SIZE];
	dma_addr_t keybuf_dma;
	unsigned int keylen;
} ____cacheline_aligned;

/**
 * hse_skcipher_alg - symmetric key cipher data
 * @skcipher: symmetric key cipher
 * @registered: algorithm registered/pending
 * @cipher_type: cipher algorithm/type
 * @block_mode: cipher block mode
 * @key_ring: available symmetric key slots
 * @dev: HSE device
 * @mu_inst: MU instance
 */
struct hse_skcipher_alg {
	struct skcipher_alg skcipher;
	bool registered;
	u8 cipher_type;
	u8 block_mode;
	struct list_head *key_ring;
	struct device *dev;
	void *mu_inst;
};

/**
 * hse_skcipher_get_alg - get cipher algorithm data from crypto transformation
 * @tfm: symmetric key cipher transformation
 *
 * Return: pointer to cipher algorithm data
 */
static inline void *hse_skcipher_get_alg(struct crypto_skcipher *tfm)
{
	struct skcipher_alg *alg = crypto_skcipher_alg(tfm);

	return container_of(alg, struct hse_skcipher_alg, skcipher);
}

/**
 * hse_skcipher_setkey - symmetric key cipher setkey operation
 * @tfm: crypto skcipher transformation
 * @key: input key
 * @keylen: input key size
 */
static int hse_skcipher_setkey(struct crypto_skcipher *tfm, const u8 *key,
			       unsigned int keylen)
{
	struct hse_skcipher_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_alg *alg = hse_skcipher_get_alg(tfm);
	int err;

	/* do not update the key if already imported */
	if (keylen == tctx->keylen &&
	    unlikely(!crypto_memneq(key, tctx->keybuf, keylen)))
		return 0;

	if (alg->cipher_type == HSE_CIPHER_ALGO_AES) {
		err = hse_check_aes_keylen(keylen);
		if (err) {
			crypto_skcipher_set_flags(tfm,
						  CRYPTO_TFM_RES_BAD_KEY_LEN);
			return err;
		}
	}

	memcpy(tctx->keybuf, key, keylen);
	dma_sync_single_for_device(alg->dev, tctx->keybuf_dma, keylen,
				   DMA_TO_DEVICE);
	tctx->keylen = keylen;

	tctx->keyinf.key_flags = HSE_KF_USAGE_ENCRYPT | HSE_KF_USAGE_DECRYPT;
	tctx->keyinf.key_bit_len = keylen * BITS_PER_BYTE;
	if (alg->cipher_type == HSE_CIPHER_ALGO_AES)
		tctx->keyinf.key_type = HSE_KEY_TYPE_AES;

	dma_sync_single_for_device(alg->dev, tctx->keyinf_dma,
				   sizeof(tctx->keyinf), DMA_TO_DEVICE);

	tctx->srv_desc.srv_id = HSE_SRV_ID_IMPORT_KEY;
	tctx->srv_desc.priority = HSE_SRV_PRIO_HIGH;

	tctx->srv_desc.import_key_req.key_handle = tctx->key_slot->handle;
	tctx->srv_desc.import_key_req.key_info = tctx->keyinf_dma;
	tctx->srv_desc.import_key_req.key = tctx->keybuf_dma;
	tctx->srv_desc.import_key_req.key_len = keylen;
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
 * hse_skcipher_done - symmetric key cipher rx callback
 * @mu_inst: MU instance
 * @channel: service channel index
 * @skreq: symmetric key cipher request
 *
 * Common rx callback for symmetric key cipher operations.
 */
static void hse_skcipher_done(void *mu_inst, u8 channel, void *skreq)
{
	struct skcipher_request *req = skreq;
	struct hse_skcipher_req_ctx *rctx = skcipher_request_ctx(req);
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct hse_skcipher_alg *alg = hse_skcipher_get_alg(tfm);
	int err, nbytes, ivsize = crypto_skcipher_ivsize(tfm);

	dma_unmap_single(alg->dev, rctx->srv_desc_dma, sizeof(rctx->srv_desc),
			 DMA_TO_DEVICE);
	dma_unmap_single(alg->dev, rctx->iv_dma, sizeof(rctx->iv),
			 DMA_TO_DEVICE);
	dma_unmap_single(alg->dev, rctx->buf_dma, rctx->buflen,
			 DMA_BIDIRECTIONAL);

	err = hse_mu_async_req_recv(mu_inst, channel);
	if (unlikely(err)) {
		dev_dbg(alg->dev, "%s: skcipher request failed: %d\n",
			__func__, err);
		goto out_free_buf;
	}

	nbytes = sg_copy_from_buffer(req->dst, sg_nents(req->dst),
				     rctx->buf, req->cryptlen);
	if (nbytes != req->cryptlen) {
		err = -ENODATA;
		goto out_free_buf;
	}

	/* req->iv is expected to be set to the last ciphertext block */
	if (rctx->direction == HSE_CIPHER_DIR_ENCRYPT &&
	    alg->cipher_type == HSE_CIPHER_ALGO_AES &&
	    alg->block_mode == HSE_CIPHER_BLOCK_MODE_CBC)
		scatterwalk_map_and_copy(req->iv, req->dst, req->cryptlen -
					 ivsize, ivsize, 0);

out_free_buf:
	kfree(rctx->buf);
	skcipher_request_complete(req, err);
}

/**
 * hse_skcipher_crypt - symmetric key cipher operation
 * @req: symmetric key cipher request
 * @direction: encrypt/decrypt
 */
static int hse_skcipher_crypt(struct skcipher_request *req, u8 direction)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct hse_skcipher_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_req_ctx *rctx = skcipher_request_ctx(req);
	struct hse_skcipher_alg *alg = hse_skcipher_get_alg(tfm);
	int err, nbytes, ivsize = crypto_skcipher_ivsize(tfm);

	if (unlikely(!tctx->keylen))
		return -ENOKEY;

	/* handle zero-length input because it's a valid operation */
	if (!req->cryptlen)
		return 0;

	rctx->buf = kzalloc(req->cryptlen, GFP_KERNEL);
	if (IS_ERR_OR_NULL(rctx->buf))
		return -ENOMEM;

	nbytes = sg_copy_to_buffer(req->src, sg_nents(req->src),
				   rctx->buf, req->cryptlen);
	if (nbytes != req->cryptlen) {
		err = -ENODATA;
		goto err_free_buf;
	}

	rctx->buf_dma = dma_map_single(alg->dev, rctx->buf, req->cryptlen,
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

	rctx->srv_desc.srv_id = HSE_SRV_ID_SYM_CIPHER;
	rctx->srv_desc.priority = HSE_SRV_PRIO_LOW;
	rctx->srv_desc.skcipher_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
	rctx->srv_desc.skcipher_req.cipher_algo = alg->cipher_type;
	rctx->srv_desc.skcipher_req.block_mode = alg->block_mode;
	rctx->srv_desc.skcipher_req.cipher_dir = direction;
	rctx->srv_desc.skcipher_req.key_handle = tctx->key_slot->handle;
	rctx->srv_desc.skcipher_req.iv_len = ivsize;
	rctx->srv_desc.skcipher_req.iv = rctx->iv_dma;
	rctx->srv_desc.skcipher_req.input_len = req->cryptlen;
	rctx->srv_desc.skcipher_req.input = rctx->buf_dma;
	rctx->srv_desc.skcipher_req.output = rctx->buf_dma;

	rctx->srv_desc_dma = dma_map_single(alg->dev, &rctx->srv_desc,
					    sizeof(rctx->srv_desc),
					    DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->srv_desc_dma))) {
		err = -ENOMEM;
		goto err_unmap_iv;
	}

	/* req->iv is expected to be set to the last ciphertext block */
	if (direction == HSE_CIPHER_DIR_DECRYPT &&
	    alg->cipher_type == HSE_CIPHER_ALGO_AES &&
	    alg->block_mode == HSE_CIPHER_BLOCK_MODE_CBC)
		scatterwalk_map_and_copy(req->iv, req->src, req->cryptlen -
					 ivsize, ivsize, 0);

	rctx->direction = direction;
	rctx->buflen = req->cryptlen;

	err = hse_mu_async_req_send(alg->mu_inst, HSE_ANY_CHANNEL,
				    lower_32_bits(rctx->srv_desc_dma), req,
				    hse_skcipher_done);
	if (err)
		goto err_unmap_srv_desc;

	return -EINPROGRESS;
err_unmap_srv_desc:
	dma_unmap_single(alg->dev, rctx->srv_desc_dma, sizeof(rctx->srv_desc),
			 DMA_TO_DEVICE);
err_unmap_iv:
	dma_unmap_single(alg->dev, rctx->iv_dma, sizeof(rctx->iv),
			 DMA_TO_DEVICE);
err_unmap_buf:
	dma_unmap_single(alg->dev, rctx->buf_dma, req->cryptlen,
			 DMA_BIDIRECTIONAL);
err_free_buf:
	kfree(rctx->buf);
	return err;
}

static int hse_skcipher_encrypt(struct skcipher_request *req)
{
	return hse_skcipher_crypt(req, HSE_CIPHER_DIR_ENCRYPT);
}

static int hse_skcipher_decrypt(struct skcipher_request *req)
{
	return hse_skcipher_crypt(req, HSE_CIPHER_DIR_DECRYPT);
}

/**
 * hse_skcipher_init - symmetric key cipher init
 * @tfm: symmetric key cipher transformation
 */
static int hse_skcipher_init(struct crypto_skcipher *tfm)
{
	struct hse_skcipher_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_alg *alg = hse_skcipher_get_alg(tfm);
	struct hse_drvdata *drv = dev_get_drvdata(alg->dev);
	int err;

	crypto_skcipher_set_reqsize(tfm, sizeof(struct hse_skcipher_req_ctx));

	/* remove key slot from ring */
	spin_lock(&drv->key_lock);
	tctx->key_slot = list_first_entry_or_null(alg->key_ring, struct hse_key,
						  entry);
	if (IS_ERR_OR_NULL(tctx->key_slot)) {
		dev_dbg(alg->dev, "%s: cannot acquire key slot\n", __func__);
		spin_unlock(&drv->key_lock);
		return -ENOKEY;
	}
	list_del(&tctx->key_slot->entry);
	spin_unlock(&drv->key_lock);

	tctx->srv_desc_dma = dma_map_single_attrs(alg->dev, &tctx->srv_desc,
						  sizeof(tctx->srv_desc),
						  DMA_TO_DEVICE,
						  DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->srv_desc_dma))) {
		err = -ENOMEM;
		goto err_release_key_slot;
	}

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

	tctx->keylen = 0;

	return 0;
err_unmap_keyinf:
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_srv_desc:
	dma_unmap_single_attrs(alg->dev, tctx->srv_desc_dma,
			       sizeof(tctx->srv_desc), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
err_release_key_slot:
	list_add_tail(&tctx->key_slot->entry, alg->key_ring);
	return err;
}

/**
 * hse_skcipher_exit - symmetric key cipher exit
 * @tfm: symmetric key cipher transformation
 */
static void hse_skcipher_exit(struct crypto_skcipher *tfm)
{
	struct hse_skcipher_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_alg *alg = hse_skcipher_get_alg(tfm);

	/* add key slot back to appropriate ring */
	list_add_tail(&tctx->key_slot->entry, alg->key_ring);

	dma_unmap_single_attrs(alg->dev, tctx->srv_desc_dma,
			       sizeof(tctx->srv_desc), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keybuf_dma, sizeof(tctx->keybuf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
}

static struct hse_skcipher_alg skcipher_algs[] = {
	{
		.skcipher = {
			.base = {
				.cra_name = "cbc(aes)",
				.cra_driver_name = "cbc-aes-hse",
				.cra_module = THIS_MODULE,
				.cra_priority = HSE_CRA_PRIORITY,
				.cra_flags = CRYPTO_ALG_ASYNC |
					     CRYPTO_ALG_TYPE_SKCIPHER,
				.cra_ctxsize =
					sizeof(struct hse_skcipher_tfm_ctx),
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = hse_skcipher_setkey,
			.encrypt = hse_skcipher_encrypt,
			.decrypt = hse_skcipher_decrypt,
			.init = hse_skcipher_init,
			.exit = hse_skcipher_exit,
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.ivsize = AES_BLOCK_SIZE,
		},
		.registered = false,
		.cipher_type = HSE_CIPHER_ALGO_AES,
		.block_mode = HSE_CIPHER_BLOCK_MODE_CBC,
	}
};

/**
 * hse_skcipher_register - register symmetric key algorithms
 * @dev: HSE device
 */
void hse_skcipher_register(struct device *dev)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	int i, err;

	/* register crypto algorithms the device supports */
	for (i = 0; i < ARRAY_SIZE(skcipher_algs); i++) {
		struct hse_skcipher_alg *alg = &skcipher_algs[i];
		const char *name = alg->skcipher.base.cra_name;

		alg->dev = dev;
		alg->mu_inst = drvdata->mu_inst;

		if (alg->cipher_type == HSE_CIPHER_ALGO_AES)
			alg->key_ring = &drvdata->aes_keys;
		else
			alg->key_ring = NULL;

		err = crypto_register_skcipher(&alg->skcipher);
		if (err) {
			dev_err(dev, "failed to register %s: %d", name, err);
		} else {
			alg->registered = true;
			dev_info(dev, "registered alg %s\n", name);
		}
	}
}

/**
 * hse_skcipher_unregister - unregister symmetric key algorithms
 */
void hse_skcipher_unregister(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(skcipher_algs); i++)
		if (skcipher_algs[i].registered)
			crypto_unregister_skcipher(&skcipher_algs[i].skcipher);
}
