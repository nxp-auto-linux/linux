// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Symmetric Key Cipher Support
 *
 * This file contains the implementation of the symmetric key block cipher
 * algorithms supported for hardware offloading via HSE.
 *
 * Copyright 2019-2022 NXP
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/crypto.h>
#include <crypto/skcipher.h>
#include <crypto/internal/skcipher.h>
#include <crypto/aes.h>
#include <crypto/scatterwalk.h>

#include "hse-abi.h"
#include "hse-core.h"

#define HSE_SKCIPHER_MAX_BLOCK_SIZE    AES_BLOCK_SIZE
#define HSE_SKCIPHER_MAX_KEY_SIZE      AES_MAX_KEY_SIZE

/**
 * struct hse_skcipher_tpl - algorithm template
 * @cipher_name: cipher algorithm name
 * @cipher_drv: cipher driver name
 * @blocksize: block size
 * @min_keysize: minimum key size
 * @max_keysize: maximum key size
 * @ivsize: initialization vector size
 * @cipher_type: cipher algorithm/type
 * @block_mode: cipher block mode
 * @key_type: type of key used
 */
struct hse_skcipher_tpl {
	char cipher_name[CRYPTO_MAX_ALG_NAME];
	char cipher_drv[CRYPTO_MAX_ALG_NAME];
	unsigned int blocksize;
	unsigned int min_keysize;
	unsigned int max_keysize;
	unsigned int ivsize;
	enum hse_cipher_algorithm cipher_type;
	enum hse_block_mode block_mode;
	enum hse_key_type key_type;
};

/**
 * hse_skcipher_alg - algorithm private data
 * @skcipher: symmetric key cipher
 * @entry: position in supported algorithms list
 * @cipher_type: cipher algorithm/type
 * @block_mode: cipher block mode
 * @key_type: type of key used
 * @dev: HSE device
 */
struct hse_skcipher_alg {
	struct skcipher_alg skcipher;
	struct list_head entry;
	enum hse_cipher_algorithm cipher_type;
	enum hse_block_mode block_mode;
	enum hse_key_type key_type;
	struct device *dev;
};

/**
 * struct hse_skcipher_tfm_ctx - crypto transformation context
 * @srv_desc: service descriptor for setkey ops
 * @key_slot: current key entry in cipher key ring
 * @keyinf: key information/flags, used for import
 * @keyinf_dma: key information/flags DMA address
 * @keybuf: buffer containing current key
 * @keybuf_dma: current key DMA address
 * @keylen: size of the last imported key
 */
struct hse_skcipher_tfm_ctx {
	struct hse_srv_desc srv_desc;
	struct hse_key *key_slot;
	struct hse_key_info keyinf;
	dma_addr_t keyinf_dma;
	u8 keybuf[HSE_SKCIPHER_MAX_KEY_SIZE];
	dma_addr_t keybuf_dma;
	unsigned int keylen;
};

/**
 * struct hse_skcipher_req_ctx - crypto request context
 * @srv_desc: service descriptor for skcipher ops
 * @iv: current initialization vector
 * @iv_dma: current initialization vector DMA address
 * @buf: linearized input/output buffer
 * @buf_dma: linearized input/output buffer DMA address
 * @buflen: size of current linearized input buffer
 * @direction: encrypt/decrypt selector
 */
struct hse_skcipher_req_ctx {
	struct hse_srv_desc srv_desc;
	u8 iv[HSE_SKCIPHER_MAX_BLOCK_SIZE];
	dma_addr_t iv_dma;
	void *buf;
	dma_addr_t buf_dma;
	size_t buflen;
	enum hse_cipher_dir direction;
};

/**
 * hse_skcipher_get_alg - get cipher algorithm data from crypto transformation
 * @tfm: crypto skcipher transformation
 */
static inline void *hse_skcipher_get_alg(struct crypto_skcipher *tfm)
{
	struct skcipher_alg *alg = crypto_skcipher_alg(tfm);

	return container_of(alg, struct hse_skcipher_alg, skcipher);
}

/**
 * hse_skcipher_done - symmetric key cipher request done callback
 * @err: service response error code
 * @_req: symmetric key cipher request
 */
static void hse_skcipher_done(int err, void *_req)
{
	struct skcipher_request *req = (struct skcipher_request *)_req;
	struct hse_skcipher_req_ctx *rctx = skcipher_request_ctx(req);
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct hse_skcipher_alg *alg = hse_skcipher_get_alg(tfm);
	unsigned int i, blocksize = crypto_skcipher_blocksize(tfm);
	unsigned int nbytes, ivsize = crypto_skcipher_ivsize(tfm);

	dma_unmap_single(alg->dev, rctx->iv_dma, sizeof(rctx->iv),
			 DMA_TO_DEVICE);
	dma_unmap_single(alg->dev, rctx->buf_dma, rctx->buflen,
			 DMA_BIDIRECTIONAL);

	if (unlikely(err)) {
		dev_dbg(alg->dev, "%s: %s request failed: %d\n", __func__,
			crypto_tfm_alg_name(crypto_skcipher_tfm(tfm)), err);
		goto out_free_buf;
	}

	/* copy result from linear buffer */
	nbytes = sg_copy_from_buffer(req->dst, sg_nents(req->dst), rctx->buf,
				     req->cryptlen);
	if (unlikely(nbytes != req->cryptlen)) {
		err = -ENODATA;
		goto out_free_buf;
	}

	switch (alg->block_mode) {
	case HSE_CIPHER_BLOCK_MODE_CTR:
		/* increment req_iv counter by the number of encrypted blocks */
		for (i = 0; i < rctx->buflen / blocksize; i++)
			crypto_inc(req->iv, ivsize);
		break;
	case HSE_CIPHER_BLOCK_MODE_CBC:
		/* req->iv is expected to be set to the last ciphertext block */
		if (rctx->direction == HSE_CIPHER_DIR_ENCRYPT)
			scatterwalk_map_and_copy(req->iv, req->dst,
						 req->cryptlen - ivsize,
						 ivsize, 0);
		break;
	default:
		break;
	}

out_free_buf:
	kfree(rctx->buf);

	skcipher_request_complete(req, err);
}

/**
 * hse_skcipher_crypt - symmetric key cipher operation
 * @req: symmetric key cipher request
 * @direction: encrypt/decrypt selector
 */
static int hse_skcipher_crypt(struct skcipher_request *req,
			      enum hse_cipher_dir direction)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct hse_skcipher_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_req_ctx *rctx = skcipher_request_ctx(req);
	struct hse_skcipher_alg *alg = hse_skcipher_get_alg(tfm);
	unsigned int blocksize = crypto_skcipher_blocksize(tfm);
	int err, nbytes, ivsize = crypto_skcipher_ivsize(tfm);

	if (unlikely(!tctx->keylen))
		return -ENOKEY;

	/* exit if no data */
	if (!req->cryptlen)
		return 0;

	rctx->buflen = roundup(req->cryptlen, blocksize);
	rctx->buf = kzalloc(rctx->buflen, GFP_KERNEL);
	if (IS_ERR_OR_NULL(rctx->buf)) {
		rctx->buflen = 0;
		return -ENOMEM;
	}

	/* copy source to linear buffer */
	nbytes = sg_copy_to_buffer(req->src, sg_nents(req->src),
				   rctx->buf, req->cryptlen);
	if (nbytes != req->cryptlen) {
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

	rctx->srv_desc.srv_id = HSE_SRV_ID_SYM_CIPHER;
	rctx->srv_desc.skcipher_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
	rctx->srv_desc.skcipher_req.cipher_algo = alg->cipher_type;
	rctx->srv_desc.skcipher_req.block_mode = alg->block_mode;
	rctx->srv_desc.skcipher_req.cipher_dir = direction;
	rctx->srv_desc.skcipher_req.key_handle = tctx->key_slot->handle;
	rctx->srv_desc.skcipher_req.iv = rctx->iv_dma;
	rctx->srv_desc.skcipher_req.sgt_opt = HSE_SGT_OPT_NONE;
	rctx->srv_desc.skcipher_req.input_len = rctx->buflen;
	rctx->srv_desc.skcipher_req.input = rctx->buf_dma;
	rctx->srv_desc.skcipher_req.output = rctx->buf_dma;

	/* req->iv is expected to be set to the last ciphertext block */
	if (alg->block_mode == HSE_CIPHER_BLOCK_MODE_CBC &&
	    direction == HSE_CIPHER_DIR_DECRYPT)
		scatterwalk_map_and_copy(req->iv, req->src, req->cryptlen -
					 ivsize, ivsize, 0);

	rctx->direction = direction;

	err = hse_srv_req_async(alg->dev, HSE_CHANNEL_ANY, &rctx->srv_desc, req,
				hse_skcipher_done);
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

static int hse_skcipher_encrypt(struct skcipher_request *req)
{
	return hse_skcipher_crypt(req, HSE_CIPHER_DIR_ENCRYPT);
}

static int hse_skcipher_decrypt(struct skcipher_request *req)
{
	return hse_skcipher_crypt(req, HSE_CIPHER_DIR_DECRYPT);
}

/**
 * hse_skcipher_setkey - symmetric key cipher setkey operation
 * @tfm: crypto skcipher transformation
 * @key: input key
 * @keylen: input key length, in bytes
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
		err = aes_check_keylen(keylen);
		if (err)
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

	err = hse_srv_req_sync(alg->dev, HSE_CHANNEL_ANY, &tctx->srv_desc);
	if (unlikely(err))
		dev_dbg(alg->dev, "%s: setkey failed for %s: %d\n", __func__,
			crypto_tfm_alg_name(crypto_skcipher_tfm(tfm)), err);

	return err;
}

/**
 * hse_skcipher_init - crypto transformation init
 * @tfm: crypto skcipher transformation
 */
static int hse_skcipher_init(struct crypto_skcipher *tfm)
{
	struct hse_skcipher_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_alg *alg = hse_skcipher_get_alg(tfm);
	int err;

	crypto_skcipher_set_reqsize(tfm, sizeof(struct hse_skcipher_req_ctx));

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

	return 0;
err_unmap_keybuf:
	dma_unmap_single_attrs(alg->dev, tctx->keybuf_dma, sizeof(tctx->keybuf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_keyinf:
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
	return err;
}

/**
 * hse_skcipher_exit - crypto transformation exit
 * @tfm: crypto skcipher transformation
 */
static void hse_skcipher_exit(struct crypto_skcipher *tfm)
{
	struct hse_skcipher_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_alg *alg = hse_skcipher_get_alg(tfm);

	hse_key_slot_release(alg->dev, tctx->key_slot);

	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keybuf_dma, sizeof(tctx->keybuf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
}

static const struct hse_skcipher_tpl hse_skcipher_algs_tpl[] = {
	{
		.cipher_name = "ctr(aes)",
		.cipher_drv = "ctr-aes-hse",
		.blocksize = AES_BLOCK_SIZE,
		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.ivsize = AES_BLOCK_SIZE,
		.cipher_type = HSE_CIPHER_ALGO_AES,
		.block_mode = HSE_CIPHER_BLOCK_MODE_CTR,
		.key_type = HSE_KEY_TYPE_AES,
	}, {
		.cipher_name = "cbc(aes)",
		.cipher_drv = "cbc-aes-hse",
		.blocksize = AES_BLOCK_SIZE,
		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.ivsize = AES_BLOCK_SIZE,
		.cipher_type = HSE_CIPHER_ALGO_AES,
		.block_mode = HSE_CIPHER_BLOCK_MODE_CBC,
		.key_type = HSE_KEY_TYPE_AES,
	}, {
		.cipher_name = "ecb(aes)",
		.cipher_drv = "ecb-aes-hse",
		.blocksize = AES_BLOCK_SIZE,
		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.ivsize = 0u,
		.cipher_type = HSE_CIPHER_ALGO_AES,
		.block_mode = HSE_CIPHER_BLOCK_MODE_ECB,
		.key_type = HSE_KEY_TYPE_AES,
	}, {
		.cipher_name = "cfb(aes)",
		.cipher_drv = "cfb-aes-hse",
		.blocksize = AES_BLOCK_SIZE,
		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.ivsize = AES_BLOCK_SIZE,
		.cipher_type = HSE_CIPHER_ALGO_AES,
		.block_mode = HSE_CIPHER_BLOCK_MODE_CFB,
		.key_type = HSE_KEY_TYPE_AES,
	}, {
		.cipher_name = "ofb(aes)",
		.cipher_drv = "ofb-aes-hse",
		.blocksize = AES_BLOCK_SIZE,
		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.ivsize = AES_BLOCK_SIZE,
		.cipher_type = HSE_CIPHER_ALGO_AES,
		.block_mode = HSE_CIPHER_BLOCK_MODE_OFB,
		.key_type = HSE_KEY_TYPE_AES,
	},
};

/**
 * hse_skcipher_alloc - allocate cipher algorithm
 * @dev: HSE device
 * @tpl: cipher algorithm template
 */
static struct hse_skcipher_alg *
hse_skcipher_alloc(struct device *dev, const struct hse_skcipher_tpl *tpl)
{
	struct hse_skcipher_alg *alg;
	struct crypto_alg *base;

	alg = devm_kzalloc(dev, sizeof(*alg), GFP_KERNEL);
	if (IS_ERR_OR_NULL(alg))
		return ERR_PTR(-ENOMEM);

	base = &alg->skcipher.base;

	alg->cipher_type = tpl->cipher_type;
	alg->block_mode = tpl->block_mode;
	alg->key_type = tpl->key_type;
	alg->dev = dev;

	alg->skcipher.init = hse_skcipher_init;
	alg->skcipher.exit = hse_skcipher_exit;
	alg->skcipher.setkey = hse_skcipher_setkey;
	alg->skcipher.encrypt = hse_skcipher_encrypt;
	alg->skcipher.decrypt = hse_skcipher_decrypt;
	alg->skcipher.min_keysize = tpl->min_keysize;
	alg->skcipher.max_keysize = tpl->max_keysize;
	alg->skcipher.ivsize = tpl->ivsize;

	snprintf(base->cra_name, CRYPTO_MAX_ALG_NAME, "%s", tpl->cipher_name);
	snprintf(base->cra_driver_name, CRYPTO_MAX_ALG_NAME, "%s",
		 tpl->cipher_drv);

	base->cra_module = THIS_MODULE;
	base->cra_ctxsize = sizeof(struct hse_skcipher_tfm_ctx);
	base->cra_priority = HSE_CRA_PRIORITY;
	base->cra_blocksize = tpl->blocksize;
	base->cra_alignmask = 0u;
	base->cra_flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_KERN_DRIVER_ONLY;

	return alg;
}

/**
 * hse_skcipher_register - register symmetric key algorithms
 * @dev: HSE device
 * @alg_list: list of registered algorithms
 */
void hse_skcipher_register(struct device *dev, struct list_head *alg_list)
{
	int i, err = 0;

	INIT_LIST_HEAD(alg_list);

	/* register crypto algorithms supported by device */
	for (i = 0; i < ARRAY_SIZE(hse_skcipher_algs_tpl); i++) {
		struct hse_skcipher_alg *alg;
		const struct hse_skcipher_tpl *tpl = &hse_skcipher_algs_tpl[i];

		alg = hse_skcipher_alloc(dev, tpl);
		if (IS_ERR(alg)) {
			dev_err(dev, "failed to allocate %s\n",
				tpl->cipher_drv);
			continue;
		}

		err = crypto_register_skcipher(&alg->skcipher);
		if (unlikely(err)) {
			dev_err(dev, "failed to register alg %s: %d\n",
				tpl->cipher_name, err);
		} else {
			list_add_tail(&alg->entry, alg_list);
			dev_info(dev, "registered alg %s\n", tpl->cipher_name);
		}
	}
}

/**
 * hse_skcipher_unregister - unregister symmetric key algorithms
 * @alg_list: list of registered algorithms
 */
void hse_skcipher_unregister(struct list_head *alg_list)
{
	struct hse_skcipher_alg *alg, *tmp;

	if (unlikely(!alg_list->next))
		return;

	list_for_each_entry_safe(alg, tmp, alg_list, entry) {
		crypto_unregister_skcipher(&alg->skcipher);
		list_del(&alg->entry);
	}
}
