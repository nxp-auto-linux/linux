// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Symmetric Key Cipher Support
 *
 * This file contains the kernel crypto API implementation for the
 * symmetric key block cipher algorithms supported by HSE.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/crypto.h>
#include <crypto/skcipher.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/skcipher.h>

#include "hse.h"
#include "hse-abi.h"
#include "hse-mu.h"

/**
 * struct hse_skcipher_ctx - skcipher request context
 * @srv_desc: HSE service descriptor
 * @iv: initialization vector
 * @direction: encrypt/decrypt selector
 * @buf: linearized buffer
 */
struct hse_skcipher_ctx {
	struct hse_srv_desc srv_desc;
	u8 iv[AES_BLOCK_SIZE];
	u8 direction;
	void *buf;
};

/**
 * struct hse_skcipher_state - crypto transformation state/context
 * @key_srv_desc: HSE service descriptor, used by setkey
 * @key_info: key flags, used for import
 * @crt_key: key currently in use
 * @keybuf: buffer containing the current key
 */
struct hse_skcipher_state {
	struct hse_srv_desc key_srv_desc;
	struct hse_key_info key_info;
	struct hse_key *crt_key;
	u8 keybuf[AES_MAX_KEY_SIZE];
};

/**
 * hse_skcipher_alg - symmetric key cipher data
 * @skcipher: symmetric key cipher
 * @registered: tfm registered/pending
 * @cipher_type: HSE cipher type
 * @block_mode: cipher block mode
 * @keys_list: available symmetric key slots
 * @dev: HSE device
 * @mu_inst: MU instance
 */
struct hse_skcipher_alg {
	struct skcipher_alg skcipher;
	bool registered;
	u8 cipher_type;
	u8 block_mode;
	struct list_head *keys_list;
	struct device *dev;
	void *mu_inst;
};

/**
 * hse_get_skcipher_alg - get cipher algorithm data from crypto transformation
 * @tfm: symmetric key cipher transformation
 *
 * Return: pointer to cipher algorithm data
 */
static struct hse_skcipher_alg *hse_get_skcipher(struct crypto_skcipher *tfm)
{
	struct crypto_alg *base = tfm->base.__crt_alg;
	struct skcipher_alg *alg = container_of(base, struct skcipher_alg,
						base);
	struct hse_skcipher_alg *hsealg = container_of(alg,
						      struct hse_skcipher_alg,
						      skcipher);

	return hsealg;
}

/**
 * hse_skcipher_setkey - symmetric key cipher setkey operation
 * @skcipher: symmetric key cipher
 * @key: input key
 * @keylen: input key size
 */
static int hse_skcipher_setkey(struct crypto_skcipher *skcipher, const u8 *key,
			       unsigned int keylen)
{
	struct hse_skcipher_state *state = crypto_skcipher_ctx(skcipher);
	struct hse_skcipher_alg *hsealg = hse_get_skcipher(skcipher);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&state->key_srv_desc));
	int err;

	if (hsealg->cipher_type == HSE_CIPHER_ALGO_AES) {
		err = hse_check_aes_keylen(keylen);
		if (err) {
			crypto_skcipher_set_flags(skcipher,
						  CRYPTO_TFM_RES_BAD_KEY_LEN);
			return err;
		}
	}

	memcpy(&state->keybuf, key, keylen);

	state->key_info.key_flags = HSE_KF_MU_INST | HSE_KF_USAGE_ENCRYPT |
			      HSE_KF_USAGE_DECRYPT;
	state->key_info.key_bit_len = keylen << 3;

	if (hsealg->cipher_type == HSE_CIPHER_ALGO_AES)
		state->key_info.key_type = HSE_KEY_TYPE_AES;

	state->key_srv_desc.srv_id = HSE_SRV_ID_IMPORT_KEY;
	state->key_srv_desc.priority = HSE_SRV_PRIO_HIGH;

	state->key_srv_desc.import_key_req.key_handle =
		state->crt_key->handle;
	state->key_srv_desc.import_key_req.key_info =
		hse_addr(&state->key_info);
	state->key_srv_desc.import_key_req.key = hse_addr(&state->keybuf);
	state->key_srv_desc.import_key_req.key_len = keylen;
	state->key_srv_desc.import_key_req.cipher_key = HSE_INVALID_KEY_HANDLE;
	state->key_srv_desc.import_key_req.auth_key = HSE_INVALID_KEY_HANDLE;

	err = hse_mu_sync_req(hsealg->mu_inst, HSE_ANY_CHANNEL, srv_desc_addr);
	if (unlikely(err))
		dev_dbg(hsealg->dev, "%s: key import request failed: %d\n",
			__func__, err);

	return err;
}

/**
 * hse_skcipher_done - symmetric key cipher rx callback
 * @mu_inst: MU instance
 * @channel: service channel index
 * @skreq: symmetric key cipher request
 *
 * Common RX callback for symmetric key cipher operations.
 */
static void hse_skcipher_done(void *mu_inst, u8 channel, void *skreq)
{
	struct skcipher_request *req = skreq;
	struct hse_skcipher_ctx *ctx = skcipher_request_ctx(req);
	struct crypto_skcipher *skcipher = crypto_skcipher_reqtfm(req);
	struct hse_skcipher_alg *hsealg = hse_get_skcipher(skcipher);
	int err, nbytes, ivsize = crypto_skcipher_ivsize(skcipher);

	err = hse_mu_async_req_recv(mu_inst, channel);
	if (unlikely(err)) {
		dev_dbg(hsealg->dev, "%s: skcipher request failed: %d\n",
			__func__, err);
		goto out;
	}

	nbytes = sg_copy_from_buffer(req->dst, sg_nents(req->dst),
				     ctx->buf, req->cryptlen);
	if (nbytes != req->cryptlen) {
		err = -ENODATA;
		goto out;
	}

	/* req->iv is expected to be set to the last ciphertext block */
	if (ctx->direction == HSE_CIPHER_DIR_ENCRYPT &&
	    hsealg->cipher_type == HSE_CIPHER_ALGO_AES &&
	    hsealg->block_mode == HSE_CIPHER_BLOCK_MODE_CBC)
		scatterwalk_map_and_copy(req->iv, req->dst, req->cryptlen -
					 ivsize, ivsize, 0);

out:
	kfree(ctx->buf);
	skcipher_request_complete(req, err);
}

/**
 * hse_skcipher_crypt - symmetric key cipher operation
 * @req: symmetric key cipher request
 * @direction: encrypt/decrypt
 */
static int hse_skcipher_crypt(struct skcipher_request *req, u8 direction)
{
	struct crypto_skcipher *skcipher = crypto_skcipher_reqtfm(req);
	struct hse_skcipher_state *state = crypto_skcipher_ctx(skcipher);
	struct hse_skcipher_ctx *ctx = skcipher_request_ctx(req);
	struct hse_skcipher_alg *hsealg = hse_get_skcipher(skcipher);
	int err, nbytes, ivsize = crypto_skcipher_ivsize(skcipher);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));

	/* handle zero-length input because it's a valid operation */
	if (!req->cryptlen)
		return 0;

	ctx->buf = kzalloc(req->cryptlen, GFP_KERNEL);
	if (IS_ERR_OR_NULL(ctx->buf))
		return -ENOMEM;

	/* Make sure IV is located in a DMAable area */
	memcpy(&ctx->iv, req->iv, ivsize);

	ctx->srv_desc.srv_id = HSE_SRV_ID_SYM_CIPHER;
	ctx->srv_desc.priority = HSE_SRV_PRIO_LOW;

	ctx->srv_desc.skcipher_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
	ctx->srv_desc.skcipher_req.cipher_algo = hsealg->cipher_type;
	ctx->srv_desc.skcipher_req.block_mode = hsealg->block_mode;
	ctx->srv_desc.skcipher_req.cipher_dir = direction;
	ctx->srv_desc.skcipher_req.key_handle = state->crt_key->handle;
	ctx->srv_desc.skcipher_req.iv_len = ivsize;
	ctx->srv_desc.skcipher_req.iv = hse_addr(&ctx->iv);
	ctx->srv_desc.skcipher_req.input_len = req->cryptlen;
	ctx->srv_desc.skcipher_req.input = hse_addr(ctx->buf);
	ctx->srv_desc.skcipher_req.output = hse_addr(ctx->buf);

	nbytes = sg_copy_to_buffer(req->src, sg_nents(req->src),
				   ctx->buf, req->cryptlen);
	if (nbytes != req->cryptlen) {
		err = -ENODATA;
		goto err_free_buf;
	}

	ctx->direction = direction;

	/* req->iv is expected to be set to the last ciphertext block */
	if (direction == HSE_CIPHER_DIR_DECRYPT &&
	    hsealg->cipher_type == HSE_CIPHER_ALGO_AES &&
	    hsealg->block_mode == HSE_CIPHER_BLOCK_MODE_CBC)
		scatterwalk_map_and_copy(req->iv, req->src, req->cryptlen -
					 ivsize, ivsize, 0);

	err = hse_mu_async_req_send(hsealg->mu_inst, HSE_ANY_CHANNEL,
				    srv_desc_addr, req, hse_skcipher_done);
	if (err)
		goto err_free_buf;

	return -EINPROGRESS;
err_free_buf:
	kfree(ctx->buf);
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
	struct hse_skcipher_state *state = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_alg *hsealg = hse_get_skcipher(tfm);
	struct hse_drvdata *drv = dev_get_drvdata(hsealg->dev);

	crypto_skcipher_set_reqsize(tfm, sizeof(struct hse_skcipher_ctx));

	/* acquire key handle from key list */
	spin_lock(&drv->key_lock);
	state->crt_key = list_first_entry_or_null(hsealg->keys_list,
						  struct hse_key, entry);
	if (IS_ERR_OR_NULL(state->crt_key)) {
		dev_dbg(hsealg->dev, "%s: cannot acquire key slot\n", __func__);
		spin_unlock(&drv->key_lock);
		return -ENOKEY;
	}
	list_del(&state->crt_key->entry);
	spin_unlock(&drv->key_lock);

	return 0;
}

/**
 * hse_skcipher_exit - symmetric key cipher exit
 * @tfm: symmetric key cipher transformation
 */
static void hse_skcipher_exit(struct crypto_skcipher *tfm)
{
	struct hse_skcipher_state *state = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_alg *hsealg = hse_get_skcipher(tfm);

	/* add key slot back to appropriate ring */
	list_add_tail(&state->crt_key->entry, hsealg->keys_list);
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
					sizeof(struct hse_skcipher_state),
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
	},
};

/**
 * hse_skcipher_register - register symmetric key algorithms
 * @dev: HSE device
 */
void hse_skcipher_register(struct device *dev)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	int i, err = 0;

	/* register crypto algorithms the device supports */
	for (i = 0; i < ARRAY_SIZE(skcipher_algs); i++) {
		struct hse_skcipher_alg *alg = &skcipher_algs[i];
		const char *name = alg->skcipher.base.cra_name;

		alg->dev = dev;
		alg->mu_inst = drvdata->mu_inst;

		if (alg->cipher_type == HSE_CIPHER_ALGO_AES)
			alg->keys_list = &drvdata->aes_keys;
		else
			alg->keys_list = NULL;

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
