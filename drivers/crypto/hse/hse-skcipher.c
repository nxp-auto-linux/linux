// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Symmetric Key Cipher Support
 *
 * This file contains the kernel crypto API implementation for the
 * symmetric key cipher algorithms supported by HSE.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/crypto.h>
#include <crypto/aes.h>
#include <crypto/skcipher.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/skcipher.h>

#include "hse.h"
#include "hse-abi.h"
#include "hse-mu.h"

/**
 * struct hse_symkey - symmetric key entry
 * @entry: key entry in the available keys list
 * @handle: hse key handle
 */
struct hse_symkey {
	struct list_head entry;
	u32 handle;
};

/**
 * struct hse_skcipher_ctx - skcipher request context
 * @srv_desc: HSE service descriptor
 * @iv: initialization vector
 * @buf: linearized buffer
 */
struct hse_skcipher_ctx {
	struct hse_srv_desc srv_desc;
	u8 iv[AES_BLOCK_SIZE];
	void *buf;
};

/**
 * struct hse_skcipher_state - crypto transformation state/context
 * @key_srv_desc: HSE service descriptor, used by setkey
 * @key_info: key flags, used for import
 * @crt_key: key currently in use
 * @keybuf: buffer containing the current key
 * @channel: MU channel
 */
struct hse_skcipher_state {
	struct hse_srv_desc key_srv_desc;
	struct hse_key_info key_info;
	struct hse_symkey *crt_key;
	u8 keybuf[AES_MAX_KEY_SIZE];
	u8 channel;
};

/**
 * hse_skcipher_alg - symmetric key cipher data
 * @skcipher: symmetric key cipher
 * @registered: tfm registered/pending
 * @cipher_type: HSE cipher type
 * @block_mode: cipher block mode
 * @key_group_id: key group id from key catalog
 * @keys_list: list of available symmetric aes key slots
 * @dev: HSE device
 * @mu_inst: MU instance
 */
struct hse_skcipher_alg {
	struct skcipher_alg skcipher;
	bool registered;
	u8 cipher_type;
	u8 block_mode;
	u8 key_group_id;
	u8 key_group_size;
	struct list_head keys_list;
	struct device *dev;
	void *mu_inst;
};

/**
 * hse_get_skcipher_alg - get cipher algorithm data from crypto request
 * @req: symmetric key cipher request
 *
 * Return: pointer to cipher algorithm data
 */
static struct hse_skcipher_alg
*get_hse_skcipher_alg(struct crypto_skcipher *tfm)
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
 * skcipher_setkey - setkey operation
 * @skcipher: symmetric key cipher
 * @key: input key
 * @keylen: input key size
 */
static int skcipher_setkey(struct crypto_skcipher *skcipher, const u8 *key,
			   unsigned int keylen)
{
	struct hse_skcipher_state *state = crypto_skcipher_ctx(skcipher);
	struct hse_skcipher_alg *hsealg = get_hse_skcipher_alg(skcipher);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&state->key_srv_desc));
	u32 reply;
	int err;

	memcpy(&state->keybuf, key, keylen);

	state->key_info.key_flags = HSE_KF_MU_INST | HSE_KF_USAGE_ENCRYPT |
			      HSE_KF_USAGE_DECRYPT;
	state->key_info.key_bit_len = keylen << 3;
	state->key_info.key_type = HSE_KEY_TYPE_AES;

	state->key_srv_desc.srv_id = HSE_SRV_ID_IMPORT_KEY;
	state->key_srv_desc.priority = HSE_SRV_PRIO_HIGH;

	state->key_srv_desc.import_key_req.target_key_handle =
		state->crt_key->handle;
	state->key_srv_desc.import_key_req.p_key_info =
		hse_addr(&state->key_info);
	state->key_srv_desc.import_key_req.p_key = hse_addr(&state->keybuf);
	state->key_srv_desc.import_key_req.key_len = keylen;
	state->key_srv_desc.import_key_req.cipher_key = HSE_INVALID_KEY_HANDLE;
	state->key_srv_desc.import_key_req.auth_key = HSE_INVALID_KEY_HANDLE;

	err = hse_mu_request_srv(hsealg->mu_inst, HSE_ANY_CHANNEL,
				 srv_desc_addr, &reply);
	err = err ? err : hse_err_decode(reply);
	if (err)
		dev_dbg(hsealg->dev, "service response 0x%08X\n", reply);

	return err;
}

/**
 * skcipher_done - symmetric key cipher rx callback
 * @mu_inst: MU instance
 * @channel: service channel index
 * @skreq: symmetric key cipher request
 *
 * Common RX callback for symmetric key cipher operations.
 */
static void skcipher_done(void *mu_inst, u8 channel, void *skreq)
{
	struct skcipher_request *req = skreq;
	struct hse_skcipher_ctx *ctx = skcipher_request_ctx(req);
	struct crypto_skcipher *skcipher = crypto_skcipher_reqtfm(req);
	struct hse_skcipher_alg *hsealg = get_hse_skcipher_alg(skcipher);
	int err, nbytes, ivsize = crypto_skcipher_ivsize(skcipher);
	u32 reply;

	err = hse_mu_recv_response(mu_inst, channel, &reply);
	err = err ? err : hse_err_decode(reply);
	if (err)
		dev_dbg(hsealg->dev, "service response 0x%08X on channel %d\n",
			reply, channel);

	nbytes = sg_copy_from_buffer(req->dst, sg_nents(req->dst),
				     ctx->buf, req->cryptlen);
	if (nbytes != req->cryptlen)
		err = -ENODATA;

	skcipher_request_complete(req, err);
}

/**
 * skcipher_crypt - symmetric cipher operation
 * @req: symmetric key cipher request
 * @direction: encrypt/decrypt
 */
static int skcipher_crypt(struct skcipher_request *req, u8 direction)
{
	struct crypto_skcipher *skcipher = crypto_skcipher_reqtfm(req);
	struct hse_skcipher_state *state = crypto_skcipher_ctx(skcipher);
	struct hse_skcipher_ctx *ctx = skcipher_request_ctx(req);
	struct hse_skcipher_alg *hsealg = get_hse_skcipher_alg(skcipher);
	int err, nbytes, ivsize = crypto_skcipher_ivsize(skcipher);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));

	ctx->buf = kzalloc(req->cryptlen, GFP_KERNEL);
	if (IS_ERR_OR_NULL(ctx->buf))
		return -ENOMEM;
	memcpy(&ctx->iv, req->iv, ivsize);

	ctx->srv_desc.srv_id = HSE_SRV_ID_SYM_CIPHER;
	ctx->srv_desc.priority = HSE_SRV_PRIO_LOW;

	ctx->srv_desc.skcipher_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
	ctx->srv_desc.skcipher_req.cipher_algo = hsealg->cipher_type;
	ctx->srv_desc.skcipher_req.block_mode = hsealg->block_mode;
	ctx->srv_desc.skcipher_req.cipher_dir = direction;
	ctx->srv_desc.skcipher_req.key_handle = state->crt_key->handle;
	ctx->srv_desc.skcipher_req.iv_len = ivsize;
	ctx->srv_desc.skcipher_req.p_iv = hse_addr(&ctx->iv);
	ctx->srv_desc.skcipher_req.input_len = req->cryptlen;
	ctx->srv_desc.skcipher_req.p_input = hse_addr(ctx->buf);
	ctx->srv_desc.skcipher_req.p_output = hse_addr(ctx->buf);

	nbytes = sg_copy_to_buffer(req->src, sg_nents(req->src),
				   ctx->buf, req->cryptlen);
	if (nbytes != req->cryptlen) {
		err = -ENODATA;
		goto err_free_buf;
	}

	err = hse_mu_send_request(hsealg->mu_inst, HSE_ANY_CHANNEL,
				  srv_desc_addr, req, skcipher_done);
	if (err)
		goto err_free_buf;

	return -EINPROGRESS;
err_free_buf:
	kfree(ctx->buf);
	return err;
}

static int skcipher_encrypt(struct skcipher_request *req)
{
	return skcipher_crypt(req, HSE_CIPHER_DIR_ENCRYPT);
}

static int skcipher_decrypt(struct skcipher_request *req)
{
	return skcipher_crypt(req, HSE_CIPHER_DIR_DECRYPT);
}

/**
 * hse_skcipher_cra_init - cryto algorithm init
 * @tfm: crypto transformation
 */
static int hse_skcipher_cra_init(struct crypto_skcipher *tfm)
{
	struct hse_skcipher_state *state = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_alg *hsealg = get_hse_skcipher_alg(tfm);

	crypto_skcipher_set_reqsize(tfm, sizeof(struct hse_skcipher_ctx));

	state->crt_key = list_first_entry_or_null(&hsealg->keys_list,
						  struct hse_symkey, entry);
	if (IS_ERR_OR_NULL(state->crt_key))
		return -ENOKEY;
	list_del(&state->crt_key->entry);

	return 0;
}

/**
 * hse_skcipher_cra_exit - cryto algorithm exit
 * @tfm: crypto transformation
 */
static void hse_skcipher_cra_exit(struct crypto_skcipher *tfm)
{
	struct hse_skcipher_state *state = crypto_skcipher_ctx(tfm);
	struct hse_skcipher_alg *hsealg = get_hse_skcipher_alg(tfm);

	list_add_tail(&state->crt_key->entry, &hsealg->keys_list);
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
			.setkey = skcipher_setkey,
			.encrypt = skcipher_encrypt,
			.decrypt = skcipher_decrypt,
			.init = hse_skcipher_cra_init,
			.exit = hse_skcipher_cra_exit,
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.ivsize = AES_BLOCK_SIZE,
		},
		.registered = false,
		.cipher_type = HSE_CIPHER_ALGO_AES,
		.block_mode = HSE_CIPHER_BLOCK_MODE_CBC,
		.key_group_id = CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GID,
		.key_group_size = CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GSIZE,
	},
};

/**
 * hse_skcipher_init - init skcipher algs
 * @dev: HSE device
 *
 * Return: skcipher component pointer, error code otherwise
 */
int hse_skcipher_init(struct device *dev)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	struct hse_symkey *key, *tmp;
	int i, err = 0;

	/* register crypto algorithms the device supports */
	for (i = 0; i < ARRAY_SIZE(skcipher_algs); i++) {
		struct hse_skcipher_alg *alg = &skcipher_algs[i];
		const char *name = alg->skcipher.base.cra_driver_name;

		alg->dev = dev;
		alg->mu_inst = drvdata->mu_inst;

		INIT_LIST_HEAD(&alg->keys_list);
		for (i = 0; i < alg->key_group_size; i++) {
			key = kzalloc(sizeof(*key), GFP_KERNEL);
			if (IS_ERR_OR_NULL(key)) {
				err = -ENOMEM;
				list_for_each_entry_safe(key, tmp,
							 &alg->keys_list,
							 entry) {
					list_del(&key->entry);
					kfree(key);
				}
				return err;
			}
			key->handle = HSE_KEY_HANDLE(alg->key_group_id, i);
			list_add_tail(&key->entry, &alg->keys_list);
		}

		err = crypto_register_skcipher(&alg->skcipher);
		if (err) {
			dev_err(dev, "failed to register %s: %d", name, err);
		} else {
			alg->registered = true;
			dev_info(dev, "successfully registered alg %s\n", name);
		}
	}

	return err;
}

/**
 * hse_skcipher_free - free skcipher algs
 */
void hse_skcipher_free(void)
{
	int i;
	struct hse_symkey *key, *tmp;

	for (i = 0; i < ARRAY_SIZE(skcipher_algs); i++) {
		struct hse_skcipher_alg *alg = &skcipher_algs[i];

		if (alg->registered) {
			crypto_unregister_skcipher(&alg->skcipher);

			list_for_each_entry_safe(key, tmp,
						 &alg->keys_list, entry) {
				list_del(&key->entry);
				kfree(key);
			}
		}
	}
}
