// SPDX-License-Identifier: GPL-2.0+
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 *
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 * CSE3 Linux Crypto API Interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * later as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/internal/skcipher.h>
#include <crypto/internal/hash.h>
#include <crypto/scatterwalk.h>

#include "cse3.h"
#include "cse3_capi.h"
#include "cse3_req.h"

#define CSE_CRA_PRIORITY	100

/* Crypto API algorithms wrappers */
struct cse_cipher_alg {
	struct skcipher_alg alg;
	u8 registered;
};

struct cse_ahash_alg {
	struct ahash_alg alg;
	u8 registered;
};

static int capi_copy_output(struct cse_device_data *dev,
			    struct cse_request *req)
{
	struct skcipher_request *cipher_req;
	struct ahash_request *hash_req;
	cse_desc_t *desc = dev->hw_desc;

	if (req->flags & FLAG_GEN_MAC) {
		hash_req = (struct ahash_request *) dev->req->extra;
		memcpy(hash_req->result, desc->mac, AES_MAC_SIZE);
	} else if (req->flags & (FLAG_ENC|FLAG_DEC)) {
		cipher_req = (struct skcipher_request *) dev->req->extra;
		dma_sync_single_for_cpu(dev->device, dev->buffer_out_phys,
				cipher_req->cryptlen, DMA_FROM_DEVICE);
		sg_copy_from_buffer(cipher_req->dst, sg_nents(cipher_req->dst),
				dev->buffer_out, cipher_req->cryptlen);
	}

	return 0;
}

static int capi_copy_input(struct cse_device_data *dev, struct cse_request *req)
{
	struct skcipher_request *cipher_req;
	struct ahash_request *ahash_req;
	cse_desc_t *desc = dev->hw_desc;

	if (req->flags & (FLAG_ENC|FLAG_DEC)) {
		cipher_req = (struct skcipher_request *)req->extra;
		desc->nbits = desc_len(cipher_req->cryptlen);
		memcpy(desc->aes_key, req->ctx->aes_key, AES_KEY_SIZE);
		if (req->flags & FLAG_CBC)
			memcpy(desc->aes_iv, req->ctx->aes_iv, AES_KEY_SIZE);

		if (cse_allocate_buffer(dev->device, &dev->buffer_in,
				&dev->buffer_in_phys, cipher_req->cryptlen,
				DMA_TO_DEVICE))
			return -ENOMEM;
		if (cse_allocate_buffer(dev->device, &dev->buffer_out,
				&dev->buffer_out_phys, cipher_req->cryptlen,
				DMA_FROM_DEVICE))
			return -ENOMEM;
		sg_copy_to_buffer(cipher_req->src, sg_nents(cipher_req->src),
				dev->buffer_in, cipher_req->cryptlen);
		dma_sync_single_for_device(dev->device, dev->buffer_in_phys,
				cipher_req->cryptlen, DMA_TO_DEVICE);

	} else if (req->flags & FLAG_GEN_MAC) {
		ahash_req = (struct ahash_request *)req->extra;
		desc->nbits = desc_len(ahash_req->nbytes);
		memcpy(desc->aes_key, req->ctx->aes_key, AES_KEY_SIZE);

		if (ahash_req->nbytes) {
			if (cse_allocate_buffer(dev->device, &dev->buffer_in,
				&dev->buffer_in_phys, ahash_req->nbytes,
				DMA_TO_DEVICE))
				return -ENOMEM;
			sg_copy_to_buffer(ahash_req->src,
					sg_nents(ahash_req->src),
					dev->buffer_in, ahash_req->nbytes);
			dma_sync_single_for_device(dev->device,
					dev->buffer_in_phys, ahash_req->nbytes,
					DMA_TO_DEVICE);
		}
	}

	return 0;
}

static void capi_complete(struct cse_device_data *dev, struct cse_request *req)
{
	struct crypto_async_request base_req;

	if (dev->req->flags & FLAG_GEN_MAC) {
		base_req = ((struct ahash_request *)dev->req->extra)->base;
		base_req.complete(&base_req, req->error);
	} else if (dev->req->flags & (FLAG_ENC|FLAG_DEC)) {
		base_req = ((struct skcipher_request *)dev->req->extra)->base;
		base_req.complete(&base_req, req->error);
	}
	cse_finish_req(dev, req);
}

static void capi_init_ops(struct cse_request *req)
{
	req->copy_output = capi_copy_output;
	req->copy_input = capi_copy_input;
	req->comp = capi_complete;
	req->free_extra = NULL;
}

static int capi_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
			   unsigned int keylen)
{
	cse_ctx_t *ctx = crypto_skcipher_ctx(tfm);

	if (keylen != AES_KEYSIZE_128)
		return -EINVAL;

	memcpy(ctx->aes_key, key, AES_KEYSIZE_128);
	return 0;
}

static int capi_aes_crypto(struct skcipher_request *req, int flags)
{
	int ret;
	cse_req_t *new_req;

	/* Init context and check for key */
	cse_ctx_t *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));

	if (flags & FLAG_CBC)
		memcpy(ctx->aes_iv, req->iv, AES_KEYSIZE_128);

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	if (!new_req) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for crypto request.\n");
		return -ENOMEM;
	}

	new_req->ctx = ctx;
	new_req->flags = flags;
	new_req->phase = 1;
	new_req->key_id = UNDEFINED;
	new_req->extra = req;
	capi_init_ops(new_req);

	ret = cse_handle_request(ctx->dev, new_req);
	return ret ? ret : -EINPROGRESS;
}

static int capi_aes_ecb_encrypt(struct skcipher_request *req)
{
	return capi_aes_crypto(req, FLAG_ENC);
}

static int capi_aes_ecb_decrypt(struct skcipher_request *req)
{
	return capi_aes_crypto(req, FLAG_DEC);
}

static int capi_aes_cbc_encrypt(struct skcipher_request *req)
{
	return capi_aes_crypto(req, FLAG_ENC|FLAG_CBC);
}

static int capi_aes_cbc_decrypt(struct skcipher_request *req)
{
	return capi_aes_crypto(req, FLAG_DEC|FLAG_CBC);
}

static int capi_cmac_finup(struct ahash_request *req)
{
	int ret;
	cse_req_t *new_req;
	/* Init context and check for key */
	cse_ctx_t *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	if (!new_req) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for cmac request.\n");
		return -ENOMEM;
	}

	new_req->ctx = ctx;
	new_req->flags = FLAG_GEN_MAC;
	new_req->phase = 1;
	new_req->key_id = UNDEFINED;
	new_req->extra = req;
	capi_init_ops(new_req);

	ret = cse_handle_request(ctx->dev, new_req);
	return ret ? ret : -EINPROGRESS;
}

static int capi_cmac_digest(struct ahash_request *req)
{
	return capi_cmac_finup(req);
}

static int capi_cmac_init(struct ahash_request *req)
{
	/* TODO: init state for update operation */
	return 0;
}

static int capi_cmac_setkey(struct crypto_ahash *tfm, const u8 *key,
			    unsigned int keylen)
{
	cse_ctx_t *ctx = crypto_ahash_ctx(tfm);
	int err;

	err = aes_check_keylen(keylen);
	if (err)
		return err;

	memcpy(ctx->aes_key, key, AES_KEYSIZE_128);
	return 0;
}

/**
 * Called at socket bind
 */
static int capi_cra_init(struct crypto_skcipher *tfm)
{
	cse_ctx_t *ctx;

	/* TODO: also set software fallback */
	if (down_trylock(&cse_dev_ptr->access))
		return -EBUSY;
	ctx = crypto_skcipher_ctx(tfm);
	ctx->dev = cse_dev_ptr;

	return 0;
}

static void capi_cra_exit(struct crypto_skcipher *tfm)
{
	cse_ctx_t *ctx = crypto_skcipher_ctx(tfm);

	up(&ctx->dev->access);
}

static struct cse_cipher_alg cipher_algs[] = {
	{
	.alg = {
		.base = {
			.cra_name         = "ecb(aes)",
			.cra_driver_name  = "cse-ecb-aes",
			.cra_priority     = CSE_CRA_PRIORITY,
			.cra_flags        = CRYPTO_ALG_ASYNC,
			.cra_blocksize    = AES_BLOCK_SIZE,
			.cra_ctxsize      = sizeof(cse_ctx_t),
			.cra_alignmask    = 0x0,
			.cra_module       = THIS_MODULE,
		},
		.min_keysize    = AES_MIN_KEY_SIZE,
		.max_keysize    = AES_MAX_KEY_SIZE,
		.setkey         = capi_aes_setkey,
		.encrypt        = capi_aes_ecb_encrypt,
		.decrypt        = capi_aes_ecb_decrypt,
		.init		= capi_cra_init,
		.exit		= capi_cra_exit,
	},
	.registered = 0
	},
	{
	.alg = {

		.base = {
			.cra_name         = "cbc(aes)",
			.cra_driver_name  = "cse-cbc-aes",
			.cra_priority     = CSE_CRA_PRIORITY,
			.cra_flags        = CRYPTO_ALG_ASYNC,
			.cra_blocksize    = AES_BLOCK_SIZE,
			.cra_ctxsize      = sizeof(cse_ctx_t),
			.cra_alignmask    = 0x0,
			.cra_module       = THIS_MODULE,
		},
		.min_keysize    = AES_MIN_KEY_SIZE,
		.max_keysize    = AES_MAX_KEY_SIZE,
		.ivsize         = AES_BLOCK_SIZE,
		.setkey         = capi_aes_setkey,
		.encrypt        = capi_aes_cbc_encrypt,
		.decrypt        = capi_aes_cbc_decrypt,
		.init		= capi_cra_init,
		.exit		= capi_cra_exit,
	},
	.registered = 0
	},
};

static struct cse_ahash_alg hash_algs[] = {
	{
	.alg = {
		.init = capi_cmac_init,
		/* TODO: implement update
		 * .update = capi_cmac_update,
		 * .final = capi_cmac_final,
		 */
		.finup = capi_cmac_finup,
		.digest = capi_cmac_digest,
		.setkey = capi_cmac_setkey,
		.halg.digestsize = AES_BLOCK_SIZE,
		.halg.base = {
			.cra_name = "cmac(aes)",
			.cra_driver_name = "cse-cmac-aes",
			.cra_flags = (CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC),
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(cse_ctx_t),
			.cra_module = THIS_MODULE,
		} },
	.registered = 0
	}
};

void cse_register_crypto_api(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cipher_algs); i++) {
		if (!crypto_register_skcipher(&cipher_algs[i].alg))
			cipher_algs[i].registered = 1;
		else
			dev_err(cse_dev_ptr->device,
					"failed to register %s algo to crypto API.\n",
					cipher_algs[i].alg.base.cra_name);
	}

	for (i = 0; i < ARRAY_SIZE(hash_algs); i++) {
		if (!crypto_register_ahash(&hash_algs[i].alg))
			hash_algs[i].registered = 1;
		else
			dev_err(cse_dev_ptr->device,
					"failed to register %s algo to crypto API.\n",
					hash_algs[i].alg.halg.base.cra_name);
	}
}

void cse_unregister_crypto_api(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hash_algs); i++) {
		if (hash_algs[i].registered)
			crypto_unregister_ahash(&hash_algs[i].alg);
	}

	for (i = 0; i < ARRAY_SIZE(cipher_algs); i++) {
		if (cipher_algs[i].registered)
			crypto_unregister_skcipher(&cipher_algs[i].alg);
	}
}
