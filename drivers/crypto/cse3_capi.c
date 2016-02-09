// SPDX-License-Identifier: GPL-2.0+
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 *
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * CSE3 Linux Crypto API Interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * later as publishhed by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/scatterlist.h>
#include <crypto/internal/skcipher.h>
#include <crypto/scatterwalk.h>

#include "cse3.h"
#include "cse3_req.h"
#include "cse3_capi.h"

int capi_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
		unsigned int keylen)
{
	cse_ctx_t *ctx = crypto_skcipher_ctx(tfm);

	ctx->dev = cse_dev_ptr;
	if (keylen != AES_KEYSIZE_128)
		return -EINVAL;

	memcpy(ctx->aes_key, key, AES_KEYSIZE_128);
	return 0;
}

static int capi_aes_crypto(struct skcipher_request *req, int flags)
{
	struct cse_crypt_request *new_req;

	/* Init context and check for key */
	cse_ctx_t *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));

	ctx->dev = cse_dev_ptr;
	if (flags & FLAG_CBC)
		memcpy(ctx->aes_iv, req->iv, AES_KEYSIZE_128);

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	if (!new_req) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for crypto request.\n");
		return -ENOMEM;
	}
	new_req->base.ctx = ctx;
	new_req->len_in = new_req->len_out = req->cryptlen;
	new_req->base.flags = flags;
	new_req->base.phase = 1;
	new_req->base.key_id = UNDEFINED;
	new_req->base.extra = req;

	/* Copy input text and send command to the device */
	/* TODO: copy in loop */
	sg_copy_to_buffer(req->src, sg_nents(req->src), new_req->buffer_in,
			new_req->len_in);
	cse_handle_request(ctx->dev, (cse_req_t *)new_req);
	return -EINPROGRESS;
}

int capi_aes_ecb_encrypt(struct skcipher_request *req)
{
	return capi_aes_crypto(req, FLAG_ENC|FLAG_CRYPTO_REQ);
}

int capi_aes_ecb_decrypt(struct skcipher_request *req)
{
	return capi_aes_crypto(req, FLAG_DEC|FLAG_CRYPTO_REQ);
}

int capi_aes_cbc_encrypt(struct skcipher_request *req)
{
	return capi_aes_crypto(req, FLAG_ENC|FLAG_CRYPTO_REQ|FLAG_CBC);
}

int capi_aes_cbc_decrypt(struct skcipher_request *req)
{
	return capi_aes_crypto(req, FLAG_DEC|FLAG_CRYPTO_REQ|FLAG_CBC);
}

int capi_cmac_finup(struct ahash_request *req)
{
	struct cse_cmac_request *new_req;

	/* Init context and check for key */
	cse_ctx_t *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));

	ctx->dev = cse_dev_ptr;

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	if (!new_req) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for cmac request.\n");
		return -ENOMEM;
	}
	new_req->base.ctx = ctx;
	new_req->len_in = req->nbytes;
	new_req->base.flags = FLAG_GEN_MAC|FLAG_CRYPTO_REQ;
	new_req->base.phase = 1;
	new_req->base.key_id = UNDEFINED;
	new_req->base.extra = req;

	/* Copy input text and send command to the device */
	/* TODO: copy in loop */
	sg_copy_to_buffer(req->src, sg_nents(req->src), new_req->buffer_in,
			new_req->len_in);
	cse_handle_request(ctx->dev, (cse_req_t *)new_req);
	return -EINPROGRESS;
}

int capi_cmac_digest(struct ahash_request *req)
{
	return capi_cmac_finup(req);
}

int capi_cmac_init(struct ahash_request *req)
{
	return 0;
}

int capi_cmac_setkey(struct crypto_ahash *tfm, const u8 *key,
		unsigned int keylen)
{
	cse_ctx_t *ctx = crypto_ahash_ctx(tfm);
	int err;

	ctx->dev = cse_dev_ptr;
	err = aes_check_keylen(keylen);
	if (err)
		return err;

	memcpy(ctx->aes_key, key, AES_KEYSIZE_128);
	return 0;
}

int capi_aes_cra_init(struct crypto_skcipher *tfm)
{
	return 0;
}

void capi_aes_cra_exit(struct crypto_skcipher *tfm)
{
}

int capi_cmac_cra_init(struct crypto_tfm *tfm)
{
	return 0;
}
