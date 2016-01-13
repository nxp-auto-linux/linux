/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 * CSE3 Linux Crypto API Interface
 *
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
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

#ifndef _CSE_CAPI_H
#define _CSE_CAPI_H

#include <linux/crypto.h>
#include <crypto/internal/hash.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>

int capi_aes_ecb_encrypt(struct skcipher_request *req);
int capi_aes_ecb_decrypt(struct skcipher_request *req);
int capi_aes_cbc_encrypt(struct skcipher_request *req);
int capi_aes_cbc_decrypt(struct skcipher_request *req);
int capi_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
		unsigned int keylen);

int capi_cmac_finup(struct ahash_request *req);
int capi_cmac_digest(struct ahash_request *req);
int capi_cmac_init(struct ahash_request *req);
int capi_cmac_setkey(struct crypto_ahash *tfm, const u8 *key,
		unsigned int keylen);

int capi_cra_init(struct crypto_tfm *tfm);
void capi_cra_exit(struct crypto_tfm *tfm);
#endif /* _CSE_CAPI_H */
