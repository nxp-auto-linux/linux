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
 * struct hse_skcipher_priv - HASH component private data
 * @dev: parent device to be used for error logging
 * @mu_inst: MU instance
 */
struct hse_skcipher_priv {
	struct device *dev;
	void *mu_inst;
};

struct hse_skalg {
	struct skcipher_alg skcipher;
	bool registered;
};

static int skcipher_setkey(struct crypto_skcipher *skcipher, const u8 *key,
			   unsigned int keylen)
{
	return 0;
}

static int skcipher_encrypt(struct skcipher_request *req)
{
	int err = 0;

	return err;
}

static int skcipher_decrypt(struct skcipher_request *req)
{
	int err = 0;

	return err;
}

static struct hse_skalg skcipher_algs[] = {
	{
		.skcipher = {
			.base = {
				.cra_name = "cbc(aes)",
				.cra_driver_name = "cbc-aes-hse",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = skcipher_setkey,
			.encrypt = skcipher_encrypt,
			.decrypt = skcipher_decrypt,
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.ivsize = AES_BLOCK_SIZE,
		},
	},
};

/**
 * hse_skcipher_init - init skcipher algs
 * @dev: parent device
 * @mu_inst: MU instance
 *
 * Return: skcipher component pointer, error code otherwise
 */
int hse_skcipher_init(struct device *dev)
{
	struct hse_drvdata *priv = dev_get_drvdata(dev);
	int i, err = 0;

	/* register crypto algorithms the device supports */
	for (i = 0; i < ARRAY_SIZE(skcipher_algs); i++) {
		struct hse_skalg *alg = &skcipher_algs[i];

		err = crypto_register_skcipher(&alg->skcipher);
		if (err) {
			pr_warn("%s alg registration failed\n",
				alg->skcipher.base.cra_driver_name);
			continue;
		}
		alg->registered = true;
	}

	return err;
}

/**
 * hse_skcipher_free - free skcipher algs
 */
void hse_skcipher_free(void)
{
	struct hse_skalg *alg;
	int i;

	for (i = 0; i < ARRAY_SIZE(skcipher_algs); i++) {
		alg = &skcipher_algs[i];

		if (alg->registered)
			crypto_unregister_skcipher(&alg->skcipher);
	}
}
