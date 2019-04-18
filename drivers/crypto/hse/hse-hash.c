// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - HASH Component
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/crypto.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/md5.h>
#include <crypto/sha.h>
#include <crypto/scatterwalk.h>

#include "hse.h"
#include "hse-mu.h"
#include "hse-hash.h"

#define HSE_CRA_PRIORITY    2000u

/**
 * struct hse_hash_data - HASH component private data
 * @hash_list: list of supported crypto algorithms
 * @dev: parent device to be used for error logging
 * @mu_inst: MU instance
 */
static struct hse_hash_data {
	struct list_head hash_list;
	struct device *dev;
	void *mu_inst;
} priv;

/**
 * struct hse_hash_state - crypto request context
 * @srv_desc: HSE service descriptor
 * @channel: MU channel
 */
struct hse_hash_state {
	struct hse_srv_desc *srv_desc;
	u8 channel;
};

/**
 * struct hse_hash_ctx - crypto transformation context
 * @stream_id: stream to use for START, UPDATE, FINISH access modes
 */
struct hse_hash_ctx {
	u8 stream_id;
};

struct hse_hash_template {
	char name[CRYPTO_MAX_ALG_NAME];
	char drvname[CRYPTO_MAX_ALG_NAME];
	unsigned int blocksize;
	struct ahash_alg template_ahash;
	u8 alg_type;
};

struct hse_halg {
	struct list_head entry;
	u8 alg_type;
	u32 srv_id;
	struct ahash_alg ahash_alg;
};

static int ahash_init(struct ahash_request *req)
{
	return -EOPNOTSUPP;
}

static int ahash_update(struct ahash_request *req)
{
	return -EOPNOTSUPP;
}

static int ahash_final(struct ahash_request *req)
{
	return -EOPNOTSUPP;
}

static int ahash_digest(struct ahash_request *req)
{
	return -EOPNOTSUPP;
}

static int hse_hash_cra_init(struct crypto_tfm *tfm)
{
	struct crypto_ahash *ahash = __crypto_ahash_cast(tfm);

	crypto_ahash_set_reqsize(ahash, sizeof(struct hse_hash_state));

	return 0;
}

static void hse_hash_cra_exit(struct crypto_tfm *tfm)
{
}

static const struct hse_hash_template driver_hash[] = {
	{
		.name = "md5",
		.drvname = "md5-hse",
		.blocksize = MD5_BLOCK_WORDS * 4,
		.template_ahash = {
			.init = ahash_init,
			.update = ahash_update,
			.final = ahash_final,
			.finup = NULL,
			.digest = ahash_digest,
			.export = NULL,
			.import = NULL,
			.setkey = NULL,
			.halg = {
				.digestsize = MD5_DIGEST_SIZE,
				.statesize = sizeof(struct hse_hash_ctx),
			},
		},
		.alg_type = HSE_HASH_ALGO_MD5,
	}, {
		.name = "sha1",
		.drvname = "sha1-hse",
		.blocksize = SHA1_BLOCK_SIZE,
		.template_ahash = {
			.init = ahash_init,
			.update = ahash_update,
			.final = ahash_final,
			.finup = NULL,
			.digest = ahash_digest,
			.export = NULL,
			.import = NULL,
			.setkey = NULL,
			.halg = {
				.digestsize = SHA1_DIGEST_SIZE,
				.statesize = sizeof(struct hse_hash_ctx),
			},
		},
		.alg_type = HSE_HASH_ALGO_SHA_1,
	},
};

static struct hse_halg *hse_hash_alloc(struct device *dev, bool keyed,
				       const struct hse_hash_template *tpl)
{
	struct hse_halg *t_alg;
	struct ahash_alg *halg;
	struct crypto_alg *alg;

	t_alg = devm_kzalloc(dev, sizeof(*t_alg), GFP_KERNEL);
	if (!t_alg)
		return ERR_PTR(-ENOMEM);

	t_alg->ahash_alg = tpl->template_ahash;
	halg = &t_alg->ahash_alg;
	alg = &halg->halg.base;

	snprintf(alg->cra_name, CRYPTO_MAX_ALG_NAME, "%s", tpl->name);
	snprintf(alg->cra_driver_name, CRYPTO_MAX_ALG_NAME, "%s", tpl->drvname);

	alg->cra_module = THIS_MODULE;
	alg->cra_init = hse_hash_cra_init;
	alg->cra_exit = hse_hash_cra_exit;
	alg->cra_ctxsize = sizeof(struct hse_hash_ctx);
	alg->cra_priority = HSE_CRA_PRIORITY;
	alg->cra_blocksize = tpl->blocksize;
	alg->cra_alignmask = 0;
	alg->cra_flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_TYPE_AHASH;
	alg->cra_type = &crypto_ahash_type;

	t_alg->alg_type = tpl->alg_type;
	t_alg->srv_id = HSE_SRV_ID_HASH;

	return t_alg;
}

/**
 * hse_hash_init - HASH component initial setup
 * @dev: parent device
 * @mu_inst: MU instance
 *
 * Return: 0 on success, error code otherwise
 */
int hse_hash_init(struct device *dev, void *mu_inst)
{
	int i, err = 0;

	INIT_LIST_HEAD(&priv.hash_list);

	priv.dev = dev;
	priv.mu_inst = mu_inst;

	/* register crypto algorithms the device supports */
	for (i = 0; i < ARRAY_SIZE(driver_hash); i++) {
		struct hse_halg *t_alg;
		const struct hse_hash_template *alg = &driver_hash[i];
		char *name;

		/* register unkeyed hash */
		t_alg = hse_hash_alloc(dev, false, alg);
		if (IS_ERR(t_alg)) {
			err = PTR_ERR(t_alg);
			dev_err(dev, "failed to allocate %s\n", alg->drvname);
			continue;
		}
		name = t_alg->ahash_alg.halg.base.cra_driver_name;

		err = crypto_register_ahash(&t_alg->ahash_alg);
		if (err) {
			dev_err(dev, "failed to register %s: %d\n", name, err);
			continue;
		} else {
			list_add_tail(&t_alg->entry, &priv.hash_list);
			dev_info(dev, "successfully registered alg %s\n", name);
		}
	}

	return err;
}

/**
 * hse_hash_free - HASH component final cleanup
 */
void hse_hash_free(void)
{
	struct hse_halg *t_alg, *n;

	if (!priv.hash_list.next)
		return;

	list_for_each_entry_safe(t_alg, n, &priv.hash_list, entry) {
		crypto_unregister_ahash(&t_alg->ahash_alg);
		list_del(&t_alg->entry);
	}
}
