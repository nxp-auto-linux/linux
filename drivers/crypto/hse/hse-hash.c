// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Hash Support
 *
 * This file contains the kernel crypto API implementation for the hash
 * algorithms and hash-based message authentication codes supported by HSE.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/crypto.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/scatterwalk.h>

#include "hse.h"
#include "hse-mu.h"
#include "hse-abi.h"

#define HSE_MAX_DIGEST_SIZE    SHA512_DIGEST_SIZE
#define HSE_MAX_BLOCK_SIZE     SHA512_BLOCK_SIZE

/**
 * struct hse_hash_ctx - crypto request context
 * @srv_desc: HSE service descriptor
 * @crt_buf: linearized input buffer
 * @dyn_buf: dynamically allocated buffer
 * @len: digest size
 * @crt_idx: current byte index in buffer
 * @channel: MU channel/stream
 */
struct hse_hash_ctx {
	struct hse_srv_desc srv_desc;
	u8 crt_buf[HSE_MAX_BLOCK_SIZE];
	void *dyn_buf;
	u32 len;
	u32 crt_idx;
	u8 channel;
};

/**
 * struct hse_hash_state - crypto transformation state
 * @partial_hash: buffer containing the intermediate result
 */
struct hse_hash_state {
	u8 partial_hash[HSE_MAX_DIGEST_SIZE];
};

/**
 * hse_hash_template - hash template
 * @name: algorithm name
 * @drvname: driver name
 * @blocksize: block size
 * @ahash_alg: hash algorithm template
 * @alg_type: HSE algorithm type (hash/hmac)
 */
struct hse_hash_template {
	char name[CRYPTO_MAX_ALG_NAME];
	char drvname[CRYPTO_MAX_ALG_NAME];
	unsigned int blocksize;
	struct ahash_alg template_ahash;
	u8 alg_type;
};

/**
 * hse_halg - hash algorithm data
 * @entry: list of supported algorithms
 * @alg_type: HSE algorithm type (hash/hmac)
 * @srv_id: HSE service ID
 * @ahash_alg: hash algorithm
 * @dev: HSE device
 * @mu_inst: MU instance
 */
struct hse_halg {
	struct list_head entry;
	u8 alg_type;
	u32 srv_id;
	struct ahash_alg ahash_alg;
	struct device *dev;
	void *mu_inst;
};

/**
 * hse_get_halg - get hash algorithm data from crypto request
 * @req: asynchronous hash request
 *
 * Return: pointer to hash algorithm data
 */
static inline struct hse_halg *hse_get_halg(struct ahash_request *req)
{
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	struct crypto_alg *base = ahash->base.__crt_alg;
	struct hash_alg_common *halg = container_of(base,
						    struct hash_alg_common,
						    base);
	struct ahash_alg *ahalg = container_of(halg, struct ahash_alg, halg);

	return container_of(ahalg, struct hse_halg, ahash_alg);
}

/**
 * ahash_done - asynchronous hash rx callback
 * @mu_inst: MU instance
 * @channel: service channel index
 * @req: asynchronous hash request
 *
 * Common rx callback for hash-related sevice requests.
 */
static void ahash_done(void *mu_inst, u8 channel, void *req)
{
	struct hse_hash_ctx *ctx = ahash_request_ctx(req);
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	unsigned int blocksize = crypto_ahash_blocksize(ahash);
	unsigned int access_mode = ctx->srv_desc.hash_req.access_mode;
	struct hse_halg *halg = hse_get_halg(req);
	int err;
	u32 reply;

	err = hse_mu_recv_response(mu_inst, channel, &reply);
	err = err ? err : hse_err_decode(reply);
	if (err)
		dev_dbg(halg->dev, "service response 0x%08X on channel %d\n",
			reply, channel);

	switch (access_mode) {
	case HSE_ACCESS_MODE_START:
		if (err)
			hse_mu_release_channel(mu_inst, channel);
		break;
	case HSE_ACCESS_MODE_UPDATE:
		if (ctx->srv_desc.hash_req.input_len > blocksize)
			kfree(ctx->dyn_buf);
		break;
	case HSE_ACCESS_MODE_FINISH:
		hse_mu_release_channel(mu_inst, channel);
		break;
	case HSE_ACCESS_MODE_ONE_PASS:
		kfree(ctx->dyn_buf);
		break;
	default:
		break;
	}

	ahash_request_complete(req, err);
}

/**
 * ahash_init - asynchronous hash request init
 * @req: asynchronous hash request
 */
static int ahash_init(struct ahash_request *req)
{
	struct hse_hash_ctx *ctx = ahash_request_ctx(req);
	struct hse_halg *halg = hse_get_halg(req);
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));
	int err;

	ctx->crt_idx = 0;
	ctx->len = crypto_ahash_digestsize(ahash);

	err = hse_mu_reserve_channel(halg->mu_inst, &ctx->channel, true);
	if (err)
		return -EBUSY;

	ctx->srv_desc.srv_id = halg->srv_id;
	ctx->srv_desc.priority = HSE_SRV_PRIO_MED;
	ctx->srv_desc.hash_req.hash_algo = halg->alg_type;
	ctx->srv_desc.hash_req.access_mode = HSE_ACCESS_MODE_START;
	ctx->srv_desc.hash_req.stream_id = ctx->channel;
	ctx->srv_desc.hash_req.input_len = 0;

	err = hse_mu_send_request(halg->mu_inst, ctx->channel,
				  srv_desc_addr, req, ahash_done);
	if (err) {
		hse_mu_release_channel(halg->mu_inst, ctx->channel);
		return err;
	}

	return -EINPROGRESS;
}

/**
 * ahash_update - asynchronous hash request update
 * @req: asynchronous hash request
 */
static int ahash_update(struct ahash_request *req)
{
	struct hse_hash_ctx *ctx = ahash_request_ctx(req);
	struct hse_halg *halg = hse_get_halg(req);
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	unsigned int blocksize = crypto_ahash_blocksize(ahash);
	unsigned int num_blocks, bytes_left = ctx->crt_idx + req->nbytes;
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));
	int err;

	if (req->nbytes == 0) {
		ctx->srv_desc.hash_req.input_len = ctx->crt_idx;
		return 0;
	}

	if (bytes_left <= blocksize) {
		scatterwalk_map_and_copy(ctx->crt_buf + ctx->crt_idx,
					 req->src, 0, req->nbytes, 0);
		ctx->crt_idx += req->nbytes;
		return 0;
	}

	num_blocks = bytes_left / blocksize;

	ctx->dyn_buf = kzalloc(num_blocks * blocksize, GFP_KERNEL);
	if (IS_ERR_OR_NULL(ctx->dyn_buf))
		return -ENOMEM;

	/* copy full blocks */
	memcpy(ctx->dyn_buf, ctx->crt_buf, ctx->crt_idx);
	scatterwalk_map_and_copy(ctx->dyn_buf + ctx->crt_idx,
				 req->src, 0, num_blocks * blocksize, 0);
	bytes_left -= num_blocks * blocksize;

	/* copy residue */
	scatterwalk_map_and_copy(ctx->crt_buf + ctx->crt_idx, req->src,
				 num_blocks * blocksize, bytes_left, 0);
	ctx->crt_idx = bytes_left;

	ctx->srv_desc.hash_req.access_mode = HSE_ACCESS_MODE_UPDATE;
	ctx->srv_desc.hash_req.input_len = num_blocks * blocksize;
	ctx->srv_desc.hash_req.input = hse_addr(ctx->dyn_buf);

	err = hse_mu_send_request(halg->mu_inst, ctx->channel,
				  srv_desc_addr, req, ahash_done);
	if (err) {
		kfree(ctx->dyn_buf);
		return err;
	}

	return -EINPROGRESS;
}

/**
 * ahash_final - asynchronous hash request final
 * @req: asynchronous hash request
 */
static int ahash_final(struct ahash_request *req)
{
	struct hse_hash_ctx *ctx = ahash_request_ctx(req);
	struct hse_halg *halg = hse_get_halg(req);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));
	int err;

	ctx->srv_desc.hash_req.access_mode = HSE_ACCESS_MODE_FINISH;
	ctx->srv_desc.hash_req.input_len = ctx->crt_idx;
	ctx->srv_desc.hash_req.input = hse_addr(ctx->crt_buf);
	ctx->srv_desc.hash_req.hash_len = hse_addr(&ctx->len);
	ctx->srv_desc.hash_req.hash = hse_addr(req->result);

	err = hse_mu_send_request(halg->mu_inst, ctx->channel,
				  srv_desc_addr, req, ahash_done);

	return !err ? -EINPROGRESS : err;
}

/**
 * ahash_digest - asynchronous hash request digest
 * @req: asynchronous hash request
 */
static int ahash_digest(struct ahash_request *req)
{
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	struct hse_hash_ctx *ctx = ahash_request_ctx(req);
	struct hse_halg *halg = hse_get_halg(req);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));
	int err;

	ctx->dyn_buf = kzalloc(req->nbytes, GFP_KERNEL);
	if (IS_ERR_OR_NULL(ctx->dyn_buf))
		return -ENOMEM;
	ctx->len = crypto_ahash_digestsize(ahash);

	ctx->srv_desc.srv_id = halg->srv_id;
	ctx->srv_desc.priority = HSE_SRV_PRIO_LOW;
	ctx->srv_desc.hash_req.hash_algo = halg->alg_type;

	ctx->srv_desc.hash_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
	ctx->srv_desc.hash_req.input_len = req->nbytes;
	ctx->srv_desc.hash_req.input = hse_addr(ctx->dyn_buf);
	ctx->srv_desc.hash_req.hash_len = hse_addr(&ctx->len);
	ctx->srv_desc.hash_req.hash = hse_addr(req->result);

	scatterwalk_map_and_copy(ctx->dyn_buf, req->src, 0, req->nbytes, 0);

	err = hse_mu_send_request(halg->mu_inst, HSE_ANY_CHANNEL,
				  srv_desc_addr, req, ahash_done);
	if (err) {
		kfree(ctx->dyn_buf);
		return err;
	}

	return -EINPROGRESS;
}

/**
 * hse_hash_cra_init - cryto algorithm init
 * @tfm: crypto transformation
 */
static int hse_hash_cra_init(struct crypto_tfm *tfm)
{
	struct crypto_ahash *ahash = __crypto_ahash_cast(tfm);

	crypto_ahash_set_reqsize(ahash, sizeof(struct hse_hash_ctx));

	return 0;
}

/**
 * hse_hash_cra_exit - cryto algorithm exit
 * @tfm: crypto transformation
 */
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
			.digest = ahash_digest,
			.halg = {
				.digestsize = MD5_DIGEST_SIZE,
				.statesize = sizeof(struct hse_hash_state),
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
			.digest = ahash_digest,
			.halg = {
				.digestsize = SHA1_DIGEST_SIZE,
				.statesize = sizeof(struct hse_hash_state),
			},
		},
		.alg_type = HSE_HASH_ALGO_SHA_1,
	},
};

/**
 * hse_hash_alloc - allocate hash algorithm
 * @dev: HSE device
 * @keyed: unkeyed/hash or keyed/hmac
 * @tpl: hash algorithm template
 */
static struct hse_halg *hse_hash_alloc(struct device *dev, bool keyed,
				       const struct hse_hash_template *tpl)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	struct hse_halg *halg;
	struct crypto_alg *alg;

	halg = devm_kzalloc(dev, sizeof(*halg), GFP_KERNEL);
	if (IS_ERR_OR_NULL(halg))
		return ERR_PTR(-ENOMEM);

	halg->ahash_alg = tpl->template_ahash;
	alg = &halg->ahash_alg.halg.base;

	snprintf(alg->cra_name, CRYPTO_MAX_ALG_NAME, "%s", tpl->name);
	snprintf(alg->cra_driver_name, CRYPTO_MAX_ALG_NAME, "%s", tpl->drvname);

	alg->cra_module = THIS_MODULE;
	alg->cra_init = hse_hash_cra_init;
	alg->cra_exit = hse_hash_cra_exit;
	alg->cra_ctxsize = sizeof(struct hse_hash_state);
	alg->cra_priority = HSE_CRA_PRIORITY;
	alg->cra_blocksize = tpl->blocksize;
	alg->cra_alignmask = 0;
	alg->cra_flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_TYPE_AHASH;
	alg->cra_type = &crypto_ahash_type;

	halg->alg_type = tpl->alg_type;
	halg->srv_id = HSE_SRV_ID_HASH;
	halg->dev = dev;
	halg->mu_inst = drvdata->mu_inst;

	return halg;
}

/**
 * hse_hash_register - register hash and hmac algorithms
 * @dev: HSE device
 */
void hse_hash_register(struct device *dev)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	int i, err = 0;

	INIT_LIST_HEAD(&drvdata->hash_algs);

	/* register crypto algorithms supported by device */
	for (i = 0; i < ARRAY_SIZE(driver_hash); i++) {
		struct hse_halg *halg;
		const struct hse_hash_template *alg = &driver_hash[i];
		const char *name;

		/* register unkeyed hash */
		halg = hse_hash_alloc(dev, false, alg);
		if (IS_ERR(halg)) {
			err = PTR_ERR(halg);
			dev_err(dev, "failed to allocate %s\n", alg->drvname);
			continue;
		}
		name = halg->ahash_alg.halg.base.cra_driver_name;

		err = crypto_register_ahash(&halg->ahash_alg);
		if (unlikely(err)) {
			dev_err(dev, "failed to register %s: %d\n", name, err);
			continue;
		} else {
			list_add_tail(&halg->entry, &drvdata->hash_algs);
			dev_info(dev, "successfully registered alg %s\n", name);
		}
	}
}

/**
 * hse_hash_unregister - unregister hash and hmac algorithms
 * @dev: HSE device
 */
void hse_hash_unregister(struct device *dev)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	struct hse_halg *halg, *n;

	if (!drvdata->hash_algs.next)
		return;

	list_for_each_entry_safe(halg, n, &drvdata->hash_algs, entry) {
		crypto_unregister_ahash(&halg->ahash_alg);
		list_del(&halg->entry);
	}
}
