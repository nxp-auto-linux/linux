// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Hash Support
 *
 * This file contains the kernel crypto API implementation for the hash
 * algorithms and message authentication codes supported by HSE.
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

/**
 * struct hse_hash_state - crypto request context
 * @srv_desc: HSE service descriptor
 * @crt_buf: linearized input buffer
 * @large_buf: buffer larger than block size
 * @len: digest size
 * @crt_idx: current byte index in buffer
 * @channel: MU channel/stream
 */
struct hse_hash_state {
	struct hse_srv_desc *srv_desc;
	void *crt_buf;
	void *large_buf;
	u32 *len;
	u32 crt_idx;
	u8 channel;
};

/**
 * struct hse_hash_ctx - crypto transformation context
 * @partial_hash: buffer containing the intermediate result
 */
struct hse_hash_ctx {
	u8 partial_hash[HSE_MAX_DIGEST_SIZE];
};

/**
 * hse_hash_template - hash template
 * @name: algorithm name
 * @drvname: driver name
 * @blocksize: block size
 * @ahash_alg: hash algorithm template
 * @alg_type: HSE algorithm type (hash/mac)
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
 * @alg_type: HSE algorithm type (hash/mac)
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
static struct hse_halg *hse_get_halg(struct ahash_request *req)
{
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	struct crypto_alg *base = ahash->base.__crt_alg;
	struct hash_alg_common *halg = container_of(base,
						    struct hash_alg_common,
						    base);
	struct ahash_alg *alg = container_of(halg, struct ahash_alg, halg);
	struct hse_halg *t_alg = container_of(alg, struct hse_halg, ahash_alg);

	return t_alg;
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
	struct hse_hash_state *state = ahash_request_ctx(req);
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	unsigned int blocksize = crypto_ahash_blocksize(ahash);
	unsigned int access_mode = state->srv_desc->hash_req.access_mode;
	struct hse_halg *t_alg = hse_get_halg(req);
	int err;
	u32 reply;

	err = hse_mu_recv_response(mu_inst, channel, &reply);
	err = err ? err : hse_err_decode(reply);
	if (err)
		dev_dbg(t_alg->dev, "service response 0x%08x on channel %d\n",
			reply, channel);

	switch (access_mode) {
	case HSE_ACCESS_MODE_UPDATE:
		if (state->srv_desc->hash_req.input_len > blocksize)
			kfree(state->large_buf);
		break;
	case HSE_ACCESS_MODE_FINISH:
		err = hse_mu_release_channel(mu_inst, channel);
	case HSE_ACCESS_MODE_ONE_PASS:
		kfree(state->len);
		kfree(state->srv_desc);
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
	struct hse_hash_state *state = ahash_request_ctx(req);
	struct hse_halg *t_alg = hse_get_halg(req);
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	unsigned int blocksize = crypto_ahash_blocksize(ahash);
	int err;
	u8 channel;

	state->len = kzalloc(sizeof(*state->len) + blocksize, GFP_KERNEL);
	if (!state->len)
		return -ENOMEM;
	*state->len = crypto_ahash_digestsize(ahash);
	state->crt_buf = state->len + 1;
	state->crt_idx = 0;

	state->srv_desc = kzalloc(sizeof(*state->srv_desc), GFP_KERNEL);
	if (!state->srv_desc) {
		err = -ENOMEM;
		goto err_free_buf;
	}

	err = hse_mu_reserve_channel(t_alg->mu_inst, &channel);
	if (err)
		goto err_free_desc;

	state->srv_desc->hash_req.stream_id = channel;
	state->channel = channel;

	state->srv_desc->srv_id = t_alg->srv_id;
	state->srv_desc->srv_meta_data.prio = HSE_SRV_PRIO_MED;
	state->srv_desc->hash_req.hash_algo = t_alg->alg_type;
	state->srv_desc->hash_req.access_mode = HSE_ACCESS_MODE_START;
	state->srv_desc->hash_req.input_len = 0;

	err = hse_mu_send_request(t_alg->mu_inst, channel,
				  hse_addr(state->srv_desc), req, ahash_done);
	if (err)
		goto err_free_desc;

	return -EINPROGRESS;

err_free_desc:
	kfree(state->srv_desc);
err_free_buf:
	kfree(state->len);
	return err;
}

/**
 * ahash_update - asynchronous hash request update
 * @req: asynchronous hash request
 */
static int ahash_update(struct ahash_request *req)
{
	struct hse_hash_state *state = ahash_request_ctx(req);
	struct hse_halg *t_alg = hse_get_halg(req);
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	unsigned int blocksize = crypto_ahash_blocksize(ahash);
	int err;
	unsigned int num_blocks, bytes_left = state->crt_idx + req->nbytes;

	if (req->nbytes == 0) {
		state->srv_desc->hash_req.input_len = state->crt_idx;
		return 0;
	}

	if (bytes_left <= blocksize) {
		scatterwalk_map_and_copy(state->crt_buf + state->crt_idx,
					 req->src, 0, req->nbytes, 0);
		state->crt_idx += req->nbytes;
		return 0;
	}

	num_blocks = bytes_left / blocksize;

	state->large_buf = kzalloc(num_blocks * blocksize, GFP_KERNEL);
	if (IS_ERR_OR_NULL(state->large_buf))
		return -ENOMEM;

	/* copy full blocks */
	memcpy(state->large_buf, state->crt_buf, state->crt_idx);
	scatterwalk_map_and_copy(state->large_buf + state->crt_idx,
				 req->src, 0, num_blocks * blocksize, 0);
	bytes_left -= num_blocks * blocksize;

	/* copy residue */
	scatterwalk_map_and_copy(state->crt_buf + state->crt_idx, req->src,
				 num_blocks * blocksize, bytes_left, 0);
	state->crt_idx = bytes_left;

	state->srv_desc->hash_req.access_mode = HSE_ACCESS_MODE_UPDATE;
	state->srv_desc->hash_req.input_len = num_blocks * blocksize;
	state->srv_desc->hash_req.p_input = hse_addr(state->large_buf);

	err = hse_mu_send_request(t_alg->mu_inst, state->channel,
				  hse_addr(state->srv_desc), req, ahash_done);
	if (err)
		goto err_free_buf;

	return -EINPROGRESS;
err_free_buf:
	kfree(state->large_buf);
	return err;
}

/**
 * ahash_final - asynchronous hash request final
 * @req: asynchronous hash request
 */
static int ahash_final(struct ahash_request *req)
{
	struct hse_hash_state *state = ahash_request_ctx(req);
	struct hse_halg *t_alg = hse_get_halg(req);
	int err;

	state->srv_desc->hash_req.access_mode = HSE_ACCESS_MODE_FINISH;
	state->srv_desc->hash_req.input_len = state->crt_idx;
	state->srv_desc->hash_req.p_input = hse_addr(state->crt_buf);
	state->srv_desc->hash_req.p_hash_len = hse_addr(state->len);
	state->srv_desc->hash_req.p_hash = hse_addr(req->result);

	err = hse_mu_send_request(t_alg->mu_inst, state->channel,
				  hse_addr(state->srv_desc), req, ahash_done);

	return !err ? -EINPROGRESS : err;
}

/**
 * ahash_digest - asynchronous hash request digest
 * @req: asynchronous hash request
 */
static int ahash_digest(struct ahash_request *req)
{
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(req);
	struct hse_hash_state *state = ahash_request_ctx(req);
	struct hse_halg *t_alg = hse_get_halg(req);
	int err;

	state->len = kzalloc(sizeof(*state->len) + req->nbytes, GFP_KERNEL);
	if (!state->len)
		return -ENOMEM;
	*state->len = crypto_ahash_digestsize(ahash);
	state->crt_buf = state->len + 1;

	state->srv_desc = kzalloc(sizeof(*state->srv_desc), GFP_KERNEL);
	if (!state->srv_desc) {
		err = -ENOMEM;
		goto err_free_buf;
	}

	state->srv_desc->srv_id = t_alg->srv_id;
	state->srv_desc->srv_meta_data.prio = HSE_SRV_PRIO_LOW;
	state->srv_desc->hash_req.hash_algo = t_alg->alg_type;

	state->srv_desc->hash_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
	state->srv_desc->hash_req.input_len = req->nbytes;
	state->srv_desc->hash_req.p_input = hse_addr(state->crt_buf);
	state->srv_desc->hash_req.p_hash_len = hse_addr(state->len);
	state->srv_desc->hash_req.p_hash = hse_addr(req->result);

	scatterwalk_map_and_copy(state->crt_buf, req->src, 0, req->nbytes, 0);

	err = hse_mu_send_request(t_alg->mu_inst, HSE_ANY_CHANNEL,
				  hse_addr(state->srv_desc), req, ahash_done);
	if (err)
		goto err_free_desc;

	return -EINPROGRESS;
err_free_desc:
	kfree(state->srv_desc);
err_free_buf:
	kfree(state->len);
	return err;
}

/**
 * hse_hash_cra_init - cryto algorithm init
 * @tfm: crypto transformation
 */
static int hse_hash_cra_init(struct crypto_tfm *tfm)
{
	struct crypto_ahash *ahash = __crypto_ahash_cast(tfm);

	crypto_ahash_set_reqsize(ahash, sizeof(struct hse_hash_state));

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
			.digest = ahash_digest,
			.halg = {
				.digestsize = SHA1_DIGEST_SIZE,
				.statesize = sizeof(struct hse_hash_ctx),
			},
		},
		.alg_type = HSE_HASH_ALGO_SHA_1,
	},
};

/**
 * hse_hash_alloc - allocate hash algorithm
 * @dev: HSE device
 * @keyed: unkeyed/hash or keyed/mac
 * @tpl: hash algorithm template
 */
static struct hse_halg *hse_hash_alloc(struct device *dev, bool keyed,
				       const struct hse_hash_template *tpl)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
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
	t_alg->dev = dev;
	t_alg->mu_inst = drvdata->mu_inst;

	return t_alg;
}

/**
 * hse_hash_init - hash component initial setup
 * @dev: HSE device
 *
 * Return: 0 on success, error code otherwise
 */
int hse_hash_init(struct device *dev)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	int i, err = 0;

	INIT_LIST_HEAD(&drvdata->hash_list);

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
			list_add_tail(&t_alg->entry, &drvdata->hash_list);
			dev_info(dev, "successfully registered alg %s\n", name);
		}
	}

	return err;
}

/**
 * hse_hash_free - hash component final cleanup
 * @dev: HSE device
 */
void hse_hash_free(struct device *dev)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	struct hse_halg *t_alg, *n;

	if (!drvdata->hash_list.next)
		return;

	list_for_each_entry_safe(t_alg, n, &drvdata->hash_list, entry) {
		crypto_unregister_ahash(&t_alg->ahash_alg);
		list_del(&t_alg->entry);
	}
}
