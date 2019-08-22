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
#include <crypto/sha.h>
#include <crypto/scatterwalk.h>

#include "hse.h"
#include "hse-mu.h"
#include "hse-abi.h"

#define HSE_MAX_BLOCK_SIZE    SHA512_BLOCK_SIZE

/**
 * struct hse_ahash_ctx - crypto request context
 * @srv_desc: HSE service descriptor
 * @crt_buf: linearized input buffer
 * @dyn_buf: dynamically allocated buffer
 * @len: output buffer/digest size
 * @crt_idx: current byte index in buffer
 * @start_pending: true until the first streaming mode request has been sent
 * @stream: MU stream type channel acquired, if any
 */
struct hse_ahash_ctx {
	struct hse_srv_desc srv_desc;
	u8 crt_buf[HSE_MAX_BLOCK_SIZE];
	void *dyn_buf;
	u32 len;
	u32 crt_idx;
	bool start_pending;
	u8 stream;
};

/**
 * struct hse_ahash_state - crypto transformation state
 * @srv_desc: service descriptor used for setkey operations
 * @key_info: key flags, used for import
 * @crt_key: key currently in use
 * @keybuf: buffer containing the current key
 * @keylen: current (shortened) key size
 */
struct hse_ahash_state {
	struct hse_srv_desc srv_desc;
	struct hse_key_info key_info;
	struct hse_key *crt_key;
	u8 keybuf[HSE_MAX_BLOCK_SIZE];
	u32 keylen;
};

/**
 * hse_hash_template - hash alg template
 * @hash_name: hash algorithm name
 * @hash_drv: hash driver name
 * @hmac_name: hmac algorithm name
 * @hmac_drv: hmac driver name
 * @blocksize: block size
 * @ahash_alg: hash algorithm template
 * @alg_type: HSE algorithm type (hash/hmac)
 */
struct hse_hash_template {
	char hash_name[CRYPTO_MAX_ALG_NAME];
	char hash_drv[CRYPTO_MAX_ALG_NAME];
	char hmac_name[CRYPTO_MAX_ALG_NAME];
	char hmac_drv[CRYPTO_MAX_ALG_NAME];
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
 * @hmac_keys: keys available for hmac
 */
struct hse_halg {
	struct list_head entry;
	u8 alg_type;
	u32 srv_id;
	struct ahash_alg ahash_alg;
	struct device *dev;
	void *mu_inst;
	struct list_head *hmac_keys;
};

/**
 * hse_get_halg - get hash algorithm data from crypto ahash transformation
 * @tfm: crypto ahash transformation
 *
 * Return: pointer to hash algorithm data
 */
static inline struct hse_halg *hse_get_halg(struct crypto_ahash *tfm)
{
	struct crypto_alg *base = tfm->base.__crt_alg;
	struct hash_alg_common *halg = container_of(base,
						    struct hash_alg_common,
						    base);
	struct ahash_alg *ahalg = container_of(halg, struct ahash_alg, halg);

	return container_of(ahalg, struct hse_halg, ahash_alg);
}

/**
 * hse_ahash_done - asynchronous hash request rx callback
 * @mu_inst: MU instance
 * @channel: service channel index
 * @req: asynchronous hash request
 *
 * Common rx callback for hash-related service requests.
 */
static void hse_ahash_done(void *mu_inst, u8 channel, void *req)
{
	struct hse_ahash_ctx *ctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_halg *halg = hse_get_halg(tfm);
	unsigned int input_len, blocksize = crypto_ahash_blocksize(tfm);
	u8 access_mode;
	int err;

	err = hse_mu_async_req_recv(mu_inst, channel);
	if (unlikely(err))
		dev_dbg(halg->dev, "%s: hash/hmac request failed: %d\n",
			__func__, err);

	switch (halg->srv_id) {
	case HSE_SRV_ID_HASH:
		access_mode = ctx->srv_desc.hash_req.access_mode;
		input_len = ctx->srv_desc.hash_req.input_len;
		break;
	case HSE_SRV_ID_MAC:
		access_mode = ctx->srv_desc.mac_req.access_mode;
		input_len = ctx->srv_desc.mac_req.input_len;
		break;
	default:
		dev_dbg(halg->dev, "%s: undefined service ID\n", __func__);
		ahash_request_complete(req, -EIDRM);
		return;
	}

	if (access_mode == HSE_ACCESS_MODE_FINISH && !err)
		hse_mu_channel_release(halg->mu_inst, ctx->stream);

	if (input_len > blocksize)
		kfree(ctx->dyn_buf);

	ahash_request_complete(req, err);
}

/**
 * hse_ahash_init - asynchronous hash request init
 * @req: asynchronous hash request
 */
static int hse_ahash_init(struct ahash_request *req)
{
	struct hse_ahash_ctx *ctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_ahash_state *state = crypto_ahash_ctx(tfm);
	struct hse_halg *halg = hse_get_halg(tfm);

	ctx->srv_desc.srv_id = halg->srv_id;
	ctx->srv_desc.priority = HSE_SRV_PRIO_MED;

	switch (halg->srv_id) {
	case HSE_SRV_ID_HASH:
		ctx->srv_desc.hash_req.hash_algo = halg->alg_type;
		break;
	case HSE_SRV_ID_MAC:
		ctx->srv_desc.mac_req.auth_dir = HSE_AUTH_DIR_GENERATE;
		ctx->srv_desc.mac_req.scheme.mac_algo = HSE_MAC_ALGO_HMAC;
		ctx->srv_desc.mac_req.scheme.hmac.hash_algo = halg->alg_type;
		ctx->srv_desc.mac_req.key_handle = state->crt_key->handle;
		break;
	default:
		dev_dbg(halg->dev, "%s: undefined service ID\n", __func__);
		return -EIDRM;
	}

	ctx->crt_idx = 0;
	ctx->start_pending = true;

	return 0;
}

/**
 * hse_ahash_update - asynchronous hash request update
 * @req: asynchronous hash request
 */
static int hse_ahash_update(struct ahash_request *req)
{
	struct hse_ahash_ctx *ctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_halg *halg = hse_get_halg(tfm);
	unsigned int blocksize = crypto_ahash_blocksize(tfm);
	unsigned int full_blocks, bytes_left;
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));
	u8 access_mode;
	int err;

	/* exit if no data */
	if (req->nbytes == 0)
		return 0;

	bytes_left = ctx->crt_idx + req->nbytes;
	full_blocks = (bytes_left / blocksize) * blocksize;
	if (bytes_left <= blocksize) {
		/* buffer data for next update and exit */
		scatterwalk_map_and_copy(ctx->crt_buf + ctx->crt_idx,
					 req->src, 0, req->nbytes, 0);
		ctx->crt_idx = bytes_left;
		return 0;
	}

	ctx->dyn_buf = kzalloc(full_blocks, GFP_KERNEL);
	if (IS_ERR_OR_NULL(ctx->dyn_buf))
		return -ENOMEM;

	/* copy full blocks to dynamic buffer */
	memcpy(ctx->dyn_buf, ctx->crt_buf, ctx->crt_idx);
	scatterwalk_map_and_copy(ctx->dyn_buf + ctx->crt_idx, req->src, 0,
				 full_blocks - ctx->crt_idx, 0);
	bytes_left -= full_blocks;

	/* check if first streaming request */
	if (unlikely(ctx->start_pending)) {
		access_mode = HSE_ACCESS_MODE_START;
		ctx->start_pending = false;

		err = hse_mu_channel_acquire(halg->mu_inst, &ctx->stream, true);
		if (unlikely(err))
			goto err_free_buf;
	} else {
		access_mode = HSE_ACCESS_MODE_UPDATE;
	}

	switch (halg->srv_id) {
	case HSE_SRV_ID_HASH:
		ctx->srv_desc.hash_req.access_mode = access_mode;
		ctx->srv_desc.hash_req.stream_id = ctx->stream;
		ctx->srv_desc.hash_req.input_len = full_blocks;
		ctx->srv_desc.hash_req.input = hse_addr(ctx->dyn_buf);
		break;
	case HSE_SRV_ID_MAC:
		ctx->srv_desc.mac_req.access_mode = access_mode;
		ctx->srv_desc.mac_req.stream_id = ctx->stream;
		ctx->srv_desc.mac_req.input_len = full_blocks;
		ctx->srv_desc.mac_req.input = hse_addr(ctx->dyn_buf);
		break;
	default:
		dev_dbg(halg->dev, "%s: undefined service ID\n", __func__);
		err = -EIDRM;
		goto err_release_channel;
	}

	err = hse_mu_async_req_send(halg->mu_inst, ctx->stream,
				    srv_desc_addr, req, hse_ahash_done);
	if (unlikely(err))
		goto err_release_channel;

	/* copy residue to static buffer */
	scatterwalk_map_and_copy(ctx->crt_buf, req->src, full_blocks -
				 ctx->crt_idx, bytes_left, 0);
	ctx->crt_idx = bytes_left;

	return -EINPROGRESS;
err_release_channel:
	if (access_mode == HSE_ACCESS_MODE_START)
		hse_mu_channel_release(halg->mu_inst, ctx->stream);
err_free_buf:
	kfree(ctx->dyn_buf);
	return err;
}

/**
 * hse_ahash_final - asynchronous hash request final
 * @req: asynchronous hash request
 */
static int hse_ahash_final(struct ahash_request *req)
{
	struct hse_ahash_ctx *ctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_halg *halg = hse_get_halg(tfm);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));
	u8 access_mode;
	int err;

	ctx->len = crypto_ahash_digestsize(tfm);

	/* check if first streaming request */
	if (unlikely(ctx->start_pending)) {
		access_mode = HSE_ACCESS_MODE_ONE_PASS;
		ctx->stream = HSE_ANY_CHANNEL;
		ctx->srv_desc.priority = HSE_SRV_PRIO_LOW;
		ctx->start_pending = false;
	} else {
		access_mode = HSE_ACCESS_MODE_FINISH;
	}

	switch (halg->srv_id) {
	case HSE_SRV_ID_HASH:
		ctx->srv_desc.hash_req.access_mode = access_mode;
		ctx->srv_desc.hash_req.stream_id = ctx->stream;
		ctx->srv_desc.hash_req.input_len = ctx->crt_idx;
		ctx->srv_desc.hash_req.input = hse_addr(ctx->crt_buf);
		ctx->srv_desc.hash_req.hash_len = hse_addr(&ctx->len);
		ctx->srv_desc.hash_req.hash = hse_addr(req->result);
		break;
	case HSE_SRV_ID_MAC:
		ctx->srv_desc.mac_req.access_mode = access_mode;
		ctx->srv_desc.mac_req.stream_id = ctx->stream;
		ctx->srv_desc.mac_req.input_len = ctx->crt_idx;
		ctx->srv_desc.mac_req.input = hse_addr(ctx->crt_buf);
		ctx->srv_desc.mac_req.tag_len = hse_addr(&ctx->len);
		ctx->srv_desc.mac_req.tag = hse_addr(req->result);
		break;
	default:
		dev_dbg(halg->dev, "%s: undefined service ID\n", __func__);
		return -EIDRM;
	}

	err = hse_mu_async_req_send(halg->mu_inst, ctx->stream,
				    srv_desc_addr, req, hse_ahash_done);

	return !err ? -EINPROGRESS : err;
}

/**
 * hse_ahash_finup - asynchronous hash request finup
 * @req: asynchronous hash request
 */
static int hse_ahash_finup(struct ahash_request *req)
{
	struct hse_ahash_ctx *ctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_halg *halg = hse_get_halg(tfm);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));
	unsigned int bytes_left, blocksize = crypto_ahash_blocksize(tfm);
	u8 access_mode;
	int err;

	ctx->len = crypto_ahash_digestsize(tfm);

	/* check if first streaming request */
	if (unlikely(ctx->start_pending)) {
		access_mode = HSE_ACCESS_MODE_ONE_PASS;
		ctx->stream = HSE_ANY_CHANNEL;
		ctx->srv_desc.priority = HSE_SRV_PRIO_LOW;
		ctx->start_pending = false;
	} else {
		access_mode = HSE_ACCESS_MODE_FINISH;
	}

	bytes_left = ctx->crt_idx + req->nbytes;
	if (bytes_left > blocksize) {
		/* relocate remaining data to dynamic buffer */
		ctx->dyn_buf = kzalloc(bytes_left, GFP_KERNEL);
		if (IS_ERR_OR_NULL(ctx->dyn_buf))
			return -ENOMEM;
		memcpy(ctx->dyn_buf, ctx->crt_buf, ctx->crt_idx);
	} else {
		/* reuse static buffer */
		ctx->dyn_buf = ctx->crt_buf;
	}

	scatterwalk_map_and_copy(ctx->dyn_buf + ctx->crt_idx,
				 req->src, 0, req->nbytes, 0);

	switch (halg->srv_id) {
	case HSE_SRV_ID_HASH:
		ctx->srv_desc.hash_req.access_mode = access_mode;
		ctx->srv_desc.hash_req.stream_id = ctx->stream;
		ctx->srv_desc.hash_req.input_len = bytes_left;
		ctx->srv_desc.hash_req.input = hse_addr(ctx->dyn_buf);
		ctx->srv_desc.hash_req.hash_len = hse_addr(&ctx->len);
		ctx->srv_desc.hash_req.hash = hse_addr(req->result);
		break;
	case HSE_SRV_ID_MAC:
		ctx->srv_desc.mac_req.access_mode = access_mode;
		ctx->srv_desc.mac_req.stream_id = ctx->stream;
		ctx->srv_desc.mac_req.input_len = bytes_left;
		ctx->srv_desc.mac_req.input = hse_addr(ctx->dyn_buf);
		ctx->srv_desc.mac_req.tag_len = hse_addr(&ctx->len);
		ctx->srv_desc.mac_req.tag = hse_addr(req->result);
		break;
	default:
		dev_dbg(halg->dev, "%s: undefined service ID\n", __func__);
		err = -EIDRM;
		goto err_free_buf;
	}

	err = hse_mu_async_req_send(halg->mu_inst, ctx->stream,
				    srv_desc_addr, req, hse_ahash_done);
	if (unlikely(err))
		goto err_free_buf;

	return -EINPROGRESS;
err_free_buf:
	kfree(ctx->dyn_buf);
	return err;
}

/**
 * hse_ahash_digest - asynchronous hash request digest
 * @req: asynchronous hash request
 */
static int hse_ahash_digest(struct ahash_request *req)
{
	struct hse_ahash_ctx *ctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_ahash_state *state = crypto_ahash_ctx(tfm);
	struct hse_halg *halg = hse_get_halg(tfm);
	unsigned int blocksize = crypto_ahash_blocksize(tfm);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));
	int err;

	if (req->nbytes > blocksize) {
		ctx->dyn_buf = kzalloc(req->nbytes, GFP_KERNEL);
		if (IS_ERR_OR_NULL(ctx->dyn_buf))
			return -ENOMEM;
	} else {
		/* reuse static buffer */
		ctx->dyn_buf = ctx->crt_buf;
	}
	ctx->len = crypto_ahash_digestsize(tfm);

	ctx->srv_desc.srv_id = halg->srv_id;
	ctx->srv_desc.priority = HSE_SRV_PRIO_LOW;

	switch (halg->srv_id) {
	case HSE_SRV_ID_HASH:
		ctx->srv_desc.hash_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
		ctx->srv_desc.hash_req.hash_algo = halg->alg_type;
		ctx->srv_desc.hash_req.input_len = req->nbytes;
		ctx->srv_desc.hash_req.input = hse_addr(ctx->dyn_buf);
		ctx->srv_desc.hash_req.hash_len = hse_addr(&ctx->len);
		ctx->srv_desc.hash_req.hash = hse_addr(req->result);
		break;
	case HSE_SRV_ID_MAC:
		ctx->srv_desc.mac_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
		ctx->srv_desc.mac_req.auth_dir = HSE_AUTH_DIR_GENERATE;
		ctx->srv_desc.mac_req.scheme.mac_algo = HSE_MAC_ALGO_HMAC;
		ctx->srv_desc.mac_req.scheme.hmac.hash_algo = halg->alg_type;
		ctx->srv_desc.mac_req.key_handle = state->crt_key->handle;
		ctx->srv_desc.mac_req.input_len = req->nbytes;
		ctx->srv_desc.mac_req.input = hse_addr(ctx->dyn_buf);
		ctx->srv_desc.mac_req.tag_len = hse_addr(&ctx->len);
		ctx->srv_desc.mac_req.tag = hse_addr(req->result);
		break;
	default:
		dev_dbg(halg->dev, "%s: undefined service ID\n", __func__);
		err = -EIDRM;
		goto err_free_buf;
	}

	scatterwalk_map_and_copy(ctx->dyn_buf, req->src, 0, req->nbytes, 0);

	err = hse_mu_async_req_send(halg->mu_inst, HSE_ANY_CHANNEL,
				    srv_desc_addr, req, hse_ahash_done);
	if (unlikely(err))
		goto err_free_buf;

	return -EINPROGRESS;
err_free_buf:
	kfree(ctx->dyn_buf);
	return err;
}

/**
 * HSE doesn't support import/export operations
 */
static int hse_ahash_export(struct ahash_request *req, void *out)
{
	return -EOPNOTSUPP;
}

/**
 * HSE doesn't support import/export operations
 */
static int hse_ahash_import(struct ahash_request *req, const void *in)
{
	return -EOPNOTSUPP;
}

/**
 * hse_ahash_setkey - asynchronous hash setkey operation
 * @tfm: crypto ahash transformation
 * @key: input key
 * @keylen: input key size
 *
 * The maximum hmac key size supported by HSE is equal to the hash algorithm
 * block size. Any key exceeding this size is shortened by hashing it before
 * being imported into the key store, in accordance with hmac specification.
 * Zero padding shall be added to keys shorter than HSE_KEY_HMAC_MIN_SIZE.
 */
static int hse_ahash_setkey(struct crypto_ahash *tfm, const u8 *key,
			    unsigned int keylen)
{
	struct hse_ahash_state *state = crypto_ahash_ctx(tfm);
	struct hse_halg *halg = hse_get_halg(tfm);
	u32 srv_desc_addr = lower_32_bits(hse_addr(&state->srv_desc));
	unsigned int blocksize = crypto_ahash_blocksize(tfm);
	int err;

	/* do not update the key if already imported */
	if (keylen == state->keylen &&
	    unlikely(!crypto_memneq(key, state->keybuf, keylen)))
		return 0;

	if (keylen > blocksize) {
		state->keylen = crypto_ahash_digestsize(tfm);

		memzero_explicit(&state->srv_desc, sizeof(state->srv_desc));
		state->srv_desc.srv_id = HSE_SRV_ID_HASH;
		state->srv_desc.priority = HSE_SRV_PRIO_HIGH;

		state->srv_desc.hash_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
		state->srv_desc.hash_req.hash_algo = halg->alg_type;
		state->srv_desc.hash_req.input_len = keylen;
		state->srv_desc.hash_req.input = hse_addr((void *)key);
		state->srv_desc.hash_req.hash_len = hse_addr(&state->keylen);
		state->srv_desc.hash_req.hash = hse_addr(state->keybuf);

		err = hse_mu_sync_req(halg->mu_inst, HSE_ANY_CHANNEL,
				      srv_desc_addr);
		if (unlikely(err)) {
			dev_dbg(halg->dev, "%s: key hash request failed: %d\n",
				__func__, err);
			return err;
		}

		memzero_explicit(&state->srv_desc, sizeof(state->srv_desc));
	} else {
		state->keylen = max(HSE_KEY_HMAC_MIN_SIZE, keylen);
		memcpy(state->keybuf, key, keylen);
		memset(state->keybuf + keylen, 0u, state->keylen - keylen);
	}

	state->key_info.key_flags = HSE_KF_USAGE_SIGN;
	state->key_info.key_bit_len = state->keylen * BITS_PER_BYTE;
	state->key_info.key_type = HSE_KEY_TYPE_HMAC;

	state->srv_desc.srv_id = HSE_SRV_ID_IMPORT_KEY;
	state->srv_desc.priority = HSE_SRV_PRIO_HIGH;

	state->srv_desc.import_key_req.key_handle = state->crt_key->handle;
	state->srv_desc.import_key_req.key_info = hse_addr(&state->key_info);
	state->srv_desc.import_key_req.key = hse_addr(state->keybuf);
	state->srv_desc.import_key_req.key_len = state->keylen;
	state->srv_desc.import_key_req.cipher_key = HSE_INVALID_KEY_HANDLE;
	state->srv_desc.import_key_req.auth_key = HSE_INVALID_KEY_HANDLE;

	err = hse_mu_sync_req(halg->mu_inst, HSE_ANY_CHANNEL, srv_desc_addr);
	if (unlikely(err))
		dev_dbg(halg->dev, "%s: key import request failed: %d\n",
			__func__, err);

	return err;
}

/**
 * hse_ahash_cra_init - cryto algorithm init
 * @gtfm: generic crypto transformation
 */
static int hse_ahash_cra_init(struct crypto_tfm *gtfm)
{
	struct crypto_ahash *tfm = __crypto_ahash_cast(gtfm);
	struct hse_ahash_state *state = crypto_ahash_ctx(tfm);
	struct hse_halg *halg = hse_get_halg(tfm);
	struct hse_drvdata *drv = dev_get_drvdata(halg->dev);

	crypto_ahash_set_reqsize(tfm, sizeof(struct hse_ahash_ctx));

	if (halg->srv_id != HSE_SRV_ID_MAC)
		return 0;

	/* remove key slot from hmac ring */
	spin_lock(&drv->key_lock);
	state->crt_key = list_first_entry_or_null(halg->hmac_keys,
						  struct hse_key, entry);
	if (IS_ERR_OR_NULL(state->crt_key)) {
		dev_dbg(halg->dev, "%s: cannot acquire key slot\n", __func__);
		spin_unlock(&drv->key_lock);
		return -ENOKEY;
	}
	list_del(&state->crt_key->entry);
	spin_unlock(&drv->key_lock);

	return 0;
}

/**
 * hse_ahash_cra_exit - cryto algorithm exit
 * @gtfm: generic crypto transformation
 */
static void hse_ahash_cra_exit(struct crypto_tfm *gtfm)
{
	struct crypto_ahash *tfm = __crypto_ahash_cast(gtfm);
	struct hse_ahash_state *state = crypto_ahash_ctx(tfm);
	struct hse_halg *halg = hse_get_halg(tfm);

	if (halg->srv_id == HSE_SRV_ID_HASH)
		return;

	/* add key slot back to hmac ring */
	list_add_tail(&state->crt_key->entry, halg->hmac_keys);
}

static const struct hse_hash_template driver_hash[] = {
	{
		.hash_name = "md5",
		.hash_drv = "md5-hse",
		.hmac_name = "hmac(md5)",
		.hmac_drv = "hmac-md5-hse",
		.blocksize = MD5_BLOCK_WORDS * 4,
		.template_ahash = {
			.halg = {
				.digestsize = MD5_DIGEST_SIZE,
			},
		},
		.alg_type = HSE_HASH_ALGO_MD5,
	}, {
		.hash_name = "sha1",
		.hash_drv = "sha1-hse",
		.hmac_name = "hmac(sha1)",
		.hmac_drv = "hmac-sha1-hse",
		.blocksize = SHA1_BLOCK_SIZE,
		.template_ahash = {
			.halg = {
				.digestsize = SHA1_DIGEST_SIZE,
			},
		},
		.alg_type = HSE_HASH_ALGO_SHA_1,
	},
};

/**
 * hse_hash_alloc - allocate hash algorithm
 * @dev: HSE device
 * @keyed: unkeyed hash or hmac
 * @tpl: hash algorithm template
 */
static struct hse_halg *hse_hash_alloc(struct device *dev, bool keyed,
				       const struct hse_hash_template *tpl)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	struct hse_halg *halg;
	struct crypto_alg *alg;
	const char *name, *drvname;

	halg = devm_kzalloc(dev, sizeof(*halg), GFP_KERNEL);
	if (IS_ERR_OR_NULL(halg))
		return ERR_PTR(-ENOMEM);

	halg->ahash_alg = tpl->template_ahash;
	alg = &halg->ahash_alg.halg.base;

	halg->alg_type = tpl->alg_type;
	halg->dev = dev;
	halg->mu_inst = drvdata->mu_inst;
	halg->hmac_keys = &drvdata->hmac_keys;

	halg->ahash_alg.init = hse_ahash_init;
	halg->ahash_alg.update = hse_ahash_update;
	halg->ahash_alg.final = hse_ahash_final;
	halg->ahash_alg.finup = hse_ahash_finup;
	halg->ahash_alg.digest = hse_ahash_digest;
	halg->ahash_alg.export = hse_ahash_export;
	halg->ahash_alg.import = hse_ahash_import;
	halg->ahash_alg.halg.statesize = sizeof(struct hse_ahash_state);

	if (keyed) {
		halg->srv_id = HSE_SRV_ID_MAC;
		name = tpl->hmac_name;
		drvname = tpl->hmac_drv;
		halg->ahash_alg.setkey = hse_ahash_setkey;
	} else {
		halg->srv_id = HSE_SRV_ID_HASH;
		name = tpl->hash_name;
		drvname = tpl->hash_drv;
		halg->ahash_alg.setkey = NULL;
	}

	snprintf(alg->cra_name, CRYPTO_MAX_ALG_NAME, "%s", name);
	snprintf(alg->cra_driver_name, CRYPTO_MAX_ALG_NAME, "%s", drvname);

	alg->cra_module = THIS_MODULE;
	alg->cra_init = hse_ahash_cra_init;
	alg->cra_exit = hse_ahash_cra_exit;
	alg->cra_ctxsize = sizeof(struct hse_ahash_state);
	alg->cra_priority = HSE_CRA_PRIORITY;
	alg->cra_blocksize = tpl->blocksize;
	alg->cra_alignmask = 0;
	alg->cra_flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_TYPE_AHASH;
	alg->cra_type = &crypto_ahash_type;

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

		/* register unkeyed hash */
		halg = hse_hash_alloc(dev, false, alg);
		if (IS_ERR(halg)) {
			dev_err(dev, "failed to allocate %s\n", alg->hash_drv);
			continue;
		}

		err = crypto_register_ahash(&halg->ahash_alg);
		if (unlikely(err)) {
			dev_err(dev, "failed to register alg %s: %d\n",
				alg->hash_name, err);
			continue;
		} else {
			list_add_tail(&halg->entry, &drvdata->hash_algs);
		}

		/* register hmac version */
		halg = hse_hash_alloc(dev, true, alg);
		if (IS_ERR(halg)) {
			dev_err(dev, "failed to allocate %s\n", alg->hmac_drv);
			continue;
		}

		err = crypto_register_ahash(&halg->ahash_alg);
		if (unlikely(err)) {
			dev_info(dev, "registered alg %s\n", alg->hash_name);
			dev_err(dev, "failed to register alg %s: %d\n",
				alg->hmac_name, err);
			continue;
		} else {
			list_add_tail(&halg->entry, &drvdata->hash_algs);
		}

		dev_info(dev, "registered algs %s, %s\n", alg->hash_name,
			 alg->hmac_name);
	}
}

/**
 * hse_hash_unregister - unregister hash and hmac algorithms
 * @dev: HSE device
 */
void hse_hash_unregister(struct device *dev)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	struct hse_halg *halg, *tmp;
	int err;

	if (unlikely(!drvdata->hash_algs.next))
		return;

	list_for_each_entry_safe(halg, tmp, &drvdata->hash_algs, entry) {
		err = crypto_unregister_ahash(&halg->ahash_alg);
		if (unlikely(err))
			dev_warn(dev, "failed to unregister %s: %d\n",
				 halg->ahash_alg.halg.base.cra_name, err);
		else
			list_del(&halg->entry);
	}
}
