// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Asynchronous Hash Support
 *
 * This file contains the implementation of the hash algorithms and hash-based
 * message authentication codes supported for hardware offloading via HSE.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/crypto.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/scatterwalk.h>

#include "hse-abi.h"
#include "hse-core.h"
#include "hse-mu.h"

#define HSE_AHASH_MAX_BLOCK_SIZE     SHA512_BLOCK_SIZE
#define HSE_AHASH_MAX_DIGEST_SIZE    SHA512_DIGEST_SIZE

/**
 * struct hse_ahash_tpl - algorithm template
 * @hash_name: hash algorithm name
 * @hash_drv: hash driver name
 * @hmac_name: hmac algorithm name
 * @hmac_drv: hmac driver name
 * @blocksize: block size
 * @ahash_tpl: ahash template
 * @alg_type: hash algorithm type
 */
struct hse_ahash_tpl {
	char hash_name[CRYPTO_MAX_ALG_NAME];
	char hash_drv[CRYPTO_MAX_ALG_NAME];
	char hmac_name[CRYPTO_MAX_ALG_NAME];
	char hmac_drv[CRYPTO_MAX_ALG_NAME];
	unsigned int blocksize;
	struct ahash_alg ahash_tpl;
	enum hse_hash_algorithm alg_type;
};

/**
 * struct hse_ahash_alg - algorithm private data
 * @ahash: ahash algorithm
 * @entry: position in supported algorithms list
 * @srv_id: HSE service ID
 * @alg_type: hash algorithm type
 * @dev: HSE device
 * @mu_inst: MU instance
 */
struct hse_ahash_alg {
	struct ahash_alg ahash;
	struct list_head entry;
	u32 srv_id;
	enum hse_hash_algorithm alg_type;
	struct device *dev;
	void *mu_inst;
};

/**
 * struct hse_ahash_tfm_ctx - crypto transformation context
 * @srv_desc: service descriptor for setkey ops
 * @srv_desc_dma: service descriptor DMA address
 * @key_slot: current key entry in hmac keyring
 * @keyinf: key information/flags, used for import
 * @keyinf_dma: key information/flags DMA address
 * @keylen: shortened key size, less than block size
 * @keylen_dma: shortened key size DMA address
 * @keybuf: buffer containing current shortened key
 * @keybuf_dma: current shortened key DMA address
 */
struct hse_ahash_tfm_ctx {
	struct hse_srv_desc srv_desc;
	dma_addr_t srv_desc_dma;
	struct hse_key *key_slot;
	struct hse_key_info keyinf;
	dma_addr_t keyinf_dma;
	u32 keylen;
	dma_addr_t keylen_dma;
	u8 keybuf[HSE_AHASH_MAX_BLOCK_SIZE] ____cacheline_aligned;
	dma_addr_t keybuf_dma;
} ____cacheline_aligned;

/**
 * struct hse_ahash_req_ctx - crypto request context
 * @srv_desc: service descriptor for hash/hmac ops
 * @srv_desc_dma: service descriptor DMA address
 * @streaming_mode: request in streaming mode
 * @stream: acquired MU stream type channel
 * @cache: block-sized cache for small input fragments
 * @cache_idx: current written byte index in the cache
 * @buf: dynamically allocated linearized input buffer
 * @buf_dma: linearized input buffer DMA address
 * @buflen: size of current linearized input buffer
 * @outlen: result buffer size, equal to digest size
 * @outlen_dma: result buffer size DMA address
 * @result: result buffer containing message digest
 * @result_dma: result buffer DMA address
 */
struct hse_ahash_req_ctx {
	struct hse_srv_desc srv_desc;
	dma_addr_t srv_desc_dma;
	bool streaming_mode;
	u8 stream;
	u8 cache[HSE_AHASH_MAX_BLOCK_SIZE];
	u32 cache_idx;
	void *buf;
	dma_addr_t buf_dma;
	u32 buflen;
	u32 outlen;
	dma_addr_t outlen_dma;
	u8 result[HSE_AHASH_MAX_DIGEST_SIZE] ____cacheline_aligned;
	dma_addr_t result_dma;
} ____cacheline_aligned;

/**
 * hse_ahash_get_alg - get hash algorithm data from crypto ahash transformation
 * @tfm: crypto ahash transformation
 *
 * Return: pointer to hash algorithm data
 */
static inline struct hse_ahash_alg *hse_ahash_get_alg(struct crypto_ahash *tfm)
{
	struct ahash_alg *alg = container_of(crypto_hash_alg_common(tfm),
					     struct ahash_alg, halg);

	return container_of(alg, struct hse_ahash_alg, ahash);
}

/**
 * hse_ahash_done - asynchronous hash request rx callback
 * @mu_inst: MU instance
 * @channel: service channel index
 * @req: asynchronous hash request
 *
 * Common rx callback for hash/hmac service requests in any access mode.
 */
static void hse_ahash_done(void *mu_inst, u8 channel, void *req)
{
	struct hse_ahash_req_ctx *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);
	enum hse_srv_access_mode access_mode;
	int err;

	if (alg->srv_id == HSE_SRV_ID_HASH)
		access_mode = rctx->srv_desc.hash_req.access_mode;
	else
		access_mode = rctx->srv_desc.mac_req.access_mode;

	err = hse_mu_async_req_recv(mu_inst, channel);
	if (unlikely(err)) {
		dev_dbg(alg->dev, "%s: %s request in mode %d failed: %d\n",
			__func__, crypto_ahash_alg_name(tfm), access_mode, err);

		switch (access_mode) {
		case HSE_ACCESS_MODE_FINISH:
			hse_mu_channel_release(alg->mu_inst, rctx->stream);
			fallthrough;
		case HSE_ACCESS_MODE_ONE_PASS:
			dma_unmap_single(alg->dev, rctx->outlen_dma,
					 sizeof(rctx->outlen), DMA_TO_DEVICE);
			dma_unmap_single(alg->dev, rctx->result_dma,
					 rctx->outlen, DMA_FROM_DEVICE);
			fallthrough;
		default:
			dma_unmap_single(alg->dev, rctx->srv_desc_dma,
					 sizeof(rctx->srv_desc), DMA_TO_DEVICE);
			dma_free_coherent(alg->dev, rctx->buflen, rctx->buf,
					  rctx->buf_dma);
			rctx->buflen = 0;
			break;
		}

		ahash_request_complete(req, err);
		return;
	}

	switch (access_mode) {
	case HSE_ACCESS_MODE_START:
		rctx->streaming_mode = true;
		break;
	case HSE_ACCESS_MODE_FINISH:
		hse_mu_channel_release(alg->mu_inst, rctx->stream);
		fallthrough;
	case HSE_ACCESS_MODE_ONE_PASS:
		dma_unmap_single(alg->dev, rctx->srv_desc_dma,
				 sizeof(rctx->srv_desc), DMA_TO_DEVICE);
		dma_free_coherent(alg->dev, rctx->buflen, rctx->buf,
				  rctx->buf_dma);
		rctx->buflen = 0;

		dma_unmap_single(alg->dev, rctx->outlen_dma,
				 sizeof(rctx->outlen), DMA_TO_DEVICE);
		dma_unmap_single(alg->dev, rctx->result_dma, rctx->outlen,
				 DMA_FROM_DEVICE);

		/* copy message digest */
		memcpy(((struct ahash_request *)req)->result, rctx->result,
		       crypto_ahash_digestsize(tfm));
		break;
	default:
		break;
	}

	ahash_request_complete(req, 0);
}

/**
 * hse_ahash_init - asynchronous hash request init
 * @req: asynchronous hash request
 */
static int hse_ahash_init(struct ahash_request *req)
{
	struct hse_ahash_req_ctx *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_ahash_tfm_ctx *tctx = crypto_ahash_ctx(tfm);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);
	unsigned int blocksize = crypto_ahash_blocksize(tfm);
	int err;

	rctx->srv_desc.srv_id = alg->srv_id;
	rctx->srv_desc.priority = HSE_SRV_PRIO_MED;

	switch (alg->srv_id) {
	case HSE_SRV_ID_HASH:
		rctx->srv_desc.hash_req.hash_algo = alg->alg_type;
		break;
	case HSE_SRV_ID_MAC:
		rctx->srv_desc.mac_req.auth_dir = HSE_AUTH_DIR_GENERATE;
		rctx->srv_desc.mac_req.scheme.mac_algo = HSE_MAC_ALGO_HMAC;
		rctx->srv_desc.mac_req.scheme.hmac.hash_algo = alg->alg_type;
		rctx->srv_desc.mac_req.key_handle = tctx->key_slot->handle;
		break;
	}

	rctx->srv_desc_dma = dma_map_single(alg->dev, &rctx->srv_desc,
					    sizeof(rctx->srv_desc),
					    DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->srv_desc_dma)))
		return -ENOMEM;

	rctx->buf = dma_alloc_coherent(alg->dev, blocksize, &rctx->buf_dma,
				       GFP_KERNEL);
	if (IS_ERR_OR_NULL(rctx->buf)) {
		err = -ENOMEM;
		goto err_unmap_srv_desc;
	}
	rctx->buflen = blocksize;
	rctx->cache_idx = 0;
	rctx->streaming_mode = false;

	err = hse_mu_channel_acquire(alg->mu_inst, &rctx->stream, true);
	if (err)
		goto err_free_buf;

	return 0;
err_free_buf:
	dma_free_coherent(alg->dev, rctx->buflen, rctx->buf, rctx->buf_dma);
err_unmap_srv_desc:
	dma_unmap_single(alg->dev, rctx->srv_desc_dma, sizeof(rctx->srv_desc),
			 DMA_TO_DEVICE);
	return err;
}

/**
 * hse_ahash_update - asynchronous hash request update
 * @req: asynchronous hash request
 */
static int hse_ahash_update(struct ahash_request *req)
{
	struct hse_ahash_req_ctx *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);
	unsigned int blocksize = crypto_ahash_blocksize(tfm);
	unsigned int full_blocks, bytes_left;
	int err;

	/* exit if no data */
	if (req->nbytes == 0)
		return 0;

	bytes_left = rctx->cache_idx + req->nbytes;
	full_blocks = (bytes_left / blocksize) * blocksize;
	if (bytes_left <= blocksize) {
		/* cache data for next update and exit */
		scatterwalk_map_and_copy(rctx->cache + rctx->cache_idx,
					 req->src, 0, req->nbytes, 0);
		rctx->cache_idx = bytes_left;
		return 0;
	}

	if (rctx->buflen < full_blocks) {
		void *oldbuf = rctx->buf;
		dma_addr_t oldbuf_dma = rctx->buf_dma;

		/* realloc larger dynamic buffer */
		rctx->buf = dma_alloc_coherent(alg->dev, full_blocks,
					       &rctx->buf_dma, GFP_KERNEL);
		if (IS_ERR_OR_NULL(rctx->buf)) {
			rctx->buf = oldbuf;
			rctx->buf_dma = oldbuf_dma;
			err = -ENOMEM;
			goto err_release_channel;
		}
		dma_free_coherent(alg->dev, rctx->buflen, oldbuf, oldbuf_dma);
		rctx->buflen = full_blocks;
	}

	/* copy full blocks to dynamic buffer */
	memcpy(rctx->buf, rctx->cache, rctx->cache_idx);
	scatterwalk_map_and_copy(rctx->buf + rctx->cache_idx, req->src, 0,
				 full_blocks - rctx->cache_idx, 0);
	bytes_left -= full_blocks;
	/* sync needed as the cores and HSE do not share a coherency domain */
	dma_sync_single_for_device(alg->dev, rctx->buf_dma, full_blocks,
				   DMA_TO_DEVICE);

	switch (alg->srv_id) {
	case HSE_SRV_ID_HASH:
		rctx->srv_desc.hash_req.access_mode = rctx->streaming_mode ?
						      HSE_ACCESS_MODE_UPDATE :
						      HSE_ACCESS_MODE_START;
		rctx->srv_desc.hash_req.stream_id = rctx->stream;
		rctx->srv_desc.hash_req.input_len = full_blocks;
		rctx->srv_desc.hash_req.input = rctx->buf_dma;
		break;
	case HSE_SRV_ID_MAC:
		rctx->srv_desc.mac_req.access_mode = rctx->streaming_mode ?
						     HSE_ACCESS_MODE_UPDATE :
						     HSE_ACCESS_MODE_START;
		rctx->srv_desc.mac_req.stream_id = rctx->stream;
		rctx->srv_desc.mac_req.input_len = full_blocks;
		rctx->srv_desc.mac_req.input = rctx->buf_dma;
		break;
	}

	dma_sync_single_for_device(alg->dev, rctx->srv_desc_dma,
				   sizeof(rctx->srv_desc), DMA_TO_DEVICE);
	err = hse_mu_async_req_send(alg->mu_inst, rctx->stream,
				    lower_32_bits(rctx->srv_desc_dma),
				    req, hse_ahash_done);
	if (unlikely(err))
		goto err_release_channel;

	/* copy residue to block-sized cache */
	scatterwalk_map_and_copy(rctx->cache, req->src, full_blocks -
				 rctx->cache_idx, bytes_left, 0);
	rctx->cache_idx = bytes_left;

	return -EINPROGRESS;
err_release_channel:
	hse_mu_channel_release(alg->mu_inst, rctx->stream);
	dma_free_coherent(alg->dev, rctx->buflen, rctx->buf, rctx->buf_dma);
	dma_unmap_single(alg->dev, rctx->srv_desc_dma, sizeof(rctx->srv_desc),
			 DMA_TO_DEVICE);
	return err;
}

/**
 * hse_ahash_final - asynchronous hash request final
 * @req: asynchronous hash request
 */
static int hse_ahash_final(struct ahash_request *req)
{
	struct hse_ahash_req_ctx *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);
	int err;

	rctx->outlen = crypto_ahash_digestsize(tfm);
	rctx->result_dma = dma_map_single(alg->dev, rctx->result,
					  rctx->outlen, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->result_dma))) {
		err = -ENOMEM;
		goto err_release_channel;
	}
	rctx->outlen_dma = dma_map_single(alg->dev, &rctx->outlen,
					  sizeof(rctx->outlen), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->outlen_dma))) {
		err = -ENOMEM;
		goto err_unmap_result;
	}

	/* copy remaining data to buffer */
	memcpy(rctx->buf, rctx->cache, rctx->cache_idx);
	/* sync needed as the cores and HSE do not share a coherency domain */
	dma_sync_single_for_device(alg->dev, rctx->buf_dma, rctx->cache_idx,
				   DMA_TO_DEVICE);

	/* use ONE-PASS access mode if no START request has been issued */
	if (!rctx->streaming_mode) {
		hse_mu_channel_release(alg->mu_inst, rctx->stream);
		rctx->stream = HSE_ANY_CHANNEL;
		rctx->srv_desc.priority = HSE_SRV_PRIO_LOW;
	}

	switch (alg->srv_id) {
	case HSE_SRV_ID_HASH:
		rctx->srv_desc.hash_req.access_mode = rctx->streaming_mode ?
						      HSE_ACCESS_MODE_FINISH :
						      HSE_ACCESS_MODE_ONE_PASS;
		rctx->srv_desc.hash_req.stream_id = rctx->stream;
		rctx->srv_desc.hash_req.input_len = rctx->cache_idx;
		rctx->srv_desc.hash_req.input = rctx->buf_dma;
		rctx->srv_desc.hash_req.hash_len = rctx->outlen_dma;
		rctx->srv_desc.hash_req.hash = rctx->result_dma;
		break;
	case HSE_SRV_ID_MAC:
		rctx->srv_desc.mac_req.access_mode = rctx->streaming_mode ?
						     HSE_ACCESS_MODE_FINISH :
						     HSE_ACCESS_MODE_ONE_PASS;
		rctx->srv_desc.mac_req.stream_id = rctx->stream;
		rctx->srv_desc.mac_req.input_len = rctx->cache_idx;
		rctx->srv_desc.mac_req.input = rctx->buf_dma;
		rctx->srv_desc.mac_req.tag_len = rctx->outlen_dma;
		rctx->srv_desc.mac_req.tag = rctx->result_dma;
		break;
	}

	dma_sync_single_for_device(alg->dev, rctx->srv_desc_dma,
				   sizeof(rctx->srv_desc), DMA_TO_DEVICE);
	err = hse_mu_async_req_send(alg->mu_inst, rctx->stream,
				    lower_32_bits(rctx->srv_desc_dma),
				    req, hse_ahash_done);
	if (unlikely(err))
		goto err_unmap_outlen;

	return -EINPROGRESS;
err_unmap_outlen:
	dma_unmap_single(alg->dev, rctx->outlen_dma, sizeof(rctx->outlen),
			 DMA_TO_DEVICE);
err_unmap_result:
	dma_unmap_single(alg->dev, rctx->result_dma, rctx->outlen,
			 DMA_FROM_DEVICE);
err_release_channel:
	hse_mu_channel_release(alg->mu_inst, rctx->stream);
	dma_free_coherent(alg->dev, rctx->buflen, rctx->buf, rctx->buf_dma);
	dma_unmap_single(alg->dev, rctx->srv_desc_dma, sizeof(rctx->srv_desc),
			 DMA_TO_DEVICE);
	return err;
}

/**
 * hse_ahash_finup - asynchronous hash request finup
 * @req: asynchronous hash request
 */
static int hse_ahash_finup(struct ahash_request *req)
{
	struct hse_ahash_req_ctx *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);
	unsigned int bytes_left;
	int err;

	rctx->outlen = crypto_ahash_digestsize(tfm);
	rctx->result_dma = dma_map_single(alg->dev, rctx->result,
					  rctx->outlen, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->result_dma))) {
		err = -ENOMEM;
		goto err_release_channel;
	}
	rctx->outlen_dma = dma_map_single(alg->dev, &rctx->outlen,
					  sizeof(rctx->outlen), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->outlen_dma))) {
		err = -ENOMEM;
		goto err_unmap_result;
	}

	bytes_left = rctx->cache_idx + req->nbytes;
	if (rctx->buflen < bytes_left) {
		void *oldbuf = rctx->buf;
		dma_addr_t oldbuf_dma = rctx->buf_dma;

		/* realloc larger dynamic buffer */
		rctx->buf = dma_alloc_coherent(alg->dev, bytes_left,
					       &rctx->buf_dma, GFP_KERNEL);
		if (IS_ERR_OR_NULL(rctx->buf)) {
			rctx->buf = oldbuf;
			rctx->buf_dma = oldbuf_dma;
			err = -ENOMEM;
			goto err_unmap_outlen;
		}
		dma_free_coherent(alg->dev, rctx->buflen, oldbuf, oldbuf_dma);
		rctx->buflen = bytes_left;
	}

	/* copy remaining data to buffer */
	memcpy(rctx->buf, rctx->cache, rctx->cache_idx);
	scatterwalk_map_and_copy(rctx->buf + rctx->cache_idx,
				 req->src, 0, req->nbytes, 0);
	/* sync needed as the cores and HSE do not share a coherency domain */
	dma_sync_single_for_device(alg->dev, rctx->buf_dma, rctx->cache_idx,
				   DMA_TO_DEVICE);

	/* use ONE-PASS access mode if no START request has been issued */
	if (!rctx->streaming_mode) {
		hse_mu_channel_release(alg->mu_inst, rctx->stream);
		rctx->stream = HSE_ANY_CHANNEL;
		rctx->srv_desc.priority = HSE_SRV_PRIO_LOW;
	}

	switch (alg->srv_id) {
	case HSE_SRV_ID_HASH:
		rctx->srv_desc.hash_req.access_mode = rctx->streaming_mode ?
						      HSE_ACCESS_MODE_FINISH :
						      HSE_ACCESS_MODE_ONE_PASS;
		rctx->srv_desc.hash_req.stream_id = rctx->stream;
		rctx->srv_desc.hash_req.input_len = bytes_left;
		rctx->srv_desc.hash_req.input = rctx->buf_dma;
		rctx->srv_desc.hash_req.hash_len = rctx->outlen_dma;
		rctx->srv_desc.hash_req.hash = rctx->result_dma;
		break;
	case HSE_SRV_ID_MAC:
		rctx->srv_desc.mac_req.access_mode = rctx->streaming_mode ?
						     HSE_ACCESS_MODE_FINISH :
						     HSE_ACCESS_MODE_ONE_PASS;
		rctx->srv_desc.mac_req.stream_id = rctx->stream;
		rctx->srv_desc.mac_req.input_len = bytes_left;
		rctx->srv_desc.mac_req.input = rctx->buf_dma;
		rctx->srv_desc.mac_req.tag_len = rctx->outlen_dma;
		rctx->srv_desc.mac_req.tag = rctx->result_dma;
		break;
	}

	dma_sync_single_for_device(alg->dev, rctx->srv_desc_dma,
				   sizeof(rctx->srv_desc), DMA_TO_DEVICE);
	err = hse_mu_async_req_send(alg->mu_inst, rctx->stream,
				    lower_32_bits(rctx->srv_desc_dma),
				    req, hse_ahash_done);
	if (unlikely(err))
		goto err_unmap_outlen;

	return -EINPROGRESS;
err_unmap_outlen:
	dma_unmap_single(alg->dev, rctx->outlen_dma, sizeof(rctx->outlen),
			 DMA_TO_DEVICE);
err_unmap_result:
	dma_unmap_single(alg->dev, rctx->result_dma, rctx->outlen,
			 DMA_FROM_DEVICE);
err_release_channel:
	hse_mu_channel_release(alg->mu_inst, rctx->stream);
	dma_free_coherent(alg->dev, rctx->buflen, rctx->buf, rctx->buf_dma);
	dma_unmap_single(alg->dev, rctx->srv_desc_dma, sizeof(rctx->srv_desc),
			 DMA_TO_DEVICE);
	return err;
}

/**
 * hse_ahash_digest - asynchronous hash request digest
 * @req: asynchronous hash request
 */
static int hse_ahash_digest(struct ahash_request *req)
{
	struct hse_ahash_req_ctx *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_ahash_tfm_ctx *tctx = crypto_ahash_ctx(tfm);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);
	unsigned int blocksize = crypto_ahash_blocksize(tfm);
	int err;

	rctx->outlen = crypto_ahash_digestsize(tfm);
	rctx->result_dma = dma_map_single(alg->dev, rctx->result,
					  rctx->outlen, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->result_dma)))
		return -ENOMEM;

	rctx->outlen_dma = dma_map_single(alg->dev, &rctx->outlen,
					  sizeof(rctx->outlen), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->outlen_dma))) {
		err = -ENOMEM;
		goto err_unmap_result;
	}

	rctx->buflen = max(req->nbytes, blocksize);
	rctx->buf = dma_alloc_coherent(alg->dev, rctx->buflen, &rctx->buf_dma,
				       GFP_KERNEL);
	if (IS_ERR_OR_NULL(rctx->buf)) {
		rctx->buflen = 0;
		err = -ENOMEM;
		goto err_unmap_outlen;
	}

	scatterwalk_map_and_copy(rctx->buf, req->src, 0, req->nbytes, 0);
	/* sync needed as the cores and HSE do not share a coherency domain */
	dma_sync_single_for_device(alg->dev, rctx->buf_dma, req->nbytes,
				   DMA_TO_DEVICE);

	rctx->srv_desc.srv_id = alg->srv_id;
	rctx->srv_desc.priority = HSE_SRV_PRIO_LOW;

	switch (alg->srv_id) {
	case HSE_SRV_ID_HASH:
		rctx->srv_desc.hash_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
		rctx->srv_desc.hash_req.hash_algo = alg->alg_type;
		rctx->srv_desc.hash_req.input_len = req->nbytes;
		rctx->srv_desc.hash_req.input = rctx->buf_dma;
		rctx->srv_desc.hash_req.hash_len = rctx->outlen_dma;
		rctx->srv_desc.hash_req.hash = rctx->result_dma;
		break;
	case HSE_SRV_ID_MAC:
		rctx->srv_desc.mac_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
		rctx->srv_desc.mac_req.auth_dir = HSE_AUTH_DIR_GENERATE;
		rctx->srv_desc.mac_req.scheme.mac_algo = HSE_MAC_ALGO_HMAC;
		rctx->srv_desc.mac_req.scheme.hmac.hash_algo = alg->alg_type;
		rctx->srv_desc.mac_req.key_handle = tctx->key_slot->handle;
		rctx->srv_desc.mac_req.input_len = req->nbytes;
		rctx->srv_desc.mac_req.input = rctx->buf_dma;
		rctx->srv_desc.mac_req.tag_len = rctx->outlen_dma;
		rctx->srv_desc.mac_req.tag = rctx->result_dma;
		break;
	}

	rctx->srv_desc_dma = dma_map_single(alg->dev, &rctx->srv_desc,
					    sizeof(rctx->srv_desc),
					    DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(alg->dev, rctx->srv_desc_dma))) {
		err = -ENOMEM;
		goto err_free_buf;
	}

	err = hse_mu_async_req_send(alg->mu_inst, HSE_ANY_CHANNEL,
				    lower_32_bits(rctx->srv_desc_dma),
				    req, hse_ahash_done);
	if (unlikely(err))
		goto err_unmap_srv_desc;

	return -EINPROGRESS;
err_unmap_srv_desc:
	dma_unmap_single(alg->dev, rctx->srv_desc_dma, sizeof(rctx->srv_desc),
			 DMA_TO_DEVICE);
err_free_buf:
	dma_free_coherent(alg->dev, rctx->buflen, rctx->buf, rctx->buf_dma);
	rctx->buflen = 0;
err_unmap_outlen:
	dma_unmap_single(alg->dev, rctx->outlen_dma, sizeof(rctx->outlen),
			 DMA_TO_DEVICE);
err_unmap_result:
	dma_unmap_single(alg->dev, rctx->result_dma, rctx->outlen,
			 DMA_FROM_DEVICE);
	return err;
}

/**
 * hse_ahash_export - HSE doesn't support import/export operations
 */
static int hse_ahash_export(struct ahash_request *req, void *out)
{
	struct hse_ahash_req_ctx *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);

	dev_err(alg->dev, "%s: partial hash ops not supported\n", __func__);

	hse_mu_channel_release(alg->mu_inst, rctx->stream);
	dma_free_coherent(alg->dev, rctx->buflen, rctx->buf, rctx->buf_dma);
	dma_unmap_single(alg->dev, rctx->srv_desc_dma, sizeof(rctx->srv_desc),
			 DMA_TO_DEVICE);

	return -EOPNOTSUPP;
}

/**
 * hse_ahash_import - HSE doesn't support import/export operations
 */
static int hse_ahash_import(struct ahash_request *req, const void *in)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);

	dev_err(alg->dev, "%s: partial hash ops not supported\n", __func__);

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
	struct hse_ahash_tfm_ctx *tctx = crypto_ahash_ctx(tfm);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);
	unsigned int blocksize = crypto_ahash_blocksize(tfm);
	int err;

	/* do not update the key if already imported */
	if (keylen == tctx->keylen &&
	    unlikely(!crypto_memneq(key, tctx->keybuf, keylen)))
		return 0;

	if (keylen > blocksize) {
		void *tmp_keybuf;
		dma_addr_t tmp_keybuf_dma;

		tmp_keybuf = kmemdup(key, keylen, GFP_KERNEL);
		if (IS_ERR_OR_NULL(tmp_keybuf))
			return -ENOMEM;

		tmp_keybuf_dma = dma_map_single(alg->dev, tmp_keybuf, keylen,
						DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(alg->dev, tmp_keybuf_dma))) {
			kfree(tmp_keybuf);
			return -ENOMEM;
		}

		tctx->keylen = crypto_ahash_digestsize(tfm);
		dma_sync_single_for_device(alg->dev, tctx->keylen_dma,
					   sizeof(tctx->keylen), DMA_TO_DEVICE);

		tctx->srv_desc.srv_id = HSE_SRV_ID_HASH;
		tctx->srv_desc.priority = HSE_SRV_PRIO_HIGH;

		tctx->srv_desc.hash_req.access_mode = HSE_ACCESS_MODE_ONE_PASS;
		tctx->srv_desc.hash_req.hash_algo = alg->alg_type;
		tctx->srv_desc.hash_req.input_len = keylen;
		tctx->srv_desc.hash_req.input = tmp_keybuf_dma;
		tctx->srv_desc.hash_req.hash_len = tctx->keylen_dma;
		tctx->srv_desc.hash_req.hash = tctx->keybuf_dma;

		dma_sync_single_for_device(alg->dev, tctx->srv_desc_dma,
					   sizeof(tctx->srv_desc),
					   DMA_TO_DEVICE);

		err = hse_mu_sync_req(alg->mu_inst, HSE_ANY_CHANNEL,
				      lower_32_bits(tctx->srv_desc_dma));
		memzero_explicit(&tctx->srv_desc, sizeof(tctx->srv_desc));
		dma_unmap_single(alg->dev, tmp_keybuf_dma, keylen,
				 DMA_TO_DEVICE);
		kfree(tmp_keybuf);
		if (unlikely(err)) {
			dev_dbg(alg->dev, "%s: shorten key failed for %s: %d\n",
				__func__, crypto_ahash_alg_name(tfm), err);
			return err;
		}
		dma_sync_single_for_cpu(alg->dev, tctx->keylen_dma,
					sizeof(tctx->keylen), DMA_FROM_DEVICE);
	} else {
		tctx->keylen = max(HSE_KEY_HMAC_MIN_SIZE, keylen);
		memcpy(tctx->keybuf, key, keylen);
		memzero_explicit(tctx->keybuf + keylen, tctx->keylen - keylen);
		dma_sync_single_for_device(alg->dev, tctx->keybuf_dma,
					   tctx->keylen, DMA_TO_DEVICE);
	}

	tctx->keyinf.key_flags = HSE_KF_USAGE_SIGN;
	tctx->keyinf.key_bit_len = tctx->keylen * BITS_PER_BYTE;
	tctx->keyinf.key_type = HSE_KEY_TYPE_HMAC;

	dma_sync_single_for_device(alg->dev, tctx->keyinf_dma,
				   sizeof(tctx->keyinf), DMA_TO_DEVICE);

	tctx->srv_desc.srv_id = HSE_SRV_ID_IMPORT_KEY;
	tctx->srv_desc.priority = HSE_SRV_PRIO_HIGH;
	tctx->srv_desc.import_key_req.key_handle = tctx->key_slot->handle;
	tctx->srv_desc.import_key_req.key_info = tctx->keyinf_dma;
	tctx->srv_desc.import_key_req.sym.key = tctx->keybuf_dma;
	tctx->srv_desc.import_key_req.sym.keylen = tctx->keylen;
	tctx->srv_desc.import_key_req.cipher_key = HSE_INVALID_KEY_HANDLE;
	tctx->srv_desc.import_key_req.auth_key = HSE_INVALID_KEY_HANDLE;

	dma_sync_single_for_device(alg->dev, tctx->srv_desc_dma,
				   sizeof(tctx->srv_desc), DMA_TO_DEVICE);

	err = hse_mu_sync_req(alg->mu_inst, HSE_ANY_CHANNEL,
			      lower_32_bits(tctx->srv_desc_dma));
	if (unlikely(err))
		dev_dbg(alg->dev, "%s: key import request failed for %s: %d\n",
			__func__, crypto_ahash_alg_name(tfm), err);

	return err;
}

/**
 * hse_ahash_cra_init - crypto transformation init
 * @gtfm: generic crypto transformation
 */
static int hse_ahash_cra_init(struct crypto_tfm *gtfm)
{
	struct crypto_ahash *tfm = __crypto_ahash_cast(gtfm);
	struct hse_ahash_tfm_ctx *tctx = crypto_ahash_ctx(tfm);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);
	int err;

	crypto_ahash_set_reqsize(tfm, sizeof(struct hse_ahash_req_ctx));

	if (alg->srv_id != HSE_SRV_ID_MAC)
		return 0;

	tctx->key_slot = hse_key_slot_acquire(alg->dev, HSE_KEY_TYPE_HMAC);
	if (IS_ERR_OR_NULL(tctx->key_slot)) {
		dev_dbg(alg->dev, "%s: cannot acquire key slot for %s\n",
			__func__, crypto_ahash_alg_name(tfm));
		return PTR_ERR(tctx->key_slot);
	}

	tctx->srv_desc_dma = dma_map_single_attrs(alg->dev, &tctx->srv_desc,
						  sizeof(tctx->srv_desc),
						  DMA_TO_DEVICE,
						  DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->srv_desc_dma))) {
		err = -ENOMEM;
		goto err_release_key_slot;
	}

	tctx->keyinf_dma = dma_map_single_attrs(alg->dev, &tctx->keyinf,
						sizeof(tctx->keyinf),
						DMA_TO_DEVICE,
						DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->keyinf_dma))) {
		err = -ENOMEM;
		goto err_unmap_srv_desc;
	}

	tctx->keybuf_dma = dma_map_single_attrs(alg->dev, tctx->keybuf,
						sizeof(tctx->keybuf),
						DMA_BIDIRECTIONAL,
						DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->keybuf_dma))) {
		err = -ENOMEM;
		goto err_unmap_keyinf;
	}

	tctx->keylen_dma = dma_map_single_attrs(alg->dev, &tctx->keylen,
						sizeof(tctx->keylen),
						DMA_BIDIRECTIONAL,
						DMA_ATTR_SKIP_CPU_SYNC);
	if (unlikely(dma_mapping_error(alg->dev, tctx->keylen_dma))) {
		err = -ENOMEM;
		goto err_unmap_keybuf;
	}
	tctx->keylen = 0;

	return 0;
err_unmap_keybuf:
	dma_unmap_single_attrs(alg->dev, tctx->keybuf_dma, sizeof(tctx->keybuf),
			       DMA_BIDIRECTIONAL, DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_keyinf:
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
err_unmap_srv_desc:
	dma_unmap_single_attrs(alg->dev, tctx->srv_desc_dma,
			       sizeof(tctx->srv_desc), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
err_release_key_slot:
	hse_key_slot_release(alg->dev, tctx->key_slot);
	return err;
}

/**
 * hse_ahash_cra_exit - crypto transformation exit
 * @gtfm: generic crypto transformation
 */
static void hse_ahash_cra_exit(struct crypto_tfm *gtfm)
{
	struct crypto_ahash *tfm = __crypto_ahash_cast(gtfm);
	struct hse_ahash_tfm_ctx *tctx = crypto_ahash_ctx(tfm);
	struct hse_ahash_alg *alg = hse_ahash_get_alg(tfm);

	if (alg->srv_id != HSE_SRV_ID_MAC)
		return;

	hse_key_slot_release(alg->dev, tctx->key_slot);

	dma_unmap_single_attrs(alg->dev, tctx->srv_desc_dma,
			       sizeof(tctx->srv_desc), DMA_TO_DEVICE,
			       DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keyinf_dma, sizeof(tctx->keyinf),
			       DMA_TO_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keybuf_dma, sizeof(tctx->keybuf),
			       DMA_BIDIRECTIONAL, DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_single_attrs(alg->dev, tctx->keylen_dma, sizeof(tctx->keylen),
			       DMA_BIDIRECTIONAL, DMA_ATTR_SKIP_CPU_SYNC);
}

static const struct hse_ahash_tpl hse_ahash_algs_tpl[] = {
	{
		.hash_name = "md5",
		.hash_drv = "md5-hse",
		.hmac_name = "hmac(md5)",
		.hmac_drv = "hmac-md5-hse",
		.blocksize = MD5_BLOCK_WORDS * 4,
		.ahash_tpl.halg = {
			.digestsize = MD5_DIGEST_SIZE,
		},
		.alg_type = HSE_HASH_ALGO_MD5,
	}, {
		.hash_name = "sha1",
		.hash_drv = "sha1-hse",
		.hmac_name = "hmac(sha1)",
		.hmac_drv = "hmac-sha1-hse",
		.blocksize = SHA1_BLOCK_SIZE,
		.ahash_tpl.halg = {
			.digestsize = SHA1_DIGEST_SIZE,
		},
		.alg_type = HSE_HASH_ALGO_SHA1,
	}, {
		.hash_name = "sha224",
		.hash_drv = "sha224-hse",
		.hmac_name = "hmac(sha224)",
		.hmac_drv = "hmac-sha224-hse",
		.blocksize = SHA224_BLOCK_SIZE,
		.ahash_tpl.halg = {
			.digestsize = SHA224_DIGEST_SIZE,
		},
		.alg_type = HSE_HASH_ALGO_SHA2_224,
	}, {
		.hash_name = "sha256",
		.hash_drv = "sha256-hse",
		.hmac_name = "hmac(sha256)",
		.hmac_drv = "hmac-sha256-hse",
		.blocksize = SHA256_BLOCK_SIZE,
		.ahash_tpl.halg = {
			.digestsize = SHA256_DIGEST_SIZE,
		},
		.alg_type = HSE_HASH_ALGO_SHA2_256,
	}, {
		.hash_name = "sha384",
		.hash_drv = "sha384-hse",
		.hmac_name = "hmac(sha384)",
		.hmac_drv = "hmac-sha384-hse",
		.blocksize = SHA384_BLOCK_SIZE,
		.ahash_tpl.halg = {
			.digestsize = SHA384_DIGEST_SIZE,
		},
		.alg_type = HSE_HASH_ALGO_SHA2_384,
	}, {
		.hash_name = "sha512",
		.hash_drv = "sha512-hse",
		.hmac_name = "hmac(sha512)",
		.hmac_drv = "hmac-sha512-hse",
		.blocksize = SHA512_BLOCK_SIZE,
		.ahash_tpl.halg = {
			.digestsize = SHA512_DIGEST_SIZE,
		},
		.alg_type = HSE_HASH_ALGO_SHA2_512,
	},
};

/**
 * hse_ahash_alloc - allocate hash algorithm
 * @dev: HSE device
 * @keyed: unkeyed hash or hmac
 * @tpl: hash algorithm template
 */
static struct hse_ahash_alg *hse_ahash_alloc(struct device *dev, bool keyed,
					     const struct hse_ahash_tpl *tpl)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	struct hse_ahash_alg *alg;
	struct crypto_alg *base;
	const char *name, *drvname;

	alg = devm_kzalloc(dev, sizeof(*alg), GFP_KERNEL);
	if (IS_ERR_OR_NULL(alg))
		return ERR_PTR(-ENOMEM);

	alg->ahash = tpl->ahash_tpl;
	base = &alg->ahash.halg.base;

	alg->alg_type = tpl->alg_type;
	alg->dev = dev;
	alg->mu_inst = drv->mu_inst;

	alg->ahash.init = hse_ahash_init;
	alg->ahash.update = hse_ahash_update;
	alg->ahash.final = hse_ahash_final;
	alg->ahash.finup = hse_ahash_finup;
	alg->ahash.digest = hse_ahash_digest;
	alg->ahash.export = hse_ahash_export;
	alg->ahash.import = hse_ahash_import;
	alg->ahash.halg.statesize = sizeof(struct hse_ahash_tfm_ctx);

	if (keyed) {
		alg->srv_id = HSE_SRV_ID_MAC;
		name = tpl->hmac_name;
		drvname = tpl->hmac_drv;
		alg->ahash.setkey = hse_ahash_setkey;
	} else {
		alg->srv_id = HSE_SRV_ID_HASH;
		name = tpl->hash_name;
		drvname = tpl->hash_drv;
		alg->ahash.setkey = NULL;
	}

	snprintf(base->cra_name, CRYPTO_MAX_ALG_NAME, "%s", name);
	snprintf(base->cra_driver_name, CRYPTO_MAX_ALG_NAME, "%s", drvname);

	base->cra_module = THIS_MODULE;
	base->cra_init = hse_ahash_cra_init;
	base->cra_exit = hse_ahash_cra_exit;
	base->cra_ctxsize = sizeof(struct hse_ahash_tfm_ctx);
	base->cra_priority = HSE_CRA_PRIORITY;
	base->cra_blocksize = tpl->blocksize;
	base->cra_alignmask = 0u;
	base->cra_flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_KERN_DRIVER_ONLY;
	base->cra_type = &crypto_ahash_type;

	return alg;
}

/**
 * hse_ahash_register - register hash and hmac algorithms
 * @dev: HSE device
 */
void hse_ahash_register(struct device *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	int i, err = 0;

	INIT_LIST_HEAD(&drv->ahash_algs);

	/* register crypto algorithms supported by device */
	for (i = 0; i < ARRAY_SIZE(hse_ahash_algs_tpl); i++) {
		struct hse_ahash_alg *alg;
		const struct hse_ahash_tpl *tpl = &hse_ahash_algs_tpl[i];

		/* register unkeyed hash */
		alg = hse_ahash_alloc(dev, false, tpl);
		if (IS_ERR(alg)) {
			dev_err(dev, "failed to allocate %s\n", tpl->hash_drv);
			continue;
		}

		err = crypto_register_ahash(&alg->ahash);
		if (unlikely(err)) {
			dev_err(dev, "failed to register alg %s: %d\n",
				tpl->hash_name, err);
			continue;
		} else {
			list_add_tail(&alg->entry, &drv->ahash_algs);
		}

		/* register hmac version */
		alg = hse_ahash_alloc(dev, true, tpl);
		if (IS_ERR(alg)) {
			dev_err(dev, "failed to allocate %s\n", tpl->hmac_drv);
			continue;
		}

		err = crypto_register_ahash(&alg->ahash);
		if (unlikely(err)) {
			dev_info(dev, "registered alg %s\n", tpl->hash_name);
			dev_err(dev, "failed to register alg %s: %d\n",
				tpl->hmac_name, err);
			continue;
		} else {
			list_add_tail(&alg->entry, &drv->ahash_algs);
		}

		dev_info(dev, "registered algs %s,%s\n", tpl->hash_name,
			 tpl->hmac_name);
	}
}

/**
 * hse_ahash_unregister - unregister hash and hmac algorithms
 * @dev: HSE device
 */
void hse_ahash_unregister(struct device *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	struct hse_ahash_alg *alg, *tmp;
	int err;

	if (unlikely(!drv->ahash_algs.next))
		return;

	list_for_each_entry_safe(alg, tmp, &drv->ahash_algs, entry) {
		err = crypto_unregister_ahash(&alg->ahash);
		if (unlikely(err))
			dev_warn(dev, "failed to unregister %s: %d\n",
				 alg->ahash.halg.base.cra_name, err);
		else
			list_del(&alg->entry);
	}
}
