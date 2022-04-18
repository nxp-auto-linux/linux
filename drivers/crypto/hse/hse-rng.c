// SPDX-License-Identifier: BSD 3-clause
/*
 * NXP HSE Driver - Hardware True Random Number Generator Support
 *
 * This file contains the hw_random framework support for HSE hardware TRNG.
 *
 * Copyright 2019-2022 NXP
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/hw_random.h>

#include "hse-abi.h"
#include "hse-core.h"

#define HSE_RNG_QUALITY    1024u /* number of entropy bits per 1024 bits */

#define HSE_RNG_CACHE_MIN    64u /* minimum threshold for cache refill */
#define HSE_RNG_CACHE_MAX    512u /* total size of driver internal cache */

/**
 * struct hse_rng_ctx - RNG context
 * @cache: driver internal random data cache
 * @dev: HSE device
 * @srv_desc: service descriptor used for cache refill
 * @cache_idx: current index in internal cache
 * @cache_dma: DMA address of internal cache
 * @req_lock: mutex used for cache refill operations
 */
struct hse_rng_ctx {
	u8 cache[HSE_RNG_CACHE_MAX];
	struct device *dev;
	struct hse_srv_desc srv_desc;
	size_t cache_idx;
	dma_addr_t cache_dma;
	struct mutex req_lock; /* cache request mutex */
};

/**
 * hse_rng_done - RNG request done callback
 * @err: service response error code
 * @_ctx: RNG context
 */
static void hse_rng_done(int err, void *_ctx)
{
	struct hse_rng_ctx *ctx = (struct hse_rng_ctx *)_ctx;

	if (likely(!err))
		ctx->cache_idx += ctx->srv_desc.rng_req.random_num_len;

	mutex_unlock(&ctx->req_lock);

	if (unlikely(err))
		dev_dbg(ctx->dev, "%s: request failed: %d\n", __func__, err);
}

/**
 * hse_rng_refill_cache - refill internal cache if below threshold
 * @rng: hwrng instance
 */
static void hse_rng_refill_cache(struct hwrng *rng)
{
	struct hse_rng_ctx *ctx = (struct hse_rng_ctx *)rng->priv;
	int err;

	if (ctx->cache_idx >= HSE_RNG_CACHE_MIN)
		return;

	if (!mutex_trylock(&ctx->req_lock)) {
		dev_dbg(ctx->dev, "%s: other request in progress\n", __func__);
		return;
	}

	ctx->cache_idx = 0; /* discard remining data bytes */
	ctx->srv_desc.rng_req.random_num_len = HSE_RNG_CACHE_MAX;
	ctx->srv_desc.rng_req.random_num = ctx->cache_dma;

	err = hse_srv_req_async(ctx->dev, HSE_CHANNEL_ANY, &ctx->srv_desc, ctx,
				hse_rng_done);
	if (unlikely(err)) {
		mutex_unlock(&ctx->req_lock);
		dev_dbg(ctx->dev, "%s: request failed: %d\n", __func__, err);
	}
}

/**
 * hse_rng_read - generate random bytes of data into a supplied buffer
 * @rng: hwrng instance
 * @buf: destination buffer
 * @count: number of bytes, multiple of 4, more than 32 bytes, less than 2k
 * @wait: synchronous request flag, set by upper layer if it can wait for data
 *
 * If possible, get random data from internal cache and trigger a refill.
 * Otherwise, if the upper layer can wait, send a synchronous request to HSE.
 * HSE will always provide the exact number of bytes requested, up to 2k.
 *
 * Return: number of random bytes on success, -EINVAL for invalid parameter,
 *         -ENOMEM for DMA mapping failed, -EIO for service request error
 */
static int hse_rng_read(struct hwrng *rng, void *buf, size_t count, bool wait)
{
	struct hse_rng_ctx *ctx = (struct hse_rng_ctx *)rng->priv;
	struct hse_srv_desc srv_desc;
	dma_addr_t buf_dma;
	int err;

	count = min(count, HSE_MAX_RNG_SIZE);

	if (ctx->cache_idx > 0) {
		count = min(count, ctx->cache_idx);

		dma_sync_single_for_cpu(ctx->dev, ctx->cache_dma - count +
					ctx->cache_idx, count, DMA_FROM_DEVICE);
		memcpy(buf, &ctx->cache[ctx->cache_idx - count], count);
		ctx->cache_idx -= count;

		/* refill cache if depleted */
		hse_rng_refill_cache(rng);

		return count;
	}

	if (!wait) {
		hse_rng_refill_cache(rng);
		return 0;
	}

	if (unlikely(count < HSE_MIN_RNG_SIZE))
		return -EINVAL;

	buf_dma = dma_map_single(ctx->dev, buf, count, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(ctx->dev, buf_dma)))
		return -ENOMEM;

	srv_desc.srv_id = _get_rng_srv_id(ctx->dev);
	srv_desc.rng_req.rng_class = HSE_RNG_CLASS_PTG3;
	srv_desc.rng_req.random_num_len = count;
	srv_desc.rng_req.random_num = buf_dma;

	err = hse_srv_req_sync(ctx->dev, HSE_CHANNEL_ANY, &srv_desc);
	if (err)
		dev_dbg(ctx->dev, "%s: request failed: %d\n", __func__, err);

	dma_unmap_single(ctx->dev, buf_dma, count, DMA_FROM_DEVICE);

	return err ? -EIO : count;
}

/**
 * hse_rng_init - initialize RNG
 * @rng: hwrng instance
 */
static int hse_rng_init(struct hwrng *rng)
{
	struct hse_rng_ctx *ctx = (struct hse_rng_ctx *)rng->priv;

	mutex_init(&ctx->req_lock);

	ctx->srv_desc.srv_id = _get_rng_srv_id(ctx->dev);
	ctx->srv_desc.rng_req.rng_class = HSE_RNG_CLASS_PTG3;

	ctx->cache_dma = dma_map_single(ctx->dev, ctx->cache, HSE_RNG_CACHE_MAX,
					DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(ctx->dev, ctx->cache_dma)))
		return -ENOMEM;

	return 0;
}

/**
 * hse_rng_cleanup - RNG cleanup
 * @rng: hwrng instance
 */
static void hse_rng_cleanup(struct hwrng *rng)
{
	struct hse_rng_ctx *ctx = (struct hse_rng_ctx *)rng->priv;

	dma_unmap_single(ctx->dev, ctx->cache_dma, HSE_RNG_CACHE_MAX,
			 DMA_FROM_DEVICE);
}

static struct hwrng hse_rng = {
	.name = "hwrng-hse",
	.init = hse_rng_init,
	.cleanup = hse_rng_cleanup,
	.read = hse_rng_read,
	.quality = HSE_RNG_QUALITY,
};

/**
 * hse_rng_register - register RNG
 * @dev: HSE device
 */
void hse_rng_register(struct device *dev)
{
	struct hse_rng_ctx *ctx;
	int err;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ctx))
		return;
	ctx->dev = dev;

	hse_rng.priv = (unsigned long)ctx;

	err = devm_hwrng_register(dev, &hse_rng);
	if (err) {
		dev_err(dev, "failed to register %s: %d", hse_rng.name, err);
		return;
	}

	dev_info(dev, "registered %s\n", hse_rng.name);
}

/**
 * hse_rng_unregister - unregister RNG
 * @dev: HSE device
 */
void hse_rng_unregister(struct device *dev)
{
	devm_hwrng_unregister(dev, &hse_rng);

	dev_info(dev, "unregistered %s", hse_rng.name);
}
