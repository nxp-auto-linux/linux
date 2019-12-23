// SPDX-License-Identifier: BSD 3-clause
/*
 * NXP HSE Driver - Hardware True RNG Support
 *
 * This file contains hw_random framework support for HSE hardware true RNG.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/hw_random.h>

#include "hse-abi.h"
#include "hse-core.h"

#define HSE_RNG_QUALITY    1024u /* number of entropy bits per 1024 bits */

/**
 * struct hse_rng_ctx - hwrng context
 * @srv_desc: HSE service descriptor
 * @dev: HSE device
 * @req_lock: service descriptor mutex
 */
struct hse_rng_ctx {
	struct hse_srv_desc srv_desc;
	struct device *dev;
	struct mutex req_lock; /* descriptor mutex */
};

/**
 * hse_hwrng_read - read max bytes of data into buffer
 * @rng: hwrng instance
 * @data: destination buffer
 * @max: max bytes to read, multiple of 4 and >= 32 bytes
 * @wait: wait for data
 *
 * Though crypto API expects up to max bytes, HSE will always provide the
 * exact number of bytes requested or zero in case of any error.
 */
static int hse_hwrng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
	struct hse_rng_ctx *ctx = (struct hse_rng_ctx *)rng->priv;
	dma_addr_t srv_desc_dma, data_dma;
	int err;

	if (!mutex_trylock(&ctx->req_lock)) {
		dev_dbg(ctx->dev, "%s: request in progress\n", __func__);
		return 0;
	}

	data_dma = dma_map_single(ctx->dev, data, max, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(ctx->dev, data_dma)))
		return 0;

	ctx->srv_desc.srv_id = HSE_SRV_ID_GET_RANDOM_NUM;
	ctx->srv_desc.priority = HSE_SRV_PRIO_LOW;
	ctx->srv_desc.rng_req.rng_class = HSE_RNG_CLASS_PTG3;
	ctx->srv_desc.rng_req.random_num_len = max;
	ctx->srv_desc.rng_req.random_num = data_dma;

	srv_desc_dma = dma_map_single(ctx->dev, &ctx->srv_desc,
				      sizeof(ctx->srv_desc), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(ctx->dev, srv_desc_dma))) {
		dma_unmap_single(ctx->dev, data_dma, max, DMA_FROM_DEVICE);
		return 0;
	}

	err = hse_srv_req_sync(ctx->dev, HSE_CHANNEL_ANY, srv_desc_dma);
	if (unlikely(err))
		dev_dbg(ctx->dev, "%s: request failed: %d\n", __func__, err);

	dma_unmap_single(ctx->dev, srv_desc_dma, sizeof(ctx->srv_desc),
			 DMA_TO_DEVICE);
	dma_unmap_single(ctx->dev, data_dma, max, DMA_FROM_DEVICE);

	mutex_unlock(&ctx->req_lock);

	return !err ? max : 0;
}

static struct hwrng hse_rng = {
	.name = "hwrng-hse",
	.read = hse_hwrng_read,
	.quality = HSE_RNG_QUALITY,
};

/**
 * hse_hwrng_register - register random number generator
 * @dev: HSE device
 */
void hse_hwrng_register(struct device *dev)
{
	struct hse_rng_ctx *ctx;
	int err;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ctx))
		return;

	ctx->dev = dev;

	mutex_init(&ctx->req_lock);

	hse_rng.priv = (unsigned long)ctx;

	err = devm_hwrng_register(dev, &hse_rng);
	if (err) {
		dev_err(dev, "failed to register %s: %d", hse_rng.name, err);
		return;
	}

	dev_info(dev, "registered %s\n", hse_rng.name);
}
