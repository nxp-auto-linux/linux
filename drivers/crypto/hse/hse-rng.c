// SPDX-License-Identifier: BSD 3-clause
/*
 * NXP HSE Driver - Hardware True RNG Support
 *
 * This file contains hw_random framework support for HSE hardware true RNG.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/crypto.h>
#include <linux/hw_random.h>

#include "hse.h"
#include "hse-abi.h"
#include "hse-mu.h"

/* 1-1024, number of entropy bits per 1024 bits, 0 for unknown */
#define HSE_RNG_QUALITY 1024u

/**
 * struct hse_rng_ctx - hwrng context
 * @srv_desc: HSE service descriptor
 * @dev: HSE device
 * @mu_inst: MU instance
 */
struct hse_rng_ctx {
	struct hse_srv_desc srv_desc;
	struct device *dev;
	void *mu_inst;
};

/**
 * hse_hwrng_read - read max bytes of data into buffer
 * @rng: hwrng instance
 * @data: destination buffer
 * @max: max bytes to read, multiple of 4 and >= 32 bytes
 * @wait: wait for data
 *
 * Though crypto API expects up to max bytes, HSE will always provide the
 * exact number of bytes requested.
 */
static int hse_hwrng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
	struct hse_rng_ctx *ctx = (struct hse_rng_ctx *)rng->priv;
	int err;
	u32 srv_desc_addr = lower_32_bits(hse_addr(&ctx->srv_desc));

	ctx->srv_desc.srv_id = HSE_SRV_ID_GET_RANDOM_NUM;
	ctx->srv_desc.priority = HSE_SRV_PRIO_MED;
	ctx->srv_desc.rng_req.rng_class = HSE_RNG_CLASS_PTG3;
	ctx->srv_desc.rng_req.random_num_len = max;
	ctx->srv_desc.rng_req.random_num = hse_addr(data);

	err = hse_mu_sync_req(ctx->mu_inst, HSE_ANY_CHANNEL, srv_desc_addr);
	if (unlikely(err)) {
		dev_dbg(ctx->dev, "%s: request failed: %d\n", __func__, err);
		return 0;
	}

	return max;
}

static struct hwrng hse_rng = {
	.name = "hwrng-hse",
	.read = hse_hwrng_read,
	.quality = HSE_RNG_QUALITY,
};

/**
 * hse_hwrng_register - register hwrng
 * @dev: HSE device
 */
void hse_hwrng_register(struct device *dev)
{
	struct hse_drvdata *drvdata = dev_get_drvdata(dev);
	struct hse_rng_ctx *ctx;
	const char *name = hse_rng.name;
	int err = 0;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ctx)) {
		err = -ENOMEM;
		return;
	}

	ctx->dev = dev;
	ctx->mu_inst = drvdata->mu_inst;

	hse_rng.priv = (unsigned long)ctx;

	err = devm_hwrng_register(dev, &hse_rng);
	if (err) {
		dev_err(dev, "failed to register %s: %d", name, err);
		return;
	}

	dev_info(dev, "registered %s\n", name);
}
