// SPDX-License-Identifier: GPL-2.0+
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 *
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * CSE3 Linux HWRng Interface
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

#include <linux/hw_random.h>

#include "cse3.h"
#include "cse3_req.h"
#include "cse3_rng.h"

static int cse_rng_copy_output
(struct cse_device_data *dev, struct cse_request *req)
{
	struct cse_rval_request *rval_req = (struct cse_rval_request *)dev->req;
	cse_desc_t *desc = dev->hw_desc;

	memcpy(rval_req->rval, desc->rval, RND_VAL_SIZE);

	return 0;
}

static void cse_rng_complete
(struct cse_device_data *dev, struct cse_request *req)
{
	complete(&req->complete);
}

static void cse_rng_init_ops(struct cse_request *req)
{
	req->copy_output = cse_rng_copy_output;
	req->comp = cse_rng_complete;
}

static int cse_rng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
	int size = 0;
	struct cse_rval_request *new_req;
	cse_ctx_t *ctx;

	if (!wait)
		return 0;

	if (down_trylock(&cse_dev_ptr->access))
		return -EBUSY;

	ctx = kzalloc(sizeof(cse_ctx_t), GFP_KERNEL);
	if (!ctx) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for crypto context.\n");
		up(&cse_dev_ptr->access);
		return -ENOMEM;
	}
	ctx->dev = cse_dev_ptr;

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	new_req->base.ctx = ctx;
	new_req->base.phase = 0;
	new_req->base.flags = FLAG_RND;
	cse_rng_init_ops(&new_req->base);
	init_completion(&new_req->base.complete);

	if (!cse_handle_request(ctx->dev, (cse_req_t *)new_req)) {

		if (wait_for_completion_interruptible(
					&new_req->base.complete)) {
			cse_cancel_request((cse_req_t *)new_req);
			goto out;
		} else if (!new_req->base.error) {
			size = max < RND_VAL_SIZE ? max : RND_VAL_SIZE;
			memcpy(data, new_req->rval, size);
		}

	}

	cse_finish_req(ctx->dev, (cse_req_t *)new_req);
out:
	up(&ctx->dev->access);
	kfree(ctx);

	return size;
}

static int cse_rng_data_read(struct hwrng *rng, u32 *data)
{
	return cse_rng_read(rng, data, RND_VAL_SIZE, 1);
}

static struct hwrng cse_rng = {
	.name		= "rng-cse",
	/* .cleanup	= cse_rng_cleanup, */
	.data_read	= cse_rng_data_read,
	.read		= cse_rng_read,
};

/**
 * Register HW Random Number Generator API
 */
void cse_register_rng(void)
{
	if (devm_hwrng_register(cse_dev_ptr->device, &cse_rng))
		dev_err(cse_dev_ptr->device, "failed to register hwrng.\n");

}
