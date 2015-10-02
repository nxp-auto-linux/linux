// SPDX-License-Identifier: GPL-2.0+
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 *
 * Copyright (c) 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * later as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/hw_random.h>
#include <linux/io.h>

#include "cse3.h"
#include "cse3_req.h"
#include "cse3_hw.h"


#define LOG_LEVEL		KERN_DEBUG

#define CSE3_MAJOR		42
#define CSE3_MINOR		0
#define NUM_MINORS		1
#define CSE3_NAME		"cse3"
#define NBITS			8

struct cse_device_data *cse_dev_ptr;

char *errmsg[35] = {
	"", /* padding */
	"", /* padding */
	/* CSE3 error codes, ecr between 0x02-0x0C */
	"Command sequence error",
	"Key not available",
	"Invalid key",
	"Empty key",
	"No secure boot",
	"Key write protected",
	"Key update error",
	"Random number seed not initialized",
	"Internal debug not allowed",
	"Command issued while busy",
	"System memory error",
	"", /* padding */
	"", /* padding */
	"", /* padding */
	/* CSE3 error codes, ecr between 0x10-0x22 */
	"Internal memory error",
	"Invalid command",
	"TRNG error",
	"CSE flash block error",
	"Internal command processor error",
	"Length error",
	"No valid key image",
	"No valid firmware",
	"No valid key block",
	"Invalid secure counter publish value",
	"Publish key image command not allowed",
	"Incorrect firmware",
	"Secure boot timeout",
	"Reserved",
	"Secure RAM start address mismatch",
	"Secure RAM operation not allowed",
	"Secure RAM authentication error",
	"Secure RAM key error",
	"Secure RAM invalid address",
};

static inline void cse_print_err(struct cse_device_data *dev, uint32_t err_code)
{
	dev_err(dev->device, "%s\n", errmsg[err_code]);
}

/**
 * Copy data from current request to hardware descriptor
 * Send request to the hardware
 * Returns error if the request wasn't submitted successfully
 */
static int cse_submit_request(cse_req_t *req)
{
	struct cse_device_data *dev = req->ctx->dev;
	struct cse_ldkey_request *key_req;
	struct cse_crypt_request *crypt_req;
	struct cse_cmac_request *cmac_req;
	struct cse_mp_request *mp_req;
	cse_desc_t *desc;

	dev->hw_desc = dma_alloc_coherent(dev->device, sizeof(cse_desc_t),
			&dev->hw_desc_phys, GFP_KERNEL);
	if (!dev->hw_desc) {
		dev_err(dev->device, "unable to alloc memory for command buffers.\n");
		return -ENOMEM;
	}
	desc = dev->hw_desc;

	if (req->flags & FLAG_LOAD_KEY) {
		key_req = (struct cse_ldkey_request *)req;
		memcpy(desc->m1, key_req->m1, M1_KEY_SIZE);
		memcpy(desc->m2, key_req->m2, M2_KEY_SIZE);
		memcpy(desc->m3, key_req->m3, M3_KEY_SIZE);

	} else if (req->flags & (FLAG_ENC|FLAG_DEC)) {
		crypt_req = (struct cse_crypt_request *)req;
		memcpy(desc->aes_key, req->ctx->aes_key, AES_KEY_SIZE);
		memcpy(desc->buffer_in, crypt_req->buffer_in,
				crypt_req->len_in);
		desc->len_in = crypt_req->len_in*NBITS;
		if (req->flags & FLAG_CBC)
			memcpy(desc->aes_iv, req->ctx->aes_iv, AES_KEY_SIZE);

	} else if (req->flags & (FLAG_GEN_MAC|FLAG_VER_MAC)) {
		cmac_req = (struct cse_cmac_request *)req;
		memcpy(desc->aes_key, req->ctx->aes_key, AES_KEY_SIZE);
		memcpy(desc->buffer_in, cmac_req->buffer_in,
				cmac_req->len_in);
		desc->len_in = cmac_req->len_in*NBITS;
		if (req->flags & FLAG_VER_MAC)
			memcpy(desc->mac, cmac_req->buffer_out, AES_MAC_SIZE);

	} else if (req->flags & FLAG_MP_COMP) {
		mp_req = (struct cse_mp_request *)req;
		memcpy(desc->buffer_in, mp_req->buffer_in, mp_req->len_in);
		desc->len_in = mp_req->len_in*NBITS;

	} else if (req->flags & FLAG_LOAD_PLKEY) {
		memcpy(desc->aes_key, req->ctx->aes_key, AES_KEY_SIZE);
	}

	/* Wait for memory consistency before sending request */
	smp_wmb();

	return cse_hw_comm(dev, req->flags, req->phase);
}

/**
 * If device not busy, submit the current request or
 * the first one in the queue. If it couldn't be submitted,
 * enqueue the current request
 */
int cse_handle_request(struct cse_device_data *dev, cse_req_t *req)
{
	int status = 0, ret = 0;
	cse_req_t *nextReq;

	spin_lock_bh(&dev->lock);
	/* Enqueue current request */
	if (req) {
		ret = cse_enqueue_request(&dev->queue, req);
		if (ret) {
			spin_unlock_bh(&dev->lock);
			return ret;
		}
	}

	/* Check if we can submit the next request */
	if (dev->flags & FLAG_BUSY) {
		spin_unlock_bh(&dev->lock);
		return ret;
	}
	nextReq = cse_dequeue_request(&dev->queue);
	if (nextReq) {
		dev->req = nextReq;
		set_state(&nextReq->state, FLAG_SUBMITTED);
		dev->flags |= FLAG_BUSY;
	}
	spin_unlock_bh(&dev->lock);

	if (!nextReq)
		return ret;

	status = cse_submit_request(nextReq);
	if (status) {
		nextReq->error = status;
		complete(&nextReq->complete);
	}

	return ret;
}

static int cse_cdev_open(struct inode *inode, struct file *file)
{
	cse_ctx_t *context;
	struct cse_device_data *dev =
		container_of(inode->i_cdev, struct cse_device_data, cdev);

	if (down_trylock(&dev->access))
		return -EBUSY;

	context = kzalloc(sizeof(cse_ctx_t), GFP_KERNEL);
	if (!context) {
		dev_err(dev->device, "failed to alloc mem for crypto context.\n");
		up(&dev->access);
		return -ENOMEM;
	}
	context->dev = dev;
	file->private_data = context;

	return 0;
}

static int cse_cdev_release(struct inode *inode, struct file *file)
{
	cse_ctx_t *context = (cse_ctx_t *) file->private_data;

	kfree(context);
	file->private_data = NULL;
	up(&context->dev->access);

	return 0;
}

/**
 * Deallocate currently running request, reset BUSY flag
 * and handle the next request
 */
void cse_finish_req(struct cse_device_data *dev, cse_req_t *req)
{
	/* TODO: depending on source, can use
	 * hw_desc directly for output
	 */
	if (IS_SUBMITTED(req->state) || IS_DONE(req->state)) {
		if (dev->hw_desc) {
			dma_free_coherent(dev->device, sizeof(cse_desc_t),
					dev->hw_desc, dev->hw_desc_phys);
			dev->hw_desc = NULL;
		}
		kfree(dev->req);
		dev->req = NULL;

		spin_lock_bh(&dev->lock);
		dev->flags &= ~FLAG_BUSY;
		spin_unlock_bh(&dev->lock);

		/** Submit the next request */
		cse_handle_request(dev, NULL);

	} else { /* failed before submission e.g. -EBUSY, -EINTR */
		kfree(req);
	}
}

/**
 * Cancel request on interrupt signal
 * Remove from queue and free
 * or ignore result if already submitted
 */
void cse_cancel_request(cse_req_t *req)
{
	uint8_t state;

	spin_lock_bh(&req->ctx->dev->lock);
	state = req->state;
	set_canceled(&req->state);
	/* Dequeue if still in queue */
	if (IS_INIT(req->state))
		cse_remove_request(&req->ctx->dev->queue, req);
	spin_unlock_bh(&req->ctx->dev->lock);

	/* The requests that are submitted but not finished yet
	 * will be freed in done_task
	 */
	if (IS_INIT(state) || IS_DONE(state))
		cse_finish_req(req->ctx->dev, req);
}

static long cse_cdev_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	cse_ctx_t *ctx = (cse_ctx_t *) file->private_data;

	int ret = 0;
	int remains;

	switch (cmd) {
	case CSE3_IOCTL_RND:
		ret = cse_ioctl_rnd(ctx, cmd, arg);
		break;
	case CSE3_IOCTL_ENC_CBC:
	case CSE3_IOCTL_ENC_ECB:
	case CSE3_IOCTL_DEC_CBC:
	case CSE3_IOCTL_DEC_ECB:
		ret = cse_ioctl_crypt(ctx, cmd, arg, 0);
		break;
	case CSE3_IOCTL_ENC_CBC_WK:
	case CSE3_IOCTL_ENC_ECB_WK:
	case CSE3_IOCTL_DEC_CBC_WK:
	case CSE3_IOCTL_DEC_ECB_WK:
		ret = cse_ioctl_crypt(ctx, cmd, arg, 1);
		break;
	case CSE3_IOCTL_LOAD_KEY:
		ret = cse_ioctl_load_key(ctx, cmd, arg);
		break;
	case CSE3_IOCTL_LOAD_PLKEY:
		ret = cse_ioctl_load_plkey(ctx, cmd, arg);
		break;
	case CSE3_IOCTL_CHECK_MAC:
		ret = cse_ioctl_cmac(ctx, cmd, arg, 0, 1);
		break;
	case CSE3_IOCTL_GEN_MAC:
		ret = cse_ioctl_cmac(ctx, cmd, arg, 0, 0);
		break;
	case CSE3_IOCTL_COMPRESS_MP:
		ret = cse_ioctl_comp(ctx, cmd, arg);
		break;
	case CSE3_IOCTL_SET_KEY:
		remains = copy_from_user(ctx->aes_key,
				(unsigned char __user *)arg, AES_KEY_SIZE);
		if (remains)
			ret = -EFAULT;
		break;
	case CSE3_IOCTL_SET_IV:
		remains = copy_from_user(ctx->aes_iv,
				(unsigned char __user *)arg, AES_KEY_SIZE);
		if (remains)
			ret = -EFAULT;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

/**
 * Tasklet job scheduled from irq
 * TODO: different done task based on source of request?
 */
static void cse_done_task(unsigned long data)
{
	uint8_t state;
	struct cse_crypt_request *crypt_req;
	struct cse_cmac_request *cmac_req;
	struct cse_mp_request *mp_req;
	struct cse_rval_request *rval_req;
	struct cse_device_data *dev = (struct cse_device_data *)data;
	cse_desc_t *desc = dev->hw_desc;

	if (dev->req->error)
		goto compl;

	/** Continue multi-phase request */
	if (dev->req->phase) {
		dev->req->phase--;
		cse_hw_comm(dev, dev->req->flags, dev->req->phase);
		return;
	}

	/** Copy result */
	if (dev->req->flags & (FLAG_ENC|FLAG_DEC)) {
		crypt_req = (struct cse_crypt_request *)dev->req;
		memcpy(crypt_req->buffer_out, desc->buffer_out,
				crypt_req->len_out);

	} else if (dev->req->flags & (FLAG_GEN_MAC|FLAG_VER_MAC)) {
		cmac_req = (struct cse_cmac_request *)dev->req;
		if (dev->req->flags & FLAG_GEN_MAC)
			memcpy(cmac_req->buffer_out, desc->mac, AES_MAC_SIZE);
		else
			cmac_req->status = readl(&dev->base->cse_param[4]);

	} else if (dev->req->flags & FLAG_MP_COMP) {
		mp_req = (struct cse_mp_request *)dev->req;
		memcpy(mp_req->buffer_out, desc->mp, MP_COMP_SIZE);

	} else if (dev->req->flags & FLAG_RND) {
		rval_req = (struct cse_rval_request *)dev->req;
		memcpy(rval_req->rval, desc->rval, RND_VAL_SIZE);
	}

compl:
	spin_lock(&dev->lock);
	state = dev->req->state;
	set_state(&dev->req->state, FLAG_DONE);
	spin_unlock(&dev->lock);

	if (IS_CANCELED(state)) {
		cse_finish_req(dev, dev->req);
	} else {
		/* TODO: call complete based on algo type */
		complete(&dev->req->complete);
	}
}

irqreturn_t cse_irq_handler(int irq_no, void *dev_id)
{
	struct cse_device_data *cse_dev = (struct cse_device_data *) dev_id;
	uint32_t status = readl(&cse_dev->base->cse_sr);
	uint32_t error = readl(&cse_dev->base->cse_ecr);
	uint32_t ctrl = readl(&cse_dev->base->cse_cr);

	/* Check status */
	if (status & CSE_SR_BSY || !cse_dev->req) {
		return IRQ_NONE;
	} else if (!error) {
		cse_dev->req->error = 0;
	} else {
		cse_print_err(cse_dev, error);
		cse_dev->req->error = -EINVAL;
	}

	/* Acknowledge interrupt */
	writel(CSE_IR_CIF, &cse_dev->base->cse_ir);
	/* If used, disable second key bank as a default state */
	writel((ctrl & ~CSE_CR_KBS), &cse_dev->base->cse_cr);
	tasklet_schedule(&cse_dev->done_task);

	return IRQ_HANDLED;
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
	kfree(ctx);
	up(&ctx->dev->access);

	return size;
}

static int cse_rng_data_read(struct hwrng *rng, u32 *data)
{
	return cse_rng_read(rng, data, RND_VAL_SIZE, 1);
}

static const struct file_operations cse_fops = {
	.owner = THIS_MODULE,
	.open = cse_cdev_open,
	.release = cse_cdev_release,
	.unlocked_ioctl = cse_cdev_ioctl,
};

static struct hwrng cse_rng = {
	.name		= "rng-cse",
	/* .cleanup	= cse_rng_cleanup, */
	.data_read	= cse_rng_data_read,
	.read		= cse_rng_read,
};

static struct platform_device_id cse3_platform_ids[] = {
	{ .name = "cse3-s32v234" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, cse3_platform_ids);

static const struct of_device_id cse3_dt_ids[] = {
	{ .compatible = "fsl,s32v234-cse3" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cse3_dt_ids);

static int cse_probe(struct platform_device *pdev)
{
	int err = 0;
	struct cse_device_data *cse_dev;
	struct resource *res;

	/** Allocate device data */
	cse_dev = devm_kzalloc(&pdev->dev, sizeof(struct cse_device_data),
			GFP_KERNEL);
	if (cse_dev == NULL) {
		dev_err(&pdev->dev, "unable to alloc data struct.\n");
		return -ENOMEM;
	}
	cse_dev->device = &pdev->dev;
	platform_set_drvdata(pdev, cse_dev);

	/* Request IRQ */
	cse_dev->irq = platform_get_irq(pdev, 0);
	if (cse_dev->irq < 0) {
		err = cse_dev->irq;
		dev_err(&pdev->dev, "failed to get irq resource.\n");
		goto out_irq;
	}

	err = devm_request_irq(&pdev->dev, cse_dev->irq, cse_irq_handler,
			0, dev_name(&pdev->dev), cse_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to request irq.\n");
		goto out_irq;
	}

	/** Request CSE3 memory */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		err = -ENXIO;
		dev_err(&pdev->dev, "failed to get memory resource.\n");
		goto out_io;
	}
	cse_dev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(cse_dev->base)) {
		err = PTR_ERR(cse_dev->base);
		goto out_io;
	}

	/** Enable interrupts */
	writel(CSE_IR_CIF, &cse_dev->base->cse_ir);
	writel(CSE_CR_CIE, &cse_dev->base->cse_cr);

	cse_dev_ptr = cse_dev;
	sema_init(&cse_dev->access, MAX_ACCESS);
	tasklet_init(&cse_dev->done_task, cse_done_task,
			(unsigned long)cse_dev);
	cse_init_queue(&cse_dev->queue, CSE_QUEUE_LEN);

	/** Register character device interface */
	err = register_chrdev_region(MKDEV(CSE3_MAJOR, CSE3_MINOR),
			NUM_MINORS, CSE3_NAME);
	if (err) {
		dev_err(&pdev->dev, "failed to register character device.\n");
		goto out_dev;
	}

	cse_dev->flags = 0;
	cdev_init(&cse_dev->cdev, &cse_fops);
	cdev_add(&cse_dev->cdev, MKDEV(CSE3_MAJOR, 0), 1);

	/** Register HW Random Number Generator API */
	if (hwrng_register(&cse_rng)) {
		dev_err(&pdev->dev, "failed to register hwrng.\n");
		goto out_dev;
	}

	return 0;

out_dev:
	tasklet_kill(&cse_dev->done_task);
out_io:
	devm_free_irq(&pdev->dev, cse_dev->irq, cse_dev);
out_irq:
	devm_kfree(&pdev->dev, cse_dev);

	return err;
}

static int cse_remove(struct platform_device *pdev)
{
	struct cse_device_data *cse_dev = platform_get_drvdata(pdev);

	hwrng_unregister(&cse_rng);
	cdev_del(&cse_dev->cdev);
	unregister_chrdev_region(MKDEV(CSE3_MAJOR, CSE3_MINOR), NUM_MINORS);

	cse_destroy_queue(&cse_dev->queue);
	cse_dev_ptr = NULL;

	tasklet_kill(&cse_dev->done_task);

	return 0;
}

static struct platform_driver cse_driver = {
	.probe      = cse_probe,
	.remove     = __exit_p(cse_remove),
	.driver     = {
		.name   = CSE3_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = cse3_dt_ids,
	},
};

static int __init cse_init(void)
{
	return platform_driver_probe(&cse_driver, cse_probe);
}

static void __exit cse_exit(void)
{
	platform_driver_unregister(&cse_driver);
}

module_init(cse_init);
module_exit(cse_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale");
MODULE_DESCRIPTION("CSE3 HW crypto driver");
