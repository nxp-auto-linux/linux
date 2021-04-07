// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - User-space Driver Support
 *
 * This file contains the HSE user-space I/O driver support.
 *
 * Copyright 2021 NXP
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/uio_driver.h>

#include "hse-abi.h"
#include "hse-core.h"
#include "hse-mu.h"

#define HSE_SHARED_RAM_ADDR    0x22C00000ul /* HSE shared RAM */
#define HSE_SHARED_RAM_SIZE    0x4000u /* 16k */

/**
 * enum hse_uio_irqctl - HSE UIO interrupt control commands
 * @HSE_UIO_DISABLE_RX_IRQ_CMD: disable HSE MU rx interrupt
 * @HSE_UIO_ENABLE_RX_IRQ_CMD: enable HSE MU rx interrupt
 * @HSE_UIO_DISABLE_TX_IRQ_CMD: disable HSE MU tx interrupt
 * @HSE_UIO_ENABLE_TX_IRQ_CMD: enable HSE MU tx interrupt
 * @HSE_UIO_DISABLE_EVT_IRQ_CMD: disable HSE MU sys event interrupt
 * @HSE_UIO_ENABLE_EVT_IRQ_CMD: enable HSE MU sys event interrupt
 * @HSE_UIO_CLEAR_EVT_IRQ_CMD: clear HSE MU sys event interrupt
 */
enum hse_uio_irqctl {
	HSE_UIO_DISABLE_RX_IRQ_CMD = 0u,
	HSE_UIO_ENABLE_RX_IRQ_CMD = 1u,
	HSE_UIO_DISABLE_TX_IRQ_CMD = 2u,
	HSE_UIO_ENABLE_TX_IRQ_CMD = 3u,
	HSE_UIO_DISABLE_EVT_IRQ_CMD = 4u,
	HSE_UIO_ENABLE_EVT_IRQ_CMD = 5u,
	HSE_UIO_CLEAR_EVT_IRQ_CMD = 6u,
};

/**
 * struct hse_uio_shm - HSE shared RAM layout
 * @ready[n]: reply ready on channel n
 * @reply[n]: service response on channel n
 */
struct hse_uio_shm {
	u8 ready[HSE_NUM_CHANNELS];
	u32 reply[HSE_NUM_CHANNELS];
};

/**
 * struct hse_uio_priv - HSE UIO component private data
 * @dev: HSE device
 * @mu: MU instance handle
 * @info: UIO device info
 * @refcnt: reference counter
 * @shm: pointer to HSE shared RAM
 */
struct hse_uio_priv {
	struct device *dev;
	void *mu;
	struct uio_info info;
	atomic_t refcnt;
	struct hse_uio_shm __iomem *shm;
};

/**
 * hse_uio_open - open /dev/uioX device
 * @info: UIO device info
 * @inode: inode, not used
 *
 * Protects against multiple driver instances using UIO support.
 */
static int hse_uio_open(struct uio_info *info, struct inode *inode)
{
	struct hse_uio_priv *priv = info->priv;

	if (!atomic_dec_and_test(&priv->refcnt)) {
		dev_err(priv->dev, "%s device already in use\n", info->name);
		atomic_inc(&priv->refcnt);

		return -EBUSY;
	}

	return 0;
}

/**
 * hse_uio_release - release /dev/uioX device
 * @info: UIO device info
 * @inode: inode, not used
 *
 * Protects against multiple driver instances using UIO support.
 */
static int hse_uio_release(struct uio_info *info, struct inode *inode)
{
	struct hse_uio_priv *priv = info->priv;

	atomic_inc(&priv->refcnt);

	return 0;
}

/**
 * hse_uio_irqcontrol - control HSE MU interrupt status by writing to /dev/uioX
 * @info: UIO device info
 * @cmd: interrupt control command
 */
static int hse_uio_irqcontrol(struct uio_info *info, int cmd)
{
	struct hse_uio_priv *priv = info->priv;

	switch (cmd) {
	case HSE_UIO_DISABLE_RX_IRQ_CMD:
		hse_mu_irq_disable(priv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_ENABLE_RX_IRQ_CMD:
		hse_mu_irq_enable(priv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_DISABLE_TX_IRQ_CMD:
		hse_mu_irq_disable(priv->mu, HSE_INT_ACK_REQUEST,
				   HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_ENABLE_TX_IRQ_CMD:
		hse_mu_irq_enable(priv->mu, HSE_INT_ACK_REQUEST,
				  HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_DISABLE_EVT_IRQ_CMD:
		hse_mu_irq_enable(priv->mu, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_ENABLE_EVT_IRQ_CMD:
		hse_mu_irq_enable(priv->mu, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_CLEAR_EVT_IRQ_CMD:
		hse_mu_irq_clear(priv->mu, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);
		break;
	default:
		break;
	}

	return 0;
}

/**
 * hse_uio_notify - notify upper layer of HSE reply
 * @uio: UIO device handle
 * @channel: channel index
 * @srv_rsp: service response
 */
void hse_uio_notify(void *uio, u8 channel, u32 srv_rsp)
{
	struct uio_info *info = uio;
	struct hse_uio_priv *priv = info->priv;

	priv->shm->ready[channel] = 1;
	priv->shm->reply[channel] = srv_rsp;

	uio_event_notify(uio);
}

/**
 * hse_uio_register - initialize and register UIO device
 * @dev: HSE device
 * @mu: MU instance handle, for interrupt control
 *
 * Return: UIO device handle on success, error code otherwise
 */
void *hse_uio_register(struct device *dev, void *mu)
{
	struct hse_uio_priv *priv;
	struct resource *reg;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (IS_ERR_OR_NULL(priv))
		return ERR_PTR(-ENOMEM);
	priv->dev = dev;
	priv->mu = mu;

	atomic_set(&priv->refcnt, 1);

	/* get HSE MU register space info from device tree */
	reg = platform_get_resource(to_platform_device(dev), IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(reg))
		return ERR_PTR(-ENODEV);

	/* expose HSE MU register space to upper layer */
	priv->info.mem[0].name = "HSE MU registers";
	priv->info.mem[0].addr = reg->start;
	priv->info.mem[0].size = resource_size(reg);
	priv->info.mem[0].memtype = UIO_MEM_PHYS;

	priv->info.version = "0.8.5";
	priv->info.name = "HSE UIO driver";

	priv->info.open = hse_uio_open;
	priv->info.release = hse_uio_release;

	priv->info.irq = UIO_IRQ_CUSTOM;
	priv->info.irqcontrol = hse_uio_irqcontrol;
	priv->info.priv = priv;

	/* map HSE shared RAM area */
	priv->shm = devm_ioremap(dev, HSE_SHARED_RAM_ADDR,
					 HSE_SHARED_RAM_SIZE);
	if (IS_ERR_OR_NULL(priv->shm))
		return ERR_PTR(-ENOMEM);

	/* expose HSE shared RAM to upper layer */
	priv->info.mem[1].name = "HSE shared RAM";
	priv->info.mem[1].addr = HSE_SHARED_RAM_ADDR;
	priv->info.mem[1].internal_addr = priv->shm;
	priv->info.mem[1].size = HSE_SHARED_RAM_SIZE;
	priv->info.mem[1].memtype = UIO_MEM_PHYS;

	err = uio_register_device(dev, &priv->info);
	if (err) {
		dev_err(dev, "failed to register UIO device: %d\n", err);
		return ERR_PTR(err);
	}

	dev_info(dev, "successfully registered UIO device\n");

	return &priv->info;
}
