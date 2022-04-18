// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE UIO Driver
 *
 * This file contains the HSE user-space I/O driver support.
 *
 * Copyright 2021-2022 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/uio_driver.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>

#define HSE_REGS_NAME       "hse-" CONFIG_UIO_NXP_HSE_MU "-regs"
#define HSE_DESC_NAME       "hse-" CONFIG_UIO_NXP_HSE_MU "-desc"
#define HSE_RX_IRQ_NAME     "hse-" CONFIG_UIO_NXP_HSE_MU "-rx"
#define HSE_ERR_IRQ_NAME    "hse-" CONFIG_UIO_NXP_HSE_MU "-err"

#define HSE_NUM_CHANNELS    16u /* number of available service channels */

#define HSE_CHANNEL_INV    0xFFu /* invalid acquired service channel index */
#define HSE_CH_MASK_ALL    0x0000FFFFul /* all available channels irq mask */

#define HSE_STATUS_MASK     0xFFFF0000ul /* HSE global status FSR mask */

#define HSE_EVT_MASK_ERR     0x000000FFul /* fatal error GSR mask */
#define HSE_EVT_MASK_WARN    0x0000FF00ul /* warning GSR mask */
#define HSE_EVT_MASK_INTL    0xFFFF0000ul /* NXP internal flags GSR mask */
#define HSE_EVT_MASK_ALL     0xFFFFFFFFul /* all events GSR mask */

/**
 * enum hse_uio_map_idx - HSE UIO mapping index
 * @HSE_UIO_MAP_REGS: MU hardware register space
 * @HSE_UIO_MAP_DESC: service descriptor space
 * @HSE_UIO_MAP_INTL: driver internal shared memory
 * @HSE_UIO_MAP_RMEM: reserved DMA-able memory range
 */
enum hse_uio_map_idx {
	HSE_UIO_MAP_REGS = 0u,
	HSE_UIO_MAP_DESC = 1u,
	HSE_UIO_MAP_INTL = 2u,
	HSE_UIO_MAP_RMEM = 3u,
};

/**
 * enum hse_irq_type - HSE interrupt type
 * @HSE_INT_ACK_REQUEST: TX Interrupt, triggered when HSE acknowledged the
 *                       service request and released the service channel
 * @HSE_INT_RESPONSE: RX Interrupt, triggered when HSE wrote the response
 * @HSE_INT_SYS_EVENT: General Purpose Interrupt, triggered when HSE sends
 *                     a system event, generally an error notification
 */
enum hse_irq_type {
	HSE_INT_ACK_REQUEST = 0u,
	HSE_INT_RESPONSE = 1u,
	HSE_INT_SYS_EVENT = 2u,
};

/**
 * struct hse_uio_intl - driver internal shared memory layout
 * @ready[n]: reply ready on channel n
 * @reply[n]: service response on channel n
 * @event: HSE system event mask
 */
struct hse_uio_intl {
	u8 ready[HSE_NUM_CHANNELS];
	u32 reply[HSE_NUM_CHANNELS];
	u32 event;
} __packed;

/**
 * struct hse_mu_regs - HSE Messaging Unit Registers
 * @ver: Version ID Register, offset 0x0
 * @par: Parameter Register, offset 0x4
 * @cr: Control Register, offset 0x8
 * @sr: Status Register, offset 0xC
 * @fcr: Flag Control Register, offset 0x100
 * @fsr: Flag Status Register, offset 0x104
 * @gier: General Interrupt Enable Register, offset 0x110
 * @gcr: General Control Register, offset 0x114
 * @gsr: General Status Register, offset 0x118
 * @tcr: Transmit Control Register, offset 0x120
 * @tsr: Transmit Status Register, offset 0x124
 * @rcr: Receive Control Register, offset 0x128
 * @rsr: Receive Status Register, offset 0x12C
 * @tr[n]: Transmit Register n, offset 0x200 + 4*n
 * @rr[n]: Receive Register n, offset 0x280 + 4*n
 */
struct hse_mu_regs {
	const u32 ver;
	const u32 par;
	u32 cr;
	u32 sr;
	u8 reserved0[240]; /* 0xF0 */
	u32 fcr;
	const u32 fsr;
	u8 reserved1[8]; /* 0x8 */
	u32 gier;
	u32 gcr;
	u32 gsr;
	u8 reserved2[4]; /* 0x4 */
	u32 tcr;
	const u32 tsr;
	u32 rcr;
	const u32 rsr;
	u8 reserved3[208]; /* 0xD0 */
	u32 tr[16];
	u8 reserved4[64]; /* 0x40 */
	const u32 rr[16];
};

/**
 * struct hse_uio_drvdata - HSE UIO driver private data
 * @dev: HSE UIO device
 * @info: UIO device info
 * @refcnt: open/close reference counter
 * @regs: MU register space base virtual address
 * @desc: service descriptor space base virtual address
 * @intl: driver internal shared memory base address
 * @rmem: driver DMA-able reserved memory range
 * @reg_lock: spinlock preventing concurrent register access
 */
struct hse_uio_drvdata {
	struct device *dev;
	struct uio_info info;
	refcount_t refcnt;
	struct hse_mu_regs __iomem *regs;
	void __iomem *desc;
	struct hse_uio_intl *intl;
	void __iomem *rmem;
	spinlock_t reg_lock; /* covers irq enable/disable */
};

/**
 * enum hse_uio_irqctl - HSE UIO interrupt control commands
 * @HSE_UIO_DISABLE_ALL_IRQ_CMD: disable all HSE MU interrupts
 * @HSE_UIO_DISABLE_RX_IRQ_CMD: disable HSE MU rx interrupt
 * @HSE_UIO_ENABLE_RX_IRQ_CMD: enable HSE MU rx interrupt
 * @HSE_UIO_DISABLE_TX_IRQ_CMD: disable HSE MU tx interrupt
 * @HSE_UIO_ENABLE_TX_IRQ_CMD: enable HSE MU tx interrupt
 * @HSE_UIO_DISABLE_EVT_IRQ_CMD: disable HSE MU sys event interrupt
 * @HSE_UIO_ENABLE_EVT_IRQ_CMD: enable HSE MU sys event interrupt
 * @HSE_UIO_CLEAR_EVT_IRQ_CMD: clear HSE MU sys event interrupt
 */
enum hse_uio_irqctl {
	HSE_UIO_DISABLE_ALL_IRQ_CMD = 0,
	HSE_UIO_ENABLE_ALL_IRQ_CMD = 1,
	HSE_UIO_DISABLE_RX_IRQ_CMD = 2,
	HSE_UIO_ENABLE_RX_IRQ_CMD = 3,
	HSE_UIO_DISABLE_TX_IRQ_CMD = 4,
	HSE_UIO_ENABLE_TX_IRQ_CMD = 5,
	HSE_UIO_DISABLE_EVT_IRQ_CMD = 6,
	HSE_UIO_ENABLE_EVT_IRQ_CMD = 7,
	HSE_UIO_CLEAR_EVT_IRQ_CMD = 8,
};

/**
 * hse_uio_check_status - check the HSE global status
 * @dev: HSE UIO device
 *
 * Return: 16 MSB of MU instance FSR
 */
static u16 hse_uio_check_status(struct device *dev)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	u32 fsrval;

	fsrval = ioread32(&drv->regs->fsr);
	fsrval = (fsrval & HSE_STATUS_MASK) >> 16u;

	return (u16)fsrval;
}

/**
 * hse_uio_check_event - check for HSE system events
 * @dev: HSE UIO device
 *
 * Return: HSE system event mask
 */
static u32 hse_uio_check_event(struct device *dev)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);

	return ioread32(&drv->regs->gsr);
}

/**
 * hse_uio_irq_enable - enable a specific type of interrupt using a mask
 * @dev: HSE UIO device
 * @irq_type: interrupt type
 * @irq_mask: interrupt mask
 */
static void hse_uio_irq_enable(struct device *dev, enum hse_irq_type irq_type,
			       u32 irq_mask)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	void __iomem *regaddr;
	unsigned long flags;

	switch (irq_type) {
	case HSE_INT_ACK_REQUEST:
		regaddr = &drv->regs->tcr;
		irq_mask &= HSE_CH_MASK_ALL;
		break;
	case HSE_INT_RESPONSE:
		regaddr = &drv->regs->rcr;
		irq_mask &= HSE_CH_MASK_ALL;
		break;
	case HSE_INT_SYS_EVENT:
		regaddr = &drv->regs->gier;
		irq_mask &= HSE_EVT_MASK_ALL;
		break;
	default:
		return;
	}

	spin_lock_irqsave(&drv->reg_lock, flags);

	iowrite32(ioread32(regaddr) | irq_mask, regaddr);

	spin_unlock_irqrestore(&drv->reg_lock, flags);
}

/**
 * hse_uio_irq_disable - disable a specific type of interrupt using a mask
 * @dev: HSE UIO device
 * @irq_type: interrupt type
 * @irq_mask: interrupt mask
 */
static void hse_uio_irq_disable(struct device *dev, enum hse_irq_type irq_type,
				u32 irq_mask)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	void __iomem *regaddr;
	unsigned long flags;

	switch (irq_type) {
	case HSE_INT_ACK_REQUEST:
		regaddr = &drv->regs->tcr;
		irq_mask &= HSE_CH_MASK_ALL;
		break;
	case HSE_INT_RESPONSE:
		regaddr = &drv->regs->rcr;
		irq_mask &= HSE_CH_MASK_ALL;
		break;
	case HSE_INT_SYS_EVENT:
		regaddr = &drv->regs->gier;
		irq_mask &= HSE_EVT_MASK_ALL;
		break;
	default:
		return;
	}

	spin_lock_irqsave(&drv->reg_lock, flags);

	iowrite32(ioread32(regaddr) & ~irq_mask, regaddr);

	spin_unlock_irqrestore(&drv->reg_lock, flags);
}

/**
 * hse_uio_irq_clear - clear a pending general purpose interrupt using a mask
 * @dev: HSE UIO device
 * @irq_type: interrupt type
 * @irq_mask: interrupt mask
 *
 * Only general purpose interrupts can be cleared. TX and RX irq lines, if
 * enabled, will remain asserted until the appropriate MU register is read.
 */
static void hse_uio_irq_clear(struct device *dev, enum hse_irq_type irq_type,
			      u32 irq_mask)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);

	if (unlikely(irq_type != HSE_INT_SYS_EVENT))
		return;

	iowrite32(irq_mask & HSE_EVT_MASK_ALL, &drv->regs->gsr);
}

/**
 * hse_uio_irqcontrol - control HSE MU interrupt status by writing to /dev/uioX
 * @info: UIO device info
 * @cmd: interrupt control command
 */
static int hse_uio_irqcontrol(struct uio_info *info, s32 cmd)
{
	struct hse_uio_drvdata *drv = info->priv;
	struct device *dev = drv->dev;

	switch (cmd) {
	case HSE_UIO_DISABLE_ALL_IRQ_CMD:
		hse_uio_irq_disable(dev, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
		hse_uio_irq_disable(dev, HSE_INT_ACK_REQUEST, HSE_CH_MASK_ALL);
		hse_uio_irq_disable(dev, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_ENABLE_ALL_IRQ_CMD:
		hse_uio_irq_enable(dev, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
		hse_uio_irq_enable(dev, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_DISABLE_RX_IRQ_CMD:
		hse_uio_irq_disable(dev, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_ENABLE_RX_IRQ_CMD:
		hse_uio_irq_enable(dev, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_DISABLE_TX_IRQ_CMD:
		hse_uio_irq_disable(dev, HSE_INT_ACK_REQUEST, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_ENABLE_TX_IRQ_CMD:
		hse_uio_irq_enable(dev, HSE_INT_ACK_REQUEST, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_DISABLE_EVT_IRQ_CMD:
		hse_uio_irq_enable(dev, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_ENABLE_EVT_IRQ_CMD:
		hse_uio_irq_enable(dev, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);
		break;
	case HSE_UIO_CLEAR_EVT_IRQ_CMD:
		hse_uio_irq_clear(dev, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * hse_uio_next_pending_channel - find the next channel with pending message
 * @dev: HSE UIO device
 *
 * Return: channel index, HSE_CHANNEL_INV if no message pending
 */
static u8 hse_uio_next_pending_channel(struct device *dev)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	u32 rsrval = ioread32(&drv->regs->rsr) & HSE_CH_MASK_ALL;

	if (!ffs(rsrval))
		return HSE_CHANNEL_INV;

	return ffs(rsrval) - 1;
}

/**
 * hse_uio_msg_pending - check if a service request response is pending
 * @dev: HSE UIO device
 * @channel: channel index
 *
 * Return: true for response ready, false otherwise
 */
static bool hse_uio_msg_pending(struct device *dev, u8 channel)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	u32 rsrval;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return false;

	rsrval = ioread32(&drv->regs->rsr) & BIT(channel);
	if (!rsrval)
		return false;

	return true;
}

/**
 * hse_uio_msg_recv - read a message received over MU (non-blocking)
 * @dev: HSE UIO device
 * @channel: channel index
 * @msg: output message
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -ENOMSG for no reply pending on selected channel
 */
static int hse_uio_msg_recv(struct device *dev, u8 channel, u32 *msg)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);

	if (unlikely(!dev || !msg))
		return -EINVAL;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return -ECHRNG;

	if (unlikely(!hse_uio_msg_pending(dev, channel)))
		return -ENOMSG;

	*msg = ioread32(&drv->regs->rr[channel]);

	return 0;
}

/**
 * hse_uio_open - open /dev/uioX device
 * @info: UIO device info
 * @inode: inode, not used
 *
 * Protects against multiple driver instances using UIO support.
 */
static int hse_uio_open(struct uio_info *info, struct inode *inode)
{
	struct hse_uio_drvdata *drv = info->priv;

	if (!refcount_dec_if_one(&drv->refcnt)) {
		dev_err(drv->dev, "device %s already in use\n", info->name);

		return -EBUSY;
	}

	dev_info(drv->dev, "device %s open\n", info->name);

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
	struct hse_uio_drvdata *drv = info->priv;

	refcount_set(&drv->refcnt, 1);

	dev_info(drv->dev, "device %s released\n", info->name);

	return 0;
}

/**
 * hse_uio_rx_dispatcher - deferred handler for HSE_INT_RESPONSE type interrupts
 * @irq: interrupt line
 * @dev: HSE UIO device
 */
static irqreturn_t hse_uio_rx_dispatcher(int irq, void *dev)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	u8 channel = hse_uio_next_pending_channel(dev);
	u32 srv_rsp;
	int err;

	while (channel != HSE_CHANNEL_INV) {
		/* handle reply */
		err = hse_uio_msg_recv(dev, channel, &srv_rsp);
		if (likely(!err)) {
			drv->intl->reply[channel] = srv_rsp;
			drv->intl->ready[channel] = 1;

			/* notify upper layer */
			uio_event_notify(&drv->info);
		}

		channel = hse_uio_next_pending_channel(dev);
	}

	return IRQ_HANDLED;
}

/**
 * hse_uio_evt_dispatcher - deferred handler for HSE_INT_SYS_EVENT interrupts
 * @irq: interrupt line
 * @dev: HSE device
 *
 * In case a fatal error has been reported, all MU interfaces are disabled
 * and communication with HSE terminated. Therefore, all interrupts are
 * disabled and a notification with the event mask is sent to the upper layer.
 */
static irqreturn_t hse_uio_evt_dispatcher(int irq, void *dev)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	u32 event = hse_uio_check_event(dev);

	if (event & HSE_EVT_MASK_WARN) {
		dev_warn(dev, "warning, event mask 0x%08x\n", event);
		hse_uio_irq_clear(dev, HSE_INT_SYS_EVENT, HSE_EVT_MASK_WARN);

		return IRQ_HANDLED;
	}

	/* disable all interrupt sources */
	hse_uio_irqcontrol(&drv->info, HSE_UIO_DISABLE_ALL_IRQ_CMD);

	dev_crit(dev, "fatal error, event mask 0x%08x\n", event);
	hse_uio_irq_clear(dev, HSE_INT_SYS_EVENT, HSE_EVT_MASK_ERR);

	/* notify upper layer */
	drv->intl->event = event;
	uio_event_notify(&drv->info);

	dev_crit(dev, "communication terminated, reset system to recover\n");

	return IRQ_HANDLED;
}

static int hse_uio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hse_uio_drvdata *drv;
	struct resource *res;
	struct device_node *rmem_node;
	struct reserved_mem *rmem;
	int irq, err;
	u16 status;

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (IS_ERR_OR_NULL(drv))
		return -ENOMEM;
	drv->dev = dev;
	platform_set_drvdata(pdev, drv);

	/* map hardware register space */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, HSE_REGS_NAME);
	drv->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(drv->regs)) {
		dev_err(dev, "failed to map %s @%pR\n", HSE_REGS_NAME, res);
		return -ENXIO;
	}

	/* check firmware status */
	status = hse_uio_check_status(dev);
	if (unlikely(!status)) {
		dev_warn(dev, "firmware not found\n");
		return -ENODEV;
	}

	/* expose HSE MU register space to upper layer */
	drv->info.mem[HSE_UIO_MAP_REGS].name = "hse-mu-registers";
	drv->info.mem[HSE_UIO_MAP_REGS].addr = (uintptr_t)res->start;
	drv->info.mem[HSE_UIO_MAP_REGS].size = resource_size(res);
	drv->info.mem[HSE_UIO_MAP_REGS].memtype = UIO_MEM_PHYS;

	drv->info.version = "1.0";
	drv->info.name = "hse-uio";

	drv->info.open = hse_uio_open;
	drv->info.release = hse_uio_release;

	drv->info.irq = UIO_IRQ_CUSTOM;
	drv->info.irqcontrol = hse_uio_irqcontrol;
	drv->info.priv = drv;

	/* map service descriptor space */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, HSE_DESC_NAME);
	drv->desc = devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(drv->desc)) {
		dev_err(dev, "failed to map %s @%pR\n", HSE_DESC_NAME, res);
		return -ENXIO;
	}

	/* expose service descriptor space to upper layer */
	drv->info.mem[HSE_UIO_MAP_DESC].name = "hse-service-descriptors";
	drv->info.mem[HSE_UIO_MAP_DESC].addr = (uintptr_t)res->start;
	drv->info.mem[HSE_UIO_MAP_DESC].internal_addr = drv->desc;
	drv->info.mem[HSE_UIO_MAP_DESC].size = resource_size(res);
	drv->info.mem[HSE_UIO_MAP_DESC].memtype = UIO_MEM_PHYS;

	/* alloc internal shared memory */
	drv->intl = devm_kzalloc(dev, PAGE_SIZE, GFP_KERNEL);
	if (IS_ERR_OR_NULL(drv->intl))
		return -ENOMEM;

	/* expose driver internal memory to upper layer */
	drv->info.mem[HSE_UIO_MAP_INTL].name = "hse-driver-internal";
	drv->info.mem[HSE_UIO_MAP_INTL].addr = (uintptr_t)drv->intl;
	drv->info.mem[HSE_UIO_MAP_INTL].size = PAGE_SIZE;
	drv->info.mem[HSE_UIO_MAP_INTL].memtype = UIO_MEM_LOGICAL;

	/* map HSE reserved memory */
	rmem_node = of_parse_phandle(dev->of_node->parent, "memory-region", 0);
	if (!rmem_node) {
		dev_err(dev, "reserved memory-region node not found\n");
		return -ENXIO;
	}

	rmem = of_reserved_mem_lookup(rmem_node);
	if (!rmem) {
		dev_err(dev, "reserved memory-region lookup failed\n");
		return -ENXIO;
	}
	of_node_put(rmem_node);

	drv->rmem = devm_ioremap(dev, rmem->base, rmem->size);
	if (IS_ERR_OR_NULL(drv->rmem))
		return -EINVAL;
	/* workaround: use reserved memory as internal */
	memcpy(&drv->intl, &drv->rmem, sizeof(drv->intl));

	/* expose HSE reserved memory to upper layer */
	drv->info.mem[HSE_UIO_MAP_RMEM].name = "hse-reserved-memory";
	drv->info.mem[HSE_UIO_MAP_RMEM].addr = (uintptr_t)rmem->base;
	drv->info.mem[HSE_UIO_MAP_RMEM].internal_addr = drv->rmem;
	drv->info.mem[HSE_UIO_MAP_RMEM].size = rmem->size;
	drv->info.mem[HSE_UIO_MAP_RMEM].memtype = UIO_MEM_PHYS;

	refcount_set(&drv->refcnt, 1);
	spin_lock_init(&drv->reg_lock);

	/* disable all interrupt sources */
	hse_uio_irqcontrol(&drv->info, HSE_UIO_DISABLE_ALL_IRQ_CMD);

	/* register RX interrupt handler */
	irq = platform_get_irq_byname(pdev, HSE_RX_IRQ_NAME);
	err = devm_request_threaded_irq(dev, irq, NULL, hse_uio_rx_dispatcher,
					IRQF_ONESHOT, HSE_RX_IRQ_NAME, dev);
	if (unlikely(err)) {
		dev_err(dev, "failed to register %s irq, line %d\n",
			HSE_RX_IRQ_NAME, irq);
		return -ENXIO;
	}

	/* register SYS_EVENT interrupt handler */
	irq = platform_get_irq_byname(pdev, HSE_ERR_IRQ_NAME);
	err = devm_request_threaded_irq(dev, irq, NULL, hse_uio_evt_dispatcher,
					IRQF_ONESHOT, HSE_ERR_IRQ_NAME, dev);
	if (unlikely(err)) {
		dev_err(dev, "failed to register %s irq, line %d\n",
			HSE_ERR_IRQ_NAME, irq);
		return -ENXIO;
	}

	/* register device */
	err = uio_register_device(dev, &drv->info);
	if (err) {
		dev_err(dev, "failed to register UIO device: %d\n", err);
		return err;
	}

	/* enable RX and system event notifications */
	hse_uio_irqcontrol(&drv->info, HSE_UIO_ENABLE_ALL_IRQ_CMD);

	dev_info(dev, "successfully registered device\n");

	return 0;
}

static int hse_uio_remove(struct platform_device *pdev)
{
	struct hse_uio_drvdata *drv = platform_get_drvdata(pdev);

	uio_unregister_device(&drv->info);

	return 0;
}

static const struct of_device_id hse_uio_of_match[] = {
	{
		.name = CONFIG_UIO_NXP_HSE_MU,
		.compatible = "nxp,s32cc-hse",
	}, {}
};
MODULE_DEVICE_TABLE(of, hse_uio_of_match);

static struct platform_driver hse_uio_driver = {
	.driver = {
		.name = "hse-uio",
		.of_match_table	= hse_uio_of_match,
	},
	.probe = hse_uio_probe,
	.remove = hse_uio_remove,
};

module_platform_driver(hse_uio_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("NXP HSE UIO Driver");
