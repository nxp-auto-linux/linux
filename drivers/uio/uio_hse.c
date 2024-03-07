// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE UIO Driver
 *
 * This file contains the user-space I/O support for the Hardware Security
 * Engine user space driver. Used by the HSE user space library - libhse.
 *
 * Copyright 2021-2024 NXP
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
#include <linux/dma-mapping.h>

#define DRIVER_NAME       "hse-uio"
#define DRIVER_VERSION    "2.1"

#define HSE_REGS_NAME       "hse-" CONFIG_UIO_NXP_HSE_MU "-regs"
#define HSE_DESC_NAME       "hse-" CONFIG_UIO_NXP_HSE_MU "-desc"
#define HSE_RX_IRQ_NAME     "hse-" CONFIG_UIO_NXP_HSE_MU "-rx"
#define HSE_ERR_IRQ_NAME    "hse-" CONFIG_UIO_NXP_HSE_MU "-err"

#define HSE_NUM_CHANNELS    16u /* number of available service channels */

#define HSE_CHANNEL_ANY    0xACu /* use any channel, no request ordering */
#define HSE_CHANNEL_ADM    0u /* channel reserved for administrative services */
#define HSE_CHANNEL_INV    0xFFu /* invalid acquired service channel index */
#define HSE_CH_MASK_ALL    0x0000FFFFul /* all available channels irq mask */

#define HSE_STATUS_MASK    0xFFFF0000ul /* HSE global status FSR mask */

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
 * enum hse_status - HSE status
 * @HSE_STATUS_RNG_INIT_OK: RNG initialization successfully completed
 * @HSE_STATUS_INIT_OK: HSE initialization successfully completed
 * @HSE_STATUS_INSTALL_OK: HSE installation phase successfully completed,
 *                         key stores have been formatted and can be used
 * @HSE_STATUS_PUBLISH_SYS_IMAGE: volatile HSE configuration detected
 */
enum hse_status {
	HSE_STATUS_RNG_INIT_OK = BIT(5),
	HSE_STATUS_INIT_OK = BIT(8),
	HSE_STATUS_INSTALL_OK = BIT(9),
	HSE_STATUS_PUBLISH_SYS_IMAGE = BIT(13),
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
 * enum hse_srv_id - HSE service ID
 * @HSE_SRV_ID_GET_ATTR: get attribute, such as firmware version
 * @HSE_SRV_ID_PREPARE_FOR_STANDBY: prepare for system stand-by mode
 */
enum hse_srv_id {
	HSE_SRV_ID_GET_ATTR = 0x00A50002ul,
	HSE_SRV_ID_PREPARE_FOR_STANDBY = 0x00A50017ul,
};

/**
 * enum hse_srv_response - HSE service response
 * @HSE_SRV_RSP_OK: service successfully executed with no error
 * @HSE_SRV_RSP_NOT_ALLOWED: operation subject to restrictions (in attributes,
 *                           life-cycle dependent operations, key-management)
 * @HSE_SRV_RSP_CANCELED: service has been canceled
 */
enum hse_srv_response {
	HSE_SRV_RSP_OK = 0x55A5AA33ul,
	HSE_SRV_RSP_NOT_ALLOWED = 0xAA55A21Cul,
	HSE_SRV_RSP_CANCELED = 0x33D6D396ul,
};

/**
 * enum hse_host_event - HSE host event
 * @HSE_HOST_PERIPH_CONFIG_DONE: sent by the host to notify HSE when external
 *                               peripherals have been configured at init-time
 *                               (signal valid only when triggered from MU0)
 */
enum hse_host_event {
	HSE_HOST_PERIPH_CONFIG_DONE = BIT(0),
};

/**
 * enum hse_fw_status - HSE firmware status
 * @HSE_FW_SHUTDOWN: firmware not initialized or shut down due to fatal error
 * @HSE_FW_RUNNING: firmware running and able to service any type of request
 * @HSE_FW_STANDBY: firmware considered in stand-by state, no service requests
 */
enum hse_fw_status {
	HSE_FW_SHUTDOWN = 0u,
	HSE_FW_RUNNING = 1u,
	HSE_FW_STANDBY = 2u,
};

/**
 * enum hse_fw_type - HSE firmware type
 * @HSE_FW_STANDARD: standard firmware
 * @HSE_FW_PREMIUM: premium firmware
 * @HSE_FW_CUSTOM: custom firmware
 */
enum hse_fw_type {
	HSE_FW_STANDARD = 0u,
	HSE_FW_PREMIUM = 1u,
	HSE_FW_CUSTOM = 8u,
};

/**
 * enum hse_attr - HSE attribute
 * @HSE_FW_VERSION_ATTR_ID: firmware version
 */
enum hse_attr {
	HSE_FW_VERSION_ATTR_ID = 1u,
};

/**
 * struct hse_attr_fw_version - firmware version
 * @fw_type: attribute ID
 * @major: major revision
 * @minor: minor revision
 * @patch: patch version
 */
struct hse_attr_fw_version {
	u8 reserved[2];
	u16 fw_type;
	u8 major;
	u8 minor;
	u16 patch;
} __packed;

/**
 * struct hse_srv_desc - HSE service descriptor, used to retrieve fw version
 * @srv_id: service ID of the HSE request
 * @get_attr_req.attr_id: attribute ID
 * @get_attr_req.attr_len: attribute length, in bytes
 * @get_attr_req.attr: DMA address of the attribute
 */
struct hse_srv_desc {
	u32 srv_id;
	u8 reserved[4];
	struct hse_get_attr_srv {
		u16 attr_id;
		u8 reserved[2];
		u32 attr_len;
		u64 attr;
	} get_attr_req;
} __packed;

/**
 * struct hse_uio_intl - driver internal shared memory layout
 * @channel_ready[n]: reply ready on channel n
 * @channel_reply[n]: response on channel n
 * @event: HSE firmware system event mask
 * @setup_done: initialization sequence done flag
 * @firmware_status: cached status of HSE firmware
 * @channel_busy[n]: service channel busy flag
 * @channel_res[n]: channel currently reserved
 */
struct hse_uio_intl {
	u8 channel_ready[HSE_NUM_CHANNELS];
	u32 channel_reply[HSE_NUM_CHANNELS];
	u32 event;
	bool setup_done;
	enum hse_fw_status firmware_status;
	u8 reserved[2];
	bool channel_busy[HSE_NUM_CHANNELS];
	bool channel_res[HSE_NUM_CHANNELS];
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
 * @refcnt: open instances reference counter
 * @regs: MU register space base virtual address
 * @desc: service descriptor space base virtual address
 * @intl: driver internal shared memory base address
 * @reg_lock: spinlock preventing concurrent register access
 * @firmware_version: firmware version attribute structure
 */
struct hse_uio_drvdata {
	struct device *dev;
	struct uio_info info;
	atomic_t inst_refcnt;
	struct hse_mu_regs __iomem *regs;
	void __iomem *desc;
	struct hse_uio_intl *intl;
	spinlock_t reg_lock; /* covers irq enable/disable */
	struct hse_attr_fw_version firmware_version ____cacheline_aligned;
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
 * hse_uio_trigger_event - trigger HSE host event
 * @dev: HSE UIO device
 *
 * Return: HSE host event mask
 */
static void hse_uio_trigger_event(struct device *dev, u32 evt)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);

	return iowrite32(evt, &drv->regs->gcr);
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
 * hse_uio_channel_available - check service channel status
 * @dev: HSE UIO device
 * @channel: channel index
 *
 * The 16 LSB of MU instance FSR are used by HSE for signaling channel status
 * as busy after a service request has been sent, until the HSE reply is ready.
 *
 * Return: true for channel available, false for invalid index or channel busy
 */
static bool hse_uio_channel_available(struct device *dev, u8 channel)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	u32 fsrval, tsrval, rsrval;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return false;

	fsrval = ioread32(&drv->regs->fsr) & BIT(channel);
	tsrval = ioread32(&drv->regs->tsr) & BIT(channel);
	rsrval = ioread32(&drv->regs->rsr) & BIT(channel);

	if (fsrval || !tsrval || rsrval)
		return false;

	return true;
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
 * hse_uio_msg_send - send a message over MU (non-blocking)
 * @dev: HSE UIO device
 * @channel: channel index
 * @msg: input message
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -EBUSY for selected channel busy
 */
static int hse_uio_msg_send(struct device *dev, u8 channel, u32 msg)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);

	if (unlikely(!dev))
		return -EINVAL;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return -ECHRNG;

	if (unlikely(!hse_uio_channel_available(dev, channel)))
		return -EBUSY;

	iowrite32(msg, &drv->regs->tr[channel]);

	return 0;
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
 * hse_uio_check_fw_version - retrieve firmware version
 * @dev: HSE UIO device
 */
static int hse_uio_check_fw_version(struct device *dev)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	u32 msg = lower_32_bits(drv->info.mem[HSE_UIO_MAP_DESC].addr);
	dma_addr_t firmware_version_dma;
	struct hse_srv_desc srv_desc;
	int err = 0;

	firmware_version_dma = dma_map_single(dev, &drv->firmware_version,
					      sizeof(drv->firmware_version),
					      DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(dev, firmware_version_dma)))
		return -ENOMEM;

	/* reserve channel zero */
	if (unlikely(drv->intl->channel_busy[HSE_CHANNEL_ADM])) {
		dev_err(dev, "%s: channel %d busy\n", __func__, HSE_CHANNEL_ADM);
		goto out;
	}
	drv->intl->channel_busy[HSE_CHANNEL_ADM] = true;

	/* fill descriptor in */
	memzero_explicit(&srv_desc, sizeof(srv_desc));
	srv_desc.srv_id = HSE_SRV_ID_GET_ATTR;
	srv_desc.get_attr_req.attr_id = HSE_FW_VERSION_ATTR_ID;
	srv_desc.get_attr_req.attr_len = sizeof(drv->firmware_version);
	srv_desc.get_attr_req.attr = firmware_version_dma;
	memcpy_toio(drv->desc, &srv_desc, sizeof(srv_desc));

	/* send request */
	err = hse_uio_msg_send(dev, HSE_CHANNEL_ADM, msg);
	if (unlikely(err)) {
		dev_err(dev, "%s: send request failed: %d\n", __func__, err);
		goto out;
	}

	/* wait for response from firmware */
	while (!hse_uio_msg_pending(dev, HSE_CHANNEL_ADM))
		;

	/* read reply */
	err = hse_uio_msg_recv(dev, HSE_CHANNEL_ADM, &msg);
	if (unlikely(err)) {
		dev_err(dev, "%s: read reply failed: %d\n", __func__, err);
		goto out;
	}

	if (unlikely(msg != HSE_SRV_RSP_OK))
		dev_warn(dev, "%s: request failed: 0x%08x\n", __func__, msg);
out:
	dma_unmap_single(dev, firmware_version_dma,
			 sizeof(drv->firmware_version), DMA_FROM_DEVICE);
	drv->intl->channel_busy[HSE_CHANNEL_ADM] = false;
	return err;
}

/**
 * hse_uio_open - open /dev/uioX device
 * @info: UIO device info
 * @inode: inode, not used
 *
 * Enables RX interrupts if there is at least one libhse instance active.
 */
static int hse_uio_open(struct uio_info *info, struct inode *inode)
{
	struct hse_uio_drvdata *drv = info->priv;

	if (!atomic_read(&drv->inst_refcnt))
		hse_uio_irqcontrol(&drv->info, HSE_UIO_ENABLE_RX_IRQ_CMD);

	atomic_inc(&drv->inst_refcnt);

	dev_info(drv->dev, "device %s v%s open, instances: %d\n",
		 info->name, info->version, atomic_read(&drv->inst_refcnt));

	return 0;
}

/**
 * hse_uio_release - release /dev/uioX device
 * @info: UIO device info
 * @inode: inode, not used
 *
 * Disables RX interrupts if there are no libhse instances active.
 */
static int hse_uio_release(struct uio_info *info, struct inode *inode)
{
	struct hse_uio_drvdata *drv = info->priv;

	if (atomic_dec_and_test(&drv->inst_refcnt))
		hse_uio_irqcontrol(&drv->info, HSE_UIO_DISABLE_RX_IRQ_CMD);

	dev_info(drv->dev, "device %s v%s released, instances: %d\n",
		 info->name, info->version, atomic_read(&drv->inst_refcnt));

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

	/* process pending replies, if any */
	while (channel != HSE_CHANNEL_INV) {
		/* handle reply */
		err = hse_uio_msg_recv(dev, channel, &srv_rsp);
		if (likely(!err)) {
			drv->intl->channel_reply[channel] = srv_rsp;
			drv->intl->channel_ready[channel] = 1;

			/* notify upper layer */
			uio_event_notify(&drv->info);
		}

		channel = hse_uio_next_pending_channel(dev);
	}

	return IRQ_HANDLED;
}

/**
 * hse_srv_req_cancel_all - cancel all requests currently in progress
 * @dev: HSE UIO device
 */
static void hse_srv_req_cancel_all(struct device *dev)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	u8 channel = hse_uio_next_pending_channel(dev);
	u32 srv_rsp;
	int err;

	/* process pending replies, if any */
	while (channel != HSE_CHANNEL_INV) {
		/* handle reply */
		err = hse_uio_msg_recv(dev, channel, &srv_rsp);
		if (likely(!err)) {
			drv->intl->channel_reply[channel] = srv_rsp;
			drv->intl->channel_ready[channel] = 1;

			/* notify upper layer */
			uio_event_notify(&drv->info);
		}

		channel = hse_uio_next_pending_channel(dev);
	}

	/* notify upper layer that all requests are canceled */
	for (channel = 0; channel < HSE_NUM_CHANNELS; channel++) {
		if (!drv->intl->channel_busy[channel])
			continue;

		drv->intl->channel_busy[channel] = false;
		drv->intl->channel_reply[channel] = HSE_SRV_RSP_CANCELED;
		drv->intl->channel_ready[channel] = 1;
		dev_warn(dev, "request canceled, channel %d\n", channel);
	}
}

/**
 * hse_uio_evt_dispatcher - deferred handler for HSE_INT_SYS_EVENT interrupts
 * @irq: interrupt line
 * @dev: HSE UIO device
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

	/* stop any subsequent requests */
	drv->intl->firmware_status = HSE_FW_SHUTDOWN;

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
	unsigned long intl_page;
	unsigned int channel;
	char *fw_type;
	int irq, err, page_num;
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

	/* expose HSE MU register space to upper layer */
	drv->info.mem[HSE_UIO_MAP_REGS].name = "hse-mu-registers";
	drv->info.mem[HSE_UIO_MAP_REGS].addr = (phys_addr_t)res->start;
	drv->info.mem[HSE_UIO_MAP_REGS].size = resource_size(res);
	drv->info.mem[HSE_UIO_MAP_REGS].memtype = UIO_MEM_PHYS;

	drv->info.version = DRIVER_VERSION;
	drv->info.name = DRIVER_NAME;

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
	drv->info.mem[HSE_UIO_MAP_DESC].addr = (phys_addr_t)res->start;
	drv->info.mem[HSE_UIO_MAP_DESC].size = resource_size(res);
	drv->info.mem[HSE_UIO_MAP_DESC].memtype = UIO_MEM_PHYS;

	/* alloc internal shared memory */
	intl_page = devm_get_free_pages(dev,
					GFP_KERNEL | __GFP_ZERO | __GFP_COMP,
					CONFIG_UIO_NXP_HSE_INTL_MEM_SIZE);
	if (unlikely(!intl_page))
		return -ENOMEM;
	drv->intl = (void *)intl_page;

	page_num = BIT(CONFIG_UIO_NXP_HSE_INTL_MEM_SIZE);

	/* expose driver internal memory to upper layer */
	drv->info.mem[HSE_UIO_MAP_INTL].name = "hse-driver-internal";
	drv->info.mem[HSE_UIO_MAP_INTL].addr = (phys_addr_t)drv->intl;
	drv->info.mem[HSE_UIO_MAP_INTL].size = page_num * PAGE_SIZE;
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
		of_node_put(rmem_node);
		return -ENXIO;
	}
	of_node_put(rmem_node);

	/* expose HSE reserved memory to upper layer */
	drv->info.mem[HSE_UIO_MAP_RMEM].name = "hse-reserved-memory";
	drv->info.mem[HSE_UIO_MAP_RMEM].addr = (phys_addr_t)rmem->base;
	drv->info.mem[HSE_UIO_MAP_RMEM].size = rmem->size;
	drv->info.mem[HSE_UIO_MAP_RMEM].memtype = UIO_MEM_PHYS;

	/* initialize reference counter and lock */
	atomic_set(&drv->inst_refcnt, 0);
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

	/* reset channels to initial state */
	for (channel = 0; channel < HSE_NUM_CHANNELS; channel++) {
		drv->intl->channel_ready[channel] = 0;
		drv->intl->channel_reply[channel] = 0;
		drv->intl->channel_busy[channel] = false;
		drv->intl->channel_res[channel] = false;
	}
	drv->intl->setup_done = false;
	drv->intl->event = 0;
	drv->intl->channel_res[0] = true; /* restrict channel zero */

	/* check firmware status */
	status = hse_uio_check_status(dev);
	if (!likely(status & HSE_STATUS_INIT_OK)) {
		if (IS_ENABLED(CONFIG_UIO_NXP_HSE_MU0))
			dev_err(dev, "firmware not found\n");
		else
			dev_err(dev, "interface %s not active\n",
				CONFIG_UIO_NXP_HSE_MU);
		return -ENODEV;
	}
	drv->intl->firmware_status = HSE_FW_RUNNING;

	/* register device */
	err = uio_register_device(dev, &drv->info);
	if (err) {
		dev_err(dev, "failed to register UIO device: %d\n", err);
		return err;
	}

	/* check firmware version */
	err = hse_uio_check_fw_version(dev);
	if (likely(!err)) {
		switch (drv->firmware_version.fw_type) {
		case HSE_FW_STANDARD:
			fw_type = "standard";
			break;
		case HSE_FW_PREMIUM:
			fw_type = "premium";
			break;
		default:
			fw_type = "custom";
			break;
		}

		dev_info(dev, "%s firmware, version %d.%d.%d\n", fw_type,
			 drv->firmware_version.major,
			 drv->firmware_version.minor,
			 drv->firmware_version.patch);
	}

	/* enable system event notifications */
	hse_uio_irqcontrol(&drv->info, HSE_UIO_ENABLE_EVT_IRQ_CMD);

	dev_info(dev, "successfully registered device\n");

	return 0;
}

static int hse_uio_remove(struct platform_device *pdev)
{
	struct hse_uio_drvdata *drv = platform_get_drvdata(pdev);

	/* disable all firmware notifications */
	hse_uio_irqcontrol(&drv->info, HSE_UIO_DISABLE_ALL_IRQ_CMD);

	/* unregister device */
	uio_unregister_device(&drv->info);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hse_pm_suspend(struct device *dev)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	u32 msg = lower_32_bits(drv->info.mem[HSE_UIO_MAP_DESC].addr);
	struct hse_srv_desc srv_desc;
	int err = 0;

	/* stop subsequent requests */
	drv->intl->firmware_status = HSE_FW_STANDBY;

	/* disable RX notifications */
	hse_uio_irq_disable(dev, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);

	/* reserve channel zero */
	if (unlikely(drv->intl->channel_busy[HSE_CHANNEL_ADM])) {
		dev_err(dev, "%s: channel %d busy\n", __func__, HSE_CHANNEL_ADM);
		goto err_enable_irq;
	}
	drv->intl->channel_busy[HSE_CHANNEL_ADM] = true;

	/* prepare firmware for stand-by */
	memzero_explicit(&srv_desc, sizeof(srv_desc));
	srv_desc.srv_id = HSE_SRV_ID_PREPARE_FOR_STANDBY;
	memcpy_toio(drv->desc, &srv_desc, sizeof(srv_desc));

	/* send request */
	err = hse_uio_msg_send(dev, HSE_CHANNEL_ADM, msg);
	if (unlikely(err)) {
		dev_err(dev, "%s: send request failed: %d\n", __func__, err);
		goto err_free_channel;
	}

	/* wait for response from firmware */
	while (!hse_uio_msg_pending(dev, HSE_CHANNEL_ADM))
		;

	/* read reply */
	err = hse_uio_msg_recv(dev, HSE_CHANNEL_ADM, &msg);
	if (unlikely(err)) {
		dev_err(dev, "%s: read reply failed: %d\n", __func__, err);
		goto err_free_channel;
	}

	if (unlikely(msg != HSE_SRV_RSP_OK &&
		     msg != HSE_SRV_RSP_NOT_ALLOWED)) {
		dev_err(dev, "%s: request failed: 0x%08x\n", __func__, msg);
		goto err_free_channel;
	}
	drv->intl->channel_busy[HSE_CHANNEL_ADM] = false;

	/* cancel other requests */
	hse_srv_req_cancel_all(dev);

	dev_info(dev, "device ready for stand-by\n");

	return 0;
err_free_channel:
	drv->intl->channel_busy[HSE_CHANNEL_ADM] = false;
err_enable_irq:
	hse_uio_irq_enable(dev, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	drv->intl->firmware_status = HSE_FW_RUNNING;
	return err;
}

static int hse_pm_resume(struct device *dev)
{
	struct hse_uio_drvdata *drv = dev_get_drvdata(dev);
	u16 status;
	int err;

	if (IS_ENABLED(CONFIG_UIO_NXP_HSE_MU0)) {
		/* signal firmware not to wait for peripheral configuration */
		hse_uio_trigger_event(dev, HSE_HOST_PERIPH_CONFIG_DONE);
	}

	/* enable RX and system event notifications */
	if (!atomic_read(&drv->inst_refcnt))
		hse_uio_irqcontrol(&drv->info, HSE_UIO_ENABLE_RX_IRQ_CMD);
	hse_uio_irqcontrol(&drv->info, HSE_UIO_ENABLE_EVT_IRQ_CMD);

	/* check firmware status */
	status = hse_uio_check_status(dev);
	if (!(status & HSE_STATUS_INIT_OK)) {
		/* wait for firmware init */
		err = hse_uio_check_fw_version(dev);
		if (unlikely(err)) {
			dev_err(dev, "%s: request failed: %d\n", __func__, err);
			return err;
		}
		status = hse_uio_check_status(dev);
	}

	drv->intl->firmware_status = HSE_FW_RUNNING;
	dev_info(dev, "device resumed, status 0x%04X\n", status);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(hse_pm_ops, hse_pm_suspend, hse_pm_resume);

static const struct of_device_id hse_uio_of_match[] = {
	{
		.name = CONFIG_UIO_NXP_HSE_MU,
		.compatible = "nxp,s32cc-hse",
	}, {}
};
MODULE_DEVICE_TABLE(of, hse_uio_of_match);

static struct platform_driver hse_uio_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table	= hse_uio_of_match,
		.pm = &hse_pm_ops,
	},
	.probe = hse_uio_probe,
	.remove = hse_uio_remove,
};

module_platform_driver(hse_uio_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("NXP HSE UIO Driver");
MODULE_VERSION(DRIVER_VERSION);
