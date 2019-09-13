// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Messaging Unit Interface
 *
 * This file contains the interface implementation for the Messaging Unit
 * instance used by host application cores to request services from HSE.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include "hse-mu.h"

#define HSE_MU_RX_IRQ     "hse-" HSE_MU_INST "-rx"
#define HSE_MU_ERR_IRQ    "hse-" HSE_MU_INST "-err"

#define HSE_STREAM_COUNT    2u /* number of usable streams per MU instance */
#define HSE_NUM_CHANNELS    16u /* number of available service channels */
#define HSE_CH_MASK_ALL     0x0000FFFFul /* all available channels irq mask */
#define HSE_STATUS_MASK     0xFFFF0000ul /* HSE global status FSR mask */

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
 * struct hse_mu_data - MU interface private data
 * @dev: parent device to be used for error logging
 * @regs: MU instance register space base virtual address
 * @rx_cbk[n].fn: upper layer rx callback for channel n
 * @rx_cbk[n].ctx: context passed to the callback on channel n
 * @sync[n]: completion for synchronous request rx on channel n
 * @refcnt: number of times each channel is currently acquired
 * @acq_lock: spinlock guarding stream channel acquisition
 * @tx_lock: spinlock guarding service request transmission
 * @reg_lock: spinlock guarding concurrent register access
 * @decode: upper layer service response decode function
 */
struct hse_mu_data {
	struct device *dev;
	struct hse_mu_regs __iomem *regs;
	struct {
		void (*fn)(void *mu_inst, u8 channel, void *ctx);
		void *ctx;
	} rx_cbk[HSE_NUM_CHANNELS];
	struct completion sync[HSE_NUM_CHANNELS];
	atomic_t refcnt[HSE_NUM_CHANNELS];
	spinlock_t acq_lock; /* for stream channel acquisition */
	spinlock_t tx_lock; /* for service request transmission */
	spinlock_t reg_lock; /* for concurrent register access */
	int (*decode)(u32 srv_rsp);
};

/**
 * enum hse_mu_irq_type - HSE MU interrupt type
 * @HSE_MU_INT_ACK_REQUEST: TX Interrupt, triggered when HSE acknowledged the
 *                          service request and released the service channel
 * @HSE_MU_INT_RESPONSE: RX Interrupt, triggered when HSE wrote the response
 * @HSE_MU_INT_SYS_EVENT: General Purpose Interrupt, triggered when HSE sends
 *                        a system event, generally an error notification
 */
enum hse_irq_type {
	HSE_MU_INT_ACK_REQUEST = 0x00u,
	HSE_MU_INT_RESPONSE = 0x01u,
	HSE_MU_INT_SYS_EVENT = 0x02u,
};

/**
 * hse_mu_channel_available - check service channel status
 * @mu_inst: MU instance
 * @channel: service channel index
 *
 * The 16 LSB of MU instance FSR are used by HSE for signaling channel status
 * as busy after a service request has been sent, until the HSE reply is ready.
 *
 * Return: true for channel available, false for invalid index or channel busy
 */
static bool hse_mu_channel_available(void *mu_inst, u8 channel)
{
	struct hse_mu_data *priv = mu_inst;
	u32 fsrval, tsrval, rsrval;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return false;

	fsrval = ioread32(&priv->regs->fsr) & BIT(channel);
	tsrval = ioread32(&priv->regs->tsr) & BIT(channel);
	rsrval = ioread32(&priv->regs->rsr) & BIT(channel);

	if (fsrval || !tsrval || rsrval)
		return false;

	return true;
}

/**
 * hse_mu_next_free_channel - find the next free service channel or stream
 * @mu_inst: MU instance
 * @request_stream: stream type channel requested
 *
 * If request_stream=true, select a channel not currently in use from the first
 * HSE_STREAM_COUNT channels, which are restricted to streaming use. Otherwise,
 * find an available service channel, excluding any reserved for streaming use.
 *
 * Return: channel index, HSE_INVALID_CHANNEL if none available
 */
static u8 hse_mu_next_free_channel(void *mu_inst, bool request_stream)
{
	struct hse_mu_data *priv = mu_inst;
	u8 channel, start_idx, end_idx;

	if (request_stream) {
		start_idx = 0u;
		end_idx = HSE_STREAM_COUNT;
	} else {
		start_idx = HSE_STREAM_COUNT;
		end_idx = HSE_NUM_CHANNELS;
	}

	for (channel = start_idx; channel < end_idx; channel++) {
		if (request_stream && atomic_read(&priv->refcnt[channel]))
			continue;
		if (hse_mu_channel_available(mu_inst, channel))
			return channel;
	}

	return HSE_INVALID_CHANNEL;
}

/**
 * hse_mu_channel_acquire - acquire a stream or a shared channel (non-blocking)
 * @mu_inst: MU instance
 * @channel: service channel index
 * @request_stream: stream type channel requested
 *
 * The first HSE_STREAM_COUNT channels of the MU instance are restricted to
 * streaming use and are reserved over the duration they are used. All other
 * service channels function as shared and may be acquired multiple times
 * before being released. Shared channels should only be acquired if the order
 * of requests must be preserved, as this is not guaranteed by HSE_ANY_CHANNEL.
 *
 * If request_stream=true, reserve the first available stream and return its
 * channel index, which, for convenience, matches the HSE stream ID. Otherwise,
 * find and acquire the shared channel with the lowest reference count.
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -EBUSY for no stream
 *         currently available
 */
int hse_mu_channel_acquire(void *mu_inst, u8 *channel, bool request_stream)
{
	struct hse_mu_data *priv = mu_inst;
	unsigned int min, i;

	if (unlikely(!mu_inst || !channel))
		return -EINVAL;

	if (request_stream) {
		spin_lock(&priv->acq_lock);

		/* find the first available stream */
		*channel = hse_mu_next_free_channel(mu_inst, true);
		if (*channel == HSE_INVALID_CHANNEL) {
			spin_unlock(&priv->acq_lock);
			dev_dbg(priv->dev, "failed to acquire stream\n");
			return -EBUSY;
		}

		/* mark stream as reserved */
		atomic_set(&priv->refcnt[*channel], 1);

		spin_unlock(&priv->acq_lock);
	} else {
		*channel = HSE_STREAM_COUNT;
		min = atomic_read(&priv->refcnt[*channel]);

		/* find the shared channel with lowest refcount */
		for (i = HSE_STREAM_COUNT; min > 0 && i < HSE_NUM_CHANNELS; i++)
			if (atomic_read(&priv->refcnt[i]) < min) {
				*channel = i;
				min = atomic_read(&priv->refcnt[i]);
			}

		/* increment channel refcount */
		atomic_inc(&priv->refcnt[*channel]);
	}

	return 0;
}

/**
 * hse_mu_channel_release - release a stream or a shared channel
 * @mu_inst: MU instance
 * @channel: service channel index
 *
 * Mark stream as released or decrement the reference count of shared channel.
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range
 */
int hse_mu_channel_release(void *mu_inst, u8 channel)
{
	struct hse_mu_data *priv = mu_inst;

	if (unlikely(!mu_inst))
		return -EINVAL;

	if (channel > HSE_NUM_CHANNELS)
		return -ECHRNG;

	atomic_dec_if_positive(&priv->refcnt[channel]);

	return 0;
}

/**
 * hse_mu_irq_enable - enable a specific type of interrupt using the mask
 * @mu_inst: MU instance
 * @type: MU interrupt type
 * @mask: interrupt mask
 */
static void hse_mu_irq_enable(void *mu_inst, enum hse_irq_type type, u32 mask)
{
	struct hse_mu_data *priv = mu_inst;
	void *regaddr;
	unsigned long flags;
	u32 regval;

	switch (type) {
	case HSE_MU_INT_ACK_REQUEST:
		regaddr = &priv->regs->tcr;
		break;
	case HSE_MU_INT_RESPONSE:
		regaddr = &priv->regs->rcr;
		break;
	case HSE_MU_INT_SYS_EVENT:
		regaddr = &priv->regs->gier;
		break;
	default:
		return;
	}

	spin_lock_irqsave(&priv->reg_lock, flags);

	regval = ioread32(regaddr);
	regval |= mask & HSE_CH_MASK_ALL;
	iowrite32(regval, regaddr);

	spin_unlock_irqrestore(&priv->reg_lock, flags);
}

/**
 * hse_mu_irq_disable - disable a specific type of interrupt using the mask
 * @mu_inst: MU instance
 * @type: MU interrupt type
 * @mask: interrupt mask
 */
static void hse_mu_irq_disable(void *mu_inst, enum hse_irq_type type, u32 mask)
{
	struct hse_mu_data *priv = mu_inst;
	void *regaddr;
	unsigned long flags;
	u32 regval;

	switch (type) {
	case HSE_MU_INT_ACK_REQUEST:
		regaddr = &priv->regs->tcr;
		break;
	case HSE_MU_INT_RESPONSE:
		regaddr = &priv->regs->rcr;
		break;
	case HSE_MU_INT_SYS_EVENT:
		regaddr = &priv->regs->gier;
		break;
	default:
		return;
	}

	spin_lock_irqsave(&priv->reg_lock, flags);

	regval = ioread32(regaddr);
	regval &= ~(mask & HSE_CH_MASK_ALL);
	iowrite32(regval, regaddr);

	spin_unlock_irqrestore(&priv->reg_lock, flags);
}

/**
 * hse_mu_irq_clear - clear pending general purpose interrupt using the mask
 * @mu_inst: MU instance
 * @type: MU interrupt type
 * @mask: interrupt mask
 */
static void hse_mu_irq_clear(void *mu_inst, enum hse_irq_type type, u32 mask)
{
	struct hse_mu_data *priv = mu_inst;
	u32 gsrval;

	if (unlikely(type != HSE_MU_INT_SYS_EVENT))
		return;

	gsrval = ioread32(&priv->regs->gsr);
	gsrval |= mask & HSE_CH_MASK_ALL;
	iowrite32(gsrval, &priv->regs->gsr);
}

/**
 * hse_mu_async_req_send - send an asynchronous service request (non-blocking)
 * @mu_inst: MU instance
 * @channel: service channel index
 * @srv_desc: service descriptor physical address
 * @ctx: context passed to rx callback
 * @rx_cbk: upper layer rx callback
 *
 * Send (non-blocking) a HSE service request on the selected channel and
 * register a callback function to be executed asynchronously upon the
 * completion. Context shall be supplied to the callback. The channel index
 * shall be set to HSE_ANY_CHANNEL unless obtained via hse_mu_channel_acquire().
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -EBUSY for selected channel busy or no channel
 *         currently available, -ENOSTR for channel not acquired beforehand
 */
int hse_mu_async_req_send(void *mu_inst, u8 channel, u32 srv_desc, void *ctx,
			  void (*rx_cbk)(void *mu_inst, u8 channel, void *ctx))
{
	struct hse_mu_data *priv = mu_inst;

	if (unlikely(!mu_inst || !rx_cbk || !ctx))
		return -EINVAL;

	if (unlikely(channel != HSE_ANY_CHANNEL && channel > HSE_NUM_CHANNELS))
		return -ECHRNG;

	if (channel == HSE_ANY_CHANNEL) {
		channel = hse_mu_next_free_channel(mu_inst, false);
		if (unlikely(channel == HSE_INVALID_CHANNEL)) {
			dev_dbg(priv->dev, "no service channel available\n");
			return -EBUSY;
		}
	} else if (!atomic_read(&priv->refcnt[channel])) {
		dev_dbg(priv->dev, "channel %d not acquired\n", channel);
		return -ENOSTR;
	}

	spin_lock(&priv->tx_lock);

	if (unlikely(!hse_mu_channel_available(mu_inst, channel))) {
		spin_unlock(&priv->tx_lock);
		dev_dbg(priv->dev, "service channel %d busy\n", channel);
		return -EBUSY;
	}

	iowrite32(srv_desc, &priv->regs->tr[channel]);

	spin_unlock(&priv->tx_lock);

	priv->rx_cbk[channel].fn = rx_cbk;
	priv->rx_cbk[channel].ctx = ctx;

	hse_mu_irq_enable(mu_inst, HSE_MU_INT_RESPONSE, BIT(channel));

	return 0;
}

/**
 * hse_mu_response_ready - check if a service request response is pending
 * @mu_inst: MU instance
 * @channel: service channel index
 *
 * Return: true for response ready, false otherwise
 */
static bool hse_mu_response_ready(void *mu_inst, u8 channel)
{
	struct hse_mu_data *priv = mu_inst;
	u32 rsrval;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return false;

	rsrval = ioread32(&priv->regs->rsr) & BIT(channel);
	if (!rsrval)
		return false;

	return true;
}

/**
 * hse_mu_async_req_recv - read an asynchronous request response (non-blocking)
 * @mu_inst: MU instance
 * @channel: service channel index
 *
 * Read (non-blocking) and decode the HSE response for the asynchronous service
 * request sent on the selected channel. Shall be called from the rx callback.
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -ENOMSG for no reply pending on selected channel
 */
int hse_mu_async_req_recv(void *mu_inst, u8 channel)
{
	struct hse_mu_data *priv = mu_inst;
	u32 srv_rsp;

	if (unlikely(!mu_inst))
		return -EINVAL;

	if (unlikely(channel > HSE_NUM_CHANNELS))
		return -ECHRNG;

	if (unlikely(!hse_mu_response_ready(mu_inst, channel))) {
		dev_dbg(priv->dev, "no response yet on channel %d\n", channel);
		return -ENOMSG;
	}

	/* reset current rx callback */
	priv->rx_cbk[channel].fn = NULL;

	srv_rsp = ioread32(&priv->regs->rr[channel]);
	if (priv->decode(srv_rsp))
		dev_dbg(priv->dev, "service response 0x%08X on channel %d\n",
			srv_rsp, channel);

	return priv->decode(srv_rsp);
}

/**
 * hse_mu_sync_req - issue a synchronous service request (blocking)
 * @mu_inst: MU instance
 * @channel: service channel index
 * @srv_desc: service descriptor physical address
 *
 * Send a HSE service descriptor on the selected channel and block until the
 * HSE response becomes available, then read the reply. The channel index
 * shall be set to HSE_ANY_CHANNEL unless obtained via hse_mu_channel_acquire().
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -EBUSY for selected channel busy or no channel
 *         currently available, -ENOSTR for channel not acquired beforehand
 */
int hse_mu_sync_req(void *mu_inst, u8 channel, u32 srv_desc)
{
	struct hse_mu_data *priv = mu_inst;
	u32 srv_rsp;

	if (unlikely(!mu_inst))
		return -EINVAL;

	if (unlikely(channel != HSE_ANY_CHANNEL && channel > HSE_NUM_CHANNELS))
		return -ECHRNG;

	if (channel == HSE_ANY_CHANNEL) {
		channel = hse_mu_next_free_channel(mu_inst, false);
		if (unlikely(channel == HSE_INVALID_CHANNEL)) {
			dev_dbg(priv->dev, "no service channel available\n");
			return -EBUSY;
		}
	} else if (!atomic_read(&priv->refcnt[channel])) {
		dev_dbg(priv->dev, "channel %d not acquired\n", channel);
		return -ENOSTR;
	}

	spin_lock(&priv->tx_lock);

	if (unlikely(!hse_mu_channel_available(mu_inst, channel))) {
		spin_unlock(&priv->tx_lock);
		dev_dbg(priv->dev, "service channel %d busy\n", channel);
		return -EBUSY;
	}

	iowrite32(srv_desc, &priv->regs->tr[channel]);

	spin_unlock(&priv->tx_lock);

	reinit_completion(&priv->sync[channel]);

	hse_mu_irq_enable(mu_inst, HSE_MU_INT_RESPONSE, BIT(channel));

	wait_for_completion_interruptible(&priv->sync[channel]);

	srv_rsp = ioread32(&priv->regs->rr[channel]);
	if (priv->decode(srv_rsp))
		dev_dbg(priv->dev, "service response 0x%08X on channel %d\n",
			srv_rsp, channel);

	return priv->decode(srv_rsp);
}

/**
 * hse_mu_next_pending - find the next channel with a service response pending
 * @mu_inst: MU instance
 *
 * Return: channel index, HSE_INVALID_CHANNEL if none pending
 */
static u8 hse_mu_next_pending(void *mu_inst)
{
	struct hse_mu_data *priv = mu_inst;
	u32 rsrval = ioread32(&priv->regs->rsr) & HSE_CH_MASK_ALL;

	if (!ffs(rsrval))
		return HSE_INVALID_CHANNEL;

	return ffs(rsrval) - 1;
}

/**
 * hse_rx_soft_handler - deferred handler for HSE_MU_INT_RESPONSE interrupts
 * @irq: interrupt line
 * @mu_inst: MU instance
 *
 * For each pending service response, execute the upper layer callback in case
 * of an asynchronous request or signal completion of a synchronous request.
 */
static irqreturn_t hse_rx_soft_handler(int irq, void *mu_inst)
{
	struct hse_mu_data *priv = mu_inst;
	u8 channel = hse_mu_next_pending(mu_inst);

	while (channel != HSE_INVALID_CHANNEL) {
		hse_mu_irq_disable(mu_inst, HSE_MU_INT_RESPONSE, BIT(channel));

		if (priv->rx_cbk[channel].fn) {
			void *ctx = priv->rx_cbk[channel].ctx;

			priv->rx_cbk[channel].fn(mu_inst, channel, ctx);
		} else {
			complete(&priv->sync[channel]);
		}
		channel = hse_mu_next_pending(mu_inst);
	}

	return IRQ_HANDLED;
}

/**
 * hse_mu_status - check the HSE global status
 * @mu_inst: MU instance
 *
 * Return: 16 MSB of MU instance FSR
 */
u16 hse_mu_status(void *mu_inst)
{
	struct hse_mu_data *priv = mu_inst;
	u32 fsrval;

	fsrval = ioread32(&priv->regs->fsr);
	fsrval = (fsrval & HSE_STATUS_MASK) >> 16u;

	return (u16)fsrval;
}

/**
 * hse_mu_err_handler - ISR for HSE_MU_INT_SYS_EVENT type interrupts
 * @irq: interrupt line
 * @mu_inst: MU instance
 */
static irqreturn_t hse_mu_err_handler(int irq, void *mu_inst)
{
	struct hse_mu_data *priv = mu_inst;
	u16 status = hse_mu_status(mu_inst);
	u32 gsrval = ioread32(&priv->regs->gsr);

	dev_crit(priv->dev, "system error 0x%x reported, status 0x%04x\n",
		 gsrval, status);

	hse_mu_irq_clear(mu_inst, HSE_MU_INT_SYS_EVENT, gsrval);

	return IRQ_HANDLED;
}

/**
 * hse_mu_init - initial setup of MU interface
 * @dev: parent device
 * @decode: upper layer service response decode function
 *
 * Return: MU instance, error code otherwise
 */
void *hse_mu_init(struct device *dev, int (*decode)(u32 srv_rsp))
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hse_mu_data *mu_inst;
	struct resource *res;
	int irq, err;
	u8 channel;

	if (unlikely(!dev || !decode))
		return ERR_PTR(-EINVAL);

	mu_inst = devm_kzalloc(dev, sizeof(*mu_inst), GFP_KERNEL);
	if (IS_ERR_OR_NULL(mu_inst))
		return ERR_PTR(-ENOMEM);
	mu_inst->dev = dev;
	mu_inst->decode = decode;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mu_inst->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(mu_inst->regs)) {
		dev_err(dev, "failed to map %s register space\n", HSE_MU_INST);
		return ERR_PTR(-ENOMEM);
	}

	spin_lock_init(&mu_inst->acq_lock);
	spin_lock_init(&mu_inst->tx_lock);
	spin_lock_init(&mu_inst->reg_lock);

	/* disable all interrupt sources */
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_ACK_REQUEST, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_SYS_EVENT, HSE_CH_MASK_ALL);

	irq = platform_get_irq_byname(pdev, HSE_MU_RX_IRQ);
	err = devm_request_threaded_irq(dev, irq, NULL, hse_rx_soft_handler,
					IRQF_ONESHOT, HSE_MU_RX_IRQ, mu_inst);
	if (unlikely(err)) {
		dev_err(dev, "failed to register %s irq, line %d\n",
			HSE_MU_RX_IRQ, irq);
		return ERR_PTR(-ENXIO);
	}

	irq = platform_get_irq_byname(pdev, HSE_MU_ERR_IRQ);
	err = devm_request_irq(dev, irq, hse_mu_err_handler, IRQF_TRIGGER_NONE,
			       HSE_MU_ERR_IRQ, mu_inst);
	if (unlikely(err)) {
		dev_err(dev, "failed to register %s irq, line %d\n",
			HSE_MU_ERR_IRQ, irq);
		return ERR_PTR(-ENXIO);
	}

	for (channel = 0; channel < HSE_NUM_CHANNELS; channel++) {
		if (!hse_mu_channel_available(mu_inst, channel))
			dev_warn(dev, "channel %d not available\n", channel);
		init_completion(&mu_inst->sync[channel]);
	}

	/* enable error notification */
	hse_mu_irq_enable(mu_inst, HSE_MU_INT_SYS_EVENT, HSE_CH_MASK_ALL);

	return mu_inst;
}

/**
 * hse_mu_free - final cleanup of MU interface
 * @mu_inst: MU instance
 */
void hse_mu_free(void *mu_inst)
{
	if (unlikely(!mu_inst))
		return;

	/* disable all interrupt sources */
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_ACK_REQUEST, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_SYS_EVENT, HSE_CH_MASK_ALL);
}
