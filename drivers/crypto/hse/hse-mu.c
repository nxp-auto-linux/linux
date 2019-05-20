// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Messaging Unit Interface
 *
 * This file contains the interface implementation for the Messaging Unit
 * instance used by host application cores to communicate with HSE.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/io.h>

#include "hse-mu.h"

#define HSE_MU_INST    "mu" __stringify(CONFIG_CRYPTO_DEV_NXP_HSE_MU_ID) "b"
#define HSE_RX_IRQ     "hse-" HSE_MU_INST "-rx"
#define HSE_ERR_IRQ    "hse-" HSE_MU_INST "-err"

#define HSE_STREAM_COUNT    2u /* number of usable streams per MU instance*/
#define HSE_NUM_CHANNELS    16u /* number of available service channels */
#define HSE_ALL_CHANNELS    0x0000FFFFul /* available channels irq mask */

/**
 * struct hse_mu_regs - HSE Messaging Unit Registers
 * @ver: Version ID Register, offset 0x0
 * @par: Version ID Register, offset 0x4
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
 * @mu_base: HSE MU instance base virtual address
 * @rx_cbk[n].cbk: upper layer rx callback for channel n
 * @rx_cbk[n].ctx: context passed to the rx callback n
 * @stream_status: status of current MU instance streams
 * @mu_lock: spinlock guarding access to HSE MU registers
 */
struct hse_mu_data {
	struct device *dev;
	void __iomem *mu_base;
	struct {
		void (*cbk)(void *mu_inst, u8 channel, void *ctx);
		void *ctx;
	} rx_cbk[HSE_NUM_CHANNELS];
	u16 stream_status;
	spinlock_t mu_lock; /* used for concurrent MU access */
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
	HSE_MU_INT_RESPONSE    = 0x01u,
	HSE_MU_INT_SYS_EVENT   = 0x02u,
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
	struct hse_mu_regs *mu = priv->mu_base;
	u32 fsrval, tsrval, rsrval;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return false;

	fsrval = ioread32(&mu->fsr) & (1 << channel);
	tsrval = ioread32(&mu->tsr) & (1 << channel);
	rsrval = ioread32(&mu->rsr) & (1 << channel);

	if (fsrval || !tsrval || rsrval)
		return false;

	return true;
}

/**
 * hse_mu_next_free_channel - find the next free service channel
 * @mu_inst: MU instance
 *
 * Find an available service channel, excluding any reserved for streaming use.
 *
 * Return: channel index, HSE_INVALID_CHANNEL if none available
 */
static u8 hse_mu_next_free_channel(void *mu_inst)
{
	u8 channel;

	for (channel = HSE_STREAM_COUNT; channel < HSE_NUM_CHANNELS; channel++)
		if (hse_mu_channel_available(mu_inst, channel))
			return channel;

	return HSE_INVALID_CHANNEL;
}

/**
 * hse_mu_reserve_channel - reserve a channel and its associated stream resource
 * @mu_inst: MU instance
 * @channel: service channel index
 *
 * The first HSE_STREAM_COUNT channels of the MU instance are restricted for
 * streaming usage. Provided that there is at least one stream resource
 * currently available, this function reserves one of these channels and returns
 * the channel index to the caller, which matches the stream ID for convenience.
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ENOSR for no stream
 *         resource currently available
 */
int hse_mu_reserve_channel(void *mu_inst, u8 *channel)
{
	struct hse_mu_data *priv = mu_inst;
	unsigned long flags;
	u8 stream;

	if (!mu_inst || !channel)
		return -EINVAL;

	spin_lock_irqsave(&priv->mu_lock, flags);

	for (stream = 0; stream < HSE_STREAM_COUNT; stream++) {
		if ((priv->stream_status & (1 << stream)) == 0) {
			priv->stream_status |= (1 << stream);
			break;
		}
	}

	spin_unlock_irqrestore(&priv->mu_lock, flags);

	if (stream >= HSE_STREAM_COUNT) {
		*channel = HSE_INVALID_CHANNEL;
		dev_dbg(priv->dev, "failed to acquire stream resource\n");
		return -ENOSR;
	}

	*channel = stream;

	return 0;
}

/**
 * hse_mu_release_channel - release a channel and its associated stream resource
 * @mu_inst: MU instance
 * @channel: service channel index
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range
 */
int hse_mu_release_channel(void *mu_inst, u8 channel)
{
	struct hse_mu_data *priv = mu_inst;
	unsigned long flags;

	if (!mu_inst)
		return -EINVAL;

	if (channel > HSE_STREAM_COUNT)
		return -ECHRNG;

	spin_lock_irqsave(&priv->mu_lock, flags);

	priv->stream_status &= ~(1 << channel);

	spin_unlock_irqrestore(&priv->mu_lock, flags);

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
	struct hse_mu_regs *mu = priv->mu_base;
	void *regaddr;
	unsigned long flags;
	u32 regval;

	switch (type) {
	case HSE_MU_INT_ACK_REQUEST:
		regaddr = &mu->tcr;
		break;
	case HSE_MU_INT_RESPONSE:
		regaddr = &mu->rcr;
		break;
	case HSE_MU_INT_SYS_EVENT:
		regaddr = &mu->gier;
		break;
	default:
		return;
	}

	spin_lock_irqsave(&priv->mu_lock, flags);

	regval = ioread32(regaddr);
	regval |= mask & HSE_ALL_CHANNELS;
	iowrite32(regval, regaddr);

	spin_unlock_irqrestore(&priv->mu_lock, flags);
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
	struct hse_mu_regs *mu = priv->mu_base;
	void *regaddr;
	unsigned long flags;
	u32 regval;

	switch (type) {
	case HSE_MU_INT_ACK_REQUEST:
		regaddr = &mu->tcr;
		break;
	case HSE_MU_INT_RESPONSE:
		regaddr = &mu->rcr;
		break;
	case HSE_MU_INT_SYS_EVENT:
		regaddr = &mu->gier;
		break;
	default:
		return;
	}

	spin_lock_irqsave(&priv->mu_lock, flags);

	regval = ioread32(regaddr);
	regval &= ~(mask & HSE_ALL_CHANNELS);
	iowrite32(regval, regaddr);

	spin_unlock_irqrestore(&priv->mu_lock, flags);
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
	struct hse_mu_regs *mu = priv->mu_base;
	u32 gierval;

	if (unlikely(type != HSE_MU_INT_SYS_EVENT))
		return;

	gierval = ioread32(&mu->gier);
	gierval |= mask & HSE_ALL_CHANNELS;
	iowrite32(gierval, &mu->gier);
}

/**
 * hse_mu_send_request - send a service request to HSE
 * @mu_inst: MU instance
 * @channel: service channel index
 * @srv_desc: service descriptor physical address
 * @ctx: context passed to rx callback
 * @rx_cbk: upper layer rx callback
 *
 * Send (non-blocking) a HSE service descriptor on the selected channel and
 * register a callback function to be executed when the operation completes.
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -EBUSY for service channel busy, -ENOSTR for
 *         channel not a valid stream, -EAGAIN for no channel available
 */
int hse_mu_send_request(void *mu_inst, u8 channel, u32 srv_desc, void *ctx,
			void (*rx_cbk)(void *mu_inst, u8 channel, void *ctx))
{
	struct hse_mu_data *priv = mu_inst;
	struct hse_mu_regs *mu;
	unsigned long flags;

	if (unlikely(!mu_inst || !rx_cbk || !ctx))
		return -EINVAL;
	mu = priv->mu_base;

	if (unlikely(channel >= HSE_NUM_CHANNELS && channel != HSE_ANY_CHANNEL))
		return -ECHRNG;

	if (channel == HSE_ANY_CHANNEL) {
		channel = hse_mu_next_free_channel(mu_inst);
	} else if (!(priv->stream_status & (1 << channel))) {
		dev_dbg(priv->dev, "channel %d not a valid stream\n", channel);
		return -ENOSTR;
	}

	if (unlikely(channel == HSE_INVALID_CHANNEL)) {
		dev_dbg(priv->dev, "all normal channels busy\n");
		return -EAGAIN;
	}

	spin_lock_irqsave(&priv->mu_lock, flags);

	if (!hse_mu_channel_available(mu_inst, channel)) {
		spin_unlock_irqrestore(&priv->mu_lock, flags);
		dev_dbg(priv->dev, "service channel %d busy\n", channel);
		return -EBUSY;
	}

	priv->rx_cbk[channel].cbk = rx_cbk;
	priv->rx_cbk[channel].ctx = ctx;

	iowrite32(srv_desc, &mu->tr[channel]);

	spin_unlock_irqrestore(&priv->mu_lock, flags);

	hse_mu_irq_enable(mu_inst, HSE_MU_INT_RESPONSE, (1 << channel));

	return 0;
}

/**
 * hse_mu_response_ready - check if a HSE response is ready to be read
 * @mu_inst: MU instance
 * @channel: service channel index
 *
 * Return: true for response ready, false otherwise
 */
static bool hse_mu_response_ready(void *mu_inst, u8 channel)
{
	struct hse_mu_data *priv = mu_inst;
	struct hse_mu_regs *mu = priv->mu_base;
	u32 rsrval;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return false;

	rsrval = ioread32(&mu->rsr) & (1 << channel);
	if (!rsrval)
		return false;

	return true;
}

/**
 * hse_mu_recv_response - read (non-blocking) the HSE response
 * @mu_inst: MU instance
 * @channel: service channel index
 * @srv_rsp: HSE service response
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ENOMSG for response
 *         not pending on selected service channel
 */
int hse_mu_recv_response(void *mu_inst, u8 channel, u32 *srv_rsp)
{
	struct hse_mu_data *priv = mu_inst;
	struct hse_mu_regs *mu;

	if (unlikely(!mu_inst || !srv_rsp))
		return -EINVAL;
	mu = priv->mu_base;

	if (unlikely(!hse_mu_response_ready(mu_inst, channel))) {
		dev_dbg(priv->dev, "no response yet on channel %d\n", channel);
		return -ENOMSG;
	}

	priv->rx_cbk[channel].cbk = NULL;
	priv->rx_cbk[channel].ctx = NULL;

	*srv_rsp = ioread32(&mu->rr[channel]);

	return 0;
}

/**
 * hse_mu_rx_handler - ISR for HSE_MU_INT_RESPONSE type interrupts
 */
static irqreturn_t hse_mu_rx_handler(int irq, void *mu_inst)
{
	struct hse_mu_data *priv = mu_inst;
	u8 channel;
	void *ctx;

	hse_mu_irq_disable(mu_inst, HSE_MU_INT_RESPONSE, HSE_ALL_CHANNELS);

	for (channel = 0; channel < HSE_NUM_CHANNELS; channel++)
		if (hse_mu_response_ready(mu_inst, channel) &&
		    priv->rx_cbk[channel].cbk) {
			ctx = priv->rx_cbk[channel].ctx;
			priv->rx_cbk[channel].cbk(mu_inst, channel, ctx);
		}

	return IRQ_HANDLED;
}

/**
 * hse_mu_get_status - check the HSE global status
 * @mu_inst: MU instance
 *
 * Return: 16 MSB of MU instance FSR
 */
static u16 hse_mu_get_status(void *mu_inst)
{
	struct hse_mu_data *priv = mu_inst;
	struct hse_mu_regs *mu = priv->mu_base;
	u32 fsrval;

	fsrval = ioread32(&mu->fsr);
	fsrval = (fsrval & 0xFFFF0000ul) >> 16;

	return (u16)fsrval;
}

/**
 * hse_mu_err_handler - ISR for HSE_MU_INT_SYS_EVENT type interrupts
 */
static irqreturn_t hse_mu_err_handler(int irq, void *mu_inst)
{
	struct hse_mu_data *priv = mu_inst;
	u16 status = hse_mu_get_status(mu_inst);

	hse_mu_irq_clear(mu_inst, HSE_MU_INT_SYS_EVENT, HSE_ALL_CHANNELS);

	dev_err(priv->dev, "system error reported by HSE\n");
	dev_err(priv->dev, "HSE status: 0x%04X\n", status);

	return IRQ_HANDLED;
}

/**
 * hse_mu_init - initial setup of MU interface
 * @dev: parent device
 *
 * Return: MU instance, error code otherwise
 */
void *hse_mu_init(struct device *dev)
{
	struct hse_mu_data *mu_inst;
	struct device_node *mu_node;
	int rx_irq, err_irq, err;
	struct resource mu_reg;

	mu_inst = devm_kzalloc(dev, sizeof(*mu_inst), GFP_KERNEL);
	if (IS_ERR_OR_NULL(mu_inst))
		return ERR_PTR(-ENOMEM);
	mu_inst->dev = dev;

	mu_node = of_get_child_by_name(dev->of_node, HSE_MU_INST);
	if (IS_ERR_OR_NULL(mu_node))
		return ERR_PTR(-ENODEV);

	if (unlikely(!of_device_is_available(mu_node))) {
		err = -ENODEV;
		goto err_node_put;
	}

	err = of_address_to_resource(mu_node, 0, &mu_reg);
	if (unlikely(err)) {
		err = -EFAULT;
		goto err_node_put;
	}

	rx_irq = of_irq_get_byname(mu_node, HSE_RX_IRQ);
	if (unlikely(rx_irq <= 0)) {
		err = -ENXIO;
		goto err_node_put;
	}

	err_irq = of_irq_get_byname(mu_node, HSE_ERR_IRQ);
	if (unlikely(err_irq <= 0)) {
		err = -ENXIO;
		goto err_node_put;
	}

	of_node_put(mu_node);

	mu_inst->mu_base = devm_ioremap_resource(dev, &mu_reg);
	if (IS_ERR_OR_NULL(mu_inst->mu_base)) {
		dev_err(dev, "map address 0x%08llx failed\n", mu_reg.start);
		return ERR_PTR(-ENOMEM);
	}

	spin_lock_init(&mu_inst->mu_lock);

	/* disable all interrupt sources */
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_ACK_REQUEST, HSE_ALL_CHANNELS);
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_RESPONSE, HSE_ALL_CHANNELS);
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_SYS_EVENT, HSE_ALL_CHANNELS);

	err = devm_request_irq(dev, rx_irq, hse_mu_rx_handler, 0,
			       HSE_RX_IRQ, mu_inst);
	if (unlikely(err)) {
		dev_err(dev, "register irq %d failed\n", rx_irq);
		return ERR_PTR(-ENXIO);
	}

	err = devm_request_irq(dev, err_irq, hse_mu_err_handler, 0,
			       HSE_ERR_IRQ, mu_inst);
	if (unlikely(err)) {
		dev_err(dev, "register irq %d failed\n", err_irq);
		return ERR_PTR(-ENXIO);
	}

	/* enable error notification */
	hse_mu_irq_enable(mu_inst, HSE_MU_INT_SYS_EVENT, HSE_ALL_CHANNELS);

	return mu_inst;
err_node_put:
	of_node_put(mu_node);
	return ERR_PTR(err);
}

/**
 * hse_mu_free - final cleanup of MU interface
 * @mu_inst: MU instance
 */
void hse_mu_free(void *mu_inst)
{
	/* disable all interrupt sources */
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_ACK_REQUEST, HSE_ALL_CHANNELS);
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_RESPONSE, HSE_ALL_CHANNELS);
	hse_mu_irq_disable(mu_inst, HSE_MU_INT_SYS_EVENT, HSE_ALL_CHANNELS);
}
