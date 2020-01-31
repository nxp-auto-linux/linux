// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Messaging Unit Interface
 *
 * This file contains the interface implementation for the Messaging Unit
 * instance used by host application cores to request services from HSE.
 *
 * Copyright 2019-2020 NXP
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include "hse-mu.h"

#define HSE_STATUS_MASK     0xFFFF0000ul /* HSE global status FSR mask */

#define HSE_RX_IRQ_NAME     "hse-" HSE_MU_INST "-rx"
#define HSE_ERR_IRQ_NAME    "hse-" HSE_MU_INST "-err"

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
 * @dev: HSE device, only used here for error logging
 * @regs: MU instance register space base virtual address
 * @reg_lock: spinlock preventing concurrent register access
 */
struct hse_mu_data {
	struct device *dev;
	struct hse_mu_regs __iomem *regs;
	spinlock_t reg_lock; /* covers irq enable/disable */
};

/**
 * hse_mu_check_status - check the HSE global status
 * @mu: MU instance handle
 *
 * Return: 16 MSB of MU instance FSR
 */
u16 hse_mu_check_status(void *mu)
{
	struct hse_mu_data *priv = mu;
	u32 fsrval;

	fsrval = ioread32(&priv->regs->fsr);
	fsrval = (fsrval & HSE_STATUS_MASK) >> 16u;

	return (u16)fsrval;
}

/**
 * hse_mu_check_event - check for HSE system events
 * @mu: MU instance handle
 *
 * Return: first bit set in the HSE system event mask
 */
u32 hse_mu_check_event(void *mu)
{
	struct hse_mu_data *priv = mu;
	u32 gsrval;

	gsrval = ioread32(&priv->regs->gsr);

	return gsrval & (1 << (ffs(gsrval) - 1));
}

/**
 * hse_mu_irq_enable - enable a specific type of interrupt using a mask
 * @mu: MU instance handle
 * @irq_type: interrupt type
 * @irq_mask: interrupt mask
 */
void hse_mu_irq_enable(void *mu, enum hse_irq_type irq_type, u32 irq_mask)
{
	struct hse_mu_data *priv = mu;
	void *regaddr;
	unsigned long flags;
	u32 regval;

	switch (irq_type) {
	case HSE_INT_ACK_REQUEST:
		regaddr = &priv->regs->tcr;
		break;
	case HSE_INT_RESPONSE:
		regaddr = &priv->regs->rcr;
		break;
	case HSE_INT_SYS_EVENT:
		regaddr = &priv->regs->gier;
		break;
	default:
		return;
	}

	spin_lock_irqsave(&priv->reg_lock, flags);

	regval = ioread32(regaddr);
	regval |= irq_mask & HSE_CH_MASK_ALL;
	iowrite32(regval, regaddr);

	spin_unlock_irqrestore(&priv->reg_lock, flags);
}

/**
 * hse_mu_irq_disable - disable a specific type of interrupt using a mask
 * @mu: MU instance handle
 * @irq_type: interrupt type
 * @irq_mask: interrupt mask
 */
void hse_mu_irq_disable(void *mu, enum hse_irq_type irq_type, u32 irq_mask)
{
	struct hse_mu_data *priv = mu;
	void *regaddr;
	unsigned long flags;
	u32 regval;

	switch (irq_type) {
	case HSE_INT_ACK_REQUEST:
		regaddr = &priv->regs->tcr;
		break;
	case HSE_INT_RESPONSE:
		regaddr = &priv->regs->rcr;
		break;
	case HSE_INT_SYS_EVENT:
		regaddr = &priv->regs->gier;
		break;
	default:
		return;
	}

	spin_lock_irqsave(&priv->reg_lock, flags);

	regval = ioread32(regaddr);
	regval &= ~(irq_mask & HSE_CH_MASK_ALL);
	iowrite32(regval, regaddr);

	spin_unlock_irqrestore(&priv->reg_lock, flags);
}

/**
 * hse_mu_irq_clear - clear a pending general purpose interrupt using a mask
 * @mu: MU instance handle
 * @irq_type: interrupt type
 * @irq_mask: interrupt mask
 *
 * Only general purpose interrupts can be cleared. TX and RX irq lines, if
 * enabled, will remain asserted until the appropriate MU register is read.
 */
void hse_mu_irq_clear(void *mu, enum hse_irq_type irq_type, u32 irq_mask)
{
	struct hse_mu_data *priv = mu;

	if (unlikely(irq_type != HSE_INT_SYS_EVENT))
		return;

	iowrite32(irq_mask & HSE_CH_MASK_ALL, &priv->regs->gsr);
}

/**
 * hse_mu_channel_available - check service channel status
 * @mu: MU instance handle
 * @channel: channel index
 *
 * The 16 LSB of MU instance FSR are used by HSE for signaling channel status
 * as busy after a service request has been sent, until the HSE reply is ready.
 *
 * Return: true for channel available, false for invalid index or channel busy
 */
static bool hse_mu_channel_available(void *mu, u8 channel)
{
	struct hse_mu_data *priv = mu;
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
 * hse_mu_next_pending_channel - find the next channel with pending message
 * @mu: MU instance handle
 *
 * Return: channel index, HSE_CHANNEL_INV if no message pending
 */
u8 hse_mu_next_pending_channel(void *mu)
{
	struct hse_mu_data *priv = mu;
	u32 rsrval = ioread32(&priv->regs->rsr) & HSE_CH_MASK_ALL;

	if (!ffs(rsrval))
		return HSE_CHANNEL_INV;

	return ffs(rsrval) - 1;
}

/**
 * hse_mu_msg_pending - check if a service request response is pending
 * @mu: MU instance handle
 * @channel: channel index
 *
 * Return: true for response ready, false otherwise
 */
static bool hse_mu_msg_pending(void *mu, u8 channel)
{
	struct hse_mu_data *priv = mu;
	u32 rsrval;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return false;

	rsrval = ioread32(&priv->regs->rsr) & BIT(channel);
	if (!rsrval)
		return false;

	return true;
}

/**
 * hse_mu_msg_send - send a message over MU (non-blocking)
 * @mu: MU instance handle
 * @channel: channel index
 * @msg: input message
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -EBUSY for selected channel busy
 */
int hse_mu_msg_send(void *mu, u8 channel, u32 msg)
{
	struct hse_mu_data *priv = mu;

	if (unlikely(!mu))
		return -EINVAL;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return -ECHRNG;

	if (unlikely(!hse_mu_channel_available(mu, channel))) {
		dev_dbg(priv->dev, "%s: channel %d busy\n", __func__, channel);
		return -EBUSY;
	}

	iowrite32(msg, &priv->regs->tr[channel]);

	return 0;
}

/**
 * hse_mu_msg_recv - read a message received over MU (non-blocking)
 * @mu: MU instance handle
 * @channel: channel index
 * @msg: output message
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -ENOMSG for no reply pending on selected channel
 */
int hse_mu_msg_recv(void *mu, u8 channel, u32 *msg)
{
	struct hse_mu_data *priv = mu;

	if (unlikely(!mu || !msg))
		return -EINVAL;

	if (unlikely(channel >= HSE_NUM_CHANNELS))
		return -ECHRNG;

	if (unlikely(!hse_mu_msg_pending(mu, channel))) {
		dev_dbg(priv->dev, "%s: no message pending on channel %d\n",
			__func__, channel);
		return -ENOMSG;
	}

	*msg = ioread32(&priv->regs->rr[channel]);

	return 0;
}

/**
 * hse_mu_init - initial setup of MU interface
 * @dev: parent device
 * @rx_isr: RX soft handler
 * @err_isr: sys event handler
 *
 * Return: MU instance handle on success, error code otherwise
 */
void *hse_mu_init(struct device *dev, irqreturn_t (*rx_isr)(int irq, void *dev),
		  irqreturn_t (*event_isr)(int irq, void *dev))
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hse_mu_data *mu;
	struct resource *res;
	int irq, err;
	u8 channel;
	u32 msg;

	if (unlikely(!dev))
		return ERR_PTR(-EINVAL);

	mu = devm_kzalloc(dev, sizeof(*mu), GFP_KERNEL);
	if (IS_ERR_OR_NULL(mu))
		return ERR_PTR(-ENOMEM);
	mu->dev = dev;

	/* map hardware register space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mu->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(mu->regs)) {
		dev_err(dev, "failed to map %s regs @%pR\n", HSE_MU_INST, res);
		return ERR_PTR(-ENOMEM);
	}
	spin_lock_init(&mu->reg_lock);

	/* disable all interrupt sources */
	hse_mu_irq_disable(mu, HSE_INT_ACK_REQUEST, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(mu, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);

	/* discard any pending messages */
	for (channel = 0; channel < HSE_NUM_CHANNELS; channel++)
		if (hse_mu_msg_pending(mu, channel))
			msg = ioread32(&mu->regs->rr[channel]);

	/* register RX and event handlers */
	irq = platform_get_irq_byname(pdev, HSE_RX_IRQ_NAME);
	err = devm_request_threaded_irq(dev, irq, NULL, rx_isr, IRQF_ONESHOT,
					HSE_RX_IRQ_NAME, dev);
	if (unlikely(err)) {
		dev_err(dev, "failed to register %s irq, line %d\n",
			HSE_RX_IRQ_NAME, irq);
		return ERR_PTR(-ENXIO);
	}

	irq = platform_get_irq_byname(pdev, HSE_ERR_IRQ_NAME);
	err = devm_request_threaded_irq(dev, irq, NULL, event_isr, IRQF_ONESHOT,
					HSE_ERR_IRQ_NAME, dev);
	if (unlikely(err)) {
		dev_err(dev, "failed to register %s irq, line %d\n",
			HSE_ERR_IRQ_NAME, irq);
		return ERR_PTR(-ENXIO);
	}

	return mu;
}
