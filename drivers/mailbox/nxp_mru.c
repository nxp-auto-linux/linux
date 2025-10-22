// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2025 NXP
 * NXP MRU (Message Receive Unit) Driver
 *
 * This driver provides mailbox communication where each message
 * consists of exactly one u32 notification value. The MRU hardware
 * does not support larger payloads- actual data exchange should
 * be done through shared memory, with MRU used only for notifications.
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/* MRU Hardware Configuration */

/* Hardware channels per MRU instance */
#define MRU_NUM_HW_CHANNELS      3
/* Mailboxes per hardware channel */
#define MRU_NUM_MAILBOXES        6
/* 2 virtual channels per mailbox: sender and receiver roles */
#define MRU_NUM_VIRT_CHANNELS    (MRU_NUM_HW_CHANNELS * MRU_NUM_MAILBOXES * 2)

/* MRU Register Offsets */

/* Configuration register 1 */
#define MRU_CFG1_OFFSET          0x4
/* Mailbox status register n */
#define MRU_MBSTATn_OFFSET(n)    ((n) * 0x10 + 0x8)

/* MRU CFG0 Register Bit Fields */

/* Channel Enable */
#define MRU_CFG0_CHE             BIT(0)
/* Interrupt Enable */
#define MRU_CFG0_IE              BIT(2)
/* Mailbox Enable bit shift */
#define MRU_CFG0_MBE_SHIFT       16
/* Mailbox Enable mask (6 bits) */
#define MRU_CFG0_MBE_MASK        0x3F

/* MRU MBSTAT Register Bit Fields */

/* Mailbox Flag bit shift */
#define MRU_MBSTAT_MBF_SHIFT     16
/* Mailbox Flag for mailbox mb */
#define MRU_MBSTAT_MBF(mb)       BIT(MRU_MBSTAT_MBF_SHIFT + (mb))

/* MRU Memory Layout */

/* Offset between channel register sets */
#define MRU_CHANNEL_OFFSET       0x10
/* Base offset for mailbox data registers */
#define MRU_MAILBOX_BASE         0x10000

/* MRU Virtual Channel Mapping */
#define MRU_VIRT_SENDER_OFFSET    0
#define MRU_VIRT_RECEIVER_OFFSET  1
#define MRU_VIRT_CHANNELS_PER_PHYSICAL  2

/* MRU Channel Roles */
enum mru_role {
	MRU_ROLE_SENDER = 0,
	MRU_ROLE_RECEIVER = 1,
};

struct mru_chan_data {
	spinlock_t chan_lock; /* channel configuration registers lock */
	unsigned int hw_channel;
	unsigned int mailbox;
	bool is_receiver;
};

struct mru_dev {
	struct mbox_controller mbox;
	struct mru_chan_data con_priv[MRU_NUM_VIRT_CHANNELS];
	struct device *dev;
	struct clk *ipg_clk;
	void __iomem *base;
	int irq;
	bool irq_requested;
};

/**
 * mru_mailbox_offset - Calculate mailbox register address
 * @ch: Hardware channel number (0-2)
 * @mb: Mailbox number within channel (0-5)
 *
 * Returns: Register offset for the specified mailbox
 */
static inline u32 mru_mailbox_offset(unsigned int ch, unsigned int mb)
{
	return ((ch + 1) * MRU_MAILBOX_BASE + mb * 4);
}

/**
 * mru_get_channel_base - Get channel configuration register base address
 * @priv: MRU device instance
 * @hw_channel: Hardware channel number (0-2)
 *
 * Returns: Virtual address of the channel's configuration register base
 */
static inline void __iomem *mru_get_channel_base(struct mru_dev *priv, unsigned int hw_channel)
{
	return priv->base + hw_channel * MRU_CHANNEL_OFFSET;
}

/**
 * mru_get_mb_idx - Calculate mailbox index
 * @hw_channel: Hardware channel number (0-2)
 * @mailbox: Mailbox number within channel (0-5)
 *
 * Returns: Physical index for the mailbox
 */
static inline unsigned int mru_get_mb_idx(unsigned int hw_channel, unsigned int mailbox)
{
	return hw_channel * MRU_NUM_MAILBOXES + mailbox;
}

/**
 * mru_get_virtual_channel - Calculate virtual channel for given role
 * @physical_idx: Physical mailbox index
 * @role: Channel role (MRU_ROLE_SENDER or MRU_ROLE_RECEIVER)
 *
 * Returns: Virtual channel index
 */
static inline unsigned int mru_get_virtual_channel(unsigned int physical_idx, enum mru_role role)
{
	return physical_idx * MRU_VIRT_CHANNELS_PER_PHYSICAL + role;
}

static int mru_send_data(struct mbox_chan *chan, void *data)
{
	struct mru_dev *priv = container_of(chan->mbox, struct mru_dev, mbox);
	struct mru_chan_data *chan_data = chan->con_priv;
	u32 *notification = data;
	u32 mb_addr;

	if (!notification || !chan_data)
		return -EINVAL;

	if (chan_data->is_receiver) {
		dev_err(priv->dev, "Cannot send on receiver channel\n");
		return -EOPNOTSUPP;
	}

	mb_addr = mru_mailbox_offset(chan_data->hw_channel, chan_data->mailbox);
	guard(spinlock_irqsave)(&chan_data->chan_lock);
	writel(*notification, priv->base + mb_addr);

	return 0;
}

static irqreturn_t mru_isr(int irq, void *data)
{
	unsigned int physical_idx, receiver_virt_ch;
	u32 mbstat, mb_flag, mb_addr, notification;
	struct mru_dev *priv = data;
	struct mbox_chan *chan;
	bool handled = false;
	int hw_ch, mb;

	for (hw_ch = 0; hw_ch < MRU_NUM_HW_CHANNELS; hw_ch++) {
		mbstat = readl(priv->base + MRU_MBSTATn_OFFSET(hw_ch));

		if (mbstat == 0)
			continue;

		handled = true;

		for (mb = 0; mb < MRU_NUM_MAILBOXES; mb++) {
			mb_flag = MRU_MBSTAT_MBF(mb);

			if (!(mbstat & mb_flag))
				continue;

			mb_addr = mru_mailbox_offset(hw_ch, mb);
			notification = readl(priv->base + mb_addr);
			/* Find receiver virtual channel */
			physical_idx = mru_get_mb_idx(hw_ch, mb);
			receiver_virt_ch = mru_get_virtual_channel(physical_idx,
					MRU_ROLE_RECEIVER);

			if (receiver_virt_ch < priv->mbox.num_chans && priv->mbox.chans) {
				chan = &priv->mbox.chans[receiver_virt_ch];
				if (chan && chan->cl && chan->cl->rx_callback)
					mbox_chan_received_data(chan, &notification);
			}
		}
		writel(mbstat, priv->base + MRU_MBSTATn_OFFSET(hw_ch));
	}
	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int mru_startup(struct mbox_chan *chan)
{
	struct mru_dev *priv = container_of(chan->mbox, struct mru_dev, mbox);
	struct mru_chan_data *chan_data = chan->con_priv;
	struct device *dev = priv->dev;
	void __iomem *chan_base;
	u32 mbe, cfg0, status;
	int ret;

	if (!chan_data) {
		dev_err(dev, "No channel data found\n");
		return -EINVAL;
	}

	/* No initialization needed if sender */
	if (!chan_data->is_receiver)
		return 0;

	if (priv->irq < 0) {
		dev_err(priv->dev, "IRQ required for receiver channels\n");
		return -EINVAL;
	}

	if (!priv->irq_requested) {
		ret = devm_request_irq(priv->dev, priv->irq, mru_isr, 0,
				dev_name(priv->dev), priv);
		if (ret) {
			dev_err(priv->dev, "Failed to request IRQ: %d\n", ret);
			return ret;
		}
		priv->irq_requested = true;
	}

	chan_base = mru_get_channel_base(priv, chan_data->hw_channel);
	mbe = BIT(MRU_CFG0_MBE_SHIFT + chan_data->mailbox);
	guard(spinlock_irqsave)(&chan_data->chan_lock);
	cfg0 = readl(chan_base);

	if (!(cfg0 & MRU_CFG0_CHE)) {
		cfg0 |= MRU_CFG0_CHE;
		writel(cfg0, chan_base);
	}

	/* Clear any pending mailbox status flags*/
	status = readl(priv->base + MRU_MBSTATn_OFFSET(chan_data->hw_channel));
	if (status)
		writel(status, priv->base + MRU_MBSTATn_OFFSET(chan_data->hw_channel));

	cfg0 |= MRU_CFG0_IE | mbe;
	writel(cfg0, chan_base);

	return 0;
}

static void mru_shutdown(struct mbox_chan *chan)
{
	struct mru_dev *priv = container_of(chan->mbox, struct mru_dev, mbox);
	struct mru_chan_data *chan_data = chan->con_priv;
	struct device *dev = priv->dev;
	void __iomem *chan_base;
	u32 mbe, cfg0;

	if (!chan_data) {
		dev_err(dev, "No channel data found\n");
		return;
	}

	if (!chan_data->is_receiver)
		return;

	chan_base = mru_get_channel_base(priv, chan_data->hw_channel);
	mbe = BIT(MRU_CFG0_MBE_SHIFT + chan_data->mailbox);
	guard(spinlock_irqsave)(&chan_data->chan_lock);
	cfg0 = readl(chan_base);
	cfg0 &= ~mbe;

	if ((cfg0 & (MRU_CFG0_MBE_MASK << MRU_CFG0_MBE_SHIFT)) == 0)
		cfg0 &= ~(MRU_CFG0_IE | MRU_CFG0_CHE);
	writel(cfg0, chan_base);
}

static const struct mbox_chan_ops mru_mbox_ops = {
	.startup = mru_startup,
	.send_data = mru_send_data,
	.shutdown = mru_shutdown,
};

static struct mbox_chan *mru_xlate(struct mbox_controller *mbox, const struct of_phandle_args *sp)
{
	struct mru_dev *priv = container_of(mbox, struct mru_dev, mbox);
	unsigned int hw_channel, mailbox, role, virt_chan, physical_idx;
	struct mru_chan_data *chan_data;

	/*
	 * Expected Device Tree usage:
	 * mboxes = <&mru_controller hw_channel mailbox role>;
	 *
	 * Arguments:
	 * args[0] = hw_channel (0-2): Hardware channel number
	 * args[1] = mailbox (0-5):    Mailbox number within the channel
	 * args[2] = role (0=sender, 1=receiver): Role for this specific channel
	 */
	if (sp->args_count != 3) {
		dev_err(mbox->dev, "Invalid argument count %d (expected 3)\n", sp->args_count);
		return ERR_PTR(-EINVAL);
	}

	hw_channel = sp->args[0];
	mailbox = sp->args[1];
	role = sp->args[2];

	if (hw_channel >= MRU_NUM_HW_CHANNELS) {
		dev_err(mbox->dev, "Invalid channel %d (max %d)\n",
				hw_channel, MRU_NUM_HW_CHANNELS);
		return ERR_PTR(-EINVAL);
	}

	if (mailbox >= MRU_NUM_MAILBOXES) {
		dev_err(mbox->dev, "Invalid mailbox %d (max %d)\n", mailbox, MRU_NUM_MAILBOXES);
		return ERR_PTR(-EINVAL);
	}

	if (role > MRU_ROLE_RECEIVER) {
		dev_err(mbox->dev, "Invalid role %d (0=sender, 1=receiver)\n", role);
		return ERR_PTR(-EINVAL);
	}

	/* Calculate virtual channel based on role */
	physical_idx = mru_get_mb_idx(hw_channel, mailbox);
	virt_chan = mru_get_virtual_channel(physical_idx, role);

	chan_data = priv->mbox.chans[virt_chan].con_priv;
	chan_data->hw_channel = hw_channel;
	chan_data->mailbox = mailbox;
	chan_data->is_receiver = (role == MRU_ROLE_RECEIVER);

	if (priv->mbox.chans[virt_chan].cl) {
		dev_err(mbox->dev, "Virtual channel %d already reserved for (%d,%d,%d)\n",
				virt_chan, hw_channel, mailbox, role);
		return ERR_PTR(-EBUSY);
	}

	return &priv->mbox.chans[virt_chan];
}

static const struct of_device_id mru_of_match[] = {
	{
		.compatible = "nxp,s32n79-mru",
	},
	{ } /* sentinel */
};
MODULE_DEVICE_TABLE(of, mru_of_match);

static int mru_probe(struct platform_device *pdev)
{
	struct mru_chan_data *cp;
	struct mru_dev *priv;
	int i, ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	priv->ipg_clk = devm_clk_get_enabled(&pdev->dev, "ipg");
	if (IS_ERR(priv->ipg_clk)) {
		dev_err(&pdev->dev, "Failed to get and enable ipg clock: %ld\n",
				PTR_ERR(priv->ipg_clk));
		return PTR_ERR(priv->ipg_clk);
	}

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		priv->irq = -1;

	priv->mbox.chans = devm_kzalloc(&pdev->dev,
			MRU_NUM_VIRT_CHANNELS * sizeof(*priv->mbox.chans),
			GFP_KERNEL);
	if (!priv->mbox.chans)
		return -ENOMEM;

	priv->mbox.dev = &pdev->dev;
	priv->mbox.num_chans = MRU_NUM_VIRT_CHANNELS;
	priv->mbox.ops = &mru_mbox_ops;
	priv->mbox.txdone_irq = false;
	priv->mbox.txdone_poll = false;
	priv->mbox.of_xlate = mru_xlate;

	for (i = 0; i < priv->mbox.num_chans; i++) {
		cp = &priv->con_priv[i];
		spin_lock_init(&cp->chan_lock);
		priv->mbox.chans[i].mbox = &priv->mbox;
		priv->mbox.chans[i].cl = NULL;
		priv->mbox.chans[i].con_priv = cp;
	}

	platform_set_drvdata(pdev, priv);

	ret = devm_mbox_controller_register(&pdev->dev, &priv->mbox);
	if (ret) {
		dev_err(&pdev->dev, "MRU: failed to register mailbox controller (%d)\n", ret);
		return ret;
	}
	return 0;
}

static struct platform_driver mru_driver = {
	.probe = mru_probe,
	.driver = {
		.name = "nxp_mru",
		.of_match_table = mru_of_match,
	},
};

module_platform_driver(mru_driver);
MODULE_AUTHOR("Georgiana Ortila <georgiana.ortila@nxp.com>");
MODULE_DESCRIPTION("Message Receive Unit driver");
MODULE_LICENSE("Dual BSD/GPL");
