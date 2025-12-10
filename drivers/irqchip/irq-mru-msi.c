// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2025 NXP
 *
 * This driver is designed to expose a set of Message Receive Units (MRUs)
 * as an MSI-X controller. Each MRU is organized into multiple channels, and
 * each channel contains a mailbox. Every channel has an associated IRQ line.
 *
 * The MRUs associated with the Cortex-A subsystem have one SPI that multiplexes
 * notifications from all three channels. To manage this efficiently, all
 * channels across the MRUs are represented in a bitmap, structured as follows:
 *
 *                ----------------||---------------||-----------------
 * Bitmap bits    | 0 | 1 | 2 | 3 || 4 | 5 | 6 | 7 || 8 | 9 | 10 | 11 |
 *                ----------------||---------------||-----------------
 * MRU Index        0   1   2   3 || 0   1   2   3 || 0   1    2    3
 * Channel index    0   0   0   0 || 1   1   1   1 || 2   2    2    2
 *
 * Assumptions for the example above:
 *  - Number of MRUs: 4
 *  - Number of channels per MRU: 3
 *
 * To achieve better IRQ load balancing, the driver prioritizes channel index
 * during linear allocation of MSI slots. This ensures that when multiple MSIs
 * are allocated, they are distributed across different MRUs, rather than
 * concentrating on channels within the same MRU.
 */
#include <linux/bitmap.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "irq-msi-lib.h"

#define CHANS_PER_MRU			3U

#define MRU_CHANNEL_OFFSET		0x10U

/* Mailbox status register n */
#define MRU_CH_MBSTATn_OFFSET(n)	((n) * MRU_CHANNEL_OFFSET + 0x8U)
#define MRU_CH_MBSTAT_OFFSET		0x8U

/* MRU CFG0 Register Bit Fields */
#define MRU_CFG0_MBE0			BIT(16U)
#define MRU_CFG0_CHE			BIT(0U)
#define MRU_CFG0_IE			BIT(2U)

#define MRU_MAILBOX_BASE		0x10000U

struct mru_msi;

struct mru {
	struct cpumask ch_affinity[CHANS_PER_MRU];
	struct mru_msi *parent;
	void __iomem *base;
	phys_addr_t addr;
	int index;
	int irq;
};

struct mru_msi {
	struct irq_domain *msi_domain;
	struct irq_domain *parent;
	unsigned long *chans_bitmap;
	struct device *dev;
	struct mru *mrus;
	spinlock_t chans_lock;  /* Protects chans_bitmap */
	u32 nmrus;
	u32 nchans;
};

static unsigned int mru_get_channels_nbits(struct mru_msi *priv)
{
	return priv->nchans ? fls(priv->nchans) : 0;
}

static unsigned long mru_bitmappos2mruidx(struct mru_msi *priv,
					  unsigned long bitmap_pos)
{
	return bitmap_pos % priv->nmrus;
}

static unsigned long mru_bitmappos2chanidx(struct mru_msi *priv,
					   unsigned long bitmap_pos)
{
	return bitmap_pos / priv->nmrus;
}

static unsigned int mru_hwirq2bitmappos(struct mru_msi *priv,
					unsigned long hw_irq)
{
	unsigned int ch_bits = mru_get_channels_nbits(priv);

	return hw_irq & GENMASK(ch_bits - 1, 0);
}

static unsigned int mru_hwirq2cookie(struct mru_msi *priv,
				     unsigned long hw_irq)
{
	unsigned int ch_bits = mru_get_channels_nbits(priv);
	unsigned int cookie;

	cookie = (hw_irq >> ch_bits) & GENMASK(31u - ch_bits, 0);

	return cookie;
}

static void mru_hwirq2chan(struct mru_msi *priv, irq_hw_number_t hw_irq,
			   unsigned long *mru_idx, unsigned long *chan_idx)
{
	unsigned long bitmap_pos;

	bitmap_pos = mru_hwirq2bitmappos(priv, hw_irq);

	if (mru_idx != NULL)
		*mru_idx = mru_bitmappos2mruidx(priv, bitmap_pos);

	if (chan_idx != NULL)
		*chan_idx = mru_bitmappos2chanidx(priv, bitmap_pos);
}

static void mru_irqd2chan(struct mru_msi *priv, struct irq_data *data,
			     unsigned long *mru_idx, unsigned long *chan_idx)
{
	mru_hwirq2chan(priv, irqd_to_hwirq(data), mru_idx, chan_idx);
}

/**
 * The hardware interrupt must be unique at the IRQ Controller level. Based on
 * its value, it should allow easy extraction of the associated MRU channel or
 * mailbox. For this reason, the hwirq value encodes both the bitmap position
 * of the channel and a cookie.
 *
 * For MSI-X interrupts, the bitmap position alone is sufficient to handle the
 * interrupt. However, for MSI interrupts, a cookie is required that represents
 * the ID of the virtual interrupt mapped to the bitmap channel.
 *
 * Consequently, the hwirq encoding allocates:
 *   - The most significant bits (MSBs) to the cookie
 *   - The least significant bits (LSBs) to the bitmap position
 *
 * The number of bits reserved for bitmap channel encoding depends on the
 * number of MRUs listed in the device tree.
 */
static unsigned int mru_build_hwirq(struct mru_msi *priv,
				    unsigned int bitmap_pos,
				    unsigned int cookie)
{
	unsigned int ch_bits = mru_get_channels_nbits(priv);
	unsigned int hw_irq;

	hw_irq = (cookie & GENMASK(31u - ch_bits, 0)) << ch_bits;
	hw_irq |= bitmap_pos;

	return hw_irq;
}

static int mru_chan2hwirq(struct mru_msi *priv, unsigned long mru_idx,
			   unsigned long chan_idx, unsigned int cookie,
			   unsigned long *hw_irq)
{
	unsigned long bitmap_pos;

	if (unlikely(check_mul_overflow(priv->nmrus, chan_idx, &bitmap_pos)))
		return -ERANGE;

	if (unlikely(check_add_overflow(bitmap_pos, mru_idx, &bitmap_pos)))
		return -ERANGE;

	*hw_irq = mru_build_hwirq(priv, bitmap_pos, cookie);

	return 0;
}

static void __iomem *mru_get_base(struct mru_msi *priv,
				  unsigned long bitmap_pos)
{
	unsigned long idx = mru_bitmappos2mruidx(priv, bitmap_pos);

	if (idx >= priv->nmrus)
		return NULL;

	return priv->mrus[idx].base;
}

static void __iomem *mru_get_chan_base(void __iomem *mru_base,
				       unsigned long mru_chan)
{
	unsigned long offset;
	uintptr_t chan_base;

	if (unlikely(check_mul_overflow(MRU_CHANNEL_OFFSET, mru_chan, &offset)))
		return NULL;

	if (unlikely(check_add_overflow((uintptr_t)mru_base, offset, &chan_base)))
		return NULL;

	return (void __iomem *)chan_base;
}

static void __iomem *mru_bitmappos2chan_base(struct mru_msi *priv,
					     unsigned long bitmap_pos)
{
	unsigned long chn_idx;
	void __iomem *mru_base;

	chn_idx = mru_bitmappos2chanidx(priv, bitmap_pos);

	mru_base = mru_get_base(priv, bitmap_pos);
	if (!mru_base)
		return NULL;

	return mru_get_chan_base(mru_base, chn_idx);
}

static int mru_irq_get_chan_base(struct irq_data *data,
				 void __iomem **chan_base)
{
	struct mru_msi *priv = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hw_irq = irqd_to_hwirq(data);
	unsigned long bitmap_pos;

	bitmap_pos = mru_hwirq2bitmappos(priv, hw_irq);

	*chan_base = mru_bitmappos2chan_base(priv, bitmap_pos);
	if (!*chan_base)
		return -EINVAL;

	return 0;
}

static void mru_set_chan_irq(void __iomem *chan_base, bool enable)
{
	u32 cfg0;

	cfg0 = readl(chan_base);

	if (enable)
		cfg0 |= MRU_CFG0_IE;
	else
		cfg0 &= ~MRU_CFG0_IE;

	writel(cfg0, chan_base);
}

static void mru_set_chan_status(void __iomem *chan_base, bool enable)
{
	u32 cfg0, mask;

	cfg0 = readl(chan_base);

	mask = MRU_CFG0_CHE;
	if (enable)
		cfg0 |= mask;
	else
		cfg0 &= ~mask;
	writel(cfg0, chan_base);

	/**
	 * Enable only mailbox 0, as this is the only mailbox the driver
	 * will use from the entire channel to allow masking of the
	 * channel interrupt.
	 */
	mask = MRU_CFG0_MBE0;
	if (enable)
		cfg0 |= mask;
	else
		cfg0 &= ~mask;

	writel(cfg0, chan_base);
}

static void mru_enable_ch_irq(void __iomem *chan_base)
{
	mru_set_chan_status(chan_base, true);
	mru_set_chan_irq(chan_base, true);
}

static void mru_disable_ch_irq(void __iomem *chan_base)
{
	mru_set_chan_irq(chan_base, false);
	mru_set_chan_status(chan_base, false);
}

static void mru_msi_parent_ack_irq(struct irq_data *data)
{
	struct mru_msi *priv = irq_data_get_irq_chip_data(data);
	void __iomem *chan_base;
	int ret;

	ret = mru_irq_get_chan_base(data, &chan_base);
	if (ret) {
		dev_err(priv->dev, "Invalid MRU instance for HW irq %lu\n",
			irqd_to_hwirq(data));
		return;
	}

	writel(MRU_CFG0_MBE0, chan_base + MRU_CH_MBSTAT_OFFSET);
}

static int mru_msi_parent_set_affinity(struct irq_data *data,
				       const struct cpumask *dest, bool force)
{
	struct mru_msi *priv = irq_data_get_irq_chip_data(data);
	struct cpumask mru_aff = CPU_MASK_NONE;
	unsigned long mru_idx, chan_idx;
	struct mru *mru;
	int mru_irq, i;

	mru_irqd2chan(priv, data, &mru_idx, &chan_idx);

	mru = &priv->mrus[mru_idx];
	mru_irq = mru->irq;

	if (mru_irq < 0)
		return -EINVAL;

	cpumask_copy(&mru->ch_affinity[chan_idx], dest);

	/**
	 *  All MRU channels share the same IRQ line.
	 *  To accommodate all requests, the IRQ line affinity will be
	 *  computed as a logical OR of the CPU masks specified for each
	 *  channel within the MRU.
	 */
	for (i = 0; i < CHANS_PER_MRU; i++)
		if (i != chan_idx)
			cpumask_or(&mru_aff, &mru_aff, &mru->ch_affinity[i]);

	if (force) {
		// Restrict the choices to only those that were requested.
		if (!cpumask_and(&mru_aff, &mru_aff,
				 &mru->ch_affinity[chan_idx])) {
			// Revert the changes if the CPU set is empty
			cpumask_copy(&mru_aff, dest);
		}
	} else {
		cpumask_or(&mru_aff, &mru_aff, &mru->ch_affinity[chan_idx]);
	}

	cpumask_and(&mru_aff, &mru_aff, cpu_online_mask);

	irq_data_update_effective_affinity(data,  &mru_aff);
	irq_set_affinity((unsigned int)mru_irq, &mru_aff);

	return IRQ_SET_MASK_OK_DONE;
}

static int mru_get_mailbox_addr(struct mru_msi *priv, unsigned long mru_idx,
				unsigned long mru_chan,
				phys_addr_t *mb_phys_addr,
				void __iomem **mb_addr)
{
	uintptr_t mru_addr, mb_virt_addr;
	phys_addr_t mru_phys_addr;
	unsigned long tmp;

	if (unlikely(check_add_overflow(mru_chan, 1, &tmp))) {
		dev_err(priv->dev, "Overflow detected for mru%ld, chan%ld line:%d\n",
			mru_idx, mru_chan, __LINE__);
		return -E2BIG;
	}

	if (unlikely(check_mul_overflow(tmp, MRU_MAILBOX_BASE, &tmp))) {
		dev_err(priv->dev, "Overflow detected for mru%ld, chan%ld line:%d\n",
			mru_idx, mru_chan, __LINE__);
		return -E2BIG;
	}

	if (mb_phys_addr) {
		mru_phys_addr = priv->mrus[mru_idx].addr;
		if (unlikely(check_add_overflow(mru_phys_addr, tmp,
						mb_phys_addr))) {
			dev_err(priv->dev, "Overflow detected for mru%ld, chan%ld line:%d\n",
				mru_idx, mru_chan, __LINE__);
			return -E2BIG;
		}
	}

	if (mb_addr) {
		mru_addr = (uintptr_t)priv->mrus[mru_idx].base;
		if (unlikely(check_add_overflow(mru_addr, tmp,
						&mb_virt_addr))) {
			dev_err(priv->dev, "Overflow detected for mru%ld, chan%ld line:%d\n",
				mru_idx, mru_chan, __LINE__);
			return -E2BIG;
		}
		*mb_addr = (void __iomem *)mb_virt_addr;
	}

	return 0;
}

static void mru_msi_parent_compose_msg(struct irq_data *data,
				       struct msi_msg *msg)
{
	struct mru_msi *priv = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hw_irq = irqd_to_hwirq(data);
	unsigned long mru_idx, mru_chan;
	phys_addr_t mb_addr;
	int ret;

	mru_hwirq2chan(priv, hw_irq, &mru_idx, &mru_chan);

	ret = mru_get_mailbox_addr(priv, mru_idx, mru_chan, &mb_addr, NULL);
	if (ret)
		return;

	msg->address_hi = upper_32_bits(mb_addr);
	msg->address_lo = lower_32_bits(mb_addr);

	msg->data = mru_hwirq2cookie(priv, hw_irq);
}

static struct irq_chip mru_msi_parent_chip = {
	.name			= "MRU",
	.irq_ack		= mru_msi_parent_ack_irq,
	.irq_compose_msi_msg	= mru_msi_parent_compose_msg,
	.irq_set_affinity	= mru_msi_parent_set_affinity,
};

static void mru_msi_domain_irq_free(struct irq_domain *domain,
				    unsigned int virq, unsigned int nr_irqs);

static int mru_msi_domain_irq_alloc(struct irq_domain *domain,
				    unsigned int virq,
				    unsigned int nr_irqs,
				    void *args)
{
	struct mru_msi *priv = domain->host_data;
	unsigned long bitmap_pos;
	void __iomem *chan_base;
	unsigned int hw_irq, i;
	int pos, ret = 0;

	/* Locate an available set of channels for use */
	scoped_guard(spinlock, &priv->chans_lock) {
		pos = bitmap_find_next_zero_area(priv->chans_bitmap,
						 priv->nchans, 0, nr_irqs, 0);
		if ((pos >= priv->nchans) || (pos < 0)) {
			dev_err(priv->dev, "No space left in bitmap for %d IRQs\n",
				nr_irqs);
			return -ENOSPC;
		}

		bitmap_set(priv->chans_bitmap, pos, nr_irqs);
	}

	for (i = 0; i < nr_irqs; i++) {
		hw_irq = mru_build_hwirq(priv, pos, i);

		irq_domain_set_info(domain, virq + i, hw_irq,
				    &mru_msi_parent_chip, priv,
				    handle_edge_irq, NULL, NULL);
	}

	bitmap_pos = pos;

	chan_base = mru_bitmappos2chan_base(priv, bitmap_pos);
	if (!chan_base) {
		ret = -EINVAL;
		goto exit;
	}

	mru_enable_ch_irq(chan_base);

exit:
	if (ret)
		mru_msi_domain_irq_free(domain, virq, nr_irqs);

	return ret;
}

static void mru_msi_domain_irq_free(struct irq_domain *domain,
				    unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct mru_msi *priv = irq_data_get_irq_chip_data(d);
	unsigned long bitmap_pos;
	void __iomem *chan_base;
	irq_hw_number_t hw_irq;

	hw_irq = irqd_to_hwirq(d);

	bitmap_pos = mru_hwirq2bitmappos(priv, hw_irq);

	chan_base = mru_bitmappos2chan_base(priv, bitmap_pos);
	if (!chan_base)
		return;

	mru_disable_ch_irq(chan_base);

	irq_domain_free_irqs_common(domain, virq, nr_irqs);

	scoped_guard(spinlock, &priv->chans_lock) {
		bitmap_clear(priv->chans_bitmap, bitmap_pos, nr_irqs);
	}
}

static const struct irq_domain_ops mru_msi_domain_ops = {
	.select	= msi_lib_irq_domain_select,
	.alloc	= mru_msi_domain_irq_alloc,
	.free	= mru_msi_domain_irq_free,
};

static struct msi_domain_info mru_msi_domain_info = {
	.flags	= MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		  MSI_FLAG_PCI_MSIX,
	.chip = &(struct irq_chip) {
		.name			= "MRU-MSI-X",
		.irq_ack		= irq_chip_ack_parent,
		.irq_set_affinity	= irq_chip_set_affinity_parent,
	},
};

static int mru_msi_domains_init(struct mru_msi *priv, struct device *dev)
{
	struct fwnode_handle *fwnodes = dev_fwnode(dev);

	priv->parent = irq_domain_create_linear(fwnodes, priv->nchans,
						&mru_msi_domain_ops, priv);
	if (!priv->parent) {
		dev_err(dev, "Failed to create IRQ domain\n");
		return -ENOMEM;
	}

	priv->msi_domain = pci_msi_create_irq_domain(fwnodes,
						     &mru_msi_domain_info,
						     priv->parent);
	if (!priv->msi_domain) {
		dev_err(priv->dev, "Failed to create MSI domain\n");
		irq_domain_remove(priv->parent);
		return -ENOMEM;
	}

	return 0;
}

static void mru_msi_parent_irq_handler(struct irq_desc *desc)
{
	struct mru *mru = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct mru_msi *priv = mru->parent;
	unsigned long hw_irq = 0U;
	void __iomem *mb_addr;
	unsigned int cookie;
	u32 mbstat, i;
	int ret;

	if (unlikely(mru->index < 0))
		dev_err_ratelimited(priv->dev, "Invalid MRU index %d\n",
				    mru->index);

	chained_irq_enter(chip, desc);

	for (i = 0; i < CHANS_PER_MRU; i++) {
		mbstat = readl(mru->base + MRU_CH_MBSTATn_OFFSET(i));

		if (mbstat == 0)
			continue;

		ret = mru_get_mailbox_addr(priv, mru->index, i,
					   NULL, &mb_addr);
		if (ret)
			continue;

		cookie = readl(mb_addr);

		ret = mru_chan2hwirq(priv, mru->index, i, cookie, &hw_irq);
		if (ret) {
			dev_err_ratelimited(priv->dev,
					    "failed to compute irq for MRU%d chan %d\n",
					    mru->index, i);
			continue;
		}

		ret = generic_handle_domain_irq(priv->parent, hw_irq);
		if (unlikely(ret))
			dev_err_ratelimited(priv->dev,
					    "unexpected IRQ for MRU%d chan %d\n",
					    mru->index, i);
	}

	chained_irq_exit(chip, desc);
}

static void detach_chained_handlers(struct mru_msi *priv)
{
	struct mru *mru;
	int i;

	for (i = 0; i < priv->nmrus; i++) {
		mru = &priv->mrus[i];

		if (mru->irq > 0)
			irq_set_chained_handler_and_data(mru->irq, NULL,
							 NULL);
	}
}

static int mru_msi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mru_reg = NULL;
	struct mru_msi *priv;
	int ret = 0, i, j;
	struct mru *mru;
	char *irq_name;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	while (platform_get_resource(pdev, IORESOURCE_MEM, priv->nmrus))
		priv->nmrus++;

	priv->mrus = devm_kcalloc(dev, priv->nmrus,
				  sizeof(*priv->mrus), GFP_KERNEL);
	if (!priv->mrus)
		return -ENOMEM;

	priv->nchans = priv->nmrus * CHANS_PER_MRU;

	priv->chans_bitmap = devm_bitmap_zalloc(dev, priv->nchans, GFP_KERNEL);
	if (!priv->chans_bitmap)
		return -ENOMEM;

	spin_lock_init(&priv->chans_lock);
	platform_set_drvdata(pdev, priv);

	for (i = 0; i < priv->nmrus; i++) {
		mru = &priv->mrus[i];

		for (j = 0; j < CHANS_PER_MRU; j++)
			cpumask_clear(&mru->ch_affinity[j]);

		mru->base = devm_platform_get_and_ioremap_resource(pdev, i,
								   &mru_reg);
		if (IS_ERR(mru->base))
			return PTR_ERR(mru->base);

		mru->addr = mru_reg->start;
		mru->parent = priv;
		mru->index = i;

		irq_name = devm_kasprintf(dev, GFP_KERNEL, "mru@%llx", mru->addr);
		if (!irq_name)
			return -ENOMEM;

		mru->irq = platform_get_irq(pdev, i);
		if (mru->irq < 0) {
			dev_err(dev, "Failed to get interrupt for entry %d\n", i);
			ret = mru->irq;
			break;
		}

		irq_set_chained_handler_and_data(mru->irq,
						 mru_msi_parent_irq_handler,
						 mru);
	}

	if (ret)
		goto exit;

	ret = mru_msi_domains_init(priv, dev);

exit:
	if (ret)
		detach_chained_handlers(priv);

	return ret;
}

static void mru_msi_remove(struct platform_device *pdev)
{
	struct mru_msi *priv = platform_get_drvdata(pdev);

	detach_chained_handlers(priv);

	irq_domain_remove(priv->parent);
	irq_domain_remove(priv->msi_domain);

	platform_set_drvdata(pdev, NULL);
}

static const struct of_device_id mru_msi_of_match[] = {
	{
		.compatible = "nxp,s32n79-mru-msi",
	},
	{ } /* sentinel */
};
MODULE_DEVICE_TABLE(of, mru_msi_of_match);

static struct platform_driver mru_msi_driver = {
	.probe = mru_msi_probe,
	.remove = mru_msi_remove,
	.driver = {
		.name = "nxp_msi_mru",
		.of_match_table = mru_msi_of_match,
	},
};

module_platform_driver(mru_msi_driver);
MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("MSI-X using MRU");
MODULE_LICENSE("Dual BSD/GPL");
