// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020-2023 NXP
 *
 */

#include <asm/barrier.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include "ddr.h"

#define TIMER_INTERVAL		1000	/* 1 second */
#define POLLING_NEEDED_KEY	0x1

struct ddr_priv {
	void __iomem *ddrc_base;
	void __iomem *perf_base;
	void __iomem *membase_reg;
	struct device *dev;
	struct timer_list timer;
	bool shutdown;

	/* When the timer is active, reading/writing the 'shutdown'
	 * flag will be done in a critical section.
	 * This way the timer cannot be reset before
	 * this flag is set.
	 */
	spinlock_t shutdown_access;
};

static void reset_timer(struct ddr_priv *data)
{
	mod_timer(&data->timer, jiffies +
			msecs_to_jiffies(TIMER_INTERVAL));
}

static void cleanup_ddr_errata(struct ddr_priv *data)
{
	if (!data)
		return;

	data->shutdown = true;
	del_timer(&data->timer);
}

/* Read lpddr4 mode register with given rank and index */
u32 read_lpddr4_mr(u8 mr_rank, u8 mr_index, void __iomem *ddrc_base,
		   void __iomem *perf_base)
{
	u32 tmp32;
	u8 succesive_reads = 0;

	/* Set MRR_DDR_SEL_REG to 0x1 to enable LPDDR4 mode */
	tmp32 = readl(perf_base + OFFSET_MRR_0_DATA_REG_ADDR);
	writel((tmp32 | MRR_0_DDR_SEL_REG_MASK), perf_base +
			OFFSET_MRR_0_DATA_REG_ADDR);

	/*
	 * Ensure no MR transaction is in progress:
	 * mr_wr_busy signal must be low
	 */
	do {
		tmp32 = readl(ddrc_base + OFFSET_DDRC_MRSTAT);
		if ((tmp32 & DDRC_MRSTAT_MR_BUSY) == DDRC_MRSTAT_MR_NOT_BUSY)
			succesive_reads++;
		else
			succesive_reads = 0;
	} while (succesive_reads != DDRC_REQUIRED_MRSTAT_READS);

	/* Set MR_TYPE = 0x1 (Read) and MR_RANK = desired rank */
	tmp32 = readl(ddrc_base + OFFSET_DDRC_MRCTRL0);
	tmp32 |= DDRC_MRCTRL0_MR_TYPE_READ;
	tmp32 = (tmp32 & ~(DDRC_MRCTRL0_RANK_ACCESS_FIELD <<
			   DDRC_MRCTRL0_RANK_ACCESS_POS)) |
		(mr_rank << DDRC_MRCTRL0_RANK_ACCESS_POS);
	writel(tmp32, ddrc_base + OFFSET_DDRC_MRCTRL0);

	/* Configure MR address: MRCTRL1[8:15] */
	tmp32 = readl(ddrc_base + OFFSET_DDRC_MRCTRL1);
	tmp32 = (tmp32 & ~(DDRC_MRCTRL1_MR_ADDRESS_FIELD <<
			   DDRC_MRCTRL1_MR_ADDRESS_POS)) |
		((uint16_t)mr_index << DDRC_MRCTRL1_MR_ADDRESS_POS);
	writel(tmp32, ddrc_base + OFFSET_DDRC_MRCTRL1);

	dsb(sy);

	/* Initiate MR transaction: MR_WR = 0x1 */
	tmp32 = readl(ddrc_base + OFFSET_DDRC_MRCTRL0);
	writel(tmp32 | (DDRC_MRCTRL0_WR_ENGAGE << DDRC_MRCTRL0_WR_ENGAGE_POS),
	       ddrc_base + OFFSET_DDRC_MRCTRL0);

	/* Wait until MR transaction completed */
	do {
		tmp32 = readl(ddrc_base + OFFSET_DDRC_MRSTAT);
		if ((tmp32 & DDRC_MRSTAT_MR_BUSY) == DDRC_MRSTAT_MR_NOT_BUSY)
			succesive_reads++;
		else
			succesive_reads = 0;
	} while (succesive_reads != DDRC_REQUIRED_MRSTAT_READS);

	return readl(perf_base + OFFSET_MRR_1_DATA_REG_ADDR);
}

/*
 * Read Temperature Update Flag from lpddr4 MR4 register.
 */
u8 read_TUF(void __iomem *ddrc_base, void __iomem *perf_base)
{
	u32 mr4_val, derateen;
	u16 mr4_die_1;
	u8 derate_byte;

	mr4_val = read_lpddr4_mr(DDRC_MRCTRL0_RANK_0, MR4_IDX, ddrc_base,
				 perf_base);
	mr4_die_1 = (u16)(mr4_val & MR4_MASK);

	/* Check which byte of the MRR data is used for derating */
	derateen = readl(ddrc_base + OFFSET_DDRC_DERATEEN);
	derate_byte = (derateen >> DDRC_DERATEEN_DERATE_BYTE_SHIFT) &
		       DDRC_DERATEEN_DERATE_BYTE_MASK;

	if (derate_byte == DDRC_DERATE_BYTE_1)
		mr4_die_1 = mr4_die_1 >> BYTE_SHIFT;

	return (u8)(mr4_die_1 & REF_RATE_MASK);
}

static void execute_polling(struct timer_list *t)
{
	struct ddr_priv *data = from_timer(data, t, timer);
	void __iomem *ddrc_base = data->ddrc_base;
	void __iomem *perf_base = data->perf_base;
	int ret;

	spin_lock_bh(&data->shutdown_access);
	if (data->shutdown) {
		spin_unlock_bh(&data->shutdown_access);
		return;
	}

	ret = poll_derating_temp_errata(ddrc_base, perf_base);
	if (ret) {
		cleanup_ddr_errata(data);
		spin_unlock_bh(&data->shutdown_access);
	} else {
		spin_unlock_bh(&data->shutdown_access);
		reset_timer(data);
	}
}

static void enable_polling(struct ddr_priv *data)
{
	if (readl(data->membase_reg) != POLLING_NEEDED_KEY)
		return;

	data->shutdown = false;
	/* Start the timer. */
	timer_setup(&data->timer, execute_polling, 0);
	reset_timer(data);
}

static void __iomem *get_perf_map_by_phandle(struct device *dev)
{
	struct device_node *perf_node;
	void __iomem *perf_regs;

	perf_node = of_parse_phandle(dev->of_node, "perf-phandle", 0);
	if (!perf_node) {
		dev_info(dev, "perf-phandle node not found\n");
		return (void __iomem *)ERR_PTR(-ENODEV);
	}

	perf_regs = of_iomap(perf_node, 0);
	if (!perf_regs) {
		dev_warn(dev, "Cannot map PERF registers\n");
		return (void __iomem *)ERR_PTR(-ENOMEM);
	}
	of_node_put(perf_node);

	return perf_regs;
}

static int ddr_probe(struct platform_device *pdev)
{
	struct ddr_priv *data;
	struct device_node *np;
	struct resource res;
	struct device *dev = &pdev->dev;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (unlikely(!data))
		return -ENOMEM;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "No 'memory_region' entry specified\n");
		return -EINVAL;
	}

	if (of_address_to_resource(np, 0, &res)) {
		dev_err(dev, "No memory address assigned to the region");
		return -EINVAL;
	}

	data->membase_reg = devm_ioremap_resource(dev, &res);
	if (IS_ERR(data->membase_reg))
		return PTR_ERR(data->membase_reg);

	data->ddrc_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->ddrc_base))
		return PTR_ERR(data->ddrc_base);

	data->perf_base = get_perf_map_by_phandle(dev);
	data->dev = dev;

	spin_lock_init(&data->shutdown_access);
	platform_set_drvdata(pdev, data);

	enable_polling(data);

	return 0;
}

static int ddr_remove(struct platform_device *pdev)
{
	struct ddr_priv *data = platform_get_drvdata(pdev);

	spin_lock_bh(&data->shutdown_access);
	if (!data->shutdown)
		cleanup_ddr_errata(data);
	spin_unlock_bh(&data->shutdown_access);

	dev_info(data->dev, "device removed\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int s32cc_ddr_suspend(struct device *dev)
{
	struct ddr_priv *data = dev_get_drvdata(dev);
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);

	spin_lock_bh(&data->shutdown_access);
	if (!data->shutdown)
		cleanup_ddr_errata(data);
	spin_unlock_bh(&data->shutdown_access);

	platform_set_drvdata(pdev, data);

	return 0;
}

static int s32cc_ddr_resume(struct device *dev)
{
	struct ddr_priv *data = dev_get_drvdata(dev);
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);

	enable_polling(data);

	platform_set_drvdata(pdev, data);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(s32cc_ddr_errata_pm_ops,
		s32cc_ddr_suspend, s32cc_ddr_resume);

static const struct of_device_id s32cc_ddr_errata_dt_ids[] = {
	{
		.compatible = "nxp,s32cc-ddr",
	}, {}
};
MODULE_DEVICE_TABLE(of, s32cc_ddr_errata_dt_ids);

static struct platform_driver s32cc_ddr_errata_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = s32cc_ddr_errata_dt_ids,
		.pm = &s32cc_ddr_errata_pm_ops,
	},
	.probe = ddr_probe,
	.remove = ddr_remove,
};

module_platform_driver(s32cc_ddr_errata_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NXP S32CC DDR Errata ERR050543 Driver");
