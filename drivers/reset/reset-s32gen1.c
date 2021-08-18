// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright 2021 NXP
 */
#include <dt-bindings/reset/s32g-scmi-reset.h>
#include <dt-bindings/reset/s32gen1-scmi-reset.h>
#include <dt-bindings/reset/s32r45-scmi-reset.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/processor.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/mfd/s32gen1-mc_me.h>
#include <linux/mfd/s32gen1-mc_rgm.h>
#include <s32/s32-gen1/clk.h>
#include <s32/s32-gen1/rdc.h>

#define S32GEN1_RESET_TIMEOUT_MS	1

#define PART_RESET(SCMI_ID, ID) \
{ .part = true, .scmi_id = (SCMI_ID), .rst_id = (ID)}

#define PERIPH_RESET(SCMI_ID, ID) \
{ .part = false, .scmi_id = (SCMI_ID), .rst_id = (ID)}

struct reset_entry {
	unsigned long scmi_id;
	unsigned long rst_id;
	bool part;
};

struct reset_data {
	const struct reset_entry *resets;
	size_t n_resets;
};

struct s32gen1_reset {
	void __iomem			*rgm;
	struct regmap			*mc_me;
	void __iomem			*rdc;
	struct reset_controller_dev	rcdev;
	const struct reset_data *data;
};

static const struct reset_entry s32gen1_reset_table[] = {
	/* Partitions */
	PART_RESET(S32GEN1_SCMI_RST_PART0, 0),
	PART_RESET(S32GEN1_SCMI_RST_PART1, 1),
	PART_RESET(S32GEN1_SCMI_RST_PART2, 2),
	PART_RESET(S32GEN1_SCMI_RST_PART3, 3),
	/* Peripherals */
	PERIPH_RESET(S32GEN1_SCMI_RST_CM7_0, 0),
	PERIPH_RESET(S32GEN1_SCMI_RST_CM7_1, 1),
	PERIPH_RESET(S32GEN1_SCMI_RST_CM7_2, 2),
	PERIPH_RESET(S32GEN1_SCMI_RST_DDR, 3),
	PERIPH_RESET(S32GEN1_SCMI_RST_PCIE0, 4),
	PERIPH_RESET(S32GEN1_SCMI_RST_SERDES0, 5),
	PERIPH_RESET(S32GEN1_SCMI_RST_PCIE1, 16),
	PERIPH_RESET(S32GEN1_SCMI_RST_SERDES1, 17),
	PERIPH_RESET(S32GEN1_SCMI_RST_A53_0, 65),
	PERIPH_RESET(S32GEN1_SCMI_RST_A53_1, 66),
	PERIPH_RESET(S32GEN1_SCMI_RST_A53_2, 67),
	PERIPH_RESET(S32GEN1_SCMI_RST_A53_3, 68),
};

static const struct reset_entry s32g2_reset_table[] = {
	PART_RESET(S32G_SCMI_RST_PFE, 2),
	PART_RESET(S32G_SCMI_RST_LLCE, 3),
};

static const struct reset_entry s32r45_reset_table[] = {
	PERIPH_RESET(S32R45_SCMI_RST_LAX, 128),
	PERIPH_RESET(S32R45_SCMI_RST_RADAR, 192),
};

static const struct reset_data s32g2_resets = {
	.resets = s32g2_reset_table,
	.n_resets = ARRAY_SIZE(s32g2_reset_table),
};

static const struct reset_data s32r45_resets = {
	.resets = s32r45_reset_table,
	.n_resets = ARRAY_SIZE(s32r45_reset_table),
};

static struct s32gen1_reset *
to_s32gen1_reset(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct s32gen1_reset, rcdev);
}

static int get_reset_regs(struct device *dev, unsigned long id,
			  uintptr_t *prst, uintptr_t *pstat)
{
	uint32_t rgm_set;

	/* MC_RGM valid reset IDs */
	switch (id) {
	case 0 ... 17:
		rgm_set = 0;
		break;
	case 64 ... 68:
		rgm_set = 1;
		break;
	case 128 ... 130:
		rgm_set = 2;
		break;
	case 192 ... 194:
		rgm_set = 3;
		break;
	default:
		dev_err(dev, "Wrong reset id: %lu\n", id);
		return -EINVAL;
	};

	*prst = RGM_PRST(rgm_set);
	*pstat = RGM_PSTAT(rgm_set);

	return 0;
}

static bool s32gen1_reset_or_timeout(void __iomem *pstat_reg, uint32_t mask,
				     bool asserted, ktime_t timeout)
{
	bool res;
	ktime_t cur = ktime_get();

	if (asserted)
		res = !!(readl(pstat_reg) & mask);
	else
		res = !(readl(pstat_reg) & mask);

	return res || ktime_after(cur, timeout);
}

static int get_entry_from_table(const struct reset_entry *table, size_t n_elem,
				unsigned long scmi_id,
				const struct reset_entry **entry)
{
	size_t i;

	for (i = 0; i < n_elem; i++) {
		if (scmi_id == table[i].scmi_id) {
			*entry = &table[i];
			return 0;
		}
	}

	return -EINVAL;
}

static int get_reset_entry(struct s32gen1_reset *reset, unsigned long scmi_id,
			   const struct reset_entry **entry)
{
	int ret;

	ret = get_entry_from_table(s32gen1_reset_table,
				   ARRAY_SIZE(s32gen1_reset_table), scmi_id,
				   entry);
	if (!ret)
		return 0;

	if (!reset->data)
		return -EINVAL;

	return get_entry_from_table(reset->data->resets, reset->data->n_resets,
				    scmi_id, entry);
}

static int s32gen1_assert_rgm(struct s32gen1_reset *rst, bool asserted,
			      uint32_t id)
{
	ktime_t timeout = ktime_add_ms(ktime_get(), S32GEN1_RESET_TIMEOUT_MS);
	struct device *dev = rst->rcdev.dev;
	uintptr_t prst, pstat;
	uint32_t id_offset, prst_val, stat_mask;
	const char *msg;
	int ret;

	id_offset = id % 32;
	stat_mask = PSTAT_PERIPH_n_STAT(id_offset);
	ret = get_reset_regs(dev, id, &prst, &pstat);
	if (ret)
		return ret;

	prst_val = readl(rst->rgm + prst);
	if (asserted) {
		msg = "assert";
		prst_val |= PRST_PERIPH_n_RST(id_offset);
	} else {
		msg = "deassert";
		prst_val &= ~PRST_PERIPH_n_RST(id_offset);
	}

	writel(prst_val, rst->rgm + prst);
	spin_until_cond(s32gen1_reset_or_timeout(rst->rgm + pstat, stat_mask,
						 asserted, timeout));
	if (asserted) {
		if (readl(rst->rgm + pstat) & stat_mask)
			return 0;
	} else {
		if (!(readl(rst->rgm + pstat) & stat_mask))
			return 0;
	}

	dev_err(dev, "Failed to %s reset for id %u\n", msg, id);
	return -EINVAL;
}

static void mc_me_wait_update(struct s32gen1_reset *rst, uint32_t partition_n,
			      uint32_t mask)
{
	unsigned int pupd;

	regmap_update_bits(rst->mc_me, MC_ME_PRTN_N_PUPD(partition_n),
			   mask, mask);

	regmap_write(rst->mc_me, MC_ME_CTL_KEY, MC_ME_CTL_KEY_KEY);
	regmap_write(rst->mc_me, MC_ME_CTL_KEY, MC_ME_CTL_KEY_INVERTEDKEY);

	do {
		regmap_read(rst->mc_me, MC_ME_PRTN_N_PUPD(partition_n),
			    &pupd);
	} while (pupd & mask);
}

static void set_rdc_lock(void __iomem *rdc, uint32_t partition_n, bool lock)
{
	uint32_t rdc_ctrl = readl_relaxed(RDC_RD_N_CTRL(rdc, partition_n));

	if (lock)
		rdc_ctrl &= ~RD_CTRL_UNLOCK_MASK;
	else
		rdc_ctrl |= RD_CTRL_UNLOCK_MASK;

	writel_relaxed(rdc_ctrl, RDC_RD_N_CTRL(rdc, partition_n));
}

static void unlock_rdc_write(void __iomem *rdc, uint32_t partition_n)
{
	set_rdc_lock(rdc, partition_n, false);
}

static void lock_rdc_write(void __iomem *rdc, uint32_t partition_n)
{
	set_rdc_lock(rdc, partition_n, true);
}

static void disable_partition(struct s32gen1_reset *rst, uint32_t partition_n)
{
	struct regmap *mc_me = rst->mc_me;
	void __iomem *rdc = rst->rdc;
	void __iomem *rgm = rst->rgm;

	uint32_t rdc_ctrl, prst;
	unsigned int part_status;

	regmap_read(mc_me, MC_ME_PRTN_N_STAT(partition_n),
		    &part_status);

	/* Skip if already disabled */
	if (!(MC_ME_PRTN_N_PCS & part_status))
		return;

	unlock_rdc_write(rdc, partition_n);

	/* Disable the XBAR interface */
	rdc_ctrl = readl_relaxed(RDC_RD_N_CTRL(rdc, partition_n));
	rdc_ctrl |= RDC_RD_INTERCONNECT_DISABLE;
	writel_relaxed(rdc_ctrl, RDC_RD_N_CTRL(rdc, partition_n));

	/* Wait until XBAR interface gets disabled */
	while (!(readl_relaxed(RDC_RD_N_STATUS(rdc, partition_n)) &
		RDC_RD_INTERCONNECT_DISABLE_STAT))
		;

	/* Disable partition clock */
	regmap_update_bits(mc_me, MC_ME_PRTN_N_PCONF(partition_n),
			   MC_ME_PRTN_N_PCE, 0);

	mc_me_wait_update(rst, partition_n, MC_ME_PRTN_N_PCUD);

	do {
		regmap_read(mc_me, MC_ME_PRTN_N_STAT(partition_n),
			    &part_status);
	} while (part_status & MC_ME_PRTN_N_PCS);

	/* Partition isolation */
	regmap_update_bits(mc_me, MC_ME_PRTN_N_PCONF(partition_n),
			   MC_ME_PRTN_N_OSSE, MC_ME_PRTN_N_OSSE);

	mc_me_wait_update(rst, partition_n, MC_ME_PRTN_N_OSSUD);

	do {
		regmap_read(mc_me, MC_ME_PRTN_N_STAT(partition_n),
			    &part_status);
	} while (!(part_status & MC_ME_PRTN_N_OSSS));

	/* Assert partition reset */
	prst = readl_relaxed(rgm + RGM_PRST(partition_n));
	writel_relaxed(prst | PRST_PERIPH_n_RST(0),
		       rgm + RGM_PRST(partition_n));

	while (!(readl_relaxed(rgm + RGM_PSTAT(partition_n)) &
			PSTAT_PERIPH_n_STAT(0)))
		;

	lock_rdc_write(rdc, partition_n);
}

static void enable_partition(struct s32gen1_reset *rst, uint32_t partition_n)
{
	struct regmap *mc_me = rst->mc_me;
	void __iomem *rdc = rst->rdc;
	void __iomem *rgm = rst->rgm;
	uint32_t prst, rdc_ctrl;
	unsigned int part_status;

	regmap_read(mc_me, MC_ME_PRTN_N_STAT(partition_n),
		    &part_status);

	/* Enable a partition only if it's disabled */
	if (MC_ME_PRTN_N_PCS & part_status)
		return;

	regmap_update_bits(mc_me, MC_ME_PRTN_N_PCONF(partition_n),
			   MC_ME_PRTN_N_PCE, MC_ME_PRTN_N_PCE);

	mc_me_wait_update(rst, partition_n, MC_ME_PRTN_N_PCUD);

	do {
		regmap_read(mc_me, MC_ME_PRTN_N_STAT(partition_n),
			    &part_status);
	} while (!(part_status & MC_ME_PRTN_N_OSSS));

	unlock_rdc_write(rdc, partition_n);

	/* Enable the XBAR interface */
	rdc_ctrl = readl_relaxed(RDC_RD_N_CTRL(rdc, partition_n));
	rdc_ctrl &= ~RDC_RD_INTERCONNECT_DISABLE;
	writel_relaxed(rdc_ctrl, RDC_RD_N_CTRL(rdc, partition_n));

	/* Wait until XBAR interface enabled */
	while ((readl_relaxed(RDC_RD_N_STATUS(rdc, partition_n)) &
		RDC_RD_INTERCONNECT_DISABLE_STAT))
		;

	/* Lift reset for partition */
	prst = readl_relaxed(rgm + RGM_PRST(partition_n));
	writel_relaxed(prst & (~PRST_PERIPH_n_RST(0)),
		       rgm + RGM_PRST(partition_n));

	/* Follow steps to clear OSSE bit */
	regmap_update_bits(mc_me, MC_ME_PRTN_N_PCONF(partition_n),
			   MC_ME_PRTN_N_OSSE, 0);

	mc_me_wait_update(rst, partition_n, MC_ME_PRTN_N_OSSUD);

	do {
		regmap_read(mc_me, MC_ME_PRTN_N_STAT(partition_n),
			    &part_status);
	} while (part_status & MC_ME_PRTN_N_OSSS);

	while (readl_relaxed(rgm + RGM_PSTAT(partition_n)) &
			PSTAT_PERIPH_n_STAT(0))
		;

	lock_rdc_write(rdc, partition_n);
}

static int s32gen1_assert_part(struct s32gen1_reset *rst, bool asserted,
			       uint32_t id)
{
	if (asserted)
		disable_partition(rst, id);
	else
		enable_partition(rst, id);

	return 0;
}

static int s32gen1_control_reset(struct reset_controller_dev *rcdev,
				 unsigned long scmi_id, bool asserted)
{
	struct s32gen1_reset *rst = to_s32gen1_reset(rcdev);
	const struct reset_entry *entry;
	int ret;

	ret = get_reset_entry(rst, scmi_id, &entry);
	if (ret)
		return ret;

	if (entry->part)
		ret = s32gen1_assert_part(rst, asserted, entry->rst_id);
	else
		ret = s32gen1_assert_rgm(rst, asserted, entry->rst_id);

	return ret;
}

static int s32gen1_assert_reset(struct reset_controller_dev *rcdev,
				unsigned long scmi_id)
{
	return s32gen1_control_reset(rcdev, scmi_id, true);
}

static int s32gen1_deassert_reset(struct reset_controller_dev *rcdev,
				  unsigned long scmi_id)
{
	return s32gen1_control_reset(rcdev, scmi_id, false);
}

static int s32gen1_reset(struct reset_controller_dev *rcdev,
			 unsigned long scmi_id)
{
	int ret;

	ret = s32gen1_assert_reset(rcdev, scmi_id);
	if (ret)
		return ret;

	return s32gen1_deassert_reset(rcdev, scmi_id);
}

static const struct reset_control_ops s32gen1_reset_ops = {
	.reset = s32gen1_reset,
	.assert = s32gen1_assert_reset,
	.deassert = s32gen1_deassert_reset,
};

static const struct of_device_id s32gen1_reset_match[];
static int s32gen1_reset_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct s32gen1_reset *reset;
	struct resource *res;
	struct device_node *np;

	reset = devm_kmalloc(dev, sizeof(*reset), GFP_KERNEL);
	if (!reset)
		return -ENOMEM;

	match = of_match_device(s32gen1_reset_match, &pdev->dev);
	if (match)
		reset->data = match->data;

	platform_set_drvdata(pdev, reset);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Missing reg region.\n");
		return -EIO;
	}

	reset->rgm = devm_ioremap(dev, res->start, resource_size(res));
	if (!reset->rgm) {
		dev_err(dev, "Failed to map s32gen1 reset registers\n");
		return -ENOMEM;
	}

	reset->mc_me =
		syscon_regmap_lookup_by_compatible("fsl,s32gen1-mc_me");
	if (IS_ERR(reset->mc_me)) {
		dev_err(&pdev->dev, "Cannot map 'MC_ME' resource\n");
		return PTR_ERR(reset->mc_me);
	}

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-rdc");
	reset->rdc = devm_of_iomap(dev, np, 0, NULL);
	if (IS_ERR(reset->rdc)) {
		dev_err(&pdev->dev, "Can not map 'RDC' resource\n");
		return PTR_ERR(reset->rdc);
	}

	reset->rcdev = (struct reset_controller_dev) {
		.owner = THIS_MODULE,
		.ops = &s32gen1_reset_ops,
		.of_node = pdev->dev.of_node,
		.nr_resets = S32GEN1_SCMI_RST_MAX_ID,
	};

	return devm_reset_controller_register(&pdev->dev, &reset->rcdev);
}

static int s32gen1_reset_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id s32gen1_reset_match[] = {
	{ .compatible = "fsl,s32g2-reset", .data = &s32g2_resets },
	{ .compatible = "fsl,s32r45-reset", .data = &s32r45_resets },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s32gen1_reset_match);

static struct platform_driver s32gen1_reset_driver = {
	.probe = s32gen1_reset_probe,
	.remove = s32gen1_reset_remove,
	.driver = {
		.name = "s32gen1_reset",
		.of_match_table = s32gen1_reset_match,
	},
};
module_platform_driver(s32gen1_reset_driver);

MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("S32GEN1 Reset driver");
MODULE_LICENSE("GPL");
