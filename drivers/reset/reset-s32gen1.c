// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright 2021 NXP
 */
#include <dt-bindings/reset/s32g-scmi-reset.h>
#include <dt-bindings/reset/s32gen1-scmi-reset.h>
#include <dt-bindings/reset/s32r45-scmi-reset.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/of_device.h>
#include <linux/processor.h>
#include <s32/s32-gen1/clk.h>
#include <s32/s32-gen1/rgm.h>

#define S32GEN1_RESET_TIMEOUT_MS	1

#define RESET_ENTRY(SCMI_ID, ID) \
{ .scmi_id = (SCMI_ID), .rst_id = (ID)}

struct reset_entry {
	unsigned long scmi_id;
	unsigned long rst_id;
};

struct reset_data {
	const struct reset_entry *resets;
	size_t n_resets;
};

struct s32gen1_reset {
	void __iomem			*rgm;
	struct reset_controller_dev	rcdev;
	const struct reset_data *data;
};

static const struct reset_entry s32gen1_reset_table[] = {
	RESET_ENTRY(S32GEN1_SCMI_RST_CM7_0, 0),
	RESET_ENTRY(S32GEN1_SCMI_RST_CM7_1, 1),
	RESET_ENTRY(S32GEN1_SCMI_RST_CM7_2, 2),
	RESET_ENTRY(S32GEN1_SCMI_RST_DDR, 3),
	RESET_ENTRY(S32GEN1_SCMI_RST_PCIE0, 4),
	RESET_ENTRY(S32GEN1_SCMI_RST_SERDES0, 5),
	RESET_ENTRY(S32GEN1_SCMI_RST_PCIE1, 16),
	RESET_ENTRY(S32GEN1_SCMI_RST_SERDES1, 17),
	RESET_ENTRY(S32GEN1_SCMI_RST_A53_0, 65),
	RESET_ENTRY(S32GEN1_SCMI_RST_A53_1, 66),
	RESET_ENTRY(S32GEN1_SCMI_RST_A53_2, 67),
	RESET_ENTRY(S32GEN1_SCMI_RST_A53_3, 68),
};

static const struct reset_entry s32g2_reset_table[] = {
	RESET_ENTRY(S32G_SCMI_RST_PFE, 128),
	RESET_ENTRY(S32G_SCMI_RST_LLCE, 192),
};

static const struct reset_entry s32r45_reset_table[] = {
	RESET_ENTRY(S32R45_SCMI_RST_LAX, 128),
	RESET_ENTRY(S32R45_SCMI_RST_RADAR, 192),
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

static int get_reset_regs(unsigned long id, struct s32gen1_reset *rst,
		      unsigned int *prst, unsigned int *pstat)
{
	struct device *dev = rst->rcdev.dev;
	u32 rgm_set;

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

static bool s32gen1_reset_or_timeout(void __iomem *pstat, u32 mask,
				     bool asserted, ktime_t timeout)
{
	bool res;
	ktime_t cur = ktime_get();

	if (asserted)
		res = !!(readl(pstat) & mask);
	else
		res = !(readl(pstat) & mask);

	return res || ktime_after(cur, timeout);
}

static int get_id_from_table(const struct reset_entry *table, size_t n_elem,
			     unsigned long scmi_id, unsigned long *id)
{
	size_t i;

	for (i = 0; i < n_elem; i++) {
		if (scmi_id == table[i].scmi_id) {
			*id = table[i].rst_id;
			return 0;
		}
	}

	return -EINVAL;
}

static int get_reset_id(struct s32gen1_reset *reset,
			unsigned long scmi_id, unsigned long *id)
{
	int ret;

	ret = get_id_from_table(s32gen1_reset_table,
				ARRAY_SIZE(s32gen1_reset_table), scmi_id, id);
	if (!ret)
		return 0;

	if (!reset->data)
		return -EINVAL;

	return get_id_from_table(reset->data->resets,
				 reset->data->n_resets, scmi_id, id);
}

static int s32gen1_assert_rgm(struct reset_controller_dev *rcdev,
			      bool asserted, unsigned long scmi_id)
{
	ktime_t timeout = ktime_add_ms(ktime_get(), S32GEN1_RESET_TIMEOUT_MS);
	struct s32gen1_reset *rst = to_s32gen1_reset(rcdev);
	struct device *dev = rst->rcdev.dev;
	unsigned int prst, pstat;
	u32 id_offset, prst_val, stat_mask;
	unsigned long id;
	const char *msg;
	int ret;

	ret = get_reset_id(rst, scmi_id, &id);
	if (ret)
		return ret;

	id_offset = id % 32;
	stat_mask = PSTAT_PERIPH_n_STAT(id_offset);
	ret = get_reset_regs(id, rst, &prst, &pstat);
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

	dev_err(dev, "Failed to %s reset for id %lu\n", msg, id);
	return -EINVAL;
}

static int s32gen1_assert_reset(struct reset_controller_dev *rcdev,
				unsigned long scmi_id)
{
	return s32gen1_assert_rgm(rcdev, true, scmi_id);
}

static int s32gen1_deassert_reset(struct reset_controller_dev *rcdev,
				  unsigned long scmi_id)
{
	return s32gen1_assert_rgm(rcdev, false, scmi_id);
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
