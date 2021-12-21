// SPDX-License-Identifier: GPL-2.0
/**
 * NXP S32GEN1 MC_RGM regmap driver
 * Copyright 2021 NXP
 */

#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/core.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mfd/s32gen1-mc_rgm.h>

static const struct regmap_range mc_rgm_ro_ranges[] = {
	regmap_reg_range(RGM_PSTAT(0), RGM_PSTAT(0)),
	regmap_reg_range(RGM_PSTAT(1), RGM_PSTAT(1)),
	regmap_reg_range(RGM_PSTAT(2), RGM_PSTAT(2)),
	regmap_reg_range(RGM_PSTAT(3), RGM_PSTAT(3)),
	regmap_reg_range(RGM_PSTAT(4), RGM_PSTAT(4)),
	regmap_reg_range(RGM_PSTAT(5), RGM_PSTAT(5)),
	regmap_reg_range(RGM_PSTAT(6), RGM_PSTAT(6)),
	regmap_reg_range(RGM_PSTAT(7), RGM_PSTAT(7)),
};

static const struct regmap_range mc_rgm_rw_ranges[] = {
	regmap_reg_range(RGM_DES, RGM_DES),
	regmap_reg_range(RGM_FES, RGM_FERD),
	regmap_reg_range(RGM_FREC, RGM_RDSS),
	regmap_reg_range(RGM_PRST(0), RGM_PRST(0)),
	regmap_reg_range(RGM_PRST(1), RGM_PRST(1)),
	regmap_reg_range(RGM_PRST(2), RGM_PRST(2)),
	regmap_reg_range(RGM_PRST(3), RGM_PRST(3)),
	regmap_reg_range(RGM_PRST(4), RGM_PRST(4)),
	regmap_reg_range(RGM_PRST(5), RGM_PRST(5)),
	regmap_reg_range(RGM_PRST(6), RGM_PRST(6)),
	regmap_reg_range(RGM_PRST(7), RGM_PRST(7)),
};

static bool mc_rgm_writeable_reg(struct device *dev, unsigned int reg)
{
	return regmap_reg_in_ranges(reg, mc_rgm_rw_ranges,
				    ARRAY_SIZE(mc_rgm_rw_ranges));
}

static bool mc_rgm_readable_reg(struct device *dev, unsigned int reg)
{
	if (!mc_rgm_writeable_reg(dev, reg))
		return regmap_reg_in_ranges(reg, mc_rgm_ro_ranges,
						ARRAY_SIZE(mc_rgm_ro_ranges));

	return true;
}

static const struct regmap_config mc_rgm_regmap_config = {
	.writeable_reg = mc_rgm_writeable_reg,
	.readable_reg = mc_rgm_readable_reg,
	.max_register = RGM_PSTAT(7),
};

static int s32gen1_mc_rgm_probe(struct platform_device *pdev)
{
	struct regmap *mc_rgm;
	struct device *dev = &pdev->dev;
	int err = 0;

	/*
	 * This will call of_syscon_register(), so a new RGM regmap
	 * will not be initialized in other syscon_regmap_lookup_by_*
	 * calls from other drivers.
	 */
	mc_rgm = syscon_node_to_regmap(dev->of_node);
	if (IS_ERR(mc_rgm)) {
		dev_err(&pdev->dev, "Cannot map 'RGM' resource\n");
		return -ENODEV;
	}

	err = regmap_reinit_cache(mc_rgm, &mc_rgm_regmap_config);
	if (err) {
		dev_err(dev, "Failed to reinit 'RGM' regmap.\n");
		return err;
	}

	return 0;
}

static const struct of_device_id s32gen1_mc_rgm_match[] = {
	{ .compatible = "fsl,s32gen1-rgm"},
	{},
};

static struct platform_driver s32gen1_mc_rgm_driver = {
	.driver = {
		.name = "s32gen1-rgm",
		.of_match_table = s32gen1_mc_rgm_match
	},
	.probe = s32gen1_mc_rgm_probe,
};

module_platform_driver(s32gen1_mc_rgm_driver);

MODULE_AUTHOR("Andra Ilie <andra.ilie@nxp.com>");
MODULE_DESCRIPTION("S32GEN1 MC_RGM regmap driver");
MODULE_LICENSE("GPL v2");
