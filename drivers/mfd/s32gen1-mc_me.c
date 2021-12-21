// SPDX-License-Identifier: GPL-2.0
/**
 * NXP S32GEN1 MC_ME regmap driver
 * Copyright 2021 NXP
 */

#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/core.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mfd/s32gen1-mc_me.h>

static const struct regmap_range mc_me_ro_ranges[] = {
	regmap_reg_range(MC_ME_MODE_STAT, MC_ME_MODE_STAT),
	regmap_reg_range(MC_ME_PRTN_N_STAT(0), MC_ME_PRTN_N_COFB0_STAT(0)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_STAT(0, 0),
			MC_ME_PRTN_N_CORE_M_STAT(0, 0)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_STAT(0, 1),
			MC_ME_PRTN_N_CORE_M_STAT(0, 1)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_STAT(0, 2),
			MC_ME_PRTN_N_CORE_M_STAT(0, 2)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_STAT(0, 3),
			MC_ME_PRTN_N_CORE_M_STAT(0, 3)),
	regmap_reg_range(MC_ME_PRTN_N_STAT(1), MC_ME_PRTN_N_STAT(1)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_STAT(1, 0),
			MC_ME_PRTN_N_CORE_M_STAT(1, 0)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_STAT(1, 1),
			MC_ME_PRTN_N_CORE_M_STAT(1, 1)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_STAT(1, 2),
			MC_ME_PRTN_N_CORE_M_STAT(1, 2)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_STAT(1, 3),
			MC_ME_PRTN_N_CORE_M_STAT(1, 3)),
	regmap_reg_range(MC_ME_PRTN_N_STAT(2), MC_ME_PRTN_N_COFB0_STAT(2)),
	regmap_reg_range(MC_ME_PRTN_N_STAT(3), MC_ME_PRTN_N_COFB0_STAT(3)),
};

static const struct regmap_range mc_me_rd_ranges[] = {
	regmap_reg_range(MC_ME_CTL_KEY, MC_ME_MAIN_COREID),
	regmap_reg_range(MC_ME_PRTN_N_PCONF(0), MC_ME_PRTN_N_STAT(0)),
	regmap_reg_range(MC_ME_PRTN_N_COFB0_STAT(0),
			MC_ME_PRTN_N_COFB0_STAT(0)),
	regmap_reg_range(MC_ME_PRTN_N_COFB0_CLKEN(0),
			MC_ME_PRTN_N_COFB0_CLKEN(0)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_PCONF(0, 0),
			MC_ME_PRTN_N_CORE_M_ADDR(0, 0)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_PCONF(0, 1),
			MC_ME_PRTN_N_CORE_M_ADDR(0, 1)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_PCONF(0, 2),
			MC_ME_PRTN_N_CORE_M_ADDR(0, 2)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_PCONF(0, 3),
			MC_ME_PRTN_N_CORE_M_ADDR(0, 3)),
	regmap_reg_range(MC_ME_PRTN_N_PCONF(1), MC_ME_PRTN_N_STAT(1)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_PCONF(1, 0),
			MC_ME_PRTN_N_CORE_M_ADDR(1, 0)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_PCONF(1, 1),
			MC_ME_PRTN_N_CORE_M_ADDR(1, 1)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_PCONF(1, 2),
			MC_ME_PRTN_N_CORE_M_ADDR(1, 2)),
	regmap_reg_range(MC_ME_PRTN_N_CORE_M_PCONF(1, 3),
			MC_ME_PRTN_N_CORE_M_ADDR(1, 3)),
	regmap_reg_range(MC_ME_PRTN_N_PCONF(2), MC_ME_PRTN_N_STAT(2)),
	regmap_reg_range(MC_ME_PRTN_N_COFB0_STAT(2),
			MC_ME_PRTN_N_COFB0_STAT(2)),
	regmap_reg_range(MC_ME_PRTN_N_COFB0_CLKEN(2),
			MC_ME_PRTN_N_COFB0_CLKEN(2)),
	regmap_reg_range(MC_ME_PRTN_N_PCONF(3), MC_ME_PRTN_N_STAT(3)),
	regmap_reg_range(MC_ME_PRTN_N_COFB0_STAT(3),
			MC_ME_PRTN_N_COFB0_STAT(3)),
	regmap_reg_range(MC_ME_PRTN_N_COFB0_CLKEN(3),
			MC_ME_PRTN_N_COFB0_CLKEN(3)),
};

static bool mc_me_readable_reg(struct device *dev, unsigned int reg)
{
	return regmap_reg_in_ranges(reg, mc_me_rd_ranges,
				    ARRAY_SIZE(mc_me_rd_ranges));
}

static bool mc_me_writeable_reg(struct device *dev, unsigned int reg)
{
	if (regmap_reg_in_ranges(reg, mc_me_ro_ranges,
				    ARRAY_SIZE(mc_me_ro_ranges)))
		return false;

	return mc_me_readable_reg(dev, reg);
}

static const struct regmap_config mc_me_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.writeable_reg = mc_me_writeable_reg,
	.readable_reg = mc_me_readable_reg,
	.max_register = MC_ME_PRTN_N_COFB0_CLKEN(3),
};

static int s32gen1_mc_me_probe(struct platform_device *pdev)
{
	struct regmap *mc_me;
	struct device *dev = &pdev->dev;
	int err = 0;

	/*
	 * This will call of_syscon_register(), so a new MC_ME regmap
	 * will not be initialized in other syscon_regmap_lookup_by_*
	 * calls from other drivers.
	 */
	mc_me = syscon_node_to_regmap(dev->of_node);
	if (IS_ERR(mc_me)) {
		dev_err(&pdev->dev, "Cannot map 'MC_ME' resource\n");
		return -ENODEV;
	}

	err = regmap_attach_dev(dev, mc_me, &mc_me_regmap_config);
	if (err) {
		dev_err(dev, "Failed to attach device to 'MC_ME' regmap.\n");
		return err;
	}

	err = regmap_reinit_cache(mc_me, &mc_me_regmap_config);
	if (err) {
		dev_err(dev, "Failed to reinit 'MC_ME' regmap.\n");
		return err;
	}

	return 0;
}

static const struct of_device_id s32gen1_mc_me_match[] = {
	{ .compatible = "fsl,s32gen1-mc_me"},
	{},
};

static struct platform_driver s32gen1_mc_me_driver = {
	.driver = {
		.name = "s32gen1-mc_me",
		.of_match_table = s32gen1_mc_me_match
	},
	.probe = s32gen1_mc_me_probe,
};

static int __init mc_me_module_init(void)
{
	return platform_driver_register(&s32gen1_mc_me_driver);
}

static void __exit mc_me_module_cleanup(void)
{
	platform_driver_unregister(&s32gen1_mc_me_driver);
}

arch_initcall(mc_me_module_init);
module_exit(mc_me_module_cleanup);

MODULE_AUTHOR("Andra Ilie <andra.ilie@nxp.com>");
MODULE_DESCRIPTION("S32GEN1 MC_ME regmap driver");
MODULE_LICENSE("GPL v2");
