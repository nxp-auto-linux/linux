// SPDX-License-Identifier: GPL-2.0
/*
 * dwmac-s32cc.c - S32x GMAC glue layer
 *
 * Copyright 2019 NXP
 *
 */

#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/stmmac.h>

#include "stmmac_platform.h"

static int s32cc_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32)) != 0) {
		dev_err(&pdev->dev, "System does not support DMA, aborting\n");
		return -EINVAL;
	}

	/* core feature set */
	plat_dat->has_gmac4 = true;
	plat_dat->pmt = 1;
	plat_dat->tso_en = of_property_read_bool(pdev->dev.of_node, "snps,tso");

	/* configure bitfield for quirks */
	plat_dat->quirk_mask_id = QUIRK_MASK_ERRATA_E50082;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_remove_config_dt;

	return 0;

err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static const struct of_device_id s32_dwmac_match[] = {
	{ .compatible = "fsl,s32cc-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, s32_dwmac_match);

static struct platform_driver s32_dwmac_driver = {
	.probe  = s32cc_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name           = "s32cc-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = s32_dwmac_match,
	},
};
module_platform_driver(s32_dwmac_driver);

MODULE_AUTHOR("Jan Petrous <jan.petrous@nxp.com>");
MODULE_DESCRIPTION("NXP S32 common chassis GMAC driver");
MODULE_LICENSE("GPL v2");

