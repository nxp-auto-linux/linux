// SPDX-License-Identifier: GPL-2.0
/*
 * dwmac-s32cc.c - S32CC GMAC glue layer
 *
 * Copyright 2019 NXP
 *
 */

#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/stmmac.h>

#include "stmmac_platform.h"

/* S32 SRC register for phyif selection */
#define PHY_INTF_SEL_SGMII      0x01
#define PHY_INTF_SEL_RGMII      0x02
#define PHY_INTF_SEL_RMII       0x08
#define PHY_INTF_SEL_MII        0x00
#define S32CC_GMAC_0_CTRL_STS		0x4007C004
#define EQOS_DMA_MODE_SWR		0x01
#define GMAC_DMA_MODE_CONFIG		0x1000

struct s32cc_priv_data {
	void __iomem *syscon;
	phy_interface_t intf_mode;
};

static int s32cc_gmac_init(struct platform_device *pdev, void *priv)
{
	struct s32cc_priv_data *gmac = priv;
	u32 intf_sel;

	gmac->syscon = ioremap(S32CC_GMAC_0_CTRL_STS, 4);
	if (!gmac->syscon) {
		dev_err(&pdev->dev, "cannot map GMAC_0_CTRL_STS\n");
		return -EIO;
	}

	switch (gmac->intf_mode) {
	case PHY_INTERFACE_MODE_SGMII:
		dev_info(&pdev->dev, "phy mode set to SGMII\n");
		intf_sel = PHY_INTF_SEL_SGMII;
		break;
	case PHY_INTERFACE_MODE_RGMII:
		dev_info(&pdev->dev, "phy mode set to RGMII\n");
		intf_sel = PHY_INTF_SEL_RGMII;
		break;
	case PHY_INTERFACE_MODE_RMII:
		dev_info(&pdev->dev, "phy mode set to RMII\n");
		intf_sel = PHY_INTF_SEL_RMII;
		break;
	case PHY_INTERFACE_MODE_MII:
		dev_info(&pdev->dev, "phy mode set to MII\n");
		intf_sel = PHY_INTF_SEL_MII;
		break;
	default:
		dev_info(&pdev->dev, "unsupported mode %d, set phy mode to RGMII\n",
			 gmac->intf_mode);
		intf_sel = PHY_INTF_SEL_RGMII;
		break;
	}

	/* set interface mode */
	writel(intf_sel, gmac->syscon);

	return 0;
}

static void s32cc_gmac_exit(struct platform_device *pdev, void *priv)
{
	struct s32cc_priv_data *gmac = priv;

	iounmap(gmac->syscon);
}

static int s32cc_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct s32cc_priv_data *gmac;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	gmac = devm_kzalloc(&pdev->dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac) {
		ret = PTR_ERR(gmac);
		goto err_exit;
	}

	/* phy mode */
	ret = of_get_phy_mode(pdev->dev.of_node, &gmac->intf_mode);
	if (ret) {
		dev_err(&pdev->dev, "missing phy-mode property\n");
		return ret;
	}

	if (gmac->intf_mode != PHY_INTERFACE_MODE_SGMII &&
	    gmac->intf_mode != PHY_INTERFACE_MODE_RGMII &&
	    gmac->intf_mode != PHY_INTERFACE_MODE_RMII &&
	    gmac->intf_mode != PHY_INTERFACE_MODE_MII) {
		dev_err(&pdev->dev, "Not supported phy interface mode: [%s]\n",
			phy_modes(gmac->intf_mode));
		return -EINVAL;
	}

	plat_dat = stmmac_probe_config_dt(pdev, stmmac_res.mac);
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

	plat_dat->init = s32cc_gmac_init;
	plat_dat->exit = s32cc_gmac_exit;
	plat_dat->bsp_priv = gmac;

	/* configure bitfield for quirks */
	plat_dat->quirk_mask_id = 0;
#if IS_ENABLED(CONFIG_DWMAC_S32CC_S32G274A_QUIRKS)
	plat_dat->quirk_mask_id |= QUIRK_MASK_S32G274A;
#endif

	ret = s32cc_gmac_init(pdev, plat_dat->bsp_priv);
	if (ret)
		goto err_remove_config_dt;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_gmac_exit;

	return 0;

err_gmac_exit:
	s32cc_gmac_exit(pdev, plat_dat->bsp_priv);
err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);
err_exit:
	return ret;
}

static const struct of_device_id s32cc_dwmac_match[] = {
	{ .compatible = "nxp,s32cc-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, s32cc_dwmac_match);

static struct platform_driver s32cc_dwmac_driver = {
	.probe  = s32cc_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name           = "s32cc-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = s32cc_dwmac_match,
	},
};
module_platform_driver(s32cc_dwmac_driver);

MODULE_AUTHOR("Jan Petrous <jan.petrous@nxp.com>");
MODULE_DESCRIPTION("NXP S32CC common chassis GMAC driver");
MODULE_LICENSE("GPL v2");

