// SPDX-License-Identifier: GPL-2.0
/*
 * dwmac-s32cc.c - S32CC GMAC glue layer
 *
 * Copyright 2019-2020 NXP
 *
 */

#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/stmmac.h>

#include "stmmac_platform.h"

#define GMAC_TX_RATE_125M	125000000	/* 125MHz */
#define GMAC_TX_RATE_25M	25000000	/* 25MHz */
#define GMAC_TX_RATE_2M5	2500000		/* 2.5MHz */

/* S32 SRC register for phyif selection */
#define PHY_INTF_SEL_SGMII      0x01
#define PHY_INTF_SEL_RGMII      0x02
#define PHY_INTF_SEL_RMII       0x08
#define PHY_INTF_SEL_MII        0x00

struct s32cc_priv_data {
	void __iomem *ctrl_sts;
	struct device *dev;
	phy_interface_t intf_mode;
	struct clk *tx_clk;
	struct clk *rx_clk;
};

static int s32cc_gmac_init(struct platform_device *pdev, void *priv)
{
	struct s32cc_priv_data *gmac = priv;
	u32 intf_sel;
	int ret;

	if (gmac->tx_clk) {
		ret = clk_prepare_enable(gmac->tx_clk);
		if (ret) {
			dev_err(&pdev->dev, "cannot set tx clock\n");
			return ret;
		}
	}

	if (gmac->rx_clk) {
		ret = clk_prepare_enable(gmac->rx_clk);
		if (ret) {
			dev_err(&pdev->dev, "cannot set rx clock\n");
			return ret;
		}
	}

	/* set interface mode */
	if (gmac->ctrl_sts) {
		switch (gmac->intf_mode) {
		default:
			dev_info(&pdev->dev, "unsupported mode %d, set the default phy mode.\n",
				 gmac->intf_mode);
			fallthrough;
		case PHY_INTERFACE_MODE_SGMII:
			dev_info(&pdev->dev, "phy mode set to SGMII\n");
			intf_sel = PHY_INTF_SEL_SGMII;
			break;
		case PHY_INTERFACE_MODE_RGMII:
		case PHY_INTERFACE_MODE_RGMII_ID:
		case PHY_INTERFACE_MODE_RGMII_TXID:
		case PHY_INTERFACE_MODE_RGMII_RXID:
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
		}

		writel(intf_sel, gmac->ctrl_sts);
	}

	return 0;
}

static void s32cc_gmac_exit(struct platform_device *pdev, void *priv)
{
	struct s32cc_priv_data *gmac = priv;

	if (gmac->tx_clk)
		clk_disable_unprepare(gmac->tx_clk);

	if (gmac->rx_clk)
		clk_disable_unprepare(gmac->rx_clk);
}

static void s32cc_fix_speed(void *priv, unsigned int speed)
{
	struct s32cc_priv_data *gmac = priv;

	if (!gmac->tx_clk || !gmac->rx_clk)
		return;

	/* SGMII mode doesn't support the clock reconfiguration */
	if (gmac->intf_mode == PHY_INTERFACE_MODE_SGMII)
		return;

	switch (speed) {
	case SPEED_1000:
		dev_info(gmac->dev, "Set RX/TX clock to 125M\n");
		clk_set_rate(gmac->tx_clk, GMAC_TX_RATE_125M);
		clk_set_rate(gmac->rx_clk, GMAC_TX_RATE_125M);
		break;
	case SPEED_100:
		dev_info(gmac->dev, "Set RX/TX clock to 25M\n");
		clk_set_rate(gmac->tx_clk, GMAC_TX_RATE_25M);
		clk_set_rate(gmac->rx_clk, GMAC_TX_RATE_25M);
		break;
	case SPEED_10:
		dev_info(gmac->dev, "Set RX/TX clock to 2.5M\n");
		clk_set_rate(gmac->tx_clk, GMAC_TX_RATE_2M5);
		clk_set_rate(gmac->rx_clk, GMAC_TX_RATE_2M5);
		break;
	default:
		dev_err(gmac->dev, "Unsupported/Invalid speed: %d\n", speed);
		return;
	}
}

static int s32cc_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct s32cc_priv_data *gmac;
	struct resource *res;
	const char *tx_clk, *rx_clk;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	gmac = devm_kzalloc(&pdev->dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac) {
		return PTR_ERR(gmac);
	}
	gmac->dev = &pdev->dev;

	/* S32G control reg */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	gmac->ctrl_sts = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(gmac->ctrl_sts)) {
		dev_err(&pdev->dev, "S32CC config region is missing\n");
		return PTR_ERR(gmac->ctrl_sts);
	}

	/* phy mode */
	ret = of_get_phy_mode(pdev->dev.of_node, &gmac->intf_mode);
	if (ret) {
		dev_err(&pdev->dev, "missing phy-mode property\n");
		return ret;
	}

	if (gmac->intf_mode != PHY_INTERFACE_MODE_SGMII &&
	    !phy_interface_mode_is_rgmii(gmac->intf_mode)) {
		dev_err(&pdev->dev, "Not supported phy interface mode: [%s]\n",
			phy_modes(gmac->intf_mode));
		return -EINVAL;
	}

	switch (gmac->intf_mode) {
	case PHY_INTERFACE_MODE_SGMII:
		tx_clk = "tx_sgmii";
		rx_clk = "rx_sgmii";
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
		tx_clk = "tx_rgmii";
		rx_clk = "rx_rgmii";
		break;
	case PHY_INTERFACE_MODE_RMII:
		tx_clk = "tx_rmii";
		rx_clk = "rx_rmii";
		break;
	default:
	case PHY_INTERFACE_MODE_MII:
		tx_clk = "tx_mii";
		rx_clk = "rx_mii";
		break;
	};

	plat_dat = stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	plat_dat->bus_id = of_alias_get_id(pdev->dev.of_node, "gmac");
	if (plat_dat->bus_id < 0)
		plat_dat->bus_id = 0;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "System does not support DMA, aborting\n");
		goto err_remove_config_dt;
	}

	/* tx clock */
	gmac->tx_clk = devm_clk_get(&pdev->dev, tx_clk);
	if (IS_ERR(gmac->tx_clk)) {
		dev_info(&pdev->dev, "tx clock not found\n");
		gmac->tx_clk = NULL;
	}

	/* rx clock */
	gmac->rx_clk = devm_clk_get(&pdev->dev, rx_clk);
	if (IS_ERR(gmac->rx_clk)) {
		dev_info(&pdev->dev, "tx clock not found\n");
		gmac->rx_clk = NULL;
	}

	ret = s32cc_gmac_init(pdev, gmac);
	if (ret)
		goto err_remove_config_dt;

	plat_dat->bsp_priv = gmac;

	/* core feature set */
	plat_dat->has_gmac4 = true;
	plat_dat->pmt = 1;
	plat_dat->tso_en = of_property_read_bool(pdev->dev.of_node, "snps,tso");

	plat_dat->init = s32cc_gmac_init;
	plat_dat->exit = s32cc_gmac_exit;
	plat_dat->bsp_priv = gmac;
	plat_dat->fix_mac_speed = s32cc_fix_speed;

	/* configure bitfield for quirks */
	plat_dat->quirk_mask_id = 0;
#if IS_ENABLED(CONFIG_DWMAC_S32CC_S32G274A_QUIRKS)
	plat_dat->quirk_mask_id |= QUIRK_MASK_S32G274A;
#endif

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_gmac_exit;

	return 0;

err_gmac_exit:
	s32cc_gmac_exit(pdev, plat_dat->bsp_priv);
err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);
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

