// SPDX-License-Identifier: GPL-2.0
/*
 * NXP S32G/R GMAC glue layer
 *
 * Copyright 2019-2025 NXP
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/of_address.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>

#include "stmmac_platform.h"

#define GMAC_INTF_RATE_125M	125000000	/* 125MHz */

/* SoC PHY interface control register */
#define PHY_INTF_SEL_MII	0x00
#define PHY_INTF_SEL_SGMII	0x01
#define PHY_INTF_SEL_RGMII	0x02
#define PHY_INTF_SEL_RMII	0x08

/* AXI4 ACE control settings */
#define ACE_DOMAIN_SIGNAL	0x2
#define ACE_CACHE_SIGNAL	0xf
#define ACE_CONTROL_SIGNALS	((ACE_DOMAIN_SIGNAL << 4) | ACE_CACHE_SIGNAL)
#define ACE_PROTECTION		0x2

struct s32_priv_data {
	void __iomem *ioaddr;
	void __iomem *ctrl_sts;
	struct device *dev;
	phy_interface_t *intf_mode;
	struct clk *tx_clk;
	struct clk *rx_clk;
};

static int s32_gmac_write_phy_intf_select(struct s32_priv_data *gmac)
{
	u32 intf_sel;

	switch (*gmac->intf_mode) {
	case PHY_INTERFACE_MODE_SGMII:
		intf_sel = PHY_INTF_SEL_SGMII;
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
		intf_sel = PHY_INTF_SEL_RGMII;
		break;
	case PHY_INTERFACE_MODE_RMII:
		intf_sel = PHY_INTF_SEL_RMII;
		break;
	case PHY_INTERFACE_MODE_MII:
		intf_sel = PHY_INTF_SEL_MII;
		break;
	default:
		dev_err(gmac->dev, "Unsupported PHY interface: %s\n",
			phy_modes(*gmac->intf_mode));
		return -EINVAL;
	}

	writel(intf_sel, gmac->ctrl_sts);

	dev_dbg(gmac->dev, "PHY mode set to %s\n", phy_modes(*gmac->intf_mode));

	return 0;
}

static int s32_gmac_init(struct platform_device *pdev, void *priv)
{
	struct s32_priv_data *gmac = priv;
	int ret;

	/* Set initial TX interface clock */
	ret = clk_prepare_enable(gmac->tx_clk);
	if (ret) {
		dev_err(&pdev->dev, "Can't enable tx clock\n");
		return ret;
	}
	ret = clk_set_rate(gmac->tx_clk, GMAC_INTF_RATE_125M);
	if (ret) {
		dev_err(&pdev->dev, "Can't set tx clock\n");
		goto err_tx_disable;
	}

	/* Set initial RX interface clock */
	ret = clk_prepare_enable(gmac->rx_clk);
	if (ret) {
		dev_err(&pdev->dev, "Can't enable rx clock\n");
		goto err_tx_disable;
	}
	ret = clk_set_rate(gmac->rx_clk, GMAC_INTF_RATE_125M);
	if (ret) {
		dev_err(&pdev->dev, "Can't set rx clock\n");
		goto err_txrx_disable;
	}

	/* Set interface mode */
	ret = s32_gmac_write_phy_intf_select(gmac);
	if (ret) {
		dev_err(&pdev->dev, "Can't set PHY interface mode\n");
		goto err_txrx_disable;
	}

	return 0;

err_txrx_disable:
	clk_disable_unprepare(gmac->rx_clk);
err_tx_disable:
	clk_disable_unprepare(gmac->tx_clk);
	return ret;
}

static void s32_gmac_exit(struct platform_device *pdev, void *priv)
{
	struct s32_priv_data *gmac = priv;

	clk_disable_unprepare(gmac->tx_clk);
	clk_disable_unprepare(gmac->rx_clk);
}

static void s32_fix_mac_speed(void *priv, unsigned int speed, unsigned int mode)
{
	struct s32_priv_data *gmac = priv;
	long tx_clk_rate;
	int ret;

	tx_clk_rate = rgmii_clock(speed);
	if (tx_clk_rate < 0) {
		dev_err(gmac->dev, "Unsupported/Invalid speed: %d\n", speed);
		return;
	}

	dev_dbg(gmac->dev, "Set tx clock to %ld Hz\n", tx_clk_rate);
	ret = clk_set_rate(gmac->tx_clk, tx_clk_rate);
	if (ret)
		dev_err(gmac->dev, "Can't set tx clock\n");
}

static void s32_dwmac_ptp_clk_freq_config(struct stmmac_priv *priv)
{
	struct plat_stmmacenet_data *plat = priv->plat;

	if (!plat->clk_ptp_ref)
		return;

	plat->clk_ptp_rate = clk_get_rate(plat->clk_ptp_ref);

	netdev_dbg(priv->dev, "PTP rate %lu\n", plat->clk_ptp_rate);
}

static int s32_config_cache_coherency(struct platform_device *pdev,
				      struct plat_stmmacenet_data *plat)
{
	if (!plat->axi) {
		plat->axi = kzalloc(sizeof(*plat->axi), GFP_KERNEL);

		if (!plat->axi)
			return -ENOMEM;
	}

	plat->axi->tx_ar_reg = (ACE_CONTROL_SIGNALS << 16)
			       | (ACE_CONTROL_SIGNALS << 8)
			       | ACE_CONTROL_SIGNALS;

	plat->axi->rx_aw_reg = (ACE_CONTROL_SIGNALS << 24)
			       | (ACE_CONTROL_SIGNALS << 16)
			       | (ACE_CONTROL_SIGNALS << 8)
			       | ACE_CONTROL_SIGNALS;

	plat->axi->txrx_awar_reg = (ACE_PROTECTION << 20)
				   | (ACE_PROTECTION << 16)
				   | (ACE_CONTROL_SIGNALS << 8)
				   | ACE_CONTROL_SIGNALS;

	return 0;
}

static int s32_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat;
	struct device *dev = &pdev->dev;
	struct stmmac_resources res;
	struct s32_priv_data *gmac;
	int ret;

	gmac = devm_kzalloc(&pdev->dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac)
		return -ENOMEM;

	gmac->dev = &pdev->dev;

	ret = stmmac_get_platform_resources(pdev, &res);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get platform resources\n");

	plat = devm_stmmac_probe_config_dt(pdev, res.mac);
	if (IS_ERR(plat))
		return dev_err_probe(dev, PTR_ERR(plat),
				     "dt configuration failed\n");

	if (plat->phy_interface != PHY_INTERFACE_MODE_SGMII &&
	    !phy_interface_mode_is_rgmii(plat->phy_interface) &&
	    plat->phy_interface != PHY_INTERFACE_MODE_RMII &&
	    plat->phy_interface != PHY_INTERFACE_MODE_MII)
		dev_err_probe(dev, -EINVAL,
			      "Not supported phy interface mode: [%s]\n",
			      phy_modes(plat->phy_interface));

	/* PHY interface mode control reg */
	gmac->ctrl_sts = devm_platform_get_and_ioremap_resource(pdev, 1, NULL);
	if (IS_ERR(gmac->ctrl_sts))
		return dev_err_probe(dev, PTR_ERR(gmac->ctrl_sts),
				     "S32CC config region is missing\n");

	/* tx clock */
	gmac->tx_clk = devm_clk_get(&pdev->dev, "tx");
	if (IS_ERR(gmac->tx_clk))
		return dev_err_probe(dev, PTR_ERR(gmac->tx_clk),
				     "tx clock not found\n");

	/* rx clock */
	gmac->rx_clk = devm_clk_get(&pdev->dev, "rx");
	if (IS_ERR(gmac->rx_clk))
		return dev_err_probe(dev, PTR_ERR(gmac->rx_clk),
				     "rx clock not found\n");

	gmac->intf_mode = &plat->phy_interface;
	gmac->ioaddr = res.addr;

	/* DMA cache coherency settings */
	if (of_dma_is_coherent(pdev->dev.of_node)) {
		ret = s32_config_cache_coherency(pdev, plat);
		if (ret)
			return dev_err_probe(dev, ret,
					     "System does not support DMA.\n");
	}

	/* S32CC core feature set */
	plat->has_gmac4 = true;
	plat->pmt = 1;
	plat->flags |= STMMAC_FLAG_SPH_DISABLE;
	plat->flags |= STMMAC_FLAG_HAS_S32CC;
	plat->flags |= STMMAC_FLAG_MULTI_MSI_EN;
	plat->rx_fifo_size = 20480;
	plat->tx_fifo_size = 20480;

	plat->init = s32_gmac_init;
	plat->exit = s32_gmac_exit;
	plat->fix_mac_speed = s32_fix_mac_speed;
	plat->ptp_clk_freq_config = s32_dwmac_ptp_clk_freq_config;

	plat->bsp_priv = gmac;

	return stmmac_pltfr_probe(pdev, plat, &res);
}

static const struct of_device_id s32_dwmac_match[] = {
	{ .compatible = "nxp,s32g2-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, s32_dwmac_match);

static struct platform_driver s32_dwmac_driver = {
	.probe = s32_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name = "s32-dwmac",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = s32_dwmac_match,
	},
};
module_platform_driver(s32_dwmac_driver);

MODULE_AUTHOR("Jan Petrous (OSS) <jan.petrous@oss.nxp.com>");
MODULE_DESCRIPTION("NXP S32G/R common chassis GMAC driver");
MODULE_LICENSE("GPL");

