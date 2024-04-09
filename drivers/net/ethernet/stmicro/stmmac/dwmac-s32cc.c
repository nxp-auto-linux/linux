// SPDX-License-Identifier: GPL-2.0
/*
 * dwmac-s32cc.c - S32G/R GMAC glue layer
 *
 * Copyright 2019-2020, 2022-2024 NXP
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
#include <linux/nvmem-consumer.h>
#if defined(CONFIG_NVMEM_S32CC_GPR)
#include <soc/s32cc/nvmem_common.h>
#endif

#include "stmmac_platform.h"

#define GMAC_RATE_125M		125000000	/* 125MHz */
#define GMAC_RATE_25M		25000000	/* 25MHz */
#define GMAC_RATE_2M5		2500000		/* 2.5MHz */

/* S32CC SRC register for phyif selection */
#define PHY_INTF_SEL_MII        0x00
#define PHY_INTF_SEL_SGMII      0x01
#define PHY_INTF_SEL_RGMII      0x02
#define PHY_INTF_SEL_RMII       0x08

/* AXI4 ACE control settings */
#define ACE_DOMAIN_SIGNAL	0x2
#define ACE_CACHE_SIGNAL	0xf
#define ACE_CONTROL_SIGNALS	((ACE_DOMAIN_SIGNAL << 4) | ACE_CACHE_SIGNAL)
#define ACE_PROTECTION		0x2

struct s32cc_priv_data {
	void __iomem *ioaddr;
	void __iomem *ctrl_sts;
	struct device *dev;
	phy_interface_t intf_mode;
	struct clk *tx_clk;
	struct clk *rx_clk;
	bool enable_rx;
	bool use_nvmem;
};

static int s32cc_gmac_write_phy_intf_select(struct s32cc_priv_data *gmac)
{
	u32 intf_sel;
	int ret;

	switch (gmac->intf_mode) {
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
			phy_modes(gmac->intf_mode));
		return -EINVAL;
	}

	if (gmac->use_nvmem) {
		ret = write_nvmem_cell(gmac->dev, "gmac_phy_intf_sel", intf_sel);
		if (ret)
			return ret;
	} else {
		writel(intf_sel, gmac->ctrl_sts);
	}

	dev_dbg(gmac->dev, "PHY mode set to %s\n", phy_modes(gmac->intf_mode));

	return 0;
}

static int s32cc_gmac_init(struct platform_device *pdev, void *priv)
{
	struct s32cc_priv_data *gmac = priv;
	int ret;

	if (gmac->tx_clk) {
		ret = clk_set_rate(gmac->tx_clk, GMAC_RATE_125M);
		if (!ret)
			ret = clk_prepare_enable(gmac->tx_clk);

		if (ret) {
			dev_err(&pdev->dev, "Can't set tx clock\n");
			return ret;
		}
	}

	if (gmac->rx_clk) {
		ret = clk_set_rate(gmac->rx_clk, GMAC_RATE_125M);
		if (!ret)
			ret = clk_prepare_enable(gmac->rx_clk);

		if (ret) {
			dev_err(&pdev->dev, "Can't set rx, clock source is disabled.\n");
			gmac->enable_rx = true;
		}
	}

	ret = s32cc_gmac_write_phy_intf_select(gmac);
	if (ret) {
		dev_err(&pdev->dev, "Can't set PHY interface mode\n");
		return ret;
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

static void s32cc_fix_mac_speed(void *priv, unsigned int speed, unsigned int mode)
{
	struct s32cc_priv_data *gmac = priv;
	unsigned long rate;
	int ret;

	if (!gmac->tx_clk || !gmac->rx_clk)
		return;

	switch (speed) {
	case SPEED_1000:
		rate = GMAC_RATE_125M;
		break;
	case SPEED_100:
		rate = GMAC_RATE_25M;
		break;
	case SPEED_10:
		rate = GMAC_RATE_2M5;
		break;
	default:
		dev_err(gmac->dev, "Unsupported/Invalid speed: %d\n", speed);
		return;
	}

	if (gmac->enable_rx) {
		/* M7 SRM clock driver requires a clk_set_rate call for each
		 * clock that needs to be enabled by an agent.
		 * Thus, set the rate for GMAC's RX clock before enabling it.
		 */
		ret = clk_set_rate(gmac->rx_clk, rate);
		if (!ret)
			ret = clk_prepare_enable(gmac->rx_clk);

		if (ret) {
			dev_err(gmac->dev, "Can't set RX clock\n");
			return;
		}
		dev_dbg(gmac->dev, "Set RX clock\n");
		gmac->enable_rx = false;
	}

	dev_info(gmac->dev, "Setting TX clock to %lu Hz\n", rate);

	ret = clk_set_rate(gmac->tx_clk, rate);
	if (ret) {
		dev_err(gmac->dev, "Can't set TX clock to rate %lu\n", rate);
		return;
	}
}

static int s32cc_config_cache_coherency(struct platform_device *pdev,
					struct plat_stmmacenet_data *plat_dat)
{
	if (!plat_dat->axi) {
		plat_dat->axi = kzalloc(sizeof(struct stmmac_axi), GFP_KERNEL);

		if (!plat_dat->axi)
			return -ENOMEM;
	}

	plat_dat->axi->tx_ar_reg = (ACE_CONTROL_SIGNALS << 16)
		| (ACE_CONTROL_SIGNALS << 8) | ACE_CONTROL_SIGNALS;

	plat_dat->axi->rx_aw_reg = (ACE_CONTROL_SIGNALS << 24)
		| (ACE_CONTROL_SIGNALS << 16) | (ACE_CONTROL_SIGNALS << 8)
		| ACE_CONTROL_SIGNALS;

	plat_dat->axi->txrx_awar_reg = (ACE_PROTECTION << 20)
		| (ACE_PROTECTION << 16) | (ACE_CONTROL_SIGNALS << 8)
		| ACE_CONTROL_SIGNALS;

	return 0;
}

static void s32cc_gmac_ptp_clk_freq_config(struct stmmac_priv *priv)
{
	struct plat_stmmacenet_data *plat = priv->plat;

	if (!plat->clk_ptp_ref)
		return;

	plat->clk_ptp_rate = clk_get_rate(plat->clk_ptp_ref);

	netdev_dbg(priv->dev, "PTP rate %lu\n", plat->clk_ptp_rate);
}

static int s32cc_dwmac_probe(struct platform_device *pdev)
{
	struct s32cc_priv_data *gmac;
	struct plat_stmmacenet_data *plat;
	struct stmmac_resources res;
	struct device *dev = &pdev->dev;
	struct nvmem_cell *cell;
	bool use_nvmem;
	int ret;

	/* Check if NVMEM for PHY Interface Mode selection is available */
	cell = nvmem_cell_get(dev, "gmac_phy_intf_sel");
	if (IS_ERR(cell)) {
		if (PTR_ERR(cell) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_info(dev, "No NVMEM for GPRs available, mapping them as resource\n");
		use_nvmem = false;
	} else {
		use_nvmem = true;
		nvmem_cell_put(cell);
	}

	gmac = devm_kzalloc(&pdev->dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac)
		return PTR_ERR(gmac);

	gmac->dev = &pdev->dev;
	gmac->use_nvmem = use_nvmem;

	ret = stmmac_get_platform_resources(pdev, &res);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get platform resources\n");

	plat = devm_stmmac_probe_config_dt(pdev, res.mac);
	if (IS_ERR(plat))
		return dev_err_probe(dev, PTR_ERR(plat),
				     "dt configuration failed\n");

	/* SoC control reg */
	if (!use_nvmem) {
		gmac->ctrl_sts = devm_platform_get_and_ioremap_resource(pdev, 1,
									NULL);
		if (IS_ERR_OR_NULL(gmac->ctrl_sts))
			return dev_err_probe(dev, PTR_ERR(gmac->ctrl_sts),
					     "S32CC config region is missing\n");
	}

	gmac->intf_mode = plat->phy_interface;
	gmac->ioaddr = res.addr;

	/* DMA cache coherency settings */
	if (of_dma_is_coherent(pdev->dev.of_node)) {
		ret = s32cc_config_cache_coherency(pdev, plat);
		if (ret)
			return dev_err_probe(dev, ret,
					     "System does not support DMA.\n");
	}

	/* S32CC core feature set */
	plat->has_gmac4 = true;
	plat->pmt = 1;
	plat->flags |= STMMAC_FLAG_SPH_DISABLE;
	plat->rx_fifo_size = 20480;
	plat->tx_fifo_size = 20480;

	plat->init = s32cc_gmac_init;
	plat->exit = s32cc_gmac_exit;
	plat->fix_mac_speed = s32cc_fix_mac_speed;
	plat->ptp_clk_freq_config = s32cc_gmac_ptp_clk_freq_config;
	plat->flags |= STMMAC_FLAG_HAS_S32CC;

	/* tx clock */
	gmac->tx_clk = devm_clk_get(&pdev->dev, "tx");
	if (IS_ERR(gmac->tx_clk)) {
		dev_info(&pdev->dev, "tx clock not found\n");
		gmac->tx_clk = NULL;
	}

	/* rx clock */
	gmac->rx_clk = devm_clk_get(&pdev->dev, "rx");
	if (IS_ERR(gmac->rx_clk)) {
		dev_info(&pdev->dev, "rx clock not found\n");
		gmac->rx_clk = NULL;
	}

	plat->bsp_priv = gmac;

	return stmmac_pltfr_probe(pdev, plat, &res);
}

static const struct of_device_id s32cc_dwmac_match[] = {
	{ .compatible = "nxp,s32cc-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, s32cc_dwmac_match);

static struct platform_driver s32cc_dwmac_driver = {
	.probe		= s32cc_dwmac_probe,
	.remove_new	= stmmac_pltfr_remove,
	.driver		= {
			    .name		= "s32cc-dwmac",
			    .pm		= &stmmac_pltfr_pm_ops,
			    .of_match_table = s32cc_dwmac_match,
	},
};
module_platform_driver(s32cc_dwmac_driver);

MODULE_AUTHOR("Jan Petrous <jan.petrous@nxp.com>");
MODULE_DESCRIPTION("NXP S32G/R common chassis GMAC driver");
MODULE_LICENSE("GPL");

