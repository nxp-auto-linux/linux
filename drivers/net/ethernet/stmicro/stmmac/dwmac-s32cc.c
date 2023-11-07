// SPDX-License-Identifier: GPL-2.0
/*
 * dwmac-s32cc.c - S32CC GMAC glue layer
 *
 * Copyright 2019-2020, 2022-2023 NXP
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
#include <soc/s32cc/nvmem_common.h>

/* S32CC Serdes */
#include <linux/pcs/pcs-xpcs.h>
#include <linux/pcs/nxp-s32cc-xpcs.h>

#include "stmmac_platform.h"

#define GMAC_TX_RATE_125M	125000000	/* 125MHz */
#define GMAC_TX_RATE_25M	25000000	/* 25MHz */
#define GMAC_TX_RATE_2M5	2500000		/* 2.5MHz */

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

#define S32CC_DUMMY_XPCS_ID		0x7996ced0
#define S32CC_DUMMY_XPCS_MASK		0xffffffff

struct s32cc_priv_data {
	void __iomem *ctrl_sts;
	struct device *dev;
	phy_interface_t intf_mode;
	struct clk *tx_clk;
	struct clk *rx_clk;
	bool enable_rx;
	bool use_nvmem;

	/* Serdes */
	int link_an;
	bool phyless_an;
	struct phy *serdes_phy;
	struct s32cc_xpcs *xpcs;
	const struct s32cc_xpcs_ops *xpcs_ops;
	struct dw_xpcs priv_dw_xpcs;
	struct phylink_pcs_ops pcs_xpcs_ops;
};

static const int xpcs_sgmii_features[] = {
	ETHTOOL_LINK_MODE_Pause_BIT,
	ETHTOOL_LINK_MODE_Asym_Pause_BIT,
	ETHTOOL_LINK_MODE_Autoneg_BIT,
	ETHTOOL_LINK_MODE_10baseT_Half_BIT,
	ETHTOOL_LINK_MODE_10baseT_Full_BIT,
	ETHTOOL_LINK_MODE_100baseT_Half_BIT,
	ETHTOOL_LINK_MODE_100baseT_Full_BIT,
	ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
	ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
	ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
	__ETHTOOL_LINK_MODE_MASK_NBITS,
};

static const phy_interface_t xpcs_sgmii_interfaces[] = {
	PHY_INTERFACE_MODE_SGMII,
};

static const struct xpcs_compat s32cc_gmac_xpcs_compat[DW_XPCS_INTERFACE_MAX] = {
	[DW_XPCS_SGMII] = {
		.supported = xpcs_sgmii_features,
		.interface = xpcs_sgmii_interfaces,
		.num_interfaces = ARRAY_SIZE(xpcs_sgmii_interfaces),
		.an_mode = DW_AN_C37_SGMII,
		.pma_config = NULL,
	},
};

static const struct xpcs_id s32cc_gmac_xpcs_id = {
	.id = S32CC_DUMMY_XPCS_ID,
	.mask = S32CC_DUMMY_XPCS_MASK,
	.compat = s32cc_gmac_xpcs_compat,
};

static int xpcs_config(struct phylink_pcs *pcs, unsigned int mode,
		       phy_interface_t interface,
		       const unsigned long *advertising,
		       bool permit_pause_to_mac)
{
	/* Advertising is the first member of struct phylink_link_state */
	struct phylink_link_state *state = (struct phylink_link_state *)advertising;
	struct phylink_link_state sgmii_state = { 0 };
	struct s32cc_priv_data *gmac = container_of(pcs->ops, struct s32cc_priv_data, pcs_xpcs_ops);

	if (gmac->intf_mode != PHY_INTERFACE_MODE_SGMII)
		return 0;

	if (!gmac->xpcs || !gmac->xpcs_ops)
		return -EINVAL;

	if (gmac->link_an == MLO_AN_FIXED || gmac->link_an == MLO_AN_PHY) {
		gmac->xpcs_ops->xpcs_get_state(gmac->xpcs, &sgmii_state);
		sgmii_state.speed = state->speed;
		sgmii_state.duplex = state->duplex;
		sgmii_state.an_enabled = false;
		gmac->xpcs_ops->xpcs_config(gmac->xpcs, &sgmii_state);
	} else if (gmac->link_an == MLO_AN_INBAND) {
		gmac->xpcs_ops->xpcs_config(gmac->xpcs, state);
	} else {
		return -EINVAL;
	}

	return 0;
}

static void xpcs_get_state(struct phylink_pcs *pcs,
			   struct phylink_link_state *state)
{
	struct s32cc_priv_data *gmac = container_of(pcs->ops, struct s32cc_priv_data, pcs_xpcs_ops);

	if (gmac->intf_mode != PHY_INTERFACE_MODE_SGMII)
		return;

	if (!gmac->xpcs || !gmac->xpcs_ops)
		return;

	gmac->xpcs_ops->xpcs_get_state(gmac->xpcs, state);

	/* Update SerDes state (Platform limitation) */
	if (gmac->phyless_an)
		gmac->xpcs_ops->xpcs_config(gmac->xpcs, state);
}

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
		ret = clk_set_rate(gmac->tx_clk, GMAC_TX_RATE_125M);
		if (!ret)
			ret = clk_prepare_enable(gmac->tx_clk);

		if (ret) {
			dev_err(&pdev->dev, "Can't set tx clock\n");
			return ret;
		}
	}

	if (gmac->rx_clk) {
		ret = clk_prepare_enable(gmac->rx_clk);
		if (ret) {
			dev_dbg(&pdev->dev, "Can't set rx, clock source is disabled.\n");
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

static void s32cc_fix_speed(void *priv, unsigned int speed)
{
	struct s32cc_priv_data *gmac = priv;
	int ret;

	if (!gmac->tx_clk || !gmac->rx_clk)
		return;

	if (gmac->enable_rx) {
		ret = clk_prepare_enable(gmac->rx_clk);
		if (ret) {
			dev_err(gmac->dev, "Can't set RX clock\n");
			return;
		}
		dev_info(gmac->dev, "Set RX clock\n");
		gmac->enable_rx = false;
	}

	/* SGMII mode doesn't support the clock reconfiguration */
	if (gmac->intf_mode == PHY_INTERFACE_MODE_SGMII)
		return;

	switch (speed) {
	case SPEED_1000:
		dev_info(gmac->dev, "Set TX clock to 125M\n");
		clk_set_rate(gmac->tx_clk, GMAC_TX_RATE_125M);
		break;
	case SPEED_100:
		dev_info(gmac->dev, "Set TX clock to 25M\n");
		clk_set_rate(gmac->tx_clk, GMAC_TX_RATE_25M);
		break;
	case SPEED_10:
		dev_info(gmac->dev, "Set TX clock to 2.5M\n");
		clk_set_rate(gmac->tx_clk, GMAC_TX_RATE_2M5);
		break;
	default:
		dev_err(gmac->dev, "Unsupported/Invalid speed: %d\n", speed);
		return;
	}
}

static int s32cc_config_cache_coherency(struct platform_device *pdev,
					struct plat_stmmacenet_data *plat_dat)
{
	plat_dat->axi4_ace_ctrl =
		devm_kzalloc(&pdev->dev,
			     sizeof(struct stmmac_axi4_ace_ctrl),
			     GFP_KERNEL);

	if (!plat_dat->axi4_ace_ctrl)
		return -ENOMEM;

	plat_dat->axi4_ace_ctrl->tx_ar_reg = (ACE_CONTROL_SIGNALS << 16)
		| (ACE_CONTROL_SIGNALS << 8) | ACE_CONTROL_SIGNALS;

	plat_dat->axi4_ace_ctrl->rx_aw_reg = (ACE_CONTROL_SIGNALS << 24)
		| (ACE_CONTROL_SIGNALS << 16) | (ACE_CONTROL_SIGNALS << 8)
		| ACE_CONTROL_SIGNALS;

	plat_dat->axi4_ace_ctrl->txrx_awar_reg = (ACE_PROTECTION << 20)
		| (ACE_PROTECTION << 16) | (ACE_CONTROL_SIGNALS << 8)
		| ACE_CONTROL_SIGNALS;

	return 0;
}

static void s32cc_init_plat_data(struct plat_stmmacenet_data *plat_dat)
{
	plat_dat->safety_feat_cfg->tsoee = 1;
	plat_dat->safety_feat_cfg->mrxpee = 1;
	plat_dat->safety_feat_cfg->mestee = 1;
	plat_dat->safety_feat_cfg->mrxee = 1;
	plat_dat->safety_feat_cfg->mtxee = 1;
	plat_dat->safety_feat_cfg->epsi = 1;
	plat_dat->safety_feat_cfg->edpp = 1;
	plat_dat->safety_feat_cfg->prtyen = 1;
	plat_dat->safety_feat_cfg->tmouten = 1;

	/* core feature set */
	plat_dat->has_gmac4 = true;
	plat_dat->pmt = 1;

	plat_dat->init = s32cc_gmac_init;
	plat_dat->exit = s32cc_gmac_exit;
	plat_dat->fix_mac_speed = s32cc_fix_speed;

	/* configure bitfield for quirks */
	plat_dat->quirk_mask_id = 0;
#if IS_ENABLED(CONFIG_DWMAC_S32CC_S32G274A_QUIRKS)
	plat_dat->quirk_mask_id |= QUIRK_MASK_S32G274A;
#endif
}

static int s32cc_configure_serdes(struct plat_stmmacenet_data *plat_dat,
				  struct phy *serdes_phy)
{
	struct s32cc_priv_data *gmac = plat_dat->bsp_priv;
	struct device_node *np = gmac->dev->of_node;
	const char *managed;

	gmac->serdes_phy = serdes_phy;

	if (!gmac->serdes_phy) {
		dev_err(gmac->dev, "SerDes PHY was not found\n");
		return -EINVAL;
	}

	if (phy_init(gmac->serdes_phy) || phy_power_on(gmac->serdes_phy)) {
		dev_err(gmac->dev, "SerDes PHY init failed\n");
		return -EINVAL;
	}

	if (phy_configure(gmac->serdes_phy, NULL)) {
		dev_err(gmac->dev, "SerDes PHY configuration failed\n");
		return -EINVAL;
	}

	gmac->xpcs = s32cc_phy2xpcs(gmac->serdes_phy);
	gmac->xpcs_ops = s32cc_xpcs_get_ops();

	/* We have to know interface type due to platform limitations */
	gmac->link_an = MLO_AN_PHY;
	if (of_phy_is_fixed_link(np))
		gmac->link_an = MLO_AN_FIXED;

	if (of_property_read_string(np, "managed", &managed) == 0 &&
	    strcmp(managed, "in-band-status") == 0) {
		gmac->link_an = MLO_AN_INBAND;
		dev_info(gmac->dev, "SGMII AN enabled\n");
	}

	gmac->phyless_an = false;
	if (gmac->link_an == MLO_AN_INBAND &&
	    !of_parse_phandle(np, "phy-handle", 0)) {
		dev_info(gmac->dev, "PHY less SGMII\n");
		gmac->phyless_an = true;
	}

	if (!gmac->xpcs || !gmac->xpcs_ops) {
		dev_err(gmac->dev, "Can't get SGMII PCS\n");
		gmac->xpcs_ops = NULL;
		gmac->xpcs = NULL;
	}

	return 0;
}

static int s32cc_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_priv *priv;
	struct stmmac_resources stmmac_res;
	struct s32cc_priv_data *gmac;
	struct resource *res;
	const char *tx_clk, *rx_clk;
	struct phy *serdes_phy = NULL;
	struct nvmem_cell *cell;
	bool use_nvmem;
	int ret;

	if (device_get_phy_mode(&pdev->dev) == PHY_INTERFACE_MODE_SGMII) {
		serdes_phy = devm_phy_get(&pdev->dev, "gmac_xpcs");

		if (IS_ERR(serdes_phy) &&
		    (PTR_ERR(serdes_phy) == -EPROBE_DEFER))
			return -EPROBE_DEFER;
		else if (IS_ERR(serdes_phy))
			serdes_phy = NULL;
	}

	/* Check if NVMEM for PHY Interface Mode selection is available */
	cell = nvmem_cell_get(&pdev->dev, "gmac_phy_intf_sel");
	if (IS_ERR(cell)) {
		if (PTR_ERR(cell) == -EPROBE_DEFER)
			return PTR_ERR(cell);

		dev_info(&pdev->dev, "No NVMEM for GPRs available, mapping them as resource\n");
		use_nvmem = false;
	} else {
		use_nvmem = true;
		nvmem_cell_put(cell);
	}

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	gmac = devm_kzalloc(&pdev->dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac) {
		return PTR_ERR(gmac);
	}

	gmac->dev = &pdev->dev;
	gmac->use_nvmem = use_nvmem;

	/* S32G control reg */
	if (!gmac->use_nvmem) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		gmac->ctrl_sts = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR_OR_NULL(gmac->ctrl_sts)) {
			dev_err(&pdev->dev, "S32CC config region is missing\n");
			return PTR_ERR(gmac->ctrl_sts);
		}
	}

	plat_dat = stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	plat_dat->bsp_priv = gmac;

	plat_dat->safety_feat_cfg = devm_kzalloc(&pdev->dev,
						 sizeof(*plat_dat->safety_feat_cfg),
						 GFP_KERNEL);
	if (!plat_dat->safety_feat_cfg)
		return PTR_ERR(plat_dat->safety_feat_cfg);

	if (plat_dat->phy_interface != PHY_INTERFACE_MODE_SGMII &&
	    !phy_interface_mode_is_rgmii(plat_dat->phy_interface)) {
		dev_err(&pdev->dev, "Not supported phy interface mode: [%s]\n",
			phy_modes(plat_dat->phy_interface));
		return -EINVAL;
	}

	switch (plat_dat->phy_interface) {
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
	case PHY_INTERFACE_MODE_MII:
		tx_clk = "tx_mii";
		rx_clk = "rx_mii";
		break;
	default:
		dev_err(&pdev->dev, "Not supported phy interface mode: [%s]\n",
			phy_modes(plat_dat->phy_interface));
		return -EINVAL;
	};

	gmac->intf_mode = plat_dat->phy_interface;

	plat_dat->bus_id = of_alias_get_id(pdev->dev.of_node, "gmac");
	if (plat_dat->bus_id < 0)
		plat_dat->bus_id = 0;

	/* DMA cache coherency settings. */
	if (of_dma_is_coherent(pdev->dev.of_node)) {
		ret = s32cc_config_cache_coherency(pdev, plat_dat);
		if (ret)
			goto err_remove_config_dt;
	}

	if (plat_dat->phy_interface == PHY_INTERFACE_MODE_SGMII) {
		ret = s32cc_configure_serdes(plat_dat, serdes_phy);
		if (ret) {
			dev_err(&pdev->dev, "SERDES is not configured\n");
			return ret;
		}
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
		dev_info(&pdev->dev, "rx clock not found\n");
		gmac->rx_clk = NULL;
	}

	ret = s32cc_gmac_init(pdev, gmac);
	if (ret)
		goto err_remove_config_dt;

	s32cc_init_plat_data(plat_dat);
	plat_dat->tso_en = of_property_read_bool(pdev->dev.of_node, "snps,tso");

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_gmac_exit;

	/* This is little hacked as we do not have
	 * the serdes accessible via MDIO
	 */
	if (gmac->xpcs) {
		gmac->pcs_xpcs_ops = (struct phylink_pcs_ops) {
			.pcs_config = xpcs_config,
			.pcs_get_state = xpcs_get_state,
		};

		priv = netdev_priv(dev_get_drvdata(&pdev->dev));
		priv->hw->xpcs = &gmac->priv_dw_xpcs;
		priv->hw->xpcs->pcs.ops = &gmac->pcs_xpcs_ops;
		priv->hw->xpcs->pcs.poll = true;
		priv->hw->xpcs->id = &s32cc_gmac_xpcs_id;
	}

	return 0;

err_gmac_exit:
	s32cc_gmac_exit(pdev, plat_dat->bsp_priv);
err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);
	return ret;
}

static int s32cc_dwmac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct plat_stmmacenet_data *plat = priv->plat;
	struct s32cc_priv_data *gmac = plat->bsp_priv;

	if (gmac->serdes_phy) {
		phy_exit(gmac->serdes_phy);
		gmac->serdes_phy = NULL;
		gmac->xpcs = NULL;
		gmac->xpcs_ops = NULL;
	}

	return stmmac_pltfr_remove(pdev);
}

static const struct of_device_id s32cc_dwmac_match[] = {
	{ .compatible = "nxp,s32cc-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, s32cc_dwmac_match);

static struct platform_driver s32cc_dwmac_driver = {
	.probe  = s32cc_dwmac_probe,
	.remove = s32cc_dwmac_remove,
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

