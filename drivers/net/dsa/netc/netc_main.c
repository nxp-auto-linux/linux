// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include <linux/clk.h>
#include <linux/etherdevice.h>
#include <linux/fsl/enetc_mdio.h>
#include <linux/of_mdio.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/unaligned.h>

#include "netc_switch.h"

static enum dsa_tag_protocol netc_get_tag_protocol(struct dsa_switch *ds, int port,
						   enum dsa_tag_protocol mprot)
{
	struct netc_switch *priv = ds->priv;

	return priv->tag_proto;
}

static void netc_mac_port_wr(struct netc_port *port, u32 reg, u32 val)
{
	if (is_netc_pseudo_port(port))
		return;

	netc_port_wr(port, reg, val);
	if (port->caps.pmac)
		netc_port_wr(port, reg + NETC_PMAC_OFFSET, val);
}

static u32 netc_mac_port_rd(struct netc_port *port, u32 reg)
{
	if (is_netc_pseudo_port(port))
		return 0;

	return netc_port_rd(port, reg);
}

static void netc_port_get_capability(struct netc_port *port)
{
	u32 val;

	val = netc_port_rd(port, NETC_PMCAPR);
	if (val & PMCAPR_HD)
		port->caps.half_duplex = true;

	if (FIELD_GET(PMCAPR_FP, val) == FP_SUPPORT)
		port->caps.pmac = true;

	val = netc_port_rd(port, NETC_PCAPR);
	if (val & PCAPR_LINK_TYPE)
		port->caps.pseudo_link = true;
}

static int netc_port_get_index_from_dt(struct device_node *node,
				       struct device *dev, u32 *index)
{
	/* Get switch port number from DT */
	if (of_property_read_u32(node, "reg", index) < 0) {
		dev_err(dev, "The reg property isn't defined in DT node\n");
		of_node_put(node);
		return -ENODEV;
	}

	return 0;
}

static int netc_port_get_info_from_dt(struct netc_port *port,
				      struct device_node *node,
				      struct device *dev)
{
	phy_interface_t phy_mode;
	int err;

	/* Get PHY mode from DT */
	err = of_get_phy_mode(node, &phy_mode);
	if (err) {
		dev_err(dev, "Failed to get phy mode for port %d\n",
			port->index);
		of_node_put(node);
		return err;
	}

	if (of_find_property(node, "clock-names", NULL)) {
		port->ref_clk = devm_get_clk_from_child(dev, node, "ref");
		if (IS_ERR(port->ref_clk)) {
			dev_err(dev, "Port %d cannot get reference clock\n", port->index);
			return PTR_ERR(port->ref_clk);
		}
	}

	port->phy_mode = phy_mode;

	return 0;
}

static bool netc_port_has_pcs(phy_interface_t phy_mode)
{
	return (phy_mode == PHY_INTERFACE_MODE_SGMII ||
		phy_mode == PHY_INTERFACE_MODE_1000BASEX ||
		phy_mode == PHY_INTERFACE_MODE_2500BASEX);
}

static int netc_port_create_internal_mdiobus(struct netc_port *port)
{
	struct netc_switch *priv = port->switch_priv;
	struct enetc_mdio_priv *mdio_priv;
	struct device *dev = priv->dev;
	void __iomem *port_iobase;
	struct dw_xpcs *xpcs;
	struct enetc_hw *hw;
	struct mii_bus *bus;
	int err;

	port_iobase = port->iobase;
	hw = enetc_hw_alloc(dev, port_iobase);
	if (IS_ERR(hw)) {
		dev_err(dev, "Failed to allocate ENETC HW structure\n");
		return PTR_ERR(hw);
	}

	bus = mdiobus_alloc_size(sizeof(*mdio_priv));
	if (!bus)
		return -ENOMEM;

	bus->name = "NXP NETC Switch internal MDIO Bus";
	bus->read = enetc_mdio_read_c22;
	bus->write = enetc_mdio_write_c22;
	bus->read_c45 = enetc_mdio_read_c45;
	bus->write_c45 = enetc_mdio_write_c45;
	bus->parent = dev;
	bus->phy_mask = ~0;
	mdio_priv = bus->priv;
	mdio_priv->hw = hw;
	mdio_priv->mdio_base = NETC_IMDIO_BASE;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-p%d-imdio",
		 dev_name(dev), port->index);

	err = mdiobus_register(bus);
	if (err) {
		dev_err(dev, "Failed to register internal MDIO bus (%d)\n",
			err);
		goto free_mdiobus;
	}

	switch (priv->revision) {
	case NETC_SWITCH_REV_4_3:
		break;
	default:
		err = -EOPNOTSUPP;
		dev_err(dev, "unsupported xpcs version\n");
		goto unregister_mdiobus;
	}

	netc_xpcs_port_init(port->index);
	/* TODO: xpcs_create_mdiodev_with_phy() should be refactored for i.MX94 */
	xpcs = xpcs_create_mdiodev(bus, port->index, port->phy_mode);
	if (IS_ERR(xpcs)) {
		err = PTR_ERR(xpcs);
		dev_err(dev, "cannot create xpcs mdiodev (%d)\n", err);
		goto unregister_mdiobus;
	}

	port->imdio = bus;
	port->pcs = &xpcs->pcs;

	return 0;

unregister_mdiobus:
	mdiobus_unregister(bus);
free_mdiobus:
	mdiobus_free(bus);

	return err;
}

#define phylink_pcs_to_xpcs(pl_pcs) \
	container_of((pl_pcs), struct dw_xpcs, pcs)

static void netc_port_remove_internal_mdiobus(struct netc_port *port)
{
	struct mii_bus *bus = port->imdio;

	if (port->pcs) {
		struct dw_xpcs *xpcs = phylink_pcs_to_xpcs(port->pcs);

		xpcs_destroy(xpcs);
	}

	if (bus) {
		mdiobus_unregister(bus);
		mdiobus_free(bus);
	}
}

static void netc_remove_all_ports_internal_mdiobus(struct dsa_switch *ds)
{
	struct netc_switch *priv = ds->priv;
	int i;

	for (i = 0; i < ds->num_ports; i++) {
		struct netc_port *port = priv->ports[i];

		if (!is_netc_pseudo_port(port) &&
		    netc_port_has_pcs(port->phy_mode))
			netc_port_remove_internal_mdiobus(port);
	}
}

static int netc_init_all_ports(struct dsa_switch *ds)
{
	struct device_node *switch_node, *ports;
	struct netc_switch *priv = ds->priv;
	struct device *dev = priv->dev;
	struct netc_port *port;
	struct dsa_port *dp;
	int i, err;

	priv->ports = devm_kcalloc(dev, ds->num_ports,
				   sizeof(struct netc_port *),
				   GFP_KERNEL);
	if (!priv->ports)
		return -ENOMEM;

	for (i = 0; i < ds->num_ports; i++) {
		port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
		if (!port)
			return -ENOMEM;

		port->index = i;
		port->switch_priv = priv;
		port->iobase = priv->regs.port + PORT_IOBASE(i);
		priv->ports[i] = port;

		netc_port_get_capability(port);
	}

	switch_node = dev->of_node;
	ports = of_get_child_by_name(switch_node, "ports");
	if (!ports)
		ports = of_get_child_by_name(switch_node, "ethernet-ports");
	if (!ports) {
		dev_err(dev, "No ports or ethernet-ports child node in switch node\n");
		return -ENODEV;
	}

	for_each_available_child_of_node_scoped(ports, child) {
		u32 index;

		err = netc_port_get_index_from_dt(child, dev, &index);
		if (err < 0)
			goto remove_internal_mdiobus;

		port = priv->ports[index];
		err = netc_port_get_info_from_dt(port, child, dev);
		if (err)
			goto remove_internal_mdiobus;

		dp = dsa_to_port(ds, index);
		if (!dp) {
			err = -ENODEV;
			goto remove_internal_mdiobus;
		}

		port->dp = dp;
		if (!is_netc_pseudo_port(port) &&
		    netc_port_has_pcs(port->phy_mode)) {
			err = netc_port_create_internal_mdiobus(port);
			if (err)
				goto remove_internal_mdiobus;
		}
	}

	of_node_put(ports);

	return 0;

remove_internal_mdiobus:
	of_node_put(ports);
	netc_remove_all_ports_internal_mdiobus(ds);

	return err;
}

static void netc_init_ntmp_tbl_versions(struct netc_switch *priv)
{
	struct ntmp_user *user = &priv->user;

	/* All tables default to version 0 */
	memset(&user->tbl, 0, sizeof(user->tbl));

	if (priv->revision == NETC_SWITCH_REV_4_3)
		user->tbl.ist_ver = 1;
}

static int netc_init_all_cbdrs(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	struct ntmp_user *user = &priv->user;
	int i, j, err;

	user->cbdr_num = NETC_CBDR_NUM;
	user->dev = priv->dev;
	user->ring = kcalloc(user->cbdr_num, sizeof(struct netc_cbdr),
			     GFP_KERNEL);
	if (!user->ring)
		return -ENOMEM;

	/* Set the system attributes of reads and writes of command
	 * descriptor and data.
	 */
	netc_base_wr(regs, NETC_CCAR, NETC_DEFAULT_CMD_CACHE_ATTR);

	for (i = 0; i < user->cbdr_num; i++) {
		struct netc_cbdr *cbdr = &user->ring[i];
		struct netc_cbdr_regs cbdr_regs;

		cbdr_regs.pir = regs->base + NETC_CBDRPIR(i);
		cbdr_regs.cir = regs->base + NETC_CBDRCIR(i);
		cbdr_regs.mr = regs->base + NETC_CBDRMR(i);
		cbdr_regs.bar0 = regs->base + NETC_CBDRBAR0(i);
		cbdr_regs.bar1 = regs->base + NETC_CBDRBAR1(i);
		cbdr_regs.lenr = regs->base + NETC_CBDRLENR(i);

		err = ntmp_init_cbdr(cbdr, user->dev, &cbdr_regs);
		if (err) {
			for (j = 0; j < i; j++)
				ntmp_free_cbdr(&user->ring[j]);

			kfree(user->ring);
			user->dev = NULL;

			return err;
		}
	}

	return 0;
}

static void netc_remove_all_cbdrs(struct netc_switch *priv)
{
	struct ntmp_user *user = &priv->user;
	int i;

	for (i = 0; i < NETC_CBDR_NUM; i++)
		ntmp_free_cbdr(&user->ring[i]);

	kfree(user->ring);
	user->dev = NULL;
}

static int netc_init_ntmp_user(struct netc_switch *priv)
{
	struct ntmp_user *user = &priv->user;
	int err;

	user->dev_type = NETC_DEV_SWITCH;
	netc_init_ntmp_tbl_versions(priv);

	err = netc_init_all_cbdrs(priv);
	if (err)
		return err;

	return 0;
}

static void netc_deinit_ntmp_user(struct netc_switch *priv)
{
	netc_remove_all_cbdrs(priv);
}

static void netc_switch_dos_default_config(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	u32 val;

	val = DOSL2CR_SAMEADDR | DOSL2CR_MSAMCC;
	netc_base_wr(regs, NETC_DOSL2CR, val);

	val = DOSL3CR_SAMEADDR | DOSL3CR_IPSAMCC;
	netc_base_wr(regs, NETC_DOSL3CR, val);
}

static void netc_switch_vfht_default_config(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	u32 val;

	val = netc_base_rd(regs, NETC_VFHTDECR2);

	/* if no match is found in the VLAN Filter table, then VFHTDECR2[MLO]
	 * will take effect. VFHTDECR2[MLO] is set to "Software MAC learning
	 * secure" by default. Notice BPCR[MLO] will override VFHTDECR2[MLO]
	 * if its value is not zero.
	 */
	val = u32_replace_bits(val, MLO_SW_SEC, VFHTDECR2_MLO);
	val = u32_replace_bits(val, MFO_NO_MATCH_DISCARD, VFHTDECR2_MFO);
	netc_base_wr(regs, NETC_VFHTDECR2, val);
}

static void netc_switch_isit_key_config(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	u32 val;

	/* Key construction rule 0: PORT + SMAC + VID */
	val = ISIDKCCR0_VALID | ISIDKCCR0_PORTP | ISIDKCCR0_SMACP |
	      ISIDKCCR0_OVIDP;
	netc_base_wr(regs, NETC_ISIDKCCR0(0), val);

	/* Key construction rule 1: PORT + DMAC + VID */
	val = ISIDKCCR0_VALID | ISIDKCCR0_PORTP | ISIDKCCR0_DMACP |
	      ISIDKCCR0_OVIDP;
	netc_base_wr(regs, NETC_ISIDKCCR0(1), val);
}

static void netc_port_set_max_frame_size(struct netc_port *port,
					 u32 max_frame_size)
{
	u32 val;

	val = PM_MAXFRAM & max_frame_size;
	netc_mac_port_wr(port, NETC_PM_MAXFRM(0), val);
}

static void netc_switch_fixed_config(struct netc_switch *priv)
{
	netc_switch_dos_default_config(priv);
	netc_switch_vfht_default_config(priv);
	netc_switch_isit_key_config(priv);
}

static void netc_port_set_tc_max_sdu(struct netc_port *port,
				     int tc, u32 max_sdu)
{
	u32 val = max_sdu & PTCTMSDUR_MAXSDU;

	val = u32_replace_bits(val, SDU_TYPE_MPDU, PTCTMSDUR_SDU_TYPE);
	netc_port_wr(port, NETC_PTCTMSDUR(tc), val);
}

static void netc_port_set_all_tc_msdu(struct netc_port *port, u32 *max_sdu)
{
	u32 overhead = ETH_FCS_LEN + VLAN_ETH_HLEN;
	int tc;

	if (dsa_port_is_cpu(port->dp))
		overhead += NETC_TAG_MAX_LEN;

	for (tc = 0; tc < NETC_TC_NUM; tc++) {
		u32 msdu = NETC_MAX_FRAME_LEN;

		if (max_sdu && max_sdu[tc])
			msdu = max_sdu[tc] + overhead;

		if (msdu > NETC_MAX_FRAME_LEN)
			msdu = NETC_MAX_FRAME_LEN;

		netc_port_set_tc_max_sdu(port, tc, msdu);
	}
}

static void netc_port_set_mlo(struct netc_port *port, int mlo)
{
	u32 val, old_val;

	old_val = netc_port_rd(port, NETC_BPCR);
	val = u32_replace_bits(old_val, mlo, BPCR_MLO);
	if (old_val != val)
		netc_port_wr(port, NETC_BPCR, val);
}

static void netc_port_fixed_config(struct netc_port *port)
{
	u32 val;

	/* Default IPV and DR setting */
	val = netc_port_rd(port, NETC_PQOSMR);
	val |= PQOSMR_VS | PQOSMR_VE;
	netc_port_wr(port, NETC_PQOSMR, val);

	/* Enable L2 and L3 DOS */
	val = netc_port_rd(port, NETC_PCR);
	val |= PCR_L2DOSE | PCR_L3DOSE;
	netc_port_wr(port, NETC_PCR, val);

	/* Enable ISIT key construction rule 0 and 1 */
	val = netc_port_rd(port, NETC_PISIDCR);
	val |= PISIDCR_KC0EN | PISIDCR_KC1EN;
	netc_port_wr(port, NETC_PISIDCR, val);
}

static void netc_port_default_config(struct netc_port *port)
{
	u32 val;

	netc_port_fixed_config(port);

	/* Default VLAN unware */
	val = netc_port_rd(port, NETC_BPDVR);
	if (!(val & BPDVR_RXVAM)) {
		val |= BPDVR_RXVAM;
		netc_port_wr(port, NETC_BPDVR, val);
	}

	if (dsa_port_is_user(port->dp)) {
		netc_port_set_mlo(port, MLO_DISABLE);
	} else {
		val = netc_port_rd(port, NETC_BPCR) | BPCR_SRCPRND;
		val = u32_replace_bits(val, MLO_HW, BPCR_MLO);
		netc_port_wr(port, NETC_BPCR, val);
	}

	netc_port_set_max_frame_size(port, NETC_MAX_FRAME_LEN);
	netc_port_set_all_tc_msdu(port, NULL);
}

static int netc_setup(struct dsa_switch *ds)
{
	struct netc_switch *priv = ds->priv;
	struct netc_port *port;
	int i, err;

	err = netc_init_all_ports(ds);
	if (err)
		return err;

	err = netc_init_ntmp_user(priv);
	if (err)
		goto free_internal_mdiobus;

	netc_switch_fixed_config(priv);

	/* default setting for ports */
	for (i = 0; i < priv->num_ports; i++) {
		port = priv->ports[i];
		if (port->dp)
			netc_port_default_config(port);
	}

	return 0;

free_internal_mdiobus:
	netc_remove_all_ports_internal_mdiobus(ds);

	return err;
}

static void netc_teardown(struct dsa_switch *ds)
{
	struct netc_switch *priv = ds->priv;

	netc_deinit_ntmp_user(priv);
	netc_remove_all_ports_internal_mdiobus(ds);
}

static bool netc_switch_is_emdio_consumer(struct device_node *ports)
{
	struct device_node *phy_node, *mdio_node;

	for_each_available_child_of_node_scoped(ports, child) {
		/* If the node does not have phy-handle property, then
		 * the port does not connect to a PHY, so the port is
		 * not the EMDIO consumer.
		 */
		phy_node = of_parse_phandle(child, "phy-handle", 0);
		if (!phy_node)
			continue;

		of_node_put(phy_node);
		/* If the port node has phy-handle property and it does
		 * not contain a mdio child node, then the switch is the
		 * EMDIO consumer.
		 */
		mdio_node = of_get_child_by_name(child, "mdio");
		if (!mdio_node)
			return true;

		of_node_put(mdio_node);

		return false;
	}

	return false;
}

static int netc_switch_add_emdio_consumer(struct device *dev)
{
	struct phy_device *phydev = NULL, *last_phydev = NULL;
	struct device_node *node = dev->of_node;
	struct device_node *ports, *phy_node;
	struct device_link *link;
	int err = 0;

	ports = of_get_child_by_name(node, "ports");
	if (!ports)
		ports = of_get_child_by_name(node, "ethernet-ports");
	if (!ports)
		return 0;

	if (!netc_switch_is_emdio_consumer(ports))
		goto out;

	for_each_available_child_of_node_scoped(ports, child) {
		phy_node = of_parse_phandle(child, "phy-handle", 0);
		if (!phy_node)
			continue;

		phydev = of_phy_find_device(phy_node);
		of_node_put(phy_node);
		if (!phydev) {
			err = -EPROBE_DEFER;
			goto out;
		}

		if (last_phydev) {
			put_device(&last_phydev->mdio.dev);
			last_phydev = phydev;
		}
	}

	if (phydev) {
		link = device_link_add(dev, phydev->mdio.bus->parent,
				       DL_FLAG_PM_RUNTIME |
				       DL_FLAG_AUTOREMOVE_SUPPLIER);
		put_device(&phydev->mdio.dev);
		if (!link) {
			err = -EINVAL;
			goto out;
		}
	}

out:
	of_node_put(ports);

	return err;
}

static int netc_switch_pci_init(struct pci_dev *pdev)
{
	struct netc_switch *priv __free(kfree) = NULL;
	struct device *dev = &pdev->dev;
	struct netc_switch_regs *regs;
	int err, len;

	pcie_flr(pdev);
	err = pci_enable_device_mem(pdev);
	if (err)
		return dev_err_probe(dev, err, "Failed to enable device\n");

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (err) {
		dev_err(dev, "Failed to configure DMA, err=%d\n", err);
		goto disable_pci_device;
	}

	err = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(dev, "Failed to request memory regions, err=%d\n", err);
		goto disable_pci_device;
	}

	pci_set_master(pdev);
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto relase_mem_regions;
	}

	priv->pdev = pdev;
	priv->dev = dev;

	regs = &priv->regs;
	len = pci_resource_len(pdev, NETC_REGS_BAR);
	regs->base = ioremap(pci_resource_start(pdev, NETC_REGS_BAR), len);
	if (!regs->base) {
		err = -ENXIO;
		dev_err(dev, "ioremap() failed\n");
		goto relase_mem_regions;
	}

	regs->port = regs->base + NETC_REGS_PORT_BASE;
	regs->global = regs->base + NETC_REGS_GLOBAL_BASE;
	pci_set_drvdata(pdev, no_free_ptr(priv));

	return 0;

relase_mem_regions:
	pci_release_mem_regions(pdev);
disable_pci_device:
	pci_disable_device(pdev);

	return err;
}

static void netc_switch_pci_destroy(struct pci_dev *pdev)
{
	struct netc_switch *priv;

	priv = pci_get_drvdata(pdev);

	iounmap(priv->regs.base);
	kfree(priv);
	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static void netc_switch_get_ip_revision(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	u32 val;

	val = netc_glb_rd(regs, NETC_IPBRR0);
	priv->revision = val & IPBRR0_IP_REV;
}

static void netc_phylink_get_caps(struct dsa_switch *ds, int port_id,
				  struct phylink_config *config)
{
	struct netc_switch *priv = ds->priv;

	if (priv->info && priv->info->phylink_get_caps)
		priv->info->phylink_get_caps(port_id, config);
}

static struct phylink_pcs *netc_mac_select_pcs(struct phylink_config *config,
					       phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct netc_switch *priv = dp->ds->priv;

	return priv->ports[dp->index]->pcs;
}

static void netc_port_set_mac_mode(struct netc_port *port,
				   unsigned int mode, phy_interface_t phy_mode)
{
	u32 val;

	val = netc_mac_port_rd(port, NETC_PM_IF_MODE(0));
	val &= ~(PM_IF_MODE_IFMODE | PM_IF_MODE_ENA);

	switch (phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val |= IFMODE_RGMII;
		/* We need to enable auto-negotiation for the MAC
		 * if its RGMII interface support In-Band status.
		 */
		if (phylink_autoneg_inband(mode))
			val |= PM_IF_MODE_ENA;
		break;
	case PHY_INTERFACE_MODE_RMII:
		val |= IFMODE_RMII;
		break;
	case PHY_INTERFACE_MODE_REVMII:
		val |= PM_IF_MODE_REVMII;
		fallthrough;
	case PHY_INTERFACE_MODE_MII:
		val |= IFMODE_MII;
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		val |= IFMODE_SGMII;
		break;
	default:
		break;
	}

	netc_mac_port_wr(port, NETC_PM_IF_MODE(0), val);
}

static void netc_mac_config(struct phylink_config *config, unsigned int mode,
			    const struct phylink_link_state *state)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct netc_switch *priv = dp->ds->priv;

	netc_port_set_mac_mode(priv->ports[dp->index], mode, state->interface);
}

static void netc_port_set_speed(struct netc_port *port, int speed)
{
	u32 old_val = netc_port_rd(port, NETC_PCR);
	u32 val = old_val & (~PCR_PSPEED);

	switch (speed) {
	case SPEED_10:
	case SPEED_100:
	case SPEED_1000:
	case SPEED_2500:
		val |= PSPEED_SET_VAL(speed);
		break;
	default:
		dev_err(port->switch_priv->dev,
			"Unsupported MAC speed:%d\n", speed);
		return;
	}

	port->speed = speed;
	if (val != old_val)
		netc_port_wr(port, NETC_PCR, val);
}

/* If the RGMII device does not support the In-Band Status (IBS), we need
 * the MAC driver to get the link speed and duplex mode from the PHY driver.
 * The MAC driver then sets the MAC for the correct speed and duplex mode
 * to match the PHY. The PHY driver gets the link status and speed and duplex
 * information from the PHY via the MDIO/MDC interface.
 */
static void netc_port_force_set_rgmii_mac(struct netc_port *port,
					  int speed, int duplex)
{
	u32 old_val, val;

	old_val = netc_mac_port_rd(port, NETC_PM_IF_MODE(0));
	val = old_val & ~(PM_IF_MODE_ENA | PM_IF_MODE_M10 | PM_IF_MODE_REVMII);

	switch (speed) {
	case SPEED_1000:
		val = u32_replace_bits(val, SSP_1G, PM_IF_MODE_SSP);
		break;
	case SPEED_100:
		val = u32_replace_bits(val, SSP_100M, PM_IF_MODE_SSP);
		break;
	case SPEED_10:
		val = u32_replace_bits(val, SSP_10M, PM_IF_MODE_SSP);
	}

	val = u32_replace_bits(val, duplex == DUPLEX_FULL ? 0 : 1,
			       PM_IF_MODE_HD);

	if (old_val == val)
		return;

	netc_mac_port_wr(port, NETC_PM_IF_MODE(0), val);
}

static void net_port_set_rmii_mii_mac(struct netc_port *port,
				      int speed, int duplex)
{
	u32 old_val, val;

	old_val = netc_mac_port_rd(port, NETC_PM_IF_MODE(0));
	val = old_val & ~(PM_IF_MODE_ENA | PM_IF_MODE_SSP);

	switch (speed) {
	case SPEED_100:
		val &= ~PM_IF_MODE_M10;
		break;
	case SPEED_10:
		val |= PM_IF_MODE_M10;
	}

	val = u32_replace_bits(val, duplex == DUPLEX_FULL ? 0 : 1,
			       PM_IF_MODE_HD);

	if (old_val == val)
		return;

	netc_mac_port_wr(port, NETC_PM_IF_MODE(0), val);
}

static void netc_port_set_hd_flow_control(struct netc_port *port,
					  bool enable)
{
	u32 old_val, val;

	if (!port->caps.half_duplex)
		return;

	old_val = netc_mac_port_rd(port, NETC_PM_CMD_CFG(0));
	val = u32_replace_bits(old_val, enable ? 1 : 0, PM_CMD_CFG_HD_FCEN);
	if (val == old_val)
		return;

	netc_mac_port_wr(port, NETC_PM_CMD_CFG(0), val);
}

static void netc_port_enable_mac_path(struct netc_port *port,
				      bool enable)
{
	u32 val;

	val = netc_mac_port_rd(port, NETC_PM_CMD_CFG(0));
	if (enable)
		val |= PM_CMD_CFG_TX_EN | PM_CMD_CFG_RX_EN;
	else
		val &= ~(PM_CMD_CFG_TX_EN | PM_CMD_CFG_RX_EN);

	netc_mac_port_wr(port, NETC_PM_CMD_CFG(0), val);
}

static void netc_mac_link_up(struct phylink_config *config,
			     struct phy_device *phy, unsigned int mode,
			     phy_interface_t interface, int speed, int duplex,
			     bool tx_pause, bool rx_pause)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct netc_switch *priv = dp->ds->priv;
	struct netc_port *port;
	bool hd_fc = false;

	port = NETC_PORT(priv, dp->index);
	netc_port_set_speed(port, speed);

	if (phy_interface_mode_is_rgmii(interface) &&
	    !phylink_autoneg_inband(mode)) {
		netc_port_force_set_rgmii_mac(port, speed, duplex);
	}

	if (interface == PHY_INTERFACE_MODE_RMII ||
	    interface == PHY_INTERFACE_MODE_REVMII ||
	    interface == PHY_INTERFACE_MODE_MII) {
		net_port_set_rmii_mii_mac(port, speed, duplex);
	}

	if (duplex == DUPLEX_HALF) {
		if (tx_pause || rx_pause)
			hd_fc = true;
	}

	netc_port_set_hd_flow_control(port, hd_fc);
	netc_port_enable_mac_path(port, true);
}

static void netc_mac_link_down(struct phylink_config *config, unsigned int mode,
			       phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct netc_switch *priv = dp->ds->priv;
	struct netc_port *port;

	port = NETC_PORT(priv, dp->index);
	netc_port_enable_mac_path(port, false);
}

static const struct phylink_mac_ops netc_phylink_mac_ops = {
	.mac_select_pcs		= netc_mac_select_pcs,
	.mac_config		= netc_mac_config,
	.mac_link_up		= netc_mac_link_up,
	.mac_link_down		= netc_mac_link_down,
};

static const struct dsa_switch_ops netc_switch_ops = {
	.get_tag_protocol		= netc_get_tag_protocol,
	.setup				= netc_setup,
	.teardown			= netc_teardown,
	.phylink_get_caps		= netc_phylink_get_caps,
};

static int netc_switch_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct netc_switch *priv;
	struct dsa_switch *ds;
	int err;

	if (!node) {
		dev_info(dev, "No DTS bindings or device is disabled, skipping\n");
		return -ENODEV;
	}

	err = netc_switch_add_emdio_consumer(dev);
	if (err)
		return err;

	err = netc_switch_pci_init(pdev);
	if (err)
		return err;

	priv = pci_get_drvdata(pdev);
	netc_switch_get_ip_revision(priv);

	err = netc_switch_platform_probe(priv);
	if (err)
		goto destroy_netc_switch;

	ds = kzalloc(sizeof(*ds), GFP_KERNEL);
	if (!ds) {
		err = -ENOMEM;
		dev_err(dev, "Failed to allocate DSA switch\n");
		goto destroy_netc_switch;
	}

	ds->dev = dev;
	ds->num_ports = priv->num_ports;
	ds->num_tx_queues = NETC_TC_NUM;
	ds->ops = &netc_switch_ops;
	ds->phylink_mac_ops = &netc_phylink_mac_ops;
	ds->priv = priv;

	priv->ds = ds;
	priv->tag_proto = DSA_TAG_PROTO_NETC;

	err = dsa_register_switch(ds);
	if (err) {
		dev_err(dev, "Failed to register DSA switch, err=%d\n", err);
		goto free_ds;
	}

	return 0;

free_ds:
	kfree(ds);
destroy_netc_switch:
	netc_switch_pci_destroy(pdev);

	return err;
}

static void netc_switch_remove(struct pci_dev *pdev)
{
	struct netc_switch *priv = pci_get_drvdata(pdev);

	if (!priv)
		return;

	dsa_unregister_switch(priv->ds);
	kfree(priv->ds);
	netc_switch_pci_destroy(pdev);
}

static const struct pci_device_id netc_switch_ids[] = {
	{ PCI_DEVICE(NETC_SWITCH_VENDOR_ID, NETC_SWITCH_DEVICE_ID) },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, netc_switch_ids);

static struct pci_driver netc_switch_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= netc_switch_ids,
	.probe		= netc_switch_probe,
	.remove		= netc_switch_remove,
};
module_pci_driver(netc_switch_driver);

MODULE_DESCRIPTION("NXP NETC Switch driver");
MODULE_LICENSE("Dual BSD/GPL");
