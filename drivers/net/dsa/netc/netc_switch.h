/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef _NETC_SWITCH_H
#define _NETC_SWITCH_H

#include <linux/dsa/tag_netc.h>
#include <linux/fsl/netc_global.h>
#include <linux/fsl/netc_lib.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/pci.h>

#include "netc_switch_hw.h"

#define NETC_REGS_BAR			0
#define NETC_MSIX_TBL_BAR		2
#define NETC_REGS_PORT_BASE		0x4000
/* register block size per port  */
#define NETC_REGS_PORT_SIZE		0x4000
#define PORT_IOBASE(p)			(NETC_REGS_PORT_SIZE * (p))
#define NETC_REGS_GLOBAL_BASE		0x70000

#define NETC_SWITCH_REV_4_3		0x0403

#define NETC_TC_NUM			8
#define NETC_CBDR_NUM			2

/* read data snoop and command buffer descriptor read snoop, coherent
 * copy of cacheable memory, lookup in downstream cache, no allocate
 * on miss.
 * write data snoop, coherent write of cacheable memory, lookup in
 * downstream cache, no allocate on miss (full cache line update)
 * command buffer descriptor write snoop, coherent write of cacheable
 * memory, lookup in downstream cache, no allocate on miss (partial
 * cache line update or unkonwn)
 */
#define NETC_DEFAULT_CMD_CACHE_ATTR	0x2b2b6727

#define NETC_MAX_FRAME_LEN		9600

struct netc_switch_info {
	u32 cpu_port_num;
	u32 usr_port_num;
	void (*phylink_get_caps)(int port, struct phylink_config *config);
};

struct netc_port_caps {
	u32 half_duplex:1; /* indicates the port whether support half-duplex */
	u32 pmac:1;	  /* indicates the port whether has preemption MAC */
	u32 pseudo_link:1;
};

struct netc_switch;

struct netc_port {
	struct netc_switch *switch_priv;
	struct netc_port_caps caps;
	struct dsa_port *dp;
	struct clk *ref_clk; /* RGMII/RMII reference clock */
	int index;

	void __iomem *iobase;
	struct mii_bus *imdio;
	struct phylink_pcs *pcs;

	u32 speed;
	phy_interface_t phy_mode;
};

struct netc_switch_regs {
	void __iomem *base;
	void __iomem *port;
	void __iomem *global;
};

struct netc_switch {
	struct pci_dev *pdev;
	struct device *dev;
	struct dsa_switch *ds;
	u16 revision;

	const struct netc_switch_info *info;
	struct netc_switch_regs regs;
	enum dsa_tag_protocol tag_proto;
	struct netc_port **ports;
	u32 num_ports;

	struct ntmp_user user;
};

#define NETC_PORT(priv, port_id)	((priv)->ports[(port_id)])

/* Generic interfaces for writing/reading Switch registers */
#define netc_reg_rd(addr)		netc_read(addr)
#define netc_reg_wr(addr, v)		netc_write(addr, v)

/* Write/Read Switch base registers */
#define netc_base_rd(r, o)		netc_read((r)->base + (o))
#define netc_base_wr(r, o, v)		netc_write((r)->base + (o), v)

/* Write/Read registers of Switch Port (including pseudo MAC port) */
#define netc_port_rd(p, o)		netc_read((p)->iobase + (o))
#define netc_port_wr(p, o, v)		netc_write((p)->iobase + (o), v)

/* Write/Read Switch global registers */
#define netc_glb_rd(r, o)		netc_read((r)->global + (o))
#define netc_glb_wr(r, o, v)		netc_write((r)->global + (o), v)

int netc_switch_platform_probe(struct netc_switch *priv);

static inline bool is_netc_pseudo_port(struct netc_port *port)
{
	return port->caps.pseudo_link;
}

#endif
