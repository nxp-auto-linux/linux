// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Freescale S32Gen1 SoCs
 *
 * Copyright 2020-2021 NXP
 */

#ifndef PCIE_S32GEN1_H

#define PCIE_S32GEN1_H

#include <linux/errno.h>
#include <linux/types.h>
#include <linux/version.h>
#include "pcie-designware.h"

/* PCIe MSI capabilities register */
#define PCI_MSI_CAP		0x50
/* MSI Enable bit */
#define MSI_EN			0x10000

/* PCIe MSI-X capabilities register */
#define PCI_MSIX_CAP	0xB0
/* MSI-X Enable bit */
#define MSIX_EN			BIT(31)

/* PCIe Capabilities ID and next pointer register */
#define PCI_EXP_CAP_ID	0x70

/* PCIe controller general control 1 (PE0_GEN_CTRL_1 / PE1_GEN_CTRL_1) */
#define PE_GEN_CTRL_1		0x1050

#define PCI_EXP_CAP_ID_OFFSET	0x70

/* PCIe controller 0 general control 3 (PE0_GEN_CTRL_3) */
#define PE0_GEN_CTRL_3		0x1058
/* Configuration Request Retry Status (CRS) Enable. Active high. */
/* Defer incoming configuration requests. */
#define CRS_EN					0x2
/* LTSSM Enable. Active high. Set it low to hold the LTSSM in Detect state. */
#define LTSSM_EN				0x1

#define LTSSM_STATE_L0		0x11 /* L0 state */

#define to_s32gen1_from_dw_pcie(x) \
	container_of(x, struct s32gen1_pcie, pcie)

#define PCIE_NR_REGIONS		6

enum pcie_dev_type {
	PCIE_EP = 0x0,
	PCIE_RC = 0x4
};

enum pcie_link_speed {
	GEN1 = 0x1,
	GEN2 = 0x2,
	GEN3 = 0x3
};

#ifdef CONFIG_PCI_S32GEN1_ACCESS_FROM_USER
struct userspace_info {
	int			user_pid;
	struct siginfo	info;    /* signal information */
	int (*send_signal_to_user)(struct s32v234_pcie *s32v234_pcie);
};
#endif

struct callback {
	void (*call_back)(u32 arg);
	struct list_head callback_list;
};

struct s32gen1_pcie {
	bool is_endpoint;
	bool has_msi_parent;
	struct dw_pcie	pcie;

	/* we have cfg in struct pcie_port and
	 * dbi in struct dw_pcie, so define only ctrl here
	 */
	void __iomem *ctrl_base;
#if KERNEL_VERSION(5, 0, 0) > LINUX_VERSION_CODE
	void __iomem *atu_base;
#endif

	int id;
	enum pcie_link_speed linkspeed;

#ifdef CONFIG_PCI_DW_DMA
	int dma_irq;
	struct dma_info	dma;
#endif

#ifdef CONFIG_PCI_S32GEN1_ACCESS_FROM_USER
	struct dentry	*dir;
	struct userspace_info uspace;
#endif

	/* TODO: change this to a list */
	void (*call_back)(u32 arg);
};

struct s32_inbound_region {
	u32 bar_nr;
	u32 target_addr;
	u32 region; /* for backwards compatibility */
};
struct s32_outbound_region {
	u64 target_addr;
	u64 base_addr;
	u32 size;
	/* region_type - for backwards compatibility;
	 * must be PCIE_ATU_TYPE_MEM
	 */
	u32 region_type;
};

void dw_pcie_writel_ctrl(struct s32gen1_pcie *pci, u32 reg, u32 val);
u32 dw_pcie_readl_ctrl(struct s32gen1_pcie *pci, u32 reg);

#endif	/*	PCIE_S32GEN1_H	*/
