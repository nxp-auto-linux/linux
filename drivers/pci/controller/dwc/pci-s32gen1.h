// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Freescale S32Gen1 SoCs
 *
 * Copyright 2020-2022 NXP
 */

#ifndef PCIE_S32GEN1_H

#define PCIE_S32GEN1_H

#include <linux/errno.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/phy/phy.h>
#include <uapi/linux/pci_regs.h>
#include <linux/pcie/fsl-s32gen1-pcie-phy-submode.h>

#include "pcie-designware.h"
#include "pci-ioctl-s32.h"
#include "pci-dma-s32.h"

#define BUILD_BIT_VALUE(field, x) (((x) & (1)) << field##_BIT)
#define BUILD_MASK_VALUE(field, x) (((x) & (field##_MASK)) << field##_LSB)

/* PCIe MSI capabilities register */
#define PCI_MSI_CAP		0x50
/* MSI Enable bit */
#define MSI_EN			0x10000

/* PCIe MSI-X capabilities register */
#define PCI_MSIX_CAP	0xB0
/* MSI-X Enable bit */
#define MSIX_EN			BIT(31)

/* PCIe controller 0 general control 1 (PE0_GEN_CTRL_1) */
#define PE0_GEN_CTRL_1			0x50
#define   DEVICE_TYPE_LSB		(0)
#define   DEVICE_TYPE_MASK		(0x0000000F)
#define   DEVICE_TYPE			((DEVICE_TYPE_MASK) << \
					(DEVICE_TYPE_LSB))
#define   SRIS_MODE_BIT			(8)
#define   SRIS_MODE_MASK		BIT(SRIS_MODE_BIT)

#define PCI_EXP_CAP_ID_OFFSET	0x70

/* PCIe controller 0 general control 3 (PE0_GEN_CTRL_3) */
#define PE0_GEN_CTRL_3			0x58
/* LTSSM Enable. Active high. Set it low to hold the LTSSM in Detect state. */
#define LTSSM_EN_MASK			0x1

#define LTSSM_STATE_L0			0x11 /* L0 state */

#define LINK_INT_CTRL_STS		0x40
#define LINK_REQ_RST_NOT_INT_EN	BIT(1)
#define LINK_REQ_RST_NOT_CLR	BIT(2)

#define PE0_INT_STS				0xE8
#define HP_INT_STS				BIT(6)

#define SERDES_CELL_SIZE		4

#define to_s32gen1_from_dw_pcie(x) \
	container_of(x, struct s32gen1_pcie, pcie)

#ifdef CONFIG_PCI_DW_DMA
#define to_s32gen1_from_dma_info(x) \
		container_of(x, struct s32gen1_pcie, dma)
#endif

enum pcie_dev_type {
	PCIE_EP = 0x0,
	PCIE_RC = 0x4
};

enum pcie_link_speed {
	GEN1 = 0x1,
	GEN2 = 0x2,
	GEN3 = 0x3
};

struct s32gen1_pcie {
	bool is_endpoint;
	bool has_msi_parent;
	struct dw_pcie	pcie;

#ifdef CONFIG_PM_SLEEP
	u32 msi_ctrl_int;
#endif

	/* we have cfg in struct pcie_port and
	 * dbi in struct dw_pcie, so define only ctrl here
	 */
	void __iomem *ctrl_base;

	int id;
	enum pcie_phy_mode phy_mode;
	enum pcie_link_speed linkspeed;

#ifdef CONFIG_PCI_DW_DMA
	int dma_irq;
	struct dma_info	dma;
#endif

	/* TODO: change call_back to a list */
	void (*call_back)(u32 arg);
	struct s32_userspace_info uinfo;

	struct phy *phy0, *phy1;

#ifndef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT
	struct resource shared_mem;
#endif
};

void dw_pcie_writel_ctrl(struct s32gen1_pcie *pci, u32 reg, u32 val);
u32 dw_pcie_readl_ctrl(struct s32gen1_pcie *pci, u32 reg);

/* Get the EndPoint data (if any) for the controller with the given ID */
struct s32gen1_pcie *s32_get_dw_pcie(int pcie_ep_id);

/* Configure Outbound window from ptrOutb for the corresponding EndPoint */
int s32_pcie_setup_outbound(struct s32_outbound_region *ptrOutb);

/* Configure Inbound window from ptrInb for the corresponding EndPoint */
int s32_pcie_setup_inbound(struct s32_inbound_region *ptrInb);

#endif	/*	PCIE_S32GEN1_H	*/
