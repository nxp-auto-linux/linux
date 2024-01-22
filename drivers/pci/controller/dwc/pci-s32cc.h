// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for NXP S32CC SoCs
 *
 * Copyright 2019-2023 NXP
 */

#ifndef PCIE_S32CC_H
#define PCIE_S32CC_H

#include <linux/errno.h>
#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/phy/phy.h>
#include <uapi/linux/pci_regs.h>
#include <linux/pcie/nxp-s32cc-pcie-phy-submode.h>

#include "pcie-designware.h"
#include "pci-ioctl-s32cc.h"
#include "pci-dma-s32cc.h"

/* PCIe MSI capabilities register */
#define PCI_MSI_CAP		0x50U
/* MSI Enable bit */
#define MSI_EN			0x10000U

/* PCIe MSI-X capabilities register */
#define PCI_MSIX_CAP		0xB0U
/* MSI-X Enable bit */
#define MSIX_EN			BIT(31)

/* PCIe controller 0 general control 1 (PE0_GEN_CTRL_1) */
#define PE0_GEN_CTRL_1		0x50U
#define   DEVICE_TYPE_LSB	(0)
#define   DEVICE_TYPE_MASK	(0x0000000F)
#define   DEVICE_TYPE		((DEVICE_TYPE_MASK) << (DEVICE_TYPE_LSB))
#define   SRIS_MODE_BIT		(8)
#define   SRIS_MODE_MASK	BIT(SRIS_MODE_BIT)

#define PCI_EXP_CAP_ID_OFFSET	0x70U

/* PCIe controller 0 general control 3 (PE0_GEN_CTRL_3) */
#define PE0_GEN_CTRL_3		0x58U
/* LTSSM Enable. Active high. Set it low to hold the LTSSM in Detect state. */
#define LTSSM_EN_MASK		0x1U

#define LTSSM_STATE_L0		0x11U /* L0 state */
#define LTSSM_STATE_L0S		0x12U /* L0S state */
#define LTSSM_STATE_L1_IDLE	0x14U /* L1_IDLE state */

#define LINK_INT_CTRL_STS	0x40U
#define LINK_REQ_RST_NOT_INT_EN	BIT(1)
#define LINK_REQ_RST_NOT_CLR	BIT(2)

#define PE0_INT_STS		0xE8U
#define HP_INT_STS		BIT(6)

#define PCI_BASE_CLASS_OFF	24U
#define PCI_SUBCLASS_OTHER	0x80U
#define PCI_SUBCLASS_OFF	16U

#define PCI_DEVICE_ID_SHIFT	16

#define SERDES_CELL_SIZE	4

#define to_s32cc_from_dw_pcie(x) \
	container_of(x, struct s32cc_pcie, pcie)

#define to_s32cc_from_dma_info(x) \
		container_of(x, struct s32cc_pcie, dma)

enum pcie_link_speed {
	GEN1 = 0x1,
	GEN2 = 0x2,
	GEN3 = 0x3
};

struct s32cc_pcie_data {
	enum dw_pcie_device_mode mode;
};

struct s32cc_pcie {
	struct dw_pcie	pcie;

	bool has_msi_parent;
#if (IS_ENABLED(CONFIG_PM_SLEEP))
	u32 msi_ctrl_int;
#endif

	/* we have cfg in struct pcie_port and
	 * dbi in struct dw_pcie, so define only ctrl here
	 */
	void __iomem *ctrl_base;

	enum dw_pcie_device_mode mode;
	int id;
	enum pcie_phy_mode phy_mode;
	enum pcie_link_speed linkspeed;

	/* eDMA related */
	int dma_irq;
	struct dma_info	dma;

	/* For interaction with the user space */
	struct s32cc_userspace_info uinfo;

	struct phy *phy0, *phy1;

	struct resource shared_mem;
	bool auto_config_bars;
};

static inline
bool is_s32cc_pcie_rc(enum dw_pcie_device_mode mode)
{
	return mode == DW_PCIE_RC_TYPE;
}

static inline
bool is_s32cc_pcie_ep(enum dw_pcie_device_mode mode)
{
	return mode == DW_PCIE_EP_TYPE;
}

static inline
const char *s32cc_pcie_ep_rc_mode_str(enum dw_pcie_device_mode mode)
{
	return is_s32cc_pcie_rc(mode) ? "RootComplex" : "EndPoint";
}

void dw_pcie_writel_ctrl(struct s32cc_pcie *pci, u32 reg, u32 val);
u32 dw_pcie_readl_ctrl(struct s32cc_pcie *pci, u32 reg);

/* Get the EndPoint data (if any) for the controller with the given ID */
struct s32cc_pcie *s32cc_get_dw_pcie(int pcie_ep_id);

/* Configure Outbound window from ptr_outb for the corresponding EndPoint */
int s32cc_pcie_setup_outbound(struct s32cc_outbound_region *ptr_outb);

/* Configure Inbound window from ptr_inb for the corresponding EndPoint */
int s32cc_pcie_setup_inbound(struct s32cc_inbound_region *ptr_inb);

u8 dw_pcie_iatu_unroll_enabled(struct dw_pcie *pci);
int s32cc_check_serdes(struct device *dev);
int s32cc_pcie_dt_init_common(struct platform_device *pdev,
				struct s32cc_pcie *s32cc_pp);
int s32cc_pcie_config_common(struct s32cc_pcie *s32cc_pp,
					struct platform_device *pdev);
int s32cc_pcie_config_irq(int *irq_id, char *irq_name,
		struct platform_device *pdev,
		irq_handler_t irq_handler, void *irq_arg);
int deinit_controller(struct s32cc_pcie *s32cc_pp);
void s32cc_pcie_shutdown(struct platform_device *pdev);

int s32cc_pcie_link_is_up(struct dw_pcie *pcie);
int s32cc_pcie_start_link(struct dw_pcie *pcie);
void s32cc_pcie_stop_link(struct dw_pcie *pcie);
void s32cc_pcie_write(struct dw_pcie *pci,
		void __iomem *base, u32 reg, size_t size, u32 val);

#if (IS_ENABLED(CONFIG_PM_SLEEP))
int s32cc_pcie_suspend(struct device *dev);
int s32cc_pcie_resume(struct device *dev);
#endif

#endif	/*	PCIE_S32CC_H	*/
