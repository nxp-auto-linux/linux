// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for NXP S32Gen1 SoCs
 *
 * Copyright 2020-2022 NXP
 */

#ifdef CONFIG_PCI_S32_DEBUG
#define DEBUG
#endif

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <linux/of_platform.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/phy.h>
#include <linux/processor.h>
#include <linux/nvmem-consumer.h>
#include <linux/ioport.h>
#include <soc/s32/revision.h>

#include "pci-s32gen1-regs.h"
#include "pci-s32gen1.h"
#include "../../pci.h"

#ifdef CONFIG_PCI_S32_DEBUG_WRITES
#define dev_dbg_w dev_dbg
#define PTR_FMT "%px"
#else
#define dev_dbg_w(fmt, ...)
#define PTR_FMT "%p"
#endif

#define PCIE_LINKUP_MASK	(PCIE_SS_SMLH_LINK_UP | PCIE_SS_RDLH_LINK_UP | \
			PCIE_SS_SMLH_LTSSM_STATE)
#define PCIE_LINKUP_EXPECT	(PCIE_SS_SMLH_LINK_UP | PCIE_SS_RDLH_LINK_UP | \
			PCIE_SS_SMLH_LTSSM_STATE_VALUE(LTSSM_STATE_L0))

/* Default timeout (ms) */
#define PCIE_CX_CPL_BASE_TIMER_VALUE	100

/* PHY link timeout */
#define PCIE_LINK_TIMEOUT_MS	1000
#define PCIE_LINK_TIMEOUT_US	(PCIE_LINK_TIMEOUT_MS * USEC_PER_MSEC)
#define PCIE_LINK_WAIT_US	100

#define PCIE_EP_RC_MODE(ep_mode) ((ep_mode) ? "EndPoint" : "RootComplex")

#define PCI_BASE_CLASS_OFF	24
#define PCI_SUBCLASS_OTHER	(0x80)
#define PCI_SUBCLASS_OFF	16

#define PCIE_EP_DEFAULT_BAR_SIZE	SZ_1M
#define PCI_BASE_ADDRESS_MEM_NON_PREFETCH	0x00	/* non-prefetchable */
#define PCIE_EP_BAR_DEFAULT_INIT_FLAGS	(PCI_BASE_ADDRESS_SPACE_MEMORY | \
			PCI_BASE_ADDRESS_MEM_TYPE_32 | \
			PCI_BASE_ADDRESS_MEM_NON_PREFETCH)

#ifdef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT

#ifndef CONFIG_SYS_PCI_EP_MEMORY_BASE
#define CONFIG_SYS_PCI_EP_MEMORY_BASE 0xc0000000
#endif /* CONFIG_SYS_PCI_EP_MEMORY_BASE */

/* EP BARs */

#define PCIE_EP_BAR0_ADDR		CONFIG_SYS_PCI_EP_MEMORY_BASE
#define PCIE_EP_BAR0_SIZE		SZ_1M
#define PCIE_EP_BAR1_ADDR		(PCIE_EP_BAR0_ADDR + PCIE_EP_BAR0_SIZE)
#define PCIE_EP_BAR1_SIZE		0
#define PCIE_EP_BAR2_ADDR		(PCIE_EP_BAR1_ADDR + PCIE_EP_BAR1_SIZE)
#define PCIE_EP_BAR2_SIZE		(4 * SZ_1M)
#define PCIE_EP_BAR3_ADDR		(PCIE_EP_BAR2_ADDR + PCIE_EP_BAR2_SIZE)
#define PCIE_EP_BAR3_SIZE		0
#define PCIE_EP_BAR4_ADDR		(PCIE_EP_BAR3_ADDR + PCIE_EP_BAR3_SIZE)
#define PCIE_EP_BAR4_SIZE		0
#define PCIE_EP_BAR5_ADDR		(PCIE_EP_BAR4_ADDR + PCIE_EP_BAR4_SIZE)
#define PCIE_EP_BAR5_SIZE		0

#define PCIE_EP_BAR_INIT(bar_no) \
		{PCIE_EP_BAR ## bar_no ## _ADDR, \
			NULL, \
			PCIE_EP_BAR ## bar_no ## _SIZE, \
			BAR_ ## bar_no, \
			PCIE_EP_BAR_DEFAULT_INIT_FLAGS}

static struct pci_epf_bar s32gen1_ep_bars[] = {
		PCIE_EP_BAR_INIT(0),
		PCIE_EP_BAR_INIT(1),
		PCIE_EP_BAR_INIT(2),
		PCIE_EP_BAR_INIT(3),
		PCIE_EP_BAR_INIT(4),
		PCIE_EP_BAR_INIT(5)
};
#endif

#define PCI_DEVICE_ID_SHIFT	16

struct s32gen1_pcie_data {
	enum dw_pcie_device_mode mode;
};

#define xstr(s) str(s)
#define str(s) #s

/* For kernel version less than 5.0.0, unrolled access to iATU
 * is done using a hardcoded iATU offset (0x3 << 20), which is
 * wrong and we must patch it.
 * Starting with kernel version 5.0.0, struct dw_pcie has a new
 * member: void __iomem		*atu_base;
 * and dedicated functions for accessing that memory space:
 * dw_pcie_writel_atu() and dw_pcie_readl_atu().
 */

static inline void s32gen1_pcie_write(struct dw_pcie *pci,
		void __iomem *base, u32 reg, size_t size, u32 val)
{
	int ret;
	struct s32gen1_pcie *s32_pci = to_s32gen1_from_dw_pcie(pci);

#ifdef CONFIG_PCI_S32_DEBUG_WRITES
	if ((uintptr_t)base == (uintptr_t)(s32_pci->ctrl_base))
		dev_dbg_w(pci->dev, "W%d(ctrl+0x%x, 0x%x)\n",
			(int)size * 8, (u32)(reg), (u32)(val));
	else if ((uintptr_t)base == (uintptr_t)(pci->atu_base))
		dev_dbg_w(pci->dev, "W%d(atu+0x%x, 0x%x)\n",
			(int)size * 8, (u32)(reg), (u32)(val));
	else if ((uintptr_t)base == (uintptr_t)(pci->dbi_base))
		dev_dbg_w(pci->dev, "W%d(dbi+0x%x, 0x%x)\n",
			(int)size * 8, (u32)(reg), (u32)(val));
	else if ((uintptr_t)base == (uintptr_t)(pci->dbi_base2))
		dev_dbg_w(pci->dev, "W%d(dbi2+0x%x, 0x%x)\n",
			(int)size * 8, (u32)(reg), (u32)(val));
#ifdef CONFIG_PCI_DW_DMA
	else if ((uintptr_t)base == (uintptr_t)(s32_pci->dma.dma_base))
		dev_dbg_w(pci->dev, "W%d(dma+0x%x, 0x%x)\n",
			(int)size * 8, (u32)(reg), (u32)(val));
#endif
	else
		dev_dbg_w(pci->dev, "W%d(%lx+0x%x, 0x%x)\n",
			(int)size * 8, (uintptr_t)(base), (u32)(reg), (u32)(val));
#endif

	ret = dw_pcie_write(base + reg, size, val);
	if (ret)
		dev_err(pci->dev, "(pcie%d): Write to address 0x%lx failed\n",
			s32_pci->id, (uintptr_t)(base + reg));
}

#if (defined(CONFIG_PCI_DW_DMA) && defined(CONFIG_PCI_S32_DEBUG_WRITES))
/* Allow printing DMA writes */
static inline void s32gen1_pcie_write_dma(struct dma_info *di,
		void __iomem *base, u32 reg, size_t size, u32 val)
{
	struct s32gen1_pcie *s32_pci = to_s32gen1_from_dma_info(di);

	s32gen1_pcie_write(&s32_pci->pcie, base, reg, size, val);
}
#endif

void dw_pcie_writel_ctrl(struct s32gen1_pcie *pci, u32 reg, u32 val)
{
	s32gen1_pcie_write(&pci->pcie, pci->ctrl_base, reg, 0x4, val);
}

u32 dw_pcie_readl_ctrl(struct s32gen1_pcie *pci, u32 reg)
{
	u32 val = 0;

	if (dw_pcie_read(pci->ctrl_base + reg, 0x4, &val))
		dev_err(pci->pcie.dev, "Read ctrl address failed\n");

	return val;
}

static LIST_HEAD(s32gen1_pcie_ep_list);
static DEFINE_MUTEX(s32gen1_pcie_ep_list_mutex);

struct s32gen1_pcie_ep_node {
	struct list_head list;
	struct s32gen1_pcie *ep;
};

#ifdef CONFIG_PCI_DW_DMA

struct dma_info *dw_get_dma_info(struct dw_pcie *pcie)
{
	struct s32gen1_pcie *s32_pp =
		to_s32gen1_from_dw_pcie(pcie);
	return &s32_pp->dma;
}

static irqreturn_t s32gen1_pcie_dma_handler(int irq, void *arg)
{
	struct s32gen1_pcie *s32_pp = arg;
	struct dma_info *di = &(s32_pp->dma);
	u32 dma_error = DMA_ERR_NONE;

	u32 val_write = dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_STATUS);
	u32 val_read = dw_pcie_readl_dma(di, PCIE_DMA_READ_INT_STATUS);

	if (val_write) {
		bool signal = (di->wr_ch.status == DMA_CH_RUNNING);

		dma_error = dw_handle_dma_irq_write(di, val_write);
		if (dma_error != DMA_ERR_NONE)
			dev_info(s32_pp->pcie.dev, "dma write error 0x%0x.\n",
					dma_error);
#ifdef CONFIG_PCI_EPF_TEST
		else if (di->complete)
			complete(di->complete);
#endif

		if (signal && s32_pp->uinfo.send_signal_to_user)
			s32_pp->uinfo.send_signal_to_user(&s32_pp->uinfo);

		if (s32_pp->call_back)
			s32_pp->call_back(val_write);
	}
	if (val_read) {
		bool signal = (di->rd_ch.status == DMA_CH_RUNNING);

		dma_error = dw_handle_dma_irq_read(di, val_read);
		if (dma_error != DMA_ERR_NONE)
			dev_info(s32_pp->pcie.dev, "dma read error 0x%0x.\n",
					dma_error);
#ifdef CONFIG_PCI_EPF_TEST
		else if (di->complete)
			complete(di->complete);
#endif

		if (signal && s32_pp->uinfo.send_signal_to_user)
			s32_pp->uinfo.send_signal_to_user(&s32_pp->uinfo);
	}

	return IRQ_HANDLED;
}
#endif /* CONFIG_PCI_DW_DMA */

struct s32_userspace_info *dw_get_userspace_info(struct dw_pcie *pcie)
{
	struct s32gen1_pcie *s32_pci = to_s32gen1_from_dw_pcie(pcie);

	return &s32_pci->uinfo;
}

void s32_register_callback(struct dw_pcie *pcie,
		void (*call_back)(u32 arg))
{
	struct s32gen1_pcie *s32_pci = to_s32gen1_from_dw_pcie(pcie);

	s32_pci->call_back = call_back;
}
EXPORT_SYMBOL(s32_register_callback);

static bool s32gen1_has_msi_parent(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct s32gen1_pcie *s32_pci = to_s32gen1_from_dw_pcie(pcie);

	return s32_pci->has_msi_parent;
}

#ifdef CONFIG_PCI_S32GEN1_EP_MSI
/* Chained MSI interrupt service routine, for EP */
static void dw_ep_chained_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct pcie_port *pp;

	chained_irq_enter(chip, desc);

	pp = irq_desc_get_handler_data(desc);
	dw_handle_msi_irq(pp);

	chained_irq_exit(chip, desc);
}
#endif

static u8 dw_pcie_iatu_unroll_enabled(struct dw_pcie *pci)
{
	u32 val;

	val = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT);
	if (val == 0xffffffff)
		return 1;

	return 0;
}

static int s32gen1_check_serdes(struct device *dev)
{
	struct nvmem_cell *serdes_cell;
	size_t read_len = 0;
	u8 *serdes = NULL;
	int ret = 0;

	serdes_cell = devm_nvmem_cell_get(dev, "serdes_presence");
	if (IS_ERR(serdes_cell)) {
		if (PTR_ERR(serdes_cell) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get serdes cell\n");

		return PTR_ERR(serdes_cell);
	}

	serdes = nvmem_cell_read(serdes_cell, &read_len);
	devm_nvmem_cell_put(dev, serdes_cell);
	if (IS_ERR(serdes)) {
		dev_err(dev, "Failed to read serdes cell\n");
		return PTR_ERR(serdes);
	} else if (read_len != SERDES_CELL_SIZE) {
		dev_err(dev, "Invalid read size of serdes cell\n");
		ret = -EINVAL;
		goto out;
	}

	if (!(*serdes)) {
		dev_err(dev, "SerDes subsystem is not present, skipping configuring PCIe\n");
		ret = -ENODEV;
	}

out:
	kfree(serdes);
	return ret;
}

/* Return the s32gen1_pcie object for the EP with the given PCIe ID,
 * as specified in the device tree
 */
struct s32gen1_pcie *s32_get_dw_pcie(int pcie_ep_id)
{
	struct s32gen1_pcie_ep_node *pci_node;
	struct s32gen1_pcie *res = ERR_PTR(-ENODEV);

	mutex_lock(&s32gen1_pcie_ep_list_mutex);
	list_for_each_entry(pci_node, &s32gen1_pcie_ep_list, list) {
#ifndef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT
		if (pci_node->ep->id == pcie_ep_id) {
			res = pci_node->ep;
			break;
		}
#else
		res = pci_node->ep;
		break;
#endif
	}
	mutex_unlock(&s32gen1_pcie_ep_list_mutex);

	return res;
}
EXPORT_SYMBOL(s32_get_dw_pcie);

static int s32gen1_add_pcie_ep_to_list(struct s32gen1_pcie *s32_ep)
{
	struct s32gen1_pcie_ep_node *ep_entry;

	if (!s32_ep)
		return -EINVAL;

	ep_entry = kmalloc(sizeof(struct s32gen1_pcie_ep_node),
				  GFP_KERNEL);
	if (!ep_entry)
		return -ENOMEM;

	ep_entry->ep = s32_ep;
	mutex_lock(&s32gen1_pcie_ep_list_mutex);
	list_add(&ep_entry->list, &s32gen1_pcie_ep_list);
	mutex_unlock(&s32gen1_pcie_ep_list_mutex);

	return 0;
}

static void s32gen1_del_pcie_ep_from_list(int pcie_id)
{
	struct s32gen1_pcie_ep_node *ep_entry;

	mutex_lock(&s32gen1_pcie_ep_list_mutex);
	list_for_each_entry(ep_entry, &s32gen1_pcie_ep_list, list) {
		if (ep_entry->ep->id == pcie_id) {
			list_del(&ep_entry->list);
			kfree(ep_entry);
			break;
		}
	}
	mutex_unlock(&s32gen1_pcie_ep_list_mutex);
}

static void s32gen1_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pcie;
	u32 tmp = 0;

	if (!ep) {
		pr_err("%s: No S32Gen1 EP configuration found\n", __func__);
		return;
	}
	pcie = to_dw_pcie_from_ep(ep);
	dev_dbg(pcie->dev, "%s\n", __func__);

	pcie->iatu_unroll_enabled = dw_pcie_iatu_unroll_enabled(pcie);
	dev_dbg(pcie->dev, "iATU unroll: %s\n",
			pcie->iatu_unroll_enabled ? "enabled" : "disabled");

	dw_pcie_dbi_ro_wr_en(pcie);

	/*
	 * Configure the class and revision for the EP device,
	 * to enable human friendly enumeration by the RC (e.g. by lspci)
	 * EPF will set its own IDs.
	 */
	tmp = dw_pcie_readl_dbi(pcie, PCI_CLASS_REVISION) |
			((PCI_BASE_CLASS_PROCESSOR << PCI_BASE_CLASS_OFF) |
			(PCI_SUBCLASS_OTHER << PCI_SUBCLASS_OFF));
	dw_pcie_writel_dbi(pcie, PCI_CLASS_REVISION, tmp);

	dev_dbg(pcie->dev, "%s: Enable MSI/MSI-X capabilities\n", __func__);

	/* Enable MSIs by setting the capability bit */
	tmp = dw_pcie_readl_dbi(pcie, PCI_MSI_CAP) | MSI_EN;
	dw_pcie_writel_dbi(pcie, PCI_MSI_CAP, tmp);

	/* Enable MSI-Xs by setting the capability bit */
	tmp = dw_pcie_readl_dbi(pcie, PCI_MSIX_CAP) | MSIX_EN;
	dw_pcie_writel_dbi(pcie, PCI_MSIX_CAP, tmp);

	/* CMD reg:I/O space, MEM space, and Bus Master Enable */
	tmp = dw_pcie_readl_dbi(pcie, PCI_COMMAND) |
			PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
	dw_pcie_writel_dbi(pcie, PCI_COMMAND, tmp);

	dw_pcie_dbi_ro_wr_dis(pcie);
}

/* Only for EP. Currently only one EP supported. */
int s32_pcie_setup_outbound(struct s32_outbound_region *ptrOutb)
{
	int ret = 0;
	struct pci_epc *epc;
	struct s32gen1_pcie *s32gen1_pcie_ep;

	if (!ptrOutb) {
		pr_err("%s: Invalid Outbound data\n", __func__);
		return -EINVAL;
	}

#ifndef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT
	s32gen1_pcie_ep = s32_get_dw_pcie(ptrOutb->pcie_id);
	if (IS_ERR(s32gen1_pcie_ep)) {
		pr_err("%s: No S32Gen1 EP configuration found for PCIe%d\n",
			__func__, ptrOutb->pcie_id);
		return -ENODEV;
	}
#else
	s32gen1_pcie_ep = s32_get_dw_pcie(0);
	if (IS_ERR(s32gen1_pcie_ep)) {
		pr_err("%s: No S32Gen1 EP configuration found\n",
			__func__);
		return -ENODEV;
	}
#endif

	dev_dbg(s32gen1_pcie_ep->pcie.dev, "%s\n", __func__);

	epc = s32gen1_pcie_ep->pcie.ep.epc;

	if (!epc || !epc->ops) {
		dev_err(s32gen1_pcie_ep->pcie.dev,
			"Invalid S32Gen1 EP configuration\n");
		return -ENODEV;
	}

	/* Setup outbound region */
	ret = epc->ops->map_addr(epc, 0, ptrOutb->base_addr,
			ptrOutb->target_addr, ptrOutb->size);

	return ret;
}
EXPORT_SYMBOL(s32_pcie_setup_outbound);

int s32_pcie_setup_inbound(struct s32_inbound_region *ptrInb)
{
	int ret = 0;
	struct pci_epc *epc;
	int bar_num;
#ifdef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT
	int idx;
#else
	struct pci_epf_bar bar = {
		.size = PCIE_EP_DEFAULT_BAR_SIZE,
		.flags = PCIE_EP_BAR_DEFAULT_INIT_FLAGS,
	};
#endif
	struct s32gen1_pcie *s32gen1_pcie_ep;

	if (!ptrInb) {
		pr_err("%s: Invalid Inbound data\n", __func__);
		return -EINVAL;
	}

#ifndef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT
	s32gen1_pcie_ep = s32_get_dw_pcie(ptrInb->pcie_id);
	if (IS_ERR(s32gen1_pcie_ep)) {
		pr_err("%s: No S32Gen1 EP configuration found for PCIe%d\n",
			__func__, ptrInb->pcie_id);
		return -ENODEV;
	}
#else
	s32gen1_pcie_ep = s32_get_dw_pcie(0);
	if (IS_ERR(s32gen1_pcie_ep)) {
		pr_err("%s: No S32Gen1 EP configuration found\n",
			__func__);
		return -ENODEV;
	}
#endif

	dev_dbg(s32gen1_pcie_ep->pcie.dev, "%s\n", __func__);

	epc = s32gen1_pcie_ep->pcie.ep.epc;

	if (!epc || !epc->ops) {
		dev_err(s32gen1_pcie_ep->pcie.dev,
			"Invalid S32Gen1 EP configuration\n");
		return -ENODEV;
	}

	/* Setup inbound region */
	bar_num = ptrInb->bar_nr;
	if (bar_num < BAR_0 || bar_num >= BAR_5) {
		dev_err(s32gen1_pcie_ep->pcie.dev,
			"Invalid BAR number (%d)\n", bar_num);
		return -EINVAL;
	}

#ifndef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT
	bar.barno = bar_num;
	bar.phys_addr = ptrInb->target_addr;
	ret = epc->ops->set_bar(epc, 0, &bar);
#else

	s32gen1_ep_bars[bar_num].phys_addr = ptrInb->target_addr;
	if (!s32gen1_ep_bars[bar_num].size)
		s32gen1_ep_bars[bar_num].size = PCIE_EP_DEFAULT_BAR_SIZE;

	/* Setup BARs and inbound regions, up to BAR4 inclusivelly */
	for (idx = bar_num; (idx < BAR_5); idx++) {
		if (s32gen1_ep_bars[idx].size) {
			ret = epc->ops->set_bar(epc, 0,
					&s32gen1_ep_bars[idx]);
			if (idx + 1 <= BAR_5) {
				dma_addr_t	next_phys_addr =
					s32gen1_ep_bars[idx].phys_addr +
					s32gen1_ep_bars[idx].size;

				/* shift following BARs if address spaces interfere */
				if (next_phys_addr > s32gen1_ep_bars[idx + 1].phys_addr)
					s32gen1_ep_bars[idx + 1].phys_addr = next_phys_addr;
			}
			if (ret) {
				dev_err(s32gen1_pcie_ep->pcie.dev,
						"%s: Unable to init BAR%d\n",
						__func__, idx);
			}
		}
	}
#endif

	return ret;
}
EXPORT_SYMBOL(s32_pcie_setup_inbound);

static void s32gen1_pcie_enable_hot_unplug_irq(struct s32gen1_pcie *pci)
{
	u32 tmp = dw_pcie_readl_ctrl(pci, LINK_INT_CTRL_STS) |
			LINK_REQ_RST_NOT_INT_EN;

	dw_pcie_writel_ctrl(pci, LINK_INT_CTRL_STS, tmp);
}

static void s32gen1_pcie_disable_hot_unplug_irq(struct s32gen1_pcie *pci)
{
	u32 tmp = dw_pcie_readl_ctrl(pci, LINK_INT_CTRL_STS)
			& ~(LINK_REQ_RST_NOT_INT_EN);

	dw_pcie_writel_ctrl(pci, LINK_INT_CTRL_STS, tmp);
}

static void s32gen1_pcie_disable_hot_plug_irq(struct dw_pcie *pcie)
{
	u32 reg;

	dw_pcie_dbi_ro_wr_en(pcie);
	reg = dw_pcie_readl_dbi(pcie, PCIE_SLOT_CONTROL_SLOT_STATUS);
	reg &= ~(PCIE_CAP_PRESENCE_DETECT_CHANGE_EN | PCIE_CAP_HOT_PLUG_INT_EN |
			PCIE_CAP_DLL_STATE_CHANGED_EN);
	dw_pcie_writel_dbi(pcie, PCIE_SLOT_CONTROL_SLOT_STATUS, reg);
	dw_pcie_dbi_ro_wr_dis(pcie);
}

static void s32gen1_pcie_enable_hot_plug_irq(struct dw_pcie *pcie)
{
	u32 reg;

	dw_pcie_dbi_ro_wr_en(pcie);
	reg = dw_pcie_readl_dbi(pcie, PCIE_SLOT_CONTROL_SLOT_STATUS);
	reg |= (PCIE_CAP_PRESENCE_DETECT_CHANGE_EN | PCIE_CAP_HOT_PLUG_INT_EN |
			PCIE_CAP_DLL_STATE_CHANGED_EN);
	dw_pcie_writel_dbi(pcie, PCIE_SLOT_CONTROL_SLOT_STATUS, reg);
	dw_pcie_dbi_ro_wr_dis(pcie);
}

static void s32gen1_pcie_disable_ltssm(struct s32gen1_pcie *pci)
{
	u32 tmp = dw_pcie_readl_ctrl(pci, PE0_GEN_CTRL_3)
			& ~(LTSSM_EN_MASK);

	dev_dbg(pci->pcie.dev, "%s\n", __func__);

	dw_pcie_dbi_ro_wr_en(&pci->pcie);
	dw_pcie_writel_ctrl(pci, PE0_GEN_CTRL_3, tmp);
	dw_pcie_dbi_ro_wr_dis(&pci->pcie);
}

static void s32gen1_pcie_enable_ltssm(struct s32gen1_pcie *pci)
{
	u32 tmp = dw_pcie_readl_ctrl(pci, PE0_GEN_CTRL_3) |
				LTSSM_EN_MASK;

	dev_dbg(pci->pcie.dev, "%s\n", __func__);

	dw_pcie_dbi_ro_wr_en(&pci->pcie);
	dw_pcie_writel_ctrl(pci, PE0_GEN_CTRL_3, tmp);
	dw_pcie_dbi_ro_wr_dis(&pci->pcie);
}

static bool is_s32gen1_pcie_ltssm_enabled(struct s32gen1_pcie *pci)
{
	return (dw_pcie_readl_ctrl(pci, PE0_GEN_CTRL_3) & LTSSM_EN_MASK);
}

static bool has_data_phy_link(struct s32gen1_pcie *s32_pp)
{
	u32 val = dw_pcie_readl_ctrl(s32_pp, PCIE_SS_PE0_LINK_DBG_2);

	return (val & PCIE_LINKUP_MASK) == PCIE_LINKUP_EXPECT;
}

static int s32gen1_pcie_link_is_up(struct dw_pcie *pcie)
{
	struct s32gen1_pcie *s32_pp = to_s32gen1_from_dw_pcie(pcie);

	if (!is_s32gen1_pcie_ltssm_enabled(s32_pp))
		return 0;

	return has_data_phy_link(s32_pp);
}

static int s32gen1_pcie_get_link_speed(struct s32gen1_pcie *s32_pp)
{
	struct dw_pcie *pcie = &s32_pp->pcie;
	u32 cap_offset = dw_pcie_find_capability(pcie, PCI_CAP_ID_EXP);
	u32 link_sta = dw_pcie_readw_dbi(pcie, cap_offset + PCI_EXP_LNKSTA);

	dev_dbg(pcie->dev, "PCIe%d: Speed Gen%d\n", s32_pp->id,
			link_sta & PCI_EXP_LNKSTA_CLS);

	/* return link speed based on negotiated link status */
	return link_sta & PCI_EXP_LNKSTA_CLS;
}

static struct pci_bus *s32gen1_get_child_downstream_bus(struct pci_bus *bus)
{
	struct pci_bus *child, *root_bus = NULL;

	list_for_each_entry(child, &bus->children, node) {
		if (child->parent == bus) {
			root_bus = child;
			break;
		}
	}

	if (!root_bus)
		return ERR_PTR(-ENODEV);

	return root_bus;
}

static int s32gen1_enable_hotplug_cap(struct dw_pcie *pcie)
{
	struct pcie_port *pp = &pcie->pp;
	struct pci_bus *bus = pp->bridge->bus;
	struct pci_bus *root_bus;
	struct pci_dev *dev;
	int pos;
	u32 reg32;
	u16 reg16;

	root_bus = s32gen1_get_child_downstream_bus(bus);
	if (IS_ERR(root_bus)) {
		dev_err(pcie->dev, "Failed to find downstream devices\n");
		return PTR_ERR(root_bus);
	}
	dev = root_bus->self;

	pos = pci_find_capability(dev, PCI_CAP_ID_EXP);
	if (!pos) {
		dev_err(pcie->dev, "Unable to find PCI_CAP_ID_EXP capability\n");
		return -ENXIO;
	}

	pci_read_config_word(dev, pos + PCI_EXP_FLAGS, &reg16);
	reg16 |= PCI_EXP_FLAGS_SLOT;
	pci_write_config_word(dev, pos + PCI_EXP_FLAGS, reg16);

	pcie_capability_read_dword(dev, PCI_EXP_SLTCAP, &reg32);
	reg32 |= (PCI_EXP_SLTCAP_HPC | PCI_EXP_SLTCAP_HPS);
	pcie_capability_write_dword(dev, PCI_EXP_SLTCAP, reg32);

	s32gen1_pcie_enable_hot_plug_irq(pcie);

	return 0;
}

static int s32gen1_pcie_start_link(struct dw_pcie *pcie)
{
	struct s32gen1_pcie *s32_pp = to_s32gen1_from_dw_pcie(pcie);
	u32 tmp, cap_offset;
	int ret = 0, count;

	dev_dbg(pcie->dev, "%s\n", __func__);

	/* Don't do anything for End Point */
	if (s32_pp->is_endpoint) {
		return 0;
	}

	/* Try to (re)establish the link, starting with Gen1 */
	s32gen1_pcie_disable_ltssm(s32_pp);

	dw_pcie_dbi_ro_wr_en(pcie);
	cap_offset = dw_pcie_find_capability(pcie, PCI_CAP_ID_EXP);
	tmp = (dw_pcie_readl_dbi(pcie, cap_offset + PCI_EXP_LNKCAP) &
			~(PCI_EXP_LNKCAP_SLS)) | PCI_EXP_LNKCAP_SLS_2_5GB;
	dw_pcie_writel_dbi(pcie, cap_offset + PCI_EXP_LNKCAP, tmp);
	dw_pcie_dbi_ro_wr_dis(pcie);

	/* Start LTSSM. */
	s32gen1_pcie_enable_ltssm(s32_pp);
	ret = dw_pcie_wait_for_link(pcie);

	if (ret) {
		/* We do not exit with error if link up was unsuccessful
		 * Endpoint may be connected.
		 */
		ret = 0;
		goto out;
	}

	dw_pcie_dbi_ro_wr_en(pcie);
	/* Allow Gen2 or Gen3 mode after the link is up.
	 * s32gen1_pcie.linkspeed is one of the speeds defined in pci_regs.h:
	 * PCI_EXP_LNKCAP_SLS_2_5GB for Gen1
	 * PCI_EXP_LNKCAP_SLS_5_0GB for Gen2
	 * PCI_EXP_LNKCAP_SLS_8_0GB for Gen3
	 */
	tmp = (dw_pcie_readl_dbi(pcie, cap_offset + PCI_EXP_LNKCAP) &
			~(PCI_EXP_LNKCAP_SLS)) | s32_pp->linkspeed;
	dw_pcie_writel_dbi(pcie, cap_offset + PCI_EXP_LNKCAP, tmp);

	/*
	 * Start Directed Speed Change so the best possible speed both link
	 * partners support can be negotiated.
	 * The manual says:
	 * When you set the default of the Directed Speed Change field of the
	 * Link Width and Speed Change Control register
	 * (GEN2_CTRL_OFF.DIRECT_SPEED_CHANGE) using the
	 * DEFAULT_GEN2_SPEED_CHANGE configuration parameter to 1, then
	 * the speed change is initiated automatically after link up, and the
	 * controller clears the contents of GEN2_CTRL_OFF.DIRECT_SPEED_CHANGE.
	 */
	tmp = dw_pcie_readl_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL) |
			PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL, tmp);
	dw_pcie_dbi_ro_wr_dis(pcie);

	count = 1000;
	while (count--) {
		tmp = dw_pcie_readl_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL);
		/* Test if the speed change finished. */
		if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
			break;
		usleep_range(100, 1000);
	}

	/* Make sure link training is finished as well! */
	if (count)
		ret = dw_pcie_wait_for_link(pcie);
	else {
		dev_err(pcie->dev, "Speed change timeout\n");
		ret = -EINVAL;
	}

	if (!ret) {
		dev_info(pcie->dev, "Link up, Gen=%d\n",
				s32gen1_pcie_get_link_speed(s32_pp));
	}

out:
	return ret;
}

static void s32gen1_pcie_stop_link(struct dw_pcie *pcie)
{
	struct s32gen1_pcie *s32_pp = to_s32gen1_from_dw_pcie(pcie);

	s32gen1_pcie_disable_ltssm(s32_pp);
}

#ifdef CONFIG_PCI_MSI
/* msi IRQ handler
 * irq - interrupt number
 * arg - pointer to the "struct pcie_port" object
 */
static irqreturn_t s32gen1_pcie_msi_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;
#ifdef DEBUG
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);

	dev_dbg(pcie->dev, "%s\n", __func__);
#endif

	return dw_handle_msi_irq(pp);
}
#endif

static int s32gen1_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct s32gen1_pcie *s32_pci = to_s32gen1_from_dw_pcie(pcie);
	int ret;

	dev_dbg(pcie->dev, "%s\n", __func__);

	dw_pcie_setup_rc(pp);

	ret = s32gen1_pcie_start_link(pcie);
	if (ret) {
		if (!phy_validate(s32_pci->phy0, PHY_MODE_PCIE, 0, NULL)) {
			dev_err(pcie->dev, "Failed to get link up with EP connected\n");
			return ret;
		}
	}

#ifdef CONFIG_PCI_MSI
	if (!s32gen1_has_msi_parent(pp))
		dw_pcie_msi_init(pp);
#endif

	return 0;
}

#ifdef CONFIG_PCI_MSI
static void s32gen1_pcie_set_num_vectors(struct pcie_port *pp)
{
	pp->num_vectors = MAX_MSI_IRQS;
}
#endif

static struct dw_pcie_ops s32_pcie_ops = {
	.link_up = s32gen1_pcie_link_is_up,
	.start_link = s32gen1_pcie_start_link,
	.stop_link = s32gen1_pcie_stop_link,
	.write_dbi = s32gen1_pcie_write,
};

static int s32gen1_pcie_msi_host_init(struct pcie_port *pp)
{
	return 0;
}

static struct dw_pcie_host_ops s32gen1_pcie_host_ops = {
	.host_init = s32gen1_pcie_host_init,
#ifdef CONFIG_PCI_MSI
	.set_num_vectors = s32gen1_pcie_set_num_vectors
#endif
};

static struct dw_pcie_host_ops s32gen1_pcie_host_ops2 = {
	.host_init = s32gen1_pcie_host_init,
	.msi_host_init = s32gen1_pcie_msi_host_init,
};

static irqreturn_t s32gen1_pcie_hot_unplug_irq(int irq, void *arg)
{
	struct s32gen1_pcie *s32_pci = arg;
	struct dw_pcie *pcie = &s32_pci->pcie;
	u32 tmp = dw_pcie_readl_ctrl(s32_pci, LINK_INT_CTRL_STS) |
				LINK_REQ_RST_NOT_CLR;

	dw_pcie_writel_ctrl(s32_pci, LINK_INT_CTRL_STS, tmp);

	if (s32gen1_pcie_link_is_up(pcie))
		return IRQ_HANDLED;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t s32gen1_pcie_hot_unplug_thread(int irq, void *arg)
{
	struct s32gen1_pcie *s32_pci = arg;
	struct dw_pcie *pcie = &s32_pci->pcie;
	struct pcie_port *pp = &pcie->pp;
	struct pci_bus *bus = pp->bridge->bus;
	struct pci_bus *root_bus;
	struct pci_dev *pdev, *temp;
	int ret;

	pci_lock_rescan_remove();

	root_bus = s32gen1_get_child_downstream_bus(bus);
	if (IS_ERR(root_bus)) {
		dev_err(pcie->dev, "Failed to find downstream devices\n");
		goto out_unlock_rescan;
	}

	/* if EP is not connected -- Hot-Unplug Surprise event */
	if (phy_validate(s32_pci->phy0, PHY_MODE_PCIE, 0, NULL))
		pci_walk_bus(root_bus, pci_dev_set_disconnected, NULL);

	list_for_each_entry_safe_reverse(pdev, temp,
					 &root_bus->devices, bus_list) {
		pci_dev_get(pdev);
		pci_stop_and_remove_bus_device(pdev);
		pci_dev_put(pdev);
	}

	ret = s32gen1_enable_hotplug_cap(pcie);
	if (ret)
		dev_err(pcie->dev, "Failed to enable hotplug capability\n");

out_unlock_rescan:
	pci_unlock_rescan_remove();
	return IRQ_HANDLED;
}

static irqreturn_t s32gen1_pcie_hot_plug_irq(int irq, void *arg)
{
	struct s32gen1_pcie *s32_pci = arg;
	u32 tmp = dw_pcie_readl_ctrl(s32_pci, PE0_INT_STS) |
				HP_INT_STS;

	dw_pcie_writel_ctrl(s32_pci, PE0_INT_STS, tmp);

	/* if EP is not connected, we exit */
	if (phy_validate(s32_pci->phy0, PHY_MODE_PCIE, 0, NULL))
		return IRQ_HANDLED;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t s32gen1_pcie_hot_plug_thread(int irq, void *arg)
{
	struct s32gen1_pcie *s32_pci = arg;
	struct dw_pcie *pcie = &s32_pci->pcie;
	struct pcie_port *pp = &pcie->pp;
	struct pci_bus *bus = pp->bridge->bus;
	struct pci_dev *dev;
	struct pci_bus *root_bus;
	int num, ret;

	pci_lock_rescan_remove();

	root_bus = s32gen1_get_child_downstream_bus(bus);
	if (IS_ERR(root_bus)) {
		dev_err(pcie->dev, "Failed to find downstream devices\n");
		goto out_unlock_rescan;
	}

	num = pci_scan_slot(root_bus, PCI_DEVFN(0, 0));
	if (!num) {
		dev_err(pcie->dev, "No new device found\n");
		goto out_unlock_rescan;
	}

	for_each_pci_bridge(dev, root_bus) {
		ret = pci_hp_add_bridge(dev);
		if (ret)
			dev_warn(pcie->dev, "Failed to add hp bridge\n");
	}

	pci_assign_unassigned_bridge_resources(root_bus->self);
	pcie_bus_configure_settings(root_bus);
	pci_bus_add_devices(root_bus);

out_unlock_rescan:
	pci_unlock_rescan_remove();
	return IRQ_HANDLED;
}

#define MAX_IRQ_NAME_SIZE 32
static int s32gen1_pcie_config_irq(int *irq_id, char *irq_name,
		struct platform_device *pdev,
		irq_handler_t irq_handler, void *irq_arg)
{
	int ret = 0;
	char irq_display_name[MAX_IRQ_NAME_SIZE];

	dev_dbg(&pdev->dev, "%s\n", __func__);

	*(irq_id) = platform_get_irq_byname(pdev, irq_name);
	if (*(irq_id) <= 0) {
		dev_err(&pdev->dev, "failed to get %s irq\n", irq_name);
		return -ENODEV;
	}
	snprintf(irq_display_name, MAX_IRQ_NAME_SIZE, "s32gen1-pcie-%s",
			irq_name);
	ret = devm_request_irq(&pdev->dev, *(irq_id), irq_handler,
			IRQF_SHARED,  irq_name, irq_arg);
	if (ret) {
		dev_info(&pdev->dev, "Register interrupt %d (%s) failed (%d)\n",
			 *(irq_id), irq_name, ret);
		return ret;
	}

	dev_info(&pdev->dev, "Allocated line %d for interrupt %d (%s)",
			ret, *(irq_id), irq_name);

	return 0;
}

static int s32gen1_pcie_config_hp_irq(struct s32gen1_pcie *s32_pp,
				      struct platform_device *pdev)
{
	int irq_id, ret;

	irq_id = platform_get_irq_byname(pdev, "link_req_stat");
	if (irq_id <= 0) {
		dev_err(&pdev->dev, "Failed to get link_req_stat irq\n");
		return -ENODEV;
	}

	ret = request_threaded_irq(irq_id, s32gen1_pcie_hot_unplug_irq,
				   s32gen1_pcie_hot_unplug_thread, IRQF_SHARED,
				   "s32gen1-pcie-hot-unplug", s32_pp);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request link_reg_stat irq\n");
		return ret;
	}

	irq_id = platform_get_irq_byname(pdev, "misc");
	if (irq_id <= 0) {
		dev_err(&pdev->dev, "Failed to get misc irq\n");
		return -ENODEV;
	}

	ret = request_threaded_irq(irq_id, s32gen1_pcie_hot_plug_irq,
				   s32gen1_pcie_hot_plug_thread, IRQF_SHARED,
				   "s32gen1-pcie-hotplug", s32_pp);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request misc irq\n");
		return ret;
	}

	return ret;
}

static int __init s32gen1_add_pcie_port(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	int ret;

	dev_dbg(pcie->dev, "%s\n", __func__);

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(pcie->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int s32gen1_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
		enum pci_epc_irq_type type, u16 interrupt_num)
{
	struct dw_pcie *pci;

	if (!ep) {
		pr_err("%s: No S32Gen1 EP configuration found\n", __func__);
		return -ENODEV;
	}

	pci = to_dw_pcie_from_ep(ep);
	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		dev_dbg(pci->dev, "%s: func %d: legacy int\n",
			__func__, func_no);
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		dev_dbg(pci->dev, "%s: func %d: MSI %d\n",
			__func__, func_no, interrupt_num);
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		dev_dbg(pci->dev, "%s: func %d: MSI-X %d\n",
			__func__, func_no, interrupt_num);
		return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "%s: UNKNOWN IRQ type\n", __func__);
	}

	return 0;
}

static const struct pci_epc_features s32gen1_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = false,
	.msix_capable = true,
	.reserved_bar = BIT(BAR_1) | BIT(BAR_5),
	.bar_fixed_64bit = BIT(BAR_0),
	.bar_fixed_size[0] = SZ_1M,
	.bar_fixed_size[2] = (4 * SZ_1M),
	.bar_fixed_size[3] = SZ_64K,
	.bar_fixed_size[4] = 256,
};

static const struct pci_epc_features*
s32gen1_pcie_ep_get_features(struct dw_pcie_ep *ep)
{
	return &s32gen1_pcie_epc_features;
}

static struct dw_pcie_ep_ops s32gen1_pcie_ep_ops = {
	.ep_init = s32gen1_pcie_ep_init,
	.raise_irq = s32gen1_pcie_ep_raise_irq,
	.get_features = s32gen1_pcie_ep_get_features,
#if (defined(CONFIG_PCI_DW_DMA) && defined(CONFIG_PCI_EPF_TEST))
	.start_dma = dw_pcie_ep_start_dma,
#endif
};

int s32_send_msi(struct dw_pcie *pcie)
{
	/* Trigger the first MSI, since all MSIs are routed through the same
	 * physical interrupt anyway
	 */
	return s32gen1_pcie_ep_raise_irq(&pcie->ep, 1,
		PCI_EPC_IRQ_MSI, 1);
}

static int __init s32gen1_add_pcie_ep(struct s32gen1_pcie *s32_pp)
{
	int ret;
	struct dw_pcie *pcie = &s32_pp->pcie;
	struct dw_pcie_ep *ep = &pcie->ep;
	struct device *dev = pcie->dev;

	dev_dbg(pcie->dev, "%s\n", __func__);

	ep->ops = &s32gen1_pcie_ep_ops;

	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "failed to initialize endpoint\n");
		return ret;
	}
	ret = s32gen1_add_pcie_ep_to_list(s32_pp);
	if (ret)
		dev_warn(s32_pp->pcie.dev, "can't populate EP list\n");

	return 0;
}

static void s32gen1_pcie_shutdown(struct platform_device *pdev)
{
	struct s32gen1_pcie *s32_pp = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (!s32_pp->is_endpoint) {
		/* bring down link, so bootloader gets clean state
		 * in case of reboot
		 */
		s32gen1_pcie_disable_ltssm(s32_pp);
	}

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
}

static const struct of_device_id s32gen1_pcie_of_match[];

static int s32gen1_pcie_dt_init(struct platform_device *pdev,
				struct s32gen1_pcie *s32_pp)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dw_pcie *pcie = &s32_pp->pcie;
	struct resource *res;
	const char *pcie_phy_mode;
	const struct of_device_id *match;
	const struct s32gen1_pcie_data *data;
	enum dw_pcie_device_mode mode;
	struct dw_pcie_ep *ep = &pcie->ep;
	u32 pcie_vendor_id = PCI_VENDOR_ID_FREESCALE, pcie_variant_bits = 0;
	int ret;
#ifndef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT
	struct device_node *shmn;
#endif

	match = of_match_device(s32gen1_pcie_of_match, dev);
	if (!match)
		return -EINVAL;

	data = match->data;
	mode = data->mode;

	s32_pp->is_endpoint = (mode == DW_PCIE_EP_TYPE);
	dev_info(dev, "Configured as %s\n",
		 PCIE_EP_RC_MODE(s32_pp->is_endpoint));

	ret = of_property_read_u32(np, "device_id", &s32_pp->id);
	if (ret) {
		dev_err(dev, "Missing 'device_id' property\n");
		return ret;
	}

	ret = of_property_read_string(np, "nxp,phy-mode", &pcie_phy_mode);
	if (ret) {
		dev_info(dev, "Missing 'nxp,phy-mode' property, using default CRNS\n");
		s32_pp->phy_mode = CRNS;
	} else if (!strcmp(pcie_phy_mode, "crns")) {
		s32_pp->phy_mode = CRNS;
	} else if (!strcmp(pcie_phy_mode, "crss")) {
		s32_pp->phy_mode = CRSS;
	} else if (!strcmp(pcie_phy_mode, "sris")) {
		s32_pp->phy_mode = SRIS;
	} else {
		dev_info(dev, "Unsupported 'nxp,phy-mode' specified, using default CRNS\n");
		s32_pp->phy_mode = CRNS;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pcie->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->dbi_base))
		return PTR_ERR(pcie->dbi_base);
	dev_dbg(dev, "dbi: %pR\n", res);
	dev_dbg(dev, "dbi virt: 0x" PTR_FMT "\n", pcie->dbi_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi2");
	pcie->dbi_base2 = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->dbi_base2))
		return PTR_ERR(pcie->dbi_base2);
	dev_dbg(dev, "dbi2: %pR\n", res);
	dev_dbg(dev, "dbi2 virt: 0x" PTR_FMT "\n", pcie->dbi_base2);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "atu");
	dev_dbg(dev, "atu: %pR\n", res);
	pcie->atu_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->atu_base))
		return PTR_ERR(pcie->atu_base);
	dev_dbg(dev, "atu virt: 0x" PTR_FMT "\n", pcie->atu_base);

#ifdef CONFIG_PCI_DW_DMA
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	dev_dbg(dev, "dma: %pR\n", res);
	s32_pp->dma.dma_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(s32_pp->dma.dma_base))
		return PTR_ERR(s32_pp->dma.dma_base);
	dev_dbg(dev, "dma virt: 0x" PTR_FMT "\n", s32_pp->dma.dma_base);
	s32_pp->dma.iatu_unroll_enabled = dw_pcie_iatu_unroll_enabled(pcie);
#if defined(CONFIG_PCI_S32_DEBUG_WRITES)
	s32_pp->dma.write_dma = s32gen1_pcie_write_dma;
#endif
#endif

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctrl");
	s32_pp->ctrl_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(s32_pp->ctrl_base))
		return PTR_ERR(s32_pp->ctrl_base);
	dev_dbg(dev, "ctrl: %pR\n", res);
	dev_dbg(dev, "ctrl virt: 0x" PTR_FMT "\n", s32_pp->ctrl_base);

	s32_pp->linkspeed = (enum pcie_link_speed)of_pci_get_max_link_speed(np);
	if (s32_pp->linkspeed < GEN1 || s32_pp->linkspeed > GEN3) {
		dev_warn(dev, "Invalid PCIe speed; setting to GEN1\n");
		s32_pp->linkspeed = GEN1;
	}

#ifndef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT
	/* Reserved memory */
	/* Get pointer to shared mem region device node from "memory-region" phandle.
	 * Don't throw errors if not available, just warn and go on without.
	 */
	shmn = of_parse_phandle(np, "shared-mem", 0);
	if (shmn) {
		/* Convert memory region to a struct resource */
		ret = of_address_to_resource(shmn, 0, &s32_pp->shared_mem);
		of_node_put(shmn);
		if (ret) {
			dev_warn(dev, "Failed to translate shared-mem to a resource\n");
			s32_pp->shared_mem.start = 0;
			s32_pp->shared_mem.end = 0;
		}
	} else {
		dev_warn(dev, "No shared-mem node\n");
	}
#endif

	/* This is for EP only */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	dev_dbg(dev, "addr_space: %pR\n", res);
	if (!res)
		return -EINVAL;
	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	/* If "msi-parent" property is present in device tree and the PCIe
	 * is RC, MSIs will not be handled by iMSI-RX (default mechanism
	 * implemented in DesignWare core).
	 * The MSIs will be forwarded through AXI bus to the msi parent,
	 * which should be the GIC, which will generate MSIs as SPIs.
	 */
	if (!s32_pp->is_endpoint && of_parse_phandle(np, "msi-parent", 0))
		s32_pp->has_msi_parent = true;

	ret = of_property_read_u32(np, "pcie_device_id", &pcie_variant_bits);
	if (ret) {
		ret = s32_siul2_nvmem_get_pcie_dev_id(dev, "pcie_variant",
						      &pcie_variant_bits);
		if (ret) {
			dev_info(dev, "Error reading SIUL2 Device ID\n");
			return ret;
		}
	}

	if (!pcie_variant_bits)
		return 0;

	/* Write PCI Vendor and Device ID. */
	pcie_vendor_id |= pcie_variant_bits << PCI_DEVICE_ID_SHIFT;
	dev_dbg(dev, "Setting PCI Device and Vendor IDs to 0x%x:0x%x\n",
		 (u32)(pcie_vendor_id >> PCI_DEVICE_ID_SHIFT),
		 (u32)(pcie_vendor_id & GENMASK(15, 0)));

	dw_pcie_dbi_ro_wr_en(pcie);
	dw_pcie_writel_dbi(pcie, PCI_VENDOR_ID, pcie_vendor_id);

	if (pcie_vendor_id != dw_pcie_readl_dbi(pcie, PCI_VENDOR_ID))
		dev_warn(dev, "PCI Device and Vendor IDs could not be set\n");

	dw_pcie_dbi_ro_wr_dis(pcie);

	return 0;
}

static void disable_equalization(struct dw_pcie *pcie)
{
	u32 val;

	dw_pcie_dbi_ro_wr_en(pcie);

	val = dw_pcie_readl_dbi(pcie, PORT_LOGIC_GEN3_EQ_CONTROL);
	val &= ~(PCIE_GEN3_EQ_FB_MODE | PCIE_GEN3_EQ_PSET_REQ_VEC);
	val |= BUILD_MASK_VALUE(PCIE_GEN3_EQ_FB_MODE, 1) |
		 BUILD_MASK_VALUE(PCIE_GEN3_EQ_PSET_REQ_VEC, 0x84);
	dw_pcie_writel_dbi(pcie, PORT_LOGIC_GEN3_EQ_CONTROL, val);

	dw_pcie_dbi_ro_wr_dis(pcie);

	/* Test value */
	dev_dbg(pcie->dev, "PCIE_PORT_LOGIC_GEN3_EQ_CONTROL: 0x%08x\n",
		dw_pcie_readl_dbi(pcie, PORT_LOGIC_GEN3_EQ_CONTROL));
}

static int find_first_system_ram_res(struct resource *res, void *arg)
{
	u64 *prev_min = arg;

	if (!arg || !res)
		return 0;
	if (res->start < *prev_min)
		*prev_min = res->start;

	/* Continue iteration in walk_system_ram_res() */
	return 0;
}

static u64 get_system_ram_base(void)
{
	u64 prev_min = GENMASK_ULL(40, 40);

	walk_system_ram_res(0x0, GENMASK_ULL(40, 40) - 1, &prev_min,
			    find_first_system_ram_res);
	return prev_min;
}

static void s32_pcie_reset_mstr_ace(struct dw_pcie *pcie)
{
	u64 ddr_base_addr = get_system_ram_base();
	u32 ddr_base_low = lower_32_bits(ddr_base_addr);
	u32 ddr_base_high = upper_32_bits(ddr_base_addr);

	dw_pcie_dbi_ro_wr_en(pcie);
	dw_pcie_writel_dbi(pcie, PORT_LOGIC_COHERENCY_CONTROL_3, 0x0);
	/* Transactions to peripheral targets should be non-coherent,
	 * or Ncore might drop them. Define the start of DDR as seen by Linux
	 * as the boundary between "memory" and "peripherals", with peripherals
	 * being below this boundary, and memory addresses being above it.
	 * One example where this is needed are PCIe MSIs, which use NoSnoop=0
	 * and might end up routed to Ncore.
	 */
	dw_pcie_writel_dbi(pcie, PORT_LOGIC_COHERENCY_CONTROL_1,
		(ddr_base_low & CC_1_MEMTYPE_BOUNDARY_MASK) |
		(CC_1_MEMTYPE_LOWER_PERIPH & CC_1_MEMTYPE_VALUE_MASK));
	dw_pcie_writel_dbi(pcie, PORT_LOGIC_COHERENCY_CONTROL_2, ddr_base_high);
	dw_pcie_dbi_ro_wr_dis(pcie);
}

static int init_pcie(struct s32gen1_pcie *pci)
{
	struct dw_pcie *pcie = &pci->pcie;
	struct device *dev = pcie->dev;
	u32 val;

	if (pci->is_endpoint) {
		val = dw_pcie_readl_ctrl(pci, PE0_GEN_CTRL_1) |
				BUILD_MASK_VALUE(DEVICE_TYPE, PCIE_EP);
		dw_pcie_writel_ctrl(pci, PE0_GEN_CTRL_1, val);
	} else {
		val = dw_pcie_readl_ctrl(pci, PE0_GEN_CTRL_1) |
				BUILD_MASK_VALUE(DEVICE_TYPE, PCIE_RC);
		dw_pcie_writel_ctrl(pci, PE0_GEN_CTRL_1, val);
	}

	if (pci->phy_mode == SRIS) {
		val = dw_pcie_readl_ctrl(pci, PE0_GEN_CTRL_1) |
				SRIS_MODE_MASK;
		dw_pcie_writel_ctrl(pci, PE0_GEN_CTRL_1, val);
	}

	/* Enable writing dbi registers */
	dw_pcie_dbi_ro_wr_en(pcie);

	/* Enable direct speed change */
	val = dw_pcie_readl_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL, val);
	dw_pcie_dbi_ro_wr_dis(pcie);

	/* Disable phase 2,3 equalization */
	disable_equalization(pcie);

	/* Make sure DBI registers are R/W - see dw_pcie_setup_rc */
	dw_pcie_dbi_ro_wr_en(pcie);
	dw_pcie_setup(pcie);
	dw_pcie_dbi_ro_wr_dis(pcie);

	/* Make sure we use the coherency defaults (just in case the settings
	 * have been changed from their reset values
	 */
	s32_pcie_reset_mstr_ace(pcie);

	/* Test value for coherency control reg */
	dev_dbg(dev, "COHERENCY_CONTROL_3_OFF: 0x%08x\n",
		dw_pcie_readl_dbi(pcie, PORT_LOGIC_COHERENCY_CONTROL_3));

	/* Make sure DBI registers are R/W */
	dw_pcie_dbi_ro_wr_en(pcie);

	val = dw_pcie_readl_dbi(pcie, PORT_LOGIC_PORT_FORCE_OFF);
	val |= PCIE_DO_DESKEW_FOR_SRIS;
	dw_pcie_writel_dbi(pcie, PORT_LOGIC_PORT_FORCE_OFF, val);

	if (!pci->is_endpoint) {
		/* Set max payload supported, 256 bytes and
		 * relaxed ordering.
		 */
		val = dw_pcie_readl_dbi(pcie, CAP_DEVICE_CONTROL_DEVICE_STATUS);
		val &= ~(CAP_EN_REL_ORDER | CAP_MAX_PAYLOAD_SIZE_CS |
			 CAP_MAX_READ_REQ_SIZE);
		val |= CAP_EN_REL_ORDER |
			BUILD_MASK_VALUE(CAP_MAX_PAYLOAD_SIZE_CS, 1) |
			BUILD_MASK_VALUE(CAP_MAX_READ_REQ_SIZE, 1),
		dw_pcie_writel_dbi(pcie, CAP_DEVICE_CONTROL_DEVICE_STATUS, val);

		/* Enable the IO space, Memory space, Bus master,
		 * Parity error, Serr and disable INTx generation
		 */
		dw_pcie_writel_dbi(pcie, PCIE_CTRL_TYPE1_STATUS_COMMAND_REG,
				   PCIE_SERREN | PCIE_PERREN | PCIE_INT_EN |
				   PCIE_IO_EN | PCIE_MSE | PCIE_BME);
		/* Test value */
		dev_dbg(dev, "PCIE_CTRL_TYPE1_STATUS_COMMAND_REG reg: 0x%08x\n",
			dw_pcie_readl_dbi(pcie,
					  PCIE_CTRL_TYPE1_STATUS_COMMAND_REG));

		/* Enable errors */
		val = dw_pcie_readl_dbi(pcie, CAP_DEVICE_CONTROL_DEVICE_STATUS);
		val |=  CAP_CORR_ERR_REPORT_EN |
			CAP_NON_FATAL_ERR_REPORT_EN |
			CAP_FATAL_ERR_REPORT_EN |
			CAP_UNSUPPORT_REQ_REP_EN;
		dw_pcie_writel_dbi(pcie, CAP_DEVICE_CONTROL_DEVICE_STATUS, val);
	}

	val = dw_pcie_readl_dbi(pcie, PORT_GEN3_RELATED_OFF);
	val |= PCIE_EQ_PHASE_2_3;
	dw_pcie_writel_dbi(pcie, PORT_GEN3_RELATED_OFF, val);

	/* Disable writing dbi registers */
	dw_pcie_dbi_ro_wr_dis(pcie);

	s32gen1_pcie_enable_ltssm(pci);

	return 0;
}

static int init_pcie_phy(struct s32gen1_pcie *s32_pp)
{
	struct dw_pcie *pcie = &s32_pp->pcie;
	struct device *dev = pcie->dev;
	int ret;

	dev_dbg(pcie->dev, "%s\n", __func__);

	ret = phy_init(s32_pp->phy0);
	if (ret) {
		dev_err(dev, "Failed to init 'serdes_lane0' PHY\n");
		return ret;
	}

	ret = phy_set_mode_ext(s32_pp->phy0, PHY_MODE_PCIE, s32_pp->phy_mode);
	if (ret) {
		dev_err(dev, "Failed to set mode on 'serdes_lane0' PHY\n");
		return ret;
	}

	ret = phy_power_on(s32_pp->phy0);
	if (ret) {
		dev_err(dev, "Failed to power on 'serdes_lane0' PHY\n");
		return ret;
	}

	/*
	 *	It is safe to consider that a null phy1 means phy1 uninitialized
	 *	and non-zero initialized.
	 *	So if phy1 is non-zero then reuse the handler.
	 */
	if (!s32_pp->phy1)
		s32_pp->phy1 = devm_phy_optional_get(dev, "serdes_lane1");

	ret = phy_init(s32_pp->phy1);
	if (ret) {
		dev_err(dev, "Failed to init 'serdes_lane1' PHY\n");
		return ret;
	}

	ret = phy_set_mode_ext(s32_pp->phy1, PHY_MODE_PCIE, s32_pp->phy_mode);
	if (ret) {
		dev_err(dev, "Failed to set mode on 'serdes_lane1' PHY\n");
		return ret;
	}

	ret = phy_power_on(s32_pp->phy1);
	if (ret) {
		dev_err(dev, "Failed to power on 'serdes_lane1' PHY\n");
		return ret;
	}

	return 0;
}

static int deinit_pcie_phy(struct s32gen1_pcie *s32_pp)
{
	struct dw_pcie *pcie = &s32_pp->pcie;
	struct device *dev = pcie->dev;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	if (s32_pp->phy0) {
		ret = phy_power_off(s32_pp->phy0);
		if (ret) {
			dev_err(dev, "Failed to power off 'serdes_lane0' PHY\n");
			return ret;
		}

		ret = phy_exit(s32_pp->phy0);
		if (ret) {
			dev_err(dev, "Failed to exit 'serdes_lane0' PHY\n");
			return ret;
		}
	}

	if (s32_pp->phy1) {
		ret = phy_power_off(s32_pp->phy1);
		if (ret) {
			dev_err(dev, "Failed to power off 'serdes_lane1' PHY\n");
			return ret;
		}

		ret = phy_exit(s32_pp->phy1);
		if (ret) {
			dev_err(dev, "Failed to exit 'serdes_lane1' PHY\n");
			return ret;
		}
	}

	return 0;
}

static void s32gen1_pcie_pme_turnoff(struct s32gen1_pcie *s32_pp)
{
	/* TODO: We may want to transition to L2 low power state here, after
	 *	removal of power and clocks.
	 *	This needs more investigation (see SerDes manual, chapter
	 *	"L2 and L3 power down entry and exit conditions").
	 *	We're able to disable and power off the PHYs from this driver,
	 *	however clocks are controlled from the SerDes driver.
	 *
	 *	For now, just disable LTSSM.
	 */

	s32gen1_pcie_disable_ltssm(s32_pp);
}

static int deinit_controller(struct s32gen1_pcie *s32_pp)
{
	/* Other drivers assert controller reset, then disable phys,
	 *	then de-assert reset and disable clocks. On our platform
	 *	reset and clocks management is done in the SerDes driver,
	 *	so we need to investigate how and whether we should do
	 *	that here.
	 */

	return deinit_pcie_phy(s32_pp);
}

static int wait_phy_data_link(struct s32gen1_pcie *s32_pp)
{
	bool has_link;
	int ret = read_poll_timeout(has_data_phy_link, has_link, has_link,
			PCIE_LINK_WAIT_US, PCIE_LINK_TIMEOUT_US, 0, s32_pp);

	if (ret)
		dev_info(s32_pp->pcie.dev, "Failed to stabilize PHY link\n");

	return ret;
}

static void s32gen1_pcie_downstream_dev_to_D0(struct s32gen1_pcie *s32_pp)
{
	struct dw_pcie *pcie = &(s32_pp->pcie);
	struct pcie_port *pp = &(pcie->pp);
	struct pci_bus *root_bus = NULL;
	struct pci_dev *pdev;

	dev_dbg(pcie->dev, "%s\n", __func__);

	/*
	 * link doesn't go into L2 state with some of the endpoints
	 * if they are not in D0 state. So, we need to make sure that immediate
	 * downstream devices are in D0 state before sending PME_TurnOff to put
	 * link into L2 state.
	 */

	root_bus = s32gen1_get_child_downstream_bus(pp->bridge->bus);
	if (IS_ERR(root_bus)) {
		dev_err(pcie->dev, "Failed to find downstream devices\n");
		return;
	}

	list_for_each_entry(pdev, &root_bus->devices, bus_list) {
		if (PCI_SLOT(pdev->devfn) == 0) {
			if (pci_set_power_state(pdev, PCI_D0))
				dev_err(pcie->dev,
					"Failed to transition %s to D0 state\n",
					dev_name(&pdev->dev));
		}
	}
}

static int s32gen1_pcie_deinit_controller(struct s32gen1_pcie *s32_pp)
{
	struct dw_pcie *pcie = &(s32_pp->pcie);
	struct pcie_port *pp = &(pcie->pp);

	/* TODO: investigate if this is really necessary*/
	s32gen1_pcie_downstream_dev_to_D0(s32_pp);

	if (!s32_pp->is_endpoint)
		dw_pcie_host_deinit(pp);
	else {
		dw_pcie_ep_exit(&pcie->ep);
		s32gen1_del_pcie_ep_from_list(s32_pp->id);
	}

	s32gen1_pcie_pme_turnoff(s32_pp);

	return deinit_controller(s32_pp);
}

static int s32gen1_pcie_init_controller(struct s32gen1_pcie *s32_pp)
{
	struct dw_pcie *pcie = &(s32_pp->pcie);
	struct pcie_port *pp = &(pcie->pp);
	int ret = 0;

	s32gen1_pcie_disable_ltssm(s32_pp);

	ret = init_pcie_phy(s32_pp);
	if (ret)
		return ret;

	ret = init_pcie(s32_pp);
	if (ret)
		return ret;

	/* Only wait for link if RC */
	if (!s32_pp->is_endpoint) {
		ret = wait_phy_data_link(s32_pp);
		if (ret) {
			if (!phy_validate(s32_pp->phy0, PHY_MODE_PCIE, 0, NULL)) {
				dev_err(pcie->dev, "Failed to get link up with EP connected\n");
				return ret;
			}
		}
	}

	dev_info(pcie->dev, "Configuring as %s\n",
			PCIE_EP_RC_MODE(s32_pp->is_endpoint));

	if (s32_pp->has_msi_parent)
		pp->ops = &s32gen1_pcie_host_ops2;
	else
		pp->ops = &s32gen1_pcie_host_ops;

	return 0;
}

static int s32gen1_pcie_config_common(struct s32gen1_pcie *s32_pp,
					struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pcie = &(s32_pp->pcie);
	struct pcie_port *pp = &(pcie->pp);
	int ret = 0;

	dev_dbg(dev, "%s\n", __func__);

#ifdef CONFIG_PCI_S32GEN1_EP_MSI
	struct dw_pcie *pcie = &(s32_pp->pcie);
	u32 val, ctrl, num_ctrls;
	struct pcie_port *pp = &(pcie->pp);
#endif

	/* MSI configuration, for both RC and EP */
#ifdef CONFIG_PCI_MSI
	if ((!s32_pp->is_endpoint) && (!s32gen1_has_msi_parent(pp))) {
		ret = s32gen1_pcie_config_irq(&pp->msi_irq, "msi", pdev,
					      s32gen1_pcie_msi_handler, pp);
		if (ret) {
			dev_err(dev, "failed to request msi irq\n");
			return ret;
		}
	}
#endif

#ifdef CONFIG_PCI_S32GEN1_EP_MSI
	if (s32_pp->is_endpoint) {
		ret = s32gen1_pcie_config_irq(&(pcie->pp.msi_irq), "msi", pdev,
				s32gen1_pcie_msi_handler, &(pcie->pp));
		if (ret) {
			dev_err(dev, "failed to request msi irq\n");
			return ret;
		}

		pp->num_vectors = MSI_DEF_NUM_VECTORS;
		ret = dw_pcie_allocate_domains(pp);
		if (ret)
			dev_err(dev, "Unable to setup MSI domain for EP\n");

		if (pp->msi_irq)
			irq_set_chained_handler_and_data(pp->msi_irq,
							dw_ep_chained_msi_isr,
							pp);

		num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;

		/* Initialize IRQ Status array */
		for (ctrl = 0; ctrl < num_ctrls; ctrl++) {
			dw_pcie_writel_dbi(pcie, PCIE_MSI_INTR0_MASK +
					(ctrl * MSI_REG_CTRL_BLOCK_SIZE), ~0);
			dw_pcie_writel_dbi(pcie, PCIE_MSI_INTR0_ENABLE +
					(ctrl * MSI_REG_CTRL_BLOCK_SIZE), ~0);
			pcie->pp.irq_status[ctrl] = 0;
		}

		/* Setup interrupt pins */
		val = dw_pcie_readl_dbi(pcie, PCI_INTERRUPT_LINE);
		val &= 0xffff00ff;
		val |= 0x00000100;
		dw_pcie_writel_dbi(pcie, PCI_INTERRUPT_LINE, val);

		dw_pcie_msi_init(&pcie->pp);
	}
#endif /* CONFIG_PCI_S32GEN1_EP_MSI */

	pm_runtime_enable(dev);

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "Failed to get runtime sync for PCIe dev: %d\n",
			ret);
		goto fail_pm_get_sync;
	}

	ret = s32gen1_pcie_init_controller(s32_pp);
	if (ret)
		return ret;

	if (!s32_pp->is_endpoint) {
		ret = s32gen1_add_pcie_port(pp);
		if (ret)
			goto fail_host_init;

	} else {
		s32_pp->call_back = NULL;

#ifdef CONFIG_PCI_DW_DMA
		ret = s32gen1_pcie_config_irq(&s32_pp->dma_irq,
				"dma", pdev,
				s32gen1_pcie_dma_handler, s32_pp);
		if (ret) {
			dev_err(dev, "failed to request dma irq\n");
			goto fail_host_init;
		}
		dw_pcie_dma_clear_regs(&s32_pp->dma);
#endif /* CONFIG_PCI_DW_DMA */

		ret = s32gen1_add_pcie_ep(s32_pp);
		if (ret)
			goto fail_host_init;
		s32_config_user_space_data(&s32_pp->uinfo, pcie);
	}

	if (!s32_pp->is_endpoint) {
		ret = s32gen1_pcie_config_hp_irq(s32_pp, pdev);
		if (ret)
			goto fail_host_init;

		ret = s32gen1_enable_hotplug_cap(pcie);
		if (ret) {
			dev_err(dev, "Failed to enable hotplug capability\n");
			goto fail_host_init;
		}

		s32gen1_pcie_enable_hot_unplug_irq(s32_pp);
	}

	/* TODO: Init debugfs here */

	return ret;

fail_host_init:
	s32gen1_pcie_deinit_controller(s32_pp);
fail_pm_get_sync:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	return ret;
}

static int s32gen1_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s32gen1_pcie *s32_pp;
	struct dw_pcie *pcie;
	struct pcie_port *pp;
	struct phy *phy;
	int ret = 0;

	dev_dbg(dev, "%s\n", __func__);

	ret = s32gen1_check_serdes(dev);
	if (ret)
		return ret;

	phy = devm_phy_get(dev, "serdes_lane0");
	if (IS_ERR(phy)) {
		if (PTR_ERR(phy) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get 'serdes_lane0' PHY\n");

		return PTR_ERR(phy);
	}

	s32_pp = devm_kzalloc(dev, sizeof(*s32_pp), GFP_KERNEL);
	if (!s32_pp)
		return -ENOMEM;

	s32_pp->phy0 = phy;
	pcie = &(s32_pp->pcie);
	pp = &(pcie->pp);

	pcie->dev = dev;
	pcie->ops = &s32_pcie_ops;

	ret = s32gen1_pcie_dt_init(pdev, s32_pp);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, s32_pp);

	ret = s32gen1_pcie_config_common(s32_pp, pdev);
	if (ret) {
		dev_err(dev, "failed to set common PCIe settings\n");
		goto err;
	}

err:
	if (ret) {
		pm_runtime_put(dev);
		pm_runtime_disable(dev);
	}

	dw_pcie_dbi_ro_wr_dis(pcie);
	return ret;
}

#ifdef CONFIG_PM_SLEEP

static int s32gen1_pcie_suspend(struct device *dev)
{
	struct s32gen1_pcie *s32_pp = dev_get_drvdata(dev);
	struct dw_pcie *pcie = &(s32_pp->pcie);
	struct pcie_port *pp = &(pcie->pp);
	struct pci_bus *bus = pp->bridge->bus;
	struct pci_bus *root_bus;

	dev_dbg(pcie->dev, "%s\n", __func__);

	/* Save MSI interrupt vector */
	s32_pp->msi_ctrl_int = dw_pcie_readl_dbi(pcie,
					       PORT_MSI_CTRL_INT_0_EN_OFF);

	if (!s32_pp->is_endpoint) {
		/* Disable Hot-Unplug irq */
		s32gen1_pcie_disable_hot_unplug_irq(s32_pp);
		/* Disable Hot-Plug irq */
		s32gen1_pcie_disable_hot_plug_irq(pcie);

		s32gen1_pcie_downstream_dev_to_D0(s32_pp);

		root_bus = s32gen1_get_child_downstream_bus(bus);
		if (!IS_ERR(root_bus))
			pci_walk_bus(root_bus, pci_dev_set_disconnected, NULL);

		pci_stop_root_bus(pp->bridge->bus);
		pci_remove_root_bus(pp->bridge->bus);
	}

	s32gen1_pcie_pme_turnoff(s32_pp);

	return deinit_controller(s32_pp);
}

static int s32gen1_pcie_resume(struct device *dev)
{
	struct s32gen1_pcie *s32_pp = dev_get_drvdata(dev);
	struct dw_pcie *pcie = &(s32_pp->pcie);
	struct pcie_port *pp = &(pcie->pp);
	int ret = 0;

	dev_dbg(pcie->dev, "%s\n", __func__);

	/* TODO: we should keep link state (bool) in struct s32gen1_pcie
	 *	and not do anything on resume if there was no link.
	 *	Otherwise resuming is slow since it will get link timeouts.
	 */

	ret = s32gen1_pcie_init_controller(s32_pp);
	if (ret < 0)
		return ret;

	if (!s32_pp->is_endpoint) {
		ret = s32gen1_pcie_host_init(pp);
		if (ret < 0) {
			dev_err(dev, "Failed to init host: %d\n", ret);
			goto fail_host_init;
		}

		ret = pci_host_probe(pp->bridge);
		if (ret)
			return ret;
	}

	/* Restore MSI interrupt vector */
	dw_pcie_writel_dbi(pcie, PORT_MSI_CTRL_INT_0_EN_OFF,
			   s32_pp->msi_ctrl_int);

	if (!s32_pp->is_endpoint) {
		/* Enable Hot-Plug capability */
		ret = s32gen1_enable_hotplug_cap(pcie);
		if (ret) {
			dev_err(dev, "Failed to enable hotplug capability\n");
			goto fail_host_init;
		}

		/* Enable Hot-Unplug irq */
		s32gen1_pcie_enable_hot_unplug_irq(s32_pp);
	}

	return 0;

fail_host_init:
	return deinit_controller(s32_pp);
}

#endif

static const struct dev_pm_ops s32gen1_pcie_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(s32gen1_pcie_suspend,
					s32gen1_pcie_resume)
};

static const struct s32gen1_pcie_data rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
};

static const struct s32gen1_pcie_data ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
};

static const struct of_device_id s32gen1_pcie_of_match[] = {
	{ .compatible = "nxp,s32cc-pcie", .data = &rc_of_data },
	{ .compatible = "nxp,s32cc-pcie-ep", .data = &ep_of_data },
	{},
};
MODULE_DEVICE_TABLE(of, s32gen1_pcie_of_match);

static struct platform_driver s32gen1_pcie_driver = {
	.driver = {
		.name	= "s32gen1-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = s32gen1_pcie_of_match,
		.pm = &s32gen1_pcie_pm_ops,
	},
	.probe = s32gen1_pcie_probe,
	.shutdown = s32gen1_pcie_shutdown,
};

module_platform_driver(s32gen1_pcie_driver);

MODULE_AUTHOR("Ionut Vicovan <Ionut.Vicovan@nxp.com>");
MODULE_DESCRIPTION("NXP S32Gen1 PCIe host controller driver");
MODULE_LICENSE("GPL v2");
