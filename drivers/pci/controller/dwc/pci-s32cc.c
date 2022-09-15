// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for NXP S32CC SoCs
 *
 * Copyright 2019-2020 NXP
 */

#ifdef CONFIG_PCI_S32CC_DEBUG
#ifndef DEBUG
#define DEBUG
#endif
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

#ifdef CONFIG_PCI_S32CC_ACCESS_FROM_USER
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/sched/signal.h>
#include <linux/uaccess.h>
#endif

#include "pci-s32cc-regs.h"
#include "pci-s32cc.h"
#include "../../pci.h"

#ifdef CONFIG_PCI_DW_DMA
#include <linux/dma-mapping.h>
#include "pci-dma-s32cc.h"
#endif

#ifdef DEBUG
#ifdef pr_debug
#undef pr_debug
#define pr_debug pr_info
#endif
#ifdef dev_dbg
#undef dev_dbg
#define dev_dbg dev_info
#endif
#endif /* CONFIG_PCI_S32CC_DEBUG */

#define DEBUG_FUNC pr_debug("%s\n", __func__)

#ifdef DEBUG_WR
#define pr_debug_rw pr_debug
#define dev_dbg_rw dev_dbg
#else
#define pr_debug_rw(fmt, ...)
#define dev_dbg_rw(fmt, ...)
#endif

#define PCIE_LINKUP_MASK	(PCIE_SS_SMLH_LINK_UP | PCIE_SS_RDLH_LINK_UP | \
			PCIE_SS_SMLH_LTSSM_STATE)
#define PCIE_LINKUP_EXPECT	(PCIE_SS_SMLH_LINK_UP | PCIE_SS_RDLH_LINK_UP | \
			PCIE_SS_SMLH_LTSSM_STATE_VALUE(LTSSM_STATE_L0))

/* Default timeout (ms) */
#define PCIE_CX_CPL_BASE_TIMER_VALUE	100

#define PCIE_EP_RC_MODE(ep_mode) ((ep_mode) ? "EndPoint" : "RootComplex")

#define PCI_BASE_CLASS_OFF	24
#define PCI_SUBCLASS_OTHER	(0x80)
#define PCI_SUBCLASS_OFF	16

#define PCIE_NUM_BARS	6
#define PCIE_EP_DEFAULT_BAR_SIZE	SZ_1M

#ifdef CONFIG_PCI_S32CC_INIT_EP_BARS

#ifndef CONFIG_SYS_PCI_EP_MEMORY_BASE
/* Use the reserved memory from device tree
 * TODO: read it dynamically via fdt api
 * TODO: only one PCIe controller can be used in EP mode with
 * this enabled
 */
#define CONFIG_SYS_PCI_EP_MEMORY_BASE 0xc0000000
#endif /* CONFIG_SYS_PCI_EP_MEMORY_BASE */

/* EP BARs */
#define PCI_BASE_ADDRESS_MEM_NON_PREFETCH	0x00	/* non-prefetchable */

#define PCIE_EP_BAR0_ADDR		CONFIG_SYS_PCI_EP_MEMORY_BASE
#define PCIE_EP_BAR0_SIZE		SZ_1M
#define PCIE_EP_BAR1_ADDR		(PCIE_EP_BAR0_ADDR + PCIE_EP_BAR0_SIZE)
#define PCIE_EP_BAR1_SIZE		0
#define PCIE_EP_BAR2_ADDR		(PCIE_EP_BAR1_ADDR + PCIE_EP_BAR1_SIZE)
#define PCIE_EP_BAR2_SIZE		SZ_1M
#define PCIE_EP_BAR3_ADDR		(PCIE_EP_BAR2_ADDR + PCIE_EP_BAR2_SIZE)
#define PCIE_EP_BAR3_SIZE		0
#define PCIE_EP_BAR4_ADDR		(PCIE_EP_BAR3_ADDR + PCIE_EP_BAR3_SIZE)
#define PCIE_EP_BAR4_SIZE		0
#define PCIE_EP_BAR5_ADDR		(PCIE_EP_BAR4_ADDR + PCIE_EP_BAR4_SIZE)
#define PCIE_EP_BAR5_SIZE		0
#define PCIE_EP_BAR0_EN_DIS		1
#define PCIE_EP_BAR1_EN_DIS		0
#define PCIE_EP_BAR2_EN_DIS		1
#define PCIE_EP_BAR3_EN_DIS		1
#define PCIE_EP_BAR4_EN_DIS		1
#define PCIE_EP_BAR5_EN_DIS		0
#define PCIE_EP_BAR0_INIT	(PCI_BASE_ADDRESS_SPACE_MEMORY | \
			PCI_BASE_ADDRESS_MEM_TYPE_32 | \
			PCI_BASE_ADDRESS_MEM_NON_PREFETCH)
#define PCIE_EP_BAR1_INIT	(PCI_BASE_ADDRESS_SPACE_MEMORY | \
			PCI_BASE_ADDRESS_MEM_TYPE_32 | \
			PCI_BASE_ADDRESS_MEM_NON_PREFETCH)
#define PCIE_EP_BAR2_INIT	(PCI_BASE_ADDRESS_SPACE_MEMORY | \
			PCI_BASE_ADDRESS_MEM_TYPE_32 | \
			PCI_BASE_ADDRESS_MEM_NON_PREFETCH)
#define PCIE_EP_BAR3_INIT	(PCI_BASE_ADDRESS_SPACE_MEMORY | \
			PCI_BASE_ADDRESS_MEM_TYPE_32 | \
			PCI_BASE_ADDRESS_MEM_NON_PREFETCH)
#define PCIE_EP_BAR4_INIT	(PCI_BASE_ADDRESS_SPACE_MEMORY | \
			PCI_BASE_ADDRESS_MEM_TYPE_32 | \
			PCI_BASE_ADDRESS_MEM_NON_PREFETCH)
#define PCIE_EP_BAR5_INIT	0

#define PCIE_EP_BAR_INIT(bar_no) \
		{PCIE_EP_BAR ## bar_no ## _ADDR, \
			NULL, \
			PCIE_EP_BAR ## bar_no ## _SIZE, \
			BAR_ ## bar_no, \
			PCIE_EP_BAR ## bar_no ## _INIT}

static struct pci_epf_bar s32cc_ep_bars[] = {
		PCIE_EP_BAR_INIT(0),
		PCIE_EP_BAR_INIT(1),
		PCIE_EP_BAR_INIT(2),
		PCIE_EP_BAR_INIT(3),
		PCIE_EP_BAR_INIT(4),
		PCIE_EP_BAR_INIT(5)
};

static int s32cc_ep_bars_en[] = {
		PCIE_EP_BAR0_EN_DIS,
		PCIE_EP_BAR1_EN_DIS,
		PCIE_EP_BAR2_EN_DIS,
		PCIE_EP_BAR3_EN_DIS,
		PCIE_EP_BAR4_EN_DIS,
		PCIE_EP_BAR5_EN_DIS
};

#endif /* CONFIG_PCI_S32CC_INIT_EP_BARS */
/* End EP BARs defines */

#define xstr(s) str(s)
#define str(s) #s

#define clrbits(type, addr, clear) \
	write ## type(read ## type(addr) & ~(clear), (addr))

#define setbits(type, addr, set) \
	write ## type(read ## type(addr) | (set), (addr))

#define clrsetbits(type, addr, clear, set) \
	write ## type((read ## type(addr) & ~(clear)) | (set), (addr))

#define BCLR16(pci, base, reg, mask) \
do { \
	pr_debug_rw("%s: BCLR16(" str(base) "+0x%x, 0x%x)\n", __func__, \
		(u32)(reg), (u16)(mask)); \
	clrbits(w, (pci)->base ## _base + reg, (u16)mask); \
} while (0)

#define BSET16(pci, base, reg, mask) \
do { \
	pr_debug_rw("%s: BSET16(" str(base) "+0x%x, 0x%x)\n", __func__, \
		(u32)(reg), (u16)(mask)); \
	setbits(w, (pci)->base ## _base + reg, (u16)mask); \
} while (0)

#define BCLRSET16(pci, base, reg, write_data, mask) \
do { \
	pr_debug_rw("%s: BCLRSET16(" str(base) "+0x%x, 0x%x, mask 0x%x)\n", __func__, \
		(u32)(reg), (u16)(write_data), \
		(u16)(mask)); \
	clrsetbits(w, (pci)->base ## _base + reg, (u16)write_data, (u16)mask); \
} while (0)

#define BCLR32(pci, base, reg, mask) \
do { \
	pr_debug_rw("%s: BCLR32(" str(base) "+0x%x, 0x%x)\n", __func__, \
		(u32)(reg), (u32)(mask)); \
	clrbits(l, (pci)->base ## _base + reg, mask); \
} while (0)

#define BSET32(pci, base, reg, mask) \
do { \
	pr_debug_rw("%s: BSET32(" str(base) "+0x%x, 0x%x)\n", __func__, \
		(u32)(reg), (u32)(mask)); \
	setbits(l, (pci)->base ## _base + reg, mask); \
} while (0)

#define BCLRSET32(pci, base, reg, write_data, mask) \
do { \
	pr_debug_rw("%s: BCLRSET32(" str(base) "+0x%llx, 0x%x, mask 0x%x)\n", __func__, \
		(u32)(reg), (u32)(write_data), \
		(u32)(mask)); \
	clrsetbits(l, (pci)->base ## _base + reg, write_data, mask); \
} while (0)

static inline void s32cc_pcie_write(struct dw_pcie *pci,
		void __iomem *base, u32 reg, size_t size, u32 val)
{
	int ret;

	struct s32cc_pcie *s32cc_pci = to_s32cc_from_dw_pcie(pci);

	ret = dw_pcie_write(base + reg, size, val);
	if (ret)
		dev_err(pci->dev, "(pcie%d): Write DBI address failed\n",
				s32cc_pci->id);
}

void dw_pcie_writel_ctrl(struct s32cc_pcie *pci, u32 reg, u32 val)
{
	s32cc_pcie_write(&pci->pcie, pci->ctrl_base, reg, 0x4, val);
}

u32 dw_pcie_readl_ctrl(struct s32cc_pcie *pci, u32 reg)
{
	u32 val = 0;

	if (dw_pcie_read(pci->ctrl_base + reg, 0x4, &val))
		dev_err(pci->pcie.dev, "Read ctrl address failed\n");

	return val;
}

static struct s32cc_pcie *s32cc_pcie_ep;

#ifdef CONFIG_PCI_DW_DMA
static irqreturn_t s32cc_pcie_dma_handler(int irq, void *arg)
{
	struct s32cc_pcie *s32cc_pp = arg;
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct dma_info *di = &s32cc_pp->dma;

	u32 val_write = 0;
	u32 val_read = 0;

	val_write = dw_pcie_readl_dbi(pcie, PCIE_DMA_WRITE_INT_STATUS);
	val_read = dw_pcie_readl_dbi(pcie, PCIE_DMA_READ_INT_STATUS);

	if (val_write) {
#ifdef CONFIG_PCI_S32CC_ACCESS_FROM_USER
		bool signal = (di->wr_ch.status == DMA_CH_RUNNING);
#endif
		dw_handle_dma_irq_write(pcie, di, val_write);
#ifdef CONFIG_PCI_S32CC_ACCESS_FROM_USER
		if (signal && s32cc_pp->uspace.send_signal_to_user)
			s32cc_pp->uspace.send_signal_to_user(s32cc_pp);
#endif

		if (s32cc_pp->call_back)
			s32cc_pp->call_back(val_write);
	}
	if (val_read) {
#ifdef CONFIG_PCI_S32CC_ACCESS_FROM_USER
		bool signal = (di->rd_ch.status == DMA_CH_RUNNING);
#endif
		dw_handle_dma_irq_read(pcie, di, val_read);
#ifdef CONFIG_PCI_S32CC_ACCESS_FROM_USER
		if (signal && s32cc_pp->uspace.send_signal_to_user)
			s32cc_pp->uspace.send_signal_to_user(s32cc_pp);
#endif
	}

	return IRQ_HANDLED;
}
#endif /* CONFIG_PCI_DW_DMA */

#ifdef CONFIG_PCI_S32CC_ACCESS_FROM_USER
ssize_t s32cc_ioctl(struct file *filp, u32 cmd,
		unsigned long data);

static const struct file_operations s32v_pcie_ep_dbgfs_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.unlocked_ioctl = s32cc_ioctl,
};
#endif /* CONFIG_PCI_S32CC_ACCESS_FROM_USER */

#ifdef CONFIG_PCI_S32CC_EP_MSI
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

static void s32cc_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pcie = to_dw_pcie_from_ep(ep);
	int bar;
#ifdef CONFIG_PCI_S32CC_INIT_EP_BARS
	struct pci_epc *epc = ep->epc;
	int ret = 0;
#endif
#ifdef CONFIG_PCI_S32CC_EP_MSI
	u32 val, ctrl, num_ctrls;
	struct pcie_port *pp = &pcie->pp;
#endif

	DEBUG_FUNC;

	if (!ep) {
		pr_err("%s: No S32CC EP configuration found\n", __func__);
		return;
	}

	pcie->iatu_unroll_enabled = dw_pcie_iatu_unroll_enabled(pcie);
	dev_dbg(pcie->dev, "iATU unroll: %s\n",
		pcie->iatu_unroll_enabled ? "enabled" : "disabled");

	dw_pcie_dbi_ro_wr_en(pcie);
	dw_pcie_setup(pcie);

	/*
	 * Configure the class and revision for the EP device,
	 * to enable human friendly enumeration by the RC (e.g. by lspci)
	 */
	BSET32(pcie, dbi, PCI_CLASS_REVISION,
			((PCI_BASE_CLASS_PROCESSOR << PCI_BASE_CLASS_OFF) |
			(PCI_SUBCLASS_OTHER << PCI_SUBCLASS_OFF)));

#ifdef CONFIG_PCI_S32CC_EP_MSI
	pp->num_vectors = MSI_DEF_NUM_VECTORS;
	ret = dw_pcie_allocate_domains(pp);
	if (ret)
		dev_err(pcie->dev, "Unable to setup MSI domain for EP\n");

	if (pp->msi_irq)
		irq_set_chained_handler_and_data(pp->msi_irq,
				dw_ep_chained_msi_isr, pp);

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
#else
	pr_debug("%s: Enable MSI/MSI-X capabilities\n", __func__);

	/* Enable MSIs by setting the capability bit */
	BSET32(pcie, dbi, PCI_MSI_CAP, MSI_EN);

	/* Enable MSI-Xs by setting the capability bit */
	BSET32(pcie, dbi, PCI_MSIX_CAP, MSIX_EN);
#endif /* CONFIG_PCI_S32CC_EP_MSI */

	dw_pcie_dbi_ro_wr_dis(pcie);

#ifdef CONFIG_PCI_S32CC_INIT_EP_BARS
	/* Setup BARs and inbound regions */
	for (bar = BAR_0; (bar < PCIE_NUM_BARS); bar++) {
		if (s32cc_ep_bars_en[bar]) {
			ret = epc->ops->set_bar(epc, 0, 0,
					&s32cc_ep_bars[bar]);
			if (ret)
				pr_err("%s: Unable to init BAR%d\n",
						 __func__, bar);
		}
	}
#else
	for (bar = BAR_0; bar <= BAR_5; bar++)
		dw_pcie_ep_reset_bar(pcie, bar);
#endif

	dw_pcie_dbi_ro_wr_en(pcie);

	/* CMD reg:I/O space, MEM space, and Bus Master Enable */
	BSET32(pcie, dbi, PCI_COMMAND,
			PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

	dw_pcie_dbi_ro_wr_dis(pcie);
}

/* Only for EP */
int s32cc_pcie_setup_outbound(struct s32cc_outbound_region *ptr_outb)
{
	int ret = 0;
	struct pci_epc *epc;

	DEBUG_FUNC;

	if (!s32cc_pcie_ep) {
		pr_err("%s: No S32CC EP configuration found\n", __func__);
		return -ENODEV;
	}

	epc = s32cc_pcie_ep->pcie.ep.epc;

	if (!epc || !epc->ops) {
		pr_err("Invalid S32CC EP configuration\n");
		return -ENODEV;
	}

	if (!ptr_outb)
		return -EINVAL;

	/* Setup outbound region */
	ret = epc->ops->map_addr(epc, 0, 0, ptr_outb->base_addr,
			ptr_outb->target_addr, ptr_outb->size);

	return ret;
}
EXPORT_SYMBOL(s32cc_pcie_setup_outbound);

/* Only for EP */
int s32cc_pcie_setup_inbound(struct s32cc_inbound_region *inb_str)
{
	int ret = 0;
	struct pci_epc *epc;
	int bar_num;
#ifndef CONFIG_PCI_S32CC_INIT_EP_BARS
	struct pci_epf_bar bar = {
		.size = PCIE_EP_DEFAULT_BAR_SIZE
	};
#endif

	DEBUG_FUNC;

	if (!s32cc_pcie_ep) {
		pr_err("%s: No S32CC EP configuration found\n", __func__);
		return -ENODEV;
	}

	epc = s32cc_pcie_ep->pcie.ep.epc;

	if (!epc || !epc->ops) {
		pr_err("Invalid S32CC EP configuration\n");
		return -ENODEV;
	}

	if (!inb_str)
		return -EINVAL;

	/* Setup inbound region */
	bar_num = inb_str->bar_nr;
	if (bar_num >= PCIE_NUM_BARS) {
		pr_err("Invalid BAR number (%d)\n", bar_num);
		return -EINVAL;
	}
#ifdef CONFIG_PCI_S32CC_INIT_EP_BARS
	/* Reconfigure existing BAR */
	s32cc_ep_bars[bar_num].phys_addr = inb_str->target_addr;
	ret = epc->ops->set_bar(epc, 0, 0, &s32cc_ep_bars[bar_num]);
#else
	bar.barno = bar_num;
	bar.phys_addr = inb_str->target_addr;
	ret = epc->ops->set_bar(epc, 0, 0, &bar);
#endif

	return ret;
}
EXPORT_SYMBOL(s32cc_pcie_setup_inbound);

static bool s32cc_pcie_is_hw_mode_ep(struct dw_pcie *pci)
{
	u8 header_type;

	header_type = dw_pcie_readb_dbi(pci, PCI_HEADER_TYPE);
	return ((header_type & 0x7f) == PCI_HEADER_TYPE_NORMAL);
}

static void s32cc_pcie_disable_ltssm(struct s32cc_pcie *pci)
{
	dw_pcie_dbi_ro_wr_en(&pci->pcie);
	BCLR32(pci, ctrl, PE0_GEN_CTRL_3, LTSSM_EN);
	dw_pcie_dbi_ro_wr_dis(&pci->pcie);
}

static void s32cc_pcie_enable_ltssm(struct s32cc_pcie *pci)
{
	dw_pcie_dbi_ro_wr_en(&pci->pcie);
	BSET32(pci, ctrl, PE0_GEN_CTRL_3, LTSSM_EN);
	dw_pcie_dbi_ro_wr_dis(&pci->pcie);
}

static bool is_s32cc_pcie_ltssm_enabled(struct s32cc_pcie *pci)
{
	return (dw_pcie_readl_ctrl(pci, PE0_GEN_CTRL_3) & LTSSM_EN);
}

static int s32cc_pcie_link_is_up(struct dw_pcie *pcie)
{
	struct s32cc_pcie *s32cc_pp = to_s32cc_from_dw_pcie(pcie);

	if (!is_s32cc_pcie_ltssm_enabled(s32cc_pp))
		return 0;

	return ((dw_pcie_readl_ctrl(s32cc_pp, PCIE_SS_PE0_LINK_DBG_2) &
				(PCIE_LINKUP_MASK)) ==  ((u32)(PCIE_LINKUP_EXPECT)));
}

static int s32cc_pcie_get_link_speed(struct s32cc_pcie *s32cc_pp)
{
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	u32 link_sta = dw_pcie_readw_dbi(pcie, PCI_CAP_ID_EXP + PCI_EXP_LNKSTA);

	pr_debug("PCIe%d: Speed Gen%d\n", s32cc_pp->id,
		 link_sta & PCI_EXP_LNKSTA_CLS);

	/* return link speed based on negotiated link status */
	return link_sta & PCI_EXP_LNKSTA_CLS;
}

static int s32cc_pcie_start_link(struct dw_pcie *pcie)
{
	struct s32cc_pcie *s32cc_pp = to_s32cc_from_dw_pcie(pcie);
	u32 tmp;
	int ret = 0, count;
	int link_speed = -1;

	DEBUG_FUNC;

	dw_pcie_dbi_ro_wr_en(pcie);

	if (!dw_pcie_link_up(pcie)) {
		ret = dw_pcie_wait_for_link(pcie);
		goto out;
	}

	link_speed = s32cc_pcie_get_link_speed(s32cc_pp);

	/* Gen 1 devices work with u-boot link;
	 * Gen2 or higher (e.g. NVMe) need re-ajustments
	 */
	if (link_speed > GEN1) {
		/* Try to (re)establish the link, starting with Gen1 */
		s32cc_pcie_disable_ltssm(s32cc_pp);

		BCLRSET16(pcie, dbi, PCI_CAP_ID_EXP + PCI_EXP_LNKCAP,
			  PCI_EXP_LNKCAP_SLS_2_5GB, PCI_EXP_LNKCAP_SLS);

		/* Start LTSSM. */
		s32cc_pcie_enable_ltssm(s32cc_pp);
		ret = dw_pcie_wait_for_link(pcie);

		if (ret)
			goto out;

		/* Allow Gen2 or Gen3 mode after the link is up. */
		BCLRSET16(pcie, dbi, PCI_EXP_CAP_ID + PCI_EXP_LNKCAP,
			  s32cc_pp->linkspeed, PCI_EXP_LNKCAP_SLS);

		/*
		 * Start Directed Speed Change so the best possible speed both
		 * link partners support can be negotiated.
		 * The manual says:
		 * When you set the default of the Directed Speed Change field
		 * of the Link Width and Speed Change Control register
		 * (GEN2_CTRL_OFF.DIRECT_SPEED_CHANGE) using the
		 * DEFAULT_GEN2_SPEED_CHANGE configuration parameter to 1, then
		 * the speed change is initiated automatically after link up,
		 * and the controller clears the contents of
		 * GEN2_CTRL_OFF.DIRECT_SPEED_CHANGE.
		 */
		BSET32(pcie, dbi, PCIE_LINK_WIDTH_SPEED_CONTROL,
		       PORT_LOGIC_SPEED_CHANGE);

		count = 1000;
		while (count--) {
			tmp = dw_pcie_readl_dbi(pcie,
				PCIE_LINK_WIDTH_SPEED_CONTROL);
			/* Test if the speed change finished. */
			if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
				break;
			usleep_range(100, 1000);
		}

		/* Make sure link training is finished as well! */
		if (count) {
			ret = dw_pcie_wait_for_link(pcie);
		} else {
			dev_err(pcie->dev, "Speed change timeout\n");
			ret = -EINVAL;
		}
	}

out:
	if (!ret) {
		dev_info(pcie->dev, "Link up, Gen=%d\n",
				s32cc_pcie_get_link_speed(s32cc_pp));
	}

	dw_pcie_dbi_ro_wr_dis(pcie);
	return ret;
}

static void s32cc_pcie_stop_link(struct dw_pcie *pcie)
{
	struct s32cc_pcie *s32cc_pp = to_s32cc_from_dw_pcie(pcie);

	s32cc_pcie_disable_ltssm(s32cc_pp);
}

#ifdef CONFIG_PCI_MSI
/* msi IRQ handler
 * irq - interrupt number
 * arg - pointer to the "struct pcie_port" object
 */
static irqreturn_t s32cc_pcie_msi_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;
#ifdef DEBUG
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct s32cc_pcie *s32cc_pci = to_s32cc_from_dw_pcie(pcie);

	pr_debug("%s(pcie%d)\n", __func__, s32cc_pci->id);
#endif

	return dw_handle_msi_irq(pp);
}
#endif

static int s32cc_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);

	DEBUG_FUNC;

	dw_pcie_setup_rc(pp);

	s32cc_pcie_start_link(pcie);
	dw_pcie_wait_for_link(pcie);

#ifdef CONFIG_PCI_MSI
	dw_pcie_msi_init(pp);
#endif

	return 0;
}

static struct dw_pcie_ops s32cc_pcie_ops = {
	.link_up = s32cc_pcie_link_is_up,
	.start_link = s32cc_pcie_start_link,
	.stop_link = s32cc_pcie_stop_link,
	.write_dbi = s32cc_pcie_write,
};

static struct dw_pcie_host_ops s32cc_pcie_host_ops = {
	.host_init = s32cc_pcie_host_init,
};

#define MAX_IRQ_NAME_SIZE 32
static int s32cc_pcie_config_irq(int *irq_id, char *irq_name,
		struct platform_device *pdev,
		irq_handler_t irq_handler, void *irq_arg)
{
	int ret = 0;
	char irq_display_name[MAX_IRQ_NAME_SIZE];

	DEBUG_FUNC;

	*(irq_id) = platform_get_irq_byname(pdev, irq_name);
	if (*(irq_id) <= 0) {
		dev_err(&pdev->dev, "failed to get %s irq\n", irq_name);
		return -ENODEV;
	}
	snprintf(irq_display_name, MAX_IRQ_NAME_SIZE, "s32cc-pcie-%s",
			irq_name);
	ret = devm_request_irq(&pdev->dev, *(irq_id), irq_handler,
			IRQF_SHARED,  irq_name, irq_arg);
	if (ret) {
		dev_info(&pdev->dev, "Register interrupt %d (%s) failed (%d)\n",
				*irq_id, irq_name, ret);
		return ret;
	}

	dev_info(&pdev->dev, "Allocated line %d for interrupt %d (%s)",
			ret, *irq_id, irq_name);

	return 0;
}

static int __init s32cc_add_pcie_port(struct pcie_port *pp,
			struct platform_device *pdev)
{
	int ret;

	DEBUG_FUNC;

#ifdef CONFIG_PCI_MSI
	ret = s32cc_pcie_config_irq(&pp->msi_irq, "msi", pdev,
			s32cc_pcie_msi_handler, pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request msi irq\n");
		return ret;
	}
#endif

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int s32cc_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				 enum pci_epc_irq_type type, u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	DEBUG_FUNC;

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
	}

	return 0;
}

static struct dw_pcie_ep_ops pcie_ep_ops = {
	.ep_init = s32cc_pcie_ep_init,
	.raise_irq = s32cc_pcie_ep_raise_irq,
};

static int __init s32cc_add_pcie_ep(struct s32cc_pcie *s32cc_pp,
				     struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct dw_pcie_ep *ep = &pcie->ep;
	struct resource *res;

	DEBUG_FUNC;

#ifdef CONFIG_PCI_S32CC_EP_MSI
	ret = s32cc_pcie_config_irq(&pcie->pp.msi_irq, "msi", pdev,
			s32cc_pcie_msi_handler, &pcie->pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request msi irq\n");
		return ret;
	}
#endif

	ep->ops = &pcie_ep_ops;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res)
		return -EINVAL;

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "failed to initialize endpoint\n");
		return ret;
	}

	return 0;
}

static void s32cc_pcie_shutdown(struct platform_device *pdev)
{
	struct s32cc_pcie *s32cc_pp = platform_get_drvdata(pdev);

	DEBUG_FUNC;

	if (!s32cc_pp->is_endpoint) {
		/* bring down link, so bootloader gets clean state
		 * in case of reboot
		 */
		s32cc_pcie_stop_link(&s32cc_pp->pcie);

		pm_runtime_put_sync(&pdev->dev);
		pm_runtime_disable(&pdev->dev);

		mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
	}
}

struct s32cc_pcie *s32cc_get_dw_pcie(void)
{
	return s32cc_pcie_ep;
}
EXPORT_SYMBOL(s32cc_get_dw_pcie);

static int s32cc_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct s32cc_pcie *s32cc_pp;
	struct resource *res;
	struct dw_pcie *pcie;
	struct pcie_port *pp;

	int ret = 0;
	unsigned int ltssm_en;

	DEBUG_FUNC;

	s32cc_pp = devm_kzalloc(dev, sizeof(*s32cc_pp), GFP_KERNEL);
	if (!s32cc_pp)
		return -ENOMEM;

	pcie = &s32cc_pp->pcie;
	pp = &pcie->pp;
#ifdef CONFIG_PCI_MSI
	pp->num_vectors = MAX_MSI_IRQS;
#endif

	pcie->dev = dev;
	pcie->ops = &s32cc_pcie_ops;

	of_property_read_u32(np, "id", &s32cc_pp->id);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pcie->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->dbi_base))
		return PTR_ERR(pcie->dbi_base);
	dev_dbg(dev, "dbi: %pR\n", res);
	dev_dbg(dev, "dbi virt: 0x%p\n", pcie->dbi_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi2");
	pcie->dbi_base2 = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->dbi_base2))
		return PTR_ERR(pcie->dbi_base2);
	dev_dbg(dev, "dbi2: %pR\n", res);
	dev_dbg(dev, "dbi2 virt: 0x%p\n", pcie->dbi_base2);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "atu");
	dev_dbg(dev, "atu: %pR\n", res);
	pcie->atu_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->atu_base))
		return PTR_ERR(pcie->atu_base);
	dev_dbg(dev, "atu virt: 0x%p\n", pcie->atu_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctrl");
	s32cc_pp->ctrl_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(s32cc_pp->ctrl_base))
		return PTR_ERR(s32cc_pp->ctrl_base);
	dev_dbg(dev, "ctrl: %pR\n", res);
	dev_dbg(dev, "ctrl virt: 0x%p\n", s32cc_pp->ctrl_base);

	s32cc_pp->is_endpoint = s32cc_pcie_is_hw_mode_ep(pcie);
	dev_dbg(dev, "Configured as %s\n",
			PCIE_EP_RC_MODE(s32cc_pp->is_endpoint));

	/* Attempt to figure out whether u-boot has preconfigured PCIE; if it
	 * did not, we will not be able to tell whether we should run as EP
	 * (whose configuration value is the same as the reset value) or RC.
	 * Failing to do so might result in a hardware freeze, if u-boot was
	 * compiled without PCIE support at all.
	 *
	 * Test the LTSSM_ENABLE bit, whose reset value
	 * is different from the value set by u-boot.
	 */
	ltssm_en = is_s32cc_pcie_ltssm_enabled(s32cc_pp);
	if (!ltssm_en) {
		dev_err(dev,
			"u-boot did not initialize PCIE PHY; "
			"is u-boot compiled with PCIE support?\n");
		ret = -ENODEV;
		goto err_cfg;
	}

	platform_set_drvdata(pdev, s32cc_pp);
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err;
	}

	s32cc_pp->linkspeed = of_pci_get_max_link_speed(np);
	if (s32cc_pp->linkspeed < GEN1 || s32cc_pp->linkspeed > GEN3) {
		dev_warn(dev, "Invalid PCIe speed; setting to GEN1\n");
		s32cc_pp->linkspeed = GEN1;
	}

	dev_info(dev, "Configuring as %s\n",
			PCIE_EP_RC_MODE(s32cc_pp->is_endpoint));

	pp->ops = &s32cc_pcie_host_ops;

	if (!s32cc_pp->is_endpoint) {
		ret = s32cc_add_pcie_port(pp, pdev);
		if (ret < 0)
			goto err;
	} else {
#ifdef CONFIG_PCI_S32CC_ACCESS_FROM_USER
		struct dentry *pfile;
#endif

		s32cc_pcie_ep = s32cc_pp;
		s32cc_pp->call_back = NULL;

#ifdef CONFIG_PCI_DW_DMA
		ret = s32cc_pcie_config_irq(&s32cc_pp->dma_irq,
				"dma", pdev,
				s32cc_pcie_dma_handler, s32cc_pp);
		if (ret) {
			dev_err(dev, "failed to request dma irq\n");
			goto err;
		}
		dw_pcie_dma_clear_regs(pcie, &s32cc_pp->dma);
#endif /* CONFIG_PCI_DW_DMA */

#ifdef CONFIG_PCI_S32CC_ACCESS_FROM_USER
		s32cc_pp->uspace.user_pid = 0;
		memset(&s32cc_pp->uspace.info, 0, sizeof(struct siginfo));
		s32cc_pp->uspace.info.si_signo = SIGUSR1;
		s32cc_pp->uspace.info.si_code = SI_USER;
		s32cc_pp->uspace.info.si_int = 0;

		s32cc_pp->dir = debugfs_create_dir("ep_dbgfs", NULL);
		if (!s32cc_pp->dir)
			dev_info(dev, "Creating debugfs dir failed\n");
		pfile = debugfs_create_file("ep_file", 0444, s32cc_pp->dir,
			(void *)s32cc_pp, &s32v_pcie_ep_dbgfs_fops);
		if (!pfile)
			dev_info(dev, "Creating debugfs failed\n");
#endif /* CONFIG_PCI_S32CC_ACCESS_FROM_USER */

		s32cc_add_pcie_ep(s32cc_pp, pdev);
	}

err:
	if (ret)
		pm_runtime_disable(dev);

err_cfg:
	dw_pcie_dbi_ro_wr_dis(pcie);
	return ret;
}

static const struct of_device_id s32cc_pcie_of_match[] = {
	{ .compatible = "nxp,s32cc-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, s32cc_pcie_of_match);

static struct platform_driver s32cc_pcie_driver = {
	.driver = {
		.name	= "s32cc-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = s32cc_pcie_of_match,
	},
	.probe = s32cc_pcie_probe,
	.shutdown = s32cc_pcie_shutdown,
};

module_platform_driver(s32cc_pcie_driver);

MODULE_AUTHOR("Ionut Vicovan <Ionut.Vicovan@nxp.com>");
MODULE_DESCRIPTION("NXP S32CC PCIe host controller driver");
MODULE_LICENSE("GPL v2");
