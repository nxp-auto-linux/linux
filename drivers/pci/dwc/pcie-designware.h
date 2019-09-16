/*
 * Synopsys DesignWare PCIe host controller driver
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Jingoo Han <jg1.han@samsung.com>
 *
 * Customizations for the NXP S32V PCIE driver
 * Copyright 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PCIE_DESIGNWARE_H
#define _PCIE_DESIGNWARE_H

#include <linux/irq.h>
#include <linux/msi.h>
#include <linux/pci.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

/* Parameters for the waiting for link up routine */
#define LINK_WAIT_MAX_RETRIES		10
#define LINK_WAIT_USLEEP_MIN		90000
#define LINK_WAIT_USLEEP_MAX		100000

/* Parameters for the waiting for iATU enabled routine */
#define LINK_WAIT_MAX_IATU_RETRIES	5
#define LINK_WAIT_IATU			9

/* Synopsys-specific PCIe configuration registers */
#define PCIE_PORT_LINK_CONTROL		0x710
#define PORT_LINK_MODE_MASK		(0x3f << 16)
#define PORT_LINK_MODE_1_LANES		(0x1 << 16)
#define PORT_LINK_MODE_2_LANES		(0x3 << 16)
#define PORT_LINK_MODE_4_LANES		(0x7 << 16)
#define PORT_LINK_MODE_8_LANES		(0xf << 16)

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C
#define PORT_LOGIC_SPEED_CHANGE		(0x1 << 17)
#define PORT_LOGIC_LINK_WIDTH_MASK	(0x1f << 8)
#define PORT_LOGIC_LINK_WIDTH_1_LANES	(0x1 << 8)
#define PORT_LOGIC_LINK_WIDTH_2_LANES	(0x2 << 8)
#define PORT_LOGIC_LINK_WIDTH_4_LANES	(0x4 << 8)
#define PORT_LOGIC_LINK_WIDTH_8_LANES	(0x8 << 8)

#define PCIE_MSI_ADDR_LO		0x820
#define PCIE_MSI_ADDR_HI		0x824
#define PCIE_MSI_INTR0_ENABLE		0x828
#define PCIE_MSI_INTR0_MASK		0x82C
#define PCIE_MSI_INTR0_STATUS		0x830

#define PCIE_ATU_VIEWPORT		0x900
#define PCIE_ATU_REGION_INBOUND		(0x1 << 31)
#define PCIE_ATU_REGION_OUTBOUND	(0x0 << 31)
#define PCIE_ATU_REGION_INDEX2		(0x2 << 0)
#define PCIE_ATU_REGION_INDEX1		(0x1 << 0)
#define PCIE_ATU_REGION_INDEX0		(0x0 << 0)
#define PCIE_ATU_CR1			0x904
#define PCIE_ATU_TYPE_MEM		(0x0 << 0)
#define PCIE_ATU_TYPE_IO		(0x2 << 0)
#define PCIE_ATU_TYPE_CFG0		(0x4 << 0)
#define PCIE_ATU_TYPE_CFG1		(0x5 << 0)
#define PCIE_ATU_CR2			0x908
#define PCIE_ATU_ENABLE			(0x1 << 31)
#define PCIE_ATU_BAR_MODE_ENABLE	(0x1 << 30)
#define PCIE_ATU_LOWER_BASE		0x90C
#define PCIE_ATU_UPPER_BASE		0x910
#define PCIE_ATU_LIMIT			0x914
#define PCIE_ATU_LOWER_TARGET		0x918
#define PCIE_ATU_BUS(x)			(((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)			(((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)		(((x) & 0x7) << 16)
#define PCIE_ATU_UPPER_TARGET		0x91C

#ifdef CONFIG_PCI_DW_DMA
#define PCIE_DMA_WRITE_ENGINE_EN				0x97c
#define PCIE_DMA_WRITE_DOORBELL					0x980
#define PCIE_DMA_READ_ENGINE_EN					0x99C
#define PCIE_DMA_READ_DOORBELL					0x9A0
#define PCIE_DMA_WRITE_INT_STATUS				0x9BC
#define PCIE_DMA_WRITE_INT_MASK					0x9C4
#define PCIE_DMA_WRITE_INT_CLEAR				0x9C8
#define PCIE_DMA_WRITE_ERR_STATUS				0x9CC
#define PCIE_DMA_WRITE_DONE_IMWR_LOW			0x9D0
#define PCIE_DMA_WRITE_DONE_IMWR_HIGH			0x9D4
#define PCIE_DMA_WRITE_ABORT_IMWR_LOW			0x9D8
#define PCIE_DMA_WRITE_ABORT_IMWR_HIGH			0x9DC
#define PCIE_DMA_WRITE_CH01_IMWR_DATA			0x9E0
#define PCIE_DMA_WRITE_CH23_IMWR_DATA			0x9E4
#define PCIE_DMA_WRITE_CH45_IMWR_DATA			0x9E8
#define PCIE_DMA_WRITE_CH67_IMWR_DATA			0x9EC
#define PCIE_DMA_WRITE_LINKED_LIST_ERR_EN		0xA00

#define PCIE_DMA_READ_INT_STATUS				0xA10
#define PCIE_DMA_READ_INT_MASK					0xA18
#define PCIE_DMA_READ_INT_CLEAR					0xA1C
#define PCIE_DMA_READ_ERR_STATUS_LOW			0xA24
#define PCIE_DMA_READ_ERR_STATUS_HIGH			0xA28
#define PCIE_DMA_READ_LINKED_LIST_ERR_EN		0xA34
#define PCIE_DMA_READ_DONE_IMWR_LOW				0xA3C
#define PCIE_DMA_READ_DONE_IMWR_HIGH			0xA40
#define PCIE_DMA_READ_ABORT_IMWR_LOW			0xA44
#define PCIE_DMA_READ_ABORT_IMWR_HIGH			0xA48
#define PCIE_DMA_READ_CH01_IMWR_DATA			0xA4C
#define PCIE_DMA_READ_CH23_IMWR_DATA			0xA50
#define PCIE_DMA_READ_CH45_IMWR_DATA			0xA54
#define PCIE_DMA_READ_CH67_IMWR_DATA			0xA58

#define PCIE_DMA_VIEWPORT_SEL					0xA6C
#define PCIE_DMA_CH_CONTROL1					0xA70
#define PCIE_DMA_CH_CONTROL2					0xA74
#define PCIE_DMA_TRANSFER_SIZE					0xA78
#define PCIE_DMA_SAR_LOW						0xA7C
#define PCIE_DMA_SAR_HIGH						0xA80
#define PCIE_DMA_DAR_LOW						0xA84
#define PCIE_DMA_DAR_HIGH						0xA88
#define PCIE_DMA_LLP_LOW						0xA8C
#define PCIE_DMA_LLP_HIGH						0xA90

#endif /* CONFIG_PCI_DW_DMA */

#define PCIE_MISC_CONTROL_1_OFF		0x8BC
#define PCIE_DBI_RO_WR_EN		(0x1 << 0)

/*
 * iATU Unroll-specific register definitions
 * From 4.80 core version the address translation will be made by unroll
 */
#define PCIE_ATU_UNR_REGION_CTRL1	0x00
#define PCIE_ATU_UNR_REGION_CTRL2	0x04
#define PCIE_ATU_UNR_LOWER_BASE		0x08
#define PCIE_ATU_UNR_UPPER_BASE		0x0C
#define PCIE_ATU_UNR_LIMIT		0x10
#define PCIE_ATU_UNR_LOWER_TARGET	0x14
#define PCIE_ATU_UNR_UPPER_TARGET	0x18

/* Register address builder */
#define PCIE_GET_ATU_OUTB_UNR_REG_OFFSET(region)	\
			((0x3 << 20) | ((region) << 9))

#define PCIE_GET_ATU_INB_UNR_REG_OFFSET(region)				\
			((0x3 << 20) | ((region) << 9) | (0x1 << 8))

#define MSI_MESSAGE_CONTROL		0x52
#define MSI_CAP_MMC_SHIFT		1
#define MSI_CAP_MMC_MASK		(7 << MSI_CAP_MMC_SHIFT)
#define MSI_CAP_MME_SHIFT		4
#define MSI_CAP_MSI_EN_MASK		0x1
#define MSI_CAP_MME_MASK		(7 << MSI_CAP_MME_SHIFT)
#define MSI_MESSAGE_ADDR_L32		0x54
#define MSI_MESSAGE_ADDR_U32		0x58

/*
 * Maximum number of MSI IRQs can be 256 per controller. But keep
 * it 32 as of now. Probably we will never need more than 32. If needed,
 * then increment it in multiple of 32.
 */
#define MAX_MSI_IRQS			32
#define MAX_MSI_CTRLS			(MAX_MSI_IRQS / 32)

#ifdef CONFIG_PCI_DW_DMA
/* DW DMA Internal flags */
/* TODO: Number of channels and max size should come from a
 * kernel config node under CONFIG_PCI_DW_DMA
 */
#ifndef PCIE_DMA_NR_CH
#define PCIE_DMA_NR_CH		1
#endif
#ifndef PCIE_DMA_MAX_SIZE
#define PCIE_DMA_MAX_SIZE	(4 * 1024 * 1024)  /* 4G bytes */
#endif
#define DMA_FLAG_LIE         (1 << 0)
#define DMA_FLAG_RIE         (1 << 1)
#define DMA_FLAG_LLP         (1 << 2)
#define DMA_FLAG_WRITE_ELEM			(1 << 3)
#define DMA_FLAG_READ_ELEM			(1 << 4)
#define DMA_FLAG_EN_DONE_INT		(1 << 5)
#define DMA_FLAG_EN_ABORT_INT		(1 << 6)
#define DMA_FLAG_EN_REMOTE_DONE_INT			(1 << 7)
#define DMA_FLAG_EN_REMOTE_ABORT_INT		(1 << 8)
#endif

#ifdef CONFIG_PCI_DW_DMA
enum DMA_CH_FLAGS {
	DMA_CH_STOPPED = 0,
	DMA_CH_RUNNING,
	DMA_CH_HALTED,
	DMA_CH_STOPPED_FATAL,
};
enum DMA_CH_DIR {
	DMA_CH_WRITE = 0,
	DMA_CH_READ
};
enum DMA_ERROR {
	DMA_ERR_NONE = 0,
	DMA_ERR_WR,
	DMA_ERR_RD,
	DMA_ERR_FETCH_LL,
	DMA_ERR_UNSUPPORTED_REQ,
	DMA_ERR_CPL_ABORT,
	DMA_ERR_CPL_TIMEOUT,
	DMA_ERR_DATA_POISIONING
};
/* Linked list mode struct */
struct dma_ll_info {
	u32 direction;
	u32 ch_num;
	u32 nr_elem;
	u32 phy_list_addr;
	u32 next_phy_list_addr;
};
/* Channel info struct */
struct dma_ch_info {
	u32 direction;
	u32	status;
	u32 errors;
	u32 phy_list_addr;
	u32 current_sar;
	u32 current_dar;
	u32 current_size;
	u32 *virt_addr;
	u8 current_elem_idx;
	u8 current_list_size;
};
/* Single block DMA transfer struct */
struct dma_data_elem {
	u64 sar;
	u64 dar;
	u64 imwr;
	u32 size;
	u32 flags;
	u32 ch_num;
};
/* Type of array of structures for passing linked list  */
struct dma_list {
	u64 sar;
	u64 dar;
	u32 size;
};
#endif

struct pcie_port;
struct dw_pcie;
struct dw_pcie_ep;

enum dw_pcie_region_type {
	DW_PCIE_REGION_UNKNOWN,
	DW_PCIE_REGION_INBOUND,
	DW_PCIE_REGION_OUTBOUND,
};

enum dw_pcie_device_mode {
	DW_PCIE_UNKNOWN_TYPE,
	DW_PCIE_EP_TYPE,
	DW_PCIE_LEG_EP_TYPE,
	DW_PCIE_RC_TYPE,
};

struct dw_pcie_host_ops {
	int (*rd_own_conf)(struct pcie_port *pp, int where, int size, u32 *val);
	int (*wr_own_conf)(struct pcie_port *pp, int where, int size, u32 val);
	int (*rd_other_conf)(struct pcie_port *pp, struct pci_bus *bus,
			     unsigned int devfn, int where, int size, u32 *val);
	int (*wr_other_conf)(struct pcie_port *pp, struct pci_bus *bus,
			     unsigned int devfn, int where, int size, u32 val);
	int (*host_init)(struct pcie_port *pp);
	void (*msi_set_irq)(struct pcie_port *pp, int irq);
	void (*msi_clear_irq)(struct pcie_port *pp, int irq);
	phys_addr_t (*get_msi_addr)(struct pcie_port *pp);
	u32 (*get_msi_data)(struct pcie_port *pp, int pos);
	void (*scan_bus)(struct pcie_port *pp);
	int (*msi_host_init)(struct pcie_port *pp, struct msi_controller *chip);
};

struct pcie_port {
	u8			root_bus_nr;
	u64			cfg0_base;
	void __iomem		*va_cfg0_base;
	u32			cfg0_size;
	u64			cfg1_base;
	void __iomem		*va_cfg1_base;
	u32			cfg1_size;
	resource_size_t		io_base;
	phys_addr_t		io_bus_addr;
	u32			io_size;
	u64			mem_base;
	phys_addr_t		mem_bus_addr;
	u32			mem_size;
	struct resource		*cfg;
	struct resource		*io;
	struct resource		*mem;
	struct resource		*busn;
	int			irq;
	const struct dw_pcie_host_ops *ops;
	int			msi_irq;
	#ifdef CONFIG_PCI_DW_DMA
	int			dma_irq;
	int			link_req_rst_not_irq;
	#endif
	struct irq_domain	*irq_domain;
	unsigned long		msi_data;
	DECLARE_BITMAP(msi_irq_in_use, MAX_MSI_IRQS);
	#ifdef CONFIG_PCI_S32V234
	struct dentry		*dir;
	int			user_pid;
	struct siginfo	info;    /* signal information */
	void (*call_back)(u32 arg);
	#endif
	#ifdef CONFIG_PCI_DW_DMA
	struct dma_ch_info	wr_ch;
	struct dma_ch_info	rd_ch;
	struct dma_ll_info	ll_info;
	struct dma_list(*dma_linked_list)[];
	int (*ptr_func)(u32 arg);
	#endif

};

enum dw_pcie_as_type {
	DW_PCIE_AS_UNKNOWN,
	DW_PCIE_AS_MEM,
	DW_PCIE_AS_IO,
};

struct dw_pcie_ep_ops {
	void	(*ep_init)(struct dw_pcie_ep *ep);
	int	(*raise_irq)(struct dw_pcie_ep *ep, enum pci_epc_irq_type type,
			     u8 interrupt_num);
};

struct dw_pcie_ep {
	struct pci_epc		*epc;
	struct dw_pcie_ep_ops	*ops;
	phys_addr_t		phys_base;
	size_t			addr_size;
	size_t			page_size;
	u8			bar_to_atu[6];
	phys_addr_t		*outbound_addr;
	unsigned long		ib_window_map;
	unsigned long		ob_window_map;
	u32			num_ib_windows;
	u32			num_ob_windows;
};

struct dw_pcie_ops {
	u64	(*cpu_addr_fixup)(u64 cpu_addr);
	u32	(*read_dbi)(struct dw_pcie *pcie, void __iomem *base, u32 reg,
			    size_t size);
	void	(*write_dbi)(struct dw_pcie *pcie, void __iomem *base, u32 reg,
			     size_t size, u32 val);
	int	(*link_up)(struct dw_pcie *pcie);
	int	(*start_link)(struct dw_pcie *pcie);
	void	(*stop_link)(struct dw_pcie *pcie);
	#ifdef CONFIG_PCI_S32V234
	int (*send_signal_to_user)(struct pcie_port *pp);
	#endif
};

struct dw_pcie {
	struct device		*dev;
	void __iomem		*dbi_base;
	void __iomem		*dbi_base2;
	u32			num_viewport;
	u8			iatu_unroll_enabled;
	struct pcie_port	pp;
	struct dw_pcie_ep	ep;
	const struct dw_pcie_ops *ops;
};

#define to_dw_pcie_from_pp(port) container_of((port), struct dw_pcie, pp)

#define to_dw_pcie_from_ep(endpoint)   \
		container_of((endpoint), struct dw_pcie, ep)

int dw_pcie_read(void __iomem *addr, int size, u32 *val);
int dw_pcie_write(void __iomem *addr, int size, u32 val);

u32 __dw_pcie_read_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg,
		       size_t size);
void __dw_pcie_write_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg,
			 size_t size, u32 val);
int dw_pcie_link_up(struct dw_pcie *pci);
int dw_pcie_wait_for_link(struct dw_pcie *pci);
void dw_pcie_prog_outbound_atu(struct dw_pcie *pci, int index,
			       int type, u64 cpu_addr, u64 pci_addr,
			       u32 size);
int dw_pcie_prog_inbound_atu(struct dw_pcie *pci, int index, int bar,
			     u64 cpu_addr, enum dw_pcie_as_type as_type);
void dw_pcie_disable_atu(struct dw_pcie *pci, int index,
			 enum dw_pcie_region_type type);
void dw_pcie_setup(struct dw_pcie *pci);

static inline void dw_pcie_writel_dbi(struct dw_pcie *pci, u32 reg, u32 val)
{
	__dw_pcie_write_dbi(pci, pci->dbi_base, reg, 0x4, val);
}

static inline u32 dw_pcie_readl_dbi(struct dw_pcie *pci, u32 reg)
{
	return __dw_pcie_read_dbi(pci, pci->dbi_base, reg, 0x4);
}

static inline void dw_pcie_writew_dbi(struct dw_pcie *pci, u32 reg, u16 val)
{
	__dw_pcie_write_dbi(pci, pci->dbi_base, reg, 0x2, val);
}

static inline u16 dw_pcie_readw_dbi(struct dw_pcie *pci, u32 reg)
{
	return __dw_pcie_read_dbi(pci, pci->dbi_base, reg, 0x2);
}

static inline void dw_pcie_writeb_dbi(struct dw_pcie *pci, u32 reg, u8 val)
{
	__dw_pcie_write_dbi(pci, pci->dbi_base, reg, 0x1, val);
}

static inline u8 dw_pcie_readb_dbi(struct dw_pcie *pci, u32 reg)
{
	return __dw_pcie_read_dbi(pci, pci->dbi_base, reg, 0x1);
}

static inline void dw_pcie_writel_dbi2(struct dw_pcie *pci, u32 reg, u32 val)
{
	__dw_pcie_write_dbi(pci, pci->dbi_base2, reg, 0x4, val);
}

static inline u32 dw_pcie_readl_dbi2(struct dw_pcie *pci, u32 reg)
{
	return __dw_pcie_read_dbi(pci, pci->dbi_base2, reg, 0x4);
}

static inline void dw_pcie_dbi_ro_wr_en(struct dw_pcie *pci)
{
	u32 reg;
	u32 val;

	reg = PCIE_MISC_CONTROL_1_OFF;
	val = dw_pcie_readl_dbi(pci, reg);
	val |= PCIE_DBI_RO_WR_EN;
	dw_pcie_writel_dbi(pci, reg, val);
}

static inline void dw_pcie_dbi_ro_wr_dis(struct dw_pcie *pci)
{
	u32 reg;
	u32 val;

	reg = PCIE_MISC_CONTROL_1_OFF;
	val = dw_pcie_readl_dbi(pci, reg);
	val &= ~PCIE_DBI_RO_WR_EN;
	dw_pcie_writel_dbi(pci, reg, val);
}

#ifdef CONFIG_PCIE_DW_HOST
irqreturn_t dw_handle_msi_irq(struct pcie_port *pp);
void dw_pcie_msi_init(struct pcie_port *pp);
void dw_pcie_setup_rc(struct pcie_port *pp);
int dw_pcie_host_init(struct pcie_port *pp);
#else
static inline irqreturn_t dw_handle_msi_irq(struct pcie_port *pp)
{
	return IRQ_NONE;
}

static inline void dw_pcie_msi_init(struct pcie_port *pp)
{
}

static inline void dw_pcie_setup_rc(struct pcie_port *pp)
{
}

static inline int dw_pcie_host_init(struct pcie_port *pp)
{
	return 0;
}
#endif

#ifdef CONFIG_PCI_DW_DMA
int dw_pcie_dma_write_en(struct pcie_port *pp);
int dw_pcie_dma_read_en(struct pcie_port *pp);
int dw_pcie_dma_write_soft_reset(struct pcie_port *pp);
int dw_pcie_dma_read_soft_reset(struct pcie_port *pp);
irqreturn_t dw_handle_dma_irq(struct pcie_port *pp);
void dw_pcie_dma_set_wr_remote_done_int(struct pcie_port *pp, u64 val);
void dw_pcie_dma_set_wr_remote_abort_int(struct pcie_port *pp, u64 val);
void dw_pcie_dma_en_local_int(struct pcie_port *pp/* , u64 val */);
void dw_pcie_dma_set_sar(struct pcie_port *pp, u64 val);
void dw_pcie_dma_set_dar(struct pcie_port *pp, u64 val);
void dw_pcie_dma_set_transfer_size(struct pcie_port *pp, u32 val);
void dw_pcie_dma_set_rd_viewport(struct pcie_port *pp, u8 ch_nr);
void dw_pcie_dma_set_wr_viewport(struct pcie_port *pp, u8 ch_nr);
void dw_pcie_dma_clear_regs(struct pcie_port *pp);
int dw_pcie_dma_single_rw(struct pcie_port *pp,
	struct dma_data_elem *dma_single_rw);
int dw_pcie_dma_load_linked_list(struct pcie_port *pp,
	struct dma_list arr_ll[], u8 arr_sz, u32 phy_list_addr,
	u32 next_phy_list_addr, u8 direction);
int dw_pcie_dma_start_linked_list(struct pcie_port *pp,
	u32 phy_list_addr,
	u8 direction);
void dw_start_dma_llw(struct pcie_port *pp, u64 phy_list_addr);
#else
static inline int dw_pcie_dma_write_en(struct pcie_port *pp)
{
	return 0;
}
static inline int dw_pcie_dma_read_en(struct pcie_port *pp)
{
	return 0;
}
static inline int dw_pcie_dma_write_soft_reset(struct pcie_port *pp)
{
	return 0;
}
static inline int dw_pcie_dma_read_soft_reset(struct pcie_port *pp)
{
	return 0;
}
static inline irqreturn_t dw_handle_dma_irq(struct pcie_port *pp)
{
	return IRQ_NONE;
}
static inline void dw_pcie_dma_set_wr_remote_done_int(struct pcie_port *pp,
	u64 val)
{
}
static inline void dw_pcie_dma_set_wr_remote_abort_int(struct pcie_port *pp,
	u64 val)
{
}
static inline void dw_pcie_dma_en_local_int(struct pcie_port *pp/* , u64 val */)
{
}
static inline void dw_pcie_dma_set_sar(struct pcie_port *pp, u64 val)
{
}
static inline void dw_pcie_dma_set_dar(struct pcie_port *pp, u64 val)
{
}
static inline void dw_pcie_dma_set_transfer_size(struct pcie_port *pp, u32 val)
{
}
static inline void dw_pcie_dma_set_rd_viewport(struct pcie_port *pp, u8 ch_nr)
{
}
static inline void dw_pcie_dma_set_wr_viewport(struct pcie_port *pp, u8 ch_nr)
{
}
static inline void dw_pcie_dma_clear_regs(struct pcie_port *pp)
{
}
static inline int dw_pcie_dma_single_rw(struct pcie_port *pp,
	struct dma_data_elem *dma_single_rw)
{
	return 0;
}
static inline int dw_pcie_dma_load_linked_list(struct pcie_port *pp,
	struct dma_list (*arr_ll)[], u8 arr_sz, u32 phy_list_addr,
	u32 next_phy_list_addr, u8 direction)
{
	return 0;
}
static inline int dw_pcie_dma_start_linked_list(struct pcie_port *pp,
	u32 phy_list_addr,
	u8 direction)
{
	return 0;
}
static inline void dw_start_dma_llw(struct pcie_port *pp, u64 val)
{
}
#endif

#ifdef CONFIG_PCIE_DW_EP
void dw_pcie_ep_linkup(struct dw_pcie_ep *ep);
int dw_pcie_ep_init(struct dw_pcie_ep *ep);
void dw_pcie_ep_exit(struct dw_pcie_ep *ep);
#else
static inline void dw_pcie_ep_linkup(struct dw_pcie_ep *ep)
{
}

static inline int dw_pcie_ep_init(struct dw_pcie_ep *ep)
{
	return 0;
}

static inline void dw_pcie_ep_exit(struct dw_pcie_ep *ep)
{
}
#endif
#endif /* _PCIE_DESIGNWARE_H */
