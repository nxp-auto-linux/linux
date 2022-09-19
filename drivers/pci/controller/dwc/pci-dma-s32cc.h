/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DMA support for the Synopsys DesignWare
 * PCIe host controller driver, customized
 * for the NXP S32CC PCIe driver
 *
 * Copyright 2017-2021 NXP
 */

#ifndef PCIE_DMA_S32CC_H
#define PCIE_DMA_S32CC_H

#include "pcie-designware.h"

#define PCIE_DMA_BASE	0x70000

/* Synopsys-specific PCIe configuration registers */
#define PCIE_DMA_CTRL				(PCIE_DMA_BASE + 0x008)
#define PCIE_DMA_WRITE_ENGINE_EN		(PCIE_DMA_BASE + 0x00C)
#define PCIE_DMA_WRITE_DOORBELL			(PCIE_DMA_BASE + 0x010)
#define PCIE_DMA_READ_ENGINE_EN			(PCIE_DMA_BASE + 0x02C)
#define PCIE_DMA_READ_DOORBELL			(PCIE_DMA_BASE + 0x030)
#define PCIE_DMA_WRITE_INT_STATUS		(PCIE_DMA_BASE + 0x04C)
#define PCIE_DMA_WRITE_INT_MASK			(PCIE_DMA_BASE + 0x054)
#define PCIE_DMA_WRITE_INT_CLEAR		(PCIE_DMA_BASE + 0x058)
#define PCIE_DMA_WRITE_ERR_STATUS		(PCIE_DMA_BASE + 0x05C)
#define PCIE_DMA_WRITE_DONE_IMWR_LOW		(PCIE_DMA_BASE + 0x060)
#define PCIE_DMA_WRITE_DONE_IMWR_HIGH		(PCIE_DMA_BASE + 0x064)
#define PCIE_DMA_WRITE_ABORT_IMWR_LOW		(PCIE_DMA_BASE + 0x068)
#define PCIE_DMA_WRITE_ABORT_IMWR_HIGH		(PCIE_DMA_BASE + 0x06C)
#define PCIE_DMA_WRITE_CH01_IMWR_DATA		(PCIE_DMA_BASE + 0x070)
#define PCIE_DMA_WRITE_CH23_IMWR_DATA		(PCIE_DMA_BASE + 0x074)
#define PCIE_DMA_WRITE_CH45_IMWR_DATA		(PCIE_DMA_BASE + 0x078)
#define PCIE_DMA_WRITE_CH67_IMWR_DATA		(PCIE_DMA_BASE + 0x07C)
#define PCIE_DMA_WRITE_LINKED_LIST_ERR_EN	(PCIE_DMA_BASE + 0x090)

#define PCIE_DMA_READ_INT_STATUS		(PCIE_DMA_BASE + 0x0A0)
#define PCIE_DMA_READ_INT_MASK			(PCIE_DMA_BASE + 0x0A8)
#define PCIE_DMA_READ_INT_CLEAR			(PCIE_DMA_BASE + 0x0AC)
#define PCIE_DMA_READ_ERR_STATUS_LOW		(PCIE_DMA_BASE + 0x0B4)
#define PCIE_DMA_READ_ERR_STATUS_HIGH		(PCIE_DMA_BASE + 0x0B8)
#define PCIE_DMA_READ_LINKED_LIST_ERR_EN	(PCIE_DMA_BASE + 0x0C4)
#define PCIE_DMA_READ_DONE_IMWR_LOW		(PCIE_DMA_BASE + 0x0CC)
#define PCIE_DMA_READ_DONE_IMWR_HIGH		(PCIE_DMA_BASE + 0x0D0)
#define PCIE_DMA_READ_ABORT_IMWR_LOW		(PCIE_DMA_BASE + 0x0D4)
#define PCIE_DMA_READ_ABORT_IMWR_HIGH		(PCIE_DMA_BASE + 0x0D8)
#define PCIE_DMA_READ_CH01_IMWR_DATA		(PCIE_DMA_BASE + 0x0DC)
#define PCIE_DMA_READ_CH23_IMWR_DATA		(PCIE_DMA_BASE + 0x0E0)
#define PCIE_DMA_READ_CH45_IMWR_DATA		(PCIE_DMA_BASE + 0x0E4)
#define PCIE_DMA_READ_CH67_IMWR_DATA		(PCIE_DMA_BASE + 0x0E8)

#define PCIE_DMA_VIEWPORT_SEL			(PCIE_DMA_BASE + 0x0FC)
#define PCIE_DMA_CH_CONTROL1			(PCIE_DMA_BASE + 0x100)
#define PCIE_DMA_CH_CONTROL2			(PCIE_DMA_BASE + 0x104)
#define PCIE_DMA_TRANSFER_SIZE			(PCIE_DMA_BASE + 0x108)
#define PCIE_DMA_SAR_LOW			(PCIE_DMA_BASE + 0x10C)
#define PCIE_DMA_SAR_HIGH			(PCIE_DMA_BASE + 0x110)
#define PCIE_DMA_DAR_LOW			(PCIE_DMA_BASE + 0x114)
#define PCIE_DMA_DAR_HIGH			(PCIE_DMA_BASE + 0x118)
#define PCIE_DMA_LLP_LOW			(PCIE_DMA_BASE + 0x11C)
#define PCIE_DMA_LLP_HIGH			(PCIE_DMA_BASE + 0x120)

#define DMA_FLAG_LIE         BIT(0)
#define DMA_FLAG_RIE         BIT(1)
#define DMA_FLAG_LLP         BIT(2)
#define DMA_FLAG_WRITE_ELEM			BIT(3)
#define DMA_FLAG_READ_ELEM			BIT(4)
#define DMA_FLAG_EN_DONE_INT		BIT(5)
#define DMA_FLAG_EN_ABORT_INT		BIT(6)
#define DMA_FLAG_EN_REMOTE_DONE_INT			BIT(7)
#define DMA_FLAG_EN_REMOTE_ABORT_INT		BIT(8)

#define NUM_DMA_RD_CHAN_MASK		0xF0000
#define NUM_DMA_RD_CHAN_SHIFT		16
#define NUM_DMA_WR_CHAN_MASK		0xF

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

struct dma_info {
	struct dma_ch_info	wr_ch;
	struct dma_ch_info	rd_ch;
	struct dma_ll_info	ll_info;
	struct dma_list(*dma_linked_list)[];
#ifdef DMA_PTR_FUNC
	int (*ptr_func)(u32 arg);
#endif /* DMA_PTR_FUNC */
};

int dw_pcie_dma_write_en(struct dw_pcie *pci);
int dw_pcie_dma_read_en(struct dw_pcie *pci);
int dw_pcie_dma_write_soft_reset(struct dw_pcie *pci, struct dma_info *di);
int dw_pcie_dma_read_soft_reset(struct dw_pcie *pci, struct dma_info *di);
irqreturn_t dw_handle_dma_irq(struct dw_pcie *pci);
void dw_pcie_dma_set_wr_remote_done_int(struct dw_pcie *pci, u64 val);
void dw_pcie_dma_set_wr_remote_abort_int(struct dw_pcie *pci, u64 val);
void dw_pcie_dma_en_local_int(struct dw_pcie *pci/* , u64 val */);
void dw_pcie_dma_set_sar(struct dw_pcie *pci, u64 val);
void dw_pcie_dma_set_dar(struct dw_pcie *pci, u64 val);
void dw_pcie_dma_set_transfer_size(struct dw_pcie *pci, u32 val);
void dw_pcie_dma_set_rd_viewport(struct dw_pcie *pci, u8 ch_nr);
void dw_pcie_dma_set_wr_viewport(struct dw_pcie *pci, u8 ch_nr);
void dw_pcie_dma_clear_regs(struct dw_pcie *pci, struct dma_info *di);
int dw_pcie_dma_single_rw(struct dw_pcie *pci, struct dma_info *di,
	struct dma_data_elem *dma_single_rw);
int dw_pcie_dma_load_linked_list(struct dw_pcie *pci, struct dma_info *di,
	u8 arr_sz, u32 phy_list_addr,
	u32 next_phy_list_addr, u8 direction);
int dw_pcie_dma_start_linked_list(struct dw_pcie *pci, struct dma_info *di,
	u32 phy_list_addr,
	u8 direction);
void dw_pcie_dma_start_llw(struct dw_pcie *pci, struct dma_info *di,
		u64 phy_list_addr);
irqreturn_t dw_handle_dma_irq_write(struct dw_pcie *pci, struct dma_info *di,
					u32 val_write);
irqreturn_t dw_handle_dma_irq_read(struct dw_pcie *pci, struct dma_info *di,
					u32 val_read);

#endif  /* PCIE_DMA_S32CC_H */
