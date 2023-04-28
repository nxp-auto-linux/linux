/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DMA support for the Synopsys DesignWare
 * PCIe host controller driver, customized
 * for the NXP S32CC PCIe driver
 *
 * Copyright 2017-2023 NXP
 */

#ifndef PCIE_DMA_S32CC_H
#define PCIE_DMA_S32CC_H

#include "pcie-designware.h"
#if (IS_ENABLED(CONFIG_PCI_EPF_TEST))
#include <linux/completion.h>
#endif

/* Synopsys-specific PCIe configuration registers */
#define PCIE_DMA_CTRL				(0x008)
#define PCIE_DMA_WRITE_ENGINE_EN		(0x00C)
#define PCIE_DMA_WRITE_DOORBELL			(0x010)
#define PCIE_DMA_READ_ENGINE_EN			(0x02C)
#define PCIE_DMA_READ_DOORBELL			(0x030)
#define PCIE_DMA_WRITE_INT_STATUS		(0x04C)
#define PCIE_DMA_WRITE_INT_MASK			(0x054)
#define PCIE_DMA_WRITE_INT_CLEAR		(0x058)
#define PCIE_DMA_WRITE_ERR_STATUS		(0x05C)
#define PCIE_DMA_WRITE_DONE_IMWR_LOW		(0x060)
#define PCIE_DMA_WRITE_DONE_IMWR_HIGH		(0x064)
#define PCIE_DMA_WRITE_ABORT_IMWR_LOW		(0x068)
#define PCIE_DMA_WRITE_ABORT_IMWR_HIGH		(0x06C)
#define PCIE_DMA_WRITE_CH01_IMWR_DATA		(0x070)
#define PCIE_DMA_WRITE_CH23_IMWR_DATA		(0x074)
#define PCIE_DMA_WRITE_CH45_IMWR_DATA		(0x078)
#define PCIE_DMA_WRITE_CH67_IMWR_DATA		(0x07C)
#define PCIE_DMA_WRITE_LINKED_LIST_ERR_EN	(0x090)

#define PCIE_DMA_READ_INT_STATUS		(0x0A0)
#define PCIE_DMA_READ_INT_MASK			(0x0A8)
#define PCIE_DMA_READ_INT_CLEAR			(0x0AC)
#define PCIE_DMA_READ_ERR_STATUS_LOW		(0x0B4)
#define PCIE_DMA_READ_ERR_STATUS_HIGH		(0x0B8)
#define PCIE_DMA_READ_LINKED_LIST_ERR_EN	(0x0C4)
#define PCIE_DMA_READ_DONE_IMWR_LOW		(0x0CC)
#define PCIE_DMA_READ_DONE_IMWR_HIGH		(0x0D0)
#define PCIE_DMA_READ_ABORT_IMWR_LOW		(0x0D4)
#define PCIE_DMA_READ_ABORT_IMWR_HIGH		(0x0D8)
#define PCIE_DMA_READ_CH01_IMWR_DATA		(0x0DC)
#define PCIE_DMA_READ_CH23_IMWR_DATA		(0x0E0)
#define PCIE_DMA_READ_CH45_IMWR_DATA		(0x0E4)
#define PCIE_DMA_READ_CH67_IMWR_DATA		(0x0E8)

/* These are register offsets used for the compressed/viewport layout */
#define PCIE_DMA_VIEWPORT_SEL			(0x0FC)
#define PCIE_DMA_CH_BASE				(0x100)
/* This is the base register offset to be used for the unrolled layout */
#define PCIE_DMA_CH_BASE_UNROLL			(0x200)
#define PCIE_DMA_IATU_INBOUND_OFF		(0x100)

#define PCIE_DMA_CH_CONTROL1_OFF		(0x00)
#define PCIE_DMA_CH_CONTROL2_OFF		(0x04)
#define PCIE_DMA_TRANSFER_SIZE_OFF		(0x08)
#define PCIE_DMA_SAR_LOW_OFF			(0x0C)
#define PCIE_DMA_SAR_HIGH_OFF			(0x10)
#define PCIE_DMA_DAR_LOW_OFF			(0x14)
#define PCIE_DMA_DAR_HIGH_OFF			(0x18)
#define PCIE_DMA_LLP_LOW_OFF			(0x1C)
#define PCIE_DMA_LLP_HIGH_OFF			(0x120)

#define NUM_DMA_RD_CHAN_MASK		GENMASK(19, 16)
#define NUM_DMA_RD_CHAN_SHIFT		16

#define NUM_DMA_WR_CHAN_MASK	GENMASK(3, 0)

#define DMA_FLAG_LIE					BIT(0)
#define DMA_FLAG_RIE					BIT(1)
#define DMA_FLAG_LLP					BIT(2)
#define DMA_FLAG_WRITE_ELEM				BIT(3)
#define DMA_FLAG_READ_ELEM				BIT(4)
#define DMA_FLAG_EN_DONE_INT			BIT(5)
#define DMA_FLAG_EN_ABORT_INT			BIT(6)
#define DMA_FLAG_EN_REMOTE_DONE_INT		BIT(7)
#define DMA_FLAG_EN_REMOTE_ABORT_INT	BIT(8)

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

/* Single block DMA transfer struct, also used for ioctl.
 * Should match the data structure used in the user space.
 */
struct dma_data_elem {
	unsigned long sar;
	unsigned long dar;
	unsigned long imwr;
	unsigned int size;
	unsigned int flags;
	unsigned int ch_num;
};

struct dma_info {
	void __iomem *dma_base;
	u8	iatu_unroll_enabled; /* copy of its homonym in struct dw_pcie */
	u32	(*read_dma)(struct dma_info *di, void __iomem *base,
			u32 reg, size_t size);
	void (*write_dma)(struct dma_info *di, void __iomem *base,
			u32 reg, size_t size, u32 val);

	struct dma_ch_info	wr_ch;
	struct dma_ch_info	rd_ch;
#if (IS_ENABLED(CONFIG_PCI_EPF_TEST))
	struct completion *complete;
#endif
	void (*call_back)(u32 arg);
};

void s32cc_config_dma_data(struct dma_info *di, struct dw_pcie *pcie);
struct dma_info *dw_get_dma_info(struct dw_pcie *pcie);

u32 dw_pcie_read_dma(struct dma_info *di, u32 reg, size_t size);
void dw_pcie_write_dma(struct dma_info *di, u32 reg, size_t size, u32 val);

static inline void dw_pcie_writel_dma(struct dma_info *di, u32 reg, u32 val)
{
	dw_pcie_write_dma(di, reg, 0x4, val);
}

static inline u32 dw_pcie_readl_dma(struct dma_info *di, u32 reg)
{
	return dw_pcie_read_dma(di, reg, 0x4);
}

int dw_pcie_dma_write_en(struct dma_info *di);
int dw_pcie_dma_read_en(struct dma_info *di);
int dw_pcie_dma_write_soft_reset(struct dma_info *di);
int dw_pcie_dma_read_soft_reset(struct dma_info *di);

irqreturn_t s32cc_pcie_dma_handler(int irq, void *arg);

static inline int dw_pcie_dma_get_nr_chan(struct dma_info *di)
{
	u32 dma_nr_ch = dw_pcie_readl_dma(di, PCIE_DMA_CTRL);

	dma_nr_ch = (dma_nr_ch & NUM_DMA_RD_CHAN_MASK)
					>> NUM_DMA_RD_CHAN_SHIFT;
	return dma_nr_ch;
}

void dw_pcie_dma_set_sar(struct dma_info *di, u64 val, u8 ch_nr, u8 dir);
void dw_pcie_dma_set_dar(struct dma_info *di, u64 val, u8 ch_nr, u8 dir);
void dw_pcie_dma_set_transfer_size(struct dma_info *di,
	u32 val, u8 ch_nr, u8 dir);

void dw_pcie_dma_set_viewport(struct dma_info *di, u8 ch_nr, u8 dir);

void dw_pcie_dma_clear_regs(struct dma_info *di);
int dw_pcie_dma_single_rw(struct dma_info *di,
	struct dma_data_elem *dma_single_rw);

void s32cc_register_callback(struct dw_pcie *pcie,
	void (*call_back)(u32 arg));

#if (IS_ENABLED(CONFIG_PCI_EPF_TEST))
/**
 * dw_pcie_ep_start_dma - Start DMA on S32CC PCIE EP.
 * @ep: the EP start the DMA transmission.
 * @dir: direction of the DMA, 1 read, 0 write;
 * @src: source DMA address.
 * @dst: destination DMA address.
 * @len: transfer length.
 */
int dw_pcie_ep_start_dma(struct dw_pcie_ep *ep, bool dir,
				 dma_addr_t src, dma_addr_t dst, u32 len,
				 struct completion *complete);
#endif

#endif  /* PCIE_DMA_S32CC_H */
