// SPDX-License-Identifier: GPL-2.0
/*
 * DMA support for the Synopsys DesignWare
 * PCIe host controller driver, customized
 * for the NXP S32 PCIE driver
 *
 * Copyright 2017-2021 NXP
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/sizes.h>
#include <linux/of_platform.h>
#include <linux/sched/signal.h>
#include "pci-dma-s32.h"


u32 dw_pcie_read_dma(struct dma_info *di,
			u32 reg, size_t size)
{
	int ret;
	u32 val;

	if (di->read_dma)
		return di->read_dma(di, di->dma_base, reg, size);

	ret = dw_pcie_read(di->dma_base + reg, size, &val);
	if (ret)
		pr_err("%s: Read DMA address failed\n", __func__);

	return val;
}

void dw_pcie_write_dma(struct dma_info *di,
			u32 reg, size_t size, u32 val)
{
	int ret;

	if (di->write_dma) {
		di->write_dma(di, di->dma_base, reg, size, val);
		return;
	}

	ret = dw_pcie_write(di->dma_base + reg, size, val);
	if (ret)
		pr_err("%s: Write DMA address failed\n", __func__);
}

static inline u32 dw_pcie_get_dma_channel_base(struct dma_info *di,
	u8 ch_nr, u8 dir)
{
	if (di->iatu_unroll_enabled)
		return PCIE_DMA_CH_BASE_UNROLL * (ch_nr + 1) +
				0x100 * dir;
	return PCIE_DMA_CH_BASE;
}

int dw_pcie_dma_write_en(struct dma_info *di)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ENGINE_EN, 0x1);
	return 0;
}

int dw_pcie_dma_write_soft_reset(struct dma_info *di)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ENGINE_EN, 0x0);
	while (dw_pcie_readl_dma(di, PCIE_DMA_WRITE_ENGINE_EN) == 1)
		;
	di->wr_ch.status = DMA_CH_STOPPED;
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ENGINE_EN, 0x1);
	return 0;
}

int dw_pcie_dma_read_en(struct dma_info *di)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ENGINE_EN, 0x1);
	return 0;
}

int dw_pcie_dma_read_soft_reset(struct dma_info *di)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ENGINE_EN, 0x0);
	while (dw_pcie_readl_dma(di, PCIE_DMA_READ_ENGINE_EN) == 1)
		;
	di->rd_ch.status = DMA_CH_STOPPED;
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ENGINE_EN, 0x1);
	return 0;
}

static void dw_pcie_dma_set_wr_remote_done_int(struct dma_info *di, u64 val, u8 ch_nr)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, DMA_CH_WRITE);

	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DONE_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DONE_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x10);
}

static void dw_pcie_dma_set_wr_remote_abort_int(struct dma_info *di, u64 val, u8 ch_nr)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, DMA_CH_WRITE);

	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ABORT_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ABORT_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x10);
}

static void dw_pcie_dma_set_rd_remote_done_int(struct dma_info *di, u64 val, u8 ch_nr)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, DMA_CH_READ);

	dw_pcie_writel_dma(di, PCIE_DMA_READ_DONE_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_READ_DONE_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x10);
}

static void dw_pcie_dma_set_rd_remote_abort_int(struct dma_info *di, u64 val, u8 ch_nr)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, DMA_CH_READ);

	dw_pcie_writel_dma(di, PCIE_DMA_READ_ABORT_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ABORT_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x10);
}

static void dw_pcie_dma_en_local_int(struct dma_info *di, u8 ch_nr, u8 dir)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x8);
}

/* Interrupts mask and clear functions */
static void dw_pcie_dma_clear_wr_done_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_MASK) & (~val));
}

static void dw_pcie_dma_clear_wr_abort_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_MASK) &
				((~val) << 16));
}

static void dw_pcie_dma_clear_rd_done_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_READ_INT_MASK) & (~val));
}

static void dw_pcie_dma_clear_rd_abort_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_READ_INT_MASK) &
			((~val) << 16));
}

void dw_pcie_dma_set_sar(struct dma_info *di, u64 val, u8 ch_nr, u8 dir)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_SAR_LOW_OFF,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_SAR_HIGH_OFF,
		upper_32_bits(val));
}

void dw_pcie_dma_set_dar(struct dma_info *di, u64 val, u8 ch_nr, u8 dir)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_DAR_LOW_OFF,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_DAR_HIGH_OFF,
		upper_32_bits(val));
}

void dw_pcie_dma_set_transfer_size(struct dma_info *di, u32 val,
		u8 ch_nr, u8 dir)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_TRANSFER_SIZE_OFF, val);
}

void dw_pcie_dma_set_viewport(struct dma_info *di, u8 ch_nr, u8 dir)
{
	if (!di->iatu_unroll_enabled)
		dw_pcie_writel_dma(di, PCIE_DMA_VIEWPORT_SEL,
				(dir << 31) | ch_nr);
}

void dw_pcie_dma_clear_regs(struct dma_info *di)
{
	u8 ch = 0;
	u8 dir = DMA_CH_WRITE;

	di->wr_ch.status = DMA_CH_STOPPED;
	di->rd_ch.status = DMA_CH_STOPPED;

	for (dir = DMA_CH_WRITE; dir <= DMA_CH_READ; dir++) {
		u32 ch_base = dw_pcie_get_dma_channel_base(di, ch, dir);

		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL2_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_TRANSFER_SIZE_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_SAR_LOW_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_SAR_HIGH_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_DAR_LOW_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_DAR_HIGH_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_LLP_LOW_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_LLP_HIGH_OFF, 0);
	}

	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_CLEAR, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DONE_IMWR_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DONE_IMWR_HIGH, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ABORT_IMWR_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ABORT_IMWR_HIGH, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_CH01_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_CH23_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_CH45_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_CH67_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_CLEAR, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_DONE_IMWR_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_DONE_IMWR_HIGH, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ABORT_IMWR_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ABORT_IMWR_HIGH, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_CH01_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_CH23_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_CH45_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_CH67_IMWR_DATA, 0);
}

int dw_pcie_dma_single_rw(struct dma_info *di,
	struct dma_data_elem *dma_single_rw)
{
	u32 flags;
	struct dma_ch_info *ptr_ch;
	u8 ch_nr = dma_single_rw->ch_num;
	const u32 dma_nr_ch = dw_pcie_dma_get_nr_chan(di);
	u8 dir;
	u32 ch_base;

	/* Invalid channel number */
	if (ch_nr > dma_nr_ch - 1)
		return -EINVAL;

	/* Invalid transfer size */
	if (dma_single_rw->size > CONFIG_PCIE_DMA_MAX_SIZE)
		return -EINVAL;

	flags = dma_single_rw->flags;
	ptr_ch = (flags & DMA_FLAG_WRITE_ELEM) ?
		&di->wr_ch : &di->rd_ch;

	if (flags & DMA_FLAG_WRITE_ELEM) {
		/* Invalid channel number */
		if (dma_single_rw->ch_num > dma_nr_ch - 1)
			return -EINVAL;

		if (di->wr_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		di->wr_ch.status = DMA_CH_RUNNING;
		di->wr_ch.errors = 0;
		dw_pcie_dma_write_en(di);
		dir = DMA_CH_WRITE;
	} else {
		/* Invalid channel number */
		if (dma_single_rw->ch_num > dma_nr_ch - 1)
			return -EINVAL;

		if (di->rd_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		di->rd_ch.status = DMA_CH_RUNNING;
		di->rd_ch.errors = 0;
		dw_pcie_dma_read_en(di);
	}

	if (!di->iatu_unroll_enabled)
		dw_pcie_dma_set_viewport(di, ch_nr, dir);
	/* Clear CR1 for proper init */
	ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0);

	if (flags & (DMA_FLAG_EN_DONE_INT | DMA_FLAG_EN_ABORT_INT)) {
		dw_pcie_dma_en_local_int(di, ch_nr, dir);

		if (flags & (DMA_FLAG_RIE | DMA_FLAG_LIE)) {
			if (flags & DMA_FLAG_WRITE_ELEM) {
				dw_pcie_dma_set_wr_remote_abort_int(di,
					dma_single_rw->imwr, ch_nr);
				dw_pcie_dma_set_wr_remote_done_int(di,
					dma_single_rw->imwr, ch_nr);
				dw_pcie_dma_clear_wr_done_int_mask(di,
					(1 << ch_nr));
				dw_pcie_dma_clear_wr_abort_int_mask(di,
					(1 << ch_nr));
			} else if (flags & DMA_FLAG_READ_ELEM) {
				dw_pcie_dma_set_rd_remote_abort_int(di,
					dma_single_rw->imwr, ch_nr);
				dw_pcie_dma_set_rd_remote_done_int(di,
					dma_single_rw->imwr, ch_nr);
				dw_pcie_dma_clear_rd_done_int_mask(di,
					(1 << ch_nr));
				dw_pcie_dma_clear_rd_abort_int_mask(di,
					(1 << ch_nr));
			}
		}
	}
	/* Set transfer size */
	dw_pcie_dma_set_transfer_size(di, dma_single_rw->size, ch_nr, dir);
	/* Set SAR & DAR */
	dw_pcie_dma_set_sar(di, dma_single_rw->sar, ch_nr, dir);
	dw_pcie_dma_set_dar(di, dma_single_rw->dar, ch_nr, dir);

	if (flags & DMA_FLAG_WRITE_ELEM)
		dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DOORBELL, 0);
	else
		dw_pcie_writel_dma(di, PCIE_DMA_READ_DOORBELL, 0);

	return 0;
}

static void dw_pcie_dma_check_errors(struct dma_info *di,
	u32 direction, u32 *error)
{
	u32 val = 0;
	*error = DMA_ERR_NONE;

	if (direction == DMA_CH_WRITE) {
		val = dw_pcie_readl_dma(di, PCIE_DMA_WRITE_ERR_STATUS);
		if (val & 0x1)
			*error |= DMA_ERR_WR;
		if (val & 0x10000)
			*error |= DMA_ERR_FETCH_LL;
	} else {
		/* Get error status low */
		val = dw_pcie_readl_dma(di, PCIE_DMA_READ_ERR_STATUS_LOW);
		if (val & 0x1)
			*error |= DMA_ERR_RD;
		if (val & 0x10000)
			*error |= DMA_ERR_FETCH_LL;
		/* Get error status high */
		val = dw_pcie_readl_dma(di, PCIE_DMA_READ_ERR_STATUS_HIGH);
		if (val & 0x1)
			*error |= DMA_ERR_UNSUPPORTED_REQ;
		if (val & 0x100)
			*error |= DMA_ERR_CPL_ABORT;
		if (val & 0x10000)
			*error |= DMA_ERR_CPL_TIMEOUT;
		if (val & 0x1000000)
			*error |= DMA_ERR_DATA_POISIONING;
	}
}

/* Generic int handlers for DMA read and write (separate).
 * They should be called from the platform specific interrupt handler.
 */
u32 dw_handle_dma_irq_write(struct dma_info *di, u32 val_write)
{
	u32 err_type = DMA_ERR_NONE;

	if (val_write) {
		if (di->wr_ch.status == DMA_CH_RUNNING) {
			if (val_write & 0x10000) { /* Abort interrupt */
				/* Get error type */
				dw_pcie_dma_check_errors(di,
					DMA_FLAG_WRITE_ELEM,
					&di->wr_ch.errors);
				dw_pcie_writel_dma(di,
					PCIE_DMA_WRITE_INT_CLEAR,
					0x00FF0000);
				err_type = di->wr_ch.errors;
			} else { /* Done interrupt */
				dw_pcie_writel_dma(di,
					PCIE_DMA_WRITE_INT_CLEAR,
					0x000000FF);
				/* Check channel list mode */
			}
			di->wr_ch.status = DMA_CH_STOPPED;
		} else
			dw_pcie_writel_dma(di,
				PCIE_DMA_WRITE_INT_CLEAR, 0x00FF00FF);

#ifdef DMA_PTR_FUNC
		if (di->ptr_func)
			di->ptr_func(err_type);
#endif /* DMA_PTR_FUNC */
	}

	return err_type;
}

u32 dw_handle_dma_irq_read(struct dma_info *di, u32 val_read)
{
	u32 err_type = DMA_ERR_NONE;

	if (val_read) {
		if (di->rd_ch.status == DMA_CH_RUNNING) {
			/* Search interrupt type, abort or done */
			/* Abort interrupt */
			if (val_read & 0x80000) {
				/* Get error type */
				dw_pcie_dma_check_errors(di,
					DMA_FLAG_READ_ELEM,
					&di->rd_ch.errors);
				dw_pcie_writel_dma(di,
					PCIE_DMA_READ_INT_CLEAR,
					0x00FF0000);
				err_type = di->rd_ch.errors;
			} else { /* Done interrupt */
				dw_pcie_writel_dma(di,
					PCIE_DMA_READ_INT_CLEAR,
					0x000000FF);
				/* Check channel list mode */
			}
			di->rd_ch.status = DMA_CH_STOPPED;
		} else
			dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_CLEAR,
						0x00FF00FF);

#ifdef DMA_PTR_FUNC
		if (di->ptr_func)
			di->ptr_func(err_type);
#endif /* DMA_PTR_FUNC */
	}

	return err_type;
}

#if (defined(CONFIG_PCI_EPF_TEST))

int dw_pcie_ep_start_dma(struct dw_pcie_ep *ep, bool dir,
				 dma_addr_t src, dma_addr_t dst, u32 len,
				 struct completion *complete)
{
	struct dw_pcie *pcie = to_dw_pcie_from_ep(ep);
	struct dma_info *di = dw_get_dma_info(pcie);

	int ret = 0;
/* TODO: make channel configurable, or get automatically
 * the next one available.
 */
	struct dma_data_elem dma_single = {
		.ch_num = 0,
		.flags = (DMA_FLAG_WRITE_ELEM | DMA_FLAG_EN_DONE_INT |
				DMA_FLAG_LIE),
	};

	dev_dbg(pcie->dev, "%s\n", __func__);

	dma_single.size = len;
	dma_single.sar = src;
	dma_single.dar = dst;

	di->complete = complete;

	/* Test the DMA benchmark */
	ret = dw_pcie_dma_single_rw(di, &dma_single);
	return ret;
}
#endif
