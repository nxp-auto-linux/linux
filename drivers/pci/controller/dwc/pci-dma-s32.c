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

int dw_pcie_dma_write_en(struct dma_info *di)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ENGINE_EN, 0x1);
	return 0;
}

inline int dw_pcie_dma_get_nr_chan(struct dma_info *di)
{
	u32 dma_nr_ch = dw_pcie_readl_dma(di, PCIE_DMA_CTRL);

	dma_nr_ch = (dma_nr_ch & NUM_DMA_RD_CHAN_MASK)
					>> NUM_DMA_RD_CHAN_SHIFT;
	return dma_nr_ch;
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

static void dw_pcie_dma_set_wr_remote_done_int(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DONE_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DONE_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, PCIE_DMA_CH_CONTROL1,
		dw_pcie_readl_dma(di, PCIE_DMA_CH_CONTROL1) | 0x10);
}
static void dw_pcie_dma_set_wr_remote_abort_int(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ABORT_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ABORT_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, PCIE_DMA_CH_CONTROL1,
		dw_pcie_readl_dma(di, PCIE_DMA_CH_CONTROL1) | 0x10);
}
static void dw_pcie_dma_set_rd_remote_done_int(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_DONE_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_READ_DONE_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, PCIE_DMA_CH_CONTROL1,
		dw_pcie_readl_dma(di, PCIE_DMA_CH_CONTROL1) | 0x10);
}
static void dw_pcie_dma_set_rd_remote_abort_int(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ABORT_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ABORT_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, PCIE_DMA_CH_CONTROL1,
		dw_pcie_readl_dma(di, PCIE_DMA_CH_CONTROL1) | 0x10);
}

static void dw_pcie_dma_en_local_int(struct dma_info *di)
{
	dw_pcie_writel_dma(di, PCIE_DMA_CH_CONTROL1,
		dw_pcie_readl_dma(di, PCIE_DMA_CH_CONTROL1) | 0x8);
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

void dw_pcie_dma_set_sar(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_SAR_LOW, lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_SAR_HIGH, upper_32_bits(val));
}

void dw_pcie_dma_set_dar(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_DAR_LOW, lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_DAR_HIGH, upper_32_bits(val));
}

void dw_pcie_dma_set_transfer_size(struct dma_info *di, u32 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_TRANSFER_SIZE, val);
}

void dw_pcie_dma_set_rd_viewport(struct dma_info *di, u8 ch_nr)
{
	dw_pcie_writel_dma(di, PCIE_DMA_VIEWPORT_SEL, (1 << 31) | ch_nr);
}

void dw_pcie_dma_set_viewport(struct dma_info *di, u8 ch_nr, u8 direction)
{
	if (direction == DMA_CH_WRITE)
		dw_pcie_writel_dma(di, PCIE_DMA_VIEWPORT_SEL,
			(0 << 31) | ch_nr);
	else
		dw_pcie_writel_dma(di, PCIE_DMA_VIEWPORT_SEL,
			(1 << 31) | ch_nr);
}

void dw_pcie_dma_set_wr_viewport(struct dma_info *di, u8 ch_nr)
{
	dw_pcie_writel_dma(di, PCIE_DMA_VIEWPORT_SEL,
		(0 << 31) | ch_nr);
}

void dw_pcie_dma_clear_regs(struct dma_info *di)
{
	di->wr_ch.status = DMA_CH_STOPPED;
	di->rd_ch.status = DMA_CH_STOPPED;
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
	dw_pcie_writel_dma(di, PCIE_DMA_CH_CONTROL1, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_CH_CONTROL2, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_TRANSFER_SIZE, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_SAR_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_SAR_HIGH, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_DAR_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_DAR_HIGH, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_LLP_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_LLP_HIGH, 0);

}
void dw_pcie_dma_set_list_ptr(struct dma_info *di, u8 direction,
	u32 phy_list_addr)
{
	dw_pcie_writel_dma(di, PCIE_DMA_LLP_LOW,
				lower_32_bits(phy_list_addr));
	dw_pcie_writel_dma(di, PCIE_DMA_LLP_HIGH,
				upper_32_bits(phy_list_addr));
}

int dw_pcie_dma_single_rw(struct dma_info *di,
	struct dma_data_elem *dma_single_rw)
{
	u32 flags, dma_nr_ch;
	struct dma_ch_info *ptr_ch;

	/* Invalid transfer size */
	if (dma_single_rw->size > CONFIG_PCIE_DMA_MAX_SIZE)
		return -EINVAL;

	flags = dma_single_rw->flags;
	ptr_ch = (flags & DMA_FLAG_WRITE_ELEM) ?
		&di->wr_ch : &di->rd_ch;

	if (flags & DMA_FLAG_WRITE_ELEM) {

		dma_nr_ch = dw_pcie_dma_get_nr_chan(di);

		/* Invalid channel number */
		if (dma_single_rw->ch_num > dma_nr_ch - 1)
			return -EINVAL;

		if (di->wr_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		di->wr_ch.status = DMA_CH_RUNNING;
		di->wr_ch.errors = 0;
		dw_pcie_dma_write_en(di);
		dw_pcie_dma_set_viewport(di, dma_single_rw->ch_num,
			DMA_CH_WRITE);
	} else {

		dma_nr_ch = dw_pcie_dma_get_nr_chan(di);

		/* Invalid channel number */
		if (dma_single_rw->ch_num > dma_nr_ch - 1)
			return -EINVAL;

		if (di->rd_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		di->rd_ch.status = DMA_CH_RUNNING;
		di->rd_ch.errors = 0;
		dw_pcie_dma_read_en(di);

		dw_pcie_dma_set_viewport(di, dma_single_rw->ch_num,
			DMA_CH_READ);
	}

	/* Clear CR1 for proper init */
	dw_pcie_writel_dma(di, PCIE_DMA_CH_CONTROL1, 0);

	if (flags & (DMA_FLAG_EN_DONE_INT | DMA_FLAG_EN_ABORT_INT)) {
		dw_pcie_dma_en_local_int(di);

		if (flags & (DMA_FLAG_RIE | DMA_FLAG_LIE)) {
			if (flags & DMA_FLAG_WRITE_ELEM) {
				dw_pcie_dma_set_wr_remote_abort_int(di,
					dma_single_rw->imwr);
				dw_pcie_dma_set_wr_remote_done_int(di,
					dma_single_rw->imwr);
				dw_pcie_dma_clear_wr_done_int_mask(di,
					(1 << dma_single_rw->ch_num));
				dw_pcie_dma_clear_wr_abort_int_mask(di,
					(1 << dma_single_rw->ch_num));
			} else if (flags & DMA_FLAG_READ_ELEM) {
				dw_pcie_dma_set_rd_remote_abort_int(di,
					dma_single_rw->imwr);
				dw_pcie_dma_set_rd_remote_done_int(di,
					dma_single_rw->imwr);
				dw_pcie_dma_clear_rd_done_int_mask(di,
					(1 << dma_single_rw->ch_num));
				dw_pcie_dma_clear_rd_abort_int_mask(di,
					(1 << dma_single_rw->ch_num));
			}
		}
	}
	/* Set transfer size */
	dw_pcie_dma_set_transfer_size(di, dma_single_rw->size);
	/* Set SAR & DAR */
	dw_pcie_dma_set_sar(di, dma_single_rw->sar);
	dw_pcie_dma_set_dar(di, dma_single_rw->dar);

	if (flags & DMA_FLAG_WRITE_ELEM)
		dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DOORBELL, 0);
	else
		dw_pcie_writel_dma(di, PCIE_DMA_READ_DOORBELL, 0);

	return 0;
}

void dw_pcie_dma_check_errors(struct dma_info *di,
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
irqreturn_t dw_handle_dma_irq_write(struct dma_info *di, u32 val_write)
{
#ifdef DMA_PTR_FUNC
	u32 err_type = DMA_ERR_NONE;
#endif

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
#ifdef DMA_PTR_FUNC
				err_type = di->wr_ch.errors;
#endif
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

	return IRQ_HANDLED;
}

irqreturn_t dw_handle_dma_irq_read(struct dma_info *di, u32 val_read)
{
#ifdef DMA_PTR_FUNC
	u32 err_type = DMA_ERR_NONE;
#endif

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
#ifdef DMA_PTR_FUNC
				err_type = di->rd_ch.errors;
#endif
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

	return IRQ_HANDLED;
}
