// SPDX-License-Identifier: GPL-2.0
/*
 * DMA support for the Synopsys DesignWare
 * PCIe host controller driver, customized
 * for the NXP S32CC PCIe driver
 *
 * Copyright 2017-2020 NXP
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
#include "pci-dma-s32cc.h"

/* TODO: dw_pcie_*_rc functions should be removed,
 * and the new API used instead
 */
static inline void dw_pcie_writel_rc(struct dw_pcie *pci, u32 val, u32 reg)
{
	dw_pcie_writel_dbi(pci, reg, val);
}

int dw_pcie_dma_write_en(struct dw_pcie *pci)
{
	dw_pcie_writel_rc(pci, 0x1, PCIE_DMA_WRITE_ENGINE_EN);
	return 0;
}

int dw_pcie_dma_write_soft_reset(struct dw_pcie *pci, struct dma_info *di)
{
	dw_pcie_writel_rc(pci, 0x0, PCIE_DMA_WRITE_ENGINE_EN);
	while (dw_pcie_readl_dbi(pci, PCIE_DMA_WRITE_ENGINE_EN) == 1)
		;
	di->wr_ch.status = DMA_CH_STOPPED;
	dw_pcie_writel_rc(pci, 0x1, PCIE_DMA_WRITE_ENGINE_EN);
	return 0;
}

int dw_pcie_dma_read_en(struct dw_pcie *pci)
{
	dw_pcie_writel_rc(pci, 0x1, PCIE_DMA_READ_ENGINE_EN);
	return 0;
}

int dw_pcie_dma_read_soft_reset(struct dw_pcie *pci, struct dma_info *di)
{
	dw_pcie_writel_rc(pci, 0x0, PCIE_DMA_READ_ENGINE_EN);
	while (dw_pcie_readl_dbi(pci, PCIE_DMA_READ_ENGINE_EN) == 1)
		;
	di->rd_ch.status = DMA_CH_STOPPED;
	dw_pcie_writel_rc(pci, 0x1, PCIE_DMA_READ_ENGINE_EN);
	return 0;
}

void dw_pcie_dma_set_wr_remote_done_int(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, lower_32_bits(val),
		PCIE_DMA_WRITE_DONE_IMWR_LOW);
	dw_pcie_writel_rc(pci, upper_32_bits(val),
		PCIE_DMA_WRITE_DONE_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_CH_CONTROL1) |
				0x10, PCIE_DMA_CH_CONTROL1);
}

void dw_pcie_dma_set_wr_remote_abort_int(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, lower_32_bits(val),
		PCIE_DMA_WRITE_ABORT_IMWR_LOW);
	dw_pcie_writel_rc(pci, upper_32_bits(val),
		PCIE_DMA_WRITE_ABORT_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_CH_CONTROL1) |
				0x10, PCIE_DMA_CH_CONTROL1);
}

void dw_pcie_dma_set_rd_remote_done_int(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, lower_32_bits(val),
		PCIE_DMA_READ_DONE_IMWR_LOW);
	dw_pcie_writel_rc(pci, upper_32_bits(val),
		PCIE_DMA_READ_DONE_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_CH_CONTROL1) |
				0x10, PCIE_DMA_CH_CONTROL1);
}

void dw_pcie_dma_set_rd_remote_abort_int(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, lower_32_bits(val),
		PCIE_DMA_READ_ABORT_IMWR_LOW);
	dw_pcie_writel_rc(pci, upper_32_bits(val),
		PCIE_DMA_READ_ABORT_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_CH_CONTROL1) |
				0x10, PCIE_DMA_CH_CONTROL1);
}

void dw_pcie_dma_en_local_int(struct dw_pcie *pci)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_CH_CONTROL1) |
				0x8, PCIE_DMA_CH_CONTROL1);
}

/* Interrupts mask and clear functions */
int dw_pcie_dma_clear_wr_done_int_mask(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_WRITE_INT_MASK) &
				(~val), PCIE_DMA_WRITE_INT_MASK);
	return 0;
}

int dw_pcie_dma_clear_wr_abort_int_mask(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_WRITE_INT_MASK) &
				((~val) << 16), PCIE_DMA_WRITE_INT_MASK);
	return 0;
}

int dw_pcie_dma_clear_rd_done_int_mask(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_READ_INT_MASK) &
				(~val), PCIE_DMA_READ_INT_MASK);
	return 0;
}

int dw_pcie_dma_clear_rd_abort_int_mask(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_READ_INT_MASK) &
				((~val) << 16), PCIE_DMA_READ_INT_MASK);
	return 0;
}

int dw_pcie_dma_set_wr_done_int_mask(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_WRITE_INT_MASK) |
				val, PCIE_DMA_WRITE_INT_MASK);
	return 0;
}

int dw_pcie_dma_set_wr_abort_int_mask(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_WRITE_INT_MASK) |
				val << 16, PCIE_DMA_WRITE_INT_MASK);
	return 0;
}

int dw_pcie_dma_set_rd_done_int_mask(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_WRITE_INT_MASK) |
				val, PCIE_DMA_READ_INT_MASK);
	return 0;
}

int dw_pcie_dma_set_rd_abort_int_mask(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_WRITE_INT_MASK) |
				val << 16, PCIE_DMA_READ_INT_MASK);
	return 0;
}

int dw_pcie_dma_clear_wr_done_int(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci,
				PCIE_DMA_WRITE_INT_CLEAR) | val,
				PCIE_DMA_WRITE_INT_CLEAR);
	return 0;
}

int dw_pcie_dma_clear_wr_abort_int(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci,
				PCIE_DMA_WRITE_INT_CLEAR) | val << 16,
				PCIE_DMA_WRITE_INT_CLEAR);
	return 0;
}

int dw_pcie_dma_clear_rd_done_int(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_READ_INT_CLEAR) |
				val, PCIE_DMA_READ_INT_CLEAR);
	return 0;
}

int dw_pcie_dma_clear_rd_abort_int(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, dw_pcie_readl_dbi(pci, PCIE_DMA_READ_INT_CLEAR) |
				val << 16, PCIE_DMA_READ_INT_CLEAR);
	return 0;
}

void dw_pcie_dma_set_sar(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, lower_32_bits(val), PCIE_DMA_SAR_LOW);
	dw_pcie_writel_rc(pci, upper_32_bits(val), PCIE_DMA_SAR_HIGH);
}

void dw_pcie_dma_set_dar(struct dw_pcie *pci, u64 val)
{
	dw_pcie_writel_rc(pci, lower_32_bits(val), PCIE_DMA_DAR_LOW);
	dw_pcie_writel_rc(pci, upper_32_bits(val), PCIE_DMA_DAR_HIGH);
}

void dw_pcie_dma_set_transfer_size(struct dw_pcie *pci, u32 val)
{
	dw_pcie_writel_rc(pci, val, PCIE_DMA_TRANSFER_SIZE);
}

void dw_pcie_dma_set_rd_viewport(struct dw_pcie *pci, u8 ch_nr)
{
	dw_pcie_writel_rc(pci, (1 << 31) | ch_nr, PCIE_DMA_VIEWPORT_SEL);
}

void dw_pcie_dma_set_viewport(struct dw_pcie *pci, u8 ch_nr, u8 direction)
{
	if (direction == DMA_CH_WRITE)
		dw_pcie_writel_rc(pci, (0 << 31) | ch_nr,
				PCIE_DMA_VIEWPORT_SEL);
	else
		dw_pcie_writel_rc(pci, (1 << 31) | ch_nr,
				PCIE_DMA_VIEWPORT_SEL);
}

void dw_pcie_dma_set_wr_viewport(struct dw_pcie *pci, u8 ch_nr)
{
	dw_pcie_writel_rc(pci, (0 << 31) | ch_nr, PCIE_DMA_VIEWPORT_SEL);
}

void dw_pcie_dma_clear_regs(struct dw_pcie *pci, struct dma_info *di)
{
	di->wr_ch.status = DMA_CH_STOPPED;
	di->rd_ch.status = DMA_CH_STOPPED;
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_INT_MASK, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_INT_CLEAR, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_DONE_IMWR_LOW, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_DONE_IMWR_HIGH, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_ABORT_IMWR_LOW, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_ABORT_IMWR_HIGH, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_CH01_IMWR_DATA, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_CH23_IMWR_DATA, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_CH45_IMWR_DATA, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_CH67_IMWR_DATA, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_READ_INT_MASK, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_READ_INT_CLEAR, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_READ_DONE_IMWR_LOW, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_READ_DONE_IMWR_HIGH, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_READ_ABORT_IMWR_LOW, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_READ_ABORT_IMWR_HIGH, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_READ_CH01_IMWR_DATA, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_READ_CH23_IMWR_DATA, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_READ_CH45_IMWR_DATA, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_READ_CH67_IMWR_DATA, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_CH_CONTROL1, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_CH_CONTROL2, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_TRANSFER_SIZE, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_SAR_LOW, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_SAR_HIGH, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_DAR_LOW, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_DAR_HIGH, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_LLP_LOW, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_LLP_HIGH, 0);
}

void dw_pcie_dma_set_list_ptr(struct dw_pcie *pci, u8 direction,
	u32 phy_list_addr)
{
	dw_pcie_writel_dbi(pci, PCIE_DMA_LLP_LOW,
				lower_32_bits(phy_list_addr));
	dw_pcie_writel_dbi(pci, PCIE_DMA_LLP_HIGH,
				upper_32_bits(phy_list_addr));
}

static void dw_pcie_dma_set_link_elem(u32 *ptr_list_base,
	u8 arr_sz, u32 phy_list_addr)
{
	*(ptr_list_base + (arr_sz * 6) + 0x1) = 0;
	*(ptr_list_base + (arr_sz * 6) + 0x2) = lower_32_bits(phy_list_addr);
	*(ptr_list_base + (arr_sz * 6) + 0x3) = upper_32_bits(phy_list_addr);
	/* LLP | TCB | CB */
	*(ptr_list_base + (arr_sz * 6) + 0) = 0x5;
}

static void dw_pcie_dma_set_data_elem(u32 *ptr_list_base, u8 index,
	u64 sar, u64 dar, u32 size, u8 intr)
{
	u8 i = index;
	u8 ctrl_LIE = (intr) ? 0x8 : 0x0;

	*(ptr_list_base + (i * 6) + 0x1) = size;
	*(ptr_list_base + (i * 6) + 0x2) = lower_32_bits(sar);
	*(ptr_list_base + (i * 6) + 0x3) = upper_32_bits(sar);
	*(ptr_list_base + (i * 6) + 0x4) = lower_32_bits(dar);
	*(ptr_list_base + (i * 6) + 0x5) = upper_32_bits(dar);
	*(ptr_list_base + (i * 6) + 0) = ctrl_LIE | 0x1;
}

void dw_pcie_dma_start_llw(struct dw_pcie *pci, struct dma_info *di,
				u64 phy_list_addr)
{
	/* Program DMA regs for LL mode */
	dw_pcie_writel_dbi(pci, PCIE_DMA_CH_CONTROL1, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_ENGINE_EN, 1);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_INT_MASK, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_LINKED_LIST_ERR_EN, 0x10000);
	dw_pcie_writel_dbi(pci, PCIE_DMA_VIEWPORT_SEL, 0);
	dw_pcie_writel_dbi(pci, PCIE_DMA_CH_CONTROL1, 0x04000300);

	/* Set pointer to start of first list */
	dw_pcie_dma_set_list_ptr(pci, DMA_CH_WRITE, phy_list_addr);
	/* Ring doorbell */
	di->wr_ch.status = DMA_CH_RUNNING;
	dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_DOORBELL, 0);
}
EXPORT_SYMBOL(dw_pcie_dma_start_llw);

int dw_pcie_dma_start_linked_list(struct dw_pcie *pci, struct dma_info *di,
	u32 phy_list_addr,
	u8 direction)
{
	if (direction == DMA_CH_WRITE) {
		if (di->wr_ch.status != DMA_CH_RUNNING) {
			/* Set ch_status */
			di->wr_ch.status = DMA_CH_RUNNING;
			/* Set last data elem */
			dw_pcie_dma_set_data_elem((u32 *)di->wr_ch.virt_addr,
			di->wr_ch.current_list_size - 1,
			di->wr_ch.current_sar,
			di->wr_ch.current_dar, di->wr_ch.current_size, 1);

			/* Program DMA regs for LL mode */
			dw_pcie_writel_dbi(pci, PCIE_DMA_CH_CONTROL1, 0);
			di->wr_ch.phy_list_addr = phy_list_addr;
			dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_ENGINE_EN, 1);
			dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_INT_MASK, 0);
			dw_pcie_writel_dbi(pci,
				PCIE_DMA_WRITE_LINKED_LIST_ERR_EN, 0x10000);
			dw_pcie_writel_dbi(pci, PCIE_DMA_VIEWPORT_SEL, 0);
			dw_pcie_writel_dbi(pci,
				PCIE_DMA_CH_CONTROL1, 0x04000300);

			/* Set pointer to start of first list */
			dw_pcie_dma_set_list_ptr(pci, direction, phy_list_addr);
			/* Ring doorbell */
			dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_DOORBELL, 0);
		} else {
			return -EBUSY;
		}
	} else {/* Read request */
		if (di->rd_ch.status != DMA_CH_RUNNING) {
			/* Set ch_status */
			di->rd_ch.status = DMA_CH_RUNNING;
			/* Set last data elem */
			dw_pcie_dma_set_data_elem((u32 *)di->rd_ch.virt_addr,
			di->rd_ch.current_list_size - 1,
			di->rd_ch.current_sar,
			di->rd_ch.current_dar,  di->rd_ch.current_size, 1);

			di->rd_ch.errors = 0;
			/* Clear CR1 for proper init */
			dw_pcie_writel_dbi(pci, PCIE_DMA_CH_CONTROL1, 0);

			di->rd_ch.phy_list_addr = phy_list_addr;
			dw_pcie_writel_dbi(pci, PCIE_DMA_READ_ENGINE_EN, 1);
			dw_pcie_writel_dbi(pci, PCIE_DMA_READ_INT_MASK, 0);
			dw_pcie_writel_dbi(pci,
				PCIE_DMA_READ_LINKED_LIST_ERR_EN, 0x10000);
			dw_pcie_writel_dbi(pci,
				PCIE_DMA_VIEWPORT_SEL, 0x80000000);
			dw_pcie_writel_dbi(pci,
				PCIE_DMA_CH_CONTROL1, 0x04000300);

			/* Set pointer to start of first list */
			dw_pcie_dma_set_list_ptr(pci, direction, phy_list_addr);
			/* Ring doorbell */
			dw_pcie_writel_dbi(pci, PCIE_DMA_READ_DOORBELL, 0);
		} else {
			return -EBUSY;
		}
	}
	return 0;
}

int dw_pcie_dma_load_linked_list(struct dw_pcie *pci, struct dma_info *di,
	u8 arr_sz, u32 phy_list_addr,
	u32 next_phy_list_addr, u8 direction)
{
	u8 i;
	u32 *ptr_list;
	struct dma_list *arr_ll;

	struct dma_ch_info *ptr_ch = (direction == DMA_CH_WRITE) ?
		&di->wr_ch : &di->rd_ch;
	ptr_list = (u32 *)(ioremap(phy_list_addr, SZ_1K));
	if (!ptr_list)
		return -EFAULT;

	if (!di->dma_linked_list)
		return -EINVAL;
	arr_ll = *di->dma_linked_list;

	arr_sz = di->ll_info.nr_elem;
	di->ll_info.phy_list_addr = phy_list_addr;

	for (i = 0 ; i < arr_sz ; i++) {
		dw_pcie_dma_set_data_elem((u32 *)ptr_list, i,
			arr_ll[i].sar, arr_ll[i].dar,
			arr_ll[i].size, 0);
		ptr_ch->current_sar = arr_ll[i].sar;
		ptr_ch->current_dar = arr_ll[i].dar;
		ptr_ch->current_size = arr_ll[i].size;
		ptr_ch->current_elem_idx = i;
		ptr_ch->current_list_size = arr_sz;
		ptr_ch->virt_addr = ptr_list;
	}

	dw_pcie_dma_set_link_elem(ptr_list, arr_sz, next_phy_list_addr);

	iounmap(ptr_list);
	return 0;
}

int dw_pcie_dma_single_rw(struct dw_pcie *pci, struct dma_info *di,
	struct dma_data_elem *dma_single_rw)
{
	u32 flags;
	struct dma_ch_info *ptr_ch;

	/* Invalid channel number */
	if (dma_single_rw->ch_num > PCIE_DMA_NR_CH - 1)
		return -EINVAL;

	/* Invalid transfer size */
	if (dma_single_rw->size > PCIE_DMA_MAX_SIZE)
		return -EINVAL;

	flags = dma_single_rw->flags;
	ptr_ch = (flags & DMA_FLAG_WRITE_ELEM) ?
		&di->wr_ch : &di->rd_ch;

	if (flags & DMA_FLAG_WRITE_ELEM) {
		if (di->wr_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		di->wr_ch.status = DMA_CH_RUNNING;
		di->wr_ch.errors = 0;
		dw_pcie_dma_write_en(pci);
		dw_pcie_dma_set_viewport(pci, dma_single_rw->ch_num,
			DMA_CH_WRITE);
	} else {
		if (di->rd_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		di->rd_ch.status = DMA_CH_RUNNING;
		di->rd_ch.errors = 0;
		dw_pcie_dma_read_en(pci);

		dw_pcie_dma_set_viewport(pci, dma_single_rw->ch_num,
			DMA_CH_READ);
	}

	/* Clear CR1 for proper init */
	dw_pcie_writel_dbi(pci, PCIE_DMA_CH_CONTROL1, 0);

	if (flags & (DMA_FLAG_EN_DONE_INT | DMA_FLAG_EN_ABORT_INT)) {
		dw_pcie_dma_en_local_int(pci);

		if (flags & (DMA_FLAG_RIE | DMA_FLAG_LIE)) {
			if (flags & DMA_FLAG_WRITE_ELEM) {
				dw_pcie_dma_set_wr_remote_abort_int(pci,
					dma_single_rw->imwr);
				dw_pcie_dma_set_wr_remote_done_int(pci,
					dma_single_rw->imwr);
				dw_pcie_dma_clear_wr_done_int_mask(pci,
					(1 << dma_single_rw->ch_num));
				dw_pcie_dma_clear_wr_abort_int_mask(pci,
					(1 << dma_single_rw->ch_num));
			} else if (flags & DMA_FLAG_READ_ELEM) {
				dw_pcie_dma_set_rd_remote_abort_int(pci,
					dma_single_rw->imwr);
				dw_pcie_dma_set_rd_remote_done_int(pci,
					dma_single_rw->imwr);
				dw_pcie_dma_clear_rd_done_int_mask(pci,
					(1 << dma_single_rw->ch_num));
				dw_pcie_dma_clear_rd_abort_int_mask(pci,
					(1 << dma_single_rw->ch_num));
			}
		}
	}
	/* Set transfer size */
	dw_pcie_dma_set_transfer_size(pci, dma_single_rw->size);
	/* Set SAR & DAR */
	dw_pcie_dma_set_sar(pci, dma_single_rw->sar);
	dw_pcie_dma_set_dar(pci, dma_single_rw->dar);

	if (flags & DMA_FLAG_WRITE_ELEM)
		dw_pcie_writel_dbi(pci, PCIE_DMA_WRITE_DOORBELL, 0);
	else
		dw_pcie_writel_dbi(pci, PCIE_DMA_READ_DOORBELL, 0);

	return 0;
}

void dw_pcie_dma_check_errors(struct dw_pcie *pci,
	u32 direction, u32 *error)
{
	u32 val = 0;
	*error = DMA_ERR_NONE;

	if (direction == DMA_CH_WRITE) {
		val = dw_pcie_readl_dbi(pci, PCIE_DMA_WRITE_ERR_STATUS);
		if (val & 0x1)
			*error |= DMA_ERR_WR;
		if (val & 0x10000)
			*error |= DMA_ERR_FETCH_LL;
	} else {
		/* Get error status low */
		val = dw_pcie_readl_dbi(pci, PCIE_DMA_READ_ERR_STATUS_LOW);
		if (val & 0x1)
			*error |= DMA_ERR_RD;
		if (val & 0x10000)
			*error |= DMA_ERR_FETCH_LL;
		/* Get error status high */
		val = dw_pcie_readl_dbi(pci, PCIE_DMA_READ_ERR_STATUS_HIGH);
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
irqreturn_t dw_handle_dma_irq_write(struct dw_pcie *pci, struct dma_info *di,
					u32 val_write)
{
#ifdef DMA_PTR_FUNC
	u32 err_type = DMA_ERR_NONE;
#endif

	if (val_write) {
		if (di->wr_ch.status == DMA_CH_RUNNING) {
			if (val_write & 0x10000) { /* Abort interrupt */
				/* Get error type */
				dw_pcie_dma_check_errors(pci,
					DMA_FLAG_WRITE_ELEM,
					&di->wr_ch.errors);
				dw_pcie_writel_dbi(pci,
					PCIE_DMA_WRITE_INT_CLEAR,
					0x00FF0000);
#ifdef DMA_PTR_FUNC
				err_type = di->wr_ch.errors;
#endif
			} else { /* Done interrupt */
				dw_pcie_writel_dbi(pci,
					PCIE_DMA_WRITE_INT_CLEAR,
					0x000000FF);
				/* Check channel list mode */
			}
			di->wr_ch.status = DMA_CH_STOPPED;
		} else
			dw_pcie_writel_dbi(pci,
				PCIE_DMA_WRITE_INT_CLEAR, 0x00FF00FF);

#ifdef DMA_PTR_FUNC
		if (di->ptr_func)
			di->ptr_func(err_type);
#endif /* DMA_PTR_FUNC */
	}

	return IRQ_HANDLED;
}

irqreturn_t dw_handle_dma_irq_read(struct dw_pcie *pci, struct dma_info *di,
					u32 val_read)
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
				dw_pcie_dma_check_errors(pci,
					DMA_FLAG_READ_ELEM,
					&di->rd_ch.errors);
				dw_pcie_writel_dbi(pci,
					PCIE_DMA_READ_INT_CLEAR,
					0x00FF0000);
#ifdef DMA_PTR_FUNC
				err_type = pp->rd_ch.errors;
#endif
			} else { /* Done interrupt */
				dw_pcie_writel_dbi(pci,
					PCIE_DMA_READ_INT_CLEAR,
					0x000000FF);
				/* Check channel list mode */
			}
			di->rd_ch.status = DMA_CH_STOPPED;
		} else {
			dw_pcie_writel_dbi(pci, PCIE_DMA_READ_INT_CLEAR,
						0x00FF00FF);
		}

#ifdef DMA_PTR_FUNC
		if (di->ptr_func)
			di->ptr_func(err_type);
#endif /* DMA_PTR_FUNC */
	}

	return IRQ_HANDLED;
}
