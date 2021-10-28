// SPDX-License-Identifier: GPL-2.0
/*
 * DMA support for the Synopsys DesignWare
 * PCIe host controller driver, customized
 * for the NXP S32V PCIE driver
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
	u8 ch;
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ENGINE_EN, 0x0);
	while (dw_pcie_readl_dma(di, PCIE_DMA_WRITE_ENGINE_EN) == 1)
		;
	for (ch = 0; ch < PCIE_DMA_NR_CH; ch++)
		di->wr_ch[ch].status = DMA_CH_STOPPED;
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
	u8 ch;
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ENGINE_EN, 0x0);
	while (dw_pcie_readl_dma(di, PCIE_DMA_READ_ENGINE_EN) == 1)
		;
	for (ch = 0; ch < PCIE_DMA_NR_CH; ch++)
		di->rd_ch[ch].status = DMA_CH_STOPPED;
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ENGINE_EN, 0x1);
	return 0;
}

void dw_pcie_dma_set_wr_remote_done_int(struct dma_info *di, u64 val, u8 ch_nr)
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

void dw_pcie_dma_set_wr_remote_abort_int(struct dma_info *di, u64 val, u8 ch_nr)
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

void dw_pcie_dma_set_rd_remote_done_int(struct dma_info *di, u64 val, u8 ch_nr)
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

void dw_pcie_dma_set_rd_remote_abort_int(struct dma_info *di, u64 val, u8 ch_nr)
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

void dw_pcie_dma_en_local_int(struct dma_info *di, u8 ch_nr, u8 dir)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x8);
}
/* Interrupts mask and clear functions */
int dw_pcie_dma_clear_wr_done_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_MASK) & (~val));
	return 0;
}
int dw_pcie_dma_clear_wr_abort_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_MASK) &
				((~val) << 16));
	return 0;
}
int dw_pcie_dma_clear_rd_done_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_READ_INT_MASK) & (~val));
	return 0;
}
int dw_pcie_dma_clear_rd_abort_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_READ_INT_MASK) &
			((~val) << 16));
	return 0;
}

int dw_pcie_dma_set_wr_done_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_MASK) | val);
	return 0;
}
int dw_pcie_dma_set_wr_abort_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_MASK) | val << 16);
	return 0;
}

int dw_pcie_dma_set_rd_done_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_MASK) | val);
	return 0;
}
int dw_pcie_dma_set_rd_abort_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_MASK) | val << 16);
	return 0;
}

int dw_pcie_dma_clear_wr_done_int(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, dw_pcie_readl_dma(di,
				PCIE_DMA_WRITE_INT_CLEAR) | val,
				PCIE_DMA_WRITE_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_wr_abort_int(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, dw_pcie_readl_dma(di,
				PCIE_DMA_WRITE_INT_CLEAR) | val << 16,
				PCIE_DMA_WRITE_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_rd_done_int(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_CLEAR,
		dw_pcie_readl_dma(di, PCIE_DMA_READ_INT_CLEAR) | val);
	return 0;
}
int dw_pcie_dma_clear_rd_abort_int(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_CLEAR,
		dw_pcie_readl_dma(di, PCIE_DMA_READ_INT_CLEAR) | val << 16);
	return 0;
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
	u8 ch = 0, ch_max = 0;
	u8 dir = DMA_CH_WRITE;

	if (di->iatu_unroll_enabled)
		ch_max = PCIE_DMA_NR_CH - 1;

	for (ch = 0; ch <= ch_max; ch++) {
		di->wr_ch[ch].status = DMA_CH_STOPPED;
		di->rd_ch[ch].status = DMA_CH_STOPPED;
		for (dir = DMA_CH_WRITE; dir <= DMA_CH_READ; dir++) {
			u32 ch_base = dw_pcie_get_dma_channel_base(di,
				ch, dir);

			dw_pcie_writel_dma(di,
				ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0);
			dw_pcie_writel_dma(di,
				ch_base + PCIE_DMA_CH_CONTROL2_OFF, 0);
			dw_pcie_writel_dma(di,
				ch_base + PCIE_DMA_TRANSFER_SIZE_OFF, 0);
			dw_pcie_writel_dma(di,
				ch_base + PCIE_DMA_SAR_LOW_OFF, 0);
			dw_pcie_writel_dma(di,
				ch_base + PCIE_DMA_SAR_HIGH_OFF, 0);
			dw_pcie_writel_dma(di,
				ch_base + PCIE_DMA_DAR_LOW_OFF, 0);
			dw_pcie_writel_dma(di,
				ch_base + PCIE_DMA_DAR_HIGH_OFF, 0);
			dw_pcie_writel_dma(di,
				ch_base + PCIE_DMA_LLP_LOW_OFF, 0);
			dw_pcie_writel_dma(di,
				ch_base + PCIE_DMA_LLP_HIGH_OFF, 0);
		}
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

static void dw_pcie_dma_set_list_ptr(struct dma_info *di, u32 phy_list_addr,
	u8 ch_nr, u8 dir)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_LLP_LOW_OFF,
				lower_32_bits(phy_list_addr));
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_LLP_HIGH_OFF,
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

void dw_pcie_dma_start_llw(struct dma_info *di, u64 phy_list_addr)
{
	u8 ch_nr = di->ll_info.ch_num;
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, DMA_CH_WRITE);

	/* Program DMA regs for LL mode */
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ENGINE_EN, 1);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_LINKED_LIST_ERR_EN, 0x10000);

	if (!di->iatu_unroll_enabled)
		dw_pcie_writel_dma(di, PCIE_DMA_VIEWPORT_SEL, ch_nr);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0x04000300);

	/* Set pointer to start of first list */
	dw_pcie_dma_set_list_ptr(di, phy_list_addr, ch_nr, DMA_CH_WRITE);
	/* Ring doorbell */
	di->wr_ch[ch_nr].status = DMA_CH_RUNNING;
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DOORBELL, 0);
}
EXPORT_SYMBOL(dw_pcie_dma_start_llw);

static int dw_pcie_dma_start_ll_write(struct dma_info *di, u32 phy_list_addr)
{
	u8 ch_nr = di->ll_info.ch_num;

	if (di->wr_ch[ch_nr].status != DMA_CH_RUNNING) {
		u32 ch_base = dw_pcie_get_dma_channel_base(di,
			ch_nr, DMA_CH_WRITE);

		/* Set ch_status */
		di->wr_ch[ch_nr].status = DMA_CH_RUNNING;
		/* Set last data elem */
		dw_pcie_dma_set_data_elem((u32 *)di->wr_ch[ch_nr].virt_addr,
		di->wr_ch[ch_nr].current_list_size - 1,
		di->wr_ch[ch_nr].current_sar,
		di->wr_ch[ch_nr].current_dar, di->wr_ch[ch_nr].current_size, 1);

		/* Program DMA regs for LL mode */
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0);
		di->wr_ch[ch_nr].phy_list_addr = phy_list_addr;
		dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ENGINE_EN, 1);
		dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK, 0);
		dw_pcie_writel_dma(di,
			PCIE_DMA_WRITE_LINKED_LIST_ERR_EN, 0x10000);
		if (!di->iatu_unroll_enabled)
			dw_pcie_writel_dma(di, PCIE_DMA_VIEWPORT_SEL, ch_nr);
		dw_pcie_writel_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0x04000300);

		/* Set pointer to start of first list */
		dw_pcie_dma_set_list_ptr(di, phy_list_addr,
			ch_nr, DMA_CH_WRITE);
		/* Ring doorbell */
		dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DOORBELL, 0);
	} else {
		return -EBUSY;
	}

	return 0;
}

static int dw_pcie_dma_start_ll_read(struct dma_info *di, u32 phy_list_addr)
{
	u8 ch_nr = di->ll_info.ch_num;

	if (di->rd_ch[ch_nr].status != DMA_CH_RUNNING) {
		u32 ch_base = dw_pcie_get_dma_channel_base(di,
			ch_nr, DMA_CH_READ);

		/* Set ch_status */
		di->rd_ch[ch_nr].status = DMA_CH_RUNNING;
		/* Set last data elem */
		dw_pcie_dma_set_data_elem((u32 *)di->rd_ch[ch_nr].virt_addr,
			di->rd_ch[ch_nr].current_list_size - 1,
			di->rd_ch[ch_nr].current_sar,
			di->rd_ch[ch_nr].current_dar,
			di->rd_ch[ch_nr].current_size, 1);

		di->rd_ch[ch_nr].errors = 0;
		/* Clear CR1 for proper init */
		dw_pcie_writel_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0);

		di->rd_ch[ch_nr].phy_list_addr = phy_list_addr;
		dw_pcie_writel_dma(di, PCIE_DMA_READ_ENGINE_EN, 1);
		dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK, 0);
		dw_pcie_writel_dma(di,
			PCIE_DMA_READ_LINKED_LIST_ERR_EN, 0x10000);
		if (!di->iatu_unroll_enabled)
			dw_pcie_writel_dma(di,
				PCIE_DMA_VIEWPORT_SEL, 0x80000000 | ch_nr);
		dw_pcie_writel_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0x04000300);

		/* Set pointer to start of first list */
		dw_pcie_dma_set_list_ptr(di, phy_list_addr,
			ch_nr, DMA_CH_READ);
		/* Ring doorbell */
		dw_pcie_writel_dma(di, PCIE_DMA_READ_DOORBELL, 0);
	} else {
		return -EBUSY;
	}

	return 0;
}

int dw_pcie_dma_start_linked_list(struct dma_info *di,
	u32 phy_list_addr)
{
	u8 direction = di->ll_info.direction;
	if (direction == DMA_CH_WRITE)
		return dw_pcie_dma_start_ll_write(di, phy_list_addr);

	/* else,  Read request */
	return dw_pcie_dma_start_ll_read(di, phy_list_addr);
}

/* direction, channel and number of elements come from ll_info */
int dw_pcie_dma_load_linked_list(struct dma_info *di,
	u8 arr_sz, u32 phy_list_addr,
	u32 next_phy_list_addr)
{
	u8 i;
	u32 *ptr_list;
	struct dma_list *arr_ll;
	u8 ch_nr = di->ll_info.ch_num, dir = di->ll_info.direction;

	struct dma_ch_info *ptr_ch = (dir == DMA_CH_WRITE) ?
		&di->wr_ch[ch_nr] : &di->rd_ch[ch_nr];
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

int dw_pcie_dma_single_rw(struct dma_info *di,
	struct dma_data_elem *dma_single_rw)
{
	u32 flags;
	struct dma_ch_info *ptr_ch;
	u8 ch_nr = dma_single_rw->ch_num;
	u8 dir;
	u32 ch_base;

	/* Invalid channel number */
	if (ch_nr > PCIE_DMA_NR_CH - 1)
		return -EINVAL;

	/* Invalid transfer size */
	if (dma_single_rw->size > PCIE_DMA_MAX_SIZE)
		return -EINVAL;

	flags = dma_single_rw->flags;
	ptr_ch = (flags & DMA_FLAG_WRITE_ELEM) ?
		&di->wr_ch[ch_nr] : &di->rd_ch[ch_nr];

	if (flags & DMA_FLAG_WRITE_ELEM) {
		if (di->wr_ch[ch_nr].status == DMA_CH_RUNNING)
			return -EBUSY;

		di->wr_ch[ch_nr].status = DMA_CH_RUNNING;
		di->wr_ch[ch_nr].errors = 0;
		dw_pcie_dma_write_en(di);
		dir = DMA_CH_WRITE;
	} else {
		if (di->rd_ch[ch_nr].status == DMA_CH_RUNNING)
			return -EBUSY;

		di->rd_ch[ch_nr].status = DMA_CH_RUNNING;
		di->rd_ch[ch_nr].errors = 0;
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
u32 dw_handle_dma_irq_write(struct dma_info *di, u8 ch, u32 val_write)
{
	u32 err_type = DMA_ERR_NONE;

	if (val_write) {
		if (di->wr_ch[ch].status == DMA_CH_RUNNING) {
			/* Search interrupt type, abort or done */
			/* Abort interrupt */
			u8 int_bit = 1 << ch;

			if (val_write & 0x10000) {
				/* Get error type */
				dw_pcie_dma_check_errors(di,
					DMA_FLAG_WRITE_ELEM,
					&di->wr_ch[ch].errors);
				dw_pcie_writel_dma(di,
					PCIE_DMA_WRITE_INT_CLEAR,
					int_bit << 16);
				/* Hold only the first error for now */
				if ((err_type == DMA_ERR_NONE) &&
					(di->wr_ch[ch].errors !=
						DMA_ERR_NONE)) {
					err_type = di->wr_ch[ch].errors;
				}
			} else { /* Done interrupt */
				dw_pcie_writel_dma(di,
					PCIE_DMA_WRITE_INT_CLEAR,
					int_bit);
				/* Check channel list mode */
			}
			di->wr_ch[ch].status = DMA_CH_STOPPED;
		}

#ifdef DMA_PTR_FUNC
		if (di->ptr_func)
			di->ptr_func(err_type);
#endif /* DMA_PTR_FUNC */
	}

	return err_type;
}
u32 dw_handle_dma_irq_read(struct dma_info *di, u8 ch, u32 val_read)
{
	u32 err_type = DMA_ERR_NONE;

	if (val_read) {
		if (di->rd_ch[ch].status == DMA_CH_RUNNING) {
			/* Search interrupt type, abort or done */
			/* Abort interrupt */
			u8 int_bit = 1 << ch;
			if (val_read & 0x80000) {
				/* Get error type */
				dw_pcie_dma_check_errors(di,
					DMA_FLAG_READ_ELEM,
					&di->rd_ch[ch].errors);
				dw_pcie_writel_dma(di,
					PCIE_DMA_READ_INT_CLEAR,
					int_bit << 16);
				if ((err_type == DMA_ERR_NONE) &&
					(di->rd_ch[ch].errors !=
						DMA_ERR_NONE)) {
					err_type = di->rd_ch[ch].errors;
				}
			} else { /* Done interrupt */
				dw_pcie_writel_dma(di,
					PCIE_DMA_READ_INT_CLEAR,
					int_bit);
				/* Check channel list mode */
			}
			di->rd_ch[ch].status = DMA_CH_STOPPED;
		}

#ifdef DMA_PTR_FUNC
		if (di->ptr_func)
			di->ptr_func(err_type);
#endif /* DMA_PTR_FUNC */
	}

	return err_type;
}
