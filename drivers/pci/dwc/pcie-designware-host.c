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

#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>

#include "pcie-designware.h"

static struct pci_ops dw_pcie_ops;

static int dw_pcie_rd_own_conf(struct pcie_port *pp, int where, int size,
			       u32 *val)
{
	struct dw_pcie *pci;

	if (pp->ops->rd_own_conf)
		return pp->ops->rd_own_conf(pp, where, size, val);

	pci = to_dw_pcie_from_pp(pp);
	return dw_pcie_read(pci->dbi_base + where, size, val);
}

static int dw_pcie_wr_own_conf(struct pcie_port *pp, int where, int size,
			       u32 val)
{
	struct dw_pcie *pci;

	if (pp->ops->wr_own_conf)
		return pp->ops->wr_own_conf(pp, where, size, val);

	pci = to_dw_pcie_from_pp(pp);
	return dw_pcie_write(pci->dbi_base + where, size, val);
}

#ifdef CONFIG_PCI_DW_DMA
/* TODO: dw_pcie_*_rc functions should be removed,
 * and the new API used instead
 * TODO: dma related code should be moved to new files,
 * and pcie-designware*.* code cleaned up
 */
static inline void dw_pcie_readl_rc(struct pcie_port *pp, u32 reg, u32 *val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	*val = dw_pcie_readl_dbi(pci, PCI_INTERRUPT_LINE);
}
static inline void dw_pcie_writel_rc(struct pcie_port *pp, u32 val, u32 reg)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_dbi(pci, reg, val);
}
int dw_pcie_dma_write_en(struct pcie_port *pp)
{
	dw_pcie_writel_rc(pp, 0x1, PCIE_DMA_WRITE_ENGINE_EN);
	return 0;
}
int dw_pcie_dma_write_soft_reset(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, 0x0, PCIE_DMA_WRITE_ENGINE_EN);
	while (readl(pci->dbi_base + PCIE_DMA_WRITE_ENGINE_EN) == 1)
		;
	pp->wr_ch.status = DMA_CH_STOPPED;
	dw_pcie_writel_rc(pp, 0x1, PCIE_DMA_WRITE_ENGINE_EN);
	return 0;
}
int dw_pcie_dma_read_en(struct pcie_port *pp)
{
	dw_pcie_writel_rc(pp, 0x1, PCIE_DMA_READ_ENGINE_EN);
	return 0;
}
int dw_pcie_dma_read_soft_reset(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, 0x0, PCIE_DMA_READ_ENGINE_EN);
	while (readl(pci->dbi_base + PCIE_DMA_READ_ENGINE_EN) == 1)
		;
	pp->rd_ch.status = DMA_CH_STOPPED;
	dw_pcie_writel_rc(pp, 0x1, PCIE_DMA_READ_ENGINE_EN);
	return 0;
}
void dw_pcie_dma_set_wr_remote_done_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_WRITE_DONE_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_WRITE_DONE_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_set_wr_remote_abort_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_WRITE_ABORT_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_WRITE_ABORT_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_set_rd_remote_done_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_READ_DONE_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_READ_DONE_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_set_rd_remote_abort_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_READ_ABORT_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_READ_ABORT_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_en_local_int(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x8,
		PCIE_DMA_CH_CONTROL1);
}
/* Interrupts mask and clear functions */
int dw_pcie_dma_clear_wr_done_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) & (~val),
		PCIE_DMA_WRITE_INT_MASK);
	return 0;
}
int dw_pcie_dma_clear_wr_abort_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) & ((~val)<<16),
		PCIE_DMA_WRITE_INT_MASK);
	return 0;
}
int dw_pcie_dma_clear_rd_done_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_READ_INT_MASK) & (~val),
		PCIE_DMA_READ_INT_MASK);
	return 0;
}
int dw_pcie_dma_clear_rd_abort_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_READ_INT_MASK) & ((~val)<<16),
		PCIE_DMA_READ_INT_MASK);
	return 0;
}

int dw_pcie_dma_set_wr_done_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val, PCIE_DMA_WRITE_INT_MASK);
	return 0;
}
int dw_pcie_dma_set_wr_abort_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val<<16, PCIE_DMA_WRITE_INT_MASK);
	return 0;
}

int dw_pcie_dma_set_rd_done_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val, PCIE_DMA_READ_INT_MASK);
	return 0;
}
int dw_pcie_dma_set_rd_abort_int_mask(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val<<16, PCIE_DMA_READ_INT_MASK);
	return 0;
}

int dw_pcie_dma_clear_wr_done_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_CLEAR) | val, PCIE_DMA_WRITE_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_wr_abort_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_WRITE_INT_CLEAR) | val<<16, PCIE_DMA_WRITE_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_rd_done_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_READ_INT_CLEAR) | val, PCIE_DMA_READ_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_rd_abort_int(struct pcie_port *pp, u64 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_writel_rc(pp, readl(pci->dbi_base +
		PCIE_DMA_READ_INT_CLEAR) | val<<16, PCIE_DMA_READ_INT_CLEAR);
	return 0;
}
void dw_pcie_dma_set_sar(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, lower_32_bits(val), PCIE_DMA_SAR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val), PCIE_DMA_SAR_HIGH);
}
void dw_pcie_dma_set_dar(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, lower_32_bits(val), PCIE_DMA_DAR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val), PCIE_DMA_DAR_HIGH);
}
void dw_pcie_dma_set_transfer_size(struct pcie_port *pp, u32 val)
{
	dw_pcie_writel_rc(pp, val, PCIE_DMA_TRANSFER_SIZE);
}

void dw_pcie_dma_set_rd_viewport(struct pcie_port *pp, u8 ch_nr)
{
	dw_pcie_writel_rc(pp, (1<<31) | ch_nr, PCIE_DMA_VIEWPORT_SEL);
}
void dw_pcie_dma_set_viewport(struct pcie_port *pp, u8 ch_nr, u8 direction)
{
	if (direction == DMA_CH_WRITE)
		dw_pcie_writel_rc(pp, (0<<31) | ch_nr, PCIE_DMA_VIEWPORT_SEL);
	else
		dw_pcie_writel_rc(pp, (1<<31) | ch_nr, PCIE_DMA_VIEWPORT_SEL);
}
void dw_pcie_dma_set_wr_viewport(struct pcie_port *pp, u8 ch_nr)
{
	dw_pcie_writel_rc(pp, (0<<31) | ch_nr, PCIE_DMA_VIEWPORT_SEL);
}
void dw_pcie_dma_clear_regs(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	pp->wr_ch.status = DMA_CH_STOPPED;
	pp->rd_ch.status = DMA_CH_STOPPED;
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_INT_MASK);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_INT_CLEAR);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_DONE_IMWR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_DONE_IMWR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_ABORT_IMWR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_ABORT_IMWR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_CH01_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_CH23_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_CH45_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_CH67_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_READ_INT_MASK);
	writel(0, pci->dbi_base + PCIE_DMA_READ_INT_CLEAR);
	writel(0, pci->dbi_base + PCIE_DMA_READ_DONE_IMWR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_READ_DONE_IMWR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_READ_ABORT_IMWR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_READ_ABORT_IMWR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_READ_CH01_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_READ_CH23_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_READ_CH45_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_READ_CH67_IMWR_DATA);
	writel(0, pci->dbi_base + PCIE_DMA_CH_CONTROL1);
	writel(0, pci->dbi_base + PCIE_DMA_CH_CONTROL2);
	writel(0, pci->dbi_base + PCIE_DMA_TRANSFER_SIZE);
	writel(0, pci->dbi_base + PCIE_DMA_SAR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_SAR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_DAR_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_DAR_HIGH);
	writel(0, pci->dbi_base + PCIE_DMA_LLP_LOW);
	writel(0, pci->dbi_base + PCIE_DMA_LLP_HIGH);

}
void dma_set_list_ptr(struct pcie_port *pp, u8 direction,
	u32 phy_list_addr)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	writel(lower_32_bits(phy_list_addr), pci->dbi_base + PCIE_DMA_LLP_LOW);
	writel(upper_32_bits(phy_list_addr), pci->dbi_base + PCIE_DMA_LLP_HIGH);
}
static void dma_set_link_elem(u32 *ptr_list_base,
	u8 arr_sz, u32 phy_list_addr)
{
	*(ptr_list_base + (arr_sz * 6) + 0x1) = 0;
	*(ptr_list_base + (arr_sz * 6) + 0x2) = lower_32_bits(phy_list_addr);
	*(ptr_list_base + (arr_sz * 6) + 0x3) = upper_32_bits(phy_list_addr);
	/* LLP | TCB | CB */
	*(ptr_list_base + (arr_sz * 6) + 0) = 0x5;
}

static void dma_set_data_elem(u32 *ptr_list_base, u8 index,
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

void dw_start_dma_llw(struct pcie_port *pp, u64 phy_list_addr)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	/* Program DMA regs for LL mode */
	writel(0x0, pci->dbi_base + PCIE_DMA_CH_CONTROL1);
	writel(0x1, pci->dbi_base + PCIE_DMA_WRITE_ENGINE_EN);
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_INT_MASK);
	writel(0x10000, pci->dbi_base +
		PCIE_DMA_WRITE_LINKED_LIST_ERR_EN);
	writel(0, pci->dbi_base + PCIE_DMA_VIEWPORT_SEL);
	writel(0x04000300, pci->dbi_base + PCIE_DMA_CH_CONTROL1);

	/* Set pointer to start of first list */
	dma_set_list_ptr(pp, DMA_CH_WRITE, phy_list_addr);
	/* Ring doorbell */
	pp->wr_ch.status = DMA_CH_RUNNING;
	writel(0, pci->dbi_base + PCIE_DMA_WRITE_DOORBELL);
}
EXPORT_SYMBOL(dw_start_dma_llw);

int dw_pcie_dma_start_linked_list(struct pcie_port *pp,
	u32 phy_list_addr,
	u8 direction)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (direction == DMA_CH_WRITE) {
		if (pp->wr_ch.status != DMA_CH_RUNNING) {
			/* Set ch_status */
			pp->wr_ch.status = DMA_CH_RUNNING;
			/* Set last data elem */
			dma_set_data_elem((u32 *)pp->wr_ch.virt_addr,
			pp->wr_ch.current_list_size - 1,
			pp->wr_ch.current_sar,
			pp->wr_ch.current_dar, pp->wr_ch.current_size, 1);

			/* Program DMA regs for LL mode */
			writel(0x0, pci->dbi_base + PCIE_DMA_CH_CONTROL1);
			pp->wr_ch.phy_list_addr = phy_list_addr;
			writel(0x1, pci->dbi_base + PCIE_DMA_WRITE_ENGINE_EN);
			writel(0, pci->dbi_base + PCIE_DMA_WRITE_INT_MASK);
			writel(0x10000, pci->dbi_base +
			PCIE_DMA_WRITE_LINKED_LIST_ERR_EN);
			writel(0, pci->dbi_base + PCIE_DMA_VIEWPORT_SEL);
			writel(0x04000300, pci->dbi_base +
					PCIE_DMA_CH_CONTROL1);

			/* Set pointer to start of first list */
			dma_set_list_ptr(pp, direction, phy_list_addr);
			/* Ring doorbell */
			writel(0, pci->dbi_base + PCIE_DMA_WRITE_DOORBELL);
		} else {
			return -EBUSY;
		}
	} else {/* Read request */
		if (pp->rd_ch.status != DMA_CH_RUNNING) {
			/* Set ch_status */
			pp->rd_ch.status = DMA_CH_RUNNING;
			/* Set last data elem */
			dma_set_data_elem((u32 *)pp->rd_ch.virt_addr,
			pp->rd_ch.current_list_size - 1,
			pp->rd_ch.current_sar,
			pp->rd_ch.current_dar,  pp->rd_ch.current_size, 1);

			pp->rd_ch.errors = 0;
			/* Clear CR1 for proper init */
			writel(0x0, pci->dbi_base + PCIE_DMA_CH_CONTROL1);

			pp->rd_ch.phy_list_addr = phy_list_addr;
			writel(0x1, pci->dbi_base + PCIE_DMA_READ_ENGINE_EN);
			writel(0, pci->dbi_base + PCIE_DMA_READ_INT_MASK);
			writel(0x10000, pci->dbi_base +
				PCIE_DMA_READ_LINKED_LIST_ERR_EN);
			writel(0x80000000, pci->dbi_base +
				PCIE_DMA_VIEWPORT_SEL);
			writel(0x04000300, pci->dbi_base +
				PCIE_DMA_CH_CONTROL1);

			/* Set pointer to start of first list */
			dma_set_list_ptr(pp, direction, phy_list_addr);
			/* Ring doorbell */
			writel(0, pci->dbi_base + PCIE_DMA_READ_DOORBELL);
		} else {
			return -EBUSY;
		}
	}
	return 0;
}

int dw_pcie_dma_load_linked_list(struct pcie_port *pp,
	struct dma_list arr_ll[], u8 arr_sz, u32 phy_list_addr,
	u32 next_phy_list_addr, u8 direction)
{
	u8 i;
	u32 *ptr_list;

	struct dma_ch_info *ptr_ch = (direction == DMA_CH_WRITE) ?
		&pp->wr_ch : &pp->rd_ch;
	ptr_list = (u32 *)(ioremap(phy_list_addr, SZ_1K));
	if (!ptr_list)
		return -EFAULT;

	arr_sz = pp->ll_info.nr_elem;
	pp->ll_info.phy_list_addr = phy_list_addr;

	for (i = 0 ; i < arr_sz ; i++) {
		dma_set_data_elem((u32 *)ptr_list, i,
			arr_ll[i].sar, arr_ll[i].dar,
			arr_ll[i].size, 0);
		ptr_ch->current_sar = arr_ll[i].sar;
		ptr_ch->current_dar = arr_ll[i].dar;
		ptr_ch->current_size = arr_ll[i].size;
		ptr_ch->current_elem_idx = i;
		ptr_ch->current_list_size = arr_sz;
		ptr_ch->virt_addr = ptr_list;
	}

	dma_set_link_elem(ptr_list, arr_sz, next_phy_list_addr);

	iounmap(ptr_list);
	return 0;
}
int dw_pcie_dma_single_rw(struct pcie_port *pp,
	struct dma_data_elem *dma_single_rw)
{
	u32 flags;
	struct dma_ch_info *ptr_ch;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	/* Invalid channel number */
	if (dma_single_rw->ch_num > PCIE_DMA_NR_CH - 1)
		return -EINVAL;

	/* Invalid transfer size */
	if (dma_single_rw->size > PCIE_DMA_MAX_SIZE)
		return -EINVAL;

	flags = dma_single_rw->flags;
	ptr_ch = (flags & DMA_FLAG_WRITE_ELEM) ?
		&pp->wr_ch : &pp->rd_ch;

	if (flags & DMA_FLAG_WRITE_ELEM) {
		if (pp->wr_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		pp->wr_ch.status = DMA_CH_RUNNING;
		pp->wr_ch.errors = 0;
		dw_pcie_dma_write_en(pp);
		dw_pcie_dma_set_viewport(pp, dma_single_rw->ch_num,
			DMA_CH_WRITE);
	} else {
		if (pp->rd_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		pp->rd_ch.status = DMA_CH_RUNNING;
		pp->rd_ch.errors = 0;
		dw_pcie_dma_read_en(pp);

		dw_pcie_dma_set_viewport(pp, dma_single_rw->ch_num,
			DMA_CH_READ);
	}

	/* Clear CR1 for proper init */
	writel(0x0, pci->dbi_base + PCIE_DMA_CH_CONTROL1);

	if (flags & (DMA_FLAG_EN_DONE_INT | DMA_FLAG_EN_ABORT_INT)) {
		dw_pcie_dma_en_local_int(pp);

		if (flags & (DMA_FLAG_RIE | DMA_FLAG_LIE)) {
			if (flags & DMA_FLAG_WRITE_ELEM) {
				dw_pcie_dma_set_wr_remote_abort_int(pp,
					dma_single_rw->imwr);
				dw_pcie_dma_set_wr_remote_done_int(pp,
					dma_single_rw->imwr);
				dw_pcie_dma_clear_wr_done_int_mask(pp,
					(1 << dma_single_rw->ch_num));
				dw_pcie_dma_clear_wr_abort_int_mask(pp,
					(1 << dma_single_rw->ch_num));
			} else if (flags & DMA_FLAG_READ_ELEM) {
				dw_pcie_dma_set_rd_remote_abort_int(pp,
					dma_single_rw->imwr);
				dw_pcie_dma_set_rd_remote_done_int(pp,
					dma_single_rw->imwr);
				dw_pcie_dma_clear_rd_done_int_mask(pp,
					(1 << dma_single_rw->ch_num));
				dw_pcie_dma_clear_rd_abort_int_mask(pp,
					(1 << dma_single_rw->ch_num));
			}
		}
	}
	/* Set transfer size */
	dw_pcie_dma_set_transfer_size(pp, dma_single_rw->size);
	/* Set SAR & DAR */
	dw_pcie_dma_set_sar(pp, dma_single_rw->sar);
	dw_pcie_dma_set_dar(pp, dma_single_rw->dar);

	if (flags & DMA_FLAG_WRITE_ELEM)
		writel(0, pci->dbi_base + PCIE_DMA_WRITE_DOORBELL);
	else
		writel(0, pci->dbi_base + PCIE_DMA_READ_DOORBELL);

	return 0;
}

void dw_pcie_dma_check_errors(struct pcie_port *pp,
	u32 direction, u32 *error)
{
	u32 val = 0;
	*error = DMA_ERR_NONE;

	if (direction == DMA_CH_WRITE) {
		dw_pcie_readl_rc(pp, PCIE_DMA_WRITE_ERR_STATUS, &val);
		if (val & 0x1)
			*error |= DMA_ERR_WR;
		if (val & 0x10000)
			*error |= DMA_ERR_FETCH_LL;
	} else {
		/* Get error status low */
		dw_pcie_readl_rc(pp, PCIE_DMA_READ_ERR_STATUS_LOW, &val);
		if (val & 0x1)
			*error |= DMA_ERR_RD;
		if (val & 0x10000)
			*error |= DMA_ERR_FETCH_LL;
		/* Get error status high */
		dw_pcie_readl_rc(pp, PCIE_DMA_READ_ERR_STATUS_HIGH, &val);
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
irqreturn_t dw_handle_dma_irq(struct pcie_port *pp)
{
	u32 val_write = 0;
	u32 val_read = 0;
	u32 err_type = DMA_ERR_NONE;
	irqreturn_t ret = IRQ_HANDLED;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_readl_rc(pp, PCIE_DMA_WRITE_INT_STATUS, &val_write);
	dw_pcie_readl_rc(pp, PCIE_DMA_READ_INT_STATUS, &val_read);

	if (val_write) {
		if (pp->wr_ch.status == DMA_CH_RUNNING) {
			if (val_write & 0x10000) { /* Abort interrupt */
				/* Get error type */
				dw_pcie_dma_check_errors(pp,
					DMA_FLAG_WRITE_ELEM,
					&pp->wr_ch.errors);
					writel(0x00FF0000, pci->dbi_base +
						PCIE_DMA_WRITE_INT_CLEAR);
				err_type = pp->wr_ch.errors;
			} else { /* Done interrupt */
				writel(0x000000FF, pci->dbi_base +
					PCIE_DMA_WRITE_INT_CLEAR);
				/* Check channel list mode */
			}
			pp->wr_ch.status = DMA_CH_STOPPED;
			#ifdef CONFIG_PCI_S32V234
			if (pci->ops->send_signal_to_user)
				pci->ops->send_signal_to_user(pp);
			#endif
		} else
			writel(0x00FF00FF, pci->dbi_base +
				PCIE_DMA_WRITE_INT_CLEAR);

		#ifdef CONFIG_PCI_S32V234
		if (pp->call_back)
			pp->call_back(val_write);
		#endif
	}
	if (val_read) {
		if (pp->rd_ch.status == DMA_CH_RUNNING) {
			/* Search interrupt type, abort or done */
			/* Abort interrupt */
			if (val_read & 0x80000) {
				/* Get error type */
				dw_pcie_dma_check_errors(pp, DMA_FLAG_READ_ELEM,
					&pp->rd_ch.errors);
				writel(0x00FF0000, pci->dbi_base +
						PCIE_DMA_READ_INT_CLEAR);
				err_type = pp->rd_ch.errors;
			} else { /* Done interrupt */
				writel(0x000000FF, pci->dbi_base +
					PCIE_DMA_READ_INT_CLEAR);
				/* Check channel list mode */
			}
			pp->rd_ch.status = DMA_CH_STOPPED;
			#ifdef CONFIG_PCI_S32V234
			if (pci->ops->send_signal_to_user)
				pci->ops->send_signal_to_user(pp);
			#endif
		} else
			writel(0x00FF00FF, pci->dbi_base +
				PCIE_DMA_READ_INT_CLEAR);
	}
	if (pp->ptr_func)
		pp->ptr_func(err_type);

	return ret;
}
#endif /* CONFIG_PCI_DW_DMA */

static void dwc_irq_ack(struct irq_data *d)
{
	struct msi_desc *msi = irq_data_get_msi_desc(d);
	struct pcie_port *pp = msi_desc_to_pci_sysdata(msi);
	int pos = d->hwirq % 32;
	int i = d->hwirq / 32;

	dw_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_STATUS + i * 12, 4, BIT(pos));
}

static struct irq_chip dw_msi_irq_chip = {
	.name = "PCI-MSI",
	.irq_ack = dwc_irq_ack,
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

/* MSI int handler */
irqreturn_t dw_handle_msi_irq(struct pcie_port *pp)
{
	u32 val;
	int i, pos, irq;
	irqreturn_t ret = IRQ_NONE;

	for (i = 0; i < MAX_MSI_CTRLS; i++) {
		dw_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_STATUS + i * 12, 4,
				    &val);
		if (!val)
			continue;

		ret = IRQ_HANDLED;
		pos = 0;
		while ((pos = find_next_bit((unsigned long *) &val, 32,
					    pos)) != 32) {
			irq = irq_find_mapping(pp->irq_domain, i * 32 + pos);
			generic_handle_irq(irq);
			pos++;
		}
	}

	return ret;
}

void dw_pcie_msi_init(struct pcie_port *pp)
{
	u64 msi_target;

	pp->msi_data = __get_free_pages(GFP_KERNEL, 0);
	msi_target = virt_to_phys((void *)pp->msi_data);

	/* program the msi_data */
	dw_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_LO, 4,
			    (u32)(msi_target & 0xffffffff));
	dw_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_HI, 4,
			    (u32)(msi_target >> 32 & 0xffffffff));
}

static void dw_pcie_msi_clear_irq(struct pcie_port *pp, int irq)
{
	unsigned int res, bit, val;

	res = (irq / 32) * 12;
	bit = irq % 32;
	dw_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, &val);
	val &= ~(1 << bit);
	dw_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, val);
}

static void clear_irq_range(struct pcie_port *pp, unsigned int irq_base,
			    unsigned int nvec, unsigned int pos)
{
	unsigned int i;

	for (i = 0; i < nvec; i++) {
		irq_set_msi_desc_off(irq_base, i, NULL);
		/* Disable corresponding interrupt on MSI controller */
		if (pp->ops->msi_clear_irq)
			pp->ops->msi_clear_irq(pp, pos + i);
		else
			dw_pcie_msi_clear_irq(pp, pos + i);
	}

	bitmap_release_region(pp->msi_irq_in_use, pos, order_base_2(nvec));
}

static void dw_pcie_msi_set_irq(struct pcie_port *pp, int irq)
{
	unsigned int res, bit, val;

	res = (irq / 32) * 12;
	bit = irq % 32;
	dw_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, &val);
	val |= 1 << bit;
	dw_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, val);
}

static int assign_irq(int no_irqs, struct msi_desc *desc, int *pos)
{
	int irq, pos0, i;
	struct pcie_port *pp;

	pp = (struct pcie_port *)msi_desc_to_pci_sysdata(desc);
	pos0 = bitmap_find_free_region(pp->msi_irq_in_use, MAX_MSI_IRQS,
				       order_base_2(no_irqs));
	if (pos0 < 0)
		goto no_valid_irq;

	irq = irq_find_mapping(pp->irq_domain, pos0);
	if (!irq)
		goto no_valid_irq;

	/*
	 * irq_create_mapping (called from dw_pcie_host_init) pre-allocates
	 * descs so there is no need to allocate descs here. We can therefore
	 * assume that if irq_find_mapping above returns non-zero, then the
	 * descs are also successfully allocated.
	 */

	for (i = 0; i < no_irqs; i++) {
		if (irq_set_msi_desc_off(irq, i, desc) != 0) {
			clear_irq_range(pp, irq, i, pos0);
			goto no_valid_irq;
		}
		/*Enable corresponding interrupt in MSI interrupt controller */
		if (pp->ops->msi_set_irq)
			pp->ops->msi_set_irq(pp, pos0 + i);
		else
			dw_pcie_msi_set_irq(pp, pos0 + i);
	}

	*pos = pos0;
	desc->nvec_used = no_irqs;
	desc->msi_attrib.multiple = order_base_2(no_irqs);

	return irq;

no_valid_irq:
	*pos = pos0;
	return -ENOSPC;
}

static void dw_msi_setup_msg(struct pcie_port *pp, unsigned int irq, u32 pos)
{
	struct msi_msg msg;
	u64 msi_target;

	if (pp->ops->get_msi_addr)
		msi_target = pp->ops->get_msi_addr(pp);
	else
		msi_target = virt_to_phys((void *)pp->msi_data);

	msg.address_lo = (u32)(msi_target & 0xffffffff);
	msg.address_hi = (u32)(msi_target >> 32 & 0xffffffff);

	if (pp->ops->get_msi_data)
		msg.data = pp->ops->get_msi_data(pp, pos);
	else
		msg.data = pos;

	pci_write_msi_msg(irq, &msg);
}

static int dw_msi_setup_irq(struct msi_controller *chip, struct pci_dev *pdev,
			    struct msi_desc *desc)
{
	int irq, pos;
	struct pcie_port *pp = pdev->bus->sysdata;

	if (desc->msi_attrib.is_msix)
		return -EINVAL;

	irq = assign_irq(1, desc, &pos);
	if (irq < 0)
		return irq;

	dw_msi_setup_msg(pp, irq, pos);

	return 0;
}

static int dw_msi_setup_irqs(struct msi_controller *chip, struct pci_dev *pdev,
			     int nvec, int type)
{
#ifdef CONFIG_PCI_MSI
	int irq, pos;
	struct msi_desc *desc;
	struct pcie_port *pp = pdev->bus->sysdata;

	/* MSI-X interrupts are not supported */
	if (type == PCI_CAP_ID_MSIX)
		return -EINVAL;

	WARN_ON(!list_is_singular(&pdev->dev.msi_list));
	desc = list_entry(pdev->dev.msi_list.next, struct msi_desc, list);

	irq = assign_irq(nvec, desc, &pos);
	if (irq < 0)
		return irq;

	dw_msi_setup_msg(pp, irq, pos);

	return 0;
#else
	return -EINVAL;
#endif
}

static void dw_msi_teardown_irq(struct msi_controller *chip, unsigned int irq)
{
	struct irq_data *data = irq_get_irq_data(irq);
	struct msi_desc *msi = irq_data_get_msi_desc(data);
	struct pcie_port *pp = (struct pcie_port *)msi_desc_to_pci_sysdata(msi);

	clear_irq_range(pp, irq, 1, data->hwirq);
}

static struct msi_controller dw_pcie_msi_chip = {
	.setup_irq = dw_msi_setup_irq,
	.setup_irqs = dw_msi_setup_irqs,
	.teardown_irq = dw_msi_teardown_irq,
};

static int dw_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
			   irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dw_msi_irq_chip, handle_edge_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = dw_pcie_msi_map,
};

int dw_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pdev = to_platform_device(dev);
	struct pci_bus *bus, *child;
	struct pci_host_bridge *bridge;
	struct resource *cfg_res;
	int i, ret;
	struct resource_entry *win, *tmp;

	cfg_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	if (cfg_res) {
		pp->cfg0_size = resource_size(cfg_res) / 2;
		pp->cfg1_size = resource_size(cfg_res) / 2;
		pp->cfg0_base = cfg_res->start;
		pp->cfg1_base = cfg_res->start + pp->cfg0_size;
	} else if (!pp->va_cfg0_base) {
		dev_err(dev, "missing *config* reg space\n");
	}

	bridge = pci_alloc_host_bridge(0);
	if (!bridge)
		return -ENOMEM;

	ret = of_pci_get_host_bridge_resources(np, 0, 0xff,
					&bridge->windows, &pp->io_base);
	if (ret)
		return ret;

	ret = devm_request_pci_bus_resources(dev, &bridge->windows);
	if (ret)
		goto error;

	/* Get the I/O and memory ranges from DT */
	resource_list_for_each_entry_safe(win, tmp, &bridge->windows) {
		switch (resource_type(win->res)) {
		case IORESOURCE_IO:
			ret = pci_remap_iospace(win->res, pp->io_base);
			if (ret) {
				dev_warn(dev, "error %d: failed to map resource %pR\n",
					 ret, win->res);
				resource_list_destroy_entry(win);
			} else {
				pp->io = win->res;
				pp->io->name = "I/O";
				pp->io_size = resource_size(pp->io);
				pp->io_bus_addr = pp->io->start - win->offset;
			}
			break;
		case IORESOURCE_MEM:
			pp->mem = win->res;
			pp->mem->name = "MEM";
			pp->mem_size = resource_size(pp->mem);
			pp->mem_bus_addr = pp->mem->start - win->offset;
			break;
		case 0:
			pp->cfg = win->res;
			pp->cfg0_size = resource_size(pp->cfg) / 2;
			pp->cfg1_size = resource_size(pp->cfg) / 2;
			pp->cfg0_base = pp->cfg->start;
			pp->cfg1_base = pp->cfg->start + pp->cfg0_size;
			break;
		case IORESOURCE_BUS:
			pp->busn = win->res;
			break;
		}
	}

	if (!pci->dbi_base) {
		pci->dbi_base = devm_pci_remap_cfgspace(dev,
						pp->cfg->start,
						resource_size(pp->cfg));
		if (!pci->dbi_base) {
			dev_err(dev, "error with ioremap\n");
			ret = -ENOMEM;
			goto error;
		}
	}

	pp->mem_base = pp->mem->start;

	if (!pp->va_cfg0_base) {
		pp->va_cfg0_base = devm_pci_remap_cfgspace(dev,
					pp->cfg0_base, pp->cfg0_size);
		if (!pp->va_cfg0_base) {
			dev_err(dev, "error with ioremap in function\n");
			ret = -ENOMEM;
			goto error;
		}
	}

	if (!pp->va_cfg1_base) {
		pp->va_cfg1_base = devm_pci_remap_cfgspace(dev,
						pp->cfg1_base,
						pp->cfg1_size);
		if (!pp->va_cfg1_base) {
			dev_err(dev, "error with ioremap\n");
			ret = -ENOMEM;
			goto error;
		}
	}

	ret = of_property_read_u32(np, "num-viewport", &pci->num_viewport);
	if (ret)
		pci->num_viewport = 2;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		if (!pp->ops->msi_host_init) {
			pp->irq_domain = irq_domain_add_linear(dev->of_node,
						MAX_MSI_IRQS, &msi_domain_ops,
						&dw_pcie_msi_chip);
			if (!pp->irq_domain) {
				dev_err(dev, "irq domain init failed\n");
				ret = -ENXIO;
				goto error;
			}

			for (i = 0; i < MAX_MSI_IRQS; i++)
				irq_create_mapping(pp->irq_domain, i);
		} else {
			ret = pp->ops->msi_host_init(pp, &dw_pcie_msi_chip);
			if (ret < 0)
				goto error;
		}
	}

	if (pp->ops->host_init) {
		ret = pp->ops->host_init(pp);
		if (ret)
			goto error;
	}

	pp->root_bus_nr = pp->busn->start;

	bridge->dev.parent = dev;
	bridge->sysdata = pp;
	bridge->busnr = pp->root_bus_nr;
	bridge->ops = &dw_pcie_ops;
	bridge->map_irq = of_irq_parse_and_map_pci;
	bridge->swizzle_irq = pci_common_swizzle;
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		bridge->msi = &dw_pcie_msi_chip;
		dw_pcie_msi_chip.dev = dev;
	}

	ret = pci_scan_root_bus_bridge(bridge);
	if (ret)
		goto error;

	bus = bridge->bus;

	if (pp->ops->scan_bus)
		pp->ops->scan_bus(pp);

	pci_bus_size_bridges(bus);
	pci_bus_assign_resources(bus);

	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);

	pci_bus_add_devices(bus);
	return 0;

error:
	pci_free_host_bridge(bridge);
	return ret;
}

static int dw_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				 u32 devfn, int where, int size, u32 *val)
{
	int ret, type;
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (pp->ops->rd_other_conf)
		return pp->ops->rd_other_conf(pp, bus, devfn, where, size, val);

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (bus->parent->number == pp->root_bus_nr) {
		type = PCIE_ATU_TYPE_CFG0;
		cpu_addr = pp->cfg0_base;
		cfg_size = pp->cfg0_size;
		va_cfg_base = pp->va_cfg0_base;
	} else {
		type = PCIE_ATU_TYPE_CFG1;
		cpu_addr = pp->cfg1_base;
		cfg_size = pp->cfg1_size;
		va_cfg_base = pp->va_cfg1_base;
	}

	dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX1,
				  type, cpu_addr,
				  busdev, cfg_size);
	ret = dw_pcie_read(va_cfg_base + where, size, val);
	if (pci->num_viewport <= 2)
		dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX1,
					  PCIE_ATU_TYPE_IO, pp->io_base,
					  pp->io_bus_addr, pp->io_size);

	return ret;
}

static int dw_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				 u32 devfn, int where, int size, u32 val)
{
	int ret, type;
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (pp->ops->wr_other_conf)
		return pp->ops->wr_other_conf(pp, bus, devfn, where, size, val);

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (bus->parent->number == pp->root_bus_nr) {
		type = PCIE_ATU_TYPE_CFG0;
		cpu_addr = pp->cfg0_base;
		cfg_size = pp->cfg0_size;
		va_cfg_base = pp->va_cfg0_base;
	} else {
		type = PCIE_ATU_TYPE_CFG1;
		cpu_addr = pp->cfg1_base;
		cfg_size = pp->cfg1_size;
		va_cfg_base = pp->va_cfg1_base;
	}

	dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX1,
				  type, cpu_addr,
				  busdev, cfg_size);
	ret = dw_pcie_write(va_cfg_base + where, size, val);
	if (pci->num_viewport <= 2)
		dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX1,
					  PCIE_ATU_TYPE_IO, pp->io_base,
					  pp->io_bus_addr, pp->io_size);

	return ret;
}

static int dw_pcie_valid_device(struct pcie_port *pp, struct pci_bus *bus,
				int dev)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	/* If there is no link, then there is no device */
	if (bus->number != pp->root_bus_nr) {
		if (!dw_pcie_link_up(pci))
			return 0;
	}

	/* access only one slot on each root port */
	if (bus->number == pp->root_bus_nr && dev > 0)
		return 0;

	return 1;
}

static int dw_pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			   int size, u32 *val)
{
	struct pcie_port *pp = bus->sysdata;

	if (!dw_pcie_valid_device(pp, bus, PCI_SLOT(devfn))) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (bus->number == pp->root_bus_nr)
		return dw_pcie_rd_own_conf(pp, where, size, val);

	return dw_pcie_rd_other_conf(pp, bus, devfn, where, size, val);
}

static int dw_pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			   int where, int size, u32 val)
{
	struct pcie_port *pp = bus->sysdata;

	if (!dw_pcie_valid_device(pp, bus, PCI_SLOT(devfn)))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == pp->root_bus_nr)
		return dw_pcie_wr_own_conf(pp, where, size, val);

	return dw_pcie_wr_other_conf(pp, bus, devfn, where, size, val);
}

static struct pci_ops dw_pcie_ops = {
	.read = dw_pcie_rd_conf,
	.write = dw_pcie_wr_conf,
};

static u8 dw_pcie_iatu_unroll_enabled(struct dw_pcie *pci)
{
	u32 val;

	val = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT);
	if (val == 0xffffffff)
		return 1;

	return 0;
}

void dw_pcie_setup_rc(struct pcie_port *pp)
{
	u32 val;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_setup(pci);

	/* setup RC BARs */
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0x00000004);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1, 0x00000000);

	/* setup interrupt pins */
	dw_pcie_dbi_ro_wr_en(pci);
	val = dw_pcie_readl_dbi(pci, PCI_INTERRUPT_LINE);
	val &= 0xffff00ff;
	val |= 0x00000100;
	dw_pcie_writel_dbi(pci, PCI_INTERRUPT_LINE, val);
	dw_pcie_dbi_ro_wr_dis(pci);

	/* setup bus numbers */
	val = dw_pcie_readl_dbi(pci, PCI_PRIMARY_BUS);
	val &= 0xff000000;
	val |= 0x00ff0100;
	dw_pcie_writel_dbi(pci, PCI_PRIMARY_BUS, val);

	/* setup command register */
	val = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	val &= 0xffff0000;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	dw_pcie_writel_dbi(pci, PCI_COMMAND, val);

	/*
	 * If the platform provides ->rd_other_conf, it means the platform
	 * uses its own address translation component rather than ATU, so
	 * we should not program the ATU here.
	 */
	if (!pp->ops->rd_other_conf) {
		/* get iATU unroll support */
		pci->iatu_unroll_enabled = dw_pcie_iatu_unroll_enabled(pci);
		dev_dbg(pci->dev, "iATU unroll: %s\n",
			pci->iatu_unroll_enabled ? "enabled" : "disabled");

		dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX0,
					  PCIE_ATU_TYPE_MEM, pp->mem_base,
					  pp->mem_bus_addr, pp->mem_size);
		if (pci->num_viewport > 2)
			dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX2,
						  PCIE_ATU_TYPE_IO, pp->io_base,
						  pp->io_bus_addr, pp->io_size);
	}

	dw_pcie_wr_own_conf(pp, PCI_BASE_ADDRESS_0, 4, 0);

	/* Enable write permission for the DBI read-only register */
	dw_pcie_dbi_ro_wr_en(pci);
	/* program correct class for RC */
	dw_pcie_wr_own_conf(pp, PCI_CLASS_DEVICE, 2, PCI_CLASS_BRIDGE_PCI);
	/* Better disable write permission right after the update */
	dw_pcie_dbi_ro_wr_dis(pci);

	dw_pcie_rd_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, &val);
	val |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_wr_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, val);
}
