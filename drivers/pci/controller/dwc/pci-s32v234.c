// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Freescale S32V SoCs
 *
 * Copyright (C) 2013 Kosagi
 *		http://www.kosagi.com
 *
 * Author: Sean Cross <xobs@kosagi.com>
 *
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2017-2021 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/s32v234-src.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/of_platform.h>

#include "pci-s32v234.h"

/* TODO: across the entire file:
 * - use dedicated dw_* functions for dbi_base access
 * - replace function argument struct pcie_port* with struct dw_pcie*
 * - find duplicate atributes and functions (e.g. related to ATU
 * inbound/outbound setup)
 * - update endpoint functionality based on the new dw ep implementation
 * and types
 */

#define SETUP_OUTBOUND		_IOWR('S', 1, struct s32v_outbound_region)
#define SETUP_INBOUND		_IOWR('S', 2, struct s32v_inbound_region)
#define SEND_MSI		_IOWR('S', 3, u64)
#define GET_BAR_INFO		_IOWR('S', 4, struct s32v_bar)
#define STORE_PID		_IOR('S', 7,  s32)
#define SEND_SIGNAL		_IOR('S', 8,  int)
#ifdef CONFIG_PCI_DW_DMA
#define SEND_SINGLE_DMA		_IOWR('S', 6, struct dma_data_elem)
#define GET_DMA_CH_ERRORS	_IOR('S', 9,  u32)
#define RESET_DMA_WRITE		_IOW('S', 10,  u32)
#define RESET_DMA_READ		_IOW('S', 11,  u32)
#define STORE_LL_INFO		_IOR('S', 12,  struct dma_ll_info)
#define SEND_LL			_IOWR('S', 13, struct dma_list(*)[])
#define START_LL		_IOWR('S', 14, u32)
#endif

#define PCIE_MSI_CAP			0x50
#define PCIE_MSI_ADDR_LOWER		0x54
#define PCIE_MSI_ADDR_UPPER		0x58
#define PCIE_MSI_DATA			0x5C
#define PCIE_ATU_VIEWPORT		0x900
#define PCIE_ATU_CR1			0x904
#define PCIE_ATU_CR2			0x908
#define PCIE_ATU_BAR_NUM(bar)	((bar) << 8)
#define PCIE_ATU_UPPER_TARGET		0x91C

/* PCIe Root Complex registers (memory-mapped) */
#define PCIE_RC_LCR				0x7c
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1	0x1
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2	0x2
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK	0xf

#define PCIE_RC_LCSR			0x80

/* PCIe Port Logic registers (memory-mapped) */
#define PL_OFFSET 0x700
#define PCIE_PL_PFLR (PL_OFFSET + 0x08)
#define PCIE_PL_PFLR_LINK_STATE_MASK		(0x3f << 16)
#define PCIE_PL_PFLR_FORCE_LINK			(1 << 15)
#define PCIE_PHY_DEBUG_R0 (PL_OFFSET + 0x28)
#define PCIE_PHY_DEBUG_R1 (PL_OFFSET + 0x2c)
#define PCIE_PHY_DEBUG_R1_XMLH_LINK_IN_TRAINING	(1 << 29)
#define PCIE_PHY_DEBUG_R1_XMLH_LINK_UP		(1 << 4)

#define PCIE_PHY_CTRL (PL_OFFSET + 0x114)
#define PCIE_PHY_CTRL_DATA_LOC 0
#define PCIE_PHY_CTRL_CAP_ADR_LOC 16
#define PCIE_PHY_CTRL_CAP_DAT_LOC 17
#define PCIE_PHY_CTRL_WR_LOC 18
#define PCIE_PHY_CTRL_RD_LOC 19

#define PCIE_PHY_STAT (PL_OFFSET + 0x110)
#define PCIE_PHY_STAT_ACK_LOC 16

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C

/* PHY registers (not memory-mapped) */
#define PCIE_PHY_RX_ASIC_OUT 0x100D

#define PHY_RX_OVRD_IN_LO 0x1005
#define PHY_RX_OVRD_IN_LO_RX_DATA_EN (1 << 5)
#define PHY_RX_OVRD_IN_LO_RX_PLL_EN (1 << 3)

/* Default timeout (ms) */
#define PCIE_CX_CPL_BASE_TIMER_VALUE	10

/* MSI base region  */
#define MSI_REGION		0x72FB0000
#define PCI_BASE_ADDR		0x72000000
#define PCI_BASE_DBI		0x72FFC000
#define MSI_REGION_NR		3
#define NR_REGIONS		4
/* BAR generic definitions */
#define PCI_REGION_MEM		0x00000000	/* PCI mem space */
#define PCI_REGION_IO		0x00000001	/* PCI IO space */
#define PCI_WIDTH_32b		0x00000000	/* 32-bit BAR */
#define PCI_WIDTH_64b		0x00000004	/* 64-bit BAR */
#define PCI_REGION_PREFETCH	0x00000008	/* prefetch PCI mem */
#define PCI_REGION_NON_PREFETCH	0x00000000	/* non-prefetch PCI mem */
/* BARs sizing */
#define PCIE_BAR0_SIZE		SZ_1M	/* 1MB */
#define PCIE_BAR1_SIZE		0
#define PCIE_BAR2_SIZE		SZ_1M	/* 1MB */
#define PCIE_BAR3_SIZE		0		/* 256B Fixed sizing  */
#define PCIE_BAR4_SIZE		0		/* 4K Fixed sizing  */
#define PCIE_BAR5_SIZE		0		/* 64K Fixed sizing  */
#define PCIE_ROM_SIZE		0
/* BARs individual en/dis  */
#define PCIE_BAR0_EN_DIS		1
#define PCIE_BAR1_EN_DIS		0
#define PCIE_BAR2_EN_DIS		1
#define PCIE_BAR3_EN_DIS		1
#define PCIE_BAR4_EN_DIS		1
#define PCIE_BAR5_EN_DIS		1
#define PCIE_ROM_EN_DIS			0
/* BARs configuration */
#define PCIE_BAR0_INIT	(PCI_REGION_MEM | PCI_WIDTH_32b | \
		PCI_REGION_NON_PREFETCH)
#define PCIE_BAR1_INIT	(PCI_REGION_MEM | PCI_WIDTH_32b | \
		PCI_REGION_NON_PREFETCH)
#define PCIE_BAR2_INIT	(PCI_REGION_MEM | PCI_WIDTH_32b | \
		PCI_REGION_NON_PREFETCH)
#define PCIE_BAR3_INIT	(PCI_REGION_MEM | PCI_WIDTH_32b | \
		PCI_REGION_NON_PREFETCH)
#define PCIE_BAR4_INIT	0
#define PCIE_BAR5_INIT	0
#define PCIE_ROM_INIT	0

/* SOC revision */
#define SOC_REVISION_MINOR_MASK		(0xF)
#define SOC_REVISION_MAJOR_SHIFT	(4)
#define SOC_REVISION_MAJOR_MASK		(0xF << SOC_REVISION_MAJOR_SHIFT)
#define SOC_REVISION_MASK		(SOC_REVISION_MINOR_MASK | \
					    SOC_REVISION_MAJOR_MASK)

#define SIUL2_MIDR1_OFF			0x4
#define pr_soc_debug pr_debug

static int s32v234_pcie_get_soc_revision(void)
{
	struct device_node *node = NULL;
	int rev = -1;
	const __be32 *siul2_base = NULL;
	u64 siul2_base_address = OF_BAD_ADDR;

	pr_soc_debug("Searching SIUL2 MIDR registers in device-tree\n");
	node = of_find_node_by_name(NULL, "siul2");
	if (node) {
		siul2_base = of_get_property(node, "midr-reg", NULL);

		if (siul2_base)
			siul2_base_address =
				of_translate_address(node, siul2_base);

		of_node_put(node);
	} else {
		pr_warn("Could not get siul2 node from device-tree\n");
		goto err_find_node;
	}

	if (siul2_base_address != OF_BAD_ADDR) {
		char *siul2_virt_addr = ioremap(siul2_base_address,
							SZ_1K);

		pr_soc_debug("Resolved SIUL2 base address to 0x%llx\n",
				siul2_base_address);

		if (siul2_virt_addr) {
			rev = readl(siul2_virt_addr + SIUL2_MIDR1_OFF) &
				(SOC_REVISION_MASK);
			pr_soc_debug("SIUL2_MIDR1 (0x%llx) revision: 0x%x\n",
				siul2_base_address + SIUL2_MIDR1_OFF, rev);
			iounmap(siul2_virt_addr);
			return rev;
		}
		pr_warn("Could not remap SIUL2 memory\n");
	} else
		pr_warn("Could not translate SIUL2 base address\n");

err_find_node:
	return rev;
}

struct task_struct *task;
struct s32v_bar {
	u32 bar_nr;
	u32 size;
	u32 addr;
};

struct s32v_outbound_region restore_outb_arr[4];
struct s32v_inbound_region restore_inb_arr[4];
static struct s32v234_pcie *s32v234_pcie_ep;

#ifdef CONFIG_PCI_DW_DMA
static int s32v234_store_ll_array(struct s32v234_pcie *s32v234_pp, void __user *argp)
{
	int ret = 0;
	struct dw_pcie *pcie = &s32v234_pp->pcie;
	struct dma_info *di = &(s32v234_pp->dma);
	u32 ll_nr_elem = di->ll_info.nr_elem;

	if (argp && di->dma_linked_list) {
		if (copy_from_user(di->dma_linked_list, argp,
			sizeof(struct dma_list) * ll_nr_elem))
			return -EFAULT;
	} else {/* Null argument */
		return -EFAULT;
	}
	ret = dw_pcie_dma_load_linked_list(pcie, di,
		ll_nr_elem, di->ll_info.phy_list_addr,
		di->ll_info.next_phy_list_addr,
		di->ll_info.direction);

	return ret;
}

int s32v234_start_dma_ll(struct s32v234_pcie *s32v234_pp, void __user *argp)
{
	int ret = 0;
	u32 phy_addr;
	struct dw_pcie *pcie = &(s32v234_pp->pcie);
	struct dma_info *di = &(s32v234_pp->dma);

	if (argp) {
		if (copy_from_user(&phy_addr, argp, sizeof(phy_addr)))
			return -EFAULT;
	} else {/* Null argument */
		return -EFAULT;
	}
	ret = dw_pcie_dma_start_linked_list(pcie, di,
		phy_addr,
		di->ll_info.direction);
	return ret;
}

int s32v234_store_ll_array_info(struct dma_info *di, void __user *argp)
{
	int ret = 0;

	if (argp) {
		if (copy_from_user(&di->ll_info, argp,
			sizeof(struct dma_ll_info)))
			return -EFAULT;
	} else {/* Null argument */
		return -EFAULT;
	}
	/* Alloc here space for pointer to array of structs */
	/* Make sure it is null before allocating space */
	if (!di->dma_linked_list) {
		di->dma_linked_list =
			(struct dma_list(*)[])kcalloc(di->ll_info.nr_elem,
				sizeof(struct dma_list), GFP_KERNEL);
	}

	return ret;
}

static int s32v234_send_dma_errors(struct dma_info *di, void __user *argp)
{
	int ret = 0;
	u32 dma_errors;

	dma_errors = ((di->wr_ch.errors) << 16) | di->rd_ch.errors;

	if (copy_to_user((unsigned int *)argp, &dma_errors, sizeof(u32)))
		return -EFAULT;
	return ret;
}

static int s32v234_send_dma_single(struct s32v234_pcie *s32v234_pp, void __user *argp)
{
	int ret = 0;
	struct dma_data_elem dma_elem_local;
	struct dw_pcie *pcie = &(s32v234_pp->pcie);
	struct dma_info *di = &(s32v234_pp->dma);

	if (argp) {
		if (copy_from_user(&dma_elem_local, argp,
			sizeof(struct dma_data_elem)))
			return -EFAULT;
	} else
		return -EFAULT;
	ret = dw_pcie_dma_single_rw(pcie, di, &dma_elem_local);
	return ret;
}

void s32v234_reset_dma_write(struct s32v234_pcie *s32v234_pp)
{
	struct dw_pcie *pcie = &(s32v234_pp->pcie);
	struct dma_info *di = &(s32v234_pp->dma);

	dw_pcie_dma_write_soft_reset(pcie, di);
}

void s32v234_reset_dma_read(struct s32v234_pcie *s32v234_pp)
{
	struct dw_pcie *pcie = &(s32v234_pp->pcie);
	struct dma_info *di = &(s32v234_pp->dma);

	dw_pcie_dma_read_soft_reset(pcie, di);
}

static irqreturn_t s32v234_pcie_dma_handler(int irq, void *arg)
{
	struct s32v234_pcie *s32v234_pp = arg;
	struct dw_pcie *pcie = &(s32v234_pp->pcie);
	struct dma_info *di = &(s32v234_pp->dma);

	u32 val_write = 0;
	u32 val_read = 0;

	val_write = dw_pcie_readl_dbi(pcie, PCIE_DMA_WRITE_INT_STATUS);
	val_read = dw_pcie_readl_dbi(pcie, PCIE_DMA_READ_INT_STATUS);

	if (val_write) {
		bool signal = (di->wr_ch.status == DMA_CH_RUNNING);

		dw_handle_dma_irq_write(pcie, di, val_write);
		if (signal && s32v234_pp->send_signal_to_user)
			s32v234_pp->send_signal_to_user(s32v234_pp);

		if (s32v234_pp->call_back)
			s32v234_pp->call_back(val_write);
	}
	if (val_read) {
		bool signal = (di->rd_ch.status == DMA_CH_RUNNING);

		dw_handle_dma_irq_read(pcie, di, val_read);
		if (signal && s32v234_pp->send_signal_to_user)
			s32v234_pp->send_signal_to_user(s32v234_pp);
	}

	return IRQ_HANDLED;
}

#endif /* CONFIG_PCI_DW_DMA */

int send_signal_to_user(struct s32v234_pcie *s32v234_pp)
{
	int ret = 0;

	if (s32v234_pp->user_pid > 0) {

		rcu_read_lock();
		task = pid_task(find_pid_ns(s32v234_pp->user_pid, &init_pid_ns),
						PIDTYPE_PID);
		rcu_read_unlock();

		ret = send_sig_info(SIGUSR1, &s32v234_pp->info, task);
		if (ret < 0)
			ret = -EFAULT;
	}

	return ret;
}

int s32v234_store_pid(struct s32v234_pcie *s32v234_pp, void __user *argp)
{
	int ret = 0;

	if (argp) {
		if (copy_from_user(&s32v234_pp->user_pid, argp, sizeof(s32v234_pp->user_pid)))
			return -EFAULT;
	}
	return ret;
}

static int s32v234_setup_MSI(struct s32v234_pcie *s32v234_pp, void __user *argp)
{
	int ret = 0;
	u32 *ptr;
	u64 msi_addr;
	struct dw_pcie *pcie = &(s32v234_pp->pcie);

	if (argp) {
		if (copy_from_user(&msi_addr, argp, sizeof(msi_addr)))
			return -EFAULT;
	} else {
		msi_addr = (((u64)(readl(pcie->dbi_base + PCIE_MSI_ADDR_UPPER))
		<< 32) | readl(pcie->dbi_base + PCIE_MSI_ADDR_LOWER));
	}

	if (!((msi_addr >= PCI_BASE_ADDR) &&
		(msi_addr < PCI_BASE_DBI))) {
		/* Region 3 used */
		dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT,
			PCIE_ATU_REGION_OUTBOUND | MSI_REGION_NR);
		/* Setup last aligned 64K before DBI */
		dw_pcie_writel_dbi(pcie, PCIE_ATU_LOWER_BASE, MSI_REGION);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_UPPER_BASE, 0);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_LIMIT, MSI_REGION + SZ_64K);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_LOWER_TARGET,
			lower_32_bits(msi_addr));
		dw_pcie_writel_dbi(pcie, PCIE_ATU_UPPER_TARGET,
			upper_32_bits(msi_addr));
		dw_pcie_writel_dbi(pcie, PCIE_ATU_CR1, PCIE_ATU_TYPE_MEM);
		/* Enable region */
		dw_pcie_writel_dbi(pcie, PCIE_ATU_CR2, PCIE_ATU_ENABLE);
		ptr = (u32 *)ioremap(MSI_REGION, SZ_4K);
		*ptr = 0;
	} else { /* MSI addr is inside outbound region for MSI */
		ptr = (u32 *)ioremap(msi_addr, SZ_4K);
		*ptr = 0;
	}
	return ret;
}

static void store_inb_atu(struct s32v_inbound_region *ptrInb)
{
	int region_nr;

	region_nr = ptrInb->region;
	restore_inb_arr[region_nr].bar_nr = ptrInb->bar_nr;
	restore_inb_arr[region_nr].target_addr = ptrInb->target_addr;
	restore_inb_arr[region_nr].region = ptrInb->region;
}

static void store_outb_atu(struct s32v_outbound_region *ptrOutb)
{
	int region_nr;

	region_nr = ptrOutb->region;
	restore_outb_arr[region_nr].target_addr = ptrOutb->target_addr;
	restore_outb_arr[region_nr].base_addr = ptrOutb->base_addr;
	restore_outb_arr[region_nr].size = ptrOutb->size;
	restore_outb_arr[region_nr].region_type = ptrOutb->region_type;
	restore_outb_arr[region_nr].region = ptrOutb->region;
}

static int s32v234_pcie_iatu_outbound_set(struct dw_pcie *pcie,
		struct s32v_outbound_region *ptrOutb)
{
	int ret = 0;

	if ((ptrOutb->size < (64 * SZ_1K)) ||
		(ptrOutb->region > NR_REGIONS - 1))
		return -EINVAL;

	dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT,
		PCIE_ATU_REGION_OUTBOUND | ptrOutb->region);
	dw_pcie_writel_dbi(pcie, PCIE_ATU_LOWER_BASE,
		lower_32_bits(ptrOutb->base_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_UPPER_BASE,
		upper_32_bits(ptrOutb->base_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_LIMIT,
		lower_32_bits(ptrOutb->base_addr + ptrOutb->size - 1));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_LOWER_TARGET,
		lower_32_bits(ptrOutb->target_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_UPPER_TARGET,
		upper_32_bits(ptrOutb->target_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_CR1, ptrOutb->region_type);
	dw_pcie_writel_dbi(pcie, PCIE_ATU_CR2, PCIE_ATU_ENABLE);

	return ret;
}

static int s32v234_pcie_iatu_inbound_set(struct dw_pcie *pcie,
		struct s32v_inbound_region *ptrInb)
{
	int ret = 0;

	if (ptrInb->region > NR_REGIONS - 1)
		return -EINVAL;

	dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT,
		PCIE_ATU_REGION_INBOUND | ptrInb->region);
	dw_pcie_writel_dbi(pcie, PCIE_ATU_LOWER_TARGET,
		lower_32_bits(ptrInb->target_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_UPPER_TARGET,
		upper_32_bits(ptrInb->target_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_CR1,
		PCIE_ATU_TYPE_MEM);
	dw_pcie_writel_dbi(pcie, PCIE_ATU_CR2,
		PCIE_ATU_ENABLE | PCIE_ATU_BAR_MODE_ENABLE |
		PCIE_ATU_BAR_NUM(ptrInb->bar_nr));

	return ret;
}

#ifdef CONFIG_PCI_S32V234_IGNORE_ERR009852
/* User choice: ignore erratum regardless of chip version. */
static bool s32v234_pcie_ignore_err009852(struct s32v234_pcie *s32v234_pp)
{
	return true;
}
#else
/* It is safe to override Kconfig selection if the chip revision is in fact
 * not affected by the erratum.
 * The erratumonly affects chips revision 1.0.
 * We rely on u-boot passing the chip revision along, via the fdt.
 */
static bool s32v234_pcie_ignore_err009852(struct s32v234_pcie *s32v234_pp)
{
	return s32v234_pp->soc_revision > 0;
}
#endif

static void restore_inb_atu(struct dw_pcie *pcie)
{
	int i;

	for (i = 0 ; i < NR_REGIONS ; i++) {
		struct s32v_inbound_region *ptrInb = &restore_inb_arr[i];

		if (ptrInb->target_addr == 0)
			continue;
		s32v234_pcie_iatu_inbound_set(pcie, ptrInb);
	}
}

static void restore_outb_atu(struct dw_pcie *pcie)
{
	int i;

	for (i = 0 ; i < NR_REGIONS ; i++) {
		struct s32v_outbound_region *ptrOutb = &restore_outb_arr[i];

		if (ptrOutb->base_addr == 0)
			continue;
		s32v234_pcie_iatu_outbound_set(pcie, ptrOutb);
	}
}

static int s32v234_get_bar_info(struct s32v234_pcie *s32v234_pp, void __user *argp)
{
	struct s32v_bar bar_info;
	struct dw_pcie *pcie = &s32v234_pp->pcie;
	u8	bar_nr = 0;
	u32 addr = 0;
	int ret = 0;

	if (copy_from_user(&bar_info, argp, sizeof(bar_info))) {
		dev_err(pcie->dev, "Error while copying from user\n");
		return -EFAULT;
	}
	if (bar_info.bar_nr)
		bar_nr = bar_info.bar_nr;

	addr = readl(pcie->dbi_base + (PCI_BASE_ADDRESS_0 +
				bar_info.bar_nr * 4));
	bar_info.addr = addr & 0xFFFFFFF0;
	writel(0xFFFFFFFF, pcie->dbi_base +
		(PCI_BASE_ADDRESS_0 + bar_nr * 4));
	bar_info.size = readl(pcie->dbi_base +
		(PCI_BASE_ADDRESS_0 + bar_nr * 4));
	bar_info.size = ~(bar_info.size & 0xFFFFFFF0) + 1;
	writel(addr, pcie->dbi_base +
		(PCI_BASE_ADDRESS_0 + bar_nr * 4));

	if (copy_to_user(argp, &bar_info, sizeof(bar_info)))
		return -EFAULT;

	return ret;
}

static ssize_t s32v234_ioctl(struct file *filp, u32 cmd,
		unsigned long data)
{
	int ret = 0;
	void __user *argp = (void __user *)data;
	struct s32v234_pcie *s32v234_pp = (struct s32v234_pcie *)(filp->private_data);
#ifdef CONFIG_PCI_DW_DMA
	struct dma_info *di = &(s32v234_pp->dma);
#endif
	struct s32v_inbound_region	inbStr;
	struct s32v_outbound_region	outbStr;

	switch (cmd) {
		/* Call to retrieve BAR setup*/
	case GET_BAR_INFO:
		ret = s32v234_get_bar_info(s32v234_pp, argp);
		break;
	case SETUP_OUTBOUND:
		/* Call to setup outbound region */
		if (copy_from_user(&outbStr, argp, sizeof(outbStr)))
			return -EFAULT;
		store_outb_atu(&outbStr);
		ret = s32v234_pcie_iatu_outbound_set(&s32v234_pp->pcie, &outbStr);
		return ret;
	case SETUP_INBOUND:
		/* Call to setup inbound region */
		if (copy_from_user(&inbStr, argp, sizeof(inbStr)))
			return -EFAULT;
		store_inb_atu(&inbStr);
		ret = s32v234_pcie_iatu_inbound_set(&s32v234_pp->pcie, &inbStr);
		return ret;
	case SEND_MSI:
		/* Send MSI */
		ret = s32v234_setup_MSI(s32v234_pp, argp);
		return ret;
	case STORE_PID:
		ret = s32v234_store_pid(s32v234_pp, argp);
		return ret;
	case SEND_SIGNAL:
		ret = send_signal_to_user(s32v234_pp);
		return ret;
	#ifdef CONFIG_PCI_DW_DMA
	case SEND_SINGLE_DMA:
		ret = s32v234_send_dma_single(s32v234_pp, argp);
		return ret;
	case GET_DMA_CH_ERRORS:
		ret = s32v234_send_dma_errors(di, argp);
		return ret;
	case RESET_DMA_WRITE:
		s32v234_reset_dma_write(s32v234_pp);
		return ret;
	case RESET_DMA_READ:
		s32v234_reset_dma_read(s32v234_pp);
		return ret;
	case STORE_LL_INFO:
		ret = s32v234_store_ll_array_info(di, argp);
		return ret;
	case SEND_LL:
		ret = s32v234_store_ll_array(s32v234_pp, argp);
		return ret;
	case START_LL:
		ret = s32v234_start_dma_ll(s32v234_pp, argp);
		return ret;
	#endif
	default:
		return -EINVAL;
	}
	return ret;
}

static const struct file_operations s32v_pcie_ep_dbgfs_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.unlocked_ioctl = s32v234_ioctl,
};

static void s32v234_pcie_set_bar(struct s32v234_pcie *s32v234_pp,
				 int baroffset, int enable,
				 unsigned int size,
				 unsigned int init)
{
	struct dw_pcie *pcie = &(s32v234_pp->pcie);
	uint32_t mask = (enable) ? ((size - 1) & ~1) : 0;

	/* According to the RM, you have to enable the BAR before you
	 * can modify the mask value. While it appears that this may
	 * be ok in a single write anyway, we play it safe.
	 */
	dw_pcie_writel_dbi(pcie, 0x1000 + baroffset, 1);

	dw_pcie_writel_dbi(pcie, 0x1000 + baroffset, enable | mask);
	dw_pcie_writel_dbi(pcie, baroffset, init);
}

static void s32v234_pcie_setup_ep(struct s32v234_pcie *s32v234_pp)
{
	struct dw_pcie *pcie = &s32v234_pp->pcie;

	/* CMD reg:I/O space, MEM space, and Bus Master Enable */

	/*
	 * configure the class_rev(emulate one memory EP device),
	 * BAR0 and BAR2 of EP
	 */
	dw_pcie_writel_dbi(pcie, PCI_CLASS_REVISION,
		dw_pcie_readl_dbi(pcie, PCI_CLASS_REVISION)
		| ((PCI_BASE_CLASS_PROCESSOR << 24) |
			     (0x80 /* other */ << 16)));

	/* Erratum ERR009852 requires us to avoid
	 * any memory access from the RC! We solve this
	 * by disabling all BARs and ROM access
	 */
	if (s32v234_pcie_ignore_err009852(s32v234_pp)) {
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_0,
				     PCIE_BAR0_EN_DIS,
				     PCIE_BAR0_SIZE,
				     PCIE_BAR0_INIT);
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_1,
				     PCIE_BAR1_EN_DIS,
				     PCIE_BAR1_SIZE,
				     PCIE_BAR1_INIT);
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_2,
				     PCIE_BAR2_EN_DIS,
				     PCIE_BAR2_SIZE,
				     PCIE_BAR2_INIT);
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_3,
				     PCIE_BAR3_EN_DIS,
				     PCIE_BAR3_SIZE,
				     PCIE_BAR3_INIT);
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_4,
				     PCIE_BAR4_EN_DIS,
				     PCIE_BAR4_SIZE,
				     PCIE_BAR4_INIT);
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_5,
				     PCIE_BAR5_EN_DIS,
				     PCIE_BAR5_SIZE,
				     PCIE_BAR5_INIT);
		s32v234_pcie_set_bar(s32v234_pp, PCI_ROM_ADDRESS,
				     PCIE_ROM_EN_DIS,
				     PCIE_ROM_SIZE,
				     PCIE_ROM_INIT);

		dw_pcie_writel_dbi(pcie, PCI_COMMAND,
			dw_pcie_readl_dbi(pcie, PCI_COMMAND)
				| PCI_COMMAND_IO
				| PCI_COMMAND_MEMORY
				| PCI_COMMAND_MASTER);
	} else {
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_0, 0, 0, 0);
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_1,
				     0, 0, 0);
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_2,
				     0, 0, 0);
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_3,
				     0, 0, 0);
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_4,
				     0, 0, 0);
		s32v234_pcie_set_bar(s32v234_pp, PCI_BASE_ADDRESS_5,
				     0, 0, 0);
		s32v234_pcie_set_bar(s32v234_pp, PCI_ROM_ADDRESS,
				     0, 0, 0);
	}
}

int s32v_pcie_setup_outbound(struct s32v_outbound_region *outbStr)
{
	int ret = 0;

	if (!s32v234_pcie_ep)
		return -ENODEV;

	if (!outbStr)
		return -EINVAL;

	/* Call to setup outbound region */
	store_outb_atu(outbStr);
	ret = s32v234_pcie_iatu_outbound_set(&(s32v234_pcie_ep->pcie), outbStr);

	return ret;
}
EXPORT_SYMBOL(s32v_pcie_setup_outbound);

int s32v_pcie_setup_inbound(struct s32v_inbound_region *inbStr)
{
	int ret = 0;

	if (!s32v234_pcie_ep)
		return -ENODEV;

	if (!inbStr)
		return -EINVAL;

	/* Call to setup inbound region */
	store_inb_atu(inbStr);
	ret = s32v234_pcie_iatu_inbound_set(&(s32v234_pcie_ep->pcie), inbStr);
	return ret;
}
EXPORT_SYMBOL(s32v_pcie_setup_inbound);

static int pcie_phy_poll_ack(void __iomem *dbi_base, int exp_val)
{
	u32 val;
	u32 max_iterations = 10;
	u32 wait_counter = 0;

	do {
		val = readl(dbi_base + PCIE_PHY_STAT);
		val = (val >> PCIE_PHY_STAT_ACK_LOC) & 0x1;
		wait_counter++;

		if (val == exp_val)
			return 0;

		udelay(1);
	} while (wait_counter < max_iterations);

	return -ETIMEDOUT;
}

static int pcie_phy_wait_ack(void __iomem *dbi_base, int addr)
{
	u32 val;
	int ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	writel(val, dbi_base + PCIE_PHY_CTRL);

	val |= (0x1 << PCIE_PHY_CTRL_CAP_ADR_LOC);
	writel(val, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	writel(val, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	return 0;
}

/* Read from the 16-bit PCIe PHY control registers (not memory-mapped) */
static int pcie_phy_read(void __iomem *dbi_base, int addr, int *data)
{
	u32 val, phy_ctl;
	int ret;

	ret = pcie_phy_wait_ack(dbi_base, addr);
	if (ret)
		return ret;

	/* assert Read signal */
	phy_ctl = 0x1 << PCIE_PHY_CTRL_RD_LOC;
	writel(phy_ctl, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	val = readl(dbi_base + PCIE_PHY_STAT);
	*data = val & 0xffff;

	/* deassert Read signal */
	writel(0x00, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	return 0;
}

static int pcie_phy_write(void __iomem *dbi_base, int addr, int data)
{
	u32 var;
	int ret;

	/* write addr */
	/* cap addr */
	ret = pcie_phy_wait_ack(dbi_base, addr);
	if (ret)
		return ret;

	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* capture data */
	var |= (0x1 << PCIE_PHY_CTRL_CAP_DAT_LOC);
	writel(var, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	/* deassert cap data */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	/* assert wr signal */
	var = 0x1 << PCIE_PHY_CTRL_WR_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack */
	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	/* deassert wr signal */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	writel(0x0, dbi_base + PCIE_PHY_CTRL);

	return 0;
}

static inline bool is_S32V234_pcie(struct s32v234_pcie *s32v234_pp)
{
	struct dw_pcie *pcie = &s32v234_pp->pcie;
	struct device_node *np	= pcie->dev->of_node;

	return of_device_is_compatible(np, "fsl,s32v234-pcie");
}

static void s32v234_pcie_assert_core_reset(struct s32v234_pcie *s32v234_pp)
{
	if (is_S32V234_pcie(s32v234_pp)) {
		regmap_update_bits(s32v234_pp->src, SRC_GPR5,
				SRC_GPR5_GPR_PCIE_BUTTON_RST_N,
				SRC_GPR5_GPR_PCIE_BUTTON_RST_N);
	}
}

static void s32v234_pcie_deassert_core_reset(struct s32v234_pcie *s32v234_pp)
{
	/* allow the clocks to stabilize */
	mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
	/*
	 * Release the PCIe PHY reset here, that we have set in
	 * s32v234_pcie_init_phy() now
	 */
	if (is_S32V234_pcie(s32v234_pp))
		regmap_update_bits(s32v234_pp->src, SRC_GPR5,
		SRC_GPR5_GPR_PCIE_BUTTON_RST_N, 0);
	mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
}

/* Perform a soft-reset of the PCIE core. Needed e.g. in the case of a
 * 'link_req_rst_not' interrupt.
 */
static void s32v234_pcie_soft_reset(struct s32v234_pcie *s32v234_pp)
{
	/* Temporarily deassert 'app_ltssm_enable' */
	regmap_update_bits(s32v234_pp->src, SRC_GPR5,
			   SRC_GPR5_PCIE_APP_LTSSM_ENABLE, 0);
	mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
	regmap_update_bits(s32v234_pp->src, SRC_GPR5,
			   SRC_GPR5_PCIE_APP_LTSSM_ENABLE,
			   SRC_GPR5_PCIE_APP_LTSSM_ENABLE);

	/* Reset PCIE core */
	s32v234_pcie_assert_core_reset(s32v234_pp);
	mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
	s32v234_pcie_deassert_core_reset(s32v234_pp);
	mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
}

static int s32v234_pcie_init_phy(struct s32v234_pcie *s32v234_pp)
{
	regmap_update_bits(s32v234_pp->src, SRC_GPR5,
			SRC_GPR5_PCIE_APP_LTSSM_ENABLE, 0 << 10);
	mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
	regmap_update_bits(s32v234_pp->src, SRC_GPR5,
			SRC_GPR5_PCIE_DEVICE_TYPE_MASK,
			PCI_EXP_TYPE_ROOT_PORT << 1);
	mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
	regmap_update_bits(s32v234_pp->src, SRC_GPR5,
			SRC_GPR5_PCIE_PHY_LOS_LEVEL_MASK, (0x9 << 22));
	mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
	return 0;
}

static int s32v234_pcie_wait_for_link(struct s32v234_pcie *s32v234_pp)
{
	struct dw_pcie *pcie = &s32v234_pp->pcie;

	/* check if the link is up or not */
	if (!dw_pcie_wait_for_link(pcie))
		return 0;

	dev_err(pcie->dev, "phy link never came up\n");
	dev_info(pcie->dev, "DEBUG_R0: 0x%08x, DEBUG_R1: 0x%08x\n",
		dw_pcie_readl_dbi(pcie, PCIE_PHY_DEBUG_R0),
		dw_pcie_readl_dbi(pcie, PCIE_PHY_DEBUG_R1));
	return -ETIMEDOUT;
}

#ifdef CONFIG_PCI_MSI
static irqreturn_t s32v234_pcie_msi_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_msi_irq(pp);
}
#endif

static int s32v234_pcie_start_link(struct s32v234_pcie *s32v234_pp)
{
	struct dw_pcie *pcie = &s32v234_pp->pcie;
	uint32_t tmp;
	int ret, count;

	/*
	 * Force Gen1 operation when starting the link.  In case the link is
	 * started in Gen2 mode, there is a possibility the devices on the
	 * bus will not be detected at all.  This happens with PCIe switches.
	 */
	tmp = dw_pcie_readl_dbi(pcie, PCIE_RC_LCR);
	tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
	tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1;
	dw_pcie_writel_dbi(pcie, PCIE_RC_LCR, tmp);

	/* Start LTSSM. */

	regmap_update_bits(s32v234_pp->src, SRC_GPR5,
			SRC_GPR5_PCIE_APP_LTSSM_ENABLE,
			SRC_GPR5_PCIE_APP_LTSSM_ENABLE);

	ret = s32v234_pcie_wait_for_link(s32v234_pp);

	udelay(200);
	if (ret)
		goto out;

	/* Allow Gen2 mode after the link is up. */
	tmp = dw_pcie_readl_dbi(pcie, PCIE_RC_LCR);
	tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
	tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2;
	dw_pcie_writel_dbi(pcie, PCIE_RC_LCR, tmp);

	/*
	 * Start Directed Speed Change so the best possible speed both link
	 * partners support can be negotiated.
	 */
	tmp = dw_pcie_readl_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL);
	tmp |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL, tmp);

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
		ret = s32v234_pcie_wait_for_link(s32v234_pp);
	else {
		dev_err(pcie->dev, "Speed change timeout\n");
		ret = -EINVAL;
	}

out:
	if (ret) {
		dev_err(pcie->dev, "Failed to bring link up!\n");
	} else {
		tmp = dw_pcie_readl_dbi(pcie, PCIE_RC_LCSR);
		dev_dbg(pcie->dev, "Link up, Gen=%i\n", (tmp >> 16) & 0xf);
	}
	return ret;
}

static int s32v234_pcie_host_init(struct pcie_port *pp)
{
	int socmask_info;
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct s32v234_pcie *s32v234_pp = to_s32v234_from_dw_pcie(pcie);

	/* enable disp_mix power domain */
	pm_runtime_get_sync(pcie->dev);

	s32v234_pcie_assert_core_reset(s32v234_pp);
	s32v234_pcie_init_phy(s32v234_pp);
	s32v234_pcie_deassert_core_reset(s32v234_pp);

	/* We set up the ID for all Rev 1.x chips */
	socmask_info = s32v234_pp->soc_revision;
	if (socmask_info >= 0) {
		dev_info(pcie->dev, "SOC revision: 0x%x\n", socmask_info);
		if ((socmask_info & SOC_REVISION_MAJOR_MASK) == 0) {
			/*
			 * Vendor ID is Freescale (now NXP): 0x1957
			 * Device ID is split as follows
			 * Family 15:12, Device 11:6, Personality 5:0
			 * S32V is in the automotive family: 0100
			 * S32V is the first auto device with PCIe: 000000
			 * S32V has not export controlled cryptography: 00001
			 */
			dev_info(pcie->dev,
				 "Setting PCIE Vendor and Device ID\n");
			dw_pcie_writel_dbi(pcie, PCI_VENDOR_ID,
				(0x4001 << 16) | 0x1957);
		}
	}
	dw_pcie_setup_rc(pp);

	s32v234_pcie_start_link(s32v234_pp);

#ifdef CONFIG_PCI_MSI
	dw_pcie_msi_init(pp);
#endif

	return 0;
}

static void s32v234_pcie_reset_phy(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	uint32_t temp;

	pcie_phy_read(pcie->dbi_base, PHY_RX_OVRD_IN_LO, &temp);
	temp |= (PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		 PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(pcie->dbi_base, PHY_RX_OVRD_IN_LO, temp);

	udelay(2000);

	pcie_phy_read(pcie->dbi_base, PHY_RX_OVRD_IN_LO, &temp);
	temp &= ~(PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		  PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(pcie->dbi_base, PHY_RX_OVRD_IN_LO, temp);
}

static int s32v234_pcie_link_up(struct dw_pcie *pcie)
{
	u32 rc, debug_r0, rx_valid;
	int count = 15000;
	struct pcie_port *pp = &pcie->pp;

	/*
	 * Test if the PHY reports that the link is up and also that the LTSSM
	 * training finished. There are three possible states of the link when
	 * this code is called:
	 * 1) The link is DOWN (unlikely)
	 *     The link didn't come up yet for some reason. This usually means
	 *     we have a real problem somewhere. Reset the PHY and exit. This
	 *     state calls for inspection of the DEBUG registers.
	 * 2) The link is UP, but still in LTSSM training
	 *     Wait for the training to finish, which should take a very short
	 *     time. If the training does not finish, we have a problem and we
	 *     need to inspect the DEBUG registers. If the training does finish,
	 *     the link is up and operating correctly.
	 * 3) The link is UP and no longer in LTSSM training
	 *     The link is up and operating correctly.
	 */
	while (1) {
		rc = readl(pcie->dbi_base + PCIE_PHY_DEBUG_R1);
		if (!(rc & PCIE_PHY_DEBUG_R1_XMLH_LINK_UP))
			break;
		if (!(rc & PCIE_PHY_DEBUG_R1_XMLH_LINK_IN_TRAINING))
			return 1;
		if (!count--)
			break;
		dev_dbg(pcie->dev, "Link is up, but still in training\n");
		/*
		 * Wait a little bit, then re-check if the link finished
		 * the training.
		 */
		udelay(10);
	}
	/*
	 * From L0, initiate MAC entry to gen2 if EP/RC supports gen2.
	 * Wait 2ms (LTSSM timeout is 24ms, PHY lock is ~5us in gen2).
	 * If (MAC/LTSSM.state == Recovery.RcvrLock)
	 * && (PHY/rx_valid==0) then pulse PHY/rx_reset. Transition
	 * to gen2 is stuck
	 */
	pcie_phy_read(pcie->dbi_base, PCIE_PHY_RX_ASIC_OUT, &rx_valid);
	debug_r0 = readl(pcie->dbi_base + PCIE_PHY_DEBUG_R0);

	if (rx_valid & 0x01)
		return 0;

	if ((debug_r0 & 0x3f) != 0x0d)
		return 0;

	dev_err(pcie->dev, "transition to gen2 is stuck, reset PHY!\n");
	dev_dbg(pcie->dev, "debug_r0=%08x debug_r1=%08x\n", debug_r0, rc);

	s32v234_pcie_reset_phy(pp);

	return 0;
}

static struct dw_pcie_ops s32v234_pcie_ops = {
	.link_up = s32v234_pcie_link_up,
};

static struct dw_pcie_host_ops s32v234_pcie_host_ops = {
	.host_init = s32v234_pcie_host_init,
};

static int __init s32v234_add_pcie_port(struct pcie_port *pp,
			struct platform_device *pdev)
{
	int ret;
#ifdef CONFIG_PCI_MSI
	pp->msi_irq = platform_get_irq_byname(pdev, "msi");
	if (pp->msi_irq <= 0) {
		dev_err(&pdev->dev, "failed to get MSI irq\n");
		return -ENODEV;
	}

	ret = devm_request_irq(&pdev->dev, pp->msi_irq,
		s32v234_pcie_msi_handler,
		IRQF_SHARED, "s32v-pcie-msi", pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request MSI irq\n");
		return -ENODEV;
	}
	dev_info(&pdev->dev, "Allocated line %d for interrupt %d",
		ret, pp->msi_irq);
#endif

	pp->ops = &s32v234_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static void s32v234_pcie_shutdown(struct platform_device *pdev)
{
	struct s32v234_pcie *s32v234_pp = platform_get_drvdata(pdev);

	if (!s32v234_pp->is_endpoint) {
		/* bring down link, so bootloader gets clean state
		 * in case of reboot
		 */

		devm_free_irq(&pdev->dev,
			s32v234_pp->link_req_rst_not_irq,
			s32v234_pp);

		s32v234_pcie_assert_core_reset(s32v234_pp);
		mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
	}
}

/* link_req_rst_not IRQ handler */
static irqreturn_t s32v234_pcie_link_req_rst_not_handler(int irq, void *arg)
{
	struct s32v234_pcie *s32v234_pp = arg;
	u32 rc;
	struct dw_pcie *pcie = &s32v234_pp->pcie;

	regmap_update_bits(s32v234_pp->src, SRC_PCIE_CONFIG0,
			   SRC_CONFIG0_PCIE_LNK_REQ_RST_CLR, 1);

	/* Handler code for EP */
	if (s32v234_pp->is_endpoint) {

		s32v234_pcie_setup_ep(s32v234_pp);
		regmap_update_bits(s32v234_pp->src, SRC_GPR11,
				SRC_GPR11_PCIE_PCIE_CFG_READY,
				SRC_GPR11_PCIE_PCIE_CFG_READY);
		if (s32v234_pcie_ignore_err009852(s32v234_pp)) {
			restore_inb_atu(pcie);
			restore_outb_atu(pcie);
		}

		goto done;
	}

	/* Handler code for RC */
	/* Note that this interrupt can be shared - e.g. with a USB-PCI device.
	 * However, we can't read the "link_req_rst_not" signal directly;
	 * our best heuristic is to look at the PHY link state and only
	 * if it is down acknowledge the interrupt as ours.
	 */
	rc = dw_pcie_readl_dbi(pcie, PCIE_PHY_DEBUG_R1);
	if ((rc & PCIE_PHY_DEBUG_R1_XMLH_LINK_UP) ||
	    (rc & PCIE_PHY_DEBUG_R1_XMLH_LINK_IN_TRAINING))
		return IRQ_NONE;

	/* Must reset the PCIE core, according to the reference manual */
	s32v234_pcie_soft_reset(s32v234_pp);

done:

	return IRQ_HANDLED;
}

struct s32v234_pcie *s32v_get_dw_pcie(void)
{
	return s32v234_pcie_ep;
}
EXPORT_SYMBOL(s32v_get_dw_pcie);

static int s32v234_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s32v234_pcie *s32v234_pp;
	struct resource *dbi_base;
	struct dw_pcie *pcie;
	struct pcie_port *pp;

	int ret;
	unsigned int src_gpr5, pcie_device_type, ltssm_en;

	s32v234_pp = devm_kzalloc(dev, sizeof(*s32v234_pp), GFP_KERNEL);
	if (!s32v234_pp)
		return -ENOMEM;

	pcie = &(s32v234_pp->pcie);
	pp = &(pcie->pp);

	pcie->dev = dev;
	pcie->ops = &s32v234_pcie_ops;

	/* Added for PCI abort handling */
	dbi_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pcie->dbi_base = devm_ioremap_resource(dev, dbi_base);
	if (IS_ERR(pcie->dbi_base))
		return PTR_ERR(pcie->dbi_base);

	/* Grab SRC config register range */
	s32v234_pp->src =
		 syscon_regmap_lookup_by_compatible("fsl,s32v234-src");
	if (IS_ERR(s32v234_pp->src)) {
		dev_err(&pdev->dev, "unable to find SRC registers\n");
		return PTR_ERR(s32v234_pp->src);
	}

	ret = regmap_read(s32v234_pp->src, SRC_GPR5, &src_gpr5);
	if (ret) {
		dev_err(&pdev->dev, "could not read SRC_GPR5 register\n");
		return -ENODEV;
	}

	pcie_device_type = src_gpr5 & SRC_GPR5_PCIE_DEVICE_TYPE_MASK;
	s32v234_pp->is_endpoint =
		(pcie_device_type != SRC_GPR5_PCIE_DEVICE_TYPE_RC);

	/* Attempt to figure out whether u-boot has preconfigured PCIE; if it
	 * did not, we will not be able to tell whether we should run as EP
	 * (whose configuration value is the same as the reset value) or RC.
	 * Failing to do so might result in a hardware freeze, if u-boot was
	 * compiled without PCIE support at all.
	 *
	 * Test SRC_GPR5:GPR_PCIE_APP_LTSSM_ENABLE, whose reset value
	 * is different from the value set by u-boot.
	 */
	ltssm_en = src_gpr5 & SRC_GPR5_PCIE_APP_LTSSM_ENABLE;
	if (!ltssm_en) {
		dev_info(&pdev->dev,
			 "u-boot did not initialize PCIE PHY; is u-boot compiled with PCIE support?\n");
		return -ENODEV;
	}

	dev_info(dev, "Configuring as %s\n",
		 (s32v234_pp->is_endpoint) ? "EP" : "RC");

	s32v234_pp->soc_revision = s32v234_pcie_get_soc_revision();

	if (!s32v234_pp->is_endpoint) {
		ret = s32v234_add_pcie_port(pp, pdev);
		if (ret < 0)
			return ret;
	} else {
		struct dentry *pfile;

		s32v234_pcie_ep = s32v234_pp;

		s32v234_pp->call_back = NULL;
		s32v234_pp->user_pid = 0;

#ifdef CONFIG_PCI_DW_DMA
		s32v234_pp->dma_irq = platform_get_irq_byname(pdev, "dma");
		if (s32v234_pp->dma_irq <= 0) {
			dev_err(&pdev->dev, "failed to get DMA irq\n");
			return -ENODEV;
		}
		ret = devm_request_irq(&pdev->dev, s32v234_pp->dma_irq,
			s32v234_pcie_dma_handler,
			IRQF_SHARED, "s32v-pcie-dma", s32v234_pp);
		if (ret) {
			dev_err(&pdev->dev, "failed to request DMA irq\n");
			return -ENODEV;
		}
		dw_pcie_dma_clear_regs(pcie, &s32v234_pp->dma);

		memset(&s32v234_pp->info, 0, sizeof(struct siginfo));
		s32v234_pp->info.si_signo = SIGUSR1;
		s32v234_pp->info.si_code = SI_USER;
		s32v234_pp->info.si_int = 0;
#endif /* CONFIG_PCI_DW_DMA */

		s32v234_pp->dir = debugfs_create_dir("ep_dbgfs", NULL);
		if (!s32v234_pp->dir)
			dev_info(dev, "Creating debugfs dir failed\n");
		pfile = debugfs_create_file("ep_file", 0444, s32v234_pp->dir,
			(void *)s32v234_pp, &s32v_pcie_ep_dbgfs_fops);
		if (!pfile)
			dev_info(dev, "debugfs regs for failed\n");

		writel((readl(pcie->dbi_base + PCIE_MSI_CAP) | 0x10000),
			pcie->dbi_base +  PCIE_MSI_CAP);
	}

	s32v234_pp->link_req_rst_not_irq = platform_get_irq_byname(pdev,
					"link_req_rst_not");
	if (s32v234_pp->link_req_rst_not_irq <= 0) {
		dev_err(&pdev->dev, "failed to get link_req_rst_not irq\n");
		return -ENODEV;
	}
	ret = devm_request_irq(&pdev->dev, s32v234_pp->link_req_rst_not_irq,
		s32v234_pcie_link_req_rst_not_handler,
		IRQF_SHARED, "link_req_rst_not", s32v234_pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request link_req_rst_not irq\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, s32v234_pp);

	return 0;
}

static const struct of_device_id s32v234_pcie_of_match[] = {
	{ .compatible = "fsl,s32v234-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, s32v234_pcie_of_match);

static struct platform_driver s32v234_pcie_driver = {
	.driver = {
		.name	= "s32v234-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = s32v234_pcie_of_match,
	},
	.probe = s32v234_pcie_probe,
	.shutdown = s32v234_pcie_shutdown,
};

/* Freescale PCIe driver does not allow module unload */

static int __init s32v234_pcie_init(void)
{
	return platform_driver_probe(&s32v234_pcie_driver, s32v234_pcie_probe);
}
module_init(s32v234_pcie_init);

MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("Freescale S32V PCIe host controller driver");
MODULE_LICENSE("GPL v2");
