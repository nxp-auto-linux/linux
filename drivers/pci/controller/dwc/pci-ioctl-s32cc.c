// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe ioctl handler for Freescale S32CC SoCs
 *
 * Copyright (C) 2013 Kosagi
 *		http://www.kosagi.com
 *
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2017-2023 NXP
 */

#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include "pcie-designware.h"
#include "pci-dma-s32cc.h"
#include "pci-ioctl-s32cc.h"

static struct task_struct *task;

#define PCIE_BAR_ADDRESS_MASK	GENMASK(31, 4)
#define PCIE_BAR_MASK			GENMASK(31, 0)

static int s32cc_send_dma_errors(struct dma_info *di, void __user *argp)
{
	if (IS_ENABLED(CONFIG_PCI_DW_DMA)) {
		u32 dma_errors;

		if (!argp)
			return -EFAULT;

		dma_errors = ((di->wr_ch.errors) << 16) | di->rd_ch.errors;

		if (copy_to_user(argp, &dma_errors, sizeof(u32)))
			return -EFAULT;
		return 0;
	}

	pr_info("%s: DMA not enabled\n", __func__);
	return -EINVAL;
}

static int s32cc_send_dma_single(struct dma_info *di, void __user *argp)
{
	if (IS_ENABLED(CONFIG_PCI_DW_DMA)) {
		struct dma_data_elem dma_elem_local;

		if (!argp)
			return -EFAULT;

		if (copy_from_user(&dma_elem_local, argp,
				sizeof(struct dma_data_elem)))
			return -EFAULT;

		return dw_pcie_dma_single_rw(di, &dma_elem_local);
	}

	pr_info("%s: DMA not enabled\n", __func__);
	return -EINVAL;
}

static int s32cc_dma_write_soft_reset(struct dma_info *di)
{
	if (IS_ENABLED(CONFIG_PCI_DW_DMA))
		return dw_pcie_dma_write_soft_reset(di);

	pr_info("%s: DMA not enabled\n", __func__);
	return -EINVAL;
}

static int s32cc_dma_read_soft_reset(struct dma_info *di)
{
	if (IS_ENABLED(CONFIG_PCI_DW_DMA))
		return dw_pcie_dma_read_soft_reset(di);

	pr_info("%s: DMA not enabled\n", __func__);
	return -EINVAL;
}

static int send_signal_to_user(struct s32cc_userspace_info *uinfo)
{
	int ret = 0;

	if (!uinfo)
		return -EFAULT;

	if (uinfo->user_pid <= 0)
		return 0;

	rcu_read_lock();
	task = pid_task(find_pid_ns(uinfo->user_pid, &init_pid_ns),
					PIDTYPE_PID);
	rcu_read_unlock();

	ret = send_sig_info(SIGUSR1, &uinfo->info, task);
	if (ret < 0)
		ret = -EFAULT;

	return ret;
}

static int s32cc_store_pid(struct s32cc_userspace_info *uinfo,
			   void __user *argp)
{
	int ret = 0;

	if (!argp)
		return -EFAULT;

	if (copy_from_user(&uinfo->user_pid, argp,
			sizeof(uinfo->user_pid)))
		ret = -EFAULT;

	return ret;
}

static int s32cc_get_bar_info(struct dw_pcie *pcie, void __user *argp)
{
	struct s32cc_bar bar_info;
	u8	bar_nr = 0;
	u32 addr = 0;
	int ret = 0;

	if (!argp)
		return -EFAULT;

	if (copy_from_user(&bar_info, argp, sizeof(bar_info))) {
		dev_err(pcie->dev, "Error while copying from user\n");
		return -EFAULT;
	}
	if (bar_info.bar_nr)
		bar_nr = bar_info.bar_nr;

	addr = dw_pcie_readl_dbi(pcie, (PCI_BASE_ADDRESS_0 +
				bar_info.bar_nr * 4));
	bar_info.addr = addr & PCIE_BAR_ADDRESS_MASK;
	dw_pcie_writel_dbi(pcie, (PCI_BASE_ADDRESS_0 + bar_nr * 4),
			PCIE_BAR_MASK);
	bar_info.size = dw_pcie_readl_dbi(pcie,
		(PCI_BASE_ADDRESS_0 + bar_nr * 4));
	bar_info.size = ~(bar_info.size & PCIE_BAR_ADDRESS_MASK) + 1;
	dw_pcie_writel_dbi(pcie,
		(PCI_BASE_ADDRESS_0 + bar_nr * 4), addr);

	if (copy_to_user(argp, &bar_info, sizeof(bar_info)))
		return -EFAULT;

	return ret;
}

static ssize_t s32cc_ioctl(struct file *filp, u32 cmd,
		unsigned long data)
{
	ssize_t ret = 0;
	void __user *argp = (void __user *)data;
	struct dw_pcie *pcie = (struct dw_pcie *)(filp->private_data);
	struct s32cc_userspace_info *uinfo = dw_get_userspace_info(pcie);
	struct dma_info *di = dw_get_dma_info(pcie);

	struct s32cc_inbound_region	inbStr;
	struct s32cc_outbound_region	outbStr;

	switch (cmd) {
		/* Call to retrieve BAR setup*/
	case GET_BAR_INFO:
		ret = s32cc_get_bar_info(pcie, argp);
		break;
	case SETUP_OUTBOUND:
		/* Call to setup outbound region */
		if (copy_from_user(&outbStr, argp, sizeof(outbStr)))
			return -EFAULT;
		ret = s32cc_pcie_setup_outbound(&outbStr);
		return ret;
	case SETUP_INBOUND:
		/* Call to setup inbound region */
		if (copy_from_user(&inbStr, argp, sizeof(inbStr)))
			return -EFAULT;
		ret = s32cc_pcie_setup_inbound(&inbStr);
		return ret;
	case SEND_MSI:
		/* Setup MSI */
		/* TODO: allow selection of the MSI index and
		 * also handle it on the receiver side
		 */
		ret = s32cc_send_msi(pcie);
		return ret;
	case STORE_PID:
		ret = s32cc_store_pid(uinfo, argp);
		return ret;
	case SEND_SIGNAL:
		ret = send_signal_to_user(uinfo);
		return ret;
	case SEND_SINGLE_DMA:
		ret = s32cc_send_dma_single(di, argp);
		return ret;
	case GET_DMA_CH_ERRORS:
		ret = s32cc_send_dma_errors(di, argp);
		return ret;
	case RESET_DMA_WRITE:
		ret = s32cc_dma_write_soft_reset(di);
		return ret;
	case RESET_DMA_READ:
		ret = s32cc_dma_read_soft_reset(di);
		return ret;

	default:
		return -EINVAL;
	}
	return ret;
}

static const struct file_operations s32cc_pcie_ep_dbgfs_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.unlocked_ioctl = s32cc_ioctl,
};

void s32cc_config_user_space_data(struct s32cc_userspace_info *uinfo,
		struct dw_pcie *pcie)
{
	struct dentry *pfile;

	uinfo->send_signal_to_user = send_signal_to_user;
	uinfo->user_pid = 0;

	/* Init signal info data */
	memset(&uinfo->info, 0, sizeof(struct kernel_siginfo));
	uinfo->info.si_signo = SIGUSR1;
	uinfo->info.si_code = SI_USER;
	uinfo->info.si_int = 0;

	/* Init debugfs entry */
	uinfo->dir = debugfs_create_dir("ep_dbgfs", NULL);
	if (!uinfo->dir)
		dev_info(pcie->dev, "Creating debugfs dir failed\n");
	pfile = debugfs_create_file("ep_file", 0444, uinfo->dir,
		(void *)pcie, &s32cc_pcie_ep_dbgfs_fops);
	if (!pfile)
		dev_info(pcie->dev, "debugfs regs for failed\n");
}
