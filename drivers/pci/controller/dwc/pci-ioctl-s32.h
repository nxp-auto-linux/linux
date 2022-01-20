/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2013 Kosagi
 *              http://www.kosagi.com
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2016-2022 NXP
 *
 * Author: Sean Cross <xobs@kosagi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef PCI_IOCTL_S32_H
#define PCI_IOCTL_S32_H

#include <linux/signal_types.h>
#include "pcie-designware.h"

#define SETUP_OUTBOUND		_IOWR('S', 1, struct s32_outbound_region)
#define SETUP_INBOUND		_IOWR('S', 2, struct s32_inbound_region)
#define SEND_MSI			_IOWR('S', 3, unsigned long long)
#define GET_BAR_INFO		_IOWR('S', 4, struct s32_bar)
#define SEND_SINGLE_DMA		_IOWR('S', 6, struct dma_data_elem)
#define STORE_PID			_IOR('S', 7, unsigned int)
#define SEND_SIGNAL			_IOR('S', 8,  unsigned int)
#define GET_DMA_CH_ERRORS	_IOR('S', 9,  unsigned int)
#define RESET_DMA_WRITE		_IOW('S', 10,  unsigned int)
#define RESET_DMA_READ		_IOW('S', 11,  unsigned int)

struct s32_inbound_region {
#ifndef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT
	int pcie_id; /* must match the id of a device tree pcie node */
#endif
	u32 bar_nr;
	u32 target_addr;
	u32 region; /* for backwards compatibility */
};

struct s32_outbound_region {
#ifndef CONFIG_PCI_S32GEN1_IOCTL_LIMIT_ONE_ENDPOINT
	int pcie_id; /* must match the id of a device tree pcie node */
#endif
	u64 target_addr;
	u64 base_addr;
	u32 size;
	/* region and region_type - for backwards compatibility;
	 * region_type must be PCIE_ATU_TYPE_MEM
	 */
	u32 region;
	u32 region_type;
};

struct s32_bar {
	u32 bar_nr;
	u32 size;
	u32 addr;
};

struct s32_userspace_info;

struct s32_userspace_info {
	struct dentry		*dir;
	int		user_pid;
	struct kernel_siginfo	info;    /* signal information */
	int (*send_signal_to_user)(struct s32_userspace_info *s32_user);
};

void s32_config_user_space_data(struct s32_userspace_info *uinfo,
	struct dw_pcie *pcie);

/* Following functions need to be implemented in each driver supporting
 * this ioctl API, since it requires access to some internal data not
 * accessible at this level
 */

struct s32_userspace_info *dw_get_userspace_info(struct dw_pcie *pcie);

int s32_pcie_setup_outbound(struct s32_outbound_region *outbStr);
int s32_pcie_setup_inbound(struct s32_inbound_region *inbStr);

int s32_send_msi(struct dw_pcie *pcie);

void s32_register_callback(struct dw_pcie *pcie,
	void (*call_back)(u32 arg));

#endif /* PCI_IOCTL_S32_H */
