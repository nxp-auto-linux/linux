/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2013 Kosagi
 *              http://www.kosagi.com
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2016-2021 NXP
 *
 * Author: Sean Cross <xobs@kosagi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef PCI_IOCTL_S32_H
#define PCI_IOCTL_S32_H

#define SETUP_OUTBOUND		_IOWR('S', 1, struct s32v_outbound_region)
#define SETUP_INBOUND		_IOWR('S', 2, struct s32v_inbound_region)
#define SEND_MSI			_IOWR('S', 3, unsigned long long)
#define GET_BAR_INFO		_IOWR('S', 4, struct s32v_bar)
#define SEND_SINGLE_DMA		_IOWR('S', 6, struct dma_data_elem)
#define STORE_PID			_IOR('S', 7, unsigned int)
#define SEND_SIGNAL			_IOR('S', 8,  unsigned int)
#define GET_DMA_CH_ERRORS	_IOR('S', 9,  unsigned int)
#define RESET_DMA_WRITE		_IOW('S', 10,  unsigned int)
#define RESET_DMA_READ		_IOW('S', 11,  unsigned int)
#define STORE_LL_INFO		_IOR('S', 12,  struct dma_ll_info)
#define SEND_LL				_IOWR('S', 13, struct dma_list(*)[])
#define START_LL			_IOWR('S', 14, unsigned int)

#define DMA_FLAG_LIE					(1 << 0)
#define DMA_FLAG_RIE					(1 << 1)
#define DMA_FLAG_LLP					(1 << 2)
#define DMA_FLAG_WRITE_ELEM				(1 << 3)
#define DMA_FLAG_READ_ELEM				(1 << 4)
#define DMA_FLAG_EN_DONE_INT			(1 << 5)
#define DMA_FLAG_EN_ABORT_INT			(1 << 6)
#define DMA_FLAG_EN_REMOTE_DONE_INT		(1 << 7)
#define DMA_FLAG_EN_REMOTE_ABORT_INT	(1 << 8)

struct s32v_inbound_region {
	unsigned int  bar_nr;
	unsigned int  target_addr;
	unsigned int  region;
};

struct s32v_outbound_region {
	unsigned long target_addr;
	unsigned long base_addr;
	unsigned int  size;
	unsigned int  region;
	unsigned int  region_type;
};

struct s32v_bar {
	unsigned int bar_nr;
	unsigned int size;
	unsigned int addr;
};

/* Single block DMA transfer struct */
struct dma_data_elem {
	unsigned long sar;
	unsigned long dar;
	unsigned long imwr;
	unsigned int size;
	unsigned int flags;
	unsigned int ch_num;
};
/* Type of array of structures for passing linked list  */
struct dma_list {
	unsigned long sar;
	unsigned long dar;
	unsigned int size;
};

/* Linked list mode struct */
struct dma_ll_info {
	unsigned int direction;
	unsigned int ch_num;
	unsigned int nr_elem;
	unsigned int phy_list_addr;
	unsigned int next_phy_list_addr;
};

#endif /* PCI_IOCTL_S32_H */
