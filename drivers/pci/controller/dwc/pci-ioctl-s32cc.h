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

#ifndef PCI_IOCTL_S32CC_H
#define PCI_IOCTL_S32CC_H

#define SEND_MSI			_IOWR('S', 3, unsigned long long)
#define SEND_SINGLE_DMA		_IOWR('S', 6, struct dma_data_elem)
#define STORE_PID			_IOR('S', 7, unsigned int)
#define SEND_SIGNAL			_IOR('S', 8,  unsigned int)
#define GET_DMA_CH_ERRORS	_IOR('S', 9,  unsigned int)
#define RESET_DMA_WRITE		_IOW('S', 10,  unsigned int)
#define RESET_DMA_READ		_IOW('S', 11,  unsigned int)

#define DMA_FLAG_LIE					BIT(0)
#define DMA_FLAG_RIE					BIT(1)
#define DMA_FLAG_LLP					BIT(2)
#define DMA_FLAG_WRITE_ELEM				BIT(3)
#define DMA_FLAG_READ_ELEM				BIT(4)
#define DMA_FLAG_EN_DONE_INT			BIT(5)
#define DMA_FLAG_EN_ABORT_INT			BIT(6)
#define DMA_FLAG_EN_REMOTE_DONE_INT		BIT(7)
#define DMA_FLAG_EN_REMOTE_ABORT_INT	BIT(8)

/* Single block DMA transfer struct */
struct dma_data_elem {
	unsigned long sar;
	unsigned long dar;
	unsigned long imwr;
	unsigned int size;
	unsigned int flags;
	unsigned int ch_num;
};

#endif /* PCI_IOCTL_S32CC_H */
