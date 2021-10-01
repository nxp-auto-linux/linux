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

#ifndef _PCIE_S32V234_H
#define _PCIE_S32V234_H

#include "pcie-designware.h"
#include "pci-dma-s32.h"

#define to_s32v234_from_dw_pcie(x) \
	container_of(x, struct s32v234_pcie, pcie)

struct s32v234_pcie {
	bool			is_endpoint;
	int			soc_revision;
	int			reset_gpio;
	int			power_on_gpio;
	struct dw_pcie	pcie;
	struct regmap		*src;

	int			link_req_rst_not_irq;
	struct dentry		*dir;
	int			user_pid;
	struct kernel_siginfo	info;    /* signal information */
	void (*call_back)(u32 arg);
	int (*send_signal_to_user)(struct s32v234_pcie *s32v234_pcie);

#ifdef CONFIG_PCI_DW_DMA
	int	dma_irq;
	struct dma_info	dma;
#endif
};

#endif  /* _PCIE_S32V234_H */
