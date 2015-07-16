// SPDX-License-Identifier: GPL-2.0-or-later
/*
* Copyright (C) 2012 Freescale Semiconductor, Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#ifndef __LINUX_S32V234SRC_GPR_H
#define __LINUX_S32V234SRC_GPR_H

#include <linux/bitops.h>

#define SRC_GPR3 0x108
#define SRC_GPR5 0x110

#define SRC_GPR5_PCIE_DEVICE_TYPE_EP				(0x0 << 1)
#define SRC_GPR5_PCIE_DEVICE_TYPE_RC				(0x4 << 1)
#define SRC_GPR5_PCIE_DEVICE_TYPE_MASK				(0xf << 1)

#define SRC_GPR5_PCIE_DIAG_CTRL_BUS_MASK			(0x7 << 5)
#define SRC_GPR5_GPR_PCIE_SYS_INT					(1<<8)
#define SRC_GPR5_PCIE_APP_LTSSM_ENABLE				(1<<9)
#define SRC_GPR5_GPR_PCIE_APP_INIT_RST				(1<<11)
#define SRC_GPR5_GPR_PCIE_APP_REQ_ENTR_L1			(1<<12)
#define SRC_GPR5_GPR_PCIE_APP_READY_ENTR_L23		(1<<13)
#define SRC_GPR5_GPR_PCIE_APP_REQ_EXIT_L1			(1<<14)
#define SRC_GPR5_GPR_PCIE_BUTTON_RST_N				(1<<15)
#define SRC_GPR5_GPR_PCIE_PERST_N					(1<<16)
#define SRC_GPR5_PCIE_APPS_PM_XMT_TURNOFF			(1<<17)
#define SRC_GPR5_PCIE_PHY_LOS_BIAS_MASK				(0x7 << 19)

#define SRC_GPR5_PCIE_PHY_LOS_LEVEL_9				(0x9 << 22)
#define SRC_GPR5_PCIE_PHY_LOS_LEVEL_MASK			(0x1f << 22)

#define SRC_GPR5_PCIE_PHY_RX0_EQ_2					(0x2 << 27)
#define SRC_GPR5_PCIE_PHY_RX0_EQ_MASK				(0x7 << 27)

#define SRC_GPR6_PCIE_PCS_TX_DEEMPH_GEN1_MASK			(0x3f << 12)
#define SRC_GPR6_PCIE_PCS_TX_DEEMPH_GEN1_OFFSET			12
#define SRC_GPR6_PCIE_PCS_TX_DEEMPH_GEN2_3P5DB_MASK		(0x3f << 0)
#define SRC_GPR6_PCIE_PCS_TX_DEEMPH_GEN2_3P5DB_OFFSET	0
#define SRC_GPR6_PCIE_PCS_TX_DEEMPH_GEN2_6DB_MASK		(0x3f << 6)
#define SRC_GPR6_PCIE_PCS_TX_DEEMPH_GEN2_6DB_OFFSET		6
#define SRC_GPR6_PCIE_PCS_TX_SWING_FULL_MASK			(0x7f << 18)
#define SRC_GPR6_PCIE_PCS_TX_SWING_FULL_OFFSET			18
#define SRC_GPR6_PCIE_PCS_TX_SWING_LOW_MASK				(0x7f << 25)



#endif /* __LINUX_S32V234SRC_GPR_H */
