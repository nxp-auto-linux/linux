/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 NXP
 */
#ifndef S32GEN1_RDC_H
#define S32GEN1_RDC_H

/* Reset domain definitions */
#define RDC_RD_N_CTRL(RDC, N)		((RDC) + (0x4 * (N)))
#define RDC_RD_N_STATUS(RDC, N)		((RDC) + 0x80 + (0x4 * (N)))
#define RD_CTRL_UNLOCK_MASK		(0x80000000)
#define RDC_RD_INTERCONNECT_DISABLE	BIT(3)
#define RDC_RD_INTERCONNECT_DISABLE_STAT BIT(4)

#endif
