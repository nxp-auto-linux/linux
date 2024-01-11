/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/*
 * Copyright 2023-2024 NXP
 *
 * This header provides constants for CAN timestamp sources for NXP S32R45 SoCs
 */

#ifndef _DT_BINDINGS_MUX_NXP_S32R45_CAN_TS
#define _DT_BINDINGS_MUX_NXP_S32R45_CAN_TS

#define S32R45_CAN_TS_DISABLE		0x0
#define S32R45_CAN_TS_GMAC0_PTP		0x4
#define S32R45_CAN_TS_GMAC1_PTP		0x5
#define S32R45_CAN_TS_STM7			0x6

#endif
