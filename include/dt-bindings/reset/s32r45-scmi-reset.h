/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 NXP
 */

#ifndef __DT_BINDINGS_SCMI_RESET_S32R45_H
#define __DT_BINDINGS_SCMI_RESET_S32R45_H

#include <dt-bindings/reset/s32gen1-scmi-reset.h>

#define S32R45_SCMI_RST_LAX	S32GEN1_PLAT_SCMI_RST(0)
#define S32R45_SCMI_RST_RADAR	S32GEN1_PLAT_SCMI_RST(1)
#define S32R45_SCMI_RST_MAX_ID	S32GEN1_PLAT_SCMI_RST(2)

#if S32GEN1_SCMI_RST_MAX_ID < S32R45_SCMI_RST_MAX_ID
#error Please increase the value of S32GEN1_SCMI_RST_MAX_ID
#endif

#endif
