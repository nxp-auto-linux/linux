/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 NXP
 */

#ifndef __DT_BINDINGS_SCMI_RESET_S32R45_H
#define __DT_BINDINGS_SCMI_RESET_S32R45_H

#include <dt-bindings/reset/s32cc-scmi-reset.h>

#define S32R45_SCMI_RST_LAX		S32CC_PLAT_SCMI_RST(0)
#define S32R45_SCMI_RST_RADAR	S32CC_PLAT_SCMI_RST(1)
#define S32R45_SCMI_RST_MAX_ID	S32CC_PLAT_SCMI_RST(2)

#if S32CC_SCMI_RST_MAX_ID < S32R45_SCMI_RST_MAX_ID
#error Please increase the value of S32CC_SCMI_RST_MAX_ID
#endif

#endif
