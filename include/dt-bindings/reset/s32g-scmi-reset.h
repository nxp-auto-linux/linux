/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021, 2023-2024 NXP
 */
#ifndef __DT_BINDINGS_S32G_SCMI_RESET_H
#define __DT_BINDINGS_S32G_SCMI_RESET_H

#include <dt-bindings/reset/s32cc-scmi-reset.h>

#define S32G_SCMI_RST_PFE	S32CC_PLAT_SCMI_RST(0)
#define S32G_SCMI_RST_LLCE	S32CC_PLAT_SCMI_RST(1)
#define S32G_SCMI_RST_MAX_ID	S32CC_PLAT_SCMI_RST(2)

#if S32CC_SCMI_RST_MAX_ID < S32G_SCMI_RST_MAX_ID
#error Please increase the value of S32CC_SCMI_RST_MAX_ID
#endif

#endif
