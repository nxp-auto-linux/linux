/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
 */
#ifndef __DT_BINDINGS_S32G_SCMI_RESET_H
#define __DT_BINDINGS_S32G_SCMI_RESET_H

#include <dt-bindings/reset/s32gen1-scmi-reset.h>

#define S32G_SCMI_RST_PFE	S32GEN1_PLAT_SCMI_RST(0)
#define S32G_SCMI_RST_LLCE	S32GEN1_PLAT_SCMI_RST(1)
#define S32G_SCMI_RST_MAX_ID	S32GEN1_PLAT_SCMI_RST(2)

#if S32GEN1_SCMI_RST_MAX_ID < S32G_SCMI_RST_MAX_ID
#error Please increase the value of S32GEN1_SCMI_RST_MAX_ID
#endif

#endif

