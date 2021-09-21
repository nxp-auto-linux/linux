/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
 */
#ifndef __DT_BINDINGS_S32G3_SCMI_RESET_H
#define __DT_BINDINGS_S32G3_SCMI_RESET_H

#include <dt-bindings/reset/s32g-scmi-reset.h>

#define S32G3_SCMI_RST(N)	((N) + S32G_SCMI_RST_MAX_ID)

#define S32G3_SCMI_RST_CM7_3	S32G3_SCMI_RST(0)
#define S32G3_SCMI_RST_A53_4	S32G3_SCMI_RST(1)
#define S32G3_SCMI_RST_A53_5	S32G3_SCMI_RST(2)
#define S32G3_SCMI_RST_A53_6	S32G3_SCMI_RST(3)
#define S32G3_SCMI_RST_A53_7	S32G3_SCMI_RST(4)
#define S32G3_SCMI_RST_MAX_ID	S32G3_SCMI_RST(5)

#if S32GEN1_SCMI_RST_MAX_ID < S32G3_SCMI_RST_MAX_ID
#error Please increase the value of S32GEN1_SCMI_RST_MAX_ID
#endif

#endif

