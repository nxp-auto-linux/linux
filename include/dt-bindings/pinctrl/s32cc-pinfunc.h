/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#ifndef _DT_BINDINGS_S32CC_PINFUNC_H
#define _DT_BINDINGS_S32CC_PINFUNC_H

/* Pins functions (SSS field) */
#define FUNC0	0
#define FUNC1	1
#define FUNC2	2
#define FUNC3	3
#define FUNC4	4
#define FUNC5	5
#define FUNC6	6
#define FUNC7	7

#define S32CC_PINMUX(PIN, FUNC) (((PIN) << 4) | (FUNC))

#define S32CC_SLEW_208MHZ	0
#define S32CC_SLEW_166MHZ	4
#define S32CC_SLEW_150MHZ	5
#define S32CC_SLEW_133MHZ	6
#define S32CC_SLEW_83MHZ	7

#define S32CC_GPIO_SLEW_50MHZ	4
#define S32CC_GPIO_SLEW_10MHZ	6
#define S32CC_GPIO_SLEW_1MHZ	7

#endif
