/*
 * Copyright 2018,2020 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __DT_BINDINGS_S32_GEN1_PINCTRL_H__
#define __DT_BINDINGS_S32_GEN1_PINCTRL_H__

/*
 * Use to set PAD control
 */

#define PAD_CTL_OBE		(1 << 21)
#define PAD_CTL_ODE		(1 << 20)
#define PAD_CTL_IBE		(1 << 19)
#define PAD_CTL_INV		(1 << 17)

#define PAD_CTL_SRE_OFS		(14)
#define PAD_CTL_SRE_208MHZ	(0 << PAD_CTL_SRE_OFS)
#define PAD_CTL_SRE_150MHZ	(4 << PAD_CTL_SRE_OFS)
#define PAD_CTL_SRE_100MHZ	(5 << PAD_CTL_SRE_OFS)
#define PAD_CTL_SRE_50MHZ	(6 << PAD_CTL_SRE_OFS)
#define PAD_CTL_SRE_25MHZ	(7 << PAD_CTL_SRE_OFS)

#define PAD_CTL_PUE		(1 << 13)
#define PAD_CTL_PUS		(1 << 12)
#define PAD_CTL_RCVR		(1 << 10)
#define PAD_CTL_SMC             (1 << 5)

#define PAD_CTL_SRC_SIG_SEL0   (0)
#define PAD_CTL_SRC_SIG_SEL1   (1)
#define PAD_CTL_SRC_SIG_SEL2   (2)
#define PAD_CTL_SRC_SIG_SEL3   (3)
#define PAD_CTL_SRC_SIG_SEL4   (4)
#define PAD_CTL_SRC_SIG_SEL5   (5)
#define PAD_CTL_SRC_SIG_SEL6   (6)
#define PAD_CTL_SRC_SIG_SEL7   (7)
#define PAD_CTL_MUX_MODE_MASK   (0xF)

#define S32GEN1_INVALID_GPIO	(-1)

#endif /* __DT_BINDINGS_S32_GEN1_PINCTRL_H__ */
