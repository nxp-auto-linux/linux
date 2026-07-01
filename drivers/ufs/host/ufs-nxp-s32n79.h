/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/*
 * Copyright 2025 NXP
 */

#ifndef _UFS_NXP_S32N79_H_
#define _UFS_NXP_S32N79_H_

/* MPHY register definitions */
#define MPHY_REG_LSB_MASK			0x00FF
#define MPHY_REG_MSB_MASK			0xFF00

/* MPHY specific register addresses */
#define RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_0	0x3006
#define RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_1	0x3106

/* Vendor specific attributes */
#define CBRATESEL				0x8114
#define CBCREGADDRLSB				0x8116
#define CBCREGADDRMSB				0x8117
#define CBCREGWRLSB				0x8118
#define CBCREGWRMSB				0x8119
#define CBCREGRDLSB				0x811A
#define CBCREGRDMSB				0x811B
#define CBCREGRDWRSEL				0x811C
#define CBCRCTRL				0x811F
#define CBREFCLKCTRL2				0x8132

/* Vendor specific MPHY disable */
#define VS_MPHYDISABLE				0xD0C1

/* CPort definitions */
#define CPORT_0					0

#endif /* _UFS_NXP_S32N79_H_ */
