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
#define MPHY_REG_UNKNOWN_1			0xC002
#define SUP_DIG_MPLLB_OVRD_IN_5			0x001C
#define SUP_DIG_MPLLB_OVRD_IN_6			0x011C
#define RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_0	0x3006
#define RAWLANEN_DIG_PCS_XF_RX_PCS_OUT_0	0x300f
#define RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_1	0x3106
#define RAWLANEN_DIG_PCS_XF_RX_PCS_OUT		0x310f

/* Vendor specific attributes */
#define RXSQCONTROL				0x8009
#define RXOVRCTRL1				0x800A
#define RXLINERSTCFGMSB				0x8011
#define RXLINERSTCFGLSB				0x8012
#define RXRHOLDCTRLOPT				0x8013
#define CBENBLCPBATTRWR				0x8113
#define CBRATESEL				0x8114
#define CBCREGADDRLSB				0x8116
#define CBCREGADDRMSB				0x8117
#define CBCREGWRLSB				0x8118
#define CBCREGWRMSB				0x8119
#define CBCREGRDLSB				0x811A
#define CBCREGRDMSB				0x811B
#define CBCREGRDWRSEL				0x811C
#define CBCRCTRL				0x811F
#define CBUPLH8					0x8130
#define CBREFCLKCTRL2				0x8132

/* Vendor specific MPHY disable */
#define VS_MPHYDISABLE				0xD0C1

/* TX FSM states */
#define TX_FSM_HIBERN8				1
#define TX_FSM_DISABLE				0

/* CPort definitions */
#define CPORT_0					0
#define CPORT_1					1

/* CPort flags */
#define CPORT_FLAGS(csv_n, csd_n, e2e)		((csv_n) | (csd_n) | (e2e))

#endif /* _UFS_NXP_S32N79_H_ */
