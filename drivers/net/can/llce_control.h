/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020 NXP
 */
#ifndef	LLCE_CONTROL_H
#define	LLCE_CONTROL_H

/* Can_Log_Init */
/* This is not an A53 interrupt, it doesn't go through GIC.
 * This interrupt is from inside the LLCE.
 */
#define	INSIDE_LLCE_IRQ_LINE		24
#define LLCE_FIFO_FNEMTY			((uint32_t)0x00000800U)
#define LLCE_ICSR_INTFLAG_05		0x20U

#define LLCE_CAN_CONFIG_FIFO_FIXED_MASK	\
	((uint32_t)0x0007FFFF)

/* Call-back */
#define LLCE_CAN_CONFIG_MAXRXMB		2048
#define LLCE_CAN_CONFIG_MAXTXMB		256
#define LLCE_CAN_CONFIG_MAXAFFRMB	256
#define LLCE_CAN_CONFIG_TOTAL \
	(LLCE_CAN_CONFIG_MAXRXMB + LLCE_CAN_CONFIG_MAXTXMB + \
	 LLCE_CAN_CONFIG_MAXAFFRMB)

#endif
