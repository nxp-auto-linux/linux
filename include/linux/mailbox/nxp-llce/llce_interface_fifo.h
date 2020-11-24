/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020 NXP */
#ifndef LLCE_FIFOINTERFACE_H
#define LLCE_FIFOINTERFACE_H

#define LLCE_CONFIG_FIFO_DEPTH		(16U)

/** Configuration value of register FMR of the FIFO. */
#define LLCE_FIFO_CONFIG_FMR		(0x00000420)

#define LLCE_FIFO_FFULLD		(0x00000001U)
#define LLCE_FIFO_FEMTYD		(0x00000002U)
#define LLCE_FIFO_SR_QCOUNT_SHIFT	(24U)

#define LLCE_FIFO_FCR_FIFOEN		(0x00000001U)
#define LLCE_FIFO_FCR_FLENOWEN		(0x00000002U)
#define LLCE_FIFO_FCR_POPEN		(0x00000004U)
#define LLCE_FIFO_FCR_PUSHEN		(0x00000008U)
#define LLCE_FIFO_FCR_FLUSH		(0x00000010U)

#define LLCE_FIFO_FCR_RESET		(0x0U)

/* Generic defines used to access STATUS, ILR and IER registers of FIFOs. */
#define LLCE_FIFO_FFULL		(0x00000100U)
#define LLCE_FIFO_FNFULL	(0x00000200U)
#define LLCE_FIFO_FEMTY		(0x00000400U)
#define LLCE_FIFO_FNEMTY	(0x00000800U)
#define LLCE_FIFO_POPEVT	(0x00001000U)
#define LLCE_FIFO_WMKFL		(0x00002000U)
#define LLCE_FIFO_WMKEM		(0x00004000U)

/* Default value returned when popping an empty fifo */
#define LLCE_FIFO_NULL_VALUE	(0xFFFFFFFFU)

/* Specific FIFOs base address calculation macros. */
#define LLCE_GET_BLRIN_BASE_ADDRESS(hw_ctrl)                            \
	((u32)(LLCE_BLRIN0_BASEADDR + ((u32)(hw_ctrl)*0x400U)))
#define LLCE_GET_BLROUT_BASE_ADDRESS(hw_ctrl)                           \
	((u32)(LLCE_BLROUT0_BASEADDR + ((u32)(hw_ctrl)*0x400U)))
#define LLCE_GET_TXACK_BASE_ADDRESS(hw_ctrl)                            \
	((u32)(LLCE_TXACK0_BASEADDR + ((u32)(hw_ctrl)*0x400U)))
#define LLCE_GET_RXIN_BASE_ADDRESS(hw_ctrl)                             \
	((u32)(LLCE_RXIN0_BASEADDR + ((u32)(hw_ctrl)*0x400U)))
#define LLCE_GET_RXOUT_BASE_ADDRESS(hw_ctrl)                            \
	((u32)(LLCE_RXOUT0_BASEADDR + ((u32)(hw_ctrl)*0x400U)))
#define LLCE_GET_GENERIC_FIFO_BASE_ADDRESS(gen_ffo)                     \
	((u32)(LLCE_GENERIC_FIFO_BASEADDR + ((u32)(gen_ffo)*0x400U)))
#define LLCE_GENERIC_FIFO_CONFIG(gen_ffo)                               \
	((u32)(LLCE_GET_GENERIC_FIFO_BASE_ADDRESS(gen_ffo)))
#define LLCE_GENERIC_FIFO_STATUS0(gen_ffo)                              \
	((u32)(LLCE_GET_GENERIC_FIFO_BASE_ADDRESS(gen_ffo) + 0x04U))
#define LLCE_GENERIC_FIFO_STATUS1(gen_ffo)                              \
	((u32)(LLCE_GET_GENERIC_FIFO_BASE_ADDRESS(gen_ffo) + 0x08U))
#define LLCE_GENERIC_FIFO_IER(gen_ffo)                                  \
	((u32)(LLCE_GET_GENERIC_FIFO_BASE_ADDRESS(gen_ffo) + 0x0CU))
#define LLCE_GENERIC_FIFO_ILR(gen_ffo)                                  \
	((u32)(LLCE_GET_GENERIC_FIFO_BASE_ADDRESS(gen_ffo) + 0x10U))
#define LLCE_GENERIC_FIFO_PUSH0(gen_ffo)                                \
	((u32)(LLCE_GET_GENERIC_FIFO_BASE_ADDRESS(gen_ffo) + 0x14U))
#define LLCE_GENERIC_FIFO_POP0(gen_ffo)                                 \
	((u32)(LLCE_GET_GENERIC_FIFO_BASE_ADDRESS(gen_ffo) + 0x24U))
#define LLCE_GENERIC_FIFO_FMR(gen_ffo)                                  \
	((u32)(LLCE_GET_GENERIC_FIFO_BASE_ADDRESS(gen_ffo) + 0x34U))

#define LLCE_FIFO_CONFIG(BASE)	(BASE)
#define LLCE_FIFO_STATUS0(BASE)	((BASE) + 0x04U)
#define LLCE_FIFO_STATUS1(BASE)	((BASE) + 0x08U)
#define LLCE_FIFO_IER(BASE)	((BASE) + 0x0CU)
#define LLCE_FIFO_ILR(BASE)	((BASE) + 0x10U)
#define LLCE_FIFO_PUSH0(BASE)	((BASE) + 0x14U)
#define LLCE_FIFO_POP0(BASE)	((BASE) + 0x24U)
#define LLCE_FIFO_FMR(BASE)	((BASE) + 0x34U)

#endif /* LLCE_FIFOINTERFACE_H */
