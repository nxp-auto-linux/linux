/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 NXP
 */
#ifndef S32GEN1_MC_RGM_H
#define S32GEN1_MC_RGM_H

/* RGM */
#define RGM_PRST(per)			(0x40 + ((per) * 0x8))
#define RGM_PSTAT(per)			(0x140 + ((per) * 0x8))
#define PSTAT_PERIPH_n_STAT(n)		BIT(n)
#define PRST_PERIPH_n_RST(n)		BIT(n)

#define RGM_DES             (0x00)
#define RGM_FES             (0x08)
#define RGM_FERD            (0x0C)
#define RGM_FREC            (0x14)
#define RGM_RDSS            (0x24)

/* MC_RGM_FRET */
#define MC_RGM_FRET			(0x18U)

#define MC_RGM_FRET_VALUE		(0xF)

#endif /* S32GEN1_MC_RGM_H */
