/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 NXP
 */
#ifndef S32GEN1_MC_ME_H
#define S32GEN1_MC_ME_H

/* MC_ME_CTL */
#define MC_ME_CTL_KEY			(0x00000000U)
#define MC_ME_CTL_KEY_KEY		(0x00005AF0)
#define MC_ME_CTL_KEY_INVERTEDKEY	(0x0000A50F)

/* MC_ME_MODE_CONF */
#define MC_ME_MODE_CONF			(0x00000004U)
#define MC_ME_MODE_CONF_FUNC_RST	(0x1 << 1)

/* MC_ME_MODE_UPD */
#define MC_ME_MODE_UPD			(0x00000008U)
#define MC_ME_MODE_UPD_UPD		(0x1 << 0)

/* MC_ME partition definitions */
#define MC_ME_PRTN_N(n)			(0x100 + (n) * 0x200)
#define MC_ME_PRTN_N_PCONF(n)		(MC_ME_PRTN_N(n))
#define MC_ME_PRTN_N_PUPD(n)		(MC_ME_PRTN_N(n) + 0x4)
#define MC_ME_PRTN_N_STAT(n)		(MC_ME_PRTN_N(n) + 0x8)
#define MC_ME_PRTN_N_COFB0_STAT(n)	(MC_ME_PRTN_N(n) + 0x10)
#define MC_ME_PRTN_N_COFB0_CLKEN(n)	(MC_ME_PRTN_N(n) + 0x30)

/* MC_ME_PRTN_N_* register fields */
#define MC_ME_PRTN_N_PCE		(1 << 0)
#define MC_ME_PRTN_N_PCUD		BIT(0)
#define MC_ME_PRTN_N_PCS		BIT(0)
#define MC_ME_PRTN_N_OSSE		(1 << 2)
#define MC_ME_PRTN_N_OSSUD		BIT(2)
#define MC_ME_PRTN_N_OSSS		BIT(2)
#define MC_ME_PRTN_N_REQ(n)		BIT(n)

#endif
