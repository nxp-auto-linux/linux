/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _PLL_S32GEN1_H
#define _PLL_S32GEN1_H

/* PLLDIG PLL Control Register (PLLDIG_PLLCR) */
#define PLLDIG_PLLCR(pll)               (pll)
#define PLLDIG_PLLCR_PLLPD              (1 << 31)

/* PLLDIG PLL Status Register (PLLDIG_PLLSR) */
#define PLLDIG_PLLSR(pll)               ((pll) + 0x00000004)
#define PLLDIG_PLLSR_LOCK               (1 << 2)

/* PLLDIG PLL Divider Register (PLLDIG_PLLDV) */
#define PLLDIG_PLLDV(base)		((base) + 0x00000008)
#define PLLDIG_PLLDV_MFI_SET(val)	(PLLDIG_PLLDV_MFI_MASK & (val))
#define PLLDIG_PLLDV_MFI_MASK		(0x000000FF)

#define PLLDIG_PLLDV_RDIV_SET(val)	(PLLDIG_PLLDV_RDIV_MASK & \
					(((val) & \
					PLLDIG_PLLDV_RDIV_MAXVALUE) \
					<< PLLDIG_PLLDV_RDIV_OFFSET))
#define PLLDIG_PLLDV_RDIV_MASK		(0x00007000)
#define PLLDIG_PLLDV_RDIV_MAXVALUE	(0x7)
#define PLLDIG_PLLDV_RDIV_OFFSET	(12)

/* PLLDIG PLL Fractional  Divide Register (PLLDIG_PLLFD) */
#define PLLDIG_PLLFD(base)		((base) + 0x00000010)
#define PLLDIG_PLLFD_MFN_SET(val)	(PLLDIG_PLLFD_MFN_MASK & (val))
#define PLLDIG_PLLFD_MFN_MASK		(0x00007FFF)
#define PLLDIG_PLLFD_SMDEN              (1 << 30)

/* PLL Clock Mux (PLLCLKMUX) */
#define PLLDIG_PLLCLKMUX(pll)                   ((pll) + 0x00000020)
#define PLLDIG_PLLCLKMUX_REFCLKSEL_SET(val)     ((val) & \
						PLLDIG_PLLCLKMUX_REFCLKSEL_MASK)
#define PLLDIG_PLLCLKMUX_REFCLKSEL_MASK         (0x3)
#define PLLDIG_PLLCLKMUX_REFCLKSEL_SIZE		(2)
#define PLLDIG_PLLCLKMUX_REFCLKSEL_OFFSET	(0)

#define PLLDIG_PLLCLKMUX_REFCLKSEL_SET_FIRC     (0x0)
#define PLLDIG_PLLCLKMUX_REFCLKSEL_SET_XOSC     (0x1)


/* PLL Output Divider (PLLODIV0 - PLLODIV7) */
#define PLLDIG_PLLODIV(pll, n)          ((pll) + 0x00000080 + n * 0x4)
#define PLLDIG_PLLODIV_DIV_SET(val)     (PLLDIG_PLLODIV_DIV_MASK & \
					 ((val) << PLLDIG_PLLODIV_DIV_OFFSET))
#define PLLDIG_PLLODIV_DIV_MASK         (0x00FF0000)
#define PLLDIG_PLLODIV_DIV_OFFSET       (16)

#define PLLDIG_PLLODIV_DE               (1 << 31)

/* Naming convention for PLL:
 * ARMPLL - PLL0
 * PERIPHPLL - PLL1
 * DDRPLL - PLL2
 * ACCELPLL - PLL3
 * AURORAPLL - PLL4
 */
/* The min,max values for PLL VCO (Hz) */
#define ARMPLL_MAX_VCO_RATE		(2000000000)
#define PERIPHPLL_MAX_VCO_RATE		(2000000000)
#define DDRPLL_MAX_VCO_RATE		(1600000000)
#define ACCELPLL_MAX_VCO_RATE		(2400000000)
#define AURORAPLL_MAX_VCO_RATE		(5000000000)

/* Number of PHIs */
#define ARMPLL_PHI_Nr			(2)
#define PERIPHPLL_PHI_Nr		(8)
#define DDRPLL_PHI_Nr			(1)
#define ACCELPLL_PHI_Nr			(2)
#define AURORAPLL_PHI_Nr		(1)

/* The min,max values for PLL PHIn outputs (Hz) */
#define ARMPLL_MAX_PHI0_MAX_RATE	(1000000000)
#define ARMPLL_MAX_PHI1_MAX_RATE	(500000000)
#define PERIPHPLL_MAX_PHI0_MAX_RATE	(125000000)
#define PERIPHPLL_MAX_PHI1_MAX_RATE	(80000000)
#define PERIPHPLL_MAX_PHI2_MAX_RATE	(80000000)
#define PERIPHPLL_MAX_PHI3_MAX_RATE	(133000000)
#define PERIPHPLL_MAX_PHI4_MAX_RATE	(200000000)
#define PERIPHPLL_MAX_PHI5_MAX_RATE	(125000000)
#define PERIPHPLL_MAX_PHI6_MAX_RATE	(100000000)
#define PERIPHPLL_MAX_PHI7_MAX_RATE	(100000000)
#define DDRPLL_MAX_PHI0_MAX_RATE	(80000000)
#define ACCELPLL_MAX_PHI0_MAX_RATE	(80000000)
#define ACCELPLL_MAX_PHI1_MAX_RATE	(80000000)
#define AURORAPLL_MAX_PHI0_MAX_RATE	(5000000000)

/* The maximum value for PLL VCO according to data sheet */
#define MAX_VCO_RATE			(5000000000)
#define MIN_VCO_RATE			(1300000000)

#define PHI_MAXNUMBER                   (8)
#define DFS_MAXNUMBER                   (6)

/* DIV for each PHI. */
#define ARM_PLLDIG_PLLODIV0		(0x2)
#define ARM_PLLDIG_PLLODIV1		(0x5)
#define PERIPH_PLLDIG_PLLODIV0		(0x10)
#define PERIPH_PLLDIG_PLLODIV1		(0x19)
#define PERIPH_PLLDIG_PLLODIV2		(0x19)
#define PERIPH_PLLDIG_PLLODIV3		(0xf)
#define PERIPH_PLLDIG_PLLODIV4		(0xa)
#define PERIPH_PLLDIG_PLLODIV5		(0x10)
#define PERIPH_PLLDIG_PLLODIV6		(0x14)
#define DDR_PLLDIG_PLLODIV0		(0x2)
#define ACCEL_PLLDIG_PLLODIV0		(0x3)
#define ACCEL_PLLDIG_PLLODIV1		(0x3)
#define AURORA_PLLDIG_PLLODIV0		(0x1)

enum s32gen1_plldig_type {
	S32GEN1_PLLDIG_ARM,
	S32GEN1_PLLDIG_PERIPH,
	S32GEN1_PLLDIG_DDR,
	S32GEN1_PLLDIG_ACCEL,
	S32GEN1_PLLDIG_AURORA,
};

#endif
