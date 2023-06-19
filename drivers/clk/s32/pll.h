/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _PLL_H
#define _PLL_H

/* PLLDIG PLL Divider Register (PLLDIG_PLLDV) */
#define PLLDIG_PLLDV(base)		((base) + 0x00000028)
#define PLLDIG_PLLDV_MFD_SET(val)	(PLLDIG_PLLDV_MFD_MASK & (val))
#define PLLDIG_PLLDV_MFD_MASK		(0x000000FF)

#define PLLDIG_PLLDV_RFDPHI_SET(val)	(PLLDIG_PLLDV_RFDPHI_MASK & \
					(((val) & \
					PLLDIG_PLLDV_RFDPHI_MAXVALUE) \
					<< PLLDIG_PLLDV_RFDPHI_OFFSET))
#define PLLDIG_PLLDV_RFDPHI_MASK	(0x003F0000)
#define PLLDIG_PLLDV_RFDPHI_MAXVALUE	(0x3F)
#define PLLDIG_PLLDV_RFDPHI_MINVALUE	(0x0)

#define PLLDIG_PLLDV_RFDPHI_OFFSET	(16)

#define PLLDIG_PLLDV_RFDPHI1_SET(val)   (PLLDIG_PLLDV_RFDPHI1_MASK & \
					(((val) & \
					PLLDIG_PLLDV_RFDPHI1_MAXVALUE) \
					<< PLLDIG_PLLDV_RFDPHI1_OFFSET))
#define PLLDIG_PLLDV_RFDPHI1_MASK	(0x7E000000)
#define PLLDIG_PLLDV_RFDPHI1_MAXVALUE	(0x3F)
#define PLLDIG_PLLDV_RFDPHI1_MINVALUE	(0x0)
#define PLLDIG_PLLDV_RFDPHI1_OFFSET	(25)

#define PLLDIG_PLLDV_PREDIV_SET(val)	(PLLDIG_PLLDV_PREDIV_MASK & \
					(((val) & \
					PLLDIG_PLLDV_PREDIV_MAXVALUE) \
					<< PLLDIG_PLLDV_PREDIV_OFFSET))
#define PLLDIG_PLLDV_PREDIV_MASK	(0x00007000)
#define PLLDIG_PLLDV_PREDIV_MAXVALUE	(0x7)
#define PLLDIG_PLLDV_PREDIV_OFFSET	(12)

/* PLLDIG PLL Fractional  Divide Register (PLLDIG_PLLFD) */
#define PLLDIG_PLLFD(base)		((base) + 0x00000030)
#define PLLDIG_PLLFD_MFN_SET(val)	(PLLDIG_PLLFD_MFN_MASK & (val))
#define PLLDIG_PLLFD_MFN_MASK		(0x00007FFF)

/* PLL Calibration Register 1 (PLLDIG_PLLCAL1) */
#define PLLDIG_PLLCAL1(base)	((base) + 0x00000038)
#define PLLDIG_PLLCAL1_NDAC1_SET(val)	(PLLDIG_PLLCAL1_NDAC1_MASK & \
					((val) \
					 << PLLDIG_PLLCAL1_NDAC1_OFFSET))
#define PLLDIG_PLLCAL1_NDAC1_OFFSET	(24)
#define PLLDIG_PLLCAL1_NDAC1_MASK	(0x7F000000)

/* Naming convention for PLL:
 * ARMPLL - PLL0
 * PERIPHPLL - PLL1
 * ENETPLL - PLL2
 * DDRPLL - PLL3
 * VIDEOPLL - PLL4
 */
/* The min,max values for PLL VCO (Hz) */
#define ARMPLL_MAX_VCO_RATE             (1000000000)
#define PERIPHPLL_MAX_VCO_RATE          (1200000000)
#define ENETPLL_MAX_VCO_RATE            (1000000000)
#define DDRPLL_MAX_VCO_RATE             (1066000000)
#define VIDEOPLL_MAX_VCO_RATE           (1200000000)

/* The min,max values for PLL PHI0 and PHI1 outputs (Hz) */
#define ARMPLL_MAX_PHI0_MAX_RATE	(1000000000)
#define ARMPLL_MAX_PHI1_MAX_RATE	(1000000000)
#define PERIPHPLL_MAX_PHI0_MAX_RATE	(400000000)
#define PERIPHPLL_MAX_PHI1_MAX_RATE	(100000000)
#define ENETPLL_MAX_PHI0_MAX_RATE	(500000000)
#define ENETPLL_MAX_PHI1_MAX_RATE	(1000000000)
#define DDRPLL_MAX_PHI0_MAX_RATE	(533000000)
#define DDRPLL_MAX_PHI1_MAX_RATE	(1066000000)
#define VIDEOPLL_MAX_PHI0_MAX_RATE	(600000000)

/* The maximum value for PLL VCO according to data sheet */
#define MAX_VCO_RATE			(1300000000)
#define MIN_VCO_RATE			(650000000)

#endif
