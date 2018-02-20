/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __DT_BINDINGS_CLOCK_S32GEN1_H
#define __DT_BINDINGS_CLOCK_S32GEN1_H

#define	S32GEN1_CLK_DUMMY			0
#define	S32GEN1_CLK_FIRC			1
#define	S32GEN1_CLK_SIRC			2
#define	S32GEN1_CLK_FXOSC			3
#define	S32GEN1_CLK_SXOSC			4
/* ARM PLL */
#define	S32GEN1_CLK_ARMPLL_SRC_SEL		5
#define	S32GEN1_CLK_ARMPLL_VCO			6
#define	S32GEN1_CLK_ARMPLL_PHI0			7
#define	S32GEN1_CLK_ARMPLL_PHI1			8
#define	S32GEN1_CLK_ARMPLL_DFS1			9
#define	S32GEN1_CLK_ARMPLL_DFS2			10
#define	S32GEN1_CLK_ARMPLL_DFS3			11
#define	S32GEN1_CLK_ARMPLL_DFS4			12
#define	S32GEN1_CLK_ARMPLL_DFS5			13
#define	S32GEN1_CLK_ARMPLL_DFS6			14
/* PERIPH PLL */
#define	S32GEN1_CLK_PERIPHPLL_SRC_SEL		15
#define	S32GEN1_CLK_PERIPHPLL_VCO		16
#define	S32GEN1_CLK_PERIPHPLL_PHI0		17
#define	S32GEN1_CLK_PERIPHPLL_PHI1		18
#define	S32GEN1_CLK_PERIPHPLL_PHI2		19
#define	S32GEN1_CLK_PERIPHPLL_PHI3		20
#define	S32GEN1_CLK_PERIPHPLL_PHI4		21
#define	S32GEN1_CLK_PERIPHPLL_PHI5		22
#define	S32GEN1_CLK_PERIPHPLL_PHI6		23
#define	S32GEN1_CLK_PERIPHPLL_PHI7		24
#define	S32GEN1_CLK_PERIPHPLL_DFS1		25
#define	S32GEN1_CLK_PERIPHPLL_DFS2		26
#define	S32GEN1_CLK_PERIPHPLL_DFS3		27
#define	S32GEN1_CLK_PERIPHPLL_DFS4		28
#define	S32GEN1_CLK_PERIPHPLL_DFS5		29
#define	S32GEN1_CLK_PERIPHPLL_DFS6		30
/* DDR PLL */
#define	S32GEN1_CLK_DDRPLL_SRC_SEL		31
#define	S32GEN1_CLK_DDRPLL_VCO			32
#define	S32GEN1_CLK_DDRPLL_PHI0			33
/* ACCEL PLL */
#define	S32GEN1_CLK_ACCELPLL_SRC_SEL		34
#define	S32GEN1_CLK_ACCELPLL_VCO		35
#define	S32GEN1_CLK_ACCELPLL_PHI0		36
#define	S32GEN1_CLK_ACCELPLL_PHI1		37
/* AURORA PLL */
#define	S32GEN1_CLK_AURORAPLL_SRC_SEL		38
#define	S32GEN1_CLK_AURORAPLL_VCO		39
#define	S32GEN1_CLK_AURORAPLL_PHI0		40
/* Cores Clock */
#define	S32GEN1_CLK_CORES_SEL			41
#define	S32GEN1_CLK_CORE			42
#define	S32GEN1_CLK_CORE2			43
#define	S32GEN1_CLK_COREDBG			44
/* LINFLEX Clock */
#define	S32GEN1_CLK_LIN_BAUD			45
#define	S32GEN1_CLK_LIN				46
#define	S32GEN1_CLK_LIN0_BAUD			47
#define	S32GEN1_CLK_LIN0			48
#define	S32GEN1_CLK_LIN1_BAUD			49
#define	S32GEN1_CLK_LIN1			50
#define	S32GEN1_CLK_LIN2_BAUD			51
#define	S32GEN1_CLK_LIN2			52
/* DSPI Clock*/
#define S32GEN1_CLK_SPI0                        53
#define S32GEN1_CLK_SPI1                        54
#define S32GEN1_CLK_SPI2                        55
#define S32GEN1_CLK_SPI3                        56
#define S32GEN1_CLK_SPI4                        57
#define S32GEN1_CLK_SPI5                        58
#define S32GEN1_CLK_DSPI                        59

#define	S32GEN1_CLK_END				60
#endif /* __DT_BINDINGS_CLOCK_S32GEN1_H */
