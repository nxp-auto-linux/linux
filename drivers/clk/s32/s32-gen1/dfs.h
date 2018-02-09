/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _DFS_S32GEN1_H
#define _DFS_S32GEN1_H

/* DFS Control Register (DFS_CTRL) */
#define DFS_CTL(dfs)			((dfs) + 0x00000018)
#define DFS_CTL_RESET			(1 << 1)

/* DFS Port Status (DFS_PORTSR) */
#define DFS_PORTSR(dfs)			((dfs) + 0x0000000C)

/* DFS Port Reset Register (DFS_PORTRESET) */
#define DFS_PORTRESET(dfs)		((dfs) + 0x00000014)
#define DFS_PORTRESET_PORTRESET_SET(val) \
		(DFS_PORTRESET_PORTRESET_MASK | \
		(((val) & DFS_PORTRESET_PORTRESET_MAXVAL) \
		<< DFS_PORTRESET_PORTRESET_OFFSET))
#define DFS_PORTRESET_PORTRESET_MAXVAL	(0x3F)
#define DFS_PORTRESET_PORTRESET_MASK	(0x00000003F)
#define DFS_PORTRESET_PORTRESET_OFFSET	(0)

/* DFS Divide Register Portn (DFS_DVPORTn) */
#define DFS_DVPORTn(dfs, n)		((dfs) + (0x0000001C + \
					((n) * sizeof(u32))))
#define DFS_DVPORTn_MFI_SET(val)	(DFS_DVPORTn_MFI_MASK & \
					(((val) & DFS_DVPORTn_MFI_MAXVAL) \
					<< DFS_DVPORTn_MFI_OFFSET))
#define DFS_DVPORTn_MFN_SET(val)	(DFS_DVPORTn_MFN_MASK & \
					(((val) & DFS_DVPORTn_MFN_MAXVAL) \
					<< DFS_DVPORTn_MFN_OFFSET))
#define DFS_DVPORTn_MFI_MASK		(0x0000FF00)
#define DFS_DVPORTn_MFN_MASK		(0x000000FF)
#define DFS_DVPORTn_MFI_MAXVAL		(0xFF)
#define DFS_DVPORTn_MFN_MAXVAL		(0xFF)
#define DFS_DVPORTn_MFI_OFFSET		(8)
#define DFS_DVPORTn_MFN_OFFSET		(0)
#define DFS_MAXNUMBER			(6)

/*
 * Naming convention for PLL:
 * ARMPLL - PLL0
 * PERIPHPLL - PLL1
 * ENETPLL - PLL2
 * DDRPLL - PLL3
 * VIDEOPLL - PLL4
 */

/* The max values for PLL DFS is in Hz */
/* ARMPLL */
#define ARMPLL_DFS1_MAX_RATE		(800000000)
#define ARMPLL_DFS2_MAX_RATE		(800000000)
#define ARMPLL_DFS3_MAX_RATE		(500000000)
#define ARMPLL_DFS4_MAX_RATE		(300000000)
#define ARMPLL_DFS5_MAX_RATE		(600000000)
#define ARMPLL_DFS6_MAX_RATE		(600000000)
/* PERIPHPLL */
#define PERIPHPLL_DFS1_MAX_RATE		(800000000)
#define PERIPHPLL_DFS2_MAX_RATE		(960000000)
#define PERIPHPLL_DFS3_MAX_RATE		(800000000)
#define PERIPHPLL_DFS4_MAX_RATE		(600000000)
#define PERIPHPLL_DFS5_MAX_RATE		(330000000)
#define PERIPHPLL_DFS6_MAX_RATE		(500000000)

#define ARMPLL_DFS_NR			(6)
#define PERIPHPLL_DFS_NR		(6)

#endif
