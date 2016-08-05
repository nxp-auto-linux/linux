/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _MC_ME_H
#define _MC_ME_H

/* MC_ME registers definitions */
/* MC_ME_GS */
 #define MC_ME_GS(mc_me)                ((mc_me) + 0x00000000)

/* MC_ME_MCTL */
#define MC_ME_MCTL(mc_me)               ((mc_me) + 0x00000004)
#define MC_ME_MCTL_RESET                (0x0 << 28)
#define MC_ME_MCTL_TEST                 (0x1 << 28)
#define MC_ME_MCTL_DRUN                 (0x3 << 28)
#define MC_ME_MCTL_RUN0                 (0x4 << 28)
#define MC_ME_MCTL_RUN1                 (0x5 << 28)
#define MC_ME_MCTL_RUN2                 (0x6 << 28)
#define MC_ME_MCTL_RUN3                 (0x7 << 28)

#define MC_ME_GS_S_MTRANS               BIT(27)

#define MC_ME_MCTL_KEY                  (0x00005AF0)
#define MC_ME_MCTL_INVERTEDKEY          (0x0000A50F)

/*
 * MC_ME_RESET_MC/MC_ME_TEST_MC
 * MC_ME_DRUN_MC
 * MC_ME_RUNn_MC
 */
#define MC_ME_RESET_MC(mc_me)           ((mc_me) + 0x00000020)
#define MC_ME_TEST_MC(mc_me)            ((mc_me) + 0x00000024)
#define MC_ME_DRUN_MC(mc_me)            ((mc_me) + 0x0000002C)
#define MC_ME_RUNn_MC(mc_me, n)         ((mc_me) + 0x00000030 + 0x4 * (n))
#define MC_ME_MODE_MC_SYSCLK_OFFSET     (0)
#define MC_ME_MODE_MC_SYSCLK_SIZE       (0x3)
#define MC_ME_MODE_MC_SYSCLK(val)       (MC_ME_MODE_MC_SYSCLK_MASK & (val))
#define MC_ME_MODE_MC_SYSCLK_MASK       (0x0000000F)
#define MC_ME_MODE_MC_FIRCON            BIT(4)
#define MC_ME_MODE_MC_XOSCON            BIT(5)
#define MC_ME_MODE_MC_ARMPLL            BIT(6)
#define MC_ME_MODE_MC_PERIPHPLL         BIT(7)
#define MC_ME_MODE_MC_ENETPLL           BIT(8)
#define MC_ME_MODE_MC_DDRPLL            BIT(9)
#define MC_ME_MODE_MC_VIDEOPLL          BIT(10)
#define MC_ME_MODE_MC_MVRON             BIT(20)

/* MC_ME_DRUN_SEC_CC_I */
#define MC_ME_DRUN_SEC_CC_I(mc_me)              ((mc_me) + 0x260)
/* MC_ME_RUNn_SEC_CC_I */
#define MC_ME_RUNn_SEC_CC_I(mc_me, n)           ((mc_me) + 0x270 + (n) * 0x10)
#define MC_ME_MODE_SEC_CC_I_SYSCLK1_OFFSET      (4)
#define MC_ME_MODE_SEC_CC_I_SYSCLK2_OFFSET      (8)
#define MC_ME_MODE_SEC_CC_I_SYSCLK3_OFFSET      (12)
/* Consider only the defined clocks */
#define MC_ME_MODE_SEC_CC_I_SYSCLK1_SIZE        (0x3)
#define MC_ME_MODE_SEC_CC_I_SYSCLK2_SIZE        (0x3)
#define MC_ME_MODE_SEC_CC_I_SYSCLK3_SIZE        (0x3)

/* MC_ME_RUN_PCn */
#define MC_ME_RUN_PCn(mc_me, n)         (mc_me + 0x00000080 + 0x4 * (n))

#define MC_ME_RUN_PCn_MIN_IDX           (0)
#define MC_ME_RUN_PCn_MAX_IDX           (7)
#define MC_ME_RUN_PCn_RESET             BIT(0)
#define MC_ME_RUN_PCn_TEST              BIT(1)
#define MC_ME_RUN_PCn_DRUN              BIT(3)
#define MC_ME_RUN_PCn_RUN0              BIT(4)
#define MC_ME_RUN_PCn_RUN1              BIT(5)
#define MC_ME_RUN_PCn_RUN2              BIT(6)
#define MC_ME_RUN_PCn_RUN3              BIT(7)


#define MC_ME_PCTLn(mc_me, n)           (mc_me + 0xC0 + 4*(n>>2) + (3-(n)%4))

/* Peripherals PCTL indexes */
#define DEC200_PCTL	39
#define DCU_PCTL	40
#define CSI0_PCTL	48
#define DMACHMUX0_PCTL	49
#define ENET_PCTL	50
#define FRAY_PCTL	52
#define MMDC0_PCTL	54
#define PIT0_PCTL	58
#define SARADC0_PCTL	77
#define FlexTIMER0_PCTL	79
#define IIC0_PCTL	81
#define LINFLEX0_PCTL	83
#define CANFD0_PCTL	85
#define DSPI0_PCTL	87
#define DSPI2_PCTL	89
#define CRC0_PCTL	91
#define SDHC_PCTL	93
#define VIU0_PCTL	100
#define HPSMI_PCTL	104
#define SIPI_PCTL	116
#define LFAST_PCTL	120
#define CSI1_PCTL	160
#define DMACHMUX1_PCTL	161
#define MMDC1_PCTL	162
#define QUADSPI0_PCTL	166
#define PIT1_PCTL	170
#define FlexTIMER1_PCTL	182
#define IIC1_PCTL	184
#define IIC2_PCTL	186
#define LINFLEX1_PCTL	188
#define CANFD1_PCTL	190
#define DSPI1_PCTL	192
#define DSPI3_PCTL	194
#define CRC1_PCTL	204
#define TSENS_PCTL	206
#define VIU1_PCTL	208
#define JPEG_PCTL	212
#define H264DEC_PCTL	216
#define H264ENC_PCTL	220
#define MBIST_PCTL	236

static inline void entry_to_target_mode(void __iomem *mc_me, u32 mode)
{
	writel_relaxed(mode | MC_ME_MCTL_KEY, MC_ME_MCTL(mc_me));
	writel_relaxed(mode | MC_ME_MCTL_INVERTEDKEY, MC_ME_MCTL(mc_me));
	while ((readl_relaxed(MC_ME_GS(mc_me)) &
		MC_ME_GS_S_MTRANS) != 0x00000000)
		;
}

static inline void enable_cpumodes_onperipheralconfig(void __iomem *mc_me,
			u32 modes, u32 run_pc_idx)
{
	BUG_ON(run_pc_idx < MC_ME_RUN_PCn_MIN_IDX ||
	       run_pc_idx >= MC_ME_RUN_PCn_MAX_IDX);
	writel_relaxed(modes, MC_ME_RUN_PCn(mc_me, run_pc_idx));
}

static inline void enable_clocks_sources(u32 flags, u32 clks,
			void __iomem *xrun_mc_addr)
{
	writel_relaxed(readl_relaxed(xrun_mc_addr) | flags | clks,
		       xrun_mc_addr);
}

static inline void enable_sysclock(u32 clk, void __iomem *xrun_mc_addr)
{
	writel_relaxed(readl_relaxed(xrun_mc_addr) & clk,
		       xrun_mc_addr);
}

#endif
