/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/of_address.h>
#include <linux/clk.h>
#include <dt-bindings/clock/s32v234-clock.h>

#include "clk.h"

static void __iomem *mc_cgm0_base;
static void __iomem *mc_cgm1_base;
static void __iomem *mc_cgm2_base;
static void __iomem *mc_cgm3_base;
static void __iomem *mc_me_base;
static void __iomem *src_base;

/* sources for multiplexer clocks, this is used multiple times */
PNAME(osc_sels) = {"firc", "fxosc", };

PNAME(cores_sels) = {"firc", "fxosc", "armpll_phi0", };

PNAME(sys_sels) = {"firc", "fxosc", "armpll_dfs0", };

PNAME(perifray_sels) = {"firc", "fxosc", "dummy",
			"periphpll_phi0_div5", };

PNAME(can_sels) = {"firc", "fxosc", "dummy",
		   "periphpll_phi0_div5", };

PNAME(lin_sels) = {"firc", "fxosc", "dummy",
		   "periphpll_phi0_div3", "dummy", "dummy",
		   "dummy", "dummy", "sys6",};

PNAME(sdhc_sels) = {"firc", "fxosc", "dummy",
		    "dummy", "enetpll_dfs3",};

PNAME(enet_sels) = {"firc", "fxosc", "dummy",
		    "dummy", "enetpll_phi0",};

PNAME(enet_time_sels) = {"firc", "fxosc", "dummy",
			 "dummy", "enetpll_phi0",};

PNAME(dcu_sels) = {"firc", "fxosc", "dummy", "dummy",
		   "dummy", "dummy", "dummy", "dummy", "dummy",
		   "sys6",};

PNAME(gpu_sels) = {"firc", "fxosc", "armpll_dfs1", };

PNAME(gpu_shdmipi_sels) = {"firc", "fxosc",
			   "armpll_dfs2", };
PNAME(h264dcd_sels) = {"firc", "fxosc", "dummy", "dummy",
		       "enetpll_dfs0", };
PNAME(h264enc_sels) = {"firc", "fxosc", "dummy", "dummy",
		       "enetpll_dfs1", };
PNAME(jpegdec_sels) = {"firc", "fxosc", "dummy", "dummy",
		       "dummy", "ddrpll_dfs2", };

static struct clk *clk[S32V234_CLK_END];
static struct clk_onecell_data clk_data;

static u32 share_count_csi0gate;
static u32 share_count_csi1gate;
static u32 share_count_dcugate;
#if defined(DEC200_CLK)
static u32 share_count_dec200gate;
#endif
static u32 share_count_h264dcdgate;
static u32 share_count_h264encgate;
static u32 share_count_jpegdecgate;
static u32 share_count_linflex0gate;
static u32 share_count_linflex1gate;
static u32 share_count_sdhcgate;
static u32 share_count_viu0gate;
static u32 share_count_viu1gate;

#if 0
/* TBD: Enable gating for ENET */
static u32 share_count_enetgate;
#endif

static void __init s32v234_clocks_init(struct device_node *mc_cgm0_node)
{
	struct device_node *np;

	clk[S32V234_CLK_DUMMY] = s32_clk_fixed("dummy", 0);
	clk[S32V234_CLK_FXOSC] = s32_obtain_fixed_clock("fxosc", 0);
	clk[S32V234_CLK_FIRC] = s32_obtain_fixed_clock("firc", 0);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_me");
	mc_me_base = of_iomap(np, 0);
	BUG_ON(!mc_me_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-src");
	src_base = of_iomap(np, 0);
	BUG_ON(!src_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_cgm1");
	mc_cgm1_base = of_iomap(np, 0);
	BUG_ON(!mc_cgm1_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_cgm2");
	mc_cgm2_base = of_iomap(np, 0);
	BUG_ON(!mc_cgm2_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_cgm3");
	mc_cgm3_base = of_iomap(np, 0);
	BUG_ON(!mc_cgm3_base);

	np = mc_cgm0_node;
	mc_cgm0_base = of_iomap(np, 0);
	BUG_ON(!mc_cgm0_base);

	enable_cpumodes_onperipheralconfig(mc_me_base, MC_ME_RUN_PCn_DRUN |
					    MC_ME_RUN_PCn_RUN0 |
					    MC_ME_RUN_PCn_RUN1 |
					    MC_ME_RUN_PCn_RUN2 |
					    MC_ME_RUN_PCn_RUN3,
					    1);

	/* turn on XOSC and FIRC */
	enable_clocks_sources(MC_ME_MODE_MC_MVRON, MC_ME_MODE_MC_XOSCON |
			      MC_ME_MODE_MC_FIRCON,
			      MC_ME_RUNn_MC(mc_me_base, 0));

	/* transition the core to RUN0 mode */
	entry_to_target_mode(mc_me_base, MC_ME_MCTL_RUN0);

	clk[S32V234_CLK_ARMPLL_SRC_SEL] = s32_clk_mux("armpll_sel",
		SRC_GPR1, SRC_GPR1_ARMPLL_SRC_SEL_OFFSET,
		SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels));

	clk[S32V234_CLK_ENETPLL_SRC_SEL] = s32_clk_mux("enetpll_sel",
		SRC_GPR1, SRC_GPR1_ENETPLL_SRC_SEL_OFFSET,
		SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels));

	clk[S32V234_CLK_DDRPLL_SRC_SEL] = s32_clk_mux("ddrpll_sel",
		SRC_GPR1, SRC_GPR1_DDRPLL_SRC_SEL_OFFSET,
		SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels));

	clk[S32V234_CLK_PERIPHPLL_SRC_SEL] = s32_clk_mux("periphpll_sel",
		SRC_GPR1, SRC_GPR1_PERIPHPLL_SRC_SEL_OFFSET,
		SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels));

	clk[S32V234_CLK_VIDEOPLL_SRC_SEL] = s32_clk_mux("videopll_sel",
		SRC_GPR1, SRC_GPR1_VIDEOPLL_SRC_SEL_OFFSET,
		SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels));

	/* ARM_PLL */
	clk[S32V234_CLK_ARMPLL_VCO] = s32_clk_plldig(S32_PLLDIG_ARM,
		"armpll_vco", "armpll_sel", ARMPLL_PLLDIG(mc_cgm0_base),
		ARMPLL_PLLDIG_PLLDV_MFD, ARMPLL_PLLDIG_PLLDV_MFN,
		ARMPLL_PLLDIG_PLLDV_RFDPHI0, ARMPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_ARMPLL_PHI0] = s32_clk_plldig_phi(S32_PLLDIG_ARM,
		"armpll_phi0", "armpll_vco",
		ARMPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_ARMPLL_PHI1] = s32_clk_plldig_phi(S32_PLLDIG_ARM,
		"armpll_phi1", "armpll_vco",
		ARMPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_ARMPLL_DFS0] = s32_clk_dfs(S32_PLLDIG_ARM,
		 "armpll_dfs0", "armpll_phi1",
		 ARMPLL_PLLDIG_DFS(mc_cgm0_base), 0,
		 ARMPLL_PLLDIG_DFS0_MFN);

	clk[S32V234_CLK_ARMPLL_DFS1] = s32_clk_dfs(S32_PLLDIG_ARM,
		 "armpll_dfs1", "armpll_phi1",
		 ARMPLL_PLLDIG_DFS(mc_cgm0_base), 1,
		 ARMPLL_PLLDIG_DFS1_MFN);

	clk[S32V234_CLK_ARMPLL_DFS2] = s32_clk_dfs(S32_PLLDIG_ARM,
		 "armpll_dfs2", "armpll_phi1",
		 ARMPLL_PLLDIG_DFS(mc_cgm0_base), 2,
		 ARMPLL_PLLDIG_DFS2_MFN);

	clk[S32V234_CLK_CORES_SEL] = s32_clk_mux("cores_sels",
		MC_ME_RUNn_SEC_CC_I(mc_me_base, 0),
		MC_ME_MODE_SEC_CC_I_SYSCLK1_OFFSET,
		MC_ME_MODE_SEC_CC_I_SYSCLK1_SIZE,
		cores_sels, ARRAY_SIZE(cores_sels));

	clk[S32V234_CLK_CORE] = s32_clk_divider("core", "cores_sels",
		CGM_SC_DCn(mc_cgm1_base, 0), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_CORE2] = s32_clk_divider("core2", "cores_sels",
		CGM_SC_DCn(mc_cgm1_base, 1), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_COREDBG] = s32_clk_divider("coredbgcoredbg",
		"cores_sels",
		CGM_SC_DCn(mc_cgm1_base, 2), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_SYS_SEL] = s32_clk_mux("sys_sel",
		MC_ME_RUNn_MC(mc_me_base, 0),
		MC_ME_MODE_MC_SYSCLK_OFFSET,
		MC_ME_MODE_MC_SYSCLK_SIZE,
		sys_sels, ARRAY_SIZE(sys_sels));

	clk[S32V234_CLK_SYS3] = s32_clk_divider("sys3", "sys_sel",
		CGM_SC_DCn(mc_cgm0_base, 0), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_SYS6] = s32_clk_divider("sys6", "sys_sel",
		CGM_SC_DCn(mc_cgm0_base, 1), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_SYS6_DIV2] = s32_clk_divider("sys6_div2", "sys_sel",
		CGM_SC_DCn(mc_cgm0_base, 2), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_IIC0] = s32_clk_gate2("iic0", "sys6",
		mc_me_base, IIC0_PCTL, 0, 1);
	clk[S32V234_CLK_IIC1] = s32_clk_gate2("iic1", "sys6",
		mc_me_base, IIC1_PCTL, 0, 1);
	clk[S32V234_CLK_IIC2] = s32_clk_gate2("iic2", "sys6",
		mc_me_base, IIC2_PCTL, 0, 1);

#if defined(DEC200_CLK)
	clk[S32V234_CLK_DEC200_ENC_AHB] = s32_clk_gate2_shared(
		"dec200_enc_ahb", "sys3", mc_me_base,
		DEC200_PCTL, 0, 1,
		&share_count_dec200gate);

	clk[S32V234_CLK_DEC200_ENC_IPS] = s32_clk_gate2_shared(
		"dec200_enc_ips", "sys6", mc_me_base,
		DEC200_PCTL, 0, 1,
		&share_count_dec200gate);
#endif

	clk[S32V234_CLK_DMACHMUX0] = s32_clk_gate2("dmachmux0", "sys6",
		mc_me_base, DMACHMUX0_PCTL, 0, 1);
	clk[S32V234_CLK_DMACHMUX1] = s32_clk_gate2("dmachmux1", "sys6",
		mc_me_base, DMACHMUX1_PCTL, 0, 1);

	clk[S32V234_CLK_SPI0] = s32_clk_gate2("spi0", "sys6",
		mc_me_base, DSPI0_PCTL, 0, 1);
	clk[S32V234_CLK_SPI1] = s32_clk_gate2("spi1", "sys6",
		mc_me_base, DSPI1_PCTL, 0, 1);
	clk[S32V234_CLK_SPI2] = s32_clk_gate2("spi2", "sys6",
		mc_me_base, DSPI2_PCTL, 0, 1);
	clk[S32V234_CLK_SPI3] = s32_clk_gate2("spi3", "sys6",
		mc_me_base, DSPI3_PCTL, 0, 1);

	clk[S32V234_CLK_PIT0] = s32_clk_gate2("pit0", "sys6",
		 mc_me_base, PIT0_PCTL, 0, 1);
	clk[S32V234_CLK_PIT1] = s32_clk_gate2("pit1", "sys6",
		 mc_me_base, PIT1_PCTL, 0, 1);

	clk[S32V234_CLK_VIU0_AHB]  = s32_clk_gate2_shared("viu0_ahb",
		"sys3",
		mc_me_base, VIU0_PCTL, 0, 1, &share_count_viu0gate);
	clk[S32V234_CLK_VIU0_IPS] = s32_clk_gate2_shared("viu0_ips",
		"sys6", mc_me_base, VIU0_PCTL, 0, 1,
		&share_count_viu0gate);

	clk[S32V234_CLK_VIU1_AHB]  = s32_clk_gate2_shared("viu1_ahb",
		"sys3",
		mc_me_base, VIU1_PCTL, 0, 1, &share_count_viu0gate);
	clk[S32V234_CLK_VIU1_IPS] = s32_clk_gate2_shared("viu1_ips",
		"sys6", mc_me_base, VIU1_PCTL, 0, 1,
		&share_count_viu1gate);

	/* enable ARMPLL */
	enable_clocks_sources(0, MC_ME_MODE_MC_ARMPLL,
			      MC_ME_RUNn_MC(mc_me_base, 0));

	/* PERIPH_PLL */
	clk[S32V234_CLK_PERIPHPLL_VCO] = s32_clk_plldig(S32_PLLDIG_PERIPH,
		"periphpll_vco", "periphpll_sel",
		PERIPHPLL_PLLDIG(mc_cgm0_base),
		PERIPHPLL_PLLDIG_PLLDV_MFD, PERIPHPLL_PLLDIG_PLLDV_MFN,
		PERIPHPLL_PLLDIG_PLLDV_RFDPHI0,
		PERIPHPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_PERIPHPLL_PHI0] =
		s32_clk_plldig_phi(S32_PLLDIG_PERIPH,
		"periphpll_phi0", "periphpll_vco",
		PERIPHPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_PERIPHPLL_PHI1] =
		s32_clk_plldig_phi(S32_PLLDIG_PERIPH,
		"periphpll_phi1", "periphpll_vco",
		PERIPHPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_PERIPHPLL_PHI0_DIV3] = s32_clk_fixed_factor(
		"periphpll_phi0_div3", "periphpll_phi0", 1, 3);

	clk[S32V234_CLK_PERIPHPLL_PHI0_DIV5] = s32_clk_fixed_factor(
		"periphpll_phi0_div5", "periphpll_phi0", 1, 5);

	/* Peri Clock */
	clk[S32V234_CLK_PERI_FRAY_PLL_SEL] = s32_clk_mux("perifray_sel",
		CGM_ACn_SC(mc_cgm0_base, 5),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		perifray_sels, ARRAY_SIZE(perifray_sels));

	clk[S32V234_CLK_CAN_SEL] = s32_clk_mux("can_sel",
		CGM_ACn_SC(mc_cgm0_base, 6),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		can_sels, ARRAY_SIZE(can_sels));

	clk[S32V234_CLK_PERI] = s32_clk_divider("peri", "perifray_sel",
		CGM_ACn_DCm(mc_cgm0_base, 5, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	clk[S32V234_CLK_SARADC0] = s32_clk_gate2("adc", "peri",
		mc_me_base, SARADC0_PCTL, 0, 1);

	clk[S32V234_CLK_PERI_FRAY_PLL] = s32_clk_divider("perifray",
		"perifray_sel",
		CGM_ACn_DCm(mc_cgm0_base, 5, 1),
		MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	/* Can Clock */
	clk[S32V234_CLK_CAN0] = s32_clk_gate2("can0", "sys6",
		mc_me_base, CANFD0_PCTL, 0, 1);
	clk[S32V234_CLK_CAN1] = s32_clk_gate2("can1", "sys6",
		mc_me_base, CANFD1_PCTL, 0, 1);
	clk[S32V234_CLK_CAN] = s32_clk_divider("can", "can_sel",
		CGM_ACn_DCm(mc_cgm0_base, 6, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	/* Lin Clock */
	clk[S32V234_CLK_LIN_SEL] = s32_clk_mux("lin_sel",
		CGM_ACn_SC(mc_cgm0_base, 5),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		lin_sels, ARRAY_SIZE(lin_sels));

	clk[S32V234_CLK_LIN] = s32_clk_divider("lin", "lin_sel",
		CGM_ACn_DCm(mc_cgm0_base, 3, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	clk[S32V234_CLK_LIN_IPG] = s32_clk_fixed_factor("lin_ipg",
		"lin", 1, 2);

	clk[S32V234_CLK_LIN0]  = s32_clk_gate2_shared("lin0", "lin",
		mc_me_base, LINFLEX0_PCTL, 0, 1, &share_count_linflex0gate);
	clk[S32V234_CLK_LIN0_IPG] = s32_clk_gate2_shared("lin0_ipg",
		"lin_ipg", mc_me_base, LINFLEX0_PCTL, 0, 1,
		&share_count_linflex0gate);
	clk[S32V234_CLK_LIN1]  = s32_clk_gate2_shared("lin1", "lin",
		mc_me_base, LINFLEX1_PCTL, 0, 1, &share_count_linflex1gate);
	clk[S32V234_CLK_LIN1_IPG] = s32_clk_gate2_shared("lin1_ipg",
		"lin_ipg", mc_me_base, LINFLEX1_PCTL, 0, 1,
		&share_count_linflex1gate);

	/* enable DCU */
	clk[S32V234_CLK_DCU_SEL] = s32_clk_mux("videopll_sel",
		CGM_ACn_SC(mc_cgm0_base, 9),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		dcu_sels, ARRAY_SIZE(dcu_sels));

	clk[S32V234_CLK_DCU_AXI_DIV] = s32_clk_divider("dcu_axi_div",
		"videopll_sel", CGM_ACn_DCm(mc_cgm0_base, 9, 1),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	clk[S32V234_CLK_DCU_PIX_DIV] = s32_clk_divider("dcu_pix_div",
		"videopll_sel", CGM_ACn_DCm(mc_cgm0_base, 9, 1),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	clk[S32V234_CLK_DCU_AXI] = s32_clk_gate2_shared("dcu_axi",
		"dcu_axi_div", mc_me_base, DCU_PCTL, 0, 1,
		 &share_count_dcugate);

	clk[S32V234_CLK_DCU_PIX] = s32_clk_gate2_shared("dcu_pix",
		"dcu_pix_div", mc_me_base, DCU_PCTL, 0, 1,
		 &share_count_dcugate);

	/* enable PERIPHPLL */
	enable_clocks_sources(0, MC_ME_MODE_MC_PERIPHPLL,
			      MC_ME_RUNn_MC(mc_me_base, 0));

	/* ENET_PLL */
	clk[S32V234_CLK_ENETPLL_VCO] = s32_clk_plldig(S32_PLLDIG_ENET,
		"enetpll_vco", "enetpll_sel", ENETPLL_PLLDIG(mc_cgm0_base),
		ENETPLL_PLLDIG_PLLDV_MFD, ENETPLL_PLLDIG_PLLDV_MFN,
		ENETPLL_PLLDIG_PLLDV_RFDPHI0, ENETPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_ENETPLL_PHI0] = s32_clk_plldig_phi(S32_PLLDIG_ENET,
		"enetpll_phi0", "enetpll_vco",
		ENETPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_ENETPLL_PHI1] = s32_clk_plldig_phi(S32_PLLDIG_ENET,
		"enetpll_phi1", "enetpll_vco",
		ENETPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_ENETPLL_DFS0] = s32_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs0", "enetpll_phi1",
		 ENETPLL_PLLDIG_DFS(mc_cgm0_base), 0,
		 ENETPLL_PLLDIG_DFS0_MFN);

	clk[S32V234_CLK_ENETPLL_DFS1] = s32_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs1", "enetpll_phi1",
		 ENETPLL_PLLDIG_DFS(mc_cgm0_base), 1,
		 ENETPLL_PLLDIG_DFS1_MFN);

	clk[S32V234_CLK_ENETPLL_DFS2] = s32_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs2", "enetpll_phi1",
		 ENETPLL_PLLDIG_DFS(mc_cgm0_base), 2,
		 ENETPLL_PLLDIG_DFS2_MFN);

	clk[S32V234_CLK_ENETPLL_DFS3] = s32_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs3", "enetpll_phi1",
		 ENETPLL_PLLDIG_DFS(mc_cgm0_base), 3,
		 ENETPLL_PLLDIG_DFS3_MFN);

	/* ENET Clock */
	clk[S32V234_CLK_ENET_SEL] = s32_clk_mux("enet_sel",
		CGM_ACn_SC(mc_cgm2_base, 2),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		enet_sels, ARRAY_SIZE(enet_sels));

	clk[S32V234_CLK_ENET_TIME_SEL] = s32_clk_mux("enet_time_sel",
		CGM_ACn_SC(mc_cgm0_base, 7),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		enet_time_sels, ARRAY_SIZE(enet_time_sels));

	clk[S32V234_CLK_ENET] = s32_clk_divider("enet", "enet_sel",
		CGM_ACn_DCm(mc_cgm2_base, 2, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	clk[S32V234_CLK_ENET_TIME] = s32_clk_divider("enet_time",
		"enet_time_sel",
		CGM_ACn_DCm(mc_cgm0_base, 7, 1),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);
#if 0
	/* Temporarily disabled until we have the clarifications
	 about ENET clock from Design team */
	clk[S32V234_CLK_ENET_AHB] = s32_clk_gate2_shared("enet_ahb",
		"sys3", mc_me_base, ENET_PCTL, 0, 1,
		&share_count_enetgate);
	clk[S32V234_CLK_ENET_IPS] = s32_clk_gate2_shared("enet_ips",
		"sys6", mc_me_base, ENET_PCTL, 0, 1,
		&share_count_enetgate);
	clk[S32V234_CLK_ENET_TIME] = s32_clk_gate2_shared("enet_time",
		"enet_time_div", mc_me_base, ENET_PCTL, 0, 1,
		&share_count_enetgate);
#endif
	/* H264DCD Clock */
	clk[S32V234_CLK_H264DCD_SEL] = s32_clk_mux("h264dcd_sel",
		CGM_ACn_SC(mc_cgm0_base, 12),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		h264dcd_sels, ARRAY_SIZE(h264dcd_sels));
	clk[S32V234_CLK_H264DCD_DIV] = s32_clk_divider("h264dcd_div",
		"h264dec_sel", CGM_ACn_DCm(mc_cgm0_base, 12, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);
	clk[S32V234_CLK_H264DCD] = s32_clk_gate2_shared("h264dcd",
		"h264dec_div", mc_me_base, H264DEC_PCTL, 0, 1,
		&share_count_h264dcdgate);
	clk[S32V234_CLK_H264DCD_IPS] = s32_clk_gate2_shared("h264dcd_ips",
		"sys6", mc_me_base, H264DEC_PCTL, 0, 1,
		&share_count_h264dcdgate);
	clk[S32V234_CLK_H264DCD_SRAM_IPS] =
		s32_clk_gate2_shared("h264dcd_sram_ips",
		"sys3", mc_me_base, H264DEC_PCTL, 0, 1,
		&share_count_h264dcdgate);


	/* H264ENC Clock */
	clk[S32V234_CLK_H264ENC_SEL] = s32_clk_mux("h264enc_sel",
		CGM_ACn_SC(mc_cgm0_base, 13),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		h264enc_sels, ARRAY_SIZE(h264enc_sels));
	clk[S32V234_CLK_H264ENC_DIV] = s32_clk_divider("h264enc_div",
		"h264enc_sel", CGM_ACn_DCm(mc_cgm0_base, 13, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);
	clk[S32V234_CLK_H264ENC] = s32_clk_gate2_shared("h264enc",
		"h264enc_div", mc_me_base, H264ENC_PCTL, 0, 1,
		&share_count_h264encgate);
	clk[S32V234_CLK_H264ENC_IPS] = s32_clk_gate2_shared("h264enc_ips",
		"sys6", mc_me_base, H264ENC_PCTL, 0, 1,
		&share_count_h264encgate);
	clk[S32V234_CLK_H264ENC_SRAM_IPS] =
		s32_clk_gate2_shared("h264enc_sram_ips",
		"sys3", mc_me_base, H264ENC_PCTL, 0, 1,
		&share_count_h264encgate);

	/* SDHC Clock */
	clk[S32V234_CLK_SDHC_SEL] = s32_clk_mux("sdhc_sel",
		CGM_ACn_SC(mc_cgm0_base, 15),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		sdhc_sels, ARRAY_SIZE(sdhc_sels));

	clk[S32V234_CLK_SDHC_DIV] = s32_clk_divider("sdhc_div", "sdhc_sel",
		CGM_ACn_DCm(mc_cgm0_base, 15, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);
	clk[S32V234_CLK_SDHC] = s32_clk_gate2_shared("sdhc",
		"sdhc_div", mc_me_base, SDHC_PCTL, 0, 1,
		&share_count_sdhcgate);
	clk[S32V234_CLK_SDHC_IPS] = s32_clk_gate2_shared("sdhc_ips",
		"sys6", mc_me_base, SDHC_PCTL, 0, 1,
		&share_count_sdhcgate);
	clk[S32V234_CLK_SDHC_AHB] = s32_clk_gate2_shared("sdhc_ahb",
		"sys6", mc_me_base, SDHC_PCTL, 0, 1,
		&share_count_sdhcgate);

	/* GPU Clocks */
	clk[S32V234_CLK_GPU_SEL] = s32_clk_mux("gpu_sels",
		MC_ME_RUNn_SEC_CC_I(mc_me_base, 0),
		MC_ME_MODE_SEC_CC_I_SYSCLK2_OFFSET,
		MC_ME_MODE_SEC_CC_I_SYSCLK2_SIZE,
		gpu_sels, ARRAY_SIZE(gpu_sels));

	clk[S32V234_CLK_GPUSHD_MIPI_SEL] = s32_clk_mux("gpu_shdmipi_sels",
		MC_ME_RUNn_SEC_CC_I(mc_me_base, 0),
		MC_ME_MODE_SEC_CC_I_SYSCLK3_OFFSET,
		MC_ME_MODE_SEC_CC_I_SYSCLK3_SIZE,
		gpu_shdmipi_sels, ARRAY_SIZE(gpu_shdmipi_sels));

	clk[S32V234_CLK_GPU] = s32_clk_divider("gpu", "gpu_sels",
		CGM_SC_DCn(mc_cgm2_base, 0), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_GPU_SHD] = s32_clk_divider("gpu_shd",
		"gpu_shdmipi_sels",
		CGM_SC_DCn(mc_cgm3_base, 0), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_MIPI_LI] = s32_clk_divider("mipi_li",
		"gpu_shdmipi_sels",
		CGM_SC_DCn(mc_cgm3_base, 1), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_CSI0] = s32_clk_gate2_shared("csi0",
		"sys6", mc_me_base, CSI0_PCTL, 0, 1,
		&share_count_csi0gate);
	clk[S32V234_CLK_CSI0_CLI] = s32_clk_gate2_shared("csi0_cli",
		"mipi_li", mc_me_base, CSI0_PCTL, 0, 1,
		&share_count_csi0gate);
	clk[S32V234_CLK_CSI0_CUI] = s32_clk_gate2_shared("csi0_cui",
		"sys3", mc_me_base, CSI0_PCTL, 0, 1,
		&share_count_csi0gate);
	clk[S32V234_CLK_CSI0_DPHY] = s32_clk_gate2_shared("csi0_dphy",
		"sys6_div2", mc_me_base, CSI0_PCTL, 0, 1,
		&share_count_csi0gate);

	clk[S32V234_CLK_CSI1] = s32_clk_gate2_shared("csi1",
		"sys6", mc_me_base, CSI1_PCTL, 0, 1,
		&share_count_csi1gate);
	clk[S32V234_CLK_CSI1_CLI] = s32_clk_gate2_shared("csi1_cli",
		"mipi_li", mc_me_base, CSI1_PCTL, 0, 1,
		&share_count_csi1gate);
	clk[S32V234_CLK_CSI1_CUI] = s32_clk_gate2_shared("csi1_cui",
		"sys3", mc_me_base, CSI1_PCTL, 0, 1,
		&share_count_csi1gate);
	clk[S32V234_CLK_CSI1_DPHY] = s32_clk_gate2_shared("csi1_dphy",
		"sys6_div2", mc_me_base, CSI1_PCTL, 0, 1,
		&share_count_csi1gate);

	/* enable ENETPLL */
	enable_clocks_sources(0, MC_ME_MODE_MC_ENETPLL,
			      MC_ME_RUNn_MC(mc_me_base, 0));
	/* DDR_PLL */
	clk[S32V234_CLK_DDRPLL_VCO] = s32_clk_plldig(S32_PLLDIG_DDR,
		"ddrpll_vco", "ddrpll_sel", DDRPLL_PLLDIG(mc_cgm0_base),
		DDRPLL_PLLDIG_PLLDV_MFD, DDRPLL_PLLDIG_PLLDV_MFN,
		DDRPLL_PLLDIG_PLLDV_RFDPHI0, DDRPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_DDRPLL_PHI0] = s32_clk_plldig_phi(S32_PLLDIG_DDR,
		"ddrpll_phi0", "ddrpll_vco",
		DDRPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_DDRPLL_PHI1] = s32_clk_plldig_phi(S32_PLLDIG_DDR,
		"ddrpll_phi1", "ddrpll_vco",
		DDRPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_DDRPLL_DFS0] = s32_clk_dfs(S32_PLLDIG_DDR,
		 "ddrpll_dfs0", "ddrpll_phi1",
		 DDRPLL_PLLDIG_DFS(mc_cgm0_base), 0,
		 DDRPLL_PLLDIG_DFS0_MFN);

	clk[S32V234_CLK_DDRPLL_DFS1] = s32_clk_dfs(S32_PLLDIG_DDR,
		 "ddrpll_dfs1", "ddrpll_phi1",
		 DDRPLL_PLLDIG_DFS(mc_cgm0_base), 1,
		 DDRPLL_PLLDIG_DFS1_MFN);

	clk[S32V234_CLK_DDRPLL_DFS2] = s32_clk_dfs(S32_PLLDIG_DDR,
		 "ddrpll_dfs2", "ddrpll_phi1",
		 DDRPLL_PLLDIG_DFS(mc_cgm0_base), 2,
		 DDRPLL_PLLDIG_DFS2_MFN);

	/* JPEGDEC Clock */
	clk[S32V234_CLK_MJPEG_SEL] = s32_clk_mux("mjpeg_sel",
		CGM_ACn_SC(mc_cgm0_base, 2),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		jpegdec_sels, ARRAY_SIZE(jpegdec_sels));

	clk[S32V234_CLK_MJPEG_DIV] = s32_clk_divider("mjpeg_div",
		"mjpeg_sel", CGM_ACn_DCm(mc_cgm0_base, 2, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	clk[S32V234_CLK_MJPEG] = s32_clk_gate2_shared("mjpeg",
		"mjpeg_div", mc_me_base, JPEG_PCTL, 0, 1,
		&share_count_jpegdecgate);
	clk[S32V234_CLK_JPEG_IPS] = s32_clk_gate2_shared("jpeg_ips",
		"sys6", mc_me_base, JPEG_PCTL, 0, 1,
		&share_count_jpegdecgate);
	clk[S32V234_CLK_JPEG_SRAM_IPS] = s32_clk_gate2_shared("jpeg_sram_ips",
		"sys3", mc_me_base, JPEG_PCTL, 0, 1,
		&share_count_jpegdecgate);

	/* enable DDRPLL */
	enable_clocks_sources(0, MC_ME_MODE_MC_DDRPLL,
			      MC_ME_RUNn_MC(mc_me_base, 0));

	/* set the system clock */
	enable_sysclock(MC_ME_MODE_MC_SYSCLK(0x2),
			MC_ME_RUNn_MC(mc_me_base, 0));

	/* transition the core to RUN0 mode */
	entry_to_target_mode(mc_me_base, MC_ME_MCTL_RUN0);

	/* Add the clocks to provider list */
	clk_data.clks = clk;
	clk_data.clk_num = ARRAY_SIZE(clk);
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
}

CLK_OF_DECLARE(S32V234, "fsl,s32v234-mc_cgm0", s32v234_clocks_init);
