/*
 * Copyright 2018,2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/of_address.h>
#include <linux/clk.h>
#include <dt-bindings/clock/s32gen1-clock.h>

#include "clk.h"
#include "mc_cgm.h"

static void __iomem *mc_cgm0_base;
static void __iomem *mc_cgm1_base;
static void __iomem *mc_cgm2_base;
static void __iomem *mc_cgm5_base;
static void *armpll;
static void *periphpll;
static void *ddrpll;
static void *accelpll;
static void *armdfs;
static void *periphdfs;

DEFINE_SPINLOCK(s32gen1_lock);

/* sources for multiplexer clocks, this is used multiple times */
PNAME(osc_sels) = {"firc", "fxosc", };
static u32 osc_mux_ids[] = {
	PLLDIG_PLLCLKMUX_REFCLKSEL_SET_FIRC,
	PLLDIG_PLLCLKMUX_REFCLKSEL_SET_XOSC,
};

PNAME(can_sels) = {"firc", "fxosc", "periphpll_phi2", };
static u32 can_mux_ids[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC,
	MC_CGM_MUXn_CSC_SEL_FXOSC,
	MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI2,
};

PNAME(per_sels) = {"firc", "periphpll_phi1", };
static u32 per_mux_ids[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC,
	MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI1,
};

PNAME(ftm0_sels) = {"firc", "periphpll_phi1", };
static u32 ftm0_mux_ids[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC,
	MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI1,
};

PNAME(ftm1_sels) = {"firc", "periphpll_phi1", };
static u32 ftm1_mux_ids[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC,
	MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI1,
};

PNAME(lin_sels) = {"firc", "fxosc", "periphpll_phi3", };
static u32 lin_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_FXOSC,
	MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI3,
};

PNAME(sdhc_sels) = {"firc", "periphll_dfs3",};
static u32 sdhc_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_DFS3,
};

PNAME(qspi_sels) = {"firc", "periphll_dfs1", };
static u32 qspi_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_DFS1,
};

PNAME(dspi_sels) = {"firc", "periphpll_phi7", };
static u32 dspi_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI7,
};

PNAME(a53_core_sels) = {"firc", "armpll_dfs2", "armpll_phi0"};
static u32 a53_core_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_ARM_PLL_DFS2,
	MC_CGM_MUXn_CSC_SEL_ARM_PLL_PHI0,
};

PNAME(xbar_sels) = {"firc", "armpll_dfs1", };
static u32 xbar_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC,
	MC_CGM_MUXn_CSC_SEL_ARM_PLL_DFS1,
};

PNAME(ddr_sels) = {"firc", "ddrpll_phi0", };
static u32 ddr_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC,
	MC_CGM_MUXn_CSC_SEL_DDR_PLL_PHI0,
};

PNAME(gmac_0_tx_sels) = {"firc", "periphpll_phi5", "serdes_0_lane_0", };
static u32 gmac_0_tx_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI5,
	MC_CGM_MUXn_CSC_SEL_SERDES_0_LANE_0_TX_CLK,
};

PNAME(gmac_1_tx_sels) = {"firc", "periphpll_phi5", "serdes_1_lane_0", };
static u32 gmac_1_tx_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI5,
	MC_CGM_MUXn_CSC_SEL_SERDES_1_LANE_0_TX_CLK_R45,
};

PNAME(pfe_pe_sels) = {"firc", "accelpll_phi1",};
static u32 pfe_pe_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_ACCEL_PLL_PHI1,
};

PNAME(pfe_emac_0_tx_sels) = {"firc", "periphpll_phi5", "serdes_1_lane_0_tx", };
static u32 pfe_emac_0_tx_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI5,
	MC_CGM_MUXn_CSC_SEL_SERDES_1_LANE_0_TX_CLK,
};

PNAME(pfe_emac_1_tx_sels) = {"firc", "periphpll_phi5", "serdes_1_lane_1_tx", };
static u32 pfe_emac_1_tx_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI5,
	MC_CGM_MUXn_CSC_SEL_SERDES_1_LANE_1_TX_CLK,
};

PNAME(pfe_emac_2_tx_sels) = {"firc", "periphpll_phi5", "serdes_0_lane_1_tx", };
static u32 pfe_emac_2_tx_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_PERIPH_PLL_PHI5,
	MC_CGM_MUXn_CSC_SEL_SERDES_0_LANE_1_TX_CLK,
};

PNAME(accel_3_sels) = {"firc", "accelpll_phi0",};
static u32 accel_3_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_ACCEL_PLL_PHI0_2,
};

PNAME(accel_4_sels) = {"firc", "armpll_dfs4",};
static u32 accel_4_mux_idx[] = {
	MC_CGM_MUXn_CSC_SEL_FIRC, MC_CGM_MUXn_CSC_SEL_ARM_PLL_DFS4_2,
};

static struct clk *clk[S32GEN1_CLK_END];
static struct clk_onecell_data clk_data;

static void __init s32g274_extra_clocks_init(struct device_node *clocking_node)
{
	/* PFE */
	clk[S32GEN1_CLK_PFE_PE_SEL] = s32_clk_mux_table("pfe_pe_sel",
		CGM_MUXn_CSC(mc_cgm2_base, 0),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		pfe_pe_sels, ARRAY_SIZE(pfe_pe_sels), pfe_pe_mux_idx,
		&s32gen1_lock);
	clk[S32GEN1_CLK_PFE_PE] = s32_clk_divider_flags("pfe_pe",
		"pfe_pe_sel",
		CGM_MUXn_DC(mc_cgm2_base, 0), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, 0, &s32gen1_lock);
	clk[S32GEN1_CLK_PFE_SYS] = s32_clk_fixed_factor("pfe_sys",
		"pfe_pe", 1, 2);

	/* PFE EMAC 0 clocks */
	clk[S32GEN1_CLK_PFE_EMAC_0_TX_SEL] = s32_clk_mux_table(
		"pfe_emac_0_tx_sel",
		CGM_MUXn_CSC(mc_cgm2_base, 1),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		pfe_emac_0_tx_sels, ARRAY_SIZE(pfe_emac_0_tx_sels),
		pfe_emac_0_tx_mux_idx, &s32gen1_lock);
	clk[S32GEN1_CLK_PFE_EMAC_0_TX] = s32_clk_divider_flags("pfe_emac_0_tx",
		"pfe_emac_0_tx_sel",
		CGM_MUXn_DC(mc_cgm2_base, 1), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, 0, &s32gen1_lock);

	/* PFE EMAC 1 clocks */
	clk[S32GEN1_CLK_PFE_EMAC_1_TX_SEL] = s32_clk_mux_table(
		"pfe_emac_1_tx_sel",
		CGM_MUXn_CSC(mc_cgm2_base, 2),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		pfe_emac_1_tx_sels, ARRAY_SIZE(pfe_emac_1_tx_sels),
		pfe_emac_1_tx_mux_idx, &s32gen1_lock);
	clk[S32GEN1_CLK_PFE_EMAC_1_TX] = s32_clk_divider_flags("pfe_emac_1_tx",
		"pfe_emac_1_tx_sel",
		CGM_MUXn_DC(mc_cgm2_base, 2), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, 0, &s32gen1_lock);

	/* PFE EMAC 2 clocks */
	clk[S32GEN1_CLK_PFE_EMAC_2_TX_SEL] = s32_clk_mux_table(
		"pfe_emac_2_tx_sel",
		CGM_MUXn_CSC(mc_cgm2_base, 3),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		pfe_emac_2_tx_sels, ARRAY_SIZE(pfe_emac_2_tx_sels),
		pfe_emac_2_tx_mux_idx, &s32gen1_lock);
	clk[S32GEN1_CLK_PFE_EMAC_2_TX] = s32_clk_divider_flags("pfe_emac_2_tx",
		"pfe_emac_2_tx_sel",
		CGM_MUXn_DC(mc_cgm2_base, 3), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, 0, &s32gen1_lock);
}

static void __init s32r45_extra_clocks_init(struct device_node *clocking_node)
{
	/* ACCEL_3_CLK (SPT) */
	clk[S32GEN1_CLK_ACCEL_3] = s32_clk_mux_table("accel_3",
		CGM_MUXn_CSC(mc_cgm2_base, 0),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		accel_3_sels, ARRAY_SIZE(accel_3_sels), accel_3_mux_idx,
		&s32gen1_lock);
	clk[S32GEN1_CLK_ACCEL_3_DIV3] = s32_clk_fixed_factor("accel_3",
		"accel_3_div3", 1, 3);

	/* ACCEL_4_CLK (LAX) */
	clk[S32GEN1_CLK_ACCEL_4] = s32_clk_mux_table("accel_4",
		CGM_MUXn_CSC(mc_cgm2_base, 1),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		accel_4_sels, ARRAY_SIZE(accel_4_sels), accel_4_mux_idx,
		&s32gen1_lock);

	/* GMAC 1 clock */
	clk[S32GEN1_CLK_GMAC_1_TX_SEL] = s32_clk_mux_table("gmac_1_tx_sel",
		CGM_MUXn_CSC(mc_cgm2_base, 2),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		gmac_1_tx_sels, ARRAY_SIZE(gmac_1_tx_sels), gmac_1_tx_mux_idx,
		&s32gen1_lock);
	clk[S32GEN1_CLK_GMAC_1_TX] = s32_clk_divider_flags("gmac_1_tx",
		"gmac_1_tx_sel",
		CGM_MUXn_DC(mc_cgm2_base, 2), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, 0, &s32gen1_lock);
}

static void __init s32gen1_clocks_init(struct device_node *clocking_node)
{
	struct device_node *np;
	u32 armpll_pllodiv[] = {
		ARM_PLLDIG_PLLODIV0, ARM_PLLDIG_PLLODIV1
	};
	u32 periphpll_pllodiv[] = {
		PERIPH_PLLDIG_PLLODIV0, PERIPH_PLLDIG_PLLODIV1,
		PERIPH_PLLDIG_PLLODIV2, PERIPH_PLLDIG_PLLODIV3,

		PERIPH_PLLDIG_PLLODIV4, PERIPH_PLLDIG_PLLODIV5,
		PERIPH_PLLDIG_PLLODIV6
	};
	u32 ddrpll_pllodiv[] = { DDR_PLLDIG_PLLODIV0 };
	u32 accelpll_pllodiv[] = {
		ACCEL_PLLDIG_PLLODIV0, ACCEL_PLLDIG_PLLODIV1
	};

	clk[S32GEN1_CLK_DUMMY] = s32_clk_fixed("dummy", 0);
	clk[S32GEN1_CLK_FIRC] = s32_obtain_fixed_clock("firc", 0);

	np = clocking_node;
	armpll = of_iomap(np, 0);
	if (WARN_ON(!armpll))
		return;

	periphpll = of_iomap(np, 1);
	if (WARN_ON(!periphpll))
		return;

	accelpll = of_iomap(np, 2);
	if (WARN_ON(!accelpll))
		return;

	ddrpll = of_iomap(np, 3);
	if (WARN_ON(!ddrpll))
		return;

	armdfs = of_iomap(np, 4);
	if (WARN_ON(!armdfs))
		return;

	periphdfs = of_iomap(np, 5);
	if (WARN_ON(!periphdfs))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-mc_cgm0");
	mc_cgm0_base = of_iomap(np, 0);
	if (WARN_ON(!mc_cgm0_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-mc_cgm1");
	mc_cgm1_base = of_iomap(np, 0);
	if (WARN_ON(!mc_cgm1_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-mc_cgm2");
	mc_cgm2_base = of_iomap(np, 0);
	if (WARN_ON(!mc_cgm2_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-mc_cgm5");
	mc_cgm5_base = of_iomap(np, 0);
	if (WARN_ON(!mc_cgm5_base))
		return;

	clk[S32GEN1_CLK_FXOSC] = s32gen1_fxosc("fsl,s32gen1-fxosc");

	clk[S32GEN1_CLK_ARMPLL_SRC_SEL] = s32gen1_clk_pll_mux("armpll_sel",
		PLLDIG_PLLCLKMUX(armpll),
		PLLDIG_PLLCLKMUX_REFCLKSEL_OFFSET,
		PLLDIG_PLLCLKMUX_REFCLKSEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels), osc_mux_ids, &s32gen1_lock);

	clk[S32GEN1_CLK_PERIPHPLL_SRC_SEL] = s32gen1_clk_pll_mux(
		"periphpll_sel",
		PLLDIG_PLLCLKMUX(periphpll),
		PLLDIG_PLLCLKMUX_REFCLKSEL_OFFSET,
		PLLDIG_PLLCLKMUX_REFCLKSEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels), osc_mux_ids, &s32gen1_lock);

	clk[S32GEN1_CLK_DDRPLL_SRC_SEL] = s32gen1_clk_pll_mux("ddrpll_sel",
		PLLDIG_PLLCLKMUX(ddrpll),
		PLLDIG_PLLCLKMUX_REFCLKSEL_OFFSET,
		PLLDIG_PLLCLKMUX_REFCLKSEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels), osc_mux_ids, &s32gen1_lock);

	clk[S32GEN1_CLK_ACCELPLL_SRC_SEL] = s32gen1_clk_pll_mux("accelpll_sel",
		PLLDIG_PLLCLKMUX(accelpll),
		PLLDIG_PLLCLKMUX_REFCLKSEL_OFFSET,
		PLLDIG_PLLCLKMUX_REFCLKSEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels), osc_mux_ids, &s32gen1_lock);

	/* ARM_PLL */
	clk[S32GEN1_CLK_ARMPLL_VCO] = s32gen1_clk_plldig(S32GEN1_PLLDIG_ARM,
		"armpll_vco",  "armpll_sel", clk[S32GEN1_CLK_ACCELPLL_SRC_SEL],
		armpll, armpll_pllodiv, ARMPLL_PHI_Nr);

	clk[S32GEN1_CLK_ARMPLL_PHI0] = s32gen1_clk_plldig_phi(
		S32GEN1_PLLDIG_ARM, "armpll_phi0", "armpll_vco", armpll, 0);

	clk[S32GEN1_CLK_ARMPLL_PHI1] = s32gen1_clk_plldig_phi(
		S32GEN1_PLLDIG_ARM, "armpll_phi1", "armpll_vco", armpll, 1);

	clk[S32GEN1_CLK_ARMPLL_DFS1] = s32gen1_clk_dfs(S32GEN1_PLLDIG_ARM,
		 "armpll_dfs1", "armpll_vco",
		 armdfs, 1);

	clk[S32GEN1_CLK_ARMPLL_DFS2] = s32gen1_clk_dfs(S32GEN1_PLLDIG_ARM,
		 "armpll_dfs2", "armpll_vco",
		 armdfs, 2);

	clk[S32GEN1_CLK_ARMPLL_DFS3] = s32gen1_clk_dfs(S32GEN1_PLLDIG_ARM,
		 "armpll_dfs3", "armpll_vco",
		 armdfs, 3);

	clk[S32GEN1_CLK_ARMPLL_DFS4] = s32gen1_clk_dfs(S32GEN1_PLLDIG_ARM,
		 "armpll_dfs4", "armpll_vco",
		 armdfs, 4);

	clk[S32GEN1_CLK_ARMPLL_DFS5] = s32gen1_clk_dfs(S32GEN1_PLLDIG_ARM,
		 "armpll_dfs5", "armpll_vco",
		 armdfs, 5);

	clk[S32GEN1_CLK_ARMPLL_DFS6] = s32gen1_clk_dfs(S32GEN1_PLLDIG_ARM,
		 "armpll_dfs6", "armpll_vco",
		 armdfs, 6);

	/* XBAR CLKS */
	clk[S32GEN1_CLK_XBAR_SEL] = s32_clk_mux_table("xbar_sel",
		CGM_MUXn_CSC(mc_cgm0_base, 0),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		xbar_sels, ARRAY_SIZE(xbar_sels), xbar_mux_idx, &s32gen1_lock);

	clk[S32GEN1_CLK_XBAR] = s32_clk_divider("xbar", "xbar_sel",
		CGM_MUXn_DC(mc_cgm0_base, 0), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, &s32gen1_lock);

	clk[S32GEN1_CLK_XBAR_DIV2] = s32_clk_fixed_factor("xbar_div2",
		"xbar", 1, 2);
	clk[S32GEN1_CLK_XBAR_DIV3] = s32_clk_fixed_factor("xbar_div3",
		"xbar", 1, 3);
	clk[S32GEN1_CLK_XBAR_DIV4] = s32_clk_fixed_factor("xbar_div4",
		"xbar", 1, 4);

	clk[S32GEN1_CLK_XBAR_DIV6] = s32_clk_fixed_factor("xbar_div6",
		"xbar", 1, 6);

	/* PERIPH_PLL */
	clk[S32GEN1_CLK_PERIPHPLL_VCO] = s32gen1_clk_plldig(
		S32GEN1_PLLDIG_PERIPH, "periphpll_vco", "periphpll_sel",
		clk[S32GEN1_CLK_PERIPHPLL_SRC_SEL], periphpll,
		periphpll_pllodiv, PERIPHPLL_PHI_Nr);

	clk[S32GEN1_CLK_PERIPHPLL_PHI0] =
		s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH,
		"periphpll_phi0", "periphpll_vco",
		periphpll, 0);

	clk[S32GEN1_CLK_PERIPHPLL_PHI1] =
		s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH,
		"periphpll_phi1", "periphpll_vco",
		periphpll, 1);

	clk[S32GEN1_CLK_PERIPHPLL_PHI2] =
		s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH,
		"periphpll_phi2", "periphpll_vco",
		periphpll, 2);

	clk[S32GEN1_CLK_PERIPHPLL_PHI3] =
		s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH,
		"periphpll_phi3", "periphpll_vco",
		periphpll, 3);

	clk[S32GEN1_CLK_PERIPHPLL_PHI4] =
		s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH,
		"periphpll_phi4", "periphpll_vco",
		periphpll, 4);

	clk[S32GEN1_CLK_PERIPHPLL_PHI5] =
		s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH,
		"periphpll_phi5", "periphpll_vco",
		periphpll, 5);

	clk[S32GEN1_CLK_PERIPHPLL_PHI6] =
		s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH,
		"periphpll_phi6", "periphpll_vco",
		periphpll, 6);

	clk[S32GEN1_CLK_PERIPHPLL_PHI7] =
		s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH,
		"periphpll_phi7", "periphpll_vco",
		periphpll, 7);

	clk[S32GEN1_CLK_PERIPHPLL_DFS1] = s32gen1_clk_dfs(S32GEN1_PLLDIG_PERIPH,
		 "periphll_dfs1", "periphpll_vco",
		 periphdfs, 1);

	clk[S32GEN1_CLK_PERIPHPLL_DFS2] = s32gen1_clk_dfs(S32GEN1_PLLDIG_PERIPH,
		 "periphll_dfs2", "periphpll_vco",
		 periphdfs, 2);

	clk[S32GEN1_CLK_PERIPHPLL_DFS3] = s32gen1_clk_dfs(S32GEN1_PLLDIG_PERIPH,
		 "periphll_dfs3", "periphpll_vco",
		 periphdfs, 3);

	clk[S32GEN1_CLK_PERIPHPLL_DFS4] = s32gen1_clk_dfs(S32GEN1_PLLDIG_PERIPH,
		 "periphll_dfs4", "periphpll_vco",
		 periphdfs, 4);

	clk[S32GEN1_CLK_PERIPHPLL_DFS5] = s32gen1_clk_dfs(S32GEN1_PLLDIG_PERIPH,
		 "periphll_dfs5", "periphpll_vco",
		 periphdfs, 5);

	clk[S32GEN1_CLK_PERIPHPLL_DFS6] = s32gen1_clk_dfs(S32GEN1_PLLDIG_PERIPH,
		 "periphll_dfs6", "periphpll_vco",
		 periphdfs, 6);

	clk[S32GEN1_CLK_SERDES_INT_REF] = s32_clk_fixed_factor("serdes_int",
		"periphpll_phi0", 1, 1);
	clk[S32GEN1_CLK_SERDES_EXT_REF] = s32_obtain_fixed_clock("serdes_ext", 0);

	/* PER Clock */
	clk[S32GEN1_CLK_PER_SEL] = s32_clk_mux_table("per_sel",
		CGM_MUXn_CSC(mc_cgm0_base, 3),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		per_sels, ARRAY_SIZE(per_sels), per_mux_ids, &s32gen1_lock);

	clk[S32GEN1_CLK_PER] = s32_clk_divider("per", "per_sel",
		CGM_MUXn_DC(mc_cgm0_base, 3), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, &s32gen1_lock);

	/* FTM0 Clock */
	clk[S32GEN1_CLK_FTM0_REF_SEL] = s32_clk_mux_table("ftm0_ref_sel",
		CGM_MUXn_CSC(mc_cgm0_base, 4),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		ftm0_sels, ARRAY_SIZE(ftm0_sels), ftm0_mux_ids, &s32gen1_lock);

	clk[S32GEN1_CLK_FTM0_REF] = s32_clk_divider("ftm0_ref", "ftm0_ref_sel",
		CGM_MUXn_DC(mc_cgm0_base, 4), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, &s32gen1_lock);

	/* FTM1 Clock */
	clk[S32GEN1_CLK_FTM1_REF] = s32_clk_mux_table("ftm1_ref_sel",
		CGM_MUXn_CSC(mc_cgm0_base, 5),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		ftm1_sels, ARRAY_SIZE(ftm1_sels), ftm1_mux_ids, &s32gen1_lock);

	clk[S32GEN1_CLK_FTM1_REF] = s32_clk_divider("ftm1_ref", "ftm1_ref_sel",
		CGM_MUXn_DC(mc_cgm0_base, 5), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, &s32gen1_lock);

	/* Can Clock */
	clk[S32GEN1_CLK_CAN] = s32_clk_mux_table("can",
		CGM_MUXn_CSC(mc_cgm0_base, 7),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		can_sels, ARRAY_SIZE(can_sels), can_mux_ids, &s32gen1_lock);

	/* Lin Clock */
	clk[S32GEN1_CLK_LIN_BAUD] = s32_clk_mux_table("lin_baud",
		CGM_MUXn_CSC(mc_cgm0_base, 8),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		lin_sels, ARRAY_SIZE(lin_sels), lin_mux_idx, &s32gen1_lock);

	clk[S32GEN1_CLK_LIN] = s32_clk_fixed_factor("lin",
		"lin_baud", 1, 2);

	/* DSPI Clock */
	clk[S32GEN1_CLK_DSPI] = s32_clk_mux_table("dspi",
		CGM_MUXn_CSC(mc_cgm0_base, 16),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		dspi_sels, ARRAY_SIZE(dspi_sels), dspi_mux_idx, &s32gen1_lock);

	/* QSPI Clock */
	clk[S32GEN1_CLK_QSPI_SEL] = s32_clk_mux_table("qspi_sel",
		CGM_MUXn_CSC(mc_cgm0_base, 12),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		qspi_sels, ARRAY_SIZE(qspi_sels), qspi_mux_idx, &s32gen1_lock);

	clk[S32GEN1_CLK_QSPI_2X] = s32_clk_divider("qspi_2x", "qspi_sel",
		CGM_MUXn_DC(mc_cgm0_base, 12), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, &s32gen1_lock);

	clk[S32GEN1_CLK_QSPI_1X] = s32_clk_fixed_factor("qspi_1x",
		"qspi_2x", 1, 2);

	/* A53 cores */
	clk[S32GEN1_CLK_A53] = s32_clk_mux_table("a53_core",
		CGM_MUXn_CSC(mc_cgm1_base, 0),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		a53_core_sels, ARRAY_SIZE(a53_core_sels), a53_core_mux_idx,
		&s32gen1_lock);
	clk[S32GEN1_CLK_A53_DIV2] = s32_clk_fixed_factor("a53_core_div2",
		"a53_core", 1, 2);
	clk[S32GEN1_CLK_A53_DIV10] = s32_clk_fixed_factor("a53_core_div10",
		"a53_core", 1, 10);

	/* SDHC Clock */
	clk[S32GEN1_CLK_SDHC_SEL] = s32_clk_mux_table("sdhc_sel",
		CGM_MUXn_CSC(mc_cgm0_base, 14),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		sdhc_sels, ARRAY_SIZE(sdhc_sels), sdhc_mux_idx, &s32gen1_lock);

	clk[S32GEN1_CLK_SDHC] = s32_clk_divider("sdhc", "sdhc_sel",
		CGM_MUXn_DC(mc_cgm0_base, 14), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, &s32gen1_lock);

	/* DDR_PLL */
	clk[S32GEN1_CLK_DDRPLL_VCO] = s32gen1_clk_plldig(S32GEN1_PLLDIG_DDR,
		"ddrpll_vco", "ddrpll_sel",
		clk[S32GEN1_CLK_DDRPLL_SRC_SEL],
		ddrpll, ddrpll_pllodiv, DDRPLL_PHI_Nr);

	clk[S32GEN1_CLK_DDRPLL_PHI0] = s32gen1_clk_plldig_phi(
		S32GEN1_PLLDIG_DDR, "ddrpll_phi0", "ddrpll_vco", ddrpll, 0);

	/* ACCEL_PLL */
	clk[S32GEN1_CLK_ACCELPLL_VCO] = s32gen1_clk_plldig(S32GEN1_PLLDIG_ACCEL,
		"accelpll_vco", "accelpll_sel",
		clk[S32GEN1_CLK_ACCELPLL_SRC_SEL],
		accelpll, accelpll_pllodiv,
		ACCELPLL_PHI_Nr);

	clk[S32GEN1_CLK_ACCELPLL_PHI0] = s32gen1_clk_plldig_phi(
		S32GEN1_PLLDIG_ACCEL, "accelpll_phi0", "accelpll_vco",
		accelpll, 0);

	clk[S32GEN1_CLK_ACCELPLL_PHI1] = s32gen1_clk_plldig_phi(
		S32GEN1_PLLDIG_ACCEL, "accelpll_phi1", "accelpll_vco",
		accelpll, 1);

	clk[S32GEN1_CLK_DDR] = s32_clk_mux_table("ddr",
		CGM_MUXn_CSC(mc_cgm5_base, 0),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		ddr_sels, ARRAY_SIZE(ddr_sels), ddr_mux_idx, &s32gen1_lock);

	/* GMAC clock */
	clk[S32GEN1_CLK_GMAC_0_TX_SEL] = s32_clk_mux_table("gmac_0_tx_sel",
		CGM_MUXn_CSC(mc_cgm0_base, 10),
		MC_CGM_MUXn_CSC_SELCTL_OFFSET,
		MC_CGM_MUXn_CSC_SELCTL_SIZE,
		gmac_0_tx_sels, ARRAY_SIZE(gmac_0_tx_sels), gmac_0_tx_mux_idx,
		&s32gen1_lock);
	clk[S32GEN1_CLK_GMAC_0_TX] = s32_clk_divider_flags("gmac_0_tx",
		"gmac_0_tx_sel",
		CGM_MUXn_DC(mc_cgm0_base, 10), MC_CGM_MUX_DCn_DIV_OFFSET,
		MC_CGM_MUX_DCn_DIV_SIZE, 0, &s32gen1_lock);

	/* Add the clocks to provider list */
	clk_data.clks = clk;
	clk_data.clk_num = ARRAY_SIZE(clk);
	of_clk_add_provider(clocking_node, of_clk_src_onecell_get, &clk_data);
}

static void __init s32v344_clocks_init(struct device_node *clks_node)
{
	s32gen1_clocks_init(clks_node);
}

static void __init s32g274_clocks_init(struct device_node *clks_node)
{
	s32gen1_clocks_init(clks_node);
	s32g274_extra_clocks_init(clks_node);
}

static void __init s32r45_clocks_init(struct device_node *clks_node)
{
	s32gen1_clocks_init(clks_node);
	s32r45_extra_clocks_init(clks_node);
}

CLK_OF_DECLARE(S32V344, "fsl,s32v344-clocking", s32v344_clocks_init);
CLK_OF_DECLARE(S32G274, "fsl,s32g274-clocking", s32g274_clocks_init);
CLK_OF_DECLARE(S32R45, "fsl,s32r45-clocking", s32r45_clocks_init);
