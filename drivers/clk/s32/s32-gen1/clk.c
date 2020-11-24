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
#include <linux/clk-provider.h>
#include <dt-bindings/clock/s32gen1-clock.h>
#include <dt-bindings/clock/s32g274a-scmi-clock.h>
#include <dt-bindings/clock/s32r45-scmi-clock.h>
#include <linux/syscore_ops.h>

#include "clk.h"
#include "mc_cgm.h"

#define MAX_SCMI_CLKS	100

static struct s32gen1_clk_modules clk_modules;

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

struct s32gen1_clocks {
	struct clk_onecell_data plat_clks;
	struct clk_onecell_data scmi_clks;
};

struct clk *s32gen1_clk_src_get(struct of_phandle_args *clkspec, void *data)
{
	struct s32gen1_clocks *clks = data;
	unsigned int idx = clkspec->args[0];
	struct clk *clk;

	if (idx >= S32GEN1_CLK_PLAT_BASE)
		clk = clks->plat_clks.clks[S32GEN1_CLK_ARR_INDEX(idx)];
	else
		clk = clks->scmi_clks.clks[idx];

	if (!clk)
		pr_err("Unhandled clock ID : %u\n", idx);

	return clk;
}

static struct clk *clk[S32GEN1_CLK_ARR_INDEX(S32GEN1_CLK_END)];
static struct clk *scmi_clk[MAX_SCMI_CLKS];
static struct s32gen1_clocks plat_clks;

static void set_plat_clk(uint32_t pos, struct clk *c)
{
	if (pos < S32GEN1_CLK_PLAT_BASE ||
	    S32GEN1_CLK_ARR_INDEX(pos) >= ARRAY_SIZE(clk)) {
		pr_err("Invalid clock ID: %u\n", pos);
		return;
	}

	clk[S32GEN1_CLK_ARR_INDEX(pos)] = c;
}

static struct clk *get_plat_clk(uint32_t pos)
{
	return clk[S32GEN1_CLK_ARR_INDEX(pos)];
}

static void __init s32g274_extra_clocks_init(struct device_node *clocking_node)
{
	struct clk *c;

	/* PFE */
	c = s32gen1_clk_cgm_mux("pfe_pe_sel",
				clk_modules.mc_cgm2_base,  0,
				pfe_pe_sels, ARRAY_SIZE(pfe_pe_sels),
				pfe_pe_mux_idx, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PFE_PE_SEL, c);
	c = s32gen1_clk_part_block("pfe_sys_part_block", "pfe_pe_sel",
				   &clk_modules, 2, 3, &s32gen1_lock, false);
	set_plat_clk(S32GEN1_CLK_PFE_SYS_PART_BLOCK, c);
	c = s32gen1_clk_cgm_div("pfe_pe",
				"pfe_sys_part_block",
				clk_modules.mc_cgm2_base, 0, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PFE_PE, c);
	c = s32_clk_fixed_factor("pfe_sys",
				 "pfe_pe", 1, 2);
	set_plat_clk(S32GEN1_CLK_PFE_SYS, c);

	/* PFE EMAC 0 clocks */
	c = s32gen1_clk_cgm_mux("pfe_emac_0_tx_sel",
				clk_modules.mc_cgm2_base,  1,
				pfe_emac_0_tx_sels,
				ARRAY_SIZE(pfe_emac_0_tx_sels),
				pfe_emac_0_tx_mux_idx, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PFE_EMAC_0_TX_SEL, c);
	c = s32gen1_clk_part_block("pfe0_tx_part_block", "pfe_emac_0_tx_sel",
				   &clk_modules, 2, 0, &s32gen1_lock, false);
	set_plat_clk(S32GEN1_CLK_PFE0_TX_PART_BLOCK, c);
	c = s32gen1_clk_cgm_div("pfe_emac_0_tx", "pfe0_tx_part_block",
				clk_modules.mc_cgm2_base, 1, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PFE_EMAC_0_TX, c);

	/* PFE EMAC 1 clocks */
	c = s32gen1_clk_cgm_mux("pfe_emac_1_tx_sel", clk_modules.mc_cgm2_base,
				2, pfe_emac_1_tx_sels,
				ARRAY_SIZE(pfe_emac_1_tx_sels),
				pfe_emac_1_tx_mux_idx, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PFE_EMAC_1_TX_SEL, c);
	c = s32gen1_clk_part_block("pfe1_tx_part_block", "pfe_emac_1_tx_sel",
				   &clk_modules, 2, 1, &s32gen1_lock, false);
	set_plat_clk(S32GEN1_CLK_PFE1_TX_PART_BLOCK, c);
	c = s32gen1_clk_cgm_div("pfe_emac_1_tx", "pfe1_tx_part_block",
				clk_modules.mc_cgm2_base, 2, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PFE_EMAC_1_TX, c);

	/* PFE EMAC 2 clocks */
	c = s32gen1_clk_cgm_mux("pfe_emac_2_tx_sel", clk_modules.mc_cgm2_base,
				3, pfe_emac_2_tx_sels,
				ARRAY_SIZE(pfe_emac_2_tx_sels),
				pfe_emac_2_tx_mux_idx, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PFE_EMAC_2_TX_SEL, c);
	c = s32gen1_clk_part_block("pfe2_tx_part_block", "pfe_emac_2_tx_sel",
				   &clk_modules, 2, 2, &s32gen1_lock, false);
	set_plat_clk(S32GEN1_CLK_PFE2_TX_PART_BLOCK, c);
	c = s32gen1_clk_cgm_div("pfe_emac_2_tx", "pfe2_tx_part_block",
				clk_modules.mc_cgm2_base, 3, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PFE_EMAC_2_TX, c);
}

static void __init s32r45_extra_clocks_init(struct device_node *clocking_node)
{
	struct clk *c;

	/* ACCEL_3_CLK (SPT) */
	c = s32gen1_clk_cgm_mux("accel_3", clk_modules.mc_cgm2_base,  0,
				accel_3_sels, ARRAY_SIZE(accel_3_sels),
				accel_3_mux_idx, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_ACCEL_3, c);
	c = s32_clk_fixed_factor("accel_3", "accel_3_div3", 1, 3);
	set_plat_clk(S32GEN1_CLK_ACCEL_3_DIV3, c);

	/* ACCEL_4_CLK (LAX) */
	c = s32gen1_clk_cgm_mux("accel_4", clk_modules.mc_cgm2_base,  1,
				accel_4_sels, ARRAY_SIZE(accel_4_sels),
				accel_4_mux_idx, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_ACCEL_4, c);

	/* GMAC 1 clock */
	c = s32gen1_clk_cgm_mux("gmac_1_tx_sel",
				clk_modules.mc_cgm2_base,  2, gmac_1_tx_sels,
				ARRAY_SIZE(gmac_1_tx_sels), gmac_1_tx_mux_idx,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_GMAC_1_TX_SEL, c);
	c = s32gen1_clk_cgm_div("gmac_1_tx", "gmac_1_tx_sel",
				clk_modules.mc_cgm2_base, 2, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_GMAC_1_TX, c);
}

void __init s32gen1_clocks_init(struct device_node *clocking_node)
{
	struct device_node *np;
	struct clk *c;

	u32 armpll_pllodiv[] = {
		ARM_PLLDIG_PLLODIV0, ARM_PLLDIG_PLLODIV1
	};
	u32 periphpll_pllodiv[] = {
		PERIPH_PLLDIG_PLLODIV0, PERIPH_PLLDIG_PLLODIV1,
		PERIPH_PLLDIG_PLLODIV2, PERIPH_PLLDIG_PLLODIV3,

		PERIPH_PLLDIG_PLLODIV4, PERIPH_PLLDIG_PLLODIV5,
		PERIPH_PLLDIG_PLLODIV6, PERIPH_PLLDIG_PLLODIV7,
	};
	u32 ddrpll_pllodiv[] = { DDR_PLLDIG_PLLODIV0 };
	u32 accelpll_pllodiv[] = {
		ACCEL_PLLDIG_PLLODIV0, ACCEL_PLLDIG_PLLODIV1
	};

	c = s32_clk_fixed("dummy", 0);
	set_plat_clk(S32GEN1_CLK_DUMMY, c);
	c = s32_obtain_fixed_clock("firc", 0);
	set_plat_clk(S32GEN1_CLK_FIRC, c);

	np = clocking_node;
	clk_modules.armpll = of_iomap(np, 0);
	if (WARN_ON(!clk_modules.armpll))
		return;

	clk_modules.periphpll = of_iomap(np, 1);
	if (WARN_ON(!clk_modules.periphpll))
		return;

	clk_modules.accelpll = of_iomap(np, 2);
	if (WARN_ON(!clk_modules.accelpll))
		return;

	clk_modules.ddrpll = of_iomap(np, 3);
	if (WARN_ON(!clk_modules.ddrpll))
		return;

	clk_modules.armdfs = of_iomap(np, 4);
	if (WARN_ON(!clk_modules.armdfs))
		return;

	clk_modules.periphdfs = of_iomap(np, 5);
	if (WARN_ON(!clk_modules.periphdfs))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-mc_cgm0");
	clk_modules.mc_cgm0_base = of_iomap(np, 0);
	if (WARN_ON(!clk_modules.mc_cgm0_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-mc_cgm1");
	clk_modules.mc_cgm1_base = of_iomap(np, 0);
	if (WARN_ON(!clk_modules.mc_cgm1_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-mc_cgm2");
	clk_modules.mc_cgm2_base = of_iomap(np, 0);
	if (WARN_ON(!clk_modules.mc_cgm2_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-mc_cgm5");
	clk_modules.mc_cgm5_base = of_iomap(np, 0);
	if (WARN_ON(!clk_modules.mc_cgm5_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-mc_me");
	clk_modules.mc_me = of_iomap(np, 0);
	if (WARN_ON(!clk_modules.mc_me))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-rdc");
	clk_modules.rdc = of_iomap(np, 0);
	if (WARN_ON(!clk_modules.rdc))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-rgm");
	clk_modules.rgm = of_iomap(np, 0);
	if (WARN_ON(!clk_modules.rgm))
		return;

	c = s32gen1_fxosc("fsl,s32gen1-fxosc");
	set_plat_clk(S32GEN1_CLK_FXOSC, c);

	c = s32gen1_clk_pll_mux("armpll_sel",
				PLLDIG_PLLCLKMUX(clk_modules.armpll),
				PLLDIG_PLLCLKMUX_REFCLKSEL_OFFSET,
				PLLDIG_PLLCLKMUX_REFCLKSEL_SIZE,
				osc_sels, ARRAY_SIZE(osc_sels), osc_mux_ids,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_ARMPLL_SRC_SEL, c);

	c = s32gen1_clk_pll_mux("periphpll_sel",
				PLLDIG_PLLCLKMUX(clk_modules.periphpll),
				PLLDIG_PLLCLKMUX_REFCLKSEL_OFFSET,
				PLLDIG_PLLCLKMUX_REFCLKSEL_SIZE, osc_sels,
				ARRAY_SIZE(osc_sels), osc_mux_ids,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_SRC_SEL, c);

	c = s32gen1_clk_pll_mux("ddrpll_sel",
				PLLDIG_PLLCLKMUX(clk_modules.ddrpll),
				PLLDIG_PLLCLKMUX_REFCLKSEL_OFFSET,
				PLLDIG_PLLCLKMUX_REFCLKSEL_SIZE,
				osc_sels, ARRAY_SIZE(osc_sels), osc_mux_ids,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_DDRPLL_SRC_SEL, c);

	c = s32gen1_clk_pll_mux("accelpll_sel",
				PLLDIG_PLLCLKMUX(clk_modules.accelpll),
				PLLDIG_PLLCLKMUX_REFCLKSEL_OFFSET,
				PLLDIG_PLLCLKMUX_REFCLKSEL_SIZE, osc_sels,
				ARRAY_SIZE(osc_sels), osc_mux_ids,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_ACCELPLL_SRC_SEL, c);

	/* ARM_PLL */
	c = s32gen1_clk_plldig(S32GEN1_PLLDIG_ARM, "armpll_vco", "armpll_sel",
			       get_plat_clk(S32GEN1_CLK_ACCELPLL_SRC_SEL),
			       clk_modules.armpll, armpll_pllodiv,
			       ARRAY_SIZE(armpll_pllodiv));
	set_plat_clk(S32GEN1_CLK_ARMPLL_VCO, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_ARM, "armpll_phi0",
				   "armpll_vco", clk_modules.armpll, 0);
	set_plat_clk(S32GEN1_CLK_ARMPLL_PHI0, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_ARM, "armpll_phi1",
				   "armpll_vco", clk_modules.armpll, 1);
	set_plat_clk(S32GEN1_CLK_ARMPLL_PHI1, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_ARM, "armpll_dfs1", "armpll_vco",
				clk_modules.armdfs, 1);
	set_plat_clk(S32GEN1_CLK_ARMPLL_DFS1, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_ARM, "armpll_dfs2", "armpll_vco",
				clk_modules.armdfs, 2);
	set_plat_clk(S32GEN1_CLK_ARMPLL_DFS2, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_ARM, "armpll_dfs3", "armpll_vco",
				clk_modules.armdfs, 3);
	set_plat_clk(S32GEN1_CLK_ARMPLL_DFS3, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_ARM, "armpll_dfs4", "armpll_vco",
				clk_modules.armdfs, 4);
	set_plat_clk(S32GEN1_CLK_ARMPLL_DFS4, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_ARM, "armpll_dfs5", "armpll_vco",
				clk_modules.armdfs, 5);
	set_plat_clk(S32GEN1_CLK_ARMPLL_DFS5, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_ARM, "armpll_dfs6", "armpll_vco",
				clk_modules.armdfs, 6);
	set_plat_clk(S32GEN1_CLK_ARMPLL_DFS6, c);

	/* XBAR CLKS */
	c = s32gen1_clk_cgm_mux("xbar_sel", clk_modules.mc_cgm0_base,  0,
				xbar_sels, ARRAY_SIZE(xbar_sels), xbar_mux_idx,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_XBAR_SEL, c);

	c = s32_clk_fixed_factor("xbar", "xbar_sel", 1, 2);
	set_plat_clk(S32GEN1_CLK_XBAR, c);

	c = s32_clk_fixed_factor("xbar_div2", "xbar", 1, 2);
	set_plat_clk(S32GEN1_CLK_XBAR_DIV2, c);

	c = s32_clk_fixed_factor("xbar_div3", "xbar", 1, 3);
	set_plat_clk(S32GEN1_CLK_XBAR_DIV3, c);

	c = s32_clk_fixed_factor("xbar_div4", "xbar", 1, 4);
	set_plat_clk(S32GEN1_CLK_XBAR_DIV4, c);

	c = s32_clk_fixed_factor("xbar_div6", "xbar", 1, 6);
	set_plat_clk(S32GEN1_CLK_XBAR_DIV6, c);

	/* PERIPH_PLL */
	c = s32gen1_clk_plldig(S32GEN1_PLLDIG_PERIPH, "periphpll_vco",
			       "periphpll_sel",
			       get_plat_clk(S32GEN1_CLK_PERIPHPLL_SRC_SEL),
			       clk_modules.periphpll, periphpll_pllodiv,
			       ARRAY_SIZE(periphpll_pllodiv));
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_VCO, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH, "periphpll_phi0",
				   "periphpll_vco", clk_modules.periphpll, 0);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_PHI0, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH, "periphpll_phi1",
				   "periphpll_vco",
				   clk_modules.periphpll, 1);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_PHI1, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH, "periphpll_phi2",
				   "periphpll_vco", clk_modules.periphpll, 2);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_PHI2, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH, "periphpll_phi3",
				  "periphpll_vco", clk_modules.periphpll, 3);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_PHI3, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH, "periphpll_phi4",
				   "periphpll_vco", clk_modules.periphpll, 4);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_PHI4, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH, "periphpll_phi5",
				   "periphpll_vco", clk_modules.periphpll, 5);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_PHI5, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH, "periphpll_phi6",
				   "periphpll_vco", clk_modules.periphpll, 6);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_PHI6, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_PERIPH, "periphpll_phi7",
				   "periphpll_vco", clk_modules.periphpll, 7);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_PHI7, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_PERIPH, "periphll_dfs1",
				"periphpll_vco", clk_modules.periphdfs, 1);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_DFS1, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_PERIPH, "periphll_dfs2",
				"periphpll_vco", clk_modules.periphdfs, 2);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_DFS2, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_PERIPH, "periphll_dfs3",
				"periphpll_vco", clk_modules.periphdfs, 3);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_DFS3, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_PERIPH, "periphll_dfs4",
				"periphpll_vco", clk_modules.periphdfs, 4);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_DFS4, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_PERIPH, "periphll_dfs5",
				"periphpll_vco", clk_modules.periphdfs, 5);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_DFS5, c);

	c = s32gen1_clk_dfs_out(S32GEN1_PLLDIG_PERIPH, "periphll_dfs6",
				"periphpll_vco", clk_modules.periphdfs, 6);
	set_plat_clk(S32GEN1_CLK_PERIPHPLL_DFS6, c);

	c = s32_clk_fixed_factor("serdes_int", "periphpll_phi0", 1, 1);
	set_plat_clk(S32GEN1_CLK_SERDES_INT_REF, c);

	c = s32_obtain_fixed_clock("serdes_ext", 0);
	set_plat_clk(S32GEN1_CLK_SERDES_EXT_REF, c);

	/* PER Clock */
	c = s32gen1_clk_cgm_mux("per_sel", clk_modules.mc_cgm0_base,  3,
				per_sels, ARRAY_SIZE(per_sels), per_mux_ids,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PER_SEL, c);

	c = s32gen1_clk_cgm_div("per", "per_sel", clk_modules.mc_cgm0_base, 3,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_PER, c);

	/* FTM0 Clock */
	c = s32gen1_clk_cgm_mux("ftm0_ref_sel", clk_modules.mc_cgm0_base,  4,
				ftm0_sels, ARRAY_SIZE(ftm0_sels), ftm0_mux_ids,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_FTM0_REF_SEL, c);

	c = s32gen1_clk_cgm_div("ftm0_ref", "ftm0_ref_sel",
				clk_modules.mc_cgm0_base, 4, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_FTM0_REF, c);

	/* FTM1 Clock */
	c = s32gen1_clk_cgm_mux("ftm1_ref_sel", clk_modules.mc_cgm0_base,  5,
				ftm1_sels, ARRAY_SIZE(ftm1_sels), ftm1_mux_ids,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_FTM1_REF, c);

	c = s32gen1_clk_cgm_div("ftm1_ref", "ftm1_ref_sel",
				clk_modules.mc_cgm0_base, 5, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_FTM1_REF, c);

	/* Can Clock */
	c = s32gen1_clk_cgm_mux("can", clk_modules.mc_cgm0_base,  7, can_sels,
				ARRAY_SIZE(can_sels), can_mux_ids,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_CAN, c);

	/* Lin Clock */
	c = s32gen1_clk_cgm_mux("lin_baud", clk_modules.mc_cgm0_base,  8,
				lin_sels, ARRAY_SIZE(lin_sels), lin_mux_idx,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_LIN_BAUD, c);

	c = s32_clk_fixed_factor("lin", "lin_baud", 1, 2);
	set_plat_clk(S32GEN1_CLK_LIN, c);

	/* DSPI Clock */
	c = s32gen1_clk_cgm_mux("dspi", clk_modules.mc_cgm0_base,  16,
				dspi_sels, ARRAY_SIZE(dspi_sels), dspi_mux_idx,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_DSPI, c);

	/* QSPI Clock */
	c = s32gen1_clk_cgm_mux("qspi_sel", clk_modules.mc_cgm0_base,  12,
				qspi_sels, ARRAY_SIZE(qspi_sels), qspi_mux_idx,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_QSPI_SEL, c);

	c = s32gen1_clk_cgm_div("qspi_2x", "qspi_sel", clk_modules.mc_cgm0_base,
				12, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_QSPI_2X, c);

	c = s32_clk_fixed_factor("qspi_1x", "qspi_2x", 1, 2);
	set_plat_clk(S32GEN1_CLK_QSPI_1X, c);

	/* A53 cores */
	c = s32gen1_clk_cgm_mux("a53_core", clk_modules.mc_cgm1_base,  0,
				a53_core_sels, ARRAY_SIZE(a53_core_sels),
				a53_core_mux_idx, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_A53, c);

	c = s32_clk_fixed_factor("a53_core_div2", "a53_core", 1, 2);
	set_plat_clk(S32GEN1_CLK_A53_DIV2, c);

	c = s32_clk_fixed_factor("a53_core_div10", "a53_core", 1, 10);
	set_plat_clk(S32GEN1_CLK_A53_DIV10, c);

	/* SDHC Clock */
	c = s32gen1_clk_cgm_mux("sdhc_sel", clk_modules.mc_cgm0_base,  14,
				sdhc_sels, ARRAY_SIZE(sdhc_sels), sdhc_mux_idx,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_SDHC_SEL, c);

	c = s32gen1_clk_part_block("sdhc_part_block", "sdhc_sel", &clk_modules,
				   0, 0, &s32gen1_lock, true);
	set_plat_clk(S32GEN1_CLK_SDHC_PART_BLOCK, c);

	c = s32gen1_clk_cgm_div("sdhc", "sdhc_part_block",
				clk_modules.mc_cgm0_base, 14, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_SDHC, c);

	c =  s32gen1_clk_part_block("ddr_part_block", "ddrpll_sel",
				    &clk_modules, 0, 1, &s32gen1_lock, true);
	set_plat_clk(S32GEN1_CLK_DDR_PART_BLOCK, c);

	/* DDR_PLL */
	c = s32gen1_clk_plldig(S32GEN1_PLLDIG_DDR, "ddrpll_vco",
			       "ddr_part_block",
			       get_plat_clk(S32GEN1_CLK_DDRPLL_SRC_SEL),
			       clk_modules.ddrpll, ddrpll_pllodiv,
			       ARRAY_SIZE(ddrpll_pllodiv));
	set_plat_clk(S32GEN1_CLK_DDRPLL_VCO, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_DDR, "ddrpll_phi0",
				   "ddrpll_vco", clk_modules.ddrpll, 0);
	set_plat_clk(S32GEN1_CLK_DDRPLL_PHI0, c);

	/* ACCEL_PLL */
	c = s32gen1_clk_plldig(S32GEN1_PLLDIG_ACCEL, "accelpll_vco",
			       "accelpll_sel",
			       get_plat_clk(S32GEN1_CLK_ACCELPLL_SRC_SEL),
			       clk_modules.accelpll, accelpll_pllodiv,
			       ARRAY_SIZE(accelpll_pllodiv));
	set_plat_clk(S32GEN1_CLK_ACCELPLL_VCO, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_ACCEL, "accelpll_phi0",
				   "accelpll_vco", clk_modules.accelpll, 0);
	set_plat_clk(S32GEN1_CLK_ACCELPLL_PHI0, c);

	c = s32gen1_clk_plldig_phi(S32GEN1_PLLDIG_ACCEL, "accelpll_phi1",
				   "accelpll_vco", clk_modules.accelpll, 1);
	set_plat_clk(S32GEN1_CLK_ACCELPLL_PHI1, c);

	c = s32gen1_clk_cgm_mux("ddr", clk_modules.mc_cgm5_base,  0,
				ddr_sels, ARRAY_SIZE(ddr_sels), ddr_mux_idx,
				&s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_DDR, c);

	/* GMAC clock */
	c = s32gen1_clk_cgm_mux("gmac_0_tx_sel", clk_modules.mc_cgm0_base,  10,
				gmac_0_tx_sels, ARRAY_SIZE(gmac_0_tx_sels),
				gmac_0_tx_mux_idx, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_GMAC_0_TX_SEL, c);

	c = s32gen1_clk_cgm_div("gmac_0_tx", "gmac_0_tx_sel",
				clk_modules.mc_cgm0_base, 10, &s32gen1_lock);
	set_plat_clk(S32GEN1_CLK_GMAC_0_TX, c);

	/* Add the clocks to provider list */
	plat_clks.plat_clks.clks = clk;
	plat_clks.plat_clks.clk_num = ARRAY_SIZE(clk);
	of_clk_add_provider(clocking_node, s32gen1_clk_src_get, &plat_clks);
}

void s32gen1_scmi_clocks_init(void)
{
	scmi_clk[S32GEN1_SCMI_CLK_A53] =
		get_plat_clk(S32GEN1_CLK_A53);
	scmi_clk[S32GEN1_SCMI_CLK_SERDES_AXI] =
		get_plat_clk(S32GEN1_CLK_XBAR);
	scmi_clk[S32GEN1_SCMI_CLK_SERDES_AUX] =
		get_plat_clk(S32GEN1_CLK_FIRC);
	scmi_clk[S32GEN1_SCMI_CLK_SERDES_APB] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_FTM0_SYS] =
		get_plat_clk(S32GEN1_CLK_PER);
	scmi_clk[S32GEN1_SCMI_CLK_FTM0_EXT] =
		get_plat_clk(S32GEN1_CLK_FTM0_REF);
	scmi_clk[S32GEN1_SCMI_CLK_FTM1_SYS] =
		get_plat_clk(S32GEN1_CLK_PER);
	scmi_clk[S32GEN1_SCMI_CLK_FTM1_EXT] =
		get_plat_clk(S32GEN1_CLK_FTM1_REF);
	scmi_clk[S32GEN1_SCMI_CLK_FLEXCAN_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_FLEXCAN_SYS] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_FLEXCAN_CAN] =
		get_plat_clk(S32GEN1_CLK_CAN);
	scmi_clk[S32GEN1_SCMI_CLK_FLEXCAN_TS] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV2);
	scmi_clk[S32GEN1_SCMI_CLK_LINFLEX_XBAR] =
		get_plat_clk(S32GEN1_CLK_LIN);
	scmi_clk[S32GEN1_SCMI_CLK_LINFLEX_LIN] =
		get_plat_clk(S32GEN1_CLK_LIN_BAUD);

	scmi_clk[S32GEN1_SCMI_CLK_GMAC0_RX_SGMII] =
		get_plat_clk(S32GEN1_CLK_GMAC_0_RX);
	scmi_clk[S32GEN1_SCMI_CLK_GMAC0_TX_SGMII] =
		get_plat_clk(S32GEN1_CLK_GMAC_0_TX);
	scmi_clk[S32GEN1_SCMI_CLK_GMAC0_RX_RGMII] =
		get_plat_clk(S32GEN1_CLK_GMAC_0_RX);
	scmi_clk[S32GEN1_SCMI_CLK_GMAC0_TX_RGMII] =
		get_plat_clk(S32GEN1_CLK_GMAC_0_TX);
	scmi_clk[S32GEN1_SCMI_CLK_GMAC0_RX_RMII] =
		get_plat_clk(S32GEN1_CLK_GMAC_0_RX);
	scmi_clk[S32GEN1_SCMI_CLK_GMAC0_TX_RMII] =
		get_plat_clk(S32GEN1_CLK_GMAC_0_TX);
	scmi_clk[S32GEN1_SCMI_CLK_GMAC0_RX_MII] =
		get_plat_clk(S32GEN1_CLK_GMAC_0_RX);
	scmi_clk[S32GEN1_SCMI_CLK_GMAC0_TX_MII] =
		get_plat_clk(S32GEN1_CLK_GMAC_0_TX);
	scmi_clk[S32GEN1_SCMI_CLK_GMAC0_AXI] =
		get_plat_clk(S32GEN1_CLK_XBAR);

	scmi_clk[S32GEN1_SCMI_CLK_SPI_REG] =
		get_plat_clk(S32GEN1_CLK_DSPI);
	scmi_clk[S32GEN1_SCMI_CLK_SPI_MODULE] =
		get_plat_clk(S32GEN1_CLK_DSPI);
	scmi_clk[S32GEN1_SCMI_CLK_QSPI_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_QSPI_AHB] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_QSPI_FLASH2X] =
		get_plat_clk(S32GEN1_CLK_QSPI_2X);
	scmi_clk[S32GEN1_SCMI_CLK_QSPI_FLASH1X] =
		get_plat_clk(S32GEN1_CLK_QSPI_1X);
	scmi_clk[S32GEN1_SCMI_CLK_USDHC_AHB] =
		get_plat_clk(S32GEN1_CLK_XBAR);
	scmi_clk[S32GEN1_SCMI_CLK_USDHC_MODULE] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_USDHC_CORE] =
		get_plat_clk(S32GEN1_CLK_SDHC);
	scmi_clk[S32GEN1_SCMI_CLK_USDHC_MOD32K] =
		get_plat_clk(S32GEN1_CLK_SIRC);
	scmi_clk[S32GEN1_SCMI_CLK_DDR_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_DDR_PLL_REF] =
		get_plat_clk(S32GEN1_CLK_DDR);
	scmi_clk[S32GEN1_SCMI_CLK_DDR_AXI] =
		get_plat_clk(S32GEN1_CLK_DDR);
	scmi_clk[S32GEN1_SCMI_CLK_SRAM_AXI] =
		get_plat_clk(S32GEN1_CLK_XBAR);
	scmi_clk[S32GEN1_SCMI_CLK_SRAM_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_I2C_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_I2C_MODULE] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_RTC_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV6);
	scmi_clk[S32GEN1_SCMI_CLK_RTC_SIRC] =
		get_plat_clk(S32GEN1_CLK_SIRC);
	scmi_clk[S32GEN1_SCMI_CLK_RTC_FIRC] =
		get_plat_clk(S32GEN1_CLK_FIRC);
	scmi_clk[S32GEN1_SCMI_CLK_SIUL2_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV6);
	scmi_clk[S32GEN1_SCMI_CLK_SIUL2_FILTER] =
		get_plat_clk(S32GEN1_CLK_FIRC);
	scmi_clk[S32GEN1_SCMI_CLK_CRC_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_CRC_MODULE] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_EIM0_REG] =
		get_plat_clk(S32GEN1_CLK_A53_DIV10);
	scmi_clk[S32GEN1_SCMI_CLK_EIM0_MODULE] =
		get_plat_clk(S32GEN1_CLK_A53_DIV10);
	scmi_clk[S32GEN1_SCMI_CLK_EIM123_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV6);
	scmi_clk[S32GEN1_SCMI_CLK_EIM123_MODULE] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV6);
	scmi_clk[S32GEN1_SCMI_CLK_EIM_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV6);
	scmi_clk[S32GEN1_SCMI_CLK_EIM_MODULE] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV6);
	scmi_clk[S32GEN1_SCMI_CLK_FCCU_MODULE] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV6);
	scmi_clk[S32GEN1_SCMI_CLK_FCCU_SAFE] =
		get_plat_clk(S32GEN1_CLK_FIRC);
	scmi_clk[S32GEN1_SCMI_CLK_SWT_MODULE] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_SWT_COUNTER] =
		get_plat_clk(S32GEN1_CLK_FIRC);
	scmi_clk[S32GEN1_SCMI_CLK_STM_MODULE] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_STM_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_PIT_MODULE] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_PIT_REG] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV3);
	scmi_clk[S32GEN1_SCMI_CLK_EDMA_MODULE] =
		get_plat_clk(S32GEN1_CLK_XBAR);
	scmi_clk[S32GEN1_SCMI_CLK_EDMA_AHB] =
		get_plat_clk(S32GEN1_CLK_XBAR);


	plat_clks.scmi_clks.clks = scmi_clk;
	plat_clks.scmi_clks.clk_num = ARRAY_SIZE(scmi_clk);
}

static void s32g274a_scmi_clocks_init(void)
{
	/* LLCE */
	scmi_clk[S32G274A_SCMI_CLK_LLCE_SYS] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV2);
	scmi_clk[S32G274A_SCMI_CLK_LLCE_CAN_PE] =
		get_plat_clk(S32GEN1_CLK_CAN);

	scmi_clk[S32G274A_SCMI_CLK_USB_MEM] =
		get_plat_clk(S32GEN1_CLK_XBAR_DIV4);
	scmi_clk[S32G274A_SCMI_CLK_USB_LOW] =
		get_plat_clk(S32GEN1_CLK_SIRC);
	scmi_clk[S32G274A_SCMI_CLK_PFE_AXI] =
		get_plat_clk(S32GEN1_CLK_PFE_SYS);
	scmi_clk[S32G274A_SCMI_CLK_PFE_APB] =
		get_plat_clk(S32GEN1_CLK_PFE_SYS);
	scmi_clk[S32G274A_SCMI_CLK_PFE_PE] =
		get_plat_clk(S32GEN1_CLK_PFE_PE);

	/* PFE 0 */
	scmi_clk[S32G274A_SCMI_CLK_PFE0_RX_SGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_0_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE0_TX_SGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_0_TX);
	scmi_clk[S32G274A_SCMI_CLK_PFE0_RX_RGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_0_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE0_TX_RGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_0_TX);
	scmi_clk[S32G274A_SCMI_CLK_PFE0_RX_RMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_0_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE0_TX_RMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_0_TX);
	scmi_clk[S32G274A_SCMI_CLK_PFE0_RX_MII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_0_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE0_TX_MII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_0_TX);
	/* PFE 1 */
	scmi_clk[S32G274A_SCMI_CLK_PFE1_RX_SGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_1_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE1_TX_SGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_1_TX);
	scmi_clk[S32G274A_SCMI_CLK_PFE1_RX_RGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_1_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE1_TX_RGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_1_TX);
	scmi_clk[S32G274A_SCMI_CLK_PFE1_RX_RMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_1_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE1_TX_RMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_1_TX);
	scmi_clk[S32G274A_SCMI_CLK_PFE1_RX_MII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_1_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE1_TX_MII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_1_TX);
	/* PFE 2 */
	scmi_clk[S32G274A_SCMI_CLK_PFE2_RX_SGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_2_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE2_TX_SGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_2_TX);
	scmi_clk[S32G274A_SCMI_CLK_PFE2_RX_RGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_2_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE2_TX_RGMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_2_TX);
	scmi_clk[S32G274A_SCMI_CLK_PFE2_RX_RMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_2_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE2_TX_RMII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_2_TX);
	scmi_clk[S32G274A_SCMI_CLK_PFE2_RX_MII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_2_RX);
	scmi_clk[S32G274A_SCMI_CLK_PFE2_TX_MII] =
		get_plat_clk(S32GEN1_CLK_PFE_EMAC_2_TX);
}

static void s32r45_scmi_clocks_init(void)
{
	/* LAX */
	scmi_clk[S32R45_SCMI_CLK_LAX_MODULE] =
		get_plat_clk(S32GEN1_CLK_ACCEL_4);

	/* SPT */
	scmi_clk[S32R45_SCMI_CLK_SPT_SPT] =
		get_plat_clk(S32GEN1_CLK_ACCEL_3);
	scmi_clk[S32R45_SCMI_CLK_SPT_AXI] =
		get_plat_clk(S32GEN1_CLK_ACCEL_3);
	scmi_clk[S32R45_SCMI_CLK_SPT_MODULE] =
		get_plat_clk(S32GEN1_CLK_ACCEL_3_DIV3);

	/* GMAC1 */
	scmi_clk[S32R45_SCMI_CLK_GMAC1_TX_SGMII] =
		get_plat_clk(S32GEN1_CLK_GMAC_1_TX);
	scmi_clk[S32R45_SCMI_CLK_GMAC1_TX_RGMII] =
		get_plat_clk(S32GEN1_CLK_GMAC_1_TX);
	scmi_clk[S32R45_SCMI_CLK_GMAC1_TX_RMII] =
		get_plat_clk(S32GEN1_CLK_GMAC_1_TX);
	scmi_clk[S32R45_SCMI_CLK_GMAC1_TX_MII] =
		get_plat_clk(S32GEN1_CLK_GMAC_1_TX);
	scmi_clk[S32R45_SCMI_CLK_GMAC1_AXI] =
		get_plat_clk(S32GEN1_CLK_XBAR);
}

#ifdef CONFIG_PM_DEBUG
static int s32gen1_clk_suspend(void)
{
	size_t i;
	unsigned int en_count;

	for (i = 0; i < ARRAY_SIZE(clk); i++) {
		en_count = __clk_get_enable_count(clk[i]);
		if (!en_count)
			continue;

		pr_warn("The clock '%s' (refcount = %d) wasn't disabled before suspend.",
			__clk_get_name(clk[i]), en_count);
	}

	return 0;
}
#else
static int s32gen1_clk_suspend(void)
{
	return 0;
}
#endif

static void s32gen1_clk_resume(void)
{
}

static struct syscore_ops s32gen1_clk_syscore_ops = {
	.suspend = s32gen1_clk_suspend,
	.resume = s32gen1_clk_resume,
};

static void __init s32g274_clocks_init(struct device_node *clks_node)
{
	s32gen1_clocks_init(clks_node);
	s32g274_extra_clocks_init(clks_node);
	s32gen1_scmi_clocks_init();
	s32g274a_scmi_clocks_init();
	register_syscore_ops(&s32gen1_clk_syscore_ops);
}

static void __init s32r45_clocks_init(struct device_node *clks_node)
{
	s32gen1_clocks_init(clks_node);
	s32r45_extra_clocks_init(clks_node);
	s32gen1_scmi_clocks_init();
	s32r45_scmi_clocks_init();
	register_syscore_ops(&s32gen1_clk_syscore_ops);
}

CLK_OF_DECLARE(S32G274, "fsl,s32g274-clocking", s32g274_clocks_init);
CLK_OF_DECLARE(S32R45, "fsl,s32r45-clocking", s32r45_clocks_init);
