/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/err.h>
#include "clk.h"

/**
 * struct clk_dfs - S32 DFS clock
 * @clk_hw:	clock source
 * @reg:	DFS register address
 * @idx:	the index of DFS encoded in the register
 *
 * DFS clock found on S32 series. Each register for DFS has 4 clk_dfs
 * data encoded, and member idx is used to specify the one.
 * Only ARMPLL(3 DFS), ENETPLL(4 DFS) and DDRPLL(3 DFS) has DFS outputs.
 */
struct clk_dfs {
	struct clk_hw	hw;
	void __iomem	*reg;
	enum s32gen1_plldig_type plltype;
	u8		idx;
};

#define to_clk_dfs(_hw) container_of(_hw, struct clk_dfs, hw)

static int get_pllx_dfs_nr(enum s32gen1_plldig_type plltype)
{
	switch (plltype) {
	case S32GEN1_PLLDIG_ARM:
		return ARMPLL_DFS_NR;
	case S32GEN1_PLLDIG_PERIPH:
		return PERIPHPLL_DFS_NR;
	case S32GEN1_PLLDIG_DDR:
	case S32GEN1_PLLDIG_ACCEL:
	case S32GEN1_PLLDIG_AURORA:
		pr_warn("Current selected PLL has no DFS\n");
		break;
	}

	return -EINVAL;
}
static unsigned long get_pllx_dfsy_max_rate(enum s32gen1_plldig_type plltype,
					    int dfsno)
{
	switch (plltype) {
	case S32GEN1_PLLDIG_ARM:
		switch (dfsno) {
		case 1:
			return ARMPLL_DFS1_MAX_RATE;
		case 2:
			return ARMPLL_DFS2_MAX_RATE;
		case 3:
			return ARMPLL_DFS3_MAX_RATE;
		case 4:
			return ARMPLL_DFS4_MAX_RATE;
		case 5:
			return ARMPLL_DFS5_MAX_RATE;
		case 6:
			return ARMPLL_DFS6_MAX_RATE;
		}
		break;
	case S32GEN1_PLLDIG_PERIPH:
		switch (dfsno) {
		case 1:
			return PERIPHPLL_DFS1_MAX_RATE;
		case 2:
			return PERIPHPLL_DFS2_MAX_RATE;
		case 3:
			return PERIPHPLL_DFS3_MAX_RATE;
		case 4:
			return PERIPHPLL_DFS4_MAX_RATE;
		case 5:
			return PERIPHPLL_DFS5_MAX_RATE;
		case 6:
			return PERIPHPLL_DFS6_MAX_RATE;
		}
		break;
	case S32GEN1_PLLDIG_DDR:
	case S32GEN1_PLLDIG_ACCEL:
	case S32GEN1_PLLDIG_AURORA:
		pr_warn("Current selected PLL has no DFS.");
		break;
	default:
		pr_warn("Unsupported PLL. Use %d or %d\n",
			S32GEN1_PLLDIG_ARM,	S32GEN1_PLLDIG_AURORA);
		break;
	}

	return -EINVAL;
}
static int clk_dfs_enable(struct clk_hw *hw)
{
	/*
	 * TODO: When SOC is available, this function
	 * should be tested and implemented for DFS
	 * if it is possible
	 */
	return 0;
}

static void clk_dfs_disable(struct clk_hw *hw)
{
	/*
	 * TODO: When SOC is available, this function
	 * should be tested and implemented for DFS
	 * if it is possible
	 */
}

static unsigned long clk_dfs_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_dfs *dfs = to_clk_dfs(hw);
	u32 mfn, mfi, rate;
	u32 dvport = readl_relaxed(DFS_DVPORTn(dfs->reg, dfs->idx));

	mfn = (dvport & DFS_DVPORTn_MFN_MASK) >> DFS_DVPORTn_MFN_OFFSET;
	mfi = (dvport & DFS_DVPORTn_MFI_MASK) >> DFS_DVPORTn_MFI_OFFSET;
	rate = (18 * parent_rate) / (36 * mfi + mfn);

	return rate;
}

static long clk_dfs_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	struct clk_dfs *dfs = to_clk_dfs(hw);
	unsigned long max_allowed_rate;

	max_allowed_rate = get_pllx_dfsy_max_rate(dfs->plltype, dfs->idx);

	if (rate > max_allowed_rate)
		rate = max_allowed_rate;

	return rate;
}

static int clk_dfs_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clk_dfs *dfs = to_clk_dfs(hw);
	u32 mfi, mfn;
	u32 portreset = readl_relaxed(DFS_PORTRESET(dfs->reg));

	writel_relaxed(portreset | DFS_PORTRESET_PORTRESET_SET(dfs->idx),
			DFS_PORTRESET(dfs->reg));
	writel_relaxed(DFS_CTL_RESET, DFS_CTL(dfs->reg));

	mfi = parent_rate / (2 * rate);
	mfn = (parent_rate % (2 * rate)) * 36;
	writel_relaxed(DFS_DVPORTn_MFI_SET(mfi) |
		       DFS_DVPORTn_MFN_SET(mfn),
		       DFS_DVPORTn(dfs->reg, dfs->idx));

	/* DFS clk enable programming */
	writel_relaxed(~DFS_CTL_RESET, DFS_CTL(dfs->reg));
	portreset = readl_relaxed(DFS_PORTRESET(dfs->reg));
	writel(portreset & ~DFS_PORTRESET_PORTRESET_SET(dfs->idx),
			DFS_PORTRESET(dfs->reg));

	while ((readl_relaxed(DFS_PORTSR(dfs->reg)) & (1 << (dfs->idx)))
			!= (1 << (dfs->idx)))
		;

	return 0;
}

static int clk_dfs_is_enabled(struct clk_hw *hw)
{
	struct clk_dfs *dfs = to_clk_dfs(hw);

	/* Check if current DFS output port is locked */
	if (readl_relaxed(DFS_PORTSR(dfs->reg)) & (1 << (dfs->idx)))
		return 0;

	return 1;
}

static const struct clk_ops clk_dfs_ops = {
	.enable		= clk_dfs_enable,
	.disable	= clk_dfs_disable,
	.recalc_rate	= clk_dfs_recalc_rate,
	.round_rate	= clk_dfs_round_rate,
	.set_rate	= clk_dfs_set_rate,
	.is_enabled	= clk_dfs_is_enabled,
};


struct clk *s32gen1_clk_dfs(enum s32gen1_plldig_type type, const char *name,
			const char *parent_name, void __iomem *reg,
			u8 idx)
{
	struct clk_dfs *dfs;
	struct clk *clk;
	struct clk_init_data init;

	/* Only ARM and PERIPH PLL have DFS */
	if (type != S32GEN1_PLLDIG_ARM && type != S32GEN1_PLLDIG_PERIPH)
		return ERR_PTR(-EINVAL);

	/* check if DFS index is valid for current pll */
	if (idx <= 0 || idx > get_pllx_dfs_nr(type))
		return ERR_PTR(-EINVAL);

	dfs = kzalloc(sizeof(*dfs), GFP_KERNEL);
	if (!dfs)
		return ERR_PTR(-ENOMEM);

	dfs->reg = reg;
	/* Even if DFS name starts with DFS1, the index used in registers starts
	 * from 0.
	 */
	dfs->idx = idx - 1;
	dfs->plltype = type;

	init.name = name;
	init.ops = &clk_dfs_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	dfs->hw.init = &init;

	clk = clk_register(NULL, &dfs->hw);
	if (IS_ERR(clk))
		kfree(dfs);

	return clk;
}
