/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017,2020 NXP
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
 * struct clk_dfs_out - S32-GEN1 DFS port clock
 * @clk_hw:	clock source
 * @dfs_base:	DFS register address
 * @idx:	the index of DFS encoded in the register
 * @mfi:	integer part of division factor of the port
 * @mfn:	fractional part of division factor of the port
 *
 * DFS port clock found on S32-GEN1 series.
 */
struct clk_dfs_out {
	struct clk_hw	hw;
	void __iomem	*dfs_base;
	enum s32gen1_plldig_type plltype;
	u8		idx;
	u32		mfi;
	u32		mfn;
};

#define to_clk_dfs_out(_hw) container_of(_hw, struct clk_dfs_out, hw)

static int get_pllx_dfs_nr(enum s32gen1_plldig_type plltype)
{
	switch (plltype) {
	case S32GEN1_PLLDIG_ARM:
		return ARMPLL_DFS_NR;
	case S32GEN1_PLLDIG_PERIPH:
		return PERIPHPLL_DFS_NR;
	case S32GEN1_PLLDIG_DDR:
	case S32GEN1_PLLDIG_ACCEL:
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
		pr_warn("Current selected PLL has no DFS.");
		break;
	default:
		pr_warn("Unsupported PLL. Use %d or %d\n",
			S32GEN1_PLLDIG_ARM, S32GEN1_PLLDIG_ACCEL);
		break;
	}

	return -EINVAL;
}

static void read_mfi_mfn(void *dfs_addr, u32 port, u32 *mfi, u32 *mfn)
{
	u32 dvport = readl_relaxed(DFS_DVPORTn(dfs_addr, port));

	*mfi = DFS_DVPORTn_MFI(dvport);
	*mfn = DFS_DVPORTn_MFN(dvport);
}

static int clk_dfs_out_is_enabled(struct clk_hw *hw)
{
	struct clk_dfs_out *dfs = to_clk_dfs_out(hw);
	void __iomem *base = dfs->dfs_base;
	u32 portsr, portolsr, mfi, mfn;

	portsr = readl_relaxed(DFS_PORTSR(base));
	portolsr = readl_relaxed(DFS_PORTOLSR(base));

	/* Check if current DFS output port is locked */
	if (!(portsr & BIT(dfs->idx)))
		return 0;

	if (portolsr & BIT(dfs->idx))
		return 0;

	read_mfi_mfn(dfs->dfs_base, dfs->idx, &mfi, &mfn);
	if (mfi != dfs->mfi || mfn != dfs->mfn)
		return 0;

	return 1;
}

static int clk_dfs_out_enable(struct clk_hw *hw)
{
	struct clk_dfs_out *dfs = to_clk_dfs_out(hw);
	void __iomem *base = dfs->dfs_base;
	bool init_dfs;
	u32 mask, portreset, portsr, portolsr;

	if (clk_dfs_out_is_enabled(hw))
		return 0;

	portsr = readl(DFS_PORTSR(base));
	init_dfs = (!portsr);

	if (init_dfs)
		mask = DFS_PORTRESET_PORTRESET_MAXVAL;
	else
		mask = DFS_PORTRESET_PORTRESET_SET(BIT(dfs->idx));

	writel(mask, DFS_PORTOLSR(base));
	writel(mask, DFS_PORTRESET(base));

	while (readl(DFS_PORTSR(base)) & mask)
		;

	if (init_dfs)
		writel(DFS_CTL_RESET, DFS_CTL(base));

	writel(DFS_DVPORTn_MFI_SET(dfs->mfi) | DFS_DVPORTn_MFN_SET(dfs->mfn),
	       DFS_DVPORTn(base, dfs->idx));

	if (init_dfs)
		/* DFS clk enable programming */
		writel(~DFS_CTL_RESET, DFS_CTL(base));

	portreset = readl(DFS_PORTRESET(base));
	portreset &= ~BIT(dfs->idx);
	writel(portreset, DFS_PORTRESET(base));

	while ((readl(DFS_PORTSR(base)) & BIT(dfs->idx)) != BIT(dfs->idx))
		;

	portolsr = readl(DFS_PORTOLSR(base));
	if (portolsr & DFS_PORTOLSR_LOL(dfs->idx)) {
		pr_err("Failed to lock DFS divider\n");
		return -EINVAL;
	}

	return 0;
}

static void clk_dfs_out_disable(struct clk_hw *hw)
{
	/*
	 * TODO: When SOC is available, this function
	 * should be tested and implemented for DFS
	 * if it is possible
	 */
}

static unsigned long clk_dfs_out_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_dfs_out *dfs = to_clk_dfs_out(hw);
	u32 mfn, mfi, rate;
	u32 dvport = readl_relaxed(DFS_DVPORTn(dfs->dfs_base, dfs->idx));

	mfn = (dvport & DFS_DVPORTn_MFN_MASK) >> DFS_DVPORTn_MFN_OFFSET;
	mfi = (dvport & DFS_DVPORTn_MFI_MASK) >> DFS_DVPORTn_MFI_OFFSET;
	rate = (18 * parent_rate) / (36 * mfi + mfn);

	return rate;
}

static long clk_dfs_out_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	struct clk_dfs_out *dfs = to_clk_dfs_out(hw);
	unsigned long max_allowed_rate;

	max_allowed_rate = get_pllx_dfsy_max_rate(dfs->plltype, dfs->idx);

	if (rate > max_allowed_rate)
		rate = max_allowed_rate;

	return rate;
}

static int clk_dfs_out_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clk_dfs_out *dfs = to_clk_dfs_out(hw);
	u32 mfi, mfn;
	u32 portreset = readl_relaxed(DFS_PORTRESET(dfs->dfs_base));

	writel_relaxed(portreset | DFS_PORTRESET_PORTRESET_SET(dfs->idx),
			DFS_PORTRESET(dfs->dfs_base));
	writel_relaxed(DFS_CTL_RESET, DFS_CTL(dfs->dfs_base));

	mfi = parent_rate / (2 * rate);
	mfn = (parent_rate % (2 * rate)) * 36;
	writel_relaxed(DFS_DVPORTn_MFI_SET(mfi) |
		       DFS_DVPORTn_MFN_SET(mfn),
		       DFS_DVPORTn(dfs->dfs_base, dfs->idx));

	/* DFS clk enable programming */
	writel_relaxed(~DFS_CTL_RESET, DFS_CTL(dfs->dfs_base));
	portreset = readl_relaxed(DFS_PORTRESET(dfs->dfs_base));
	writel(portreset & ~DFS_PORTRESET_PORTRESET_SET(dfs->idx),
			DFS_PORTRESET(dfs->dfs_base));

	while ((readl_relaxed(DFS_PORTSR(dfs->dfs_base)) & (1 << (dfs->idx)))
			!= (1 << (dfs->idx)))
		;

	return 0;
}

static const struct clk_ops clk_dfs_out_ops = {
	.enable		= clk_dfs_out_enable,
	.disable	= clk_dfs_out_disable,
	.recalc_rate	= clk_dfs_out_recalc_rate,
	.round_rate	= clk_dfs_out_round_rate,
	.set_rate	= clk_dfs_out_set_rate,
	.is_enabled	= clk_dfs_out_is_enabled,
};

struct clk *s32gen1_clk_dfs_out(enum s32gen1_plldig_type type, const char *name,
			const char *parent_name, void __iomem *dfs_base,
			u8 idx)
{
	struct clk_dfs_out *dfs;
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

	dfs->dfs_base = dfs_base;

	/* Even if DFS name starts with DFS1, the index used in registers starts
	 * from 0.
	 */
	dfs->idx = idx - 1;
	dfs->plltype = type;

	read_mfi_mfn(dfs_base, dfs->idx, &dfs->mfi, &dfs->mfn);

	init.name = name;
	init.ops = &clk_dfs_out_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	dfs->hw.init = &init;

	clk = clk_register(NULL, &dfs->hw);
	if (IS_ERR(clk))
		kfree(dfs);

	return clk;
}
