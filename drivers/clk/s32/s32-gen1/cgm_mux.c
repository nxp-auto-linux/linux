// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/slab.h>

#include "mc_cgm.h"
#include "mux.h"

struct clk_cgm_mux {
	struct s32gen1_clk_mux s32mux;
	void __iomem *cgm_addr;
	u32 index;
	u32 source;
};

static inline struct clk_cgm_mux *to_clk_cgm_mux(struct clk_hw *hw)
{
	struct s32gen1_clk_mux *s32mux = to_s32gen1_clk_mux(hw);

	return container_of(s32mux, struct clk_cgm_mux, s32mux);
}


static int clk_cgm_mux_is_enabled(struct clk_hw *hw)
{
	struct clk_cgm_mux *mux = to_clk_cgm_mux(hw);
	void __iomem *cgm_addr = mux->cgm_addr;
	u32 index = mux->index;
	u32 css;

	css = readl(CGM_MUXn_CSS(cgm_addr, index));

	if (MC_CGM_MUXn_CSS_SELSTAT(css) == mux->source &&
	    MC_CGM_MUXn_CSS_SWTRG(css) == MC_CGM_MUXn_CSS_SWTRG_SUCCESS &&
	    !(css & MC_CGM_MUXn_CSS_SWIP))
		return 1;

	return 0;
}

static int clk_cgm_mux_enable(struct clk_hw *hw)
{
	struct clk_cgm_mux *mux = to_clk_cgm_mux(hw);
	void __iomem *cgm_addr = mux->cgm_addr;
	u32 index = mux->index;
	u32 css, csc;

	if (clk_cgm_mux_is_enabled(hw))
		return 0;

	css = readl(CGM_MUXn_CSS(cgm_addr, index));

	/* Ongoing clock switch? */
	while (readl(CGM_MUXn_CSS(cgm_addr, index)) & MC_CGM_MUXn_CSS_SWIP)
		;

	csc = readl(CGM_MUXn_CSC(cgm_addr, index));

	/* Clear previous source. */
	csc &= ~(MC_CGM_MUXn_CSC_SELCTL_MASK);

	/* Select the clock source and trigger the clock switch. */
	writel(csc | MC_CGM_MUXn_CSC_SELCTL(mux->source) |
	       MC_CGM_MUXn_CSC_CLK_SW,
	       CGM_MUXn_CSC(cgm_addr, index));

	/* Wait for configuration bit to auto-clear. */
	while (readl(CGM_MUXn_CSC(cgm_addr, index)) & MC_CGM_MUXn_CSC_CLK_SW)
		;

	/* Is the clock switch completed? */
	while (readl(CGM_MUXn_CSS(cgm_addr, index)) & MC_CGM_MUXn_CSS_SWIP)
		;

	/*
	 * Check if the switch succeeded.
	 * Check switch trigger cause and the source.
	 */
	css = readl(CGM_MUXn_CSS(cgm_addr, index));
	if ((MC_CGM_MUXn_CSS_SWTRG(css) == MC_CGM_MUXn_CSS_SWTRG_SUCCESS) &&
	    (MC_CGM_MUXn_CSS_SELSTAT(css) == mux->source))
		return 0;

	pr_err("Failed to change the clock source of mux %d to %d (CGM = %p)\n",
	       index, mux->source, cgm_addr);

	return -EINVAL;
}

static u32 get_cgm_mux_source(void __iomem *cgm_addr, u32 index)
{
	u32 css = readl(CGM_MUXn_CSS(cgm_addr, index));

	return MC_CGM_MUXn_CSS_SELSTAT(css);
}

static const struct clk_ops clk_cgm_mux_ops = {
	.get_parent = s32gen1_clk_mux_get_parent,
	.set_parent = s32gen1_clk_mux_set_parent,
	.determine_rate = s32gen1_clk_mux_determine_rate,
	.enable = clk_cgm_mux_enable,
	.is_enabled = clk_cgm_mux_is_enabled,
};

struct clk *s32gen1_clk_cgm_mux(const char *name, void __iomem *cgm_addr,
				u32 index, const char **parents,
				int num_parents, u32 *table, spinlock_t *lock)
{
	struct clk *clk;
	struct clk_cgm_mux *cgm_mux;
	struct clk_init_data init;

	cgm_mux = kzalloc(sizeof(*cgm_mux), GFP_KERNEL);
	if (!cgm_mux)
		return ERR_PTR(-ENOMEM);


	cgm_mux->cgm_addr = cgm_addr;
	cgm_mux->index = index;
	cgm_mux->source = get_cgm_mux_source(cgm_addr, index);

	init.name = name;
	init.parent_names = parents;
	init.num_parents = num_parents;
	init.ops = &clk_cgm_mux_ops;

	init_s32gen1_clk_mux(&cgm_mux->s32mux, CGM_MUXn_CSC(cgm_addr, index),
			     MC_CGM_MUXn_CSC_SELCTL_OFFSET,
			     MC_CGM_MUXn_CSC_SELCTL_SIZE,
			     table, lock, &init);

	clk = clk_register(NULL, &cgm_mux->s32mux.mux.hw);
	if (IS_ERR(clk))
		kfree(cgm_mux);

	return clk;

}
