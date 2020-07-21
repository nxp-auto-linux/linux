// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include "clk.h"
#include "mc_cgm.h"

struct clk_cgm_div {
	struct clk_divider div;
	const struct clk_ops *ops;
	void __iomem *cgm_addr;
	u32 index;
	u32 div_factor;
};

static inline struct clk_cgm_div *to_clk_cgm_div(struct clk_hw *hw)
{
	struct clk_divider *div = to_clk_divider(hw);

	return container_of(div, struct clk_cgm_div, div);
}

static unsigned long clk_cgm_div_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_cgm_div *cgm_div = to_clk_cgm_div(hw);

	return cgm_div->ops->recalc_rate(&cgm_div->div.hw, parent_rate);
}

static long clk_cgm_div_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct clk_cgm_div *cgm_div = to_clk_cgm_div(hw);

	return cgm_div->ops->round_rate(&cgm_div->div.hw, rate, prate);
}

static int clk_cgm_div_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_cgm_div *cgm_div = to_clk_cgm_div(hw);

	return cgm_div->ops->set_rate(&cgm_div->div.hw, rate, parent_rate);
}

static int clk_cgm_div_is_enabled(struct clk_hw *hw)
{
	struct clk_cgm_div *cgm_div = to_clk_cgm_div(hw);
	u32 dc_val = readl_relaxed(CGM_MUXn_DC(cgm_div->cgm_addr,
					       cgm_div->index));

	if (MC_CGM_MUX_DCn_DIV_VAL(dc_val) == cgm_div->div_factor &&
	    dc_val & MC_CGM_MUX_DCn_DE)
		return 1;

	return 0;
}

static int clk_cgm_div_enable(struct clk_hw *hw)
{
	struct clk_cgm_div *cgm_div = to_clk_cgm_div(hw);
	void __iomem *cgm_addr = cgm_div->cgm_addr;
	u32 index = cgm_div->index;
	u32 updstat;

	if (clk_cgm_div_is_enabled(hw))
		return 0;

	/* Set the divider */
	writel_relaxed(MC_CGM_MUX_DCn_DE |
		       MC_CGM_MUX_DCn_DIV(cgm_div->div_factor),
		       CGM_MUXn_DC(cgm_addr, index));

	/* Wait for divider gets updated */
	do {
		updstat = readl_relaxed(CGM_MUXn_DIV_UPD_STAT(cgm_addr, index));
	} while (MC_CGM_MUXn_DIV_UPD_STAT_DIVSTAT(updstat));

	return 0;
}

const struct clk_ops clk_cgm_div_ops = {
	.recalc_rate = clk_cgm_div_recalc_rate,
	.round_rate = clk_cgm_div_round_rate,
	.set_rate = clk_cgm_div_set_rate,
	.enable = clk_cgm_div_enable,
	.is_enabled = clk_cgm_div_is_enabled,
};

static u32 get_cgm_div_factor(void __iomem *cgm_addr, u32 index)
{
	u32 dc_val = readl_relaxed(CGM_MUXn_DC(cgm_addr, index));

	return MC_CGM_MUX_DCn_DIV_VAL(dc_val);
}

struct clk *s32gen1_clk_cgm_div(const char *name, const char *parent,
				void __iomem *cgm_addr, u32 index,
				spinlock_t *lock)
{
	struct clk_cgm_div *cgm_div;
	struct clk *clk;
	struct clk_init_data init;
	struct clk_divider *div;

	cgm_div = kzalloc(sizeof(*cgm_div), GFP_KERNEL);
	if (!cgm_div)
		return ERR_PTR(-ENOMEM);

	cgm_div->ops = &clk_divider_ops;
	cgm_div->cgm_addr = cgm_addr;
	cgm_div->index = index;
	cgm_div->div_factor = get_cgm_div_factor(cgm_addr, index);

	init.name = name;
	init.ops = &clk_cgm_div_ops;
	init.flags = CLK_SET_RATE_PARENT | CLK_IS_BASIC;
	init.parent_names = &parent;
	init.num_parents =  1;

	div = &cgm_div->div;

	/* struct clk_divider assignments */
	div->reg = CGM_MUXn_DC(cgm_addr, index);
	div->shift = MC_CGM_MUX_DCn_DIV_OFFSET;
	div->width = MC_CGM_MUX_DCn_DIV_SIZE;
	div->flags = 0;
	div->lock = lock;
	div->hw.init = &init;
	div->table = NULL;

	clk = clk_register(NULL, &cgm_div->div.hw);
	if (IS_ERR(clk))
		kfree(cgm_div);

	return clk;
}


