// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */
#include <linux/slab.h>
#include "pll_mux.h"

struct clk *s32gen1_clk_pll_mux(const char *name, void __iomem *reg, u8 shift,
				u8 width, const char **parents, int num_parents,
				u32 *table, spinlock_t *lock)
{
	struct clk *clk;
	struct s32gen1_pll_mux *pll_mux;
	struct clk_init_data init;

	pll_mux = kzalloc(sizeof(*pll_mux), GFP_KERNEL);
	if (!pll_mux)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.parent_names = parents;
	init.num_parents = num_parents;
	init.ops = NULL;

	init_s32gen1_clk_mux(&pll_mux->s32mux, reg, shift, width, table,
			     lock, &init);

	pll_mux->source = readl(reg);

	clk = clk_register(NULL, &pll_mux->s32mux.mux.hw);
	if (IS_ERR(clk))
		kfree(pll_mux);

	return clk;
}

