// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */
#include "pll_mux.h"

u8 s32gen1_clk_mux_get_parent(struct clk_hw *hw)
{
	struct s32gen1_clk_mux *mux = to_s32gen1_clk_mux(hw);

	return mux->ops->get_parent(&mux->mux.hw);
}

int s32gen1_clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct s32gen1_clk_mux *mux = to_s32gen1_clk_mux(hw);

	return mux->ops->set_parent(&mux->mux.hw, index);
}

int s32gen1_clk_mux_determine_rate(struct clk_hw *hw,
				   struct clk_rate_request *req)
{
	struct s32gen1_clk_mux *mux = to_s32gen1_clk_mux(hw);

	return mux->ops->determine_rate(&mux->mux.hw, req);
}

static const struct clk_ops s32gen1_clk_mux_ops = {
	.get_parent = s32gen1_clk_mux_get_parent,
	.set_parent = s32gen1_clk_mux_set_parent,
	.determine_rate = s32gen1_clk_mux_determine_rate,
};

void init_s32gen1_clk_mux(struct s32gen1_clk_mux *s32mux,
			  void __iomem *reg, u8 shift, u8 width,
			  u32 *table, spinlock_t *lock,
			  struct clk_init_data *init)
{
	struct clk_mux *mux;

	s32mux->ops = &clk_mux_ops;

	if (!init->ops)
		init->ops = &s32gen1_clk_mux_ops;

	init->flags = CLK_SET_RATE_NO_REPARENT;

	mux = &s32mux->mux;

	/* struct clk_mux assignments */
	mux->reg = reg;
	mux->shift = shift;
	mux->mask = BIT(width) - 1;
	mux->flags = 0;
	mux->lock = lock;
	mux->table = table;
	mux->hw.init = init;
}

