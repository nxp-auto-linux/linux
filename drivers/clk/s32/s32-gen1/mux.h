/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2020 NXP
 */
#ifndef S32GEN1_MUX_H
#define S32GEN1_MUX_H

#include <linux/clk-provider.h>

struct s32gen1_clk_mux {
	struct clk_mux mux;
	const struct clk_ops *ops;
};

static inline struct s32gen1_clk_mux *to_s32gen1_clk_mux(struct clk_hw *hw)
{
	struct clk_mux *mux = to_clk_mux(hw);

	return container_of(mux, struct s32gen1_clk_mux, mux);
}

void init_s32gen1_clk_mux(struct s32gen1_clk_mux *s32mux,
			  void __iomem *reg, u8 shift, u8 width,
			  u32 *table, spinlock_t *lock,
			  struct clk_init_data *init);

/* S32GEN1 multiplexer ops */
u8 s32gen1_clk_mux_get_parent(struct clk_hw *hw);
int s32gen1_clk_mux_set_parent(struct clk_hw *hw, u8 index);
int s32gen1_clk_mux_determine_rate(struct clk_hw *hw,
				   struct clk_rate_request *req);

#endif
