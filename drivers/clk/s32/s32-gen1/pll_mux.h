/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2020 NXP
 */
#ifndef S32GEN1_PLL_MUX
#define S32GEN1_PLL_MUX

#include "mux.h"

struct s32gen1_pll_mux {
	struct s32gen1_clk_mux s32mux;
	u32 source;
};

static inline struct s32gen1_pll_mux *to_s32gen1_clk_pll_mux(struct clk_hw *hw)
{
	struct s32gen1_clk_mux *s32mux = to_s32gen1_clk_mux(hw);

	return container_of(s32mux, struct s32gen1_pll_mux, s32mux);
}

#endif
