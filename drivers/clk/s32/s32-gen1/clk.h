/*
 * Copyright 2018,2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MACH_S32GEN1_CLK_H
#define __MACH_S32GEN1_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>
#include "dfs.h"
#include "pll.h"
#include "../clk.h"

struct clk *s32gen1_clk_plldig(enum s32gen1_plldig_type type, const char *name,
			   const char *parent_name, struct clk *pll_mux,
			   void __iomem *base, u32 pllodiv[PHI_MAXNUMBER],
			   u32 phi_nr);

struct clk *s32gen1_clk_plldig_phi(enum s32gen1_plldig_type type,
			       const char *name, const char *parent,
			       void __iomem *base, u32 phi);

struct clk *s32gen1_clk_dfs_out(enum s32gen1_plldig_type type, const char *name,
			const char *parent_name, void __iomem *dfs_base,
			u8 idx);

struct clk *s32gen1_fxosc(const char *compatible);

struct clk *s32gen1_clk_pll_mux(const char *name, void __iomem *reg, u8 shift,
				u8 width, const char **parents, int num_parents,
				u32 *table, spinlock_t *lock);

struct clk *s32gen1_clk_cgm_mux(const char *name, void __iomem *cgm_addr,
				u32 index, const char **parents,
				int num_parents, u32 *table, spinlock_t *lock);

struct clk *s32gen1_clk_cgm_div(const char *name, const char *parent,
				void __iomem *cgm_addr, u32 index,
				spinlock_t *lock);

#endif
