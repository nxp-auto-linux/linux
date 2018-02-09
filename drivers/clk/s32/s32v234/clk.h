/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __MACH_S32V234_CLK_H
#define __MACH_S32V234_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>
#include "dfs.h"
#include "mc_cgm.h"
#include "mc_me.h"
#include "pll.h"
#include "src.h"
#include "../clk.h"

struct clk *s32v234_clk_plldig(enum s32v234_plldig_type type, const char *name,
			   const char *parent_name, void __iomem *base,
			   u32 plldv_mfd, u32 plldv_mfn,
			   u32 plldv_rfdphi, u32 plldv_rfdphi1);

struct clk *s32v234_clk_plldig_phi(enum s32v234_plldig_type type,
				const char *name, const char *parent,
				void __iomem *base, u32 phi);

struct clk *s32v234_clk_dfs(enum s32v234_plldig_type type, const char *name,
			const char *parent_name,
			void __iomem *reg, u8 idx, u32 mfn);

struct clk *s32v234_clk_register_gate2(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u32 pctln, u8 bit_idx, u8 val,
		u8 clk_gate_flags, spinlock_t *lock,
		unsigned int *share_count);

static inline struct clk *s32v234_clk_gate2(const char *name,
		const char *parent, void __iomem *reg, u32 pctln,
		u8 shift, u8 val, spinlock_t *lock)
{
	return s32v234_clk_register_gate2(NULL, name, parent,
		CLK_SET_RATE_PARENT, reg, pctln, shift, val, 0,
		lock, NULL);
}

static inline struct clk *s32v234_clk_gate2_shared(const char *name,
		const char *parent, void __iomem *reg, u32 pctln, u8 shift,
		u8 val, unsigned int *share_count, spinlock_t *lock)
{
	return s32v234_clk_register_gate2(NULL, name, parent,
			CLK_SET_RATE_PARENT, reg, pctln, shift, val, 0,
			lock, share_count);
}

#endif
