/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __MACH_S32_CLK_H
#define __MACH_S32_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>
#include "dfs.h"
#include "mc_cgm.h"
#include "mc_me.h"
#include "pll.h"
#include "src.h"

#define PNAME(x) \
	static const char *x[] __initconst

extern spinlock_t s32_cgm_lock;

void s32_check_clocks(struct clk *clks[], unsigned int count);

enum s32_plldig_type {
	S32_PLLDIG_ARM,
	S32_PLLDIG_PERIPH,
	S32_PLLDIG_ENET,
	S32_PLLDIG_DDR,
	S32_PLLDIG_VIDEO,
};

struct clk *s32_clk_plldig(enum s32_plldig_type type, const char *name,
			   const char *parent_name, void __iomem *base,
			   u32 plldv_mfd, u32 plldv_mfn,
			   u32 plldv_rfdphi, u32 plldv_rfdphi1);

struct clk *s32_clk_plldig_phi(enum s32_plldig_type type, const char *name,
			       const char *parent, void __iomem *base,
			       u32 phi);
struct clk *s32_clk_dfs(enum s32_plldig_type type, const char *name,
			const char *parent_name,
			void __iomem *reg, u8 idx, u32 mfn);

struct clk *clk_register_gate2(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u32 pctln, u8 bit_idx, u8 val,
		u8 clk_gate_flags, spinlock_t *lock,
		unsigned int *share_count);

struct clk *s32_obtain_fixed_clock(
			const char *name, unsigned long rate);

struct clk *s32_clk_gate_exclusive(const char *name, const char *parent,
	 void __iomem *reg, u8 shift, u32 exclusive_mask);

static inline struct clk *s32_clk_gate2(const char *name, const char *parent,
		void __iomem *reg, u32 pctln, u8 shift, u8 val)
{
	return clk_register_gate2(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
		pctln, shift, val, 0, &s32_cgm_lock, NULL);
}

static inline struct clk *s32_clk_gate2_shared(const char *name,
		const char *parent, void __iomem *reg, u32 pctln, u8 shift,
		u8 val, unsigned int *share_count)
{
	return clk_register_gate2(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			pctln, shift, val, 0, &s32_cgm_lock, share_count);
}

struct clk *s32_clk_busy_divider(const char *name, const char *parent_name,
				 void __iomem *reg, u8 shift, u8 width,
				 void __iomem *busy_reg, u8 busy_shift);

struct clk *s32_clk_busy_mux(const char *name, void __iomem *reg, u8 shift,
			     u8 width, void __iomem *busy_reg, u8 busy_shift,
			     const char **parent_names, int num_parents);

struct clk *s32_clk_fixup_divider(const char *name, const char *parent,
				  void __iomem *reg, u8 shift, u8 width,
				  void (*fixup)(u32 *val));

struct clk *s32_clk_fixup_mux(const char *name, void __iomem *reg,
			      u8 shift, u8 width, const char **parents,
			      int num_parents, void (*fixup)(u32 *val));

static inline struct clk *s32_clk_fixed(const char *name, int rate)
{
	return clk_register_fixed_rate(NULL, name, NULL, 0, rate);
}

static inline struct clk *s32_clk_divider(const char *name, const char *parent,
		void __iomem *reg, u8 shift, u8 width)
{
	struct clk *tmp_clk = clk_register_divider(NULL, name, parent,
			      CLK_SET_RATE_PARENT,
			      reg, shift, width, 0, &s32_cgm_lock);

	return tmp_clk;
}

static inline struct clk *s32_clk_divider_flags(const char *name,
		const char *parent, void __iomem *reg, u8 shift, u8 width,
		unsigned long flags)
{
	return clk_register_divider(NULL, name, parent, flags,
			reg, shift, width, 0, &s32_cgm_lock);
}

static inline struct clk *s32_clk_gate(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, 0, &s32_cgm_lock);
}

static inline struct clk *s32_clk_gate_dis(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, CLK_GATE_SET_TO_DISABLE, &s32_cgm_lock);
}

static inline struct clk *s32_clk_mux(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT, reg, shift,
			width, 0, &s32_cgm_lock);
}

static inline struct clk *s32_clk_mux_flags(const char *name,
		void __iomem *reg, u8 shift, u8 width, const char **parents,
		int num_parents, unsigned long flags)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			flags | CLK_SET_RATE_NO_REPARENT, reg, shift, width, 0,
			&s32_cgm_lock);
}

static inline struct clk *s32_clk_fixed_factor(const char *name,
		const char *parent, unsigned int mult, unsigned int div)
{
	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, mult, div);
}

#endif
