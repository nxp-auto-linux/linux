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

#ifndef __MACH_S32_CLK_H
#define __MACH_S32_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>

#define PNAME(x) \
	static const char *x[] __initconst

void s32_check_clocks(struct clk *clks[], unsigned int count);

struct clk *s32_obtain_fixed_clock(
			const char *name, unsigned long rate);

struct clk *s32_clk_gate_exclusive(const char *name, const char *parent,
	 void __iomem *reg, u8 shift, u32 exclusive_mask);

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
		void __iomem *reg, u8 shift, u8 width, spinlock_t *lock)
{
	struct clk *tmp_clk = clk_register_divider(NULL, name, parent,
			      CLK_SET_RATE_PARENT,
			      reg, shift, width, 0, lock);

	return tmp_clk;
}

static inline struct clk *s32_clk_divider_flags(const char *name,
		const char *parent, void __iomem *reg, u8 shift, u8 width,
		unsigned long flags, spinlock_t *lock)
{
	return clk_register_divider(NULL, name, parent, flags,
			reg, shift, width, 0, lock);
}

static inline struct clk *s32_clk_gate(const char *name, const char *parent,
		void __iomem *reg, u8 shift, spinlock_t *lock)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, 0, lock);
}

static inline struct clk *s32_clk_gate_dis(const char *name, const char *parent,
		void __iomem *reg, u8 shift, spinlock_t *lock)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, CLK_GATE_SET_TO_DISABLE, lock);
}

static inline struct clk *s32_clk_mux(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, int num_parents,
		spinlock_t *lock)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT, reg, shift,
			width, 0, lock);
}

static inline struct clk *s32_clk_mux_flags(const char *name,
		void __iomem *reg, u8 shift, u8 width, const char **parents,
		int num_parents, unsigned long flags, spinlock_t *lock)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			flags | CLK_SET_RATE_NO_REPARENT, reg, shift, width, 0,
			lock);
}

static inline struct clk *s32_clk_mux_table(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, int num_parents,
		u32 *table, spinlock_t *lock)
{
	u32 mask = BIT(width) - 1;

	return clk_register_mux_table(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT, reg, shift,
			mask, 0, table, lock);
}

static inline struct clk *s32_clk_fixed_factor(const char *name,
		const char *parent, unsigned int mult, unsigned int div)
{
	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, mult, div);
}

#endif
