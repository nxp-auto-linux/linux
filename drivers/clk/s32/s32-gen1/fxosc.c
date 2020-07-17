// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include "clk.h"

/* Control Register */
#define FXOSC_CTRL(BASE)		((BASE) + 0x0)
#define FXOSC_CTRL_OSCON		BIT(0)

#define FXOSC_CTRL_GM_SEL(VAL)		(FXOSC_CTRL_GM_SEL_MASK & ((VAL) << \
					 FXOSC_CTRL_GM_SEL_OFFSET))
#define FXOSC_CTRL_GM_SEL_MASK		(0x000000F0)
#define FXOSC_CTRL_GM_SEL_OFFSET	(4)

#define FXOSC_CTRL_EOCV(VAL)		(FXOSC_CTRL_EOCV_MASK & ((VAL) << \
					 FXOSC_CTRL_EOCV_OFFSET))
#define FXOSC_CTRL_EOCV_MASK		(0x00FF0000)
#define FXOSC_CTRL_EOCV_OFFSET		(16)

#define FXOSC_CTRL_COMP_EN		BIT(24)
#define FXOSC_CTRL_OSC_BYP		BIT(31)

/* Status register */
#define FXOSC_STAT(BASE)		((BASE) + 0x4)
#define FXOSC_STAT_OSC_STAT		BIT(31)

#define to_clk_fixed_rate(_hw) container_of(_hw, struct clk_fixed_rate, hw)

struct fxosc {
	struct clk_fixed_rate fixed;
	const struct clk_ops *ops;
	void __iomem *base;
};

static inline struct fxosc *to_fxosc(struct clk_hw *hw)
{
	struct clk_fixed_rate *fixed = to_clk_fixed_rate(hw);

	return container_of(fixed, struct fxosc, fixed);
}

static unsigned long fxosc_recalc_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct fxosc *osc = to_fxosc(hw);

	return osc->ops->recalc_rate(&osc->fixed.hw, parent_rate);
}

static unsigned long fxosc_recalc_accuracy(struct clk_hw *hw,
					   unsigned long parent_accuracy)
{
	struct fxosc *osc = to_fxosc(hw);

	return osc->ops->recalc_accuracy(&osc->fixed.hw, parent_accuracy);
}

static int fxosc_is_enabled(struct clk_hw *hw)
{
	struct fxosc *osc = to_fxosc(hw);
	void *base = osc->base;

	if (readl(FXOSC_CTRL(base)) & FXOSC_CTRL_OSCON)
		return 1;

	return 0;
}

static int fxosc_enable(struct clk_hw *hw)
{
	struct fxosc *osc = to_fxosc(hw);
	void *base = osc->base;
	uint32_t ctrl;

	if (fxosc_is_enabled(hw))
		return 0;

	ctrl = FXOSC_CTRL_COMP_EN;
	ctrl &= ~FXOSC_CTRL_OSC_BYP;
	ctrl |= FXOSC_CTRL_EOCV(0x1);
	ctrl |= FXOSC_CTRL_GM_SEL(0x7);
	writel(ctrl, FXOSC_CTRL(base));

	/* Switch ON the crystal oscillator. */
	writel(FXOSC_CTRL_OSCON | readl(FXOSC_CTRL(base)), FXOSC_CTRL(base));

	/* Wait until the clock is stable. */
	while (!(readl(FXOSC_STAT(base)) & FXOSC_STAT_OSC_STAT))
		;

	return 0;
}

const struct clk_ops fxosc_ops = {
	.recalc_rate = fxosc_recalc_rate,
	.recalc_accuracy = fxosc_recalc_accuracy,
	.enable = fxosc_enable,
	.is_enabled = fxosc_is_enabled,
};

struct clk *s32gen1_fxosc(const char *compatible)
{
	struct device_node *np;
	struct fxosc *osc;
	struct clk *clk;
	struct clk_init_data init;
	struct clk_fixed_rate *fixed;
	void __iomem *base;
	u32 rate;

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np) {
		pr_crit("Unable to get %s node\n", compatible);
		return NULL;
	}

	if (of_property_read_u32(np, "clock-frequency", &rate)) {
		pr_crit("Unable to get %s clock frequency\n", np->name);
		return NULL;
	}

	base = of_iomap(np, 0);
	if (WARN_ON(!base))
		return NULL;

	osc = kzalloc(sizeof(*osc), GFP_KERNEL);
	if (!osc)
		return ERR_PTR(-ENOMEM);

	osc->ops = &clk_fixed_rate_ops;
	osc->base = base;

	init.name = "fxosc";
	init.ops = &fxosc_ops;
	init.flags = 0;
	init.parent_names = NULL;
	init.num_parents = 0;

	fixed = &osc->fixed;

	/* struct clk_fixed_rate assignments */
	fixed->fixed_rate = rate;
	fixed->fixed_accuracy = 0;
	fixed->hw.init = &init;

	clk = clk_register(NULL, &osc->fixed.hw);
	if (IS_ERR(clk))
		kfree(osc);

	return clk;
}
