/*
 * Copyright 2018,2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include "clk.h"
#include "pll_mux.h"

/*
 * struct clk_plldig - S32 gen1 PLLDIG clock
 * @clk_hw:	   clock source
 * @base:	   base address of PLL registers
 * @plldv_mfi:	   multiplication loop factor divider
 * @plldv_pllodiv:  PHI frequency divider
 * @phi_nr: PHI number
 *
 * PLLDIG clock, found on S32GEN1 series.
 */
struct clk_plldig {
	struct clk_hw	hw;
	void __iomem	*base;
	struct clk_hw	*pll_mux;
	enum s32gen1_plldig_type type;
	u32		plldv_mfi;
	u32		pllfd_mfn;
	u32		plldv_pllodiv[PHI_MAXNUMBER];
	u32		phi_nr;
};

struct clk_plldig_phi {
	struct clk_fixed_factor fix;
	const struct clk_ops *ops;
	void __iomem *pll_base;
	u32 index;
};

static const unsigned long arm_phis_max_freq[] = {
	ARMPLL_MAX_PHI0_MAX_RATE,
	ARMPLL_MAX_PHI1_MAX_RATE
};

static const unsigned long periph_phis_max_freq[] = {
	PERIPHPLL_MAX_PHI0_MAX_RATE,
	PERIPHPLL_MAX_PHI1_MAX_RATE,
	PERIPHPLL_MAX_PHI2_MAX_RATE,
	PERIPHPLL_MAX_PHI3_MAX_RATE,
	PERIPHPLL_MAX_PHI4_MAX_RATE,
	PERIPHPLL_MAX_PHI5_MAX_RATE,
	PERIPHPLL_MAX_PHI6_MAX_RATE,
	PERIPHPLL_MAX_PHI7_MAX_RATE
};

static const unsigned long ddr_phis_max_freq[] = {
	DDRPLL_MAX_PHI0_MAX_RATE
};

static const unsigned long accel_phis_max_freq[] = {
	ACCELPLL_MAX_PHI0_MAX_RATE,
	ACCELPLL_MAX_PHI1_MAX_RATE
};

static inline struct clk_plldig_phi *to_plldig_phi(struct clk_hw *hw)
{
	struct clk_fixed_factor *fix = to_clk_fixed_factor(hw);

	return container_of(fix, struct clk_plldig_phi, fix);
}

static inline u32 get_phi_div_factor(void __iomem *base, u32 phinum)
{
	u32 pllodiv = readl_relaxed(PLLDIG_PLLODIV(base, phinum));

	return PLLDIG_PLLODIV_DIV(pllodiv) + 1;
}

static inline bool is_pll_enabled(void __iomem *pll_base)
{
	u32 pllcr, pllsr;

	pllcr = readl_relaxed(PLLDIG_PLLCR(pll_base));
	pllsr = readl_relaxed(PLLDIG_PLLSR(pll_base));

	/* Enabled and locked PLL */
	return !(pllcr & PLLDIG_PLLCR_PLLPD) && (pllsr & PLLDIG_PLLSR_LOCK);
}

static inline void disable_pll_hw(void __iomem *pll_addr)
{
	writel(PLLDIG_PLLCR_PLLPD, PLLDIG_PLLCR(pll_addr));
}

static inline void enable_pll_hw(void __iomem *pll_addr)
{
	/* Enable the PLL. */
	writel(0x0, PLLDIG_PLLCR(pll_addr));

	/* Poll until PLL acquires lock. */
	while (!(readl(PLLDIG_PLLSR(pll_addr)) & PLLDIG_PLLSR_LOCK))
		;
}

#define to_clk_plldig(_hw) container_of(_hw, struct clk_plldig, hw)
static unsigned long get_pllx_max_vco_rate(enum s32gen1_plldig_type type)
{
	switch (type) {
	case S32GEN1_PLLDIG_ARM:
		return ARMPLL_MAX_VCO_RATE;
	case S32GEN1_PLLDIG_PERIPH:
		return PERIPHPLL_MAX_VCO_RATE;
	case S32GEN1_PLLDIG_DDR:
		return DDRPLL_MAX_VCO_RATE;
	case S32GEN1_PLLDIG_ACCEL:
		return ACCELPLL_MAX_VCO_RATE;
	default:
		pr_warn("Unsupported PLL. Use: %d or %d\n",
			S32GEN1_PLLDIG_ARM,
			S32GEN1_PLLDIG_ACCEL);
		return 0;
	}
}

static int get_pllx_phi_nr(enum s32gen1_plldig_type type)
{
	switch (type) {
	case S32GEN1_PLLDIG_ARM:
		return ARMPLL_PHI_Nr;
	case S32GEN1_PLLDIG_PERIPH:
		return PERIPHPLL_PHI_Nr;
	case S32GEN1_PLLDIG_ACCEL:
		return ACCELPLL_PHI_Nr;
	case S32GEN1_PLLDIG_DDR:
		return DDRPLL_PHI_Nr;
	default:
		return -EINVAL;
	}
}

static unsigned long get_pllx_phiy_max_rate(enum s32gen1_plldig_type type,
					    unsigned int phi)
{
	if (phi >= get_pllx_phi_nr(type))
		return 0;

	switch (type) {
	case S32GEN1_PLLDIG_ARM:
		return arm_phis_max_freq[phi];
	case S32GEN1_PLLDIG_PERIPH:
		return periph_phis_max_freq[phi];
	case S32GEN1_PLLDIG_ACCEL:
		return accel_phis_max_freq[phi];
	case S32GEN1_PLLDIG_DDR:
		return ddr_phis_max_freq[phi];
	default:
		pr_warn("Unsupported PLL. Use: %d or %d\n",
			S32GEN1_PLLDIG_ARM,
			S32GEN1_PLLDIG_ACCEL);
	}
	return 0;
}

static unsigned long clk_plldig_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	u32 plldv = readl_relaxed(PLLDIG_PLLDV(pll->base));
	u32 pllfd = readl_relaxed(PLLDIG_PLLFD(pll->base));
	u32 rdiv, mfi, mfn;
	unsigned long vco;

	rdiv = (plldv & PLLDIG_PLLDV_RDIV_MASK)
			 >> PLLDIG_PLLDV_RDIV_OFFSET;
	mfi = (plldv & PLLDIG_PLLDV_MFI_MASK);

	mfn = (pllfd & PLLDIG_PLLFD_MFN_MASK);

	if (rdiv == 0)
		rdiv = 1;

	vco = (parent_rate * (18432 * mfi + mfn)) / (18432 * rdiv);

	return vco;
}

static long clk_plldig_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *prate)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	unsigned long max_allowed_rate = get_pllx_max_vco_rate(pll->type);

	if (rate > max_allowed_rate)
		rate = max_allowed_rate;
	else if (rate < MIN_VCO_RATE)
		rate = MIN_VCO_RATE;

	return rate;
}

static int clk_plldig_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	u32 pllfd, rdiv, pllodiv;
	int i;
	unsigned long max_phi_freq;
	unsigned long max_allowed_rate = get_pllx_max_vco_rate(pll->type);

	if (rate < MIN_VCO_RATE || rate > max_allowed_rate)
		return -EINVAL;

	for (i = 0; i < pll->phi_nr; i++) {
		max_phi_freq = get_pllx_phiy_max_rate(pll->type, i);
		if (max_phi_freq <= 0 ||
				(rate / pll->plldv_pllodiv[i]) > max_phi_freq)
			return -EINVAL;
	}

	pllfd = readl_relaxed(PLLDIG_PLLFD(pll->base));
	rdiv = (parent_rate * (18432 * pll->plldv_mfi + pll->pllfd_mfn)) /
		(18432 * rate);

	/* Disable dividers. */
	for (i = 0; i < pll->phi_nr; i++)
		writel_relaxed(0x0, PLLDIG_PLLODIV(pll, i));

	/* Disable PLL. */
	writel_relaxed(PLLDIG_PLLCR_PLLPD, PLLDIG_PLLCR(pll->base));

	writel_relaxed(PLLDIG_PLLDV_RDIV_SET(rdiv) |
			PLLDIG_PLLDV_MFI_SET(pll->plldv_mfi),
			PLLDIG_PLLDV(pll->base));

	writel_relaxed(pllfd | PLLDIG_PLLFD_MFN_SET(pll->pllfd_mfn) |
			PLLDIG_PLLFD_SMDEN, PLLDIG_PLLFD(pll->base));

	/* Calculate Output Frequency Divider. */
	for (i = 0; i < pll->phi_nr; i++)  {
		pllodiv = readl_relaxed(PLLDIG_PLLODIV(pll->base, i));
		writel_relaxed(pllodiv |
			PLLDIG_PLLODIV_DIV_SET(pll->plldv_pllodiv[i] - 1),
			PLLDIG_PLLODIV(pll, i));
	}

	/* Enable the PLL. */
	writel_relaxed(0x0, PLLDIG_PLLCR(pll));

	/* Poll until PLL acquires lock. */
	while (!(readl_relaxed(PLLDIG_PLLSR(pll)) & PLLDIG_PLLSR_LOCK))
		;

	/* Enable dividers. */
	for (i = 0; i < pll->phi_nr; i++)
		writel_relaxed(PLLDIG_PLLODIV_DE |
				readl_relaxed(PLLDIG_PLLODIV(pll, i)),
				PLLDIG_PLLODIV(pll, i));

	return 0;
}

static inline u32 get_pll_mfi(void __iomem *base)
{
	u32 plldv = readl_relaxed(PLLDIG_PLLDV(base));

	return PLLDIG_PLLDV_MFI(plldv);
}

static inline u32 get_pll_mfn(void __iomem *base)
{
	u32 pllfd = readl_relaxed(PLLDIG_PLLFD(base));

	return PLLDIG_PLLFD_MFN_SET(pllfd);
}

static int clk_plldig_is_enabled(struct clk_hw *hw)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	void __iomem *base = pll->base;
	struct s32gen1_pll_mux *pll_mux;
	u32 mfi, mfn;
	u32 pll_source;

	if (!pll->pll_mux)
		return -EINVAL;

	if (!is_pll_enabled(base))
		return 0;

	pll_mux = to_s32gen1_clk_pll_mux(pll->pll_mux);
	pll_source = readl_relaxed(pll_mux->s32mux.mux.reg);

	if (pll_source != pll_mux->source)
		return 0;

	mfi = get_pll_mfi(base);
	mfn = get_pll_mfn(base);

	if (mfn != pll->pllfd_mfn || mfi != pll->plldv_mfi)
		return 0;

	return 1;
}

static int clk_plldig_enable(struct clk_hw *hw)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	void __iomem *base = pll->base;
	struct s32gen1_pll_mux *pll_mux;
	u32 rdiv = 1;

	pll_mux = to_s32gen1_clk_pll_mux(pll->pll_mux);

	if (clk_plldig_is_enabled(hw))
		return 0;

	/* Disable PLL */
	disable_pll_hw(base);

	/* Program PLLCLKMUX */
	writel_relaxed(pll_mux->source, pll_mux->s32mux.mux.reg);

	/* Program VCO */
	writel(PLLDIG_PLLDV_RDIV_SET(rdiv) | PLLDIG_PLLDV_MFI(pll->plldv_mfi),
	       PLLDIG_PLLDV(base));
	writel(PLLDIG_PLLFD_MFN_SET(pll->pllfd_mfn) |
	       PLLDIG_PLLFD_SMDEN, PLLDIG_PLLFD(base));

	enable_pll_hw(base);

	return 0;
}

static long plldig_phi_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *prate)
{
	struct clk_plldig_phi *phi = to_plldig_phi(hw);

	return phi->ops->round_rate(&phi->fix.hw, rate, prate);
}

static int plldig_phi_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct clk_plldig_phi *phi = to_plldig_phi(hw);

	return phi->ops->set_rate(&phi->fix.hw, rate, parent_rate);
}

static unsigned long plldig_phi_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct clk_plldig_phi *phi = to_plldig_phi(hw);

	return phi->ops->recalc_rate(&phi->fix.hw, parent_rate);
}

static int plldig_phi_is_enabled(struct clk_hw *hw)
{
	struct clk_plldig_phi *phi = to_plldig_phi(hw);
	void __iomem *base = phi->pll_base;

	if (get_phi_div_factor(base, phi->index) == phi->fix.div)
		return 1;

	return 0;
}

static void disable_odiv(void __iomem *pll_addr, u32 div_index)
{
	u32 pllodiv = readl_relaxed(PLLDIG_PLLODIV(pll_addr, div_index));

	writel_relaxed(pllodiv & ~PLLDIG_PLLODIV_DE,
	       PLLDIG_PLLODIV(pll_addr, div_index));
}

static void enable_odiv(void __iomem *pll_addr, u32 div_index)
{
	u32 pllodiv = readl_relaxed(PLLDIG_PLLODIV(pll_addr, div_index));

	writel_relaxed(pllodiv | PLLDIG_PLLODIV_DE,
	       PLLDIG_PLLODIV(pll_addr, div_index));
}

static int plldig_phi_enable(struct clk_hw *hw)
{
	struct clk_plldig_phi *phi = to_plldig_phi(hw);
	void __iomem *base = phi->pll_base;
	u32 index = phi->index;
	u32 pllodiv;

	if (plldig_phi_is_enabled(hw))
		return 0;

	pllodiv = readl_relaxed(PLLDIG_PLLODIV(base, index));
	if (pllodiv & PLLDIG_PLLODIV_DE)
		disable_odiv(base, index);

	pllodiv = PLLDIG_PLLODIV_DIV_SET(phi->fix.div - 1);
	writel_relaxed(pllodiv, PLLDIG_PLLODIV(base, index));

	enable_odiv(base, index);

	return 0;
}

const struct clk_ops plldig_phi_ops = {
	.round_rate = plldig_phi_round_rate,
	.set_rate = plldig_phi_set_rate,
	.recalc_rate = plldig_phi_recalc_rate,
	.enable = plldig_phi_enable,
	.is_enabled = plldig_phi_is_enabled,
};

struct clk *s32gen1_clk_plldig_phi(enum s32gen1_plldig_type type,
			       const char *name, const char *parent,
			       void __iomem *base, u32 phinum)
{
	struct clk_plldig_phi *phi;
	struct clk *clk;
	struct clk_init_data init;
	struct clk_fixed_factor *fix;
	u32 factor;

	if (!base)
		return ERR_PTR(-ENOMEM);

	if (phinum >= get_pllx_phi_nr(type))
		return ERR_PTR(-EINVAL);

	phi = kzalloc(sizeof(*phi), GFP_KERNEL);
	if (!phi)
		return ERR_PTR(-ENOMEM);

	factor = get_phi_div_factor(base, phinum);

	phi->ops = &clk_fixed_factor_ops;
	phi->pll_base = base;
	phi->index = phinum;

	fix = &phi->fix;

	init.name = name;
	init.ops = &plldig_phi_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = &parent;
	init.num_parents = 1;

	/* struct clk_fixed_factor assignments */
	fix->mult = 1;
	fix->div = factor;
	fix->hw.init = &init;

	clk = clk_register(NULL, &phi->fix.hw);
	if (IS_ERR(clk))
		kfree(phi);

	return clk;
}

static const struct clk_ops clk_plldig_ops = {
	.recalc_rate	= clk_plldig_recalc_rate,
	.round_rate	= clk_plldig_round_rate,
	.set_rate	= clk_plldig_set_rate,
	.enable		= clk_plldig_enable,
	.is_enabled	= clk_plldig_is_enabled,
};

struct clk *s32gen1_clk_plldig(enum s32gen1_plldig_type type, const char *name,
			   const char *parent_name, struct clk *pll_mux,
			   void __iomem *base, u32 pllodiv[PHI_MAXNUMBER],
			   u32 phi_nr)
{
	struct clk_plldig *pll;
	const struct clk_ops *ops;
	struct clk *clk;
	struct clk_init_data init;
	int i;

	if (get_pllx_phi_nr(type) == -EINVAL ||
			phi_nr > get_pllx_phi_nr(type))
		return ERR_PTR(-EINVAL);

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	ops = &clk_plldig_ops;

	pll->base = base;
	pll->type = type;
	pll->plldv_mfi = get_pll_mfi(base);
	pll->pllfd_mfn = get_pll_mfn(base);
	pll->phi_nr = phi_nr;

	for (i = 0; i < phi_nr; i++)
		pll->plldv_pllodiv[i] = pllodiv[i];

	init.name = name;
	init.ops = ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
