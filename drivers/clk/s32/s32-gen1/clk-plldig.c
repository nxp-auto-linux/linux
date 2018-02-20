/*
 * Copyright 2018 NXP
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
	enum s32gen1_plldig_type type;
	u32		plldv_mfi;
	u32		pllfd_mfn;
	u32		plldv_pllodiv[PHI_MAXNUMBER];
	u32		phi_nr;
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

static const unsigned long aurora_phis_max_freq[] = {
	AURORAPLL_MAX_PHI0_MAX_RATE
};

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
	case S32GEN1_PLLDIG_AURORA:
		return AURORAPLL_MAX_VCO_RATE;
	default:
		pr_warn("Unsupported PLL. Use: %d or %d\n",
			S32GEN1_PLLDIG_ARM,
			S32GEN1_PLLDIG_AURORA);
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
	case S32GEN1_PLLDIG_AURORA:
		return AURORAPLL_PHI_Nr;
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
	case S32GEN1_PLLDIG_AURORA:
		return aurora_phis_max_freq[phi];
	default:
		pr_warn("Unsupported PLL. Use: %d or %d\n",
			S32GEN1_PLLDIG_ARM,
			S32GEN1_PLLDIG_AURORA);
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

static const struct clk_ops clk_plldig_ops = {
	.recalc_rate	= clk_plldig_recalc_rate,
	.round_rate	= clk_plldig_round_rate,
	.set_rate	= clk_plldig_set_rate,
};

struct clk *s32gen1_clk_plldig_phi(enum s32gen1_plldig_type type,
			       const char *name, const char *parent,
			       void __iomem *base, u32 phi)
{
	u32 pllodiv, rfd_phi;

	if (!base)
		return ERR_PTR(-ENOMEM);

	if (phi >= get_pllx_phi_nr(type))
		return ERR_PTR(-EINVAL);

	pllodiv = readl_relaxed(PLLDIG_PLLODIV(base, phi));
	rfd_phi = ((pllodiv & PLLDIG_PLLODIV_DIV_MASK)
		>> PLLDIG_PLLODIV_DIV_OFFSET) + 1;

	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, 1, rfd_phi);

}

struct clk *s32gen1_clk_plldig(enum s32gen1_plldig_type type, const char *name,
			       const char *parent_name, void __iomem *base,
			       u32 plldv_mfi, u32 pllfd_mfn,
			       u32 pllodiv[PHI_MAXNUMBER], u32 phi_nr)
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
	pll->plldv_mfi = plldv_mfi;
	pll->pllfd_mfn = pllfd_mfn;
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
