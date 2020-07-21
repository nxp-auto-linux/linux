// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include "clk.h"

/* MC_ME registers. */
#define MC_ME_CTL_KEY(MC_ME)		((MC_ME) + 0x0)
#define MC_ME_CTL_KEY_KEY		(0x00005AF0)
#define MC_ME_CTL_KEY_INVERTEDKEY	(0x0000A50F)

/* MC_ME partition definitions */
#define MC_ME_PRTN_N(MC_ME, n)			((MC_ME) + 0x100 + \
						 (n) * 0x200)
#define MC_ME_PRTN_N_PCONF(MC_ME, n)		(MC_ME_PRTN_N(MC_ME, n))
#define MC_ME_PRTN_N_PUPD(MC_ME, n)		(MC_ME_PRTN_N(MC_ME, n) + 0x4)
#define MC_ME_PRTN_N_STAT(MC_ME, n)		(MC_ME_PRTN_N(MC_ME, n) + 0x8)
#define MC_ME_PRTN_N_COFB0_STAT(MC_ME, n)	(MC_ME_PRTN_N(MC_ME, n) + 0x10)
#define MC_ME_PRTN_N_COFB0_CLKEN(MC_ME, n)	(MC_ME_PRTN_N(MC_ME, n) + 0x30)

/* MC_ME_PRTN_N_* register fields */
#define MC_ME_PRTN_N_PCE		(1 << 0)
#define MC_ME_PRTN_N_PCUD		BIT(0)
#define MC_ME_PRTN_N_PCS		BIT(0)
#define MC_ME_PRTN_N_OSSE		(1 << 2)
#define MC_ME_PRTN_N_OSSUD		BIT(2)
#define MC_ME_PRTN_N_OSSS		BIT(2)
#define MC_ME_PRTN_N_REQ(n)		BIT(n)

/* Reset domain definitions */
#define RDC_RD_N_CTRL(RDC, N)		((RDC) + (0x4 * (N)))
#define RDC_RD_N_STATUS(RDC, N)		((RDC) + 0x80 + (0x4 * (N)))
#define RD_CTRL_UNLOCK_MASK		(0x80000000)
#define RDC_RD_INTERCONNECT_DISABLE	BIT(3)
#define RDC_RD_INTERCONNECT_DISABLE_STAT BIT(4)

/* RGM */
#define RGM_PRST(MC_RGM, per)		((MC_RGM) + 0x40 + \
					 ((per) * 0x8))
#define RGM_PSTAT(rgm, per)		((rgm) + 0x140 + \
					 ((per) * 0x8))
#define PSTAT_PERIPH_n_STAT(n)		BIT(n)
#define PRST_PERIPH_n_RST(n)		BIT(n)

struct clk_part_block {
	struct clk_hw hw;
	const struct clk_ops *ops;
	struct s32gen1_clk_modules *clk_mods;
	u32 part;
	u32 block;
	bool check_status;
};

static inline struct clk_part_block *to_clk_part_block(struct clk_hw *hw)
{
	return container_of(hw, struct clk_part_block, hw);
}

static void mc_me_wait_update(u32 partition_n, u32 mask,
			      struct s32gen1_clk_modules *clk_modules)
{
	void __iomem *mc_me = clk_modules->mc_me;
	u32 pupd = readl_relaxed(MC_ME_PRTN_N_PUPD(mc_me, partition_n));

	writel_relaxed(pupd | mask, MC_ME_PRTN_N_PUPD(mc_me, partition_n));
	writel_relaxed(MC_ME_CTL_KEY_KEY, MC_ME_CTL_KEY(mc_me));
	writel_relaxed(MC_ME_CTL_KEY_INVERTEDKEY, MC_ME_CTL_KEY(mc_me));

	while (readl_relaxed(MC_ME_PRTN_N_PUPD(mc_me, partition_n)) & mask)
		;
}

static void enable_partition(u32 partition_n,
			     struct s32gen1_clk_modules *clk_modules)
{
	void __iomem *mc_me = clk_modules->mc_me;
	void __iomem *rdc = clk_modules->rdc;
	void __iomem *rgm = clk_modules->rgm;
	u32 pconf, prst;
	u32 rdc_ctrl;

	pconf = readl_relaxed(MC_ME_PRTN_N_PCONF(mc_me, partition_n));

	writel_relaxed(pconf | MC_ME_PRTN_N_PCE,
		       MC_ME_PRTN_N_PCONF(mc_me, partition_n));

	mc_me_wait_update(partition_n, MC_ME_PRTN_N_PCUD, clk_modules);

	while (!(readl_relaxed(MC_ME_PRTN_N_STAT(mc_me, partition_n)) &
	       MC_ME_PRTN_N_PCS))
		;

	/* Unlock RDC register write */
	rdc_ctrl = readl_relaxed(RDC_RD_N_CTRL(rdc, partition_n));
	writel_relaxed(rdc_ctrl | RD_CTRL_UNLOCK_MASK,
		       RDC_RD_N_CTRL(rdc, partition_n));

	/* Enable the XBAR interface */
	rdc_ctrl = readl_relaxed(RDC_RD_N_CTRL(rdc, partition_n));
	rdc_ctrl &= ~RDC_RD_INTERCONNECT_DISABLE;
	writel_relaxed(rdc_ctrl, RDC_RD_N_CTRL(rdc, partition_n));

	/* Wait until XBAR interface enabled */
	while ((readl_relaxed(RDC_RD_N_STATUS(rdc, partition_n)) &
		RDC_RD_INTERCONNECT_DISABLE_STAT))
		;

	/* Lift reset for partition */
	prst = readl_relaxed(RGM_PRST(rgm, partition_n));
	writel_relaxed(prst & (~PRST_PERIPH_n_RST(0)),
		       RGM_PRST(rgm, partition_n));

	/* Follow steps to clear OSSE bit */
	pconf = readl_relaxed(MC_ME_PRTN_N_PCONF(mc_me, partition_n));
	writel_relaxed(pconf & ~MC_ME_PRTN_N_OSSE,
			MC_ME_PRTN_N_PCONF(mc_me, partition_n));

	mc_me_wait_update(partition_n, MC_ME_PRTN_N_OSSUD, clk_modules);

	while (readl_relaxed(MC_ME_PRTN_N_STAT(mc_me, partition_n)) &
			MC_ME_PRTN_N_OSSS)
		;

	while (readl_relaxed(RGM_PSTAT(rgm, partition_n)) &
			PSTAT_PERIPH_n_STAT(0))
		;

	/* Lock RDC register write */
	rdc_ctrl = readl_relaxed(RDC_RD_N_CTRL(rdc, partition_n));
	writel_relaxed(rdc_ctrl & ~RD_CTRL_UNLOCK_MASK,
		       RDC_RD_N_CTRL(rdc, partition_n));
}

static int is_enabled_clk_part_block(struct clk_hw *hw)
{
	struct clk_part_block *block = to_clk_part_block(hw);
	void __iomem *mc_me = block->clk_mods->mc_me;
	u32 part_status;
	u32 partition_n = block->part;
	u32 block_mask = readl_relaxed(MC_ME_PRTN_N_COFB0_CLKEN(mc_me,
								partition_n));

	part_status = readl_relaxed(MC_ME_PRTN_N_STAT(mc_me, partition_n));
	if ((MC_ME_PRTN_N_PCS & part_status) &&
	    (block_mask & MC_ME_PRTN_N_REQ(block->block)))
		return 1;

	return 0;
}

static int enable_clk_part_block(struct clk_hw *hw)
{
	struct clk_part_block *block = to_clk_part_block(hw);
	struct s32gen1_clk_modules *clk_modules = block->clk_mods;
	void *mc_me = clk_modules->mc_me;
	u32 block_mask = MC_ME_PRTN_N_REQ(block->block);
	u32 partition_n = block->part;
	u32 part_status, clken, pconf;

	if (is_enabled_clk_part_block(hw))
		return 0;

	part_status = readl_relaxed(MC_ME_PRTN_N_STAT(mc_me, partition_n));

	/* Enable a partition only if it's disabled */
	if (!(MC_ME_PRTN_N_PCS & part_status))
		enable_partition(partition_n, clk_modules);

#ifndef CONFIG_S32GEN1_SIMULATOR
	clken = readl_relaxed(MC_ME_PRTN_N_COFB0_CLKEN(mc_me, partition_n));
	writel_relaxed(clken | block_mask,
		       MC_ME_PRTN_N_COFB0_CLKEN(mc_me, partition_n));

	pconf = readl_relaxed(MC_ME_PRTN_N_PCONF(mc_me, partition_n));
	writel_relaxed(pconf | MC_ME_PRTN_N_PCE,
		       MC_ME_PRTN_N_PCONF(mc_me, partition_n));

	mc_me_wait_update(partition_n, MC_ME_PRTN_N_PCUD, clk_modules);

	if (!block->check_status)
		return 0;

	while (!(readl_relaxed(MC_ME_PRTN_N_COFB0_STAT(mc_me, partition_n)) &
		 block_mask))
		;
#endif
	return 0;
}

static const struct clk_ops clk_part_block_ops = {
	.enable = enable_clk_part_block,
	.is_enabled = is_enabled_clk_part_block,
};

struct clk *s32gen1_clk_part_block(const char *name, const char *parent,
				struct s32gen1_clk_modules *clk_modules,
				u32 part, u32 block_id,
				spinlock_t *lock, bool check_status)
{
	struct clk_part_block *block;
	struct clk *clk;
	struct clk_init_data init;

	block = kzalloc(sizeof(*block), GFP_KERNEL);
	if (!block)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_part_block_ops;
	init.flags = 0;
	init.parent_names = &parent;
	init.num_parents =  1;

	block->hw.init = &init;
	block->part = part;
	block->block = block_id;
	block->check_status = check_status;
	block->clk_mods = clk_modules;

	clk = clk_register(NULL, &block->hw);
	if (IS_ERR(clk))
		kfree(block);

	return clk;
}
