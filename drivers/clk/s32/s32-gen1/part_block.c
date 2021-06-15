// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020-2021 NXP
 */
#include "clk.h"
#include "rdc.h"
#include <linux/mfd/s32gen1-mc_me.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of_address.h>
#include <linux/slab.h>

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
			      struct s32gen1_clk_modules *clk_mod)
{
	unsigned int pupd;

	regmap_update_bits(clk_mod->mc_me, MC_ME_PRTN_N_PUPD(partition_n),
			   mask, mask);

	regmap_write(clk_mod->mc_me, MC_ME_CTL_KEY, MC_ME_CTL_KEY_KEY);
	regmap_write(clk_mod->mc_me, MC_ME_CTL_KEY,
		     MC_ME_CTL_KEY_INVERTEDKEY);

	do {
		regmap_read(clk_mod->mc_me, MC_ME_PRTN_N_PUPD(partition_n),
			    &pupd);
	} while (pupd & mask);

}

static int is_enabled_clk_part_block(struct clk_hw *hw)
{
	struct clk_part_block *block = to_clk_part_block(hw);
	void __iomem *mc_me = block->clk_mods->mc_me;
	unsigned int block_mask, part_status;
	u32 partition_n = block->part;

	regmap_read(mc_me, MC_ME_PRTN_N_COFB0_CLKEN(partition_n),
		    &block_mask);
	regmap_read(mc_me, MC_ME_PRTN_N_STAT(partition_n),
		    &part_status);

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
	unsigned int cofb_stat, part_status;
	u32 block_mask = MC_ME_PRTN_N_REQ(block->block);
	u32 partition_n = block->part;

	if (is_enabled_clk_part_block(hw))
		return 0;

	regmap_read(mc_me, MC_ME_PRTN_N_STAT(partition_n),
		    &part_status);

	WARN(!(MC_ME_PRTN_N_PCS & part_status),
	     "Clock partition %u is not enabled. Please use reset controller driver to enable it",
	     partition_n);

#ifndef CONFIG_S32GEN1_SIMULATOR
	regmap_update_bits(mc_me, MC_ME_PRTN_N_COFB0_CLKEN(partition_n),
			   block_mask, block_mask);
	regmap_update_bits(mc_me, MC_ME_PRTN_N_PCONF(partition_n),
			   MC_ME_PRTN_N_PCE, MC_ME_PRTN_N_PCE);

	mc_me_wait_update(partition_n, MC_ME_PRTN_N_PCUD, clk_modules);

	if (!block->check_status)
		return 0;

	do {
		regmap_read(mc_me, MC_ME_PRTN_N_COFB0_STAT(partition_n),
			    &cofb_stat);
	} while (!(cofb_stat & block_mask));
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
