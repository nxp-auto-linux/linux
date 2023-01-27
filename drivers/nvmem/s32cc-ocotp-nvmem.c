// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <dt-bindings/nvmem/s32cc-ocotp-nvmem.h>

#define	OCOTP_WORD(X)		BIT(X)
#define	OCOTP_WORD_RANGE(L, H)	GENMASK(H, L)

#define S32CC_OCOTP_BANK_OFFSET     0x200U
#define S32CC_OCOTP_BANK_SIZE       0x20U
#define S32CC_OCOTP_WORD_SIZE       0x4U

struct s32cc_fuse {
	u8 bank;
	u8 words_mask;
};

struct s32cc_fuse_map {
	const struct s32cc_fuse *map;
	size_t n_entries;
};

static const struct s32cc_fuse s32cc_fuse_map[] = {
	{ .bank = 0, .words_mask = OCOTP_WORD_RANGE(2, 6) },
	{ .bank = 1, .words_mask = OCOTP_WORD_RANGE(5, 7) },
	{ .bank = 2, .words_mask = OCOTP_WORD_RANGE(0, 1) },
	{ .bank = 2, .words_mask = OCOTP_WORD_RANGE(2, 4) },
	{ .bank = 4, .words_mask = OCOTP_WORD(6) },
	{ .bank = 5, .words_mask = OCOTP_WORD(1) },
	{ .bank = 5, .words_mask = OCOTP_WORD(2) },
	{ .bank = 6, .words_mask = OCOTP_WORD(7) },
	{ .bank = 7, .words_mask = OCOTP_WORD_RANGE(0, 1) },
	{ .bank = 11, .words_mask = OCOTP_WORD_RANGE(0, 5) },
	{ .bank = 11, .words_mask = OCOTP_WORD_RANGE(6, 7) },
	{ .bank = 12, .words_mask = OCOTP_WORD_RANGE(0, 2) },
	{ .bank = 12, .words_mask = OCOTP_WORD(7) },
	{ .bank = 13, .words_mask = OCOTP_WORD_RANGE(2, 4) },
	{ .bank = 14,
		.words_mask = OCOTP_WORD(1) | OCOTP_WORD(4) | OCOTP_WORD(5) },
	{ .bank = 15, .words_mask = OCOTP_WORD_RANGE(5, 7) },
};

static const struct s32cc_fuse_map s32cc_map = {
	.map = s32cc_fuse_map,
	.n_entries = ARRAY_SIZE(s32cc_fuse_map),
};

struct s32cc_ocotp_priv {
	struct device *dev;
	void __iomem *base;
	struct nvmem_config *config;
	const struct s32cc_fuse_map *fuses;
};

static const struct of_device_id ocotp_of_match[] = {
	{ .compatible = "nxp,s32g-ocotp", .data = &s32cc_map},
	{ .compatible = "nxp,s32r45-ocotp", .data = &s32cc_map},
	{ /* sentinel */ }
};

static u32 get_bank_index(unsigned int offset)
{
	return (offset - S32CC_OCOTP_BANK_OFFSET) / S32CC_OCOTP_BANK_SIZE;
}

static u32 get_word_index(unsigned int offset)
{
	return offset % S32CC_OCOTP_BANK_SIZE / S32CC_OCOTP_WORD_SIZE;
}

static const struct s32cc_fuse *get_fuse(const struct s32cc_fuse_map *map,
					 u32 bank, u32 word)
{
	size_t i;

	for (i = 0; i < map->n_entries; i++) {
		if (map->map[i].bank == bank &&
		    map->map[i].words_mask & OCOTP_WORD(word))
			return &map->map[i];
	}

	return NULL;
}

static bool is_valid_word(const struct s32cc_fuse_map *map,
			  u32 bank, u32 word)
{
	if (bank >= map->n_entries)
		return false;

	if (get_fuse(map, bank, word))
		return true;

	return false;
}

static int init_word_bank(struct s32cc_ocotp_priv *s32cc_data,
			  unsigned int offset, int size, u32 *bank, u32 *word)
{
	if (offset < S32CC_OCOTP_BANK_OFFSET)
		return -EINVAL;

	if (size != 4)
		return -EPERM;

	*bank = get_bank_index(offset);
	*word = get_word_index(offset);

	if (!is_valid_word(s32cc_data->fuses, *bank, *word)) {
		dev_err(s32cc_data->dev, "[bank %u, word %u] is not a valid fuse\n",
			*bank, *word);
		return -EINVAL;
	}

	return 0;
}

static int s32cc_ocotp_read(void *context, unsigned int offset,
			    void *val, size_t bytes)
{
	struct s32cc_ocotp_priv *s32cc_data = context;
	u32 bank, word;
	int ret;

	if (bytes != OCOTP_CELL_SIZE)
		return -EOPNOTSUPP;

	ret = init_word_bank(s32cc_data, offset, bytes, &bank, &word);
	if (ret)
		return ret;

	/* Read from Fuse OCOTP Shadow registers */
	*(u32 *)val = ioread32(s32cc_data->base + offset);

	return 0;
}

static struct nvmem_config s32cc_ocotp_nvmem_config = {
	.name = "s32cc-ocotp",
	.read_only = true,
	.word_size = 4,
	.reg_read = s32cc_ocotp_read,
};

static int s32cc_ocotp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	const struct of_device_id *of_matched_dt_id;
	struct s32cc_ocotp_priv *s32cc_data;
	struct nvmem_device *nvmem;

	of_matched_dt_id = of_match_device(ocotp_of_match, dev);
	if (!of_matched_dt_id) {
		dev_err(dev, "Unable to find driver data.\n");
		return -ENODEV;
	}

	s32cc_data = devm_kzalloc(dev, sizeof(struct s32cc_ocotp_priv),
				  GFP_KERNEL);
	if (!s32cc_data)
		return -ENOMEM;

	s32cc_data->fuses = of_device_get_match_data(dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Cannot obtain OCOTP resource.\n");
		return -ENODEV;
	}

	s32cc_data->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(s32cc_data->base)) {
		dev_err(dev, "Cannot map OCOTP device");
		return PTR_ERR(s32cc_data->base);
	}

	s32cc_data->dev = dev;
	s32cc_ocotp_nvmem_config.dev = dev;
	s32cc_ocotp_nvmem_config.priv = s32cc_data;
	s32cc_ocotp_nvmem_config.size = resource_size(res);
	s32cc_data->config = &s32cc_ocotp_nvmem_config;

	nvmem = devm_nvmem_register(dev, &s32cc_ocotp_nvmem_config);

	return PTR_ERR_OR_ZERO(nvmem);
}

static struct platform_driver s32cc_ocotp_driver = {
	.probe = s32cc_ocotp_probe,
	.driver = {
		.name = "s32cc-ocotp",
		.of_match_table = ocotp_of_match,
	},
};
module_platform_driver(s32cc_ocotp_driver);
MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("S32CC OCOTP driver");
MODULE_LICENSE("GPL v2");
