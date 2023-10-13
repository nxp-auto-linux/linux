// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <dt-bindings/nvmem/s32cc-gpr-nvmem.h>
#include <dt-bindings/nvmem/s32r45-gpr-nvmem.h>

#define SRC_0_OFF			0x0
#define DDR_GPR_OFF			0x600
#define SRC_1_OFF			0xA00

#define SRC_0_GMAC_0_CTRL_STS_OFF	0x4
#define SRC_0_GMAC_0_CTRL_STS_SHIFT	0
#define SRC_0_GMAC_0_CTRL_STS_MASK	GENMASK(3, SRC_0_GMAC_0_CTRL_STS_SHIFT)

#define DDR_GPR_CONFIG_0_OFF		0x0
#define DDR_GPR_CONFIG_0_PERF_CNT_SHIFT	31
#define DDR_GPR_CONFIG_0_PERF_CNT_MASK	BIT(DDR_GPR_CONFIG_0_PERF_CNT_SHIFT)

#define SRC_1_GMAC_1_CTRL_STS_OFF		0x0
#define SRC_1_GMAC_1_CTRL_STS_PHY_INTF_SHIFT	0
#define SRC_1_GMAC_1_CTRL_STS_PHY_INTF_MASK	\
	GENMASK(3, SRC_1_GMAC_1_CTRL_STS_PHY_INTF_SHIFT)

struct s32cc_gpr_nvmem_priv {
	struct device *dev;
	void __iomem *gpr_base;
	struct nvmem_config *config;
	struct nvmem_device *nvmem;
};

static int s32cc_gpr_nvmem_write_reg32(void __iomem *addr, u32 shift, u32 mask,
				       u32 value)
{
	u32 reg;

	reg = ioread32(addr);
	reg = (reg & ~mask) | ((value << shift) & mask);
	iowrite32(reg, addr);

	return 0;
}

static int s32cc_gpr_nvmem_read(void *context, unsigned int offset,
				void *val, size_t bytes)
{
	return -EOPNOTSUPP;
}

static int s32cc_gpr_nvmem_write(void *context, unsigned int offset,
				 void *val, size_t bytes)
{
	struct s32cc_gpr_nvmem_priv *priv = context;
	u32 value, shift, mask;
	void __iomem *addr;

	if (bytes != S32CC_GPR_CELL_SIZE)
		return -EINVAL;

	value = *(u32 *)val;

	dev_dbg(priv->dev, "offset: %u, value: %u, bytes: %lu\n", offset,
		value, bytes);

	if (offset == S32CC_GPR_DDR_PMU_IRQ_OFFSET) {
		/* PERF_CNT field is only 1-bit long inside CONFIG_0 reg */
		value = !!(value);
		addr = priv->gpr_base + DDR_GPR_OFF + DDR_GPR_CONFIG_0_OFF;
		shift = DDR_GPR_CONFIG_0_PERF_CNT_SHIFT;
		mask = DDR_GPR_CONFIG_0_PERF_CNT_MASK;

		return s32cc_gpr_nvmem_write_reg32(addr, shift, mask, value);
	}

	if (offset == S32CC_GPR_GMAC0_PHY_INTF_SEL_OFFSET) {
		addr = priv->gpr_base + SRC_0_OFF + SRC_0_GMAC_0_CTRL_STS_OFF;
		shift = SRC_0_GMAC_0_CTRL_STS_SHIFT;
		mask = SRC_0_GMAC_0_CTRL_STS_MASK;

		return s32cc_gpr_nvmem_write_reg32(addr, shift, mask, value);
	}

	if (offset == S32R45_GPR_GMAC1_PHY_INTF_SEL_OFFSET) {
		addr = priv->gpr_base + SRC_1_OFF + SRC_1_GMAC_1_CTRL_STS_OFF;
		shift = SRC_1_GMAC_1_CTRL_STS_PHY_INTF_SHIFT;
		mask = SRC_1_GMAC_1_CTRL_STS_PHY_INTF_MASK;

		return s32cc_gpr_nvmem_write_reg32(addr, shift, mask, value);
	}

	return -EINVAL;
}

static struct nvmem_config s32cc_gpr_nvmem_config = {
	.name = "s32cc-gpr-nvmem",
	.read_only = false,
	.word_size = 1,
	.size = 4,
	.reg_read = s32cc_gpr_nvmem_read,
	.reg_write = s32cc_gpr_nvmem_write,
};

static int s32cc_gpr_nvmem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s32cc_gpr_nvmem_priv *priv;
	struct resource *res;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Cannot obtain S32CC GPR resource.\n");
		return -ENODEV;
	}

	priv->gpr_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->gpr_base)) {
		dev_err(dev, "Cannot map S32CC GPR address space.\n");
		return PTR_ERR(priv->gpr_base);
	}

	s32cc_gpr_nvmem_config.dev = dev;
	s32cc_gpr_nvmem_config.priv = priv;

	priv->dev = dev;
	priv->config = &s32cc_gpr_nvmem_config;
	priv->nvmem = devm_nvmem_register(dev, &s32cc_gpr_nvmem_config);

	return PTR_ERR_OR_ZERO(priv->nvmem);
}

static const struct of_device_id s32cc_gpr_nvmem_match[] = {
	{ .compatible = "nxp,s32cc-gpr-nvmem", },
	{ /* sentinel */ },
};

static struct platform_driver s32cc_gpr_nvmem_driver = {
	.probe = s32cc_gpr_nvmem_probe,
	.driver = {
		.name = "s32cc-gpr-nvmem",
		.of_match_table = s32cc_gpr_nvmem_match,
	},
};
module_platform_driver(s32cc_gpr_nvmem_driver);
MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("S32CC GPR NVMEM Provider Driver");
MODULE_LICENSE("GPL v2");
