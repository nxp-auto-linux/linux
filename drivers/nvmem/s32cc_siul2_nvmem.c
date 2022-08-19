// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 */

#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <dt-bindings/nvmem/s32cc-siul2-nvmem.h>
#include <soc/s32cc/revision_defs.h>

/* SoC revision */
#define SIUL2_MIDR1_OFF				(0x00000004)
#define SIUL2_MIDR2_OFF				(0x00000008)

/* SIUL2_MIDR1 masks */
#define SIUL2_MIDR1_MINOR_MASK		(0xF << 0)
#define SIUL2_MIDR1_MAJOR_SHIFT		(4)
#define SIUL2_MIDR1_MAJOR_MASK		(0xF << SIUL2_MIDR1_MAJOR_SHIFT)

/* SIUL2_MIDR2 masks */
#define SIUL2_MIDR2_SERDES_MASK	BIT(15)

struct s32cc_siul2_nvmem_data {
	struct device *dev;
	struct nvmem_device *nvmem;
	void __iomem *siul2;
};

static struct nvmem_config econfig_0 = {
	.name = "s32cc-siul2_0_nvmem",
	.owner = THIS_MODULE,
	.word_size = 4,
	.size = 4,
	.read_only = true,
};

static struct nvmem_config econfig_1 = {
	.name = "s32cc-siul2_1_nvmem",
	.owner = THIS_MODULE,
	.word_size = 4,
	.size = 4,
	.read_only = true,
};

static int s32cc_siul2_0_nvmem_read(void *context, unsigned int offset,
				    void *val, size_t bytes)
{
	struct s32cc_siul2_nvmem_data *priv = context;
	u32 midr1;
	u32 major, minor;

	if (offset != SOC_REVISION_OFFSET || bytes != SOC_REVISION_SIZE)
		return -EOPNOTSUPP;

	midr1 = ioread32(priv->siul2 + SIUL2_MIDR1_OFF);
	major = (midr1 & SIUL2_MIDR1_MAJOR_MASK) >> SIUL2_MIDR1_MAJOR_SHIFT;
	minor = midr1 & SIUL2_MIDR1_MINOR_MASK;

	/* Bytes format: (MAJOR+1).MINOR.0.0 */
	*(u32 *)val = (major + 1) << S32CC_SOC_REV_MAJOR_SHIFT
			| minor << S32CC_SOC_REV_MINOR_SHIFT;

	return 0;
}

static int s32cc_siul2_1_nvmem_read(void *context, unsigned int offset,
				    void *val, size_t bytes)
{
	struct s32cc_siul2_nvmem_data *priv = context;
	u32 midr2;
	u32 serdes;

	if (offset != SERDES_PRESENCE_OFFSET || bytes != SERDES_PRESENCE_SIZE)
		return -EOPNOTSUPP;

	midr2 = ioread32(priv->siul2 + SIUL2_MIDR2_OFF);
	serdes = !!(midr2 & SIUL2_MIDR2_SERDES_MASK);
	*(u32 *)val = serdes;

	return 0;
}

static const struct of_device_id s32cc_siul2_nvmem_match[] = {
	{ .compatible = "nxp,s32cc-siul2_0-nvmem", },
	{ .compatible = "nxp,s32cc-siul2_1-nvmem", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, s32cc_siul2_nvmem_match);

static int s32cc_siul2_nvmem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct s32cc_siul2_nvmem_data *priv;
	struct nvmem_config *econfig = NULL;
	struct resource *res;

	priv = devm_kzalloc(dev, sizeof(struct s32cc_siul2_nvmem_data),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->siul2 = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->siul2)) {
		dev_err(dev, "Cannot map SIUL2 registers.\n");
		return PTR_ERR(priv->siul2);
	}

	priv->dev = dev;
	if (of_device_is_compatible(np, "nxp,s32cc-siul2_0-nvmem")) {
		econfig = &econfig_0;
		econfig->reg_read = s32cc_siul2_0_nvmem_read;
	} else if (of_device_is_compatible(np, "nxp,s32cc-siul2_1-nvmem")) {
		econfig = &econfig_1;
		econfig->reg_read = s32cc_siul2_1_nvmem_read;
	} else {
		return -ENODEV;
	}

	econfig->dev = dev;
	econfig->priv = priv;

	priv->nvmem = devm_nvmem_register(dev, econfig);

	dev_info(&pdev->dev, "Initialized s32cc siul2 nvmem driver\n");

	return PTR_ERR_OR_ZERO(priv->nvmem);
}

static struct platform_driver s32cc_siul2_nvmem_driver = {
	.probe = s32cc_siul2_nvmem_probe,
	.driver = {
		.name = "s32-siul2-nvmem",
		.of_match_table = s32cc_siul2_nvmem_match,
	},
};

module_platform_driver(s32cc_siul2_nvmem_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("S32CC SIUL2 NVMEM driver");
MODULE_LICENSE("GPL");
