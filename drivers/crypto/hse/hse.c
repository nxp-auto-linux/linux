// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>

#define HSE_MU_INST    "mu" __stringify(CONFIG_CRYPTO_DEV_NXP_HSE_MU_ID) "b"
#define HSE_RX_IRQ     "hse-" HSE_MU_INST "-rx"
#define HSE_ERR_IRQ    "hse-" HSE_MU_INST "-err"

/**
 * struct hse_drvdata - HSE driver private data
 * @mu_base: HSE MU base virtual address
 */
struct hse_drvdata {
	void __iomem *mu_base;
};

static irqreturn_t hse_rx_handler(int irq, void *hse_dev)
{
	return IRQ_HANDLED;
}

static irqreturn_t hse_err_handler(int irq, void *hse_dev)
{
	return IRQ_HANDLED;
}

static int hse_probe(struct platform_device *pdev)
{
	struct device_node *mu_node;
	struct hse_drvdata *priv;
	int rx_irq, err_irq, err;
	struct resource mu_reg;

	dev_info(&pdev->dev, "probing HSE device %s\n", pdev->name);

	priv = devm_kzalloc(&pdev->dev, sizeof(struct hse_drvdata), GFP_KERNEL);
	if (IS_ERR_OR_NULL(priv))
		return -ENOMEM;

	mu_node = of_get_child_by_name(pdev->dev.of_node, HSE_MU_INST);
	if (unlikely(!mu_node))
		return -ENODEV;
	if (unlikely(!of_device_is_available(mu_node))) {
		err = -ENODEV;
		goto err_node_put;
	}

	err = of_address_to_resource(mu_node, 0, &mu_reg);
	if (unlikely(err)) {
		err = -EFAULT;
		goto err_node_put;
	}

	rx_irq = of_irq_get_byname(mu_node, HSE_RX_IRQ);
	if (unlikely(rx_irq <= 0)) {
		err = -ENXIO;
		goto err_node_put;
	}

	err_irq = of_irq_get_byname(mu_node, HSE_ERR_IRQ);
	if (unlikely(err_irq <= 0)) {
		err = -ENXIO;
		goto err_node_put;
	}

	of_node_put(mu_node);

	priv->mu_base = devm_ioremap_resource(&pdev->dev, &mu_reg);
	if (IS_ERR_OR_NULL(priv->mu_base)) {
		dev_err(&pdev->dev, "map addr 0x%08llx failed\n", mu_reg.start);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, priv);

	err = devm_request_irq(&pdev->dev, rx_irq, hse_rx_handler, 0,
			       HSE_RX_IRQ, pdev);
	if (unlikely(err)) {
		dev_err(&pdev->dev, "register irq %d failed\n", rx_irq);
		return -ENXIO;
	}

	err = devm_request_irq(&pdev->dev, err_irq, hse_err_handler, 0,
			       HSE_ERR_IRQ, pdev);
	if (unlikely(err)) {
		dev_err(&pdev->dev, "register irq %d failed\n", err_irq);
		return -ENXIO;
	}

	dev_info(&pdev->dev, "HSE device %s initialized\n", pdev->name);

	return 0;

err_node_put:
	of_node_put(mu_node);
	return err;
}

static int hse_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "HSE device %s removed", pdev->name);

	return 0;
}

static const struct of_device_id hse_of_match[] = {
	{
		.compatible = "fsl,s32gen1-hse"
	}, {}
};
MODULE_DEVICE_TABLE(of, hse_of_match);

static struct platform_driver hse_driver = {
	.driver = {
		.name = "nxp_hse",
		.of_match_table	= hse_of_match,
	},
	.probe = hse_probe,
	.remove = hse_remove,
};

module_platform_driver(hse_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("nxp_hse");
MODULE_DESCRIPTION("NXP Hardware Security Engine (HSE) Driver");
