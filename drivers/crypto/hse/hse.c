// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright 2019 NXP
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>

static int hse_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "probing HSE device %s using MU%d\n", pdev->name,
		 CONFIG_CRYPTO_DEV_NXP_HSE_MU_ID);

	return 0;
}

static int hse_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "removing HSE device %s using MU\n", pdev->name);

	return 0;
}

static const struct of_device_id hse_of_match[] = {
	{
		.compatible = "fsl,s32gen1-hse"
	},
	{}
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
