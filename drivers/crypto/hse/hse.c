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

#include "hse.h"
#include "hse-mu.h"

static int hse_probe(struct platform_device *pdev)
{
	struct hse_drvdata *pdata;
	int err;

	dev_info(&pdev->dev, "probing HSE device %s\n", pdev->name);

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (IS_ERR_OR_NULL(pdata))
		return -ENOMEM;

	platform_set_drvdata(pdev, pdata);

	pdata->mu_inst = hse_mu_init(&pdev->dev);
	if (IS_ERR(pdata->mu_inst))
		return PTR_ERR(pdata->mu_inst);

	err = hse_hash_init(&pdev->dev);
	if (err)
		return err;

	dev_info(&pdev->dev, "HSE device %s initialized\n", pdev->name);

	return 0;
}

static int hse_remove(struct platform_device *pdev)
{
	struct hse_drvdata *pdata = platform_get_drvdata(pdev);

	hse_hash_free(&pdev->dev);
	hse_mu_free(pdata->mu_inst);

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
