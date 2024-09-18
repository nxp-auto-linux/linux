// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2024 NXP
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include "ufshcd-pltfrm.h"
#include "ufshcd-dwc.h"

static int ufs_s32n79_phy_initialization(struct ufs_hba *hba)
{
	/*
	 * TODO: will be implemented when the PHY initialization
	 * sequence is available.
	 */
	return 0;
}

static struct ufs_hba_variant_ops ufs_hba_s32n79_vops = {
	.name = "s32n79",
	.link_startup_notify = ufshcd_dwc_link_startup_notify,
	.phy_initialization = ufs_s32n79_phy_initialization,
};

static const struct of_device_id ufs_s32n79_match[] = {
	{
		.compatible = "nxp,s32n79-ufshc",
		.data = &ufs_hba_s32n79_vops,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ufs_s32n79_match);

static int ufs_s32n79_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;

	of_id = of_match_node(ufs_s32n79_match, pdev->dev.of_node);

	return ufshcd_pltfrm_init(pdev, of_id->data);
}

static void ufs_s32n79_remove(struct platform_device *pdev)
{
	struct ufs_hba *hba =  platform_get_drvdata(pdev);

	pm_runtime_get_sync(&(pdev)->dev);
	ufshcd_remove(hba);
}

static const struct dev_pm_ops ufs_s32n79_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ufshcd_system_suspend, ufshcd_system_resume)
	SET_RUNTIME_PM_OPS(ufshcd_runtime_suspend, ufshcd_runtime_resume, NULL)
};

static struct platform_driver ufs_s32n79_driver = {
	.probe = ufs_s32n79_probe,
	.remove_new = ufs_s32n79_remove,
	.driver	= {
		.name	= "ufs-s32n79",
		.pm	= &ufs_s32n79_pm_ops,
		.of_match_table	= ufs_s32n79_match,
	},
};

module_platform_driver(ufs_s32n79_driver);

MODULE_AUTHOR("Larisa Grigore <larisa.grigore@nxp.com>");
MODULE_DESCRIPTION("NXP S32N79 UFS Host Controller platform driver");
MODULE_LICENSE("GPL");
