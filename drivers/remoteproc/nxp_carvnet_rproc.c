// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2024 NXP
 *
 * CAR-V NET remoteproc driver
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>

static bool auto_boot;
module_param(auto_boot, bool, 0400);
MODULE_PARM_DESC(auto_boot,
		 "Auto-boot the CAR-V-NET remote processor [default=false]");

static int carvnet_rproc_start(struct rproc *rproc)
{
	return 0;
}

static int carvnet_rproc_stop(struct rproc *rproc)
{
	return 0;
}

static void carvnet_rproc_kick(struct rproc *rproc, int vqid)
{
	/* no op */
}

static const struct rproc_ops carvnet_rproc_ops = {
	.start		= carvnet_rproc_start,
	.stop		= carvnet_rproc_stop,
	.kick		= carvnet_rproc_kick,
};

static int carvnet_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	int ret;

	rproc = devm_rproc_alloc(dev, np->name, &carvnet_rproc_ops, NULL, 0);
	if (!rproc)
		return -ENOMEM;

	dev_set_drvdata(dev, rproc);

	rproc->auto_boot = auto_boot;

	ret = devm_rproc_add(dev, rproc);

	return ret;
}

static void carvnet_rproc_remove(struct platform_device *pdev)
{
	/* no op */
}

static const struct of_device_id carvnet_rproc_of_match[] = {
	{ .compatible = "nxp,carvnet-rproc" },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, carvnet_rproc_of_match);

static struct platform_driver carvnet_rproc_driver = {
	.probe = carvnet_rproc_probe,
	.remove_new = carvnet_rproc_remove,
	.driver = {
		.name = "nxp-carvnet",
		.of_match_table = carvnet_rproc_of_match,
	},
};

module_platform_driver(carvnet_rproc_driver);

MODULE_AUTHOR("Jan Petrous <jan.petrous@nxp.com>");
MODULE_DESCRIPTION("NXP S32N7 CAR-V-NET rproc driver");
MODULE_LICENSE("Dual BSD/GPL");
