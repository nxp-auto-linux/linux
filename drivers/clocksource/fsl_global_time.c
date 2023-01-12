// SPDX-License-Identifier: GPL-2.0-or-later
/* Copyright 2020 NXP
 *
 * Driver which provides universal timestamp at SoC level for S32Gen1.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/clk.h>

#define STM_CR				0x00
#define STM_CNT				0x04
#define STM_CR_CPS_OFFSET	8
#define STM_CR_CPS_MASK		0xFF
#define STM_CR_CPS			(STM_CR_CPS_MASK << STM_CR_CPS_OFFSET)
#define STM_CR_FRZ			BIT(1)
#define STM_CR_TEN			BIT(0)
#define DRIVER_NAME			"NXP S32GEN1 Univesal Time Source"
#define INPUT_LEN			2
#define OUTPUT_LEN			11

struct stm_driver {
	void __iomem *base;
};

static ssize_t get_init(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stm_driver *drv = dev_get_drvdata(dev);
	int initialized;
	u32 reg_content;

	reg_content = readl(drv->base + STM_CR);
	initialized = !!(reg_content & STM_CR_TEN);

	return snprintf(buf, INPUT_LEN, "%d\n", initialized);
}

static ssize_t set_init(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct stm_driver *drv = dev_get_drvdata(dev);
	int rc;
	int should_init;

	rc = sscanf(buf, "%du", &should_init);
	if (rc != 1 || (should_init != 0 && should_init != 1)) {
		dev_err(dev, "Wrong input. Should be 1 or 0\n");
		return count;
	}
	if (should_init) {
		writel(0, drv->base + STM_CNT);
		writel(STM_CR_CPS | STM_CR_FRZ | STM_CR_TEN,
		       drv->base + STM_CR);
	} else {
		writel(0, drv->base + STM_CR);
	}
	return count;
}

static ssize_t get_time(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stm_driver *drv = dev_get_drvdata(dev);
	u32 cur_time;

	cur_time = readl(drv->base + STM_CNT);

	return snprintf(buf, OUTPUT_LEN, "0x%08x\n", cur_time);
}

static const struct device_attribute dev_attrs[] = {
	__ATTR(init,	0660, get_init,	set_init),
	__ATTR(value,	0440, get_time,	NULL)
};

static int init_sysfs_interface(struct device *dev)
{
	size_t i, j;
	int rc;

	for (i = 0; i < ARRAY_SIZE(dev_attrs); i++) {
		rc = device_create_file(dev, &dev_attrs[i]);
		if (rc) {
			for (j = i; j >= 0; j--)
				device_remove_file(dev, &dev_attrs[i]);
			return rc;
		}
	}

	return rc;
}

static void deinit_sysfs_interface(struct device *dev)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(dev_attrs); i++)
		device_remove_file(dev, &dev_attrs[i]);
}

static int devm_clk_prepare_enable(struct device *dev, struct clk *clk)
{
	int ret;

	ret = clk_prepare_enable(clk);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev,
					(void(*)(void *))clk_disable_unprepare,
					clk);
}

static int global_timer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm_driver *drv;
	struct clk *clock;
	int rc;

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	clock = devm_clk_get(dev, "stm");
	if (IS_ERR(clock)) {
		dev_err(dev, "Failed getting clock from dtb\n");
		return PTR_ERR(clock);
	}

	rc = devm_clk_prepare_enable(dev, clock);
	if (rc) {
		dev_err(dev, "Failed initializing clock\n");
		return rc;
	}

	drv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(drv->base)) {
		dev_err(dev, "Couldn't remap timer base address\n");
		return PTR_ERR(drv->base);
	}

	rc = init_sysfs_interface(dev);
	if (rc) {
		dev_err(dev, "Failed initiating sysfs interface\n");
	}

	platform_set_drvdata(pdev, drv);

	return rc;
}

static int global_timer_remove(struct platform_device *pdev)
{
	deinit_sysfs_interface(&pdev->dev);

	return 0;
}

static const struct of_device_id global_timer_dt_ids[] = {
	{
		.compatible = "nxp,s32cc-stm-global",
	},
	{ /* sentinel */ }
};

static struct platform_driver global_time_driver = {
	.probe	= global_timer_probe,
	.remove	= global_timer_remove,
	.driver	= {
		.name			= DRIVER_NAME,
		.owner			= THIS_MODULE,
		.of_match_table = global_timer_dt_ids,
	},
};
module_platform_driver(global_time_driver)

MODULE_DESCRIPTION("NXP SoC Level Time Source for Gen1");
MODULE_LICENSE("GPL v2");
