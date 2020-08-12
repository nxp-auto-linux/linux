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

static void __iomem *timer_base;
struct clk *clock;

static ssize_t get_init(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int initialized;
	u32	reg_content;

	reg_content = ioread32be(timer_base + STM_CR);
	initialized = !!(reg_content & STM_CR_TEN);

	return snprintf(buf, INPUT_LEN, "%d\n", initialized);
}

static ssize_t set_init(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	int should_init;

	rc = sscanf(buf, "%du", &should_init);
	if (rc != 1 || (should_init != 0 && should_init != 1)) {
		dev_err(dev, "Wrong input. Should be 1 or 0\n");
		return count;
	}
	if (should_init) {
		iowrite32be(0, timer_base + STM_CNT);
		iowrite32be(STM_CR_CPS | STM_CR_FRZ | STM_CR_TEN,
						timer_base + STM_CR);
	} else {
		iowrite32be(0, timer_base + STM_CR);
	}
	return count;
}

static ssize_t get_time(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	u32 cur_time;

	cur_time = ioread32be(timer_base + STM_CNT);

	return snprintf(buf, OUTPUT_LEN, "0x%08x\n", cur_time);
}


static const struct device_attribute dev_attrs[] = {
	__ATTR(init,	0660, get_init,	set_init),
	__ATTR(value,	0440, get_time,	NULL)
};

static int init_interface(struct device *dev)
{
	int rc, i, j;

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

static int global_timer_probe(struct platform_device *pdev)
{
	struct resource *res;
	int rc;

	clock = devm_clk_get(&pdev->dev, "stm");
	if (IS_ERR(clock)) {
		dev_err(&pdev->dev, "Failed getting clock from dtb\n");
		return PTR_ERR(clock);
	}
	rc = clk_prepare_enable(clock);
	if (rc) {
		dev_err(&pdev->dev, "Failed initializing clock\n");
		return rc;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed getting timer address from dtb.\n");
		clk_disable_unprepare(clock);
		return -ENODEV;
	}
	timer_base = ioremap(res->start, res->end - res->start);
	if (!timer_base) {
		dev_err(&pdev->dev, "Couldn't remap timer base address\n");
		clk_disable_unprepare(clock);
		return -ENOMEM;
	}

	rc = init_interface(&pdev->dev);
	if (rc) {
		dev_err(&pdev->dev, "Failed initiating sysfs interface\n");
		iounmap(timer_base);
		clk_disable_unprepare(clock);
	}

	return rc;
}

static int global_timer_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dev_attrs); i++)
		device_remove_file(&pdev->dev, &dev_attrs[i]);
	iounmap(timer_base);
	clk_disable_unprepare(clock);

	return 0;
}

static const struct of_device_id global_timer_dt_ids[] = {
	{
		.compatible = "fsl,s32gen1-stm-global",
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


static int __init global_time_init(void)
{
	int ret;

	ret = platform_driver_register(&global_time_driver);
	if (ret) {
		pr_err("%s Problem registering global timing drive\n",
				DRIVER_NAME);
	}

	return ret;
}

static void __exit global_time_exit(void)
{
	platform_driver_unregister(&global_time_driver);
}

module_init(global_time_init);
module_exit(global_time_exit);

MODULE_DESCRIPTION("NXP SoC Level Time Source for Gen1");
MODULE_LICENSE("GPL v2");
