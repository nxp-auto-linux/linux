// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/current.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/rtc.h>

#include "rtc-s32gen1.h"

#define DRIVER_NAME		"rtc_s32gen1"
#define DRIVER_VERSION		"0.1"
#define ENABLE_WAKEUP		1

/**
 * struct rtc_s32gen1_priv - RTC driver private data
 * @rtc_base: rtc base address
 * @dt_irq_id: rtc interrupt id
 * @res: rtc resource
 * @rtc_s32gen1_kobj: sysfs kernel object
 * @rtc_s32gen1_attr: sysfs command attributes
 * @pdev: platform device structure
 */
struct rtc_s32gen1_priv {
	u8 __iomem *rtc_base;
	unsigned int dt_irq_id;
	struct resource *res;
	struct kobject *rtc_s32gen1_kobj;
	struct kobj_attribute rtc_s32gen1_attr;
	struct platform_device *pdev;
	struct rtc_device *rdev;
};

static void print_rtc(struct platform_device *pdev)
{
	struct rtc_s32gen1_priv *priv = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "[notify] RTCSUPV = 0x%08x\n",
		ioread32(priv->rtc_base + RTCSUPV_OFFSET));
	dev_dbg(&pdev->dev, "[notify] RTCC = 0x%08x\n",
		ioread32(priv->rtc_base + RTCC_OFFSET));
	dev_dbg(&pdev->dev, "[notify] RTCS = 0x%08x\n",
		ioread32(priv->rtc_base + RTCS_OFFSET));
	dev_dbg(&pdev->dev, "[notify] RTCCNT = 0x%08x\n",
		ioread32(priv->rtc_base + RTCCNT_OFFSET));
	dev_dbg(&pdev->dev, "[notify] APIVAL = 0x%08x\n",
		ioread32(priv->rtc_base + APIVAL_OFFSET));
	dev_dbg(&pdev->dev, "[notify] RTCVAL = 0x%08x\n",
		ioread32(priv->rtc_base + RTCVAL_OFFSET));
}

static irqreturn_t rtc_handler(int irq, void *dev)
{
	struct rtc_s32gen1_priv *priv = platform_get_drvdata(dev);

	/* Clear the IRQ */
	iowrite32(RTCF, priv->rtc_base + RTCS_OFFSET);

	return IRQ_HANDLED;
}

static int s32gen1_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);

	return 0;
}

static int s32gen1_rtc_set_time(struct device *dev, struct rtc_time *t)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);

	return 0;
}

static int s32gen1_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);

	return 0;
}

static int s32gen1_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);

	return 0;
}

static int s32gen1_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);
	u32 rtcc_val;

	if (!priv->dt_irq_id)
		return -EIO;

	rtcc_val = ioread32(priv->rtc_base + RTCC_OFFSET);
	iowrite32(rtcc_val | CNTEN | RTCIE, priv->rtc_base + RTCC_OFFSET);

	rtcc_val = ioread32(priv->rtc_base + RTCC_OFFSET);

	return (rtcc_val & RTCIE) ? 0 : -EFAULT;
}

static const struct rtc_class_ops s32gen1_rtc_ops = {
	.read_time = s32gen1_rtc_read_time,
	.set_time = s32gen1_rtc_set_time,
	.read_alarm = s32gen1_rtc_read_alarm,
	.set_alarm = s32gen1_rtc_set_alarm,
	.alarm_irq_enable = s32gen1_alarm_irq_enable,
};

static int s32gen1_rtc_probe(struct platform_device *pdev)
{
	struct rtc_s32gen1_priv *priv = NULL;
	struct device_node *rtc_node;
	int err = 0;

	dev_dbg(&pdev->dev, "Probing platform device: %s\n", pdev->name);

	/* alloc private data struct */
	priv = devm_kzalloc(&pdev->dev, sizeof(struct rtc_s32gen1_priv),
		GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!priv->res) {
		dev_err(&pdev->dev, "Failed to get resource");
		err = -ENOMEM;
		goto err_platform_get_resource;
	}

	priv->rtc_base = devm_ioremap(&pdev->dev,
		priv->res->start, (priv->res->end - priv->res->start));
	if (!priv->rtc_base) {
		dev_err(&pdev->dev, "Failed to map IO address 0x%016llx\n",
			priv->res->start);
		err = -ENOMEM;
		goto err_ioremap_nocache;
	}
	dev_dbg(&pdev->dev, "RTC successfully mapped to 0x%p\n",
		priv->rtc_base);

	priv->rdev = devm_rtc_device_register(&pdev->dev, "s32gen1_rtc",
					&s32gen1_rtc_ops, THIS_MODULE);
	if (IS_ERR_OR_NULL(priv->rdev)) {
		dev_err(&pdev->dev, "devm_rtc_device_register error %ld\n",
			PTR_ERR(priv->rdev));
		err = -ENXIO;
		goto err_devm_rtc_device_register;
	}

	err = device_init_wakeup(&pdev->dev, ENABLE_WAKEUP);
	if (err) {
		dev_err(&pdev->dev, "device_init_wakeup err %d\n", err);
		err = -ENXIO;
		goto err_device_init_wakeup;
	}

	rtc_node = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-rtc");
	if (!rtc_node) {
		dev_err(&pdev->dev, "Unable to find RTC node\n");
		err = -ENXIO;
		goto err_of_find_compatible;
	}
	priv->dt_irq_id = of_irq_get(rtc_node, 0);
	of_node_put(rtc_node);

	if (priv->dt_irq_id <= 0) {
		err = -ENXIO;
		goto err_of_irq_get;
	}

	platform_set_drvdata(pdev, priv);

	err = devm_request_irq(&pdev->dev, priv->dt_irq_id, rtc_handler, 0,
		"rtc", pdev);
	if (err) {
		dev_err(&pdev->dev, "Request interrupt %d failed\n",
			priv->dt_irq_id);
		err = -ENXIO;
		goto err_devm_request_irq;
	}

	priv->pdev = pdev;
	print_rtc(pdev);

	return 0;

err_devm_request_irq:
err_of_irq_get:
err_of_find_compatible:
err_device_init_wakeup:
err_devm_rtc_device_register:
err_ioremap_nocache:
	release_resource(priv->res);
err_platform_get_resource:
	return err;
}

static int s32gen1_rtc_remove(struct platform_device *pdev)
{
	u32 rtcc_val;
	struct rtc_s32gen1_priv *priv = platform_get_drvdata(pdev);

	rtcc_val = ioread32(priv->rtc_base + RTCC_OFFSET);
	iowrite32(rtcc_val & (~CNTEN), priv->rtc_base + RTCC_OFFSET);

	release_resource(priv->res);

	dev_info(&pdev->dev, "Removed successfully\n");
	return 0;
}

static const struct of_device_id s32gen1_rtc_of_match[] = {
	{.compatible = "fsl,s32gen1-rtc" },
};

static struct platform_driver s32gen1_rtc_driver = {
	.probe		= s32gen1_rtc_probe,
	.remove		= s32gen1_rtc_remove,
	.driver		= {
		.name	= "s32gen1-rtc",
		.of_match_table	= of_match_ptr(s32gen1_rtc_of_match),
	},
};
module_platform_driver(s32gen1_rtc_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_DESCRIPTION("RTC driver for S32GEN1");
MODULE_VERSION(DRIVER_VERSION);
