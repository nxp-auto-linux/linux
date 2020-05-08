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
#include <linux/init.h>
#include <linux/delay.h>
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
 * @div512: enable DIV512 frequency divider
 * @div32: enable DIV32 frequency divider
 * @clk_source: one of S32GEN1_RTC_SOURCE_* input clocks
 * @rtc_hz: current frequency of the timer
 */
struct rtc_s32gen1_priv {
	u8 __iomem *rtc_base;
	unsigned int dt_irq_id;
	struct resource *res;
	struct kobject *rtc_s32gen1_kobj;
	struct kobj_attribute rtc_s32gen1_attr;
	struct platform_device *pdev;
	struct rtc_device *rdev;
	bool div512;
	bool div32;
	u8 clk_source;
	u32 rtc_hz;
};

static void print_rtc(struct platform_device *pdev)
{
	struct rtc_s32gen1_priv *priv = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "RTCSUPV = 0x%08x\n",
		ioread32(priv->rtc_base + RTCSUPV_OFFSET));
	dev_dbg(&pdev->dev, "RTCC = 0x%08x\n",
		ioread32(priv->rtc_base + RTCC_OFFSET));
	dev_dbg(&pdev->dev, "RTCS = 0x%08x\n",
		ioread32(priv->rtc_base + RTCS_OFFSET));
	dev_dbg(&pdev->dev, "RTCCNT = 0x%08x\n",
		ioread32(priv->rtc_base + RTCCNT_OFFSET));
	dev_dbg(&pdev->dev, "APIVAL = 0x%08x\n",
		ioread32(priv->rtc_base + APIVAL_OFFSET));
	dev_dbg(&pdev->dev, "RTCVAL = 0x%08x\n",
		ioread32(priv->rtc_base + RTCVAL_OFFSET));
}

/* Convert a number of seconds to a value suitable for RTCVAL in our clock's
 * current configuration.
 * @rtcval: The value to go into RTCVAL[RTCVAL]
 * Returns: 0 for success, -EINVAL if @seconds push the counter at least
 *          twice the rollover interval
 */
static int s32gen1_sec_to_rtcval(const struct rtc_s32gen1_priv *priv,
				 unsigned long seconds, u32 *rtcval)
{
	u32 rtccnt, delta_cnt;
	u32 target_cnt = 0;

	/* For now, support at most one roll-over of the counter */
	if (!seconds || seconds > ULONG_MAX / priv->rtc_hz)
		return -EINVAL;

	/* RTCCNT is read-only; we must return a value relative to the
	 * current value of the counter (and hope we don't linger around
	 * too much before we get to enable the interrupt)
	 */
	delta_cnt = seconds * priv->rtc_hz;
	rtccnt = ioread32(priv->rtc_base + RTCCNT_OFFSET);
	/* ~rtccnt just stands for (ULONG_MAX - rtccnt) */
	if (~rtccnt < delta_cnt)
		target_cnt = (delta_cnt - ~rtccnt);
	else
		target_cnt = rtccnt + delta_cnt;

	/* RTCVAL must be at least random() :-) */
	if (unlikely(target_cnt < 4))
		target_cnt = 4;

	*rtcval = target_cnt;
	return 0;
}

static irqreturn_t s32gen1_rtc_handler(int irq, void *dev)
{
	struct rtc_s32gen1_priv *priv = platform_get_drvdata(dev);

	/* Clear the IRQ */
	iowrite32(RTCS_RTCF, priv->rtc_base + RTCS_OFFSET);

	return IRQ_HANDLED;
}

static int s32gen1_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	/* Dummy reading so we appease rtc_valid_tm(); note that this means
	 * we won't have a monotonic timestamp, in case someone wants to use
	 * this RTC as the system timer.
	 */
	static struct rtc_time stm = {
		.tm_year = 118,	/* 2018 */
		.tm_mon = 7,	/* August */
		.tm_mday = 10,
		.tm_hour = 18,
	};

	if (!tm)
		return -EINVAL;
	*tm = stm;

	return 0;
}

static int s32gen1_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	/* For the moment, leave this callback empty as it is here to shun a
	 * run-time warning from rtcwake.
	 */
	return 0;
}

static int s32gen1_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);
	u32 rtcc_val;

	if (!priv->dt_irq_id)
		return -EIO;

	rtcc_val = ioread32(priv->rtc_base + RTCC_OFFSET);
	if (enabled)
		rtcc_val |= RTCC_RTCIE;
	else
		rtcc_val &= ~RTCC_RTCIE;
	iowrite32(rtcc_val, priv->rtc_base + RTCC_OFFSET);

	return 0;
}

static int s32gen1_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned long t_crt, t_alrm;
	struct rtc_time time_crt;
	u32 rtcval;
	int err = 0, ret;
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);

	/* Assuming the alarm is being set relative to the same time
	 * returned by our .rtc_read_time callback
	 */
	err = s32gen1_rtc_read_time(dev, &time_crt);
	if (err)
		return err;
	rtc_tm_to_time(&time_crt, &t_crt);
	rtc_tm_to_time(&alrm->time, &t_alrm);
	if (t_alrm <= t_crt) {
		dev_warn(dev, "Alarm is set in the past\n");
		return -EINVAL;
	}

	/* RTCVAL can only be written either: a) after reset, or
	 * b) when CNTEN is set. If the latter, we'd want to know interrupts
	 * are disabled first.
	 */
	if (s32gen1_alarm_irq_enable(dev, 0)) {
		dev_warn(dev, "Error disabling IRQ\n");
		return -EIO;
	}
	err = s32gen1_sec_to_rtcval(priv, t_alrm - t_crt, &rtcval);
	if (err) {
		dev_warn(dev, "Alarm too far in the future\n");
		goto err_sec_to_rtcval;
	}
	iowrite32(rtcval, priv->rtc_base + RTCVAL_OFFSET);

err_sec_to_rtcval:
	ret = s32gen1_alarm_irq_enable(dev, !!alrm->enabled);
	return err ? err : ret;
}

static const struct rtc_class_ops s32gen1_rtc_ops = {
	.read_time = s32gen1_rtc_read_time,
	.read_alarm = s32gen1_rtc_read_alarm,
	.set_alarm = s32gen1_rtc_set_alarm,
	.alarm_irq_enable = s32gen1_alarm_irq_enable,
};

static void s32gen1_rtc_disable(struct rtc_s32gen1_priv *priv)
{
	u32 rtcc = ioread32(priv->rtc_base + RTCC_OFFSET);

	rtcc &= ~RTCC_CNTEN;
	iowrite32(rtcc, priv->rtc_base + RTCC_OFFSET);
}

static void s32gen1_rtc_enable(struct rtc_s32gen1_priv *priv)
{
	u32 rtcc = ioread32(priv->rtc_base + RTCC_OFFSET);

	rtcc |= RTCC_CNTEN;
	iowrite32(rtcc, priv->rtc_base + RTCC_OFFSET);
}

static int s32gen1_get_firc_hz(struct platform_device *pdev,
			       u32 *clock_frequency)
{
	struct device_node *clock_node;

	if (!clock_frequency)
		return -EINVAL;

	clock_node = of_find_node_by_name(NULL, "firc");
	if (!clock_node) {
		dev_err(&pdev->dev, "Unable to find the clock node\n");
		return -ENXIO;
	}

	if (of_property_read_u32(clock_node,
		"clock-frequency", clock_frequency)) {
		dev_err(&pdev->dev, "Unable to read the clock frequency\n");
		return -EINVAL;
	}

	return 0;
}

static int s32gen1_get_sirc_hz(struct platform_device *pdev,
			       u32 *clock_frequency)
{
	struct device_node *clock_node;

	if (!clock_frequency)
		return -EINVAL;

	clock_node = of_find_node_by_name(NULL, "sirc");
	if (!clock_node) {
		dev_err(&pdev->dev, "Unable to find the clock node\n");
		return -ENXIO;
	}

	if (of_property_read_u32(clock_node,
		"clock-frequency", clock_frequency)) {
		dev_err(&pdev->dev, "Unable to read the clock frequency\n");
		return -EINVAL;
	}

	return 0;
}

/* RTC specific initializations
 * Note: This function will leave the clock disabled. This means APIVAL and
 *       RTCVAL will need to be configured (again) *after* this call.
 */
static int s32gen1_rtc_init(struct rtc_s32gen1_priv *priv)
{
	u32 rtcc = 0;
	u32 clksel;
	int err;

	/* Make sure the clock is disabled before we configure dividers */
	s32gen1_rtc_disable(priv);

	clksel = RTCC_CLKSEL(priv->clk_source);
	rtcc |= clksel;

	/* Precompute the base frequency of the clock */
	switch (clksel) {
	case RTCC_CLKSEL(S32GEN1_RTC_SOURCE_SIRC):
		err = s32gen1_get_sirc_hz(priv->pdev, &priv->rtc_hz);
		if (err)
			return -EINVAL;
		break;
	case RTCC_CLKSEL(S32GEN1_RTC_SOURCE_FIRC):
		err = s32gen1_get_firc_hz(priv->pdev, &priv->rtc_hz);
		if (err)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	if (!priv->rtc_hz)
		return -EINVAL;

	/* Adjust frequency if dividers are enabled */
	if (priv->div512) {
		rtcc |= RTCC_DIV512EN;
		priv->rtc_hz /= 512;
	}
	if (priv->div32) {
		rtcc |= RTCC_DIV32EN;
		priv->rtc_hz /= 32;
	}

	rtcc |= RTCC_RTCIE;
	iowrite32(rtcc, priv->rtc_base + RTCC_OFFSET);

	return 0;
}

/* Initialize priv members with values from the device-tree */
static int s32g_priv_dts_init(const struct platform_device *pdev,
			      struct rtc_s32gen1_priv *priv)
{
	struct device_node *rtc_node;
	u32 div[2];	/* div512 and div32 */
	u32 clksel;

	rtc_node = of_find_compatible_node(NULL, NULL, "fsl,s32gen1-rtc");
	if (!rtc_node) {
		dev_err(&pdev->dev, "Unable to find RTC node\n");
		return -ENXIO;
	}

	priv->dt_irq_id = of_irq_get(rtc_node, 0);
	of_node_put(rtc_node);
	if (priv->dt_irq_id <= 0) {
		dev_err(&pdev->dev, "Error reading interrupt # from dts\n");
		return -EINVAL;
	}

	if (of_property_read_u32_array(rtc_node, "dividers", div,
				       ARRAY_SIZE(div))) {
		dev_err(&pdev->dev, "Error reading dividers configuration\n");
		return -EINVAL;
	}
	priv->div512 = !!div[0];
	priv->div32 = !!div[1];

	if (of_property_read_u32(rtc_node, "clksel", &clksel)) {
		dev_err(&pdev->dev, "Error reading clksel configuration\n");
		return -EINVAL;
	}
	switch (clksel) {
	case S32GEN1_RTC_SOURCE_SIRC:
	case S32GEN1_RTC_SOURCE_FIRC:
		priv->clk_source = clksel;
		break;
	default:
		dev_err(&pdev->dev, "Unsupported clksel: %d\n", clksel);
		return -EINVAL;
	}

	return 0;
}

static int s32gen1_rtc_probe(struct platform_device *pdev)
{
	struct rtc_s32gen1_priv *priv = NULL;
	int err = 0;

	dev_dbg(&pdev->dev, "Probing platform device: %s\n", pdev->name);

	/* alloc and initialize private data struct */
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

	priv->rtc_base = devm_ioremap_nocache(&pdev->dev,
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

	if (s32g_priv_dts_init(pdev, priv)) {
		err = -EINVAL;
		goto err_dts_init;
	}

	if (s32gen1_rtc_init(priv)) {
		err = -EINVAL;
		goto err_rtc_init;
	}

	priv->pdev = pdev;
	platform_set_drvdata(pdev, priv);
	s32gen1_rtc_enable(priv);

	err = devm_request_irq(&pdev->dev, priv->dt_irq_id,
			       s32gen1_rtc_handler, 0, "rtc", pdev);
	if (err) {
		dev_err(&pdev->dev, "Request interrupt %d failed\n",
			priv->dt_irq_id);
		err = -ENXIO;
		goto err_devm_request_irq;
	}

	print_rtc(pdev);

	return 0;

err_devm_request_irq:
err_rtc_init:
err_dts_init:
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
	iowrite32(rtcc_val & (~RTCC_CNTEN), priv->rtc_base + RTCC_OFFSET);

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
	.driver		= {.name = "s32gen1-rtc",
			   .of_match_table = of_match_ptr(s32gen1_rtc_of_match),
			  },
};
module_platform_driver(s32gen1_rtc_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_DESCRIPTION("RTC driver for S32GEN1");
MODULE_VERSION(DRIVER_VERSION);
