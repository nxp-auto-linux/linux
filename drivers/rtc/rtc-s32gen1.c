// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019-2022 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/rtc.h>
#include <linux/pm_wakeup.h>
#include <linux/clk.h>

#include "rtc-s32gen1.h"

#define DRIVER_NAME		"rtc_s32gen1"
#define DRIVER_VERSION		"0.1"
#define ENABLE_WAKEUP		1

#define ROLLOVER_VAL	0xFFFFFFFFULL

struct rtc_time_base {
	unsigned long sec;
	u64 cycles;
	u64 rollovers;
#ifdef CONFIG_PM_SLEEP
	struct rtc_time tm;
#endif
};

/**
 * struct rtc_s32gen1_priv - RTC driver private data
 * @rtc_base: rtc base address
 * @dt_irq_id: rtc interrupt id
 * @rtc_s32gen1_kobj: sysfs kernel object
 * @rtc_s32gen1_attr: sysfs command attributes
 * @pdev: platform device structure
 * @div512: enable DIV512 frequency divider
 * @div32: enable DIV32 frequency divider
 * @clk_source: one of S32GEN1_RTC_SOURCE_* input clocks
 * @rtc_hz: current frequency of the timer
 * @rollovers: number of counter rollovers
 * @base: time baseline in cycles + seconds
 * @firc: reference to FIRC clock
 * @sirc: reference to SIRC clock
 * @ipg: reference to clock that powers the registers
 */
struct rtc_s32gen1_priv {
	u8 __iomem *rtc_base;
	unsigned int dt_irq_id;
	struct kobject *rtc_s32gen1_kobj;
	struct kobj_attribute rtc_s32gen1_attr;
	struct platform_device *pdev;
	struct rtc_device *rdev;
	bool div512;
	bool div32;
	u8 clk_source;
	unsigned long rtc_hz;
	u64 rollovers;
	struct rtc_time_base base;
	struct clk *firc;
	struct clk *sirc;
	struct clk *ipg;
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

static u64 cycles_to_sec(const struct rtc_s32gen1_priv *priv, u64 cycles)
{
	return cycles / priv->rtc_hz;
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
	if (!seconds || seconds > cycles_to_sec(priv, ULONG_MAX))
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
	u32 status;

	status = ioread32(priv->rtc_base + RTCS_OFFSET);

	if (status & RTCS_ROVRF)
		priv->rollovers++;

	if (status & RTCS_RTCF) {
		/* Disable the trigger */
		iowrite32(0x0, priv->rtc_base + RTCVAL_OFFSET);
		rtc_update_irq(priv->rdev, 1, RTC_AF);
	}

	if (status & RTCS_APIF)
		rtc_update_irq(priv->rdev, 1, RTC_PF);

	/* Clear the IRQ */
	iowrite32(status, priv->rtc_base + RTCS_OFFSET);

	return IRQ_HANDLED;
}

static int __s32gen1_rtc_read_time(struct rtc_s32gen1_priv *priv,
				   struct rtc_time *tm)
{
	u32 rtccnt = ioread32(priv->rtc_base + RTCCNT_OFFSET);
	u64 cycles, sec, base_cycles;

	if (!tm)
		return -EINVAL;

	cycles = priv->rollovers * ROLLOVER_VAL + rtccnt;
	base_cycles = priv->base.cycles + priv->base.rollovers * ROLLOVER_VAL;

	if (cycles < base_cycles)
		return -EINVAL;

	/* Subtract time base */
	cycles -= base_cycles;
	sec = priv->base.sec + cycles_to_sec(priv, cycles);
	rtc_time64_to_tm(sec, tm);

	return 0;
}

static int s32gen1_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);

	return __s32gen1_rtc_read_time(priv, tm);
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

	/**
	 * RTCIE cannot be deasserted because it will also disable the
	 * rollover interrupt.
	 */
	rtcc_val = ioread32(priv->rtc_base + RTCC_OFFSET);
	if (enabled)
		rtcc_val |= RTCC_RTCIE;

	iowrite32(rtcc_val, priv->rtc_base + RTCC_OFFSET);

	return 0;
}

static int s32gen1_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned long t_crt, t_alrm;
	struct rtc_time time_crt;
	u32 rtcval;
	int err = 0;
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);

	/* Disable the trigger */
	iowrite32(0x0, priv->rtc_base + RTCVAL_OFFSET);

	t_alrm = rtc_tm_to_time64(&alrm->time);

	/* Assuming the alarm is being set relative to the same time
	 * returned by our .rtc_read_time callback
	 */
	err = s32gen1_rtc_read_time(dev, &time_crt);
	if (err)
		return err;

	t_crt = rtc_tm_to_time64(&time_crt);
	if (t_alrm <= t_crt) {
		dev_warn(dev, "Alarm is set in the past\n");
		return -EINVAL;
	}

	err = s32gen1_sec_to_rtcval(priv, t_alrm - t_crt, &rtcval);
	if (err) {
		dev_warn(dev, "Alarm too far in the future\n");
		goto err_sec_to_rtcval;
	}

	/* Synchronization period */
	while (ioread32(priv->rtc_base + RTCS_OFFSET) & RTCS_INV_RTC)
		;

	iowrite32(rtcval, priv->rtc_base + RTCVAL_OFFSET);

err_sec_to_rtcval:
	return err;
}

static int __s32gen1_rtc_set_time(struct rtc_s32gen1_priv *priv,
				  struct rtc_time *time)
{
	u32 rtccnt = ioread32(priv->rtc_base + RTCCNT_OFFSET);

	if (!time)
		return -EINVAL;

	priv->base.rollovers = priv->rollovers;
	priv->base.cycles = rtccnt;
	priv->base.sec = rtc_tm_to_time64(time);

	return 0;
}

static int s32gen1_rtc_set_time(struct device *dev, struct rtc_time *time)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);

	return __s32gen1_rtc_set_time(priv, time);
}

static const struct rtc_class_ops s32gen1_rtc_ops = {
	.read_time = s32gen1_rtc_read_time,
	.set_time = s32gen1_rtc_set_time,
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

/* RTC specific initializations
 * Note: This function will leave the clock disabled. This means APIVAL and
 *       RTCVAL will need to be configured (again) *after* this call.
 */
static int s32gen1_rtc_init(struct rtc_s32gen1_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	struct clk *sclk;
	u32 rtcc = 0;
	u32 clksel;
	int err;

	err = clk_prepare_enable(priv->ipg);
	if (err) {
		dev_err(dev, "Can't enable 'ipg' clock\n");
		return err;
	}

	err = clk_prepare_enable(priv->sirc);
	if (err) {
		dev_err(dev, "Can't enable 'sirc' clock\n");
		return err;
	}

	err = clk_prepare_enable(priv->firc);
	if (err) {
		dev_err(dev, "Can't enable 'firc' clock\n");
		return err;
	}

	/* Make sure the clock is disabled before we configure dividers */
	s32gen1_rtc_disable(priv);

	priv->rtc_hz = 0;

	clksel = RTCC_CLKSEL(priv->clk_source);
	rtcc |= clksel;

	/* Precompute the base frequency of the clock */
	switch (clksel) {
	case RTCC_CLKSEL(S32GEN1_RTC_SOURCE_SIRC):
		sclk = priv->sirc;
		break;
	case RTCC_CLKSEL(S32GEN1_RTC_SOURCE_FIRC):
		sclk = priv->firc;
		break;
	default:
		dev_err(dev, "Invalid clksel value: %u\n", clksel);
		return -EINVAL;
	}

	priv->rtc_hz = clk_get_rate(sclk);
	if (!priv->rtc_hz) {
		dev_err(dev, "Invalid RTC frequency\n");
		return -EINVAL;
	}

	/* Adjust frequency if dividers are enabled */
	if (priv->div512) {
		rtcc |= RTCC_DIV512EN;
		priv->rtc_hz /= 512;
	}
	if (priv->div32) {
		rtcc |= RTCC_DIV32EN;
		priv->rtc_hz /= 32;
	}

	rtcc |= RTCC_RTCIE | RTCC_ROVREN;
	iowrite32(rtcc, priv->rtc_base + RTCC_OFFSET);

	return 0;
}

/* Initialize priv members with values from the device-tree */
static int s32g_priv_dts_init(struct platform_device *pdev,
			      struct rtc_s32gen1_priv *priv)
{
	struct device_node *np;
	struct device *dev = &pdev->dev;
	u32 div[2];	/* div512 and div32 */
	u32 clksel;

	priv->sirc = devm_clk_get(dev, "sirc");
	if (IS_ERR(priv->sirc)) {
		dev_err(dev, "Failed to get 'sirc' clock\n");
		return -EINVAL;
	}

	priv->firc = devm_clk_get(dev, "firc");
	if (IS_ERR(priv->firc)) {
		dev_err(dev, "Failed to get 'firc' clock\n");
		return -EINVAL;
	}

	priv->ipg = devm_clk_get(dev, "ipg");
	if (IS_ERR(priv->ipg)) {
		dev_err(dev, "Failed to get 'ipg' clock\n");
		return -EINVAL;
	}

	priv->dt_irq_id = platform_get_irq(pdev, 0);
	if (priv->dt_irq_id <= 0) {
		dev_err(dev, "Error reading interrupt # from dts\n");
		return -EINVAL;
	}

	np = dev_of_node(dev);

	if (of_property_read_u32_array(np, "nxp,dividers", div,
				       ARRAY_SIZE(div))) {
		dev_err(dev, "Error reading dividers configuration\n");
		return -EINVAL;
	}
	priv->div512 = !!div[0];
	priv->div32 = !!div[1];

	if (of_property_read_u32(np, "nxp,clksel", &clksel)) {
		dev_err(dev, "Error reading clksel configuration\n");
		return -EINVAL;
	}

	switch (clksel) {
	case S32GEN1_RTC_SOURCE_SIRC:
	case S32GEN1_RTC_SOURCE_FIRC:
		priv->clk_source = clksel;
		break;
	default:
		dev_err(dev, "Unsupported clksel: %d\n", clksel);
		return -EINVAL;
	}

	return 0;
}

static int s32gen1_rtc_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct rtc_s32gen1_priv *priv = NULL;
	int err = 0;

	dev = &pdev->dev;

	/* alloc and initialize private data struct */
	priv = devm_kzalloc(dev, sizeof(struct rtc_s32gen1_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->rtc_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->rtc_base)) {
		dev_err(dev, "Failed to map registers\n");
		return PTR_ERR(priv->rtc_base);
	}

	dev_dbg(dev, "RTC successfully mapped to 0x%p\n",
		priv->rtc_base);

	err = device_init_wakeup(dev, ENABLE_WAKEUP);
	if (err) {
		dev_err(dev, "device_init_wakeup err %d\n", err);
		return -ENXIO;
	}

	if (s32g_priv_dts_init(pdev, priv))
		return -EINVAL;

	if (s32gen1_rtc_init(priv))
		return -EINVAL;

	priv->pdev = pdev;
	platform_set_drvdata(pdev, priv);
	s32gen1_rtc_enable(priv);

	err = devm_request_irq(dev, priv->dt_irq_id,
			       s32gen1_rtc_handler, 0, "rtc", pdev);
	if (err) {
		dev_err(&pdev->dev, "Request interrupt %d failed\n",
			priv->dt_irq_id);
		return -ENXIO;
	}

	print_rtc(pdev);

	priv->rdev = devm_rtc_device_register(dev, "s32gen1_rtc",
					      &s32gen1_rtc_ops, THIS_MODULE);
	if (IS_ERR_OR_NULL(priv->rdev)) {
		dev_err(dev, "devm_rtc_device_register error %ld\n",
			PTR_ERR(priv->rdev));
		return -ENXIO;
	}

	return 0;
}

static int s32gen1_rtc_remove(struct platform_device *pdev)
{
	u32 rtcc_val;
	struct rtc_s32gen1_priv *priv = platform_get_drvdata(pdev);

	rtcc_val = ioread32(priv->rtc_base + RTCC_OFFSET);
	iowrite32(rtcc_val & (~RTCC_CNTEN), priv->rtc_base + RTCC_OFFSET);

	dev_info(&pdev->dev, "Removed successfully\n");
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void s32gen1_enable_api_irq(struct device *dev, unsigned int enabled)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);
	u32 api_irq = RTCC_APIEN | RTCC_APIIE;
	u32 rtcc_val;

	rtcc_val = ioread32(priv->rtc_base + RTCC_OFFSET);
	if (enabled)
		rtcc_val |= api_irq;
	else
		rtcc_val &= ~api_irq;
	iowrite32(rtcc_val, priv->rtc_base + RTCC_OFFSET);
}

static int get_time_left(struct device *dev, struct rtc_s32gen1_priv *priv,
			 u32 *sec)
{
	u32 rtccnt = ioread32(priv->rtc_base + RTCCNT_OFFSET);
	u32 rtcval = ioread32(priv->rtc_base + RTCVAL_OFFSET);

	if (rtcval < rtccnt) {
		dev_err(dev, "RTC timer expired before entering in suspend\n");
		return -EIO;
	}

	*sec = cycles_to_sec(priv, rtcval - rtccnt);
	return 0;
}

static int adjust_dividers(u32 sec, struct rtc_s32gen1_priv *priv)
{
	u64 rtcval_max = (u32)(-1);
	u64 rtcval;

	priv->div32 = 0;
	priv->div512 = 0;

	rtcval = sec * priv->rtc_hz;
	if (rtcval < rtcval_max)
		return 0;

	if (rtcval / 32 < rtcval_max) {
		priv->div32 = 1;
		return 0;
	}

	if (rtcval / 512 < rtcval_max) {
		priv->div512 = 1;
		return 0;
	}

	if (rtcval / (512 * 32) < rtcval_max) {
		priv->div32 = 1;
		priv->div512 = 1;
		return 0;
	}

	return -EINVAL;
}

static int s32gen1_rtc_prepare_suspend(struct device *dev)
{
	struct rtc_s32gen1_priv *init_priv = dev_get_drvdata(dev);
	struct rtc_s32gen1_priv priv;
	int ret = 0;
	u32 rtcval;
	u32 sec;
	unsigned long base_sec;

	/*
	 * Use a local copy of the RTC control block to
	 * avoid restoring it on resume path.
	 */
	memcpy(&priv, init_priv, sizeof(priv));

	if (priv.clk_source == S32GEN1_RTC_SOURCE_SIRC)
		return 0;

	/* Switch to SIRC */
	priv.clk_source = S32GEN1_RTC_SOURCE_SIRC;

	ret = get_time_left(dev, init_priv, &sec);
	if (ret)
		goto enable_rtc;

	/* Adjust for the number of seconds we'll be asleep */
	base_sec = rtc_tm_to_time64(&init_priv->base.tm);
	base_sec += sec;
	rtc_time64_to_tm(base_sec, &init_priv->base.tm);

	s32gen1_rtc_disable(&priv);

	ret = adjust_dividers(sec, &priv);
	if (ret) {
		dev_err(dev, "Failed to adjust RTC dividers to match a %u seconds delay\n", sec);
		goto enable_rtc;
	}


	ret = s32gen1_rtc_init(&priv);
	if (ret)
		goto enable_rtc;

	ret = s32gen1_sec_to_rtcval(&priv, sec, &rtcval);
	if (ret) {
		dev_warn(dev, "Alarm too far in the future\n");
		goto enable_rtc;
	}

	s32gen1_alarm_irq_enable(dev, 0);
	s32gen1_enable_api_irq(dev, 1);
	iowrite32(rtcval, priv.rtc_base + APIVAL_OFFSET);
	iowrite32(0, priv.rtc_base + RTCVAL_OFFSET);

enable_rtc:
	s32gen1_rtc_enable(&priv);
	return ret;
}

static int s32gen1_rtc_suspend(struct device *dev)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);

	if (!device_may_wakeup(dev))
		return 0;

	/* Save last known timestamp before we switch clocks and reinit RTC */
	if (__s32gen1_rtc_read_time(priv, &priv->base.tm))
		return -EINVAL;

	return s32gen1_rtc_prepare_suspend(dev);
}

static int s32gen1_rtc_resume(struct device *dev)
{
	struct rtc_s32gen1_priv *priv = dev_get_drvdata(dev);
	int ret;

	if (!device_may_wakeup(dev))
		return 0;

	/* Disable wake-up interrupts */
	s32gen1_enable_api_irq(dev, 0);

	/* Reinitialize the driver using the initial settings */
	ret = s32gen1_rtc_init(priv);

	s32gen1_rtc_enable(priv);
	/* Now RTCCNT has just been reset, and is out of sync with priv->base;
	 * reapply the saved time settings
	 */
	if (__s32gen1_rtc_set_time(priv, &priv->base.tm))
		return -EINVAL;

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id s32gen1_rtc_of_match[] = {
	{.compatible = "fsl,s32gen1-rtc" },
	{ /* sentinel */ },
};

static SIMPLE_DEV_PM_OPS(s32gen1_rtc_pm_ops,
			 s32gen1_rtc_suspend, s32gen1_rtc_resume);

static struct platform_driver s32gen1_rtc_driver = {
	.probe		= s32gen1_rtc_probe,
	.remove		= s32gen1_rtc_remove,
	.driver		= {
		.name		= "s32gen1-rtc",
		.pm		= &s32gen1_rtc_pm_ops,
		.of_match_table = of_match_ptr(s32gen1_rtc_of_match),
	},
};
module_platform_driver(s32gen1_rtc_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_DESCRIPTION("RTC driver for S32GEN1");
MODULE_VERSION(DRIVER_VERSION);
