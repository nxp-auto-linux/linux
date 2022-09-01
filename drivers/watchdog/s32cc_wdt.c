// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Watchdog driver for S32CC SoC
 *
 *  Copyright (C) 2014 Freescale Semiconductor, Inc.
 *  Copyright 2017-2019, 2021-2022 NXP.
 *
 * Based on imx2_wdt.c
 * Drives the Software Watchdog Timer module
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>

#define DRIVER_NAME "s32cc-wdt"

#define S32CC_SWT_CR			0x00	/* Control Register */
#define S32CC_SWT_CR_FIXED_SS	(0 << 9)	/* -> Fixed Service Sequence */
#define S32CC_SWT_CR_STP		BIT(2)	/* -> Stop Mode Control */
#define S32CC_SWT_CR_FRZ		BIT(1)	/* -> Debug Mode Control */
#define S32CC_SWT_CR_WEN		BIT(0)	/* -> Watchdog Enable */

#define S32CC_SWT_TO		0x08	/* Timeout Register */

#define S32CC_SWT_SR		0x10	/* Service Register */
#define S32CC_WDT_SEQ1		0xA602	/* -> service sequence 1 */
#define S32CC_WDT_SEQ2		0xB480	/* -> service sequence 2 */

#define S32CC_WDT_TO_MAX_VALUE	0xFFFFFFFF
#define S32CC_WDT_DEFAULT_TIME	30
#define S32CC_WDT_TO_MIN_COUNT	0x100

enum wdt_flags {
	S32CC_NO_CONTINUE_IN_STBY,
	S32CC_CONTINUE_IN_STBY,
};

struct s32cc_data {
	enum wdt_flags flags;
};

struct s32cc_wdt_device {
	struct clk *clk;
	void __iomem *base;
	enum wdt_flags flags;
	unsigned long status;
	struct timer_list timer;	/* Pings the watchdog when closed */
	struct watchdog_device wdog;
};

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int timeout = S32CC_WDT_DEFAULT_TIME;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default="
		 __MODULE_STRING(S32CC_WDT_DEFAULT_TIME) ")");

static const struct watchdog_info s32cc_wdt_info = {
	.identity = "s32cc watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE |
	WDIOC_GETTIMEOUT,
};

static unsigned int wdog_sec_to_count(struct s32cc_wdt_device *wdev,
				      unsigned int timeout)
{
	unsigned int to = (unsigned int)(clk_get_rate(wdev->clk) * timeout);

	if (to < S32CC_WDT_TO_MIN_COUNT)
		to = S32CC_WDT_TO_MIN_COUNT;

	return to;
}

static bool s32cc_wdt_is_running(struct s32cc_wdt_device *wdev)
{
	u32 val;

	val = __raw_readl(wdev->base + S32CC_SWT_CR);

	return val & S32CC_SWT_CR_WEN;
}

static int s32cc_wdt_ping(struct watchdog_device *wdog)
{
	struct s32cc_wdt_device *wdev = watchdog_get_drvdata(wdog);

	__raw_writel(S32CC_WDT_SEQ1, wdev->base + S32CC_SWT_SR);
	__raw_writel(S32CC_WDT_SEQ2, wdev->base + S32CC_SWT_SR);

	return 0;
}

static void s32cc_wdt_timer_ping(struct timer_list *t)
{
	struct s32cc_wdt_device *wdev = from_timer(wdev, t, timer);
	struct watchdog_device wdog = wdev->wdog;

	s32cc_wdt_ping(&wdog);
	mod_timer(&wdev->timer,
		  jiffies + wdog.timeout * (unsigned long)HZ / 2);
}

static inline void s32cc_wdt_setup(struct watchdog_device *wdog)
{
	u32 val = 0;
	struct s32cc_wdt_device *wdev = watchdog_get_drvdata(wdog);

	/* Set the watchdog's Time-Out value */
	val = wdog_sec_to_count(wdev, wdog->timeout);

	__raw_writel(val, wdev->base + S32CC_SWT_TO);

	/* Configure Timer */
	val = __raw_readl(wdev->base + S32CC_SWT_CR);

	/* Allows watchdog timer to be stopped when device enters debug mode
	 * or when device is in stopped mode
	 */
	if (!(wdev->flags & S32CC_CONTINUE_IN_STBY))
		val |= S32CC_SWT_CR_STP;

	val |= S32CC_SWT_CR_FRZ;
	/* Use Fixed Service Sequence to ping the watchdog */
	val |= S32CC_SWT_CR_FIXED_SS;
	/* Enable the watchdog */
	val |= S32CC_SWT_CR_WEN;

	__raw_writel(val, wdev->base + S32CC_SWT_CR);
}

static int s32cc_wdt_set_timeout(struct watchdog_device *wdog,
				 unsigned int new_timeout)
{
	struct s32cc_wdt_device *wdev = watchdog_get_drvdata(wdog);

	__raw_writel(wdog_sec_to_count(wdev, new_timeout),
		     wdev->base + S32CC_SWT_TO);
	wdog->timeout = clamp_t(unsigned int, new_timeout, 1, wdog->max_timeout);

	return 0;
}

static int s32cc_wdt_start(struct watchdog_device *wdog)
{
	struct s32cc_wdt_device *wdev = watchdog_get_drvdata(wdog);

	if (s32cc_wdt_is_running(wdev)) {
		del_timer_sync(&wdev->timer);
		s32cc_wdt_set_timeout(wdog, wdog->timeout);
	} else {
		s32cc_wdt_setup(wdog);
	}

	return s32cc_wdt_ping(wdog);
}

static int s32cc_wdt_stop(struct watchdog_device *wdog)
{
	struct s32cc_wdt_device *wdev = watchdog_get_drvdata(wdog);

	s32cc_wdt_timer_ping(&wdev->timer);

	return 0;
}

static const struct watchdog_ops s32cc_wdt_ops = {
	.owner = THIS_MODULE,
	.start = s32cc_wdt_start,
	.stop = s32cc_wdt_stop,
	.ping = s32cc_wdt_ping,
	.set_timeout = s32cc_wdt_set_timeout,
};

static int __init s32cc_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res = NULL;
	unsigned long clk_rate = 0;
	struct s32cc_wdt_device *wdev = NULL;
	struct watchdog_device *wdog = NULL;
	const struct s32cc_data *data;

	data = of_device_get_match_data(&pdev->dev);

	wdev = devm_kzalloc(&pdev->dev, sizeof(struct s32cc_wdt_device),
			    GFP_KERNEL);
	if (!wdev)
		return -ENOMEM;

	if (data)
		wdev->flags = data->flags;
	else
		wdev->flags = S32CC_NO_CONTINUE_IN_STBY;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wdev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wdev->base)) {
		dev_err(&pdev->dev, "can not get resource\n");
		return PTR_ERR(wdev->base);
	}

	wdev->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(wdev->clk)) {
		dev_err(&pdev->dev, "can't get Watchdog clock\n");
		return PTR_ERR(wdev->clk);
	}

	clk_rate = clk_get_rate(wdev->clk);
	if (!clk_rate) {
		dev_err(&pdev->dev, "Input clock rate is not valid\n");
		return -EINVAL;
	}

	wdog = &wdev->wdog;
	wdog->info = &s32cc_wdt_info;
	wdog->ops = &s32cc_wdt_ops;
	wdog->min_timeout = 1;
	wdog->max_timeout = (unsigned int)(S32CC_WDT_TO_MAX_VALUE / clk_rate);

	wdog->timeout = clamp_t(unsigned int, timeout, 1, wdog->max_timeout);
	if (wdog->timeout != timeout)
		dev_warn(&pdev->dev, "timeout out of range! Clamped from %u to %u\n",
			 timeout, wdog->timeout);

	timer_setup(&wdev->timer, s32cc_wdt_timer_ping, 0);

	platform_set_drvdata(pdev, wdog);
	watchdog_set_drvdata(wdog, wdev);
	watchdog_set_nowayout(wdog, nowayout);

	ret = watchdog_register_device(wdog);
	if (ret) {
		dev_err(&pdev->dev, "cannot register watchdog device\n");
		return ret;
	}

	dev_info(&pdev->dev,
		 "S32CC Watchdog Timer Registered. timeout=%ds (nowayout=%d)\n",
		 wdog->timeout, nowayout);

	return 0;
}

static int __exit s32cc_wdt_remove(struct platform_device *pdev)
{
	struct watchdog_device *wdog = platform_get_drvdata(pdev);
	struct s32cc_wdt_device *wdev = watchdog_get_drvdata(wdog);

	watchdog_unregister_device(wdog);

	if (s32cc_wdt_is_running(wdev)) {
		del_timer_sync(&wdev->timer);
		s32cc_wdt_ping(wdog);
		dev_crit(&pdev->dev, "Device removed: Expect reboot!\n");
	}

	return 0;
}

static void s32cc_wdt_shutdown(struct platform_device *pdev)
{
	struct watchdog_device *wdog = platform_get_drvdata(pdev);
	struct s32cc_wdt_device *wdev = watchdog_get_drvdata(wdog);

	if (s32cc_wdt_is_running(wdev)) {
		/*
		 * We are running, we need to delete the timer but will
		 * give max timeout before reboot will take place
		 */
		del_timer_sync(&wdev->timer);
		s32cc_wdt_set_timeout(wdog, wdog->max_timeout);
		s32cc_wdt_ping(wdog);
		dev_crit(&pdev->dev, "Device shutdown: Expect reboot!\n");
	}
}

static const struct s32cc_data s32cc_data = {
	.flags = S32CC_CONTINUE_IN_STBY,
};

static const struct of_device_id s32cc_wdt_dt_ids[] = {
	{.compatible = "nxp,s32cc-wdt", .data = &s32cc_data},
	{ /* sentinel */ }
};

static struct platform_driver s32cc_wdt_driver = {
	.remove = __exit_p(s32cc_wdt_remove),
	.shutdown = s32cc_wdt_shutdown,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = s32cc_wdt_dt_ids,
		   },
};

module_platform_driver_probe(s32cc_wdt_driver, s32cc_wdt_probe);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("Watchdog driver for S32CC SoC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
