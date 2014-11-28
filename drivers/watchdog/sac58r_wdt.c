// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Watchdog driver for SAC58R SoC
 *
 *  Copyright (C) 2014 Freescale Semiconductor, Inc.
 *  Copyright 2017-2019 NXP.
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
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#define DRIVER_NAME "sac58r-wdt"

#define SAC58R_SWT_CR		0x00	/* Control Register */
#define SAC58R_SWT_CR_FIXED_SS	(0 << 9)	/* -> Fixed Service Sequence */
#define SAC58R_SWT_CR_KEYED_SS	(1 << 9)	/* -> Keyed Service Sequence */
#define SAC58R_SWT_CR_RIA		(1 << 8)	/* -> Reset on Invalid Access */
#define SAC58R_SWT_CR_WND		(1 << 7)	/* -> Window Mode */
#define SAC58R_SWT_CR_ITR		(1 << 6)	/* -> Interrupt then reset */
#define SAC58R_SWT_CR_HLK		(1 << 5)	/* -> Hard Lock */
#define SAC58R_SWT_CR_SLK		(1 << 4)	/* -> Soft Lock */
#define SAC58R_SWT_CR_STP		(1 << 2)	/* -> Stop Mode Control */
#define SAC58R_SWT_CR_FRZ		(1 << 1)	/* -> Debug Mode Control */
#define SAC58R_SWT_CR_WEN		(1 << 0)	/* -> Watchdog Enable */

#define SAC58R_SWT_IR		0x04	/* Interrupt Register */

#define SAC58R_SWT_TO		0x08	/* Timeout Register */

#define SAC58R_SWT_SR		0x10	/* Service Register */
#define SAC58R_WDT_SEQ1		0xA602	/* -> service sequence 1 */
#define SAC58R_WDT_SEQ2		0xB480	/* -> service sequence 2 */

#define SAC58R_SWT_SK		0x18	/* Service Key Register */

#define SAC58R_WDT_TO_MAX_VALUE	0xFFFFFFFF
#define SAC58R_WDT_DEFAULT_TIME	30	/* in seconds */
#define SAC58R_SHUTDOWN_TIME		5	/* in seconds */
#define SAC58R_WDT_TO_MIN_COUNT	0x100

#define SAC58R_WDT_STATUS_OPEN	0
#define SAC58R_WDT_STATUS_STARTED	1
#define SAC58R_WDT_EXPECT_CLOSE	2

static struct {
	struct clk *clk;
	void __iomem *base;
	unsigned int timeout;
	unsigned long status;
	struct timer_list timer;	/* Pings the watchdog when closed */
	unsigned long maxtime;
} sac58r_wdt;

static struct miscdevice sac58r_wdt_miscdev;

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int timeout = SAC58R_WDT_DEFAULT_TIME;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default="
		 __MODULE_STRING(SAC58R_WDT_DEFAULT_TIME) ")");

static const struct watchdog_info sac58r_wdt_info = {
	.identity = "sac58r watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
};

static inline int WDOG_SEC_TO_COUNT(int timeout)
{
	int to = (clk_get_rate(sac58r_wdt.clk) * timeout);

	if (to < SAC58R_WDT_TO_MIN_COUNT)
		to = SAC58R_WDT_TO_MIN_COUNT;

	return to;
}

static inline void sac58r_wdt_setup(void)
{
	u32 val = 0;

	/* Set the watchdog's Time-Out value */
	val = WDOG_SEC_TO_COUNT(sac58r_wdt.timeout);
	__raw_writel(val, sac58r_wdt.base + SAC58R_SWT_TO);

	/* Configure Timer */
	val = __raw_readl(sac58r_wdt.base + SAC58R_SWT_CR);

	/* Allows watchdog timer to be stopped when device enters debug mode
	 * or when device is in stopped mode
	 */

	val |= SAC58R_SWT_CR_STP | SAC58R_SWT_CR_FRZ;
	/* Use Fixed Service Sequence to ping the watchdog */
	val |= SAC58R_SWT_CR_FIXED_SS;
	/* Enable the watchdog */
	val |= SAC58R_SWT_CR_WEN;

	__raw_writel(val, sac58r_wdt.base + SAC58R_SWT_CR);
}

static inline void sac58r_wdt_ping(void)
{
	__raw_writel(SAC58R_WDT_SEQ1, sac58r_wdt.base + SAC58R_SWT_SR);
	__raw_writel(SAC58R_WDT_SEQ2, sac58r_wdt.base + SAC58R_SWT_SR);
}

static void sac58r_wdt_timer_ping(struct timer_list *t)
{
	/* ping it every sac58r_wdt.timeout / 2 seconds to prevent reboot */
	sac58r_wdt_ping();
	mod_timer(&sac58r_wdt.timer, jiffies + sac58r_wdt.timeout * HZ / 2);
}

static void sac58r_wdt_start(void)
{
	if (!test_and_set_bit(SAC58R_WDT_STATUS_STARTED, &sac58r_wdt.status)) {
		/* at our first start we enable clock and do initialisations */
		clk_prepare_enable(sac58r_wdt.clk);

		sac58r_wdt_setup();
	} else			/* delete the timer that pings the watchdog after close */
		del_timer_sync(&sac58r_wdt.timer);

	/* Watchdog is enabled - time to reload the timeout value */
	sac58r_wdt_ping();
}

static void sac58r_wdt_stop(void)
{
	/* we don't need a clk_disable, it cannot be disabled once started.
	 * We use a timer to ping the watchdog while /dev/watchdog is closed
	 */
	sac58r_wdt_timer_ping(0);
}

static void sac58r_wdt_set_timeout(int new_timeout)
{
	u32 cr = 0;
	u32 to = 0;

	/* Disable the watchdog */
	cr = __raw_readl(sac58r_wdt.base + SAC58R_SWT_CR);
	cr &= ~SAC58R_SWT_CR_WEN;
	__raw_writel(cr, sac58r_wdt.base + SAC58R_SWT_CR);

	/* set the new timeout value in the TO register */
	to = WDOG_SEC_TO_COUNT(new_timeout);
	__raw_writel(to, sac58r_wdt.base + SAC58R_SWT_TO);

	/* Re-enable the watchdog */
	cr |= SAC58R_SWT_CR_WEN;
	__raw_writel(cr, sac58r_wdt.base + SAC58R_SWT_CR);
}

static int sac58r_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(SAC58R_WDT_STATUS_OPEN, &sac58r_wdt.status))
		return -EBUSY;

	sac58r_wdt_start();
	return nonseekable_open(inode, file);
}

static int sac58r_wdt_close(struct inode *inode, struct file *file)
{
	if (test_bit(SAC58R_WDT_EXPECT_CLOSE, &sac58r_wdt.status) && !nowayout)
		sac58r_wdt_stop();
	else {
		dev_crit(sac58r_wdt_miscdev.parent,
			 "Unexpected close: Expect reboot!\n");
		sac58r_wdt_ping();
	}

	clear_bit(SAC58R_WDT_EXPECT_CLOSE, &sac58r_wdt.status);
	clear_bit(SAC58R_WDT_STATUS_OPEN, &sac58r_wdt.status);
	return 0;
}

static long sac58r_wdt_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &sac58r_wdt_info,
				    sizeof(struct watchdog_info)) ? -EFAULT : 0;

	case WDIOC_GETSTATUS:
		return put_user(0, p);

	case WDIOC_GETBOOTSTATUS:
		/* Not supported now */
		new_value = 0;
		return put_user(new_value, p);

	case WDIOC_KEEPALIVE:
		sac58r_wdt_ping();
		return 0;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;
		if ((new_value < 1) || (new_value > sac58r_wdt.maxtime))
			return -EINVAL;
		sac58r_wdt_set_timeout(new_value);
		sac58r_wdt.timeout = new_value;
		sac58r_wdt_ping();
		fallthrough;

	case WDIOC_GETTIMEOUT:
		return put_user(sac58r_wdt.timeout, p);

	default:
		return -ENOTTY;
	}
}

static ssize_t sac58r_wdt_write(struct file *file, const char __user *data,
				size_t len, loff_t *ppos)
{
	size_t i;
	char c;

	if (len == 0)		/* Can we see this even ? */
		return 0;

	clear_bit(SAC58R_WDT_EXPECT_CLOSE, &sac58r_wdt.status);
	/* scan to see whether or not we got the magic character */
	for (i = 0; i != len; i++) {
		if (get_user(c, data + i))
			return -EFAULT;
		if (c == 'V')
			set_bit(SAC58R_WDT_EXPECT_CLOSE, &sac58r_wdt.status);
	}

	sac58r_wdt_ping();
	return len;
}

static const struct file_operations sac58r_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = sac58r_wdt_ioctl,
	.open = sac58r_wdt_open,
	.release = sac58r_wdt_close,
	.write = sac58r_wdt_write,
};

static struct miscdevice sac58r_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &sac58r_wdt_fops,
};

static int __init sac58r_wdt_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	unsigned long clk_rate;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sac58r_wdt.base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sac58r_wdt.base))
		return PTR_ERR(sac58r_wdt.base);

	sac58r_wdt.clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(sac58r_wdt.clk)) {
		dev_err(&pdev->dev, "can't get Watchdog clock\n");
		return PTR_ERR(sac58r_wdt.clk);
	}

	clk_rate = clk_get_rate(sac58r_wdt.clk);
	if (!clk_rate) {
		dev_err(&pdev->dev, "Input clock rate is not valid\n");
		return -EINVAL;
	}

	sac58r_wdt.maxtime = SAC58R_WDT_TO_MAX_VALUE / clk_rate;

	sac58r_wdt.timeout = clamp_t(unsigned int, timeout, 1, sac58r_wdt.maxtime);
	if (sac58r_wdt.timeout != timeout)
		dev_warn(&pdev->dev,
			 "Initial timeout out of range! Clamped from %u to %u\n",
			 timeout, sac58r_wdt.timeout);

	timer_setup(&sac58r_wdt.timer, sac58r_wdt_timer_ping, 0);

	sac58r_wdt_miscdev.parent = &pdev->dev;
	ret = misc_register(&sac58r_wdt_miscdev);
	if (ret)
		goto fail;

	dev_info(&pdev->dev,
		 "SAC58R Watchdog Timer Registered. timeout=%ds (nowayout=%d)\n",
		 sac58r_wdt.timeout, nowayout);
	return 0;

 fail:
	sac58r_wdt_miscdev.parent = NULL;
	clk_put(sac58r_wdt.clk);
	return ret;
}

static int __exit sac58r_wdt_remove(struct platform_device *pdev)
{
	misc_deregister(&sac58r_wdt_miscdev);

	if (test_bit(SAC58R_WDT_STATUS_STARTED, &sac58r_wdt.status)) {
		del_timer_sync(&sac58r_wdt.timer);

		dev_crit(sac58r_wdt_miscdev.parent,
			 "Device removed: Expect reboot!\n");
	} else
		clk_put(sac58r_wdt.clk);

	sac58r_wdt_miscdev.parent = NULL;
	return 0;
}

static void sac58r_wdt_shutdown(struct platform_device *pdev)
{
	if (test_bit(SAC58R_WDT_STATUS_STARTED, &sac58r_wdt.status)) {
		/* we are running, we need to delete the timer but will give
		 * sometime before reboot will take place
		 */
		del_timer_sync(&sac58r_wdt.timer);
		sac58r_wdt_set_timeout(SAC58R_SHUTDOWN_TIME);
		sac58r_wdt_ping();

		dev_crit(sac58r_wdt_miscdev.parent,
			 "Device shutdown: Expect reboot!\n");
	}
}

static const struct of_device_id sac58r_wdt_dt_ids[] = {
	{.compatible = "fsl,sac58r-wdt",},
	{ /* sentinel */ }
};

static struct platform_driver sac58r_wdt_driver = {
	.remove = __exit_p(sac58r_wdt_remove),
	.shutdown = sac58r_wdt_shutdown,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = sac58r_wdt_dt_ids,
		   },
};

module_platform_driver_probe(sac58r_wdt_driver, sac58r_wdt_probe);

MODULE_AUTHOR("Gilles Talis");
MODULE_DESCRIPTION("Watchdog driver for SAC58R SoC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:" DRIVER_NAME);
