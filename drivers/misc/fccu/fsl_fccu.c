// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * FCCU driver for S32 SoC
 *
 * Copyright 2018 NXP.
 *
 * Drives the Fault Collection and Control Unit.
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
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/of.h>

#define DRIVER_NAME "fsl_fccu"
#define FCCU_MINOR	1

#define FCCU_CTRL			0x0
#define FCCU_CTRLK			0x4
#define FCCU_CFG			0x8
#define FCCU_STAT			0xC0
#define FCCU_CFG_TO			0xB8
#define FCCU_TRANS_LOCK		0xF0
#define FCCU_NCFE2			0x9C
#define FCCU_NCFS_CFG4		0x5C
#define FCCU_NCFS_CFG5		0x60
#define FCCU_NCF_S2			0x88
#define FCCU_NCFK			0x90

#define FCCU_CTRL_OPS_MASK			0xC0
#define FCCU_CTRL_OPS_IDLE			0x0
#define FCCU_CTRL_OPS_INPROGRESS	0x40
#define FCCU_CTRL_OPS_ABORTED		0x80
#define FCCU_CTRL_OPS_SUCCESS		0xC0

#define FCCU_CTRL_OPR_OP0			0x0
#define FCCU_CRTL_OPR_OP1			0x1
#define FCCU_CTRL_OPR_OP2			0x2
#define FCCU_CTRL_OPR_OP3			0x3

#define FCCU_STAT_STATUS_MASK		0x7
#define FCCU_STAT_STATUS_NORMAL		0x0
#define FCCU_STAT_STATUS_CONFIG		0x1
#define FCCU_STAT_STATUS_ALARM		0x2
#define FCCU_STAT_STATUS_FAULT		0x3

#define FCCU_TL_TRANSKEY_UNLOCK		0xBC

#define FCCU_CTRLK_UNLOCK_OP1		0x913756AF
#define FCCU_CTRLK_UNLOCK_OP2		0x825A132B

#define FCCU_NCFK_UNLOCK_SEQUENCE	0xAB3498FE

#define FCCU_TIME_OUT				0x7UL

#define FCCU_CFG_DEFAULT			0x8

#define DIFF_MODE					1
#define EQUAL_MODE					0

static struct miscdevice fccu_miscdev = {
	.minor = FCCU_MINOR,
	.name = "fccu",
};

struct fccu_pri_data_t {
	struct resource *res;
	void __iomem *base;
};

static void fccu_wait_and_get_ops(void __iomem *base, u32 *ops_val_out)
{
	u32 reg = 0;

	while (true) {
		reg = __raw_readl(base + FCCU_CTRL);
		if ((reg & FCCU_CTRL_OPS_MASK) != FCCU_CTRL_OPS_INPROGRESS)
			break;
	}

	if (ops_val_out != NULL)
		*ops_val_out = reg & FCCU_CTRL_OPS_MASK;
}

static int wait_fccu_op_success(void __iomem *base)
{
	u32 ops_val = 0;

	fccu_wait_and_get_ops(base, &ops_val);
	if (ops_val != FCCU_CTRL_OPS_SUCCESS)
		return -1;

	return 0;
}

static int wait_fccu_become_normal_st(void __iomem *base)
{
	u32 ops_val = 0;
	u32 status = 0;

	if (IS_ERR_OR_NULL(base)) {
		dev_err(fccu_miscdev.parent,
			"%s, %d, invalid argument\n", __func__, __LINE__);
		return -EINVAL;
	}

	fccu_wait_and_get_ops(base, &ops_val);

	__raw_writel(FCCU_CTRL_OPR_OP3, base + FCCU_CTRL);

	if (wait_fccu_op_success(base) != 0)
		return -1;

	status = __raw_readl(base + FCCU_STAT) & FCCU_STAT_STATUS_MASK;
	if (status != FCCU_STAT_STATUS_NORMAL)
		return -1;

	return 0;
}

static int clear_fault_status(void __iomem *base)
{
	if (IS_ERR_OR_NULL(base)) {
		dev_err(fccu_miscdev.parent,
			"%s, %d, invalid argument\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* Unlock FCCU */
	__raw_writel(FCCU_TL_TRANSKEY_UNLOCK, base + FCCU_TRANS_LOCK);

	/* Unlock FCCU fault status source register */
	__raw_writel(FCCU_NCFK_UNLOCK_SEQUENCE, base + FCCU_NCFK);

	/* Clear previous reset status from SWT0, SWT1, SWT2, SWT3 */
	__raw_writel(0xFFFFFFFF, base + FCCU_NCF_S2);

	if (wait_fccu_op_success(base) != 0)
		return -1;

	return 0;
}

static int enable_swt_rs_chanel(struct device *dev, void __iomem *base)
{
	unsigned int *reg_val_arr = NULL;
	unsigned int *reg_off_arr = NULL;
	size_t size = 0;
	int i = 0;
	int ret = -1;
	struct property *prop = NULL;
	struct device_node *np = NULL;

	if (IS_ERR_OR_NULL(base) || IS_ERR_OR_NULL(dev)) {
		dev_err(fccu_miscdev.parent,
			"%s, %d, invalid argument\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (wait_fccu_become_normal_st(base) != 0)
		return ret;

	np = dev->of_node;
	prop = of_find_property(np, "cfg_reg_off", NULL);

	if (prop == NULL) {
		dev_err(fccu_miscdev.parent,
			"%s, %d, can not find property\n", __func__, __LINE__);
		return ret;
	}

	size = prop->length / sizeof(*reg_off_arr);

	reg_off_arr = devm_kcalloc(dev,
		size, sizeof(*reg_off_arr), GFP_KERNEL);
	if (reg_off_arr == NULL)
		return ret;

	reg_val_arr = devm_kcalloc(dev,
		size, sizeof(*reg_val_arr), GFP_KERNEL);
	if (reg_val_arr == NULL) {
		devm_kfree(dev, reg_off_arr);
		return ret;
	}

	of_property_read_u32_array(np,
		"cfg_reg_off", reg_off_arr, size);
	of_property_read_u32_array(np,
		"cfg_reg_val", reg_val_arr, size);

	/* Set time out for configuration */
	__raw_writel(FCCU_TIME_OUT, base + FCCU_CFG_TO);

	/* Put FCCC into configuration state */
	__raw_writel(FCCU_TL_TRANSKEY_UNLOCK, base + FCCU_TRANS_LOCK);
	__raw_writel(FCCU_CTRLK_UNLOCK_OP1, base + FCCU_CTRLK);
	__raw_writel(FCCU_CRTL_OPR_OP1, base + FCCU_CTRL);

	if (wait_fccu_op_success(base) != 0)
		goto out;

	/* Setup reset channel for FCCU */
	for (i = 0; i < size; i++)
		__raw_writel(reg_val_arr[i], base + reg_off_arr[i]);

	/* Put FCCU into normal state */
	__raw_writel(FCCU_CTRLK_UNLOCK_OP2, base + FCCU_CTRLK);
	__raw_writel(FCCU_CTRL_OPR_OP2, base + FCCU_CTRL);

	if (wait_fccu_op_success(base) != 0)
		goto out;

	ret = 0;

out:
	devm_kfree(dev, reg_off_arr);
	devm_kfree(dev, reg_val_arr);
	return ret;
}

static int __init s32_fccu_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct fccu_pri_data_t *priv_data = NULL;

	priv_data = devm_kzalloc(&pdev->dev,
		sizeof(struct fccu_pri_data_t), GFP_KERNEL);
	if (priv_data == NULL)
		return -ENOMEM;

	fccu_miscdev.parent = &pdev->dev;
	priv_data->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv_data->base = devm_ioremap_resource(&pdev->dev, priv_data->res);

	if (IS_ERR_OR_NULL(priv_data->base)) {
		dev_err(fccu_miscdev.parent,
			"%s, %d, invalid argument\n", __func__, __LINE__);
		release_resource(priv_data->res);
		devm_kfree(&pdev->dev, priv_data);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, &priv_data);

	if (clear_fault_status(priv_data->base) != 0) {
		dev_err(fccu_miscdev.parent, "%s, %d, configuration meet timeout\n",
			__func__, __LINE__);
		devm_kfree(&pdev->dev, priv_data);
		return ret;
	}

	if (enable_swt_rs_chanel(&pdev->dev, priv_data->base) != 0) {
		dev_err(fccu_miscdev.parent, "%s, %d, configuration meet timeout\n",
			__func__, __LINE__);
		devm_kfree(&pdev->dev, priv_data);
		return ret;
	}

	ret = misc_register(&fccu_miscdev);

	return ret;
}

static int __exit s32_fccu_remove(struct platform_device *pdev)
{
	struct fccu_pri_data_t *priv_data = NULL;

	priv_data = platform_get_drvdata(pdev);
	if (IS_ERR_OR_NULL(priv_data->base)) {
		dev_err(fccu_miscdev.parent,
			"%s, %d, invalid argument\n", __func__, __LINE__);
		return -EINVAL;
	}

	clear_fault_status(priv_data->base);
	release_resource(priv_data->res);
	devm_kfree(&pdev->dev, priv_data);

	return 0;
}

static const struct of_device_id s32_fccu_dt_ids[] = {
	{.compatible = "fsl,s32v234-fccu",},
	{.compatible = "fsl,s32gen1-fccu",},
	{ /* sentinel */ }
};

static struct platform_driver s32_fccu_driver = {
	.remove = __exit_p(s32_fccu_remove),
	.driver = {
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table = s32_fccu_dt_ids,
			},
};

module_platform_driver_probe(s32_fccu_driver,
	s32_fccu_probe);

MODULE_AUTHOR("Phu Luu An");
MODULE_DESCRIPTION("FCCU driver for S32 SoC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_MISCDEV(FCCU_MINOR);
MODULE_ALIAS("platform:" DRIVER_NAME);
