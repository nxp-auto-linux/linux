// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * FCCU driver for S32 SoC
 *
 * Copyright 2018,2021 NXP.
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
#include <linux/slab.h>

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

#define FCCU_NCFS_CFGN_OFF			0x4C
#define FCCU_NCF_EN_OFF				0x94
#define FCCU_NCF_SN_OFF				0x80
#define FCCU_NCF_SN_MAX_NUM			0x4
#define FCCU_NCFS_CFGN_MAX_NUM		0x8
#define FCCU_NCFS_CFG_SIZE			16
#define FCCU_NCF_SN(N)				(FCCU_NCF_SN_OFF + 4 * (N))
#define FCCU_NCF_EN(N)				(FCCU_NCF_EN_OFF + 4 * (N))
#define FCCU_NCFS_CFGN(N)			(FCCU_NCFS_CFGN_OFF + 4 * (N))

#define DIFF_MODE					1
#define EQUAL_MODE					0

static struct miscdevice fccu_miscdev = {
	.minor = FCCU_MINOR,
	.name = "fccu",
};

struct fccu_pri_data_t {
	struct resource *res;
	void __iomem *base;
	u32 ncfn_mask[FCCU_NCF_SN_MAX_NUM];
	u32 ncfs_cfgn_mask[FCCU_NCFS_CFGN_MAX_NUM];
};

static int fccu_get_ncf_reg_number(u32 val)
{
	int ncf_num = val / 32;

	return (ncf_num < FCCU_NCF_SN_MAX_NUM) ? ncf_num : -EINVAL;
}

/* Set maximal fault masks for every NCF_Sn */
static void set_maximal_fault_lists(struct device *dev,
		struct fccu_pri_data_t *priv_data,
		u32 *fault_list, u32 size)
{
	u32 i;
	int ncf_num;
	u32 reg_fault_pos;

	for (i = 0; i < size; i++) {
		ncf_num = fccu_get_ncf_reg_number(fault_list[i]);
		if (ncf_num < 0) {
			dev_err(dev, "Invalid NCF_Sn fault argument %d\n",
					fault_list[i]);
			continue;
		}

		reg_fault_pos = fault_list[i] % 32;
		priv_data->ncfn_mask[ncf_num] |= BIT(reg_fault_pos);

		if (reg_fault_pos < FCCU_NCFS_CFG_SIZE) {
			priv_data->ncfs_cfgn_mask[ncf_num * 2] |=
				BIT(reg_fault_pos * 2);
		} else {
			priv_data->ncfs_cfgn_mask[ncf_num * 2 + 1] |=
				BIT((reg_fault_pos % FCCU_NCFS_CFG_SIZE) * 2);
		}
	}
}

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

static int wait_op_success(void __iomem *base)
{
	u32 ops_val = 0;

	fccu_wait_and_get_ops(base, &ops_val);
	if (ops_val != FCCU_CTRL_OPS_SUCCESS)
		return -1;

	return 0;
}

static int wait_become_normal_st(void __iomem *base)
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

	if (wait_op_success(base) != 0)
		return -1;

	status = __raw_readl(base + FCCU_STAT) & FCCU_STAT_STATUS_MASK;
	if (status != FCCU_STAT_STATUS_NORMAL)
		return -1;

	return 0;
}

static int clear_fault_status(struct device *dev,
		struct fccu_pri_data_t *priv_data)
{
	void __iomem *base = priv_data->base;
	u32 reg_ncf_val;
	int i, ret;

	if (IS_ERR_OR_NULL(base)) {
		dev_err(fccu_miscdev.parent,
			"%s, %d, invalid argument\n", __func__, __LINE__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(priv_data->ncfn_mask); i++) {
		reg_ncf_val = __raw_readl(base + FCCU_NCF_SN(i));

		if (priv_data->ncfn_mask[i] & reg_ncf_val) {
			/* Unlock FCCU fault status source register */
			__raw_writel(FCCU_NCFK_UNLOCK_SEQUENCE,
					base + FCCU_NCFK);

			/* Clear faults with maximal fault mask for NCF_Sn */
			__raw_writel(priv_data->ncfn_mask[i],
					base + FCCU_NCF_SN(i));

			ret = wait_op_success(base);
			if (ret)
				return ret;

			dev_info(dev, "Cleared faults from NCF_S%d\n", i);
		}
	}

	return 0;
}

static int enable_rs_channel(struct device *dev,
		struct fccu_pri_data_t *priv_data)
{
	void __iomem *base = priv_data->base;
	int i = 0, ret;

	if (IS_ERR_OR_NULL(base) || IS_ERR_OR_NULL(dev)) {
		dev_err(fccu_miscdev.parent,
			"%s, %d, invalid argument\n", __func__, __LINE__);
		return -EINVAL;
	}

	ret = wait_become_normal_st(base);
	if (ret)
		return ret;

	/* Set time out for configuration */
	__raw_writel(FCCU_TIME_OUT, base + FCCU_CFG_TO);

	/* Put FCCC into configuration state */
	__raw_writel(FCCU_TL_TRANSKEY_UNLOCK, base + FCCU_TRANS_LOCK);
	__raw_writel(FCCU_CTRLK_UNLOCK_OP1, base + FCCU_CTRLK);
	__raw_writel(FCCU_CRTL_OPR_OP1, base + FCCU_CTRL);

	ret = wait_op_success(base);
	if (ret)
		return ret;

	/* Setup Non-critical Fault Enable registers */
	for (i = 0; i < ARRAY_SIZE(priv_data->ncfn_mask); i++)
		__raw_writel(priv_data->ncfn_mask[i],
				base + FCCU_NCF_EN(i));

	/* Setup Non-critical Fault-State Configuration registers */
	for (i = 0; i < ARRAY_SIZE(priv_data->ncfs_cfgn_mask); i++)
		__raw_writel(priv_data->ncfs_cfgn_mask[i],
				base + FCCU_NCFS_CFGN(i));

	/* Put FCCU into normal state */
	__raw_writel(FCCU_CTRLK_UNLOCK_OP2, base + FCCU_CTRLK);
	__raw_writel(FCCU_CTRL_OPR_OP2, base + FCCU_CTRL);

	return wait_op_success(base);
}

static int __init s32_fccu_probe(struct platform_device *pdev)
{
	int ret;
	struct fccu_pri_data_t *priv_data = NULL;
	struct property *prop = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	size_t size;
	u32 *ncf_fault_arr = NULL;

	priv_data = devm_kzalloc(dev,
		sizeof(struct fccu_pri_data_t), GFP_KERNEL);
	if (priv_data == NULL)
		return -ENOMEM;

	fccu_miscdev.parent = dev;
	priv_data->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv_data->base = devm_ioremap_resource(dev, priv_data->res);

	if (IS_ERR_OR_NULL(priv_data->base)) {
		dev_err(fccu_miscdev.parent,
			"%s, %d, invalid argument\n", __func__, __LINE__);
		release_resource(priv_data->res);
		devm_kfree(dev, priv_data);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, priv_data);

	prop = of_find_property(np, "nxp,ncf_fault_list", NULL);
	if (prop == NULL) {
		dev_err(fccu_miscdev.parent,
				"can not find property: 'nxp,ncf_fault_list'\n");
		return -EINVAL;
	}

	size = prop->length / sizeof(*ncf_fault_arr);
	ncf_fault_arr = kcalloc(size, sizeof(*ncf_fault_arr), GFP_KERNEL);
	if (ncf_fault_arr == NULL)
		return -ENOMEM;

	ret = of_property_read_u32_array(np,
			"nxp,ncf_fault_list", ncf_fault_arr, size);
	if (ret) {
		dev_err(fccu_miscdev.parent,
				"No fault status registers offset specified\n");
		goto out;
	}

	set_maximal_fault_lists(dev, priv_data,
			ncf_fault_arr, size);

	ret = clear_fault_status(dev, priv_data);
	if (ret) {
		dev_err(fccu_miscdev.parent, "%s, %d, configuration meet timeout\n",
			__func__, __LINE__);
		goto out;
	}

	ret = enable_rs_channel(dev, priv_data);
	if (ret) {
		dev_err(fccu_miscdev.parent, "%s, %d, configuration meet timeout\n",
			__func__, __LINE__);
		goto out;
	}

	ret = misc_register(&fccu_miscdev);

out:
	kfree(ncf_fault_arr);
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

	clear_fault_status(&pdev->dev, priv_data);
	release_resource(priv_data->res);

	return 0;
}

static int __maybe_unused s32_fccu_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused s32_fccu_resume(struct device *dev)
{
	struct fccu_pri_data_t *priv_data = dev_get_drvdata(dev);
	int ret;

	ret = enable_rs_channel(dev, priv_data);
	if (ret)
		dev_err(dev, "%s, %d, configuration meet timeout\n",
			__func__, __LINE__);

	return ret;
}

static SIMPLE_DEV_PM_OPS(s32_fccu_pm_ops, s32_fccu_suspend, s32_fccu_resume);

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
			.pm = &s32_fccu_pm_ops,
		},
};

module_platform_driver_probe(s32_fccu_driver,
	s32_fccu_probe);

MODULE_AUTHOR("Phu Luu An");
MODULE_DESCRIPTION("FCCU driver for S32 SoC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_MISCDEV(FCCU_MINOR);
MODULE_ALIAS("platform:" DRIVER_NAME);
