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
#include <dt-bindings/misc/s32-fccu.h>

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


#define FCCU_NCFS_CFGN_OFF			0x4C
#define FCCU_NCF_EN_OFF				0x94
#define FCCU_NCF_SN_OFF				0x80
#define FCCU_NCF_TOEN_OFF			0xA4
#define FCCU_IRQ_ALARM_EN_OFF			0xFC
#define FCCU_NMI_EN_OFF				0x10C
#define FCCU_EOUT_SIG_EN_OFF			0x11C

#define FCCU_NCF_SN_MAX_NUM			0x4
#define FCCU_NCFS_CFG_SIZE			16

#define FCCU_ALARM_IRQ_NAME			"fccu_alarm"
#define FCCU_MISC_IRQ_NAME			"fccu_misc"

/* Maximum number of retries to reset the NCF fault. This is not specified in
 * RM which assumes an infinite loop, but it is used here to avoid blocking
 * forever in interrupt handler
 */
#define FCCU_RESET_MAX_NUMBER			20000


static struct miscdevice fccu_miscdev = {
	.minor = FCCU_MINOR,
	.name = "fccu",
};

struct fccu_ncf_regs {
	u32 *mask;
	const size_t size;
	const u32 offset;
};

struct fccu_alarm_mask {
	struct fccu_ncf_regs status;
	struct fccu_ncf_regs alarm_state;
	struct fccu_ncf_regs alarm_irq;
	struct fccu_ncf_regs func_reset;
	struct fccu_ncf_regs nmi;
	struct fccu_ncf_regs eout;

	struct fccu_ncf_regs ncf_enable;
};

static u32 fccu_status_mask[4];
static u32 fccu_alarm_mask[4];
static u32 fccu_func_reset_mask[8];
static u32 fccu_nmi_mask[4];
static u32 fccu_eout_mask[4];
static u32 fccu_ncf_enable_mask[4];

static struct fccu_alarm_mask mask = {
	.status = {
		.mask = fccu_status_mask,
		.size = ARRAY_SIZE(fccu_status_mask),
		.offset = FCCU_NCF_SN_OFF,
	},
	.alarm_state = {
		.mask = fccu_alarm_mask,
		.size = ARRAY_SIZE(fccu_alarm_mask),
		.offset = FCCU_NCF_TOEN_OFF,
	},
	.alarm_irq = {
		/* common mask with alarm_state */
		.mask = fccu_alarm_mask,
		.size = ARRAY_SIZE(fccu_alarm_mask),
		.offset = FCCU_IRQ_ALARM_EN_OFF,
	},
	.func_reset = {
		.mask = fccu_func_reset_mask,
		.size = ARRAY_SIZE(fccu_func_reset_mask),
		.offset = FCCU_NCFS_CFGN_OFF,
	},
	.nmi = {
		.mask = fccu_nmi_mask,
		.size = ARRAY_SIZE(fccu_nmi_mask),
		.offset = FCCU_NMI_EN_OFF,
	},
	.eout = {
		.mask = fccu_eout_mask,
		.size = ARRAY_SIZE(fccu_eout_mask),
		.offset = FCCU_EOUT_SIG_EN_OFF,
	},
	.ncf_enable = {
		.mask = fccu_ncf_enable_mask,
		.size = ARRAY_SIZE(fccu_ncf_enable_mask),
		.offset = FCCU_NCF_EN_OFF,
	},
};

struct fccu_pri_data {
	struct resource *res;
	struct device *dev;
	void __iomem *base;
	struct fccu_alarm_mask *mask;
};

static int fccu_get_ncf_reg_number(u32 val)
{
	int ncf_num = val / 32;

	return (ncf_num < FCCU_NCF_SN_MAX_NUM) ? ncf_num : -EINVAL;
}

/* Set maximal fault masks for every NCF_Sn */
static void set_maximal_fault_lists(struct device *dev,
		struct fccu_pri_data *priv_data,
		u32 *fault_list, u32 size,
		u32 *reaction_list, u32 size_r)
{
	u32 i;
	int ncf_num;
	u32 reg_fault_pos;
	struct fccu_alarm_mask *mask = priv_data->mask;
	int reactions = 0;

	for (i = 0; i < size; i++) {
		ncf_num = fccu_get_ncf_reg_number(fault_list[i]);
		if (ncf_num < 0) {
			dev_err(dev, "Invalid NCF_Sn fault argument %d\n",
					fault_list[i]);
			continue;
		}

		reg_fault_pos = fault_list[i] % 32;

		/* Setup status bits */
		mask->status.mask[ncf_num] |= BIT(reg_fault_pos);

		if (size_r == 0)
			continue;

		/* Setup reaction bits */
		if (reaction_list[i] & S32_FCCU_REACTION_ALARM) {
			mask->alarm_state.mask[ncf_num] |= BIT(reg_fault_pos);
			reactions++;
		}

		if (reaction_list[i] & S32_FCCU_REACTION_NMI) {
			mask->nmi.mask[ncf_num] |= BIT(reg_fault_pos);
			reactions++;
		}

		if (reaction_list[i] & S32_FCCU_REACTION_EOUT) {
			mask->eout.mask[ncf_num] |= BIT(reg_fault_pos);
			reactions++;
		}

		if (reaction_list[i] & S32_FCCU_REACTION_FUNC_RESET) {
			if (reg_fault_pos < FCCU_NCFS_CFG_SIZE) {
				mask->func_reset.mask[ncf_num * 2] |=
				    BIT(reg_fault_pos * 2);
			} else {
				mask->func_reset.mask[ncf_num * 2 + 1] |=
				    BIT((reg_fault_pos % FCCU_NCFS_CFG_SIZE) * 2);
			}
			reactions++;
		}

		/* Enable NCF reaction if at least one reaction is enabled */
		if (reactions)
			mask->ncf_enable.mask[ncf_num] |= BIT(reg_fault_pos);
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

static int get_fccu_status(void __iomem *base)
{
	return __raw_readl(base + FCCU_STAT) & FCCU_STAT_STATUS_MASK;
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

static inline u32 get_offset(struct fccu_ncf_regs *reg, int reg_index)
{
	return reg->offset + 4 * reg_index;
}

static void write_ncf_regs(struct fccu_pri_data *priv_data,
			  struct fccu_ncf_regs *reg)
{
	void __iomem *base = priv_data->base;
	int i;

	for (i = 0; i < reg->size; i++)
		__raw_writel(reg->mask[i], base + get_offset(reg, i));
}


static int reset_ncf_register(struct device *dev,
				struct fccu_pri_data *priv_data,
				int reg_index,
				u32 *fault)
{
	void __iomem *base = priv_data->base;
	struct fccu_ncf_regs *status = &priv_data->mask->status;
	u32 ncf_val = 0;
	int ret;
	int count = 0;
	int state = get_fccu_status(base);

	if (IS_ERR_OR_NULL(fault))
		return -EINVAL;

	while ((ncf_val = status->mask[reg_index] &
		 __raw_readl(base + get_offset(status, reg_index)))) {

		if (++count == 1)
			*fault = ncf_val;
		else if (count >= FCCU_RESET_MAX_NUMBER) {
			dev_err(dev, "Failed to reset NCF after %d retries state=%d->%d\n",
				count, state, get_fccu_status(base));
			return -EOVERFLOW;
		}

		/* Unlock FCCU fault status source register */
		__raw_writel(FCCU_NCFK_UNLOCK_SEQUENCE,
			     base + FCCU_NCFK);

		/* Clear faults with maximal fault mask for NCF_Sn */
		__raw_writel(status->mask[reg_index],
			     base + get_offset(status, reg_index));

		ret = wait_op_success(base);
		if (ret)
			return ret;
	}

	return count;
}

static int clear_fault_status(struct device *dev,
		struct fccu_pri_data *priv_data)
{
	void __iomem *base = priv_data->base;
	struct fccu_alarm_mask *mask = priv_data->mask;
	u32 reg_ncf_val;
	int i, ret;
	int state = get_fccu_status(base);

	if (IS_ERR_OR_NULL(base)) {
		dev_err(fccu_miscdev.parent,
			"%s, %d, invalid argument\n", __func__, __LINE__);
		return -EINVAL;
	}

	for (i = 0; i < mask->status.size; i++) {
		ret = reset_ncf_register(dev, priv_data, i, &reg_ncf_val);

		if (ret < 0)
			return ret;

		if (ret > 0) {
			dev_info(dev, "Cleared faults 0x%x from NCF_S%d (%d) state=%d->%d\n",
				reg_ncf_val, i, ret,
				state, get_fccu_status(base));
		}
	}

	return 0;
}

static int enable_rs_channel(struct device *dev,
		struct fccu_pri_data *priv_data)
{
	void __iomem *base = priv_data->base;
	struct fccu_alarm_mask *mask = priv_data->mask;
	int ret;

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

	/* 67.8.3 Configure the non-critical fault channels */
	/* 1. Recovery type: NCF_CFGa - Default is SW recoverable, and the
	 * recommended recovery type is SW. Therefore no change needed
	 */

	/*2. Enable Fault-state reaction
	 * - chip functional reset, NMI, or EOUT signal
	 */
	write_ncf_regs(priv_data, &mask->func_reset);
	write_ncf_regs(priv_data, &mask->nmi);
	write_ncf_regs(priv_data, &mask->eout);

	/* 3. Set Alarm-state timeout
	 * Leave default: 0x0003_A980
	 */

	/* 4. Enable Alarm state NCF_TOEa */
	write_ncf_regs(priv_data, &mask->alarm_state);

	/* 5. Enable Alarm state reaction IRQ_ALARM_ENa */
	write_ncf_regs(priv_data, &mask->alarm_irq);

	/* 6. Enable Non-critical Fault Enable registers */
	write_ncf_regs(priv_data, &mask->ncf_enable);

	/* Put FCCU into normal state */
	__raw_writel(FCCU_CTRLK_UNLOCK_OP2, base + FCCU_CTRLK);
	__raw_writel(FCCU_CTRL_OPR_OP2, base + FCCU_CTRL);

	ret = wait_op_success(base);
	dev_info(dev, "FCCU status is %d (normal)\n", get_fccu_status(base));

	return ret;
}

static irqreturn_t irq_alarm_handler(int irq, void *data)
{
	struct fccu_pri_data *priv_data = data;
	struct device *dev = priv_data->dev;

	clear_fault_status(dev, priv_data);
	return IRQ_HANDLED;
}

static irqreturn_t irq_misc_handler(int irq, void *data)
{
	struct fccu_pri_data *priv_data = data;
	struct device *dev = priv_data->dev;

	/* The misc alarm handling is just informational */
	dev_info(dev, "interrupt handler: misc. No action performed\n");
	return IRQ_HANDLED;
}

static int init_fccu_irq_resources(struct platform_device *pdev,
				   struct fccu_pri_data *priv_data)
{
	int irq_alarm, irq_misc;
	struct device *dev = &pdev->dev;
	int ret;

	irq_alarm = platform_get_irq_byname(pdev, FCCU_ALARM_IRQ_NAME);
	if (irq_alarm < 0) {
		dev_err(dev, "Failed to request IRQ 0\n");
		return irq_alarm;
	}
	irq_misc = platform_get_irq_byname(pdev, FCCU_MISC_IRQ_NAME);
	if (irq_misc < 0) {
		dev_err(dev, "Failed to request IRQ 1\n");
		return irq_misc;
	}

	/* Interrtups are threaded because, along with clearing the NCF
	 * they are also intended for printing the list of detect faults
	 * in the log (dmesg)
	 */
	ret = devm_request_threaded_irq(dev, irq_alarm, NULL,
					irq_alarm_handler, IRQF_ONESHOT,
					FCCU_ALARM_IRQ_NAME, priv_data);
	if (ret < 0) {
		dev_err(dev, "Failed to register IRQ\n");
		return ret;
	}

	ret = devm_request_threaded_irq(dev, irq_misc, NULL,
					irq_misc_handler, IRQF_ONESHOT,
					FCCU_MISC_IRQ_NAME, priv_data);
	if (ret < 0) {
		dev_err(dev, "Failed to register IRQ\n");
		return ret;
	}

	return 0;
}

static int get_fccu_array(struct device *dev,
			  const char *name,
			  u32 **parray,
			  size_t *psize)
{
	int ret;
	struct device_node *np = dev->of_node;
	struct property *prop = NULL;

	if (IS_ERR_OR_NULL(parray) || IS_ERR_OR_NULL(psize)) {
		dev_err(dev, "Invalid array parameters\n");
		return -EINVAL;
	}

	prop = of_find_property(np, name, NULL);
	if (prop == NULL) {
		/* Allow missing property */
		*psize = 0;
		*parray = NULL;
		return 0;
	}

	ret = of_property_count_elems_of_size(np, name, sizeof(u32));
	if (ret <= 0) {
		dev_err(dev, "Invalid number of elements in %s\n", name);
		return -EINVAL;
	}

	*psize = ret;

	*parray = kcalloc(*psize, sizeof(u32), GFP_KERNEL);
	if (*parray == NULL)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, name, *parray, *psize);
	if (ret) {
		kfree(*parray);
		dev_err(dev, "Error reading %s array\n", name);
		return -EINVAL;
	}
	return 0;

}

static int init_fccu_array_resources(struct device *dev,
				     struct fccu_pri_data *priv_data)
{
	int ret = 0;
	size_t size, size_r;
	u32 *ncf_fault_arr = NULL;
	u32 *ncf_reaction_arr = NULL;


	ret = get_fccu_array(dev, "nxp,ncf_fault_list",
			     &ncf_fault_arr, &size);
	if (ret)
		return ret;

	ret = get_fccu_array(dev, "nxp,ncf_actions",
			     &ncf_reaction_arr, &size_r);
	if (ret)
		goto free_fault;

	/* If action list is present, its size must be the same as
	 * the fault list: each action apply to a fault from the list
	 */
	if ((size_r) && (size != size_r)) {
		dev_err(dev, "Error: size mismatch for fault properties list");
		ret = -EINVAL;
		goto out;
	}

	priv_data->mask = &mask;
	set_maximal_fault_lists(dev, priv_data,
				ncf_fault_arr, size,
				ncf_reaction_arr, size_r);
out:
	kfree(ncf_reaction_arr);
free_fault:
	kfree(ncf_fault_arr);

	return ret;
}


static int __init s32_fccu_probe(struct platform_device *pdev)
{
	int ret;
	struct fccu_pri_data *priv_data = NULL;
	struct device *dev = &pdev->dev;

	priv_data = devm_kzalloc(dev,
		sizeof(struct fccu_pri_data), GFP_KERNEL);
	if (priv_data == NULL)
		return -ENOMEM;

	fccu_miscdev.parent = dev;
	priv_data->dev = dev;
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

	ret = init_fccu_irq_resources(pdev, priv_data);
	if (ret)
		return ret;

	ret = init_fccu_array_resources(dev, priv_data);
	if (ret)
		return ret;

	ret = clear_fault_status(dev, priv_data);
	if (ret) {
		dev_err(fccu_miscdev.parent, "%s, %d, reset alarm timeout\n",
			__func__, __LINE__);
		return ret;
	}

	ret = enable_rs_channel(dev, priv_data);
	if (ret) {
		dev_err(fccu_miscdev.parent, "%s, %d, configuration timeout\n",
			__func__, __LINE__);
		return ret;
	}

	ret = misc_register(&fccu_miscdev);

	return ret;
}

static int __exit s32_fccu_remove(struct platform_device *pdev)
{
	struct fccu_pri_data *priv_data = NULL;

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
	struct fccu_pri_data *priv_data = dev_get_drvdata(dev);
	int ret;

	ret = enable_rs_channel(dev, priv_data);
	if (ret)
		dev_err(dev, "%s, %d, configuration meet timeout\n",
			__func__, __LINE__);

	return ret;
}

static SIMPLE_DEV_PM_OPS(s32_fccu_pm_ops, s32_fccu_suspend, s32_fccu_resume);

static const struct of_device_id s32_fccu_dt_ids[] = {
	{.compatible = "nxp,s32cc-fccu",},
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
