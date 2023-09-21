// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Core driver for the S32CC pin controller
 *
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018, 2020-2023 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/gpio/driver.h>
#include <linux/slab.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"
#include "pinctrl-s32cc.h"

#define S32CC_PIN_NO_SHIFT	4

#define S32CC_MSCR_SSS_MASK	GENMASK(2, 0)
#define S32CC_MSCR_PUS		BIT(12)
#define S32CC_MSCR_PUE		BIT(13)
#define S32CC_MSCR_SRE(X)		(((X) & GENMASK(3, 0)) << 14)
#define S32CC_MSCR_IBE		BIT(19)
#define S32CC_MSCR_ODE		BIT(20)
#define S32CC_MSCR_OBE		BIT(21)
#define S32CC_MSCR_CFG_MASK	GENMASK(21, 12)

#define S32CC_NXP_PINS		"nxp,pins"

static u32 get_pin_no(u32 pinmux)
{
	return pinmux >> S32CC_PIN_NO_SHIFT;
}

static u32 get_pin_func(u32 pinmux)
{
	return pinmux & GENMASK(3, 0);
}

struct s32cc_pinctrl_mem_region {
	void __iomem *base;
	unsigned int start_pin;
	unsigned int end_pin;
};

/**
 * Holds pin configuration for GPIO's.
 * @pin: Pin settings
 * @list: Linked list entry for each gpio pin
 */
struct gpio_pin_config {
	unsigned int pin_id;
	unsigned int config;
	struct list_head list;
};

struct s32cc_pinctrl_context {
	unsigned long *pads;
};

/**
 * @dev: a pointer back to containing device
 * @pctl: a pointer to the pinctrl device structure
 * @regions: reserved memory regions with start/end pin
 * @info: structure containing information about the pin
 * @gpio_configs: Saved configurations for GPIO pins
 * @gpiop_configs_lock: lock for the `gpio_configs` list
 * @s32cc_pinctrl_context: Configuration saved over system sleep
 * @reg_lock: lock for the `mscr/imcrs` registers
 */
struct s32cc_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	struct s32cc_pinctrl_mem_region *regions;
	struct s32cc_pinctrl_soc_info *info;
	struct list_head gpio_configs;
	/* lock for the `gpio_configs` list */
	spinlock_t gpio_configs_lock;
#ifdef CONFIG_PM_SLEEP
	struct s32cc_pinctrl_context saved_context;
#endif
	/* lock for the `mscr/imcrs` registers */
	spinlock_t reg_lock;
};

static const char *pin_get_name_from_info(struct s32cc_pinctrl_soc_info *info,
					  const unsigned int pin_id)
{
	int i;

	for (i = 0; i < info->npins; i++) {
		if (info->pins[i].number == pin_id)
			return info->pins[i].name;
	}

	return NULL;
}

static struct s32cc_pinctrl_mem_region *
	s32cc_get_region(struct pinctrl_dev *pctldev, unsigned int pin)
{
	int i;
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	u32 mem_regions = ipctl->info->mem_regions;

	for (i = 0; i < mem_regions; ++i) {
		if (pin >= ipctl->regions[i].start_pin &&
		    pin <= ipctl->regions[i].end_pin) {
			return &ipctl->regions[i];
		}
	}

	return NULL;
}

static inline int s32cc_check_pin(struct pinctrl_dev *pctldev,
				  unsigned int pin)
{
	return s32cc_get_region(pctldev, pin) ? 0 : -EINVAL;
}

static inline int s32cc_pinctrl_readl_nolock(struct pinctrl_dev *pctldev,
					     unsigned int pin,
					     unsigned long *config)
{
	struct s32cc_pinctrl_mem_region *region;
	void __iomem *base;
	unsigned int offset;

	region = s32cc_get_region(pctldev, pin);
	if (!region)
		return -EINVAL;

	offset = pin - region->start_pin;
	base = region->base;

	*config = readl(base + S32CC_PAD_CONFIG(offset));

	return 0;
}

static inline int s32cc_pinctrl_readl(struct pinctrl_dev *pctldev,
				      unsigned int pin,
				      unsigned long *config)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ipctl->reg_lock, flags);
	ret = s32cc_pinctrl_readl_nolock(pctldev, pin, config);
	spin_unlock_irqrestore(&ipctl->reg_lock, flags);

	return ret;
}

static inline int s32cc_pinctrl_writel_nolock(struct pinctrl_dev *pctldev,
					      unsigned int pin,
					      unsigned long config)
{
	struct s32cc_pinctrl_mem_region *region;
	void __iomem *base;
	unsigned int offset;

	region = s32cc_get_region(pctldev, pin);
	if (!region)
		return -EINVAL;

	offset = pin - region->start_pin;
	base = region->base;

	writel(config, base + S32CC_PAD_CONFIG(offset));

	return 0;
}

static inline int s32cc_pinctrl_writel(unsigned long config,
				       struct pinctrl_dev *pctldev,
				       unsigned int pin)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ipctl->reg_lock, flags);
	ret = s32cc_pinctrl_writel_nolock(pctldev, pin, config);
	spin_unlock_irqrestore(&ipctl->reg_lock, flags);

	return ret;
}

static inline const
struct s32cc_pin *s32cc_pinctrl_find_pin(const struct s32cc_pin_group *grp,
					 unsigned int pin_id)
{
	unsigned i;

	if (!grp)
		return NULL;

	for (i = 0; i < grp->npins; i++) {
		struct s32cc_pin *pin = &grp->pins[i];

		if (pin->pin_id == pin_id)
			return pin;
	}

	return NULL;
}

static int s32cc_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32cc_pinctrl_soc_info *info = ipctl->info;

	return info->ngroups;
}

static const char *s32cc_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned int selector)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32cc_pinctrl_soc_info *info = ipctl->info;

	return info->groups[selector].name;
}

static int s32cc_get_group_pins(struct pinctrl_dev *pctldev,
			       unsigned int selector, const unsigned int **pins,
			       unsigned int *npins)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32cc_pinctrl_soc_info *info = ipctl->info;

	if (selector >= info->ngroups)
		return -EINVAL;

	*pins = info->groups[selector].pin_ids;
	*npins = info->groups[selector].npins;

	return 0;
}

static void s32cc_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
		   unsigned int offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static int s32cc_dt_group_node_to_map(struct pinctrl_dev *pctldev,
				      struct device_node *np,
				      struct pinctrl_map **map,
				      unsigned int *reserved_maps,
				      unsigned int *num_maps,
				      const char *func_name)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long *cfgs = NULL;
	unsigned int n_cfgs, reserve = 1;
	struct device *dev;
	int n_pins, ret;

	if (!ipctl)
		return -EINVAL;

	dev = ipctl->dev;

	n_pins = of_property_count_elems_of_size(np, "pinmux", sizeof(u32));
	if (n_pins < 0) {
		dev_err(dev, "Failed to read 'pinmux' property in node %s.\n",
			np->name);
	} else if (!n_pins) {
		return -EINVAL;
	}

	ret = pinconf_generic_parse_dt_config(np, pctldev, &cfgs, &n_cfgs);
	if (ret) {
		dev_err(dev, "%pOF: could not parse node property\n", np);
		return ret;
	}

	if (n_cfgs)
		reserve++;

	ret = pinctrl_utils_reserve_map(pctldev, map, reserved_maps, num_maps,
					reserve);
	if (ret < 0)
		goto free_cfgs;

	ret = pinctrl_utils_add_map_mux(pctldev, map, reserved_maps, num_maps,
					np->name, func_name);
	if (ret < 0)
		goto free_cfgs;

	if (n_cfgs) {
		ret = pinctrl_utils_add_map_configs(pctldev, map, reserved_maps,
						    num_maps, np->name, cfgs, n_cfgs,
						    PIN_MAP_TYPE_CONFIGS_GROUP);
		if (ret < 0)
			goto free_cfgs;
	}

free_cfgs:
	kfree(cfgs);
	return ret;
}

static int s32cc_dt_node_to_map(struct pinctrl_dev *pctldev,
				struct device_node *np_config,
				struct pinctrl_map **map,
				unsigned int *num_maps)
{
	unsigned int reserved_maps;
	struct device_node *np;
	int ret = 0;

	reserved_maps = 0;
	*map = NULL;
	*num_maps = 0;

	for_each_available_child_of_node(np_config, np) {
		ret = s32cc_dt_group_node_to_map(pctldev, np, map,
						 &reserved_maps, num_maps,
						 np_config->name);
		if (ret < 0)
			break;
	}

	if (ret)
		pinctrl_utils_free_map(pctldev, *map, *num_maps);

	return ret;

}

static const struct pinctrl_ops s32cc_pctrl_ops = {
	.get_groups_count = s32cc_get_groups_count,
	.get_group_name = s32cc_get_group_name,
	.get_group_pins = s32cc_get_group_pins,
	.pin_dbg_show = s32cc_pin_dbg_show,
	.dt_node_to_map = s32cc_dt_node_to_map,
	.dt_free_map = pinctrl_utils_free_map,
};

static int s32cc_update_pin_mscr(struct pinctrl_dev *pctldev,
				 unsigned int pin,
				 unsigned long mask,
				 unsigned long new_mask)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long config, flags;
	int ret;

	spin_lock_irqsave(&ipctl->reg_lock, flags);

	ret = s32cc_pinctrl_readl_nolock(pctldev, pin, &config);
	if (ret)
		goto unlock;

	config &= ~mask;
	config |= new_mask;

	ret = s32cc_pinctrl_writel_nolock(pctldev, pin, config);
	if (ret)
		goto unlock;

unlock:
	spin_unlock_irqrestore(&ipctl->reg_lock, flags);

	return ret;
}

static int s32cc_pmx_set(struct pinctrl_dev *pctldev, unsigned int selector,
		       unsigned int group)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32cc_pinctrl_soc_info *info = ipctl->info;
	unsigned int npins;
	int i, ret = 0, err;
	struct s32cc_pin_group *grp;

	/*
	 * Configure the mux mode for each pin in the group for a specific
	 * function.
	 */
	grp = &info->groups[group];
	npins = grp->npins;

	dev_dbg(ipctl->dev, "set mux for function %s group %s\n",
		info->functions[selector].name, grp->name);

	/* Check beforehand so we don't have a partial config. */
	for (i = 0; i < npins; ++i) {
		struct s32cc_pin *pin = &grp->pins[i];

		if (s32cc_check_pin(pctldev, pin->pin_id) != 0) {
			dev_err(info->dev, "invalid pin: %d in group: %d\n",
				pin->pin_id, group);
			return -EINVAL;
		}
	}

	for (i = 0; i < npins; ++i) {
		struct s32cc_pin *pin = &grp->pins[i];

		err = s32cc_update_pin_mscr(pctldev, pin->pin_id,
					    S32CC_MSCR_SSS_MASK, pin->sss);
		if (err) {
			if (!ret)
				ret = err;
		}
	}

	return ret;
}

static int s32cc_pmx_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32cc_pinctrl_soc_info *info = ipctl->info;

	return info->nfunctions;
}

static const char *s32cc_pmx_get_func_name(struct pinctrl_dev *pctldev,
					  unsigned int selector)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32cc_pinctrl_soc_info *info = ipctl->info;

	return info->functions[selector].name;
}

static int s32cc_pmx_get_groups(struct pinctrl_dev *pctldev,
			       unsigned int selector,
			       const char * const **groups,
			       unsigned int * const num_groups)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32cc_pinctrl_soc_info *info = ipctl->info;

	*groups = info->functions[selector].groups;
	*num_groups = info->functions[selector].num_groups;

	return 0;
}

static int s32cc_pmx_gpio_request_enable(struct pinctrl_dev *pctldev,
			struct pinctrl_gpio_range *range, unsigned int offset)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long config;
	struct gpio_pin_config *gpio_pin;
	unsigned long flags;
	int ret;

	ret = s32cc_pinctrl_readl(pctldev, offset, &config);
	if (ret)
		return -EINVAL;

	/* Save current configuration */
	gpio_pin = kmalloc(sizeof(*gpio_pin), GFP_KERNEL);
	if (!gpio_pin)
		return -ENOMEM;

	gpio_pin->pin_id = offset;
	gpio_pin->config = config;

	spin_lock_irqsave(&ipctl->gpio_configs_lock, flags);
	list_add(&(gpio_pin->list), &(ipctl->gpio_configs));
	spin_unlock_irqrestore(&ipctl->gpio_configs_lock, flags);

	/* GPIO pin means SSS = 0 */
	config &= ~S32CC_MSCR_SSS_MASK;

	return s32cc_pinctrl_writel(config, pctldev, offset);
}

static void s32cc_pmx_gpio_disable_free(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range,
		unsigned int offset)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	struct list_head *pos, *tmp;
	struct gpio_pin_config *gpio_pin;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ipctl->gpio_configs_lock, flags);

	list_for_each_safe(pos, tmp, &ipctl->gpio_configs) {
		gpio_pin = list_entry(pos, struct gpio_pin_config, list);

		if (gpio_pin->pin_id == offset) {
			ret = s32cc_pinctrl_writel(gpio_pin->config, pctldev,
						   gpio_pin->pin_id);
			if (ret != 0)
				goto unlock;

			list_del(pos);
			kfree(gpio_pin);
			break;
		}
	}

unlock:
	spin_unlock_irqrestore(&ipctl->gpio_configs_lock, flags);
}

static int s32cc_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
	   struct pinctrl_gpio_range *range, unsigned int offset, bool input)
{
	unsigned long config;
	int ret;

	ret = s32cc_pinctrl_readl(pctldev, offset, &config);
	if (ret)
		return -EINVAL;

	/* Always enable IBE for GPIOs. This allows us to read the
	 * actual line value and compare it with the one set.
	 */
	config |= S32CC_MSCR_IBE;

	if (input)
		config &= ~S32CC_MSCR_OBE;
	else
		config |= S32CC_MSCR_OBE;

	return s32cc_pinctrl_writel(config, pctldev, offset);
}

static const struct pinmux_ops s32cc_pmx_ops = {
	.get_functions_count = s32cc_pmx_get_funcs_count,
	.get_function_name = s32cc_pmx_get_func_name,
	.get_function_groups = s32cc_pmx_get_groups,
	.set_mux = s32cc_pmx_set,
	.gpio_request_enable = s32cc_pmx_gpio_request_enable,
	.gpio_disable_free = s32cc_pmx_gpio_disable_free,
	.gpio_set_direction = s32cc_pmx_gpio_set_direction,
};

static int s32cc_pinconf_get(struct pinctrl_dev *pctldev,
			     unsigned int pin_id, unsigned long *config)
{
	int ret;

	ret = s32cc_pinctrl_readl(pctldev, pin_id, config);
	if (ret)
		return -EINVAL;

	return 0;
}

static int s32cc_get_pin_conf(struct s32cc_pinctrl *ipctl,
			      unsigned int pin_id,
			      enum pin_config_param param,
			      u32 arg,
			      unsigned long *mask,
			      unsigned long *config)
{
	switch (param) {
	/* All pins are persistent over suspend */
	case PIN_CONFIG_PERSIST_STATE:
		return 0;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		*config |= S32CC_MSCR_ODE;
		*mask |= S32CC_MSCR_ODE;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		*config &= ~S32CC_MSCR_ODE;
		*mask |= S32CC_MSCR_ODE;
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		if (arg)
			*config |= S32CC_MSCR_OBE;
		else
			*config &= ~S32CC_MSCR_OBE;
		*mask |= S32CC_MSCR_OBE;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		if (arg)
			*config |= S32CC_MSCR_IBE;
		else
			*config &= ~S32CC_MSCR_IBE;
		*mask |= S32CC_MSCR_IBE;
		break;
	case PIN_CONFIG_SLEW_RATE:
		*config |= S32CC_MSCR_SRE(arg);
		*mask |= S32CC_MSCR_SRE(~0);
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (arg)
			*config |= S32CC_MSCR_PUS;
		else
			*config &= ~S32CC_MSCR_PUS;
		fallthrough;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (arg)
			*config |= S32CC_MSCR_PUE;
		else
			*config &= ~S32CC_MSCR_PUE;
		*mask |= S32CC_MSCR_PUE | S32CC_MSCR_PUS;
		break;
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		*config &= ~(S32CC_MSCR_ODE | S32CC_MSCR_OBE | S32CC_MSCR_IBE);
		*mask |= S32CC_MSCR_ODE | S32CC_MSCR_OBE | S32CC_MSCR_IBE;
		fallthrough;
	case PIN_CONFIG_BIAS_DISABLE:
		*config &= ~(S32CC_MSCR_PUS | S32CC_MSCR_PUE);
		*mask |= S32CC_MSCR_PUS | S32CC_MSCR_PUE;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int s32cc_pinconf_mscr_modify_write(struct pinctrl_dev *pctldev,
					   unsigned int pin_id,
					   unsigned long *configs,
					   unsigned int num_configs, bool write)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long config = 0, mask = 0;
	int i, ret;

	if (s32cc_check_pin(pctldev, pin_id) != 0)
		return -EINVAL;

	dev_dbg(ipctl->dev, "pinconf set pin %s with %d configs\n",
		pin_get_name(pctldev, pin_id), num_configs);

	for (i = 0; i < num_configs; i++) {
		ret = s32cc_get_pin_conf(ipctl, pin_id,
					 pinconf_to_config_param(configs[i]),
					 pinconf_to_config_argument(configs[i]),
					 &mask, &config);
		if (ret)
			return ret;
	}

	/* If the MSCR configuration has to be written,
	 * the SSS field should not be touched.
	 */
	if (write)
		mask = ~S32CC_MSCR_SSS_MASK;

	if (!config && !mask)
		return 0;

	ret = s32cc_update_pin_mscr(pctldev, pin_id, mask, config);

	if (write)
		dev_dbg(ipctl->dev, "set: pin %d cfg 0x%lx\n", pin_id, config);
	else
		dev_dbg(ipctl->dev, "update: pin %d cfg 0x%lx\n", pin_id,
			config);

	return ret;
}

static int s32cc_pinconf_mscr_write(struct pinctrl_dev *pctldev,
				    unsigned int pin_id,
				    unsigned long *configs,
				    unsigned int num_configs)
{
	return s32cc_pinconf_mscr_modify_write(pctldev, pin_id, configs,
					     num_configs, true);
}

static int s32cc_pinconf_mscr_modify(struct pinctrl_dev *pctldev,
				     unsigned int pin_id,
				     unsigned long *configs,
				     unsigned int num_configs)
{
	return s32cc_pinconf_mscr_modify_write(pctldev, pin_id, configs,
					     num_configs, false);
}

static int s32cc_pinconf_set(struct pinctrl_dev *pctldev,
			     unsigned int pin_id,
			     unsigned long *configs,
			     unsigned int num_configs)
{
	return s32cc_pinconf_mscr_modify(pctldev, pin_id, configs, num_configs);
}

static int s32cc_pconf_group_set(struct pinctrl_dev *pctldev,
				 unsigned int group,
				 unsigned long *configs,
				 unsigned int num_configs)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32cc_pinctrl_soc_info *info;
	struct s32cc_pin_group *grp;
	struct s32cc_pin *pin;
	int i, ret;

	if (!ipctl)
		return -EINVAL;

	info = ipctl->info;

	if (group > info->ngroups)
		return -EINVAL;

	grp = &info->groups[group];
	for (i = 0; i < grp->npins; i++) {
		pin = &grp->pins[i];

		ret = s32cc_pinconf_mscr_write(pctldev, pin->pin_id,
					       configs, num_configs);
		if (ret)
			return ret;
	}

	return 0;
}

static void s32cc_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				   struct seq_file *s, unsigned int pin_id)
{
	unsigned long config;
	int ret;

	ret = s32cc_pinctrl_readl(pctldev, pin_id, &config);
	if (!ret)
		seq_printf(s, "0x%lx", config);
}

static void s32cc_pinconf_group_dbg_show(struct pinctrl_dev *pctldev,
					 struct seq_file *s, unsigned int group)
{
	struct s32cc_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32cc_pinctrl_soc_info *info = ipctl->info;
	struct s32cc_pin_group *grp;
	unsigned long config;
	const char *name;
	int i, ret;

	if (group > info->ngroups) {
		seq_printf(s, "Warning: %d group > num pinctrl groups, %d\n",
			   group, info->ngroups);
		return;
	}

	seq_puts(s, "\n");
	grp = &info->groups[group];
	for (i = 0; i < grp->npins; i++) {
		struct s32cc_pin *pin = &grp->pins[i];

		name = pin_get_name(pctldev, pin->pin_id);
		ret = s32cc_pinconf_get(pctldev, pin->pin_id, &config);
		if (ret)
			return;
		seq_printf(s, "%s: 0x%lx\n", name, config);
	}
}

static const struct pinconf_ops s32cc_pinconf_ops = {
	.pin_config_get = s32cc_pinconf_get,
	.pin_config_set	= s32cc_pinconf_set,
	.pin_config_group_set = s32cc_pconf_group_set,
	.pin_config_dbg_show = s32cc_pinconf_dbg_show,
	.pin_config_group_dbg_show = s32cc_pinconf_group_dbg_show,
};

static struct pinctrl_desc s32cc_pinctrl_desc = {
	.pctlops = &s32cc_pctrl_ops,
	.pmxops = &s32cc_pmx_ops,
	.confops = &s32cc_pinconf_ops,
	.owner = THIS_MODULE,
};

#ifdef CONFIG_PM_SLEEP
static bool s32cc_pinctrl_should_save(struct s32cc_pinctrl *ipctl,
				      unsigned int pin)
{
	const struct pin_desc *pd = pin_desc_get(ipctl->pctl, pin);

	if (!pd)
		return false;

	/*
	 * Only restore the pin if it is actually in use by the kernel (or
	 * by userspace).
	 */
	if (pd->mux_owner || pd->gpio_owner)
		return true;

	return false;
}

int s32cc_pinctrl_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s32cc_pinctrl *ipctl = platform_get_drvdata(pdev);
	const struct pinctrl_pin_desc *pin;
	const struct s32cc_pinctrl_soc_info *info = ipctl->info;
	struct s32cc_pinctrl_context *saved_context = &ipctl->saved_context;
	int i;
	int ret;
	unsigned long config;

	for (i = 0; i < info->npins; i++) {
		pin = &info->pins[i];

		if (!s32cc_pinctrl_should_save(ipctl, pin->number))
			continue;

		ret = s32cc_pinctrl_readl(ipctl->pctl, pin->number, &config);
		if (ret)
			return -EINVAL;

		saved_context->pads[i] = config;
	}

	return 0;
}

int s32cc_pinctrl_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s32cc_pinctrl *ipctl = platform_get_drvdata(pdev);
	const struct s32cc_pinctrl_soc_info *info = ipctl->info;
	const struct pinctrl_pin_desc *pin;
	struct s32cc_pinctrl_context *saved_context = &ipctl->saved_context;
	int ret, i;

	for (i = 0; i < info->npins; i++) {
		pin = &info->pins[i];

		if (!s32cc_pinctrl_should_save(ipctl, pin->number))
			continue;

		ret = s32cc_pinctrl_writel(saved_context->pads[i],
					   ipctl->pctl, pin->number);
		if (ret)
			return ret;
	}

	return 0;
}
#endif

static int s32cc_pinctrl_parse_groups(struct device_node *np,
				    struct s32cc_pin_group *grp,
				    struct s32cc_pinctrl_soc_info *info,
				    u32 index)
{
	const __be32 *p;
	struct device *dev;
	struct property *prop;
	int i, npins;
	u32 pinmux = 0U;

	dev = info->dev;

	dev_dbg(dev, "group(%d): %s\n", index, np->name);

	/* Initialise group */
	grp->name = np->name;

	npins = of_property_count_elems_of_size(np, "pinmux", sizeof(u32));
	if (npins < 0) {
		dev_err(dev, "Failed to read 'pinmux' property in node %s.\n",
			np->name);
	} else if (!npins) {
		dev_err(dev, "The group %s has no pins.\n", np->name);
		return -EINVAL;
	}
	grp->npins = npins;

	grp->pins = devm_kzalloc(info->dev,
				 grp->npins * sizeof(struct s32cc_pin),
				 GFP_KERNEL);
	grp->pin_ids = devm_kzalloc(info->dev,
				    grp->npins * sizeof(unsigned int),
				    GFP_KERNEL);
	if (!grp->pins || !grp->pin_ids)
		return -ENOMEM;

	i = 0;
	of_property_for_each_u32(np, "pinmux", prop, p, pinmux) {
		struct s32cc_pin *pin = &grp->pins[i];

		pin->pin_id = get_pin_no(pinmux);
		pin->sss = get_pin_func(pinmux);
		grp->pin_ids[i] = grp->pins[i].pin_id;

		dev_dbg(info->dev, "%s: 0x%08lx",
			pin_get_name_from_info(info, pin->pin_id), pin->sss);
		i++;
	}

	return 0;
}

static int s32cc_pinctrl_parse_functions(struct device_node *np,
				       struct s32cc_pinctrl_soc_info *info,
				       u32 index)
{
	struct device_node *child;
	struct s32cc_pmx_func *func;
	struct s32cc_pin_group *grp;
	u32 i = 0;

	dev_dbg(info->dev, "parse function(%d): %s\n", index, np->name);

	func = &info->functions[index];

	/* Initialise function */
	func->name = np->name;
	func->num_groups = of_get_child_count(np);
	if (func->num_groups == 0) {
		dev_err(info->dev, "no groups defined in %s\n", np->full_name);
		return -EINVAL;
	}
	func->groups = devm_kzalloc(info->dev,
			func->num_groups * sizeof(char *), GFP_KERNEL);

	for_each_child_of_node(np, child) {
		if (info->grp_index >= info->ngroups) {
			dev_err(info->dev, "Invalid grp_index: %d\n", info->grp_index);
			return -EINVAL;
		}
		func->groups[i] = child->name;
		grp = &info->groups[info->grp_index++];
		s32cc_pinctrl_parse_groups(child, grp, info, i++);
	}

	return 0;
}

static int s32cc_pinctrl_probe_dt(struct platform_device *pdev,
				struct s32cc_pinctrl *ipctl)
{
	struct s32cc_pinctrl_soc_info *info = ipctl->info;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	struct resource *res;
	int mem_regions;
	u32 start, end;
	u32 nfuncs = 0;
	u32 i = 0;
	int ret;

	if (!np)
		return -ENODEV;

	mem_regions = of_property_count_elems_of_size(np, S32CC_NXP_PINS,
						      sizeof(u32));
	if (mem_regions < 0)
		return mem_regions;

	/* The array is composed of (start, end) pairs */
	mem_regions /= 2;

	info->mem_regions = mem_regions;

	ipctl->regions = devm_kzalloc(&pdev->dev,
				      mem_regions * sizeof(*ipctl->regions),
				      GFP_KERNEL);
	if (!ipctl->regions)
		return -ENOMEM;

	for (i = 0; i < mem_regions; ++i) {
		ret = of_property_read_u32_index(np, S32CC_NXP_PINS, i * 2,
						 &start);
		if (ret) {
			dev_err(&pdev->dev, "%s error\n", S32CC_NXP_PINS);
			return -EINVAL;
		}

		ret = of_property_read_u32_index(np, S32CC_NXP_PINS, i * 2 + 1,
						 &end);
		if (ret) {
			dev_err(&pdev->dev, "%s error\n", S32CC_NXP_PINS);
			return -EINVAL;
		}

		ipctl->regions[i].start_pin = start;
		ipctl->regions[i].end_pin = end;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		ipctl->regions[i].base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(ipctl->regions[i].base))
			return PTR_ERR(ipctl->regions[i].base);
	}

	nfuncs = of_get_child_count(np);
	if (nfuncs <= 0) {
		dev_err(&pdev->dev, "no functions defined\n");
		return -EINVAL;
	}

	info->nfunctions = nfuncs;
	info->functions = devm_kzalloc(&pdev->dev,
				       nfuncs * sizeof(struct s32cc_pmx_func),
				       GFP_KERNEL);
	if (!info->functions)
		return -ENOMEM;

	info->ngroups = 0;
	for_each_child_of_node(np, child)
		info->ngroups += of_get_child_count(child);
	info->groups = devm_kzalloc(&pdev->dev,
				    info->ngroups * sizeof(struct s32cc_pin_group),
				    GFP_KERNEL);
	if (!info->groups)
		return -ENOMEM;

	i = 0;
	for_each_child_of_node(np, child)
		s32cc_pinctrl_parse_functions(child, info, i++);

	return 0;
}

int s32cc_pinctrl_probe(struct platform_device *pdev,
		      struct s32cc_pinctrl_soc_info *info)
{
	struct s32cc_pinctrl *ipctl;
	int ret;
	struct pinctrl_desc *desc;
#ifdef CONFIG_PM_SLEEP
	struct s32cc_pinctrl_context *saved_context;
#endif

	if (!info || !info->pins || !info->npins) {
		dev_err(&pdev->dev, "wrong pinctrl info\n");
		return -EINVAL;
	}

	info->dev = &pdev->dev;

	/* Create state holders etc for this driver */
	ipctl = devm_kzalloc(&pdev->dev, sizeof(*ipctl), GFP_KERNEL);
	if (!ipctl)
		return -ENOMEM;
	ipctl->info = info;

	desc = devm_kmalloc(&pdev->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	memcpy(desc, &s32cc_pinctrl_desc, sizeof(*desc));

	desc->name = dev_name(&pdev->dev);
	desc->pins = info->pins;
	desc->npins = info->npins;

	ret = s32cc_pinctrl_probe_dt(pdev, ipctl);
	if (ret) {
		dev_err(&pdev->dev, "fail to probe dt properties\n");
		return ret;
	}

	ipctl->dev = info->dev;
	platform_set_drvdata(pdev, ipctl);
	ipctl->pctl = pinctrl_register(desc, &pdev->dev, ipctl);
	if (IS_ERR(ipctl->pctl)) {
		dev_err(&pdev->dev, "Could not register s32cc pinctrl driver\n");
		return PTR_ERR(ipctl->pctl);
	}

	INIT_LIST_HEAD(&ipctl->gpio_configs);
	spin_lock_init(&ipctl->gpio_configs_lock);
	spin_lock_init(&ipctl->reg_lock);

#ifdef CONFIG_PM_SLEEP
	saved_context = &ipctl->saved_context;
	saved_context->pads =
		devm_kcalloc(&pdev->dev, info->npins,
			     sizeof(*saved_context->pads),
			     GFP_KERNEL);
	if (!saved_context->pads)
		return -ENOMEM;
#endif

	dev_info(&pdev->dev, "Initialized s32cc pinctrl driver\n");

	return 0;
}

int s32cc_pinctrl_remove(struct platform_device *pdev)
{
	struct list_head *pos, *tmp;
	struct gpio_pin_config *gpio_pin;
	struct s32cc_pinctrl *ipctl = platform_get_drvdata(pdev);
	unsigned long flags;

	pinctrl_unregister(ipctl->pctl);

	spin_lock_irqsave(&ipctl->gpio_configs_lock, flags);

	list_for_each_safe(pos, tmp, &ipctl->gpio_configs) {
		gpio_pin = list_entry(pos, struct gpio_pin_config, list);
		list_del(pos);
		kfree(gpio_pin);
	}

	spin_unlock_irqrestore(&ipctl->gpio_configs_lock, flags);

	return 0;
}
