// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 *  Copyright 2022-2023 NXP
 */
#include "asm-generic/errno.h"
#include "linux/overflow.h"
#include <linux/err.h>
#include <linux/module.h>
#include <linux/scmi_protocol.h>
#include <linux/scmi_pinctrl_protocol.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/mutex.h>
#include <linux/limits.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"

#define SCMI_PINCTRL_PIN_NO_SHIFT	4
#define SCMI_PINCTRL_GPIO_FUNC		0

/**
 * struct scmi_pinctrl_pin_group - describes a pin group
 * @name: the name of the group
 * @npins: the number of pins in the group
 * @pin_ids: an array with each of the pin's id
 * @pin_funcs: the mux configuration for the group
 */
struct scmi_pinctrl_pin_group {
	const char *name;
	unsigned int npins;
	unsigned int *pin_ids;
	unsigned int *pin_funcs;
};

/**
 * struct scmi_pinctrl_func - describes a function pinmux functions
 * @name: the name of this specific function
 * @groups: corresponding pin groups
 * @num_groups: the number of groups
 */
struct scmi_pinctrl_func {
	const char *name;
	const char **groups;
	unsigned int num_groups;
};

/**
 * struct scmi_pinctrl_desc - describes the groups and functions of the
 *			      current instance
 * @nfunctions: the number of elements in the functions array
 * @ngroups: the number of elements in the groups array
 * @functions: the array of functions described in the .dts file
 * @goups: the array containing all the groups described in the .dts file
 */
struct scmi_pinctrl_desc {
	unsigned int nfunctions;
	unsigned int ngroups;
	struct scmi_pinctrl_func *functions;
	struct scmi_pinctrl_pin_group *groups;
};


/**
 * struct scmi_pinctrl_pm_pin: holds suspend/resume information for a pin.
 * @function: the function of a pin
 * @function_initialized: is the function field valid
 */
struct scmi_pinctrl_pm_pin {
	u16 function;
	bool function_initialized;
};

/**
 * struct scmi_pinctrl_priv - private internal data for the pinctrl_dev
 * @pctldev: a reference to the registered pinctrl device
 * @pinctrl_ops: the pinctrl ops over SCMI
 * @pin_ranges: the available MSCRs/IMCRs to use
 * @ph: the SCMI protocol handle to be used
 * @desc: information about the available groups and functions
 * @gpios_list: a list with the previous values of a pin before
 *		configuring it as a GPIO.
 * @gpios_list_lock: lock protecting the gpios_list
 * @num_ranges: the number of pin_ranges elements
 * @pins_lock: lock for accessing the pins
 */
struct scmi_pinctrl_priv {
	struct pinctrl_dev *pctldev;
	const struct scmi_pinctrl_proto_ops *pinctrl_ops;
	const struct scmi_pinctrl_pin_range *pin_ranges;
	struct scmi_protocol_handle *ph;
	struct scmi_pinctrl_desc desc;
	struct list_head gpios_list;
	struct mutex gpios_list_lock;
	u16 num_ranges;
	struct mutex pins_lock;
};

/**
 * struct scmi_pinctrl_gpio_config
 * @id: the pin/gpio id (they are the same)
 * @func: the previous function for the pin
 * @num_configs: the number of entries in the configs array
 * @configs: the previous pin_configs set
 */
struct scmi_pinctrl_gpio_config {
	struct list_head list;
	unsigned int id;
	u16 func;
	unsigned int num_configs;
	unsigned long *configs;
};

static int scmi_pinctrl_pin_to_pindesc_index(struct pinctrl_dev *pctldev,
					     u16 pin,
					     u16 *index)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	u16 start, end;
	int i;

	*index = 0;

	for (i = 0; i < priv->num_ranges; i++) {
		start = priv->pin_ranges[i].start;
		end = start + priv->pin_ranges[i].no_pins;

		if (pin >= start && pin < end) {
			if (check_add_overflow(*index, (u16)(pin - start),
					       index))
				return -EINVAL;
			return 0;
		}

		if (check_add_overflow(*index, priv->pin_ranges[i].no_pins,
				       index))
			return -EINVAL;
	}

	return -EINVAL;
}

static u32 get_pin_no(u32 pinmux)
{
	return pinmux >> SCMI_PINCTRL_PIN_NO_SHIFT;
}

static u32 get_pin_func(u32 pinmux)
{
	return pinmux & GENMASK(3, 0);
}

static int scmi_pinctrl_save_pin_function(struct pinctrl_dev *pctldev, u16 pin,
					  u16 func)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_pm_pin *pm_pin = NULL;
	u16 index;
	int ret;

	ret = scmi_pinctrl_pin_to_pindesc_index(pctldev, pin, &index);
	if (ret)
		return ret;

	mutex_lock(&priv->pins_lock);

	pm_pin = pctldev->desc->pins[index].drv_data;
	pm_pin->function = func;
	pm_pin->function_initialized = true;

	mutex_unlock(&priv->pins_lock);

	return 0;
}

static int scmi_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	return priv->desc.ngroups;
}

static const char *scmi_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
					       unsigned int selector)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	return priv->desc.groups[selector].name;
}

static int scmi_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
				       unsigned int selector,
				       const unsigned int **pins,
				       unsigned int *npins)
{

	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	*npins = priv->desc.groups[selector].npins;
	*pins = priv->desc.groups[selector].pin_ids;

	return 0;
}

static int scmi_pinctrl_dt_group_node_to_map(struct pinctrl_dev *pctldev,
					     struct device_node *np,
					     struct pinctrl_map **map,
					     unsigned int *reserved_maps,
					     unsigned int *num_maps,
					     const char *func_name)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	unsigned long *cfgs = NULL;
	unsigned int n_cfgs, reserve = 1;
	struct device *dev;
	int n_pins, ret;

	if (!priv)
		return -EINVAL;

	dev = pctldev->dev;

	n_pins = of_property_count_elems_of_size(np, "pinmux", sizeof(u32));
	if (n_pins < 0) {
		dev_err(dev, "Failed to read 'pinmux' property in node %s.\n",
			np->name);
	} else if (!n_pins) {
		return -EINVAL;
	}

	ret = pinconf_generic_parse_dt_config(np, pctldev, &cfgs, &n_cfgs);
	if (ret) {
		dev_err(dev, "Could not parse node %s pinconf properties\n",
			np->name);
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

static int scmi_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
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
		ret = scmi_pinctrl_dt_group_node_to_map(pctldev,
							np,
							map,
							&reserved_maps,
							num_maps,
							np_config->name);
		if (ret < 0)
			break;
	}

	if (ret)
		pinctrl_utils_free_map(pctldev, *map, *num_maps);

	return ret;
}

static const struct pinctrl_ops scmi_pinctrl_pctrl_ops = {
	.get_groups_count = scmi_pinctrl_get_groups_count,
	.get_group_name = scmi_pinctrl_get_group_name,
	.get_group_pins = scmi_pinctrl_get_group_pins,
	.dt_node_to_map = scmi_pinctrl_dt_node_to_map,
	.dt_free_map = pinctrl_utils_free_map,
};

static int scmi_pinctrl_pmx_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	return priv->desc.nfunctions;
}

static const char *scmi_pinctrl_pmx_get_func_name(struct pinctrl_dev *pctldev,
					  unsigned int selector)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	return priv->desc.functions[selector].name;
}

static int scmi_pinctrl_pmx_get_groups(struct pinctrl_dev *pctldev,
				       unsigned int selector,
				       const char * const **groups,
				       unsigned int * const num_groups)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	*num_groups = priv->desc.functions[selector].num_groups;
	*groups = priv->desc.functions[selector].groups;

	return 0;
}

static int scmi_pinctrl_pmx_set(struct pinctrl_dev *pctldev,
				unsigned int selector,
				unsigned int group)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_pin_group *grp = &priv->desc.groups[group];
	struct scmi_pinctrl_pin_function *pf;
	int ret = 0, i;

	pf = devm_kcalloc(pctldev->dev, grp->npins, sizeof(*pf), GFP_KERNEL);
	if (!pf) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < grp->npins; ++i) {
		if (grp->pin_ids[i] > U16_MAX ||
		    grp->pin_funcs[i] > U16_MAX) {
			ret = -EINVAL;
			break;
		}
		pf[i].pin = grp->pin_ids[i];
		pf[i].function = grp->pin_funcs[i];

		ret = scmi_pinctrl_save_pin_function(pctldev, pf[i].pin,
						     pf[i].function);
		if (ret)
			break;
	}

	if (ret)
		goto err_free;

	ret = priv->pinctrl_ops->pinmux_set(priv->ph, grp->npins, pf);
	if (ret) {
		dev_err(pctldev->dev, "Failed to set group pinmux!\n");
		goto err_free;
	}

err_free:
	devm_kfree(pctldev->dev, pf);
err:
	return ret;
}

static void save_gpio_config(struct scmi_pinctrl_priv *priv,
			     struct scmi_pinctrl_gpio_config *cfg)
{
	mutex_lock(&priv->gpios_list_lock);
	list_add(&cfg->list, &priv->gpios_list);
	mutex_unlock(&priv->gpios_list_lock);
}

static int scmi_pinctrl_pmx_gpio_request_enable(struct pinctrl_dev *pctldev,
						struct pinctrl_gpio_range *r,
						unsigned int offset)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_gpio_config *gpio_config;
	struct scmi_pinctrl_pin_function pf;
	struct scmi_pinctrl_pinconf pcf;
	unsigned int max_mb_configs;
	int ret = -EINVAL;

	if (offset > U16_MAX)
		goto err;

	max_mb_configs = hweight32(SCMI_PINCTRL_MULTI_BIT_CFGS);

	gpio_config = devm_kmalloc(pctldev->dev, sizeof(*gpio_config),
				   GFP_KERNEL);
	if (!gpio_config) {
		ret = -ENOMEM;
		dev_err(pctldev->dev, "Error allocating gpio_config!\n");
		goto err;
	}
	gpio_config->id = offset;

	ret = priv->pinctrl_ops->pinmux_get(priv->ph, offset,
					    &gpio_config->func);
	if (ret) {
		dev_err(pctldev->dev, "Error getting pin mux!\n");
		goto err_free;
	}

	pcf.multi_bit_values = devm_kmalloc(pctldev->dev,
					    (max_mb_configs *
					     sizeof(*pcf.multi_bit_values)),
					    GFP_KERNEL);
	if (!pcf.multi_bit_values) {
		ret = -ENOMEM;
		goto err_free;
	}

	ret = priv->pinctrl_ops->pinconf_get(priv->ph, offset, &pcf);
	if (ret) {
		dev_err(pctldev->dev, "Error getting configs!\n");
		goto err_free_pcf;
	}

	gpio_config->configs = devm_kmalloc(pctldev->dev,
					    (hweight32(pcf.mask) *
					     sizeof(*gpio_config->configs)),
					    GFP_KERNEL);
	if (!gpio_config->configs) {
		ret = -ENOMEM;
		goto err_free_pcf;
	}

	ret = scmi_pinctrl_convert_from_pcf(&gpio_config->configs,
					    &pcf);
	if (ret) {
		dev_err(pctldev->dev, "Error converting from pcf!\n");
		goto err_free_pcf;
	}

	ret = scmi_pinctrl_save_pin_function(pctldev, offset,
					     SCMI_PINCTRL_GPIO_FUNC);
	if (ret)
		goto err_free_configs;

	pf.pin = offset;
	pf.function = SCMI_PINCTRL_GPIO_FUNC;
	ret = priv->pinctrl_ops->pinmux_set(priv->ph, 1, &pf);
	if (ret) {
		dev_err(pctldev->dev, "Error configuring the GPIO!\n");
		goto err_free_configs;
	}

	save_gpio_config(priv, gpio_config);

	return 0;

err_free_configs:
	kfree(gpio_config->configs);
err_free_pcf:
	devm_kfree(pctldev->dev, pcf.multi_bit_values);
err_free:
	devm_kfree(pctldev->dev, gpio_config);
err:
	return ret;
}

static void scmi_pinctrl_pmx_gpio_disable_free(struct pinctrl_dev *pctldev,
					       struct pinctrl_gpio_range *range,
					       unsigned int offset)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_gpio_config *gpio_config, *tmp;
	struct scmi_pinctrl_pin_function pf;
	struct scmi_pinctrl_pinconf pcf;
	unsigned int no_mb_cfgs;
	int ret;

	if (offset > U16_MAX)
		return;

	mutex_lock(&priv->gpios_list_lock);
	list_for_each_entry_safe(gpio_config, tmp, &priv->gpios_list, list) {
		if (gpio_config->id == offset) {
			list_del(&gpio_config->list);
			break;
		}
	}
	mutex_unlock(&priv->gpios_list_lock);

	if (!gpio_config)
		return;

	ret = scmi_pinctrl_save_pin_function(pctldev, offset,
					     gpio_config->func);
	if (ret) {
		save_gpio_config(priv, gpio_config);
		return;
	}

	pf.pin = offset;
	pf.function = gpio_config->func;
	ret = priv->pinctrl_ops->pinmux_set(priv->ph, 1, &pf);
	if (ret) {
		dev_err(pctldev->dev, "Failed to set pin function!\n");
		save_gpio_config(priv, gpio_config);
		return;
	}

	no_mb_cfgs =
		scmi_pinctrl_count_multi_bit_values(gpio_config->configs,
						    gpio_config->num_configs);

	pcf.multi_bit_values = devm_kmalloc(pctldev->dev,
					    (no_mb_cfgs *
					     sizeof(*pcf.multi_bit_values)),
					    GFP_KERNEL);
	if (!pcf.multi_bit_values)
		goto err_free;

	ret = scmi_pinctrl_create_pcf(gpio_config->configs,
				      gpio_config->num_configs,
				      &pcf);
	if (ret) {
		dev_err(pctldev->dev, "Failed to convert pinconf setting!\n");
		goto err_pcf;
	}

	ret = priv->pinctrl_ops->pinconf_set(priv->ph, offset, &pcf, true);
	if (ret)
		dev_err(pctldev->dev, "Failed to set pinconf values!\n");

err_pcf:
	devm_kfree(pctldev->dev, pcf.multi_bit_values);
err_free:
	kfree(gpio_config->configs);
	devm_kfree(pctldev->dev, gpio_config);
}

static int scmi_pinctrl_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
					       struct pinctrl_gpio_range *range,
					       unsigned int offset, bool input)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_pinconf pcf;
	unsigned long configs[] = {
		/* We always have the input buffer enabled, even if
		 * we have an output GPIO. This way we can read the
		 * actual value on the line to check it against the
		 * value that the GPIO was supposed to have.
		 */
		pinconf_to_config_packed(PIN_CONFIG_INPUT_ENABLE, 1),
		pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, !input)
	};
	unsigned int no_mb_cfgs;
	int ret;

	if (offset > U16_MAX)
		return -EINVAL;

	no_mb_cfgs =
		scmi_pinctrl_count_multi_bit_values(configs,
						    ARRAY_SIZE(configs));

	pcf.multi_bit_values = devm_kmalloc(pctldev->dev,
					    (no_mb_cfgs *
					     sizeof(*pcf.multi_bit_values)),
					    GFP_KERNEL);
	if (!pcf.multi_bit_values) {
		ret = -ENOMEM;
		goto err;
	}

	ret = scmi_pinctrl_create_pcf(configs, ARRAY_SIZE(configs), &pcf);
	if (ret) {
		dev_err(pctldev->dev, "Error converting to pcf!\n");
		goto err_pcf;
	}

	ret = priv->pinctrl_ops->pinconf_set(priv->ph, offset, &pcf, false);
	if (ret)
		dev_err(pctldev->dev, "Error setting pinconf!\n");

err_pcf:
	devm_kfree(pctldev->dev, pcf.multi_bit_values);
err:
	return ret;
}

static const struct pinmux_ops scmi_pinctrl_pmx_ops = {
	.get_functions_count = scmi_pinctrl_pmx_get_funcs_count,
	.get_function_name = scmi_pinctrl_pmx_get_func_name,
	.get_function_groups = scmi_pinctrl_pmx_get_groups,
	.set_mux = scmi_pinctrl_pmx_set,
	.gpio_request_enable = scmi_pinctrl_pmx_gpio_request_enable,
	.gpio_disable_free = scmi_pinctrl_pmx_gpio_disable_free,
	.gpio_set_direction = scmi_pinctrl_pmx_gpio_set_direction,
};

static bool is_config_disabled(enum pin_config_param p, u32 arg)
{
	switch (p) {
	case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
	case PIN_CONFIG_INPUT_ENABLE:
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
	case PIN_CONFIG_OUTPUT_ENABLE:
		if (arg == 0)
			return true;
	default:
		break;
	}

	return false;
}

static int scmi_pinctrl_pinconf_get(struct pinctrl_dev *pctldev,
				    unsigned int offset, unsigned long *config)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config), p;
	struct scmi_pinctrl_pinconf pcf;
	unsigned long *configs;
	unsigned int max_mb_configs;
	int ret, i;
	u32 arg;

	if (offset > U16_MAX) {
		ret = -EINVAL;
		goto err;
	}

	max_mb_configs = hweight32(SCMI_PINCTRL_MULTI_BIT_CFGS);

	pcf.multi_bit_values = devm_kmalloc(pctldev->dev,
					    (max_mb_configs *
					     sizeof(*pcf.multi_bit_values)),
					    GFP_KERNEL);
	if (!pcf.multi_bit_values) {
		ret = -ENOMEM;
		goto err;
	}

	ret = priv->pinctrl_ops->pinconf_get(priv->ph, offset, &pcf);
	if (ret) {
		dev_err(pctldev->dev, "Error getting config!\n");
		goto err_pcf;
	}

	configs = devm_kmalloc(pctldev->dev,
			       hweight32(pcf.mask) * sizeof(*configs),
			       GFP_KERNEL);
	if (!configs) {
		ret = -ENOMEM;
		goto err_pcf;
	}

	ret = scmi_pinctrl_convert_from_pcf(&configs, &pcf);
	if (ret) {
		dev_err(pctldev->dev, "Error converting pcf!\n");
		goto err_cfgs;
	}

	ret = -EOPNOTSUPP;
	for (i = 0; i < hweight32(pcf.mask); ++i) {
		p = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);
		if (param == p) {
			*config = PIN_CONF_PACKED(p, arg);
			ret = 0;
			if (is_config_disabled(p, arg))
				ret = -EINVAL;
			break;
		}
	}

err_cfgs:
	kfree(configs);
err_pcf:
	devm_kfree(pctldev->dev, pcf.multi_bit_values);
err:
	return ret;
}

static int scmi_pinctrl_pinconf_set(struct pinctrl_dev *pctldev,
				    unsigned int offset,
				    unsigned long *configs,
				    unsigned int num_configs)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_pinconf pcf;
	unsigned int no_mb_cfgs;
	int ret = -EINVAL;

	if (offset > U16_MAX)
		goto err;

	no_mb_cfgs =
		scmi_pinctrl_count_multi_bit_values(configs, num_configs);

	pcf.multi_bit_values = devm_kmalloc(pctldev->dev,
					    (no_mb_cfgs *
					     sizeof(*pcf.multi_bit_values)),
					    GFP_KERNEL);
	if (!pcf.multi_bit_values) {
		ret = -ENOMEM;
		goto err;
	}

	ret = scmi_pinctrl_create_pcf(configs, num_configs, &pcf);
	if (ret) {
		dev_err(pctldev->dev, "Error converting to pcf!\n");
		goto err_free;
	}

	ret = priv->pinctrl_ops->pinconf_set(priv->ph, offset, &pcf, false);
	if (ret)
		dev_err(pctldev->dev, "Error setting pinconf!\n");

err_free:
	devm_kfree(pctldev->dev, pcf.multi_bit_values);
err:
	return ret;
}

static int scmi_pinctrl_pconf_group_set(struct pinctrl_dev *pctldev,
					unsigned int group,
					unsigned long *configs,
					unsigned int num_configs)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_desc *desc = &priv->desc;
	struct scmi_pinctrl_pinconf pcf;
	unsigned int offset, no_mb_cfgs;
	int ret, i;

	no_mb_cfgs =
		scmi_pinctrl_count_multi_bit_values(configs, num_configs);

	pcf.multi_bit_values = devm_kmalloc(pctldev->dev,
					    (no_mb_cfgs *
					     sizeof(*pcf.multi_bit_values)),
					    GFP_KERNEL);
	if (!pcf.multi_bit_values) {
		ret = -ENOMEM;
		goto err;
	}

	ret = scmi_pinctrl_create_pcf(configs, num_configs, &pcf);
	if (ret) {
		dev_err(pctldev->dev, "Error converting to pcf!\n");
		goto err_pcf;
	}

	for (i = 0; i < desc->groups[group].npins; ++i) {
		offset = desc->groups[group].pin_ids[i];
		if (offset > U16_MAX)
			break;

		ret = priv->pinctrl_ops->pinconf_set(priv->ph, offset, &pcf, true);
		if (ret) {
			dev_err(pctldev->dev, "Error setting pinconf!\n");
			break;
		}
	}

err_pcf:
	devm_kfree(pctldev->dev, pcf.multi_bit_values);
err:
	return ret;
}

static const struct pinconf_ops scmi_pinctrl_pinconf_ops = {
	.pin_config_get = scmi_pinctrl_pinconf_get,
	.pin_config_set	= scmi_pinctrl_pinconf_set,
	.pin_config_group_set = scmi_pinctrl_pconf_group_set,
};

static int scmi_pinctrl_parse_groups(struct scmi_device *sdev,
				     struct device_node *np,
				     struct scmi_pinctrl_pin_group *grp,
				     u32 index)
{
	const __be32 *p;
	struct device *dev = &sdev->dev;
	struct property *prop;
	int i, npins;
	u32 pinmux = 0U;

	/* Initialise group */
	grp->name = np->name;

	npins = of_property_count_elems_of_size(np, "pinmux", sizeof(u32));
	if (npins < 0) {
		dev_err(dev, "Failed to read 'pinmux' property in node %s.\n",
			np->name);
		return -EINVAL;
	} else if (!npins) {
		dev_err(dev, "The group %s has no pins.\n", np->name);
		return -EINVAL;
	}
	grp->npins = npins;

	grp->pin_ids = devm_kzalloc(dev,
				    grp->npins * sizeof(*grp->pin_ids),
				    GFP_KERNEL);
	if (!grp->pin_ids) {
		dev_err(dev, "Can't allocate memory for pin_ids!\n");
		return -ENOMEM;
	}

	grp->pin_funcs = devm_kzalloc(dev,
				      grp->npins * sizeof(*grp->pin_funcs),
				      GFP_KERNEL);
	if (!grp->pin_funcs) {
		dev_err(dev, "Can't allocate memory for pin_funcs!\n");
		return -ENOMEM;
	}

	i = 0;
	of_property_for_each_u32(np, "pinmux", prop, p, pinmux) {
		grp->pin_ids[i] = get_pin_no(pinmux);
		grp->pin_funcs[i] = get_pin_func(pinmux);

		dev_dbg(dev, "PIN_%d: 0x%08x",
			grp->pin_ids[i],
			grp->pin_funcs[i]);
		++i;
	}

	return 0;
}

static int scmi_pinctrl_parse_functions(struct scmi_device *sdev,
					struct device_node *np,
					struct scmi_pinctrl_desc *desc,
					u32 index,
					u32 *grp_index)
{
	struct device_node *child;
	struct device *dev = &sdev->dev;
	struct scmi_pinctrl_func *func;
	struct scmi_pinctrl_pin_group *grp;
	u32 i = 0;
	int ret;

	func = &desc->functions[index];

	/* Initialise function */
	func->name = np->name;
	func->num_groups = of_get_child_count(np);
	if (!func->num_groups) {
		dev_err(dev, "no groups defined in %s\n", np->full_name);
		return -EINVAL;
	}
	func->groups = devm_kzalloc(dev,
				    func->num_groups * sizeof(char *),
				    GFP_KERNEL);

	for_each_child_of_node(np, child) {
		func->groups[i] = child->name;
		grp = &desc->groups[(*grp_index)++];
		ret = scmi_pinctrl_parse_groups(sdev, child, grp, i++);
		if (ret)
			return ret;
	}

	return 0;
}

static int scmi_pinctrl_probe_dt(struct scmi_device *sdev,
				 struct scmi_pinctrl_priv *priv)
{
	struct scmi_pinctrl_desc *desc = &priv->desc;
	struct device_node *np = sdev->dev.of_node;
	struct device_node *child;
	u32 i = 0, grp_idx = 0;

	if (!np)
		return -ENODEV;

	desc->nfunctions = 0;
	desc->ngroups = 0;
	for_each_child_of_node(np, child) {
		desc->nfunctions++;
		desc->ngroups += of_get_child_count(child);
	}

	desc->functions = devm_kzalloc(&sdev->dev,
				       (desc->nfunctions *
					sizeof(*desc->functions)),
				       GFP_KERNEL);
	if (!desc->functions)
		return -ENOMEM;


	desc->groups = devm_kzalloc(&sdev->dev,
				    desc->ngroups * sizeof(*desc->groups),
				    GFP_KERNEL);
	if (!desc->groups)
		return -ENOMEM;

	i = 0;
	for_each_child_of_node(np, child)
		scmi_pinctrl_parse_functions(sdev, child, desc, i++, &grp_idx);

	return 0;
}

static int scmi_pinctrl_probe(struct scmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	unsigned int pin_count = 0, i, j, pin = 0, pin_id;
	const struct scmi_pinctrl_proto_ops *pinctrl_ops;
	const struct scmi_handle *handle = sdev->handle;
	struct scmi_pinctrl_pin_range *ranges;
	struct scmi_pinctrl_pm_pin *pm_pin;
	struct pinctrl_pin_desc *pin_desc;
	struct scmi_protocol_handle *ph;
	struct scmi_pinctrl_priv *priv;
	struct pinctrl_desc *desc;
	char *pin_name;
	int ret;

	if (!handle) {
		dev_err(dev, "Could not get handle [protocol %u]\n",
			SCMI_PROTOCOL_ID_PINCTRL);
		return -EPROBE_DEFER;
	}

	priv = devm_kmalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	desc = devm_kmalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->name = dev_name(dev);

	pinctrl_ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_ID_PINCTRL,
						&ph);
	if (IS_ERR(pinctrl_ops)) {
		dev_err(dev, "Could not retrieve protocol ops.\n");
		return -EPROBE_DEFER;
	}

	priv->pinctrl_ops = pinctrl_ops;
	priv->ph = ph;

	priv->num_ranges = priv->pinctrl_ops->get_num_ranges(ph);
	ranges = devm_kmalloc(dev, sizeof(*ranges) * priv->num_ranges, GFP_KERNEL);
	if (!ranges)
		return -ENOMEM;

	ret = priv->pinctrl_ops->describe(ph, ranges);
	if (ret)
		return ret;

	priv->pin_ranges = ranges;

	for (i = 0; i < priv->num_ranges; ++i)
		pin_count += priv->pin_ranges[i].no_pins;

	INIT_LIST_HEAD(&priv->gpios_list);
	mutex_init(&priv->gpios_list_lock);
	mutex_init(&priv->pins_lock);

	desc->npins = pin_count;
	pin_desc = devm_kcalloc(dev, pin_count, sizeof(*pin_desc), GFP_KERNEL);
	if (!pin_desc) {
		dev_err(dev, "Could not allocate memory for pinctrl_desc!\n");
		return -ENOMEM;
	}
	desc->pins = pin_desc;

	for (i = 0; i < priv->num_ranges; ++i) {
		for (j = 0; j < priv->pin_ranges[i].no_pins; ++j) {
			pin_id = priv->pin_ranges[i].start + j;
			pin_name = devm_kasprintf(dev, GFP_KERNEL, "PIN_%d",
						  pin_id);
			if (!pin_name)
				return -ENOMEM;

			pm_pin = devm_kmalloc(dev, sizeof(*pm_pin), GFP_KERNEL);
			if (!pm_pin)
				return -ENOMEM;
			pm_pin->function_initialized = false;

			pin_desc[pin].number = pin_id;
			pin_desc[pin].name = pin_name;
			pin_desc[pin++].drv_data = pm_pin;
		}
	}

	ret = scmi_pinctrl_probe_dt(sdev, priv);
	if (ret) {
		dev_err(dev, "Failed to parse DT\n");
		return ret;
	}

	desc->pctlops = &scmi_pinctrl_pctrl_ops;
	desc->pmxops = &scmi_pinctrl_pmx_ops;
	desc->confops = &scmi_pinctrl_pinconf_ops;
	desc->owner = THIS_MODULE;

	dev_set_drvdata(dev, priv);

	priv->pctldev = devm_pinctrl_register(dev, desc, priv);
	if (IS_ERR(priv->pctldev)) {
		dev_err(dev, "Failed to register pinctrl driver: %ld!\n",
			PTR_ERR(priv->pctldev));
		return PTR_ERR(priv->pctldev);
	}

	return 0;
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_ID_PINCTRL, "pinctrl" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver scmi_pinctrl_driver = {
	.name = "scmi-pinctrl",
	.probe = scmi_pinctrl_probe,
	.id_table = scmi_id_table,
};
module_scmi_driver(scmi_pinctrl_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP SCMI Pinctrl driver");
MODULE_LICENSE("Dual BSD/GPL");
