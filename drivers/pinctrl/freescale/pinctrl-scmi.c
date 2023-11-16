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
#include <linux/hashtable.h>
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

#define SCMI_PINCTRL_HASHTABLE_SIZE	7

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
 * @no_groups: the number of groups
 */
struct scmi_pinctrl_func {
	const char *name;
	const char **groups;
	unsigned int no_groups;
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
 * struct scmi_pinctrl_pm_conf - hash node for with a pin's pinconf
 *				 as the key
 * @pcf: the current pinconf of the pins
 * @node: used by the hashtable
 */
struct scmi_pinctrl_pm_conf {
	struct scmi_pinctrl_pinconf key;
	struct hlist_node node;
	struct scmi_pinctrl_pin_list list;
};

/**
 * struct scmi_pinctrl_pm_pin: holds suspend/resume information for a pin.
 * @function: the function of a pin
 * @function_initialized: is the function field valid
 * @pcf: the current pinconf configuration of a pin
 * @pcf_initialized: is the pcf field valid
 */
struct scmi_pinctrl_pm_pin {
	u16 function;
	struct scmi_pinctrl_pinconf pcf;
	bool function_initialized;
	bool pcf_initialized;
};

/**
 * struct scmi_pinctrl_pm - holds necessary information for resuming after STR.
 * @pcfs: hashtable equivalent to (pcf, u16 *pins_w_that_pcf)
 * @pcfs_lock: lock for accessing the pcfs hashtable
 */
struct scmi_pinctrl_pm {
	DECLARE_HASHTABLE(pcfs, SCMI_PINCTRL_HASHTABLE_SIZE);
	struct mutex pcfs_lock;
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
 * @no_ranges: the number of pin_ranges elements
 * @pm_info: information needed for suspend/resume
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
	u16 no_ranges;
	struct scmi_pinctrl_pm pm_info;
	struct mutex pins_lock;
};

/**
 * struct scmi_pinctrl_gpio_config
 * @id: the pin/gpio id (they are the same)
 * @func: the previous function for the pin
 * @no_configs: the number of entries in the configs array
 * @configs: the previous pin_configs set
 */
struct scmi_pinctrl_gpio_config {
	struct list_head list;
	unsigned int id;
	u16 func;
	unsigned int no_configs;
	unsigned long *configs;
};

static int scmi_pinctrl_pinconf_common_set(struct pinctrl_dev *pctldev,
					   unsigned int *pins,
					   unsigned int no_pins,
					   unsigned long *configs,
					   unsigned int no_configs,
					   bool override);

static int scmi_pinctrl_pin_to_pindesc_index(struct pinctrl_dev *pctldev,
					     u16 pin,
					     u16 *index)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	u16 start, end;
	int i;

	*index = 0;

	for (i = 0; i < priv->no_ranges; i++) {
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

static void *scmi_pinctrl_get_pcf(struct pinctrl_dev *pctldev,
				  struct scmi_pinctrl_pinconf *pcf)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_pm_conf *temp, *ret;
	unsigned int hash = scmi_pinctrl_hash_pcf(pcf);

	ret = NULL;
	hash_for_each_possible(priv->pm_info.pcfs, temp, node, hash) {
		if (scmi_pinctrl_are_pcfs_equal(&temp->key, pcf)) {
			ret = temp;
			break;
		}
	}

	return ret;
}

static void scmi_pinctrl_store_pcf(struct pinctrl_dev *pctldev,
				  struct scmi_pinctrl_pinconf *pcf, void *data)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_pm_conf *pm_pcf = data;

	hash_add(priv->pm_info.pcfs, &pm_pcf->node,
		 scmi_pinctrl_hash_pcf(pcf));
}

static int scmi_pinctrl_allocate_pcf(struct pinctrl_dev *pctldev,
				     struct scmi_pinctrl_pm_pin *pm_pin,
				     struct scmi_pinctrl_pm_conf **pm_pcf)
{
	struct scmi_pinctrl_pm_conf *tmp_pm_pcf;

	tmp_pm_pcf = devm_kmalloc(pctldev->dev, sizeof(*tmp_pm_pcf),
				  GFP_KERNEL);
	if (!tmp_pm_pcf)
		return -ENOMEM;

	tmp_pm_pcf->key = pm_pin->pcf;
	tmp_pm_pcf->key.multi_bit_values =
		devm_kmemdup(pctldev->dev, pm_pin->pcf.multi_bit_values,
			     scmi_pinctrl_mb_configs_size(tmp_pm_pcf->key.mask),
			     GFP_KERNEL);
	if (!tmp_pm_pcf->key.multi_bit_values) {
		devm_kfree(pctldev->dev, tmp_pm_pcf);
		return -ENOMEM;
	}

	scmi_pinctrl_pin_list_init(&tmp_pm_pcf->list);
	*pm_pcf = tmp_pm_pcf;

	return 0;
}

/* Must be called with pins and pcfs locks acquired. */
static int scmi_pinctrl_populate_new_config(struct pinctrl_dev *pctldev,
					    u16 pin,
					    struct scmi_pinctrl_pm_pin *pm_pin,
					    struct scmi_pinctrl_pinconf *pcf,
					    bool override)
{
	struct scmi_pinctrl_pm_conf *pm_pcf = NULL;
	struct scmi_pinctrl_pin_list_elem *p;
	bool new_config = false;
	int ret;

	p = devm_kmalloc(pctldev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	ret = scmi_pinctrl_add_pcf(pctldev->dev, GFP_KERNEL, &pm_pin->pcf, pcf,
				   override);
	if (ret)
		goto err_free;

	pm_pcf = scmi_pinctrl_get_pcf(pctldev, &pm_pin->pcf);
	if (!pm_pcf) {
		ret = scmi_pinctrl_allocate_pcf(pctldev, pm_pin, &pm_pcf);
		if (ret)
			goto err_free;

		new_config = true;
	}

	p->pin = pin;
	ret = scmi_pinctrl_pin_list_add_pin(&pm_pcf->list, p);
	if (ret)
		goto err_free;

	pm_pin->pcf_initialized = true;

	if (new_config)
		scmi_pinctrl_store_pcf(pctldev, &pm_pcf->key, pm_pcf);

	return 0;

err_free:
	devm_kfree(pctldev->dev, p);

	return ret;
}

static int scmi_pinctrl_update_pcf(struct pinctrl_dev *pctldev, u16 pin,
				   struct scmi_pinctrl_pinconf *pcf,
				   bool override)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_pm *pm = &priv->pm_info;
	struct scmi_pinctrl_pm_conf *pm_pcf = NULL;
	struct scmi_pinctrl_pm_pin *pm_pin = NULL;
	struct scmi_pinctrl_pin_list_elem *p;
	int ret = 0;
	u16 index;

	ret = scmi_pinctrl_pin_to_pindesc_index(pctldev, pin, &index);
	if (ret)
		return ret;

	mutex_lock(&priv->pins_lock);
	mutex_lock(&pm->pcfs_lock);

	pm_pin = pctldev->desc->pins[index].drv_data;

	if (!pm_pin->pcf_initialized)
		goto populate_new_config;

	if (scmi_pinctrl_are_pcfs_equal(&pm_pin->pcf, pcf))
		goto unlock;

	pm_pcf = scmi_pinctrl_get_pcf(pctldev, &pm_pin->pcf);
	if (!pm_pcf) {
		dev_err(pctldev->dev, "Could not find pcf: %X for pin: %d\n",
			pm_pin->pcf.mask, pin);
		ret = -EINVAL;
		goto unlock;
	}

	p = scmi_pinctrl_pin_list_remove_pin(&pm_pcf->list, pin);
	if (!p)
		goto unlock;
	devm_kfree(pctldev->dev, p);

populate_new_config:
	ret = scmi_pinctrl_populate_new_config(pctldev, pin, pm_pin, pcf,
					       override);

unlock:
	mutex_unlock(&pm->pcfs_lock);
	mutex_unlock(&priv->pins_lock);

	return ret;
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
					     unsigned int *no_maps,
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

	ret = pinctrl_utils_reserve_map(pctldev, map, reserved_maps, no_maps,
					reserve);
	if (ret < 0)
		goto free_cfgs;

	ret = pinctrl_utils_add_map_mux(pctldev, map, reserved_maps, no_maps,
					np->name, func_name);
	if (ret < 0)
		goto free_cfgs;

	if (n_cfgs) {
		ret = pinctrl_utils_add_map_configs(pctldev, map, reserved_maps,
						    no_maps, np->name, cfgs, n_cfgs,
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
				       unsigned int *no_maps)
{
	unsigned int reserved_maps;
	struct device_node *np;
	int ret = 0;

	reserved_maps = 0;
	*map = NULL;
	*no_maps = 0;

	for_each_available_child_of_node(np_config, np) {
		ret = scmi_pinctrl_dt_group_node_to_map(pctldev,
							np,
							map,
							&reserved_maps,
							no_maps,
							np_config->name);
		if (ret < 0)
			break;
	}

	if (ret)
		pinctrl_utils_free_map(pctldev, *map, *no_maps);

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
				       unsigned int * const no_groups)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	*no_groups = priv->desc.functions[selector].no_groups;
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

	gpio_config->no_configs = hweight32(pcf.mask);
	gpio_config->configs = devm_kmalloc(pctldev->dev,
					    (gpio_config->no_configs *
					     sizeof(*gpio_config->configs)),
					    GFP_KERNEL);
	if (!gpio_config->configs) {
		ret = -ENOMEM;
		goto err_free_pcf;
	}

	ret = scmi_pinctrl_convert_from_pcf(gpio_config->configs,
					    &pcf);
	if (ret) {
		dev_err(pctldev->dev, "Error converting from pcf!\n");
		goto err_free_configs;
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
		goto err_restore_function;
	}

	save_gpio_config(priv, gpio_config);

	return 0;

err_restore_function:
	(void)scmi_pinctrl_save_pin_function(pctldev, offset,
					     gpio_config->func);
err_free_configs:
	devm_kfree(pctldev->dev, gpio_config->configs);
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
	bool found = false;
	int ret;

	if (offset > U16_MAX)
		return;

	mutex_lock(&priv->gpios_list_lock);
	list_for_each_entry_safe(gpio_config, tmp, &priv->gpios_list, list) {
		if (gpio_config->id == offset) {
			list_del(&gpio_config->list);
			found = true;
			break;
		}
	}
	mutex_unlock(&priv->gpios_list_lock);

	if (!found) {
		dev_err(pctldev->dev, "Error finding gpio config for pin: %d\n",
			offset);
		return;
	}

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

	ret = scmi_pinctrl_pinconf_common_set(pctldev, &offset, 1,
					      gpio_config->configs,
					      gpio_config->no_configs,
					      true);
	if (ret)
		dev_err(pctldev->dev,
			"Failed to restore pinconf values for GPIO: %d!\n",
			offset);

	devm_kfree(pctldev->dev, gpio_config->configs);
	devm_kfree(pctldev->dev, gpio_config);
}

static int scmi_pinctrl_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
					       struct pinctrl_gpio_range *range,
					       unsigned int offset, bool input)
{
	unsigned long configs[] = {
		/* We always have the input buffer enabled, even if
		 * we have an output GPIO. This way we can read the
		 * actual value on the line to check it against the
		 * value that the GPIO was supposed to have.
		 */
		pinconf_to_config_packed(PIN_CONFIG_INPUT_ENABLE, 1),
		pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, !input)
	};
	int ret;

	if (offset > U16_MAX)
		return -EINVAL;

	ret = scmi_pinctrl_pinconf_common_set(pctldev, &offset, 1, configs,
					      ARRAY_SIZE(configs), false);
	if (ret)
		dev_err(pctldev->dev, "Error setting pinconf!\n");

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

	ret = scmi_pinctrl_convert_from_pcf(configs, &pcf);
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

static int scmi_pinctrl_pinconf_common_set(struct pinctrl_dev *pctldev,
					   unsigned int *pins,
					   unsigned int no_pins,
					   unsigned long *configs,
					   unsigned int no_configs,
					   bool override)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_pin_list tmp_list;
	struct scmi_pinctrl_pin_list_elem *tmp_pins;
	struct scmi_pinctrl_pinconf pcf;
	unsigned int no_mb_cfgs;
	int ret = -EINVAL, i;
	u16 pin;

	scmi_pinctrl_pin_list_init(&tmp_list);

	for (i = 0; i < no_pins; i++)
		if (pins[i] > U16_MAX)
			goto err;

	no_mb_cfgs =
		scmi_pinctrl_count_multi_bit_values(configs, no_configs);

	pcf.multi_bit_values = devm_kmalloc(pctldev->dev,
					    (no_mb_cfgs *
					     sizeof(*pcf.multi_bit_values)),
					    GFP_KERNEL);
	if (!pcf.multi_bit_values) {
		ret = -ENOMEM;
		goto err;
	}

	ret = scmi_pinctrl_create_pcf(configs, no_configs, &pcf);
	if (ret) {
		dev_err(pctldev->dev, "Error converting to pcf!\n");
		goto err_free;
	}

	tmp_pins = devm_kmalloc(pctldev->dev, no_pins * sizeof(*tmp_pins),
				  GFP_KERNEL);
	if (!tmp_pins) {
		ret = -ENOMEM;
		goto err_free;
	}

	for (i = 0; i < no_pins; i++) {
		pin = pins[i];

		ret = scmi_pinctrl_update_pcf(pctldev, pin, &pcf, override);
		if (ret) {
			dev_err(pctldev->dev,
				"Error saving pinconf for pin: %d!\n",
				pins[i]);
			break;
		}

		tmp_pins[i].pin = pin;
		ret = scmi_pinctrl_pin_list_add_pin(&tmp_list,
						    &tmp_pins[i]);
		if (ret)
			break;
	}

	if (ret)
		goto err_tmp;

	ret = priv->pinctrl_ops->pinconf_set(priv->ph, &tmp_list, &pcf,
					     override);
	if (ret)
		dev_err(pctldev->dev, "Error setting pinconf!\n");

err_tmp:
	devm_kfree(pctldev->dev, tmp_pins);
err_free:
	devm_kfree(pctldev->dev, pcf.multi_bit_values);
err:
	return ret;
}

static int scmi_pinctrl_pinconf_set(struct pinctrl_dev *pctldev,
				    unsigned int offset,
				    unsigned long *configs,
				    unsigned int no_configs)
{
	if (offset > U16_MAX)
		return -EINVAL;

	return scmi_pinctrl_pinconf_common_set(pctldev, &offset, 1, configs,
					       no_configs,
					       false);
}

static int scmi_pinctrl_pconf_group_set(struct pinctrl_dev *pctldev,
					unsigned int group,
					unsigned long *configs,
					unsigned int no_configs)
{
	struct scmi_pinctrl_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct scmi_pinctrl_desc *desc = &priv->desc;

	return scmi_pinctrl_pinconf_common_set(pctldev,
					       desc->groups[group].pin_ids,
					       desc->groups[group].npins,
					       configs,
					       no_configs,
					       true);
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
	func->no_groups = of_get_child_count(np);
	if (!func->no_groups) {
		dev_err(dev, "no groups defined in %s\n", np->full_name);
		return -EINVAL;
	}
	func->groups = devm_kzalloc(dev,
				    func->no_groups * sizeof(char *),
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

	priv->no_ranges = priv->pinctrl_ops->get_no_ranges(ph);
	ranges = devm_kmalloc(dev, sizeof(*ranges) * priv->no_ranges, GFP_KERNEL);
	if (!ranges)
		return -ENOMEM;

	ret = priv->pinctrl_ops->describe(ph, ranges);
	if (ret)
		return ret;

	priv->pin_ranges = ranges;

	for (i = 0; i < priv->no_ranges; ++i)
		pin_count += priv->pin_ranges[i].no_pins;

	INIT_LIST_HEAD(&priv->gpios_list);
	mutex_init(&priv->gpios_list_lock);
	mutex_init(&priv->pins_lock);
	mutex_init(&priv->pm_info.pcfs_lock);
	hash_init(priv->pm_info.pcfs);

	desc->npins = pin_count;
	pin_desc = devm_kcalloc(dev, pin_count, sizeof(*pin_desc), GFP_KERNEL);
	if (!pin_desc)
		return -ENOMEM;

	desc->pins = pin_desc;

	for (i = 0; i < priv->no_ranges; ++i) {
		for (j = 0; j < priv->pin_ranges[i].no_pins; ++j) {
			pin_id = priv->pin_ranges[i].start + j;
			pin_name = devm_kasprintf(dev, GFP_KERNEL, "PIN_%d",
						  pin_id);
			if (!pin_name)
				return -ENOMEM;

			pm_pin = devm_kzalloc(dev, sizeof(*pm_pin), GFP_KERNEL);
			if (!pm_pin)
				return -ENOMEM;
			pm_pin->pcf_initialized = false;
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

static int scmi_pinctrl_resume(struct scmi_device *sdev)
{
	struct scmi_pinctrl_priv *priv = dev_get_drvdata(&sdev->dev);
	struct pinctrl_dev *pctldev = priv->pctldev;
	unsigned int it, i, count = 0, index = 0;
	struct scmi_pinctrl_pin_function *pf;
	struct scmi_pinctrl_pm_conf *pm_pcf;
	struct scmi_pinctrl_pm_pin *pm_pin;
	struct scmi_pinctrl_pinconf *pcf;
	int ret = 0;

	mutex_lock(&priv->pins_lock);

	for (i = 0; i < priv->pctldev->desc->npins; i++) {
		pm_pin = priv->pctldev->desc->pins[i].drv_data;
		if (pm_pin->function_initialized)
			count++;
	}

	if (count > U16_MAX) {
		dev_err(pctldev->dev, "Too many pins to resume: %d!\n", count);
		goto unlock_pins;
	}

	pf = kmalloc(sizeof(*pf) * count, GFP_KERNEL);
	if (!pf) {
		ret = -ENOMEM;
		goto unlock_pins;
	}

	for (i = 0; i < priv->pctldev->desc->npins; i++) {
		pm_pin = priv->pctldev->desc->pins[i].drv_data;
		if (!pm_pin->function_initialized)
			continue;
		pf[index].pin = priv->pctldev->desc->pins[i].number;
		pf[index++].function = pm_pin->function;
	}

	ret = priv->pinctrl_ops->pinmux_set(priv->ph, count, pf);
	if (ret) {
		dev_err(pctldev->dev, "Error resuming pinmux: %d!\n", ret);
		goto free_pf;
	}

	mutex_lock(&priv->pm_info.pcfs_lock);
	hash_for_each(priv->pm_info.pcfs, it, pm_pcf, node) {
		pcf = &pm_pcf->key;

		ret = priv->pinctrl_ops->pinconf_set(priv->ph, &pm_pcf->list,
						     pcf, true);
		if (ret) {
			dev_err(pctldev->dev,
				"Error resuming pinconf: %d!\n",
				ret);
			break;
		}
	}

	mutex_unlock(&priv->pm_info.pcfs_lock);
free_pf:
	kfree(pf);
unlock_pins:
	mutex_unlock(&priv->pins_lock);

	return ret;
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_ID_PINCTRL, "pinctrl" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver scmi_pinctrl_driver = {
	.name = "scmi-pinctrl",
	.probe = scmi_pinctrl_probe,
	.resume = scmi_pinctrl_resume,
	.id_table = scmi_id_table,
};
module_scmi_driver(scmi_pinctrl_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP SCMI Pinctrl driver");
MODULE_LICENSE("Dual BSD/GPL");
