// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * SCMI pinctrl Protocol - NXP vendor extension
 *
 * Copyright 2022-2023 NXP
 */

#define pr_fmt(fmt) "SCMI pinctrl - " fmt

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/scmi_protocol.h>
#include <linux/scmi_pinctrl_protocol.h>

#include <linux/sort.h>

#include "common.h"
#include "notify.h"

/*
 * Remaining message size is 100. The longer messages that use a variable
 * number of pins are PINMUX_SET and PICONF_SET_*.
 *
 * PINMUX_SET would allow for maximum (100 - 4) / 4 = 24
 * PINCONF_SET_* would allow for maximum (100 - 4 - 4 - 4 - 8 * 4) / 2 = 28
 *
 * Because of ALB-10137 we cannot use the maximum size. Switch to 23
 * until it's fixed.
 *
 */
#define PINMUX_MAX_PINS 23

/*
 * SCMI Pinctrl commands
 */
enum scmi_pinctrl_protocol_cmd {
	PINCTRL_DESCRIBE = 0x3,
	PINCTRL_PINMUX_GET = 0x4,
	PINCTRL_PINMUX_SET = 0x5,
	PINCTRL_PINCONF_GET = 0x6,
	PINCTRL_PINCONF_SET_OVERRIDE = 0x7,
	PINCTRL_PINCONF_SET_APPEND = 0x8,

	PINCTRL_NUM_COMMANDS
};

struct scmi_msg_resp_pinctrl_attributes {
	__le16 num_ranges;
};

struct scmi_msg_resp_pinctrl_describe {
	struct {
		__le16 start;
		__le16 no_pins;
	} range[0];
};

struct scmi_msg_pinctrl_pmx_get {
	__le16 pin;
};

struct scmi_msg_resp_pinctrl_pmx_get {
	__le16 function;
};

struct scmi_pin_function {
	__le16 pin;
	__le16 function;
};

struct scmi_msg_pinctrl_pmx_set {
	__le32 no_pins;
	struct scmi_pin_function settings[];
};

struct scmi_pinctrl_info {
	u32 version;
	u16 num_ranges;
};

struct scmi_msg_pinctrl_pcf_get {
	__le16 pin;
};

struct scmi_msg_resp_pinctrl_pcf_get {
	__le32 mask;
	__le32 boolean_values;
	__le32 multi_bit_values[];
};

struct scmi_msg_pinctrl_pcf_set_pins {
	__le32 no_pins;
	__le16 pins[];
};

struct scmi_msg_pinctrl_pcf_set_pcf {
	__le32 mask;
	__le32 boolean_values;
	__le32 multi_bit_values[];
};

static bool is_multi_bit_value(enum pin_config_param p)
{
	return !!(SCMI_PINCTRL_MULTI_BIT_CFGS & BIT(p));
}

void scmi_pinctrl_pin_list_init(struct scmi_pinctrl_pin_list *list)
{
	INIT_LIST_HEAD(&list->list);
}

static int scmi_pinctrl_pin_list_pin_index(struct scmi_pinctrl_pin_list *list,
					   u16 pin)
{
	struct scmi_pinctrl_pin_list_elem *it;
	u32 index = 0;

	list_for_each_entry(it, &list->list, list) {
		if (pin == it->pin)
			return index;

		if (index == S32_MAX)
			return -EINVAL;
		index++;
	}

	return -EINVAL;
}

struct scmi_pinctrl_pin_list_elem *
	scmi_pinctrl_pin_list_remove_pin(struct scmi_pinctrl_pin_list *list,
					 u16 pin)
{
	struct scmi_pinctrl_pin_list_elem *it, *tmp;

	list_for_each_entry_safe(it, tmp, &list->list, list) {
		if (it->pin == pin) {
			list_del(&it->list);
			return it;
		}
	}

	return NULL;
}

int scmi_pinctrl_pin_list_add_pin(struct scmi_pinctrl_pin_list *list,
				  struct scmi_pinctrl_pin_list_elem *p)
{
	if (scmi_pinctrl_pin_list_pin_index(list, p->pin) >= 0) {
		pr_err("Pin: %d already present!\n", p->pin);
		return -EINVAL;
	}

	list_add(&p->list, &list->list);

	return 0;
}

unsigned int scmi_pinctrl_count_multi_bit_values(unsigned long *configs,
						 unsigned int num_configs)
{
	unsigned int i, count = 0;

	for (i = 0; i < num_configs; ++i)
		if (is_multi_bit_value(pinconf_to_config_param(configs[i])))
			++count;

	return count;
}

static int compare_configs(const void *a, const void *b)
{
	int pa, pb;

	pa = pinconf_to_config_param(*(enum pin_config_param *)a);
	pb = pinconf_to_config_param(*(enum pin_config_param *)b);

	return pb - pa;
}

static void set_boolean_value(struct scmi_pinctrl_pinconf *pcf,
			      enum pin_config_param param,
			      bool enable)
{

	if (param >= BITS_PER_BYTE * sizeof(pcf->boolean_values))
		return;

	pcf->boolean_values &= ~BIT(param);
	if (enable)
		pcf->boolean_values |= BIT(param);
}

int scmi_pinctrl_create_pcf(unsigned long *configs,
			    unsigned int num_configs,
			    struct scmi_pinctrl_pinconf *pcf)
{
	unsigned int i, multi_bit_idx = 0;
	enum pin_config_param param;
	int ret = 0;
	u32 arg;

	if (!pcf->multi_bit_values)
		return -EINVAL;

	pcf->mask = 0;
	pcf->boolean_values = 0;

	/* Sorting needs to be done in order to lay out
	 * the configs in descending order of their
	 * pinconf parameter value which matches
	 * the protocol specification.
	 */

	sort(configs, num_configs, sizeof(*configs), compare_configs, NULL);

	for (i = 0; i < num_configs; ++i) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		if (param >= BITS_PER_BYTE * sizeof(pcf->mask)) {
			ret = -EINVAL;
			break;
		}

		pcf->mask |= BIT(param);

		if (is_multi_bit_value(param)) {
			if (multi_bit_idx == U32_MAX) {
				ret = -EINVAL;
				break;
			}
			pcf->multi_bit_values[multi_bit_idx++] = arg;
		} else {
			set_boolean_value(pcf, param, !!arg);
		}
	}

	return ret;
}

int scmi_pinctrl_convert_from_pcf(unsigned long **configs,
				  struct scmi_pinctrl_pinconf *pcf)
{
	unsigned int index = 0, m_idx = 0;
	unsigned long bit, mask = pcf->mask, cfg;

	for_each_set_bit(bit, &mask, sizeof(pcf->mask) * BITS_PER_BYTE) {
		if (is_multi_bit_value((enum pin_config_param)bit))
			cfg = PIN_CONF_PACKED(bit,
					      pcf->multi_bit_values[m_idx++]);
		else
			cfg = PIN_CONF_PACKED(bit,
					      pcf->boolean_values & BIT(bit));
		(*configs)[index++] = cfg;
	}

	return 0;
}

bool scmi_pinctrl_are_pcfs_equal(struct scmi_pinctrl_pinconf *pcfa,
				 struct scmi_pinctrl_pinconf *pcfb)
{
	if (pcfa->mask != pcfb->mask)
		return false;

	if (pcfa->boolean_values != pcfb->boolean_values)
		return false;

	return !memcmp(pcfa->multi_bit_values, pcfb->multi_bit_values,
		       scmi_pinctrl_mb_configs_size(pcfa->mask));
}

unsigned int scmi_pinctrl_hash_pcf(struct scmi_pinctrl_pinconf *pcf)
{
	unsigned int hash = 0, i;

	hash |= pcf->mask ^ pcf->boolean_values;
	for (i = 0; i < scmi_pinctrl_count_mb_configs(pcf->mask); i++)
		hash ^= pcf->multi_bit_values[i];

	return hash;
}

static unsigned int scmi_pinctrl_get_mb_index(struct scmi_pinctrl_pinconf *pcf,
					      enum pin_config_param param)
{
	/* The index is represented by the number of set bits, in positions
	 * that correspond to multi_bit_values, that occur before(MSB to LSB)
	 * the one at the position corresponding to param.
	 */
	uint32_t higher_bits;
	u8 max_pos;

	max_pos = sizeof(pcf->mask) * BITS_PER_BYTE - 1;
	if (param + 1 > max_pos)
		return 0;

	higher_bits = pcf->mask & GENMASK(max_pos, param + 1);
	higher_bits &= SCMI_PINCTRL_MULTI_BIT_CFGS;

	return hweight32(higher_bits);
}

int scmi_pinctrl_add_mb_to_pcf(struct device *dev,
			       gfp_t flags,
			       struct scmi_pinctrl_pinconf *pcf,
			       enum pin_config_param param,
			       u32 value)
{
	u32 *temp, count = scmi_pinctrl_count_mb_configs(pcf->mask);
	unsigned int index = scmi_pinctrl_get_mb_index(pcf, param);

	temp = devm_krealloc(dev, pcf->multi_bit_values,
			     (count + 1) * sizeof(*pcf->multi_bit_values),
			     flags);
	if (!temp)
		return -ENOMEM;

	memmove(temp + index + 1, temp + index,
		(count - index) * sizeof(*temp));
	temp[index] = value;
	pcf->multi_bit_values = temp;

	return 0;
}

int scmi_pinctrl_add_pcf(struct device *dev,
			 gfp_t flags,
			 struct scmi_pinctrl_pinconf *res,
			 struct scmi_pinctrl_pinconf *src,
			 bool override)
{
	unsigned long mask = src->mask, b;
	unsigned int index, mb_index = 0;
	enum pin_config_param p;
	u32 *temp, v;
	int ret;

	if (override) {
		res->mask = src->mask;
		res->boolean_values = src->boolean_values;
		temp = devm_krealloc(dev,
				     res->multi_bit_values,
				     scmi_pinctrl_mb_configs_size(src->mask),
				     flags);
		if (!temp)
			return -ENOMEM;

		res->multi_bit_values = temp;

		memcpy(res->multi_bit_values, src->multi_bit_values,
		       scmi_pinctrl_mb_configs_size(src->mask));

		return 0;
	}

	for_each_set_bit(b, &mask, sizeof(src->mask) * BITS_PER_BYTE) {
		p = (enum pin_config_param)b;
		if (!is_multi_bit_value(p)) {
			res->mask |= BIT(b);
			set_boolean_value(res, p,
					  !!(src->boolean_values & BIT(b)));
		} else if (res->mask & BIT(b)) {
			index = scmi_pinctrl_get_mb_index(res, p);
			res->multi_bit_values[index] =
				src->multi_bit_values[mb_index++];
		} else {
			res->mask |= BIT(b);
			v = src->multi_bit_values[mb_index++];
			ret = scmi_pinctrl_add_mb_to_pcf(dev, flags, res, p, v);

			if (ret)
				return ret;
		}
	}

	return 0;
}

static int scmi_pinctrl_attributes_get(const struct scmi_protocol_handle *ph,
				       struct scmi_pinctrl_info *pinfo)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_msg_resp_pinctrl_attributes *attr;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES, 0, sizeof(*attr),
				      &t);
	if (ret) {
		dev_err(ph->dev, "Error initializing xfer!\n");
		return ret;
	}

	attr = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret)
		pinfo->num_ranges = le16_to_cpu(attr->num_ranges);
	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_pinctrl_protocol_describe(const struct scmi_protocol_handle *ph,
					  struct scmi_pinctrl_pin_range *rv)
{
	struct scmi_msg_resp_pinctrl_describe *ranges;
	struct scmi_pinctrl_info *pinfo;
	struct scmi_xfer *t;
	int ret, i;

	pinfo = ph->get_priv(ph);
	if (!pinfo)
		return -ENODEV;

	ret = ph->xops->xfer_get_init(ph, PINCTRL_DESCRIBE, 0, 0, &t);
	if (ret) {
		dev_err(ph->dev, "Error initializing xfer!\n");
		return ret;
	}

	ranges = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (ret) {
		dev_err(ph->dev, "Transfer error!\n");
		goto done;
	}

	for (i = 0; i < pinfo->num_ranges; ++i) {
		rv[i].start = le16_to_cpu(ranges->range[i].start);
		rv[i].no_pins = le16_to_cpu(ranges->range[i].no_pins);
	}

done:
	ph->xops->xfer_put(ph, t);

	return ret;
}

static int
scmi_pinctrl_protocol_pinmux_get(const struct scmi_protocol_handle *ph, u16 pin,
				 u16 *func)
{
	struct scmi_msg_resp_pinctrl_pmx_get *rv;
	struct scmi_msg_pinctrl_pmx_get *params;
	size_t tx_size, rx_size;
	struct scmi_xfer *t;
	int ret;

	tx_size = sizeof(struct scmi_msg_pinctrl_pmx_get);
	rx_size = sizeof(struct scmi_msg_resp_pinctrl_pmx_get);

	ret = ph->xops->xfer_get_init(ph, PINCTRL_PINMUX_GET, tx_size, rx_size,
				      &t);
	if (ret) {
		dev_err(ph->dev, "Error initializing xfer!\n");
		return ret;
	}

	params = t->tx.buf;
	rv = t->rx.buf;

	params->pin = cpu_to_le16(pin);

	ret = ph->xops->do_xfer(ph, t);
	if (ret) {
		dev_err(ph->dev, "Error getting pinmux %d!\n", ret);
		goto end;
	}

	*func = le16_to_cpu(rv->function);

end:
	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_pinctrl_pinmux_set_chunk(const struct scmi_protocol_handle *ph,
					 u32 no_pins,
					 const struct scmi_pinctrl_pin_function *pf)
{
	struct scmi_msg_pinctrl_pmx_set *params;
	struct scmi_xfer *t;
	unsigned int i;
	size_t tx_size;
	int ret;

	if (no_pins > U8_MAX)
		return -EINVAL;

	tx_size = sizeof(*params) +
		  no_pins * sizeof(params->settings[0]);

	ret = ph->xops->xfer_get_init(ph, PINCTRL_PINMUX_SET, tx_size, 0, &t);
	if (ret) {
		dev_err(ph->dev, "Error initializing xfer!\n");
		return -EOPNOTSUPP;
	}

	params = t->tx.buf;
	params->no_pins = cpu_to_le32(no_pins);

	for (i = 0; i < no_pins; ++i) {
		params->settings[i].pin = cpu_to_le16(pf[i].pin);
		params->settings[i].function = cpu_to_le16(pf[i].function);
	}

	ret = ph->xops->do_xfer(ph, t);
	if (ret) {
		dev_err(ph->dev, "Error setting pinmux!\n");
		ret = -EOPNOTSUPP;
		goto err;
	}

err:
	ph->xops->xfer_put(ph, t);
	return ret;

}

static int
scmi_pinctrl_protocol_pinmux_set(const struct scmi_protocol_handle *ph,
				 u16 no_pins,
				 const struct scmi_pinctrl_pin_function *pf)
{
	unsigned int i, off = 0;
	int ret = 0;

	for (i = 0; i < no_pins / PINMUX_MAX_PINS; ++i) {
		ret = scmi_pinctrl_pinmux_set_chunk(ph, PINMUX_MAX_PINS,
						    pf + off);
		if (ret)
			return ret;

		off += PINMUX_MAX_PINS;
	}

	if (no_pins % PINMUX_MAX_PINS != 0)
		ret = scmi_pinctrl_pinmux_set_chunk(ph,
						    no_pins % PINMUX_MAX_PINS,
						    pf + off);

	return ret;
}

static int
scmi_pinctrl_add_multi_bit_values(const struct scmi_protocol_handle *ph,
				  struct scmi_pinctrl_pinconf *pcf,
				  struct scmi_msg_resp_pinctrl_pcf_get *rv)
{
	unsigned int b = sizeof(pcf->mask) * BITS_PER_BYTE - 1;
	unsigned int mb_idx = 0;
	u32 v;

	do {
		if (!(pcf->mask & BIT(b)))
			continue;

		if (!is_multi_bit_value((enum pin_config_param)b))
			continue;

		v = le32_to_cpu(rv->multi_bit_values[mb_idx]);
		pcf->multi_bit_values[mb_idx++] = v;
	} while (b-- != 0);

	return 0;
}

static int
scmi_pinctrl_protocol_pinconf_get(const struct scmi_protocol_handle *ph,
				  u16 pin,
				  struct scmi_pinctrl_pinconf *pcf)
{
	struct scmi_msg_resp_pinctrl_pcf_get *rv;
	struct scmi_msg_pinctrl_pcf_get *params;
	struct scmi_xfer *t;
	int ret;

	if (!pcf->multi_bit_values)
		return -EINVAL;

	pcf->mask = 0;
	pcf->boolean_values = 0;

	ret = ph->xops->xfer_get_init(ph, PINCTRL_PINCONF_GET,
				      sizeof(struct scmi_msg_pinctrl_pcf_get),
				      0, &t);
	if (ret) {
		dev_err(ph->dev, "Error setting pinmux!\n");
		return -EOPNOTSUPP;
	}

	params = t->tx.buf;
	rv = t->rx.buf;
	params->pin = cpu_to_le16(pin);

	ret = ph->xops->do_xfer(ph, t);
	if (ret) {
		dev_err(ph->dev, "Error getting pinconf!\n");
		goto err;
	}

	pcf->mask = le32_to_cpu(rv->mask);
	pcf->boolean_values = le32_to_cpu(rv->boolean_values);

	ret = scmi_pinctrl_add_multi_bit_values(ph, pcf, rv);
err:
	ph->xops->xfer_put(ph, t);

	return ret;
}

static int
scmi_pinctrl_pinconf_set_chunk(const struct scmi_protocol_handle *ph,
			       u32 no_pins,
			       u16 *pins,
			       struct scmi_pinctrl_pinconf *pcf,
			       bool override)
{
	u32 multi_bit_count = scmi_pinctrl_count_mb_configs(pcf->mask);
	struct scmi_msg_pinctrl_pcf_set_pcf *param_pcf;
	struct scmi_msg_pinctrl_pcf_set_pins *param_pins;
	struct scmi_xfer *t;
	int ret, i = 0;
	u8 msg_id;
	size_t tx_size;

	if (override)
		msg_id = PINCTRL_PINCONF_SET_OVERRIDE;
	else
		msg_id = PINCTRL_PINCONF_SET_APPEND;

	tx_size = sizeof(struct scmi_msg_pinctrl_pcf_set_pcf);
	if (check_add_overflow(tx_size,
			       scmi_pinctrl_mb_configs_size(pcf->mask),
			       &tx_size)) {
		dev_err(ph->dev, "Error computing tx_size!\n");
		return -EINVAL;
	}

	if (check_add_overflow(tx_size,
			       sizeof(struct scmi_msg_pinctrl_pcf_set_pins),
			       &tx_size)) {
		dev_err(ph->dev, "Error computing tx_size!\n");
		return -EINVAL;
	}

	if (check_add_overflow(tx_size, sizeof(__le16) * no_pins, &tx_size)) {
		dev_err(ph->dev, "Error computing tx_size!\n");
		return -EINVAL;
	}

	ret = ph->xops->xfer_get_init(ph, msg_id, tx_size, 0, &t);
	if (ret) {
		dev_err(ph->dev, "Error initializing transfer!\n");
		return -EOPNOTSUPP;
	}

	param_pins = t->tx.buf;
	param_pins->no_pins = cpu_to_le32(no_pins);
	for (i = 0; i < no_pins; i++)
		param_pins->pins[i] = cpu_to_le16(pins[i]);

	param_pcf = (void *)(param_pins->pins + i);
	param_pcf->mask = cpu_to_le32(pcf->mask);
	param_pcf->boolean_values = cpu_to_le32(pcf->boolean_values);

	for (i = 0; i < multi_bit_count; ++i)
		param_pcf->multi_bit_values[i] =
			cpu_to_le32(pcf->multi_bit_values[i]);

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		dev_err(ph->dev, "Error setting pinconf!\n");

	ph->xops->xfer_put(ph, t);

	return ret;

}

static int
scmi_pinctrl_protocol_pinconf_set(const struct scmi_protocol_handle *ph,
				  struct scmi_pinctrl_pin_list *pins,
				  struct scmi_pinctrl_pinconf *pcf,
				  bool override)
{
	struct scmi_pinctrl_pin_list_elem *it;
	u16 pins_array[PINMUX_MAX_PINS];
	u32 no_pins = 0;
	int ret = 0;

	memset(pins_array, 0, sizeof(pins_array));

	list_for_each_entry(it, &pins->list, list) {
		if (no_pins == PINMUX_MAX_PINS) {
			ret = scmi_pinctrl_pinconf_set_chunk(ph,
							     PINMUX_MAX_PINS,
							     pins_array, pcf,
							     override);
			if (ret)
				return ret;

			no_pins = 0;
		}

		pins_array[no_pins++] = it->pin;
	}

	if (no_pins) {
		ret = scmi_pinctrl_pinconf_set_chunk(ph,
						     no_pins,
						     pins_array, pcf,
						     override);
		if (ret)
			return ret;
	}

	return ret;
}

static u16
scmi_pinctrl_protocol_get_num_ranges(const struct scmi_protocol_handle *ph)
{
	struct scmi_pinctrl_info *pinfo = ph->get_priv(ph);

	return pinfo->num_ranges;
}

static int scmi_pinctrl_protocol_init(const struct scmi_protocol_handle *ph)
{
	struct scmi_pinctrl_info *pinfo;
	u32 version;
	int ret;

	ret = ph->xops->version_get(ph, &version);
	if (ret)
		return ret;

	dev_info(ph->dev, "pinctrl Version %u.%u\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	pinfo = devm_kzalloc(ph->dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	pinfo->version = version;
	ret = ph->set_priv(ph, pinfo);
	if (ret) {
		dev_err(ph->dev, "Failed to set priv: %d!\n", ret);
		return ret;
	}

	ret = scmi_pinctrl_attributes_get(ph, pinfo);
	if (ret) {
		dev_err(ph->dev, "Error getting protocol attributes!\n");
		return ret;
	}

	return 0;
}

static const struct scmi_pinctrl_proto_ops pinctrl_proto_ops = {
	.describe = scmi_pinctrl_protocol_describe,
	.pinmux_get = scmi_pinctrl_protocol_pinmux_get,
	.pinmux_set = scmi_pinctrl_protocol_pinmux_set,
	.pinconf_get = scmi_pinctrl_protocol_pinconf_get,
	.pinconf_set = scmi_pinctrl_protocol_pinconf_set,
	.get_num_ranges = scmi_pinctrl_protocol_get_num_ranges,
};

static const struct scmi_protocol scmi_pinctrl = {
	.id = SCMI_PROTOCOL_ID_PINCTRL,
	.owner = THIS_MODULE,
	.instance_init = &scmi_pinctrl_protocol_init,
	.ops = &pinctrl_proto_ops,
};

static int __init scmi_pinctrl_register(void)
{
	return scmi_protocol_register(&scmi_pinctrl);
}
arch_initcall_sync(scmi_pinctrl_register);

static void __exit scmi_pinctrl_unregister(void)
{
	scmi_protocol_unregister(&scmi_pinctrl);
}
module_exit(scmi_pinctrl_unregister);
