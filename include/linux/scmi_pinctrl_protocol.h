/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * SCMI Pinctrl Protocol
 *
 * Copyright 2022-2023 NXP
 */
#ifndef SCMI_PINCTRL_PROTOCOL_H
#define SCMI_PINCTRL_PROTOCOL_H

#include <linux/list.h>
#include <linux/types.h>
#include <linux/scmi_protocol.h>
#include <linux/pinctrl/pinconf-generic.h>

/*
 * SCMI Pinctrl Protocol
 */
#define SCMI_PROTOCOL_ID_PINCTRL	U32_C(0x80)

/*
 * SCMI Pinctrl protocol version
 */
#define SCMI_PROTOCOL_PINCTRL_VERSION	U32_C(0x10000)

#define SCMI_PINCTRL_MULTI_BIT_CFGS					 \
	(BIT(PIN_CONFIG_SLEW_RATE) | BIT(PIN_CONFIG_SKEW_DELAY) |	 \
	 BIT(PIN_CONFIG_POWER_SOURCE) | BIT(PIN_CONFIG_MODE_LOW_POWER) | \
	 BIT(PIN_CONFIG_INPUT_SCHMITT) | BIT(PIN_CONFIG_INPUT_DEBOUNCE) |\
	 BIT(PIN_CONFIG_DRIVE_STRENGTH_UA)|				 \
	 BIT(PIN_CONFIG_DRIVE_STRENGTH))

struct scmi_pinctrl_pin_range {
	u16 start;
	u16 no_pins;
};

struct scmi_pinctrl_pin_function {
	u16 pin;
	u16 function;
};

struct scmi_pinctrl_pinconf {
	u32 mask;
	u32 boolean_values;
	u32 *multi_bit_values;
};

struct scmi_pinctrl_pin_list_elem {
	struct list_head list;
	u16 pin;
};

struct scmi_pinctrl_pin_list {
	struct list_head list;
};

/**
 * struct scmi_pinctrl_proto_ops - represents the various operations provided
 *	by the SCMI Pinctrl Protocol
 *
 * @describe: return the pin ranges available
 * @pinmux_get: return the current mux for the pin
 * @pinmux_set: set the function for a pin
 * @pinconf_get: return the pinconfig of a pin. Caller must call kfree on pcf.
 * @pinconf_set: set the pinconfig for a pin.
 * @get_num_ranges: get the number of pinctrl ranges.
 */
struct scmi_pinctrl_proto_ops {
	int (*describe)(const struct scmi_protocol_handle *ph,
			struct scmi_pinctrl_pin_range *ranges);
	int (*pinmux_get)(const struct scmi_protocol_handle *ph, u16 pin,
			  u16 *func);
	int (*pinmux_set)(const struct scmi_protocol_handle *ph, u16 no_pins,
			  const struct scmi_pinctrl_pin_function *pf);
	int (*pinconf_get)(const struct scmi_protocol_handle *ph, u16 pin,
			   struct scmi_pinctrl_pinconf *pcf);
	int (*pinconf_set)(const struct scmi_protocol_handle *ph, u16 pin,
			   struct scmi_pinctrl_pinconf *pcf, bool override);
	u16 (*get_num_ranges)(const struct scmi_protocol_handle *ph);
};

static inline u32 scmi_pinctrl_count_mb_configs(u32 mask)
{
	return hweight32(mask & SCMI_PINCTRL_MULTI_BIT_CFGS);
}

static inline size_t scmi_pinctrl_mb_configs_size(u32 mask)
{
	return hweight32(mask & SCMI_PINCTRL_MULTI_BIT_CFGS) *
	       sizeof_field(struct scmi_pinctrl_pinconf, multi_bit_values);
}

void scmi_pinctrl_pin_list_init(struct scmi_pinctrl_pin_list *list);
struct scmi_pinctrl_pin_list_elem *
	scmi_pinctrl_pin_list_remove_pin(struct scmi_pinctrl_pin_list *list,
					 u16 pin);
int scmi_pinctrl_pin_list_add_pin(struct scmi_pinctrl_pin_list *list,
				  struct scmi_pinctrl_pin_list_elem *p);
int scmi_pinctrl_create_pcf(unsigned long *configs,
			    unsigned int num_configs,
			    struct scmi_pinctrl_pinconf *pcf);
int scmi_pinctrl_convert_from_pcf(unsigned long **configs,
				  struct scmi_pinctrl_pinconf *pcf);
unsigned int scmi_pinctrl_count_multi_bit_values(unsigned long *configs,
						 unsigned int num_configs);
bool scmi_pinctrl_are_pcfs_equal(struct scmi_pinctrl_pinconf *pcfa,
				 struct scmi_pinctrl_pinconf *pcfb);
unsigned int scmi_pinctrl_hash_pcf(struct scmi_pinctrl_pinconf *pcf);
int scmi_pinctrl_add_mb_to_pcf(struct device *dev,
			       gfp_t flags,
			       struct scmi_pinctrl_pinconf *pcf,
			       enum pin_config_param param,
			       u32 value);
int scmi_pinctrl_add_pcf(struct device *dev,
			 gfp_t flags,
			 struct scmi_pinctrl_pinconf *res,
			 struct scmi_pinctrl_pinconf *src,
			 bool override);

#endif /* SCMI_PINCTRL_PROTOCOL_H defined */
