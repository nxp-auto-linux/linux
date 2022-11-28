/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * SCMI GPIO Protocol
 *
 * Copyright 2022-2023 NXP
 */

#ifndef GPIO_SCMI_OPS_H
#define GPIO_SCMI_OPS_H

#include <linux/scmi_protocol.h>

struct scmi_gpio_eirq_report {
	ktime_t timestamp;
	u32 eirq_mask;
};

struct scmi_gpio_proto_ops {
	int (*get_gpio_base)(const struct scmi_protocol_handle *ph);
	u16 (*get_ngpios)(const struct scmi_protocol_handle *ph);
	const unsigned long *(*get_valid_mask)(const struct scmi_protocol_handle *ph,
					       u16 *ngpios, u16 *slots);
	int (*gpio_request)(const struct scmi_protocol_handle *ph, u32 gpio);
	void (*gpio_free)(const struct scmi_protocol_handle *ph, u32 gpio);
	void (*gpio_set_value)(const struct scmi_protocol_handle *ph, u32 gpio,
			       u8 val);
	u8 (*gpio_get_value)(const struct scmi_protocol_handle *ph, u32 gpio);
	u32 (*get_neirqs)(const struct scmi_protocol_handle *ph);
	int (*gpio_get_irq)(const struct scmi_protocol_handle *ph,
			    u32 gpio, u32 *irq);
	int (*gpio_enable_irq)(const struct scmi_protocol_handle *ph,
			       u32 gpio, u32 type);
	void (*gpio_disable_irq)(const struct scmi_protocol_handle *ph,
				 u32 gpio);
	int (*gpio_unmask_irq)(const struct scmi_protocol_handle *ph,
			       u32 gpio);
	int (*gpio_mask_irq)(const struct scmi_protocol_handle *ph,
			     u32 gpio);
	int (*gpio_ack_irq_mask)(const struct scmi_protocol_handle *ph);
	int (*gpio_get_direction)(const struct scmi_protocol_handle *ph, u32 gpio);
};

#endif
