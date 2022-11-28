// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * SCMI GPIO Protocol - NXP vendor extension
 *
 * Copyright 2022-2023 NXP
 */

#define pr_fmt(fmt) "scmi:gpio:proto: " fmt

#include <linux/irq.h>
#include <linux/module.h>
#include <linux/scmi_protocol.h>
#include <linux/gpio_scmi_protocol.h>
#include <linux/gpio_ops_scmi_protocol.h>

#include "common.h"
#include "notify.h"

struct gpio_info {
	unsigned long *gavail;
	u32 version;
	u32 gbase;
	u16 ngpios;
	u16 nirqs;
};

struct scmi_gpio_proto_attrs {
	__le32 attrs;
};

struct scmi_gpio_desc {
	__le32 base_id;
	__le32 avail[0];
};

struct scmi_gpio_set_conf {
	__le32 gpio;
	__le32 config;
};

struct scmi_gpio_set_val {
	__le32 gpio;
	__le32 value;
};

struct scmi_gpio_req_eirq {
	__le32 gpio;
	__le32 type;
};

struct scmi_gpio_eirq_notif_payld {
	__le32 eirq_mask;
};

static int get_gpio_base(const struct scmi_protocol_handle *ph)
{
	struct gpio_info *ginfo = ph->get_priv(ph);

	if (ginfo->gbase > INT_MAX)
		return INT_MAX;

	return ginfo->gbase;
}

static u16 get_ngpios(const struct scmi_protocol_handle *ph)
{
	struct gpio_info *ginfo = ph->get_priv(ph);

	return ginfo->ngpios;
}

static u16 get_nslots(struct gpio_info *ginfo)
{
	u16 nslots = ginfo->ngpios / SCMI_GPIO_PINS_PER_SLOT;

	if (ginfo->ngpios % SCMI_GPIO_PINS_PER_SLOT)
		nslots++;

	return nslots;
}

static const unsigned long *get_valid_mask(const struct scmi_protocol_handle *ph,
					   u16 *ngpios, u16 *slots)
{
	struct gpio_info *ginfo = ph->get_priv(ph);

	*slots = get_nslots(ginfo);
	*ngpios = ginfo->ngpios;

	return ginfo->gavail;
}

static int gpio_request(const struct scmi_protocol_handle *ph, u32 gpio)
{
	int ret;
	struct scmi_xfer *t;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_REQUEST,
				      sizeof(gpio), 0, &t);
	if (ret)
		return ret;

	put_unaligned_le32(gpio, t->tx.buf);
	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		dev_err(ph->dev, "Failed to request gpio %u\n", gpio);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static void gpio_free(const struct scmi_protocol_handle *ph, u32 gpio)
{
	int ret;
	struct scmi_xfer *t;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_FREE,
				      sizeof(gpio), 0, &t);
	if (ret)
		return;

	put_unaligned_le32(gpio, t->tx.buf);
	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		dev_err(ph->dev, "Failed to free gpio %u\n", gpio);

	ph->xops->xfer_put(ph, t);
}

static void gpio_set_value(const struct scmi_protocol_handle *ph, u32 gpio,
			   u8 val)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_gpio_set_val *conf;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_SET_VALUE,
				      sizeof(*conf), 0, &t);
	if (ret) {
		dev_err(ph->dev, "Failed to init GPIO_SET_VALUE context\n");
		return;
	}

	conf = t->tx.buf;
	conf->gpio = cpu_to_le32(gpio);
	conf->value = cpu_to_le32(val);

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		dev_err(ph->dev, "Failed to set value on gpio %u\n", gpio);

	ph->xops->xfer_put(ph, t);
}

static u8 gpio_get_value(const struct scmi_protocol_handle *ph, u32 gpio)
{
	int ret;
	struct scmi_xfer *t;
	__le32 *pin;
	u32 value;
	u8 val = 0u;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_GET_VALUE,
				      sizeof(*pin), sizeof(value), &t);
	if (ret) {
		dev_err(ph->dev, "Failed to init GPIO_GET_VALUE context\n");
		return val;
	}

	pin = t->tx.buf;
	*pin = cpu_to_le32(gpio);

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		value = le32_to_cpu(*(__le32 *)t->rx.buf);
		if (value)
			val = 1u;
	} else {
		dev_err(ph->dev, "Failed to get GPIO %u value\n", gpio);
	}

	ph->xops->xfer_put(ph, t);

	return val;
}

static int gpio_get_direction(const struct scmi_protocol_handle *ph, u32 gpio)
{
	int ret;
	struct scmi_xfer *t;
	__le32 *pin;
	u32 direction;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_GET_DIRECTION,
				      sizeof(*pin), sizeof(direction), &t);
	if (ret) {
		dev_err(ph->dev, "Failed to init GPIO_GET_DIRECTION context\n");
		return ret;
	}

	pin = t->tx.buf;
	*pin = cpu_to_le32(gpio);

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		direction = le32_to_cpu(*(__le32 *)t->rx.buf);
		if (direction == SCMI_GPIO_DIR_OUT)
			ret = SCMI_GPIO_DIR_OUT;
		else
			ret = SCMI_GPIO_DIR_IN;
	} else {
		dev_err(ph->dev, "Failed to get GPIO %u direction\n", gpio);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static u32 get_neirqs(const struct scmi_protocol_handle *ph)
{
	struct gpio_info *ginfo = ph->get_priv(ph);

	return ginfo->nirqs;
}

static int gpio_get_irq(const struct scmi_protocol_handle *ph,
			u32 gpio, u32 *irq)
{
	int ret;
	struct scmi_xfer *t;
	__le32 *req, *res;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_GET_IRQ,
				      sizeof(gpio), sizeof(*irq), &t);
	if (ret)
		return ret;

	req = t->tx.buf;
	*req = cpu_to_le32(gpio);

	res = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		*irq = le32_to_cpu(*res);
	} else {
		dev_err(ph->dev, "Failed check IRQ capability of GPIO %u\n",
			gpio);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int gpio_enable_irq(const struct scmi_protocol_handle *ph,
			   u32 gpio, u32 type)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_gpio_req_eirq *req;
	u32 scmi_irq_type;

	if ((type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH)
		scmi_irq_type = SCMI_GPIO_EIRQ_TYPE_BOTH;
	else if ((type & IRQ_TYPE_EDGE_RISING) == IRQ_TYPE_EDGE_RISING)
		scmi_irq_type = SCMI_GPIO_EIRQ_TYPE_EDGE_RISING;
	else if ((type & IRQ_TYPE_EDGE_FALLING) == IRQ_TYPE_EDGE_FALLING)
		scmi_irq_type = SCMI_GPIO_EIRQ_TYPE_EDGE_FALLING;
	else if ((type & IRQ_TYPE_LEVEL_HIGH) == IRQ_TYPE_LEVEL_HIGH)
		scmi_irq_type = SCMI_GPIO_EIRQ_TYPE_LEVEL_HIGH;
	else if ((type & IRQ_TYPE_LEVEL_LOW) == IRQ_TYPE_LEVEL_LOW)
		scmi_irq_type = SCMI_GPIO_EIRQ_TYPE_LEVEL_LOW;
	else
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_IRQ_ENABLE,
				      sizeof(*req), 0, &t);
	if (ret)
		return ret;

	req = t->tx.buf;
	req->gpio = cpu_to_le32(gpio);
	req->type = cpu_to_le32(scmi_irq_type);

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		dev_err(ph->dev, "Failed to request IRQ for GPIO %u\n",
			gpio);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static void gpio_disable_irq(const struct scmi_protocol_handle *ph, u32 gpio)
{
	int ret;
	struct scmi_xfer *t;
	__le32 *gpio_index;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_IRQ_DISABLE,
				      sizeof(u32), 0, &t);
	if (ret)
		return;

	gpio_index = t->tx.buf;
	*gpio_index = cpu_to_le32(gpio);

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		dev_err(ph->dev, "Failed to disable IRQ of GPIO %u\n",
			gpio);

	ph->xops->xfer_put(ph, t);
}

static int gpio_unmask_irq(const struct scmi_protocol_handle *ph,
			   u32 index)
{
	int ret;
	struct scmi_xfer *t;
	__le32 *pair_index;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_IRQ_UNMASK,
				      sizeof(u32), 0, &t);
	if (ret)
		return ret;

	pair_index = t->tx.buf;
	*pair_index = cpu_to_le32(index);

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		dev_err(ph->dev, "Failed to unmask GPIO IRQ index=%u\n",
			index);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int gpio_mask_irq(const struct scmi_protocol_handle *ph,
			 u32 index)
{
	int ret;
	struct scmi_xfer *t;
	__le32 *pair_index;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_IRQ_MASK,
				      sizeof(u32), 0, &t);
	if (ret)
		return ret;

	pair_index = t->tx.buf;
	*pair_index = cpu_to_le32(index);

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		dev_err(ph->dev, "Failed to mask GPIO IRQ index=%u\n",
			index);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int gpio_ack_irq_mask(const struct scmi_protocol_handle *ph)
{
	int ret;
	struct scmi_xfer *t;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_ACK_IRQ, 0, 0, &t);
	if (ret)
		return ret;

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		dev_err(ph->dev, "Failed to ACK GPIO IRQ\n");

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_gpio_proto_attrs_get(const struct scmi_protocol_handle *ph,
				     struct gpio_info *ginfo)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_gpio_proto_attrs *attr;
	u32 attrs;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES,
				      0, sizeof(*attr), &t);
	if (ret)
		return ret;

	attr = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		attrs = le32_to_cpu(attr->attrs);
		ginfo->nirqs = (attrs & GENMASK(31, 16)) >> 16;
		ginfo->ngpios = attrs & GENMASK(15, 0);
	} else {
		dev_err(ph->dev, "Failed to get protocol attributes\n");
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_gpio_describe(const struct scmi_protocol_handle *ph,
			      struct gpio_info *ginfo)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_gpio_desc *resp;
	u32 i, j, nslots = 0, slot, pos;

	ret = ph->xops->xfer_get_init(ph, SCMI_GPIO_DESCRIBE,
				      0, 0, &t);
	if (ret)
		return ret;

	resp = t->tx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		goto release_xfer;

	nslots = get_nslots(ginfo);

	for (i = 0u; i < nslots; i++) {
		slot = le32_to_cpu(resp->avail[i]);
		for (j = 0; j < SCMI_GPIO_PINS_PER_SLOT; j++) {
			if (slot & BIT(j)) {
				pos = i * SCMI_GPIO_PINS_PER_SLOT + j;
				bitmap_set(ginfo->gavail, pos, 1);
			}
		}
	}

release_xfer:
	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_gpio_protocol_init(const struct scmi_protocol_handle *ph)
{
	int ret;
	u32 version;
	struct gpio_info *ginfo;

	ret = ph->xops->version_get(ph, &version);
	if (ret) {
		dev_err(ph->dev, "Failed to get version\n");
		return ret;
	}

	dev_info(ph->dev, "SCMI GPIO Version %d.%d\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	ginfo = devm_kzalloc(ph->dev, sizeof(*ginfo), GFP_KERNEL);
	if (!ginfo)
		return -ENOMEM;

	ginfo->version = version;

	ret = scmi_gpio_proto_attrs_get(ph, ginfo);
	if (ret)
		return ret;

	ginfo->gavail = devm_bitmap_zalloc(ph->dev, ginfo->ngpios, GFP_KERNEL);
	if (!ginfo->gavail)
		return -ENOMEM;

	ret = scmi_gpio_describe(ph, ginfo);
	if (ret)
		return ret;

	return ph->set_priv(ph, ginfo);
}

static int scmi_gpio_protocol_deinit(const struct scmi_protocol_handle *ph)
{
	return 0;
}

static const struct scmi_gpio_proto_ops gpio_ops = {
	.get_gpio_base = get_gpio_base,
	.get_ngpios = get_ngpios,
	.get_valid_mask = get_valid_mask,
	.gpio_request = gpio_request,
	.gpio_free = gpio_free,
	.gpio_get_value = gpio_get_value,
	.gpio_set_value = gpio_set_value,
	.get_neirqs = get_neirqs,
	.gpio_get_irq = gpio_get_irq,
	.gpio_enable_irq = gpio_enable_irq,
	.gpio_disable_irq = gpio_disable_irq,
	.gpio_unmask_irq = gpio_unmask_irq,
	.gpio_mask_irq = gpio_mask_irq,
	.gpio_ack_irq_mask = gpio_ack_irq_mask,
	.gpio_get_direction = gpio_get_direction,
};

static const struct scmi_event gpio_events[] = {
	{
		.id = SCMI_GPIO_IRQ_NOTIFICATION,
		.max_payld_sz = sizeof(struct scmi_gpio_eirq_notif_payld),
		.max_report_sz = sizeof(struct scmi_gpio_eirq_report),
	},
};

static int scmi_gpio_get_num_sources(const struct scmi_protocol_handle *ph)
{
	return 1;
}

static int scmi_gpio_set_notify_enabled(const struct scmi_protocol_handle *ph,
					u8 evt_id, u32 src_id, bool enable)
{
	/* GPIO EIRQs are enabled by default */
	return 0;
}

static void *scmi_gpio_fill_custom_report(const struct scmi_protocol_handle *ph,
					  u8 evt_id, ktime_t timestamp,
					  const void *payld, size_t payld_sz,
					  void *report, u32 *src_id)
{
	void *rep = NULL;

	switch (evt_id) {
	case SCMI_GPIO_IRQ_NOTIFICATION:
	{
		const struct scmi_gpio_eirq_notif_payld *p = payld;
		struct scmi_gpio_eirq_report *r = report;

		if (sizeof(*p) != payld_sz)
			break;

		r->timestamp = timestamp;
		r->eirq_mask = le32_to_cpu(p->eirq_mask);
		*src_id = 0;
		rep = r;
		break;
	}
	default:
		break;
	}

	return rep;
}

static const struct scmi_event_ops gpio_event_ops = {
	.get_num_sources = scmi_gpio_get_num_sources,
	.set_notify_enabled = scmi_gpio_set_notify_enabled,
	.fill_custom_report = scmi_gpio_fill_custom_report,
};

static const struct scmi_protocol_events gpio_protocol_events = {
	.queue_sz = SCMI_PROTO_QUEUE_SZ,
	.ops = &gpio_event_ops,
	.evts = gpio_events,
	.num_events = ARRAY_SIZE(gpio_events),
};

static struct scmi_protocol scmi_gpio = {
	.id = SCMI_PROTOCOL_ID_GPIO,
	.owner = THIS_MODULE,
	.instance_init = &scmi_gpio_protocol_init,
	.instance_deinit = &scmi_gpio_protocol_deinit,
	.ops = &gpio_ops,
	.events = &gpio_protocol_events,
};

static int __init register_scmi_gpio_proto(void)
{
	return scmi_protocol_register(&scmi_gpio);
}
arch_initcall_sync(register_scmi_gpio_proto);

static void __exit unregister_scmi_gpio_proto(void)
{
	scmi_protocol_unregister(&scmi_gpio);
}
module_exit(unregister_scmi_gpio_proto);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP SCMI GPIO Protocol");
MODULE_LICENSE("Dual BSD/GPL");
