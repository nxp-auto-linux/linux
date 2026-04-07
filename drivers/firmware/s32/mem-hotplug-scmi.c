// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2026 NXP
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/scmi_protocol.h>
#include <linux/nxp_misc_protocol.h>

struct scmi_mem_hotplug_data {
	struct device *dev;
	const struct scmi_protocol_handle *ph;
	const struct scmi_nxp_misc_proto_ops *ops;
};

/**
 * scmi_nxp_misc_init_ddr_partial - Initialize a DDR region
 * @start_addr: Starting physical address of the DDR region
 * @size: Size of the DDR region to initialize
 *
 * Return: 0 on success, error code otherwise
 */
static int scmi_nxp_misc_init_ddr_partial(struct device *dev,
	u64 start_addr, u64 size, u32 id)
{
	struct scmi_mem_hotplug_data *priv = dev_get_drvdata(dev);

	return priv->ops->init_ddr_partial(priv->ph, start_addr, size, id);
}

static int mem_hotplug_scmi_probe(struct scmi_device *sdev)
{
	const struct scmi_handle *handle = sdev->handle;
	const struct scmi_nxp_misc_proto_ops *nxp_misc_ops;
	struct device *dev = &sdev->dev;
	struct scmi_protocol_handle *ph;
	struct scmi_mem_hotplug_data *priv;

	if (!handle)
		return -ENODEV;

	nxp_misc_ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_NXP_MISC, &ph);
	if (IS_ERR(nxp_misc_ops)) {
		dev_err(dev, "Could not retrieve SCMI NXP Misc ops\n");
		return PTR_ERR(nxp_misc_ops);
	}

	if (!ph)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &sdev->dev;
	priv->ph = ph;
	priv->ops = nxp_misc_ops;

	dev_set_drvdata(dev, priv);

	return 0;
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_NXP_MISC, "nxp-misc" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver mem_hotplug_scmi_driver = {
	.name = "mem-hotplug-scmi",
	.probe = mem_hotplug_scmi_probe,
	.id_table = scmi_id_table,
};
module_scmi_driver(mem_hotplug_scmi_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("S32N Memory Hotplug SCMI-based driver");
MODULE_LICENSE("Dual BSD/GPL");
