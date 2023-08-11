// SPDX-License-Identifier: GPL-2.0
/*
 *  Driver using SCMI NVMEM Protocol
 *  Copyright 2023 NXP
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/err.h>
#include <linux/module.h>
#include <linux/nvmem_scmi_protocol.h>
#include <linux/nvmem-provider.h>

static const struct scmi_nvmem_proto_ops *nvmem_ops;

struct nvmem_scmi_data {
	struct device *dev;
	struct nvmem_device *nvmem;
	const struct scmi_protocol_handle *ph;
};

static int scmi_nvmem_read_cell(void *priv, unsigned int offset, void *val,
				size_t bytes)
{
	struct nvmem_scmi_data *dev_priv = priv;
	struct device *dev = dev_priv->dev;
	u32 value;
	int ret;

	ret = nvmem_ops->nvmem_read_cell(dev_priv->ph, offset, bytes,
					 &value);
	if (ret) {
		dev_err(dev, "Failed to read cell %u with size %lu\n", offset,
			bytes);
		return ret;
	}

	*(u32 *)val = value;

	return 0;
}

static int scmi_nvmem_write_cell(void *priv, unsigned int offset, void *val,
				 size_t bytes)
{
	struct nvmem_scmi_data *dev_priv = priv;
	struct device *dev = dev_priv->dev;
	int ret;
	u32 value = 0;

	memcpy(&value, val, bytes);

	ret = nvmem_ops->nvmem_write_cell(dev_priv->ph, offset, bytes, value);
	if (ret) {
		dev_err(dev, "Failed to write cell %u with size %lu, value: %u\n",
			offset, bytes, value);
		return ret;
	}

	return 0;
}

static struct nvmem_config scmi_nvmem_config = {
	.name = "nvmem_scmi",
	.owner = THIS_MODULE,
	.word_size = 1,
	.size = 4,
	.read_only = false,
	.reg_read = scmi_nvmem_read_cell,
	.reg_write = scmi_nvmem_write_cell,
};

static int scmi_nvmem_probe(struct scmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	const struct scmi_handle *handle = sdev->handle;
	struct scmi_protocol_handle *ph;
	struct nvmem_scmi_data *priv;
	struct nvmem_config *econfig = &scmi_nvmem_config;

	if (!handle) {
		dev_info(dev, "SCMI handle not found [protocol 0x%x]\n",
			 SCMI_PROTOCOL_ID_NVMEM);
		return -ENODEV;
	}

	nvmem_ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_ID_NVMEM,
					      &ph);
	if (IS_ERR(nvmem_ops)) {
		dev_info(dev, "Could not retrieve SCMI NVMEM ops\n");
		return PTR_ERR(nvmem_ops);
	}

	priv = devm_kzalloc(dev, sizeof(struct nvmem_scmi_data), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->ph = ph;

	econfig->dev = dev;
	econfig->priv = priv;
	priv->nvmem = devm_nvmem_register(dev, econfig);

	dev_info(dev, "SCMI NVMEM Driver registered\n");

	return PTR_ERR_OR_ZERO(priv->nvmem);
}

static const struct scmi_device_id scmi_nvmem_id_table[] = {
	{ SCMI_PROTOCOL_ID_NVMEM, "nvmem" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_nvmem_id_table);

static struct scmi_driver scmi_nvmem_driver = {
	.name = "scmi-nvmem",
	.probe = scmi_nvmem_probe,
	.id_table = scmi_nvmem_id_table,
};
module_scmi_driver(scmi_nvmem_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("SCMI NVMEM Driver");
MODULE_LICENSE("Dual BSD/GPL");
