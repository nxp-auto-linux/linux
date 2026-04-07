// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2026 NXP
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/scmi_protocol.h>
#include <linux/nxp_misc_protocol.h>

#define	LOW_DDR_MEM_SMMS_0_START	0x80000000ULL
#define	LOW_DDR_MEM_SMMS_0_END		0xbfffffffULL /* 1GB */
#define	LOW_DDR_MEM_SMMS_1_START	0xc0000000ULL
#define	LOW_DDR_MEM_SMMS_1_END		0xffffffffULL /* 1GB*/

#define	HIGH_DDR_MEM_SMMS_0_START	0x8800000000ULL
#define	HIGH_DDR_MEM_SMMS_0_END		0x8bffffffffULL /* 16GB */
#define	HIGH_DDR_MEM_SMMS_1_START	0xc000000000ULL
#define	HIGH_DDR_MEM_SMMS_1_END		0xc3ffffffffULL /* 16GB */

struct ddr_region {
	u64 start_addr;
	u64 size;
	u32 instance_id;
};

struct scmi_mem_hotplug_data {
	struct device *dev;
	const struct scmi_protocol_handle *ph;
	const struct scmi_nxp_misc_proto_ops *ops;
	struct ddr_region *regions;
	size_t num_regions;
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

static bool is_ddr_reg_valid(u64 addr, u64 size, u64 start, u64 end)
{
	return (addr >= start && addr <= end) &&
		(addr + size - 1 <= end);
}

static int get_instance_id(u64 addr, u64 size)
{
	if (is_ddr_reg_valid(addr, size,
			LOW_DDR_MEM_SMMS_0_START, LOW_DDR_MEM_SMMS_0_END))
		return 0;

	if (is_ddr_reg_valid(addr, size,
			HIGH_DDR_MEM_SMMS_0_START, HIGH_DDR_MEM_SMMS_0_END))
		return 0;

	if (is_ddr_reg_valid(addr, size,
			LOW_DDR_MEM_SMMS_1_START, LOW_DDR_MEM_SMMS_1_END))
		return 1;

	if (is_ddr_reg_valid(addr, size,
			HIGH_DDR_MEM_SMMS_1_START, HIGH_DDR_MEM_SMMS_1_END))
		return 1;

	return -EINVAL;
}

static int dt_parse_ddr_regions(struct device *dev, struct scmi_mem_hotplug_data *priv)
{
	struct device_node *np = dev->of_node;
	struct device_node *mem_reg_node;
	int id, num_regions;
	u64 addr, size;
	size_t i;

	if (!np)
		return -ENODEV;

	num_regions = of_count_phandle_with_args(np, "nxp,ddr-regions", NULL);
	if (num_regions <= 0) {
		dev_info(dev, "'nxp,ddr-regions' property not defined or empty\n");
		priv->num_regions = 0;
		return 0;
	}

	priv->num_regions = num_regions;
	priv->regions = devm_kcalloc(dev, priv->num_regions, sizeof(*priv->regions),
						GFP_KERNEL);
	if (!priv->regions)
		return -ENOMEM;

	for (i = 0; i < priv->num_regions; i++) {
		mem_reg_node = of_parse_phandle(np, "nxp,ddr-regions", i);
		if (!mem_reg_node) {
			dev_err(dev, "Failed to parse phandle at index %zu\n", i);
			return -EINVAL;
		}

		if (of_property_read_reg(mem_reg_node, 0, &addr, &size)) {
			dev_err(dev, "%pOF: unable to find 'reg' property\n", mem_reg_node);
			of_node_put(mem_reg_node);
			return -EINVAL;
		}

		of_node_put(mem_reg_node);

		priv->regions[i].start_addr = addr;
		priv->regions[i].size = size;

		id = get_instance_id(addr, size);
		if (id < 0) {
			dev_err(dev, "Invalid DDR region provided: [0x%llx, 0x%llx]\n", addr, size);
			return -EINVAL;
		}
		priv->regions[i].instance_id = id;

		dev_dbg(dev, "DDR region %zu: addr=0x%llx size=0x%llx instance=%u\n",
			i, priv->regions[i].start_addr, priv->regions[i].size,
			priv->regions[i].instance_id);
	}

	return 0;
}

static int mem_hotplug_scmi_probe(struct scmi_device *sdev)
{
	const struct scmi_handle *handle = sdev->handle;
	const struct scmi_nxp_misc_proto_ops *nxp_misc_ops;
	struct device *dev = &sdev->dev;
	struct scmi_protocol_handle *ph;
	struct scmi_mem_hotplug_data *priv;
	int ret;

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

	ret = dt_parse_ddr_regions(dev, priv);
	if (ret) {
		dev_err(dev, "Failed to parse DDR regions: %d\n", ret);
		return ret;
	}

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
