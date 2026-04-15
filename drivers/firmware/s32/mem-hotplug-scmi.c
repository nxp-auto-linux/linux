// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2026 NXP
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/memory.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/scmi_protocol.h>
#include <linux/nxp_misc_protocol.h>
#include <linux/workqueue.h>

#define	LOW_DDR_MEM_SMMS_0_START	0x80000000ULL
#define	LOW_DDR_MEM_SMMS_0_END		0xbfffffffULL /* 1GB */
#define	LOW_DDR_MEM_SMMS_1_START	0xc0000000ULL
#define	LOW_DDR_MEM_SMMS_1_END		0xffffffffULL /* 1GB*/

#define	HIGH_DDR_MEM_SMMS_0_START	0x8800000000ULL
#define	HIGH_DDR_MEM_SMMS_0_END		0x8bffffffffULL /* 16GB */
#define	HIGH_DDR_MEM_SMMS_1_START	0xc000000000ULL
#define	HIGH_DDR_MEM_SMMS_1_END		0xc3ffffffffULL /* 16GB */

/*
 * Split the DDR memory regions into chunks to avoid long waiting times
 * until the memory is initialized, as the operations are synchronous.
 */
#define CHUNK_SIZE	0x40000000ULL /* 1GB */

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
	struct work_struct hotplug_work;
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

/**
 * init_ddr_region_in_chunks - Initialize a DDR region by splitting it in smaller
 * block sizes.
 * @dev: Device pointer
 * @reg: Pointer to the DDR region descriptor
 *
 * Initializes a DDR memory region by splitting it into smaller blocks of
 * CHUNK_SIZE (1GB) to avoid long blocking times, as the initialization
 * operations over SCMI are synchronous. Each chunk size is aligned down to
 * the system's memory block size (128MB) to ensure compatibility with the
 * memory subsystem.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int init_ddr_region_in_chunks(struct device *dev, struct ddr_region *reg)
{
	u64 min_block_size = memory_block_size_bytes();
	u64 start = reg->start_addr;
	u64 end = start + reg->size;
	u64 addr, chunk_size;
	int ret;

	for (addr = start; addr < end; addr += CHUNK_SIZE) {
		chunk_size = min(CHUNK_SIZE, end - addr);
		/* chunk_size must be aligned to memory block size */
		chunk_size = round_down(chunk_size, min_block_size);
		if (chunk_size) {
			ret = scmi_nxp_misc_init_ddr_partial(dev, addr,
						chunk_size, reg->instance_id);
			if (ret) {
				dev_err(dev, "Error %d initializing DDR region at 0x%llx\n",
					ret, addr);
				return ret;
			}
		} else {
			dev_warn(dev, "Skipping memory region init at 0x%llx\n", addr);
		}
	}

	return 0;
}

/**
 * ddr_mem_init - Initialize all DDR regions in chunks
 * @dev: Device pointer
 */
static int ddr_mem_init(struct device *dev)
{
	struct scmi_mem_hotplug_data *priv = dev_get_drvdata(dev);
	unsigned int i;
	int ret = 0;

	if (!priv->regions || !priv->num_regions) {
		dev_info(dev, "No DDR memory to be hotplugged\n");
		return 0;
	}

	for (i = 0; i < priv->num_regions; i++) {
		dev_dbg(dev, "Initializing DDR region addr=0x%llx size=0x%llx\n",
			priv->regions[i].start_addr, priv->regions[i].size);

		ret = init_ddr_region_in_chunks(dev, &priv->regions[i]);
		if (ret) {
			dev_err(dev, "Error %d initializing DDR region\n", ret);
			return ret;
		}
	}

	return 0;
}

/**
 * ddr_mem_init_work - Work handler for DDR memory initialization
 * @work: Pointer to the work_struct
 */
static void ddr_mem_init_work(struct work_struct *work)
{
	struct scmi_mem_hotplug_data *priv =
		container_of(work, struct scmi_mem_hotplug_data, hotplug_work);
	int ret;

	dev_info(priv->dev, "Starting DDR memory initialization\n");

	ret = ddr_mem_init(priv->dev);
	if (ret)
		dev_err(priv->dev, "Failed to initialize DDR memory: %d\n", ret);
	else
		dev_info(priv->dev, "DDR memory initialization completed successfully\n");
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

	INIT_WORK(&priv->hotplug_work, ddr_mem_init_work);
	if (!queue_work(system_long_wq, &priv->hotplug_work))
		dev_warn(dev, "DDR memory initialization already queued\n");

	return 0;
}

static void mem_hotplug_scmi_remove(struct scmi_device *sdev)
{
	struct scmi_mem_hotplug_data *priv = dev_get_drvdata(&sdev->dev);

	cancel_work_sync(&priv->hotplug_work);
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_NXP_MISC, "nxp-misc" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver mem_hotplug_scmi_driver = {
	.name = "mem-hotplug-scmi",
	.probe = mem_hotplug_scmi_probe,
	.remove = mem_hotplug_scmi_remove,
	.id_table = scmi_id_table,
};
module_scmi_driver(mem_hotplug_scmi_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("S32N Memory Hotplug SCMI-based driver");
MODULE_LICENSE("Dual BSD/GPL");
