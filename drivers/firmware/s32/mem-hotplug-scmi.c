// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2026 NXP
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bits.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/nxp_ddr_protocol.h>
#include <linux/of_address.h>
#include <linux/scmi_protocol.h>
#include <linux/workqueue.h>

/*
 * Split the DDR memory regions into chunks to avoid long waiting times
 * until the memory is initialized, as the operations are synchronous.
 */
#define CHUNK_SIZE	0x40000000ULL /* 1GB */

/**
 * struct s32_ddr_region - A DDR region this cohort wants online
 * @start: Physical start address of the region.
 * @size: Size of the region in bytes.
 * @instance: DDR controller (SMMS) instance owning the region.
 * @needs_init: Whether the region must be initialized before hot-add.
 *		False when ECC is disabled on @instance, in which case the
 *		DRAM is usable without a partial-init call.
 */
struct s32_ddr_region {
	u64  start;
	u64  size;
	u8   instance;
	bool needs_init;
};

struct scmi_mem_hotplug_data {
	struct device *dev;
	const struct scmi_protocol_handle *ph;
	const struct scmi_nxp_ddr_proto_ops *ops;
	struct scmi_nxp_ddr_info info;
	struct s32_ddr_region *regions;
	size_t num_regions;
	struct work_struct hotplug_work;
};

/**
 * hotplug_memory_region - Add a memory region to the system
 * @dev: Device pointer
 * @addr: Physical start address of the memory region
 * @size: Size of the memory region in bytes
 *
 * Registers a physical memory region within the kernel's memory subsystem,
 * making it available for use. The driver currently enforces the auto-online
 * policy for the hotplugged memory, so no further action is required and
 * the memory is ready to use.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int hotplug_memory_region(struct device *dev, u64 addr, u64 size)
{
	int nid = memory_add_physaddr_to_nid(addr);
	int ret;

	if (nid == NUMA_NO_NODE)
		nid = first_online_node;

	ret = add_memory(nid, addr, size, MHP_MERGE_RESOURCE | MHP_MEMMAP_ON_MEMORY);
	if (ret)
		dev_err(dev, "Failed to add memory (addr=0x%llx): %d\n", addr, ret);

	return ret;
}

/**
 * init_ddr_region_in_chunks - Initialize a DDR region by splitting it in smaller
 * block sizes.
 * @priv: Driver private data
 * @reg: Pointer to the DDR region descriptor
 *
 * Initializes a DDR memory region by splitting it into smaller blocks of
 * CHUNK_SIZE (1GB) to avoid long blocking times, as the initialization
 * operations over SCMI are synchronous. Each chunk size is aligned down to
 * the system's memory block size (128MB) to ensure compatibility with the
 * memory subsystem. When the region does not require initialization (ECC
 * disabled on its instance), it is hot-added directly.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int init_ddr_region_in_chunks(struct scmi_mem_hotplug_data *priv,
				     struct s32_ddr_region *reg)
{
	u64 min_block_size = memory_block_size_bytes();
	struct device *dev = priv->dev;
	u64 start = reg->start;
	u64 end = start + reg->size;
	u64 addr, chunk_size;
	int ret;

	for (addr = start; addr < end; addr += CHUNK_SIZE) {
		chunk_size = min(CHUNK_SIZE, end - addr);
		/* chunk_size must be aligned to memory block size */
		chunk_size = round_down(chunk_size, min_block_size);
		if (!chunk_size) {
			dev_warn(dev, "Skipping memory region init at 0x%llx\n", addr);
			continue;
		}

		if (reg->needs_init) {
			ret = priv->ops->partial_init(priv->ph, addr,
						      chunk_size, false);
			if (ret) {
				dev_err(dev, "Error %d initializing DDR region at 0x%llx\n",
					ret, addr);
				return ret;
			}
		}

		/* register each memory block and make it available to the kernel */
		ret = hotplug_memory_region(dev, addr, chunk_size);
		if (ret) {
			dev_err(dev, "Error %d hotplugging memory at 0x%llx\n", ret, addr);
			return ret;
		}
	}

	return 0;
}

/**
 * ddr_mem_init - Initialize all DDR regions in chunks
 * @priv: Driver private data
 */
static int ddr_mem_init(struct scmi_mem_hotplug_data *priv)
{
	struct device *dev = priv->dev;
	unsigned int i;
	int ret = 0;

	if (!priv->regions || !priv->num_regions) {
		dev_info(dev, "No DDR memory to be hotplugged\n");
		return 0;
	}

	for (i = 0; i < priv->num_regions; i++) {
		dev_dbg(dev, "Initializing DDR region addr=0x%llx size=0x%llx instance=%u init=%d\n",
			priv->regions[i].start, priv->regions[i].size,
			priv->regions[i].instance, priv->regions[i].needs_init);

		ret = init_ddr_region_in_chunks(priv, &priv->regions[i]);
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

	ret = ddr_mem_init(priv);
	if (ret)
		dev_err(priv->dev, "Failed to initialize DDR memory: %d\n", ret);
	else
		dev_info(priv->dev, "DDR memory initialization completed successfully\n");
}

/**
 * find_discovered_region - Locate the discovered region containing @addr
 * @discovered: Array of regions reported by DDR_DRAM_DISCOVERY
 * @count: Number of entries in @discovered
 * @addr: Physical start address of the DT-requested region
 * @size: Size of the DT-requested region in bytes
 *
 * The discovery reply is the authoritative set of uninitialized,
 * write-permitted regions. A DT-requested region is only actionable if it
 * falls entirely within one of the discovered regions.
 *
 * Return: pointer to the containing discovered region, or NULL if none.
 */
static const struct scmi_ddr_region *
find_discovered_region(const struct scmi_ddr_region *discovered, size_t count,
		       u64 addr, u64 size)
{
	u64 d_end;
	size_t i;

	for (i = 0; i < count; i++) {
		d_end = discovered[i].start + discovered[i].size;
		if (addr >= discovered[i].start && addr + size <= d_end)
			return &discovered[i];
	}

	return NULL;
}

/**
 * dt_parse_ddr_regions - Build the region list from DT policy and discovery
 * @priv: Driver private data
 * @discovered: Array of regions reported by DDR_DRAM_DISCOVERY
 * @num_discovered: Number of entries in @discovered
 *
 * The DT 'nxp,ddr-regions' property is the policy list of regions this cohort
 * wants online. Each is intersected with the discovered set: a region not
 * present in discovery is skipped (it may already be initialized or is not
 * permitted for this cohort); a region whose instance has ECC disabled is
 * marked as not needing initialization so it is hot-added directly.
 *
 * Return: 0 on success (including when there is nothing to do), or a negative
 * error code on failure.
 */
static int dt_parse_ddr_regions(struct scmi_mem_hotplug_data *priv,
				const struct scmi_ddr_region *discovered,
				size_t num_discovered)
{
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	struct device_node *mem_reg_node;
	const struct scmi_ddr_region *match;
	int num_regions;
	u64 addr, size;
	size_t i, n = 0;

	if (!np)
		return -ENODEV;

	num_regions = of_count_phandle_with_args(np, "nxp,ddr-regions", NULL);
	if (num_regions <= 0) {
		dev_info(dev, "'nxp,ddr-regions' property not defined or empty\n");
		priv->num_regions = 0;
		return 0;
	}

	priv->regions = devm_kcalloc(dev, num_regions, sizeof(*priv->regions),
				     GFP_KERNEL);
	if (!priv->regions)
		return -ENOMEM;

	for (i = 0; i < (size_t)num_regions; i++) {
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

		match = find_discovered_region(discovered, num_discovered,
					       addr, size);
		if (!match) {
			dev_warn(dev, "DDR region [0x%llx, 0x%llx] not reported by discovery, skipping\n",
				 addr, size);
			continue;
		}

		priv->regions[n].start = addr;
		priv->regions[n].size = size;
		priv->regions[n].instance = match->instance;
		priv->regions[n].needs_init =
			!!(priv->info.ecc_mask & BIT(match->instance));

		dev_dbg(dev, "DDR region %zu: addr=0x%llx size=0x%llx instance=%u init=%d\n",
			n, priv->regions[n].start, priv->regions[n].size,
			priv->regions[n].instance, priv->regions[n].needs_init);
		n++;
	}

	priv->num_regions = n;

	return 0;
}

static int mem_hotplug_scmi_probe(struct scmi_device *sdev)
{
	const struct scmi_handle *handle = sdev->handle;
	const struct scmi_nxp_ddr_proto_ops *ddr_ops;
	struct scmi_ddr_region *discovered;
	struct device *dev = &sdev->dev;
	struct scmi_protocol_handle *ph;
	struct scmi_mem_hotplug_data *priv;
	int ret, num_discovered;

	if (!handle)
		return -ENODEV;

	ddr_ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_NXP_DDR, &ph);
	if (IS_ERR(ddr_ops)) {
		dev_err(dev, "Could not retrieve SCMI NXP DDR ops\n");
		return PTR_ERR(ddr_ops);
	}

	if (!ph)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->ph = ph;
	priv->ops = ddr_ops;

	discovered = devm_kcalloc(dev, SCMI_NXP_DDR_MAX_REGIONS,
				  sizeof(*discovered), GFP_KERNEL);
	if (!discovered)
		return -ENOMEM;

	num_discovered = ddr_ops->discover_regions(ph, discovered,
						   SCMI_NXP_DDR_MAX_REGIONS);
	if (num_discovered < 0) {
		dev_err(dev, "Failed to discover DDR regions: %d\n", num_discovered);
		return num_discovered;
	}

	/*
	 * Fetch protocol info only after discovery: the ECC configuration
	 * bitmap (DdrCfgInfo) is reported by DDR_DRAM_DISCOVERY, so a
	 * get_info() before discovery would capture a zero ecc_mask and
	 * wrongly mark every region as not needing initialization.
	 */
	ret = ddr_ops->get_info(ph, &priv->info);
	if (ret) {
		dev_err(dev, "Failed to get NXP DDR protocol info: %d\n", ret);
		return ret;
	}

	ret = dt_parse_ddr_regions(priv, discovered, num_discovered);
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
	{ SCMI_PROTOCOL_NXP_DDR, "nxp-ddr" },
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
