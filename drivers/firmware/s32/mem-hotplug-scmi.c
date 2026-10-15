// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2026 NXP
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/memory.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/nxp_ddr_protocol.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/scmi_protocol.h>
#include <linux/workqueue.h>

/*
 * Split the DDR memory regions into chunks to bound the time spent per init
 * request. In synchronous mode this keeps each blocking SCMI call short; in
 * asynchronous mode it keeps each request well inside the delayed-response
 * timeout.
 */
#define CHUNK_SIZE	0x40000000ULL /* 1GB */

/*
 * A busy platform rejects an init request with -EBUSY while it serves another
 * request on the same controller, which can be a multi-gigabyte region
 * requested by another OS instance sharing that controller. The work is
 * deferrable, so back off and retry.
 */
#define S32_DDR_INIT_RETRIES		50
#define S32_DDR_INIT_BACKOFF_MS		100

static bool force_sync;
module_param(force_sync, bool, 0444);
MODULE_PARM_DESC(force_sync,
		 "Force synchronous DDR initialization even when the platform supports the asynchronous form");

/**
 * struct s32_ddr_region - A DDR region this OS instance wants online
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

struct scmi_mem_hotplug_data;

/**
 * struct s32_ddr_worker - Per-instance DDR initialization worker
 * @work: Work item queued on the system long workqueue.
 * @priv: Back pointer to the driver private data.
 * @instance: DDR controller (SMMS) instance this worker handles.
 *
 * One worker is queued per active DDR instance so the instances initialize in
 * parallel, which is the benefit of the asynchronous form: while one instance
 * waits for its delayed response the other can make progress.
 */
struct s32_ddr_worker {
	struct work_struct		work;
	struct scmi_mem_hotplug_data	*priv;
	u8				instance;
};

/**
 * struct scmi_mem_hotplug_data - Driver private data
 * @dev: Device pointer.
 * @ph: NXP DDR protocol handle.
 * @ops: NXP DDR protocol operations.
 * @info: Negotiated protocol capabilities.
 * @regions: Region list built from DT policy intersected with discovery.
 * @num_regions: Number of entries in @regions.
 * @use_async: Whether to request the asynchronous form of partial init.
 * @workers: One worker per DDR instance.
 * @active_instances: Bitmap of instances that have at least one region queued.
 */
struct scmi_mem_hotplug_data {
	struct device *dev;
	const struct scmi_protocol_handle *ph;
	const struct scmi_nxp_ddr_proto_ops *ops;
	struct scmi_nxp_ddr_info info;
	struct s32_ddr_region *regions;
	size_t num_regions;
	bool use_async;
	struct s32_ddr_worker workers[SCMI_NXP_DDR_MAX_INSTANCES];
	unsigned long active_instances;
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
 * Initialize a DDR memory region by splitting it into blocks of CHUNK_SIZE
 * (1GB) to bound the time spent per init request. Each chunk is aligned down
 * to the system's memory block size (128MB) for compatibility with the memory
 * subsystem. When the region does not require initialization (ECC disabled on
 * its instance), it is hot-added directly.
 *
 * A busy platform can reject a request with -EBUSY; back off and retry before
 * giving up, because the work is deferrable. Each initialized chunk is
 * hot-added before the next chunk is initialized, so a failure never leaves an
 * uninitialized chunk visible to the kernel.
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
	int attempt, ret;

	for (addr = start; addr < end; addr += CHUNK_SIZE) {
		chunk_size = min(CHUNK_SIZE, end - addr);
		/* chunk_size must be aligned to memory block size */
		chunk_size = round_down(chunk_size, min_block_size);
		if (!chunk_size) {
			dev_warn(dev, "Skipping memory region init at 0x%llx\n", addr);
			continue;
		}

		if (reg->needs_init) {
			ret = -EBUSY;
			for (attempt = 0; attempt <= S32_DDR_INIT_RETRIES; attempt++) {
				ret = priv->ops->partial_init(priv->ph, addr,
							      chunk_size,
							      priv->use_async);
				if (ret != -EBUSY)
					break;
				msleep(S32_DDR_INIT_BACKOFF_MS);
			}
			if (ret) {
				if (ret == -EBUSY)
					dev_dbg(dev, "DDR region at 0x%llx still busy after %d attempts (%d ms back-off)\n",
						addr, attempt,
						attempt * S32_DDR_INIT_BACKOFF_MS);
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
 * ddr_mem_init_work - Per-instance DDR memory initialization work handler
 * @work: Pointer to the work_struct embedded in a struct s32_ddr_worker
 *
 * Initialize every region belonging to this worker's DDR instance. One worker
 * runs per instance so the instances progress in parallel. A region that fails
 * is logged and skipped rather than aborting the worker, so the board boots
 * with less memory rather than none.
 */
static void ddr_mem_init_work(struct work_struct *work)
{
	struct s32_ddr_worker *w = container_of(work, struct s32_ddr_worker, work);
	struct scmi_mem_hotplug_data *priv = w->priv;
	struct device *dev = priv->dev;
	unsigned int i;
	int ret;

	dev_info(dev, "Starting DDR memory initialization for instance %u\n",
		 w->instance);

	for (i = 0; i < priv->num_regions; i++) {
		if (priv->regions[i].instance != w->instance)
			continue;

		dev_dbg(dev, "Initializing DDR region addr=0x%llx size=0x%llx instance=%u init=%d\n",
			priv->regions[i].start, priv->regions[i].size,
			priv->regions[i].instance, priv->regions[i].needs_init);

		/* Log with the instance number, then keep going (see above). */
		ret = init_ddr_region_in_chunks(priv, &priv->regions[i]);
		if (ret)
			dev_err(dev,
				"instance %u: failed to initialize region 0x%llx: %d\n",
				w->instance, priv->regions[i].start, ret);
	}

	dev_info(dev, "DDR memory initialization for instance %u completed\n",
		 w->instance);
}

/**
 * ddr_region_is_online - Whether a region is already live system RAM
 * @dev: Device pointer
 * @addr: Physical start address of the region
 * @size: Size of the region in bytes
 *
 * Discovery also lists regions another agent, such as U-Boot's ddr_init,
 * already brought up. Initializing them again would zero live memory.
 *
 * Return: true unless the region is entirely disjoint from system RAM.
 */
static bool ddr_region_is_online(struct device *dev, u64 addr, u64 size)
{
	switch (region_intersects(addr, size, IORESOURCE_SYSTEM_RAM,
				  IORES_DESC_NONE)) {
	case REGION_DISJOINT:
		return false;
	case REGION_INTERSECTS:
		dev_info(dev, "DDR region [0x%llx, 0x%llx] is already system RAM, skipping\n",
			 addr, size);
		return true;
	default:
		dev_warn(dev, "DDR region [0x%llx, 0x%llx] is partly system RAM already, skipping all of it\n",
			 addr, size);
		return true;
	}
}

/**
 * find_discovered_region - Locate the discovered region containing @addr
 * @discovered: Array of regions reported by DDR_DRAM_DISCOVERY
 * @count: Number of entries in @discovered
 * @addr: Physical start address of the DT-requested region
 * @size: Size of the DT-requested region in bytes
 *
 * The discovery reply is the authoritative set of available regions. A
 * DT-requested region is only actionable if it falls entirely within one of
 * the discovered regions.
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
 * use_discovered_ddr_regions - Take every discovered region as the policy
 * @priv: Driver private data
 * @discovered: Array of regions reported by DDR_DRAM_DISCOVERY
 * @num_discovered: Number of entries in @discovered
 *
 * Used when the device tree carries no 'nxp,ddr-regions' property.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int use_discovered_ddr_regions(struct scmi_mem_hotplug_data *priv,
				      const struct scmi_ddr_region *discovered,
				      size_t num_discovered)
{
	struct device *dev = priv->dev;
	size_t i, n = 0;

	priv->num_regions = 0;
	if (!num_discovered)
		return 0;

	priv->regions = devm_kcalloc(dev, num_discovered,
				     sizeof(*priv->regions), GFP_KERNEL);
	if (!priv->regions)
		return -ENOMEM;

	for (i = 0; i < num_discovered; i++) {
		if (ddr_region_is_online(dev, discovered[i].start,
					 discovered[i].size))
			continue;

		priv->regions[n].start = discovered[i].start;
		priv->regions[n].size = discovered[i].size;
		priv->regions[n].instance = discovered[i].instance;
		priv->regions[n].needs_init =
			!!(priv->info.ecc_mask & BIT(discovered[i].instance));

		dev_dbg(dev, "DDR region %zu: addr=0x%llx size=0x%llx instance=%u init=%d\n",
			n, priv->regions[n].start, priv->regions[n].size,
			priv->regions[n].instance, priv->regions[n].needs_init);
		n++;
	}

	priv->num_regions = n;

	return 0;
}

/**
 * dt_parse_ddr_regions - Build the region list from DT policy and discovery
 * @priv: Driver private data
 * @discovered: Array of regions reported by DDR_DRAM_DISCOVERY
 * @num_discovered: Number of entries in @discovered
 *
 * The DT 'nxp,ddr-regions' property is the optional policy list of regions this
 * OS instance wants online; without it every discovered region is used. An
 * enabled node or a region already online as system RAM is left alone. The
 * rest are intersected with the discovered set: a region not present in
 * discovery is skipped (it may already be initialized or is not permitted for
 * this agent); a region whose instance has ECC disabled is marked as not
 * needing initialization so it is hot-added directly.
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
		dev_dbg(dev, "no 'nxp,ddr-regions' policy, using every discovered region\n");
		return use_discovered_ddr_regions(priv, discovered,
						  num_discovered);
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

		/* An enabled memory node is boot memory already. */
		if (of_device_is_available(mem_reg_node)) {
			dev_dbg(dev, "%pOF is enabled, leaving it alone\n",
				mem_reg_node);
			of_node_put(mem_reg_node);
			continue;
		}

		if (of_property_read_reg(mem_reg_node, 0, &addr, &size)) {
			dev_err(dev, "%pOF: unable to find 'reg' property\n", mem_reg_node);
			of_node_put(mem_reg_node);
			return -EINVAL;
		}

		of_node_put(mem_reg_node);

		if (ddr_region_is_online(dev, addr, size))
			continue;

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
	unsigned int i, inst;

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

	if (!priv->num_regions) {
		dev_info(dev, "No DDR memory to be hotplugged\n");
		return 0;
	}

	/* Prefer async unless the platform lacks it or the user forced sync. */
	priv->use_async = priv->info.async_supported && !force_sync;

	/* Queue one worker per instance that has at least one region. */
	for (i = 0; i < priv->num_regions; i++)
		__set_bit(priv->regions[i].instance, &priv->active_instances);

	for_each_set_bit(inst, &priv->active_instances,
			 SCMI_NXP_DDR_MAX_INSTANCES) {
		priv->workers[inst].priv = priv;
		priv->workers[inst].instance = inst;
		INIT_WORK(&priv->workers[inst].work, ddr_mem_init_work);
		if (!queue_work(system_long_wq, &priv->workers[inst].work))
			dev_warn(dev, "DDR init for instance %u already queued\n",
				 inst);
	}

	return 0;
}

static void mem_hotplug_scmi_remove(struct scmi_device *sdev)
{
	struct scmi_mem_hotplug_data *priv = dev_get_drvdata(&sdev->dev);
	unsigned int inst;

	for_each_set_bit(inst, &priv->active_instances,
			 SCMI_NXP_DDR_MAX_INSTANCES)
		cancel_work_sync(&priv->workers[inst].work);
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
