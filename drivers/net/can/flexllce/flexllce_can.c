// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * CAN driver for the NXP S32N79 FlexLLCE ("RCU") firmware-offloaded FlexCAN
 * controllers.
 *
 * The FlexLLCE firmware runs on the Cortex-R52 real-time cluster, owns the
 * FlexCAN peripherals and exposes them to the Cortex-A host through a
 * shared-memory + MRU-doorbell host interface (HIF). This driver attaches to
 * an already-running firmware (it does not load or reset it), maps the shared
 * memory, and drives each offloaded controller as a SocketCAN device by
 * issuing commands through the HIF command block and MRU mailboxes.
 *
 * Copyright 2026 NXP
 */
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/skb.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/net_tstamp.h>
#include <linux/iopoll.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include "flexllce_if.h"

/* Controller instance ids span the SoC's FlexCAN instances. */
#define FLEXLLCE_MAX_INSTANCE		48

#define FLEXLLCE_FW_MIN_MAJOR		0
#define FLEXLLCE_FW_MIN_MINOR		8

/*
 * The driver is layout-agnostic: it never assumes where a HIF's memory sits or
 * that the HIFs are contiguous. Each HIF device supplies, by name, its own
 * reserved-memory regions ("memory-region" + "memory-region-names"):
 *
 *   "shm"         this HIF's firmware shared-memory struct. The command block,
 *                 the RX/TX/notif comtables and the RX/ack/notif mailbox buffers
 *                 all live here at fixed offsets from the region base - there is
 *                 no HIF stride and no assumption that this is HIF0.
 *   "host"        host-private, RCU-reachable scratch this HIF owns: command
 *                 arguments, the TX-buffer pool and the SETFILTERTABLE arrays.
 *                 Host DRAM is not reachable by the RCU core, so this has to be a
 *                 reserved RCU-visible region, not kmalloc'd memory.
 *   "boot-status" the single global boot-status word (one per firmware core,
 *                 shared by every HIF).
 *
 * The offsets below are within a region, never across HIFs.
 */

/*
 * Driver-chosen sub-layout of the per-HIF "host" scratch region (this is host
 * memory, not a firmware layout): one page each for the command arguments, the
 * TX-buffer pool and the SETFILTERTABLE arrays. The static_asserts below guard
 * the two slots that scale with configuration against overrunning the next one;
 * the device tree sizes the "host" region to hold all three.
 */
#define FLEXLLCE_HOST_ARG_OFFSET	0x0	/* command-argument scratch */
#define FLEXLLCE_HOST_TXPOOL_OFFSET	0x1000	/* host TX-buffer pool */
#define FLEXLLCE_HOST_FILTER_OFFSET	0x2000	/* SETFILTERTABLE arrays */
static_assert(sizeof(struct flexllce_init_cmd) <=
	      FLEXLLCE_HOST_TXPOOL_OFFSET - FLEXLLCE_HOST_ARG_OFFSET);
static_assert(FLEXLLCE_TX_DESC_COUNT * sizeof(struct flexllce_tx_data) <=
	      FLEXLLCE_HOST_FILTER_OFFSET - FLEXLLCE_HOST_TXPOOL_OFFSET);

/*
 * A shared-memory sub-region reachable from both sides: @base is the host's
 * ioremapped pointer, @phys the RCU-visible physical address the firmware uses.
 */
struct flexllce_mem {
	void __iomem	*base;
	u32		phys;
};

/* Sub-region at @off inside @r (advances both the host and firmware address). */
static struct flexllce_mem flexllce_mem_at(struct flexllce_mem r, u32 off)
{
	return (struct flexllce_mem){ .base = r.base + off, .phys = r.phys + off };
}

/* Per Host-Interface state (shared by all controllers this HIF exposes). */
struct flexllce_hif {
	struct device			*dev;
	struct flexllce_shared_memory __iomem	*shm;	/* this HIF's shared-memory region */
	resource_size_t			shm_phys;	/* physical base of @shm */
	resource_size_t			shm_size;
	const void __iomem		*bootstatus;	/* global boot-status (read-only) */

	/* Serialise the single per-HIF command block. */
	struct mutex			cmd_lock;

	/*
	 * Host-private, RCU-reachable scratch region ("host" memory-region);
	 * @cmd_arg / @txpool are cached sub-regions of it (see FLEXLLCE_HOST_*).
	 */
	struct flexllce_mem		host;
	struct flexllce_mem		cmd_arg;	/* command-argument scratch */
	struct flexllce_mem		txpool;	/* host TX-buffer pool */
	spinlock_t			tx_lock;	/* TX + ack rings, tx_owner[] */
	spinlock_t			rx_lock;	/* RX and notification rings */

	/* Data-path mailboxes are held only while at least one netdev is up. */
	struct mutex			data_lock;
	u32				hif_id;
};

/* Read the boot-status word and confirm the firmware is up and compatible. */
static int flexllce_attach(struct flexllce_hif *hif)
{
	const struct flexllce_boot_status __iomem *bs = hif->bootstatus;
	u8 core0, major, minor;

	core0 = ioread8(&bs->core0_boot_status);
	if (core0 != FLEXLLCE_SUCCESS) {
		dev_info(hif->dev, "FlexLLCE core0 not ready (status %u)\n",
			 core0);
		return -EPROBE_DEFER;
	}

	major = ioread8(&bs->fw_info.major_version);
	minor = ioread8(&bs->fw_info.minor_version);
	if (major < FLEXLLCE_FW_MIN_MAJOR ||
	    (major == FLEXLLCE_FW_MIN_MAJOR && minor < FLEXLLCE_FW_MIN_MINOR)) {
		dev_err(hif->dev, "FlexLLCE firmware %u.%u too old\n",
			major, minor);
		return -ENODEV;
	}

	dev_info(hif->dev, "attached to FlexLLCE firmware v%u.%u\n",
		 major, minor);
	return 0;
}

/*
 * Map one named entry of the device's "memory-region" list and return its
 * ioremapped base (plus optional physical base and size). Selecting by name
 * keeps the driver layout-agnostic: a HIF's regions are wherever the device
 * tree places them, in any order.
 */
static void __iomem *flexllce_map_region(struct device *dev, const char *name,
					 resource_size_t *phys,
					 resource_size_t *size)
{
	struct device_node *np __free(device_node) = NULL;
	struct resource res;
	void __iomem *base;
	int idx, err;

	idx = of_property_match_string(dev->of_node, "memory-region-names", name);
	if (idx < 0)
		return ERR_PTR(dev_err_probe(dev, idx,
					     "no '%s' memory-region\n", name));

	np = of_parse_phandle(dev->of_node, "memory-region", idx);
	if (!np)
		return ERR_PTR(dev_err_probe(dev, -EINVAL,
					     "bad '%s' memory-region phandle\n", name));
	err = of_address_to_resource(np, 0, &res);
	if (err)
		return ERR_PTR(dev_err_probe(dev, err,
					     "bad '%s' memory-region\n", name));

	base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!base)
		return ERR_PTR(dev_err_probe(dev, -ENOMEM,
					     "mapping '%s' memory-region\n", name));

	if (phys)
		*phys = res.start;
	if (size)
		*size = resource_size(&res);
	return base;
}

/* Map this HIF's data-path regions and derive the sub-pointers into them. */
static int flexllce_map_data_regions(struct flexllce_hif *hif)
{
	struct device *dev = hif->dev;
	resource_size_t host_phys;

	/* This HIF's own firmware shared-memory region (offset 0 = its base). */
	hif->shm = (struct flexllce_shared_memory __iomem *)
		flexllce_map_region(dev, "shm", &hif->shm_phys, &hif->shm_size);
	if (IS_ERR(hif->shm))
		return PTR_ERR(hif->shm);

	/* Host-private scratch: command args, TX pool and filter arrays. */
	hif->host.base = flexllce_map_region(dev, "host", &host_phys, NULL);
	if (IS_ERR(hif->host.base))
		return PTR_ERR(hif->host.base);

	if (upper_32_bits(host_phys))
		return dev_err_probe(dev, -ERANGE,
				     "'host' region above 4 GB not addressable by firmware\n");

	hif->host.phys = lower_32_bits(host_phys);

	hif->cmd_arg = flexllce_mem_at(hif->host, FLEXLLCE_HOST_ARG_OFFSET);
	hif->txpool = flexllce_mem_at(hif->host, FLEXLLCE_HOST_TXPOOL_OFFSET);

	return 0;
}

static int flexllce_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct flexllce_hif *hif;
	int err;

	hif = devm_kzalloc(dev, sizeof(*hif), GFP_KERNEL);
	if (!hif)
		return -ENOMEM;

	hif->dev = dev;
	mutex_init(&hif->cmd_lock);
	mutex_init(&hif->data_lock);
	spin_lock_init(&hif->tx_lock);
	spin_lock_init(&hif->rx_lock);

	/*
	 * Gate on firmware readiness as early as possible: map only the global
	 * boot-status region and confirm the core is up and compatible before
	 * mapping this HIF's data regions or acquiring any resource.
	 */
	hif->bootstatus = flexllce_map_region(dev, "boot-status", NULL, NULL);
	if (IS_ERR(hif->bootstatus))
		return PTR_ERR(hif->bootstatus);

	err = flexllce_attach(hif);
	if (err)
		return err;

	err = of_property_read_u32(dev->of_node, "nxp,hif-id", &hif->hif_id);
	if (err)
		return dev_err_probe(dev, err, "missing 'nxp,hif-id'\n");

	err = flexllce_map_data_regions(hif);
	if (err)
		return err;

	platform_set_drvdata(pdev, hif);

	return 0;
}

static void flexllce_remove(struct platform_device *pdev)
{
}

static const struct of_device_id flexllce_of_match[] = {
	{ .compatible = "nxp,s32n79-flexllce-can" },
	{ }
};
MODULE_DEVICE_TABLE(of, flexllce_of_match);

static struct platform_driver flexllce_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = flexllce_of_match,
	},
	.probe = flexllce_probe,
	.remove = flexllce_remove,
};
module_platform_driver(flexllce_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP S32N79 FlexLLCE (RCU) CAN driver");
MODULE_LICENSE("GPL");
