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

#define FLEXLLCE_CMD_TIMEOUT_MS		100

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

/* Two accept-all host-receive filters: one for standard, one for extended ids. */
#define FLEXLLCE_NR_RX_FILTERS		2
/*
 * Per-filter in-flight host-RX credit: frames a filter may have delivered but
 * not yet had acknowledged before the firmware drops with a no-buffer event.
 * SETFILTERTABLE (per controller) caps the sum of its filters' counts at 128,
 * so with FLEXLLCE_NR_RX_FILTERS (2) the most each may claim is 64; use that
 * full budget so the RX ring depth (FLEXLLCE_RX_DESC_COUNT) is what actually
 * bounds the outstanding frames, not this credit.
 */
#define FLEXLLCE_RX_FILTER_MB		64

/* MRU doorbell channels used by a HIF, indexed by mbox-names order. */
enum flexllce_mbox {
	FLEXLLCE_MBOX_CMD,	/* host -> RCU: submit a command */
	FLEXLLCE_NUM_MBOX
};

static const char * const flexllce_mbox_names[FLEXLLCE_NUM_MBOX] = {
	[FLEXLLCE_MBOX_CMD]	= "cmd",
};

struct flexllce_chan;

/* Communication-table ring status. */
enum flexllce_ring_status {
	FLEXLLCE_RING_EMPTY,
	FLEXLLCE_RING_OPERATIONAL,
	FLEXLLCE_RING_FULL,
};

/*
 * Host-side mirror of a firmware communication channel. The descriptor table
 * (@table, holding one u32 per descriptor) lives in shared memory and is
 * written only by the producer; the head/tail indices are private to each
 * peer. The MRU doorbell carries the descriptor index between peers. This is a
 * mirror of the firmware's ring so the two stay coherent.
 */
struct flexllce_ring {
	void __iomem		*table;
	u32			start;
	u32			stop;
	u32			wr_idx;		/* head - advanced by the producer */
	u32			rd_idx;		/* tail - advanced by the consumer */
	enum flexllce_ring_status status;
};

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

	struct mbox_client		cl[FLEXLLCE_NUM_MBOX];
	struct mbox_chan		*chan[FLEXLLCE_NUM_MBOX];

	/* Serialise the single per-HIF command block. */
	struct mutex			cmd_lock;

	/*
	 * Host-private, RCU-reachable scratch region ("host" memory-region);
	 * @cmd_arg / @txpool are cached sub-regions of it (see FLEXLLCE_HOST_*).
	 */
	struct flexllce_mem		host;
	struct flexllce_mem		cmd_arg;	/* command-argument scratch */
	struct flexllce_mem		txpool;	/* host TX-buffer pool */
	struct flexllce_ring		tx_ring;
	struct flexllce_ring		rx_ring;
	struct flexllce_ring		notif_ring;	/* async notifications */
	spinlock_t			tx_lock;	/* TX + ack rings, tx_owner[] */
	spinlock_t			rx_lock;	/* RX and notification rings */

	/* Data-path mailboxes are held only while at least one netdev is up. */
	struct mutex			data_lock;
	u32				hif_id;
};

/* Ring helpers - producer/consumer semantics for the shared descriptor rings. */

static void flexllce_ring_init(struct flexllce_ring *r, void __iomem *table,
			       u32 size)
{
	r->table = table;
	r->start = 0;
	r->stop = size;
	r->wr_idx = 0;
	r->rd_idx = 0;
	r->status = FLEXLLCE_RING_EMPTY;
}

static u32 flexllce_ring_next(const struct flexllce_ring *r, u32 idx)
{
	if (++idx >= r->stop)
		idx = r->start;
	return idx;
}

/*
 * Recompute the ring status after an index moves. When the head and tail meet
 * the ring is at a boundary that only the caller can name: @meet is the state
 * that a producer's advance (FULL) or a consumer's advance (EMPTY) leaves it in;
 * otherwise the ring is operational.
 */
static void flexllce_ring_set_status(struct flexllce_ring *r,
				     enum flexllce_ring_status meet)
{
	r->status = (r->wr_idx == r->rd_idx) ? meet : FLEXLLCE_RING_OPERATIONAL;
}

/* Producer: claim the next free descriptor. */
static int flexllce_ring_get_free(struct flexllce_ring *r, u32 *desc)
{
	if (r->status == FLEXLLCE_RING_FULL)
		return -ENOSPC;

	*desc = r->wr_idx;
	r->wr_idx = flexllce_ring_next(r, r->wr_idx);
	flexllce_ring_set_status(r, FLEXLLCE_RING_FULL);
	return 0;
}

/* Consumer: take the next full descriptor. */
static int flexllce_ring_get_full(struct flexllce_ring *r, u32 *desc)
{
	if (r->status == FLEXLLCE_RING_EMPTY)
		return -ENODATA;

	*desc = r->rd_idx;
	r->rd_idx = flexllce_ring_next(r, r->rd_idx);
	flexllce_ring_set_status(r, FLEXLLCE_RING_EMPTY);
	return 0;
}

/* Consumer: record the producer's head from a received doorbell. */
static void flexllce_ring_register_full(struct flexllce_ring *r, u32 desc)
{
	r->wr_idx = flexllce_ring_next(r, desc);
	flexllce_ring_set_status(r, FLEXLLCE_RING_FULL);
}

/* Per-controller (SocketCAN device) state. */
struct flexllce_chan {
	struct can_priv			can;	/* must be first */
	struct net_device		*ndev;
	struct flexllce_hif		*hif;
	u8				instance;
};

/*
 * FlexCAN nominal-phase bit-timing constraints. The firmware validates the
 * classical CAN timing against the FlexCAN CTRL1 register limits
 * (TSEG1 = prop_seg + phase_seg1 <= 16, TSEG2 = phase_seg2 <= 8), so the core
 * must compute timings within that range - a wider range makes the firmware
 * reject INIT with an invalid-parameters error.
 */
static const struct can_bittiming_const flexllce_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

/* CAN FD data-phase bit-timing constraints. */
static const struct can_bittiming_const flexllce_data_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,
	.tseg1_max = 39,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 8,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

/*
 * Issue a command through the HIF command block: optionally stage @arg into the
 * shared command-argument scratch, write the command block, ring the "cmd"
 * doorbell, poll the block for the firmware's return value, then optionally copy
 * the firmware's result back out of the scratch into @arg. @arg may be NULL for
 * commands that take no argument and return no data.
 */
static int flexllce_send_cmd(struct flexllce_hif *hif, u8 cmd_id, u8 instance,
			     void *arg, size_t arglen)
{
	struct flexllce_command cmd = {
		.cmd_ptr = cpu_to_le32(hif->cmd_arg.phys),
		.cmd_id = cmd_id,
		.return_value = FLEXLLCE_NO_RETURN_VALUE,
		.instance = instance,
	};
	u32 doorbell = hif->hif_id;
	int err;
	u8 ret;

	guard(mutex)(&hif->cmd_lock);

	/* Stage the command argument into the shared scratch under the lock. */
	if (arg)
		memcpy_toio(hif->cmd_arg.base, arg, arglen);

	/*
	 * Publish the whole command block in one shot. The firmware reads it
	 * only after the doorbell below, so the order within the block does not
	 * matter; return_value is pre-cleared to FLEXLLCE_NO_RETURN_VALUE (0) so
	 * the poll below can detect the firmware writing its result.
	 */
	memcpy_toio(&hif->shm->cmd, &cmd, sizeof(cmd));

	/* Ring the command doorbell. */
	err = mbox_send_message(hif->chan[FLEXLLCE_MBOX_CMD], &doorbell);
	if (err < 0)
		return err;

	mbox_client_txdone(hif->chan[FLEXLLCE_MBOX_CMD], 0);

	err = readb_poll_timeout(&hif->shm->cmd.return_value, ret,
				 ret != FLEXLLCE_NO_RETURN_VALUE, 10,
				 FLEXLLCE_CMD_TIMEOUT_MS * USEC_PER_MSEC);
	if (err) {
		dev_err(hif->dev, "cmd %u (inst %u) timed out (no firmware return)\n",
			cmd_id, instance);
		return err;
	}

	if (ret != FLEXLLCE_SUCCESS) {
		dev_err(hif->dev, "cmd %u (inst %u) returned %u\n",
			cmd_id, instance, ret);
		return -EIO;
	}

	/* Copy the firmware's result back out of the scratch under the lock. */
	if (arg)
		memcpy_fromio(arg, hif->cmd_arg.base, arglen);

	return 0;
}

static int flexllce_get_status(struct flexllce_chan *chan,
			       struct flexllce_get_status_cmd *st)
{
	struct flexllce_hif *hif = chan->hif;

	return flexllce_send_cmd(hif, FLEXLLCE_CMD_GETSTATUS, chan->instance,
				 st, sizeof(*st));
}

/*
 * Wait for a controller to reach the stopped state after a STOP command. STOP
 * is asynchronous: the firmware returns before the hardware has finished
 * stopping, and only advances the controller to STOPPED (and accepts DEINIT)
 * once a GETSTATUS poll observes the hardware idle.
 */
static int flexllce_wait_stopped(struct flexllce_chan *chan)
{
	struct flexllce_get_status_cmd st = {};
	int i, err;

	for (i = 0; i < 10; i++) {
		err = flexllce_get_status(chan, &st);
		if (err)
			return err;
		if (!st.can_started)
			return 0;
		usleep_range(100, 200);
	}
	return -ETIMEDOUT;
}

/*
 * Report the controller's transmit/receive error counters. GETSTATUS returns
 * the raw FlexCAN Error Counter Register; its low two bytes hold the standard
 * transmit (TXERRCNT) and receive (RXERRCNT) error counts.
 */
static int flexllce_get_berr_counter(const struct net_device *ndev,
				     struct can_berr_counter *bec)
{
	struct flexllce_chan *chan = netdev_priv(ndev);
	struct flexllce_get_status_cmd st = {};
	u32 ecr;
	int err;

	err = flexllce_get_status(chan, &st);
	if (err)
		return err;

	ecr = le32_to_cpu(st.ecr);
	bec->txerr = ecr & 0xff;
	bec->rxerr = (ecr >> 8) & 0xff;
	return 0;
}

/*
 * Host-side layout of the five per-filter arrays SETFILTERTABLE consumes. The
 * firmware dereferences each array through its own pointer in
 * flexllce_set_filter_cmd.
 */
struct flexllce_filter_scratch {
	struct flexllce_id_filter	ids[FLEXLLCE_NR_RX_FILTERS];
	u8				rx_if[FLEXLLCE_NR_RX_FILTERS];
	__le16				route[FLEXLLCE_NR_RX_FILTERS];
	__le16				mbc[FLEXLLCE_NR_RX_FILTERS];
	__le16				f2hrh[FLEXLLCE_NR_RX_FILTERS];
};

/*
 * Install an accept-all host-receive filter table for a controller: INIT
 * installs none, so without it no frame reaches the host. Two ONE_ID filters
 * with an all-zero mask (route none, non-zero mailbox count) accept every
 * standard and extended id into the interrupt RX channel the NAPI poll drains.
 * The arrays are read by physical address, so they live in the RCU-reachable
 * host scratch region at FLEXLLCE_HOST_FILTER_OFFSET.
 */
static int flexllce_set_filter(struct flexllce_chan *chan)
{
	struct flexllce_set_filter_cmd cmd = {};
	struct flexllce_filter_scratch scratch = {};
	struct flexllce_hif *hif = chan->hif;
	struct flexllce_mem filt;
	int i;

	filt = flexllce_mem_at(hif->host, FLEXLLCE_HOST_FILTER_OFFSET);

	for (i = 0; i < FLEXLLCE_NR_RX_FILTERS; i++) {
		scratch.ids[i].filter_type = FLEXLLCE_FILTER_ONE_ID;
		scratch.ids[i].is_extended = (i == 1) ? FLEXLLCE_TRUE : FLEXLLCE_FALSE;
		scratch.ids[i].id1 = cpu_to_le32(0);	/* mask: accept every id */
		scratch.ids[i].id2 = cpu_to_le32(0);	/* match code (unused when mask is 0) */
		scratch.rx_if[i] = FLEXLLCE_RX_COMCH_INTERRUPT;
		scratch.route[i] = cpu_to_le16(FLEXLLCE_FILTER_ROUTE_NONE);
		scratch.mbc[i] = cpu_to_le16(FLEXLLCE_RX_FILTER_MB);
	}

	/* Stage all five arrays in one shot; f2hrh is left zeroed. */
	memcpy_toio(filt.base, &scratch, sizeof(scratch));

	/* Each command pointer is that array's physical address in the scratch. */
	cmd.id_table	   = cpu_to_le32(filt.phys + offsetof(typeof(scratch), ids));
	cmd.rx_interface   = cpu_to_le32(filt.phys + offsetof(typeof(scratch), rx_if));
	cmd.route_table_id = cpu_to_le32(filt.phys + offsetof(typeof(scratch), route));
	cmd.mb_count	   = cpu_to_le32(filt.phys + offsetof(typeof(scratch), mbc));
	cmd.filter2hrh	   = cpu_to_le32(filt.phys + offsetof(typeof(scratch), f2hrh));
	cmd.filters_no	   = FLEXLLCE_NR_RX_FILTERS;

	return flexllce_send_cmd(hif, FLEXLLCE_CMD_SETFILTERTABLE, chan->instance,
				 &cmd, sizeof(cmd));
}

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
 * Set up one mailbox client and request its channel by device-tree name. The
 * caller assigns cl->rx_callback beforehand for receiver channels; senders
 * leave it NULL. This is the generic per-mailbox primitive used by both the
 * command and data-path requesters.
 */
static struct mbox_chan *flexllce_request_mbox(struct flexllce_hif *hif,
					       enum flexllce_mbox id)
{
	struct mbox_client *cl = &hif->cl[id];

	cl->dev = hif->dev;
	cl->tx_block = false;
	cl->knows_txdone = true;

	return mbox_request_channel_byname(cl, flexllce_mbox_names[id]);
}

static int flexllce_request_cmd_mbox(struct flexllce_hif *hif)
{
	/*
	 * The "cmd" doorbell (host->RCU) is held for the lifetime of the device
	 * and command completion is polled from the shared command block, so no
	 * "cmdack" receiver is needed. The data-path doorbells are requested on
	 * demand while a controller is up (flexllce_request_data_mboxes()).
	 */
	hif->chan[FLEXLLCE_MBOX_CMD] = flexllce_request_mbox(hif, FLEXLLCE_MBOX_CMD);
	if (IS_ERR(hif->chan[FLEXLLCE_MBOX_CMD]))
		return dev_err_probe(hif->dev,
				     PTR_ERR(hif->chan[FLEXLLCE_MBOX_CMD]),
				     "failed to get 'cmd' mailbox\n");
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

	err = flexllce_request_cmd_mbox(hif);
	if (err)
		return err;

	platform_set_drvdata(pdev, hif);

	return 0;
}

static void flexllce_remove(struct platform_device *pdev)
{
	struct flexllce_hif *hif = platform_get_drvdata(pdev);

	mbox_free_channel(hif->chan[FLEXLLCE_MBOX_CMD]);
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
