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

/* Per Host-Interface state (shared by all controllers this HIF exposes). */
struct flexllce_hif {
	struct device			*dev;

	/* Serialise the single per-HIF command block. */
	struct mutex			cmd_lock;

	/* Data-path mailboxes are held only while at least one netdev is up. */
	struct mutex			data_lock;
	spinlock_t			tx_lock;	/* TX + ack rings, tx_owner[] */
	spinlock_t			rx_lock;	/* RX and notification rings */
};

static int flexllce_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct flexllce_hif *hif;

	hif = devm_kzalloc(dev, sizeof(*hif), GFP_KERNEL);
	if (!hif)
		return -ENOMEM;

	hif->dev = dev;
	mutex_init(&hif->cmd_lock);
	mutex_init(&hif->data_lock);
	spin_lock_init(&hif->tx_lock);
	spin_lock_init(&hif->rx_lock);

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
