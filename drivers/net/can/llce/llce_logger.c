// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2020-2021 NXP
 *
 * Driver for the NXP Semiconductors LLCE engine logging of CAN messages.
 * The LLCE can be found on S32G2xx.
 */
#include <linux/can/dev.h>
#include <linux/can/dev/llce_can_common.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>

#define LLCE_LOGGER_NETDEV_IF_NAME	"llcelogger"

struct llce_logger {
	struct llce_can_dev common; /* Must be the first member */
};

static int llce_logger_open(struct net_device *dev)
{
	struct llce_logger *llce = netdev_priv(dev);
	struct llce_can_dev *llce_dev = &llce->common;
	int ret = 0;

	napi_enable(&llce_dev->napi);

	llce_dev->rx = mbox_request_channel(&llce_dev->rx_client, 0);
	if (IS_ERR(llce_dev->rx)) {
		ret = PTR_ERR(llce_dev->rx);
		netdev_err(dev, "Failed to get logger RX mailbox: %d\n",
			   ret);
		goto disable_napi;
	}

	ret = enable_llce_rx_notif(llce_dev);
	if (ret)
		netdev_err(dev, "Failed to enable interrupt\n");

disable_napi:
	if (ret)
		napi_disable(&llce_dev->napi);

	return ret;
}

static int llce_logger_close(struct net_device *dev)
{
	struct llce_logger *llce = netdev_priv(dev);
	struct llce_can_dev *common = &llce->common;

	napi_disable(&common->napi);
	mbox_free_channel(common->rx);

	return 0;
}

static const struct net_device_ops llce_logger_netdev_ops = {
	.ndo_open	= llce_logger_open,
	.ndo_stop	= llce_logger_close,
};

static int llce_logger_probe(struct platform_device *pdev)
{
	struct net_device *netdev;
	struct llce_logger *logger;
	struct llce_can_dev *common;
	struct device *dev = &pdev->dev;
	struct can_priv *can;
	int ret;

	common = init_llce_can_dev(dev, sizeof(struct llce_logger),
				   LLCE_LOGGER_NETDEV_IF_NAME);
	if (IS_ERR(common))
		return PTR_ERR(common);

	logger = container_of(common, struct llce_logger, common);
	can = &common->can;
	netdev = can->dev;

	platform_set_drvdata(pdev, netdev);

	netdev->netdev_ops = &llce_logger_netdev_ops;

	can->restart_ms = 1;
	can->state = CAN_STATE_STOPPED;
	can->ctrlmode_supported = CAN_CTRLMODE_FD |
		CAN_CTRLMODE_FD_NON_ISO |
		CAN_CTRLMODE_LISTENONLY;

	enable_llce_napi(common);

	ret = register_candev(netdev);
	if (ret) {
		dev_err(dev, "Failed to register %s\n", netdev->name);
		free_llce_netdev(common);
	}

	return ret;
}

static int llce_logger_remove(struct platform_device *pdev)
{
	struct net_device *netdev = platform_get_drvdata(pdev);
	struct llce_logger *logger = netdev_priv(netdev);
	struct llce_can_dev *common = &logger->common;

	unregister_candev(netdev);
	netif_napi_del(&common->napi);
	free_candev(netdev);

	return 0;
}

static int __maybe_unused llce_logger_suspend(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct llce_logger *llce = netdev_priv(dev);
	struct llce_can_dev *common = &llce->common;
	int ret;

	if (netif_running(dev))
		ret = llce_logger_close(dev);

	common->can.state = CAN_STATE_SLEEPING;

	return 0;
}

static int __maybe_unused llce_logger_resume(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct llce_logger *llce = netdev_priv(dev);
	struct llce_can_dev *common = &llce->common;
	int ret;

	if (netif_running(dev))
		ret = llce_logger_open(dev);

	common->can.state = CAN_STATE_ERROR_ACTIVE;

	return 0;
}

static const struct of_device_id llce_logger_match[] = {
	{
		.compatible = "nxp,s32g-llce-can-logger",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, llce_logger_match);

static SIMPLE_DEV_PM_OPS(llce_logger_pm_ops, llce_logger_suspend,
			 llce_logger_resume);

static struct platform_driver llce_logger_driver = {
	.probe = llce_logger_probe,
	.remove = llce_logger_remove,
	.driver = {
		.name = "llce_logger",
		.of_match_table = llce_logger_match,
		.pm = &llce_logger_pm_ops,
	},
};
module_platform_driver(llce_logger_driver)

MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("NXP LLCE CAN Logger");
MODULE_LICENSE("Dual BSD/GPL");
