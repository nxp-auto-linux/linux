// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2020-2021 NXP */
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
#include <linux/stringify.h>

/* 10 ms timeout on all channels */
#define CHAN_TIMEOUT			10
#define LLCE_CAN_DRV_NAME		"llce_can"
#define LLCE_CAN_NETDEV_IF_NAME		"llcecan"

#define LLCE_CBT_PRESDIV_OFFSET		23U
#define LLCE_CBT_RJW_OFFSET		16U
#define LLCE_CBT_TSEG2_OFFSET		9U
#define LLCE_CBT_TSEG1_OFFSET		0U

#define LLCE_CAN_MAX_TX_MB		16U

struct llce_can {
	struct llce_can_dev common; /* Must be the first member */

	struct completion config_done;

	struct mbox_client config_client, tx_client;
	struct mbox_chan *config, *tx;

	struct clk *clk;
};

static bool logging;
module_param(logging, bool, 0660);

static const struct can_bittiming_const llce_can_bittiming = {
	.name = LLCE_CAN_DRV_NAME,
	.tseg1_min = 4,
	.tseg1_max = 256,
	.tseg2_min = 4,
	.tseg2_max = 128,
	.sjw_max = 128,
	.brp_min = 1,
	.brp_max = 512,
	.brp_inc = 1,
};

static const struct can_bittiming_const llce_can_data_bittiming = {
	.name = LLCE_CAN_DRV_NAME,
	.tseg1_min = 2,
	.tseg1_max = 32,
	.tseg2_min = 2,
	.tseg2_max = 16,
	.sjw_max = 16,
	.brp_min = 1,
	.brp_max = 32,
	.brp_inc = 1,
};

static void config_rx_callback(struct mbox_client *cl, void *msg)
{
	struct llce_can *llce = container_of(cl, struct llce_can,
					     config_client);

	complete(&llce->config_done);
}

static struct device *llce_can_chan_dev(struct mbox_chan *conf_chan)
{
	struct mbox_client *cl = conf_chan->cl;

	return cl->dev;
}

static int send_cmd_msg(struct mbox_chan *conf_chan,
			struct llce_can_command *cmd)
{
	struct device *dev = llce_can_chan_dev(conf_chan);
	struct mbox_client *cl = conf_chan->cl;
	struct llce_can *can = container_of(cl, struct llce_can, config_client);
	int ret;

	ret = mbox_send_message(conf_chan, cmd);
	if (ret < 0)
		return ret;

	wait_for_completion(&can->config_done);
	if (cmd->return_value != LLCE_FW_SUCCESS) {
		dev_err(dev, "LLCE FW error %d\n", cmd->return_value);
		return -EIO;
	}

	return 0;
}

static int llce_can_init(struct llce_can *llce)
{
	struct mbox_chan *conf_chan = llce->config;
	struct device *dev = llce_can_chan_dev(conf_chan);
	struct llce_can_dev *llce_dev = &llce->common;
	struct can_priv *can = &llce_dev->can;
	u32 ctrl_config = 0;
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_INIT,
		.cmd_list.init = {
			.ctrl_config = LLCE_CAN_CONTROLLERCONFIG_CTRL_EN
			    | LLCE_CAN_CONTROLLERCONFIG_ABR_EN,
			.tx_mb_count = LLCE_CAN_MAX_TX_MB,
		},
	};
	int ret;

	if (can->ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		ctrl_config |= LLCE_CAN_CONTROLLERCONFIG_LPB_EN;
		ctrl_config |= LLCE_CAN_CONTROLLERCONFIG_SRX_EN;
	}

	if (can->ctrlmode & CAN_CTRLMODE_LISTENONLY)
		ctrl_config |= LLCE_CAN_CONTROLLERCONFIG_LOM_EN;

	cmd.cmd_list.init.ctrl_config |= ctrl_config;

	ret = send_cmd_msg(conf_chan, &cmd);
	if (ret) {
		dev_err(dev, "Failed to init LLCE CAN\n");
		return ret;
	}

	return 0;
}

static int llce_can_deinit(struct llce_can *llce)
{
	struct mbox_chan *conf_chan = llce->config;
	struct device *dev = llce_can_chan_dev(conf_chan);
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_DEINIT,
	};
	int ret;

	ret = send_cmd_msg(conf_chan, &cmd);
	if (ret) {
		dev_err(dev, "Failed to deinit LLCE CAN\n");
		return ret;
	}

	return 0;
}

static void set_rx_filter(struct llce_can_rx_filter *rx_filter, u8 intf,
			  bool canfd)
{
	enum llce_can_rx_mb_length len;

	if (canfd)
		len = USE_LONG_MB;
	else
		len = USE_SHORT_MB;

	*rx_filter = (struct llce_can_rx_filter) {
		.id_mask = 0,
		.message_id = 0,
		/* Use packet type as filter id */
		.filter_id = len,
		.mb_count = LLCE_CAN_MAX_TX_MB,
		.entry_type = LLCE_CAN_ENTRY_CFG_MASKED,
		.filter_mb_length = len,
		.rx_dest_interface = intf,
	};
}

static void set_basic_filter(struct llce_can_command *cmd, u8 intf, bool canfd)
{
	*cmd = (struct llce_can_command) {
		.cmd_id = LLCE_CAN_CMD_SETFILTER,
		.cmd_list.set_filter = {
			.rx_filters_count = 1,
		},
	};

	set_rx_filter(&cmd->cmd_list.set_filter.rx_filters[0], intf, canfd);
}

static void set_advanced_filter(struct llce_can_command *cmd, u8 intf,
				bool canfd)
{
	struct llce_can_advanced_filter *afilt;

	*cmd = (struct llce_can_command) {
		.cmd_id = LLCE_CAN_CMD_SETADVANCEDFILTER,
		.cmd_list.set_advanced_filter = {
			.rx_filters_count = 1,
		},
	};

	afilt = &cmd->cmd_list.set_advanced_filter.advanced_filters[0];
	afilt->llce_can_Advanced_feature = (struct llce_can_advanced_feature) {
		.can_authentication_feature = LLCE_AF_AUTHENTICATION_DISABLED,
		.can_custom_processing = LLCE_AF_CUSTOMPROCESSING_DISABLED,
		.can_logging_feature = LLCE_AF_LOGGING_ENABLED,
		.host_receive = LLCE_AF_HOSTRECEIVE_ENABLED,
		.can2can_routing_table_idx = (u8)0x0U,
		.can2eth_routing_table_idx = (u8)0x0U,

	};

	set_rx_filter(&afilt->llce_can_Rx_filter, intf, canfd);
}

static int can_add_open_filter(struct mbox_chan *conf_chan, bool canfd)
{
	struct device *dev = llce_can_chan_dev(conf_chan);
	struct llce_chan_priv *priv = conf_chan->con_priv;
	struct llce_can_command cmd;
	int ret;

	if (logging)
		set_advanced_filter(&cmd, priv->index, canfd);
	else
		set_basic_filter(&cmd, priv->index, canfd);

	ret = send_cmd_msg(conf_chan, &cmd);
	if (ret) {
		dev_err(dev, "Failed to set RX filter\n");
		return ret;
	}

	return 0;
}

static int set_controller_mode(struct mbox_chan *conf_chan,
			       enum llce_can_state_transition mode)
{
	struct device *dev = llce_can_chan_dev(conf_chan);
	const char *mode_str;
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_SETCONTROLLERMODE,
		.cmd_list.set_controller_mode = {
			.transition = mode,
		},
	};
	int ret;

	ret = send_cmd_msg(conf_chan, &cmd);
	if (ret) {
		if (mode == LLCE_CAN_T_STOP)
			mode_str = __stringify_1(LLCE_CAN_T_STOP);
		else
			mode_str = __stringify_1(LLCE_CAN_T_START);

		dev_err(dev, "Failed to transition to %s\n", mode_str);
		return ret;
	}

	return 0;
}

static int start_llce_can(struct llce_can *llce)
{
	return set_controller_mode(llce->config, LLCE_CAN_T_START);
}

static int stop_llce_can(struct llce_can *llce)
{
	return set_controller_mode(llce->config, LLCE_CAN_T_STOP);
}

static u32 get_ntseg1(const struct can_bittiming *bt)
{
	return bt->prop_seg + bt->phase_seg1 - 1;
}

static u32 get_cbt(const struct can_bittiming *bt)
{
	u32 val, ntseg1, ntseg2, presdiv, nrjw;

	presdiv = bt->brp - 1;
	nrjw = bt->sjw - 1;
	ntseg1 = get_ntseg1(bt);
	ntseg2 = bt->phase_seg2 - 1;

	val = presdiv << LLCE_CBT_PRESDIV_OFFSET;
	val |= nrjw << LLCE_CBT_RJW_OFFSET;
	val |= ntseg2 << LLCE_CBT_TSEG2_OFFSET;
	val |= ntseg1 << LLCE_CBT_TSEG1_OFFSET;

	return val;
}

static u32 get_tdc_off(const struct can_bittiming *bt)
{
	/* Based on CiA 601-3 v. 1.0.0 */
	return bt->brp * (get_ntseg1(bt) + 2) - 1;
}

static bool is_canfd_dev(struct can_priv *can)
{
	if (can->ctrlmode & CAN_CTRLMODE_FD)
		return true;

	return false;
}

static int llce_set_data_bittiming(struct net_device *dev)
{
	int ret;
	struct llce_can *llce = netdev_priv(dev);
	struct llce_can_dev *llce_dev = &llce->common;
	struct can_priv *can = &llce_dev->can;
	struct llce_can_controller_fd_config *controller_fd;
	const struct can_bittiming *dbt = &can->data_bittiming;
	const struct can_bittiming *bt = &can->bittiming;
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_SETBAUDRATE,
		.cmd_list.set_baudrate = {
			.nominal_baudrate_config = get_cbt(bt),
			.controller_fd = {
				.fd_enable = is_canfd_dev(can),
				.data_baudrate_config = get_cbt(dbt),
				.controller_tx_bit_rate_switch = true,
				.trcv_delay_comp_enable = true,
				.trcv_delay_meas_enable = true,
				.trcv_delay_comp_offset = get_tdc_off(dbt),
			},
		},
	};

	if (is_canfd_dev(can) && bt->brp != dbt->brp) {
		netdev_err(dev, "Different values for nominal and data prescalers\n");
		return -EINVAL;
	}

	controller_fd = &cmd.cmd_list.set_baudrate.controller_fd;

	/* Disable delay compensation in loopback mode */
	if (can->ctrlmode & CAN_CTRLMODE_LOOPBACK)
		controller_fd->trcv_delay_comp_enable = false;

	ret = send_cmd_msg(llce->config, &cmd);
	if (ret) {
		netdev_err(dev, "Failed to set bit timing\n");
		return ret;
	}

	return 0;
}

static int llce_can_open(struct net_device *dev)
{
	struct llce_can *llce = netdev_priv(dev);
	struct llce_can_dev *common = &llce->common;
	struct can_priv *can = &common->can;
	int ret, ret1;

	if (!can->bittiming.bitrate)
		return -EINVAL;

	ret = open_candev(dev);
	if (ret)
		return ret;

	ret = llce_can_init(llce);
	if (ret)
		goto close_dev;

	ret = can_add_open_filter(llce->config, is_canfd_dev(can));
	if (ret)
		goto can_deinit;

	ret = llce_set_data_bittiming(dev);
	if (ret)
		goto can_deinit;

	common->rx = mbox_request_channel_byname(&llce->common.rx_client, "rx");
	if (IS_ERR(common->rx)) {
		netdev_err(dev, "Failed to get rx mailbox: %d\n", ret);
		ret = PTR_ERR(common->rx);
		goto can_deinit;
	}

	llce->tx = mbox_request_channel_byname(&llce->tx_client, "tx");
	if (IS_ERR(llce->tx)) {
		netdev_err(dev, "Failed to get tx mailbox: %d\n", ret);
		ret = PTR_ERR(llce->tx);
		goto release_rx_chan;
	}

	napi_enable(&common->napi);

	ret = start_llce_can(llce);
	if (ret)
		goto release_tx_chan;

	netif_start_queue(dev);

	return 0;

release_tx_chan:
	if (ret) {
		napi_disable(&common->napi);
		mbox_free_channel(llce->tx);
	}
release_rx_chan:
	if (ret)
		mbox_free_channel(common->rx);
can_deinit:
	if (ret) {
		ret1 = llce_can_deinit(llce);
		if (ret1) {
			ret = ret1;
			netdev_err(dev, "Failed to deinitialize LLCE CAN channel\n");
		}
	}
close_dev:
	if (ret)
		close_candev(dev);

	return ret;
}

static int llce_can_close(struct net_device *dev)
{
	struct llce_can *llce = netdev_priv(dev);
	struct llce_can_dev *common = &llce->common;
	int ret, ret1;

	netif_stop_queue(dev);

	ret = stop_llce_can(llce);
	if (ret)
		netdev_err(dev, "Failed to stop\n");

	napi_disable(&common->napi);
	mbox_free_channel(llce->tx);
	mbox_free_channel(common->rx);

	ret1 = llce_can_deinit(llce);
	if (ret1) {
		netdev_err(dev, "Failed to deinitialize LLCE CAN channel\n");
		if (!ret)
			ret = ret1;
	}

	close_candev(dev);

	return ret;
}

static void llce_tx_notif_callback(struct mbox_client *cl, void *msg)
{
	struct llce_tx_notif *notif = msg;
	struct llce_can *llce = container_of(cl, struct llce_can,
					     tx_client);
	struct net_device_stats *net_stats = &llce->common.can.dev->stats;

	/* This is executed in IRQ context */
	if (notif->error) {
		process_llce_can_error(&llce->common, notif->error, LLCE_TX);
		return;
	}

	net_stats->tx_bytes += can_get_echo_skb(llce->common.can.dev, 0, 0);
	net_stats->tx_packets++;
	netif_wake_queue(llce->common.can.dev);
}

static netdev_tx_t llce_can_start_xmit(struct sk_buff *skb,
				       struct net_device *dev)
{
	int ret;
	struct llce_can *llce = netdev_priv(dev);
	struct llce_can_dev *llce_dev = &llce->common;
	struct can_priv *can = &llce_dev->can;
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	struct llce_tx_msg msg = {
		.fd_msg = can_is_canfd_skb(skb),
		.long_msg = is_canfd_dev(can),
		.cf = cf,
	};

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);

	ret = mbox_send_message(llce->tx, &msg);
	if (ret < 0) {
		netdev_err(dev, "Failed to send CAN frame\n");
		return NETDEV_TX_BUSY;
	}

	/* Put the skb on can loopback stack */
	can_put_echo_skb(skb, dev, 0, 0);

	return NETDEV_TX_OK;
}

static const struct net_device_ops llce_can_netdev_ops = {
	.ndo_open	= llce_can_open,
	.ndo_stop	= llce_can_close,
	.ndo_start_xmit	= llce_can_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int llce_can_set_mode(struct net_device *netdev, enum can_mode mode)
{
	struct llce_can *llce = netdev_priv(netdev);
	struct llce_can_dev *common = &llce->common;
	struct net_device *dev = common->can.dev;
	int ret;

	switch (mode) {
	case CAN_MODE_START:
		ret = start_llce_can(llce);
		if (ret) {
			netdev_err(dev, "Failed to start LLCE\n");
			return ret;
		}

		netif_wake_queue(netdev);
		return ret;
	case CAN_MODE_STOP:
	case CAN_MODE_SLEEP:
		return stop_llce_can(llce);
	}

	return 0;
}

static int init_llce_chans(struct llce_can *llce, struct device *dev)
{
	long ret = 0;

	llce->config_client.dev = dev;
	llce->config_client.tx_block = true;
	llce->config_client.rx_callback = config_rx_callback;

	llce->tx_client.dev = dev;
	llce->tx_client.tx_block = false;
	llce->tx_client.rx_callback = llce_tx_notif_callback;

	llce->config = mbox_request_channel_byname(&llce->config_client,
						   "config");
	if (IS_ERR(llce->config)) {
		ret = PTR_ERR(llce->config);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get config mailbox\n");
		return ret;
	}

	return ret;
}

static int llce_init_can_priv(struct llce_can *llce, struct device *dev)
{
	struct llce_can_dev *common = &llce->common;
	struct can_priv *can = &common->can;
	unsigned long rate;
	int ret;

	llce->clk = devm_clk_get(dev, "can_pe");
	if (IS_ERR(llce->clk)) {
		dev_err(dev, "No clock available\n");
		return PTR_ERR(llce->clk);
	}

	ret = clk_prepare_enable(llce->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	rate = clk_get_rate(llce->clk);

	can->restart_ms = 1;
	can->state = CAN_STATE_STOPPED;
	can->clock.freq = rate;
	can->bittiming_const = &llce_can_bittiming;
	can->data_bittiming_const = &llce_can_data_bittiming;
	can->do_set_mode = &llce_can_set_mode;
	can->ctrlmode_supported = CAN_CTRLMODE_FD |
		CAN_CTRLMODE_FD_NON_ISO |
		CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_LISTENONLY;

	return 0;
}

static int llce_can_probe(struct platform_device *pdev)
{
	int ret;
	struct llce_can *llce;
	struct llce_can_dev *common;
	struct device *dev = &pdev->dev;
	struct net_device *netdev;

	common = init_llce_can_dev(dev, sizeof(struct llce_can),
				   LLCE_CAN_NETDEV_IF_NAME);
	if (IS_ERR(common))
		return PTR_ERR(common);

	llce = container_of(common, struct llce_can, common);
	netdev = common->can.dev;

	platform_set_drvdata(pdev, netdev);

	netdev->netdev_ops = &llce_can_netdev_ops;
	netdev->flags |= IFF_ECHO;

	init_completion(&llce->config_done);

	ret = llce_init_can_priv(llce, dev);
	if (ret)
		goto free_mem;

	ret = init_llce_chans(llce, dev);
	if (ret)
		goto free_mem;

	enable_llce_napi(common);

	ret = register_candev(netdev);
	if (ret) {
		dev_err(dev, "Failed to register %s\n", netdev->name);
		mbox_free_channel(llce->config);
	}

free_mem:
	if (ret)
		free_llce_netdev(common);

	return ret;
}

static int llce_can_remove(struct platform_device *pdev)
{
	struct net_device *netdev = platform_get_drvdata(pdev);
	struct llce_can *llce = netdev_priv(netdev);
	struct llce_can_dev *common = &llce->common;

	unregister_candev(netdev);
	netif_napi_del(&common->napi);

	clk_disable_unprepare(llce->clk);
	mbox_free_channel(llce->config);
	free_candev(netdev);

	return 0;
}

static int __maybe_unused llce_can_suspend(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct llce_can *llce = netdev_priv(dev);
	struct llce_can_dev *common = &llce->common;
	int ret = 0;

	if (netif_running(dev)) {
		ret = llce_can_close(dev);
		netif_device_detach(dev);
	}

	common->can.state = CAN_STATE_SLEEPING;

	return ret;
}

static int __maybe_unused llce_can_resume(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct llce_can *llce = netdev_priv(dev);
	struct llce_can_dev *common = &llce->common;
	int ret = 0;

	common->can.state = CAN_STATE_ERROR_ACTIVE;

	if (netif_running(dev)) {
		netif_device_attach(dev);
		ret = llce_can_open(dev);
	}

	return ret;
}

static const struct of_device_id llce_can_match[] = {
	{
		.compatible = "nxp,s32g-llce-can",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, llce_can_match);

static SIMPLE_DEV_PM_OPS(llce_can_pm_ops, llce_can_suspend, llce_can_resume);

static struct platform_driver llce_can_driver = {
	.probe = llce_can_probe,
	.remove = llce_can_remove,
	.driver = {
		.name = "llce_can",
		.of_match_table = llce_can_match,
		.pm = &llce_can_pm_ops,
	},
};
module_platform_driver(llce_can_driver)

MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("NXP LLCE CAN");
MODULE_LICENSE("Dual BSD/GPL");
