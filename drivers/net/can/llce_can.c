// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2020 NXP */
#include <linux/can/dev.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
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
#define LLCE_CAN_MAX_RX_MB		16U

struct llce_xceiver {
	bool delay_comp;
	u32 delay_offset;
};

struct llce_can {
	struct can_priv can;
	struct llce_xceiver xceiver;

	struct completion config_done;

	struct mbox_client config_client, tx_client, rx_client;
	struct mbox_chan *config, *tx, *rx;

	struct clk *clk;
};

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
	if (cmd->return_value != LLCE_CAN_SUCCESS) {
		dev_err(dev, "LLCE FW error %d\n", cmd->return_value);
		return -EIO;
	}

	return 0;
}

static int llce_can_init(struct llce_can *llce)
{
	struct mbox_chan *conf_chan = llce->config;
	struct device *dev = llce_can_chan_dev(conf_chan);
	struct can_priv *can = &llce->can;
	u32 ctrl_config = 0;
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_INIT,
		.cmd_list.init = {
			.ctrl_config = LLCE_CAN_CONTROLLERCONFIG_RXINT_EN |
				LLCE_CAN_CONTROLLERCONFIG_TXINT_EN |
				LLCE_CAN_CONTROLLERCONFIG_CTRL_EN,
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
		dev_err(dev, "Failed to init LLCE CAN\n");
		return ret;
	}

	return 0;
}

static int can_add_open_filter(struct mbox_chan *conf_chan)
{
	struct device *dev = llce_can_chan_dev(conf_chan);
	struct llce_chan_priv *priv = conf_chan->con_priv;
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_SETFILTER,
		.cmd_list.set_filter = {
			.rx_filters_count = 1,
			.rx_filters = {
				{
					.id_mask = 0,
					.message_id = 0,
					.filter_id = 0,
					.mb_count = LLCE_CAN_MAX_TX_MB,
					.rx_dest_interface = priv->index,
					.entry_type = LLCE_CAN_ENTRY_CFG_MASKED,
				},
			},
		},
	};
	int ret;

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
	struct can_priv *can = &llce->can;
	struct llce_can_controller_fd_config *controller_fd;
	const struct can_bittiming *dbt = &can->data_bittiming;
	const struct can_bittiming *bt = &can->bittiming;
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_SETBAUDRATE,
		.cmd_list.set_baudrate = {
			.nominal_baudrate_config = get_cbt(bt),
			.controller_fd = {
				.can_fd_enable = is_canfd_dev(can),
				.can_data_baudrate_config = get_cbt(dbt),
				.can_controller_tx_bit_rate_switch = true,
				.can_trcv_delay_comp_enable = true,
				.can_trcv_delay_meas_enable = true,
				.can_trcv_delay_comp_offset = get_tdc_off(dbt),
			},
		},
	};

	if (bt->brp != dbt->brp) {
		netdev_err(dev, "Different values for nominal and data prescalers\n");
		return -EINVAL;
	}

	controller_fd = &cmd.cmd_list.set_baudrate.controller_fd;

	/* Disable delay compensation in loopback mode */
	if (can->ctrlmode & CAN_CTRLMODE_LOOPBACK)
		controller_fd->can_trcv_delay_comp_enable = false;

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
	int ret, ret1;

	ret = open_candev(dev);
	if (ret)
		return ret;

	ret = llce_can_init(llce);
	if (ret)
		goto close_dev;

	ret = can_add_open_filter(llce->config);
	if (ret)
		goto can_deinit;

	ret = llce_set_data_bittiming(dev);
	if (ret)
		goto can_deinit;

	llce->rx = mbox_request_channel_byname(&llce->rx_client, "rx");
	if (IS_ERR(llce->rx)) {
		netdev_err(dev, "Failed to get rx mailbox: %d\n", ret);
		ret = PTR_ERR(llce->rx);
		goto can_deinit;
	}

	llce->tx = mbox_request_channel_byname(&llce->tx_client, "tx");
	if (IS_ERR(llce->tx)) {
		netdev_err(dev, "Failed to get tx mailbox: %d\n", ret);
		ret = PTR_ERR(llce->tx);
		goto release_rx_chan;
	}

	ret = start_llce_can(llce);
	if (ret)
		goto release_tx_chan;

	netif_start_queue(dev);

	return 0;

release_tx_chan:
	if (ret)
		mbox_free_channel(llce->tx);
release_rx_chan:
	if (ret)
		mbox_free_channel(llce->rx);
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
	int ret, ret1;

	netif_stop_queue(dev);

	ret = stop_llce_can(llce);
	if (ret)
		netdev_err(dev, "Failed to stop\n");

	mbox_free_channel(llce->tx);
	mbox_free_channel(llce->rx);

	ret1 = llce_can_deinit(llce);
	if (ret1) {
		netdev_err(dev, "Failed to deinitialize LLCE CAN channel\n");
		if (!ret)
			ret = ret1;
	}

	close_candev(dev);

	return ret;
}

static void llce_process_error(struct llce_can *llce, enum llce_can_error error,
			       enum llce_can_module module)
{
	struct can_device_stats *can_stats = &llce->can.can_stats;
	struct net_device_stats *net_stats = &llce->can.dev->stats;
	struct net_device *dev = llce->can.dev;

	if (module == LLCE_TX)
		net_stats->tx_errors++;
	else
		net_stats->rx_errors++;

	switch (error) {
	case LLCE_ERROR_BCAN_ACKERR:
	case LLCE_ERROR_BCAN_BIT0ERR:
	case LLCE_ERROR_BCAN_BIT1ERR:
	case LLCE_ERROR_BCAN_CRCERR:
	case LLCE_ERROR_BCAN_FRMERR:
	case LLCE_ERROR_BCAN_FRZ_ENTER:
	case LLCE_ERROR_BCAN_FRZ_EXIT:
	case LLCE_ERROR_BCAN_LPM_EXIT:
	case LLCE_ERROR_BCAN_SRT_ENTER:
	case LLCE_ERROR_BCAN_STFERR:
	case LLCE_ERROR_BCAN_SYNC:
	case LLCE_ERROR_BCAN_UNKNOWN_ERROR:
		can_stats->bus_error++;
		return;
	case LLCE_ERROR_BUSOFF:
	case LLCE_ERROR_HARDWARE_BUSOFF:
		/* A restart is needed after a bus off error */
		can_bus_off(dev);
		can_stats->bus_off++;
		can_free_echo_skb(dev, 0);
		return;
	case LLCE_ERROR_DATA_LOST:
	case LLCE_ERROR_MB_NOTAVAILABLE:
	case LLCE_ERROR_RXOUT_FIFO_FULL:
		net_stats->rx_dropped++;
		return;
	default:
		break;
	}

	netdev_err(llce->can.dev, "Unhandled %d error %d\n", module, error);
}

static void llce_tx_notif_callback(struct mbox_client *cl, void *msg)
{
	struct llce_notif *notif = msg;
	struct llce_can *llce = container_of(cl, struct llce_can,
					     tx_client);
	struct net_device_stats *net_stats = &llce->can.dev->stats;

	/* This is executed in IRQ context */
	if (notif->error) {
		llce_process_error(llce, notif->error, LLCE_TX);
		return;
	}

	net_stats->tx_bytes += can_get_echo_skb(llce->can.dev, 0);
	net_stats->tx_packets++;
	netif_wake_queue(llce->can.dev);
}

static void unpack_word0(u32 word0, bool *rtr, bool *ide,
			 u32 *std_id, u32 *ext_id)
{
	if (word0 & LLCE_CAN_MB_IDE) {
		*ide = true;
		*ext_id = (word0 & CAN_EFF_MASK) >> CAN_SFF_ID_BITS;
		*std_id = word0 & CAN_SFF_MASK;
	} else {
		*ide = false;
		*std_id = (word0 & LLCE_CAN_MB_IDSTD_MASK) >>
			LLCE_CAN_MB_IDSTD_SHIFT;
	}

	*rtr = !!(word0 & LLCE_CAN_MB_RTR);
}

static void unpack_word1(u32 word1, bool *fdf, u8 *len, bool *brs,
			 bool *esi)
{
	*len = word1 & LLCE_CAN_MB_DLC_MASK;
	*brs = !!(word1 & LLCE_CAN_MB_BRS);
	*esi = !!(word1 & LLCE_CAN_MB_ESI);
	*fdf = !!(word1 & LLCE_CAN_MB_FDF);
}

static void llce_rx_notif_callback(struct mbox_client *cl, void *msg)
{
	struct llce_notif *notif = msg;
	struct llce_can *llce = container_of(cl, struct llce_can,
					     rx_client);
	struct net_device *dev = llce->can.dev;
	struct net_device_stats *net_stats = &llce->can.dev->stats;
	struct llce_can_mb *can_mb = notif->can_mb;
	struct sk_buff *skb;
	struct canfd_frame *cf;
	u32 std_id, ext_id;
	bool rtr, ide, brs, esi, fdf;
	u8 len;

	/* This is executed in IRQ context */
	if (notif->error) {
		llce_process_error(llce, notif->error, LLCE_RX);
		return;
	}

	unpack_word0(can_mb->word0, &rtr, &ide, &std_id, &ext_id);
	unpack_word1(can_mb->word1, &fdf, &len, &brs, &esi);

	if (fdf)
		skb = alloc_canfd_skb(dev, &cf);
	else
		skb = alloc_can_skb(dev, (struct can_frame **)&cf);

	if (!skb) {
		net_stats->rx_dropped++;
		goto notif_exit;
	}

	cf->can_id = std_id & CAN_SFF_MASK;

	if (fdf) {
		if (ide) {
			cf->can_id |= ((ext_id << CAN_SFF_ID_BITS) &
				       CAN_EFF_MASK);
			cf->can_id |= CAN_EFF_FLAG;
		}

		if (brs)
			cf->flags |= CANFD_BRS;

		if (esi)
			cf->flags |= CANFD_ESI;

	} else {
		if (rtr)
			cf->can_id |= CAN_RTR_FLAG;
	}
	cf->len = can_dlc2len(len);

	memcpy(cf->data, can_mb->payload, cf->len);

	net_stats->rx_packets++;
	net_stats->rx_bytes += cf->len;

	netif_receive_skb(skb);

notif_exit:
	return;
}

static netdev_tx_t llce_can_start_xmit(struct sk_buff *skb,
				       struct net_device *dev)
{
	int ret;
	struct llce_can *llce = netdev_priv(dev);
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	struct llce_tx_msg msg = {
		.fd = can_is_canfd_skb(skb),
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
	can_put_echo_skb(skb, dev, 0);

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
	struct net_device *dev = llce->can.dev;
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

static int get_llce_can_id(const char *node_name, unsigned long *id)
{
	const char *p = node_name + strlen(node_name) - 1;

	while (isdigit(*p))
		p--;

	return kstrtoul(p + 1, 10, id);
}

static void *get_netdev_name(struct device *dev)
{
	unsigned long id;
	char *dev_name;
	const char *node_name;
	size_t name_len;

	node_name = dev->of_node->name;
	if (get_llce_can_id(node_name, &id)) {
		dev_err(dev, "Failed to detect node id for: %s\n", node_name);
		return ERR_PTR(-EIO);
	}

	/* 0-99 device ids + \0 */
	name_len = strlen(LLCE_CAN_NETDEV_IF_NAME) + 3;
	dev_name = devm_kmalloc(dev, name_len, GFP_KERNEL);
	if (!dev_name)
		return ERR_PTR(-ENOMEM);

	snprintf(dev_name, name_len, LLCE_CAN_NETDEV_IF_NAME "%lu", id);

	return dev_name;
}

static int init_llce_chans(struct llce_can *llce, struct device *dev)
{
	llce->config_client.dev = dev;
	llce->config_client.tx_block = true;
	llce->config_client.rx_callback = config_rx_callback;

	llce->tx_client.dev = dev;
	llce->tx_client.tx_block = false;
	llce->tx_client.rx_callback = llce_tx_notif_callback;
	llce->rx_client.dev = dev;
	llce->rx_client.tx_block = true;
	llce->rx_client.rx_callback = llce_rx_notif_callback;

	llce->config = mbox_request_channel_byname(&llce->config_client,
						   "config");
	if (IS_ERR(llce->config)) {
		dev_err(dev, "Failed to get config mailbox\n");
		return PTR_ERR(llce->config);
	}

	return 0;
}

static int llce_init_can_priv(struct llce_can *llce, struct device *dev)
{
	unsigned long rate;
	int ret;
	struct can_priv *can = &llce->can;

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
	struct device *dev = &pdev->dev;
	struct net_device *netdev;
	char *dev_name;

	netdev = alloc_candev(sizeof(struct llce_can), 1);
	if (!netdev)
		return -ENOMEM;

	dev_name = get_netdev_name(dev);
	if (IS_ERR_VALUE(dev_name)) {
		ret = PTR_ERR(dev_name);
		goto free_mem;
	}

	strncpy(netdev->name, dev_name, sizeof(netdev->name));

	platform_set_drvdata(pdev, netdev);
	SET_NETDEV_DEV(netdev, &pdev->dev);

	netdev->netdev_ops = &llce_can_netdev_ops;
	netdev->flags |= IFF_ECHO;

	llce = netdev_priv(netdev);

	init_completion(&llce->config_done);

	ret = llce_init_can_priv(llce, dev);
	if (ret)
		goto free_mem;

	ret = init_llce_chans(llce, dev);
	if (ret)
		goto free_mem;

	ret = register_candev(netdev);
	if (ret) {
		dev_err(dev, "Failed to register %s\n", dev_name);
		mbox_free_channel(llce->config);
	}

free_mem:
	if (ret)
		free_candev(netdev);

	return ret;
}

static int llce_can_remove(struct platform_device *pdev)
{
	struct net_device *netdev = platform_get_drvdata(pdev);
	struct llce_can *llce = netdev_priv(netdev);

	unregister_candev(netdev);

	clk_disable_unprepare(llce->clk);
	mbox_free_channel(llce->config);
	free_candev(netdev);

	return 0;
}

static const struct of_device_id llce_can_match[] = {
	{
		.compatible = "nxp,s32g274a-llce-can",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, llce_can_match);

static struct platform_driver llce_can_driver = {
	.probe = llce_can_probe,
	.remove = llce_can_remove,
	.driver = {
		.name = "llce_can",
		.of_match_table = llce_can_match,
	},
};
module_platform_driver(llce_can_driver)

MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("NXP LLCE CAN");
MODULE_LICENSE("GPL");
