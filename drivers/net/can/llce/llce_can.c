// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2020-2023 NXP */
#include <linux/can/dev.h>
#include <linux/can/dev/llce_can_common.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/ethtool.h>
#include <linux/kernel.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/processor.h>
#include <linux/stringify.h>
#include <net/devlink.h>

/* 10 ms timeout on all channels */
#define CHAN_TIMEOUT			10
#define LLCE_STATE_TRANSITION_NS	(500 * NSEC_PER_MSEC)
#define LLCE_CAN_DRV_NAME		"llce_can"

#define LLCE_CBT_PRESDIV_OFFSET		23U
#define LLCE_CBT_RJW_OFFSET		16U
#define LLCE_CBT_TSEG2_OFFSET		9U
#define LLCE_CBT_TSEG1_OFFSET		0U

struct llce_can_dl_params {
	struct llce_can *llce;
	u8 host_rx_mb;
	bool self_recv;
};

struct llce_can {
	struct llce_can_dev common; /* Must be the first member */

	struct completion config_done;

	struct mbox_client config_client, tx_client;
	struct mbox_chan *config, *tx;

	struct clk *clk;

	struct llce_can_dl_params *dl_params;

	const struct ethtool_ops *common_ethtool_ops;

	u8 fifo;
};

enum llce_can_devlink_param_id {
	LLCE_CAN_DL_PARAM_BASE_ID = DEVLINK_PARAM_GENERIC_ID_MAX,
	LLCE_CAN_DL_SELF_RECV_ID,
	LLCE_CAN_DL_HOST_RX_MB_ID,
};

static const struct devlink_ops llce_can_dl_ops = {
};

/* Used to protect access to llce_can_interfaces[i] */
static DEFINE_MUTEX(llce_can_interfaces_lock);
static struct llce_can *llce_can_interfaces[LLCE_CAN_CONFIG_MAXCTRL_COUNT];

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

static bool is_canfd_dev(struct can_priv *can);

static void config_rx_callback(struct mbox_client *cl, void *msg)
{
	struct llce_can *llce = container_of(cl, struct llce_can,
					     config_client);

	complete(&llce->config_done);
}

static int llce_get_can_stats(struct llce_can *llce,
			      struct llce_can_rx_tx_count *stats)
{
	struct mbox_chan *conf_chan = llce->config;
	struct device *dev = llce_can_chan_dev(conf_chan);
	struct llce_can_rx_tx_count *cmd_stats;
	struct llce_config_msg msg = {
		.cmd = LLCE_EXECUTE_SW_CMD,
		.sw_cmd = {
			.cmd = LLCE_GET_CAN_STATS,
		},
	};
	int ret;

	cmd_stats = &msg.sw_cmd.stats_cmd.stats;
	memset(cmd_stats, 0, sizeof(*cmd_stats));

	ret = llce_send_config_cmd(conf_chan, &msg, &llce->config_done);
	if (ret) {
		dev_err(dev, "Failed to get CAN stats: %d\n", ret);
		return ret;
	}

	*stats = *cmd_stats;

	return 0;
}

static int llce_can_init(struct llce_can *llce)
{
	struct mbox_chan *conf_chan = llce->config;
	struct device *dev = llce_can_chan_dev(conf_chan);
	struct llce_can_dev *llce_dev = &llce->common;
	struct can_priv *can = &llce_dev->can;
	u32 ctrl_config = 0;
	struct llce_config_msg msg = {
		.cmd = LLCE_EXECUTE_FW_CMD,
		.fw_cmd = {
			.hw_ctrl = LLCE_HW_CTRL_FROM_CHAN_IDX,
			.cmd = {
				.cmd_id = LLCE_CAN_CMD_INIT,
				.cmd_list.init = {
					.ctrl_config = LLCE_CAN_CONTROLLERCONFIG_CTRL_EN
					    | LLCE_CAN_CONTROLLERCONFIG_ABR_EN
					    | LLCE_CAN_CONTROLLERCONFIG_TST_END,
					.tx_mb_count = llce->dl_params->host_rx_mb,
				},
			},
		},
	};
	int ret;

	if (can->ctrlmode & CAN_CTRLMODE_LOOPBACK)
		ctrl_config |= LLCE_CAN_CONTROLLERCONFIG_LPB_EN;

	if (llce->dl_params->self_recv)
		ctrl_config |= LLCE_CAN_CONTROLLERCONFIG_SRX_EN;

	if (can->ctrlmode & CAN_CTRLMODE_LISTENONLY)
		ctrl_config |= LLCE_CAN_CONTROLLERCONFIG_LOM_EN;

	msg.fw_cmd.cmd.cmd_list.init.ctrl_config |= ctrl_config;

	ret = llce_send_config_cmd(conf_chan, &msg, &llce->config_done);
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
	struct llce_config_msg msg = {
		.cmd = LLCE_EXECUTE_FW_CMD,
		.fw_cmd = {
			.hw_ctrl = LLCE_HW_CTRL_FROM_CHAN_IDX,
			.cmd = {
				.cmd_id = LLCE_CAN_CMD_DEINIT,
			},
		},
	};
	int ret;

	ret = llce_send_config_cmd(conf_chan, &msg, &llce->config_done);
	if (ret) {
		dev_err(dev, "Failed to deinitialize LLCE CAN\n");
		return ret;
	}

	return 0;
}

static int llce_can_interfaces_set(struct llce_can *llce)
{
	unsigned int id = llce->common.id;

	if (id >= LLCE_CAN_CONFIG_MAXCTRL_COUNT)
		return -EINVAL;

	mutex_lock(&llce_can_interfaces_lock);
	llce_can_interfaces[id] = llce;
	mutex_unlock(&llce_can_interfaces_lock);

	return 0;
}

static bool state_transition(struct mbox_chan *conf_chan,
			     enum llce_can_ctrl_state state)
{
	int ret;
	struct device *dev = llce_can_chan_dev(conf_chan);
	struct mbox_client *cl = conf_chan->cl;
	struct llce_can *llce = container_of(cl, struct llce_can, config_client);
	struct llce_config_msg msg = {
		.cmd = LLCE_EXECUTE_FW_CMD,
		.fw_cmd = {
			.hw_ctrl = LLCE_HW_CTRL_FROM_CHAN_IDX,
			.cmd = {
				.cmd_id = LLCE_CAN_CMD_GETCONTROLLERMODE,
			},
		},
	};

	ret = llce_send_config_cmd(conf_chan, &msg, &llce->config_done);
	if (ret) {
		dev_err(dev, "Failed to get controller's state\n");
		return false;
	}

	return (msg.fw_cmd.cmd.cmd_list.get_controller_mode.controller_state == state);
}

static bool state_transition_timeout(struct mbox_chan *conf_chan,
				     enum llce_can_ctrl_state state,
				     ktime_t timeout)
{
	ktime_t cur = ktime_get();

	return state_transition(conf_chan, state) || ktime_after(cur, timeout);
}

static int set_controller_mode(struct mbox_chan *conf_chan,
			       enum llce_can_state_transition mode)
{
	ktime_t timeout = ktime_add_ns(ktime_get(), LLCE_STATE_TRANSITION_NS);
	struct device *dev = llce_can_chan_dev(conf_chan);
	struct mbox_client *cl = conf_chan->cl;
	struct llce_can *llce = container_of(cl, struct llce_can, config_client);
	const char *mode_str;
	enum llce_can_ctrl_state exp_state;
	struct llce_config_msg set_cmd = {
		.cmd = LLCE_EXECUTE_FW_CMD,
		.fw_cmd = {
			.hw_ctrl = LLCE_HW_CTRL_FROM_CHAN_IDX,
			.cmd = {
				.cmd_id = LLCE_CAN_CMD_SETCONTROLLERMODE,
				.cmd_list.set_controller_mode = {
					.transition = mode,
				},
			},
		}
	};
	int ret;

	if (mode == LLCE_CAN_T_STOP) {
		mode_str = __stringify_1(LLCE_CAN_T_STOP);
		exp_state = LLCE_CAN_STOPPED;
	} else {
		mode_str = __stringify_1(LLCE_CAN_T_START);
		exp_state = LLCE_CAN_STARTED;
	}

	ret = llce_send_config_cmd(conf_chan, &set_cmd, &llce->config_done);
	if (ret) {
		dev_err(dev, "Failed to send %s command\n", mode_str);
		return ret;
	}

	spin_until_cond(state_transition_timeout(conf_chan, exp_state, timeout));
	if (!state_transition(conf_chan, exp_state)) {
		dev_err(dev, "Failed the transition to %s state\n", mode_str);
		return -EIO;
	}

	return 0;
}

static int stop_llce_can(struct llce_can *llce)
{
	return set_controller_mode(llce->config, LLCE_CAN_T_STOP);
}

static int start_llce_can(struct llce_can *llce)
{
	int ret = set_controller_mode(llce->config, LLCE_CAN_T_START);

	if (ret)
		stop_llce_can(llce);

	return ret;
}

static int get_ntseg1(const struct can_bittiming *bt, u32 *tseg1)
{
	if (!bt->prop_seg && !bt->phase_seg1)
		return -EINVAL;

	*tseg1 = bt->prop_seg + bt->phase_seg1 - 1;
	return 0;
}

static int get_cbt(const struct can_bittiming *bt, u32 *cbt)
{
	int ret;
	u32 val, ntseg1, ntseg2, presdiv, nrjw;

	if (!bt->brp || !bt->sjw || !bt->phase_seg2)
		return -EINVAL;

	presdiv = bt->brp - 1;
	nrjw = bt->sjw - 1;

	ret = get_ntseg1(bt, &ntseg1);
	if (ret)
		return ret;

	ntseg2 = bt->phase_seg2 - 1;

	val = presdiv << LLCE_CBT_PRESDIV_OFFSET;
	val |= nrjw << LLCE_CBT_RJW_OFFSET;
	val |= ntseg2 << LLCE_CBT_TSEG2_OFFSET;
	val |= ntseg1 << LLCE_CBT_TSEG1_OFFSET;

	*cbt = val;
	return 0;
}

static int get_tdc_off(const struct can_bittiming *bt, u32 *tdc_off)
{
	int ret;
	u32 tseg1, tsegsum, prod;

	ret = get_ntseg1(bt, &tseg1);
	if (ret)
		return ret;

	/**
	 * Based on CiA 601-3 v. 1.0.0
	 * tdc = bt->brp * (tseg1 + 2) - 1
	 */
	if (unlikely(check_add_overflow(tseg1, 2u, &tsegsum)))
		return -E2BIG;

	if (unlikely(check_mul_overflow(bt->brp, tsegsum, &prod)))
		return -E2BIG;

	if (!prod)
		return -E2BIG;

	*tdc_off = prod - 1;
	return 0;
}

static bool is_canfd_dev(struct can_priv *can)
{
	if (can->ctrlmode & CAN_CTRLMODE_FD)
		return true;

	return false;
}

static int init_baudrate_configs(struct net_device *dev,
				 u32 *n_baudrate, u32 *d_baudrate,
				 u32 *tdc_offset)
{
	struct llce_can *llce = netdev_priv(dev);
	struct llce_can_dev *llce_dev = &llce->common;
	struct can_priv *can = &llce_dev->can;
	const struct can_bittiming *dbt = &can->data_bittiming;
	const struct can_bittiming *bt = &can->bittiming;
	bool fd = is_canfd_dev(can);
	int ret;

	*n_baudrate = 0u;
	*d_baudrate = 0u;
	*tdc_offset = 0u;

	ret = get_cbt(bt, n_baudrate);
	if (ret) {
		netdev_err(dev, "Failed to compute the nominal baudrate\n");
		return ret;
	}

	if (!fd)
		return 0;

	if (bt->brp != dbt->brp) {
		netdev_err(dev, "Different values for nominal and data prescalers\n");
		return -EINVAL;
	}

	ret = get_cbt(dbt, d_baudrate);
	if (ret) {
		netdev_err(dev, "Failed to compute the data baudrate\n");
		return ret;
	}

	ret = get_tdc_off(dbt, tdc_offset);
	if (ret) {
		netdev_err(dev, "Failed to determine Transceiver Delay Compensation Offset\n");
		return ret;
	}

	if (*tdc_offset > U8_MAX) {
		netdev_err(dev, "Transceiver Delay Compensation Offset exceeds its max allowed value 0x%x\n",
			   *tdc_offset);
		return -EINVAL;
	}

	return 0;
}

static int llce_set_data_bittiming(struct net_device *dev)
{
	struct llce_can *llce = netdev_priv(dev);
	struct llce_can_dev *llce_dev = &llce->common;
	struct can_priv *can = &llce_dev->can;
	struct llce_can_controller_fd_config *controller_fd;
	u8 fd_enable = is_canfd_dev(can) ? 1u : 0u;
	struct llce_can_set_baudrate_cmd *baudrate_cmd;
	struct llce_config_msg msg = {
		.cmd = LLCE_EXECUTE_FW_CMD,
		.fw_cmd = {
			.hw_ctrl = LLCE_HW_CTRL_FROM_CHAN_IDX,
			.cmd = {
				.cmd_id = LLCE_CAN_CMD_SETBAUDRATE,
				.cmd_list.set_baudrate = {
					/* It will be overwritten below */
					.nominal_baudrate_config = 0,
					.controller_fd = {
						.fd_enable = fd_enable,
						/* It will be overwritten below */
						.data_baudrate_config = 0,
						.controller_tx_bit_rate_switch = true,
						.trcv_delay_comp_enable = true,
						.trcv_delay_meas_enable = true,
						/* It will be overwritten below */
						.trcv_delay_comp_offset = 0,
					},
				},
			},
		},
	};
	u32 n_baudrate, d_baudrate, tdc_offset;
	int ret;

	ret = init_baudrate_configs(dev, &n_baudrate,
				    &d_baudrate, &tdc_offset);
	if (ret) {
		netdev_err(dev, "Failed to initialize baudrate settings\n");
		return ret;
	}

	baudrate_cmd = &msg.fw_cmd.cmd.cmd_list.set_baudrate;
	controller_fd = &baudrate_cmd->controller_fd;

	baudrate_cmd->nominal_baudrate_config = n_baudrate;
	controller_fd->data_baudrate_config = d_baudrate;
	controller_fd->trcv_delay_comp_offset = tdc_offset;

	/* Disable delay compensation in loopback mode */
	if (can->ctrlmode & CAN_CTRLMODE_LOOPBACK)
		controller_fd->trcv_delay_comp_enable = false;

	ret = llce_send_config_cmd(llce->config, &msg, &llce->config_done);
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

	/* Free RX queue */
	while (is_llce_rx_busy(common))
		;

	napi_disable(&common->napi);

	ret1 = llce_can_deinit(llce);
	if (ret1) {
		netdev_err(dev, "Failed to deinitialize LLCE CAN channel\n");
		if (!ret)
			ret = ret1;
	}

	mbox_free_channel(llce->tx);
	mbox_free_channel(common->rx);

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
	/* Put the skb on can loopback stack */
	can_put_echo_skb(skb, dev, 0, 0);

	ret = mbox_send_message(llce->tx, &msg);
	if (ret < 0) {
		can_free_echo_skb(dev, 0, NULL);
		netdev_err(dev, "Failed to send CAN frame\n");
		return NETDEV_TX_BUSY;
	}


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
		CAN_CTRLMODE_FD_NON_ISO | CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_BERR_REPORTING | CAN_CTRLMODE_LISTENONLY;

	return 0;
}

static int llce_can_dl_get_self_recv(struct devlink *dl, u32 id,
				     struct devlink_param_gset_ctx *ctx)
{
	struct llce_can_dl_params *llce_can_dl;

	llce_can_dl = devlink_priv(dl);
	ctx->val.vbool = llce_can_dl->self_recv;

	return 0;
}

static int llce_can_dl_set_self_recv(struct devlink *dl, u32 id,
				     struct devlink_param_gset_ctx *ctx)
{
	struct llce_can_dl_params *llce_can_dl;

	llce_can_dl = devlink_priv(dl);
	llce_can_dl->self_recv = ctx->val.vbool;

	return 0;
}

static int llce_can_dl_get_host_rx_mb(struct devlink *dl, u32 id,
				      struct devlink_param_gset_ctx *ctx)
{
	struct llce_can_dl_params *llce_can_dl;

	llce_can_dl = devlink_priv(dl);
	ctx->val.vu8 = llce_can_dl->host_rx_mb;

	return 0;
}

static int llce_can_dl_set_host_rx_mb(struct devlink *dl, u32 id,
				      struct devlink_param_gset_ctx *ctx)
{
	struct llce_can_dl_params *llce_can_dl;

	llce_can_dl = devlink_priv(dl);
	llce_can_dl->host_rx_mb = ctx->val.vu8;

	return 0;
}

static const struct devlink_param llce_can_devlink_params[] = {
	DEVLINK_PARAM_DRIVER(LLCE_CAN_DL_SELF_RECV_ID,
			     "self-recv", DEVLINK_PARAM_TYPE_BOOL,
			     BIT(DEVLINK_PARAM_CMODE_RUNTIME),
			     llce_can_dl_get_self_recv,
			     llce_can_dl_set_self_recv,
			     NULL),
	DEVLINK_PARAM_DRIVER(LLCE_CAN_DL_HOST_RX_MB_ID,
			     "host-rx-mb", DEVLINK_PARAM_TYPE_U8,
			     BIT(DEVLINK_PARAM_CMODE_RUNTIME),
			     llce_can_dl_get_host_rx_mb,
			     llce_can_dl_set_host_rx_mb,
			     NULL),
};

static void init_llce_can_dl_params(struct llce_can_dl_params *llce_can_dl)
{
	*llce_can_dl = (struct llce_can_dl_params){
		.self_recv = false,
		.host_rx_mb = 10,
	};
}

static int register_devlink_params(struct llce_can *llce, struct device *dev)
{
	struct llce_can_dl_params *llce_can_dl;
	struct devlink *devlink;
	int ret;

	devlink = devlink_alloc(&llce_can_dl_ops, sizeof(*llce_can_dl),
				dev);
	if (!devlink)
		return -ENOMEM;

	ret = devlink_register(devlink);
	if (ret)
		goto free_devlink;

	ret = devlink_params_register(devlink, llce_can_devlink_params,
				      ARRAY_SIZE(llce_can_devlink_params));
	if (ret)
		goto unreg_devlink;

	devlink_params_publish(devlink);

	llce_can_dl = devlink_priv(devlink);

	init_llce_can_dl_params(llce_can_dl);

	llce->dl_params = llce_can_dl;
	llce_can_dl->llce = llce;

unreg_devlink:
	if (ret)
		devlink_unregister(devlink);
free_devlink:
	if (ret)
		devlink_free(devlink);

	return ret;
}

static void unregister_devlink_params(struct llce_can *llce)
{
	struct devlink *devlink;

	devlink = priv_to_devlink(llce->dl_params);

	devlink_params_unpublish(devlink);
	devlink_params_unregister(devlink, llce_can_devlink_params,
				  ARRAY_SIZE(llce_can_devlink_params));
	devlink_unregister(devlink);
	devlink_free(devlink);
}

static const char can_stats_names[][ETH_GSTRING_LEN] = {
	"general_rx",
	"general_tx",
	"can2can_in",
	"can2can_out",
	"can2eth",
	"eth2can",
};

static size_t stats2array(struct llce_can_rx_tx_count *stats, u64 *data)
{
	u64 order[] = {
		stats->general_rx_count,
		stats->general_tx_count,
		stats->can2can_in_count,
		stats->can2can_out_count,
		stats->can2eth_count,
		stats->eth2can_count,
	};

	memcpy(data, order, sizeof(order));

	return ARRAY_SIZE(order);
}

static void get_can_ethtool_stats(struct net_device *dev,
				  struct ethtool_stats *stats,
				  u64 *data)
{
	struct llce_can *llce = netdev_priv(dev);
	const struct ethtool_ops *common_ops;
	struct llce_can_rx_tx_count llce_stats;
	size_t n_stats;
	int ret;

	memset(&llce_stats, 0, sizeof(llce_stats));
	ret = llce_get_can_stats(llce, &llce_stats);
	if (ret)
		netdev_err(dev, "Failed to get CAN stats\n");

	n_stats = stats2array(&llce_stats, data);
	common_ops = llce->common_ethtool_ops;

	common_ops->get_ethtool_stats(dev, stats, data + n_stats);
}

static void get_can_strings(struct net_device *dev, u32 stringset, u8 *buf)
{
	struct llce_can *llce = netdev_priv(dev);
	const struct ethtool_ops *common_ops;

	common_ops = llce->common_ethtool_ops;

	memcpy(buf, can_stats_names, sizeof(can_stats_names));
	buf += sizeof(can_stats_names);

	common_ops->get_strings(dev, stringset, buf);
}

static int get_can_sset_count(struct net_device *dev, int sset)
{
	struct llce_can *llce = netdev_priv(dev);
	const struct ethtool_ops *common_ops;
	size_t n_stats = ARRAY_SIZE(can_stats_names);
	int n_common_stats;

	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	common_ops = llce->common_ethtool_ops;

	n_common_stats = common_ops->get_sset_count(dev, sset);
	if (n_common_stats < 0)
		return n_common_stats;

	return n_common_stats + n_stats;
}

static const struct ethtool_ops llce_can_ethtool_ops = {
	.get_ethtool_stats = get_can_ethtool_stats,
	.get_strings = get_can_strings,
	.get_sset_count = get_can_sset_count,
};

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

	/* Add a decorator over the common ethtool ops */
	llce->common_ethtool_ops = netdev->ethtool_ops;
	netdev->ethtool_ops = &llce_can_ethtool_ops;

	platform_set_drvdata(pdev, netdev);

	netdev->netdev_ops = &llce_can_netdev_ops;
	netdev->flags |= IFF_ECHO;

	init_completion(&llce->config_done);

	ret = register_devlink_params(llce, dev);
	if (ret)
		goto free_mem;

	ret = llce_init_can_priv(llce, dev);
	if (ret)
		goto unreg_devlink;

	ret = init_llce_chans(llce, dev);
	if (ret)
		goto unreg_devlink;

	enable_llce_napi(common);

	ret = llce_can_interfaces_set(llce);
	if (ret) {
		dev_err(dev, "LLCE interface ID %d equal or greather than %u\n",
			LLCE_CAN_CONFIG_MAXCTRL_COUNT, common->id);
		goto free_conf_chan;
	}

	ret = register_candev(netdev);
	if (ret) {
		dev_err(dev, "Failed to register %s\n", netdev->name);
		goto free_conf_chan;
	}

free_conf_chan:
	if (ret)
		mbox_free_channel(llce->config);
unreg_devlink:
	if (ret)
		unregister_devlink_params(llce);
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
	unregister_devlink_params(llce);
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

static __init int llce_can_drv_init(void)
{
	return platform_driver_register(&llce_can_driver);
}

static void __exit llce_can_drv_exit(void)
{
	platform_driver_unregister(&llce_can_driver);
}

module_init(llce_can_drv_init);
module_exit(llce_can_drv_exit);

MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("NXP LLCE CAN");
MODULE_LICENSE("Dual BSD/GPL");
