// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2023 NXP */
#include <linux/can/dev.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>

#include "llce_can_core.h"

#define LLCE_CAN_MAX_FILTER_MB		(16U)

#define INVALID_HW_FIFO			(0xFFU)

static int get_filter_id(struct filter_state *filter, u16 *id)
{
	if (!filter)
		return -EINVAL;

	if (filter->advanced) {
		*id = filter->f.advanced.llce_can_Rx_filter.filter_addr;
		return 0;
	}

	*id = filter->f.base.filter_addr;

	return 0;
}

static void set_filter_fifo(struct filter_state *filter, u8 fifo)
{
	struct llce_can_rx_filter *rx_filter;

	if (filter->advanced)
		rx_filter = &filter->f.advanced.llce_can_Rx_filter;
	else
		rx_filter = &filter->f.base;

	rx_filter->rx_dest_interface = fifo;
}

static u8 get_filter_hwctrl(struct filter_state *filter)
{
	struct llce_can_rx_filter *rx_filter;

	if (filter->advanced)
		rx_filter = &filter->f.advanced.llce_can_Rx_filter;
	else
		rx_filter = &filter->f.base;

	return llce_filter_get_hw_ctrl(rx_filter->filter_id);
}

static struct can_ctrl_state *get_ctrl_state(struct llce_can_core *can_core,
					     u8 hw_ctrl)
{
	if (hw_ctrl >= ARRAY_SIZE(can_core->ctrls))
		return NULL;

	return &can_core->ctrls[hw_ctrl];
}

static int llce_get_fifo(struct llce_can_core *can_core, u8 hw_ctrl, u8 *fifo)
{
	struct mbox_chan *conf_chan = can_core->config;
	struct device *dev = get_can_core_dev(can_core);
	struct llce_config_msg msg = {
		.cmd = LLCE_GET_FIFO_INDEX,
		.fifo_cmd = {
			.hw_ctrl = hw_ctrl,
		},
	};
	int ret;

	ret = llce_send_config_cmd(conf_chan, &msg, &can_core->config_cmd_done);
	if (ret) {
		dev_err(dev, "Failed to get the FIFO index\n");
		return ret;
	}

	*fifo = msg.fifo_cmd.fifo;

	return 0;
}

static void llce_can_core_config_cb(struct mbox_client *cl, void *msg)
{
	struct llce_can_core *can_core = container_of(cl, struct llce_can_core,
						      config_client);
	complete(&can_core->config_cmd_done);
}

static void base_filter_init(struct llce_can_rx_filter *rx_filter, u8 hw_ctrl,
			     bool canfd)
{
	enum llce_can_rx_mb_length len;
	u16 filter = 0;

	/* Encode HW ctrl & MB type into filter id */
	llce_filter_set_hwctrl(&filter, hw_ctrl);
	llce_filter_set_mb_type(&filter, canfd);

	if (canfd)
		len = USE_LONG_MB;
	else
		len = USE_SHORT_MB;

	*rx_filter = (struct llce_can_rx_filter) {
		.id_mask = 0,
		.message_id = 0,
		.filter_id = filter,
		.mb_count = LLCE_CAN_MAX_FILTER_MB,
		.entry_type = LLCE_CAN_ENTRY_CFG_MASKED,
		.filter_mb_length = len,
	};
}

static void adv_filter_set_logging(struct llce_can_advanced_filter *afilt,
				   bool logging)
{
	enum llce_af_logging_options logging_opt;

	if (logging)
		logging_opt = LLCE_AF_LOGGING_ENABLED;
	else
		logging_opt = LLCE_AF_LOGGING_DISABLED;

	afilt->llce_can_advanced_feature.can_logging_feature = logging_opt;
}

static void adv_filter_basic_init(struct llce_can_advanced_filter *afilt,
				  u8 hw_ctrl, bool fd)
{
	afilt->llce_can_advanced_feature = (struct llce_can_advanced_feature) {
		.can_authentication_feature = LLCE_AF_AUTHENTICATION_DISABLED,
		.can_custom_processing = LLCE_AF_CUSTOMPROCESSING_DISABLED,
		.can_logging_feature = LLCE_AF_LOGGING_DISABLED,
		.host_receive = LLCE_AF_HOSTRECEIVE_ENABLED,
		.can2can_routing_table_idx = LLCE_CAN_ADVANCED_FILTER_NOT_USED,
		.can2eth_routing_table_idx = LLCE_CAN_ADVANCED_FILTER_NOT_USED,
	};

	base_filter_init(&afilt->llce_can_Rx_filter, hw_ctrl, fd);
}

static int setup_ctrl_host_filters(struct llce_can_core *can_core,
				   u8 hw_ctrl, bool fd,
				   struct filter_state *base,
				   struct filter_state *adv)
{
	struct can_ctrl_state *ctrl_state;
	struct llce_can_advanced_filter *afilt;
	struct llce_can_rx_filter *bfilt;

	ctrl_state = get_ctrl_state(can_core, hw_ctrl);
	if (!ctrl_state)
		return -EINVAL;

	memset(base, 0, sizeof(*base));
	memset(adv, 0, sizeof(*adv));

	bfilt = &base->f.base;
	afilt = &adv->f.advanced;

	adv->advanced = true;

	/* This will be used for basic RX traffic */
	base_filter_init(bfilt, hw_ctrl, fd);

	/* This will be used for basic RX traffic + logger */
	adv_filter_basic_init(afilt, hw_ctrl, true);
	adv_filter_set_logging(afilt, true);

	return 0;
}

static int register_ctrl_filter(struct llce_can_core *can_core,
				struct filter_state *filter,
				struct filter_state **ofilter)
{
	struct filter_state *sfilter;

	sfilter = kmemdup(filter, sizeof(*sfilter), GFP_KERNEL);
	if (!sfilter)
		return -ENOMEM;

	list_add_tail(&sfilter->link, &can_core->filters_list);

	if (ofilter)
		*ofilter = sfilter;

	return 0;
}

static void release_ctrl_filter(struct filter_state *filter)
{
	list_del(&filter->link);

	kfree(filter);
}

static int init_ctrl_fifo(struct llce_can_core *can_core, u8 hw_ctrl)
{
	struct can_ctrl_state *ctrl = get_ctrl_state(can_core, hw_ctrl);
	struct device *dev = get_can_core_dev(can_core);
	int ret;

	if (!ctrl)
		return -EINVAL;

	if (ctrl->fifo != INVALID_HW_FIFO)
		return 0;

	ret = llce_get_fifo(can_core, hw_ctrl, &ctrl->fifo);
	if (ret) {
		dev_err(dev, "Failed to get FIFO index for controller %u\n",
			hw_ctrl);
		return ret;
	}

	return 0;
}

static int add_fw_filter(struct llce_can_core *can_core,
			 u8 hw_ctrl, struct filter_state *filter,
			 bool register_filter,
			 struct filter_state **ofilter)
{
	struct device *dev = get_can_core_dev(can_core);
	struct can_ctrl_state *ctrl = get_ctrl_state(can_core, hw_ctrl);
	struct mbox_chan *conf_chan = can_core->config;
	enum llce_can_command_id *cmd_id;
	union llce_can_command_list *cmd_list;
	struct llce_can_set_advanced_filter_cmd *adv_filt_cmd;
	struct llce_can_set_filter_cmd *base_filt_cmd;
	struct llce_config_msg msg = {
		.cmd = LLCE_EXECUTE_FW_CMD,
		.fw_cmd = {
			.hw_ctrl = hw_ctrl,
		}
	};
	int ret;

	if (!ctrl)
		return -EINVAL;

	ret = init_ctrl_fifo(can_core, hw_ctrl);
	if (ret)
		return ret;

	set_filter_fifo(filter, ctrl->fifo);

	cmd_id = &msg.fw_cmd.cmd.cmd_id;
	cmd_list = &msg.fw_cmd.cmd.cmd_list;

	adv_filt_cmd = &cmd_list->set_advanced_filter;
	base_filt_cmd = &cmd_list->set_filter;

	if (filter->advanced) {
		*cmd_id = LLCE_CAN_CMD_SETADVANCEDFILTER;
		adv_filt_cmd->rx_filters_count = 1;
		adv_filt_cmd->advanced_filters[0] = filter->f.advanced;
	} else {
		*cmd_id = LLCE_CAN_CMD_SETFILTER;
		base_filt_cmd->rx_filters_count = 1;
		base_filt_cmd->rx_filters[0] = filter->f.base;
	}

	ret = llce_send_config_cmd(conf_chan, &msg, &can_core->config_cmd_done);
	if (ret) {
		if (ret != -EOPNOTSUPP)
			dev_err(dev, "Failed to add an RX filter to controller %u\n",
				hw_ctrl);
		return ret;
	}

	if (filter->advanced)
		filter->f.advanced = adv_filt_cmd->advanced_filters[0];
	else
		filter->f.base = base_filt_cmd->rx_filters[0];

	filter->enabled = true;

	if (register_filter)
		ret = register_ctrl_filter(can_core, filter, ofilter);

	return ret;
}

static int set_status_fw_filter(struct llce_can_core *can_core,
				u8 hw_ctrl, struct filter_state *filter,
				bool enable)
{
	u16 filter_id;
	struct device *dev = get_can_core_dev(can_core);
	struct mbox_chan *conf_chan = can_core->config;
	struct llce_config_msg msg = {
		.cmd = LLCE_EXECUTE_FW_CMD,
		.fw_cmd = {
			.hw_ctrl = hw_ctrl,
			.cmd = {
				.cmd_id = LLCE_CAN_CMD_SETFILTERENABLESTATUS,
				.cmd_list.change_filter.filter_enabled =
				    enable ? 1u : 0u,
			}
		},
	};
	int ret;

	if (!filter) {
		dev_err(dev, "Invalid filter\n");
		return -EINVAL;
	}

	ret = get_filter_id(filter, &filter_id);
	if (ret) {
		dev_err(dev, "Failed to get filter's id\n");
		return ret;
	}

	msg.fw_cmd.cmd.cmd_list.change_filter.filter_addr = filter_id;

	ret = llce_send_config_cmd(conf_chan, &msg, &can_core->config_cmd_done);
	if (ret) {
		dev_err(dev, "Failed to change the status of %u filter of the controller %u\n",
			filter_id, hw_ctrl);
		return ret;
	}

	filter->enabled = enable;

	return ret;
}

static int remove_fw_filter(struct llce_can_core *can_core,
			    u8 hw_ctrl, struct filter_state *filter)
{
	u16 filter_id;
	struct device *dev = get_can_core_dev(can_core);
	struct mbox_chan *conf_chan = can_core->config;
	struct llce_config_msg msg = {
		.cmd = LLCE_EXECUTE_FW_CMD,
		.fw_cmd = {
			.hw_ctrl = hw_ctrl,
			.cmd = {
				.cmd_id = LLCE_CAN_CMD_REMOVE_FILTER,
			}
		},
	};
	int ret;

	if (!filter)
		return -EINVAL;

	ret = get_filter_id(filter, &filter_id);
	if (ret) {
		dev_err(dev, "Failed to get filter's id for remove op\n");
		return ret;
	}

	msg.fw_cmd.cmd.cmd_list.change_filter.filter_addr = filter_id;

	ret = llce_send_config_cmd(conf_chan, &msg, &can_core->config_cmd_done);
	if (ret) {
		dev_err(dev, "Failed to remove filter %u of the controller %u\n",
			filter_id, hw_ctrl);
		return ret;
	}

	release_ctrl_filter(filter);

	return ret;
}

static int add_host_rx_filters(struct llce_can_core *can_core, u8 hw_ctrl,
			       bool fd)
{
	struct filter_state base, adv;
	struct can_ctrl_state *ctrl = get_ctrl_state(can_core, hw_ctrl);
	struct device *dev = get_can_core_dev(can_core);
	struct filter_state *dis_filter;
	bool has_logging_cap;
	int ret;

	if (!ctrl)
		return -EINVAL;

	ret = setup_ctrl_host_filters(can_core, hw_ctrl, fd, &base, &adv);
	if (ret) {
		dev_err(dev, "Failed to initialize filters\n");
		return ret;
	}

	/**
	 * The order matters because both filters are configured using the same
	 * set of settings except the logging feature.
	 */
	ret = add_fw_filter(can_core, hw_ctrl, &base, true, &ctrl->base_filter);
	if (ret) {
		dev_err(dev, "Failed to add base RX filter\n");
		return ret;
	}

	ret = add_fw_filter(can_core, hw_ctrl, &adv, true, &ctrl->logging_filter);
	if (ret && ret != -EOPNOTSUPP) {
		dev_err(dev, "Failed to add advanced RX filter\n");
		goto remove_base_filter;
	}

	if (ret == -EOPNOTSUPP)
		has_logging_cap = false;
	else
		has_logging_cap = true;

	if (!has_logging_cap)
		return 0;

	if (ctrl->logging)
		dis_filter = ctrl->base_filter;
	else
		dis_filter = ctrl->logging_filter;

	ret = set_status_fw_filter(can_core, hw_ctrl, dis_filter, false);
	if (ret) {
		dev_err(dev, "Failed to disable a filter\n");
		goto remove_adv_filter;
	}

remove_adv_filter:
	if (ret)
		(void)remove_fw_filter(can_core, hw_ctrl, ctrl->logging_filter);
remove_base_filter:
	if (ret)
		(void)remove_fw_filter(can_core, hw_ctrl, ctrl->base_filter);

	return ret;
}

static void init_ctrl_meta(struct can_ctrl_state *ctrl_state)
{
	ctrl_state->base_filter = NULL;
	ctrl_state->logging_filter = NULL;

	ctrl_state->fifo = INVALID_HW_FIFO;
}

/**
 * Assumption: LLCE Firmware will automatically remove all the filters added
 * for a CAN controller during the deinitialization sequence.
 */
static void release_host_rx_filters(struct llce_can_core *can_core, u8 hw_ctrl)
{
	struct can_ctrl_state *ctrl_state = get_ctrl_state(can_core, hw_ctrl);
	struct filter_state *filter, *next;

	if (!ctrl_state)
		return;

	list_for_each_entry_safe(filter, next, &can_core->filters_list, link) {
		if (get_filter_hwctrl(filter) != hw_ctrl)
			continue;

		release_ctrl_filter(filter);
	}

	init_ctrl_meta(ctrl_state);
}

static int get_hw_ctrl(struct net_device *netdev, u8 *hw_ctrl)
{
	size_t name_size = sizeof(netdev->name);
	char *ptr = &netdev->name[0];
	int ret;

	/* Look for a number */
	while (*ptr && !isdigit(*ptr) && name_size) {
		ptr++;
		name_size--;
	}

	ret = sscanf(ptr, "%hhu", hw_ctrl);
	if (ret <= 0) {
		netdev_err(netdev, "Failed to get controller id from '%s'\n",
			   netdev->name);
		return -EINVAL;
	}

	return 0;
}

static int register_can_dest(struct llce_can_core *can_core,
			     struct can_destination **dest)
{
	*dest = kmalloc(sizeof(**dest), GFP_KERNEL);
	if (!*dest)
		return -ENOMEM;

	mutex_lock(&can_core->can_dest_lock);
	list_add_tail(&(*dest)->link, &can_core->can_dest_list);
	mutex_unlock(&can_core->can_dest_lock);

	return 0;
}

static void release_can_dest(struct can_destination *dest)
{
	list_del(&dest->link);

	kfree(dest);
}

static void release_all_can_dest(struct llce_can_core *can_core)
{
	struct can_destination *dest, *next;

	mutex_lock(&can_core->can_dest_lock);
	list_for_each_entry_safe(dest, next, &can_core->can_dest_list, link)
		release_can_dest(dest);
	mutex_unlock(&can_core->can_dest_lock);
}

static int add_fw_destination(struct llce_can_core *can_core,
			      struct can_af_dest_rules *dest,
			      u8 *dest_id)
{
	struct device *dev = get_can_core_dev(can_core);
	struct mbox_chan *conf_chan = can_core->config;
	struct llce_can_create_af_destination *af_dest;
	struct llce_config_msg msg = {
		.cmd = LLCE_EXECUTE_FW_HIF_CMD,
		.fw_cmd = {
			.cmd = {
				.cmd_id = LLCE_CAN_CMD_CREATE_AF_DESTINATION,
			}
		},
	};
	int ret;

	if (!dest)
		return -EINVAL;

	af_dest = &msg.fw_cmd.cmd.cmd_list.create_af_dest;
	af_dest->rule = *dest;

	ret = llce_send_config_cmd(conf_chan, &msg, &can_core->config_cmd_done);
	if (ret) {
		dev_err(dev, "Failed to add a new destination\n");
		return ret;
	}

	*dest = af_dest->rule;
	*dest_id = af_dest->idx;

	return 0;
}

int llce_add_can_dest(struct llce_can_core *can_core,
		      struct llce_can_can2can_routing_table *can_dest,
		      u8 *dest_id)
{
	int ret;
	struct can_destination *dest;

	struct can_af_dest_rules af_dest = {
		.af_dest = {
			.can2can = *can_dest,
		},
		.af_dest_id = CAN_AF_CAN2CAN,
	};

	ret = register_can_dest(can_core, &dest);
	if (ret)
		return ret;

	ret = add_fw_destination(can_core, &af_dest, &dest->id);
	if (ret)
		goto release_dest;

	dest->dest = af_dest.af_dest.can2can;
	*dest_id = dest->id;

release_dest:
	if (ret)
		release_can_dest(dest);

	return ret;
}

static int llce_can_event(struct llce_can_core *can_core, unsigned long action,
			  struct net_device *netdev)
{
	struct can_priv *priv = safe_candev_priv(netdev);
	int ret = 0;
	u8 hw_ctrl;
	bool fd;

	if (!priv)
		return NOTIFY_BAD;

	ret = get_hw_ctrl(netdev, &hw_ctrl);
	if (ret)
		return NOTIFY_BAD;

	switch (action) {
	case NETDEV_UP:
		fd = !!(priv->ctrlmode & CAN_CTRLMODE_FD);
		ret = add_host_rx_filters(can_core, hw_ctrl, fd);
		break;
	case NETDEV_DOWN:
		release_host_rx_filters(can_core, hw_ctrl);
		break;
	default:
		break;
	};

	if (ret)
		return NOTIFY_BAD;

	return NOTIFY_OK;
}

static int change_rx_logging_filter(struct llce_can_core *can_core, u8 hw_ctrl,
				    bool logging)
{
	struct can_ctrl_state *ctrl_state;
	struct device *dev = get_can_core_dev(can_core);
	struct filter_state *en_filter, *dis_filter;
	u16 filter_id = 0;
	int ret;

	ctrl_state = get_ctrl_state(can_core, hw_ctrl);
	if (!ctrl_state)
		return -EINVAL;

	/**
	 * Nothing to do if the interface is not configured yet
	 */
	if (!ctrl_state->base_filter) {
		ctrl_state->logging = logging;
		return 0;
	}

	if (!logging) {
		/* Enable standard rx filter if disabled */
		en_filter = ctrl_state->base_filter;
		/* Disable logging filter if enabled */
		dis_filter = ctrl_state->logging_filter;
	} else {
		/* Enable logging filter */
		en_filter = ctrl_state->logging_filter;
		/* Disable standard filter */
		dis_filter = ctrl_state->base_filter;
	}

	ret = set_status_fw_filter(can_core, hw_ctrl, en_filter, true);
	if (ret) {
		(void)get_filter_id(en_filter, &filter_id);
		dev_err(dev, "Failed to enable filter %u for controller %u\n",
			filter_id, hw_ctrl);
		return ret;
	}

	ret = set_status_fw_filter(can_core, hw_ctrl, dis_filter, false);
	if (ret) {
		(void)get_filter_id(dis_filter, &filter_id);
		dev_err(dev, "Failed to disable filter %u for controller %u\n",
			filter_id, hw_ctrl);
		return ret;
	}

	ctrl_state->logging = logging;

	return ret;
}

static int llce_logger_event(struct llce_can_core *can_core,
			     unsigned long action,
			     struct net_device *netdev)
{
	int ret;
	u8 hw_ctrl;

	ret = get_hw_ctrl(netdev, &hw_ctrl);
	if (ret)
		return NOTIFY_BAD;

	switch (action) {
	case NETDEV_UP:
		ret = change_rx_logging_filter(can_core, hw_ctrl, true);
		break;
	case NETDEV_GOING_DOWN:
		ret = change_rx_logging_filter(can_core, hw_ctrl, false);
		break;
	default:
		break;
	}

	if (ret)
		return NOTIFY_BAD;

	return NOTIFY_OK;
}

static int llce_can_core_event(struct notifier_block *nb,
			       unsigned long action, void *data)
{
	struct net_device *netdev = netdev_notifier_info_to_dev(data);
	struct can_priv *priv = safe_candev_priv(netdev);
	struct llce_can_core *can_core;
	bool llce_logger_if = false, llce_can_if = false;

	if (!priv)
		return NOTIFY_DONE;

	if (strstr(netdev->name, LLCE_CAN_NETDEV_IF_NAME))
		llce_can_if = true;

	if (strstr(netdev->name, LLCE_LOGGER_NETDEV_IF_NAME))
		llce_logger_if = true;

	if (!llce_can_if && !llce_logger_if)
		return NOTIFY_DONE;

	can_core = container_of(nb, struct llce_can_core, notifier);

	if (llce_can_if)
		return llce_can_event(can_core, action, netdev);

	if (llce_logger_if)
		return llce_logger_event(can_core, action, netdev);

	return NOTIFY_DONE;
}

static int setup_config_mailbox(struct llce_can_core *can_core,
				struct device *dev)
{
	int ret = 0;

	init_completion(&can_core->config_cmd_done);

	can_core->config_client = (struct mbox_client) {
		.dev = dev,
		.tx_block = true,
		.rx_callback = llce_can_core_config_cb,
	};

	can_core->config = mbox_request_channel_byname(&can_core->config_client,
						       "config");
	if (IS_ERR(can_core->config)) {
		ret = PTR_ERR(can_core->config);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request config channel\n");
		return ret;
	}

	return ret;
}

static void release_config_mailbox(struct llce_can_core *can_core)
{
	mbox_free_channel(can_core->config);
}

static void init_hw_fifo(struct llce_can_core *can_core)
{
	size_t i;

	INIT_LIST_HEAD(&can_core->filters_list);
	INIT_LIST_HEAD(&can_core->can_dest_list);

	mutex_init(&can_core->can_dest_lock);

	for (i = 0; i < ARRAY_SIZE(can_core->ctrls); i++)
		init_ctrl_meta(&can_core->ctrls[i]);
}

static int llce_can_core_probe(struct platform_device *pdev)
{
	struct llce_can_core *can_core;
	struct device *dev = &pdev->dev;
	int ret;

	can_core = devm_kzalloc(dev, sizeof(*can_core), GFP_KERNEL);
	if (!can_core)
		return -ENOMEM;

	init_hw_fifo(can_core);

	can_core->notifier.notifier_call = llce_can_core_event;
	platform_set_drvdata(pdev, can_core);

	ret = setup_config_mailbox(can_core, dev);
	if (ret) {
		dev_err(dev, "Failed to request the config channel\n");
		return ret;
	}

	ret = register_netdevice_notifier(&can_core->notifier);
	if (ret) {
		release_config_mailbox(can_core);
		dev_err(dev, "Failed to register the notifier\n");
	}

	return ret;
}

static int llce_can_core_remove(struct platform_device *pdev)
{
	struct llce_can_core *can_core = platform_get_drvdata(pdev);
	u8 hw_ctrl;

	unregister_netdevice_notifier(&can_core->notifier);

	for (hw_ctrl = 0; hw_ctrl < ARRAY_SIZE(can_core->ctrls); hw_ctrl++)
		release_host_rx_filters(can_core, hw_ctrl);

	release_all_can_dest(can_core);

	release_config_mailbox(can_core);

	return 0;
}

static const struct of_device_id llce_can_core_match[] = {
	{
		.compatible = "nxp,s32g-llce-can-core",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, llce_can_core_match);

static int restore_fw_filter(struct llce_can_core *can_core,
			     struct filter_state *filter)
{
	struct device *dev = get_can_core_dev(can_core);
	bool enabled;
	u16 filter_id = 0;
	int ret;
	u8 hw_ctrl;

	hw_ctrl = get_filter_hwctrl(filter);

	enabled = filter->enabled;

	/* This will enable the filter by default */
	ret = add_fw_filter(can_core, hw_ctrl, filter, false, NULL);
	if (ret) {
		(void)get_filter_id(filter, &filter_id);
		dev_err(dev, "Failed to add filter 0x%x for ctrl %x\n",
			filter_id, hw_ctrl);

		return ret;
	}

	if (enabled)
		return 0;

	ret = set_status_fw_filter(can_core, hw_ctrl, filter, false);
	if (ret) {
		(void)get_filter_id(filter, &filter_id);
		dev_err(dev, "Failed change the state of the filter 0x%x for ctrl %x\n",
			filter_id, hw_ctrl);

		return ret;
	}

	return ret;
}

static int __maybe_unused llce_can_core_resume(struct device *device)
{
	struct llce_can_core *can_core = dev_get_drvdata(device);
	struct filter_state *filter;
	int ret;

	list_for_each_entry(filter, &can_core->filters_list, link) {
		ret = restore_fw_filter(can_core, filter);
		if (ret)
			return ret;
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(llce_can_core_pm_ops, NULL,
			 llce_can_core_resume);

static struct platform_driver llce_can_core_driver = {
	.probe = llce_can_core_probe,
	.remove = llce_can_core_remove,
	.driver = {
		.name = "llce_can_core",
		.of_match_table = llce_can_core_match,
		.pm = &llce_can_core_pm_ops,
	},
};
module_platform_driver(llce_can_core_driver)

MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("NXP LLCE CAN Core");
MODULE_LICENSE("Dual BSD/GPL");
