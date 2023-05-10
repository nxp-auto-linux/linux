/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/* Copyright 2023 NXP */

#ifndef LLCE_CAN_CORE_H
#define LLCE_CAN_CORE_H

#include <linux/can/dev/llce_can_common.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/mailbox_client.h>
#include <linux/notifier.h>
#include <linux/types.h>

struct filter_state {
	bool enabled;
	bool advanced;
	union {
		struct llce_can_rx_filter base;
		struct llce_can_advanced_filter advanced;
	} f;
	struct list_head link;
};

struct can_destination {
	struct llce_can_can2can_routing_table dest;
	struct list_head link;
	u8 id;
};

struct can_ctrl_state {
	struct filter_state *base_filter;
	struct filter_state *logging_filter;
	bool logging;
	u8 fifo;
};

struct llce_can_core {
	struct can_ctrl_state ctrls[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	struct notifier_block notifier;
	struct completion config_cmd_done;
	struct mbox_client config_client;
	struct list_head filters_list;
	struct list_head can_dest_list;
	/* Protects can_dest_list */
	struct mutex can_dest_lock;
	struct mbox_chan *config;
};

static inline struct device *get_can_core_dev(struct llce_can_core *can_core)
{
	struct mbox_chan *conf_chan = can_core->config;

	return llce_can_chan_dev(conf_chan);
}

#endif
