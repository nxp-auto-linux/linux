/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/* Copyright 2023 NXP */

#ifndef LLCE_CAN_CORE_H
#define LLCE_CAN_CORE_H

#include <linux/can/dev/llce_can_common.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/mailbox_client.h>
#include <linux/notifier.h>
#include <linux/types.h>

#include "llce_can_core_debugfs.h"

struct filter_state {
	bool enabled;
	bool advanced;
	union {
		struct llce_can_rx_filter base;
		struct llce_can_advanced_filter advanced;
	} f;
	u8 hw_ctrl;
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
	/* Protects filters_list */
	struct mutex filters_lock;
	struct list_head can_dest_list;
	/* Protects can_dest_list */
	struct mutex can_dest_lock;
	struct llce_can_core_debugfs debugfs;
	struct mbox_chan *config;
};

static inline struct device *get_can_core_dev(struct llce_can_core *can_core)
{
	struct mbox_chan *conf_chan = can_core->config;

	return llce_can_chan_dev(conf_chan);
}

static inline struct llce_can_rx_filter *
get_base_filter(struct filter_state *filter)
{
	if (filter->advanced)
		return &filter->f.advanced.llce_can_Rx_filter;

	return &filter->f.base;
}

static inline int get_filter_addr(struct filter_state *filter, u16 *addr)
{
	struct llce_can_rx_filter *rx_filter;

	if (!filter)
		return -EINVAL;

	rx_filter = get_base_filter(filter);

	*addr = rx_filter->filter_addr;

	return 0;
}

int llce_add_can_dest(struct llce_can_core *can_core,
		      struct llce_can_can2can_routing_table *can_dest,
		      u8 *dest_id);

int llce_add_can_filter(struct llce_can_core *can_core, u8 hw_ctrl,
			struct filter_state *filter);

int llce_set_filter_status(struct llce_can_core *can_core, u16 filter_addr,
			   bool enabled);

int llce_remove_filter(struct llce_can_core *can_core, u16 filter_addr);

#endif
