/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/* Copyright 2023 NXP */
#ifndef LLCE_CAN_CORE_DEBUGFS_H
#define LLCE_CAN_CORE_DEBUGFS_H

#include <linux/debugfs.h>

struct debugfs_u8_array {
	u8 max_elems;
	u8 n_elements;
	u8 *array;
};

typedef int (*debugfs_cmd_t)(void *, u64);
typedef int (*debugfs_cmd_get_val_t)(void *, u64 *);

struct debugfs_cmd_block {
	debugfs_cmd_t clb;
	debugfs_cmd_get_val_t get_val;
};

struct debugfs_choice {
	const char *name;
	u64 id;
};

struct debugfs_choice_block {
	const struct debugfs_choice *choices;
	const struct debugfs_choice *selected;
	size_t n_choices;
};

struct llce_can_dest_debugfs {
	u8 dest_hw_ch_list[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	u32 routing_opts;
	u32 can_id_remap;
	struct debugfs_u8_array ch_list;
	struct debugfs_cmd_block add_exec;
	u8 id;
};

struct llce_can_filters_debugfs {
	struct llce_can_rx_filter base_opts;
	struct llce_can_advanced_feature adv_opts;
	struct debugfs_choice_block status;
	/* Basic options */
	struct debugfs_choice_block entry;
	struct debugfs_choice_block mb_type;
	/* Advanced options */
	struct debugfs_choice_block auth;
	struct debugfs_choice_block host;
	struct debugfs_choice_block logging;
	struct debugfs_choice_block custom_proc;
	struct debugfs_cmd_block add_exec;
	struct debugfs_cmd_block set_status_exec;
	struct debugfs_cmd_block remove_exec;
	u8 hw_ctrl;
};

struct llce_can_core_debugfs {
	struct llce_can_dest_debugfs can_dest;
	struct llce_can_filters_debugfs can_filters;
	struct dentry *root_dir;
};

int llce_create_debugfs(struct llce_can_core_debugfs *root);
void llce_remove_debugs(struct llce_can_core_debugfs *root);
#endif
