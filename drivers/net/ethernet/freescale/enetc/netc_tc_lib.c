// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC NTMP (NETC Table Management Protocol) 2.0 driver
 * Copyright 2025 NXP
 */

#include <linux/fsl/netc_lib.h>

#include "ntmp_private.h"

int netc_setup_taprio(struct ntmp_user *user, u32 entry_id,
		      struct tc_taprio_qopt_offload *f)
{
	struct tgst_cfge_data *cfge __free(kfree) = NULL;
	struct netlink_ext_ack *extack = f->extack;
	u64 base_time = f->base_time;
	u64 max_cycle_time;
	int i, err;
	u32 size;

	if (!user->ops->get_tgst_free_words) {
		NL_SET_ERR_MSG_MOD(extack, "get_tgst_free_words() is undefined");
		return -EINVAL;
	}

	max_cycle_time = f->cycle_time + f->cycle_time_extension;
	if (max_cycle_time > U32_MAX) {
		NL_SET_ERR_MSG_MOD(extack, "Max cycle time exceeds U32_MAX");
		return -EINVAL;
	}

	/* Delete the pending administrative control list if it exists */
	err = ntmp_tgst_delete_admin_gate_list(user, entry_id);
	if (err)
		return err;

	if (f->num_entries > user->ops->get_tgst_free_words(user)) {
		NL_SET_ERR_MSG_MOD(extack, "TGST doesn't have enough free words");
		return -EINVAL;
	}

	size = struct_size(cfge, ge, f->num_entries);
	cfge = kzalloc(size, GFP_KERNEL);
	if (!cfge)
		return -ENOMEM;

	if (user->ops->adjust_base_time)
		base_time = user->ops->adjust_base_time(user, base_time,
							f->cycle_time);

	cfge->admin_bt = cpu_to_le64(base_time);
	cfge->admin_ct = cpu_to_le32(f->cycle_time);
	cfge->admin_ct_ext = cpu_to_le32(f->cycle_time_extension);
	cfge->admin_cl_len = cpu_to_le16(f->num_entries);
	for (i = 0; i < f->num_entries; i++) {
		struct tc_taprio_sched_entry *temp_entry = &f->entries[i];

		switch (temp_entry->command) {
		case TC_TAPRIO_CMD_SET_GATES:
			cfge->ge[i].hr_cb = HR_CB_SET_GATES;
			break;
		case TC_TAPRIO_CMD_SET_AND_HOLD:
			cfge->ge[i].hr_cb = HR_CB_SET_AND_HOLD;
			break;
		case TC_TAPRIO_CMD_SET_AND_RELEASE:
			cfge->ge[i].hr_cb = HR_CB_SET_AND_RELEASE;
			break;
		default:
			return -EOPNOTSUPP;
		}

		cfge->ge[i].tc_state = temp_entry->gate_mask;
		cfge->ge[i].interval = cpu_to_le32(temp_entry->interval);
	}

	err = ntmp_tgst_update_admin_gate_list(user, entry_id, cfge);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Update control list failed");
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(netc_setup_taprio);
