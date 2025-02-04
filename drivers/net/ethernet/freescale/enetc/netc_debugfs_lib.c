// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC NTMP (NETC Table Management Protocol) 2.0 driver
 * Copyright 2024 NXP
 */

#include <linux/fsl/netc_lib.h>

#include "ntmp_private.h"

int netc_show_tgst_entry(struct ntmp_user *user, struct seq_file *s,
			 u32 entry_id)
{
	struct tgst_query_data *qdata __free(kfree);
	int i, err;

	qdata = kzalloc(sizeof(*qdata), GFP_KERNEL);
	if (!qdata)
		return -ENOMEM;

	err = ntmp_tgst_query_entry(user, entry_id, qdata);
	if (err)
		return err;

	seq_puts(s, "Dump Time Gate Scheduling Table Entry:\n");
	seq_printf(s, "Entry ID:%d\n", entry_id);
	seq_printf(s, "Admin Base Time:%llu\n", le64_to_cpu(qdata->admin_bt));
	seq_printf(s, "Admin Cycle Time:%u\n", le32_to_cpu(qdata->admin_ct));
	seq_printf(s, "Admin Cycle Extend Time:%u\n",
		   le32_to_cpu(qdata->admin_ct_ext));
	seq_printf(s, "Admin Control List Length:%u\n",
		   le16_to_cpu(qdata->admin_cl_len));
	for (i = 0; i < le16_to_cpu(qdata->admin_cl_len); i++) {
		seq_printf(s, "Gate Entry %d info:\n", i);
		seq_printf(s, "\tAdmin time interval:%u\n",
			   le32_to_cpu(qdata->cfge_ge[i].interval));
		seq_printf(s, "\tAdmin Traffic Class states:%02x\n",
			   qdata->cfge_ge[i].tc_state);
		seq_printf(s, "\tAdministrative gate operation type:%u\n",
			   qdata->cfge_ge[i].hr_cb);
	}

	seq_printf(s, "Config Change Time:%llu\n", le64_to_cpu(qdata->oper_cfg_ct));
	seq_printf(s, "Config Change Error:%llu\n", le64_to_cpu(qdata->oper_cfg_ce));
	seq_printf(s, "Operation Base Time:%llu\n", le64_to_cpu(qdata->oper_bt));
	seq_printf(s, "Operation Cycle Time:%u\n", le32_to_cpu(qdata->oper_ct));
	seq_printf(s, "Operation Cycle Extend Time:%u\n",
		   le32_to_cpu(qdata->oper_ct_ext));
	seq_printf(s, "Operation Control List Length:%u\n",
		   le16_to_cpu(qdata->oper_cl_len));
	for (i = 0; i < le16_to_cpu(qdata->oper_cl_len); i++) {
		seq_printf(s, "Gate Entry %d info:\n", i);
		seq_printf(s, "\tOperation time interval:%u\n",
			   le32_to_cpu(qdata->olse_ge[i].interval));
		seq_printf(s, "\tOperation Traffic Class states:%02x\n",
			   qdata->olse_ge[i].tc_state);
		seq_printf(s, "\tOperation gate operation type:%u\n",
			   qdata->olse_ge[i].hr_cb);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_tgst_entry);
