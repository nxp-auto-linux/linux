/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef __NETC_LIB_H
#define __NETC_LIB_H

#include <linux/fsl/ntmp.h>
#include <net/pkt_sched.h>

#if IS_ENABLED(CONFIG_NXP_NETC_LIB)
int netc_setup_taprio(struct ntmp_user *user, u32 entry_id,
		      struct tc_taprio_qopt_offload *f);
#else
static inline int netc_setup_taprio(struct ntmp_user *user, u32 entry_id,
				    struct tc_taprio_qopt_offload *f)
{
	return 0;
}
#endif

#endif
