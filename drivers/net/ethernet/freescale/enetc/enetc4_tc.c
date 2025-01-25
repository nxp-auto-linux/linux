// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2025 NXP
 */
#include <net/pkt_sched.h>

#include "enetc_pf.h"
#include "enetc4_tc.h"

static int enetc4_tc_query_caps(struct net_device *ndev, void *type_data)
{
	struct tc_query_caps_base *base = type_data;

	switch (base->type) {
	case TC_SETUP_QDISC_MQPRIO: {
		struct tc_mqprio_caps *caps = base->caps;

		caps->validate_queue_counts = true;

		return 0;
	}
	default:
		return -EOPNOTSUPP;
	}
}

int enetc4_pf_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		       void *type_data)
{
	switch (type) {
	case TC_QUERY_CAPS:
		return enetc4_tc_query_caps(ndev, type_data);
	case TC_SETUP_QDISC_MQPRIO:
		return enetc_setup_tc_mqprio(ndev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}
