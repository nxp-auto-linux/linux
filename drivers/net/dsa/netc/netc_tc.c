// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include "netc_switch.h"

int netc_tc_query_caps(struct tc_query_caps_base *base)
{
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

static void netc_port_reset_mqprio(struct netc_port *port)
{
	struct net_device *ndev = port->dp->user;

	netdev_reset_tc(ndev);
	netif_set_real_num_tx_queues(ndev, NETC_TC_NUM);
}

int netc_tc_setup_mqprio(struct netc_switch *priv, int port_id,
			 struct tc_mqprio_qopt_offload *mqprio)
{
	struct netc_port *port = NETC_PORT(priv, port_id);
	struct tc_mqprio_qopt *qopt = &mqprio->qopt;
	struct net_device *ndev = port->dp->user;
	struct netlink_ext_ack *extack;
	u8 num_tc = qopt->num_tc;
	int tc, err;

	extack = mqprio->extack;

	if (!num_tc) {
		netc_port_reset_mqprio(port);
		return 0;
	}

	err = netdev_set_num_tc(ndev, num_tc);
	if (err)
		return err;

	for (tc = 0; tc < num_tc; tc++) {
		if (qopt->count[tc] != 1) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Only one TXQ per TC supported");
			return -EINVAL;
		}

		err = netdev_set_tc_queue(ndev, tc, 1, qopt->offset[tc]);
		if (err)
			goto reset_mqprio;
	}

	err = netif_set_real_num_tx_queues(ndev, num_tc);
	if (err)
		goto reset_mqprio;

	return 0;

reset_mqprio:
	netc_port_reset_mqprio(port);

	return err;
}
