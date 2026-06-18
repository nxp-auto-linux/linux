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

static void netc_port_change_preemptible_tcs(struct netc_port *port,
					     unsigned long preemptible_tcs)
{
	if (!port->caps.pmac)
		return;

	port->preemptible_tcs = preemptible_tcs;
	netc_port_mm_commit_preemptible_tcs(port);
}

static void netc_port_reset_mqprio(struct netc_port *port)
{
	struct net_device *ndev = port->dp->user;

	netdev_reset_tc(ndev);
	netif_set_real_num_tx_queues(ndev, NETC_TC_NUM);
	netc_port_change_preemptible_tcs(port, 0);
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

	netc_port_change_preemptible_tcs(port, mqprio->preemptible_tcs);

	return 0;

reset_mqprio:
	netc_port_reset_mqprio(port);

	return err;
}

static bool netc_port_tc_cbs_is_enable(struct netc_port *port, int tc)
{
	return !!(netc_port_rd(port, NETC_PTCCBSR2(tc)) & PTCCBSR2_CBSE);
}

static void netc_port_enable_time_gating(struct netc_port *port, bool en)
{
	u32 old_val, val;

	old_val = netc_port_rd(port, NETC_PTGSCR);
	val = u32_replace_bits(old_val, en ? 1 : 0, PTGSCR_TGE);
	if (val != old_val)
		netc_port_wr(port, NETC_PTGSCR, val);
}

static void netc_port_set_tc_cbs_params(struct netc_port *port, int tc,
					bool en, u32 idleslope)
{
	if (en) {
		u32 val = PTCCBSR2_CBSE;

		val |= idleslope & PTCCBSR2_IDLESLOPE;

		netc_port_wr(port, NETC_PTCCBSR1(tc), 0xffffffff);
		netc_port_wr(port, NETC_PTCCBSR2(tc), val);
	} else {
		netc_port_wr(port, NETC_PTCCBSR1(tc), 0);
		netc_port_wr(port, NETC_PTCCBSR2(tc), 0);
	}
}

static u32 netc_port_get_tc_cbs_idleslope(struct netc_port *port, int tc)
{
	return netc_port_rd(port, NETC_PTCCBSR2(tc)) & PTCCBSR2_IDLESLOPE;
}

static int netc_port_setup_cbs(struct netc_port *port,
			       struct tc_cbs_qopt_offload *cbs)
{
	struct net_device *ndev = port->dp->user;
	u8 num_tc = netdev_get_num_tc(ndev);
	u8 top_prio_tc, second_prio_tc, tc;
	u32 total_idleslope;

	top_prio_tc = num_tc - 1;
	second_prio_tc = num_tc - 2;
	tc = netdev_txq_to_tc(ndev, cbs->queue);
	if (tc != top_prio_tc && tc != second_prio_tc)
		return -EOPNOTSUPP;

	if (!cbs->enable) {
		/* Make sure the other TC that are numerically lower than
		 * this TC have been disabled.
		 */
		if (tc == top_prio_tc &&
		    netc_port_tc_cbs_is_enable(port, second_prio_tc)) {
			netdev_err(ndev, "Disable TC%d before disable TC%d\n",
				   second_prio_tc, tc);
			return -EINVAL;
		}

		netc_port_set_tc_cbs_params(port, tc, false, 0);

		if (tc == top_prio_tc)
			port->offloads &= ~NETC_FLAG_QAV;

		return 0;
	}

	/* The unit of idleslope and sendslope is kbps. The sendslope should be
	 * a negative number, it can be calculated as follows, IEEE 802.1Q-2014
	 * Section 8.6.8.2 item g):
	 * sendslope = idleslope - port_transmit_rate
	 */
	if (cbs->idleslope - cbs->sendslope != port->speed * 1000L ||
	    cbs->idleslope < 0 || cbs->sendslope > 0)
		return -EOPNOTSUPP;

	total_idleslope = cbs->idleslope;
	/* Make sure the credit-based shaper of highest priority TC has been
	 * enabled before the secondary priority TC.
	 */
	if (tc == second_prio_tc) {
		if (!netc_port_tc_cbs_is_enable(port, top_prio_tc)) {
			netdev_err(ndev, "Enable TC%d first before enable TC%d\n",
				   top_prio_tc, second_prio_tc);
			return -EINVAL;
		}
		total_idleslope += netc_port_get_tc_cbs_idleslope(port, top_prio_tc);
	}

	/* The unit of port speed is Mbps */
	if (total_idleslope > port->speed * 1000L) {
		netdev_err(ndev,
			   "The total bandwidth of CBS can't exceed the link rate\n");
		return -EINVAL;
	}

	/* If CBS is going to be used in combination with frame preemption, then time
	 * gate scheduling should be enabled for the port.
	 */
	if (port->offloads & NETC_FLAG_QBU)
		netc_port_enable_time_gating(port, true);

	netc_port_set_tc_cbs_params(port, tc, true, cbs->idleslope);

	port->offloads |= NETC_FLAG_QAV;

	return 0;
}

int netc_tc_setup_cbs(struct netc_switch *priv, int port_id,
		      struct tc_cbs_qopt_offload *cbs)
{
	return netc_port_setup_cbs(priv->ports[port_id], cbs);
}
