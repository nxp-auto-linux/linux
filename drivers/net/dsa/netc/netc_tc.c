// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include "netc_switch.h"

static const struct netc_flower netc_flow_filter[] = {
	{
		BIT_ULL(FLOW_ACTION_GATE),
		BIT_ULL(FLOW_ACTION_POLICE),
		BIT_ULL(FLOW_DISSECTOR_KEY_ETH_ADDRS) |
		BIT_ULL(FLOW_DISSECTOR_KEY_VLAN),
		FLOWER_TYPE_PSFP
	},
};

int netc_tc_query_caps(struct tc_query_caps_base *base)
{
	switch (base->type) {
	case TC_SETUP_QDISC_MQPRIO: {
		struct tc_mqprio_caps *caps = base->caps;

		caps->validate_queue_counts = true;

		return 0;
	}
	case TC_SETUP_QDISC_TAPRIO: {
		struct tc_taprio_caps *caps = base->caps;

		caps->supports_queue_max_sdu = true;

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

		if (tc == top_prio_tc) {
			if (!(port->offloads & NETC_FLAG_QBV))
				netc_port_enable_time_gating(port, false);

			port->offloads &= ~NETC_FLAG_QAV;
		}

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

static bool netc_port_get_tge_status(struct netc_port *port)
{
	u32 val;

	val = netc_port_rd(port, NETC_PTGSCR);
	if (val & PTGSCR_TGE)
		return true;

	return false;
}

static int netc_port_setup_taprio(struct netc_port *port,
				  struct tc_taprio_qopt_offload *taprio)
{
	struct netc_switch *priv = port->switch_priv;
	u32 entry_id = port->index;
	bool tge;
	int err;

	/* Set the maximum frame size for each traffic class */
	netc_port_set_all_tc_msdu(port, taprio->max_sdu);

	tge = netc_port_get_tge_status(port);
	if (!tge)
		netc_port_enable_time_gating(port, true);

	err = netc_setup_taprio(&priv->ntmp, entry_id, taprio);
	if (err)
		goto disable_time_gating;

	port->offloads |= NETC_FLAG_QBV;

	return 0;

disable_time_gating:
	if (!tge)
		netc_port_enable_time_gating(port, false);

	netc_port_set_all_tc_msdu(port, NULL);

	return err;
}

static int netc_tc_taprio_replace(struct netc_switch *priv, int port_id,
				  struct tc_taprio_qopt_offload *taprio)
{
	struct netc_port *port = NETC_PORT(priv, port_id);
	struct netlink_ext_ack *extack = taprio->extack;
	int err;

	err = netc_tc_setup_mqprio(priv, port_id, &taprio->mqprio);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Setup mqprio failed");
		return err;
	}

	err = netc_port_setup_taprio(port, taprio);
	if (err)
		netc_port_reset_mqprio(port);

	return err;
}

static int netc_port_reset_taprio(struct netc_port *port)
{
	/* Remove both operational and administrative gate control list from
	 * the corresponding table entry by disabling time gate scheduling on
	 * the port.
	 */
	netc_port_enable_time_gating(port, false);

	/* Time gate scheduling should be enabled for the port if credit-based
	 * shaper is going to be used in combination with frame preemption.
	 */
	if (port->offloads & NETC_FLAG_QAV && port->offloads & NETC_FLAG_QBU)
		netc_port_enable_time_gating(port, true);

	/* Reset TC max SDU */
	netc_port_set_all_tc_msdu(port, NULL);

	port->offloads &= ~NETC_FLAG_QBV;

	return 0;
}

static int netc_tc_taprio_destroy(struct netc_switch *priv, int port_id)
{
	struct netc_port *port = NETC_PORT(priv, port_id);

	netc_port_reset_taprio(port);
	netc_port_reset_mqprio(port);

	return 0;
}

int netc_tc_setup_taprio(struct netc_switch *priv, int port_id,
			 struct tc_taprio_qopt_offload *taprio)
{
	switch (taprio->cmd) {
	case TAPRIO_CMD_REPLACE:
		return netc_tc_taprio_replace(priv, port_id, taprio);
	case TAPRIO_CMD_DESTROY:
		return netc_tc_taprio_destroy(priv, port_id);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct netc_flower *netc_parse_tc_flower(u64 actions, u64 keys)
{
	u64 key_acts, all_acts;
	int i;

	for (i = 0; i < ARRAY_SIZE(netc_flow_filter); i++) {
		key_acts = netc_flow_filter[i].key_acts;
		all_acts = netc_flow_filter[i].key_acts |
			   netc_flow_filter[i].opt_acts;

		/* key_acts must be matched */
		if ((actions & key_acts) == key_acts &&
		    (actions & all_acts) == actions &&
		    keys & netc_flow_filter[i].keys)
			return &netc_flow_filter[i];
	}

	return NULL;
}

int netc_port_flow_cls_replace(struct netc_port *port,
			       struct flow_cls_offload *f)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	struct netlink_ext_ack *extack = f->common.extack;
	struct netc_switch *priv = port->switch_priv;
	struct flow_action *action = &rule->action;
	struct flow_dissector *dissector;
	const struct netc_flower *flower;
	struct flow_action_entry *entry;
	u64 actions = 0;
	int i;

	dissector = rule->match.dissector;

	if (!flow_action_has_entries(action)) {
		NL_SET_ERR_MSG_MOD(extack, "At least one action is needed");
		return -EINVAL;
	}

	if (!flow_action_basic_hw_stats_check(action, extack))
		return -EOPNOTSUPP;

	flow_action_for_each(i, entry, action)
		actions |= BIT_ULL(entry->id);

	flower = netc_parse_tc_flower(actions, dissector->used_keys);
	if (!flower) {
		NL_SET_ERR_MSG_MOD(extack, "Unsupported actions or keys");
		return -EOPNOTSUPP;
	}

	switch (flower->type) {
	case FLOWER_TYPE_PSFP:
		return netc_setup_psfp(&priv->ntmp, port->index, f);
	default:
		NL_SET_ERR_MSG_MOD(extack, "Unsupported flower type");
		return -EOPNOTSUPP;
	}
}

static void netc_delete_flower_rule(struct ntmp_priv *ntmp,
				    struct netc_flower_rule *rule)
{
	switch (rule->flower_type) {
	case FLOWER_TYPE_PSFP:
		netc_delete_psfp_flower_rule(ntmp, rule);
		break;
	default:
		break;
	}
}

int netc_port_flow_cls_destroy(struct netc_port *port,
			       struct flow_cls_offload *f)
{
	struct netlink_ext_ack *extack = f->common.extack;
	struct netc_switch *priv = port->switch_priv;
	struct ntmp_priv *ntmp = &priv->ntmp;
	unsigned long cookie = f->cookie;
	struct netc_flower_rule *rule;

	guard(mutex)(&ntmp->flower_lock);
	rule = netc_find_flower_rule_by_cookie(ntmp, port->index, cookie);
	if (!rule) {
		NL_SET_ERR_MSG_MOD(extack, "Cannot find the rule");
		return -EINVAL;
	}

	netc_delete_flower_rule(ntmp, rule);

	return 0;
}

int netc_port_flow_cls_stats(struct netc_port *port,
			     struct flow_cls_offload *f)
{
	struct netlink_ext_ack *extack = f->common.extack;
	struct netc_switch *priv = port->switch_priv;
	u64 pkt_cnt = 0, drop_cnt = 0, byte_cnt = 0;
	struct ntmp_priv *ntmp = &priv->ntmp;
	unsigned long cookie = f->cookie;
	struct netc_flower_rule *rule;
	int err;

	guard(mutex)(&ntmp->flower_lock);
	rule = netc_find_flower_rule_by_cookie(ntmp, port->index, cookie);
	if (!rule) {
		NL_SET_ERR_MSG_MOD(extack, "Cannot find the rule");
		return -EINVAL;
	}

	switch (rule->flower_type) {
	case FLOWER_TYPE_PSFP:
		err = netc_psfp_flower_stat(ntmp, rule, &byte_cnt,
					    &pkt_cnt, &drop_cnt);
		if (err)
			goto err_out;
		break;
	default:
		NL_SET_ERR_MSG_MOD(extack, "Unknown flower type");
		return -EINVAL;
	}

	flow_stats_update(&f->stats, byte_cnt, pkt_cnt, drop_cnt,
			  rule->lastused, FLOW_ACTION_HW_STATS_IMMEDIATE);
	rule->lastused = jiffies;

	return 0;

err_out:
	NL_SET_ERR_MSG_MOD(extack, "Failed to get statistics");

	return err;
}

void netc_destroy_flower_list(struct netc_switch *priv)
{
	struct ntmp_priv *ntmp = &priv->ntmp;
	struct netc_flower_rule *rule;
	struct hlist_node *tmp;

	guard(mutex)(&ntmp->flower_lock);
	hlist_for_each_entry_safe(rule, tmp, &ntmp->flower_list, node)
		netc_delete_flower_rule(ntmp, rule);
}
