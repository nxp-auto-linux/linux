// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include "netc_switch.h"

int netc_port_get_mm(struct dsa_switch *ds, int port_id,
		     struct ethtool_mm_state *state)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	u32 val, rafs, lafs;

	if (!port->caps.pmac)
		return -EOPNOTSUPP;

	guard(mutex)(&port->mm_lock);

	val = netc_port_rd(port, NETC_MAC_MERGE_MMCSR);
	if (MMCSR_GET_ME(val) == MMCSR_ME_FP_1B_BOUNDARY ||
	    MMCSR_GET_ME(val) == MMCSR_ME_FP_4B_BOUNDARY)
		state->pmac_enabled = true;
	else
		state->pmac_enabled = false;

	switch (MMCSR_GET_VSTS(val)) {
	case 0:
		state->verify_status = ETHTOOL_MM_VERIFY_STATUS_DISABLED;
		break;
	case 2:
		state->verify_status = ETHTOOL_MM_VERIFY_STATUS_VERIFYING;
		break;
	case 3:
		state->verify_status = ETHTOOL_MM_VERIFY_STATUS_SUCCEEDED;
		break;
	case 4:
		state->verify_status = ETHTOOL_MM_VERIFY_STATUS_FAILED;
		break;
	case 5:
	default:
		state->verify_status = ETHTOOL_MM_VERIFY_STATUS_UNKNOWN;
		break;
	}

	rafs = MMCSR_GET_RAFS(val);
	state->tx_min_frag_size = ethtool_mm_frag_size_add_to_min(rafs);
	lafs = MMCSR_GET_LAFS(val);
	state->rx_min_frag_size = ethtool_mm_frag_size_add_to_min(lafs);
	state->tx_enabled = !!(val & MAC_MERGE_MMCSR_LPE);
	state->tx_active = state->tx_enabled &&
			   (state->verify_status == ETHTOOL_MM_VERIFY_STATUS_SUCCEEDED ||
			    state->verify_status == ETHTOOL_MM_VERIFY_STATUS_DISABLED);
	state->verify_enabled = !(val & MAC_MERGE_MMCSR_VDIS);
	state->verify_time = MMCSR_GET_VT(val);
	state->max_verify_time = (MAC_MERGE_MMCSR_VT >> 23) - 1;

	return 0;
}

static int netc_port_mm_wait_verify_status(struct netc_port *port, int verify_time)
{
	int timeout = verify_time * USEC_PER_MSEC * NETC_MM_VERIFY_RETRIES;
	u32 val;

	return read_poll_timeout(netc_port_rd, val, MMCSR_GET_VSTS(val) == 3,
				 USEC_PER_MSEC, timeout, true, port,
				 NETC_MAC_MERGE_MMCSR);
}

void netc_port_mm_commit_preemptible_tcs(struct netc_port *port)
{
	int preemptible_tcs = 0, err;
	u32 val;

	val = netc_port_rd(port, NETC_MAC_MERGE_MMCSR);
	if (MMCSR_GET_ME(val) != MMCSR_ME_FP_1B_BOUNDARY &&
	    MMCSR_GET_ME(val) != MMCSR_ME_FP_4B_BOUNDARY)
		goto end;

	if (!(val & MAC_MERGE_MMCSR_VDIS)) {
		err = netc_port_mm_wait_verify_status(port, MMCSR_GET_VT(val));
		if (err)
			goto end;
	}

	preemptible_tcs = port->preemptible_tcs;

end:
	netc_port_wr(port, NETC_PFPCR, preemptible_tcs);
}

static void netc_port_restart_emac_rx(struct netc_port *port)
{
	u32 val;

	val = netc_port_rd(port, NETC_PM_CMD_CFG(0));
	netc_port_wr(port, NETC_PM_CMD_CFG(0), val & ~PM_CMD_CFG_RX_EN);

	if (val & PM_CMD_CFG_RX_EN)
		netc_port_wr(port, NETC_PM_CMD_CFG(0), val);
}

int netc_port_set_mm(struct dsa_switch *ds, int port_id,
		     struct ethtool_mm_cfg *cfg,
		     struct netlink_ext_ack *extack)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	u32 add_frag_size, val;
	int err;

	if (!port->caps.pmac)
		return -EOPNOTSUPP;

	err = ethtool_mm_frag_size_min_to_add(cfg->tx_min_frag_size,
					      &add_frag_size, extack);
	if (err)
		return err;

	guard(mutex)(&port->mm_lock);

	val = netc_port_rd(port, NETC_MAC_MERGE_MMCSR);
	val = u32_replace_bits(val, cfg->verify_enabled ? 0 : 1,
			       MAC_MERGE_MMCSR_VDIS);

	if (cfg->tx_enabled)
		port->offloads |= NETC_FLAG_QBU;
	else
		port->offloads &= ~NETC_FLAG_QBU;

	/* If link is up, enable/disable MAC Merge right away */
	if (!(val & MAC_MERGE_MMCSR_LINK_FAIL)) {
		if (port->offloads & NETC_FLAG_QBU || cfg->pmac_enabled) {
			val = u32_replace_bits(val, MMCSR_ME_FP_4B_BOUNDARY,
					       MAC_MERGE_MMCSR_ME);

			/* When preemption is enabled, generation of PAUSE must
			 * be disabled.
			 */
			netc_port_set_tx_pause(port, false);
		} else {
			netc_port_set_tx_pause(port, port->tx_pause);
			val = u32_replace_bits(val, 0, MAC_MERGE_MMCSR_ME);
		}
	}

	val = u32_replace_bits(val, cfg->verify_time, MAC_MERGE_MMCSR_VT);
	val = u32_replace_bits(val, add_frag_size, MAC_MERGE_MMCSR_RAFS);

	netc_port_wr(port, NETC_MAC_MERGE_MMCSR, val);

	/* TODO: Do we really need to re-enable the Rx of the eMAC? */
	netc_port_restart_emac_rx(port);

	netc_port_mm_commit_preemptible_tcs(port);

	return 0;
}

void netc_port_get_mm_stats(struct dsa_switch *ds, int port_id,
			    struct ethtool_mm_stats *stats)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);

	if (!port->caps.pmac)
		return;

	stats->MACMergeFrameAssErrorCount = netc_port_rd(port, NETC_MAC_MERGE_MMFAECR);
	stats->MACMergeFrameSmdErrorCount = netc_port_rd(port, NETC_MAC_MERGE_MMFSECR);
	stats->MACMergeFrameAssOkCount = netc_port_rd(port, NETC_MAC_MERGE_MMFAOCR);
	stats->MACMergeFragCountRx = netc_port_rd(port, NETC_MAC_MERGE_MMFCRXR);
	stats->MACMergeFragCountTx = netc_port_rd(port, NETC_MAC_MERGE_MMFCTXR);
	stats->MACMergeHoldCount = netc_port_rd(port, NETC_MAC_MERGE_MMHCR);
}
