// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2025 NXP
 */

#include <linux/fsl/netc_lib.h>

#include "enetc_pf_common.h"
#include "enetc4_tc.h"

static bool enetc4_tc_cbs_is_enable(struct enetc_hw *hw, int tc)
{
	u32 val;

	val = enetc_port_rd(hw, ENETC4_PTCCBSR0(tc));
	if (val & PTCCBSR0_CBSE)
		return true;

	return false;
}

static void enetc4_set_ptccbsr(struct enetc_hw *hw, int tc,
			       bool en, u32 bw, u32 hi_credit)
{
	if (en) {
		u32 val = PTCCBSR0_CBSE;

		val |= (bw / 10) & PTCCBSR0_BW;
		val |= (bw % 10) << 16;

		enetc_port_wr(hw, ENETC4_PTCCBSR1(tc), hi_credit);
		enetc_port_wr(hw, ENETC4_PTCCBSR0(tc), val);

	} else {
		enetc_port_wr(hw, ENETC4_PTCCBSR1(tc), 0);
		enetc_port_wr(hw, ENETC4_PTCCBSR0(tc), 0);
	}
}

static u32 enetc4_get_tc_cbs_bw(struct enetc_hw *hw, int tc)
{
	u32 val, bw;

	val = enetc_port_rd(hw, ENETC4_PTCCBSR0(tc));
	bw = (val & PTCCBSR0_BW) * 10 + PTCCBSR0_FRACT_GET(val);

	return bw;
}

static u32 enetc4_get_tc_msdu(struct enetc_hw *hw, int tc)
{
	return enetc_port_rd(hw, ENETC4_PTCTMSDUR(tc)) & PTCTMSDUR_MAXSDU;
}

static int enetc4_setup_tc_cbs(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct tc_cbs_qopt_offload *cbs = type_data;
	u32 port_transmit_rate = priv->speed;
	u8 tc_nums = netdev_get_num_tc(ndev);
	u64 sysclk_freq = priv->sysclk_freq;
	u32 hi_credit_bit, hi_credit_reg;
	u8 high_prio_tc, second_prio_tc;
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	u32 max_interference_size;
	u32 port_frame_max_size;
	u32 bw, bw_sum;
	u8 tc;

	high_prio_tc = tc_nums - 1;
	second_prio_tc = tc_nums - 2;

	tc = netdev_txq_to_tc(ndev, cbs->queue);

	/* Support highest prio and second prio tc in cbs mode */
	if (tc != high_prio_tc && tc != second_prio_tc)
		return -EOPNOTSUPP;

	if (!cbs->enable) {
		/* Make sure the other TC that are numerically
		 * lower than this TC have been disabled.
		 */
		if (tc == high_prio_tc &&
		    enetc4_tc_cbs_is_enable(hw, second_prio_tc)) {
			dev_err(&ndev->dev,
				"Disable TC%d before disable TC%d\n",
				second_prio_tc, tc);
			return -EINVAL;
		}

		enetc4_set_ptccbsr(hw, tc, false, 0, 0);

		return 0;
	}

	/* The unit of idleslope and sendslope is kbps. And the sendslope should be
	 * a negative number, it can be calculated as follows, IEEE 802.1Q-2014
	 * Section 8.6.8.2 item g):
	 * sendslope = idleslope - port_transmit_rate
	 */
	if (cbs->idleslope - cbs->sendslope != port_transmit_rate * 1000L ||
	    cbs->idleslope < 0 || cbs->sendslope > 0)
		return -EOPNOTSUPP;

	port_frame_max_size = ndev->mtu + VLAN_ETH_HLEN + ETH_FCS_LEN;

	/* The unit of port_transmit_rate is Mbps, the unit of bw is 1/1000 */
	bw = cbs->idleslope / port_transmit_rate;
	bw_sum = bw;

	/* Make sure the credit-based shaper of highest priority TC has been enabled
	 * before the secondary priority TC.
	 */
	if (tc == second_prio_tc) {
		if (!enetc4_tc_cbs_is_enable(hw, high_prio_tc)) {
			dev_err(&ndev->dev,
				"Enable TC%d first before enable TC%d\n",
				high_prio_tc, second_prio_tc);
			return -EINVAL;
		}
		bw_sum += enetc4_get_tc_cbs_bw(hw, high_prio_tc);
	}

	if (bw_sum >= 1000) {
		dev_err(&ndev->dev,
			"The sum of all CBS Bandwidth can't exceed 1000\n");
		return -EINVAL;
	}

	/* For the AVB Class A (highest priority TC), the max_interfrence_size is
	 * maximum sized frame for the port.
	 * For the AVB Class B (second highest priority TC), the max_interfrence_size
	 * is calculated as below:
	 *
	 *      max_interference_size = (Ra * M0) / (R0 - Ra) + MA + M0
	 *
	 *	- RA: idleSlope for AVB Class A
	 *	- R0: port transmit rate
	 *	- M0: maximum sized frame for the port
	 *	- MA: maximum sized frame for AVB Class A
	 */

	if (tc == high_prio_tc) {
		max_interference_size = port_frame_max_size * 8;
	} else {
		u32 m0, ma;
		u64 ra, r0;

		m0 = port_frame_max_size * 8;
		ma = enetc4_get_tc_msdu(hw, high_prio_tc) * 8;
		ra = enetc4_get_tc_cbs_bw(hw, high_prio_tc) *
		     port_transmit_rate * 1000ULL;
		r0 = port_transmit_rate * 1000000ULL;
		max_interference_size = m0 + ma + (u32)div_u64(ra * m0, r0 - ra);
	}

	/* hiCredit bits calculate by:
	 *
	 * max_interference_size * (idleslope / port_transmit_rate)
	 */
	hi_credit_bit = max_interference_size * bw / 1000;

	/* Number of credits per bit is calculated as follows:
	 *
	 * (enetClockFrequency / port_transmit_rate) * 100
	 */
	hi_credit_reg = (u32)div_u64(sysclk_freq * 1000ULL * hi_credit_bit,
				     port_transmit_rate * 1000000ULL);

	enetc4_set_ptccbsr(hw, tc, true, bw, hi_credit_reg);

	return 0;
}

static int enetc4_devfn_to_port(struct pci_dev *pdev)
{
	switch (pdev->devfn) {
	case 0:
		return 0;
	case 64:
		return 1;
	case 128:
		return 2;
	default:
		return -1;
	}
}

int enetc4_pf_to_port(struct enetc_si *si)
{
	switch (si->revision) {
	case ENETC_REV_4_1:
		return enetc4_devfn_to_port(si->pdev);
	default:
		return -1;
	}
}

static void enetc4_pf_set_tc_msdu(struct enetc_hw *hw, u32 *max_sdu)
{
	int tc;

	for (tc = 0; tc < 8; tc++) {
		u32 val = ENETC_MAC_MAXFRM_SIZE;

		if (max_sdu[tc])
			val = max_sdu[tc] + VLAN_ETH_HLEN;

		val = u32_replace_bits(val, SDU_TYPE_MPDU, PTCTMSDUR_SDU_TYPE);
		enetc_port_wr(hw, ENETC4_PTCTMSDUR(tc), val);
	}
}

void enetc4_pf_reset_tc_msdu(struct enetc_hw *hw)
{
	u32 val = ENETC_MAC_MAXFRM_SIZE;
	int tc;

	val = u32_replace_bits(val, SDU_TYPE_MPDU, PTCTMSDUR_SDU_TYPE);

	for (tc = 0; tc < ENETC_NUM_TC; tc++)
		enetc_port_wr(hw, ENETC4_PTCTMSDUR(tc), val);
}

static void enetc4_pf_set_time_gating(struct enetc_hw *hw, bool en)
{
	u32 old_val, val;

	old_val = enetc_port_rd(hw, ENETC4_PTGSCR);
	val = u32_replace_bits(old_val, en ? 1 : 0, PTGSCR_TGE);
	if (val != old_val)
		enetc_port_wr(hw, ENETC4_PTGSCR, val);
}

static int enetc4_taprio_replace(struct net_device *ndev,
				 struct tc_taprio_qopt_offload *offload)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	int port, err;

	port = enetc4_pf_to_port(si);
	if (port < 0)
		return -EINVAL;

	err = enetc_setup_tc_mqprio(ndev, &offload->mqprio);
	if (err)
		return err;

	/* Set the maximum frame size for each traffic class */
	enetc4_pf_set_tc_msdu(hw, offload->max_sdu);
	enetc4_pf_set_time_gating(hw, true);

	err = netc_setup_taprio(&si->ntmp_user, port, offload);
	if (err)
		goto err_setup_taprio;

	priv->active_offloads |= ENETC_F_QBV;

	return 0;

err_setup_taprio:
	enetc4_pf_set_time_gating(hw, false);
	enetc4_pf_reset_tc_msdu(hw);
	enetc_reset_tc_mqprio(ndev);

	return err;
}

static void enetc4_taprio_destroy(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;

	enetc4_pf_set_time_gating(hw, false);
	enetc4_pf_reset_tc_msdu(hw);
	enetc_reset_tc_mqprio(ndev);
	enetc_reset_taprio_stats(priv);
	priv->active_offloads &= ~ENETC_F_QBV;
}

static int enetc4_setup_tc_taprio(struct net_device *ndev, void *type_data)
{
	struct tc_taprio_qopt_offload *offload = type_data;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	int err = 0;

	if (!(si->hw_features & ENETC_SI_F_QBV))
		return -EOPNOTSUPP;

	switch (offload->cmd) {
	case TAPRIO_CMD_REPLACE:
		err = enetc4_taprio_replace(ndev, offload);
		break;
	case TAPRIO_CMD_DESTROY:
		enetc4_taprio_destroy(ndev);
		break;
	case TAPRIO_CMD_STATS:
		enetc_taprio_stats(ndev, &offload->stats);
		break;
	case TAPRIO_CMD_QUEUE_STATS:
		enetc_taprio_queue_stats(ndev, &offload->queue_stats);
		break;
	default:
		err = -EOPNOTSUPP;
	}

	return err;
}

static void enetc4_pf_set_tc_tsd(struct enetc_hw *hw, int tc, bool en)
{
	enetc_port_wr(hw, ENETC4_PTCTSDR(tc), en ? PTCTSDR_TSDE : 0);
}

static int enetc4_setup_tc_txtime(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct tc_etf_qopt_offload *qopt = type_data;
	u8 tc_nums = netdev_get_num_tc(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	int i, tc;

	if (!tc_nums)
		return -EOPNOTSUPP;

	if (qopt->queue < 0 || qopt->queue >= ndev->real_num_tx_queues)
		return -EINVAL;

	tc = netdev_txq_to_tc(ndev, qopt->queue);
	if (tc < 0 || tc >= tc_nums)
		return -EINVAL;

	/* Accordiing to the NETC block guide, all traffic on the traffic class
	 * should use time specific departure operation.
	 */
	for (i = 0; i < ndev->tc_to_txq[tc].count; i++) {
		u16 offset = ndev->tc_to_txq[tc].offset + i;

		priv->tx_ring[offset]->tsd_enable = qopt->enable;
	}

	enetc4_pf_set_tc_tsd(hw, tc, qopt->enable);

	return 0;
}

int enetc4_pf_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		       void *type_data)
{
	switch (type) {
	case TC_QUERY_CAPS:
		return enetc_qos_query_caps(ndev, type_data);
	case TC_SETUP_QDISC_MQPRIO:
		return enetc_setup_tc_mqprio(ndev, type_data);
	case TC_SETUP_QDISC_CBS:
		return enetc4_setup_tc_cbs(ndev, type_data);
	case TC_SETUP_QDISC_TAPRIO:
		return enetc4_setup_tc_taprio(ndev, type_data);
	case TC_SETUP_QDISC_ETF:
		return enetc4_setup_tc_txtime(ndev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}
