// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2024 NXP */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/unaligned.h>

#include "enetc_pf_common.h"
#include "enetc4_debugfs.h"
#include "enetc4_tc.h"

#define ENETC_SI_MAX_RING_NUM	8

#define ENETC_MAC_FILTER_TYPE_UC	BIT(0)
#define ENETC_MAC_FILTER_TYPE_MC	BIT(1)
#define ENETC_MAC_FILTER_TYPE_ALL	(ENETC_MAC_FILTER_TYPE_UC | \
					 ENETC_MAC_FILTER_TYPE_MC)

#define ntmp_user_to_enetc_si(user)	\
	container_of((user), struct enetc_si, ntmp_user)

static void enetc4_get_port_caps(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	u32 val;

	val = enetc_port_rd(hw, ENETC4_ECAPR1);
	pf->caps.num_vsi = (val & ECAPR1_NUM_VSI) >> 24;
	pf->caps.num_msix = ((val & ECAPR1_NUM_MSIX) >> 12) + 1;

	val = enetc_port_rd(hw, ENETC4_ECAPR2);
	pf->caps.num_rx_bdr = (val & ECAPR2_NUM_RX_BDR) >> 16;
	pf->caps.num_tx_bdr = val & ECAPR2_NUM_TX_BDR;

	val = enetc_port_rd(hw, ENETC4_PMCAPR);
	pf->caps.half_duplex = (val & PMCAPR_HD) ? 1 : 0;

	val = enetc_port_rd(hw, ENETC4_PSIMAFCAPR);
	pf->caps.mac_filter_num = val & PSIMAFCAPR_NUM_MAC_AFTE;
}

static void enetc4_pf_set_si_primary_mac(struct enetc_hw *hw, int si,
					 const u8 *addr)
{
	u16 lower = get_unaligned_le16(addr + 4);
	u32 upper = get_unaligned_le32(addr);

	if (si != 0) {
		__raw_writel(upper, hw->port + ENETC4_PSIPMAR0(si));
		__raw_writew(lower, hw->port + ENETC4_PSIPMAR1(si));
	} else {
		__raw_writel(upper, hw->port + ENETC4_PMAR0);
		__raw_writew(lower, hw->port + ENETC4_PMAR1);
	}
}

static void enetc4_pf_get_si_primary_mac(struct enetc_hw *hw, int si,
					 u8 *addr)
{
	u32 upper;
	u16 lower;

	upper = __raw_readl(hw->port + ENETC4_PSIPMAR0(si));
	lower = __raw_readw(hw->port + ENETC4_PSIPMAR1(si));

	put_unaligned_le32(upper, addr);
	put_unaligned_le16(lower, addr + 4);
}

static void enetc4_pf_set_si_mac_promisc(struct enetc_hw *hw, int si,
					 bool uc_promisc, bool mc_promisc)
{
	u32 val = enetc_port_rd(hw, ENETC4_PSIPMMR);

	if (uc_promisc)
		val |= PSIPMMR_SI_MAC_UP(si);
	else
		val &= ~PSIPMMR_SI_MAC_UP(si);

	if (mc_promisc)
		val |= PSIPMMR_SI_MAC_MP(si);
	else
		val &= ~PSIPMMR_SI_MAC_MP(si);

	enetc_port_wr(hw, ENETC4_PSIPMMR, val);
}

static void enetc4_pf_set_si_uc_hash_filter(struct enetc_hw *hw, int si,
					    u64 hash)
{
	enetc_port_wr(hw, ENETC4_PSIUMHFR0(si), lower_32_bits(hash));
	enetc_port_wr(hw, ENETC4_PSIUMHFR1(si), upper_32_bits(hash));
}

static void enetc4_pf_set_si_mc_hash_filter(struct enetc_hw *hw, int si,
					    u64 hash)
{
	enetc_port_wr(hw, ENETC4_PSIMMHFR0(si), lower_32_bits(hash));
	enetc_port_wr(hw, ENETC4_PSIMMHFR1(si), upper_32_bits(hash));
}

static void enetc4_pf_set_loopback(struct net_device *ndev, bool en)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	u32 val;

	val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	val = u32_replace_bits(val, en ? 1 : 0, PM_CMD_CFG_LOOP_EN);
	/* Default to select MAC level loopback mode if loopback is enabled. */
	val = u32_replace_bits(val, en ? LPBCK_MODE_MAC_LEVEL : 0,
			       PM_CMD_CFG_LPBK_MODE);

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_pf_clear_maft_entries(struct enetc_pf *pf)
{
	int i;

	for (i = 0; i < pf->num_mfe; i++)
		ntmp_maft_delete_entry(&pf->si->ntmp_user, i);

	pf->num_mfe = 0;
}

static int enetc4_pf_add_maft_entries(struct enetc_pf *pf,
				      struct enetc_mac_addr *mac,
				      int mac_cnt)
{
	struct maft_entry_data maft = {};
	u16 si_bit = BIT(0);
	int i, err;

	maft.cfge.si_bitmap = cpu_to_le16(si_bit);
	for (i = 0; i < mac_cnt; i++) {
		ether_addr_copy(maft.keye.mac_addr, mac[i].addr);
		err = ntmp_maft_add_entry(&pf->si->ntmp_user, i, &maft);
		if (unlikely(err)) {
			pf->num_mfe = i;
			goto clear_maft_entries;
		}
	}

	pf->num_mfe = mac_cnt;

	return 0;

clear_maft_entries:
	enetc4_pf_clear_maft_entries(pf);

	return  err;
}

static int enetc4_pf_set_uc_exact_filter(struct enetc_pf *pf)
{
	int max_num_mfe = pf->caps.mac_filter_num;
	struct enetc_mac_filter mac_filter = {};
	struct net_device *ndev = pf->si->ndev;
	struct enetc_hw *hw = &pf->si->hw;
	struct enetc_mac_addr *mac_tbl;
	struct netdev_hw_addr *ha;
	int i = 0, err;
	int mac_cnt;

	netif_addr_lock_bh(ndev);

	mac_cnt = netdev_uc_count(ndev);
	if (!mac_cnt) {
		netif_addr_unlock_bh(ndev);
		/* clear both MAC hash and exact filters */
		enetc4_pf_set_si_uc_hash_filter(hw, 0, 0);
		enetc4_pf_clear_maft_entries(pf);

		return 0;
	}

	if (mac_cnt > max_num_mfe) {
		err = -ENOSPC;
		goto unlock_netif_addr;
	}

	mac_tbl = kcalloc(mac_cnt, sizeof(*mac_tbl), GFP_ATOMIC);
	if (!mac_tbl) {
		err = -ENOMEM;
		goto unlock_netif_addr;
	}

	netdev_for_each_uc_addr(ha, ndev) {
		enetc_add_mac_addr_ht_filter(&mac_filter, ha->addr);
		ether_addr_copy(mac_tbl[i++].addr, ha->addr);
	}

	netif_addr_unlock_bh(ndev);

	/* Set temporary unicast hash filters in case of Rx loss when
	 * updating MAC address filter table
	 */
	enetc4_pf_set_si_uc_hash_filter(hw, 0, *mac_filter.mac_hash_table);
	enetc4_pf_clear_maft_entries(pf);

	if (!enetc4_pf_add_maft_entries(pf, mac_tbl, i))
		enetc4_pf_set_si_uc_hash_filter(hw, 0, 0);

	kfree(mac_tbl);

	return 0;

unlock_netif_addr:
	netif_addr_unlock_bh(ndev);

	return err;
}

static void enetc4_pf_set_mac_hash_filter(struct enetc_pf *pf, int type)
{
	struct net_device *ndev = pf->si->ndev;
	struct enetc_mac_filter *mac_filter;
	struct enetc_hw *hw = &pf->si->hw;
	struct netdev_hw_addr *ha;

	netif_addr_lock_bh(ndev);
	if (type & ENETC_MAC_FILTER_TYPE_UC) {
		mac_filter = &pf->mac_filter[UC];
		enetc_reset_mac_addr_filter(mac_filter);
		netdev_for_each_uc_addr(ha, ndev)
			enetc_add_mac_addr_ht_filter(mac_filter, ha->addr);

		enetc4_pf_set_si_uc_hash_filter(hw, 0,
						*mac_filter->mac_hash_table);
	}

	if (type & ENETC_MAC_FILTER_TYPE_MC) {
		mac_filter = &pf->mac_filter[MC];
		enetc_reset_mac_addr_filter(mac_filter);
		netdev_for_each_mc_addr(ha, ndev)
			enetc_add_mac_addr_ht_filter(mac_filter, ha->addr);

		enetc4_pf_set_si_mc_hash_filter(hw, 0,
						*mac_filter->mac_hash_table);
	}
	netif_addr_unlock_bh(ndev);
}

static void enetc4_pf_set_mac_filter(struct enetc_pf *pf, int type)
{
	/* Currently, the MAC address filter table (MAFT) only has 4 entries,
	 * and multiple multicast addresses for filtering will be configured
	 * in the default network configuration, so MAFT is only suitable for
	 * unicast filtering. If the number of unicast addresses exceeds the
	 * table capacity, the MAC hash filter will be used.
	 */
	if (type & ENETC_MAC_FILTER_TYPE_UC && enetc4_pf_set_uc_exact_filter(pf)) {
		/* Fall back to the MAC hash filter */
		enetc4_pf_set_mac_hash_filter(pf, ENETC_MAC_FILTER_TYPE_UC);
		/* Clear the old MAC exact filter */
		enetc4_pf_clear_maft_entries(pf);
	}

	if (type & ENETC_MAC_FILTER_TYPE_MC)
		enetc4_pf_set_mac_hash_filter(pf, ENETC_MAC_FILTER_TYPE_MC);
}

static void enetc4_pf_get_mm(struct enetc_ndev_priv *priv,
			     struct enetc_mm *mm)
{
	struct enetc_hw *hw = &priv->si->hw;
	u32 val;

	val = enetc_port_rd(hw, ENETC4_MMCSR);
	mm->pmac_enabled = !!(val & MMCSR_ME);
	mm->tx_enabled = !!(val & MMCSR_LPE);
	mm->verify_enabled = !(val & MMCSR_VDIS);
	mm->vsts = FIELD_GET(MMCSR_VSTS, val);
	mm->rafs = FIELD_GET(MMCSR_RAFS, val);
	mm->lafs = FIELD_GET(MMCSR_LAFS, val);
	mm->vt = FIELD_GET(MMCSR_VT, val);
}

static void enetc4_restart_emac_rx(struct enetc_si *si)
{
	struct enetc_hw *hw = &si->hw;
	u32 val;

	val = enetc_port_rd(hw, ENETC4_PM_CMD_CFG(0));

	enetc_port_wr(hw, ENETC4_PM_CMD_CFG(0), val & ~PM_CMD_CFG_RX_EN);

	if (val & PM_CMD_CFG_RX_EN)
		enetc_port_wr(hw, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_pf_set_mm(struct enetc_ndev_priv *priv,
			     struct ethtool_mm_cfg *cfg,
			     u32 add_frag_size)
{
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	u32 val;

	val = enetc_port_rd(hw, ENETC4_MMCSR);
	if (cfg->pmac_enabled)
		val = u32_replace_bits(val, MMCSR_ME_4B_BOUNDARY, MMCSR_ME);
	else
		val = u32_replace_bits(val, MMCSR_ME_DISABLE, MMCSR_ME);

	if (cfg->verify_enabled)
		val &= ~MMCSR_VDIS;
	else
		val |= MMCSR_VDIS;

	/* If link is up, enable/disable MAC Merge right away */
	if (!(val & MMCSR_LINK_FAIL)) {
		if (priv->active_offloads & ENETC_F_QBU) {
			val = u32_replace_bits(val, MMCSR_ME_4B_BOUNDARY, MMCSR_ME);

			/* When preemption is enabled, generation of PAUSE must be
			 * disabled.
			 */
			enetc_port_wr(hw, ENETC4_PPAUONTR, 0);
			enetc_port_wr(hw, ENETC4_PPAUOFFTR, 0);
		} else {
			val = u32_replace_bits(val, MMCSR_ME_DISABLE, MMCSR_ME);
		}
	}

	val = u32_replace_bits(val, cfg->verify_time, MMCSR_VT);
	val = u32_replace_bits(val, add_frag_size, MMCSR_RAFS);

	enetc_port_wr(hw, ENETC4_MMCSR, val);

	enetc4_restart_emac_rx(si);
}

static int enetc4_mm_wait_tx_active(struct enetc_hw *hw, int verify_time)
{
	int timeout = verify_time * USEC_PER_MSEC * ENETC_MM_VERIFY_RETRIES;
	u32 val;

	return read_poll_timeout(enetc_port_rd, val,
				 MMCSR_VSTS_GET(val) == MMCSR_VSTS_SUCCESSFUL,
				 ENETC_MM_VERIFY_SLEEP_US, timeout,
				 true, hw, ENETC4_MMCSR);
}

static void enetc4_pf_set_preemptible_tcs(struct enetc_ndev_priv *priv)
{
	struct enetc_hw *hw = &priv->si->hw;
	u32 preemptible_tcs = 0, val;
	int err;

	val = enetc_port_rd(hw, ENETC4_MMCSR);
	if (!(val & MMCSR_ME))
		goto out;

	if (!(val & MMCSR_VDIS)) {
		err = enetc4_mm_wait_tx_active(hw, FIELD_GET(MMCSR_VT, val));
		if (err)
			goto out;
	}

	preemptible_tcs = priv->preemptible_tcs;

out:
	enetc_port_wr(hw, ENETC4_PFPCR, preemptible_tcs);
}

static void enetc4_get_psi_hw_features(struct enetc_si *si)
{
	struct enetc_hw *hw = &si->hw;
	u32 val;

	val = enetc_port_rd(hw, ENETC4_PMCAPR);
	if (FIELD_GET(PMCAPR_FP, val) == PMCAPR_FP_SUPP)
		si->hw_features |= ENETC_SI_F_QBU;

	val = enetc_port_rd(hw, ENETC4_PCAPR);
	if (val & PCAPR_TGS)
		si->hw_features |= ENETC_SI_F_QBV;

	val = enetc_port_rd(hw, ENETC4_IPCAPR);
	if (val & IPCAPR_ISID)
		si->hw_features |= ENETC_SI_F_PSFP;
}

static const struct enetc_pf_ops enetc4_pf_ops = {
	.set_si_primary_mac = enetc4_pf_set_si_primary_mac,
	.get_si_primary_mac = enetc4_pf_get_si_primary_mac,
	.get_mm = enetc4_pf_get_mm,
	.set_mm = enetc4_pf_set_mm,
	.set_preemptible_tcs = enetc4_pf_set_preemptible_tcs,
};

static int enetc4_pf_struct_init(struct enetc_si *si)
{
	struct enetc_pf *pf = enetc_si_priv(si);

	pf->si = si;
	pf->total_vfs = pci_sriov_get_totalvfs(si->pdev);
	pf->ops = &enetc4_pf_ops;

	if (pf->total_vfs) {
		pf->vf_state = kcalloc(pf->total_vfs,
				       sizeof(struct enetc_vf_state),
				       GFP_KERNEL);
		if (!pf->vf_state)
			return -ENOMEM;
	}

	enetc4_get_port_caps(pf);
	enetc4_get_psi_hw_features(si);

	return 0;
}

static void enetc4_pf_struct_free(struct enetc_pf *pf)
{
	kfree(pf->vf_state);
}

static u32 enetc4_psicfgr0_val_construct(bool is_vf, u32 num_tx_bdr, u32 num_rx_bdr)
{
	u32 val;

	val = ENETC_PSICFGR0_SET_TXBDR(num_tx_bdr);
	val |= ENETC_PSICFGR0_SET_RXBDR(num_rx_bdr);
	val |= ENETC_PSICFGR0_SIVC(ENETC_VLAN_TYPE_C | ENETC_VLAN_TYPE_S);

	if (is_vf)
		val |= ENETC_PSICFGR0_VTE | ENETC_PSICFGR0_SIVIE;

	return val;
}

static void enetc4_default_rings_allocation(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	u32 num_rx_bdr, num_tx_bdr, val;
	u32 vf_tx_bdr, vf_rx_bdr;
	int i, rx_rem, tx_rem;

	if (pf->caps.num_rx_bdr < ENETC_SI_MAX_RING_NUM + pf->caps.num_vsi)
		num_rx_bdr = pf->caps.num_rx_bdr - pf->caps.num_vsi;
	else
		num_rx_bdr = ENETC_SI_MAX_RING_NUM;

	if (pf->caps.num_tx_bdr < ENETC_SI_MAX_RING_NUM + pf->caps.num_vsi)
		num_tx_bdr = pf->caps.num_tx_bdr - pf->caps.num_vsi;
	else
		num_tx_bdr = ENETC_SI_MAX_RING_NUM;

	val = enetc4_psicfgr0_val_construct(false, num_tx_bdr, num_rx_bdr);
	enetc_port_wr(hw, ENETC4_PSICFGR0(0), val);

	num_rx_bdr = pf->caps.num_rx_bdr - num_rx_bdr;
	rx_rem = num_rx_bdr % pf->caps.num_vsi;
	num_rx_bdr = num_rx_bdr / pf->caps.num_vsi;

	num_tx_bdr = pf->caps.num_tx_bdr - num_tx_bdr;
	tx_rem = num_tx_bdr % pf->caps.num_vsi;
	num_tx_bdr = num_tx_bdr / pf->caps.num_vsi;

	for (i = 0; i < pf->caps.num_vsi; i++) {
		vf_tx_bdr = (i < tx_rem) ? num_tx_bdr + 1 : num_tx_bdr;
		vf_rx_bdr = (i < rx_rem) ? num_rx_bdr + 1 : num_rx_bdr;
		val = enetc4_psicfgr0_val_construct(true, vf_tx_bdr, vf_rx_bdr);
		enetc_port_wr(hw, ENETC4_PSICFGR0(i + 1), val);
	}
}

static void enetc4_allocate_si_rings(struct enetc_pf *pf)
{
	enetc4_default_rings_allocation(pf);
}

static void enetc4_pf_set_si_vlan_promisc(struct enetc_hw *hw, int si, bool en)
{
	u32 val = enetc_port_rd(hw, ENETC4_PSIPVMR);

	if (en)
		val |= BIT(si);
	else
		val &= ~BIT(si);

	enetc_port_wr(hw, ENETC4_PSIPVMR, val);
}

static void enetc4_set_default_si_vlan_promisc(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	int num_si = pf->caps.num_vsi + 1;
	int i;

	/* enforce VLAN promiscuous mode for all SIs */
	for (i = 0; i < num_si; i++)
		enetc4_pf_set_si_vlan_promisc(hw, i, true);
}

/* Allocate the number of MSI-X vectors for per SI. */
static void enetc4_set_si_msix_num(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	int i, num_msix, total_si;
	u32 val;

	total_si = pf->caps.num_vsi + 1;

	num_msix = pf->caps.num_msix / total_si +
		   pf->caps.num_msix % total_si - 1;
	val = num_msix & PSICFGR2_NUM_MSIX;
	enetc_port_wr(hw, ENETC4_PSICFGR2(0), val);

	num_msix = pf->caps.num_msix / total_si - 1;
	val = num_msix & PSICFGR2_NUM_MSIX;
	for (i = 0; i < pf->caps.num_vsi; i++)
		enetc_port_wr(hw, ENETC4_PSICFGR2(i + 1), val);
}

static void enetc4_enable_all_si(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	int num_si = pf->caps.num_vsi + 1;
	u32 si_bitmap = 0;
	int i;

	/* Master enable for all SIs */
	for (i = 0; i < num_si; i++)
		si_bitmap |= PMR_SI_EN(i);

	enetc_port_wr(hw, ENETC4_PMR, si_bitmap);
}

static void enetc4_configure_port_si(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;

	enetc4_allocate_si_rings(pf);

	/* Outer VLAN tag will be used for VLAN filtering */
	enetc_port_wr(hw, ENETC4_PSIVLANFMR, PSIVLANFMR_VS);

	enetc4_set_default_si_vlan_promisc(pf);

	/* Disable SI MAC multicast & unicast promiscuous */
	enetc_port_wr(hw, ENETC4_PSIPMMR, 0);

	enetc4_set_si_msix_num(pf);

	enetc4_enable_all_si(pf);
}

static void enetc4_set_trx_frame_size(struct enetc_pf *pf)
{
	struct enetc_si *si = pf->si;

	enetc_port_mac_wr(si, ENETC4_PM_MAXFRM(0),
			  ENETC_SET_MAXFRM(ENETC_MAC_MAXFRM_SIZE));

	enetc4_pf_reset_tc_msdu(&si->hw);
}

static void enetc4_enable_trx(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;

	/* Enable port transmit/receive */
	enetc_port_wr(hw, ENETC4_POR, 0);
}

static void enetc4_set_isit_key_profile(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	u32 val;

	/* Key construction rule 0: SMAC + VID */
	val = ISIDKCCR0_VALID | ISIDKCCR0_SMACP | ISIDKCCR0_OVIDP;
	enetc_port_wr(hw, ENETC4_ISIDKC0CR0, val);

	/* Key construction rule 1: DMAC + VID */
	val = ISIDKCCR0_VALID | ISIDKCCR0_DMACP | ISIDKCCR0_OVIDP;
	enetc_port_wr(hw, ENETC4_ISIDKC1CR0, val);

	/* Enable key construction rule 0 and 1 */
	val = enetc_port_rd(hw, ENETC4_PISIDCR);
	val |= PISIDCR_KC0EN | PISIDCR_KC1EN;
	enetc_port_wr(hw, ENETC4_PISIDCR, val);
}

static void enetc4_configure_port(struct enetc_pf *pf)
{
	enetc4_configure_port_si(pf);
	enetc4_set_trx_frame_size(pf);
	enetc_set_default_rss_key(pf);
	enetc4_set_isit_key_profile(pf);
	enetc4_enable_trx(pf);
}

static u64 enetc4_get_current_time(struct enetc_si *si)
{
	u32 time_l, time_h;
	u64 current_time;

	time_l = enetc_rd_hot(&si->hw, ENETC_SICTR0);
	time_h = enetc_rd_hot(&si->hw, ENETC_SICTR1);
	current_time = (u64)time_h << 32 | time_l;

	return current_time;
}

static u64 enetc4_adjust_base_time(struct ntmp_user *user, u64 base_time,
				   u32 cycle_time)
{
	struct enetc_si *si = ntmp_user_to_enetc_si(user);
	u64 current_time, delta, n;

	current_time = enetc4_get_current_time(si);
	if (base_time >= current_time)
		return base_time;

	delta = current_time - base_time;
	n = DIV_ROUND_UP_ULL(delta, cycle_time);
	base_time += (n * (u64)cycle_time);

	return base_time;
}

static u32 enetc4_get_tgst_free_words(struct ntmp_user *user)
{
	struct enetc_si *si = ntmp_user_to_enetc_si(user);
	struct enetc_hw *hw = &si->hw;
	u32 words_in_use;
	u32 total_words;

	/* Notice that the admin gate list should be delete first before call
	 * this function, so the ENETC4_PTGAGLLR[ADMIN_GATE_LIST_LENGTH] equal
	 * to zero. That is, the ENETC4_TGSTMOR only contains the words of the
	 * operational gate control list.
	 */
	words_in_use = enetc_port_rd(hw, ENETC4_TGSTMOR) & TGSTMOR_NUM_WORDS;
	total_words = enetc_port_rd(hw, ENETC4_TGSTCAPR) & TGSTCAPR_NUM_WORDS;

	return total_words - words_in_use;
}

static const struct ntmp_ops ntmp_ops = {
	.adjust_base_time = enetc4_adjust_base_time,
	.get_tgst_free_words = enetc4_get_tgst_free_words,
};

static void enetc4_get_ntmp_caps(struct enetc_si *si)
{
	struct ntmp_caps *caps = &si->ntmp_user.caps;
	struct enetc_hw *hw = &si->hw;
	u32 val;

	/* Get the max number of entris of RP table */
	val = enetc_port_rd(hw, ENETC4_RPITCAPR);
	caps->rpt_num_entries = val & RPITCAPR_NUM_ENTRIES;

	/* Get the max number of entris of IS table */
	val = enetc_port_rd(hw, ENETC4_ISITCAPR);
	caps->ist_num_entries = val & ISITCAPR_NUM_ENTRIES;

	/* Get the max number of entris of SGI table */
	val = enetc_port_rd(hw, ENETC4_SGIITCAPR);
	caps->sgit_num_entries = val & SGITCAPR_NUM_ENTRIES;

	/* Get the max number of entris of ISC table */
	val = enetc_port_rd(hw, ENETC4_ISCICAPR);
	caps->isct_num_entries = val & ISCICAPR_NUM_ENTRIES;

	/* Get the max number of words of SGCL table */
	val = enetc_port_rd(hw, ENETC4_SGCLITCAPR);
	caps->sgclt_num_words = val & SGCLITCAPR_NUM_WORDS;
}

static int enetc4_ntmp_bitmap_init(struct ntmp_user *user)
{
	struct ntmp_caps *caps = &user->caps;

	user->ist_eid_bitmap = bitmap_zalloc(caps->ist_num_entries, GFP_KERNEL);
	if (!user->ist_eid_bitmap)
		return -ENOMEM;

	user->sgit_eid_bitmap = bitmap_zalloc(caps->sgit_num_entries, GFP_KERNEL);
	if (!user->sgit_eid_bitmap)
		goto free_ist_bitmap;

	user->sgclt_word_bitmap = bitmap_zalloc(caps->sgclt_num_words, GFP_KERNEL);
	if (!user->sgclt_word_bitmap)
		goto free_sgit_bitmap;

	user->isct_eid_bitmap = bitmap_zalloc(caps->isct_num_entries, GFP_KERNEL);
	if (!user->isct_eid_bitmap)
		goto free_sgclt_bitmap;

	user->rpt_eid_bitmap = bitmap_zalloc(caps->rpt_num_entries, GFP_KERNEL);
	if (!user->rpt_eid_bitmap)
		goto free_isct_bitmap;

	return 0;

free_isct_bitmap:
	bitmap_free(user->isct_eid_bitmap);
	user->isct_eid_bitmap = NULL;

free_sgclt_bitmap:
	bitmap_free(user->sgclt_word_bitmap);
	user->sgclt_word_bitmap = NULL;

free_sgit_bitmap:
	bitmap_free(user->sgit_eid_bitmap);
	user->sgit_eid_bitmap = NULL;

free_ist_bitmap:
	bitmap_free(user->ist_eid_bitmap);
	user->ist_eid_bitmap = NULL;

	return -ENOMEM;
}

static void enetc4_ntmp_bitmap_free(struct ntmp_user *user)
{
	bitmap_free(user->rpt_eid_bitmap);
	user->rpt_eid_bitmap = NULL;

	bitmap_free(user->isct_eid_bitmap);
	user->isct_eid_bitmap = NULL;

	bitmap_free(user->sgclt_word_bitmap);
	user->sgclt_word_bitmap = NULL;

	bitmap_free(user->sgit_eid_bitmap);
	user->sgit_eid_bitmap = NULL;

	bitmap_free(user->ist_eid_bitmap);
	user->ist_eid_bitmap = NULL;
}

static int enetc4_init_ntmp_user(struct enetc_si *si)
{
	struct ntmp_user *user = &si->ntmp_user;
	int err;

	user->ops = &ntmp_ops;
	user->dev_type = NETC_DEV_ENETC;
	if (si->revision == ENETC_REV_4_1)
		user->errata = NTMP_ERR052134;

	/* For ENETC 4.1, all table versions are 0 */
	memset(&user->tbl, 0, sizeof(user->tbl));

	err = enetc4_setup_cbdr(si);
	if (err)
		return err;

	enetc4_get_ntmp_caps(si);
	err = enetc4_ntmp_bitmap_init(user);
	if (err)
		goto teardown_cbdr;

	INIT_HLIST_HEAD(&user->flower_list);
	mutex_init(&user->flower_lock);

	return 0;

teardown_cbdr:
	enetc4_teardown_cbdr(si);

	return err;
}

static void enetc4_free_ntmp_user(struct enetc_si *si)
{
	struct ntmp_user *user = &si->ntmp_user;

	enetc4_clear_flower_list(si);
	mutex_destroy(&user->flower_lock);
	enetc4_ntmp_bitmap_free(user);
	enetc4_teardown_cbdr(si);
}

static int enetc4_pf_init(struct enetc_pf *pf)
{
	struct device *dev = &pf->si->pdev->dev;
	int err;

	/* Initialize the MAC address for PF and VFs */
	err = enetc_setup_mac_addresses(dev->of_node, pf);
	if (err) {
		dev_err(dev, "Failed to set MAC addresses\n");
		return err;
	}

	err = enetc4_init_ntmp_user(pf->si);
	if (err) {
		dev_err(dev, "Failed to init CBDR\n");
		return err;
	}

	enetc4_configure_port(pf);

	return 0;
}

static void enetc4_pf_free(struct enetc_pf *pf)
{
	enetc4_free_ntmp_user(pf->si);
}

static void enetc4_psi_do_set_rx_mode(struct work_struct *work)
{
	struct enetc_si *si = container_of(work, struct enetc_si, rx_mode_task);
	struct enetc_pf *pf = enetc_si_priv(si);
	struct net_device *ndev = si->ndev;
	struct enetc_hw *hw = &si->hw;
	bool uc_promisc = false;
	bool mc_promisc = false;
	int type = 0;

	rtnl_lock();

	if (ndev->flags & IFF_PROMISC) {
		uc_promisc = true;
		mc_promisc = true;
	} else if (ndev->flags & IFF_ALLMULTI) {
		mc_promisc = true;
		type = ENETC_MAC_FILTER_TYPE_UC;
	} else {
		type = ENETC_MAC_FILTER_TYPE_ALL;
	}

	enetc4_pf_set_si_mac_promisc(hw, 0, uc_promisc, mc_promisc);

	if (uc_promisc) {
		enetc4_pf_set_si_uc_hash_filter(hw, 0, 0);
		enetc4_pf_clear_maft_entries(pf);
	}

	if (mc_promisc)
		enetc4_pf_set_si_mc_hash_filter(hw, 0, 0);

	/* Set new MAC filter */
	enetc4_pf_set_mac_filter(pf, type);

	rtnl_unlock();
}

static void enetc4_pf_set_rx_mode(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;

	queue_work(si->workqueue, &si->rx_mode_task);
}

static int enetc4_pf_set_features(struct net_device *ndev,
				  netdev_features_t features)
{
	netdev_features_t changed = ndev->features ^ features;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;

	if (changed & NETIF_F_HW_VLAN_CTAG_FILTER) {
		bool promisc_en = !(features & NETIF_F_HW_VLAN_CTAG_FILTER);

		enetc4_pf_set_si_vlan_promisc(hw, 0, promisc_en);
	}

	if (changed & NETIF_F_LOOPBACK)
		enetc4_pf_set_loopback(ndev, !!(features & NETIF_F_LOOPBACK));

	enetc_set_features(ndev, features);

	return 0;
}

static int enetc4_pf_set_vf_vlan(struct net_device *ndev, int vf,
				 u16 vlan, u8 qos, __be16 proto)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_hw *hw = &priv->si->hw;
	u32 val = vlan & PSIVLANR_VID;

	if (vf >= pf->total_vfs || vlan >= VLAN_N_VID || qos > 7)
		return -EINVAL;

	if (proto != htons(ETH_P_8021Q))
		/* only C-tags supported for now */
		return -EPROTONOSUPPORT;

	if (val) {
		val = u32_replace_bits(val, qos, PSIVLANR_PCP);
		val |= PSIVLANR_E;
	}

	enetc_port_wr(hw, ENETC4_PSIVLANR(vf + 1), val);

	return 0;
}

static int enetc4_pf_set_vf_spoofchk(struct net_device *ndev, int vf, bool en)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_hw *hw = &priv->si->hw;
	u32 val;

	if (vf >= pf->total_vfs)
		return -EINVAL;

	val = enetc_port_rd(hw, ENETC4_PSICFGR0(vf + 1));
	val = u32_replace_bits(val, en ? 1 : 0, PSICFGR0_ASE);
	enetc_port_wr(hw, ENETC4_PSICFGR0(vf + 1), val);

	return 0;
}

static bool enetc4_pf_get_vf_spoofchk(struct enetc_hw *hw, int vf)
{
	u32 val = enetc_port_rd(hw, ENETC4_PSICFGR0(vf + 1));

	return !!(val & PSICFGR0_ASE);
}

static void enetc4_pf_get_si_based_vlan(struct enetc_hw *hw, int si,
					u32 *vid, u32 *pcp)
{
	u32 val = enetc_port_rd(hw, ENETC4_PSIVLANR(si));

	*vid = val & PSIVLANR_VID;
	*pcp = FIELD_GET(PSIVLANR_PCP, val);
}

static int enetc4_pf_get_vf_config(struct net_device *ndev, int vf,
				   struct ifla_vf_info *ivi)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_hw *hw = &pf->si->hw;
	struct enetc_vf_state *vf_state;

	if (vf >= pf->num_vfs)
		return -EINVAL;

	vf_state = &pf->vf_state[vf];
	ivi->vf = vf;
	enetc4_pf_get_si_based_vlan(hw, vf + 1, &ivi->vlan, &ivi->qos);
	enetc4_pf_get_si_primary_mac(hw, vf + 1, ivi->mac);
	ivi->spoofchk = enetc4_pf_get_vf_spoofchk(hw, vf);
	ivi->trusted = !!(vf_state->flags & ENETC_VF_FLAG_TRUSTED);

	return 0;
}

static const struct net_device_ops enetc4_ndev_ops = {
	.ndo_open		= enetc_open,
	.ndo_stop		= enetc_close,
	.ndo_start_xmit		= enetc_xmit,
	.ndo_get_stats		= enetc_get_stats,
	.ndo_set_mac_address	= enetc_pf_set_mac_addr,
	.ndo_set_rx_mode	= enetc4_pf_set_rx_mode,
	.ndo_set_features	= enetc4_pf_set_features,
	.ndo_vlan_rx_add_vid	= enetc_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= enetc_vlan_rx_del_vid,
	.ndo_eth_ioctl		= enetc_ioctl,
	.ndo_hwtstamp_get	= enetc_hwtstamp_get,
	.ndo_hwtstamp_set	= enetc_hwtstamp_set,
	.ndo_setup_tc		= enetc4_pf_setup_tc,
	.ndo_set_vf_trust	= enetc_pf_set_vf_trust,
	.ndo_set_vf_mac		= enetc_pf_set_vf_mac,
	.ndo_set_vf_vlan	= enetc4_pf_set_vf_vlan,
	.ndo_set_vf_spoofchk	= enetc4_pf_set_vf_spoofchk,
	.ndo_get_vf_config	= enetc4_pf_get_vf_config,
};

static struct phylink_pcs *
enetc4_pl_mac_select_pcs(struct phylink_config *config, phy_interface_t iface)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);

	return pf->pcs;
}

static void enetc4_mac_config(struct enetc_pf *pf, unsigned int mode,
			      phy_interface_t phy_mode)
{
	struct enetc_ndev_priv *priv = netdev_priv(pf->si->ndev);
	struct enetc_si *si = pf->si;
	u32 val;

	val = enetc_port_mac_rd(si, ENETC4_PM_IF_MODE(0));
	val &= ~(PM_IF_MODE_IFMODE | PM_IF_MODE_ENA);

	switch (phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val |= IFMODE_RGMII;
		/* We need to enable auto-negotiation for the MAC
		 * if its RGMII interface support In-Band status.
		 */
		if (phylink_autoneg_inband(mode))
			val |= PM_IF_MODE_ENA;
		break;
	case PHY_INTERFACE_MODE_RMII:
		val |= IFMODE_RMII;
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		val |= IFMODE_SGMII;
		break;
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_XGMII:
	case PHY_INTERFACE_MODE_USXGMII:
		val |= IFMODE_XGMII;
		break;
	default:
		dev_err(priv->dev,
			"Unsupported PHY mode:%d\n", phy_mode);
		return;
	}

	enetc_port_mac_wr(si, ENETC4_PM_IF_MODE(0), val);
}

static void enetc4_pl_mac_config(struct phylink_config *config, unsigned int mode,
				 const struct phylink_link_state *state)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);

	enetc4_mac_config(pf, mode, state->interface);
}

static void enetc4_set_port_speed(struct enetc_ndev_priv *priv, int speed)
{
	u32 old_speed = priv->speed;
	u32 val;

	if (speed == old_speed)
		return;

	val = enetc_port_rd(&priv->si->hw, ENETC4_PCR);
	val &= ~PCR_PSPEED;

	switch (speed) {
	case SPEED_100:
	case SPEED_1000:
	case SPEED_2500:
	case SPEED_10000:
		val |= (PCR_PSPEED & PCR_PSPEED_VAL(speed));
		break;
	case SPEED_10:
	default:
		val |= (PCR_PSPEED & PCR_PSPEED_VAL(SPEED_10));
	}

	priv->speed = speed;
	enetc_port_wr(&priv->si->hw, ENETC4_PCR, val);
}

static void enetc4_set_rgmii_mac(struct enetc_pf *pf, int speed, int duplex)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_IF_MODE(0));
	val = old_val & ~(PM_IF_MODE_ENA | PM_IF_MODE_M10 | PM_IF_MODE_REVMII);

	switch (speed) {
	case SPEED_1000:
		val = u32_replace_bits(val, SSP_1G, PM_IF_MODE_SSP);
		break;
	case SPEED_100:
		val = u32_replace_bits(val, SSP_100M, PM_IF_MODE_SSP);
		break;
	case SPEED_10:
		val = u32_replace_bits(val, SSP_10M, PM_IF_MODE_SSP);
	}

	val = u32_replace_bits(val, duplex == DUPLEX_FULL ? 0 : 1,
			       PM_IF_MODE_HD);

	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_IF_MODE(0), val);
}

static void enetc4_set_rmii_mac(struct enetc_pf *pf, int speed, int duplex)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_IF_MODE(0));
	val = old_val & ~(PM_IF_MODE_ENA | PM_IF_MODE_SSP);

	switch (speed) {
	case SPEED_100:
		val &= ~PM_IF_MODE_M10;
		break;
	case SPEED_10:
		val |= PM_IF_MODE_M10;
	}

	val = u32_replace_bits(val, duplex == DUPLEX_FULL ? 0 : 1,
			       PM_IF_MODE_HD);

	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_IF_MODE(0), val);
}

static void enetc4_set_hd_flow_control(struct enetc_pf *pf, bool enable)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	if (!pf->caps.half_duplex)
		return;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	val = u32_replace_bits(old_val, enable ? 1 : 0, PM_CMD_CFG_HD_FCEN);
	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_set_rx_pause(struct enetc_pf *pf, bool rx_pause)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	val = u32_replace_bits(old_val, rx_pause ? 0 : 1, PM_CMD_CFG_PAUSE_IGN);
	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_set_tx_pause(struct enetc_pf *pf, int num_rxbdr, bool tx_pause)
{
	u32 pause_off_thresh = 0, pause_on_thresh = 0;
	u32 init_quanta = 0, refresh_quanta = 0;
	struct enetc_hw *hw = &pf->si->hw;
	u32 rbmr, old_rbmr;
	int i;

	for (i = 0; i < num_rxbdr; i++) {
		old_rbmr = enetc_rxbdr_rd(hw, i, ENETC_RBMR);
		rbmr = u32_replace_bits(old_rbmr, tx_pause ? 1 : 0, ENETC_RBMR_CM);
		if (rbmr == old_rbmr)
			continue;

		enetc_rxbdr_wr(hw, i, ENETC_RBMR, rbmr);
	}

	if (tx_pause) {
		/* When the port first enters congestion, send a PAUSE request
		 * with the maximum number of quanta. When the port exits
		 * congestion, it will automatically send a PAUSE frame with
		 * zero quanta.
		 */
		init_quanta = 0xffff;

		/* Also, set up the refresh timer to send follow-up PAUSE
		 * frames at half the quanta value, in case the congestion
		 * condition persists.
		 */
		refresh_quanta = 0xffff / 2;

		/* Start emitting PAUSE frames when 3 large frames (or more
		 * smaller frames) have accumulated in the FIFO waiting to be
		 * DMAed to the RX ring.
		 */
		pause_on_thresh = 3 * ENETC_MAC_MAXFRM_SIZE;
		pause_off_thresh = 1 * ENETC_MAC_MAXFRM_SIZE;
	}

	enetc_port_mac_wr(pf->si, ENETC4_PM_PAUSE_QUANTA(0), init_quanta);
	enetc_port_mac_wr(pf->si, ENETC4_PM_PAUSE_THRESH(0), refresh_quanta);
	enetc_port_wr(hw, ENETC4_PPAUONTR, pause_on_thresh);
	enetc_port_wr(hw, ENETC4_PPAUOFFTR, pause_off_thresh);
}

static void enetc4_enable_mac(struct enetc_pf *pf, bool en)
{
	struct enetc_si *si = pf->si;
	u32 val;

	val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	val &= ~(PM_CMD_CFG_TX_EN | PM_CMD_CFG_RX_EN);
	val |= en ? (PM_CMD_CFG_TX_EN | PM_CMD_CFG_RX_EN) : 0;

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_mm_link_state_update(struct enetc_pf *pf,
					bool link)
{
	struct enetc_ndev_priv *priv = netdev_priv(pf->si->ndev);
	struct enetc_hw *hw = &pf->si->hw;
	u32 val;

	if (!(pf->si->hw_features & ENETC_SI_F_QBU))
		return;

	mutex_lock(&priv->mm_lock);

	val = enetc_port_rd(hw, ENETC4_MMCSR);

	if (link) {
		val &= ~MMCSR_LINK_FAIL;
		if (priv->active_offloads & ENETC_F_QBU)
			val = u32_replace_bits(val, MMCSR_ME_4B_BOUNDARY,
					       MMCSR_ME);
	} else {
		val |= MMCSR_LINK_FAIL;
		if (priv->active_offloads & ENETC_F_QBU)
			val = u32_replace_bits(val, MMCSR_ME_DISABLE,
					       MMCSR_ME);
	}

	enetc_port_wr(hw, ENETC4_MMCSR, val);

	enetc_mm_commit_preemptible_tcs(priv);

	mutex_unlock(&priv->mm_lock);
}

static void enetc4_pl_mac_link_up(struct phylink_config *config,
				  struct phy_device *phy, unsigned int mode,
				  phy_interface_t interface, int speed,
				  int duplex, bool tx_pause, bool rx_pause)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);
	struct enetc_si *si = pf->si;
	struct enetc_ndev_priv *priv;
	bool hd_fc = false;

	priv = netdev_priv(si->ndev);
	enetc4_set_port_speed(priv, speed);

	if (!phylink_autoneg_inband(mode) &&
	    phy_interface_mode_is_rgmii(interface))
		enetc4_set_rgmii_mac(pf, speed, duplex);

	if (interface == PHY_INTERFACE_MODE_RMII)
		enetc4_set_rmii_mac(pf, speed, duplex);

	if (duplex == DUPLEX_FULL) {
		/* When preemption is enabled, generation of PAUSE frames
		 * must be disabled, as stated in the IEEE 802.3 standard.
		 */
		if (priv->active_offloads & ENETC_F_QBU)
			tx_pause = false;
	} else { /* DUPLEX_HALF */
		if (tx_pause || rx_pause)
			hd_fc = true;

		/* As per 802.3 annex 31B, PAUSE frames are only supported
		 * when the link is configured for full duplex operation.
		 */
		tx_pause = false;
		rx_pause = false;
	}

	enetc4_set_hd_flow_control(pf, hd_fc);
	enetc4_set_tx_pause(pf, priv->num_rx_rings, tx_pause);
	enetc4_set_rx_pause(pf, rx_pause);
	enetc4_enable_mac(pf, true);
	enetc4_mm_link_state_update(pf, true);

	enetc_pf_send_link_status_msg(pf, true);
}

static void enetc4_pl_mac_link_down(struct phylink_config *config,
				    unsigned int mode,
				    phy_interface_t interface)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);

	enetc_pf_send_link_status_msg(pf, false);
	enetc4_mm_link_state_update(pf, false);
	enetc4_enable_mac(pf, false);
}

static const struct phylink_mac_ops enetc_pl_mac_ops = {
	.mac_select_pcs = enetc4_pl_mac_select_pcs,
	.mac_config = enetc4_pl_mac_config,
	.mac_link_up = enetc4_pl_mac_link_up,
	.mac_link_down = enetc4_pl_mac_link_down,
};

static void enetc4_pci_remove(void *data)
{
	struct pci_dev *pdev = data;

	enetc_pci_remove(pdev);
}

static int enetc4_link_init(struct enetc_ndev_priv *priv,
			    struct device_node *node)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct device *dev = priv->dev;
	int err;

	err = of_get_phy_mode(node, &pf->if_mode);
	if (err) {
		dev_err(dev, "Failed to get PHY mode\n");
		return err;
	}

	err = enetc_mdiobus_create(pf, node);
	if (err) {
		dev_err(dev, "Failed to create MDIO bus\n");
		return err;
	}

	err = enetc_phylink_create(priv, node, &enetc_pl_mac_ops);
	if (err) {
		dev_err(dev, "Failed to create phylink\n");
		goto err_phylink_create;
	}

	return 0;

err_phylink_create:
	enetc_mdiobus_destroy(pf);

	return err;
}

static void enetc4_link_deinit(struct enetc_ndev_priv *priv)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);

	enetc_phylink_destroy(priv);
	enetc_mdiobus_destroy(pf);
}

static int enetc4_psi_wq_task_init(struct enetc_si *si)
{
	char wq_name[24];

	INIT_WORK(&si->rx_mode_task, enetc4_psi_do_set_rx_mode);
	snprintf(wq_name, sizeof(wq_name), "enetc-%s", pci_name(si->pdev));
	si->workqueue = create_singlethread_workqueue(wq_name);
	if (!si->workqueue)
		return -ENOMEM;

	return 0;
}

static int enetc4_pf_netdev_create(struct enetc_si *si)
{
	struct device *dev = &si->pdev->dev;
	struct enetc_ndev_priv *priv;
	struct net_device *ndev;
	int err;

	ndev = alloc_etherdev_mqs(sizeof(struct enetc_ndev_priv),
				  si->num_tx_rings, si->num_rx_rings);
	if (!ndev)
		return  -ENOMEM;

	priv = netdev_priv(ndev);
	mutex_init(&priv->mm_lock);
	priv->ref_clk = devm_clk_get_optional(dev, "ref");
	if (IS_ERR(priv->ref_clk)) {
		dev_err(dev, "Get reference clock failed\n");
		err = PTR_ERR(priv->ref_clk);
		goto err_clk_get;
	}

	enetc_pf_netdev_setup(si, ndev, &enetc4_ndev_ops);

	enetc_init_si_rings_params(priv);

	err = enetc_configure_si(priv);
	if (err) {
		dev_err(dev, "Failed to configure SI\n");
		goto err_config_si;
	}

	err = enetc_alloc_msix(priv);
	if (err) {
		dev_err(dev, "Failed to alloc MSI-X\n");
		goto err_alloc_msix;
	}

	err = enetc4_link_init(priv, dev->of_node);
	if (err)
		goto err_link_init;

	err = enetc4_psi_wq_task_init(si);
	if (err) {
		dev_err(dev, "Failed to init workqueue\n");
		goto err_wq_init;
	}

	err = register_netdev(ndev);
	if (err) {
		dev_err(dev, "Failed to register netdev\n");
		goto err_reg_netdev;
	}

	return 0;

err_reg_netdev:
	destroy_workqueue(si->workqueue);
err_wq_init:
	enetc4_link_deinit(priv);
err_link_init:
	enetc_free_msix(priv);
err_alloc_msix:
err_config_si:
err_clk_get:
	mutex_destroy(&priv->mm_lock);
	free_netdev(ndev);

	return err;
}

static void enetc4_pf_netdev_destroy(struct enetc_si *si)
{
	struct enetc_ndev_priv *priv = netdev_priv(si->ndev);
	struct net_device *ndev = si->ndev;

	unregister_netdev(ndev);
	cancel_work(&si->rx_mode_task);
	destroy_workqueue(si->workqueue);
	enetc4_link_deinit(priv);
	enetc_free_msix(priv);
	mutex_destroy(&priv->mm_lock);
	free_netdev(ndev);
}

static const struct enetc_si_ops enetc4_psi_ops = {
	.get_rss_table = enetc4_get_rss_table,
	.set_rss_table = enetc4_set_rss_table,
};

static int enetc4_pf_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct enetc_si *si;
	struct enetc_pf *pf;
	int err;

	err = enetc_pci_probe(pdev, KBUILD_MODNAME, sizeof(*pf));
	if (err)
		return dev_err_probe(dev, err, "PCIe probing failed\n");

	err = devm_add_action_or_reset(dev, enetc4_pci_remove, pdev);
	if (err)
		return err;

	/* si is the private data. */
	si = pci_get_drvdata(pdev);
	if (!si->hw.port || !si->hw.global)
		return dev_err_probe(dev, -ENODEV,
				     "Couldn't map PF only space\n");

	si->revision = enetc_get_ip_revision(&si->hw);
	si->ops = &enetc4_psi_ops;
	err = enetc_get_driver_data(si);
	if (err)
		return dev_err_probe(dev, err,
				     "Could not get PF driver data\n");

	err = enetc4_pf_struct_init(si);
	if (err)
		return err;

	pf = enetc_si_priv(si);
	err = enetc4_pf_init(pf);
	if (err)
		goto err_pf_init;

	enetc_get_si_caps(si);

	err = enetc4_pf_netdev_create(si);
	if (err)
		goto err_netdev_create;

	enetc_create_debugfs(si);

	return 0;

err_netdev_create:
	enetc4_pf_free(pf);
err_pf_init:
	enetc4_pf_struct_free(pf);

	return err;
}

static void enetc4_pf_remove(struct pci_dev *pdev)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_pf *pf = enetc_si_priv(si);

	if (pf->num_vfs)
		enetc_sriov_configure(pdev, 0);

	enetc_remove_debugfs(si);
	enetc4_pf_netdev_destroy(si);
	enetc4_pf_free(pf);
	enetc4_pf_struct_free(pf);
}

static const struct pci_device_id enetc4_pf_id_table[] = {
	{ PCI_DEVICE(NXP_ENETC_VENDOR_ID, NXP_ENETC_PF_DEV_ID) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc4_pf_id_table);

static struct pci_driver enetc4_pf_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc4_pf_id_table,
	.probe = enetc4_pf_probe,
	.remove = enetc4_pf_remove,
#ifdef CONFIG_PCI_IOV
	.sriov_configure = enetc_sriov_configure,
#endif
};
module_pci_driver(enetc4_pf_driver);

MODULE_DESCRIPTION("ENETC4 PF Driver");
MODULE_LICENSE("Dual BSD/GPL");
