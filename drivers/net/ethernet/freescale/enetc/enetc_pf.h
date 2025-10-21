/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2017-2019 NXP */

#include "enetc.h"
#include <linux/phylink.h>
#include <net/devlink.h>

#define ENETC_PF_NUM_RINGS	8
#define ENETC_MAX_NUM_MAC_FLT	((ENETC_MAX_NUM_VFS + 1) * MADDR_TYPE)

#define ENETC_VLAN_HT_SIZE	64

enum enetc_vf_flags {
	ENETC_VF_FLAG_PF_SET_MAC	= BIT(0),
	ENETC_VF_FLAG_TRUSTED		= BIT(1)
};

struct enetc_vf_state {
	enum enetc_vf_flags flags;
};

struct enetc_port_caps {
	u32 half_duplex:1;
	u32 wol:1;
	int num_vsi;
	int num_msix;
	int num_rx_bdr;
	int num_tx_bdr;
	int mac_filter_num;
	int ipf_words_num;
};

struct enetc_mm {
	u8 pmac_enabled:1;
	u8 tx_enabled:1;
	u8 verify_enabled:1;
	u8 lafs;
	u8 rafs;
	u8 vsts;
	u8 vt;
};

struct enetc_pf;

struct enetc_pf_ops {
	void (*set_si_primary_mac)(struct enetc_hw *hw, int si, const u8 *addr);
	void (*get_si_primary_mac)(struct enetc_hw *hw, int si, u8 *addr);
	struct phylink_pcs *(*create_pcs)(struct enetc_pf *pf, struct mii_bus *bus);
	void (*destroy_pcs)(struct phylink_pcs *pcs);
	int (*enable_psfp)(struct enetc_ndev_priv *priv);
	void (*get_mm)(struct enetc_ndev_priv *priv, struct enetc_mm *mm);
	void (*set_mm)(struct enetc_ndev_priv *priv, struct ethtool_mm_cfg *cfg,
		       u32 add_frag_size);
	void (*set_preemptible_tcs)(struct enetc_ndev_priv *priv);
	void (*set_si_mac_promisc)(struct enetc_hw *hw, int si,
				   enum enetc_mac_addr_type type, bool en);
	void (*set_si_mac_hash_filter)(struct enetc_hw *hw, int si,
				       enum enetc_mac_addr_type type, u64 hash);
	void (*set_si_vlan_promisc)(struct enetc_hw *hw, int si, bool en);
	void (*set_si_vlan_hash_filter)(struct enetc_si *si, int si_id, u64 hash);
};

struct enetc_pf {
	struct enetc_si *si;
	int num_vfs; /* number of active VFs, after sriov_init */
	int total_vfs; /* max number of VFs, set for PF at probe */
	struct enetc_vf_state *vf_state;

	struct enetc_msg_swbd rxmsg[ENETC_MAX_NUM_VFS];
	bool vf_link_status_notify[ENETC_MAX_NUM_VFS];
	u8 mac_addr_base[ETH_ALEN];

	char vlan_promisc_simap; /* bitmap of SIs in VLAN promisc mode */

	struct mii_bus *mdio; /* saved for cleanup */
	struct mii_bus *imdio;
	struct phylink_pcs *pcs;

	phy_interface_t if_mode;
	struct phylink_config phylink_config;

	struct enetc_port_caps caps;
	const struct enetc_pf_ops *ops;

	int num_mfe;	/* number of mac address filter table entries */
	struct devlink *devlink;
};

#define phylink_to_enetc_pf(config) \
	container_of((config), struct enetc_pf, phylink_config)
