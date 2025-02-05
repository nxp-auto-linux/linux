/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef __ENETC4_TC_H
#define __ENETC4_TC_H

int enetc4_pf_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		       void *type_data);
void enetc4_pf_reset_tc_msdu(struct enetc_hw *hw);
int enetc4_pf_to_port(struct enetc_si *si);
void enetc4_clear_flower_list(struct enetc_si *si);

#endif
