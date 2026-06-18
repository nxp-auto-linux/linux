/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef __ENETC4_TC_H_
#define __ENETC4_TC_H_

int enetc4_pf_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		       void *type_data);

#endif
