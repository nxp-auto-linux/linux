/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2024 NXP */
#ifndef __ENETC_DEVLINK_H
#define __ENETC_DEVLINK_H

#define ENETC_MAX_SI_NUM		16

struct enetc_devlink_priv {
	struct enetc_pf *pf;
	u8 si_num_rings[ENETC_MAX_SI_NUM];

	int (*pf_load)(struct enetc_pf *pf);
	int (*pf_unload)(struct enetc_pf *pf);
};

enum enetc_devlink_param_id {
	ENETC_DEVLINK_PARAM_ID_BASE = DEVLINK_PARAM_GENERIC_ID_MAX,
	ENETC_DEVLINK_PARAM_ID_RING_NUM_ASSIGN,
};

int enetc4_devlink_params_register(struct devlink *devlink);
void enetc4_devlink_params_unregister(struct devlink *devlink);
void enetc4_devlink_init_params(struct devlink *devlink);
int enetc4_devlink_alloc(struct enetc_pf *pf);

#endif /* __ENETC_DEVLINK_H__ */
