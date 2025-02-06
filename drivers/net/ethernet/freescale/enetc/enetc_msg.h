/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef __ENETC_MSG_H
#define __ENETC_MSG_H

#if IS_ENABLED(CONFIG_PCI_IOV)
int enetc_msg_psi_init(struct enetc_pf *pf);
void enetc_msg_psi_free(struct enetc_pf *pf);
#endif

#endif
