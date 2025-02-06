/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef __ENETC_MSG_H
#define __ENETC_MSG_H

#define ENETC_MSG_CODE_SUCCESS		0x100
#define ENETC_MSG_CODE_PERMISSION_DENY	0x200
#define ENETC_MSG_CODE_NOT_SUPPORT	0x300
#define ENETC_MSG_CODE_BUSY		0x400
#define ENETC_MSG_CODE_CRC_ERROR	0x500

#if IS_ENABLED(CONFIG_PCI_IOV)
int enetc_msg_psi_init(struct enetc_pf *pf);
void enetc_msg_psi_free(struct enetc_pf *pf);
#endif

#endif
