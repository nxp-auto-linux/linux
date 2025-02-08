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
int enetc_pf_send_msg(struct enetc_pf *pf, u32 msg_code, u16 ms_mask);
#endif

#endif
