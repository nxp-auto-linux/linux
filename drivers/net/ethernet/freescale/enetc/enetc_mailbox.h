/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef __ENETC_MAILBOX_H
#define __ENETC_MAILBOX_H

#include <linux/crc-itu-t.h>

#define ENETC_CRC_INIT				0xffff
#define ENETC_MSG_ALIGN				32

#define ENETC_MSG_EXT_BODY_LEN(l)		((l) / ENETC_MSG_ALIGN - 1)
#define ENETC_MSG_SIZE(l)			(((l) + 1) * ENETC_MSG_ALIGN)

/* Common Class ID for PSI-TO-VSI and VSI-TO-PSI messages */
#define ENETC_MSG_CLASS_ID_MAC_FILTER		0x20

/* Class ID for PSI-TO-VSI messages */
#define ENETC_MSG_CLASS_ID_CMD_SUCCESS		0x1
#define ENETC_MSG_CLASS_ID_PERMISSION_DENY	0x2
#define ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT	0x3
#define ENETC_MSG_CLASS_ID_PSI_BUSY		0x4
#define ENETC_MSG_CLASS_ID_CRC_ERROR		0x5
#define ENETC_MSG_CLASS_ID_PROTO_NOT_SUPPORT	0x6
#define ENETC_MSG_CLASS_ID_INVALID_MSG_LEN	0x7
#define ENETC_MSG_CLASS_ID_CMD_TIMEOUT		0x8
#define ENETC_MSG_CLASS_ID_CMD_DEFERED		0xf

/* Class-specific error return codes for MAC filter */
#define ENETC_PF_RC_MAC_FILTER_INVALID_MAC	0x0
#define ENETC_PF_RC_MAC_FILTER_DUPLICATE_MAC	0x1
#define ENETC_PF_RC_MAC_FILTER_MAC_NOT_FOUND	0x2
#define ENETC_PF_RC_MAC_FILTER_NO_RESOURCE	0x3

enum enetc_msg_mac_filter_cmd_id {
	ENETC_MSG_SET_PRIMARY_MAC,
};

struct enetc_msg_swbd {
	void *vaddr;
	dma_addr_t dma;
	int size;
};

/* The format of PSI-TO-VSI message, only a 16-bits code */
union enetc_pf_msg {
	struct {
		u8 cookie:4;
		u8 class_code:4;
		u8 class_id;
	};
	u16 code;
};

/* The formats of VSI-TO-PSI messages */
struct enetc_msg_header {
	__be16 crc16;
	u8 class_id;
	u8 cmd_id;
	u8 proto_ver;
	u8 len;
	u8 resv0;
	u8 cookie:4;
	u8 resv1:4;
	u8 resv2[8];
};

struct enetc_mac_addr {
	u8 addr[ETH_ALEN];
};

/* message format of class_id 0x20, cmd_id: 0x0
 * cmd_id 0: set primary MAC
 */
struct enetc_msg_mac_exact_filter {
	struct enetc_msg_header hdr;
	u8 mac_cnt;
	u8 resv[3];
	struct enetc_mac_addr mac[];
};

#endif
