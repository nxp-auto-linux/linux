/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 * FlexLLCE (RCU) CAN host-interface definitions for NXP S32N79.
 *
 * The FlexLLCE firmware runs on the Cortex-R52 "RCU" cluster and exposes its
 * FlexCAN controllers to the Cortex-A host through a shared-memory + MRU
 * doorbell "Host Interface" (HIF).
 *
 * Copyright 2026 NXP
 */
#ifndef _FLEXLLCE_IF_H_
#define _FLEXLLCE_IF_H_

#include <linux/bits.h>
#include <linux/types.h>

#include "flexllce_fw_abi.h"

/* Number of notification categories (enum flexllce_notif_category). */
#define FLEXLLCE_NOTIF_CATEGORY_SIZE	3

/*
 * Enhanced Rx FIFO ID-filter element counts requested at INIT. The firmware
 * always enables an enhanced Rx FIFO on a controller it INITs and rejects the
 * command unless it is given a non-zero, even standard-ID count (the FIFO packs
 * two standard IDs per filter word). The driver requests a small even count for
 * both; the accept-all host receive filter is then installed separately with
 * SETFILTERTABLE.
 */
#define FLEXLLCE_INIT_STD_ID_FILTERS	8
#define FLEXLLCE_INIT_EXT_ID_FILTERS	4

/*
 * Per-controller FlexCAN message-buffer budget requested at INIT
 * (flexllce_init_cmd.max_mb_no). These firmware constants live in the firmware's
 * intercore_shared.h, which the generator does not parse, so they are mirrored
 * here rather than in the generated flexllce_fw_abi.h.
 */
/* Maximum usable message buffers per FlexCAN instance on S32N79. */
#define FLEXLLCE_FLEXCAN_MAX_MB_NUM		56
/* Message buffers reserved for the controller's internal CAN router. */
#define FLEXLLCE_FLEXCAN_MIN_ROUTE_MB		1
/* Host-TX ceiling: all message buffers except the router-reserved minimum. */
#define FLEXLLCE_MAX_HOST_TX_MB		\
	(FLEXLLCE_FLEXCAN_MAX_MB_NUM - FLEXLLCE_FLEXCAN_MIN_ROUTE_MB)

/* FlexCAN ESR1 fields carried in flexllce_notification.error_reg (bus errors). */
#define FLEXLLCE_ESR1_BIT1_ERR		BIT(15)
#define FLEXLLCE_ESR1_BIT0_ERR		BIT(14)
#define FLEXLLCE_ESR1_ACK_ERR		BIT(13)
#define FLEXLLCE_ESR1_CRC_ERR		BIT(12)
#define FLEXLLCE_ESR1_FRM_ERR		BIT(11)
#define FLEXLLCE_ESR1_STF_ERR		BIT(10)
/* FlexCAN ECR error counters (low half of the error-passive report). */
#define FLEXLLCE_ECR_TX_ERR(x)		((x) & 0xff)
#define FLEXLLCE_ECR_RX_ERR(x)		(((x) >> 8) & 0xff)

/*
 * Destination host RX communication channel selected in flexllce_set_filter_cmd
 * per-filter rx_interface. The firmware uses it directly to index
 * axRxComChannels[], bounded by FLEXLLCE_COM_CHANNEL_MAX_CNT (6); a wildcard
 * such as 0xFF fails the bounds check and the frame is silently dropped.
 * Channel 0 is the interrupt-driven RX channel the NAPI poll drains.
 */
#define FLEXLLCE_RX_COMCH_INTERRUPT	0

/* Frame payload size shared by the RX and TX buffers (CAN FD maximum). */
#define FLEXLLCE_MAX_PAYLOAD		64

/* FlexCAN message-buffer code/status (flexllce_rx_data.cs) frame-decode flags. */
#define FLEXLLCE_CS_RTR			BIT(20)
#define FLEXLLCE_CS_IDE			BIT(21)
#define FLEXLLCE_CS_ESI			BIT(29)
#define FLEXLLCE_CS_BRS			BIT(30)
#define FLEXLLCE_CS_EDL			BIT(31)	/* CAN FD frame */

#endif /* _FLEXLLCE_IF_H_ */
