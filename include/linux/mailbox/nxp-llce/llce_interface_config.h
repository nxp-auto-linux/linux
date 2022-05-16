/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020-2022 NXP */
#ifndef LLCE_INTERFACE_CONFIG_H
#define LLCE_INTERFACE_CONFIG_H

/* LLCE configuration parameters.
 */
/**
 * Default controller ID needed by the host 0 interface in order to
 * transmit INIT_PLATFORM and DEINIT_PLATFORM commands from host to LLCE.
 */
#define LLCE_CAN_CONFIG_DEFAULT_CAN_CTRL_HOST0_U8 0U
/**
 * Default controller ID needed by the host 1 interface in order to
 * transmit INIT_PLATFORM and DEINIT_PLATFORM commands from host to LLCE.
 */
#define LLCE_CAN_CONFIG_DEFAULT_CAN_CTRL_HOST1_U8 8U

/**
 * Maximum number of notifications which can be reported by LLCE
 * to host.
 */
#define LLCE_CAN_CONFIG_NOTIF_TABLE_SIZE 17U
/**
 * Maximum buffer size used to store the CAN FD frame payload.
 * See Llce_can_Mb_type
 */
#define LLCE_CAN_CONFIG_PAYLOAD_MAX_SIZE 64U
/**
 * Maximum buffer size used to store the short CAN FD frame payload.
 * See Llce_can_Short_mb_type
 */
#define LLCE_CAN_CONFIG_SHORT_PAYLOAD_MAX_SIZE 8U
/**
 * Maximum number of hardware controllers usable inside LLCE.
 * See Llce_can_Init_cmd_type
 */
#define LLCE_CAN_CONFIG_MAXCTRL_COUNT 16U
/**
 * Maximum number of polling classes usable inside LLCE.
 * A polling class is used to tie together multiple filters which
 * report the received frames in polling.
 * See Llce_can_Init_cmd_type
 */
#define LLCE_CAN_MAX_POLLING_CLASSES 6U

/**
 * Maximum number of transmission message buffers.
 * See Llce_can_Tx_mb_descriptor_type
 */
#define LLCE_CAN_CONFIG_MAXTXMB 256U
/**
 * Maximum number of 64B reception message buffers.
 * Note: 32 from those are reserved
 * for internal usage and are not available to the host.
 * See Llce_can_Rx_mb_descriptor_type
 */
#define LLCE_CAN_CONFIG_MAXRXMB 1732U

/**
 * Maximum number of 8B reception message buffers.
 * for internal usage and are not available to the host.
 * See Llce_can_Rx_mb_descriptor_type
 */
#define LLCE_CAN_CONFIG_MAX_SHORTRXMB 2396U

/**
 * Number of AF descriptors reserved for each
 * internal path within LLCE frame routing
*/
#define LLCE_CAN_CONFIG_MAXAFRXMB 256U
#define LLCE_CAN_CONFIG_MAXAFTXMB 256U
#define LLCE_CAN_CONFIG_MAXAFFRMB 256U

/**
 * Maximum number of standard filters which can be configured using
 * using a single command. Multiple commands can be executed when more filters
 * are needed.
 * See Llce_can_Receive_filter_type
 */
#define LLCE_CAN_CONFIG_MAX_FILTERS_COUNT (20U)
/**
 * Number of entries of the circular buffer used to send ack information
 * from TX core to host core. There is 1 extra buffer for each interface for
 * consistency purpose.
 */
#define LLCE_CAN_CONFIG_MAX_TXACKINFO ((u16)(512U + LLCE_CAN_RX_TX_INTERFACES))
/**
 * Mask used to get the right data from FIFOs.
 * See FMR config register of FIFO.
 */
#define LLCE_CAN_CONFIG_FIFO_FIXED_MASK (0x0007FFFF)
/**
 * Maximum number of advanced filters which can be configured using a
 * single command. Multiple commands can be executed when more filters are
 * needed.
 * See Llce_can_Advanced_filter_type
 */
#define LLCE_CAN_CONFIG_ADVANCED_FILTERS_COUNT 8U
/**
 * Shared memory size allocated for each channel for
 * commands exchange.
 */
#define LLCE_CAN_CONFIG_CTRL_SHARED_MEMORY_SIZE 0x400
/**
 * Reserved value in order to detect if an advanced filter entry
 * is not used.
 */
#define LLCE_CAN_ADVANCED_FILTER_NOT_USED 0xFFU
/** Interface ID used by different hosts for multihost scenarios. */
#define LLCE_CAN_HIF0 0U
/** Interface ID used by different hosts for multihost scenarios. */
#define LLCE_CAN_HIF1 1U
/** Number of interfaces which can be used by host cores. */
#define LLCE_CAN_CONFIG_HIF_COUNT 2U
/** Number of semaphores for each HIF. */
#define LLCE_CAN_CONFIG_IER_SEMA4_COUNT 2U
/** Index of FIFO_RXOUT inside Can_sema4_Ier array. */
#define LLCE_FIFO_RXOUT_INDEX 0U
/** Index of FIFO_TXACK inside Can_sema4_Ier array. */
#define LLCE_FIFO_TXACK_INDEX 1U
/** Number of FIFOs used by first HIF. */
#define LLCE_CAN_HIF0_FIFO_CNT 8U
/** Semaphore core domain */
#define LLCE_HOST_CORE_SEMA42_DOMAIN LLCE_SEMA42_GR_GTFSM_9
/** No result from Rx_lut2 */
#define LLCE_RXLUT2_NO_RESULT 0xFFFFU

/* LIN defines
 */
/** Maximum buffer size used to store the LIN frame payload */
#define LLCE_LIN_CONFIG_PAYLOAD_MAX_SIZE 8U
/** Maximum number of LIN transmission buffers */
#define LLCE_LIN_CONFIG_MAXTXBUFF 64U
/** Maximum number of LIN reception buffers */
#define LLCE_LIN_CONFIG_MAXRXBUFF 64U

#endif /* LLCE_FIFO_H */
