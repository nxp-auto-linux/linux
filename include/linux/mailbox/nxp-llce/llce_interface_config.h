/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020 NXP */
#ifndef LLCE_INTERFACECONFIG_H
#define LLCE_INTERFACECONFIG_H

#include <linux/types.h>

/* LLCE cores startup ending information values */

/**
 * Define used to detect if the boot was finalized for a specific LLCE
 * internal core.
 */
#define LLCE_MGR_DTE_BOOT_END (0x0000000F)
/**
 * Define used to detect if the boot was finalized for a specific LLCE
 * internal core.
 */
#define LLCE_MGR_RX_BOOT_END (0x000000F0)
/**
 * Define used to detect if the boot was finalized for a specific LLCE
 * internal core.
 */
#define LLCE_MGR_TX_BOOT_END (0x00000F00)
/**
 * Define used to detect if the boot was finalized for a specific LLCE
 * internal core.
 */
#define LLCE_MGR_FRPE_BOOT_END (0x0000F000)
/**
 * Define used to detect if the boot was finalized for all LLCE
 * internal cores.
 */
#define LLCE_MGR_BOOT_END_ALL_CORES_MASK (0x0000FFFF)

/* LLCE configuration parameters. */

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

/** Maximum number of notifications which can be reported by LLCE to host. */
#define LLCE_CAN_CONFIG_NOTIF_TABLE_SIZE 17U
/**
 * Number fo ETH buffers used to interface with the host core for can2eth
 * use case.
 */
#define LLCE_CAN_CONFIG_MAX_ETH_FRAME 2U
/**
 * Maximum buffer size used to store the CAN FD frame payload.
 * See llce_can_mb_type
 */
#define LLCE_CAN_CONFIG_PAYLOAD_MAX_SIZE 64U
/**
 * Maximum number of hardware controllers usable inside LLCE.
 * See llce_can_init_cmd_type
 */
#define LLCE_CAN_CONFIG_MAXCTRL_COUNT 16U
/**
 * Maximum number of hardware controllers usable inside LLCE.
 * See llce_can_init_cmd_type
 */
#define LLCE_CAN_MAX_POLLING_CLASSES 6U

/**
 * Maximum number of transmission message buffers.
 * See llce_can_tx_mb_desc_type
 */
#define LLCE_CAN_CONFIG_MAXTXMB 256U
/**
 * Maximum number of reception message buffers.32 from those are
 * reserved for internal usage and are not available to the host.
 * See llce_can_rx_mb_desc_type
 */
#define LLCE_CAN_CONFIG_MAXRXMB 2048U
#define LLCE_CAN_CONFIG_MAXAFRXMB 256U
#define LLCE_CAN_CONFIG_MAXAFTXMB 256U
#define LLCE_CAN_CONFIG_MAXAFFRMB 256U

/**
 * Maximum number of standard filters which can be configured using a
 * single command.Multiple commands can be executed when more filters are
 * needed.
 * See llce_can_rx_filter_type
 */
#define LLCE_CAN_CONFIG_MAX_FILTERS_COUNT (20U)
/**
 * Number of entries of the circular buffer used to send ack
 * information from Tx core to host core. There is 1 extra buffer for each
 * interface for consistency purpose.
 */
#define LLCE_CAN_CONFIG_MAX_TXACKINFO ((u16)(512U + LLCE_CAN_RX_TX_INTERFACES))
/**
 * Mask used to get the right data from FIFOs. See FMR config register
 * of FIFO.
 */
#define LLCE_CAN_CONFIG_FIFO_FIXED_MASK (0x0007FFFF)
/**
 * Maximum number of advanced filters which can be configured using a
 * single command.Multiple commands can be executed when more filters are
 * needed.
 * See llce_can_advanced_filter_type
 */
#define LLCE_CAN_CONFIG_ADVANCED_FILTERS_COUNT 8U
/** Shared memory size allocated for each channel for commands exchange. */
#define LLCE_CAN_CONFIG_CTRL_SHARED_MEMORY_SIZE 0x400
/** Reserved value in order to detect if an advanced filter entry is not used */
#define LLCE_CAN_ADVANCED_FILTER_NOT_USED 0xFFU
/** Interface ID used by different hosts for multihost scenarios. */
#define LLCE_CAN_HIF0 0U
/** Interface ID used by different hosts for multihost scenarios. */
#define LLCE_CAN_HIF1 1U
/** Number of interfaces which can be used by host cores. */
#define LLCE_CAN_CONFIG_HIF_COUNT 2U
/**
 * Number of ocurrences of last error reported by the notification mechanism.
 * Limited by \link llce_can_error_notif_type \endlink structure size.
 */
#define LLCE_CAN_CONFIG_MAX_OCCURENCES 255U

/* LIN defines */

/** Maximum buffer size used to store the LIN frame payload */
#define LLCE_LIN_CONFIG_PAYLOAD_MAX_SIZE 8U
/** Maximum number of LIN transmission buffers */
#define LLCE_LIN_CONFIG_MAXTXBUFF 64U
/** Maximum number of LIN reception buffers */
#define LLCE_LIN_CONFIG_MAXRXBUFF 64U

/**
 * Boot sequence data type.
 *
 * Data type used to access shared memory area for managing LLCE boot sequence
 */
struct llce_mgr_status {
	/**
	 * Information used by any host application to detect the TXPE
	 * startup ending.
	 */
	u32 tx_boot_end;
	/**
	 * Information used by any host application to detect the RXPE
	 * startup ending.
	 */
	u32 rx_boot_end;
	/**
	 * Information used by any host application to detect the DTE
	 * startup ending.
	 */
	u32 dte_boot_end;
	/**
	 * Information used by any host application to detect the FRPE
	 * startup ending.
	 */
	u32 frpe_boot_end;
	/** Information used by host for the runtime error count about LLCE */
	u32 error_counter;
	/* Information used by host for the LLCE performance benchmark. */
	u32 stm_init_count;
} __aligned(4);

#endif /* LLCE_FIFO_H */
