/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020-2024 NXP */
#ifndef LLCE_INTERFACE_LIN_H
#define LLCE_INTERFACE_LIN_H

/* LIN defines
 */
/** Maximum number of hardware LIN controllers usable inside LLCE */
#define LLCE_LIN_MAXCTRL_COUNT (4U)
/** Maximum buffer size used to store the LIN frame payload */
#define LLCE_LIN_CONFIG_PAYLOAD_MAX_SIZE (8U)

/**
 * Type of the LIN node.
 * Type of the LIN node. Master or Slave.
 **/
enum llce_lin_node {
	/** This is used for Master node */
	LLCE_LIN_MASTER_NODE = 0x0U,
	/** This is used for Slave node */
	LLCE_LIN_SLAVE_NODE = 0x1U
} __packed;

/**
 * Lin Slave error codes.
 * Lin Slave error codes.
 **/
enum llce_lin_slave_error {
	/** Error in header */
	LLCE_LIN_ERR_HEADER = 0,
	/** Framing error in response */
	LLCE_LIN_ERR_RESP_STOPBIT,
	/** Checksum error */
	LLCE_LIN_ERR_RESP_CHKSUM,
	/** Monitoring error of transmitted data bit in response */
	LLCE_LIN_ERR_RESP_DATABIT,
	/** No response */
	LLCE_LIN_ERR_NO_RESP,
	/** Incomplete response */
	LLCE_LIN_ERR_INC_RESP
} __packed;

/**
 * Return status codes reported at the end of each command execution.
 * Return status codes reported at the end of each command execution.
 * The meaning of these codes are command dependent.
 **/
enum llce_lin_return {
	/** Default state of the command. */
	LLCE_LIN_NOTRUN = 0x11,
	/** Command was executed successfully. */
	LLCE_LIN_SUCCESS = 0x55,
	/**
	 * During command execution it was detected an error
	 * condition.
	 */
	LLCE_LIN_ERROR = 0x56,
	/** Command was executed, but a timeout was present. */
	LLCE_LIN_ERROR_TIMEOUT = 0x57,
	/** Command was executed, but LIN channel is in wrong state. */
	LLCE_LIN_ERROR_STATE = 0x58,
	/**
	 * Command was executed, but LIN channel returned a
	 * FALSE response.
	 */
	LLCE_LIN_ERROR_FALSE = 0x59
} __packed;

/**
 * LIN command IDs used to interface with LLCE.
 * Some of these commands are sent by the host to LLCE module and others
 * are sent by LLCE module to the host.
 **/
enum llce_lin_command_id {
	/** Host initializes LIN channels inside LLCE. */
	LLCE_LIN_CMD_INIT = 0UL,
	/** Host sets LIN channel to sleep state. */
	LLCE_LIN_CMD_GOTOSLEEP,
	/** Host sets LIN channel to internal sleep state. */
	LLCE_LIN_CMD_GOTOSLEEPINTERNAL,
	/** Host sends wakeup command to LIN channel. */
	LLCE_LIN_CMD_WAKEUP,
	/** Host sends internal wakeup command to LIN channel. */
	LLCE_LIN_CMD_WAKEUPINTERNAL,
	/** Host requests the status of LIN channel. */
	LLCE_LIN_CMD_GETSTATUS,
	/** Host sends a frame to a LIN channel. */
	LLCE_LIN_CMD_SENDFRAME,
	/** LLCE notifies host about wakeup event. */
	LLCE_LIN_CMD_WAKEUP_CONFIRMATION,
	/**
	 * LLCE notifies host about Reception of a header.
	 * Only for Slave nodes.
	 */
	LLCE_LIN_CMD_HEADER_INDICATION,
	/**
	 * LLCE notifies host about an error occurred.
	 * Only for Slave nodes.
	 */
	LLCE_LIN_CMD_ERROR_INDICATION,
	/**
	 * LLCE notifies host about Reception event.
	 * Only for Slave nodes.
	 */
	LLCE_LIN_CMD_RX_INDICATION,
	/**
	 * LLCE notifies host about Transmission Event.
	 * Only for Slave nodes.
	 */
	LLCE_LIN_CMD_TX_CONFIRMATION,
	/** Enables interrupt forwarding for all LIN Channels. */
	LLCE_LIN_CMD_ENABLEINTRFORWARD
} __packed;

/**
 * Checksum models for the LIN Frame.
 * This type is used to specify the Checksum model to be used
 */
enum llce_lin_frame_cs_model {
	/** Enhanced checksum model.*/
	LLCE_LIN_ENHANCED_CS,
	/** Classic checksum model.*/
	LLCE_LIN_CLASSIC_CS
} __packed;

/**
 * Frame response types.
 * This type is used to specify whether the frame processor is required to
 * transmit the response part of the LIN frame.
 */
enum llce_lin_frame_response {
	/** Response is generated from this (master) node.*/
	LLCE_LIN_FRAMERESPONSE_TX = 0,
	/** Response is generated from a remote slave node.*/
	LLCE_LIN_FRAMERESPONSE_RX,
	/** Response is generated from one slave to another slave.*/
	LLCE_LIN_FRAMERESPONSE_IGNORE
	/**
	 * For the master the response will be anonymous, it does not
	 * have to receive the response.
	 */
} __packed;

/**
 * LIN Frame and Channel states operation.
 * LIN operation states for a LIN channel or frame
 */
enum llce_lin_status {
	/** Development or production error  occurred.*/
	LLCE_LIN_NOT_OK = 0,
	/** Successful transmission.*/
	LLCE_LIN_TX_OK,
	/** Ongoing transmission (Header or Response).*/
	LLCE_LIN_TX_BUSY,
	/**
	 * Erroneous header transmission such as:
	 * - Mismatch between sent and read back data
	 * - Identifier parity error
	 * - Physical bus error.
	 */
	LLCE_LIN_TX_HEADER_ERROR,
	/**
	 * Erroneous transmission such as:
	 * - Mismatch between sent and read back data
	 * - Physical bus error.
	 */
	LLCE_LIN_TX_ERROR,
	/** Reception of correct response.*/
	LLCE_LIN_RX_OK,
	/**
	 * Ongoing reception: at least one response byte has been
	 * received, but the checksum byte has not been received.
	 */
	LLCE_LIN_RX_BUSY,
	/**
	 * Erroneous reception such as:
	 * - Framing error
	 * - Overrun error
	 * - Checksum error
	 * - Short response.
	 */
	LLCE_LIN_RX_ERROR,
	/**
	 * No response byte has been received so far.
	 *
	 * Frame status is mixed with channel status but it's kept here only
	 * because of LIN168.
	 */
	LLCE_LIN_RX_NO_RESPONSE,
	/**
	 * Normal operation:
	 * - The related LIN channel is ready to transmit next header
	 * - No data from previous frame available
	 * (e.g. after initialization).
	 */
	LLCE_LIN_OPERATIONAL,
	/**
	 * Sleep mode operation:
	 * - In this mode wake-up detection from slave
	 * nodes is enabled.
	 */
	LLCE_LIN_CH_SLEEP,
} __packed;

/**
 * LIN channel configuration type structure.
 * This is the type of the external data structure containing the overall
 * initialization data for one LIN Channel.
 */
struct llce_lin_channel_config {
	/** LIN baud rate value.*/
	u32 lin_baud_rate_reg_value;
	/** Wakeup enable.*/
	u8 lin_channel_wakeup_support;
	/** Disable frame timeout. */
	u8 lin_channel_disable_frame_timeout;
	/** These bits indicate the Break length in Master mode.*/
	u8 lin_channel_brk_length_master;
	/** These bits indicate the Break length in Slave mode.*/
	u8 lin_channel_brk_length_slave;
	/** Response timeout value LINTOCR[RTO] in bit time.*/
	u8 response_timeout;
	/** Header timeout value LINTOCR[HTO].*/
	u8 header_timeout;
	/** LIN Node Type.*/
	enum llce_lin_node lin_node_type;
} __aligned(4) __packed;

/**
 * LIN initialization command.
 * It is sent by the host to LLCE in order to load and configure all
 * needed parameters for the LIN channels inside LLCE.
 **/
struct llce_lin_init_cmd {
	/**
	 * INPUT: Configuration options for all hardware LIN
	 * controllers.
	 */
	struct llce_lin_channel_config ctrl_config;
} __aligned(4) __packed;

/**
 * Get LIN status command.
 * It is sent by the host to LLCE in order to get the status of a LIN
 * controller.
 **/
struct llce_lin_get_status_cmd {
	/** OUTPUT: LIN channel state. */
	u8 lin_ch_status;
	/** OUTPUT: LIN frame state. */
	u8 lin_ch_frame_status;
	/** OUTPUT: LIN Frame error status. */
	u8 lin_ch_frame_error_status;
	/** OUTPUT: LIN header command type */
	u8 lin_transmit_header_command;
	/** OUTPUT: LIN controller status */
	enum llce_lin_status controller_status;
} __aligned(4) __packed;

/**
 * LIN sleep/wakeup management commands.
 * It is sent by the host to LLCE in order to manage sleep/wakeup
 * for a specific channel.
 **/
struct llce_lin_wakeup_cmd {
	/** INPUT: Wake-up support enabled. */
	u8 lin_channel_wakeup_support;
	/** OUTPUT: LIN channel state. */
	u8 lin_ch_status;
} __aligned(4) __packed;

/**
 * LIN send frame commands.
 * It is sent by the host to LLCE in order to send a frame
 * on a specific channel. This Type is used to provide PID,
 * checksum model, data length and SDU pointer from the LIN Driver
 * to the LIN controllers inside LLCE.
 **/
struct llce_lin_send_frame_cmd {
	/** Pointer to sdu. */
	u8 sdu[LLCE_LIN_CONFIG_PAYLOAD_MAX_SIZE];
	/** LIN frame identifier. */
	u8 pid;
	/** Data length. */
	u8 dl;
	/** Checksum model type. */
	enum llce_lin_frame_cs_model cs;
	/** Response type. */
	enum llce_lin_frame_response drc;
} __aligned(4) __packed;

struct llce_lin_header_indication_cmd {
	/** LIN frame identifier. */
	u8 pid;
	/** Data length. */
	u8 dl;
	/** Checksum model type. */
	enum llce_lin_frame_cs_model cs;
	/** Response type. */
	enum llce_lin_frame_response drc;
} __aligned(4) __packed;

struct llce_lin_error_indication_cmd {
	/** Response type. */
	enum llce_lin_slave_error slave_error;
} __aligned(4) __packed;

/**
 * List of commands used by host.
 * It is used in order to use the same memory area for all commands send
 * from host to LLCE.
 **/
union llce_lin_command_list {
	/** Init Command. */
	struct llce_lin_init_cmd init;
	/** Get Status Command. */
	struct llce_lin_get_status_cmd get_status;
	/** Wakeup Command. */
	struct llce_lin_wakeup_cmd wakeup;
	/** Send Frame Command. */
	struct llce_lin_send_frame_cmd send_frame;
	/** Header Indication Notification. */
	struct llce_lin_header_indication_cmd header_indication;
	/** Error Notification. */
	struct llce_lin_error_indication_cmd error_indication;
};

/**
 * Command used by host.
 * It is used in order to send commands from host to LLCE using shared memory.
 **/
struct llce_lin_command {
	/** INPUT/OUTPUT: Command parameters. */
	union llce_lin_command_list cmd_list;
	/** INPUT: Command ID. */
	enum llce_lin_command_id cmd_id;
	/** OUTPUT: Return status code after command completion.*/
	enum llce_lin_return return_value;
} __aligned(4) __packed;

/**
 * Shared memory structure
 * Structure that encapsulates all the shared memory with LlCE on the LIN side.
 **/
struct llce_lin_shared_memory {
	/**
	 * Array used to exchange commands between host and firmware
	 * for each LIN channel
	 */
	struct llce_lin_command lin_cmd[LLCE_LIN_MAXCTRL_COUNT];
	/**
	 * Array used to exchange commands between firmware and host
	 * for each LIN channel
	 */
	struct llce_lin_command lin_notif[LLCE_LIN_MAXCTRL_COUNT];
	/**
	 * Reception message buffers located in the shared
	 * memory.
	 */
	u8 lin_mb[LLCE_LIN_MAXCTRL_COUNT][LLCE_LIN_CONFIG_PAYLOAD_MAX_SIZE];
} __aligned(4) __packed;

#endif /*LLCE_INTERFACE_LIN_H*/
