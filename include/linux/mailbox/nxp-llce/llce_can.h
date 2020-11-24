/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020 NXP */
#ifndef LLCE_INTERFACECANTYPES_H
#define LLCE_INTERFACECANTYPES_H

#include <linux/types.h>
#include "llce_fw_version.h"
#include "llce_interface_config.h"

/**
 * Controller option used by the initialization command in order to
 * inform LLCE firmware that a specific controller shall be initialized.
 * See struct llce_can_init_cmd
 */
#define LLCE_CAN_CONTROLLERCONFIG_CTRL_EN (0x10000000U)
/** Not used currently by LLCE firmware.*/
#define LLCE_CAN_CONTROLLERCONFIG_RXPOL_EN (0x00000400U)
/** Not used currently by LLCE firmware.*/
#define LLCE_CAN_CONTROLLERCONFIG_RXINT_EN (0x00000200U)
/** Not used currently by LLCE firmware.*/
#define LLCE_CAN_CONTROLLERCONFIG_TXPOL_EN (0x00000800U)
/** Not used currently by LLCE firmware.*/
#define LLCE_CAN_CONTROLLERCONFIG_TXINT_EN (0x00000100U)
/** Not used currently by LLCE firmware.*/
#define LLCE_CAN_CONTROLLERCONFIG_BOPOL_EN (0x00001000U)
/** Not used currently by LLCE firmware.*/
#define LLCE_CAN_CONTROLLERCONFIG_ERR_EN (0x00020000U)
/**
 * CAN controller option used to enable reporting of the Protocol
 * Exception errors.See struct llce_can_get_status_cmd.
 */
#define LLCE_CAN_CONTROLLERCONFIG_PE_EN (0x00040000U)
/** CAN controller option used to enable Timestamp feature. */
#define LLCE_CAN_CONTROLLERCONFIG_TST_END (0x00080000U)
/**
 * CAN controller option used to enable Timestamp feature at the start
 * of the CAN frame.
 */
#define LLCE_CAN_CONTROLLERCONFIG_TST_START (0x00000080U)
/**
 * CAN controller option used to enable Timestamp feature in the start
 * of frame for classical CAN frames and in the res bit for CAN FD frames.
 */
#define LLCE_CAN_CONTROLLERCONFIG_TST_FD (0x00000040U)
/** CAN controller option used to enable Listen-Only mode. */
#define LLCE_CAN_CONTROLLERCONFIG_LOM_EN (0x00100000U)
/** CAN controller option used to enable internal Loop_back mode. */
#define LLCE_CAN_CONTROLLERCONFIG_LPB_EN (0x00200000U)
/** CAN controller option used to enable self-reception mode. */
#define LLCE_CAN_CONTROLLERCONFIG_SRX_EN (0x00400000U)
/** CAN controller option used to enable automatic bus-off recovery. */
#define LLCE_CAN_CONTROLLERCONFIG_ABR_EN (0x00000001U)

/**
 * Number of interfaces used for interrupt reporting (one per channel)
 * + number of polling classes.
 */
#define LLCE_CAN_RX_TX_INTERFACES                                              \
	((u8)(LLCE_CAN_CONFIG_MAXCTRL_COUNT + LLCE_CAN_MAX_POLLING_CLASSES))
/** Not used currently by LLCE firmware.*/
#define LLCE_CAN_REFERENCE_NOT_USED (0xFFU)
/**
 * Default value in the transmission request informing LLCE that tx
 * confirmation is not needed for that frame.
 */
#define LLCE_CAN_ACK_DISABLED (0xADU)

/** Frame DLC field mask. */
#define LLCE_CAN_MB_DLC_MASK (0x0000000FU)
/** Frame ID field mask. */
#define LLCE_CAN_MB_ID_MASK (0x1FFFFFFFU)
/** Frame FDF field mask. */
#define LLCE_CAN_MB_FDF (0x00008000U)
/** Frame BRS field mask. */
#define LLCE_CAN_MB_BRS (0x00010000U)
/** Frame ESI field mask. */
#define LLCE_CAN_MB_ESI (0x00020000U)
/** Frame IDE field mask. */
#define LLCE_CAN_MB_IDE (0x40000000U)
/** Frame RTR field mask. */
#define LLCE_CAN_MB_RTR (0x80000000U)
/** Frame standard ID field mask. */
#define LLCE_CAN_MB_IDSTD_MASK (0x1FFC0000U)
/** Frame extended ID field mask. */
#define LLCE_CAN_MB_IDEXT_MASK (0x0003FFFFU)
/** Routing feature default configuration. */
#define LLCE_CAN_ROUTING_OPTION_DEFAULT_CONFIG (0x00000000U)
/** Routing feature mask for non-converting frame. */
#define LLCE_CAN_ROUTING_NOCHANGE (0x00000001U)
/** Routing feature mask for converting frame (CANFD->CAN). */
#define LLCE_CAN_ROUTING_CAN (0x00000002U)
/** Routing feature mask for converting frame (CAN->CANFD). */
#define LLCE_CAN_ROUTING_CANFD (0x00000004U)
/** Routing feature mask for id remapping. */
#define LLCE_CAN_ROUTING_ID_REMAPPING_EN (0x00000008U)

/** Shift value for extracting FD flag  from CAN frame. */
#define LLCE_CAN_MB_FDF_SHIFT (15U)
/** Shift value for extracting Baud Rate Switch flag from CAN frame. */
#define LLCE_CAN_MB_BRS_SHIFT (16U)
/** Shift value for extracting ESI flag feature from CAN frame. */
#define LLCE_CAN_MB_ESI_SHIFT (17U)
/** Shift value for extracting frame ID from CAN frame. */
#define LLCE_CAN_MB_IDSTD_SHIFT (18U)
/** Shift value for extracting IDE flag from CAN frame. */
#define LLCE_CAN_MB_IDE_SHIFT (30U)
/** Shift value for extracting RTR flag from CAN frame. */
#define LLCE_CAN_MB_RTR_SHIFT (31U)

/** Shift value for extracting precalculated length from CAN frame. */
#define LLCE_CAN_MB_PRECALC_LEN_SHIFT (24U)

/** Constant used to identify a reserved mask id. */
#define LLCE_CAN_FULLCAN_MASK 0xFFFFFFFFU

/**
 * Notifications sent by LLCE to host core.
 */
enum llce_can_notification_id {
	/** No error. */
	LLCE_CAN_NOTIF_NOERROR = 0U,
	/** Error related to the common platform area. */
	LLCE_CAN_NOTIF_PLATFORMERROR,
	/** Error related to a specific channel.*/
	LLCE_CAN_NOTIF_CHANNELERROR,
	/** Notification related to changing CAN controller mode.*/
	LLCE_CAN_NOTIF_CTRLMODE,
} __packed;

/**
 * Command IDs used to interface with LLCE.
 *
 * Some of those commands are sent by the host to LLCE module and
 * others are send by LLCE module to the host.
 */
enum llce_can_command_id {
	/** Host initializes LLCE module. */
	LLCE_CAN_CMD_INIT = 0U,
	/** Host De_initialize a specific CAN controller. */
	LLCE_CAN_CMD_DEINIT,
	/** Host sets a baud rate for a specific CAN controller.*/
	LLCE_CAN_CMD_SETBAUDRATE,
	/** Host checks the state for a specific CAN controller.*/
	LLCE_CAN_CMD_GETCONTROLLERMODE,
	/** Host changes the state for a specific CAN controller.*/
	LLCE_CAN_CMD_SETCONTROLLERMODE,
	/**
	 * LLCE notify host about bus off event for a specific CAN
	 * controller.
	 */
	LLCE_CAN_CMD_BUSOFF_CONFIRMATION,
	/**
	 * LLCE deliver to the host the content of all status register
	 * of CAN controller.
	 */
	LLCE_CAN_CMD_GETSTATUS,
	/** Host configure multiple filters on the reception side.*/
	LLCE_CAN_CMD_SETFILTER,
	/**
	 * Host configure multiple advanced feature filters on the
	 * reception side.
	 */
	LLCE_CAN_CMD_SETADVANCEDFILTER,
	/** Request version string from FW.*/
	LLCE_CAN_CMD_GETFWVERSION,
	/** Host request for platform initialization.*/
	LLCE_CAN_CMD_INIT_PLATFORM,
	/**
	 * LLCE internal command request for initialization of common
	 * resources.
	 */
	LLCE_CAN_CMD_INIT_PLATFORM_COMMON,
	/* Host request for platform deinitialization.*/
	LLCE_CAN_CMD_DEINIT_PLATFORM,
	/**
	 * Host request for platform initialization regarding can2eth
	 * use case.
	 */
	LLCE_CAN_CMD_INIT_PFE,
	/* Host invalidate a specific filter.*/
	LLCE_CAN_CMD_REMOVE_FILTER,
	/**
	 * Host creates a destination to be used by advanced routing
	 * filters.
	 */
	LLCE_CAN_CMD_CREATE_AF_DESTINATION
} __packed;

/** Return status codes reported at the end of each command execution.*/
enum llce_can_return {
	/* Command was executed successfully. */
	LLCE_CAN_SUCCESS = 0x55,
	/* During command execution it was detected an error condition. */
	LLCE_CAN_ERROR,
	/* Command default value set by the host before to send it to LLCE. */
	LLCE_CAN_NOTRUN
} __packed;

/**
 * CAN frame ID type.
 *
 * It specify the CAN frame ID type based on it's length as it is defined by CAN
 * specification.
 */
enum llce_can_id_length {
	/** Extended ID (29 bits) */
	LLCE_CAN_EXTENDED = 0U,
	/** Standard ID (11 bits) */
	LLCE_CAN_STANDARD,
	/** Mixed ID (29 bits) */
	LLCE_CAN_MIXED
} __packed;

/**
 * Requested transitions of a CAN controller.
 *
 * Those controller state transitions are requested by the host in a
 * specific order.
 */
enum llce_can_state_transition {
	/** Request transition from START state into STOP state. */
	LLCE_CAN_T_STOP = 0U,
	/** Request transition from STOP state into START state. */
	LLCE_CAN_T_START,
} __packed;

/**
 * CAN controller states.
 *
 * CAN controller states as they are reported by the LLCE firmware as a
 * result of state transition requests.
 */
enum llce_can_ctrl_state {
	/** Controller is uninitialised (default) */
	LLCE_CAN_UNINIT_CTRL = 0U,
	/**
	 * Controller is in a pending state of unitialization, waiting
	 * for the resources to be restored (e.g Rx tokens)
	 */
	LLCE_CAN_UNINIT_CTRL_PENDING,
	/** Controller is stopping, but not offline yet */
	LLCE_CAN_STOP_PENDING,
	/** Controller is in the STOPPED state which means that it does
	 * not do any bus transactions.
	 */
	LLCE_CAN_STOPPED,
	/** Controller is starting, but cannot do bus transactions yet. */
	LLCE_CAN_START_PENDING,
	/**
	 * Controller is in the STARTED state which means that it do
	 * bus transactions.
	 */
	LLCE_CAN_STARTED,
	/**
	 * Controller is in the IDLE state. This state is not used by
	 * LLCE module.
	 */
	LLCE_CAN_IDLE,
	/**
	 * Controller is in state when the common components of the
	 * platfor are not initialized.
	 */
	LLCE_CAN_UNINIT_PLATFORM
} __packed;

/**
 * CAN firmware error values.
 *
 * CAN error values as they are reported by the LLCE firmware.Some of
 * them are channel related and other are platform related.
 */
enum llce_can_error {
	/** FIFO is full. */
	LLCE_ERROR_TXACK_FIFO_FULL = 1U,
	/** FIFO is full. */
	LLCE_ERROR_RXOUT_FIFO_FULL,
	/** FIFO is empty. */
	LLCE_ERROR_FIFO_EMPTY,
	/** Message buffer is not avaialable. */
	LLCE_ERROR_MB_NOTAVAILABLE,
	/**
	 * CAN protocol error due to inability to get out from the
	 * freeze mode.
	 */
	LLCE_ERROR_BCAN_FRZ_EXIT,
	/**
	 * CAN protocol error due to inability to synchronize on the
	 * bus.
	 */
	LLCE_ERROR_BCAN_SYNC,
	/** CAN protocol error due to inability to enter in freeze mode. */
	LLCE_ERROR_BCAN_FRZ_ENTER,
	/** CAN protocol error due to inability to enter in low-power mode */
	LLCE_ERROR_BCAN_LPM_EXIT,
	/** CAN protocol error due to inability to enter in soft reset. */
	LLCE_ERROR_BCAN_SRT_ENTER,
	/** CAN protocol error due to inability to enter in soft reset. */
	LLCE_ERROR_BCAN_UNKNOWN_ERROR,
	/**
	 * ACKERR indicates that an acknowledge error has been detected
	 * by the transmitter node.
	 */
	LLCE_ERROR_BCAN_ACKERR,
	/**
	 * CRCERR indicates that a CRC error has been detected by the
	 * receiver node in a CAN frame.
	 */
	LLCE_ERROR_BCAN_CRCERR,
	/**
	 * BIT0ERR indicates when an inconsistency occurs between the
	 * transmitted and the received bit in a CAN frame.
	 */
	LLCE_ERROR_BCAN_BIT0ERR,
	/**
	 * BIT1ERR indicates when an inconsistency occurs between the
	 * transmitted and the received bit in a CAN frame.
	 */
	LLCE_ERROR_BCAN_BIT1ERR,
	/**
	 * FRMERR indicates that a form error has been detected by the
	 * receiver node in a CAN frame, that is, a fixed-form bit field
	 * contains at least one illegal bit.
	 */
	LLCE_ERROR_BCAN_FRMERR,
	/**
	 * STFERR indicates that a stuffing error has been detected by
	 * the receiver node in a CAN frame.
	 */
	LLCE_ERROR_BCAN_STFERR,
	/**
	 * Reports data lost event due to resources exceeded after the
	 * frame was received
	 */
	LLCE_ERROR_DATA_LOST,
	/** TXLUT acclerator is full. */
	LLCE_ERROR_TXLUT_FULL,
	/** Error during command processing. */
	LLCE_ERROR_CMD_PROCESSING,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_SLOW_SEARCH,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_ACCESS_MODE,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_SEARCH_MODE,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_SLOW_OPERATION,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_INCOMPLETE_OP,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_OPERATING_MODE,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_INIT_SLOW_OP,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_DEINIT_SLOW_OP,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_INIT_OPERATING_MODE,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_DEINIT_OPERATING_MODE1,
	/** Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_DEINIT_OPERATING_MODE2,
	/** Error regarding bus off event. */
	LLCE_ERROR_HARDWARE_BUSOFF,
	/** Controller is not ready. */
	LLCE_ERROR_CTRL_NOT_READY,
	/** Error regarding bus off. */
	LLCE_ERROR_BUSOFF,
	/** Logging fifo is full. */
	LLCE_ERROR_FIFO_LOG_FULL,
	/** Error reported due to can2can routing error. */
	LLCE_ERROR_CAN2CAN,
	/** Error reported due to wrong command parameters received from host */
	LLCE_ERROR_COMMAND_PARAM,
	/** Error reported due to the rx core not responding. */
	LLCE_ERROR_COMMAND_RXPPE_NOTRESPONSE,
	/** Error reported because the controller is not stopped. */
	LLCE_ERROR_COMMAND_DEINIT_NOTSTOP,
	/**
	 * Error reported because the host didn't read all the RX
	 * tokens (indexes in fifos). LLCE waits for indexes to be read and
	 * returned.
	 */
	LLCE_ERROR_RXTOKENS_UNRETURNED,
	/**
	 * Error reported because the host didn't read all the ACKs
	 * (indexes in fifos). LLCE waits for indexes to be read.
	 */
	LLCE_ERROR_TXACK_NOT_READ,
	/** Error reported because the requested command is not supported. */
	LLCE_ERROR_COMMAND_NOTSUPPORTED,
	/**
	 * Error reported because command is not validated by the
	 * command flow.
	 */
	LLCE_ERROR_COMMAND_NOTVALIDATED,
	/**
	 * Error reported because the requested command is correct but
	 * it not accepted.
	 */
	LLCE_ERROR_COMMAND_NOTACCEPTED,
	/**
	 * Error reported because the requested command parameters are
	 * invalid.
	 */
	LLCE_ERROR_COMMAND_INVALID_PARAMS,
	/** Controller is not started. */
	LLCE_ERROR_CTRL_NOT_STARTED,
	/**
	 * Reports frame accepted, but not delivered to host because of
	 * filters misconfiguration.
	 */
	LLCE_ERROR_FRAME_NOT_DELIVERED,
	/**
	 * Reports frame accepted, but not delivered to AF destination
	 * because of full fifo.
	 */
	LLCE_ERROR_FRAME_NOT_DELIVERED_TO_AF,
	/**
	 * Reports frame accepted, but not delivered to host due to
	 * lack of descriptors in sw fifo.
	 */
	LLCE_ERROR_FRAME_NOT_DELIVERED_TO_HOST,
	/** Reports detection of lost indexes in RX-DTE subsystem . */
	LLCE_ERROR_LOST_INDEXES,
	/**
	 * Error reported because there are no filters avaialable to be
	 * set for a specific controller.
	 */
	LLCE_ERROR_FILTERS_FULL,
	/**
	 * The filter pointed by the related address is not used by the
	 * related controller.
	 */
	LLCE_ERROR_FILTERS_NOTEXIST,
	/** There are no free configuration filters. */
	LLCE_ERROR_FILTERS_MASK_EMPTY,
	/** There are no free configuration filters. */
	LLCE_ERROR_FILTERS_RANGE_EMPTY,
	/** There are no free exact match filters. */
	LLCE_ERROR_FILTERS_EM_EMPTY,
	/** The index return by host is not valid. */
	LLCE_ERROR_IDX_NOT_VALID_HOST,
	/** The index return by logging is not valid. */
	LLCE_ERROR_IDX_NOT_VALID_LOG,
	/**
	 * The host core which sent a free rx descriptor index to Llce
	 * is invalid.
	 */
	LLCE_ERROR_INVALID_HOST_CORE,
	/**
	 * Reports frame accepted, but not delivered to HSE because of
	 * full fifo.
	 */
	LLCE_ERROR_RXFRAME_NOT_DELIVERED_TO_HSE,
	/** Tx frame was not delivered to HSE because of full fifo. */
	LLCE_ERROR_TXFRAME_NOT_DELIVERED_TO_HSE,
	/** Rx frame was dropped because it is not authentic. */
	LLCE_ERROR_RXFRAME_AUTH_ERROR,
	/** FRPE core received an invalid request from Tx core. */
	LLCE_ERROR_INVALID_REQUEST_FROM_TX,
	/** FRPE core received an invalid request from Rx core. */
	LLCE_ERROR_INVALID_REQUEST_FROM_RX,
	/** RX Software FIFO is empty. */
	LLCE_ERROR_RX_SW_FIFO_EMPTY
} __packed;

/**
 * CAN firmware components IDs.
 *
 * CAN firmware components IDs used to identify the component which
 * generated a specific error.
 */
enum llce_can_module {
	/** CAN TX firmware component. */
	LLCE_TX = 101U,
	/** CAN RX firmware component. */
	LLCE_RX,
	/** CAN DTE firmware component. */
	LLCE_DTE,
	/** CAN FRPE firmware component. */
	LLCE_FRPE
} __packed;

/**
 * CAN Logging options
 *
 * CAN options for logging frames feature.
 */
enum llce_af_logging_options {
	/** Logging of CAN frame is disabled.*/
	LLCE_AF_LOGGING_DISABLED = 1U,
	/** Logging of CAN frame is enabled.*/
	LLCE_AF_LOGGING_ENABLED
} __packed;

enum llce_can_host_receive_options {
	/** Logging of CAN frame is disabled.*/
	LLCE_AF_HOSTRECEIVE_DISABLED = 1U,
	/** Logging of CAN frame is enabled.*/
	LLCE_AF_HOSTRECEIVE_ENABLED
} __packed;

enum llce_af_authentication_options {
	/** Authentication of CAN frame is disabled.*/
	LLCE_AF_AUTHENTICATION_DISABLED = 1U,
	/** Authentication of CAN frame is enabled.*/
	LLCE_AF_AUTHENTICATION_ENABLED,
	LLCE_AF_AUTHENTICATION_NOT_SUPPORTED
} __packed;

/**
 * RXLUT entries type
 *
 * Specifies the type of entry in the table.
 */
enum llce_can_entry {
	/** Exact match entry type. */
	LLCE_CAN_ENTRY_EXACT_MATCH = 0U,
	/** Masked match entry type. */
	LLCE_CAN_ENTRY_CFG_MASKED,
	/** Range match entry type. */
	LLCE_CAN_ENTRY_CFG_RANGED,
} __packed;

/**
 * Type of Advanced Feature(AF) rule.
 *
 * Specifies the type of entry in the destination rule table.
 */
enum llce_af_rule_id {
	/** Destination rule type used for can2can use case. */
	CAN_AF_CAN2CAN = 0U,
	/** Destination rule type used for can2eth use case. */
	CAN_AF_CAN2ETH
} __packed;

/**
 * Initialization status of the controllers.
 * See struct llce_can_init_platform_cmd
 */
enum llce_can_status {
	/** Entity is initialised */
	INITIALIZED = 1U,
	/** Entity is uninitialised (default) */
	UNINITIALIZED = 2U
} __packed;

/**
 * Processing type of a specific error.
 * See struct llce_can_init_platform_cmd.
 */
enum llce_can_error_processing {
	/** LLCE Firmware does not report the error */
	IGNORE = 1U,
	/**
	 * LLCE Firmware reports the error through the notification
	 * table corresponding to interrupt processing
	 */
	INTERRUPT,
	/**
	 * LLCE Firmware reports the error through the notification
	 * table corresponding to polling processing
	 */
	POLLING,
} __packed;

#pragma pack(4)

/**
 * CAN message buffer.
 *
 * CAN message buffer is a memory area placed in the shared memory which is used
 * by the LLCE firmware to receive/transmit from/to BCAN controller.
 * LLCE firmware transmits/receives the frame in a word by word way so the
 * content of the 4 structure fields contains the frame fields.
 * For the reception process the LLCE firmware store inside message buffer frame
 * the time stamp read from the hardware CAN controller.
 * Before to use any message buffer it is needed to initialize, configure and
 * start a CAN controller
 */
struct llce_can_mb {
	/**
	 * INPUT/OUTPUT: The first word of a frame as it is
	 * expected/provided by the CAN controller.
	 */
	u32 word0;
	/**
	 * INPUT/OUTPUT: The second word of a frame as it is
	 * expected/provided by the CAN controller.
	 */
	u32 word1;
	/**
	 * INPUT/OUTPUT: Frame payload needed for the maximum payload
	 * size case.
	 */
	u8 payload[LLCE_CAN_CONFIG_PAYLOAD_MAX_SIZE];
	/**
	 * INPUT: Time stamp of the received frames.It is not used for
	 * the transmitted frames.
	 */
	u32 timestamp;
};

/** Reception message buffer descriptor.
 *
 * Reception message buffer descriptor is a memory area placed in the
 * shared memory which is written by the LLCE firmware  with the specific
 * runtime info needed by the host software.(e.g.matching filter ID ).
 * Also it includes an index to a CAN message buffer allocated during
 * initialization to each descriptor.
 * After reception, the host shall copy the content of the reception message
 * buffer descriptor and the referred message buffer by this descriptor from
 * the shared memory into the host memory in order to be processed later by
 * the host software and to allow the current message buffer descriptor to be
 * used by LLCE firmware for the reception of a new frame.
 * Before to use any receive message buffer descriptor it is needed to
 * initialize, configure and start a CAN controller
 */
struct llce_can_rx_mb_desc {
	/**
	 * OUTPUT: Filter identifier resulted at the end of filtering
	 * process.
	 * This field is completed by the LLCE filtering mechanism with a
	 * value which was configured during initialization time.
	 * It is used in order to map a received frame to a specific filter
	 * defined by the host.
	 */
	u16 filter_id;
	/**
	 * OUTPUT: Index to the CAN message buffer.
	 * See struct llce_can_mb
	 */
	u16 mb_frame_idx;
};

/**
 * Transmission message buffer descriptor.
 *
 * Transmission message buffer descriptor is a memory area placed in the shared
 * memory which is written by the host software with other additional info
 * (e.g. frame tag IDs) which is send back to the host by the LLCE firmware as
 * acknowledge information.
 *
 * Those internal tags are not changed/used by the LLCE firmware.
 * Before to use any transmission message buffer descriptor it is needed to
 * initialize, configure and start a CAN controller
 */
struct llce_can_tx_mb_desc {
	/**
	 * INPUT: Host defined tag used to track a specific frame. This
	 * field is not changed by the LLCE firmware and is returned back to the
	 * host as it is.
	 * See struct llce_can_tx2host_ack_info
	 */
	u16 frame_tag1;
	/**
	 * INPUT: Host defined tag used to track a specific frame. This
	 * field is not changed by the LLCE firmware and is returned back to the
	 * host as it is.
	 * See  struct llce_can_tx2host_ack_info
	 */
	u16 frame_tag2;
	/**
	 * INPUT: Host defined interface used to select the acknowledge
	 * interface of a specific frame. This field is not changed by the LLCE
	 * firmware.
	 */
	u8 ack_interface;
	/**
	 * OUTPUT: Index to the frame message buffer.
	 * See struct llce_can_mb
	 */
	u16 mb_frame_idx;
	/**
	 * INPUT: Request firmware to add MAC code to the transmitted
	 * frame payload.
	 */
	bool enable_tx_frame_mac;
};

/**
 * Acknowledge transmission information send from LLCE to host.
 *
 * It is used in order to send from LLCE to host needed information in order to
 * identify and confirm that a specific frame was transmitted on the CAN bus.
 * This data structure type is used in order to implement a circular buffer for
 * each channel which is accessed by using indexes transferred from LLCE to host
 * by using TXACK FIFOs.
 * This approach allows usage of existing hardware FIFOs even the size of the
 * transferred data is higher than the FIFO element width size.
 * Before to read any acknowledge information, it is needed to do a transmission
 * request.
 */
struct llce_can_tx2host_ack_info {
	/**
	 * OUTPUT: Host defined tag used to track a specific frame. This
	 * field is not changed by the LLCE firmware and is returned back to the
	 * host as it is.
	 * See struct llce_can_tx_mb_desc
	 */
	u16 frame_tag1;
	/**
	 * OUTPUT: Host defined tag used to track a specific frame. This field
	 * is not changed by the LLCE firmware and is returned back to the host
	 * as it is.
	 * See  struct llce_can_tx_mb_desc
	 */
	u16 frame_tag2;
	/** OUTPUT: Transmission time stamp.*/
	u32 tx_timestamp;

};

/**
 * Command for polling of controller state.
 *
 * It is send from host to LLCE to query it for the controller state.
 */
struct llce_can_get_controller_mode_cmd {
	/** OUTPUT: Current state of the CAN controller. */
	enum llce_can_ctrl_state controller_state;

};

/**
 * Set controller mode command.
 *
 * It is send from host to LLCE module in order request changing the state of a
 * CAN controller. Currently it allows only to start and stop a controller.
 * When a controller is started it allows to transmit and receive frames from
 * the bus. When the controller is stopped it ignores all frames from the bus
 * and it doesn't transmit any frame.
 *
 * Before changing the controller state it must be initialized.
 */
struct llce_can_set_controller_mode_cmd {
	/** INPUT: The new state which is requested. */
	enum llce_can_state_transition transition;
};

/**
 * Data baud rate settings for a CAN FD controller.
 *
 * It is used to configure the CAN FD settings including baudrate used
 * during data phase.
 */
struct llce_can_controller_fd_config {
	/**
	 * INPUT: Enable or disable FD related features of the CAN controller.
	 */
	bool can_fd_enable;
	/**
	 * INPUT: Configuration of data phase baud rate:
	 *	- prescaler divisor (bit23-27)
	 *	- Resynchronization Jump Width (bit16-19)
	 *	- Time Segment2 (bit9-12)
	 *	- Time Segment1 (bit0-4)
	 * Each parameter value shall be decreased by 1 when it is
	 * written into this data structure field.
	 */
	u32 can_data_baudrate_config;
	/**
	 * INPUT: Enable or disable baud rate switch(BRS) at the level
	 * of CAN controller.
	 */
	bool can_controller_tx_bit_rate_switch;
	/**
	 * INPUT: Enable or disable Transceiver Delay Compensation:
	 * TRUE=enabled, FALSE=disabled.
	 */
	bool can_trcv_delay_comp_enable;
	/**
	 * INPUT: Enable or disable Transceiver Delay Measurement:
	 *	- TRUE=enabled,
	 *	- FALSE=disabled.
	 * When it is enabled, the secondary sample point is determined by the
	 * sum of the tranceiver delay measurement plus transceiver delay
	 * compensation offset.
	 * When it is disabled, the secondary sample point  is determined only
	 * by the transceiver delay compensation offset.
	 */
	bool can_trcv_delay_meas_enable;
	/** INPUT: Value of Transceiver Delay Compensation Offset*/
	u8 can_trcv_delay_comp_offset;
};

/**
 * Set baud rate command.
 *
 * It is send from host to LLCE module in order to configure baud rate
 * parameters for arbitration phase.
 */
struct llce_can_set_baudrate_cmd {
	/**
	 * INPUT: Configuration parameters for nominal baud
	 * rate:
	 *	- prescaler divisor(bit23-31),
	 *	- Resynchronization Jump Width(bit16-22),
	 *	- Time Segment2(bit9-15),
	 *	- Time Segment1(bit0-7).
	 * Each parameter value shall be decreased by 1 when it is
	 * written into this data structure field.
	 */
	u32 nominal_baudrate_config;
	/**
	 * INPUT: Configuration parameters for data baud rate of the CAN
	 * controller.
	 */
	struct llce_can_controller_fd_config controller_fd;
};

/**
 * Filter element settings.
 *
 * It is used to define a specific filter.Current filtering process suppose to
 * accept a frame if it's frame ID match the filter ID masked with the mask
 * value. At the end of filtering process an internal filter ID is mapped to the
 * accepted frame in order to track it later by the host software.
 * A maximum number of frames accepted by a specific filter can be managed by
 * LLCE at each point in time.
 */
struct llce_can_rx_filter {
	/**
	 * INPUT: For MASK filters: Frame id mask value. Bit fields containing
	 * 0 means don't care.
	 * For RANGE filters: Maximum accepted id value.
	 * For EXACT MATCH: not used.
	 */
	u32 id_mask;
	/**
	 * INPUT: For MASK filters: CAN frame ID value.
	 * For RANGE filters: Minimum accepted id value.
	 * For EXACT MATCH: id value
	 */
	u32 message_id;
	/**
	 * INPUT: Filter identifier used to track frames after filtering
	 * process on the reception side.
	 * See also: struct llce_can_rx_mb_desc
	 */
	u16 filter_id;
	/**
	 * INPUT: Maximum number of message buffers which can be used to
	 * store frames accepted by this filter at each specific point in time.
	 * When the maximum value is reached the firmware will begin to drop the
	 * received frames accepted by that filter.It is used also in order to
	 * prevent that the frames accepted by a specific filter do not overload
	 * the LLCE internal hardware resources (e.g. message buffers, FIFOs).
	 */
	u16 mb_count;
	/**
	 * INPUT: Reception interface id used to deliver frames accepted
	 * by that filter to the host.
	 */
	u8 rx_dest_interface;
	/** INPUT: Filter entry type: mask, range, exact match */
	enum llce_can_entry entry_type;
	/**
	 * OUTPUT: Filter address inside hw filtering accelerator where
	 * the filter fields are stored. Host side application can use this to
	 * track used filter entries.
	 * The search operation start from low filter addresses and continue to
	 * the high filter addresses.
	 */
	u16 filter_addr;

};

/**
 * Set filter command.
 *
 * It is send by the host to LLCE in order to configure one or more reception
 * filters inside LLCE.
 */
struct llce_can_set_filter_cmd {
	/** INPUT: Number of configured filters. */
	u16 rx_filters_count;
	/** INPUT: Array containing configuration for reception filters. */
	struct llce_can_rx_filter rx_filters[LLCE_CAN_CONFIG_MAX_FILTERS_COUNT];
};

/**
 * CAN to CAN routing filter configuration.
 *
 * It is used to define a specific routing filter. Current routing
 * implementation suppose to accept a received frame for a specific ID and ID
 * mask combination, then route that frame to one or more transmission channels.
 */
struct llce_can_can2can_routing_table {
	/** INPUT: List of destination CAN controllers for the accepted frame */
	u8 dest_hw_ch_list[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	/** INPUT: Length of the destination CAN controller list.*/
	u8 dest_hw_ch_list_count;
	/** INPUT: Special option for advanced routing.*/
	u32 can2can_routing_options;
	/** INPUT: Can Id Remap Value.*/
	u32 can_id_remap_value;
} __aligned(4);

/**
 * Data strucure type containing CAN to Ethernet destination rule configuration.
 *
 * It is used to define a specific destination rule for can2eth internal routing
 */
struct llce_can_can2eth_routing_table {
	/**
	 * INPUT: Ethernet MAC destination address as it is defined by
	 * IEEE 802.3 standard.
	 */
	u8 can2eth_dest_mac[6];
} __aligned(4);

/**
 * Data structure type representing  destination rule used by
 * Advanced Features(AF)
 *
 * Used to hold a generic type of AF destination rule
 */
struct can_af_dest_rules {
	/** INPUT: Destination rule type.*/
	enum llce_af_rule_id af_dest_id;
	/** Destination rule content.*/
	union {
		/**
		 * INPUT: Destination rule for can2can internal routing
		 * use case.
		 */
		struct llce_can_can2can_routing_table can2can;
		/** INPUT: Destination rule for can2eth use case.*/
		struct llce_can_can2eth_routing_table can2eth;
	} af_dest;
};

/**
 * Advanced filter configuration.
 *
 * It is used to define an advanced filter. It contains references to
 * the individual features configured by the host.
 */
struct llce_can_advanced_feature {
	/** INPUT: Option for frame authentication feature. */
	enum llce_af_authentication_options can_authentication_feature;
	/** INPUT: Option for host receive feature. */
	enum llce_can_host_receive_options host_receive;
	/** INPUT: Option for logging feature. */
	enum llce_af_logging_options can_logging_feature;
	/**
	 * INPUT: can2can routing table index. Reference to the routing
	 * table rule. See  struct llce_can_can2can_routing_table
	 */
	u8 can2can_routing_table_idx;
	/**
	 * INPUT: can2eth routing table index. Reference to the routing
	 * table rule. See struct llce_can_can2eth_routing_table
	 */
	u8 can2eth_routing_table_idx;
} __aligned(4);

/**
 * Advanced filter element configuration.
 *
 * It is used to define a specific filter. Current filtering suppose to accept
 * a frame for processing if it's frame ID match the filter ID masked with the
 * mask value. At the end of filtering process the frame is processed according
 * to the advanced configuration of the filter.
 */
struct llce_can_advanced_filter {
	/** INPUT: Standard filter configuration. */
	struct llce_can_rx_filter llce_can_rx_filter;
	/** INPUT: Can advanced features used by the filter. */
	struct llce_can_advanced_feature llce_can_advanced_feature;
} __aligned(4);

/**
 * Set advanced filter command.
 *
 * It is send by the host to LLCE in order to set one or more advanced filters.
 */
struct llce_can_set_advanced_filter_cmd {
	/** INPUT: Number of configured filters. */
	u16 rx_filters_count;
	/** INPUT: Array containing configuration for one or more filters. */
	struct llce_can_advanced_filter
		advanced_filters[LLCE_CAN_CONFIG_ADVANCED_FILTERS_COUNT];
};

/** Configurable errors that LLCE Firmware handles.
 *
 * Configurable errors that LLCE Firmware handles using different types of
 * processing. It is part of platform init command. Only Bus_off processing type
 * is selectable per channel.
 */
struct llce_can_error_category {
	/** CAN Protocol errors supported by CAN controller. */
	enum llce_can_error_processing can_protocol_err;
	/** DATALOST event as described by Autosar standard */
	enum llce_can_error_processing data_lost_err;
	/**
	 * Initialization errors, like resources overflow, bad commands flow,
	 * bad command parameters, invalid indexes or any other errors caused by
	 * bad usage or a malicious host.
	 */
	enum llce_can_error_processing init_err;
	/** Internal errors, like timeouts. */
	enum llce_can_error_processing internal_err;
	/** Bus_off processing is selectable per channel */
	enum llce_can_error_processing
		bus_off_err[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
};

/**
 * Platform initialization command.
 *
 * It is send by the host to LLCE in order to configure the platform related
 * parameters. It is the first command which shall be send to LLCE module after
 * booting in order to configure common LLCE components.
 * Most important operations done by LLCE firmware at platform initialization
 * are:
 * - initialize also all common internal LLCE components.
 * - prepare filter management data structures
 */
struct llce_can_init_platform_cmd {
	/**
	 * INPUT: Array containing the initialization status of the controllers
	 */
	enum llce_can_status ctrl_init_status[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	/**
	 * INPUT: Array containing maximum number of filters per channel.
	 * See LLCE_CAN_CONTROLLERCONFIG_CTRL_EN_U32 as a controller option
	 * example
	 */
	u16 max_filter_count[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	/**
	 * INPUT: Array containing maximum number of RX message buffers
	 * per channel, considering interrupt processing.
	 */
	u16 max_int_mb_count[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	/**
	 * INPUT: Array containing maximum number of RX message buffers per
	 * polling class, considering polling processing. There are a number
	 * of LLCE_CAN_MAX_POLLING_CLASSES provided.
	 */
	u16 max_poll_mb_count[LLCE_CAN_MAX_POLLING_CLASSES];
	/**
	 * INPUT: Array containing maximum number of reserved TX confirmation
	 * buffers per channel, considering interrupt processing.
	 */
	u16 max_int_tx_ack_count[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	/**
	 * INPUT: Array containing maximum number of reserved TX confirmation
	 * buffers per polling class, considering polling processing. There are
	 * a number of  LLCE_CAN_MAX_POLLING_CLASSES provided.
	 */
	u16 max_poll_tx_ack_count[LLCE_CAN_MAX_POLLING_CLASSES];
	/**
	 * INPUT: Structure describing the way of processing each errors
	 * category. This can be: INTERRUPT, POLLING or IGNORE.
	 */
	struct llce_can_error_category can_error_reporting;
};

/**
 * Initialization command.
 *
 * It is send by the host to LLCE in order to load and configure all needed
 * parameters inside LLCE. It is the first command which shall be send to LLCE
 * module after booting.
 * Most important operations done by LLCE firmware at initialization are:
 * - when this command is executed for the first CAN channel it initialize also
 * all internal LLCE components.
 * - configure the hardware features of a CAN controller.
 * - set a CAN controller in the stop state.
 */
struct llce_can_init_cmd {
	/**
	 * INPUT: Configuration options for a hardware CAN controller.
	 * See LLCE_CAN_CONTROLLERCONFIG_CTRL_EN_U32 as a controller option
	 * example
	 */
	u32 ctrl_config;
	/**
	 * INPUT: Number of transmission message buffer descriptors used
	 * for transmissions initialized by the host.The remaining elements are
	 * used for internal routing scenarios.
	 */
	u8 tx_mb_count;
};

/**
 * LLCE-PFE Initialization command.
 *
 * It is sent by the host to LLCE in order to activate the LLCE-PFE
 * interface, and obtain the addresses of the relevant data structures.
 */
struct llce_can_init_pfe_cmd {
	/** OUTPUT: Address of the PFE RX Ring in LLCE memory */
	void *rx_ring;
	/** OUTPUT: Address of the PFE RX Writeback Ring in LLCE memory */
	void *rx_wb_ring;
	/** OUTPUT: Address of the PFE TX Ring in LLCE memory */
	void *tx_ring;
	/** OUTPUT: Address of the PFE TX Writeback Ring in LLCE memory */
	void *tx_wb_ring;
};

/**
 * Get status command.
 *
 * It is send by the host to LLCE in order to get the content of all status
 * registers of a specific CAN controller. This command makes only a read
 * operation on the status registers of CAN controller.
 */
struct llce_can_get_status_cmd {
	/** OUTPUT: Register ECR of CAN controller. */
	u32 ecr;
	/** OUTPUT: Register ISR of CAN controller. */
	u32 isr;
	/** OUTPUT: Register SR of CAN controller. */
	u32 sr;
	/** OUTPUT: Register CRC of CAN controller. */
	u32 crc;
};

/**
 * Get firmware version command.
 *
 * It is send by the host to LLCE in order to get the firmware version string.
 * It is copied in the response.
 */
struct llce_can_get_fw_version {
	/** OUTPUT: LLCE FW version string actual length. */
	u8 string_length;
	/** OUTPUT: LLCE FW version string. */
	u8 version_string[LLCE_VERSION_MAX_LENGTH];
};

/**
 * Remove filter command structure.
 *
 * It is send by the host to LLCE in order to remove a specific filter
 * identified by a hardware address.
 */
struct llce_can_remove_filter {
	/** INPUT: Address of the filter which shall be removed/disabled. */
	u16 filter_addr;

};

/**
 * Add AF destination command structure.
 *
 * It is send by the host to LLCE in order add a destination to be used by AF
 */
struct llce_can_create_af_destination {
	/** INPUT: Destination to add to the list */
	struct can_af_dest_rules rule;
	/** OUTPUT: Index in LLCE list where destination was inserted */
	u8 idx;
};

/**
 * List of commands used by host.
 *
 * It is used in order to use the same memory area for all commands send from
 * host to LLCE.
 */
union llce_can_command_list {
	/** Command for initializing a specific CAN channel. */
	struct llce_can_init_cmd init;
	/**
	 * Command for getting the hardware status information for a
	 * specific CAN controller.
	 */
	struct llce_can_get_status_cmd get_status;
	/**
	 * Command for configuring filters for a specific CAN controller
	 * in order to deliver frames to the host.
	 */
	struct llce_can_set_filter_cmd set_filter;
	/**
	 * Command for configuring baud rate parameters for a specific
	 * CAN controller.
	 */
	struct llce_can_set_baudrate_cmd set_baudrate;
	/** Command for getting the status of a specific CAN controller. */
	struct llce_can_get_controller_mode_cmd get_controller_mode;
	/** Command for changing the status of a specific CAN controller. */
	struct llce_can_set_controller_mode_cmd set_controller_mode;
	/**
	 * Command for configuring filters in order to route frames to other
	 * destinations than host.
	 */
	struct llce_can_set_advanced_filter_cmd set_advanced_filter;
	/** Command for getting the firmware version. */
	struct llce_can_get_fw_version get_fw_version;
	/**
	 * Command for configuring platform related parameters and common HW
	 * components used by all CAN channels
	 */
	struct llce_can_init_platform_cmd init_platform;
	/**
	 * Command for initializing the LLCE-PFE interface and getting buffer
	 * locations
	 */
	struct llce_can_init_pfe_cmd init_pfe;
	/** Command for removing/disable a single filter. */
	struct llce_can_remove_filter remove_filter;
	/** Command for creating a destination for AF */
	struct llce_can_create_af_destination create_af_dest;
};

/**
 * Command used by host.
 *
 * It is used in order to send commands from host to LLCE using shared memory.
 */
struct llce_can_command {
	/** INPUT: Command ID. */
	enum llce_can_command_id cmd_id;
	/** OUTPUT: Return status code after command completion.*/
	enum llce_can_return return_value;
	/** INPUT: Command parameters. */
	union llce_can_command_list cmd_list;
};

/**
 * Set controller mode notification.
 *
 * Notification send from LLCe to host in order to inform about a specific
 * controller state change.
 */
struct llce_can_ctrl_mode_notif {
	/**
	 * OUTPUT: Current state of the CAN controller.
	 * See also enum llce_can_ctrl_state
	 */
	enum llce_can_ctrl_state controller_state;
	/** OUTPUT: CAN controller id. */
	u8 hw_ctrl;
};

/**
 * Platform specific error.
 *
 * Platform error details send to host in order to report an internal LLCE error
 */
struct llce_can_error_notif {
	/**
	 * OUTPUT: LLCE firmware component id.
	 * See also enum llce_can_module
	 */
	enum llce_can_module module_id;
	/** OUTPUT: LLCE error code.See also  enum llce_can_error */
	enum llce_can_error error_code;
	/** OUTPUT: Number of ocurrences of the last error. */
	u8 error_count;
};

/**
 * Channel specific error.
 *
 * Channel error details send to host in order to report an internal LLCE error.
 */
struct llce_can_channel_error_notif {
	struct llce_can_error_notif error_info;
	/** OUTPUT: Controller ID. */
	u8 hw_ctrl;
};

/**
 * List of notifications send by LLCe to host. used by host.
 *
 * It is used by LLCE to notify host about specific events inside LLCE.
 */
union llce_can_notification_list {
	/**
	 * OUTPUT: Notification parameters for controller state changes.
	 * See also struct llce_can_ctrl_mode_notif
	 */
	struct llce_can_ctrl_mode_notif ctrl_mode;
	/**
	 * OUTPUT: Notification parameters for platform errors.
	 * See also struct llce_can_error_notif
	 */
	struct llce_can_error_notif platform_error;
	/**
	 * OUTPUT: Notification parameters for channel errors.
	 * See also struct llce_can_channel_error_notif
	 */
	struct llce_can_channel_error_notif channel_error;
};

/**
 * Notifications used by LLCE.
 *
 * It is used in order to send notifications from LLCE to host by using shared
 * memory.
 */
struct llce_can_notification {
	/**
	 * OUTPUT: Notification ID.
	 * See also enum llce_can_notification_id
	 */
	enum llce_can_notification_id notif_id;
	/**
	 * OUTPUT: Notification parameters.
	 * See also llce_can_notification_list_type
	 */
	union llce_can_notification_list notif_list;
};

/**
 * Notification tables.
 *
 * Notification tables used to store the details of the notifications.The index
 * of entries are send to host cores.The two tables are related to reporting
 * method:interrupt or polling.
 */
struct llce_can_notification_table {
	/**
	 * OUTPUT: Table used to report notifications in interrupt mode.
	 * See also  struct llce_can_notification
	 */
	struct llce_can_notification
		can_notif_intr_table[LLCE_CAN_CONFIG_HIF_COUNT]
		[LLCE_CAN_CONFIG_NOTIF_TABLE_SIZE];
	/**
	 * OUTPUT: Table used to report notifications in polling mode.
	 * See also  struct llce_can_notification
	 */
	struct llce_can_notification
		can_notif_poll_table[LLCE_CAN_CONFIG_HIF_COUNT]
		[LLCE_CAN_CONFIG_NOTIF_TABLE_SIZE];
};

/**
 * Shared memory structure
 *
 * Structure that encapsulates all the shared memory with llce on the can side.
 */
struct llce_can_shared_memory {
	/** Receive message buffer descriptors. */
	struct llce_can_rx_mb_desc can_rx_mb_desc[LLCE_CAN_CONFIG_MAXRXMB];
	/** Transmit message buffer descriptors. */
	struct llce_can_tx_mb_desc can_tx_mb_desc[LLCE_CAN_CONFIG_MAXTXMB];
	/** Shared memory used store the CAN message buffers. */
	struct llce_can_mb can_mb[LLCE_CAN_CONFIG_MAXTXMB +
		LLCE_CAN_CONFIG_MAXRXMB +
		LLCE_CAN_CONFIG_MAXAFFRMB];
	/** Shared memory used to send commands from Host to Llce . */
	struct llce_can_command can_cmd[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	/** Shared memory used to store notifications from LLCE to host. */
	struct llce_can_notification_table can_notification_table;
	/**
	 * Circular buffer used to send ack info from Tx core to HOST core.
	 * Reserved 1 extra buffer for each interface for consistency
	 * purposes.
	 */
	struct llce_can_tx2host_ack_info
		can_tx_ack_info[LLCE_CAN_CONFIG_MAX_TXACKINFO];
};

#pragma pack()

#endif /* LLCE_INTERFACECANTYPES_H */
