/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020-2024 NXP */
#ifndef LLCE_FW_INTERFACE_H
#define LLCE_FW_INTERFACE_H

#include "llce_fw_version.h"

/**
 * CAN firmware notification categories.
 * Contains the notification categories of the values returned by the
 * LLCE Firmware.
 * @see enum llce_fw_return
 **/
enum llce_fw_notification_category {
	LLCE_NOTIFCAT_CAN_PROTOCOL = 1U,
	LLCE_NOTIFCAT_DATA_LOST,
	LLCE_NOTIFCAT_CONFIGURATION,
	LLCE_NOTIFCAT_INTERNAL,
	LLCE_NOTIFCAT_BUSOFF,
	LLCE_NOTIFCAT_FW_STATUS,
	LLCE_NOTIFCAT_CAN_PROTOCOL_CRITICAL_STATE

} __packed;

/**
 * CAN firmware error values.
 * CAN error values as they are reported by the LLCE firmware. Some of them are
 * channel related and other are platform related.
 **/
enum llce_fw_return {
	/** CAN firmware error: BOTH SW AND HW TXACK FIFOS are full. */
	LLCE_ERROR_TXACK_FIFO_FULL = 1U,
	/** CAN firmware error: RXOUT SW FIFO is full. */
	LLCE_ERROR_RXOUT_FIFO_FULL,
	/** CAN firmware error: Reserved for future use. */
	LLCE_ERROR_CODE_RESERVED_0,
	/**
	 * CAN firmware error: The system lost indexes or HW FIFO was
	 * not cleared and it is full.
	 */
	LLCE_ERROR_HW_FIFO_FULL,
	/**
	 * CAN firmware error: There was an attempt to perform a pop
	 * operation on an empty SW FIFO.
	 */
	LLCE_ERROR_SW_FIFO_EMPTY,
	/**
	 * CAN firmware error: There was an attempt to perform a push
	 * operation on a full SW FIFO.
	 */
	LLCE_ERROR_SW_FIFO_FULL,
	/**
	 * CAN firmware error: The maximum hardware object count on the
	 * reception side was reached.
	 */
	LLCE_ERROR_MB_NOTAVAILABLE,
	/**
	 * CAN firmware error: Short Message buffer is configured but a
	 * long frame is received.
	 */
	LLCE_ERROR_SHORT_MB_NOTAVAILABLE,
	/**
	 * CAN firmware error: Reserved for future use.
	 */
	LLCE_ERROR_BCAN_FRZ_EXIT,
	/**
	 * CAN firmware error: Reserved for future use.
	 */
	LLCE_ERROR_BCAN_SYNC,
	/**
	 * CAN firmware error: Reserved for future use.
	 */
	LLCE_ERROR_BCAN_FRZ_ENTER,
	/**
	 * CAN firmware error: CAN protocol error due to inability to
	 * enter in low-power mode.
	 */
	LLCE_ERROR_BCAN_LPM_EXIT,
	/**
	 * CAN firmware error: CAN protocol error due to inability to
	 * enter in soft reset.
	 */
	LLCE_ERROR_BCAN_SRT_ENTER,
	/**
	 * CAN firmware error: An error callback was called, but no error
	 * code matches : unknown CAN protocol error
	 */
	LLCE_ERROR_BCAN_UNKNOWN_ERROR,
	/**
	 * CAN firmware error: ACKERR indicates that an acknowledge error
	 * has been detected by the transmitter node.
	 */
	LLCE_ERROR_BCAN_ACKERR,
	/**
	 * CAN firmware error: CRCERR indicates that a CRC error has been
	 * detected by the receiver node in a CAN frame.
	 */
	LLCE_ERROR_BCAN_CRCERR,
	/**
	 * CAN firmware error: BIT0ERR indicates when an inconsistency
	 * occurs between the transmitted and the received bit in a CAN frame.
	 */
	LLCE_ERROR_BCAN_BIT0ERR,
	/**
	 * CAN firmware error: BIT1ERR indicates when an inconsistency
	 * occurs between the transmitted and the received bit in a CAN frame.
	 */
	LLCE_ERROR_BCAN_BIT1ERR,
	/**
	 * CAN firmware error: DPBIT1ERR indicates when an inconsistency
	 * occurs between the transmitted and the received bit in the data phase
	 * of a CAN-FD frame.
	 */
	LLCE_ERROR_BCAN_DPBIT1ERR,
	/**
	 * CAN firmware error: DPBIT0ERR indicates when an inconsistency
	 * occurs between the transmitted and the received bit in the data phase
	 * of a CAN-FD frame.
	 */
	LLCE_ERROR_BCAN_DPBIT0ERR,
	/**
	 * CAN firmware error: DPSTFERR indicates that a stuffing error
	 * has been detected by the receiver node in the data phase of a CAN-FD
	 * frame.
	 */
	LLCE_ERROR_BCAN_DPSTFERR,
	/**
	 * CAN firmware error: DPFRMERR indicates that a form error has
	 * been detected by the receiver node in the data phase of a CAN-FD
	 * frame
	 * - a fixed-form bit field contains at least one illegal bit.
	 */
	LLCE_ERROR_BCAN_DPFRMERR,
	/**
	 * CAN firmware error: DPCRCERR indicates that a CRC error has
	 * been detected by the receiver node in the data phase of a CAN-FD
	 * frame
	 */
	LLCE_ERROR_BCAN_DPCRCERR,
	/**
	 * CAN firmware error: FRMERR indicates that a form error has
	 * been detected by the receiver node in a CAN frame - a fixed-form bit
	 * field contains at least one illegal bit.
	 */
	LLCE_ERROR_BCAN_FRMERR,
	/**
	 * CAN firmware error: STFERR indicates that a stuffing error has
	 * been detected by the receiver node in a CAN frame.
	 */
	LLCE_ERROR_BCAN_STFERR,
	/**
	 * CAN firmware error: TDC mechanism is out of range,
	 * unable to compensate the transceiver's loop delay.
	 */
	LLCE_ERROR_BCAN_TDCFAIL,
	/**
	 * CAN firmware error: Data_lost event caused by BCAN RX Fifo
	 * Overrun.
	 */
	LLCE_ERROR_BCAN_RXFIFO_OVERRUN,
	/**
	 * CAN firmware error: Reports data lost event due to resources
	 * exceeded after the frame was received
	 */
	LLCE_ERROR_DATA_LOST,
	/**
	 * CAN firmware error: The number of Message Buffers available
	 *  for transmission arbitration was exceeded or the status register
	 * indicates that the TXLUT accelerator is full.
	 */
	LLCE_ERROR_TXLUT_FULL,
	/**
	 * CAN firmware error: A command with an unknown id was issued
	 *  or the current host tried to overlap the other host's resources at
	 * initialization.
	 */
	LLCE_ERROR_CMD_PROCESSING,
	/** CAN firmware error: Reserved for future use. */
	LLCE_ERROR_CODE_RESERVED_1,
	/**
	 * CAN firmware error: Indicates hardware malfunction when
	 * trying to perform a read or write operation on RXLUT registers.
	 */
	LLCE_ERROR_RXLUT_ACCESS_MODE,
	/**
	 * CAN firmware error: Indicates hardware malfunction when
	 * trying to perform a read or write operation on RXLUT registers.
	 */
	LLCE_ERROR_RXLUT_SEARCH_MODE,
	/**
	 * CAN firmware error: Indicates hardware malfunction when
	 * trying to perform a read on RXLUT registers and no value was obtained
	 * within a configured timeout.
	 */
	LLCE_ERROR_RXLUT_SLOW_OPERATION,
	/**
	 * CAN firmware error: Indicates hardware malfunction when
	 *  trying to perform a read on the RXLUT status register during search
	 * operation and no value was obtained within a configured timeout.
	 */
	LLCE_ERROR_RXLUT_INCOMPLETE_OP,
	/**
	 * CAN firmware error: Indicates hardware malfunction when
	 * trying to perform a read on the RXLUT status register during flush
	 * operation and no value was obtained within a configured timeout.
	 */
	LLCE_ERROR_RXLUT_OPERATING_MODE,
	/**
	 * CAN firmware error: Failed to configure the filter entry
	 * address and issue write command.
	 */
	LLCE_ERROR_RXLUT_INIT_SLOW_OP,
	/** CAN firmware error: Reserved for future use. */
	LLCE_ERROR_CODE_RESERVED_2,
	/** CAN firmware error: Reserved for future use. */
	LLCE_ERROR_CODE_RESERVED_3,
	/** CAN firmware error: Reserved for future use. */
	LLCE_ERROR_CODE_RESERVED_4,
	/** CAN firmware error: Reserved for future use. */
	LLCE_ERROR_CODE_RESERVED_5,
	/** CAN firmware error: Reserved for future use. */
	LLCE_ERROR_CODE_RESERVED_6,
	/**
	 * CAN firmware error: Controller is not started or bus-off
	 * event has occurred.
	 */
	LLCE_ERROR_CTRL_NOT_READY,
	/**
	 * CAN firmware error: A bus off event was triggered.
	 * This notification is skipped in case of auto-recovery.
	 */
	LLCE_ERROR_BUSOFF,
	/** CAN firmware error: Logging FIFO is full. */
	LLCE_ERROR_FIFO_LOG_FULL,
	/**
	 * CAN firmware error: Reserved for future use.
	 */
	LLCE_ERROR_CODE_RESERVED_7,
	/**
	 * CAN firmware error: Reserved for future use.
	 */
	LLCE_ERROR_CODE_RESERVED_8,
	/**
	 * CAN firmware error: Error reported due to the rx core not
	 * responding.
	 */
	LLCE_ERROR_COMMAND_RXPPE_NORESPONSE,
	/**
	 * CAN firmware error: Error reported due to frpe core not
	 * responding.
	 */
	LLCE_ERROR_COMMAND_AF_NORESPONSE,
	/**
	 * CAN firmware error: Error reported because the controller is
	 * not stopped.
	 */
	LLCE_ERROR_COMMAND_DEINIT_NOTSTOP,
	/**
	 * CAN firmware error: Error reported because the host didn't
	 * read all the RX tokens (indexes in fifos). LLCE waits for indexes
	 * to be read and returned.
	 */
	LLCE_ERROR_RXTOKENS_UNRETURNED,
	/**
	 * CAN firmware error: Error reported because the host didn't
	 * read all the ACKs (indexes in fifos). LLCE waits for indexes to be
	 * read.
	 */
	LLCE_ERROR_TXACK_NOT_READ,
	/**
	 * CAN firmware error: Error reported because the requested
	 * command is not in the list of supported commands.
	 */
	LLCE_ERROR_COMMAND_NOTSUPPORTED,
	/**
	 * CAN firmware error: Error reported because command is not
	 * validated by the command flow.
	 */
	LLCE_ERROR_COMMAND_NOTVALIDATED,
	/**
	 * CAN firmware error: Error reported because the requested
	 * command is correct but it not accepted.
	 */
	LLCE_ERROR_COMMAND_NOTACCEPTED,
	/**
	 * CAN firmware error: Error reported because the requested
	 * command parameters are invalid.
	 */
	LLCE_ERROR_COMMAND_INVALID_PARAMS,
	/** CAN firmware error: Reserved for future use. */
	LLCE_ERROR_CODE_RESERVED_9,
	/**
	 * CAN firmware error: Reports frame accepted, but not delivered
	 * to host because of filters misconfiguration.
	 */
	LLCE_ERROR_FRAME_NOT_DELIVERED,
	/**
	 * CAN firmware error: Reserved for future use.
	 */
	LLCE_ERROR_CODE_RESERVED_10,
	/**
	 * CAN firmware error: Reports frame accepted, but not delivered
	 * to host due to lack of descriptors in software FIFO.
	 */
	LLCE_ERROR_FRAME_NOT_DELIVERED_TO_HOST,
	/**
	 * CAN firmware error: Reserved for future use.
	 */
	LLCE_ERROR_CODE_RESERVED_12,
	/**
	 * CAN firmware error: Error reported because there are no
	 * filters available to be set for a specific controller.
	 */
	LLCE_ERROR_FILTERS_FULL,
	/**
	 * CAN firmware error: The filter pointed by the related address
	 * is not used by the related controller.
	 */
	LLCE_ERROR_FILTERS_NOTEXIST,
	/**
	 * CAN firmware error: There are no free configuration filters.
	 */
	LLCE_ERROR_FILTERS_MASK_EMPTY,
	/**
	 * CAN firmware error: There are no free configuration filters.
	 */
	LLCE_ERROR_FILTERS_RANGE_EMPTY,
	/** CAN firmware error: There are no free exact match filters. */
	LLCE_ERROR_FILTERS_EM_EMPTY,
	/**
	 * CAN firmware error: The index returned by host is not valid,
	 * possibly a duplicate index.
	 */
	LLCE_ERROR_IDX_NOT_VALID_HOST,
	/**
	 * CAN firmware error: The index returned by logging is not
	 * valid, the associated destination didn't match.
	 */
	LLCE_ERROR_IDX_NOT_VALID_LOG,
	/**
	 * CAN firmware error: Reserved for future use.
	 */
	LLCE_ERROR_CODE_RESERVED_13,
	/**
	 * CAN firmware error: Reserved for future use.
	 * to HSE because of full FIFO.
	 */
	LLCE_ERROR_CODE_RESERVED_14,
	/**
	 * CAN firmware error: Reserved for future use.
	 */
	LLCE_ERROR_CODE_RESERVED_15,
	/**
	 * CAN firmware error: Rx frame was dropped because it is not
	 * authentic.
	 */
	LLCE_ERROR_RXFRAME_AUTH_ERROR,
	/**
	 * CAN firmware error: Reserved for future use.
	 * TX core.
	 */
	LLCE_ERROR_CODE_RESERVED_16,
	/**
	 * CAN firmware error: Reserved for future use.
	 */
	LLCE_ERROR_CODE_RESERVED_17,
	/** CAN firmware error: RX Software FIFO is empty. */
	LLCE_ERROR_RX_SW_FIFO_EMPTY,
	/**
	 * AF error : error communicating with PFE due to PFE internal
	 * error
	 */
	LLCE_ERROR_PFEIF,
	/**
	 * AF error : error communicating with HSE due to HSE internal
	 * error
	 */
	LLCE_ERROR_HSEIF,
	/**
	 * Generic firmware code: Command was executed successfully by
	 * LLCE Firmware.
	 */
	LLCE_FW_SUCCESS,
	/**
	 * Generic firmware error: During command execution
	 * an error condition was detected.
	 */
	LLCE_FW_ERROR,
	/**
	 * Generic firmware code: Default value of command return
	 * status, set by the host before to send it to LLCE firmware.
	 */
	LLCE_FW_NOTRUN,
	/**
	 * CAN firmware error: Internal Descriptor was not returned
	 * to the source.
	 */
	LLCE_ERROR_INTERNALDESC_NOT_RETURNED,
	/**
	 * CAN firmware error: Internal Descriptor was not delivered
	 * to the destination.
	 */
	LLCE_ERROR_INTERNALDESC_NOT_DELIVERED,
	/**
	 * CAN firmware error: Internal Descriptor is not available
	 * because the internal LLCE software FIFO is empty.
	 */
	LLCE_ERROR_INTERNALDESC_NOTAVAIL,
	/**
	 * CAN firmware error: Internal Descriptor software FIFO is full.
	 */
	LLCE_ERROR_INTERNALDESC_FIFO_FULL,
	/**
	 * CAN firmware error: Message Buffer is not available.
	 * the internal message buffer software FIFO is empty.
	 */
	LLCE_ERROR_MB_NOTAVAIL,
	/** CAN firmware error: Message Buffer software FIFO is full. */
	LLCE_ERROR_MB_FIFO_FULL,
	/**
	 * CAN firmware error: Maximum number of Tx MB per controller for
	 * AF is reached.
	 */
	LLCE_ERROR_NO_MB_AVAILABLE,
	/**
	 * CAN firmware error: The source of the request did not match
	 * any of the LLCE cores ids.
	 */
	LLCE_ERROR_UNKNOWN_SRC,
	/**
	 * CAN firmware error: Unknown destination of the request : it's
	 * neither a CAN or ETHERNET channel, nor the host.
	 */
	LLCE_ERROR_UNKNOWN_DEST,
	/**
	 * CAN firmware error: Unknown request in intercore
	 * communication.
	 */
	LLCE_ERROR_UNKNOWN_REQUEST,
	/**
	 * CAN firmware error: Issued for frames with DLC greater than 8
	 * where destination is not FD .
	 */
	LLCE_ERROR_CONVERSION,
	/**
	 * CAN firmware error: AbortMB request failed due to no pending
	 * transmission that can be aborted.
	 */
	LLCE_ERROR_NO_MB_TO_ABORT,
	/**
	 * CAN firmware error: Index not recovered from DTE after Stop
	 * or busoff event.
	 */
	LLCE_ERROR_INDEX_NOT_RECOVERED,
	/**
	 * CAN firmware error: The controller did not exit from freeze
	 * mode within a configured timeout.
	 */
	LLCE_ERROR_RESET_PENDING,
	/**
	 * CAN firmware fatal error: core has entered an error state and
	 * is not recoverable.
	 */
	LLCE_ERROR_FATAL,
	/**
	 * CAN firmware notification: BCAN is ready to leave bus-off
	 * state after the automatic recovery procedure
	 */
	LLCE_NOTIF_BUSOFF_DONE,
	/**
	 * CAN firmware error: TXWRN is set when the Tx error counter
	 * ECR[TEC] reached 96.
	 */
	LLCE_ERROR_BCAN_TXWRN,
	/**
	 * CAN firmware error: RXWRN is set when the Rx error counter
	 * ECR[REC] reached 96.
	 */
	LLCE_ERROR_BCAN_RXWRN,
	/**
	 * CAN firmware error: PASSERR is set when BCAN enters Passive
	 * state.
	 */
	LLCE_ERROR_BCAN_PASSERR,
	/** CAN firmware notification: BCAN exits Passive state. */
	LLCE_NOTIF_BCAN_EXIT_PASSIVE_STATE,
	/**
	 * CAN firmware notification: There was an attempt to perform
	 * routing to a disabled destination.
	 */
	LLCE_ERROR_ROUTINGCHANNEL_DISABLED,
	/**
	 * CAN firmware notification: Shutdown request received from
	 * host.
	 */
	LLCE_SHUTDOWN_REQUESTED,
	/**
	 * CAN firmware notification: Core has entered shutdown state.
	 */
	LLCE_SHUTDOWN_ENTERED,
	/**
	 * CAN firmware notification: Core has received an unexpected
	 * event during shutdown and is halted but awake
	 */
	LLCE_SHUTDOWN_FAILED,
	/**
	 * Number of enum elements. It must be kept as the last member
	 * of the list.
	 */
	LLCE_FW_RETURNTYPE_COUNT

} __packed;

/**
 * Boot sequence data type.
 * Data type used to access shared memory area for managing LLCE boot sequence
 **/
struct llce_mgr_status {
	/**
	 * OUTPUT: Boot Status of TXPPE. This can be NOTRUN, SUCCESS, or
	 * a specific ERROR information
	 */
	enum llce_fw_return tx_boot_status;
	/**
	 * OUTPUT: Boot Status of RXPPE. This can be NOTRUN, SUCCESS, or
	 * a specific ERROR information
	 */
	enum llce_fw_return rx_boot_status;
	/**
	 * OUTPUT: Boot Status of RXPPE. This can be NOTRUN, SUCCESS, or
	 * a specific ERROR information
	 */
	enum llce_fw_return dte_boot_status;
	/**
	 * OUTPUT: Boot Status of RXPPE. This can be NOTRUN, SUCCESS, or
	 * a specific ERROR information
	 */
	enum llce_fw_return frpe_boot_status;
	/** OUTPUT: LLCE FW version structure. */
	struct llce_fw_version llce_fw_version;

} __aligned(4) __packed;

/**
 * Structure for the timestamps of cores 1, 2, and 3
 * Structure contains the timestamps for the cores 1, 2, and 3 and shall be
 * read by host in order to verify if a crash, or high delay has occurred or
 * not.
 **/
struct llce_mgr_time_stamp_cores {
	/** Timestamp for Core 1. */
	u32 time_stamp_core1;
	/** Timestamp for Core 2. */
	u32 time_stamp_core2;
	/** Timestamp for Core 3. */
	u32 time_stamp_core3;
} __aligned(4) __packed;

/**
 * Helper function that returns the category of a value returned by
 * the LLCE FW.
 * Determines the category of a value returned by the LLCE Firmware.
 * It can take as input errors, notifications and status values (see
 * enum llce_fw_return). It will return the category that value belongs to.
 * @param[in] notification_code CAN error, notification or status values as they
 * are reported by the LLCE firmware.
 * @return CAN firmware notification category.
 */
static inline enum llce_fw_notification_category
llce_get_notification_category(enum llce_fw_return notification_code)
{
	enum llce_fw_notification_category notif_category;

	switch (notification_code) {
	case LLCE_ERROR_BUSOFF:
	case LLCE_NOTIF_BUSOFF_DONE:
		notif_category = LLCE_NOTIFCAT_BUSOFF;
		break;

	/* Treat all the cases leading to DATA_LOST which should be reported to
	 * the host
	 */
	case LLCE_ERROR_RXOUT_FIFO_FULL:
	case LLCE_ERROR_MB_NOTAVAILABLE:
	case LLCE_ERROR_BCAN_RXFIFO_OVERRUN:
		notif_category = LLCE_NOTIFCAT_DATA_LOST;
		break;

		/* Can protocol errors
		 */
	case LLCE_ERROR_BCAN_TDCFAIL:
	case LLCE_ERROR_BCAN_ACKERR:
	case LLCE_ERROR_BCAN_CRCERR:
	case LLCE_ERROR_BCAN_BIT0ERR:
	case LLCE_ERROR_BCAN_BIT1ERR:
	case LLCE_ERROR_BCAN_DPBIT0ERR:
	case LLCE_ERROR_BCAN_DPBIT1ERR:
	case LLCE_ERROR_BCAN_STFERR:
	case LLCE_ERROR_BCAN_FRMERR:
	case LLCE_ERROR_BCAN_DPSTFERR:
	case LLCE_ERROR_BCAN_DPFRMERR:
	case LLCE_ERROR_BCAN_DPCRCERR:
	case LLCE_ERROR_BCAN_UNKNOWN_ERROR:
	case LLCE_ERROR_BCAN_FRZ_EXIT:
	case LLCE_ERROR_BCAN_SYNC:
	case LLCE_ERROR_BCAN_FRZ_ENTER:
	case LLCE_ERROR_BCAN_LPM_EXIT:
	case LLCE_ERROR_BCAN_SRT_ENTER:
	case LLCE_NOTIF_BCAN_EXIT_PASSIVE_STATE:
		notif_category = LLCE_NOTIFCAT_CAN_PROTOCOL;
		break;
	case LLCE_ERROR_BCAN_TXWRN:
	case LLCE_ERROR_BCAN_RXWRN:
	case LLCE_ERROR_BCAN_PASSERR:
		notif_category = LLCE_NOTIFCAT_CAN_PROTOCOL_CRITICAL_STATE;
		break;

	/* Initialization errors. This category contains errors caused by bad
	 * usage or a malicious host (eg initialization errors, bad cmd
	 * parameters, invalid indexes etc).
	 */
	case LLCE_ERROR_COMMAND_NOTSUPPORTED:
	case LLCE_ERROR_COMMAND_NOTACCEPTED:
	case LLCE_ERROR_COMMAND_INVALID_PARAMS:
	case LLCE_ERROR_RXTOKENS_UNRETURNED:
	case LLCE_ERROR_FILTERS_NOTEXIST:
	case LLCE_ERROR_FILTERS_FULL:
	case LLCE_ERROR_CMD_PROCESSING:
	case LLCE_ERROR_TXACK_NOT_READ:
	case LLCE_ERROR_COMMAND_DEINIT_NOTSTOP:
		notif_category = LLCE_NOTIFCAT_CONFIGURATION;
		break;

		/* Values that are neither errors nor notifications.
		 */
	case LLCE_FW_SUCCESS:
	case LLCE_FW_ERROR:
	case LLCE_FW_NOTRUN:
		notif_category = LLCE_NOTIFCAT_FW_STATUS;
		break;

	default:
		notif_category = LLCE_NOTIFCAT_INTERNAL;
		break;
	}

	return notif_category;
}

#endif /* LLCE_FW_INTERFACE_H */
