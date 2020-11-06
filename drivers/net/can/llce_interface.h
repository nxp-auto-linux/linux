/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020 NXP
 *
 * Driver for the NXP Semiconductors LLCE engine logging of CAN messages.
 * The LLCE can be found on S32G2xx.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef LLCE_INTERFACE_H
#define LLCE_INTERFACE_H

/* Maximum buffer size used to store the CAN FD frame payload. See
 * llce_rx_mb and llce_tx_mb
 */
#define LLCE_CAN_CONFIG_PAYLOAD_MAX_SIZE        64
/* Maximum number of standard filters which can be configured using
 * a single command.Multiple commands can be executed when more filters
 * are needed. See llce_can_receive_filter
 */
#define LLCE_CAN_CONFIG_MAX_FILTERS_COUNT       ((uint16_t)20)
/* Maximum number of hardware controllers usable inside LLCE. See
 * llce_can_init_cmd
 */
#define LLCE_CAN_CONFIG_MAXCTRL_COUNT           16U
/* Maximum number of advanced filters which can be configured using
 * a single command.Multiple commands can be executed when more
 * filters are needed. See llce_can_advanced_filter
 */
#define LLCE_CAN_CONFIG_ADVANCED_FILTERS_COUNT  8U
#define LLCE_VERSION_MAX_LENGTH					50
/* Maximum number of notifications which can be reported by LLCE to host.
 */
#define LLCE_CAN_CONFIG_NOTIF_TABLE_SIZE		16
/* Number of entries of the circular buffer used to send ack information
 * from Tx core to host core.
 */
#define LLCE_CAN_CONFIG_TXACKINFO_LOCATIONS     17

/** @brief    CAN message buffer.
 *  @details  CAN message buffer is a memory area placed in the
 *		 shared memory which is used by the LLCE firmware to
 *		 receive/transmit from/to BCAN controller.
 *
 *  LLCE firmware transmit/receive the frame in a word by word way
 *	so the content of the 4 structure fields contains the frame fields
 *	as they are described in the picture from below.
 *  For the reception process the LLCE firmware store inside message buffer
 *	frame the time stamp read from the hardware CAN controller.
 *
 *  @pre	Before to use any message buffer it is needed to initialize,
 *		 configure and start a CAN controller
 **/
struct llce_can_mb {
/* @brief   INPUT/OUTPUT: The first word of a frame as it is
 *				expected/provided by the CAN controller.
 */
	u32 u32_word_0;
/* @brief   INPUT/OUTPUT: The second word of a frame as it is
 *				expected/provided by the CAN controller.
 */
	u32 u32_word_1;
/* @brief   INPUT/OUTPUT: Frame payload needed for the maximum
 *				payload size case.
 */
	u8	payload[LLCE_CAN_CONFIG_PAYLOAD_MAX_SIZE];
/* @brief   INPUT: Time stamp of the received frames. It is not
 *				used for the transmitted frames.
 */
	u32 u32_tstamp;
};

/** @brief    Acknowledge transmission information send from LLCE to host.
 *  @details  It is used in order to send from LLCE to host needed
 *			information in order to identify and confirm that a
 *			specific frame was transmitted on the CAN bus. This
 *			data structure type is used in order to implement a
 *			circular buffer for each channel  which is accessed
 *			by using indexes transferred from LLCE to host by
 *			using TXACK FIFOs.
 *          This approach allows usage of existing hardware FIFOs
 *			even the size of the transferred data is higher than
 *			the FIFO element width size.
 *
 *  @pre      Before to read any acknowledge information, it is needed to
 *			do a transmission request.
 **/
struct llce_can_tx2hostackinfo {
/** @brief   OUTPUT: Host defined tag used to track a specific
 *			frame. This field is not changed by the LLCE firmware
 *			and is returned back to the host as it is.
 */
	u16 u16_frame_tag1;
/** @brief   OUTPUT: Host defined tag used to track a specific frame.
 *			This field is not changed by the LLCE firmware and is
 *			returned back to the host as it is.
 */
	u16 u16_frame_tag2;
/** @brief   OUTPUT: Transmission time stamp.
 */
	u32 u32_tx_timestamp;
};

/**
 * @brief		  CAN firmware error values.
 * @details        CAN error values as they are reported by the LLCE
 *			firmware. Some of them are channel related and other are
 *			platform related.
 */
enum llce_can_error {
	LLCE_ERROR_TXACK_FIFO_FULL = 1U,	/** @brief FIFO is full. */
	LLCE_ERROR_RXOUT_FIFO_FULL,    /** @brief FIFO is full. */
	LLCE_ERROR_FIFO_EMPTY,              /** @brief FIFO is empty. */
	LLCE_ERROR_MB_NOTAVAILABLE,
	/** @brief Message buffer is not avaialable. */
	LLCE_ERROR_BCAN_FRZ_EXIT,
	/** @brief CAN protocol error due to inability to get out from
	 * the freeze mode.
	 */
	LLCE_ERROR_BCAN_SYNC,
	/** @brief CAN protocol error due to inability to synchronize on
	 * the bus.
	 */
	LLCE_ERROR_BCAN_FRZ_ENTER,
	/** @brief CAN protocol error due to inability to enter in
	 * freeze mode.
	 */
	LLCE_ERROR_BCAN_LPM_EXIT,
	/**< @brief CAN protocol error due to inability to enter in
	 * low-power mode.
	 */
	LLCE_ERROR_BCAN_SRT_ENTER,
	/** @brief CAN protocol error due to inability to enter in
	 * soft reset.
	 */
	LLCE_ERROR_DATA_LOST,
	/** @brief Reports data lost event due to resources exceeded
	 * after the frame was received
	 */
	LLCE_ERROR_TXLUT_FULL,
	/** @brief TXLUT acclerator is full. */
	LLCE_ERROR_CMD_PROCESSING,
	/** @brief Error during command processing. */
	LLCE_ERROR_RXLUT_SLOW_SEARCH,
	/** @brief Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_ACCESS_MODE,
	/** @brief Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_SEARCH_MODE,
	/** @brief Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_SLOW_OPERATION,
	/** @brief Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_INCOMPLETE_OP,
	/** @brief Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_OPERATING_MODE,
	/** @brief Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_INIT_SLOW_OP,
	/** @brief Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_DEINIT_SLOW_OP,
	/** @brief Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_INIT_OPERATING_MODE,
	/** @brief Error regarding RXLUT hardware. */
	LLCE_ERROR_RXLUT_DEINIT_OPERATING_MODE1,
	/** @brief Error regarding RXLUT hardware. */
	LCE_ERROR_RXLUT_DEINIT_OPERATING_MODE2,
	/** @brief Error regarding RXLUT hardware. */
	LLCE_ERROR_HARDWARE_BUSOFF,
	/** @brief Error regarding bus off event. */
	LLCE_ERROR_CTRL_NOT_READY,
	/** @brief Controller is not ready. */
	LLCE_ERROR_BUSOFF,
	/** @brief Error regarding bus off. */
	LLCE_ERROR_FIFO_LOG_FULL,
	/** @brief Logging fifo is full. */
	LLCE_ERROR_CAN2CAN,
	/** @brief Error reported due to CAN2CAN routing error. */
	LLCE_ERROR_COMMAND_PARAM,
	/** @brief Error reported due to wrong command parameters
	 * received from host.
	 */
	LLCE_ERROR_COMMAND_RXPPE_NOTRESPONSE,
	/** @brief Error reported due to the rx core not responding. */
	LLCE_ERROR_COMMAND_DEINIT_NOTSTOP,
	/** @brief Error reported because the controller is not stopped. */
	LLCE_ERROR_COMMAND_NOTSUPPORTED,
	/** @brief Error reported because the requested command
	 * is not supported.
	 */
	LLCE_ERROR_DEINIT_COMMAND_NOTSUPPORTED,
	/** @brief Error reported because deinit command is not supported. */
	LLCE_ERROR_COMMAND_NOTVALIDATED,
	/** @brief Error reported because command is not validated by
	 *	the command flow.
	 */
	LLCE_ERROR_COMMAND_NOTACCEPTED,
	/** @brief Error reported because the requested command is
	 * correct but it not accepted.
	 */
	LLCE_ERROR_COMMAND_INVALID_PARAMS,
	/** @brief Error reported because the requested command
	 * parameters are invalid.
	 */
	LLCE_ERROR_CTRL_NOT_STARTED, /**< @brief Controller is not started. */
	LLCE_ERROR_FRAME_NOT_DELIVERED,
	/** @brief Reports frame accepted, but not delivered to
	 * host because of filters misconfiguration.
	 */
	LLCE_ERROR_FRAME_NOT_DELIVERED_TO_AF,
	/** @brief Reports frame accepted, but not delivered to
	 * AF destination because of full fifo.
	 */
	LLCE_ERROR_FRAME_NOT_DELIVERED_TO_HOST,
	/** @brief Reports frame accepted, but not delivered to
	 * host due to lack of descriptors in sw fifo.
	 */
	LLCE_ERROR_LOST_INDEXES
	/** @brief Reports detection of lost indexes in RX-DTE subsystem . */
};

/**
 * @brief          CAN firmware components IDs.
 * @details        CAN firmware components IDs used to identify the
 *				component which generated a specific error.
 **/
enum llce_can_module {
	LLCE_TX = 101U, /** @brief CAN TX firmware component. */
	LLCE_RX,        /** @brief CAN RX firmware component. */
	LLCE_DTE        /** @brief CAN DTE firmware component. */
};

struct llce_can_errornotif {
	/** @brief   OUTPUT: LLCE firmware component id.
	 *				See also llce_can_module.
	 */
	enum llce_can_module e_module_id;
    /** @brief   OUTPUT: LLCE error code.See also llce_can_error. */
	enum llce_can_error e_error_code;
};

/** @brief    Channel specific error.
 *  @details  Channel error details send to host in order to report
 *				an internal LLCE error.
 **/
struct llce_can_channelerrornotif {
	struct llce_can_errornotif	error_info;
	/** @brief   OUTPUT: Controller ID. */
	u8	u8_hw_ctrl;
};

/**
 * @brief          CAN controller states.
 * @details        CAN controller states as they are reported by the
 *			LLCE firmware as a result of state transition requests.
 **/
enum llce_can_ctrlstate {
	LLCE_CAN_UNINIT_CTRL  = 0U,
	/** @brief Controller is uninitialised (default) */
	LLCE_CAN_STOP_PENDING,
	/** @brief Controller is stopping, but not offline yet */
	LLCE_CAN_STOPPED,
	/** @brief Controller is in the STOPPED state which means that
	 * it does not do any bus transactions.
	 */
	LLCE_CAN_START_PENDING,
	/** @brief Controller is starting, but cannot do bus
	 * transactions yet.
	 */
	LLCE_CAN_STARTED,
	/** @brief Controller is in the STARTED state which
	 * means that it do bus transactions.
	 */
	LLCE_CAN_IDLE,
	/** @brief Controller is in the IDLE state. This state
	 * is not used by LLCE module.
	 */
	LLCE_CAN_UNINIT_PLATFORM
	/** @brief Controller is in state when the common components
	 * of the platfor are not initialized.
	 */
};

/** @brief    Set controller mode notification.
 *  @details  Notification send from LLCe to host in order to
 *		inform about a specific controller state change.
 **/
struct llce_can_ctrlmodenotif {
	/** @brief   OUTPUT: Current state of the CAN controller.
	 *		See also llce_can_ctrlstate.
	 */
	enum llce_can_ctrlstate    e_controller_state;
    /** @brief   OUTPUT: CAN controller id. */
	u8					 u8_hw_ctrl;
};

/** @brief    List of notifications send by LLCe to host. used by host.
 *  @details  It is used by LLCE to notify host about specific events
 *			inside LLCE.
 **/
union llce_can_notificationlist {
	/** @brief   OUTPUT: Notification parameters for controller state
	 *			changes. See also llce_can_ctrlmodenotif.
	 */
	struct llce_can_ctrlmodenotif       ctrl_mode;
	/** @brief   OUTPUT: Notification parameters for platform errors.
	 *		See also llce_can_errornotif
	 */
	struct llce_can_errornotif	        platform_error;
	/** @brief   OUTPUT: Notification parameters for channel errors.
	 *		See also llce_can_channelerrornotif
	 */
	struct llce_can_channelerrornotif	channel_error;
};

/** @brief    Notification IDs used to interface with LLCE.
 *  @details  Notifications send by LLCE to host core.
 **/
enum llce_can_notificationid {
	LLCE_CAN_NOTIF_PLATFORMERROR = 0UL,
	/** @brief Error related to the common platform area. */
	LLCE_CAN_NOTIF_CHANNELERROR,
	/** @brief Error related to a specific channel.*/
	LLCE_CAN_NOTIF_CTRLMODE,
	/** @brief Notification related to changing CAN controller mode.*/
};

/** @brief    Notifications used by LLCE.
 *  @details  It is used in order to send notifications from LLCE to
 *					host by using shared memory.
 **/
struct llce_can_notification {
	/** @brief   OUTPUT: Notification ID.
	 * See also llce_can_notificationid
	 */
	enum llce_can_notificationid e_notif_id;
	/** @brief   OUTPUT: Notification parameters.
	 * See also llce_can_notificationlist
	 */
	union llce_can_notificationlist notif_list;
};

/** @brief    Notification tables.
 *  @details  Notification tables used to store the details of
 *				the notifications.The index of entries are send
 *				to host cores.The two tables are related to
 *				reporting method:interrupt or polling.
 **/
struct llce_can_notification_table {
	/** @brief   OUTPUT: Table used to report notifications in
	 *				interrupt mode.
	 *				See also llce_can_notification.
	 */
	struct llce_can_notification
		can_notif_intr_table[LLCE_CAN_CONFIG_NOTIF_TABLE_SIZE];
	/** @brief   OUTPUT: Table used to report notifications in
	 *				polling mode.
	 *				See also llce_can_notification
	 */
	struct llce_can_notification
		can_notif_poll_table[LLCE_CAN_CONFIG_NOTIF_TABLE_SIZE];
};

/** @brief    LLCE-PFE initialization command.
 *  @details  It is sent by the host to LLCE in order to
 *				activate the LLCE-PFE interface, and obtain the
 *				addresses of the relevant data structures.
 **/
struct llce_can_init_pfe_cmd {
	/** @brief   OUTPUT: Address of the PFE RX Ring in LLCE memory */
	void *p_rx_ring;
	/** @brief   OUTPUT: Address of the PFE RX Writeback Ring
	 *				 in LLCE memory
	 */
	void *p_rx_wb_ring;
	/** @brief   OUTPUT: Address of the PFE TX Ring in LLCE memory */
	void *p_tx_ring;
	/** @brief   OUTPUT: Address of the PFE TX Writeback Ring
	 *				in LLCE memory
	 */
	void *p_tx_wb_ring;
};

/** @brief    Platform initialization command.
 *  @details  It is send by the host to LLCE in order to configure
 *			the platform reletaed parameters.
 *            It is the first command which shall be send to LLCE
 *			module after booting in order to configure common
 *			LLCE components.
 *            Most important operations done by LLCE firmware at
 *			platform initialization are:
 *          - initialize also all common internal LLCE components.
 *          - prepare filter management data structures
 **/
struct llce_can_init_platform_cmd {
	/** @brief   INPUT: Configuration options for a hardware CAN
	 *			controller.
	 *			See LLCE_CAN_CONTROLLERCONFIG_CTRL_EN_U32
	 *			as a controller  option example
	 */
	u16 max_filter_count[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
};

/** @brief    Get firmware version command.
 *  @details  It is send by the host to LLCE in order to get the
 *				firmware version string. It is copied in the
 *				response.
 **/
struct llce_can_get_fw_version {
	/** @brief   OUTPUT: LLCE FW version string actual length. */
	u8 string_length;
	/** @brief   OUTPUT: LLCE FW version string. */
	u8 version_string[LLCE_VERSION_MAX_LENGTH];
};

/** @brief   RXLUT entries type
 *  @details  Specifies the type of entry in the table.
 **/
enum llce_can_entry {
	LLCE_CAN_ENTRY_EXACT_MATCH = 0UL,
	/** @brief Exact match entry type. */
	LLCE_CAN_ENTRY_CFG_MASKED,
	/** @brief Masked match entry type. */
	LLCE_CAN_ENTRY_CFG_RANGED,
	/** @brief Range match entry type. */
};

/** @brief    CAN to Ethernet routing filter configuration.
 *  @details  It is used to define a specific routing filter.
 *			Current routing implementation suppose to accept
 *			a received frame for a specific
 *          ID and ID mask combination, then route that frame
 *			to an Ethernet interface.
 **/
struct llce_can_can2eth_routing_table {
	/** @brief   INPUT: CAN routing table index. */
	u8 can2eth_routing_table_idx;
	/** @brief   INPUT: Ethernet MAC destination address.*/
	u8 can2eth_dest_mac[6];
} __aligned(4);

/** @brief    CAN to CAN routing filter configuration.
 *  @details  It is used to define a specific routing filter.
 *			Current routing implementation suppose to accept a
 *			received frame for a specific ID and ID mask combination
 *			then route that frame to one or more transmission
 *			channels.
 **/
struct llce_can_can2can_routing_table {
	/** @brief   INPUT: CAN routing table index. */
	u8 can2can_routing_table_idx;
	/** @brief   INPUT: List of destination channels for the
	 *			accepted frame.
	 */
	u8 dest_hw_ch_list[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	/** @brief   INPUT: Length of the destination channel list.*/
	u8 dest_hw_ch_list_count;
	/** @brief   INPUT: Special option for advanced routing.*/
	u32 can2can_routing_options;
	/** @brief   INPUT: Can Id Remap Value.*/
	u32 can_id_remap_value;
} __aligned(4);

/**
 * @brief          CAN Logging options.
 * @details        CAN options for logging frames feature.
 **/
enum llce_af_logging_options {
	LLCE_AF_LOGGING_DISABLED = 1U,
	/** @brief Logging of CAN frame si disabled.*/
	LLCE_AF_LOGGING_ENABLED
	/** @brief Logging of CAN frame si enabled.*/
};

enum llce_can_host_receive_options {
	LLCE_AF_HOSTRECEIVE_DISABLED = 1U,
	/** @brief Logging of CAN frame si disabled.*/
	LLCE_AF_HOSTRECEIVE_ENABLED
	/** @brief Logging of CAN frame si enabled.*/
};

/** @brief    Advanced filter configuration.
 *  @details  It is used to define an advanced filter.
 *		It contains references to the individual features
 *		configured by the host.
 **/
struct llce_can_advanced_feature {
	/** @brief   INPUT: CAN advanced feature index. */
	u8 can_advanced_feature_idx;
	/** @brief   INPUT: Option for host receive feature. */
	enum llce_can_host_receive_options host_receive;
	/** @brief   INPUT: Option for logging feature. */
	enum llce_af_logging_options can_logging_feature;
	/** @brief   INPUT: CAN2CAN routing table index. Reference to
	 *		the routing table rule.
	 *		See llce_can_can2can_routing_table
	 */
	u8 can2can_routing_table_idx;
	/** @brief  INPUT: CAN2ETH routing table index. Reference to
	 *		the routing table rule.
	 *		See llce_can_can2eth_routing_table
	 */
	u8 can2eth_routing_table_idx;
} __aligned(4);

/** @brief    CAN frame ID type.
 *  @details  It specify the CAN frame ID type based on it's
 *			length as it is defined by CAN specification.
 **/
enum llce_can_id_length {
	LLCE_CAN_EXTENDED = 0U,          /**< @brief Extended ID (29 bits) */
	LLCE_CAN_STANDARD,               /**< @brief Standard ID (11 bits) */
	LLCE_CAN_MIXED                   /**< @brief Mixed ID (29 bits) */
};

/** @brief    Advanced filter element configuration.
 *  @details  It is used to define a specific filter. Current
 *		filtering suppose to accept a frame for processing if
 *      it's frame ID match the filter ID masked with the mask
 *		value. At the end of filtering process the frame
 *      is processed according to the advanced configuration
 *		of the filter.
 **/
struct llce_can_advanced_filter {
	/** @brief   INPUT: Frame id mask value. Bit fields containing
	 *		 \b 0 means don't care.
	 */
	u32 id_mask;
	/** @brief   INPUT: CAN frame ID type. */
	enum llce_can_id_length id;
	/** @brief   INPUT: CAN frame ID value. */
	u32 message_id;
	/** @brief  INPUT: Filter identifier used to track frames
	 *		after filtering process on the reception side.
	 *		See also llce_can_rx_mb_descriptor
	 */
	u16 filter_id;
	/** @brief  INPUT: Maximum number of message buffers which
	 *		can be used to store frames accepted by this filter
	 *		at each specific point in time.
	 *		It is used in order to prevent that the frames accepted
	 *		by a specific filter do not overload the LLCE internal
	 *		hardware resources (e.g. message buffers, FIFOs).
	 */
	u16 mb_count;
	/** @brief   INPUT: Can advanced features used by the filter. */
	struct llce_can_advanced_feature
		llce_can_advanced_feature;
	/** @brief   INPUT: CAN2CAN routing rules used by the filter. */
	struct llce_can_can2can_routing_table
		llce_can_can2can_routing_table;
	/** @brief   INPUT: CAN2ETH routing rules used by the filter. */
	struct llce_can_can2eth_routing_table
		llce_can_can2eth_routing_table;
	/** @brief   INPUT: Filter entry type. */
	enum llce_can_entry entry;
} __aligned(4);

/** @brief    Set advanced filter command.
 *  @details  It is send by the host to LLCE in order to set one
 *			or more advanced filters.
 **/
struct llce_can_set_advanced_filter_cmd {
	/** @brief   INPUT: Number of configured filters. */
	u16 rx_filters_count;
	/** @brief  INPUT: Array containing configuration for one
	 *		or more filters.
	 */
	struct llce_can_advanced_filter
		advanced_filters[LLCE_CAN_CONFIG_ADVANCED_FILTERS_COUNT];
};

/** @brief    Requested transitions of a CAN controller.
 *  @details  Those controller state transitions are requested by
 *			the host in a specific order.
 **/
enum llce_can_state_transition {
		LLCE_CAN_T_STOP = 0U,
		/** @brief Request transition from
		 *	START state into STOP state.
		 */
		LLCE_CAN_T_START,
		/** @brief Request transition from
		 *	STOP state into START state.
		 */
};

/** @brief    Set controller mode command.
 *  @details  It is send from host to LLCE module in order request
 *		changing the state of a CAN controller.
 *      Currently it allows only to start and stop a controller.
 *      When a controller is started it allows to transmit and
 *		receive frames from the bus.
 *      When the controller is stopped it ignores all frames from
 *		the bus and it doesn't transmit any frame.
 *
 *  @pre      Before changing the controller state it must be
 *		initialized.
 **/
struct llce_can_set_controller_mode_cmd {
	/** @brief   INPUT: The new state which is requested. */
	enum llce_can_state_transition transition;
};

/** @brief    Command for polling of controller state .
 *  @details  It is send from host to LLCE to query it for the
 *		controller state.
 **/
struct llce_can_get_controller_mode_cmd {
	/** @brief   OUTPUT: Current state of the CAN controller. */
	enum llce_can_ctrlstate e_controller_state;
};

/** @brief    Data baud rate settings for a CAN FD controller.
 *  @details  It is used to configure the CAN FD settings including
 *		baudrate used during data phase.
 **/
struct llce_can_controller_fd_config {
	/** @brief   INPUT: Enable or disable FD related features of
	 *		the CAN controller.
	 */
	unsigned char can_fd_enable;
	/** @brief   INPUT: Configuration of data phase baud rate:
	 *	prescaler divisor(bit23-27), Resynchronization Jump
	 *	Width(bit16-19),Time Segment2(bit9-12), Time Segment1(bit0-4).
	 *	Each parameter value shall be decreased by 1 when it is
	 *	written into this data structure field.
	 */
	u32 can_data_baudrate_config;
	/** @brief   INPUT: Enable or disable baud rate switch(BRS) at
	 *		the level of CAN controller.
	 */
	unsigned char can_controller_tx_bit_rate_switch;
	/** @brief   INPUT: Enable or disable Transceiver Delay
	 *		Compensation: TRUE=enabled, FALSE=disabled.
	 */
	unsigned char can_trcv_delay_comp_enable;
	/** @brief   INPUT: Enable or disable Transceiver Delay
	 *		Measurement: TRUE=enabled, FALSE=disabled. When it is
	 * enabled, the secondary sample point  is determined by the
	 * sum of the tranceiver delay measurement plus transceiver
	 * delay compensation offset.
	 * When it is disabled, the secondary sample point is
	 * determined only by the transceiver delay compensation
	 * offset.
	 */
	unsigned char can_trcv_delay_meas_enable;
	/** @brief   INPUT: Value of Transceiver Delay Compensation
	 * Offset
	 */
	u8 can_trcv_delay_comp_offset;
};

/** @brief    Set baud rate command.
 *  @details  It is send from host to LLCE module in order to
 *		configure baud rate parameters for arbitration phase.
 **/
struct llce_can_set_baudrate_cmd {
	/** @brief   INPUT: Configuration parameters for nominal
	 *		baud rate:prescaler divisor(bit23-31),
	 *		Resynchronization Jump Width(bit16-22),
	 *		Time Segment2(bit9-15),
	 *		Time Segment1(bit0-7).
	 * Each parameter value shall be decreased by 1 when it is
	 *		written into this data structure field.
	 */
	u32 nominal_baudrate_config;
	/** @brief   INPUT: Configuration parameters for data baud
	 *		rate of the CAN controller.
	 */
	struct llce_can_controller_fd_config controller_fd;
};

/** @brief    Filter element settings.
 *  @details  It is used to define a specific filter.Current
 *		filtering process suppose to accept a frame if it's frame ID
 *		match the filter ID masked with the mask value. At the end
 *		of filtering process an internal filter ID is mapped to the
 *		accepted frame in order to track it later by the host
 *		software. A maximum number of frames accepted by a specific
 *		filter can be managed by LLCE at each point in time.
 **/
struct llce_can_receive_filter {
	/** @brief   INPUT: Frame id mask value. Bit fields containing
	 *			\b 0 means don't care.
	 */
	u32 id_mask;
	/** @brief   INPUT: CAN frame ID type. */
	enum llce_can_id_length id;
	/** @brief   INPUT: CAN frame ID value. */
	u32 message_id;
	/** @brief  INPUT: Filter identifier used to track frames
	 *		after filtering process on the reception side. See
	 *		also llce_can_rx_mb_descriptor
	 */
	u16 filter_id;
	/** @brief  INPUT: Maximum number of message buffers which
	 *	can be used to store frames accepted by this filter at
	 *	each specific point in time.
	 *	It is used in order to prevent that the frames accepted
	 *	by a specific filter do not overload the LLCE internal
	 *	hardware resources (e.g. message buffers, FIFOs).
	 */
	u16 mb_count;
	/** @brief   INPUT: Entry type. */
	enum llce_can_entry entry;
};

/** @brief    Set filter command.
 *  @details  It is send by the host to LLCE in order to configure
 *		one or more reception filters inside LLCE.
 **/
struct llce_can_set_filter_cmd {
	/** @brief   INPUT: Number of configured filters. */
	u16 rx_filters_count;
	/** @brief  INPUT: Array containing configuration for
	 *		reception filters.
	 */
	struct llce_can_receive_filter
		rx_filters[LLCE_CAN_CONFIG_MAX_FILTERS_COUNT];
};

/** @brief    Get status command.
 *  @details  It is send by the host to LLCE in order to get the
 *		content of all status registers of a specific CAN controller.
 *		This command makes only a read operation on the status
 *			registers of CAN controller.
 **/
struct llce_can_get_status_cmd {
	/** @brief   OUTPUT: Register ECR of CAN controller. */
	u32 u32ECR;
	/** @brief   OUTPUT: Register ISR of CAN controller. */
	u32 u32ISR;
	/** @brief   OUTPUT: Register SR of CAN controller. */
	u32 u32SR;
	/** @brief   OUTPUT: Register CRC of CAN controller. */
	u32 u32CRC;
};

/** @brief    initialization command.
 *  @details  It is send by the host to LLCE in order to load and
 *		configure all needed parameters inside LLCE.
 *      It is the first command which shall be send to LLCE module
 *		after booting.
 *      Most important operations done by LLCE firmware at
 *		initialization are:
 *          - when this command is executed for the first CAN
 *				channel it initialize also all internal LLCE
 *				components.
 *          - configure the hardware features of a CAN controller.
 *          - set a CAN controller in the stop state.
 **/
struct llce_can_init_cmd {
	/** @brief   INPUT: Configuration options for a hardware CAN
	 *		controller. See LLCE_CAN_CONTROLLERCONFIG_CTRL_EN_U32
	 *		as a controller  option example
	 */
	u32 ctrl_config;
	/** @brief   INPUT: Number of transmission message buffer
	 *		descriptors used for transmissions initialized by
	 *		the host.The remaining elements are used for internal
	 *		routing scenarios.
	 */
	u8 tx_mb_count;
};

/** @brief    List of commands used by host.
 *  @details  It is used in order to use the same memory area for all
 *		commands send from host to LLCE.
 **/
union llce_can_command_list {
	struct llce_can_init_cmd				init;
	/** @brief   Command for initializing a specific CAN channel */
	struct llce_can_get_status_cmd			get_status;
	/** @brief   Command for getting the hardware status
	 *		information for a specific CAN controller.
	 */
	struct llce_can_set_filter_cmd         set_filter;
	/** @brief   Command for configuring filters for a
	 *		specific CAN controller in order to deliver
	 *		frames to the host.
	 */
	struct llce_can_set_baudrate_cmd       set_baudrate;
	/** @brief   Command for configuring baud rate parameters
	 *		for a specific CAN controller.
	 */
	struct llce_can_get_controller_mode_cmd get_controller_mode;
	/** @brief   Command for getting the status of a specific
	 *		CAN controller.
	 */
	struct llce_can_set_controller_mode_cmd set_controller_mode;
	/** @brief   Command for changing the status of a specific
	 *		CAN controller.
	 */
	struct llce_can_set_advanced_filter_cmd set_advanced_filter;
	/** @brief   Command for configuring filters in order to
	 *		route frames to other destinations than host.
	 */
	struct llce_can_get_fw_version			get_fw_version;
	/** @brief   Command for getting the firmware version. */
	struct llce_can_init_platform_cmd      init_platform;
	/** @brief   Command for configuring platform related
	 *		parameters and common HW components used by all
	 *		CAN channels
	 */
	struct llce_can_init_pfe_cmd           init_pfe;
	/** @brief   Command for initializing the LLCE-PFE
	 *		interface and getting buffer locations
	 */
};

/** @brief    Return status codes reported at the end of each
 *		command execution.
 */
enum llce_can_return {
	LLCE_CAN_SUCCESS = 0x55,
	/** @brief Command was executed successfully. */
	LLCE_CAN_ERROR,
	/** @brief During command execution it was detected an
	 *	error condition.
	 */
	LLCE_CAN_NOTRUN
	/** @brief Command default value set by the host before
	 *	to send it to LLCE.
	 */
};

/** @brief    Command IDs used to interface with LLCE.
 *  @details  Some of those commands are send by the host to
 *		LLCE module and others are send by LLCE module to the host.
 **/
enum llce_can_command_id {
	LLCE_CAN_CMD_INIT = 0UL,
	/** @brief Host initializes LLCE module. */
	LLCE_CAN_CMD_DEINIT,
	/** @brief Host Deinitialize a specific CAN controller. */
	LLCE_CAN_CMD_SETBAUDRATE,
	/** @brief Host sets a baud rate for a specific
	 *		CAN controller.
	 */
	LLCE_CAN_CMD_GETCONTROLLERMODE,
	/** @brief Host checks the state for a specific
	 *		CAN controller.
	 */
	LLCE_CAN_CMD_SETCONTROLLERMODE,
	/** @brief Host changes the state for a specific
	 *		CAN controller.
	 */
	LLCE_CAN_CMD_BUSOFF_CONFIRMATION,
	/** @brief LLCE notify host about bus off event for
	 *		a specific CAN controller.
	 */
	LLCE_CAN_CMD_GETSTATUS,
	/** @brief LLCE deliver to the host the content of
	 *		all status register of CAN controller.
	 */
	LLCE_CAN_CMD_SETFILTER,
	/** @brief Host configure multiple filters on the
	 *		reception side.
	 */
	LLCE_CAN_CMD_SETADVANCEDFILTER,
	/** @brief Host configure multiple advanced feature
	 *		filters on the reception side.
	 */
	LLCE_CAN_CMD_GETFWVERSION,
	/** @brief Request version string from FW.*/
	LLCE_CAN_CMD_INIT_PLATFORM,
	/** @brief Host request for platform initialization.*/
	LLCE_CAN_CMD_DEINIT_PLATFORM,
	/** @brief Host request for platform deinitialization.*/
	LLCE_CAN_CMD_INIT_PFE
	/** @brief Host request for platform initialization
	 *		regarding CAN2ETH use case.
	 */
};

/** @brief    Command used by host.
 *  @details  It is used in order to send commands
 *			from host to LLCe using shared memory.
 **/
struct llce_can_command {
	/** @brief   INPUT: Command ID. */
	enum llce_can_command_id cmd_id;
	/** @brief   OUTPUT: Return status code after command completion.*/
	enum llce_can_return return_value;
	/** @brief   INPUT: Command parameters. */
	union llce_can_command_list cmd_list;
};

/** @brief    Transmission message buffer descriptor.
 *  @details  Transmission message buffer descriptor is a memory
 *		area placed in the shared memory which is written by the
 *		host software with other additional info (e.g. frame tag
 *		IDs) which is send back to the host by the LLCE firmware
 *		as acknowledge information.
 *            Those internal tags are not changed/used by the LLCE firmware.
 *
 *  @pre      Before to use any transmission message buffer
 *		descriptor it is needed to initialize, configure and
 *		start a CAN controller.
 **/
struct llce_can_tx_mb_descriptor {
	/** @brief   INPUT: Host defined tag used to track a specific
	 *		frame. This field is not changed by the LLCE firmware
	 *		and is returned back to the host as it is.
	 *		See llce_can_tx2hostackinfo
	 */
	u16 u16_frame_tag1;
	/** @brief   INPUT: Host defined tag used to track a specific
	 *		frame. This field is not changed by the LLCE firmware
	 *		and is returned back to the host as it is.
	 *		See llce_can_tx2hostackinfo
	 */
	u16 u16_frame_tag2;
	/** @brief   OUTPUT: Index to the frame message buffer.
	 *		See llce_can_mbtype
	 */
	u8 u8_ack_interface;
	u16 u16mb_frame_idx;
	unsigned char enable_tx_frame_max;
};

/** @brief    Reception message buffer descriptor.
 *  @details  Reception message buffer descriptor is a memory area
 *		placed in the shared memory which is written by the LLCE
 *		firmware with the specific runtime info needed by the host
 *		software.(e.g.matching filter ID ). Also it includes an
 *		index to a CAN message buffer allocated during initialization
 *		to each descriptor.
 *      After reception, the host shall copy the content of the
 *		reception message buffer descriptor and the referred message
 *		buffer by this descriptor from the shared memory into the
 *		host memory in order to be processed later by the host
 *		software and to allow the current message buffer descriptor
 *		to be used by LLCE firmware for the reception of a new frame.
 *
 *  @pre	 Before to use any receive message buffer descriptor it
 *		is needed to initialize, configure and start a CAN controller
 **/
struct llce_can_rx_mb_descriptor {
	/** @brief   OUTPUT: Filter identifier resulted at the end of
	 *		filtering process.
	 *	This field is completed by the LLCE filtering mechanism with
	 *	a value which was configured during initialization time.
	 *	It is used in order to map a received frame to a specific
	 *	filter defined by the host.
	 */
	u16 filter_id;
	/** @brief   OUTPUT: Index to the CAN message buffer.
	 *	See llce_can_mbtype
	 */
	u16 u16mb_frame_idx;
};

/** @brief    Shared memory structure
 *  @details  Structure that encapsulates all the shared memory
 *		with llce on the can side.
 **/
struct llce_can_shared_memory {
	/** @brief Receive message buffer descriptors. */
	struct llce_can_rx_mb_descriptor
		can_rx_mb_desc[LLCE_CAN_CONFIG_MAXRXMB];
	/** @brief Transmit message buffer descriptors. */
	struct llce_can_tx_mb_descriptor
		can_tx_mb_desc[LLCE_CAN_CONFIG_MAXTXMB];
	/** @brief Shared memory used store the CAN message buffers. */
	struct llce_can_mb
		can_mb[LLCE_CAN_CONFIG_TOTAL];
	/** @brief Shared memory used to send commands from Host to Llce . */
	struct llce_can_command
		can_cmd[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	/** @brief Shared memory used to store notifications from LLCE
	 *		to host.
	 */
	struct llce_can_notification_table
		can_notification_table;
	/** @brief Circular buffer used to send ack info from Tx core
	 *		to HOST core.
	 */
	struct llce_can_tx2hostackinfo
		can_tx_ack_info
			[LLCE_CAN_CONFIG_MAXCTRL_COUNT]
			[LLCE_CAN_CONFIG_TXACKINFO_LOCATIONS];
};

/* Logging structure
 */
struct frame_log {
	struct llce_can_mb frame;
	u8 u8_hw_ctrl;
};

#endif
