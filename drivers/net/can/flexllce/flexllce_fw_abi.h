/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 * FlexLLCE (RCU) firmware ABI - host-side mirror of the shared-memory structures.
 *
 * These structures mirror the FlexLLCE firmware Flexllce_InterfaceTypes.h layout
 * 1:1: field widths, packing and offsets are the firmware-imposed ABI and must
 * match the running firmware exactly.
 *
 * The host-side conventions built on top of this ABI - command/return/event
 * enumerations, bit-field masks, shared-memory offsets and helper macros - are
 * hand-maintained in flexllce_if.h, which includes this file.
 *
 * Copyright 2026 NXP
 */
#ifndef _FLEXLLCE_FW_ABI_H_
#define _FLEXLLCE_FW_ABI_H_

#include <linux/build_bug.h>
#include <linux/types.h>

enum flexllce_return {
	FLEXLLCE_NO_RETURN_VALUE = 0,
	FLEXLLCE_SUCCESS = 1,
	FLEXLLCE_ERROR = 2,
	FLEXLLCE_NOTRUN = 3,
	FLEXLLCE_ERROR_SW_FIFO_EMPTY = 4,
	FLEXLLCE_ERROR_SW_FIFO_FULL = 5,
	FLEXLLCE_ERROR_COMTABLE_EMPTY = 6,
	FLEXLLCE_ERROR_COMTABLE_FULL = 7,
} __packed;

enum flexllce_cmd_id {
	FLEXLLCE_CMD_INVALID = 0,
	FLEXLLCE_CMD_INIT = 1,
	FLEXLLCE_CMD_DEINIT = 2,
	FLEXLLCE_CMD_SETBAUDRATE = 3,
	FLEXLLCE_CMD_GETSTATUS = 4,
	FLEXLLCE_CMD_ABORTTRANSFER = 5,
	FLEXLLCE_CMD_ABORTALL = 6,
	FLEXLLCE_CMD_START = 7,
	FLEXLLCE_CMD_STOP = 8,
	FLEXLLCE_CMD_SETFILTERTABLE = 9,
	FLEXLLCE_CMD_SETCURRENTTIME = 10,
	FLEXLLCE_CMD_GETCURRENTTIME = 11,
	FLEXLLCE_CMD_INITROUTER = 12,
	FLEXLLCE_CMD_DEINITROUTER = 13,
	FLEXLLCE_CMD_LISTENONLYMODE = 14,
	FLEXLLCE_CMD_MANUAL_BUSOFF_RECOVERY = 15,
	FLEXLLCE_CMD_GET_STATISTICS = 16,
	FLEXLLCE_CMD_SET_MRU_ADDR = 17,
	FLEXLLCE_CMD_SET_NOTIFICATIONS = 18,
} __packed;

enum flexllce_mode {
	FLEXLLCE_NOTIF_ENABLE = 1,
	FLEXLLCE_NOTIF_DISABLE = 2,
} __packed;

enum flexllce_notif_processing {
	FLEXLLCE_NOTIF_IGNORE = 0,
	FLEXLLCE_NOTIF_USING_POLLING = 1,
	FLEXLLCE_NOTIF_USING_INTERRUPTS = 2,
} __packed;

enum flexllce_notif_category {
	FLEXLLCE_NOTIF_CAT_BUSOFF = 0,
	FLEXLLCE_NOTIF_CAT_CONTROLLER = 1,
	FLEXLLCE_NOTIF_CAT_GENERIC = 2,
} __packed;

enum flexllce_event_code {
	FLEXLLCE_EVENT_NO_ERROR = 0,
	FLEXLLCE_EVENT_BUSOFF = 1,
	FLEXLLCE_EVENT_RX_WARNING = 2,
	FLEXLLCE_EVENT_TX_WARNING = 3,
	FLEXLLCE_EVENT_ERROR = 4,
	FLEXLLCE_EVENT_ERROR_FAST = 5,
	FLEXLLCE_EVENT_RXFIFO_OVERFLOW = 6,
	FLEXLLCE_EVENT_IPCF_CMD_ERROR = 7,
	FLEXLLCE_EVENT_IPCF_RX_ERROR = 8,
	FLEXLLCE_EVENT_IPCF_TX_ERROR = 9,
	FLEXLLCE_EVENT_IPCF_TX_ACK_ERROR = 10,
	FLEXLLCE_EVENT_PASSIVE_STATE = 11,
	FLEXLLCE_EVENT_EXIT_PASSIVE_STATE = 12,
	FLEXLLCE_EVENT_COMMAND_NOT_SUPPORTED = 13,
	FLEXLLCE_EVENT_INVALID_DESTINATION = 14,
	FLEXLLCE_EVENT_NO_MB_AVAILABLE = 15,
	FLEXLLCE_EVENT_ROUTER_NOT_INITIALIZED = 16,
	FLEXLLCE_EVENT_INIT_FAILURE = 17,
	FLEXLLCE_EVENT_ETH_ALREADY_INITIALIZED = 18,
	FLEXLLCE_EVENT_ETH_NOT_INITIALIZED = 19,
	FLEXLLCE_EVENT_ETH_NOT_ENABLED = 20,
	FLEXLLCE_EVENT_ETH_START_FAILED = 21,
	FLEXLLCE_EVENT_ETH_STOP_FAILED = 22,
	FLEXLLCE_EVENT_ETH_SEND_FAILED_ERROR = 23,
	FLEXLLCE_EVENT_ETH_BUFFER_BUSY = 24,
	FLEXLLCE_EVENT_ETH_INVALID_BUFF_CFG_ID = 25,
	FLEXLLCE_EVENT_ETH_INVALID_MODE = 26,
	FLEXLLCE_EVENT_ETH_INVALID_ACF_MSG_TYPE = 27,
	FLEXLLCE_EVENT_ETH_WRONG_AVTP_TYPE = 28,
	FLEXLLCE_EVENT_ETH_WRONG_AVTP_LENGTH = 29,
	FLEXLLCE_EVENT_ETH_WRONG_ACF_LENGTH = 30,
	FLEXLLCE_EVENT_INVALID_PARAMS = 31,
	FLEXLLCE_EVENT_COMMAND_NOTACCEPTED = 32,
	FLEXLLCE_EVENT_LIN_TRANSMISSION_ERROR = 33,
	FLEXLLCE_EVENT_DMA_DBE = 34,
	FLEXLLCE_EVENT_DMA_SBE = 35,
	FLEXLLCE_EVENT_DMA_SGE = 36,
	FLEXLLCE_EVENT_DMA_NCE = 37,
	FLEXLLCE_EVENT_DMA_DOE = 38,
	FLEXLLCE_EVENT_DMA_DAE = 39,
	FLEXLLCE_EVENT_DMA_SOE = 40,
	FLEXLLCE_EVENT_DMA_SAE = 41,
	FLEXLLCE_EVENT_DMA_NA = 42,
	FLEXLLCE_EVENT_HIF_CUSTOM_SHARED_MEMORY_OVERLAP = 44,
} __packed;

enum flexllce_stats_module {
	FLEXLLCE_STATS_CAN = 0,
	FLEXLLCE_STATS_LIN = 1,
	FLEXLLCE_STATS_ETH = 2,
} __packed;

enum flexllce_filter_type {
	FLEXLLCE_FILTER_ONE_ID = 0,
	FLEXLLCE_FILTER_RANGE = 1,
	FLEXLLCE_FILTER_TWO_ID = 2,
};

enum flexllce_bool {
	FLEXLLCE_FALSE = 0,
	FLEXLLCE_TRUE = 1,
} __packed;

enum flexllce_timestamp_src {
	FLEXLLCE_CAN_CLK_TIMESTAMP_SRC = 0,
	FLEXLLCE_ONCHIP_CLK_TIMESTAMP_SRC = 1,
} __packed;

enum flexllce_msgbuff_timestamp {
	FLEXLLCE_MSGBUFFTIMESTAMP_TIMER = 0,
	FLEXLLCE_MSGBUFFTIMESTAMP_LOWER = 1,
	FLEXLLCE_MSGBUFFTIMESTAMP_UPPER = 2,
} __packed;

enum flexllce_timestamp_capture {
	FLEXLLCE_TIMESTAMPCAPTURE_DISABLE = 0,
	FLEXLLCE_TIMESTAMPCAPTURE_END = 1,
	FLEXLLCE_TIMESTAMPCAPTURE_START = 2,
	FLEXLLCE_TIMESTAMPCAPTURE_FD = 3,
} __packed;

enum flexllce_timestamp_hr_src {
	FLEXLLCE_HR_TIMESTAMP_SRC = 1,
} __packed;

/*
 * Command used by host. It is used in order to send commands from host to
 * FLEXLLCE using shared memory.
 */
struct flexllce_command {
	/* Command parameters. */
	__le32 cmd_ptr;
	/* Command ID. */
	enum flexllce_cmd_id cmd_id;
	/* Return status code after command completion. */
	enum flexllce_return return_value;
	/* Controller ID. */
	u8 instance;
	u8 reserved;
} __packed;
static_assert(sizeof(struct flexllce_command) == 8);

/* FlexLLCE bitrate related structures */
struct flexllce_time_segment {
	/* Propagation segment */
	__le32 prop_seg;
	/* Phase segment 1 */
	__le32 phase_seg1;
	/* Phase segment 2 */
	__le32 phase_seg2;
	/* Clock prescaler division factor */
	__le32 pre_divider;
	/* Resync jump width */
	__le32 rjw;
} __packed;
static_assert(sizeof(struct flexllce_time_segment) == 20);

/* FlexLLCE Set baudrate command structure */
struct flexllce_set_baudrate_cmd {
	/*
	 * The bitrate used for standard frames or for the arbitration phase of FD
	 * frames.
	 */
	struct flexllce_time_segment nominal;
	/* The bitrate used for the data phase of FD frames. */
	struct flexllce_time_segment data;
	/* Enable/Disable the Flexible Data Rate feature. */
	u8 fd_enable;
	/*
	 * The use of enhanced bit time segments format from ExCBT register, instead
	 * of CTRL1 or CBT register
	 */
	u8 enh_cbt_enable;
	/* Enable of BitRate Switch support for FD frames */
	u8 brs_enable;
	/* Transmiter Delay Compensation Enable. */
	enum flexllce_bool trcv_delay_enable;
	/* Specifies the Transmiter Delay Compensation Offset. */
	u8 trcv_delay_comp_offset;
	u8 reserved[3];
} __packed;
static_assert(sizeof(struct flexllce_set_baudrate_cmd) == 48);

/* FlexCAN Free Running Counter Time Stamp config. */
struct flexllce_timestamp_cfg {
	/* Timestamp Timer Source selection */
	enum flexllce_timestamp_src timestamp_source;
	/*
	 * Field selects which time base is used for capturing the 16-bit TIME_STAMP
	 * field of the message buffer
	 */
	enum flexllce_msgbuff_timestamp msg_buff_timestamp;
	/*
	 * Field configures the point in time when a 32-bit time base is captured
	 * during a CAN frame and stored in the high resolution time stamp
	 */
	enum flexllce_timestamp_capture hr_config;
	/* Field configures the HR Timestamp timer source */
	enum flexllce_timestamp_hr_src hr_src;
} __packed;
static_assert(sizeof(struct flexllce_timestamp_cfg) == 4);

/* FlexLLCE configuration */
struct flexllce_init_cmd {
	/* The maximum number of TX Message Buffers */
	__le32 max_mb_no;
	/* The number of standard ID filter elements */
	__le32 std_id_filters_no;
	/* The number of extended ID filter elements */
	__le32 ext_id_filters_no;
	/* Free Running Counter Time Stamp config */
	struct flexllce_timestamp_cfg time_stamp;
	/*
	 * Use of different features support like ISO-FD, EDGE_FILTER,
	 * AUTO_BussOffRecovery, Protocol_Exception.
	 */
	__le32 ctrl_options;
	/* Enable/Disable Self Receive feature. */
	u8 loopback_enable;
	/* Enable/Disable Listen Only feature. */
	u8 listen_only_enable;
	/* 3 bytes padding left */
	u8 padding[3];
	/* The reporting of the notification in Polling or Interrupt. */
	enum flexllce_notif_processing notif_reporting[3];
	/* The options that are relevant to baudrate configuration. */
	struct flexllce_set_baudrate_cmd baudrate;
} __packed;
static_assert(sizeof(struct flexllce_init_cmd) == 76);

/* FlexLLCE Get Status Command */
struct flexllce_get_status_cmd {
	/* Error and Status 1 Register */
	__le32 esr1;
	/* Error Counter Register */
	__le32 ecr;
	/* Start state of the Can Controller */
	u8 can_started;
	u8 reserved[3];
} __packed;
static_assert(sizeof(struct flexllce_get_status_cmd) == 12);

/* FlexLLCE Enhanced Rx FIFO ID filter table structure */
struct flexllce_id_filter {
	/* Enhanced Rx FIFO filter type */
	enum flexllce_filter_type filter_type;
	/* Extended frame */
	enum flexllce_bool is_extended;
	/* Enhanced Rx FIFO RTR2 */
	enum flexllce_bool rtr2;
	/* Enhanced Rx FIFO RTR1 */
	enum flexllce_bool rtr1;
	/* Enhanced Rx FIFO ID2 filter element */
	__le32 id2;
	/* Enhanced Rx FIFO ID1 filter element */
	__le32 id1;
	u8 reserved;
} __packed;
static_assert(sizeof(struct flexllce_id_filter) == 16);

/* FlexLLCE Set filter command structure */
struct flexllce_set_filter_cmd {
	/* pointer to FlexLLCE_Ip_EnhancedIdTableType */
	__le32 id_table;
	/*
	 * Field holds the address to the list of uint8 values representing receive
	 * interface for every filter. 0xFF is Interrupt.
	 */
	__le32 rx_interface;
	/* Field holds the reference to the Routing Table Rule. 0xFFFF is not used. */
	__le32 route_table_id;
	/* Field represents the count of hardware objects */
	__le32 mb_count;
	/* Field holds the reference to the FilterId to Hrh ObjectId mapping array */
	__le32 filter2hrh;
	/* Number of filters. */
	u8 filters_no;
	/* Padding */
	u8 padding[3];
} __packed;
static_assert(sizeof(struct flexllce_set_filter_cmd) == 24);

/* CAN statistics counters structure */
struct flexllce_can_stats {
	/* Number of transmitted Can frames, per controller */
	__le32 tx_count;
	/* Number of Can frames received by the host, per controller */
	__le32 rx_count;
	/* Number of Can frames routed to other controllers, per controller */
	__le32 routed_count;
} __packed;
static_assert(sizeof(struct flexllce_can_stats) == 12);

/* LIN statistics counters structure */
struct flexllce_lin_statistics_counter {
	/*
	 * Number of transmitted Lin frames, per controller, in Transmit Mode (the
	 * master sends the payload)
	 */
	__le32 lin_tx_cnt;
	/*
	 * Number of Lin frames received from the slave, per controller, in Receive
	 * Mode (the slave sends the payload)
	 */
	__le32 lin_rx_cnt;
} __packed;
static_assert(sizeof(struct flexllce_lin_statistics_counter) == 8);

/* ETH statistics counters structure */
struct flexllce_eth_statistics_counter {
	/* Number of transmitted Eth frames, per HIF */
	__le32 eth_tx_cnt;
	/* Number of CAN ACF received frames, per HIF */
	__le32 eth_can_acf_rx_cnt;
	/* Number of LIN ACF received frames, per HIF */
	__le32 eth_lin_acf_rx_cnt;
} __packed;
static_assert(sizeof(struct flexllce_eth_statistics_counter) == 12);

/* FlexLLCE Statistics Counter */
union flexllce_statistics_counter {
	struct flexllce_can_stats can;
	struct flexllce_lin_statistics_counter lin;
	struct flexllce_eth_statistics_counter eth;
} __packed;
static_assert(sizeof(union flexllce_statistics_counter) == 12);

/* FlexLLCE Get Statistics Command */
struct flexllce_statistics_cmd {
	/* Counters returned for the selected firmware module. */
	union flexllce_statistics_counter statistics;
	/* Firmware module whose counters are requested. */
	enum flexllce_stats_module module;
	u8 reserved[3];
} __packed;
static_assert(sizeof(struct flexllce_statistics_cmd) == 16);

struct flexllce_notification {
	/*
	 * Associated error register, if any: ECR for FLEXLLCE_EVENT_PASSIVE_STATE,
	 * ESR for FLEXLLCE_EVENT_ERROR, FLEXLLCE_EVENT_ERROR_FAST
	 */
	__le32 error_reg;
	/* Error category. */
	enum flexllce_notif_category notif_categ;
	/* Error code. */
	enum flexllce_event_code event_code;
	/* Controller ID. */
	u8 instance;
	u8 reserved;
} __packed;
static_assert(sizeof(struct flexllce_notification) == 8);

struct flexllce_build_info {
	char build_date[12];
	char build_time[9];
	u8 reserved[3];
} __packed;
static_assert(sizeof(struct flexllce_build_info) == 24);

struct flexllce_fw_info {
	/* FLEXLLCE FW build date */
	struct flexllce_build_info build_info;
	/*
	 * Identifies the FW type: - 0 - Standard FW targeting all customers - 1-n >=
	 * Custom1, Custom2... etc
	 */
	u8 fw_type_id;
	/* Major revision */
	u8 major_version;
	/* Minor revision, bumped on new compatible changes added */
	u8 minor_version;
	/* Hotfix release (patch version, bug fix releases) */
	u8 patch_version;
	/* Code drop revision */
	u8 code_drop;
	u8 reserved[3];
} __packed;
static_assert(sizeof(struct flexllce_fw_info) == 32);

/* FlexLLCE Boot Status */
struct flexllce_boot_status {
	struct flexllce_fw_info fw_info;
	/* Boot Status of FlexLLCE Core 0. This can be NOTRUN or SUCCESS */
	enum flexllce_return core0_boot_status;
	/* Boot Status of FlexLLCE Core 1. This can be NOTRUN or SUCCESS */
	enum flexllce_return core1_boot_status;
	u8 reserved[2];
} __packed;
static_assert(sizeof(struct flexllce_boot_status) == 36);

/* FlexLLCE_TimeStampType Used to express time stamps based on relative time. */
struct flexllce_time_stamp {
	/* Nanoseconds part of the time. */
	__le32 nanoseconds;
	/* Seconds part of the time. */
	__le32 seconds;
} __packed;
static_assert(sizeof(struct flexllce_time_stamp) == 8);

struct flexllce_rx_data {
	/* Code and Status */
	__le32 cs;
	/* Message Buffer ID */
	__le32 msg_id;
	/* Data bytes of the FlexCAN message */
	u8 payload[64];
	/* Length of data in bytes */
	u8 data_len;
	/* Identifier Acceptance Filter Hit Indicator */
	u8 filter_id;
	/* Controller ID */
	u8 instance;
	/* 1 byte padding left */
	u8 padding;
	/* Time Stamp in s+ns */
	struct flexllce_time_stamp timestamp;
} __packed;
static_assert(sizeof(struct flexllce_rx_data) == 84);

/*
 * Command used by host. It is used in order to send commands from host to
 * FLEXLLCE using shared memory.
 */
struct flexllce_tx_descriptor {
	/* Frame ID. */
	__le32 msg_id;
	/* Type of message ID (standard or extended). */
	u8 ext_id;
	/* Length of Data in Bytes. */
	u8 data_len;
	/*
	 * Set a value for padding. It will be used when the data length code (DLC)
	 * specifies a bigger payload size than data_length to fill the MB.
	 */
	u8 fd_padding;
	/*
	 * First bit(LSB) represents enable(1) or disable FD(0). The second bit(second
	 * LSB) represents enable(1) or disable(0) bit rate switch inside a CAN FD
	 * format frame.
	 */
	u8 ctrl_options;
} __packed;
static_assert(sizeof(struct flexllce_tx_descriptor) == 8);

struct flexllce_tx_data {
	/* Data bytes of the FlexCAN message. */
	u8 payload[64];
	/* Frame ID. */
	struct flexllce_tx_descriptor descriptor;
	/* Unique tag to identify the sent frame on the confirmation flow. */
	__le16 tracking_idx;
	/* Controller ID. */
	u8 instance;
	/*
	 * ack channel: 0 = interrupt (drained by the host), 1-5 = poll classes; a
	 * value >= the channel count suppresses the ack (0xFF = no ack)
	 */
	u8 ack_interface;
} __packed;
static_assert(sizeof(struct flexllce_tx_data) == 76);

struct flexllce_tx_ack {
	/* Transmission time stamp. */
	struct flexllce_time_stamp tx_timestamp;
	/* Tag returned by the fw on the confirmation flow. */
	__le16 tracking_idx;
	u8 reserved[2];
} __packed;
static_assert(sizeof(struct flexllce_tx_ack) == 12);

struct flexllce_shared_memory {
	struct flexllce_rx_data rx_mb[256];
	/*
	 * descriptors (continuous range) to random buffer indexes (to allow
	 * partitioning of buffers per polling class)
	 */
	__le32 rx_comtable[6][32];
	/* cmd is filled in host memory, only the pointer to cmd is sent to fw */
	struct flexllce_command cmd;
	/* indexed with a CommDescType and returns MbIdxType */
	__le32 tx_comtable[32];
	struct flexllce_tx_ack ack_mb[256];
	/* indexed with a CommDescType and returns MbIdxType */
	__le32 ack_comtable[6][32];
	struct flexllce_notification notif_mb[32];
	/* ComDesc for Notif table */
	__le32 notif_comtable[2][16];
	/* FlexLLCE_DevErrorReportType aDevErrorReport (unused) */
	u8 reserved_dev_error[1080];
} __packed;
static_assert(sizeof(struct flexllce_shared_memory) == 27712);

/* Firmware constants (values taken from the firmware header). */
#define FLEXLLCE_CTRL_OPT_THREE_SAMPLES	0x02
#define FLEXLLCE_CTRL_OPT_ISO_FD	0x20
#define FLEXLLCE_CTRL_OPT_BUSOFF_MANUAL	0x80
#define FLEXLLCE_TX_OPT_FD	0x01
#define FLEXLLCE_TX_OPT_BRS	0x02
#define FLEXLLCE_FILTER_ROUTE_NONE	0xffff
#define FLEXLLCE_TX_ACK_INTERRUPT	0
#define FLEXLLCE_TX_ACK_SUPPRESS	0xff
#define FLEXLLCE_TX_DESC_COUNT	32
#define FLEXLLCE_RX_DESC_COUNT	32
#define FLEXLLCE_ACK_DESC_COUNT	32
#define FLEXLLCE_NOTIF_DESC_COUNT	16

#endif /* _FLEXLLCE_FW_ABI_H_ */
