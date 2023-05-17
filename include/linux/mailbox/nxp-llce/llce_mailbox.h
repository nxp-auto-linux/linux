/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020-2022 NXP */
#ifndef LLCE_MAILBOX_H
#define LLCE_MAILBOX_H

#include <linux/spinlock_types.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <uapi/linux/can.h>

/* LLCE CAN filter structure */
#define LLCE_FILTER_HW_CTRL_SHIFT	0
#define LLCE_FILTER_HW_CTRL_MASK	((u16)GENMASK(7, LLCE_FILTER_HW_CTRL_SHIFT))

#define LLCE_FILTER_MB_TYPE_SHIFT	8
#define LLCE_FILTER_MB_TYPE_MASK	((u16)GENMASK(15, LLCE_FILTER_MB_TYPE_SHIFT))

#define LLCE_HW_CTRL_FROM_CHAN_IDX	(0xffu)

struct llce_mb;

enum llce_chan_state {
	LLCE_UNREGISTERED_CHAN,
	LLCE_REGISTERED_CHAN,
};

/** Private data attached to a LLCE channel */
struct llce_chan_priv {
	struct llce_mb *mb;
	void *last_msg;
	unsigned int type;
	unsigned int index;
	enum llce_chan_state state;
	spinlock_t lock;
	void *data;
};

struct llce_tx_msg {
	bool fd_msg;
	bool long_msg;
	struct canfd_frame *cf;
};

struct llce_tx_notif {
	enum llce_fw_return error;
	uint32_t tx_timestamp;
};

enum llce_rx_cmd {
	LLCE_RX_NOTIF,
	LLCE_DISABLE_RX_NOTIF,
	LLCE_ENABLE_RX_NOTIF,
	LLCE_IS_RX_EMPTY,
	LLCE_POP_RX,
	LLCE_RELEASE_RX_INDEX,
	LLCE_ERROR,
};

struct llce_rx_can_mb {
	union {
		struct llce_can_short_mb *shortm;
		struct llce_can_mb *longm;
	} data;
	bool is_long;
};

struct llce_rx_msg {
	enum llce_rx_cmd cmd;
	enum llce_fw_return error;
	union {
		bool is_rx_empty;
		struct {
			struct llce_rx_can_mb mb;
			uint32_t index;
			bool skip;
		} rx_pop;
		struct {
			uint32_t index;
		} rx_release;
	};
};

struct llce_logger_msg {
	struct llce_can_mb *frame;
	u8 hw_ctrl;
};

enum logger_config_cmd {
	LOGGER_CMD_FW_SUPPORT = 1,
};

struct logger_config_msg {
	enum logger_config_cmd cmd;
	bool fw_logger_support;
};

static inline void llce_filter_set_mb_type(u16 *filter, bool long_mb)
{
	*filter &= ~LLCE_FILTER_MB_TYPE_MASK;

	if (long_mb)
		*filter |= (USE_LONG_MB << LLCE_FILTER_MB_TYPE_SHIFT);
	else
		*filter |= (USE_SHORT_MB << LLCE_FILTER_MB_TYPE_SHIFT);
}

static inline void llce_filter_set_hwctrl(u16 *filter, u8 hwctrl)
{
	*filter &= LLCE_FILTER_HW_CTRL_MASK;

	*filter |= (hwctrl << LLCE_FILTER_HW_CTRL_SHIFT);
}

static inline u16 llce_filter_get_mb_type(u16 filter)
{
	return (filter & LLCE_FILTER_MB_TYPE_MASK) >> LLCE_FILTER_MB_TYPE_SHIFT;
}

static inline u8 llce_filter_get_hw_ctrl(u16 filter)
{
	return (filter & LLCE_FILTER_HW_CTRL_MASK) >> LLCE_FILTER_HW_CTRL_SHIFT;
}

#endif
