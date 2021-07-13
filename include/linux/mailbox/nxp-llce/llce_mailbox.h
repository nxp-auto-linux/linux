/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020-2021 NXP */
#ifndef LLCE_MAILBOX_H
#define LLCE_MAILBOX_H

#include <linux/spinlock_types.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <uapi/linux/can.h>

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
	/* LLCE channel lock. */
	spinlock_t lock;
};

struct llce_tx_msg {
	bool fd;
	struct canfd_frame *cf;
};

struct llce_tx_notif {
	enum llce_fw_return error;
	u32 tx_timestamp;
};

enum llce_rx_cmd {
	LLCE_RX_NOTIF,
	LLCE_DISABLE_RX_NOTIF,
	LLCE_ENABLE_RX_NOTIF,
	LLCE_IS_RX_EMPTY,
	LLCE_POP_RX,
	LLCE_RELESE_RX_INDEX,
	LLCE_ERROR,
};

struct llce_rx_msg {
	enum llce_rx_cmd cmd;
	enum llce_fw_return error;
	union {
		bool is_rx_empty;
		struct {
			u32 index;
			struct llce_can_mb *can_mb;
		} rx_pop;
		struct {
			u32 index;
		} rx_release;
	};
};

struct llce_logger_msg {
	struct llce_can_mb *frame;
	u8 hw_ctrl;
};

#endif
