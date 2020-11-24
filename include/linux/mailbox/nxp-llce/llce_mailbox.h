/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020 NXP */
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
	spinlock_t lock;
};

struct llce_tx_msg {
	bool fd;
	struct canfd_frame *cf;
};

struct llce_notif {
	enum llce_can_error error;
	union {
		/* TX notification */
		uint32_t tx_timestamp;
		/* RX notiication */
		struct llce_can_mb *can_mb;
	};
};

#endif
