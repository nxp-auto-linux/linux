/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/* Copyright 2021-2023 NXP */

#ifndef LLCE_CAN_COMMON_H
#define LLCE_CAN_COMMON_H

#include <linux/can/dev.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <linux/mailbox/nxp-llce/llce_interface_can_utils.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/netdevice.h>
#include <uapi/linux/can.h>

#define LLCE_CAN_NETDEV_IF_NAME		"llcecan"
#define LLCE_LOGGER_NETDEV_IF_NAME	"llcelogger"

struct llce_can_dev {
	struct can_priv can; /* Must be the first member */
	struct napi_struct napi;

	struct completion config_cmd_done;

	struct mbox_client config_client, rx_client;
	struct mbox_chan *config, *rx;
	u64 *stats;
	atomic_t rx_processing;

	unsigned int id;
};

static inline bool is_llce_rx_busy(struct llce_can_dev *dev)
{
	return !!atomic_read(&dev->rx_processing);
}

static inline void unpack_word0(u32 word0, bool *rtr, bool *ide,
				u32 *std_id, u32 *ext_id)
{
	if (word0 & LLCE_CAN_MB_IDE) {
		*ide = true;
		*ext_id = (word0 & CAN_EFF_MASK) >> CAN_SFF_ID_BITS;
		*std_id = word0 & CAN_SFF_MASK;
	} else {
		*ide = false;
		*std_id = (word0 & LLCE_CAN_MB_IDSTD_MASK) >>
		    LLCE_CAN_MB_IDSTD_SHIFT;
	}

	*rtr = !!(word0 & LLCE_CAN_MB_RTR);
}

static inline u32 pack_word0(bool rtr, bool ide, u32 id)
{
	u32 word0;

	word0 = (rtr << LLCE_CAN_MB_RTR_SHIFT);
	word0 |= (ide << LLCE_CAN_MB_IDE_SHIFT);

	if (ide)
		word0 |= (id & CAN_EFF_MASK);
	else
		word0 |= (id & CAN_SFF_MASK) <<  LLCE_CAN_MB_IDSTD_SHIFT;

	return word0;
}

static inline void unpack_word1(u32 word1, bool *fdf, u8 *dlc, bool *brs,
				bool *esi)
{
	*dlc = word1 & LLCE_CAN_MB_DLC_MASK;
	*brs = !!(word1 & LLCE_CAN_MB_BRS);
	*esi = !!(word1 & LLCE_CAN_MB_ESI);
	*fdf = !!(word1 & LLCE_CAN_MB_FDF);
}

static inline u32 pack_word1(bool fdf, u8 dlc, bool brs, bool esi)
{
	u32 word1;

	word1 = (brs << LLCE_CAN_MB_BRS_SHIFT);
	word1 |= (fdf << LLCE_CAN_MB_FDF_SHIFT);
	word1 |= (esi << LLCE_CAN_MB_ESI_SHIFT);
	word1 |= dlc;

	return word1;
}

static inline struct device *llce_can_chan_dev(struct mbox_chan *conf_chan)
{
	struct mbox_client *cl = conf_chan->cl;

	return cl->dev;
}

struct llce_can_dev *init_llce_can_dev(struct device *dev, size_t priv_size,
				       const char *basename);

void free_llce_netdev(struct llce_can_dev *dev);

void enable_llce_napi(struct llce_can_dev *dev);

void process_llce_can_error(struct llce_can_dev *llce,
			    enum llce_fw_return error,
			    enum llce_can_module module);

int enable_llce_rx_notif(struct llce_can_dev *llce);

int llce_send_config_cmd(struct mbox_chan *conf_chan,
			 struct llce_config_msg *msg,
			 struct completion *config_done);
#endif
