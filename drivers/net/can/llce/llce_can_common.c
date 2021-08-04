// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2021 NXP */
#include <linux/can/dev.h>
#include <linux/can/dev/llce_can_common.h>
#include <linux/ctype.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/of.h>

#define LLCE_CAN_MAX_RX_MB		16U

static int get_llce_can_id(const char *node_name, unsigned long *id)
{
	const char *p = node_name + strlen(node_name) - 1;

	while (isdigit(*p))
		p--;

	return kstrtoul(p + 1, 10, id);
}

static void *get_netdev_name(struct device *dev, const char *basename)
{
	unsigned long id;
	char *dev_name;
	const char *node_name;
	size_t name_len;

	node_name = dev->of_node->name;
	if (get_llce_can_id(node_name, &id)) {
		dev_err(dev, "Failed to detect node id for: %s\n", node_name);
		return ERR_PTR(-EIO);
	}

	/* 0-99 device ids + \0 */
	name_len = strlen(basename) + 3;
	dev_name = devm_kmalloc(dev, name_len, GFP_KERNEL);
	if (!dev_name)
		return ERR_PTR(-ENOMEM);

	snprintf(dev_name, name_len, "%s%lu", basename, id);

	return dev_name;
}

static int send_rx_msg(struct llce_can_dev *llce, struct llce_rx_msg *msg)
{
	int ret = mbox_send_message(llce->rx, msg);

	if (ret < 0)
		return ret;

	mbox_client_txdone(llce->rx, 0);

	return 0;
}

int enable_llce_rx_notif(struct llce_can_dev *llce)
{
	struct llce_rx_msg msg = {
		.cmd = LLCE_ENABLE_RX_NOTIF,
	};

	return send_rx_msg(llce, &msg);
}
EXPORT_SYMBOL(enable_llce_rx_notif);

static bool is_rx_empty(struct llce_can_dev *llce)
{
	struct llce_rx_msg msg = {
		.cmd = LLCE_IS_RX_EMPTY,
	};

	if (send_rx_msg(llce, &msg))
		return false;

	return msg.is_rx_empty;
}

static int pop_rx_fifo(struct llce_can_dev *llce, uint32_t *index, bool *skip,
		       struct llce_can_mb **can_mb)
{
	int ret;
	struct llce_rx_msg msg = {
		.cmd = LLCE_POP_RX,
	};

	ret = send_rx_msg(llce, &msg);

	*can_mb = msg.rx_pop.can_mb;
	*index = msg.rx_pop.index;
	*skip = msg.rx_pop.skip;
	return ret;
}

static int release_rx_index(struct llce_can_dev *llce, uint32_t index)
{
	struct llce_rx_msg msg = {
		.cmd = LLCE_RELEASE_RX_INDEX,
		.rx_release = {
			.index = index,
		},
	};

	return send_rx_msg(llce, &msg);
}

static void add_hwtimestamp(struct sk_buff *skb, u32 timestamp)
{
	struct skb_shared_hwtstamps *shhwtstamps;

	shhwtstamps = skb_hwtstamps(skb);

	/* Report cycles as seconds */
	shhwtstamps->hwtstamp = ms_to_ktime((u64)timestamp * 1000ULL);
}

static void process_rx_msg(struct llce_can_dev *llce, struct llce_can_mb *can_mb)
{
	struct net_device *dev = llce->can.dev;
	struct net_device_stats *net_stats = &llce->can.dev->stats;
	struct sk_buff *skb;
	struct canfd_frame *cf;
	u32 std_id, ext_id;
	bool rtr, ide, brs, esi, fdf;
	u8 len;

	unpack_word0(can_mb->word0, &rtr, &ide, &std_id, &ext_id);
	unpack_word1(can_mb->word1, &fdf, &len, &brs, &esi);

	if (fdf)
		skb = alloc_canfd_skb(dev, &cf);
	else
		skb = alloc_can_skb(dev, (struct can_frame **)&cf);

	if (!skb) {
		net_stats->rx_dropped++;
		goto notif_exit;
	}

	cf->can_id = std_id & CAN_SFF_MASK;
	if (ide) {
		cf->can_id |= ((ext_id << CAN_SFF_ID_BITS) &
				   CAN_EFF_MASK);
		cf->can_id |= CAN_EFF_FLAG;
	}

	if (fdf) {
		if (brs)
			cf->flags |= CANFD_BRS;

		if (esi)
			cf->flags |= CANFD_ESI;

	} else {
		if (rtr)
			cf->can_id |= CAN_RTR_FLAG;
	}
	cf->len = can_dlc2len(len);

	memcpy(cf->data, can_mb->payload, cf->len);

	net_stats->rx_packets++;
	net_stats->rx_bytes += cf->len;

	add_hwtimestamp(skb, can_mb->timestamp);

	netif_receive_skb(skb);

notif_exit:
	return;
}

static int llce_rx_poll(struct napi_struct *napi, int quota)
{
	struct llce_can_dev *llce = container_of(napi, struct llce_can_dev, napi);
	struct net_device *dev = llce->can.dev;
	int num_pkts = 0;
	struct llce_can_mb *can_mb;
	u32 index;
	int ret;
	bool skip = false;

	while (!is_rx_empty(llce) && num_pkts < quota) {
		ret = pop_rx_fifo(llce, &index, &skip, &can_mb);
		if (ret) {
			netdev_err(dev, "Failed to pop RX FIFO\n");
			return num_pkts;
		}

		if (skip)
			break;

		process_rx_msg(llce, can_mb);

		num_pkts++;

		ret = release_rx_index(llce, index);
		if (ret)
			netdev_err(dev, "Failed to release RX FIFO index\n");
	}

	/* All packets processed */
	if (num_pkts < quota) {
		napi_complete_done(napi, num_pkts);

		/* Enable RX notification / IRQ */
		if (!skip && enable_llce_rx_notif(llce))
			netdev_err(dev, "Failed to enable RX notifications\n");
	}

	return num_pkts;
}

void enable_llce_napi(struct llce_can_dev *dev)
{
	netif_napi_add(dev->can.dev, &dev->napi, llce_rx_poll,
		       LLCE_CAN_MAX_RX_MB);
}
EXPORT_SYMBOL(enable_llce_napi);

void process_llce_can_error(struct llce_can_dev *llce,
			    enum llce_fw_return error,
			    enum llce_can_module module)
{
	struct can_frame *cf;
	struct can_device_stats *can_stats = &llce->can.can_stats;
	struct net_device_stats *net_stats = &llce->can.dev->stats;
	struct sk_buff *skb = alloc_can_err_skb(llce->can.dev, &cf);
	struct net_device *dev = llce->can.dev;

	if (!skb) {
		netdev_dbg(llce->can.dev, "Could not allocate error frame\n");
		return;
	}

	if (module == LLCE_TX)
		net_stats->tx_errors++;
	else
		net_stats->rx_errors++;

	/* Propagate the error condition to the CAN stack */
	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	switch (error) {
	case LLCE_ERROR_BCAN_ACKERR:
		cf->can_id |= CAN_ERR_ACK;
		cf->data[3] = CAN_ERR_PROT_LOC_ACK;
		fallthrough;
	case LLCE_ERROR_BCAN_BIT0ERR:
		if (error == LLCE_ERROR_BCAN_BIT0ERR)
			cf->data[2] |= CAN_ERR_PROT_BIT0;
		fallthrough;
	case LLCE_ERROR_BCAN_BIT1ERR:
		if (error == LLCE_ERROR_BCAN_BIT1ERR)
			cf->data[2] |= CAN_ERR_PROT_BIT1;
		fallthrough;
	case LLCE_ERROR_BCAN_CRCERR:
		if (error == LLCE_ERROR_BCAN_CRCERR) {
			cf->data[2] |= CAN_ERR_PROT_BIT;
			cf->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
		}
		fallthrough;
	case LLCE_ERROR_BCAN_FRMERR:
		if (error == LLCE_ERROR_BCAN_FRMERR)
			cf->data[2] |= CAN_ERR_PROT_FORM;
		fallthrough;
	case LLCE_ERROR_BCAN_FRZ_ENTER:
	case LLCE_ERROR_BCAN_FRZ_EXIT:
	case LLCE_ERROR_BCAN_LPM_EXIT:
	case LLCE_ERROR_BCAN_SRT_ENTER:
	case LLCE_ERROR_BCAN_STFERR:
		if (error == LLCE_ERROR_BCAN_STFERR)
			cf->data[2] |= CAN_ERR_PROT_STUFF;
		fallthrough;
	case LLCE_ERROR_BCAN_SYNC:
	case LLCE_ERROR_BCAN_UNKNOWN_ERROR:
		can_stats->bus_error++;
		break;
	case LLCE_ERROR_BUSOFF:
	case LLCE_ERROR_HARDWARE_BUSOFF:
		/**
		 * A restart is not needed as we have automatic
		 * bus-off recovery
		 */
		can_stats->bus_off++;
		break;
	case LLCE_ERROR_DATA_LOST:
	case LLCE_ERROR_MB_NOTAVAILABLE:
	case LLCE_ERROR_RXOUT_FIFO_FULL:
	case LLCE_ERROR_SW_FIFO_FULL:
		net_stats->rx_dropped++;
		break;
	case LLCE_ERROR_BCAN_RXFIFO_OVERRUN:
		net_stats->rx_over_errors++;
		net_stats->rx_errors++;
		break;
	default:
		netdev_err(llce->can.dev, "Unhandled %d error %d\n",
			   module, error);
		break;
	}

	can_free_echo_skb(dev, 0);
	netif_rx(skb);
}
EXPORT_SYMBOL(process_llce_can_error);

static void llce_rx_notif_callback(struct mbox_client *cl, void *msg)
{
	struct llce_rx_msg *rx_msg = msg;
	struct llce_can_dev *llce = container_of(cl, struct llce_can_dev,
						 rx_client);

	/* This is executed in IRQ context */
	if (rx_msg->error)
		process_llce_can_error(llce, rx_msg->error, LLCE_RX);

	if (rx_msg->cmd == LLCE_RX_NOTIF) {
		if (!napi_schedule_prep(&llce->napi))
			return;

		__napi_schedule(&llce->napi);
	}
}

static void init_llce_rx_client(struct llce_can_dev *llce, struct device *dev)
{
	llce->rx_client = (struct mbox_client) {
		.dev = dev,
		.tx_block = false,
		.rx_callback = llce_rx_notif_callback,
	};
}

struct llce_can_dev *init_llce_can_dev(struct device *dev, size_t priv_size,
				       const char *basename)
{
	char *dev_name;
	struct llce_can_dev *llce;
	struct net_device *netdev;
	int ret = 0;

	netdev = alloc_candev(priv_size, 1);
	if (!netdev)
		return ERR_PTR(-ENOMEM);

	dev_name = get_netdev_name(dev, basename);
	if (IS_ERR_VALUE(dev_name)) {
		ret = PTR_ERR(dev_name);
		goto free_mem;
	}

	strncpy(netdev->name, dev_name, sizeof(netdev->name));
	SET_NETDEV_DEV(netdev, dev);

	llce = netdev_priv(netdev);

	init_llce_rx_client(llce, dev);

free_mem:
	if (ret) {
		free_candev(netdev);
		return ERR_PTR(ret);
	}

	return llce;
}
EXPORT_SYMBOL(init_llce_can_dev);

void free_llce_netdev(struct llce_can_dev *dev)
{
	free_candev(dev->can.dev);
}
EXPORT_SYMBOL(free_llce_netdev);

