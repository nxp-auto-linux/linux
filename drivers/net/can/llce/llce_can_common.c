// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2021-2024 NXP */
#include <linux/can/dev.h>
#include <linux/can/dev/llce_can_common.h>
#include <linux/ctype.h>
#include <linux/ethtool.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/of.h>

#define LLCE_CAN_MAX_RX_MB		16U

#define LLCE_ARR_ERR_ENTRY(BASE_INDEX, ENTRY, IS_CAN) \
	[(ENTRY) - (BASE_INDEX)] = { \
		.name = __stringify_1(ENTRY), \
		.is_can_error = (IS_CAN),\
	}

#define LLCE_ERROR_ENTRY(ERROR) \
	LLCE_ARR_ERR_ENTRY(LLCE_ERROR_TXACK_FIFO_FULL, ERROR, false)

#define LLCE_CAN_ERROR_ENTRY(ERROR) \
	LLCE_ARR_ERR_ENTRY(LLCE_ERROR_TXACK_FIFO_FULL, ERROR, true)

#define LLCE_ERROR_PREFIX "LLCE_ERROR_"
#define LLCE_NOTIF_PREFIX "LLCE_NOTIF_"

struct llce_error {
	const char *name;
	bool is_can_error;
};

static const struct llce_error llce_errors[] = {
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_TXACK_FIFO_FULL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RXOUT_FIFO_FULL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_0),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_HW_FIFO_FULL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_SW_FIFO_EMPTY),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_SW_FIFO_FULL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_MB_NOTAVAILABLE),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_SHORT_MB_NOTAVAILABLE),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_FRZ_EXIT),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_SYNC),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_FRZ_ENTER),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_LPM_EXIT),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_SRT_ENTER),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_UNKNOWN_ERROR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_ACKERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_CRCERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_BIT0ERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_BIT1ERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_DPBIT1ERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_DPBIT0ERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_DPSTFERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_DPFRMERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_DPCRCERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_FRMERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_STFERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_TDCFAIL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_RXFIFO_OVERRUN),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_DATA_LOST),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_TXLUT_FULL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CMD_PROCESSING),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_1),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RXLUT_ACCESS_MODE),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RXLUT_SEARCH_MODE),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RXLUT_SLOW_OPERATION),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RXLUT_INCOMPLETE_OP),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RXLUT_OPERATING_MODE),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RXLUT_INIT_SLOW_OP),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_2),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_3),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_4),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_5),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_6),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CTRL_NOT_READY),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BUSOFF),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_FIFO_LOG_FULL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_7),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_8),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_COMMAND_RXPPE_NORESPONSE),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_COMMAND_AF_NORESPONSE),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_COMMAND_DEINIT_NOTSTOP),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RXTOKENS_UNRETURNED),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_TXACK_NOT_READ),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_COMMAND_NOTSUPPORTED),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_COMMAND_NOTVALIDATED),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_COMMAND_NOTACCEPTED),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_COMMAND_INVALID_PARAMS),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_9),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_FRAME_NOT_DELIVERED),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_10),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_FRAME_NOT_DELIVERED_TO_HOST),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_12),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_FILTERS_FULL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_FILTERS_NOTEXIST),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_FILTERS_MASK_EMPTY),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_FILTERS_RANGE_EMPTY),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_FILTERS_EM_EMPTY),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_IDX_NOT_VALID_HOST),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_IDX_NOT_VALID_LOG),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_13),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_14),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_15),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RXFRAME_AUTH_ERROR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_16),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CODE_RESERVED_17),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RX_SW_FIFO_EMPTY),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_PFEIF),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_HSEIF),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_INTERNALDESC_NOT_RETURNED),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_INTERNALDESC_NOT_DELIVERED),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_INTERNALDESC_NOTAVAIL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_INTERNALDESC_FIFO_FULL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_MB_NOTAVAIL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_MB_FIFO_FULL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_NO_MB_AVAILABLE),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_UNKNOWN_SRC),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_UNKNOWN_DEST),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_UNKNOWN_REQUEST),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_CONVERSION),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_NO_MB_TO_ABORT),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_INDEX_NOT_RECOVERED),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_RESET_PENDING),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_FATAL),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_TXWRN),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_RXWRN),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_BCAN_PASSERR),
	LLCE_CAN_ERROR_ENTRY(LLCE_ERROR_ROUTINGCHANNEL_DISABLED),
};

static int get_llce_can_id(const char *node_name, int *id)
{
	const char *p = node_name + strlen(node_name) - 1;

	while (p >= node_name && isdigit(*p))
		p--;

	return kstrtoint(p + 1, 10, id);
}

static int get_netdev_id(struct device *dev)
{
	const char *node_name;
	int id;

	node_name = dev->of_node->name;
	if (get_llce_can_id(node_name, &id)) {
		dev_err(dev, "Failed to detect node id for: %s\n", node_name);
		return -EIO;
	}

	return id;
}

static void *get_netdev_name(struct device *dev, const char *basename, int id)
{
	char *dev_name;
	size_t name_len;

	/* 0-99 device ids + \0 */
	name_len = strlen(basename) + 3;
	dev_name = devm_kmalloc(dev, name_len, GFP_KERNEL);
	if (!dev_name)
		return ERR_PTR(-ENOMEM);

	snprintf(dev_name, name_len, "%s%d", basename, id);

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
		.error = LLCE_FW_SUCCESS,
		.cmd = LLCE_ENABLE_RX_NOTIF,
	};

	return send_rx_msg(llce, &msg);
}
EXPORT_SYMBOL(enable_llce_rx_notif);

static bool is_rx_empty(struct llce_can_dev *llce)
{
	struct llce_rx_msg msg = {
		.error = LLCE_FW_SUCCESS,
		.cmd = LLCE_IS_RX_EMPTY,
	};

	if (send_rx_msg(llce, &msg))
		return false;

	return msg.is_rx_empty;
}

static int pop_rx_fifo(struct llce_can_dev *llce, uint32_t *index, bool *skip,
		       struct llce_rx_can_mb *can_mb)
{
	int ret;
	struct llce_rx_msg msg = {
		.error = LLCE_FW_SUCCESS,
		.cmd = LLCE_POP_RX,
	};

	ret = send_rx_msg(llce, &msg);

	memcpy(can_mb, &msg.rx_pop.mb, sizeof(*can_mb));
	*index = msg.rx_pop.index;
	*skip = msg.rx_pop.skip;
	return ret;
}

static int release_rx_index(struct llce_can_dev *llce, uint32_t index)
{
	struct llce_rx_msg msg = {
		.error = LLCE_FW_SUCCESS,
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

static void process_rx_msg(struct llce_can_dev *llce,
			   struct llce_rx_can_mb *can_mb)
{
	struct net_device *dev = llce->can.dev;
	struct net_device_stats *net_stats = &llce->can.dev->stats;
	struct sk_buff *skb;
	struct canfd_frame *cf;
	u32 std_id, ext_id, word0, word1, timestamp;
	bool rtr, ide, brs, esi, fdf;
	u8 len, *payload;

	if (can_mb->is_long) {
		word0 = can_mb->data.longm.word0;
		word1 = can_mb->data.longm.word1;
		payload = &can_mb->data.longm.payload[0];
		timestamp = can_mb->data.longm.timestamp;
	} else {
		word0 = can_mb->data.shortm.word0;
		word1 = can_mb->data.shortm.word1;
		payload = &can_mb->data.shortm.payload[0];
		timestamp = can_mb->data.shortm.timestamp;
	}

	unpack_word0(word0, &rtr, &ide, &std_id, &ext_id);
	unpack_word1(word1, &fdf, &len, &brs, &esi);

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
	cf->len = can_fd_dlc2len(len);

	memcpy(cf->data, payload, cf->len);

	net_stats->rx_packets++;
	net_stats->rx_bytes += cf->len;

	add_hwtimestamp(skb, timestamp);

	netif_receive_skb(skb);

notif_exit:
	return;
}

static int llce_rx_poll(struct napi_struct *napi, int quota)
{
	struct llce_can_dev *llce = container_of(napi, struct llce_can_dev, napi);
	struct net_device *dev = llce->can.dev;
	int num_pkts = 0;
	struct llce_rx_can_mb can_mb;
	u32 index;
	int ret = 0;
	bool skip = false;

	atomic_set(&llce->rx_processing, 1);
	while (!is_rx_empty(llce) && num_pkts < quota) {
		ret = pop_rx_fifo(llce, &index, &skip, &can_mb);
		if (ret) {
			netdev_err(dev, "Failed to pop RX FIFO\n");
			break;
		}

		if (skip)
			break;

		process_rx_msg(llce, &can_mb);

		num_pkts++;

		ret = release_rx_index(llce, index);
		if (ret)
			netdev_err(dev, "Failed to release RX FIFO index\n");
	}

	if (ret)
		return num_pkts;

	/* All packets processed */
	if (num_pkts < quota) {
		atomic_set(&llce->rx_processing, 0);
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
	struct can_priv can = llce->can;
	struct can_device_stats *can_stats = &llce->can.can_stats;
	struct net_device_stats *net_stats = &llce->can.dev->stats;
	struct net_device *dev = llce->can.dev;
	struct sk_buff *skb;

	llce->stats[error - LLCE_ERROR_TXACK_FIFO_FULL]++;

	if (!(can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING))
		return;

	skb = alloc_can_err_skb(llce->can.dev, &cf);
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
		break;
	}

	can_free_echo_skb(dev, 0, 0);
	netif_rx(skb);
}
EXPORT_SYMBOL(process_llce_can_error);

static void llce_rx_notif_callback(struct mbox_client *cl, void *msg)
{
	struct llce_rx_msg *rx_msg = msg;
	struct llce_can_dev *llce = container_of(cl, struct llce_can_dev,
						 rx_client);

	/* This is executed in IRQ context */
	if (rx_msg->error != LLCE_FW_SUCCESS)
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

static void get_ethtool_stats(struct net_device *dev,
			      struct ethtool_stats *stats,
			      u64 *data)
{
	struct llce_can_dev *llce = netdev_priv(dev);
	size_t i, j = 0;

	for (i = 0; i < ARRAY_SIZE(llce_errors); i++) {
		if (!llce_errors[i].is_can_error)
			continue;

		data[j] = llce->stats[i];
		j++;
	}
}

static void strncpylower(char *dest, const char *source, size_t len)
{
	size_t i;

	strncpy(dest, source, len);

	for (i = 0; i < len && dest[i]; i++)
		dest[i] = tolower(dest[i]);
}

static void get_strings(struct net_device *dev, u32 stringset, u8 *buf)
{
	char (*strings)[ETH_GSTRING_LEN] = (void *)buf;
	size_t error_prefix_len = strlen(LLCE_ERROR_PREFIX);
	size_t notif_prefix_len = strlen(LLCE_NOTIF_PREFIX);
	const char *err_name;
	size_t i, j = 0;

	for (i = 0; i < ARRAY_SIZE(llce_errors); i++) {
		if (!llce_errors[i].is_can_error)
			continue;

		err_name = llce_errors[i].name;

		if (!strncmp(err_name, LLCE_ERROR_PREFIX,
			     error_prefix_len))
			err_name += error_prefix_len;
		else if (!strncmp(err_name, LLCE_NOTIF_PREFIX,
				  notif_prefix_len))
			err_name += notif_prefix_len;

		strncpylower(strings[j], err_name, ETH_GSTRING_LEN);
		j++;
	}
}

static int get_sset_count(struct net_device *dev, int sset)
{
	int i, count = 0;

	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	for (i = 0; i < ARRAY_SIZE(llce_errors); i++) {
		if (!llce_errors[i].is_can_error)
			continue;
		count++;
	}

	return count;
}

static const struct ethtool_ops llce_can_common_ethtool_ops = {
	.get_ethtool_stats = get_ethtool_stats,
	.get_strings = get_strings,
	.get_sset_count = get_sset_count,
};

struct llce_can_dev *init_llce_can_dev(struct device *dev, size_t priv_size,
				       const char *basename)
{
	char *dev_name;
	struct llce_can_dev *llce = NULL;
	struct net_device *netdev;
	int id, ret = 0;

	netdev = alloc_candev(priv_size, 1);
	if (!netdev)
		return ERR_PTR(-ENOMEM);

	id = get_netdev_id(dev);
	if (id < 0) {
		ret = id;
		goto free_mem;
	}

	dev_name = get_netdev_name(dev, basename, id);
	if (IS_ERR_VALUE(dev_name)) {
		ret = PTR_ERR(dev_name);
		goto free_mem;
	}

	strncpy(netdev->name, dev_name, sizeof(netdev->name) - 1);
	SET_NETDEV_DEV(netdev, dev);

	llce = netdev_priv(netdev);
	llce->id = id;
	llce->stats = devm_kcalloc(dev, ARRAY_SIZE(llce_errors),
				   sizeof(*llce->stats), GFP_KERNEL);
	if (!llce->stats) {
		ret = -ENOMEM;
		goto free_mem;
	}

	init_llce_rx_client(llce, dev);

	netdev->ethtool_ops = &llce_can_common_ethtool_ops;

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

int llce_send_config_cmd(struct mbox_chan *conf_chan,
			 struct llce_config_msg *msg,
			 struct completion *config_done)
{
	struct device *dev = llce_can_chan_dev(conf_chan);
	enum llce_fw_return cmd_ret;
	int ret;

	ret = mbox_send_message(conf_chan, msg);
	if (ret < 0)
		return ret;

	wait_for_completion(config_done);
	switch (msg->cmd) {
	case LLCE_EXECUTE_FW_CMD:
	case LLCE_EXECUTE_FW_HIF_CMD:
		cmd_ret = msg->fw_cmd.cmd.return_value;
		if (cmd_ret == LLCE_ERROR_COMMAND_NOTSUPPORTED)
			return -EOPNOTSUPP;

		if (cmd_ret != LLCE_FW_SUCCESS) {
			dev_err(dev, "LLCE FW error %d\n",
				msg->fw_cmd.cmd.return_value);
			return -EIO;
		}

		break;
	case LLCE_EXECUTE_SW_CMD:
		break;
	default:
		dev_err(dev, "Unknown command for CAN cfg channel %u\n",
			msg->cmd);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(llce_send_config_cmd);
