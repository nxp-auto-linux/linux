// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright 2020 NXP
 */
#include <dt-bindings/mailbox/nxp-llce-mb.h>
#include <linux/can/dev.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <linux/mailbox/nxp-llce/llce_interface_fifo.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/processor.h>
#include <linux/slab.h>
#include <uapi/linux/can.h>

#include "mailbox.h"

#define LLCE_FIFO_SIZE			0x400

#define LLCE_NFIFO_WITH_IRQ		16
#define LLCE_RXIN_N_FIFO		20

#define LLCE_NTXACK_FIFOS		21
#define LLCE_NRXOUT_FIFOS		21

#define LLCE_CAN_RXIN_ICSR_0_7		14
#define LLCE_CAN_RXIN_ICSR_8_15		15
#define LLCE_CAN_RXOUT_ICSR_0_7		16
#define LLCE_CAN_RXOUT_ICSR_8_15	17
#define LLCE_CAN_TXACK_ICSR_0_7		22
#define LLCE_CAN_TXACK_ICSR_8_15	23
#define LLCE_CAN_ICSR_N_ACK		8

#define LLCE_CAN_ICSR_RXIN_INDEX	0
#define LLCE_CAN_ICSR_RXOUT_INDEX	1
#define LLCE_CAN_ICSR_TXACK_INDEX	2

#define LLCE_CAN_COMPATIBLE "nxp,s32g274a-llce-can"

#define LLCE_ARR_ENTRY(BASE_INDEX, ENTRY) \
	[ENTRY - BASE_INDEX] = __stringify_1(ENTRY)

#define LLCE_ERROR_ENTRY(ERROR) \
	LLCE_ARR_ENTRY(LLCE_ERROR_TXACK_FIFO_FULL, ERROR)

#define LLCE_MODULE_ENTRY(MODULE) \
	LLCE_ARR_ENTRY(LLCE_TX, MODULE)

struct llce_icsr {
	uint8_t icsr0_num;
	uint8_t icsr8_num;
};

struct llce_fifoirq {
	int irq0;
	int irq8;
};

struct llce_mb {
	struct mbox_controller controller;
	struct llce_fifoirq rxin_irqs;
	struct llce_fifoirq rxout_irqs;
	struct llce_fifoirq txack_irqs;
	struct mutex txack_lock;
	struct llce_can_shared_memory *sh_mem;
	void __iomem *rxout_fifo;
	void __iomem *rxin_fifo;
	void __iomem *txack_fifo;
	void __iomem *blrout_fifo;
	void __iomem *blrin_fifo;
	void __iomem *icsr;
	struct clk *clk;
};

struct sram_pool {
	struct gen_pool *pool;
	size_t size;
	unsigned long vaddr;
	const char *name;
};

struct llce_mb_desc {
	unsigned int nchan;
	int (*startup)(struct mbox_chan *chan);
	void (*shutdown)(struct mbox_chan *chan);
};

static int llce_rx_startup(struct mbox_chan *chan);
static int llce_tx_startup(struct mbox_chan *chan);
static void llce_rx_shutdown(struct mbox_chan *chan);
static void llce_tx_shutdown(struct mbox_chan *chan);

const char *llce_errors[] = {
	LLCE_ERROR_ENTRY(LLCE_ERROR_TXACK_FIFO_FULL),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXOUT_FIFO_FULL),
	LLCE_ERROR_ENTRY(LLCE_ERROR_MB_NOTAVAILABLE),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_FRZ_EXIT),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_SYNC),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_FRZ_ENTER),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_LPM_EXIT),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_SRT_ENTER),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_ACKERR),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_CRCERR),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_BIT0ERR),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_BIT1ERR),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_FRMERR),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BCAN_STFERR),
	LLCE_ERROR_ENTRY(LLCE_ERROR_DATA_LOST),
	LLCE_ERROR_ENTRY(LLCE_ERROR_TXLUT_FULL),
	LLCE_ERROR_ENTRY(LLCE_ERROR_CMD_PROCESSING),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_SLOW_SEARCH),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_ACCESS_MODE),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_SEARCH_MODE),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_SLOW_OPERATION),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_INCOMPLETE_OP),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_OPERATING_MODE),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_INIT_SLOW_OP),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_DEINIT_SLOW_OP),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_INIT_OPERATING_MODE),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_DEINIT_OPERATING_MODE1),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXLUT_DEINIT_OPERATING_MODE2),
	LLCE_ERROR_ENTRY(LLCE_ERROR_HARDWARE_BUSOFF),
	LLCE_ERROR_ENTRY(LLCE_ERROR_CTRL_NOT_READY),
	LLCE_ERROR_ENTRY(LLCE_ERROR_BUSOFF),
	LLCE_ERROR_ENTRY(LLCE_ERROR_FIFO_LOG_FULL),
	LLCE_ERROR_ENTRY(LLCE_ERROR_CAN2CAN),
	LLCE_ERROR_ENTRY(LLCE_ERROR_COMMAND_PARAM),
	LLCE_ERROR_ENTRY(LLCE_ERROR_COMMAND_DEINIT_NOTSTOP),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXTOKENS_UNRETURNED),
	LLCE_ERROR_ENTRY(LLCE_ERROR_TXACK_NOT_READ),
	LLCE_ERROR_ENTRY(LLCE_ERROR_COMMAND_NOTSUPPORTED),
	LLCE_ERROR_ENTRY(LLCE_ERROR_COMMAND_NOTVALIDATED),
	LLCE_ERROR_ENTRY(LLCE_ERROR_COMMAND_NOTACCEPTED),
	LLCE_ERROR_ENTRY(LLCE_ERROR_COMMAND_INVALID_PARAMS),
	LLCE_ERROR_ENTRY(LLCE_ERROR_CTRL_NOT_STARTED),
	LLCE_ERROR_ENTRY(LLCE_ERROR_FRAME_NOT_DELIVERED),
	LLCE_ERROR_ENTRY(LLCE_ERROR_FRAME_NOT_DELIVERED_TO_AF),
	LLCE_ERROR_ENTRY(LLCE_ERROR_FRAME_NOT_DELIVERED_TO_HOST),
	LLCE_ERROR_ENTRY(LLCE_ERROR_LOST_INDEXES),
	LLCE_ERROR_ENTRY(LLCE_ERROR_FILTERS_FULL),
	LLCE_ERROR_ENTRY(LLCE_ERROR_FILTERS_NOTEXIST),
	LLCE_ERROR_ENTRY(LLCE_ERROR_FILTERS_MASK_EMPTY),
	LLCE_ERROR_ENTRY(LLCE_ERROR_FILTERS_RANGE_EMPTY),
	LLCE_ERROR_ENTRY(LLCE_ERROR_FILTERS_EM_EMPTY),
	LLCE_ERROR_ENTRY(LLCE_ERROR_IDX_NOT_VALID_HOST),
	LLCE_ERROR_ENTRY(LLCE_ERROR_IDX_NOT_VALID_LOG),
	LLCE_ERROR_ENTRY(LLCE_ERROR_INVALID_HOST_CORE),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXFRAME_NOT_DELIVERED_TO_HSE),
	LLCE_ERROR_ENTRY(LLCE_ERROR_TXFRAME_NOT_DELIVERED_TO_HSE),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RXFRAME_AUTH_ERROR),
	LLCE_ERROR_ENTRY(LLCE_ERROR_INVALID_REQUEST_FROM_TX),
	LLCE_ERROR_ENTRY(LLCE_ERROR_INVALID_REQUEST_FROM_RX),
	LLCE_ERROR_ENTRY(LLCE_ERROR_RX_SW_FIFO_EMPTY),
};

const char *llce_modules[] = {
	LLCE_MODULE_ENTRY(LLCE_TX),
	LLCE_MODULE_ENTRY(LLCE_RX),
	LLCE_MODULE_ENTRY(LLCE_DTE),
	LLCE_MODULE_ENTRY(LLCE_FRPE),
};

static const struct llce_mb_desc mb_map[] = {
	[S32G274_LLCE_HIF_CONF_MB] = {
		.nchan = 2,
	},
	[S32G274_LLCE_CAN_CONF_MB] = {
		.nchan = 16,
	},
	[S32G274_LLCE_CAN_RX_MB] = {
		.nchan = 16,
		.startup = llce_rx_startup,
		.shutdown = llce_rx_shutdown,
	},
	[S32G274_LLCE_CAN_TX_MB] = {
		.nchan = 16,
		.startup = llce_tx_startup,
		.shutdown = llce_tx_shutdown,
	},
};

static const struct llce_icsr icsrs[] = {
	[LLCE_CAN_ICSR_RXIN_INDEX] = {
		.icsr0_num = LLCE_CAN_RXIN_ICSR_0_7,
		.icsr8_num = LLCE_CAN_RXIN_ICSR_8_15,
	},
	[LLCE_CAN_ICSR_RXOUT_INDEX] = {
		.icsr0_num = LLCE_CAN_RXOUT_ICSR_0_7,
		.icsr8_num = LLCE_CAN_RXOUT_ICSR_8_15,
	},
	[LLCE_CAN_ICSR_TXACK_INDEX] = {
		.icsr0_num = LLCE_CAN_TXACK_ICSR_0_7,
		.icsr8_num = LLCE_CAN_TXACK_ICSR_8_15,
	},
};

static const char *get_error_name(enum llce_can_error err)
{
	uint32_t index = err - LLCE_ERROR_TXACK_FIFO_FULL;

	if (index > ARRAY_SIZE(llce_errors))
		return "Undefined error";

	return llce_errors[index];
}

static const char *get_module_name(enum llce_can_module module)
{
	uint32_t index = module - LLCE_TX;

	if (index > ARRAY_SIZE(llce_modules))
		return "Unknown module";

	return llce_modules[index];
}

static unsigned int get_num_chans(void)
{
	size_t i;
	unsigned int num = 0;

	for (i = 0; i < ARRAY_SIZE(mb_map); i++)
		num += mb_map[i].nchan;

	return num;
}

static unsigned int get_channels_for_type(unsigned int type)
{
	return mb_map[type].nchan;
}

static unsigned int get_channel_offset(unsigned int type, unsigned int index)
{
	size_t i;
	unsigned int off = index;

	for (i = 0; i < ARRAY_SIZE(mb_map); i++) {
		if (type == i)
			return off;

		off += mb_map[i].nchan;
	}

	return off;
}

static bool is_tx_fifo_empty(void __iomem *tx_fifo)
{
	void __iomem *status = LLCE_FIFO_STATUS0(tx_fifo);

	return !(readl(status) & LLCE_FIFO_FNEMTY);
}

static void __iomem *get_fifo_by_index(void __iomem *fifo_base,
				       unsigned int max_index,
				       unsigned int index)
{
	if (index < max_index)
		return fifo_base + (LLCE_FIFO_SIZE * index);

	return NULL;
}

static void __iomem *get_rxin_by_index(struct llce_mb *mb, unsigned int index)
{
	return get_fifo_by_index(mb->rxin_fifo, LLCE_RXIN_N_FIFO, index);
}

static void __iomem *get_txack_by_index(struct llce_mb *mb, unsigned int index)
{
	return get_fifo_by_index(mb->txack_fifo, LLCE_NTXACK_FIFOS, index);
}

static void __iomem *get_rxout_by_index(struct llce_mb *mb, unsigned int index)
{
	return get_fifo_by_index(mb->rxout_fifo, LLCE_NRXOUT_FIFOS, index);
}

static void __iomem *get_host_rxin(struct llce_mb *mb, unsigned int host_index)
{
	if (host_index == LLCE_CAN_HIF0)
		return get_rxin_by_index(mb, 16);

	return get_rxin_by_index(mb, 18);
}

static void __iomem *get_host_notif(struct llce_mb *mb, unsigned int host_index)
{
	if (host_index == LLCE_CAN_HIF0)
		return get_rxin_by_index(mb, 0);

	return get_rxin_by_index(mb, 8);
}

static void __iomem *get_host_txack(struct llce_mb *mb, unsigned int host_index)
{
	if (host_index == LLCE_CAN_HIF0)
		return get_txack_by_index(mb, 17);

	return get_txack_by_index(mb, 18);
}

static void __iomem *get_txack_fifo(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	if (priv->type == S32G274_LLCE_HIF_CONF_MB)
		return get_host_txack(mb, LLCE_CAN_HIF0);

	return get_txack_by_index(mb, priv->index);
}

static void __iomem *get_rxout_fifo(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	return get_rxout_by_index(mb, priv->index);
}

static void __iomem *get_blrout_fifo(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	return get_fifo_by_index(mb->blrout_fifo, LLCE_NFIFO_WITH_IRQ,
				 priv->index);
}

static void __iomem *get_blrin_fifo(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	return get_fifo_by_index(mb->blrin_fifo, LLCE_NFIFO_WITH_IRQ,
				 priv->index);
}

static bool is_config_chan(unsigned int chan_type)
{
	return chan_type == S32G274_LLCE_CAN_CONF_MB ||
		chan_type == S32G274_LLCE_HIF_CONF_MB;
}

static int init_chan_priv(struct mbox_chan *chan, struct llce_mb *mb,
			  unsigned int type, unsigned int index)
{
	struct llce_chan_priv *priv;

	priv = kmalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->mb = mb;
	priv->type = type;
	priv->index = index;

	chan->con_priv = priv;

	/* Polling for firmware configuration */
	if (is_config_chan(type)) {
		chan->txdone_method = TXDONE_BY_POLL;
		priv->state = LLCE_REGISTERED_CHAN;
	} else {
		chan->txdone_method = TXDONE_BY_IRQ;
		priv->state = LLCE_UNREGISTERED_CHAN;
	}

	spin_lock_init(&priv->lock);

	return 0;
}

static void deinit_chan_priv(struct mbox_chan *chan)
{
	kfree(chan->con_priv);
}

static struct mbox_chan *llce_mb_xlate(struct mbox_controller *mbox,
				       const struct of_phandle_args *args)
{
	struct llce_mb *mb = container_of(mbox, struct llce_mb, controller);
	struct device *dev = mbox->dev;
	struct mbox_chan *chan;
	unsigned int type = args->args[0];
	unsigned int index = args->args[1];
	unsigned int off;
	int ret;

	if (type >= ARRAY_SIZE(mb_map)) {
		dev_err(dev, "%u is not a valid channel type\n", type);
		return ERR_PTR(-EINVAL);
	}

	if (index >= get_channels_for_type(type)) {
		dev_err(dev, "%u exceeds the number of allocated channels for type : %d\n",
			index, type);
		return ERR_PTR(-EINVAL);
	}

	off = get_channel_offset(type, index);
	if (off >= mbox->num_chans) {
		dev_err(dev, "Out of bounds access\n");
		return ERR_PTR(-EINVAL);
	}

	chan = &mbox->chans[off];
	ret = init_chan_priv(chan, mb, type, index);
	if (ret)
		return ERR_PTR(ret);

	return chan;
}

static int execute_config_cmd(struct mbox_chan *chan,
			      struct llce_can_command *cmd)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	unsigned int idx = priv->index;
	struct llce_can_command *sh_cmd;
	void __iomem *txack, *push0;
	int ret = 0;

	mutex_lock_io(&mb->txack_lock);

	txack = get_host_txack(mb, LLCE_CAN_HIF0);

	sh_cmd = &mb->sh_mem->can_cmd[idx];
	push0 = LLCE_FIFO_PUSH0(txack);

	if (!is_tx_fifo_empty(txack)) {
		ret = -EBUSY;
		goto release_lock;
	}

	priv->last_msg = cmd;
	memcpy(sh_cmd, cmd, sizeof(*cmd));
	sh_cmd->return_value = LLCE_CAN_NOTRUN;

	/* Trigger an interrupt to the LLCE */
	writel(idx, push0);

release_lock:
	mutex_unlock(&mb->txack_lock);
	return ret;
}

static bool is_blrin_full(struct mbox_chan *chan)
{
	void __iomem *blrin = get_blrin_fifo(chan);
	void __iomem *status1 = LLCE_FIFO_STATUS1(blrin);

	return !!(readl(status1) & LLCE_FIFO_FFULLD);
}

static uint32_t build_word0(bool rtr, bool ide, uint32_t std_id,
			    uint32_t ext_id)
{
	uint32_t word0;

	if (ide) {
		word0 = ext_id << CAN_SFF_ID_BITS | std_id | LLCE_CAN_MB_IDE;
	} else {
		word0 = (std_id << LLCE_CAN_MB_IDSTD_SHIFT);

		/* No retransmission with CAN FD */
		if (rtr)
			word0 |= LLCE_CAN_MB_RTR;
	}

	return word0;
}

static int send_can_msg(struct mbox_chan *chan, struct llce_tx_msg *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	void __iomem *blrout = get_blrout_fifo(chan);
	void __iomem *pop0 = LLCE_FIFO_POP0(blrout);
	void __iomem *blrin = get_blrin_fifo(chan);
	void __iomem *push0 = LLCE_FIFO_PUSH0(blrin);
	struct llce_can_shared_memory *sh_cmd = mb->sh_mem;
	unsigned int idx = priv->index;
	struct canfd_frame *cf = msg->cf;
	uint32_t mb_index;
	uint16_t frame_index;
	uint32_t word0, std_id, ext_id;
	u8 dlc, *payload;
	uint32_t mb_config;

	/* Get a free message buffer from BLROUT queue */
	mb_index = readl(pop0);

	if (mb_index == LLCE_FIFO_NULL_VALUE) {
		pr_err("All LLCE buffers are in use\n");
		return -EAGAIN;
	}

	mb_index &= LLCE_CAN_CONFIG_FIFO_FIXED_MASK;

	std_id = cf->can_id & CAN_SFF_MASK;
	ext_id = (cf->can_id & CAN_EFF_MASK) >> CAN_SFF_ID_BITS;

	word0 = build_word0(!!(cf->can_id & CAN_RTR_FLAG),
			    !!(cf->can_id & CAN_EFF_FLAG),
			    std_id, ext_id);

	/* Get the index of the frame reserved by the firmware */
	frame_index = sh_cmd->can_tx_mb_desc[mb_index].mb_frame_idx;
	/* Set CAN ID */
	sh_cmd->can_mb[frame_index].word0 = word0;
	/* Attach a token (channel ID) to be used in ACK handler */
	sh_cmd->can_tx_mb_desc[mb_index].frame_tag1 = idx;
	sh_cmd->can_tx_mb_desc[mb_index].enable_tx_frame_mac = false;
	/* Set the notification interface */
	sh_cmd->can_tx_mb_desc[mb_index].ack_interface = idx;
	payload = &sh_cmd->can_mb[frame_index].payload[0];

	memcpy(payload, cf->data, cf->len);
	dlc = can_len2dlc(cf->len);

	mb_config = dlc;
	if (msg->fd) {
		/* Configure the tx mb as a CAN FD frame. */
		mb_config |= LLCE_CAN_MB_FDF;

		/* Enable BRS feature to allow receiveing of CAN FD frames */
		if (cf->flags & CANFD_BRS)
			mb_config |= LLCE_CAN_MB_BRS;

		if (cf->flags & CANFD_ESI)
			mb_config |= LLCE_CAN_MB_ESI;
	}

	sh_cmd->can_mb[frame_index].word1 = mb_config;

	spin_until_cond(!is_blrin_full(chan));

	/* Submit the buffer in BLRIN queue */
	writel(mb_index, push0);

	return 0;
}

static int llce_mb_send_data(struct mbox_chan *chan, void *data)
{
	struct llce_chan_priv *priv = chan->con_priv;
	int ret = -EINVAL;

	if (is_config_chan(priv->type))
		ret = execute_config_cmd(chan, data);

	if (priv->type == S32G274_LLCE_CAN_TX_MB)
		ret = send_can_msg(chan, data);

	return ret;
}

static int llce_rx_startup(struct mbox_chan *chan)
{
	void __iomem *rxout = get_rxout_fifo(chan);
	void __iomem *status0 = LLCE_FIFO_STATUS0(rxout);
	void __iomem *ier = LLCE_FIFO_IER(rxout);
	struct llce_chan_priv *priv = chan->con_priv;
	unsigned long flags;

	/* State change must go under the lock protection */
	spin_lock_irqsave(&priv->lock, flags);

	priv->state = LLCE_REGISTERED_CHAN;

	/* Clear interrupt status flags. */
	writel(readl(status0), status0);
	/* Enable interrupt */
	writel(LLCE_FIFO_FNEMTY, ier);

	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;
}

static void llce_rx_shutdown(struct mbox_chan *chan)
{
	void __iomem *rxout = get_rxout_fifo(chan);
	void __iomem *ier = LLCE_FIFO_IER(rxout);
	struct llce_chan_priv *priv = chan->con_priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->state = LLCE_UNREGISTERED_CHAN;

	/* Disable interrupts */
	writel(0, ier);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static void enable_bus_off_irq(struct llce_mb *mb)
{
	void __iomem *rxin = get_host_notif(mb, LLCE_CAN_HIF0);
	void __iomem *ier = LLCE_FIFO_IER(rxin);
	uint32_t ier_val;

	/* Enable BusOff for host 0 only */
	ier_val = readl(ier) | LLCE_FIFO_FNEMTY;
	writel(ier_val, ier);
}

static int llce_tx_startup(struct mbox_chan *chan)
{
	void __iomem *txack = get_txack_fifo(chan);
	void __iomem *status1 = LLCE_FIFO_STATUS1(txack);
	void __iomem *ier = LLCE_FIFO_IER(txack);
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->state = LLCE_REGISTERED_CHAN;

	/* Clear interrupt status flags. */
	writel(readl(status1), status1);
	/* Enable interrupt */
	writel(LLCE_FIFO_FNEMTY, ier);
	enable_bus_off_irq(mb);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void llce_tx_shutdown(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	void __iomem *txack = get_txack_fifo(chan);
	void __iomem *ier = LLCE_FIFO_IER(txack);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->state = LLCE_UNREGISTERED_CHAN;

	/* Disable interrupts */
	writel(0, ier);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static int llce_mb_startup(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;

	if (mb_map[priv->type].startup)
		return mb_map[priv->type].startup(chan);

	return 0;
}

static void llce_mbox_chan_received_data(struct mbox_chan *chan, void *mssg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if (priv->state == LLCE_REGISTERED_CHAN)
		mbox_chan_received_data(chan, mssg);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static bool llce_mb_last_tx_done(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	unsigned int idx = priv->index;
	void __iomem *txack;
	struct llce_mb *mb = priv->mb;
	struct llce_can_command *cmd;
	struct llce_can_command *sh_cmd;

	if (!is_config_chan(priv->type))
		return false;

	mutex_lock_io(&mb->txack_lock);

	txack = get_host_txack(mb, LLCE_CAN_HIF0);

	/* Wait an answer from LLCE FW */
	spin_until_cond(is_tx_fifo_empty(txack));

	cmd = priv->last_msg;
	sh_cmd = &mb->sh_mem->can_cmd[idx];

	memcpy(cmd, sh_cmd, sizeof(*cmd));

	mutex_unlock(&mb->txack_lock);

	if (priv->type != S32G274_LLCE_HIF_CONF_MB)
		llce_mbox_chan_received_data(chan, cmd);

	return true;
}

static void llce_mb_shutdown(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;

	if (mb_map[priv->type].shutdown)
		mb_map[priv->type].shutdown(chan);

	deinit_chan_priv(chan);
}

static const struct mbox_chan_ops llce_mb_ops = {
	.send_data = llce_mb_send_data,
	.startup = llce_mb_startup,
	.shutdown = llce_mb_shutdown,
	.last_tx_done = llce_mb_last_tx_done,
};

static void devm_sram_pool_release(struct device *dev, void *res)
{
	struct sram_pool *spool = res;

	gen_pool_free(spool->pool, spool->vaddr, spool->size);
}

static struct sram_pool *devm_sram_pool_alloc(struct device *dev,
					      struct gen_pool *pool)
{
	struct sram_pool *spool = devres_alloc(devm_sram_pool_release,
					       sizeof(*spool), GFP_KERNEL);

	if (!spool)
		return ERR_PTR(-ENOMEM);

	spool->size = gen_pool_size(pool);
	spool->vaddr = gen_pool_alloc(pool, spool->size);
	if (!spool->vaddr) {
		devres_free(spool);
		return ERR_PTR(-ENOMEM);
	}

	spool->pool = pool;
	devres_add(dev, spool);

	return spool;
}

static int alloc_sram_pool(struct platform_device *pdev,
			   struct llce_mb *mb)
{
	struct device_node *sram_node;
	struct device *dev = &pdev->dev;
	struct gen_pool *pool;
	const char *name;
	struct sram_pool *spool;
	struct platform_device *sram_pdev;

	sram_node = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!sram_node) {
		dev_err(&pdev->dev, "Failed to get 'memory-region'\n");
		return -EIO;
	}

	name = sram_node->name;
	sram_pdev = of_find_device_by_node(sram_node);
	if (!sram_pdev) {
		dev_err(dev, "failed to find sram device for '%s'!\n", name);
		return -ENODEV;
	}

	pool = gen_pool_get(&sram_pdev->dev, NULL);
	if (!pool) {
		dev_err(dev, "Pool '%s' is unavailable!\n", name);
		return -ENODEV;
	}

	spool = devm_sram_pool_alloc(dev, pool);
	if (IS_ERR(spool)) {
		dev_err(dev, "Unable to alloc '%s' pool\n", name);
		return PTR_ERR(spool);
	}

	mb->sh_mem = (void *)spool->vaddr;

	return 0;
}

static void __iomem *get_icsr(struct llce_mb *mb, struct llce_fifoirq *irqs,
			      uint32_t icsr_index, int irq,
			      uint8_t *base_id)
{
	uint32_t icsr_id;
	const struct llce_icsr *icsrs_conf = &icsrs[icsr_index];

	if (irq == irqs->irq0) {
		icsr_id = icsrs_conf->icsr0_num;
		*base_id = 0;
	} else {
		icsr_id = icsrs_conf->icsr8_num;
		*base_id = 8;
	}

	return mb->icsr + icsr_id * sizeof(uint32_t);
}

static void __iomem *get_txack_icsr(struct llce_mb *mb, int irq,
				    uint8_t *base_id)
{
	return get_icsr(mb, &mb->txack_irqs, LLCE_CAN_ICSR_TXACK_INDEX,
			irq, base_id);
}

static void __iomem *get_rxin_icsr(struct llce_mb *mb, int irq,
				   uint8_t *base_id)
{
	return get_icsr(mb, &mb->rxin_irqs, LLCE_CAN_ICSR_RXIN_INDEX,
			irq, base_id);
}

static void __iomem *get_rxout_icsr(struct llce_mb *mb, int irq,
				   uint8_t *base_id)
{
	return get_icsr(mb, &mb->rxout_irqs, LLCE_CAN_ICSR_RXOUT_INDEX,
			irq, base_id);
}

static void llce_process_tx_ack(struct llce_mb *mb, uint8_t index)
{
	void __iomem *tx_ack = get_txack_by_index(mb, index);
	void __iomem *status1 = LLCE_FIFO_STATUS1(tx_ack);
	void __iomem *pop0 = LLCE_FIFO_POP0(tx_ack);
	struct mbox_controller *ctrl = &mb->controller;
	struct llce_can_shared_memory *sh_mem = mb->sh_mem;
	struct llce_can_tx2host_ack_info *info;
	struct llce_notif notif;
	uint32_t ack_id;
	unsigned int chan_index;

	while (!(readl(status1) & LLCE_FIFO_FEMTYD)) {
		/* Get ACK mailbox */
		ack_id = readl(pop0) & LLCE_CAN_CONFIG_FIFO_FIXED_MASK;

		info = &sh_mem->can_tx_ack_info[ack_id];
		chan_index = get_channel_offset(S32G274_LLCE_CAN_TX_MB,
						info->frame_tag1);
		notif.error = 0;
		notif.tx_timestamp = info->tx_timestamp;

		/* Notify the client and send the timestamp */
		llce_mbox_chan_received_data(&ctrl->chans[chan_index], &notif);
		mbox_chan_txdone(&ctrl->chans[chan_index], 0);
	}

	/* Clear the interrupt status flag. */
	writel(LLCE_FIFO_FNEMTY, status1);
}

static void process_chan_err(struct llce_mb *mb, uint32_t chan_type,
			     struct llce_can_channel_error_notif *error)
{
	unsigned int chan_index;
	struct mbox_controller *ctrl = &mb->controller;
	struct llce_notif notif = {
		.error = error->error_info.error_code,
	};

	chan_index = get_channel_offset(chan_type, error->hw_ctrl);
	notif.error = error->error_info.error_code;

	/* Release the channel if an error occurred */
	if (chan_type == S32G274_LLCE_CAN_TX_MB)
		mbox_chan_txdone(&ctrl->chans[chan_index], 0);

	llce_mbox_chan_received_data(&ctrl->chans[chan_index], &notif);
}

static void process_channel_err(struct llce_mb *mb,
				struct llce_can_channel_error_notif *error)
{
	enum llce_can_module module_id = error->error_info.module_id;

	dev_warn(mb->controller.dev, "Error module:%s Error:%s HW module:%d\n",
		 get_module_name(module_id),
		 get_error_name(error->error_info.error_code),
		 error->hw_ctrl);

	switch (module_id) {
	case LLCE_TX:
		return process_chan_err(mb, S32G274_LLCE_CAN_TX_MB, error);
	case LLCE_RX:
		return process_chan_err(mb, S32G274_LLCE_CAN_RX_MB, error);
	default:
		break;
	}
}

static void process_platform_err(struct llce_mb *mb,
				 struct llce_can_error_notif *error)
{
}

static void process_ctrl_err(struct llce_mb *mb,
			     struct llce_can_ctrl_mode_notif *error)
{
}

static void llce_process_rxin(struct llce_mb *mb, uint8_t index)
{
	void __iomem *rxin = get_rxin_by_index(mb, index);
	void __iomem *status1 = LLCE_FIFO_STATUS1(rxin);
	void __iomem *pop0 = LLCE_FIFO_POP0(rxin);
	struct llce_can_shared_memory *sh_mem = mb->sh_mem;
	struct llce_can_notification_table *table;
	struct llce_can_notification *notif;
	union llce_can_notification_list *list;

	uint32_t rxin_id;

	while (!(readl(status1) & LLCE_FIFO_FEMTYD)) {
		/* Get notification mailbox */
		rxin_id = readl(pop0) & LLCE_CAN_CONFIG_FIFO_FIXED_MASK;
		table = &sh_mem->can_notification_table;
		notif = &table->can_notif_intr_table[LLCE_CAN_HIF0][rxin_id];
		list = &notif->notif_list;

		switch (notif->notif_id) {
		case LLCE_CAN_NOTIF_CHANNELERROR:
			process_channel_err(mb, &list->channel_error);
			break;
		case LLCE_CAN_NOTIF_PLATFORMERROR:
			process_platform_err(mb, &list->platform_error);
			break;
		case LLCE_CAN_NOTIF_CTRLMODE:
			process_ctrl_err(mb, &list->ctrl_mode);
			break;
		case LLCE_CAN_NOTIF_NOERROR:
			break;
		}
	}

	/* Clear the interrupt status flag. */
	writel(LLCE_FIFO_FNEMTY, status1);
}

static void read_rxout_message(struct llce_mb *mb, uint32_t rx_mb,
			       uint8_t rx_index)
{
	struct mbox_controller *ctrl = &mb->controller;
	struct llce_can_shared_memory *sh_mem = mb->sh_mem;
	struct llce_notif notif = {
		.error = 0,
	};
	uint32_t frame_id;
	unsigned int chan_index;

	frame_id = sh_mem->can_rx_mb_desc[rx_mb].mb_frame_idx;
	chan_index = get_channel_offset(S32G274_LLCE_CAN_RX_MB, rx_index);
	notif.can_mb = &sh_mem->can_mb[frame_id];
	llce_mbox_chan_received_data(&ctrl->chans[chan_index], &notif);
}

static void llce_process_rxout(struct llce_mb *mb, uint8_t index)
{
	void __iomem *host_rxin = get_host_rxin(mb, LLCE_CAN_HIF0);
	void __iomem *rxout = get_rxout_by_index(mb, index);
	void __iomem *status1 = LLCE_FIFO_STATUS1(rxout);
	void __iomem *pop0 = LLCE_FIFO_POP0(rxout);
	void __iomem *host_push0 = LLCE_FIFO_PUSH0(host_rxin);
	uint32_t rx_mb;

	while (!(readl(status1) & LLCE_FIFO_FEMTYD)) {
		/* Get RX mailbox */
		rx_mb = readl(pop0) & LLCE_CAN_CONFIG_FIFO_FIXED_MASK;
		/* Process RX message */
		read_rxout_message(mb, rx_mb, index);
		/* Make the index available for another reception flow */
		writel(rx_mb, host_push0);
	}

	/* Clear the interrupt status flag. */
	writel(LLCE_FIFO_FNEMTY, status1);
}

typedef void (*icsr_consumer_t)(struct llce_mb *, uint8_t);

static void llce_consume_icsr(struct llce_mb *mb, void __iomem *icsr_addr,
			      uint8_t base, icsr_consumer_t callback)
{
	uint32_t icsr;
	uint8_t i;

	icsr = readl(icsr_addr);
	for (i = 0; i < LLCE_CAN_ICSR_N_ACK; i++) {
		if (!(icsr & BIT(i)))
			continue;

		callback(mb, base + i);
	}
}

static irqreturn_t llce_rxin_fifo_irq(int irq, void *data)
{
	struct llce_mb *mb = data;
	uint8_t base_icsr;
	void __iomem *icsr_addr = get_rxin_icsr(mb, irq, &base_icsr);

	llce_consume_icsr(mb, icsr_addr, base_icsr, llce_process_rxin);

	return IRQ_HANDLED;
}

static irqreturn_t llce_txack_fifo_irq(int irq, void *data)
{
	struct llce_mb *mb = data;
	uint8_t base_icsr;
	void __iomem *icsr_addr = get_txack_icsr(mb, irq, &base_icsr);

	llce_consume_icsr(mb, icsr_addr, base_icsr, llce_process_tx_ack);

	return IRQ_HANDLED;
}

static irqreturn_t llce_rxout_fifo_irq(int irq, void *data)
{
	struct llce_mb *mb = data;
	uint8_t base_icsr;
	void __iomem *icsr_addr = get_rxout_icsr(mb, irq, &base_icsr);

	llce_consume_icsr(mb, icsr_addr, base_icsr, llce_process_rxout);

	return IRQ_HANDLED;
}

static int init_llce_irq_resources(struct platform_device *pdev,
				   struct llce_mb *mb)
{
	int irq, ret;
	size_t i;
	struct device *dev = &pdev->dev;
	struct {
		const char *name;
		int *irq;
		irq_handler_t handler;
	} resources[] = {
		{
			.name = "rxin_fifo_0_7",
			.irq = &mb->rxin_irqs.irq0,
			.handler = llce_rxin_fifo_irq,
		},
		{
			.name = "rxin_fifo_8_15",
			.irq = &mb->rxin_irqs.irq8,
			.handler = llce_rxin_fifo_irq,
		},
		{
			.name = "rxout_fifo_0_7",
			.irq = &mb->rxout_irqs.irq0,
			.handler = llce_rxout_fifo_irq,
		},
		{
			.name = "rxout_fifo_8_15",
			.irq = &mb->rxout_irqs.irq8,
			.handler = llce_rxout_fifo_irq,
		},
		{
			.name = "txack_fifo_0_7",
			.irq = &mb->txack_irqs.irq0,
			.handler = llce_txack_fifo_irq,
		},
		{
			.name = "txack_fifo_8_15",
			.irq = &mb->txack_irqs.irq8,
			.handler = llce_txack_fifo_irq,
		},
	};

	for (i = 0; i < ARRAY_SIZE(resources); i++) {
		irq = platform_get_irq_byname(pdev, resources[i].name);
		if (irq < 0) {
			dev_err(dev, "Failed to request '%s' IRQ\n",
				resources[i].name);
			return irq;
		}

		*resources[i].irq = irq;
		ret = devm_request_irq(dev, irq, resources[i].handler,
				       IRQF_SHARED,
				       resources[i].name, mb);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to register '%s' IRQ\n",
				resources[i].name);
			return ret;
		}
	}

	return 0;
}

static int init_llce_mem_resources(struct platform_device *pdev,
				   struct llce_mb *mb)
{
	size_t i;
	struct resource *res;
	void __iomem *vaddr;
	struct device *dev = &pdev->dev;
	struct {
		const char *res_name;
		void __iomem **vaddr;
	} resources[] = {
		{ .res_name = "rxout_fifo", .vaddr = &mb->rxout_fifo, },
		{ .res_name = "txack_fifo", .vaddr = &mb->txack_fifo, },
		{ .res_name = "blrout_fifo", .vaddr = &mb->blrout_fifo, },
		{ .res_name = "blrin_fifo", .vaddr = &mb->blrin_fifo, },
		{ .res_name = "rxin_fifo", .vaddr = &mb->rxin_fifo, },
		{ .res_name = "icsr", .vaddr = &mb->icsr, },
	};

	for (i = 0; i < ARRAY_SIZE(resources); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   resources[i].res_name);
		if (!res) {
			dev_err(dev, "Missing '%s' reg region.\n",
				resources[i].res_name);
			return -EIO;
		}

		vaddr = devm_ioremap(dev, res->start,
				     resource_size(res));
		if (!vaddr) {
			dev_err(dev, "Failed to map '%s'\n",
				resources[i].res_name);
			return -ENOMEM;
		}

		*resources[i].vaddr = vaddr;
	}

	return 0;
}

static int get_llce_can_id(const char *node_name, unsigned long *id)
{
	const char *p = node_name + strlen(node_name) - 1;

	while (isdigit(*p))
		p--;

	return kstrtoul(p + 1, 10, id);
}

static struct mbox_chan *get_hif_cfg_chan(struct llce_mb *mb)
{
	struct mbox_controller *ctrl = &mb->controller;
	unsigned int chan_index;

	chan_index = get_channel_offset(S32G274_LLCE_HIF_CONF_MB,
					LLCE_CAN_HIF0);
	return &ctrl->chans[chan_index];
}

static int init_hif_config_chan(struct llce_mb *mb)
{
	struct mbox_chan *chan = get_hif_cfg_chan(mb);

	return init_chan_priv(chan, mb, S32G274_LLCE_HIF_CONF_MB,
			      LLCE_CAN_HIF0);
}

static void deinit_hif_config_chan(struct llce_mb *mb)
{
	struct mbox_chan *chan = get_hif_cfg_chan(mb);

	deinit_chan_priv(chan);
}

static int execute_hif_cmd(struct llce_mb *mb,
			   struct llce_can_command *cmd)
{
	struct mbox_controller *ctrl = &mb->controller;
	struct device *dev = ctrl->dev;
	static struct mbox_chan *chan;
	int ret;

	chan = get_hif_cfg_chan(mb);

	ret = execute_config_cmd(chan, cmd);
	if (ret) {
		dev_err(dev, "Failed to send command\n");
		return ret;
	}

	/* Wait for command completion */
	if (!llce_mb_last_tx_done(chan))
		return -EIO;

	if (cmd->return_value != LLCE_CAN_SUCCESS) {
		dev_err(dev, "LLCE FW error %d\n", cmd->return_value);
		return -EIO;
	}

	return 0;
}

static int llce_platform_init(struct device *dev, struct llce_mb *mb)
{
	struct device_node *child, *parent;
	struct llce_can_init_platform_cmd *pcmd;
	const char *node_name;
	unsigned long id;
	int ret;

	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_INIT_PLATFORM,
		.cmd_list.init_platform = {
			.can_error_reporting = {
				.can_protocol_err = INTERRUPT,
				.data_lost_err = INTERRUPT,
				.init_err = INTERRUPT,
				.internal_err = INTERRUPT,
			},
		},
	};

	pcmd = &cmd.cmd_list.init_platform;
	memset(&pcmd->ctrl_init_status, UNINITIALIZED,
	       sizeof(pcmd->ctrl_init_status));
	memset(&pcmd->max_int_tx_ack_count, 0,
	       sizeof(pcmd->max_int_tx_ack_count));
	memset(&pcmd->max_poll_tx_ack_count, 0,
	       sizeof(pcmd->max_poll_tx_ack_count));
	memset(&pcmd->can_error_reporting.bus_off_err, IGNORE,
	       sizeof(pcmd->can_error_reporting.bus_off_err));
	memset(&pcmd->max_filter_count, 0, sizeof(pcmd->max_filter_count));
	memset(&pcmd->max_int_mb_count, 0, sizeof(pcmd->max_int_mb_count));
	memset(&pcmd->max_poll_mb_count, 0, sizeof(pcmd->max_poll_mb_count));

	parent = dev->of_node->parent;

	for_each_child_of_node(dev->of_node->parent, child) {
		if (!(of_device_is_compatible(child, LLCE_CAN_COMPATIBLE) &&
		      of_device_is_available(child)))
			continue;

		node_name = child->name;
		ret = get_llce_can_id(node_name, &id);
		if (ret) {
			dev_err(dev, "Failed to get ID of the node: %s\n",
				node_name);
			return ret;
		}

		if (id >= LLCE_NFIFO_WITH_IRQ)
			continue;

		pcmd->ctrl_init_status[id] = INITIALIZED;
		pcmd->max_filter_count[id] = 16;
		pcmd->max_int_mb_count[id] = 100;
		pcmd->max_int_tx_ack_count[id] = 16;
		pcmd->can_error_reporting.bus_off_err[id] = INTERRUPT;
	}

	return execute_hif_cmd(mb, &cmd);
}

static int llce_platform_deinit(struct llce_mb *mb)
{
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_DEINIT_PLATFORM,
	};

	return execute_hif_cmd(mb, &cmd);
}

static int print_fw_version(struct llce_mb *mb)
{
	struct mbox_controller *ctrl = &mb->controller;
	struct device *dev = ctrl->dev;
	struct llce_can_get_fw_version *ver;
	char *ver_str;
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_GETFWVERSION,
	};
	int ret;

	ret = execute_hif_cmd(mb, &cmd);
	if (ret)
		return ret;

	ver = &cmd.cmd_list.get_fw_version;
	ver_str = ver->version_string;

	dev_info(dev, "LLCE interface version: %c\n",
		 ver_str[LLCE_VERSION_INTERFACE]);
	dev_info(dev, "LLCE basic version: %c\n",
		 ver_str[LLCE_VERSION_BASIC_FUNC]);
	dev_info(dev, "LLCE CAN2CAN version: %c\n",
		 ver_str[LLCE_VERSION_ROUTING_CAN2CAN]);
	dev_info(dev, "LLCE CAN2ETH version: %c\n",
		 ver_str[LLCE_VERSION_ROUTING_CAN2ETH]);
	dev_info(dev, "LLCE CAN logging version: %c\n",
		 ver_str[LLCE_VERSION_LOGGING]);

	return 0;
}

static int init_core_clock(struct device *dev, struct clk **clk)
{
	int ret;

	*clk = devm_clk_get(dev, "llce_sys");
	if (IS_ERR(*clk)) {
		dev_err(dev, "No clock available\n");
		return PTR_ERR(*clk);
	}

	ret = clk_prepare_enable(*clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	return 0;
}

static void deinit_core_clock(struct clk *clk)
{
	clk_disable_unprepare(clk);
}

static int llce_mb_probe(struct platform_device *pdev)
{
	struct mbox_controller *ctrl;
	struct llce_mb *mb;
	struct device *dev = &pdev->dev;
	int ret;

	mb = devm_kzalloc(&pdev->dev, sizeof(*mb), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;

	mutex_init(&mb->txack_lock);

	ctrl = &mb->controller;
	ctrl->txdone_irq = false;
	ctrl->txdone_poll = true;
	ctrl->txpoll_period = 1;
	ctrl->of_xlate = llce_mb_xlate;
	ctrl->num_chans = get_num_chans();
	ctrl->dev = dev;
	ctrl->ops = &llce_mb_ops;

	ctrl->chans = devm_kcalloc(dev, ctrl->num_chans,
				   sizeof(*ctrl->chans), GFP_KERNEL);
	if (!ctrl->chans)
		return -ENOMEM;

	platform_set_drvdata(pdev, mb);

	ret = init_llce_mem_resources(pdev, mb);
	if (ret)
		return ret;

	ret = init_llce_irq_resources(pdev, mb);
	if (ret)
		return ret;

	ret = alloc_sram_pool(pdev, mb);
	if (ret)
		return ret;

	ret = init_hif_config_chan(mb);
	if (ret) {
		dev_err(dev, "Failed to initialize HIF config channel\n");
		return ret;
	}

	ret = init_core_clock(dev, &mb->clk);
	if (ret)
		goto hif_deinit;

	ret = print_fw_version(mb);
	if (ret) {
		dev_err(dev, "Failed to get firmware version\n");
		goto disable_clk;
	}

	ret = llce_platform_init(dev, mb);
	if (ret) {
		dev_err(dev, "Failed to initialize platform\n");
		goto disable_clk;
	}

	ret = devm_mbox_controller_register(dev, ctrl);
	if (ret < 0) {
		dev_err(dev, "Failed to register can config mailbox: %d\n",
			ret);
		goto deinit_plat;
	}

deinit_plat:
	if (ret)
		llce_platform_deinit(mb);

disable_clk:
	if (ret)
		deinit_core_clock(mb->clk);

hif_deinit:
	if (ret)
		deinit_hif_config_chan(mb);

	return ret;
}

static int llce_mb_remove(struct platform_device *pdev)
{
	struct llce_mb *mb = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret;

	ret = llce_platform_deinit(mb);
	if (ret) {
		dev_err(dev, "Failed to deinitialize LLCE platform");
		return ret;
	}

	deinit_core_clock(mb->clk);
	deinit_hif_config_chan(mb);

	return 0;
}

static const struct of_device_id llce_mb_match[] = {
	{
		.compatible = "nxp,s32g274a-llce-mailbox",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, llce_mb_match);

static struct platform_driver llce_mb_driver = {
	.probe = llce_mb_probe,
	.remove = llce_mb_remove,
	.driver = {
		.name = "llce_mb",
		.of_match_table = llce_mb_match,
	},
};
module_platform_driver(llce_mb_driver)

MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("NXP LLCE Mailbox");
MODULE_LICENSE("GPL");
