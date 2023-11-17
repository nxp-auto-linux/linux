// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright 2020-2023 NXP
 */
#include <dt-bindings/mailbox/nxp-llce-mb.h>
#include <linux/can/dev.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <linux/mailbox/nxp-llce/llce_core.h>
#include <linux/mailbox/nxp-llce/llce_interface_core2core.h>
#include <linux/mailbox/nxp-llce/llce_interface_fifo.h>
#include <linux/mailbox/nxp-llce/llce_interface_lin.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/mailbox/nxp-llce/llce_sema42.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/processor.h>
#include <linux/slab.h>
#include <uapi/linux/can.h>

#include "mailbox.h"

#define LLCE_SYSRSTR			0x0

#define MAX_FILTER_PER_CHAN		(16u)
#define FIFO_MB_PER_CHAN		(100u)
#define MAX_CONFIRM_BUF_PER_CHAN	(16u)

#define LLCE_FIFO_SIZE			0x400
#define LLCE_RXMBEXTENSION_OFFSET	0x3C8F0U

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
#define LLCE_CAN_LOGGER_ICSR		24
#define LLCE_CAN_ICSR_N_ACK		8

#define LLCE_CAN_ICSR_RXIN_INDEX	0
#define LLCE_CAN_ICSR_RXOUT_INDEX	1
#define LLCE_CAN_ICSR_TXACK_INDEX	2

#define LLCE_CAN_LOGGER_IN_FIFO		16
#define LLCE_CAN_LOGGER_OUT_FIFO	17

#define LLCE_LOGGER_ICSR_IRQ		BIT(5)

#define LLCE_CAN_COMPATIBLE		"nxp,s32g-llce-can"
#define LLCE_SHMEM_REG_NAME		"shmem"

#define LLCE_MODULE_ENTRY(MODULE) \
	[MODULE - LLCE_TX] = __stringify_1(MODULE)

/* Mask to extract the hw ctrl where the frame comes from */
#define HWCTRL_MBEXTENSION_MASK		0x0FFU
#define HWCTRL_MBEXTENSION_SHIFT	24

#define LLCE_FEATURE_DISABLED	'_'

#define LLCE_LOGGING		(4)

#define UNINITIALIZED_FIFO		(U32_MAX)

#define LIN_MEM_OFFSET		(0x3C800U)
#define CAN_RX_TX_STATS_OFFSET	(0x4FD50)

#define LLCE_LINFLEX_NR		(4)
#define LLCE_LIN_CHANNEL0	(0)

#define LLCE_DELAY_US		(1000)

#define LLCE_LPSPI_NR		(4)
#define LLCE_LPSPI_CHANNEL0	(0)

/* LLCE Mailbox is an interrupt controller for LPSPI and LinflexD.
 * First LLCE_LINFLEX_NR interrupts are used for LinflexD instances
 * and next LLCE_LPSPI_NR interrupts are used for LPSPI instances.
 */
/* Get LinflexD/LPSPI instance corresponding to hw_irq. */
#define LLCE_IRQ_TO_LIN_HWCTRL(hw_irq)		(hw_irq)
#define LLCE_IRQ_TO_LPSPI_HWCTRL(hw_irq)	((hw_irq) - LLCE_LINFLEX_NR)

/* Get hw_irq corresponding to LinflexD/LPSPI instance. */
#define LLCE_LIN_HWTCRL_TO_IRQ(hw_irq)		(hw_irq)
#define LLCE_LPSPI_HWCTRL_TO_IRQ(hw_irq)	((hw_irq) + LLCE_LINFLEX_NR)

struct llce_icsr {
	u8 icsr0_num;
	u8 icsr8_num;
};

struct llce_fifoirq {
	int num;
	const char *name;
	irq_handler_t handler;
	bool registered;
};

struct llce_pair_irq {
	struct llce_fifoirq irq0;
	struct llce_fifoirq irq8;
};

struct fifos_ref_cnt {
	u8 rxout[LLCE_NFIFO_WITH_IRQ];
	u8 txack[LLCE_NFIFO_WITH_IRQ];
	/* spinlock used to protect above arrays */
	spinlock_t lock;
};

struct llce_chan_params {
	unsigned int fifo;
	u16 max_regular_filters;
	u16 max_adv_filters;
	u16 max_rx_mb;
	u16 max_tx_ack;
};

struct llce_mb {
	struct mbox_controller controller;
	struct llce_pair_irq rxin_irqs;
	struct llce_pair_irq rxout_irqs;
	struct llce_pair_irq txack_irqs;

	/* spinlock used to protect the execution of the config commands. */
	spinlock_t txack_lock;
	/* spinlock used to protect linflex interrupts related registers. */
	spinlock_t lin_lock;
	/* spinlock used to protect lpspi interrupts related registers. */
	spinlock_t lpspi_lock;

	struct llce_can_shared_memory __iomem *can_sh_mem;
	struct llce_lin_shared_memory __iomem *lin_sh_mem;
	void __iomem *status;
	void __iomem *rxout_fifo;
	void __iomem *rxin_fifo;
	void __iomem *txack_fifo;
	void __iomem *blrout_fifo;
	void __iomem *blrin_fifo;
	void __iomem *icsr;
	void __iomem *sema42;
	void __iomem *core2core;
	void __iomem *system_ctrl;
	struct llce_can_rx_tx_count __iomem *can_stats;
	struct clk *clk;
	struct device *dev;
	struct llce_chan_params chans_params[LLCE_CAN_CONFIG_MAXCTRL_COUNT];
	struct fifos_ref_cnt fifos_irq_ref_cnt;
	struct llce_fifoirq logger_irq;
	u32 hif_id;
	bool multihif;
	bool suspended;
	bool lin_irq_enabled;
	bool lpspi_irq_enabled;
	bool fw_logger_support;
	struct irq_chip irq_chip;
	struct irq_domain *domain;
	raw_spinlock_t wa_lock;
};

struct llce_mb_desc {
	const char *name;
	unsigned int nchan;
	int (*startup)(struct mbox_chan *chan);
	void (*shutdown)(struct mbox_chan *chan);
};

struct llce_rx_data {
	struct llce_rx_can_mb mb;
	u32 index;
	bool has_leftover;
};

typedef void (*enable_irq_cb_t)(void *);

struct shared_rxout_data {
	struct llce_mb *mb;
	void __iomem *rxout;
	bool enable;
};

struct shared_txack_data {
	struct llce_mb *mb;
	void __iomem *txack;
	bool enable;
};

static int llce_rx_startup(struct mbox_chan *chan);
static int llce_tx_startup(struct mbox_chan *chan);
static void llce_rx_shutdown(struct mbox_chan *chan);
static void llce_tx_shutdown(struct mbox_chan *chan);
static int llce_hif_startup(struct mbox_chan *chan);
static int llce_logger_startup(struct mbox_chan *chan);
static void llce_logger_shutdown(struct mbox_chan *chan);
static int process_rx_cmd(struct mbox_chan *chan, struct llce_rx_msg *msg);
static int process_logger_cmd(struct mbox_chan *chan, struct llce_rx_msg *msg);
static int submit_hif_cmd(struct llce_mb *mb, struct llce_can_command *cmd);

/**
 * Platform configuration can be skipped for cases when the HIF is completely
 * managed by another software
 */
static bool config_platform = true;
module_param(config_platform, bool, 0660);

static const char * const llce_modules[] = {
	LLCE_MODULE_ENTRY(LLCE_TX),
	LLCE_MODULE_ENTRY(LLCE_RX),
	LLCE_MODULE_ENTRY(LLCE_DTE),
	LLCE_MODULE_ENTRY(LLCE_FRPE),
	LLCE_MODULE_ENTRY(LLCE_CAN2CAN_TX),
	LLCE_MODULE_ENTRY(LLCE_CAN2CAN_RX),
	LLCE_MODULE_ENTRY(LLCE_CAN2CAN_FRPE),
	LLCE_MODULE_ENTRY(LLCE_AF_ETH_TX),
	LLCE_MODULE_ENTRY(LLCE_AF_ETH_RX),
	LLCE_MODULE_ENTRY(LLCE_AF_ETH_FRPE),
	LLCE_MODULE_ENTRY(LLCE_AF_HSE_TX),
	LLCE_MODULE_ENTRY(LLCE_AF_HSE_RX),
	LLCE_MODULE_ENTRY(LLCE_AF_HSE_FRPE),
	LLCE_MODULE_ENTRY(LLCE_AF_TX),
	LLCE_MODULE_ENTRY(LLCE_AF_RX),
	LLCE_MODULE_ENTRY(LLCE_AF_FRPE),
};

static const struct llce_mb_desc mb_map[] = {
	[S32G_LLCE_HIF_CONF_MB] = {
		.name = "HIF Config",
		.nchan = 2,
		.startup = llce_hif_startup,
	},
	[S32G_LLCE_CAN_CONF_MB] = {
		.name = "CAN Config",
		.nchan = 16,
	},
	[S32G_LLCE_CAN_RX_MB] = {
		.name = "CAN RX",
		.nchan = 16,
		.startup = llce_rx_startup,
		.shutdown = llce_rx_shutdown,
	},
	[S32G_LLCE_CAN_TX_MB] = {
		.name = "CAN TX",
		.nchan = 16,
		.startup = llce_tx_startup,
		.shutdown = llce_tx_shutdown,
	},
	[S32G_LLCE_CAN_LOGGER_MB] = {
		.name = "CAN Logger",
		.nchan = 16,
		.startup = llce_logger_startup,
		.shutdown = llce_logger_shutdown,
	},
	[S32G_LLCE_CAN_LOGGER_CONFIG_MB] = {
		.name = "CAN Logger Config",
		.nchan = 16,
	},
	[S32G_LLCE_CAN_CORE_CONFIG_MB] = {
		.name = "CAN Core Config",
		.nchan = 1,
	},
};

static inline void __iomem *llce_get_lin_shmem_addr(void __iomem *shmem)
{
	return shmem + LIN_MEM_OFFSET;
}

static inline void __iomem *llce_get_rx_tx_stats_addr(void __iomem *shmem)
{
	return shmem + CAN_RX_TX_STATS_OFFSET;
}

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

static const char *get_module_name(enum llce_can_module module)
{
	u32 index = module - LLCE_TX;

	if (index >= ARRAY_SIZE(llce_modules))
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

static const char *get_channel_type_name(unsigned int type)
{
	if (type >= ARRAY_SIZE(mb_map))
		return "Unknown channel type";

	return mb_map[type].name;
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

static void __iomem *get_host_rxin(struct llce_mb *mb)
{
	if (mb->hif_id == LLCE_CAN_HIF0)
		return get_rxin_by_index(mb, 16);

	return get_rxin_by_index(mb, 18);
}

static void __iomem *get_host_notif(struct llce_mb *mb)
{
	if (mb->hif_id == LLCE_CAN_HIF0)
		return get_rxin_by_index(mb, 0);

	return get_rxin_by_index(mb, 8);
}

static void __iomem *get_logger_in(struct llce_mb *mb)
{
	return get_rxout_by_index(mb, LLCE_CAN_LOGGER_IN_FIFO);
}

static void __iomem *get_logger_out(struct llce_mb *mb)
{
	return get_rxout_by_index(mb, LLCE_CAN_LOGGER_OUT_FIFO);
}

static void __iomem *get_host_txack(struct llce_mb *mb)
{
	if (mb->hif_id == LLCE_CAN_HIF0)
		return get_txack_by_index(mb, 17);

	return get_txack_by_index(mb, 18);
}

static unsigned int get_ctrl_fifo(struct llce_mb *mb, unsigned int ctrl_id)
{
	return mb->chans_params[ctrl_id].fifo;
}

static void __iomem *get_txack_fifo(struct mbox_chan *chan,
				    unsigned int *txack_id)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	unsigned int fifo_id;

	if (priv->type == S32G_LLCE_HIF_CONF_MB)
		return get_host_txack(mb);

	fifo_id = get_ctrl_fifo(mb, priv->index);

	if (txack_id)
		*txack_id = fifo_id;

	return get_txack_by_index(mb, fifo_id);
}

static void __iomem *get_rxout_fifo(struct mbox_chan *chan,
				    unsigned int *rxout_id)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	unsigned int fifo_id = get_ctrl_fifo(mb, priv->index);

	if (rxout_id)
		*rxout_id = fifo_id;

	return get_rxout_by_index(mb, fifo_id);
}

static void __iomem *get_logger_out_fifo(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	return get_rxout_by_index(mb, LLCE_CAN_LOGGER_OUT_FIFO);
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

static bool is_can_core_config_chan(unsigned int chan_type)
{
	return chan_type == S32G_LLCE_CAN_CORE_CONFIG_MB;
}

static bool is_config_chan(unsigned int chan_type)
{
	return chan_type == S32G_LLCE_CAN_CONF_MB ||
		chan_type == S32G_LLCE_HIF_CONF_MB ||
		is_can_core_config_chan(chan_type);
}

static bool is_rx_chan(unsigned int chan_type)
{
	return chan_type == S32G_LLCE_CAN_RX_MB;
}

static bool is_tx_chan(unsigned int chan_type)
{
	return chan_type == S32G_LLCE_CAN_TX_MB;
}

static bool is_logger_chan(unsigned int chan_type)
{
	return chan_type == S32G_LLCE_CAN_LOGGER_MB;
}

static bool is_logger_config_chan(unsigned int chan_type)
{
	return chan_type == S32G_LLCE_CAN_LOGGER_CONFIG_MB;
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
	if (is_config_chan(type) || is_logger_config_chan(type)) {
		chan->txdone_method = TXDONE_BY_POLL;
		priv->state = LLCE_REGISTERED_CHAN;
	} else {
		if (is_rx_chan(type) || is_logger_chan(type))
			chan->txdone_method = TXDONE_BY_ACK;
		else
			chan->txdone_method = TXDONE_BY_IRQ;

		priv->state = LLCE_UNREGISTERED_CHAN;
	}

	priv->data = NULL;
	spin_lock_init(&priv->lock);

	return 0;
}

static void deinit_chan_priv(struct mbox_chan *chan)
{
	kfree(chan->con_priv);
	chan->con_priv = NULL;
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
	chan = &mbox->chans[off];
	ret = init_chan_priv(chan, mb, type, index);
	if (ret)
		return ERR_PTR(ret);

	return chan;
}

static int execute_config_cmd(struct mbox_chan *chan,
			      struct llce_can_command *cmd,
			      u8 hw_ctrl)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	unsigned int idx = priv->index;
	struct llce_can_command __iomem *sh_cmd;
	void __iomem *txack, *push0;
	unsigned long flags;
	int ret = 0;

	if (hw_ctrl != LLCE_HW_CTRL_FROM_CHAN_IDX)
		idx = hw_ctrl;

	txack = get_host_txack(mb);

	sh_cmd = &mb->can_sh_mem->can_cmd[mb->hif_id];
	push0 = LLCE_FIFO_PUSH0(txack);

	spin_lock_irqsave(&mb->txack_lock, flags);

	if (!is_tx_fifo_empty(txack)) {
		ret = -EBUSY;
		goto release_lock;
	}

	cmd->return_value = LLCE_FW_NOTRUN;

	priv->last_msg = cmd;
	memcpy_toio(sh_cmd, cmd, sizeof(*cmd));

	/* Trigger an interrupt to the LLCE */
	writel(idx, push0);

release_lock:
	spin_unlock_irqrestore(&mb->txack_lock, flags);
	return ret;
}

static int process_get_fifo_index(struct mbox_chan *chan,
				  struct llce_config_msg *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	struct device *dev = mb->controller.dev;
	unsigned int fifo_id;

	fifo_id = get_ctrl_fifo(mb, msg->sw_cmd.fifo_cmd.hw_ctrl);
	if (fifo_id >= LLCE_NFIFO_WITH_IRQ) {
		dev_err(dev, "Invalid FIFO id: %u\n", fifo_id);
		return -EINVAL;
	}

	msg->sw_cmd.fifo_cmd.fifo = fifo_id;

	/* No actual FW command executed */
	priv->last_msg = NULL;

	return 0;
}

static int process_get_can_stats(struct mbox_chan *chan,
				 struct llce_config_msg *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	unsigned int hw_ctrl = priv->index;

	memcpy_fromio(&msg->sw_cmd.stats_cmd.stats,
		      &mb->can_stats[hw_ctrl],
		      sizeof(msg->sw_cmd.stats_cmd.stats));

	/* No actual FW command executed */
	priv->last_msg = NULL;

	return 0;
}

static int execute_sw_cmd(struct mbox_chan *chan, struct llce_config_msg *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	struct device *dev = mb->controller.dev;

	switch (msg->sw_cmd.cmd) {
	case LLCE_GET_FIFO_INDEX:
		return process_get_fifo_index(chan, msg);
	case LLCE_GET_CAN_STATS:
		return process_get_can_stats(chan, msg);
	default:
		dev_err(dev, "Failed to interpret sw cmd=%u\n",
			msg->sw_cmd.cmd);
		return -EINVAL;
	}
}

static int process_config_msg(struct mbox_chan *chan,
			      struct llce_config_msg *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	struct device *dev = mb->controller.dev;

	switch (msg->cmd) {
	case LLCE_EXECUTE_FW_HIF_CMD:
		if (!is_can_core_config_chan(priv->type))
			return -EPERM;

		priv->last_msg = &msg->fw_cmd.cmd;
		return submit_hif_cmd(mb, &msg->fw_cmd.cmd);
	case LLCE_EXECUTE_FW_CMD:
		return execute_config_cmd(chan, &msg->fw_cmd.cmd,
					  msg->fw_cmd.hw_ctrl);
	case LLCE_EXECUTE_SW_CMD:
		return execute_sw_cmd(chan, msg);
	default:
		dev_err(dev, "Failed to interpret config cmd=%u\n",
			msg->cmd);
		return -EINVAL;
	}
}

static bool is_blrin_full(struct mbox_chan *chan)
{
	void __iomem *blrin = get_blrin_fifo(chan);
	void __iomem *status1 = LLCE_FIFO_STATUS1(blrin);

	return !!(readl(status1) & LLCE_FIFO_FFULLD);
}

static u32 build_word0(bool rtr, bool ide, u32 std_id,
		       u32 ext_id)
{
	u32 word0;

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
	struct llce_can_shared_memory __iomem *sh_cmd = mb->can_sh_mem;
	unsigned int ack_interface, chan_idx = priv->index;
	struct canfd_frame *cf = msg->cf;
	u32 mb_index;
	u16 frame_index;
	u32 word0, std_id, ext_id;
	u8 dlc, add_mac = 0u;
	u8 __iomem *payload;
	u32 mb_config;

	/* Get a free message buffer from BLROUT queue */
	mb_index = readl(pop0);

	if (mb_index == LLCE_FIFO_NULL_VALUE)
		return -EAGAIN;

	if (chan_idx >= LLCE_CAN_CONFIG_MAXCTRL_COUNT) {
		dev_err(mb->dev, "%u channel cannot be set as tag filter\n",
			chan_idx);
		return -EINVAL;
	}

	ack_interface = get_ctrl_fifo(mb, chan_idx);
	if (ack_interface >= LLCE_NFIFO_WITH_IRQ) {
		dev_err(mb->dev, "ACK interface (%u) exceeds the allowed limit\n",
			ack_interface);
		return -EINVAL;
	}

	mb_index &= LLCE_CAN_CONFIG_FIFO_FIXED_MASK;

	std_id = cf->can_id & CAN_SFF_MASK;
	ext_id = (cf->can_id & CAN_EFF_MASK) >> CAN_SFF_ID_BITS;

	word0 = build_word0(!!(cf->can_id & CAN_RTR_FLAG),
			    !!(cf->can_id & CAN_EFF_FLAG),
			    std_id, ext_id);

	/* Get the index of the frame reserved by the firmware */
	memcpy_fromio(&frame_index,
		      &sh_cmd->can_tx_mb_desc[mb_index].mb_frame_idx,
		      sizeof(frame_index));
	/* Set CAN ID */
	memcpy_toio(&sh_cmd->can_mb[frame_index].word0,
		    &word0,
		    sizeof(word0));
	/* Attach a token (channel ID) to be used in ACK handler */
	memcpy_toio(&sh_cmd->can_tx_mb_desc[mb_index].frame_tag1,
		    &chan_idx,
		    sizeof(chan_idx));
	memcpy_toio(&sh_cmd->can_tx_mb_desc[mb_index].enable_tx_frame_mac,
		    &add_mac,
		    sizeof(add_mac));
	/* Set the notification interface */
	memcpy_toio(&sh_cmd->can_tx_mb_desc[mb_index].ack_interface,
		    &ack_interface,
		    sizeof(ack_interface));
	payload = &sh_cmd->can_mb[frame_index].payload[0];

	memcpy_toio(payload, cf->data, cf->len);
	dlc = can_fd_len2dlc(cf->len);

	mb_config = dlc;
	if (msg->fd_msg) {
		/* Configure the tx mb as a CAN FD frame. */
		mb_config |= LLCE_CAN_MB_FDF;

		/* Enable BRS feature to allow receiveing of CAN FD frames */
		if (cf->flags & CANFD_BRS)
			mb_config |= LLCE_CAN_MB_BRS;

		if (cf->flags & CANFD_ESI)
			mb_config |= LLCE_CAN_MB_ESI;
	}

	memcpy_toio(&sh_cmd->can_mb[frame_index].word1,
		    &mb_config,
		    sizeof(mb_config));

	spin_until_cond(!is_blrin_full(chan));

	/* Submit the buffer in BLRIN queue */
	writel(mb_index, push0);

	return 0;
}

static int process_logger_config_cmd(struct mbox_chan *chan, void *data)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct logger_config_msg *msg = data;
	struct llce_mb *mb = priv->mb;

	switch (msg->cmd) {
	case LOGGER_CMD_FW_SUPPORT:
		msg->fw_logger_support = mb->fw_logger_support;
		return 0;
	default:
		return -EINVAL;
	}
}

static int llce_mb_send_data(struct mbox_chan *chan, void *data)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	/* Client should not flush new tasks if suspended. */
	WARN_ON(mb->suspended);

	if (is_config_chan(priv->type))
		return process_config_msg(chan, data);

	if (is_tx_chan(priv->type))
		return send_can_msg(chan, data);

	if (is_rx_chan(priv->type))
		return process_rx_cmd(chan, data);

	if (is_logger_chan(priv->type))
		return process_logger_cmd(chan, data);

	if (is_logger_config_chan(priv->type))
		return process_logger_config_cmd(chan, data);

	return -EINVAL;
}

static void enable_fifo_irq(void __iomem *fifo)
{
	void __iomem *status1 = LLCE_FIFO_STATUS1(fifo);
	void __iomem *ier = LLCE_FIFO_IER(fifo);
	u32 ier_val;

	/* Clear interrupt status flags. */
	writel(readl(status1), status1);
	/* Enable interrupt */
	ier_val = readl(ier) | LLCE_FIFO_FNEMTY;
	writel(ier_val, ier);
}

static void disable_fifo_irq(void __iomem *fifo)
{
	void __iomem *ier = LLCE_FIFO_IER(fifo);
	u32 ier_val;

	ier_val = readl(ier) & ~LLCE_FIFO_FNEMTY;
	writel(ier_val, ier);
}

static enum llce_sema42_gate get_sema42_gate(u8 fifo, u32 host)
{
	/**
	 * Semaphore used to protect acces to TXACK and RXOUT between LLCE and
	 * host on interrupt enable/disable.
	 */
	static const enum llce_sema42_gate
	sema4_ier[LLCE_CAN_CONFIG_IER_SEMA4_COUNT][LLCE_CAN_CONFIG_HIF_COUNT] = {
		{LLCE_SEMA42_GATE20, LLCE_SEMA42_GATE21},
		{LLCE_SEMA42_GATE22, LLCE_SEMA42_GATE23}
	};

	return sema4_ier[fifo][host];
}

static void ctrl_fifo_irq_with_lock(struct llce_mb *mb, void __iomem *fifo,
				    enum llce_sema42_gate gate, bool enable)
{
	llce_sema42_lock(mb->sema42, gate, LLCE_HOST_CORE_SEMA42_DOMAIN);

	if (enable)
		enable_fifo_irq(fifo);
	else
		disable_fifo_irq(fifo);

	llce_sema42_unlock(mb->sema42, gate);
}

static void ctrl_txack_irq_with_lock(struct llce_mb *mb, void __iomem *txack,
				     bool enable)
{
	enum llce_sema42_gate gate = get_sema42_gate(LLCE_FIFO_TXACK_INDEX,
						     mb->hif_id);

	ctrl_fifo_irq_with_lock(mb, txack, gate, enable);
}

static void ctrl_rxout_irq_with_lock(struct llce_mb *mb, void __iomem *rxout,
				     bool enable)
{
	enum llce_sema42_gate gate = get_sema42_gate(LLCE_FIFO_RXOUT_INDEX,
						     mb->hif_id);

	ctrl_fifo_irq_with_lock(mb, rxout, gate, enable);
}

static void disable_rxout_irq(struct llce_mb *mb, void __iomem *rxout)
{
	ctrl_rxout_irq_with_lock(mb, rxout, false);
}

static void disable_rxout_irq_by_index(struct llce_mb *mb, u8 index)
{
	void __iomem *rxout = get_rxout_by_index(mb, index);

	disable_rxout_irq(mb, rxout);
}

static void enable_rxout_irq(struct llce_mb *mb, void __iomem *rxout)
{
	ctrl_rxout_irq_with_lock(mb, rxout, true);
}

static void enable_logger_irq(struct llce_mb *mb)
{
	void __iomem *rxout = get_logger_out(mb);

	ctrl_rxout_irq_with_lock(mb, rxout, true);
}

static void disable_logger_irq(struct llce_mb *mb)
{
	void __iomem *rxout = get_logger_out(mb);

	ctrl_rxout_irq_with_lock(mb, rxout, false);
}

static int request_llce_irq(struct llce_mb *mb, struct llce_fifoirq *fifo_irq)
{
	int ret;

	if (fifo_irq->registered)
		return 0;

	ret = devm_request_irq(mb->dev, fifo_irq->num, fifo_irq->handler,
			       IRQF_SHARED, fifo_irq->name, mb);
	if (ret < 0)
		dev_err(mb->dev, "Failed to register '%s' IRQ\n",
			fifo_irq->name);

	fifo_irq->registered = true;
	return ret;
}

static int request_llce_pair_irq(struct llce_mb *mb, struct llce_pair_irq *pair)
{
	int ret;

	if (!mb->multihif || (mb->multihif && mb->hif_id == LLCE_CAN_HIF0)) {
		ret = request_llce_irq(mb, &pair->irq0);
		if (ret)
			return ret;
	}

	if (!mb->multihif || (mb->multihif && mb->hif_id == LLCE_CAN_HIF1)) {
		ret = request_llce_irq(mb, &pair->irq8);
		if (ret)
			return ret;
	}

	return 0;
}

static int llce_hif_startup(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	request_llce_pair_irq(mb, &mb->rxin_irqs);

	return 0;
}

static struct llce_rx_data *alloc_pop_data(void)
{
	struct llce_rx_data *ret;

	ret = kmalloc(sizeof(*ret), GFP_KERNEL);

	return ret;
}

static void init_pop_data(struct llce_chan_priv *priv, struct llce_rx_data *data)
{
	data->has_leftover = false;
	priv->data = data;
}

static void release_pop_data(struct llce_chan_priv *priv)
{
	kfree(priv->data);
	priv->data = NULL;
}

static void control_shared_rxout_cb(void *data)
{
	struct shared_rxout_data *rx = (struct shared_rxout_data *)data;

	if (rx->enable)
		enable_rxout_irq(rx->mb, rx->rxout);
	else
		disable_rxout_irq(rx->mb, rx->rxout);
}

static void control_shared_txack_cb(void *data)
{
	struct shared_txack_data *ack = (struct shared_txack_data *)data;

	ctrl_txack_irq_with_lock(ack->mb, ack->txack, ack->enable);
}

static u8 *get_shared_irq_counter(struct fifos_ref_cnt *ref_cnt, bool rxout,
				  unsigned int index)
{
	if (rxout)
		return &ref_cnt->rxout[index];

	return &ref_cnt->txack[index];
}

static void enable_shared_irq(struct fifos_ref_cnt *ref_cnt, bool rxout,
			      unsigned int index, enable_irq_cb_t cb,
			      void *data)
{
	unsigned long flags;
	u8 *counter = get_shared_irq_counter(ref_cnt, rxout, index);

	spin_lock_irqsave(&ref_cnt->lock, flags);

	if (!*counter)
		cb(data);
	(*counter)++;

	spin_unlock_irqrestore(&ref_cnt->lock, flags);
}

static void disable_shared_irq(struct device *dev,
			       struct fifos_ref_cnt *ref_cnt, bool rxout,
			       unsigned int index, enable_irq_cb_t cb,
			       void *data)
{
	unsigned long flags;
	const char *queue_name;
	u8 *counter = get_shared_irq_counter(ref_cnt, rxout, index);

	if (rxout)
		queue_name = "rxout";
	else
		queue_name = "txack";

	spin_lock_irqsave(&ref_cnt->lock, flags);

	if (!*counter) {
		dev_err(dev, "Trying to release an already released %s%d irq\n",
			queue_name, index);
	} else {
		(*counter)--;

		/* If no one else is using it */
		if (!*counter)
			cb(data);
	}

	spin_unlock_irqrestore(&ref_cnt->lock, flags);
}

static int llce_rx_startup(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_rx_data *data;
	struct llce_mb *mb = priv->mb;
	unsigned int rxout_id;
	void __iomem *rxout = get_rxout_fifo(chan, &rxout_id);
	struct shared_rxout_data en_irq_data = {
		.mb = mb,
		.rxout = rxout,
		.enable = true,
	};
	unsigned long flags;

	request_llce_pair_irq(mb, &mb->rxout_irqs);

	data = alloc_pop_data();
	if (!data)
		return -ENOMEM;

	/* State change must go under the lock protection */
	spin_lock_irqsave(&priv->lock, flags);

	init_pop_data(priv, data);
	priv->state = LLCE_REGISTERED_CHAN;

	enable_shared_irq(&mb->fifos_irq_ref_cnt, true,
			  rxout_id, control_shared_rxout_cb,
			  &en_irq_data);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void llce_rx_shutdown(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	struct device *dev = mb->controller.dev;
	unsigned int rxout_id;
	void __iomem *rxout = get_rxout_fifo(chan, &rxout_id);
	struct shared_rxout_data dis_irq_data = {
		.mb = mb,
		.rxout = rxout,
		.enable = false,
	};
	unsigned long flags;

	disable_shared_irq(dev, &mb->fifos_irq_ref_cnt, true, rxout_id,
			   control_shared_rxout_cb, &dis_irq_data);

	spin_lock_irqsave(&priv->lock, flags);

	priv->state = LLCE_UNREGISTERED_CHAN;
	release_pop_data(priv);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static void enable_bus_off_irq(struct llce_mb *mb)
{
	void __iomem *rxin = get_host_notif(mb);
	void __iomem *ier = LLCE_FIFO_IER(rxin);
	u32 ier_val;

	/* Enable BusOff for host 0 only */
	ier_val = readl(ier) | LLCE_FIFO_FNEMTY;
	writel(ier_val, ier);
}

static int llce_tx_startup(struct mbox_chan *chan)
{
	unsigned int txack_id;
	void __iomem *txack = get_txack_fifo(chan, &txack_id);
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	struct shared_txack_data en_irq_data = {
		.mb = mb,
		.txack = txack,
		.enable = true,
	};
	unsigned long flags;

	request_llce_pair_irq(mb, &mb->txack_irqs);

	spin_lock_irqsave(&priv->lock, flags);
	priv->state = LLCE_REGISTERED_CHAN;

	enable_bus_off_irq(mb);

	spin_unlock_irqrestore(&priv->lock, flags);

	enable_shared_irq(&mb->fifos_irq_ref_cnt, false,
			  txack_id, control_shared_txack_cb,
			  &en_irq_data);

	return 0;
}

static void llce_tx_shutdown(struct mbox_chan *chan)
{
	unsigned int txack_id;
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	struct device *dev = mb->controller.dev;
	void __iomem *txack = get_txack_fifo(chan, &txack_id);
	struct shared_txack_data dis_irq_data = {
		.mb = mb,
		.txack = txack,
		.enable = false,
	};
	unsigned long flags;

	disable_shared_irq(dev, &mb->fifos_irq_ref_cnt, false,
			   txack_id, control_shared_txack_cb,
			   &dis_irq_data);

	spin_lock_irqsave(&priv->lock, flags);
	priv->state = LLCE_UNREGISTERED_CHAN;

	spin_unlock_irqrestore(&priv->lock, flags);
}

static int llce_logger_startup(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_rx_data *data;
	struct llce_mb *mb = priv->mb;
	unsigned long flags;
	int ret = 0;

	ret = request_llce_irq(mb, &mb->logger_irq);
	if (ret)
		return ret;

	data = alloc_pop_data();
	if (!data)
		return -ENOMEM;

	spin_lock_irqsave(&priv->lock, flags);
	priv->state = LLCE_REGISTERED_CHAN;

	init_pop_data(priv, data);

	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static void llce_logger_shutdown(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	priv->state = LLCE_UNREGISTERED_CHAN;

	release_pop_data(priv);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static int llce_mb_startup(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;

	if (mb_map[priv->type].startup)
		return mb_map[priv->type].startup(chan);

	return 0;
}

static bool is_chan_registered(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	unsigned long flags;
	bool ret;

	if (!priv)
		return false;

	spin_lock_irqsave(&priv->lock, flags);
	ret = (priv->state == LLCE_REGISTERED_CHAN);
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static void llce_mbox_chan_received_data(struct mbox_chan *chan, void *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;

	if (!is_chan_registered(chan)) {
		if (!priv)
			dev_err(chan->mbox->dev,
				"Received a message on an unregistered channel\n");
		else
			dev_err(chan->mbox->dev,
				"Received a message on an unregistered channel (type: %s, index: %u)\n",
				get_channel_type_name(priv->type), priv->index);
		return;
	}

	mbox_chan_received_data(chan, msg);
}

static bool llce_mb_last_tx_done(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	void __iomem *txack;
	struct llce_mb *mb = priv->mb;
	struct llce_can_command *cmd;
	struct llce_can_command __iomem *sh_cmd;
	unsigned long flags;

	if (is_logger_config_chan(priv->type)) {
		llce_mbox_chan_received_data(chan, NULL);
		return true;
	}

	if (!is_config_chan(priv->type))
		return false;

	/* Non-firmware commands */
	if (!priv->last_msg) {
		/* Unblock the caller */
		llce_mbox_chan_received_data(chan, NULL);
		return true;
	}

	txack = get_host_txack(mb);
	sh_cmd = &mb->can_sh_mem->can_cmd[mb->hif_id];

	spin_lock_irqsave(&mb->txack_lock, flags);

	if (!is_tx_fifo_empty(txack))
		goto out_busy;

	cmd = priv->last_msg;
	memcpy_fromio(cmd, sh_cmd, sizeof(*cmd));

	spin_unlock_irqrestore(&mb->txack_lock, flags);

	if (priv->type != S32G_LLCE_HIF_CONF_MB)
		llce_mbox_chan_received_data(chan, cmd);

	return true;
out_busy:
	spin_unlock_irqrestore(&mb->txack_lock, flags);
	return false;
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

static struct device_node *get_sram_node(struct device *dev, const char *name)
{
	struct device_node *node, *dev_node;
	int idx;

	dev_node = dev->of_node;
	idx = of_property_match_string(dev_node, "memory-region-names", name);
	node = of_parse_phandle(dev_node, "memory-region", idx);
	if (!node) {
		dev_err(dev, "Failed to get '%s' memory region\n", name);
		return ERR_PTR(-EIO);
	}

	return node;
}

static int map_sram_node(struct device *dev, const char *name,
			 void __iomem **addr)
{
	struct device_node *sram_node;
	struct resource r;
	resource_size_t size;
	int ret;

	sram_node = get_sram_node(dev, name);
	if (IS_ERR(sram_node))
		return PTR_ERR(sram_node);

	ret = of_address_to_resource(sram_node, 0, &r);
	of_node_put(sram_node);
	if (ret)
		return ret;

	size = resource_size(&r);

	*addr = devm_ioremap_wc(dev, r.start, size);
	if (!*addr) {
		dev_err(dev, "Failed to map '%s' memory region\n", name);
		return -ENOMEM;
	}

	return 0;
}

static int map_llce_shmem(struct llce_mb *mb)
{
	void __iomem *shmem;
	int ret;

	ret = map_sram_node(mb->dev, LLCE_SHMEM_REG_NAME, &shmem);
	if (ret < 0)
		return ret;

	mb->can_sh_mem = shmem;
	mb->lin_sh_mem = llce_get_lin_shmem_addr(shmem);
	mb->status = llce_get_status_regs_addr(shmem);
	mb->can_stats = llce_get_rx_tx_stats_addr(shmem);

	return 0;
}

static void __iomem *get_icsr_addr(struct llce_mb *mb, u32 icsr_id)
{
	return mb->icsr + icsr_id * sizeof(u32);
}

static void __iomem *get_icsr(struct llce_mb *mb, struct llce_pair_irq *irqs,
			      u32 icsr_index, int irq,
			      u8 *base_id)
{
	u32 icsr_id;
	const struct llce_icsr *icsrs_conf = &icsrs[icsr_index];

	if (irq == irqs->irq0.num) {
		icsr_id = icsrs_conf->icsr0_num;
		*base_id = 0;
	} else {
		icsr_id = icsrs_conf->icsr8_num;
		*base_id = 8;
	}

	return get_icsr_addr(mb, icsr_id);
}

static void __iomem *get_txack_icsr(struct llce_mb *mb, int irq,
				    u8 *base_id)
{
	return get_icsr(mb, &mb->txack_irqs, LLCE_CAN_ICSR_TXACK_INDEX,
			irq, base_id);
}

static void __iomem *get_rxin_icsr(struct llce_mb *mb, int irq,
				   u8 *base_id)
{
	return get_icsr(mb, &mb->rxin_irqs, LLCE_CAN_ICSR_RXIN_INDEX,
			irq, base_id);
}

static void __iomem *get_rxout_icsr(struct llce_mb *mb, int irq,
				    u8 *base_id)
{
	return get_icsr(mb, &mb->rxout_irqs, LLCE_CAN_ICSR_RXOUT_INDEX,
			irq, base_id);
}

static void __iomem *get_logger_icsr(struct llce_mb *mb)
{
	return get_icsr_addr(mb, LLCE_CAN_LOGGER_ICSR);
}

static void llce_process_tx_ack(struct llce_mb *mb, u8 index)
{
	void __iomem *tx_ack = get_txack_by_index(mb, index);
	void __iomem *status1 = LLCE_FIFO_STATUS1(tx_ack);
	void __iomem *pop0 = LLCE_FIFO_POP0(tx_ack);
	struct mbox_controller *ctrl = &mb->controller;
	struct llce_can_shared_memory __iomem *can_sh_mem = mb->can_sh_mem;
	struct llce_can_tx2host_ack_info __iomem *info;
	struct llce_tx_notif notif;
	struct device *dev = mb->controller.dev;
	u32 ack_id;
	u16 frame_tag1;
	unsigned int chan_index;

	while (!(readl(status1) & LLCE_FIFO_FEMTYD)) {
		/* Get ACK mailbox */
		ack_id = readl(pop0) & LLCE_CAN_CONFIG_FIFO_FIXED_MASK;

		info = &can_sh_mem->can_tx_ack_info[ack_id];
		memcpy_fromio(&frame_tag1, &info->frame_tag1,
			      sizeof(frame_tag1));

		if (frame_tag1 >= get_channels_for_type(S32G_LLCE_CAN_TX_MB)) {
			dev_err(dev,
				"%u exceeds the number of allocated channels for type : %d\n",
				frame_tag1, S32G_LLCE_CAN_TX_MB);
			continue;
		}

		chan_index = get_channel_offset(S32G_LLCE_CAN_TX_MB,
						frame_tag1);
		if (!is_chan_registered(&ctrl->chans[chan_index])) {
			dev_err(dev,
				"Received a TX ACK message on an unregistered channel (type: %s, index: %u)\n",
				get_channel_type_name(S32G_LLCE_CAN_TX_MB),
				frame_tag1);
			continue;
		}
		notif.error = 0;
		memcpy_fromio(&notif.tx_timestamp,
			      &info->tx_timestamp,
			      sizeof(notif.tx_timestamp));

		/* Notify the client and send the timestamp */
		mbox_chan_received_data(&ctrl->chans[chan_index], &notif);
		mbox_chan_txdone(&ctrl->chans[chan_index], 0);
	}

	/* Clear the interrupt status flag. */
	writel(LLCE_FIFO_FNEMTY, status1);
}

static void process_chan_err(struct llce_mb *mb, u32 chan_type,
			     struct llce_can_channel_error_notif __iomem *error)
{
	unsigned int chan_index;
	enum llce_can_module module_id;
	enum llce_fw_return fw_error;
	struct mbox_controller *ctrl = &mb->controller;
	struct device *dev = ctrl->dev;
	struct llce_rx_msg rx_notif;
	struct llce_tx_notif tx_notif;
	struct mbox_chan *chan;
	void *notif;
	u8 hw_ctrl;

	memcpy_fromio(&fw_error, &error->error_info.error_code,
		      sizeof(fw_error));
	memcpy_fromio(&module_id, &error->error_info.module_id,
		      sizeof(module_id));
	memcpy_fromio(&hw_ctrl, &error->hw_ctrl, sizeof(hw_ctrl));

	chan_index = get_channel_offset(chan_type, hw_ctrl);
	chan = &ctrl->chans[chan_index];

	if (!is_chan_registered(chan)) {
		net_err_ratelimited("%s: Received error %u on '%s' %u channel (module %s)\n",
				    dev_name(dev),
				    fw_error, get_channel_type_name(chan_type),
				    hw_ctrl,
				    get_module_name(module_id));
		return;
	}

	if (is_tx_chan(chan_type)) {
		tx_notif.error = fw_error;

		/* Release the channel if an error occurred */
		mbox_chan_txdone(chan, 0);
		notif = &tx_notif;
	} else {
		rx_notif.cmd = LLCE_ERROR;
		rx_notif.error = fw_error;

		notif = &rx_notif;
	}

	mbox_chan_received_data(chan, notif);
}

static void process_channel_err(struct llce_mb *mb,
				struct llce_can_channel_error_notif __iomem *error)
{
	enum llce_can_module module_id;
	enum llce_fw_return err;
	u8 hw_ctrl;

	memcpy_fromio(&module_id, &error->error_info.module_id, sizeof(module_id));
	memcpy_fromio(&err, &error->error_info.error_code, sizeof(err));

	switch (module_id) {
	case LLCE_TX:
	case LLCE_CAN2CAN_TX:
	case LLCE_AF_ETH_TX:
	case LLCE_AF_HSE_TX:
	case LLCE_AF_TX:
		return process_chan_err(mb, S32G_LLCE_CAN_TX_MB, error);
	case LLCE_RX:
	case LLCE_CAN2CAN_RX:
	case LLCE_AF_ETH_RX:
	case LLCE_AF_HSE_RX:
	case LLCE_AF_RX:
		return process_chan_err(mb, S32G_LLCE_CAN_RX_MB, error);
	default:
		memcpy_fromio(&hw_ctrl, &error->hw_ctrl, sizeof(hw_ctrl));
		net_warn_ratelimited("%s: Error module:%s Error:%d HW module:%u\n",
				     dev_name(mb->controller.dev),
				     get_module_name(module_id), err,
				     hw_ctrl);
		break;
	}
}

static void process_platform_err(struct llce_mb *mb,
				 struct llce_can_error_notif __iomem *error)
{
}

static void process_ctrl_err(struct llce_mb *mb,
			     struct llce_can_ctrl_mode_notif __iomem *error)
{
}

static void llce_process_rxin(struct llce_mb *mb, u8 index)
{
	void __iomem *rxin = get_rxin_by_index(mb, index);
	void __iomem *status1 = LLCE_FIFO_STATUS1(rxin);
	void __iomem *pop0 = LLCE_FIFO_POP0(rxin);
	struct llce_can_shared_memory __iomem *can_sh_mem = mb->can_sh_mem;
	struct llce_can_notification_table __iomem *table;
	struct llce_can_notification __iomem *notif;
	union llce_can_notification_list __iomem *list;
	enum llce_can_notification_id notif_id;

	u32 rxin_id;

	while (!(readl(status1) & LLCE_FIFO_FEMTYD)) {
		/* Get notification mailbox */
		rxin_id = readl(pop0) & LLCE_CAN_CONFIG_FIFO_FIXED_MASK;
		table = &can_sh_mem->can_notification_table;
		notif = &table->can_notif0_table[mb->hif_id][rxin_id];
		list = &notif->notif_list;

		memcpy_fromio(&notif_id, &notif->notif_id, sizeof(notif_id));

		switch (notif_id) {
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

static bool has_leftovers(struct mbox_chan *chan)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_rx_data *data = priv->data;
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&priv->lock, flags);
	ret = data->has_leftover;
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static void push_llce_rx_data(struct mbox_chan *chan, struct llce_rx_can_mb *mb,
			      u32 index)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_rx_data *data = priv->data;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if (data->has_leftover)
		dev_err(chan->cl->dev, "Overwriting logger's internal frame\n");

	data->has_leftover = true;
	data->index = index;
	data->mb = *mb;

	spin_unlock_irqrestore(&priv->lock, flags);
}

static bool pop_chan_rx_data(struct mbox_chan *chan, struct llce_rx_can_mb *mb,
			     u32 *index)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_rx_data *data = priv->data;
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&priv->lock, flags);

	if (!data->has_leftover) {
		ret = false;
	} else {
		ret = true;
		data->has_leftover = false;
		*mb = data->mb;
		*index = data->index;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static int process_is_rx_empty(struct mbox_chan *chan, struct llce_rx_msg *msg)
{
	void __iomem *rxout = get_rxout_fifo(chan, NULL);
	void __iomem *status1 = LLCE_FIFO_STATUS1(rxout);

	if (has_leftovers(chan)) {
		msg->is_rx_empty = false;
		return 0;
	}

	msg->is_rx_empty = !!(readl(status1) & LLCE_FIFO_FEMTYD);
	return 0;
}

static int process_is_logger_empty(struct mbox_chan *chan,
				   struct llce_rx_msg *msg)
{
	void __iomem *rxout = get_logger_out_fifo(chan);
	void __iomem *status1 = LLCE_FIFO_STATUS1(rxout);

	if (has_leftovers(chan)) {
		msg->is_rx_empty = false;
		return 0;
	}

	msg->is_rx_empty = !!(readl(status1) & LLCE_FIFO_FEMTYD);
	return 0;
}

static int pop_rxout_frame(struct llce_mb *mb, void __iomem *rxout,
			   struct llce_rx_msg *msg, u8 *hw_ctrl)
{
	void __iomem *pop0 = LLCE_FIFO_POP0(rxout);
	struct device *dev = mb->controller.dev;
	struct llce_can_shared_memory __iomem *can_sh_mem = mb->can_sh_mem;
	u32 rx_mb, rx_short_mb;
	u16 filter_id;
	u8 mb_type;

	/* Get RX mailbox */
	rx_mb = readl(pop0) & LLCE_CAN_CONFIG_FIFO_FIXED_MASK;

	memcpy_fromio(&filter_id,
		      &can_sh_mem->can_rx_mb_desc[rx_mb].filter_id,
		      sizeof(filter_id));

	mb_type = llce_filter_get_mb_type(filter_id);
	*hw_ctrl = llce_filter_get_hw_ctrl(filter_id);

	if (mb_type == USE_LONG_MB) {
		memcpy_fromio(&msg->rx_pop.mb.data.longm,
			      &can_sh_mem->can_mb[rx_mb],
			      sizeof(msg->rx_pop.mb.data.longm));
		msg->rx_pop.mb.is_long = true;
	} else {
		if (rx_mb < LLCE_CAN_CONFIG_MAXRXMB) {
			dev_err(dev, "Failed to identify the mailbox\n");
			return -EINVAL;
		}

		rx_short_mb = rx_mb - LLCE_CAN_CONFIG_MAXRXMB;
		memcpy_fromio(&msg->rx_pop.mb.data.shortm,
			      &can_sh_mem->can_short_mb[rx_short_mb],
			      sizeof(msg->rx_pop.mb.data.shortm));
		msg->rx_pop.mb.is_long = false;
	}

	msg->rx_pop.index = rx_mb;
	msg->rx_pop.skip = false;

	return 0;
}

static void release_rxout_index(struct llce_mb *mb, u32 index)
{
	void __iomem *host_rxin = get_host_rxin(mb);
	void __iomem *host_push0 = LLCE_FIFO_PUSH0(host_rxin);

	writel(index, host_push0);
}

static void push_rxout_and_notif(struct llce_mb *mb, u8 hw_ctrl,
				 struct llce_rx_msg *msg,
				 void __iomem *rxout)
{
	struct mbox_controller *ctrl = &mb->controller;
	struct mbox_chan *chan;
	unsigned int chan_index;
	struct llce_rx_msg notif_msg = {
		.error = LLCE_FW_SUCCESS,
		.cmd = LLCE_RX_NOTIF,
	};

	chan_index = get_channel_offset(S32G_LLCE_CAN_RX_MB, hw_ctrl);

	chan = &ctrl->chans[chan_index];

	if (chan->con_priv && is_chan_registered(chan)) {
		push_llce_rx_data(chan, &msg->rx_pop.mb,
				  msg->rx_pop.index);
		mbox_chan_received_data(chan, &notif_msg);
	} else {
		/* Release the index if there are no clients to process it */
		release_rxout_index(mb, msg->rx_pop.index);
		enable_rxout_irq(mb, rxout);
	}
}

static int process_pop_rxout(struct mbox_chan *chan, struct llce_rx_msg *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	void __iomem *rxout = get_rxout_fifo(chan, NULL);
	struct llce_mb *mb = priv->mb;
	struct llce_rx_can_mb can_mb;
	int ret;
	u32 mb_index;
	u8 hw_ctrl;
	bool pop;

	/* Use a stashed frame if available */
	pop = pop_chan_rx_data(chan, &can_mb, &mb_index);
	if (pop) {
		msg->rx_pop.skip = false;
		msg->rx_pop.mb = can_mb;
		msg->rx_pop.index = mb_index;
		return 0;
	}

	ret = pop_rxout_frame(mb, rxout, msg, &hw_ctrl);
	if (ret)
		return ret;

	/* Skip the frame as it's not for this channel */
	if (hw_ctrl != priv->index) {
		push_rxout_and_notif(mb, hw_ctrl, msg, rxout);
		/* Mark it as skipped for the current channel */
		msg->rx_pop.skip = true;
	}

	return ret;
}

static u32 __iomem *get_ctrl_extension(struct llce_mb *mb)
{
	return (void __iomem *)mb->can_sh_mem + LLCE_RXMBEXTENSION_OFFSET;
}

static u8 get_hwctrl(struct llce_mb *mb, u32 frame_id)
{
	u32 __iomem *ctrl_extensions = get_ctrl_extension(mb);

	return (readl(&ctrl_extensions[frame_id]) >> HWCTRL_MBEXTENSION_SHIFT) &
	    HWCTRL_MBEXTENSION_MASK;
}

static void pop_logger_frame(struct llce_mb *mb, struct llce_can_mb *frame,
			     u32 *index, u32 *hw_ctrl)
{
	struct llce_can_shared_memory __iomem *can_sh_mem = mb->can_sh_mem;
	void __iomem *out_fifo = get_logger_out(mb);
	void __iomem *pop0 = LLCE_FIFO_POP0(out_fifo);

	*index = readl(pop0) & LLCE_CAN_CONFIG_FIFO_FIXED_MASK;

	memcpy_fromio(frame, &can_sh_mem->can_mb[*index], sizeof(*frame));
	*hw_ctrl = get_hwctrl(mb, *index);
}

static void release_logger_index(struct llce_mb *mb, u32 index)
{
	void __iomem *in_fifo = get_logger_in(mb);
	void __iomem *push0 = LLCE_FIFO_PUSH0(in_fifo);

	writel(index, push0);
}

static void send_llce_logger_notif(struct llce_mb *mb, u32 hw_ctrl,
				   struct llce_can_mb *frame, u32 mb_index)
{
	struct mbox_controller *ctrl = &mb->controller;
	struct mbox_chan *chan;
	unsigned int chan_index;
	struct llce_rx_msg msg = {
		.error = LLCE_FW_SUCCESS,
		.cmd = LLCE_RX_NOTIF,
	};
	struct llce_rx_can_mb can_mb = {
		.data = {
			.longm = *frame,
		},
		.is_long = true,
	};

	chan_index = get_channel_offset(S32G_LLCE_CAN_LOGGER_MB, hw_ctrl);

	chan = &ctrl->chans[chan_index];
	if (chan->con_priv && is_chan_registered(chan)) {
		push_llce_rx_data(chan, &can_mb, mb_index);
		mbox_chan_received_data(chan, &msg);
	} else {
		/* Release the index if there are no clients to process it */
		release_logger_index(mb, mb_index);
		enable_logger_irq(mb);
	}
}

static int process_pop_logger(struct mbox_chan *chan, struct llce_rx_msg *msg)
{
	u32 hw_ctrl, mb_index;
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	struct llce_can_mb frame;
	struct llce_rx_can_mb can_mb;
	bool pop;

	/* Logger works with long MBs only */
	msg->rx_pop.mb.is_long = true;

	/* Use a stashed frame */
	pop = pop_chan_rx_data(chan, &can_mb, &mb_index);
	if (pop) {
		msg->rx_pop.skip = false;
		msg->rx_pop.mb = can_mb;
		msg->rx_pop.index = mb_index;

		return 0;
	}

	pop_logger_frame(mb, &frame, &mb_index, &hw_ctrl);

	/* Skip the frame as it's not for this channel */
	if (hw_ctrl != priv->index) {
		msg->rx_pop.skip = true;
		send_llce_logger_notif(mb, hw_ctrl, &frame, mb_index);
		return 0;
	}

	msg->rx_pop.skip = false;
	msg->rx_pop.mb.data.longm = frame;
	msg->rx_pop.index = mb_index;

	return 0;
}

static int process_release_rxout_index(struct mbox_chan *chan,
				       struct llce_rx_msg *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	release_rxout_index(mb, msg->rx_release.index);

	return 0;
}

static int process_release_logger_index(struct mbox_chan *chan,
					struct llce_rx_msg *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	release_logger_index(mb, msg->rx_release.index);
	return 0;
}

static int process_disable_rx_notif(struct mbox_chan *chan,
				    struct llce_rx_msg __always_unused *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	void __iomem *rxout = get_rxout_fifo(chan, NULL);

	/* Disable FIFO irq even if it's shared */
	disable_rxout_irq(mb, rxout);
	return 0;
}

static int process_disable_logger_notif(struct mbox_chan *chan,
					struct llce_rx_msg __always_unused *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	disable_logger_irq(mb);
	return 0;
}

static int process_enable_rx_notif(struct mbox_chan *chan,
				   struct llce_rx_msg __always_unused *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;
	void __iomem *rxout = get_rxout_fifo(chan, NULL);

	/* Disable FIFO irq even if it's shared */
	enable_rxout_irq(mb, rxout);

	return 0;
}

static int process_enable_logger_notif(struct mbox_chan *chan,
				       struct llce_rx_msg __always_unused *msg)
{
	struct llce_chan_priv *priv = chan->con_priv;
	struct llce_mb *mb = priv->mb;

	enable_logger_irq(mb);
	return 0;
}

static int process_rx_cmd(struct mbox_chan *chan, struct llce_rx_msg *msg)
{
	if (msg->cmd == LLCE_DISABLE_RX_NOTIF)
		return process_disable_rx_notif(chan, msg);
	if (msg->cmd == LLCE_ENABLE_RX_NOTIF)
		return process_enable_rx_notif(chan, msg);
	if (msg->cmd == LLCE_IS_RX_EMPTY)
		return process_is_rx_empty(chan, msg);
	if (msg->cmd == LLCE_POP_RX)
		return process_pop_rxout(chan, msg);
	if (msg->cmd == LLCE_RELEASE_RX_INDEX)
		return process_release_rxout_index(chan, msg);

	return 0;
}

static int process_logger_cmd(struct mbox_chan *chan, struct llce_rx_msg *msg)
{
	if (msg->cmd == LLCE_DISABLE_RX_NOTIF)
		return process_disable_logger_notif(chan, msg);
	if (msg->cmd == LLCE_ENABLE_RX_NOTIF)
		return process_enable_logger_notif(chan, msg);
	if (msg->cmd == LLCE_IS_RX_EMPTY)
		return process_is_logger_empty(chan, msg);
	if (msg->cmd == LLCE_POP_RX)
		return process_pop_logger(chan, msg);
	if (msg->cmd == LLCE_RELEASE_RX_INDEX)
		return process_release_logger_index(chan, msg);

	return 0;
}

static void llce_process_rxout(struct llce_mb *mb, u8 index)
{
	void __iomem *rxout = get_rxout_by_index(mb, index);
	struct device *dev = mb->controller.dev;
	struct llce_rx_msg msg;
	int ret;
	u8 hw_ctrl;

	disable_rxout_irq_by_index(mb, index);

	ret = pop_rxout_frame(mb, rxout, &msg, &hw_ctrl);
	if (ret) {
		dev_err(dev, "Failed to pop frame from RXOUT%u\n", index);
		return;
	}

	/* Save the frame and notify the channel to consume it */
	push_rxout_and_notif(mb, hw_ctrl, &msg, rxout);
}

typedef void (*icsr_consumer_t)(struct llce_mb *, u8);

static void llce_consume_icsr(struct llce_mb *mb, void __iomem *icsr_addr,
			      u8 base, icsr_consumer_t callback)
{
	u32 icsr;
	u8 i;

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
	u8 base_icsr;
	void __iomem *icsr_addr = get_rxin_icsr(mb, irq, &base_icsr);

	llce_consume_icsr(mb, icsr_addr, base_icsr, llce_process_rxin);

	return IRQ_HANDLED;
}

static irqreturn_t llce_txack_fifo_irq(int irq, void *data)
{
	struct llce_mb *mb = data;
	u8 base_icsr;
	void __iomem *icsr_addr = get_txack_icsr(mb, irq, &base_icsr);

	llce_consume_icsr(mb, icsr_addr, base_icsr, llce_process_tx_ack);

	return IRQ_HANDLED;
}

static irqreturn_t llce_rxout_fifo_irq(int irq, void *data)
{
	struct llce_mb *mb = data;
	u8 base_icsr;
	void __iomem *icsr_addr = get_rxout_icsr(mb, irq, &base_icsr);

	llce_consume_icsr(mb, icsr_addr, base_icsr, llce_process_rxout);

	return IRQ_HANDLED;
}

static irqreturn_t llce_logger_rx_irq(int irq, void *data)
{
	struct llce_mb *mb = data;
	void __iomem *icsr_addr = get_logger_icsr(mb);
	u32 icsr, mb_index, hw_ctrl;
	struct llce_can_mb frame;

	icsr = readl(icsr_addr);
	if (!(icsr & LLCE_LOGGER_ICSR_IRQ))
		return IRQ_NONE;

	disable_logger_irq(mb);

	pop_logger_frame(mb, &frame, &mb_index, &hw_ctrl);

	send_llce_logger_notif(mb, hw_ctrl, &frame, mb_index);

	return IRQ_HANDLED;
}

/* Enable interrupts from tx core to host core for a specific channel.*/
static void host2tx_enable_interrupt(struct llce_mb *mb, u32 hw_ctrl)
{
	void __iomem *hintc2er = LLCE_CORE2CORE_HINTC2ER(mb->core2core);
	u32 hintc2er_val;
	unsigned long flags;

	spin_lock_irqsave(&mb->lin_lock, flags);

	hintc2er_val = readl(hintc2er);
	writel(hintc2er_val | BIT(hw_ctrl), hintc2er);

	spin_unlock_irqrestore(&mb->lin_lock, flags);
}

/* Disable interrupts from tx core to host core for a specific channel.*/
static void host2tx_disable_interrupt(struct llce_mb *mb, u32 hw_ctrl)
{
	void __iomem *hintc2er = LLCE_CORE2CORE_HINTC2ER(mb->core2core);
	u32 hintc2er_val;
	unsigned long flags;

	spin_lock_irqsave(&mb->lin_lock, flags);

	hintc2er_val = readl(hintc2er);
	writel(hintc2er_val & (~(BIT(hw_ctrl))), hintc2er);

	spin_unlock_irqrestore(&mb->lin_lock, flags);
}

/* Clear existing interrupts from tx core for a specific channel.*/
static void host2tx_clear_interrupt(struct llce_mb *mb, u32 hw_ctrl)
{
	void __iomem *hintc2r = LLCE_CORE2CORE_HINTC2R(mb->core2core);
	unsigned long flags;

	spin_lock_irqsave(&mb->lin_lock, flags);

	writel(BIT(hw_ctrl), hintc2r);

	spin_unlock_irqrestore(&mb->lin_lock, flags);
}

static void host2tx_assert_interrupt(struct llce_mb *mb, u32 hw_ctrl)
{
	void __iomem *c2inthr = LLCE_CORE2CORE_C2INTHR(mb->core2core);
	u32 c2inthr_val;
	unsigned long flags;

	spin_lock_irqsave(&mb->lin_lock, flags);

	c2inthr_val = readl(c2inthr);
	writel(c2inthr_val | BIT(hw_ctrl), c2inthr);

	spin_unlock_irqrestore(&mb->lin_lock, flags);
}

static u32 host2tx_get_interrupts_status(struct llce_mb *mb)
{
	void __iomem *c2inthr = LLCE_CORE2CORE_C2INTHR(mb->core2core);
	u32 c2inthr_val;
	unsigned long flags;

	spin_lock_irqsave(&mb->lin_lock, flags);
	c2inthr_val = readl(c2inthr);
	spin_unlock_irqrestore(&mb->lin_lock, flags);

	return c2inthr_val;
}

static enum llce_lin_return get_config_lin_cmd_status(struct llce_mb *mb, u8 hw_ctrl)
{
	struct llce_lin_command sh_cmd;

	memcpy_fromio(&sh_cmd, &mb->lin_sh_mem->lin_cmd[hw_ctrl],
		      sizeof(sh_cmd));

	return sh_cmd.return_value;
}

static int lin_init(struct llce_mb *mb)
{
	struct llce_lin_command cmd = {
		.cmd_id = LLCE_LIN_CMD_ENABLEINTRFORWARD,
		.return_value = LLCE_LIN_SUCCESS,
	};
	struct mbox_controller *ctrl = &mb->controller;
	struct device *dev = ctrl->dev;
	u8 retries = 10, hw_ctrl = LLCE_LIN_CHANNEL0;
	int ret;
	u32 val, i;
	struct llce_lin_command __iomem *sh_cmd =
		&mb->lin_sh_mem->lin_cmd[hw_ctrl];

	/* Interrupt forwarding should be enabled only once. */
	if (mb->lin_irq_enabled)
		return 0;

	/* Disable and clear interrupts for LIN channels. */
	for (i = 0; i < LLCE_LINFLEX_NR; i++) {
		host2tx_disable_interrupt(mb, i);
		host2tx_clear_interrupt(mb, i);
	}

	/* Enable interrupt forwarding.
	 * LLCE_LIN_CMD_ENABLEINTRFORWARD enables interrupt
	 * forwarding for all LIN Channels.
	 */
	memcpy_toio(sh_cmd, &cmd, sizeof(cmd));

	/* Trigger an interrupt using CORE2CORE module
	 * on the corresponding bit of the command.
	 */
	host2tx_assert_interrupt(mb, hw_ctrl);

	/* Wait for command completion.
	 * The command is completed when core 2 clears interrupt bit
	 * for LIN channel 0.
	 */
	ret = readx_poll_timeout(host2tx_get_interrupts_status, mb, val,
				 !(val & BIT(hw_ctrl)),
				 LLCE_DELAY_US, LLCE_DELAY_US * retries);

	if (ret < 0) {
		dev_err(dev, "LLCE LIN interrupt forwarding timeout\n");
		return ret;
	}

	/* Check command status. */
	if (get_config_lin_cmd_status(mb, hw_ctrl) != LLCE_LIN_SUCCESS) {
		dev_err(dev, "LLCE LIN interrupt forwarding failed error = %d\n",
			cmd.return_value);
		return -EIO;
	}

	mb->lin_irq_enabled = true;

	return 0;
}

static irqreturn_t llce_mb_lin_handler(int irq, void *data)
{
	struct llce_mb *mb = data;
	void __iomem *hintc2r = LLCE_CORE2CORE_HINTC2R(mb->core2core);
	u32 i, hintc2r_val;
	unsigned int virq;
	unsigned long wa_lock_flags;
	irqreturn_t ret = IRQ_NONE;

	hintc2r_val = readl(hintc2r);
	for (i = 0; i < LLCE_LINFLEX_NR; i++) {
		if (!(hintc2r_val & BIT(i)))
			continue;
		virq = irq_find_mapping(mb->domain, LLCE_LIN_HWTCRL_TO_IRQ(i));
		if (!virq)
			continue;

		raw_spin_lock_irqsave(&mb->wa_lock, wa_lock_flags);
		generic_handle_irq(virq);
		raw_spin_unlock_irqrestore(&mb->wa_lock, wa_lock_flags);
		ret |= IRQ_HANDLED;
	}

	return ret;
}

static void llce_mb_lin_irq_mask(struct irq_data *data)
{
	struct llce_mb *mb = data->chip_data;
	u8 hw_ctrl = (u8)LLCE_IRQ_TO_LIN_HWCTRL(data->hwirq);

	host2tx_disable_interrupt(mb, hw_ctrl);
	host2tx_clear_interrupt(mb, hw_ctrl);
}

static void llce_mb_lin_irq_unmask(struct irq_data *data)
{
	struct llce_mb *mb = data->chip_data;
	u8 hw_ctrl = (u8)LLCE_IRQ_TO_LIN_HWCTRL(data->hwirq);

	host2tx_enable_interrupt(mb, hw_ctrl);
}

static struct irq_chip llce_mb_lin_irq_chip = {
	.name = "llce",
	.irq_mask = llce_mb_lin_irq_mask,
	.irq_unmask = llce_mb_lin_irq_unmask,
};

/* Enable interrupts from rx core to host core for a specific channel.*/
static void rx2host_enable_interrupt(struct llce_mb *mb, u32 hw_ctrl)
{
	void __iomem *hintc1er = LLCE_CORE2CORE_HINTC1ER(mb->core2core);
	u32 hintc1er_val;
	unsigned long flags;

	spin_lock_irqsave(&mb->lpspi_lock, flags);

	hintc1er_val = readl(hintc1er);
	writel(hintc1er_val | BIT(hw_ctrl), hintc1er);

	spin_unlock_irqrestore(&mb->lpspi_lock, flags);
}

/* Disable interrupts from rx core to host core for a specific channel.*/
static void rx2host_disable_interrupt(struct llce_mb *mb, u32 hw_ctrl)
{
	void __iomem *hintc1er = LLCE_CORE2CORE_HINTC1ER(mb->core2core);
	u32 hintc1er_val;
	unsigned long flags;

	spin_lock_irqsave(&mb->lpspi_lock, flags);

	hintc1er_val = readl(hintc1er);
	writel(hintc1er_val & (~(BIT(hw_ctrl))), hintc1er);

	spin_unlock_irqrestore(&mb->lpspi_lock, flags);
}

/* Clear existing interrupts from rx core for a specific channel.*/
static void rx2host_clear_interrupt(struct llce_mb *mb, u32 hw_ctrl)
{
	void __iomem *hintc1r = LLCE_CORE2CORE_HINTC1R(mb->core2core);
	unsigned long flags;

	spin_lock_irqsave(&mb->lpspi_lock, flags);

	writel(BIT(hw_ctrl), hintc1r);

	spin_unlock_irqrestore(&mb->lpspi_lock, flags);
}

static void rx2host_assert_interrupt(struct llce_mb *mb, u32 hw_ctrl)
{
	void __iomem *c1inthr = LLCE_CORE2CORE_C1INTHR(mb->core2core);
	u32 c1inthr_val;
	unsigned long flags;

	spin_lock_irqsave(&mb->lpspi_lock, flags);

	c1inthr_val = readl(c1inthr);
	writel(c1inthr_val | BIT(hw_ctrl), c1inthr);

	spin_unlock_irqrestore(&mb->lpspi_lock, flags);
}

static u32 rx2host_get_interrupts_status(struct llce_mb *mb)
{
	void __iomem *c1inthr = LLCE_CORE2CORE_C1INTHR(mb->core2core);
	u32 c1inthr_val;
	unsigned long flags;

	spin_lock_irqsave(&mb->lpspi_lock, flags);
	c1inthr_val = readl(c1inthr);
	spin_unlock_irqrestore(&mb->lpspi_lock, flags);

	return c1inthr_val;
}

static int lpspi_init(struct llce_mb *mb)
{
	struct mbox_controller *ctrl = &mb->controller;
	struct device *dev = ctrl->dev;
	u8 retries = 10, hw_ctrl = LLCE_LPSPI_CHANNEL0;
	int ret;
	u32 val, i;

	/* Interrupt forwarding should be enabled only once. */
	if (mb->lpspi_irq_enabled)
		return 0;

	/* Enable interrupts for LPSI channels. */
	for (i = 0; i < LLCE_LPSPI_NR; i++) {
		rx2host_disable_interrupt(mb, i);
		rx2host_clear_interrupt(mb, i);
		rx2host_enable_interrupt(mb, i);
	}

	/* Trigger an interrupt using CORE2CORE module
	 * on the corresponding bit of the command.
	 */
	rx2host_assert_interrupt(mb, hw_ctrl);

	/* Wait for command completion.
	 * The command is completed when core 1 clears interrupt bit
	 * for LPSPI interface 0.
	 */
	ret = readx_poll_timeout(rx2host_get_interrupts_status, mb, val,
				 !(val & BIT(hw_ctrl)),
				 LLCE_DELAY_US, LLCE_DELAY_US * retries);

	if (ret < 0) {
		dev_err(dev, "LLCE LPSPI interrupt forwarding timeout\n");
		return ret;
	}

	mb->lpspi_irq_enabled = true;

	return 0;
}

static irqreturn_t llce_mb_lspi_handler(int irq, void *data)
{
	struct llce_mb *mb = data;
	void __iomem *hintc1r = LLCE_CORE2CORE_HINTC1R(mb->core2core);
	u32 i, hintc1r_val;
	unsigned int virq;
	unsigned long wa_lock_flags;
	irqreturn_t ret = IRQ_NONE;

	hintc1r_val = readl(hintc1r);
	for (i = 0; i < LLCE_LPSPI_NR; i++) {
		if (!(hintc1r_val & BIT(i)))
			continue;
		virq = irq_find_mapping(mb->domain, LLCE_LPSPI_HWCTRL_TO_IRQ(i));
		if (!virq)
			continue;

		raw_spin_lock_irqsave(&mb->wa_lock, wa_lock_flags);
		generic_handle_irq(virq);
		raw_spin_unlock_irqrestore(&mb->wa_lock, wa_lock_flags);
		ret |= IRQ_HANDLED;
	}

	return ret;
}

static void llce_mb_lpspi_irq_mask(struct irq_data *data)
{
	struct llce_mb *mb = data->chip_data;
	u8 hw_ctrl = (u8)LLCE_IRQ_TO_LPSPI_HWCTRL(data->hwirq);

	rx2host_disable_interrupt(mb, hw_ctrl);
	rx2host_clear_interrupt(mb, hw_ctrl);
}

static void llce_mb_lpspi_irq_unmask(struct irq_data *data)
{
	struct llce_mb *mb = data->chip_data;
	u8 hw_ctrl = (u8)LLCE_IRQ_TO_LPSPI_HWCTRL(data->hwirq);

	rx2host_enable_interrupt(mb, hw_ctrl);
}

static struct irq_chip llce_mb_lpspi_irq_chip = {
	.name = "llce",
	.irq_mask = llce_mb_lpspi_irq_mask,
	.irq_unmask = llce_mb_lpspi_irq_unmask,
};

static inline bool is_lin_irq(unsigned int hw_irq)
{
	return hw_irq < LLCE_LINFLEX_NR;
}
static inline bool is_lpspi_irq(unsigned int hw_irq)
{
	return hw_irq >= LLCE_LINFLEX_NR &&
		hw_irq  < LLCE_LINFLEX_NR + LLCE_LPSPI_NR;
}

static int llce_mb_irq_map(struct irq_domain *d,
			 unsigned int irq,
			 irq_hw_number_t hw_irq)
{
	struct llce_mb *mb = d->host_data;
	int ret;

	if (is_lin_irq(hw_irq)) {
		ret = lin_init(mb);
		if (ret)
			return ret;
		irq_set_chip_and_handler(irq, &llce_mb_lin_irq_chip,
					 handle_level_irq);
	} else if (is_lpspi_irq(hw_irq)) {
		ret = lpspi_init(mb);
		if (ret)
			return ret;
		irq_set_chip_and_handler(irq, &llce_mb_lpspi_irq_chip,
					 handle_level_irq);

	} else
		/* Should not be get here. */
		return -EINVAL;

	irq_set_chip_data(irq, mb);
	irq_set_noprobe(irq);

	return 0;
}

static const struct irq_domain_ops llce_mb_irq_ops = {
	.map = llce_mb_irq_map,
	.xlate = irq_domain_xlate_onecell,
};

static int init_llce_irq(struct platform_device *pdev,
				   struct llce_mb *mb, const char *irq_name,
				   irq_handler_t handler)
{
	struct device *dev = &pdev->dev;
	int irq;

	irq = platform_get_irq_byname(pdev, irq_name);
	if (irq < 0) {
		dev_err(dev, "%s not found\n", irq_name);
		return irq;
	}

	return devm_request_irq(dev, irq, handler, 0,
			       irq_name, (void *)mb);
}

static void deinit_llce_interrupt_ctrl(struct llce_mb *mb)
{
	if (mb->domain)
		irq_domain_remove(mb->domain);
}

static int init_llce_irq_resources(struct platform_device *pdev,
				   struct llce_mb *mb)
{
	int irq, ret;
	size_t i;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct llce_fifoirq *fifo_irq;
	struct {
		const char *name;
		irq_handler_t handler;
		struct llce_fifoirq *fifo_irq;
	} resources[] = {
		{
			.name = "rxin_fifo_0_7",
			.handler = llce_rxin_fifo_irq,
			.fifo_irq = &mb->rxin_irqs.irq0,
		},
		{
			.name = "rxin_fifo_8_15",
			.handler = llce_rxin_fifo_irq,
			.fifo_irq = &mb->rxin_irqs.irq8,
		},
		{
			.name = "rxout_fifo_0_7",
			.handler = llce_rxout_fifo_irq,
			.fifo_irq = &mb->rxout_irqs.irq0,
		},
		{
			.name = "rxout_fifo_8_15",
			.handler = llce_rxout_fifo_irq,
			.fifo_irq = &mb->rxout_irqs.irq8,
		},
		{
			.name = "txack_fifo_0_7",
			.handler = llce_txack_fifo_irq,
			.fifo_irq = &mb->txack_irqs.irq0,
		},
		{
			.name = "txack_fifo_8_15",
			.handler = llce_txack_fifo_irq,
			.fifo_irq = &mb->txack_irqs.irq8,
		},
		{
			.name = "logger_rx",
			.handler = llce_logger_rx_irq,
			.fifo_irq = &mb->logger_irq,
		},
	};

	struct {
		const char *name;
		irq_handler_t handler;
	} resources_ic[] = {
		{
			.name = "linflex_irq",
			.handler = llce_mb_lin_handler,
		},
		{
			.name = "lpspi_irq",
			.handler = llce_mb_lspi_handler,
		},
	};

	for (i = 0; i < ARRAY_SIZE(resources); i++) {
		irq = platform_get_irq_byname(pdev, resources[i].name);
		if (irq < 0) {
			dev_err(dev, "Failed to request '%s' IRQ\n",
				resources[i].name);
			return irq;
		}

		fifo_irq = resources[i].fifo_irq;
		fifo_irq->name = resources[i].name;
		fifo_irq->handler = resources[i].handler;
		fifo_irq->num = irq;
	}

	if (!of_find_property(np, "interrupt-controller", NULL)) {
		dev_err(dev, "Not an interrupt-controller\n");
		return -ENXIO;
	}

	mb->domain = irq_domain_add_linear(np, LLCE_LINFLEX_NR + LLCE_LPSPI_NR,
					   &llce_mb_irq_ops, mb);

	if (!mb->domain) {
		dev_err(dev, "Failed to add irq_domain\n");
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(resources_ic); i++) {
		ret = init_llce_irq(pdev, mb, resources_ic[i].name,
				    resources_ic[i].handler);
		if (ret) {
			dev_err(dev, "Failed to request interrupt err = %d\n",
				ret);
			irq_domain_remove(mb->domain);
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
		{ .res_name = "sema42", .vaddr = &mb->sema42, },
		{ .res_name = "core2core", .vaddr = &mb->core2core, },
		{ .res_name = "system_control", .vaddr = &mb->system_ctrl, },
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

	chan_index = get_channel_offset(S32G_LLCE_HIF_CONF_MB,
					mb->hif_id);
	return &ctrl->chans[chan_index];
}

static int init_hif_config_chan(struct llce_mb *mb)
{
	struct mbox_chan *chan = get_hif_cfg_chan(mb);
	int ret;

	ret = init_chan_priv(chan, mb, S32G_LLCE_HIF_CONF_MB,
			     mb->hif_id);
	if (ret)
		return ret;

	return llce_hif_startup(chan);
}

static void deinit_hif_config_chan(struct llce_mb *mb)
{
	struct mbox_chan *chan = get_hif_cfg_chan(mb);

	deinit_chan_priv(chan);
}

static int submit_hif_cmd(struct llce_mb *mb, struct llce_can_command *cmd)
{
	struct mbox_controller *ctrl = &mb->controller;
	struct device *dev = ctrl->dev;
	static struct mbox_chan *chan;
	int ret;

	chan = get_hif_cfg_chan(mb);

	ret = execute_config_cmd(chan, cmd, LLCE_HW_CTRL_FROM_CHAN_IDX);
	if (ret) {
		dev_err(dev, "Failed to send command\n");
		return ret;
	}

	return ret;
}

static int execute_hif_cmd(struct llce_mb *mb, struct llce_can_command *cmd)
{
	struct mbox_controller *ctrl = &mb->controller;
	struct device *dev = ctrl->dev;
	static struct mbox_chan *chan;
	unsigned char retries = 10;
	int ret;

	ret = submit_hif_cmd(mb, cmd);
	if (ret)
		return ret;

	chan = get_hif_cfg_chan(mb);

	/* Wait for command completion */
	while (!llce_mb_last_tx_done(chan) && retries--)
		msleep(100);

	if (cmd->return_value != LLCE_FW_SUCCESS) {
		dev_err(dev, "LLCE FW error %d\n", cmd->return_value);
		return -EIO;
	}

	return 0;
}

static unsigned long get_fifo_mask(u32 hif, bool multihost)
{
	/* All 16 FIFOs available */
	if (!multihost)
		return GENMASK(15, 0);

	/* 0-7 FIFOs are reserved for HIF0 */
	if (hif == 1)
		return GENMASK(15, 8);

	/* 8-15 FIFOs are reserved for HIF1 */
	return GENMASK(7, 0);
}

static unsigned long get_next_free_fifo(unsigned long *availability,
					u32 hif, bool multihost)
{
	unsigned long index;

	if (!*availability)
		*availability = get_fifo_mask(hif, multihost);

	index = find_first_bit(availability, LLCE_NFIFO_WITH_IRQ);
	*availability &= ~BIT(index);

	return index;
}

static void init_chans_params(struct llce_mb *mb)
{
	unsigned long ctrl_id;

	for (ctrl_id = 0; ctrl_id < ARRAY_SIZE(mb->chans_params); ctrl_id++) {
		mb->chans_params[ctrl_id] = (struct llce_chan_params) {
			.fifo = UNINITIALIZED_FIFO,
			.max_regular_filters = MAX_FILTER_PER_CHAN,
			.max_adv_filters = MAX_FILTER_PER_CHAN,
			.max_rx_mb = FIFO_MB_PER_CHAN,
			.max_tx_ack = MAX_CONFIRM_BUF_PER_CHAN,
		};
	}
}

static int of_read_adv_ctrl_options(struct device *dev,
				    struct device_node *np,
				    struct llce_chan_params *params)
{
	int ret;
	size_t i;
	u32 val;
	struct {
		const char *name;
		u16 *value;
	} props[] = {
		{
			.name = "nxp,max_regular_filters",
			.value = &params->max_regular_filters
		},
		{
			.name = "nxp,max_adv_filters",
			.value = &params->max_adv_filters,
		},
		{
			.name = "nxp,max_rx_mb",
			.value = &params->max_rx_mb,
		},
		{
			.name = "nxp,max_tx_ack",
			.value = &params->max_tx_ack,
		},
	};

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		ret = of_property_read_u32(np, props[i].name, &val);
		if (!ret) {
			if (val > U16_MAX)
				return -E2BIG;

			*props[i].value = val;
			dev_dbg(dev, "%s:%s = %u\n", of_node_full_name(np),
				props[i].name, *props[i].value);
			continue;
		}

		/* The property does not exist */
		if (ret == -EINVAL)
			continue;

		dev_err(dev, "Failed to parse property %s of the node %s\n",
			props[i].name, of_node_full_name(np));
		return ret;
	}

	return 0;
}

static int llce_init_chan_map(struct device *dev, struct llce_mb *mb)
{
	const char *node_name;
	struct device_node *child;
	struct device_node *node = dev->of_node;
	unsigned long fifo_availability = 0;
	unsigned long fifo_id, ctrl_id;
	u8 fifos_refcnt[LLCE_NFIFO_WITH_IRQ];
	bool shared_fifo = false;
	size_t i;
	u32 hif_id = 0;
	int ret;

	mb->multihif = of_property_read_bool(node, "nxp,multi-hif");

	ret = of_property_read_u32(node, "nxp,hif-id", &hif_id);
	if (ret)
		hif_id = LLCE_CAN_HIF0;

	if (hif_id != LLCE_CAN_HIF0 && hif_id != LLCE_CAN_HIF1) {
		dev_err(dev, "Unknown HIF id %u\n", hif_id);
		return -EINVAL;
	}

	mb->hif_id = hif_id;

	init_chans_params(mb);

	memset(&fifos_refcnt, 0, sizeof(fifos_refcnt));

	for_each_child_of_node(node->parent, child) {
		if (!(of_device_is_compatible(child, LLCE_CAN_COMPATIBLE) &&
		      of_device_is_available(child)))
			continue;

		node_name = child->name;
		ret = get_llce_can_id(node_name, &ctrl_id);
		if (ret) {
			dev_err(dev, "Failed to get ID of the node: %s\n",
				node_name);
			return ret;
		}

		if (ctrl_id >= LLCE_CAN_CONFIG_MAXCTRL_COUNT) {
			dev_info(dev, "Ignoring controller %s\n", node_name);
			continue;
		}

		fifo_id = get_next_free_fifo(&fifo_availability, mb->hif_id,
					     mb->multihif);
		if (fifo_id >= LLCE_NFIFO_WITH_IRQ) {
			dev_err(dev, "Failed to identify set of FIFOs for BCAN %lu\n",
				ctrl_id);
			return -EINVAL;
		}

		mb->chans_params[ctrl_id].fifo = fifo_id;

		/* Detect shared FIFOs */
		fifos_refcnt[fifo_id]++;
		if (fifos_refcnt[fifo_id] > 1)
			shared_fifo = true;

		ret = of_read_adv_ctrl_options(dev, child,
					       &mb->chans_params[ctrl_id]);
		if (ret)
			return ret;
	}

	if (shared_fifo) {
		dev_warn(dev, "Interfaces that use shared TX/RX FIFOs:");
		for (i = 0; i < ARRAY_SIZE(mb->chans_params); i++) {
			fifo_id = mb->chans_params[i].fifo;

			if (fifo_id >= sizeof(fifos_refcnt))
				continue;

			if (fifos_refcnt[fifo_id] > 1)
				dev_warn(dev, "\tLLCE CAN %lu uses shared FIFOs: %lu\n",
					 i, fifo_id);
		}
	}

	return 0;
}

static int llce_platform_init(struct device *dev, struct llce_mb *mb)
{
	struct llce_can_init_platform_cmd *pcmd;
	struct llce_chan_params *chan_params;
	unsigned long ctrl_id, fifo_id;

	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_INIT_PLATFORM,
		.cmd_list.init_platform = {
			.can_error_reporting = {
				.can_protocol_err = NOTIF_FIFO0,
				.data_lost_err = NOTIF_FIFO0,
				.init_err = NOTIF_FIFO0,
				.internal_err = NOTIF_FIFO0,
			},
		},
	};

	if (!config_platform)
		return 0;

	pcmd = &cmd.cmd_list.init_platform;
	memset(&pcmd->ctrl_init_status, UNINITIALIZED,
	       sizeof(pcmd->ctrl_init_status));
	memset(&pcmd->max_int_tx_ack_count, 0,
	       sizeof(pcmd->max_int_tx_ack_count));
	memset(&pcmd->max_poll_tx_ack_count, 0,
	       sizeof(pcmd->max_poll_tx_ack_count));
	memset(&pcmd->can_error_reporting.bus_off_err, IGNORE,
	       sizeof(pcmd->can_error_reporting.bus_off_err));
	memset(&pcmd->max_regular_filter_count, 0,
	       sizeof(pcmd->max_regular_filter_count));
	memset(&pcmd->max_advanced_filter_count, 0,
	       sizeof(pcmd->max_advanced_filter_count));
	memset(&pcmd->max_int_mb_count, 0, sizeof(pcmd->max_int_mb_count));
	memset(&pcmd->max_poll_mb_count, 0, sizeof(pcmd->max_poll_mb_count));

	for (ctrl_id = 0; ctrl_id < ARRAY_SIZE(mb->chans_params); ctrl_id++) {
		chan_params = &mb->chans_params[ctrl_id];
		fifo_id = chan_params->fifo;

		/* Not initialized */
		if (fifo_id == UNINITIALIZED_FIFO)
			continue;

		/* Per controller settings */
		pcmd->ctrl_init_status[ctrl_id] = INITIALIZED;
		pcmd->max_regular_filter_count[ctrl_id] =
		    chan_params->max_regular_filters;
		pcmd->max_advanced_filter_count[ctrl_id] =
		    chan_params->max_adv_filters;
		pcmd->can_error_reporting.bus_off_err[ctrl_id] = NOTIF_FIFO0;

		/* Per FIFO settings */
		if (check_add_overflow(pcmd->max_int_mb_count[fifo_id],
				       chan_params->max_rx_mb,
				       &pcmd->max_int_mb_count[fifo_id]))
			return -EOVERFLOW;

		if (check_add_overflow(pcmd->max_int_tx_ack_count[fifo_id],
				       chan_params->max_tx_ack,
				       &pcmd->max_int_tx_ack_count[fifo_id]))
			return -EOVERFLOW;
	}

	return execute_hif_cmd(mb, &cmd);
}

static int llce_platform_deinit(struct llce_mb *mb)
{
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_DEINIT_PLATFORM,
	};

	if (!config_platform)
		return 0;

	return execute_hif_cmd(mb, &cmd);
}

static int get_fw_version(struct llce_mb *mb,
			  struct llce_fw_version *ver)
{
	struct llce_can_command cmd = {
		.cmd_id = LLCE_CAN_CMD_GETFWVERSION,
	};
	int ret;

	if (!config_platform)
		return 0;

	ret = execute_hif_cmd(mb, &cmd);
	if (ret)
		return ret;

	*ver = cmd.cmd_list.get_fw_version;

	return 0;
}

static void fw_logger_support(struct llce_mb *mb,
			      struct llce_fw_version *ver)
{
	struct device *dev = mb->controller.dev;

	mb->fw_logger_support =
		!(ver->version_string[LLCE_LOGGING] == LLCE_FEATURE_DISABLED);
	dev_info(dev, "LLCE firmware: logging support %s\n",
		 mb->fw_logger_support ? "enabled" : "disabled");
}

static void print_fw_version(struct llce_mb *mb,
			     struct llce_fw_version *ver)
{
	struct device *dev = mb->controller.dev;
	unsigned char *ver_str = ver->version_string;
	size_t extra_pos, max_size = sizeof(ver->version_string);
	size_t ver_len = strnlen((const char *)ver_str, max_size);
	unsigned char *extra = NULL, *ptr;

	/* Just to be sure */
	ver_str[max_size - 1] = 0;

	extra_pos = ver_len + 1;
	if (extra_pos < max_size && isprint(ver_str[extra_pos])) {
		extra = &ver_str[extra_pos];
		ptr = extra;

		/* Version's extra tokens are separated by '\0' */
		while (*ptr && (ptr + 2 < ver_str + max_size)) {
			if (!*(ptr + 1) && isprint(*(ptr + 2)))
				*(ptr + 1) = '|';
			ptr++;
		}
	}

	if (extra)
		dev_info(dev, "LLCE firmware version: %s [%s]\n", ver_str, extra);
	else
		dev_info(dev, "LLCE firmware version: %s\n", ver_str);
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
	struct llce_fw_version ver;
	struct mbox_controller *ctrl;
	struct llce_mb *mb;
	struct device *dev = &pdev->dev;
	int ret;
	u32 sysrstr;

	mb = devm_kzalloc(&pdev->dev, sizeof(*mb), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;

	spin_lock_init(&mb->txack_lock);
	spin_lock_init(&mb->lin_lock);
	spin_lock_init(&mb->fifos_irq_ref_cnt.lock);
	raw_spin_lock_init(&mb->wa_lock);

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

	mb->dev = dev;

	ret = llce_init_chan_map(dev, mb);
	if (ret)
		return ret;

	ret = init_llce_mem_resources(pdev, mb);
	if (ret)
		return ret;

	ret = init_llce_irq_resources(pdev, mb);
	if (ret)
		return ret;

	ret = map_llce_shmem(mb);
	if (ret)
		goto interrupt_ctrl_deinit;

	ret = init_hif_config_chan(mb);
	if (ret) {
		dev_err(dev, "Failed to initialize HIF config channel\n");
		goto interrupt_ctrl_deinit;
	}

	ret = init_core_clock(dev, &mb->clk);
	if (ret)
		goto hif_deinit;

	sysrstr = readl(mb->system_ctrl + LLCE_SYSRSTR);
	/* If LLCE modules are under reset, the firmware was not loaded. */
	if (!sysrstr) {
		dev_err(dev, "LLCE modules are under reset. Is the LLCE firmware loaded?\n");
		ret = -EACCES;
		goto hif_deinit;
	}

	ret = get_fw_version(mb, &ver);
	if (ret) {
		dev_err(dev, "Failed to get firmware version\n");
		goto disable_clk;
	}

	print_fw_version(mb, &ver);

	fw_logger_support(mb, &ver);

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

interrupt_ctrl_deinit:
	if (ret)
		deinit_llce_interrupt_ctrl(mb);

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
	deinit_llce_interrupt_ctrl(mb);

	return 0;
}

static int __maybe_unused llce_mb_suspend(struct device *dev)
{
	struct llce_mb *mb = dev_get_drvdata(dev);
	int ret;

	mb->suspended = true;

	ret = llce_platform_deinit(mb);
	if (ret)
		dev_err(dev, "Failed to deinitialize LLCE platform");

	clk_disable_unprepare(mb->clk);

	return 0;
}

static int __maybe_unused llce_mb_resume(struct device *dev)
{
	int ret;
	struct llce_mb *mb = dev_get_drvdata(dev);

	ret = clk_prepare_enable(mb->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	ret = llce_platform_init(dev, mb);
	if (ret)
		dev_err(dev, "Failed to initialize platform\n");

	/* Force lin intterupt forwarding again. */
	if (mb->lin_irq_enabled) {
		mb->lin_irq_enabled = false;
		ret = lin_init(mb);
		if (ret)
			return ret;
	}

	/* Force lpspi intterupt forwarding again. */
	if (mb->lpspi_irq_enabled) {
		mb->lpspi_irq_enabled = false;
		ret = lpspi_init(mb);
		if (ret)
			return ret;
	}

	mb->suspended = false;

	return ret;
}

static const struct of_device_id llce_mb_match[] = {
	{
		.compatible = "nxp,s32g-llce-mailbox",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, llce_mb_match);

static SIMPLE_DEV_PM_OPS(llce_mb_pm_ops, llce_mb_suspend, llce_mb_resume);

static struct platform_driver llce_mb_driver = {
	.probe = llce_mb_probe,
	.remove = llce_mb_remove,
	.driver = {
		.name = "llce_mb",
		.of_match_table = llce_mb_match,
		.pm = &llce_mb_pm_ops,
	},
};
module_platform_driver(llce_mb_driver)

MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("NXP LLCE Mailbox");
MODULE_LICENSE("GPL");
