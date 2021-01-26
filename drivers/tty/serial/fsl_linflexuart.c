// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Freescale LINFlexD UART serial port driver
 *
 * Copyright 2012-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
 */

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>
#include <linux/jiffies.h>
#include <linux/kgdb.h>
#include <linux/delay.h>

/* All registers are 32-bit width */

#define LINCR1	0x0000	/* LIN control register				*/
#define LINIER	0x0004	/* LIN interrupt enable register		*/
#define LINSR	0x0008	/* LIN status register				*/
#define LINESR	0x000C	/* LIN error status register			*/
#define UARTCR	0x0010	/* UART mode control register			*/
#define UARTSR	0x0014	/* UART mode status register			*/
#define LINTCSR	0x0018	/* LIN timeout control status register		*/
#define LINOCR	0x001C	/* LIN output compare register			*/
#define LINTOCR	0x0020	/* LIN timeout control register			*/
#define LINFBRR	0x0024	/* LIN fractional baud rate register		*/
#define LINIBRR	0x0028	/* LIN integer baud rate register		*/
#define LINCFR	0x002C	/* LIN checksum field register			*/
#define LINCR2	0x0030	/* LIN control register 2			*/
#define BIDR	0x0034	/* Buffer identifier register			*/
#define BDRL	0x0038	/* Buffer data register least significant	*/
#define BDRM	0x003C	/* Buffer data register most significant	*/
#define IFER	0x0040	/* Identifier filter enable register		*/
#define IFMI	0x0044	/* Identifier filter match index		*/
#define IFMR	0x0048	/* Identifier filter mode register		*/
#define GCR	0x004C	/* Global control register			*/
#define UARTPTO	0x0050	/* UART preset timeout register			*/
#define UARTCTO	0x0054	/* UART current timeout register		*/
/* The offsets for DMARXE/DMATXE in master mode only			*/
#define DMATXE	0x0058	/* DMA Tx enable register			*/
#define DMARXE	0x005C	/* DMA Rx enable register			*/

/*
 * Register field definitions
 */

#define LINFLEXD_LINCR1_INIT		BIT(0)
#define LINFLEXD_LINCR1_MME		BIT(4)
#define LINFLEXD_LINCR1_BF		BIT(7)

#define LINFLEXD_LINSR_LINS_INITMODE	BIT(12)
#define LINFLEXD_LINSR_LINS_MASK	(0xF << 12)

#define LINFLEXD_LINIER_SZIE		BIT(15)
#define LINFLEXD_LINIER_OCIE		BIT(14)
#define LINFLEXD_LINIER_BEIE		BIT(13)
#define LINFLEXD_LINIER_CEIE		BIT(12)
#define LINFLEXD_LINIER_HEIE		BIT(11)
#define LINFLEXD_LINIER_FEIE		BIT(8)
#define LINFLEXD_LINIER_BOIE		BIT(7)
#define LINFLEXD_LINIER_LSIE		BIT(6)
#define LINFLEXD_LINIER_WUIE		BIT(5)
#define LINFLEXD_LINIER_DBFIE		BIT(4)
#define LINFLEXD_LINIER_DBEIETOIE	BIT(3)
#define LINFLEXD_LINIER_DRIE		BIT(2)
#define LINFLEXD_LINIER_DTIE		BIT(1)
#define LINFLEXD_LINIER_HRIE		BIT(0)

#define LINFLEXD_UARTCR_OSR_MASK	(0xF << 24)
#define LINFLEXD_UARTCR_OSR(uartcr)	(((uartcr) \
					& LINFLEXD_UARTCR_OSR_MASK) >> 24)

#define LINFLEXD_UARTCR_ROSE		BIT(23)

#define LINFLEXD_UARTCR_RFBM		BIT(9)
#define LINFLEXD_UARTCR_TFBM		BIT(8)
#define LINFLEXD_UARTCR_WL1		BIT(7)
#define LINFLEXD_UARTCR_PC1		BIT(6)

#define LINFLEXD_UARTCR_RXEN		BIT(5)
#define LINFLEXD_UARTCR_TXEN		BIT(4)
#define LINFLEXD_UARTCR_PC0		BIT(3)

#define LINFLEXD_UARTCR_PCE		BIT(2)
#define LINFLEXD_UARTCR_WL0		BIT(1)
#define LINFLEXD_UARTCR_UART		BIT(0)

#define LINFLEXD_UARTSR_SZF		BIT(15)
#define LINFLEXD_UARTSR_OCF		BIT(14)
#define LINFLEXD_UARTSR_PE3		BIT(13)
#define LINFLEXD_UARTSR_PE2		BIT(12)
#define LINFLEXD_UARTSR_PE1		BIT(11)
#define LINFLEXD_UARTSR_PE0		BIT(10)
#define LINFLEXD_UARTSR_RMB		BIT(9)
#define LINFLEXD_UARTSR_FEF		BIT(8)
#define LINFLEXD_UARTSR_BOF		BIT(7)
#define LINFLEXD_UARTSR_RPS		BIT(6)
#define LINFLEXD_UARTSR_WUF		BIT(5)
#define LINFLEXD_UARTSR_4		BIT(4)

#define LINFLEXD_UARTSR_TO		BIT(3)

#define LINFLEXD_UARTSR_DRFRFE		BIT(2)
#define LINFLEXD_UARTSR_DTFTFF		BIT(1)
#define LINFLEXD_UARTSR_NF		BIT(0)
#define LINFLEXD_UARTSR_PE		(LINFLEXD_UARTSR_PE0 |\
					 LINFLEXD_UARTSR_PE1 |\
					 LINFLEXD_UARTSR_PE2 |\
					 LINFLEXD_UARTSR_PE3)

#define FSL_UART_RX_DMA_BUFFER_SIZE	(PAGE_SIZE)

#define LINFLEXD_UARTCR_FIFO_SIZE	(4)

#define DRIVER_NAME	"fsl-linflexuart"
#define DEV_NAME	"ttyLF"
#define UART_NR		4

#define EARLYCON_BUFFER_INITIAL_CAP	8

#define PREINIT_DELAY			2000 /* us */

#ifdef CONFIG_CONSOLE_POLL
struct linflex_poll_ctx {
	u32 ier;
	u32 cr;
	u32 dmatxe;
	u32 dmarxe;
	bool in_use;
};
#endif

struct linflex_port {
	struct uart_port	port;
	struct clk		*clk;
	struct clk		*clk_ipg;
	unsigned int		txfifo_size;
	unsigned int		rxfifo_size;
	bool			dma_tx_use;
	bool			dma_rx_use;
	struct dma_chan		*dma_tx_chan;
	struct dma_chan		*dma_rx_chan;
	struct dma_async_tx_descriptor  *dma_tx_desc;
	struct dma_async_tx_descriptor  *dma_rx_desc;
	dma_addr_t		dma_tx_buf_bus;
	dma_addr_t		dma_rx_buf_bus;
	dma_cookie_t		dma_tx_cookie;
	dma_cookie_t		dma_rx_cookie;
	unsigned char		*dma_tx_buf_virt;
	unsigned char		*dma_rx_buf_virt;
	unsigned int		dma_tx_bytes;
	int			dma_tx_in_progress;
	int			dma_rx_in_progress;
	unsigned int		dma_rx_timeout;
	struct timer_list	timer;
#ifdef CONFIG_CONSOLE_POLL
	struct linflex_poll_ctx poll_ctx;
#endif
};

static const struct of_device_id linflex_dt_ids[] = {
	{
		.compatible = "nxp,s32cc-linflexuart",
	},
	{
		.compatible = "fsl,s32v234-linflexuart",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, linflex_dt_ids);

#ifdef CONFIG_SERIAL_FSL_LINFLEXUART_CONSOLE
static struct uart_port *earlycon_port;
static bool linflex_earlycon_same_instance;
static DEFINE_SPINLOCK(init_lock);
static bool during_init;

static struct {
	char *content;
	unsigned int len, cap;
} earlycon_buf;
#endif

/* Forward declare this for the dma callbacks. */
static void linflex_dma_tx_complete(void *arg);
static void linflex_dma_rx_complete(void *arg);
static void linflex_console_putchar(struct uart_port *port, int ch);
#ifdef CONFIG_CONSOLE_POLL
static void linflex_poll_release(struct linflex_port *sport);
#endif

static inline struct linflex_port *
to_linflex_port(struct uart_port *uart)
{
	return container_of(uart, struct linflex_port, port);
}

static void linflex_copy_rx_to_tty(struct linflex_port *lfport,
				   struct tty_port *tty, int count)
{
	int copied;

	lfport->port.icount.rx += count;

	if (!tty) {
		dev_err(lfport->port.dev, "No tty port\n");
		return;
	}

	dma_sync_single_for_cpu(lfport->port.dev, lfport->dma_rx_buf_bus,
				FSL_UART_RX_DMA_BUFFER_SIZE, DMA_FROM_DEVICE);
	copied = tty_insert_flip_string(tty,
					((unsigned char *)(lfport->dma_rx_buf_virt)),
					count);

	if (copied != count) {
		WARN_ON(1);
		dev_err(lfport->port.dev, "RxData copy to tty layer failed\n");
	}

	dma_sync_single_for_device(lfport->port.dev, lfport->dma_rx_buf_bus,
				   FSL_UART_RX_DMA_BUFFER_SIZE, DMA_TO_DEVICE);
}

static void linflex_stop_tx(struct uart_port *port)
{
	unsigned long ier;
	unsigned int count;
	struct dma_tx_state state;
	struct linflex_port *lfport = to_linflex_port(port);
	struct circ_buf *xmit = &port->state->xmit;

	if (!lfport->dma_tx_use) {
		ier = readl(port->membase + LINIER);
		ier &= ~(LINFLEXD_LINIER_DTIE);
		writel(ier, port->membase + LINIER);
	} else if (lfport->dma_tx_in_progress) {
		dmaengine_pause(lfport->dma_tx_chan);
		dmaengine_tx_status(lfport->dma_tx_chan,
				    lfport->dma_tx_cookie, &state);
		dmaengine_terminate_all(lfport->dma_tx_chan);
		dma_sync_single_for_cpu(lfport->port.dev, lfport->dma_tx_buf_bus,
					lfport->dma_tx_bytes, DMA_TO_DEVICE);
		count = lfport->dma_tx_bytes - state.residue;
		xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
		port->icount.tx += count;

		lfport->dma_tx_in_progress = 0;
	}
}

static void linflex_stop_rx(struct uart_port *port)
{
	unsigned long ier;
	unsigned int count;
	struct dma_tx_state state;
	struct linflex_port *lfport = to_linflex_port(port);

	if (!lfport->dma_rx_use) {
		ier = readl(port->membase + LINIER);
		writel(ier & ~LINFLEXD_LINIER_DRIE, port->membase + LINIER);
	} else if (lfport->dma_rx_in_progress) {
		del_timer(&lfport->timer);
		dmaengine_pause(lfport->dma_rx_chan);
		dmaengine_tx_status(lfport->dma_rx_chan,
				    lfport->dma_rx_cookie, &state);
		dmaengine_terminate_all(lfport->dma_rx_chan);
		count = FSL_UART_RX_DMA_BUFFER_SIZE - state.residue;

		lfport->dma_rx_in_progress = 0;
		linflex_copy_rx_to_tty(lfport, &port->state->port, count);
		tty_flip_buffer_push(&port->state->port);
	}
}

static void linflex_put_char(struct uart_port *sport, unsigned char c)
{
	struct linflex_port *lfport = to_linflex_port(sport);
	unsigned long status;

	writeb(c, sport->membase + BDRL);

	/* Waiting for data transmission completed. */
	if (!lfport->dma_tx_use) {
		while (((status = readl(sport->membase + UARTSR)) &
					LINFLEXD_UARTSR_DTFTFF) !=
					LINFLEXD_UARTSR_DTFTFF)
			;
	} else {
		while (((status = readl(sport->membase + UARTSR)) &
					LINFLEXD_UARTSR_DTFTFF))
			;
	}

	if (!lfport->dma_tx_use)
		writel(status | LINFLEXD_UARTSR_DTFTFF,
		       sport->membase + UARTSR);
}

static inline void linflex_transmit_buffer(struct uart_port *sport)
{
	struct circ_buf *xmit = &sport->state->xmit;

	while (!uart_circ_empty(xmit)) {
		linflex_put_char(sport, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(sport);

	if (uart_circ_empty(xmit))
		linflex_stop_tx(sport);
}

static int linflex_dma_tx(struct linflex_port *lfport, unsigned long count)
{
	struct circ_buf *xmit = &lfport->port.state->xmit;
	struct uart_port *sport = &lfport->port;
	dma_addr_t tx_bus_addr;

	while ((readl(sport->membase + UARTSR) & LINFLEXD_UARTSR_DTFTFF))
		;

	dma_sync_single_for_device(sport->dev, lfport->dma_tx_buf_bus,
				   UART_XMIT_SIZE, DMA_TO_DEVICE);
	lfport->dma_tx_bytes = count;
	tx_bus_addr = lfport->dma_tx_buf_bus + xmit->tail;
	lfport->dma_tx_desc =
		dmaengine_prep_slave_single(lfport->dma_tx_chan, tx_bus_addr,
					    lfport->dma_tx_bytes, DMA_MEM_TO_DEV,
					    DMA_PREP_INTERRUPT | DMA_CTRL_ACK);

	if (!lfport->dma_tx_desc) {
		dev_err(sport->dev, "Not able to get desc for tx\n");
		return -EIO;
	}

	lfport->dma_tx_desc->callback = linflex_dma_tx_complete;
	lfport->dma_tx_desc->callback_param = sport;
	lfport->dma_tx_in_progress = 1;
	lfport->dma_tx_cookie = dmaengine_submit(lfport->dma_tx_desc);
	dma_async_issue_pending(lfport->dma_tx_chan);

	return 0;
}

static void linflex_prepare_tx(struct linflex_port *lfport)
{
	struct circ_buf *xmit = &lfport->port.state->xmit;
	unsigned long count =  CIRC_CNT_TO_END(xmit->head, xmit->tail,
					       UART_XMIT_SIZE);

	if (!count || lfport->dma_tx_in_progress)
		return;

	linflex_dma_tx(lfport, count);
}

static void linflex_dma_tx_complete(void *arg)
{
	struct linflex_port *lfport = arg;
	struct circ_buf *xmit = &lfport->port.state->xmit;
	unsigned long flags;

	spin_lock_irqsave(&lfport->port.lock, flags);

	xmit->tail = (xmit->tail + lfport->dma_tx_bytes) & (UART_XMIT_SIZE - 1);
	lfport->port.icount.tx += lfport->dma_tx_bytes;
	lfport->dma_tx_in_progress = 0;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&lfport->port);

	linflex_prepare_tx(lfport);

	spin_unlock_irqrestore(&lfport->port.lock, flags);
}

static void linflex_flush_buffer(struct uart_port *port)
{
	struct linflex_port *lfport = to_linflex_port(port);

	if (lfport->dma_tx_use) {
		dmaengine_terminate_all(lfport->dma_tx_chan);
		lfport->dma_tx_in_progress = 0;
	}
}

static int linflex_dma_rx(struct linflex_port *lfport)
{
	dma_sync_single_for_device(lfport->port.dev, lfport->dma_rx_buf_bus,
				   FSL_UART_RX_DMA_BUFFER_SIZE,
				   DMA_FROM_DEVICE);
	lfport->dma_rx_desc = dmaengine_prep_slave_single(lfport->dma_rx_chan,
							  lfport->dma_rx_buf_bus,
							  FSL_UART_RX_DMA_BUFFER_SIZE,
							  DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT |
							  DMA_CTRL_ACK);

	if (!lfport->dma_rx_desc) {
		dev_err(lfport->port.dev, "Not able to get desc for rx\n");
		return -EIO;
	}

	lfport->dma_rx_desc->callback = linflex_dma_rx_complete;
	lfport->dma_rx_desc->callback_param = lfport;
	lfport->dma_rx_in_progress = 1;
	lfport->dma_rx_cookie = dmaengine_submit(lfport->dma_rx_desc);
	dma_async_issue_pending(lfport->dma_rx_chan);

	return 0;
}

static void linflex_dma_rx_complete(void *arg)
{
	struct linflex_port *lfport = arg;
	struct tty_port *port = &lfport->port.state->port;
	unsigned long flags;

	del_timer(&lfport->timer);

	spin_lock_irqsave(&lfport->port.lock, flags);

#ifdef CONFIG_CONSOLE_POLL
	if (!kgdb_connected && sport->poll_ctx.in_use) {
		sport->poll_ctx.in_use = false;
		linflex_poll_release(sport);
	}
#endif
	lfport->dma_rx_in_progress = 0;
	linflex_copy_rx_to_tty(lfport, port, FSL_UART_RX_DMA_BUFFER_SIZE);
	tty_flip_buffer_push(port);
	linflex_dma_rx(lfport);

	spin_unlock_irqrestore(&lfport->port.lock, flags);

	mod_timer(&lfport->timer, jiffies + lfport->dma_rx_timeout);
}

static void linflex_timer_func(unsigned long data)
{
	struct linflex_port *lfport = (struct linflex_port *)data;
	struct tty_port *port = &lfport->port.state->port;
	struct dma_tx_state state;
	unsigned long flags;
	int count;

#ifdef CONFIG_CONSOLE_POLL
	if (!kgdb_connected && sport->poll_ctx.in_use) {
		sport->poll_ctx.in_use = false;
		linflex_poll_release(sport);
	}
#endif
	del_timer(&lfport->timer);
	dmaengine_pause(lfport->dma_rx_chan);
	dmaengine_tx_status(lfport->dma_rx_chan, lfport->dma_rx_cookie, &state);
	dmaengine_terminate_all(lfport->dma_rx_chan);
	count = FSL_UART_RX_DMA_BUFFER_SIZE - state.residue;

	spin_lock_irqsave(&lfport->port.lock, flags);

	lfport->dma_rx_in_progress = 0;
	linflex_copy_rx_to_tty(lfport, port, count);
	tty_flip_buffer_push(port);

	linflex_dma_rx(lfport);

	spin_unlock_irqrestore(&lfport->port.lock, flags);
	mod_timer(&lfport->timer, jiffies + lfport->dma_rx_timeout);
}

static void linflex_start_tx(struct uart_port *port)
{
	struct linflex_port *lfport = to_linflex_port(port);
	unsigned long ier;

	if (lfport->dma_tx_use) {
		linflex_prepare_tx(lfport);
	} else {
		linflex_transmit_buffer(port);
		ier = readl(port->membase + LINIER);
		writel(ier | LINFLEXD_LINIER_DTIE, port->membase + LINIER);
	}
}

static irqreturn_t linflex_txint(int irq, void *dev_id)
{
	struct linflex_port *lfport = dev_id;
	struct uart_port *sport = &lfport->port;
	struct circ_buf *xmit = &sport->state->xmit;
	unsigned long flags;

	spin_lock_irqsave(&sport->lock, flags);

	if (sport->x_char) {
		linflex_put_char(sport, sport->x_char);
		goto out;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(sport)) {
		linflex_stop_tx(sport);
		goto out;
	}

	linflex_transmit_buffer(sport);
out:
	spin_unlock_irqrestore(&sport->lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t linflex_rxint(int irq, void *dev_id)
{
	struct linflex_port *lfport = dev_id;
	struct uart_port *sport = &lfport->port;
	unsigned int flg;
	struct tty_port *port = &sport->state->port;
	unsigned long flags, status;
	unsigned char rx;
	bool brk;

	spin_lock_irqsave(&sport->lock, flags);

	status = readl(sport->membase + UARTSR);
	while (status & LINFLEXD_UARTSR_RMB) {
		rx = readb(sport->membase + BDRM);
		brk = false;
		flg = TTY_NORMAL;
		sport->icount.rx++;

		if (status & (LINFLEXD_UARTSR_BOF | LINFLEXD_UARTSR_FEF |
				LINFLEXD_UARTSR_PE)) {
			if (status & LINFLEXD_UARTSR_BOF)
				sport->icount.overrun++;
			if (status & LINFLEXD_UARTSR_FEF) {
				if (!rx) {
					brk = true;
					sport->icount.brk++;
				} else
					sport->icount.frame++;
			}
			if (status & LINFLEXD_UARTSR_PE)
				sport->icount.parity++;
		}

		writel(status, sport->membase + UARTSR);
		status = readl(sport->membase + UARTSR);

		if (brk) {
			uart_handle_break(sport);
		} else {
			if (uart_handle_sysrq_char(sport, (unsigned char)rx))
				continue;
			tty_insert_flip_char(port, rx, flg);
		}
	}

	spin_unlock_irqrestore(&sport->lock, flags);

	tty_flip_buffer_push(port);

	return IRQ_HANDLED;
}

static irqreturn_t linflex_int(int irq, void *dev_id)
{
	struct linflex_port *lfport = dev_id;

	unsigned long status;

	status = readl(lfport->port.membase + UARTSR);

	if (status & LINFLEXD_UARTSR_DRFRFE && !lfport->dma_rx_use)
		linflex_rxint(irq, dev_id);
	if (status & LINFLEXD_UARTSR_DTFTFF && !lfport->dma_rx_use)
		linflex_txint(irq, dev_id);

	return IRQ_HANDLED;
}

/* return TIOCSER_TEMT when transmitter is not busy */
static unsigned int linflex_tx_empty(struct uart_port *port)
{
	struct linflex_port *lfport = to_linflex_port(port);
	unsigned long status;

	status = readl(port->membase + UARTSR) & LINFLEXD_UARTSR_DTFTFF;

	if (!lfport->dma_tx_use)
		return status ? TIOCSER_TEMT : 0;
	else
		return status ? 0 : TIOCSER_TEMT;
}

static unsigned int linflex_get_mctrl(struct uart_port *port)
{
	return 0;
}

static void linflex_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static void linflex_break_ctl(struct uart_port *port, int break_state)
{
}

static void linflex_setup_watermark(struct uart_port *sport)
{
	struct linflex_port *lfport = to_linflex_port(sport);
	unsigned long cr, ier, cr1;

	/* Disable transmission/reception */
	ier = readl(sport->membase + LINIER);
	ier &= ~(LINFLEXD_LINIER_DRIE | LINFLEXD_LINIER_DTIE);
	writel(ier, sport->membase + LINIER);

	cr = readl(sport->membase + UARTCR);
	cr &= ~(LINFLEXD_UARTCR_RXEN | LINFLEXD_UARTCR_TXEN);
	writel(cr, sport->membase + UARTCR);

	/* Enter initialization mode by setting INIT bit */

	/* set the Linflex in master mode and activate by-pass filter */
	cr1 = LINFLEXD_LINCR1_BF | LINFLEXD_LINCR1_MME
	      | LINFLEXD_LINCR1_INIT;
	writel(cr1, sport->membase + LINCR1);

	/* wait for init mode entry */
	while ((readl(sport->membase + LINSR)
		& LINFLEXD_LINSR_LINS_MASK)
		!= LINFLEXD_LINSR_LINS_INITMODE)
		;

	/*
	 *	UART = 0x1;		- Linflex working in UART mode
	 *	TXEN = 0x1;		- Enable transmission of data now
	 *	RXEn = 0x1;		- Receiver enabled
	 *	WL0 = 0x1;		- 8 bit data
	 *	PCE = 0x0;		- No parity
	 */

	/* set UART bit to allow writing other bits */
	writel(LINFLEXD_UARTCR_UART, sport->membase + UARTCR);

	cr = (LINFLEXD_UARTCR_RXEN | LINFLEXD_UARTCR_TXEN |
	      LINFLEXD_UARTCR_WL0 | LINFLEXD_UARTCR_UART);

	/* FIFO mode enabled for DMA Rx mode. */
	if (lfport->dma_rx_use)
		cr |= LINFLEXD_UARTCR_RFBM;

	/* FIFO mode enabled for DMA Tx mode. */
	if (lfport->dma_tx_use)
		cr |= LINFLEXD_UARTCR_TFBM;

	writel(cr, sport->membase + UARTCR);

	cr1 &= ~(LINFLEXD_LINCR1_INIT);

	writel(cr1, sport->membase + LINCR1);

	ier = readl(sport->membase + LINIER);
	if (!lfport->dma_rx_use)
		ier |= LINFLEXD_LINIER_DRIE;

	if (!lfport->dma_tx_use)
		ier |= LINFLEXD_LINIER_DTIE;

	writel(ier, sport->membase + LINIER);
}

static int linflex_dma_tx_request(struct uart_port *port)
{
	struct linflex_port *lfport = to_linflex_port(port);
	struct dma_slave_config dma_tx_sconfig;
	dma_addr_t dma_bus;
	unsigned char *dma_buf;
	int ret;

	dma_bus = dma_map_single(lfport->dma_tx_chan->device->dev,
				 lfport->port.state->xmit.buf,
				 UART_XMIT_SIZE, DMA_TO_DEVICE);

	if (dma_mapping_error(lfport->dma_tx_chan->device->dev, dma_bus)) {
		dev_err(port->dev, "dma_map_single tx failed\n");
		return -ENOMEM;
	}

	dma_buf = port->state->xmit.buf;
	dma_tx_sconfig.dst_addr = port->mapbase + BDRL;
	dma_tx_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_tx_sconfig.dst_maxburst = 1;
	dma_tx_sconfig.direction = DMA_MEM_TO_DEV;
	ret = dmaengine_slave_config(lfport->dma_tx_chan, &dma_tx_sconfig);

	if (ret < 0) {
		dev_err(port->dev, "Dma slave config failed, err = %d\n",
			ret);
		return ret;
	}

	lfport->dma_tx_buf_virt = dma_buf;
	lfport->dma_tx_buf_bus = dma_bus;
	lfport->dma_tx_in_progress = 0;

	return 0;
}

static int linflex_dma_rx_request(struct uart_port *port)
{
	struct linflex_port *lfport = to_linflex_port(port);
	struct dma_slave_config dma_rx_sconfig;
	dma_addr_t dma_bus;
	unsigned char *dma_buf;
	int ret;

	dma_buf = devm_kzalloc(port->dev, FSL_UART_RX_DMA_BUFFER_SIZE,
			       GFP_KERNEL);

	if (!dma_buf) {
		dev_err(port->dev, "Dma rx alloc failed\n");
		return -ENOMEM;
	}

	dma_bus = dma_map_single(lfport->dma_rx_chan->device->dev, dma_buf,
				 FSL_UART_RX_DMA_BUFFER_SIZE, DMA_FROM_DEVICE);

	if (dma_mapping_error(lfport->dma_rx_chan->device->dev, dma_bus)) {
		dev_err(port->dev, "dma_map_single rx failed\n");
		return -ENOMEM;
	}

	dma_rx_sconfig.src_addr = port->mapbase + BDRM;
	dma_rx_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_rx_sconfig.src_maxburst = 1;
	dma_rx_sconfig.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(lfport->dma_rx_chan, &dma_rx_sconfig);

	if (ret < 0) {
		dev_err(port->dev, "Dma slave config failed, err = %d\n",
			ret);
		return ret;
	}

	lfport->dma_rx_buf_virt = dma_buf;
	lfport->dma_rx_buf_bus = dma_bus;
	lfport->dma_rx_in_progress = 0;

	return 0;
}

static void linflex_dma_tx_free(struct uart_port *port)
{
	struct linflex_port *lfport = to_linflex_port(port);

	dma_unmap_single(lfport->port.dev, lfport->dma_tx_buf_bus, UART_XMIT_SIZE,
			 DMA_TO_DEVICE);

	lfport->dma_tx_buf_bus = 0;
	lfport->dma_tx_buf_virt = NULL;
}

static void linflex_dma_rx_free(struct uart_port *port)
{
	struct linflex_port *lfport = to_linflex_port(port);

	dma_unmap_single(lfport->port.dev, lfport->dma_rx_buf_bus,
			 FSL_UART_RX_DMA_BUFFER_SIZE, DMA_FROM_DEVICE);

	lfport->dma_rx_buf_bus = 0;
	lfport->dma_rx_buf_virt = NULL;
}

static int linflex_startup(struct uart_port *port)
{
	struct linflex_port *lfport = to_linflex_port(port);
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	lfport->port.fifosize = LINFLEXD_UARTCR_FIFO_SIZE;

	linflex_setup_watermark(port);

	spin_unlock_irqrestore(&port->lock, flags);

	if (!lfport->dma_rx_use || !lfport->dma_tx_use) {
		ret = devm_request_irq(port->dev, port->irq, linflex_int, 0,
				       DRIVER_NAME, lfport);
	}

	return ret;
}

static void linflex_shutdown(struct uart_port *port)
{
	struct linflex_port *lfport = to_linflex_port(port);
	unsigned long ier;
	unsigned long flags, dmarxe, dmatxe;

	spin_lock_irqsave(&port->lock, flags);

	/* disable Rx/Tx and interrupts */
	ier = readl(port->membase + LINIER);
	ier &= ~(LINFLEXD_LINIER_DRIE | LINFLEXD_LINIER_DTIE);
	writel(ier, port->membase + LINIER);

	spin_unlock_irqrestore(&port->lock, flags);

	if (!lfport->dma_rx_use || !lfport->dma_tx_use)
		devm_free_irq(port->dev, port->irq, lfport);

	if (lfport->dma_rx_use) {
		del_timer(&lfport->timer);
		dmaengine_terminate_all(lfport->dma_rx_chan);

		dmarxe = readl(port->membase + DMARXE);
		writel(dmarxe & 0xFFFF0000, port->membase + DMARXE);

		linflex_dma_rx_free(&lfport->port);
		lfport->dma_rx_in_progress = 0;
	}

	if (lfport->dma_tx_use) {
		dmaengine_terminate_all(lfport->dma_tx_chan);

		dmatxe = readl(port->membase + DMATXE);
		writel(dmatxe & 0xFFFF0000, port->membase + DMATXE);

		linflex_dma_tx_free(&lfport->port);
		lfport->dma_tx_in_progress = 0;
	}
}

static void
linflex_set_termios(struct uart_port *port, struct ktermios *termios,
		    struct ktermios *old)
{
	struct linflex_port *lfport = to_linflex_port(port);
	unsigned long flags;
	unsigned long cr, old_cr, cr1;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	unsigned long baud, ibr, fbr, divisr, dividr;

	spin_lock_irqsave(&port->lock, flags);

	cr = readl(port->membase + UARTCR);
	old_cr = cr;

	/* Enter initialization mode by setting INIT bit */
	cr1 = readl(port->membase + LINCR1);
	cr1 |= LINFLEXD_LINCR1_INIT;
	writel(cr1, port->membase + LINCR1);

	/* wait for init mode entry */
	while ((readl(port->membase + LINSR)
		& LINFLEXD_LINSR_LINS_MASK)
		!= LINFLEXD_LINSR_LINS_INITMODE)
		;

	/*
	 * only support CS8 and CS7, and for CS7 must enable PE.
	 * supported mode:
	 *	- (7,e/o,1)
	 *	- (8,n,1)
	 *	- (8,e/o,1)
	 */
	/* enter the UART into configuration mode */

	while ((termios->c_cflag & CSIZE) != CS8 &&
	       (termios->c_cflag & CSIZE) != CS7) {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= old_csize;
		old_csize = CS8;
	}

	if ((termios->c_cflag & CSIZE) == CS7) {
		/* Word length: WL1WL0:00 */
		cr = old_cr & ~LINFLEXD_UARTCR_WL1 & ~LINFLEXD_UARTCR_WL0;
	}

	if ((termios->c_cflag & CSIZE) == CS8) {
		/* Word length: WL1WL0:01 */
		cr = (old_cr | LINFLEXD_UARTCR_WL0) & ~LINFLEXD_UARTCR_WL1;
	}

	if (termios->c_cflag & CMSPAR) {
		if ((termios->c_cflag & CSIZE) != CS8) {
			termios->c_cflag &= ~CSIZE;
			termios->c_cflag |= CS8;
		}
		/* has a space/sticky bit */
		cr |= LINFLEXD_UARTCR_WL0;
	}

	if (termios->c_cflag & CSTOPB)
		termios->c_cflag &= ~CSTOPB;

	/* parity must be enabled when CS7 to match 8-bits format */
	if ((termios->c_cflag & CSIZE) == CS7)
		termios->c_cflag |= PARENB;

	if ((termios->c_cflag & PARENB)) {
		cr |= LINFLEXD_UARTCR_PCE;
		if (termios->c_cflag & PARODD)
			cr = (cr | LINFLEXD_UARTCR_PC0) &
			     (~LINFLEXD_UARTCR_PC1);
		else
			cr = cr & (~LINFLEXD_UARTCR_PC1 &
				   ~LINFLEXD_UARTCR_PC0);
	} else {
		cr &= ~LINFLEXD_UARTCR_PCE;
	}

	/* ask the core to calculate the divisor */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 16);

	port->read_status_mask = 0;

	if (termios->c_iflag & INPCK)
		port->read_status_mask |=	(LINFLEXD_UARTSR_FEF |
						 LINFLEXD_UARTSR_PE0 |
						 LINFLEXD_UARTSR_PE1 |
						 LINFLEXD_UARTSR_PE2 |
						 LINFLEXD_UARTSR_PE3);
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= LINFLEXD_UARTSR_FEF;

	/* characters to ignore */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= LINFLEXD_UARTSR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= LINFLEXD_UARTSR_PE;
		/*
		 * if we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= LINFLEXD_UARTSR_BOF;
	}

	/* update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);
	lfport->dma_rx_timeout = msecs_to_jiffies(DIV_ROUND_UP(10000000, baud));

	/* disable transmit and receive */
	writel(old_cr & ~(LINFLEXD_UARTCR_RXEN | LINFLEXD_UARTCR_TXEN),
	       port->membase + UARTCR);

	divisr = port->uartclk;	//freq in Hz
	dividr = (baud * 16);

	ibr = divisr / dividr;
	fbr = (divisr % dividr) * 16;

	writel(ibr, port->membase + LINIBRR);
	writel(fbr, port->membase + LINFBRR);

	writel(cr, port->membase + UARTCR);

	cr1 &= ~(LINFLEXD_LINCR1_INIT);

	writel(cr1, port->membase + LINCR1);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *linflex_type(struct uart_port *port)
{
	return "FSL_LINFLEX";
}

static void linflex_release_port(struct uart_port *port)
{
	/* nothing to do */
}

static int linflex_request_port(struct uart_port *port)
{
	return 0;
}

/* configure/auto-configure the port */
static void linflex_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_LINFLEXUART;
}

#ifdef CONFIG_CONSOLE_POLL
static int linflex_poll_init(struct uart_port *port)
{
	struct linflex_port *sport = container_of(port, struct linflex_port,
						  port);
	u32 ier, cr, dmatxe, dmarxe;

	/* Save control and DMA settings and disable interrupts */
	ier = readl(sport->port.membase + LINIER);

	sport->poll_ctx.ier = ier;
	if (!sport->dma_tx_use) {
		ier &= ~(LINFLEXD_LINIER_DTIE);
		writel(ier, sport->port.membase + LINIER);
	} else {
		dmaengine_terminate_all(sport->dma_tx_chan);

		dmatxe = readl(sport->port.membase + DMATXE);
		sport->poll_ctx.dmatxe = dmatxe;
		writel(dmatxe & 0xFFFF0000, sport->port.membase + DMATXE);
	}

	if (!sport->dma_rx_use) {
		ier &= ~(LINFLEXD_LINIER_DRIE);
		writel(ier, sport->port.membase + LINIER);
	} else {
		dmaengine_terminate_all(sport->dma_rx_chan);

		dmarxe = readl(sport->port.membase + DMARXE);
		sport->poll_ctx.dmarxe = dmarxe;
		writel(dmarxe & 0xFFFF0000, sport->port.membase + DMARXE);
	}

	cr = readl(sport->port.membase + UARTCR);
	sport->poll_ctx.cr = cr;

	cr |= (LINFLEXD_UARTCR_TXEN) | (LINFLEXD_UARTCR_RXEN);

	writel(cr, sport->port.membase + UARTCR);

	return 0;
}

static void linflex_poll_release(struct linflex_port *sport)
{
	/* Restore settings from poll context */
	if (!sport->dma_tx_use || !!sport->dma_rx_use)
		writel(sport->poll_ctx.ier, sport->port.membase + LINIER);

	if (sport->dma_tx_use)
		writel(sport->poll_ctx.dmatxe, sport->port.membase + DMATXE);

	if (sport->dma_rx_use)
		writel(sport->poll_ctx.dmarxe, sport->port.membase + DMARXE);

	writel(sport->poll_ctx.cr, sport->port.membase + UARTCR);
}

static void linflex_poll_putchar(struct uart_port *port, unsigned char ch)
{
	int ich = ch;
	unsigned long flags;
	struct linflex_port *sport = container_of(port, struct linflex_port,
						  port);

	spin_lock_irqsave(&sport->port.lock, flags);

	if (!sport->poll_ctx.in_use) {
		sport->poll_ctx.in_use = true;
		linflex_poll_init(port);
	}

	linflex_console_putchar(port, ich);
	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static int linflex_poll_getchar(struct uart_port *port)
{
	int ret = NO_POLL_CHAR;
	unsigned long flags;
	struct linflex_port *sport = container_of(port, struct linflex_port,
						  port);

	spin_lock_irqsave(&sport->port.lock, flags);

	if (!sport->poll_ctx.in_use) {
		sport->poll_ctx.in_use = true;
		linflex_poll_init(port);
	}

	if (!sport->dma_rx_use) {
		/*
		 * Buffer mode: wait until the bytes programmed
		 * in RDFL are received
		 */
		while ((readl(port->membase + UARTSR) &
		       LINFLEXD_UARTSR_DRFRFE)
				!= LINFLEXD_UARTSR_DRFRFE)
			;
	} else {
		/* FIFO mode: Busy waiting while FIFO is empty */
		while ((readl(port->membase + UARTSR) &
			LINFLEXD_UARTSR_DRFRFE))
			;
	}

	spin_unlock_irqrestore(&sport->port.lock, flags);

	writel((readl(port->membase + UARTSR) | LINFLEXD_UARTSR_DRFRFE),
	       port->membase + UARTSR);

	ret = readb(port->membase + BDRM);

	return ret;
}
#endif

static const struct uart_ops linflex_pops = {
	.tx_empty	= linflex_tx_empty,
	.set_mctrl	= linflex_set_mctrl,
	.get_mctrl	= linflex_get_mctrl,
	.stop_tx	= linflex_stop_tx,
	.start_tx	= linflex_start_tx,
	.stop_rx	= linflex_stop_rx,
	.break_ctl	= linflex_break_ctl,
	.startup	= linflex_startup,
	.shutdown	= linflex_shutdown,
	.set_termios	= linflex_set_termios,
	.type		= linflex_type,
	.request_port	= linflex_request_port,
	.release_port	= linflex_release_port,
	.config_port	= linflex_config_port,
	.flush_buffer	= linflex_flush_buffer,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= linflex_poll_getchar,
	.poll_put_char	= linflex_poll_putchar,
#endif
};

static struct uart_port *linflex_ports[UART_NR];

#ifdef CONFIG_SERIAL_FSL_LINFLEXUART_CONSOLE
static void linflex_console_putchar(struct uart_port *port, int ch)
{
	unsigned long cr;

	cr = readl(port->membase + UARTCR);

	writeb(ch, port->membase + BDRL);

	if (!(cr & LINFLEXD_UARTCR_TFBM))
		while ((readl(port->membase + UARTSR) &
					LINFLEXD_UARTSR_DTFTFF)
				!= LINFLEXD_UARTSR_DTFTFF)
			;
	else
		while (readl(port->membase + UARTSR) &
					LINFLEXD_UARTSR_DTFTFF)
			;

	if (!(cr & LINFLEXD_UARTCR_TFBM)) {
		writel((readl(port->membase + UARTSR) |
					LINFLEXD_UARTSR_DTFTFF),
					port->membase + UARTSR);
	}
}

static void linflex_earlycon_putchar(struct uart_port *port, int ch)
{
	unsigned long flags;
	char *ret;

	if (!linflex_earlycon_same_instance) {
		linflex_console_putchar(port, ch);
		return;
	}

	spin_lock_irqsave(&init_lock, flags);
	if (!during_init)
		goto outside_init;
	if (earlycon_buf.len >= 1 << CONFIG_LOG_BUF_SHIFT)
		goto init_release;

	if (!earlycon_buf.cap) {
		earlycon_buf.content = kmalloc(EARLYCON_BUFFER_INITIAL_CAP,
					       GFP_ATOMIC);
		earlycon_buf.cap = earlycon_buf.content ?
				   EARLYCON_BUFFER_INITIAL_CAP : 0;
	} else if (earlycon_buf.len == earlycon_buf.cap) {
		ret = krealloc(earlycon_buf.content, earlycon_buf.cap << 1,
			       GFP_ATOMIC);
		if (ret) {
			earlycon_buf.content = ret;
			earlycon_buf.cap <<= 1;
		}
	}

	if (earlycon_buf.len < earlycon_buf.cap)
		earlycon_buf.content[earlycon_buf.len++] = ch;

	goto init_release;

outside_init:
	linflex_console_putchar(port, ch);
init_release:
	spin_unlock_irqrestore(&init_lock, flags);
}

static void linflex_string_write(struct uart_port *sport, const char *s,
				 unsigned int count)
{
	unsigned long cr, ier = 0;

	ier = readl(sport->membase + LINIER);
	linflex_stop_tx(sport);

	cr = readl(sport->membase + UARTCR);
	cr |= (LINFLEXD_UARTCR_TXEN);
	writel(cr, sport->membase + UARTCR);

	uart_console_write(sport, s, count, linflex_console_putchar);

	writel(ier, sport->membase + LINIER);
}

static void
linflex_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port *sport = linflex_ports[co->index];
	unsigned long flags;
	int locked = 1;

	if (sport->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock_irqsave(&sport->lock, flags);
	else
		spin_lock_irqsave(&sport->lock, flags);

	linflex_string_write(sport, s, count);

	if (locked)
		spin_unlock_irqrestore(&sport->lock, flags);
}

/*
 * if the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init
linflex_console_get_options(struct uart_port *sport, int *parity, int *bits)
{
	unsigned long cr;

	cr = readl(sport->membase + UARTCR);
	cr &= LINFLEXD_UARTCR_RXEN | LINFLEXD_UARTCR_TXEN;

	if (!cr)
		return;

	/* ok, the port was enabled */

	*parity = 'n';
	if (cr & LINFLEXD_UARTCR_PCE) {
		if (cr & LINFLEXD_UARTCR_PC0)
			*parity = 'o';
		else
			*parity = 'e';
	}

	if ((cr & LINFLEXD_UARTCR_WL0) && ((cr & LINFLEXD_UARTCR_WL1) == 0)) {
		if (cr & LINFLEXD_UARTCR_PCE)
			*bits = 9;
		else
			*bits = 8;
	}
}

static int __init linflex_console_setup(struct console *co, char *options)
{
	struct uart_port *sport;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;
	int i;
	unsigned long flags;
	/*
	 * check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index == -1 || co->index >= ARRAY_SIZE(linflex_ports))
		co->index = 0;

	sport = linflex_ports[co->index];
	if (!sport)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		linflex_console_get_options(sport, &parity, &bits);

	if (earlycon_port && sport->mapbase == earlycon_port->mapbase) {
		linflex_earlycon_same_instance = true;

		spin_lock_irqsave(&init_lock, flags);
		during_init = true;
		spin_unlock_irqrestore(&init_lock, flags);

		/* Workaround for character loss or output of many invalid
		 * characters, when INIT mode is entered shortly after a
		 * character has just been printed.
		 */
		udelay(PREINIT_DELAY);
	}

	linflex_setup_watermark(sport);

	ret = uart_set_options(sport, co, baud, parity, bits, flow);

	if (!linflex_earlycon_same_instance)
		goto done;

	spin_lock_irqsave(&init_lock, flags);

	/* Emptying buffer */
	if (earlycon_buf.len) {
		for (i = 0; i < earlycon_buf.len; i++)
			linflex_console_putchar(earlycon_port,
				earlycon_buf.content[i]);

		kfree(earlycon_buf.content);
		earlycon_buf.len = 0;
	}

	during_init = false;
	spin_unlock_irqrestore(&init_lock, flags);

done:
	return ret;
}

static struct uart_driver linflex_reg;
static struct console linflex_console = {
	.name		= DEV_NAME,
	.write		= linflex_console_write,
	.device		= uart_console_device,
	.setup		= linflex_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &linflex_reg,
};

static void linflex_earlycon_write(struct console *con, const char *s,
				   unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, linflex_earlycon_putchar);
}

static int __init linflex_early_console_setup(struct earlycon_device *device,
					      const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = linflex_earlycon_write;
	earlycon_port = &device->port;

	return 0;
}

OF_EARLYCON_DECLARE(linflex, "nxp,s32cc-linflexuart",
		    linflex_early_console_setup);
OF_EARLYCON_DECLARE(linflex, "fsl,s32v234-linflexuart",
		    linflex_early_console_setup);

#define LINFLEX_CONSOLE	(&linflex_console)
#else
#define LINFLEX_CONSOLE	NULL
#endif

static struct uart_driver linflex_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= DEV_NAME,
	.nr		= ARRAY_SIZE(linflex_ports),
	.cons		= LINFLEX_CONSOLE,
};

static int linflex_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct linflex_port *lfport;
	struct uart_port *sport;
	struct resource *res;
	int ret;

	lfport = devm_kzalloc(&pdev->dev, sizeof(*lfport), GFP_KERNEL);
	if (!lfport)
		return -ENOMEM;

	sport = &lfport->port;
	sport->dev = &pdev->dev;

	lfport->dma_tx_chan = dma_request_chan(sport->dev, "tx");
	if (IS_ERR(lfport->dma_tx_chan)) {
		ret = PTR_ERR(lfport->dma_tx_chan);
		if (ret == -EPROBE_DEFER)
			return ret;

		dev_info(sport->dev,
			 "DMA tx channel request failed, operating without tx DMA %ld\n",
			 PTR_ERR(lfport->dma_tx_chan));
		lfport->dma_tx_chan = NULL;
	}

	lfport->dma_rx_chan = dma_request_chan(sport->dev, "rx");
	if (IS_ERR(lfport->dma_rx_chan)) {
		ret = PTR_ERR(lfport->dma_rx_chan);
		if (ret == -EPROBE_DEFER) {
			dma_release_channel(lfport->dma_tx_chan);
			return ret;
		}

		dev_info(sport->dev,
			 "DMA rx channel request failed, operating without rx DMA %ld\n",
			 PTR_ERR(lfport->dma_rx_chan));
		lfport->dma_rx_chan = NULL;
	}

	pdev->dev.coherent_dma_mask = 0;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", ret);
		goto linflex_probe_free_dma;
	}
	if (ret >= UART_NR) {
		dev_err(&pdev->dev, "driver limited to %d serial ports\n",
			UART_NR);
		ret = -ENOMEM;
		goto linflex_probe_free_dma;
	}

	sport->line = ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto linflex_probe_free_dma;
	}

	sport->mapbase = res->start;
	sport->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sport->membase)) {
		ret = PTR_ERR(sport->membase);
		goto linflex_probe_free_dma;
	}

	sport->type = PORT_LINFLEXUART;
	sport->iotype = UPIO_MEM;
	sport->irq = platform_get_irq(pdev, 0);
	sport->ops = &linflex_pops;
	sport->flags = UPF_BOOT_AUTOCONF;
	sport->has_sysrq = IS_ENABLED(CONFIG_SERIAL_FSL_LINFLEXUART_CONSOLE);

	lfport->clk = devm_clk_get(&pdev->dev, "lin");
	if (IS_ERR(lfport->clk)) {
		ret = PTR_ERR(lfport->clk);
		dev_err(&pdev->dev, "failed to get uart clk: %d\n", ret);
		goto linflex_probe_free_dma;
	}

	ret = clk_prepare_enable(lfport->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable uart clk: %d\n", ret);
		goto linflex_probe_free_dma;
	}

	lfport->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(lfport->clk_ipg)) {
		ret = PTR_ERR(lfport->clk_ipg);
		dev_err(&pdev->dev, "failed to get ipg uart clk: %d\n", ret);
		clk_disable_unprepare(lfport->clk);
		goto linflex_probe_free_dma;
	}

	ret = clk_prepare_enable(lfport->clk_ipg);
	if (ret) {
		clk_disable_unprepare(lfport->clk);
		dev_err(&pdev->dev, "failed to enable ipg uart clk: %d\n", ret);
		goto linflex_probe_free_dma;
	}

	sport->uartclk = clk_get_rate(lfport->clk);

	linflex_ports[sport->line] = sport;

	platform_set_drvdata(pdev, sport);

	ret = uart_add_one_port(&linflex_reg, sport);
	if (ret) {
		clk_disable_unprepare(lfport->clk);
		clk_disable_unprepare(lfport->clk_ipg);
		goto linflex_probe_free_dma;
	}

	return 0;

linflex_probe_free_dma:
	if (lfport->dma_tx_chan)
		dma_release_channel(lfport->dma_tx_chan);
	if (lfport->dma_rx_chan)
		dma_release_channel(lfport->dma_rx_chan);

	return ret;
}

static int linflex_remove(struct platform_device *pdev)
{
	struct linflex_port *lfport = platform_get_drvdata(pdev);
	struct uart_port *sport = &lfport->port;

	uart_remove_one_port(&linflex_reg, sport);

	clk_disable_unprepare(lfport->clk);
	clk_disable_unprepare(lfport->clk_ipg);

	if (lfport->dma_tx_chan)
		dma_release_channel(lfport->dma_tx_chan);

	if (lfport->dma_rx_chan)
		dma_release_channel(lfport->dma_rx_chan);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int linflex_suspend(struct device *dev)
{
	struct linflex_port *lfport = dev_get_drvdata(dev);
	struct uart_port *sport = &lfport->port;

	uart_suspend_port(&linflex_reg, sport);

	clk_disable_unprepare(lfport->clk);
	clk_disable_unprepare(lfport->clk_ipg);

	return 0;
}

static int linflex_resume(struct device *dev)
{
	struct linflex_port *lfport = dev_get_drvdata(dev);
	struct uart_port *sport = &lfport->port;
	int ret;

	ret = clk_prepare_enable(lfport->clk);
	if (ret) {
		dev_err(dev, "failed to enable uart clk: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(lfport->clk_ipg);
	if (ret) {
		clk_disable_unprepare(lfport->clk);
		dev_err(dev, "failed to enable ipg uart clk: %d\n", ret);
		return ret;
	}

	uart_resume_port(&linflex_reg, sport);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(linflex_pm_ops, linflex_suspend, linflex_resume);

static struct platform_driver linflex_driver = {
	.probe		= linflex_probe,
	.remove		= linflex_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table	= linflex_dt_ids,
		.pm	= &linflex_pm_ops,
	},
};

static int __init linflex_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&linflex_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&linflex_driver);
	if (ret)
		uart_unregister_driver(&linflex_reg);

	return ret;
}

static void __exit linflex_serial_exit(void)
{
	platform_driver_unregister(&linflex_driver);
	uart_unregister_driver(&linflex_reg);
}

module_init(linflex_serial_init);
module_exit(linflex_serial_exit);

MODULE_DESCRIPTION("Freescale LINFlexD serial port driver");
MODULE_LICENSE("GPL v2");
