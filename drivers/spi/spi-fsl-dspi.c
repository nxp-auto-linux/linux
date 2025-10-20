// SPDX-License-Identifier: GPL-2.0+
//
// Copyright 2013 Freescale Semiconductor, Inc.
// Copyright 2020-2025 NXP
//
// Freescale DSPI driver
// This file contains a driver for the Freescale DSPI

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-fsl-dspi.h>

#define DRIVER_NAME			"fsl-dspi"

#define SPI_MCR				0x00
#define SPI_MCR_HOST			BIT(31)
#define SPI_MCR_MTFE			BIT(26)
#define SPI_MCR_PCSIS(x)		((x) << 16)
#define SPI_MCR_CLR_TXF			BIT(11)
#define SPI_MCR_CLR_RXF			BIT(10)
#define SPI_MCR_XSPI			BIT(3)
#define SPI_MCR_DIS_TXF			BIT(13)
#define SPI_MCR_DIS_RXF			BIT(12)
#define SPI_MCR_HALT			BIT(0)

#define SPI_TCR				0x08
#define SPI_TCR_GET_TCNT(x)		(((x) & GENMASK(31, 16)) >> 16)

#define SPI_CTAR(x)			(0x0c + (((x) & GENMASK(2, 0)) * 4))
#define SPI_CTAR_FMSZ(x)		(((x) << 27) & GENMASK(30, 27))
/* FMSZ is stored in PUSR_CMD regisger for DSPI V2 */
#define SPI_V2_PUSHR_CMD_FMSZ(x)	(((x) & GENMASK(15, 0)))

#define SPI_CTAR_DBR			BIT(31)
#define SPI_CTAR_CPOL			BIT(26)
#define SPI_CTAR_CPHA			BIT(25)
#define SPI_CTAR_LSBFE			BIT(24)
#define SPI_CTAR_PCSSCK(x)		(((x) << 22) & GENMASK(23, 22))
#define SPI_CTAR_PASC(x)		(((x) << 20) & GENMASK(21, 20))
#define SPI_CTAR_PDT(x)			(((x) << 18) & GENMASK(19, 18))
#define SPI_CTAR_PBR(x)			(((x) << 16) & GENMASK(17, 16))
#define SPI_CTAR_CSSCK(x)		(((x) << 12) & GENMASK(15, 12))
#define SPI_CTAR_ASC(x)			(((x) << 8) & GENMASK(11, 8))
#define SPI_CTAR_DT(x)			(((x) << 4) & GENMASK(7, 4))
#define SPI_CTAR_BR(x)			((x) & GENMASK(3, 0))
#define SPI_CTAR_SCALE_BITS		0xf

#define SPI_CTAR0_SLAVE			0x0c

#define SPI_SR			0x2c
#define SPI_V2_SR			0x180
#define SPI_SR_TCFQF			BIT(31)
#define SPI_SR_TFUF			BIT(27)
#define SPI_SR_TFFF			BIT(25)
#define SPI_SR_CMDTCF			BIT(23)
#define SPI_SR_SPEF			BIT(21)
#define SPI_SR_RFOF			BIT(19)
#define SPI_SR_TFIWF			BIT(18)
#define SPI_SR_RFDF			BIT(17)
#define SPI_SR_CMDFFF			BIT(16)
#define SPI_SR_TXRXS			BIT(30)
#define SPI_SR_CLEAR			(SPI_SR_TCFQF | \
					SPI_SR_TFUF | SPI_SR_TFFF | \
					SPI_SR_CMDTCF | SPI_SR_SPEF | \
					SPI_SR_RFOF | SPI_SR_TFIWF | \
					SPI_SR_RFDF | SPI_SR_CMDFFF)
/* FIFO Status (FSR) */
#define SPI_V2_FIFO_SR			0x184
#define SPI_V2_FIFO_SR_TXCTR		(((x) << 13) & GENMASK(17, 13) >> 13)
#define SPI_V2_FIFO_SR_RXCTR		(((x) << 22) & GENMASK(26, 22) >> 22)

/* FIFO Control (FCR) */
#define SPI_V2_FCR			0x3c
#define SPI_V2_FCR_RXWATER		(((x) << 8) & GENMASK(11, 8) >> 8)
#define SPI_V2_FCR_TXWATER		(((x) << 0) & GENMASK(3, 0) >> 0)

/* Reset to Initial State (RTIS) */
#define SPI_RTIS			0x50
/* 0x01 - Graceful reset */
#define RTIS_SRST			BIT(0)

#define SPI_RSER_CMDFFFE		BIT(30)
#define SPI_RSER_TFFFE			BIT(25)
#define SPI_RSER_TFFFD			BIT(24)
#define SPI_RSER_RFDFE			BIT(17)
#define SPI_RSER_RFDFD			BIT(16)
#define SPI_RSER_CMDFFFD		BIT(15)

/* DMA and Interrupt Request Select and Enable */
#define SPI_RSER			0x30
#define SPI_RSER_TCFQE			BIT(31)

#define SPI_V2_RSER			0x2c
#define SPI_RSER_TCFE			BIT(31)
#define SPI_RSER_CMDTCFE		BIT(23)

#define SPI_V2_PUSHR_CMD		0x30

#define SPI_PUSHR			0x34

#define SPI_PUSHR_CMD_CONT		BIT(15)
#define SPI_V2_PUSHR_CMD_CONT		BIT(31)

#define SPI_PUSHR_CMD_CTAS(x)	(((x) << 12 & GENMASK(14, 12)))
#define SPI_V2_PUSHR_CMD_CTAS(x)	(((x) << 28 & GENMASK(30, 28)))

#define SPI_PUSHR_CMD_EOQ		BIT(11)
#define SPI_V2_PUSHR_CMD_EOQ		BIT(27)

#define SPI_PUSHR_CMD_CTCNT		BIT(10)
#define SPI_V2_PUSHR_CMD_CTCNT		BIT(26)

#define SPI_PUSHR_CMD_PCS(x)		(BIT(x) & GENMASK(5, 0))

#define SPI_PUSHR_SLAVE			0x34

#define SPI_POPR			0x38

/* Transmit FIFO TXFR */
#define SPI_TXFR(x)			(0x3c + ((x) * 4))
#define SPI_V2_TXFR(x)		(0x80 + ((x) * 4))

/* Receive FIFO (RXFR) */
#define SPI_RXFR(x)			(0x7c + ((x) * 4))
#define SPI_V2_RXFR(x)		(0xc0 + ((x) * 4))

/* Clock and Transfer Attributes Register Extended */
#define SPI_CTARE(x)			(0x11c + (((x) & GENMASK(2, 0)) * 4))

#define SPI_CTARE_FMSZE(x)		(((x) & 0x1) << 16)
/* Data Transfer Count Preload */
#define SPI_CTARE_DTCP(x)		((x) & 0x7ff)

/* Clock and transfer attributes */
#define DSPI_V2_CTAR(x)			(0x0c + (((x) & GENMASK(2, 0)) * 4))
#define DSPI_V2_CTAR_DBR		BIT(31)
/* Delay After Transfer Scaler */
#define DSPI_V2_CTAR_DT(x)		(0x0c + (((x) & GENMASK(7, 4)) * 4))

/* Status Register Extended (SREX) replaced by FIFO_SR, see DSPI_FSR */
#define SPI_SREX			0x13c
/* FIFO Status (FIFO_SR) */
#define SPI_FRAME_BITS(bits)		SPI_CTAR_FMSZ((bits) - 1)

#define SPI_FRAME_EBITS(bits)	SPI_CTARE_FMSZE(((bits) - 1) >> 4)

#define DMA_COMPLETION_TIMEOUT		msecs_to_jiffies(3000)

#define SPI_25MHZ			25000000

/* The maximum that edma can transfer once.*/
#define SPI_DMA_BUFFER_SIZE	(GENMASK(14, 0) * DMA_SLAVE_BUSWIDTH_4_BYTES)

/* SPI frames with sizes up to 65536 bits can have a single command
 * associated with them.
 */
#define SPI_V2_MAX_FRAME_PER_CMD	65536
#define SPI_V2_MAX_WORDS_PER_CMD	(SPI_V2_MAX_FRAME_PER_CMD / 32)

struct chip_data {
	u32			ctar_val;
};

enum dspi_hw_ver {
	DSPI_V1,
	DSPI_V2,
};

enum dspi_trans_mode {
	DSPI_XSPI_MODE,
	DSPI_DMA_MODE,
};

struct fsl_dspi_devtype_data {
	enum dspi_trans_mode	trans_mode;
	u8			max_clock_factor;
	int			fifo_size;
	enum dspi_hw_ver	hw_version;
};

enum {
	LS1021A,
	LS1012A,
	LS1028A,
	LS1043A,
	LS1046A,
	LS2080A,
	LS2085A,
	LX2160A,
	MCF5441X,
	VF610,
	S32CC,
	S32CC_TARGET,
	S32N79,
	S32N79_TARGET,
};

static inline u32 dspi_get_sr(enum dspi_hw_ver ver)
{
	return ver == DSPI_V1 ? SPI_SR : SPI_V2_SR;
}

static inline u32 dspi_get_rser(enum dspi_hw_ver ver)
{
	return ver == DSPI_V1 ? SPI_RSER : SPI_V2_RSER;
}

static inline u32 dspi_pushr_cmd_cont_mask(enum dspi_hw_ver ver)
{
	return ver == DSPI_V1 ? SPI_PUSHR_CMD_CONT : SPI_V2_PUSHR_CMD_CONT;
}

static inline u32 dspi_pushr_cmd_eoq_mask(enum dspi_hw_ver ver)
{
	return ver == DSPI_V1 ? SPI_PUSHR_CMD_EOQ : SPI_V2_PUSHR_CMD_EOQ;
}

static inline u32 dspi_pushr_cmd_ctas_mask(enum dspi_hw_ver ver, uint ctar)
{
	return ver == DSPI_V1 ? SPI_PUSHR_CMD_CTAS(ctar) : SPI_V2_PUSHR_CMD_CTAS(ctar);
}

static const struct fsl_dspi_devtype_data devtype_data[] = {
	[VF610] = {
		.trans_mode		= DSPI_DMA_MODE,
		.max_clock_factor	= 2,
		.fifo_size		= 4,
		.hw_version		= DSPI_V1,
	},
	[LS1021A] = {
		/* Has A-011218 DMA erratum */
		.trans_mode		= DSPI_XSPI_MODE,
		.max_clock_factor	= 8,
		.fifo_size		= 4,
		.hw_version		= DSPI_V1,
	},
	[LS1012A] = {
		/* Has A-011218 DMA erratum */
		.trans_mode		= DSPI_XSPI_MODE,
		.max_clock_factor	= 8,
		.fifo_size		= 16,
		.hw_version		= DSPI_V1,
	},
	[LS1028A] = {
		.trans_mode		= DSPI_XSPI_MODE,
		.max_clock_factor	= 8,
		.fifo_size		= 4,
		.hw_version		= DSPI_V1,
	},
	[LS1043A] = {
		/* Has A-011218 DMA erratum */
		.trans_mode		= DSPI_XSPI_MODE,
		.max_clock_factor	= 8,
		.fifo_size		= 16,
		.hw_version		= DSPI_V1,
	},
	[LS1046A] = {
		/* Has A-011218 DMA erratum */
		.trans_mode		= DSPI_XSPI_MODE,
		.max_clock_factor	= 8,
		.fifo_size		= 16,
		.hw_version		= DSPI_V1,
	},
	[LS2080A] = {
		.trans_mode		= DSPI_XSPI_MODE,
		.max_clock_factor	= 8,
		.fifo_size		= 4,
		.hw_version		= DSPI_V1,
	},
	[LS2085A] = {
		.trans_mode		= DSPI_XSPI_MODE,
		.max_clock_factor	= 8,
		.fifo_size		= 4,
		.hw_version		= DSPI_V1,
	},
	[LX2160A] = {
		.trans_mode		= DSPI_XSPI_MODE,
		.max_clock_factor	= 8,
		.fifo_size		= 4,
		.hw_version		= DSPI_V1,
	},
	[MCF5441X] = {
		.trans_mode		= DSPI_DMA_MODE,
		.max_clock_factor	= 8,
		.fifo_size		= 16,
		.hw_version		= DSPI_V1,
	},
	[S32CC] = {
		.trans_mode		= DSPI_XSPI_MODE,
		.max_clock_factor	= 1,
		.fifo_size		= 5,
		.hw_version		= DSPI_V1,
	},
	[S32CC_TARGET] = {
		.trans_mode		= DSPI_DMA_MODE,
		.max_clock_factor	= 1,
		.fifo_size		= 5,
		.hw_version		= DSPI_V1,
	},
	[S32N79] = {
		.trans_mode		= DSPI_XSPI_MODE,
		.max_clock_factor	= 1,
		.fifo_size		= 16,
		.hw_version		= DSPI_V2,
	},
	[S32N79_TARGET] = {
		.trans_mode		= DSPI_DMA_MODE,
		.max_clock_factor	= 1,
		.fifo_size		= 16,
		.hw_version		= DSPI_V2,
	},
};

struct fsl_dspi_dma {
	/* TX FIFO DMA resources. */
	u32					*tx_dma_buf;
	struct dma_chan				*chan_tx;
	dma_addr_t				tx_dma_phys;
	struct completion			cmd_tx_complete;
	struct dma_async_tx_descriptor		*tx_desc;

	/* RX FIFO DMA resources. */
	u32					*rx_dma_buf;
	struct dma_chan				*chan_rx;
	dma_addr_t				rx_dma_phys;
	struct completion			cmd_rx_complete;
	struct dma_async_tx_descriptor		*rx_desc;
	unsigned int				dma_bufsize;

	/* CMD FIFO DMA resources. Used in DSPI_V2 case. */
	u32					*cmd_dma_buf;
	struct dma_chan				*chan_cmd;
	dma_addr_t				cmd_dma_phys;
	struct dma_async_tx_descriptor		*cmd_desc;
};

struct fsl_dspi {
	struct spi_controller			*ctlr;
	struct platform_device			*pdev;

	struct regmap				*regmap;
	/* SPI V1 combines 16bit command 16bit TX data into the same 32bit register.
	 * SPI V2 has separate 32bit PUSHR and PUSHR_CMD registers.
	 */
	struct regmap				*regmap_pushr;
	int						irq;
	struct clk				*clk;
	bool					mtf_enabled;
	struct spi_transfer			*cur_transfer;
	struct spi_message			*cur_msg;
	struct chip_data			*cur_chip;
	size_t					progress;
	size_t					len;
	const void				*tx;
	void					*rx;
	u16					tx_cmd;
	const struct fsl_dspi_devtype_data	*devtype_data;

	struct completion			xfer_done;

	struct fsl_dspi_dma			*dma;

	int					oper_word_size;
	int					oper_bits_per_word;

	int					words_in_flight;

	/*
	 * Offsets for CMD and TXDATA within SPI_PUSHR when accessed
	 * individually (in XSPI mode)
	 * Note: this only applies to V1 hw_version.
	 */
	int					pushr_cmd;
	int					pushr_tx;

	void (*host_to_dev)(struct fsl_dspi *dspi, u32 *txdata);
	void (*dev_to_host)(struct fsl_dspi *dspi, u32 rxdata);
};

static int dspi_init(struct fsl_dspi *dspi);

static inline bool is_s32cc_dspi(struct fsl_dspi *data)
{
	return data->devtype_data == &devtype_data[S32CC] ||
		data->devtype_data == &devtype_data[S32CC_TARGET];
}

static inline bool is_s32n_dspi(struct fsl_dspi *data)
{
	return data->devtype_data == &devtype_data[S32N79] ||
		data->devtype_data == &devtype_data[S32N79_TARGET];
}

static void dspi_native_host_to_dev(struct fsl_dspi *dspi, u32 *txdata)
{
	switch (dspi->oper_word_size) {
	case 1:
		*txdata = *(u8 *)dspi->tx;
		break;
	case 2:
		*txdata = *(u16 *)dspi->tx;
		break;
	case 4:
		*txdata = *(u32 *)dspi->tx;
		break;
	}
	dspi->tx += dspi->oper_word_size;
}

static void dspi_native_dev_to_host(struct fsl_dspi *dspi, u32 rxdata)
{
	switch (dspi->oper_word_size) {
	case 1:
		*(u8 *)dspi->rx = rxdata;
		break;
	case 2:
		*(u16 *)dspi->rx = rxdata;
		break;
	case 4:
		*(u32 *)dspi->rx = rxdata;
		break;
	}
	dspi->rx += dspi->oper_word_size;
}

static void dspi_8on32_host_to_dev(struct fsl_dspi *dspi, u32 *txdata)
{
	*txdata = cpu_to_be32(*(u32 *)dspi->tx);
	dspi->tx += sizeof(u32);
}

static void dspi_8on32_dev_to_host(struct fsl_dspi *dspi, u32 rxdata)
{
	*(u32 *)dspi->rx = be32_to_cpu(rxdata);
	dspi->rx += sizeof(u32);
}

static void dspi_8on16_host_to_dev(struct fsl_dspi *dspi, u32 *txdata)
{
	*txdata = cpu_to_be16(*(u16 *)dspi->tx);
	dspi->tx += sizeof(u16);
}

static void dspi_8on16_dev_to_host(struct fsl_dspi *dspi, u32 rxdata)
{
	*(u16 *)dspi->rx = be16_to_cpu(rxdata);
	dspi->rx += sizeof(u16);
}

static void dspi_16on32_host_to_dev(struct fsl_dspi *dspi, u32 *txdata)
{
	u16 hi = *(u16 *)dspi->tx;
	u16 lo = *(u16 *)(dspi->tx + 2);

	*txdata = (u32)hi << 16 | lo;
	dspi->tx += sizeof(u32);
}

static void dspi_16on32_dev_to_host(struct fsl_dspi *dspi, u32 rxdata)
{
	u16 hi = rxdata & 0xffff;
	u16 lo = rxdata >> 16;

	*(u16 *)dspi->rx = lo;
	*(u16 *)(dspi->rx + 2) = hi;
	dspi->rx += sizeof(u32);
}

/*
 * Pop one word from the TX buffer for pushing into the
 * PUSHR register (TX FIFO)
 */
static u32 dspi_pop_tx(struct fsl_dspi *dspi)
{
	u32 txdata = 0;

	if (dspi->tx)
		dspi->host_to_dev(dspi, &txdata);
	dspi->len -= dspi->oper_word_size;
	return txdata;
}

/* Prepare one TX FIFO entry:
 * - 16 bit txdata plus 16 bit cmd for SPI V1 hw version
 * - 32 bit txdata for SPI V2 hw version
 */
static u32 dspi_pop_tx_pushr(struct fsl_dspi *dspi)
{
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;
	u32 data = dspi_pop_tx(dspi);
	u16 cmd = dspi->tx_cmd;

	if (spi_controller_is_target(dspi->ctlr))
		return data;

	if (dspi->len > 0)
		cmd |= dspi_pushr_cmd_cont_mask(hw_ver);

	if (hw_ver == DSPI_V1)
		return cmd << 16 | data;
	else
		return data;
}

/* Push one word to the RX buffer from the POPR register (RX FIFO) */
static void dspi_push_rx(struct fsl_dspi *dspi, u32 rxdata)
{
	if (!dspi->rx)
		return;
	dspi->dev_to_host(dspi, rxdata);
}

static void dspi_tx_dma_callback(void *arg)
{
	struct fsl_dspi *dspi = arg;
	struct fsl_dspi_dma *dma = dspi->dma;

	complete(&dma->cmd_tx_complete);
}

static void dspi_rx_dma_callback(void *arg)
{
	struct fsl_dspi *dspi = arg;
	struct fsl_dspi_dma *dma = dspi->dma;
	int i;

	if (dspi->rx) {
		for (i = 0; i < dspi->words_in_flight; i++)
			dspi_push_rx(dspi, dspi->dma->rx_dma_buf[i]);
	}

	complete(&dma->cmd_rx_complete);
}

static u32 dspi_get_cmd_fifo(struct fsl_dspi *dspi, int num_words);
static void dspi_pushr_cmd_write(struct fsl_dspi *dspi, u32 cmd);

static inline bool dspi_use_single_cmd(struct fsl_dspi *dspi)
{
	return dspi->oper_bits_per_word == 32;
}

static int dspi_next_xfer_dma_submit(struct fsl_dspi *dspi)
{
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;
	struct device *dev = &dspi->pdev->dev;
	struct fsl_dspi_dma *dma = dspi->dma;
	int time_left, num_words = dspi->words_in_flight;
	bool use_dma_for_cmd = false;
	int i;

	/* DSPI_V1 adds the command during dspi_pop_tx_pushr call.
	 * DSPI_V2 will use only one command if 32bits words are used,
	 * otherwise, will add a command for each word. In the later case,
	 * dma will be used to feed the CMD FIFO.
	 */
	if (hw_ver != DSPI_V1) {
		use_dma_for_cmd = !dspi_use_single_cmd(dspi);

		/* Write the command if no DMA is used for it. */
		if (!use_dma_for_cmd)
			dspi_pushr_cmd_write(dspi,
					     dspi_get_cmd_fifo(dspi, num_words));
	}

	for (i = 0; i < num_words; i++) {
		dspi->dma->tx_dma_buf[i] = dspi_pop_tx_pushr(dspi);
		if (use_dma_for_cmd)
			dspi->dma->cmd_dma_buf[i] =
				dspi_get_cmd_fifo(dspi, num_words);
	}

	if (use_dma_for_cmd) {
		dma->cmd_desc = dmaengine_prep_slave_single(dma->chan_cmd,
						dma->cmd_dma_phys,
						dspi->words_in_flight *
						DMA_SLAVE_BUSWIDTH_4_BYTES,
						DMA_MEM_TO_DEV,
						DMA_PREP_INTERRUPT |
						DMA_CTRL_ACK);
		if (!dma->cmd_desc) {
			dev_err(dev, "Not able to get desc for DMA xfer cmd\n");
			return -EIO;
		}

		if (dma_submit_error(dmaengine_submit(dma->cmd_desc))) {
			dev_err(dev, "DMA submit failed\n");
			return -EINVAL;
		}
	}

	dma->tx_desc = dmaengine_prep_slave_single(dma->chan_tx,
					dma->tx_dma_phys,
					dspi->words_in_flight *
					DMA_SLAVE_BUSWIDTH_4_BYTES,
					DMA_MEM_TO_DEV,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!dma->tx_desc) {
		dev_err(dev, "Not able to get desc for DMA xfer\n");
		return -EIO;
	}

	dma->tx_desc->callback = dspi_tx_dma_callback;
	dma->tx_desc->callback_param = dspi;
	if (dma_submit_error(dmaengine_submit(dma->tx_desc))) {
		dev_err(dev, "DMA submit failed\n");
		return -EINVAL;
	}

	dma->rx_desc = dmaengine_prep_slave_single(dma->chan_rx,
					dma->rx_dma_phys,
					dspi->words_in_flight *
					DMA_SLAVE_BUSWIDTH_4_BYTES,
					DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!dma->rx_desc) {
		dev_err(dev, "Not able to get desc for DMA xfer\n");
		return -EIO;
	}

	dma->rx_desc->callback = dspi_rx_dma_callback;
	dma->rx_desc->callback_param = dspi;
	if (dma_submit_error(dmaengine_submit(dma->rx_desc))) {
		dev_err(dev, "DMA submit failed\n");
		return -EINVAL;
	}

	reinit_completion(&dspi->dma->cmd_rx_complete);
	reinit_completion(&dspi->dma->cmd_tx_complete);

	dma_async_issue_pending(dma->chan_rx);
	/* CMD FIFO should be populated before the TX FIFO. */
	if (use_dma_for_cmd)
		dma_async_issue_pending(dma->chan_cmd);
	dma_async_issue_pending(dma->chan_tx);

	if (spi_controller_is_target(dspi->ctlr)) {
		wait_for_completion_interruptible(&dspi->dma->cmd_rx_complete);
		return 0;
	}

	time_left = wait_for_completion_timeout(&dspi->dma->cmd_tx_complete,
						DMA_COMPLETION_TIMEOUT);
	if (time_left == 0) {
		dev_err(dev, "DMA tx timeout\n");
		dmaengine_terminate_all(dma->chan_tx);
		dmaengine_terminate_all(dma->chan_rx);
		if (use_dma_for_cmd)
			dmaengine_terminate_all(dma->chan_cmd);
		return -ETIMEDOUT;
	}

	time_left = wait_for_completion_timeout(&dspi->dma->cmd_rx_complete,
						DMA_COMPLETION_TIMEOUT);
	if (time_left == 0) {
		dev_err(dev, "DMA rx timeout\n");
		dmaengine_terminate_all(dma->chan_tx);
		dmaengine_terminate_all(dma->chan_rx);
		if (use_dma_for_cmd)
			dmaengine_terminate_all(dma->chan_cmd);
		return -ETIMEDOUT;
	}

	return 0;
}

static void dspi_setup_accel(struct fsl_dspi *dspi);

static int dspi_dma_xfer(struct fsl_dspi *dspi)
{
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;
	struct spi_message *message = dspi->cur_msg;
	struct device *dev = &dspi->pdev->dev;
	int max_words_per_buffer = dspi->dma->dma_bufsize /
				   DMA_SLAVE_BUSWIDTH_4_BYTES;
	int ret = 0;

	if (spi_controller_is_target(dspi->ctlr) &&
			dspi->len / dspi->oper_word_size > max_words_per_buffer)
		dev_warn(dev, "Current message has more than %d words. Underrun may occur.\n",
				max_words_per_buffer);
	/*
	 * dspi->len gets decremented by dspi_pop_tx_pushr in
	 * dspi_next_xfer_dma_submit
	 */
	while (dspi->len) {
		/* Figure out operational bits-per-word for this chunk */
		dspi_setup_accel(dspi);
		dspi->words_in_flight = dspi->len / dspi->oper_word_size;

		/* In V2 case, PUSHR_CMD[FMSZ] allows to send up to 65536
		 * bits using a single command frame.
		 */
		if ((hw_ver != DSPI_V1) && dspi_use_single_cmd(dspi) &&
			(dspi->words_in_flight > SPI_V2_MAX_WORDS_PER_CMD))
			dspi->words_in_flight = SPI_V2_MAX_WORDS_PER_CMD;

		if (dspi->words_in_flight > max_words_per_buffer)
			dspi->words_in_flight = max_words_per_buffer;

		message->actual_length += dspi->words_in_flight *
					  dspi->oper_word_size;

		ret = dspi_next_xfer_dma_submit(dspi);
		if (ret) {
			dev_err(dev, "DMA transfer failed\n");
			break;
		}
	}

	return ret;
}

static int dspi_request_dma(struct fsl_dspi *dspi, phys_addr_t phy_addr)
{
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;
	unsigned int dma_bufsize = SPI_DMA_BUFFER_SIZE;
	struct device *dev = &dspi->pdev->dev;
	struct dma_slave_config cfg;
	struct fsl_dspi_dma *dma;
	int ret;

	dma = devm_kzalloc(dev, sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return -ENOMEM;

	dma->chan_rx = dma_request_chan(dev, "rx");
	if (IS_ERR(dma->chan_rx))
		return dev_err_probe(dev, PTR_ERR(dma->chan_rx), "rx dma channel not available\n");

	if (hw_ver != DSPI_V1) {
		dma->chan_cmd = dma_request_chan(dev, "cmd");
		if (IS_ERR(dma->chan_cmd)) {
			ret = dev_err_probe(dev, PTR_ERR(dma->chan_cmd),
					    "cmd dma channel not available\n");
			goto err_cmd_channel;
		}
	}

	dma->chan_tx = dma_request_chan(dev, "tx");
	if (IS_ERR(dma->chan_tx)) {
		ret = dev_err_probe(dev, PTR_ERR(dma->chan_tx), "tx dma channel not available\n");
		goto err_tx_channel;
	}

	dma->tx_dma_buf = dma_alloc_coherent(dma->chan_tx->device->dev,
					     dma_bufsize, &dma->tx_dma_phys,
					     GFP_KERNEL);
	if (!dma->tx_dma_buf) {
		ret = -ENOMEM;
		goto err_tx_dma_buf;
	}

	dma->rx_dma_buf = dma_alloc_coherent(dma->chan_rx->device->dev,
					     dma_bufsize, &dma->rx_dma_phys,
					     GFP_KERNEL);
	if (!dma->rx_dma_buf) {
		ret = -ENOMEM;
		goto err_rx_dma_buf;
	}

	if (dma->chan_cmd) {
		dma->cmd_dma_buf = dma_alloc_coherent(dma->chan_cmd->device->dev,
						      dma_bufsize,
						      &dma->cmd_dma_phys,
						      GFP_KERNEL);
		if (!dma->cmd_dma_buf) {
			ret = -ENOMEM;
			goto err_cmd_dma_buf;
		}
	}

	memset(&cfg, 0, sizeof(cfg));
	cfg.src_addr = phy_addr + SPI_POPR;
	cfg.dst_addr = phy_addr + SPI_PUSHR;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.src_maxburst = 1;
	cfg.dst_maxburst = 1;

	cfg.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(dma->chan_rx, &cfg);
	if (ret) {
		dev_err_probe(dev, ret, "can't configure rx dma channel\n");
		goto err_slave_config;
	}

	cfg.direction = DMA_MEM_TO_DEV;
	ret = dmaengine_slave_config(dma->chan_tx, &cfg);
	if (ret) {
		dev_err_probe(dev, ret, "can't configure tx dma channel\n");
		goto err_slave_config;
	}

	if (dma->chan_cmd) {
		cfg.dst_addr = phy_addr + SPI_V2_PUSHR_CMD;
		ret = dmaengine_slave_config(dma->chan_cmd, &cfg);
		if (ret) {
			dev_err_probe(dev, ret,
				      "can't configure cmd dma channel\n");
			goto err_slave_config;
		}
	}

	dma->dma_bufsize = dma_bufsize;
	dspi->dma = dma;
	init_completion(&dma->cmd_tx_complete);
	init_completion(&dma->cmd_rx_complete);

	return 0;

err_slave_config:
	if (dma->chan_cmd)
		dma_free_coherent(dma->chan_cmd->device->dev,
				  dma_bufsize, dma->cmd_dma_buf,
				  dma->cmd_dma_phys);
err_cmd_dma_buf:
	dma_free_coherent(dma->chan_rx->device->dev,
			  dma_bufsize, dma->rx_dma_buf, dma->rx_dma_phys);
err_rx_dma_buf:
	dma_free_coherent(dma->chan_tx->device->dev,
			  dma_bufsize, dma->tx_dma_buf, dma->tx_dma_phys);
err_tx_dma_buf:
	dma_release_channel(dma->chan_tx);
err_tx_channel:
	if (dma->chan_cmd)
		dma_release_channel(dma->chan_cmd);
err_cmd_channel:
	dma_release_channel(dma->chan_rx);

	devm_kfree(dev, dma);
	dspi->dma = NULL;

	return ret;
}

static void dspi_release_dma(struct fsl_dspi *dspi)
{
	struct fsl_dspi_dma *dma = dspi->dma;

	if (!dma)
		return;

	if (dma->chan_tx) {
		dma_free_coherent(dma->chan_tx->device->dev, dma->dma_bufsize,
				  dma->tx_dma_buf, dma->tx_dma_phys);
		dma_release_channel(dma->chan_tx);
	}

	if (dma->chan_rx) {
		dma_free_coherent(dma->chan_rx->device->dev, dma->dma_bufsize,
				  dma->rx_dma_buf, dma->rx_dma_phys);
		dma_release_channel(dma->chan_rx);
	}
}

static void hz_to_spi_baud(char *pbr, char *br, int speed_hz,
			   unsigned long clkrate, bool mtf_enabled)
{
	/* Valid baud rate pre-scaler values */
	int pbr_tbl[4] = {2, 3, 5, 7};
	int brs[16] = {	2,	4,	6,	8,
			16,	32,	64,	128,
			256,	512,	1024,	2048,
			4096,	8192,	16384,	32768 };
	int scale_needed, scale, minscale = INT_MAX;
	int i, j, dbr = 1;

	scale_needed = clkrate / speed_hz;
	if (clkrate % speed_hz)
		scale_needed++;

	for (i = 0; i < ARRAY_SIZE(brs); i++)
		for (j = 0; j < ARRAY_SIZE(pbr_tbl); j++) {
			if (!mtf_enabled)
				scale = brs[i] * pbr_tbl[j];
			else
				scale = (brs[i] * pbr_tbl[j]) / (1 + dbr);

			if (scale >= scale_needed) {
				if (scale < minscale) {
					minscale = scale;
					*br = i;
					*pbr = j;
				}
				break;
			}
		}

	if (minscale == INT_MAX) {
		pr_warn("Can not find valid baud rate,speed_hz is %d,clkrate is %ld, we use the max prescaler value.\n",
			speed_hz, clkrate);
		*pbr = ARRAY_SIZE(pbr_tbl) - 1;
		*br =  ARRAY_SIZE(brs) - 1;
	}
}

static void ns_delay_scale(char *psc, char *sc, int delay_ns,
			   unsigned long clkrate)
{
	int scale_needed, scale, minscale = INT_MAX;
	int pscale_tbl[4] = {1, 3, 5, 7};
	u32 remainder;
	int i, j;

	scale_needed = div_u64_rem((u64)delay_ns * clkrate, NSEC_PER_SEC,
				   &remainder);
	if (remainder)
		scale_needed++;

	for (i = 0; i < ARRAY_SIZE(pscale_tbl); i++)
		for (j = 0; j <= SPI_CTAR_SCALE_BITS; j++) {
			scale = pscale_tbl[i] * (2 << j);
			if (scale >= scale_needed) {
				if (scale < minscale) {
					minscale = scale;
					*psc = i;
					*sc = j;
				}
				break;
			}
		}

	if (minscale == INT_MAX) {
		pr_warn("Cannot find correct scale values for %dns delay at clkrate %ld, using max prescaler value",
			delay_ns, clkrate);
		*psc = ARRAY_SIZE(pscale_tbl) - 1;
		*sc = SPI_CTAR_SCALE_BITS;
	}
}

static void dspi_pushr_cmd_write(struct fsl_dspi *dspi, u32 cmd)
{
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;

	/*
	 * The only time when the PCS doesn't need continuation after this word
	 * is when it's last. We need to look ahead, because we actually call
	 * dspi_pop_tx (the function that decrements dspi->len) _after_
	 * dspi_pushr_cmd_write with XSPI mode. As for how much in advance? One
	 * word is enough. If there's more to transmit than that,
	 * dspi_xspi_write will know to split the FIFO writes in 2, and
	 * generate a new PUSHR command with the final word that will have PCS
	 * deasserted (not continued) here.
	 */
	if (dspi->len > dspi->oper_word_size)
		cmd |= dspi_pushr_cmd_cont_mask(hw_ver);
	if (hw_ver == DSPI_V1)
		regmap_write(dspi->regmap_pushr, dspi->pushr_cmd, (u16)cmd);
	else
		regmap_write(dspi->regmap, SPI_V2_PUSHR_CMD, cmd);
}

static void dspi_pushr_txdata_write(struct fsl_dspi *dspi, u32 txdata)
{
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;

	if (hw_ver == DSPI_V1)
		regmap_write(dspi->regmap_pushr, dspi->pushr_tx, txdata);
	else
		regmap_write(dspi->regmap, SPI_PUSHR, txdata);
}

static u32 dspi_get_cmd_fifo(struct fsl_dspi *dspi, int num_words)
{
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;
	int num_bytes = num_words * dspi->oper_word_size;
	u32 fmsz, tx_cmd = dspi->tx_cmd;

	if (hw_ver == DSPI_V1 && spi_controller_is_target(dspi->ctlr))
		return tx_cmd;
	/*
	 * If the PCS needs to de-assert (i.e. we're at the end of the buffer
	 * and cs_change does not want the PCS to stay on), then we need a new
	 * PUSHR command, since this one (for the body of the buffer)
	 * necessarily has the CONT bit set.
	 * So send one word less during this go, to force a split and a command
	 * with a single word next time, when CONT will be unset.
	 */
	if (!spi_controller_is_target(dspi->ctlr) &&
	    !(dspi->tx_cmd & dspi_pushr_cmd_cont_mask(hw_ver)) && num_bytes == dspi->len)
		tx_cmd |= dspi_pushr_cmd_eoq_mask(hw_ver);

	/* Update frame size for V2. */
	if (hw_ver != DSPI_V1) {
		fmsz = SPI_V2_PUSHR_CMD_FMSZ(dspi->oper_bits_per_word *
				num_words - 1);
		if (spi_controller_is_target(dspi->ctlr))
			tx_cmd = fmsz;
		else
			tx_cmd  = tx_cmd << 16 | fmsz;
	}

	return tx_cmd;
}

static void dspi_xspi_fifo_write(struct fsl_dspi *dspi, int num_words)
{
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;

	/* Update frame size for V1. */
	if (hw_ver == DSPI_V1)
		regmap_write(dspi->regmap, SPI_CTARE(0),
			     SPI_FRAME_EBITS(dspi->oper_bits_per_word) |
			     SPI_CTARE_DTCP(num_words));

	/*
	 * Write the CMD FIFO entry first, and then the two
	 * corresponding TX FIFO entries (or one...).
	 */
	dspi_pushr_cmd_write(dspi, dspi_get_cmd_fifo(dspi, num_words));

	/* Fill TX FIFO with as many transfers as possible */
	while (num_words--) {
		u32 data = dspi_pop_tx(dspi);

		if (hw_ver == DSPI_V1) {
			dspi_pushr_txdata_write(dspi, data & 0xFFFF);
			if (dspi->oper_bits_per_word > 16)
				dspi_pushr_txdata_write(dspi, data >> 16);
		} else {
			dspi_pushr_txdata_write(dspi, data);
		}
	}
}

static u32 dspi_popr_read(struct fsl_dspi *dspi)
{
	u32 rxdata = 0;

	regmap_read(dspi->regmap, SPI_POPR, &rxdata);
	return rxdata;
}

static void dspi_fifo_read(struct fsl_dspi *dspi)
{
	int num_fifo_entries = dspi->words_in_flight;

	/* Read one FIFO entry and push to rx buffer */
	while (num_fifo_entries--)
		dspi_push_rx(dspi, dspi_popr_read(dspi));
}

static inline void dspi_setup_accel_v1(struct fsl_dspi *dspi)
{
	bool odd = !!(dspi->len & 1);

	if (!odd && dspi->len <= dspi->devtype_data->fifo_size * 2) {
		dspi->oper_bits_per_word = 16;
	} else if (odd && dspi->len <= dspi->devtype_data->fifo_size) {
		dspi->oper_bits_per_word = 8;
	} else {
		/* Start off with maximum supported by hardware */
		if (dspi->devtype_data->trans_mode == DSPI_XSPI_MODE)
			dspi->oper_bits_per_word = 32;
		else
			dspi->oper_bits_per_word = 16;

		/*
		 * And go down only if the buffer can't be sent with
		 * words this big
		 */
		do {
			if (dspi->len >= DIV_ROUND_UP(dspi->oper_bits_per_word, 8))
				break;

			dspi->oper_bits_per_word /= 2;
		} while (dspi->oper_bits_per_word > 8);
	}
}

static inline void dspi_setup_accel_v2(struct fsl_dspi *dspi)
{
	struct spi_transfer *xfer = dspi->cur_transfer;
	size_t total_len = dspi->len * xfer->bits_per_word;

	/* A single CMD FIFO entry can be used to TX multiple TX FIFO entries.
	 * PUSHR_CMD[FMSZ] decides the number of SPI data frames to be
	 * transmitted using the current command entry.
	 * Since a TX entry occupies 32bits, send data as 32bits to reuse the
	 * same command.
	 */
	if (total_len >= 32) {
		dspi->oper_bits_per_word = 32;
	} else if (total_len >= 16) {
		dspi->oper_bits_per_word = 16;
	} else {
		dspi->oper_bits_per_word = 8;
	}
}

static void dspi_setup_accel(struct fsl_dspi *dspi)
{
	struct spi_transfer *xfer = dspi->cur_transfer;

	/* No accel for DMA transfers or for frames not multiple of 8 bits
	 * at the moment
	 */
	if (dspi->devtype_data->trans_mode == DSPI_DMA_MODE ||
			xfer->bits_per_word % 8)
		goto no_accel;

	if (dspi->devtype_data->hw_version == DSPI_V1)
		dspi_setup_accel_v1(dspi);
	else
		dspi_setup_accel_v2(dspi);

	if (xfer->bits_per_word == 8 && dspi->oper_bits_per_word == 32) {
		dspi->dev_to_host = dspi_8on32_dev_to_host;
		dspi->host_to_dev = dspi_8on32_host_to_dev;
	} else if (xfer->bits_per_word == 8 && dspi->oper_bits_per_word == 16) {
		dspi->dev_to_host = dspi_8on16_dev_to_host;
		dspi->host_to_dev = dspi_8on16_host_to_dev;
	} else if (xfer->bits_per_word == 16 && dspi->oper_bits_per_word == 32) {
		dspi->dev_to_host = dspi_16on32_dev_to_host;
		dspi->host_to_dev = dspi_16on32_host_to_dev;
	} else {
no_accel:
		dspi->dev_to_host = dspi_native_dev_to_host;
		dspi->host_to_dev = dspi_native_host_to_dev;
		dspi->oper_bits_per_word = xfer->bits_per_word;
	}

	dspi->oper_word_size = DIV_ROUND_UP(dspi->oper_bits_per_word, 8);

	/*
	 * Update CTAR here (code is common for XSPI and DMA modes).
	 * We will update CTARE in the portion specific to XSPI, when we
	 * also know the preload value (DTCP).
	 */
	if (dspi->devtype_data->hw_version == DSPI_V1)
		dspi->cur_chip->ctar_val |= SPI_FRAME_BITS(dspi->oper_bits_per_word);

	regmap_write(dspi->regmap, SPI_CTAR(0), dspi->cur_chip->ctar_val);
}

static void dspi_fifo_write(struct fsl_dspi *dspi)
{
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;
	int num_fifo_entries = dspi->devtype_data->fifo_size;
	struct spi_transfer *xfer = dspi->cur_transfer;
	struct spi_message *msg = dspi->cur_msg;
	int num_words, num_bytes;

	dspi_setup_accel(dspi);

	/* In XSPI mode each 32-bit word occupies 2 TX FIFO entries for DSPI V1 */
	if (hw_ver == DSPI_V1 && dspi->oper_word_size == 4)
		num_fifo_entries /= 2;

	/*
	 * Integer division intentionally trims off odd (or non-multiple of 4)
	 * numbers of bytes at the end of the buffer, which will be sent next
	 * time using a smaller oper_word_size.
	 * In V2 case, only 32b words can be grouped to be sent with the same
	 * command. 8b and 16b words should be sent using a single command.
	 */
	if (hw_ver != DSPI_V1 && !dspi_use_single_cmd(dspi))
		num_words = 1;
	else
		num_words = dspi->len / dspi->oper_word_size;

	if (num_words > num_fifo_entries)
		num_words = num_fifo_entries;

	/* Update total number of bytes that were transferred */
	num_bytes = num_words * dspi->oper_word_size;
	msg->actual_length += num_bytes;
	dspi->progress += num_bytes / DIV_ROUND_UP(xfer->bits_per_word, 8);

	/*
	 * Update shared variable for use in the next interrupt (both in
	 * dspi_fifo_read and in dspi_fifo_write).
	 */
	dspi->words_in_flight = num_words;

	spi_take_timestamp_pre(dspi->ctlr, xfer, dspi->progress, !dspi->irq);

	dspi_xspi_fifo_write(dspi, num_words);
	/*
	 * Everything after this point is in a potential race with the next
	 * interrupt, so we must never use dspi->words_in_flight again since it
	 * might already be modified by the next dspi_fifo_write.
	 */

	spi_take_timestamp_post(dspi->ctlr, dspi->cur_transfer,
				dspi->progress, !dspi->irq);
}

static int dspi_rxtx(struct fsl_dspi *dspi)
{
	dspi_fifo_read(dspi);

	if (!dspi->len)
		/* Success! */
		return 0;

	dspi_fifo_write(dspi);

	return -EINPROGRESS;
}

static int dspi_poll(struct fsl_dspi *dspi)
{
	const u32 sr = dspi_get_sr(dspi->devtype_data->hw_version);
	int tries = 1000;
	u32 sr_val;

	do {
		regmap_read(dspi->regmap, sr, &sr_val);
		regmap_write(dspi->regmap, sr, sr_val);

		if (sr_val & SPI_SR_CMDTCF)
			break;
	} while (--tries);

	if (!tries)
		return -ETIMEDOUT;

	return dspi_rxtx(dspi);
}

static irqreturn_t dspi_interrupt(int irq, void *dev_id)
{
	struct fsl_dspi *dspi = (struct fsl_dspi *)dev_id;
	const u32 sr = dspi_get_sr(dspi->devtype_data->hw_version);
	u32 sr_val;

	regmap_read(dspi->regmap, sr, &sr_val);
	regmap_write(dspi->regmap, sr, sr_val);

	if (!(sr_val & SPI_SR_CMDTCF))
		return IRQ_NONE;

	if (dspi_rxtx(dspi) == 0)
		complete(&dspi->xfer_done);

	return IRQ_HANDLED;
}

static void dspi_assert_cs(struct spi_device *spi, bool *cs)
{
	if (!spi_get_csgpiod(spi, 0) || *cs)
		return;

	gpiod_set_value_cansleep(spi_get_csgpiod(spi, 0), true);
	*cs = true;
}

static void dspi_deassert_cs(struct spi_device *spi, bool *cs)
{
	if (!spi_get_csgpiod(spi, 0) || !*cs)
		return;

	gpiod_set_value_cansleep(spi_get_csgpiod(spi, 0), false);
	*cs = false;
}

static int dspi_transfer_one_message(struct spi_controller *ctlr,
				     struct spi_message *message)
{
	struct fsl_dspi *dspi = spi_controller_get_devdata(ctlr);
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;
	const u32 sr = dspi_get_sr(hw_ver);
	struct spi_device *spi = message->spi;
	struct spi_transfer *transfer;
	bool cs = false;
	int status = 0;
	u32 val = 0;
	bool cs_change = false;

	message->actual_length = 0;

	/* Put DSPI in running mode if halted. */
	regmap_read(dspi->regmap, SPI_MCR, &val);
	if (val & SPI_MCR_HALT) {
		regmap_update_bits(dspi->regmap, SPI_MCR, SPI_MCR_HALT, 0);
		while (regmap_read(dspi->regmap, sr, &val) >= 0 &&
				!(val & SPI_SR_TXRXS))
			;
	}

	list_for_each_entry(transfer, &message->transfers, transfer_list) {
		dspi->cur_transfer = transfer;
		dspi->cur_msg = message;
		dspi->cur_chip = spi_get_ctldata(spi);

		dspi_assert_cs(spi, &cs);

		/* Prepare command word for CMD FIFO */
		dspi->tx_cmd = dspi_pushr_cmd_ctas_mask(hw_ver, 0);
		if (!spi_get_csgpiod(spi, 0))
			dspi->tx_cmd |= SPI_PUSHR_CMD_PCS(spi_get_chipselect(spi, 0));

		if (list_is_last(&dspi->cur_transfer->transfer_list,
				 &dspi->cur_msg->transfers)) {
			/* Leave PCS activated after last transfer when
			 * cs_change is set.
			 */
			if (transfer->cs_change)
				dspi->tx_cmd |= dspi_pushr_cmd_cont_mask(hw_ver);
		} else {
			/* Keep PCS active between transfers in same message
			 * when cs_change is not set, and de-activate PCS
			 * between transfers in the same message when
			 * cs_change is set.
			 */
			if (!transfer->cs_change)
				dspi->tx_cmd |= dspi_pushr_cmd_cont_mask(hw_ver);
		}

		cs_change = transfer->cs_change;
		dspi->tx = transfer->tx_buf;
		dspi->rx = transfer->rx_buf;
		dspi->len = transfer->len;
		dspi->progress = 0;

		regmap_update_bits(dspi->regmap, SPI_MCR,
				   SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF,
				   SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF);
		regmap_write(dspi->regmap, sr, SPI_SR_CLEAR);

		spi_take_timestamp_pre(dspi->ctlr, dspi->cur_transfer,
				       dspi->progress, !dspi->irq);

		if (dspi->devtype_data->trans_mode == DSPI_DMA_MODE) {
			status = dspi_dma_xfer(dspi);
		} else {
			dspi_fifo_write(dspi);

			if (dspi->irq) {
				wait_for_completion(&dspi->xfer_done);
				reinit_completion(&dspi->xfer_done);
			} else {
				do {
					status = dspi_poll(dspi);
				} while (status == -EINPROGRESS);
			}
		}
		if (status)
			break;

		spi_transfer_delay_exec(transfer);

		if (!(dspi->tx_cmd & dspi_pushr_cmd_cont_mask(hw_ver)))
			dspi_deassert_cs(spi, &cs);
	}

	if (status || !cs_change) {
		/* Put DSPI in stop mode */
		regmap_update_bits(dspi->regmap, SPI_MCR,
				   SPI_MCR_HALT, SPI_MCR_HALT);
		while (regmap_read(dspi->regmap, sr, &val) >= 0 &&
		       val & SPI_SR_TXRXS)
			;
	}

	message->status = status;
	spi_finalize_current_message(ctlr);

	return status;
}

static int dspi_set_mtf(struct fsl_dspi *dspi)
{
	if (spi_controller_is_target(dspi->ctlr))
		return 0;

	if (dspi->mtf_enabled)
		regmap_update_bits(dspi->regmap, SPI_MCR, SPI_MCR_MTFE,
				   SPI_MCR_MTFE);
	else
		regmap_update_bits(dspi->regmap, SPI_MCR, SPI_MCR_MTFE, 0);

	return 0;
}

static int dspi_setup(struct spi_device *spi)
{
	struct fsl_dspi *dspi = spi_controller_get_devdata(spi->controller);
	u32 period_ns = DIV_ROUND_UP(NSEC_PER_SEC, spi->max_speed_hz);
	unsigned char br = 0, pbr = 0, pcssck = 0, cssck = 0;
	u32 quarter_period_ns = DIV_ROUND_UP(period_ns, 4);
	u32 cs_sck_delay = 0, sck_cs_delay = 0;
	struct fsl_dspi_platform_data *pdata;
	unsigned char pasc = 0, asc = 0;
	struct gpio_desc *gpio_cs;
	struct chip_data *chip;
	unsigned long clkrate;
	bool cs = true;
	int val;

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (chip == NULL) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
	}

	pdata = dev_get_platdata(&dspi->pdev->dev);

	if (!pdata) {
		val = spi_delay_to_ns(&spi->cs_setup, NULL);
		cs_sck_delay = val >= 0 ? val : 0;
		if (!cs_sck_delay)
			of_property_read_u32(spi->dev.of_node,
					     "fsl,spi-cs-sck-delay",
					     &cs_sck_delay);

		val = spi_delay_to_ns(&spi->cs_hold, NULL);
		sck_cs_delay =  val >= 0 ? val : 0;
		if (!sck_cs_delay)
			of_property_read_u32(spi->dev.of_node,
					     "fsl,spi-sck-cs-delay",
					     &sck_cs_delay);
	} else {
		cs_sck_delay = pdata->cs_sck_delay;
		sck_cs_delay = pdata->sck_cs_delay;
	}

	/* Since tCSC and tASC apply to continuous transfers too, avoid SCK
	 * glitches of half a cycle by never allowing tCSC + tASC to go below
	 * half a SCK period.
	 */
	if (cs_sck_delay < quarter_period_ns)
		cs_sck_delay = quarter_period_ns;
	if (sck_cs_delay < quarter_period_ns)
		sck_cs_delay = quarter_period_ns;

	dev_dbg(&spi->dev,
		"DSPI controller timing params: CS-to-SCK delay %u ns, SCK-to-CS delay %u ns\n",
		cs_sck_delay, sck_cs_delay);

	clkrate = clk_get_rate(dspi->clk);

	if (is_s32cc_dspi(dspi) && spi->max_speed_hz > SPI_25MHZ)
		dspi->mtf_enabled = true;
	else
		dspi->mtf_enabled = false;

	dspi_set_mtf(dspi);

	hz_to_spi_baud(&pbr, &br, spi->max_speed_hz, clkrate, dspi->mtf_enabled);

	/* Set PCS to SCK delay scale values */
	ns_delay_scale(&pcssck, &cssck, cs_sck_delay, clkrate);

	/* Set After SCK delay scale values */
	ns_delay_scale(&pasc, &asc, sck_cs_delay, clkrate);

	chip->ctar_val = 0;
	if (spi->mode & SPI_CPOL)
		chip->ctar_val |= SPI_CTAR_CPOL;
	if (spi->mode & SPI_CPHA)
		chip->ctar_val |= SPI_CTAR_CPHA;

	if (!spi_controller_is_target(dspi->ctlr)) {
		chip->ctar_val |= SPI_CTAR_PCSSCK(pcssck) |
				  SPI_CTAR_CSSCK(cssck) |
				  SPI_CTAR_PASC(pasc) |
				  SPI_CTAR_ASC(asc) |
				  SPI_CTAR_PBR(pbr) |
				  SPI_CTAR_BR(br);

		if (dspi->mtf_enabled)
			chip->ctar_val |= SPI_CTAR_DBR;

		if (spi->mode & SPI_LSB_FIRST)
			chip->ctar_val |= SPI_CTAR_LSBFE;
	}

	gpio_cs = spi_get_csgpiod(spi, 0);
	if (gpio_cs)
		gpiod_direction_output(gpio_cs, false);

	dspi_deassert_cs(spi, &cs);

	spi_set_ctldata(spi, chip);

	return 0;
}

static void dspi_cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata(spi);

	dev_dbg(&spi->dev, "spi_device %u.%u cleanup\n",
		spi->controller->bus_num, spi_get_chipselect(spi, 0));

	kfree(chip);
}

static const struct of_device_id fsl_dspi_dt_ids[] = {
	{
		.compatible = "fsl,vf610-dspi",
		.data = &devtype_data[VF610],
	}, {
		.compatible = "fsl,ls1021a-v1.0-dspi",
		.data = &devtype_data[LS1021A],
	}, {
		.compatible = "fsl,ls1012a-dspi",
		.data = &devtype_data[LS1012A],
	}, {
		.compatible = "fsl,ls1028a-dspi",
		.data = &devtype_data[LS1028A],
	}, {
		.compatible = "fsl,ls1043a-dspi",
		.data = &devtype_data[LS1043A],
	}, {
		.compatible = "fsl,ls1046a-dspi",
		.data = &devtype_data[LS1046A],
	}, {
		.compatible = "fsl,ls2080a-dspi",
		.data = &devtype_data[LS2080A],
	}, {
		.compatible = "fsl,ls2085a-dspi",
		.data = &devtype_data[LS2085A],
	}, {
		.compatible = "fsl,lx2160a-dspi",
		.data = &devtype_data[LX2160A],
	}, {
		.compatible = "nxp,s32cc-dspi",
		.data = &devtype_data[S32CC],
	}, {
		.compatible = "nxp,s32n79-dspi",
		.data = &devtype_data[S32N79],
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_dspi_dt_ids);

#ifdef CONFIG_PM_SLEEP
static int dspi_suspend(struct device *dev)
{
	struct fsl_dspi *dspi = dev_get_drvdata(dev);

	if (dspi->irq)
		disable_irq(dspi->irq);
	spi_controller_suspend(dspi->ctlr);
	clk_disable_unprepare(dspi->clk);

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int dspi_resume(struct device *dev)
{
	struct fsl_dspi *dspi = dev_get_drvdata(dev);
	int ret;

	pinctrl_pm_select_default_state(dev);

	ret = clk_prepare_enable(dspi->clk);
	if (ret)
		return ret;
	spi_controller_resume(dspi->ctlr);

	ret = dspi_init(dspi);
	if (ret) {
		dev_err(dev, "failed to initialize dspi during resume\n");
		return ret;
	}

	dspi_set_mtf(dspi);

	if (dspi->irq)
		enable_irq(dspi->irq);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(dspi_pm, dspi_suspend, dspi_resume);

static const struct regmap_range dspi_yes_ranges[] = {
	regmap_reg_range(SPI_MCR, SPI_MCR),
	regmap_reg_range(SPI_TCR, SPI_CTAR(3)),
	regmap_reg_range(SPI_SR, SPI_TXFR(3)),
	regmap_reg_range(SPI_RXFR(0), SPI_RXFR(3)),
	regmap_reg_range(SPI_CTARE(0), SPI_CTARE(3)),
	regmap_reg_range(SPI_SREX, SPI_SREX),
};

static const struct regmap_range s32cc_dspi_yes_ranges[] = {
	regmap_reg_range(SPI_MCR, SPI_MCR),
	regmap_reg_range(SPI_TCR, SPI_CTAR(5)),
	regmap_reg_range(SPI_SR, SPI_TXFR(4)),
	regmap_reg_range(SPI_RXFR(0), SPI_RXFR(4)),
	regmap_reg_range(SPI_CTARE(0), SPI_CTARE(5)),
	regmap_reg_range(SPI_SREX, SPI_SREX),
};

static const struct regmap_range s32n_dspi_yes_ranges[] = {
	regmap_reg_range(SPI_MCR, SPI_MCR),
	regmap_reg_range(SPI_TCR, SPI_CTAR(5)),
	regmap_reg_range(SPI_V2_RSER, SPI_V2_PUSHR_CMD),
	regmap_reg_range(SPI_V2_PUSHR_CMD, SPI_PUSHR),
	regmap_reg_range(SPI_PUSHR, SPI_V2_RXFR(0)),
	regmap_reg_range(SPI_V2_TXFR(0), SPI_V2_TXFR(15)),
	regmap_reg_range(SPI_V2_RXFR(0), SPI_V2_RXFR(15)),
	regmap_reg_range(SPI_V2_SR, SPI_V2_SR),
	regmap_reg_range(SPI_V2_FIFO_SR, SPI_V2_FIFO_SR),
};

static const struct regmap_access_table dspi_access_table = {
	.yes_ranges	= dspi_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(dspi_yes_ranges),
};

static const struct regmap_access_table s32cc_dspi_access_table = {
	.yes_ranges	= s32cc_dspi_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(s32cc_dspi_yes_ranges),
};

static const struct regmap_access_table s32n_dspi_access_table = {
	.yes_ranges	= s32n_dspi_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(s32n_dspi_yes_ranges),
};

static const struct regmap_range dspi_volatile_ranges[] = {
	regmap_reg_range(SPI_MCR, SPI_TCR),
	regmap_reg_range(SPI_SR, SPI_SR),
	regmap_reg_range(SPI_PUSHR, SPI_RXFR(4)),
};

static const struct regmap_access_table dspi_volatile_table = {
	.yes_ranges	= dspi_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(dspi_volatile_ranges),
};

static const struct regmap_range dspi_xspi_volatile_ranges[] = {
	regmap_reg_range(SPI_MCR, SPI_TCR),
	regmap_reg_range(SPI_SR, SPI_SR),
	regmap_reg_range(SPI_PUSHR, SPI_RXFR(4)),
	regmap_reg_range(SPI_SREX, SPI_SREX),
};

static const struct regmap_access_table dspi_xspi_volatile_table = {
	.yes_ranges	= dspi_xspi_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(dspi_xspi_volatile_ranges),
};

enum {
	DSPI_REGMAP,
	S32CC_DSPI_REGMAP,
	DSPI_XSPI_REGMAP,
	S32CC_DSPI_XSPI_REGMAP,
	DSPI_PUSHR,
	S32N_DSPI_REGMAP
};

static const struct regmap_config dspi_regmap_config[] = {
	[DSPI_REGMAP] = {
		.reg_bits	= 32,
		.val_bits	= 32,
		.reg_stride	= 4,
		.max_register	= 0x8C,
		.volatile_table	= &dspi_volatile_table,
		.wr_table	= &dspi_access_table,
		.rd_table	= &dspi_access_table,
	},
	[S32CC_DSPI_REGMAP] = {
		.reg_bits	= 32,
		.val_bits	= 32,
		.reg_stride	= 4,
		.max_register	= 0x8C,
		.volatile_table	= &dspi_volatile_table,
		.wr_table	= &s32cc_dspi_access_table,
		.rd_table	= &s32cc_dspi_access_table,
	},
	[DSPI_XSPI_REGMAP] = {
		.reg_bits	= 32,
		.val_bits	= 32,
		.reg_stride	= 4,
		.max_register	= 0x13c,
		.volatile_table	= &dspi_xspi_volatile_table,
		.wr_table	= &dspi_access_table,
		.rd_table	= &dspi_access_table,
	},
	[S32CC_DSPI_XSPI_REGMAP] = {
		.reg_bits	= 32,
		.val_bits	= 32,
		.reg_stride	= 4,
		.max_register	= 0x13c,
		.volatile_table	= &dspi_xspi_volatile_table,
		.wr_table	= &s32cc_dspi_access_table,
		.rd_table	= &s32cc_dspi_access_table,
	},
	[DSPI_PUSHR] = {
		.name		= "pushr",
		.reg_bits	= 16,
		.val_bits	= 16,
		.reg_stride	= 2,
		.max_register	= 0x2,
	},
	[S32N_DSPI_REGMAP] = {
		.reg_bits	= 32,
		.val_bits	= 32,
		.reg_stride	= 4,
		.max_register	= 0x184,
		.wr_table	= &s32n_dspi_access_table,
		.rd_table	= &s32n_dspi_access_table,
	},
};

static int dspi_init(struct fsl_dspi *dspi)
{
	enum dspi_hw_ver hw_ver = dspi->devtype_data->hw_version;
	const u32 rser = dspi_get_rser(hw_ver);
	const u32 sr = dspi_get_sr(hw_ver);
	u32 rser_val;

	unsigned int mcr;

	/* Set idle states for all chip select signals to high */
	mcr = SPI_MCR_PCSIS(GENMASK(dspi->ctlr->max_native_cs - 1, 0));

	if (dspi->devtype_data->trans_mode == DSPI_XSPI_MODE &&
	    dspi->devtype_data->hw_version == DSPI_V1)
		mcr |= SPI_MCR_XSPI;
	if (!spi_controller_is_target(dspi->ctlr))
		mcr |= SPI_MCR_HOST;

	mcr |= SPI_MCR_HALT;

	regmap_write(dspi->regmap, SPI_MCR, mcr);
	regmap_write(dspi->regmap, sr, SPI_SR_CLEAR);

	switch (dspi->devtype_data->trans_mode) {
	case DSPI_XSPI_MODE:
		regmap_write(dspi->regmap, rser, SPI_RSER_CMDTCFE);
		break;
	case DSPI_DMA_MODE:
		rser_val = SPI_RSER_TFFFE | SPI_RSER_TFFFD |
				SPI_RSER_RFDFE | SPI_RSER_RFDFD;
		if (hw_ver == DSPI_V2)
			rser_val |= SPI_RSER_CMDFFFE | SPI_RSER_CMDFFFD;

		regmap_write(dspi->regmap, rser, rser_val);
		break;
	default:
		dev_err(&dspi->pdev->dev, "unsupported trans_mode %u\n",
			dspi->devtype_data->trans_mode);
		return -EINVAL;
	}

	return 0;
}

static int dspi_target_abort(struct spi_controller *host)
{
	struct fsl_dspi *dspi = spi_controller_get_devdata(host);

	/*
	 * Terminate all pending DMA transactions for the SPI working
	 * in TARGET mode.
	 */
	if (dspi->devtype_data->trans_mode == DSPI_DMA_MODE) {
		dmaengine_terminate_sync(dspi->dma->chan_rx);
		dmaengine_terminate_sync(dspi->dma->chan_tx);
		if (dspi->dma->chan_cmd)
			dmaengine_terminate_sync(dspi->dma->chan_cmd);
	}

	/* Clear the internal DSPI RX and TX FIFO buffers */
	regmap_update_bits(dspi->regmap, SPI_MCR,
			   SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF,
			   SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF);

	return 0;
}

static int dspi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct regmap_config *regmap_config;
	struct fsl_dspi_platform_data *pdata;
	struct spi_controller *ctlr;
	int ret, cs_num, bus_num = -1;
	enum dspi_hw_ver hw_ver;
	struct fsl_dspi *dspi;
	struct resource *res;
	void __iomem *base;
	bool big_endian;

	dspi = devm_kzalloc(&pdev->dev, sizeof(*dspi), GFP_KERNEL);
	if (!dspi)
		return -ENOMEM;

	if (of_property_read_bool(np, "spi-slave"))
		ctlr = spi_alloc_target(&pdev->dev, 0);
	else
		ctlr = spi_alloc_host(&pdev->dev, 0);
	if (!ctlr)
		return -ENOMEM;

	spi_controller_set_devdata(ctlr, dspi);
	platform_set_drvdata(pdev, dspi);

	dspi->pdev = pdev;
	dspi->ctlr = ctlr;

	ctlr->setup = dspi_setup;
	ctlr->transfer_one_message = dspi_transfer_one_message;
	ctlr->dev.of_node = pdev->dev.of_node;

	ctlr->cleanup = dspi_cleanup;
	ctlr->target_abort = dspi_target_abort;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;
	ctlr->use_gpio_descriptors = true;

	pdata = dev_get_platdata(&pdev->dev);
	if (pdata) {
		ctlr->num_chipselect = ctlr->max_native_cs = pdata->cs_num;
		ctlr->bus_num = pdata->bus_num;

		/* Only Coldfire uses platform data */
		dspi->devtype_data = &devtype_data[MCF5441X];
		big_endian = true;
	} else {

		ret = of_property_read_u32(np, "spi-num-chipselects", &cs_num);
		if (ret < 0) {
			dev_err(&pdev->dev, "can't get spi-num-chipselects\n");
			goto out_ctlr_put;
		}
		ctlr->num_chipselect = ctlr->max_native_cs = cs_num;

		of_property_read_u32(np, "bus-num", &bus_num);
		ctlr->bus_num = bus_num;

		if (of_property_read_bool(np, "spi-slave"))
			ctlr->target = true;

		dspi->devtype_data = of_device_get_match_data(&pdev->dev);
		if (!dspi->devtype_data) {
			dev_err(&pdev->dev, "can't get devtype_data\n");
			ret = -EFAULT;
			goto out_ctlr_put;
		}

		if (ctlr->target && is_s32cc_dspi(dspi))
			dspi->devtype_data = &devtype_data[S32CC_TARGET];

		if (ctlr->target && is_s32n_dspi(dspi))
			dspi->devtype_data = &devtype_data[S32N79_TARGET];

		big_endian = of_device_is_big_endian(np);
	}

	hw_ver = dspi->devtype_data->hw_version;

	/* This data and cmd separation only applies for V1 hw version, since V2
	 * has separate 32Bit PUSHR_CMD register
	 */
	if (hw_ver == DSPI_V1) {
		if (big_endian) {
			dspi->pushr_cmd = 0;
			dspi->pushr_tx = 2;
		} else {
			dspi->pushr_cmd = 2;
			dspi->pushr_tx = 0;
		}
	}

	if (dspi->devtype_data->trans_mode == DSPI_XSPI_MODE ||
			hw_ver != DSPI_V1)
		ctlr->bits_per_word_mask = SPI_BPW_RANGE_MASK(4, 32);
	else
		ctlr->bits_per_word_mask = SPI_BPW_RANGE_MASK(4, 16);

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		goto out_ctlr_put;
	}

	if (is_s32n_dspi(dspi)) {
		regmap_config = &dspi_regmap_config[S32N_DSPI_REGMAP];
	} else if (is_s32cc_dspi(dspi)) {
		if (dspi->devtype_data->trans_mode == DSPI_XSPI_MODE)
			regmap_config =
				&dspi_regmap_config[S32CC_DSPI_XSPI_REGMAP];
		else
			regmap_config =
				&dspi_regmap_config[S32CC_DSPI_REGMAP];
	} else {
		if (dspi->devtype_data->trans_mode == DSPI_XSPI_MODE)
			regmap_config =
				&dspi_regmap_config[DSPI_XSPI_REGMAP];
		else
			regmap_config =
				&dspi_regmap_config[DSPI_REGMAP];
	}

	dspi->regmap = devm_regmap_init_mmio(&pdev->dev, base, regmap_config);
	if (IS_ERR(dspi->regmap)) {
		dev_err(&pdev->dev, "failed to init regmap: %ld\n",
				PTR_ERR(dspi->regmap));
		ret = PTR_ERR(dspi->regmap);
		goto out_ctlr_put;
	}

	if (hw_ver == DSPI_V1 && dspi->devtype_data->trans_mode == DSPI_XSPI_MODE) {
		dspi->regmap_pushr = devm_regmap_init_mmio(
			&pdev->dev, base + SPI_PUSHR,
			&dspi_regmap_config[DSPI_PUSHR]);
		if (IS_ERR(dspi->regmap_pushr)) {
			dev_err(&pdev->dev,
				"failed to init pushr regmap: %ld\n",
				PTR_ERR(dspi->regmap_pushr));
			ret = PTR_ERR(dspi->regmap_pushr);
			goto out_ctlr_put;
		}
	}

	dspi->clk = devm_clk_get_enabled(&pdev->dev, "dspi");
	if (IS_ERR(dspi->clk)) {
		ret = PTR_ERR(dspi->clk);
		dev_err(&pdev->dev, "unable to get clock\n");
		goto out_ctlr_put;
	}

	ret = dspi_init(dspi);
	if (ret)
		goto out_ctlr_put;

	init_completion(&dspi->xfer_done);

	if (dspi->devtype_data->trans_mode != DSPI_DMA_MODE) {
		dspi->irq = platform_get_irq(pdev, 0);
		if (dspi->irq <= 0) {
			dev_info(&pdev->dev,
				 "can't get platform irq, using poll mode\n");
			dspi->irq = 0;
			goto poll_mode;
		}

		ret = request_threaded_irq(dspi->irq, dspi_interrupt, NULL,
					   IRQF_SHARED, pdev->name, dspi);
		if (ret < 0) {
			dev_err(&pdev->dev, "Unable to attach DSPI interrupt\n");
			goto out_ctlr_put;
		}
	}

poll_mode:

	if (dspi->devtype_data->trans_mode == DSPI_DMA_MODE) {
		ret = dspi_request_dma(dspi, res->start);
		if (ret < 0) {
			dev_err(&pdev->dev, "can't get dma channels\n");
			goto out_free_irq;
		}
	}

	ctlr->max_speed_hz =
		clk_get_rate(dspi->clk) / dspi->devtype_data->max_clock_factor;

	if (dspi->devtype_data->trans_mode != DSPI_DMA_MODE)
		ctlr->ptp_sts_supported = true;

	ret = spi_register_controller(ctlr);
	if (ret != 0) {
		dev_err(&pdev->dev, "Problem registering DSPI ctlr\n");
		goto out_release_dma;
	}

	return ret;

out_release_dma:
	dspi_release_dma(dspi);
out_free_irq:
	if (dspi->irq)
		free_irq(dspi->irq, dspi);
out_ctlr_put:
	spi_controller_put(ctlr);

	return ret;
}

static void dspi_remove(struct platform_device *pdev)
{
	struct fsl_dspi *dspi = platform_get_drvdata(pdev);

	/* Disconnect from the SPI framework */
	spi_unregister_controller(dspi->ctlr);

	/* Disable RX and TX */
	regmap_update_bits(dspi->regmap, SPI_MCR,
			   SPI_MCR_DIS_TXF | SPI_MCR_DIS_RXF,
			   SPI_MCR_DIS_TXF | SPI_MCR_DIS_RXF);

	/* Stop Running */
	regmap_update_bits(dspi->regmap, SPI_MCR, SPI_MCR_HALT, SPI_MCR_HALT);

	dspi_release_dma(dspi);
	if (dspi->irq)
		free_irq(dspi->irq, dspi);
}

static void dspi_shutdown(struct platform_device *pdev)
{
	dspi_remove(pdev);
}

static struct platform_driver fsl_dspi_driver = {
	.driver.name		= DRIVER_NAME,
	.driver.of_match_table	= fsl_dspi_dt_ids,
	.driver.pm		= &dspi_pm,
	.probe			= dspi_probe,
	.remove_new		= dspi_remove,
	.shutdown		= dspi_shutdown,
};
module_platform_driver(fsl_dspi_driver);

MODULE_DESCRIPTION("Freescale DSPI Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
