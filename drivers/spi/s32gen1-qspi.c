// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020-2021 NXP
 */
#include <linux/io.h>
#include <linux/mtd/spi-nor.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/cache.h>
#include <asm/cacheflush.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <asm/div64.h>
#include "spi-fsl-qspi.h"

#define QUADSPI_MCR_DQS_FA_SEL_SHIFT		24
#define QUADSPI_MCR_DQS_PAD_LOOPBACK_MASK	(0x2 << QUADSPI_MCR_DQS_FA_SEL_SHIFT)
#define QUADSPI_MCR_DQS_FA_SEL_SHIFT		24
#define QUADSPI_MCR_DQS_EXTERNAL_MASK		(0x3 << QUADSPI_MCR_DQS_FA_SEL_SHIFT)
#define QUADSPI_MCR_DQS_MASK				(0x3 << QUADSPI_MCR_DQS_FA_SEL_SHIFT)
#define QUADSPI_MCR_DLPEN_MASK				BIT(12)
#define QUADSPI_MCR_ISD2FA_EN_MASK			BIT(16)
#define QUADSPI_MCR_ISD3FA_EN_MASK			BIT(17)
#define QUADSPI_MCR_ISD2FB_EN_MASK			BIT(18)
#define QUADSPI_MCR_ISD3FB_EN_MASK			BIT(19)

#define QUADSPI_IPCR_SEQID_SHIFT			24

#define QUADSPI_DLLCR_SLV_UPD_EN_MASK		BIT(0)
#define QUADSPI_DLLCR_SLV_BYPASS_SHIFT		1
#define QUADSPI_DLLCR_SLV_BYPASS_EN_MASK	BIT(1)
#define QUADSPI_DLLCR_SLV_EN_SHIFT			2
#define QUADSPI_DLLCR_SLV_EN_MASK			BIT(2)
#define QUADSPI_DLLCR_SLV_AUTO_UPDT_SHIFT	3
#define QUADSPI_DLLCR_SLV_AUTO_UPDT_EN_MASK	BIT(3)
#define QUADSPI_DLLCR_SLV_DLY_COARSE_SHIFT	8
#define QUADSPI_DLLCR_SLV_DLY_COARSE_N_MASK	((N) << \
		QUADSPI_DLLCR_SLV_DLY_COARSE_SHIFT)
#define QUADSPI_DLLCR_DLLRES_SHIFT			20
#define QUADSPI_DLLCR_DLLRES_N_MASK(N)		((N) << 20)
#define QUADSPI_DLLCR_DLL_REFCNTR_SHIFT		24
#define QUADSPI_DLLCR_DLL_REFCNTR_N_MASK(N)	((N) << QUADSPI_DLLCR_DLL_REFCNTR_SHIFT)
#define QUADSPI_DLLCR_FREQEN_EN_MASK		BIT(30)
#define QUADSPI_DLLCR_DLLEN_EN_MASK			BIT(31)
#define QUADSPI_DLLCR_MASK					0x7FFFFFF0UL

#define QUADSPI_SFACR			0x104
#define QUADSPI_SFACR_BSWAP_EN	BIT(17)
#define QUADSPI_AWRCR			0x50
#define QUADSPI_DLLCRA			0x60

#define QUADSPI_SMPR_DLLFSMPFA_SHIFT	24
#define QUADSPI_SMPR_DLLFSMPFA_NTH(N)	((N) << QUADSPI_SMPR_DLLFSMPFA_SHIFT)
#define QUADSPI_SMPR_DLLFSMPFB_SHIFT	28
#define QUADSPI_SMPR_DLLFSMPFB_NTH(N)	((N) << QUADSPI_SMPR_DLLFSMPFB_SHIFT)

#define QUADSPI_DLCR					0x130
#define QUADSPI_DLCR_RESERVED_MASK		((0xff << 0) | (0xff << 16))
#define QUADSPI_DLCR_DLP_SEL_FA_SHIFT	14
#define QUADSPI_DLCR_DLP_SEL_FA_MASK(N)	((N) << QUADSPI_DLCR_DLP_SEL_FA_SHIFT)
#define QUADSPI_DLCR_DLP_SEL_FB_SHIFT	30
#define QUADSPI_DLCR_DLP_SEL_FB_MASK(N)	((N) << QUADSPI_DLCR_DLP_SEL_FB_SHIFT)

#define QUADSPI_DLPR					0x190
#define QUADSPI_DLPR_RESET_VALUE		0xaa553443

#define QUADSPI_DLLSR			0x12C
#define QUADSPI_DLLSR_SLVA_LOCK_MASK	BIT(14)
#define QUADSPI_DLLSR_DLLA_LOCK_MASK	BIT(15)
#define QUADSPI_TBSR			0x150
#define QUADSPI_TBSR_TRCTR_SHIFT	16
#define QUADSPI_TBSR_TRCTR(TBSR)	((TBSR) >> QUADSPI_TBSR_TRCTR_SHIFT)
#define QUADSPI_TBSR_TRBFL(TBSR)	((TBSR) & 0xFF)

#define	QUADSPI_FR_ALL_FLAGS_MASK	0xFFFFFFFF

#define QUADSPI_SFA_ADDR		0x10000000
#define QUADSPI_SFB_ADDR		0x20000000

#define FLASH_STATUS_WEL	0x02
#define QUADSPI_FLAG_PREV_READ_MEM	BIT(1)

#define OPRND0_SHIFT		0
#define PAD0_SHIFT			8
#define INSTR0_SHIFT		10
#define OPRND1_SHIFT		16
#define PAD1_SHIFT			24
#define INSTR1_SHIFT		26
#define OPRND0(x)			((x) << OPRND0_SHIFT)
#define PAD0(x)				((x) << PAD0_SHIFT)
#define INSTR0(x)			((x) << INSTR0_SHIFT)
#define OPRND1(x)			((x) << OPRND1_SHIFT)
#define PAD1(x)				((x) << PAD1_SHIFT)
#define INSTR1(x)			((x) << INSTR1_SHIFT)
#define LUT2PAD0(x)			(((x) >> PAD0_SHIFT) & 0x3)
#define LUT2INSTR0(x)		(((x) >> INSTR0_SHIFT) & 0x3f)

#define LUT_PAD1		0
#define LUT_PAD2		1
#define LUT_PAD4		2
#define ADDR24BIT		0x18
#define ADDR32BIT		0x20

#define LUT_INVALID_INDEX -1
#define LUT_STOP_CMD 0x0

/* JESD216D.01 */
#define SPINOR_OP_RDCR2		0x71
#define SPINOR_OP_WRCR2		0x72

#define QUADSPI_CFG2_OPI_MASK		(0x3)
#define QUADSPI_CFG2_STR_OPI_ENABLED	BIT(0)
#define QUADSPI_CFG2_DTR_OPI_ENABLED	BIT(1)

#define QUADSPI_LUT(x)	(QUADSPI_LUT_BASE + (x) * 4)

#define QUADSPI_S32GEN1_HIGH_FREQUENCY_VALUE	200000000

struct qspi_op {
	const struct spi_mem_op *op;
	u8 *cfg;
};

struct qspi_config {
	u32 mcr;
	u32 flshcr;
	u32 dllcr;
	u32 sfacr;
	u32 smpr;
	u32 dlcr;
	u32 flash1_size;
	u32 flash2_size;
	u32 dlpr;
};

/* JESD216D.01 operations used for DTR OPI switch */
static struct spi_mem_op rdcr2_sdr_op =
SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_RDCR2, 1),
	   SPI_MEM_OP_ADDR(0x4, 0x0, 1),
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_DATA_IN(1, NULL, 1));

static struct spi_mem_op wren_sdr_op =
SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_WREN, 1),
	   SPI_MEM_OP_NO_ADDR,
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_DATA_IN(0, NULL, 1));

static struct spi_mem_op rdsr_sdr_op =
SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_RDSR, 1),
	   SPI_MEM_OP_NO_ADDR,
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_DATA_IN(1, NULL, 1));

static struct spi_mem_op wrcr2_sdr_op =
SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_WRCR2, 1),
	   SPI_MEM_OP_ADDR(0x4, 0x0, 1),
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_DATA_OUT(1, NULL, 1));

/* JESD216D.01 operations used for soft reset */
static struct spi_mem_op rsten_ddr_op =
SPI_MEM_OP(SPI_MEM_OP_CMD(0x66, 8),
	   SPI_MEM_OP_NO_ADDR,
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_NO_DATA);

static struct spi_mem_op rst_ddr_op =
SPI_MEM_OP(SPI_MEM_OP_CMD(0x99, 8),
	   SPI_MEM_OP_NO_ADDR,
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_NO_DATA);

static u32 clear_fifos(struct fsl_qspi *q)
{
	u32 mcr_reg;

	mcr_reg = qspi_readl(q, q->iobase + QUADSPI_MCR);

	/* Clear TX & RX fifos */
	qspi_writel(q, mcr_reg | QUADSPI_MCR_CLR_RXF_MASK |
			QUADSPI_MCR_CLR_TXF_MASK |
			S32GEN1_QUADSPI_MCR_RESERVED_MASK | S32GEN1_QUADSPI_MCR_END_CFD_LE_MASK,
			q->iobase + QUADSPI_MCR);
	return mcr_reg;
}

static int qspi_write_reg(struct fsl_qspi *q,
			  const struct spi_mem_op *op, u8 lut_cfg)
{
	u32 mcr_reg, i, words = 0;
	u32 tbsr, trctr, trbfl;
	const u32 *txbuf = op->data.buf.out;
	u32 len = op->data.nbytes;
	void __iomem *base = q->iobase;

	mcr_reg = clear_fifos(q);

	/* Controller isn't busy */
	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	/* TX buffer is empty */
	while (QUADSPI_TBSR_TRBFL(qspi_readl(q, base + QUADSPI_TBSR)))
		;

	if (op->addr.nbytes) {
		/* Set address */
		qspi_writel(q, op->addr.val, base + QUADSPI_SFAR);
	}

	if (op->data.nbytes) {
		words = len / 4;

		if (len % 4)
			words++;

		for (i = 0; i < words; i++) {
			qspi_writel(q, *txbuf, base + QUADSPI_TBDR);
			txbuf++;
		}

		qspi_writel(q, (lut_cfg << QUADSPI_IPCR_SEQID_SHIFT) |
				len, base + QUADSPI_IPCR);

		while (qspi_readl(q, base + QUADSPI_SR) &
		       QUADSPI_SR_BUSY_MASK)
			;

		/* Wait until all bytes are transmitted */
		do {
			tbsr = qspi_readl(q, base + QUADSPI_TBSR);
			trctr = QUADSPI_TBSR_TRCTR(tbsr);
			trbfl = QUADSPI_TBSR_TRBFL(tbsr);
		} while ((trctr != words) || trbfl);
	} else {
		qspi_writel(q, (lut_cfg << QUADSPI_IPCR_SEQID_SHIFT),
				base + QUADSPI_IPCR);
	}

	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	qspi_writel(q, mcr_reg, base + QUADSPI_MCR);
	return 0;
}

static int qspi_read_reg(struct fsl_qspi *q,
			 const struct spi_mem_op *op, u8 lut_cfg)
{
	u32 mcr_reg, data;
	int i;
	u32 len = op->data.nbytes;
	u32 *rxbuf = op->data.buf.in;
	void __iomem *base = q->iobase;

	mcr_reg = clear_fifos(q);

	/* Controller isn't busy */
	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	/* Clear all flags */
	qspi_writel(q, QUADSPI_FR_ALL_FLAGS_MASK,
			base + QUADSPI_FR);

	/* Read using IP registers */
	qspi_writel(q, QUADSPI_RBCT_RXBRD_USEIPS,
			base + QUADSPI_RBCT);

	if (op->addr.nbytes)
		qspi_writel(q, op->addr.val, base + QUADSPI_SFAR);

	qspi_writel(q, (lut_cfg << QUADSPI_IPCR_SEQID_SHIFT) | len,
		base + QUADSPI_IPCR);

	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	i = 0;
	while (len > 0) {
		data = qspi_readl(q, base + QUADSPI_RBDR(i));

		if (len >= 4) {
			*((u32 *)rxbuf) = data;
			rxbuf += 4;
		} else {
			memcpy(rxbuf, &data, len);
			break;
		}

		len -= 4;
		i++;
	}

	qspi_writel(q, mcr_reg, base + QUADSPI_MCR);
	return 0;
}

static u8 busw_to_pads(u8 buswidth, int *status)
{
	*status = 0;
	switch (buswidth) {
	case 1:
		return 0x0;
	case 2:
		return 0x1;
	case 4:
		return 0x2;
	case 8:
		return 0x3;
	}

	*status = -1;
	return 0x3;
}

static void append_lut(struct lut_config *config, u16 lut)
{
	u32 conf_index = config->fill / 2;
	u32 mask, shift;

	if (config->fill % 2) {
		mask = GENMASK(31, 16);
		shift = 16;
	} else {
		mask = GENMASK(15, 0);
		shift = 0;
	}

	config->conf[conf_index] &= ~mask;
	config->conf[conf_index] |= (lut << shift);

	config->fill++;
}

static bool fill_qspi_cmd(struct fsl_qspi *q,
			  const struct spi_mem_op *op,
			  struct lut_config *lut_conf)
{
	u16 lut;
	int status;
	u8 opcode = op->cmd.opcode;
	u8 lut_cmd;

	switch (op->cmd.buswidth) {
		/* SPI */
	case 1:
		lut_cmd = LUT_CMD;
		break;
		/* OPI */
	case 8:
		if (q->ddr_mode)
			lut_cmd = LUT_CMD_DDR;
		else
			lut_cmd = LUT_CMD;
		break;
	default:
		return false;
	};

	lut = OPRND0(opcode) | PAD0(busw_to_pads(op->cmd.buswidth, &status)) |
	    INSTR0(lut_cmd);
	if (status)
		return false;

	append_lut(lut_conf, lut);

	/* Octal command */
	if (op->cmd.buswidth == 8) {
		lut = OPRND0(~opcode & 0xFFU) |
		    PAD0(busw_to_pads(op->cmd.buswidth, &status)) |
		    INSTR0(lut_cmd);
		if (status)
			return false;

		append_lut(lut_conf, lut);
	}

	return true;
}

static bool fill_qspi_addr(struct fsl_qspi *q,
			   const struct spi_mem_op *op,
			   struct lut_config *lut_conf)
{
	u16 lut;
	int status;
	u8 lut_addr;

	lut_addr = LUT_ADDR;
	switch (op->addr.buswidth) {
		/* No address */
	case 0:
		return true;
		/* SPI */
	case 1:
		break;
		/* OPI */
	case 8:
		if (op->memop && q->ddr_mode)
			lut_addr = LUT_ADDR_DDR;
		break;
	default:
		return false;
	};

	if (op->addr.nbytes) {
		lut = OPRND0(op->addr.nbytes * 8) |
		    PAD0(busw_to_pads(op->addr.buswidth, &status)) |
		    INSTR0(lut_addr);
		if (status)
			return false;

		append_lut(lut_conf, lut);
	}

	return true;
}

static bool fill_qspi_data(struct fsl_qspi *q,
			   const struct spi_mem_op *op,
			   struct lut_config *lut_conf)
{
	u16 lut;
	int status;
	u8 lut_read, lut_write;

	if (!op->data.nbytes)
		return true;

	lut_read = LUT_FSL_READ;
	lut_write = LUT_FSL_WRITE;
	switch (op->data.buswidth) {
		/* SPI */
	case 1:
		break;
		/* OPI */
	case 8:
		if (op->memop && q->ddr_mode) {
			lut_read = LUT_READ_DDR;
			lut_write = LUT_WRITE_DDR;
		}
		break;
	default:
		return false;
	};

	if (op->data.dir == SPI_MEM_DATA_IN)
		lut = INSTR0(lut_read);
	else
		lut = INSTR0(lut_write);

	/* HW limitation for memory write operation */
	if (op->data.dir == SPI_MEM_DATA_OUT && op->memop) {
		lut |= OPRND0(0);
	} else {
		if (op->data.nbytes > q->devtype_data->rxfifo)
			lut |= OPRND0(q->devtype_data->rxfifo);
		else
			lut |= OPRND0(op->data.nbytes);
	}

	if (op->data.buswidth)
		lut |= PAD0(busw_to_pads(op->data.buswidth, &status));
	else
		lut |= PAD0(busw_to_pads(op->cmd.buswidth, &status));

	if (status)
		return false;

	append_lut(lut_conf, lut);

	return true;
}

static bool add_op_to_lutdb(struct fsl_qspi *q,
			    const struct spi_mem_op *op, u8 *index)
{
	u8 opcode = op->cmd.opcode;
	struct lut_config *lut_conf;
	u16 lut;
	int status;

	lut_conf = &q->lut_configs[opcode];
	lut_conf->fill = 0;

	if (!fill_qspi_cmd(q, op, lut_conf))
		return false;

	if (!fill_qspi_addr(q, op, lut_conf))
		return false;

	if (op->dummy.nbytes) {
		lut = OPRND0(op->dummy.nbytes) |
		    INSTR0(LUT_DUMMY) |
		    PAD0(busw_to_pads(op->dummy.buswidth, &status));
		if (status)
			return false;

		append_lut(lut_conf, lut);
	}

	if (!fill_qspi_data(q, op, lut_conf))
		return false;

	append_lut(lut_conf, LUT_STOP_CMD);
	append_lut(lut_conf, LUT_STOP_CMD);

	if (!lut_conf->index) {
		lut_conf->index = q->luts_next_config;
		q->luts_next_config++;
	}
	*index = lut_conf->index;

	return true;
}

static void set_lut(struct fsl_qspi *q, u8 index, u8 opcode)
{
	u32 *iterb, *iter;
	u32 lutaddr;
	void __iomem *base = q->iobase;

	iter = &q->lut_configs[opcode].conf[0];
	iterb = iter;

	lutaddr = index * S32GEN1_LUTS_PER_CONFIG;

	/* Unlock the LUT */
	qspi_writel(q, QUADSPI_LUTKEY_VALUE, base + QUADSPI_LUTKEY);
	qspi_writel(q, QUADSPI_LCKER_UNLOCK, base + QUADSPI_LCKCR);

	while (((*iter & GENMASK(15, 0)) != LUT_STOP_CMD) &&
	       (iter - iterb < sizeof(q->lut_configs[opcode].conf))) {
		qspi_writel(q, *iter, base + QUADSPI_LUT(lutaddr));
		iter++;
		lutaddr++;
	}
	qspi_writel(q, LUT_STOP_CMD, base + QUADSPI_LUT(lutaddr));

	/* Lock the LUT */
	qspi_writel(q, QUADSPI_LUTKEY_VALUE, base + QUADSPI_LUTKEY);
	qspi_writel(q, QUADSPI_LCKER_LOCK, base + QUADSPI_LCKCR);
}

static bool enable_op(struct fsl_qspi *q, const struct spi_mem_op *op)
{
	u8 lut_index;
	u8 opcode = op->cmd.opcode;

	if (q->luts_next_config >= S32GEN1_MAX_LUTS_CONFIGS)
		return false;

	if (!q->lut_configs[opcode].enabled) {
		if (!add_op_to_lutdb(q, op, &lut_index))
			return false;

		set_lut(q, lut_index, opcode);
		q->lut_configs[opcode].enabled = true;
	}

	return true;
}

static bool enable_operators(struct fsl_qspi *q,
			     struct qspi_op *ops, size_t n_ops)
{
	bool res;
	size_t i;
	const struct spi_mem_op *op;
	u8 *cfg;

	for (i = 0; i < n_ops; i++) {
		op = ops[i].op;
		cfg = ops[i].cfg;

		/* In case it's already enabled */
		q->lut_configs[op->cmd.opcode].enabled = false;
		res = enable_op(q, op);
		*cfg = q->lut_configs[op->cmd.opcode].index;

		if (!res || !q->lut_configs[op->cmd.opcode].enabled)
			return false;
	}

	return true;
}

static void disable_operators(struct fsl_qspi *q,
		struct qspi_op *ops, size_t n_ops)
{
	size_t i;
	const struct spi_mem_op *op;

	for (i = 0; i < n_ops; i++) {
		op = ops[i].op;

		q->lut_configs[op->cmd.opcode].enabled = false;
	}
}

static int memory_enable_ddr(struct fsl_qspi *q)
{
	u8 wren_cfg, rdcr2_cfg, rdsr_cfg, wrcr2_cfg;
	u8 cfg2_reg = 0x0;
	u8 status = 0;
	u32 mcr2;
	void __iomem *base = q->iobase;
	struct qspi_op ops[] = {
		{
		 .op = &rdcr2_sdr_op,
		 .cfg = &rdcr2_cfg,
		 },
		{
		 .op = &wren_sdr_op,
		 .cfg = &wren_cfg,
		 },
		{
		 .op = &rdsr_sdr_op,
		 .cfg = &rdsr_cfg,
		 },
		{
		 .op = &wrcr2_sdr_op,
		 .cfg = &wrcr2_cfg,
		 },
	};

	rdcr2_sdr_op.data.buf.out = &cfg2_reg;
	rdsr_sdr_op.data.buf.out = &status;
	wrcr2_sdr_op.data.buf.in = &cfg2_reg;


	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	if (!enable_operators(q, ops, ARRAY_SIZE(ops)))
		return -1;

	mcr2 = qspi_readl(q, base + QUADSPI_MCR);

	/* Enable the module */
	qspi_writel(q, mcr2 & ~QUADSPI_MCR_MDIS_MASK, base + QUADSPI_MCR);

	if (qspi_read_reg(q, &rdcr2_sdr_op, rdcr2_cfg))
		return -1;

	cfg2_reg &= ~QUADSPI_CFG2_OPI_MASK;
	cfg2_reg |= QUADSPI_CFG2_DTR_OPI_ENABLED;

	/* Enable write */
	if (qspi_write_reg(q, &wren_sdr_op, wren_cfg))
		return -1;

	/* Wait write enabled */
	while (!(status & FLASH_STATUS_WEL)) {
		if (qspi_read_reg(q, &rdsr_sdr_op, rdsr_cfg))
			return -1;
	}

	if (qspi_write_reg(q, &wrcr2_sdr_op, wrcr2_cfg))
		return -1;

	qspi_writel(q, mcr2, base + QUADSPI_MCR);

	disable_operators(q, ops, ARRAY_SIZE(ops));
	udelay(400);

	return 0;
}

static void dllcra_bypass(struct fsl_qspi *q, u32 dllmask)
{
	u32 dllcra;
	void __iomem *base = q->iobase;

	dllmask &= QUADSPI_DLLCR_MASK;

	dllcra = QUADSPI_DLLCR_SLV_EN_MASK | QUADSPI_DLLCR_SLV_BYPASS_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= dllmask;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_SLV_BYPASS_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_SLV_UPD_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_DLLA_LOCK_MASK))
		;

	dllcra &= ~QUADSPI_DLLCR_SLV_UPD_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);
}

static void dllcra_manual(struct fsl_qspi *q, u32 dllmask)
{
	u32 dllcra;
	void __iomem *base = q->iobase;

	dllmask &= QUADSPI_DLLCR_MASK;

	dllcra = QUADSPI_DLLCR_SLV_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= dllmask;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_DLLEN_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_DLLA_LOCK_MASK))
		;

	dllcra &= ~QUADSPI_DLLCR_SLV_AUTO_UPDT_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra &= ~QUADSPI_DLLCR_SLV_BYPASS_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra = qspi_readl(q, base + QUADSPI_DLLCRA);
	dllcra |= QUADSPI_DLLCR_SLV_UPD_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_SLVA_LOCK_MASK))
		;
}

static void dllcra_auto(struct fsl_qspi *q, u32 dllmask)
{
	u32 dllcra;
	void __iomem *base = q->iobase;

	dllmask &= QUADSPI_DLLCR_MASK;

	dllcra = QUADSPI_DLLCR_SLV_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= dllmask;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_SLV_AUTO_UPDT_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_DLLEN_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_DLLA_LOCK_MASK))
		;

	dllcra = qspi_readl(q, base + QUADSPI_DLLCRA);
	dllcra &= ~QUADSPI_DLLCR_SLV_BYPASS_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_SLV_UPD_EN_MASK;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);
	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_SLVA_LOCK_MASK))
		;
}

static int program_dllcra(struct fsl_qspi *q, u32 dllcra)
{
	u32 bypass = (dllcra >> QUADSPI_DLLCR_SLV_BYPASS_SHIFT) & BIT(0);
	u32 slven = (dllcra >> QUADSPI_DLLCR_SLV_EN_SHIFT) & BIT(0);
	u32 autoupd = (dllcra >> QUADSPI_DLLCR_SLV_AUTO_UPDT_SHIFT) & BIT(0);

	/* Bypass mode */
	if (slven && bypass && !autoupd) {
		dllcra_bypass(q, dllcra);
		return 0;
	}

	/* Manual mode */
	if (slven && !bypass && !autoupd) {
		dllcra_manual(q, dllcra);
		return 0;
	}

	/* Auto update mode */
	if (slven && !bypass && autoupd) {
		dllcra_auto(q, dllcra);
		return 0;
	}

	return -1;
}

static struct qspi_config ddr_config = {
	.mcr = S32GEN1_QUADSPI_MCR_END_CFD_LE_MASK |
	    S32GEN1_QUADSPI_MCR_DQS_EN_MASK |
	    QUADSPI_MCR_DDR_EN_MASK |
	    QUADSPI_MCR_ISD2FA_EN_MASK |
	    QUADSPI_MCR_ISD3FA_EN_MASK |
	    QUADSPI_MCR_ISD2FB_EN_MASK |
	    QUADSPI_MCR_ISD3FB_EN_MASK |
	    QUADSPI_MCR_DQS_EXTERNAL_MASK,
	.flshcr = S32GEN1_QUADSPI_FLSHCR_TCSS_MASK(3) |
	    S32GEN1_QUADSPI_FLSHCR_TCHS_MASK(3) |
	    S32GEN1_QUADSPI_FLSHCR_TDH_MASK(1),
	.dllcr = QUADSPI_DLLCR_SLV_EN_MASK |
	    QUADSPI_DLLCR_SLV_AUTO_UPDT_EN_MASK |
	    QUADSPI_DLLCR_DLLRES_N_MASK(8) |
	    QUADSPI_DLLCR_DLL_REFCNTR_N_MASK(2) |
	    QUADSPI_DLLCR_DLLEN_EN_MASK,
	.sfacr = QUADSPI_SFACR_BSWAP_EN,
	.smpr = QUADSPI_SMPR_DLLFSMPFA_NTH(4) |
		QUADSPI_SMPR_DLLFSMPFB_NTH(4),
	.dlcr = QUADSPI_DLCR_RESERVED_MASK |
	    QUADSPI_DLCR_DLP_SEL_FA_MASK(1) |
	    QUADSPI_DLCR_DLP_SEL_FB_MASK(1),
	.flash1_size = 0x20000000,
	.flash2_size = 0x20000000,
	.dlpr = QUADSPI_DLPR_RESET_VALUE,
};

static int enable_ddr(struct fsl_qspi *q)
{
	u32 mcr;
	int ret;
	void __iomem *base = q->iobase;

	if (q->ddr_mode)
		return 0;

	if (memory_enable_ddr(q))
		return -1;

	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	if (q->clk_rate == QUADSPI_S32GEN1_HIGH_FREQUENCY_VALUE)
		ddr_config.dllcr |= QUADSPI_DLLCR_FREQEN_EN_MASK;

	/* Disable the module */
	mcr = qspi_readl(q, base + QUADSPI_MCR);
	mcr |= QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	mcr |= ddr_config.mcr;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	qspi_writel(q, ddr_config.flshcr, base + QUADSPI_FLSHCR);
	qspi_writel(q, ddr_config.sfacr, base + QUADSPI_SFACR);
	qspi_writel(q, ddr_config.smpr, base + QUADSPI_SMPR);
	qspi_writel(q, ddr_config.dlcr, base + QUADSPI_DLCR);

	qspi_writel(q, ddr_config.dlpr, base + QUADSPI_DLPR);
	qspi_writel(q, 0x0, base + QUADSPI_RBCT);

	/* Init AHB interface - 1024 bytes */
	qspi_writel(q, QUADSPI_BUF3CR_ALLMST_MASK |
		     (0x80 << S32GEN1_QUADSPI_BUF3CR_ADATSZ_SHIFT),
			 base + QUADSPI_BUF3CR);

	/* We only use the buffer3 */
	qspi_writel(q, 0, base + QUADSPI_BUF0IND);
	qspi_writel(q, 0, base + QUADSPI_BUF1IND);
	qspi_writel(q, 0, base + QUADSPI_BUF2IND);

	qspi_writel(q, ddr_config.flash1_size, base + QUADSPI_SFA1AD);
	qspi_writel(q, ddr_config.flash2_size, base + QUADSPI_SFA2AD);
	qspi_writel(q, ddr_config.flash1_size, base + QUADSPI_SFB1AD);
	qspi_writel(q, ddr_config.flash2_size, base + QUADSPI_SFB2AD);

	/* Enable the module */
	mcr = qspi_readl(q, base + QUADSPI_MCR);
	mcr &= ~QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	ret = program_dllcra(q, ddr_config.dllcr);
	if (ret)
		return ret;

	q->ddr_mode = true;
	q->num_pads = 8;

	return 0;
}

static int memory_reset(struct fsl_qspi *q)
{
	u8 rsten_cfg, rst_cfg;
	u32 mcr2;
	void __iomem *base = q->iobase;

	struct qspi_op ops[] = {
		{
		 .op = &rsten_ddr_op,
		 .cfg = &rsten_cfg,
		 },
		{
		 .op = &rst_ddr_op,
		 .cfg = &rst_cfg,
		 },
	};

	rsten_ddr_op.cmd.buswidth = q->num_pads;
	rst_ddr_op.cmd.buswidth = q->num_pads;

	mcr2 = qspi_readl(q, base + QUADSPI_MCR);
	qspi_writel(q, mcr2 & ~QUADSPI_MCR_MDIS_MASK, base + QUADSPI_MCR);

	if (!enable_operators(q, ops, ARRAY_SIZE(ops)))
		return -1;

	if (qspi_write_reg(q, &rsten_ddr_op, rsten_cfg))
		return -1;

	if (qspi_write_reg(q, &rst_ddr_op, rst_cfg))
		return -1;

	/* Reset recovery time after a read operation */
	udelay(40);
	disable_operators(q, ops, ARRAY_SIZE(ops));

	return 0;
}

void reset_bootrom_settings(struct fsl_qspi *q)
{
	u32 bfgencr, lutid, lutaddr;
	u32 lut;
	u32 instr0;
	void __iomem *base = q->iobase;

	/* Read the configuration left by BootROM */
	bfgencr = qspi_readl(q, base + QUADSPI_BFGENCR);
	lutid = (bfgencr & QUADSPI_BFGENCR_SEQID(0xF)) >>
		QUADSPI_BFGENCR_SEQID_SHIFT;
	lutaddr = lutid * S32GEN1_LUTS_PER_CONFIG;

	lut = qspi_readl(q, base + QUADSPI_LUT(lutaddr));

	/* Not configured */
	if (!lut)
		return;

	q->num_pads = (1 << LUT2PAD0(lut));
	instr0 = LUT2INSTR0(lut);

	if (instr0 == LUT_CMD_DDR)
		q->ddr_mode = true;
	else
		q->ddr_mode = false;

	memory_reset(q);
}

int enable_spi(struct fsl_qspi *q, bool force)
{
	u32 mcr;
	void __iomem *base = q->iobase;

	if (!q->ddr_mode && !force)
		return 0;

	if (q->ddr_mode) {
		if (memory_reset(q))
			return -1;
	}

	/* Controller isn't busy */
	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	/* Disable the module */
	mcr = qspi_readl(q, base + QUADSPI_MCR);
	mcr |= QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	mcr = S32GEN1_QUADSPI_MCR_DQS_EN_MASK |
	    QUADSPI_MCR_MDIS_MASK | QUADSPI_MCR_ISD2FA_EN_MASK |
	    QUADSPI_MCR_ISD3FA_EN_MASK | QUADSPI_MCR_ISD2FB_EN_MASK |
		QUADSPI_MCR_ISD3FB_EN_MASK | S32GEN1_QUADSPI_MCR_END_CFD_LE_MASK;

	qspi_writel(q, mcr, base + QUADSPI_MCR);

	mcr |= QUADSPI_MCR_DQS_PAD_LOOPBACK_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	qspi_writel(q, S32GEN1_QUADSPI_FLSHCR_TDH_MASK(0) |
			S32GEN1_QUADSPI_FLSHCR_TCHS_MASK(3) |
			S32GEN1_QUADSPI_FLSHCR_TCSS_MASK(3), base + QUADSPI_FLSHCR);

	qspi_writel(q, QUADSPI_SMPR_DLLFSMPFA_NTH(0) |
		     QUADSPI_SMPR_FSPHS_MASK, base + QUADSPI_SMPR);

	qspi_writel(q, 0x0, base + QUADSPI_SFACR);

	mcr = qspi_readl(q, base + QUADSPI_MCR);
	mcr &= ~QUADSPI_MCR_DLPEN_MASK;

	mcr &= ~QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	program_dllcra(q, QUADSPI_DLLCR_SLV_BYPASS_EN_MASK |
			QUADSPI_DLLCR_SLV_EN_MASK);

	q->ddr_mode = false;
	q->num_pads = 1;

	return 0;
}

int qspi_read_mem(struct fsl_qspi *q,
			 const struct spi_mem_op *op, u8 lut_cfg)
{
	u32 mcr_reg;
	void __iomem *base = q->iobase;
	struct timespec64 start, end, duration;
	u64 mb_int, mb_frac;
	u32 us_passed, rem;

	if (!q->ahb_addr) {
		q->ahb_addr = ioremap_cache(QUADSPI_AHB_BASE,
				q->devtype_data->ahb_buf_size);
		if (!q->ahb_addr)
			return -ENOMEM;
	}

	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;
	mcr_reg = clear_fifos(q);
	qspi_writel(q, lut_cfg << QUADSPI_BFGENCR_SEQID_SHIFT,
			base + QUADSPI_BFGENCR);

	__inval_dcache_area(q->ahb_addr + op->addr.val, op->data.nbytes);

	/* Read out the data directly from the AHB buffer. */
	ktime_get_ts64(&start);
	memcpy_fromio(op->data.buf.in, q->ahb_addr + op->addr.val,
		      op->data.nbytes);
	ktime_get_ts64(&end);

	duration = timespec64_sub(end, start);
	us_passed = duration.tv_sec * 1000000 +
		(duration.tv_nsec / NSEC_PER_USEC);

	if (us_passed > 0) {
		mb_int = div_u64_rem(op->data.nbytes, us_passed, &rem);
		mb_frac = div64_u64(rem * 1000, us_passed);
		dev_info(q->dev, "%u bytes read in %u us (%llu.%llu MB/s)\n",
				op->data.nbytes, us_passed, mb_int, mb_frac);
	}

	qspi_writel(q, mcr_reg, base + QUADSPI_MCR);

	return 0;
}

static void qspi_invalidate_ahb(struct fsl_qspi *q)
{
	u32 mcr;
	u32 reset_mask = QUADSPI_MCR_SWRSTHD_MASK | QUADSPI_MCR_SWRSTSD_MASK;
	void __iomem *base = q->iobase;

	mcr = qspi_readl(q, base + QUADSPI_MCR);

	mcr &= ~QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	mcr = qspi_readl(q, base + QUADSPI_MCR);
	mcr |= reset_mask;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	mcr = qspi_readl(q, base + QUADSPI_MCR) | QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	mcr &= ~(reset_mask);
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	mcr = qspi_readl(q, base + QUADSPI_MCR);
	mcr &= ~QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);
}

int s32gen1_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct fsl_qspi *q = spi_controller_get_devdata(mem->spi->master);
	u8 lut_cfg;
	bool enabled = false;

	lut_cfg = LUT_INVALID_INDEX;
	if (!s32gen1_supports_op(mem, op))
		return -1;

	enabled = q->lut_configs[op->cmd.opcode].enabled;
	if (!enabled)
		return -1;

	lut_cfg = q->lut_configs[op->cmd.opcode].index;
	if (lut_cfg == LUT_INVALID_INDEX)
		return -1;

	/* Register and memory write */
	if (op->data.dir == SPI_MEM_DATA_OUT) {
		q->flags &= ~QUADSPI_FLAG_PREV_READ_MEM;
		return qspi_write_reg(q, op, lut_cfg);
	}

	/* Memory operation */
	if (op->memop) {
		if (!(q->flags & QUADSPI_FLAG_PREV_READ_MEM))
			qspi_invalidate_ahb(q);

		q->flags |= QUADSPI_FLAG_PREV_READ_MEM;
		return qspi_read_mem(q, op, lut_cfg);
	}

	q->flags &= ~QUADSPI_FLAG_PREV_READ_MEM;

	/* Read Register */
	return qspi_read_reg(q, op, lut_cfg);
}

bool s32gen1_supports_op(struct spi_mem *mem,
				const struct spi_mem_op *op)
{
	struct fsl_qspi *q = spi_controller_get_devdata(mem->spi->master);

	if (op->memop)
		if (op->data.buf.in == NULL &&
				op->data.buf.out == NULL)
			return -1;

	/* Enable DTR for 8D-8D-8D mode only */
	if (op->cmd.buswidth == 8 && op->addr.buswidth == 8 &&
	    op->dummy.buswidth == 8 && op->data.buswidth == 8) {
		if (!op->memop)
			return true;

		if (enable_ddr(q))
			return -1;
	} else {
		if (enable_spi(q, false))
			return -1;
	}

	return enable_op(q, op);
}

int s32gen1_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	return 0;
}
