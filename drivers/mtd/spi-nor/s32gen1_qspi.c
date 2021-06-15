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
#include "fsl-quadspi.h"

#define LUT_INVALID_INDEX -1
#define LUT_STOP_CMD 0x00

#define QUADSPI_LUT(x)	(QUADSPI_LUT_BASE + (x) * 4)

#define QUADSPI_S32GEN1_HIGH_FREQUENCY_VALUE	200000000

static u32 clear_fifos(struct fsl_qspi *q)
{
	u32 mcr_reg;

	mcr_reg = qspi_readl(q, q->iobase + QUADSPI_MCR);

	/* Clear TX & RX fifos */
	qspi_writel(q, mcr_reg | QUADSPI_MCR_CLR_RXF_MASK |
			QUADSPI_MCR_CLR_TXF_MASK |
			QUADSPI_MCR_RESERVED_MASK | QUADSPI_MCR_END_CFD_LE,
			q->iobase + QUADSPI_MCR);
	return mcr_reg;
}

int s32gen1_mem_exec_write_op(struct fsl_qspi *q,
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

int s32gen1_mem_exec_read_op(struct fsl_qspi *q,
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
		data = qspi_readl(q, base + QUADSPI_RBDR + i * 4);

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
		if (op->data.nbytes > RX_BUFFER_SIZE)
			lut |= OPRND0(RX_BUFFER_SIZE);
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
		lut = OPRND0(op->dummy.nbytes * 8 /
			     op->dummy.buswidth) |
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

	lutaddr = index * LUTS_PER_CONFIG;

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

bool s32gen1_enable_op(struct fsl_qspi *q, const struct spi_mem_op *op)
{
	u8 lut_index;
	u8 opcode = op->cmd.opcode;

	if (q->luts_next_config >= MAX_LUTS_CONFIGS)
		return false;

	if (!q->lut_configs[opcode].enabled) {
		if (!add_op_to_lutdb(q, op, &lut_index))
			return false;

		set_lut(q, lut_index, opcode);
		q->lut_configs[opcode].enabled = true;
	}

	return true;
}

static void dllcra_bypass(struct fsl_qspi *q, u32 dllmask)
{
	u32 dllcra;
	void __iomem *base = q->iobase;

	dllmask &= QUADSPI_DLLCR_MASK;

	dllcra = QUADSPI_DLLCR_SLV_EN | QUADSPI_DLLCR_SLV_BYPASS_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= dllmask;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_SLV_BYPASS_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_SLV_UPD_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_DLLA_LOCK_MASK))
		;

	dllcra &= ~QUADSPI_DLLCR_SLV_UPD_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);
}

static void dllcra_manual(struct fsl_qspi *q, u32 dllmask)
{
	u32 dllcra;
	void __iomem *base = q->iobase;

	dllmask &= QUADSPI_DLLCR_MASK;

	dllcra = QUADSPI_DLLCR_SLV_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= dllmask;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_DLLEN_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_DLLA_LOCK_MASK))
		;

	dllcra &= ~QUADSPI_DLLCR_SLV_AUTO_UPDT_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra &= ~QUADSPI_DLLCR_SLV_BYPASS_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra = qspi_readl(q, base + QUADSPI_DLLCRA);
	dllcra |= QUADSPI_DLLCR_SLV_UPD_EN;
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

	dllcra = QUADSPI_DLLCR_SLV_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= dllmask;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_SLV_AUTO_UPDT_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_DLLEN_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_DLLA_LOCK_MASK))
		;

	dllcra = qspi_readl(q, base + QUADSPI_DLLCRA);
	dllcra &= ~QUADSPI_DLLCR_SLV_BYPASS_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_SLV_UPD_EN;
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

static struct qspi_config ddr_config;

static int enable_ddr(struct fsl_qspi *q)
{
	u32 mcr;
	int ret;
	void __iomem *base = q->iobase;

	if (q->ddr_mode)
		return 0;

#ifdef CONFIG_SPI_FLASH_MACRONIX
	if (s32gen1_mem_enable_ddr(q, &ddr_config))
		return -1;
#endif

	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	if (q->clk_rate == QUADSPI_S32GEN1_HIGH_FREQUENCY_VALUE)
		ddr_config.dllcr |= QUADSPI_DLLCR_FREQEN_EN;

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
		     (0x80 << QUADSPI_BUF3CR_ADATSZ_SHIFT),
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

int s32gen1_enable_spi(struct fsl_qspi *q, bool force)
{
	u32 mcr;
	void __iomem *base = q->iobase;

	if (!q->ddr_mode && !force)
		return 0;

#ifdef CONFIG_SPI_FLASH_MACRONIX
	if (q->ddr_mode) {
		if (s32gen1_mem_reset(q))
			return -1;
	}
#endif

	/* Controller isn't busy */
	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	/* Disable the module */
	mcr = qspi_readl(q, base + QUADSPI_MCR);
	mcr |= QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	mcr = QUADSPI_MCR_DQS_EN |
	    QUADSPI_MCR_MDIS_MASK | QUADSPI_MCR_ISD2FA_EN |
	    QUADSPI_MCR_ISD3FA_EN | QUADSPI_MCR_ISD2FB_EN |
		QUADSPI_MCR_ISD3FB_EN | QUADSPI_MCR_END_CFD_LE;

	qspi_writel(q, mcr, base + QUADSPI_MCR);

	mcr |= QUADSPI_MCR_DQS_PAD_LOOPBACK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	qspi_writel(q, QUADSPI_FLSHCR_TDH(0) | QUADSPI_FLSHCR_TCHS(3) |
			QUADSPI_FLSHCR_TCSS(3), base + QUADSPI_FLSHCR);

	qspi_writel(q, QUADSPI_SMPR_DLLFSMPFA_NTH(0) |
		     QUADSPI_SMPR_FSPHS_MASK, base + QUADSPI_SMPR);

	qspi_writel(q, 0x0, base + QUADSPI_SFACR);

	mcr = qspi_readl(q, base + QUADSPI_MCR);
	mcr &= ~QUADSPI_MCR_DLPEN_MASK;

	mcr &= ~QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	program_dllcra(q, QUADSPI_DLLCR_SLV_BYPASS_EN | QUADSPI_DLLCR_SLV_EN);

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

	/* Map AHB Buffer at first read operation */
	if (!q->ahb_addr) {
		q->ahb_addr = ioremap_cache(QUADSPI_AHB_BASE, QUADSPI_AHB_SIZE);
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
	memcpy(op->data.buf.in, q->ahb_addr + op->addr.val,
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

static int s32gen1_qspi_read_write_reg(struct spi_nor *nor,
		struct spi_mem_op *op, void *buf)
{
	if (nor->erase_opcode != op->cmd.opcode) {
		if (op->data.dir == SPI_MEM_DATA_IN)
			op->data.buf.in = buf;
		else
			op->data.buf.out = buf;
	} else {
		if (op->data.nbytes == 2)
			op->addr.val = swab16(*(u32 *)buf);
		if (op->data.nbytes == 4)
			op->addr.val = swab32(*(u32 *)buf);

		op->addr.buswidth = op->cmd.buswidth;
		op->addr.nbytes = op->data.nbytes;
		op->data.nbytes = 0;
	}

	return s32gen1_exec_op(nor, op);
}

int s32gen1_qspi_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct spi_mem_op op = SPI_MEM_OP(SPI_MEM_OP_CMD(opcode, 1),
			SPI_MEM_OP_NO_ADDR, SPI_MEM_OP_NO_DUMMY,
			SPI_MEM_OP_DATA_IN(len, NULL, 1));

	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->reg_proto);
	op.memop = false;

	return s32gen1_qspi_read_write_reg(nor, &op, buf);
}

int s32gen1_qspi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct spi_mem_op op = SPI_MEM_OP(SPI_MEM_OP_CMD(opcode, 1),
			SPI_MEM_OP_NO_ADDR,
			SPI_MEM_OP_NO_DUMMY,
			SPI_MEM_OP_DATA_OUT(len, NULL, 1));
	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->reg_proto);
	op.memop = false;

	return s32gen1_qspi_read_write_reg(nor, &op, buf);
}

ssize_t s32gen1_qspi_read(struct spi_nor *nor, loff_t from,
			     size_t len, u_char *buf)
{
	struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_CMD(nor->read_opcode, 1),
				   SPI_MEM_OP_ADDR(nor->addr_width, from, 1),
				   SPI_MEM_OP_DUMMY(nor->read_dummy, 1),
				   SPI_MEM_OP_DATA_IN(len, buf, 1));
	size_t remaining = len;
	int ret;

	op.memop = true;

	/* get transfer protocols. */
	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->read_proto);
	op.addr.buswidth = spi_nor_get_protocol_addr_nbits(nor->read_proto);
	op.dummy.buswidth = op.addr.buswidth;
	op.data.buswidth = spi_nor_get_protocol_data_nbits(nor->read_proto);

	/* convert the dummy cycles to the number of bytes */
	op.dummy.nbytes = (nor->read_dummy * op.dummy.buswidth) / 8;

	while (remaining) {
		op.data.nbytes = remaining < UINT_MAX ? remaining : UINT_MAX;

		ret = s32gen1_exec_op(nor, &op);
		if (ret)
			return ret;

		op.addr.val += op.data.nbytes;
		remaining -= op.data.nbytes;
		op.data.buf.in += op.data.nbytes;
	}

	return len;
}

ssize_t s32gen1_qspi_write(struct spi_nor *nor, loff_t to,
			      size_t len, const u_char *buf)
{
	struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_CMD(nor->program_opcode, 1),
				   SPI_MEM_OP_ADDR(nor->addr_width, to, 1),
				   SPI_MEM_OP_NO_DUMMY,
				   SPI_MEM_OP_DATA_OUT(len, buf, 1));
	size_t remaining = len;
	int ret;

	/* get transfer protocols. */
	op.memop = true;
	op.cmd.buswidth = spi_nor_get_protocol_inst_nbits(nor->write_proto);
	op.addr.buswidth = spi_nor_get_protocol_addr_nbits(nor->write_proto);
	op.data.buswidth = spi_nor_get_protocol_data_nbits(nor->write_proto);

	if (nor->program_opcode == SPINOR_OP_AAI_WP && nor->sst_write_second)
		op.addr.nbytes = 0;

	while (remaining) {
		op.data.nbytes = remaining < UINT_MAX ? remaining : UINT_MAX;

		ret = s32gen1_exec_op(nor, &op);
		if (ret)
			return ret;

		op.addr.val += op.data.nbytes;
		remaining -= op.data.nbytes;
		op.data.buf.out += op.data.nbytes;
	}

	return len;
}

int s32gen1_exec_op(struct spi_nor *nor, const struct spi_mem_op *op)
{
	u8 lut_cfg = LUT_INVALID_INDEX;
	bool enabled = false;
	struct fsl_qspi *q = nor->priv;
	int ret;

	if (!s32gen1_supports_op(nor, op))
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
		return s32gen1_mem_exec_write_op(q, op, lut_cfg);
	}

	/* Memory operation */
	if (op->memop) {
		if (!(q->flags & QUADSPI_FLAG_PREV_READ_MEM))
			qspi_invalidate_ahb(q);

		q->flags |= QUADSPI_FLAG_PREV_READ_MEM;
		ret = qspi_read_mem(q, op, lut_cfg);
		/*
		 * On S32R45EVB platform, the Macronix Flash memory
		 * does not have the 'RESET_B' (functional reset) signal wired,
		 * but only POR (power on reset).
		 * Therefore, in order to prevent an improper state on the
		 * Macronix Flash after any functional reset, we enter SPI MODE
		 * after any DTR-OPI read operation.
		 */
		if (q->no_functional_reset)
			s32gen1_enable_spi(q, true);

		return ret;
	}

	q->flags &= ~QUADSPI_FLAG_PREV_READ_MEM;

	/* Read Register */
	return s32gen1_mem_exec_read_op(q, op, lut_cfg);
}

bool s32gen1_supports_op(struct spi_nor *nor,
				const struct spi_mem_op *op)
{
	struct fsl_qspi *q = nor->priv;

	/* Enable DTR for 8D-8D-8D mode only */
	if (op->cmd.buswidth == 8 && op->addr.buswidth == 8 &&
	    op->dummy.buswidth == 8 && op->data.buswidth == 8) {
		if (enable_ddr(q))
			return -1;
	} else {
		if (s32gen1_enable_spi(q, false))
			return -1;
	}

	return s32gen1_enable_op(q, op);
}
