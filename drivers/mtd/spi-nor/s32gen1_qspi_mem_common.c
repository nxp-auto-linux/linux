// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 */

#include "fsl-quadspi.h"
#include <linux/mtd/spi-nor.h>
#include <linux/spi/spi-mem.h>

#define SPI_NOR_MAX_ID_LEN        0x06
#define SPI_NOR_MIN_ID_LEN        0x03

#define MT35XU512ABA_ID	0x2c5b1a
#define MX25UW51245G_ID 0xc2813a

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

static struct spi_mem_op read_id_op =
SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_RDID, 1),
	   SPI_MEM_OP_NO_ADDR,
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_DATA_IN(SPI_NOR_MAX_ID_LEN, NULL, 1));

int s32gen1_mem_reset(struct fsl_qspi *q)
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

	if (!s32gen1_enable_operators(q, ops, ARRAY_SIZE(ops)))
		return -1;

	if (s32gen1_mem_exec_write_op(q, &rsten_ddr_op, rsten_cfg))
		return -1;

	if (s32gen1_mem_exec_write_op(q, &rst_ddr_op, rst_cfg))
		return -1;

	/* Reset recovery time after a read operation */
	usleep_range(40, 50);
	s32gen1_disable_operators(q, ops, ARRAY_SIZE(ops));

	return 0;
}

void s32gen1_reset_bootrom_settings(struct fsl_qspi *q)
{
	u32 bfgencr, lutid, lutaddr;
	u32 lut;
	u32 instr0;
	void __iomem *base = q->iobase;

	/* Read the configuration left by BootROM */
	bfgencr = qspi_readl(q, base + QUADSPI_BFGENCR);
	lutid = (bfgencr & QUADSPI_BFGENCR_SEQID_MASK) >>
		QUADSPI_BFGENCR_SEQID_SHIFT;
	lutaddr = lutid * LUTS_PER_CONFIG;

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

	s32gen1_mem_reset(q);
}

bool s32gen1_enable_operators(struct fsl_qspi *q, struct qspi_op *ops,
			      size_t n_ops)
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
		res = s32gen1_enable_op(q, op);
		*cfg = q->lut_configs[op->cmd.opcode].index;

		if (!res || !q->lut_configs[op->cmd.opcode].enabled)
			return false;
	}

	return true;
}

void s32gen1_disable_operators(struct fsl_qspi *q, struct qspi_op *ops,
			       size_t n_ops)
{
	size_t i;
	const struct spi_mem_op *op;

	for (i = 0; i < n_ops; i++) {
		op = ops[i].op;

		q->lut_configs[op->cmd.opcode].enabled = false;
	}
}

int s32gen1_mem_enable_ddr(struct fsl_qspi *q,
			   struct qspi_config *config)
{
	void __iomem *base = q->iobase;
	u8 id[SPI_NOR_MAX_ID_LEN];
	u8 read_id_cfg = 0x0;
	u64 jedec_id = 0;
	u32 mcr2;
	u8 byte;
	int i;

	struct qspi_op ops[] = {
		{
		 .op = &read_id_op,
		 .cfg = &read_id_cfg,
		 },
	};

	read_id_op.data.buf.in = id;

	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	if (!s32gen1_enable_operators(q, ops, ARRAY_SIZE(ops)))
		return -1;

	mcr2 = qspi_readl(q, base + QUADSPI_MCR);

	/* Enable the module */
	qspi_writel(q, mcr2 & ~QUADSPI_MCR_MDIS_MASK, base + QUADSPI_MCR);

	if (s32gen1_mem_exec_read_op(q, &read_id_op, read_id_cfg))
		return -1;

	s32gen1_disable_operators(q, ops, ARRAY_SIZE(ops));
	usleep_range(400, 450);

	for (i = 0; i < SPI_NOR_MIN_ID_LEN; i++) {
		byte = (SPI_NOR_MIN_ID_LEN - 1) - i;
		jedec_id |= id[byte] << (8 * i);
	}

	switch (jedec_id) {
	case MX25UW51245G_ID:
		macronix_get_ddr_config(config);
		return macronix_mem_enable_ddr(q);
	case MT35XU512ABA_ID:
		micron_get_ddr_config(config);
		return micron_mem_enable_ddr(q);
	default:
		return -EINVAL;
	}
}
