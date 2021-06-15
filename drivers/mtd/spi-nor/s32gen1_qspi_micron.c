// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 */

#include "fsl-quadspi.h"
#include <linux/mtd/spi-nor.h>
#include <linux/spi/spi-mem.h>

#define SPINOR_OP_MT_WR_ANY_REG	0x81	/* Write volatile register */
#define SPINOR_REG_MT_CFR0V	0x00	/* For setting octal DTR mode */
#define SPINOR_REG_MT_CFR1V	0x01	/* For setting dummy cycles */
#define SPINOR_MT_OCT_DTR	0xe7	/* Enable Octal DTR. */

static struct spi_mem_op wren_sdr_op =
SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_WREN, 1),
	   SPI_MEM_OP_NO_ADDR,
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_DATA_IN(0, NULL, 1));

static struct spi_mem_op micron_io_mode =
SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_MT_WR_ANY_REG, 1),
	   SPI_MEM_OP_ADDR(3, SPINOR_REG_MT_CFR0V, 1),
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_DATA_OUT(1, NULL, 1));

static struct spi_mem_op micron_set_dummy =
SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_MT_WR_ANY_REG, 1),
	   SPI_MEM_OP_ADDR(3, SPINOR_REG_MT_CFR1V, 1),
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_DATA_OUT(1, NULL, 1));

static const struct qspi_config ddr_config_micron = {
	.mcr = QUADSPI_MCR_END_CFG_MASK |
		QUADSPI_MCR_DQS_EN |
		QUADSPI_MCR_DDR_EN_MASK |
		QUADSPI_MCR_ISD2FA_EN |
		QUADSPI_MCR_ISD3FA_EN |
		QUADSPI_MCR_ISD2FB_EN |
		QUADSPI_MCR_ISD3FB_EN |
		QUADSPI_MCR_DQS_LOOPBACK,
	.flshcr = QUADSPI_FLSHCR_TCSS(3) |
		QUADSPI_FLSHCR_TCHS(3) |
		QUADSPI_FLSHCR_TDH(1),
	.dllcr = QUADSPI_DLLCR_SLV_EN |
		QUADSPI_DLLCR_SLV_DLY_COARSE_N(0x607) |
		QUADSPI_DLLCR_DLLRES_N(8) |
		QUADSPI_DLLCR_DLL_REFCNTR_N(2) |
		QUADSPI_DLLCR_FREQEN_EN,
	.sfacr = 0,
	.smpr = QUADSPI_SMPR_DLLFSMPFA_NTH(4) |
		QUADSPI_SMPR_DLLFSMPFB_NTH(4),
	.dlcr = QUADSPI_DLCR_RESERVED_MASK |
		QUADSPI_DLCR_DLP_SEL_FA(1) |
		QUADSPI_DLCR_DLP_SEL_FB(1),
	.flash1_size = 0x20000000,
	.flash2_size = 0x20000000,
	.dlpr = QUADSPI_DLPR_RESET_VALUE,
};

void micron_get_ddr_config(struct qspi_config *ddr_config)
{
	memcpy(ddr_config, &ddr_config_micron, sizeof(ddr_config_micron));
}

int micron_mem_enable_ddr(struct fsl_qspi *q)
{
	void __iomem *base = q->iobase;
	u8 wren_cfg, micron_set_dummy_cfg, micron_io_mode_cfg;
	u8 dummy = 20;
	u8 io_mode = SPINOR_MT_OCT_DTR;
	u32 mcr2;

	struct qspi_op ops[] = {
		{
		 .op = &wren_sdr_op,
		 .cfg = &wren_cfg,
		 },
		{
		 .op = &micron_set_dummy,
		 .cfg = &micron_set_dummy_cfg,
		 },
		{
		 .op = &micron_io_mode,
		 .cfg = &micron_io_mode_cfg,
		 },
	};

	micron_set_dummy.data.buf.out = &dummy;
	micron_io_mode.data.buf.out = &io_mode;

	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY_MASK)
		;

	if (!s32gen1_enable_operators(q, ops, ARRAY_SIZE(ops)))
		return -1;

	mcr2 = qspi_readl(q, base + QUADSPI_MCR);

	/* Enable the module */
	qspi_writel(q, mcr2 & ~QUADSPI_MCR_MDIS_MASK, base + QUADSPI_MCR);

	/* Enable write */
	if (s32gen1_mem_exec_write_op(q, &wren_sdr_op, wren_cfg))
		return -1;

	if (s32gen1_mem_exec_write_op(q, &micron_set_dummy,
				      micron_set_dummy_cfg))
		return -1;

	/* Enable write */
	if (s32gen1_mem_exec_write_op(q, &wren_sdr_op, wren_cfg))
		return -1;

	if (s32gen1_mem_exec_write_op(q, &micron_io_mode, micron_io_mode_cfg))
		return -1;

	qspi_writel(q, mcr2, base + QUADSPI_MCR);

	s32gen1_disable_operators(q, ops, ARRAY_SIZE(ops));
	usleep_range(400, 450);

	return 0;
}

