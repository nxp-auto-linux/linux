// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 */

#include "fsl-quadspi.h"
#include <linux/mtd/spi-nor.h>
#include <linux/spi/spi-mem.h>

#define QSPI_CFG2_OPI_MASK		(0x03)
#define QSPI_CFG2_STR_OPI_ENABLED	BIT(0)
#define QSPI_CFG2_DTR_OPI_ENABLED	BIT(1)

/* JESD216D.01 */
#define SPINOR_OP_RDCR2		0x71
#define SPINOR_OP_WRCR2		0x72

static struct spi_mem_op rdcr2_sdr_op =
SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_RDCR2, 1),
	   SPI_MEM_OP_ADDR(0x4, 0x0, 1),
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_DATA_IN(1, NULL, 1));

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

static struct spi_mem_op wren_sdr_op =
SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_WREN, 1),
	   SPI_MEM_OP_NO_ADDR,
	   SPI_MEM_OP_NO_DUMMY,
	   SPI_MEM_OP_DATA_IN(0, NULL, 1));

static const struct qspi_config ddr_config_macronix = {
	.mcr = QUADSPI_MCR_END_CFG_MASK |
		QUADSPI_MCR_DQS_EN |
		QUADSPI_MCR_DDR_EN_MASK |
		QUADSPI_MCR_ISD2FA_EN |
		QUADSPI_MCR_ISD3FA_EN |
		QUADSPI_MCR_ISD2FB_EN |
		QUADSPI_MCR_ISD3FB_EN |
		QUADSPI_MCR_DQS_EXTERNAL,
	.flshcr = QUADSPI_FLSHCR_TCSS(3) |
		QUADSPI_FLSHCR_TCHS(3) |
		QUADSPI_FLSHCR_TDH(1),
	.dllcr = QUADSPI_DLLCR_SLV_EN |
		QUADSPI_DLLCR_SLV_AUTO_UPDT_EN |
		QUADSPI_DLLCR_DLLRES_N(8) |
		QUADSPI_DLLCR_DLL_REFCNTR_N(2) |
		QUADSPI_DLLCR_FREQEN_EN |
		QUADSPI_DLLCR_DLLEN_EN,
	.sfacr = QUADSPI_SFACR_BSWAP_EN,
	.smpr = QUADSPI_SMPR_DLLFSMPFA_NTH(4) |
		QUADSPI_SMPR_DLLFSMPFB_NTH(4),
	.dlcr = QUADSPI_DLCR_RESERVED_MASK |
		QUADSPI_DLCR_DLP_SEL_FA(1) |
		QUADSPI_DLCR_DLP_SEL_FB(1),
	.flash1_size = 0x20000000,
	.flash2_size = 0x20000000,
	.dlpr = QUADSPI_DLPR_RESET_VALUE,
};

void macronix_get_ddr_config(struct qspi_config *ddr_config)
{
	memcpy(ddr_config, &ddr_config_macronix, sizeof(ddr_config_macronix));
}

int macronix_mem_enable_ddr(struct fsl_qspi *q)
{
	void __iomem *base = q->iobase;
	u8 wren_cfg, rdcr2_cfg, rdsr_cfg, wrcr2_cfg;
	u8 cfg2_reg = 0x0;
	u8 status = 0;
	u32 mcr2;

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

	if (!s32gen1_enable_operators(q, ops, ARRAY_SIZE(ops)))
		return -1;

	mcr2 = qspi_readl(q, base + QUADSPI_MCR);

	/* Enable the module */
	qspi_writel(q, mcr2 & ~QUADSPI_MCR_MDIS_MASK, base + QUADSPI_MCR);

	if (s32gen1_mem_exec_read_op(q, &rdcr2_sdr_op, rdcr2_cfg))
		return -1;

	cfg2_reg &= ~QSPI_CFG2_OPI_MASK;
	cfg2_reg |= QSPI_CFG2_DTR_OPI_ENABLED;

	/* Enable write */
	if (s32gen1_mem_exec_write_op(q, &wren_sdr_op, wren_cfg))
		return -1;

	/* Wait write enabled */
	while (!(status & FLASH_STATUS_WEL)) {
		if (s32gen1_mem_exec_read_op(q, &rdsr_sdr_op, rdsr_cfg))
			return -1;
	}

	if (s32gen1_mem_exec_write_op(q, &wrcr2_sdr_op, wrcr2_cfg))
		return -1;

	qspi_writel(q, mcr2, base + QUADSPI_MCR);

	s32gen1_disable_operators(q, ops, ARRAY_SIZE(ops));
	usleep_range(400, 450);

	return 0;
}

