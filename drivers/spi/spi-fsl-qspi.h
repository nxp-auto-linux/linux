/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Freescale QuadSPI driver.
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2018 Bootlin
 * Copyright (C) 2018 exceet electronics GmbH
 * Copyright (C) 2018 Kontron Electronics GmbH
 * Copyright 2021 NXP
 *
 * Transition to SPI MEM interface:
 * Authors:
 *     Boris Brezillon <bbrezillon@kernel.org>
 *     Frieder Schrempf <frieder.schrempf@kontron.de>
 *     Yogesh Gaur <yogeshnarayan.gaur@nxp.com>
 *     Suresh Gupta <suresh.gupta@nxp.com>
 *
 * Based on the original fsl-quadspi.c SPI NOR driver:
 * Author: Freescale Semiconductor, Inc.
 *
 */

#ifndef _SPI_FSL_QSPI_H
#define _SPI_FSL_QSPI_H

#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/pm_qos.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

/*
 * The driver only uses one single LUT entry, that is updated on
 * each call of exec_op(). Index 0 is preset at boot with a basic
 * read operation, so let's use the last entry (15).
 */
#define	SEQID_LUT			15

/* Registers used by the driver */
#define QUADSPI_MCR			0x00
#define S32GEN1_QUADSPI_MCR_RESERVED_SHIFT	16
#define S32GEN1_QUADSPI_MCR_RESERVED_MASK	(0xF << S32GEN1_QUADSPI_MCR_RESERVED_SHIFT)
#define QUADSPI_MCR_RESERVED_MASK	GENMASK(19, 16)
#define QUADSPI_MCR_MDIS_MASK		BIT(14)
#define QUADSPI_MCR_CLR_TXF_MASK	BIT(11)
#define QUADSPI_MCR_CLR_RXF_MASK	BIT(10)
#define QUADSPI_MCR_DDR_EN_MASK		BIT(7)
#define S32GEN1_QUADSPI_MCR_DQS_EN_MASK			BIT(6)
#define S32GEN1_QUADSPI_MCR_END_CFD_SHIFT		2
#define S32GEN1_QUADSPI_MCR_END_CFD_LE_MASK		(3 << S32GEN1_QUADSPI_MCR_END_CFD_SHIFT)
#define QUADSPI_MCR_END_CFG_MASK	GENMASK(3, 2)
#define QUADSPI_MCR_SWRSTHD_MASK	BIT(1)
#define QUADSPI_MCR_SWRSTSD_MASK	BIT(0)

#define QUADSPI_IPCR			0x08
#define QUADSPI_IPCR_SEQID(x)		((x) << 24)

#define QUADSPI_FLSHCR			0x0c
#define S32GEN1_QUADSPI_FLSHCR_TCSS_SHIFT	0
#define S32GEN1_QUADSPI_FLSHCR_TCSS_MASK(N)	((N) << S32GEN1_QUADSPI_FLSHCR_TCSS_SHIFT)
#define S32GEN1_QUADSPI_FLSHCR_TCHS_SHIFT	8
#define S32GEN1_QUADSPI_FLSHCR_TCHS_MASK(N)	((N) << S32GEN1_QUADSPI_FLSHCR_TCHS_SHIFT)
#define S32GEN1_QUADSPI_FLSHCR_TDH_SHIFT_LOW	16
#define S32GEN1_QUADSPI_FLSHCR_TDH_MASK(N)	((N) << S32GEN1_QUADSPI_FLSHCR_TDH_SHIFT_LOW)
#define QUADSPI_FLSHCR_TCSS_MASK	GENMASK(3, 0)
#define QUADSPI_FLSHCR_TCSH_MASK	GENMASK(11, 8)
#define QUADSPI_FLSHCR_TDH_MASK		GENMASK(17, 16)

#define QUADSPI_BUF0CR                  0x10
#define QUADSPI_BUF1CR                  0x14
#define QUADSPI_BUF2CR                  0x18
#define QUADSPI_BUFXCR_INVALID_MSTRID   0xe

#define QUADSPI_BUF3CR			0x1c
#define QUADSPI_BUF3CR_ALLMST_MASK	BIT(31)
#define QUADSPI_BUF3CR_ADATSZ(x)	((x) << 8)
#define S32GEN1_QUADSPI_BUF3CR_ADATSZ_SHIFT	8
#define QUADSPI_BUF3CR_ADATSZ_MASK	GENMASK(15, 8)

#define QUADSPI_BFGENCR			0x20
#define QUADSPI_BFGENCR_SEQID_SHIFT	12
#define QUADSPI_BFGENCR_SEQID(x)	((x) << 12)

#define QUADSPI_BUF0IND			0x30
#define QUADSPI_BUF1IND			0x34
#define QUADSPI_BUF2IND			0x38
#define QUADSPI_SFAR			0x100

#define QUADSPI_SMPR			0x108
#define QUADSPI_SMPR_DDRSMP_MASK	GENMASK(18, 16)
#define QUADSPI_SMPR_FSDLY_MASK		BIT(6)
#define QUADSPI_SMPR_FSPHS_MASK		BIT(5)
#define QUADSPI_SMPR_HSENA_MASK		BIT(0)

#define QUADSPI_RBCT			0x110
#define QUADSPI_RBCT_WMRK_MASK	0x1F
#define QUADSPI_RBCT_RXBRD_USEIPS	BIT(8)

#define QUADSPI_TBDR			0x154

#define QUADSPI_SR			0x15c
#define QUADSPI_SR_BUSY_MASK		BIT(0)
#define QUADSPI_SR_IP_ACC_MASK		BIT(1)
#define QUADSPI_SR_AHB_ACC_MASK		BIT(2)

#define QUADSPI_FR			0x160
#define QUADSPI_FR_TFF_MASK		BIT(0)

#define QUADSPI_RSER			0x164
#define QUADSPI_RSER_TFIE		BIT(0)

#define QUADSPI_SPTRCLR			0x16c
#define QUADSPI_SPTRCLR_IPPTRC		BIT(8)
#define QUADSPI_SPTRCLR_BFPTRC		BIT(0)

#define QUADSPI_SFA1AD			0x180
#define QUADSPI_SFA2AD			0x184
#define QUADSPI_SFB1AD			0x188
#define QUADSPI_SFB2AD			0x18c
#define S32GEN1_QUADSPI_SFA1AD_VAL		0x4000000
#define S32GEN1_QUADSPI_SFA2AD_VAL		0x8000000
#define S32GEN1_QUADSPI_SFB1AD_VAL		0xC000000
#define S32GEN1_QUADSPI_SFB2AD_VAL		0x10000000
#define QUADSPI_RBDR(x)			(0x200 + ((x) * 4))

#define QUADSPI_AHB_BASE	0x0

#define QUADSPI_LUTKEY			0x300
#define QUADSPI_LUTKEY_VALUE		0x5AF05AF0

#define QUADSPI_LCKCR			0x304
#define QUADSPI_LCKER_LOCK		BIT(0)
#define QUADSPI_LCKER_UNLOCK		BIT(1)

#define QUADSPI_LUT_BASE		0x310
#define QUADSPI_LUT_OFFSET		(SEQID_LUT * 4 * 4)
#define QUADSPI_LUT_REG(idx) \
	(QUADSPI_LUT_BASE + QUADSPI_LUT_OFFSET + (idx) * 4)

/* Instruction set for the LUT register */
#define LUT_STOP			0
#define LUT_CMD				1
#define LUT_ADDR			2
#define LUT_DUMMY			3
#define LUT_MODE			4
#define LUT_MODE2			5
#define LUT_MODE4			6
#define LUT_FSL_READ		7
#define LUT_FSL_WRITE		8
#define LUT_JMP_ON_CS		9
#define LUT_ADDR_DDR		10
#define LUT_MODE_DDR		11
#define LUT_MODE2_DDR		12
#define LUT_MODE4_DDR		13
#define LUT_CMD_DDR			17
#define LUT_READ_DDR		14
#define LUT_WRITE_DDR		15
#define LUT_DATA_LEARN		16

/*
 * The PAD definitions for LUT register.
 *
 * The pad stands for the number of IO lines [0:3].
 * For example, the quad read needs four IO lines,
 * so you should use LUT_PAD(4).
 */
#define LUT_PAD(x) (fls(x) - 1)

/*
 * Macro for constructing the LUT entries with the following
 * register layout:
 *
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */
#define LUT_DEF(idx, ins, pad, opr)					\
	((((ins) << 10) | ((pad) << 8) | (opr)) << (((idx) % 2) * 16))

/* Controller needs driver to swap endianness */
#define QUADSPI_QUIRK_SWAP_ENDIAN	BIT(0)

/* Controller needs 4x internal clock */
#define QUADSPI_QUIRK_4X_INT_CLK	BIT(1)

/*
 * TKT253890, the controller needs the driver to fill the txfifo with
 * 16 bytes at least to trigger a data transfer, even though the extra
 * data won't be transferred.
 */
#define QUADSPI_QUIRK_TKT253890		BIT(2)

/* TKT245618, the controller cannot wake up from wait mode */
#define QUADSPI_QUIRK_TKT245618		BIT(3)

/*
 * Controller adds QSPI_AMBA_BASE (base address of the mapped memory)
 * internally. No need to add it when setting SFXXAD and SFAR registers
 */
#define QUADSPI_QUIRK_BASE_INTERNAL	BIT(4)

/*
 * Controller uses TDH bits in register QUADSPI_FLSHCR.
 * They need to be set in accordance with the DDR/SDR mode.
 */
#define QUADSPI_QUIRK_USE_TDH_SETTING	BIT(5)

struct fsl_qspi_devtype_data {
	unsigned int rxfifo;
	unsigned int txfifo;
	int invalid_mstrid;
	unsigned int ahb_buf_size;
	unsigned int quirks;
	bool little_endian;
};

struct fsl_qspi {
	void __iomem *iobase;
	void __iomem *ahb_addr;
	u32 memmap_phy;
	struct clk *clk, *clk_en;
	struct device *dev;
	struct completion c;
	const struct fsl_qspi_devtype_data *devtype_data;
	struct mutex lock;
	struct pm_qos_request pm_qos_req;
	int selected;
	bool ddr_mode;
	u32 clk_rate;
	u32 num_pads;
	u32 flags;
};

void qspi_writel(struct fsl_qspi *q, u32 val, void __iomem *addr);
u32 qspi_readl(struct fsl_qspi *q, void __iomem *addr);
void reset_bootrom_settings(struct fsl_qspi *q);
int enable_spi(struct fsl_qspi *q, bool force);
int s32gen1_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op);
int s32gen1_exec_op(struct spi_mem *mem, const struct spi_mem_op *op);
bool s32gen1_supports_op(struct spi_mem *mem, const struct spi_mem_op *op);

#endif /* _SPI_FSL_QSPI_H */
