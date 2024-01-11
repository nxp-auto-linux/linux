// SPDX-License-Identifier: GPL-2.0+

/*
 * Freescale QuadSPI driver.
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2018 Bootlin
 * Copyright (C) 2018 exceet electronics GmbH
 * Copyright (C) 2018 Kontron Electronics GmbH
 * Copyright 2021-2024 NXP
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

#include <asm/div64.h>
#include <linux/bitops.h>
#include <linux/cache.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/sizes.h>

#include <linux/mtd/spi-nor.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

/*
 * The driver only uses one single LUT entry, that is updated on
 * each call of exec_op(). Index 0 is preset at boot with a basic
 * read operation, so let's use the last entry (15).
 */
#define SEQID_LUT_AHB       14
#define	SEQID_LUT			15

/* Registers used by the driver */
#define QUADSPI_MCR			0x00
#define QUADSPI_MCR_DQS_EXTERNAL	(0x3 << 24)
#define QUADSPI_MCR_DQS_INT_PAD		(0x1 << 24)
#define QUADSPI_MCR_RESERVED_MASK	GENMASK(19, 16)
#define QUADSPI_MCR_ISD3FB		BIT(19)
#define QUADSPI_MCR_ISD2FB		BIT(18)
#define QUADSPI_MCR_ISD3FA		BIT(17)
#define QUADSPI_MCR_ISD2FA		BIT(16)
#define QUADSPI_MCR_MDIS_MASK		BIT(14)
#define QUADSPI_MCR_CLR_TXF_MASK	BIT(11)
#define QUADSPI_MCR_CLR_RXF_MASK	BIT(10)
#define QUADSPI_MCR_DQS_EN			BIT(6)
#define QUADSPI_MCR_DDR_EN_MASK		BIT(7)
#define QUADSPI_MCR_END_CFG_MASK	GENMASK(3, 2)
#define QUADSPI_MCR_SWRSTHD_MASK	BIT(1)
#define QUADSPI_MCR_SWRSTSD_MASK	BIT(0)

#define QUADSPI_IPCR			0x08
#define QUADSPI_IPCR_SEQID(x)		((x) << 24)

#define QUADSPI_FLSHCR			0x0c
#define QUADSPI_FLSHCR_TCSS_MASK	GENMASK(3, 0)
#define QUADSPI_FLSHCR_TCSS(N)		((N) << 0)
#define QUADSPI_FLSHCR_TCSH_MASK	GENMASK(11, 8)
#define QUADSPI_FLSHCR_TCSH(N)		((N) << 8)
#define QUADSPI_FLSHCR_TDH_MASK		GENMASK(17, 16)
#define QUADSPI_FLSHCR_TDH(N)		((N) << 16)

#define QUADSPI_BUF0CR                  0x10
#define QUADSPI_BUF1CR                  0x14
#define QUADSPI_BUF2CR                  0x18
#define QUADSPI_BUFXCR_INVALID_MSTRID   0xe

#define QUADSPI_BUF3CR			0x1c
#define QUADSPI_BUF3CR_ALLMST_MASK	BIT(31)
#define QUADSPI_BUF3CR_ADATSZ_MASK	GENMASK(15, 8)
#define QUADSPI_BUF3CR_ADATSZ_SHIFT	8
#define QUADSPI_BUF3CR_ADATSZ(x)	((x) << QUADSPI_BUF3CR_ADATSZ_SHIFT)
#define QUADSPI_BUF3CR_ADATSZ_MASK	GENMASK(15, 8)

#define QUADSPI_BFGENCR			0x20
#define QUADSPI_BFGENCR_SEQID(x)	((x) << 12)

#define QUADSPI_BUF0IND			0x30
#define QUADSPI_BUF1IND			0x34
#define QUADSPI_BUF2IND			0x38

#define QUADSPI_DLLCRA			0x60
#define QUADSPI_DLLCR_MASK		0x7FFFFFF0UL
#define QUADSPI_DLLCR_DLLEN_EN		BIT(31)
#define QUADSPI_DLLCR_FREQEN_EN		BIT(30)
#define QUADSPI_DLLCR_DLL_REFCNTR_N(N)	((N) << 24)
#define QUADSPI_DLLCR_DLLRES_N(N)		((N) << 20)
#define QUADSPI_DLLCR_SLV_DLY_OFFSET_N(N)	((N) << 12)
#define QUADSPI_DLLCR_SLV_DLY_COARSE_N(N)	((N) << 8)
#define QUADSPI_DLLCR_SLV_AUTO_UPDT_SHIFT	3
#define QUADSPI_DLLCR_SLV_AUTO_UPDT_EN		BIT(QUADSPI_DLLCR_SLV_AUTO_UPDT_SHIFT)
#define QUADSPI_DLLCR_SLV_EN_SHIFT			2
#define QUADSPI_DLLCR_SLV_EN			BIT(QUADSPI_DLLCR_SLV_EN_SHIFT)
#define QUADSPI_DLLCR_SLV_BYPASS_SHIFT	1
#define QUADSPI_DLLCR_SLV_BYPASS_EN	BIT(QUADSPI_DLLCR_SLV_BYPASS_SHIFT)
#define QUADSPI_DLLCR_SLV_UPD_EN	BIT(0)

#define QUADSPI_SFAR			0x100

#define QUADSPI_SFACR			0x104
#define QUADSPI_SFACR_BSWAP		BIT(17)

#define QUADSPI_SMPR			0x108
#define QUADSPI_SMPR_DLLFSMPFB_MASK		GENMASK(30, 28)
#define QUADSPI_SMPR_DLLFSMPFB_NTH(N)	((N) << 28)
#define QUADSPI_SMPR_DLLFSMPFA_MASK		GENMASK(26, 24)
#define QUADSPI_SMPR_DLLFSMPFA_NTH(N)	((N) << 24)
#define QUADSPI_SMPR_DDRSMP_MASK	GENMASK(18, 16)
#define QUADSPI_SMPR_FSDLY_MASK		BIT(6)
#define QUADSPI_SMPR_FSPHS_MASK		BIT(5)
#define QUADSPI_SMPR_HSENA_MASK		BIT(0)

#define QUADSPI_RBCT			0x110
#define QUADSPI_RBCT_WMRK_MASK		GENMASK(4, 0)
#define QUADSPI_RBCT_RXBRD_USEIPS	BIT(8)

#define QUADSPI_DLLSR			0x12c
#define QUADSPI_DLLSR_SLVA_LOCK		BIT(14)
#define QUADSPI_DLLSR_DLLA_LOCK		BIT(15)

#define QUADSPI_DLCR			0x130
#define QUADSPI_DLCR_RESERVED_MASK	((0xff << 0) | (0xff << 16))
#define QUADSPI_DLCR_DLP_SEL_FB(N)	((N) << 30)
#define QUADSPI_DLCR_DLP_SEL_FA(N)	((N) << 14)

#define QUADSPI_TBSR			0x150
#define QUADSPI_TBSR_TRCTR(TBSR)	((TBSR) >> 16)
#define QUADSPI_TBSR_TRBFL(TBSR)	((TBSR) & 0xFF)
#define QUADSPI_TBSR_TRBFL_MASK		0xFFU

#define QUADSPI_TBDR			0x154

#define QUADSPI_SR			0x15c
#define QUADSPI_SR_BUSY				BIT(0)
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
#define QUADSPI_RBDR(x)			(0x200 + ((x) * 4))

#define QUADSPI_DLPR			0x190
#define QUADSPI_DLPR_RESET_VALUE	0xaa553443

#define QUADSPI_LUTKEY			0x300
#define QUADSPI_LUTKEY_VALUE		0x5AF05AF0

#define QUADSPI_LCKCR			0x304
#define QUADSPI_LCKER_LOCK		BIT(0)
#define QUADSPI_LCKER_UNLOCK		BIT(1)

#define QUADSPI_LUT_BASE		0x310
#define QUADSPI_LUT_OFFSET		(SEQID_LUT * 4 * 4)
#define QUADSPI_LUT_REG(idx) \
	(QUADSPI_LUT_BASE + QUADSPI_LUT_OFFSET + (idx) * 4)
#define S32CC_QUADSPI_LUT_OFFSET (SEQID_LUT * 4 * 5)
#define S32CC_QUADSPI_LUT_REG(idx) \
	(QUADSPI_LUT_BASE + S32CC_QUADSPI_LUT_OFFSET + (idx) * 4)

#define QUADSPI_AHB_LUT_OFFSET      (SEQID_LUT_AHB * 4 * 5)
#define QUADSPI_AHB_LUT_REG(idx) \
	(QUADSPI_LUT_BASE + QUADSPI_AHB_LUT_OFFSET + (idx) * 4)

#define LUT_SIZE_BITS		16

/* Instruction set for the LUT register */
#define LUT_STOP		0
#define LUT_CMD			1
#define LUT_ADDR		2
#define LUT_DUMMY		3
#define LUT_MODE		4
#define LUT_MODE2		5
#define LUT_MODE4		6
#define LUT_FSL_READ		7
#define LUT_FSL_WRITE		8
#define LUT_JMP_ON_CS		9
#define LUT_ADDR_DDR		10
#define LUT_MODE_DDR		11
#define LUT_MODE2_DDR		12
#define LUT_MODE4_DDR		13
#define LUT_FSL_READ_DDR	14
#define LUT_FSL_WRITE_DDR	15
#define LUT_DATA_LEARN		16
#define LUT_CMD_DDR			17

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
	((((ins) << 10) | ((pad) << 8) | ((opr) & 0xFFU)) << (((idx) % 2) * 16))

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

/*
 * Controller has Delay Locked Loop logic
 */
#define QUADSPI_QUIRK_HAS_DLL			BIT(6)

/*
 * Has octal support
 */
#define QUADSPI_QUIRK_OCTAL_SUPPORT		BIT(7)

/*
 * Allow to read the entire AHB space at once
 */
#define QUADSPI_QUIRK_READ_ENTIRE_AHB		BIT(8)

/*
 * Selects delay chain for low frequency of operation
 */
#define QUADSPI_QUIRK_LOW_FREQ_DELAY_CHAIN	BIT(9)

/*
 * Selects internal pad loopback for SDR operations
 */
#define QUADSPI_QUIRK_INT_LOOPBACK_SDR		BIT(10)

struct fsl_qspi_devtype_data {
	unsigned int rxfifo;
	unsigned int txfifo;
	int invalid_mstrid;
	unsigned int ahb_buf_size;
	unsigned int quirks;
	bool little_endian;
	u32 flash1_size;
	u32 flash2_size;
};

static const struct fsl_qspi_devtype_data vybrid_data = {
	.rxfifo = SZ_128,
	.txfifo = SZ_64,
	.invalid_mstrid = QUADSPI_BUFXCR_INVALID_MSTRID,
	.ahb_buf_size = SZ_1K,
	.quirks = QUADSPI_QUIRK_SWAP_ENDIAN,
	.little_endian = true,
};

static const struct fsl_qspi_devtype_data imx6sx_data = {
	.rxfifo = SZ_128,
	.txfifo = SZ_512,
	.invalid_mstrid = QUADSPI_BUFXCR_INVALID_MSTRID,
	.ahb_buf_size = SZ_1K,
	.quirks = QUADSPI_QUIRK_4X_INT_CLK | QUADSPI_QUIRK_TKT245618,
	.little_endian = true,
};

static const struct fsl_qspi_devtype_data imx7d_data = {
	.rxfifo = SZ_128,
	.txfifo = SZ_512,
	.invalid_mstrid = QUADSPI_BUFXCR_INVALID_MSTRID,
	.ahb_buf_size = SZ_1K,
	.quirks = QUADSPI_QUIRK_TKT253890 | QUADSPI_QUIRK_4X_INT_CLK |
		  QUADSPI_QUIRK_USE_TDH_SETTING,
	.little_endian = true,
};

static const struct fsl_qspi_devtype_data imx6ul_data = {
	.rxfifo = SZ_128,
	.txfifo = SZ_512,
	.invalid_mstrid = QUADSPI_BUFXCR_INVALID_MSTRID,
	.ahb_buf_size = SZ_1K,
	.quirks = QUADSPI_QUIRK_TKT253890 | QUADSPI_QUIRK_4X_INT_CLK |
		  QUADSPI_QUIRK_USE_TDH_SETTING,
	.little_endian = true,
};

static const struct fsl_qspi_devtype_data ls1021a_data = {
	.rxfifo = SZ_128,
	.txfifo = SZ_64,
	.invalid_mstrid = QUADSPI_BUFXCR_INVALID_MSTRID,
	.ahb_buf_size = SZ_1K,
	.quirks = 0,
	.little_endian = false,
};

static const struct fsl_qspi_devtype_data ls2080a_data = {
	.rxfifo = SZ_128,
	.txfifo = SZ_64,
	.ahb_buf_size = SZ_1K,
	.invalid_mstrid = 0x0,
	.quirks = QUADSPI_QUIRK_TKT253890 | QUADSPI_QUIRK_BASE_INTERNAL,
	.little_endian = true,
};

static const struct fsl_qspi_devtype_data s32cc_data = {
	.rxfifo = SZ_128,
	.txfifo = SZ_256,
	.invalid_mstrid = QUADSPI_BUFXCR_INVALID_MSTRID,
	.ahb_buf_size = SZ_1K,
	.quirks = QUADSPI_QUIRK_USE_TDH_SETTING | QUADSPI_QUIRK_HAS_DLL |
	    QUADSPI_QUIRK_OCTAL_SUPPORT | QUADSPI_QUIRK_READ_ENTIRE_AHB |
	    QUADSPI_QUIRK_INT_LOOPBACK_SDR,
	.little_endian = true,
	.flash1_size = 0x20000000,
	.flash2_size = 0x20000000,
};

static const struct fsl_qspi_devtype_data s32g3_data = {
	.rxfifo = SZ_128,
	.txfifo = SZ_256,
	.invalid_mstrid = QUADSPI_BUFXCR_INVALID_MSTRID,
	.ahb_buf_size = SZ_1K,
	.quirks = QUADSPI_QUIRK_USE_TDH_SETTING | QUADSPI_QUIRK_HAS_DLL |
	    QUADSPI_QUIRK_OCTAL_SUPPORT | QUADSPI_QUIRK_READ_ENTIRE_AHB |
	    QUADSPI_QUIRK_INT_LOOPBACK_SDR,
	.little_endian = true,
	.flash1_size = 0x20000000,
	.flash2_size = 0x20000000,
};

static const struct fsl_qspi_devtype_data s32r45_data = {
	.rxfifo = SZ_128,
	.txfifo = SZ_256,
	.invalid_mstrid = QUADSPI_BUFXCR_INVALID_MSTRID,
	.ahb_buf_size = SZ_1K,
	.quirks = QUADSPI_QUIRK_USE_TDH_SETTING | QUADSPI_QUIRK_HAS_DLL |
	    QUADSPI_QUIRK_OCTAL_SUPPORT | QUADSPI_QUIRK_READ_ENTIRE_AHB |
	    QUADSPI_QUIRK_LOW_FREQ_DELAY_CHAIN | QUADSPI_QUIRK_INT_LOOPBACK_SDR,
	.little_endian = true,
	.flash1_size = 0x20000000,
	.flash2_size = 0x20000000,
};

struct qspi_config {
	u32 mcr;
	u32 flshcr;
	u32 dllcr;
	u32 sfacr;
	u32 smpr;
	u32 dlcr;
	u32 dlpr;
};

static const struct qspi_config octal_ddr_conf = {
	.mcr = QUADSPI_MCR_END_CFG_MASK |
	    QUADSPI_MCR_DQS_EN |
	    QUADSPI_MCR_DDR_EN_MASK |
	    QUADSPI_MCR_ISD2FA |
	    QUADSPI_MCR_ISD3FA |
	    QUADSPI_MCR_ISD2FB |
	    QUADSPI_MCR_ISD3FB |
	    QUADSPI_MCR_DQS_EXTERNAL,
	.flshcr = QUADSPI_FLSHCR_TCSS(3) |
	    QUADSPI_FLSHCR_TCSH(3) |
	    QUADSPI_FLSHCR_TDH(1),
	.dllcr = QUADSPI_DLLCR_SLV_EN |
	    QUADSPI_DLLCR_SLV_AUTO_UPDT_EN |
	    QUADSPI_DLLCR_DLLRES_N(8) |
	    QUADSPI_DLLCR_DLL_REFCNTR_N(2) |
	    QUADSPI_DLLCR_FREQEN_EN |
	    QUADSPI_DLLCR_DLLEN_EN,
	.sfacr = QUADSPI_SFACR_BSWAP,
	.smpr = QUADSPI_SMPR_DLLFSMPFA_NTH(4) |
		QUADSPI_SMPR_DLLFSMPFB_NTH(4),
	.dlcr = QUADSPI_DLCR_RESERVED_MASK |
	    QUADSPI_DLCR_DLP_SEL_FA(1) |
	    QUADSPI_DLCR_DLP_SEL_FB(1),
	.dlpr = QUADSPI_DLPR_RESET_VALUE,
};

struct fsl_qspi {
	void __iomem *iobase;
	void __iomem *ahb_addr;
	u32 memmap_phy;
	u32 memmap_size;
	struct clk *clk, *clk_en;
	struct device *dev;
	struct completion c;
	const struct fsl_qspi_devtype_data *devtype_data;
	struct mutex lock;
	struct pm_qos_request pm_qos_req;
	int selected;
	enum spi_nor_protocol proto;
	u32 clk_rate;
	bool from_str;
};

static inline int needs_swap_endian(struct fsl_qspi *q)
{
	return q->devtype_data->quirks & QUADSPI_QUIRK_SWAP_ENDIAN;
}

static inline int needs_4x_clock(struct fsl_qspi *q)
{
	return q->devtype_data->quirks & QUADSPI_QUIRK_4X_INT_CLK;
}

static inline int needs_fill_txfifo(struct fsl_qspi *q)
{
	return q->devtype_data->quirks & QUADSPI_QUIRK_TKT253890;
}

static inline int needs_wakeup_wait_mode(struct fsl_qspi *q)
{
	return q->devtype_data->quirks & QUADSPI_QUIRK_TKT245618;
}

static inline int needs_amba_base_offset(struct fsl_qspi *q)
{
	return !(q->devtype_data->quirks & QUADSPI_QUIRK_BASE_INTERNAL);
}

static inline int needs_tdh_setting(struct fsl_qspi *q)
{
	return q->devtype_data->quirks & QUADSPI_QUIRK_USE_TDH_SETTING;
}

static inline int has_dll(struct fsl_qspi *q)
{
	return q->devtype_data->quirks & QUADSPI_QUIRK_HAS_DLL;
}

static inline int has_octal_support(struct fsl_qspi *q)
{
	return q->devtype_data->quirks & QUADSPI_QUIRK_OCTAL_SUPPORT;
}

static inline int can_read_entire_ahb(struct fsl_qspi *q)
{
	return q->devtype_data->quirks & QUADSPI_QUIRK_READ_ENTIRE_AHB;
}

static inline int low_freq_chain(struct fsl_qspi *q)
{
	return q->devtype_data->quirks & QUADSPI_QUIRK_LOW_FREQ_DELAY_CHAIN;
}

static inline int sdr_internal_loopback(struct fsl_qspi *q)
{
	return q->devtype_data->quirks & QUADSPI_QUIRK_INT_LOOPBACK_SDR;
}

/*
 * An IC bug makes it necessary to rearrange the 32-bit data.
 * Later chips, such as IMX6SLX, have fixed this bug.
 */
static inline u32 fsl_qspi_endian_xchg(struct fsl_qspi *q, u32 a)
{
	return needs_swap_endian(q) ? __swab32(a) : a;
}

/*
 * R/W functions for big- or little-endian registers:
 * The QSPI controller's endianness is independent of
 * the CPU core's endianness. So far, although the CPU
 * core is little-endian the QSPI controller can use
 * big-endian or little-endian.
 */
static void qspi_writel(struct fsl_qspi *q, u32 val, void __iomem *addr)
{
	if (q->devtype_data->little_endian)
		iowrite32(val, addr);
	else
		iowrite32be(val, addr);
}

static u32 qspi_readl(struct fsl_qspi *q, void __iomem *addr)
{
	if (q->devtype_data->little_endian)
		return ioread32(addr);

	return ioread32be(addr);
}

static inline int is_s32cc_qspi(struct fsl_qspi *q)
{
	return (q->devtype_data == &s32cc_data) ||
		(q->devtype_data == &s32g3_data) ||
		(q->devtype_data == &s32r45_data);
}

static inline int is_s32g3_qspi(struct fsl_qspi *q)
{
	return q->devtype_data == &s32g3_data;
}

static irqreturn_t fsl_qspi_irq_handler(int irq, void *dev_id)
{
	struct fsl_qspi *q = dev_id;
	u32 reg;

	/* clear interrupt */
	reg = qspi_readl(q, q->iobase + QUADSPI_FR);
	qspi_writel(q, reg, q->iobase + QUADSPI_FR);

	if (reg & QUADSPI_FR_TFF_MASK)
		complete(&q->c);

	dev_dbg(q->dev, "QUADSPI_FR : 0x%.8x:0x%.8x\n", 0, reg);
	return IRQ_HANDLED;
}

static int fsl_qspi_check_buswidth(struct fsl_qspi *q, u8 width)
{
	switch (width) {
	case 1:
	case 2:
	case 4:
		return 0;
	case 8:
		if (has_octal_support(q))
			return 0;
	}

	return -ENOTSUPP;
}

static bool fsl_qspi_supports_op(struct spi_mem *mem,
				 const struct spi_mem_op *op)
{
	struct fsl_qspi *q = spi_controller_get_devdata(mem->spi->master);
	int ret;

	ret = fsl_qspi_check_buswidth(q, op->cmd.buswidth);

	if (op->addr.nbytes)
		ret |= fsl_qspi_check_buswidth(q, op->addr.buswidth);

	if (op->dummy.nbytes)
		ret |= fsl_qspi_check_buswidth(q, op->dummy.buswidth);

	if (op->data.nbytes)
		ret |= fsl_qspi_check_buswidth(q, op->data.buswidth);

	if (ret)
		return false;

	/*
	 * The number of instructions needed for the op, needs
	 * to fit into a single LUT entry.
	 */
	if (op->addr.nbytes +
	   (op->dummy.nbytes ? 1:0) +
	   (op->data.nbytes ? 1:0) > 6)
		return false;

	/* Max 64 dummy clock cycles supported */
	if (op->dummy.nbytes &&
	    (op->dummy.nbytes * 8 / op->dummy.buswidth > 64))
		return false;

	/* Max data length, check controller limits and alignment */
	if (op->data.dir == SPI_MEM_DATA_IN &&
	    (op->data.nbytes > q->devtype_data->ahb_buf_size ||
	     (op->data.nbytes > q->devtype_data->rxfifo - 4 &&
		  !IS_ALIGNED(op->data.nbytes, 8)))) {
		if (!can_read_entire_ahb(q) &&
		    op->data.nbytes > q->devtype_data->ahb_buf_size)
			return false;
	}

	if (op->data.dir == SPI_MEM_DATA_OUT &&
	    op->data.nbytes > q->devtype_data->txfifo)
		return false;

	if (op->cmd.dtr && op->addr.dtr && op->dummy.dtr && op->data.dtr)
		ret = spi_mem_dtr_supports_op(mem, op);
	else
		ret = spi_mem_default_supports_op(mem, op);

	return ret;
}

static void fsl_qspi_prepare_lut(struct fsl_qspi *q,
				 const struct spi_mem_op *op)
{
	u8 lut_cmd, lut_addr, lut_data, data_size;
	void __iomem *base = q->iobase;
	u32 lutval[4] = {};
	int lutidx = 1, i;

	if (op->cmd.dtr)
		lut_cmd = LUT_CMD_DDR;
	else
		lut_cmd = LUT_CMD;

	lutval[0] |= LUT_DEF(0, lut_cmd, LUT_PAD(op->cmd.buswidth),
			     op->cmd.opcode);

	if (op->cmd.buswidth == 8) {
		lutval[lutidx / 2] <<= LUT_SIZE_BITS;
		lutval[lutidx / 2] |= LUT_DEF(0, lut_cmd,
					      LUT_PAD(op->cmd.buswidth),
					      op->cmd.opcode >> 8);
		lutidx++;
	}

	if (op->addr.dtr)
		lut_addr = LUT_ADDR_DDR;
	else
		lut_addr = LUT_ADDR;

	if (can_read_entire_ahb(q)) {
		if (op->addr.nbytes) {
			lutval[lutidx / 2] |= LUT_DEF(lutidx, lut_addr,
					LUT_PAD(op->addr.buswidth),
					(op->addr.nbytes == 4) ? 0x20 : 0x18);
			lutidx++;
		}
	} else {
		/*
		 * For some unknown reason, using LUT_ADDR doesn't work in some
		 * cases (at least with only one byte long addresses), so
		 * let's use LUT_MODE to write the address bytes one by one
		 */
		for (i = 0; i < op->addr.nbytes; i++) {
			u8 addrbyte = op->addr.val >> (8 * (op->addr.nbytes - i - 1));

			lutval[lutidx / 2] |= LUT_DEF(lutidx, LUT_MODE,
							  LUT_PAD(op->addr.buswidth),
							  addrbyte);
			lutidx++;
		}
	}

	if (op->dummy.nbytes) {
		lutval[lutidx / 2] |= LUT_DEF(lutidx, LUT_DUMMY,
					      LUT_PAD(op->dummy.buswidth),
					      op->dummy.nbytes * 8 /
					      op->dummy.buswidth);
		lutidx++;
	}

	if (op->data.nbytes) {
		if (op->data.dir == SPI_MEM_DATA_IN) {
			if (op->addr.dtr)
				lut_data = LUT_FSL_READ_DDR;
			else
				lut_data = LUT_FSL_READ;
		} else {
			if (op->addr.dtr)
				lut_data = LUT_FSL_WRITE_DDR;
			else
				lut_data = LUT_FSL_WRITE;
		}

		/* HW limitation for memory write operation */
		if (op->data.dir == SPI_MEM_DATA_OUT && op->data.dtr) {
			data_size = 0;
		} else {
			if (op->data.nbytes > q->devtype_data->rxfifo)
				data_size = q->devtype_data->rxfifo;
			else
				data_size = op->data.nbytes;
		}

		lutval[lutidx / 2] |= LUT_DEF(lutidx, lut_data,
					      LUT_PAD(op->data.buswidth),
					      data_size);
		lutidx++;
	}

	lutval[lutidx / 2] |= LUT_DEF(lutidx, LUT_STOP, 0, 0);

	/* unlock LUT */
	qspi_writel(q, QUADSPI_LUTKEY_VALUE, q->iobase + QUADSPI_LUTKEY);
	qspi_writel(q, QUADSPI_LCKER_UNLOCK, q->iobase + QUADSPI_LCKCR);

	/* fill LUT */
	for (i = 0; i < ARRAY_SIZE(lutval); i++)
		if (is_s32cc_qspi(q))
			qspi_writel(q, lutval[i], base + S32CC_QUADSPI_LUT_REG(i));
		else
			qspi_writel(q, lutval[i], base + QUADSPI_LUT_REG(i));

	if (can_read_entire_ahb(q)) {
		if (op->data.nbytes && op->data.dir == SPI_MEM_DATA_IN &&
		    op->addr.nbytes) {
			for (i = 0; i < ARRAY_SIZE(lutval); i++)
				qspi_writel(q, lutval[i], base + QUADSPI_AHB_LUT_REG(i));
		}
	}

	/* lock LUT */
	qspi_writel(q, QUADSPI_LUTKEY_VALUE, q->iobase + QUADSPI_LUTKEY);
	qspi_writel(q, QUADSPI_LCKER_LOCK, q->iobase + QUADSPI_LCKCR);
}

static int fsl_qspi_clk_prep_enable(struct fsl_qspi *q)
{
	int ret;

	ret = clk_prepare_enable(q->clk_en);
	if (ret)
		return ret;

	ret = clk_prepare_enable(q->clk);
	if (ret) {
		clk_disable_unprepare(q->clk_en);
		return ret;
	}

	if (needs_wakeup_wait_mode(q))
		cpu_latency_qos_add_request(&q->pm_qos_req, 0);

	return 0;
}

static void fsl_qspi_clk_disable_unprep(struct fsl_qspi *q)
{
	if (needs_wakeup_wait_mode(q))
		cpu_latency_qos_remove_request(&q->pm_qos_req);

	clk_disable_unprepare(q->clk);
	clk_disable_unprepare(q->clk_en);
}

/*
 * If we have changed the content of the flash by writing or erasing, or if we
 * read from flash with a different offset into the page buffer, we need to
 * invalidate the AHB buffer. If we do not do so, we may read out the wrong
 * data. The spec tells us reset the AHB domain and Serial Flash domain at
 * the same time.
 */
static void fsl_qspi_invalidate(struct fsl_qspi *q)
{
	u32 mcr;
	u32 reset_mask = QUADSPI_MCR_SWRSTHD_MASK | QUADSPI_MCR_SWRSTSD_MASK;

	mcr = qspi_readl(q, q->iobase + QUADSPI_MCR);
	mcr &= ~QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, q->iobase + QUADSPI_MCR);

	mcr = qspi_readl(q, q->iobase + QUADSPI_MCR);
	mcr |= reset_mask;
	qspi_writel(q, mcr, q->iobase + QUADSPI_MCR);

	mcr = qspi_readl(q, q->iobase + QUADSPI_MCR) | QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, q->iobase + QUADSPI_MCR);

	mcr &= ~(reset_mask);
	qspi_writel(q, mcr, q->iobase + QUADSPI_MCR);

	mcr = qspi_readl(q, q->iobase + QUADSPI_MCR);
	mcr &= ~QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, q->iobase + QUADSPI_MCR);
}

static void fsl_qspi_select_mem(struct fsl_qspi *q, struct spi_device *spi)
{
	unsigned long rate = spi->max_speed_hz;
	int ret;

	if (q->selected == spi->chip_select)
		return;

	if (needs_4x_clock(q))
		rate *= 4;

	fsl_qspi_clk_disable_unprep(q);

	ret = clk_set_rate(q->clk, rate);
	if (ret)
		return;

	ret = fsl_qspi_clk_prep_enable(q);
	if (ret)
		return;

	q->selected = spi->chip_select;

	fsl_qspi_invalidate(q);
}

static u32 fsl_qspi_memsize_per_cs(struct fsl_qspi *q)
{
	if (can_read_entire_ahb(q))
		return q->memmap_size / 4;
	else
		return ALIGN(q->devtype_data->ahb_buf_size, 0x400);
}

static void fsl_qspi_read_ahb(struct fsl_qspi *q, const struct spi_mem_op *op)
{
	void __iomem *ahb_read_addr = q->ahb_addr;
	struct timespec64 start, end, duration;
	u64 mb_int, mb_frac;
	u32 us_passed, rem;

	if (can_read_entire_ahb(q)) {
		if (op->addr.nbytes)
			ahb_read_addr += op->addr.val;
	}

	dcache_inval_poc((unsigned long)ahb_read_addr,
			 (unsigned long)ahb_read_addr + op->data.nbytes);

	ktime_get_ts64(&start);
	memcpy_fromio(op->data.buf.in,
		      ahb_read_addr + q->selected * fsl_qspi_memsize_per_cs(q),
		      op->data.nbytes);
	ktime_get_ts64(&end);

	 duration = timespec64_sub(end, start);
	 us_passed = duration.tv_sec * 1000000 +
		 (duration.tv_nsec / NSEC_PER_USEC);

	if (us_passed > 0) {
		mb_int = div_u64_rem(op->data.nbytes, us_passed, &rem);
		mb_frac = div64_u64(rem * 1000, us_passed);
		dev_dbg(q->dev, "%u bytes read in %u us (%llu.%llu MB/s)\n",
			op->data.nbytes, us_passed, mb_int, mb_frac);
	}
}

static int fsl_qspi_readl_poll_tout(struct fsl_qspi *q, void __iomem *base,
				    u32 mask, u32 delay_us, u32 timeout_us)
{
	u32 reg;

	if (!q->devtype_data->little_endian)
		mask = (u32)cpu_to_be32(mask);

	return readl_poll_timeout(base, reg, !(reg & mask), delay_us,
				  timeout_us);
}

static int fsl_qspi_fill_txfifo(struct fsl_qspi *q,
				const struct spi_mem_op *op)
{
	void __iomem *base = q->iobase;
	int i, ret;
	u32 val;

	/* TX buffer is empty */
	ret = fsl_qspi_readl_poll_tout(q, base + QUADSPI_TBSR,
				       QUADSPI_TBSR_TRBFL_MASK,
				       10, 1000);
	if (ret)
		return ret;

	for (i = 0; i < ALIGN_DOWN(op->data.nbytes, 4); i += 4) {
		memcpy(&val, op->data.buf.out + i, 4);
		val = fsl_qspi_endian_xchg(q, val);
		qspi_writel(q, val, base + QUADSPI_TBDR);
	}

	if (i < op->data.nbytes) {
		memcpy(&val, op->data.buf.out + i, op->data.nbytes - i);
		val = fsl_qspi_endian_xchg(q, val);
		qspi_writel(q, val, base + QUADSPI_TBDR);
	}

	if (needs_fill_txfifo(q)) {
		for (i = op->data.nbytes; i < 16; i += 4)
			qspi_writel(q, 0, base + QUADSPI_TBDR);
	}

	return ret;
}

static void fsl_qspi_read_rxfifo(struct fsl_qspi *q,
			  const struct spi_mem_op *op)
{
	void __iomem *base = q->iobase;
	int i;
	u8 *buf = op->data.buf.in;
	u32 val;

	for (i = 0; i < ALIGN_DOWN(op->data.nbytes, 4); i += 4) {
		val = qspi_readl(q, base + QUADSPI_RBDR(i / 4));
		val = fsl_qspi_endian_xchg(q, val);
		memcpy(buf + i, &val, 4);
	}

	if (i < op->data.nbytes) {
		val = qspi_readl(q, base + QUADSPI_RBDR(i / 4));
		val = fsl_qspi_endian_xchg(q, val);
		memcpy(buf + i, &val, op->data.nbytes - i);
	}
}

static int fsl_qspi_do_op(struct fsl_qspi *q, const struct spi_mem_op *op)
{
	void __iomem *base = q->iobase;
	u32 tbsr, trctr, trbfl, words;
	int err = 0;

	/*
	 * Always start the sequence at the same index since we update
	 * the LUT at each exec_op() call. And also specify the DATA
	 * length, since it's has not been specified in the LUT.
	 */
	qspi_writel(q, op->data.nbytes | QUADSPI_IPCR_SEQID(SEQID_LUT),
		    base + QUADSPI_IPCR);

	if (op->data.nbytes && op->data.dir == SPI_MEM_DATA_OUT) {
		words = op->data.nbytes / 4;
		if (op->data.nbytes % 4)
			words++;

		/* Wait until all bytes are transmitted */
		do {
			tbsr = qspi_readl(q, base + QUADSPI_TBSR);
			trctr = QUADSPI_TBSR_TRCTR(tbsr);
			trbfl = QUADSPI_TBSR_TRBFL(tbsr);
		} while ((trctr != words) || trbfl);
	}

	/* Wait for the controller being ready */
	err = fsl_qspi_readl_poll_tout(q, base + QUADSPI_SR,
				       (QUADSPI_SR_IP_ACC_MASK |
					QUADSPI_SR_AHB_ACC_MASK |
					QUADSPI_SR_BUSY),
					10, 1000);

	if (!err && op->data.nbytes && op->data.dir == SPI_MEM_DATA_IN)
		fsl_qspi_read_rxfifo(q, op);

	return err;
}

static void dllcra_bypass(struct fsl_qspi *q, u32 dllmask)
{
	void __iomem *base = q->iobase;
	u32 dllcra;

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
		 QUADSPI_DLLSR_SLVA_LOCK))
		;

	dllcra &= ~QUADSPI_DLLCR_SLV_UPD_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);
}

static void dllcra_manual(struct fsl_qspi *q, u32 dllmask)
{
	void __iomem *base = q->iobase;
	u32 dllcra;

	dllmask &= QUADSPI_DLLCR_MASK;

	dllcra = QUADSPI_DLLCR_SLV_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= dllmask;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_DLLEN_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_DLLA_LOCK))
		;

	dllcra &= ~QUADSPI_DLLCR_SLV_AUTO_UPDT_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra &= ~QUADSPI_DLLCR_SLV_BYPASS_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra = qspi_readl(q, base + QUADSPI_DLLCRA);
	dllcra |= QUADSPI_DLLCR_SLV_UPD_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_SLVA_LOCK))
		;
}

static void dllcra_auto(struct fsl_qspi *q, u32 dllmask)
{
	void __iomem *base = q->iobase;
	u32 dllcra;

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
		 QUADSPI_DLLSR_DLLA_LOCK))
		;

	dllcra = qspi_readl(q, base + QUADSPI_DLLCRA);
	dllcra &= ~QUADSPI_DLLCR_SLV_BYPASS_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);

	dllcra |= QUADSPI_DLLCR_SLV_UPD_EN;
	qspi_writel(q, dllcra, base + QUADSPI_DLLCRA);
	while (!(qspi_readl(q, base + QUADSPI_DLLSR) &
		 QUADSPI_DLLSR_SLVA_LOCK))
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

	dev_err(q->dev, "Error: Failed to detect a correct mode for dllcr: 0x%X\n",
		dllcra);

	return -1;
}

static int enable_octal_ddr(struct fsl_qspi *q)
{
	void __iomem *base = q->iobase;
	u32 mcr, dllcr, smpr;
	int ret;

	if (q->proto == SNOR_PROTO_8_8_8_DTR)
		return 0;

	while (qspi_readl(q, base + QUADSPI_SR) & QUADSPI_SR_BUSY)
		;

	/* Disable the module */
	mcr = qspi_readl(q, base + QUADSPI_MCR);
	mcr |= QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	mcr |= octal_ddr_conf.mcr;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	if (is_s32g3_qspi(q))
		smpr = QUADSPI_SMPR_DLLFSMPFA_NTH(3) |
			QUADSPI_SMPR_DLLFSMPFB_NTH(3);
	else
		smpr = octal_ddr_conf.smpr;

	qspi_writel(q, octal_ddr_conf.flshcr, base + QUADSPI_FLSHCR);
	qspi_writel(q, octal_ddr_conf.sfacr, base + QUADSPI_SFACR);
	qspi_writel(q, smpr, base + QUADSPI_SMPR);
	qspi_writel(q, octal_ddr_conf.dlcr, base + QUADSPI_DLCR);

	qspi_writel(q, octal_ddr_conf.dlpr, base + QUADSPI_DLPR);

	/* Init AHB interface - 1024 bytes */
	qspi_writel(q, QUADSPI_BUF3CR_ALLMST_MASK |
		    (q->devtype_data->rxfifo << QUADSPI_BUF3CR_ADATSZ_SHIFT),
		    base + QUADSPI_BUF3CR);

	/* Enable the module */
	mcr = qspi_readl(q, base + QUADSPI_MCR);
	mcr &= ~QUADSPI_MCR_MDIS_MASK;
	qspi_writel(q, mcr, base + QUADSPI_MCR);

	dllcr = octal_ddr_conf.dllcr;
	if (low_freq_chain(q))
		dllcr &= ~QUADSPI_DLLCR_FREQEN_EN;

	ret = program_dllcra(q, dllcr);
	if (ret) {
		dev_err(q->dev, "Error: Failed to apply dllcra settings\n");
		return ret;
	}

	q->proto = SNOR_PROTO_8_8_8_DTR;

	return 0;
}

static bool is_octal_dtr_op(const struct spi_mem_op *op)
{
	/* Command */
	if (op->cmd.buswidth == 8 && op->cmd.dtr &&
	    op->data.dir == SPI_MEM_NO_DATA)
		return true;

	if (op->cmd.buswidth == 8 && op->cmd.dtr &&
	    op->addr.buswidth == 8 && op->addr.dtr &&
	    op->dummy.buswidth == 8 && op->dummy.dtr)
		return true;

	return false;
}

static bool is_spi_op(const struct spi_mem_op *op)
{
	if (op->cmd.buswidth == 1 && !op->cmd.dtr)
		return true;

	if (op->cmd.buswidth == 1 && !op->cmd.dtr &&
	    op->addr.buswidth == 1 && !op->addr.dtr &&
	    op->dummy.buswidth == 1 && !op->dummy.dtr)
		return true;

	return false;
}

static int fsl_qspi_default_setup(struct fsl_qspi *q)
{
	void __iomem *base = q->iobase;
	u32 reg, addr_offset = 0;
	int ret = 0;

	if (!is_s32cc_qspi(q)) {
		/* disable and unprepare clock to avoid glitch pass to controller */
		fsl_qspi_clk_disable_unprep(q);

		/* the default frequency, we will change it later if necessary. */
		ret = clk_set_rate(q->clk, 66000000);
		if (ret)
			return ret;

		ret = fsl_qspi_clk_prep_enable(q);
		if (ret)
			return ret;
	}

	if (!q->from_str && q->proto == SNOR_PROTO_1_1_1)
		return ret;

	/* Reset the module */
	qspi_writel(q, QUADSPI_MCR_SWRSTSD_MASK | QUADSPI_MCR_SWRSTHD_MASK,
		    base + QUADSPI_MCR);
	udelay(1);

	/* Disable the module */
	qspi_writel(q, QUADSPI_MCR_MDIS_MASK | QUADSPI_MCR_RESERVED_MASK,
		    base + QUADSPI_MCR);

	/*
	 * Previous boot stages (BootROM, bootloader) might have used DDR
	 * mode and did not clear the TDH bits. As we currently use SDR mode
	 * only, clear the TDH bits if necessary.
	 */
	if (needs_tdh_setting(q))
		qspi_writel(q, qspi_readl(q, base + QUADSPI_FLSHCR) &
			    ~QUADSPI_FLSHCR_TDH_MASK,
			    base + QUADSPI_FLSHCR);

	reg = qspi_readl(q, base + QUADSPI_SMPR);
	reg &= ~(QUADSPI_SMPR_FSDLY_MASK
		 | QUADSPI_SMPR_FSPHS_MASK
		 | QUADSPI_SMPR_HSENA_MASK
		 | QUADSPI_SMPR_DDRSMP_MASK);

	if (has_dll(q)) {
		reg &= ~(QUADSPI_SMPR_DLLFSMPFA_MASK |
			 QUADSPI_SMPR_DLLFSMPFB_MASK);
		reg |= QUADSPI_SMPR_FSPHS_MASK;
	}

	qspi_writel(q, reg, base + QUADSPI_SMPR);

	/* We only use the buffer3 for AHB read */
	qspi_writel(q, 0, base + QUADSPI_BUF0IND);
	qspi_writel(q, 0, base + QUADSPI_BUF1IND);
	qspi_writel(q, 0, base + QUADSPI_BUF2IND);

	if (can_read_entire_ahb(q))
		qspi_writel(q, QUADSPI_BFGENCR_SEQID(SEQID_LUT_AHB),
			    q->iobase + QUADSPI_BFGENCR);
	else
		qspi_writel(q, QUADSPI_BFGENCR_SEQID(SEQID_LUT),
			    q->iobase + QUADSPI_BFGENCR);
	qspi_writel(q, QUADSPI_RBCT_WMRK_MASK, base + QUADSPI_RBCT);
	qspi_writel(q, QUADSPI_BUF3CR_ALLMST_MASK |
		    QUADSPI_BUF3CR_ADATSZ(q->devtype_data->ahb_buf_size / 8),
		    base + QUADSPI_BUF3CR);

	if (needs_amba_base_offset(q))
		addr_offset = q->memmap_phy;

	/*
	 * In HW there can be a maximum of four chips on two buses with
	 * two chip selects on each bus. We use four chip selects in SW
	 * to differentiate between the four chips.
	 * We use ahb_buf_size for each chip and set SFA1AD, SFA2AD, SFB1AD,
	 * SFB2AD accordingly.
	 */
	if (!is_s32cc_qspi(q)) {
		qspi_writel(q, q->devtype_data->ahb_buf_size + addr_offset,
			    base + QUADSPI_SFA1AD);
		qspi_writel(q, q->devtype_data->ahb_buf_size * 2 + addr_offset,
			    base + QUADSPI_SFA2AD);
		qspi_writel(q, q->devtype_data->ahb_buf_size * 3 + addr_offset,
			    base + QUADSPI_SFB1AD);
		qspi_writel(q, q->devtype_data->ahb_buf_size * 4 + addr_offset,
			    base + QUADSPI_SFB2AD);
	} else {
		qspi_writel(q, q->devtype_data->flash1_size, base + QUADSPI_SFA1AD);
		qspi_writel(q, q->devtype_data->flash2_size, base + QUADSPI_SFA2AD);
		qspi_writel(q, q->devtype_data->flash1_size, base + QUADSPI_SFB1AD);
		qspi_writel(q, q->devtype_data->flash2_size, base + QUADSPI_SFB2AD);
	}

	q->selected = -1;

	/* Enable the module */
	reg = QUADSPI_MCR_RESERVED_MASK | QUADSPI_MCR_END_CFG_MASK;

	if (sdr_internal_loopback(q))
		reg |= QUADSPI_MCR_DQS_INT_PAD;

	if (has_dll(q)) {
		reg |= QUADSPI_MCR_DQS_EN;
		qspi_writel(q, 0, base + QUADSPI_SFACR);
	}

	qspi_writel(q, reg, base + QUADSPI_MCR);

	if (has_dll(q))
		ret = program_dllcra(q, QUADSPI_DLLCR_SLV_BYPASS_EN |
				     QUADSPI_DLLCR_SLV_EN);

	if (!ret)
		q->proto = SNOR_PROTO_1_1_1;
	if (q->from_str)
		q->from_str = false;

	if (!is_s32cc_qspi(q)) {
		/* clear all interrupt status */
		qspi_writel(q, 0xffffffff, q->iobase + QUADSPI_FR);

		/* enable the interrupt */
		qspi_writel(q, QUADSPI_RSER_TFIE, q->iobase + QUADSPI_RSER);
	}

	return 0;
}

static int fsl_qspi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct fsl_qspi *q = spi_controller_get_devdata(mem->spi->master);
	void __iomem *base = q->iobase;
	u32 addr_offset = 0;
	int err = 0;
	int invalid_mstrid = q->devtype_data->invalid_mstrid;

	mutex_lock(&q->lock);

	/* wait for the controller being ready */
	fsl_qspi_readl_poll_tout(q, base + QUADSPI_SR, (QUADSPI_SR_IP_ACC_MASK |
				 QUADSPI_SR_AHB_ACC_MASK | QUADSPI_SR_BUSY),
				 10, 1000);

	fsl_qspi_select_mem(q, mem->spi);

	if (needs_amba_base_offset(q))
		addr_offset = q->memmap_phy;

	if (can_read_entire_ahb(q)) {
		if (op->addr.nbytes)
			addr_offset += op->addr.val;
	}

	if (is_s32cc_qspi(q))
		qspi_writel(q,
			    q->selected * fsl_qspi_memsize_per_cs(q) + addr_offset,
			    base + QUADSPI_SFAR);
	else
		qspi_writel(q,
			    q->selected * q->devtype_data->ahb_buf_size + addr_offset,
			    base + QUADSPI_SFAR);

	qspi_writel(q, qspi_readl(q, base + QUADSPI_MCR) |
		    QUADSPI_MCR_CLR_RXF_MASK | QUADSPI_MCR_CLR_TXF_MASK,
		    base + QUADSPI_MCR);

	qspi_writel(q, QUADSPI_SPTRCLR_BFPTRC | QUADSPI_SPTRCLR_IPPTRC,
		    base + QUADSPI_SPTRCLR);

	qspi_writel(q, invalid_mstrid, base + QUADSPI_BUF0CR);
	qspi_writel(q, invalid_mstrid, base + QUADSPI_BUF1CR);
	qspi_writel(q, invalid_mstrid, base + QUADSPI_BUF2CR);

	fsl_qspi_prepare_lut(q, op);

	if (is_octal_dtr_op(op))
		enable_octal_ddr(q);

	if (is_spi_op(op))
		fsl_qspi_default_setup(q);

	/*
	 * If we have large chunks of data, we read them through the AHB bus
	 * by accessing the mapped memory. In all other cases we use
	 * IP commands to access the flash.
	 */
	if (op->data.nbytes > (q->devtype_data->rxfifo - 4) &&
	    op->data.dir == SPI_MEM_DATA_IN) {
		qspi_writel(q, QUADSPI_RBCT_WMRK_MASK |
			    0, base + QUADSPI_RBCT);
		fsl_qspi_read_ahb(q, op);
	} else {
		qspi_writel(q, QUADSPI_RBCT_WMRK_MASK |
			    QUADSPI_RBCT_RXBRD_USEIPS, base + QUADSPI_RBCT);

		if (op->data.nbytes && op->data.dir == SPI_MEM_DATA_OUT)
			err = fsl_qspi_fill_txfifo(q, op);

		if (!err)
			err = fsl_qspi_do_op(q, op);
	}

	/* Invalidate the data in the AHB buffer. */
	fsl_qspi_invalidate(q);

	mutex_unlock(&q->lock);

	return err;
}

static int fsl_qspi_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	struct fsl_qspi *q = spi_controller_get_devdata(mem->spi->master);

	if (op->data.dir == SPI_MEM_DATA_OUT) {
		if (op->data.nbytes > q->devtype_data->txfifo)
			op->data.nbytes = q->devtype_data->txfifo;
	} else {
		if (op->data.nbytes > q->devtype_data->ahb_buf_size) {
			if (!can_read_entire_ahb(q))
				op->data.nbytes = q->devtype_data->ahb_buf_size;
		} else if (op->data.nbytes > (q->devtype_data->rxfifo - 4)) {
			op->data.nbytes = ALIGN_DOWN(op->data.nbytes, 8);
		}
	}

	return 0;
}

static const char *fsl_qspi_get_name(struct spi_mem *mem)
{
	struct fsl_qspi *q = spi_controller_get_devdata(mem->spi->master);
	struct device *dev = &mem->spi->dev;
	const char *name;

	/*
	 * In order to keep mtdparts compatible with the old MTD driver at
	 * mtd/spi-nor/fsl-quadspi.c, we set a custom name derived from the
	 * platform_device of the controller.
	 */
	if (of_get_available_child_count(q->dev->of_node) == 1)
		return dev_name(q->dev);

	name = devm_kasprintf(dev, GFP_KERNEL,
			      "%s-%d", dev_name(q->dev),
			      mem->spi->chip_select);

	if (!name) {
		dev_err(dev, "failed to get memory for custom flash name\n");
		return ERR_PTR(-ENOMEM);
	}

	return name;
}

static const struct spi_controller_mem_ops fsl_qspi_mem_ops = {
	.adjust_op_size = fsl_qspi_adjust_op_size,
	.supports_op = fsl_qspi_supports_op,
	.exec_op = fsl_qspi_exec_op,
	.get_name = fsl_qspi_get_name,
};

static int fsl_qspi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	struct fsl_qspi *q;
	int ret;

	ctlr = spi_alloc_master(&pdev->dev, sizeof(*q));
	if (!ctlr)
		return -ENOMEM;

	ctlr->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD | SPI_RX_OCTAL |
			  SPI_TX_DUAL | SPI_TX_QUAD | SPI_TX_OCTAL;

	q = spi_controller_get_devdata(ctlr);
	q->dev = dev;
	q->devtype_data = of_device_get_match_data(dev);
	if (!q->devtype_data) {
		ret = -ENODEV;
		goto err_put_ctrl;
	}

	platform_set_drvdata(pdev, q);

	/* find the resources */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "QuadSPI");
	q->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(q->iobase)) {
		ret = PTR_ERR(q->iobase);
		goto err_put_ctrl;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					"QuadSPI-memory");
	if (!res) {
		ret = -EINVAL;
		goto err_put_ctrl;
	}
	q->memmap_phy = res->start;
	/* Since there are 4 cs, map size required is 4 times ahb_buf_size */
	if (is_s32cc_qspi(q))
		q->ahb_addr = ioremap_cache(q->memmap_phy,
					    res->end - res->start);
	else
		q->ahb_addr = devm_ioremap(dev, q->memmap_phy,
					   (q->devtype_data->ahb_buf_size * 4));
	if (!q->ahb_addr) {
		ret = -ENOMEM;
		goto err_put_ctrl;
	}
	q->memmap_size = res->end - res->start + 1;

	/* find the clocks */
	q->clk_en = devm_clk_get(dev, "qspi_en");
	if (IS_ERR(q->clk_en)) {
		ret = PTR_ERR(q->clk_en);
		goto err_put_ctrl;
	}

	q->clk = devm_clk_get(dev, "qspi");
	if (IS_ERR(q->clk)) {
		ret = PTR_ERR(q->clk);
		goto err_put_ctrl;
	}

	ret = fsl_qspi_clk_prep_enable(q);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		goto err_put_ctrl;
	}

	/* find the irq */
	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto err_disable_clk;

	ret = devm_request_irq(dev, ret,
			fsl_qspi_irq_handler, 0, pdev->name, q);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		goto err_disable_clk;
	}

	mutex_init(&q->lock);

	ctlr->bus_num = -1;
	ctlr->num_chipselect = 4;
	ctlr->mem_ops = &fsl_qspi_mem_ops;

	q->from_str = false;
	fsl_qspi_default_setup(q);

	ctlr->dev.of_node = np;

	if (is_s32cc_qspi(q)) {
		np = of_get_next_available_child(dev->of_node, NULL);

		ret = of_property_read_u32(np, "spi-max-frequency", &q->clk_rate);
		if (ret)
			goto err_destroy_mutex;
	}

	ret = devm_spi_register_controller(dev, ctlr);
	if (ret)
		goto err_destroy_mutex;

	return 0;

err_destroy_mutex:
	mutex_destroy(&q->lock);

err_disable_clk:
	fsl_qspi_clk_disable_unprep(q);

err_put_ctrl:
	spi_controller_put(ctlr);

	dev_err(dev, "Freescale QuadSPI probe failed\n");
	return ret;
}

static int fsl_qspi_remove(struct platform_device *pdev)
{
	struct fsl_qspi *q = platform_get_drvdata(pdev);

	/* disable the hardware */
	qspi_writel(q, QUADSPI_MCR_MDIS_MASK, q->iobase + QUADSPI_MCR);
	qspi_writel(q, 0x0, q->iobase + QUADSPI_RSER);

	fsl_qspi_clk_disable_unprep(q);

	mutex_destroy(&q->lock);

	if (is_s32cc_qspi(q) && q->ahb_addr)
		iounmap(q->ahb_addr);

	return 0;
}

static int fsl_qspi_suspend(struct device *dev)
{
	struct fsl_qspi *q = dev_get_drvdata(dev);

	if (is_s32cc_qspi(q))
		fsl_qspi_clk_disable_unprep(q);

	return 0;
}

static int fsl_qspi_resume(struct device *dev)
{
	struct fsl_qspi *q = dev_get_drvdata(dev);
	int ret;

	if (is_s32cc_qspi(q)) {
		ret = clk_set_rate(q->clk, q->clk_rate);
		if (ret) {
			dev_err(dev, "Failed to set clock rate\n");
			return ret;
		}

		ret = fsl_qspi_clk_prep_enable(q);
		if (ret) {
			dev_err(dev, "Failed to enable clock\n");
			return ret;
		}

		q->from_str = true;
	}

	fsl_qspi_default_setup(q);

	return 0;
}

static const struct of_device_id fsl_qspi_dt_ids[] = {
	{ .compatible = "fsl,vf610-qspi", .data = &vybrid_data, },
	{ .compatible = "fsl,imx6sx-qspi", .data = &imx6sx_data, },
	{ .compatible = "fsl,imx7d-qspi", .data = &imx7d_data, },
	{ .compatible = "fsl,imx6ul-qspi", .data = &imx6ul_data, },
	{ .compatible = "fsl,ls1021a-qspi", .data = &ls1021a_data, },
	{ .compatible = "fsl,ls2080a-qspi", .data = &ls2080a_data, },
	{ .compatible = "nxp,s32g-qspi", .data = &s32cc_data },
	{ .compatible = "nxp,s32g3-qspi", .data = &s32g3_data },
	{ .compatible = "nxp,s32r45-qspi", .data = &s32r45_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_qspi_dt_ids);

static const struct dev_pm_ops fsl_qspi_pm_ops = {
	.suspend	= fsl_qspi_suspend,
	.resume		= fsl_qspi_resume,
};

static struct platform_driver fsl_qspi_driver = {
	.driver = {
		.name	= "fsl-quadspi",
		.of_match_table = fsl_qspi_dt_ids,
		.pm =   &fsl_qspi_pm_ops,
	},
	.probe          = fsl_qspi_probe,
	.remove		= fsl_qspi_remove,
};
module_platform_driver(fsl_qspi_driver);

MODULE_DESCRIPTION("Freescale QuadSPI Controller Driver");
MODULE_AUTHOR("Freescale Semiconductor Inc.");
MODULE_AUTHOR("Boris Brezillon <bbrezillon@kernel.org>");
MODULE_AUTHOR("Frieder Schrempf <frieder.schrempf@kontron.de>");
MODULE_AUTHOR("Yogesh Gaur <yogeshnarayan.gaur@nxp.com>");
MODULE_AUTHOR("Suresh Gupta <suresh.gupta@nxp.com>");
MODULE_LICENSE("GPL v2");
