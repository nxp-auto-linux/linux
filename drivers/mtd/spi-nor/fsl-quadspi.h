/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2017-2021 NXP
 *
 */

#ifndef _FSL_QUADSPI_H
#define _FSL_QUADSPI_H

#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/pm_qos.h>
#include <linux/spi/spi-mem.h>

/* Controller needs driver to swap endian */
#define QUADSPI_QUIRK_SWAP_ENDIAN	(1 << 0)
/* Controller needs 4x internal clock */
#define QUADSPI_QUIRK_4X_INT_CLK	(1 << 1)
/*
 * TKT253890, Controller needs driver to fill txfifo till 16 byte to
 * trigger data transfer even though extern data will not transferred.
 */
#define QUADSPI_QUIRK_TKT253890		(1 << 2)
/* Controller cannot wake up from wait mode, TKT245618 */
#define QUADSPI_QUIRK_TKT245618         (1 << 3)

#ifdef CONFIG_SOC_S32GEN1
#define RX_BUFFER_SIZE		0x80
#define TX_BUFFER_SIZE		0x100
#endif

#define FLASH_STATUS_WEL	0x02

/* The registers */
#define QUADSPI_MCR			0x00
#define QUADSPI_MCR_RESERVED_SHIFT	16
#define QUADSPI_MCR_RESERVED_MASK	(0xF << QUADSPI_MCR_RESERVED_SHIFT)
#define QUADSPI_MCR_MDIS_SHIFT		14
#define QUADSPI_MCR_MDIS_MASK		(1 << QUADSPI_MCR_MDIS_SHIFT)
#define QUADSPI_MCR_CLR_TXF_SHIFT	11
#define QUADSPI_MCR_CLR_TXF_MASK	(1 << QUADSPI_MCR_CLR_TXF_SHIFT)
#define QUADSPI_MCR_CLR_RXF_SHIFT	10
#define QUADSPI_MCR_CLR_RXF_MASK	(1 << QUADSPI_MCR_CLR_RXF_SHIFT)
#define QUADSPI_MCR_DDR_EN_SHIFT	7
#define QUADSPI_MCR_DDR_EN_MASK		(1 << QUADSPI_MCR_DDR_EN_SHIFT)
#define QUADSPI_MCR_END_CFG_SHIFT	2
#define QUADSPI_MCR_END_CFG_MASK	(3 << QUADSPI_MCR_END_CFG_SHIFT)
#define QUADSPI_MCR_SWRSTHD_SHIFT	1
#define QUADSPI_MCR_SWRSTHD_MASK	(1 << QUADSPI_MCR_SWRSTHD_SHIFT)
#define QUADSPI_MCR_SWRSTSD_SHIFT	0
#define QUADSPI_MCR_SWRSTSD_MASK	(1 << QUADSPI_MCR_SWRSTSD_SHIFT)
#ifdef CONFIG_SOC_S32GEN1
#define QUADSPI_MCR_DQS_EN_SHIFT	6
#define QUADSPI_MCR_DQS_EN			(1 << QUADSPI_MCR_DQS_EN_SHIFT)
#define QUADSPI_MCR_END_CFD_SHIFT	2
#define QUADSPI_MCR_END_CFD_LE		(3 << QUADSPI_MCR_END_CFD_SHIFT)
#define QUADSPI_MCR_DQS_FA_SEL_SHIFT	24
#define QUADSPI_MCR_DQS_LOOPBACK	(0x1 << QUADSPI_MCR_DQS_FA_SEL_SHIFT)
#define QUADSPI_MCR_DQS_PAD_LOOPBACK	(0x2 << QUADSPI_MCR_DQS_FA_SEL_SHIFT)
#define QUADSPI_MCR_DQS_EXTERNAL	(0x3 << QUADSPI_MCR_DQS_FA_SEL_SHIFT)
#define QUADSPI_MCR_DQS_MASK		(0x3 << QUADSPI_MCR_DQS_FA_SEL_SHIFT)
#define QUADSPI_MCR_DLPEN_SHIFT		12
#define QUADSPI_MCR_DLPEN_MASK		(1 << QUADSPI_MCR_DLPEN_SHIFT)
#define QUADSPI_MCR_ISD2FA_SHIFT		16
#define QUADSPI_MCR_ISD2FA_EN		(1 << QUADSPI_MCR_ISD2FA_SHIFT)
#define QUADSPI_MCR_ISD3FA_SHIFT		17
#define QUADSPI_MCR_ISD3FA_EN		(1 << QUADSPI_MCR_ISD3FA_SHIFT)
#define QUADSPI_MCR_ISD2FB_SHIFT		18
#define QUADSPI_MCR_ISD2FB_EN		(1 << QUADSPI_MCR_ISD2FB_SHIFT)
#define QUADSPI_MCR_ISD3FB_SHIFT		19
#define QUADSPI_MCR_ISD3FB_EN		(1 << QUADSPI_MCR_ISD3FB_SHIFT)
#endif

#ifdef CONFIG_SOC_S32GEN1
#define QUADSPI_DLLCR_SLV_UPD_SHIFT			0
#define QUADSPI_DLLCR_SLV_UPD_EN	(1 << QUADSPI_DLLCR_SLV_UPD_SHIFT)
#define QUADSPI_DLLCR_SLV_BYPASS_SHIFT		1
#define QUADSPI_DLLCR_SLV_BYPASS_EN	(1 << QUADSPI_DLLCR_SLV_BYPASS_SHIFT)
#define QUADSPI_DLLCR_SLV_EN_SHIFT			2
#define QUADSPI_DLLCR_SLV_EN	(1 << QUADSPI_DLLCR_SLV_EN_SHIFT)
#define QUADSPI_DLLCR_SLV_AUTO_UPDT_SHIFT	3
#define QUADSPI_DLLCR_SLV_AUTO_UPDT_EN	(1 << QUADSPI_DLLCR_SLV_AUTO_UPDT_SHIFT)
#define QUADSPI_DLLCR_SLV_DLY_COARSE_SHIFT	8
#define QUADSPI_DLLCR_SLV_DLY_COARSE_N(N)	((N) << \
		QUADSPI_DLLCR_SLV_DLY_COARSE_SHIFT)
#define QUADSPI_DLLCR_DLLRES_SHIFT			20
#define QUADSPI_DLLCR_DLLRES_N(N)	((N) << QUADSPI_DLLCR_DLLRES_SHIFT)
#define QUADSPI_DLLCR_DLL_REFCNTR_SHIFT		24
#define QUADSPI_DLLCR_DLL_REFCNTR_N(N)	((N) << QUADSPI_DLLCR_DLL_REFCNTR_SHIFT)
#define QUADSPI_DLLCR_FREQEN_SHIFT			30
#define QUADSPI_DLLCR_FREQEN_EN	(1 << QUADSPI_DLLCR_FREQEN_SHIFT)
#define QUADSPI_DLLCR_DLLEN_SHIFT	31
#define QUADSPI_DLLCR_DLLEN_EN		(1 << QUADSPI_DLLCR_DLLEN_SHIFT)
#define QUADSPI_DLLCR_MASK			0x7FFFFFF0UL
#endif

#define QUADSPI_IPCR			0x08
#define QUADSPI_IPCR_SEQID_SHIFT	24
#define QUADSPI_IPCR_SEQID_MASK		(0xF << QUADSPI_IPCR_SEQID_SHIFT)

#define QUADSPI_BUF0CR			0x10
#define QUADSPI_BUF1CR			0x14
#define QUADSPI_BUF2CR			0x18
#define QUADSPI_BUFXCR_INVALID_MSTRID	0xe

#define QUADSPI_BUF3CR			0x1c
#define QUADSPI_BUF3CR_ALLMST_SHIFT	31
#define QUADSPI_BUF3CR_ALLMST_MASK	(1 << QUADSPI_BUF3CR_ALLMST_SHIFT)
#define QUADSPI_BUF3CR_ADATSZ_SHIFT		8
#define QUADSPI_BUF3CR_ADATSZ_MASK	(0xFF << QUADSPI_BUF3CR_ADATSZ_SHIFT)

#define QUADSPI_BFGENCR			0x20
#define QUADSPI_BFGENCR_PAR_EN_SHIFT	16
#define QUADSPI_BFGENCR_PAR_EN_MASK	(1 << (QUADSPI_BFGENCR_PAR_EN_SHIFT))
#define QUADSPI_BFGENCR_SEQID_SHIFT	12
#define QUADSPI_BFGENCR_SEQID_MASK	(0xF << QUADSPI_BFGENCR_SEQID_SHIFT)

#define QUADSPI_BUF0IND			0x30
#define QUADSPI_BUF1IND			0x34
#define QUADSPI_BUF2IND			0x38
#define QUADSPI_SFAR			0x100

#ifdef CONFIG_SOC_S32GEN1
#define QUADSPI_SFACR			0x104
#define QUADSPI_AWRCR			0x50
#define QUADSPI_DLLCRA			0x60
#endif

#define QUADSPI_SMPR_DLLFSMPFA_SHIFT	24
#define QUADSPI_SMPR_DLLFSMPFA_NTH(N)	((N) << QUADSPI_SMPR_DLLFSMPFA_SHIFT)
#define QUADSPI_SMPR_DLLFSMPFB_SHIFT	28
#define QUADSPI_SMPR_DLLFSMPFB_NTH(N)	((N) << QUADSPI_SMPR_DLLFSMPFB_SHIFT)

#define QUADSPI_FLSHCR				0xC
#define QUADSPI_FLSHCR_TCSS_SHIFT	0
#define QUADSPI_FLSHCR_TCSS(N)		((N) << QUADSPI_FLSHCR_TCSS_SHIFT)
#define QUADSPI_FLSHCR_TCHS_SHIFT	8
#define QUADSPI_FLSHCR_TCHS(N)		((N) << QUADSPI_FLSHCR_TCHS_SHIFT)
#define QUADSPI_FLSHCR_TDH_SHIFT	16
#define QUADSPI_FLSHCR_TDH(N)		((N) << QUADSPI_FLSHCR_TDH_SHIFT)

#define QUADSPI_DLCR					0x130
#define QUADSPI_DLCR_RESERVED_MASK		((0xff << 0) | (0xff << 16))
#define QUADSPI_DLCR_DLP_SEL_FA_SHIFT	14
#define QUADSPI_DLCR_DLP_SEL_FA(N)	((N) << QUADSPI_DLCR_DLP_SEL_FA_SHIFT)
#define QUADSPI_DLCR_DLP_SEL_FB_SHIFT	30
#define QUADSPI_DLCR_DLP_SEL_FB(N)	((N) << QUADSPI_DLCR_DLP_SEL_FB_SHIFT)

#define QUADSPI_DLPR	0x190
#define QUADSPI_DLPR_RESET_VALUE		0xaa553443

#define QUADSPI_SFACR_BSWAP_SHIFT		17
#define QUADSPI_SFACR_BSWAP_EN			(1 << QUADSPI_SFACR_BSWAP_SHIFT)

#define QUADSPI_SMPR			0x108
#define QUADSPI_SMPR_DDRSMP_SHIFT	16
#define QUADSPI_SMPR_DDRSMP_MASK	(7 << QUADSPI_SMPR_DDRSMP_SHIFT)
#define QUADSPI_SMPR_FSDLY_SHIFT	6
#define QUADSPI_SMPR_FSDLY_MASK		(1 << QUADSPI_SMPR_FSDLY_SHIFT)
#define QUADSPI_SMPR_FSPHS_SHIFT	5
#define QUADSPI_SMPR_FSPHS_MASK		(1 << QUADSPI_SMPR_FSPHS_SHIFT)
#define QUADSPI_SMPR_HSENA_SHIFT	0
#define QUADSPI_SMPR_HSENA_MASK		(1 << QUADSPI_SMPR_HSENA_SHIFT)

#define QUADSPI_RBSR			0x10c
#define QUADSPI_RBSR_RDBFL_SHIFT	8
#define QUADSPI_RBSR_RDBFL_MASK		(0x3F << QUADSPI_RBSR_RDBFL_SHIFT)

#define QUADSPI_RBCT			0x110
#define QUADSPI_RBCT_WMRK_MASK		0x1F
#define QUADSPI_RBCT_RXBRD_SHIFT	8
#define QUADSPI_RBCT_RXBRD_USEIPS	(0x1 << QUADSPI_RBCT_RXBRD_SHIFT)

#define QUADSPI_DLLSR			0x12C
#define QUADSPI_DLLSR_SLVA_LOCK_SHIFT	14
#define QUADSPI_DLLSR_SLVA_LOCK_MASK	(1 << QUADSPI_DLLSR_SLVA_LOCK_SHIFT)
#define QUADSPI_DLLSR_DLLA_LOCK_SHIFT	15
#define QUADSPI_DLLSR_DLLA_LOCK_MASK	(1 << QUADSPI_DLLSR_DLLA_LOCK_SHIFT)

#define QUADSPI_TBSR			0x150
#define QUADSPI_TBDR			0x154
#ifdef CONFIG_SOC_S32GEN1
#define QUADSPI_TBSR_TRCTR_SHIFT	16
#define QUADSPI_TBSR_TRCTR(TBSR)	((TBSR) >> QUADSPI_TBSR_TRCTR_SHIFT)
#define QUADSPI_TBSR_TRBFL(TBSR)	((TBSR) & 0xFF)
#endif
#define QUADSPI_SR			0x15c
#define QUADSPI_SR_IP_ACC_SHIFT		1
#define QUADSPI_SR_IP_ACC_MASK		(0x1 << QUADSPI_SR_IP_ACC_SHIFT)
#define QUADSPI_SR_AHB_ACC_SHIFT	2
#define QUADSPI_SR_AHB_ACC_MASK		(0x1 << QUADSPI_SR_AHB_ACC_SHIFT)
#ifdef CONFIG_SOC_S32GEN1
#define QUADSPI_SR_BUSY_SHIFT		0
#define QUADSPI_SR_BUSY_MASK		(1 << QUADSPI_SR_BUSY_SHIFT)
#endif

#define QUADSPI_FR			0x160
#define QUADSPI_FR_TFF_MASK		0x1
#ifdef CONFIG_SOC_S32GEN1
#define QUADSPI_FR_ALL_FLAGS_MASK	(0xFFFFFFFF)
#endif

#define QUADSPI_SFA1AD			0x180
#define QUADSPI_SFA2AD			0x184
#define QUADSPI_SFB1AD			0x188
#define QUADSPI_SFB2AD			0x18c
#ifdef CONFIG_SOC_S32GEN1
#define QUADSPI_SFA_ADDR		0x10000000
#define QUADSPI_SFB_ADDR		0x20000000
#endif
#define QUADSPI_RBDR			0x200

#define QUADSPI_LUTKEY			0x300
#define QUADSPI_LUTKEY_VALUE		0x5AF05AF0

#define QUADSPI_LCKCR			0x304
#define QUADSPI_LCKER_LOCK		0x1
#define QUADSPI_LCKER_UNLOCK		0x2

#define QUADSPI_RSER			0x164
#define QUADSPI_RSER_TFIE		(0x1 << 0)

#define QUADSPI_LUT_BASE		0x310

#define QUADSPI_AHB_BASE		0x0
#define QUADSPI_AHB_SIZE		SZ_64M

/*
 * The definition of the LUT register shows below:
 *
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */
#define OPRND0_SHIFT		0
#define PAD0_SHIFT		8
#define INSTR0_SHIFT		10
#define OPRND1_SHIFT		16
#ifdef CONFIG_SOC_S32GEN1
#define PAD1_SHIFT			24
#define INSTR1_SHIFT		26
#define OPRND0(x)           ((x) << OPRND0_SHIFT)
#define PAD0(x)             ((x) << PAD0_SHIFT)
#define INSTR0(x)           ((x) << INSTR0_SHIFT)
#define OPRND1(x)           ((x) << OPRND1_SHIFT)
#define PAD1(x)             ((x) << PAD1_SHIFT)
#define INSTR1(x)           ((x) << INSTR1_SHIFT)
#define LUT2PAD0(x)			(((x) >> PAD0_SHIFT) & 0x3)
#define LUT2INSTR0(x)		(((x) >> INSTR0_SHIFT) & 0x3f)
#endif


/* Instruction set for the LUT register. */
#define LUT_CMD				1
#define LUT_ADDR			2
#define LUT_DUMMY			3
#define LUT_FSL_READ		7
#define LUT_FSL_WRITE		8
#define LUT_ADDR_DDR		10
#define LUT_CMD_DDR			17
#define LUT_READ_DDR		14
#define LUT_WRITE_DDR		15
#ifndef CONFIG_SOC_S32GEN1
#define LUT_STOP			0
#define LUT_MODE			4
#define LUT_MODE2			5
#define LUT_MODE4			6
#define LUT_JMP_ON_CS		9
#define LUT_MODE_DDR		11
#define LUT_MODE2_DDR		12
#define LUT_MODE4_DDR		13
#define LUT_DATA_LEARN		16
#endif

/*
 * The PAD definitions for LUT register.
 *
 * The pad stands for the lines number of IO[0:3].
 * For example, the Quad read need four IO lines, so you should
 * set LUT_PAD4 which means we use four IO lines.
 */
#define LUT_PAD1		0
#define LUT_PAD2		1
#define LUT_PAD4		2

/* Oprands for the LUT register. */
#define ADDR24BIT		0x18
#define ADDR32BIT		0x20

/* Macros for constructing the LUT register. */
#define LUT0(ins, pad, opr)						\
		(((opr) << OPRND0_SHIFT) | ((LUT_##pad) << PAD0_SHIFT) | \
		((LUT_##ins) << INSTR0_SHIFT))

#define LUT1(ins, pad, opr)	(LUT0(ins, pad, opr) << OPRND1_SHIFT)

/* other macros for LUT register. */
#define QUADSPI_LUT(x)          (QUADSPI_LUT_BASE + (x) * 4)
#define QUADSPI_LUT_NUM		64

/* SEQID -- we can have 16 seqids at most. */
#define SEQID_READ		0
#define SEQID_WREN		1
#define SEQID_WRDI		2
#define SEQID_RDSR		3
#define SEQID_SE		4
#define SEQID_CHIP_ERASE	5
#define SEQID_PP		6
#define SEQID_RDID		7
#define SEQID_WRSR		8
#define SEQID_RDCR		9
#define SEQID_EN4B		10
#define SEQID_BRWR		11
#define SEQID_FAST_READ		12

#define QUADSPI_MIN_IOMAP SZ_4M

#ifdef CONFIG_SOC_S32GEN1
#define QUADSPI_CMD_RDSR       0x05    /* Read status register */
#define QUADSPI_CMD_WREN       0x06    /* Write enable */
#define QUADSPI_CMD_CHIP_ERASE 0xc7    /* Erase whole flash chip */
#define QUADSPI_CMD_RDID       0x9f    /* Read JEDEC ID */
#define QUADSPI_CMD_SE_4B      0xdc    /* Sector erase (usually 64KiB) */
/* Read data bytes (high frequency) */
#define QUADSPI_CMD_FAST_READ_4B	0x0c
#define QUADSPI_CMD_PP_4B      0x12    /* Page program (up to 256 bytes) */
#endif

#define QUADSPI_FLAG_REGMAP_ENDIAN_BIG	BIT(0)
#define QUADSPI_FLAG_PREV_READ_MEM		BIT(1)

#define FSL_QSPI_MAX_CHIP   4

enum fsl_qspi_devtype {
	FSL_QUADSPI_VYBRID,
	FSL_QUADSPI_IMX6SX,
	FSL_QUADSPI_IMX7D,
	FSL_QUADSPI_IMX6UL,
	FSL_QUADSPI_LS1021A,
	FSL_QUADSPI_S32V234,
	FSL_QUADSPI_S32GEN1,
	FSL_QUADSPI_LS2080A,
};

struct fsl_qspi_devtype_data {
	enum fsl_qspi_devtype devtype;
	int rxfifo;
	int txfifo;
	int ahb_buf_size;
	int driver_data;
};

struct fsl_qspi {
	struct spi_nor nor[FSL_QSPI_MAX_CHIP];
	void __iomem *iobase;
	void __iomem *ahb_addr;
	u32 memmap_phy;
	u32 memmap_offs;
	u32 memmap_len;
	struct clk *clk, *clk_en;
	struct device *dev;
	struct completion c;
	const struct fsl_qspi_devtype_data *devtype_data;
	u32 nor_size;
	u32 nor_num;
	u32 clk_rate;
	unsigned int chip_base_addr; /* We may support two chips. */
	bool has_second_chip;
	bool big_endian;
	struct mutex lock;
	struct pm_qos_request pm_qos_req;
#ifdef CONFIG_SOC_S32GEN1
	const struct spi_mem_op *s32gen1_mem_op;
	bool ddr_mode;
	u32 num_pads;
	u32 flags;
#endif
};

void qspi_writel(struct fsl_qspi *q, u32 val, void __iomem *addr);
u32 qspi_readl(struct fsl_qspi *q, void __iomem *addr);
void reset_bootrom_settings(struct fsl_qspi *q);
int enable_spi(struct fsl_qspi *q, bool force);
int s32gen1_qspi_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len);
int s32gen1_qspi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len);
ssize_t s32gen1_qspi_read(struct spi_nor *nor, loff_t from,
		size_t len, u_char *buf);
ssize_t s32gen1_qspi_write(struct spi_nor *nor, loff_t to,
		size_t len, const u_char *buf);
int s32gen1_exec_op(struct spi_nor *nor, const struct spi_mem_op *op);
bool s32gen1_supports_op(struct spi_nor *nor, const struct spi_mem_op *op);

#endif /* _FSL_QUADSPI_H */
