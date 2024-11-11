// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2024 NXP
 *
 * CAR-V NET remoteproc driver
 *
 */

#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>

#include "remoteproc_internal.h"

static bool auto_boot;
module_param(auto_boot, bool, 0400);
MODULE_PARM_DESC(auto_boot,
		 "Auto-boot the CAR-V-NET remote processor [default=false]");

#define CARVNET_FW_ALIGN	0x4000U

#define CARVNET_BAR_FW		0
#define CARVNET_BAR_VDEV	1

/* CAR-V Internal Memory Type and Access */

#define CARV_MEMTYPE_MASK			GENMASK(31, 28)

/* Read-only code region */
#define CARV_MEMTYPE_RO_CODE			0x02000000U
/* Local-private and DTCM region */
#define CARV_MEMTYPE_LOCAL_PRIVATE_DTCM		0x10000000U
/* Read-only data with coarse-grained mapping */
#define CARV_MEMTYPE_RO_DATA_COARSE_GRAINED	0x20000000U
/* Read-only data with fine-grained mapping */
#define CARV_MEMTYPE_RO_DATA_FINE_GRAINED	0x40000000U
/* Normal read/write memory with fine-grained mapping */
#define CARV_MEMTYPE_RW_DATA_FINE_GRAINED	0x80000000U
/* Normal read/write memory with coarse-grained mapping */
#define CARV_MEMTYPE_RW_DATA_COARSE_GRAINED	0xC0000000U

#define CARV_ADDR_GET_MEMTYPE(v)	((v) >> 30)
#define CARV_ADDR_GET_MEMSUBTYPE(v)	(((v) >> 29) & 0x1)

/* CAR-V NoCore registers */

/* Global */
#define CARV_GL_CR		0x0
#define CARV_GL_AHBER		0x10	/* deprecated */
#define CARV_GL_IMR		0x20
#define CARV_GL_FMR		0x24
#define CARV_GL_PCR		0x80
#define CARV_GL_UCR		0x84
#define CARV_GL_UPR		0x88
#define CARV_GL_AHBCR		0x8C
#define CARV_GL_USR		0x90
#define CARV_GL_INFO		0x100

#define CARV_GL_CR_IP_VID_MASK	GENMASK(31, 16)
#define GET_CARV_GL_CR_IP_VID(v)	(((v) & CARV_GL_CR_IP_VID_MASK) >> 16)
#define CARV_GL_CR_LOCK_MASK	BIT(2)
#define CARV_GL_CR_LOCK(v)	(((v) << 2) & CARV_GL_CR_LOCK_MASK)
#define GET_CARV_GL_CR_LOCK(v)	(((v) & CARV_GL_CR_LOCK_MASK) >> 2)
#define CARV_GL_CR_CLOCK_MASK	BIT(1)
#define CARV_GL_CR_CLOCK(v)	(((v) << 1) & CARV_GL_CR_CLOCK_MASK)
#define GET_CARV_GL_CR_CLOCK(v)	(((v) & CARV_GL_CR_CLOCK_MASK) >> 1)
#define CARV_GL_CR_ENA_MASK	BIT(0)
#define CARV_GL_CR_ENA(v)	((v) & CARV_GL_CR_ENA_MASK)
#define GET_CARV_GL_CR_ENA(v)	((v) & CARV_GL_CR_ENA_MASK)

#define CARV_GL_PCR_Q_POS_0	(0x0 << 2)
#define CARV_GL_PCR_Q_POS_1	(0x1 << 2)
#define CARV_GL_PCR_Q_POS_2	(0x2 << 2)
#define CARV_GL_PCR_Q_POS_3	(0x3 << 2)
#define CARV_GL_PCR_ONE_Q_ISO	(0x0)
#define CARV_GL_PCR_TWO_Q_ISO	(0x1)
#define CARV_GL_PCR_FOUR_Q_ISO	(0x2)

#define CARV_GL_UCR_USERCNT1	(0x0)
#define CARV_GL_UCR_USERCNT2	(0x1)
#define CARV_GL_UCR_USERCNT3	(0x2)
#define CARV_GL_UCR_USERCNT4	(0x3)

#define CARV_GL_UPR_PRI0(v)	((v) & 0x3)
#define CARV_GL_UPR_PRI1(v)	(((v) & 0x3) << 2)
#define CARV_GL_UPR_PRI2(v)	(((v) & 0x3) << 4)
#define CARV_GL_UPR_PRI3(v)	(((v) & 0x3) << 6)

/* User set array (set by GL_UCR) */
#define CARV_Q_USER_OFFSET(u)	((u & 3) * 0x10000)			/* u: 0..3 */
#define CARV_Q_BASE(u)		(0x10000 + CARV_Q_USER_OFFSET(u))

/* Quartile Mailbox */
#define CARV_Q_MB_CR(u)		(CARV_Q_BASE(u) + 0x0)
#define CARV_Q_MB_IR(u)		(CARV_Q_BASE(u) + 0x4)
#define CARV_Q_MB_IER(u)	(CARV_Q_BASE(u) + 0x8)
#define CARV_Q_MB_FER(u)	(CARV_Q_BASE(u) + 0xC)
#define CARV_Q_MB_INFO_CR(u, n)	(CARV_Q_BASE(u) + 0x10 + ((n & 3) * 0x4))	/* n: 0..2 */
#define CARV_Q_MB_MIR(u)	(CARV_Q_BASE(u) + 0x20)
#define CARV_Q_MBOX_IFMSK(u)	(CARV_Q_BASE(u) + 0x40)

#define CARV_Q_MB_CR_INFO_MASK		GENMASK(31, 16)
#define CARV_Q_MB_CR_INFO(v)		(((v & 0xffff) << 16) & CARV_Q_MB_CR_INFO_MASK)
#define GET_CARV_Q_MB_CR_INFO(v)	(((v) & CARV_Q_MB_CR_INFO_MASK) >> 16)
#define CARV_Q_MB_CR_ERRORS_MASK	GENMASK(15, 12)
#define CARV_Q_MB_CR_ERRORS(v)		(((v & 15) << 12) & CARV_Q_MB_CR_ERRORS_MASK)
#define GET_CARV_Q_MB_CR_ERRORS(v)	(((v) & CARV_Q_MB_CR_ERRORS_MASK) >> 12)
#define CARV_Q_MB_CR_RSTCHG_MASK	BIT(9)
#define CARV_Q_MB_CR_RSTCHG(v)		(((v & 1) << 9) & CARV_Q_MB_CR_RSTCHG_MASK)
#define GET_CARV_Q_MB_CR_RSTCHG(v)	(((v) & CARV_Q_MB_CR_RSTCHG_MASK) >> 9)
#define CARV_Q_MB_CR_RSTSTART_MASK	BIT(8)
#define CARV_Q_MB_CR_RSTSTART(v)	(((v & 1) << 8) & CARV_Q_MB_CR_RSTSTART_MASK)
#define GET_CARV_Q_MB_CR_RSTSTART(v)	(((v) & CARV_Q_MB_CR_RSTSTART_MASK) >> 8)
#define CARV_Q_MB_CR_UPDATE_MASK	BIT(4)
#define CARV_Q_MB_CR_UPDATE(v)		(((v & 1) << 4) & CARV_Q_MB_CR_UPDATE_MASK)
#define GET_CARV_Q_MB_CR_UPDATE(v)	(((v) & CARV_Q_MB_CR_UPDATE_MASK) >> 4)
#define CARV_Q_MB_CR_FAULT_MASK		BIT(3)
#define CARV_Q_MB_CR_FAULT(v)		(((v & 1) << 3) & CARV_Q_MB_CR_FAULT_MASK)
#define GET_CARV_Q_MB_CR_FAULT(v)	(((v) & CARV_Q_MB_CR_FAULT_MASK) >> 3)
#define CARV_Q_MB_CR_ERR_MASK		BIT(2)
#define CARV_Q_MB_CR_ERR(v)		(((v & 1) << 2) & CARV_Q_MB_CR_ERR_MASK)
#define GET_CARV_Q_MB_CR_ERR(v)		(((v) & CARV_Q_MB_CR_ERR_MASK) >> 2)
#define CARV_Q_MB_CR_DONE_MASK		BIT(1)
#define CARV_Q_MB_CR_DONE(v)		(((v & 1) << 1) & CARV_Q_MB_CR_DONE_MASK)
#define GET_CARV_Q_MB_CR_DONE(v)	(((v) & CARV_Q_MB_CR_DONE_MASK) >> 1)
#define CARV_Q_MB_CR_ENA_MASK		BIT(0)
#define CARV_Q_MB_CR_ENA(v)		((v) & CARV_Q_MB_CR_ENA_MASK)
#define GET_CARV_Q_MB_CR_ENA(v)		((v) & CARV_Q_MB_CR_ENA_MASK)

/* Quartile Address Translation */
#define CARV_AT_SIMPLE_REGIONS	8
#define CARV_AT_PRIO_REGIONS	4
#define CARV_Q_AT_BARL(u, n)	(CARV_Q_BASE(u) + 0x50 + ((n & 3) * 0x8))	/* n: 0..3 */
#define CARV_Q_AT_BARH(u, n)	(CARV_Q_BASE(u) + 0x54 + ((n & 3) * 0x8))	/* n: 0..3 */
#define CARV_Q_AT_SRR(u, n)	(CARV_Q_BASE(u) + 0x110 + ((n & 7) * 0x4))	/* n: 0..7 */
#define CARV_Q_AT_RCR(u, n)	(CARV_Q_BASE(u) + 0x150 + ((n & 3) * 0x10))	/* n: 0..3 */
#define CARV_Q_AT_RMR(u, n)	(CARV_Q_BASE(u) + 0x154 + ((n & 3) * 0x10))	/* n: 0..3 */

#define CARV_Q_AT_BARL_MEMTYPE_NCOH	(0x0)
#define CARV_Q_AT_BARL_MEMTYPE_COH	(0x1)
#define CARV_Q_AT_BARL_MEMTYPE_SHAR	(0x2)
#define CARV_Q_AT_BARL_ADDR_MASK	GENMASK_ULL(31, 14)
#define CARV_Q_AT_BARL_ADDR(v)		((v) & CARV_Q_AT_BARL_ADDR_MASK)
#define GET_CARV_Q_AT_BARL_ADDR(v)	((v) & CARV_Q_AT_BARL_ADDR_MASK)
#define CARV_Q_AT_BARH_ADDR_MASK	GENMASK_ULL(18, 0)
#define CARV_Q_AT_BARH_ADDR(v)		(((v) >> 32) & CARV_Q_AT_BARH_ADDR_MASK)
#define GET_CARV_Q_AT_BARH_ADDR(v)	(((v) & CARV_Q_AT_BARH_ADDR_MASK) << 32)
#define GET_CARV_Q_AT_BAR_ADDR(h, l)	(GET_CARV_Q_AT_BARH_ADDR(h) | \
					 GET_CARV_Q_AT_BARL_ADDR(l))

#define CARV_Q_AT_SRR_OFF_MASK		GENMASK(31, 14)
#define CARV_Q_AT_SRR_OFF(v)		(((v & GENMASK(17, 0)) << 14) & CARV_Q_AT_SRR_OFF_MASK)
#define GET_CARV_Q_AT_SRR_OFF(v)	(((v) & CARV_Q_AT_SRR_OFF_MASK) >> 14)
#define CARV_Q_AT_SRR_MEM_MASK		GENMASK(10, 9)
#define CARV_Q_AT_SRR_MEM(v)		(((v & 3) << 9) & CARV_Q_AT_SRR_MEM_MASK)
#define GET_CARV_Q_AT_SRR_MEM(v)	(((v) & CARV_Q_AT_SRR_MEM_MASK) >> 9)
#define CARV_Q_AT_SRR_LOCP_MASK		BIT(8)
#define CARV_Q_AT_SRR_LOCP(v)		(((v & 1) << 8) & CARV_Q_AT_SRR_LOCP_MASK)
#define GET_CARV_Q_AT_SRR_LOCP(v)	(((v) & CARV_Q_AT_SRR_LOCP_MASK) >> 8)
#define CARV_Q_AT_SRR_SIZE_MASK		GENMASK(7, 4)
#define CARV_Q_AT_SRR_SIZE(v)		(((v & 15) << 4) & CARV_Q_AT_SRR_SIZE_MASK)
#define GET_CARV_Q_AT_SRR_SIZE(v)	(((v) & CARV_Q_AT_SRR_SIZE_MASK) >> 4)
#define CARV_Q_AT_SRR_TYPE_MASK		GENMASK(3, 2)
#define CARV_Q_AT_SRR_TYPE(v)		(((v & 3) << 2) & CARV_Q_AT_SRR_TYPE_MASK)
#define GET_CARV_Q_AT_SRR_TYPE(v)	(((v) & CARV_Q_AT_SRR_TYPE_MASK) >> 2)
#define CARV_Q_AT_SRR_SUB_MASK		BIT(1)
#define CARV_Q_AT_SRR_SUB(v)		(((v & 1) << 1) & CARV_Q_AT_SRR_SUB_MASK)
#define GET_CARV_Q_AT_SRR_SUB(v)	(((v) & CARV_Q_AT_SRR_SUB_MASK) >> 1)
#define CARV_Q_AT_SRR_ENA_MASK		BIT(0)
#define CARV_Q_AT_SRR_ENA(v)		((v) & CARV_Q_AT_SRR_ENA_MASK)
#define GET_CARV_Q_AT_SRR_ENA(v)	((v) & CARV_Q_AT_SRR_ENA_MASK)

#define CARV_Q_AT_RCR_OFF_MASK		GENMASK(31, 12)
#define CARV_Q_AT_RCR_OFF(v)		(((v & GENMASK(19, 0)) << 12) & CARV_Q_AT_RCR_OFF_MASK)
#define GET_CARV_Q_AT_RCR_OFF(v)	(((v) & CARV_Q_AT_RCR_OFF_MASK) >> 12)
#define CARV_Q_AT_RCR_SIZE_MASK		GENMASK(11, 8)
#define CARV_Q_AT_RCR_SIZE(v)		(((v & 15) << 8) & CARV_Q_AT_RCR_SIZE_MASK)
#define GET_CARV_Q_AT_RCR_SIZE(v)	(((v) & CARV_Q_AT_RCR_SIZE_MASK) >> 8)
#define CARV_Q_AT_RCR_MEM_MASK		GENMASK(2, 1)
#define CARV_Q_AT_RCR_MEM(v)		(((v & 3) << 1) & CARV_Q_AT_RCR_MEM_MASK)
#define GET_CARV_Q_AT_RCR_MEM(v)	(((v) & CARV_Q_AT_RCR_MEM_MASK) >> 1)
#define CARV_Q_AT_RCR_ENA_MASK		BIT(0)
#define CARV_Q_AT_RCR_ENA(v)		((v) & CARV_Q_AT_RCR_ENA_MASK)
#define GET_CARV_Q_AT_RCR_ENA(v)	((v) & CARV_Q_AT_RCR_ENA_MASK)

#define CARV_Q_AT_RMR_MATCH_MASK	GENMASK(31, 12)
#define CARV_Q_AT_RMR_MATCH(v)		(((v & GENMASK(19, 0)) << 12) & CARV_Q_AT_RMR_MATCH_MASK)
#define GET_CARV_Q_AT_RMR_MATCH(v)	(((v) & CARV_Q_AT_RMR_MATCH_MASK) >> 12)

#define AT_OFFS_MAX			0x3F
#define AT_OFFL_MAX			0x0B

/* Quartile System Interface */
#define CARV_Q_SI_PMCR(u, n)	(CARV_Q_BASE(u) + 0x190 + ((n & 3) * 0x10))   /* n: 0..3 */
#define CARV_Q_SI_PML(u, n)	(CARV_Q_BASE(u) + 0x194 + ((n & 3) * 0x10))   /* n: 0..3 */
#define CARV_Q_SI_PMH(u, n)	(CARV_Q_BASE(u) + 0x198 + ((n & 3) * 0x10))   /* n: 0..3 */

/* Interrupt sources */
#define	CARV_INT_F_BUS_BIT		BIT(19)	/* Parity error detected on internal CAR-V bus */
#define	CARV_INT_F_L2_BIT		BIT(18)	/* Uncorrectable L2 error */
#define	CARV_INT_F_L1_BIT		BIT(17)	/* Uncorrectable L1D and/or L1I error */
#define	CARV_INT_F_CORE_BIT		BIT(16)	/* CAR-V Core related exceptions of type:
						 *   - watchdog timeout detected
						 *   - DTCM parity error detected
						 *   - software vector missing
						 *   - illegal instruction
						 */
#define	CARV_INT_F_AHB_BIT		BIT(13)	/* AHB access response error */
#define	CARV_INT_F_PART_BIT		BIT(12)	/* illegal isolated partition configuration */
#define	CARV_INT_F_AXIRP_BIT		BIT(11)	/* Parity error on AXI RDATA attribute */
#define	CARV_INT_F_AXIWE_BIT		BIT(10)	/* Response error on AXI BRESP attribute */
#define	CARV_INT_F_AXIRE_BIT		BIT(9)	/* Read error on AXI RRESP attribute */
#define	CARV_INT_F_MAP_BIT		BIT(8)	/* No external memory address mapping available
						 * (no translation)
						 */
#define	CARV_INT_ACTIVATED_BIT		BIT(4)	/* MBOX activated (MB_CRn[ENA] set and then
						 * serviced)
						 */
#define	CARV_INT_EVFLDONE_BIT		BIT(3)	/* MBOX flush/evict completed (with barrier to
						 * assure BRESP for all Flush writes)
						 */
#define	CARV_INT_UPDATE_BIT		BIT(2)	/* MBOX update received from Root Core */
#define	CARV_INT_ERR_BIT		BIT(1)	/* MBOX error received from Root Core */
#define	CARV_INT_DONE_BIT		BIT(0)	/* MBOX done received from Root Core */

#define	GET_CARV_INT_F_BUS(v)		(!!((v) & CARV_INT_F_BUS_BIT))
#define	GET_CARV_INT_F_L2(v)		(!!((v) & CARV_INT_F_L2_BIT))
#define	GET_CARV_INT_F_L1(v)		(!!((v) & CARV_INT_F_L1_BIT))
#define	GET_CARV_INT_F_CORE(v)		(!!((v) & CARV_INT_F_CORE_BIT))
#define	GET_CARV_INT_F_AHB(v)		(!!((v) & CARV_INT_F_AHB_BIT))
#define	GET_CARV_INT_F_PART(v)		(!!((v) & CARV_INT_F_PART_BIT))
#define	GET_CARV_INT_F_AXIRP(v)		(!!((v) & CARV_INT_F_AXIRP_BIT))
#define	GET_CARV_INT_F_AXIWE(v)		(!!((v) & CARV_INT_F_AXIWE_BIT))
#define	GET_CARV_INT_F_AXIRE(v)		(!!((v) & CARV_INT_F_AXIRE_BIT))
#define	GET_CARV_INT_F_MAP(v)		(!!((v) & CARV_INT_F_MAP_BIT))
#define	GET_CARV_INT_ACTIVATED(v)	(!!((v) & CARV_INT_ACTIVATED_BIT))
#define	GET_CARV_INT_EVFLDONE(v)	(!!((v) & CARV_INT_EVFLDONE_BIT))
#define	GET_CARV_INT_UPDATE(v)		(!!((v) & CARV_INT_UPDATE_BIT))
#define	GET_CARV_INT_ERR(v)		(!!((v) & CARV_INT_ERR_BIT))
#define	GET_CARV_INT_DONE(v)		(!!((v) & CARV_INT_DONE_BIT))

#define FUNC_ENABLE_MASK (CARV_INT_ACTIVATED_BIT | CARV_INT_EVFLDONE_BIT | \
	CARV_INT_UPDATE_BIT | CARV_INT_ERR_BIT | CARV_INT_DONE_BIT)

#define FAULT_ENABLE_MASK (CARV_INT_F_BUS_BIT | CARV_INT_F_L2_BIT | CARV_INT_F_L1_BIT | \
	CARV_INT_F_CORE_BIT | CARV_INT_F_AHB_BIT | CARV_INT_F_PART_BIT | CARV_INT_F_AXIRP_BIT | \
	CARV_INT_F_AXIWE_BIT | CARV_INT_F_AXIRE_BIT | CARV_INT_F_MAP_BIT)


#define CARVNET_IRQ_CNT 2

/* Dump area of CAR-V registers */
static const struct q_regmap_tab_gl {
	const char *name;
	const u32 addr;
} q_regmap_tab_gl[] = {
	{ "GL_CR", CARV_GL_CR },
	{ "GL_AHBER", CARV_GL_AHBER },
	{ "GL_IMR", CARV_GL_IMR },
	{ "GL_FMR", CARV_GL_FMR },
	{ "GL_PCR", CARV_GL_PCR },
	{ "GL_UCR", CARV_GL_UCR },
	{ "GL_UPR", CARV_GL_UPR },
	{ "GL_AHBCR", CARV_GL_AHBCR },
	{ "GL_USR", CARV_GL_USR },
	{ "GL_INFO", CARV_GL_INFO },
	{ 0 },
};

static const struct q_regmap_tab_u {
	const char *name;
	const u32 addr;
} q_regmap_tab_u[] = {
	{ "MB_CR", CARV_Q_MB_CR(0) },
	{ "MB_IR", CARV_Q_MB_IR(0) },
	{ "MB_IER", CARV_Q_MB_IER(0) },
	{ "MB_FER", CARV_Q_MB_FER(0) },
	{ "MB_INFO_CR0", CARV_Q_MB_INFO_CR(0, 0) },
	{ "MB_INFO_CR1", CARV_Q_MB_INFO_CR(0, 1) },
	{ "MB_INFO_CR2", CARV_Q_MB_INFO_CR(0, 2) },
	{ "MB_INFO_CR3", CARV_Q_MB_INFO_CR(0, 3) },
	{ "MB_MIR", CARV_Q_MB_MIR(0) },
	{ "MB_IFMSK", CARV_Q_MBOX_IFMSK(0) },
	{ "AT_BARL0", CARV_Q_AT_BARL(0, 0) },
	{ "AT_BARL1", CARV_Q_AT_BARL(0, 1) },
	{ "AT_BARL2", CARV_Q_AT_BARL(0, 2) },
	{ "AT_BARL3", CARV_Q_AT_BARL(0, 3) },
	{ "AT_BARH0", CARV_Q_AT_BARH(0, 0) },
	{ "AT_BARH1", CARV_Q_AT_BARH(0, 1) },
	{ "AT_BARH2", CARV_Q_AT_BARH(0, 2) },
	{ "AT_BARH3", CARV_Q_AT_BARH(0, 3) },
	{ "AT_SRR0", CARV_Q_AT_SRR(0, 0) },
	{ "AT_SRR1", CARV_Q_AT_SRR(0, 1) },
	{ "AT_SRR2", CARV_Q_AT_SRR(0, 2) },
	{ "AT_SRR3", CARV_Q_AT_SRR(0, 3) },
	{ "AT_SRR4", CARV_Q_AT_SRR(0, 4) },
	{ "AT_SRR5", CARV_Q_AT_SRR(0, 5) },
	{ "AT_SRR6", CARV_Q_AT_SRR(0, 6) },
	{ "AT_SRR7", CARV_Q_AT_SRR(0, 7) },
	{ "AT_RCR0", CARV_Q_AT_RCR(0, 0) },
	{ "AT_RCR1", CARV_Q_AT_RCR(0, 1) },
	{ "AT_RCR2", CARV_Q_AT_RCR(0, 2) },
	{ "AT_RCR3", CARV_Q_AT_RCR(0, 3) },
	{ "AT_RMR0", CARV_Q_AT_RMR(0, 0) },
	{ "AT_RMR1", CARV_Q_AT_RMR(0, 1) },
	{ "AT_RMR2", CARV_Q_AT_RMR(0, 2) },
	{ "AT_RMR3", CARV_Q_AT_RMR(0, 3) },
	{ "SI_PMCR0", CARV_Q_SI_PMCR(0, 0) },
	{ "SI_PMCR1", CARV_Q_SI_PMCR(0, 1) },
	{ "SI_PMCR2", CARV_Q_SI_PMCR(0, 2) },
	{ "SI_PMCR3", CARV_Q_SI_PMCR(0, 3) },
	{ "SI_PML0", CARV_Q_SI_PML(0, 0) },
	{ "SI_PML1", CARV_Q_SI_PML(0, 1) },
	{ "SI_PML2", CARV_Q_SI_PML(0, 2) },
	{ "SI_PML3", CARV_Q_SI_PML(0, 3) },
	{ "SI_PMH0", CARV_Q_SI_PMH(0, 0) },
	{ "SI_PMH1", CARV_Q_SI_PMH(0, 1) },
	{ "SI_PMH2", CARV_Q_SI_PMH(0, 2) },
	{ "SI_PMH3", CARV_Q_SI_PMH(0, 3) },
	{ 0 },
};

#define MBOX_PING_REQUEST	(250U)
#define MBOX_PING_REPLY		(251U)

enum {
	MEMRES_VDEV_ID,
	MEMRES_FW_ID,
	MEMRES_MAX
};

struct memres {
	void *va;		/* virtual address */
	dma_addr_t dma;		/* dma address */
	size_t len;		/* length in bytes */
	u32 free_idx;		/* pointer to free area inside the region */
	u32 of_resm_idx;	/* reserved memory phandle index */
	char name[32];		/* associated memory region name (optional) */
};

struct carvnet {
	struct device *dev;
	/*
	 * 0 (MEMRES_VDEV_ID): continuous memory for vdev
	 * 1 (MEMRES_FW_ID)  : continuous memory for firmware
	 */
	struct memres mrs[MEMRES_MAX];
	struct dentry *subdentry;
	void __iomem *regs;
	struct work_struct rproc_work;
	u8 q_user;		/* active Q user */
};

static int q_validate(struct carvnet *priv)
{
	struct device *dev = priv->dev;

	if (!GET_CARV_GL_CR_ENA(readl(priv->regs + CARV_GL_CR))) {
		dev_err(dev, "Quartile is disabled\n");
		return -EINVAL;
	}

	if (!GET_CARV_GL_CR_CLOCK(readl(priv->regs + CARV_GL_CR))) {
		dev_err(dev, "Quartile has MCT clock disabled\n");
		return -EINVAL;
	}

	dev_info(dev, "CAR-V revision %lx\n",
		 GET_CARV_GL_CR_IP_VID(readl(priv->regs + CARV_GL_CR)));

	if (!GET_CARV_GL_CR_LOCK(readl(priv->regs + CARV_GL_CR)))
		dev_warn(dev,
			 "Quartile has global config unlocked! It's dangerous.\n");

	return 0;
}

static int q_get_bars(struct carvnet *priv)
{
	return 2;
}

static int find_closest_bar(struct carvnet *priv, struct rproc_mem_entry *m,
			    u32 *offs)
{
	int i, res = -1;
	s64 m_offs = S64_MAX;

	/* Parse first n BARL/BARH register sets and find one
	 * with the smallest offset.
	 * There should be at least 2 BAR register sets
	 * with valid values (resmem 'fw' and 'vdev').
	 */
	for (i = 0; i < q_get_bars(priv); i++) {
		u32 barh, barl;
		u64 baddr;

		barh = readl(priv->regs + CARV_Q_AT_BARH(priv->q_user, i));
		barl = readl(priv->regs + CARV_Q_AT_BARL(priv->q_user, i));
		baddr = GET_CARV_Q_AT_BAR_ADDR(barh, barl);

		if ((m->dma - baddr) < m_offs) {
			res = i;
			m_offs = m->dma - baddr;
		}
	}

	if (res >= 0) {
		if (m_offs >= (s64)SZ_4G) {
			dev_err(priv->dev,
				"Mapping %d (%s) offset is bigger than SZ_4G: 0x%llx",
				i, m->name, m_offs);
			return -EINVAL;
		}
		*offs = (u32)m_offs;
	}

	return res;
}

static int q_set_simple_region(struct carvnet *priv, struct rproc_mem_entry *m,
			       int i)
{
	u32 r_size, r_off, r_mem, r_locp, r_type, r_sub, r_value;
	ulong m_size = roundup_pow_of_two(m->len);
	int bar;

	/*
	 * AT_SRR Register RM says:
	 * ---
	 * Note: The 8 AT_SRRn registers will have their reset values configured
	 * to support directly out of reset the internal memory type regions as
	 * defined in CAR-V Internal Memory Type and Access table in section
	 * Memory Types and Access Regions.
	 *
	 * Thus, software initialization of the AT_SRRn registers may not be
	 * required.
	 * ---
	 */

	bar = find_closest_bar(priv, m, &r_off);
	if (bar < 0)
		return -EINVAL;

	if (m_size < SZ_16K) {
		m_size = SZ_16K;
		r_size = 0;
	} else {
		r_size = fls(m_size / SZ_16K) - 1;
	}

	if (!IS_ALIGNED(r_off, SZ_16K)) {
		dev_err(priv->dev,
			"Mapping %d offset is not aligned to SZ_16K: 0x%x",
			i, r_off);
		return -EINVAL;
	}
	r_off = r_off / SZ_16K;
	r_mem = bar;
	if (m->da == CARV_MEMTYPE_LOCAL_PRIVATE_DTCM) {
		r_locp = 1;
		r_type = 0;
	} else {
		r_locp = 0;
		r_type = CARV_ADDR_GET_MEMTYPE(m->da);
	}
	r_sub = CARV_ADDR_GET_MEMSUBTYPE(m->da);

	r_value = CARV_Q_AT_SRR_OFF(r_off) |
		CARV_Q_AT_SRR_MEM(r_mem) |
		CARV_Q_AT_SRR_LOCP(r_locp) |
		CARV_Q_AT_SRR_SIZE(r_size) |
		CARV_Q_AT_SRR_TYPE(r_type) |
		CARV_Q_AT_SRR_SUB(r_sub) |
		CARV_Q_AT_SRR_ENA(1U);
	writel(r_value, priv->regs + CARV_Q_AT_SRR(priv->q_user, i));

	dev_dbg(priv->dev,
		"SRR=0x%x OFF=0x%x MEM=0x%x LOCP=%d SIZE=0x%x TYPE=0x%x SUB=%d ENA=1\n",
		readl(priv->regs + CARV_Q_AT_SRR(priv->q_user, i)), r_off,
		r_mem, r_locp, r_size, r_type, r_sub);

	return 0;
};

static int q_set_prio_region(struct carvnet *priv, struct rproc_mem_entry *m,
			     int i)
{
	u32 r_size, r_off, r_mem, r_match, r_value;
	ulong m_size = roundup_pow_of_two(m->len);
	int bar;

	bar = find_closest_bar(priv, m, &r_off);
	if (bar < 0)
		return -EINVAL;

	if (m_size < SZ_4K) {
		m_size = SZ_4K;
		r_size = 0;
	} else {
		r_size = fls(m_size / SZ_4K) - 1;
	}

	if (!IS_ALIGNED(r_off, SZ_4K)) {
		dev_err(priv->dev,
			"Mapping %d offset is not aligned to SZ_4K: 0x%x",
			i, r_off);
		return -EINVAL;
	}

	r_off = r_off / SZ_4K;
	r_mem = bar;

	r_value = CARV_Q_AT_RCR_OFF(r_off) |
		CARV_Q_AT_RCR_SIZE(r_size) |
		CARV_Q_AT_RCR_MEM(r_mem) |
		CARV_Q_AT_RCR_ENA(1U);
	writel(r_value, priv->regs + CARV_Q_AT_RCR(priv->q_user, i));

	r_match = GET_CARV_Q_AT_RMR_MATCH(m->da);

	writel(CARV_Q_AT_RMR_MATCH(r_match),
	       priv->regs + CARV_Q_AT_RMR(priv->q_user, i));

	dev_dbg(priv->dev,
		"RCR=0x%x OFF=0x%x SIZE=0x%x MEM=0x%x ENA=1\n",
		readl(priv->regs + CARV_Q_AT_RCR(priv->q_user, i)), r_off,
		r_size, r_mem);
	dev_dbg(priv->dev,
		"RMR=0x%x MATCH=0x%x\n",
		readl(priv->regs + CARV_Q_AT_RMR(priv->q_user, i)), r_match);

	return 0;
}

static int q_set_mapper(struct carvnet *priv)
{
	struct rproc *rproc;
	struct rproc_mem_entry *m = NULL;
	int simple_n = 0, prio_n = 0, datareg = 0, ret;

	rproc = dev_get_drvdata(priv->dev);

	/* Set BAR0 for firmware */
	writel(CARV_Q_AT_BARL_ADDR(priv->mrs[MEMRES_FW_ID].dma) |
	       CARV_Q_AT_BARL_MEMTYPE_NCOH,
	       priv->regs + CARV_Q_AT_BARL(priv->q_user, CARVNET_BAR_FW));
	writel(CARV_Q_AT_BARH_ADDR(priv->mrs[MEMRES_FW_ID].dma),
	       priv->regs + CARV_Q_AT_BARH(priv->q_user, CARVNET_BAR_FW));

	/* Set BAR1 for vdev */
	writel(CARV_Q_AT_BARL_ADDR(priv->mrs[MEMRES_VDEV_ID].dma) |
	       CARV_Q_AT_BARL_MEMTYPE_NCOH,
	       priv->regs + CARV_Q_AT_BARL(priv->q_user, CARVNET_BAR_VDEV));
	writel(CARV_Q_AT_BARH_ADDR(priv->mrs[MEMRES_VDEV_ID].dma),
	       priv->regs + CARV_Q_AT_BARH(priv->q_user, CARVNET_BAR_VDEV));

	/* Set memory regions */
	list_for_each_entry_reverse(m, &rproc->carveouts, node) {
		if (simple_n >= CARV_AT_SIMPLE_REGIONS ||
		    prio_n >= CARV_AT_PRIO_REGIONS) {
			dev_warn(priv->dev, "Too many mappings\n");
			break;
		}

		/*
		 * Prerequisite for prio region management:
		 * 1) first CARV_MEMTYPE_RW_DATA_FINE_GRAINED memory
		 *    mapping comes from FW, it maps FW RW data block as
		 *    simple region
		 * 2) additional CARV_MEMTYPE_RW_DATA_FINE_GRAINED memory
		 *    mappings are mapped as priority regions
		 */
		if ((m->da & CARV_MEMTYPE_MASK) != CARV_MEMTYPE_RW_DATA_FINE_GRAINED) {
			ret = q_set_simple_region(priv, m, simple_n++);
		} else if (!datareg) {
			ret = q_set_simple_region(priv, m, simple_n++);
			datareg = 1;
		} else {
			ret = q_set_prio_region(priv, m, prio_n++);
		}

		if (ret)
			return ret;
	}

	return 0;
}

static void q_clear_simple_region(struct carvnet *priv, int i)
{
	/* Clean AT_SRR and disable it */
	writel(0, priv->regs + CARV_Q_AT_SRR(priv->q_user, i));
}

static void q_clear_prio_region(struct carvnet *priv, int i)
{
	/* Clean AT_RCR/AT_RMRR and disable it */
	writel(0, priv->regs + CARV_Q_AT_RCR(priv->q_user, i));
	writel(0, priv->regs + CARV_Q_AT_RMR(priv->q_user, i));
}

static void q_clear_mapper(struct carvnet *priv)
{
	int i;

	/* Clear all simple region mappings */
	for (i = 0; i < CARV_AT_SIMPLE_REGIONS; i++)
		q_clear_simple_region(priv, i);

	/* Clear all prio region mappings */
	for (i = 0; i < CARV_AT_PRIO_REGIONS; i++)
		q_clear_prio_region(priv, i);

	/* Clear BAR0 */
	writel(0, priv->regs + CARV_Q_AT_BARL(priv->q_user, CARVNET_BAR_FW));
	writel(0, priv->regs + CARV_Q_AT_BARH(priv->q_user, CARVNET_BAR_FW));

	/* Clear BAR1 */
	writel(0, priv->regs + CARV_Q_AT_BARL(priv->q_user, CARVNET_BAR_VDEV));
	writel(0, priv->regs + CARV_Q_AT_BARH(priv->q_user, CARVNET_BAR_VDEV));
}

static int q_msg_send_info(struct carvnet *priv, u32 info)
{
	/* Send info by mailbox 0 */
	writel(info, priv->regs + CARV_Q_MB_INFO_CR(0, 0));

	return 0;
}

static void q_stop(struct carvnet *priv)
{
	/* Stop the FW */
	writel(0, priv->regs + CARV_Q_MB_CR(0));

	/* Disable Functional Interrupts: User configurable mask */
	writel(0, priv->regs + CARV_Q_MB_IER(0));
	/* Disable Fault Interrupts: User configurable mask */
	writel(0, priv->regs + CARV_Q_MB_FER(0));

	q_clear_mapper(priv);
}

static int q_start(struct carvnet *priv)
{
	int ret;

	/* Enable Functional Interrupts: User configurable mask */
	writel(FUNC_ENABLE_MASK, priv->regs + CARV_Q_MB_IER(0));
	/* Enable Fault Interrupts: User configurable mask */
	writel(FAULT_ENABLE_MASK, priv->regs + CARV_Q_MB_FER(0));

	/* Set Mapper */
	ret = q_set_mapper(priv);
	if (ret)
		goto err;

	/* Start the FW */
	writel(CARV_Q_MB_CR_ENA(1), priv->regs + CARV_Q_MB_CR(0));

	return 0;

err:
	q_stop(priv);

	return ret;
}

static int carvnet_rproc_parse_fw(struct rproc *rproc,
				  const struct firmware *fw)
{
	int ret;

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret)
		dev_warn(&rproc->dev, "No resource table in fw\n");

	return ret;
}

static int carvnet_rproc_start(struct rproc *rproc)
{
	struct carvnet *priv = rproc->priv;
	int ret;

	ret = q_start(priv);

	return ret;
}

static int carvnet_rproc_stop(struct rproc *rproc)
{
	struct carvnet *priv = rproc->priv;

	q_stop(priv);

	return 0;
}

static void carvnet_rproc_kick(struct rproc *rproc, int vqid)
{
	struct carvnet *priv = rproc->priv;

	q_msg_send_info(priv, (u32)vqid);
}

/* Expose NoCore registers via debugfs */
static int carvnet_dump_regs_show(struct seq_file *seq, void *unused)
{
	struct carvnet *priv = seq->private;
	int i;

	/* Global regs (GL_*) */
	for (i = 0; q_regmap_tab_gl[i].name; i++)
		seq_printf(seq, "%-14s: 0x%08x\n", q_regmap_tab_gl[i].name,
			   readl(priv->regs + q_regmap_tab_gl[i].addr));

	/* User 0 set */
	for (i = 0; q_regmap_tab_u[i].name; i++)
		seq_printf(seq, "%-14s: 0x%08x\n", q_regmap_tab_u[i].name,
			   readl(priv->regs + q_regmap_tab_u[i].addr +
			   CARV_Q_USER_OFFSET(0)));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(carvnet_dump_regs);

/* Expose carveout mem content via debugfs */
static int rproc_carveout_hexdump_show(struct seq_file *seq, void *unused)
{
	struct rproc_mem_entry *mem = seq->private;
	u32 offs;

	for (offs = 0; offs < mem->len; offs += CARVNET_FW_ALIGN)
		seq_hex_dump(seq, "", DUMP_PREFIX_OFFSET, 8, 1,
			     mem->va + offs, CARVNET_FW_ALIGN, true);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rproc_carveout_hexdump);

static int carvnet_rproc_elf_load(struct rproc *rproc,
				  const struct firmware *fw)
{
	struct carvnet *priv = rproc->priv;
	struct device *dev = &rproc->dev;
	struct memres *memres_fw = &priv->mrs[MEMRES_FW_ID];
	const u8 *elf_data = fw->data;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	struct rproc_mem_entry *mem;
	int i, ret = 0;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	dev_dbg(dev, " fw phnum=%d\n", ehdr->e_phnum);

	/* Remove possible existing debugfs carveouts subdir */
	debugfs_remove(priv->subdentry);
	priv->subdentry = NULL;

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 segsize = ALIGN(memsz, CARVNET_FW_ALIGN);
		u32 offset = phdr->p_offset;
		u32 flags = phdr->p_flags;

		dev_dbg(dev, "phdr[%i]: type %d da 0x%x offset 0x%x flags 0x%x memsz 0x%x filesz 0x%x\n",
			i, phdr->p_type, da, offset, flags, memsz, filesz);

		if (phdr->p_type != PT_LOAD)
			continue;

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* Create memory carveout for every fw segment.
		 * NOTE: We don't need to alloc/free such carveouts,
		 * they are part of reserved region 'fw'.
		 */
		mem = rproc_mem_entry_init(dev,
					   memres_fw->va + memres_fw->free_idx,
					   memres_fw->dma + memres_fw->free_idx,
					   segsize, da, NULL, NULL, "fwseg%d",
					   i);
		if (!mem)
			return -ENOMEM;

		if (rproc_coredump_add_segment(rproc, da, segsize))
			dev_warn(dev,
				 "Failed to add memory region '%s' to the coredump\n",
				 mem->name);

		rproc_add_carveout(rproc, mem);

		/* Move free pointer to the next aligned boundary */
		memres_fw->free_idx += segsize;

		/* Save segment in the reserved carveout memory region */
		memcpy(mem->va, elf_data + offset, filesz);

		/* Zero unused memory */
		if ((segsize - filesz) > 0)
			memset(mem->va + filesz, 0, segsize - filesz);
	}

	/* Add debugfs dump regions */
	priv->subdentry = debugfs_create_dir("carveouts", rproc->dbg_dir);
	list_for_each_entry(mem, &rproc->carveouts, node)
		debugfs_create_file(mem->name, 0400, priv->subdentry, mem,
				    &rproc_carveout_hexdump_fops);

	return ret;
}

static const struct rproc_ops carvnet_rproc_ops = {
	.start		= carvnet_rproc_start,
	.stop		= carvnet_rproc_stop,
	.kick		= carvnet_rproc_kick,
	.parse_fw	= carvnet_rproc_parse_fw,
	.load		= carvnet_rproc_elf_load,
	.find_loaded_rsc_table	= rproc_elf_find_loaded_rsc_table,
};

static int carvnet_rproc_parse_resmem(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct carvnet *priv = rproc->priv;
	struct of_phandle_iterator it;
	struct reserved_mem *rmem;
	int idx = 0, ret;

	/* Parse associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {

		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			of_node_put(it.node);
			dev_err(dev,
				"unable to acquire memory region\n");
			return -ENOMEM;
		}

		/* Verify the correct memres name */
		if (idx == MEMRES_VDEV_ID) {
			/* Required resmem 0: vdev */
			if (strcmp(it.node->name, "vdev")) {
				dev_err(dev,
					"Invalid memory region name for vdev: '%s'\n",
					it.node->name);
				return -EINVAL;
			}

			/* Associate vdev reserved memory region
			 * with device
			 */
			ret = of_reserved_mem_device_init_by_idx(dev, np, idx);
			if (ret) {
				dev_err(dev, "Failed to associate vdev resmem: %d\n",
					ret);
				return ret;
			}
		} else if (idx == MEMRES_FW_ID) {
			/* Required resmem 1: firmware */
			if (strcmp(it.node->name, "fw")) {
				dev_err(dev,
					"Invalid memory region name for firmware: '%s'\n",
					it.node->name);
				return -EINVAL;
			}
		} else {
			/* MEMRES_CUSTOM is optional, skipped for now */
			continue;
		}

		priv->mrs[idx].dma = (dma_addr_t)rmem->base;
		priv->mrs[idx].len = rmem->size;
		priv->mrs[idx].of_resm_idx = idx;
		strscpy(priv->mrs[idx].name, it.node->name,
			strlen(it.node->name));
		priv->mrs[idx].va = devm_ioremap_wc(dev, rmem->base,
						    rmem->size);
		if (!priv->mrs[idx].va) {
			dev_err(dev,
				"Unable to map memory region: '%s' %pa+0x%llx\n",
				priv->mrs[idx].name, &rmem->base, rmem->size);
			return -ENOMEM;
		}

		dev_dbg(dev, "Parsed resmem #%d for memory region '%s'\n", idx,
			it.node->name);

		idx++;
	}

	if (idx < 2)
		dev_warn(dev, "Not all required regions were declared: %d\n",
			 idx);

	return  0;
}

static void carvnet_rproc_fault_irq_msg(struct device *dev, u32 irq_mask)
{
	if (irq_mask & CARV_INT_F_BUS_BIT)
		dev_err(dev, "Fault: Parity error detected on internal CAR-V bus\n");

	if (irq_mask & CARV_INT_F_L2_BIT)
		dev_err(dev, "Fault: Uncorrectable L2 cache error\n");

	if (irq_mask & CARV_INT_F_L1_BIT)
		dev_err(dev, "Fault: Uncorrectable L1D and/or L1I error\n");

	if (irq_mask & CARV_INT_F_CORE_BIT)
		dev_err(dev, "Fault: CAR-V Core related exceptions of type\n");

	if (irq_mask & CARV_INT_F_AHB_BIT)
		dev_err(dev, "Fault: AHB access response error\n");

	if (irq_mask & CARV_INT_F_PART_BIT)
		dev_err(dev, "Fault: Illegal isolated partition configuration\n");

	if (irq_mask & CARV_INT_F_AXIRP_BIT)
		dev_err(dev, "Fault: Parity error on AXI RDATA attribute\n");

	if (irq_mask & CARV_INT_F_AXIWE_BIT)
		dev_err(dev, "Fault: Response error on AXI BRESP attribute\n");

	if (irq_mask & CARV_INT_F_AXIRE_BIT)
		dev_err(dev, "Fault: Read error on AXI RRESP attribute\n");

	if (irq_mask & CARV_INT_F_MAP_BIT)
		dev_err(dev, "Fault: No external memory address mapping available\n");
}

static void carvnet_rproc_vq_work(struct work_struct *work)
{
	struct carvnet *priv = container_of(work, struct carvnet,
					    rproc_work);
	struct rproc *rproc = dev_get_drvdata(priv->dev);

	guard(mutex)(&rproc->lock);

	if (rproc->state != RPROC_RUNNING)
		return;

	rproc_vq_interrupt(rproc, 0);
	rproc_vq_interrupt(rproc, 1);
}

static irqreturn_t carvnet_rproc_handle_irq(int irq_num, void *d)
{
	struct carvnet *priv = (struct carvnet *)d;
	u32 irq_mask, ier_mask, fer_mask;

	irq_mask = readl(priv->regs + CARV_Q_MB_IR(priv->q_user));
	ier_mask = readl(priv->regs + CARV_Q_MB_IER(priv->q_user));
	fer_mask = readl(priv->regs + CARV_Q_MB_FER(priv->q_user));

	/* Write back the interrupt bits to clear the interrupt in the CARV hardware */
	irq_mask &= (ier_mask | fer_mask);
	writel(irq_mask, priv->regs + CARV_Q_MB_IR(priv->q_user));

	if (GET_CARV_INT_UPDATE(irq_mask))
		schedule_work(&priv->rproc_work);

	if (GET_CARV_INT_ACTIVATED(irq_mask))
		dev_info(priv->dev, "Activated\n");

	if (FAULT_ENABLE_MASK & irq_mask)
		carvnet_rproc_fault_irq_msg(priv->dev, irq_mask);

	return IRQ_HANDLED;
}

static int carvnet_rproc_probe(struct platform_device *pdev)
{
	static const char *irq_name[CARVNET_IRQ_CNT] = { "func", "fault" };
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const char *fw_name = NULL;
	struct carvnet *priv;
	struct rproc *rproc;
	int irq_index;
	int irq;
	int ret;

	ret = rproc_of_parse_firmware(dev, 0, &fw_name);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to parse firmware-name property\n");
	if (!fw_name)
		return dev_err_probe(dev, -EINVAL, "FW name is missing\n");

	rproc = devm_rproc_alloc(dev, np->name, &carvnet_rproc_ops, fw_name,
				 sizeof(struct carvnet));
	if (!rproc)
		return -ENOMEM;

	priv = rproc->priv;
	priv->dev = dev;

	ret = carvnet_rproc_parse_resmem(rproc);
	if (ret)
		return ret;

	dev_set_drvdata(dev, rproc);

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (WARN_ON(IS_ERR(priv->regs)))
		return PTR_ERR(priv->regs);

	rproc->auto_boot = auto_boot;

	/* We are using only first user config */
	priv->q_user = 0;

	ret = q_validate(priv);
	if (ret)
		return ret;

	INIT_WORK(&priv->rproc_work, carvnet_rproc_vq_work);

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return dev_err_probe(dev, ret, "rproc_add failed\n");

	debugfs_create_file("dump_regs", 0400, rproc->dbg_dir, priv,
			    &carvnet_dump_regs_fops);

	for (irq_index = 0; irq_index < CARVNET_IRQ_CNT; irq_index++) {
		irq = platform_get_irq_byname(pdev, irq_name[irq_index]);
		ret = devm_request_irq(dev, irq, carvnet_rproc_handle_irq, 0,
				pdev->name, priv);
		if (ret)
			return dev_err_probe(dev, ret,
					"unable to request IRQ%d (%s) for CARV device\n",
					irq, irq_name[irq_index]);
	}

	return ret;
}

static void carvnet_rproc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc *rproc = dev_get_drvdata(dev);
	struct carvnet *priv = rproc->priv;

	/* Remove possible existing debugfs carveouts subdir */
	debugfs_remove(priv->subdentry);
	priv->subdentry = NULL;
}

static const struct of_device_id carvnet_rproc_of_match[] = {
	{ .compatible = "nxp,carvnet-rproc" },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, carvnet_rproc_of_match);

static struct platform_driver carvnet_rproc_driver = {
	.probe = carvnet_rproc_probe,
	.remove_new = carvnet_rproc_remove,
	.driver = {
		.name = "nxp-carvnet",
		.of_match_table = carvnet_rproc_of_match,
	},
};

module_platform_driver(carvnet_rproc_driver);

MODULE_AUTHOR("Jan Petrous <jan.petrous@nxp.com>");
MODULE_DESCRIPTION("NXP S32N7 CAR-V-NET rproc driver");
MODULE_LICENSE("Dual BSD/GPL");
