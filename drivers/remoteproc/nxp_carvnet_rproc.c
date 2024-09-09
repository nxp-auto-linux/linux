// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2024 NXP
 *
 * CAR-V NET remoteproc driver
 *
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>

#include "remoteproc_internal.h"

static bool auto_boot;
module_param(auto_boot, bool, 0400);
MODULE_PARM_DESC(auto_boot,
		 "Auto-boot the CAR-V-NET remote processor [default=false]");

#define CARVNET_FW_ALIGN	0x4000

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

#define CARV_GL_CR_IP_VID_MASK	(0xFFFF0000)
#define GET_CARV_GL_CR_IP_VID(v)	(((v) & CARV_GL_CR_IP_VID_MASK) >> 16)
#define CARV_GL_CR_LOCK_MASK	(0x00000004)
#define CARV_GL_CR_LOCK(v)	(((v) << 2) & CARV_GL_CR_LOCK_MASK)
#define GET_CARV_GL_CR_LOCK(v)	(((v) & CARV_GL_CR_LOCK_MASK) >> 2)
#define CARV_GL_CR_CLOCK_MASK	(0x00000002)
#define CARV_GL_CR_CLOCK(v)	(((v) << 1) & CARV_GL_CR_CLOCK_MASK)
#define GET_CARV_GL_CR_CLOCK(v)	(((v) & CARV_GL_CR_CLOCK_MASK) >> 1)
#define CARV_GL_CR_ENA_MASK	(0x00000001)
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
#define CARV_Q_USER_OFFSET(u)	((u) * 0x10000)				/* u: 0..3 */
#define CARV_Q_BASE(u)		(0x10000 + CARV_Q_USER_OFFSET(u))

/* Quartile Mailbox */
#define CARV_Q_MB_CR(u)		(CARV_Q_BASE(u) + 0x0)
#define CARV_Q_MB_IR(u)		(CARV_Q_BASE(u) + 0x4)
#define CARV_Q_MB_IER(u)	(CARV_Q_BASE(u) + 0x8)
#define CARV_Q_MB_FER(u)	(CARV_Q_BASE(u) + 0xC)
#define CARV_Q_MB_INFO_CR(u, n)	(CARV_Q_BASE(u) + 0x10 + ((n) * 0x4))	/* n: 0..2 */
#define CARV_Q_MB_MIR(u)	(CARV_Q_BASE(u) + 0x20)
#define CARV_Q_MBOX_IFMSK(u)	(CARV_Q_BASE(u) + 0x40)

#define CARV_Q_MB_CR_INFO_MASK	(0xFFFF0000)
#define CARV_Q_MB_CR_INFO(v)	(((v) << 16) & CARV_Q_MB_CR_INFO_MASK)
#define GET_CARV_Q_MB_CR_INFO(v)	(((v) & CARV_Q_MB_CR_INFO_MASK) >> 16)
#define CARV_Q_MB_CR_ERRORS_MASK	(0x0000F000)
#define CARV_Q_MB_CR_ERRORS(v)	(((v) << 12) & CARV_Q_MB_CR_ERRORS_MASK)
#define GET_CARV_Q_MB_CR_ERRORS(v)	(((v) & CARV_Q_MB_CR_ERRORS_MASK) >> 12)
#define CARV_Q_MB_CR_RSTCHG_MASK	(0x00000200)
#define CARV_Q_MB_CR_RSTCHG(v)	(((v) << 9) & CARV_Q_MB_CR_RSTCHG_MASK)
#define GET_CARV_Q_MB_CR_RSTCHG(v)	(((v) & CARV_Q_MB_CR_RSTCHG_MASK) >> 9)
#define CARV_Q_MB_CR_RSTSTART_MASK	(0x00000100)
#define CARV_Q_MB_CR_RSTSTART(v)	(((v) << 8) & CARV_Q_MB_CR_RSTSTART_MASK)
#define GET_CARV_Q_MB_CR_RSTSTART(v)	(((v) & CARV_Q_MB_CR_RSTSTART_MASK) >> 8)
#define CARV_Q_MB_CR_UPDATE_MASK	(0x00000010)
#define CARV_Q_MB_CR_UPDATE(v)	(((v) << 4) & CARV_Q_MB_CR_UPDATE_MASK)
#define GET_CARV_Q_MB_CR_UPDATE(v)	(((v) & CARV_Q_MB_CR_UPDATE_MASK) >> 4)
#define CARV_Q_MB_CR_FAULT_MASK	(0x00000008)
#define CARV_Q_MB_CR_FAULT(v)	(((v) << 3) & CARV_Q_MB_CR_FAULT_MASK)
#define GET_CARV_Q_MB_CR_FAULT(v)	(((v) & CARV_Q_MB_CR_FAULT_MASK) >> 3)
#define CARV_Q_MB_CR_ERR_MASK	(0x00000004)
#define CARV_Q_MB_CR_ERR(v)	(((v) << 2) & CARV_Q_MB_CR_ERR_MASK)
#define GET_CARV_Q_MB_CR_ERR(v)	(((v) & CARV_Q_MB_CR_ERR_MASK) >> 2)
#define CARV_Q_MB_CR_DONE_MASK	(0x00000002)
#define CARV_Q_MB_CR_DONE(v)	(((v) << 1) & CARV_Q_MB_CR_DONE_MASK)
#define GET_CARV_Q_MB_CR_DONE(v)	(((v) & CARV_Q_MB_CR_DONE_MASK) >> 1)
#define CARV_Q_MB_CR_ENA_MASK	(0x00000001)
#define CARV_Q_MB_CR_ENA(v)	((v) & CARV_Q_MB_CR_ENA_MASK)
#define GET_CARV_Q_MB_CR_ENA(v)	((v) & CARV_Q_MB_CR_ENA_MASK)

/* Quartile Address Translation */
#define CARV_AT_SIMPLE_REGIONS	8
#define CARV_AT_PRIO_REGIONS	4
#define CARV_Q_AT_BARL(u, n)	(CARV_Q_BASE(u) + 0x50 + ((n) * 0x8))	/* n: 0..3 */
#define CARV_Q_AT_BARH(u, n)	(CARV_Q_BASE(u) + 0x54 + ((n) * 0x8))	/* n: 0..3 */
#define CARV_Q_AT_SRR(u, n)	(CARV_Q_BASE(u) + 0x110 + ((n) * 0x4))	/* n: 0..7 */
#define CARV_Q_AT_RCR(u, n)	(CARV_Q_BASE(u) + 0x150 + ((n) * 0x10))	/* n: 0..3 */
#define CARV_Q_AT_RMR(u, n)	(CARV_Q_BASE(u) + 0x154 + ((n) * 0x10))	/* n: 0..3 */

#define CARV_Q_AT_BARL_MEMTYPE_NCOH	(0x0)
#define CARV_Q_AT_BARL_MEMTYPE_COH	(0x1)
#define CARV_Q_AT_BARL_MEMTYPE_SHAR	(0x2)
#define CARV_Q_AT_BARL_ADDR_MASK	(0xFFFFC000ULL)
#define CARV_Q_AT_BARL_ADDR(v)		((v) & CARV_Q_AT_BARL_ADDR_MASK)
#define GET_CARV_Q_AT_BARL_ADDR(v)	((v) & CARV_Q_AT_BARL_ADDR_MASK)
#define CARV_Q_AT_BARH_ADDR_MASK	(0x0007FFFFULL)
#define CARV_Q_AT_BARH_ADDR(v)		(((v) >> 32) & CARV_Q_AT_BARH_ADDR_MASK)
#define GET_CARV_Q_AT_BARH_ADDR(v)	(((v) & CARV_Q_AT_BARH_ADDR_MASK) << 32)
#define GET_CARV_Q_AT_BAR_ADDR(h, l)	(GET_CARV_Q_AT_BARH_ADDR(h) |             \
					 GET_CARV_Q_AT_BARL_ADDR(l))

#define CARV_Q_AT_SRR_OFFS_MASK		(0x03F00000)
#define CARV_Q_AT_SRR_OFFS(v)		(((v) << 20) & CARV_Q_AT_SRR_OFFS_MASK)
#define GET_CARV_Q_AT_SRR_OFFS(v)	(((v) & CARV_Q_AT_SRR_OFFS_MASK) >> 20)
#define CARV_Q_AT_SRR_OFFL_MASK		(0x000F0000)
#define CARV_Q_AT_SRR_OFFL(v)		(((v) << 16) & CARV_Q_AT_SRR_OFFL_MASK)
#define GET_CARV_Q_AT_SRR_OFFL(v)	(((v) & CARV_Q_AT_SRR_OFFL_MASK) >> 16)
#define CARV_Q_AT_SRR_MEM_MASK		(0x00000600)
#define CARV_Q_AT_SRR_MEM(v)		(((v) << 9) & CARV_Q_AT_SRR_MEM_MASK)
#define GET_CARV_Q_AT_SRR_MEM(v)	(((v) & CARV_Q_AT_SRR_MEM_MASK) >> 9)
#define CARV_Q_AT_SRR_LOCP_MASK		(0x00000100)
#define CARV_Q_AT_SRR_LOCP(v)		(((v) << 8) & CARV_Q_AT_SRR_LOCP_MASK)
#define GET_CARV_Q_AT_SRR_LOCP(v)	(((v) & CARV_Q_AT_SRR_LOCP_MASK) >> 8)
#define CARV_Q_AT_SRR_SIZE_MASK		(0x000000F0)
#define CARV_Q_AT_SRR_SIZE(v)		(((v) << 4) & CARV_Q_AT_SRR_SIZE_MASK)
#define GET_CARV_Q_AT_SRR_SIZE(v)	(((v) & CARV_Q_AT_SRR_SIZE_MASK) >> 4)
#define CARV_Q_AT_SRR_TYPE_MASK		(0x000000C)
#define CARV_Q_AT_SRR_TYPE(v)		(((v) << 2) & CARV_Q_AT_SRR_TYPE_MASK)
#define GET_CARV_Q_AT_SRR_TYPE(v)	(((v) & CARV_Q_AT_SRR_TYPE_MASK) >> 2)
#define CARV_Q_AT_SRR_SUB_MASK		(0x00000002)
#define CARV_Q_AT_SRR_SUB(v)		(((v) << 1) & CARV_Q_AT_SRR_SUB_MASK)
#define GET_CARV_Q_AT_SRR_SUB(v)	(((v) & CARV_Q_AT_SRR_SUB_MASK) >> 1)
#define CARV_Q_AT_SRR_ENA_MASK		(0x00000001)
#define CARV_Q_AT_SRR_ENA(v)		((v) & CARV_Q_AT_SRR_ENA_MASK)
#define GET_CARV_Q_AT_SRR_ENA(v)	((v) & CARV_Q_AT_SRR_ENA_MASK)

#define CARV_Q_AT_RCR_OFFS_MASK		(0x03F00000)
#define CARV_Q_AT_RCR_OFFS(v)		(((v) << 20) & CARV_Q_AT_RCR_OFFS_MASK)
#define GET_CARV_Q_AT_RCR_OFFS(v)	(((v) & CARV_Q_AT_RCR_OFFS_MASK) >> 20)
#define CARV_Q_AT_RCR_OFFL_MASK		(0x000F0000)
#define CARV_Q_AT_RCR_OFFL(v)		(((v) << 16) & CARV_Q_AT_RCR_OFFL_MASK)
#define GET_CARV_Q_AT_RCR_OFFL(v)	(((v) & CARV_Q_AT_RCR_OFFL_MASK) >> 16)
#define CARV_Q_AT_RCR_SIZE_MASK		(0x00000F00)
#define CARV_Q_AT_RCR_SIZE(v)		(((v) << 8) & CARV_Q_AT_RCR_SIZE_MASK)
#define GET_CARV_Q_AT_RCR_SIZE(v)	(((v) & CARV_Q_AT_RCR_SIZE_MASK) >> 8)
#define CARV_Q_AT_RCR_MEM_MASK		(0x00000006)
#define CARV_Q_AT_RCR_MEM(v)		(((v) << 1) & CARV_Q_AT_RCR_MEM_MASK)
#define GET_CARV_Q_AT_RCR_MEM(v)	(((v) & CARV_Q_AT_RCR_MEM_MASK) >> 1)
#define CARV_Q_AT_RCR_ENA_MASK		(0x00000001)
#define CARV_Q_AT_RCR_ENA(v)		((v) & CARV_Q_AT_RCR_ENA_MASK)
#define GET_CARV_Q_AT_RCR_ENA(v)	((v) & CARV_Q_AT_RCR_ENA_MASK)

#define CARV_Q_AT_RMR_MATCH_MASK	(0xFFFFF000)
#define CARV_Q_AT_RMR_MATCH(v)		(((v) << 12) & CARV_Q_AT_RMR_MATCH_MASK)
#define GET_CARV_Q_AT_RMR_MATCH(v)	(((v) & CARV_Q_AT_RMR_MATCH_MASK) >> 12)

/* Quartile System Interface */
#define CARV_Q_SI_PMCR(u, n)	(CARV_Q_BASE(u) + 0x190 + ((n) * 0x10))   /* n: 0..3 */
#define CARV_Q_SI_PML(u, n)	(CARV_Q_BASE(u) + 0x194 + ((n) * 0x10))   /* n: 0..3 */
#define CARV_Q_SI_PMH(u, n)	(CARV_Q_BASE(u) + 0x198 + ((n) * 0x10))   /* n: 0..3 */

/* Interrupt sources */
enum {
	CARV_INT_F_BUS_BIT,	/* Parity error detected on internal CAR-V bus */
	CARV_INT_F_L2_BIT,	/* Uncorrectable L2 error */
	CARV_INT_F_L1_BIT,	/* Uncorrectable L1D and/or L1I error */
	CARV_INT_F_CORE_BIT,	/* CAR-V Core related exceptions of type:
				 *   - watchdog timeout detected
				 *   - DTCM parity error detected
				 *   - software vector missing
				 *   - illegal instruction
				 */
	CARV_INT_F_AHB_BIT,	/* AHB access response error */
	CARV_INT_F_PART_BIT,	/* illegal isolated partition configuration */
	CARV_INT_F_AXIRP_BIT,	/* Parity error on AXI RDATA attribute */
	CARV_INT_F_AXIWE_BIT,	/* Response error on AXI BRESP attribute */
	CARV_INT_F_AXIRE_BIT,	/* Read error on AXI RRESP attribute */
	CARV_INT_F_MAP_BIT,	/* No external memory address mapping available
				 * (no translation)
				 */
	CARV_INT_ACTIVATED_BIT,	/* MBOX activated (MB_CRn[ENA] set and then
				 * serviced)
				 */
	CARV_INT_EVFLDONE_BIT,	/* MBOX flush/evict completed (with barrier to
				 * assure BRESP for all Flush writes)
				 */
	CARV_INT_UPDATE_BIT,	/* MBOX update received from Root Core */
	CARV_INT_ERR_BIT,	/* MBOX error received from Root Core */
	CARV_INT_DONE_BIT	/* MBOX done received from Root Core */
};

#define	GET_CARV_INT_F_BUS(v)		(!!((v) & CARV_INT_F_BUS))
#define	GET_CARV_INT_F_L2(v)		(!!((v) & CARV_INT_F_L2))
#define	GET_CARV_INT_F_L1(v)		(!!((v) & CARV_INT_F_L1))
#define	GET_CARV_INT_F_CORE(v)		(!!((v) & CARV_INT_F_CORE))
#define	GET_CARV_INT_F_AHB(v)		(!!((v) & CARV_INT_F_AHB))
#define	GET_CARV_INT_F_PART(v)		(!!((v) & CARV_INT_F_PART))
#define	GET_CARV_INT_F_AXIRP(v)		(!!((v) & CARV_INT_F_AXIRP))
#define	GET_CARV_INT_F_AXIWE(v)		(!!((v) & CARV_INT_F_AXIWE))
#define	GET_CARV_INT_F_AXIRE(v)		(!!((v) & CARV_INT_F_AXIRE))
#define	GET_CARV_INT_F_MAP(v)		(!!((v) & CARV_INT_F_MAP))
#define	GET_CARV_INT_ACTIVATED(v)	(!!((v) & CARV_INT_ACTIVATED))
#define	GET_CARV_INT_EVFLDONE(v)	(!!((v) & CARV_INT_EVFLDONE))
#define	GET_CARV_INT_UPDATE(v)		(!!((v) & CARV_INT_UPDATE))
#define	GET_CARV_INT_ERR(v)		(!!((v) & CARV_INT_ERR))
#define	GET_CARV_INT_DONE(v)		(!!((v) & CARV_INT_DONE))

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
	/*
	 * 0 (MEMRES_VDEV_ID): continuous memory for vdev
	 * 1 (MEMRES_FW_ID)  : continuous memory for firmware
	 */
	struct memres mrs[MEMRES_MAX];
	struct dentry *subdentry;
	void __iomem *regs;
};

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
	return 0;
}

static int carvnet_rproc_stop(struct rproc *rproc)
{
	return 0;
}

static void carvnet_rproc_kick(struct rproc *rproc, int vqid)
{
	/* no op */
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

static int carvnet_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	struct carvnet *priv;
	const char *fw_name = NULL;
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

	ret = carvnet_rproc_parse_resmem(rproc);
	if (ret)
		return ret;

	dev_set_drvdata(dev, rproc);

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (WARN_ON(IS_ERR(priv->regs)))
		return PTR_ERR(priv->regs);

	rproc->auto_boot = auto_boot;

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return dev_err_probe(dev, ret, "rproc_add failed\n");

	debugfs_create_file("dump_regs", 0400, rproc->dbg_dir, priv,
			    &carvnet_dump_regs_fops);

	dev_info(dev, "CAR-V revision %x\n",
		 GET_CARV_GL_CR_IP_VID(readl(priv->regs + CARV_GL_CR)));

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
