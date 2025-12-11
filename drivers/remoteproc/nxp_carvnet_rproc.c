// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2024-2025 NXP
 *
 * CAR-V NET remoteproc driver
 *
 */

#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/limits.h>
#include <linux/minmax.h>
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

#define CMD_FW_STOP			0xFFFFFFFDU

#define RSC_VENDOR_CARV_MATCH_ADDR	256U

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

/* User set array (set by GL_UCR) */
#define CARV_Q_USER_OFFSET(u)	((u & 3) * 0x10000)			/* u: 0..3 */
#define CARV_Q_BASE(u)		(CARV_Q_USER_OFFSET(u))

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
#define CARV_Q_AT_BARL_MEMTYPE_DEFAULT	CARV_Q_AT_BARL_MEMTYPE_NCOH
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

#define CARV_Q_AT_RCR_OFF_MASK		GENMASK(31, 14)
#define CARV_Q_AT_RCR_OFF(v)		(((v & GENMASK(19, 0)) << 14) & CARV_Q_AT_RCR_OFF_MASK)
#define GET_CARV_Q_AT_RCR_OFF(v)	(((v) & CARV_Q_AT_RCR_OFF_MASK) >> 14)
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
#define	CARV_INT_F_AXIRP_BIT		BIT(11)	/* Parity error on AXI RDATA attribute */
#define	CARV_INT_F_AXIWE_BIT		BIT(10)	/* Response error on AXI BRESP attribute */
#define	CARV_INT_F_AXIRE_BIT		BIT(9)	/* Read error on AXI RRESP attribute */
#define	CARV_INT_F_MAP_BIT		BIT(8)	/* No external memory address mapping available
						 * (no translation)
						 */
#define	CARV_INT_ACTIVATED_BIT		BIT(4)	/* MBOX activated (MB_CRn[ENA] set and then
						 * serviced)
						 */
#define	CARV_INT_RESERVED_BIT3		BIT(3)	/* Bit3 reserved */
#define	CARV_INT_UPDATE_BIT		BIT(2)	/* MBOX update received from Root Core */
#define	CARV_INT_ERR_BIT		BIT(1)	/* MBOX error received from Root Core */
#define	CARV_INT_DONE_BIT		BIT(0)	/* MBOX done received from Root Core */

#define	GET_CARV_INT_F_BUS(v)		(!!((v) & CARV_INT_F_BUS_BIT))
#define	GET_CARV_INT_F_L2(v)		(!!((v) & CARV_INT_F_L2_BIT))
#define	GET_CARV_INT_F_L1(v)		(!!((v) & CARV_INT_F_L1_BIT))
#define	GET_CARV_INT_F_CORE(v)		(!!((v) & CARV_INT_F_CORE_BIT))
#define	GET_CARV_INT_F_AHB(v)		(!!((v) & CARV_INT_F_AHB_BIT))
#define	GET_CARV_INT_F_AXIRP(v)		(!!((v) & CARV_INT_F_AXIRP_BIT))
#define	GET_CARV_INT_F_AXIWE(v)		(!!((v) & CARV_INT_F_AXIWE_BIT))
#define	GET_CARV_INT_F_AXIRE(v)		(!!((v) & CARV_INT_F_AXIRE_BIT))
#define	GET_CARV_INT_F_MAP(v)		(!!((v) & CARV_INT_F_MAP_BIT))
#define	GET_CARV_INT_ACTIVATED(v)	(!!((v) & CARV_INT_ACTIVATED_BIT))
#define	GET_CARV_INT_EVFLDONE(v)	(!!((v) & CARV_INT_EVFLDONE_BIT))
#define	GET_CARV_INT_UPDATE(v)		(!!((v) & CARV_INT_UPDATE_BIT))
#define	GET_CARV_INT_ERR(v)		(!!((v) & CARV_INT_ERR_BIT))
#define	GET_CARV_INT_DONE(v)		(!!((v) & CARV_INT_DONE_BIT))

#define IRQ_ENABLE_MASK (CARV_INT_ACTIVATED_BIT | \
	CARV_INT_UPDATE_BIT | CARV_INT_ERR_BIT | CARV_INT_DONE_BIT | \
	CARV_INT_F_BUS_BIT | CARV_INT_F_L2_BIT | CARV_INT_F_L1_BIT | \
	CARV_INT_F_CORE_BIT | CARV_INT_F_AHB_BIT | CARV_INT_F_AXIRP_BIT | \
	CARV_INT_F_AXIWE_BIT | CARV_INT_F_AXIRE_BIT | CARV_INT_F_MAP_BIT)

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

#define CARVEOUT_NAME_SIZE 32U

struct __packed fw_rsc_vendor_carv_match_addr {
	u32 match_addr;
	u8 carveout_name[CARVEOUT_NAME_SIZE];
};

struct __packed carvnet_bar_info {
	u8  bar_type;
	u64 addr;
	u64 addr_end;
};

#define MAX_QUARTILES 2U
#define BAR_CNT 4U
#define BAR_INVALID 0xFFFFFFFFU

/* Carveouts from Firmware segments have a special dedicated name
 * to recognize them for mapping as simple-regions
 */
#define FIRMWARE_SEGMENT_NAME "fwseg"

/**
 * struct carveout_info - additional information for each carveout memory in rproc->carveouts
 * @va: virtual address of the carveout memory region
 * @carveout_name: human-readable name of the carveout resource
 * @match_addr: the CarV side address (match address) to be used for specified Carveout
 * @base_addr: physical base address of the carveout memory region
 * @size: size of the carveout memory region in bytes
 * @bar_index: index 0..3 of the BAR to use with this carveout, or BAR_INVALID
 *             if not assigned
 * @bar_type: type of BAR memory (coherent, non-coherent, or shared)
 * @node: kernel list node, to keep infos in struct carvnet's carveout_infos list
 *
 * This resource entry provides the host information about required CARV address for
 * each Carveout region, along with memory mapping details and BAR configuration.
 */
struct carveout_info {
	void *va;
	u8 carveout_name[CARVEOUT_NAME_SIZE];
	u32 match_addr;
	phys_addr_t base_addr;
	size_t size;
	u32 bar_index;
	u8 bar_type;
	struct list_head node;
};

struct carvnet {
	struct device *dev;
	struct carvnet_bar_info bar_info[BAR_CNT];
	u8 bar_info_cnt;
	struct dentry *subdentry;
	u32 num_quartiles;
	void __iomem *regs[MAX_QUARTILES];
	struct work_struct rproc_work;
	u8 q_user;		/* active Q user */

	struct list_head carveout_infos; /* Extra information for each carveout: bar, match_addr */
};

static struct carveout_info *q_get_carveout_info(struct carvnet *priv,
						 const char name[CARVEOUT_NAME_SIZE])
{
	struct carveout_info *info;

	/* Find carveout_info for this Carveout name */
	list_for_each_entry(info, &priv->carveout_infos, node)
		if (strncmp((const char *)info->carveout_name, name, CARVEOUT_NAME_SIZE) == 0)
			return info;

	/* Allocate carveout_info and store it into carvnet::carveout_infos list */
	info = kzalloc(sizeof(struct carveout_info), GFP_KERNEL);
	if (!info)
		return NULL;

	info->bar_index = BAR_INVALID;
	strscpy((char *)info->carveout_name, name, CARVEOUT_NAME_SIZE);

	list_add_tail(&info->node, &priv->carveout_infos);

	return info;
}

static int q_set_simple_region(struct carvnet *priv, struct rproc_mem_entry *m,
			       int i)
{
	u32 r_size, r_off, r_mem, r_locp, r_type, r_sub, r_value, q;
	ulong m_size = roundup_pow_of_two(m->len);
	struct carveout_info *info;
	u64 off64;
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

	info = q_get_carveout_info(priv, m->name);
	if (!info)
		return -ENOMEM;
	if (info->bar_index >= BAR_CNT) {
		dev_err(priv->dev, "Missing bar info for region: %s", m->name);
		return -EINVAL;
	}

	bar = info->bar_index;
	off64 = m->dma - priv->bar_info[info->bar_index].addr;
	if (off64 > UINT_MAX)
		return -EINVAL;
	r_off = (u32)off64;

	if (m_size < SZ_16K) {
		m_size = SZ_16K;
		r_size = 0;
	} else {
		r_size = fls(m_size / SZ_16K) - 1;
	}

	if (!IS_ALIGNED(r_off, SZ_16K)) {
		dev_err(priv->dev,
			"Mapping %d (%s) offset is not aligned to SZ_16K: 0x%x",
			i, m->name, r_off);
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
	for (q = 0; q < priv->num_quartiles; q++) {
		writel(r_value, priv->regs[q] + CARV_Q_AT_SRR(priv->q_user, i));

		dev_dbg(priv->dev,
		"QT%u: SRR=0x%x OFF=0x%x MEM=0x%x LOCP=%d SIZE=0x%x TYPE=0x%x SUB=%d ENA=1\n",
		q, readl(priv->regs[q] + CARV_Q_AT_SRR(priv->q_user, i)), r_off,
		r_mem, r_locp, r_size, r_type, r_sub);
	}

	return 0;
};

static int q_set_prio_region(struct carvnet *priv, struct carveout_info *info,
			     dma_addr_t dma, size_t len, int *prio_n)
{
	u32 r_size, r_off, r_mem, r_match, r_value, q;
	ulong m_size = roundup_pow_of_two(len);
	int i = *prio_n;
	u64 off64;
	int bar;

	if (*prio_n >= CARV_AT_PRIO_REGIONS) {
		dev_err(priv->dev, "Too many priority region mappings\n");
		return -EINVAL;
	}
	*prio_n = *prio_n + 1;

	off64 = dma - priv->bar_info[info->bar_index].addr;
	if (off64 > UINT_MAX)
		return -EINVAL;
	r_off = (u32)off64;

	if (m_size < SZ_4K) {
		m_size = SZ_4K;
		r_size = 0;
	} else {
		r_size = fls(m_size / SZ_4K) - 1;
	}

	if (!IS_ALIGNED(r_off, SZ_16K)) {
		dev_err(priv->dev,
			"Mapping %d (%s) offset is not aligned to SZ_16K: 0x%x",
			i, info->carveout_name, r_off);
		return -EINVAL;
	}

	if (!IS_ALIGNED(info->match_addr, m_size)) {
		dev_err(priv->dev,
			"Mapping %d (%s) match_addr is not aligned to m_size: 0x%x",
			i, info->carveout_name, info->match_addr);
		return -EINVAL;
	}

	r_off = r_off / SZ_16K;
	r_mem = bar;

	r_value = CARV_Q_AT_RCR_OFF(r_off) |
		CARV_Q_AT_RCR_SIZE(r_size) |
		CARV_Q_AT_RCR_MEM(r_mem) |
		CARV_Q_AT_RCR_ENA(1U);
	r_match = GET_CARV_Q_AT_RMR_MATCH(info->match_addr);

	for (q = 0; q < priv->num_quartiles; q++) {
		writel(r_value, priv->regs[q] + CARV_Q_AT_RCR(priv->q_user, i));
		writel(CARV_Q_AT_RMR_MATCH(r_match),
				priv->regs[q] + CARV_Q_AT_RMR(priv->q_user, i));

		dev_dbg(priv->dev,
			"QT%u: RCR=0x%x OFF=0x%x SIZE=0x%x MEM=0x%x ENA=1\n", q,
			readl(priv->regs[q] + CARV_Q_AT_RCR(priv->q_user, i)), r_off,
			r_size, r_mem);
		dev_dbg(priv->dev,
			"QT%u: RMR=0x%x MATCH=0x%x\n", q,
			readl(priv->regs[q] + CARV_Q_AT_RMR(priv->q_user, i)), r_match);
	}

	return 0;
}

static int g_get_bar(struct carvnet *priv, u8 bar_type, struct rproc_mem_entry *mem)
{
	struct carvnet_bar_info *bar_info = NULL;
	u64 diff, addr, addr_end;
	int i, ret;

	/* Try to find existing bar_info - matching type and <4GB offset */
	for (i = 0; i < priv->bar_info_cnt; i++) {
		bar_info = &priv->bar_info[i];
		if (bar_info->bar_type != bar_type)
			continue;

		/* Check if superset of current BAR + the new region fits into 4GB */
		addr     = min(bar_info->addr, mem->dma);
		addr_end = max(bar_info->addr_end, mem->dma + mem->len);

		diff = addr_end - addr;
		if (diff < SZ_4G) {
			bar_info->addr = addr;
			bar_info->addr_end = addr_end;
			return i;
		}
	}

	if (priv->bar_info_cnt == BAR_CNT) {
		dev_err(priv->dev, "No free BAR available for resource: %s\n",
			mem->name);
		return -EINVAL;
	}

	/* If not found, create a new bar_info to be used */
	ret = priv->bar_info_cnt;
	bar_info = &priv->bar_info[priv->bar_info_cnt];
	bar_info->bar_type = bar_type;
	bar_info->addr = mem->dma;
	if (mem->dma > ULONG_MAX - mem->len) /* Overflow check to comply with coverity */
		return -EINVAL;
	bar_info->addr_end = mem->dma + mem->len;
	priv->bar_info_cnt++;

	return ret;
}

static void q_clear_simple_region(struct carvnet *priv, int i)
{
	u32 q;
	/* Clean AT_SRR and disable it */
	for (q = 0; q < priv->num_quartiles; q++)
		writel(0, priv->regs[q] + CARV_Q_AT_SRR(priv->q_user, i));
}

static void q_clear_prio_region(struct carvnet *priv, int i)
{
	u32 q;
	/* Clean AT_RCR/AT_RMRR and disable it */
	for (q = 0; q < priv->num_quartiles; q++) {
		writel(0, priv->regs[q] + CARV_Q_AT_RCR(priv->q_user, i));
		writel(0, priv->regs[q] + CARV_Q_AT_RMR(priv->q_user, i));
	}
}

static void q_clear_mapper(struct carvnet *priv)
{
	int i;
	u32 q;

	/* Clear all simple region mappings */
	for (i = 0; i < CARV_AT_SIMPLE_REGIONS; i++)
		q_clear_simple_region(priv, i);

	/* Clear all prio region mappings */
	for (i = 0; i < CARV_AT_PRIO_REGIONS; i++)
		q_clear_prio_region(priv, i);

	/* Clear BARs */
	for (i = 0; i < BAR_CNT; i++) {
		for (q = 0; q < priv->num_quartiles; q++) {
			writel(0, priv->regs[q] + CARV_Q_AT_BARL(priv->q_user, i));
			writel(0, priv->regs[q] + CARV_Q_AT_BARH(priv->q_user, i));
		}
	}
}

static int q_set_mapper(struct carvnet *priv)
{
	struct carveout_info *info, *aggr_info = NULL;
	int simple_n = 0, prio_n = 0, ret = 0, i;
	struct rproc_mem_entry *m;
	struct rproc *rproc;
	size_t aggr_len = 0;
	dma_addr_t aggr_dma;
	u32 q;

	q_clear_mapper(priv);

	rproc = dev_get_drvdata(priv->dev);

	/* Assign BAR index to possible auto-generated carveouts (vrings etc) */
	list_for_each_entry(m, &rproc->carveouts, node) {
		info = q_get_carveout_info(priv, m->name);
		if (!info)
			return -ENOMEM;

		if (info->bar_index == BAR_INVALID) {
			ret = g_get_bar(priv, CARV_Q_AT_BARL_MEMTYPE_DEFAULT, m);
			if (ret < 0)
				return ret;
			info->bar_index = (u32)ret;
		}
	}

	/* Write all BARs to registers */
	for (i = 0; i < priv->bar_info_cnt; i++) {
		for (q = 0; q < priv->num_quartiles; q++) {
			writel(CARV_Q_AT_BARL_ADDR(priv->bar_info[i].addr) |
				priv->bar_info[i].bar_type,
				priv->regs[q] + CARV_Q_AT_BARL(priv->q_user, i));
			writel(CARV_Q_AT_BARH_ADDR(priv->bar_info[i].addr),
				priv->regs[q] + CARV_Q_AT_BARH(priv->q_user, i));
		}
	}

	/* Set mapping of the memory regions */
	list_for_each_entry(m, &rproc->carveouts, node) {
		info = q_get_carveout_info(priv, m->name);
		if (!info)
			return -EINVAL;

		if (strncmp(m->name, FIRMWARE_SEGMENT_NAME,
			    strlen(FIRMWARE_SEGMENT_NAME)) == 0) {
			if (simple_n >= CARV_AT_SIMPLE_REGIONS)
				return -EINVAL;
			ret = q_set_simple_region(priv, m, simple_n++);
		} else {
			if (aggr_info && (aggr_dma + aggr_len == m->dma) &&
				aggr_info->bar_index == info->bar_index) {
				/* Append this region into single RCR mapping */
				aggr_len += m->len;
			} else {
				if (aggr_len)
					ret = q_set_prio_region(priv, aggr_info, aggr_dma,
								aggr_len, &prio_n);
				aggr_dma  = m->dma;
				aggr_len  = m->len;
				aggr_info = info;
			}
		}

		if (ret)
			return ret;
	}
	if (aggr_info)
		ret = q_set_prio_region(priv, aggr_info, aggr_dma, aggr_len, &prio_n);

	return ret;
}

static int q_msg_send_info(struct carvnet *priv, u32 info)
{
	/* Send info by mailbox 0 */
	writel(info, priv->regs[0] + CARV_Q_MB_INFO_CR(0, 0));

	return 0;
}

static void q_stop(struct carvnet *priv)
{
	u32 done;
	int ret;

	q_msg_send_info(priv, (u32)CMD_FW_STOP);

	/* Wait for the MB_CR Done bit with timeout 10ms */
	ret = read_poll_timeout(GET_CARV_Q_MB_CR_DONE, done, done == 1, 100, 10000, false,
			readl(priv->regs[0] + CARV_Q_MB_CR(priv->q_user)));
	if (ret)
		dev_info(priv->dev, "Waiting Done bit timeout\n");

	/* Stop the FW */
	writel(0, priv->regs[0] + CARV_Q_MB_CR(0));

	/* Disable Functional Interrupts: User configurable mask */
	writel(0, priv->regs[0] + CARV_Q_MB_IER(0));
}

static int carvnet_rproc_add_mem_regions(struct rproc *rproc)
{
	struct carvnet *priv = rproc->priv;
	struct rproc_mem_entry *mem;
	struct carveout_info *info;
	int ret;

	if (list_empty(&priv->carveout_infos))
		return 0;

	list_for_each_entry(info, &priv->carveout_infos, node) {
		mem = rproc_mem_entry_init(priv->dev, info->va,
					   (dma_addr_t)info->base_addr,
					   (size_t)info->size, 0U,
					   NULL, NULL,
					   (const char *)info->carveout_name);
		if (!mem)
			return -ENOMEM;

		/* Assign BAR to this Carveout */
		ret = g_get_bar(priv, info->bar_type, mem);
		if (ret < 0) {
			kfree(mem);
			return ret;
		}

		info->bar_index = (u32)ret;
		rproc_add_carveout(rproc, mem);
	}

	return 0;
}

static int carvnet_rproc_parse_fw(struct rproc *rproc,
				  const struct firmware *fw)
{
	int ret;

	ret = carvnet_rproc_add_mem_regions(rproc);
	if (ret) {
		dev_err(&rproc->dev, "Failed to parse memory regions\n");
		return ret;
	}

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret)
		dev_warn(&rproc->dev, "No resource table in fw\n");

	return ret;
}

static int carvnet_rproc_start(struct rproc *rproc)
{
	struct carvnet *priv = rproc->priv;
	int ret;

	/* Enable Functional Interrupts: User configurable mask */
	writel(IRQ_ENABLE_MASK, priv->regs[0] + CARV_Q_MB_IER(0));

	/* Set Mapper */
	ret = q_set_mapper(priv);
	if (ret)
		goto err;

	/* Start the FW */
	writel(CARV_Q_MB_CR_ENA(1) | CARV_Q_MB_CR_RSTSTART(1), priv->regs[0] + CARV_Q_MB_CR(0));

	return 0;

err:
	q_stop(priv);

	return ret;
}

static int carvnet_rproc_stop(struct rproc *rproc)
{
	struct carvnet *priv = rproc->priv;
	struct carveout_info *entry, *tmp;

	q_stop(priv);

	/* clean up match_addr allocations */
	list_for_each_entry_safe(entry, tmp, &priv->carveout_infos, node) {
		if (!entry->va) {
			list_del(&entry->node);
			kfree(entry);
		}
	}

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
	u32 i;
	u32 q;

	for (q = 0; q < priv->num_quartiles; q++) {
		seq_printf(seq, "Quartile %u:\n", q);

		/* User 0 set */
		for (i = 0; q_regmap_tab_u[i].name; i++)
			seq_printf(seq, "%-14s: 0x%08x\n", q_regmap_tab_u[i].name,
				   readl(priv->regs[q] + q_regmap_tab_u[i].addr +
				   CARV_Q_USER_OFFSET(0)));
	}
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

/* carvnet_release_carveout() - release acquired carveout */
static int carvnet_release_carveout(struct rproc *rproc,
				    struct rproc_mem_entry *mem)
{
	struct device *dev = &rproc->dev;

	dma_free_coherent(dev->parent, mem->len, mem->va, mem->dma);
	return 0;
}

static int carvnet_rproc_elf_load(struct rproc *rproc,
				  const struct firmware *fw)
{
	struct carvnet *priv = rproc->priv;
	struct device *dev = &rproc->dev;
	const u8 *elf_data = fw->data;
	struct rproc_mem_entry *mem;
	struct elf32_phdr *phdr;
	struct elf32_hdr *ehdr;
	dma_addr_t dma;
	int i, ret = 0;
	void *va;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

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

		dev_dbg(dev,
		"phdr[%i]: type %d da 0x%x offset 0x%x flags 0x%x memsz 0x%x filesz 0x%x\n",
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

		dma = 0U;
		va = dma_alloc_coherent(dev->parent, segsize, &dma, GFP_KERNEL);
		if (!va)
			return -ENOMEM;

		mem = rproc_mem_entry_init(dev, va, dma,
					   segsize, da, NULL, carvnet_release_carveout, "%s%d",
					   FIRMWARE_SEGMENT_NAME, i);
		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);

		/* Save segment in the reserved carveout memory region */
		memcpy(mem->va, elf_data + offset, filesz);
	}

	/* Add debugfs dump regions */
	priv->subdentry = debugfs_create_dir("carveouts", rproc->dbg_dir);
	list_for_each_entry(mem, &rproc->carveouts, node)
		debugfs_create_file(mem->name, 0400, priv->subdentry, mem,
				    &rproc_carveout_hexdump_fops);

	return ret;
}

static int carvnet_rproc_handle_rsc(struct rproc *rproc, u32 rsc_type, void *rsc,
		  int offset, int avail)
{
	struct fw_rsc_vendor_carv_match_addr *orig_rsc;
	struct carvnet *priv = rproc->priv;
	struct carveout_info *info;

	if (rsc_type != RSC_VENDOR_CARV_MATCH_ADDR)
		return RSC_IGNORED;

	orig_rsc = (struct fw_rsc_vendor_carv_match_addr *)rsc;

	info = q_get_carveout_info(priv, (const char *)orig_rsc->carveout_name);
	if (!info)
		return -ENOMEM;

	info->match_addr = orig_rsc->match_addr;

	return RSC_HANDLED;
}

static const struct rproc_ops carvnet_rproc_ops = {
	.start		= carvnet_rproc_start,
	.stop		= carvnet_rproc_stop,
	.kick		= carvnet_rproc_kick,
	.parse_fw	= carvnet_rproc_parse_fw,
	.load		= carvnet_rproc_elf_load,
	.handle_rsc	= carvnet_rproc_handle_rsc,
	.find_loaded_rsc_table	= rproc_elf_find_loaded_rsc_table,
};

static int carvnet_rproc_parse_dt_mem(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct carvnet *priv = rproc->priv;
	struct of_phandle_iterator it;
	struct reserved_mem *rmem;
	struct carveout_info *info;
	unsigned long remap_flag;
	u32 resm_idx = 1, value;
	u8 bar_type;
	void *va;
	int ret;

	/* Parse associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);

	ret = of_reserved_mem_device_init_by_idx(dev, np, 0);
	if (ret) {
		dev_err(dev, "Failed to associate vdev resmem: %d\n",
			ret);
		return ret;
	}

	/* Skip reserved memory region - already assigned above */
	of_phandle_iterator_next(&it);

	while (of_phandle_iterator_next(&it) == 0) {

		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			of_node_put(it.node);
			dev_err(dev,
				"unable to acquire memory region\n");
			return -ENOMEM;
		}

		value = 0U;
		of_property_read_u32_index(np, "nxp,s32n7-carv-mem-region-types",
								  resm_idx, &value);

		if (value == 0U) {
			bar_type = CARV_Q_AT_BARL_MEMTYPE_NCOH;
			remap_flag = MEMREMAP_WC;
		} else if (value == 1U) {
			bar_type = CARV_Q_AT_BARL_MEMTYPE_SHAR;
			remap_flag = MEMREMAP_WC;
		} else {
			bar_type = CARV_Q_AT_BARL_MEMTYPE_DEFAULT;
			remap_flag = MEMREMAP_WB;
		}

		va = devm_memremap(dev, rmem->base, rmem->size, remap_flag);
		if (!va) {
			dev_err(dev, "Failed to ioremap resmem: %d %s\n",
				resm_idx, it.node->name);
			return -ENOMEM;
		}

		info = q_get_carveout_info(priv, it.node->name);
		if (!info)
			return -ENOMEM;

		info->va = va;
		info->base_addr = rmem->base;
		info->size = rmem->size;
		info->bar_type = bar_type;

		resm_idx++;
	}

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
		dev_err(dev, "Fault: WD timeout/DTCM parity/no software vector/illegal instruction\n");

	if (irq_mask & CARV_INT_F_AHB_BIT)
		dev_err(dev, "Fault: AHB access response error\n");

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
	u32 irq_mask, ier_mask;

	irq_mask = readl(priv->regs[0] + CARV_Q_MB_IR(priv->q_user));
	ier_mask = readl(priv->regs[0] + CARV_Q_MB_IER(priv->q_user));

	/* Write back the interrupt bits to clear the interrupt in the CARV hardware */
	irq_mask &= (ier_mask);
	writel(irq_mask, priv->regs[0] + CARV_Q_MB_IR(priv->q_user));

	if (GET_CARV_INT_UPDATE(irq_mask))
		schedule_work(&priv->rproc_work);

	if (GET_CARV_INT_ACTIVATED(irq_mask))
		dev_info(priv->dev, "Activated\n");

	if (IRQ_ENABLE_MASK & irq_mask)
		carvnet_rproc_fault_irq_msg(priv->dev, irq_mask);

	return IRQ_HANDLED;
}

static int carvnet_rproc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const char *fw_name = NULL;
	struct carvnet *priv;
	struct rproc *rproc;
	int irq;
	int ret;
	u32 q;

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
	priv->bar_info_cnt = 0U;
	INIT_LIST_HEAD(&priv->carveout_infos);

	ret = carvnet_rproc_parse_dt_mem(rproc);
	if (ret)
		return ret;

	dev_set_drvdata(dev, rproc);

	if (pdev->num_resources > MAX_QUARTILES || pdev->num_resources == 0)
		return dev_err_probe(dev, -EINVAL, "Invalid number of Reg: %u\n",
				     pdev->num_resources);

	priv->num_quartiles = pdev->num_resources;

	for (q = 0; q < priv->num_quartiles; q++) {
		priv->regs[q] = devm_platform_ioremap_resource(pdev, q);
		if (!priv->regs[q])
			return dev_err_probe(dev, ret, "Failed to ioremap register region %u", q);
	}

	rproc->auto_boot = auto_boot;

	/* We are using only first user config */
	priv->q_user = 0;

	INIT_WORK(&priv->rproc_work, carvnet_rproc_vq_work);

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return dev_err_probe(dev, ret, "rproc_add failed\n");

	debugfs_create_file("dump_regs", 0400, rproc->dbg_dir, priv,
			    &carvnet_dump_regs_fops);

	irq = platform_get_irq(pdev, 0U);
	if (irq < 0)
		return dev_err_probe(dev, irq, "platform_get_irq failed\n");

	ret = devm_request_irq(dev, (u32)irq, carvnet_rproc_handle_irq, 0,
			pdev->name, priv);
	if (ret)
		return dev_err_probe(dev, ret,
				"unable to request IRQ%d for CARV device\n",
				irq);

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
