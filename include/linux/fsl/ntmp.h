/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2025 NXP */
#ifndef __NETC_NTMP_H
#define __NETC_NTMP_H

#include <linux/bitops.h>
#include <linux/if_ether.h>

struct maft_keye_data {
	u8 mac_addr[ETH_ALEN];
	__le16 resv;
};

struct maft_cfge_data {
	__le16 si_bitmap;
	__le16 resv;
};

#pragma pack(1)

struct rpt_cfge_data {
	__le32 cir;
	__le32 cbs;
	__le32 eir;
	__le32 ebs;
	__le16 cfg;
#define RPT_MREN	BIT(0)
#define RPT_DOY		BIT(1)
#define RPT_CM		BIT(2)
#define RPT_CF		BIT(3)
#define RPT_NDOR	BIT(4)
#define RPT_SDU_TYPE	GENMASK(6, 5)
};

struct rpt_stse_data {
	__le64 byte_count;
	__le32 drop_frames;
	__le32 rev0;
	__le32 dr0_grn_frames;
	__le32 rev1;
	__le32 dr1_grn_frames;
	__le32 rev2;
	__le32 dr2_ylw_frames;
	__le32 rev3;
	__le32 remark_ylw_frames;
	__le32 rev4;
	__le32 dr3_red_frames;
	__le32 rev5;
	__le32 remark_red_frames;
	__le32 rev6;
	__le32 lts;
	__le32 bci;
	__le32 bcf_bcs;
#define RPT_BCF		GENMASK(30, 0)
#define RPT_BCS		BIT(31)
	__le32 bei;
	__le32 bef_bes;
#define RPT_BEF		GENMASK(30, 0)
#define RPT_BES		BIT(31)
};

#pragma pack()

struct rpt_fee_data {
	u8 fen;
#define RPT_FEN		BIT(0)
};

struct rpt_pse_data {
	u8 mr;
#define RPT_MR		BIT(0)
};

struct netc_cbdr_regs {
	void __iomem *pir;
	void __iomem *cir;
	void __iomem *mr;

	void __iomem *bar0;
	void __iomem *bar1;
	void __iomem *lenr;
};

struct netc_tbl_vers {
	u8 maft_ver;
	u8 rsst_ver;
	u8 tgst_ver;
	u8 rpt_ver;
};

struct netc_cbdr {
	struct device *dev;
	struct netc_cbdr_regs regs;

	int bd_num;
	int next_to_use;
	int next_to_clean;

	int dma_size;
	void *addr_base;
	void *addr_base_align;
	dma_addr_t dma_base;
	dma_addr_t dma_base_align;

	/* Serialize the order of command BD ring */
	spinlock_t ring_lock;
};

struct ntmp_user;

struct ntmp_ops {
	u64 (*adjust_base_time)(struct ntmp_user *user, u64 base_time,
				u32 cycle_time);
	u32 (*get_tgst_free_words)(struct ntmp_user *user);
};

struct ntmp_user {
	int cbdr_num;	/* number of control BD ring */
	struct device *dev;
	struct netc_cbdr *ring;
	struct netc_tbl_vers tbl;
	const struct ntmp_ops *ops;
};

struct maft_entry_data {
	struct maft_keye_data keye;
	struct maft_cfge_data cfge;
};

struct ntmp_rpt_entry {
	u32 entry_id;
	struct rpt_cfge_data cfge;
	struct rpt_fee_data fee;
	struct rpt_stse_data stse;
	struct rpt_pse_data pse;
};

#if IS_ENABLED(CONFIG_NXP_NETC_LIB)
int ntmp_init_cbdr(struct netc_cbdr *cbdr, struct device *dev,
		   const struct netc_cbdr_regs *regs);
void ntmp_free_cbdr(struct netc_cbdr *cbdr);

/* NTMP APIs */
int ntmp_maft_add_entry(struct ntmp_user *user, u32 entry_id,
			struct maft_entry_data *maft);
int ntmp_maft_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct maft_entry_data *maft);
int ntmp_maft_delete_entry(struct ntmp_user *user, u32 entry_id);
int ntmp_rsst_update_entry(struct ntmp_user *user, const u32 *table,
			   int count);
int ntmp_rsst_query_entry(struct ntmp_user *user,
			  u32 *table, int count);
int ntmp_rpt_add_entry(struct ntmp_user *user, struct ntmp_rpt_entry *entry);
int ntmp_rpt_delete_entry(struct ntmp_user *user, u32 entry_id);
#else
static inline int ntmp_init_cbdr(struct netc_cbdr *cbdr, struct device *dev,
				 const struct netc_cbdr_regs *regs)
{
	return 0;
}

static inline void ntmp_free_cbdr(struct netc_cbdr *cbdr)
{
}

static inline int ntmp_maft_add_entry(struct ntmp_user *user, u32 entry_id,
				      struct maft_entry_data *maft)
{
	return 0;
}

static inline int ntmp_maft_query_entry(struct ntmp_user *user, u32 entry_id,
					struct maft_entry_data *maft)
{
	return 0;
}

static inline int ntmp_maft_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return 0;
}

static inline int ntmp_rsst_update_entry(struct ntmp_user *user,
					 const u32 *table, int count)
{
	return 0;
}

static inline int ntmp_rsst_query_entry(struct ntmp_user *user,
					u32 *table, int count)
{
	return 0;
}

static inline int ntmp_rpt_add_entry(struct ntmp_user *user,
				     struct ntmp_rpt_entry *entry)
{
	return 0;
}

static inline int ntmp_rpt_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return 0;
}

#endif

#endif
