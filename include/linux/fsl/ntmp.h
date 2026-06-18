/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2025 NXP */
#ifndef __NETC_NTMP_H
#define __NETC_NTMP_H

#include <linux/bitops.h>
#include <linux/if_ether.h>

#define NTMP_NULL_ENTRY_ID		0xffffffffU
#define NETC_CBDR_BD_NUM		256

union netc_cbd {
	struct {
		__le64 addr;
		__le32 len;
#define NTMP_RESP_LEN		GENMASK(19, 0)
#define NTMP_REQ_LEN		GENMASK(31, 20)
#define NTMP_LEN(req, resp)	(FIELD_PREP(NTMP_REQ_LEN, (req)) | \
				((resp) & NTMP_RESP_LEN))
		u8 cmd;
#define NTMP_CMD_DELETE		BIT(0)
#define NTMP_CMD_UPDATE		BIT(1)
#define NTMP_CMD_QUERY		BIT(2)
#define NTMP_CMD_ADD		BIT(3)
#define NTMP_CMD_QD		(NTMP_CMD_QUERY | NTMP_CMD_DELETE)
#define NTMP_CMD_QU		(NTMP_CMD_QUERY | NTMP_CMD_UPDATE)
#define NTMP_CMD_AU		(NTMP_CMD_ADD | NTMP_CMD_UPDATE)
#define NTMP_CMD_AQ		(NTMP_CMD_ADD | NTMP_CMD_QUERY)
#define NTMP_CMD_AQU		(NTMP_CMD_AQ | NTMP_CMD_UPDATE)
		u8 access_method;
#define NTMP_ACCESS_METHOD	GENMASK(7, 4)
#define NTMP_AM_ENTRY_ID	0
#define NTMP_AM_EXACT_KEY	1
#define NTMP_AM_SEARCH		2
#define NTMP_AM_TERNARY_KEY	3
		u8 table_id;
		u8 ver_cci_rr;
#define NTMP_HDR_VERSION	GENMASK(5, 0)
#define NTMP_HDR_VER2		2
#define NTMP_CCI		BIT(6)
#define NTMP_RR			BIT(7)
		__le32 resv[3];
		__le32 npf;
#define NTMP_NPF		BIT(15)
	} req_hdr;	/* NTMP Request Message Header Format */

	struct {
		__le32 resv0[3];
		__le16 num_matched;
		__le16 error_rr;
#define NTMP_RESP_ERROR		GENMASK(11, 0)
#define NTMP_RESP_RR		BIT(15)
		__le32 resv1[4];
	} resp_hdr; /* NTMP Response Message Header Format */
};

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
	struct netc_cbdr_regs regs;

	int bd_num;
	int next_to_use;
	int next_to_clean;

	int dma_size;
	void *addr_base;
	void *addr_base_align;
	dma_addr_t dma_base;
	dma_addr_t dma_base_align;

	spinlock_t ring_lock; /* Avoid race condition */
};

struct netc_cbdrs {
	int cbdr_num;	/* number of control BD ring */
	int cbdr_size;	/* number of BDs per control BD ring */
	struct device *dma_dev;
	struct netc_cbdr *ring;
	struct netc_tbl_vers tbl;
};

enum netc_dev_type {
	NETC_DEV_ENETC,
	NETC_DEV_SWITCH
};

struct ntmp_priv;

struct ntmp_ops {
	u64 (*adjust_base_time)(struct ntmp_priv *priv, u64 bt, u32 ct);
	u32 (*get_tgst_free_words)(struct ntmp_priv *priv);
};

struct ntmp_priv {
	enum netc_dev_type dev_type;
	struct netc_cbdrs cbdrs;
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
int netc_setup_cbdr(struct device *dev, int cbd_num,
		    struct netc_cbdr_regs *regs,
		    struct netc_cbdr *cbdr);
void netc_teardown_cbdr(struct device *dev, struct netc_cbdr *cbdr);

/* NTMP APIs */
int ntmp_maft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			struct maft_entry_data *maft);
int ntmp_maft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct maft_entry_data *maft);
int ntmp_maft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_rsst_query_or_update_entry(struct netc_cbdrs *cbdrs,
				    u32 *table, int count, bool query);
int ntmp_rpt_add_or_update_entry(struct netc_cbdrs *cbdrs,
				 struct ntmp_rpt_entry *entry);
int ntmp_rpt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
#else
static inline int netc_setup_cbdr(struct device *dev, int cbd_num,
				  struct netc_cbdr_regs *regs,
				  struct netc_cbdr *cbdr)
{
	return 0;
}

static inline void netc_teardown_cbdr(struct device *dev,
				      struct netc_cbdr *cbdr)
{
}

static inline int ntmp_maft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				      struct maft_entry_data *maft)
{
	return 0;
}

static inline int ntmp_maft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct maft_entry_data *maft)
{
	return 0;
}

static inline int ntmp_maft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_rsst_query_or_update_entry(struct netc_cbdrs *cbdrs,
						  u32 *table, int count,
						  bool query)
{
	return 0;
}

static inline int ntmp_rpt_add_or_update_entry(struct netc_cbdrs *cbdrs,
					       struct ntmp_rpt_entry *entry)
{
	return 0;
}

static inline int ntmp_rpt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

#endif

#endif
