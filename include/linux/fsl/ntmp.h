/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2025 NXP */
#ifndef __NETC_NTMP_H
#define __NETC_NTMP_H

#include <linux/bitops.h>
#include <linux/if_ether.h>

#define NTMP_NULL_ENTRY_ID		0xffffffffU
#define NETC_CBDR_BD_NUM		256

#define ISIT_FRAME_KEY_LEN		16

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

struct ist_cfge_data {
	__le32 cfg;
#define IST_SFE			BIT(0)
#define IST_RRT			BIT(1)
#define IST_BL2F		BIT(2)
#define IST_IPV			GENMASK(7, 4)
#define	IST_OIPV		BIT(8)
#define IST_DR			GENMASK(10, 9)
#define IST_ODR			BIT(11)
#define IST_IMIRE		BIT(12)
#define IST_TIMERCAPE		BIT(13)
#define IST_SPPD		BIT(15)
#define IST_ISQGA		GENMASK(17, 16)
#define IST_ORP			BIT(18)
#define IST_OSGI		BIT(19)
#define IST_HR			GENMASK(23, 20)
/* VERSION 0 */
#define IST_V0_FA		GENMASK(26, 24)
#define IST_V0_SDU_TYPE		GENMASK(28, 27)
/* VERSION 1 */
#define IST_V1_FA		GENMASK(27, 24)
#define	IST_V1_SDU_TYPE		GENMASK(29, 28)
#define IST_FA_NO_SI_BITMAP	1
#define IST_SWITCH_FA_SF	2
#define IST_SWITCH_FA_BF	3
#define IST_SWITCH_FA_SF_COPY	4
#define IST_SDFA		BIT(30)
#define IST_OSDFA		BIT(31)
	__le16 msdu;
	__le16 switch_cfg; /* Only applicable to NETC switch */
#define IST_IFME_LEN_CHANGE	GENMASK(6, 0)
#define	IST_EPORT		GENMASK(11, 7)
#define IST_OETEID		GENMASK(13, 12)
#define IST_CTD			GENMASK(15, 14)
	__le32 isqg_eid; /* Only applicable to NETC switch */
	__le32 rp_eid;
	__le32 sgi_eid;
	__le32 ifm_eid; /* Only applicable to NETC switch */
	__le32 et_eid; /* Only applicable to NETC switch */
	__le32 isc_eid;
	__le32 bitmap_evmeid; /* Only applicable to NETC switch */
#define IST_EGRESS_PORT_BITMAP	GENMASK(23, 0)
#define IST_EVMEID		GENMASK(27, 24)
	__le16 si_bitmap;
};

struct sgit_acfge_data {
	__le32 admin_sgcl_eid;
	__le64 admin_base_time;
	__le32 admin_cycle_time_ext;
};

struct sgit_sgise_data {
	__le32 oper_sgcl_eid;
	__le64 config_change_time;
	__le64 oper_base_time;
	__le32 oper_cycle_time_ext;
	u8 info;
#define SGIT_OEX		BIT(0)
#define SGIT_IRX		BIT(1)
#define SGIT_STATE		GENMASK(4, 2)
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

struct isit_keye_data {
	__le32 key_aux;
#define ISIT_KEY_TYPE			GENMASK(1, 0)
#define ISIT_KEY_TYPE0_SMAC_VLAN	0
#define ISIT_KEY_TYPE1_DMAC_VLAN	1
#define ISIT_SRC_PORT_ID		GENMASK(6, 2)
#define ISIT_SPM			BIT(7)
	u8 frame_key[ISIT_FRAME_KEY_LEN];
};

struct isft_keye_data {
	__le32 is_eid;
	u8 pcp;
#define ISFT_PCP		GENMASK(2, 0)
	u8 resv[3];
};

struct isft_cfge_data {
	__le16 cfg;
#define ISFT_IPV		GENMASK(3, 0)
#define ISFT_OIPV		BIT(4)
#define ISFT_DR			GENMASK(6, 5)
#define ISFT_ODR		BIT(7)
#define ISFT_IMIRE		BIT(8)
#define ISFT_TIMECAPE		BIT(9)
#define ISFT_OSGI		BIT(10)
#define ISFT_CTD		BIT(11)
#define ISFT_ORP		BIT(12)
#define ISFT_SDU_TYPE		GENMASK(14, 13)
	__le16 msdu;
	__le32 rp_eid;
	__le32 sgi_eid;
	__le32 isc_eid;
};

struct sgit_cfge_data {
	u8 cfg;
#define SGIT_OEXEN		BIT(0)
#define SGIT_IRXEN		BIT(1)
#define SGIT_SDU_TYPE		GENMASK(3, 2)
};

struct sgit_icfge_data {
	u8 icfg;
#define SGIT_IPV		GENMASK(3, 0)
#define SGIT_OIPV		BIT(4)
#define SGIT_GST		BIT(5)
#define SGIT_CTD		BIT(6)
};

struct sgclt_ge {
	__le32 interval;
	__le32 cfg;
#define SGCLT_IOM		GENMASK(23, 0)
#define SGCLT_IPV		GENMASK(27, 24)
#define SGCLT_OIPV		BIT(28)
#define SGCLT_CTD		BIT(29)
#define SGCLT_IOMEN		BIT(30)
#define SGCLT_GTST		BIT(31)
};

struct sgclt_cfge_data {
	__le32 cycle_time;
	u8 list_length;
	u8 resv0;
	u8 ext_cfg;
#define SGCLT_EXT_OIPV		BIT(0)
#define SGCLT_EXT_IPV		GENMASK(4, 1)
#define SGCLT_EXT_CTD		BIT(5)
#define SGCLT_EXT_GTST		BIT(6)
	u8 resv1;
	struct sgclt_ge ge[];
};

struct isct_stse_data {
	__le32 rx_count;
	__le32 resv0;
	__le32 msdu_drop_count;
	__le32 resv1;
	__le32 policer_drop_count;
	__le32 resv2;
	__le32 sg_drop_count;
	__le32 resv3;
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
	u8 isit_ver;
	u8 ist_ver;
	u8 isft_ver;
	u8 sgit_ver;
	u8 sgclt_ver;
	u8 isct_ver;
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

struct ntmp_isit_entry {
	u32 entry_id;  /* hardware assigns entry ID */
	struct isit_keye_data keye;
	__le32 is_eid; /* cfge data */
};

struct ntmp_ist_entry {
	u32 entry_id; /* software assigns entry ID */
	struct ist_cfge_data cfge;
};

struct ntmp_isft_entry {
	u32 entry_id; /* hardware assigns entry ID */
	struct isft_keye_data keye;
	struct isft_cfge_data cfge;
};

struct ntmp_sgit_entry {
	u32 entry_id; /* software assigns entry ID */
	struct sgit_acfge_data acfge;
	struct sgit_cfge_data cfge;
	struct sgit_icfge_data icfge;
	struct sgit_sgise_data sgise;
};

struct ntmp_sgclt_entry {
	u32 entry_id;
	u8 ref_count; /* SGCLSE_DATA */
	struct sgclt_cfge_data cfge; /* Must be last member */
};

struct ntmp_isct_entry {
	u32 entry_id;
	struct isct_stse_data stse;
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
int ntmp_ist_add_or_update_entry(struct netc_cbdrs *cbdrs,
				 struct ntmp_ist_entry *entry);
int ntmp_ist_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_isct_operate_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			    int cmd, struct isct_stse_data *data);
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

static inline int ntmp_ist_add_or_update_entry(struct netc_cbdrs *cbdrs,
					       struct ntmp_ist_entry *entry)
{
	return 0;
}

static inline int ntmp_ist_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_isct_operate_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					  int cmd, struct isct_stse_data *stse)
{
	return 0;
}

#endif

#endif
