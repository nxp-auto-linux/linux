/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * NTMP table request and response data buffer formats
 * Copyright 2025 NXP
 */

#ifndef __NTMP_PRIVATE_H
#define __NTMP_PRIVATE_H

#include <linux/bitfield.h>
#include <linux/fsl/ntmp.h>

#define NTMP_EID_REQ_LEN	8
#define NETC_CBDR_BD_NUM	256
#define TGST_MAX_ENTRY_NUM	64

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
#define NTMP_CMD_QU		(NTMP_CMD_QUERY | NTMP_CMD_UPDATE)
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

struct ntmp_dma_buf {
	struct device *dev;
	size_t size;
	void *buf;
	dma_addr_t dma;
};

struct ntmp_cmn_req_data {
	__le16 update_act;
	u8 dbg_opt;
	u8 tblv_qact;
#define NTMP_QUERY_ACT		GENMASK(3, 0)
#define NTMP_TBL_VER		GENMASK(7, 4)
#define NTMP_TBLV_QACT(v, a)	(FIELD_PREP(NTMP_TBL_VER, (v)) | \
				 ((a) & NTMP_QUERY_ACT))
};

struct ntmp_cmn_resp_query {
	__le32 entry_id;
};

/* Generic structure for request data by entry ID  */
struct ntmp_req_by_eid {
	struct ntmp_cmn_req_data crd;
	__le32 entry_id;
};

/* MAC Address Filter Table Request Data Buffer Format of Add action */
struct maft_req_add {
	struct ntmp_req_by_eid rbe;
	struct maft_keye_data keye;
	struct maft_cfge_data cfge;
};

/* MAC Address Filter Table Response Data Buffer Format of Query action */
struct maft_resp_query {
	__le32 entry_id;
	struct maft_keye_data keye;
	struct maft_cfge_data cfge;
};

/* RSS Table Request Data Buffer Format of Update action */
struct rsst_req_update {
	struct ntmp_req_by_eid rbe;
	u8 groups[];
};

/* Time Gate Scheduling Table Resquet and Response Data Buffer Format */
struct tgst_ge {
	__le32 interval;
	u8 tc_state;
	u8 resv0;
	u8 hr_cb;
#define HR_CB_SET_GATES		0
#define HR_CB_SET_AND_HOLD	1
#define HR_CB_SET_AND_RELEASE	2
	u8 resv1;
};

#pragma pack(1)

struct tgst_cfge_data {
	__le64 admin_bt;
	__le32 admin_ct;
	__le32 admin_ct_ext;
	__le16 admin_cl_len;
	__le16 resv;
	struct tgst_ge ge[];
};

struct tgst_olse_data {
	__le64 oper_cfg_ct;
	__le64 oper_cfg_ce;
	__le64 oper_bt;
	__le32 oper_ct;
	__le32 oper_ct_ext;
	__le16 oper_cl_len;
	__le16 resv;
	struct tgst_ge ge[];
};

struct tgst_req_update {
	struct ntmp_req_by_eid rbe;
	struct tgst_cfge_data cfge;
};

struct tgst_resp_status {
	__le64 cfg_ct;
	__le32 status_resv;
};

struct tgst_resp_query {
	struct tgst_resp_status status;
	__le32 entry_id;
	u8 data[];
};

/* Rate Policer Table Request and Response Data Buffer Format */
struct rpt_req_ua {
	struct ntmp_req_by_eid rbe;
	struct rpt_cfge_data cfge;
	struct rpt_fee_data fee;
};

struct rpt_resp_query {
	__le32 entry_id;
	struct rpt_stse_data stse;
	struct rpt_cfge_data cfge;
	struct rpt_fee_data fee;
	struct rpt_pse_data pse;
};

#pragma pack()

struct tgst_query_data {
	__le64 config_change_time;
	__le64 admin_bt;
	__le32 admin_ct;
	__le32 admin_ct_ext;
	__le16 admin_cl_len;
	__le64 oper_cfg_ct;
	__le64 oper_cfg_ce;
	__le64 oper_bt;
	__le32 oper_ct;
	__le32 oper_ct_ext;
	__le16 oper_cl_len;
	struct tgst_ge olse_ge[TGST_MAX_ENTRY_NUM];
	struct tgst_ge cfge_ge[TGST_MAX_ENTRY_NUM];
};

int ntmp_tgst_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct tgst_query_data *tgst);
int ntmp_tgst_update_admin_gate_list(struct ntmp_user *user, u32 entry_id,
				     struct tgst_cfge_data *cfge);
int ntmp_tgst_delete_admin_gate_list(struct ntmp_user *user, u32 entry_id);
int ntmp_rpt_query_entry(struct ntmp_user *user, u32 entry_id,
			 struct ntmp_rpt_entry *entry);

#endif
