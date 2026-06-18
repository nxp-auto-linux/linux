/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * NTMP table request and response data buffer formats
 * Copyright 2025 NXP
 */

#ifndef __NTMP_PRIVATE_H
#define __NTMP_PRIVATE_H

#include <linux/fsl/ntmp.h>

struct ntmp_dma_buf {
	struct device *dev;
	size_t size;
	void *buf;
	dma_addr_t dma;
};

struct common_req_data {
	__le16 update_act;
	u8 dbg_opt;
	u8 tblv_qact;
#define NTMP_QUERY_ACT		GENMASK(3, 0)
#define NTMP_TBL_VER		GENMASK(7, 0)
#define NTMP_TBLV_QACT(v, a)	(FIELD_PREP(NTMP_TBL_VER, (v)) | \
				 ((a) & NTMP_QUERY_ACT))
};

struct common_resp_query {
	__le32 entry_id;
};

struct common_resp_nq {
	__le32 status;
};

/* Generic structure for request data by entry ID  */
struct ntmp_req_by_eid {
	struct common_req_data crd;
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

#endif
