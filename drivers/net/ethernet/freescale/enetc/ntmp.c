// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC NTMP (NETC Table Management Protocol) 2.0 Library
 * Copyright 2025 NXP
 */

#include <linux/dma-mapping.h>
#include <linux/fsl/netc_global.h>
#include <linux/iopoll.h>

#include "ntmp_private.h"

#define NETC_CBDR_TIMEOUT		1000 /* us */
#define NETC_CBDR_MR_EN			BIT(31)

#define NTMP_BASE_ADDR_ALIGN		128
#define NTMP_DATA_ADDR_ALIGN		32

/* Define NTMP Table ID */
#define NTMP_MAFT_ID			1
#define NTMP_RSST_ID			3
#define NTMP_TGST_ID			5
#define NTMP_RPT_ID			10
#define NTMP_ISIT_ID			30
#define NTMP_IST_ID			31
#define NTMP_ISFT_ID			32
#define NTMP_SGIT_ID			36
#define NTMP_SGCLT_ID			37

/* Generic Update Actions for most tables */
#define NTMP_GEN_UA_CFGEU		BIT(0)
#define NTMP_GEN_UA_STSEU		BIT(1)

/* Update Actions for specific tables */
#define RPT_UA_FEEU			BIT(1)
#define RPT_UA_PSEU			BIT(2)
#define RPT_UA_STSEU			BIT(3)
#define SGIT_UA_ACFGEU			BIT(0)
#define SGIT_UA_CFGEU			BIT(1)
#define SGIT_UA_SGISEU			BIT(2)

/* Quary Action: 0: Full query, 1: Only query entry ID */
#define NTMP_QA_ENTRY_ID		1

#define NTMP_ENTRY_ID_SIZE		4
#define RSST_ENTRY_NUM			64
#define RSST_STSE_DATA_SIZE(n)		((n) * 8)
#define RSST_CFGE_DATA_SIZE(n)		(n)
#define SGCLT_MAX_GE_NUM		256

int netc_setup_cbdr(struct device *dev, int cbd_num,
		    struct netc_cbdr_regs *regs,
		    struct netc_cbdr *cbdr)
{
	size_t size;

	size = cbd_num * sizeof(union netc_cbd) + NTMP_BASE_ADDR_ALIGN;
	cbdr->addr_base = dma_alloc_coherent(dev, size, &cbdr->dma_base,
					     GFP_KERNEL);
	if (!cbdr->addr_base)
		return -ENOMEM;

	cbdr->dma_size = size;
	cbdr->bd_num = cbd_num;
	cbdr->regs = *regs;

	/* The base address of the Control BD Ring must be 128 bytes aligned */
	cbdr->dma_base_align =  ALIGN(cbdr->dma_base,  NTMP_BASE_ADDR_ALIGN);
	cbdr->addr_base_align = PTR_ALIGN(cbdr->addr_base,
					  NTMP_BASE_ADDR_ALIGN);

	cbdr->next_to_clean = 0;
	cbdr->next_to_use = 0;
	spin_lock_init(&cbdr->ring_lock);

	/* Step 1: Configure the base address of the Control BD Ring */
	netc_write(cbdr->regs.bar0, lower_32_bits(cbdr->dma_base_align));
	netc_write(cbdr->regs.bar1, upper_32_bits(cbdr->dma_base_align));

	/* Step 2: Configure the producer index register */
	netc_write(cbdr->regs.pir, cbdr->next_to_clean);

	/* Step 3: Configure the consumer index register */
	netc_write(cbdr->regs.cir, cbdr->next_to_use);

	/* Step4: Configure the number of BDs of the Control BD Ring */
	netc_write(cbdr->regs.lenr, cbdr->bd_num);

	/* Step 5: Enable the Control BD Ring */
	netc_write(cbdr->regs.mr, NETC_CBDR_MR_EN);

	return 0;
}
EXPORT_SYMBOL_GPL(netc_setup_cbdr);

void netc_teardown_cbdr(struct device *dev, struct netc_cbdr *cbdr)
{
	/* Disable the Control BD Ring */
	netc_write(cbdr->regs.mr, 0);
	dma_free_coherent(dev, cbdr->dma_size, cbdr->addr_base,
			  cbdr->dma_base);
	memset(cbdr, 0, sizeof(*cbdr));
}
EXPORT_SYMBOL_GPL(netc_teardown_cbdr);

static int netc_get_free_cbd_num(struct netc_cbdr *cbdr)
{
	return (cbdr->next_to_clean - cbdr->next_to_use - 1 +
		cbdr->bd_num) % cbdr->bd_num;
}

static union netc_cbd *netc_get_cbd(struct netc_cbdr *cbdr, int index)
{
	return &((union netc_cbd *)(cbdr->addr_base_align))[index];
}

static void netc_clean_cbdr(struct netc_cbdr *cbdr)
{
	union netc_cbd *cbd;
	int i;

	i = cbdr->next_to_clean;
	while (netc_read(cbdr->regs.cir) != i) {
		cbd = netc_get_cbd(cbdr, i);
		memset(cbd, 0, sizeof(*cbd));
		i = (i + 1) % cbdr->bd_num;
	}

	cbdr->next_to_clean = i;
}

static struct netc_cbdr *netc_select_cbdr(struct netc_cbdrs *cbdrs)
{
	int i;

	for (i = 0; i < cbdrs->cbdr_num; i++) {
		if (spin_is_locked(&cbdrs->ring[i].ring_lock))
			continue;

		return &cbdrs->ring[i];
	}

	return &cbdrs->ring[smp_processor_id() % cbdrs->cbdr_num];
}

static int netc_xmit_ntmp_cmd(struct netc_cbdrs *cbdrs, union netc_cbd *cbd)
{
	union netc_cbd *cur_cbd;
	struct netc_cbdr *cbdr;
	int i, err;
	u16 status;
	u32 val;

	if (cbdrs->cbdr_num == 1)
		cbdr = &cbdrs->ring[0];
	else
		cbdr = netc_select_cbdr(cbdrs);

	spin_lock_bh(&cbdr->ring_lock);

	if (unlikely(!netc_get_free_cbd_num(cbdr)))
		netc_clean_cbdr(cbdr);

	i = cbdr->next_to_use;
	cur_cbd = netc_get_cbd(cbdr, i);
	*cur_cbd = *cbd;

	/* Update producer index of both software and hardware */
	i = (i + 1) % cbdr->bd_num;
	cbdr->next_to_use = i;
	dma_wmb();
	netc_write(cbdr->regs.pir, i);

	err = read_poll_timeout_atomic(netc_read, val, val == i,
				       10, NETC_CBDR_TIMEOUT, true,
				       cbdr->regs.cir);
	if (unlikely(err))
		goto cbdr_unlock;

	dma_rmb();
	/* Get the writeback command BD, because the caller may need
	 * to check some other fields of the response header.
	 */
	*cbd = *cur_cbd;

	/* Check the writeback error status */
	status = le16_to_cpu(cbd->resp_hdr.error_rr) & NTMP_RESP_ERROR;
	if (unlikely(status)) {
		err = -EIO;
		dev_err(cbdrs->dma_dev, "Command BD error: 0x%04x\n", status);
	}

	netc_clean_cbdr(cbdr);
	dma_wmb();

cbdr_unlock:
	spin_unlock_bh(&cbdr->ring_lock);

	return err;
}

static int ntmp_alloc_data_mem(struct ntmp_dma_buf *data, void **buf_align)
{
	void *buf;

	buf = dma_alloc_coherent(data->dev, data->size + NTMP_DATA_ADDR_ALIGN,
				 &data->dma, GFP_ATOMIC);
	if (!buf)
		return -ENOMEM;

	data->buf = buf;
	*buf_align = PTR_ALIGN(buf, NTMP_DATA_ADDR_ALIGN);

	return 0;
}

static void ntmp_free_data_mem(struct ntmp_dma_buf *data)
{
	dma_free_coherent(data->dev, data->size + NTMP_DATA_ADDR_ALIGN,
			  data->buf, data->dma);
}

static void ntmp_fill_request_headr(union netc_cbd *cbd, dma_addr_t dma,
				    int len, int table_id, int cmd,
				    int access_method)
{
	dma_addr_t dma_align;

	memset(cbd, 0, sizeof(*cbd));
	dma_align = ALIGN(dma, NTMP_DATA_ADDR_ALIGN);
	cbd->req_hdr.addr = cpu_to_le64(dma_align);
	cbd->req_hdr.len = cpu_to_le32(len);
	cbd->req_hdr.cmd = cmd;
	cbd->req_hdr.access_method = FIELD_PREP(NTMP_ACCESS_METHOD,
						access_method);
	cbd->req_hdr.table_id = table_id;
	cbd->req_hdr.ver_cci_rr = FIELD_PREP(NTMP_HDR_VERSION,
					     NTMP_HDR_VER2);
	/* For NTMP version 2.0 or later version */
	cbd->req_hdr.npf = cpu_to_le32(NTMP_NPF);
}

static void ntmp_fill_crd(struct common_req_data *crd,
			  u8 tblv, u8 qa, u16 ua)
{
	crd->update_act = cpu_to_le16(ua);
	crd->tblv_qact = NTMP_TBLV_QACT(tblv, qa);
}

static void ntmp_fill_crd_eid(struct ntmp_req_by_eid *rbe, u8 tblv,
			      u8 qa, u16 ua, u32 entry_id)
{
	ntmp_fill_crd(&rbe->crd, tblv, qa, ua);
	rbe->entry_id = cpu_to_le32(entry_id);
}

static int ntmp_delete_entry_by_id(struct netc_cbdrs *cbdrs, int tbl_id,
				   u8 tbl_ver, u32 entry_id, u32 req_len,
				   u32 resp_len)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct ntmp_req_by_eid *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return 0;

	/* If the req_len is 0, indicates the requested length is the
	 * standard length.
	 */
	if (!req_len)
		req_len = sizeof(*req);

	data.size = req_len >= resp_len ? req_len : resp_len;
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, tbl_ver, 0, 0, entry_id);
	len = NTMP_LEN(req_len, resp_len);
	ntmp_fill_request_headr(&cbd, data.dma, len, tbl_id,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(cbdrs->dma_dev, "Delete table (id: %d) entry failed: %d",
			tbl_id, err);

	ntmp_free_data_mem(&data);

	return err;
}

static int ntmp_query_entry_by_id(struct netc_cbdrs *cbdrs, int tbl_id,
				  u32 len, struct ntmp_req_by_eid *req,
				  dma_addr_t dma, bool compare_eid)
{
	struct device *dev = cbdrs->dma_dev;
	struct common_resp_query *resp;
	int cmd = NTMP_CMD_QUERY;
	union netc_cbd cbd;
	u32 entry_id;
	int err;

	entry_id = le32_to_cpu(req->entry_id);
	if (le16_to_cpu(req->crd.update_act))
		cmd = NTMP_CMD_QU;

	/* Request header */
	ntmp_fill_request_headr(&cbd, dma, len, tbl_id,
				cmd, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err) {
		dev_err(dev, "Query table (id: %d) entry failed: %d\n",
			tbl_id, err);
		return err;
	}

	/* For a few tables, the first field of its response data is not
	 * entry_id, so directly return success.
	 */
	if (!compare_eid)
		return 0;

	resp = (struct common_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(dev, "Table (id: %d) query EID:0x%0x, response EID:0x%x\n",
			tbl_id, entry_id, le32_to_cpu(resp->entry_id));
		return -EIO;
	}

	return 0;
}

int ntmp_maft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			struct maft_entry_data *maft)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct maft_req_add *req;
	union netc_cbd cbd;
	int err;

	data.size = sizeof(*req);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Set mac address filter table request data buffer */
	ntmp_fill_crd_eid(&req->rbe, cbdrs->tbl.maft_ver, 0, 0, entry_id);
	req->keye = maft->keye;
	req->cfge = maft->cfge;

	ntmp_fill_request_headr(&cbd, data.dma, NTMP_LEN(data.size, 0),
				NTMP_MAFT_ID, NTMP_CMD_ADD,
				NTMP_AM_ENTRY_ID);
	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(cbdrs->dma_dev, "Add MAFT entry failed (%d)", err);

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_maft_add_entry);

int ntmp_maft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct maft_entry_data *maft)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct maft_resp_query *resp;
	struct ntmp_req_by_eid *req;
	u32 req_len = sizeof(*req);
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data.size = sizeof(*resp);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, cbdrs->tbl.maft_ver, 0, 0, entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_MAFT_ID,
				     NTMP_LEN(req_len, data.size),
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct maft_resp_query *)req;
	maft->keye = resp->keye;
	maft->cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_maft_query_entry);

int ntmp_maft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_MAFT_ID,
				       cbdrs->tbl.maft_ver,
				       entry_id, 0, 0);
}
EXPORT_SYMBOL_GPL(ntmp_maft_delete_entry);

int ntmp_rsst_query_or_update_entry(struct netc_cbdrs *cbdrs, u32 *table,
				    int count, bool query)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct rsst_req_update *requ;
	struct ntmp_req_by_eid *req;
	union netc_cbd cbd;
	int err, i;
	u32 len;

	if (count != RSST_ENTRY_NUM)
		/* HW only takes in a full 64 entry table */
		return -EINVAL;

	if (query)
		data.size = NTMP_ENTRY_ID_SIZE + RSST_STSE_DATA_SIZE(count) +
			    RSST_CFGE_DATA_SIZE(count);
	else
		data.size = struct_size(requ, groups, count);

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Set the request data buffer */
	if (query) {
		ntmp_fill_crd_eid(req, cbdrs->tbl.rsst_ver, 0, 0, 0);
		len = NTMP_LEN(sizeof(*req), data.size);
		ntmp_fill_request_headr(&cbd, data.dma, len, NTMP_RSST_ID,
					NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);
	} else {
		requ = (struct rsst_req_update *)req;
		ntmp_fill_crd_eid(&requ->rbe, cbdrs->tbl.rsst_ver, 0,
				  NTMP_GEN_UA_CFGEU | NTMP_GEN_UA_STSEU, 0);
		for (i = 0; i < count; i++)
			requ->groups[i] = (u8)(table[i]);

		len = NTMP_LEN(data.size, 0);
		ntmp_fill_request_headr(&cbd, data.dma, len, NTMP_RSST_ID,
					NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);
	}

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err) {
		dev_err(cbdrs->dma_dev, "%s RSS table entry failed (%d)",
			query ? "Query" : "Update", err);
		goto end;
	}

	if (query) {
		u8 *group = (u8 *)req;

		group += NTMP_ENTRY_ID_SIZE + RSST_STSE_DATA_SIZE(count);
		for (i = 0; i < count; i++)
			table[i] = group[i];
	}

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rsst_query_or_update_entry);

int ntmp_tgst_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct tgst_query_data *tgst)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct tgst_resp_query *resp;
	struct tgst_cfge_data *cfge;
	struct tgst_olse_data *olse;
	struct ntmp_req_by_eid *req;
	u32 req_len = sizeof(*req);
	int i, err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data.size = sizeof(*resp) + struct_size(cfge, ge, TGST_MAX_ENTRY_NUM) +
		    struct_size(olse, ge, TGST_MAX_ENTRY_NUM);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, cbdrs->tbl.tgst_ver, 0, 0, entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_TGST_ID,
				     NTMP_LEN(req_len, data.size),
				     req, data.dma, false);
	if (err)
		goto end;

	resp = (struct tgst_resp_query *)req;
	cfge = (struct tgst_cfge_data *)resp->data;

	tgst->config_change_time = resp->status.cfg_ct;
	tgst->admin_bt = cfge->admin_bt;
	tgst->admin_ct = cfge->admin_ct;
	tgst->admin_ct_ext = cfge->admin_ct_ext;
	tgst->admin_cl_len = cfge->admin_cl_len;
	for (i = 0; i < le16_to_cpu(cfge->admin_cl_len); i++) {
		tgst->cfge_ge[i].interval = cfge->ge[i].interval;
		tgst->cfge_ge[i].tc_state = cfge->ge[i].tc_state;
		tgst->cfge_ge[i].hr_cb = cfge->ge[i].hr_cb;
	}

	olse = (struct tgst_olse_data *)&cfge->ge[i];
	tgst->oper_cfg_ct = olse->oper_cfg_ct;
	tgst->oper_cfg_ce = olse->oper_cfg_ce;
	tgst->oper_bt = olse->oper_bt;
	tgst->oper_ct = olse->oper_ct;
	tgst->oper_ct_ext = olse->oper_ct_ext;
	tgst->oper_cl_len = olse->oper_cl_len;
	for (i = 0; i < le16_to_cpu(olse->oper_cl_len); i++) {
		tgst->olse_ge[i].interval = olse->ge[i].interval;
		tgst->olse_ge[i].tc_state = olse->ge[i].tc_state;
		tgst->olse_ge[i].hr_cb = olse->ge[i].hr_cb;
	}

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_tgst_delete_admin_gate_list(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct tgst_req_update *req;
	struct tgst_cfge_data *cfge;
	union netc_cbd cbd;
	u32 len;
	int err;

	data.size = sizeof(*req);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	cfge = &req->cfge;
	/* Set the request data buffer and set the admin control list len
	 * to zero to delete the existing admin control list.
	 */
	ntmp_fill_crd_eid(&req->rbe, cbdrs->tbl.tgst_ver, 0,
			  NTMP_GEN_UA_CFGEU, entry_id);
	cfge->admin_cl_len = 0;

	/* Request header */
	len = NTMP_LEN(data.size, sizeof(struct tgst_resp_status));
	ntmp_fill_request_headr(&cbd, data.dma, len, NTMP_TGST_ID,
				NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(cbdrs->dma_dev, "Delete TGST entry failed (%d)", err);

	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_tgst_update_admin_gate_list(struct netc_cbdrs *cbdrs, u32 entry_id,
				     struct tgst_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	u16 list_len = le16_to_cpu(cfge->admin_cl_len);
	u32 cfge_len = struct_size(cfge, ge, list_len);
	struct tgst_req_update *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	/* Calculate the size of request data buffer */
	data.size = struct_size(req, cfge.ge, list_len);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Set the request data buffer */
	ntmp_fill_crd_eid(&req->rbe, cbdrs->tbl.tgst_ver, 0,
			  NTMP_GEN_UA_CFGEU, entry_id);
	memcpy(&req->cfge, cfge, cfge_len);

	/* Request header */
	len = NTMP_LEN(data.size, sizeof(struct tgst_resp_status));
	ntmp_fill_request_headr(&cbd, data.dma, len, NTMP_TGST_ID,
				NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(cbdrs->dma_dev, "Update TGST entry failed (%d)", err);

	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_rpt_add_or_update_entry(struct netc_cbdrs *cbdrs,
				 struct ntmp_rpt_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct rpt_req_ua *req;
	union netc_cbd cbd;
	int err;

	data.size = sizeof(*req);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(&req->rbe, cbdrs->tbl.rpt_ver, 0, NTMP_GEN_UA_CFGEU |
			  RPT_UA_FEEU | RPT_UA_PSEU | RPT_UA_STSEU,
			  entry->entry_id);
	req->cfge = entry->cfge;
	req->fee = entry->fee;

	/* Request header */
	ntmp_fill_request_headr(&cbd, data.dma, NTMP_LEN(data.size, 0),
				NTMP_RPT_ID, NTMP_CMD_AU, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(cbdrs->dma_dev,
			"Add/Update RPT entry failed (%d)", err);

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rpt_add_or_update_entry);

int ntmp_rpt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			 struct ntmp_rpt_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct ntmp_req_by_eid *req;
	struct rpt_resp_query *resp;
	u32 req_len = sizeof(*req);
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data.size = sizeof(*resp);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, cbdrs->tbl.rpt_ver, 0, 0, entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_RPT_ID,
				     NTMP_LEN(req_len, data.size),
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct rpt_resp_query *)req;
	entry->stse = resp->stse;
	entry->cfge = resp->cfge;
	entry->fee = resp->fee;
	entry->pse = resp->pse;

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_rpt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_RPT_ID, cbdrs->tbl.rpt_ver,
				       entry_id, 0, 0);
}
EXPORT_SYMBOL_GPL(ntmp_rpt_delete_entry);

int ntmp_isit_add_or_update_entry(struct netc_cbdrs *cbdrs, bool add,
				  struct ntmp_isit_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct isit_resp_query *resp;
	struct isit_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	data.size = add ? sizeof(*resp) : sizeof(*req);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	if (!add)
		ntmp_fill_crd(&req->crd, cbdrs->tbl.isit_ver, 0,
			      NTMP_GEN_UA_CFGEU);
	else
		/* Query ENTRY_ID only */
		ntmp_fill_crd(&req->crd, cbdrs->tbl.isit_ver,
			      NTMP_GEN_UA_CFGEU, 0);

	req->ak.keye = entry->keye;
	req->is_eid = entry->is_eid;

	/* Request header */
	if (add) {
		len = NTMP_LEN(sizeof(*req), sizeof(*resp));
		/* Must be EXACT MATCH and the command must be
		 * add, followed by a query. So that we can get
		 * the entry id from HW.
		 */
		ntmp_fill_request_headr(&cbd, data.dma, len, NTMP_ISIT_ID,
					NTMP_CMD_AQ, NTMP_AM_EXACT_KEY);
	} else {
		len = NTMP_LEN(sizeof(*req), sizeof(struct common_resp_nq));
		ntmp_fill_request_headr(&cbd, data.dma, len, NTMP_ISIT_ID,
					NTMP_CMD_UPDATE, NTMP_AM_EXACT_KEY);
	}

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err) {
		dev_err(cbdrs->dma_dev, "%s ISIT entry failed (%d)",
			add ? "Add" : "Update", err);
		goto end;
	}

	if (add) {
		resp = (struct isit_resp_query *)req;
		entry->entry_id = le32_to_cpu(resp->entry_id);
	}

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_isit_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct ntmp_isit_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	u32 req_len = sizeof(struct isit_req_qd);
	struct isit_resp_query *resp;
	struct isit_req_qd *req;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data.size = sizeof(*resp);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd(&req->crd, cbdrs->tbl.isit_ver, 0, 0);
	req->ak.eid.entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_ISIT_ID,
				     NTMP_LEN(req_len, data.size),
				     (struct ntmp_req_by_eid *)req,
				     data.dma, false);
	if (err)
		goto end;

	resp = (struct isit_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdrs->dma_dev,
			"ISIT Query EID:0x%0x, Response EID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	entry->keye = resp->keye;
	entry->is_eid = resp->is_eid;

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_isit_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	u32 resp_len = sizeof(struct common_resp_nq);
	u32 req_len = sizeof(struct isit_req_qd);

	return ntmp_delete_entry_by_id(cbdrs, NTMP_ISIT_ID, cbdrs->tbl.isit_ver,
				       entry_id, req_len, resp_len);
}

int ntmp_ist_add_or_update_entry(struct netc_cbdrs *cbdrs,
				 struct ntmp_ist_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct ist_req_ua *req;
	union netc_cbd cbd;
	int err;

	data.size = sizeof(*req);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Fill up NTMP request data buffer */
	ntmp_fill_crd_eid(&req->rbe, cbdrs->tbl.ist_ver, 0,
			  NTMP_GEN_UA_CFGEU, entry->entry_id);
	req->cfge = entry->cfge;

	/* Request header */
	ntmp_fill_request_headr(&cbd, data.dma, NTMP_LEN(data.size, 0),
				NTMP_IST_ID, NTMP_CMD_AU, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(cbdrs->dma_dev,
			"Add/Update IST entry failed (%d)", err);

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ist_add_or_update_entry);

int ntmp_ist_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			 struct ist_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct ist_resp_query *resp;
	struct ntmp_req_by_eid *req;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data.size = sizeof(*resp);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, cbdrs->tbl.ist_ver, 0, 0, entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_IST_ID,
				     NTMP_LEN(sizeof(*req), data.size),
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct ist_resp_query *)req;
	*cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_ist_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_IST_ID, cbdrs->tbl.ist_ver,
				       entry_id, 0, 0);
}
EXPORT_SYMBOL_GPL(ntmp_ist_delete_entry);

int ntmp_isft_add_or_update_entry(struct netc_cbdrs *cbdrs, bool add,
				  struct ntmp_isft_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct isft_resp_query *resp;
	struct isft_req_ua *req;
	union netc_cbd cbd;
	u8 qa = 0;
	u32 len;
	int err;

	data.size = add ? sizeof(*resp) : sizeof(*req);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	if (add)
		qa = NTMP_QA_ENTRY_ID;

	ntmp_fill_crd(&req->crd, cbdrs->tbl.isft_ver, qa, NTMP_GEN_UA_CFGEU);
	req->ak.keye = entry->keye;
	req->cfge = entry->cfge;

	/* Request header */
	if (add) {
		len = NTMP_LEN(sizeof(*req), sizeof(*resp));
		/* Must be exact match, and command must be add,
		 * followed by a query. So that we can get entry
		 * ID from hardware.
		 */
		ntmp_fill_request_headr(&cbd, data.dma, len, NTMP_ISFT_ID,
					NTMP_CMD_AQ, NTMP_AM_EXACT_KEY);
	} else {
		len = NTMP_LEN(sizeof(*req), sizeof(struct common_resp_nq));
		ntmp_fill_request_headr(&cbd, data.dma, len, NTMP_ISFT_ID,
					NTMP_CMD_UPDATE, NTMP_AM_EXACT_KEY);
	}

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err) {
		dev_err(cbdrs->dma_dev, "%s ISFT entry failed (%d)",
			add ? "Add" : "Update", err);
		goto end;
	}

	if (add) {
		resp = (struct isft_resp_query *)req;
		entry->entry_id = le32_to_cpu(resp->entry_id);
	}

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_isft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct ntmp_isft_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct isft_resp_query *resp;
	struct isft_req_qd *req;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data.size = sizeof(*resp);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd(&req->crd, cbdrs->tbl.isft_ver, 0, 0);
	req->ak.eid.entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_ISFT_ID,
				     NTMP_LEN(sizeof(*req), data.size),
				     (struct ntmp_req_by_eid *)req,
				     data.dma, false);
	if (err)
		goto end;

	resp = (struct isft_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdrs->dma_dev,
			"ISFT Query EID:0x%0x, Response EID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	entry->keye = resp->keye;
	entry->cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_isft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	u32 resp_len = sizeof(struct common_resp_nq);
	u32 req_len = sizeof(struct isft_req_qd);

	return ntmp_delete_entry_by_id(cbdrs, NTMP_ISFT_ID, cbdrs->tbl.isft_ver,
				       entry_id, req_len, resp_len);
}

int ntmp_sgit_add_or_update_entry(struct netc_cbdrs *cbdrs,
				  struct ntmp_sgit_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct sgit_req_ua *req;
	union netc_cbd cbd;
	int err;

	data.size = sizeof(*req);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(&req->rbe, cbdrs->tbl.sgit_ver, 0, SGIT_UA_ACFGEU |
			  SGIT_UA_CFGEU | SGIT_UA_SGISEU, entry->entry_id);
	req->acfge = entry->acfge;
	req->cfge = entry->cfge;
	req->icfge = entry->icfge;

	/* Request header */
	ntmp_fill_request_headr(&cbd, data.dma, NTMP_LEN(data.size, 0),
				NTMP_SGIT_ID, NTMP_CMD_AU, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(cbdrs->dma_dev,
			"Add/Update SGIT entry failed (%d)", err);

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgit_add_or_update_entry);

int ntmp_sgit_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct ntmp_sgit_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct sgit_resp_query *resp;
	struct ntmp_req_by_eid *req;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data.size = sizeof(*resp);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, cbdrs->tbl.sgit_ver, 0, 0, entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_SGIT_ID,
				     NTMP_LEN(sizeof(*req), data.size),
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct sgit_resp_query *)req;
	entry->sgise = resp->sgise;
	entry->cfge = resp->cfge;
	entry->icfge = resp->icfge;
	entry->acfge = resp->acfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_sgit_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_SGIT_ID, cbdrs->tbl.sgit_ver,
				       entry_id, 0, 0);
}

int ntmp_sgclt_add_entry(struct netc_cbdrs *cbdrs,
			 struct ntmp_sgclt_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct sgclt_req_add *req;
	u32 num_gates, cfge_len;
	union netc_cbd cbd;
	int err;

	num_gates = entry->cfge.list_length + 1;
	data.size = struct_size(req, cfge.ge, num_gates);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Fill up NTMP request data buffer */
	ntmp_fill_crd_eid(&req->rbe, cbdrs->tbl.sgclt_ver, 0, 0,
			  entry->entry_id);
	cfge_len = struct_size_t(struct sgclt_cfge_data, ge, num_gates);
	memcpy(&req->cfge, &entry->cfge, cfge_len);

	/* Request header */
	ntmp_fill_request_headr(&cbd, data.dma, NTMP_LEN(data.size, 0),
				NTMP_SGCLT_ID, NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(cbdrs->dma_dev, "Add SGCLT entry failed (%d)", err);

	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_sgclt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			   struct ntmp_sgclt_entry *entry, u32 cfge_size)
{
	struct ntmp_dma_buf data = {.dev = cbdrs->dma_dev};
	struct sgclt_resp_query *resp;
	struct ntmp_req_by_eid *req;
	u32 num_gates, cfge_len;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data.size = struct_size(resp, cfge.ge, SGCLT_MAX_GE_NUM);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, cbdrs->tbl.sgclt_ver, 0, 0, entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_SGCLT_ID,
				     NTMP_LEN(sizeof(*req), data.size),
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct sgclt_resp_query *)req;
	entry->ref_count = resp->ref_count;
	num_gates = resp->cfge.list_length + 1;
	cfge_len = struct_size_t(struct sgclt_cfge_data, ge, num_gates);
	if (cfge_len > cfge_size) {
		err = -ENOMEM;
		dev_err(cbdrs->dma_dev,
			"SGCLT_CFGE buffer size is %u, larger than %u\n",
			cfge_size, cfge_len);

		goto end;
	}

	memcpy(&entry->cfge, &resp->cfge, cfge_len);

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_sgclt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_SGCLT_ID, cbdrs->tbl.sgclt_ver,
				       entry_id, 0, 0);
}

MODULE_DESCRIPTION("NXP NETC Library");
MODULE_LICENSE("Dual BSD/GPL");
