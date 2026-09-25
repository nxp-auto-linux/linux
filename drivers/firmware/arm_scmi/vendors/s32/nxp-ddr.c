// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * SCMI NXP DDR Protocol - NXP Vendor Extension
 *
 * Copyright 2026 NXP
 */

#define pr_fmt(fmt) "scmi:nxp-ddr:proto: " fmt

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/nxp_ddr_protocol.h>
#include <linux/time.h>

#include "../../common.h"

#define SCMI_PROTOCOL_SUPPORTED_VERSION		0x10000

enum scmi_nxp_ddr_protocol_cmd {
	SCMI_NXP_DDR_DRAM_DISCOVERY		= 0x3,
	SCMI_NXP_DDR_PARTIAL_DRAM_INIT		= 0x4,
	SCMI_NXP_DDR_GET_LAST_INIT_REQ_STATUS	= 0x5,
};

enum nxp_ddr_req_state {
	NXP_DDR_REQ_QUEUED,
	NXP_DDR_REQ_DONE,
	NXP_DDR_REQ_FAILED,
};

/* Flags field of DDR_PARTIAL_DRAM_INIT: bit 0 requests asynchronous init. */
#define SCMI_NXP_DDR_INIT_FLAG_ASYNC		BIT(0)

/*
 * IN_USE means the per-instance init queue is full and the request should be
 * retried. It is SCMI status -11, past the end of the core errno table, so the
 * core collapses it to -EIO, the same as GENERIC_ERROR (-8); the raw status in
 * the header separates the two.
 */
#define SCMI_NXP_DDR_STATUS_IN_USE		(-11)

/*
 * PROTOCOL_ATTRIBUTES carries the per-instance init queue depth in bits[7:0]
 * (MaxJobQueueNum). A non-zero depth is the indication that the platform can
 * queue an init request, which the asynchronous form relies on.
 */
#define SCMI_NXP_DDR_ATTR_QUEUE_DEPTH		GENMASK(7, 0)

/* NumberRegion field of DDR_DRAM_DISCOVERY. */
#define SCMI_NXP_DDR_DISC_NUM_RETURNED		GENMASK(7, 0)
#define SCMI_NXP_DDR_DISC_NUM_REMAINING		GENMASK(15, 8)

/* The low bits of a region's instance word select its owning DDR instance. */
#define SCMI_NXP_DDR_DISC_INSTANCE_MASK		GENMASK(1, 0)

/*
 * The spec allows up to 10 regions per DDR_DRAM_DISCOVERY reply, but such a
 * reply does not fit the transport used on S32N. The SCMI shared memory window
 * is 128 bytes (see 'scmi_tx_buf' in the S32N79 device tree) and the leading
 * struct scmi_shared_mem fields consume 28 of them, leaving 100 bytes of
 * payload. A reply is 8 bytes of leading fields plus a 20-byte record per
 * region, so only 4 regions fit (8 + 4 * 20 = 88; 5 regions would already
 * need 108). Bound the per-message region count so the reply always fits,
 * validate it against the actual received length, and paginate for the rest.
 */
#define SCMI_NXP_DDR_MAX_REGIONS_PER_MSG	4

/* Upper bound on discovery iterations to defend against a buggy platform. */
#define SCMI_NXP_DDR_MAX_DISC_ITERATIONS	64

struct scmi_ddr_discovery_a2p {
	__le32 region_index;
} __packed;

struct scmi_ddr_region_entry {
	__le32 start_lo;
	__le32 start_hi;
	__le32 end_lo;
	__le32 end_hi;
	__le32 instance_id;
} __packed;

/*
 * DDR_DRAM_DISCOVERY reply layout as emitted by the FSS firmware:
 *
 *   ddr_cfg_info		(1 word)
 *   number_region		(1 word: [7:0] returned, [15:8] remaining)
 *   region[n]			(n region records, 5 words / 20 bytes each)
 *
 * Each region record carries the start and end addresses as lo/hi 32-bit
 * halves plus an instance word whose low bits give the owning DDR (SMMS)
 * instance. The instance word is interleaved after every region rather than
 * gathered into a single trailing word, so a record is 5 words (20 bytes)
 * wide.
 */
struct scmi_ddr_discovery_p2a {
	/* Per-instance ECC enable bitmap: bit n set = ECC on for instance n. */
	__le32 ddr_cfg_info;
	__le32 number_region;
	struct scmi_ddr_region_entry region[];
} __packed;

struct scmi_ddr_partial_init_a2p {
	__le32 flags;
	__le32 start_addr_low;
	__le32 start_addr_high;
	__le32 size_low;
	__le32 size_high;
} __packed;

/* Range of a DDR_PARTIAL_DRAM_INIT request, as echoed back by the platform. */
struct scmi_ddr_req_params {
	__le32 start_lo;
	__le32 start_hi;
	__le32 size_lo;
	__le32 size_hi;
} __packed;

/*
 * Payload of the DDR_PARTIAL_DRAM_INIT delayed response. The SCMI core strips
 * the leading status word before it reaches t->rx.buf, so status is not part
 * of this struct. The echoed range lets the agent confirm the response belongs
 * to the request it is waiting on, in addition to the header token the core
 * already matches.
 */
struct scmi_ddr_partial_init_cmpl_p2a {
	struct scmi_ddr_req_params req;
} __packed;

/*
 * DDR_GET_LAST_INIT_REQUEST_STATUS reply. Per SMMS, the platform reports
 * the oldest of this OS's queued requests, or the last one completed.
 */
struct scmi_ddr_inst_status {
	__le32 hdr;	/* [7:0] requests still queued, [15:8] status */
	struct scmi_ddr_req_params req;
} __packed;

struct scmi_ddr_last_status_p2a {
	__le32 num_instances;
	struct scmi_ddr_inst_status inst[];
} __packed;

#define SCMI_NXP_DDR_LAST_PENDING		GENMASK(7, 0)
#define SCMI_NXP_DDR_LAST_STATUS		GENMASK(15, 8)
#define SCMI_NXP_DDR_LAST_STATUS_SUCCESS	0
#define SCMI_NXP_DDR_LAST_STATUS_IN_PROGRESS	2

/*
 * Wait for a request whose delayed response is late. The poll interval matches
 * the consumer's back-off on a busy platform. The timeout is a generous bound
 * on the time a request can spend queued, including behind requests from
 * other agents on the same SMMS. Past it the chunk is given up: it stays
 * unused and is never hot-added.
 */
#define SCMI_NXP_DDR_WAIT_POLL_US		(100 * USEC_PER_MSEC)
#define SCMI_NXP_DDR_WAIT_TIMEOUT_US		(30 * USEC_PER_SEC)

struct scmi_nxp_ddr_priv {
	struct scmi_nxp_ddr_info info;
};

static int nxp_ddr_get_info(const struct scmi_protocol_handle *ph,
			    struct scmi_nxp_ddr_info *info)
{
	struct scmi_nxp_ddr_priv *priv = ph->get_priv(ph);

	*info = priv->info;

	return 0;
}

/*
 * Decode a single discovery reply into @out, returning the number of regions
 * written and reporting how many regions remain via @remaining.
 *
 * The reply's first word, named DdrCfgInfo in the protocol specification, is a
 * per-instance ECC enable bitmap: bit n is set when ECC is enabled on DDR
 * instance n. It is returned via @ecc_mask. DRAM on an instance whose ECC is
 * disabled is usable without a partial-init call.
 */
static int nxp_ddr_decode_discovery(const struct scmi_protocol_handle *ph,
				    struct scmi_xfer *t, u32 *ecc_mask,
				    struct scmi_ddr_region *out, size_t max_out,
				    u8 *remaining)
{
	struct scmi_ddr_discovery_p2a *resp = t->rx.buf;
	const struct scmi_ddr_region_entry *entry;
	u32 number_region;
	unsigned int i;
	u64 start, end;
	u8 returned;

	if (t->rx.len < sizeof(*resp))
		return -EPROTO;

	*ecc_mask = le32_to_cpu(resp->ddr_cfg_info);

	number_region = le32_to_cpu(resp->number_region);
	returned = FIELD_GET(SCMI_NXP_DDR_DISC_NUM_RETURNED, number_region);
	*remaining = FIELD_GET(SCMI_NXP_DDR_DISC_NUM_REMAINING, number_region);

	/*
	 * Reject a reply that claims more regions than either the transport
	 * payload can hold or the received length actually carries. This is
	 * the primary guard against reading past the message buffer.
	 */
	if (returned > SCMI_NXP_DDR_MAX_REGIONS_PER_MSG)
		return -EPROTO;
	if (t->rx.len < struct_size(resp, region, returned))
		return -EPROTO;

	if (returned > max_out)
		returned = max_out;

	for (i = 0; i < returned; i++) {
		entry = &resp->region[i];
		start = le32_to_cpu(entry->start_lo) |
			((u64)le32_to_cpu(entry->start_hi) << 32);
		end = le32_to_cpu(entry->end_lo) |
		      ((u64)le32_to_cpu(entry->end_hi) << 32);

		if (end < start) {
			dev_err(ph->dev,
				"Discovery region %u has end 0x%llx < start 0x%llx\n",
				i, end, start);
			return -EPROTO;
		}

		out[i].start = start;
		out[i].size = end - start + 1;
		out[i].instance = le32_to_cpu(entry->instance_id) &
				  SCMI_NXP_DDR_DISC_INSTANCE_MASK;
	}

	return returned;
}

static int nxp_ddr_discover_regions(const struct scmi_protocol_handle *ph,
				    struct scmi_ddr_region *out, size_t max_out)
{
	struct scmi_nxp_ddr_priv *priv = ph->get_priv(ph);
	struct scmi_ddr_discovery_a2p *cfg;
	unsigned int iter = 0;
	u32 region_index = 0;
	struct scmi_xfer *t;
	size_t total = 0;
	int ret = 0;

	while (total < max_out) {
		u8 remaining;
		int decoded;

		if (++iter > SCMI_NXP_DDR_MAX_DISC_ITERATIONS) {
			dev_err(ph->dev, "DDR discovery did not converge\n");
			return -EPROTO;
		}

		ret = ph->xops->xfer_get_init(ph, SCMI_NXP_DDR_DRAM_DISCOVERY,
					      sizeof(*cfg), 0, &t);
		if (ret)
			return ret;

		cfg = t->tx.buf;
		cfg->region_index = cpu_to_le32(region_index);

		ret = ph->xops->do_xfer(ph, t);
		if (!ret)
			decoded = nxp_ddr_decode_discovery(ph, t,
							   &priv->info.ecc_mask,
							   out + total,
							   max_out - total,
							   &remaining);
		ph->xops->xfer_put(ph, t);

		if (ret)
			return ret;
		if (decoded < 0)
			return decoded;

		total += decoded;
		region_index += decoded;

		if (!remaining || !decoded)
			break;
	}

	if (total == max_out && ret == 0)
		dev_warn(ph->dev,
			 "DDR discovery stopped at %zu regions, some may be missing\n",
			 total);

	return total;
}

/* hdr.status is unsigned; cast to compare against negative SCMI codes. */
static int nxp_ddr_scmi_status(const struct scmi_xfer *t)
{
	return (s32)t->hdr.status;
}

/*
 * A queue-full failure is transient and worth retrying: SCMI_BUSY (async pool
 * exhausted, already -EBUSY) or IN_USE (per-instance queue full, delivered as
 * -EIO). Any other status is a failure that a retry cannot clear.
 */
static bool nxp_ddr_init_is_retryable(const struct scmi_xfer *t, int ret)
{
	if (ret == -EBUSY)
		return true;

	if (ret == -EIO && nxp_ddr_scmi_status(t) == SCMI_NXP_DDR_STATUS_IN_USE)
		return true;

	return false;
}

static void nxp_ddr_get_req_params(const struct scmi_ddr_req_params *p,
				   u64 *start, u64 *size)
{
	*start = le32_to_cpu(p->start_lo) |
		 ((u64)le32_to_cpu(p->start_hi) << 32);
	*size = le32_to_cpu(p->size_lo) |
		((u64)le32_to_cpu(p->size_hi) << 32);
}

/*
 * Confirm a delayed response belongs to the request being waited on. The core
 * already matches the header token; the echoed range is a second check against
 * a platform that reuses a token or answers out of order. Returns 0 on a match,
 * -EPROTO otherwise.
 */
static int nxp_ddr_check_completion(const struct scmi_protocol_handle *ph,
				    struct scmi_xfer *t, u64 start, u64 size)
{
	const struct scmi_ddr_partial_init_cmpl_p2a *cmpl = t->rx.buf;
	u64 got_start, got_size;

	if (t->rx.len < sizeof(*cmpl)) {
		dev_err(ph->dev,
			"DDR init completion too short: %zu bytes, want %zu\n",
			t->rx.len, sizeof(*cmpl));
		return -EPROTO;
	}

	nxp_ddr_get_req_params(&cmpl->req, &got_start, &got_size);

	if (got_start != start || got_size != size) {
		dev_err(ph->dev,
			"DDR init completion mismatch: got (0x%llx, 0x%llx), want (0x%llx, 0x%llx)\n",
			got_start, got_size, start, size);
		return -EPROTO;
	}

	return 0;
}

/*
 * Look up the state of the request for [start, start + size) in
 * DDR_GET_LAST_INIT_REQUEST_STATUS and store it in @state. Returns 0 on
 * success, -EPROTO if the reply is malformed, or the error of the query itself.
 * The platform has no transient status for this query, so a failed query is
 * not worth repeating.
 *
 * A request stays queued until it is done. Between leaving the queue and being
 * recorded as the last one completed, the platform still reports the previous
 * request, so a range that does not match is treated as still queued.
 */
static int nxp_ddr_get_req_state(const struct scmi_protocol_handle *ph,
				 u64 start, u64 size,
				 enum nxp_ddr_req_state *state)
{
	const struct scmi_ddr_last_status_p2a *resp;
	const struct scmi_ddr_inst_status *e;
	u32 i, count, hdr, status;
	u64 got_start, got_size;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_NXP_DDR_GET_LAST_INIT_REQ_STATUS,
				      0, 0, &t);
	if (ret)
		return ret;

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		goto out;

	resp = t->rx.buf;
	count = t->rx.len < sizeof(*resp) ? 0 : le32_to_cpu(resp->num_instances);
	if (count > SCMI_NXP_DDR_MAX_INSTANCES ||
	    t->rx.len < struct_size(resp, inst, count)) {
		ret = -EPROTO;
		goto out;
	}

	/* Still queued, unless the request is reported as done below. */
	*state = NXP_DDR_REQ_QUEUED;
	for (i = 0; i < count; i++) {
		e = &resp->inst[i];
		nxp_ddr_get_req_params(&e->req, &got_start, &got_size);
		if (got_start != start || got_size != size)
			continue;

		hdr = le32_to_cpu(e->hdr);
		status = FIELD_GET(SCMI_NXP_DDR_LAST_STATUS, hdr);
		if (!FIELD_GET(SCMI_NXP_DDR_LAST_PENDING, hdr) &&
		    status != SCMI_NXP_DDR_LAST_STATUS_IN_PROGRESS)
			*state = status == SCMI_NXP_DDR_LAST_STATUS_SUCCESS ?
				 NXP_DDR_REQ_DONE : NXP_DDR_REQ_FAILED;
		break;
	}

out:
	ph->xops->xfer_put(ph, t);
	return ret;
}

/* Poll the request's state until it is done or the wait times out. */
static int nxp_ddr_poll_req_done(const struct scmi_protocol_handle *ph,
				 u64 start, u64 size)
{
	enum nxp_ddr_req_state state = NXP_DDR_REQ_QUEUED;
	int ret, err;

	err = read_poll_timeout(nxp_ddr_get_req_state, ret,
				ret || state != NXP_DDR_REQ_QUEUED,
				SCMI_NXP_DDR_WAIT_POLL_US,
				SCMI_NXP_DDR_WAIT_TIMEOUT_US, false,
				ph, start, size, &state);
	if (err)
		return err;
	if (ret)
		return ret;

	return state == NXP_DDR_REQ_DONE ? 0 : -EIO;
}

static int nxp_ddr_partial_init(const struct scmi_protocol_handle *ph,
				u64 start, u64 size, bool async)
{
	struct scmi_nxp_ddr_priv *priv = ph->get_priv(ph);
	struct scmi_ddr_partial_init_a2p *cfg;
	bool timed_out = false;
	struct scmi_xfer *t;
	int ret;

	if (async && !priv->info.async_supported)
		async = false;

	ret = ph->xops->xfer_get_init(ph, SCMI_NXP_DDR_PARTIAL_DRAM_INIT,
				      sizeof(*cfg), 0, &t);
	if (ret)
		return ret;

	cfg = t->tx.buf;
	cfg->flags = cpu_to_le32(async ? SCMI_NXP_DDR_INIT_FLAG_ASYNC : 0);
	cfg->start_addr_low = cpu_to_le32(lower_32_bits(start));
	cfg->start_addr_high = cpu_to_le32(upper_32_bits(start));
	cfg->size_low = cpu_to_le32(lower_32_bits(size));
	cfg->size_high = cpu_to_le32(upper_32_bits(size));

	if (!async) {
		ret = ph->xops->do_xfer(ph, t);
		goto out;
	}

	ret = ph->xops->do_xfer_with_response(ph, t);
	if (ret == -ETIMEDOUT) {
		timed_out = true;
		goto out;
	}

	if (ret)
		goto out;

	ret = nxp_ddr_check_completion(ph, t, start, size);

out:
	if (timed_out) {
		dev_dbg(ph->dev,
			"DDR region (0x%llx, 0x%llx) has no delayed response yet, polling its status\n",
			start, size);
	} else if (ret && nxp_ddr_init_is_retryable(t, ret)) {
		/* Normalise to the code the consumer backs off and retries on. */
		dev_dbg(ph->dev,
			"DDR region (0x%llx, 0x%llx) busy, retrying: scmi_status=%d\n",
			start, size, nxp_ddr_scmi_status(t));
		ret = -EBUSY;
	} else if (ret) {
		dev_err(ph->dev,
			"Failed to init DDR region (0x%llx, 0x%llx): err=%d scmi_status=%d\n",
			start, size, ret, nxp_ddr_scmi_status(t));
	} else {
		dev_dbg(ph->dev,
			"DDR region init successful: start=0x%llx size=0x%llx\n",
			start, size);
	}

	ph->xops->xfer_put(ph, t);

	/*
	 * A missing delayed response is only late: requests from other agents
	 * may be served first on the same SMMS. The request is still queued,
	 * so ask the platform for its state instead of sending it again. The
	 * SCMI core still logs the timeout, and logs the delayed response as
	 * unexpected if it arrives later; both are expected here.
	 */
	if (timed_out) {
		ret = nxp_ddr_poll_req_done(ph, start, size);
		if (ret)
			dev_err(ph->dev,
				"Failed to init DDR region (0x%llx, 0x%llx) after polling its status: err=%d\n",
				start, size, ret);
		else
			dev_dbg(ph->dev,
				"DDR region init successful: start=0x%llx size=0x%llx\n",
				start, size);
	}

	return ret;
}

static const struct scmi_nxp_ddr_proto_ops scmi_nxp_ddr_ops = {
	.get_info = nxp_ddr_get_info,
	.discover_regions = nxp_ddr_discover_regions,
	.partial_init = nxp_ddr_partial_init,
};

/*
 * Read PROTOCOL_ATTRIBUTES and record the per-instance init queue depth from
 * bits[7:0]. A zero depth means the platform cannot queue an init request, so
 * the asynchronous form is not usable.
 */
static int nxp_ddr_attributes_get(const struct scmi_protocol_handle *ph,
				  struct scmi_nxp_ddr_priv *priv)
{
	struct scmi_xfer *t;
	__le32 *attr;
	int ret;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES, 0,
				      sizeof(*attr), &t);
	if (ret)
		return ret;

	attr = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret)
		priv->info.max_queue_depth =
			FIELD_GET(SCMI_NXP_DDR_ATTR_QUEUE_DEPTH,
				  le32_to_cpu(*attr));

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_nxp_ddr_protocol_init(const struct scmi_protocol_handle *ph)
{
	struct scmi_nxp_ddr_priv *priv;
	u32 version;
	int ret;

	ret = ph->xops->version_get(ph, &version);
	if (ret) {
		dev_err(ph->dev, "Failed to get SCMI NXP DDR version\n");
		return ret;
	}

	dev_info(ph->dev, "SCMI NXP DDR Version %d.%d\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	if (PROTOCOL_REV_MAJOR(version) != 0x1) {
		dev_err(ph->dev, "Unsupported NXP DDR protocol major version\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(ph->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->info.version = version;

	/* DRAM discovery is mandatory; partial init is implied by it. */
	ret = ph->hops->protocol_msg_check(ph, SCMI_NXP_DDR_DRAM_DISCOVERY,
					   NULL);
	if (ret) {
		dev_err(ph->dev, "DDR_DRAM_DISCOVERY command is not supported\n");
		return ret;
	}

	ret = ph->hops->protocol_msg_check(ph, SCMI_NXP_DDR_PARTIAL_DRAM_INIT,
					   NULL);
	if (ret) {
		dev_err(ph->dev, "DDR_PARTIAL_DRAM_INIT command is not supported\n");
		return ret;
	}

	ret = ph->hops->protocol_msg_check(ph,
					   SCMI_NXP_DDR_GET_LAST_INIT_REQ_STATUS,
					   NULL);
	if (ret) {
		dev_err(ph->dev, "DDR_GET_LAST_INIT_REQUEST_STATUS command is not supported\n");
		return ret;
	}

	ret = nxp_ddr_attributes_get(ph, priv);
	if (ret) {
		dev_err(ph->dev, "Failed to get NXP DDR protocol attributes\n");
		return ret;
	}

	/*
	 * PROTOCOL_MESSAGE_ATTRIBUTES cannot express whether the asynchronous
	 * form of DDR_PARTIAL_DRAM_INIT is available, so treat a non-zero queue
	 * depth as the indication. The protocol version is not a reliable
	 * signal, since the platform reports the same version whether or not
	 * async is enabled in its configuration.
	 */
	priv->info.async_supported = priv->info.max_queue_depth > 0;

	return ph->set_priv(ph, priv, version);
}

static struct scmi_protocol scmi_nxp_ddr = {
	.id = SCMI_PROTOCOL_NXP_DDR,
	.owner = THIS_MODULE,
	.instance_init = &scmi_nxp_ddr_protocol_init,
	.ops = &scmi_nxp_ddr_ops,
	.supported_version = SCMI_PROTOCOL_SUPPORTED_VERSION,
	.vendor_id = "NXP",
	.sub_vendor_id = "S32N79",
};

module_scmi_protocol(scmi_nxp_ddr);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP SCMI DDR Protocol");
MODULE_LICENSE("Dual BSD/GPL");
