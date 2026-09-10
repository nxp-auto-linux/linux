// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * SCMI NXP DDR Protocol - NXP Vendor Extension
 *
 * Copyright 2026 NXP
 */

#define pr_fmt(fmt) "scmi:nxp-ddr:proto: " fmt

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/module.h>
#include <linux/nxp_ddr_protocol.h>

#include "../../common.h"

#define SCMI_PROTOCOL_SUPPORTED_VERSION		0x10000

enum scmi_nxp_ddr_protocol_cmd {
	SCMI_NXP_DDR_DRAM_DISCOVERY		= 0x3,
	SCMI_NXP_DDR_PARTIAL_DRAM_INIT		= 0x4,
	SCMI_NXP_DDR_GET_LAST_INIT_REQ_STATUS	= 0x5,
};

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

static int nxp_ddr_partial_init(const struct scmi_protocol_handle *ph,
				u64 start, u64 size)
{
	struct scmi_ddr_partial_init_a2p *cfg;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_NXP_DDR_PARTIAL_DRAM_INIT,
				      sizeof(*cfg), 0, &t);
	if (ret)
		return ret;

	cfg = t->tx.buf;
	cfg->flags = cpu_to_le32(0);
	cfg->start_addr_low = cpu_to_le32(lower_32_bits(start));
	cfg->start_addr_high = cpu_to_le32(upper_32_bits(start));
	cfg->size_low = cpu_to_le32(lower_32_bits(size));
	cfg->size_high = cpu_to_le32(upper_32_bits(size));

	ret = ph->xops->do_xfer(ph, t);

	if (ret)
		dev_err(ph->dev,
			"Failed to init DDR region (0x%llx, 0x%llx): err=%d\n",
			start, size, ret);
	else
		dev_dbg(ph->dev,
			"DDR region init successful: start=0x%llx size=0x%llx\n",
			start, size);

	ph->xops->xfer_put(ph, t);
	return ret;
}

static const struct scmi_nxp_ddr_proto_ops scmi_nxp_ddr_ops = {
	.get_info = nxp_ddr_get_info,
	.discover_regions = nxp_ddr_discover_regions,
	.partial_init = nxp_ddr_partial_init,
};

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
