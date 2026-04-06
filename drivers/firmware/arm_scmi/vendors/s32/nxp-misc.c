// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * SCMI NXP MISC Protocol - NXP Vendor Extension
 *
 * Copyright 2026 NXP
 */

#define pr_fmt(fmt) "scmi:nxp-misc:proto: " fmt

#include <linux/module.h>
#include <linux/nxp_misc_protocol.h>

#include "../../common.h"

#define	SCMI_PROTOCOL_SUPPORTED_VERSION		0x20000

enum scmi_nxp_misc_protocol_cmd {
	SCMI_NXP_MISC_INIT_DDR_PARTIAL = 0xA,
};

struct scmi_nxp_misc_init_ddr_partial_a2p {
	__le32 addr_low;
	__le32 addr_high;
	__le32 size_low;
	__le32 size_high;
	/*
	 * Instance ID parameter is redundant.
	 * It will be removed in future versions.
	 */
	__le32 instance_id;
} __packed;

static int nxp_misc_init_ddr_partial(const struct scmi_protocol_handle *ph,
			u64 start_addr, u64 size, u32 instance_id)
{
	struct scmi_nxp_misc_init_ddr_partial_a2p *cfg;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_NXP_MISC_INIT_DDR_PARTIAL,
			sizeof(*cfg), 0, &t);
	if (ret)
		return ret;

	cfg = t->tx.buf;
	cfg->addr_low = cpu_to_le32(start_addr & GENMASK(31, 0));
	cfg->addr_high = cpu_to_le32(start_addr >> 32);
	cfg->size_low = cpu_to_le32(size & GENMASK(31, 0));
	cfg->size_high = cpu_to_le32(size >> 32);
	cfg->instance_id = cpu_to_le32(instance_id);

	ret = ph->xops->do_xfer(ph, t);
	if (ret) {
		dev_err(ph->dev, "Failed to init DDR region (0x%llx, 0x%llx): err=%d\n",
			start_addr, size, ret);
		goto done;
	}

	dev_dbg(ph->dev, "DDR region init successful: start_addr=0x%llx, size=0x%llx\n",
		start_addr, size);

done:
	ph->xops->xfer_put(ph, t);
	return ret;
}

static const struct scmi_nxp_misc_proto_ops scmi_nxp_misc_ops = {
	.init_ddr_partial = nxp_misc_init_ddr_partial,
};

static int scmi_nxp_misc_protocol_init(const struct scmi_protocol_handle *ph)
{
	u32 version;
	int ret;

	ret = ph->xops->version_get(ph, &version);
	if (ret) {
		dev_err(ph->dev, "Failed to get SCMI NXP Misc version\n");
		return ret;
	}

	dev_info(ph->dev, "SCMI NXP MISC Version %d.%d\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	/* Check if INIT_DDR_PARTIAL command is supported */
	ret = ph->hops->protocol_msg_check(ph, SCMI_NXP_MISC_INIT_DDR_PARTIAL, NULL);
	if (ret) {
		dev_err(ph->dev, "INIT_DDR_PARTIAL command is not supported!\n");
		return ret;
	}

	return ph->set_priv(ph, NULL, version);
}

static struct scmi_protocol scmi_nxp_misc = {
	.id = SCMI_PROTOCOL_NXP_MISC,
	.owner = THIS_MODULE,
	.instance_init = &scmi_nxp_misc_protocol_init,
	.ops = &scmi_nxp_misc_ops,
	.supported_version = SCMI_PROTOCOL_SUPPORTED_VERSION,
	.vendor_id = "NXP",
	.sub_vendor_id = "S32N79",
};

module_scmi_protocol(scmi_nxp_misc);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP SCMI MISC Protocol");
MODULE_LICENSE("Dual BSD/GPL");
