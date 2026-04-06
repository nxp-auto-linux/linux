/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * SCMI NXP MISC Protocol - NXP Vendor Extension
 *
 * Copyright 2026 NXP
 */
#ifndef SCMI_NXP_MISC_PROTOCOL_H
#define SCMI_NXP_MISC_PROTOCOL_H

#include <linux/types.h>
#include <linux/scmi_protocol.h>

/*
 * SCMI NXP MISC Protocol
 */
#define SCMI_PROTOCOL_NXP_MISC	0x86U

/**
 * struct scmi_nxp_misc_proto_ops - operations provided by the SCMI MISC Protocol
 *
 * @init_ddr_partial: Initialize a DDR memory region with a given
 * start address and size. The DDR controller ID handling the region must be
 * provided.
 */
struct scmi_nxp_misc_proto_ops {
	int (*init_ddr_partial)(const struct scmi_protocol_handle *ph,
				u64 start_addr, u64 size, u32 instance_id);
};

#endif /* SCMI_NXP_MISC_PROTOCOL_H */
