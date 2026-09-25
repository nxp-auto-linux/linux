/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * SCMI NXP DDR Protocol - NXP Vendor Extension
 *
 * Copyright 2026 NXP
 */
#ifndef SCMI_NXP_DDR_PROTOCOL_H
#define SCMI_NXP_DDR_PROTOCOL_H

#include <linux/types.h>
#include <linux/scmi_protocol.h>

/*
 * SCMI NXP DDR Protocol
 *
 * Dedicated vendor protocol (v1.0) that supersedes the DDR partial
 * initialization command previously carried by the NXP Misc protocol
 * (0x86). It adds runtime discovery of the uninitialized DDR regions and
 * their ECC configuration, so the agent no longer has to hardcode the
 * physical memory map or derive the DDR controller instance itself.
 */
#define SCMI_PROTOCOL_NXP_DDR	0x8BU

/* Number of DDR controller (SMMS) instances on S32N7-based platforms. */
#define SCMI_NXP_DDR_MAX_INSTANCES	2

/*
 * Upper bound on the number of DDR regions the driver tracks. This is a region
 * count, not a controller count: the platform reports one record per region,
 * each carrying its own instance id, and several regions may belong to the
 * same DDR instance.
 */
#define SCMI_NXP_DDR_MAX_REGIONS	32

/**
 * struct scmi_ddr_region - A DDR region reported by DDR_DRAM_DISCOVERY
 * @start: Physical start address of the region.
 * @size: Size of the region in bytes.
 * @instance: DDR controller (SMMS) instance owning the region.
 */
struct scmi_ddr_region {
	u64 start;
	u64 size;
	u8  instance;
};

/**
 * struct scmi_nxp_ddr_info - Protocol capabilities negotiated at init
 * @version: Protocol version reported by the platform.
 * @ecc_mask: DdrCfgInfo bitmap; bit @n set means ECC is enabled on DDR
 *	      instance @n and the region therefore requires initialization.
 * @max_queue_depth: Maximum queued init requests per DDR instance, taken from
 *		     PROTOCOL_ATTRIBUTES bits[7:0].
 * @async_supported: Set when the platform is expected to implement the
 *		     asynchronous form of DDR_PARTIAL_DRAM_INIT and deliver a
 *		     delayed response.
 */
struct scmi_nxp_ddr_info {
	u32  version;
	u32  ecc_mask;
	u32  max_queue_depth;
	bool async_supported;
};

/**
 * struct scmi_nxp_ddr_proto_ops - operations provided by the NXP DDR protocol
 *
 * @get_info: Retrieve the negotiated protocol capabilities.
 * @discover_regions: Retrieve the list of uninitialized, write-permitted DDR
 *		      regions. Returns the number of regions written into @out
 *		      (bounded by @max_out), or a negative error code.
 * @partial_init: Initialize a DDR region of @size bytes at @start. When @async
 *		  is false the call blocks until the platform completes the
 *		  initialization. When @async is true the platform runs the
 *		  work off the request path and reports completion through a
 *		  delayed response, or through DDR_GET_LAST_INIT_REQUEST_STATUS
 *		  if that response is late; the call returns only after one of
 *		  them, so the DDR region is initialized on a success return
 *		  either way. If the platform does not support async, the
 *		  request degrades to the synchronous form.
 */
struct scmi_nxp_ddr_proto_ops {
	int (*get_info)(const struct scmi_protocol_handle *ph,
			struct scmi_nxp_ddr_info *info);
	int (*discover_regions)(const struct scmi_protocol_handle *ph,
				struct scmi_ddr_region *out, size_t max_out);
	int (*partial_init)(const struct scmi_protocol_handle *ph,
			    u64 start, u64 size, bool async);
};

#endif /* SCMI_NXP_DDR_PROTOCOL_H */
