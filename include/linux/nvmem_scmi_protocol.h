/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * SCMI NVMEM Protocol - NXP Vendor Extension
 *
 * Copyright 2023 NXP
 */
#ifndef NVMEM_SCMI_PROTOCOL_H
#define NVMEM_SCMI_PROTOCOL_H

#include <linux/types.h>
#include <linux/scmi_protocol.h>

/*
 * SCMI NVMEM Protocol
 */
#define SCMI_PROTOCOL_ID_NVMEM		U32_C(0x82)

/*
 * SCMI NVMEM Protocol version
 */
#define SCMI_PROTOCOL_VERSION_NVMEM	U32_C(0x10000)

/**
 * struct scmi_nvmem_proto_ops - operations provided by the SCMI NVMEM Protocol
 *
 * @nvmem_read_cell: Read a NVMEM cell with a given offset and size, as defined
 * in the DT bindings for the NVMEM provider device.
 * @nvmem_write_cell: Write a NVMEM cell with a given offset and size, as
 * defined in the DT bindings for the NVMEM provider device.
 */
struct scmi_nvmem_proto_ops {
	int (*nvmem_read_cell)(const struct scmi_protocol_handle *ph,
			       u32 offset, u32 bytes, u32 *value);
	int (*nvmem_write_cell)(const struct scmi_protocol_handle *ph,
				u32 offset, u32 bytes, u32 value);
};

#endif /* NVMEM_SCMI_PROTOCOL_H */
