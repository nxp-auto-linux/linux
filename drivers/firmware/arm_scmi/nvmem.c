// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * SCMI NVMEM Protocol - NXP Vendor Extension
 *
 * Copyright 2023-2024 NXP
 */

#define pr_fmt(fmt) "scmi:nvmem:proto: " fmt

#include <linux/module.h>
#include <linux/nvmem_scmi_protocol.h>

#include "common.h"

enum scmi_nvmem_protocol_cmd {
	SCMI_NVMEM_READ_CELL = 0x3,
	SCMI_NVMEM_WRITE_CELL = 0x4,
};

struct scmi_nvmem_proto_attrs {
	__le32 attrs;
};

struct scmi_nvmem_read_cell_a2p {
	__le32 offset;
	__le32 bytes;
};

struct scmi_nvmem_read_cell_p2a {
	__le32 bytes;
	__le32 value;
};

struct scmi_nvmem_write_cell_a2p {
	__le32 offset;
	__le32 bytes;
	__le32 value;
};

struct scmi_nvmem_write_cell_p2a {
	__le32 bytes;
};

static int nvmem_read_cell(const struct scmi_protocol_handle *ph, u32 offset,
			   u32 bytes, u32 *value)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_nvmem_read_cell_a2p *conf;
	struct scmi_nvmem_read_cell_p2a *return_values;
	u32 bytes_read;

	ret = ph->xops->xfer_get_init(ph, SCMI_NVMEM_READ_CELL, sizeof(*conf),
				      sizeof(*return_values), &t);
	if (ret) {
		dev_err(ph->dev, "Failed to get SCMI_NVMEM_READ_CELL ctx\n");
		return ret;
	}

	return_values = t->rx.buf;
	conf = t->tx.buf;
	conf->offset = cpu_to_le32(offset);
	conf->bytes = cpu_to_le32(bytes);

	ret = ph->xops->do_xfer(ph, t);
	if (ret) {
		dev_err(ph->dev, "Failed to get cell (%u, %u) value\n", offset,
			bytes);
		goto done;
	}

	*value = le32_to_cpu(return_values->value);
	bytes_read = le32_to_cpu(return_values->bytes);

	if (bytes_read != bytes) {
		dev_err(ph->dev, "Invalid number of bytes read: %u. Expected %u\n",
			bytes_read, bytes);
		ret = -EINVAL;
	}

	dev_dbg(ph->dev, "Read cell (%u, %u), value: %u\n", offset, bytes,
		*value);
done:
	ph->xops->xfer_put(ph, t);

	return ret;
}

static int nvmem_write_cell(const struct scmi_protocol_handle *ph, u32 offset,
			    u32 bytes, u32 value)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_nvmem_write_cell_a2p *conf;
	struct scmi_nvmem_write_cell_p2a *return_values;
	u32 bytes_written;

	dev_dbg(ph->dev, "Write cell (%u, %u), value: %u\n", offset, bytes,
		value);

	ret = ph->xops->xfer_get_init(ph, SCMI_NVMEM_WRITE_CELL, sizeof(*conf),
				      sizeof(*return_values), &t);
	if (ret) {
		dev_err(ph->dev, "Failed to get SCMI_NVMEM_WRITE_CELL ctx\n");
		return ret;
	}

	return_values = t->rx.buf;
	conf = t->tx.buf;
	conf->offset = cpu_to_le32(offset);
	conf->bytes = cpu_to_le32(bytes);
	conf->value = cpu_to_le32(value);

	ret = ph->xops->do_xfer(ph, t);
	if (ret) {
		dev_err(ph->dev, "Failed to write cell (%u, %u) value: %u\n",
			offset, bytes, value);
		goto done;
	}

	bytes_written = le32_to_cpu(return_values->bytes);
	if (bytes_written != bytes) {
		dev_err(ph->dev, "Invalid number of bytes written: %u. Expected %u\n",
			bytes_written, bytes);
		ret = -EINVAL;
	}

done:
	ph->xops->xfer_put(ph, t);

	return ret;
}

static const struct scmi_nvmem_proto_ops scmi_nvmem_ops = {
	.nvmem_read_cell = nvmem_read_cell,
	.nvmem_write_cell = nvmem_write_cell,
};

static int scmi_nvmem_proto_attrs_get(const struct scmi_protocol_handle *ph,
				      u32 *num_cells)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_nvmem_proto_attrs *attr;
	u32 attrs;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES, 0, sizeof(*attr),
				    &t);
	if (ret)
		return ret;

	attr = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		attrs = le32_to_cpu(attr->attrs);
		*num_cells = attrs;
	} else {
		dev_err(ph->dev, "Failed to get protocol attributes\n");
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_nvmem_protocol_init(const struct scmi_protocol_handle *ph)
{
	int ret;
	u32 version;
	u32 num_cells = 0;

	ret = ph->xops->version_get(ph, &version);
	if (ret) {
		dev_err(ph->dev, "Failed to get version\n");
		return ret;
	}

	dev_info(ph->dev, "SCMI NVMEM Version %d.%d\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	ret = scmi_nvmem_proto_attrs_get(ph, &num_cells);
	if (ret)
		return ret;

	if (!num_cells) {
		dev_err(ph->dev, "Zero NVMEM cells exposed by SCMI Platform\n");
		return -EINVAL;
	}

	return 0;
}

static int scmi_nvmem_protocol_deinit(const struct scmi_protocol_handle *ph)
{
	return 0;
}

static struct scmi_protocol scmi_nvmem = {
	.id = SCMI_PROTOCOL_ID_NVMEM,
	.owner = THIS_MODULE,
	.instance_init = &scmi_nvmem_protocol_init,
	.instance_deinit = &scmi_nvmem_protocol_deinit,
	.ops = &scmi_nvmem_ops,
};

static int __init register_scmi_nvmem_proto(void)
{
	return scmi_protocol_register(&scmi_nvmem);
}
arch_initcall_sync(register_scmi_nvmem_proto);

static void __exit unregister_scmi_nvmem_proto(void)
{
	scmi_protocol_unregister(&scmi_nvmem);
}
module_exit(unregister_scmi_nvmem_proto);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP SCMI NVMEM Protocol");
MODULE_LICENSE("Dual BSD/GPL");
