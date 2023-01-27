/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 NXP
 *
 */

#include <linux/nvmem-consumer.h>

static inline char *read_nvmem_cell(struct device *dev,
				    const char *cname)
{
	struct nvmem_cell *cell;
	size_t len = 0;
	char *buf;

	cell = nvmem_cell_get(dev, cname);
	if (IS_ERR(cell))
		return ERR_PTR(-EINVAL);

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (IS_ERR(buf))
		return buf;

	if (len != sizeof(u32)) {
		kfree(buf);
		return ERR_PTR(-EOPNOTSUPP);
	}

	return buf;
}
