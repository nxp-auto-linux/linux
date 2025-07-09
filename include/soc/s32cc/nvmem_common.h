/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023, 2025 NXP
 *
 */

#include <linux/slab.h>
#include <linux/nvmem-consumer.h>

static inline int write_nvmem_cell(struct device *dev, const char *cname,
				   u32 value)
{
	struct nvmem_cell *cell;
	int ret;

	cell = nvmem_cell_get(dev, cname);
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	ret = nvmem_cell_write(cell, &value, sizeof(value));
	nvmem_cell_put(cell);

	if (ret < 0)
		return ret;

	if (ret != sizeof(value))
		return -EINVAL;

	return 0;
}
