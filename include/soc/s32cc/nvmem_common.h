/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 NXP
 *
 */

#include <linux/slab.h>
#include <linux/nvmem-consumer.h>

static inline int read_nvmem_cell(struct device *dev, const char *cname,
				  u32 *value)
{
	struct nvmem_cell *cell;
	size_t len = 0;
	char *buf;

	if (!value)
		return -EINVAL;

	cell = nvmem_cell_get(dev, cname);
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	if (len != sizeof(u32)) {
		kfree(buf);
		return -EINVAL;
	}

	*value = *(u32 *)buf;
	kfree(buf);

	return 0;
}

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
