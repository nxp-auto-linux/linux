/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021-2022 NXP
 *
 */

#ifndef __SOC_S32_REVISION_H__
#define __SOC_S32_REVISION_H__

#include <linux/nvmem-consumer.h>
#include <soc/s32/revision_defs.h>

static inline char *read_nvmem_cell(struct device *dev,
				    const char *cname)
{
	struct nvmem_cell *cell;
	ssize_t len;
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

static inline int s32_siul2_nvmem_get_soc_revision(struct device *dev,
						   const char *cname,
						   struct s32_soc_rev *soc_rev)
{
	char *buf = NULL;

	if (!soc_rev)
		return -EINVAL;

	buf = read_nvmem_cell(dev, cname);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	soc_rev->minor = ((*(u32 *)buf) >> S32_SOC_REV_MINOR_SHIFT) & 0xFF;
	soc_rev->major = ((*(u32 *)buf) >> S32_SOC_REV_MAJOR_SHIFT) & 0xFF;

	kfree(buf);
	return 0;
}

static inline int s32_siul2_nvmem_get_pcie_dev_id(struct device *dev,
						  const char *cname, u32 *pcie_variant_bits)
{
	char *buf;

	buf = read_nvmem_cell(dev, cname);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	*pcie_variant_bits = ((*(u32 *)buf)
			>> S32_PCIE_DEV_VARIANT) & 0xF;

	kfree(buf);
	return 0;
}

#endif /* __SOC_S32_REVISION_H__*/
