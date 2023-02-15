/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021-2023 NXP
 *
 */

#ifndef __SOC_S32_REVISION_H__
#define __SOC_S32_REVISION_H__

#include <soc/s32/revision_defs.h>
#include <soc/s32/nvmem_common.h>

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
