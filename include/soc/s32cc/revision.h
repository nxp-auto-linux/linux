/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021-2023 NXP
 *
 */

#ifndef __SOC_S32CC_REVISION_H__
#define __SOC_S32CC_REVISION_H__

#include <soc/s32cc/revision_defs.h>
#include <soc/s32cc/nvmem_common.h>

static inline
int s32cc_nvmem_get_soc_revision(struct device *dev,
				 struct s32cc_soc_rev *soc_rev)
{
	u32 major, minor;
	int ret;

	if (!soc_rev)
		return -EINVAL;

	ret = read_nvmem_cell(dev, "soc_major", &major);
	if (ret)
		return ret;

	ret = read_nvmem_cell(dev, "soc_minor", &minor);
	if (ret)
		return ret;

	if (major > U8_MAX || minor > U8_MAX)
		return -EOVERFLOW;

	soc_rev->minor = minor;
	soc_rev->major = major;

	return 0;
}

static inline
int s32cc_nvmem_get_pcie_dev_id(struct device *dev, u32 *pcie_variant_bits)
{
	if (!pcie_variant_bits)
		return -EINVAL;

	return read_nvmem_cell(dev, "pcie_dev_id", pcie_variant_bits);
}

#endif /* __SOC_S32CC_REVISION_H__*/
