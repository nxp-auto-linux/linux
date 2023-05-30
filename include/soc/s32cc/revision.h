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
	char *buf_major = NULL, *buf_minor = NULL;

	if (!soc_rev)
		return -EINVAL;

	buf_major = read_nvmem_cell(dev, "soc_major");
	if (IS_ERR(buf_major))
		return PTR_ERR(buf_major);

	buf_minor = read_nvmem_cell(dev, "soc_minor");
	if (IS_ERR(buf_minor)) {
		kfree(buf_major);
		return PTR_ERR(buf_minor);
	}

	soc_rev->minor = *(u32 *)buf_minor;
	soc_rev->major = *(u32 *)buf_major;

	kfree(buf_major);
	kfree(buf_minor);
	return 0;
}

static inline
int s32cc_nvmem_get_pcie_dev_id(struct device *dev, u32 *pcie_variant_bits)
{
	char *buf;

	buf = read_nvmem_cell(dev, "pcie_dev_id");
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	*pcie_variant_bits = *(u32 *)buf;

	kfree(buf);
	return 0;
}

#endif /* __SOC_S32CC_REVISION_H__*/
