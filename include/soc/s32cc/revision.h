/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 NXP
 *
 */

#ifndef __SOC_S32CC_REVISION_H__
#define __SOC_S32CC_REVISION_H__

#include <linux/nvmem-consumer.h>
#include <soc/s32cc/revision_defs.h>

static inline
int s32cc_siul2_nvmem_get_soc_revision(struct device *dev,
				       const char *cname,
				       struct s32cc_soc_rev *soc_rev)
{
	struct nvmem_cell *cell;
	ssize_t len;
	char *buf;
	int ret = 0;

	if (!soc_rev)
		return -EINVAL;

	cell = nvmem_cell_get(dev, cname);
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	if (len != sizeof(u32)) {
		ret = -EOPNOTSUPP;
		goto out;
	}

	soc_rev->minor = ((*(u32 *)buf) >> S32CC_SOC_REV_MINOR_SHIFT) & 0xFF;
	soc_rev->major = ((*(u32 *)buf) >> S32CC_SOC_REV_MAJOR_SHIFT) & 0xFF;

out:
	kfree(buf);
	return ret;
}
#endif /* __SOC_S32CC_REVISION_H__*/
