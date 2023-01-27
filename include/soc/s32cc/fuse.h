/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 NXP
 *
 */

#ifndef __SOC_S32CC_FUSE_H__
#define __SOC_S32CC_FUSE_H__

#include <soc/s32cc/fuse_defs.h>
#include <soc/s32cc/nvmem_common.h>

static inline
int s32cc_ocotp_nvmem_get_tmu_fuse(struct device *dev,
				   const char *cname,
				   union s32cc_tmu_fuse *tmu_fuse)
{
	char *buf = NULL;

	if (!tmu_fuse)
		return -EINVAL;

	buf = read_nvmem_cell(dev, cname);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	tmu_fuse->R = *(s32 *)buf;

	kfree(buf);
	return 0;
}

#endif /* __SOC_S32CC_FUSE_H__*/
