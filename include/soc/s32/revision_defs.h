/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 NXP
 *
 */

#ifndef __SOC_S32_REVISION_DEFS_H__
#define __SOC_S32_REVISION_DEFS_H__

#define S32_SOC_REV_MAJOR_SHIFT		(24)
#define S32_SOC_REV_MINOR_SHIFT		(16)

struct s32_soc_rev {
	u8 major;
	u8 minor;
};

#endif /* __SOC_S32_REVISION_DEFS_H__*/
