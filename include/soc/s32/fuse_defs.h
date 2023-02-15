/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 NXP
 *
 */

#ifndef __SOC_S32_FUSE_DEFS_H__
#define __SOC_S32_FUSE_DEFS_H__

union s32_tmu_fuse {
	struct {
		s32 CFG_DAC_TRIM0:5;
		u8 Reserved0:1;
		s32 CFG_DAC_TRIM1:5;
		u32 Reserved1:21;
	} B;
	s32 R;
};

#endif /* __SOC_S32_FUSE_DEFS_H__ */
