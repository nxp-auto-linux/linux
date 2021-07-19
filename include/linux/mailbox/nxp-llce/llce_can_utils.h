/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 NXP
 */
#ifndef LLCE_CAN_UTILS_H
#define LLCE_CAN_UTILS_H

#include <linux/ctype.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <uapi/linux/can.h>

static inline void unpack_word0(u32 word0, bool *rtr, bool *ide,
				u32 *std_id, u32 *ext_id)
{
	if (word0 & LLCE_CAN_MB_IDE) {
		*ide = true;
		*ext_id = (word0 & CAN_EFF_MASK) >> CAN_SFF_ID_BITS;
		*std_id = word0 & CAN_SFF_MASK;
	} else {
		*ide = false;
		*std_id = (word0 & LLCE_CAN_MB_IDSTD_MASK) >>
		    LLCE_CAN_MB_IDSTD_SHIFT;
	}

	*rtr = !!(word0 & LLCE_CAN_MB_RTR);
}

static inline u32 pack_word0(bool rtr, bool ide, u32 id)
{
	u32 word0;

	word0 = (rtr << LLCE_CAN_MB_RTR_SHIFT);
	word0 |= (ide << LLCE_CAN_MB_IDE_SHIFT);

	if (ide)
		word0 |= (id & CAN_EFF_MASK);
	else
		word0 |= (id & CAN_SFF_MASK) <<  LLCE_CAN_MB_IDSTD_SHIFT;

	return word0;
}

static inline void unpack_word1(u32 word1, bool *fdf, u8 *dlc, bool *brs,
				bool *esi)
{
	*dlc = word1 & LLCE_CAN_MB_DLC_MASK;
	*brs = !!(word1 & LLCE_CAN_MB_BRS);
	*esi = !!(word1 & LLCE_CAN_MB_ESI);
	*fdf = !!(word1 & LLCE_CAN_MB_FDF);
}

static inline u32 pack_word1(bool fdf, u8 dlc, bool brs, bool esi)
{
	u32 word1;

	word1 = (brs << LLCE_CAN_MB_BRS_SHIFT);
	word1 |= (fdf << LLCE_CAN_MB_FDF_SHIFT);
	word1 |= (esi << LLCE_CAN_MB_ESI_SHIFT);
	word1 |= dlc;

	return word1;
}

#endif
