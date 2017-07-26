/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2020 NXP */

#ifndef __S32CC_THERMAL_H__
#define __S32CC_THERMAL_H__

#define TMU_MR			0x0
#define TMU_MSR			0x8
#define TMU_MTMIR		0xC
#define TMU_TCFGR		0x80
#define TMU_SCFGR		0x84
#define TMU_RITSR(site)	(0x100 + (site) * 0x10)
#define	TMU_RATSR(site)	(0x104 + (site) * 0x10)
#define	TMU_CMCFG		0xF00
#define	TMU_TRCR(n)		(0xF10 + (n) * 0x4)

#define CAL_FUSE		0x98

union TMU_MR_u {
	u32 R;
	struct {
		u32 Reserved0:24;
		u32 ALPF:2;
		u32 Reserved1:3;
		u32 CMD:1;
		u32 ME:2; /* MODE */
	} B;
};

union TMU_MSR_u {
	u32 R;
	struct {
		u32 SITE:3;
		u32 Reserved0:29;
	} B;
};

union TMU_MTMIR_u {
	u32 R;
	struct {
		u32 TMI:4;
		u32 ORH:28;
	} B;
};

union TMU_TCFGR_u {
	u32 R;
	struct {
		u32 CAL_PT:4;
		u32 Reserved0:28;
	} B;
};

union TMU_SCFGR_u {
	u32 R;
	struct {
		u32 SENSOR:9;
		u32 Reserved0:23;
	} B;
};

union TMU_TRCR_u {
	u32 R;
	struct {
		u32 TEMP:9;
		u32 Reserved0:22;
		u32 V:1;
	} B;
};

union TMU_RITSR_u {
	u32 R;
	struct {
		u32 TEMP:9;
		u32 TP5:1;
		u32 Reserved0:21;
		u32 V:1;
	} B;
};

union TMU_RATSR_u {
	u32 R;
	struct {
		u32 TEMP:9;
		u32 Reserved0:22;
		u32 V:1;
	} B;
};

union TMU_CMCFG_u {
	u32 R;
	struct {
		u32 DAC_OFFSET:7;
		u32 Reserved0:1;
		u32 CMET:2;
		u32 DFD:2;
		u32 CLK_DIV:4;
		u32 SAR_RDY:1;
		u32 Reserved1:7;
		u32 RCTC:3;
		u32 Reserved2:1;
		u32 DEMA:1;
		u32 OCS:1;
		u32 OCM:1;
		u32 DPM:1;
	} B;
};

#endif /* __S32CC_THERMAL_H__*/
