/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2020 NXP */

#ifndef __S32GEN1_THERMAL_H__
#define __s32GEN1_THERMAL_H__


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
	uint32_t R;
	struct {
		uint32_t Reserved0:24;
		uint32_t ALPF:2;
		uint32_t Reserved1:3;
		uint32_t CMD:1;
		uint32_t ME:2; /* MODE */
	} B;
};

union TMU_MSR_u {
	uint32_t R;
	struct {
		uint32_t SITE:3;
		uint32_t Reserved0:29;
	} B;
};

union TMU_MTMIR_u {
	uint32_t R;
	struct {
		uint32_t TMI:4;
		uint32_t ORH:28;
	} B;
};

union TMU_TCFGR_u {
	uint32_t R;
	struct {
		uint32_t CAL_PT:4;
		uint32_t Reserved0:28;
	} B;
};

union TMU_SCFGR_u {
	uint32_t R;
	struct {
		uint32_t SENSOR:9;
		uint32_t Reserved0:23;
	} B;
};

union TMU_TRCR_u {
	uint32_t R;
	struct {
		uint32_t TEMP:9;
		uint32_t Reserved0:22;
		uint32_t V:1;
	} B;
};

union TMU_RITSR_u {
	uint32_t R;
	struct {
		uint32_t TEMP:9;
		uint32_t TP5:1;
		uint32_t Reserved0:21;
		uint32_t V:1;
	} B;
};

union TMU_RATSR_u {
	uint32_t R;
	struct {
		uint32_t TEMP:9;
		uint32_t Reserved0:22;
		uint32_t V:1;
	} B;
};

union TMU_CMCFG_u {
	uint32_t R;
	struct {
		uint32_t DAC_OFFSET:7;
		uint32_t Reserved0:1;
		uint32_t CMET:2;
		uint32_t DFD:2;
		uint32_t CLK_DIV:4;
		uint32_t SAR_RDY:1;
		uint32_t Reserved1:7;
		uint32_t RCTC:3;
		uint32_t Reserved2:1;
		uint32_t DEMA:1;
		uint32_t OCS:1;
		uint32_t OCM:1;
		uint32_t DPM:1;
	} B;
};


#endif
