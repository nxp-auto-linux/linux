/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018 NXP
 */

#ifndef __FSL_DEC200_REGS__H
#define __FSL_DEC200_REGS__H

#include "fsl_dcu_typedefs.h"

/* DEC200 - Peripheral register structure */
struct DEC200_MemMap {
vuint8_t  __offset_gcregDECReadConfig[0x800];
vuint32_t gcregDECReadConfig[8];        /* offset: 0x00000800*/

vuint8_t  __offset_gcregDECWriteConfig[0x20];
vuint32_t gcregDECWriteConfig[8];       /* offset: 0x00000840*/

vuint8_t  __offset_gcregDECReadBufferBase[0x20];
vuint32_t gcregDECReadBufferBase[8];    /* offset: 0x00000880*/

vuint8_t  __offset_gcregDECReadCacheBase[0x20];
vuint32_t gcregDECReadCacheBase[8];     /* offset: 0x000008C0*/

vuint8_t  __offset_gcregDECWriteBufferBase[0x20];
vuint32_t gcregDECWriteBufferBase[8];   /* offset: 0x00000900*/

vuint8_t  __offset_gcregDECWriteCacheBase[0x20];
vuint32_t gcregDECWriteCacheBase[8];    /* offset: 0x00000940*/

vuint8_t  __offset_gcregDECControl[0x20];
vuint32_t gcregDECControl;              /* offset: 0x00000980*/
vuint32_t gcregDECIntrAcknowledge;      /* offset: 0x00000984*/
vuint32_t gcregDECIntrEnbl;             /* offset: 0x00000988*/
vuint32_t gcDECTotalReadsIn;            /* offset: 0x00000998*/
vuint32_t gcDECTotalWritesIn;           /* offset: 0x0000099C*/
vuint32_t gcDECTotalReadBurstsIn;       /* offset: 0x000009A0*/
vuint32_t gcDECTotalWriteBurstsIn;      /* offset: 0x000009A4*/
vuint32_t gcDECTotalReadsReqIn;         /* offset: 0x000009A8*/
vuint32_t gcDECTotalWritesReqIn;        /* offset: 0x000009AC*/
vuint32_t gcDECTotalReadLastsIn;        /* offset: 0x000009B0*/
vuint32_t gcDECTotalWriteLastsIn;       /* offset: 0x000009B4*/
vuint32_t gcDECTotalReadsOUT;           /* offset: 0x000009B8*/
vuint32_t gcDECTotalWritesOUT;          /* offset: 0x000009BC*/
vuint32_t gcDECTotalReadBurstsOUT;      /* offset: 0x000009C0*/
vuint32_t gcDECTotalWriteBurstsOUT;     /* offset: 0x000009C4*/
vuint32_t gcDECTotalReadsReqOUT;        /* offset: 0x000009C8*/
vuint32_t gcDECTotalWritesReqOUT;       /* offset: 0x000009CC*/
vuint32_t gcDECTotalReadLastsOUT;       /* offset: 0x000009D0*/
vuint32_t gcDECTotalWriteLastsOUT;      /* offset: 0x000009D4*/
};

/* DEC200_gcregDECWriteConfign */
enum SWIZZLE {
	SWIZZLE_ARGB,
	SWIZZLE_RGBA,
	SWIZZLE_ABGR,
	SWIZZLE_BGRA,
};
#define SWIZZLE_SHIFT                   20

enum COMPRESSION_FORMAT {
	COMPRESSION_FORMAT_ARGB8,
	COMPRESSION_FORMAT_XRGB8,
	COMPRESSION_FORMAT_AYUV,
	COMPRESSION_FORMAT_UYVY,
	COMPRESSION_FORMAT_YUY2,
	COMPRESSION_FORMAT_YUV_ONLY,
	COMPRESSION_FORMAT_UV_MIX,
};
#define COMPRESSION_FORMAT_SHIFT        3

enum COMPRESSION_SIZE {
	COMPRESSION_SIZE64_BYTE,
	COMPRESSION_SIZE128_BYTE,
	COMPRESSION_SIZE256_BYTE,
};
#define COMPRESSION_SIZE_SHIFT          1

#define COMPRESSION_ENABLE              (1 << 0)

/* DEC200_gcregDECControl */
#define CLK_DIS                         (1 << 17)
#define ENABLE96_BYTE_YUV_COMP          (1 << 7)
#define ENABLE_WRITE_SYNC               (1 << 6)
#define DISABLE_RAM_POWER_OPTIMIZATION  (1 << 5)
#define SOFT_RESET                      (1 << 4)
#define DISABLE_DEBUG_REGISTERS         (1 << 3)
#define DISABLE_RAM_CLOCK_GATING        (1 << 2)
#define DISABLE_COMPRESSION             (1 << 1)
#define DECCONTROL_RESET_VALUE          0xEE08


/* DEC200_gcregDECIntrEnbl */
#define FLUSH_DN_INT_ENBL               (1 << 31)
#define AXI_ERR_INT_ENBL                (1 << 30)

/* DEC200_gcregDECIntrEnbl */
#define FLUSH_DN_INT_ENBL               (1 << 31)
#define AXI_ERR_INT_ENBL                (1 << 30)

#endif /* __FSL_DEC200_REGS__H */
