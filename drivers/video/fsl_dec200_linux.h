/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018 NXP
 *
 * DEC200 driver header for Linux
 */

#ifndef __FSL_DEC200_LINUX__H
#define __FSL_DEC200_LINUX__H

#include "fsl_dec200_regs.h"
#include "fsl_dec200_ioctl.h"

#include <linux/types.h>

enum DEC200_TYPE {
	DEC200_DEC,
	DEC200_ENC,
	DEC200_MAX_NUM,
};

void fsl_dec200_enable_compression(enum DEC200_TYPE devtype, uint8_t axi_id);
void fsl_dec200_disable_compression(enum DEC200_TYPE devtype, uint8_t axi_id);

int fsl_dec200_set_compression_size(enum DEC200_TYPE devtype, uint8_t axi_id,
				    enum SWIZZLE val);
int fsl_dec200_set_compression_format(enum DEC200_TYPE devtype, uint8_t axi_id,
				      enum COMPRESSION_FORMAT val);
int fsl_dec200_set_swizzle(enum DEC200_TYPE devtype, uint8_t axi_id,
			   enum SWIZZLE val);

int fsl_dec200_reset(enum DEC200_TYPE devtype);
void fsl_dec200_registers(enum DEC200_TYPE devtype, uint8_t axi_id);
u32 fsl_dec200_get_reg_base(enum DEC200_TYPE devtype, uint8_t axi_id);

u32 fsl_dec200_get_read_buffer_base(enum DEC200_TYPE devtype, uint8_t axi_id);
int fsl_dec200_set_read_buffer_base(enum DEC200_TYPE devtype, uint8_t axi_id,
				    u32 addr);
u32 fsl_dec200_get_read_cache_base(enum DEC200_TYPE devtype, uint8_t axi_id);
int fsl_dec200_set_read_cache_base(enum DEC200_TYPE devtype, uint8_t axi_id,
				   u32 addr);
u32 fsl_dec200_get_write_buffer_base(enum DEC200_TYPE devtype, uint8_t axi_id);
int fsl_dec200_set_write_buffer_base(enum DEC200_TYPE devtype, uint8_t axi_id,
				     u32 addr);
u32 fsl_dec200_get_write_cache_base(enum DEC200_TYPE devtype, uint8_t axi_id);
int fsl_dec200_set_write_cache_base(enum DEC200_TYPE devtype, uint8_t axi_id,
				    u32 addr);

#endif /* __FSL_DEC200_LINUX__H */
