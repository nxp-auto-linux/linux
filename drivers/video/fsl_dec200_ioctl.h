/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018 NXP
 */

#ifndef __FSL_DEC200_IOCTL__H
#define __FSL_DEC200_IOCTL__H

struct IOCTL_SAMPLE {
	uint8_t axi_id;
};

struct IOCTL_SWIZZLE {
	uint8_t axi_id;
	enum SWIZZLE val;
};

struct IOCTL_COMPRESSION_SIZE {
	uint8_t axi_id;
	enum COMPRESSION_SIZE val;
};

struct IOCTL_COMPRESSION_FORMAT {
	uint8_t axi_id;
	enum COMPRESSION_FORMAT val;
};

struct IOCTL_ADDR {
	uint8_t axi_id;
	u32 addr;
};

#define IOCTL_RESET_CONFIGURATION       1
#define IOCTL_GET_REG_BASE              3

#define IOCTL_SET_COMPRESSION_SIZE      4
#define IOCTL_SET_COMPRESSION_FORMAT    5
#define IOCTL_SET_SWIZZLE               6

#define IOCTL_ENABLE_COMPRESSION        7
#define IOCTL_DISABLE_COMPRESSION       8

#define IOCTL_GET_READ_BUFFER_BASE      9
#define IOCTL_SET_READ_BUFFER_BASE      10

#define IOCTL_GET_READ_CACHE_BASE       11
#define IOCTL_SET_READ_CACHE_BASE       12

#define IOCTL_GET_WRITE_BUFFER_BASE     13
#define IOCTL_SET_WRITE_BUFFER_BASE     14

#define IOCTL_GET_WRITE_CACHE_BASE      15
#define IOCTL_SET_WRITE_CACHE_BASE      16

#endif /* __FSL_DEC200_IOCTL__H */
