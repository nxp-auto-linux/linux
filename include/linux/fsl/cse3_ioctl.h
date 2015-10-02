/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 * CSE3 Ioctl User space Interface
 *
 * Copyright (c) 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * later as publishhed by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _CSE_IOCTL_H
#define _CSE_IOCTL_H

#ifndef __KERNEL__
#include <stdint.h>

/* TODO: unify with internal sizes */
#define AES_KEY_SIZE	16
#define RND_VAL_SIZE	16
#endif /* __KERNEL__ */

/**
 * AES-128 Encryption/Decryption requests
 * Note: For AES-128 CBC, the IV has to be set in advance.
 *       When using a plaintext key, the key also needs to be
 *       set in advance with the associated ioctl command.
 */
struct ioctl_crypt {
	uint32_t	len;
	int			key_id;
	int			kbs_key;
	void		*addr_in;
	void		*addr_out;
};

/**
 * Miyaguchi-Preneel (MP) compression request
 */
struct ioctl_mp {
	uint32_t	len;
	void		*addr_in;
	void		*addr_out;
};

/**
 * Generate and Verify CMAC requests
 */
struct ioctl_cmac {
	uint32_t	len;
	int			key_id;
	int			kbs_key;
	void		*addr_in;
	void		*addr_out;
};

/**
 * Load key request
 * Accepts either plaintext or encrypted custom keys.
 * An encrypt key is loaded using the M1, M2 and M3 blocks
 * defined by the SHE Memory Update Protocol.
 */
struct ioctl_ldkey {
	int			kbs_key;
	void		*addr_m1;
	void		*addr_m2;
	void		*addr_m3;
};

#define CSE_CMD_TYPE	0xc3

#define CSE3_IOCTL_COMPRESS_MP  _IOWR(CSE_CMD_TYPE, 1,  struct ioctl_mp)
#define CSE3_IOCTL_ENC_ECB      _IOWR(CSE_CMD_TYPE, 2,  struct ioctl_crypt)
#define CSE3_IOCTL_ENC_CBC      _IOWR(CSE_CMD_TYPE, 3,  struct ioctl_crypt)
#define CSE3_IOCTL_DEC_ECB      _IOWR(CSE_CMD_TYPE, 4,  struct ioctl_crypt)
#define CSE3_IOCTL_DEC_CBC      _IOWR(CSE_CMD_TYPE, 5,  struct ioctl_crypt)
#define CSE3_IOCTL_ENC_ECB_WK   _IOWR(CSE_CMD_TYPE, 6,  struct ioctl_crypt)
#define CSE3_IOCTL_ENC_CBC_WK   _IOWR(CSE_CMD_TYPE, 7,  struct ioctl_crypt)
#define CSE3_IOCTL_DEC_ECB_WK   _IOWR(CSE_CMD_TYPE, 8,  struct ioctl_crypt)
#define CSE3_IOCTL_DEC_CBC_WK   _IOWR(CSE_CMD_TYPE, 9,  struct ioctl_crypt)
#define CSE3_IOCTL_CHECK_MAC    _IOWR(CSE_CMD_TYPE, 10, struct ioctl_cmac)
#define CSE3_IOCTL_GEN_MAC      _IOWR(CSE_CMD_TYPE, 11, struct ioctl_cmac)
#define CSE3_IOCTL_LOAD_KEY     _IOW(CSE_CMD_TYPE,  12, struct ioctl_ldkey)
#define CSE3_IOCTL_RND          _IOC(_IOC_READ,  CSE_CMD_TYPE, 13, RND_VAL_SIZE)
#define CSE3_IOCTL_LOAD_PLKEY   _IOC(_IOC_WRITE, CSE_CMD_TYPE, 14, AES_KEY_SIZE)

#define CSE3_IOCTL_SET_KEY      _IOC(_IOC_WRITE, CSE_CMD_TYPE, 15, AES_KEY_SIZE)
#define CSE3_IOCTL_SET_IV       _IOC(_IOC_WRITE, CSE_CMD_TYPE, 16, AES_KEY_SIZE)

#endif /* _CSE_IOCTL_H */
