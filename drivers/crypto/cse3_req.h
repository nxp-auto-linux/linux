/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 * CSE3 Ioctl/User space Interface
 *
 * Copyright (c) 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * later as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _CSE_REQ_H
#define _CSE_REQ_H

/**
 * AES-128 Encryption/Decryption requests
 * Note: The Key and IV values can be set
 * independently and are stored in the context
 * field from the base request
 */
struct cse_crypt_request {
	cse_req_t	base;
	uint8_t		buffer_in[BUFSIZ];
	uint8_t		buffer_out[BUFSIZ];
	uint32_t	len_in;
	uint32_t	len_out;
};

/**
 * Miyaguchi-Preneel (MP) compression request
 */
struct cse_mp_request {
	cse_req_t	base;
	uint8_t		buffer_in[BUFSIZ];
	uint8_t		buffer_out[MP_COMP_SIZE];
	uint32_t	len_in;
};

/**
 * Random number Generator request
 */
struct cse_rval_request {
	cse_req_t	base;
	uint8_t		rval[RND_VAL_SIZE];
};

/**
 * Generate and Verify CMAC requests
 */
struct cse_cmac_request {
	cse_req_t	base;
	uint8_t		buffer_in[BUFSIZ];
	uint8_t		buffer_out[AES_MAC_SIZE];
	uint32_t	len_in;
	uint32_t	status;
};

/**
 * Load Key request
 */
struct cse_ldkey_request {
	cse_req_t	base;
	uint8_t		m1[M1_KEY_SIZE];
	uint8_t		m2[M2_KEY_SIZE];
	uint8_t		m3[M3_KEY_SIZE];
};

/********************************************
 *************** IOCTL Interface ************
 ********************************************/

struct ioctl_crypt {
	uint32_t	len;
	int			key_id;
	int			kbs_key;
	void		*addr_in;
	void		*addr_out;
};

struct ioctl_mp {
	uint32_t	len;
	void		*addr_in;
	void		*addr_out;
};

struct ioctl_cmac {
	uint32_t	len;
	int			key_id;
	int			kbs_key;
	void		*addr_in;
	void		*addr_out;
};

struct ioctl_ldkey {
	int			kbs_key;
	void		*addr_m1;
	void		*addr_m2;
	void		*addr_m3;
};

int cse_ioctl_rnd(cse_ctx_t *ctx, unsigned int cmd, unsigned long arg);
int cse_ioctl_load_plkey(cse_ctx_t *ctx, unsigned int cmd, unsigned long arg);
int cse_ioctl_load_key(cse_ctx_t *ctx, unsigned int cmd, unsigned long arg);
int cse_ioctl_comp(cse_ctx_t *ctx, unsigned int cmd, unsigned long arg);
int cse_ioctl_cmac(cse_ctx_t *ctx, unsigned int cmd, unsigned long arg,
		bool has_key, bool verif);
int cse_ioctl_crypt(cse_ctx_t *ctx, unsigned int cmd,
		unsigned long arg, bool has_key);

void cse_init_queue(struct cse_queue *queue, unsigned int max_len);
int cse_enqueue_request(struct cse_queue *queue, cse_req_t *req);
void *cse_dequeue_request(struct cse_queue *queue);
int cse_remove_request(struct cse_queue *queue, cse_req_t *req);
void cse_destroy_queue(struct cse_queue *queue);

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
#define CSE3_IOCTL_LOAD_KEY     _IOW(CSE_CMD_TYPE,  15, struct ioctl_ldkey)
#define CSE3_IOCTL_RND          _IOC(_IOC_READ,  CSE_CMD_TYPE, 12, RND_VAL_SIZE)
#define CSE3_IOCTL_SET_KEY      _IOC(_IOC_WRITE, CSE_CMD_TYPE, 13, AES_KEY_SIZE)
#define CSE3_IOCTL_SET_IV       _IOC(_IOC_WRITE, CSE_CMD_TYPE, 14, AES_KEY_SIZE)
#define CSE3_IOCTL_LOAD_PLKEY   _IOC(_IOC_WRITE, CSE_CMD_TYPE, 16, AES_KEY_SIZE)

#endif /* _CSE_REQ_H */
