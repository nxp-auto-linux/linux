/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
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

#ifndef _CSE_H
#define _CSE_H

#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#define MAX_ACCESS      10
#define AES_KEY_SIZE    16
#define RND_VAL_SIZE    16
#define AES_MAC_SIZE    16
#define MP_COMP_SIZE    16
#define M1_KEY_SIZE     16
#define M2_KEY_SIZE     32
#define M3_KEY_SIZE     16
#define M4_KEY_SIZE     32
#define M5_KEY_SIZE     16
#define BUFSIZ          4096
#define CSE_QUEUE_LEN   4

typedef struct {
	uint32_t cse_cr;            /* 0x0 */
	uint32_t cse_sr;            /* 0x4 */
	uint32_t cse_ir;            /* 0x8 */
	uint32_t cse_ecr;           /* 0xC */
	uint32_t cse_trng;          /* 0x10 */
	uint32_t cse_rsvd0[3];      /* 0x14 */
	uint32_t cse_cmd;           /* 0x20 */
	uint32_t cse_param[5];      /* 0x24 */
	uint32_t cse_rsvd1[6];      /* 0x38 */
	uint32_t cse_kia[2];        /* 0x50 */
	uint32_t cse_otp;           /* 0x58 */
	uint32_t cse_pc;            /* 0x5C */
	uint32_t cse_rsvd2[4];      /* 0x60 */
	uint32_t cse_sra0;          /* 0x70 */
	uint32_t cse_srs0;          /* 0x74 */
	uint32_t cse_rsvd3[98];     /* 0x78 */
	uint32_t cse_trng_test[64]; /* 0x200 */
	uint32_t cse_rsvd4[3904];   /* 0x300 */
} cse_reg_t;

/**
 *  Custom ioctl session context - fd private info
 *  lasts from open to release - copied on running req
 *  Keeps info set with ioctl commands
 *  Can be assigned to one or more requests
 */
typedef struct cse_req_ctx {
	struct cse_device_data *dev;
	uint8_t aes_key[AES_KEY_SIZE];
	uint8_t aes_iv[AES_KEY_SIZE];
	uint32_t flags;
} cse_ctx_t;

/**
 * Hardware context - physically contiguous, could be replaced with DMA desc
 * TODO: allocate the large buffers separately
 */

typedef struct cse_descriptor {
	uint8_t buffer_in[BUFSIZ];
	uint8_t buffer_out[BUFSIZ];
	uint8_t aes_key[AES_KEY_SIZE];
	uint8_t aes_iv[AES_KEY_SIZE];
	uint8_t rval[RND_VAL_SIZE];
	uint8_t mac[AES_MAC_SIZE];
	uint8_t mp[MP_COMP_SIZE];
	uint8_t m1[M1_KEY_SIZE];
	uint8_t m2[M2_KEY_SIZE];
	uint8_t m3[M3_KEY_SIZE];
	uint8_t m4[M4_KEY_SIZE];
	uint8_t m5[M5_KEY_SIZE];
	uint32_t len_in;
} cse_desc_t;

/**
 * Generic request info
 */
typedef struct cse_request {
	struct list_head	list;
	/* current context with request data */
	cse_ctx_t			*ctx;
	struct completion	complete;
	uint8_t				state;
	uint32_t			flags;
	int					key_id;
	int					phase;
	int					error;
} cse_req_t;

struct cse_queue {
	struct list_head list;
	unsigned int len;
	unsigned int max_len;
};

/**
 * General device info
 * Keeps a queue of pending requests
 **/
struct cse_device_data {
	struct device               *device;
	struct cdev                 cdev;
	cse_reg_t __iomem           *base;
	int                         irq;

	/* Current request info */
	cse_desc_t                  *hw_desc;
	dma_addr_t                  hw_desc_phys;
	cse_req_t                   *req;
	/* TODO: in/out dynamic desc */

	struct cse_queue            queue;
	/* Queue, tasklets, lock */
	spinlock_t                  lock;
	struct tasklet_struct       done_task;

	size_t                      size;
	unsigned long               flags;
	struct semaphore            access;
};
extern struct cse_device_data *cse_dev_ptr;

#define FLAG_BUSY		(0x1UL)

/*
 * Request state
 */
#define FLAG_SUBMITTED		(0x1UL)
#define FLAG_CANCELED		(0x2UL)
#define FLAG_DONE			(0x4UL)

/*
 * Request flags
 * [cbc:1,wk:1,alg:16,kbs:1,other:13]
 */
#define FLAG_CBC			(0x1UL)
#define FLAG_WITH_KEY		(0x2UL)
#define ALG_MASK			(0x3FCUL)
#define FLAG_RND			(0x4UL)
#define FLAG_ENC			(0x8UL)
#define FLAG_DEC			(0x10UL)
#define FLAG_GEN_MAC		(0x20UL)
#define FLAG_VER_MAC		(0x40UL)
#define FLAG_LOAD_KEY		(0x80UL)
#define FLAG_LOAD_PLKEY		(0x100UL)
#define FLAG_MP_COMP		(0x200UL)
#define FLAG_KBS			(0x400UL)

#define CSE_CR_CIE			(0x01UL)
#define CSE_CR_KBS			(0x20UL)
#define CSE_IR_CIF			(0x01UL)

#define CSE_CMD_SECURE_BOOT			(0xDUL)
#define CSE_CMD_INIT_CSE			(0x15UL)
#define CSE_CMD_PUBLISH_KEY_IMG		(0x17UL)
#define CSE_CMD_INIT_RNG			(0x0AUL)

#define CSE_CMD_ENC_ECB				(0x01UL)
#define CSE_CMD_ENC_CBC				(0x02UL)
#define CSE_CMD_DEC_ECB				(0x03UL)
#define CSE_CMD_DEC_CBC				(0x04UL)
#define CSE_CMD_GEN_MAC				(0x05UL)
#define CSE_CMD_VER_MAC				(0x06UL)
#define CSE_CMD_LOAD_KEY			(0x07UL)
#define CSE_CMD_LOAD_PLAIN_KEY		(0x08UL)
#define CSE_CMD_CANCEL				(0x11UL)
#define CSE_CMD_RND					(0x0CUL)
#define CSE_CMD_COMPRESS_MP			(0x16UL)

#define CSE_SR_BSY					(0x1UL)
#define CSE_SR_BOK					(0x10UL)

#define MIN_KEY_ID		0x0
#define MAX_KEY_ID		0xE
#define CSE_KEYID_RAM	0xE
#define UNDEFINED		-1


int cse_handle_request(struct cse_device_data *dev, cse_req_t *req);
void cse_cancel_request(cse_req_t *req);
void cse_finish_req(struct cse_device_data *dev, cse_req_t *req);

#endif /* _CSE_H */
