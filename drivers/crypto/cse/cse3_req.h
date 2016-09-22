/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 * CSE3 Requests Interface
 *
 * Copyright (c) 2015-2016 Freescale Semiconductor, Inc.
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

#include <linux/fsl/cse3_ioctl.h>

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

#endif /* _CSE_REQ_H */
