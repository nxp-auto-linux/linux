// SPDX-License-Identifier: GPL-2.0+
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 *
 * Copyright (c) 2015-2016 Freescale Semiconductor, Inc.
 * CSE3 Ioctl/User space Interface
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

#include "cse3.h"
#include "cse3_req.h"

/**
 * AES-128 Encryption/Decryption requests
 * Note: The Key and IV values can be set
 * independently and are stored in the context
 * field from the base request
 */
struct cse_crypt_request {
	cse_req_t	base;
	uint8_t		*buffer_in;
	uint8_t		*buffer_out;
	uint32_t	len_in;
	uint32_t	len_out;
};

/**
 * Miyaguchi-Preneel (MP) compression request
 */
struct cse_mp_request {
	cse_req_t	base;
	uint8_t		*buffer_in;
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
	uint8_t		*buffer_in;
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

static int cse_ioctl_copy_output(struct cse_device_data *dev,
				 struct cse_request *req)
{
	struct cse_crypt_request *crypt_req;
	struct cse_cmac_request *cmac_req;
	struct cse_mp_request *mp_req;
	struct cse_rval_request *rval_req;
	cse_desc_t *desc = dev->hw_desc;

	if (dev->req->flags & (FLAG_ENC|FLAG_DEC)) {
		crypt_req = (struct cse_crypt_request *)dev->req;
		dma_sync_single_for_cpu(dev->device, dev->buffer_out_phys,
				crypt_req->len_out, DMA_FROM_DEVICE);
		memcpy(crypt_req->buffer_out, dev->buffer_out,
				crypt_req->len_out);

	} else if (dev->req->flags & (FLAG_GEN_MAC|FLAG_VER_MAC)) {
		cmac_req = (struct cse_cmac_request *)dev->req;
		if (dev->req->flags & FLAG_GEN_MAC)
			memcpy(cmac_req->buffer_out, desc->mac, AES_MAC_SIZE);
		else
			cmac_req->status = readl(&dev->base->cse_param[4]);

	} else if (dev->req->flags & FLAG_MP_COMP) {
		mp_req = (struct cse_mp_request *)dev->req;
		memcpy(mp_req->buffer_out, desc->mp, MP_COMP_SIZE);

	} else if (dev->req->flags & FLAG_RND) {
		rval_req = (struct cse_rval_request *)dev->req;
		memcpy(rval_req->rval, desc->rval, RND_VAL_SIZE);
	}

	return 0;
}

static int cse_ioctl_copy_input(struct cse_device_data *dev,
				struct cse_request *req)
{
	struct cse_crypt_request *crypt_req;
	struct cse_cmac_request *cmac_req;
	struct cse_mp_request *mp_req;
	struct cse_ldkey_request *key_req;
	cse_desc_t *desc = dev->hw_desc;

	if (req->flags & FLAG_LOAD_KEY) {
		key_req = (struct cse_ldkey_request *)req;
		memcpy(desc->m1, key_req->m1, M1_KEY_SIZE);
		memcpy(desc->m2, key_req->m2, M2_KEY_SIZE);
		memcpy(desc->m3, key_req->m3, M3_KEY_SIZE);

	} else if (req->flags & (FLAG_ENC|FLAG_DEC)) {
		crypt_req = (struct cse_crypt_request *)req;
		if (cse_allocate_buffer(dev->device, &dev->buffer_in,
				&dev->buffer_in_phys,
				crypt_req->len_in, DMA_TO_DEVICE))
			return -ENOMEM;
		if (cse_allocate_buffer(dev->device, &dev->buffer_out,
				&dev->buffer_out_phys,
				crypt_req->len_in, DMA_FROM_DEVICE))
			return -ENOMEM;

		memcpy(desc->aes_key, req->ctx->aes_key, AES_KEY_SIZE);
		memcpy(dev->buffer_in, crypt_req->buffer_in,
				crypt_req->len_in);
		desc->nbits = desc_len(crypt_req->len_in);
		if (req->flags & FLAG_CBC)
			memcpy(desc->aes_iv, req->ctx->aes_iv, AES_KEY_SIZE);

		dma_sync_single_for_device(dev->device, dev->buffer_in_phys,
				crypt_req->len_in, DMA_TO_DEVICE);
	} else if (req->flags & (FLAG_GEN_MAC|FLAG_VER_MAC)) {
		cmac_req = (struct cse_cmac_request *)req;
		if (cse_allocate_buffer(dev->device, &dev->buffer_in,
				&dev->buffer_in_phys,
				cmac_req->len_in, DMA_TO_DEVICE))
			return -ENOMEM;

		memcpy(desc->aes_key, req->ctx->aes_key, AES_KEY_SIZE);
		if (cmac_req->len_in) {
			memcpy(dev->buffer_in, cmac_req->buffer_in,
					cmac_req->len_in);
			dma_sync_single_for_device(dev->device,
					dev->buffer_in_phys,
					cmac_req->len_in, DMA_TO_DEVICE);
		}
		desc->nbits = desc_len(cmac_req->len_in);

		if (req->flags & FLAG_VER_MAC)
			memcpy(desc->mac, cmac_req->buffer_out, AES_MAC_SIZE);

	} else if (req->flags & FLAG_MP_COMP) {
		mp_req = (struct cse_mp_request *)req;
		if (cse_allocate_buffer(dev->device, &dev->buffer_in,
				&dev->buffer_in_phys,
				mp_req->len_in, DMA_TO_DEVICE))
			return -ENOMEM;

		memcpy(dev->buffer_in, mp_req->buffer_in, mp_req->len_in);
		desc->nbits = desc_len(mp_req->len_in);

		dma_sync_single_for_device(dev->device, dev->buffer_in_phys,
				mp_req->len_in, DMA_TO_DEVICE);
	} else if (req->flags & FLAG_LOAD_PLKEY) {
		memcpy(desc->aes_key, req->ctx->aes_key, AES_KEY_SIZE);
	}

	return 0;
}

static void cse_ioctl_complete(struct cse_device_data *dev,
			       struct cse_request *req)
{
	complete(&req->complete);
}

static void cse_ioctl_free_extra(struct cse_request *req)
{
	if (req->flags & (FLAG_ENC|FLAG_DEC)) {
		kfree(((struct cse_crypt_request *)req)->buffer_in);
		kfree(((struct cse_crypt_request *)req)->buffer_out);
	} else if (req->flags & FLAG_MP_COMP) {
		kfree(((struct cse_mp_request *)req)->buffer_in);
	} else if (req->flags & (FLAG_GEN_MAC|FLAG_VER_MAC)) {
		kfree(((struct cse_cmac_request *)req)->buffer_in);
	}
}

static void cse_ioctl_init_ops(struct cse_request *req)
{
	req->copy_output = cse_ioctl_copy_output;
	req->copy_input = cse_ioctl_copy_input;
	req->comp = cse_ioctl_complete;
	req->free_extra = cse_ioctl_free_extra;
}

int cse_ioctl_rnd(cse_ctx_t *ctx, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct cse_rval_request *new_req;

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	if (!new_req) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for rnd val comm.\n");
		return -ENOMEM;
	}
	new_req->base.ctx = ctx;
	new_req->base.phase = 0;
	new_req->base.flags = FLAG_RND;
	cse_ioctl_init_ops(&new_req->base);
	init_completion(&new_req->base.complete);
	ret = cse_handle_request(ctx->dev, (cse_req_t *)new_req);

	if (!ret) {
		if (wait_for_completion_interruptible(
					&new_req->base.complete)) {
			cse_cancel_request((cse_req_t *)new_req);
			return -EINTR;
		} else if (new_req->base.error) {
			ret = new_req->base.error;
		} else if (copy_to_user((uint8_t __user *)arg,
					new_req->rval, RND_VAL_SIZE)) {
			ret = -EFAULT;
		}
	}

	cse_finish_req(ctx->dev, (cse_req_t *)new_req);
	return ret;
}

int cse_ioctl_load_plkey(cse_ctx_t *ctx, unsigned int cmd, unsigned long arg)
{
	cse_req_t *new_req;
	int ret = 0;

	/* Copy input key */
	if (copy_from_user(ctx->aes_key, (uint8_t __user *)arg, AES_KEY_SIZE))
		return -EFAULT;

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	if (!new_req) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for load key comm.\n");
		return -ENOMEM;
	}
	new_req->ctx = ctx;
	new_req->phase = 0;
	new_req->flags = FLAG_LOAD_PLKEY;
	cse_ioctl_init_ops(new_req);
	init_completion(&new_req->complete);
	ret = cse_handle_request(ctx->dev, new_req);

	if (!ret) {
		if (wait_for_completion_interruptible(
					&new_req->complete)) {
			cse_cancel_request(new_req);
			return -EINTR;
		} else if (new_req->error) {
			ret = new_req->error;
		}
	}

	cse_finish_req(ctx->dev, new_req);
	return ret;
}

int cse_ioctl_load_key(cse_ctx_t *ctx, unsigned int cmd, unsigned long arg)
{
	struct cse_ldkey_request *new_req;
	struct ioctl_ldkey ldkey;
	int ret = 0, remains1, remains2, remains3;

	if (copy_from_user(&ldkey, (struct ioctl_ldkey __user *)arg,
				sizeof(struct ioctl_ldkey)))
		return -EFAULT;

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	if (!new_req) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for load key comm.\n");
		return -ENOMEM;
	}
	new_req->base.ctx = ctx;
	new_req->base.phase = 0;
	new_req->base.flags = FLAG_LOAD_KEY;
	if (ldkey.kbs_key)
		new_req->base.flags |= FLAG_KBS;

	/* Copy input text m1,m2,m3 and send command to the device */
	remains1 = copy_from_user(new_req->m1,
			(uint8_t __user *)ldkey.addr_m1, M1_KEY_SIZE);
	remains2 = copy_from_user(new_req->m2,
			(uint8_t __user *)ldkey.addr_m2, M2_KEY_SIZE);
	remains3 = copy_from_user(new_req->m3,
			(uint8_t __user *)ldkey.addr_m3, M3_KEY_SIZE);
	if (remains1 || remains2 || remains3) {
		kfree(new_req);
		return -EFAULT;
	}

	cse_ioctl_init_ops(&new_req->base);
	init_completion(&new_req->base.complete);
	ret = cse_handle_request(ctx->dev, (cse_req_t *)new_req);

	if (!ret) {
		if (wait_for_completion_interruptible(
					&new_req->base.complete)) {
			cse_cancel_request((cse_req_t *)new_req);
			return -EINTR;
		} else if (new_req->base.error) {
			ret = new_req->base.error;
		}
	}

	cse_finish_req(ctx->dev, (cse_req_t *)new_req);
	return ret;
}

int cse_ioctl_comp(cse_ctx_t *ctx, unsigned int cmd, unsigned long arg)
{
	struct cse_mp_request *new_req;
	struct ioctl_mp comp;
	int ret = 0;

	if (copy_from_user(&comp, (struct ioctl_mp __user *)arg,
			sizeof(struct ioctl_mp)))
		return -EFAULT;

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	if (!new_req) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for compression.\n");
		return -ENOMEM;
	}
	new_req->buffer_in = kzalloc(comp.len, GFP_KERNEL);
	if (!new_req->buffer_in) {
		dev_err(cse_dev_ptr->device, "failed to alloc compression buffer.\n");
		kfree(new_req);
		return -ENOMEM;
	}
	new_req->base.ctx = ctx;
	new_req->base.phase = 0;
	new_req->base.flags = FLAG_MP_COMP;
	new_req->len_in = comp.len;

	/* Copy input text and send command to the device */
	if (copy_from_user(new_req->buffer_in,
			(uint8_t __user *)comp.addr_in, new_req->len_in)) {
		kfree(new_req->buffer_in);
		kfree(new_req);
		return -EFAULT;
	}
	cse_ioctl_init_ops(&new_req->base);
	init_completion(&new_req->base.complete);
	ret = cse_handle_request(ctx->dev, (cse_req_t *)new_req);

	if (!ret) {
		if (wait_for_completion_interruptible(
					&new_req->base.complete)) {
			cse_cancel_request((cse_req_t *)new_req);
			return -EINTR;
		} else if (new_req->base.error) {
			ret = new_req->base.error;
		} else if (copy_to_user((uint8_t __user *)comp.addr_out,
					new_req->buffer_out, MP_COMP_SIZE)) {
			ret = -EFAULT;
		}
	}

	cse_finish_req(ctx->dev, (cse_req_t *)new_req);
	return ret;
}

int cse_ioctl_cmac(cse_ctx_t *ctx, unsigned int cmd, unsigned long arg,
		bool has_key, bool verif)
{
	struct cse_cmac_request *new_req;
	struct ioctl_cmac cmac;
	int ret = 0;

	if (copy_from_user(&cmac, (struct ioctl_cmac __user *)arg,
			sizeof(struct ioctl_cmac)))
		return -EFAULT;

	/* Check key slot id; valid ranges (0x0-0xE)
	 * Note: In order to use the second key bank,
	 * flag FLAG_KBS has to be set
	 */
	if (has_key && (cmac.key_id < MIN_KEY_ID
				|| cmac.key_id > MAX_KEY_ID))
		return -EINVAL;

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	if (!new_req) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for cmac request.\n");
		return -ENOMEM;
	}
	new_req->buffer_in = kzalloc(cmac.len, GFP_KERNEL);
	if (!new_req->buffer_in) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for cmac buffer.\n");
		kfree(new_req);
		return -ENOMEM;
	}
	new_req->base.ctx = ctx;
	new_req->len_in = cmac.len;

	/* Custom key from a preloaded slot or plaintext key
	 * loaded in RAM slot
	 */
	if (has_key) {
		new_req->base.phase = 0;
		new_req->base.key_id = cmac.key_id;
		new_req->base.flags |= FLAG_WITH_KEY;
		if (cmac.kbs_key)
			new_req->base.flags |= FLAG_KBS;
	} else {
		new_req->base.phase = 1;
		new_req->base.key_id = UNDEFINED;
	}

	if (verif)
		new_req->base.flags |= FLAG_VER_MAC;
	else
		new_req->base.flags |= FLAG_GEN_MAC;

	/* Copy input text and send command to the device */
	if (copy_from_user(new_req->buffer_in,
			(uint8_t __user *)cmac.addr_in, new_req->len_in)) {
		kfree(new_req->buffer_in);
		kfree(new_req);
		return -EFAULT;
	}

	if (verif) {
		if (copy_from_user(new_req->buffer_out,
				(uint8_t __user *)cmac.addr_out,
				AES_KEY_SIZE)) {
			kfree(new_req->buffer_in);
			kfree(new_req);
			return -EFAULT;
		}
	}
	cse_ioctl_init_ops(&new_req->base);
	init_completion(&new_req->base.complete);
	ret = cse_handle_request(ctx->dev, (cse_req_t *)new_req);

	if (!ret) {
		if (wait_for_completion_interruptible(
					&new_req->base.complete)) {
			cse_cancel_request((cse_req_t *)new_req);
			return -EINTR;
		} else if (new_req->base.error) {
			ret = new_req->base.error;
		} else if (new_req->base.flags & FLAG_GEN_MAC) {
			/* Generate CMAC */
			if (copy_to_user((uint8_t __user *)cmac.addr_out,
						new_req->buffer_out,
						AES_MAC_SIZE)) {
				ret = -EFAULT;
			}
		} else { /* Verify CMAC */
			ret = new_req->status;
		}
	}

	cse_finish_req(ctx->dev, (cse_req_t *)new_req);
	return ret;
}

int cse_ioctl_crypt(cse_ctx_t *ctx, unsigned int cmd,
		unsigned long arg, bool has_key)
{
	struct cse_crypt_request *new_req;
	struct ioctl_crypt crypt;
	int ret = 0;

	if (copy_from_user(&crypt, (struct ioctl_crypt __user *)arg,
			sizeof(struct ioctl_crypt)))
		return -EFAULT;

	/* Check key slot id; valid ranges (0x0-0xE)
	 * Note: In order to use the second key bank,
	 * flag FLAG_KBS has to be set
	 */
	if (has_key && (crypt.key_id < MIN_KEY_ID
				|| crypt.key_id > MAX_KEY_ID))
		return -EINVAL;

	new_req = kzalloc(sizeof(*new_req), GFP_KERNEL);
	if (!new_req) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for crypto request.\n");
		return -ENOMEM;
	}
	new_req->buffer_in = kzalloc(crypt.len, GFP_KERNEL);
	new_req->buffer_out = kzalloc(crypt.len, GFP_KERNEL);
	if (!new_req->buffer_in || !new_req->buffer_out) {
		dev_err(cse_dev_ptr->device, "failed to alloc mem for crypto buffers.\n");
		kfree(new_req);
		return -ENOMEM;
	}
	new_req->base.ctx = ctx;
	new_req->len_in = new_req->len_out = crypt.len;
	if (cmd == CSE3_IOCTL_ENC_CBC || cmd == CSE3_IOCTL_ENC_ECB ||
			cmd == CSE3_IOCTL_ENC_CBC_WK ||
			cmd == CSE3_IOCTL_ENC_ECB_WK)
		new_req->base.flags = FLAG_ENC;
	else
		new_req->base.flags = FLAG_DEC;

	if (cmd == CSE3_IOCTL_DEC_CBC || cmd == CSE3_IOCTL_DEC_CBC_WK ||
			cmd == CSE3_IOCTL_ENC_CBC ||
			cmd == CSE3_IOCTL_ENC_CBC_WK)
		new_req->base.flags |= FLAG_CBC;

	/* Custom key from a preloaded slot or plaintext key
	 * loaded in RAM slot
	 */
	if (has_key) {
		new_req->base.phase = 0;
		new_req->base.key_id = crypt.key_id;
		new_req->base.flags |= FLAG_WITH_KEY;
		if (crypt.kbs_key)
			new_req->base.flags |= FLAG_KBS;
	} else {
		new_req->base.phase = 1;
		new_req->base.key_id = UNDEFINED;
	}

	/* Copy input text and send command to the device */
	if (copy_from_user(new_req->buffer_in,
			(uint8_t __user *)crypt.addr_in, new_req->len_in)) {
		kfree(new_req->buffer_in);
		kfree(new_req->buffer_out);
		kfree(new_req);
		return -EFAULT;
	}
	cse_ioctl_init_ops(&new_req->base);
	init_completion(&new_req->base.complete);
	ret = cse_handle_request(ctx->dev, (cse_req_t *)new_req);

	if (!ret) {
		if (wait_for_completion_interruptible(
					&new_req->base.complete)) {
			cse_cancel_request((cse_req_t *)new_req);
			return -EINTR;
		} else if (new_req->base.error) {
			ret = new_req->base.error;
		} else if (copy_to_user((uint8_t __user *)crypt.addr_out,
					new_req->buffer_out,
					new_req->len_out)) {
			ret = -EFAULT;
		}
	}

	cse_finish_req(ctx->dev, (cse_req_t *)new_req);
	return ret;
}

void cse_init_queue(struct cse_queue *queue, unsigned int max_len)
{
	INIT_LIST_HEAD(&queue->list);
	queue->len = 0;
	queue->max_len = max_len;
}

void cse_destroy_queue(struct cse_queue *queue)
{
	cse_req_t *req, *n;

	list_for_each_entry_safe(req, n, &queue->list, list) {
		list_del(&req->list);
		if (req->free_extra)
			req->free_extra(req);
		kfree(req);
	}
}

int cse_remove_request(struct cse_queue *queue, cse_req_t *delReq)
{
	cse_req_t *req, *n;

	list_for_each_entry_safe(req, n, &queue->list, list) {
		if (req == delReq) {
			list_del(&req->list);
			return 1;
		}
	}

	return 0;
}

int cse_enqueue_request(struct cse_queue *queue, cse_req_t *req)
{
	if (unlikely(queue->len >= queue->max_len))
		return -EBUSY;

	queue->len++;
	list_add_tail(&req->list, &queue->list);
	return 0;
}

void *cse_dequeue_request(struct cse_queue *queue)
{
	struct list_head *request;

	if (unlikely(!queue->len))
		return NULL;

	queue->len--;

	request = queue->list.next;
	list_del(request);

	return (char *)list_entry(request, cse_req_t, list);
}
