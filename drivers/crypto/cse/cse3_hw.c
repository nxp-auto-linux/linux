// SPDX-License-Identifier: GPL-2.0+
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 * CSE3 Hardware API
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

#include "cse3.h"
#include "cse3_hw.h"

static inline void cse_cmd_cancel(struct cse_device_data *cse_dev)
{
	/* Check for Previous Command for Command Complete */
	if (readl(&cse_dev->base->cse_sr) & CSE_SR_BSY)
		writel(CSE_CMD_CANCEL, &cse_dev->base->cse_cmd);
}

static void cse_cmd_load_plainkey(struct cse_device_data *cse_dev)
{
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, aes_key),
			&cse_dev->base->cse_param[0]);
	writel(CSE_CMD_LOAD_PLAIN_KEY, &cse_dev->base->cse_cmd);
}

static void cse_cmd_dec(struct cse_device_data *cse_dev, int key_id, int cbc)
{
	int param = 0;
	int nblocks = req_len(cse_dev->hw_desc->nbits)/AES_KEY_SIZE;

	writel(key_id, &cse_dev->base->cse_param[param++]);
	if (cbc)
		writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, aes_iv),
				&cse_dev->base->cse_param[param++]);
	writel(nblocks, &cse_dev->base->cse_param[param++]);
	writel(cse_dev->buffer_in_phys,
			&cse_dev->base->cse_param[param++]);
	writel(cse_dev->buffer_out_phys,
			&cse_dev->base->cse_param[param++]);
	writel(cbc ? CSE_CMD_DEC_CBC : CSE_CMD_DEC_ECB,
			&cse_dev->base->cse_cmd);
}

static void cse_cmd_enc(struct cse_device_data *cse_dev, int key_id, int cbc)
{
	int param = 0;
	int nblocks = req_len(cse_dev->hw_desc->nbits)/AES_KEY_SIZE;

	writel(key_id, &cse_dev->base->cse_param[param++]);
	if (cbc)
		writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, aes_iv),
				&cse_dev->base->cse_param[param++]);
	writel(nblocks, &cse_dev->base->cse_param[param++]);
	writel(cse_dev->buffer_in_phys,
			&cse_dev->base->cse_param[param++]);
	writel(cse_dev->buffer_out_phys,
			&cse_dev->base->cse_param[param++]);
	writel(cbc ? CSE_CMD_ENC_CBC : CSE_CMD_ENC_ECB,
			&cse_dev->base->cse_cmd);
}

static void cse_cmd_gen_mac(struct cse_device_data *cse_dev, int key_id)
{
	writel(key_id, &cse_dev->base->cse_param[0]);
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, nbits),
			&cse_dev->base->cse_param[1]);
	writel(cse_dev->buffer_in_phys,
			&cse_dev->base->cse_param[2]);
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, mac),
			&cse_dev->base->cse_param[3]);
	writel(CSE_CMD_GEN_MAC, &cse_dev->base->cse_cmd);
}

static void cse_cmd_ver_mac(struct cse_device_data *cse_dev,
		int key_id, int mlen)
{
	writel(key_id, &cse_dev->base->cse_param[0]);
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, nbits),
			&cse_dev->base->cse_param[1]);
	writel(cse_dev->buffer_in_phys,
			&cse_dev->base->cse_param[2]);
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, mac),
			&cse_dev->base->cse_param[3]);
	writel(mlen, &cse_dev->base->cse_param[4]);
	writel(CSE_CMD_VER_MAC, &cse_dev->base->cse_cmd);
}

static void cse_cmd_rnd(struct cse_device_data *cse_dev)
{
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, rval),
			&cse_dev->base->cse_param[0]);
	writel(CSE_CMD_RND, &cse_dev->base->cse_cmd);
}

static void cse_cmd_mp_comp(struct cse_device_data *cse_dev)
{
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, nbits),
			&cse_dev->base->cse_param[0]);
	writel(cse_dev->buffer_in_phys,
			&cse_dev->base->cse_param[1]);
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, mp),
			&cse_dev->base->cse_param[2]);
	writel(CSE_CMD_COMPRESS_MP, &cse_dev->base->cse_cmd);
}

static void cse_cmd_load_key(struct cse_device_data *cse_dev)
{
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, m1),
			&cse_dev->base->cse_param[0]);
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, m2),
			&cse_dev->base->cse_param[1]);
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, m3),
			&cse_dev->base->cse_param[2]);
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, m4),
			&cse_dev->base->cse_param[3]);
	writel(cse_dev->hw_desc_phys + offsetof(cse_desc_t, m5),
			&cse_dev->base->cse_param[4]);
	writel(CSE_CMD_LOAD_KEY, &cse_dev->base->cse_cmd);
}

/**
 * Generic CSE hardware request
 * Does some common verifications/initialization
 * and calls the associated request function
 */
int cse_hw_comm(struct cse_device_data *dev, uint32_t flags, int phase)
{
	int res = 0, key_id = CSE_KEYID_RAM;
	uint32_t ctrl = readl(&dev->base->cse_cr);

	/* Shouldn't be busy at this point */
	if (readl(&dev->base->cse_sr) & CSE_SR_BSY)
		return -EBUSY;

	/* Two-phase command, first we need to load the plaintext key */
	if (flags & (FLAG_ENC|FLAG_DEC|FLAG_GEN_MAC|FLAG_VER_MAC)
			&& IS_LOAD_KEY_PHASE(phase)) {
		cse_cmd_load_plainkey(dev);
		return 0;
	}

	if (flags & FLAG_WITH_KEY)
		key_id = dev->req->key_id;

	/* Temporary enable second key bank if needed  */
	if (flags & FLAG_KBS)
		writel(ctrl | CSE_CR_KBS, &dev->base->cse_cr);
	else
		writel(ctrl & ~CSE_CR_KBS, &dev->base->cse_cr);

	switch (flags & ALG_MASK) {
	case FLAG_ENC:
		cse_cmd_enc(dev, key_id, flags & FLAG_CBC);
		break;
	case FLAG_DEC:
		cse_cmd_dec(dev, key_id, flags & FLAG_CBC);
		break;
	case FLAG_GEN_MAC:
		cse_cmd_gen_mac(dev, key_id);
		break;
	case FLAG_VER_MAC:
		cse_cmd_ver_mac(dev, key_id, 0);
		break;
	case FLAG_RND:
		cse_cmd_rnd(dev);
		break;
	case FLAG_LOAD_PLKEY:
		cse_cmd_load_plainkey(dev);
		break;
	case FLAG_MP_COMP:
		cse_cmd_mp_comp(dev);
		break;
	case FLAG_LOAD_KEY:
		cse_cmd_load_key(dev);
		break;
	default:
		res = -EINVAL;
		break;
	}

	return res;
}
