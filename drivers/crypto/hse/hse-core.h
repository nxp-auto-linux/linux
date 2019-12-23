/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - Core
 *
 * This file defines the driver core interface for the HSE cryptographic engine.
 *
 * Copyright 2019 NXP
 */

#ifndef HSE_CORE_H
#define HSE_CORE_H

#include <crypto/aes.h>

#define HSE_NUM_CHANNELS    16u /* number of available service channels */
#define HSE_STREAM_COUNT    2u /* number of usable streams per MU instance */

#define HSE_CRA_PRIORITY    2000u /* HSE crypto algorithm priority */

#define HSE_CHANNEL_ANY    0xACu /* use any channel, no request ordering */
#define HSE_CHANNEL_INV    0xFFu /* invalid acquired service channel index */

/**
 * enum hse_ch_type - channel type
 * @HSE_CHANNEL_SHARED: shared channel
 * @HSE_CHANNEL_STREAM: stream
 */
enum hse_ch_type {
	HSE_CH_TYPE_SHARED = 0u,
	HSE_CH_TYPE_STREAM = 1u,
};

/**
 * struct hse_key - HSE key slot
 * @entry: list position
 * @handle: key handle
 * @type: key type
 */
struct hse_key {
	struct list_head entry;
	u32 handle;
	enum hse_key_type type;
};

struct hse_key *hse_key_slot_acquire(struct device *dev,
				     enum hse_key_type type);
void hse_key_slot_release(struct device *dev, struct hse_key *slot);

int hse_channel_acquire(struct device *dev, enum hse_ch_type type, u8 *channel);
int hse_channel_release(struct device *dev, u8 channel);

int hse_srv_req_async(struct device *dev, u8 channel, dma_addr_t srv_desc,
		      void *ctx, void (*rx_cbk)(int err, void *ctx));
int hse_srv_req_sync(struct device *dev, u8 channel, dma_addr_t srv_desc);

void hse_ahash_register(struct device *dev);
void hse_ahash_unregister(struct device *dev);

void hse_skcipher_register(struct device *dev);
void hse_skcipher_unregister(void);

void hse_aead_register(struct device *dev);
void hse_aead_unregister(void);

void hse_hwrng_register(struct device *dev);

#endif /* HSE_CORE_H */
