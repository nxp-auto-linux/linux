/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - Core
 *
 * This file defines the driver core interface for the HSE cryptographic engine.
 *
 * Copyright 2019-2021 NXP
 */

#ifndef HSE_CORE_H
#define HSE_CORE_H

#define HSE_CRA_PRIORITY    2000u /* HSE crypto algorithm priority */

#define HSE_CHANNEL_ANY    0xACu /* use any channel, no request ordering */
#define HSE_CHANNEL_ADM    0u /* channel reserved for administrative services */

/**
 * enum hse_ch_type - channel type
 * @HSE_CHANNEL_ADMIN: restricted to administrative services
 * @HSE_CHANNEL_SHARED: shared channel, available for crypto
 * @HSE_CHANNEL_STREAM: reserved for streaming mode use
 */
enum hse_ch_type {
	HSE_CH_TYPE_ADMIN = 0u,
	HSE_CH_TYPE_SHARED = 1u,
	HSE_CH_TYPE_STREAM = 2u,
};

/**
 * enum hse_alg_type - algorithm type
 * @HSE_ALG_TYPE_AEAD: AEAD
 * @HSE_ALG_TYPE_AUTHENC: authenticated encryption
 * @HSE_ALG_TYPE_KEYWRAP: key wrapping/blobbing
 */
enum hse_alg_type {
	HSE_ALG_TYPE_AEAD = 0u,
	HSE_ALG_TYPE_AUTHENC = 1u,
	HSE_ALG_TYPE_KEYWRAP = 2u,
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

int hse_channel_acquire(struct device *dev, enum hse_ch_type type, u8 *channel,
			u8 *stream_id);
int hse_channel_release(struct device *dev, u8 channel);

int hse_srv_req_async(struct device *dev, u8 channel, void *srv_desc,
		      void *ctx, void (*rx_cbk)(int err, void *ctx));
int hse_srv_req_sync(struct device *dev, u8 channel, void *srv_desc);

void hse_ahash_register(struct device *dev, struct list_head *alg_list);
void hse_ahash_unregister(struct list_head *alg_list);

void hse_skcipher_register(struct device *dev, struct list_head *alg_list);
void hse_skcipher_unregister(struct list_head *alg_list);

void hse_aead_register(struct device *dev, struct list_head *alg_list);
void hse_aead_unregister(struct list_head *alg_list);

void hse_hwrng_register(struct device *dev);
void hse_hwrng_unregister(struct device *dev);

u32 _get_rng_srv_id(struct device *dev);

#endif /* HSE_CORE_H */
