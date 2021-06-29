// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Core
 *
 * This file contains the device driver core for the HSE cryptographic engine.
 *
 * Copyright 2019-2021 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/dma-mapping.h>
#include <linux/crypto.h>

#include "hse-abi.h"
#include "hse-core.h"
#include "hse-mu.h"

#if !defined(CONFIG_CRYPTO_DEV_NXP_HSE_UIO)
#define HSE_KS_RAM_AES_GID       CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GROUP_ID
#define HSE_KS_RAM_AES_GSIZE     CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GROUP_SIZE
#define HSE_KS_RAM_HMAC_GID      CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GROUP_ID
#define HSE_KS_RAM_HMAC_GSIZE    CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GROUP_SIZE
#else
#define HSE_KS_RAM_AES_GID       0u
#define HSE_KS_RAM_AES_GSIZE     0u
#define HSE_KS_RAM_HMAC_GID      0u
#define HSE_KS_RAM_HMAC_GSIZE    0u
#endif /* CONFIG_CRYPTO_DEV_NXP_HSE_UIO */

/**
 * struct hse_drvdata - HSE driver private data
 * @srv_desc[n].ptr: service descriptor virtual address for channel n
 * @srv_desc[n].dma: service descriptor DMA address for channel n
 * @srv_desc[n].id: current service request ID for channel n
 * @ahash_algs: registered hash and hash-based MAC algorithms
 * @skcipher_algs: registered symmetric key cipher algorithms
 * @aead_algs: registered authenticated encryption and AEAD algorithms
 * @mu: MU instance handle returned by lower abstraction layer
 * @uio: user-space I/O device handle
 * @channel_busy[n]: internally cached status of MU channel n
 * @refcnt[n]: service channel n acquired reference counter
 * @type[n]: designated type of service channel n
 * @rx_cbk[n].fn: upper layer RX callback for channel n
 * @rx_cbk[n].ctx: context passed to the RX callback on channel n
 * @sync[n].done: completion for synchronous requests on channel n
 * @sync[n].reply: decoded service response location for channel n
 * @stream_lock: lock used for stream channel reservation
 * @tx_lock: lock used for service request transmission
 * @hmac_key_ring: HMAC key slots currently available
 * @aes_key_ring: AES key slots currently available
 * @key_ring_lock: lock used for key slot acquisition
 */
struct hse_drvdata {
	struct {
		struct hse_srv_desc *ptr;
		dma_addr_t dma;
		u32 id;
	} srv_desc[HSE_NUM_CHANNELS];
	struct list_head ahash_algs;
	struct list_head skcipher_algs;
	struct list_head aead_algs;
	void *mu;
	void *uio;
	bool channel_busy[HSE_NUM_CHANNELS];
	atomic_t refcnt[HSE_NUM_CHANNELS];
	enum hse_ch_type type[HSE_NUM_CHANNELS];
	struct {
		void (*fn)(int err, void *ctx);
		void *ctx;
	} rx_cbk[HSE_NUM_CHANNELS];
	struct {
		struct completion *done;
		int *reply;
	} sync[HSE_NUM_CHANNELS];
	spinlock_t stream_lock; /* covers stream reservation */
	spinlock_t tx_lock; /* covers request transmission */
	struct list_head hmac_key_ring;
	struct list_head aes_key_ring;
	spinlock_t key_ring_lock; /* covers key slot acquisition */
};

/**
 * hse_print_fw_version - print firmware version
 * @dev: HSE device
 *
 * Get firmware version attribute from HSE and print it. Attribute buffer is
 * encoded into the descriptor to get around HSE memory access limitations.
 */
static void hse_print_fw_version(struct device *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	struct hse_srv_desc srv_desc;
	struct hse_attr_fw_version *fw_ver;
	unsigned int fw_ver_offset;
	int err;

	/* place attribute right after descriptor */
	fw_ver_offset = offsetof(struct hse_srv_desc, get_attr_req) +
			sizeof(struct hse_get_attr_srv);
	fw_ver = (void *)drv->srv_desc[HSE_CHANNEL_ADM].ptr + fw_ver_offset;

	srv_desc.srv_id = HSE_SRV_ID_GET_ATTR;
	srv_desc.get_attr_req.attr_id = HSE_FW_VERSION_ATTR_ID;
	srv_desc.get_attr_req.attr_len = sizeof(*fw_ver);
	srv_desc.get_attr_req.attr = drv->srv_desc[HSE_CHANNEL_ADM].dma +
				     fw_ver_offset;

	err = hse_srv_req_sync(dev, HSE_CHANNEL_ADM, &srv_desc);
	if (unlikely(err)) {
		dev_dbg(dev, "%s: request failed: %d\n", __func__, err);
		return;
	}

	dev_info(dev, "firmware type %d, version %d.%d.%d\n", fw_ver->fw_type,
		 fw_ver->major, fw_ver->minor, fw_ver->patch);
}

/**
 * hse_key_ring_init - initialize all keys in a specific key group
 * @dev: HSE device
 * @key_ring: output key ring
 * @group_id: key group ID
 * @group_size: key group size
 *
 * Return: 0 on success, -ENOMEM for failed key ring allocation
 */
static int hse_key_ring_init(struct device *dev, struct list_head *key_ring,
			     enum hse_key_type type, u8 group_id, u8 group_size)
{
	struct hse_key *ring;
	unsigned int i;

	/* skip init for zero size */
	if (unlikely(!group_size))
		return 0;

	ring = devm_kmalloc_array(dev, group_size, sizeof(*ring), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ring))
		return -ENOMEM;

	INIT_LIST_HEAD(key_ring);

	for (i = 0; i < group_size; i++) {
		ring[i].handle = HSE_KEY_HANDLE(group_id, i);
		ring[i].type = type;
		list_add_tail(&ring[i].entry, key_ring);
	}

	dev_dbg(dev, "key ring type 0x%02x: group id %d, size %d\n", type,
		group_id, group_size);

	return 0;
}

/**
 * hse_key_ring_free - remove all keys in a specific key group
 * @key_ring: input key ring
 */
static void hse_key_ring_free(struct list_head *key_ring)
{
	struct hse_key *key, *tmp;

	if (unlikely(!key_ring))
		return;

	list_for_each_entry_safe(key, tmp, key_ring, entry)
		list_del(&key->entry);
}

/**
 * hse_key_slot_acquire - acquire a HSE key slot
 * @dev: HSE device
 * @type: key type
 *
 * Return: key slot of specified type if available, error code otherwise
 */
struct hse_key *hse_key_slot_acquire(struct device *dev, enum hse_key_type type)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	struct list_head *key_ring;
	struct hse_key *slot;

	if (unlikely(!dev))
		return ERR_PTR(-EINVAL);

	switch (type) {
	case HSE_KEY_TYPE_AES:
		key_ring = &drv->aes_key_ring;
		break;
	case HSE_KEY_TYPE_HMAC:
		key_ring = &drv->hmac_key_ring;
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	/* remove key slot from ring */
	spin_lock(&drv->key_ring_lock);
	slot = list_first_entry_or_null(key_ring, struct hse_key, entry);
	if (IS_ERR_OR_NULL(slot)) {
		spin_unlock(&drv->key_ring_lock);
		dev_dbg(dev, "failed to acquire key slot, type 0x%02x\n", type);
		return ERR_PTR(-ENOKEY);
	}
	list_del(&slot->entry);
	spin_unlock(&drv->key_ring_lock);

	return slot;
}

/**
 * hse_key_slot_release - release a HSE key slot
 * @dev: HSE device
 * @slot: key slot
 */
void hse_key_slot_release(struct device *dev, struct hse_key *slot)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	struct list_head *key_ring;

	switch (slot->type) {
	case HSE_KEY_TYPE_AES:
		key_ring = &drv->aes_key_ring;
		break;
	case HSE_KEY_TYPE_HMAC:
		key_ring = &drv->hmac_key_ring;
		break;
	default:
		return;
	}

	/* add key slot back to ring */
	spin_lock(&drv->key_ring_lock);
	list_add_tail(&slot->entry, key_ring);
	spin_unlock(&drv->key_ring_lock);
}

/**
 * hse_manage_channels - manage channels and descriptor space
 * @dev: HSE device
 * @desc_base_ptr: descriptor base virtual address
 * @desc_base_dma: descriptor base DMA address
 *
 * HSE firmware restricts channel zero to administrative services, all the rest
 * are usable for crypto operations. Driver reserves the last HSE_STREAM_COUNT
 * channels for streaming mode use and marks the remaining as shared channels.
 */
static inline void hse_manage_channels(struct device *dev, void *desc_base_ptr,
				       u64 desc_base_dma)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	u8 channel;

	if (unlikely(!dev || !desc_base_ptr || !desc_base_dma))
		return;

	/* set channel type */
	drv->type[0] = HSE_CH_TYPE_ADMIN;
	for (channel = 1; channel < HSE_NUM_CHANNELS; channel++)
		if (channel >= HSE_NUM_CHANNELS - HSE_STREAM_COUNT)
			drv->type[channel] = HSE_CH_TYPE_STREAM;
		else
			drv->type[channel] = HSE_CH_TYPE_SHARED;

	/* manage descriptor space */
	for (channel = 0; channel < HSE_NUM_CHANNELS; channel++) {
		drv->srv_desc[channel].ptr = desc_base_ptr +
					     channel * HSE_SRV_DESC_MAX_SIZE;
		drv->srv_desc[channel].dma = desc_base_dma +
					     channel * HSE_SRV_DESC_MAX_SIZE;
	}
}

/**
 * hse_sync_srv_desc - sync service descriptor
 * @dev: HSE device
 * @channel: service channel
 * @desc: service descriptor address
 *
 * Copy descriptor to the dedicated space and cache service ID internally.
 */
static inline void hse_sync_srv_desc(struct device *dev, u8 channel,
				     struct hse_srv_desc *srv_desc)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);

	if (unlikely(!dev || channel >= HSE_NUM_CHANNELS || !srv_desc))
		return;

	memcpy(drv->srv_desc[channel].ptr, srv_desc, sizeof(*srv_desc));
	drv->srv_desc[channel].id = srv_desc->srv_id;
}

/**
 * hse_err_decode - HSE error code translation
 * @srv_rsp: HSE service response
 *
 * Return: 0 on service request success, error code otherwise
 */
static inline int hse_err_decode(u32 srv_rsp)
{
	switch (srv_rsp) {
	case HSE_SRV_RSP_OK:
		return 0;
	case HSE_SRV_RSP_VERIFY_FAILED:
		return -EBADMSG;
	case HSE_SRV_RSP_INVALID_ADDR:
	case HSE_SRV_RSP_INVALID_PARAM:
		return -EBADR;
	case HSE_SRV_RSP_NOT_SUPPORTED:
		return -EOPNOTSUPP;
	case HSE_SRV_RSP_NOT_ALLOWED:
		return -EPERM;
	case HSE_SRV_RSP_NOT_ENOUGH_SPACE:
		return -ENOMEM;
	case HSE_SRV_RSP_KEY_NOT_AVAILABLE:
	case HSE_SRV_RSP_KEY_EMPTY:
		return -ENOKEY;
	case HSE_SRV_RSP_KEY_INVALID:
	case HSE_SRV_RSP_KEY_WRITE_PROTECTED:
	case HSE_SRV_RSP_KEY_UPDATE_ERROR:
		return -EKEYREJECTED;
	case HSE_SRV_RSP_CANCELED:
		return -ECANCELED;
	default:
		return -EFAULT;
	}
}

/**
 * hse_next_free_channel - find the next available shared channel
 * @dev: HSE device
 * @type: channel type
 *
 * Return: channel index, HSE_CHANNEL_INV if none available
 */
static u8 hse_next_free_channel(struct device *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	u8 channel;

	for (channel = 0; channel < HSE_NUM_CHANNELS; channel++)
		switch (drv->type[channel]) {
		case HSE_CH_TYPE_STREAM:
			if (atomic_read(&drv->refcnt[channel]))
				continue;
			fallthrough;
		case HSE_CH_TYPE_SHARED:
			if (!drv->channel_busy[channel])
				return channel;
			continue;
		default:
			continue;
		}

	return HSE_CHANNEL_INV;
}

/**
 * hse_channel_acquire - acquire a stream or a shared channel (non-blocking)
 * @dev: HSE device
 * @type: channel type
 * @channel: service channel index
 * @stream_id: stream ID (ignored for HSE_CH_TYPE_SHARED)
 *
 * Acquire an appropriate type channel and return its index, and also the
 * corresponding stream ID, if applicable. For HSE_CH_TYPE_STREAM, the channel
 * will be reserved entirely until release. The last HSE_STREAM_COUNT channels
 * are allocated for streaming mode use by the driver and will only accept
 * single one-shot requests when not acquired. For HSE_CH_TYPE_SHARED, return
 * the shared channel with the lowest reference count.Shared channels can be
 * acquired simultaneously and should be used if the request order preservation
 * is a concern, as this is not guaranteed by simply using HSE_CHANNEL_ANY.

 * Return: 0 on success, -EINVAL for invalid parameter, -EBUSY for no stream
 *         type channel/resource currently available
 */
int hse_channel_acquire(struct device *dev, enum hse_ch_type type, u8 *channel,
			u8 *stream_id)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	unsigned int crt = 0, min = UINT_MAX;

	if (unlikely(!dev || !channel))
		return -EINVAL;

	*channel = HSE_CHANNEL_INV;

	switch (type) {
	case HSE_CH_TYPE_STREAM:
		if (unlikely(!stream_id))
			return -EINVAL;

		*stream_id = HSE_STREAM_COUNT;
		spin_lock(&drv->stream_lock);

		/* find an available stream type channel */
		for (crt = 0; crt < HSE_NUM_CHANNELS; crt++)
			if (drv->type[crt] == HSE_CH_TYPE_STREAM &&
			    atomic_read(&drv->refcnt[crt]) == 0 &&
			    !drv->channel_busy[crt]) {
				*channel = crt;
				break;
			}
		if (*channel == HSE_CHANNEL_INV) {
			spin_unlock(&drv->stream_lock);
			dev_dbg(dev, "%s: no type %d channel available\n",
				__func__, type);
			return -EBUSY;
		}

		/* mark channel reserved */
		atomic_set(&drv->refcnt[*channel], 1);
		spin_unlock(&drv->stream_lock);

		/* allocate a stream ID */
		*stream_id = *channel + HSE_STREAM_COUNT - HSE_NUM_CHANNELS;
		break;
	case HSE_CH_TYPE_SHARED:
		/* find the shared channel with lowest refcount */
		for (crt = 0; min > 0 && crt < HSE_NUM_CHANNELS; crt++)
			if (drv->type[crt] == HSE_CH_TYPE_SHARED &&
			    atomic_read(&drv->refcnt[crt]) < min) {
				*channel = crt;
				min = atomic_read(&drv->refcnt[crt]);
			}

		/* increment channel refcount */
		atomic_inc(&drv->refcnt[*channel]);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * hse_channel_release - release a stream or a shared channel (non-blocking)
 * @dev: HSE device
 * @channel: service channel index
 *
 * Mark stream as released or decrement the reference count of shared channel.
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range
 */
int hse_channel_release(struct device *dev, u8 channel)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);

	if (unlikely(!dev))
		return -EINVAL;

	if (channel >= HSE_NUM_CHANNELS)
		return -ECHRNG;

	switch (drv->type[channel]) {
	case HSE_CH_TYPE_STREAM:
		atomic_set(&drv->refcnt[channel], 0);
		break;
	case HSE_CH_TYPE_SHARED:
		atomic_dec_if_positive(&drv->refcnt[channel]);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * hse_srv_req_async - send an asynchronous service request (non-blocking)
 * @dev: HSE device
 * @channel: service channel index
 * @srv_desc: service descriptor
 * @ctx: context passed to RX callback
 * @rx_cbk: upper layer RX callback
 *
 * Send a HSE service request on the selected channel and register a callback
 * function to be executed asynchronously upon completion. The channel index
 * must be set to HSE_CHANNEL_ANY unless obtained via hse_channel_acquire().
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -EBUSY for channel busy or none available
 */
int hse_srv_req_async(struct device *dev, u8 channel, void *srv_desc,
		      void *ctx, void (*rx_cbk)(int err, void *ctx))
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	int err;

	if (unlikely(!dev || !rx_cbk || !ctx))
		return -EINVAL;

	if (unlikely(channel != HSE_CHANNEL_ANY && channel >= HSE_NUM_CHANNELS))
		return -ECHRNG;

	spin_lock(&drv->tx_lock);

	if (channel == HSE_CHANNEL_ANY) {
		channel = hse_next_free_channel(dev);
		if (unlikely(channel == HSE_CHANNEL_INV)) {
			spin_unlock(&drv->tx_lock);
			dev_dbg(dev, "%s: no channel available\n", __func__);
			return -EBUSY;
		}
	} else if (drv->channel_busy[channel]) {
		spin_unlock(&drv->tx_lock);
		dev_dbg(dev, "%s: channel %d busy\n", __func__, channel);
		return -EBUSY;
	}

	drv->rx_cbk[channel].fn = rx_cbk;
	drv->rx_cbk[channel].ctx = ctx;

	hse_sync_srv_desc(dev, channel, srv_desc);

	err = hse_mu_msg_send(drv->mu, channel, drv->srv_desc[channel].dma);
	if (unlikely(err)) {
		spin_unlock(&drv->tx_lock);
		return err;
	}

	drv->channel_busy[channel] = true;

	spin_unlock(&drv->tx_lock);

	return err;
}

/**
 * hse_srv_req_sync - issue a synchronous service request (blocking)
 * @dev: HSE device
 * @channel: service channel index
 * @srv_desc: service descriptor
 *
 * Send a HSE service descriptor on the selected channel and block until the
 * HSE response becomes available, then read the reply. The channel index
 * shall be set to HSE_CHANNEL_ANY unless obtained via hse_channel_acquire().
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -EBUSY for channel busy or none available
 */
int hse_srv_req_sync(struct device *dev, u8 channel, void *srv_desc)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	DECLARE_COMPLETION_ONSTACK(done);
	int err, reply;

	if (unlikely(!dev))
		return -EINVAL;

	if (unlikely(channel != HSE_CHANNEL_ANY && channel >= HSE_NUM_CHANNELS))
		return -ECHRNG;

	spin_lock(&drv->tx_lock);

	if (channel == HSE_CHANNEL_ANY) {
		channel = hse_next_free_channel(dev);
		if (channel == HSE_CHANNEL_INV) {
			spin_unlock(&drv->tx_lock);
			dev_dbg(dev, "%s: no channel available\n", __func__);
			return -EBUSY;
		}
	} else if (drv->channel_busy[channel]) {
		spin_unlock(&drv->tx_lock);
		dev_dbg(dev, "%s: channel %d busy\n", __func__, channel);
		return -EBUSY;
	}

	drv->sync[channel].done = &done;
	drv->sync[channel].reply = &reply;

	hse_sync_srv_desc(dev, channel, srv_desc);

	err = hse_mu_msg_send(drv->mu, channel, drv->srv_desc[channel].dma);
	if (unlikely(err)) {
		spin_unlock(&drv->tx_lock);
		return err;
	}

	drv->channel_busy[channel] = true;

	spin_unlock(&drv->tx_lock);

	err = wait_for_completion_interruptible(&done);
	if (err) {
		drv->sync[channel].done = NULL;
		dev_dbg(dev, "%s: request interrupted: %d\n", __func__, err);
		return err;
	}

	return reply;
}

/**
 * hse_srv_rsp_dispatch - handle service response on selected channel
 * @dev: HSE device
 * @channel: service channel index
 *
 * For a pending service response, execute the upper layer callback in case
 * of an asynchronous request or signal completion of a synchronous request.
 */
static void hse_srv_rsp_dispatch(struct device *dev, u8 channel)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	u32 srv_rsp;
	int err;

	err = hse_mu_msg_recv(drv->mu, channel, &srv_rsp);
	if (err) {
		dev_err(dev, "%s: failed to read response on channel %d\n",
			__func__, channel);
		return;
	}

	err = hse_err_decode(srv_rsp);
	if (err)
		dev_dbg(dev, "%s: service response 0x%08X on channel %d\n",
			__func__, srv_rsp, channel);

	/* when UIO support is enabled, let upper layer handle the reply */
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_UIO) && likely(drv->uio)) {
		hse_uio_notify(drv->uio, channel, srv_rsp);
		return;
	}

	if (drv->rx_cbk[channel].fn) {
		drv->rx_cbk[channel].fn(err, drv->rx_cbk[channel].ctx);
		drv->rx_cbk[channel].fn = NULL;
	} else if (drv->sync[channel].done) {
		*drv->sync[channel].reply = err;
		wmb(); /* ensure reply is written before calling complete */

		complete(drv->sync[channel].done);
		drv->sync[channel].done = NULL;
	}

	drv->channel_busy[channel] = false;
}

/**
 * hse_rx_dispatcher - deferred handler for HSE_INT_RESPONSE type interrupts
 * @irq: interrupt line
 * @dev: HSE device
 */
static irqreturn_t hse_rx_dispatcher(int irq, void *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	u8 channel = hse_mu_next_pending_channel(drv->mu);

	while (channel != HSE_CHANNEL_INV) {
		hse_srv_rsp_dispatch(dev, channel);

		channel = hse_mu_next_pending_channel(drv->mu);
	}

	return IRQ_HANDLED;
}

/**
 * hse_event_dispatcher - deferred handler for HSE_INT_SYS_EVENT type interrupts
 * @irq: interrupt line
 * @dev: HSE device
 *
 * In case a fatal error has been reported, all MU interfaces are disabled
 * and communication with HSE terminated. Therefore, all service requests
 * currently in progress are canceled and any further requests are prevented.
 */
static irqreturn_t hse_event_dispatcher(int irq, void *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	u32 event = hse_mu_check_event(drv->mu);
	u8 channel;

	if (event & HSE_EVT_MASK_WARN) {
		dev_warn(dev, "warning, event mask 0x%08x\n", event);
		hse_mu_irq_clear(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_WARN);

		return IRQ_HANDLED;
	}

	/* disable RX and event notifications */
	hse_mu_irq_disable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_ALL);

	dev_crit(dev, "fatal error, event mask 0x%08x\n", event);
	hse_mu_irq_clear(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_ERR);

	/* process pending requests, if any */
	channel = hse_mu_next_pending_channel(drv->mu);
	while (channel != HSE_CHANNEL_INV) {
		hse_srv_rsp_dispatch(dev, channel);

		channel = hse_mu_next_pending_channel(drv->mu);
	}

	/* notify upper layers that all requests are canceled */
	for (channel = 0; channel < HSE_NUM_CHANNELS; channel++) {
		if (drv->rx_cbk[channel].fn) {
			void *ctx = drv->rx_cbk[channel].ctx;

			drv->rx_cbk[channel].fn(-ECANCELED, ctx);
			drv->rx_cbk[channel].fn = NULL;
		} else if (drv->sync[channel].done) {
			*drv->sync[channel].reply = -ECANCELED;
			wmb(); /* ensure reply is written before complete */

			complete(drv->sync[channel].done);
			drv->sync[channel].done = NULL;
		}

		if (drv->channel_busy[channel])
			dev_dbg(dev, "request id 0x%08x canceled, channel %d\n",
				channel, drv->srv_desc[channel].id);
		drv->channel_busy[channel] = true;
	}

	/* unregister hwrng */
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_HWRNG))
		hse_hwrng_unregister(dev);

	dev_crit(dev, "communication terminated, reset system to recover\n");

	return IRQ_HANDLED;
}

static int hse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hse_drvdata *drv;
	void *desc_base_ptr;
	u64 desc_base_dma;
	u16 status;
	int err;

	dev_dbg(dev, "probing driver\n");

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (IS_ERR_OR_NULL(drv))
		return -ENOMEM;
	platform_set_drvdata(pdev, drv);

	/* MU interface setup */
	drv->mu = hse_mu_init(dev, &desc_base_ptr, &desc_base_dma,
			      hse_rx_dispatcher, hse_event_dispatcher);
	if (IS_ERR(drv->mu)) {
		dev_dbg(dev, "failed to initialize MU communication\n");
		return PTR_ERR(drv->mu);
	}

	/* check firmware status */
	status = hse_mu_check_status(drv->mu);
	if (!likely(status & HSE_STATUS_INIT_OK)) {
		dev_err(dev, "firmware not found\n");
		return -ENODEV;
	}

	/* manage channels and descriptor space */
	hse_manage_channels(dev, desc_base_ptr, desc_base_dma);

	/* initialize locks */
	spin_lock_init(&drv->stream_lock);
	spin_lock_init(&drv->tx_lock);
	spin_lock_init(&drv->key_ring_lock);

	/* enable RX and event notifications */
	hse_mu_irq_enable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_enable(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_WARN);
	hse_mu_irq_enable(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_ERR);

	/* check firmware version */
	hse_print_fw_version(dev);

	/* check HSE global status */
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_HWRNG) &&
	    !likely(status & HSE_STATUS_RNG_INIT_OK)) {
		dev_err(dev, "RNG not initialized\n");
		err = -ENODEV;
		goto err_probe_failed;
	}
	if (!IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_UIO) &&
	    !likely(status & HSE_STATUS_INSTALL_OK)) {
		dev_err(dev, "key catalogs not formatted\n");
		err = -ENODEV;
		goto err_probe_failed;
	}
	if (unlikely(status & HSE_STATUS_PUBLISH_SYS_IMAGE))
		dev_warn(dev, "volatile configuration, publish SYS_IMAGE\n");

	/* register UIO device */
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_UIO)) {
		drv->uio = hse_uio_register(dev, drv->mu);
		if (IS_ERR_OR_NULL(drv->uio)) {
			dev_err(dev, "failed to register UIO device\n");
			return PTR_ERR(drv->uio);
		}
		return 0;
	}

	/* initialize key rings */
	err = hse_key_ring_init(dev, &drv->hmac_key_ring, HSE_KEY_TYPE_HMAC,
				HSE_KS_RAM_HMAC_GID, HSE_KS_RAM_HMAC_GSIZE);
	if (unlikely(err))
		goto err_probe_failed;

	err = hse_key_ring_init(dev, &drv->aes_key_ring, HSE_KEY_TYPE_AES,
				HSE_KS_RAM_AES_GID, HSE_KS_RAM_AES_GSIZE);
	if (unlikely(err))
		goto err_probe_failed;

	/* register kernel crypto algorithms */
	hse_ahash_register(dev, &drv->ahash_algs);
	hse_skcipher_register(dev, &drv->skcipher_algs);
	hse_aead_register(dev, &drv->aead_algs);

	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_HWRNG))
		hse_hwrng_register(dev);

	dev_info(dev, "device ready, status 0x%04X\n", status);

	return 0;
err_probe_failed:
	dev_err(dev, "probe failed with status 0x%04X\n", status);
	return err;
}

static int hse_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hse_drvdata *drv = dev_get_drvdata(dev);

	/* disable RX and event notifications */
	hse_mu_irq_disable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_ALL);

	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_UIO))
		return 0;

	/* unregister algorithms */
	hse_ahash_unregister(&drv->ahash_algs);
	hse_skcipher_unregister(&drv->skcipher_algs);
	hse_aead_unregister(&drv->aead_algs);

	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_HWRNG))
		hse_hwrng_unregister(dev);

	/* empty used key rings */
	hse_key_ring_free(&drv->aes_key_ring);
	hse_key_ring_free(&drv->hmac_key_ring);

	dev_dbg(dev, "device removed\n");

	return 0;
}

static const struct of_device_id hse_of_match[] = {
	{
		.name = HSE_MU_INST,
		.compatible = "fsl,s32gen1-hse",
	}, {}
};
MODULE_DEVICE_TABLE(of, hse_of_match);

static struct platform_driver hse_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table	= hse_of_match,
	},
	.probe = hse_probe,
	.remove = hse_remove,
};

module_platform_driver(hse_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS_CRYPTO(KBUILD_MODNAME);
MODULE_DESCRIPTION("NXP Hardware Security Engine (HSE) Driver");
