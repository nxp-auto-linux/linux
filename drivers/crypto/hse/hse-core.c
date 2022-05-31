// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Core
 *
 * This file contains the device driver core for the HSE cryptographic engine.
 *
 * Copyright 2019-2022 NXP
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

/**
 * enum hse_fw_status - HSE firmware status
 * @HSE_FW_SHUTDOWN: firmware not initialized or shut down due to fatal error
 * @HSE_FW_RUNNING: firmware running and able to service any type of request
 * @HSE_FW_STANDBY: firmware considered in stand-by state, no crypto requests
 */
enum hse_fw_status {
	HSE_FW_SHUTDOWN = 0u,
	HSE_FW_RUNNING = 1u,
	HSE_FW_STANDBY = 2u,
};

/**
 * struct hse_drvdata - HSE driver private data
 * @srv_desc[n].ptr: service descriptor virtual address for channel n
 * @srv_desc[n].dma: service descriptor DMA address for channel n
 * @srv_desc[n].id: current service request ID for channel n
 * @ahash_algs: registered hash and hash-based MAC algorithms
 * @skcipher_algs: registered symmetric key cipher algorithms
 * @aead_algs: registered authenticated encryption and AEAD algorithms
 * @mu: MU instance handle returned by lower abstraction layer
 * @channel_busy[n]: internally cached status of MU channel n
 * @refcnt[n]: service channel n acquired reference counter
 * @type[n]: designated type of service channel n
 * @rx_cbk[n].fn: upper layer RX callback for channel n
 * @rx_cbk[n].ctx: context passed to the RX callback on channel n
 * @sync[n].done: completion for synchronous requests on channel n
 * @sync[n].reply: decoded service response location for channel n
 * @hmac_key_ring: HMAC key slots currently available
 * @aes_key_ring: AES key slots currently available
 * @stream_lock: lock used for stream channel reservation
 * @tx_lock: lock used for service request transmission
 * @rx_lock: lock used for synchronous request completion
 * @key_ring_lock: lock used for key slot acquisition
 * @firmware_version: firmware version attribute structure
 * @firmware_status: internally cached status of HSE firmware
 */
struct hse_drvdata {
	struct {
		void __iomem *ptr;
		dma_addr_t dma;
		u32 id;
	} srv_desc[HSE_NUM_CHANNELS];
	struct list_head ahash_algs;
	struct list_head skcipher_algs;
	struct list_head aead_algs;
	void *mu;
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
	struct list_head hmac_key_ring;
	struct list_head aes_key_ring;
	spinlock_t stream_lock; /* covers stream reservation */
	spinlock_t tx_lock; /* covers request transmission */
	spinlock_t rx_lock; /* covers request completion */
	spinlock_t key_ring_lock; /* covers key slot acquisition */
	struct hse_attr_fw_version firmware_version;
	enum hse_fw_status firmware_status;
};

/**
 * hse_check_fw_version - retrieve firmware version
 * @dev: HSE device
 */
static int hse_check_fw_version(struct device *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	dma_addr_t firmware_version_dma;
	struct hse_srv_desc srv_desc;
	int err = 0;

	firmware_version_dma = dma_map_single(dev, &drv->firmware_version,
					      sizeof(drv->firmware_version),
					      DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(dev, firmware_version_dma)))
		return -ENOMEM;

	srv_desc.srv_id = HSE_SRV_ID_GET_ATTR;
	srv_desc.get_attr_req.attr_id = HSE_FW_VERSION_ATTR_ID;
	srv_desc.get_attr_req.attr_len = sizeof(drv->firmware_version);
	srv_desc.get_attr_req.attr = firmware_version_dma;

	err = hse_srv_req_sync(dev, HSE_CHANNEL_ADM, &srv_desc);
	if (unlikely(err))
		dev_dbg(dev, "%s: request failed: %d\n", __func__, err);

	dma_unmap_single(dev, firmware_version_dma,
			 sizeof(drv->firmware_version), DMA_FROM_DEVICE);
	return err;
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

	dev_dbg(dev, "%s key ring: group id %d, size %d\n",
		type == HSE_KEY_TYPE_AES ? "aes" : "hmac",
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
 * hse_config_channels - configure channels and manage descriptor space
 * @dev: HSE device
 *
 * HSE firmware restricts channel zero to administrative services, all the rest
 * are usable for crypto operations. Driver reserves the last HSE_STREAM_COUNT
 * channels for streaming mode use and marks the remaining as shared channels.
 */
static inline void hse_config_channels(struct device *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	unsigned int offset;
	u8 channel;

	if (unlikely(!dev))
		return;

	drv->type[0] = HSE_CH_TYPE_ADMIN;
	drv->srv_desc[0].ptr = hse_mu_desc_base_ptr(drv->mu);
	drv->srv_desc[0].dma = hse_mu_desc_base_dma(drv->mu);

	for (channel = 1; channel < HSE_NUM_CHANNELS; channel++) {
		if (channel >= HSE_NUM_CHANNELS - HSE_STREAM_COUNT)
			drv->type[channel] = HSE_CH_TYPE_STREAM;
		else
			drv->type[channel] = HSE_CH_TYPE_SHARED;

		offset = channel * HSE_SRV_DESC_MAX_SIZE;
		drv->srv_desc[channel].ptr = drv->srv_desc[0].ptr + offset;
		drv->srv_desc[channel].dma = drv->srv_desc[0].dma + offset;
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
				     const struct hse_srv_desc *srv_desc)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);

	if (unlikely(!dev || channel >= HSE_NUM_CHANNELS || !srv_desc))
		return;

	memcpy_toio(drv->srv_desc[channel].ptr, srv_desc, sizeof(*srv_desc));
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

	for (channel = HSE_NUM_CHANNELS - 1; channel > 0; channel--)
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
			dev_dbg(dev, "%s: no channel of type %d available\n",
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
 *         index out of range, -EBUSY for channel busy, no channel available or
 *         firmware on stand-by, -ENOTRECOVERABLE for firmware in shutdown state
 */
int hse_srv_req_async(struct device *dev, u8 channel, const void *srv_desc,
		      const void *ctx, void (*rx_cbk)(int err, void *ctx))
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	int err;

	if (unlikely(!dev || !rx_cbk || !ctx || !srv_desc))
		return -EINVAL;

	if (unlikely(channel != HSE_CHANNEL_ANY && channel >= HSE_NUM_CHANNELS))
		return -ECHRNG;

	switch (drv->firmware_status) {
	case HSE_FW_STANDBY:
		if (unlikely(channel != HSE_CHANNEL_ADM))
			return -EBUSY;
		break;
	case HSE_FW_SHUTDOWN:
		return -ENOTRECOVERABLE;
	default:
		break;
	}

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

	drv->channel_busy[channel] = true;

	spin_unlock(&drv->tx_lock);

	drv->rx_cbk[channel].fn = rx_cbk;
	drv->rx_cbk[channel].ctx = (void *)ctx;

	hse_sync_srv_desc(dev, channel, srv_desc);

	err = hse_mu_msg_send(drv->mu, channel, drv->srv_desc[channel].dma);

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
 *         index out of range, -EBUSY for channel busy, no channel available or
 *         firmware on stand-by, -ENOTRECOVERABLE for firmware in shutdown state
 */
int hse_srv_req_sync(struct device *dev, u8 channel, const void *srv_desc)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	DECLARE_COMPLETION_ONSTACK(done);
	unsigned long flags;
	int err, reply;

	if (unlikely(!dev || !srv_desc))
		return -EINVAL;

	if (unlikely(channel != HSE_CHANNEL_ANY && channel >= HSE_NUM_CHANNELS))
		return -ECHRNG;

	switch (drv->firmware_status) {
	case HSE_FW_STANDBY:
		if (unlikely(channel != HSE_CHANNEL_ADM))
			return -EBUSY;
		break;
	case HSE_FW_SHUTDOWN:
		return -ENOTRECOVERABLE;
	default:
		break;
	}

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

	drv->channel_busy[channel] = true;

	spin_unlock(&drv->tx_lock);

	drv->sync[channel].done = &done;
	drv->sync[channel].reply = &reply;

	hse_sync_srv_desc(dev, channel, srv_desc);

	err = hse_mu_msg_send(drv->mu, channel, drv->srv_desc[channel].dma);
	if (unlikely(err))
		return err;

	err = wait_for_completion_interruptible(&done);
	if (err) {
		spin_lock_irqsave(&drv->rx_lock, flags);
		drv->sync[channel].done = NULL;
		drv->sync[channel].reply = NULL;
		spin_unlock_irqrestore(&drv->rx_lock, flags);

		dev_dbg(dev, "%s: request id 0x%08x interrupted, channel %d\n",
			__func__, drv->srv_desc[channel].id, channel);
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
	unsigned long flags;
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
		dev_dbg(dev, "%s: request id 0x%08x reply 0x%08X, channel %d\n",
			__func__, drv->srv_desc[channel].id, srv_rsp, channel);

	if (drv->rx_cbk[channel].fn) {
		void (*rx_cbk)(int err, void *ctx) = drv->rx_cbk[channel].fn;
		void *ctx = drv->rx_cbk[channel].ctx;

		drv->rx_cbk[channel].fn = NULL;
		drv->rx_cbk[channel].ctx = NULL;

		drv->channel_busy[channel] = false;

		rx_cbk(err, ctx); /* upper layer RX callback */
		return;
	}

	spin_lock_irqsave(&drv->rx_lock, flags);
	if (drv->sync[channel].done) {
		*drv->sync[channel].reply = err;
		mb(); /* ensure reply is written before calling complete */

		complete(drv->sync[channel].done);
		drv->sync[channel].done = NULL;
		drv->sync[channel].reply = NULL;
	}
	spin_unlock_irqrestore(&drv->rx_lock, flags);

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
 * hse_srv_req_cancel_all - cancel all requests currently in progress
 * @dev: HSE device
 */
static void hse_srv_req_cancel_all(struct device *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	u8 channel;

	/* process pending replies, if any */
	channel = hse_mu_next_pending_channel(drv->mu);
	while (channel != HSE_CHANNEL_INV) {
		hse_srv_rsp_dispatch(dev, channel);

		channel = hse_mu_next_pending_channel(drv->mu);
	}

	/* notify upper layers that all requests are canceled */
	for (channel = 0; channel < HSE_NUM_CHANNELS; channel++) {
		if (drv->channel_busy[channel]) {
			dev_dbg(dev, "request id 0x%08x canceled, channel %d\n",
				channel, drv->srv_desc[channel].id);
			drv->channel_busy[channel] = false;
		}

		if (drv->rx_cbk[channel].fn) {
			void *ctx = drv->rx_cbk[channel].ctx;

			drv->rx_cbk[channel].fn(-ECANCELED, ctx);
		} else if (drv->sync[channel].done) {
			*drv->sync[channel].reply = -ECANCELED;
			mb(); /* ensure reply is written before complete */

			complete(drv->sync[channel].done);
		}
	}
}

/**
 * hse_evt_dispatcher - deferred handler for HSE_INT_SYS_EVENT type interrupts
 * @irq: interrupt line
 * @dev: HSE device
 *
 * In case a warning has been reported, log the event mask, clear irq and exit.
 * In case a fatal error has been reported, all MU interfaces are disabled
 * and communication with HSE terminated. Therefore, all service requests
 * currently in progress are canceled and any subsequent requests are prevented.
 */
static irqreturn_t hse_evt_dispatcher(int irq, void *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	u32 event = hse_mu_check_event(drv->mu);

	if (event & HSE_EVT_MASK_WARN) {
		dev_warn(dev, "warning, event mask 0x%08x\n", event);
		hse_mu_irq_clear(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_WARN);

		return IRQ_HANDLED;
	}

	/* stop any subsequent requests */
	drv->firmware_status = HSE_FW_SHUTDOWN;

	/* disable RX and event notifications */
	hse_mu_irq_disable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_ALL);

	dev_crit(dev, "fatal error, event mask 0x%08x\n", event);
	hse_mu_irq_clear(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_ERR);

	/* cancel all requests */
	hse_srv_req_cancel_all(dev);

	/* unregister kernel crypto algorithms and hwrng */
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_AHASH))
		hse_ahash_unregister(&drv->ahash_algs);
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_SKCIPHER))
		hse_skcipher_unregister(&drv->skcipher_algs);
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_AEAD))
		hse_aead_unregister(&drv->aead_algs);
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_RNG))
		hse_rng_unregister(dev);

	dev_crit(dev, "communication terminated, reset system to recover\n");

	return IRQ_HANDLED;
}

static int hse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hse_drvdata *drv;
	u16 status;
	int err;

	dev_dbg(dev, "probing driver\n");

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (IS_ERR_OR_NULL(drv))
		return -ENOMEM;
	platform_set_drvdata(pdev, drv);

	/* MU interface setup */
	drv->mu = hse_mu_init(dev, hse_rx_dispatcher, hse_evt_dispatcher);
	if (IS_ERR(drv->mu)) {
		dev_err(dev, "failed to initialize MU interface\n");
		return PTR_ERR(drv->mu);
	}

	/* check for firmware */
	status = hse_mu_check_status(drv->mu);
	if (!likely(status & HSE_STATUS_INIT_OK)) {
		if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_MU0))
			dev_warn(dev, "firmware not found\n");
		else
			dev_warn(dev, "MU interface not active\n");
		return -ENODEV;
	}

	/* configure channels */
	hse_config_channels(dev);

	/* initialize locks */
	spin_lock_init(&drv->stream_lock);
	spin_lock_init(&drv->tx_lock);
	spin_lock_init(&drv->rx_lock);
	spin_lock_init(&drv->key_ring_lock);

	/* enable RX and event notifications */
	hse_mu_irq_enable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_enable(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_WARN);
	hse_mu_irq_enable(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_ERR);

	/* enable service requests */
	drv->firmware_status = HSE_FW_RUNNING;

	/* check firmware version */
	err = hse_check_fw_version(dev);
	if (unlikely(err))
		goto err_probe_failed;

	dev_info(dev, "%s firmware, version %d.%d.%d\n",
		 drv->firmware_version.fw_type == 0 ? "standard" :
		 (drv->firmware_version.fw_type == 1 ? "premium" : "custom"),
		 drv->firmware_version.major, drv->firmware_version.minor,
		 drv->firmware_version.patch);

	/* check firmware global status */
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_RNG) &&
	    !likely(status & HSE_STATUS_RNG_INIT_OK)) {
		dev_err(dev, "RNG not initialized\n");
		err = -ENODEV;
		goto err_probe_failed;
	}
	if (!likely(status & HSE_STATUS_INSTALL_OK)) {
		dev_err(dev, "key catalogs not formatted\n");
		err = -ENODEV;
		goto err_probe_failed;
	}
	if (unlikely(status & HSE_STATUS_PUBLISH_SYS_IMAGE))
		dev_warn(dev, "volatile configuration, publish SYS_IMAGE\n");

	/* initialize key rings */
	err = hse_key_ring_init(dev, &drv->aes_key_ring, HSE_KEY_TYPE_AES,
				CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GROUP_ID,
				CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GROUP_SIZE);
	if (unlikely(err))
		goto err_probe_failed;

	err = hse_key_ring_init(dev, &drv->hmac_key_ring, HSE_KEY_TYPE_HMAC,
				CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GROUP_ID,
				CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GROUP_SIZE);
	if (unlikely(err))
		goto err_probe_failed;

	/* register kernel crypto algorithms and hwrng */
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_AHASH))
		hse_ahash_register(dev, &drv->ahash_algs);
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_SKCIPHER))
		hse_skcipher_register(dev, &drv->skcipher_algs);
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_AEAD))
		hse_aead_register(dev, &drv->aead_algs);
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_RNG))
		hse_rng_register(dev);

	dev_info(dev, "device ready, status 0x%04X\n", status);

	return 0;
err_probe_failed:
	dev_err(dev, "probe failed with status 0x%04X\n", status);
	return err;
}

static int hse_remove(struct platform_device *pdev)
{
	struct hse_drvdata *drv = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	/* disable RX and event notifications */
	hse_mu_irq_disable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_ALL);

	/* unregister kernel crypto algorithms and hwrng */
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_AHASH))
		hse_ahash_unregister(&drv->ahash_algs);
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_SKCIPHER))
		hse_skcipher_unregister(&drv->skcipher_algs);
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_AEAD))
		hse_aead_unregister(&drv->aead_algs);
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_RNG))
		hse_rng_unregister(dev);

	/* empty used key rings */
	hse_key_ring_free(&drv->aes_key_ring);
	hse_key_ring_free(&drv->hmac_key_ring);

	dev_dbg(dev, "device removed\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hse_pm_suspend(struct device *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	struct hse_srv_desc srv_desc;
	int err;

	/* stop subsequent requests */
	drv->firmware_status = HSE_FW_STANDBY;

	/* disable RX notifications, except for HSE_CHANNEL_ADM */
	hse_mu_irq_disable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_enable(drv->mu, HSE_INT_RESPONSE, BIT(HSE_CHANNEL_ADM));

	/* prepare firmware for stand-by */
	srv_desc.srv_id = HSE_SRV_ID_PREPARE_FOR_STANDBY;
	err = hse_srv_req_sync(dev, HSE_CHANNEL_ADM, &srv_desc);

	/* cancel other requests */
	hse_srv_req_cancel_all(dev);

	if (unlikely(err && err != -EPERM)) {
		dev_err(dev, "%s: request failed: %d\n", __func__, err);
		goto err_enable_irq;
	}

	dev_info(dev, "device ready for stand-by\n");

	return 0;
err_enable_irq:
	hse_mu_irq_enable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	drv->firmware_status = HSE_FW_RUNNING;
	return err;
}

static int hse_pm_resume(struct device *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	u16 status;
	int err;

	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_MU0)) {
		/* signal firmware not to wait for peripheral configuration */
		hse_mu_trigger_event(drv->mu, HSE_HOST_PERIPH_CONFIG_DONE);
	}

	/* enable RX and event notifications */
	hse_mu_irq_enable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_enable(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_WARN);
	hse_mu_irq_enable(drv->mu, HSE_INT_SYS_EVENT, HSE_EVT_MASK_ERR);

	/* check firmware status */
	status = hse_mu_check_status(drv->mu);
	if (!(status & HSE_STATUS_INIT_OK)) {
		/* wait for firmware init */
		err = hse_check_fw_version(dev);
		if (unlikely(err)) {
			dev_err(dev, "%s: request failed: %d\n", __func__, err);
			return err;
		}
		status = hse_mu_check_status(drv->mu);
	}

	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_RNG) &&
	    !likely(status & HSE_STATUS_RNG_INIT_OK))
		dev_err(dev, "RNG not initialized\n");
	if (!likely(status & HSE_STATUS_INSTALL_OK))
		dev_err(dev, "key catalogs not formatted\n");
	if (unlikely(status & HSE_STATUS_PUBLISH_SYS_IMAGE))
		dev_warn(dev, "volatile configuration, publish SYS_IMAGE\n");

	drv->firmware_status = HSE_FW_RUNNING;
	dev_info(dev, "device resumed, status 0x%04X\n", status);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(hse_pm_ops, hse_pm_suspend, hse_pm_resume);

static const struct of_device_id hse_of_match[] = {
	{
		.name = CONFIG_CRYPTO_DEV_NXP_HSE_MU,
		.compatible = "nxp,s32cc-hse",
	}, {}
};
MODULE_DEVICE_TABLE(of, hse_of_match);

static struct platform_driver hse_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table	= hse_of_match,
		.pm = &hse_pm_ops,
	},
	.probe = hse_probe,
	.remove = hse_remove,
};

module_platform_driver(hse_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS_CRYPTO(KBUILD_MODNAME);
MODULE_DESCRIPTION("NXP Hardware Security Engine (HSE) Driver");
