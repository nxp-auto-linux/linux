// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Core
 *
 * This file contains the device driver core for the HSE cryptographic engine.
 *
 * Copyright 2019-2020 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/crypto.h>

#include "hse-abi.h"
#include "hse-core.h"
#include "hse-mu.h"

/**
 * struct hse_drvdata - HSE driver private data
 * @ahash_algs: registered hash and hash-based MAC algorithms
 * @skcipher_algs: registered symmetric key cipher algorithms
 * @aead_algs: registered AEAD algorithms
 * @mu: MU instance handle
 * @channel_busy[n]: internally cached status of MU channel n
 * @refcnt: acquired service channel reference count
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
	struct list_head ahash_algs;
	struct list_head skcipher_algs;
	struct list_head aead_algs;
	void *mu;
	bool channel_busy[HSE_NUM_CHANNELS];
	atomic_t refcnt[HSE_NUM_CHANNELS];
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

	ring = devm_kmalloc_array(dev, group_size, sizeof(*ring), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ring))
		return -ENOMEM;

	INIT_LIST_HEAD(key_ring);

	for (i = 0; i < group_size; i++) {
		ring[i].handle = HSE_KEY_HANDLE(group_id, i);
		ring[i].type = type;
		list_add_tail(&ring[i].entry, key_ring);
	}

	return 0;
}

/**
 * hse_key_ring_free - remove all keys in a specific key group
 * @key_ring: input key ring
 */
static void hse_key_ring_free(struct list_head *key_ring)
{
	struct hse_key *key, *tmp;

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
 * hse_err_decode - HSE Error Code Translation
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
 * hse_next_free_channel - find the next free service channel
 * @dev: HSE device
 * @type: channel type
 *
 * If HSE_CH_TYPE_STREAM is requested, return a free channel from the first
 * HSE_STREAM_COUNT, the only ones usable for streaming mode. Otherwise,
 * if HSE_CH_TYPE_SHARED is requested, find any available service channel.
 * Channel 0 is reserved for administrative services and cannot be used.
 *
 * Return: channel index, HSE_CHANNEL_INV if none available
 */
static u8 hse_next_free_channel(struct device *dev, enum hse_ch_type type)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	u8 channel;

	for (channel = 1; channel < HSE_NUM_CHANNELS; channel++)
		switch (type) {
		case HSE_CH_TYPE_STREAM:
			if (channel >= HSE_STREAM_COUNT ||
			    atomic_read(&drv->refcnt[channel]))
				continue;
			/* fall through */
		case HSE_CH_TYPE_SHARED:
			if (!drv->channel_busy[channel])
				return channel;
			continue;
		default:
			return HSE_CHANNEL_INV;
		}

	return HSE_CHANNEL_INV;
}

/**
 * hse_channel_acquire - acquire a stream or a shared channel (non-blocking)
 * @dev: HSE device
 * @type: channel type
 * @channel: service channel index
 *
 * If HSE_CH_TYPE_STREAM is requested, reserve the first available stream and
 * return its channel index, which, for convenience, matches the HSE stream ID.
 * The stream will be reserved over the entire duration until it is released.
 * If HSE_CH_TYPE_SHARED is requested, find the shared channel with the lowest
 * reference count, excluding any restricted to streaming mode use. Shared
 * channels can be acquired simultaneously and should be used if the request
 * order preservation is a concern, as it is not guaranteed by HSE_CHANNEL_ANY.
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -EBUSY for no stream
 *         type channel currently available
 */
int hse_channel_acquire(struct device *dev, enum hse_ch_type type, u8 *channel)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	unsigned int min, i;

	if (unlikely(!dev || !channel))
		return -EINVAL;

	switch (type) {
	case HSE_CH_TYPE_STREAM:
		spin_lock(&drv->stream_lock);

		/* find the first available stream */
		*channel = hse_next_free_channel(dev, HSE_CH_TYPE_STREAM);
		if (*channel == HSE_CHANNEL_INV) {
			spin_unlock(&drv->stream_lock);
			dev_dbg(dev, "failed to acquire stream resource\n");
			return -EBUSY;
		}

		/* mark stream as reserved */
		atomic_set(&drv->refcnt[*channel], 1);

		spin_unlock(&drv->stream_lock);
		break;
	case HSE_CH_TYPE_SHARED:
		*channel = HSE_STREAM_COUNT;
		min = atomic_read(&drv->refcnt[*channel]);

		/* find the shared channel with lowest refcount */
		for (i = HSE_STREAM_COUNT; min > 0 && i < HSE_NUM_CHANNELS; i++)
			if (atomic_read(&drv->refcnt[i]) < min) {
				*channel = i;
				min = atomic_read(&drv->refcnt[i]);
			}

		/* increment channel refcount */
		atomic_inc(&drv->refcnt[*channel]);
		break;
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

	atomic_dec_if_positive(&drv->refcnt[channel]);

	return 0;
}

/**
 * hse_srv_req_async - send an asynchronous service request (non-blocking)
 * @dev: HSE device
 * @channel: service channel index
 * @srv_desc: service descriptor DMA address
 * @ctx: context passed to RX callback
 * @rx_cbk: upper layer RX callback
 *
 * Send a HSE service request on the selected channel and register a callback
 * function to be executed asynchronously upon completion. The channel index
 * must be set to HSE_CHANNEL_ANY unless obtained via hse_channel_acquire().
 * Service descriptors must be located in the 32-bit address range.
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -EBUSY for channel busy or none available
 */
int hse_srv_req_async(struct device *dev, u8 channel, dma_addr_t srv_desc,
		      void *ctx, void (*rx_cbk)(int err, void *ctx))
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	int err;

	if (unlikely(!dev || !rx_cbk || !ctx))
		return -EINVAL;

	if (unlikely(channel != HSE_CHANNEL_ANY && channel >= HSE_NUM_CHANNELS))
		return -ECHRNG;

	if (unlikely(upper_32_bits(srv_desc))) {
		dev_err(dev, "descriptor address %pad invalid\n", &srv_desc);
		return -EINVAL;
	}

	spin_lock(&drv->tx_lock);

	if (channel == HSE_CHANNEL_ANY) {
		channel = hse_next_free_channel(dev, HSE_CH_TYPE_SHARED);
		if (unlikely(channel == HSE_CHANNEL_INV)) {
			spin_unlock(&drv->tx_lock);
			dev_dbg(dev, "no service channel available\n");
			return -EBUSY;
		}
	} else if (drv->channel_busy[channel]) {
		spin_unlock(&drv->tx_lock);
		dev_dbg(dev, "%s: channel %d busy\n", __func__, channel);
		return -EBUSY;
	}

	drv->rx_cbk[channel].fn = rx_cbk;
	drv->rx_cbk[channel].ctx = ctx;

	err = hse_mu_msg_send(drv->mu, channel, lower_32_bits(srv_desc));
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
 * @srv_desc: service descriptor DMA address
 *
 * Send a HSE service descriptor on the selected channel and block until the
 * HSE response becomes available, then read the reply. The channel index
 * shall be set to HSE_CHANNEL_ANY unless obtained via hse_channel_acquire().
 * Service descriptors must be located in the 32-bit address range.
 *
 * Return: 0 on success, -EINVAL for invalid parameter, -ECHRNG for channel
 *         index out of range, -EBUSY for channel busy or none available
 */
int hse_srv_req_sync(struct device *dev, u8 channel, dma_addr_t srv_desc)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	DECLARE_COMPLETION_ONSTACK(done);
	int err, reply;

	if (unlikely(!dev))
		return -EINVAL;

	if (unlikely(channel != HSE_CHANNEL_ANY && channel >= HSE_NUM_CHANNELS))
		return -ECHRNG;

	if (unlikely(upper_32_bits(srv_desc))) {
		dev_err(dev, "%s: service descriptor address %pad invalid\n",
			__func__, &srv_desc);
		return -EINVAL;
	}

	spin_lock(&drv->tx_lock);

	if (channel == HSE_CHANNEL_ANY) {
		channel = hse_next_free_channel(dev, HSE_CH_TYPE_SHARED);
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

	err = hse_mu_msg_send(drv->mu, channel, lower_32_bits(srv_desc));
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
 * In case a fatal intrusion has been detected by HSE, all MU interfaces are
 * disabled. Therefore, all service requests currently in progress shall be
 * canceled and any further requests will be prevented.
 */
static irqreturn_t hse_event_dispatcher(int irq, void *dev)
{
	struct hse_drvdata *drv = dev_get_drvdata(dev);
	u32 event = hse_mu_check_event(drv->mu);
	u8 channel;

	switch (event) {
	case HSE_ERR_FATAL_INTRUSION:
		dev_crit(dev, "fatal intrusion detected, MU disabled\n");

		/* disable RX and error notifications */
		hse_mu_irq_disable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
		hse_mu_irq_disable(drv->mu, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);

		/* notify upper layer that all requests are canceled */
		for (channel = 0; channel < HSE_NUM_CHANNELS; channel++) {
			drv->channel_busy[channel] = true;

			if (drv->rx_cbk[channel].fn) {
				void *ctx = drv->rx_cbk[channel].ctx;

				drv->rx_cbk[channel].fn(-ECANCELED, ctx);
				drv->rx_cbk[channel].fn = NULL;
			} else if (drv->sync[channel].done) {
				*drv->sync[channel].reply = -ECANCELED;
				wmb(); /* write reply before complete */

				complete(drv->sync[channel].done);
				drv->sync[channel].done = NULL;
			}
		}
		break;
	case HSE_ERR_NON_FATAL_INTRUSION:
		dev_warn(dev, "potential intrusion reported by HSE\n");
		break;
	default:
		return IRQ_HANDLED;
	}

	hse_mu_irq_clear(drv->mu, HSE_INT_SYS_EVENT, event);

	return IRQ_HANDLED;
}

static int hse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hse_drvdata *drv;
	u16 status, init_ok_mask;
	int err;

	dev_info(dev, "probing driver\n");

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (IS_ERR_OR_NULL(drv))
		return -ENOMEM;
	platform_set_drvdata(pdev, drv);

	/* MU interface setup */
	drv->mu = hse_mu_init(dev, hse_rx_dispatcher, hse_event_dispatcher);
	if (IS_ERR(drv->mu))
		return PTR_ERR(drv->mu);

	init_ok_mask = HSE_STATUS_INIT_OK | HSE_STATUS_INSTALL_OK;
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_HWRNG))
		init_ok_mask |= HSE_STATUS_RNG_INIT_OK;
	status = hse_mu_check_status(drv->mu);
	if (unlikely((status & init_ok_mask) != init_ok_mask)) {
		dev_err(dev, "probe failed with status 0x%04X\n", status);
		return -ENODEV;
	}

	spin_lock_init(&drv->stream_lock);
	spin_lock_init(&drv->tx_lock);
	spin_lock_init(&drv->key_ring_lock);

	/* initialize key rings */
	err = hse_key_ring_init(dev, &drv->hmac_key_ring, HSE_KEY_TYPE_HMAC,
				CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GID,
				CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GSIZE);
	if (err)
		return err;

	err = hse_key_ring_init(dev, &drv->aes_key_ring, HSE_KEY_TYPE_AES,
				CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GID,
				CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GSIZE);
	if (err)
		return err;

	/* enable RX and error notifications */
	hse_mu_irq_enable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_enable(drv->mu, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);

	/* register algorithms */
	hse_ahash_register(dev, &drv->ahash_algs);
	hse_skcipher_register(dev, &drv->skcipher_algs);
	hse_aead_register(dev, &drv->aead_algs);

	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_HWRNG))
		hse_hwrng_register(dev);

	dev_info(dev, "device ready, status 0x%04X\n", status);

	return 0;
}

static int hse_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hse_drvdata *drv = dev_get_drvdata(dev);

	/* disable RX and error notifications */
	hse_mu_irq_disable(drv->mu, HSE_INT_RESPONSE, HSE_CH_MASK_ALL);
	hse_mu_irq_disable(drv->mu, HSE_INT_SYS_EVENT, HSE_CH_MASK_ALL);

	/* unregister algorithms */
	hse_ahash_unregister(&drv->ahash_algs);
	hse_skcipher_unregister(&drv->skcipher_algs);
	hse_aead_unregister(&drv->aead_algs);

	/* empty used key rings */
	hse_key_ring_free(&drv->aes_key_ring);
	hse_key_ring_free(&drv->hmac_key_ring);

	dev_info(dev, "device removed\n");

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
