// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver - Core
 *
 * This file contains the device driver core for the HSE cryptographic engine.
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/crypto.h>

#include "hse-abi.h"
#include "hse-core.h"
#include "hse-mu.h"

/**
 * hse_err_decode - HSE Error Code Translation
 * @srv_rsp: HSE service response
 *
 * Return: 0 on service request success, error code otherwise
 */
static int hse_err_decode(u32 srv_rsp)
{
	int err;

	switch (srv_rsp) {
	case HSE_SRV_RSP_OK:
		err = 0;
		break;
	case HSE_SRV_RSP_VERIFY_FAILED:
		err = -EBADMSG;
		break;
	case HSE_SRV_RSP_INVALID_ADDR:
	case HSE_SRV_RSP_INVALID_PARAM:
		err = -EBADR;
		break;
	case HSE_SRV_RSP_NOT_SUPPORTED:
		err = -EOPNOTSUPP;
		break;
	case HSE_SRV_RSP_NOT_ALLOWED:
		err = -EPERM;
		break;
	case HSE_SRV_RSP_NOT_ENOUGH_SPACE:
		err = -ENOMEM;
		break;
	case HSE_SRV_RSP_KEY_NOT_AVAILABLE:
	case HSE_SRV_RSP_KEY_EMPTY:
		err = -ENOKEY;
		break;
	case HSE_SRV_RSP_KEY_INVALID:
	case HSE_SRV_RSP_KEY_WRITE_PROTECTED:
	case HSE_SRV_RSP_KEY_UPDATE_ERROR:
		err = -EKEYREJECTED;
		break;
	case HSE_SRV_RSP_CANCELED:
		err = -ECANCELED;
		break;
	default:
		err = -EFAULT;
		break;
	}

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

	drv->mu_inst = hse_mu_init(dev, hse_err_decode);
	if (IS_ERR(drv->mu_inst))
		return PTR_ERR(drv->mu_inst);

	init_ok_mask = HSE_STATUS_INIT_OK | HSE_STATUS_INSTALL_OK;
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_HWRNG))
		init_ok_mask |= HSE_STATUS_RNG_INIT_OK;
	status = hse_mu_status(drv->mu_inst);
	if (unlikely((status & init_ok_mask) != init_ok_mask)) {
		dev_err(dev, "init failed with status 0x%04X\n", status);
		return -ENODEV;
	}

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

	hse_ahash_register(dev);

	hse_skcipher_register(dev);

	hse_aead_register(dev);

	if (IS_ENABLED(CONFIG_CRYPTO_DEV_NXP_HSE_HWRNG))
		hse_hwrng_register(dev);

	dev_info(dev, "device ready, status 0x%04X\n", status);

	return 0;
}

static int hse_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hse_drvdata *drv = dev_get_drvdata(dev);

	hse_aead_unregister();
	hse_skcipher_unregister();
	hse_ahash_unregister(dev);

	hse_key_ring_free(&drv->aes_key_ring);
	hse_key_ring_free(&drv->hmac_key_ring);

	hse_mu_free(drv->mu_inst);

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
