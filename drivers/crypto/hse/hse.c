// SPDX-License-Identifier: BSD-3-Clause
/*
 * NXP HSE Driver
 *
 * Copyright 2019 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>

#include "hse.h"
#include "hse-mu.h"
#include "hse-abi.h"

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
 * hse_init_key_ring - initialize all keys in a specific key group
 * @dev: HSE device
 * @key_ring: output key ring
 * @group_id: key group ID
 * @group_size: key group size
 *
 * Return: 0 on success, -ENOMEM for failed key ring allocation
 */
static int hse_init_key_ring(struct device *dev, struct list_head *key_ring,
			     u8 group_id, u8 group_size)
{
	struct hse_key *ring;
	unsigned int i;

	ring = devm_kzalloc(dev, group_size * sizeof(*ring), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ring))
		return -ENOMEM;

	INIT_LIST_HEAD(key_ring);

	for (i = 0; i < group_size; i++) {
		ring[i].handle = HSE_KEY_HANDLE(group_id, i);
		list_add_tail(&ring[i].entry, key_ring);
	}

	return 0;
}

/**
 * hse_free_key_ring - remove all keys in a specific key group
 * @key_ring: input key ring
 */
static void hse_free_key_ring(struct list_head *key_ring)
{
	struct hse_key *key, *tmp;

	list_for_each_entry_safe(key, tmp, key_ring, entry) {
		list_del(&key->entry);
	}
}

static int hse_probe(struct platform_device *pdev)
{
	struct hse_drvdata *pdata;
	int err;
	u16 status, init_ok_mask;

	dev_info(&pdev->dev, "probing HSE device %s\n", pdev->name);

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (IS_ERR_OR_NULL(pdata))
		return -ENOMEM;

	platform_set_drvdata(pdev, pdata);

	pdata->mu_inst = hse_mu_init(&pdev->dev, hse_err_decode);
	if (IS_ERR(pdata->mu_inst))
		return PTR_ERR(pdata->mu_inst);

	init_ok_mask = HSE_STATUS_INIT_OK | HSE_STATUS_RNG_INIT_OK |
		       HSE_STATUS_INSTALL_OK | HSE_STATUS_BOOT_OK;

	status = hse_mu_get_status(pdata->mu_inst);
	if (unlikely((status & init_ok_mask) != init_ok_mask)) {
		dev_err(&pdev->dev, "init failed with status 0x%04X\n", status);
		return -ENODEV;
	}

	err = hse_init_key_ring(&pdev->dev, &pdata->hmac_keys,
				CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GID,
				CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GSIZE);
	if (err)
		return err;

	hse_hash_register(&pdev->dev);

	err = hse_init_key_ring(&pdev->dev, &pdata->aes_keys,
				CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GID,
				CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GSIZE);
	if (err)
		return err;

	hse_skcipher_register(&pdev->dev);

	dev_info(&pdev->dev, "HSE device %s initialized\n", pdev->name);

	return 0;
}

static int hse_remove(struct platform_device *pdev)
{
	struct hse_drvdata *pdata = platform_get_drvdata(pdev);

	hse_skcipher_unregister();
	hse_free_key_ring(&pdata->aes_keys);

	hse_hash_unregister(&pdev->dev);
	hse_free_key_ring(&pdata->hmac_keys);

	hse_mu_free(pdata->mu_inst);

	dev_info(&pdev->dev, "HSE device %s removed", pdev->name);

	return 0;
}

static const struct of_device_id hse_of_match[] = {
	{
		.compatible = "fsl,s32gen1-hse"
	}, {}
};
MODULE_DEVICE_TABLE(of, hse_of_match);

static struct platform_driver hse_driver = {
	.driver = {
		.name = "nxp_hse",
		.of_match_table	= hse_of_match,
	},
	.probe = hse_probe,
	.remove = hse_remove,
};

module_platform_driver(hse_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("nxp_hse");
MODULE_DESCRIPTION("NXP Hardware Security Engine (HSE) Driver");
