/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - HSE Common
 *
 * Copyright 2019 NXP
 */

#ifndef HSE_H
#define HSE_H

#include <crypto/aes.h>

#define HSE_CRA_PRIORITY    2000u /* HSE crypto algorithm priority */

/**
 * struct hse_key - HSE key
 * @entry: list position
 * @handle: key handle
 */
struct hse_key {
	struct list_head entry;
	u32 handle;
};

/**
 * struct hse_drvdata - HSE driver private data
 * @mu_inst: MU instance
 * @hash_algs: supported hash algorithms
 * @hmac_keys: available HMAC key slots
 * @hmac_keys: available AES key slots
 * @key_lock: lock for acquiring key handle
 */
struct hse_drvdata {
	void *mu_inst;
	struct list_head hash_algs;
	struct list_head hmac_keys;
	struct list_head aes_keys;
	spinlock_t key_lock; /* lock for acquiring key handle */
};

/**
 * Translate address with 512MB, as seen by HSE
 */
static inline dma_addr_t _hse_addr(dma_addr_t addr)
{
	return addr - 0x20000000ull;
}

/**
 * hse_addr - HSE Address Translation
 * @virt_addr: virtual address to be translated
 *
 * This function only admits addresses from the kernel linear address space.
 *
 * Return: physical address as seen by HSE, zero for failed translation
 */
static __always_inline phys_addr_t hse_addr(void *virt_addr)
{
	phys_addr_t addr = virt_to_phys(virt_addr);

	/* translate DDR addresses */
	if (addr >= 0x80000000ull && addr <= 0xFFFFFFFFull)
		return _hse_addr(addr);

	return 0ull;
}

void hse_hash_register(struct device *dev);

void hse_hash_unregister(struct device *dev);

void hse_skcipher_register(struct device *dev);

void hse_skcipher_unregister(void);

void hse_aead_register(struct device *dev);

void hse_aead_unregister(void);

void hse_hwrng_register(struct device *dev);

#endif /* HSE_H */
