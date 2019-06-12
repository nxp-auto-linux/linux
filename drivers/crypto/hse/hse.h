/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - HSE Common
 *
 * Copyright 2019 NXP
 */

#ifndef HSE_H
#define HSE_H

#define HSE_CRA_PRIORITY    2000u /* crypto algorithm priority */

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
 * @hmac_keys: available AES key slots
 */
struct hse_drvdata {
	void *mu_inst;
	struct list_head hash_algs;
	struct list_head aes_keys;
};

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
		return addr - 0x20000000ull;

	return 0ull;
}

int hse_err_decode(u32 srv_rsp);

void hse_hash_register(struct device *dev);

void hse_hash_unregister(struct device *dev);

void hse_skcipher_register(struct device *dev);

void hse_skcipher_unregister(void);

#endif /* HSE_H */
