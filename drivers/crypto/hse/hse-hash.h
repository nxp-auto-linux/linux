/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - HASH Interface
 *
 * Copyright 2019 NXP
 */

#ifndef HSE_HASH_H
#define HSE_HASH_H

int hse_hash_init(struct device *dev, void *mu_inst);

void hse_hash_free(void);

#endif /* HSE_HASH_H */
