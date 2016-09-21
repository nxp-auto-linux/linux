/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 * CSE3 Linux Crypto API Interface
 *
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * later as publishhed by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _CSE_CAPI_H
#define _CSE_CAPI_H

#ifdef CONFIG_CRYPTO_DEV_FSL_CSE3_CRYPTO_API

/**
 * Register and unregister supported CSE3 crypto API algorithms
 */
void cse_register_crypto_api(void);
void cse_unregister_crypto_api(void);

#else

static inline void cse_register_crypto_api(void)
{
}

static inline void cse_unregister_crypto_api(void)
{
}

#endif

#endif /* _CSE_CAPI_H */
