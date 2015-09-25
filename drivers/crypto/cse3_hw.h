/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Freescale Cryptographic Services Engine (CSE3) Device Driver
 * CSE3 Hardware API
 *
 * Copyright (c) 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * later as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _CSE_HW_H
#define _CSE_HW_H

int cse_hw_comm(struct cse_device_data *dev, uint32_t flags, int phase);

#endif
