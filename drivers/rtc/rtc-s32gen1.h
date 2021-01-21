/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019,2021 NXP
 *
 * RTC register definition for the S32GEN1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef _RTC_S32GEN1_H_
#define _RTC_S32GEN1_H_

#define RTCSUPV_OFFSET	0x0ul
#define RTCC_OFFSET	0x4ul
#define RTCS_OFFSET	0x8ul
#define RTCCNT_OFFSET	0xCul
#define APIVAL_OFFSET	0x10ul
#define RTCVAL_OFFSET	0x14ul

/* RTCSUPV fields */
#define RTCSUPV_SUPV		BIT(31)
/* RTCC fields */
#define RTCC_CNTEN		BIT(31)
#define RTCC_RTCIE_SHIFT	30
#define RTCC_RTCIE		BIT(RTCC_RTCIE_SHIFT)
#define RTCC_ROVREN		BIT(28)
#define RTCC_APIEN		BIT(15)
#define RTCC_APIIE		BIT(14)
#define RTCC_CLKSEL_MASK	(BIT(12) | BIT(13))
#define RTCC_CLKSEL(n)		((n << 12) & RTCC_CLKSEL_MASK)
#define RTCC_DIV512EN		BIT(11)
#define RTCC_DIV32EN		BIT(10)
/* RTCS fields */
#define RTCS_RTCF		BIT(29)
#define RTCS_INV_RTC		BIT(18)
#define RTCS_APIF		BIT(13)
#define RTCS_ROVRF		BIT(10)

/* Clock sources - usable with RTCC_CLKSEL */
#define S32GEN1_RTC_SOURCE_FIRC	0x2
#define S32GEN1_RTC_SOURCE_SIRC	0x0

#endif /* _RTC_S32GEN1_H_ */
