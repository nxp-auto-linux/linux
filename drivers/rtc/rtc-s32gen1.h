/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 NXP
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
#define RTCC_OFFSET		0x4ul
#define RTCS_OFFSET		0x8ul
#define RTCCNT_OFFSET	0xCul
#define APIVAL_OFFSET	0x10ul
#define RTCVAL_OFFSET	0x14ul

#define SUPV			BIT(31)
#define CNTEN			BIT(31)
#define RTCIE			BIT(30)
#define ROVREN			BIT(28)
#define APIEN			BIT(15)
#define APIIE			BIT(14)
#define CLKSEL_MASK		(BIT(12) | BIT(13))
#define CLKSEL(n)		((n << 12) & CLKSEL_MASK)
#define DIV512EN		BIT(11)
#define DIV32EN			BIT(10)
#define RTCF			BIT(29)
#define APIF			BIT(13)
#define ROVRF			BIT(10)

#endif /* _RTC_S32GEN1_H_ */
