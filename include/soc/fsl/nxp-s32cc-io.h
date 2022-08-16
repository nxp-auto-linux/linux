/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright 2022 NXP
 *
 * Bit operations. From u-boot: arch/arm/include/asm/io.h
 */

#ifndef NXP_S32CC_IO_H
#define NXP_S32CC_IO_H

#include <linux/io.h>

#define clrbits(type, addr, clear) \
	__raw_write##type(__raw_read##type(addr) & ~(clear), (addr))

#define setbits(type, addr, set) \
	__raw_write##type(__raw_read##type(addr) | (set), (addr))

#define clrsetbits(type, addr, clear, set) \
	__raw_write##type((__raw_read##type(addr) & ~(clear)) | (set), (addr))


#define clrbits_32(addr, clear) clrbits(l, addr, clear)
#define setbits_32(addr, set) setbits(l, addr, set)
#define clrsetbits_32(addr, clear, set) clrsetbits(l, addr, clear, set)

#define clrbits_16(addr, clear) clrbits(w, addr, clear)
#define setbits_16(addr, set) setbits(w, addr, set)
#define clrsetbits_16(addr, clear, set) clrsetbits(w, addr, clear, set)

#define clrbits_8(addr, clear) clrbits(b, addr, clear)
#define setbits_8(addr, set) setbits(b, addr, set)
#define clrsetbits_8(addr, clear, set) clrsetbits(b, addr, clear, set)

#endif /* NXP_S32CC_IO_H*/
