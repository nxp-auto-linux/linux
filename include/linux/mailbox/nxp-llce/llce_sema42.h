/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020-2022 NXP */
#ifndef LLCE_SEMA42_H
#define LLCE_SEMA42_H

#include <linux/io.h>

#define LLCE_SEMA42_GR_GTFSM_0 0x0U
#define LLCE_SEMA42_GR_GTFSM_1 0x1U
#define LLCE_SEMA42_GR_GTFSM_2 0x2U
#define LLCE_SEMA42_GR_GTFSM_3 0x3U
#define LLCE_SEMA42_GR_GTFSM_4 0x4U
#define LLCE_SEMA42_GR_GTFSM_5 0x5U
#define LLCE_SEMA42_GR_GTFSM_6 0x6U
#define LLCE_SEMA42_GR_GTFSM_7 0x7U
#define LLCE_SEMA42_GR_GTFSM_8 0x8U
#define LLCE_SEMA42_GR_GTFSM_9 0x9U
#define LLCE_SEMA42_GR_GTFSM_10 0xAU
#define LLCE_SEMA42_GR_GTFSM_11 0xBU
#define LLCE_SEMA42_GR_GTFSM_12 0xCU
#define LLCE_SEMA42_GR_GTFSM_13 0xDU
#define LLCE_SEMA42_GR_GTFSM_14 0xEU
#define LLCE_SEMA42_GR_GTFSM_15 0xFU

/**
 * Gate numbers used by sema42 module to lock and unlock gates.
 **/
enum llce_sema42_gate {
	LLCE_SEMA42_GATE0 = 0UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE1 = 1UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE2 = 2UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE3 = 3UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE4 = 4UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE5 = 5UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE6 = 6UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE7 = 7UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE8 = 8UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE9 = 9UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE10 = 10UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE11 = 11UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE12 = 12UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE13 = 13UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE14 = 14UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE15 = 15UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE16 = 16UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE17 = 17UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE18 = 18UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE19 = 19UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE20 = 20UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE21 = 21UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE22 = 22UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE23 = 23UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE24 = 24UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE25 = 25UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE26 = 26UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE27 = 27UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE28 = 28UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE29 = 29UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE30 = 30UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE31 = 31UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE32 = 32UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE33 = 33UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE34 = 34UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE35 = 35UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE36 = 36UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE37 = 37UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE38 = 38UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE39 = 39UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE40 = 40UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE41 = 41UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE42 = 42UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE43 = 43UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE44 = 44UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE45 = 45UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE46 = 46UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE47 = 47UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE48 = 48UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE49 = 49UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE50 = 50UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE51 = 51UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE52 = 52UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE53 = 53UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE54 = 54UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE55 = 55UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE56 = 56UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE57 = 57UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE58 = 58UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE59 = 59UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE60 = 60UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE61 = 61UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE62 = 62UL, /** Sema42 Gate */
	LLCE_SEMA42_GATE63 = 63UL, /** Sema42 Gate */
} __packed;

static inline void __iomem *
llce_sema42_get_gate_addr(void __iomem *base_addr, enum llce_sema42_gate gate)
{
	return base_addr + ((u32)(gate) & 0x3FU);
}

static inline void llce_sema42_lock(void __iomem *base_addr,
				    enum llce_sema42_gate gate, u8 domain)
{
	void __iomem *addr = llce_sema42_get_gate_addr(base_addr, gate);

	do {
		writeb(domain, addr);
	} while (domain != readb(addr));
}

static inline void llce_sema42_unlock(void __iomem *base_addr,
				      enum llce_sema42_gate gate)
{
	void __iomem *addr = llce_sema42_get_gate_addr(base_addr, gate);

	writeb(LLCE_SEMA42_GR_GTFSM_0, addr);
}

#endif /* LLCE_SEMA42_H */
