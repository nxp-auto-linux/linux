/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2024 NXP
 */
#ifndef __NETC_GLOBAL_H
#define __NETC_GLOBAL_H

#include <linux/io.h>

static inline u32 netc_read(void __iomem *reg)
{
	return ioread32(reg);
}

static inline void netc_write(void __iomem *reg, u32 val)
{
	iowrite32(val, reg);
}

#if IS_ENABLED(CONFIG_PTP_NETC_V4_TIMER)
u64 netc_timer_get_current_time(struct pci_dev *timer_dev);
#else
static inline u64 netc_timer_get_current_time(struct pci_dev *timer_dev)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_NXP_NETC_BLK_CTRL)
void netc_xpcs_port_init(int port);
void netc_ierb_enable_wakeonlan(void);
void netc_ierb_disable_wakeonlan(void);
int netc_ierb_may_wakeonlan(void);
#else
static inline void netc_xpcs_port_init(int port)
{
}

static inline void netc_ierb_enable_wakeonlan(void)
{
}

static inline void netc_ierb_disable_wakeonlan(void)
{
}

static inline int netc_ierb_may_wakeonlan(void)
{
	return -EINVAL;
}
#endif

#endif
