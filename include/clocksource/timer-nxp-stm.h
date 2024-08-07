/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright 2024 NXP
 */
#ifndef __NXP_TIMER_STM_H
#define __NXP_TIMER_STM_H

#define STM_CR      0x00
#define STM_CNT     0x04

#define STM_CR_TEN  BIT(0)
#define STM_CR_FRZ  BIT(1)
#define STM_CR_CPS_MASK	GENMASK(15, 8)
#define STM_CR_CPS_OFFSET	8u
#define STM_CR_CPS(x)	(((x) << STM_CR_CPS_OFFSET) & STM_CR_CPS_MASK)

#define STM_ENABLE_MASK	(STM_CR_FRZ | STM_CR_TEN)

static inline u32 stm_clksrc_getcnt(void __iomem *timer_base)
{
	return readl(timer_base + STM_CNT);
}

static inline void stm_clksrc_setcnt(void __iomem *timer_base,
				     u32 cnt)
{
	writel(cnt, timer_base + STM_CNT);
}

static inline void stm_disable(void __iomem *timer_base)
{
	u32 reg = readl(timer_base + STM_CR);

	reg &= ~(STM_CR_CPS_MASK | STM_ENABLE_MASK);
	writel(reg, timer_base + STM_CR);
}

static inline void stm_enable(void __iomem *timer_base,
			      u32 prescaler)
{
	u32 reg = readl(timer_base + STM_CR);

	if (prescaler < 1) {
		reg &= ~STM_CR_CPS_MASK;
		reg |= STM_ENABLE_MASK;
	} else {
		reg |= (STM_CR_CPS(prescaler - 1) | STM_ENABLE_MASK);
	}

	writel(reg, timer_base + STM_CR);
}

#endif /* __NXP_TIMER_STM_H */
