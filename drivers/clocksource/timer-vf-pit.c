// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2012-2013 Freescale Semiconductor, Inc.
 */

#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/cpuhotplug.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

/*
 * Each pit takes 0x10 Bytes register space
 */
#define PITMCR		0x00
#define PIT0_OFFSET	0x100
#define PITn_OFFSET(n)	(PIT0_OFFSET + 0x10 * (n))
#define PITLDVAL	0x00
#define PITCVAL		0x04
#define PITTCTRL	0x08
#define PITTFLG		0x0c

#define PITMCR_MDIS	(0x1 << 1)

#define PITTCTRL_TEN	(0x1 << 0)
#define PITTCTRL_TIE	(0x1 << 1)
#define PITCTRL_CHN	(0x1 << 2)

#define PITTFLG_TIF	0x1

#if defined(CONFIG_SOC_S32V234)
#define PIT_NR 2
#else
#define PIT_NR 1
#endif

#define MASTER_CORE 0
#define TIMER_NAME "FSL pit timer"

struct pit_timer {
	void __iomem *clksrc_base;
	void __iomem *clkevt_base;
	int irq;
	unsigned int core;
	struct clk *pit_clk;
	unsigned long cycle_per_jiffy;
	struct clock_event_device clockevent_pit;
	struct irqaction pit_timer_irq;
};

static struct pit_timer *pit[PIT_NR];
static unsigned int clocksource;

static inline unsigned int evt_cpu(struct clock_event_device *evt)
{
	unsigned int cpu;
	struct pit_timer *pt = container_of(evt, struct pit_timer,
					    clockevent_pit);

	for (cpu = 0; cpu < PIT_NR; ++cpu)
		if (pit[cpu] == pt)
			break;

	return cpu;
}

static inline void pit_timer_enable(unsigned int cpu)
{
	__raw_writel(PITTCTRL_TEN | PITTCTRL_TIE,
		     pit[cpu]->clkevt_base + PITTCTRL);
}

static inline void pit_timer_disable(unsigned int cpu)
{
	__raw_writel(0, pit[cpu]->clkevt_base + PITTCTRL);
}

static inline void pit_irq_acknowledge(unsigned int cpu)
{
	__raw_writel(PITTFLG_TIF, pit[cpu]->clkevt_base + PITTFLG);
}

static u64 notrace pit_read_sched_clock(void)
{
	return ~__raw_readl(pit[clocksource]->clksrc_base + PITCVAL);
}

static int __init pit_clocksource_init(unsigned long cpu, unsigned long rate)
{
	clocksource = cpu;
	__raw_writel(0,  pit[clocksource]->clksrc_base + PITTCTRL);
	__raw_writel(0xFFFFFFFF,  pit[clocksource]->clksrc_base + PITLDVAL);
	__raw_writel(PITTCTRL_TEN,  pit[clocksource]->clksrc_base + PITTCTRL);

	sched_clock_register(pit_read_sched_clock, 32, rate);
	clocksource_mmio_init(pit[clocksource]->clksrc_base + PITCVAL,
			      "vf-pit", rate,
			      CONFIG_PIT_CLKSRC_RATE, 32,
			      clocksource_mmio_readl_down);

	return 0;
}

static int pit_set_next_event(unsigned long delta,
			      struct clock_event_device *evt)
{
	unsigned int cpu = evt_cpu(evt);
	/*
	 * set a new value to PITLDVAL register will not restart the timer,
	 * to abort the current cycle and start a timer period with the new
	 * value, the timer must be disabled and enabled again.
	 * and the PITLAVAL should be set to delta minus one according to pit
	 * hardware requirement.
	 */
	pit_timer_disable(cpu);
	__raw_writel(delta - 1, pit[cpu]->clkevt_base + PITLDVAL);
	pit_timer_enable(cpu);

	return 0;
}

static int pit_shutdown(struct clock_event_device *evt)
{
	pit_timer_disable(evt_cpu(evt));
	return 0;
}

static int pit_set_periodic(struct clock_event_device *evt)
{
	struct pit_timer *pt = container_of(evt, struct pit_timer,
					    clockevent_pit);

	pit_set_next_event(pt->cycle_per_jiffy, evt);
	return 0;
}

static irqreturn_t pit_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	unsigned int cpu = evt_cpu(evt);

	pit_irq_acknowledge(cpu);

	/*
	 * pit hardware doesn't support oneshot, it will generate an interrupt
	 * and reload the counter value from PITLDVAL when PITCVAL reach zero,
	 * and start the counter again. So software need to disable the timer
	 * to stop the counter loop in ONESHOT mode.
	 */
	if (likely(clockevent_state_oneshot(evt)))
		pit_timer_disable(cpu);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int pit_clockevent_init(unsigned int cpu, unsigned long rate, int irq)
{
	int ret;

	__raw_writel(0, pit[cpu]->clkevt_base + PITTCTRL);

	pit[cpu]->clockevent_pit.name = TIMER_NAME;
	pit[cpu]->clockevent_pit.features = CLOCK_EVT_FEAT_PERIODIC |
					    CLOCK_EVT_FEAT_ONESHOT;
	pit[cpu]->clockevent_pit.set_state_shutdown = pit_shutdown;
	pit[cpu]->clockevent_pit.set_state_periodic = pit_set_periodic;
	pit[cpu]->clockevent_pit.set_next_event = pit_set_next_event;
	pit[cpu]->clockevent_pit.rating = CONFIG_PIT_CLKEVT_RATE;
	pit[cpu]->clockevent_pit.cpumask = cpumask_of(cpu);
	pit[cpu]->clockevent_pit.irq = irq;

	pit[cpu]->pit_timer_irq.name = TIMER_NAME;
	pit[cpu]->pit_timer_irq.flags = IRQF_TIMER | IRQF_IRQPOLL;
	pit[cpu]->pit_timer_irq.handler = pit_timer_interrupt;
	pit[cpu]->pit_timer_irq.dev_id = &pit[cpu]->clockevent_pit;

	ret = setup_irq(irq, &pit[cpu]->pit_timer_irq);
	if (ret)
		return ret;

	ret = irq_force_affinity(irq, cpumask_of(cpu));
	if (ret)
		return ret;

	/*
	 * The value for the LDVAL register trigger is calculated as:
	 * LDVAL trigger = (period / clock period) - 1
	 * The pit is a 32-bit down count timer, when the conter value
	 * reaches 0, it will generate an interrupt, thus the minimal
	 * LDVAL trigger value is 1. And then the min_delta is
	 * minimal LDVAL trigger value + 1, and the max_delta is full 32-bit.
	 */
	clockevents_config_and_register(&pit[cpu]->clockevent_pit, rate, 2,
					0xffffffff);

	__raw_writel(PITTFLG_TIF, pit[cpu]->clkevt_base + PITTFLG);

	return 0;
}

static int pit_timer_starting_cpu(unsigned int cpu)
{
	if (cpu < PIT_NR)
		return pit_clockevent_init(cpu,
					   pit[cpu]->cycle_per_jiffy * (HZ),
					   pit[cpu]->irq);
	return 0;
}

static int pit_timer_dying_cpu(unsigned int cpu)
{
	if (cpu < PIT_NR)
		pit_timer_disable(cpu);
	return 0;
}

static int __init pit_timer_init(struct device_node *np)
{
	void __iomem *timer_base;
	unsigned long clk_rate;
	unsigned int core;
	int ret;

	of_property_read_u32(np, "core", &core);

	if (core > PIT_NR) {
		pr_err("Please specify a correct core number.\n");
		return -EINVAL;
	}

	pit[core] = kzalloc(sizeof(struct pit_timer), GFP_KERNEL);
	if (!pit[core])
		return -ENOMEM;

	pit[core]->core = core;

	timer_base = of_iomap(np, 0);
	if (!timer_base) {
		pr_err("Failed to iomap\n");
		return -ENXIO;
	}

	/*
	 * PIT0 and PIT1 can be chained to build a 64-bit timer,
	 * so choose PIT2 as clocksource, PIT3 as clockevent device,
	 * and leave PIT0 and PIT1 unused for anyone else who needs them.
	 */
	pit[core]->clksrc_base = timer_base + PITn_OFFSET(2);
	pit[core]->clkevt_base = timer_base + PITn_OFFSET(3);

	pit[core]->irq = irq_of_parse_and_map(np, 0);
	if (pit[core]->irq <= 0)
		return -EINVAL;

	pit[core]->pit_clk = of_clk_get(np, 0);
	if (IS_ERR(pit[core]->pit_clk))
		return PTR_ERR(pit[core]->pit_clk);

	ret = clk_prepare_enable(pit[core]->pit_clk);
	if (ret)
		return ret;

	clk_rate = clk_get_rate(pit[core]->pit_clk);
	pit[core]->cycle_per_jiffy = clk_rate / (HZ);

	/* enable the pit module */
	__raw_writel(~PITMCR_MDIS, timer_base + PITMCR);

	if (core == MASTER_CORE) {
		ret = cpuhp_setup_state_nocalls(CPUHP_AP_VF_PIT_TIMER_STARTING,
						"AP_VF_PIT_TIMER_STARTING",
						pit_timer_starting_cpu,
						pit_timer_dying_cpu);
		if (ret)
			return ret;
		ret = pit_clocksource_init(core, clk_rate);
		if (ret)
			return ret;
		ret = pit_clockevent_init(core, clk_rate, pit[core]->irq);
		if (ret)
			return ret;
	}

	return 0;
}

TIMER_OF_DECLARE(vf610, "fsl,vf610-pit", pit_timer_init);
