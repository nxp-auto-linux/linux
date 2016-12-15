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
#define PIT_CH(n)	(0x100 + 0x10 * (n))
#define PITLDVAL	0x00
#define PITCVAL		0x04
#define PITTCTRL	0x08
#define PITTFLG		0x0c

#define PITMCR_MDIS	(0x1 << 1)

#define PITTCTRL_TEN	(0x1 << 0)
#define PITTCTRL_TIE	(0x1 << 1)
#define PITCTRL_CHN	(0x1 << 2)

#define PITTFLG_TIF	0x1

#define MASTER_CPU	0
#define TIMER_NAME	"FSL pit timer"

#ifndef CONFIG_PIT_CLKSRC_RATE
#define CONFIG_PIT_CLKSRC_RATE	460
#endif

#ifndef CONFIG_PIT_CLKEVT_RATE
#define CONFIG_PIT_CLKEVT_RATE	460
#endif

struct pit_timer {
	void __iomem *clksrc_base;
	void __iomem *clkevt_base;
	int irq;
	unsigned int cpu;
	struct clk *pit_clk;
	unsigned long cycle_per_jiffy;
	struct clock_event_device clockevent_pit;
	struct list_head list;
};

static LIST_HEAD(pits_list);
static struct pit_timer *clocksource;
static bool registered;

static inline struct pit_timer *evt_pit_timer(struct clock_event_device *evt)
{
	return container_of(evt, struct pit_timer, clockevent_pit);
}

static struct pit_timer *cpu_pit_timer(unsigned int cpu)
{
	struct pit_timer *pit;

	list_for_each_entry(pit, &pits_list, list)
		if (pit->cpu == cpu)
			return pit;

	return NULL;
}

static inline void pit_timer_enable(struct pit_timer *pit)
{
	__raw_writel(PITTCTRL_TEN | PITTCTRL_TIE,
		     pit->clkevt_base + PITTCTRL);
}

static inline void pit_timer_disable(struct pit_timer *pit)
{
	__raw_writel(0, pit->clkevt_base + PITTCTRL);
}

static inline void pit_irq_acknowledge(struct pit_timer *pit)
{
	__raw_writel(PITTFLG_TIF, pit->clkevt_base + PITTFLG);
}

static u64 notrace pit_read_sched_clock(void)
{
	return ~__raw_readl(clocksource->clksrc_base + PITCVAL);
}

static int __init pit_clocksource_init(struct pit_timer *pit, unsigned long rate)
{
	clocksource = pit;
	__raw_writel(0,  clocksource->clksrc_base + PITTCTRL);
	__raw_writel(0xFFFFFFFF,  clocksource->clksrc_base + PITLDVAL);
	__raw_writel(PITTCTRL_TEN, clocksource->clksrc_base + PITTCTRL);

	sched_clock_register(pit_read_sched_clock, 32, rate);
	clocksource_mmio_init(clocksource->clksrc_base + PITCVAL,
			      "vf-pit", rate,
			      CONFIG_PIT_CLKSRC_RATE, 32,
			      clocksource_mmio_readl_down);

	return 0;
}

static int pit_set_next_event(unsigned long delta,
			      struct clock_event_device *evt)
{
	struct pit_timer *pit = evt_pit_timer(evt);
	/*
	 * set a new value to PITLDVAL register will not restart the timer,
	 * to abort the current cycle and start a timer period with the new
	 * value, the timer must be disabled and enabled again.
	 * and the PITLAVAL should be set to delta minus one according to pit
	 * hardware requirement.
	 */
	pit_timer_disable(pit);
	__raw_writel(delta - 1, pit->clkevt_base + PITLDVAL);
	pit_timer_enable(pit);

	return 0;
}

static int pit_shutdown(struct clock_event_device *evt)
{
	pit_timer_disable(evt_pit_timer(evt));
	return 0;
}

static int pit_set_periodic(struct clock_event_device *evt)
{
	struct pit_timer *pit = evt_pit_timer(evt);

	pit_set_next_event(pit->cycle_per_jiffy, evt);
	return 0;
}

static irqreturn_t pit_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	struct pit_timer *pit = evt_pit_timer(evt);

	pit_irq_acknowledge(pit);

	/*
	 * pit hardware doesn't support oneshot, it will generate an interrupt
	 * and reload the counter value from PITLDVAL when PITCVAL reach zero,
	 * and start the counter again. So software need to disable the timer
	 * to stop the counter loop in ONESHOT mode.
	 */
	if (likely(clockevent_state_oneshot(evt)))
		pit_timer_disable(pit);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int pit_clockevent_init(struct pit_timer *pit, unsigned long rate,
			       int irq)
{
	int ret;

	__raw_writel(0, pit->clkevt_base + PITTCTRL);

	pit->clockevent_pit.name = TIMER_NAME;
	pit->clockevent_pit.features = CLOCK_EVT_FEAT_PERIODIC |
					    CLOCK_EVT_FEAT_ONESHOT;
	pit->clockevent_pit.set_state_shutdown = pit_shutdown;
	pit->clockevent_pit.set_state_periodic = pit_set_periodic;
	pit->clockevent_pit.set_next_event = pit_set_next_event;
	pit->clockevent_pit.rating = CONFIG_PIT_CLKEVT_RATE;
	pit->clockevent_pit.cpumask = cpumask_of(pit->cpu);
	pit->clockevent_pit.irq = irq;

	ret = request_irq(irq, pit_timer_interrupt, IRQF_TIMER | IRQF_IRQPOLL,
			  TIMER_NAME, &pit->clockevent_pit);
	if (ret)
		return ret;

	ret = irq_force_affinity(irq, cpumask_of(pit->cpu));
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
	clockevents_config_and_register(&pit->clockevent_pit, rate, 2,
					0xffffffff);

	__raw_writel(PITTFLG_TIF, pit->clkevt_base + PITTFLG);

	return 0;
}

static int pit_timer_starting_cpu(unsigned int cpu)
{
	struct pit_timer *pit = cpu_pit_timer(cpu);

	if (pit)
		return pit_clockevent_init(pit, pit->cycle_per_jiffy * (HZ),
					   pit->irq);

	return 0;
}

static int pit_timer_dying_cpu(unsigned int cpu)
{
	struct pit_timer *pit = cpu_pit_timer(cpu);

	if (pit)
		pit_timer_disable(pit);

	return 0;
}

static int __init pit_timer_init(struct device_node *np)
{
	void __iomem *timer_base;
	unsigned long clk_rate;
	unsigned int cpu;
	int ret;
	struct pit_timer *pit;

	of_property_read_u32(np, "cpu", &cpu);
	if (cpu >= num_possible_cpus()) {
		pr_err("%s: please specify a cpu number between 0 and %d.\n",
		       TIMER_NAME, num_possible_cpus() - 1);
		return -EINVAL;
	}

	pit = kzalloc(sizeof(struct pit_timer), GFP_KERNEL);
	if (!pit)
		return -ENOMEM;

	list_add_tail(&pit->list, &pits_list);

	pit->cpu = cpu;

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
	pit->clksrc_base = timer_base + PIT_CH(2);
	pit->clkevt_base = timer_base + PIT_CH(3);

	pit->irq = irq_of_parse_and_map(np, 0);
	if (pit->irq <= 0)
		return -EINVAL;

	pit->pit_clk = of_clk_get(np, 0);
	if (IS_ERR(pit->pit_clk))
		return PTR_ERR(pit->pit_clk);

	ret = clk_prepare_enable(pit->pit_clk);
	if (ret)
		return ret;

	clk_rate = clk_get_rate(pit->pit_clk);
	pit->cycle_per_jiffy = clk_rate / (HZ);

	/* enable the pit module */
	__raw_writel(~PITMCR_MDIS, timer_base + PITMCR);

	if (registered == false) {
		ret = cpuhp_setup_state_nocalls(CPUHP_AP_VF_PIT_TIMER_STARTING,
						"AP_VF_PIT_TIMER_STARTING",
						pit_timer_starting_cpu,
						pit_timer_dying_cpu);
		if (ret)
			return ret;
		registered = true;
	}

	if (cpu == MASTER_CPU) {
		ret = pit_clocksource_init(pit, clk_rate);
		if (ret)
			return ret;
		ret = pit_clockevent_init(pit, clk_rate, pit->irq);
		if (ret)
			return ret;
	}

	return 0;
}

TIMER_OF_DECLARE(vf610, "fsl,vf610-pit", pit_timer_init);
