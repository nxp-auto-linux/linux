// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 * Copyright 2018,2021 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/cpuhotplug.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/sched_clock.h>

/*
 * Each stm takes 0x10 Bytes register space
 */
#define STM_CR		0x00
#define STM_CNT		0x04
#define STM_CH(n)	(0x10 * ((n) + 1))

#define STM_CR_CPS	(0x0 << 8)
#define STM_CR_FRZ	(0x1 << 1)
#define STM_CR_TEN	(0x1 << 0)

#define STM_CCR		0x00
#define STM_CIR		0x04
#define STM_CMP		0x08

#define STM_CCR_CEN	(0x1 << 0)
#define STM_CIR_CIF	(0x1 << 0)

#define MASTER_CPU	0
#define STM_TIMER_NAME	"FSL stm timer"

struct stm_work {
	struct work_struct work;
	int status;
};

struct stm_timer {
	void __iomem *timer_base;
	void __iomem *clkevt_base;
	int irq;
	unsigned int cpu;
	struct clk *stm_clk;
	struct device *dev;
	unsigned long cycle_per_jiffy;
	struct clock_event_device clockevent_stm;
	unsigned long delta;
	struct list_head list;
	struct clocksource clksrc;
	struct stm_work work;
	u32 saved_cnt;
};

static struct stm_timer *clocksource;
static LIST_HEAD(stms_list);

static struct stm_timer *stm_timer_from_cpu(unsigned int cpu)
{
	struct stm_timer *stm;

	list_for_each_entry(stm, &stms_list, list) {
		if (stm->cpu == cpu)
			return stm;
	}

	return NULL;
}

static inline struct stm_timer *stm_timer_from_evt(struct clock_event_device *evt)
{
	return container_of(evt, struct stm_timer, clockevent_stm);
}

static struct stm_timer *cs_to_stm(struct clocksource *cs)
{
	return container_of(cs, struct stm_timer, clksrc);
}

static struct stm_timer *work_to_stm(struct work_struct *work)
{
	struct stm_work *swork = container_of(work, struct stm_work, work);

	return container_of(swork, struct stm_timer, work);
}

static void enable_stm(struct stm_timer *stm)
{
	/* enable the stm module */
	writel(STM_CR_CPS | STM_CR_FRZ | STM_CR_TEN, stm->timer_base + STM_CR);
}

static void disable_stm(struct stm_timer *stm)
{
	writel(0,  stm->timer_base + STM_CR);
}

static inline void stm_timer_enable(struct stm_timer *stm)
{
	enable_stm(stm);

	/* enable clockevent channel */
	writel(STM_CCR_CEN, stm->clkevt_base + STM_CCR);
}

static inline void stm_timer_disable(struct stm_timer *stm)
{
	/**
	 * The counter is shared between channels and will continue to
	 * be incremented. If STM_CMP value is too small, the next event can
	 * be lost if we don't disable the entire module.
	 * Disabling the entire module, makes STM not suitable as clocksource.
	 */
	writel(0, stm->timer_base + STM_CR);
	writel(0, stm->clkevt_base + STM_CCR);
}

static u32 get_counter(struct stm_timer *stm)
{
	return readl(stm->timer_base + STM_CNT);
}

static inline void stm_irq_acknowledge(struct stm_timer *stm)
{
	u32 val;

	/* clear the interrupt */
	writel(STM_CIR_CIF, stm->clkevt_base + STM_CIR);

	/* update STM_CMP value using the counter value */
	val = get_counter(stm) + stm->delta;
	writel(val, stm->clkevt_base + STM_CMP);
}

static u64 stm_read_sched_clock(void)
{
	return readl(clocksource->timer_base + STM_CNT);
}

static void stm_clksrc_save_cnt(struct stm_timer *stm)
{
	stm->saved_cnt = readl(stm->timer_base + STM_CNT);
}

static void stm_clksrc_suspend(struct clocksource *cs)
{
	struct stm_timer *stm = cs_to_stm(cs);

	disable_stm(stm);
	stm_clksrc_save_cnt(stm);
}

static void stm_clksrc_setcnt(struct stm_timer *stm, u32 cnt)
{
	writel(cnt, stm->timer_base + STM_CNT);
}

static void stm_clksrc_resume(struct clocksource *cs)
{
	struct stm_timer *stm = cs_to_stm(cs);

	stm_clksrc_setcnt(stm,  stm->saved_cnt);
	enable_stm(stm);
}

static u64 stm_clksrc_read(struct clocksource *cs)
{
	struct stm_timer *stm = cs_to_stm(cs);

	return (u64)get_counter(stm);
}

static int __init stm_clocksource_init(struct stm_timer *stm,
				       unsigned long rate)
{
	clocksource = stm;
	local_irq_disable();
	sched_clock_register(stm_read_sched_clock, 32, rate);
	local_irq_enable();

	stm->clksrc.name = "fsl-stm";
	stm->clksrc.rating = CONFIG_STM_CLKSRC_RATE;
	stm->clksrc.read = stm_clksrc_read;
	stm->clksrc.mask = CLOCKSOURCE_MASK(32);
	stm->clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;
	stm->clksrc.suspend = stm_clksrc_suspend;
	stm->clksrc.resume = stm_clksrc_resume;

	return clocksource_register_hz(&stm->clksrc, rate);
}

static int stm_set_next_event(unsigned long delta,
			      struct clock_event_device *evt)
{
	u32 val;
	struct stm_timer *stm = stm_timer_from_evt(evt);

	stm_timer_disable(stm);

	stm->delta = delta;

	val = get_counter(stm) + delta;
	writel(val, stm->clkevt_base + STM_CMP);

	stm_timer_enable(stm);

	return 0;
}

static int stm_shutdown(struct clock_event_device *evt)
{
	stm_timer_disable(stm_timer_from_evt(evt));
	return 0;
}

static int stm_set_periodic(struct clock_event_device *evt)
{
	struct stm_timer *stm = stm_timer_from_evt(evt);

	stm_set_next_event(stm->cycle_per_jiffy, evt);
	return 0;
}

static irqreturn_t stm_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	struct stm_timer *stm = stm_timer_from_evt(evt);

	stm_irq_acknowledge(stm);

	/*
	 * stm hardware doesn't support oneshot, it will generate an interrupt
	 * and start the counter again so software need to disable the timer
	 * to stop the counter loop in ONESHOT mode.
	 */
	if (likely(clockevent_state_oneshot(evt)))
		stm_timer_disable(stm);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int stm_clockevent_init(struct stm_timer *stm, unsigned long rate,
			       int irq)
{
	int ret;

	writel(0, stm->clkevt_base + STM_CCR);

	stm->clockevent_stm.name = STM_TIMER_NAME;
	stm->clockevent_stm.features = CLOCK_EVT_FEAT_PERIODIC |
					    CLOCK_EVT_FEAT_ONESHOT;
	stm->clockevent_stm.set_state_shutdown = stm_shutdown;
	stm->clockevent_stm.set_state_periodic = stm_set_periodic;
	stm->clockevent_stm.set_next_event = stm_set_next_event;
	stm->clockevent_stm.rating = CONFIG_STM_CLKEVT_RATE;
	stm->clockevent_stm.cpumask = cpumask_of(stm->cpu);
	stm->clockevent_stm.irq = irq;

	irq_set_status_flags(irq, IRQ_NOAUTOEN);
	ret = request_irq(irq, stm_timer_interrupt,
			  IRQF_TIMER | IRQF_NOBALANCING,
			  STM_TIMER_NAME, &stm->clockevent_stm);
	if (ret)
		return ret;

	ret = irq_force_affinity(irq, cpumask_of(stm->cpu));
	if (ret)
		return ret;

	clockevents_config_and_register(&stm->clockevent_stm, rate, 1,
					0xffffffff);

	writel(STM_CIR_CIF, stm->clkevt_base + STM_CIR);
	enable_irq(irq);

	return 0;
}

static void stm_clockevent_deinit(struct stm_timer *stm)
{
	int irq = stm->clockevent_stm.irq;

	disable_irq_nosync(irq);
	free_irq(irq, &stm->clockevent_stm);
}

static int stm_timer_starting_cpu(unsigned int cpu)
{
	struct stm_timer *stm = stm_timer_from_cpu(cpu);

	if (!stm)
		return 0;

	return stm_clockevent_init(stm, stm->cycle_per_jiffy * (HZ),
				   stm->irq);
}

static int stm_timer_dying_cpu(unsigned int cpu)
{
	struct stm_timer *stm = stm_timer_from_cpu(cpu);

	if (!stm)
		return 0;

	stm_clockevent_deinit(stm);
	stm_timer_disable(stm);

	return 0;
}

static void register_clkevent_work(struct work_struct *work)
{
	struct stm_timer *stm = work_to_stm(work);
	unsigned long clk_rate = clk_get_rate(stm->stm_clk);
	int ret;

	ret = stm_clockevent_init(stm, clk_rate, stm->irq);
	if (ret)
		dev_err(stm->dev, "Failed to register STM clockevent\n");

	stm->work.status = ret;
}

static int __init fsl_stm_timer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *timer_base;
	unsigned long clk_rate;
	unsigned int cpu;
	int ret;
	struct stm_timer *stm;
	static bool added_hp_clbs;

	of_property_read_u32(np, "cpu", &cpu);
	if (cpu >= num_possible_cpus()) {
		pr_err("%s: please specify a cpu number between 0 and %d.\n",
		       STM_TIMER_NAME, num_possible_cpus() - 1);
		return -EINVAL;
	}

	stm = devm_kzalloc(dev, sizeof(*stm), GFP_KERNEL);
	if (stm == NULL)
		return -ENOMEM;

	INIT_WORK(&stm->work.work, register_clkevent_work);
	stm->dev = dev;

	platform_set_drvdata(pdev, stm);
	list_add_tail(&stm->list, &stms_list);

	stm->cpu = cpu;

	timer_base = devm_of_iomap(dev, np, 0, NULL);
	if (IS_ERR(timer_base)) {
		dev_err(dev, "Failed to iomap\n");
		return PTR_ERR(timer_base);
	}

	stm->timer_base = timer_base;

	/* use channel 0 as clockevent */
	stm->clkevt_base = timer_base + STM_CH(0);

	stm->irq = irq_of_parse_and_map(np, 0);
	if (stm->irq <= 0)
		return -EINVAL;

	stm->stm_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(stm->stm_clk)) {
		dev_err(dev, "Clock not found\n");
		return PTR_ERR(stm->stm_clk);
	}

	ret = clk_prepare_enable(stm->stm_clk);
	if (ret)
		return ret;

	clk_rate = clk_get_rate(stm->stm_clk);
	stm->cycle_per_jiffy = clk_rate / (HZ);

	if (!added_hp_clbs) {
		ret = cpuhp_setup_state_nocalls(CPUHP_AP_FSL_STM_TIMER_STARTING,
						"STM timer:starting",
						stm_timer_starting_cpu,
						stm_timer_dying_cpu);
		added_hp_clbs = true;
	}

	if (cpu == MASTER_CPU) {
		ret = stm_clocksource_init(stm, clk_rate);
		if (ret)
			return ret;
	}

	/* Register event on requested CPU */
	schedule_work_on(cpu, &stm->work.work);
	flush_work(&stm->work.work);

	/* reset counter value */
	stm_clksrc_setcnt(stm, 0);
	enable_stm(stm);

	return 0;
}

static int __maybe_unused fsl_stm_resume(struct device *dev)
{
	struct stm_timer *stm = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(stm->stm_clk);
	if (ret)
		return ret;

	enable_stm(stm);

	return 0;
}

static int fsl_stm_timer_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

static const struct of_device_id fsl_stm_of_match[] = {
	{ .compatible = "nxp,s32cc-stm", },
	{},
};
MODULE_DEVICE_TABLE(of, fsl_stm_of_match);

static SIMPLE_DEV_PM_OPS(stm_timer_pm_ops, NULL, fsl_stm_resume);

static struct platform_driver fsl_stm_probe = {
	.probe	= fsl_stm_timer_probe,
	.remove = fsl_stm_timer_remove,
	.driver	= {
		.name = "fsl-stm",
		.of_match_table = of_match_ptr(fsl_stm_of_match),
		.pm = &stm_timer_pm_ops,
	},
};
module_platform_driver(fsl_stm_probe);

MODULE_DESCRIPTION("NXP System Timer Module driver");
MODULE_LICENSE("GPL v2");
