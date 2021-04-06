// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2012-2013 Freescale Semiconductor, Inc.
 * Copyright 2021 NXP
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

struct pit_work {
	struct work_struct work;
	int status;
};

struct pit_timer {
	void __iomem *module_base;
	void __iomem *clksrc_base;
	void __iomem *clkevt_base;
	int irq;
	unsigned int cpu;
	struct clk *pit_clk;
	struct device *dev;
	unsigned long cycle_per_jiffy;
	struct clock_event_device clockevent_pit;
	struct list_head list;
	struct clocksource clksrc;
	struct pit_work work;
	u32 saved_cnt;
};

static struct pit_timer *clocksource;

static inline struct pit_timer *evt_pit_timer(struct clock_event_device *evt)
{
	return container_of(evt, struct pit_timer, clockevent_pit);
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

static struct pit_timer *cs_to_pit(struct clocksource *cs)
{
	return container_of(cs, struct pit_timer, clksrc);
}

static struct pit_timer *work_to_pit(struct work_struct *work)
{
	struct pit_work *pwork = container_of(work, struct pit_work, work);

	return container_of(pwork, struct pit_timer, work);
}

static u64 pit_read_down_val(struct pit_timer *pit)
{
	return ~__raw_readl(pit->clksrc_base + PITCVAL);
}

static u64 notrace pit_read_sched_clock(void)
{
	return pit_read_down_val(clocksource);
}

static u64 pit_clksrc_read(struct clocksource *cs)
{
	struct pit_timer *pit = cs_to_pit(cs);

	return pit_read_down_val(pit);
}

static void pit_clksrc_disable(struct pit_timer *pit)
{
	__raw_writel(0,  pit->clksrc_base + PITTCTRL);
}

static void pit_clksrc_enable(struct pit_timer *pit)
{
	__raw_writel(PITTCTRL_TEN,  pit->clksrc_base + PITTCTRL);
}

static void pit_clksrc_loadval(struct pit_timer *pit, u32 val)
{
	__raw_writel(val, pit->clksrc_base + PITLDVAL);
}

static void pit_clksrc_save_cnt(struct pit_timer *pit)
{
	pit->saved_cnt = __raw_readl(pit->clksrc_base + PITCVAL);
}

static void pit_clksrc_suspend(struct clocksource *cs)
{
	struct pit_timer *pit = cs_to_pit(cs);

	pit_clksrc_save_cnt(pit);
	pit_clksrc_disable(pit);
}

static void pit_clksrc_resume(struct clocksource *cs)
{
	struct pit_timer *pit = cs_to_pit(cs);

	pit_clksrc_disable(clocksource);

	if (pit->saved_cnt) {
		pit_clksrc_loadval(pit, pit->saved_cnt);
		pit_clksrc_enable(clocksource);
		/**
		 * From PIT documentation:
		 *   Writing a new value to this register does not restart the
		 *   timer; instead the value is loaded after the timer expires.
		 * Writing 0xFFFFFFFFU will artificially reset the conter once
		 * pit->saved_cnt expires.
		 */
		pit_clksrc_loadval(clocksource, 0xFFFFFFFFU);

		return;
	}

	pit_clksrc_loadval(clocksource, 0xFFFFFFFFU);
	pit_clksrc_enable(clocksource);
}

static int __init pit_clocksource_init(struct pit_timer *pit,
				       unsigned long rate)
{
	clocksource = pit;

	pit_clksrc_disable(clocksource);
	pit_clksrc_loadval(clocksource, 0xFFFFFFFFU);
	pit_clksrc_enable(clocksource);

	local_irq_disable();
	sched_clock_register(pit_read_sched_clock, 32, rate);
	local_irq_enable();

	pit->clksrc.name = "vf-pit";
	pit->clksrc.rating = CONFIG_PIT_CLKSRC_RATE;
	pit->clksrc.read = pit_clksrc_read;
	pit->clksrc.mask = CLOCKSOURCE_MASK(32);
	pit->clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;
	pit->clksrc.suspend = pit_clksrc_suspend;
	pit->clksrc.resume = pit_clksrc_resume;

	return clocksource_register_hz(&pit->clksrc, rate);
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

static void enable_pit(struct pit_timer *pit)
{
	__raw_writel(~PITMCR_MDIS, pit->module_base + PITMCR);
}

static void register_clkevent_work(struct work_struct *work)
{
	struct pit_timer *pit = work_to_pit(work);
	unsigned long clk_rate = clk_get_rate(pit->pit_clk);
	int ret;

	ret = pit_clockevent_init(pit, clk_rate, pit->irq);
	if (ret)
		dev_err(pit->dev, "Failed to register PIT clockevent\n");

	pit->work.status = ret;
}

static int __init fsl_pit_timer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *timer_base;
	unsigned long clk_rate;
	unsigned int cpu;
	int ret;
	struct pit_timer *pit;

	of_property_read_u32(np, "cpu", &cpu);
	if (cpu >= num_possible_cpus()) {
		dev_err(dev, "Please specify a cpu number between 0 and %d.\n",
			num_possible_cpus() - 1);
		return -EINVAL;
	}

	pit = devm_kzalloc(dev, sizeof(*pit), GFP_KERNEL);
	if (pit == NULL)
		return -ENOMEM;

	INIT_WORK(&pit->work.work, register_clkevent_work);
	pit->dev = dev;

	platform_set_drvdata(pdev, pit);

	pit->cpu = cpu;

	timer_base = devm_of_iomap(dev, np, 0, NULL);
	if (IS_ERR(timer_base)) {
		dev_err(dev, "Failed to iomap\n");
		return PTR_ERR(timer_base);
	}

	/*
	 * PIT0 and PIT1 can be chained to build a 64-bit timer,
	 * so choose PIT2 as clocksource, PIT3 as clockevent device,
	 * and leave PIT0 and PIT1 unused for anyone else who needs them.
	 */
	pit->module_base = timer_base;
	pit->clksrc_base = timer_base + PIT_CH(2);
	pit->clkevt_base = timer_base + PIT_CH(3);

	pit->irq = irq_of_parse_and_map(np, 0);
	if (pit->irq <= 0) {
		dev_err(dev, "Failed to get IRQ\n");
		return -EINVAL;
	}

	pit->pit_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(pit->pit_clk)) {
		dev_err(dev, "Clock not found\n");
		return PTR_ERR(pit->pit_clk);
	}

	ret = clk_prepare_enable(pit->pit_clk);
	if (ret)
		return ret;

	clk_rate = clk_get_rate(pit->pit_clk);
	pit->cycle_per_jiffy = clk_rate / (HZ);

	/* enable the pit module */
	enable_pit(pit);

	if (cpu == MASTER_CPU) {
		ret = pit_clocksource_init(pit, clk_rate);
		if (ret) {
			dev_err(dev, "Failed to register PIT as clocksource\n");
			return ret;
		}
	}

	/* Register event on requested CPU */
	schedule_work_on(cpu, &pit->work.work);
	flush_work(&pit->work.work);

	return pit->work.status;
}

static int __maybe_unused fsl_pit_resume(struct device *dev)
{
	struct pit_timer *pit = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(pit->pit_clk);
	if (ret)
		return ret;

	enable_pit(pit);

	return 0;
}

static int fsl_pit_timer_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

static const struct of_device_id fsl_pit_of_match[] = {
	{ .compatible = "fsl,s32v234-pit", },
	{ .compatible = "fsl,vf610-pit", },
	{ .compatible = "fsl,s32gen1-pit", },
	{},
};
MODULE_DEVICE_TABLE(of, fsl_pit_of_match);

static SIMPLE_DEV_PM_OPS(pit_timer_pm_ops, NULL, fsl_pit_resume);

static struct platform_driver fsl_pit_probe = {
	.probe	= fsl_pit_timer_probe,
	.remove = fsl_pit_timer_remove,
	.driver	= {
		.name = "fsl-pit",
		.of_match_table = of_match_ptr(fsl_pit_of_match),
		.pm = &pit_timer_pm_ops,
	},
};
module_platform_driver(fsl_pit_probe);

MODULE_DESCRIPTION("NXP System Timer Module driver");
MODULE_LICENSE("GPL v2");
