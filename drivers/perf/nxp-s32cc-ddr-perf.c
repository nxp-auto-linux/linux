// SPDX-License-Identifier: GPL-2.0
/*
 * NXP S32CC DDR Performance Monitor
 *
 * Copyright 2016 Freescale Semiconductor, Inc.
 * Copyright 2017,2020,2022-2023 NXP
 */

#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <soc/s32cc/nvmem_common.h>

#define COUNTER_CNTL		0x0
#define COUNTER_READ		0x20

#define CNTL_OVER		0x1
#define CNTL_CLEAR		0x2
#define CNTL_EN			0x4
#define CNTL_EN_MASK		0xFFFFFFFB
#define CNTL_CLEAR_MASK		0xFFFFFFFD
#define CNTL_OVER_MASK		0xFFFFFFFE

#define CNTL_CSV_MASK		GENMASK(31, 24)
#define CNTL_CP_MASK		GENMASK(23, 16)

#define EVENT_CYCLES_COUNTER	0
#define NUM_COUNTERS		4

#define DDR_GPR_CONFIG_0		0x0
#define DDR_GPR_CONFIG_0_SHIFT		31
#define DDR_GPR_CONFIG_0_PERF_CNT	BIT(DDR_GPR_CONFIG_0_SHIFT)

#define to_ddr_pmu(p)		container_of(p, struct ddr_pmu, pmu)

#define DDR_PERF_DEV_NAME	"s32cc_ddr"
#define DDR_CPUHP_CB_NAME	DDR_PERF_DEV_NAME "_perf_pmu"

static DEFINE_IDA(ddr_ida);

enum ddr_perf_event_id {
	CYCLES_EVT =			0x0,
	SELF_REFRESH_EVT =		0x1,
	WRITE_ACCESSES_EVT =		0x5,
	WRITE_QUEUE_DEPTH_EVT =		0x9,
	LP_READ_CREDIT_CNT_EVT =	0x10,
	HP_READ_CREDIT_CNT_EVT =	0x11,
	WRITE_CREDIT_CNT_EVT =		0x12,
	READ_COMMAND_EVT =		0x20,
	WRITE_COMMAND_EVT =		0x21,
	READ_MODIFY_WRITE_COMMAND_EVT = 0x22,
	HP_READ_COMMAND_EVT =		0x23,
	HP_READ_REQ_NOCREDIT_EVT =	0x24,
	HP_READ_XACT_CREDIT_EVT =	0x25,
	LP_READ_REQ_NOCREDIT_EVT =	0x26,
	LP_READ_XACT_CREDIT_EVT =	0x27,
	WRITE_XACT_CREDIT_EVT =		0x29,
	READ_CYCLES_EVT =		0x2a,
	WRITE_CYCLES_EVT =		0x2b,
	READ_WRITE_TRANSITION_EVT =	0x30,
	PRECHARGE_EVT =			0x31,
	ACTIVATE_EVT =			0x32,
	LOAD_MODE_EVT =			0x33,
	MASKED_WRITE_EVT =		0x34,
	READ_EVT =			0x35,
	READ_ACTIVATE_EVT =		0x36,
	REFRESH_EVT =			0x37,
	WRITE_EVT =			0x38,
	RAW_HAZARD_EVT =		0x39
};

struct ddr_pmu {
	struct pmu pmu;
	void __iomem *base;
	unsigned int cpu;
	struct	hlist_node node;
	struct	device *dev;
	struct perf_event *events[NUM_COUNTERS];
	int active_events;
	enum cpuhp_state cpuhp_state;
	unsigned int irq;
	unsigned int id;
};

static ssize_t ddr_perf_cpumask_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct ddr_pmu *pmu = dev_get_drvdata(dev);

	return cpumap_print_to_pagebuf(true, buf, cpumask_of(pmu->cpu));
}

static struct device_attribute ddr_perf_cpumask_attr =
	__ATTR(cpumask, 0444, ddr_perf_cpumask_show, NULL);

static struct attribute *ddr_perf_cpumask_attrs[] = {
	&ddr_perf_cpumask_attr.attr,
	NULL,
};

static struct attribute_group ddr_perf_cpumask_attr_group = {
	.attrs = ddr_perf_cpumask_attrs,
};

static ssize_t
ddr_pmu_event_show(struct device *dev, struct device_attribute *attr,
		   char *page)
{
	struct perf_pmu_events_attr *pmu_attr;

	pmu_attr = container_of(attr, struct perf_pmu_events_attr, attr);
	return sprintf(page, "event=0x%02llx\n", pmu_attr->id);
}

#define S32CC_DDR_PMU_EVENT_ATTR(_name, _id)				\
	(&((struct perf_pmu_events_attr[]) {				\
		{ .attr = __ATTR(_name, 0444, ddr_pmu_event_show, NULL),\
		  .id = _id, }						\
	})[0].attr.attr)

static struct attribute *ddr_perf_events_attrs[] = {
	S32CC_DDR_PMU_EVENT_ATTR(cycles, CYCLES_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(self-refresh, SELF_REFRESH_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(write-accesses, WRITE_ACCESSES_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(write-queue-depth, WRITE_QUEUE_DEPTH_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(lp-read-credit-cnt, LP_READ_CREDIT_CNT_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(hp-read-credit-cnt, HP_READ_CREDIT_CNT_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(write-credit-cnt, WRITE_CREDIT_CNT_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(read-command, READ_COMMAND_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(write-command, WRITE_COMMAND_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(read-modify-write-command,
			       READ_MODIFY_WRITE_COMMAND_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(hp-read, HP_READ_COMMAND_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(hp-req-nocredit, HP_READ_REQ_NOCREDIT_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(hp-xact-credit, HP_READ_XACT_CREDIT_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(lp-req-nocredit, LP_READ_REQ_NOCREDIT_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(lp-xact-credit, LP_READ_XACT_CREDIT_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(wr-xact-credit, WRITE_XACT_CREDIT_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(read-cycles, READ_CYCLES_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(write-cycles, WRITE_CYCLES_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(read-write-transition,
			       READ_WRITE_TRANSITION_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(precharge, PRECHARGE_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(activate, ACTIVATE_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(load-mode, LOAD_MODE_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(perf-mwr, MASKED_WRITE_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(read, READ_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(read-activate, READ_ACTIVATE_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(refresh, REFRESH_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(write, WRITE_EVT),
	S32CC_DDR_PMU_EVENT_ATTR(raw-hazard, RAW_HAZARD_EVT),
	NULL,
};

static struct attribute_group ddr_perf_events_attr_group = {
	.name = "events",
	.attrs = ddr_perf_events_attrs,
};

PMU_FORMAT_ATTR(event, "config:0-7");
PMU_FORMAT_ATTR(counter_cp, "config:8-15");

static struct attribute *ddr_perf_format_attrs[] = {
	&format_attr_event.attr,
	&format_attr_counter_cp.attr,
	NULL,
};

static struct attribute_group ddr_perf_format_attr_group = {
	.name = "format",
	.attrs = ddr_perf_format_attrs,
};

static const struct attribute_group *attr_groups[] = {
	&ddr_perf_events_attr_group,
	&ddr_perf_format_attr_group,
	&ddr_perf_cpumask_attr_group,
	NULL,
};

static int ddr_perf_alloc_counter(struct ddr_pmu *pmu, unsigned int event)
{
	int i;

	/*
	 * Always map cycle event to counter 0
	 * Cycles counter is dedicated for cycle event
	 * can't be used for the other events
	 */
	if (event == CYCLES_EVT) {
		if (!pmu->events[EVENT_CYCLES_COUNTER])
			return EVENT_CYCLES_COUNTER;
		else
			return -ENOENT;
	}

	for (i = 1; i < NUM_COUNTERS; i++) {
		if (!pmu->events[i])
			return i;
	}

	return -ENOENT;
}

static void ddr_perf_free_counter(struct ddr_pmu *pmu, int counter)
{
	if (counter < 0 || counter >= NUM_COUNTERS)
		return;

	pmu->events[counter] = NULL;
}

static u32 ddr_perf_read_counter_cntl(struct ddr_pmu *pmu, int counter)
{
	return ioread32(pmu->base + COUNTER_CNTL + counter * sizeof(u32));
}

static u32 ddr_perf_read_counter_value(struct ddr_pmu *pmu, int counter)
{
	return ioread32(pmu->base + COUNTER_READ + counter * sizeof(u32));
}

static u32 ddr_perf_read_counter(struct ddr_pmu *pmu, int counter, bool value)
{
	if (counter >= NUM_COUNTERS)
		return -ENOENT;

	if (value)
		return ddr_perf_read_counter_value(pmu, counter);
	return ddr_perf_read_counter_cntl(pmu, counter);
}

static void ddr_perf_write_counter(struct ddr_pmu *pmu, int counter, u32 val)
{
	if (counter >= NUM_COUNTERS)
		return;

	iowrite32(val, pmu->base + COUNTER_CNTL + counter * sizeof(u32));
}

static int ddr_perf_event_init(struct perf_event *event)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	struct perf_event *sibling;

	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	if (is_sampling_event(event) || event->attach_state & PERF_ATTACH_TASK)
		return -EOPNOTSUPP;

	if (event->cpu < 0) {
		dev_warn(pmu->dev, "Can't provide per-task data!\n");
		return -EOPNOTSUPP;
	}

	/*
	 * We must NOT create groups containing mixed PMUs, although software
	 * events are acceptable (for example to create a CCN group
	 * periodically read when a hrtimer aka cpu-clock leader triggers).
	 */
	if (event->group_leader->pmu != event->pmu &&
	    !is_software_event(event->group_leader))
		return -EINVAL;

	for_each_sibling_event(sibling, event->group_leader) {
		if (sibling->pmu != event->pmu && !is_software_event(sibling))
			return -EINVAL;
	}

	event->cpu = pmu->cpu;
	hwc->idx = -1;

	return 0;
}

static void ddr_perf_counter_clear(struct ddr_pmu *pmu, int counter)
{
	unsigned int val;

	val = ddr_perf_read_counter(pmu, counter, false);
	val &= ~CNTL_CLEAR;
	ddr_perf_write_counter(pmu, counter, val);

	/* Cycles counter also needs CLEAR bit set to reset */
	if (counter == EVENT_CYCLES_COUNTER) {
		val |= CNTL_CLEAR;
		ddr_perf_write_counter(pmu, counter, val);
	}
}

static bool ddr_perf_counter_disabled(struct ddr_pmu *pmu, int counter)
{
	unsigned int val, is_enabled, is_cleared, is_csv_set;

	val = ddr_perf_read_counter(pmu, counter, false);
	is_enabled = val & CNTL_EN;
	is_cleared = val & CNTL_CLEAR;
	is_csv_set = val & CNTL_CSV_MASK;

	return !is_enabled && is_cleared && !is_csv_set;
}

static void ddr_perf_counter_stop(struct ddr_pmu *pmu, int counter)
{
	unsigned int val;

	/* Write 0 in CSV field and CNTR_EN */
	val = ddr_perf_read_counter(pmu, counter, false);
	val &= ~CNTL_CSV_MASK;
	val &= ~CNTL_EN;

	ddr_perf_write_counter(pmu, counter, val);
}

static unsigned int ddr_perf_counter_cp_mask_by_event(unsigned int config)
{
	unsigned int event_id, counter_cp, event_mask;

	/* Bits [7:0] of the "config" attribute */
	event_id = config & 0xFF;
	/* Bits [15:8] of the "config" attribute */
	counter_cp = (config >> 8) & 0xFF;

	switch (event_id) {
	case SELF_REFRESH_EVT:
		event_mask = GENMASK(1, 0);
		break;
	case WRITE_QUEUE_DEPTH_EVT:
		event_mask = GENMASK(3, 0);
		break;
	case LP_READ_CREDIT_CNT_EVT:
	case HP_READ_CREDIT_CNT_EVT:
	case WRITE_CREDIT_CNT_EVT:
		event_mask = GENMASK(6, 0);
		break;
	default:
		event_mask = 0;
		break;
	}

	return counter_cp & event_mask;
}

static void ddr_perf_counter_enable(struct ddr_pmu *pmu, unsigned int config,
				    int counter, bool enable)
{
	u32 val, counter_cp;

	counter_cp = ddr_perf_counter_cp_mask_by_event(config);

	if (enable) {
		/*
		 * must disable first, then enable again
		 * otherwise, cycle counter will not work
		 * if previous state is enabled.
		 */
		ddr_perf_write_counter(pmu, counter, 0);
		val = CNTL_EN | CNTL_CLEAR;
		val |= FIELD_PREP(CNTL_CSV_MASK, config);
		val |= FIELD_PREP(CNTL_CP_MASK, counter_cp);
		ddr_perf_write_counter(pmu, counter, val);
	} else {
		/* Disable counter without resetting it */
		val = ddr_perf_read_counter(pmu, counter, false);
		val &= CNTL_EN_MASK;
		ddr_perf_write_counter(pmu, counter, val);
	}
}

static void ddr_perf_event_update(struct perf_event *event)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	u64 new_raw_count;
	int counter = hwc->idx;

	new_raw_count = ddr_perf_read_counter(pmu, counter, true);
	local64_add(new_raw_count, &event->count);

	/* Clear counter after each event update to prevent overflow */
	ddr_perf_counter_clear(pmu, counter);

	/* Check if counter was previously stopped and enable it */
	if (ddr_perf_counter_disabled(pmu, counter))
		ddr_perf_counter_enable(pmu, event->attr.config, counter,
					true);
}

static void ddr_perf_event_start(struct perf_event *event, int flags)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int counter = hwc->idx;

	local64_set(&hwc->prev_count, 0);

	ddr_perf_counter_enable(pmu, event->attr.config, counter, true);

	hwc->state = 0;
}

static int ddr_perf_event_add(struct perf_event *event, int flags)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int counter;
	unsigned int cfg = event->attr.config;

	counter = ddr_perf_alloc_counter(pmu, cfg);
	if (counter < 0) {
		dev_dbg(pmu->dev, "There are not enough counters\n");
		return -EOPNOTSUPP;
	}

	pmu->events[counter] = event;
	pmu->active_events++;
	hwc->idx = counter;

	hwc->state |= PERF_HES_STOPPED;

	if (flags & PERF_EF_START)
		ddr_perf_event_start(event, flags);

	return 0;
}

static void ddr_perf_event_stop(struct perf_event *event, int flags)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int counter = hwc->idx;

	ddr_perf_counter_enable(pmu, event->attr.config, counter, false);
	ddr_perf_event_update(event);

	hwc->state |= PERF_HES_STOPPED;
}

static void ddr_perf_event_del(struct perf_event *event, int flags)
{
	struct ddr_pmu *pmu = to_ddr_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int counter = hwc->idx;

	ddr_perf_event_stop(event, PERF_EF_UPDATE);

	ddr_perf_free_counter(pmu, counter);
	pmu->active_events--;
	hwc->idx = -1;
}

static void ddr_perf_pmu_enable(struct pmu *pmu)
{
	struct ddr_pmu *ddr_pmu = to_ddr_pmu(pmu);

	/* enable cycle counter if cycle is not active event list */
	if (!ddr_pmu->events[EVENT_CYCLES_COUNTER])
		ddr_perf_counter_enable(ddr_pmu, CYCLES_EVT,
					EVENT_CYCLES_COUNTER, true);
}

static void ddr_perf_pmu_disable(struct pmu *pmu)
{
	struct ddr_pmu *ddr_pmu = to_ddr_pmu(pmu);

	if (!ddr_pmu->events[EVENT_CYCLES_COUNTER])
		ddr_perf_counter_enable(ddr_pmu, CYCLES_EVT,
					EVENT_CYCLES_COUNTER, false);
}

static int ddr_perf_init(struct ddr_pmu *pmu, void __iomem *base,
			 struct device *dev)
{
	*pmu = (struct ddr_pmu) {
		.pmu = (struct pmu) {
			.capabilities = PERF_PMU_CAP_NO_EXCLUDE,
			.task_ctx_nr = perf_invalid_context,
			.attr_groups = attr_groups,
			.event_init = ddr_perf_event_init,
			.add = ddr_perf_event_add,
			.del = ddr_perf_event_del,
			.start = ddr_perf_event_start,
			.stop = ddr_perf_event_stop,
			.read = ddr_perf_event_update,
			.pmu_enable = ddr_perf_pmu_enable,
			.pmu_disable = ddr_perf_pmu_disable,
		},
		.base = base,
		.dev = dev,
	};

	pmu->id = ida_simple_get(&ddr_ida, 0, 0, GFP_KERNEL);
	return pmu->id;
}

static irqreturn_t ddr_perf_irq_handler(int irq, void *p)
{
	int i;
	unsigned int cntl;
	struct ddr_pmu *pmu = (struct ddr_pmu *)p;
	struct perf_event *event;

	/* If no overflow on counter0, return */
	cntl = ddr_perf_read_counter(pmu, EVENT_CYCLES_COUNTER, false);
	cntl &= CNTL_OVER;
	if (!cntl)
		return IRQ_NONE;

	/* all counter will stop if cycle counter disabled */
	ddr_perf_counter_enable(pmu, CYCLES_EVT, EVENT_CYCLES_COUNTER,
				false);
	/*
	 * When the cycle counter overflows, all counters are stopped,
	 * and an IRQ is raised. If any other counter overflows, it
	 * continues counting, and no IRQ is raised.
	 *
	 * Cycles occur at least 4 times as often as other events, so we
	 * can update all events on a cycle counter overflow and not
	 * lose events.
	 *
	 */

	/* Stop all counters first, to not sketch the measurement */
	for (i = 0; i < NUM_COUNTERS; i++) {
		if (!pmu->events[i])
			continue;

		ddr_perf_counter_stop(pmu, i);
	}

	/* Read all counter values and re-enable them */
	for (i = 0; i < NUM_COUNTERS; i++) {
		if (!pmu->events[i])
			continue;

		event = pmu->events[i];
		ddr_perf_event_update(event);
	}

	ddr_perf_counter_enable(pmu, CYCLES_EVT, EVENT_CYCLES_COUNTER,
				true);

	return IRQ_HANDLED;
}

static int ddr_perf_offline_cpu(unsigned int cpu, struct hlist_node *node)
{
	struct ddr_pmu *pmu = hlist_entry_safe(node, struct ddr_pmu, node);
	unsigned int target;

	if (!pmu)
		return 0;

	if (cpu != pmu->cpu)
		return 0;

	target = cpumask_any_but(cpu_online_mask, cpu);
	if (target >= nr_cpu_ids)
		return 0;

	perf_pmu_migrate_context(&pmu->pmu, cpu, target);
	pmu->cpu = target;

	WARN_ON(irq_set_affinity_hint(pmu->irq, cpumask_of(pmu->cpu)));

	return 0;
}

static int ddr_perf_probe(struct platform_device *pdev)
{
	struct ddr_pmu *pmu;
	struct nvmem_cell *cell;
	void __iomem *base;
	char *name;
	int num;
	int ret;
	int irq;

	dev_info(&pdev->dev, "probing device\n");

	/* Check if NVMEM for DDR_GPR is available */
	cell = nvmem_cell_get(&pdev->dev, "ddr_pmu_irq");
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	nvmem_cell_put(cell);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pmu = devm_kzalloc(&pdev->dev, sizeof(*pmu), GFP_KERNEL);
	if (unlikely(!pmu))
		return -ENOMEM;

	num = ddr_perf_init(pmu, base, &pdev->dev);

	platform_set_drvdata(pdev, pmu);

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, DDR_PERF_DEV_NAME "%d",
			      num);
	if (unlikely(!name))
		return -ENOMEM;

	pmu->cpu = raw_smp_processor_id();
	ret = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN, DDR_CPUHP_CB_NAME,
				      NULL, ddr_perf_offline_cpu);

	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "cpuhp_setup_state_multi failed\n");
		goto cpuhp_state_err;
	}

	pmu->cpuhp_state = ret;

	/* Register the pmu instance for cpu hotplug */
	ret = cpuhp_state_add_instance_nocalls(pmu->cpuhp_state, &pmu->node);
	if (unlikely(ret)) {
		dev_err(&pdev->dev, "Error %d registering hotplug\n", ret);
		goto cpuhp_instance_err;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Get IRQ failed: %d", irq);
		goto ddr_perf_err;
	}

	ret = devm_request_irq(&pdev->dev, irq, ddr_perf_irq_handler,
			       IRQF_NOBALANCING | IRQF_NO_THREAD,
			       DDR_CPUHP_CB_NAME, pmu);
	if (unlikely(ret)) {
		dev_err(&pdev->dev, "Request irq failed: %d", ret);
		goto ddr_perf_err;
	}

	pmu->irq = irq;
	ret = irq_set_affinity_hint(pmu->irq, cpumask_of(pmu->cpu));
	if (unlikely(ret)) {
		dev_err(pmu->dev, "Failed to set interrupt affinity!\n");
		goto ddr_perf_err;
	}

	/* Enable DDR PMU interrupt */
	ret = write_nvmem_cell(&pdev->dev, "ddr_pmu_irq", BIT(0));
	if (ret)
		goto ddr_perf_err;

	ret = perf_pmu_register(&pmu->pmu, name, -1);
	if (unlikely(ret))
		goto ddr_perf_err;

	dev_info(pmu->dev, "device initialized successfully\n");

	return 0;

ddr_perf_err:
	cpuhp_state_remove_instance_nocalls(pmu->cpuhp_state, &pmu->node);
cpuhp_instance_err:
	cpuhp_remove_multi_state(pmu->cpuhp_state);
cpuhp_state_err:
	ida_simple_remove(&ddr_ida, pmu->id);
	dev_warn(&pdev->dev, "device probe failed (%d), disabled\n", ret);
	return ret;
}

static int ddr_perf_remove(struct platform_device *pdev)
{
	struct ddr_pmu *pmu = platform_get_drvdata(pdev);

	cpuhp_state_remove_instance_nocalls(pmu->cpuhp_state, &pmu->node);
	cpuhp_remove_multi_state(pmu->cpuhp_state);
	WARN_ON(irq_set_affinity_hint(pmu->irq, NULL));

	perf_pmu_unregister(&pmu->pmu);

	ida_simple_remove(&ddr_ida, pmu->id);

	dev_info(pmu->dev, "device removed\n");

	return 0;
}

static const struct of_device_id s32cc_ddr_perf_dt_ids[] = {
	{
		.compatible = "nxp,s32cc-ddr-perf",
	}, {}
};
MODULE_DEVICE_TABLE(of, s32cc_ddr_perf_dt_ids);

static struct platform_driver s32cc_ddr_perf_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table	= s32cc_ddr_perf_dt_ids,
	},
	.probe = ddr_perf_probe,
	.remove = ddr_perf_remove,
};

module_platform_driver(s32cc_ddr_perf_driver);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NXP S32CC DDR Performance Monitor Driver");
