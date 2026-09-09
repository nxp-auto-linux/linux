// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2026 NXP
 *
 * S32N graceful platform shutdown driver.
 *
 * On S32N79 the hardware powers down the Cortex-A subsystem only after every
 * secondary core has retired through the PSCI CPU_OFF power down sequence.
 *
 * The vanilla poweroff/restart path does not take the secondary cores off (it
 * never issues a PSCI CPU_OFF for them), so the hardware cannot complete the
 * power down/restart.
 *
 * This driver registers power-off and restart PREPARE sys-off handlers that
 * offline every secondary core via remove_cpu() before the last core still
 * online issues PSCI SYSTEM_OFF / SYSTEM_RESET. The prepare hook runs
 * before the reboot core disables CPU hotplug, so the offline operations are
 * allowed and may sleep.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/reboot.h>
#include <linux/smp.h>

/**
 * s32n_offline_secondaries() - offline every secondary core before poweroff
 * @data: sys-off callback data (unused)
 *
 * The running core cannot offline itself, so it will just issue a
 * PSCI_SYSTEM_OFF or PSCI_SYSTEM_RESET instead. A prepare handler cannot veto
 * the shutdown, so residual online cores are only reported.
 *
 * Return: always NOTIFY_DONE.
 */
static int s32n_offline_secondaries(struct sys_off_data *data)
{
	unsigned int cpu, self;
	int ret;

	/* Pin self so the surviving core is well defined. */
	migrate_disable();
	self = smp_processor_id();

	for_each_online_cpu(cpu) {
		if (cpu == self)
			continue;

		ret = remove_cpu(cpu);
		if (ret)
			pr_warn("failed to offline CPU%u (%d)\n", cpu, ret);
	}

	migrate_enable();

	if (num_online_cpus() != 1)
		pr_err("CPU%u: %u cores still online; cannot power down the platform\n",
		       self, num_online_cpus());
	else
		pr_info("CPU%u: all secondary cores are offline\n", self);

	return NOTIFY_DONE;
}

static struct sys_off_handler *s32n_poweroff_handler;
static struct sys_off_handler *s32n_restart_handler;

static int __init s32n_poweroff_init(void)
{
	struct sys_off_handler *h;

	if (!of_machine_is_compatible("nxp,s32n79"))
		return 0;

	h = register_sys_off_handler(SYS_OFF_MODE_POWER_OFF_PREPARE,
				     SYS_OFF_PRIO_DEFAULT,
				     s32n_offline_secondaries, NULL);
	if (IS_ERR(h)) {
		pr_err("cannot register power-off handler (%ld)\n", PTR_ERR(h));
		return PTR_ERR(h);
	}
	s32n_poweroff_handler = h;

	h = register_sys_off_handler(SYS_OFF_MODE_RESTART_PREPARE,
				     SYS_OFF_PRIO_DEFAULT,
				     s32n_offline_secondaries, NULL);
	if (IS_ERR(h)) {
		pr_err("cannot register restart handler (%ld)\n", PTR_ERR(h));
		unregister_sys_off_handler(s32n_poweroff_handler);
		return PTR_ERR(h);
	}
	s32n_restart_handler = h;

	return 0;
}

static void __exit s32n_poweroff_exit(void)
{
	unregister_sys_off_handler(s32n_restart_handler);
	unregister_sys_off_handler(s32n_poweroff_handler);
}

module_init(s32n_poweroff_init);
module_exit(s32n_poweroff_exit);

MODULE_AUTHOR("NXP");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("S32N graceful shutdown: offline secondary cores before poweroff/reset");
