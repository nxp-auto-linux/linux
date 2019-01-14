// SPDX-License-Identifier: GPL-2.0-only
/*
 * Spin Table SMP initialisation
 *
 * Copyright (C) 2013 ARM Ltd.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <linux/types.h>
#include <linux/mm.h>

#include <asm/cacheflush.h>
#include <asm/cpu_ops.h>
#include <asm/cputype.h>
#include <asm/io.h>
#include <asm/smp_plat.h>

#ifdef CONFIG_S32GEN1_A53GPR_WORKAROUND
#include <linux/of_device.h>
#include <linux/of_address.h>

static phys_addr_t gpr_phys_base_addr = 0;
static u64 gpr_size = 0;

#endif

extern void secondary_holding_pen(void);
volatile unsigned long __section(".mmuoff.data.read")
secondary_holding_pen_release = INVALID_HWID;

static phys_addr_t cpu_release_addr[NR_CPUS];

/*
 * Write secondary_holding_pen_release in a way that is guaranteed to be
 * visible to all observers, irrespective of whether they're taking part
 * in coherency or not.  This is necessary for the hotplug code to work
 * reliably.
 */
static void write_pen_release(u64 val)
{
	void *start = (void *)&secondary_holding_pen_release;
	unsigned long size = sizeof(secondary_holding_pen_release);

	secondary_holding_pen_release = val;
	__flush_dcache_area(start, size);
}


static int smp_spin_table_cpu_init(unsigned int cpu)
{
	struct device_node *dn;
	int ret;

	dn = of_get_cpu_node(cpu, NULL);
	if (!dn)
		return -ENODEV;

	/*
	 * Determine the address from which the CPU is polling.
	 */
	ret = of_property_read_u64(dn, "cpu-release-addr",
				   &cpu_release_addr[cpu]);
	if (ret)
		pr_err("CPU %d: missing or invalid cpu-release-addr property\n",
		       cpu);

	of_node_put(dn);

	return ret;
}

#ifdef CONFIG_S32GEN1_A53GPR_WORKAROUND

#define S32GEN1_GPR01_OFFSET 0x4
#define S32GEN1_GPR01_WFE_EVT_CA53_CLUSTER1 0x2

static void smp_spin_table_sev_secondary_cluster(unsigned int cpu)
{
	volatile void __iomem *gpr_base_addr = NULL;
	u32 a53_gpr01_val = 0;

	if (!gpr_phys_base_addr) {
		struct device_node *node = NULL;
		const __be32 *reg_addr;

		node = of_find_compatible_node(NULL, NULL,
					"fsl,s32gen1-a53gpr");
		if (!node)
			goto error;

		reg_addr = of_get_address(node, 0, &gpr_size, NULL);
		gpr_phys_base_addr = of_translate_address(node, reg_addr);

		of_node_put(node);

		if (!gpr_phys_base_addr || !gpr_size)
			goto error;

		pr_info("Found A53 GPR at address 0x%llx\n", gpr_phys_base_addr);
	}

	gpr_base_addr = ioremap_nocache(gpr_phys_base_addr, gpr_size);
	if (gpr_base_addr && (cpu > 1)) {
		gpr_base_addr += S32GEN1_GPR01_OFFSET;
		u32 a53_gpr01_val_orig = ioread32(gpr_base_addr);
		a53_gpr01_val = a53_gpr01_val_orig | S32GEN1_GPR01_WFE_EVT_CA53_CLUSTER1;
		pr_info("CPU%u: Signal secondary cluster\n", cpu);
		iowrite32(a53_gpr01_val, gpr_base_addr);
		wmb();
		iowrite32(a53_gpr01_val_orig, gpr_base_addr);
		iounmap(gpr_base_addr - S32GEN1_GPR01_OFFSET);
	}

	return;

error:
	pr_err("A53 GPR Model Workaround not applied\n");
}

#define SEV_CPU(cpu) \
	smp_spin_table_sev_secondary_cluster(cpu); \
	sev()

#else
#define SEV_CPU(cpu) sev()
#endif

static int smp_spin_table_cpu_prepare(unsigned int cpu)
{
	__le64 __iomem *release_addr;

	if (!cpu_release_addr[cpu])
		return -ENODEV;

	/*
	 * The cpu-release-addr may or may not be inside the linear mapping.
	 * As ioremap_cache will either give us a new mapping or reuse the
	 * existing linear mapping, we can use it to cover both cases. In
	 * either case the memory will be MT_NORMAL.
	 */
	release_addr = ioremap_cache(cpu_release_addr[cpu],
				     sizeof(*release_addr));
	if (!release_addr)
		return -ENOMEM;

	/*
	 * We write the release address as LE regardless of the native
	 * endianness of the kernel. Therefore, any boot-loaders that
	 * read this address need to convert this address to the
	 * boot-loader's endianness before jumping. This is mandated by
	 * the boot protocol.
	 */
	writeq_relaxed(__pa_symbol(secondary_holding_pen), release_addr);
	__flush_dcache_area((__force void *)release_addr,
			    sizeof(*release_addr));

	/*
	 * Send an event to wake up the secondary CPU.
	 */
	SEV_CPU(cpu);

	iounmap(release_addr);

	return 0;
}

static int smp_spin_table_cpu_boot(unsigned int cpu)
{
	/*
	 * Update the pen release flag.
	 */
	write_pen_release(cpu_logical_map(cpu));

	/*
	 * Send an event, causing the secondaries to read pen_release.
	 */
	SEV_CPU(cpu);

	return 0;
}

const struct cpu_operations smp_spin_table_ops = {
	.name		= "spin-table",
	.cpu_init	= smp_spin_table_cpu_init,
	.cpu_prepare	= smp_spin_table_cpu_prepare,
	.cpu_boot	= smp_spin_table_cpu_boot,
};
