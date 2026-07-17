// SPDX-License-Identifier: GPL-2.0-only

#include <linux/mm.h>
#include <linux/io.h>
#include <asm/pgtable-prot.h>

static ioremap_prot_hook_t ioremap_prot_hook;

/*
 * When set (from a "nonposted-mmio" property on the DT root node, see
 * setup_machine_fdt()), all Device-nGnRE ioremap() mappings are promoted to
 * Device-nGnRnE, for platforms that require non-posted device MMIO.
 */
bool arm64_ioremap_force_nonposted __ro_after_init;

int arm64_ioremap_prot_hook_register(ioremap_prot_hook_t hook)
{
	if (WARN_ON(ioremap_prot_hook))
		return -EBUSY;

	ioremap_prot_hook = hook;
	return 0;
}

void __iomem *ioremap_prot(phys_addr_t phys_addr, size_t size,
			   unsigned long prot)
{
	unsigned long last_addr = phys_addr + size - 1;
	pgprot_t pgprot = __pgprot(prot);

	/* Don't allow outside PHYS_MASK */
	if (last_addr & ~PHYS_MASK)
		return NULL;

	/* Don't allow RAM to be mapped. */
	if (WARN_ON(pfn_is_map_memory(__phys_to_pfn(phys_addr))))
		return NULL;

	/*
	 * Promote posted device MMIO to non-posted (nGnRE -> nGnRnE) when the
	 * platform requires it. The flag is only ever set on platforms that opt
	 * in via a "nonposted-mmio" property on the DT root node.
	 */
	if (unlikely(arm64_ioremap_force_nonposted) &&
	    prot == PROT_DEVICE_nGnRE)
		pgprot = __pgprot(PROT_DEVICE_nGnRnE);

	/*
	 * If a hook is registered (e.g. for confidential computing
	 * purposes), call that now and barf if it fails.
	 */
	if (unlikely(ioremap_prot_hook) &&
	    WARN_ON(ioremap_prot_hook(phys_addr, size, &pgprot))) {
		return NULL;
	}

	return generic_ioremap_prot(phys_addr, size, pgprot);
}
EXPORT_SYMBOL(ioremap_prot);

/*
 * Must be called after early_fixmap_init
 */
void __init early_ioremap_init(void)
{
	early_ioremap_setup();
}

bool arch_memremap_can_ram_remap(resource_size_t offset, size_t size,
				 unsigned long flags)
{
	unsigned long pfn = PHYS_PFN(offset);

	return pfn_is_map_memory(pfn);
}
