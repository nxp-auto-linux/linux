// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * NXP CAR-V UIO Driver
 *
 * This file contains the user-space I/O support for the CAR-V
 * user space driver.
 *
 * Copyright 2025 NXP
 */

#include <linux/atomic.h>
#include <linux/dma-direct.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/uio_driver.h>
#include <linux/vmalloc.h>

#define DRIVER_NAME	"carv-uio"
#define DRIVER_VERSION	"1.0"

#define CARV_MAX_NUM_QUARTILES	2

#define CARV_INTL_MAX_MAPPINGS	64

/* CAR-V Registers */
#define CARV_MB_CR	0x00000000	/* Mailbox Control */

/* Application Fatal Fault Status */
#define CARV_MB_CR_FAULT_MASK	BIT(3)

/*
 * enum carv_uio_map_idx - CAR-V UIO mapping index
 * @CARV_UIO_MAP_RMEM: Reserved memory range
 * @CARV_UIO_MAP_INTL: Driver internal shared memory
 * @CARV_UIO_MAP_REGS_0: Hardware register space for quartile
 * @CARV_UIO_MAP_REGS_1: Hardware register space for quartile
 */
enum carv_uio_map_idx {
	CARV_UIO_MAP_RMEM = 0u,
	CARV_UIO_MAP_INTL = 1u,
	CARV_UIO_MAP_REGS_0 = 2u,
	CARV_UIO_MAP_REGS_1 = 3u,
};

/* Bits in carv_uio_drvdata.irq_flags */
enum {
	CARV_UIO_IRQ_DISABLED = 0,
};

/*
 * struct carv_uio_intl_mapping - CAR-V UIO driver internal virt to bus address mapping
 * @bus_addr: Bus address of the internal mapping
 * @virt: Virtual userspace address of the internal mapping
 * @size: Size of the internal mapping
 */
struct carv_uio_intl_mapping {
	dma_addr_t bus_addr;
	unsigned long virt;
	size_t size;
};

/*
 * struct carv_uio_intl - CAR-V UIO driver internal shared memory layout
 * @maps: Driver internal virt to bus address mappings
 */
struct carv_uio_intl {
	struct carv_uio_intl_mapping maps[CARV_INTL_MAX_MAPPINGS];
};

/*
 * struct carv_uio_drvdata - CAR-V UIO driver private data
 * @dev: CAR-V UIO device
 * @info: UIO device info
 * @pool: Pool for reserved memory
 * @pool_size: Size of the reserved memory pool
 * @intl: Driver internal shared memory
 * @intl_lock: Mutex protecting concurrent access to the internal shared memory.
 *             This ensures atomic updates when multiple kernel threads try to
 *             modify or read from the internal shared memory region.
 * @irq_flags: IRQ status
 * @irq_lock: Spinlock ensuring safe manipulation of IRQ enable/disable operations.
 *            This prevents race conditions when enabling/disabling IRQs, especially
 *            when multiple threads or an interrupt handler may modify the flags.
 * @dma_handle: DMA handle for memory used by CAR-V
 * @dma_cookie: Bogus virtual address needed to free allocated DMA memory
 * @busy: UIO taken by userspace process
 * @fault: True if CAR-V UIO device is faulted
 */
struct carv_uio_drvdata {
	struct device *dev;
	struct uio_info info;
	struct gen_pool *pool;
	unsigned long pool_size;
	struct carv_uio_intl *intl;
	struct mutex intl_lock;
	unsigned long irq_flags;
	raw_spinlock_t irq_lock;
	dma_addr_t dma_handle;
	void *dma_cookie;
	atomic_t busy;
	bool fault;
};

/*
 * struct carv_uio_vm_entry - CAR-V UIO vm_area_struct private data
 * @map: CAR-V UIO driver internal virt to bus address mapping
 * @priv: CAR-V UIO driver private data
 * @bus_addr: Bus address of the allocated memory from pool
 * @size: Size of the allocated memory from pool
 */
struct carv_uio_vm_entry {
	struct carv_uio_intl_mapping *map;
	struct carv_uio_drvdata *priv;
	dma_addr_t bus_addr;
	size_t size;
};

static bool carv_fault_vm_ops_enable;
module_param(carv_fault_vm_ops_enable, bool, 0644);
MODULE_PARM_DESC(carv_fault_vm_ops_enable, "Enable vm_ops fault handler for rmem mapping");

static unsigned long carv_pool_sz = SZ_16M;

/*
 * setup_carv_pool_sz - User can override above with "carv_pool_sz=nnn[KkMm]" in cmdline
 * @str: Memory string to parse
 */
static int __init setup_carv_pool_sz(char *str)
{
	carv_pool_sz = memparse(str, NULL) & PAGE_MASK;

	/* Early console might not be setup yet - it will show up later */
	pr_info("\"carv_pool_sz=%s\": CAR-V pool size set to %ldM\n", str, carv_pool_sz >> 20);

	return 0;
}
early_param("carv_pool_sz", setup_carv_pool_sz);

/*
 * carv_uio_vm_close - Free allocated memory from pool when vm is unmapped
 * @vma: Virtual memory area
 */
static void carv_uio_vm_close(struct vm_area_struct *vma)
{
	struct carv_uio_vm_entry *entry = vma->vm_private_data;
	struct carv_uio_drvdata *priv = entry->priv;

	gen_pool_free(priv->pool, entry->bus_addr, entry->size);

	scoped_guard(mutex, &priv->intl_lock) {
		memset(entry->map, 0, sizeof(*entry->map));
	}

	kfree(entry);
}

/*
 * carv_uio_vm_fault - Handle page fault for UIO DMA memory mapping
 * @vmf: vm_fault structure containing faulting address and VMA info
 *
 * Return: VM_FAULT_SIGBUS if the fault is outside the mapped range,
 * or the result of vmf_insert_page() on success.
 */
static vm_fault_t carv_uio_vm_fault(struct vm_fault *vmf)
{
	struct carv_uio_vm_entry *entry = vmf->vma->vm_private_data;
	struct carv_uio_drvdata *priv = entry->priv;
	unsigned long offset;
	unsigned long pfn;
	struct page *page;

	if (WARN_ON(!carv_fault_vm_ops_enable))
		return VM_FAULT_SIGBUS;

	if (WARN_ON(priv->dev->iommu))
		return VM_FAULT_SIGBUS;

	offset = vmf->address - vmf->vma->vm_start;
	if (offset >= entry->size)
		return VM_FAULT_SIGBUS;

	pfn = PFN_DOWN(dma_to_phys(priv->dev, entry->bus_addr) + offset);
	if (!pfn_valid(pfn))
		return VM_FAULT_SIGBUS;

	page = pfn_to_page(pfn);
	return vmf_insert_page(vmf->vma, vmf->address, page);
}

static const struct vm_operations_struct carv_uio_vm_ops = {
	.close = carv_uio_vm_close,
	.fault = carv_uio_vm_fault,
};

/*
 * carv_uio_open - Open /dev/uioX device if not busy and enable interrupts
 * @info: UIO device info
 * @inode: inode, not used
 *
 * Return: 0 if UIO is not busy, -EINVAL if irq is not valid,
 * -EBUSY if UIO is busy
 */
static int carv_uio_open(struct uio_info *info, struct inode *inode)
{
	struct carv_uio_drvdata *priv = info->priv;

	if (info->irq < 0 || info->irq > UINT_MAX)
		return -EINVAL;

	if (atomic_xchg(&priv->busy, 1))
		return -EBUSY;

	if (test_and_clear_bit(CARV_UIO_IRQ_DISABLED, &priv->irq_flags))
		enable_irq(info->irq);

	return 0;
}

/*
 * carv_uio_release - Release /dev/uioX device and disable interrupts
 * @info: UIO device info
 * @inode: inode, not used
 *
 * Return: -EINVAL if irq is not valid, 0 otherwise
 */
static int carv_uio_release(struct uio_info *info, struct inode *inode)
{
	struct carv_uio_drvdata *priv = info->priv;

	if (info->irq < 0 || info->irq > UINT_MAX)
		return -EINVAL;

	if (!test_and_set_bit(CARV_UIO_IRQ_DISABLED, &priv->irq_flags))
		disable_irq_nosync(info->irq);

	atomic_xchg(&priv->busy, 0);

	return 0;
}

/*
 * carv_uio_mmap - Map memory for userspace based on carv_uio_map_idx mappings
 * @info: UIO device info
 * @vma: Virtual memory area
 *
 * Return: -ENOMEM if no memory available, -EINVAL if vma->vm_pgoff is not an entry
 * of carv_uio_map_idx, otherwise remap_pfn_range status
 */
static int carv_uio_mmap(struct uio_info *info, struct vm_area_struct *vma)
{
	struct carv_uio_drvdata *priv = container_of(info, struct carv_uio_drvdata, info);
	struct carv_uio_vm_entry *entry;
	vm_flags_t vm_flags_mask;
	unsigned long vma_size;
	struct uio_mem *mem;
	unsigned long pfn;
	dma_addr_t addr;
	int ret = 0;

	if (vma->vm_end < vma->vm_start)
		return -EINVAL;

	vma_size = vma->vm_end - vma->vm_start;

	switch (vma->vm_pgoff) {
	case CARV_UIO_MAP_REGS_0:
	case CARV_UIO_MAP_REGS_1:
		pfn = PHYS_PFN(priv->info.mem[vma->vm_pgoff].addr);
		break;
	case CARV_UIO_MAP_RMEM:
		entry = kmalloc(sizeof(*entry), GFP_KERNEL);
		if (!entry)
			return -ENOMEM;

		addr = (dma_addr_t)gen_pool_dma_alloc_align(priv->pool, vma_size, NULL, SZ_16K);
		if (addr == 0) {
			ret = -ENOMEM;
			goto out_kfree_entry;
		}

		scoped_guard(mutex, &priv->intl_lock) {
			for (int i = 0; i < CARV_INTL_MAX_MAPPINGS; i++) {
				if (priv->intl->maps[i].bus_addr == 0) {
					priv->intl->maps[i] = (struct carv_uio_intl_mapping) {
						.bus_addr = addr,
						.virt = vma->vm_start,
						.size = vma_size
					};
					*entry = (struct carv_uio_vm_entry) {
						.map = &priv->intl->maps[i],
						.priv = priv,
						.bus_addr = addr,
						.size = vma_size
					};
					break;
				}
			}
		}

		vma->vm_ops = &carv_uio_vm_ops;
		vma->vm_private_data = entry;
		if (carv_fault_vm_ops_enable) {
			vm_flags_mask = VM_MIXEDMAP | VM_DONTEXPAND | VM_DONTDUMP;
			vm_flags_mod(vma, vm_flags_mask, VM_PFNMAP);
			goto out;
		}
		break;
	case CARV_UIO_MAP_INTL:
		pfn = PHYS_PFN(priv->info.mem[CARV_UIO_MAP_INTL].addr);
		vm_flags_mask = VM_WRITE | VM_MAYWRITE;
		vm_flags_set(vma, vma->vm_flags & (~vm_flags_mask));
		break;
	default:
		return -EINVAL;
	}

	if (vma->vm_pgoff != CARV_UIO_MAP_RMEM) {
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		ret = remap_pfn_range(vma, vma->vm_start, pfn, vma_size, vma->vm_page_prot);
	} else {
		mem = &priv->info.mem[CARV_UIO_MAP_RMEM];
		vma->vm_pgoff = (addr - mem->dma_addr) >> PAGE_SHIFT;
		ret = dma_mmap_attrs(mem->dma_device, vma, (void *)mem->addr, mem->dma_addr,
				     vma_size + (vma->vm_pgoff << PAGE_SHIFT),
				     DMA_ATTR_NO_KERNEL_MAPPING);
	}

	goto out;

out_kfree_entry:
	kfree(entry);
out:
	return ret;
}

/*
 * carv_uio_irq - Handler for CAR-V interrupts
 * @irq: Interrupt line
 * @info: CAR-V UIO info
 *
 * Return: IRQ_NONE if @irq is negative, otherwise IRQ_HANDLED
 */
static irqreturn_t carv_uio_irq(int irq, struct uio_info *info)
{
	struct carv_uio_drvdata *priv = info->priv;

	if (irq < 0)
		return IRQ_NONE;

	raw_spin_lock(&priv->irq_lock);
	if (!test_and_set_bit(CARV_UIO_IRQ_DISABLED, &priv->irq_flags))
		disable_irq_nosync(irq);
	raw_spin_unlock(&priv->irq_lock);

	return IRQ_HANDLED;
}

/*
 * carv_uio_irqcontrol - Control CAR-V interrupt status by writing to /dev/uioX
 * @info: UIO device info
 * @irq_on: Interrupt control command
 *
 * Return: -EINVAL if irq is not valid, 0 otherwise
 */
static int carv_uio_irqcontrol(struct uio_info *info, s32 irq_on)
{
	struct carv_uio_drvdata *priv = info->priv;
	unsigned long flags;

	if (info->irq < 0 || info->irq > UINT_MAX)
		return -EINVAL;

	raw_spin_lock_irqsave(&priv->irq_lock, flags);
	if (irq_on) {
		if (test_and_clear_bit(CARV_UIO_IRQ_DISABLED, &priv->irq_flags))
			enable_irq(info->irq);
	} else {
		if (!test_and_set_bit(CARV_UIO_IRQ_DISABLED, &priv->irq_flags))
			disable_irq_nosync(info->irq);
	}
	raw_spin_unlock_irqrestore(&priv->irq_lock, flags);

	return 0;
}

/*
 * carv_uio_expose_regs - Expose CAR-V register space
 * @dev: CAR-V UIO device
 */
static int carv_uio_expose_regs(struct device *dev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct carv_uio_drvdata *priv = dev_get_drvdata(dev);
	struct uio_info *info = &priv->info;
	struct resource *res;
	struct uio_mem *mem;
	int regs_idx;

	for (int i = 0; i < CARV_MAX_NUM_QUARTILES; i++) {
		/* Get resource information from quartile device tree node */
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			break;

		regs_idx = CARV_UIO_MAP_REGS_0 + i;
		if (regs_idx >= MAX_UIO_MAPS) {
			dev_err(dev, "uio_mem index: %d, valid: [0,%u)\n",
				regs_idx, MAX_UIO_MAPS);
			return -EINVAL;
		}

		/* Expose CAR-V register space */
		info->mem[regs_idx].name = "carv-registers";
		info->mem[regs_idx].addr = (phys_addr_t)res->start;
		info->mem[regs_idx].size = resource_size(res);
		info->mem[regs_idx].memtype = UIO_MEM_PHYS;
	}

	mem = &info->mem[CARV_UIO_MAP_REGS_0];
	mem->internal_addr = devm_ioremap(dev, mem->addr, mem->size);
	if (!mem->internal_addr)
		return -ENOMEM;

	return 0;
}

/*
 * carv_uio_expose_rmem - Expose CAR-V reserved memory
 * @dev: CAR-V UIO device
 */
static void carv_uio_expose_rmem(struct device *dev)
{
	struct carv_uio_drvdata *priv = dev_get_drvdata(dev);
	struct uio_info *info;

	info = &priv->info;

	/* Expose CAR-V reserved memory */
	info->mem[CARV_UIO_MAP_RMEM].name = "carv-reserved-memory";
	info->mem[CARV_UIO_MAP_RMEM].addr = (uintptr_t)priv->dma_cookie;
	info->mem[CARV_UIO_MAP_RMEM].dma_addr = priv->dma_handle;
	info->mem[CARV_UIO_MAP_RMEM].dma_device = priv->dev;
	info->mem[CARV_UIO_MAP_RMEM].size = priv->pool_size;
	info->mem[CARV_UIO_MAP_RMEM].memtype = UIO_MEM_DMA_COHERENT;
}

/*
 * carv_uio_expose_priv - Expose CAR-V driver internal shared memory
 * @dev: CAR-V UIO device
 */
static void carv_uio_expose_priv(struct device *dev)
{
	struct carv_uio_drvdata *priv = dev_get_drvdata(dev);
	struct uio_info *info;

	info = &priv->info;

	/* Expose CAR-V driver internal shared memory */
	info->mem[CARV_UIO_MAP_INTL].name = "carv-driver-intl";
	info->mem[CARV_UIO_MAP_INTL].addr = page_to_phys(vmalloc_to_page(priv->intl));
	info->mem[CARV_UIO_MAP_INTL].size = PAGE_SIZE;
	info->mem[CARV_UIO_MAP_INTL].memtype = UIO_MEM_PHYS;
}

/*
 * carv_uio_alloc_dma_memory - Alloc DMA memory used by CAR-V
 * @dev: CAR-V UIO device
 *
 * Return: -ENOMEM if no memory available, otherwise 0
 */
static int carv_uio_alloc_dma_memory(struct device *dev)
{
	struct carv_uio_drvdata *priv = dev_get_drvdata(dev);
	dma_addr_t dma_handle;
	void *dma_cookie;

	/* Alloc DMA memory */
	dma_handle = 0;
	dma_cookie = dma_alloc_attrs(dev, carv_pool_sz, &dma_handle, GFP_KERNEL,
				     DMA_ATTR_NO_KERNEL_MAPPING);
	if ((!dma_cookie) || (dma_handle == 0)) {
		dev_err(dev, "failed to alloc DMA memory\n");
		return -ENOMEM;
	}
	priv->dma_cookie = dma_cookie;
	priv->dma_handle = dma_handle;
	priv->pool_size = carv_pool_sz;

	return 0;
}

/*
 * fault_show - Show in sysfs if the CAR-V UIO device is faulted
 * @dev: The device associated with the sysfs attribute
 * @attr: The sysfs attribute permissions
 * @buf: The buffer where the fault status will be written to
 *
 * Return: The number of bytes written to the buffer, or a negative error code on failure
 */
static ssize_t fault_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct uio_device *uio_dev = dev_get_drvdata(dev);
	struct uio_info *info = uio_dev->info;
	struct carv_uio_drvdata *priv;
	void __iomem *mb_ir;

	priv = info->priv;

	if (!priv->fault) {
		mb_ir = info->mem[CARV_UIO_MAP_REGS_0].internal_addr + CARV_MB_CR;
		if (readl(mb_ir) & CARV_MB_CR_FAULT_MASK)
			priv->fault = true;
	}

	return sysfs_emit(buf, "%u\n", priv->fault);
}

static DEVICE_ATTR_RO(fault);

static int carv_uio_probe(struct platform_device *pdev)
{
	struct carv_uio_drvdata *priv;
	struct page *intl_page;
	struct uio_info *info;
	struct device *dev;
	int ret;

	dev = &pdev->dev;

	/* Alloc carv_uio_drvdata private data */
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);
	info = &priv->info;

	/* Get IRQ for user from device tree */
	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		dev_err(dev, "failed to get irq: %ld\n", info->irq);
		return info->irq;
	}

	/* Initialize mutex for driver internal shared memory */
	mutex_init(&priv->intl_lock);

	/* Initialize spin_lock for IRQ flags */
	raw_spin_lock_init(&priv->irq_lock);

	/* Set atomic variable busy to 0 (not busy) */
	atomic_set(&priv->busy, 0);

	/* Alloc one page for internal shared memory */
	intl_page = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!intl_page)
		return -ENOMEM;

	/* Vmap internal shared memory as non-cached */
	priv->intl = vmap(&intl_page, 1, VM_MAP, pgprot_noncached(PAGE_KERNEL));
	if (!priv->intl) {
		ret = -ENOMEM;
		goto out_free_intl_memory;
	}

	/* Alloc DMA memory */
	ret = carv_uio_alloc_dma_memory(dev);
	if (ret)
		goto out_unmap_intl_memory;

	/* Create gen_pool for user */
	priv->pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!priv->pool) {
		ret = -ENOMEM;
		goto out_free_dma_memory;
	}

	/* Add usable memory in the gen_pool */
	ret = gen_pool_add(priv->pool, priv->dma_handle, priv->pool_size, -1);
	if (ret)
		goto out_destroy_gen_pool;

	/* Expose CAR-V register space */
	ret = carv_uio_expose_regs(dev);
	if (ret)
		goto out_destroy_gen_pool;

	/* Expose CAR-V reserved memory */
	carv_uio_expose_rmem(dev);

	/* Expose CAR-V driver internal shared memory */
	carv_uio_expose_priv(dev);

	/* Set UIO driver version */
	info->version = DRIVER_VERSION;

	/* Set UIO driver name */
	info->name = DRIVER_NAME;

	/* Set UIO private data */
	info->priv = priv;

	/* Set UIO file operations */
	info->open = carv_uio_open;
	info->release = carv_uio_release;
	info->mmap = carv_uio_mmap;

	/* Set UIO interrupt handler */
	info->handler = carv_uio_irq;

	/* Set UIO IRQ control callback */
	info->irqcontrol = carv_uio_irqcontrol;

	/* Register UIO device */
	ret = uio_register_device(dev, info);
	if (ret) {
		dev_err(dev, "failed to register UIO device: %d\n", ret);
		goto out_destroy_gen_pool;
	}

	ret = device_create_file(&priv->info.uio_dev->dev, &dev_attr_fault);
	if (ret) {
		dev_err(dev, "failed to create sysfs fault attribute\n");
		goto out_uio_unregister_device;
	}

	dev_dbg(dev, "Successfully registered CAR-V device: /dev/%s\n",
		dev_name(&info->uio_dev->dev));

	return 0;

out_uio_unregister_device:
	uio_unregister_device(&priv->info);
out_destroy_gen_pool:
	gen_pool_destroy(priv->pool);
out_free_dma_memory:
	dma_free_attrs(priv->dev, priv->pool_size, priv->dma_cookie, priv->dma_handle,
		       DMA_ATTR_NO_KERNEL_MAPPING);
out_unmap_intl_memory:
	vunmap(priv->intl);
out_free_intl_memory:
	__free_page(intl_page);
	return ret;
}

static void carv_uio_remove(struct platform_device *pdev)
{
	struct carv_uio_drvdata *priv = platform_get_drvdata(pdev);

	/* Destroy gen pool */
	gen_pool_destroy(priv->pool);

	/* Free DMA memory */
	dma_free_attrs(priv->dev, priv->pool_size, priv->dma_cookie, priv->dma_handle,
		       DMA_ATTR_NO_KERNEL_MAPPING);

	/* Vunmap internal shared memory */
	vunmap(priv->intl);

	/* Free internal shared memory */
	__free_page(vmalloc_to_page(priv->intl));

	/* Remove sysfs fault attribute file */
	device_remove_file(&priv->info.uio_dev->dev, &dev_attr_fault);

	/* Unregister device */
	uio_unregister_device(&priv->info);
}

static const struct of_device_id carv_uio_of_match[] = {
	{ .compatible = "nxp,carv-uio", },
	{}
};
MODULE_DEVICE_TABLE(of, carv_uio_of_match);

static struct platform_driver carv_uio_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table	= carv_uio_of_match,
	},
	.probe = carv_uio_probe,
	.remove_new = carv_uio_remove,
};

module_platform_driver(carv_uio_driver);

MODULE_AUTHOR("Andrei-Edward Popa <andrei.popa_1@nxp.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("NXP CAR-V UIO Driver");
