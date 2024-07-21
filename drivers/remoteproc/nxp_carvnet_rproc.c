// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2024 NXP
 *
 * CAR-V NET remoteproc driver
 *
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>

#include "remoteproc_internal.h"

static bool auto_boot;
module_param(auto_boot, bool, 0400);
MODULE_PARM_DESC(auto_boot,
		 "Auto-boot the CAR-V-NET remote processor [default=false]");

#define CARVNET_FW_ALIGN	0x4000

enum {
	MEMRES_VDEV_ID,
	MEMRES_FW_ID,
	MEMRES_MAX
};

struct memres {
	void *va;		/* virtual address */
	dma_addr_t dma;		/* dma address */
	size_t len;		/* length in bytes */
	u32 free_idx;		/* pointer to free area inside the region */
	u32 of_resm_idx;	/* reserved memory phandle index */
	char name[32];		/* associated memory region name (optional) */
};

struct carvnet {
	/*
	 * 0 (MEMRES_VDEV_ID): continuous memory for vdev
	 * 1 (MEMRES_FW_ID)  : continuous memory for firmware
	 */
	struct memres mrs[MEMRES_MAX];
	struct dentry *subdentry;
};

static int carvnet_rproc_parse_fw(struct rproc *rproc,
				  const struct firmware *fw)
{
	int ret;

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret)
		dev_warn(&rproc->dev, "No resource table in fw\n");

	return ret;
}

static int carvnet_rproc_start(struct rproc *rproc)
{
	return 0;
}

static int carvnet_rproc_stop(struct rproc *rproc)
{
	return 0;
}

static void carvnet_rproc_kick(struct rproc *rproc, int vqid)
{
	/* no op */
}

/* Expose carveout mem content via debugfs */
static int rproc_carveout_hexdump_show(struct seq_file *seq, void *unused)
{
	struct rproc_mem_entry *mem = seq->private;
	u32 offs;

	for (offs = 0; offs < mem->len; offs += CARVNET_FW_ALIGN)
		seq_hex_dump(seq, "", DUMP_PREFIX_OFFSET, 8, 1,
			     mem->va + offs, CARVNET_FW_ALIGN, true);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rproc_carveout_hexdump);

static int carvnet_rproc_elf_load(struct rproc *rproc,
				  const struct firmware *fw)
{
	struct carvnet *priv = rproc->priv;
	struct device *dev = &rproc->dev;
	struct memres *memres_fw = &priv->mrs[MEMRES_FW_ID];
	const u8 *elf_data = fw->data;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	struct rproc_mem_entry *mem;
	int i, ret = 0;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	dev_dbg(dev, " fw phnum=%d\n", ehdr->e_phnum);

	/* Remove possible existing debugfs carveouts subdir */
	debugfs_remove(priv->subdentry);
	priv->subdentry = NULL;

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 segsize = ALIGN(memsz, CARVNET_FW_ALIGN);
		u32 offset = phdr->p_offset;
		u32 flags = phdr->p_flags;

		dev_dbg(dev, "phdr[%i]: type %d da 0x%x offset 0x%x flags 0x%x memsz 0x%x filesz 0x%x\n",
			i, phdr->p_type, da, offset, flags, memsz, filesz);

		if (phdr->p_type != PT_LOAD)
			continue;

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* Create memory carveout for every fw segment.
		 * NOTE: We don't need to alloc/free such carveouts,
		 * they are part of reserved region 'fw'.
		 */
		mem = rproc_mem_entry_init(dev,
					   memres_fw->va + memres_fw->free_idx,
					   memres_fw->dma + memres_fw->free_idx,
					   segsize, da, NULL, NULL, "fwseg%d",
					   i);
		if (!mem)
			return -ENOMEM;

		if (rproc_coredump_add_segment(rproc, da, segsize))
			dev_warn(dev,
				 "Failed to add memory region '%s' to the coredump\n",
				 mem->name);

		rproc_add_carveout(rproc, mem);

		/* Move free pointer to the next aligned boundary */
		memres_fw->free_idx += segsize;

		/* Save segment in the reserved carveout memory region */
		memcpy(mem->va, elf_data + offset, filesz);

		/* Zero unused memory */
		if ((segsize - filesz) > 0)
			memset(mem->va + filesz, 0, segsize - filesz);
	}

	/* Add debugfs dump regions */
	priv->subdentry = debugfs_create_dir("carveouts", rproc->dbg_dir);
	list_for_each_entry(mem, &rproc->carveouts, node)
		debugfs_create_file(mem->name, 0400, priv->subdentry, mem,
				    &rproc_carveout_hexdump_fops);

	return ret;
}

static const struct rproc_ops carvnet_rproc_ops = {
	.start		= carvnet_rproc_start,
	.stop		= carvnet_rproc_stop,
	.kick		= carvnet_rproc_kick,
	.parse_fw	= carvnet_rproc_parse_fw,
	.load		= carvnet_rproc_elf_load,
	.find_loaded_rsc_table	= rproc_elf_find_loaded_rsc_table,
};

static int carvnet_rproc_parse_resmem(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct carvnet *priv = rproc->priv;
	struct of_phandle_iterator it;
	struct reserved_mem *rmem;
	int idx = 0, ret;

	/* Parse associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {

		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			of_node_put(it.node);
			dev_err(dev,
				"unable to acquire memory region\n");
			return -ENOMEM;
		}

		/* Verify the correct memres name */
		if (idx == MEMRES_VDEV_ID) {
			/* Required resmem 0: vdev */
			if (strcmp(it.node->name, "vdev")) {
				dev_err(dev,
					"Invalid memory region name for vdev: '%s'\n",
					it.node->name);
				return -EINVAL;
			}

			/* Associate vdev reserved memory region
			 * with device
			 */
			ret = of_reserved_mem_device_init_by_idx(dev, np, idx);
			if (ret) {
				dev_err(dev, "Failed to associate vdev resmem: %d\n",
					ret);
				return ret;
			}
		} else if (idx == MEMRES_FW_ID) {
			/* Required resmem 1: firmware */
			if (strcmp(it.node->name, "fw")) {
				dev_err(dev,
					"Invalid memory region name for firmware: '%s'\n",
					it.node->name);
				return -EINVAL;
			}
		} else {
			/* MEMRES_CUSTOM is optional, skipped for now */
			continue;
		}

		priv->mrs[idx].dma = (dma_addr_t)rmem->base;
		priv->mrs[idx].len = rmem->size;
		priv->mrs[idx].of_resm_idx = idx;
		strscpy(priv->mrs[idx].name, it.node->name,
			strlen(it.node->name));
		priv->mrs[idx].va = devm_ioremap_wc(dev, rmem->base,
						    rmem->size);
		if (!priv->mrs[idx].va) {
			dev_err(dev,
				"Unable to map memory region: '%s' %pa+0x%llx\n",
				priv->mrs[idx].name, &rmem->base, rmem->size);
			return -ENOMEM;
		}

		dev_dbg(dev, "Parsed resmem #%d for memory region '%s'\n", idx,
			it.node->name);

		idx++;
	}

	if (idx < 2)
		dev_warn(dev, "Not all required regions were declared: %d\n",
			 idx);

	return  0;
}

static int carvnet_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	const char *fw_name = NULL;
	int ret;

	ret = rproc_of_parse_firmware(dev, 0, &fw_name);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to parse firmware-name property\n");
	if (!fw_name)
		return dev_err_probe(dev, -EINVAL, "FW name is missing\n");

	rproc = devm_rproc_alloc(dev, np->name, &carvnet_rproc_ops, fw_name,
				 sizeof(struct carvnet));
	if (!rproc)
		return -ENOMEM;

	ret = carvnet_rproc_parse_resmem(rproc);
	if (ret)
		return ret;

	dev_set_drvdata(dev, rproc);

	rproc->auto_boot = auto_boot;

	ret = devm_rproc_add(dev, rproc);

	return ret;
}

static void carvnet_rproc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc *rproc = dev_get_drvdata(dev);
	struct carvnet *priv = rproc->priv;

	/* Remove possible existing debugfs carveouts subdir */
	debugfs_remove(priv->subdentry);
	priv->subdentry = NULL;
}

static const struct of_device_id carvnet_rproc_of_match[] = {
	{ .compatible = "nxp,carvnet-rproc" },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, carvnet_rproc_of_match);

static struct platform_driver carvnet_rproc_driver = {
	.probe = carvnet_rproc_probe,
	.remove_new = carvnet_rproc_remove,
	.driver = {
		.name = "nxp-carvnet",
		.of_match_table = carvnet_rproc_of_match,
	},
};

module_platform_driver(carvnet_rproc_driver);

MODULE_AUTHOR("Jan Petrous <jan.petrous@nxp.com>");
MODULE_DESCRIPTION("NXP S32N7 CAR-V-NET rproc driver");
MODULE_LICENSE("Dual BSD/GPL");
