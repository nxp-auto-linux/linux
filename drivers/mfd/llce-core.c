// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2020-2021, 2023 NXP */
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/firmware.h>
#include <linux/genalloc.h>
#include <linux/kernel.h>
#include <linux/mailbox/nxp-llce/llce_fw_interface.h>
#include <linux/mailbox/nxp-llce/llce_core.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/processor.h>
#include <linux/slab.h>

#define LLCE_CORE_DIR		"llce_core"
#define LLCE_CORE1_TS		"core1_ts"
#define LLCE_CORE2_TS		"core2_ts"
#define LLCE_CORE3_TS		"core3_ts"

#define LLCE_SHMEM_REGION	"shmem"
#define LLCE_SYSRSTR		0x0
#define LLCE_SYSRSTR_RST0	BIT(0)
#define LLCE_SYSRSTR_RST1	BIT(1)
#define LLCE_SYSRSTR_RST2	BIT(2)
#define LLCE_SYSRSTR_RST3	BIT(3)

/* Poll for maximum 500 ms */
#define LLCE_BOOT_POLL_NS (500 * NSEC_PER_MSEC)

/* LLCE cores startup ending information values */
#define LLCE_MGR_DTE_BOOT_END			(0x0000000FU)
#define LLCE_MGR_RX_BOOT_END			(0x000000F0U)
#define LLCE_MGR_TX_BOOT_END			(0x00000F00U)
#define LLCE_MGR_FRPE_BOOT_END			(0x0000F000U)
#define LLCE_MGR_BOOT_END_ALL_CORES_MASK	(0x0000FFFFU)
#define CORES_TS_OFFSET				(0x4FFD0U)

struct sram_node {
	const char *name;
	void __iomem *addr;
	size_t size;
};

struct llce_fw {
	const struct firmware *fw_entry;
	struct sram_node *node;
};

struct llce_core {
	struct clk *clk;
	void __iomem *sysctrl_base;
	struct llce_fw *fws;
	struct device *dev;
	struct dentry *debugfs_root;

	struct sram_node *sram_nodes;
	size_t n_sram;

	size_t nfrws;
};

static bool load_fw = true;
module_param(load_fw, bool, 0660);

static struct device_node *get_sram_node(struct device *dev, const char *name)
{
	struct device_node *node, *dev_node;
	int idx;

	dev_node = dev->of_node;
	idx = of_property_match_string(dev_node, "memory-region-names", name);
	node = of_parse_phandle(dev_node, "memory-region", idx);
	if (!node) {
		dev_err(dev, "Failed to get '%s' memory region\n", name);
		return ERR_PTR(-EIO);
	}

	return node;
}

static int map_sram_nodes(struct device *dev, struct llce_core *core)
{
	int ret, n_reg, i;
	const char *img_name;
	struct device_node *node;
	struct resource r;
	resource_size_t size;

	n_reg = of_property_count_strings(dev->of_node, "memory-region-names");
	if (n_reg < 0)
		return n_reg;

	core->sram_nodes = devm_kmalloc(dev, sizeof(*core->sram_nodes) * n_reg,
					GFP_KERNEL);
	if (!core->sram_nodes)
		return -ENOMEM;

	core->n_sram = n_reg;

	for (i = 0; i < n_reg; i++) {
		ret = of_property_read_string_index(dev->of_node,
						    "memory-region-names",
						    i, &img_name);
		if (ret) {
			dev_err(dev, "Failed to get 'memory-region-names' %d property\n",
				i);
			return ret;
		}

		core->sram_nodes[i].name = img_name;
		node = get_sram_node(dev, img_name);
		if (IS_ERR(node))
			return PTR_ERR(node);

		ret = of_address_to_resource(node, 0, &r);
		of_node_put(node);
		if (ret)
			return ret;

		size = resource_size(&r);
		core->sram_nodes[i].size = size;

		core->sram_nodes[i].addr = devm_ioremap_wc(dev, r.start, size);
		if (!core->sram_nodes[i].addr) {
			dev_err(dev, "Failed to map '%s' memory region\n",
				img_name);
			return -ENOMEM;
		}
	}

	return 0;
}

static void init_sram_nodes(struct llce_core *core)
{
	size_t i;

	for (i = 0; i < core->n_sram; i++)
		memset_io(core->sram_nodes[i].addr, 0,
			  core->sram_nodes[i].size);
}

static struct sram_node *get_core_sram(struct llce_core *core, const char *name)
{
	int i;

	for (i = 0; i < core->n_sram; i++) {
		if (!strcmp(core->sram_nodes[i].name, name))
			return &core->sram_nodes[i];
	}

	return NULL;
}

static void __iomem *get_status_regs(struct llce_core *core)
{
	struct device *dev = core->dev;
	struct sram_node *shmem = get_core_sram(core, LLCE_SHMEM_REGION);

	if (!shmem) {
		dev_err(dev, "Memory region %s not found\n",
			LLCE_SHMEM_REGION);
		return NULL;
	}

	return llce_get_status_regs_addr(shmem->addr);
}

static void __iomem *get_core_ts_regs(struct llce_core *core)
{
	struct device *dev = core->dev;
	struct sram_node *shmem = get_core_sram(core, LLCE_SHMEM_REGION);

	if (!shmem) {
		dev_err(dev, "Memory region %s not found\n",
			LLCE_SHMEM_REGION);
		return NULL;
	}

	return shmem->addr + CORES_TS_OFFSET;
}

static int llce_load_fw_images(struct device *dev, struct llce_core *core)
{
	int i, ret;
	struct llce_fw *fw;
	const char *img_name;

	ret = of_property_count_strings(dev->of_node, "firmware-name");
	if (ret < 0) {
		dev_err(dev, "Failed to get 'firmware-name' property\n");
		return core->nfrws;
	}
	core->nfrws = ret;

	core->fws = devm_kmalloc(dev, core->nfrws * sizeof(*core->fws),
				 GFP_KERNEL);
	if (!core->fws)
		return -ENOMEM;

	for (i = 0; i < core->nfrws; i++) {
		ret = of_property_read_string_index(dev->of_node,
						    "firmware-name", i,
						    &img_name);
		if (ret) {
			dev_err(dev, "Failed to get firmware's name (idx=%d)\n",
				i);
			return ret;
		}

		fw = &core->fws[i];

		ret = request_firmware(&fw->fw_entry, img_name, dev);
		if (ret) {
			dev_err(dev, "Failed to load '%s' binary\n", img_name);
			return ret;
		}

		fw->node = get_core_sram(core, img_name);
		if (!fw->node) {
			dev_err(dev, "Unable to find '%s' sram node\n",
				img_name);
			return -EINVAL;
		}
	}

	return 0;
}

static void llce_flush_fw(struct llce_core *core)
{
	struct llce_fw *fw;
	size_t i;

	for (i = 0; i < core->nfrws; i++) {
		fw = &core->fws[i];

		memcpy_toio(fw->node->addr, fw->fw_entry->data,
			    fw->fw_entry->size);
	}
}

static void llce_release_fw_images(struct llce_core *core)
{
	size_t i;

	for (i = 0; i < core->nfrws; i++)
		release_firmware(core->fws[i].fw_entry);
}

static void reset_llce_cores(void __iomem *sysctrl_base)
{
	writel(0x0, sysctrl_base + LLCE_SYSRSTR);
}

static bool llce_boot_end(struct device *dev, void __iomem *status_reg,
			  bool verbose)
{
	struct llce_mgr_status mgr_status;

	memcpy_fromio(&mgr_status, status_reg, sizeof(mgr_status));

	if (mgr_status.tx_boot_status != LLCE_FW_SUCCESS) {
		if (verbose)
			dev_err(dev, "TX boot failed with status: %d\n",
				mgr_status.tx_boot_status);
		return false;
	}

	if (mgr_status.rx_boot_status != LLCE_FW_SUCCESS) {
		if (verbose)
			dev_err(dev, "RX boot failed with status: %d\n",
				mgr_status.rx_boot_status);
		return false;
	}

	if (mgr_status.dte_boot_status != LLCE_FW_SUCCESS) {
		if (verbose)
			dev_err(dev, "DTE boot failed with status: %d\n",
				mgr_status.dte_boot_status);
		return false;
	}

	if (mgr_status.frpe_boot_status != LLCE_FW_SUCCESS) {
		if (verbose)
			dev_err(dev, "FRPE boot failed with status: %d\n",
				mgr_status.frpe_boot_status);
		return false;
	}

	return true;
}

static bool llce_boot_end_or_timeout(struct device *dev,
				     void __iomem *status_reg,
				     ktime_t timeout)
{
	ktime_t cur = ktime_get();

	return llce_boot_end(dev, status_reg, false) ||
		ktime_after(cur, timeout);
}

static int llce_cores_kickoff(struct device *dev, void __iomem *sysctrl_base,
			      void __iomem *status_reg)
{
	ktime_t timeout = ktime_add_ns(ktime_get(), LLCE_BOOT_POLL_NS);
	u32 mask = LLCE_SYSRSTR_RST0 | LLCE_SYSRSTR_RST1 |
		LLCE_SYSRSTR_RST2 | LLCE_SYSRSTR_RST3;

	/* LLCE cores kickoff */
	writel(mask, sysctrl_base + LLCE_SYSRSTR);

	spin_until_cond(llce_boot_end_or_timeout(dev, status_reg, timeout));
	if (!llce_boot_end(dev, status_reg, true)) {
		dev_err(dev, "Firmware loading failed\n");
		return -EIO;
	}

	return 0;
}

static int init_core_clock(struct device *dev, struct llce_core *core)
{
	int ret;

	core->clk = devm_clk_get(dev, "llce_sys");
	if (IS_ERR(core->clk)) {
		dev_err(dev, "No clock available\n");
		return PTR_ERR(core->clk);
	}

	ret = clk_prepare_enable(core->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	return 0;
}

static void deinit_core_clock(struct llce_core *core)
{
	clk_disable_unprepare(core->clk);
}

static int start_llce_cores(struct device *dev, struct llce_core *core)
{
	void __iomem *status = get_status_regs(core);
	int ret;

	if (!status)
		return -EINVAL;

	reset_llce_cores(core->sysctrl_base);

	llce_flush_fw(core);

	ret = llce_cores_kickoff(dev, core->sysctrl_base, status);
	if (ret) {
		dev_err(dev, "Failed to start LLCE cores\n");
		return ret;
	}

	return 0;
}

static int debugfs_readl_get(void *data, u64 *val)
{
	uintptr_t addr = (uintptr_t)data;

	*val = readl((void __iomem *)addr);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_x32_readl, debugfs_readl_get, NULL, "0x%08llx\n");

static struct dentry *debugfs_create_readl(const char *name,
					   struct dentry *parent,
					   uintptr_t value)
{
	return debugfs_create_file_unsafe(name, 0400, parent, (void *)value,
					  &fops_x32_readl);
}

static void remove_debugfs_files(struct llce_core *core)
{
	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return;

	debugfs_remove_recursive(core->debugfs_root);
}

static int init_debugfs_files(struct llce_core *core)
{
	struct llce_mgr_time_stamp_cores __iomem *ts;
	struct device *dev = core->dev;
	void __iomem *cores_ts;
	struct dentry *file;

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return 0;

	cores_ts = get_core_ts_regs(core);
	if (!cores_ts) {
		dev_err(dev, "Failed to get cores TS registers\n");
		return -EINVAL;
	}

	ts = cores_ts;

	core->debugfs_root = debugfs_create_dir(LLCE_CORE_DIR, NULL);
	if (IS_ERR(core->debugfs_root)) {
		dev_err(dev, "Failed to create %s directory\n",
			LLCE_CORE_DIR);
		return PTR_ERR(core->debugfs_root);
	}

	file = debugfs_create_readl(LLCE_CORE1_TS, core->debugfs_root,
			     (uintptr_t)&ts->time_stamp_core1);
	if (IS_ERR(file))
		goto remove_folder;

	file = debugfs_create_readl(LLCE_CORE2_TS, core->debugfs_root,
			     (uintptr_t)&ts->time_stamp_core2);
	if (IS_ERR(file))
		goto remove_folder;

	file = debugfs_create_readl(LLCE_CORE3_TS, core->debugfs_root,
			     (uintptr_t)&ts->time_stamp_core3);

remove_folder:
	if (IS_ERR(file)) {
		remove_debugfs_files(core);
		return PTR_ERR(file);
	}

	return 0;
}

static int llce_core_probe(struct platform_device *pdev)
{
	struct resource *sysctrl_res;
	struct device *dev = &pdev->dev;
	struct llce_core *core;
	int ret;

	if (!load_fw)
		return devm_of_platform_populate(&pdev->dev);

	core = devm_kmalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;
	platform_set_drvdata(pdev, core);

	sysctrl_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "sysctrl");
	if (!sysctrl_res) {
		dev_err(dev, "Missing 'sysctrl' reg region.\n");
		return -EIO;
	}

	core->sysctrl_base = devm_ioremap_wc(dev, sysctrl_res->start,
					     resource_size(sysctrl_res));
	if (!core->sysctrl_base) {
		dev_err(dev, "Failed to map 'sysctrl'\n");
		return -ENOMEM;
	}

	ret = init_core_clock(dev, core);
	if (ret)
		return ret;

	ret = map_sram_nodes(dev, core);
	if (ret)
		goto disable_clk;

	init_sram_nodes(core);

	ret = llce_load_fw_images(dev, core);
	if (ret)
		goto disable_clk;

	ret = start_llce_cores(dev, core);
	if (ret)
		goto release_fw;

	dev_info(dev, "Successfully loaded LLCE firmware\n");

	ret = devm_of_platform_populate(&pdev->dev);
	if (ret) {
		dev_err(dev, "Failed to load LLCE firmware\n");
		goto release_fw;
	}

	ret = init_debugfs_files(core);
	if (ret)
		dev_err(dev, "Failed to initialize debugfs files\n");

release_fw:
	if (ret)
		llce_release_fw_images(core);
disable_clk:
	if (ret)
		deinit_core_clock(core);

	return ret;
}

static int llce_core_remove(struct platform_device *pdev)
{
	struct llce_core *core = platform_get_drvdata(pdev);

	if (!load_fw)
		return 0;

	remove_debugfs_files(core);
	llce_release_fw_images(core);
	deinit_core_clock(core);
	return 0;
}

static int __maybe_unused llce_core_suspend(struct device *dev)
{
	struct llce_core *core = dev_get_drvdata(dev);

	if (!load_fw)
		return 0;

	deinit_core_clock(core);

	return 0;
}

static int __maybe_unused llce_core_resume(struct device *dev)
{
	struct llce_core *core = dev_get_drvdata(dev);
	int ret;

	if (!load_fw)
		return 0;

	ret = init_core_clock(dev, core);
	if (ret)
		return ret;

	init_sram_nodes(core);

	return start_llce_cores(dev, core);
}

static const struct of_device_id llce_core_match[] = {
	{
		.compatible = "nxp,s32g-llce-core",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, llce_core_match);

static SIMPLE_DEV_PM_OPS(llce_core_pm_ops, llce_core_suspend, llce_core_resume);

static struct platform_driver llce_core_driver = {
	.probe = llce_core_probe,
	.remove = llce_core_remove,
	.driver = {
		.name = "llce_core",
		.of_match_table = llce_core_match,
		.pm = &llce_core_pm_ops,
	},
};
module_platform_driver(llce_core_driver)

MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("NXP LLCE Core");
MODULE_LICENSE("GPL");
