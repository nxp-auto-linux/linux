// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2020 NXP */
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/genalloc.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/processor.h>
#include <linux/slab.h>

#define LLCE_STATUS_POOL	"llce_boot_status"
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

struct llce_mrg_status {
	u32 tx;
	u32 rx;
	u32 dte;
	u32 frpe;
	u32 error_cnt;
	u32 stm_init_cnt;
};

struct llce_fw_cont {
	struct platform_device *pdev;
	int index;
	const char *img_name;
	u16 retries;
};

struct sram_pool {
	struct gen_pool *pool;
	const struct firmware *fw_entry;
	size_t size;
	unsigned long vaddr;
	const char *name;
};

struct llce_core {
	struct clk *clk;
	void __iomem *sysctrl_base;

	/* SRAM pools */
	struct sram_pool **pools;
	size_t npools;

	size_t nfrws;
};

static void devm_sram_pool_release(struct device *dev, void *res)
{
	struct sram_pool *spool = res;

	gen_pool_free(spool->pool, spool->vaddr, spool->size);
}

static struct sram_pool *devm_sram_pool_alloc(struct device *dev,
					      struct gen_pool *pool)
{
	struct sram_pool *spool = devres_alloc(devm_sram_pool_release,
					       sizeof(*spool), GFP_KERNEL);

	if (!spool)
		return ERR_PTR(-ENOMEM);

	spool->size = gen_pool_size(pool);
	spool->vaddr = gen_pool_alloc(pool, spool->size);
	if (!spool->vaddr) {
		devres_free(spool);
		return ERR_PTR(-ENOMEM);
	}

	spool->pool = pool;
	devres_add(dev, spool);

	return spool;
}

static struct sram_pool *alloc_sram_node(struct device_node *sram_node,
					 struct device *dev)
{
	struct platform_device *pdev;
	struct gen_pool *pool;
	struct sram_pool *spool;
	const char *name = sram_node->name;

	pdev = of_find_device_by_node(sram_node);
	if (!pdev) {
		dev_err(dev, "failed to find SRAM device for '%s'!\n", name);
		return ERR_PTR(-ENODEV);
	}

	pool = gen_pool_get(&pdev->dev, NULL);
	if (!pool) {
		dev_err(dev, "Pool '%s' is unavailable!\n", name);
		return ERR_PTR(-ENODEV);
	}

	spool = devm_sram_pool_alloc(dev, pool);
	if (IS_ERR(spool)) {
		dev_err(dev, "Unable to allocate '%s' pool\n", name);
		return spool;
	}
	spool->name = name;

	return spool;
}

static struct sram_pool *alloc_sram_node_index(struct platform_device *pdev,
					       int index)
{
	struct device_node *sram_node;

	sram_node = of_parse_phandle(pdev->dev.of_node, "memory-region", index);
	if (!sram_node) {
		dev_err(&pdev->dev, "Failed to get the element %d from 'memory-region' list\n",
			index);
		return ERR_PTR(-EINVAL);
	}

	return alloc_sram_node(sram_node, &pdev->dev);
}

static int llce_fw_load(struct platform_device *pdev, int index,
			struct sram_pool *spool)
{
	int ret;
	const char *img_name;

	ret = of_property_read_string_index(pdev->dev.of_node, "firmware-name",
					    index, &img_name);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get 'firmware-name' property\n");
		return ret;
	}

	ret = request_firmware(&spool->fw_entry, img_name, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to load '%s' binary\n", img_name);
		return ret;
	}

	return ret;
}

static void llce_release_fw(struct sram_pool *spool)
{
	release_firmware(spool->fw_entry);
}

static int llce_alloc_sram(struct platform_device *pdev,
			   struct llce_core *core)
{
	int i;
	struct sram_pool *spool;

	core->npools = of_count_phandle_with_args(pdev->dev.of_node,
						  "memory-region",
						  NULL);
	core->pools = devm_kmalloc(&pdev->dev,
				   core->npools * sizeof(*core->pools),
				   GFP_KERNEL);
	if (!core->pools)
		return -ENOMEM;

	for (i = 0; i < core->npools; i++) {
		spool = alloc_sram_node_index(pdev, i);
		if (IS_ERR(spool)) {
			dev_err(&pdev->dev, "Failed to initialize SRAM buffer %d\n",
				i);
			return PTR_ERR(spool);
		}
		core->pools[i] = spool;
	}

	return 0;
}

static int llce_load_fw_images(struct platform_device *pdev,
			       struct llce_core *core)
{
	int i, ret;

	core->nfrws = of_property_count_strings(pdev->dev.of_node,
						"firmware-name");
	for (i = 0; i < core->nfrws; i++) {
		ret = llce_fw_load(pdev, i, core->pools[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static void llce_flush_fw(struct llce_core *core)
{
	struct sram_pool *pool;
	size_t i;

	for (i = 0; i < core->nfrws; i++) {
		pool = core->pools[i];

		memcpy_toio((void __iomem *)pool->vaddr, pool->fw_entry->data,
			    pool->fw_entry->size);
	}
}

static void llce_release_fw_images(struct llce_core *core)
{
	size_t i;

	for (i = 0; i < core->nfrws; i++)
		llce_release_fw(core->pools[i]);
}

static void reset_llce_cores(void __iomem *sysctrl_base)
{
	writel(0x0, sysctrl_base + LLCE_SYSRSTR);
}

static bool llce_boot_end(void *status_reg)
{
	struct llce_mrg_status *mgr_status = status_reg;
	u32 status;

	status = mgr_status->tx;
	status |= mgr_status->rx;
	status |= mgr_status->dte;
	status |= mgr_status->frpe;

	return status == LLCE_MGR_BOOT_END_ALL_CORES_MASK;
}

static bool llce_boot_end_or_timeout(void *status_reg, ktime_t timeout)
{
	ktime_t cur = ktime_get();

	return llce_boot_end(status_reg) || ktime_after(cur, timeout);
}

static int llce_cores_kickoff(struct device *dev, void __iomem *sysctrl_base,
			    void *status_reg)
{
	ktime_t timeout = ktime_add_ns(ktime_get(), LLCE_BOOT_POLL_NS);
	u32 mask = LLCE_SYSRSTR_RST0 | LLCE_SYSRSTR_RST1 |
		LLCE_SYSRSTR_RST2 | LLCE_SYSRSTR_RST3;

	/* LLCE cores kickoff */
	writel(mask, sysctrl_base + LLCE_SYSRSTR);

	spin_until_cond(llce_boot_end_or_timeout(status_reg, timeout));
	if (!llce_boot_end(status_reg)) {
		dev_err(dev, "Firmware loading failed\n");
		return -EIO;
	}

	return 0;
}

static struct sram_pool *get_status_pool(struct llce_core *core)
{
	size_t i;
	struct sram_pool *pool;

	for (i = 0; i < core->npools; i++) {
		pool = core->pools[i];
		if (!strcmp(LLCE_STATUS_POOL, pool->name))
			return pool;
	}

	return NULL;
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
	struct sram_pool *status_pool;
	int ret;

	reset_llce_cores(core->sysctrl_base);

	llce_flush_fw(core);

	status_pool = get_status_pool(core);
	if (!status_pool) {
		dev_err(dev, "'%s' pool is not attached to LLCE core\n",
			LLCE_STATUS_POOL);
		return -EIO;
	}

	ret = llce_cores_kickoff(dev, core->sysctrl_base,
				 (void *)status_pool->vaddr);
	if (ret) {
		dev_err(dev, "Failed to start LLCE cores\n");
		return ret;
	}

	return 0;
}

static int llce_core_probe(struct platform_device *pdev)
{
	struct resource *sysctrl_res;
	struct device *dev = &pdev->dev;
	struct llce_core *core;
	int ret;

	core = devm_kmalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	platform_set_drvdata(pdev, core);

	sysctrl_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "sysctrl");
	if (!sysctrl_res) {
		dev_err(dev, "Missing 'sysctrl' reg region.\n");
		return -EIO;
	}

	core->sysctrl_base = devm_ioremap(dev, sysctrl_res->start,
				    resource_size(sysctrl_res));
	if (!core->sysctrl_base) {
		dev_err(dev, "Failed to map 'sysctrl'\n");
		return -ENOMEM;
	}

	ret = init_core_clock(dev, core);
	if (ret)
		return ret;

	ret = llce_alloc_sram(pdev, core);
	if (ret)
		goto disable_clk;

	ret = llce_load_fw_images(pdev, core);
	if (ret)
		goto disable_clk;

	ret = start_llce_cores(dev, core);
	if (ret)
		goto release_fw;

	dev_info(dev, "Successfully loaded LLCE firmware\n");

	ret = devm_of_platform_populate(&pdev->dev);
	if (ret)
		dev_err(dev, "Failed to load LLCE firmware\n");

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

	llce_release_fw_images(core);
	deinit_core_clock(core);
	return 0;
}

static int __maybe_unused llce_core_suspend(struct device *dev)
{
	struct llce_core *core = dev_get_drvdata(dev);

	deinit_core_clock(core);

	return 0;
}

static int __maybe_unused llce_core_resume(struct device *dev)
{
	struct llce_core *core = dev_get_drvdata(dev);
	int ret;

	ret = init_core_clock(dev, core);
	if (ret)
		return ret;

	return start_llce_cores(dev, core);
}

static const struct of_device_id llce_core_match[] = {
	{
		.compatible = "nxp,s32g274a-llce-core",
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
