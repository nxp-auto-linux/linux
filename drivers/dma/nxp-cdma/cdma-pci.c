// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * NXP S32N79 CDMA — PCI driver entry point
 *
 * Each CDMA Station Interface (SI) appears as a separate PCIe function
 * using Enhanced Allocation for its 128 KB register aperture.
 * The PCI core auto-populates pdev->resource[0] from the EA capability.
 *
 * Each SI independently provides TX (ring 0) and RX (ring 1) using
 * TYPE=PIPE descriptors. Hardware pipe matching via IERB-configured
 * pipe IDs handles cross-SI rendezvous — no peer discovery or global
 * SI registry needed. Works for both single-Linux (two SIs) and
 * multi-Linux (one SI each) deployments.
 *
 * Probe flow:
 *   1. Enable PCI device and map BAR 0 (EA aperture)
 *   2. Read SICAPR1 to discover number of BD rings and MSI-X vectors
 *   3. Allocate SI + flex-array rings, set register base pointers
 *   4. Allocate DMA-coherent BD ring memory for each ring
 *   5. Enable the Station Interface (SIMR[EN])
 *   6. Set up MSI-X interrupts
 *   7. Expose per-function sysfs pipe configuration
 *   8. Optionally activate the configured pipe for one selected BDF
 *
 * Copyright 2026 NXP
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/string.h>
#include "cdma.h"

#define CDMA_AUTO_BDF_LEN	16

static int default_pipe_id = 10;
module_param(default_pipe_id, int, 0444);
MODULE_PARM_DESC(default_pipe_id, "Initial IERB-authorized pipe ID");

static bool auto_activate;
module_param(auto_activate, bool, 0444);
MODULE_PARM_DESC(auto_activate,
		 "Activate the default pipe on the PCI function selected by auto_activate_bdf");

static char auto_activate_bdf[CDMA_AUTO_BDF_LEN];
module_param_string(auto_activate_bdf, auto_activate_bdf,
		    sizeof(auto_activate_bdf), 0444);
MODULE_PARM_DESC(auto_activate_bdf,
		 "Exact PCI BDF to auto-activate, for example 0000:01:00.2");

/*
 * Threaded IRQ handler — shared by all BD rings within one SI.
 *
 * Runs in process context so we can safely call complete(). Completion and
 * error status comes from DMA-coherent writebacks, avoiding a PCIe register
 * read for each completed request.
 *
 * SIIDR is cleared unconditionally.
 * Per-ring completion is detected via the writeback W bit in DMA-coherent
 * memory: W=1 means pending, W=0 (inverted by HW) means complete.
 * Per the spec, a set/clear collision on SIIDR auto-generates a new
 * interrupt, so no completions are lost.
 */
static irqreturn_t cdma_irq_thread(int irq, void *data)
{
	struct cdma_si *si = data;
	struct cdma_ring *ring;
	struct cdma_wb *wb;
	unsigned int i;
	u32 status;

	/* Clear all pending completion/threshold interrupts */
	cdma_si_write(si, CDMA_SIIDR, ~0U);

	for (i = 0; i < si->num_rings; i++) {
		bool async_completed = false;

		ring = &si->rings[i];

		while (cdma_ring_async_pending(ring)) {
			bool pass_completed = false;

			cdma_ring_read(ring, CDMA_BaIDR);

			while (1) {
				struct cdma_async_req *req;
				int result;

				req = cdma_ring_complete_async(ring, &status);
				if (!req)
					break;
				async_completed = true;
				pass_completed = true;

				if (status & (CDMA_WB_DE | CDMA_WB_SPE | CDMA_WB_DPE)) {
					dev_err(&si->cdev->pdev->dev,
						"ring%u IRQ writeback error: status=0x%08x\n",
						i, status);
					result = -EIO;
				} else {
					result = 0;
				}

				cdma_async_req_complete(req, result);
			}

			if (!pass_completed)
				break;
		}

		if (cdma_ring_async_pending(ring) || async_completed)
			continue;

		/* Writeback overwrites SD at the position recorded during submit */
		wb = ring->bd_base + ring->wb_idx * CDMA_BD_ENTRY_SIZE;
		dma_rmb();
		status = le32_to_cpu(READ_ONCE(wb->status));

		if (!(status & CDMA_WB_W)) {
			int result;

			/* W=0 means writeback complete */
			if (status & (CDMA_WB_DE | CDMA_WB_SPE | CDMA_WB_DPE)) {
				dev_err(&si->cdev->pdev->dev,
					"ring%u IRQ writeback error: status=0x%08x\n",
					i, status);
				result = -EIO;
			} else {
				result = 0;
			}

			ring->result = result;
			complete(&ring->done);
		}
	}

	return IRQ_HANDLED;
}

/*
 * cdma_irq_init - Allocate MSI-X vectors and setup IRQ for an SI.
 *
 * CDMA hardware only supports MSI-X (no MSI capability). Since probe
 * already validated num_msix > 0 from SICAPR1, allocation failure here
 * indicates a system-level problem and should fail probe.
 *
 * Allocates one MSI-X vector (minimum) shared across all BD rings in the SI.
 * Each BD ring's BaIVR is pointed at vector 0.
 */
static int cdma_irq_init(struct cdma_si *si)
{
	struct pci_dev *pdev = si->cdev->pdev;
	struct cdma_ring *ring;
	unsigned int i;
	int ret;

	ret = pci_alloc_irq_vectors(pdev, 1, si->num_msix, PCI_IRQ_MSIX);
	if (ret < 0) {
		dev_err(&pdev->dev, "MSI-X alloc failed: %d\n", ret);
		return ret;
	}

	si->irq = pci_irq_vector(pdev, 0);

	ret = request_threaded_irq(si->irq, NULL, cdma_irq_thread,
				   IRQF_ONESHOT, CDMA_DRV_NAME, si);
	if (ret) {
		dev_err(&pdev->dev, "request_threaded_irq failed: %d\n", ret);
		pci_free_irq_vectors(pdev);
		return ret;
	}

	/*
	 * Point all BD rings at MSI-X vector 0 and enable completion IRQ.
	 * When multiple rings are active concurrently, distribute them
	 * across available MSI-X vectors for better interrupt spreading.
	 */
	for (i = 0; i < si->num_rings; i++) {
		ring = &si->rings[i];

		cdma_ring_write(ring, CDMA_BaIVR, 0);
		cdma_ring_write(ring, CDMA_BaIER, CDMA_BaIER_CIE);
	}

	dev_info(&pdev->dev, "MSI-X enabled (IRQ %d)\n", si->irq);
	return 0;
}

static void cdma_irq_fini(struct cdma_si *si)
{
	unsigned int i;

	/* Disable interrupts on all rings */
	for (i = 0; i < si->num_rings; i++)
		cdma_ring_write(&si->rings[i], CDMA_BaIER, 0);

	free_irq(si->irq, si);
	pci_free_irq_vectors(si->cdev->pdev);
}

static int cdma_si_init(struct cdma_device *cdev, void __iomem *base)
{
	struct pci_dev *pdev = cdev->pdev;
	u32 capr1, num_rings, num_msix;
	struct cdma_ring *ring;
	struct cdma_si *si;
	unsigned int i;
	int ret;

	/* Discover capabilities from read-only SICAPR1 register */
	capr1 = ioread32(base + CDMA_SICAPR1);
	num_rings = capr1 & CDMA_SICAPR1_NUM_BDR_MASK;
	num_msix  = (capr1 & CDMA_SICAPR1_NUM_MSIX_MASK) >>
		    CDMA_SICAPR1_NUM_MSIX_SHIFT;

	if (!num_rings || !num_msix) {
		dev_err(&pdev->dev, "SICAPR1=0x%08x: no %s\n", capr1,
			!num_rings ? "BD rings" : "MSI-X vectors");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "SICAPR1=0x%08x: %u BD rings, %u MSI-X vectors\n",
		 capr1, num_rings, num_msix);

	/* Allocate SI with flex-array of rings */
	si = devm_kzalloc(&pdev->dev,
			  struct_size(si, rings, num_rings), GFP_KERNEL);
	if (!si)
		return -ENOMEM;

	si->cdev = cdev;
	si->regs = base;
	si->num_rings = num_rings;
	si->num_msix = num_msix;
	si->irq = 0;

	/* Initialize each BD ring */
	for (i = 0; i < num_rings; i++) {
		ring = &si->rings[i];

		ring->si = si;
		ring->idx = i;
		ring->regs = cdma_ring_reg_base(si, i);

		ret = cdma_ring_alloc(ring);
		if (ret) {
			dev_err(&pdev->dev,
				"ring %u: alloc failed (%d)\n", i, ret);
			goto err_free_rings;
		}

		/* Configure and enable ring - stays enabled across transfers */
		ret = cdma_ring_init(ring);
		if (ret) {
			dev_err(&pdev->dev,
				"ring %u: enable failed (%d)\n", i, ret);
			goto err_free_rings;
		}
	}

	/* Enable the Station Interface */
	cdma_si_write(si, CDMA_SIMR, CDMA_SIMR_EN);

	cdev->si = si;
	return 0;

err_free_rings:
	while (i > 0)
		cdma_ring_free(&si->rings[--i]);
	return ret;
}

static void cdma_si_fini(struct cdma_device *cdev)
{
	struct cdma_si *si = cdev->si;
	unsigned int i;

	if (!si)
		return;

	/* Disable SI */
	cdma_si_write(si, CDMA_SIMR, 0);

	/* Free ring DMA memory */
	for (i = 0; i < si->num_rings; i++)
		cdma_ring_free(&si->rings[i]);

	cdev->si = NULL;
}

static int cdma_activate_pipe(struct cdma_device *cdev)
{
	int ret;

	lockdep_assert_held(&cdev->config_lock);

	if (cdev->pipe_active)
		return 0;
	if (cdev->si->num_rings < 2)
		return -ENOSPC;

	ret = cdma_cdev_init(cdev);
	if (!ret)
		cdev->pipe_active = true;

	return ret;
}

static ssize_t pipe_id_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct cdma_device *cdev = dev_get_drvdata(dev);
	u8 pipe_id;

	mutex_lock(&cdev->config_lock);
	pipe_id = cdev->pipe_id;
	mutex_unlock(&cdev->config_lock);

	return sysfs_emit(buf, "%u\n", pipe_id);
}

static ssize_t pipe_id_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct cdma_device *cdev = dev_get_drvdata(dev);
	u8 pipe_id;
	int ret;

	ret = kstrtou8(buf, 0, &pipe_id);
	if (ret)
		return ret;

	mutex_lock(&cdev->config_lock);
	if (cdev->pipe_active)
		ret = -EBUSY;
	else
		cdev->pipe_id = pipe_id;
	mutex_unlock(&cdev->config_lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(pipe_id);

static ssize_t activate_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct cdma_device *cdev = dev_get_drvdata(dev);
	bool activate;
	int ret;

	ret = kstrtobool(buf, &activate);
	if (ret)
		return ret;
	if (!activate)
		return -EINVAL;

	mutex_lock(&cdev->config_lock);
	ret = cdma_activate_pipe(cdev);
	mutex_unlock(&cdev->config_lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(activate);

static ssize_t state_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct cdma_device *cdev = dev_get_drvdata(dev);
	bool active;
	u8 pipe_id;

	mutex_lock(&cdev->config_lock);
	active = cdev->pipe_active;
	pipe_id = cdev->pipe_id;
	mutex_unlock(&cdev->config_lock);

	return sysfs_emit(buf, "%s pipe_id=%u pci=%s\n",
			  active ? "active" : "inactive", pipe_id,
			  pci_name(cdev->pdev));
}
static DEVICE_ATTR_RO(state);

static struct attribute *cdma_config_attrs[] = {
	&dev_attr_pipe_id.attr,
	&dev_attr_activate.attr,
	&dev_attr_state.attr,
	NULL,
};

static const struct attribute_group cdma_config_group = {
	.name = "cdma",
	.attrs = cdma_config_attrs,
};

static const struct attribute_group *cdma_config_groups[] = {
	&cdma_config_group,
	NULL,
};

static int cdma_pci_probe(struct pci_dev *pdev,
			  const struct pci_device_id *id)
{
	struct cdma_device *cdev;
	void __iomem *base;
	int ret;

	cdev = devm_kzalloc(&pdev->dev, sizeof(*cdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;
	if (default_pipe_id < 0 || default_pipe_id > U8_MAX) {
		dev_err(&pdev->dev, "default_pipe_id %d is outside [0, %u]\n",
			default_pipe_id, U8_MAX);
		return -EINVAL;
	}

	cdev->pdev = pdev;
	mutex_init(&cdev->config_lock);
	cdev->pipe_id = default_pipe_id;
	pci_set_drvdata(pdev, cdev);

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "pcim_enable_device failed: %d\n", ret);
		return ret;
	}

	/*
	 * Request and map BAR 0 - populated by PCI core from EA capability.
	 * The EA aperture is 128 KB covering SI registers + MSI-X table/PBA.
	 * pcim_iomap_region() claims the region so it shows up in
	 * /proc/iomem and refuses any conflicting mapper.
	 */
	base = pcim_iomap_region(pdev, 0, KBUILD_MODNAME);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		dev_err(&pdev->dev, "BAR 0 iomap failed (EA aperture): %d\n", ret);
		return ret;
	}

	/* Set DMA mask - CDMA descriptors have 64-bit address fields */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret) {
		/* Failsafe to 32-bit addresses */
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(&pdev->dev, "DMA mask setup failed: %d\n",
				ret);
			return ret;
		}
	}

	/* Initialize Station Interface - clear SIMR, reset rings to a known
	 * state before allowing the device to bus-master.
	 */
	ret = cdma_si_init(cdev, base);
	if (ret)
		return ret;

	/* Device is now in a defined state - enable bus mastering so it can
	 * initiate DMA when rings are subsequently armed.
	 */
	pci_set_master(pdev);

	/* Create DMA pool for S/G tables */
	cdev->sgt_pool = dma_pool_create("cdma-sgt", &pdev->dev,
					 CDMA_SGT_SIZE, CDMA_SGT_ALIGN, 0);
	if (!cdev->sgt_pool) {
		dev_err(&pdev->dev, "failed to create S/G table pool\n");
		ret = -ENOMEM;
		goto err_si;
	}

	/*
	 * Dedicated workqueue for deferred async completions. Having our own
	 * queue lets cdma_pci_remove() flush every outstanding completion
	 * before the BD rings and DMA pool they touch are torn down.
	 */
	cdev->wq = alloc_workqueue("nxp-cdma", WQ_UNBOUND, 0);
	if (!cdev->wq) {
		dev_err(&pdev->dev, "failed to create completion workqueue\n");
		ret = -ENOMEM;
		goto err_pool;
	}

	/* Set up MSI-X interrupts */
	ret = cdma_irq_init(cdev->si);
	if (ret) {
		dev_err(&pdev->dev, "IRQ init failed: %d\n", ret);
		goto err_wq;
	}

	if (auto_activate && !auto_activate_bdf[0]) {
		dev_err(&pdev->dev,
			"auto_activate requires an exact auto_activate_bdf\n");
		ret = -EINVAL;
		goto err_irq;
	}

	if (auto_activate && !strcmp(auto_activate_bdf, pci_name(pdev))) {
		mutex_lock(&cdev->config_lock);
		ret = cdma_activate_pipe(cdev);
		mutex_unlock(&cdev->config_lock);
		if (ret) {
			dev_err(&pdev->dev,
				"default pipe activation failed: %d\n", ret);
			goto err_irq;
		}
	}

	dev_info(&pdev->dev,
		 "NXP CDMA ready (%u rings, pipe_id=%u, state=%s)\n",
		 cdev->si->num_rings, cdev->pipe_id,
		 cdev->pipe_active ? "active" : "inactive");

	return 0;

err_irq:
	cdma_irq_fini(cdev->si);
err_wq:
	destroy_workqueue(cdev->wq);
err_pool:
	dma_pool_destroy(cdev->sgt_pool);
err_si:
	cdma_si_fini(cdev);
	return ret;
}

/*
 * Teardown ordering matters here.
 *
 * Open file descriptors are expected to be closed before the driver is
 * unbound. The pipe memory is devm-allocated against the PCI device, so a
 * per-function unbind (or surprise removal) reclaims it; the driver does
 * NOT guarantee that an fd may outlive its pipe. See cdma_{tx,rx}_open().
 *
 * The SI is disabled (SIMR=0) up front so hardware can no longer land a
 * writeback into a BD ring while cdma_pipe_destroy() reaps in-flight async
 * requests. cdma_irq_fini() then runs before cdma_cdev_fini() so the
 * threaded IRQ handler can no longer schedule async completions while
 * cdma_pipe_destroy() drains the rings and flushes the completion
 * workqueue. The workqueue is destroyed only after the drain/flush, and
 * before the DMA pool so no completion can reference freed pool memory.
 */
static void cdma_pci_remove(struct pci_dev *pdev)
{
	struct cdma_device *cdev = pci_get_drvdata(pdev);

	mutex_lock(&cdev->config_lock);
	cdev->pipe_active = false;
	mutex_unlock(&cdev->config_lock);

	/*
	 * Quiesce the SI before reaping in-flight transfers. Skip the MMIO
	 * write if the function is already gone (surprise removal / link
	 * down), where it would raise a synchronous external abort.
	 */
	if (!pci_channel_offline(pdev) && !pci_dev_is_disconnected(pdev))
		cdma_si_write(cdev->si, CDMA_SIMR, 0);

	cdma_irq_fini(cdev->si);
	cdma_cdev_fini(cdev);
	destroy_workqueue(cdev->wq);
	dma_pool_destroy(cdev->sgt_pool);
	cdma_si_fini(cdev);
}

/*
 * PCI vendor 0x1131 is used by NXP for CDMA.
 * Device ID 0xe200 is programmed in IERB by firmware.
 */
static const struct pci_device_id cdma_pci_ids[] = {
	{ PCI_DEVICE(CDMA_PCI_VENDOR_ID, CDMA_PCI_DEVICE_ID) },
	{ }
};
MODULE_DEVICE_TABLE(pci, cdma_pci_ids);

static struct pci_driver cdma_pci_driver = {
	.name     = CDMA_DRV_NAME,
	.id_table = cdma_pci_ids,
	.probe    = cdma_pci_probe,
	.remove   = cdma_pci_remove,
	.dev_groups = cdma_config_groups,
};
module_pci_driver(cdma_pci_driver);

MODULE_AUTHOR("Ciprian Marian Costea <ciprianmarian.costea@nxp.com>");
MODULE_DESCRIPTION("NXP S32N79 Central DMA (CDMA) Controller Driver");
MODULE_LICENSE("Dual BSD/GPL");
