// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * NXP S32N79 CDMA — PCI driver entry point
 *
 * Each CDMA Station Interface (SI) appears as a separate PCIe function
 * using Enhanced Allocation for its 128 KB register aperture.
 * The PCI core auto-populates pdev->resource[0] from the EA capability.
 *
 * Probe flow:
 *   1. Enable PCI device and map BAR 0 (EA aperture)
 *   2. Read SICAPR1 to discover number of BD rings and MSI-X vectors
 *   3. Allocate SI + flex-array rings, set register base pointers
 *   4. Allocate DMA-coherent BD ring memory for each ring
 *   5. Enable the Station Interface (SIMR[EN])
 *   6. Set up MSI-X interrupts
 *   7. Create pipe char devices (/dev/cdma-pipeN-tx, /dev/cdma-pipeN-rx)
 *
 * Copyright 2026 NXP
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include "cdma.h"

/*
 * Threaded IRQ handler — shared by all BD rings within one SI.
 *
 * Runs in process context so we can safely call complete().  Uses writeback
 * descriptors in DMA-coherent memory to detect completion and errors,
 * avoiding all PCIe register reads on the hot path.
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
		ring = &si->rings[i];

		/* Writeback overwrites SD at the position recorded during submit */
		wb = ring->bd_base + ring->wb_idx * CDMA_BD_ENTRY_SIZE;
		dma_rmb();
		status = le32_to_cpu(READ_ONCE(wb->status));

		if (!(status & CDMA_WB_W)) {
			/* W=0 means writeback complete */
			if (status & (CDMA_WB_DE | CDMA_WB_SPE | CDMA_WB_DPE)) {
				dev_err(&si->cdev->pdev->dev,
					"SI%u ring%u IRQ writeback error: status=0x%08x\n",
					si->si_idx, i, status);
				ring->result = -EIO;
			} else {
				ring->result = 0;
			}
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

	dev_info(&pdev->dev, "SI%u: MSI-X enabled (IRQ %d)\n",
		 si->si_idx, si->irq);
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
	si->si_idx = PCI_SLOT(pdev->devfn);
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

static int cdma_pci_probe(struct pci_dev *pdev,
			  const struct pci_device_id *id)
{
	struct cdma_device *cdev;
	void __iomem *base;
	int ret;

	cdev = devm_kzalloc(&pdev->dev, sizeof(*cdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;

	cdev->pdev = pdev;
	pci_set_drvdata(pdev, cdev);

	/* Enable PCI device (managed) */
	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "pcim_enable_device failed: %d\n", ret);
		return ret;
	}

	pci_set_master(pdev);

	/* Set DMA mask — CDMA descriptors have 64-bit address fields */
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

	/*
	 * Map BAR 0 — populated by PCI core from EA capability (BEI=0).
	 * The EA aperture is 128 KB covering SI registers + MSI-X table/PBA.
	 */
	base = pcim_iomap(pdev, 0, 0);
	if (!base) {
		dev_err(&pdev->dev, "BAR 0 iomap failed (EA aperture)\n");
		return -ENOMEM;
	}

	/* Initialize Station Interface */
	ret = cdma_si_init(cdev, base);
	if (ret)
		return ret;

	/* Set up MSI-X interrupts */
	ret = cdma_irq_init(cdev->si);
	if (ret) {
		dev_err(&pdev->dev, "IRQ init failed: %d\n", ret);
		goto err_si;
	}

	/* Create pipe char devices (/dev/cdma-pipe0-tx, /dev/cdma-pipe0-rx) */
	ret = cdma_cdev_init(cdev);
	if (ret) {
		dev_err(&pdev->dev, "cdev init failed: %d\n", ret);
		goto err_irq;
	}

	dev_info(&pdev->dev, "NXP CDMA SI%u ready (%u rings)\n",
		 cdev->si->si_idx, cdev->si->num_rings);
	return 0;

err_irq:
	cdma_irq_fini(cdev->si);
err_si:
	cdma_si_fini(cdev);
	return ret;
}

static void cdma_pci_remove(struct pci_dev *pdev)
{
	struct cdma_device *cdev = pci_get_drvdata(pdev);

	cdma_cdev_fini(cdev);
	cdma_irq_fini(cdev->si);
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
};
module_pci_driver(cdma_pci_driver);

MODULE_AUTHOR("Ciprian Marian Costea <ciprianmarian.costea@nxp.com>");
MODULE_DESCRIPTION("NXP S32N79 Central DMA (CDMA) Controller Driver");
MODULE_LICENSE("Dual BSD/GPL");
