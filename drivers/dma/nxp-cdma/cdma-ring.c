// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * NXP S32N79 CDMA — BD Ring management and descriptor construction
 *
 * Copyright 2026 NXP
 */

#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/iopoll.h>
#include "cdma.h"

/*
 * cdma_ring_alloc - Allocate DMA-coherent memory for a BD ring.
 *
 * The ring memory holds CDMA_BD_ENTRIES × 16-byte descriptors and must be
 * 64-byte aligned (hardware requirement).
 */
int cdma_ring_alloc(struct cdma_ring *ring)
{
	struct device *dev = &ring->si->cdev->pdev->dev;

	ring->bd_base = dma_alloc_coherent(dev, CDMA_BD_RING_SIZE,
					   &ring->bd_dma, GFP_KERNEL);
	if (!ring->bd_base)
		return -ENOMEM;

	init_completion(&ring->done);
	return 0;
}

void cdma_ring_free(struct cdma_ring *ring)
{
	struct device *dev;

	if (!ring->bd_base)
		return;

	dev = &ring->si->cdev->pdev->dev;
	dma_free_coherent(dev, CDMA_BD_RING_SIZE, ring->bd_base, ring->bd_dma);
	ring->bd_base = NULL;
}

/*
 * cdma_ring_reset - Disable ring and reset indices.
 */
void cdma_ring_reset(struct cdma_ring *ring)
{
	u32 mr;

	/* Disable ring */
	mr = cdma_ring_read(ring, CDMA_BaMR);
	mr &= ~CDMA_BaMR_EN;
	cdma_ring_write(ring, CDMA_BaMR, mr);

	/* Clear any pending errors (w1c) */
	cdma_ring_write(ring, CDMA_BaSR, CDMA_BaSR_ERR_MASK);

	/* Reset indices */
	cdma_ring_write(ring, CDMA_BaPIR, 0);
	cdma_ring_write(ring, CDMA_BaCIR, 0);

	/* Reset software state */
	ring->pir = 0;
	ring->wb_idx = 0;
}

/*
 * cdma_ring_wait_idle - Wait for BaSR[BUSY] to clear.
 *
 * Spec requires BUSY=0 before re-enabling a ring to avoid conflicts
 * with cleanup of previously started transfers.
 */
static int cdma_ring_wait_idle(struct cdma_ring *ring)
{
	u32 sr;
	int ret;

	ret = read_poll_timeout(cdma_ring_read, sr, !(sr & CDMA_BaSR_BUSY),
				10, CDMA_POLL_TIMEOUT_US, false, ring, CDMA_BaSR);
	if (ret)
		dev_err(&ring->si->cdev->pdev->dev,
			"ring%u: timeout waiting for BUSY to clear (BSR=0x%08x)\n",
			ring->idx, sr);

	return ret;
}

/*
 * cdma_ring_cancel - Safely abort a pending ring operation and recover.
 *
 * Disables the ring, waits for hardware to go idle, clears errors, resets
 * indices, then re-enables the ring (with WBM). After this returns, the
 * ring is in the same state as right after cdma_ring_init() — ready for
 * fresh submissions.
 *
 * Used on signal-interrupted waits (-ERESTARTSYS) before unmapping DMA
 * buffers, and on hardware errors (BaSR[PE/SBE/ME]) to clear the failure.
 *
 * If BUSY does not clear within the timeout, the ring is still re-enabled
 * — the hardware is stalled and only an FLR / soft reset can recover, so
 * we let the caller unmap DMA buffers and propagate the error.
 */
void cdma_ring_cancel(struct cdma_ring *ring)
{
	struct pci_dev *pdev = ring->si->cdev->pdev;
	u32 mr;

	/*
	 * If the peer has gone away (PCIe link down, surprise removal, or the
	 * remote SoC rebooted mid-transfer), MMIO reads into the SI's BAR
	 * raise a synchronous external abort instead of returning all-ones.
	 * Bail out early so the dying ring doesn't take the kernel down with
	 * it - resources get cleaned up when the driver eventually unbinds.
	 */
	if (pci_channel_offline(pdev) || pci_dev_is_disconnected(pdev))
		return;

	/* Disable ring */
	mr = cdma_ring_read(ring, CDMA_BaMR);
	mr &= ~CDMA_BaMR_EN;
	cdma_ring_write(ring, CDMA_BaMR, mr);

	cdma_ring_wait_idle(ring);

	/* Clear any pending errors (w1c) */
	cdma_ring_write(ring, CDMA_BaSR, CDMA_BaSR_ERR_MASK);

	/* Reset indices */
	cdma_ring_write(ring, CDMA_BaPIR, 0);
	cdma_ring_write(ring, CDMA_BaCIR, 0);
	ring->pir = 0;
	ring->wb_idx = 0;

	/* Clear stale interrupt detect */
	cdma_ring_read(ring, CDMA_BaIDR);

	/* Re-enable with writeback mode (matches cdma_ring_init) */
	mr |= CDMA_BaMR_EN | CDMA_BaMR_WBM;
	cdma_ring_write(ring, CDMA_BaMR, mr);
}

/*
 * cdma_ring_init - Configure and enable a BD ring.
 *
 * Called once during SI init. Programs base address, length, and enables
 * the ring with writeback mode. The ring stays enabled across transfers;
 * only cdma_ring_submit() advances PIR to kick new work.
 *
 * After an error, call cdma_ring_reset() then cdma_ring_init() to restart.
 */
int cdma_ring_init(struct cdma_ring *ring)
{
	int ret;
	u32 mr;

	if (!ring->bd_base)
		return -EINVAL;

	/* Ensure ring is disabled before configuration */
	mr = cdma_ring_read(ring, CDMA_BaMR);
	mr &= ~CDMA_BaMR_EN;
	cdma_ring_write(ring, CDMA_BaMR, mr);

	ret = cdma_ring_wait_idle(ring);
	if (ret)
		return ret;

	/* Program base address */
	cdma_ring_write(ring, CDMA_BaBAR0, lower_32_bits(ring->bd_dma));
	cdma_ring_write(ring, CDMA_BaBAR1, upper_32_bits(ring->bd_dma));

	/* Program ring length */
	cdma_ring_write(ring, CDMA_BaLENR,
			CDMA_BaLENR_LENGTH(CDMA_BD_ENTRIES));

	/* Reset indices */
	cdma_ring_write(ring, CDMA_BaCIR, 0);
	cdma_ring_write(ring, CDMA_BaPIR, 0);
	ring->pir = 0;
	ring->wb_idx = 0;

	/* Clear any stale errors and interrupt detect */
	cdma_ring_write(ring, CDMA_BaSR, CDMA_BaSR_ERR_MASK);
	cdma_ring_read(ring, CDMA_BaIDR);

	/* Enable ring with writeback mode - stays enabled across transfers */
	mr |= CDMA_BaMR_EN | CDMA_BaMR_WBM;
	cdma_ring_write(ring, CDMA_BaMR, mr);

	return 0;
}

/*
 * cdma_ring_submit - Submit BDs by advancing the producer index.
 * @num_bds: number of 16-byte BD entries to process
 *
 * The ring must already be enabled via cdma_ring_init(). Caller must
 * have written descriptors at ring->pir position before calling.
 */
int cdma_ring_submit(struct cdma_ring *ring, u32 num_bds)
{
	struct pci_dev *pdev = ring->si->cdev->pdev;

	if (!ring->bd_base || num_bds > CDMA_BD_ENTRIES)
		return -EINVAL;

	/*
	 * If the peer has gone away (PCIe link down, surprise removal, or
	 * the remote SoC rebooted), the BaPIR MMIO write below would raise
	 * a synchronous external abort.
	 */
	if (pci_channel_offline(pdev) || pci_dev_is_disconnected(pdev))
		return -ENODEV;

	/* Record SD position for writeback detection in IRQ handler */
	ring->wb_idx = ring->pir;

	/* Advance producer index (wraps at ring length) */
	ring->pir = (ring->pir + num_bds) % CDMA_BD_ENTRIES;

	/* Prepare for completion */
	reinit_completion(&ring->done);
	ring->result = 0;

	/* Clear BaIDR so next completion generates a fresh interrupt */
	cdma_ring_read(ring, CDMA_BaIDR);

	/* Ensure BD writes are visible before advancing PIR */
	dma_wmb();

	/* Advance hardware producer index - HW begins processing */
	cdma_ring_write(ring, CDMA_BaPIR, ring->pir);

	return 0;
}

static inline void cdma_write_sd(void *bd, u64 addr, u32 length, u32 config)
{
	struct cdma_sd *sd = bd;

	sd->addr   = cpu_to_le64(addr);
	sd->length = cpu_to_le32(length);
	sd->config = cpu_to_le32(config);
}

static inline void cdma_write_dd(void *bd, u64 addr, u32 length, u32 config)
{
	struct cdma_dd *dd = bd;

	dd->addr   = cpu_to_le64(addr);
	dd->length = cpu_to_le32(length);
	dd->config = cpu_to_le32(config);
}

/*
 * cdma_build_sg_table - Convert a DMA-mapped sg_table into a CDMA HW S/G table.
 *
 * Allocates a CDMA_SGT_SIZE block from the DMA pool, fills it with SGE entries
 * matching the scatterlist, and returns the DMA address for use in BD descriptors.
 *
 * Returns 0 on success, negative errno on failure.
 */
int cdma_build_sg_table(struct cdma_device *cdev, struct sg_table *sgt,
			dma_addr_t *sgt_dma_out, void **sgt_cpu_out)
{
	struct cdma_sge *sge_table;
	struct scatterlist *sg;
	dma_addr_t sgt_dma;
	unsigned int i;
	u32 cfg;

	if (sgt->nents > CDMA_SG_MAX_ENTRIES)
		return -EINVAL;

	sge_table = dma_pool_zalloc(cdev->sgt_pool, GFP_KERNEL, &sgt_dma);
	if (!sge_table)
		return -ENOMEM;

	for_each_sgtable_dma_sg(sgt, sg, i) {
		cfg = (CDMA_TYPE_DATA << CDMA_DESC_TYPE_SHIFT) |
		      (CDMA_DESC_TSIZE_DEFAULT << CDMA_DESC_TSIZE_SHIFT);

		/* Set F=1 on the last entry */
		if (i == sgt->nents - 1)
			cfg |= CDMA_DESC_FINAL;

		sge_table[i].addr   = cpu_to_le64(sg_dma_address(sg));
		sge_table[i].length = cpu_to_le32(sg_dma_len(sg));
		sge_table[i].config = cpu_to_le32(cfg);
	}

	/* Ensure SGE writes are visible to the DMA device before returning */
	dma_wmb();

	*sgt_dma_out = sgt_dma;
	*sgt_cpu_out = sge_table;
	return 0;
}

/*
 * cdma_free_sg_table - Return a HW S/G table to the DMA pool.
 */
void cdma_free_sg_table(struct cdma_device *cdev, void *sgt_cpu,
			dma_addr_t sgt_dma)
{
	if (sgt_cpu)
		dma_pool_free(cdev->sgt_pool, sgt_cpu, sgt_dma);
}

/*
 * cdma_build_sg_pipe_sender - Build sender BD pair (SD[SG_FULL] + DD[PIPE]).
 *
 * The source data is described by a CDMA HW S/G table; the DD carries
 * the SND_PIPE_ID_VALUE slots and SND_TAG so the hardware pipe matcher
 * can find a receiver authorized for the same pipe ID.
 *
 * Writes 2 BD entries (32 bytes) at the current producer index. The SD
 * has WB=1 so the IRQ handler can detect completion by reading the
 * inverted W bit at ring->wb_idx (set by cdma_ring_submit). Submit with
 * num_bds = CDMA_BD_PAIR after calling.
 *
 * Per spec, all four SND_PIPE_ID_VALUE_n slots must hold legal PIDaAUR
 * values, regardless of NPR ("fields not used should specify legal
 * values as defined in PIDaAUR"). For NPR=0 (one receiver), use the
 * same pipe id in all four slots.
 */
void cdma_build_sg_pipe_sender(struct cdma_ring *ring,
			       dma_addr_t src_sgt, u32 src_len,
			       u8 pipe_id, u32 tag)
{
	void *base = ring->bd_base + ring->pir * CDMA_BD_ENTRY_SIZE;
	u32 sd_cfg, dd_cfg;
	u64 dd_addr;

	memset(base, 0, 2 * CDMA_BD_ENTRY_SIZE);

	/*
	 * SD: TYPE=SG_FULL(0x2), F=1, CI=1, W=1 (writeback)
	 * TSIZE is reserved for TYPE=bxx10 — must be 0 per spec.
	 */
	sd_cfg = (CDMA_TYPE_SG_FULL << CDMA_DESC_TYPE_SHIFT) |
		 CDMA_DESC_FINAL | CDMA_DESC_CI | CDMA_DESC_WB;
	cdma_write_sd(base, src_sgt, src_len, sd_cfg);

	/*
	 * DD: TYPE=PIPE(3), F=1, NPR=0 (one receiver)
	 *   addr[7:0]   = SND_PIPE_ID_VALUE_0
	 *   addr[15:8]  = SND_PIPE_ID_VALUE_1
	 *   addr[23:16] = SND_PIPE_ID_VALUE_2
	 *   addr[31:24] = SND_PIPE_ID_VALUE_3
	 *   addr[63:32] = SND_TAG
	 */
	dd_addr = ((u64)tag << 32) |
		  ((u32)pipe_id << 24) | ((u32)pipe_id << 16) |
		  ((u32)pipe_id << 8)  | pipe_id;
	dd_cfg = (CDMA_TYPE_PIPE << CDMA_DESC_TYPE_SHIFT) |
		 CDMA_DESC_FINAL |
		 (0 << CDMA_DD_NPR_SHIFT);	/* NPR=0 — 1 receiver */
	cdma_write_dd(base + CDMA_BD_ENTRY_SIZE, dd_addr, 0, dd_cfg);
}

/*
 * cdma_build_sg_pipe_receiver - Build receiver BD pair (SD[PIPE] + DD[SG_FULL]).
 *
 * SD: TYPE=PIPE(0x3), F=1, CI=1, W=1 — addr[7:0] = RCV_PIPE_ID_VALUE
 * DD: TYPE=SG_FULL(0x2), F=1 — points to destination S/G table
 *
 * Writes 2 BD entries at the current producer index. SD has WB=1 for
 * IRQ-driven completion detection (length and rcv_tag are written back
 * by hardware on match). Submit with num_bds = CDMA_BD_PAIR.
 */
void cdma_build_sg_pipe_receiver(struct cdma_ring *ring,
				 dma_addr_t dst_sgt, u32 dst_len,
				 u8 pipe_id)
{
	void *base = ring->bd_base + ring->pir * CDMA_BD_ENTRY_SIZE;
	u32 sd_cfg, dd_cfg;

	memset(base, 0, 2 * CDMA_BD_ENTRY_SIZE);

	/* SD: TYPE=PIPE(3), F=1, CI=1, W=1 — addr[7:0] = RCV_PIPE_ID_VALUE */
	sd_cfg = (CDMA_TYPE_PIPE << CDMA_DESC_TYPE_SHIFT) |
		 CDMA_DESC_FINAL | CDMA_DESC_CI | CDMA_DESC_WB;
	cdma_write_sd(base, (u64)pipe_id, 0, sd_cfg);

	/* DD: TYPE=SG_FULL(0x2), F=1 — TSIZE reserved for bxx10 */
	dd_cfg = (CDMA_TYPE_SG_FULL << CDMA_DESC_TYPE_SHIFT) |
		 CDMA_DESC_FINAL;
	cdma_write_dd(base + CDMA_BD_ENTRY_SIZE, dst_sgt, dst_len, dd_cfg);
}
