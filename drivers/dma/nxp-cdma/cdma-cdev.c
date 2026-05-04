// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * NXP S32N79 CDMA — Char device layer for cross-SI pipe I/O
 *
 * Each SI provides both TX (ring 0) and RX (ring 1) using TYPE=PIPE
 * descriptors. TX and RX operate fully independently — no shared
 * state, no reader-first coordination. Cross-SI rendezvous is handled
 * by the hardware pipe matcher.
 *
 * /dev/cdma-pipeN-tx — write-only sender (ring 0)
 * /dev/cdma-pipeN-rx — read-only  receiver (ring 1)
 *
 * The same pipe ID must be authorized in IERB for both endpoints.
 *
 * Locking:
 *   - pipe->tx_lock guards TX ring submission
 *   - pipe->rx_lock guards RX ring submission
 *   - The two locks are independent and never need to be held together
 *
 * Each ring is enabled once at probe (cdma_ring_init) and stays enabled
 * across transfers; cdma_ring_submit just advances PIR. On signal
 * interrupt or hardware error, cdma_ring_cancel disables the ring,
 * waits for BUSY to clear, clears errors, and re-enables — leaving the
 * ring in a clean state for the next submission.
 *
 * Copyright 2026 NXP
 */

#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include "cdma.h"

static DEFINE_IDA(cdma_pipe_ida);

/*
 * Unwind state for a single pin/map/build cycle. Lives on the caller's
 * stack; cdma_zc_map() fills it on success, cdma_zc_unmap() consumes it.
 */
struct cdma_zc {
	struct page **pages;
	unsigned int nr_pages;
	struct sg_table sgt;
	dma_addr_t sgt_dma;	/* DMA addr of HW S/G table */
	void *sgt_cpu;		/* CPU addr of HW S/G table */
};

static struct cdma_pipe *miscdev_to_pipe(struct miscdevice *misc, bool is_tx)
{
	if (is_tx)
		return container_of(misc, struct cdma_pipe, miscdev_tx);

	return container_of(misc, struct cdma_pipe, miscdev_rx);
}

/*
 * Wait for ring completion via MSI-X IRQ. Bounded by a kernel-side
 * timeout as a safety net — a stuck IRQ (e.g. a misconfigured MSI)
 * shouldn't hang the syscall forever. The hardware CPTCR timer fires
 * a programming error well before this if the remote peer never
 * matches.
 */
static int cdma_wait_ring(struct cdma_ring *ring)
{
	unsigned long tmo = usecs_to_jiffies(CDMA_COMP_TIMEOUT_US);
	int ret;

	ret = wait_for_completion_interruptible_timeout(&ring->done, tmo);
	if (!ret)
		return -ETIMEDOUT;
	if (ret < 0)
		return ret;
	return ring->result;
}

/*
 * Pin user pages, DMA-map them, and build a CDMA HW S/G table. On
 * success @zc carries the unwind state for cdma_zc_unmap(); on error
 * nothing is left allocated and @zc is untouched.
 */
static int cdma_zc_map(struct cdma_device *cdev, unsigned long uaddr,
		       size_t len, enum dma_data_direction dir,
		       struct cdma_zc *zc)
{
	struct device *dev = &cdev->pdev->dev;
	unsigned int nr_pages, offset;
	struct page **pages;
	int ret, pinned;

	offset = offset_in_page(uaddr);
	nr_pages = DIV_ROUND_UP(len + offset, PAGE_SIZE);

	if (nr_pages > CDMA_SG_MAX_ENTRIES)
		return -EINVAL;

	pages = kmalloc_array(nr_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	pinned = pin_user_pages_fast(uaddr, nr_pages,
				     (dir == DMA_FROM_DEVICE) ? FOLL_WRITE : 0,
				     pages);
	if (pinned < 0) {
		ret = pinned;
		goto err_free_pages;
	}
	if (pinned != nr_pages) {
		unpin_user_pages(pages, pinned);
		ret = -EFAULT;
		goto err_free_pages;
	}

	ret = sg_alloc_table_from_pages_segment(&zc->sgt, pages, nr_pages,
						offset, len,
						dma_max_mapping_size(dev),
						GFP_KERNEL);
	if (ret)
		goto err_unpin;

	ret = dma_map_sgtable(dev, &zc->sgt, dir, 0);
	if (ret)
		goto err_free_sgt;

	ret = cdma_build_sg_table(cdev, &zc->sgt, &zc->sgt_dma, &zc->sgt_cpu);
	if (ret)
		goto err_unmap;

	zc->pages = pages;
	zc->nr_pages = nr_pages;
	return 0;

err_unmap:
	dma_unmap_sgtable(dev, &zc->sgt, dir, 0);
err_free_sgt:
	sg_free_table(&zc->sgt);
err_unpin:
	unpin_user_pages(pages, nr_pages);
err_free_pages:
	kfree(pages);
	return ret;
}

/*
 * Undo cdma_zc_map(). For DMA_FROM_DEVICE the device wrote the user
 * pages, so they must be marked dirty before unpinning to ensure
 * file-backed mappings see the data on writeback.
 */
static void cdma_zc_unmap(struct cdma_device *cdev, struct cdma_zc *zc,
			  enum dma_data_direction dir)
{
	struct device *dev = &cdev->pdev->dev;

	cdma_free_sg_table(cdev, zc->sgt_cpu, zc->sgt_dma);
	dma_unmap_sgtable(dev, &zc->sgt, dir, 0);
	sg_free_table(&zc->sgt);

	if (dir == DMA_FROM_DEVICE)
		unpin_user_pages_dirty_lock(zc->pages, zc->nr_pages, true);
	else
		unpin_user_pages(zc->pages, zc->nr_pages);

	kfree(zc->pages);
}

/* TX file operations (write-only, /dev/cdma-pipeN-tx) */
static int cdma_tx_open(struct inode *inode, struct file *filp)
{
	struct cdma_pipe *pipe = miscdev_to_pipe(filp->private_data, true);

	filp->private_data = pipe;
	return 0;
}

/*
 * Synchronous TX write — pins pages, builds sender BD pair at the
 * ring's current PIR, submits to tx_ring, blocks until hardware
 * completes (a remote receiver matches via the IERB-configured pipe
 * id). Fully independent from RX.
 */
static ssize_t cdma_tx_write(struct file *filp, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	struct cdma_pipe *pipe = filp->private_data;
	struct cdma_device *cdev = pipe->cdev;
	struct cdma_zc zc = {};
	int ret;

	if (!count)
		return 0;

	if (count > pipe->max_xfer) {
		dev_warn_ratelimited(&cdev->pdev->dev,
				     "pipe%u: write %zu > max_xfer %zu — chunk the write\n",
				     pipe->pipe_idx, count, pipe->max_xfer);
		return -EMSGSIZE;
	}

	if (filp->f_flags & O_NONBLOCK)
		return -EAGAIN;

	mutex_lock(&pipe->tx_lock);

	ret = cdma_zc_map(cdev, (unsigned long)ubuf, count, DMA_TO_DEVICE,
			  &zc);
	if (ret) {
		mutex_unlock(&pipe->tx_lock);
		return ret;
	}

	cdma_build_sg_pipe_sender(pipe->tx_ring, zc.sgt_dma, count,
				  pipe->hw_pipe_id, 0);
	ret = cdma_ring_submit(pipe->tx_ring, CDMA_BD_PAIR);

	mutex_unlock(&pipe->tx_lock);

	if (!ret)
		ret = cdma_wait_ring(pipe->tx_ring);

	/*
	 * On signal interrupt or hardware error, cancel the ring before
	 * unmapping — the hardware may still hold a reference to the BD
	 * ring memory or the user pages. cdma_ring_cancel re-enables the
	 * ring, so subsequent submissions work normally.
	 */
	if (ret < 0)
		cdma_ring_cancel(pipe->tx_ring);

	cdma_zc_unmap(cdev, &zc, DMA_TO_DEVICE);

	return ret ? ret : (ssize_t)count;
}

static const struct file_operations cdma_tx_fops = {
	.owner	= THIS_MODULE,
	.open	= cdma_tx_open,
	.write	= cdma_tx_write,
};

/* RX file operations (read-only, /dev/cdma-pipeN-rx) */
static int cdma_rx_open(struct inode *inode, struct file *filp)
{
	struct cdma_pipe *pipe = miscdev_to_pipe(filp->private_data, false);

	filp->private_data = pipe;
	return 0;
}

/*
 * Synchronous RX read — pins pages, builds receiver BD pair at the
 * ring's current PIR, submits to rx_ring, blocks until hardware
 * completes (a remote sender matches via the IERB-configured pipe id).
 * Fully independent from TX.
 */
static ssize_t cdma_rx_read(struct file *filp, char __user *ubuf,
			    size_t count, loff_t *ppos)
{
	struct cdma_pipe *pipe = filp->private_data;
	struct cdma_device *cdev = pipe->cdev;
	struct cdma_zc zc = {};
	int ret;

	if (!count)
		return 0;

	if (count > pipe->max_xfer) {
		dev_warn_ratelimited(&cdev->pdev->dev,
				     "pipe%u: read %zu > max_xfer %zu — chunk the read\n",
				     pipe->pipe_idx, count, pipe->max_xfer);
		return -EMSGSIZE;
	}

	if (filp->f_flags & O_NONBLOCK)
		return -EAGAIN;

	mutex_lock(&pipe->rx_lock);

	ret = cdma_zc_map(cdev, (unsigned long)ubuf, count, DMA_FROM_DEVICE,
			  &zc);
	if (ret) {
		mutex_unlock(&pipe->rx_lock);
		return ret;
	}

	cdma_build_sg_pipe_receiver(pipe->rx_ring, zc.sgt_dma, count,
				    pipe->hw_pipe_id);
	ret = cdma_ring_submit(pipe->rx_ring, CDMA_BD_PAIR);

	mutex_unlock(&pipe->rx_lock);

	if (!ret)
		ret = cdma_wait_ring(pipe->rx_ring);

	if (ret < 0)
		cdma_ring_cancel(pipe->rx_ring);

	cdma_zc_unmap(cdev, &zc, DMA_FROM_DEVICE);

	return ret ? ret : (ssize_t)count;
}

static const struct file_operations cdma_rx_fops = {
	.owner	= THIS_MODULE,
	.open	= cdma_rx_open,
	.read	= cdma_rx_read,
};

/* Pipe allocation and registration */
static void cdma_pipe_destroy(struct cdma_device *cdev)
{
	struct cdma_pipe *pipe = cdev->pipes;

	if (!pipe)
		return;

	if (pipe->tx_ring)
		pipe->tx_ring->pipe = NULL;
	if (pipe->rx_ring)
		pipe->rx_ring->pipe = NULL;

	misc_deregister(&pipe->miscdev_rx);
	misc_deregister(&pipe->miscdev_tx);

	cdev->pipes = NULL;
	cdev->num_pipes = 0;
}

/*
 * Create a single-SI pipe with TX (ring 0) and RX (ring 1) char devices.
 * Caller must ensure the SI has at least 2 BD rings.
 */
static int cdma_pipe_create(struct cdma_device *cdev, struct cdma_si *si,
			    int pipe_idx)
{
	struct device *dev = &cdev->pdev->dev;
	struct cdma_pipe *pipe;
	int ret;

	pipe = devm_kzalloc(dev, sizeof(*pipe), GFP_KERNEL);
	if (!pipe)
		return -ENOMEM;

	pipe->cdev = cdev;
	pipe->hw_pipe_id = cdev->hw_pipe_id;
	pipe->pipe_idx = pipe_idx;
	pipe->max_xfer = CDMA_PIPE_BUF_SIZE;
	mutex_init(&pipe->tx_lock);
	mutex_init(&pipe->rx_lock);

	/* Ring 0 = TX (sender), Ring 1 = RX (receiver) */
	pipe->tx_ring = &si->rings[0];
	pipe->rx_ring = &si->rings[1];
	pipe->tx_ring->pipe = pipe;
	pipe->rx_ring->pipe = pipe;

	/* Register TX char device: /dev/cdma-pipeN-tx */
	snprintf(pipe->name_tx, sizeof(pipe->name_tx),
		 "cdma-pipe%d-tx", pipe_idx);
	pipe->miscdev_tx.minor = MISC_DYNAMIC_MINOR;
	pipe->miscdev_tx.name  = pipe->name_tx;
	pipe->miscdev_tx.fops  = &cdma_tx_fops;

	ret = misc_register(&pipe->miscdev_tx);
	if (ret) {
		dev_err(dev, "failed to register %s: %d\n",
			pipe->name_tx, ret);
		return ret;
	}

	/* Register RX char device: /dev/cdma-pipeN-rx */
	snprintf(pipe->name_rx, sizeof(pipe->name_rx),
		 "cdma-pipe%d-rx", pipe_idx);
	pipe->miscdev_rx.minor = MISC_DYNAMIC_MINOR;
	pipe->miscdev_rx.name  = pipe->name_rx;
	pipe->miscdev_rx.fops  = &cdma_rx_fops;

	ret = misc_register(&pipe->miscdev_rx);
	if (ret) {
		dev_err(dev, "failed to register %s: %d\n",
			pipe->name_rx, ret);
		misc_deregister(&pipe->miscdev_tx);
		return ret;
	}

	cdev->pipes = pipe;
	cdev->num_pipes = 1;

	dev_info(dev,
		 "registered /dev/%s + /dev/%s (hw_pipe_id=%u, ring0=TX, ring1=RX)\n",
		 pipe->name_tx, pipe->name_rx, pipe->hw_pipe_id);
	return 0;
}

static void cdma_ida_free(void *data)
{
	ida_free(&cdma_pipe_ida, (unsigned long)data);
}

int cdma_cdev_init(struct cdma_device *cdev)
{
	struct device *dev = &cdev->pdev->dev;
	int idx, ret;

	if (!cdev->si)
		return -ENODEV;

	idx = ida_alloc(&cdma_pipe_ida, GFP_KERNEL);
	if (idx < 0)
		return idx;

	ret = devm_add_action_or_reset(dev, cdma_ida_free,
				       (void *)(unsigned long)idx);
	if (ret)
		return ret;

	return cdma_pipe_create(cdev, cdev->si, idx);
}

void cdma_cdev_fini(struct cdma_device *cdev)
{
	cdma_pipe_destroy(cdev);
}
