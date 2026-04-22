// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * NXP S32N79 CDMA — Char device layer for pipe I/O
 *
 * Reader calls read() first, which pins user pages and blocks.
 * Writer calls write(), pins its pages, builds S/G descriptors, submits
 * a DMA S/G memcpy from TX user pages to RX user pages, then wakes
 * the reader. If no reader is waiting, write() returns -EAGAIN.
 *
 * /dev/cdma-pipeN-tx — write-only (sender)
 * /dev/cdma-pipeN-rx — read-only, exclusive open (receiver, blocks
 *                                                 until data arrives)
 *
 * Locking and lifecycle:
 *   - pipe->lock guards rx_zc, dying, rx_open, inflight.
 *   - Reader drops the lock while sleeping; writer takes the lock to do
 *     the DMA transfer and signal the reader.
 *   - cdma_pipe_enter()/cdma_pipe_leave() bracket every read/write so the
 *     destroy path can wait for in-flight ops to drain before tearing
 *     down DMA pool / ring memory.
 *   - Writer signals reader unconditionally (success or error) via
 *     rx_zc.result so a transfer failure can't strand the reader.
 *   - DMA_FROM_DEVICE pages are unpinned via unpin_user_pages_dirty_lock()
 *     so file-backed mappings see the device's writes on writeback.
 *
 * Copyright 2026 NXP
 */

#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include "cdma.h"

static DEFINE_IDA(cdma_pipe_ida);

static struct cdma_pipe *miscdev_to_pipe(struct miscdevice *misc, bool is_tx)
{
	if (is_tx)
		return container_of(misc, struct cdma_pipe, miscdev_tx);

	return container_of(misc, struct cdma_pipe, miscdev_rx);
}

/*
 * Track in-flight read()/write() callers so cdma_pipe_destroy() can wait
 * for them to leave before freeing DMA pool / ring memory. Returns
 * -ENODEV if the pipe is being torn down.
 */
static int cdma_pipe_enter(struct cdma_pipe *pipe)
{
	mutex_lock(&pipe->lock);
	if (pipe->dying) {
		mutex_unlock(&pipe->lock);
		return -ENODEV;
	}
	pipe->inflight++;
	mutex_unlock(&pipe->lock);
	return 0;
}

static void cdma_pipe_leave(struct cdma_pipe *pipe)
{
	mutex_lock(&pipe->lock);
	pipe->inflight--;
	if (!pipe->inflight && pipe->dying)
		wake_up(&pipe->drain_wq);
	mutex_unlock(&pipe->lock);
}

/* Wait for ring completion via MSI-X IRQ */
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
 * cdma_sg_transfer - DMA S/G memcpy from TX S/G table to RX S/G table.
 *
 * Uses SD[SG_FULL] + DD[SG_FULL] pointing to hardware S/G tables built
 * from pinned user pages.
 */
static int cdma_sg_transfer(struct cdma_pipe *pipe,
			    dma_addr_t src_sgt, u32 src_len,
			    dma_addr_t dst_sgt, u32 dst_len)
{
	struct device *dev = &pipe->cdev->pdev->dev;
	struct cdma_ring *ring = pipe->ring;
	int ret;

	cdma_build_sg_memcpy(ring, src_sgt, src_len, dst_sgt, dst_len);

	ret = cdma_ring_submit(ring, CDMA_MEMCPY_NUM_BDS);
	if (ret) {
		dev_err(dev, "pipe%u: S/G submit failed: %d\n",
			pipe->pipe_id, ret);
		return ret;
	}

	ret = cdma_wait_ring(ring);
	if (ret) {
		dev_err(dev, "pipe%u: S/G completion error: %d\n",
			pipe->pipe_id, ret);
		/* Reset and re-enable ring after error */
		cdma_ring_reset(ring);
		cdma_ring_init(ring);
		return ret;
	}

	return 0;
}

/*
 * Pin user pages, DMA-map them, and build a CDMA HW S/G table. Returns 0
 * on success; caller must clean up via cdma_zc_unmap() on success.
 */
static int cdma_zc_map(struct cdma_device *cdev, unsigned long uaddr,
		       size_t len, enum dma_data_direction dir,
		       struct cdma_zc_state *zc)
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
	zc->result = 0;
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
static void cdma_zc_unmap(struct cdma_device *cdev,
			  struct cdma_zc_state *zc,
			  enum dma_data_direction dir)
{
	struct device *dev = &cdev->pdev->dev;

	if (!zc->pages)
		return;

	cdma_free_sg_table(cdev, zc->sgt_cpu, zc->sgt_dma);
	dma_unmap_sgtable(dev, &zc->sgt, dir, 0);
	sg_free_table(&zc->sgt);

	if (dir == DMA_FROM_DEVICE)
		unpin_user_pages_dirty_lock(zc->pages, zc->nr_pages, true);
	else
		unpin_user_pages(zc->pages, zc->nr_pages);

	kfree(zc->pages);
	zc->pages = NULL;
	zc->nr_pages = 0;
}

/* TX file operations (write-only, /dev/cdma-pipeN-tx) */
static int cdma_tx_open(struct inode *inode, struct file *filp)
{
	struct cdma_pipe *pipe = miscdev_to_pipe(filp->private_data, true);

	filp->private_data = pipe;
	return 0;
}

static ssize_t cdma_tx_write(struct file *filp, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	struct cdma_pipe *pipe = filp->private_data;
	struct cdma_device *cdev = pipe->cdev;
	struct cdma_zc_state tx_zc = {};
	size_t xfer_len;
	ssize_t ret;

	if (!count)
		return 0;

	if (count > pipe->max_xfer) {
		dev_warn_ratelimited(&cdev->pdev->dev,
				     "pipe%u: write %zu > max_xfer %zu — chunk the write\n",
				     pipe->pipe_id, count, pipe->max_xfer);
		return -EMSGSIZE;
	}

	if (filp->f_flags & O_NONBLOCK)
		return -EAGAIN;

	ret = cdma_pipe_enter(pipe);
	if (ret)
		return ret;

	mutex_lock(&pipe->lock);

	if (pipe->dying) {
		ret = -ENODEV;
		goto out;
	}

	/* Require-reader-first: writer returns -EAGAIN if no reader waiting */
	if (!pipe->rx_zc.waiting) {
		ret = -EAGAIN;
		goto out;
	}

	xfer_len = min(count, pipe->rx_zc.len);

	/*
	 * From here we own the reader: any return must release it via
	 * rx_zc.result + waiting=false + wake. Skipping the release would
	 * strand the reader and leak its DMA mapping until a signal.
	 */
	ret = cdma_zc_map(cdev, (unsigned long)ubuf, xfer_len, DMA_TO_DEVICE,
			  &tx_zc);
	if (!ret) {
		ret = cdma_sg_transfer(pipe, tx_zc.sgt_dma, xfer_len,
				       pipe->rx_zc.sgt_dma, xfer_len);
		cdma_zc_unmap(cdev, &tx_zc, DMA_TO_DEVICE);
	}

	pipe->rx_zc.result = ret;
	if (!ret)
		pipe->rx_zc.transferred = xfer_len;
	pipe->rx_zc.waiting = false;
	wake_up_interruptible(&pipe->rx_wq);

	if (ret)
		goto out;

	ret = xfer_len;
out:
	mutex_unlock(&pipe->lock);
	cdma_pipe_leave(pipe);
	return ret;
}

static __poll_t cdma_tx_poll(struct file *filp,
			     struct poll_table_struct *wait)
{
	struct cdma_pipe *pipe = filp->private_data;
	__poll_t mask = 0;

	poll_wait(filp, &pipe->tx_wq, wait);

	mutex_lock(&pipe->lock);
	if (pipe->dying)
		mask |= EPOLLHUP;
	else if (pipe->rx_zc.waiting)
		mask |= EPOLLOUT | EPOLLWRNORM;
	mutex_unlock(&pipe->lock);

	return mask;
}

static const struct file_operations cdma_tx_fops = {
	.owner		= THIS_MODULE,
	.open		= cdma_tx_open,
	.write		= cdma_tx_write,
	.poll		= cdma_tx_poll,
};

/*
 * RX file operations (read-only, /dev/cdma-pipeN-rx).
 * Exclusive open: only one fd may have the RX device open at a time.
 */
static int cdma_rx_open(struct inode *inode, struct file *filp)
{
	struct cdma_pipe *pipe = miscdev_to_pipe(filp->private_data, false);

	mutex_lock(&pipe->lock);
	if (pipe->dying) {
		mutex_unlock(&pipe->lock);
		return -ENODEV;
	}
	if (pipe->rx_open) {
		mutex_unlock(&pipe->lock);
		return -EBUSY;
	}
	pipe->rx_open = true;
	mutex_unlock(&pipe->lock);

	filp->private_data = pipe;
	return 0;
}

static int cdma_rx_release(struct inode *inode, struct file *filp)
{
	struct cdma_pipe *pipe = filp->private_data;

	mutex_lock(&pipe->lock);
	pipe->rx_open = false;
	mutex_unlock(&pipe->lock);
	return 0;
}

static ssize_t cdma_rx_read(struct file *filp, char __user *ubuf,
			    size_t count, loff_t *ppos)
{
	struct cdma_pipe *pipe = filp->private_data;
	struct cdma_device *cdev = pipe->cdev;
	ssize_t ret;

	if (!count)
		return 0;

	if (count > pipe->max_xfer) {
		dev_warn_ratelimited(&cdev->pdev->dev,
				     "pipe%u: read %zu > max_xfer %zu — chunk the read\n",
				     pipe->pipe_id, count, pipe->max_xfer);
		return -EMSGSIZE;
	}

	if (filp->f_flags & O_NONBLOCK)
		return -EAGAIN;

	ret = cdma_pipe_enter(pipe);
	if (ret)
		return ret;

	mutex_lock(&pipe->lock);

	if (pipe->dying) {
		ret = -ENODEV;
		goto out;
	}

	/* Exclusive open guarantees only one in-flight reader */
	if (pipe->rx_zc.waiting) {
		ret = -EBUSY;
		goto out;
	}

	/* Pin RX user pages and build S/G table */
	ret = cdma_zc_map(cdev, (unsigned long)ubuf, count, DMA_FROM_DEVICE,
			  &pipe->rx_zc);
	if (ret)
		goto out;

	pipe->rx_zc.len = count;
	pipe->rx_zc.transferred = 0;
	pipe->rx_zc.result = 0;
	pipe->rx_zc.waiting = true;

	/* Wake any writer waiting for a reader */
	wake_up_interruptible(&pipe->tx_wq);

	/* Drop lock while waiting — writer needs it for the transfer */
	mutex_unlock(&pipe->lock);

	/* Block until writer completes the transfer or device dies */
	ret = wait_event_interruptible(pipe->rx_wq, !pipe->rx_zc.waiting);

	mutex_lock(&pipe->lock);

	if (pipe->rx_zc.waiting) {
		/*
		 * Signal arrived before any writer claimed the slot.
		 * Safe to clean up under the lock.
		 */
		pipe->rx_zc.waiting = false;
		cdma_zc_unmap(cdev, &pipe->rx_zc, DMA_FROM_DEVICE);
		goto out;
	}

	/*
	 * Writer ran (success or error). Always unmap, then propagate the
	 * result: writer-side errors travel back via rx_zc.result.
	 */
	cdma_zc_unmap(cdev, &pipe->rx_zc, DMA_FROM_DEVICE);

	if (pipe->rx_zc.result)
		ret = pipe->rx_zc.result;
	else
		ret = pipe->rx_zc.transferred;

	pipe->rx_zc.transferred = 0;
	pipe->rx_zc.result = 0;
out:
	mutex_unlock(&pipe->lock);
	cdma_pipe_leave(pipe);
	return ret;
}

static const struct file_operations cdma_rx_fops = {
	.owner		= THIS_MODULE,
	.open		= cdma_rx_open,
	.release	= cdma_rx_release,
	.read		= cdma_rx_read,
};

/* Pipe allocation and registration */
static int cdma_pipe_create(struct cdma_device *cdev, struct cdma_si *si,
			    int pipe_idx)
{
	struct device *dev = &cdev->pdev->dev;
	struct cdma_pipe *pipe;
	int ret;

	if (si->num_rings < 1) {
		dev_warn(dev, "SI%u: need at least 1 BD ring (have %u)\n",
			 si->si_idx, si->num_rings);
		return -EINVAL;
	}

	pipe = devm_kzalloc(dev, sizeof(*pipe), GFP_KERNEL);
	if (!pipe)
		return -ENOMEM;

	pipe->cdev = cdev;
	pipe->pipe_id = pipe_idx;
	pipe->max_xfer = CDMA_PIPE_BUF_SIZE;
	mutex_init(&pipe->lock);
	init_waitqueue_head(&pipe->rx_wq);
	init_waitqueue_head(&pipe->tx_wq);
	init_waitqueue_head(&pipe->drain_wq);

	/* Use ring 0 for transfers */
	pipe->ring = &si->rings[0];
	pipe->ring->pipe = pipe;

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
		 "registered /dev/%s + /dev/%s (pipe_id=%u, SI%u ring 0, S/G zero-copy)\n",
		 pipe->name_tx, pipe->name_rx, pipe->pipe_id, si->si_idx);
	return 0;
}

/*
 * Tear down a pipe. Must run before any DMA pool / ring / SI cleanup so
 * in-flight read()/write() callers don't access freed memory.
 *
 *   1. Set dying under lock so new ops bail with -ENODEV.
 *   2. Release any waiting reader with -ENODEV.
 *   3. Wake all waiters so they observe the new state.
 *   4. Wait for in-flight read/write callers to drain.
 *   5. Unregister the misc devices.
 */
static void cdma_pipe_destroy(struct cdma_device *cdev)
{
	struct cdma_pipe *pipe = cdev->pipes;

	if (!pipe)
		return;

	mutex_lock(&pipe->lock);
	pipe->dying = true;
	if (pipe->rx_zc.waiting) {
		pipe->rx_zc.result = -ENODEV;
		pipe->rx_zc.waiting = false;
	}
	mutex_unlock(&pipe->lock);

	wake_up_interruptible_all(&pipe->rx_wq);
	wake_up_interruptible_all(&pipe->tx_wq);

	wait_event(pipe->drain_wq, !READ_ONCE(pipe->inflight));

	misc_deregister(&pipe->miscdev_rx);
	misc_deregister(&pipe->miscdev_tx);

	cdev->pipes = NULL;
	cdev->num_pipes = 0;
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
