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

#include <linux/aio.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uio.h>
#include "cdma.h"

static DEFINE_IDA(cdma_pipe_ida);

static struct cdma_pipe *miscdev_to_pipe(struct miscdevice *misc, bool is_tx)
{
	if (is_tx)
		return container_of(misc, struct cdma_pipe, miscdev_tx);

	return container_of(misc, struct cdma_pipe, miscdev_rx);
}

/*
 * Extract a user-space virtual address from an iov_iter. The cdev only
 * supports user-backed iters covering a single contiguous range
 * (ITER_UBUF or a single-segment ITER_IOVEC). Returns 0 if the iter
 * shape isn't supported.
 */
static unsigned long cdma_iter_uaddr(struct iov_iter *iter)
{
	if (!user_backed_iter(iter))
		return 0;

	if (iter_is_iovec(iter) && iter->nr_segs != 1)
		return 0;

	return (unsigned long)iter_iov_addr(iter);
}

/*
 * Wait for ring completion via MSI-X IRQ. Bounded by a kernel-side
 * timeout - a stuck IRQ (e.g. a misconfigured MSI) shouldn't hang the
 * syscall forever.
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

/*
 * Async deferred completion. Called from a workqueue after the IRQ
 * handler decides an async request is done. Owns the lifecycle of
 * @req: unmaps the pinned user buffer, signals the kiocb, and frees
 * @req.
 *
 * ki_complete() must NOT be called under the pipe lock: completion
 * handlers in io_uring may resubmit on the same fd, which would
 * deadlock on the same lock if called inline.
 */
static void cdma_async_complete_work(struct work_struct *work)
{
	struct cdma_async_req *req = container_of(work, struct cdma_async_req,
						  complete_work);
	struct cdma_ring *ring = req->ring;
	struct cdma_device *cdev = ring->si->cdev;
	struct cdma_pipe *pipe = ring->pipe;
	enum dma_data_direction dir = req->dir;
	struct kiocb *iocb = req->iocb;
	int result = req->result;
	size_t len = req->len;

	cdma_zc_unmap(cdev, &req->zc, dir);

	/*
	 * Wake any epoll/poll waiters before signalling the kiocb so a
	 * userspace event loop that submits on completion notification
	 * sees the slot already free.
	 */
	if (pipe) {
		if (dir == DMA_TO_DEVICE)
			wake_up_interruptible(&pipe->tx_wq);
		else
			wake_up_interruptible(&pipe->rx_wq);
	}

	iocb->ki_complete(iocb, result ? result : (long)len);
	kfree(req);
}

/*
 * Helper exposed to cdma-pci.c IRQ thread so the handler can defer
 * async completion without needing the full layout of
 * struct cdma_async_req.
 *
 * Uses the device-private workqueue so cdma_pci_remove() can flush every
 * outstanding completion before the BD rings and DMA pool are freed.
 */
void cdma_async_req_complete(struct cdma_async_req *req, int result)
{
	req->result = result;
	queue_work(req->ring->si->cdev->wq, &req->complete_work);
}

/* TX file operations (write-only, /dev/cdma-pipeN-tx) */
static int cdma_tx_open(struct inode *inode, struct file *filp)
{
	struct cdma_pipe *pipe = miscdev_to_pipe(filp->private_data, true);
	struct cdma_device *cdev = pipe->cdev;
	int ret;

	mutex_lock(&cdev->config_lock);
	if (!cdev->pipe_active || cdev->pipes != pipe) {
		ret = -ENODEV;
	} else {
		filp->private_data = pipe;
		/* Deliberately do NOT advertise FMODE_NOWAIT. The async
		 * submit path (cdma_tx_write_async) must run in a context
		 * where it can block: it does kzalloc(GFP_KERNEL), builds
		 * an S/G table and pin_user_pages_fast()s the user buffer,
		 * none of which honour IOCB_NOWAIT. It therefore bounces a
		 * nowait submission back with -EAGAIN.
		 * Without FMODE_NOWAIT io_uring never sets IOCB_NOWAIT on
		 * these fds, so the submit always runs from a blocking-capable
		 * context and both io_uring/epoll async and plain read()/
		 * write() sync I/O work.
		 */
		ret = stream_open(inode, filp);
	}
	mutex_unlock(&cdev->config_lock);

	return ret;
}

/*
 * Synchronous TX write — pins pages, builds sender BD pair at the
 * ring's current PIR, submits to tx_ring, blocks until hardware
 * completes (a remote receiver matches via the IERB-configured pipe
 * id). Fully independent from RX.
 *
 * Holds pipe->tx_lock across submit + wait so an async submitter
 * (which also takes the lock) cannot interleave and confuse the
 * IRQ-handler dispatch (async_req vs ring->done).
 */
static ssize_t cdma_tx_write_sync(struct cdma_pipe *pipe, struct file *filp,
				  unsigned long uaddr, size_t count)
{
	struct cdma_device *cdev = pipe->cdev;
	struct cdma_zc zc = {};
	int ret;

	if (filp->f_flags & O_NONBLOCK)
		return -EAGAIN;

	mutex_lock(&pipe->tx_lock);

	if (cdma_ring_async_pending(pipe->tx_ring)) {
		mutex_unlock(&pipe->tx_lock);
		return -EBUSY;
	}

	ret = cdma_zc_map(cdev, uaddr, count, DMA_TO_DEVICE, &zc);
	if (ret) {
		mutex_unlock(&pipe->tx_lock);
		return ret;
	}

	cdma_build_sg_pipe_sender(pipe->tx_ring, zc.sgt_dma, count,
				  pipe->hw_pipe_id, 0);
	ret = cdma_ring_submit(pipe->tx_ring, CDMA_BD_PAIR);

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

	mutex_unlock(&pipe->tx_lock);

	cdma_zc_unmap(cdev, &zc, DMA_TO_DEVICE);

	wake_up_interruptible(&pipe->tx_wq);

	return ret ? ret : (ssize_t)count;
}

/*
 * io_uring cancel callback for an async TX kiocb.
 * Symmetric with cdma_rx_cancel().
 */
static int cdma_tx_cancel(struct kiocb *iocb)
{
	return -EAGAIN;
}

/*
 * Async (io_uring) TX write - pins pages, submits the sender BD pair,
 * returns -EIOCBQUEUED. The IRQ handler routes completion to
 * cdma_async_complete_work() which unpins the pages and calls
 * ki_complete().
 */
static ssize_t cdma_tx_write_async(struct cdma_pipe *pipe, struct kiocb *iocb,
				   unsigned long uaddr, size_t count)
{
	struct cdma_device *cdev = pipe->cdev;
	struct cdma_async_req *req;
	int ret;

	/*
	 * The async submit path must allocate the request, pin and DMA-map
	 * the user buffer inline in the submitting task. kzalloc(),
	 * kmalloc_array() and sg_alloc_table_from_pages_segment() all use
	 * GFP_KERNEL, and pin_user_pages_fast() can drop to the sleeping
	 * slow path (mmap_lock, faulting pages in). None of that can honour
	 * IOCB_NOWAIT, so bounce a nowait submission straight back to
	 * io_uring, which reissues it from an io-wq worker where blocking is
	 * allowed and IOCB_NOWAIT is cleared.
	 */
	if (iocb->ki_flags & IOCB_NOWAIT)
		return -EAGAIN;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->ring = pipe->tx_ring;
	req->iocb = iocb;
	req->dir = DMA_TO_DEVICE;
	req->len = count;
	INIT_LIST_HEAD(&req->node);
	INIT_WORK(&req->complete_work, cdma_async_complete_work);

	mutex_lock(&pipe->tx_lock);

	if (!cdma_ring_async_has_space(pipe->tx_ring, CDMA_BD_PAIR)) {
		mutex_unlock(&pipe->tx_lock);
		kfree(req);
		return -EAGAIN;
	}

	ret = cdma_zc_map(cdev, uaddr, count, DMA_TO_DEVICE, &req->zc);
	if (ret) {
		mutex_unlock(&pipe->tx_lock);
		kfree(req);
		return ret;
	}

	cdma_build_sg_pipe_sender(pipe->tx_ring, req->zc.sgt_dma, count,
				  pipe->hw_pipe_id, 0);

	ret = cdma_ring_submit_async(pipe->tx_ring, req, CDMA_BD_PAIR);
	if (ret) {
		mutex_unlock(&pipe->tx_lock);
		cdma_zc_unmap(cdev, &req->zc, DMA_TO_DEVICE);
		kfree(req);
		return ret;
	}

	kiocb_set_cancel_fn(iocb, cdma_tx_cancel);

	mutex_unlock(&pipe->tx_lock);
	return -EIOCBQUEUED;
}

/* write_iter dispatcher - sync (regular write()) vs async (io_uring) */
static ssize_t cdma_tx_write_iter(struct kiocb *iocb, struct iov_iter *from)
{
	struct cdma_pipe *pipe = iocb->ki_filp->private_data;
	struct cdma_device *cdev = pipe->cdev;
	size_t count = iov_iter_count(from);
	unsigned long uaddr;

	if (!count)
		return 0;

	if (count > pipe->max_xfer) {
		dev_warn_ratelimited(&cdev->pdev->dev,
				     "pipe%d: write %zu > max_xfer %zu — chunk the write\n",
				     pipe->pipe_idx, count, pipe->max_xfer);
		return -EMSGSIZE;
	}

	uaddr = cdma_iter_uaddr(from);
	if (!uaddr)
		return -EINVAL;

	if (is_sync_kiocb(iocb))
		return cdma_tx_write_sync(pipe, iocb->ki_filp, uaddr, count);

	return cdma_tx_write_async(pipe, iocb, uaddr, count);
}

/* EPOLLOUT ready when the TX ring has room for another async request. */
static __poll_t cdma_tx_poll(struct file *filp,
			     struct poll_table_struct *wait)
{
	struct cdma_pipe *pipe = filp->private_data;

	poll_wait(filp, &pipe->tx_wq, wait);

	if (cdma_ring_async_has_space(pipe->tx_ring, CDMA_BD_PAIR))
		return EPOLLOUT | EPOLLWRNORM;

	return 0;
}

static const struct file_operations cdma_tx_fops = {
	.owner		= THIS_MODULE,
	.open		= cdma_tx_open,
	.write_iter	= cdma_tx_write_iter,
	.poll		= cdma_tx_poll,
};

/* RX file operations (read-only, /dev/cdma-pipeN-rx) */
static int cdma_rx_open(struct inode *inode, struct file *filp)
{
	struct cdma_pipe *pipe = miscdev_to_pipe(filp->private_data, false);
	struct cdma_device *cdev = pipe->cdev;
	int ret;

	mutex_lock(&cdev->config_lock);
	if (!cdev->pipe_active || cdev->pipes != pipe) {
		ret = -ENODEV;
	} else {
		filp->private_data = pipe;
		/* No FMODE_NOWAIT: see cdma_tx_open() for the rationale. */
		ret = stream_open(inode, filp);
	}
	mutex_unlock(&cdev->config_lock);

	return ret;
}

/*
 * Synchronous RX read — pins pages, builds receiver BD pair at the
 * ring's current PIR, submits to rx_ring, blocks until hardware
 * completes (a remote sender matches via the IERB-configured pipe id).
 * Fully independent from TX.
 */
static ssize_t cdma_rx_read_sync(struct cdma_pipe *pipe, struct file *filp,
				 unsigned long uaddr, size_t count)
{
	struct cdma_device *cdev = pipe->cdev;
	struct cdma_zc zc = {};
	int ret;

	if (filp->f_flags & O_NONBLOCK)
		return -EAGAIN;

	mutex_lock(&pipe->rx_lock);

	if (cdma_ring_async_pending(pipe->rx_ring)) {
		mutex_unlock(&pipe->rx_lock);
		return -EBUSY;
	}

	ret = cdma_zc_map(cdev, uaddr, count, DMA_FROM_DEVICE, &zc);
	if (ret) {
		mutex_unlock(&pipe->rx_lock);
		return ret;
	}

	cdma_build_sg_pipe_receiver(pipe->rx_ring, zc.sgt_dma, count,
				    pipe->hw_pipe_id);
	ret = cdma_ring_submit(pipe->rx_ring, CDMA_BD_PAIR);

	if (!ret)
		ret = cdma_wait_ring(pipe->rx_ring);

	if (ret < 0)
		cdma_ring_cancel(pipe->rx_ring);

	mutex_unlock(&pipe->rx_lock);

	cdma_zc_unmap(cdev, &zc, DMA_FROM_DEVICE);

	wake_up_interruptible(&pipe->rx_wq);

	return ret ? ret : (ssize_t)count;
}

/*
 * io_uring cancel callback for an async RX kiocb.
 *
 * Once BaPIR has been written, the hardware owns the BD pair and the
 * pipe matcher may complete the transfer at any time. We cannot
 * reliably cancel a submitted-but-not-yet-completed receiver: forcing
 * a ring reset would also tear down other in-flight reqs.
 *
 * Return -EAGAIN so io_uring knows the cancellation didn't take
 * effect; the IO completes naturally via the work queue.
 */
static int cdma_rx_cancel(struct kiocb *iocb)
{
	return -EAGAIN;
}

/*
 * Async (io_uring) RX read - pins destination pages, submits the
 * receiver BD pair, returns -EIOCBQUEUED. Completion handled by
 * cdma_async_complete_work().
 */
static ssize_t cdma_rx_read_async(struct cdma_pipe *pipe, struct kiocb *iocb,
				  unsigned long uaddr, size_t count)
{
	struct cdma_device *cdev = pipe->cdev;
	struct cdma_async_req *req;
	int ret;

	if (iocb->ki_flags & IOCB_NOWAIT)
		return -EAGAIN;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->ring = pipe->rx_ring;
	req->iocb = iocb;
	req->dir = DMA_FROM_DEVICE;
	req->len = count;
	INIT_LIST_HEAD(&req->node);
	INIT_WORK(&req->complete_work, cdma_async_complete_work);

	mutex_lock(&pipe->rx_lock);

	if (!cdma_ring_async_has_space(pipe->rx_ring, CDMA_BD_PAIR)) {
		mutex_unlock(&pipe->rx_lock);
		kfree(req);
		return -EAGAIN;
	}

	ret = cdma_zc_map(cdev, uaddr, count, DMA_FROM_DEVICE, &req->zc);
	if (ret) {
		mutex_unlock(&pipe->rx_lock);
		kfree(req);
		return ret;
	}

	cdma_build_sg_pipe_receiver(pipe->rx_ring, req->zc.sgt_dma, count,
				    pipe->hw_pipe_id);

	kiocb_set_cancel_fn(iocb, cdma_rx_cancel);

	ret = cdma_ring_submit_async(pipe->rx_ring, req, CDMA_BD_PAIR);
	if (ret) {
		mutex_unlock(&pipe->rx_lock);
		cdma_zc_unmap(cdev, &req->zc, DMA_FROM_DEVICE);
		kfree(req);
		return ret;
	}

	mutex_unlock(&pipe->rx_lock);
	return -EIOCBQUEUED;
}

/*
 * read_iter dispatcher — sync (regular read()) vs async (io_uring).
 */
static ssize_t cdma_rx_read_iter(struct kiocb *iocb, struct iov_iter *to)
{
	struct cdma_pipe *pipe = iocb->ki_filp->private_data;
	struct cdma_device *cdev = pipe->cdev;
	size_t count = iov_iter_count(to);
	unsigned long uaddr;

	if (!count)
		return 0;

	if (count > pipe->max_xfer) {
		dev_warn_ratelimited(&cdev->pdev->dev,
				     "pipe%d: read %zu > max_xfer %zu — chunk the read\n",
				     pipe->pipe_idx, count, pipe->max_xfer);
		return -EMSGSIZE;
	}

	uaddr = cdma_iter_uaddr(to);
	if (!uaddr)
		return -EINVAL;

	if (is_sync_kiocb(iocb))
		return cdma_rx_read_sync(pipe, iocb->ki_filp, uaddr, count);

	return cdma_rx_read_async(pipe, iocb, uaddr, count);
}

/* EPOLLIN ready when the RX ring has room for another async request. */
static __poll_t cdma_rx_poll(struct file *filp,
			     struct poll_table_struct *wait)
{
	struct cdma_pipe *pipe = filp->private_data;

	poll_wait(filp, &pipe->rx_wq, wait);

	if (cdma_ring_async_has_space(pipe->rx_ring, CDMA_BD_PAIR))
		return EPOLLIN | EPOLLRDNORM;

	return 0;
}

static const struct file_operations cdma_rx_fops = {
	.owner		= THIS_MODULE,
	.open		= cdma_rx_open,
	.read_iter	= cdma_rx_read_iter,
	.poll		= cdma_rx_poll,
};

/*
 * Reap any async requests still queued on a ring during teardown.
 *
 * The caller (cdma_pci_remove) has already disabled the SI (SIMR=0) and
 * torn down the IRQ, so hardware cannot land another writeback and the
 * IRQ thread cannot dequeue concurrently. Every request the IRQ thread
 * had not yet completed is popped, unmapped and completed with -ENODEV.
 * Requests the IRQ thread already dequeued are in flight on the workqueue
 * and are handled by the flush_workqueue() that follows in
 * cdma_pipe_destroy().
 */
static void cdma_ring_drain_async(struct cdma_device *cdev,
				  struct cdma_ring *ring)
{
	struct cdma_async_req *req;

	if (!ring)
		return;

	while ((req = cdma_ring_pop_async(ring))) {
		cdma_zc_unmap(cdev, &req->zc, req->dir);
		req->iocb->ki_complete(req->iocb, -ENODEV);
		kfree(req);
	}
}

/* Pipe allocation and registration */
static void cdma_pipe_destroy(struct cdma_device *cdev)
{
	struct cdma_pipe *pipe = cdev->pipes;

	if (!pipe)
		return;

	/*
	 * Deregister the char devices first so no new open()/read()/write()
	 * can start, then drain the rings and flush the completion workqueue
	 * while the pipe, rings and DMA pool are all still valid. Only after
	 * every in-flight async completion has run do we clear ring->pipe.
	 *
	 * The caller (cdma_pci_remove) must have already run cdma_irq_fini()
	 * so the IRQ thread cannot schedule new completions concurrently with
	 * the drain below.
	 */
	misc_deregister(&pipe->miscdev_rx);
	misc_deregister(&pipe->miscdev_tx);

	if (pipe->tx_ring)
		cancel_delayed_work_sync(&pipe->tx_ring->timeout_work);
	if (pipe->rx_ring)
		cancel_delayed_work_sync(&pipe->rx_ring->timeout_work);

	cdma_ring_drain_async(cdev, pipe->tx_ring);
	cdma_ring_drain_async(cdev, pipe->rx_ring);
	flush_workqueue(cdev->wq);

	if (pipe->tx_ring)
		pipe->tx_ring->pipe = NULL;
	if (pipe->rx_ring)
		pipe->rx_ring->pipe = NULL;

	cdev->pipes = NULL;
	cdev->num_pipes = 0;
	ida_free(&cdma_pipe_ida, pipe->pipe_idx);
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
	pipe->hw_pipe_id = cdev->pipe_id;
	pipe->pipe_idx = pipe_idx;
	pipe->max_xfer = CDMA_PIPE_BUF_SIZE;
	mutex_init(&pipe->tx_lock);
	mutex_init(&pipe->rx_lock);
	init_waitqueue_head(&pipe->tx_wq);
	init_waitqueue_head(&pipe->rx_wq);

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
		pipe->tx_ring->pipe = NULL;
		pipe->rx_ring->pipe = NULL;
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
		pipe->tx_ring->pipe = NULL;
		pipe->rx_ring->pipe = NULL;
		return ret;
	}

	cdev->pipes = pipe;
	cdev->num_pipes = 1;

	dev_info(dev,
		 "registered /dev/%s + /dev/%s (hw_pipe_id=%u, ring0=TX, ring1=RX)\n",
		 pipe->name_tx, pipe->name_rx, pipe->hw_pipe_id);
	return 0;
}

int cdma_cdev_init(struct cdma_device *cdev)
{
	int idx, ret;

	if (!cdev->si)
		return -ENODEV;

	idx = ida_alloc(&cdma_pipe_ida, GFP_KERNEL);
	if (idx < 0)
		return idx;

	ret = cdma_pipe_create(cdev, cdev->si, idx);
	if (ret)
		ida_free(&cdma_pipe_ida, idx);

	return ret;
}

void cdma_cdev_fini(struct cdma_device *cdev)
{
	cdma_pipe_destroy(cdev);
}
