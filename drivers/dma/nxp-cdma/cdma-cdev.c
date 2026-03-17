// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * NXP S32N79 CDMA — Char device layer for pipe I/O
 *
 * Phase 0.5 uses HW memcpy (tx_buf --> rx_buf) on a single SI, with two char
 * devices providing the pipe-like interface:
 *   /dev/cdma-pipeN-tx — write-only (sender)
 *   /dev/cdma-pipeN-rx — read-only  (receiver, blocks until data arrives)
 *
 * Real cross-SI pipe transfers (BD TYPE=PIPE) require IERB pipe-ID routing
 * configured by firmware, which is not yet available.  Phase 1 will switch
 * to true pipe descriptors once firmware support is in place.
 *
 * Copyright 2026 NXP
 */

#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include "cdma.h"

static DEFINE_IDA(cdma_pipe_ida);

static struct cdma_pipe *miscdev_to_pipe(struct miscdevice *misc, bool is_tx)
{
	if (is_tx)
		return container_of(misc, struct cdma_pipe, miscdev_tx);

	return container_of(misc, struct cdma_pipe, miscdev_rx);
}

/* Wait for ring completion via MSI-X IRQ */
static int cdma_wait_ring(struct cdma_ring *ring)
{
	unsigned long tmo = usecs_to_jiffies(CDMA_COMP_TIMEOUT_US);
	int ret;

	ret = wait_for_completion_interruptible_timeout(&ring->done, tmo);
	if (ret == 0)
		return -ETIMEDOUT;
	if (ret < 0)
		return ret;
	return ring->result;
}

/*
 * cdma_memcpy_transfer - DMA memcpy from tx_buf to rx_buf on one SI.
 *
 * Uses a single BD ring with SD[data,src] + DD[data,dst]
 */
static int cdma_memcpy_transfer(struct cdma_pipe *pipe, size_t len)
{
	struct device *dev = &pipe->cdev->pdev->dev;
	struct cdma_ring *ring = pipe->ring;
	int ret;

	cdma_build_memcpy(ring, pipe->tx_dma, pipe->rx_dma, len);

	ret = cdma_ring_submit(ring, CDMA_MEMCPY_NUM_BDS);
	if (ret) {
		dev_err(dev, "pipe%u: memcpy submit failed: %d\n",
			pipe->pipe_id, ret);
		return ret;
	}

	ret = cdma_wait_ring(ring);
	if (ret) {
		dev_err(dev, "pipe%u: memcpy completion error: %d\n",
			pipe->pipe_id, ret);
		/* Reset and re-enable ring after error */
		cdma_ring_reset(ring);
		cdma_ring_init(ring);
		return ret;
	}

	return 0;
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
	ssize_t ret;

	if (!count)
		return 0;

	if (count > pipe->buf_size)
		count = pipe->buf_size;

	mutex_lock(&pipe->lock);

	if (copy_from_user(pipe->tx_buf, ubuf, count)) {
		ret = -EFAULT;
		goto out;
	}

	ret = cdma_memcpy_transfer(pipe, count);
	if (ret)
		goto out;

	/* Mark data available and wake any blocking reader */
	pipe->rx_len = count;
	pipe->rx_consumed = false;
	wake_up_interruptible(&pipe->rx_wq);

	ret = count;
out:
	mutex_unlock(&pipe->lock);
	return ret;
}

static const struct file_operations cdma_tx_fops = {
	.owner		= THIS_MODULE,
	.open		= cdma_tx_open,
	.write		= cdma_tx_write,
};

/* RX file operations (read-only, /dev/cdma-pipeN-rx) */
static int cdma_rx_open(struct inode *inode, struct file *filp)
{
	struct cdma_pipe *pipe = miscdev_to_pipe(filp->private_data, false);

	filp->private_data = pipe;
	return 0;
}

static ssize_t cdma_rx_read(struct file *filp, char __user *ubuf,
			    size_t count, loff_t *ppos)
{
	struct cdma_pipe *pipe = filp->private_data;
	ssize_t ret;

	/* Block until data is available */
	if (filp->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&pipe->lock))
			return -EAGAIN;
		if (pipe->rx_consumed || !pipe->rx_len) {
			mutex_unlock(&pipe->lock);
			return -EAGAIN;
		}
	} else {
		ret = wait_event_interruptible(pipe->rx_wq,
					       !pipe->rx_consumed &&
					       pipe->rx_len > 0);
		if (ret)
			return ret;
		mutex_lock(&pipe->lock);
		if (pipe->rx_consumed || !pipe->rx_len) {
			mutex_unlock(&pipe->lock);
			return 0;
		}
	}

	if (count > pipe->rx_len)
		count = pipe->rx_len;

	if (copy_to_user(ubuf, pipe->rx_buf, count)) {
		ret = -EFAULT;
		goto out;
	}

	pipe->rx_consumed = true;
	ret = count;
out:
	mutex_unlock(&pipe->lock);
	return ret;
}

static __poll_t cdma_rx_poll(struct file *filp,
			     struct poll_table_struct *wait)
{
	struct cdma_pipe *pipe = filp->private_data;
	__poll_t mask = 0;

	poll_wait(filp, &pipe->rx_wq, wait);

	mutex_lock(&pipe->lock);
	if (!pipe->rx_consumed && pipe->rx_len > 0)
		mask = EPOLLIN | EPOLLRDNORM;
	mutex_unlock(&pipe->lock);

	return mask;
}

static const struct file_operations cdma_rx_fops = {
	.owner		= THIS_MODULE,
	.open		= cdma_rx_open,
	.read		= cdma_rx_read,
	.poll		= cdma_rx_poll,
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
	pipe->buf_size = CDMA_PIPE_BUF_SIZE;
	pipe->state = CDMA_PIPE_IDLE;
	pipe->rx_consumed = true;
	mutex_init(&pipe->lock);
	init_waitqueue_head(&pipe->rx_wq);

	/* Use ring 0 for memcpy transfers */
	pipe->ring = &si->rings[0];
	pipe->ring->pipe = pipe;

	/* Allocate DMA-coherent bounce buffers */
	pipe->tx_buf = dmam_alloc_coherent(dev, pipe->buf_size,
					   &pipe->tx_dma, GFP_KERNEL);
	if (!pipe->tx_buf)
		return -ENOMEM;

	pipe->rx_buf = dmam_alloc_coherent(dev, pipe->buf_size,
					   &pipe->rx_dma, GFP_KERNEL);
	if (!pipe->rx_buf)
		return -ENOMEM;

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
		goto err_dereg_tx;
	}

	cdev->pipes = pipe;
	cdev->num_pipes = 1;

	dev_info(dev,
		 "registered /dev/%s + /dev/%s (pipe_id=%u, SI%u ring 0, memcpy mode)\n",
		 pipe->name_tx, pipe->name_rx, pipe->pipe_id, si->si_idx);
	return 0;

err_dereg_tx:
	misc_deregister(&pipe->miscdev_tx);
	return ret;
}

static void cdma_pipe_destroy(struct cdma_device *cdev)
{
	struct cdma_pipe *pipe = cdev->pipes;

	if (!pipe)
		return;

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
