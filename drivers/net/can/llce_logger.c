// SPDX-License-Identifier: GPL-2.0-or-later
/* Copyright 2020-2021 NXP
 *
 * Driver for the NXP Semiconductors LLCE engine logging of CAN messages.
 * The LLCE can be found on S32G2xx.
 */

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/can/dev.h>
#include <linux/circ_buf.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox/nxp-llce/llce_can.h>
#include <linux/mailbox/nxp-llce/llce_can_utils.h>
#include <linux/mailbox/nxp-llce/llce_interface_fifo.h>
#include <linux/mailbox/nxp-llce/llce_mailbox.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define	DRIVER_NAME			"llce-logger"
#define	INPUT_STRING_LENGTH 5

/* Logging structure
 */
struct frame_log {
	struct llce_can_mb frame;
	u8 hw_ctrl;
};

struct llce_dbg {
	struct dentry *dir;
	struct dentry *log;
};

struct llce_syncs {
	wait_queue_head_t fifo_event;
	/* guarantees max 1 userspace thread for consistency */
	struct mutex userspace_access;
};

struct llce_data {
	struct circ_buf cbuf;
	atomic_t frames_received;
	size_t max_ser_size;
};

struct llce_priv {
	struct llce_data data;
	struct llce_syncs syncs;
	struct llce_dbg dbg;
	struct mbox_client rx_client;

	struct device *dev;
	struct mbox_chan *rx_chan;
};

/* This must be a power of two due to circular buffer implementation */
static unsigned long log_size = 4096;
module_param(log_size, ulong, 0660);

static size_t get_left_len(size_t cur_idx, size_t str_len)
{
	if (!str_len)
		return 0;
	return str_len - cur_idx;
}

static unsigned int create_entry_string(struct frame_log *frame,
					char *out_str, int cur_idx,
					size_t str_len)
{
	int wr_size, j;
	bool ide, fdf, rtr, brs, esi;
	u32 tstamp, std_id, ext_id, can_id;
	u8 cur_char, dlc;
	char *str_start;

	unpack_word0(frame->frame.word0, &rtr, &ide, &std_id, &ext_id);
	unpack_word1(frame->frame.word1, &fdf, &dlc, &brs, &esi);
	can_id = std_id & CAN_SFF_MASK;
	if (fdf && ide)
		can_id |= ((ext_id << CAN_SFF_ID_BITS) & CAN_EFF_MASK);

	tstamp = frame->frame.timestamp;
	str_start = out_str + cur_idx;
	wr_size = snprintf(str_start, get_left_len(cur_idx, str_len),
			   "t=%04x rtr=%d brs=%d esi=%d ",
			   tstamp, rtr, brs, esi);
	cur_idx += wr_size;

	str_start = out_str + cur_idx;
	wr_size = snprintf(str_start, get_left_len(cur_idx, str_len),
			   "id=%x ", can_id);
	cur_idx += wr_size;

	str_start = out_str + cur_idx;
	wr_size = snprintf(str_start, get_left_len(cur_idx, str_len), "d=(");
	cur_idx += wr_size;

	for (j = 0; j < can_dlc2len(dlc); j++) {
		cur_char = frame->frame.payload[j];
		str_start = out_str + cur_idx;
		wr_size = snprintf(str_start, get_left_len(cur_idx, str_len),
				   "%02x ", cur_char);
		cur_idx += wr_size;
	}

	/* Remove extra space */
	if (j)
		cur_idx--;
	str_start = out_str + cur_idx;
	wr_size = snprintf(str_start, get_left_len(cur_idx, str_len), ")\n");
	cur_idx += wr_size;

	return cur_idx;
}

static bool has_elem_space(struct llce_priv *priv, size_t fill, size_t size)
{
	if (priv->data.max_ser_size + fill >= size)
		return false;

	return true;
}

static int dump_can_frames(struct llce_priv *priv, char *out_str,
			   unsigned int *cur_idx, size_t size)
{
	struct llce_data *data = &priv->data;
	struct frame_log *buffer;
	size_t circ_elem, i;
	int head, tail, ret;

	buffer = (struct frame_log *)data->cbuf.buf;
	ret = wait_event_freezable(priv->syncs.fifo_event,
				   CIRC_CNT(data->cbuf.head,
					    data->cbuf.tail, log_size) >= 1);
	if (ret)
		return ret;

	/* Read index before reading contents at that index. */
	head = smp_load_acquire(&data->cbuf.head);
	tail = data->cbuf.tail;

	circ_elem = CIRC_CNT_TO_END(head, tail, log_size);

	for (i = 0; i < circ_elem && has_elem_space(priv, *cur_idx, size);
	     i++) {
		*cur_idx = create_entry_string(&buffer[tail + i], out_str,
					       *cur_idx, size);

		atomic_inc(&priv->data.frames_received);
	}

	/* Finish reading descriptor before incrementing tail. */
	smp_store_release(&data->cbuf.tail, (tail + i) & (log_size - 1));

	return 0;
}

static int can_logger_open(struct inode *inode, struct file *file)
{
	struct llce_priv *priv;

	file->private_data = (struct llce_priv *)inode->i_private;
	priv = file->private_data;
	/* Allow only one userspace thread at a time (for consistency) */
	mutex_lock(&priv->syncs.userspace_access);

	return 0;
}

static int can_logger_release(struct inode *inode, struct file *file)
{
	struct llce_priv *priv;

	priv = file->private_data;
	mutex_unlock(&priv->syncs.userspace_access);

	return 0;
}

/**
 *  Used to return a string which contains the CAN logs.
 */
static ssize_t can_logger_read(struct file *file, char __user *user_buffer,
			       size_t size, loff_t *offset)
{
	struct llce_priv *priv;
	char *out_str;
	unsigned int fill = 0;
	int ret;

	priv = file->private_data;

	out_str = kmalloc(size, GFP_KERNEL);
	if (!out_str)
		return -ENOMEM;

	ret = dump_can_frames(priv, out_str, &fill, size);
	if (ret) {
		if (ret == -ERANGE)
			ret = 0;
		goto free_mem;
	}

	if (copy_to_user(user_buffer, out_str, fill)) {
		ret = -EFAULT;
		goto free_mem;
	}

	*offset += fill;

free_mem:
	kfree(out_str);
	return fill;
}

static const struct file_operations fops = {
	.owner		= THIS_MODULE,
	.open		= can_logger_open,
	.release	= can_logger_release,
	.read		= can_logger_read,
	.llseek		= no_llseek,
};

static const struct of_device_id llce_logger_dt_ids[] = {
	{
		.compatible = "nxp,s32g-llce-can-logger",
	},
	{ /* sentinel */ }
};

static int add_dbgfs_entry(struct llce_priv *priv)
{
	int ret = 0;
	struct device *dev = priv->dev;

	priv->dbg.dir = debugfs_create_dir("llce", 0);
	if (!priv->dbg.dir) {
		dev_err(dev, "Couldn't create debugfs dir\n");
		return -ENOENT;
	}

	priv->dbg.log = debugfs_create_file("log", 0600, priv->dbg.dir,
					    priv, &fops);
	if (!priv->dbg.log) {
		dev_err(dev, "Couldn't create debugfs file\n");
		ret = -ENOENT;
	}

	if (ret)
		debugfs_remove_recursive(priv->dbg.dir);

	return ret;
}

static void remove_dbgfs_entry(struct llce_priv *priv)
{
	debugfs_remove_recursive(priv->dbg.dir);
}

/**
 * This is called from IRQ context
 */
static void logger_notif_callback(struct mbox_client *cl, void *msg)
{
	struct llce_priv *priv = container_of(cl, struct llce_priv, rx_client);
	struct llce_data *data = &priv->data;
	int head = data->cbuf.head;
	int tail = READ_ONCE(data->cbuf.tail);
	struct llce_logger_msg *item = msg;
	struct frame_log *log_frame, *buffer;

	buffer = (struct frame_log *)data->cbuf.buf;
	if (CIRC_SPACE(head, tail, log_size) >= 1) {
		log_frame = &buffer[head];

		memcpy_fromio(&log_frame->frame, item->frame,
			      sizeof(log_frame->frame));

		/* Increment head and wake-up once consume */
		smp_store_release(&data->cbuf.head,
				  (head + 1) & (log_size - 1));
		wake_up(&priv->syncs.fifo_event);
	} else {
		dev_warn_ratelimited(priv->dev, "Dropped a packet\n");
	}
}

static int init_mb_channel(struct llce_priv *priv)
{
	int ret = 0;

	priv->rx_client.dev = priv->dev;
	priv->rx_client.tx_block = false;
	priv->rx_client.rx_callback = logger_notif_callback;

	priv->rx_chan = mbox_request_channel(&priv->rx_client, 0);
	if (IS_ERR(priv->rx_chan)) {
		ret = PTR_ERR(priv->rx_chan);
		dev_err(priv->dev, "Failed to get logger mailbox: %d\n", ret);
	}

	return ret;
}

static void release_mb_channel(struct llce_priv *priv)
{
	mbox_free_channel(priv->rx_chan);
}

static void init_max_ser_size(struct llce_priv *priv)
{
	struct frame_log dummy = {
		.frame = {
			.word0 = pack_word0(true, true, CAN_EFF_MASK),
			.word1 = pack_word1(true, 0xFFU, true, true),
		},
	};

	priv->data.max_ser_size = create_entry_string(&dummy, NULL, 0, 0);
}

static int llce_logger_probe(struct platform_device *pdev)
{
	int err;
	struct llce_priv *priv;
	struct device *dev = &pdev->dev;

	if (hweight_long(log_size) != 1) {
		dev_err(dev, "log_size parameter must be a power of two\n");
		return -EINVAL;
	}

	priv = devm_kmalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->dev = dev;

	init_max_ser_size(priv);

	priv->data.cbuf.tail = 0;
	priv->data.cbuf.head = 0;

	priv->data.cbuf.buf = devm_kmalloc(dev, sizeof(*priv) * log_size,
					   GFP_KERNEL);
	if (!priv->data.cbuf.buf)
		return -ENOMEM;

	atomic_set(&priv->data.frames_received, 0);
	init_waitqueue_head(&priv->syncs.fifo_event);
	mutex_init(&priv->syncs.userspace_access);

	err = add_dbgfs_entry(priv);
	if (err)
		return err;

	err = init_mb_channel(priv);
	if (err)
		goto remove_dbgfs_entry;

remove_dbgfs_entry:
	if (err)
		remove_dbgfs_entry(priv);

	return err;
}

static int llce_logger_remove(struct platform_device *pdev)
{
	struct llce_priv *priv;

	priv = platform_get_drvdata(pdev);

	dev_info(&pdev->dev,
		 "Total number of frames received = %u.\n",
		 atomic_read(&priv->data.frames_received));

	release_mb_channel(priv);
	remove_dbgfs_entry(priv);

	return 0;
}

static struct platform_driver llce_logger_driver = {
	.probe	= llce_logger_probe,
	.remove	= llce_logger_remove,
	.driver	= {
		.name			= DRIVER_NAME,
		.owner			= THIS_MODULE,
		.of_match_table = llce_logger_dt_ids,
	},
};

static int __init llce_logger_init(void)
{
	int ret;

	ret = platform_driver_register(&llce_logger_driver);
	if (ret) {
		pr_err("%s Problem registering platform driver\n",
		       DRIVER_NAME);
	}

	return ret;
}

static void __exit llce_logger_exit(void)
{
	platform_driver_unregister(&llce_logger_driver);
}

module_init(llce_logger_init);
module_exit(llce_logger_exit);

MODULE_DESCRIPTION("NXP LLCE logger driver for S32G");
MODULE_LICENSE("GPL v2");
