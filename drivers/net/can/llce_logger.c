// SPDX-License-Identifier: GPL-2.0-or-later
/* Copyright 2020 NXP
 *
 * Driver for the NXP Semiconductors LLCE engine logging of CAN messages.
 * The LLCE can be found on S32G2xx.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include "llce_control.h"
#include "llce_interface.h"

#define	DRIVER_NAME			"llce-logger"
#define	INPUT_STRING_LENGTH 5
#define MAX_LOG_SIZE		4096

struct llce_hw {
	void __iomem *fifo_ier;
	void __iomem *fifo_pop0;
	void __iomem *fifo_push0;
	void __iomem *fifo_status1;
	struct llce_can_shared_memory *shared_mem;
};

struct llce_dbg {
	struct dentry *dir;
	struct dentry *log;
};

struct llce_syncs {
	struct completion new_frames_received;
	/* guarantees max 1 userspace thread for consistency */
	struct mutex userspace_access;
	/* protects against race condition between read and
	 * logging thread on the actual logged frames
	 */
	struct mutex logged_frames_lck;
	u32 can_idx;
	atomic_t frames_received;
	atomic_t log_size;
};

struct llce_data {
	struct frame_log logged_frames[MAX_LOG_SIZE];
	/* input_string will hold the number written by the user in
	 * the llce char device as a string.
	 * Consider a maximum of MAX_LOG_SIZE frames logged on host.
	 * MAX_LOG_SIZE has 4 digits. +1 because of string terminator.
	 */
	char input_string[INPUT_STRING_LENGTH];
	size_t input_string_size;
	size_t entry_disp_size;
	char *out_str;
};

struct llce_priv {
	struct llce_data data;
	struct llce_syncs syncs;
	struct llce_hw hw;
	struct llce_dbg dbg;

	struct platform_device *pdev;
	struct task_struct *copy_thread;
};

static int copy_logs(void *priv_data)
{
	struct llce_priv *priv = priv_data;
	struct llce_can_rx_mb_descriptor desc;
	u16 frame_idx;
	u32 can_idx, wr_idx;
	int log_size, cnt;
	void *src, *dst;

	wr_idx = 0;
	while (1) {
		wait_for_completion(&priv->syncs.new_frames_received);
		if (kthread_should_stop())
			break;

		log_size = atomic_read(&priv->syncs.log_size);
		can_idx = priv->syncs.can_idx;
		desc = priv->hw.shared_mem->can_rx_mb_desc[can_idx];
		frame_idx = desc.u16mb_frame_idx;

		src = &priv->hw.shared_mem->can_mb[frame_idx];
		dst = &priv->data.logged_frames[wr_idx].frame;
		cnt = sizeof(priv->data.logged_frames[wr_idx].frame);
		atomic_inc(&priv->syncs.frames_received);

		mutex_lock(&priv->syncs.logged_frames_lck);
		memcpy_fromio(dst, src, cnt);
		wr_idx = (wr_idx + 1) % log_size;
		mutex_unlock(&priv->syncs.logged_frames_lck);

		reinit_completion(&priv->syncs.new_frames_received);
	}

	complete_and_exit(&priv->syncs.new_frames_received, 0);

	return 0;
}

static size_t get_left_len(size_t cur_idx, size_t str_len)
{
	if (!str_len)
		return 0;
	return str_len - cur_idx;
}

static unsigned int create_entry_string(struct frame_log *cur_frame,
					char *out_str, int cur_idx,
					size_t str_len)
{
	int wr_size, j;
	u32 tstamp;
	u8 cur_char;
	char *str_start;

	tstamp = cur_frame->frame.u32_tstamp;
	str_start = out_str + cur_idx;
	wr_size = snprintf(str_start, get_left_len(cur_idx, str_len),
			   "[t=%04x] ", tstamp);
	cur_idx += wr_size;

	for (j = 0; j < ARRAY_SIZE(cur_frame->frame.payload); j++) {
		cur_char = cur_frame->frame.payload[j];
		str_start = out_str + cur_idx;
		wr_size = snprintf(str_start, get_left_len(cur_idx, str_len),
				   "0x%02x ", cur_char);
		cur_idx += wr_size;
	}

	str_start = out_str + cur_idx;
	wr_size = snprintf(str_start, get_left_len(cur_idx, str_len), "\n");
	cur_idx += wr_size;

	return cur_idx;
}

/* Takes an @offset in the complete output string (uncreated)
 * Takes the @size of the string that will need to be shown
 * return the @ret_len length of the created string
 */
static int create_out_str(struct llce_priv *priv, loff_t *offset,
			  size_t size, int *ret_len, int *read_from,
			  size_t entry_print_size)
{
	int start_idx, end_idx, i;
	unsigned int cur_idx;
	size_t string_size;
	int log_size, frames_received, length;

	/* Maximum log size in frames */
	log_size = atomic_read(&priv->syncs.log_size);
	/* The number of frames received */
	frames_received = atomic_read(&priv->syncs.frames_received);
	if (frames_received >= log_size)
		length = log_size;
	else
		length = frames_received;

	start_idx = *offset / entry_print_size;
	if (start_idx >= length)
		return -ERANGE;
	*read_from = *offset % entry_print_size;
	end_idx = (*offset + size) / entry_print_size;
	if (end_idx >= length)
		end_idx = length - 1;

	/* If start_idx frame is same as end_idx frame then print it
	 * +1 for string terminator
	 */
	string_size = (end_idx - start_idx + 1) * entry_print_size + 1;

	priv->data.out_str = kmalloc(string_size, GFP_KERNEL);
	if (!priv->data.out_str)
		return -ENOMEM;

	mutex_lock(&priv->syncs.logged_frames_lck);
	cur_idx = 0;
	for (i = start_idx; i <= end_idx; i++)
		cur_idx = create_entry_string(&priv->data.logged_frames[i],
					      priv->data.out_str,
						  cur_idx, string_size);
	mutex_unlock(&priv->syncs.logged_frames_lck);
	*ret_len = cur_idx;

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

/**
 * Parses the input string from the user and sets the size of
 *		the log according to that number
 */
static int set_log_size(struct llce_priv *priv)
{
	int rc;
	bool good_range;
	int new_log_size;

	priv->data.input_string[priv->data.input_string_size] = '\0';

	rc = kstrtoint(priv->data.input_string, 0, &new_log_size);
	if (rc == -EINVAL) {
		dev_err(&priv->pdev->dev,
			"Wrong format of input. Accepted input examples: \"100\", \"0x20\", \"085\" .\n");
		return -EINVAL;
	}
	good_range = -ERANGE != rc &&
		new_log_size > 0 &&
		new_log_size <= ARRAY_SIZE(priv->data.logged_frames);
	if (!good_range) {
		dev_err(&priv->pdev->dev,
			"Number should be in interval (0, %d).\n",
			MAX_LOG_SIZE);
		return -ERANGE;
	}

	atomic_set(&priv->syncs.log_size, new_log_size);

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
 *  Used to return a string which contain the CAN logs.
 */
static ssize_t can_logger_read(struct file *file,
			       char __user *user_buffer,
			       size_t size,
				   loff_t *offset)
{
	struct llce_priv *priv;
	int ret_len, rc, read_from;
	size_t entry_print_size;
	int actual_size;

	priv = file->private_data;
	entry_print_size = priv->data.entry_disp_size;
	rc = create_out_str(priv, offset, size,
			    &ret_len, &read_from, entry_print_size);
	if (rc == -ERANGE)
		return 0;
	else if (rc)
		return rc;

	if (size >= ret_len - read_from)
		actual_size = ret_len - read_from;
	else
		actual_size = size;
	if (copy_to_user(user_buffer,
			 priv->data.out_str + read_from,
			 actual_size)) {
		kfree(priv->data.out_str);
		return -EFAULT;
	}

	*offset += actual_size;

	kfree(priv->data.out_str);

	return actual_size;
}

/**
 * Used to set the log size.
 */
static ssize_t can_logger_write(struct file *file,
				const char __user *user_buffer,
				size_t size,
				loff_t *offset)
{
	struct llce_priv *priv;
	int err;

	priv = file->private_data;
	if (size > sizeof(priv->data.input_string)) {
		dev_err(&priv->pdev->dev,
			"Input should be at most %zu characters long\n",
			sizeof(priv->data.input_string) - 1);
		return -EINVAL;
	}

	if (copy_from_user(priv->data.input_string, user_buffer, size)) {
		dev_err(&priv->pdev->dev, "Error copying from userspace\n");
		return -EFAULT;
	}

	priv->data.input_string_size = size;
	err = set_log_size(priv);
	if (err)
		return -ERANGE;
	dev_info(&priv->pdev->dev,
		 "Current log size is %d.\n",
		 atomic_read(&priv->syncs.log_size));

	*offset += size;
	return size;
}

static const struct file_operations fops = {
	.owner		= THIS_MODULE,
	.open		= can_logger_open,
	.release	= can_logger_release,
	.read		= can_logger_read,
	.write		= can_logger_write,
};

static int map_ctrl_regs(struct llce_priv *priv)
{
	struct device *dev;
	struct resource *res;

	dev = &priv->pdev->dev;

	res = platform_get_resource_byname(priv->pdev,
					   IORESOURCE_MEM, "fifo-ier");
	if (!res) {
		dev_err(&priv->pdev->dev, "Failed to read IER from dtb\n");
		return -EIO;
	}
	priv->hw.fifo_ier = devm_ioremap_resource(dev, res);
	if (!priv->hw.fifo_ier) {
		dev_err(&priv->pdev->dev, "Failed to map IER.\n");
		return -EIO;
	}

	res = platform_get_resource_byname(priv->pdev,
					   IORESOURCE_MEM, "fifo-pop0");
	if (!res) {
		dev_err(&priv->pdev->dev, "Failed to read POP0 from dtb\n");
		return -EIO;
	}
	priv->hw.fifo_pop0 = devm_ioremap_resource(dev, res);
	if (!priv->hw.fifo_pop0) {
		dev_err(&priv->pdev->dev, "Failed to map POP0.\n");
		return -EIO;
	}

	res = platform_get_resource_byname(priv->pdev,
					   IORESOURCE_MEM, "fifo-push0");
	if (!res) {
		dev_err(&priv->pdev->dev, "Failed to read PUSH0 from dtb\n");
		return -EIO;
	}
	priv->hw.fifo_push0 = devm_ioremap_resource(dev, res);
	if (!priv->hw.fifo_push0) {
		dev_err(&priv->pdev->dev, "Failed to map PUSH0.\n");
		return -EIO;
	}

	res = platform_get_resource_byname(priv->pdev,
					   IORESOURCE_MEM, "fifo-status1");
	if (!res) {
		dev_err(&priv->pdev->dev, "Failed to read STATUS1 from dtb\n");
		return -EIO;
	}
	priv->hw.fifo_status1 = devm_ioremap_resource(dev, res);
	if (!priv->hw.fifo_status1) {
		dev_err(&priv->pdev->dev, "Failed to map STATUS1.\n");
		return -EIO;
	}

	return 0;
}

static irqreturn_t irq_handler(int irq, void *priv_data)
{
	struct llce_priv *priv;

	priv = priv_data;
	/* Read the index of the message buffer */
	priv->syncs.can_idx = ioread32(priv->hw.fifo_pop0) &
		LLCE_CAN_CONFIG_FIFO_FIXED_MASK;
	complete(&priv->syncs.new_frames_received);
	/* Write back to Rx core the index of the receive
	 * mb descriptor
	 */
	iowrite32(priv->syncs.can_idx, priv->hw.fifo_push0);
	/* Clear the interrupt status flag */
	iowrite32(LLCE_FIFO_FNEMTY, priv->hw.fifo_status1);

	return IRQ_HANDLED;
}

/* Device Tree Parsing */

static const struct of_device_id llce_logger_dt_ids[] = {
	{
		.compatible = "fsl,s32g-llcelogger",
	},
	{ /* sentinel */ }
};

static int init_interfaces(struct llce_priv *priv)
{
	int ret;
	struct device *dev = &priv->pdev->dev;
	struct resource *res;

	ret = map_ctrl_regs(priv);
	if (ret) {
		dev_err(dev, "Failed initial mapping of registers.\n");
		return ret;
	}
	dev_info(dev, "Success initial mapping of registers.\n");

	res = platform_get_resource_byname(priv->pdev,
					   IORESOURCE_MEM,
						"llce-shared-mem");
	priv->hw.shared_mem = devm_ioremap_resource(dev, res);
	if (!priv->hw.shared_mem) {
		dev_err(dev,
			"Couldn't map the LLCE shared memory\n");
		return -ENOMEM;
	}
	dev_info(dev,
		 "Success initial mapping of shared memory.\n");

	priv->dbg.dir = debugfs_create_dir("llce", 0);
	if (!priv->dbg.dir) {
		dev_err(&priv->pdev->dev, "Couldnt' create debugfs dir\n");
		return -ENOENT;
	}
	priv->dbg.log = debugfs_create_file("log",
					    0600,
					priv->dbg.dir,
					priv,
					&fops);
	if (!priv->dbg.log) {
		dev_err(&priv->pdev->dev, "Couldn't create debugfs file\n");
		ret = -ENOENT;
		goto exit_file_fail;
	}

	return 0;

exit_file_fail:
	debugfs_remove_recursive(priv->dbg.dir);
	return ret;
}

static void release_interfaces(struct llce_priv *priv)
{
	debugfs_remove_recursive(priv->dbg.dir);
}

static int init_multithread_support(struct llce_priv *priv)
{
	struct device *dev = &priv->pdev->dev;

	atomic_set(&priv->syncs.log_size, MAX_LOG_SIZE);
	atomic_set(&priv->syncs.frames_received, 0);
	priv->data.input_string_size = 0;

	init_completion(&priv->syncs.new_frames_received);
	priv->copy_thread = kthread_run(copy_logs, priv,
					"llce_copy_thread");
	if (IS_ERR(priv->copy_thread)) {
		dev_err(dev, "Copy thread creation failed\n");
		return IS_ERR(priv->copy_thread);
	}

	mutex_init(&priv->syncs.userspace_access);
	mutex_init(&priv->syncs.logged_frames_lck);

	return 0;
}

static void kill_multithread_support(struct llce_priv *priv)
{
	complete(&priv->syncs.new_frames_received);
	kthread_stop(priv->copy_thread);
}

/**
 * llce_logger_probe - Registers interrupt handler for LLCE logging, rutes
 *		interrupts inside LLCE, prepares interfaces to LLCE & userspace.
 * @pdev: the associated platform device
 *
 * Gets the physical interrupt line number for LLCE logging from device tree
 * and maps it into a virtual interrupt line, then associates an interrupt
 * handler to that interrupt line.
 *
 * Prepare interface to LLCE (mapping registers and shared memory).
 * Prepare interface to userspace (char dev).
 *
 * Return: 0 on success, error code otherwise.
 */

static int llce_logger_probe(struct platform_device *pdev)
{
	int err, irq_no;
	unsigned int initial_content;
	struct llce_priv *priv;
	struct frame_log dummy;

	/* This driver has to be compiled as module because the probing
	 * function results in segfault if SRAM is not loaded.
	 */
	if (!IS_MODULE(CONFIG_LLCE_LOGGER))
		return -ENOMEDIUM;

	priv = devm_kmalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv);
	priv->pdev = pdev;

	err = init_multithread_support(priv);
	if (err)
		return err;

	err = init_interfaces(priv);
	if (err)
		goto release_thread;

	priv->data.entry_disp_size = create_entry_string(&dummy, NULL, 0, 0);

	irq_no = platform_get_irq(pdev, 0);
	if (irq_no < 0) {
		dev_err(&pdev->dev, "Failed platform get irq.\n");
		err = irq_no;
		goto release_interfaces;
	}
	err = devm_request_irq(&pdev->dev,
			       irq_no,
			       irq_handler,
			       IRQF_SHARED,
			       "llce-logger-handler",
			       priv);
	if (err) {
		dev_err(&pdev->dev, "Failed requesting irq.\n");
		goto release_interfaces;
	}

	/* If FW is not loaded, this mask application results in segfault.
	 * Can't be helped because checking if FW loaded triggers segfault
	 * if not loaded, as well.
	 * This mask application enables routing inside FIFO module.
	 */
	initial_content = ioread32(priv->hw.fifo_ier);
	initial_content |= LLCE_FIFO_FNEMTY;
	iowrite32(initial_content, priv->hw.fifo_ier);

release_interfaces:
	if (err)
		release_interfaces(priv);
release_thread:
	if (err)
		kill_multithread_support(priv);
	return err;
}

static int llce_logger_remove(struct platform_device *pdev)
{
	struct llce_priv *priv;

	priv = platform_get_drvdata(pdev);
	kill_multithread_support(priv);
	dev_info(&pdev->dev,
		 "Total number of frames received = %u.\n",
		 atomic_read(&priv->syncs.frames_received));
	debugfs_remove_recursive(priv->dbg.dir);

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

/**
 * llce_logger_init - maps registers, shared memory, activate interrupts,
 *		provide userspace access
 */
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
	pr_info("called exit, done\n");
}

module_init(llce_logger_init);
module_exit(llce_logger_exit);

MODULE_DESCRIPTION("NXP llce logger driver for S32G");
MODULE_LICENSE("GPL v2");
