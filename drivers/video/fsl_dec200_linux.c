// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 NXP
 */

#include "fsl_dec200_linux.h"
#include "fsl_dec200_regs.h"
#include "fsl_dec200_ioctl.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>
#include <linux/gfp.h>
#include <linux/mod_devicetable.h>
#include <linux/cdev.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>

/**********************************************************
 * INFO: Module info
 **********************************************************/
MODULE_AUTHOR("Cosmin-Dumitru Oprea <cosmin.oprea@nxp.com>");
MODULE_DESCRIPTION("Freescale DEC200 driver");
MODULE_LICENSE("GPL");

/**********************************************************
 * DEC200 defines
 **********************************************************/

#define DRIVER_NAME			"fsl_dec200"
#define DEC200_DEC_NAME			"dec200_dec"
#define DEC200_ENC_NAME			"dec200_enc"

#define DEC200_INIT_TRUE			0
#define DEC200_INIT_ERR_PROBE		1
#define DEC200_INIT_ERR_CFG		2

#define BIG_BUFFER_SIZE			1024

/* DEC200 clock definitions */
#define DEC200_SYS3_CLOCK_NAME		"sys3"
#define DEC200_SYS6_CLOCK_NAME		"sys6"
#define DEC200_AXI_CLOCK_NAME		"dcu"

/**********************************************************
 * GLOBAL DEC200 struct data
 **********************************************************/
struct dec200_hwdata {
	enum DEC200_TYPE		devtype;
	char				*name;
};

struct dec200_struct {
	struct cdev			*cdev;
	struct platform_device		*pdev;
	struct class			*class;
	const struct dec200_hwdata	*hwdata;
	struct DEC200_MemMap		*regs;
	struct {
		struct dentry		*dir;
		struct dentry		*regs[8];
		char			big_buffer[BIG_BUFFER_SIZE];
	} debugfs;
	union {
		struct clk		*axi_clk;
		struct clk		*sys3_clk;
	};
	struct clk			*sys6_clk;
	dev_t				devno;
	int				init_status;
};
struct dec200_struct *g_dec200[DEC200_MAX_NUM];

static struct dec200_hwdata fsl_dec200_hwdata[] = {
	{
		.devtype = DEC200_DEC,
		.name = DEC200_DEC_NAME,
	}, {
		.devtype = DEC200_ENC,
		.name = DEC200_ENC_NAME,
	}
};

static struct platform_device_id fsl_dec200_devtype[] = {
	{
		.name = DEC200_DEC_NAME,
		.driver_data = (kernel_ulong_t)&fsl_dec200_hwdata[DEC200_DEC],
	}, {
		.name = DEC200_ENC_NAME,
		.driver_data = (kernel_ulong_t)&fsl_dec200_hwdata[DEC200_ENC],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, fsl_dec200_devtype);

static const struct of_device_id fsl_dec200_dt_ids[] = {
	{
		.compatible = "fsl,s32v234-dec200_dec",
		.data = &fsl_dec200_hwdata[DEC200_DEC],
	},
	{
		.compatible = "fsl,s32v234-dec200_enc",
		.data = &fsl_dec200_hwdata[DEC200_ENC],
	},
};
MODULE_DEVICE_TABLE(of, fsl_dec200_dt_ids);

static struct dec200_struct *fsl_get_dec200(struct dentry *dentry)
{
	enum DEC200_TYPE dec_id = DEC200_DEC;

	const char *name = dentry->d_iname;

	if (!strncmp(name, DEC200_ENC_NAME, 10))
		dec_id = DEC200_ENC;

	if (dec_id < 0 || DEC200_MAX_NUM <= dec_id)
		return NULL;

	return g_dec200[dec_id];
}

static uint8_t fsl_dec200_get_axi_id(struct file *filp)
{
	const char *name = filp->f_path.dentry->d_iname;

	if (strncmp(name, "regs", 4) || name[4] < '0' || '7' < name[4])
		return -EINVAL;

	return name[4] - '0';
}

static void fsl_dec200_config(uint8_t axi_id, struct dec200_struct *dec200)
{
	u32 val;

	if (axi_id < 0 || 7 < axi_id)
		return;

	if (dec200->hwdata->devtype == DEC200_DEC) {
		val = readl((void *)&dec200->regs->gcregDECReadConfig);
		val |= (COMPRESSION_SIZE64_BYTE << COMPRESSION_SIZE_SHIFT) |
		(COMPRESSION_FORMAT_ARGB8 << COMPRESSION_FORMAT_SHIFT) |
		(SWIZZLE_ARGB << SWIZZLE_SHIFT) | COMPRESSION_ENABLE;
		writel(val, (void *)&dec200->regs->gcregDECReadConfig[axi_id]);
	} else if (dec200->hwdata->devtype == DEC200_ENC) {
		val = readl((void *)&dec200->regs->gcregDECWriteConfig);
		val |= (COMPRESSION_SIZE64_BYTE << COMPRESSION_SIZE_SHIFT) |
		(COMPRESSION_FORMAT_ARGB8 << COMPRESSION_FORMAT_SHIFT) |
		(SWIZZLE_ARGB << SWIZZLE_SHIFT) | COMPRESSION_ENABLE;
		writel(val, (void *)&dec200->regs->gcregDECWriteConfig[axi_id]);
	}

	/* Read pixel data */
	writel(0, (void *)&dec200->regs->gcregDECReadBufferBase[axi_id]);
	/* Read cache status */
	writel(0, (void *)&dec200->regs->gcregDECReadCacheBase[axi_id]);
	/* Write pixel data */
	writel(0, (void *)&dec200->regs->gcregDECWriteBufferBase[axi_id]);
	/* Write cache status */
	writel(0, (void *)&dec200->regs->gcregDECWriteCacheBase[axi_id]);

	val = DECCONTROL_RESET_VALUE;
	val &= ~(DISABLE_DEBUG_REGISTERS | DISABLE_COMPRESSION);
	writel(val, (void *)&dec200->regs->gcregDECControl);
}

static int fsl_dec200_init(struct dec200_struct *dec200)
{
	uint8_t axi_id;

	for (axi_id = 0; axi_id < 8; ++axi_id)
		fsl_dec200_config(axi_id, dec200);

	return 0;
}

int fsl_dec200_set_compression_size(enum DEC200_TYPE devtype, uint8_t axi_id,
				    enum SWIZZLE val)
{
	struct dec200_struct *dec200;
	u32 tmp;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];

	tmp = readl((void *)&dec200->regs->gcregDECControl);
	tmp &= ~((COMPRESSION_SIZE64_BYTE | COMPRESSION_SIZE128_BYTE |
		 COMPRESSION_SIZE256_BYTE) << COMPRESSION_SIZE_SHIFT);
	tmp |= val << COMPRESSION_SIZE_SHIFT;
	writel(tmp, (void *)&dec200->regs->gcregDECControl);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dec200_set_compression_size);

int fsl_dec200_set_compression_format(enum DEC200_TYPE devtype,
				uint8_t axi_id, enum COMPRESSION_FORMAT val)
{
	struct dec200_struct *dec200;
	u32 tmp;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];

	tmp = readl((void *)&dec200->regs->gcregDECControl);
	tmp &= ~((COMPRESSION_FORMAT_ARGB8 | COMPRESSION_FORMAT_XRGB8 |
		 COMPRESSION_FORMAT_AYUV | COMPRESSION_FORMAT_UYVY |
		 COMPRESSION_FORMAT_YUY2 | COMPRESSION_FORMAT_YUV_ONLY |
		 COMPRESSION_FORMAT_UV_MIX) << COMPRESSION_FORMAT_SHIFT);
	tmp |= val << COMPRESSION_FORMAT_SHIFT;
	writel(tmp, (void *)&dec200->regs->gcregDECControl);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dec200_set_compression_format);

int fsl_dec200_set_swizzle(enum DEC200_TYPE devtype, uint8_t axi_id,
			   enum SWIZZLE val)
{
	struct dec200_struct *dec200;
	u32 tmp;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];

	tmp = readl((void *)&dec200->regs->gcregDECControl);
	tmp &= ~((SWIZZLE_ARGB | SWIZZLE_RGBA | SWIZZLE_ABGR | SWIZZLE_BGRA) <<
		 SWIZZLE_SHIFT);
	tmp |= val << SWIZZLE_SHIFT;
	writel(tmp, (void *)&dec200->regs->gcregDECControl);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dec200_set_swizzle);

int fsl_dec200_reset(enum DEC200_TYPE devtype)
{
	struct dec200_struct *dec200;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;

	dec200 = g_dec200[devtype];

	return fsl_dec200_init(dec200);
}
EXPORT_SYMBOL_GPL(fsl_dec200_reset);

void fsl_dec200_enable_compression(enum DEC200_TYPE devtype, uint8_t axi_id)
{
	struct dec200_struct *dec200;
	u32 val;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return;
	if (axi_id < 0 || 7 < axi_id)
		return;

	dec200 = g_dec200[devtype];

	if (devtype == DEC200_ENC) {
		val = readl((void *)&dec200->regs->gcregDECWriteConfig[axi_id]);
		val |= COMPRESSION_ENABLE;
		writel(val, (void *)&dec200->regs->gcregDECWriteConfig[axi_id]);
	} else if (devtype == DEC200_DEC) {
		val = readl((void *)&dec200->regs->gcregDECReadConfig[axi_id]);
		val |= COMPRESSION_ENABLE;
		writel(val, (void *)&dec200->regs->gcregDECReadConfig[axi_id]);
	}

	val = readl((void *)&dec200->regs->gcregDECControl);
	val &= ~DISABLE_COMPRESSION;
	writel(val, (void *)&dec200->regs->gcregDECControl);
}
EXPORT_SYMBOL_GPL(fsl_dec200_enable_compression);

void fsl_dec200_disable_compression(enum DEC200_TYPE devtype, uint8_t axi_id)
{
	struct dec200_struct *dec200;
	u32 val;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return;
	if (axi_id < 0 || 7 < axi_id)
		return;

	dec200 = g_dec200[devtype];

	if (devtype == DEC200_DEC) {
		val = readl((void *)&dec200->regs->gcregDECReadConfig[axi_id]);
		val &= ~COMPRESSION_ENABLE;
		writel(val, (void *)&dec200->regs->gcregDECReadConfig[axi_id]);
	} else if (devtype == DEC200_ENC) {
		val = readl((void *)&dec200->regs->gcregDECWriteConfig[axi_id]);
		val &= ~COMPRESSION_ENABLE;
		writel(val, (void *)&dec200->regs->gcregDECWriteConfig[axi_id]);
	}

	val = readl((void *)&dec200->regs->gcregDECControl);
	val |= DISABLE_COMPRESSION;
	writel(val, (void *)&dec200->regs->gcregDECControl);
}
EXPORT_SYMBOL_GPL(fsl_dec200_disable_compression);

u32 fsl_dec200_get_reg_base(enum DEC200_TYPE devtype, uint8_t axi_id)
{
	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	return (u32)(ulong)g_dec200[devtype]->regs;
}
EXPORT_SYMBOL_GPL(fsl_dec200_get_reg_base);

u32 fsl_dec200_get_read_buffer_base(enum DEC200_TYPE devtype, uint8_t axi_id)
{
	struct dec200_struct *dec200;
	u32 val;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];
	val = readl((void *)&dec200->regs->gcregDECReadBufferBase[axi_id]);

	return val;
}
EXPORT_SYMBOL_GPL(fsl_dec200_get_read_buffer_base);

int fsl_dec200_set_read_buffer_base(enum DEC200_TYPE devtype, uint8_t axi_id,
				    u32 addr)
{
	struct dec200_struct *dec200;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];
	writel(addr, (void *)&dec200->regs->gcregDECReadBufferBase[axi_id]);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dec200_set_read_buffer_base);

u32 fsl_dec200_get_read_cache_base(enum DEC200_TYPE devtype, uint8_t axi_id)
{
	struct dec200_struct *dec200;
	u32 val;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];
	val = readl((void *)&dec200->regs->gcregDECReadCacheBase[axi_id]);

	return val;
}
EXPORT_SYMBOL_GPL(fsl_dec200_get_read_cache_base);

int fsl_dec200_set_read_cache_base(enum DEC200_TYPE devtype, uint8_t axi_id,
				   u32 addr)
{
	struct dec200_struct *dec200;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];
	writel(addr, (void *)&dec200->regs->gcregDECReadCacheBase[axi_id]);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dec200_set_read_cache_base);

u32 fsl_dec200_get_write_buffer_base(enum DEC200_TYPE devtype, uint8_t axi_id)
{
	struct dec200_struct *dec200;
	u32 val;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];
	val = readl((void *)&dec200->regs->gcregDECWriteBufferBase[axi_id]);

	return val;
}
EXPORT_SYMBOL_GPL(fsl_dec200_get_write_buffer_base);

int fsl_dec200_set_write_buffer_base(enum DEC200_TYPE devtype, uint8_t axi_id,
				     u32 addr)
{
	struct dec200_struct *dec200;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];
	writel(addr, (void *)&dec200->regs->gcregDECWriteBufferBase[axi_id]);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dec200_set_write_buffer_base);

u32 fsl_dec200_get_write_cache_base(enum DEC200_TYPE devtype, uint8_t axi_id)
{
	struct dec200_struct *dec200;
	u32 val;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];
	val = readl((void *)&dec200->regs->gcregDECWriteCacheBase[axi_id]);

	return val;
}
EXPORT_SYMBOL_GPL(fsl_dec200_get_write_cache_base);

int fsl_dec200_set_write_cache_base(enum DEC200_TYPE devtype, uint8_t axi_id,
				    u32 addr)
{
	struct dec200_struct *dec200;

	if (devtype < 0 || DEC200_MAX_NUM <= devtype)
		return -EINVAL;
	if (axi_id < 0 || 7 < axi_id)
		return -EINVAL;

	dec200 = g_dec200[devtype];
	writel(addr, (void *)&dec200->regs->gcregDECWriteCacheBase[axi_id]);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dec200_set_write_cache_base);

static ssize_t debugfs_read_file(struct file *filp, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	struct dec200_struct *dec200 =
				fsl_get_dec200(filp->f_path.dentry->d_parent);
	char *big_buffer = dec200->debugfs.big_buffer;
	char *s = big_buffer;
	uint8_t axi_id = fsl_dec200_get_axi_id(filp);

	s += sprintf(s, "gcregDECReadConfig = \t0x%08x\n",
			readl(&dec200->regs->gcregDECReadConfig[axi_id]));

	s += sprintf(s, "gcregDECWriteConfig = \t0x%08x\n",
			readl(&dec200->regs->gcregDECWriteConfig[axi_id]));

	s += sprintf(s, "gcregDECReadBufferBase = \t0x%08x\n",
			readl(&dec200->regs->gcregDECReadBufferBase[axi_id]));

	s += sprintf(s, "gcregDECReadCacheBase = \t0x%08x\n",
			readl(&dec200->regs->gcregDECReadCacheBase[axi_id]));

	s += sprintf(s, "gcregDECWriteBufferBase = \t0x%08x\n",
			readl(&dec200->regs->gcregDECWriteBufferBase[axi_id]));

	s += sprintf(s, "gcregDECWriteCacheBase = \t0x%08x\n",
			readl(&dec200->regs->gcregDECWriteCacheBase[axi_id]));

	s += sprintf(s, "gcregDECControl = \t0x%08x\n",
			readl(&dec200->regs->gcregDECControl));

	s += sprintf(s, "gcDECTotalReadsIn = \t0x%08x\n",
			readl(&dec200->regs->gcDECTotalReadsIn));

	s += sprintf(s, "gcDECTotalWritesIn = \t0x%08x\n",
			readl(&dec200->regs->gcDECTotalWritesIn));

	return  simple_read_from_buffer(userbuf, count, ppos,
				big_buffer, s - big_buffer);
}

static int fsl_dec200_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t fsl_dec200_read(struct file *filp, char *buff,
		size_t len, loff_t *off)
{
	return 0;
}

static int fsl_dec200_close(struct inode *inode, struct file *filp)
{
	return 0;
}

long fsl_dec200_ioctl(struct file *filp,
		unsigned int cmd,
		unsigned long arg)
{
	struct IOCTL_SAMPLE sample;
	struct IOCTL_SWIZZLE swizzle;
	struct IOCTL_COMPRESSION_SIZE comp_size;
	struct IOCTL_COMPRESSION_FORMAT comp_format;
	struct IOCTL_ADDR addr;
	long ret = 0;

	struct dec200_struct *dec200 = fsl_get_dec200(filp->f_path.dentry);
	enum DEC200_TYPE devtype = dec200->hwdata->devtype;

	switch (cmd) {
	case IOCTL_ENABLE_COMPRESSION:
	{
		ret = copy_from_user(&sample, (struct IOCTL_SAMPLE *)arg,
				     sizeof(sample));
		fsl_dec200_enable_compression(devtype, sample.axi_id);
		break;
	}
	case IOCTL_GET_REG_BASE:
	{
		ret = copy_from_user(&addr, (struct IOCTL_ADDR *)arg,
				sizeof(addr));
		addr.addr = fsl_dec200_get_reg_base(devtype, addr.axi_id);
		ret = copy_to_user((struct IOCTL_ADDR *)arg, &addr,
				sizeof(addr));
		break;
	}
	case IOCTL_SET_COMPRESSION_SIZE:
	{
		ret = copy_from_user(&comp_size,
				(struct IOCTL_COMPRESSION_SIZE *)arg,
				sizeof(comp_size));
		ret = fsl_dec200_set_compression_size(devtype, comp_size.axi_id,
				comp_size.val);
		break;
	}
	case IOCTL_SET_COMPRESSION_FORMAT:
	{
		ret = copy_from_user(&comp_format,
				(struct IOCTL_COMPRESSION_FORMAT *)arg,
				sizeof(comp_format));
		ret = fsl_dec200_set_compression_format(devtype,
				comp_format.axi_id, comp_format.val);
		break;
	}
	case IOCTL_SET_SWIZZLE:
	{
		ret = copy_from_user(&swizzle, (struct IOCTL_SWIZZLE *)arg,
				sizeof(swizzle));
		ret = fsl_dec200_set_swizzle(devtype, swizzle.axi_id,
				swizzle.val);
		break;
	}
	case IOCTL_DISABLE_COMPRESSION:
	{
		ret = copy_from_user(&sample, (struct IOCTL_SAMPLE *)arg,
				sizeof(sample));
		fsl_dec200_disable_compression(devtype, sample.axi_id);
		break;
	}
	case IOCTL_RESET_CONFIGURATION:
	{
		ret = fsl_dec200_reset(devtype);
		break;
	}
	case IOCTL_GET_READ_BUFFER_BASE:
	{
		ret = copy_from_user(&addr, (struct IOCTL_ADDR *)arg,
				sizeof(addr));
		addr.addr = fsl_dec200_get_read_buffer_base(devtype,
				addr.axi_id);
		ret = copy_to_user((struct IOCTL_ADDR *)arg, &addr,
				sizeof(addr));
		break;
	}
	case IOCTL_SET_READ_BUFFER_BASE:
	{
		ret = copy_from_user(&addr, (struct IOCTL_ADDR *)arg,
				sizeof(addr));
		ret = fsl_dec200_set_read_buffer_base(devtype, addr.axi_id,
				addr.addr);
		break;
	}
	case IOCTL_GET_READ_CACHE_BASE:
	{
		ret = copy_from_user(&addr, (struct IOCTL_ADDR *)arg,
				sizeof(addr));
		addr.addr = fsl_dec200_get_read_cache_base(devtype,
				addr.axi_id);
		ret = copy_to_user((struct IOCTL_ADDR *)arg, &addr,
				sizeof(addr));
		break;
	}
	case IOCTL_SET_READ_CACHE_BASE:
	{
		ret = copy_from_user(&addr, (struct IOCTL_ADDR *)arg,
				sizeof(addr));
		ret = fsl_dec200_set_read_cache_base(devtype, addr.axi_id,
				addr.addr);
		break;
	}
	case IOCTL_GET_WRITE_BUFFER_BASE:
	{
		ret = copy_from_user(&addr, (struct IOCTL_ADDR *)arg,
				sizeof(addr));
		addr.addr = fsl_dec200_get_write_buffer_base(devtype,
				addr.axi_id);
		ret = copy_to_user((struct IOCTL_ADDR *)arg, &addr,
				sizeof(addr));
		break;
	}
	case IOCTL_SET_WRITE_BUFFER_BASE:
	{
		ret = copy_from_user(&addr, (struct IOCTL_ADDR *)arg,
				sizeof(addr));
		ret = fsl_dec200_set_write_buffer_base(devtype, addr.axi_id,
				addr.addr);
		break;
	}
	case IOCTL_GET_WRITE_CACHE_BASE:
	{
		ret = copy_from_user(&addr, (struct IOCTL_ADDR *)arg,
				sizeof(addr));
		addr.addr = fsl_dec200_get_write_cache_base(devtype,
				addr.axi_id);

		ret = copy_to_user((struct IOCTL_ADDR *)arg, &addr,
				sizeof(addr));
		break;
	}
	case IOCTL_SET_WRITE_CACHE_BASE:
	{
		ret = copy_from_user(&addr, (struct IOCTL_ADDR *)arg,
				sizeof(addr));
		ret = fsl_dec200_set_write_cache_base(devtype, addr.axi_id,
				addr.addr);
		break;
	}
	break;
	}

	return ret;
}

const struct file_operations fsl_dec200_fops = {
	.owner		= THIS_MODULE,
	.read		= fsl_dec200_read,
	.open		= fsl_dec200_open,
	.release	= fsl_dec200_close,
	.unlocked_ioctl	= fsl_dec200_ioctl,
};

static const struct file_operations debugfs_fops = {
	.open = simple_open,
	.read = debugfs_read_file,
};

static void fsl_dec200_debugfs_init(struct dec200_struct *dec200)
{
	dec200->debugfs.dir = debugfs_create_dir(dec200->hwdata->name, NULL);

	dec200->debugfs.regs[0] = debugfs_create_file("regs0", 0444,
			dec200->debugfs.dir, dec200, &debugfs_fops);
	dec200->debugfs.regs[1] = debugfs_create_file("regs1", 0444,
			dec200->debugfs.dir, dec200, &debugfs_fops);
	dec200->debugfs.regs[2] = debugfs_create_file("regs2", 0444,
			dec200->debugfs.dir, dec200, &debugfs_fops);
	dec200->debugfs.regs[3] = debugfs_create_file("regs3", 0444,
			dec200->debugfs.dir, dec200, &debugfs_fops);
	dec200->debugfs.regs[4] = debugfs_create_file("regs4", 0444,
			dec200->debugfs.dir, dec200, &debugfs_fops);
	dec200->debugfs.regs[5] = debugfs_create_file("regs5", 0444,
			dec200->debugfs.dir, dec200, &debugfs_fops);
	dec200->debugfs.regs[6] = debugfs_create_file("regs6", 0444,
			dec200->debugfs.dir, dec200, &debugfs_fops);
	dec200->debugfs.regs[7] = debugfs_create_file("regs7", 0444,
			dec200->debugfs.dir, dec200, &debugfs_fops);
}

static void fsl_dec200_debugfs_remove(struct dec200_struct *dec200)
{
	uint8_t axi_id;

	for (axi_id = 0; axi_id < 8; ++axi_id)
		debugfs_remove(dec200->debugfs.regs[axi_id]);

	debugfs_remove(dec200->debugfs.dir);
}

static int fsl_dec200_dev_create(struct dec200_struct *dec200)
{
	int ret;

	/* Alloc MAJOR number for the character device  */
	ret = alloc_chrdev_region(&dec200->devno, 0, 1, dec200->hwdata->name);
	if (ret < 0) {
		dev_err(&dec200->pdev->dev,
			"DEC200: alloc_chrdev_region error %d\n", ret);
		return ret;
	}

	dec200->class = class_create(THIS_MODULE, dec200->hwdata->name);
	if (!dec200->class) {
		dev_err(&dec200->pdev->dev,
			"DEC200: class_create error\n");
		return -1;
	}

	if (!(device_create(dec200->class, NULL,
			dec200->devno, NULL, dec200->hwdata->name))) {
		dev_err(&dec200->pdev->dev,
			"DEC200: device_create error\n");
		return -1;
	}

	/* setup file operations */
	dec200->cdev = cdev_alloc();
	dec200->cdev->ops = &fsl_dec200_fops;

	/* add device */
	ret = cdev_add(dec200->cdev, dec200->devno, 1);
	if (ret < 0) {
		dev_err(&dec200->pdev->dev,
			"DEC200: cdev_add error %d\n", ret);
		return ret;
	}

	return 0;
}

int fsl_dec200_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id = of_match_device(fsl_dec200_dt_ids,
							   &pdev->dev);
	struct dec200_struct *dec200;
	struct resource *res;

	dec200 = devm_kzalloc(&pdev->dev, sizeof(*dec200), GFP_KERNEL);
	if (!dec200)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, dec200);
	dec200->pdev = pdev;

	/* map register space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dec200->init_status = DEC200_INIT_ERR_CFG;
		dev_err(&pdev->dev, "could not get memory IO resource\n");
		return -ENODEV;
	}
	dec200->regs = (struct DEC200_MemMap *)devm_ioremap_resource(&pdev->dev,
								     res);

	if (IS_ERR(dec200->regs)) {
		dec200->init_status = DEC200_INIT_ERR_CFG;
		dev_err(&pdev->dev, "DCU: could not ioremap resource\n");
		return PTR_ERR(dec200->regs);
	}

	if (of_id)
		dec200->hwdata = of_id->data;
	else
		dec200->hwdata = (struct dec200_hwdata *)
			platform_get_device_id(pdev)->driver_data;

	g_dec200[dec200->hwdata->devtype] = dec200;

	/* create device and register it in /dev through sysfs */
	fsl_dec200_dev_create(dec200);

	if (dec200->hwdata->devtype == DEC200_DEC) {
		/* enable AXI clocks for DEC200 */
		dec200->axi_clk = devm_clk_get(&pdev->dev,
						DEC200_AXI_CLOCK_NAME);
		if (IS_ERR(dec200->axi_clk)) {
			dec200->init_status = DEC200_INIT_ERR_CFG;
			dev_err(&pdev->dev,
				"DEC200: could not get axi clock\n");
			return PTR_ERR(dec200->axi_clk);
		}
		clk_prepare_enable(dec200->axi_clk);
	} else if (dec200->hwdata->devtype == DEC200_ENC) {
		/* enable SYS3 clocks for DEC200 */
		dec200->sys3_clk = devm_clk_get(&pdev->dev,
						DEC200_SYS3_CLOCK_NAME);
		if (IS_ERR(dec200->sys3_clk)) {
			dec200->init_status = DEC200_INIT_ERR_CFG;
			dev_err(&pdev->dev,
				"DEC200: could not get sys3 clock\n");
			return PTR_ERR(dec200->sys3_clk);
		}
		clk_prepare_enable(dec200->sys3_clk);
	}

	/* enable SYS6 clocks for DEC200 */
	dec200->sys6_clk = devm_clk_get(&pdev->dev,
					DEC200_SYS6_CLOCK_NAME);
	if (IS_ERR(dec200->sys6_clk)) {
		dec200->init_status = DEC200_INIT_ERR_CFG;
		dev_err(&pdev->dev,
			"DEC200: could not get sys6 clock\n");
		return PTR_ERR(dec200->sys6_clk);
	}
	clk_prepare_enable(dec200->sys6_clk);

	fsl_dec200_init(dec200);

	fsl_dec200_debugfs_init(dec200);

	return 0;
}

int fsl_dec200_remove(struct platform_device *pdev)
{
	struct dec200_struct *dec200 = platform_get_drvdata(pdev);

	fsl_dec200_debugfs_remove(dec200);

	cdev_del(dec200->cdev);
	device_destroy(dec200->class, dec200->devno);
	class_destroy(dec200->class);
	unregister_chrdev_region(dec200->devno, 1);

	pm_runtime_put_sync(&dec200->pdev->dev);
	pm_runtime_disable(&dec200->pdev->dev);

	return 0;
}

static struct platform_driver fsl_dec200_driver = {
	.probe = fsl_dec200_probe,
	.remove = fsl_dec200_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = fsl_dec200_dt_ids,
	},
	.id_table = fsl_dec200_devtype,
};

module_platform_driver(fsl_dec200_driver);
