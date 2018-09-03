/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/errno.h>
#include <linux/io.h>

static ssize_t s32_regaccess_read(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buffer, size_t count)
{
	int access_width;
	uint32_t register_addr;
	void __iomem *iomem_register_addr;

	if (sscanf(buffer, "%x %d",
		   &register_addr,
		   &access_width) != 2) {
		pr_err("Invalid argument format!\n");
		pr_err("Usage: '0x<address> <access_width>'\n");
		return count;
	}

	iomem_register_addr = ioremap(register_addr, 8);
	if (!iomem_register_addr) {
		pr_err("Failed to map memory!\n");
		return count;
	}

	switch (access_width) {
	case 8:
		pr_info("Contents of 0x%08x: 0x%08x\n",
			register_addr,
			ioread8(iomem_register_addr));
		break;
	case 16:
		pr_info("Contents of 0x%08x: 0x%08x\n",
			register_addr,
			ioread16(iomem_register_addr));
		break;
	case 32:
		pr_info("Contents of 0x%08x: 0x%08x\n",
			register_addr,
			ioread32(iomem_register_addr));
		break;
	default:
		pr_err("Invalid access width! Use 8,16,32 or 64\n");
		break;
	}

	iounmap(iomem_register_addr);
	return count;
}

static ssize_t s32_regaccess_write(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buffer, size_t count)
{
	int access_width;
	uint32_t register_addr;
	uint32_t register_data;
	void __iomem *iomem_register_addr;

	if (sscanf(buffer, "%x %d %x",
		   &register_addr,
		   &access_width,
		   &register_data) != 3) {
		pr_err("Invalid argument format!\n");
		pr_err("Usage: '0x<address> <access_width> 0x<value>'\n");
		return count;
	}

	iomem_register_addr = ioremap(register_addr, 8);
	if (!iomem_register_addr) {
		pr_err("Failed to map memory!\n");
		return count;
	}

	switch (access_width) {
	case 8:
		iowrite8((uint8_t)register_data, iomem_register_addr);
		pr_info("Contents of 0x%08x: 0x%08x\n",
			register_addr,
			ioread8(iomem_register_addr));
		break;
	case 16:
		iowrite16((uint16_t)register_data, iomem_register_addr);
		pr_info("Contents of 0x%08x: 0x%08x\n",
			register_addr,
			ioread16(iomem_register_addr));
		break;
	case 32:
		iowrite32((uint32_t)register_data, iomem_register_addr);
		pr_info("Contents of 0x%08x: 0x%08x\n",
			register_addr,
			ioread32(iomem_register_addr));
		break;
	default:
		pr_err("Invalid access width! Use 8,16,32 or 64\n");
		break;
	}

	iounmap(iomem_register_addr);
	return count;
}

static struct kobj_attribute s32_regaccess_read_attr =
__ATTR(read, 0220, NULL, s32_regaccess_read);

static struct kobj_attribute s32_regaccess_write_attr =
__ATTR(write, 0220, NULL, s32_regaccess_write);

static struct kobject *s32_regaccess_ko;

static int __init s32_regaccess_init(void)
{
	s32_regaccess_ko = kobject_create_and_add("s32_regaccess", kernel_kobj);
	if (!s32_regaccess_ko)
		goto failure_kobject_create;

	if (sysfs_create_file(s32_regaccess_ko, &s32_regaccess_read_attr.attr))
		goto failure_sysfs_read;

	if (sysfs_create_file(s32_regaccess_ko, &s32_regaccess_write_attr.attr))
		goto failure_sysfs_write;

	return 0;

failure_sysfs_write:
	sysfs_remove_file(s32_regaccess_ko, &s32_regaccess_read_attr.attr);
failure_sysfs_read:
	kobject_put(s32_regaccess_ko);
failure_kobject_create:
	return -ENOENT;
}

static void __exit s32_regaccess_exit(void)
{
	sysfs_remove_file(s32_regaccess_ko, &s32_regaccess_read_attr.attr);
	sysfs_remove_file(s32_regaccess_ko, &s32_regaccess_write_attr.attr);
	kobject_put(s32_regaccess_ko);
}

module_init(s32_regaccess_init);
module_exit(s32_regaccess_exit);
