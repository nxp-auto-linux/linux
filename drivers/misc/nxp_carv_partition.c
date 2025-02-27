// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * NXP CAR-V Partition Driver
 *
 * This file contains the partition initialization support for the CAR-V
 * accelerator.
 *
 * Copyright 2025 NXP
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define DRIVER_NAME	"carv-partition"
#define DRIVER_VERSION	"1.0"

#define KB 1024

#define CARV_MAX_NUM_QUARTILES	2u
#define CARV_MAX_NUM_USERS	4u

#define CARV_DEFAULT_STACK_SIZE	64

#define CARV_USER_REGS_OFFSET	0x10000
/* CAR-V Registers */

#define CARV_GL_CR	0x00000000	/* Global Configuration */
#define CARV_GL_AHBER	0x00000010	/* Global AHB-Lite Enable */
#define CARV_GL_IMR	0x00000020	/* Global Functional Interrupt Mask */
#define CARV_GL_FMR	0x00000024	/* Global Fault Interrupt Mask */
#define CARV_GL_FAINR	0x00000028	/* Global Fault Injection */
#define CARV_GL_PCR	0x00000080	/* Global Isolated Partition Control */
#define CARV_GL_UCR	0x00000084	/* Global User Count */
#define CARV_GL_AHBCR	0x0000008C	/* Global AHB-Lite Control */

/* CAR-V Registers Accessors */

/* Enable Quartile */
#define CARV_GL_CR_ENA_MASK			BIT(0)
#define CARV_GL_CR_ENA_SET(val)			FIELD_PREP(CARV_GL_CR_ENA_MASK, val)

/* Enable MCT Clock */
#define CARV_GL_CR_CLOCK_MASK			BIT(1)
#define CARV_GL_CR_CLOCK_SET(val)		FIELD_PREP(CARV_GL_CR_CLOCK_MASK, val)

/* Lock Out Write Access */
#define CARV_GL_CR_LOCK_MASK			BIT(2)
#define CARV_GL_CR_LOCK_SET(val)		FIELD_PREP(CARV_GL_CR_LOCK_MASK, val)

/* ECC Multibit Error Recovery Enable */
#define CARV_GL_CR_EMBER_EN_MASK		BIT(3)
#define CARV_GL_CR_EMBER_EN_SET(val)		FIELD_PREP(CARV_GL_CR_EMBER_EN_MASK, val)

/* Coherency and FSM Safety Check Enable */
#define CARV_GL_CR_COH_FSM_CHK_EN_MASK		BIT(4)
#define CARV_GL_CR_COH_FSM_CHK_EN_SET(val)	FIELD_PREP(CARV_GL_CR_COH_FSM_CHK_EN_MASK, val)

/* Global external interrupt handling enable */
#define CARV_GL_CR_IP_VID_MASK			GENMASK(31, 16)
#define CARV_GL_CR_IP_VID_SET(val)		FIELD_PREP(CARV_GL_CR_IP_VID_MASK, val)

/* Enable AHB-Lite Manager Port */
#define CARV_GL_AHBER_ENA_MASK			BIT(0)
#define CARV_GL_AHBER_ENA_SET(val)		FIELD_PREP(CARV_GL_AHBER_ENA_MASK, val)

/* Global Enable Mask for Functional Interrupts */
#define CARV_GL_IMR_MASK_MASK			GENMASK(15, 0)
#define CARV_GL_IMR_MASK_SET(val)		FIELD_PREP(CARV_GL_IMR_MASK_MASK, val)

/* Global Enable Mask for Fault Interrupts */
#define CARV_GL_FMR_MASK_MASK			GENMASK(15, 0)
#define CARV_GL_FMR_MASK_SET(val)		FIELD_PREP(CARV_GL_FMR_MASK_MASK, val)

/* Fault Injection */
#define CARV_GL_FAINR_FAULT_INJ_MASK		GENMASK(12, 0)
#define CARV_GL_FAINR_FAULT_INJ_SET(val)	FIELD_PREP(CARV_GL_FAINR_FAULT_INJ_MASK, val)

/* Programmed VM by Software */
#define CARV_GL_FAINR_VM_INJ_MASK		GENMASK(31, 30)
#define CARV_GL_FAINR_VM_INJ_SET(val)		FIELD_PREP(CARV_GL_FAINR_VM_INJ_MASK, val)

/* Isolated Partition */
#define CARV_GL_PCR_ISOPART_MASK		GENMASK(1, 0)
#define CARV_GL_PCR_ISOPART_SET(val)		FIELD_PREP(CARV_GL_PCR_ISOPART_MASK, val)

/* Quartile Position Within an Isolated Partition */
#define CARV_GL_PCR_ISOPOS_MASK			GENMASK(3, 2)
#define CARV_GL_PCR_ISOPOS_SET(val)		FIELD_PREP(CARV_GL_PCR_ISOPOS_MASK, val)

/* MCT Reset */
#define CARV_GL_PCR_MRST_MASK			GENMASK(5, 4)
#define CARV_GL_PCR_MRST_SET(val)		FIELD_PREP(CARV_GL_PCR_MRST_MASK, val)

/* Stack Size */
#define CARV_GL_PCR_STKSZ_MASK			GENMASK(15, 13)
#define CARV_GL_PCR_STKSZ_SET(val)		FIELD_PREP(CARV_GL_PCR_STKSZ_MASK, val)

/* Globally Configuration of the Number of Users */
#define CARV_GL_UCR_USERCNT_MASK		GENMASK(1, 0)
#define CARV_GL_UCR_USERCNT_SET(val)		FIELD_PREP(CARV_GL_UCR_USERCNT_MASK, val)

/* Global Enable for AHB-Lite Port */
#define CARV_GL_AHBCR_AHBEN_MASK		BIT(0)
#define CARV_GL_AHBCR_AHBEN_SET(val)		FIELD_PREP(CARV_GL_AHBCR_AHBEN_MASK, val)

/* AHB-Lite Port Selection */
#define CARV_GL_AHBCR_AHBSEL_MASK		GENMASK(5, 4)
#define CARV_GL_AHBCR_AHBSEL_SET(val)		FIELD_PREP(CARV_GL_AHBCR_AHBSEL_MASK, val)

/*
 * enum carv_global_interrupt - CAR-V global interrupts
 * @CARV_GL_INT_DONE: Done Interrupt
 * @CARV_GL_INT_ERR: Non-Fatal Error Interrupt
 * @CARV_GL_INT_UPDATE: Update Interrupt
 * @CARV_GL_INT_ACTIV: Active Interrupt
 * @CARV_GL_F_INT_MAP: Internal-System Address Translation Mapping Interrupt
 * @CARV_GL_F_INT_AXIRE: AXI Read Response Interrupt
 * @CARV_GL_F_INT_AXIWE: AXI Write Response Interrupt
 * @CARV_GL_F_INT_AXIRP: AXI Read Data Parity Interrupt
 * @CARV_GL_F_INT_AHB: AHB Access Response Interrupt
 * @CARV_GL_F_INT_CORE: Core-Fault Interrupt
 * @CARV_GL_F_INT_L1: L1 Cache Interrupt
 * @CARV_GL_F_INT_L2: L2 Cache Interrupt
 * @CARV_GL_F_INT_BUS: Internal Bus Parity Interrupt
 */
enum carv_global_interrupt {
	CARV_GL_INT_DONE	= 0,
	CARV_GL_INT_ERR		= 1,
	CARV_GL_INT_UPDATE	= 2,
	CARV_GL_INT_ACTIV	= 4,
	CARV_GL_F_INT_MAP	= 6,
	CARV_GL_F_INT_AXIRE	= 7,
	CARV_GL_F_INT_AXIWE	= 8,
	CARV_GL_F_INT_AXIRP	= 9,
	CARV_GL_F_INT_AHB	= 11,
	CARV_GL_F_INT_CORE	= 12,
	CARV_GL_F_INT_L1	= 13,
	CARV_GL_F_INT_L2	= 14,
	CARV_GL_F_INT_BUS	= 15,
};

/*
 * struct carv_partition_drvdata - CAR-V partition driver private data
 * @dev: CAR-V partition device
 * @num_quartiles: Number of quartiles of the CAR-V partition
 * @num_users: Number of users of the CAR-V partition
 * @ahb_user_id: AHB-Lite User ID of the CAR-V partition
 * @stack_size: Stack size of the CAR-V partition
 * @regs: Quartile registers of the CAR-V partition
 * @has_rproc_child: It tells if CAR-V partition has rproc child user
 */
struct carv_partition_drvdata {
	struct device *dev;
	struct miscdevice misc_dev;
	u32 num_quartiles;
	u32 num_users;
	s32 ahb_user_id;
	u32 stack_size;
	u8 __iomem **regs;
	bool has_rproc_child;
};

static int carv_partitions_count;

/*
 * carv_config_partition - CAR-V partition configuration
 * @dev: CAR-V partition device
 */
static void carv_config_partition(struct device *dev)
{
	struct carv_partition_drvdata *priv = dev_get_drvdata(dev);

	for (int i = priv->num_quartiles - 1; i >= 0; i--) {
		writel(CARV_GL_PCR_ISOPOS_SET(i) |
		       CARV_GL_PCR_ISOPART_SET(priv->num_quartiles - 1) |
		       CARV_GL_PCR_STKSZ_SET(ilog2(priv->stack_size >> 3)),
		       priv->regs[i] + CARV_GL_PCR);
		writel(CARV_GL_UCR_USERCNT_SET(priv->num_users - 1), priv->regs[i] + CARV_GL_UCR);
	}
}

/*
 * carv_enable_partition - CAR-V enable partition
 * @dev: CAR-V partition device
 */
static void carv_enable_partition(struct device *dev)
{
	struct carv_partition_drvdata *priv = dev_get_drvdata(dev);

	for (int i = priv->num_quartiles - 1; i >= 0; i--)
		writel(CARV_GL_CR_CLOCK_SET(1) | CARV_GL_CR_ENA_SET(1),
		       priv->regs[i] + CARV_GL_CR);
}

/*
 * carv_disable_partition - CAR-V disable partition
 * @dev: CAR-V partition device
 */
static void carv_disable_partition(struct device *dev)
{
	struct carv_partition_drvdata *priv = dev_get_drvdata(dev);

	for (int i = priv->num_quartiles - 1; i >= 0; i--)
		writel(0, priv->regs[i] + CARV_GL_CR);
}

/*
 * carv_config_ahb_lite_master_port - CAR-V AHB-Lite configuration
 * @dev: CAR-V partition device
 */
static void carv_config_ahb_lite_master_port(struct device *dev)
{
	struct carv_partition_drvdata *priv = dev_get_drvdata(dev);

	if (priv->ahb_user_id != -1) {
		writel(CARV_GL_AHBER_ENA_SET(1), priv->regs[0] + CARV_GL_AHBER);
		writel(CARV_GL_AHBCR_AHBEN_SET(1) | CARV_GL_AHBCR_AHBSEL_SET(priv->ahb_user_id),
		       priv->regs[0] + CARV_GL_AHBCR);
	} else {
		writel(0, priv->regs[0] + CARV_GL_AHBER);
		writel(0, priv->regs[0] + CARV_GL_AHBCR);
	}
}

/*
 * carv_config_global_interrupts - CAR-V global interrupts configuration
 * @dev: CAR-V partition device
 * @fn_ints: CAR-V functional interrupts
 * @fn_mask: CAR-V functional interrupts mask
 * @fault_ints: CAR-V fault interrupts
 * @fault_mask: CAR-V fault interrupts mask
 */
static void carv_config_global_interrupts(struct device *dev,
					  u32 fn_ints, u32 fn_mask,
					  u32 fault_ints, u32 fault_mask)
{
	struct carv_partition_drvdata *priv = dev_get_drvdata(dev);

	writel((readl(priv->regs[0] + CARV_GL_IMR) & ~fn_mask) |
	       (fn_ints & fn_mask), priv->regs[0] + CARV_GL_IMR);

	writel((readl(priv->regs[0] + CARV_GL_FMR) & ~fault_mask) |
	       (fault_ints & fault_mask), priv->regs[0] + CARV_GL_FMR);
}

/*
 * carv_init_partition - CAR-V partition initialization
 * @dev: CAR-V partition device
 */
static void carv_init_partition(struct device *dev)
{
	struct carv_partition_drvdata *priv = dev_get_drvdata(dev);
	u32 irqs;

	dev_info(dev, "%s: nQTs: %u, nUsers: %u, ahbUser: %d, stack: %u KB", __func__,
		 priv->num_quartiles, priv->num_users, priv->ahb_user_id, priv->stack_size);

	carv_disable_partition(dev);
	carv_config_partition(dev);
	carv_config_ahb_lite_master_port(dev);

	irqs = BIT(CARV_GL_INT_ACTIV) | BIT(CARV_GL_INT_UPDATE) | BIT(CARV_GL_INT_ERR) |
	       BIT(CARV_GL_INT_DONE) | BIT(CARV_GL_F_INT_BUS) | BIT(CARV_GL_F_INT_L2) |
	       BIT(CARV_GL_F_INT_L1) | BIT(CARV_GL_F_INT_CORE) | BIT(CARV_GL_F_INT_AHB) |
	       BIT(CARV_GL_F_INT_AXIRP) | BIT(CARV_GL_F_INT_AXIWE) |
	       BIT(CARV_GL_F_INT_AXIRE) | BIT(CARV_GL_F_INT_MAP);

	carv_config_global_interrupts(dev, irqs, ~0U, 0U, ~0U);
	carv_enable_partition(dev);
}

/*
 * carv_deinit_partition - CAR-V partition deinitialization
 * @dev: CAR-V partition device
 */
static void carv_deinit_partition(struct device *dev)
{
	struct carv_partition_drvdata *priv = dev_get_drvdata(dev);

	dev_info(dev, "%s: nQTs: %u, nUsers: %u", __func__,
		 priv->num_quartiles, priv->num_users);

	carv_disable_partition(dev);
}

/*
 * carv_partition_dt_part_read - Read fields from device tree for the isolated partition
 * @dev: CAR-V partition device
 */
static int carv_partition_dt_part_read(struct device *dev)
{
	struct carv_partition_drvdata *priv = dev_get_drvdata(dev);
	int ret = 0;

	priv->num_quartiles = of_address_count(dev->of_node);
	if (priv->num_quartiles == 0 || priv->num_quartiles > CARV_MAX_NUM_QUARTILES) {
		dev_err(dev, "'reg' property is empty\n");
		return -EINVAL;
	}

	priv->num_users = of_get_child_count(dev->of_node);
	if (priv->num_users == 0 || priv->num_users > CARV_MAX_NUM_USERS) {
		dev_err(dev, "num_users must be in range [1,%u]\n", CARV_MAX_NUM_USERS);
		return -EINVAL;
	}

	/* Alloc registers based on number of quartiles of the partition */
	priv->regs = devm_kzalloc(dev, priv->num_quartiles * sizeof(*priv->regs), GFP_KERNEL);
	if (!priv->regs)
		return -ENOMEM;

	for (int i = 0; i < priv->num_quartiles; i++) {
		/* Request the resource and map the MMIO */
		priv->regs[i] = devm_of_iomap(dev, dev->of_node, i, NULL);
		if (IS_ERR(priv->regs[i])) {
			dev_err(dev, "failed to map MMIO for quartile: %lu\n",
				PTR_ERR(priv->regs[i]));
			ret = PTR_ERR(priv->regs[i]);
			break;
		}
	}

	if (ret)
		return ret;

	/* Read stack size property from device tree */
	ret = of_property_read_u32(dev->of_node, "nxp,stack-size", &priv->stack_size);
	if (ret) {
		dev_err(dev, "defaulting stack size to %d KB\n", CARV_DEFAULT_STACK_SIZE);
		priv->stack_size = CARV_DEFAULT_STACK_SIZE;
	}

	return 0;
}

/*
 * carv_partition_set_ahb_user_id - Set AHB user id based on child nodes compatibles
 * @dev: CAR-V partition device
 */
static int carv_partition_set_ahb_user_id(struct device *dev)
{
	struct carv_partition_drvdata *priv = dev_get_drvdata(dev);
	struct resource res, child_res;
	struct device_node *child;
	int ret = 0;

	priv->ahb_user_id = -1;

	if (priv->has_rproc_child) {
		priv->ahb_user_id = 0;
		return 0;
	}

	for_each_child_of_node(dev->of_node, child) {
		if (of_device_is_compatible(child, "nxp,carv-ahb-uio")) {
			ret = of_address_to_resource(dev->of_node, 0, &res);
			if (ret < 0) {
				dev_err(dev, "failed to get resource\n");
				break;
			}

			ret = of_address_to_resource(child, 0, &child_res);
			if (ret < 0) {
				dev_err(dev, "failed to get child resource\n");
				break;
			}

			priv->ahb_user_id =
				((child_res.start - res.start) / CARV_USER_REGS_OFFSET) - 1;
			break;
		}
	}

	return ret;
}

/*
 * carv_partition_misc_register - Register misc device for CAR-V partition
 * @dev: CAR-V partition device
 */
static int carv_partition_misc_register(struct device *dev)
{
	struct carv_partition_drvdata *priv = dev_get_drvdata(dev);
	char dev_name[32];

	snprintf(dev_name, sizeof(dev_name), "carv_partition_%d", carv_partitions_count);
	priv->misc_dev.name = devm_kstrdup(priv->dev, dev_name, GFP_KERNEL);

	priv->misc_dev.minor = MISC_DYNAMIC_MINOR;
	priv->misc_dev.fops = NULL;

	return misc_register(&priv->misc_dev);
}

/*
 * stack_size_show - Show the partition's stack size in sysfs
 * @dev: The device associated with the sysfs attribute
 * @attr: The sysfs attribute permissions
 * @buf: The buffer where the stack size will be written to
 *
 * Return: The number of bytes written to the buffer, or a negative error code on failure
 */
static ssize_t stack_size_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct miscdevice *misc_dev = dev_get_drvdata(dev);
	struct carv_partition_drvdata *priv =
		container_of(misc_dev, struct carv_partition_drvdata, misc_dev);

	return sprintf(buf, "%u\n", priv->stack_size * KB);
}

static DEVICE_ATTR_RO(stack_size);

/*
 * users_show - Show the partition's users in sysfs
 * @dev: The device associated with the sysfs attribute
 * @attr: The sysfs attribute permissions
 * @buf: The buffer where the users will be written to
 *
 * Return: The number of bytes written to the buffer, or a negative error code on failure
 */
static ssize_t users_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct miscdevice *misc_dev = dev_get_drvdata(dev);
	struct carv_partition_drvdata *priv =
		container_of(misc_dev, struct carv_partition_drvdata, misc_dev);
	struct device_node *child;
	struct resource res;
	int ret = 0;
	int len = 0;

	for_each_child_of_node(priv->dev->of_node, child) {
		ret = of_address_to_resource(child, 0, &res);
		if (ret < 0) {
			dev_err(dev, "failed to get child resource\n");
			break;
		}
		len += sprintf(buf + len, "%pa\n", &res.start);
	}

	return len;
}

static DEVICE_ATTR_RO(users);

static int carv_partition_probe(struct platform_device *pdev)
{
	struct carv_partition_drvdata *priv;
	struct device_node *child;
	struct device *dev;
	int ret;

	dev = &pdev->dev;

	/* Alloc carv_partition_drvdata private data */
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);

	for_each_child_of_node(dev->of_node, child) {
		if (of_device_is_compatible(child, "nxp,carvnet-rproc")) {
			priv->has_rproc_child = true;
			break;
		}
	}

	/* Read device tree fields for partition */
	ret = carv_partition_dt_part_read(dev);
	if (ret)
		return ret;

	/* Set AHB user id based on child nodes compatibles */
	ret = carv_partition_set_ahb_user_id(dev);
	if (ret)
		return ret;

	/* Initialize CAR-V partition */
	carv_init_partition(dev);

	/* Register misc device for partition */
	if (!priv->has_rproc_child) {
		ret = carv_partition_misc_register(dev);
		if (ret) {
			dev_err(dev, "failed to register misc device\n");
			return ret;
		}

		ret = device_create_file(priv->misc_dev.this_device, &dev_attr_stack_size);
		if (ret) {
			dev_err(dev, "failed to create sysfs attribute for stack-size\n");
			return ret;
		}

		ret = device_create_file(priv->misc_dev.this_device, &dev_attr_users);
		if (ret) {
			dev_err(dev, "failed to create sysfs attribute for users\n");
			return ret;
		}
	}
	carv_partitions_count++;

	/* Populate users from device tree data */
	ret = devm_of_platform_populate(dev);
	if (ret) {
		dev_err(dev, "failed to populate users\n");
		return ret;
	}

	dev_dbg(dev, "Successfully initialized CAR-V partition\n");

	return 0;
}

static void carv_partition_remove(struct platform_device *pdev)
{
	struct carv_partition_drvdata *priv = platform_get_drvdata(pdev);

	/* Deregister misc device for partition */
	if (!priv->has_rproc_child)
		misc_deregister(&priv->misc_dev);

	/* Deinit CAR-V partition */
	carv_deinit_partition(priv->dev);
}

static const struct of_device_id carv_partition_of_match[] = {
	{ .compatible = "nxp,carv-partition", },
	{}
};
MODULE_DEVICE_TABLE(of, carv_partition_of_match);

static struct platform_driver carv_partition_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table	= carv_partition_of_match,
	},
	.probe = carv_partition_probe,
	.remove_new = carv_partition_remove,
};

module_platform_driver(carv_partition_driver);

MODULE_AUTHOR("Andrei-Edward Popa <andrei.popa_1@nxp.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("NXP CAR-V Partition Driver");
