// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SIUL2 GPIO support.
 *
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2018-2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * later as publishhed by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/gpio/driver.h>
#include <asm-generic/bug.h>
#include <linux/bitmap.h>

/* DMA/Interrupt Status Flag Register */
#define SIUL2_DISR0			0x10
/* DMA/Interrupt Request Enable Register */
#define SIUL2_DIRER0			0x18
/* DMA/Interrupt Request Select Register */
#define SIUL2_DIRSR0			0x20
/* Interrupt Rising-Edge Event Enable Register */
#define SIUL2_IREER0			0x28
/* Interrupt Falling-Edge Event Enable Register */
#define SIUL2_IFEER0			0x30

/* Device tree ranges */
#define SIUL2_GPIO_OUTPUT_RANGE		0
#define SIUL2_GPIO_INPUT_RANGE		1

/* Reserved for Pad Data Input/Output Registers */
#define SIUL2_GPIO_RESERVED_RANGE1	2
#define SIUL2_GPIO_RESERVED_RANGE2	3

/* Only for chips with interrupt controller */
#define SIUL2_GPIO_INTERRUPTS_RANGE	4

#define SIUL2_GPIO_PAD_SIZE		32

/**
 * enum gpio_dir - GPIO pin mode
 */
enum gpio_dir {
	IN, OUT
};

/**
 * struct siul2_gpio_dev - describes a group of GPIO pins
 * @pdev: the platform device
 * @ipads: input pads address
 * @opads: output pads address
 * @irq_base: the base address of EIRQ registers
 * @eirq_npins: number of EIRQ pins
 * @pin_dir_bitmap: bitmap with pin directions
 * @gc: the GPIO chip
 * @lock: mutual access to chip registers
 *
 * @see gpio_dir
 */
struct siul2_gpio_dev {
	struct platform_device *pdev;
	void __iomem *ipads;
	void __iomem *opads;

	void __iomem *irq_base;
	unsigned int eirq_npins;

	unsigned long *pin_dir_bitmap;

	struct gpio_chip gc;
	struct irq_chip irqchip;
	spinlock_t lock;
};

static inline int siul2_get_gpio_pinspec(
	struct platform_device *pdev,
	struct of_phandle_args *pinspec,
	unsigned int range_index)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = of_parse_phandle_with_fixed_args(np, "gpio-ranges", 3,
						   range_index, pinspec);
	if (ret)
		return -EINVAL;

	return 0;
}

static inline void gpio_set_direction(struct siul2_gpio_dev *dev, int gpio,
						enum gpio_dir dir)
{
	if (dir == IN)
		bitmap_clear(dev->pin_dir_bitmap, gpio, 1);
	else
		bitmap_set(dev->pin_dir_bitmap, gpio, 1);
}

static inline enum gpio_dir gpio_get_direction(struct siul2_gpio_dev *dev,
							int gpio)
{
	return test_bit(gpio, dev->pin_dir_bitmap) ? OUT : IN;
}

static inline struct siul2_gpio_dev *to_siul2_gpio_dev(struct gpio_chip *chip)
{
	return container_of(chip, struct siul2_gpio_dev, gc);
}

static int siul2_gpio_dir_in(struct gpio_chip *chip, unsigned int gpio)
{
	int ret = 0;
	struct siul2_gpio_dev *gpio_dev;

	ret = pinctrl_gpio_direction_input(chip->base + gpio);
	if (ret)
		return ret;

	gpio_dev = to_siul2_gpio_dev(chip);
	gpio_set_direction(gpio_dev, gpio, IN);

	return ret;
}

static int siul2_gpio_dir_out(struct gpio_chip *chip, unsigned int gpio,
			      int val)
{
	int ret = 0;
	struct siul2_gpio_dev *gpio_dev;

	ret = pinctrl_gpio_direction_output(chip->base + gpio);
	if (ret)
		return ret;

	gpio_dev = to_siul2_gpio_dev(chip);
	chip->set(chip, gpio, val);
	gpio_set_direction(gpio_dev, gpio, OUT);

	return ret;
}

static int siul2_gpio_request(struct gpio_chip *chip, unsigned int gpio_pin)
{
	return pinctrl_gpio_request(chip->base + gpio_pin);
}

static void siul2_gpio_free(struct gpio_chip *chip, unsigned int offset)
{
	pinctrl_gpio_free(chip->base + offset);
}

static int siul2_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	int eirq = d->hwirq;
	unsigned long flags;
	unsigned int irq_type = type & IRQ_TYPE_SENSE_MASK;
	int ret;
	u32 ireer0_val;
	u32 ifeer0_val;

	if (eirq < 0 || eirq >= gpio_dev->eirq_npins)
		return -EINVAL;

	ret = pinctrl_gpio_direction_input(gc->base + eirq);
	if (ret) {
		dev_err(gc->parent, "Failed to configure %d pin as input pin\n",
			eirq);
		return ret;
	}

	/* SIUL2 GPIO doesn't support level triggering */
	if ((irq_type & IRQ_TYPE_LEVEL_HIGH)
	    || (irq_type & IRQ_TYPE_LEVEL_LOW)) {
		dev_err(gc->parent,
			"Invalid SIUL2 GPIO irq type 0x%x\n", type);
		return -EINVAL;
	}

	spin_lock_irqsave(&gpio_dev->lock, flags);

	ireer0_val = readl(gpio_dev->irq_base + SIUL2_IREER0);
	ifeer0_val = readl(gpio_dev->irq_base + SIUL2_IFEER0);

	if (irq_type & IRQ_TYPE_EDGE_RISING)
		ireer0_val |= BIT(eirq);
	else
		ireer0_val &= ~BIT(eirq);

	if (irq_type & IRQ_TYPE_EDGE_FALLING)
		ifeer0_val |= BIT(eirq);
	else
		ifeer0_val &= ~BIT(eirq);

	writel(ireer0_val, gpio_dev->irq_base + SIUL2_IREER0);
	writel(ifeer0_val, gpio_dev->irq_base + SIUL2_IFEER0);

	spin_unlock_irqrestore(&gpio_dev->lock, flags);

	return 0;
}

static void siul2_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	unsigned int pin;
	unsigned long disr0_val;

	chained_irq_enter(irq_chip, desc);

	/* Go through the entire GPIO bank and handle all interrupts */
	disr0_val = readl(gpio_dev->irq_base + SIUL2_DISR0);

	for_each_set_bit(pin, &disr0_val, BITS_PER_BYTE * sizeof(disr0_val)) {
		int child_irq = irq_find_mapping(gc->irq.domain, pin);

		/*
		 * Clear the interrupt before invoking the
		 * handler, so we do not leave any window
		 */
		writel(BIT(pin), gpio_dev->irq_base + SIUL2_DISR0);

		generic_handle_irq(child_irq);
	}

	chained_irq_exit(irq_chip, desc);
}

static void siul2_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	int eirq = data->hwirq;
	unsigned long flags;
	u32 direr0_val;
	u32 disr0_val;

	if (eirq < 0 || eirq >= gpio_dev->eirq_npins)
		return;

	spin_lock_irqsave(&gpio_dev->lock, flags);

	direr0_val = readl(gpio_dev->irq_base + SIUL2_DIRER0);

	/* Disable interrupt */
	direr0_val &= ~BIT(eirq);
	/* Clear status flag */
	disr0_val = BIT(eirq);

	writel(direr0_val, gpio_dev->irq_base + SIUL2_DIRER0);
	writel(disr0_val, gpio_dev->irq_base + SIUL2_DISR0);

	/* Enable Interrupt */
	direr0_val |= BIT(eirq);

	writel(direr0_val, gpio_dev->irq_base + SIUL2_DIRER0);

	spin_unlock_irqrestore(&gpio_dev->lock, flags);
}

static void siul2_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	int eirq = data->hwirq;
	unsigned long flags;
	u32 direr0_val;
	u32 disr0_val;
	int err;

	if (eirq < 0 || eirq >= gpio_dev->eirq_npins)
		return;

	err = siul2_gpio_irq_set_type(data, IRQ_TYPE_NONE);
	if (err)
		return;

	spin_lock_irqsave(&gpio_dev->lock, flags);

	direr0_val = readl(gpio_dev->irq_base + SIUL2_DIRER0);

	/* Disable interrupt */
	direr0_val &= ~BIT(eirq);
	/* Clean status flag */
	disr0_val = BIT(eirq);

	writel(direr0_val, gpio_dev->irq_base + SIUL2_DIRER0);
	writel(disr0_val, gpio_dev->irq_base + SIUL2_DISR0);

	spin_unlock_irqrestore(&gpio_dev->lock, flags);
}

static int siul2_irq_setup(struct platform_device *pdev,
			  struct siul2_gpio_dev *gpio_dev)
{
	int err;
	const void *intspec;
	int intlen;
	struct resource *iores;
	struct of_phandle_args pinspec;
	int irq;

	/* Skip gpio node without interrupts */
	intspec = of_get_property(pdev->dev.of_node, "interrupts", &intlen);
	if (!intspec)
		return 0;

	/* Interrupt controller */
	iores = platform_get_resource(pdev, IORESOURCE_MEM,
				      SIUL2_GPIO_INTERRUPTS_RANGE);
	gpio_dev->irq_base = devm_ioremap_resource(&pdev->dev, iores);
	if (IS_ERR(gpio_dev->irq_base))
		return PTR_ERR(gpio_dev->irq_base);

	/* EIRQ pins */
	err = siul2_get_gpio_pinspec(pdev, &pinspec, 1);
	if (err) {
		dev_err(&pdev->dev,
			"unable to get pinspec from device tree\n");
		return -EIO;
	}

	/* Request IRQ */
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "failed to get irq resource.\n");
		return irq;
	}

	/* Number of pins */
	gpio_dev->eirq_npins = pinspec.args[2];

	/* Disable the interrupts and clear the status */
	writel(0, gpio_dev->irq_base + SIUL2_DIRER0);
	writel(~0, gpio_dev->irq_base + SIUL2_DISR0);

	/* Select interrupts by default */
	writel(0, gpio_dev->irq_base + SIUL2_DIRSR0);

	/* Disable rising-edge events */
	writel(0, gpio_dev->irq_base + SIUL2_IREER0);
	/* Disable falling-edge events */
	writel(0, gpio_dev->irq_base + SIUL2_IFEER0);

	/* Setup irq domain on top of the generic chip. */
	err = gpiochip_irqchip_add(&gpio_dev->gc,
				   &gpio_dev->irqchip,
				   0,
				   handle_simple_irq,
				   IRQ_TYPE_NONE);
	if (err) {
		dev_err(&pdev->dev, "could not connect irqchip to gpiochip\n");
		return err;
	}

	gpiochip_set_chained_irqchip(&gpio_dev->gc,
				     &gpio_dev->irqchip,
				     irq,
				     siul2_gpio_irq_handler);

	return 0;
}

static int siul2_gpio_remove(struct platform_device *pdev)
{
	struct siul2_gpio_dev *gpio_dev = platform_get_drvdata(pdev);
	int err = 0;

	if (!gpio_dev)
		return -EINVAL;

	gpiochip_remove(&gpio_dev->gc);
	return err;
}


static const struct of_device_id siul2_gpio_dt_ids[] = {
	{ .compatible = "fsl,s32v234-siul2-gpio" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, siul2_gpio_dt_ids);

static inline unsigned int siul2_pin2mask(int pin, struct siul2_gpio_dev *gpio_dev)
{
	unsigned int pad_size;

	pad_size = SIUL2_GPIO_PAD_SIZE;
	pin %= pad_size;
	return (1 << (pad_size - 1 - pin));
}

static inline unsigned int siul2_pin2pad(int pin, struct siul2_gpio_dev *gpio_dev)
{
	unsigned int pad_size;

	pad_size = SIUL2_GPIO_PAD_SIZE;
	return pin / pad_size;
}

static inline void __iomem *siul2_get_pad_addr(void __iomem *pads, unsigned int pad)
{
	u32 pad_offset = (SIUL2_GPIO_PAD_SIZE / BITS_PER_BYTE) * pad;
	void __iomem *opad_reg = pads + pad_offset;

	return opad_reg;
}

static inline void __iomem *siul2_get_opad_addr(void __iomem *opads,
						unsigned int pad)
{
	return siul2_get_pad_addr(opads, pad);
}

static inline void __iomem *siul2_get_ipad_addr(void __iomem *ipads,
						unsigned int pad)
{
	return siul2_get_pad_addr(ipads, pad);
}

static void siul2_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);
	void __iomem *reg;
	unsigned long flags, data;

	unsigned int mask = siul2_pin2mask(offset, gpio_dev);
	unsigned int pad = siul2_pin2pad(offset, gpio_dev);
	enum gpio_dir dir;

	dir = gpio_get_direction(gpio_dev, offset);
	if (dir == IN)
		return;

	reg = siul2_get_opad_addr(gpio_dev->opads, pad);

	spin_lock_irqsave(&gpio_dev->lock, flags);

	data = readl(reg);
	if (value)
		data |= mask;
	else
		data &= ~mask;

	writel(data, reg);

	spin_unlock_irqrestore(&gpio_dev->lock, flags);
}

static int siul2_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);
	void __iomem *reg;
	unsigned long flags, data = 0;
	enum gpio_dir dir;

	unsigned int mask = siul2_pin2mask(offset, gpio_dev);
	unsigned int pad = siul2_pin2pad(offset, gpio_dev);

	dir = gpio_get_direction(gpio_dev, offset);

	if (dir == OUT)
		reg = siul2_get_opad_addr(gpio_dev->opads, pad);
	else
		reg = siul2_get_ipad_addr(gpio_dev->ipads, pad);

	spin_lock_irqsave(&gpio_dev->lock, flags);

	data = readl(reg) & mask;

	spin_unlock_irqrestore(&gpio_dev->lock, flags);

	return data ? 1 : 0;
}


static int siul2_gpio_pads_init(struct platform_device *pdev,
				     struct siul2_gpio_dev *gpio_dev)
{
	struct resource *ores, *ires;

	/* GPIO Output Pad Data */
	ores = platform_get_resource(pdev, IORESOURCE_MEM,
				     SIUL2_GPIO_OUTPUT_RANGE);
	if (!ores)
		goto res_error;

	gpio_dev->opads = devm_ioremap_resource(&pdev->dev, ores);
	if (IS_ERR(gpio_dev->opads))
		return PTR_ERR(gpio_dev->opads);

	/* GPIO Input Pad Data */
	ires = platform_get_resource(pdev, IORESOURCE_MEM,
				     SIUL2_GPIO_INPUT_RANGE);
	if (!ores)
		goto res_error;

	gpio_dev->ipads = devm_ioremap_resource(&pdev->dev, ires);
	if (IS_ERR(gpio_dev->ipads))
		return PTR_ERR(gpio_dev->ipads);

	return 0;

res_error:
	dev_err(&pdev->dev, "can't fetch device resource info\n");
	return -EIO;
}

int siul2_gpio_probe(struct platform_device *pdev)
{
	int err = 0;
	struct siul2_gpio_dev *gpio_dev;
	struct of_phandle_args pinspec;
	struct gpio_chip *gc;
	size_t bitmap_size;

	gpio_dev = devm_kzalloc(&pdev->dev, sizeof(*gpio_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;

	err = siul2_gpio_pads_init(pdev, gpio_dev);
	if (err)
		return err;

	gpio_dev->irqchip.name = "siul2-gpio";
	gpio_dev->irqchip.irq_ack = siul2_gpio_irq_mask;
	gpio_dev->irqchip.irq_mask = siul2_gpio_irq_mask;
	gpio_dev->irqchip.irq_unmask = siul2_gpio_irq_unmask;
	gpio_dev->irqchip.irq_set_type = siul2_gpio_irq_set_type;

	gc = &gpio_dev->gc;

	platform_set_drvdata(pdev, gpio_dev);

	spin_lock_init(&gpio_dev->lock);

	err = siul2_get_gpio_pinspec(pdev, &pinspec, 0);
	if (err) {
		dev_err(&pdev->dev,
			"unable to get pinspec from device tree\n");
		return -EIO;
	}

	/* First GPIO number handled by this chip */
	gc->base = pinspec.args[1];
	/* Number of pins */
	gc->ngpio = pinspec.args[2];

	bitmap_size = BITS_TO_LONGS(gc->ngpio) *
		sizeof(*gpio_dev->pin_dir_bitmap);
	gpio_dev->pin_dir_bitmap = devm_kzalloc(&pdev->dev, bitmap_size,
						GFP_KERNEL);

	gc->parent = &pdev->dev;
	gc->label = dev_name(&pdev->dev);
	gc->set = siul2_gpio_set;
	gc->get = siul2_gpio_get;

	gc->request = siul2_gpio_request;
	gc->free = siul2_gpio_free;
	gc->direction_output = siul2_gpio_dir_out;
	gc->direction_input = siul2_gpio_dir_in;
	gc->owner = THIS_MODULE;

	err = gpiochip_add(gc);
	if (err) {
		dev_err(&pdev->dev, "unable to add gpiochip: %d\n", err);
		return -EINVAL;
	}

	/* EIRQs setup */
	err = siul2_irq_setup(pdev, gpio_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to setup IRQ : %d\n", err);
		return -EINVAL;
	}

	return err;
}

static struct platform_driver siul2_gpio_driver = {
	.driver		= {
		.name	= "siul2-gpio",
		.owner = THIS_MODULE,
		.of_match_table = siul2_gpio_dt_ids,
	},
	.probe		= siul2_gpio_probe,
	.remove		= siul2_gpio_remove,
};

int siul2_gpio_init(void)
{
	return platform_driver_register(&siul2_gpio_driver);
}
module_init(siul2_gpio_init);

void siul2_gpio_exit(void)
{
	platform_driver_unregister(&siul2_gpio_driver);
}
module_exit(siul2_gpio_exit);


MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Freescale SIUL2 GPIO");
MODULE_LICENSE("GPL");
