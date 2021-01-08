// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SIUL2 GPIO support.
 *
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2019-2021 NXP
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/gpio/driver.h>
#include <asm-generic/bug.h>
#include <linux/bitmap.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <dt-bindings/pinctrl/s32-gen1-pinctrl.h>

#define SIUL2_PGPDO(N) (((N) ^ 1) * 2)

/* DMA/Interrupt Status Flag Register */
#define SIUL2_DISR0			0x0
/* DMA/Interrupt Request Enable Register */
#define SIUL2_DIRER0			0x8
/* DMA/Interrupt Request Select Register */
#define SIUL2_DIRSR0			0x10
/* Interrupt Rising-Edge Event Enable Register */
#define SIUL2_IREER0			0x18
/* Interrupt Falling-Edge Event Enable Register */
#define SIUL2_IFEER0			0x20

/* Device tree ranges */
#define SIUL2_GPIO_OUTPUT_RANGE		0
#define SIUL2_GPIO_INPUT_RANGE		1

/* Reserved for Pad Data Input/Output Registers */
#define SIUL2_GPIO_RESERVED_RANGE1	2
#define SIUL2_GPIO_RESERVED_RANGE2	3

/* Only for chips with interrupt controller */
#define SIUL2_GPIO_INTERRUPTS_RANGE	4

#define SIUL2_GPIO_32_PAD_SIZE		32
#define SIUL2_GPIO_16_PAD_SIZE		16
#define SIUL2_GPIO_PAD_SPACE		32

#define SIUL2_0_MAX_16_PAD_BANK_NUM	6

/**
 * enum gpio_dir - GPIO pin mode
 */
enum gpio_dir {
	IN, OUT
};

/**
 * Pin used as eirq.
 * On some platforms same eirq is exported by two pins from different gpio
 * chips.
 * Taking into account that same interrupt is raised no matter what
 * pin was configured as eirq, both gpio chips will receive the interrupt.
 * We will use "used" field to distinguish between them.
 * The user should't use in the same time both pins as eirq (same IMCR will
 * be configured when the pinmuxing is done).
 */
struct eirq_pin {
	int pin;
	bool used;
};

/**
 * Platform data attached to compatible
 * @pad_access: access table for output pads
 */
struct siul2_device_data {
	const struct regmap_access_table *pad_access;
};

/**
 * struct siul2_gpio_dev - describes a group of GPIO pins
 * @pdev: the platform device
 * @data: platform data
 * @ipads: input pads address
 * @opads: output pads address
 * @irq_base: the base address of EIRQ registers
 * @eirq_pins: array of pins which can be used as eirq
 * @eirq_npins: number of EIRQ pins
 * @pin_dir_bitmap: bitmap with pin directions
 * @gc: the GPIO chip
 * @lock: mutual access to chip registers
 *
 * @see gpio_dir
 */
struct siul2_gpio_dev {
	struct platform_device *pdev;
	const struct siul2_device_data *platdata;

	void __iomem *irq_base;
	struct eirq_pin *eirq_pins;
	unsigned int eirq_npins;

	unsigned long *pin_dir_bitmap;
	struct regmap *opadmap;
	struct regmap *ipadmap;
	struct regmap *irqmap;
	struct gpio_chip gc;
	spinlock_t lock;
};

/* We will use the following variable names:
 * - eirq - number between 0 and 32.
 * - pin - real GPIO id
 * - gpio - number relative to base (first GPIO handled by this chip).
 */
static inline bool siul2_is_valid_pin(int pin)
{
	return pin != S32GEN1_INVALID_GPIO;
}

static inline int siul2_gpio_to_pin(struct gpio_chip *gc, int gpio)
{
	return gc->base + gpio;
}

static inline int siul2_pin_to_gpio(struct gpio_chip *gc, int pin)
{
	return pin - gc->base;
}

static inline int siul2_eirq_to_pin(struct siul2_gpio_dev *gpio_dev, int eirq)
{
	return gpio_dev->eirq_pins[eirq].pin;
}

static inline bool siul2_is_valid_eirq(struct siul2_gpio_dev *gpio_dev,
								int eirq)
{
	if (eirq < 0 || eirq >= gpio_dev->eirq_npins)
		return false;
	return siul2_is_valid_pin(siul2_eirq_to_pin(gpio_dev, eirq));
}

static int siul2_pin_to_eirq(struct siul2_gpio_dev *gpio_dev, int pin)
{
	int i;

	for (i = 0; i < gpio_dev->eirq_npins; i++)
		if (gpio_dev->eirq_pins[i].pin == pin)
			return i;
	return -1;
}

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

/* Not all GPIOs from gpio-ranges can be used as EIRQs.
 * Use eirq-ranges for those that can be used as EIRQs.
 * Also we can have more eirq-ranges, each of them described
 * by the first gpio and the number of consecutive gpios.
 */
static inline int siul2_get_eirq_pinspec(
	struct siul2_gpio_dev *gpio_dev,
	struct platform_device *pdev)
{
	int ret, err, i, index = 0;
	struct of_phandle_iterator it;
	struct device_node *np = pdev->dev.of_node;
	uint32_t args[MAX_PHANDLE_ARGS];

	gpio_dev->eirq_npins = 0;
	of_for_each_phandle(&it, err, np, "eirq-ranges", NULL, 3) {
		ret = of_phandle_iterator_args(&it, args, MAX_PHANDLE_ARGS);
		gpio_dev->eirq_npins += args[2];
	}
	if (!gpio_dev->eirq_npins)
		return -EINVAL;

	gpio_dev->eirq_pins =
		devm_kzalloc(&pdev->dev,
			     gpio_dev->eirq_npins *
			     sizeof(*gpio_dev->eirq_pins),
			     GFP_KERNEL);
	if (!gpio_dev->eirq_pins)
		return -ENOMEM;

	index = 0;
	of_for_each_phandle(&it, err, np, "eirq-ranges", NULL, 3) {
		ret = of_phandle_iterator_args(&it, args, MAX_PHANDLE_ARGS);
		if (!siul2_is_valid_pin(args[1])) {
			for (i = 0; i < args[2]; i++)
				gpio_dev->eirq_pins[index + i].pin =
							S32GEN1_INVALID_GPIO;
		} else {
			for (i = 0; i < args[2]; i++)
				gpio_dev->eirq_pins[index + i].pin =
							args[1] + i;
		}
		index += args[2];
	}

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
	int pin = siul2_gpio_to_pin(chip, gpio);

	ret = pinctrl_gpio_direction_input(pin);
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
	int pin = siul2_gpio_to_pin(chip, gpio);

	ret = pinctrl_gpio_direction_output(pin);
	if (ret)
		return ret;

	gpio_dev = to_siul2_gpio_dev(chip);
	chip->set(chip, gpio, val);
	gpio_set_direction(gpio_dev, gpio, OUT);

	return ret;
}

static int siul2_gpio_request(struct gpio_chip *chip, unsigned int gpio)
{
	int pin = siul2_gpio_to_pin(chip, gpio);

	return pinctrl_gpio_request(pin);
}

static void siul2_gpio_free(struct gpio_chip *chip, unsigned int gpio)
{
	int pin = siul2_gpio_to_pin(chip, gpio);

	pinctrl_gpio_free(pin);
}

static int siul2_get_eirq_from_data(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	int pin = siul2_gpio_to_pin(gc, d->hwirq);

	return siul2_pin_to_eirq(gpio_dev, pin);
}

static int siul2_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	int eirq = siul2_get_eirq_from_data(d);
	unsigned long flags;
	unsigned int irq_type = type & IRQ_TYPE_SENSE_MASK;
	int ret;
	u32 ireer0_val;
	u32 ifeer0_val;
	int pin = siul2_eirq_to_pin(gpio_dev, eirq);

	if (!siul2_is_valid_eirq(gpio_dev, eirq))
		return -EINVAL;

	ret = pinctrl_gpio_direction_input(pin);
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

	regmap_read(gpio_dev->irqmap, SIUL2_IREER0, &ireer0_val);
	regmap_read(gpio_dev->irqmap, SIUL2_IFEER0, &ifeer0_val);

	if (irq_type & IRQ_TYPE_EDGE_RISING)
		ireer0_val |= BIT(eirq);
	else
		ireer0_val &= ~BIT(eirq);

	if (irq_type & IRQ_TYPE_EDGE_FALLING)
		ifeer0_val |= BIT(eirq);
	else
		ifeer0_val &= ~BIT(eirq);

	regmap_write(gpio_dev->irqmap, SIUL2_IREER0, ireer0_val);
	regmap_write(gpio_dev->irqmap, SIUL2_IFEER0, ifeer0_val);

	spin_unlock_irqrestore(&gpio_dev->lock, flags);

	return 0;
}

static irqreturn_t siul2_gpio_irq_handler(int irq, void *data)
{
	struct siul2_gpio_dev *gpio_dev = data;
	struct gpio_chip *gc = &gpio_dev->gc;
	unsigned int eirq, pin, gpio, child_irq;
	uint32_t disr0_val;
	unsigned long disr0_val_long;
	irqreturn_t ret = IRQ_NONE;

	/* Go through the entire GPIO bank and handle all interrupts */
	regmap_read(gpio_dev->irqmap, SIUL2_DISR0, &disr0_val);
	disr0_val_long = disr0_val;

	for_each_set_bit(eirq, &disr0_val_long,
					 BITS_PER_BYTE * sizeof(disr0_val)) {
		if (!gpio_dev->eirq_pins[eirq].used)
			continue;

		pin = siul2_eirq_to_pin(gpio_dev, eirq);
		gpio = siul2_pin_to_gpio(gc, pin);
		child_irq = irq_find_mapping(gc->irq.domain, gpio);

		/*
		 * Clear the interrupt before invoking the
		 * handler, so we do not leave any window
		 */
		regmap_write(gpio_dev->irqmap, SIUL2_DISR0, BIT(eirq));

		generic_handle_irq(child_irq);

		ret |= IRQ_HANDLED;
	}

	return ret;
}

static void siul2_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	int eirq = siul2_get_eirq_from_data(data);
	unsigned long flags;
	u32 direr0_val;
	u32 disr0_val;

	if (!siul2_is_valid_eirq(gpio_dev, eirq))
		return;

	spin_lock_irqsave(&gpio_dev->lock, flags);

	regmap_read(gpio_dev->irqmap, SIUL2_DIRER0, &direr0_val);

	/* Disable interrupt */
	direr0_val &= ~BIT(eirq);
	/* Clear status flag */
	disr0_val = BIT(eirq);

	regmap_write(gpio_dev->irqmap, SIUL2_DIRER0, direr0_val);
	regmap_write(gpio_dev->irqmap, SIUL2_DISR0, disr0_val);

	/* Enable Interrupt */
	direr0_val |= BIT(eirq);
	regmap_write(gpio_dev->irqmap, SIUL2_DIRER0, direr0_val);

	gpio_dev->eirq_pins[eirq].used = 1;

	spin_unlock_irqrestore(&gpio_dev->lock, flags);
}

static void siul2_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	int eirq = siul2_get_eirq_from_data(data);
	unsigned long flags;
	u32 direr0_val;
	u32 disr0_val;

	if (!siul2_is_valid_eirq(gpio_dev, eirq))
		return;

	spin_lock_irqsave(&gpio_dev->lock, flags);

	regmap_read(gpio_dev->irqmap, SIUL2_DIRER0, &direr0_val);

	/* Disable interrupt */
	direr0_val &= ~BIT(eirq);
	/* Clean status flag */
	disr0_val = BIT(eirq);

	regmap_write(gpio_dev->irqmap, SIUL2_DIRER0, direr0_val);
	regmap_write(gpio_dev->irqmap, SIUL2_DISR0, disr0_val);

	gpio_dev->eirq_pins[eirq].used = 0;
	spin_unlock_irqrestore(&gpio_dev->lock, flags);
}

static struct irq_chip siul2_gpio_irq_chip = {
	.name			= "siul2-gpio",
	.irq_ack		= siul2_gpio_irq_mask,
	.irq_mask		= siul2_gpio_irq_mask,
	.irq_unmask		= siul2_gpio_irq_unmask,
	.irq_set_type		= siul2_gpio_irq_set_type,
};

static const struct regmap_config siul2_regmap_conf = {
	.val_bits = 32,
	.reg_bits = 32,
	.cache_type = REGCACHE_FLAT,
};

static int common_regmap_conf(struct device *dev, struct regmap *map,
			      struct regmap_config *conf, const char *name)
{
	int ret;

	conf->max_register = regmap_get_max_register(map);
	conf->reg_stride = regmap_get_reg_stride(map);
	conf->name = devm_kasprintf(dev, GFP_KERNEL, "%s-%s",
					   dev_name(dev), name);
	if (!conf->name) {
		dev_err(dev, "Failed to allocated regmap name\n");
		return -ENOMEM;
	}

	ret = regmap_attach_dev(dev, map, conf);
	if (ret) {
		dev_err(dev, "Failed to attach device to regmap\n");
		return ret;
	}

	return regmap_reinit_cache(map, conf);
}

static bool irqregmap_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SIUL2_DISR0:
	case SIUL2_DIRER0:
	case SIUL2_DIRSR0:
	case SIUL2_IREER0:
	case SIUL2_IFEER0:
		return true;
	default:
		return false;
	};
}

static u16 siul2_pin2mask(int pin)
{
	/**
	 * From Reference manual :
	 * PGPDOx[PPDOy] = GPDO(x × 16) + (15 - y)[PDO_(x × 16) + (15 - y)]
	 */
	return BIT(15 - pin % SIUL2_GPIO_16_PAD_SIZE);
}

static unsigned int siul2_pin2pad(int pin)
{
	return pin / SIUL2_GPIO_16_PAD_SIZE;
}

static inline u32 siul2_get_pad_offset(unsigned int pad)
{
	return SIUL2_PGPDO(pad);
}

static inline int siul2_get_pin(struct gpio_chip *gc, u32 offset)
{
	return ((offset / 2) ^ 1) * SIUL2_GPIO_16_PAD_SIZE;
}

static inline u32 siul2_get_opad_offset(unsigned int pad)
{
	return siul2_get_pad_offset(pad);
}

static inline u32 siul2_get_ipad_offset(unsigned int pad)
{
	return siul2_get_pad_offset(pad);
}

static const struct regmap_range s32g2_pad_yes_ranges[] = {
	regmap_reg_range(SIUL2_PGPDO(0), SIUL2_PGPDO(0)),
	regmap_reg_range(SIUL2_PGPDO(1), SIUL2_PGPDO(1)),
	regmap_reg_range(SIUL2_PGPDO(2), SIUL2_PGPDO(2)),
	regmap_reg_range(SIUL2_PGPDO(3), SIUL2_PGPDO(3)),
	regmap_reg_range(SIUL2_PGPDO(4), SIUL2_PGPDO(4)),
	regmap_reg_range(SIUL2_PGPDO(5), SIUL2_PGPDO(5)),
	regmap_reg_range(SIUL2_PGPDO(6), SIUL2_PGPDO(6)),
	regmap_reg_range(SIUL2_PGPDO(7), SIUL2_PGPDO(7)),
	regmap_reg_range(SIUL2_PGPDO(9), SIUL2_PGPDO(9)),
	regmap_reg_range(SIUL2_PGPDO(10), SIUL2_PGPDO(10)),
	regmap_reg_range(SIUL2_PGPDO(11), SIUL2_PGPDO(11)),
};

static const struct regmap_access_table s32g2_pad_access_table = {
	.yes_ranges	= s32g2_pad_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(s32g2_pad_yes_ranges),
};

static const struct siul2_device_data s32g2_device_data = {
	.pad_access = &s32g2_pad_access_table,
};

static bool regmap_accessible(struct device *dev, unsigned int reg)
{
	struct siul2_gpio_dev *gpio_dev = dev_get_drvdata(dev);
	struct gpio_chip *gc = &gpio_dev->gc;
	int pin = siul2_get_pin(gc, reg);
	const struct siul2_device_data *platdata = gpio_dev->platdata;
	const struct regmap_access_table *access;
	u32 start_off, end_off;
	bool in_range = false;

	start_off = siul2_get_pad_offset(siul2_pin2pad(gc->base));
	end_off = siul2_get_pad_offset(siul2_pin2pad(gc->base + gc->ngpio - 1));

	/* A pad is split between CC and OFFCC */
	if (reg == start_off || reg == end_off) {
		in_range = true;

		/* No access filters */
		if (!platdata)
			return true;
	}

	if (pin >= gc->base && pin < gc->base + gc->ngpio) {
		in_range = true;

		/* No access filters */
		if (!platdata)
			return true;
	}

	if (platdata && in_range) {
		access = platdata->pad_access;
		return regmap_reg_in_ranges(reg, access->yes_ranges,
					    access->n_yes_ranges);
	}

	return false;
}

static int reinit_irqregmap_conf(struct device *dev, struct regmap *map)
{
	struct regmap_config regmap_conf = siul2_regmap_conf;

	regmap_conf.writeable_reg = irqregmap_writeable;
	return common_regmap_conf(dev, map, &regmap_conf, "irq");
}

static bool not_writable(__always_unused struct device *dev,
			 __always_unused unsigned int reg)
{
	return false;
}

static int reinit_opadregmap_conf(struct device *dev, struct regmap *map)
{
	struct regmap_config regmap_conf = siul2_regmap_conf;

	regmap_conf.writeable_reg = regmap_accessible;
	regmap_conf.readable_reg = regmap_accessible;
	return common_regmap_conf(dev, map, &regmap_conf, "opad");
}

static int reinit_ipadregmap_conf(struct device *dev, struct regmap *map)
{
	struct regmap_config regmap_conf = siul2_regmap_conf;

	regmap_conf.cache_type = REGCACHE_NONE;
	regmap_conf.writeable_reg = not_writable;
	regmap_conf.readable_reg = regmap_accessible;
	return common_regmap_conf(dev, map, &regmap_conf, "ipad");
}

static int siul2_irq_setup(struct platform_device *pdev,
			  struct siul2_gpio_dev *gpio_dev)
{
	int err, ret = 0;
	const int *intspec;
	int intlen;
	int irq;
	unsigned long flags;
	/*
	 * Allow multiple instances of the gpio driver to only
	 * initialize the irq control registers only once.
	 */
	static int init_flag;

	/* Skip gpio node without interrupts */
	intspec = of_get_property(pdev->dev.of_node, "interrupts", &intlen);
	if (!intspec)
		return 0;

	gpio_dev->irqmap = syscon_regmap_lookup_by_phandle(
						pdev->dev.of_node, "regmap2");
	if (IS_ERR(gpio_dev->irqmap))
		return PTR_ERR(gpio_dev->irqmap);

	ret = reinit_irqregmap_conf(&pdev->dev, gpio_dev->irqmap);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to reinitialize regmap configuration\n");
		return ret;
	}

	/* EIRQ pins */
	err = siul2_get_eirq_pinspec(gpio_dev, pdev);
	if (err) {
		dev_err(&pdev->dev,
			"unable to get pinspec from device tree\n");
		ret = -EIO;
		goto irq_setup_err;
	}

	/* Request IRQ */
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "failed to get irq resource.\n");
		ret = irq;
		goto irq_setup_err;
	}

	spin_lock_irqsave(&gpio_dev->lock, flags);

	if (!init_flag) {
		/* Disable the interrupts and clear the status */
		regmap_write(gpio_dev->irqmap, SIUL2_DIRER0, 0);
		regmap_write(gpio_dev->irqmap, SIUL2_DISR0, ~0);

		/* Select interrupts by default */
		regmap_write(gpio_dev->irqmap, SIUL2_DIRSR0, 0);

		/* Disable rising-edge events */
		regmap_write(gpio_dev->irqmap, SIUL2_IREER0, 0);
		/* Disable falling-edge events */
		regmap_write(gpio_dev->irqmap, SIUL2_IFEER0, 0);

		/* set flag after successful initialization */
		init_flag = 1;

	}

	spin_unlock_irqrestore(&gpio_dev->lock, flags);

	/*
	 * We need to request the interrupt here (instead of providing chip
	 * to the irq directly) because both GPIO controllers share the same
	 * interrupt line.
	 */
	ret = devm_request_irq(&pdev->dev, irq, siul2_gpio_irq_handler,
			       IRQF_SHARED | IRQF_NO_THREAD,
			       dev_name(&pdev->dev), gpio_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request interrupt\n");
		return ret;
	}

	/* Setup irq domain on top of the generic chip. */
	err = gpiochip_irqchip_add(&gpio_dev->gc,
				   &siul2_gpio_irq_chip,
				   0,
				   handle_simple_irq,
				   IRQ_TYPE_NONE);
	if (err) {
		dev_err(&pdev->dev, "could not connect irqchip to gpiochip\n");
		ret = err;
		goto irq_setup_err;
	}

	gpiochip_set_chained_irqchip(&gpio_dev->gc,
				     &siul2_gpio_irq_chip,
				     irq,
				     NULL);

irq_setup_err:

	return ret;
}

int siul2_gpio_remove(struct platform_device *pdev)
{
	struct siul2_gpio_dev *gpio_dev = platform_get_drvdata(pdev);
	int err = 0;

	if (!gpio_dev)
		return -EINVAL;

	gpiochip_remove(&gpio_dev->gc);
	return err;
}

static const struct of_device_id siul2_gpio_dt_ids[] = {
	{ .compatible = "fsl,s32g274a-siul2-gpio", .data = &s32g2_device_data },
	{ .compatible = "fsl,s32gen1-siul2-gpio" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, siul2_gpio_dt_ids);

static void siul2_gpio_set(
	struct gpio_chip *chip, unsigned int offset, int value)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);
	unsigned long flags;
	unsigned int data, pad, reg_offset;
	enum gpio_dir dir;
	u16 mask;

	dir = gpio_get_direction(gpio_dev, offset);
	if (dir == IN)
		return;

	offset = siul2_gpio_to_pin(chip, offset);

	mask = siul2_pin2mask(offset);
	pad = siul2_pin2pad(offset);

	reg_offset = siul2_get_pad_offset(pad);

	spin_lock_irqsave(&gpio_dev->lock, flags);

	regmap_read(gpio_dev->opadmap, reg_offset, &data);

	if (value)
		data |= mask;
	else
		data &= ~mask;

	regmap_write(gpio_dev->opadmap, reg_offset, data);

	spin_unlock_irqrestore(&gpio_dev->lock, flags);
}

static int siul2_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);
	unsigned long flags;
	unsigned int mask, pad, reg_offset, data = 0;
	enum gpio_dir dir;

	dir = gpio_get_direction(gpio_dev, offset);

	offset = siul2_gpio_to_pin(chip, offset);

	mask = siul2_pin2mask(offset);
	pad = siul2_pin2pad(offset);

	reg_offset = siul2_get_pad_offset(pad);

	spin_lock_irqsave(&(gpio_dev->lock), flags);

	if (dir == OUT)
		regmap_read(gpio_dev->opadmap, reg_offset, &data);
	else
		regmap_read(gpio_dev->ipadmap, reg_offset, &data);

	spin_unlock_irqrestore(&(gpio_dev->lock), flags);

	return !!(data & mask);
}

static int siul2_gpio_pads_init(struct platform_device *pdev,
				     struct siul2_gpio_dev *gpio_dev)
{
	int ret;

	gpio_dev->opadmap =
		syscon_regmap_lookup_by_phandle(
		 pdev->dev.of_node, "regmap0");
	if (IS_ERR(gpio_dev->opadmap)) {
		dev_err(&pdev->dev,
			"unable to get opadmap from device tree\n");
		return PTR_ERR(gpio_dev->opadmap);
	}

	ret = reinit_opadregmap_conf(&pdev->dev, gpio_dev->opadmap);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to reinitialize opad regmap configuration\n");
		return ret;
	}

	gpio_dev->ipadmap =
		syscon_regmap_lookup_by_phandle(
		 pdev->dev.of_node, "regmap1");
	if (IS_ERR(gpio_dev->ipadmap))
		return PTR_ERR(gpio_dev->ipadmap);

	ret = reinit_ipadregmap_conf(&pdev->dev, gpio_dev->ipadmap);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to reinitialize ipad regmap configuration\n");
		return ret;
	}

	return 0;
}

int siul2_gpio_probe(struct platform_device *pdev)
{
	int err = 0;
	struct siul2_gpio_dev *gpio_dev;
	const struct of_device_id *of_id;
	struct of_phandle_args pinspec;
	struct gpio_chip *gc;
	size_t bitmap_size;

	gpio_dev = devm_kzalloc(&pdev->dev, sizeof(*gpio_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;

	err = siul2_gpio_pads_init(pdev, gpio_dev);
	if (err)
		return err;

	gc = &gpio_dev->gc;

	platform_set_drvdata(pdev, gpio_dev);

	spin_lock_init(&gpio_dev->lock);

	err = siul2_get_gpio_pinspec(pdev, &pinspec, 0);
	if (err) {
		dev_err(&pdev->dev,
			"unable to get pinspec from device tree\n");
		return -EIO;
	}

	of_id = of_match_device(siul2_gpio_dt_ids, &pdev->dev);
	if (of_id)
		gpio_dev->platdata = of_id->data;

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

static int __maybe_unused siul2_suspend(struct device *dev)
{
	struct siul2_gpio_dev *gpio_dev = dev_get_drvdata(dev);

	regcache_cache_only(gpio_dev->opadmap, true);
	regcache_mark_dirty(gpio_dev->opadmap);

	if (gpio_dev->irqmap) {
		regcache_cache_only(gpio_dev->irqmap, true);
		regcache_mark_dirty(gpio_dev->irqmap);
	}

	return 0;
}

static int __maybe_unused siul2_resume(struct device *dev)
{
	struct siul2_gpio_dev *gpio_dev = dev_get_drvdata(dev);
	int ret = 0;

	regcache_cache_only(gpio_dev->opadmap, false);
	ret = regcache_sync(gpio_dev->opadmap);
	if (ret)
		dev_err(dev, "Failed to restore opadmap: %d\n", ret);

	if (gpio_dev->irqmap) {
		regcache_cache_only(gpio_dev->irqmap, false);
		ret = regcache_sync(gpio_dev->irqmap);
		if (ret)
			dev_err(dev, "Failed to restore irqmap: %d\n", ret);
	}

	return ret;
}

static SIMPLE_DEV_PM_OPS(siul2_pm_ops, siul2_suspend, siul2_resume);

static struct platform_driver siul2_gpio_driver = {
	.driver		= {
		.name	= "s32-gen1-siul2-gpio",
		.owner = THIS_MODULE,
		.of_match_table = siul2_gpio_dt_ids,
		.pm = &siul2_pm_ops,
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


MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP SIUL2 GPIO");
MODULE_LICENSE("GPL");

