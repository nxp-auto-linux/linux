// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SIUL2 GPIO support.
 *
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2019-2022 NXP
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

#define EIRQS_DTS_TAG	"eirqs"

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
 * @data: platform data
 * @ipads: input pads address
 * @opads: output pads address
 * @irq_base: the base address of EIRQ registers
 * @eirq_pins: array of pins which can be used as eirq
 * @eirq_npins: number of EIRQ pins
 * @pin_dir_bitmap: bitmap with pin directions
 * @gc: the GPIO chip
 * @lock: mutual access to chip registers for both siul20 and siul21
 *
 * @see gpio_dir
 */
struct siul2_gpio_dev {
	const struct siul2_device_data *platdata;

	void __iomem *irq_base;
	struct eirq_pin *eirq_pins;
	unsigned int eirq_npins;

	u32 siul20_gpio_base;
	u32 siul20_gpio_num;

	u32 siul21_gpio_base;
	u32 siul21_gpio_num;

	unsigned long *pin_dir_bitmap;
	struct regmap *opadmap20;
	struct regmap *opadmap21;
	struct regmap *ipadmap20;
	struct regmap *ipadmap21;
	struct regmap *irqmap;
	struct gpio_chip gc;
	struct irq_chip irq;
	spinlock_t lock;
};

/* We will use the following variable names:
 * - eirq - number between 0 and 32.
 * - pin - real GPIO id
 * - gpio - number relative to base (first GPIO handled by this chip).
 */
static inline int siul2_gpio_to_pin(struct gpio_chip *gc, int gpio)
{
	return gpio + gc->base;
}

static inline int siul2_pin_to_gpio(struct gpio_chip *gc, int pin)
{
	return pin - gc->base;
}

static inline int siul2_eirq_to_pin(struct siul2_gpio_dev *gpio_dev, int eirq)
{
	return gpio_dev->eirq_pins[eirq].pin;
}

static inline bool siul2_is_valid_eirq_id(struct siul2_gpio_dev *gpio_dev,
					  int eirq)
{
	if (eirq < 0 || eirq >= gpio_dev->eirq_npins)
		return false;
	return true;
}

static int siul2_pin_to_eirq(struct siul2_gpio_dev *gpio_dev, int pin)
{
	int i;

	for (i = 0; i < gpio_dev->eirq_npins; i++)
		if (gpio_dev->eirq_pins[i].pin == pin)
			return i;
	return -ENXIO;
}

static inline bool siul2_gpio_to_eirq_check(struct siul2_gpio_dev *gpio_dev,
					int *eirq)
{
	int pin;

	/* GPIO lib uses GPIO as EIRQ */
	pin = *eirq;
	*eirq = siul2_pin_to_eirq(gpio_dev, pin);

	return siul2_is_valid_eirq_id(gpio_dev, *eirq);
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
	return 0;
}

static inline struct regmap *siul2_offset_to_regmap(struct siul2_gpio_dev *dev,
						    unsigned int offset, bool input)
{
	if (offset >= dev->siul20_gpio_base &&
	    offset - dev->siul20_gpio_base < dev->siul20_gpio_num)
		return input ? dev->ipadmap20 : dev->opadmap20;
	else if (offset >= dev->siul21_gpio_base &&
		 offset - dev->siul21_gpio_base < dev->siul21_gpio_base)
		return input ? dev->ipadmap21 : dev->opadmap21;

	return NULL;
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

	ret = pinctrl_gpio_direction_input(siul2_gpio_to_pin(chip, gpio));
	if (ret)
		return ret;

	gpio_dev = to_siul2_gpio_dev(chip);
	gpio_set_direction(gpio_dev, gpio, IN);

	return ret;
}

static int siul2_to_irq(struct gpio_chip *chip, unsigned int gpio)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);
	struct irq_domain *domain = chip->irq.domain;
	int eirq = siul2_pin_to_eirq(gpio_dev, gpio);

	if (eirq < 0)
		return -ENXIO;

	return irq_create_mapping(domain, gpio);
}

static int siul2_gpio_dir_out(struct gpio_chip *chip, unsigned int gpio,
			      int val)
{
	int ret = 0;
	struct siul2_gpio_dev *gpio_dev;

	ret = pinctrl_gpio_direction_output(siul2_gpio_to_pin(chip, gpio));
	if (ret)
		return ret;

	gpio_dev = to_siul2_gpio_dev(chip);
	gpio_set_direction(gpio_dev, gpio, OUT);
	chip->set(chip, gpio, val);

	return ret;
}

static int siul2_gpio_request(struct gpio_chip *chip, unsigned int gpio)
{
	return pinctrl_gpio_request(siul2_gpio_to_pin(chip, gpio));
}

static void siul2_gpio_free(struct gpio_chip *chip, unsigned int gpio)
{
	pinctrl_gpio_free(siul2_gpio_to_pin(chip, gpio));
}

static int siul2_get_eirq_from_data(struct irq_data *d)
{
	return irqd_to_hwirq(d);
}

static int siul2_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	int eirq = siul2_get_eirq_from_data(d);
	unsigned long flags;
	unsigned int irq_type = type & IRQ_TYPE_SENSE_MASK;
	int ret, pin;
	u32 ireer0_val;
	u32 ifeer0_val;

	if (!siul2_gpio_to_eirq_check(gpio_dev, &eirq))
		return -EINVAL;

	pin = siul2_eirq_to_pin(gpio_dev, eirq);

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
	struct device *dev = gc->parent;
	unsigned int eirq, child_irq, pin, gpio;
	u32 disr0_val;
	unsigned long disr0_val_long;
	irqreturn_t ret = IRQ_NONE;

	/* Go through the entire GPIO bank and handle all interrupts */
	regmap_read(gpio_dev->irqmap, SIUL2_DISR0, &disr0_val);
	disr0_val_long = disr0_val;

	for_each_set_bit(eirq, &disr0_val_long,
			 BITS_PER_BYTE * sizeof(disr0_val)) {
		if (!gpio_dev->eirq_pins[eirq].used)
			continue;

		/* GPIO lib irq */
		pin = siul2_eirq_to_pin(gpio_dev, eirq);
		gpio = siul2_pin_to_gpio(gc, pin);
		child_irq = irq_find_mapping(gc->irq.domain, gpio);

		if (!child_irq)
			dev_err(dev, "Unable to detect IRQ for GPIO %d & EIRQ %d\n",
				gpio, eirq);

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

	if (!siul2_gpio_to_eirq_check(gpio_dev, &eirq))
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

	if (!siul2_gpio_to_eirq_check(gpio_dev, &eirq))
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

static const struct regmap_config siul2_regmap_conf = {
	.val_bits = 32,
	.reg_bits = 32,
	.reg_stride = 4,
	.cache_type = REGCACHE_FLAT,
};

static struct regmap *common_regmap_init(struct platform_device *pdev,
					 struct regmap_config *conf,
					 const char *name)
{
	struct resource *res;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	resource_size_t size;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get MEM resource: %s\n", name);
		return ERR_PTR(-EINVAL);
	}
	size = resource_size(res);
	base = devm_ioremap(dev, res->start, size);
	if (IS_ERR(base))
		return ERR_PTR(-ENOMEM);

	conf->val_bits = conf->reg_stride * 8;
	conf->max_register = size - conf->reg_stride;
	conf->name = name;

	return regmap_init_mmio(dev, base, conf);
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

static bool regmap_siul20_accessible(struct device *dev, unsigned int reg)
{
	struct siul2_gpio_dev *gpio_dev = dev_get_drvdata(dev);
	struct gpio_chip *gc = &gpio_dev->gc;
	int pin = siul2_get_pin(gc, reg);
	u32 base = gpio_dev->siul20_gpio_base;
	u32 ngpio = gpio_dev->siul20_gpio_num;
	const struct siul2_device_data *platdata = gpio_dev->platdata;
	const struct regmap_access_table *access;
	bool in_range = false;

	u32 start_off = siul2_get_pad_offset(siul2_pin2pad(base));
	u32 end_off = siul2_get_pad_offset(siul2_pin2pad(base + ngpio - 1));

	if (reg < start_off || reg > end_off)
		return false;

	if (pin >= base && pin < base + ngpio) {
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

	return true;
}

static bool regmap_siul21_accessible(struct device *dev, unsigned int reg)
{
	struct siul2_gpio_dev *gpio_dev = dev_get_drvdata(dev);
	struct gpio_chip *gc = &gpio_dev->gc;
	int pin = siul2_get_pin(gc, reg);
	u32 base = gpio_dev->siul20_gpio_base;
	u32 ngpio = gpio_dev->siul20_gpio_num;
	const struct siul2_device_data *platdata = gpio_dev->platdata;
	const struct regmap_access_table *access;
	bool in_range = false;

	u32 start_off = siul2_get_pad_offset(siul2_pin2pad(base));
	u32 end_off = siul2_get_pad_offset(siul2_pin2pad(base + ngpio - 1));

	if (reg < start_off || reg > end_off)
		return false;

	if (pin >= base && pin < base + ngpio) {
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

	return true;
}

static bool irqmap_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg == SIUL2_DISR0;
}

static struct regmap *init_irqregmap(struct platform_device *pdev)
{
	struct regmap_config regmap_conf = siul2_regmap_conf;
	static struct resource *glob_res;
	static struct regmap *glob_reg;
	struct resource *res;
	struct regmap *reg = NULL;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, EIRQS_DTS_TAG);

	regmap_conf.writeable_reg = irqregmap_writeable;
	regmap_conf.volatile_reg = irqmap_volatile_reg;
	regmap_conf.val_format_endian = REGMAP_ENDIAN_LITTLE;

	/**
	 * For the cases when the same EIRQ block is shared among
	 * multiple instances
	 */
	if (!glob_res) {
		reg = common_regmap_init(pdev, &regmap_conf, EIRQS_DTS_TAG);
		glob_res = res;
		glob_reg = reg;
		return reg;
	}

	if (glob_res->start == res->start)
		return glob_reg;

	if (reg)
		return reg;

	return common_regmap_init(pdev, &regmap_conf, EIRQS_DTS_TAG);
}

static bool not_writable(__always_unused struct device *dev,
			 __always_unused unsigned int reg)
{
	return false;
}

static struct regmap *init_opadregmap(struct platform_device *pdev, int selector)
{
	struct regmap_config regmap_conf = siul2_regmap_conf;
	regmap_conf.reg_stride = 2;

	if (selector == 0) {
		regmap_conf.writeable_reg = regmap_siul20_accessible;
		regmap_conf.readable_reg = regmap_siul20_accessible;

		return common_regmap_init(pdev, &regmap_conf, "opads0");
	} else if (selector == 1) {
		regmap_conf.writeable_reg = regmap_siul21_accessible;
		regmap_conf.readable_reg = regmap_siul21_accessible;

		return common_regmap_init(pdev, &regmap_conf, "opads1");
	}

	return NULL;
}

static struct regmap *init_ipadregmap(struct platform_device *pdev, int selector)
{
	struct regmap_config regmap_conf = siul2_regmap_conf;

	regmap_conf.cache_type = REGCACHE_NONE;
	regmap_conf.writeable_reg = not_writable;
	regmap_conf.reg_stride = 2;

	if (selector == 0) {
		regmap_conf.readable_reg = regmap_siul20_accessible;
		return common_regmap_init(pdev, &regmap_conf, "ipads0");
	} else if (selector == 1) {
		regmap_conf.readable_reg = regmap_siul21_accessible;
		return common_regmap_init(pdev, &regmap_conf, "ipads1");
	}

	return NULL;
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
	struct device *dev = &pdev->dev;

	/* Skip gpio node without interrupts */
	intspec = of_get_property(pdev->dev.of_node, "interrupts", &intlen);
	if (!intspec)
		return 0;

	gpio_dev->irqmap = init_irqregmap(pdev);
	if (IS_ERR(gpio_dev->irqmap)) {
		dev_err(dev, "Failed to initialize ipad regmap configuration\n");
		return PTR_ERR(gpio_dev->irqmap);
	}

	/* EIRQ pins */
	err = siul2_get_eirq_pinspec(gpio_dev, pdev);
	if (err) {
		dev_err(&pdev->dev,
			"unable to get eirq pinspec from device tree\n");
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

irq_setup_err:

	return ret;
}

static const struct of_device_id siul2_gpio_dt_ids[] = {
	{ .compatible = "nxp,s32g-siul2-gpio", .data = &s32g2_device_data },
	{ .compatible = "nxp,s32r-siul2-gpio" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, siul2_gpio_dt_ids);

static void siul2_gpio_set(
	struct gpio_chip *chip, unsigned int offset, int value)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);
	unsigned long flags;
	unsigned int pad, reg_offset;
	enum gpio_dir dir;
	u16 mask;
	struct regmap *regmap;

	dir = gpio_get_direction(gpio_dev, offset);
	if (dir == IN)
		return;

	mask = siul2_pin2mask(offset);
	pad = siul2_pin2pad(offset);

	reg_offset = siul2_get_pad_offset(pad);
	regmap = siul2_offset_to_regmap(gpio_dev, offset, false);
	if (!regmap)
		return;

	if (value)
		value = mask;
	else
		value = 0;

	spin_lock_irqsave(&gpio_dev->lock, flags);
	regmap_update_bits(regmap, reg_offset, mask, value);
	spin_unlock_irqrestore(&gpio_dev->lock, flags);
}

static int siul2_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);
	unsigned long flags;
	unsigned int mask, pad, reg_offset, data = 0;
	enum gpio_dir dir;
	struct regmap *regmap;

	dir = gpio_get_direction(gpio_dev, offset);

	mask = siul2_pin2mask(offset);
	pad = siul2_pin2pad(offset);

	reg_offset = siul2_get_pad_offset(pad);
	regmap = siul2_offset_to_regmap(gpio_dev, offset, (dir == IN));
	if (!regmap)
		return -EINVAL;

	spin_lock_irqsave(&(gpio_dev->lock), flags);
	regmap_read(regmap, reg_offset, &data);
	spin_unlock_irqrestore(&(gpio_dev->lock), flags);

	return !!(data & mask);
}

static int siul2_gpio_pads_init(struct platform_device *pdev,
				struct siul2_gpio_dev *gpio_dev)
{
	struct device *dev = &pdev->dev;

	gpio_dev->opadmap20 = init_opadregmap(pdev, 0);
	if (IS_ERR(gpio_dev->opadmap20)) {
		dev_err(dev, "Failed to initialize opad20 regmap configuration\n");
		return PTR_ERR(gpio_dev->opadmap20);
	}

	gpio_dev->opadmap21 = init_opadregmap(pdev, 1);
	if (IS_ERR(gpio_dev->opadmap21)) {
		dev_err(dev, "Failed to initialize opad21 regmap configuration\n");
		return PTR_ERR(gpio_dev->opadmap21);
	}

	gpio_dev->ipadmap20 = init_ipadregmap(pdev, 0);
	if (IS_ERR(gpio_dev->ipadmap20)) {
		dev_err(dev, "Failed to initialize ipad20 regmap configuration\n");
		return PTR_ERR(gpio_dev->ipadmap20);
	}

	gpio_dev->ipadmap21 = init_ipadregmap(pdev, 1);
	if (IS_ERR(gpio_dev->ipadmap21)) {
		dev_err(dev, "Failed to initialize ipad21 regmap configuration\n");
		return PTR_ERR(gpio_dev->ipadmap21);
	}

	return 0;
}

static int siul2_irq_domain_xlate(struct irq_domain *d,
				  struct device_node *ctrlr, const u32 *intspec,
				  unsigned int intsize,
				  irq_hw_number_t *out_hwirq,
				  unsigned int *out_type)
{
	int ret, pin, gpio, eirq;
	struct gpio_chip *gc = d->host_data;
	struct siul2_gpio_dev *gpio_dev;

	ret = irq_domain_xlate_twocell(d, ctrlr, intspec, intsize,
				       out_hwirq, out_type);
	if (ret)
		return ret;

	gpio_dev = to_siul2_gpio_dev(gc);

	eirq = *out_hwirq;
	pin = siul2_eirq_to_pin(gpio_dev, eirq);
	gpio = siul2_pin_to_gpio(gc, pin);

	if (gpio < 0)
		return -EINVAL;

	*out_hwirq = gpio;

	return 0;
}

static const struct irq_domain_ops siul2_domain_ops = {
	.map	= gpiochip_irq_map,
	.unmap	= gpiochip_irq_unmap,
	.xlate	= siul2_irq_domain_xlate,
};

static int siul2_gpio_probe(struct platform_device *pdev)
{
	int err = 0;
	struct siul2_gpio_dev *gpio_dev;
	const struct of_device_id *of_id;
	struct of_phandle_args pinspec20;
	struct of_phandle_args pinspec21;
	struct gpio_chip *gc;
	size_t bitmap_size;
	struct device *dev = &pdev->dev;
	struct gpio_irq_chip *girq;

	gpio_dev = devm_kzalloc(dev, sizeof(*gpio_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;

	err = siul2_gpio_pads_init(pdev, gpio_dev);
	if (err)
		return err;

	gc = &gpio_dev->gc;

	platform_set_drvdata(pdev, gpio_dev);

	spin_lock_init(&gpio_dev->lock);

	err = siul2_get_gpio_pinspec(pdev, &pinspec20, 0);
	if (err) {
		dev_err(dev, "unable to get pinspec 0 from device tree\n");
		return -EIO;
	}

	err = siul2_get_gpio_pinspec(pdev, &pinspec21, 1);
	if (err) {
		dev_err(dev, "unable to get pinspec 1 from device tree\n");
		return -EIO;
	}

	gpio_dev->siul20_gpio_base = pinspec20.args[1];
	gpio_dev->siul20_gpio_num = pinspec20.args[2];

	gpio_dev->siul21_gpio_base = pinspec21.args[1];
	gpio_dev->siul21_gpio_num = pinspec21.args[2];

	of_id = of_match_device(siul2_gpio_dt_ids, dev);
	if (of_id)
		gpio_dev->platdata = of_id->data;

	gc->base = -1;

	/* In some cases, there is a gap between SIUL20 and SIUL21 GPIOS. */
	gc->ngpio = pinspec21.args[1] + pinspec21.args[2];

	bitmap_size = BITS_TO_LONGS(gc->ngpio) *
		sizeof(*gpio_dev->pin_dir_bitmap);
	gpio_dev->pin_dir_bitmap = devm_kzalloc(dev, bitmap_size,
						GFP_KERNEL);
	gpio_dev->irq = (struct irq_chip) {
		.name			= dev_name(dev),
		.irq_ack		= siul2_gpio_irq_mask,
		.irq_mask		= siul2_gpio_irq_mask,
		.irq_unmask		= siul2_gpio_irq_unmask,
		.irq_set_type		= siul2_gpio_irq_set_type,
	};

	gc->parent = dev;
	gc->label = dev_name(dev);

	gc->set = siul2_gpio_set;
	gc->get = siul2_gpio_get;
	gc->request = siul2_gpio_request;
	gc->free = siul2_gpio_free;
	gc->direction_output = siul2_gpio_dir_out;
	gc->direction_input = siul2_gpio_dir_in;
	gc->owner = THIS_MODULE;

	girq = &gc->irq;
	girq->chip = &gpio_dev->irq;
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	girq->domain_ops = &siul2_domain_ops;

	err = devm_gpiochip_add_data(dev, gc, gpio_dev);
	if (err) {
		dev_err(dev, "unable to add gpiochip: %d\n", err);
		return -EINVAL;
	}

	gc->to_irq = siul2_to_irq;

	/* EIRQs setup */
	err = siul2_irq_setup(pdev, gpio_dev);
	if (err) {
		dev_err(dev, "failed to setup IRQ : %d\n", err);
		return -EINVAL;
	}

	return err;
}

static int __maybe_unused siul2_suspend(struct device *dev)
{
	struct siul2_gpio_dev *gpio_dev = dev_get_drvdata(dev);

	regcache_cache_only(gpio_dev->opadmap20, true);
	regcache_mark_dirty(gpio_dev->opadmap20);

	regcache_cache_only(gpio_dev->opadmap21, true);
	regcache_mark_dirty(gpio_dev->opadmap21);

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

	regcache_cache_only(gpio_dev->opadmap20, false);
	ret = regcache_sync(gpio_dev->opadmap20);
	if (ret)
		dev_err(dev, "Failed to restore opadmap0: %d\n", ret);

	regcache_cache_only(gpio_dev->opadmap21, false);

	ret = regcache_sync(gpio_dev->opadmap21);
	if (ret)
		dev_err(dev, "Failed to restore opadmap1: %d\n", ret);

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
		.name	= "s32cc-siul2-gpio",
		.owner = THIS_MODULE,
		.of_match_table = siul2_gpio_dt_ids,
		.pm = &siul2_pm_ops,
	},
	.probe		= siul2_gpio_probe,
};

static int siul2_gpio_init(void)
{
	return platform_driver_register(&siul2_gpio_driver);
}
module_init(siul2_gpio_init);

static void siul2_gpio_exit(void)
{
	platform_driver_unregister(&siul2_gpio_driver);
}
module_exit(siul2_gpio_exit);


MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP SIUL2 GPIO");
MODULE_LICENSE("GPL");

