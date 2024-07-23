// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SIUL2 GPIO support.
 *
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2018-2024 NXP
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
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/gpio/driver.h>
#include <linux/pinctrl/consumer.h>
#include <linux/bitmap.h>
#include <linux/regmap.h>
#include <linux/types.h>

#define SIUL2_PGPDO(N)		(((N) ^ 1) * 2)
#define SIUL2_EIRQ_REG(r)	((r) * 4)
#define S32CC_EIRQS_NUM		32
#define S32CC_SIUL2_NUM		2
#define S32CC_PADS_DTS_TAG_LEN	(7)

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

#define SIUL2_GPIO_16_PAD_SIZE		16

#define EIRQS_DTS_TAG		"eirqs"
#define EIRQIMCRS_DTS_TAG	"eirq-imcrs"

struct eirq_mapping {
	u32 gpio;
	u16 eirq;
	u16 imscr;
	u8  imscr_conf;
};

/**
 * Platform data attached to compatible
 * @pad_access: access table for I/O pads, consists of S32CC_SIUL2_NUM tables
 * @num_irqs: the number of EIRQ - IMSCR - GPIO mappings
 * @irqs: the EIRQ - IMSCR - GPIO mappings
 * @reset_cnt: reset the pin name counter to zero when switching to SIUL2_1
 */
struct siul2_device_data {
	const struct regmap_access_table **pad_access;
	const u32 num_irqs;
	const struct eirq_mapping *irqs;
	const bool reset_cnt;
};

/**
 * struct siul2_desc - describes a SIUL2 hw module
 * @gpio_base: the first GPIO pin
 * @gpio_num: the number of GPIO pins
 * @opadmap: the regmap of the Parallel GPIO Pad Data Out Register
 * @ipadmap: the regmap of the Parallel GPIO Pad Data In Register
 * @pad_access: array of valid I/O pads
 */
struct siul2_desc {
	u32 gpio_base;
	u32 gpio_num;
	struct regmap *opadmap;
	struct regmap *ipadmap;
	const struct regmap_access_table *pad_access;
};

/**
 * struct siul2_gpio_dev - describes a group of GPIO pins
 * @platdata: the platform data
 * @siul2: the SIUL2 hw modules information
 * @eirqs_bitmap: the bitmap of currently used EIRQs
 * @pin_dir_bitmap: the bitmap with pin directions
 * @irqmap: the regmap for EIRQ registers
 * @eirqimcrsmap: the regmap for the EIRQs' IMCRs
 * @gc: the GPIO chip
 * @lock: mutual access to bitmaps
 * @wa_lock: lock to be used in the interrupt handler
 */
struct siul2_gpio_dev {
	const struct siul2_device_data *platdata;
	struct siul2_desc siul2[S32CC_SIUL2_NUM];
	unsigned long eirqs_bitmap;
	unsigned long *pin_dir_bitmap;
	struct regmap *irqmap;
	struct regmap *eirqimcrsmap;
	struct gpio_chip gc;
	raw_spinlock_t lock;
	raw_spinlock_t wa_lock;
};

static inline int siul2_get_gpio_pinspec(struct platform_device *pdev,
					 struct of_phandle_args *pinspec,
					 unsigned int range_index)
{
	struct device_node *np = pdev->dev.of_node;

	return of_parse_phandle_with_fixed_args(np, "gpio-ranges", 3,
						range_index, pinspec);
}

static inline struct regmap *siul2_offset_to_regmap(struct siul2_gpio_dev *dev,
						    unsigned int offset,
						    bool input)
{
	struct siul2_desc *siul2;
	int i;

	for (i = 0; i < ARRAY_SIZE(dev->siul2); ++i) {
		siul2 = &dev->siul2[i];
		if (offset >= siul2->gpio_base &&
		    offset - siul2->gpio_base < siul2->gpio_num)
			return input ? siul2->ipadmap : siul2->opadmap;
	}

	return NULL;
}

static inline void gpio_set_direction(struct siul2_gpio_dev *dev,
				      unsigned int gpio,
				      int dir)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&dev->lock, flags);

	if (dir == GPIO_LINE_DIRECTION_IN)
		bitmap_clear(dev->pin_dir_bitmap, gpio, 1);
	else
		bitmap_set(dev->pin_dir_bitmap, gpio, 1);

	raw_spin_unlock_irqrestore(&dev->lock, flags);
}

static inline int siul2_get_direction(struct siul2_gpio_dev *dev,
				      unsigned int gpio)
{
	return test_bit(gpio, dev->pin_dir_bitmap) ? GPIO_LINE_DIRECTION_OUT :
						     GPIO_LINE_DIRECTION_IN;
}

static inline struct siul2_gpio_dev *to_siul2_gpio_dev(struct gpio_chip *chip)
{
	return container_of(chip, struct siul2_gpio_dev, gc);
}

static int siul2_gpio_dir_in(struct gpio_chip *chip, unsigned int gpio)
{
	struct siul2_gpio_dev *gpio_dev;
	int ret = 0;

	ret = pinctrl_gpio_direction_input(gpio);
	if (ret)
		return ret;

	gpio_dev = to_siul2_gpio_dev(chip);
	gpio_set_direction(gpio_dev, gpio, GPIO_LINE_DIRECTION_IN);

	return 0;
}

static int siul2_gpio_get_dir(struct gpio_chip *chip, unsigned int gpio)
{
	return siul2_get_direction(to_siul2_gpio_dev(chip), gpio);
}

static int siul2_irq_gpio_index(const struct siul2_device_data *platdata,
				irq_hw_number_t gpio)
{
	int i;

	if (!platdata)
		return -EINVAL;

	for (i = 0; i < platdata->num_irqs; ++i)
		if (platdata->irqs[i].gpio == gpio)
			return i;

	return -ENXIO;
}

static int siul2_to_irq(struct gpio_chip *chip, unsigned int gpio)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);
	struct irq_domain *domain = chip->irq.domain;
	const struct siul2_device_data *platdata;
	int ret;

	platdata = gpio_dev->platdata;

	ret = siul2_irq_gpio_index(platdata, gpio);
	if (ret < 0)
		return ret;

	return irq_create_mapping(domain, gpio);
}

static unsigned int siul2_pin2pad(unsigned int pin)
{
	return pin / SIUL2_GPIO_16_PAD_SIZE;
}

static u16 siul2_pin2mask(unsigned int pin)
{
	/**
	 * From Reference manual :
	 * PGPDOx[PPDOy] = GPDO(x × 16) + (15 - y)[PDO_(x × 16) + (15 - y)]
	 */
	return BIT(15 - pin % SIUL2_GPIO_16_PAD_SIZE);
}

static inline u32 siul2_get_pad_offset(unsigned int pad)
{
	return SIUL2_PGPDO(pad);
}

static void siul2_gpio_set_val(struct gpio_chip *chip, unsigned int offset,
			       int value)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);
	unsigned int pad, reg_offset;
	struct regmap *regmap;
	u16 mask;

	mask = siul2_pin2mask(offset);
	pad = siul2_pin2pad(offset);

	reg_offset = siul2_get_pad_offset(pad);
	regmap = siul2_offset_to_regmap(gpio_dev, offset, false);
	if (!regmap)
		return;

	value = value ? mask : 0;

	regmap_update_bits(regmap, reg_offset, mask, value);
}

static int siul2_gpio_dir_out(struct gpio_chip *chip, unsigned int gpio,
			      int val)
{
	struct siul2_gpio_dev *gpio_dev;
	int ret = 0;

	gpio_dev = to_siul2_gpio_dev(chip);
	siul2_gpio_set_val(chip, gpio, val);

	ret = pinctrl_gpio_direction_output(gpio);
	if (ret)
		return ret;

	gpio_set_direction(gpio_dev, gpio, GPIO_LINE_DIRECTION_OUT);

	return 0;
}

static int siul2_set_config(struct gpio_chip *chip, unsigned int offset,
			    unsigned long config)
{
	return pinctrl_gpio_set_config(offset, config);
}

static int siul2_gpio_request(struct gpio_chip *chip, unsigned int gpio)
{
	return pinctrl_gpio_request(gpio);
}

static void siul2_gpio_free(struct gpio_chip *chip, unsigned int gpio)
{
	pinctrl_gpio_free(gpio);
}

static int siul2_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	unsigned int irq_type = type & IRQ_TYPE_SENSE_MASK;
	const struct siul2_device_data *platdata;
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	int index;
	u32 mask;

	platdata = gpio_dev->platdata;

	/* SIUL2 GPIO doesn't support level triggering */
	if ((irq_type & IRQ_TYPE_LEVEL_HIGH) ||
	    (irq_type & IRQ_TYPE_LEVEL_LOW)) {
		dev_err(gc->parent,
			"Invalid SIUL2 GPIO irq type 0x%x\n", type);
		return -EINVAL;
	}

	index = siul2_irq_gpio_index(platdata, gpio);
	if (index < 0)
		return index;

	mask = BIT(platdata->irqs[index].eirq);

	if (irq_type & IRQ_TYPE_EDGE_RISING)
		regmap_update_bits(gpio_dev->irqmap, SIUL2_IREER0, mask, mask);
	else
		regmap_update_bits(gpio_dev->irqmap, SIUL2_IREER0, mask, 0);

	if (irq_type & IRQ_TYPE_EDGE_FALLING)
		regmap_update_bits(gpio_dev->irqmap, SIUL2_IFEER0, mask, mask);
	else
		regmap_update_bits(gpio_dev->irqmap, SIUL2_IFEER0, mask, 0);

	return 0;
}

static irqreturn_t siul2_gpio_irq_handler(int irq, void *data)
{
	struct siul2_gpio_dev *gpio_dev = data;
	const struct siul2_device_data *plat;
	struct gpio_chip *gc = &gpio_dev->gc;
	unsigned int eirq, child_irq = 0;
	struct device *dev = gc->parent;
	unsigned long disr0_val_long;
	unsigned long wa_lock_flags;
	irqreturn_t ret = IRQ_NONE;
	u32 disr0_val;
	int i;

	plat = gpio_dev->platdata;

	/* Go through the entire GPIO bank and handle all interrupts */
	regmap_read(gpio_dev->irqmap, SIUL2_DISR0, &disr0_val);
	disr0_val_long = disr0_val;

	for_each_set_bit(eirq, &disr0_val_long,
			 BITS_PER_BYTE * sizeof(disr0_val)) {
		if (!test_bit(eirq, &gpio_dev->eirqs_bitmap))
			continue;

		/* GPIO lib irq */
		for (i = 0; i < plat->num_irqs; ++i) {
			if (plat->irqs[i].eirq == eirq) {
				child_irq =
					irq_find_mapping(gc->irq.domain,
							 plat->irqs[i].gpio);
				if (child_irq)
					break;
			}
		}

		if (!child_irq) {
			dev_err(dev,
				"Unable to detect IRQ number for EIRQ %d\n",
				eirq);
			continue;
		}

		/*
		 * Clear the interrupt before invoking the handler.
		 */
		regmap_write(gpio_dev->irqmap, SIUL2_DISR0, BIT(eirq));

		raw_spin_lock_irqsave(&gpio_dev->wa_lock, wa_lock_flags);
		generic_handle_irq(child_irq);
		raw_spin_unlock_irqrestore(&gpio_dev->wa_lock, wa_lock_flags);

		ret |= IRQ_HANDLED;
	}

	return ret;
}

static void siul2_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	irq_hw_number_t gpio = irqd_to_hwirq(data);
	const struct siul2_device_data *platdata;
	unsigned long flags;
	int index;
	u32 mask;

	platdata = gpio_dev->platdata;
	index = siul2_irq_gpio_index(platdata, gpio);
	if (index < 0)
		return;

	mask = BIT(platdata->irqs[index].eirq);

	/* Is interrupt used? */
	if (test_bit(platdata->irqs[index].eirq, &gpio_dev->eirqs_bitmap))
		return;

	gpiochip_enable_irq(gc, gpio);

	/* Disable interrupt */
	regmap_update_bits(gpio_dev->irqmap, SIUL2_DIRER0, mask, 0);

	/* Clear status flag */
	regmap_update_bits(gpio_dev->irqmap, SIUL2_DISR0, mask, mask);

	/* Enable Interrupt */
	regmap_update_bits(gpio_dev->irqmap, SIUL2_DIRER0, mask, mask);

	raw_spin_lock_irqsave(&gpio_dev->lock, flags);
	bitmap_set(&gpio_dev->eirqs_bitmap, platdata->irqs[index].eirq, 1);
	raw_spin_unlock_irqrestore(&gpio_dev->lock, flags);

	/* Set IMCR */
	regmap_write(gpio_dev->eirqimcrsmap,
		     SIUL2_EIRQ_REG(platdata->irqs[index].eirq),
		     platdata->irqs[index].imscr_conf);
}

static void siul2_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(gc);
	irq_hw_number_t gpio = irqd_to_hwirq(data);
	const struct siul2_device_data *platdata;
	unsigned long flags;
	int index;
	u32 mask;

	platdata = gpio_dev->platdata;

	index = siul2_irq_gpio_index(platdata, gpio);
	if (index < 0)
		return;

	mask = BIT(platdata->irqs[index].eirq);

	/* Is interrupt not used? */
	if (!test_bit(platdata->irqs[index].eirq, &gpio_dev->eirqs_bitmap))
		return;

	/* Disable interrupt */
	regmap_update_bits(gpio_dev->irqmap, SIUL2_DIRER0, mask, 0);

	/* Clear interrupt edge settings */
	regmap_update_bits(gpio_dev->irqmap, SIUL2_IREER0, mask, 0);
	regmap_update_bits(gpio_dev->irqmap, SIUL2_IFEER0, mask, 0);

	raw_spin_lock_irqsave(&gpio_dev->lock, flags);
	bitmap_clear(&gpio_dev->eirqs_bitmap, platdata->irqs[index].eirq, 1);
	raw_spin_unlock_irqrestore(&gpio_dev->lock, flags);

	regmap_write(gpio_dev->eirqimcrsmap,
		     SIUL2_EIRQ_REG(platdata->irqs[index].eirq), 0);

	gpiochip_disable_irq(gc, gpio);

	/* Clean status flag */
	regmap_update_bits(gpio_dev->irqmap, SIUL2_DISR0, mask, mask);
}

static const struct irq_chip siul2_gpio_irq_chip = {
	.name			= "siul2-gpio",
	.irq_ack		= siul2_gpio_irq_mask,
	.irq_mask		= siul2_gpio_irq_mask,
	.irq_unmask		= siul2_gpio_irq_unmask,
	.irq_set_type		= siul2_gpio_irq_set_type,
	.flags			= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

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
	struct device *dev = &pdev->dev;
	struct resource *res;
	resource_size_t size;
	void __iomem *base;

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
	conf->use_raw_spinlock = true;

	if (conf->cache_type != REGCACHE_NONE)
		conf->num_reg_defaults_raw = size / conf->reg_stride;

	return devm_regmap_init_mmio(dev, base, conf);
}

static bool irqregmap_accessible(struct device *dev, unsigned int reg)
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

static inline u32 siul2_get_opad_offset(unsigned int pad)
{
	return siul2_get_pad_offset(pad);
}

static inline u32 siul2_get_ipad_offset(unsigned int pad)
{
	return siul2_get_pad_offset(pad);
}

/* Common for both S32R45 an S32G* */

static const struct regmap_range s32cc_siul20_pad_yes_ranges[] = {
	regmap_reg_range(SIUL2_PGPDO(0), SIUL2_PGPDO(0)),
	regmap_reg_range(SIUL2_PGPDO(1), SIUL2_PGPDO(1)),
	regmap_reg_range(SIUL2_PGPDO(2), SIUL2_PGPDO(2)),
	regmap_reg_range(SIUL2_PGPDO(3), SIUL2_PGPDO(3)),
	regmap_reg_range(SIUL2_PGPDO(4), SIUL2_PGPDO(4)),
	regmap_reg_range(SIUL2_PGPDO(5), SIUL2_PGPDO(5)),
	regmap_reg_range(SIUL2_PGPDO(6), SIUL2_PGPDO(6)),
};

static const struct regmap_access_table s32cc_siul20_pad_access_table = {
	.yes_ranges	= s32cc_siul20_pad_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(s32cc_siul20_pad_yes_ranges),
};

static const struct regmap_range s32g_siul21_pad_yes_ranges[] = {
	regmap_reg_range(SIUL2_PGPDO(7), SIUL2_PGPDO(7)),
	regmap_reg_range(SIUL2_PGPDO(9), SIUL2_PGPDO(9)),
	regmap_reg_range(SIUL2_PGPDO(10), SIUL2_PGPDO(10)),
	regmap_reg_range(SIUL2_PGPDO(11), SIUL2_PGPDO(11)),
};

static const struct eirq_mapping s32g_irqs[] = {
	{ .gpio = 151,	.eirq = 0,	.imscr = 910,	.imscr_conf = 3 },
	{ .gpio = 19,	.eirq = 0,	.imscr = 910,	.imscr_conf = 2 },
	{ .gpio = 152,	.eirq = 1,	.imscr = 911,	.imscr_conf = 3 },
	{ .gpio = 20,	.eirq = 1,	.imscr = 911,	.imscr_conf = 2 },
	{ .gpio = 177,	.eirq = 2,	.imscr = 912,	.imscr_conf = 3 },
	{ .gpio = 21,	.eirq = 2,	.imscr = 912,	.imscr_conf = 2 },
	{ .gpio = 178,	.eirq = 3,	.imscr = 913,	.imscr_conf = 3 },
	{ .gpio = 22,	.eirq = 3,	.imscr = 913,	.imscr_conf = 2 },
	{ .gpio = 179,	.eirq = 4,	.imscr = 914,	.imscr_conf = 3 },
	{ .gpio = 23,	.eirq = 4,	.imscr = 914,	.imscr_conf = 2 },
	{ .gpio = 180,	.eirq = 5,	.imscr = 915,	.imscr_conf = 3 },
	{ .gpio = 24,	.eirq = 5,	.imscr = 915,	.imscr_conf = 2 },
	{ .gpio = 181,	.eirq = 6,	.imscr = 916,	.imscr_conf = 3 },
	{ .gpio = 25,	.eirq = 6,	.imscr = 916,	.imscr_conf = 2 },
	{ .gpio = 182,	.eirq = 7,	.imscr = 917,	.imscr_conf = 3 },
	{ .gpio = 26,	.eirq = 7,	.imscr = 917,	.imscr_conf = 2 },
	{ .gpio = 154,	.eirq = 8,	.imscr = 918,	.imscr_conf = 3 },
	{ .gpio = 27,	.eirq = 8,	.imscr = 918,	.imscr_conf = 2 },
	{ .gpio = 160,	.eirq = 9,	.imscr = 919,	.imscr_conf = 3 },
	{ .gpio = 28,	.eirq = 9,	.imscr = 919,	.imscr_conf = 2 },
	{ .gpio = 165,	.eirq = 10,	.imscr = 920,	.imscr_conf = 3 },
	{ .gpio = 29,	.eirq = 10,	.imscr = 920,	.imscr_conf = 2 },
	{ .gpio = 168,	.eirq = 11,	.imscr = 921,	.imscr_conf = 2 },
	{ .gpio = 31,	.eirq = 12,	.imscr = 922,	.imscr_conf = 2 },
	{ .gpio = 33,	.eirq = 13,	.imscr = 923,	.imscr_conf = 2 },
	{ .gpio = 34,	.eirq = 14,	.imscr = 924,	.imscr_conf = 2 },
	{ .gpio = 35,	.eirq = 15,	.imscr = 925,	.imscr_conf = 2 },
	{ .gpio = 184,	.eirq = 16,	.imscr = 926,	.imscr_conf = 2 },
	{ .gpio = 185,	.eirq = 17,	.imscr = 927,	.imscr_conf = 2 },
	{ .gpio = 186,	.eirq = 18,	.imscr = 928,	.imscr_conf = 2 },
	{ .gpio = 187,	.eirq = 19,	.imscr = 929,	.imscr_conf = 2 },
	{ .gpio = 188,	.eirq = 20,	.imscr = 930,	.imscr_conf = 2 },
	{ .gpio = 189,	.eirq = 21,	.imscr = 931,	.imscr_conf = 2 },
	{ .gpio = 190,	.eirq = 22,	.imscr = 932,	.imscr_conf = 2 },
	{ .gpio = 113,	.eirq = 23,	.imscr = 933,	.imscr_conf = 2 },
	{ .gpio = 114,	.eirq = 24,	.imscr = 934,	.imscr_conf = 2 },
	{ .gpio = 115,	.eirq = 25,	.imscr = 935,	.imscr_conf = 2 },
	{ .gpio = 117,	.eirq = 26,	.imscr = 936,	.imscr_conf = 2 },
	{ .gpio = 36,	.eirq = 27,	.imscr = 937,	.imscr_conf = 2 },
	{ .gpio = 37,	.eirq = 28,	.imscr = 938,	.imscr_conf = 2 },
	{ .gpio = 38,	.eirq = 29,	.imscr = 939,	.imscr_conf = 2 },
	{ .gpio = 39,	.eirq = 30,	.imscr = 940,	.imscr_conf = 2 },
	{ .gpio = 40,	.eirq = 31,	.imscr = 941,	.imscr_conf = 2 },
};

static const struct regmap_access_table s32g_siul21_pad_access_table = {
	.yes_ranges	= s32g_siul21_pad_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(s32g_siul21_pad_yes_ranges),
};

static const struct regmap_access_table *s32g_pad_access_table[] = {
	&s32cc_siul20_pad_access_table,
	&s32g_siul21_pad_access_table
};

static_assert(ARRAY_SIZE(s32g_pad_access_table) == S32CC_SIUL2_NUM);

static const struct siul2_device_data s32g_device_data = {
	.pad_access	= s32g_pad_access_table,
	.num_irqs	= ARRAY_SIZE(s32g_irqs),
	.irqs		= s32g_irqs,
	.reset_cnt	= true,
};

static const struct regmap_range s32r_siul21_pad_yes_ranges[] = {
	regmap_reg_range(SIUL2_PGPDO(6), SIUL2_PGPDO(6)),
	regmap_reg_range(SIUL2_PGPDO(7), SIUL2_PGPDO(7)),
	regmap_reg_range(SIUL2_PGPDO(8), SIUL2_PGPDO(8)),
};

static const struct eirq_mapping s32r45_irqs[] = {
	{ .gpio = 0,	.eirq = 0,	.imscr = 696,	.imscr_conf = 2 },
	{ .gpio = 1,	.eirq = 1,	.imscr = 697,	.imscr_conf = 2 },
	{ .gpio = 4,	.eirq = 2,	.imscr = 698,	.imscr_conf = 2 },
	{ .gpio = 5,	.eirq = 3,	.imscr = 699,	.imscr_conf = 2 },
	{ .gpio = 6,	.eirq = 4,	.imscr = 700,	.imscr_conf = 2 },
	{ .gpio = 8,	.eirq = 5,	.imscr = 701,	.imscr_conf = 2 },
	{ .gpio = 9,	.eirq = 6,	.imscr = 702,	.imscr_conf = 2 },
	{ .gpio = 10,	.eirq = 7,	.imscr = 703,	.imscr_conf = 2 },
	{ .gpio = 11,	.eirq = 8,	.imscr = 704,	.imscr_conf = 2 },
	{ .gpio = 13,	.eirq = 9,	.imscr = 705,	.imscr_conf = 2 },
	{ .gpio = 16,	.eirq = 10,	.imscr = 706,	.imscr_conf = 2 },
	{ .gpio = 17,	.eirq = 11,	.imscr = 707,	.imscr_conf = 2 },
	{ .gpio = 18,	.eirq = 12,	.imscr = 708,	.imscr_conf = 2 },
	{ .gpio = 20,	.eirq = 13,	.imscr = 709,	.imscr_conf = 2 },
	{ .gpio = 22,	.eirq = 14,	.imscr = 710,	.imscr_conf = 2 },
	{ .gpio = 23,	.eirq = 15,	.imscr = 711,	.imscr_conf = 2 },
	{ .gpio = 25,	.eirq = 16,	.imscr = 712,	.imscr_conf = 2 },
	{ .gpio = 26,	.eirq = 17,	.imscr = 713,	.imscr_conf = 2 },
	{ .gpio = 27,	.eirq = 18,	.imscr = 714,	.imscr_conf = 2 },
	{ .gpio = 28,	.eirq = 19,	.imscr = 715,	.imscr_conf = 2 },
	{ .gpio = 29,	.eirq = 20,	.imscr = 716,	.imscr_conf = 2 },
	{ .gpio = 30,	.eirq = 21,	.imscr = 717,	.imscr_conf = 2 },
	{ .gpio = 31,	.eirq = 22,	.imscr = 718,	.imscr_conf = 2 },
	{ .gpio = 32,	.eirq = 23,	.imscr = 719,	.imscr_conf = 2 },
	{ .gpio = 33,	.eirq = 24,	.imscr = 720,	.imscr_conf = 2 },
	{ .gpio = 35,	.eirq = 25,	.imscr = 721,	.imscr_conf = 2 },
	{ .gpio = 36,	.eirq = 26,	.imscr = 722,	.imscr_conf = 2 },
	{ .gpio = 37,	.eirq = 27,	.imscr = 723,	.imscr_conf = 2 },
	{ .gpio = 38,	.eirq = 28,	.imscr = 724,	.imscr_conf = 2 },
	{ .gpio = 39,	.eirq = 29,	.imscr = 725,	.imscr_conf = 2 },
	{ .gpio = 40,	.eirq = 30,	.imscr = 726,	.imscr_conf = 2 },
	{ .gpio = 44,	.eirq = 31,	.imscr = 727,	.imscr_conf = 2 },
};

static const struct regmap_access_table s32r_siul21_pad_access_table = {
	.yes_ranges	= s32r_siul21_pad_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(s32r_siul21_pad_yes_ranges),
};

static const struct regmap_access_table *s32r_pad_access_table[] = {
	&s32cc_siul20_pad_access_table,
	&s32r_siul21_pad_access_table,
};

static_assert(ARRAY_SIZE(s32r_pad_access_table) == S32CC_SIUL2_NUM);

static const struct siul2_device_data s32r45_device_data = {
	.pad_access	= s32r_pad_access_table,
	.num_irqs	= ARRAY_SIZE(s32r45_irqs),
	.irqs		= s32r45_irqs,
	.reset_cnt	= false,
};

static bool irqmap_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg == SIUL2_DISR0;
}

static struct regmap *init_irqregmap(struct platform_device *pdev)
{
	struct regmap_config regmap_conf = siul2_regmap_conf;

	regmap_conf.writeable_reg = irqregmap_accessible;
	regmap_conf.readable_reg = irqregmap_accessible;
	regmap_conf.volatile_reg = irqmap_volatile_reg;
	regmap_conf.val_format_endian = REGMAP_ENDIAN_LITTLE;

	return common_regmap_init(pdev, &regmap_conf, EIRQS_DTS_TAG);
}

static bool not_writable(__always_unused struct device *dev,
			 __always_unused unsigned int reg)
{
	return false;
}

static bool eirq_accessible(struct device *dev, unsigned int reg)
{
	if (reg < SIUL2_EIRQ_REG(S32CC_EIRQS_NUM))
		return true;

	return false;
}

static struct regmap *init_padregmap(struct platform_device *pdev,
				     struct siul2_gpio_dev *gpio_dev,
				     int selector, bool input)
{
	const struct siul2_device_data *platdata = gpio_dev->platdata;
	struct regmap_config regmap_conf = siul2_regmap_conf;
	char dts_tag[S32CC_PADS_DTS_TAG_LEN];
	int err;

	regmap_conf.reg_stride = 2;

	if (selector != 0 && selector != 1)
		return ERR_PTR(-EINVAL);

	regmap_conf.rd_table = platdata->pad_access[selector];

	err = snprintf(dts_tag, ARRAY_SIZE(dts_tag),  "%cpads%d",
		       input ? 'i' : 'o', selector);
	if (err < 0)
		return ERR_PTR(-EINVAL);

	if (input) {
		regmap_conf.writeable_reg = not_writable;
		regmap_conf.cache_type = REGCACHE_NONE;
	} else {
		regmap_conf.wr_table = platdata->pad_access[selector];
	}

	return common_regmap_init(pdev, &regmap_conf, dts_tag);
}

static struct regmap *init_eirqimcrsregmap(struct platform_device *pdev)
{
	struct regmap_config regmap_conf = siul2_regmap_conf;

	regmap_conf.writeable_reg = eirq_accessible;
	regmap_conf.readable_reg = eirq_accessible;

	return common_regmap_init(pdev, &regmap_conf, EIRQIMCRS_DTS_TAG);
}

static int siul2_irq_setup(struct platform_device *pdev,
			   struct siul2_gpio_dev *gpio_dev)
{
	int ret = 0, intlen, irq;
	const int *intspec;

	/* Skip gpio node without interrupts */
	intspec = of_get_property(pdev->dev.of_node, "interrupts", &intlen);
	if (!intspec)
		return -EINVAL;

	gpio_dev->irqmap = init_irqregmap(pdev);
	if (IS_ERR(gpio_dev->irqmap)) {
		dev_err(&pdev->dev, "Failed to initialize irq regmap config\n");
		return PTR_ERR(gpio_dev->irqmap);
	}

	gpio_dev->eirqimcrsmap = init_eirqimcrsregmap(pdev);
	if (IS_ERR(gpio_dev->eirqimcrsmap))
		dev_err(&pdev->dev,
			"Failed to initialize EIRQ IMCRS' regmap config\n");

	/* Request IRQ */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get irq resource\n");
		return irq;
	}

	/* Disable the interrupts and clear the status */
	regmap_write(gpio_dev->irqmap, SIUL2_DIRER0, 0);
	regmap_write(gpio_dev->irqmap, SIUL2_DISR0, ~0U);

	/* Select no interrupts by default */
	regmap_write(gpio_dev->irqmap, SIUL2_DIRSR0, 0);

	/* Disable rising-edge events */
	regmap_write(gpio_dev->irqmap, SIUL2_IREER0, 0);
	/* Disable falling-edge events */
	regmap_write(gpio_dev->irqmap, SIUL2_IFEER0, 0);

	ret = devm_request_irq(&pdev->dev, irq, siul2_gpio_irq_handler,
			       IRQF_SHARED,
			       dev_name(&pdev->dev), gpio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request interrupt\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id siul2_gpio_dt_ids[] = {
	{ .compatible = "nxp,s32g-siul2-gpio", .data = &s32g_device_data },
	{ .compatible = "nxp,s32r-siul2-gpio", .data = &s32r45_device_data},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, siul2_gpio_dt_ids);

static void siul2_gpio_set(struct gpio_chip *chip, unsigned int offset,
			   int value)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);

	if (!gpio_dev)
		return;

	if (siul2_get_direction(gpio_dev, offset) == GPIO_LINE_DIRECTION_IN)
		return;

	siul2_gpio_set_val(chip, offset, value);
}

static int siul2_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct siul2_gpio_dev *gpio_dev = to_siul2_gpio_dev(chip);
	unsigned int mask, pad, reg_offset, data = 0;
	struct regmap *regmap;

	mask = siul2_pin2mask(offset);
	pad = siul2_pin2pad(offset);

	reg_offset = siul2_get_pad_offset(pad);
	regmap = siul2_offset_to_regmap(gpio_dev, offset, true);
	if (!regmap)
		return -EINVAL;

	regmap_read(regmap, reg_offset, &data);

	return !!(data & mask);
}

static int siul2_gpio_pads_init(struct platform_device *pdev,
				struct siul2_gpio_dev *gpio_dev)
{
	struct device *dev = &pdev->dev;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(gpio_dev->siul2); ++i) {
		gpio_dev->siul2[i].opadmap = init_padregmap(pdev, gpio_dev, i,
							    false);
		if (IS_ERR(gpio_dev->siul2[i].opadmap)) {
			dev_err(dev,
				"Failed to initialize opad2%lu regmap config\n",
				i);
			return PTR_ERR(gpio_dev->siul2[i].opadmap);
		}

		gpio_dev->siul2[i].ipadmap = init_padregmap(pdev, gpio_dev, i,
							    true);
		if (IS_ERR(gpio_dev->siul2[i].ipadmap)) {
			dev_err(dev,
				"Failed to initialize ipad2%lu regmap config\n",
				i);
			return PTR_ERR(gpio_dev->siul2[i].ipadmap);
		}
	}

	return 0;
}

/* The hwirq number is the GPIO number. This is because an EIRQ
 * can be mapped in some cases to more GPIOs. Therefore, using the GPIO
 * as the hwirq we know the exact GPIO and we can find the EIRQ (since
 * there isn't a case where a GPIO can have more EIRQs attached to it).
 */
static int siul2_irq_domain_xlate(struct irq_domain *d,
				  struct device_node *ctrlr, const u32 *intspec,
				  unsigned int intsize,
				  irq_hw_number_t *out_hwirq,
				  unsigned int *out_type)
{
	struct gpio_chip *gc = d->host_data;
	irq_hw_number_t gpio;
	struct siul2_gpio_dev *gpio_dev;
	bool valid = false;
	size_t i;
	u32 base;
	int ret;

	ret = irq_domain_xlate_twocell(d, ctrlr, intspec, intsize,
				       &gpio, out_type);
	if (ret)
		return ret;

	gpio_dev = to_siul2_gpio_dev(gc);

	for (i = 0; i < ARRAY_SIZE(gpio_dev->siul2); ++i) {
		base = gpio_dev->siul2[i].gpio_base;
		if (gpio >= base && gpio - base < gpio_dev->siul2[i].gpio_num) {
			valid = true;
			break;
		}
	}

	if (!valid)
		return -EINVAL;

	*out_hwirq = gpio;

	return 0;
}

static const struct irq_domain_ops siul2_domain_ops = {
	.map	= gpiochip_irq_map,
	.unmap	= gpiochip_irq_unmap,
	.xlate	= siul2_irq_domain_xlate,
};

static int siul2_gen_names(struct device *dev, unsigned int cnt, char **names,
			   char *ch_index, int *num_index)
{
	unsigned int i;

	for (i = 0; i < cnt; i++) {
		if (i != 0 && !(*num_index % 16))
			(*ch_index)++;

		names[i] = devm_kasprintf(dev, GFP_KERNEL, "P%c_%02d",
					  *ch_index, 0x0F & (*num_index)++);
		if (!names[i])
			return -ENOMEM;
	}

	return 0;
}

static int siul2_gpio_populate_names(struct device *dev,
				     struct siul2_gpio_dev *gpio_dev)
{
	struct device_node *np = dev->of_node;
	char ch_index = 'A';
	int num_index = 0;
	int num_ranges;
	u32 base_gpio;
	u32 num_gpio;
	char **names;
	int ret;
	int i, j;

	names = devm_kcalloc(dev, gpio_dev->gc.ngpio, sizeof(*names),
			     GFP_KERNEL);
	if (!names)
		return -ENOMEM;

	ret = siul2_gen_names(dev, gpio_dev->siul2[0].gpio_num, names,
			      &ch_index, &num_index);
	if (ret) {
		dev_err(dev, "Could not set names for SIUL20 GPIOs\n");
		return ret;
	}

	if (gpio_dev->platdata->reset_cnt)
		num_index = 0;

	ch_index++;
	ret = siul2_gen_names(dev, gpio_dev->siul2[1].gpio_num,
			      names + gpio_dev->siul2[1].gpio_base, &ch_index,
			      &num_index);
	if (ret) {
		dev_err(dev, "Could not set names for SIUL21 GPIOs\n");
		return ret;
	}

	gpio_dev->gc.names = (const char *const *)names;

	/* Parse the gpio-reserved-ranges to know what GPIOs to exclude. */

	num_ranges = of_property_count_u32_elems(dev->of_node,
						 "gpio-reserved-ranges");

	/* The "gpio-reserved-ranges" is optional. */
	if (num_ranges < 0)
		return 0;
	num_ranges /= 2;

	for (i = 0; i < num_ranges; ++i) {
		ret = of_property_read_u32_index(np, "gpio-reserved-ranges",
						 i * 2, &base_gpio);
		if (ret) {
			dev_err(dev, "Could not parse the start GPIO: %d\n",
				ret);
			return ret;
		}

		ret = of_property_read_u32_index(np, "gpio-reserved-ranges",
						 i * 2 + 1, &num_gpio);
		if (ret) {
			dev_err(dev, "Could not parse num. GPIOs: %d\n", ret);
			return ret;
		}

		if (base_gpio + num_gpio > gpio_dev->gc.ngpio) {
			dev_err(dev, "Reserved GPIOs outside of GPIO range\n");
			return -EINVAL;
		}

		/* Remove names set for reserved GPIOs. */
		for (j = base_gpio; j < base_gpio + num_gpio; ++j) {
			devm_kfree(dev, names[j]);
			names[j] = NULL;
		}
	}

	return 0;
}

static int siul2_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct siul2_gpio_dev *gpio_dev;
	struct device *dev = &pdev->dev;
	struct of_phandle_args pinspec;
	struct gpio_irq_chip *girq;
	struct gpio_chip *gc;
	size_t bitmap_size;
	int ret = 0;
	int i;

	gpio_dev = devm_kzalloc(dev, sizeof(*gpio_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;

	of_id = of_match_device(siul2_gpio_dt_ids, dev);
	if (!of_id) {
		dev_err(dev, "Could not retrieve platdata\n");
		return -EINVAL;
	}

	gpio_dev->platdata = of_id->data;

	for (i = 0; i < S32CC_SIUL2_NUM; ++i)
		gpio_dev->siul2[i].pad_access =
			gpio_dev->platdata->pad_access[i];

	ret = siul2_gpio_pads_init(pdev, gpio_dev);
	if (ret)
		return ret;

	gc = &gpio_dev->gc;

	platform_set_drvdata(pdev, gpio_dev);

	raw_spin_lock_init(&gpio_dev->lock);
	raw_spin_lock_init(&gpio_dev->wa_lock);

	for (i = 0; i < ARRAY_SIZE(gpio_dev->siul2); ++i) {
		ret = siul2_get_gpio_pinspec(pdev, &pinspec, i);
		if (ret) {
			dev_err(dev,
				"unable to get pinspec %d from device tree\n",
				i);
			return -EINVAL;
		}

		if (pinspec.args_count != 3) {
			dev_err(dev, "Invalid pinspec count: %d\n",
				pinspec.args_count);
			return -EINVAL;
		}

		gpio_dev->siul2[i].gpio_base = pinspec.args[1];
		gpio_dev->siul2[i].gpio_num = pinspec.args[2];
	}

	gc->base = 0;

	/* In some cases, there is a gap between the SIUL GPIOs. */
	gc->ngpio = gpio_dev->siul2[S32CC_SIUL2_NUM - 1].gpio_base +
		    gpio_dev->siul2[S32CC_SIUL2_NUM - 1].gpio_num;

	ret = siul2_gpio_populate_names(&pdev->dev, gpio_dev);
	if (ret)
		return ret;

	gpio_dev->eirqs_bitmap = 0;

	bitmap_size = gc->ngpio * sizeof(*gpio_dev->pin_dir_bitmap);
	gpio_dev->pin_dir_bitmap = devm_bitmap_zalloc(dev, bitmap_size,
						      GFP_KERNEL);
	if (!gpio_dev->pin_dir_bitmap)
		return -ENOMEM;

	gc->parent = dev;
	gc->label = dev_name(dev);

	gc->set = siul2_gpio_set;
	gc->get = siul2_gpio_get;
	gc->set_config = siul2_set_config;
	gc->request = siul2_gpio_request;
	gc->free = siul2_gpio_free;
	gc->direction_output = siul2_gpio_dir_out;
	gc->direction_input = siul2_gpio_dir_in;
	gc->get_direction = siul2_gpio_get_dir;
	gc->owner = THIS_MODULE;

	girq = &gc->irq;
	gpio_irq_chip_set_chip(girq, &siul2_gpio_irq_chip);
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	girq->domain_ops = &siul2_domain_ops;

	ret = devm_gpiochip_add_data(dev, gc, gpio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "unable to add gpiochip\n");

	gc->to_irq = siul2_to_irq;

	/* EIRQs setup */
	ret = siul2_irq_setup(pdev, gpio_dev);
	if (ret) {
		dev_err(dev, "failed to setup IRQ : %d\n", ret);
		return ret;
	}

	return 0;
}

static int __maybe_unused siul2_suspend(struct device *dev)
{
	struct siul2_gpio_dev *gpio_dev = dev_get_drvdata(dev);
	size_t i;

	for (i = 0; i < ARRAY_SIZE(gpio_dev->siul2); ++i) {
		regcache_cache_only(gpio_dev->siul2[i].opadmap, true);
		regcache_mark_dirty(gpio_dev->siul2[i].opadmap);
	}

	regcache_cache_only(gpio_dev->irqmap, true);
	regcache_mark_dirty(gpio_dev->irqmap);

	regcache_cache_only(gpio_dev->eirqimcrsmap, true);
	regcache_mark_dirty(gpio_dev->eirqimcrsmap);

	return 0;
}

static int __maybe_unused siul2_resume(struct device *dev)
{
	struct siul2_gpio_dev *gpio_dev = dev_get_drvdata(dev);
	int ret = 0, err = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(gpio_dev->siul2); ++i) {
		regcache_cache_only(gpio_dev->siul2[i].opadmap, false);
		ret = regcache_sync(gpio_dev->siul2[i].opadmap);
		err |= ret;
		if (ret)
			dev_err(dev, "Failed to restore opadmap%lu: %d\n", i,
				ret);
	}

	regcache_cache_only(gpio_dev->irqmap, false);
	ret = regcache_sync(gpio_dev->irqmap);
	err |= ret;
	if (ret)
		dev_err(dev, "Failed to restore irqmap: %d\n", ret);

	regcache_cache_only(gpio_dev->eirqimcrsmap, false);
	ret = regcache_sync(gpio_dev->eirqimcrsmap);
	err |= ret;
	if (ret)
		dev_err(dev, "Failed to restore EIRQ IMCRs: %d\n", ret);

	return err ? -EIO : 0;
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
