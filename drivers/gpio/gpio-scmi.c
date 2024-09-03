// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 *  Copyright 2022-2024 NXP
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/pinctrl/consumer.h>
#include <linux/module.h>
#include <linux/gpio_scmi_protocol.h>
#include <linux/gpio_ops_scmi_protocol.h>

#include "gpiolib.h"

/**
 * Describes the state of a GPIO IRQ
 */
struct gpio_eirq_state {
	u32 gpio; /* The GPIO to which the IRQ is attached */
	unsigned int virtirq, type;
	bool active; /* Is this mapping active or not ? */
	bool masked;
};

struct gpio_eirqs {
	struct gpio_eirq_state *states;
	/* Protect access to eirq states */
	raw_spinlock_t states_lock;
	u32 num;
};

struct scmi_gpio_sync {
	struct mutex irq_lock;
	bool needs_enable;
	bool needs_unmask;
	bool needs_mask;
};

struct scmi_gpio_dev {
	struct gpio_chip gc;
	struct notifier_block eirq_nb;
	struct gpio_eirqs eirqs;
	struct scmi_protocol_handle *scmi_ph;
	const struct scmi_gpio_proto_ops *scmi_ops;
	unsigned long *pins_value;
	struct scmi_gpio_sync sync;
};

static u32 get_ngpios(struct scmi_gpio_dev *gpio_dev)
{
	return gpio_dev->gc.ngpio;
}

static void save_gpio_value(struct scmi_gpio_dev *gpio_dev,
			    unsigned int gpio,
			    int value)
{
	if (gpio >= get_ngpios(gpio_dev))
		return;

	if (value)
		bitmap_set(gpio_dev->pins_value, gpio, 1);
	else
		bitmap_clear(gpio_dev->pins_value, gpio, 1);
}

static int get_saved_value(struct scmi_gpio_dev *gpio_dev,
			   unsigned int gpio)
{
	if (gpio >= get_ngpios(gpio_dev))
		return -EINVAL;

	if (test_bit(gpio, gpio_dev->pins_value))
		return 1;

	return 0;
}

static inline struct scmi_gpio_dev *to_scmi_gpio_dev(struct gpio_chip *chip)
{
	return container_of(chip, struct scmi_gpio_dev, gc);
}

static int scmi_gpio_init_valid_mask(struct gpio_chip *gc,
				     unsigned long *valid_mask,
				     unsigned int ngpios)
{
	u16 gc_gpios, slots;
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(gc);
	struct device *dev = gc->parent;
	const unsigned long *mask;

	mask = gpio_dev->scmi_ops->get_valid_mask(gpio_dev->scmi_ph, &gc_gpios,
						  &slots);

	if (gc_gpios != ngpios) {
		dev_err(dev, "Received an unexpected number of pins\n");
		return -EINVAL;
	}

	memcpy(valid_mask, mask,
	       SCMI_GPIO_PINS_PER_SLOT / BITS_PER_BYTE * slots);

	return 0;
}

static int request_scmi_gpio(struct scmi_gpio_dev *gpio_dev, unsigned int gpio)
{
	return gpio_dev->scmi_ops->gpio_request(gpio_dev->scmi_ph, gpio);
}

static int scmi_gpio_request(struct gpio_chip *chip, unsigned int gpio)
{
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);
	int ret;

	ret = pinctrl_gpio_request(chip, gpio);
	if (ret)
		return ret;

	ret = request_scmi_gpio(gpio_dev, gpio);
	if (ret)
		pinctrl_gpio_free(chip, gpio);

	return ret;
}

/*
 * NOTE: To be called with taken gpio_dev->eirqs.states_lock.
 */
static bool get_gpio_irq_state(struct scmi_gpio_dev *gpio_dev,
			       unsigned int gpio,
			       u32 *index)
{
	u32 i;

	for (i = 0u; i < gpio_dev->eirqs.num; i++) {
		if (gpio_dev->eirqs.states[i].gpio == gpio) {
			*index = i;
			return true;
		}
	}

	return false;
}

/*
 * NOTE: To be called with taken gpio_dev->eirqs.states_lock.
 */
static bool get_gpio_irq_active_state(struct scmi_gpio_dev *gpio_dev,
				      unsigned int gpio,
				      u32 *index)
{
	if (get_gpio_irq_state(gpio_dev, gpio, index))
		return gpio_dev->eirqs.states[*index].active;

	return false;
}

static int irq_state_set_type(struct scmi_gpio_dev *gpio_dev, unsigned int gpio,
			      unsigned int type)
{
	int ret = -EINVAL;
	u32 index;

	raw_spin_lock(&gpio_dev->eirqs.states_lock);

	if (get_gpio_irq_state(gpio_dev, gpio, &index)) {
		gpio_dev->eirqs.states[index].type = type;
		ret = 0;
	}

	raw_spin_unlock(&gpio_dev->eirqs.states_lock);

	return ret;
}

static void mark_masked_irq(struct scmi_gpio_dev *gpio_dev, unsigned int gpio,
			    bool mask)
{
	u32 index;

	raw_spin_lock(&gpio_dev->eirqs.states_lock);

	if (get_gpio_irq_active_state(gpio_dev, gpio, &index))
		gpio_dev->eirqs.states[index].masked = mask;

	raw_spin_unlock(&gpio_dev->eirqs.states_lock);
}

static void release_virt_irq(struct scmi_gpio_dev *gpio_dev, unsigned int gpio)
{
	struct gpio_eirq_state *state;
	u32 index;

	raw_spin_lock(&gpio_dev->eirqs.states_lock);

	if (get_gpio_irq_state(gpio_dev, gpio, &index)) {
		state = &gpio_dev->eirqs.states[index];
		/* This is usually created by .to_irq callback */
		state->virtirq = 0;
	}

	raw_spin_unlock(&gpio_dev->eirqs.states_lock);
}

static void scmi_gpio_free(struct gpio_chip *chip, unsigned int gpio)
{
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);

	release_virt_irq(gpio_dev, gpio);
	gpio_dev->scmi_ops->gpio_free(gpio_dev->scmi_ph, gpio);
	pinctrl_gpio_free(chip, gpio);
}

static int scmi_set_config(struct gpio_chip *chip, unsigned int offset,
			   unsigned long config)
{
	return pinctrl_gpio_set_config(chip, offset, config);
}

static void scmi_gpio_set(struct gpio_chip *chip, unsigned int offset,
			  int value)
{
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);

	gpio_dev->scmi_ops->gpio_set_value(gpio_dev->scmi_ph, offset,
					   value ? 1u : 0u);

	save_gpio_value(gpio_dev, offset, value);
}

static int scmi_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);

	return gpio_dev->scmi_ops->gpio_get_value(gpio_dev->scmi_ph, offset);
}

static int scmi_gpio_dir_out(struct gpio_chip *chip, unsigned int gpio,
			     int val)
{
	scmi_gpio_set(chip, gpio, val);

	return pinctrl_gpio_direction_output(chip, gpio);
}

static bool is_gpio_requested(struct scmi_gpio_dev *gpio_dev, unsigned int gpio)
{
	struct gpio_desc *desc = gpiochip_get_desc(&gpio_dev->gc, gpio);

	if (IS_ERR_OR_NULL(desc))
		return false;

	return test_bit(FLAG_REQUESTED, &desc->flags);
}

static int scmi_gpio_get_dir(struct gpio_chip *chip, unsigned int gpio)
{
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);
	int ret;

	if (!is_gpio_requested(gpio_dev, gpio))
		return -EINVAL;

	ret = gpio_dev->scmi_ops->gpio_get_direction(gpio_dev->scmi_ph, gpio);
	if (ret < 0)
		return ret;

	if (ret == SCMI_GPIO_DIR_OUT)
		return GPIO_LINE_DIRECTION_OUT;

	return GPIO_LINE_DIRECTION_IN;
}

static int scmi_gpio_dir_in(struct gpio_chip *chip, unsigned int gpio)
{
	return pinctrl_gpio_direction_input(chip, gpio);
}

static int update_irq_mapping(struct scmi_gpio_dev *gpio_dev,
			      unsigned int gpio,
			      u32 *irq,
			      bool assign_interrupt)
{
	const struct scmi_gpio_proto_ops *scmi_ops = gpio_dev->scmi_ops;
	struct gpio_eirq_state *state;
	bool found = false;
	int ret;

	/* Try to see if the IRQ is already mapped
	 * so we can avoid making an SCMI call
	 * via the ->gpio_get_irq call.
	 * The found variable is again set a few lines below
	 * in case another thread modified it after
	 * releasing the `eirq.states_lock` here.
	 */
	raw_spin_lock(&gpio_dev->eirqs.states_lock);
	found = get_gpio_irq_active_state(gpio_dev, gpio, irq);
	raw_spin_unlock(&gpio_dev->eirqs.states_lock);

	if (found)
		return 0;

	ret = scmi_ops->gpio_get_irq(gpio_dev->scmi_ph, gpio, irq);
	if (ret)
		return ret;

	if (*irq >= gpio_dev->eirqs.num)
		return -ENXIO;

	raw_spin_lock(&gpio_dev->eirqs.states_lock);

	/* Make sure another thread didn't reserve this. */
	found = get_gpio_irq_active_state(gpio_dev, gpio, irq);
	if (found) {
		raw_spin_unlock(&gpio_dev->eirqs.states_lock);
		return -EINVAL;
	}

	state = &gpio_dev->eirqs.states[*irq];

	state->gpio = gpio;

	/**
	 * There are functions which want to know if a GPIO
	 * has an interrupt associated with it. They don't
	 * necessarily want to enable the interrupt. For
	 * example, sysfs uses this information in order
	 * to figure out whether to display the edge file
	 * for a GPIO.
	 */
	state->active = assign_interrupt;
	raw_spin_unlock(&gpio_dev->eirqs.states_lock);

	return 0;
}

static int release_irq_mapping(struct scmi_gpio_dev *gpio_dev, unsigned int gpio)
{
	struct gpio_eirq_state *state = NULL;
	u32 index;

	raw_spin_lock(&gpio_dev->eirqs.states_lock);

	if (get_gpio_irq_active_state(gpio_dev, gpio, &index))
		state = &gpio_dev->eirqs.states[index];

	/* Invalidate the mapping */
	if (state)
		state->active = false;

	raw_spin_unlock(&gpio_dev->eirqs.states_lock);

	if (!state)
		return -EINVAL;

	return 0;
}

static int enable_eirq(struct scmi_gpio_dev *gpio_dev, unsigned int gpio,
		       u32 type)
{
	const struct scmi_gpio_proto_ops *scmi_ops = gpio_dev->scmi_ops;
	u32 irq;
	int ret;

	ret = update_irq_mapping(gpio_dev, gpio, &irq, true);
	if (ret)
		return ret;

	ret = irq_state_set_type(gpio_dev, gpio, type);
	if (ret)
		goto release_mapping;

	ret = scmi_ops->gpio_enable_irq(gpio_dev->scmi_ph, gpio, type);

release_mapping:
	if (ret)
		release_irq_mapping(gpio_dev, gpio);

	return ret;
}

static int release_irq(struct scmi_gpio_dev *gpio_dev, unsigned int gpio)
{
	const struct scmi_gpio_proto_ops *scmi_ops = gpio_dev->scmi_ops;

	scmi_ops->gpio_disable_irq(gpio_dev->scmi_ph, gpio);

	return release_irq_mapping(gpio_dev, gpio);
}

static int unmask_gpio_irq(struct scmi_gpio_dev *gpio_dev, unsigned int gpio)
{
	mark_masked_irq(gpio_dev, gpio, false);

	return gpio_dev->scmi_ops->gpio_unmask_irq(gpio_dev->scmi_ph, gpio);
}

static int mask_gpio_irq(struct scmi_gpio_dev *gpio_dev, unsigned int gpio)
{
	mark_masked_irq(gpio_dev, gpio, true);

	return gpio_dev->scmi_ops->gpio_mask_irq(gpio_dev->scmi_ph, gpio);
}

static void scmi_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);
	irq_hw_number_t gpio = irqd_to_hwirq(data);

	if (gpio > U32_MAX)
		return;

	gpiochip_enable_irq(chip, gpio);
	gpio_dev->sync.needs_unmask = true;
}

static void scmi_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);
	irq_hw_number_t gpio = irqd_to_hwirq(data);

	if (gpio > U32_MAX)
		return;

	gpio_dev->sync.needs_mask = true;
	gpiochip_disable_irq(chip, gpio);
}

static int scmi_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);
	irq_hw_number_t gpio = irqd_to_hwirq(d);

	if (gpio > U32_MAX)
		return -EINVAL;

	gpio_dev->sync.needs_enable = true;

	irq_state_set_type(gpio_dev, gpio, type);

	return IRQ_SET_MASK_OK;
}

static void scmi_gpio_bus_lock(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);

	mutex_lock(&gpio_dev->sync.irq_lock);
	gpio_dev->sync.needs_enable = false;
	gpio_dev->sync.needs_mask = false;
	gpio_dev->sync.needs_unmask = false;
}

static void scmi_gpio_bus_sync_unlock(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);
	irq_hw_number_t gpio = irqd_to_hwirq(data);
	const struct scmi_gpio_proto_ops *scmi_ops;
	unsigned int type;
	u32 index;
	int ret;

	if (gpio > U32_MAX)
		return;

	scmi_ops = gpio_dev->scmi_ops;

	if (gpio_dev->sync.needs_enable) {
		raw_spin_lock(&gpio_dev->eirqs.states_lock);

		if (!get_gpio_irq_state(gpio_dev, gpio, &index)) {
			dev_err(chip->parent,
				"No state set for GPIO IRQ: %lu\n",
				gpio);
			raw_spin_unlock(&gpio_dev->eirqs.states_lock);
			goto unlock;
		}

		type = gpio_dev->eirqs.states[index].type;
		raw_spin_unlock(&gpio_dev->eirqs.states_lock);

		ret = scmi_ops->gpio_enable_irq(gpio_dev->scmi_ph, gpio, type);
		if (ret) {
			dev_err(chip->parent,
				"Failed to enable GPIO IRQ: %lu error: %d\n",
				gpio, ret);
			goto unlock;
		}
	}

	if (gpio_dev->sync.needs_unmask)
		unmask_gpio_irq(gpio_dev, gpio);
	else if (gpio_dev->sync.needs_mask)
		mask_gpio_irq(gpio_dev, gpio);

unlock:
	mutex_unlock(&gpio_dev->sync.irq_lock);
}

static int scmi_gpio_irq_reqres(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);
	const struct scmi_gpio_proto_ops *scmi_ops;
	irq_hw_number_t gpio = irqd_to_hwirq(data);
	bool requested_gpio;
	int ret;
	u32 irq;

	if (gpio > U32_MAX)
		return -EINVAL;

	scmi_ops = gpio_dev->scmi_ops;
	requested_gpio = is_gpio_requested(gpio_dev, gpio);

	ret = update_irq_mapping(gpio_dev, gpio, &irq, true);
	if (ret)
		return ret;

	/**
	 * There are cases when an IRQ line can be requested without ownership
	 * of the GPIO pin. This is the case of EIRQs referenced through the
	 * device tree nodes.
	 */
	if (!requested_gpio) {
		ret = pinctrl_gpio_request(chip, gpio);
		if (ret)
			goto release_mapping;

		ret = pinctrl_gpio_direction_input(chip, gpio);
		if (ret)
			goto free_pinctrl_gpio;

		ret = request_scmi_gpio(gpio_dev, gpio);
		if (ret)
			goto free_pinctrl_gpio;
	}

	ret = gpiochip_irq_reqres(data);
	if (ret)
		goto free_scmi;

	return 0;

free_scmi:
	scmi_ops->gpio_free(gpio_dev->scmi_ph, gpio);
free_pinctrl_gpio:
	pinctrl_gpio_free(chip, gpio);
release_mapping:
	release_irq_mapping(gpio_dev, gpio);

	return ret;
}

static void scmi_gpio_irq_relres(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);
	const struct scmi_gpio_proto_ops *scmi_ops;
	irq_hw_number_t gpio = irqd_to_hwirq(data);

	if (gpio > U32_MAX)
		return;

	scmi_ops = gpio_dev->scmi_ops;

	gpiochip_irq_relres(data);

	/* Release the GPIO pin if it was explicitly used for EIRQ only */
	if (!is_gpio_requested(gpio_dev, gpio)) {
		scmi_ops->gpio_free(gpio_dev->scmi_ph, gpio);
		pinctrl_gpio_free(chip, gpio);
	}

	release_irq(gpio_dev, gpio);
}


static const struct irq_chip scmi_gpio_irq_chip = {
	.name			= "scmi-gpio",
	.irq_mask		= scmi_gpio_irq_mask,
	.irq_unmask		= scmi_gpio_irq_unmask,
	.irq_set_type		= scmi_gpio_irq_set_type,
	.irq_bus_lock		= scmi_gpio_bus_lock,
	.irq_bus_sync_unlock	= scmi_gpio_bus_sync_unlock,
	.irq_request_resources	= scmi_gpio_irq_reqres,
	.irq_release_resources	= scmi_gpio_irq_relres,
	.flags			= IRQCHIP_SET_TYPE_MASKED | IRQCHIP_IMMUTABLE,
};

static int scmi_gpio_to_irq(struct gpio_chip *chip, unsigned int gpio)
{
	struct scmi_gpio_dev *gpio_dev = to_scmi_gpio_dev(chip);
	struct irq_domain *domain = chip->irq.domain;
	unsigned int virt_irq;
	u32 irq;
	int ret;

	ret = update_irq_mapping(gpio_dev, gpio, &irq, false);
	if (ret)
		return ret;

	virt_irq = irq_find_mapping(domain, gpio);
	if (!virt_irq)
		virt_irq = irq_create_mapping(domain, gpio);

	raw_spin_lock(&gpio_dev->eirqs.states_lock);
	gpio_dev->eirqs.states[irq].virtirq = virt_irq;
	raw_spin_unlock(&gpio_dev->eirqs.states_lock);

	if (virt_irq > INT_MAX)
		return -E2BIG;

	return virt_irq;
}

static int scmi_gpio_irq_setup(struct scmi_gpio_dev *gpio_dev)
{
	struct device *dev = gpio_dev->gc.parent;
	const struct scmi_gpio_proto_ops *scmi_ops = gpio_dev->scmi_ops;
	int ret = 0;

	gpio_dev->eirqs.num = scmi_ops->get_neirqs(gpio_dev->scmi_ph);
	if (!gpio_dev->eirqs.num) {
		gpio_dev->eirqs.states = NULL;
		return 0;
	}

	gpio_dev->eirqs.states = devm_kcalloc(dev, gpio_dev->eirqs.num,
					      sizeof(*gpio_dev->eirqs.states),
					      GFP_KERNEL);
	if (!gpio_dev->eirqs.states)
		return -ENOMEM;

	raw_spin_lock_init(&gpio_dev->eirqs.states_lock);

	return ret;
}

static int eirq_notification_cb(struct notifier_block *nb, unsigned long event,
				void *data)
{
	struct scmi_gpio_eirq_report *eirq_update = data;
	struct scmi_gpio_dev *gpio_dev;
	struct gpio_chip *gc;
	struct device *dev;
	unsigned long flags, eirq, eirq_mask = eirq_update->eirq_mask;
	unsigned int child_irq = 0u;
	int ret;
	bool active;
	u32 gpio;

	gpio_dev = container_of(nb, struct scmi_gpio_dev, eirq_nb);
	gc = &gpio_dev->gc;
	dev = gc->parent;

	for_each_set_bit(eirq, &eirq_mask,
			 BITS_PER_BYTE * sizeof(eirq_update->eirq_mask)) {
		if (eirq >= gpio_dev->eirqs.num)
			continue;
		raw_spin_lock(&gpio_dev->eirqs.states_lock);

		gpio = gpio_dev->eirqs.states[eirq].gpio;
		active = gpio_dev->eirqs.states[eirq].active;

		raw_spin_unlock(&gpio_dev->eirqs.states_lock);

		if (!active)
			continue;

		child_irq = irq_find_mapping(gc->irq.domain, gpio);
		if (!child_irq) {
			dev_err(dev, "Unable to detect IRQ number for EIRQ %lu\n",
				eirq);
			continue;
		}

		local_irq_save(flags);
		generic_handle_irq(child_irq);
		local_irq_restore(flags);
	}

	ret = gpio_dev->scmi_ops->gpio_ack_irq_mask(gpio_dev->scmi_ph);
	if (ret)
		return NOTIFY_DONE;

	return NOTIFY_OK;
}

static int restore_gpio_pins(struct scmi_gpio_dev *gpio_dev)
{
	struct gpio_chip *gc = &gpio_dev->gc;
	struct device *dev = gc->parent;
	struct gpio_desc *desc;
	unsigned int gpio;
	int value;
	int ret = 0;

	gc = &gpio_dev->gc;

	if (gc->base < 0)
		return -EINVAL;

	if (gc->base > UINT_MAX - get_ngpios(gpio_dev))
		return -E2BIG;

	for (gpio = gc->base; gpio < gc->base + get_ngpios(gpio_dev); gpio++) {
		value = 0;

		desc = gpiochip_get_desc(gc, gpio);
		if (IS_ERR_OR_NULL(desc)) {
			dev_err(dev, "Failed to get GPIO description for %u\n",
				gpio);
			return PTR_ERR(desc);
		}

		if (!test_bit(FLAG_REQUESTED, &desc->flags))
			continue;

		ret = request_scmi_gpio(gpio_dev, gpio);
		if (ret) {
			dev_err(dev, "Failed to request GPIO %u\n", gpio);
			return ret;
		}

		if (test_bit(FLAG_IS_OUT, &desc->flags)) {
			value = get_saved_value(gpio_dev, gpio);
			if (value < 0) {
				dev_err(dev, "Invalid value found for GPIO %u (%d)\n",
					gpio, value);
				return value;
			}

			scmi_gpio_set(gc, gpio, value);
		}

	}

	return ret;
}

static int restore_irq_settings(struct scmi_gpio_dev *gpio_dev)
{
	struct gpio_chip *gc = &gpio_dev->gc;
	struct device *dev = gc->parent;
	struct gpio_eirq_state *irq_state;
	int ret = 0;
	u32 i;

	for (i = 0u; i < gpio_dev->eirqs.num; i++) {
		irq_state = &gpio_dev->eirqs.states[i];
		if (!irq_state->active)
			continue;

		if (!is_gpio_requested(gpio_dev, irq_state->gpio)) {
			ret = request_scmi_gpio(gpio_dev, irq_state->gpio);
			if (ret) {
				dev_err(dev, "Failed to request GPIO %u for IRQ\n",
					irq_state->gpio);
				return ret;
			}
		}

		ret = enable_eirq(gpio_dev, irq_state->gpio, irq_state->type);
		if (ret) {
			dev_err(dev, "Failed to enable IRQ for GPIO %u\n",
				irq_state->gpio);
			return ret;
		}

		if (irq_state->masked)
			ret = mask_gpio_irq(gpio_dev, irq_state->gpio);
		else
			ret = unmask_gpio_irq(gpio_dev, irq_state->gpio);

		if (ret) {
			dev_err(dev, "Failed to change IRQ mask for GPIO %u\n",
				irq_state->gpio);
			return ret;
		}
	}

	return ret;
}

static int scmi_gpio_resume(struct scmi_device *sdev)
{
	struct scmi_gpio_dev *gpio_dev;
	int ret = 0;

	gpio_dev = dev_get_drvdata(&sdev->dev);
	if (!gpio_dev)
		return -EINVAL;

	ret = restore_gpio_pins(gpio_dev);
	if (ret)
		return ret;

	ret = restore_irq_settings(gpio_dev);
	if (ret)
		return ret;

	return ret;
}

static int scmi_gpio_probe(struct scmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	const struct scmi_handle *handle = sdev->handle;
	struct scmi_gpio_dev *gpio_dev;
	struct gpio_chip *gc;
	int ret;
	struct gpio_irq_chip *girq;
	u16 ngpios;

	if (!handle) {
		dev_err(dev, "Could not get handle [protocol %u]\n",
			SCMI_PROTOCOL_ID_GPIO);
		return -ENODEV;
	}

	gpio_dev = devm_kzalloc(dev, sizeof(*gpio_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;

	gpio_dev->scmi_ops = handle->devm_protocol_get(sdev,
						       SCMI_PROTOCOL_ID_GPIO,
						       &gpio_dev->scmi_ph);
	if (IS_ERR(gpio_dev->scmi_ops)) {
		dev_err(dev, "Could not retrieve ops.\n");
		return PTR_ERR(gpio_dev->scmi_ops);
	}

	ngpios = gpio_dev->scmi_ops->get_ngpios(gpio_dev->scmi_ph);

	gpio_dev->pins_value = devm_bitmap_zalloc(dev, ngpios, GFP_KERNEL);
	if (!gpio_dev->pins_value)
		return -ENOMEM;

	mutex_init(&gpio_dev->sync.irq_lock);

	gpio_dev->eirq_nb.notifier_call = eirq_notification_cb;

	gc = &gpio_dev->gc;
	gc->can_sleep = true;
	gc->base = gpio_dev->scmi_ops->get_gpio_base(gpio_dev->scmi_ph);
	gc->ngpio = ngpios;

	gc->parent = dev;
	gc->label = dev_name(dev);
	gc->init_valid_mask = scmi_gpio_init_valid_mask;
	gc->set = scmi_gpio_set;
	gc->get = scmi_gpio_get;
	gc->set_config = scmi_set_config;
	gc->request = scmi_gpio_request;
	gc->free = scmi_gpio_free;
	gc->direction_output = scmi_gpio_dir_out;
	gc->direction_input = scmi_gpio_dir_in;
	gc->get_direction = scmi_gpio_get_dir;
	gc->owner = THIS_MODULE;

	girq = &gc->irq;
	gpio_irq_chip_set_chip(girq, &scmi_gpio_irq_chip);
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;

	ret = devm_gpiochip_add_data(dev, gc, gpio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to add gpiochip\n");

	gc->to_irq = scmi_gpio_to_irq;

	/* EIRQs setup */
	ret = scmi_gpio_irq_setup(gpio_dev);
	if (ret) {
		dev_err(dev, "Failed to setup IRQ : %d\n", ret);
		return ret;
	}

	ret = handle->notify_ops->devm_event_notifier_register(sdev,
				SCMI_PROTOCOL_ID_GPIO,
				SCMI_GPIO_IRQ_NOTIFICATION,
				NULL,
				&gpio_dev->eirq_nb);
	if (ret) {
		dev_err(dev,
			"Error in registering GPIO EIRQ update notifier (%d)",
			ret);
		return ret;
	}

	dev_set_drvdata(&sdev->dev, gpio_dev);

	return 0;
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_ID_GPIO, "gpio" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver scmi_gpio_driver = {
	.name = "scmi-gpio",
	.probe = scmi_gpio_probe,
	.resume = scmi_gpio_resume,
	.id_table = scmi_id_table,
};
module_scmi_driver(scmi_gpio_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP SCMI GPIO driver");
MODULE_LICENSE("Dual BSD/GPL");
