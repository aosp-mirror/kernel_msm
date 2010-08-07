/* arch/arm/mach-msm/generic_gpio.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include "gpio_chip.h"

#undef gpio_chip
static int gpio_chip_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct old_gpio_chip *old_chip;
	unsigned long irq_flags;
	int ret = -ENOTSUPP;

	old_chip = container_of(chip, struct old_gpio_chip, gpio_chip);
	spin_lock_irqsave(&old_chip->lock, irq_flags);
	ret = old_chip->configure(old_chip, chip->base + offset, GPIOF_INPUT);
	spin_unlock_irqrestore(&old_chip->lock, irq_flags);
	return ret;
}

static int gpio_chip_get(struct gpio_chip *chip, unsigned offset)
{
	struct old_gpio_chip *old_chip;
	unsigned long irq_flags;
	int ret = -ENOTSUPP;

	old_chip = container_of(chip, struct old_gpio_chip, gpio_chip);
	spin_lock_irqsave(&old_chip->lock, irq_flags);
	if (old_chip->read)
		ret = old_chip->read(old_chip, chip->base + offset);
	spin_unlock_irqrestore(&old_chip->lock, irq_flags);
	return ret;
}

static int
gpio_chip_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct old_gpio_chip *old_chip;
	unsigned long irq_flags;
	int ret = -ENOTSUPP;

	old_chip = container_of(chip, struct old_gpio_chip, gpio_chip);
	spin_lock_irqsave(&old_chip->lock, irq_flags);
	if (old_chip->write)
		old_chip->write(old_chip, chip->base + offset, value);
	ret = old_chip->configure(old_chip, chip->base + offset,
				  GPIOF_DRIVE_OUTPUT);
	spin_unlock_irqrestore(&old_chip->lock, irq_flags);
	return ret;
}

static void gpio_chip_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct old_gpio_chip *old_chip;
	unsigned long irq_flags;

	old_chip = container_of(chip, struct old_gpio_chip, gpio_chip);
	spin_lock_irqsave(&old_chip->lock, irq_flags);
	if (old_chip->write)
		old_chip->write(old_chip, chip->base + offset, value);
	spin_unlock_irqrestore(&old_chip->lock, irq_flags);
}

static int gpio_chip_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct old_gpio_chip *old_chip;
	unsigned long irq_flags;
	int ret = -ENOTSUPP;
	int irq;

	old_chip = container_of(chip, struct old_gpio_chip, gpio_chip);
	spin_lock_irqsave(&old_chip->lock, irq_flags);
	if (old_chip->get_irq_num)
		ret = old_chip->get_irq_num(old_chip, chip->base + offset,
					    &irq, NULL);
	spin_unlock_irqrestore(&old_chip->lock, irq_flags);
	if (ret)
		return ret;
	return irq;
}

int register_gpio_chip(struct old_gpio_chip *new_gpio_chip)
{
	spin_lock_init(&new_gpio_chip->lock);
	new_gpio_chip->gpio_chip.direction_input = gpio_chip_direction_input;
	new_gpio_chip->gpio_chip.get = gpio_chip_get;
	new_gpio_chip->gpio_chip.direction_output = gpio_chip_direction_output;
	new_gpio_chip->gpio_chip.set = gpio_chip_set;
	new_gpio_chip->gpio_chip.to_irq = gpio_chip_to_irq;
	new_gpio_chip->gpio_chip.base = new_gpio_chip->start;
	new_gpio_chip->gpio_chip.ngpio =
		new_gpio_chip->end - new_gpio_chip->start + 1;

	return gpiochip_add(&new_gpio_chip->gpio_chip);
}

