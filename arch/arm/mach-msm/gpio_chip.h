/* arch/arm/mach-msm/gpio_chip.h
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

#ifndef _LINUX_GPIO_CHIP_H
#define _LINUX_GPIO_CHIP_H

#include <linux/list.h>

struct gpio_chip {
	struct list_head list;
	struct gpio_state *state;

	unsigned int start;
	unsigned int end;

	int (*configure)(struct gpio_chip *chip, unsigned int gpio, unsigned long flags);
	int (*get_irq_num)(struct gpio_chip *chip, unsigned int gpio, unsigned int *irqp, unsigned long *irqnumflagsp);
	int (*read)(struct gpio_chip *chip, unsigned int gpio);
	int (*write)(struct gpio_chip *chip, unsigned int gpio, unsigned on);
	int (*read_detect_status)(struct gpio_chip *chip, unsigned int gpio);
	int (*clear_detect_status)(struct gpio_chip *chip, unsigned int gpio);
};

int register_gpio_chip(struct gpio_chip *gpio_chip);

#endif
