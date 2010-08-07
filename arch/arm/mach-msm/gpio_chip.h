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
#include <linux/gpio.h>

/* use to specify edge detection without */
#define GPIOF_IRQF_MASK         0x0000ffff
/* IRQF_TRIGGER_NONE is 0 which also means "as already configured" */
#define GPIOF_IRQF_TRIGGER_NONE 0x00010000
#define GPIOF_INPUT             0x00020000
#define GPIOF_DRIVE_OUTPUT      0x00040000
#define GPIOF_OUTPUT_LOW        0x00080000
#define GPIOF_OUTPUT_HIGH       0x00100000

struct old_gpio_chip {
	struct gpio_chip gpio_chip;
#define gpio_chip old_gpio_chip

	spinlock_t lock;
	unsigned int start;
	unsigned int end;

	int (*configure)(struct gpio_chip *chip, unsigned int gpio,
			 unsigned long flags);
	int (*get_irq_num)(struct gpio_chip *chip, unsigned int gpio,
			   unsigned int *irqp, unsigned long *irqnumflagsp);
	int (*read)(struct gpio_chip *chip, unsigned int gpio);
	int (*write)(struct gpio_chip *chip, unsigned int gpio, unsigned on);
	int (*read_detect_status)(struct gpio_chip *chip, unsigned int gpio);
	int (*clear_detect_status)(struct gpio_chip *chip, unsigned int gpio);
};

int register_gpio_chip(struct gpio_chip *gpio_chip);

#endif
