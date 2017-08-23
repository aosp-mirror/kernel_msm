/*
 * Copyright (C) 2017 Google, Inc.
 *
 * Author: Trevor Bunker <trevorbunker@google.com>
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

#include <linux/bitops.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "s2mpb04-core.h"

#define DRIVER_NAME "s2mpb04-gpio"

struct s2mpb04_gpio {
	struct gpio_chip gpio_chip;
	struct s2mpb04_core *s2mpb04_core;
	unsigned long int pin_used;
};

static inline struct s2mpb04_gpio *to_s2mpb04_gpio
	(struct gpio_chip *_gpio_chip)
{
	return container_of(_gpio_chip, struct s2mpb04_gpio, gpio_chip);
}

static int s2mpb04_gpio_request(struct gpio_chip *gpio_chip,
				unsigned int offset)
{
	struct s2mpb04_gpio *s2mpb04_gpio = to_s2mpb04_gpio(gpio_chip);

	if (test_and_set_bit(offset, &s2mpb04_gpio->pin_used))
		return -EBUSY;
	else
		return 0;
}

static void s2mpb04_gpio_free(struct gpio_chip *gpio_chip,
			      unsigned int offset)
{
	struct s2mpb04_gpio *s2mpb04_gpio = to_s2mpb04_gpio(gpio_chip);

	clear_bit(offset, &s2mpb04_gpio->pin_used);
}

static int s2mpb04_gpio_direction_input(struct gpio_chip *gpio_chip,
					unsigned int offset)
{
	/* pins are hard-coded as outputs */
	return -EIO;
}

static int s2mpb04_gpio_get(struct gpio_chip *gpio_chip, unsigned int offset)
{
	struct s2mpb04_gpio *s2mpb04_gpio = to_s2mpb04_gpio(gpio_chip);
	struct s2mpb04_core *s2mpb04_core = s2mpb04_gpio->s2mpb04_core;
	u8 byte;

	s2mpb04_read_byte(s2mpb04_core, S2MPB04_REG_GPIO_Y, &byte);
	return byte & (1 << offset);
}

static void s2mpb04_gpio_set(struct gpio_chip *gpio_chip, unsigned int offset,
			     int value)
{
	struct s2mpb04_gpio *s2mpb04_gpio = to_s2mpb04_gpio(gpio_chip);
	struct s2mpb04_core *s2mpb04_core = s2mpb04_gpio->s2mpb04_core;

	/*
	 * GPIO1 is an open-drain output and has to have the output disabled to
	 * allow it to float high when value is not 0.
	 */
	if ((offset == 1) && (value != 0)) {
		s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_GPIO_CTRL, 0x40);
	} else {
		s2mpb04_update_bits(s2mpb04_core, S2MPB04_REG_GPIO_A,
				    (1 << offset), ((value ? 1 : 0) << offset));
		s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_GPIO_CTRL, 0xC0);
	}
}

static int s2mpb04_gpio_direction_output(struct gpio_chip *gpio_chip,
					 unsigned int offset, int value)
{
	/* pins are already configured as outputs */
	s2mpb04_gpio_set(gpio_chip, offset, value);
	return 0;
}

static const struct gpio_chip s2mpb04_gpio_chip = {
	.label			= DRIVER_NAME,
	.owner			= THIS_MODULE,
	.request		= s2mpb04_gpio_request,
	.free			= s2mpb04_gpio_free,
	.direction_input	= s2mpb04_gpio_direction_input,
	.get			= s2mpb04_gpio_get,
	.direction_output	= s2mpb04_gpio_direction_output,
	.set			= s2mpb04_gpio_set,
	.base			= -1,
	.ngpio			= S2MPB04_NUM_GPIOS,
	.can_sleep		= 1,
};

static int s2mpb04_gpio_probe(struct platform_device *pdev)
{
	struct s2mpb04_core *s2mpb04_core = dev_get_drvdata(pdev->dev.parent);
	struct s2mpb04_gpio *s2mpb04_gpio;

	s2mpb04_gpio = devm_kzalloc(&pdev->dev, sizeof(*s2mpb04_gpio),
				    GFP_KERNEL);
	if (!s2mpb04_gpio)
		return -ENOMEM;

	s2mpb04_gpio->s2mpb04_core = s2mpb04_core;
	s2mpb04_gpio->gpio_chip = s2mpb04_gpio_chip;
	s2mpb04_gpio->gpio_chip.parent = &pdev->dev;

	platform_set_drvdata(pdev, s2mpb04_gpio);

	/* enable both GPIOs for output with no-pull */
	s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_GPIO_A, 0x00);
	s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_GPIO_CTRL, 0xC0);

	return gpiochip_add(&s2mpb04_gpio->gpio_chip);
}

static int s2mpb04_gpio_remove(struct platform_device *pdev)
{
	struct s2mpb04_gpio *s2mpb04_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&s2mpb04_gpio->gpio_chip);
	return 0;
}

static const struct of_device_id s2mpb04_gpio_of_match[] = {
	{ .compatible = "samsung,s2mpb04-gpio", },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpb04_gpio_of_match);

static struct platform_driver s2mpb04_gpio_driver = {
	.probe = s2mpb04_gpio_probe,
	.remove = s2mpb04_gpio_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = s2mpb04_gpio_of_match,
	},
};
module_platform_driver(s2mpb04_gpio_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("S2MPB04 GPIO Driver");
MODULE_LICENSE("GPL");
