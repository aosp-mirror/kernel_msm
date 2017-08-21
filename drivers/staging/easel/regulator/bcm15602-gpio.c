/*
 * Copyright (C) 2016 Google, Inc.
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

#define DEBUG

#include <linux/bitops.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "bcm15602-regulator.h"

#define DRIVER_NAME "bcm15602-gpio"

struct bcm15602_gpio {
	struct gpio_chip gpio_chip;
	struct bcm15602_chip *bcm15602;
	unsigned long int pin_used;
	int value[BCM15602_NUM_GPIOS];
};

static inline struct bcm15602_gpio *to_bcm15602_gpio
	(struct gpio_chip *_gpio_chip)
{
	return container_of(_gpio_chip, struct bcm15602_gpio, gpio_chip);
}

static int bcm15602_gpio_request(struct gpio_chip *gpio_chip, unsigned offset)
{
	struct bcm15602_gpio *bcm15602_gpio = to_bcm15602_gpio(gpio_chip);

	if (test_and_set_bit(offset, &bcm15602_gpio->pin_used))
		return -EBUSY;
	else
		return 0;
}

static void bcm15602_gpio_free(struct gpio_chip *gpio_chip, unsigned offset)
{
	struct bcm15602_gpio *bcm15602_gpio = to_bcm15602_gpio(gpio_chip);

	clear_bit(offset, &bcm15602_gpio->pin_used);
}

static int bcm15602_gpio_direction_input(struct gpio_chip *gpio_chip,
					 unsigned offset)
{
	/* pins are hard-coded as outputs */
	return -EIO;
}

static int bcm15602_gpio_get(struct gpio_chip *gpio_chip, unsigned offset)
{
	struct bcm15602_gpio *bcm15602_gpio = to_bcm15602_gpio(gpio_chip);
	struct bcm15602_chip *bcm15602 = bcm15602_gpio->bcm15602;
	u8 byte;

	bcm15602_read_byte(bcm15602, BCM15602_REG_SYS_GPIO_IN, &byte);
	return byte & (1 << offset);
}

static void bcm15602_gpio_set(struct gpio_chip *gpio_chip, unsigned offset,
			      int value)
{
	struct bcm15602_gpio *bcm15602_gpio = to_bcm15602_gpio(gpio_chip);
	struct bcm15602_chip *bcm15602 = bcm15602_gpio->bcm15602;
	u8 reg_data = 0xC;

	bcm15602_gpio->value[offset] = (value ? 1 : 0);
	reg_data |= (bcm15602_gpio->value[1] << 1) | bcm15602_gpio->value[0];
	bcm15602_write_byte(bcm15602, BCM15602_REG_SYS_GPIO_OUT_CTRL, reg_data);
}

static int bcm15602_gpio_direction_output(struct gpio_chip *gpio_chip,
					  unsigned offset, int value)
{
	/* pins are already configured as outputs */
	bcm15602_gpio_set(gpio_chip, offset, value);
	return 0;
}

static const struct gpio_chip bcm15602_gpio_chip = {
	.label			= DRIVER_NAME,
	.owner			= THIS_MODULE,
	.request		= bcm15602_gpio_request,
	.free			= bcm15602_gpio_free,
	.direction_input	= bcm15602_gpio_direction_input,
	.get			= bcm15602_gpio_get,
	.direction_output	= bcm15602_gpio_direction_output,
	.set			= bcm15602_gpio_set,
	.base			= -1,
	.ngpio			= BCM15602_NUM_GPIOS,
	.can_sleep		= 1,
};

static int bcm15602_gpio_probe(struct platform_device *pdev)
{
	struct bcm15602_chip *bcm15602 = dev_get_drvdata(pdev->dev.parent);
	struct bcm15602_gpio *bcm15602_gpio;

	bcm15602_gpio = devm_kzalloc(&pdev->dev, sizeof(*bcm15602_gpio),
				     GFP_KERNEL);
	if (!bcm15602_gpio)
		return -ENOMEM;

	bcm15602_gpio->bcm15602 = bcm15602;
	bcm15602_gpio->gpio_chip = bcm15602_gpio_chip;
	bcm15602_gpio->gpio_chip.dev = &pdev->dev;

	platform_set_drvdata(pdev, bcm15602_gpio);

	return gpiochip_add(&bcm15602_gpio->gpio_chip);
}

static int bcm15602_gpio_remove(struct platform_device *pdev)
{
	struct bcm15602_gpio *bcm15602_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&bcm15602_gpio->gpio_chip);
	return 0;
}

static const struct of_device_id bcm15602_gpio_of_match[] = {
	{ .compatible = "brcm,bcm15602-gpio", },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm15602_gpio_of_match);

static struct platform_driver bcm15602_gpio_driver = {
	.probe = bcm15602_gpio_probe,
	.remove = bcm15602_gpio_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = bcm15602_gpio_of_match,
	},
};
module_platform_driver(bcm15602_gpio_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("BCM15602 GPIO Driver");
MODULE_LICENSE("GPL");
