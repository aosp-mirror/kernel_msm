/* Copyright (c) 2017, LGE Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/leds.h>
#include <linux/platform_data/leds-lm36272.h>

#define LM36272_BL_DEV "lcd-backlight"
#define I2C_BL_NAME "leds_lm36272"

#define LM36272_BRT_LSB_MASK	(BIT(0) | BIT(1) | BIT(2))
#define LM36272_BRT_MSB_SHIFT	3
#define MAX_BRIGHTNESS_LM36272	255
#define MIN_BRIGHTNESS_LM36272	0
#define BL_ON	1
#define BL_OFF	0

#define DSV_OFF 0
#define DSV_ON  1

struct lm36272_device {
	struct led_classdev led_dev;
	struct i2c_client *client;
	int bl_gpio;
	int dsv_p_gpio;
	int dsv_n_gpio;
	int min_brightness;
	int max_brightness;
	int default_brightness;
	int status;
	struct mutex bl_mutex;
	int blmap_size;
	u16 *blmap;
};

static const struct i2c_device_id lm36272_bl_id[] = {
	{ I2C_BL_NAME, 0 },
	{ },
};

static int lm36272_write_reg(struct i2c_client *client,
		unsigned char reg, unsigned char val);

static struct lm36272_device *this;

int lm36272_dsv_ctrl(int dsv_en)
{
	struct lm36272_device *dev = this;

	if (!dev) {
		pr_err("%s : lm36272 is not ready\n", __func__);
		return -ENODEV;
	}

	switch (dsv_en) {
	case DSV_ON:
		lm36272_write_reg(dev->client, 0x09, 0x99);
		lm36272_write_reg(dev->client, 0x0C, 0x28);
		lm36272_write_reg(dev->client, 0x0D, 0x1E);
		lm36272_write_reg(dev->client, 0x0E, 0x1E);
		usleep_range(1000, 1000);

		if (gpio_is_valid(dev->dsv_p_gpio))
			gpio_set_value_cansleep(dev->dsv_p_gpio, 1);
		usleep_range(2000, 2000);

		if (gpio_is_valid(dev->dsv_n_gpio))
			gpio_set_value_cansleep(dev->dsv_n_gpio, 1);
		usleep_range(12000, 12000);
		break;
	case DSV_OFF:
	default:
		if (gpio_is_valid(dev->dsv_n_gpio))
			gpio_set_value_cansleep(dev->dsv_n_gpio, 0);
		usleep_range(500 , 500);
		if (gpio_is_valid(dev->dsv_p_gpio))
			gpio_set_value_cansleep(dev->dsv_p_gpio, 0);
		usleep_range(2000, 2000);
		break;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(lm36272_dsv_ctrl);

static int lm36272_write_reg(struct i2c_client *client,
		unsigned char reg, unsigned char val)
{
	int err;
	u8 buf[2];
	struct i2c_msg msg = {
		client->addr,
		0,
		2,
		buf
	};

	buf[0] = reg;
	buf[1] = val;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err < 0)
		dev_err(&client->dev, "i2c write error reg: 0x%x, val: 0x%x\n",
				buf[0], buf[1]);
	return 0;
}

static void lm36272_set_main_current_level(struct i2c_client *client, int level)
{
	struct lm36272_device *dev = i2c_get_clientdata(client);
	int min_brightness = dev->min_brightness;
	int max_brightness = dev->max_brightness;
	int cal_value = level;
	u8 data;

	if (level != 0) {
		if (level > 0 && level <= min_brightness)
			level = min_brightness;
		else if (level > max_brightness)
			level = max_brightness;
		if (dev->blmap) {
			if (level < dev->blmap_size) {
				cal_value = dev->blmap[level];
				data = cal_value & LM36272_BRT_LSB_MASK;
				lm36272_write_reg(client, 0x04, data);
				data = (cal_value >> LM36272_BRT_MSB_SHIFT) & 0xFF;
				lm36272_write_reg(client, 0x05, data);
			} else
				dev_warn(&client->dev, "invalid index %d:%d\n",
						dev->blmap_size,
						level);
		} else {
			lm36272_write_reg(client, 0x05, cal_value);
		}
	} else {
		lm36272_write_reg(client, 0x04, cal_value);
		lm36272_write_reg(client, 0x05, cal_value);
	}

	if (dev->status == BL_OFF)
		lm36272_write_reg(dev->client, 0x08, 0x13);

	pr_debug("%s: level=%d, cal_value=%d \n",
				__func__, level, cal_value);
}

void lm36272_backlight_ctrl(int level)
{
	struct lm36272_device *dev = this;

	if (!dev) {
		pr_err("%s : lm36272 is not ready\n", __func__);
		return;
	}

	if (level == 0) {
		if (dev->status == BL_OFF)
			return;

		lm36272_set_main_current_level(dev->client, 0);
		lm36272_write_reg(dev->client, 0x08, 0x00);
		dev->status = BL_OFF;
	} else {
		if (dev->status == BL_OFF)
			lm36272_write_reg(dev->client, 0x02, 0x60);

		usleep_range(1000, 1000);
		lm36272_set_main_current_level(dev->client, level);
		dev->status = BL_ON;
	}
}
EXPORT_SYMBOL_GPL(lm36272_backlight_ctrl);

static void lm36272_lcd_backlight_set_level(struct led_classdev *led_cdev,
		enum led_brightness level)
{
	struct lm36272_device *dev =
	    container_of(led_cdev, struct lm36272_device, led_dev);

	if (!dev)
		return;

	if (level > MAX_BRIGHTNESS_LM36272)
		level = MAX_BRIGHTNESS_LM36272;

	mutex_lock(&dev->bl_mutex);
	lm36272_backlight_ctrl(level);
	mutex_unlock(&dev->bl_mutex);
}

static int lm36272_parse_dt(struct device *dev,
		struct lm36272_platform_data *pdata)
{
	int rc = 0, i;
	u32 *array;
	struct device_node *np = dev->of_node;

	pdata->dsv_p_gpio = of_get_named_gpio(np, "lm36272,dsv_p_en", 0);
	pdata->dsv_n_gpio = of_get_named_gpio(np, "lm36272,dsv_n_en", 0);
	rc = of_property_read_u32(np, "lm36272,min_brightness",
			&pdata->min_brightness);
	rc = of_property_read_u32(np, "lm36272,default_brightness",
			&pdata->default_brightness);
	rc = of_property_read_u32(np, "lm36272,max_brightness",
			&pdata->max_brightness);
	rc = of_property_read_u32(np, "lm36272,blmap_size",
			&pdata->blmap_size);

	if (pdata->blmap_size) {
		array = kzalloc(sizeof(u32) * pdata->blmap_size, GFP_KERNEL);
		if (!array)
			return -ENOMEM;

		rc = of_property_read_u32_array(np, "lm36272,blmap",
				array, pdata->blmap_size);
		if (rc) {
			pr_err("%s:%d, unable to read backlight map\n",
					__func__, __LINE__);
			if (array)
				kfree(array);
			return -EINVAL;
		}
		pdata->blmap = kzalloc(sizeof(u16) * pdata->blmap_size,
				GFP_KERNEL);
		if (!pdata->blmap) {
			if (array)
				kfree(array);
			return -ENOMEM;
		}

		for (i = 0; i < pdata->blmap_size; i++ )
			pdata->blmap[i] = (u16)array[i];

		if (array)
			kfree(array);

	} else {
		pdata->blmap = NULL;
	}


	pr_info("%s bl_gpio : %d, dsv_p_gpio : %d, dsv_n_gpio : %d\n",
			__func__,
			pdata->bl_gpio, pdata->dsv_p_gpio, pdata->dsv_n_gpio);
	pr_info("%s min: %d, default: %d, max: %d, blmap_size : %d\n",
			__func__,
			pdata->min_brightness, pdata->default_brightness,
			pdata->max_brightness, pdata->blmap_size);

	return rc;
}

static int lm36272_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct lm36272_platform_data *pdata;
	struct lm36272_device *dev;
	int err;
	int i;

	if (&client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct lm36272_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		err = lm36272_parse_dt(&client->dev, pdata);
		if (err != 0) {
			pr_err ("%s parse fail err = %d\n",__func__,err);
			return err;
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (gpio_request(pdata->dsv_p_gpio, "lm36272_dsv_p") != 0) {
		pr_err("%s request dsv_p_gpio failed\n",__func__);
		return -ENODEV;
	}

	if (gpio_request(pdata->dsv_n_gpio, "lm36272_dsv_n") != 0) {
		pr_err("%s request dsv_n_gpio failed\n",__func__);
		return -ENODEV;
	}

	dev = devm_kzalloc(&client->dev,
			sizeof(struct lm36272_device),
			GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&client->dev, "fail alloc for lm36272_device\n");
		return 0;
	}

	this = dev;
	dev->client = client;

	dev->dsv_p_gpio = pdata->dsv_p_gpio;
	dev->dsv_n_gpio = pdata->dsv_n_gpio;
	dev->min_brightness = pdata->min_brightness;
	if (dev->min_brightness < MIN_BRIGHTNESS_LM36272)
		dev->min_brightness = dev->min_brightness;
	dev->max_brightness = pdata->max_brightness;
	if (dev->max_brightness > MAX_BRIGHTNESS_LM36272)
		dev->max_brightness = MAX_BRIGHTNESS_LM36272;
	dev->default_brightness = pdata->default_brightness;
	dev->blmap_size = pdata->blmap_size;

	dev->led_dev.name= LM36272_BL_DEV;
	dev->led_dev.brightness_set = lm36272_lcd_backlight_set_level;
	dev->led_dev.max_brightness = MAX_BRIGHTNESS_LM36272;

	if (dev->blmap_size) {
		dev->blmap = kzalloc(sizeof(u16) * dev->blmap_size, GFP_KERNEL);
		if (!dev->blmap) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		for (i = 0; i < dev->blmap_size; i++ )
			dev->blmap[i]= (u16)pdata->blmap[i];
	} else {
		dev->blmap = NULL;
	}

	i2c_set_clientdata(client, dev);

	mutex_init(&dev->bl_mutex);

	err = led_classdev_register(&client->dev, &dev->led_dev);
	if (err < 0) {
		dev_err(&client->dev, "Register led class failed: %d\n", err);
		return err;
	}

	/* set the default brightness */
	lm36272_lcd_backlight_set_level(&dev->led_dev, dev->default_brightness);

	dev_info(&client->dev, "probe done\n");

	return 0;
}

static int lm36272_remove(struct i2c_client *client)
{
	struct lm36272_device *dev = i2c_get_clientdata(client);

	this = NULL;
	led_classdev_unregister(&dev->led_dev);

	if (gpio_is_valid(dev->dsv_n_gpio))
		gpio_free(dev->dsv_n_gpio);
	if (gpio_is_valid(dev->dsv_p_gpio))
		gpio_free(dev->dsv_p_gpio);

	return 0;
}

static struct of_device_id lm36272_match_table[] = {
	{ .compatible = "leds,lm36272",},
	{ },
};

static struct i2c_driver main_lm36272_driver = {
	.probe = lm36272_probe,
	.remove = lm36272_remove,
	.suspend = NULL,
	.resume = NULL,
	.id_table = lm36272_bl_id,
	.driver = {
		.name = I2C_BL_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm36272_match_table,
	},
};

static int __init lcd_backlight_init(void)
{
	static int err;

	err = i2c_add_driver(&main_lm36272_driver);

	return err;
}

module_init(lcd_backlight_init);
MODULE_DESCRIPTION("Texas Instruments Backlight+LCD bias driver for lm36272");
MODULE_AUTHOR("Yongtae Kim <yongtae3.kim@lge.com>");
MODULE_LICENSE("GPL v2");
