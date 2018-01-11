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

#define DSV_PRE_ON_DEALY_US 1000
#define DSV_P_ON_DELAY_US   2000
#define DSV_N_ON_DELAY_US   1000
#define DSV_P_OFF_DELAY_US  2000
#define DSV_N_OFF_DELAY_US  500

enum {
	BL_OFF = 0,
	BL_ON
};

enum {
	DSV_OFF = 0,
	DSV_ON
};

struct lm36272_device {
	struct led_classdev led_dev;
	struct i2c_client *client;
	int dsv_p_gpio;
	int dsv_n_gpio;
	u32 *dsv_on_delay;
	u32 *dsv_off_delay;
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
	struct lm36272_device *ldev = this;

	if (!ldev) {
		pr_warn("%s: lm36272 is not ready\n", __func__);
		return -ENODEV;
	}

	if (dsv_en) {
		lm36272_write_reg(ldev->client, 0x09, 0x99);
		lm36272_write_reg(ldev->client, 0x0C, 0x28);
		lm36272_write_reg(ldev->client, 0x0D, 0x1E);
		lm36272_write_reg(ldev->client, 0x0E, 0x1E);
		usleep_range(ldev->dsv_on_delay[0], ldev->dsv_on_delay[0]);

		gpio_set_value_cansleep(ldev->dsv_p_gpio, 1);
		usleep_range(ldev->dsv_on_delay[1], ldev->dsv_on_delay[1]);

		gpio_set_value_cansleep(ldev->dsv_n_gpio, 1);
		usleep_range(ldev->dsv_on_delay[2], ldev->dsv_on_delay[2]);
	} else {
		gpio_set_value_cansleep(ldev->dsv_n_gpio, 0);
		usleep_range(ldev->dsv_off_delay[0], ldev->dsv_off_delay[0]);

		gpio_set_value_cansleep(ldev->dsv_p_gpio, 0);
		usleep_range(ldev->dsv_off_delay[1], ldev->dsv_off_delay[1]);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(lm36272_dsv_ctrl);

static int lm36272_write_reg(struct i2c_client *client,
		unsigned char reg, unsigned char val)
{
	int err;
	u8 buf[2] = {reg, val};
	struct i2c_msg msg = {
		client->addr,
		0,
		2,
		buf
	};

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err < 0)
		dev_err(&client->dev, "i2c write error reg: 0x%x, val: 0x%x\n",
				buf[0], buf[1]);
	return 0;
}

static void lm36272_set_main_current_level(struct i2c_client *client, int level)
{
	struct lm36272_device *ldev = i2c_get_clientdata(client);
	int min_brightness = ldev->min_brightness;
	int max_brightness = ldev->max_brightness;
	int cal_value = level;
	u8 data;

	if (level != 0) {
		if (level > 0 && level <= min_brightness)
			level = min_brightness;
		else if (level > max_brightness)
			level = max_brightness;
		if (ldev->blmap) {
			if (level < ldev->blmap_size) {
				cal_value = ldev->blmap[level];
				data = cal_value & LM36272_BRT_LSB_MASK;
				lm36272_write_reg(client, 0x04, data);
				data = (cal_value >> LM36272_BRT_MSB_SHIFT) & 0xFF;
				lm36272_write_reg(client, 0x05, data);
			} else
				dev_warn(&client->dev, "invalid index %d:%d\n",
						ldev->blmap_size,
						level);
		} else {
			lm36272_write_reg(client, 0x05, cal_value);
		}
	} else {
		lm36272_write_reg(client, 0x04, cal_value);
		lm36272_write_reg(client, 0x05, cal_value);
	}

	if (ldev->status == BL_OFF)
		lm36272_write_reg(ldev->client, 0x08, 0x13);

	pr_debug("%s: level=%d, cal_value=%d \n",
				__func__, level, cal_value);
}

void lm36272_backlight_ctrl(int level)
{
	struct lm36272_device *ldev = this;

	if (!ldev) {
		pr_err("%s: lm36272 is not ready\n", __func__);
		return;
	}

	if (level == 0) {
		if (ldev->status == BL_OFF)
			return;

		lm36272_set_main_current_level(ldev->client, 0);
		lm36272_write_reg(ldev->client, 0x08, 0x00);
		ldev->status = BL_OFF;
	} else {
		if (ldev->status == BL_OFF)
			lm36272_write_reg(ldev->client, 0x02, 0x60);

		usleep_range(1000, 1000);
		lm36272_set_main_current_level(ldev->client, level);
		ldev->status = BL_ON;
	}
}
EXPORT_SYMBOL_GPL(lm36272_backlight_ctrl);

static void lm36272_lcd_backlight_set_level(struct led_classdev *led_cdev,
		enum led_brightness level)
{
	struct lm36272_device *ldev =
	    container_of(led_cdev, struct lm36272_device, led_dev);

	if (!ldev)
		return;

	if (level > MAX_BRIGHTNESS_LM36272)
		level = MAX_BRIGHTNESS_LM36272;

	mutex_lock(&ldev->bl_mutex);
	lm36272_backlight_ctrl(level);
	mutex_unlock(&ldev->bl_mutex);
}

static int lm36272_parse_dt(struct device *dev,
		struct lm36272_platform_data *pdata)
{
	int rc = 0, i;
	u32 *array = NULL;
	struct device_node *np = dev->of_node;

	pdata->dsv_p_gpio = of_get_named_gpio(np, "lm36272,gpio-dsv-p-en", 0);
	if (pdata->dsv_p_gpio < 0) {
		pr_err("%s: no gpio-dsv-p-en in device tree\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	pdata->dsv_n_gpio = of_get_named_gpio(np, "lm36272,gpio-dsv-n-en", 0);
	if (pdata->dsv_n_gpio < 0) {
		pr_err("%s: no gpio-dsv-n-en in device tree\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	rc = of_property_read_u32_array(np, "lm36272,dsv-on-delay",
			pdata->dsv_on_delay, 3);
	if (rc) {
		pdata->dsv_on_delay[0] = DSV_PRE_ON_DEALY_US;
		pdata->dsv_on_delay[1] = DSV_P_ON_DELAY_US;
		pdata->dsv_on_delay[2] = DSV_N_ON_DELAY_US;
	}

	rc = of_property_read_u32_array(np, "lm36272,dsv-off-delay",
			pdata->dsv_off_delay, 2);
	if (rc) {
		pdata->dsv_off_delay[0] = DSV_N_OFF_DELAY_US;
		pdata->dsv_off_delay[1] = DSV_P_OFF_DELAY_US;
	}

	rc = of_property_read_u32(np, "lm36272,min-brightness",
			&pdata->min_brightness);
	if (rc)
		pdata->min_brightness = MIN_BRIGHTNESS_LM36272;

	rc = of_property_read_u32(np, "lm36272,default-brightness",
			&pdata->default_brightness);
	if (rc)
		pdata->default_brightness = MAX_BRIGHTNESS_LM36272;

	rc = of_property_read_u32(np, "lm36272,max-brightness",
			&pdata->max_brightness);
	if (rc)
		pdata->max_brightness = MAX_BRIGHTNESS_LM36272;

	rc = of_property_read_u32(np, "lm36272,blmap-size",
			&pdata->blmap_size);
	if (!rc && pdata->blmap_size) {
		if (pdata->blmap_size > 256) {
			pr_err("%s: blmap size is too big %d\n", __func__,
					pdata->blmap_size);
			rc = -EINVAL;
			goto out;
		}
		array = devm_kzalloc(dev, sizeof(u32) * pdata->blmap_size,
				GFP_KERNEL);
		if (!array) {
			pr_err("%s: no memroy\n", __func__);
			rc = -ENOMEM;
			goto out;
		}

		rc = of_property_read_u32_array(np, "lm36272,blmap",
				array, pdata->blmap_size);
		if (rc) {
			pr_err("%s: unable to read backlight map\n",
					__func__);
			goto out;
		}
		pdata->blmap = devm_kzalloc(dev,
				sizeof(u16) * pdata->blmap_size,
				GFP_KERNEL);
		if (!pdata->blmap) {
			rc = -ENOMEM;
			goto out;
		}

		for (i = 0; i < pdata->blmap_size; i++ )
			pdata->blmap[i] = (u16)array[i];

		devm_kfree(dev, array);

	}

	pr_info("%s: dsv_p_gpio : %d, dsv_n_gpio : %d\n",
			__func__,
			pdata->dsv_p_gpio, pdata->dsv_n_gpio);
	pr_info("%s: min: %d, default: %d, max: %d, blmap_size : %d\n",
			__func__,
			pdata->min_brightness, pdata->default_brightness,
			pdata->max_brightness, pdata->blmap_size);
	return 0;

out:
	return rc;
}

static int lm36272_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct lm36272_platform_data *pdata;
	struct lm36272_device *ldev;
	int err;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct lm36272_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		err = lm36272_parse_dt(&client->dev, pdata);
		if (err) {
			pr_err ("%s: parse failed err = %d\n", __func__, err);
			return err;
		}
	} else {
		pdata = client->dev.platform_data;
	}

	/*
	 * GPIO direction has been set by bootloader.
	 * We don't need to set here, so just request only.
	 */
	err = devm_gpio_request(&client->dev, pdata->dsv_p_gpio,
			"lm36272_dsv_p");
	if (err) {
		pr_err("%s: Failed to request dsv_p_gpio\n",__func__);
		return err;
	}

	err = devm_gpio_request(&client->dev, pdata->dsv_n_gpio,
			"lm36272_dsv_n");
	if (err) {
		pr_err("%s: Failed to request dsv_n_gpio\n",__func__);
		return err;
	}

	ldev = devm_kzalloc(&client->dev,
			sizeof(struct lm36272_device),
			GFP_KERNEL);
	if (ldev == NULL) {
		dev_err(&client->dev, "Failed to alloc memory\n");
		return -ENOMEM;
	}

	this = ldev;
	ldev->client = client;

	ldev->dsv_p_gpio = pdata->dsv_p_gpio;
	ldev->dsv_n_gpio = pdata->dsv_n_gpio;
	ldev->dsv_on_delay = pdata->dsv_on_delay;
	ldev->dsv_off_delay = pdata->dsv_off_delay;
	ldev->min_brightness = pdata->min_brightness;
	if (ldev->min_brightness < MIN_BRIGHTNESS_LM36272)
		ldev->min_brightness = MIN_BRIGHTNESS_LM36272;
	ldev->max_brightness = pdata->max_brightness;
	if (ldev->max_brightness > MAX_BRIGHTNESS_LM36272)
		ldev->max_brightness = MAX_BRIGHTNESS_LM36272;
	ldev->default_brightness = pdata->default_brightness;
	ldev->blmap_size = pdata->blmap_size;

	ldev->led_dev.name= LM36272_BL_DEV;
	ldev->led_dev.brightness_set = lm36272_lcd_backlight_set_level;
	ldev->led_dev.max_brightness = MAX_BRIGHTNESS_LM36272;

	if (ldev->blmap_size)
		ldev->blmap = pdata->blmap;

	i2c_set_clientdata(client, ldev);

	mutex_init(&ldev->bl_mutex);

	err = led_classdev_register(&client->dev, &ldev->led_dev);
	if (err < 0) {
		dev_err(&client->dev, "Failed to register led class\n");
		return err;
	}

	/* set the default brightness */
	lm36272_lcd_backlight_set_level(&ldev->led_dev,
			ldev->default_brightness);

	dev_info(&client->dev, "probe done\n");

	return 0;
}

static int lm36272_remove(struct i2c_client *client)
{
	struct lm36272_device *ldev = i2c_get_clientdata(client);

	led_classdev_unregister(&ldev->led_dev);
	this = NULL;

	return 0;
}

static struct of_device_id lm36272_match_table[] = {
	{ .compatible = "leds,lm36272",},
	{ },
};

static struct i2c_driver main_lm36272_driver = {
	.probe = lm36272_probe,
	.remove = lm36272_remove,
	.id_table = lm36272_bl_id,
	.driver = {
		.name = I2C_BL_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm36272_match_table,
	},
};

static int __init lcd_backlight_init(void)
{
	return i2c_add_driver(&main_lm36272_driver);
}

module_init(lcd_backlight_init);
MODULE_DESCRIPTION("Texas Instruments LM36272 driver");
MODULE_AUTHOR("Yongtae Kim <yongtae3.kim@lge.com>");
MODULE_LICENSE("GPL v2");
