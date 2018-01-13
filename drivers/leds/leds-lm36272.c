/* Copyright (c) 2017-2018, LGE Inc. All rights reserved.
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
#include <linux/pm_wakeup.h>

#define LM36272_BL_DEV "lcd-backlight"
#define I2C_BL_NAME "leds_lm36272"

#define LM36272_BRT_LSB_MASK	(BIT(0) | BIT(1) | BIT(2))
#define LM36272_BRT_MSB_SHIFT	3
#define MAX_BRIGHTNESS_LM36272	255
#define MIN_BRIGHTNESS_LM36272	0
#define DIM_BRIGHTENSS_LM36272  1

#define DSV_PRE_ON_DEALY_US 1000
#define DSV_P_ON_DELAY_US   2000
#define DSV_N_ON_DELAY_US   1000
#define DSV_P_OFF_DELAY_US  2000
#define DSV_N_OFF_DELAY_US  500

enum {
	BL_OFF = 0,
	BL_ON,
	BL_OFF_PENDING,
	BL_DIM_PENDING
};

enum {
	DSV_OFF = 0,
	DSV_ON
};

struct lm36272_device {
	struct led_classdev led_dev;
	struct i2c_client *client;
#ifdef CONFIG_FB
	struct notifier_block fb_notifier;
#endif
	int fb_blank;
	int dsv_p_gpio;
	int dsv_n_gpio;
	u32 *dsv_on_delay;
	u32 *dsv_off_delay;
	int min_brightness;
	int max_brightness;
	int dim_brightness;
	bool dim_enabled;
	int default_brightness;
	int status;
	struct mutex bl_mutex;
	int blmap_size;
	u16 *blmap;

	struct work_struct bl_work;
	int level_saved;
	int level_target;
	struct wakeup_source ws_dev;
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

static void lm36272_backlight_ctrl_internal(struct lm36272_device *ldev,
		int level)
{
	if (0 == level) {
		if (ldev->status == BL_OFF) {
			mutex_unlock(&ldev->bl_mutex);
			return;
		}

		lm36272_set_main_current_level(ldev->client, 0);
		lm36272_write_reg(ldev->client, 0x08, 0x00);
		ldev->status = BL_OFF;
		pr_info("backlight off\n");
	} else {
		if (ldev->status == BL_OFF)
			lm36272_write_reg(ldev->client, 0x02, 0x60);

		usleep_range(1000, 1000);
		lm36272_set_main_current_level(ldev->client, level);
		ldev->status = BL_ON;
		if (ldev->dim_enabled && ldev->dim_brightness == level)
			pr_info("backlight dim\n");
	}
	ldev->level_saved = level;
}

void lm36272_backlight_ctrl(int level)
{
	struct lm36272_device *ldev = this;

	if (!ldev) {
		pr_err("%s: lm36272 is not ready\n", __func__);
		return;
	}

	mutex_lock(&ldev->bl_mutex);
	if (0 == level &&
	    (FB_BLANK_NORMAL != ldev->fb_blank ||
	     FB_BLANK_POWERDOWN != ldev->fb_blank)) {
		ldev->status = BL_OFF_PENDING;
		pr_info("pending backlight off\n");
	} else if (!ldev->dim_enabled) {
		lm36272_backlight_ctrl_internal(ldev, level);
	} else if (ldev->dim_brightness == level &&
		   FB_BLANK_VSYNC_SUSPEND != ldev->fb_blank) {
		ldev->status = BL_DIM_PENDING;
		pr_info("pending backlight dim\n");
	} else {
		if (level >= (ldev->level_saved - 5)) {
			mutex_unlock(&ldev->bl_mutex);
			cancel_work_sync(&ldev->bl_work);
			mutex_lock(&ldev->bl_mutex);
			lm36272_backlight_ctrl_internal(ldev, level);
		} else {
			mutex_unlock(&ldev->bl_mutex);
			ldev->level_target = level;
			schedule_work(&ldev->bl_work);
			mutex_lock(&ldev->bl_mutex);
		}
	}
	mutex_unlock(&ldev->bl_mutex);
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

	lm36272_backlight_ctrl(level);
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

	rc = of_property_read_u32(np, "lm36272,dim-brightness",
			&pdata->dim_brightness);
	if (rc)
		pdata->dim_brightness = DIM_BRIGHTENSS_LM36272;

	pdata->dim_enabled = of_property_read_bool(np, "lm36272,dim-enabled");

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

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct lm36272_device *ldev;
	struct fb_event *evdata = data;
	int fb_blank = 0;

	if (!evdata) {
		pr_err("%s: event data not available\n", __func__);
		return NOTIFY_BAD;
	}

	if (FB_EVENT_BLANK != event)
		return 0;

	ldev = container_of(self, struct lm36272_device, fb_notifier);

	mutex_lock(&ldev->bl_mutex);
	fb_blank = *(int *)evdata->data;

	if (BL_OFF_PENDING == ldev->status) {
		if (FB_BLANK_NORMAL == fb_blank ||
		    FB_BLANK_POWERDOWN == fb_blank) {
			/* backlight off */
			lm36272_backlight_ctrl_internal(ldev, 0);
		} else {
			ldev->status = BL_ON;
		}
	} else if (ldev->dim_enabled && BL_DIM_PENDING == ldev->status) {
		if (FB_BLANK_VSYNC_SUSPEND == fb_blank) {
			/* backlight dim */
			ldev->level_target = 1;
			schedule_work(&ldev->bl_work);
		} else {
			ldev->status = BL_ON;
		}
	}
	ldev->fb_blank = fb_blank;
	mutex_unlock(&ldev->bl_mutex);

	return 0;
}
#endif

static void lm36272_bl_work(struct work_struct *work)
{
	struct lm36272_device *ldev = container_of(work,
			struct lm36272_device, bl_work);
	int level;
	bool exit = false;

	mutex_lock(&ldev->bl_mutex);
	__pm_stay_awake(&ldev->ws_dev);
	level = ldev->level_saved;

	while (level > ldev->level_target) {
		level -= 5;
		if (level <= ldev->level_target) {
			level = ldev->level_target;
			exit = true;
		}

		lm36272_backlight_ctrl_internal(ldev, level);
		if (exit)
			break;
		usleep_range(9000,10000);
	}
	__pm_relax(&ldev->ws_dev);
	mutex_unlock(&ldev->bl_mutex);
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
	ldev->dim_brightness = clamp(pdata->dim_brightness,
				     MIN_BRIGHTNESS_LM36272,
				     MAX_BRIGHTNESS_LM36272);
	ldev->dim_enabled = pdata->dim_enabled;
	ldev->default_brightness = pdata->default_brightness;
	ldev->blmap_size = pdata->blmap_size;

	ldev->led_dev.name= LM36272_BL_DEV;
	ldev->led_dev.brightness_set = lm36272_lcd_backlight_set_level;
	ldev->led_dev.max_brightness = MAX_BRIGHTNESS_LM36272;

	if (ldev->blmap_size)
		ldev->blmap = pdata->blmap;

	i2c_set_clientdata(client, ldev);

	mutex_init(&ldev->bl_mutex);

#ifdef CONFIG_FB
	ldev->fb_notifier.notifier_call = fb_notifier_callback;
	err = fb_register_client(&ldev->fb_notifier);
	if (err < 0) {
		dev_err(&client->dev, "Failed to register fb client\n");
		goto err_fb_client;
	}
#endif

	INIT_WORK(&ldev->bl_work, lm36272_bl_work);
	wakeup_source_init(&ldev->ws_dev, "backlight_ws_dev");

	err = led_classdev_register(&client->dev, &ldev->led_dev);
	if (err < 0) {
		dev_err(&client->dev, "Failed to register led class\n");
		goto err_led_classdev;
	}

	/* set the default brightness */
	lm36272_lcd_backlight_set_level(&ldev->led_dev,
			ldev->default_brightness);

	dev_info(&client->dev, "probe done\n");

	return 0;

err_led_classdev:
#ifdef CONFIG_FB
	fb_unregister_client(&ldev->fb_notifier);
err_fb_client:
#endif
	mutex_destroy(&ldev->bl_mutex);
	this = NULL;

	return err;
}

static int lm36272_remove(struct i2c_client *client)
{
	struct lm36272_device *ldev = i2c_get_clientdata(client);

	led_classdev_unregister(&ldev->led_dev);
#ifdef CONFIG_FB
	fb_unregister_client(&ldev->fb_notifier);
#endif
	mutex_destroy(&ldev->bl_mutex);
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
