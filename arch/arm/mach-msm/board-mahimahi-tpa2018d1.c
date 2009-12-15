/* drivers/i2c/chips/tpa2018d1.c
 *
 * TI TPA2018D1 Speaker Amplifier
 *
 * Copyright (C) 2009 HTC Corporation
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

/* TODO: content validation in TPA2018_SET_CONFIG */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include "board-mahimahi-tpa2018d1.h"

static struct i2c_client *this_client;
static struct tpa2018d1_platform_data *pdata;
static int is_on;
static const char spk_amp_on[] = {
	0x01, 0xc3, 0x20, 0x01, 0x00, 0x08, 0x1a, 0x21
};
static const char spk_amp_off[] = {0x01, 0xa2};

static int tpa2018_i2c_write(const char *txData, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	if (i2c_transfer(this_client->adapter, msg, 1) < 0) {
		pr_err("%s: I2C transfer error\n", __func__);
		return -EIO;
	} else
		return 0;
}

void tpa2018d1_set_speaker_amp(int on)
{
	if (!pdata) {
		pr_err("%s: no platform data!\n", __func__);
		return;
	}
	if (on && !is_on) {
		gpio_set_value(pdata->gpio_tpa2018_spk_en, 1);
		mdelay(5); /* According to TPA2018D1 Spec */

		if (tpa2018_i2c_write(spk_amp_on, sizeof(spk_amp_on)) == 0) {
			is_on = 1;
			pr_info("%s: ON\n", __func__);
		}
	} else if (!on && is_on) {
		if (tpa2018_i2c_write(spk_amp_off, sizeof(spk_amp_off)) == 0) {
			is_on = 0;
			mdelay(2);
			gpio_set_value(pdata->gpio_tpa2018_spk_en, 0);
			pr_info("%s: OFF\n", __func__);
		}
	}
}

static int tpa2018d1_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	pdata = client->dev.platform_data;

	if (!pdata) {
		ret = -EINVAL;
		pr_err("%s: platform data is NULL\n", __func__);
		goto err_no_pdata;
	}

	this_client = client;

	ret = gpio_request(pdata->gpio_tpa2018_spk_en, "tpa2018");
	if (ret < 0) {
		pr_err("%s: gpio request aud_spk_en pin failed\n", __func__);
		goto err_free_gpio;
	}

	ret = gpio_direction_output(pdata->gpio_tpa2018_spk_en, 1);
	if (ret < 0) {
		pr_err("%s: request aud_spk_en gpio direction failed\n",
			__func__);
		goto err_free_gpio;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto err_free_gpio;
	}

	gpio_set_value(pdata->gpio_tpa2018_spk_en, 0); /* Default Low */

	return 0;

err_free_gpio:
	gpio_free(pdata->gpio_tpa2018_spk_en);
err_no_pdata:
	return ret;
}

static int tpa2018d1_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int tpa2018d1_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tpa2018d1_id[] = {
	{ TPA2018D1_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver tpa2018d1_driver = {
	.probe = tpa2018d1_probe,
	.suspend = tpa2018d1_suspend,
	.resume = tpa2018d1_resume,
	.id_table = tpa2018d1_id,
	.driver = {
		.name = TPA2018D1_I2C_NAME,
	},
};

static int __init tpa2018d1_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&tpa2018d1_driver);
}

module_init(tpa2018d1_init);

MODULE_DESCRIPTION("tpa2018d1 speaker amp driver");
MODULE_LICENSE("GPL");
