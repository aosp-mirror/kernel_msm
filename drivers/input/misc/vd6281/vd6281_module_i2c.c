/*
 *  Copyright (C) 2017 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
/*
 * power specific includes
 */
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>

#include "vd6281.h"

static int vd6281_parse_dt(struct device *dev, struct vd6281_data *data)
{
	struct device_node *dt = dev->of_node;

	data->irq_gpio = of_get_named_gpio(dt, "rainbow,int-gpio", 0);
	if (!gpio_is_valid(data->irq_gpio))
		dev_err(dev, "irq_gpio value is not valid\n");
	else
		dev_info(dev, "irq_gpio = %d\n", data->irq_gpio);

	data->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(data->vdd)) {
		data->vdd = NULL;
		dev_err(dev, "Unable to get vdd\n");
	}

	return 0;
}

static int vd6281_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int rc = 0;
	struct vd6281_data *vd6281_data = NULL;
	struct i2c_data *i2c_object = NULL;
	int read_data = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "fail at %s, %d\n", __func__, __LINE__);
		return -EIO;
	}

	vd6281_data = kzalloc(sizeof(struct vd6281_data), GFP_KERNEL);
	if (!vd6281_data)
		return -ENOMEM;

	if (vd6281_data) {
		vd6281_data->client_object =
			kzalloc(sizeof(struct i2c_data), GFP_KERNEL);
		i2c_object = (struct i2c_data *)vd6281_data->client_object;
	}
	i2c_object->client = client;

	if (client->dev.of_node)
		vd6281_parse_dt(&client->dev, vd6281_data);

	/* setup device name */
	vd6281_data->dev_name = dev_name(&client->dev);

	/* setup device data */
	dev_set_drvdata(&client->dev, vd6281_data);

	/* setup client data */
	i2c_set_clientdata(client, vd6281_data);

	/* power up */
	rc = regulator_enable(vd6281_data->vdd);
	if (rc)
		dev_err(&client->dev, "Failed to enable vdd\n");

	/* Test reading device ID at 0x00 */
	read_data = i2c_smbus_read_byte_data(client, 0x00);

	dev_info(&client->dev, "vd6281 device ID or return code: %d (0x%x)\n",
		read_data, read_data);
	if (read_data < 0) {
		dev_err(&client->dev, "i2c read failed: %d\n", read_data);
		rc = read_data;
	}

	/* init default value */
	i2c_object->power_up = 0;

	rc = regulator_disable(vd6281_data->vdd);
	if (rc)
		dev_err(&client->dev, "Failed to disable vdd\n");

	return rc;
}

static int vd6281_remove(struct i2c_client *client)
{
	struct vd6281_data *data = i2c_get_clientdata(client);

	/* Power down the device */
	vd6281_power_down_i2c(data->client_object);
	kfree(data->client_object);
	kfree(data);
	return 0;
}

static const struct i2c_device_id vd6281_id[] = {
	{ VD6281_DRV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, vd6281_id);

static const struct of_device_id st_vd6281_dt_match[] = {
	{ .compatible = "st,vd6281"},
	{ },
};

static struct i2c_driver vd6281_driver = {
	.driver = {
		.name	= VD6281_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = st_vd6281_dt_match,
	},
	.probe	= vd6281_probe,
	.remove	= vd6281_remove,
	.id_table = vd6281_id,

};

int vd6281_power_up_i2c(void *i2c_object, unsigned int *preset_flag)
{
	int ret = 0;
	struct i2c_data *data = (struct i2c_data *)i2c_object;
	struct i2c_client *client = data->client;
	struct vd6281_data *vd6281_data = i2c_get_clientdata(client);

	ret = regulator_enable(vd6281_data->vdd);

	return ret;
}

int vd6281_power_down_i2c(void *i2c_object)
{
	int ret = 0;
	struct i2c_data *data = (struct i2c_data *)i2c_object;
	struct i2c_client *client = data->client;
	struct vd6281_data *vd6281_data = i2c_get_clientdata(client);

	ret = regulator_disable(vd6281_data->vdd);

	return ret;
}

int vd6281_init_i2c(void)
{
	int ret = 0;

	/* register as a i2c client device */
	ret = i2c_add_driver(&vd6281_driver);
	if (ret)
		pr_err("%s: %d erro ret:%d\n", __func__, __LINE__, ret);

	return ret;
}

void vd6281_exit_i2c(void *i2c_object)
{
	i2c_del_driver(&vd6281_driver);
}

