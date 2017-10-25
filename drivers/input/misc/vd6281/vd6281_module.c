/*
 *  vd6281_module.c - Linux kernel module for rainbow sensor
 *
 *  Copyright (C) 2017 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>


#define VD6281_DRV_NAME	"vd6281"

struct vd6281_ctrl_t {
	struct i2c_client *client;
	struct regulator *vdd;
	const char *dev_name;
	int irq_gpio;
	struct mutex work_mutex;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	int is_power_up;
};

static int vd6281_power_up(struct vd6281_ctrl_t *ctrl)
{
	return regulator_enable(ctrl->vdd);
}

static int vd6281_power_down(struct vd6281_ctrl_t *ctrl)
{
	return regulator_disable(ctrl->vdd);
}

static ssize_t rainbow_enable_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct vd6281_ctrl_t *ctrl = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->is_power_up);
}

static ssize_t rainbow_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct vd6281_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc;
	bool value;

	rc = kstrtobool(buf, &value);

	if (rc)
		return rc;

	mutex_lock(&ctrl->work_mutex);

	if (value == 1)
		rc = vd6281_power_up(ctrl);
	else
		rc = vd6281_power_down(ctrl);

	if (rc == 0)
		ctrl->is_power_up = (int)value;

	mutex_unlock(&ctrl->work_mutex);

	return rc;
}

static ssize_t rainbow_read_byte_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct vd6281_ctrl_t *ctrl = dev_get_drvdata(dev);
	unsigned long value;
	int rc;

	rc = kstrtoul(buf, 0, &value);

	if (rc)
		return rc;

	mutex_lock(&ctrl->work_mutex);

	rc = i2c_smbus_read_byte_data(ctrl->client, value & 0xFF);

	if (rc < 0)
		dev_err(dev, "%s I2C read failed: %d.", __func__, rc);

	mutex_unlock(&ctrl->work_mutex);

	dev_dbg(dev, "I2C read addr: 0x%lx, data: 0x%x", value, rc);

	return rc;
}

static ssize_t rainbow_write_byte_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct vd6281_ctrl_t *ctrl = dev_get_drvdata(dev);
	unsigned long value;
	int rc;
	u8 addr;
	u8 data;

	rc = kstrtoul(buf, 0, &value);

	if (rc)
		return rc;

	addr = (value >> 8) & 0xFF;
	data = value & 0xFF;

	mutex_lock(&ctrl->work_mutex);

	rc = i2c_smbus_write_byte_data(ctrl->client, addr, data);

	if (rc < 0)
		dev_err(dev, "%s I2C write failed: %d.", __func__, rc);

	mutex_unlock(&ctrl->work_mutex);

	dev_dbg(dev, "I2C write addr: 0x%x, data: 0x%x", addr, data);

	return rc;
}

static DEVICE_ATTR_RW(rainbow_enable);
static DEVICE_ATTR_WO(rainbow_read_byte);
static DEVICE_ATTR_WO(rainbow_write_byte);

static struct attribute *rainbow_dev_attrs[] = {
	&dev_attr_rainbow_enable.attr,
	&dev_attr_rainbow_read_byte.attr,
	&dev_attr_rainbow_write_byte.attr,
	NULL
};

ATTRIBUTE_GROUPS(rainbow_dev);

static int vd6281_parse_dt(struct device *dev, struct vd6281_ctrl_t *ctrl)
{
	struct device_node *dt = dev->of_node;
	int rc = 0;

	ctrl->irq_gpio = of_get_named_gpio(dt, "rainbow,int-gpio", 0);
	if (!gpio_is_valid(ctrl->irq_gpio))
		dev_warn(dev, "irq_gpio value is not valid\n");

	ctrl->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ctrl->vdd)) {
		ctrl->vdd = NULL;
		dev_err(dev, "Unable to get vdd\n");
		rc = -ENOENT;
	}

	return rc;
}

static int vd6281_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int rc = 0;
	struct vd6281_ctrl_t *vd6281_ctrl = NULL;
	int read_data = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "i2c_check_functionality failed\n");
		return -EIO;
	}

	vd6281_ctrl = kzalloc(sizeof(struct vd6281_ctrl_t), GFP_KERNEL);
	if (!vd6281_ctrl)
		return -ENOMEM;

	vd6281_ctrl->client = client;

	if (client->dev.of_node)
		rc = vd6281_parse_dt(&client->dev, vd6281_ctrl);

	if (rc)
		goto error;

	/* setup device name */
	vd6281_ctrl->dev_name = dev_name(&client->dev);

	/* setup device data */
	dev_set_drvdata(&client->dev, vd6281_ctrl);

	/* setup client data */
	i2c_set_clientdata(client, vd6281_ctrl);

	rc = vd6281_power_up(vd6281_ctrl);
	if (rc < 0) {
		dev_err(&client->dev, "failed to power up\n");
		goto error;
	}

	/* Test reading device ID at 0x00 */
	read_data = i2c_smbus_read_byte_data(client, 0x00);

	dev_dbg(&client->dev, "device ID or return code: 0x%x(%d)",
		read_data, read_data);

	if (read_data < 0) {
		dev_err(&client->dev,
			"i2c read failed: %d maybe due to HW delay up to 10 us",
			read_data);
	}

	rc = vd6281_power_down(vd6281_ctrl);
	if (rc < 0) {
		dev_err(&client->dev, "failed to power down\n");
		goto error;
	}

	rc = sysfs_create_groups(&client->dev.kobj, rainbow_dev_groups);
	if (rc) {
		dev_err(&client->dev, "failed to create sysfs files");
		goto error;
	}

	mutex_init(&vd6281_ctrl->work_mutex);

	return rc;

error:
	kfree(vd6281_ctrl);

	return rc;
}

static int vd6281_remove(struct i2c_client *client)
{
	struct vd6281_ctrl_t *ctrl = i2c_get_clientdata(client);

	sysfs_remove_groups(&client->dev.kobj, rainbow_dev_groups);
	mutex_destroy(&ctrl->work_mutex);
	vd6281_power_down(ctrl);
	kfree(ctrl);

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

static int __init vd6281_init(void)
{
	return i2c_add_driver(&vd6281_driver);
}

static void __exit vd6281_exit(void)
{
	i2c_del_driver(&vd6281_driver);
}

MODULE_DESCRIPTION("ST rainbow sensor");
MODULE_LICENSE("GPL");

module_init(vd6281_init);
module_exit(vd6281_exit);

