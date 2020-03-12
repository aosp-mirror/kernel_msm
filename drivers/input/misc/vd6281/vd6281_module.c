// SPDX-License-Identifier: GPL-2.0+
/*
 *  vd6281_module.c - Linux kernel modules for rainbow sensor
 *
 *  Copyright (C) 2019 Google, Inc.
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
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <media/cam_sensor.h>


#define VD6281_DEV_NAME	"vd6281"

#define VIO_VOLTAGE_MIN 1800000
#define VIO_VOLTAGE_MAX 1800000

enum RAINBOW_POWER {
	REGULATOR_VIO,
	REGULATOR_VDD,
	POWER_MAX
};

struct rainbow_ctrl_t {
	struct i2c_client *client;
	struct regulator *vdd;
	struct regulator *vio;
	const char *dev_name;
	dev_t dev;
	bool is_power_up[POWER_MAX];
	bool is_probed;
	uint32_t read_addr;
	uint32_t read_data;
	struct cdev c_dev;
	struct class *cl;
	unsigned int reset;
	struct mutex work_mutex;
};

static int vd6281_write_data(
	struct rainbow_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t data)
{
	int rc;

	rc = i2c_smbus_write_byte_data(ctrl->client, addr, data);

	if (rc != 0)
		dev_err(&ctrl->client->dev, "%s failed rc = %d", __func__, rc);
	else
		dev_dbg(&ctrl->client->dev,
			"%s: set data 0x%x to 0x%x rc %d",
			__func__,
			data,
			addr,
			rc);

	return rc;
}

static int vd6281_read_data(
	struct rainbow_ctrl_t *ctrl,
	uint32_t addr)
{
	int value;

	value = i2c_smbus_read_byte_data(ctrl->client, addr);

	if (value < 0)
		dev_err(&ctrl->client->dev, "%s failed addr 0x%x value = %d",
		__func__, addr, value);
	else
		dev_dbg(&ctrl->client->dev,
			"%s: got data 0x%x from 0x%x",
			__func__,
			value,
			addr);

	return value;
}

static int vd6281_parse_dt(struct device *dev)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc = 0;

	ctrl->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(ctrl->vio)) {
		ctrl->vio = NULL;
		dev_err(dev, "Unable to get vio\n");
		rc = -ENOENT;
	}

	ctrl->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ctrl->vdd)) {
		ctrl->vdd = NULL;
		dev_err(dev, "Unable to get vdd\n");
		rc = -ENOENT;
	}

	return rc;
}

static int vd6281_power_up(struct rainbow_ctrl_t *ctrl)
{
	int rc;

	if (!ctrl->is_power_up[REGULATOR_VDD]) {
		rc = regulator_enable(ctrl->vdd);
		if (rc < 0) {
			dev_err(&ctrl->client->dev,
				"%s vdd regulator_enable failed: rc: %d",
				__func__, rc);
			return rc;
		}
		ctrl->is_power_up[REGULATOR_VDD] = true;
	}

	/* At least 1 ms delay after VDD power up */
	usleep_range(1000, 3000);

	if (!ctrl->is_power_up[REGULATOR_VIO]) {
		rc = regulator_set_voltage(ctrl->vio,
			VIO_VOLTAGE_MIN, VIO_VOLTAGE_MAX);
		if (rc < 0) {
			dev_err(&ctrl->client->dev,
				"set vio voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->vio);
		if (rc < 0) {
			dev_err(&ctrl->client->dev,
				"%s vio regulator_enable failed: rc: 0x%x",
				__func__, rc);
			return rc;
		}
		ctrl->is_power_up[REGULATOR_VIO] = true;
	}
	/* At least 1 ms delay after power up */
	usleep_range(1000, 3000);

	return rc;
}

static int vd6281_power_down(struct rainbow_ctrl_t *ctrl)
{
	int rc = 0, is_error;

	if (ctrl->is_power_up[REGULATOR_VIO]) {
		is_error = regulator_disable(ctrl->vio);
		if (is_error < 0) {
			rc = is_error;
			dev_err(&ctrl->client->dev,
				"%s vio regulator_disable failed: rc: %d",
				__func__, rc);
		} else
			ctrl->is_power_up[REGULATOR_VIO] = false;
	}

	/* At least 1 ms delay after VIO power down */
	usleep_range(1000, 3000);

	if (ctrl->is_power_up[REGULATOR_VDD]) {
		is_error = regulator_disable(ctrl->vdd);
		if (is_error < 0) {
			rc = is_error;
			dev_err(&ctrl->client->dev,
				"%s vdd regulator_disable failed: rc: %d",
				__func__, rc);
		} else
			ctrl->is_power_up[REGULATOR_VDD] = false;
	}

	return rc;
}

static ssize_t rainbow_enable_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	int is_enabled;

	mutex_lock(&ctrl->work_mutex);
	is_enabled = (ctrl->is_power_up[REGULATOR_VDD] &&
		ctrl->is_power_up[REGULATOR_VDD]);
	mutex_unlock(&ctrl->work_mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", is_enabled);
}

static ssize_t rainbow_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc = 0;
	bool value = 0;

	rc = kstrtobool(buf, &value);
	if (rc)
		return rc;

	mutex_lock(&ctrl->work_mutex);
	if (value == true) {
		rc = vd6281_power_up(ctrl);
		if (rc != 0) {
			vd6281_power_down(ctrl);
			mutex_unlock(&ctrl->work_mutex);
			return rc;
		}
	} else
		rc = vd6281_power_down(ctrl);
	mutex_unlock(&ctrl->work_mutex);

	return rc < 0 ? rc : count;
}

static ssize_t rainbow_read_byte_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&ctrl->work_mutex);
	rc = scnprintf(buf, PAGE_SIZE, "%x\n", ctrl->read_data);
	mutex_unlock(&ctrl->work_mutex);
	return rc;
}

static ssize_t rainbow_read_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	uint32_t read_data = 0;
	int rc;

	mutex_lock(&ctrl->work_mutex);
	if (!(ctrl->is_power_up[REGULATOR_VDD] &&
		ctrl->is_power_up[REGULATOR_VIO])) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &value);
	if (rc)
		goto error_out;

	value &= 0xFF;

	read_data = vd6281_read_data(ctrl, value);

	if (read_data < 0)
		goto error_out;
	else {
		ctrl->read_addr = value;
		ctrl->read_data = read_data;
	}
	mutex_unlock(&ctrl->work_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->work_mutex);
	return rc;
}

static ssize_t rainbow_write_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	uint32_t addr;
	uint32_t data;
	int rc;

	mutex_lock(&ctrl->work_mutex);
	if (!(ctrl->is_power_up[REGULATOR_VDD] &&
		ctrl->is_power_up[REGULATOR_VIO])) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &value);
	if (rc)
		goto error_out;

	addr = (value >> 8) & 0xFF;
	data = value & 0xFF;

	rc = vd6281_write_data(ctrl, addr, data);
	if (rc < 0)
		goto error_out;
	else {
		if (addr == ctrl->read_addr)
			ctrl->read_data = data;
	}
	mutex_unlock(&ctrl->work_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->work_mutex);
	return rc;
}

static DEVICE_ATTR_RW(rainbow_enable);
static DEVICE_ATTR_RW(rainbow_read_byte);
static DEVICE_ATTR_WO(rainbow_write_byte);

static struct attribute *rainbow_dev_attrs[] = {
	&dev_attr_rainbow_enable.attr,
	&dev_attr_rainbow_read_byte.attr,
	&dev_attr_rainbow_write_byte.attr,
	NULL
};

ATTRIBUTE_GROUPS(rainbow_dev);

static int32_t vd6281_platform_remove(struct i2c_client *client)
{
	struct rainbow_ctrl_t *ctrl;
	int32_t rc = 0;

	ctrl = i2c_get_clientdata(client);
	if (!ctrl) {
		dev_err(&client->dev, "rainbow device is NULL");
		return 0;
	}
	if (!ctrl->is_probed)
		return 0;

	device_destroy(ctrl->cl, ctrl->dev);
	class_destroy(ctrl->cl);
	cdev_del(&ctrl->c_dev);
	unregister_chrdev_region(ctrl->dev, 1);
	sysfs_remove_groups(&client->dev.kobj, rainbow_dev_groups);
	vd6281_power_down(ctrl);
	mutex_destroy(&ctrl->work_mutex);

	return rc;
}
static int vd6281_open(struct inode *inode, struct file *file)
{
	struct rainbow_ctrl_t *ctrl = container_of(inode->i_cdev,
		struct rainbow_ctrl_t, c_dev);
	get_device(&ctrl->client->dev);
	file->private_data = ctrl;
	return 0;
}

static int vd6281_release(struct inode *inode, struct file *file)
{
	struct rainbow_ctrl_t *ctrl = container_of(inode->i_cdev,
		struct rainbow_ctrl_t, c_dev);
	put_device(&ctrl->client->dev);
	vd6281_power_down(ctrl);
	return 0;
}

static long vd6281_ioctl_handler(struct file *file, unsigned int cmd,
	unsigned long arg, void __user *p)
{
	int rc, value = 0;
	int i;
	uint32_t isEnable;
	struct rainbow_ctrl_t *ctrl = file->private_data;
	struct rainbow_config config;
	uint8_t data[MAX_RAINBOW_CONFIG_SIZE];

	mutex_lock(&ctrl->work_mutex);
	switch (cmd) {
	case RAINBOW_CONFIG:
		if (copy_from_user(&config, p, sizeof(config))) {
			rc = -EFAULT;
			goto out;
		}

		if (config.size == 0 ||
			config.size > MAX_RAINBOW_CONFIG_SIZE) {
			rc = -EFAULT;
			goto out;
		}

		if (config.operation == RAINBOW_RANDOM_READ) {
			for (i = 0; i < config.size; i++) {
				value = i2c_smbus_read_byte_data(ctrl->client,
					config.reg_addr[i] & 0xFF);

				if (value < 0) {
					dev_err(&ctrl->client->dev,
					"%s: got data 0x%x from 0x%x",
					__func__,
					value,
					config.reg_addr[i] & 0xFF);
					rc = -EINVAL;
					goto out;
				} else
					config.reg_data[i] = value;
			}
			rc = copy_to_user(p, &config, sizeof(config));
		} else if (config.operation == RAINBOW_SEQ_READ) {
			for (i = 0; i < config.size; i++) {
				value = i2c_smbus_read_byte_data(ctrl->client,
					config.reg_addr[i] & 0xFF);

				if (value < 0) {
					dev_err(&ctrl->client->dev,
					"%s: got data 0x%x from 0x%x",
					__func__,
					value,
					config.reg_addr[i] & 0xFF);
					rc = -EINVAL;
					goto out;
				} else
					data[i] = value;
			}
			for (i = 0; i < config.size; i++)
				config.reg_data[i] = data[i];

			rc = copy_to_user(p, &config, sizeof(config));
		} else if (config.operation == RAINBOW_RANDOM_WRITE) {
			for (i = 0; i < config.size; i++) {
				rc = i2c_smbus_write_byte_data(ctrl->client,
					config.reg_addr[i] & 0xFF,
					config.reg_data[i] & 0xFF);

				if (rc != 0) {
					dev_err(&ctrl->client->dev,
					"%s: set data 0x%x from 0x%x",
					__func__,
					config.reg_data[i] & 0xFF,
					config.reg_addr[i] & 0xFF);
				}
			}
		} else if (config.operation == RAINBOW_ENABLE) {
			isEnable = config.reg_data[0];

			if (isEnable == 1) {
				rc = vd6281_power_up(ctrl);
				if (rc != 0) {
					vd6281_power_down(ctrl);
				}
			} else {
				rc = vd6281_power_down(ctrl);
				if (rc != 0) {
					dev_err(&ctrl->client->dev,
					"%s: Fail to power down\n",
					__func__);
				}
			}
		} else {
			dev_err(&ctrl->client->dev,
				"%s: Unsupported opertion type %d\n",
				__func__, config.operation);
			rc = -EINVAL;
			goto out;
		}
		break;
	default:
		dev_err(&ctrl->client->dev,
			"%s: Unsupported ioctl command %u\n", __func__, cmd);
		rc = -EINVAL;
		break;
	}

out:
	mutex_unlock(&ctrl->work_mutex);
	return rc;
}

static long vd6281_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return vd6281_ioctl_handler(file, cmd, arg, (void __user *)arg);
}

#ifdef CONFIG_COMPAT
static long vd6281_compat_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return vd6281_ioctl_handler(file, cmd, arg, compat_ptr(arg));
}
#endif

static const struct file_operations vd6281_fops = {
	.owner		= THIS_MODULE,
	.open		= vd6281_open,
	.release	= vd6281_release,
	.unlocked_ioctl	= vd6281_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= vd6281_compat_ioctl,
#endif
};

static int32_t vd6281_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int32_t rc, device_id = 0;
	struct device *dev_ret;
	struct rainbow_ctrl_t *ctrl;

	/* Check i2c functionality*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "fail at %s, %d\n", __func__, __LINE__);
		return -EIO;
	}

	/* Create sensor control structure */
	ctrl = devm_kzalloc(&client->dev,
		sizeof(struct rainbow_ctrl_t), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	/* Fill in platform device */
	ctrl->client = client;
	ctrl->is_power_up[REGULATOR_VIO] = false;
	ctrl->is_power_up[REGULATOR_VDD] = false;
	ctrl->is_probed = false;

	/* setup device name */
	ctrl->dev_name = dev_name(&client->dev);

	/* setup device data */
	dev_set_drvdata(&client->dev, ctrl);

	/* setup client data */
	i2c_set_clientdata(client, ctrl); //dev_set_drvdata

	rc = vd6281_parse_dt(&client->dev);
	if (rc) {
		dev_err(&client->dev, "paring rainbow dt failed rc %d", rc);
		return rc;
	}

	mutex_init(&ctrl->work_mutex);

	rc = sysfs_create_groups(&client->dev.kobj, rainbow_dev_groups);
	if (rc) {
		dev_err(&client->dev, "failed to create sysfs files");
		goto error_destroy_mutex;
	}

	rc = alloc_chrdev_region(&ctrl->dev, 0, 1, "vd6281_ioctl");
	if (rc) {
		dev_err(&client->dev, "failed to alloc chrdev region");
		goto error_remove_sysfs;
	}

	cdev_init(&ctrl->c_dev, &vd6281_fops);

	rc = cdev_add(&ctrl->c_dev, ctrl->dev, 1);
	if (rc) {
		dev_err(&client->dev, "failed to add cdev");
		goto error_unregister_chrdev;
	}

	ctrl->cl = class_create(THIS_MODULE, "char");
	rc = IS_ERR(ctrl->cl);
	if (rc)
		goto error_del_cdev;

	dev_ret = device_create(ctrl->cl, NULL, ctrl->dev, NULL, "vd6281");
	rc = IS_ERR(dev_ret);
	if (rc)
		goto error_destroy_class;
	/* Read device id */
	rc = vd6281_power_up(ctrl);
	if (rc != 0) {
		vd6281_power_down(ctrl);
		goto error_destroy_device;
	}

	device_id = vd6281_read_data(ctrl, 0x00);
	if (device_id < 0) {
		vd6281_power_down(ctrl);
		goto error_destroy_device;
	}

	rc = vd6281_power_down(ctrl);
	if (rc != 0)
		goto error_destroy_device;

	_dev_info(&client->dev, "probe success, device id 0x%x rc = %d",
			device_id, rc);
	ctrl->is_probed = true;

	return rc;

error_destroy_device:
	device_destroy(ctrl->cl, ctrl->dev);
error_destroy_class:
	class_destroy(ctrl->cl);
error_del_cdev:
	cdev_del(&ctrl->c_dev);
error_unregister_chrdev:
	unregister_chrdev_region(ctrl->dev, 1);
error_remove_sysfs:
	sysfs_remove_groups(&client->dev.kobj, rainbow_dev_groups);
error_destroy_mutex:
	mutex_destroy(&ctrl->work_mutex);
	return rc;
}



static const struct of_device_id st_vd6281_dt_match[] = {
	{ .compatible = "st,vd6281"},
	{ },
};

MODULE_DEVICE_TABLE(of, st_vd6281_dt_match);

static struct i2c_driver vd6281_platform_driver = {
	.probe = vd6281_probe,
	.driver = {
		.name = VD6281_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = st_vd6281_dt_match,
	},
	.remove = vd6281_platform_remove,
};

static int __init vd6281_init(void)
{
	return i2c_add_driver(&vd6281_platform_driver);
}

static void __exit vd6281_exit(void)
{
	i2c_del_driver(&vd6281_platform_driver);
}

MODULE_DESCRIPTION("ST VD6281 rainbow sensor");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xu Han <xuhanyz@google.com>");
MODULE_AUTHOR("Speth Chang <spethchang@google.com>");
MODULE_AUTHOR("Nick Chung <nickchung@google.com>");

module_init(vd6281_init);
module_exit(vd6281_exit);
