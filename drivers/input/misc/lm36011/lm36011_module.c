/*
 *  lm36011_module.c - Linux kernel module for led laser
 *
 *  Copyright (C) 2018 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include "cam_sensor_dev.h"
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
#include <linux/cdev.h>
#include <linux/device.h>
#include <media/cam_sensor.h>

#define LM36011_DEV_NAME "lm36011"

#define ENABLE_REG 0x01
#define CONFIGURATION_REG 0x02
#define LED_FLASH_BRIGHTNESS_REG 0x03
#define LED_TORCH_BRIGHTNESS_REG 0x04
#define FLAG_REG 0x05
#define DEVICE_ID_REG 0x06

#define IR_STANDBY_MODE 0x04
#define IR_ENABLE_MODE 0x05
#define DEVICE_ID 0x01

enum LASER_TYPE {
	LASER_FLOOD,
	LASER_DOT,
	LASER_TYPE_MAX
};

struct reg_setting {
	uint32_t addr;
	uint32_t data;
};

struct led_laser_ctrl_t {
	struct platform_device *pdev;
	struct cam_hw_soc_info soc_info;
	struct mutex cam_sensor_mutex;
	struct camera_io_master io_master_info;
	bool is_power_up;
	bool is_cci_init;
	bool is_probed;
	bool is_silego_validated;
	bool is_silego_power_up;
	bool is_sx9320_power_up;
	bool is_certified;
	struct regulator *vio;
	struct regulator *silego_vdd;
	struct regulator *sx9320_vdd;
	enum LASER_TYPE type;
	uint32_t read_addr;
	uint32_t read_data;
	dev_t dev;
	struct cdev c_dev;
	struct class *cl;
};

static const struct reg_setting silego_reg_settings[] = {
	{0xc0, 0x09}, {0xc1, 0x5b}, {0xc2, 0x09}, {0xc3, 0x5b}, {0xcb, 0x93},
	{0xcc, 0x81}, {0xcd, 0x93}, {0xce, 0x83}, {0x92, 0x00}, {0x93, 0x00}
};

static int sx9320_cleanup_nirq(struct led_laser_ctrl_t *ctrl)
{
	int rc, io_release_rc;
	uint32_t old_sid, old_cci_master, data;

	old_sid = ctrl->io_master_info.cci_client->sid;
	old_cci_master = ctrl->io_master_info.cci_client->cci_i2c_master;
	ctrl->io_master_info.cci_client->sid = 0x28;
	ctrl->io_master_info.cci_client->cci_i2c_master = 0;

	rc = camera_io_init(&(ctrl->io_master_info));
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"cam io init for cap sense failed: rc: %d", rc);
		ctrl->io_master_info.cci_client->sid = old_sid;
		ctrl->io_master_info.cci_client->cci_i2c_master =
			old_cci_master;
		return rc;
	}

	rc = camera_io_dev_read(
		&ctrl->io_master_info,
		0x00,
		&data,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
		dev_err(ctrl->soc_info.dev, "clean up NIRQ failed");

	io_release_rc = camera_io_release(&(ctrl->io_master_info));
	if (io_release_rc < 0) {
		dev_err(ctrl->soc_info.dev, "cap sense cci_release failed");
		if (!rc)
			rc = io_release_rc;
	}
	ctrl->io_master_info.cci_client->sid = old_sid;
	ctrl->io_master_info.cci_client->cci_i2c_master = old_cci_master;
	return rc;
}

static int silego_correct_setting(struct led_laser_ctrl_t *ctrl)
{
	int rc;
	uint32_t data;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;

	reg_settings.reg_addr = 0xcd;
	reg_settings.reg_data = 0x93;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&ctrl->io_master_info, &write_setting);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: failed to overwrite setting: rc: %d",
			__func__, rc);
		return rc;
	}

	rc = camera_io_dev_read(
		&ctrl->io_master_info,
		reg_settings.reg_addr,
		&data,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: failed to read back setting: rc: %d",
			__func__, rc);
		return rc;
	}

	if (data != 0x93) {
		dev_err(ctrl->soc_info.dev,
			"%s: failed, expected 0x93, got: 0x%x: rc: %d",
			__func__, rc, data);
		return -EINVAL;
	}
	return rc;
};

static int32_t silego_verify_settings(struct led_laser_ctrl_t *ctrl)
{
	uint32_t data;
	int rc, io_release_rc;
	size_t i, settings_size = ARRAY_SIZE(silego_reg_settings);
	uint32_t old_sid, old_cci_master;

	old_sid = ctrl->io_master_info.cci_client->sid;
	old_cci_master = ctrl->io_master_info.cci_client->cci_i2c_master;
	ctrl->io_master_info.cci_client->sid = 0x08;
	ctrl->io_master_info.cci_client->cci_i2c_master = 0;

	rc = camera_io_init(&(ctrl->io_master_info));
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: cam io init for silego failed: rc: %d",
			__func__, rc);
		ctrl->io_master_info.cci_client->sid = old_sid;
		ctrl->io_master_info.cci_client->cci_i2c_master =
			old_cci_master;
		return rc;
	}

	for (i = 0; i < settings_size; i++) {
		rc = camera_io_dev_read(
			&ctrl->io_master_info,
			silego_reg_settings[i].addr,
			&data,
			CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev, "%s failed on read 0x%x",
				__func__, silego_reg_settings[i].addr);
			goto out;
		}
		/* Some of Silego part isn't store final version value. Need to
		   overwrite to correct value to provide proper functionality.
		*/
		if (silego_reg_settings[i].addr == 0xcd && data == 0xb3) {
			rc = silego_correct_setting(ctrl);
			if (rc < 0)
				goto out;
		} else if (data != silego_reg_settings[i].data) {
			dev_err(ctrl->soc_info.dev,
				"address 0x%x mismatch,"
				" expected 0x%x but got 0x%x",
				silego_reg_settings[i].addr,
				silego_reg_settings[i].data,
				data);
			rc = -EINVAL;
			goto out;
		}
	}
out:
	io_release_rc = camera_io_release(&(ctrl->io_master_info));
	if (io_release_rc < 0) {
		dev_err(ctrl->soc_info.dev, "%s: silego cci_release failed",
			__func__);
		if (!rc)
			rc = io_release_rc;
	}
	ctrl->io_master_info.cci_client->sid = old_sid;
	ctrl->io_master_info.cci_client->cci_i2c_master = old_cci_master;
	return rc;
}

static int lm36011_read_data(
	struct led_laser_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t *data)
{
	int rc = 0;
	rc = camera_io_dev_read(
		&ctrl->io_master_info,
		addr,
		data,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);

	if (rc != 0)
		pr_err("%s failed rc = %d", __func__, rc);
	else
		pr_debug("%s: got data 0x%x from 0x%x rc %d",
			__func__,
			*data,
			addr,
			rc);
	return rc;
}

static int lm36011_write_data(
	struct led_laser_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t data)
{
	int rc;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;
	reg_settings.reg_addr = addr;
	reg_settings.reg_data = data;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&ctrl->io_master_info, &write_setting);

	if (rc != 0)
		pr_err("%s failed rc = %d", __func__, rc);
	else
		pr_debug("%s: set data 0x%x to 0x%x rc %d",
			__func__,
			data,
			addr,
			rc);
	return rc;
}

static int lm36011_power_up(struct led_laser_ctrl_t *ctrl)
{
	int rc;

	if (!ctrl->is_sx9320_power_up) {
		rc = regulator_enable(ctrl->sx9320_vdd);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"cap sense regulator_enable failed: rc: %d",
				rc);
			return rc;
		}
		ctrl->is_sx9320_power_up = true;
	}

	if (!ctrl->is_silego_power_up) {
		rc = regulator_enable(ctrl->silego_vdd);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"silego regulator_enable failed: rc: %d", rc);
			return rc;
		}
		ctrl->is_silego_power_up = true;
	}

	if (!ctrl->is_power_up) {
		rc = regulator_enable(ctrl->vio);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"regulator_enable failed: rc: %d", rc);
			return rc;
		}
		ctrl->is_power_up = true;
	}

	rc = sx9320_cleanup_nirq(ctrl);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"clean up cap sense irq failed: rc: %d", rc);
		ctrl->is_silego_validated = false;
		return rc;
	}

	rc = silego_verify_settings(ctrl);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"verify silego reg setting failed: rc: %d",
			rc);
		ctrl->is_silego_validated = false;
		return rc;
	}
	ctrl->is_silego_validated = true;

	if (!ctrl->is_cci_init) {
		rc = camera_io_init(&(ctrl->io_master_info));
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"cam io init failed: rc: %d", rc);
			return rc;
		}
		ctrl->is_cci_init = true;
	}

	return rc;
}

static int lm36011_power_down(struct led_laser_ctrl_t *ctrl)
{
	int rc = 0, is_error;

	if (ctrl->is_cci_init) {
		is_error = camera_io_release(&(ctrl->io_master_info));
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"laser cci_release failed: rc: %d", rc);
		} else
			ctrl->is_cci_init = false;
	}

	if (ctrl->is_power_up) {
		is_error = regulator_disable(ctrl->vio);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"laser regulator_disable failed: rc: %d", rc);
		} else
			ctrl->is_power_up = false;
	}

	if (ctrl->is_silego_power_up) {
		is_error = regulator_disable(ctrl->silego_vdd);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"silego regulator_disable failed: rc: %d", rc);
		} else
			ctrl->is_silego_power_up = false;
	}

	if (ctrl->is_sx9320_power_up) {
		is_error = regulator_disable(ctrl->sx9320_vdd);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"cap sense regulator_disable failed: rc: %d",
				rc);
		} else
			ctrl->is_sx9320_power_up = false;
	}

	return rc;
}

static int lm36011_parse_dt(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int value = 0;

	ctrl->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(ctrl->vio)) {
		ctrl->vio = NULL;
		dev_err(dev, "unable to get vio");
		return -ENOENT;
	}

	ctrl->silego_vdd = devm_regulator_get(dev, "silego_vdd");
	if (IS_ERR(ctrl->silego_vdd)) {
		ctrl->silego_vdd = NULL;
		dev_err(dev, "unable to get silego vdd");
		return -ENOENT;
	}

	ctrl->sx9320_vdd = devm_regulator_get(dev, "sx9320_vdd");
	if (IS_ERR(ctrl->sx9320_vdd)) {
		ctrl->sx9320_vdd = NULL;
		dev_err(dev, "unable to get cap sense vdd");
		return -ENOENT;
	}

	if (of_property_read_u32(dev->of_node, "laser-type", &value)) {
		dev_err(dev, "laser-type not specified in dt");
		return -ENOENT;
	}
	ctrl->type = value;

	return 0;
}

static int32_t lm36011_update_i2c_info(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int32_t value = 0;

	if (of_property_read_u32(dev->of_node, "cci-master", &value)) {
		dev_err(dev, "cci master not specified in dt");
		return -EINVAL;
	}
	ctrl->io_master_info.cci_client->cci_i2c_master = value;

	if (of_property_read_u32(dev->of_node, "reg", &value)) {
		dev_err(dev, "slave address is not specified in dt");
		return -EINVAL;

	}
	ctrl->io_master_info.cci_client->sid = value;

	if (of_property_read_u32(dev->of_node, "cci-device", &value) ||
		value >= CCI_DEVICE_MAX) {
		dev_err(dev, "cci device is not specified in dt");
		return -EINVAL;

	}
	ctrl->io_master_info.cci_client->cci_device = value;
	ctrl->io_master_info.cci_client->retries = 3;
	ctrl->io_master_info.cci_client->id_map = 0;
	ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_MODE;

	return 0;
}

static ssize_t led_laser_enable_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	bool is_enabled;
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	is_enabled = (ctrl->is_power_up == true && ctrl->is_cci_init == true);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", is_enabled);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t led_laser_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc;
	bool value;

	if (!ctrl->is_certified) {
		dev_err(dev, "Cannot enable laser due to uncertified");
		return -EINVAL;
	}

	rc = kstrtobool(buf, &value);
	if (rc != 0)
		return rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (value == true) {
		rc = lm36011_power_up(ctrl);
		if (rc != 0) {
			rc = lm36011_power_down(ctrl);
			mutex_unlock(&ctrl->cam_sensor_mutex);
			return rc < 0 ? rc : count;
		}
		rc = lm36011_write_data(ctrl,
			ENABLE_REG, IR_ENABLE_MODE);
	} else
		rc = lm36011_power_down(ctrl);
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return rc < 0 ? rc : count;
}

static ssize_t led_laser_read_byte_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	rc = scnprintf(buf, PAGE_SIZE, "%x\n", ctrl->read_data);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t led_laser_read_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t addr = 0;
	uint32_t read_data = 0;
	int rc = 0;

	if (!ctrl->is_certified) {
		dev_err(dev, "Cannot enable laser due to uncertified");
		return -EINVAL;
	}

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (!ctrl->is_cci_init || !ctrl->is_power_up) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &addr);
	if (rc)
		goto error_out;

	addr &= 0xFF;

	rc = lm36011_read_data(ctrl, addr, &read_data);
	if (rc < 0) {
		dev_err(dev, "i2c read failed, rc = %d", rc);
		goto error_out;
	} else {
		ctrl->read_addr = addr;
		ctrl->read_data = read_data;
	}
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t led_laser_write_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	uint32_t addr;
	uint32_t data;
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (!ctrl->is_cci_init || !ctrl->is_power_up) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &value);

	if (rc)
		goto error_out;

	addr = (value >> 8) & 0xFF;
	data = value & 0xFF;

	rc = lm36011_write_data(ctrl, addr, data);
	if (rc < 0) {
		dev_err(dev, "%s i2c write failed: %d.", __func__, rc);
		goto error_out;
	} else {
		if (addr == ctrl->read_addr)
			ctrl->read_data = data;
	}
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t is_silego_validated_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	mutex_lock(&ctrl->cam_sensor_mutex);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->is_silego_validated);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t is_certified_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->is_certified);
}

static DEVICE_ATTR_RW(led_laser_enable);
static DEVICE_ATTR_RW(led_laser_read_byte);
static DEVICE_ATTR_WO(led_laser_write_byte);
static DEVICE_ATTR_RO(is_silego_validated);
static DEVICE_ATTR_RO(is_certified);

static struct attribute *led_laser_dev_attrs[] = {
	&dev_attr_led_laser_enable.attr,
	&dev_attr_led_laser_read_byte.attr,
	&dev_attr_led_laser_write_byte.attr,
	&dev_attr_is_silego_validated.attr,
	&dev_attr_is_certified.attr,
	NULL
};

ATTRIBUTE_GROUPS(led_laser_dev);

static int32_t lm36011_platform_remove(struct platform_device *pdev)
{
	struct led_laser_ctrl_t *ctrl;

	ctrl = platform_get_drvdata(pdev);
	if (!ctrl) {
		dev_err(&pdev->dev, "led laser device is NULL");
		return 0;
	}

	if (!ctrl->is_probed)
		return 0;

	class_destroy(ctrl->cl);
	cdev_del(&ctrl->c_dev);
	unregister_chrdev_region(ctrl->dev, 1);
	sysfs_remove_groups(&pdev->dev.kobj, led_laser_dev_groups);
	mutex_destroy(&ctrl->cam_sensor_mutex);
	lm36011_power_down(ctrl);
	return 0;
}

static int lm36011_open(struct inode *inode, struct file *file)
{
	struct led_laser_ctrl_t *ctrl = container_of(inode->i_cdev,
		struct led_laser_ctrl_t, c_dev);
	get_device(ctrl->soc_info.dev);
	file->private_data = ctrl;
	return 0;
}

static int lm36011_release(struct inode *inode, struct file *file)
{
	struct led_laser_ctrl_t *ctrl = container_of(inode->i_cdev,
		struct led_laser_ctrl_t, c_dev);
	put_device(ctrl->soc_info.dev);
	return 0;
}

static long lm36011_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	int rc = 0;
	struct led_laser_ctrl_t *ctrl = file->private_data;

	switch (cmd) {
	case LM36011_SET_CERTIFICATION_STATUS:
		ctrl->is_certified = (arg == 1 ? true : false);
		break;
	default:
		dev_err(ctrl->soc_info.dev,
			"%s: Unsupported ioctl command %u", __func__, cmd);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static const struct file_operations lm36011_fops = {
	.owner		= THIS_MODULE,
	.open		= lm36011_open,
	.release	= lm36011_release,
	.unlocked_ioctl	= lm36011_ioctl,
};

static int32_t lm36011_driver_platform_probe(
	struct platform_device *pdev)
{
	int32_t rc;
	uint32_t device_id = 0;
	struct device *dev_ret;
	struct led_laser_ctrl_t *ctrl;
	char *class_name, *device_name;

	if (cam_cci_get_subdev(CCI_DEVICE_0) == NULL ||
		cam_cci_get_subdev(CCI_DEVICE_1) == NULL) {
		dev_warn(&pdev->dev, "wait for cci driver probe");
		return -EPROBE_DEFER;
	}

	/* Create sensor control structure */
	ctrl = devm_kzalloc(&pdev->dev,
		sizeof(struct led_laser_ctrl_t), GFP_KERNEL);
	if (!ctrl) {
		dev_err(&pdev->dev, "no memory for driver ctrl");
		return -ENOMEM;
	}

	/*fill in platform device*/
	ctrl->soc_info.pdev = pdev;
	ctrl->soc_info.dev = &pdev->dev;
	ctrl->soc_info.dev_name = pdev->name;
	ctrl->io_master_info.master_type = CCI_MASTER;
	ctrl->is_power_up = false;
	ctrl->is_cci_init = false;
	ctrl->is_probed = false;
	ctrl->is_silego_power_up = false;
	ctrl->is_silego_validated = false;

	ctrl->io_master_info.cci_client = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(ctrl->io_master_info.cci_client)) {
		dev_err(&pdev->dev, "no memory for cci client");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ctrl);
	dev_set_drvdata(&pdev->dev, ctrl);

	rc = lm36011_parse_dt(&(pdev->dev));
	if (rc) {
		dev_err(&pdev->dev, "paring led laser dt failed rc %d", rc);
		return rc;
	}

	rc = lm36011_update_i2c_info(&(pdev->dev));
	if (rc) {
		dev_err(&pdev->dev, "update i2c info failed rc %d", rc);
		return rc;
	}

	/* Fill platform device id*/
	pdev->id = ctrl->soc_info.index;

	mutex_init(&ctrl->cam_sensor_mutex);

	rc = sysfs_create_groups(&pdev->dev.kobj, led_laser_dev_groups);
	if (rc != 0) {
		dev_err(&pdev->dev, "failed to create sysfs files");
		goto error_destroy_mutex;
	}

	rc = alloc_chrdev_region(&ctrl->dev, 0, 1, "lm36011_ioctl");
	if (rc)
		goto error_remove_sysfs;

	cdev_init(&ctrl->c_dev, &lm36011_fops);

	rc = cdev_add(&ctrl->c_dev, ctrl->dev, 1);
	if (rc)
		goto error_unregister_chrdev;

	class_name = (ctrl->type == LASER_FLOOD ? "char_flood" : "char_dot");
	ctrl->cl = class_create(THIS_MODULE, class_name);
	rc = IS_ERR(ctrl->cl);
	if (rc)
		goto error_del_cdev;

	device_name =
		(ctrl->type == LASER_FLOOD ? "lm36011_flood" : "lm36011_dot");
	dev_ret = device_create(ctrl->cl, NULL, ctrl->dev, NULL, device_name);
	rc = IS_ERR(dev_ret);
	if (rc)
		goto error_destroy_class;

	/* Read device id */
	rc = lm36011_power_up(ctrl);
	if (rc != 0) {
		lm36011_power_down(ctrl);
		goto error_destroy_device;
	}

	rc = lm36011_read_data(ctrl,
		DEVICE_ID_REG, &device_id);
	if (rc != 0) {
		lm36011_power_down(ctrl);
		goto error_destroy_device;
	}

	rc = lm36011_power_down(ctrl);
	if (rc != 0)
		goto error_destroy_device;

	if (device_id == DEVICE_ID)
		_dev_info(&pdev->dev, "probe success, device id 0x%x rc = %d",
			device_id, rc);
	else
		dev_warn(&pdev->dev, "Device id mismatch, got 0x%x,"
			" expected 0x%x rc = %d", device_id, DEVICE_ID, rc);

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
	sysfs_remove_groups(&pdev->dev.kobj, led_laser_dev_groups);
error_destroy_mutex:
	mutex_destroy(&ctrl->cam_sensor_mutex);
	return rc;
}


static const struct of_device_id lm36011_driver_dt_match[] = {
	{.compatible = "qcom,cam-led-laser"},
	{}
};

MODULE_DEVICE_TABLE(of, lm36011_driver_dt_match);

static struct platform_driver lm36011_platform_driver = {
	.probe = lm36011_driver_platform_probe,
	.driver = {
		.name = LM36011_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm36011_driver_dt_match,
	},
	.remove = lm36011_platform_remove,
};

static int __init lm36011_init(void)
{
	return platform_driver_register(&lm36011_platform_driver);
}

static void __exit lm36011_exit(void)
{
	platform_driver_unregister(&lm36011_platform_driver);
}

MODULE_DESCRIPTION("Led laser driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Speth Chang <spethchang@google.com>");

module_init(lm36011_init);
module_exit(lm36011_exit);
