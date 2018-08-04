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

struct led_laser_ctrl_t {
	struct platform_device *pdev;
	struct cam_hw_soc_info soc_info;
	struct mutex cam_sensor_mutex;
	struct camera_io_master io_master_info;
	bool is_power_up;
	bool is_cci_init;
	struct regulator *vio;
	dev_t dev;
	enum LASER_TYPE type;
	uint32_t read_addr;
	uint32_t read_data;
};

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

	if (rc != 0) {
		pr_err("%s failed rc = %d", __func__, rc);
	}

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
	int rc = 0;
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

	if (rc != 0) {
		pr_err("%s failed rc = %d", __func__, rc);
	}

	pr_debug("%s: set data 0x%x to 0x%x rc %d",
		__func__,
		data,
		addr,
		rc);
	return rc;
}

static int lm36011_power_up(struct led_laser_ctrl_t *ctrl)
{
	int rc = 0;

	if (!ctrl->is_power_up) {
		rc = regulator_enable(ctrl->vio);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"regulator_enable failed: rc: %d", rc);
			return rc;
		} else {
			ctrl->is_power_up = true;
		}
	}

	if (!ctrl->is_cci_init) {
		rc = camera_io_init(&(ctrl->io_master_info));
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"cam io init failed: rc: %d", rc);
			return rc;
		} else {
			ctrl->is_cci_init = true;
		}
	}

	return rc;
}

static int lm36011_power_down(struct led_laser_ctrl_t *ctrl)
{
	int rc = 0;

	if (ctrl->is_cci_init) {
		rc = camera_io_release(&(ctrl->io_master_info));
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"cci_release failed: rc: %d", rc);
		} else {
			ctrl->is_cci_init = false;
		}
	}

	if (ctrl->is_power_up) {
		rc = regulator_disable(ctrl->vio);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"regulator_disable failed: rc: %d", rc);
		} else {
			ctrl->is_power_up = false;
		}
	}

	return rc;
}

static int lm36011_parse_dt(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc = 0, value = 0;

	ctrl->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(ctrl->vio)) {
		ctrl->vio = NULL;
		dev_err(dev, "unable to get vio");
		rc = -ENOENT;
	}

	if (of_property_read_u32(dev->of_node, "laser-type",
		&value)) {
		dev_warn(dev, "laser-type not specified in dt");
	} else {
		ctrl->type = value;
	}

	return rc;
}

static int32_t lm36011_update_i2c_info(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int32_t rc = 0, value = 0;

	if (of_property_read_u32(dev->of_node, "cci-master", &value)) {
		dev_err(dev, "cci master not specified in dt");
		return -EINVAL;
	} else {
		ctrl->io_master_info.cci_client->cci_i2c_master = value;
	}

	if (of_property_read_u32(dev->of_node, "reg", &value)) {
		dev_err(dev, "slave address is not specified in dt");
		return -EINVAL;

	} else {
		ctrl->io_master_info.cci_client->sid = value;
	}

	ctrl->io_master_info.cci_client->cci_device = CCI_DEVICE_0;
	ctrl->io_master_info.cci_client->retries = 3;
	ctrl->io_master_info.cci_client->id_map = 0;
	ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_MODE;

	return rc;
}

static ssize_t led_laser_enable_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	bool is_enabled = false;
	int rc = 0;

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
	int rc = 0;
	bool value;

	rc = kstrtobool(buf, &value);
	if (rc != 0) {
		return rc;
	}

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (value == true) {
		rc = lm36011_power_up(ctrl);
		if (rc != 0) {
			mutex_unlock(&ctrl->cam_sensor_mutex);
			return rc;
		}
		rc = lm36011_write_data(ctrl,
			ENABLE_REG, IR_ENABLE_MODE);
	} else {
		rc = lm36011_power_down(ctrl);
	}
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return rc < 0 ? rc : count;
}

static ssize_t led_laser_read_byte_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc = 0;

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

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (!ctrl->is_cci_init || !ctrl->is_power_up) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &addr);
	if (rc) {
		goto error_out;
	}

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
	uint32_t addr = 0;
	uint32_t data = 0;
	int rc = 0;

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (!ctrl->is_cci_init || !ctrl->is_power_up) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &value);

	if (rc) {
		goto error_out;
	}

	addr = (value >> 8) & 0xFF;
	data = value & 0xFF;

	rc = lm36011_write_data(ctrl, addr, data);
	if (rc < 0) {
		dev_err(dev, "%s i2c write failed: %d.", __func__, rc);
		goto error_out;
	} else {
		if (addr == ctrl->read_addr) {
			ctrl->read_data = data;
		}
	}
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static DEVICE_ATTR_RW(led_laser_enable);
static DEVICE_ATTR_RW(led_laser_read_byte);
static DEVICE_ATTR_WO(led_laser_write_byte);

static struct attribute *led_laser_dev_attrs[] = {
	&dev_attr_led_laser_enable.attr,
	&dev_attr_led_laser_read_byte.attr,
	&dev_attr_led_laser_write_byte.attr,
	NULL
};

ATTRIBUTE_GROUPS(led_laser_dev);

static int32_t lm36011_platform_remove(struct platform_device *pdev)
{
	struct led_laser_ctrl_t  *ctrl;
	int32_t rc = 0;

	ctrl = platform_get_drvdata(pdev);
	if (!ctrl) {
		dev_err(&pdev->dev, "led laser device is NULL");
		return 0;
	}

	sysfs_remove_groups(&pdev->dev.kobj, led_laser_dev_groups);
	mutex_destroy(&ctrl->cam_sensor_mutex);
	lm36011_power_down(ctrl);
	kfree(ctrl->io_master_info.cci_client);
	ctrl->io_master_info.cci_client = NULL;
	devm_kfree(&pdev->dev, ctrl);

	return rc;
}

static int32_t lm36011_driver_platform_probe(
	struct platform_device *pdev)
{
	int32_t rc = 0;
	uint32_t device_id = 0;
	struct led_laser_ctrl_t *ctrl;

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

	ctrl->io_master_info.cci_client = kzalloc(sizeof(
		struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(ctrl->io_master_info.cci_client)) {
		dev_err(&pdev->dev, "no memory for cci client");
		rc = -ENOMEM;
		goto error_free_ctrl;
	}

	platform_set_drvdata(pdev, ctrl);
	dev_set_drvdata(&pdev->dev, ctrl);

	rc = lm36011_parse_dt(&(pdev->dev));
	if (rc) {
		dev_err(&pdev->dev, "paring led laser dt failed rc %d", rc);
		goto error_free_all;
	}

	rc = lm36011_update_i2c_info(&(pdev->dev));
	if (rc) {
		dev_err(&pdev->dev, "update i2c info failed rc %d", rc);
		goto error_free_all;
	}

	/* Fill platform device id*/
	pdev->id = ctrl->soc_info.index;

	/* Read device id */
	rc = lm36011_power_up(ctrl);
	if (rc != 0) {
		lm36011_power_down(ctrl);
		goto error_free_all;
	}

	rc = lm36011_read_data(ctrl,
		DEVICE_ID_REG, &device_id);
	if (rc != 0) {
		lm36011_power_down(ctrl);
		goto error_free_all;
	}

	rc = lm36011_power_down(ctrl);
	if (rc != 0) {
		goto error_free_all;
	}

	if (device_id == DEVICE_ID) {
		_dev_info(&pdev->dev, "probe success, device id 0x%x rc = %d",
			device_id, rc);
	} else {
		dev_warn(&pdev->dev, "Device id mismatch, got 0x%x,"
			" expected 0x%x rc = %d", device_id, DEVICE_ID, rc);
	}

	mutex_init(&ctrl->cam_sensor_mutex);

	rc = sysfs_create_groups(&pdev->dev.kobj, led_laser_dev_groups);
	if (rc != 0) {
		dev_err(&pdev->dev, "failed to create sysfs files");
		mutex_destroy(&ctrl->cam_sensor_mutex);
		goto error_free_all;
	}

	return rc;

error_free_all:
	kfree(ctrl->io_master_info.cci_client);
error_free_ctrl:
	devm_kfree(&pdev->dev, ctrl);
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
	int rc = 0;

	rc = platform_driver_register(&lm36011_platform_driver);

	return rc;
}

static void __exit lm36011_exit(void)
{
	platform_driver_unregister(&lm36011_platform_driver);
}

MODULE_DESCRIPTION("Led laser driver");
MODULE_LICENSE("GPL");

module_init(lm36011_init);
module_exit(lm36011_exit);