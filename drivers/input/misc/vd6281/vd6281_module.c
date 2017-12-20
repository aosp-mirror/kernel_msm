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
#include "cam_sensor_dev.h"

#define VD6281_DEV_NAME	"vd6281"

struct rainbow_ctrl_t {
	struct platform_device *pdev;
	struct cam_hw_soc_info soc_info;
	struct mutex cam_sensor_mutex;
	enum cci_i2c_master_t cci_i2c_master;
	struct camera_io_master io_master_info;
	uint32_t id;
	struct device_node *of_node;
	struct cam_subdev v4l2_dev_str;
	int  is_power_up;
	struct regulator *vdd;
};

int vd6281_write_data(struct rainbow_ctrl_t *ctrl,
					uint32_t addr, uint32_t data)
{
	int rc = 0;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array i2c_reg_array;

	i2c_reg_array.reg_addr = addr;
	i2c_reg_array.reg_data = data;
	i2c_reg_array.delay = 1;
	write_setting.reg_setting = &i2c_reg_array;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&(ctrl->io_master_info), &write_setting);
	return rc;
}

static int vd6281_power_up(struct rainbow_ctrl_t *ctrl)
{
	int rc;

	rc = regulator_enable(ctrl->vdd);
	if (rc < 0)
		pr_err("%s regulator_enable failed: rc: %d", __func__, rc);
	rc = camera_io_init(&(ctrl->io_master_info));
	if (rc < 0)
		pr_err("%s cci_release failed: rc: %d", __func__, rc);

	return rc;
}

static int vd6281_power_down(struct rainbow_ctrl_t *ctrl)
{
	int rc;

	rc = camera_io_release(&(ctrl->io_master_info));
	if (rc < 0)
		pr_err("%s cci_release failed: rc: %d", __func__, rc);
	rc = regulator_disable(ctrl->vdd);
	if (rc < 0)
		pr_err("%s regulator_disable failed: rc: %d", __func__, rc);

	return rc;
}

static ssize_t rainbow_enable_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->is_power_up);
}

static ssize_t rainbow_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc;
	bool value;

	rc = kstrtobool(buf, &value);
	if (rc)
		return rc;

	mutex_lock(&ctrl->cam_sensor_mutex);

	if (value == 1)
		rc = vd6281_power_up(ctrl);
	else
		rc = vd6281_power_down(ctrl);

	if (rc == 0)
		ctrl->is_power_up = (int)value;

	mutex_unlock(&ctrl->cam_sensor_mutex);

	return rc;
}

static ssize_t rainbow_read_byte_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	unsigned long value;
	uint32_t read_data = 0;
	int rc;

	rc = kstrtoul(buf, 0, &value);
	if (rc)
		return rc;

	mutex_lock(&ctrl->cam_sensor_mutex);

	rc = camera_io_dev_read(&(ctrl->io_master_info), value & 0xFF,
		&read_data, CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);

	mutex_unlock(&ctrl->cam_sensor_mutex);

	if (rc < 0) {
		dev_err(dev, "%s I2C read failed: %d.\n", __func__, rc);
		return rc;
	}

	dev_dbg(dev, "I2C read addr: 0x%lx, data: 0x%x\n", value, read_data);

	return read_data;
}

static ssize_t rainbow_write_byte_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	unsigned long value;
	int rc;
	u8 addr;
	u8 data;

	rc = kstrtoul(buf, 0, &value);

	if (rc)
		return rc;

	addr = (value >> 8) & 0xFF;
	data = value & 0xFF;

	mutex_lock(&ctrl->cam_sensor_mutex);

	rc = vd6281_write_data(ctrl, addr, data);

	mutex_unlock(&ctrl->cam_sensor_mutex);

	if (rc < 0)
		dev_err(dev, "%s I2C write failed: %d.", __func__, rc);

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

int32_t vd6281_update_i2c_info(struct rainbow_ctrl_t *ctrl)
{
	int32_t rc = 0;

	ctrl->cci_i2c_master = MASTER_0;
	ctrl->io_master_info.cci_client->sid = 0x40 >> 1;
	ctrl->io_master_info.cci_client->retries = 3;
	ctrl->io_master_info.cci_client->id_map = 0;
	ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_STANDARD_MODE;

	return rc;
}

static int vd6281_parse_dt(struct device *dev)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc = 0;

	/* Initialize mutex */
	mutex_init(&ctrl->cam_sensor_mutex);

	ctrl->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ctrl->vdd)) {
		ctrl->vdd = NULL;
		dev_err(dev, "unable to get vdd\n");
		rc = -ENOENT;
	}

	return rc;
}

static int32_t vd6281_platform_remove(struct platform_device *pdev)
{
	struct rainbow_ctrl_t  *ctrl;
	int32_t rc = 0;

	ctrl = platform_get_drvdata(pdev);
	if (!ctrl) {
		pr_err("rainbow device is NULL");
		return 0;
	}

	mutex_destroy(&ctrl->cam_sensor_mutex);
	sysfs_remove_groups(&pdev->dev.kobj, rainbow_dev_groups);
	kfree(ctrl->io_master_info.cci_client);
	ctrl->io_master_info.cci_client = NULL;
	devm_kfree(&pdev->dev, ctrl);

	return rc;
}

static int32_t vd6281_driver_platform_probe(
	struct platform_device *pdev)
{
	int32_t rc = 0;
	struct rainbow_ctrl_t *ctrl = NULL;

	/* Create sensor control structure */
	ctrl = devm_kzalloc(&pdev->dev,
		sizeof(struct rainbow_ctrl_t), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	if (cam_cci_get_subdev() == NULL) {
		dev_err(&pdev->dev, "wait cci driver probe\n");
		return -EPROBE_DEFER;
	}

	/*fill in platform device*/
	ctrl->v4l2_dev_str.pdev = pdev;
	ctrl->soc_info.pdev = pdev;
	ctrl->soc_info.dev = &pdev->dev;
	ctrl->soc_info.dev_name = pdev->name;
	ctrl->io_master_info.master_type = CCI_MASTER;

	ctrl->io_master_info.cci_client = kzalloc(sizeof(
		struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(ctrl->io_master_info.cci_client))
		return -ENOMEM;

	platform_set_drvdata(pdev, ctrl);
	v4l2_set_subdevdata(&ctrl->v4l2_dev_str.sd, ctrl);
	dev_set_drvdata(&pdev->dev, ctrl);

	rc = vd6281_parse_dt(&(pdev->dev));
	if (rc)
		dev_err(&pdev->dev, "paring rainbow dt failed rc %d", rc);

	vd6281_update_i2c_info(ctrl);

	/* Fill platform device id*/
	pdev->id = ctrl->soc_info.index;

	rc = sysfs_create_groups(&pdev->dev.kobj, rainbow_dev_groups);
	if (rc)
		dev_err(&pdev->dev, "failed to create sysfs files");

	return rc;
}

static const struct of_device_id vd6281_driver_dt_match[] = {
	{.compatible = "qcom,rainbow"},
	{}
};

MODULE_DEVICE_TABLE(of, vd6281_driver_dt_match);

static struct platform_driver vd6281_platform_driver = {
	.probe = vd6281_driver_platform_probe,
	.driver = {
		.name = VD6281_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vd6281_driver_dt_match,
	},
	.remove = vd6281_platform_remove,
};

static int __init vd6281_init(void)
{
	int rc = 0;

	rc = platform_driver_register(&vd6281_platform_driver);

	return rc;
}

static void __exit vd6281_exit(void)
{
	platform_driver_unregister(&vd6281_platform_driver);
}

MODULE_DESCRIPTION("ST rainbow sensor");
MODULE_LICENSE("GPL");

module_init(vd6281_init);
module_exit(vd6281_exit);

