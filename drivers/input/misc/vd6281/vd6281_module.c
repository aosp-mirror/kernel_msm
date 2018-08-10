/*
 *  vd6281_module.c - Linux kernel module for rainbow sensor
 *
 *  Copyright (C) 2017-2018 Google, Inc.
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
#include <linux/cdev.h>
#include <linux/device.h>
#include <media/cam_sensor.h>
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
	int hw_version;
	struct regulator *vdd;
	dev_t dev;
	struct cdev c_dev;
	struct class *cl;
};

static struct rainbow_ctrl_t *ctrl;
static const struct file_operations vd6281_fops;

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

	if (ctrl->hw_version == 20)
		ctrl->io_master_info.cci_client->sid = 0x7F;
	else
		ctrl->io_master_info.cci_client->sid = 0x20;

	ctrl->io_master_info.cci_client->retries = 1;
	ctrl->io_master_info.cci_client->id_map = 0;
	ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;

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

	if (of_property_read_u32(dev->of_node, "hw-version",
		&(ctrl->hw_version)))
		dev_warn(dev, "hw-version not specified in dt\n");

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

	device_destroy(ctrl->cl, ctrl->dev);
	class_destroy(ctrl->cl);
	cdev_del(&ctrl->c_dev);
	unregister_chrdev_region(ctrl->dev, 1);

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
	struct device *dev_ret;

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

	if (rc)
		return -EINVAL;

	rc = alloc_chrdev_region(&ctrl->dev, 0, 1, "vd6281_ioctl");

	if (rc)
		return rc;

	cdev_init(&ctrl->c_dev, &vd6281_fops);

	rc = cdev_add(&ctrl->c_dev, ctrl->dev, 1);
	if (rc)
		return rc;

	rc = IS_ERR(ctrl->cl = class_create(THIS_MODULE, "char"));
	if (rc) {
		cdev_del(&ctrl->c_dev);
		unregister_chrdev_region(ctrl->dev, 1);
		return PTR_ERR(ctrl->cl);
	}

	rc = IS_ERR(dev_ret =
			device_create(ctrl->cl, NULL,
			ctrl->dev, NULL, "vd6281"));
	if (rc) {
		class_destroy(ctrl->cl);
		cdev_del(&ctrl->c_dev);
		unregister_chrdev_region(ctrl->dev, 1);
		return PTR_ERR(dev_ret);
	}

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

static int vd6281_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int vd6281_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long vd6281_ioctl_handler(struct file *file, unsigned int cmd,
				 unsigned long arg, void __user *p)
{
	int rc = 0;
	int i = 0;
	struct rainbow_config config;

	mutex_lock(&ctrl->cam_sensor_mutex);

	switch (cmd) {
	case RAINBOW_CONFIG:
		if (copy_from_user(&config, p, sizeof(config))) {
			rc = -EFAULT;
			goto out;
		}

		if (config.size == 0 || config.size > MAX_RAINBOW_CONFIG_SIZE) {
			rc = -EFAULT;
			goto out;
		}

		if (config.operation == RAINBOW_RANDOM_READ) {
			for (i = 0; i < config.size; i++) {
				rc = camera_io_dev_read(&(ctrl->io_master_info),
					config.reg_addr[i],
					&config.reg_data[i],
					CAMERA_SENSOR_I2C_TYPE_BYTE,
					CAMERA_SENSOR_I2C_TYPE_BYTE);
				if (rc) {
					pr_err("%s i2c random read failed: %d\n",
						__func__, rc);
					goto out;
				}
			}
			rc = copy_to_user(p, &config, sizeof(config));
		} else if (config.operation == RAINBOW_SEQ_READ) {
			uint8_t data[MAX_RAINBOW_CONFIG_SIZE];

			rc = camera_io_dev_read_seq(&(ctrl->io_master_info),
					config.reg_addr[0],
					data,
					CAMERA_SENSOR_I2C_TYPE_BYTE,
					config.size);
			if (rc) {
				pr_err("%s i2c cont read failed: %d\n",
					__func__, rc);
				goto out;
			}
			for (i = 0; i < config.size; i++)
				config.reg_data[i] = data[i];

			rc = copy_to_user(p, &config, sizeof(config));
		} else if (config.operation == RAINBOW_RANDOM_WRITE ||
				   config.operation == RAINBOW_SEQ_WRITE) {
			struct cam_sensor_i2c_reg_setting write_setting;
			struct cam_sensor_i2c_reg_array
				reg_setting[MAX_RAINBOW_CONFIG_SIZE] = { {0} };

			write_setting.reg_setting = reg_setting;
			write_setting.size = config.size;
			write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
			write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
			write_setting.delay = 0;

			for (i = 0; i < config.size; i++) {
				reg_setting[i].reg_addr = config.reg_addr[0];
				reg_setting[i].reg_data = config.reg_data[0];
				reg_setting[i].delay = 0;
				reg_setting[i].data_mask = 0;
			}

			if (config.operation == RAINBOW_RANDOM_WRITE)
				rc = camera_io_dev_write(
					&(ctrl->io_master_info),
					&write_setting);
			else
				rc = camera_io_dev_write_continuous(
					&(ctrl->io_master_info),
					&write_setting, 0);

		} else
			pr_err("%s: Unsupported opertion type\n", __func__);

		break;
	default:
		pr_err("%s: Unsupported ioctl command %u\n", __func__, cmd);
		rc = -EINVAL;
		break;
	}

out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
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

