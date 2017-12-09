/*
 * cs40l20.c -- CS40L20 Haptics Driver
 *
 * Copyright 2017 Cirrus Logic, Inc.
 *
 * Author: Jeff LaBundy <jeff.labundy@cirrus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/leds.h>
#include <linux/sysfs.h>
#include <linux/firmware.h>

#include "cs40l20.h"

struct cs40l20_private {
	struct led_classdev led_dev;
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[2];
	unsigned int num_supplies;
	struct work_struct vibe_start_work;
	struct work_struct vibe_stop_work;
	struct workqueue_struct *vibe_workqueue;
	struct mutex lock; /* protect hw register access */
	unsigned int cp_trigger_index;
	unsigned int num_waves;
	bool vibe_init_success;
};

static const char *const cs40l20_supplies[] = {
	"VA",
	"VP",
};

static int cs40l20_index_get(struct cs40l20_private *cs40l20,
			     unsigned int index_reg)
{
	int ret;
	unsigned int val;

	mutex_lock(&cs40l20->lock);

	switch (index_reg) {
	case CS40L20_VIBEGEN_GPIO1_RINDEX:
	case CS40L20_VIBEGEN_GPIO1_FINDEX:
		ret = regmap_read(cs40l20->regmap, index_reg, &val);
		if (!ret)
			ret = val;
		break;
	default:
		ret = cs40l20->cp_trigger_index;
	}

	mutex_unlock(&cs40l20->lock);

	return ret;
}

static int cs40l20_index_set(struct cs40l20_private *cs40l20,
			     unsigned int index_reg, unsigned int index)
{
	int ret;

	mutex_lock(&cs40l20->lock);

	if (index > (cs40l20->num_waves - 1)) {
		ret = -EACCES;
	} else {
		switch (index_reg) {
		case CS40L20_VIBEGEN_GPIO1_RINDEX:
		case CS40L20_VIBEGEN_GPIO1_FINDEX:
			ret = regmap_write(cs40l20->regmap, index_reg, index);
			break;
		default:
			cs40l20->cp_trigger_index = index;
			ret = 0;
		}
	}

	mutex_unlock(&cs40l20->lock);

	return ret;
}

static ssize_t cs40l20_cp_trigger_index_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct cs40l20_private *cs40l20 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", cs40l20_index_get(cs40l20, 0));
}

static ssize_t cs40l20_cp_trigger_index_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = dev_get_drvdata(dev);
	int ret, index;

	ret = kstrtoint(buf, 10, &index);
	if (ret) {
		pr_err("Invalid input: %d\n", ret);
		goto err;
	}

	ret = cs40l20_index_set(cs40l20, 0, index);
	if (ret) {
		pr_err("Failed to set index to %d: %d\n", index, ret);
		goto err;
	}

	return count;
err:
	return ret;
}

static ssize_t cs40l20_gpio1_rise_index_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct cs40l20_private *cs40l20 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			cs40l20_index_get(cs40l20,
					  CS40L20_VIBEGEN_GPIO1_RINDEX));
}

static ssize_t cs40l20_gpio1_rise_index_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = dev_get_drvdata(dev);
	int ret, index;

	ret = kstrtoint(buf, 10, &index);
	if (ret) {
		pr_err("Invalid input: %d\n", ret);
		goto err;
	}

	ret = cs40l20_index_set(cs40l20, CS40L20_VIBEGEN_GPIO1_RINDEX, index);
	if (ret) {
		pr_err("Failed to set index to %d: %d\n", index, ret);
		goto err;
	}

	return count;
err:
	return ret;
}

static ssize_t cs40l20_gpio1_fall_index_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct cs40l20_private *cs40l20 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			cs40l20_index_get(cs40l20,
					  CS40L20_VIBEGEN_GPIO1_FINDEX));
}

static ssize_t cs40l20_gpio1_fall_index_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = dev_get_drvdata(dev);
	int ret, index;

	ret = kstrtoint(buf, 10, &index);
	if (ret) {
		pr_err("Invalid input: %d\n", ret);
		goto err;
	}

	ret = cs40l20_index_set(cs40l20, CS40L20_VIBEGEN_GPIO1_FINDEX, index);
	if (ret) {
		pr_err("Failed to set index to %d: %d\n", index, ret);
		goto err;
	}

	return count;
err:
	return ret;
}

static DEVICE_ATTR(cp_trigger_index, 0660, cs40l20_cp_trigger_index_show,
		   cs40l20_cp_trigger_index_store);
static DEVICE_ATTR(gpio1_rise_index, 0660, cs40l20_gpio1_rise_index_show,
		   cs40l20_gpio1_rise_index_store);
static DEVICE_ATTR(gpio1_fall_index, 0660, cs40l20_gpio1_fall_index_show,
		   cs40l20_gpio1_fall_index_store);

static struct attribute *cs40l20_dev_attrs[] = {
	&dev_attr_cp_trigger_index.attr,
	&dev_attr_gpio1_rise_index.attr,
	&dev_attr_gpio1_fall_index.attr,
	NULL,
};

static struct attribute_group cs40l20_dev_attr_group = {
	.attrs = cs40l20_dev_attrs,
};

static void cs40l20_vibe_start_worker(struct work_struct *work)
{
	struct cs40l20_private *cs40l20 =
	    container_of(work, struct cs40l20_private, vibe_start_work);
	int ret;
	unsigned int control_reg;

	mutex_lock(&cs40l20->lock);

	control_reg = (cs40l20->cp_trigger_index == 0) ?
	    CS40L20_VIBEGEN_TRIG_MS : CS40L20_VIBEGEN_TRIG_INDEX;

	ret = regmap_write(cs40l20->regmap,
			   control_reg, cs40l20->cp_trigger_index);
	if (ret)
		dev_err(cs40l20->dev, "Failed to start playback\n");
	else
		cs40l20->led_dev.brightness = LED_FULL;

	mutex_unlock(&cs40l20->lock);
}

static void cs40l20_vibe_stop_worker(struct work_struct *work)
{
	struct cs40l20_private *cs40l20 =
	    container_of(work, struct cs40l20_private, vibe_stop_work);
	int ret;

	mutex_lock(&cs40l20->lock);

	ret = regmap_write(cs40l20->regmap, CS40L20_VIBEGEN_STOP, 1);
	if (ret)
		dev_err(cs40l20->dev, "Failed to stop playback\n");
	else
		cs40l20->led_dev.brightness = LED_OFF;

	mutex_unlock(&cs40l20->lock);
}

static void cs40l20_vibe_state_change(struct led_classdev *led_cdev,
				      enum led_brightness brightness)
{
	struct cs40l20_private *cs40l20 =
	    container_of(led_cdev, struct cs40l20_private, led_dev);

	switch (brightness) {
	case LED_OFF:
		queue_work(cs40l20->vibe_workqueue, &cs40l20->vibe_stop_work);
		break;
	default:
		queue_work(cs40l20->vibe_workqueue, &cs40l20->vibe_start_work);
	}
}

static void cs40l20_vibe_init(struct cs40l20_private *cs40l20)
{
	int ret;
	struct led_classdev *led_dev = &cs40l20->led_dev;
	struct device *dev = cs40l20->dev;

	led_dev->name = "vibrator";
	led_dev->max_brightness = LED_FULL;
	led_dev->brightness_set = cs40l20_vibe_state_change;

	ret = led_classdev_register(dev, led_dev);
	if (ret) {
		dev_err(dev, "Failed to register LED device: %d\n", ret);
		return;
	}

	mutex_init(&cs40l20->lock);

	cs40l20->vibe_workqueue =
	    alloc_ordered_workqueue("vibe_workqueue", WQ_HIGHPRI);
	if (!cs40l20->vibe_workqueue) {
		dev_err(dev, "Failed to allocate workqueue\n");
		ret = -ENOMEM;
		return;
	}

	INIT_WORK(&cs40l20->vibe_start_work, cs40l20_vibe_start_worker);
	INIT_WORK(&cs40l20->vibe_stop_work, cs40l20_vibe_stop_worker);

	ret = sysfs_create_group(&cs40l20->dev->kobj, &cs40l20_dev_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group: %d\n", ret);
		return;
	}

	cs40l20->vibe_init_success = true;
}

static void cs40l20_dsp_start(struct cs40l20_private *cs40l20)
{
	int ret;
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;
	unsigned int val;

	ret = regmap_update_bits(regmap, CS40L20_DSP1_CCM_CORE_CTRL,
				 CS40L20_DSP1_EN_MASK,
				 1 << CS40L20_DSP1_EN_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to enable DSP\n");
		return;
	}

	ret = regmap_read(regmap, CS40L20_VIBEGEN_FW_STATE, &val);
	if (ret) {
		dev_err(dev, "Failed to read haptics algorithm status\n");
		return;
	}

	if (val == CS40L20_VIBEGEN_FW_RUN) {
		dev_info(dev, "Haptics algorithm started\n");
	} else {
		dev_err(dev, "Failed to start haptics algorithm\n");
		return;
	}

	ret = regmap_write(regmap, CS40L20_VIBEGEN_TIME_MS,
			   CS40L20_VIBEGEN_TIME_MS_MAX);
	if (ret) {
		dev_err(dev, "Failed to extend playback timeout\n");
		return;
	}

	ret = regmap_read(regmap, CS40L20_VIBEGEN_NUM_WAVES, &val);
	if (ret) {
		dev_err(dev, "Failed to read number of waveforms\n");
		return;
	}

	if (val) {
		dev_info(dev, "Found %d waveforms\n", val);
		cs40l20->num_waves = val;
	} else {
		dev_err(dev, "Failed to find waveforms\n");
		return;
	}

	cs40l20_vibe_init(cs40l20);
}

static void cs40l20_waveform_load(const struct firmware *fw, void *context)
{
	struct cs40l20_private *cs40l20 = context;
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;
	int ret;
	unsigned int pos = CS40L20_WT_FILE_HEADER_SIZE;
	unsigned int block_type, block_length;

	if (!fw) {
		dev_err(dev, "Failed to request waveform file\n");
		return;
	}

	if (memcmp(fw->data, "WMDR", 4)) {
		dev_err(dev, "Failed to recognize waveform file\n");
		goto err_rls_fw;
	}

	while (pos < fw->size) {
		/* block offset is not used here */
		pos += CS40L20_WT_DBLK_OFFSET_SIZE;

		block_type = fw->data[pos]
		    + (fw->data[pos + 1] << 8);
		pos += (CS40L20_WT_DBLK_TYPE_SIZE
			+ CS40L20_WT_DBLK_UNUSED_SIZE);

		block_length = fw->data[pos]
		    + (fw->data[pos + 1] << 8)
		    + (fw->data[pos + 2] << 16)
		    + (fw->data[pos + 3] << 24);
		pos += CS40L20_WT_DBLK_LENGTH_SIZE;

		if (block_type == CS40L20_XM_UNPACKED_TYPE) {
			ret = regmap_raw_write_async(regmap,
						     CS40L20_VIBEGEN_WAVE_TABLE,
						     &fw->data[pos],
						     block_length);
			if (ret) {
				dev_err(dev,
					"Failed to write XM_UNPACKED memory\n");
				goto err_rls_fw;
			}
		}

		pos += block_length;
	}

	ret = regmap_async_complete(regmap);
	if (ret) {
		dev_err(dev, "Failed to complete async write\n");
		goto err_rls_fw;
	}

	ret = regmap_async_complete(regmap);
	if (ret) {
		dev_err(dev, "Failed to complete async write\n");
		goto err_rls_fw;
	}

	cs40l20_dsp_start(cs40l20);
err_rls_fw:
	release_firmware(fw);
}

static void cs40l20_firmware_load(const struct firmware *fw, void *context)
{
	struct cs40l20_private *cs40l20 = context;
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;
	int ret;
	unsigned int pos = CS40L20_FW_FILE_HEADER_SIZE;
	unsigned int block_offset, block_length;
	char block_type;

	if (!fw) {
		dev_err(dev, "Failed to request firmware file\n");
		return;
	}

	if (memcmp(fw->data, "WMFW", 4)) {
		dev_err(dev, "Failed to recognize firmware file\n");
		goto err_rls_fw;
	}

	while (pos < fw->size) {
		block_offset = fw->data[pos]
		    + (fw->data[pos + 1] << 8)
		    + (fw->data[pos + 2] << 16);
		pos += CS40L20_FW_DBLK_OFFSET_SIZE;

		block_type = fw->data[pos];
		pos += CS40L20_FW_DBLK_TYPE_SIZE;

		block_length = fw->data[pos]
		    + (fw->data[pos + 1] << 8)
		    + (fw->data[pos + 2] << 16)
		    + (fw->data[pos + 3] << 24);
		pos += CS40L20_FW_DBLK_LENGTH_SIZE;

		switch (block_type) {
		case CS40L20_PM_PACKED_TYPE:
			ret = regmap_raw_write_async(regmap,
						     CS40L20_DSP1_PMEM_0
						     + block_offset * 5,
						     &fw->data[pos],
						     block_length);
			if (ret) {
				dev_err(dev,
					"Failed to write PM_PACKED memory\n");
				goto err_rls_fw;
			}
			break;
		case CS40L20_XM_PACKED_TYPE:
			ret = regmap_raw_write_async(regmap,
						     CS40L20_DSP1_XMEM_PACK_0
						     + block_offset * 3,
						     &fw->data[pos],
						     block_length);
			if (ret) {
				dev_err(dev,
					"Failed to write XM_PACKED memory\n");
				goto err_rls_fw;
			}
			break;
		case CS40L20_YM_PACKED_TYPE:
			ret = regmap_raw_write_async(regmap,
						     CS40L20_DSP1_YMEM_PACK_0
						     + block_offset * 3,
						     &fw->data[pos],
						     block_length);
			if (ret) {
				dev_err(dev,
					"Failed to write YM_PACKED memory\n");
				goto err_rls_fw;
			}
			break;
		}

		pos += block_length;
	}

	ret = regmap_async_complete(regmap);
	if (ret) {
		dev_err(dev, "Failed to complete async write\n");
		goto err_rls_fw;
	}

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "cs40l20.bin",
				dev, GFP_KERNEL, cs40l20,
				cs40l20_waveform_load);
err_rls_fw:
	release_firmware(fw);
}

static int cs40l20_dsp_boot(struct cs40l20_private *cs40l20)
{
	int ret;
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;
	unsigned int i;

	ret = regmap_update_bits(regmap, CS40L20_DSP1_CCM_CORE_CTRL,
				 CS40L20_DSP1_RESET_MASK,
				 1 << CS40L20_DSP1_RESET_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to reset DSP\n");
		goto err;
	}

	ret = regmap_write(regmap, CS40L20_MPU_UNLOCK_ADDR,
			   CS40L20_MPU_UNLOCK_CODE1);
	if (ret) {
		dev_err(dev, "Failed to unlock DSP MPU (step 1 of 2)\n");
		goto err;
	}

	ret = regmap_write(regmap, CS40L20_MPU_UNLOCK_ADDR,
			   CS40L20_MPU_UNLOCK_CODE2);
	if (ret) {
		dev_err(dev, "Failed to unlock DSP MPU (step 2 of 2)\n");
		goto err;
	}

	for (i = CS40L20_MPU_START; i <= CS40L20_MPU_STOP; i += 4) {
		ret = regmap_write(regmap, i, 0xFFFFFFFF);

		if (ret) {
			dev_err(dev, "Failed to free DSP memory\n");
			goto err;
		}
	}

	ret = regmap_write(regmap, CS40L20_MPU_UNLOCK_ADDR, 0);
	if (ret) {
		dev_err(dev, "Failed to lock DSP MPU\n");
		goto err;
	}

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "cs40l20.wmfw",
				dev, GFP_KERNEL, cs40l20,
				cs40l20_firmware_load);
err:
	return ret;
}

static int cs40l20_init(struct cs40l20_private *cs40l20)
{
	int ret;
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;

	cs40l20->cp_trigger_index = 0;
	cs40l20->vibe_init_success = false;

	ret = regmap_update_bits(regmap, CS40L20_DAC_PCM1_SRC,
				 CS40L20_DAC_PCM1_SRC_MASK,
				 CS40L20_DAC_PCM1_SRC_DSP1TX1
				 << CS40L20_DAC_PCM1_SRC_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to route DSP to amplifier\n");
		goto err;
	}

	ret = regmap_update_bits(regmap, CS40L20_PWR_CTRL1,
				 CS40L20_GLOBAL_EN_MASK,
				 1 << CS40L20_GLOBAL_EN_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to enable device\n");
		goto err;
	}

	ret = cs40l20_dsp_boot(cs40l20);
	if (ret) {
		dev_err(dev, "Failed to boot DSP\n");
		goto err;
	}
err:
	return ret;
}

static struct regmap_config cs40l20_regmap = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = CS40L20_LASTREG,
	.reg_defaults = cs40l20_reg,
	.num_reg_defaults = ARRAY_SIZE(cs40l20_reg),
	.volatile_reg = cs40l20_volatile_reg,
	.readable_reg = cs40l20_readable_reg,
	.cache_type = REGCACHE_RBTREE,
};

static int cs40l20_i2c_probe(struct i2c_client *i2c_client,
			     const struct i2c_device_id *id)
{
	struct cs40l20_private *cs40l20;
	struct device *dev = &i2c_client->dev;
	int ret;
	unsigned int reg_devid, reg_revid, reg_otpid, i;

	cs40l20 = devm_kzalloc(dev, sizeof(struct cs40l20_private), GFP_KERNEL);
	if (!cs40l20)
		return -ENOMEM;

	cs40l20->dev = dev;
	dev_set_drvdata(dev, cs40l20);
	i2c_set_clientdata(i2c_client, cs40l20);

	cs40l20->regmap = devm_regmap_init_i2c(i2c_client, &cs40l20_regmap);
	if (IS_ERR(cs40l20->regmap)) {
		ret = PTR_ERR(cs40l20->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(cs40l20_supplies); i++)
		cs40l20->supplies[i].supply = cs40l20_supplies[i];

	cs40l20->num_supplies = ARRAY_SIZE(cs40l20_supplies);

	ret = devm_regulator_bulk_get(dev, cs40l20->num_supplies,
				      cs40l20->supplies);
	if (ret) {
		dev_err(dev, "Failed to request core supplies: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(cs40l20->num_supplies, cs40l20->supplies);
	if (ret) {
		dev_err(dev, "Failed to enable core supplies: %d\n", ret);
		return ret;
	}

	ret = regmap_read(cs40l20->regmap, CS40L20_DEVID, &reg_devid);
	if (ret) {
		dev_err(dev, "Failed to read device ID\n");
		goto err;
	}

	if (reg_devid != CS40L20_CHIP_ID) {
		dev_err(dev, "Failed to recognize device ID: %X\n", reg_devid);
		ret = -ENODEV;
		goto err;
	}

	ret = regmap_read(cs40l20->regmap, CS40L20_REVID, &reg_revid);
	if (ret) {
		dev_err(dev, "Failed to read revision ID\n");
		goto err;
	}

	ret = regmap_read(cs40l20->regmap, CS40L20_OTPID, &reg_otpid);
	if (ret) {
		dev_err(dev, "Failed to read OTP ID\n");
		goto err;
	}

	dev_info(dev, "Cirrus Logic CS40L20 revision %02X\n", reg_revid >> 8);

	ret = cs40l20_init(cs40l20);
	if (ret) {
		dev_err(dev, "Failed to initialize device: %d\n", ret);
		goto err;
	}

	return ret;
err:
	regulator_bulk_disable(cs40l20->num_supplies, cs40l20->supplies);
	return ret;
}

static int cs40l20_i2c_remove(struct i2c_client *i2c_client)
{
	struct cs40l20_private *cs40l20 = i2c_get_clientdata(i2c_client);
	int ret;

	ret = regmap_update_bits(cs40l20->regmap, CS40L20_DSP1_CCM_CORE_CTRL,
				 CS40L20_DSP1_EN_MASK, 0);
	if (ret)
		dev_err(cs40l20->dev, "Failed to disable DSP: %d\n", ret);

	regulator_bulk_disable(cs40l20->num_supplies, cs40l20->supplies);

	if (cs40l20->vibe_init_success) {
		led_classdev_unregister(&cs40l20->led_dev);

		cancel_work_sync(&cs40l20->vibe_start_work);
		cancel_work_sync(&cs40l20->vibe_stop_work);

		destroy_workqueue(cs40l20->vibe_workqueue);

		mutex_destroy(&cs40l20->lock);

		sysfs_remove_group(&cs40l20->dev->kobj,
				   &cs40l20_dev_attr_group);
	}

	return 0;
}

static const struct of_device_id cs40l20_of_match[] = {
	{.compatible = "cirrus,cs40l20"},
	{},
};

MODULE_DEVICE_TABLE(of, cs40l20_of_match);

static const struct i2c_device_id cs40l20_id[] = {
	{"cs40l20", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cs40l20_id);

static struct i2c_driver cs40l20_i2c_driver = {
	.driver = {
		   .name = "cs40l20",
		   .of_match_table = cs40l20_of_match,
		   },
	.id_table = cs40l20_id,
	.probe = cs40l20_i2c_probe,
	.remove = cs40l20_i2c_remove,
};

module_i2c_driver(cs40l20_i2c_driver);

MODULE_DESCRIPTION("CS40L20 Haptics Driver");
MODULE_AUTHOR("Jeff LaBundy, Cirrus Logic Inc, <jeff.labundy@cirrus.com>");
MODULE_LICENSE("GPL");
