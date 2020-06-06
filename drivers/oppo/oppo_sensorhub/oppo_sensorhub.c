/*
 * oppo_sensorhub.c - Linux kernel modules for OPPO SensorHub
 *
 * Copyright (C), 2008-2019, OPPO Mobile Comm Corp., Ltd.
 * Author: Zeng Zhaoxiu <zengzhaoxiu@oppo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include "oppo_sensorhub.h"

struct oppo_sensorhub {
	struct device *dev;
	struct oppo_sensorhub_platform_data *pdata;
	struct mutex reset_lock;
	struct blocking_notifier_head rst_nh;
};

#define SNSHUB_RESET_ACTIVE_TIME	10
#define SNSHUB_RESET_DELAY_TIME		50

static inline void oppo_sensorhub_hw_reset(struct oppo_sensorhub *snshub, bool assert)
{
	gpio_set_value(snshub->pdata->gpio_reset,
					!!snshub->pdata->gpio_reset_active_low ^ !!assert);
}

int oppo_sensorhub_reset_notifier_register(struct device *dev, struct notifier_block *nb)
{
	struct oppo_sensorhub *snshub = dev_get_drvdata(dev->parent);
	return blocking_notifier_chain_register(&snshub->rst_nh, nb);
}
EXPORT_SYMBOL_GPL(oppo_sensorhub_reset_notifier_register);

int oppo_sensorhub_reset_notifier_unregister(struct device *dev, struct notifier_block *nb)
{
	struct oppo_sensorhub *snshub = dev_get_drvdata(dev->parent);
	return blocking_notifier_chain_unregister(&snshub->rst_nh, nb);
}
EXPORT_SYMBOL_GPL(oppo_sensorhub_reset_notifier_unregister);

static void __oppo_sensorhub_assert_reset(struct oppo_sensorhub *snshub, void *v)
{
	mutex_lock(&snshub->reset_lock);

	blocking_notifier_call_chain(&snshub->rst_nh, SNSHUB_PRE_RESET, v);

	oppo_sensorhub_hw_reset(snshub, true);
	msleep(SNSHUB_RESET_ACTIVE_TIME);
}

static void __oppo_sensorhub_deassert_reset(struct oppo_sensorhub *snshub, void *v)
{
	oppo_sensorhub_hw_reset(snshub, false);
	msleep(SNSHUB_RESET_DELAY_TIME);

	blocking_notifier_call_chain(&snshub->rst_nh, SNSHUB_POST_RESET, v);

	mutex_unlock(&snshub->reset_lock);
}

static void __oppo_sensorhub_reset(struct oppo_sensorhub *snshub, void *v)
{
	if (mutex_trylock(&snshub->reset_lock)) {
		blocking_notifier_call_chain(&snshub->rst_nh, SNSHUB_PRE_RESET, v);

		/* assert reset pin */
		oppo_sensorhub_hw_reset(snshub, true);
		msleep(SNSHUB_RESET_ACTIVE_TIME);

		/* deassert reset pin */
		oppo_sensorhub_hw_reset(snshub, false);
		msleep(SNSHUB_RESET_DELAY_TIME);

		blocking_notifier_call_chain(&snshub->rst_nh, SNSHUB_POST_RESET, v);
	} else {
		/* another resetting is in progress, wait it done */
		mutex_lock(&snshub->reset_lock);
	}
	mutex_unlock(&snshub->reset_lock);
}

void oppo_sensorhub_assert_reset(struct device *dev)
{
	struct oppo_sensorhub *snshub = dev_get_drvdata(dev->parent);
	__oppo_sensorhub_assert_reset(snshub, dev);
}
EXPORT_SYMBOL_GPL(oppo_sensorhub_assert_reset);

void oppo_sensorhub_deassert_reset(struct device *dev)
{
	struct oppo_sensorhub *snshub = dev_get_drvdata(dev->parent);
	__oppo_sensorhub_deassert_reset(snshub, dev);
}
EXPORT_SYMBOL_GPL(oppo_sensorhub_deassert_reset);

void oppo_sensorhub_reset(struct device *dev)
{
	struct oppo_sensorhub *snshub = dev_get_drvdata(dev->parent);
	__oppo_sensorhub_reset(snshub, dev);
}
EXPORT_SYMBOL_GPL(oppo_sensorhub_reset);

static ssize_t oppo_sensorhub_store_reset(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct oppo_sensorhub *snshub = dev_get_drvdata(dev);
	long val;

	if (kstrtol(buf, 10, &val) != 0 || !val)
		return -EINVAL;

	__oppo_sensorhub_reset(snshub, NULL);

	return count;
}

static DEVICE_ATTR(reset, 0200, NULL, oppo_sensorhub_store_reset);

static struct attribute *oppo_sensorhub_attributes[] = {
	&dev_attr_reset.attr,
	NULL
};

static const struct attribute_group oppo_sensorhub_attr_group = {
	.attrs	= oppo_sensorhub_attributes,
};

static struct oppo_sensorhub_platform_data *oppo_sensorhub_parse_dt(struct device *dev)
{
	struct oppo_sensorhub_platform_data *pdata;
	enum of_gpio_flags flags;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	/* reset gpio */
	ret = of_get_named_gpio_flags(dev->of_node, "reset-gpio", 0, &flags);
	if (ret < 0) {
		dev_err(dev, "Unable to read reset gpio\n");
		return NULL;
	}

	pdata->gpio_reset = ret;
	pdata->gpio_reset_active_low = (flags & OF_GPIO_ACTIVE_LOW) ? true : false;

	return pdata;
}

static const struct mfd_cell oppo_sensorhub_devs[] = {
	{
		.name = "swd",
		.of_compatible = "oppo,swd",
	},
	{
		.name = "spidev-snshub",
		.of_compatible = "oppo,spidev-snshub",
	},
};

static int oppo_sensorhub_probe(struct platform_device *pdev)
{
	struct oppo_sensorhub_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct oppo_sensorhub *snshub;
	struct pinctrl *pinctrl;
	int ret;
	dev_err(&pdev->dev, "libin-1\n");
	if (!pdata) {
		pdata = oppo_sensorhub_parse_dt(&pdev->dev);
		if (!pdata)
			return -ENOMEM;
	}

	snshub = devm_kzalloc(&pdev->dev, sizeof(*snshub), GFP_KERNEL);
	if (!snshub)
		return -ENOMEM;

	snshub->dev = &pdev->dev;
	snshub->pdata = pdata;
	BLOCKING_INIT_NOTIFIER_HEAD(&snshub->rst_nh);
	mutex_init(&snshub->reset_lock);

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);

	ret = mfd_add_devices(&pdev->dev, 0, oppo_sensorhub_devs,
			ARRAY_SIZE(oppo_sensorhub_devs), NULL, 0, NULL);
	if (ret) {
		dev_err(&pdev->dev, "add mfd devices failed: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &oppo_sensorhub_attr_group);
	if (ret) {
		mfd_remove_devices(&pdev->dev);

		dev_err(&pdev->dev,
			"Failed to create attribute group: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, snshub);
	dev_info(&pdev->dev, "OPPO SensorHub register successful\n");

	return 0;
}

static int oppo_sensorhub_remove(struct platform_device *pdev)
{
//	struct oppo_sensorhub *snshub = platform_get_drvdata(pdev); 

	sysfs_remove_group(&pdev->dev.kobj, &oppo_sensorhub_attr_group);

	mfd_remove_devices(&pdev->dev);

	return 0;
}

static struct of_device_id of_oppo_sensorhub_match_tbl[] = {
	{ .compatible = "oppo,sensor-hub", },
	{},
};

static struct platform_driver oppo_sensorhub_driver = {
	.driver = {
		.name	= "sensor-hub",
		.of_match_table = of_oppo_sensorhub_match_tbl,
	},
	.probe	= oppo_sensorhub_probe,
	.remove	= oppo_sensorhub_remove,
};
module_platform_driver(oppo_sensorhub_driver);

MODULE_AUTHOR("Zeng Zhaoxiu <zengzhaoxiu@oppo.com>");
MODULE_DESCRIPTION("OPPO SensorHub driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: OPPO-SensorHub");
