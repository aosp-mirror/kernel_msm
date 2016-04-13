/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>

struct rpm_version {
	uint8_t build;
	uint8_t minor;
	uint16_t major;
};

struct drv_data {
	struct rpm_version *ver;
};

static ssize_t rpm_stat_ver_get(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct drv_data *data = dev_get_drvdata(dev);
	struct rpm_version *ver = data->ver;

	ret = snprintf(buf, PAGE_SIZE, "%d.%d.%d", ver->major, ver->minor, ver->build);

	return ret;
}

static DEVICE_ATTR(version, S_IRUSR, rpm_stat_ver_get, NULL);

static int rpm_stats_ver_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	void __iomem *rpm_ver_base;
	struct drv_data *data;

	data = kzalloc(sizeof(struct drv_data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Cannot create device attribute");
		return -ENOMEM;
	}

	/* Get the location of the NPA log's start address offset */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rpm_ver_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rpm_ver_base))
		return PTR_ERR(rpm_ver_base);

	data->ver = rpm_ver_base;

	platform_set_drvdata(pdev, data);

	ret = device_create_file(&pdev->dev, &dev_attr_version);
	if (ret)
		dev_err(&pdev->dev, "Cannot create device attribute");

	pr_info("RPM Version: %d.%d.%d\n", data->ver->major, data->ver->minor, data->ver->build);

	return ret;
}

static int rpm_stats_ver_remove(struct platform_device *pdev)
{
	struct drv_data *data;

	data = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_version);

	kfree(data);

	return 0;
}

static const struct of_device_id rpm_stats_ver_table[] = {
	{.compatible = "qcom,rpm-stats-ver"},
	{},
};

static struct platform_driver rpm_stats_ver_driver = {
	.probe  = rpm_stats_ver_probe,
	.remove = rpm_stats_ver_remove,
	.driver = {
		.name = "rpm-stats-ver",
		.of_match_table = rpm_stats_ver_table,
	},
};
module_platform_driver(rpm_stats_ver_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("RPM Stats Version driver");
MODULE_ALIAS("platform:rpm-stats-ver");
