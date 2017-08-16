/*
 * Copyright (C) 2016 Google, Inc.
 *
 * Author: Trevor Bunker <trevorbunker@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#include "bcm15602-regulator.h"

#define DRIVER_NAME "bcm15602-thermal"

#define BCM15602_THERM_NUM_TRIPS 0
#define BCM15602_THERM_TRIPS_MASK ((1 << BCM15602_THERM_NUM_TRIPS) - 1)

struct bcm15602_thermal {
	struct bcm15602_chip *bcm15602;
	struct thermal_zone_device *tz_dev;
};

static int bcm15602_thermal_get_temp(struct thermal_zone_device *tz_dev,
				     int *temp)
{
	struct bcm15602_thermal *bcm15602_thermal = tz_dev->devdata;
	struct bcm15602_chip *bcm15602 = bcm15602_thermal->bcm15602;
	u16 chan_data = 0;
	int ret;

	ret = bcm15602_read_adc_chan(bcm15602, BCM15602_ADC_PTAT, &chan_data);
	if (ret)
		return ret;

	*temp = PTAT_CODE_TO_TEMP(chan_data);
	return 0;
}

static int bcm15602_thermal_get_mode(struct thermal_zone_device *tz_dev,
				     enum thermal_device_mode *mode)
{
	*mode = THERMAL_DEVICE_ENABLED;
	return 0;
}

static struct thermal_zone_device_ops bcm15602_thermal_ops = {
	.get_temp = bcm15602_thermal_get_temp,
	.get_mode = bcm15602_thermal_get_mode,
};

static int bcm15602_thermal_probe(struct platform_device *pdev)
{
	struct bcm15602_chip *bcm15602 = dev_get_drvdata(pdev->dev.parent);
	struct bcm15602_thermal *bcm15602_thermal;
	char tz_name[] = "bcm15602_tz";

	bcm15602_thermal = devm_kzalloc(&pdev->dev,
					sizeof(struct bcm15602_thermal),
					GFP_KERNEL);
	if (!bcm15602_thermal)
		return -ENOMEM;

	bcm15602_thermal->bcm15602 = bcm15602;

	/* register the thermal zone device */
	bcm15602_thermal->tz_dev =
		thermal_zone_device_register(tz_name, BCM15602_THERM_NUM_TRIPS,
					     BCM15602_THERM_TRIPS_MASK,
					     bcm15602_thermal,
					     &bcm15602_thermal_ops,
					     NULL, 0, 0);
	if (IS_ERR(bcm15602_thermal->tz_dev)) {
		dev_err(&pdev->dev, "%s: failed to register thermal zone device (%ld)\n",
			__func__, PTR_ERR(bcm15602_thermal->tz_dev));
		return -ENODEV;
	}

	platform_set_drvdata(pdev, bcm15602_thermal);

	return 0;
}

static int bcm15602_thermal_remove(struct platform_device *pdev)
{
	struct bcm15602_thermal *bcm15602_thermal = platform_get_drvdata(pdev);

	if (bcm15602_thermal->tz_dev)
		thermal_zone_device_unregister(bcm15602_thermal->tz_dev);
	return 0;
}

static const struct of_device_id bcm15602_thermal_of_match[] = {
	{ .compatible = "brcm,bcm15602-thermal", },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm15602_thermal_of_match);

static struct platform_driver bcm15602_thermal_driver = {
	.probe = bcm15602_thermal_probe,
	.remove = bcm15602_thermal_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = bcm15602_thermal_of_match,
	},
};
module_platform_driver(bcm15602_thermal_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("BCM15602 Thermal Sensor Driver");
MODULE_LICENSE("GPL");
