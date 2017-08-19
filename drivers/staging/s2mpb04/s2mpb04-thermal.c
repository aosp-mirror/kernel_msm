/*
 * Copyright (C) 2017 Google, Inc.
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

#include "s2mpb04-core.h"

#define DRIVER_NAME "s2mpb04-thermal"

#define S2MPB04_THERM_NUM_TRIPS 0
#define S2MPB04_THERM_TRIPS_MASK ((1 << S2MPB04_THERM_NUM_TRIPS) - 1)

struct s2mpb04_thermal {
	struct s2mpb04_core *s2mpb04;
	struct thermal_zone_device *tz_dev;
};

static int s2mpb04_thermal_get_temp(struct thermal_zone_device *tz_dev,
				    int *temp)
{
	struct s2mpb04_thermal *s2mpb04_thermal = tz_dev->devdata;
	struct s2mpb04_core *s2mpb04 = s2mpb04_thermal->s2mpb04;
	u8 chan_data = 0;
	int ret;

	ret = s2mpb04_read_adc_chan(s2mpb04, S2MPB04_ADC_PTAT, &chan_data);
	if (ret)
		return ret;

	*temp = PTAT_CODE_TO_TEMP(chan_data);
	return 0;
}

static int s2mpb04_thermal_get_mode(struct thermal_zone_device *tz_dev,
				    enum thermal_device_mode *mode)
{
	*mode = THERMAL_DEVICE_ENABLED;
	return 0;
}

static struct thermal_zone_device_ops s2mpb04_thermal_ops = {
	.get_temp = s2mpb04_thermal_get_temp,
	.get_mode = s2mpb04_thermal_get_mode,
};

static int s2mpb04_thermal_probe(struct platform_device *pdev)
{
	struct s2mpb04_core *s2mpb04 = dev_get_drvdata(pdev->dev.parent);
	struct s2mpb04_thermal *s2mpb04_thermal;
	char tz_name[] = "s2mpb04_tz";

	s2mpb04_thermal = devm_kzalloc(&pdev->dev,
				       sizeof(struct s2mpb04_thermal),
				       GFP_KERNEL);
	if (!s2mpb04_thermal)
		return -ENOMEM;

	s2mpb04_thermal->s2mpb04 = s2mpb04;

	/* register the thermal zone device */
	s2mpb04_thermal->tz_dev =
		thermal_zone_device_register(tz_name, S2MPB04_THERM_NUM_TRIPS,
					     S2MPB04_THERM_TRIPS_MASK,
					     s2mpb04_thermal,
					     &s2mpb04_thermal_ops,
					     NULL, 0, 0);
	if (IS_ERR(s2mpb04_thermal->tz_dev)) {
		dev_err(&pdev->dev, "%s: failed to register thermal zone device (%ld)\n",
			__func__, PTR_ERR(s2mpb04_thermal->tz_dev));
		return -ENODEV;
	}

	platform_set_drvdata(pdev, s2mpb04_thermal);

	return 0;
}

static int s2mpb04_thermal_remove(struct platform_device *pdev)
{
	struct s2mpb04_thermal *s2mpb04_thermal = platform_get_drvdata(pdev);

	if (s2mpb04_thermal->tz_dev)
		thermal_zone_device_unregister(s2mpb04_thermal->tz_dev);
	return 0;
}

static const struct of_device_id s2mpb04_thermal_of_match[] = {
	{ .compatible = "brcm,s2mpb04-thermal", },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpb04_thermal_of_match);

static struct platform_driver s2mpb04_thermal_driver = {
	.probe = s2mpb04_thermal_probe,
	.remove = s2mpb04_thermal_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = s2mpb04_thermal_of_match,
	},
};
module_platform_driver(s2mpb04_thermal_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("S2MPB04 Thermal Sensor Driver");
MODULE_LICENSE("GPL");
