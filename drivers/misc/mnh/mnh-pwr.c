/*
 *
 * MNH PWR APIs
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include "mnh-pwr.h"

static enum mnh_pwr_state _sys_state = -1;

static int is_valid_system_state(enum mnh_pwr_state sys_state)
{
	int ret = 0;

	if ((sys_state == MNH_PWR_S0) ||
		(sys_state == MNH_PWR_S3) ||
		(sys_state == MNH_PWR_S4)) {
		ret = 1;
	}
	return ret;
}

static void mnh_pwr_config_gpios(struct device *dev,
				 const struct mnh_pwr_controls *gpios)
{
	if (gpio_direction_input(gpios->soc_pwr_good.gpio)) {
		dev_err(dev, "%s could not make soc_pwr_good (%d) an input\n",
			__func__, gpios->soc_pwr_good.gpio);
	} else {
		dev_info(dev, "%s soc_pwr_good (%d) set to input\n",
			__func__, gpios->soc_pwr_good.gpio);
	}

	if (gpio_direction_output(gpios->reset_n.gpio, 0)) {
		dev_err(dev, "%s could not make reset_n (%d) an output\n",
			__func__, gpios->reset_n.gpio);
	} else {
		dev_info(dev, "%s reset_n (%d) set to output low\n",
			__func__, gpios->reset_n.gpio);
	}

	if (gpio_direction_output(gpios->suspend_n.gpio, 0)) {
		dev_err(dev, "%s could not make suspend_n (%d) an output\n",
			__func__, gpios->suspend_n.gpio);
	} else {
		dev_info(dev, "%s suspend_n (%d) set to output low\n",
			__func__, gpios->suspend_n.gpio);
	}

	if (gpio_direction_output(gpios->power_on.gpio, 0)) {
		dev_err(dev, "%s could not make power_on (%d) an output\n",
			__func__, gpios->power_on.gpio);
	} else {
		dev_info(dev, "%s power_on (%d) set to output low\n",
			__func__, gpios->power_on.gpio);
	}

	if (gpio_direction_output(gpios->ddr_iso_n.gpio, 1)) {
		dev_err(dev, "%s could not make ddr_iso_n (%d) an output\n",
			__func__, gpios->ddr_iso_n.gpio);
	} else {
		dev_info(dev, "%s ddr_iso_n (%d) set to output high\n",
			__func__, gpios->ddr_iso_n.gpio);
	}

}

static void mnh_pwr_down(struct device *dev,
			 const struct mnh_pwr_controls *gpios)
{
	if (!is_valid_system_state(_sys_state)) {
		mnh_pwr_config_gpios(dev, gpios);
	} else {
		gpio_set_value(gpios->reset_n.gpio,   0);
		gpio_set_value(gpios->power_on.gpio,  0);
		gpio_set_value(gpios->suspend_n.gpio, 0);
	}
	_sys_state = MNH_PWR_S4;
}

static void mnh_pwr_suspend(struct device *dev,
			    const struct mnh_pwr_controls *gpios)
{
	/*
	 * should have already put mem system into refresh
	 * and asserted ddr_iso_n from mnh-dram driver.
	 */
	gpio_set_value(gpios->suspend_n.gpio, 0);
	while (gpio_get_value(gpios->soc_pwr_good.gpio) == 1)
		udelay(1);
	_sys_state = MNH_PWR_S3;
}

static void mnh_pwr_up(struct device *dev, const struct mnh_pwr_controls *gpios)
{
	if (!is_valid_system_state(_sys_state)) {
		/* first time powering up set to power down config first */
		mnh_pwr_down(dev, gpios);
		udelay(30);
	}

	gpio_set_value(gpios->power_on.gpio,  1);
	gpio_set_value(gpios->suspend_n.gpio, 1);
	while (gpio_get_value(gpios->soc_pwr_good.gpio) == 0)
		udelay(1);
	udelay(30);
	gpio_set_value(gpios->reset_n.gpio,   1);
	_sys_state = MNH_PWR_S0;
}

int mnh_pwr_set_state(struct device *dev,
			const struct mnh_pwr_controls *gpios,
			enum mnh_pwr_state system_state)
{
	int ret = -1;

	if (is_valid_system_state(system_state)) {
		ret = 0;
		switch (system_state) {
		case MNH_PWR_S0:
			mnh_pwr_up(dev, gpios);
			break;
		case MNH_PWR_S3:
			mnh_pwr_suspend(dev, gpios);
			break;
		case MNH_PWR_S4:
			mnh_pwr_down(dev, gpios);
			break;
		}
	}
	return ret;
}

enum mnh_pwr_state mnh_pwr_get_state(struct device *dev)
{
	return _sys_state;
}
