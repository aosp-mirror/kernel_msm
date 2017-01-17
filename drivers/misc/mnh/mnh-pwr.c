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
#define CFG_OUTPUT(elem, val) do { \
		if (gpio_direction_output(gpios->elem.gpio, val)) { \
			dev_err(dev, "%s could not make %s (%d) an output.\n",\
			__func__, gpios->elem.label, gpios->elem.gpio);\
		} else {\
			dev_info(dev, "%s %s (%d) set to output.\n",\
			__func__, gpios->elem.label, gpios->elem.gpio);\
		} \
	} while (0)

#define CFG_INPUT(elem) do { \
		if (gpio_direction_input(gpios->elem.gpio)) {\
			dev_err(dev, "%s could not make %s (%d) an input.\n",\
			__func__, gpios->elem.label, gpios->elem.gpio);\
		} else {\
			dev_info(dev, "%s %s (%d) set to input.\n",\
			__func__, gpios->elem.label, gpios->elem.gpio);\
		} \
	} while (0)

	CFG_OUTPUT(power_on,       0);
	CFG_OUTPUT(suspend_n,      0);
	CFG_OUTPUT(ddr_iso_n,      1);
	CFG_OUTPUT(reset_n,        0);
	CFG_INPUT(soc_pwr_good);
	CFG_OUTPUT(clk32k_stop_n,  0);
	CFG_OUTPUT(pcie_clk_sel,   0);
	CFG_OUTPUT(clk_sel,        0);
	CFG_OUTPUT(bypasswake,     0);
	/* will need to be overridden later for spi */
	CFG_OUTPUT(boot_strap,     0);
	CFG_INPUT(thermtrip);
	CFG_INPUT(ready);
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
	gpio_set_value(gpios->reset_n.gpio,      0);
	gpio_set_value(gpios->suspend_n.gpio,    0);
	_sys_state = MNH_PWR_S3;
}

static void mnh_pwr_up(struct device *dev, const struct mnh_pwr_controls *gpios)
{
	if (!is_valid_system_state(_sys_state)) {
		/* first time powering up set to power down config first */
		mnh_pwr_down(dev, gpios);
		udelay(30);
	}

	if (_sys_state == MNH_PWR_S4)
		gpio_set_value(gpios->power_on.gpio, 1);

	gpio_set_value(gpios->suspend_n.gpio, 1);
	msleep(20);
	dev_info(dev, "%s hardcoded wait for power good done.", __func__);
	gpio_set_value(gpios->reset_n.gpio, 1);

	msleep(20);
	dev_info(dev, "%s hardcoded wait for ready done.", __func__);
	_sys_state = MNH_PWR_S0;
}

int mnh_pwr_set_state(struct device *dev,
			const struct mnh_pwr_controls *gpios,
			enum mnh_pwr_state system_state)
{
	int ret = -1;

	dev_info(dev, "%s req: %d, current: %d\n",
		 __func__, system_state, mnh_pwr_get_state(dev));
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
	dev_info(dev, "%s done with state: %d\n",
		 __func__, mnh_pwr_get_state(dev));
	return ret;
}
EXPORT_SYMBOL_GPL(mnh_pwr_set_state);

enum mnh_pwr_state mnh_pwr_get_state(struct device *dev)
{
	return _sys_state;
}
EXPORT_SYMBOL_GPL(mnh_pwr_get_state);
