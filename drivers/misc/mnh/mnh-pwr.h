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

#ifndef __MNH_PWR_H__
#define __MNH_PWR_H__

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>

struct mnh_pwr_controls {
	struct gpio power_on;
	struct gpio suspend_n;
	struct gpio ddr_iso_n;
	struct gpio reset_n;
	struct gpio soc_pwr_good;
	struct gpio clk32k_stop_n;
	struct gpio pcie_clk_sel;
	struct gpio clk_sel;
	struct gpio bypasswake;
	struct gpio boot_strap;
	struct gpio thermtrip;
	struct gpio ready;
};

enum mnh_pwr_state {
	MNH_PWR_S0 = 0,
	MNH_PWR_S3 = 3,
	MNH_PWR_S4 = 4
};

int mnh_pwr_set_state(struct device *dev,
		      const struct mnh_pwr_controls *gpios,
		      enum mnh_pwr_state system_state);
enum mnh_pwr_state mnh_pwr_get_state(struct device *dev);

#endif /* __MNH_PWR_H__ */
