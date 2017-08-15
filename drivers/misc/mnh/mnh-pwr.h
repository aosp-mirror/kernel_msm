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
#include <linux/platform_device.h>

enum mnh_pwr_state {
	MNH_PWR_S0 = 0, /* active mode */
	MNH_PWR_S3 = 3, /* suspend mode */
	MNH_PWR_S4 = 4, /* power off */
};

int mnh_pwr_set_state(enum mnh_pwr_state system_state);
enum mnh_pwr_state mnh_pwr_get_state(void);
int mnh_pwr_init(struct platform_device *pdev, struct device *dev);

#endif /* __MNH_PWR_H__ */
