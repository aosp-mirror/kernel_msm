/* drivers/input/touchscreen/input_bezel.h
 *
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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

#ifndef _INPUT_BEZEL_H
#define _INPUT_BEZEL_H

#include <linux/list.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/mod_devicetable.h>

#define BEZEL_INSET_POS 60
/* Approximated to distance for angular movement of 3 degree :
   2 * pi * 227 * (3/360): rounded up; square of 11.88... */
#define ANGULAR_DISTANCE_THRESHOLD  16

/* Approximated to distance for angular movement of 3 degree :
   2 * pi * 227 * (10/360): rounded up; square of 39.61... */
#define INIT_ANGULAR_DISTANCE_THRESHOLD  1568

struct bezel_data {
	u8 inset;
	u16 angular_dist_thresh;
	u16 step_threshold2;
	u8 thickness;
	u32 sq_rad_position;
	bool bezel_touch_status;
};

int bezel_register_device(struct device *dev, unsigned int panel_maxx);
int bezel_report_wheel(int x, int y, struct bezel_data *bdata);
void bezel_reset(void);
int ABS(int a, int b) ;

#endif /* _INPUT_BEZEL_H */
