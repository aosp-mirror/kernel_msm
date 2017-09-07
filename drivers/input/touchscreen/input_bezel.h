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

/* After first touch on Bezel, it becomes "20px + BEZEL_INSET_POS" wide
   This should ideally be 48dp or bigger to avoid finger generating display touch/swipe events
   accidentally. So INSET can be 96-20 = 76 or larger. Note a very large value of this
   but a very small value of INIT_ANGULAR_DISTANCE_THRESHOLD will block display swipe events from
   edge of matted bezel area. e.g. Swiping down in Watchface to bring system UI will do nothing till
   we cross this inset */
#define BEZEL_INSET_POS 160

/* Any event in outer this much pixel of display will never be sent as display touch events.
   This value must not be large. Any touch elements in this area will not be tappable/swipable */
#define INIT_BEZEL_INSET 5

/* Approximated to distance for angular movement of 1 degree :
   2 * pi * 227 * (1/360): rounded up; square of 3.96 ... */
#define ANGULAR_DISTANCE_THRESHOLD  16
#define ANGULAR_DISTANCE_THRESHOLD_ROOT  4

/* Approximated to distance for angular movement of 5 degree :
   2 * pi * 227 * (5/360): rounded up; square of 3.9635 * 5 */
#define INIT_ANGULAR_DISTANCE_THRESHOLD  393

/* used in get multiplier for batching case */
#define MAX_ANGLE_REPORTED 200

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
