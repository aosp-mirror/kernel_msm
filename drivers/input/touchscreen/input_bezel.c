/* drivers/input/touchscreen/input_bezel.c
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

#define pr_fmt(fmt) KBUILD_BASENAME ": " fmt

#include <linux/init.h>
#include <linux/types.h>
#include <linux/idr.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/major.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/input.h>
#include "input_bezel.h"

#define DEVICE_NAME "touch_bezel"


static struct input_dev *input_dev;
static int first_y = -1, first_x = -1;

/* Diameter of circular touch panel in pixels is 494,
   Hence Radius of circular panel is 494/2 = 247 */
static int tp_center;

/* Calculate absolute value between two codinates */

int ABS(int a, int b) {
	if (b > a) {
		return (b - a);
	}
	else {
		return (a - b);
	}
}

int SquareRootRounded(int a_nInput)
{
	int op  = a_nInput;
	int res = 0;
	int one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

	// "one" starts at the highest power of four <= than the argument.
	while (one > op) {
		one >>= 2;
	}

	while (one != 0) {
		if (op >= res + one) {
			op = op - (res + one);
			res = res +  2 * one;
		}
		res >>= 1;
		one >>= 2;
	}

	if (op > res) {
		res++;
	}

	return res;
}

/* Check the rotation is clock-wise/anticlock-wise */
bool isAntiClockWise(int x, int y)
{
     int res = ((first_x - tp_center)*(y - tp_center) - (first_y - tp_center)*(x -tp_center)) ;
     if(res > 0)
	return true;
     else
	return false;
}

/* this api is added to avoid rounding errors of division to find multipler */
int getMultiplier(int dist, int step)
{
	int mult, nxtMult ;
	// KEEP STEP EVEN to make this work well
	int stepHalf = step/2;
	int lower_limit, upper_limit = 0;
	for(mult = 1; mult < MAX_ANGLE_REPORTED ; ++mult) {
		nxtMult = mult+1;
		// initially will be 2 & 6; then 6 & 10, 10 & 14 etc
		lower_limit = step * mult - stepHalf;
		upper_limit = step * nxtMult - stepHalf;
		if( (dist >= lower_limit) && (dist < upper_limit) )
			break;
	}
	return mult;
}


/* Report bezel event while touching into bezel area */
int bezel_report_wheel(int x, int y, struct bezel_data *bdata)
{
	int dx = 0, dy = 0, tempDis2 = 0;
	int total_ev = 0;
	int rad_pos = 0;
        int dist = 0;


	rad_pos = SquareRootRounded(bdata->sq_rad_position);
        x = tp_center + (x - tp_center) * (tp_center - 20) / rad_pos;
        y = tp_center + (y - tp_center) * (tp_center - 20) / rad_pos;
	/* If the it is bezel first touch,
	 * it will store initial x and y codinate
	 */
	if((first_x == -1)&&(first_y == -1)) {
		first_y = y;
		first_x = x;
		bdata->step_threshold2 = INIT_ANGULAR_DISTANCE_THRESHOLD;
		return 0;
	}
	/* find distance travelled since last EV_REL
	 * posted or since bezel touch started
	 */
	dx = ABS(x , first_x);
	dy = ABS(y , first_y);

        // pr_err("bezel_report_wheel:after mapping x = %d, y = %d, first_x = %d, first_y = %d\n", x, y, first_x, first_y);

	tempDis2 = dx * dx + dy * dy;

	/* If the distance between codinate (x0, y0) and codinate (x1, y1)
	 * is gretter thaen the 3 drgee angular displacement, check for the
	 * bezel rotation direct either clock-wise or aniclock-wise and post
	 * event to user space depending on number of 3 degree displacement
	 */
	if(tempDis2 >= (bdata->step_threshold2)) {
		/* Check for number of 1 degree displacement with circular rotation */
                dist = SquareRootRounded(tempDis2);
                // division is causing rounding to lower int
		// total_ev = dist/ANGULAR_DISTANCE_THRESHOLD_ROOT;
		total_ev = getMultiplier(dist, ANGULAR_DISTANCE_THRESHOLD_ROOT);
		// pr_err("tempDis2= %d, threshold= %d, dist= %d, total_ev = %d \n\n", tempDis2, bdata->step_threshold2, dist, total_ev);

		if(isAntiClockWise(x,y)) {
			input_report_rel(input_dev, REL_WHEEL, -1*total_ev);
		} else {
			input_report_rel(input_dev, REL_WHEEL, total_ev);
		}

		input_sync(input_dev);

		/* Save previous coordinates */
		first_y = y;
		first_x = x;
		if(bdata->step_threshold2 == INIT_ANGULAR_DISTANCE_THRESHOLD)
			bdata->step_threshold2 = bdata->angular_dist_thresh;
	}

	return 0;
}

/* Reset bezel coordinates to -1 with finger lift */
void bezel_reset(void) {
	first_y = -1;
	first_x = -1;
}

/* Register Bezel as separate input device */
int bezel_register_device(struct device *dev, unsigned int panel_maxx)
{
	int err;

	input_dev = input_allocate_device();
	if(!input_dev) {
		err = -ENOMEM;
		goto error_1;
	}

	input_dev->name = DEVICE_NAME;
        input_dev->id.bustype = BUS_I2C;
	__set_bit(EV_REL, input_dev->evbit);
	__set_bit(REL_WHEEL, input_dev->relbit);

	if (input_register_device(input_dev)) {
                dev_err(dev, "failed to register input device\n");
                goto error_1;
        }

	tp_center = panel_maxx/2;
	return 0;

error_1:
	return err;
}
