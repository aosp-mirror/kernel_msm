/*
 * touch_spi.c
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the¬
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include "touch_core.h"
#include "touch_filter.h"

/* MACROs and functions for event_filter */
#define f_sub(x, y)	(x > y ? x - y : y - x)
#define time_interval(a, b)	(a >= b ? a - b : 1000000 + a - b)

struct timeval ex_debug[EX_PROFILE_MAX];
struct t_data prev_touch;
unsigned int debug_cnt = 0;

static bool is_valid_jitter(struct device *dev,
		struct t_data prev_data, u16 x, u16 y, int jitter)
{
	bool ret;
	ret = (f_sub(prev_data.x, x) <= jitter
			&& (f_sub(prev_data.y, y)
			<= jitter));
	return ret;
}

bool chk_time_interval(struct timeval t_aft, struct timeval t_bef, int t_val)
{
	int interval = t_val * 1000;
	if (t_aft.tv_sec - t_bef.tv_sec == 0) {
		if ((time_interval(t_aft.tv_usec, t_bef.tv_usec)) <= interval)
			return true;
	} else if (t_aft.tv_sec - t_bef.tv_sec == 1) {
		if (t_aft.tv_usec + 1000000 - t_bef.tv_usec <= interval)
			return true;
	}

	return false;
}

int debug_detect_filter(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int id = count - 1;
	bool chk_time_1;
	bool chk_time_2;
	bool is_valid;
	u16 z_chk = 0;

	/* z-value check */
	z_chk = ts->tdata[id].pressure;
	if (count > 0 && ((z_chk > 255) || (z_chk < 20))) {
		TOUCH_I("abnormal z value : %d\n", debug_cnt);
		debug_cnt++;
	}

	is_valid = is_valid_jitter(dev,
			prev_touch,
			ts->tdata[id].x,
			ts->tdata[id].y,
			DEBUG_DETECT_JITTER);

	/* plus count senario */
	if (prev_touch.count < count) {
		do_gettimeofday(&ex_debug[DEBUG_CURR_P]);

		chk_time_1 = chk_time_interval(ex_debug[DEBUG_CURR_P],
				ex_debug[DEBUG_PREV_P],
				DEBUG_PRESS_TIME);

		chk_time_2 = chk_time_interval(ex_debug[DEBUG_CURR_P],
				ex_debug[DEBUG_CURR_R],
				DEBUG_RELEASE_TIME);

		if (prev_touch.count > 0
				&& chk_time_1 && is_valid) {
			TOUCH_I("P-P check : %d\n", debug_cnt);
			debug_cnt++;
		} else if (chk_time_2 && is_valid) {
			TOUCH_I("P-R-P check : %d\n", debug_cnt);
			debug_cnt++;
		}

		if (f_sub(prev_touch.count, count) >= 5) {
			TOUCH_I("diff fingers num check : %d\n", debug_cnt);
			debug_cnt++;
		}

		ex_debug[DEBUG_PREV_P] = ex_debug[DEBUG_CURR_P];

	/* minus count senario */
	} else if (prev_touch.count > count) {
		do_gettimeofday(&ex_debug[DEBUG_CURR_R]);

		if (chk_time_interval(ex_debug[DEBUG_CURR_R],
					ex_debug[DEBUG_PREV_P],
					DEBUG_SUBTRACTION_TIME) && is_valid) {
			TOUCH_I("finger subtraction check : %d\n", debug_cnt);
			debug_cnt++;
		}
	}

	prev_touch.x = ts->tdata[id].x;
	prev_touch.y = ts->tdata[id].y;
	prev_touch.count = count;

	if (debug_cnt > DEBUG_REPORT_CNT)
		goto report_debug_info;

	return DEBUG_NOT_REPORT;

report_debug_info:
	debug_cnt = 0;
	TOUCH_I("debug info logging start\n");
	return DEBUG_REPORT;
}

