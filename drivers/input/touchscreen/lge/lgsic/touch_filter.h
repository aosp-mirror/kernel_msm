/*
 * touch_spi.h
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
#define DEBUG_PRESS_TIME 10
#define DEBUG_RELEASE_TIME 10
#define DEBUG_SUBTRACTION_TIME 10
#define DEBUG_WQ_TIME 8
#define DEBUG_FRAME_CNT 500
#define DEBUG_REPORT_CNT 10
#define DEBUG_DETECT_JITTER 20

enum {
	DEBUG_PREV_P,
	DEBUG_CURR_P,
	DEBUG_CURR_R,
	FORCE_CURR_P,
	FORCE_PREV_P,
	EX_PROFILE_MAX,
};

enum {
	DEBUG_NOT_REPORT = 0,
	DEBUG_REPORT,
};

struct t_data {
	u16	id;
	u16	x;
	u16	y;
	u16	width_major;
	u16	width_minor;
	s16	orientation;
	u16	pressure;
	u16	type;
	u16	count;
};

extern int debug_detect_filter(struct device *dev, int count);
