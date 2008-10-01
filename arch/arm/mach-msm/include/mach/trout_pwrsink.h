/* include/asm/mach-msm/trout_pwrsink.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (C) 2008 HTC Corporation.
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
#ifndef _ARCH_ARM_MACH_MSM_TROUT_PWRSINK_H_
#define _ARCH_ARM_MACH_MSM_TROUT_PWRSINK_H_

typedef enum {
	PWRSINK_SYSTEM_LOAD = 0,
	PWRSINK_AUDIO,
	PWRSINK_BACKLIGHT,
	PWRSINK_LED_BUTTON,
	PWRSINK_LED_KEYBOARD,
	PWRSINK_GP_CLK,
	PWRSINK_BLUETOOTH,
	PWRSINK_CAMERA,
	PWRSINK_SDCARD,
	PWRSINK_VIDEO,
	PWRSINK_WIFI,

	PWRSINK_LAST = PWRSINK_WIFI,
	PWRSINK_INVALID
} pwrsink_id_type;

struct pwr_sink {
	pwrsink_id_type	id;
	unsigned	ua_max;
	unsigned	percent_util;
};

struct pwr_sink_platform_data {
	unsigned	num_sinks;
	struct pwr_sink	*sinks;
};

#ifndef CONFIG_TROUT_PWRSINK
static inline int trout_pwrsink_set(pwrsink_id_type id, unsigned percent)
{
printk("%s:STUB!\n", __func__);
	return 0;
}
#else
extern int trout_pwrsink_set(pwrsink_id_type id, unsigned percent);
#endif

#endif

