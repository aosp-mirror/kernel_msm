/*
 * arch/arm/mach-msm/include/mach/msm_flashlight.h - The flashlight header
 * Copyright (C) 2009-2015  HTC Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __HTC_FLASHLIGHT_H
#define __HTC_FLASHLIGHT_H

#include <linux/list.h>

enum flashlight_mode_flags {
	FL_MODE_OFF = 0,
	FL_MODE_TORCH,
	FL_MODE_FLASH,
	FL_MODE_PRE_FLASH,
	FL_MODE_TORCH_LED_A,
	FL_MODE_TORCH_LED_B,
	FL_MODE_TORCH_LEVEL_0,
	FL_MODE_TORCH_LEVEL_1,
	FL_MODE_TORCH_LEVEL_2,
	FL_MODE_CAMERA_EFFECT_FLASH,
	FL_MODE_CAMERA_EFFECT_PRE_FLASH,
	FL_MODE_FLASH_LEVEL1,
	FL_MODE_FLASH_LEVEL2,
	FL_MODE_FLASH_LEVEL3,
	FL_MODE_FLASH_LEVEL4,
	FL_MODE_FLASH_LEVEL5,
	FL_MODE_FLASH_LEVEL6,
	FL_MODE_FLASH_LEVEL7,
	FL_MODE_VIDEO_TORCH = 30,
	FL_MODE_VIDEO_TORCH_1,
	FL_MODE_VIDEO_TORCH_2,
	FL_MODE_VIDEO_TORCH_3,
	FL_MODE_VIDEO_TORCH_4,
};

struct htc_flashlight_dev {
	u8 id;
	struct list_head list;

	int (*flash_func)(struct htc_flashlight_dev *, int, int);
	int (*torch_func)(struct htc_flashlight_dev *, int, int);
};

extern int (*htc_flash_main)(int, int);
extern int (*htc_torch_main)(int, int);
extern int (*htc_flash_front)(int, int);
extern int (*htc_torch_front)(int, int);

#ifdef __KERNEL__
#ifdef CONFIG_HTC_FLASHLIGHT
int register_htc_flashlight(struct htc_flashlight_dev *);
int unregister_htc_flashlight(struct htc_flashlight_dev *);
#else
static inline int register_htc_flashlight(struct htc_flashlight_dev *dev)
{
	return -EINVAL;
}

static inline int unregister_htc_flashlight(struct htc_flashlight_dev *dev)
{
	return -EINVAL;
}
#endif
#endif

#endif
