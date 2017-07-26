/* Copyright (c) 2017, LGE Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LEDS_LM36272_H
#define __LEDS_LM36272_H

struct lm36272_platform_data {
	void (*platform_init)(void);
	int bl_gpio;
	int dsv_p_gpio;
	int dsv_n_gpio;
	int min_brightness;
	int max_brightness;
	int default_brightness;
	int blmap_size;
	u16 *blmap;
};

#ifdef CONFIG_LEDS_LM36272
int lm36272_dsv_ctrl(int dsv_en);
void lm36272_backlight_ctrl(int level);
#else
static inline int lm36272_dsv_ctrl(int dsv_en)
{
	return 0;
}
static inline void lm36272_backlight_ctrl(int level);
{
	return;
}
#endif

#endif /* __LEDS_LM36272_H */
