/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt)	"%s:%d: " fmt, __func__, __LINE__
#include <linux/backlight.h>

#include "dsi_panel.h"

#define BL_NODE_NAME_SIZE 32


static u32 dsi_backlight_calculate(struct dsi_backlight_config *bl,
				   int brightness)
{
	int bl_lvl = 0;

	if (brightness) {
		int bl_min = bl->bl_min_level ? : 1;
		int bl_range = bl->bl_max_level - bl_min;
		int bl_temp;

		/* scale backlight */
		bl_temp = mult_frac(brightness, bl->bl_scale,
				    MAX_BL_SCALE_LEVEL);

		bl_temp = mult_frac(bl_temp, bl->bl_scale_ad,
				    MAX_AD_BL_SCALE_LEVEL);


		/* map UI brightness into driver backlight level rounding it */
		if (bl_temp > 1)
			bl_lvl = DIV_ROUND_CLOSEST((bl_temp - 1) * bl_range,
					bl->brightness_max_level - 1);
		bl_lvl += bl_min;

		pr_debug("brightness=%d, bl_scale=%d, ad=%d, bl_lvl=%d\n",
			 brightness, bl->bl_scale,
			 bl->bl_scale_ad, bl_lvl);
	}

	return bl_lvl;
}

static int dsi_backlight_update_status(struct backlight_device *bd)
{
	struct dsi_backlight_config *bl = bl_get_data(bd);
	struct dsi_panel *panel = container_of(bl, struct dsi_panel, bl_config);
	int brightness = bd->props.brightness;
	int bl_lvl;
	int rc = 0;

	if ((bd->props.state & (BL_CORE_FBBLANK | BL_CORE_SUSPENDED)) ||
			(bd->props.power != FB_BLANK_UNBLANK))
		brightness = 0;

	bl_lvl = dsi_backlight_calculate(bl, brightness);
	if (bl_lvl == bl->bl_actual)
		return 0;

	mutex_lock(&panel->panel_lock);
	if (dsi_panel_initialized(panel)) {
		rc = dsi_panel_set_backlight(panel, bl_lvl);
		if (rc) {
			pr_err("unable to set backlight (%d)\n", rc);
			goto done;
		}
	}
	bl->bl_actual = bl_lvl;

done:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

static int dsi_backlight_get_brightness(struct backlight_device *bd)
{
	struct dsi_backlight_config *bl = bl_get_data(bd);

	return bl->bl_actual;
}

static const struct backlight_ops dsi_backlight_ops = {
	.update_status = dsi_backlight_update_status,
	.get_brightness = dsi_backlight_get_brightness,
};

int dsi_backlight_register(struct dsi_backlight_config *bl)
{
	static int display_count;
	char bl_node_name[BL_NODE_NAME_SIZE];
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.power = FB_BLANK_UNBLANK,
	};
	struct dsi_panel *panel = container_of(bl, struct dsi_panel, bl_config);

	props.max_brightness = bl->brightness_max_level;
	props.brightness = bl->brightness_max_level / 2;

	snprintf(bl_node_name, BL_NODE_NAME_SIZE, "panel%u-backlight",
		 display_count);
	bl->bl_device = devm_backlight_device_register(panel->parent,
				bl_node_name, panel->parent, bl,
				&dsi_backlight_ops, &props);
	if (IS_ERR_OR_NULL(bl->bl_device)) {
		bl->bl_device = NULL;
		return -ENODEV;
	}
	display_count++;
	return 0;
}

int dsi_backlight_update_dpms(struct dsi_backlight_config *bl, int power_mode)
{
	struct backlight_device *bd = bl->bl_device;
	int rc = 0;

	if (!bd)
		return 0;

	mutex_lock(&bd->ops_lock);
	switch (power_mode) {
	case SDE_MODE_DPMS_ON:
		bd->props.power = FB_BLANK_UNBLANK;
		bd->props.state &= ~BL_CORE_FBBLANK;
		break;
	case SDE_MODE_DPMS_OFF:
		bd->props.power = FB_BLANK_POWERDOWN;
		bd->props.state |= BL_CORE_FBBLANK;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	if (!rc)
		backlight_update_status(bd);
	mutex_unlock(&bd->ops_lock);

	return rc;
}
