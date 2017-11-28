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
#include <linux/of_gpio.h>

#include "dsi_panel.h"

#define BL_NODE_NAME_SIZE 32

#define BL_STATE_LP		BL_CORE_DRIVER1
#define BL_STATE_LP2		BL_CORE_DRIVER2

static int dsi_backlight_update_dcs(struct dsi_backlight_config *bl, u32 bl_lvl)
{
	int rc = 0;
	struct dsi_panel *panel;
	struct mipi_dsi_device *dsi;
	size_t num_params;

	if (!bl || (bl_lvl > 0xffff)) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	panel = container_of(bl, struct dsi_panel, bl_config);
	/* if no change in backlight, abort */
	if (bl_lvl == bl->bl_actual)
		return 0;

	dsi = &panel->mipi_device;

	num_params = bl->bl_max_level > 0xFF ? 2 : 1;
	rc = mipi_dsi_dcs_set_display_brightness(dsi, bl_lvl, num_params);
	if (rc < 0)
		pr_err("failed to update dcs backlight:%d\n", bl_lvl);

	return rc;
}

static int dsi_panel_set_backlight(struct dsi_panel *panel, u32 bl_lvl)
{
	int rc = 0;
	struct dsi_backlight_config *bl = &panel->bl_config;

	pr_debug("backlight type:%d lvl:%d\n", bl->type, bl_lvl);
	switch (bl->type) {
	case DSI_BACKLIGHT_WLED:
		rc = backlight_device_set_brightness(bl->bl_device, bl_lvl);
		break;
	case DSI_BACKLIGHT_DCS:
		rc = dsi_backlight_update_dcs(bl, bl_lvl);
		break;
	default:
		pr_err("Backlight type(%d) not supported\n", bl->type);
		rc = -ENOTSUPP;
	}

	return rc;
}

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

static int dsi_backlight_register(struct dsi_backlight_config *bl)
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

	pr_info("power_mode:%d state:0x%0x\n", power_mode, bd->props.state);

	mutex_lock(&bd->ops_lock);
	switch (power_mode) {
	case SDE_MODE_DPMS_ON:
		bd->props.power = FB_BLANK_UNBLANK;
		bd->props.state &= ~(BL_CORE_FBBLANK | BL_STATE_LP |
				     BL_STATE_LP2);
		break;
	case SDE_MODE_DPMS_OFF:
		bd->props.power = FB_BLANK_POWERDOWN;
		bd->props.state &= ~(BL_STATE_LP | BL_STATE_LP2);
		bd->props.state |= BL_CORE_FBBLANK;
		break;
	case SDE_MODE_DPMS_LP1:
		bd->props.state |= BL_STATE_LP;
		bd->props.state &= ~BL_STATE_LP2;
		break;
	case SDE_MODE_DPMS_LP2:
		bd->props.state |= BL_STATE_LP | BL_STATE_LP2;
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

int dsi_panel_bl_register(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_backlight_config *bl = &panel->bl_config;

	switch (bl->type) {
	case DSI_BACKLIGHT_WLED:
	case DSI_BACKLIGHT_DCS:
		break;
	default:
		pr_err("Backlight type(%d) not supported\n", bl->type);
		rc = -ENOTSUPP;
		goto error;
	}

	rc = dsi_backlight_register(bl);
error:
	return rc;
}


int dsi_panel_bl_unregister(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_backlight_config *bl = &panel->bl_config;

	switch (bl->type) {
	case DSI_BACKLIGHT_WLED:
	case DSI_BACKLIGHT_DCS:
		break;
	default:
		pr_err("Backlight type(%d) not supported\n", bl->type);
		rc = -ENOTSUPP;
		goto error;
	}

error:
	return rc;
}

static int dsi_panel_bl_parse_pwm_config(struct dsi_panel *panel)
{
	int rc = 0;
	u32 val;
	struct dsi_backlight_config *config = &panel->bl_config;
	struct dsi_parser_utils *utils = &panel->utils;

	rc = utils->read_u32(utils->data, "qcom,dsi-bl-pmic-bank-select",
				  &val);
	if (rc) {
		pr_err("bl-pmic-bank-select is not defined, rc=%d\n", rc);
		goto error;
	}
	config->pwm_pmic_bank = val;

	rc = utils->read_u32(utils->data, "qcom,dsi-bl-pmic-pwm-frequency",
				  &val);
	if (rc) {
		pr_err("bl-pmic-bank-select is not defined, rc=%d\n", rc);
		goto error;
	}
	config->pwm_period_usecs = val;

	config->pwm_pmi_control = utils->read_bool(utils->data,
						"qcom,mdss-dsi-bl-pwm-pmi");

	config->pwm_gpio = utils->get_named_gpio(utils->data,
					     "qcom,mdss-dsi-pwm-gpio",
					     0);
	if (!gpio_is_valid(config->pwm_gpio)) {
		pr_err("pwm gpio is invalid\n");
		rc = -EINVAL;
		goto error;
	}

error:
	return rc;
}

int dsi_panel_bl_parse_config(struct dsi_backlight_config *bl)
{
	struct dsi_panel *panel = container_of(bl, struct dsi_panel, bl_config);
	int rc = 0;
	u32 val = 0;
	const char *bl_type;
	struct dsi_parser_utils *utils = &panel->utils;

	bl_type = utils->get_property(utils->data,
				  "qcom,mdss-dsi-bl-pmic-control-type",
				  NULL);
	if (!bl_type) {
		bl->type = DSI_BACKLIGHT_UNKNOWN;
	} else if (!strcmp(bl_type, "bl_ctrl_pwm")) {
		bl->type = DSI_BACKLIGHT_PWM;
	} else if (!strcmp(bl_type, "bl_ctrl_wled")) {
		bl->type = DSI_BACKLIGHT_WLED;
	} else if (!strcmp(bl_type, "bl_ctrl_dcs")) {
		bl->type = DSI_BACKLIGHT_DCS;
	} else {
		pr_debug("[%s] bl-pmic-control-type unknown-%s\n",
			 panel->name, bl_type);
		bl->type = DSI_BACKLIGHT_UNKNOWN;
	}

	bl->bl_scale = MAX_BL_SCALE_LEVEL;
	bl->bl_scale_ad = MAX_AD_BL_SCALE_LEVEL;

	rc = utils->read_u32(utils->data, "qcom,mdss-dsi-bl-min-level", &val);
	if (rc) {
		pr_debug("[%s] bl-min-level unspecified, defaulting to zero\n",
			 panel->name);
		bl->bl_min_level = 0;
	} else {
		bl->bl_min_level = val;
	}

	rc = utils->read_u32(utils->data, "qcom,mdss-dsi-bl-max-level", &val);
	if (rc) {
		pr_debug("[%s] bl-max-level unspecified, defaulting to max level\n",
			 panel->name);
		bl->bl_max_level = MAX_BL_LEVEL;
	} else {
		bl->bl_max_level = val;
	}

	rc = utils->read_u32(utils->data, "qcom,mdss-brightness-max-level",
		&val);
	if (rc) {
		pr_debug("[%s] brigheness-max-level unspecified, defaulting to 255\n",
			 panel->name);
		bl->brightness_max_level = 255;
	} else {
		bl->brightness_max_level = val;
	}

	if (bl->type == DSI_BACKLIGHT_PWM) {
		rc = dsi_panel_bl_parse_pwm_config(panel);
		if (rc) {
			pr_err("[%s] failed to parse pwm config, rc=%d\n",
			       panel->name, rc);
			goto error;
		}
	}

	panel->bl_config.en_gpio = utils->get_named_gpio(utils->data,
					      "qcom,platform-bklight-en-gpio",
					      0);
	if (!gpio_is_valid(bl->en_gpio)) {
		pr_debug("[%s] failed get bklt gpio, rc=%d\n", panel->name, rc);
		rc = 0;
		goto error;
	}

error:
	return rc;
}
