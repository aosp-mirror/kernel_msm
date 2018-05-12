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
#include <linux/sysfs.h>
#include <linux/list.h>
#include <linux/list_sort.h>
#include <video/mipi_display.h>

#include "dsi_panel.h"

#define BL_NODE_NAME_SIZE 32

#define BL_STATE_LP		BL_CORE_DRIVER1
#define BL_STATE_LP2		BL_CORE_DRIVER2

struct dsi_backlight_pwm_config {
	bool pwm_pmi_control;
	u32 pwm_pmic_bank;
	u32 pwm_period_usecs;
	int pwm_gpio;
};

static inline bool is_lp_mode(unsigned long state)
{
	return (state & (BL_STATE_LP | BL_STATE_LP2)) != 0;
}

static int dsi_panel_pwm_bl_register(struct dsi_backlight_config *bl);

static void dsi_panel_bl_free_unregister(struct dsi_backlight_config *bl)
{
	kfree(bl->priv);
}

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

	mutex_lock(&panel->panel_lock);
	num_params = bl->bl_max_level > 0xFF ? 2 : 1;
	rc = mipi_dsi_dcs_set_display_brightness(dsi, bl_lvl, num_params);
	if (rc < 0)
		pr_err("failed to update dcs backlight:%d\n", bl_lvl);

	mutex_unlock(&panel->panel_lock);
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
	if (bl_lvl == bl->bl_actual && bl->last_state == bd->props.state)
		goto done;

	if (dsi_panel_initialized(panel) && bl->update_bl) {
		pr_info("req:%d bl:%d state:0x%x\n",
			bd->props.brightness, bl_lvl, bd->props.state);

		rc = bl->update_bl(bl, bl_lvl);
		if (rc) {
			pr_err("unable to set backlight (%d)\n", rc);
			goto done;
		}
	}
	bl->bl_actual = bl_lvl;
	bl->last_state = bd->props.state;

done:
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

static ssize_t alpm_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct dsi_backlight_config *bl = bl_get_data(bd);
	struct dsi_panel *panel = container_of(bl, struct dsi_panel, bl_config);
	int rc, alpm_mode;
	const unsigned int lp_state = bl->bl_device->props.state &
			(BL_STATE_LP | BL_STATE_LP2);

	rc = kstrtoint(buf, 0, &alpm_mode);
	if (rc)
		return rc;

	if (bl->bl_device->props.state & BL_CORE_FBBLANK) {
		return -EINVAL;
	} else if ((alpm_mode == 1) && (lp_state != BL_STATE_LP)) {
		pr_info("activating lp1 mode\n");
		dsi_panel_set_lp1(panel);
	} else if ((alpm_mode > 1) && !(lp_state & BL_STATE_LP2)) {
		pr_info("activating lp2 mode\n");
		dsi_panel_set_lp2(panel);
	} else if (!alpm_mode && lp_state) {
		pr_info("activating normal mode\n");
		dsi_panel_set_nolp(panel);
	}

	return count;
}

static ssize_t alpm_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	int alpm_mode;

	if (bd->props.state & BL_STATE_LP2)
		alpm_mode = 2;
	else
		alpm_mode = (bd->props.state & BL_STATE_LP) != 0;

	return sprintf(buf, "%d\n", alpm_mode);
}
static DEVICE_ATTR_RW(alpm_mode);

static ssize_t vr_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = NULL;
	struct dsi_backlight_config *bl = NULL;
	struct dsi_panel *panel = NULL;
	int rc = 0;
	bool vr_mode = false;

	/* dev is non-NULL, enforced by sysfs_create_file_ns */
	bd = to_backlight_device(dev);
	bl = bl_get_data(bd);
	panel = container_of(bl, struct dsi_panel, bl_config);

	rc = kstrtobool(buf, &vr_mode);
	if (rc)
		return rc;

	rc = dsi_panel_update_vr_mode(panel, vr_mode);

	if (!rc && !vr_mode) {
		bl->bl_actual = -1;
		if (backlight_update_status(bd))
			pr_warn("couldn't restore backlight after VR exit");
	}

	if (rc)
		return rc;
	else
		return count;
}

static ssize_t vr_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = NULL;
	struct dsi_backlight_config *bl = NULL;
	struct dsi_panel *panel = NULL;
	bool vr_mode = false;

	/* dev is non-NULL, enforced by sysfs_create_file_ns */
	bd = to_backlight_device(dev);
	bl = bl_get_data(bd);
	panel = container_of(bl, struct dsi_panel, bl_config);

	vr_mode = dsi_panel_get_vr_mode(panel);

	return snprintf(buf, PAGE_SIZE, "%s\n", vr_mode ? "on" : "off");
}

static DEVICE_ATTR_RW(vr_mode);

static struct attribute *bl_device_attrs[] = {
	&dev_attr_alpm_mode.attr,
	&dev_attr_vr_mode.attr,
	NULL,
};
ATTRIBUTE_GROUPS(bl_device);

#ifdef CONFIG_LEDS_TRIGGERS
static int dsi_panel_update_backlight_led(struct dsi_backlight_config *bl,
					  u32 bl_lvl)
{
	struct led_trigger *wled = bl->priv;
	if (!wled)
		return -EINVAL;

	if (bl_lvl != bl->bl_actual)
		led_trigger_event(wled, bl_lvl);

	return 0;
}

static void dsi_panel_led_bl_unregister(struct dsi_backlight_config *bl)
{
	struct led_trigger *wled = bl->priv;

	if (wled) {
		led_trigger_unregister_simple(wled);
		bl->priv = NULL;
	}
}

static int dsi_panel_led_bl_register(struct dsi_backlight_config *bl)
{
	struct dsi_panel *panel = container_of(bl, struct dsi_panel, bl_config);
	struct led_trigger *wled;
	int rc = 0;

	led_trigger_register_simple("bkl-trigger", &wled);

	/* LED APIs don't tell us directly whether a classdev has yet
	 * been registered to service this trigger. Until classdev is
	 * registered, calling led_trigger has no effect, and doesn't
	 * fail. Classdevs are associated with any registered triggers
	 * when they do register, but that is too late for FBCon.
	 * Check the cdev list directly and defer if appropriate.
	 */
	if (!wled) {
		pr_err("[%s] backlight registration failed\n", panel->name);
		rc = -EINVAL;
	} else {
		read_lock(&wled->leddev_list_lock);
		if (list_empty(&wled->led_cdevs))
			rc = -EPROBE_DEFER;
		read_unlock(&wled->leddev_list_lock);

		if (rc) {
			pr_info("[%s] backlight %s not ready, defer probe\n",
				panel->name, wled->name);
			led_trigger_unregister_simple(wled);
		}

		bl->priv = wled;
		bl->update_bl = dsi_panel_update_backlight_led;
		bl->unregister = dsi_panel_led_bl_unregister;
	}

	return rc;
}
#else
static int dsi_panel_led_bl_register(struct dsi_backlight_config *bl)
{
	return 0;
}
#endif

static int dsi_backlight_register(struct dsi_backlight_config *bl)
{
	static int display_count;
	char bl_node_name[BL_NODE_NAME_SIZE];
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.power = FB_BLANK_UNBLANK,
	};
	struct dsi_panel *panel = container_of(bl, struct dsi_panel, bl_config);
	struct regulator *reg;

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

	if (sysfs_create_groups(&bl->bl_device->dev.kobj, bl_device_groups))
		pr_warn("unable to create device groups\n");

	reg = regulator_get(panel->parent, "lab");
	if (!PTR_ERR_OR_ZERO(reg)) {
		pr_info("LAB regulator found\n");
		panel->bl_config.lab_vreg = reg;
	}

	display_count++;
	return 0;
}

static unsigned long get_state_after_dpms(struct dsi_backlight_config *bl,
				   int power_mode)
{
	struct backlight_device *bd = bl->bl_device;
	unsigned long state = bd->props.state;

	switch (power_mode) {
	case SDE_MODE_DPMS_ON:
		state &= ~(BL_CORE_FBBLANK | BL_STATE_LP | BL_STATE_LP2);
		break;
	case SDE_MODE_DPMS_OFF:
		state &= ~(BL_STATE_LP | BL_STATE_LP2);
		state |= BL_CORE_FBBLANK;
		break;
	case SDE_MODE_DPMS_LP1:
		state |= BL_STATE_LP;
		state &= ~BL_STATE_LP2;
		break;
	case SDE_MODE_DPMS_LP2:
		state |= BL_STATE_LP | BL_STATE_LP2;
		break;
	}

	return state;
}

int dsi_backlight_early_dpms(struct dsi_backlight_config *bl, int power_mode)
{
	struct backlight_device *bd = bl->bl_device;
	unsigned long state;

	if (!bd)
		return 0;

	pr_info("power_mode:%d state:0x%0x\n", power_mode, bd->props.state);

	mutex_lock(&bd->ops_lock);
	state = get_state_after_dpms(bl, power_mode);

	if (bl->lab_vreg) {
		if (is_lp_mode(bl->last_state)) {
			if (state & BL_CORE_FBBLANK) {
				/* LP -> OFF */
				pr_debug("set lab vreg mode: standby\n");
				regulator_set_mode(bl->lab_vreg,
						   REGULATOR_MODE_STANDBY);
			} else if (!is_lp_mode(state)) {
				/* LP -> ON */
				pr_debug("set lab vreg mode: normal\n");
				regulator_set_mode(bl->lab_vreg,
						   REGULATOR_MODE_NORMAL);
			}
		} else if (is_lp_mode(state)) {
			/* no LP -> LP */
			pr_debug("set lab vreg mode: idle\n");
			regulator_set_mode(bl->lab_vreg, REGULATOR_MODE_IDLE);
		}
	}
	mutex_unlock(&bd->ops_lock);

	return 0;
}

int dsi_backlight_late_dpms(struct dsi_backlight_config *bl, int power_mode)
{
	struct backlight_device *bd = bl->bl_device;
	unsigned long state;

	if (!bd)
		return 0;

	pr_debug("power_mode:%d state:0x%0x\n", power_mode, bd->props.state);

	mutex_lock(&bd->ops_lock);
	state = get_state_after_dpms(bl, power_mode);

	bd->props.power = state & BL_CORE_FBBLANK ? FB_BLANK_POWERDOWN :
			FB_BLANK_UNBLANK;
	bd->props.state = state;

	backlight_update_status(bd);
	mutex_unlock(&bd->ops_lock);

	return 0;
}

int dsi_backlight_get_dpms(struct dsi_backlight_config *bl)
{
	struct backlight_device *bd = bl->bl_device;
	int power = 0;
	int state = 0;

	mutex_lock(&bd->ops_lock);
	power = bd->props.power;
	state = bd->props.state;
	mutex_unlock(&bd->ops_lock);

	if (power == FB_BLANK_POWERDOWN)
		return SDE_MODE_DPMS_OFF;
	else if (state & BL_STATE_LP2)
		return SDE_MODE_DPMS_LP2;
	else if (state & BL_STATE_LP)
		return SDE_MODE_DPMS_LP1;
	else
		return SDE_MODE_DPMS_ON;
}

#define MAX_BINNED_BL_MODES 10

struct binned_lp_node {
	struct list_head head;
	const char *name;
	u32 bl_threshold;
	struct dsi_panel_cmd_set dsi_cmd;
};

struct binned_lp_data {
	struct list_head mode_list;
	struct binned_lp_node *last_lp_mode;
	struct binned_lp_node priv_pool[MAX_BINNED_BL_MODES];
};

static int dsi_panel_binned_bl_update(struct dsi_backlight_config *bl,
				      u32 bl_lvl)
{
	struct dsi_panel *panel = container_of(bl, struct dsi_panel, bl_config);
	struct binned_lp_data *lp_data = bl->priv;
	struct binned_lp_node *node = NULL;
	struct backlight_properties *props = &bl->bl_device->props;

	if (is_lp_mode(props->state)) {
		struct binned_lp_node *tmp;

		list_for_each_entry(tmp, &lp_data->mode_list, head) {
			if (props->brightness <= tmp->bl_threshold) {
				node = tmp;
				break;
			}
		}
		WARN(!node, "unable to find lp node for bl_lvl: %d\n",
		     props->brightness);
	}

	if (node != lp_data->last_lp_mode) {
		lp_data->last_lp_mode = node;
		if (node) {
			pr_debug("switching display lp mode: %s (%d)\n",
				node->name, props->brightness);
			mutex_lock(&panel->panel_lock);
			dsi_panel_cmd_set_transfer(panel, &node->dsi_cmd);
			mutex_unlock(&panel->panel_lock);
		} else {
			/* ensure update after lpm */
			bl->bl_actual = -1;
		}
	}

	/* no need to send backlight command if HLPM active */
	if (node)
		return 0;

	return dsi_backlight_update_dcs(bl, bl_lvl);
}

static int _dsi_panel_binned_lp_parse(struct device_node *np,
				      struct binned_lp_node *node)
{
	int rc;
	u32 val = 0;

	rc = of_property_read_u32(np, "google,dsi-lp-brightness-threshold",
				  &val);
	/* treat lack of property as max threshold */
	node->bl_threshold = !rc ? val : UINT_MAX;

	rc = dsi_panel_parse_dt_cmd_set(np, "google,dsi-lp-command",
					"google,dsi-lp-command-state",
					&node->dsi_cmd);
	if (rc) {
		pr_err("Unable to parse dsi-lp-command\n");
		return rc;
	}

	of_property_read_string(np, "label", &node->name);

	pr_debug("Successfully parsed lp mode: %s threshold: %d\n",
		node->name, node->bl_threshold);

	return 0;
}

static int _dsi_panel_binned_bl_cmp(void *priv, struct list_head *lha,
				    struct list_head *lhb)
{
	struct binned_lp_node *a = list_entry(lha, struct binned_lp_node, head);
	struct binned_lp_node *b = list_entry(lhb, struct binned_lp_node, head);

	return a->bl_threshold - b->bl_threshold;
}

static int dsi_panel_binned_lp_register(struct dsi_backlight_config *bl)
{
	struct dsi_panel *panel = container_of(bl, struct dsi_panel, bl_config);
	struct binned_lp_data *lp_data;
	struct device_node *lp_modes_np, *child_np;
	struct binned_lp_node *lp_node;
	int num_modes;
	int rc = -ENOTSUPP;

	lp_data = kzalloc(sizeof(*lp_data), GFP_KERNEL);
	if (!lp_data)
		return -ENOMEM;

	lp_modes_np = of_get_child_by_name(panel->panel_of_node,
					   "google,lp-modes");

	if (!lp_modes_np) {
		kfree(lp_data);
		return rc;
	}

	num_modes = of_get_child_count(lp_modes_np);
	if (!num_modes || (num_modes > MAX_BINNED_BL_MODES)) {
		pr_err("Invalid binned brightness modes: %d\n", num_modes);
		goto exit;
	}

	INIT_LIST_HEAD(&lp_data->mode_list);
	lp_node = lp_data->priv_pool;

	for_each_child_of_node(lp_modes_np, child_np) {
		rc = _dsi_panel_binned_lp_parse(child_np, lp_node);
		if (rc)
			goto exit;

		list_add_tail(&lp_node->head, &lp_data->mode_list);
		lp_node++;
		if (lp_node > &lp_data->priv_pool[MAX_BINNED_BL_MODES - 1]) {
			pr_err("Too many LP modes\n");
			rc = -ENOTSUPP;
			goto exit;
		}
	}
	list_sort(NULL, &lp_data->mode_list, _dsi_panel_binned_bl_cmp);

	bl->update_bl = dsi_panel_binned_bl_update;
	bl->unregister = dsi_panel_bl_free_unregister;
	bl->priv = lp_data;

exit:
	of_node_put(lp_modes_np);
	if (rc)
		kfree(lp_data);

	return rc;
}

static const struct of_device_id dsi_backlight_dt_match[] = {
	{
		.compatible = "google,dsi_binned_lp",
		.data = dsi_panel_binned_lp_register,
	},
	{}
};

int dsi_panel_bl_register(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_backlight_config *bl = &panel->bl_config;
	const struct of_device_id *match;
	int (*register_func)(struct dsi_backlight_config *) = NULL;

	match = of_match_node(dsi_backlight_dt_match, panel->panel_of_node);
	if (match && match->data) {
		register_func = match->data;
		rc = register_func(bl);
	}

	if (!register_func || (rc == -ENOTSUPP)) {
		switch (bl->type) {
		case DSI_BACKLIGHT_WLED:
			register_func = dsi_panel_led_bl_register;
			break;
		case DSI_BACKLIGHT_DCS:
			bl->update_bl = dsi_backlight_update_dcs;
			break;
		case DSI_BACKLIGHT_PWM:
			register_func = dsi_panel_pwm_bl_register;
			break;
		default:
			pr_err("Backlight type(%d) not supported\n", bl->type);
			rc = -ENOTSUPP;
			break;
		}

		if (register_func)
			rc = register_func(bl);
	}

	if (!rc)
		rc = dsi_backlight_register(bl);

	return rc;
}

int dsi_panel_bl_unregister(struct dsi_panel *panel)
{
	struct dsi_backlight_config *bl = &panel->bl_config;

	if (bl->unregister)
		bl->unregister(bl);

	if (bl->bl_device)
		sysfs_remove_groups(&bl->bl_device->dev.kobj, bl_device_groups);

	return 0;
}

static int dsi_panel_bl_parse_pwm_config(struct dsi_backlight_pwm_config *config,
					 struct device_node *of_node)
{
	int rc = 0;
	u32 val;

	rc = of_property_read_u32(of_node, "qcom,dsi-bl-pmic-bank-select",
				  &val);
	if (rc) {
		pr_err("bl-pmic-bank-select is not defined, rc=%d\n", rc);
		goto error;
	}
	config->pwm_pmic_bank = val;

	rc = of_property_read_u32(of_node, "qcom,dsi-bl-pmic-pwm-frequency",
				  &val);
	if (rc) {
		pr_err("bl-pmic-bank-select is not defined, rc=%d\n", rc);
		goto error;
	}
	config->pwm_period_usecs = val;

	config->pwm_pmi_control = of_property_read_bool(of_node,
						"qcom,mdss-dsi-bl-pwm-pmi");

	config->pwm_gpio = of_get_named_gpio(of_node,
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

static int dsi_panel_pwm_bl_register(struct dsi_backlight_config *bl)
{
	struct dsi_panel *panel = container_of(bl, struct dsi_panel, bl_config);
	struct dsi_backlight_pwm_config *pwm_cfg;
	int rc;

	pwm_cfg = kzalloc(sizeof(*pwm_cfg), GFP_KERNEL);
	if (!pwm_cfg)
		return -ENOMEM;


	rc = dsi_panel_bl_parse_pwm_config(pwm_cfg, panel->panel_of_node);
	if (rc) {
		kfree(pwm_cfg);
		return rc;
	}

	bl->priv = pwm_cfg;
	bl->unregister = dsi_panel_bl_free_unregister;

	return 0;
}

int dsi_panel_bl_parse_config(struct dsi_backlight_config *bl,
			      struct device_node *of_node)
{
	struct dsi_panel *panel = container_of(bl, struct dsi_panel, bl_config);
	int rc = 0;
	const char *bl_type;
	u32 val = 0;

	bl_type = of_get_property(of_node,
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

	rc = of_property_read_u32(of_node, "qcom,mdss-dsi-bl-min-level", &val);
	if (rc) {
		pr_debug("[%s] bl-min-level unspecified, defaulting to zero\n",
			 panel->name);
		bl->bl_min_level = 0;
	} else {
		bl->bl_min_level = val;
	}

	rc = of_property_read_u32(of_node, "qcom,mdss-dsi-bl-max-level", &val);
	if (rc) {
		pr_debug("[%s] bl-max-level unspecified, defaulting to max level\n",
			 panel->name);
		bl->bl_max_level = MAX_BL_LEVEL;
	} else {
		bl->bl_max_level = val;
	}

	rc = of_property_read_u32(of_node, "qcom,mdss-brightness-max-level",
		&val);
	if (rc) {
		pr_debug("[%s] brigheness-max-level unspecified, defaulting to 255\n",
			 panel->name);
		bl->brightness_max_level = 255;
	} else {
		bl->brightness_max_level = val;
	}

	bl->en_gpio = of_get_named_gpio(of_node,
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
