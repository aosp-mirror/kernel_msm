/*
 * Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
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

#ifndef _DSI_PANEL_H_
#define _DSI_PANEL_H_

#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/leds.h>
#include <drm/drm_panel.h>
#include <drm/msm_drm.h>

#include "dsi_defs.h"
#include "dsi_ctrl_hw.h"
#include "dsi_clk.h"
#include "dsi_pwr.h"
#include "msm_drv.h"

#define MAX_BL_LEVEL 4096
#define MAX_BL_SCALE_LEVEL 1024
#define MAX_AD_BL_SCALE_LEVEL 65535
#define DSI_CMD_PPS_SIZE 135

#define DSI_MODE_MAX 5
#define HBM_RANGE_MAX 4

enum dsi_panel_rotation {
	DSI_PANEL_ROTATE_NONE = 0,
	DSI_PANEL_ROTATE_HV_FLIP,
	DSI_PANEL_ROTATE_H_FLIP,
	DSI_PANEL_ROTATE_V_FLIP
};

enum dsi_backlight_type {
	DSI_BACKLIGHT_PWM = 0,
	DSI_BACKLIGHT_WLED,
	DSI_BACKLIGHT_DCS,
	DSI_BACKLIGHT_UNKNOWN,
	DSI_BACKLIGHT_MAX,
};

enum {
	MODE_GPIO_NOT_VALID = 0,
	MODE_SEL_DUAL_PORT,
	MODE_SEL_SINGLE_PORT,
	MODE_GPIO_HIGH,
	MODE_GPIO_LOW,
};

enum dsi_dms_mode {
	DSI_DMS_MODE_DISABLED = 0,
	DSI_DMS_MODE_RES_SWITCH_IMMEDIATE,
};

struct dsi_dfps_capabilities {
	bool dfps_support;
	enum dsi_dfps_type type;
	u32 min_refresh_rate;
	u32 max_refresh_rate;
};

struct dsi_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *active;
	struct pinctrl_state *suspend;
};

struct dsi_panel_phy_props {
	u32 panel_width_mm;
	u32 panel_height_mm;
	enum dsi_panel_rotation rotation;
};

struct hbm_range {
	/* Userspace brightness range (inclusive) for this HBM range */
	u32 user_bri_start;
	u32 user_bri_end;

	/* Panel brightness range (inclusive) for this HBM range */
	u32 panel_bri_start;
	u32 panel_bri_end;

	/* Command to be sent to the panel when entering this HBM range */
	struct dsi_panel_cmd_set entry_cmd;
	/*
	 * Command to be sent to the panel to stop brightness dimming while
	 * in this HBM range.
	 */
	struct dsi_panel_cmd_set dimming_stop_cmd;
	/* Number of frames dimming will take. */
	u32 num_dimming_frames;
};

struct hbm_data {
	/* Command to be sent to the panel when exiting HBM */
	struct dsi_panel_cmd_set exit_cmd;
	/* Command to be sent to the panel to stop brightness dimming */
	struct dsi_panel_cmd_set exit_dimming_stop_cmd;
	/* Number of frames dimming will take */
	u32 exit_num_dimming_frames;

	struct hbm_range ranges[HBM_RANGE_MAX];
	u32 num_ranges;
	u32 cur_range;

	/* Brightness dimming currently active */
	bool dimming_active;
	/* Total number of frames brightness dimming takes */
	u32 dimming_frames_total;
	/* Number of frames remaining until brightness settles */
	u32 dimming_frames_left;
	/* DSI command to send once brightness dimming settles */
	struct dsi_panel_cmd_set *dimming_stop_cmd;

	/* Work queue used to count frames during dimming */
	struct workqueue_struct *dimming_workq;
	struct work_struct dimming_work;
	struct dsi_panel *panel;
};

struct dsi_backlight_config {
	enum dsi_backlight_type type;

	u32 bl_min_level;
	u32 bl_max_level;
	u32 brightness_max_level;
	u32 bl_scale;
	u32 bl_scale_ad;
	u32 bl_actual;
	u16 *lut;
	unsigned int last_state;

	/* Minimum safe brightness level during VR mode */
	u32 bl_vr_min_safe_level;

	struct mutex state_lock;
	struct hbm_data *hbm;

	int en_gpio;

	struct backlight_device *bl_device;
	struct regulator *lab_vreg;

	void *priv;

	/**
	 * update_bl - function used to update backlight. Called with panel_lock
	 * locked.
	 * @bl_cfg - ptr to backlight config struct
	 * @bl_lvl - backlight level set
	 *
	 * return: non-zero on success otherwise errno
	 */
	int (*update_bl)(struct dsi_backlight_config *bl_cfg, u32 bl_lvl);

	/**
	 * unregister - unregisters and frees any backlight data
	 * @bl_cfg - ptr to backlight config struct
	 */
	void (*unregister)(struct dsi_backlight_config *bl_cfg);

	/**
	 * debugfs_init - debugfs initialization for DSI backlight
	 * @parent - dentry to create
	 * @bl_cfg - ptr to backlight config struct
	 */
	void (*debugfs_init)(struct dentry *parent,
			     struct dsi_backlight_config *bl_cfg);
};

struct dsi_reset_seq {
	u32 level;
	u32 sleep_ms;
};

struct dsi_panel_reset_config {
	struct dsi_reset_seq *sequence;
	u32 count;

	int reset_gpio;
	int disp_en_gpio;
	int lcd_mode_sel_gpio;
	u32 mode_sel_state;
};

struct dsi_panel_debug {
	u8 reg_read_cmd;
	size_t reg_read_len;
};

enum esd_check_status_mode {
	ESD_MODE_REG_READ,
	ESD_MODE_SW_BTA,
	ESD_MODE_PANEL_TE,
	ESD_MODE_IRQ_GPIO,
	ESD_MODE_MAX
};

enum esd_irq_mode {
	ESD_IRQ_MODE_TRIGGER_RISING = 1,
	ESD_IRQ_MODE_TRIGGER_FALLING = 2,
	ESD_IRQ_MODE_TRIGGER_HIGH = 4,
	ESD_IRQ_MODE_TRIGGER_LOW = 8,
	ESD_IRQ_MODE_MAX
};

struct drm_panel_esd_config {
	bool esd_enabled;

	enum esd_check_status_mode status_mode;
	struct dsi_panel_cmd_set status_cmd;
	u32 *status_cmds_rlen;
	u32 *status_valid_params;
	u32 *status_value;
	u8 *return_buf;
	u8 *status_buf;
	u32 groups;

	enum esd_irq_mode irq_mode;
	int irq_gpio;
	bool irq_enabled;
	bool irq_mode_prepared;
};

enum dsi_panel_type {
	DSI_PANEL = 0,
	EXT_BRIDGE,
	DSI_PANEL_TYPE_MAX,
};

struct dsi_panel_sn_location {
	u32 start_byte;
	u32 sn_length;
	u8 addr;
};

struct dsi_panel_vendor_info {
	struct dsi_panel_sn_location location;
	bool is_sn;
	u8 *sn;
	const char *name;
};

struct dsi_panel {
	const char *name;
	enum dsi_panel_type type;
	struct device_node *panel_of_node;
	struct mipi_dsi_device mipi_device;

	struct mutex panel_lock;
	struct drm_panel drm_panel;
	struct mipi_dsi_host *host;
	struct device *parent;

	struct dsi_host_common_cfg host_config;
	struct dsi_video_engine_cfg video_config;
	struct dsi_cmd_engine_cfg cmd_config;
	enum dsi_op_mode panel_mode;

	struct dsi_dfps_capabilities dfps_caps;
	struct dsi_panel_phy_props phy_props;

	struct dsi_display_mode *cur_mode;
	u32 num_timing_nodes;

	struct dsi_regulator_info power_info;
	struct dsi_backlight_config bl_config;
	struct dsi_panel_reset_config reset_config;
	struct dsi_pinctrl_info pinctrl;
	struct drm_panel_hdr_properties hdr_props;
	struct dsi_panel_debug debug;
	struct drm_panel_esd_config esd_config;
	struct dsi_panel_vendor_info vendor_info;

	u32 init_delay_us;
	bool hs_pps;
	bool lp11_init;
	bool ulps_enabled;
	bool ulps_suspend_enabled;
	bool allow_phy_power_off;

	bool panel_initialized;
	bool te_using_watchdog_timer;

	char dsc_pps_cmd[DSI_CMD_PPS_SIZE];
	enum dsi_dms_mode dms_mode;

	bool sync_broadcast_en;

	/* the following set of members are guarded by panel_lock */
	bool vr_mode;
	bool hbm_mode;
};

static inline bool dsi_panel_ulps_feature_enabled(struct dsi_panel *panel)
{
	return panel->ulps_enabled;
}

static inline bool dsi_panel_initialized(struct dsi_panel *panel)
{
	return panel->panel_initialized;
}

static inline void dsi_panel_acquire_panel_lock(struct dsi_panel *panel)
{
	mutex_lock(&panel->panel_lock);
}

static inline void dsi_panel_release_panel_lock(struct dsi_panel *panel)
{
	mutex_unlock(&panel->panel_lock);
}

struct dsi_panel *dsi_panel_get(struct device *parent,
				struct device_node *of_node,
				int topology_override,
				enum dsi_panel_type type);

int dsi_panel_trigger_esd_attack(struct dsi_panel *panel);

void dsi_panel_put(struct dsi_panel *panel);

int dsi_panel_drv_init(struct dsi_panel *panel, struct mipi_dsi_host *host);

int dsi_panel_drv_deinit(struct dsi_panel *panel);

void dsi_panel_debugfs_init(struct dsi_panel *panel, struct dentry *dir);

int dsi_panel_get_mode_count(struct dsi_panel *panel,
		struct device_node *of_node);

void dsi_panel_put_mode(struct dsi_display_mode *mode);

int dsi_panel_get_mode(struct dsi_panel *panel,
		       u32 index,
		       struct dsi_display_mode *mode,
		       int topology_override);

int dsi_panel_validate_mode(struct dsi_panel *panel,
			    struct dsi_display_mode *mode);

int dsi_panel_get_host_cfg_for_mode(struct dsi_panel *panel,
				    struct dsi_display_mode *mode,
				    struct dsi_host_config *config);

int dsi_panel_get_phy_props(struct dsi_panel *panel,
			    struct dsi_panel_phy_props *phy_props);
int dsi_panel_get_dfps_caps(struct dsi_panel *panel,
			    struct dsi_dfps_capabilities *dfps_caps);

int dsi_panel_pre_prepare(struct dsi_panel *panel);

int dsi_panel_set_lp1(struct dsi_panel *panel);

int dsi_panel_set_lp2(struct dsi_panel *panel);

int dsi_panel_set_nolp(struct dsi_panel *panel);

int dsi_panel_set_vr(struct dsi_panel *panel);

int dsi_panel_set_novr(struct dsi_panel *panel);

int dsi_panel_prepare(struct dsi_panel *panel);

int dsi_panel_enable(struct dsi_panel *panel);

int dsi_panel_post_enable(struct dsi_panel *panel);

int dsi_panel_pre_disable(struct dsi_panel *panel);

int dsi_panel_disable(struct dsi_panel *panel);

int dsi_panel_unprepare(struct dsi_panel *panel);

int dsi_panel_post_unprepare(struct dsi_panel *panel);

int dsi_panel_update_pps(struct dsi_panel *panel);

int dsi_panel_send_roi_dcs(struct dsi_panel *panel, int ctrl_idx,
		struct dsi_rect *roi);

int dsi_panel_switch(struct dsi_panel *panel);

int dsi_panel_post_switch(struct dsi_panel *panel);

void dsi_dsc_pclk_param_calc(struct msm_display_dsc_info *dsc, int intf_width);

struct dsi_panel *dsi_panel_ext_bridge_get(struct device *parent,
				struct device_node *of_node,
				int topology_override);

int dsi_panel_parse_esd_reg_read_configs(struct dsi_panel *panel,
				struct device_node *of_node);
int dsi_panel_parse_esd_irq_gpio_configs(struct dsi_panel *panel,
				struct device_node *of_node);

void dsi_panel_ext_bridge_put(struct dsi_panel *panel);

int dsi_panel_cmd_set_transfer(struct dsi_panel *panel,
			       struct dsi_panel_cmd_set *cmd);
int dsi_panel_parse_dt_cmd_set(struct device_node *of_node,
			       const char *cmd_str,
			       const char *cmd_state_str,
			       struct dsi_panel_cmd_set *cmd);
void dsi_panel_destroy_cmd_packets(struct dsi_panel_cmd_set *set);
void dsi_panel_debugfs_create_cmdset(struct dentry *parent,
				     const char *label,
				     struct dsi_panel *panel,
				     struct dsi_panel_cmd_set *set);

int dsi_backlight_early_dpms(struct dsi_backlight_config *bl, int power_state);
int dsi_backlight_late_dpms(struct dsi_backlight_config *bl, int power_state);

int dsi_backlight_get_dpms(struct dsi_backlight_config *bl);

int dsi_backlight_hbm_dimming_start(struct dsi_backlight_config *bl,
	u32 num_frames, struct dsi_panel_cmd_set *stop_cmd);
void dsi_backlight_hbm_dimming_stop(struct dsi_backlight_config *bl);

int dsi_panel_bl_register(struct dsi_panel *panel);
int dsi_panel_bl_unregister(struct dsi_panel *panel);
int dsi_panel_bl_parse_config(struct device *parent,
		struct dsi_backlight_config *bl, struct device_node *of_node);
void dsi_panel_bl_debugfs_init(struct dentry *parent, struct dsi_panel *panel);

int dsi_panel_update_vr_mode(struct dsi_panel *panel, bool enable);
bool dsi_panel_get_vr_mode(struct dsi_panel *panel);

int dsi_panel_get_sn(struct dsi_panel *panel);

/* Set/get high brightness mode */
int dsi_panel_update_hbm(struct dsi_panel *panel, bool enable);
bool dsi_panel_get_hbm(struct dsi_panel *panel);

#endif /* _DSI_PANEL_H_ */
