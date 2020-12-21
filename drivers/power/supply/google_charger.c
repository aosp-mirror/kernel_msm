/*
 * Copyright 2018 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/pmic-voter.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define CHG_TEMP_NB_LIMITS_MAX 10
#define CHG_VOLT_NB_LIMITS_MAX 5
#define CHG_DELAY_INIT_MS 250
#define CHG_DELAY_INIT_DETECT_MS 1000

#define DEFAULT_CHARGE_STOP_LEVEL 100
#define DEFAULT_CHARGE_START_LEVEL 0

#define MSC_CHG_VOTER			"msc_chg"

#define CHG_WORK_ERROR_RETRY_MS		1000
#define CHG_WORK_BD_TRIGGERED_MS	(5 * 60 * 1000)

#define MAX_BD_VOLTAGE	4300000
#define MIN_BD_VOLTAGE	3800000
#define MAX_BD_SOC	100
#define MIN_BD_SOC	0
#define MAX_BD_TEMP	500
#define MIN_BD_TEMP	0


struct chg_profile {
	u32 update_interval;
	u32 battery_capacity;
	int temp_nb_limits;
	s32 temp_limits[CHG_TEMP_NB_LIMITS_MAX];
	int volt_nb_limits;
	s32 volt_limits[CHG_VOLT_NB_LIMITS_MAX];
	/* Array of constant current limits */
	s32 *cccm_limits;
	u32 cv_hw_resolution;
	u32 cc_hw_resolution;
	u32 fv_uv_margin_dpct;
	u32 cv_range_accuracy;
	u32 cv_otv_margin;
	u32 cv_debounce_cnt;
	u32 cv_update_interval;
	u32 chg_cc_tolerance;
	u32 cv_tier_ov_cnt;
	u32 cv_tier_switch_cnt;
};

/* Taper WA */
struct taper_wa_struct {
	bool taper_wa_en;
	u32 v_taper[CHG_VOLT_NB_LIMITS_MAX];
	s32 i_taper[CHG_VOLT_NB_LIMITS_MAX];
	u32 soc_taper[CHG_VOLT_NB_LIMITS_MAX];
	int taper_cnt_target;

	int taper_wa_cnt;
	unsigned long last_taper_wa_cnt_time;
};

struct bd_data {
	u32 bd_trigger_voltage;	/* also recharge upper bound */
	u32 bd_trigger_temp;	/* single reading */
	u32 bd_trigger_time;	/* debounce window after trigger*/
	u32 bd_drainto_soc;
	u32 bd_recharge_voltage;
	u32 bd_recharge_soc;

	u32 bd_resume_abs_temp;	/* at any time after trigger */

	u32 bd_resume_soc;	/* any time after disconnect */
	u32 bd_resume_time;	/* debounce window for resume temp */
	u32 bd_resume_temp;	/* check resume_time after disconnect */

	long long temp_sum;
	time_t time_sum;

	int last_voltage;
	int last_temp;
	time_t last_update;

	time_t disconnect_time;

	u32 triggered;		/* (d) */
	u32 enabled;		/* (d) */
	u32 bd_temp_enable;	/* for UI setting interface */

	bool lowerbd_reached;
};

struct chg_drv {
	struct votable	*usb_icl_votable;
	struct votable	*dc_suspend_votable;

	struct device *device;
	struct power_supply *chg_psy;
	const char *chg_psy_name;
	struct power_supply *usb_psy;
	struct power_supply *wlc_psy;
	const char *wlc_psy_name;
	struct power_supply *bat_psy;
	const char *bat_psy_name;
	struct chg_profile chg_profile;
	struct notifier_block psy_nb;

	struct delayed_work init_work;
	struct delayed_work chg_work;
	struct wakeup_source chg_ws;

	bool stop_charging;
	int temp_idx;
	int vbatt_idx;
	int checked_cv_cnt;
	int checked_ov_cnt;
	int checked_tier_switch_cnt;

	int plugged;

	int chg_mode;
	int fv_uv;

	/* retail & battery defender */
	struct delayed_work bd_work;

	struct mutex bd_lock;
	struct bd_data bd_state;
	struct wakeup_source bd_ws;

	bool overheat;			/* battery reports overheat */
	bool freeze_soc;		/* battery soc freeze & thaw */

	int disable_charging;
	int disable_pwrsrc;
	bool lowerdb_reached;
	int charge_stop_level;
	int charge_start_level;
	unsigned long last_cnt_time;
	bool is_full;

	struct taper_wa_struct taper;
};

/* Used as left operand also */
#define CCCM_LIMITS(profile, ti, vi) \
	profile->cccm_limits[(ti * profile->volt_nb_limits) + vi]

#define get_boot_sec() div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC)

static int psy_changed(struct notifier_block *nb,
		       unsigned long action, void *data)
{
	struct power_supply *psy = data;
	struct chg_drv *chg_drv = container_of(nb, struct chg_drv, psy_nb);

	pr_debug("name=%s evt=%lu\n", psy->desc->name, action);

	if ((action != PSY_EVENT_PROP_CHANGED) ||
	    (psy == NULL) || (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (action == PSY_EVENT_PROP_CHANGED &&
	    (!strcmp(psy->desc->name, chg_drv->chg_psy_name) ||
	     !strcmp(psy->desc->name, chg_drv->bat_psy_name) ||
	     !strcmp(psy->desc->name, "usb") ||
	     (chg_drv->wlc_psy_name &&
	      !strcmp(psy->desc->name, chg_drv->wlc_psy_name)))) {
		cancel_delayed_work(&chg_drv->chg_work);
		schedule_delayed_work(&chg_drv->chg_work, 0);
	}
	return NOTIFY_OK;
}

static char *psy_chgt_str[] = {
	"Unknown", "N/A", "Trickle", "Fast", "Taper"
};

static char *psy_usb_type_str[] = {
	"Unknown", "Battery", "UPS", "Mains", "USB", "USB_DCP",
	"USB_CDP", "USB_ACA", "USB_HVDCP", "USB_HVDCP_3", "USB_PD",
	"Wireless", "USB_FLOAT", "BMS", "Parallel", "Main", "Wipower",
	"TYPEC", "TYPEC_UFP", "TYPEC_DFP"
};

#define PSY_GET_PROP(psy, psp) psy_get_prop(psy, psp, #psp)
static inline int psy_get_prop(struct power_supply *psy,
			       enum power_supply_property psp, char *prop_name)
{
	union power_supply_propval val;
	int ret = 0;

	if (!psy)
		return -EINVAL;
	ret = power_supply_get_property(psy, psp, &val);
	if (ret < 0) {
		pr_err("failed to get %s from '%s', ret=%d\n",
		       prop_name, psy->desc->name, ret);
		return -EINVAL;
	}
	pr_debug("get %s for '%s' => %d\n",
		 prop_name, psy->desc->name, val.intval);
	return val.intval;
}

#define PSY_SET_PROP(psy, psp, val) psy_set_prop(psy, psp, val, #psp)
static inline int psy_set_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       int intval, char *prop_name)
{
	union power_supply_propval val;
	int ret = 0;

	if (!psy)
		return -EINVAL;
	val.intval = intval;
	pr_debug("set %s for '%s' to %d\n", prop_name, psy->desc->name, intval);
	ret = power_supply_set_property(psy, psp, &val);
	if (ret < 0) {
		pr_err("failed to set %s for '%s', ret=%d\n",
		       prop_name, psy->desc->name, ret);
		return -EINVAL;
	}
	return 0;
}

static inline void reset_chg_drv_state(struct chg_drv *chg_drv)
{
	chg_drv->temp_idx = -1;
	chg_drv->vbatt_idx = -1;
	chg_drv->fv_uv = -1;
	chg_drv->checked_cv_cnt = 0;
	chg_drv->checked_ov_cnt = 0;
	chg_drv->checked_tier_switch_cnt = 0;
	chg_drv->disable_charging = -1;
	chg_drv->disable_pwrsrc = -1;
	chg_drv->lowerdb_reached = true;
	chg_drv->stop_charging = true;
	chg_drv->last_cnt_time = 0;
	chg_drv->taper.taper_wa_cnt = 0;
	chg_drv->taper.last_taper_wa_cnt_time = 0;
	PSY_SET_PROP(chg_drv->chg_psy,
		     POWER_SUPPLY_PROP_TAPER_CONTROL,
		     POWER_SUPPLY_TAPER_CONTROL_OFF);
	PSY_SET_PROP(chg_drv->chg_psy, POWER_SUPPLY_PROP_CHARGE_DISABLE, 1);
}

static void pr_info_states(struct power_supply *chg_psy,
			   struct power_supply *usb_psy,
			   struct power_supply *wlc_psy,
			   int temp, int ibatt, int vbatt, int vchrg,
			   int chg_type, int fv_uv,
			   int soc, int usb_present, int wlc_online)
{
	int usb_type = PSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_REAL_TYPE);

	pr_info("l=%d vb=%d vc=%d c=%d fv=%d t=%d s=%s usb=%d wlc=%d\n",
		soc, vbatt / 1000, vchrg / 1000, ibatt / 1000,
		fv_uv, temp, psy_chgt_str[chg_type],
		usb_present, wlc_online);

	if (usb_present)
		pr_info("usbchg=%s usbv=%d usbc=%d usbMv=%d usbMc=%d\n",
			psy_usb_type_str[usb_type],
			PSY_GET_PROP(usb_psy,
				     POWER_SUPPLY_PROP_VOLTAGE_NOW) / 1000,
			PSY_GET_PROP(usb_psy,
				     POWER_SUPPLY_PROP_INPUT_CURRENT_NOW)/1000,
			PSY_GET_PROP(usb_psy,
				     POWER_SUPPLY_PROP_VOLTAGE_MAX) / 1000,
			PSY_GET_PROP(usb_psy,
				     POWER_SUPPLY_PROP_CURRENT_MAX) / 1000);

	if (wlc_online)
		pr_info("wlcv=%d wlcc=%d wlcMv=%d wlcMc=%d wlct=%d\n",
			PSY_GET_PROP(wlc_psy,
				     POWER_SUPPLY_PROP_VOLTAGE_NOW) / 1000,
			PSY_GET_PROP(wlc_psy,
				     POWER_SUPPLY_PROP_CURRENT_NOW) / 1000,
			PSY_GET_PROP(wlc_psy,
				     POWER_SUPPLY_PROP_VOLTAGE_MAX) / 1000,
			PSY_GET_PROP(wlc_psy,
				     POWER_SUPPLY_PROP_CURRENT_MAX) / 1000,
			PSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_TEMP));
}

/* returns 1 if charging should be disabled given the current battery capacity
 * given in percent, return 0 if charging should happen
 */
static int is_charging_disabled(struct chg_drv *chg_drv, int capacity)
{
	int disable_charging = 0;
	int upperbd = chg_drv->charge_stop_level;
	int lowerbd = chg_drv->charge_start_level;

	if ((upperbd == DEFAULT_CHARGE_STOP_LEVEL) &&
	    (lowerbd == DEFAULT_CHARGE_START_LEVEL))
		return 0;

	if ((upperbd > lowerbd) &&
	    (upperbd <= DEFAULT_CHARGE_STOP_LEVEL) &&
	    (lowerbd >= DEFAULT_CHARGE_START_LEVEL)) {
		if (chg_drv->lowerdb_reached && upperbd <= capacity) {
			pr_info("%s: lowerbd=%d, upperbd=%d, capacity=%d, lowerdb_reached=1->0, charging off\n",
				__func__, lowerbd, upperbd, capacity);
			disable_charging = 1;
			chg_drv->lowerdb_reached = false;
		} else if (!chg_drv->lowerdb_reached && lowerbd < capacity) {
			pr_info("%s: lowerbd=%d, upperbd=%d, capacity=%d, charging off\n",
				__func__, lowerbd, upperbd, capacity);
			disable_charging = 1;
		} else if (!chg_drv->lowerdb_reached && capacity <= lowerbd) {
			pr_info("%s: lowerbd=%d, upperbd=%d, capacity=%d, lowerdb_reached=0->1, charging on\n",
				__func__, lowerbd, upperbd, capacity);
			chg_drv->lowerdb_reached = true;
		} else {
			pr_info("%s: lowerbd=%d, upperbd=%d, capacity=%d, charging on\n",
				__func__, lowerbd, upperbd, capacity);
		}
	}

	return disable_charging;
}


/* 1. charge profile idx based on the battery temperature */
static int msc_temp_idx(struct chg_profile *profile, int temp)
{
	int temp_idx = 0;

	while (temp_idx < profile->temp_nb_limits - 1 &&
	       temp >= profile->temp_limits[temp_idx + 1])
		temp_idx++;

	return temp_idx;
}

/* 2. compute the step index given the battery voltage
 * When selecting an index need to make sure that headroom for the tier voltage
 * will allow to send to the battery _at least_ next tier max FCC current and
 * well over charge termination current.
 */
static int msc_voltage_idx(struct chg_profile *profile, int vbatt)
{
	int vbatt_idx = 0;

	while (vbatt_idx < profile->volt_nb_limits - 1 &&
	       vbatt > profile->volt_limits[vbatt_idx])
		vbatt_idx++;

	/* assumes that 3 times the hardware resolution is ok */
	if (vbatt_idx != profile->volt_nb_limits - 1) {
		const int vt = profile->volt_limits[vbatt_idx];
		const int headr = profile->cv_hw_resolution * 3;

		if ((vt - vbatt) < headr)
			vbatt_idx += 1;
	}

	return vbatt_idx;
}

/* Cap to fv_uv_margin_pct of VTIER if needed */
static int msc_round_fv_uv(struct chg_profile *profile, int vtier, int fv_uv)
{
	int result;
	const unsigned int fv_uv_max = (vtier / 1000)
					* profile->fv_uv_margin_dpct;

	if (fv_uv_max != 0 && fv_uv > fv_uv_max)
		fv_uv = fv_uv_max;

	result = fv_uv - (fv_uv % profile->cv_hw_resolution);

	if (fv_uv_max != 0)
		pr_info("MSC_ROUND: fv_uv=%d vtier=%d fv_uv_max=%d -> %d\n",
			fv_uv, vtier, fv_uv_max, result);

	return result;
}

/* TODO: now created in qcom code, create in chg_create_votables() */
static int chg_find_votables(struct chg_drv *chg_drv)
{
	if (!chg_drv->usb_icl_votable)
		chg_drv->usb_icl_votable = find_votable("USB_ICL");
	if (!chg_drv->dc_suspend_votable)
		chg_drv->dc_suspend_votable = find_votable("DC_SUSPEND");

	return (!chg_drv->usb_icl_votable || !chg_drv->dc_suspend_votable)
		? -EINVAL : 0;
}

/* input suspend votes 0 ICL and call suspend on DC_ICL */
static int chg_vote_input_suspend(struct chg_drv *chg_drv, char *voter,
				  bool suspend)
{
	int rc;

	if (chg_find_votables(chg_drv) < 0)
		return -EINVAL;

	rc = vote(chg_drv->usb_icl_votable, voter, suspend, 0);
	if (rc < 0) {
		dev_err(chg_drv->device, "Couldn't vote to %s USB rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg_drv->dc_suspend_votable, voter, suspend, 0);
	if (rc < 0) {
		dev_err(chg_drv->device, "Couldn't vote to %s DC rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}

	return 0;
}

/* check for retail, battery defender */
static void chg_update_charging_state(struct chg_drv *chg_drv,
				      int disable_charging,
				      int disable_pwrsrc)
{

	/* disable charging is set in retail mode */
	if (disable_charging != chg_drv->disable_charging) {
		pr_info("MSC_CHG disable_charging %d -> %d",
			chg_drv->disable_charging, disable_charging);
		PSY_SET_PROP(chg_drv->chg_psy, POWER_SUPPLY_PROP_CHARGE_DISABLE,
			     disable_charging);
	}
	chg_drv->disable_charging = disable_charging;

	/* when disable_pwrsrc is set, disable_charging is set also */
	if (disable_pwrsrc != chg_drv->disable_pwrsrc) {
		pr_info("MSC_CHG disable_pwrsrc %d -> %d",
			chg_drv->disable_pwrsrc, disable_pwrsrc);

		/* applied right away */
		chg_vote_input_suspend(chg_drv, MSC_CHG_VOTER, disable_pwrsrc);
		/* take a wakelock */
		if (disable_pwrsrc)
			__pm_stay_awake(&chg_drv->bd_ws);
		else
			__pm_relax(&chg_drv->bd_ws);

	}
	chg_drv->disable_pwrsrc = disable_pwrsrc;
}

/* called on init, and to reset the trigger */
static void bd_reset(struct bd_data *bd_state)
{
	bool can_resume = bd_state->bd_resume_abs_temp ||
			  bd_state->bd_resume_time ||
			  (bd_state->bd_resume_time == 0 &&
			  bd_state->bd_resume_soc == 0);

	bd_state->time_sum = 0;
	bd_state->temp_sum = 0;
	bd_state->last_update = 0;
	bd_state->last_voltage = 0;
	bd_state->last_temp = 0;
	bd_state->triggered = 0;
	/* also disabled when externally triggered, resume_temp is optional */
	bd_state->enabled = ((bd_state->bd_trigger_voltage &&
			      bd_state->bd_recharge_voltage) ||
			      (bd_state->bd_drainto_soc &&
			      bd_state->bd_recharge_soc)) &&
			    bd_state->bd_trigger_time &&
			    bd_state->bd_trigger_temp &&
			    bd_state->bd_temp_enable &&
			    can_resume;
}

static void bd_init(struct bd_data *bd_state, struct device *dev)
{
	int ret;

	ret = of_property_read_u32(dev->of_node, "google,bd-trigger-voltage",
				   &bd_state->bd_trigger_voltage);
	if (ret < 0)
		bd_state->bd_trigger_voltage = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-drainto-soc",
				   &bd_state->bd_drainto_soc);
	if (ret < 0)
		bd_state->bd_drainto_soc = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-trigger-temp",
				   &bd_state->bd_trigger_temp);
	if (ret < 0)
		bd_state->bd_trigger_temp = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-trigger-time",
				   &bd_state->bd_trigger_time);
	if (ret < 0)
		bd_state->bd_trigger_time = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-recharge-voltage",
				   &bd_state->bd_recharge_voltage);
	if (ret < 0)
		bd_state->bd_recharge_voltage = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-recharge-soc",
				   &bd_state->bd_recharge_soc);
	if (ret < 0)
		bd_state->bd_recharge_soc = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-resume-abs-temp",
				   &bd_state->bd_resume_abs_temp);
	if (ret < 0)
		bd_state->bd_resume_abs_temp = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-resume-soc",
				   &bd_state->bd_resume_soc);
	if (ret < 0)
		bd_state->bd_resume_soc = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-resume-temp",
				   &bd_state->bd_resume_temp);
	if (ret < 0)
		bd_state->bd_resume_temp = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-resume-time",
				   &bd_state->bd_resume_time);
	if (ret < 0)
		bd_state->bd_resume_time = 0;


	/* also call to resume charging */
	bd_reset(bd_state);
	if (!bd_state->enabled)
		dev_warn(dev, "TEMP-DEFEND not enabled\n", ret);

	pr_info("MSC_BD: trig volt=%d,%d temp=%d,time=%d drainto=%d,%d resume=%d,%d %d,%d\n",
		bd_state->bd_trigger_voltage, bd_state->bd_recharge_voltage,
		bd_state->bd_trigger_temp, bd_state->bd_trigger_time,
		bd_state->bd_drainto_soc, bd_state->bd_recharge_soc,
		bd_state->bd_resume_abs_temp, bd_state->bd_resume_soc,
		bd_state->bd_resume_temp, bd_state->bd_resume_time);
}

/* */
static int bd_update_stats(struct bd_data *bd_state,
			   struct power_supply *bat_psy)
{
	const bool triggered = bd_state->triggered;
	const time_t now = get_boot_sec();
	int vbatt, temp;
	long long temp_avg;

	if (!bd_state->enabled)
		return 0;

	vbatt = PSY_GET_PROP(bat_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (vbatt == -EINVAL)
		return 0;

	/* not over vbat and !triggered, nothing to see here */
	if (vbatt < bd_state->bd_trigger_voltage && !triggered)
		return 0;

	/* it needs to keep averaging after trigger */
	temp = PSY_GET_PROP(bat_psy, POWER_SUPPLY_PROP_TEMP);
	if (temp == -EINVAL)
		return 0;

	/* it needs to keep averaging if triggered */
	if (bd_state->last_update == 0)
		bd_state->last_update = now;

	if (temp >= bd_state->bd_trigger_temp) {
		bd_state->time_sum += now - bd_state->last_update;
		bd_state->temp_sum += temp * (now - bd_state->last_update);
	}
	bd_state->last_voltage = vbatt;
	bd_state->last_update = now;
	bd_state->last_temp = temp;

	/* wait until we have at least bd_trigger_time */
	if (bd_state->time_sum < bd_state->bd_trigger_time)
		return 0;

	/* exit and entry criteria */
	temp_avg = bd_state->temp_sum / bd_state->time_sum;
	if (triggered && temp <= bd_state->bd_resume_abs_temp) {
		pr_info("MSC_BD: resume time_sum=%lld, temp_sum=%lld, temp_avg=%d\n",
			bd_state->time_sum, bd_state->temp_sum, temp_avg);
		bd_reset(bd_state);
	} else if (temp_avg >= bd_state->bd_trigger_temp) {
		pr_info("MSC_BD: trigger time_sum=%lld, temp_sum=%lld, temp_avg=%d\n",
			bd_state->time_sum, bd_state->temp_sum, temp_avg);
		bd_state->triggered = 1;
	}

	return 0;
}

/* @return true if BD needs to be triggered */
static int bd_recharge_logic(struct bd_data *bd_state, int val)
{
	int lowerbd, upperbd;
	int disable_charging = 0;

	if (!bd_state->triggered)
		return 0;

	if (bd_state->bd_drainto_soc && bd_state->bd_recharge_soc) {
		lowerbd = bd_state->bd_recharge_soc;
		upperbd = bd_state->bd_drainto_soc;
	} else if (bd_state->bd_recharge_voltage &&
		   bd_state->bd_trigger_voltage) {
		lowerbd = bd_state->bd_recharge_voltage;
		upperbd = bd_state->bd_trigger_voltage;
	} else {
		return 0;
	}

	/* recharge logic between bd_recharge_voltage and bd_trigger_voltage */
	if (bd_state->lowerbd_reached && val >= upperbd) {
		pr_info("MSC_BD lowerbd=%d, upperbd=%d, val=%d, lowerbd_reached=1->0, charging off\n",
			lowerbd, upperbd, val);
		bd_state->lowerbd_reached = false;
		disable_charging = 1;
	} else if (!bd_state->lowerbd_reached && val > lowerbd) {
		pr_info("MSC_BD lowerbd=%d, upperbd=%d, val=%d, charging off\n",
			lowerbd, upperbd, val);
		disable_charging = 1;
	} else if (!bd_state->lowerbd_reached && val <= lowerbd) {
		pr_info("MSC_BD lowerbd=%d, upperbd=%d, val=%d, lowerbd_reached=0->1, charging on\n",
			lowerbd, upperbd, val);
		bd_state->lowerbd_reached = true;
	} else {
		pr_info("MSC_BD lowerbd=%d, upperbd=%d, val=%d, charging on\n",
			lowerbd, upperbd, val);
	}

	return disable_charging;
}


/* ingnore the failure to set CAPACITY, it might not be implemented */
static int bd_batt_set_soc(struct chg_drv *chg_drv , int soc)
{
	const bool freeze = soc >= 0;
	int rc = 0;

	pr_debug("MSC_BD soc=%d %s (%d)\n", soc,
		 soc == -1 ? "THAW" : "FREEZE", rc);

	chg_drv->freeze_soc = freeze;
	return 0;
}

static int bd_batt_set_overheat(struct chg_drv *chg_drv , bool hot)
{
	const int health = hot ? POWER_SUPPLY_HEALTH_OVERHEAT :
				 POWER_SUPPLY_HEALTH_UNKNOWN;
	int ret;

	ret = PSY_SET_PROP(chg_drv->bat_psy, POWER_SUPPLY_PROP_HEALTH, health);
	if (ret == 0)
		chg_drv->overheat = hot;

	pr_info("MSC_BD OVERHEAT hot=%d (%d)\n", hot, ret);

	return ret;
}

static int bd_batt_set_state(struct chg_drv *chg_drv, bool hot, int soc)
{
	const bool freeze = soc != -1;
	int ret = 0;

	/* OVERHEAT is used to rebuild the curve */
	if (freeze != chg_drv->freeze_soc)
		ret = bd_batt_set_soc(chg_drv, soc);
	if (ret == 0 && chg_drv->overheat != hot)
		ret = bd_batt_set_overheat(chg_drv, hot);
	return ret;
}

static int chg_work_read_soc(struct power_supply *bat_psy, int *soc)
{
	union power_supply_propval val;
	int ret = 0;

	ret = power_supply_get_property(bat_psy, POWER_SUPPLY_PROP_CAPACITY,
					&val);
	if (ret == 0)
		*soc = val.intval;

	return ret;
}

/*
 * Run in background after disconnect to reset the trigger.
 * The UI% is not frozen here: only battery health state remain
 * POWER_SUPPLY_HEALTH_OVERHEAT until the condition clears.
 */

static void bd_work(struct work_struct *work)
{
	struct chg_drv *chg_drv =
		container_of(work, struct chg_drv, bd_work.work);
	struct bd_data *bd_state = &chg_drv->bd_state;
	int interval_ms = CHG_WORK_BD_TRIGGERED_MS;
	const time_t now = get_boot_sec();
	const long long delta_time = now - bd_state->disconnect_time;
	int ret, soc = 0;

	__pm_stay_awake(&chg_drv->bd_ws);

	mutex_lock(&chg_drv->bd_lock);

	pr_debug("MSC_BD_WORK: triggered=%d dsc_time=%ld delta=%lld\n",
		bd_state->triggered, bd_state->disconnect_time, delta_time);

	/* stop running if the cable in. */
	if (!chg_drv->stop_charging)
		goto bd_done;

	/* stop running if battery defender or retail mode are triggered */
	if (!bd_state->triggered || !bd_state->disconnect_time)
		goto bd_done;

	if (bd_state->bd_resume_soc &&
	    chg_work_read_soc(chg_drv->bat_psy, &soc) == 0 &&
	    soc < bd_state->bd_resume_soc) {
		pr_info("MSC_BD_WORK: done soc=%d limit=%d\n",
			soc, bd_state->bd_resume_soc);
		bd_reset(bd_state);
		goto bd_rerun;
	}

	/* reset based on average or instant temp */
	ret = bd_update_stats(bd_state, chg_drv->bat_psy);
	if (ret < 0) {
		pr_err("MSC_BD_WORK: update stats: %d\n", ret);
		interval_ms = 1000;
		goto bd_rerun;
	}

	/* reset on time since disconnect & optional temperature reading */
	if (bd_state->bd_resume_time && delta_time > bd_state->bd_resume_time) {
		const int temp = bd_state->last_temp; /* or use avg */
		int triggered;

		/* single reading < of resume temp, or total average temp */
		triggered = temp > bd_state->bd_resume_temp;
		if (!triggered) {
			pr_info("MSC_BD_WORK: done time=%lld limit=%d, temp=%d limit=%d\n",
				delta_time, bd_state->bd_resume_time,
				temp, bd_state->bd_resume_temp);
			bd_state->triggered = triggered;
			goto bd_rerun;
		}
	}

	pr_info("MSC_BD_WORK: triggered=%d time=%lld < %lld soc=%d > %d temp=%lld > %d\n",
		bd_state->triggered,
		delta_time, bd_state->bd_resume_time,
		soc, bd_state->bd_resume_soc,
		bd_state->last_temp,
		bd_state->bd_resume_temp);

bd_rerun:
	if (!bd_state->triggered) {
		/* disable the overheat flag, race with DWELL-DEFEND */
		bd_batt_set_overheat(chg_drv, false);
	} else {
		schedule_delayed_work(&chg_drv->bd_work,
				      msecs_to_jiffies(interval_ms));
	}

bd_done:
	mutex_unlock(&chg_drv->bd_lock);
	__pm_relax(&chg_drv->bd_ws);
}

static void chg_work(struct work_struct *work)
{
	struct chg_drv *chg_drv =
	    container_of(work, struct chg_drv, chg_work.work);
	struct chg_profile *profile = &chg_drv->chg_profile;
	union power_supply_propval val;
	struct power_supply *chg_psy = chg_drv->chg_psy;
	struct power_supply *usb_psy = chg_drv->usb_psy;
	struct power_supply *wlc_psy = chg_drv->wlc_psy;
	struct power_supply *bat_psy = chg_drv->bat_psy;
	int temp, ibatt, vbatt, vchrg, soc, chg_type;
	int usb_present, wlc_online = 0;
	int vbatt_idx = chg_drv->vbatt_idx, fv_uv = chg_drv->fv_uv, temp_idx;
	int update_interval = chg_drv->chg_profile.update_interval;
	int plugged, batt_status;
	bool rerun_work = false;
	int disable_charging;
	int disable_pwrsrc = 0;
	int rc;
	static bool cable_in;

	__pm_stay_awake(&chg_drv->chg_ws);
	pr_debug("battery charging work item\n");

	usb_present = PSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_PRESENT);
	if (wlc_psy)
		wlc_online = PSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_ONLINE);

	plugged = usb_present || wlc_online;
	if (chg_drv->plugged != plugged) {
		int ret = 0;

		/* only after boot */
		if (chg_drv->plugged != -1) {
			ret = PSY_SET_PROP(bat_psy,
					   POWER_SUPPLY_PROP_BATT_CE_CTRL,
					   plugged);
		}

		if (ret == 0)
			chg_drv->plugged = plugged;
	}

	/* If no power source, disable charging and exit */
	if (!usb_present && !wlc_online) {
		const bool stop_charging = chg_drv->stop_charging != 1;
		struct bd_data *bd_state = &chg_drv->bd_state;

		if (stop_charging) {
			pr_info("no power source detected, disabling charging\n");
			if (!bd_state->triggered)
				bd_reset(bd_state);
			reset_chg_drv_state(chg_drv);
			chg_drv->stop_charging = 1;
			bd_batt_set_state(chg_drv, false, -1);
		}

		mutex_lock(&chg_drv->bd_lock);
		if (bd_state->triggered) {
			const bool bd_ena = bd_state->bd_resume_soc ||
					    bd_state->bd_resume_time;
			/*
			 * clear overheat and thaw soc on disconnect.
			 * Don not clear the defender state, will
			 * re-evaluate on next connect.
			 */
			if (!bd_state->disconnect_time) {
				rc = bd_batt_set_state(chg_drv, false, -1);
				if (rc < 0) {
					pr_err("MSC_BD set_batt_state (%d)\n",
					rc);
					mutex_unlock(&chg_drv->bd_lock);
					goto error_rerun;
				}
				bd_state->disconnect_time = get_boot_sec();
			}
			if (bd_ena)
				mod_delayed_work(system_wq,
						 &chg_drv->bd_work, 0);
		}
		mutex_unlock(&chg_drv->bd_lock);

		/* allow sleep (if disconnected) while draining */
		if (chg_drv->disable_pwrsrc)
			__pm_relax(&chg_drv->bd_ws);

		goto exit_chg_work;
	}

	/* debug option  */
	if (chg_drv->chg_mode)
		bat_psy = chg_psy;

	temp = PSY_GET_PROP(bat_psy, POWER_SUPPLY_PROP_TEMP);
	ibatt = PSY_GET_PROP(bat_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	vbatt = PSY_GET_PROP(bat_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	soc = PSY_GET_PROP(bat_psy, POWER_SUPPLY_PROP_CAPACITY);
	vchrg = PSY_GET_PROP(chg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	chg_type = PSY_GET_PROP(chg_psy, POWER_SUPPLY_PROP_CHARGE_TYPE);

	if (temp == -EINVAL || ibatt == -EINVAL || vbatt == -EINVAL ||
	    usb_present == -EINVAL || wlc_online == -EINVAL)
		goto error_rerun;

	pr_info_states(chg_psy, usb_psy, wlc_psy,
		       temp, ibatt, vbatt, vchrg,
		       chg_type, chg_drv->fv_uv,
		       soc, usb_present, wlc_online);

	if (temp < profile->temp_limits[0] ||
	    temp > profile->temp_limits[profile->temp_nb_limits - 1]) {
		if (!chg_drv->stop_charging) {
			pr_info("batt. temp. off limits, disabling charging\n");
			reset_chg_drv_state(chg_drv);
		}
		/* status will be discharging when disabled but we want to keep
		 * monitoring temperature to re-enable charging
		 */
		rerun_work = true;
		goto handle_rerun;
	} else if (chg_drv->stop_charging) {
		pr_info("batt. temp. ok, enabling charging\n");
		PSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_CHARGE_DISABLE, 0);
		chg_drv->stop_charging = false;
		cable_in = true;
	}

	/* BD and retail: disable_pwrsrc = fast drain to the stop level */
	mutex_lock(&chg_drv->bd_lock);
	disable_charging = is_charging_disabled(chg_drv, soc);
	if (disable_charging && soc > chg_drv->charge_stop_level)
		disable_pwrsrc = 1;
	if (disable_charging) {
		/* force TEMP-DEFEND off, stop bd_work() */
		if (chg_drv->bd_state.triggered) {
			bd_reset(&chg_drv->bd_state);
			bd_batt_set_state(chg_drv, false, -1);
		}
		chg_drv->bd_state.enabled = false;

		/*
		 * This mode can be enabled from DWELL-DEFEND when in "idle",
		 * while TEMP-DEFEND is triggered and from Retail Mode.
		 *
		 * Retail should probably not flag OVERHEAT as this might
		 * have unwanted interactions with the demo application.
		 */
		if (!chg_drv->disable_charging) {
			rc = bd_batt_set_state(chg_drv, false, -1);
			if (rc < 0)
				pr_err("MSC_BD batt reset (%d)\n", rc);
		}
	} else if (chg_drv->bd_state.enabled) {
		struct bd_data *bd_state = &chg_drv->bd_state;
		const bool was_triggered = bd_state->triggered;

		/* bd_work() is not running here */
		rc = bd_update_stats(bd_state, bat_psy);
		if (rc < 0)
			pr_debug("MSC_DB BD update stats: %d\n", rc);

		/* thaw SOC, clear overheat */
		if (!bd_state->triggered && was_triggered) {
			rc = bd_batt_set_state(chg_drv, false, -1);
			if (rc < 0)
				pr_err("MSC_BD cannot reset state (%d)\n", rc);
		} else if (bd_state->triggered) {
			const int lock_soc = soc; /* or set to 100 */

			/* recharge logic */
			if (bd_state->bd_drainto_soc) {
				disable_charging =
				    bd_recharge_logic(bd_state, soc);
				if (disable_charging)
					disable_pwrsrc =
						soc > bd_state->bd_drainto_soc;
			} else {
				disable_charging = bd_recharge_logic(bd_state,
							bd_state->last_voltage);
				if (disable_charging)
					disable_pwrsrc =
						bd_state->last_voltage >
						bd_state->bd_trigger_voltage;
			}

			/* overheat and freeze, on trigger and reconnect */
			if (!was_triggered || chg_drv->stop_charging
			    || cable_in) {
				cable_in = false;
				rc = bd_batt_set_state(chg_drv, true, lock_soc);
				if (rc < 0)
					pr_err("MSC_BD cannot set overheat (%d)\n",
						rc);
			}
		}
	} else if (chg_drv->disable_charging) {
		/* re-enable TEMP-DEFEND */
		bd_reset(&chg_drv->bd_state);
		/* DWELL-DEFEND handles OVERHEAT status */
	}

	/* state in chg_drv->disable_charging, chg_drv->disable_pwrsrc */
	chg_update_charging_state(chg_drv, disable_charging, disable_pwrsrc);
	mutex_unlock(&chg_drv->bd_lock);

	/* no need to reschedule, battery psy event will reschedule work item */
	if (chg_drv->disable_charging || chg_drv->disable_pwrsrc) {
		rerun_work = false;
		goto exit_chg_work;
	}

	/* Multi Step Chargings with compensation of IRDROP
	 * vbatt_idx = chg_drv->vbatt_idx, fv_uv = chg_drv->fv_uv
	 */
	temp_idx = msc_temp_idx(profile, temp);
	if (temp_idx != chg_drv->temp_idx || chg_drv->fv_uv == -1
		|| chg_drv->vbatt_idx == -1) {
		/* seed voltage only when really needed */
		if (chg_drv->vbatt_idx == -1)
			vbatt_idx = msc_voltage_idx(profile, vbatt);

		pr_info("MSC_SEED temp=%d vbatt=%d temp_idx:%d->%d, vbatt_idx:%d->%d\n",
			temp, vbatt, chg_drv->temp_idx, temp_idx,
			chg_drv->vbatt_idx, vbatt_idx);

		/* Debounce tier switch only when not already switching */
		if (chg_drv->checked_tier_switch_cnt == 0)
			chg_drv->checked_cv_cnt = profile->cv_debounce_cnt;
	} else if (ibatt > 0) {
		/* Track battery voltage if discharging is due to system load,
		 * low ILIM or lack of headroom; stop charging work and reset
		 * chg_drv state() when discharging is due to disconnect.
		 * NOTE: POWER_SUPPLY_PROP_STATUS return *_DISCHARGING only on
		 * disconnect.
		 * NOTE: same vbat_idx will not change fv_uv
		 */
		vbatt_idx = msc_voltage_idx(profile, vbatt);
		update_interval = profile->cv_update_interval;

		pr_info("MSC_DSG vbatt_idx:%d->%d vbatt=%d ibatt=%d fv_uv=%d cv_cnt=%d ov_cnt=%d\n",
			chg_drv->vbatt_idx, vbatt_idx,
			vbatt, ibatt, fv_uv,
			chg_drv->checked_cv_cnt,
			chg_drv->checked_ov_cnt);
	} else if (chg_drv->vbatt_idx == profile->volt_nb_limits - 1) {
		/* will not adjust charger voltage only in the configured
		 * last tier.
		 * NOTE: might not be the "real" last tier since can I have
		 * tiers with max charge current == 0.
		 * NOTE: should I use a voltage limit instead?
		 */
		pr_info("MSC_LAST vbatt=%d ibatt=%d fv_uv=%d\n",
			vbatt, ibatt, fv_uv);
	} else {
		const int vtier = profile->volt_limits[vbatt_idx];
		const int utv_margin = profile->cv_range_accuracy;
		const int otv_margin = profile->cv_otv_margin;
		const int switch_cnt = profile->cv_tier_switch_cnt;
		const int cc_next_max = CCCM_LIMITS(profile, temp_idx,
						    vbatt_idx + 1);
		unsigned long cur_time, last_cnt_interval;
		bool skip_cnt = false;

		cur_time = jiffies_to_msecs(jiffies);
		last_cnt_interval = cur_time - chg_drv->last_cnt_time;
		skip_cnt = (last_cnt_interval <
			      profile->cv_update_interval) ? true : false;

		if ((vbatt - vtier) > otv_margin) {
		/* OVER: vbatt over vtier for more than margin (usually 0) */
			const int cc_max =
				CCCM_LIMITS(profile, temp_idx, vbatt_idx);

			/* pullback when over tier voltage, fast poll, penalty
			 * on TAPER_RAISE and no cv debounce (so will consider
			 * switching voltage tiers if the current is right).
			 * NOTE: lowering voltage might cause a small drop in
			 * current (we should remain  under next tier)
			 */
			fv_uv = msc_round_fv_uv(profile, vtier,
				fv_uv - profile->cv_hw_resolution);
			if (fv_uv < vtier)
				fv_uv = vtier;

			update_interval = profile->cv_update_interval;
			chg_drv->checked_ov_cnt = profile->cv_tier_ov_cnt;
			chg_drv->checked_cv_cnt = 0;

			if (chg_drv->checked_tier_switch_cnt > 0) {
			/* no pullback, next tier if already counting */
				vbatt_idx = chg_drv->vbatt_idx + 1;

				pr_info("MSC_VSWITCH vt=%d vb=%d ibatt=%d\n",
					vtier, vbatt, ibatt);
			} else if (-ibatt == cc_max) {
			/* pullback, double penalty if at full current */
				chg_drv->checked_ov_cnt *= 2;
				pr_info("MSC_VOVER vt=%d  vb=%d ibatt=%d fv_uv=%d->%d\n",
					vtier, vbatt, ibatt,
					chg_drv->fv_uv, fv_uv);
			} else {
				pr_info("MSC_PULLBACK vt=%d vb=%d ibatt=%d fv_uv=%d->%d\n",
					vtier, vbatt, ibatt,
					chg_drv->fv_uv, fv_uv);
			}

			/* NOTE: might get here after windup because algo will
			 * track the voltage drop caused from load as IRDROP.
			 * TODO: make sure that being current limited clear
			 * the taper condition.
			 */

		} else if (chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST) {
		/* FAST: usual compensation (vchrg is vqcom)
		 * NOTE: there is a race in reading from charger and data here
		 * might not be consistent (b/110318684)
		 * NOTE: could add PID loop for management of thermals
		 */
			if (vchrg > vbatt) {
				fv_uv = msc_round_fv_uv(profile, vtier,
					vtier + (vchrg - vbatt));
			} else {
				/* could keep it steady instead */
				fv_uv = vtier;
			}

			/* no tier switch during fast charge */
			if (chg_drv->checked_cv_cnt == 0)
				chg_drv->checked_cv_cnt = 1;

			pr_info("MSC_FAST vt=%d vb=%d fv_uv=%d->%d vchrg=%d cv_cnt=%d \n",
				vtier, vbatt, chg_drv->fv_uv, fv_uv,
				vchrg, chg_drv->checked_cv_cnt);

		} else if (chg_type != POWER_SUPPLY_CHARGE_TYPE_TAPER) {
		/* Not fast or taper: set checked_cv_cnt=0 to make sure we test
		 * for current and avoid early termination in case of lack of
		 * headroom (Vfloat ~= Vbatt)
		 * NOTE: this can cause early switch on low ilim
		 */
			update_interval = profile->cv_update_interval;
			chg_drv->checked_cv_cnt = 0;

			pr_info("MSC_TYPE vt=%d vb=%d fv_uv=%d chg_type=%d\n",
				vtier, vbatt, fv_uv, chg_type);

		} else if (chg_drv->checked_cv_cnt + chg_drv->checked_ov_cnt) {
		/* TAPER_COUNTDOWN: countdown to raise fv_uv and/or check
		 * for tier switch, will keep steady...
		 */
			pr_info("MSC_DLY vt=%d vb=%d fv_uv=%d margin=%d cv_cnt=%d, ov_cnt=%d, skip_cnt=%d\n",
				vtier, vbatt, fv_uv, profile->cv_range_accuracy,
				chg_drv->checked_cv_cnt,
				chg_drv->checked_ov_cnt, skip_cnt);

			update_interval = profile->cv_update_interval;
			if (!skip_cnt) {
				if (chg_drv->checked_cv_cnt)
					chg_drv->checked_cv_cnt -= 1;
				if (chg_drv->checked_ov_cnt)
					chg_drv->checked_ov_cnt -= 1;
				chg_drv->last_cnt_time = cur_time;
			}

		} else if ((vtier - vbatt) < utv_margin) {
		/* TAPER_STEADY: close enough to tier, don't need to adjust */
			update_interval = profile->cv_update_interval;

			pr_info("MSC_STEADY vt=%d vb=%d fv_uv=%d margin=%d\n",
				vtier, vbatt, fv_uv,
				profile->cv_range_accuracy);
		} else {
		/* TAPER_RAISE: under tier vlim, raise one click & debounce
		 * taper (see above handling of "close enough")
		 */
			fv_uv = msc_round_fv_uv(profile, vtier,
				fv_uv + profile->cv_hw_resolution);
			update_interval = profile->cv_update_interval;

			/* debounce next taper voltage adjustment */
			chg_drv->checked_cv_cnt = profile->cv_debounce_cnt;

			pr_info("MSC_RAISE vt=%d vb=%d fv_uv=%d->%d\n",
				vtier, vbatt, chg_drv->fv_uv, fv_uv);
		}

		if (chg_drv->checked_cv_cnt > 0) {
		/* debounce period on tier switch */
			pr_info("MSC_WAIT vt=%d vb=%d fv_uv=%d ibatt=%d cv_cnt=%d ov_cnt=%d\n",
				vtier, vbatt, fv_uv, ibatt,
				chg_drv->checked_cv_cnt,
				chg_drv->checked_ov_cnt);
		} else if (-ibatt > cc_next_max) {
		/* current over next tier, reset tier switch count */
			chg_drv->checked_tier_switch_cnt = 0;

			pr_info("MSC_RSTC vt=%d vb=%d fv_uv=%d ibatt=%d cc_next_max=%d t_cnt=%d\n",
				vtier, vbatt, fv_uv, ibatt, cc_next_max,
				chg_drv->checked_tier_switch_cnt);
		} else if (chg_drv->checked_tier_switch_cnt >= switch_cnt) {
		/* next tier, fv_uv detemined at MSC_SET */
			vbatt_idx = chg_drv->vbatt_idx + 1;

			pr_info("MSC_NEXT tier vb=%d ibatt=%d vbatt_idx=%d->%d\n",
				vbatt, ibatt, chg_drv->vbatt_idx, vbatt_idx);
		} else {
		/* current under next tier, increase tier switch count */
			if (!skip_cnt) {
				chg_drv->checked_tier_switch_cnt++;
				chg_drv->last_cnt_time = cur_time;
			}

			pr_info("MSC_NYET ibatt=%d cc_next_max=%d t_cnt=%d, skip_cnt=%d\n",
				ibatt, cc_next_max,
				chg_drv->checked_tier_switch_cnt, skip_cnt);
		}

		if (skip_cnt &&
		    (update_interval == profile->cv_update_interval)) {
			update_interval -= last_cnt_interval;
		}
	}

	/* TAPER WA */
	if (chg_drv->taper.taper_wa_en &&
	    (chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST) &&
	    (vbatt_idx != profile->volt_nb_limits - 1)) {
		struct taper_wa_struct *taper = &chg_drv->taper;
		int chg_status_fast;
		unsigned long curr_time, last_cnt_interval;
		bool skip_cnt = true;

		chg_status_fast =
		    PSY_GET_PROP(chg_psy,
				 POWER_SUPPLY_PROP_CHARGER_STATUS_FAST);

		curr_time = jiffies_to_msecs(jiffies);
		last_cnt_interval = curr_time - taper->last_taper_wa_cnt_time;
		skip_cnt = (update_interval != profile->update_interval) ||
			    (update_interval > last_cnt_interval);

		if (((vbatt / 1000) > taper->v_taper[vbatt_idx]) &&
		    ((-ibatt / 1000) < taper->i_taper[vbatt_idx]) &&
		    (soc > taper->soc_taper[vbatt_idx]) &&
		    chg_status_fast) {
			if (!skip_cnt) {
				taper->taper_wa_cnt++;
				taper->last_taper_wa_cnt_time = curr_time;
				pr_info("MSC_TAPER ibatt=%d, vbat=%d, soc=%d, taper_cnt=%d, chg_status_fast=%d\n",
					vbatt, ibatt, soc, taper->taper_wa_cnt,
					chg_status_fast);
			}
			if (taper->taper_wa_cnt > taper->taper_cnt_target) {
				vbatt_idx++;
				taper->taper_wa_cnt = 0;
				taper->last_taper_wa_cnt_time = 0;
				pr_info("MSC_TAPER next vbatt_idx=%d->%d\n",
					chg_drv->vbatt_idx, vbatt_idx);
			}
		} else {
			taper->taper_wa_cnt = 0;
			taper->last_taper_wa_cnt_time = 0;
		}
	} else {
		chg_drv->taper.taper_wa_cnt = 0;
		chg_drv->taper.last_taper_wa_cnt_time = 0;
	}

	/* update fv or cc will change in last tier... */
	if ((vbatt_idx != chg_drv->vbatt_idx) || (temp_idx != chg_drv->temp_idx)
		|| (fv_uv != chg_drv->fv_uv)) {
		const int cc_max = CCCM_LIMITS(profile, temp_idx, vbatt_idx);
		int rc;

		/* need a new fv_uv only on a new voltage tier */
		if (vbatt_idx != chg_drv->vbatt_idx) {
			fv_uv = profile->volt_limits[vbatt_idx];
			chg_drv->checked_tier_switch_cnt = 0;
			chg_drv->checked_ov_cnt = 0;
		}

		pr_info("MSC_SET cv_cnt=%d ov_cnt=%d temp_idx:%d->%d, vbatt_idx:%d->%d, fv=%d->%d, cc_max=%d\n",
			chg_drv->checked_cv_cnt, chg_drv->checked_ov_cnt,
			chg_drv->temp_idx, temp_idx, chg_drv->vbatt_idx,
			vbatt_idx, chg_drv->fv_uv, fv_uv, cc_max);

		/* taper control on last tier with nonzero charge current */
		if (vbatt_idx == (profile->volt_nb_limits - 1) ||
			CCCM_LIMITS(profile, temp_idx, vbatt_idx + 1) == 0) {
			PSY_SET_PROP(chg_drv->chg_psy,
				POWER_SUPPLY_PROP_TAPER_CONTROL,
				POWER_SUPPLY_TAPER_CONTROL_MODE_IMMEDIATE);
		}

		rc = PSY_SET_PROP(chg_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, cc_max);
		if (rc == 0)
			rc = PSY_SET_PROP(chg_psy,
					POWER_SUPPLY_PROP_VOLTAGE_MAX, fv_uv);

		if (rc != 0) {
			pr_err("MSC_SET: error rc=%d\n", rc);
			goto error_rerun;
		}

		chg_drv->vbatt_idx = vbatt_idx;
		chg_drv->temp_idx = temp_idx;
		chg_drv->fv_uv = fv_uv;
	}

	/* DISCHARGING only when not connected */
	batt_status = PSY_GET_PROP(chg_psy, POWER_SUPPLY_PROP_STATUS);

	/* reset charger if status full but soc < 100%, except recharge */
	if (batt_status == POWER_SUPPLY_STATUS_FULL) {
		if (soc == chg_drv->charge_stop_level) {
			chg_drv->is_full = true;
		} else if (!chg_drv->is_full) {
			pr_info("MSC_RESET: charge full in unexpected soc. reset chg\n");
			reset_chg_drv_state(chg_drv);
		}
	} else {
		chg_drv->is_full = false;
	}

	switch (batt_status) {
	case POWER_SUPPLY_STATUS_DISCHARGING:
		rerun_work = false;
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
	case POWER_SUPPLY_STATUS_FULL:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		rerun_work = true;
		break;
	case POWER_SUPPLY_STATUS_UNKNOWN:
		pr_err("chg_work charging status UNKNOWN\n");
		goto error_rerun;
	default:
		pr_err("chg_work invalid charging status %d\n", val.intval);
		goto error_rerun;
	}

handle_rerun:
	if (rerun_work) {
		pr_debug("rerun battery charging work in %d ms\n",
			 update_interval);
		schedule_delayed_work(&chg_drv->chg_work,
				      msecs_to_jiffies(update_interval));
	} else {
		pr_info("stop battery charging work: batt_status=%d\n",
			batt_status);
		reset_chg_drv_state(chg_drv);
	}
	goto exit_chg_work;

error_rerun:
	pr_err("error occurred, rerun battery charging work in %d ms\n",
	       CHG_WORK_ERROR_RETRY_MS);
	schedule_delayed_work(&chg_drv->chg_work,
			      msecs_to_jiffies(CHG_WORK_ERROR_RETRY_MS));

exit_chg_work:
	__pm_relax(&chg_drv->chg_ws);
}

static void dump_profile(struct chg_profile *profile)
{
	char buff[256];
	int ti, vi, count, len = sizeof(buff);

	pr_info("Profile constant charge limits:\n");
	count = 0;
	for (vi = 0; vi < profile->volt_nb_limits; vi++) {
		count += scnprintf(buff + count, len - count, "  %4d",
				   profile->volt_limits[vi] / 1000);
	}
	pr_info("|T \\ V%s\n", buff);

	for (ti = 0; ti < profile->temp_nb_limits - 1; ti++) {
		count = 0;
		count += scnprintf(buff + count, len - count, "|%2d:%2d",
				   profile->temp_limits[ti] / 10,
				   profile->temp_limits[ti + 1] / 10);
		for (vi = 0; vi < profile->volt_nb_limits; vi++) {
			count += scnprintf(buff + count, len - count, "  %4d",
					   CCCM_LIMITS(profile, ti, vi) / 1000);
		}
		pr_info("%s\n", buff);
	}
}

static int chg_init_chg_profile(struct chg_drv *chg_drv)
{
	struct device *dev = chg_drv->device;
	struct device_node *node = dev->of_node;
	struct chg_profile *profile = &chg_drv->chg_profile;
	struct taper_wa_struct *taper = &chg_drv->taper;
	u32 cccm_array_size, ccm;
	int ret = 0, vi, ti;
	int taper_tmp;

	ret = of_property_read_u32(node, "google,chg-update-interval",
				   &profile->update_interval);
	if (ret < 0) {
		pr_err("cannot read chg-update-interval, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,chg-battery-capacity",
				   &profile->battery_capacity);
	if (ret < 0) {
		pr_err("cannot read chg-battery-capacity, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,cv-hw-resolution",
				   &profile->cv_hw_resolution);
	if (ret < 0)
		profile->cv_hw_resolution = 25000;

	ret = of_property_read_u32(node, "google,cc-hw-resolution",
				   &profile->cc_hw_resolution);
	if (ret < 0)
		profile->cc_hw_resolution = 25000;

	/* IEEE1725, default to 0, 1030 for 3% of VTIER */
	ret = of_property_read_u32(node, "google,fv-uv-margin-dpct",
				   &profile->fv_uv_margin_dpct);
	if (ret < 0)
		profile->fv_uv_margin_dpct = 0;

	ret = of_property_read_u32(node, "google,cv-range-accuracy",
				   &profile->cv_range_accuracy);
	if (ret < 0)
		profile->cv_range_accuracy = profile->cv_hw_resolution / 2;

	/* allow being "a little" over tier voltage, experimental */
	ret = of_property_read_u32(node, "google,cv-otv-margin",
				   &profile->cv_otv_margin);
	if (ret < 0)
		profile->cv_otv_margin = 0;

	ret = of_property_read_u32(node, "google,cv-debounce-cnt",
				   &profile->cv_debounce_cnt);
	if (ret < 0)
		profile->cv_debounce_cnt = 3;

	ret = of_property_read_u32(node, "google,cv-update-interval",
				   &profile->cv_update_interval);
	if (ret < 0)
		profile->cv_update_interval = 2000;

	ret = of_property_read_u32(node, "google,cv-tier-ov-cnt",
				   &profile->cv_tier_ov_cnt);
	if (ret < 0)
		profile->cv_tier_ov_cnt = 10;

	ret = of_property_read_u32(node, "google,cv-tier-switch-cnt",
				   &profile->cv_tier_switch_cnt);
	if (ret < 0)
		profile->cv_tier_switch_cnt = 3;

	profile->temp_nb_limits =
	    of_property_count_elems_of_size(node, "google,chg-temp-limits",
					    sizeof(u32));
	if (profile->temp_nb_limits <= 0) {
		ret = profile->temp_nb_limits;
		pr_err("cannot read chg-temp-limits, ret=%d\n", ret);
		return ret;
	}
	if (profile->temp_nb_limits > CHG_TEMP_NB_LIMITS_MAX) {
		pr_err("chg-temp-nb-limits exceeds driver max: %d\n",
		       CHG_TEMP_NB_LIMITS_MAX);
		return -EINVAL;
	}
	ret = of_property_read_u32_array(node, "google,chg-temp-limits",
					 profile->temp_limits,
					 profile->temp_nb_limits);
	if (ret < 0) {
		pr_err("cannot read chg-temp-limits table, ret=%d\n", ret);
		return ret;
	}

	profile->volt_nb_limits =
	    of_property_count_elems_of_size(node, "google,chg-cv-limits",
					    sizeof(u32));
	if (profile->volt_nb_limits <= 0) {
		ret = profile->volt_nb_limits;
		pr_err("cannot read chg-cv-limits, ret=%d\n", ret);
		return ret;
	}
	if (profile->volt_nb_limits > CHG_VOLT_NB_LIMITS_MAX) {
		pr_err("chg-cv-nb-limits exceeds driver max: %d\n",
		       CHG_VOLT_NB_LIMITS_MAX);
		return -EINVAL;
	}
	ret = of_property_read_u32_array(node, "google,chg-cv-limits",
					 profile->volt_limits,
					 profile->volt_nb_limits);
	if (ret < 0) {
		pr_err("cannot read chg-cv-limits table, ret=%d\n", ret);
		return ret;
	}
	for (vi = 0; vi < profile->volt_nb_limits; vi++)
		profile->volt_limits[vi] = profile->volt_limits[vi] /
		    profile->cv_hw_resolution * profile->cv_hw_resolution;

	cccm_array_size =
	    (profile->temp_nb_limits - 1) * profile->volt_nb_limits;
	profile->cccm_limits = devm_kzalloc(dev,
					    sizeof(s32) * cccm_array_size,
					    GFP_KERNEL);

	ret = of_property_read_u32_array(node, "google,chg-cc-limits",
					 profile->cccm_limits, cccm_array_size);
	if (ret < 0) {
		pr_err("cannot read chg-cc-limits table, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,chg-cc-tolerance",
				   &profile->chg_cc_tolerance);
	if (ret < 0)
		profile->chg_cc_tolerance = 0;
	else if (profile->chg_cc_tolerance > 250)
		profile->chg_cc_tolerance = 250;

	/* chg-battery-capacity is in mAh, chg-cc-limits relative to 100 */
	for (ti = 0; ti < profile->temp_nb_limits - 1; ti++) {
		for (vi = 0; vi < profile->volt_nb_limits; vi++) {
			ccm = CCCM_LIMITS(profile, ti, vi);
			ccm *= profile->battery_capacity * 10;
			ccm = ccm * (1000 - profile->chg_cc_tolerance) / 1000;
			// round to the nearest resolution the PMIC can handle
			ccm = DIV_ROUND_CLOSEST(ccm, profile->cc_hw_resolution)
					* profile->cc_hw_resolution;
			CCCM_LIMITS(profile, ti, vi) = ccm;
		}
	}

	/* Taper WA */
	taper->taper_wa_en = of_property_read_bool(node, "google,chg-taper-wa");

	taper_tmp = of_property_count_elems_of_size(node, "google,chg-vtaper",
						    sizeof(u32));
	if (taper_tmp != profile->volt_nb_limits) {
		pr_err("wrong vtaper size\n");
		taper->taper_wa_en = false;
	} else {
		ret = of_property_read_u32_array(node, "google,chg-vtaper",
						 taper->v_taper,
						 profile->volt_nb_limits);
		if (ret < 0) {
			pr_err("can't read vtaper, ret=%d\n", ret);
			taper->taper_wa_en = false;
		}
	}

	taper_tmp = of_property_count_elems_of_size(node, "google,chg-itaper",
						    sizeof(u32));
	if (taper_tmp != profile->volt_nb_limits) {
		pr_err("wrong itaper size\n");
		taper->taper_wa_en = false;
	} else {
		ret = of_property_read_u32_array(node, "google,chg-itaper",
						 taper->i_taper,
						 profile->volt_nb_limits);
		if (ret < 0) {
			pr_err("can't read itaper, ret=%d\n", ret);
			taper->taper_wa_en = false;
		}
	}

	taper_tmp = of_property_count_elems_of_size(node, "google,chg-soctaper",
						    sizeof(u32));
	if (taper_tmp != profile->volt_nb_limits) {
		pr_err("wrong soctaper size\n");
		taper->taper_wa_en = false;
	} else {
		ret = of_property_read_u32_array(node, "google,chg-soctaper",
						 taper->soc_taper,
						 profile->volt_nb_limits);
		if (ret < 0) {
			pr_err("can't read soctaper, ret=%d\n", ret);
			taper->taper_wa_en = false;
		}
	}

	ret = of_property_read_u32(node, "google,chg-taper-cnt-target",
				   &taper->taper_cnt_target);
	if (ret < 0) {
		pr_err("can't read taper-cnt-target, ret=%d\n", ret);
		taper->taper_wa_en = false;
	}

	pr_info("successfully read charging profile:\n");
	dump_profile(profile);

	return 0;
}

static ssize_t show_tier_ovc(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		chg_drv->chg_profile.cv_tier_ov_cnt);
}

static ssize_t set_tier_ovc(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val < 0)
		return count;

	chg_drv->chg_profile.cv_tier_ov_cnt = val;
	return count;
}

static DEVICE_ATTR(tier_ovc, 0660, show_tier_ovc, set_tier_ovc);

static ssize_t show_cv_update_interval(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		chg_drv->chg_profile.cv_update_interval);
}

static ssize_t set_cv_update_interval(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val < 1500)
		return count;

	chg_drv->chg_profile.cv_update_interval = val;
	return count;
}

static DEVICE_ATTR(cv_update_interval, 0660,
		   show_cv_update_interval,
		   set_cv_update_interval);

static ssize_t show_cv_range_accuracy(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		chg_drv->chg_profile.cv_range_accuracy);
}

static ssize_t set_cv_range_accuracy(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	chg_drv->chg_profile.cv_range_accuracy = val;
	return count;
}

static DEVICE_ATTR(cv_range_accuracy, 0660,
		   show_cv_range_accuracy,
		   set_cv_range_accuracy);

static ssize_t show_charge_stop_level(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_drv->charge_stop_level);
}

static ssize_t set_charge_stop_level(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (!chg_drv->bat_psy) {
		pr_err("chg_drv->bat_psy is not ready");
		return -ENODATA;
	}

	if ((val == chg_drv->charge_stop_level) ||
	    (val <= chg_drv->charge_start_level) ||
	    (val > DEFAULT_CHARGE_STOP_LEVEL))
		return count;

	chg_drv->charge_stop_level = val;
	if (chg_drv->bat_psy)
		power_supply_changed(chg_drv->bat_psy);
	return count;
}

static DEVICE_ATTR(charge_stop_level, 0660,
		   show_charge_stop_level, set_charge_stop_level);

static ssize_t
show_charge_start_level(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_drv->charge_start_level);
}

static ssize_t set_charge_start_level(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (!chg_drv->bat_psy) {
		pr_err("chg_drv->bat_psy is not ready");
		return -ENODATA;
	}

	if ((val == chg_drv->charge_start_level) ||
	    (val >= chg_drv->charge_stop_level) ||
	    (val < DEFAULT_CHARGE_START_LEVEL))
		return count;

	chg_drv->charge_start_level = val;
	if (chg_drv->bat_psy)
		power_supply_changed(chg_drv->bat_psy);
	return count;
}

static DEVICE_ATTR(charge_start_level, 0660,
		   show_charge_start_level, set_charge_start_level);

static ssize_t show_bd_temp_enable(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_temp_enable);
}

static ssize_t set_bd_temp_enable(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (chg_drv->bd_state.bd_temp_enable == val)
		return count;

	chg_drv->bd_state.bd_temp_enable = val;

	bd_reset(&chg_drv->bd_state);

	if (chg_drv->bat_psy)
		power_supply_changed(chg_drv->bat_psy);

	return count;
}
static DEVICE_ATTR(bd_temp_enable, 0660,
		   show_bd_temp_enable, set_bd_temp_enable);


static ssize_t show_bd_trigger_voltage(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_trigger_voltage);
}

static ssize_t set_bd_trigger_voltage(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_VOLTAGE || val < MIN_BD_VOLTAGE)
		return -EINVAL;

	chg_drv->bd_state.bd_trigger_voltage = val;

	return count;
}

static DEVICE_ATTR(bd_trigger_voltage, 0660,
		   show_bd_trigger_voltage, set_bd_trigger_voltage);

static ssize_t show_bd_drainto_soc(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_drainto_soc);
}

static ssize_t set_bd_drainto_soc(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_SOC || val < MIN_BD_SOC)
		return -EINVAL;

	chg_drv->bd_state.bd_drainto_soc = val;

	return count;
}

static DEVICE_ATTR(bd_drainto_soc, 0660,
		   show_bd_drainto_soc, set_bd_drainto_soc);

static ssize_t show_bd_trigger_temp(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_trigger_temp);
}

static ssize_t set_bd_trigger_temp(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_TEMP || val < MIN_BD_TEMP)
		return -EINVAL;

	chg_drv->bd_state.bd_trigger_temp = val;

	return count;
}

static DEVICE_ATTR(bd_trigger_temp, 0660,
		   show_bd_trigger_temp, set_bd_trigger_temp);

static ssize_t show_bd_trigger_time(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_trigger_time);
}

static ssize_t set_bd_trigger_time(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val < 0)
		return -EINVAL;

	chg_drv->bd_state.bd_trigger_time = val;

	return count;
}

static DEVICE_ATTR(bd_trigger_time, 0660,
		   show_bd_trigger_time, set_bd_trigger_time);

static ssize_t show_bd_recharge_voltage(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_recharge_voltage);
}

static ssize_t set_bd_recharge_voltage(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_VOLTAGE || val < MIN_BD_VOLTAGE)
		return -EINVAL;

	chg_drv->bd_state.bd_recharge_voltage = val;

	return count;
}

static DEVICE_ATTR(bd_recharge_voltage, 0660,
		   show_bd_recharge_voltage, set_bd_recharge_voltage);

static ssize_t show_bd_recharge_soc(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_recharge_soc);
}

static ssize_t set_bd_recharge_soc(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_SOC || val < MIN_BD_SOC)
		return -EINVAL;

	chg_drv->bd_state.bd_recharge_soc = val;

	return count;
}

static DEVICE_ATTR(bd_recharge_soc, 0660,
		   show_bd_recharge_soc, set_bd_recharge_soc);


static ssize_t show_bd_resume_abs_temp(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_resume_abs_temp);
}

static ssize_t set_bd_resume_abs_temp(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_TEMP || val < MIN_BD_TEMP)
		return -EINVAL;

	chg_drv->bd_state.bd_resume_abs_temp = val;

	return count;
}

static DEVICE_ATTR(bd_resume_abs_temp, 0660,
		   show_bd_resume_abs_temp, set_bd_resume_abs_temp);


static ssize_t show_bd_resume_time(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_resume_time);
}

static ssize_t set_bd_resume_time(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val < 0)
		return -EINVAL;

	chg_drv->bd_state.bd_resume_time = val;

	return count;
}

static DEVICE_ATTR(bd_resume_time, 0660,
		   show_bd_resume_time, set_bd_resume_time);

static ssize_t show_bd_resume_temp(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_resume_temp);
}

static ssize_t set_bd_resume_temp(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_TEMP || val < MIN_BD_TEMP)
		return -EINVAL;

	chg_drv->bd_state.bd_resume_temp = val;

	return count;
}

static DEVICE_ATTR(bd_resume_temp, 0660,
		    show_bd_resume_temp, set_bd_resume_temp);

static ssize_t show_bd_resume_soc(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_resume_soc);
}

static ssize_t set_bd_resume_soc(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_SOC || val < MIN_BD_SOC)
		return -EINVAL;

	chg_drv->bd_state.bd_resume_soc = val;

	return count;
}
static DEVICE_ATTR(bd_resume_soc, 0660, show_bd_resume_soc, set_bd_resume_soc);

#ifdef CONFIG_DEBUG_FS

/* use qcom VS maxim fg and more... */
static int get_chg_mode(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	*val = chg_drv->chg_mode;
	return 0;
}

static int set_chg_mode(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	chg_drv->chg_mode = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(chg_mode_fops, get_chg_mode, set_chg_mode, "%llu\n");

/* allow vbatt to be over vtier by margin */
static int get_cv_otv_margin(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	*val = chg_drv->chg_profile.cv_otv_margin;
	return 0;
}

static int set_cv_otv_margin(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	chg_drv->chg_profile.cv_otv_margin = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cv_otv_margin_fops, get_cv_otv_margin,
	set_cv_otv_margin, "%llu\n");

/* cap fv_uv to a percentage of vtier (1000 -> vtier, 1020 -> 2% over vtier) */
static int get_fv_uv_margin(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	*val = chg_drv->chg_profile.fv_uv_margin_dpct;
	return 0;
}

static int set_fv_uv_margin(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	chg_drv->chg_profile.fv_uv_margin_dpct = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fv_uv_margin_fops, get_fv_uv_margin,
	set_fv_uv_margin, "%llu\n");

static int bd_enabled_get(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	mutex_lock(&chg_drv->bd_lock);
	*val = chg_drv->bd_state.enabled;
	mutex_unlock(&chg_drv->bd_lock);

	return 0;
}

static int bd_enabled_set(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	mutex_lock(&chg_drv->bd_lock);

	bd_reset(&chg_drv->bd_state);
	if (val == 0)
		chg_drv->bd_state.enabled = 0;

	mutex_unlock(&chg_drv->bd_lock);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bd_enabled_fops, bd_enabled_get,
			bd_enabled_set, "%lld\n");

#endif

static int init_debugfs(struct chg_drv *chg_drv)
{
#ifdef CONFIG_DEBUG_FS
	struct dentry *de;

	de = debugfs_create_dir("google_charger", 0);
	if (de) {
		debugfs_create_file("chg_mode", 0644, de,
				   chg_drv, &chg_mode_fops);
		debugfs_create_file("cv_otv_margin", 0644, de,
				   chg_drv, &cv_otv_margin_fops);
		debugfs_create_file("fv_uv_margin", 0644, de,
				   chg_drv, &fv_uv_margin_fops);
		debugfs_create_u32("bd_triggered", 0644, de,
			   &chg_drv->bd_state.triggered);
		debugfs_create_file("bd_enabled", 0600, de,
			    chg_drv, &bd_enabled_fops);
	}
#endif
	return 0;

}


static void google_charger_init_work(struct work_struct *work)
{
	struct chg_drv *chg_drv = container_of(work, struct chg_drv,
					       init_work.work);
	struct power_supply *chg_psy, *usb_psy, *wlc_psy = NULL, *bat_psy;
	int ret = 0;

	chg_psy = power_supply_get_by_name(chg_drv->chg_psy_name);
	if (!chg_psy) {
		pr_info("failed to get \"%s\" power supply\n",
			chg_drv->chg_psy_name);
		goto retry_init_work;
	}

	bat_psy = power_supply_get_by_name(chg_drv->bat_psy_name);
	if (!bat_psy) {
		pr_info("failed to get \"%s\" power supply\n",
			chg_drv->bat_psy_name);
		power_supply_put(chg_psy);
		goto retry_init_work;
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_info("failed to get \"usb\" power supply\n");
		power_supply_put(chg_psy);
		power_supply_put(bat_psy);
		goto retry_init_work;
	}

	if (chg_drv->wlc_psy_name) {
		wlc_psy = power_supply_get_by_name(chg_drv->wlc_psy_name);
		if (!wlc_psy) {
			pr_info("failed to get \"%s\" power supply\n",
				chg_drv->wlc_psy_name);
			power_supply_put(chg_psy);
			power_supply_put(bat_psy);
			power_supply_put(usb_psy);
			goto retry_init_work;
		}
	}

	chg_drv->chg_psy = chg_psy;
	chg_drv->wlc_psy = wlc_psy;
	chg_drv->usb_psy = usb_psy;
	chg_drv->bat_psy = bat_psy;

	chg_drv->charge_stop_level = DEFAULT_CHARGE_STOP_LEVEL;
	chg_drv->charge_start_level = DEFAULT_CHARGE_START_LEVEL;

	reset_chg_drv_state(chg_drv);
	chg_drv->plugged = -1;

	chg_drv->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chg_drv->psy_nb);
	if (ret < 0)
		pr_err("Cannot register power supply notifer, ret=%d\n", ret);

	wakeup_source_init(&chg_drv->chg_ws, "google-charger");
	pr_info("google_charger_init_work done\n");

	mutex_init(&chg_drv->bd_lock);
	wakeup_source_init(&chg_drv->bd_ws, "defender");
	INIT_DELAYED_WORK(&chg_drv->bd_work, bd_work);
	bd_init(&chg_drv->bd_state, chg_drv->device);

	/* catch state changes that happened before registering the notifier */
	schedule_delayed_work(&chg_drv->chg_work,
		msecs_to_jiffies(CHG_DELAY_INIT_DETECT_MS));
	return;

retry_init_work:
	schedule_delayed_work(&chg_drv->init_work,
			      msecs_to_jiffies(CHG_DELAY_INIT_MS));
}

static int google_charger_probe(struct platform_device *pdev)
{
	const char *chg_psy_name, *bat_psy_name, *wlc_psy_name = NULL;
	struct chg_drv *chg_drv;
	int ret;

	chg_drv = devm_kzalloc(&pdev->dev, sizeof(*chg_drv), GFP_KERNEL);
	if (!chg_drv)
		return -ENOMEM;

	chg_drv->device = &pdev->dev;

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,chg-power-supply",
				      &chg_psy_name);
	if (ret != 0) {
		pr_err("cannot read google,chg-power-supply, ret=%d\n", ret);
		return -EINVAL;
	}
	chg_drv->chg_psy_name =
	    devm_kstrdup(&pdev->dev, chg_psy_name, GFP_KERNEL);
	if (!chg_drv->chg_psy_name)
		return -ENOMEM;

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,bat-power-supply",
				      &bat_psy_name);
	if (ret != 0) {
		pr_err("cannot read google,bat-power-supply, ret=%d\n", ret);
		return -EINVAL;
	}
	chg_drv->bat_psy_name =
	    devm_kstrdup(&pdev->dev, bat_psy_name, GFP_KERNEL);
	if (!chg_drv->bat_psy_name)
		return -ENOMEM;

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,wlc-power-supply",
				      &wlc_psy_name);
	if (ret != 0)
		pr_warn("google,wlc-power-supply not defined\n");
	if (wlc_psy_name) {
		chg_drv->wlc_psy_name =
		    devm_kstrdup(&pdev->dev, wlc_psy_name, GFP_KERNEL);
		if (!chg_drv->wlc_psy_name)
			return -ENOMEM;
	}

	ret = chg_init_chg_profile(chg_drv);
	if (ret < 0) {
		pr_err("cannot read charging profile from dt, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_charge_stop_level);
	if (ret != 0) {
		pr_err("Failed to create charge_stop_level files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_charge_start_level);
	if (ret != 0) {
		pr_err("Failed to create charge_start_level files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_temp_enable);
	if (ret != 0) {
		pr_err("Failed to create bd_temp_enable files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_trigger_voltage);
	if (ret != 0) {
		pr_err("Failed to create bd_trigger_voltage files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_drainto_soc);
	if (ret != 0) {
		pr_err("Failed to create bd_drainto_soc files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_trigger_temp);
	if (ret != 0) {
		pr_err("Failed to create bd_trigger_temp files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_trigger_time);
	if (ret != 0) {
		pr_err("Failed to create bd_trigger_time files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device,
				 &dev_attr_bd_recharge_voltage);
	if (ret != 0) {
		pr_err("Failed to create bd_recharge_voltage files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_recharge_soc);
	if (ret != 0) {
		pr_err("Failed to create bd_recharge_soc files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_resume_abs_temp);
	if (ret != 0) {
		pr_err("Failed to create bd_resume_abs_temp files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_resume_time);
	if (ret != 0) {
		pr_err("Failed to create bd_trigger_time files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_resume_temp);
	if (ret != 0) {
		pr_err("Failed to create bd_resume_temp files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_resume_soc);
	if (ret != 0) {
		pr_err("Failed to create bd_resume_soc files, ret=%d\n",
		       ret);
		return ret;
	}

	// TODO: move to debugfs
	ret = device_create_file(&pdev->dev, &dev_attr_cv_range_accuracy);
	if (ret != 0) {
		pr_err("Failed to create cv_range_accuracy files, ret=%d\n",
		       ret);
		return ret;
	}

	// TODO: move to debugfs
	ret = device_create_file(&pdev->dev, &dev_attr_cv_update_interval);
	if (ret != 0) {
		pr_err("Failed to create cv_update_interval files, ret=%d\n",
			ret);
		return ret;
	}

	// TODO: move to debugfs
	ret = device_create_file(&pdev->dev, &dev_attr_tier_ovc);
	if (ret != 0) {
		pr_err("Failed to create tier_ovc files, ret=%d\n", ret);
		return ret;
	}

	/* debug */
	init_debugfs(chg_drv);

	INIT_DELAYED_WORK(&chg_drv->init_work, google_charger_init_work);
	INIT_DELAYED_WORK(&chg_drv->chg_work, chg_work);
	platform_set_drvdata(pdev, chg_drv);

	schedule_delayed_work(&chg_drv->init_work,
			      msecs_to_jiffies(CHG_DELAY_INIT_MS));

	return 0;
}

static int google_charger_remove(struct platform_device *pdev)
{
	struct chg_drv *chg_drv = platform_get_drvdata(pdev);

	if (chg_drv) {
		if (chg_drv->chg_psy)
			power_supply_put(chg_drv->chg_psy);
		if (chg_drv->bat_psy)
			power_supply_put(chg_drv->bat_psy);
		if (chg_drv->usb_psy)
			power_supply_put(chg_drv->usb_psy);
		if (chg_drv->wlc_psy)
			power_supply_put(chg_drv->wlc_psy);
		wakeup_source_trash(&chg_drv->bd_ws);
		wakeup_source_trash(&chg_drv->chg_ws);
	}

	return 0;
}

static const struct of_device_id match_table[] = {
	{.compatible = "google,charger"},
	{},
};

static struct platform_driver charger_driver = {
	.driver = {
		   .name = "google,charger",
		   .owner = THIS_MODULE,
		   .of_match_table = match_table,
		   },
	.probe = google_charger_probe,
	.remove = google_charger_remove,
};

static int __init google_charger_init(void)
{
	int ret;

	ret = platform_driver_register(&charger_driver);
	if (ret < 0) {
		pr_err("device registration failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static void __init google_charger_exit(void)
{
	platform_driver_unregister(&charger_driver);
	pr_info("unregistered platform driver\n");
}

module_init(google_charger_init);
module_exit(google_charger_exit);
MODULE_DESCRIPTION("Multi-step battery charger driver");
MODULE_AUTHOR("Thierry Strudel <tstrudel@google.com>");
MODULE_LICENSE("GPL");
