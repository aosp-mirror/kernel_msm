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
#include "google_bms.h"
#include "google_psy.h"


#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define CHG_DELAY_INIT_MS 250
#define CHG_DELAY_INIT_DETECT_MS 1000

#define DEFAULT_CHARGE_STOP_LEVEL 100
#define DEFAULT_CHARGE_START_LEVEL 0

#define CHG_WORK_ERROR_RETRY_MS 1000

#define CHG_DRV_CC_HW_TOLERANCE_MAX	250

#define CHG_DRV_MODE_NOIRDROP	1

#define DRV_DEFAULTCC_UPDATE_INTERVAL	30000
#define DRV_DEFAULTCV_UPDATE_INTERVAL	2000

struct chg_drv {
	struct device *device;
	struct power_supply *chg_psy;
	const char *chg_psy_name;
	struct power_supply *usb_psy;
	struct power_supply *wlc_psy;
	const char *wlc_psy_name;
	struct power_supply *bat_psy;
	const char *bat_psy_name;
	struct power_supply *tcpm_psy;
	const char *tcpm_psy_name;
	struct notifier_block psy_nb;
	struct delayed_work init_work;
	struct delayed_work chg_work;
	struct wakeup_source chg_ws;

	u32 cv_update_interval;
	u32 cc_update_interval;

	int fv_uv;		/* newgen */
	int cc_max;		/* newgen */
	int chg_cc_tolerance;
	int chg_mode;		/* debug */
	bool stop_charging;
	int disable_charging;
	int disable_pwrsrc;
	bool lowerdb_reached;	/* retail */
	int charge_stop_level;	/* retail */
	int charge_start_level;	/* retail */
};

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

static char *psy_usb_type_str[] = {
	"Unknown", "Battery", "UPS", "Mains", "USB",
	"USB_DCP", "USB_CDP", "USB_ACA", "USB_C",
	"USB_PD", "USB_PD_DRP", "BrickID",
	"USB_HVDCP", "USB_HVDCP_3", "Wireless", "USB_FLOAT",
	"BMS", "Parallel", "Main", "Wipower", "USB_C_UFP", "USB_C_DFP",
};

static char *psy_usbc_type_str[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

static inline void reset_chg_drv_state(struct chg_drv *chg_drv)
{
	union gbms_charger_state chg_state = { .v = 0 };

	chg_drv->fv_uv = -1;
	chg_drv->cc_max = -1;
	chg_drv->disable_charging = 0;
	chg_drv->disable_pwrsrc = 0;
	chg_drv->lowerdb_reached = true;

	chg_drv->stop_charging = true;
	GPSY_SET_PROP(chg_drv->chg_psy, POWER_SUPPLY_PROP_CHARGE_DISABLE, 1);

	GPSY_SET_PROP(chg_drv->chg_psy,
		      POWER_SUPPLY_PROP_TAPER_CONTROL,
		      POWER_SUPPLY_TAPER_CONTROL_OFF);

	/* make sure the battery knows that it's disconnected */
	GPSY_SET_INT64_PROP(chg_drv->bat_psy,
		POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE, chg_state.v);
}

static void pr_info_usb_state(struct power_supply *usb_psy,
			      struct power_supply *tcpm_psy)
{
	int usb_type, usbc_type;

	usb_type = GPSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_REAL_TYPE);
	if (tcpm_psy)
		usbc_type = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_USB_TYPE);

	pr_info("usbchg=%s typec=%s usbv=%d usbc=%d usbMv=%d usbMc=%d\n",
		psy_usb_type_str[usb_type],
		tcpm_psy ? psy_usbc_type_str[usbc_type] : "null",
		GPSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW) / 1000,
		GPSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW)/1000,
		GPSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX) / 1000,
		GPSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_CURRENT_MAX) / 1000);
}

static void pr_info_wlc_state(struct power_supply *wlc_psy)
{
	pr_info("wlcv=%d wlcc=%d wlcMv=%d wlcMc=%d wlct=%d\n",
		GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW) / 1000,
		GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_CURRENT_NOW) / 1000,
		GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX) / 1000,
		GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_CURRENT_MAX) / 1000,
		GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_TEMP));
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

static int chg_set_charger(struct power_supply *chg_psy, int fv_uv, int cc_max)
{
	int rc;

	/* TAPER CONTROL is in the charger */
	rc = GPSY_SET_PROP(chg_psy,
		POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, cc_max);
	if (rc != 0) {
		pr_err("MSC_CHG cannot set charging current rc=%d\n", rc);
		return -EIO;
	}

	rc = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, fv_uv);
	if (rc != 0) {
		pr_err("MSC_CHG cannot set float voltage rc=%d\n", rc);
		return -EIO;
	}

	return rc;
}

static uint8_t chg_work_gen_flags(int chg_status, int chg_type)
{
	uint8_t flags = 0;

	if (chg_status != POWER_SUPPLY_STATUS_DISCHARGING)
		flags |= GBMS_CS_FLAG_BUCK_EN;
	if (chg_status == POWER_SUPPLY_STATUS_FULL)
		flags |= GBMS_CS_FLAG_DONE;
	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_TAPER)
		flags |= GBMS_CS_FLAG_CV;
	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST)
		flags |= GBMS_CS_FLAG_CC;

	return flags;
}

static int chg_work_gen_state(union gbms_charger_state *chg_state,
			       struct power_supply *chg_psy)
{
	int vchrg, chg_type, chg_status;

	/* TODO: if (chg_drv->chg_mode == CHG_DRV_MODE_NOIRDROP) vchrg = 0; */
	/* Battery needs to know charger voltage and state to run the irdrop
	 * compensation code, can disable here sending a 0 vchgr
	 */
	vchrg = GPSY_GET_PROP(chg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	chg_type = GPSY_GET_PROP(chg_psy, POWER_SUPPLY_PROP_CHARGE_TYPE);
	chg_status = GPSY_GET_PROP(chg_psy, POWER_SUPPLY_PROP_STATUS);
	if (vchrg == -EINVAL || chg_type == -EINVAL || chg_status == -EINVAL) {
		pr_err("MSC_CHG error vchrg=%d chg_type=%d chg_status=%d\n",
			vchrg, chg_type, chg_status);
		return -EINVAL;
	}

	chg_state->f.chg_type = chg_type;
	chg_state->f.chg_status = chg_status;
	chg_state->f.vchrg = vchrg / 1000; /* vchrg is in uA, f.vchrg us mA */
	chg_state->f.flags |= chg_work_gen_flags(chg_status, chg_type);

	return 0;
}

static int chg_work_read_state(union gbms_charger_state *chg_state,
			       struct power_supply *chg_psy)
{
	union power_supply_propval val;
	int ichg, ret = 0;

	ret = power_supply_get_property(chg_psy,
					POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE,
					&val);
	if (ret == 0) {
		chg_state->v = val.int64val;
	} else {
		ret = chg_work_gen_state(chg_state, chg_psy);
		if (ret < 0)
			return ret;
	}

	/* debug only */
	ichg = GPSY_GET_PROP(chg_psy, POWER_SUPPLY_PROP_CURRENT_NOW);

	pr_info("MSC_CHG chg_state[%x:%x:%x:%x]:%lx ichg=%d chg_t=%s chg_s=%s\n",
		chg_state->f.chg_type, chg_state->f.chg_status,
		chg_state->f.flags, chg_state->f.vchrg,
		(unsigned long)chg_state->v, ichg,
		gbms_chg_type_s(chg_state->f.chg_type),
		gbms_chg_status_s(chg_state->f.chg_status));

	return 0;
}

/* 0 stop charging, <0 error, positive keep going */
static int chg_work_roundtrip(const union gbms_charger_state *chg_state,
			      struct power_supply *bat_psy,
			      int *fv_uv, int *cc_max)
{
	int rc;

	rc = GPSY_SET_INT64_PROP(bat_psy,
		POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE, chg_state->v);
	if (rc < 0) {
		pr_err("MSC_CHG error cannot set CHARGE_CHARGER_STATE rc=%d\n",
		       rc);
		return -EINVAL;
	}

	*cc_max = GPSY_GET_PROP(bat_psy,
		POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT);
	*fv_uv = GPSY_GET_PROP(bat_psy,
		POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE);

	/* ASSERT: (chg_state.f.flags&GBMS_CS_FLAG_DONE) && cc_max == 0 */

	if (*cc_max == 0)
		return 0;

	if (*fv_uv < 0 || *cc_max < 0) {
		pr_err("MSC_CHG error on fv_uv=%d cc_max=%d\n",
		       *fv_uv, *cc_max);
		return -EINVAL;
	}

	return 0;
}

/* b/117985113 */
static int chg_usb_online(struct power_supply *usb_psy)
{
	int usb_online, mode;

	mode = GPSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_TYPEC_MODE);
	if (mode < 0)
		return mode;

	switch (mode) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		usb_online = 1;
		break;
	default:
		usb_online = 0;
		break;
	}

	return usb_online;
}

/* new gen charging code */
/* TODO: find a strategy for update_interval. Could start with
 * long interval during fast charge (discharge while connected?)
 * and short interval at taper. NOTE that  battery might require
 * a different strategy. So, either the charge code read the
 * update interval as part of the charging loop and use a MIN
 * votable to set the rate, or battery send a PS notification
 * (same type? new PS type?) to request re-running the chage
 * logic again. Also, battery could vote to a common votable in
 * google_bms (to reduce churn due to PS notifications)
*/
static void chg_work(struct work_struct *work)
{
	struct chg_drv *chg_drv =
	    container_of(work, struct chg_drv, chg_work.work);
	struct power_supply *usb_psy = chg_drv->usb_psy;
	struct power_supply *tcpm_psy = chg_drv->tcpm_psy;
	struct power_supply *wlc_psy = chg_drv->wlc_psy;
	struct power_supply *bat_psy = chg_drv->bat_psy;
	struct power_supply *chg_psy = chg_drv->chg_psy;
	int usb_online, wlc_online = 0;
	int update_interval = chg_drv->cc_update_interval;
	int soc, disable_charging;
	int disable_pwrsrc;
	int rc = 0;

	__pm_stay_awake(&chg_drv->chg_ws);
	pr_debug("battery charging work item\n");

	usb_online = chg_usb_online(usb_psy);
	if (wlc_psy)
		wlc_online = GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_ONLINE);

	/* If no power source, stop charging and exit */
	if (usb_online  < 0 || wlc_online < 0) {
		pr_err("error reading usb=%d wlc=%d\n",
			usb_online, wlc_online);
		goto error_rerun;
	} else if (!usb_online && !wlc_online) {
		pr_info("MSC_CHG no power source detected, disabling charging\n");
		reset_chg_drv_state(chg_drv);

		goto exit_chg_work;
	} else if (chg_drv->stop_charging) {
	/* might be disabled later due to is_charging_enabled */
		pr_info("MSC_CHG power source detected, enabling charging\n");
		GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_CHARGE_DISABLE, 0);
		chg_drv->stop_charging = false;
	}

	if (usb_online)
		pr_info_usb_state(usb_psy, tcpm_psy);
	if (wlc_online)
		pr_info_wlc_state(wlc_psy);

	soc = GPSY_GET_PROP(bat_psy, POWER_SUPPLY_PROP_CAPACITY);
	if (soc == -EINVAL)
		goto error_rerun;

	/* this force drain: we might decide to run from power instead.
	 * Enable/disable charging comes only from is_charging_disabled
	 * */
	disable_charging = is_charging_disabled(chg_drv, soc);
	if (disable_charging && soc > chg_drv->charge_stop_level)
		disable_pwrsrc = 1;
	else
		disable_pwrsrc = 0;

	if (disable_charging != chg_drv->disable_charging) {
		pr_info("set disable_charging(%d)", disable_charging);
		GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_CHARGE_DISABLE,
			     disable_charging);
	}
	chg_drv->disable_charging = disable_charging;

	if (disable_pwrsrc != chg_drv->disable_pwrsrc) {
		pr_info("set disable_pwrsrc(%d)", disable_pwrsrc);
		GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_INPUT_SUSPEND,
			     disable_pwrsrc);
	}
	chg_drv->disable_pwrsrc = disable_pwrsrc;

	if (chg_drv->disable_charging || chg_drv->disable_pwrsrc) {

		rc = chg_set_charger(chg_psy, chg_drv->fv_uv, 0);
		if (rc != 0)
			goto error_rerun;

		/* NOTE: might still need/want to roundtrip to the battery */

		/* no need to reschedule, battery psy event will reschedule */
	} else {
		int fv_uv, cc_max;
		union gbms_charger_state chg_state = { .v = 0 };

		rc = chg_work_read_state(&chg_state, chg_psy);
		if (rc < 0)
			goto error_rerun;

		switch (chg_state.f.chg_status) {
		case POWER_SUPPLY_STATUS_FULL:
			update_interval = chg_drv->cc_update_interval;
			break;
		case POWER_SUPPLY_STATUS_CHARGING:
			/* use cc interval ONLY when in CC */
			if ((chg_state.f.flags & GBMS_CS_FLAG_CC) == 0)
				update_interval = chg_drv->cv_update_interval;
			break;
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			update_interval = chg_drv->cv_update_interval;
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			/* DISCHARGING only when not connected, stop charging */
			update_interval = 0;
			break;
		default:
			pr_err("invalid charging status %d\n",
			       chg_state.f.chg_status);
			update_interval = chg_drv->cv_update_interval;
			break;
		}

		rc = chg_work_roundtrip(&chg_state, bat_psy, &fv_uv, &cc_max);
		if (rc < 0)
			goto error_rerun;

		/* TODO: b/117949231, support for PSS */

		/* sanity/termination */
		if (fv_uv == -1)
			fv_uv = chg_drv->fv_uv;
		if  (cc_max == -1)
			cc_max = 0;

		/* apply charger tolerance (if it has to...) */
		cc_max = ( cc_max / 1000 ) * (1000 - chg_drv->chg_cc_tolerance);

		if (chg_drv->fv_uv != fv_uv || chg_drv->cc_max != cc_max) {

			/* TAPER CONTROL is in the charger */
			/* charge tolerance is applied in the charger. */
			rc = chg_set_charger(chg_psy, fv_uv, cc_max);
			if (rc != 0)
				goto error_rerun;

			pr_info("MSC_CHG l=%d fv_uv=%d->%d cc_max=%d->%d ui=%d\n",
				soc,
				chg_drv->fv_uv, fv_uv,
				chg_drv->cc_max, cc_max,
				update_interval);

			chg_drv->cc_max = cc_max;
			chg_drv->fv_uv = fv_uv;
		} else {
			pr_info("MSC_CHG l=%d fv=%d cc_max=%d usb=%d wlc=%d ui=%d\n",
				soc, fv_uv, cc_max,
				usb_online, wlc_online,
				update_interval);
		}

	}


	if (update_interval) {
		pr_debug("rerun battery charging work in %d ms\n",
			 update_interval);
		schedule_delayed_work(&chg_drv->chg_work,
				      msecs_to_jiffies(update_interval));
	} else {
		pr_info("stop battery charging work:\n");
		reset_chg_drv_state(chg_drv);
	}

	goto exit_chg_work;

error_rerun:
	pr_err("error %d occurred, rerun battery charging work in %d ms\n",
	       rc, CHG_WORK_ERROR_RETRY_MS);
	schedule_delayed_work(&chg_drv->chg_work,
			      msecs_to_jiffies(CHG_WORK_ERROR_RETRY_MS));

exit_chg_work:
	__pm_relax(&chg_drv->chg_ws);
}

// ----------------------------------------------------------------------------

/* return negative when using ng charging */
static int chg_init_chg_profile(struct chg_drv *chg_drv)
{
	struct device *dev = chg_drv->device;
	struct device_node *node = dev->of_node;
	int ret;

	/* TODO: allow 0 to have gas gauge decide this */
	ret = of_property_read_u32(node, "google,cv-update-interval",
				   &chg_drv->cv_update_interval);
	if (ret < 0 || chg_drv->cv_update_interval == 0)
		chg_drv->cv_update_interval = DRV_DEFAULTCV_UPDATE_INTERVAL;

	ret = of_property_read_u32(node, "google,cc-update-interval",
				   &chg_drv->cc_update_interval);
	if (ret < 0 || chg_drv->cc_update_interval == 0)
		chg_drv->cc_update_interval = DRV_DEFAULTCC_UPDATE_INTERVAL;

	/* add "safety" margin for C rates if the charger doesn't do it.
	 * when set will reduce cc_max by
	 * 	cc_max = cc_max * (1000 - chg_cc_tolerance) / 1000;
	 */
	ret = of_property_read_u32(node, "google,chg-cc-tolerance",
				   &chg_drv->chg_cc_tolerance);
	if (ret < 0)
		chg_drv->chg_cc_tolerance = 0;
	else if (chg_drv->chg_cc_tolerance > CHG_DRV_CC_HW_TOLERANCE_MAX)
		chg_drv->chg_cc_tolerance = CHG_DRV_CC_HW_TOLERANCE_MAX;

	pr_info("charging profile in the battery\n");

	return 0;
}

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
#endif

static int init_debugfs(struct chg_drv *chg_drv)
{
#ifdef CONFIG_DEBUG_FS
	struct dentry *de;

	de = debugfs_create_dir("google_charger", 0);
	if (de) {
		debugfs_create_file("chg_mode", 0644, de,
				   chg_drv, &chg_mode_fops);
	}
#endif
	return 0;

}


static void google_charger_init_work(struct work_struct *work)
{
	struct chg_drv *chg_drv = container_of(work, struct chg_drv,
					       init_work.work);
	struct power_supply *chg_psy, *usb_psy, *wlc_psy = NULL, *bat_psy;
	struct power_supply *tcpm_psy = NULL;
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

	if (chg_drv->tcpm_psy_name) {
		tcpm_psy = power_supply_get_by_name(chg_drv->tcpm_psy_name);
		if (!tcpm_psy) {
			pr_info("failed to get \"%s\" power supply\n",
				chg_drv->tcpm_psy_name);
			power_supply_put(chg_psy);
			power_supply_put(bat_psy);
			power_supply_put(usb_psy);
			if (wlc_psy)
				power_supply_put(wlc_psy);
			goto retry_init_work;
		}
	}

	chg_drv->chg_psy = chg_psy;
	chg_drv->wlc_psy = wlc_psy;
	chg_drv->usb_psy = usb_psy;
	chg_drv->bat_psy = bat_psy;
	chg_drv->tcpm_psy = tcpm_psy;

	chg_drv->charge_stop_level = DEFAULT_CHARGE_STOP_LEVEL;
	chg_drv->charge_start_level = DEFAULT_CHARGE_START_LEVEL;

	reset_chg_drv_state(chg_drv);

	chg_drv->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chg_drv->psy_nb);
	if (ret < 0)
		pr_err("Cannot register power supply notifer, ret=%d\n", ret);

	wakeup_source_init(&chg_drv->chg_ws, "google-charger");
	pr_info("google_charger_init_work done\n");

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
	const char *tcpm_psy_name = NULL;
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

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,tcpm-power-supply",
				      &tcpm_psy_name);
	if (ret != 0)
		pr_warn("google,tcpm-power-supply not defined\n");
	if (tcpm_psy_name) {
		chg_drv->tcpm_psy_name =
		    devm_kstrdup(&pdev->dev, tcpm_psy_name, GFP_KERNEL);
		if (!chg_drv->tcpm_psy_name)
			return -ENOMEM;
	}

	/* NOTE: newgen charging table is configured in google_battery */
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
		if (chg_drv->tcpm_psy)
			power_supply_put(chg_drv->tcpm_psy);

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