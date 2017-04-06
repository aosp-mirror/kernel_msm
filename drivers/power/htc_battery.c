/* Copyright (c) 2010-2017, The Linux Foundation. All rights reserved.
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
#include <linux/alarmtimer.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_batterydata.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/reboot.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#define HTC_BATT_NAME "htc_battery"
static struct htc_battery_info htc_batt_info;
static struct htc_battery_timer htc_batt_timer;

#define BATT_LOG(x...) pr_info("[BATT] " x)

#define BATT_DEBUG(x...) do { \
		if (g_flag_enable_batt_debug_log) \
			pr_info("[BATT] " x); \
		else \
			pr_debug("[BATT] " x); \
	} while (0)

#define BATT_ERR(x...) do { \
	struct timespec _ts; \
	struct rtc_time _tm; \
	getnstimeofday(&_ts); \
	rtc_time_to_tm(_ts.tv_sec, &_tm); \
	pr_err("[BATT] err:" x); \
	pr_err("[BATT] at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
	       ktime_to_ns(ktime_get()), \
	       _tm.tm_year + 1900, _tm.tm_mon + 1, _tm.tm_mday, \
	       _tm.tm_hour, _tm.tm_min, _tm.tm_sec, _ts.tv_nsec); \
	} while (0)

#define BATT_EMBEDDED(x...) do { \
	struct timespec _ts; \
	struct rtc_time _tm; \
	getnstimeofday(&_ts); \
	rtc_time_to_tm(_ts.tv_sec, &_tm); \
	pr_info("[BATT] at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
	       ktime_to_ns(ktime_get()), \
	       _tm.tm_year + 1900, _tm.tm_mon + 1, _tm.tm_mday, \
	       _tm.tm_hour, _tm.tm_min, _tm.tm_sec, _ts.tv_nsec); \
	pr_info("[BATT] :" x); \
} while (0)

struct battery_info_reply {
	u32 batt_vol;
	u32 batt_id;
	s32 batt_temp;
	s32 batt_current;
	u32 charging_source;
	u32 level;
	u32 level_raw;
	u32 full_level;
	u32 status;
	u32 chg_src;
	u32 chg_en;
	u32 chg_batt_en;
	u32 full_level_dis_batt_chg;
	u32 overload;
	u32 over_vchg;
	u32 health;
	bool is_full;
};

struct battery_info_previous {
	s32 batt_temp;
	u32 charging_source;
	u32 level;
	u32 level_raw;
};

struct htc_thermal_stage {
	int nextTemp;
	int recoverTemp;
};

struct htc_battery_info {
	struct battery_info_reply	rep;
	struct battery_info_previous	prev;
	struct htc_charger	*icharger;
	struct power_supply	*batt_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*usb_psy;
	int critical_low_voltage_mv;
	int smooth_chg_full_delay_min;
	int decreased_batt_level_check;
	int batt_full_voltage_mv;
	int batt_full_current_ma;
	int overload_curr_thr_ma;
	struct wake_lock charger_exist_lock;
	struct work_struct batt_update_work;
	int state;
	int vbus;
	int batt_fcc_ma;
	int batt_capacity_mah;
	int batt_thermal_limit_vol;
	struct notifier_block fb_notif;
	struct notifier_block htc_batt_cb;
};

struct htc_battery_timer {
	unsigned long batt_system_jiffies;
	unsigned long batt_suspend_ms;
	unsigned long total_time_ms;	/* since last do batt_work */
	struct work_struct batt_work;
	struct timer_list batt_timer;
	struct workqueue_struct *batt_wq;
	struct wake_lock battery_lock;
	unsigned int time_out;
	struct alarm batt_check_wakeup_alarm;
};

enum {
	BATT_ID_1 = 1,
	BATT_ID_2,
	BATT_ID_UNKNOWN = 255,
};

#define BATT_SUSPEND_CHECK_TIME			(3600)
#define BATT_SUSPEND_HIGHFREQ_CHECK_TIME	(300)
#define BATT_TIMER_CHECK_TIME			(360)
#define BATT_TIMER_UPDATE_TIME			(60)
#define CHECH_TIME_TOLERANCE_MS			(1000)

/* disable pwrsrc reason */
#define HTC_BATT_PWRSRC_DIS_BIT_MFG		BIT(0)
#define HTC_BATT_PWRSRC_DIS_BIT_API		BIT(1)
#define HTC_BATT_PWRSRC_DIS_BIT_USB_OVERHEAT	BIT(2)
#define HTC_BATT_PWRSRC_DIS_BIT_FTM		BIT(3)
static int g_pwrsrc_dis_reason;

/* disable charging reason */
#define HTC_BATT_CHG_DIS_BIT_EOC		BIT(0)
#define HTC_BATT_CHG_DIS_BIT_ID			BIT(1)
#define HTC_BATT_CHG_DIS_BIT_TMP		BIT(2)
#define HTC_BATT_CHG_DIS_BIT_OVP		BIT(3)
#define HTC_BATT_CHG_DIS_BIT_TMR		BIT(4)
#define HTC_BATT_CHG_DIS_BIT_MFG		BIT(5)
#define HTC_BATT_CHG_DIS_BIT_USR_TMR		BIT(6)
#define HTC_BATT_CHG_DIS_BIT_STOP_SWOLLEN	BIT(7)
#define HTC_BATT_CHG_DIS_BIT_USB_OVERHEAT	BIT(8)
#define HTC_BATT_CHG_DIS_BIT_FTM		BIT(9)
static int g_chg_dis_reason;

/* for htc_batt_info.state */
#define STATE_SCREEN_OFF			(1)

/* Set true when all battery need file probe done */
static bool g_htc_battery_probe_done;

/* Enable batterydebug log*/
static bool g_flag_enable_batt_debug_log;

static int gs_prev_charging_enabled;

/* reference from power_supply.h power_supply_type */
const char *g_chr_src[] = {
	"NONE", "BATTERY", "UPS", "MAINS", "USB", "AC(USB_DCP)",
	"USB_CDP", "USB_ACA", "USB_HVDCP", "USB_HVDCP_3", "USB_PD",
	"WIRELESS", "BMS", "USB_PARALLEL", "WIPOWER", "TYPEC", "UFP", "DFP"};

static void batt_set_check_timer(u32 seconds)
{
	pr_debug("[BATT] %s(%u sec)\n", __func__, seconds);
	mod_timer(&htc_batt_timer.batt_timer,
		  jiffies + msecs_to_jiffies(seconds * 1000));
}

static int get_property(struct power_supply *psy,
			enum power_supply_property prop)
{
	union power_supply_propval ret = {0, };
	int rc = 0;

	if (!g_htc_battery_probe_done)
		return -EINVAL;

	if (psy) {
		rc = power_supply_get_property(psy, prop, &ret);
		if (rc) {
			pr_err("[BATT] failed to retrieve value(%d) rc=%d\n",
			       prop, rc);
			return -EINVAL;
		}
	} else {
		pr_err("[BATT] psy is null.\n");
		return -EINVAL;
	}

	return ret.intval;
}

static void batt_update_info_from_charger(void)
{
	htc_batt_info.prev.batt_temp = htc_batt_info.rep.batt_temp;

	htc_batt_info.rep.batt_current =
		get_property(htc_batt_info.bms_psy,
			     POWER_SUPPLY_PROP_CURRENT_NOW);

	htc_batt_info.rep.batt_vol =
		get_property(htc_batt_info.bms_psy,
			     POWER_SUPPLY_PROP_VOLTAGE_NOW) / 1000;

	htc_batt_info.rep.batt_temp =
		get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_TEMP);

	htc_batt_info.vbus =
		get_property(htc_batt_info.usb_psy,
			     POWER_SUPPLY_PROP_VOLTAGE_NOW);

	htc_batt_info.rep.status =
		get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_STATUS);

	htc_batt_info.rep.health =
		get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_HEALTH);
}

static void batt_update_info_from_gauge(void)
{
	htc_batt_info.rep.level = get_property(htc_batt_info.batt_psy,
					       POWER_SUPPLY_PROP_CAPACITY);
}

void update_htc_chg_src(void)
{
/* In bootable/offmode_charging/offmode_charging.c
 * The charging_source type is set as below,
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC,
	CHARGER_9VAC,
	CHARGER_WIRELESS,
	CHARGER_MHL_AC,
	CHARGER_DETECTING,
	CHARGER_UNKNOWN_USB,
	CHARGER_PQM_FASTCHARGE,

*/
	int chg_src = 0;

	switch (htc_batt_info.rep.charging_source) {
	case POWER_SUPPLY_TYPE_UNKNOWN:
	case POWER_SUPPLY_TYPE_BATTERY:
		chg_src = 0;
		break;
	case POWER_SUPPLY_TYPE_USB:
	case POWER_SUPPLY_TYPE_USB_CDP:
		chg_src = 1; /* USB */
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
	case POWER_SUPPLY_TYPE_USB_ACA:
	case POWER_SUPPLY_TYPE_USB_HVDCP:
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
	case POWER_SUPPLY_TYPE_USB_PD:
	case POWER_SUPPLY_TYPE_TYPEC:
		chg_src = 2; /* AC */
		break;
	case POWER_SUPPLY_TYPE_WIRELESS:
		chg_src = 4; /* WIRELESS */
		break;
	default:
		chg_src = 9;
		break;
	}
	htc_batt_info.rep.chg_src = chg_src;
}

int htc_batt_schedule_batt_info_update(void)
{
	if (!g_htc_battery_probe_done)
		return 1;

	if (!work_pending(&htc_batt_timer.batt_work)) {
		wake_lock(&htc_batt_timer.battery_lock);
		queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
	}
	return 0;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;

		switch (*blank) {
		case FB_BLANK_UNBLANK:
			htc_batt_info.state &= ~STATE_SCREEN_OFF;
			BATT_LOG("%s-> display is On", __func__);
			htc_batt_schedule_batt_info_update();
			break;
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			htc_batt_info.state |= STATE_SCREEN_OFF;
			BATT_LOG("%s-> display is Off", __func__);
			htc_batt_schedule_batt_info_update();
			break;
		}
	}

	return 0;
}

static void batt_worker(struct work_struct *work)
{
	static int s_first = 1;
	static int s_prev_pwrsrc_enabled = 1;
	int pwrsrc_enabled = s_prev_pwrsrc_enabled;
	int charging_enabled = gs_prev_charging_enabled;
	int src = 0, present = 0;
	unsigned long time_since_last_update_ms;
	unsigned long cur_jiffies;

	/* STEP 1: print out and reset total_time since last update */
	cur_jiffies = jiffies;
	time_since_last_update_ms = htc_batt_timer.total_time_ms +
		((cur_jiffies - htc_batt_timer.batt_system_jiffies) *
		 MSEC_PER_SEC / HZ);
	BATT_DEBUG("%s: total_time since last batt update = %lu ms.\n",
		   __func__, time_since_last_update_ms);
	htc_batt_timer.total_time_ms = 0; /* reset total time */
	htc_batt_timer.batt_system_jiffies = cur_jiffies;

	/* STEP 2: setup next batt uptate timer (can put in the last step)*/
	del_timer_sync(&htc_batt_timer.batt_timer);
	batt_set_check_timer(htc_batt_timer.time_out);

	/* STEP 3: update charging_source */
	htc_batt_info.prev.charging_source = htc_batt_info.rep.charging_source;
	present = get_property(htc_batt_info.usb_psy,
			       POWER_SUPPLY_PROP_PRESENT);
	if (present)
		htc_batt_info.rep.charging_source =
				get_property(htc_batt_info.usb_psy,
					     POWER_SUPPLY_PROP_TYPE);
	else
		htc_batt_info.rep.charging_source = POWER_SUPPLY_TYPE_UNKNOWN;

	/* STEP 4: fresh battery information from gauge/charger */
	batt_update_info_from_gauge();
	batt_update_info_from_charger();
	update_htc_chg_src();

	if (htc_batt_info.rep.charging_source > POWER_SUPPLY_TYPE_UNKNOWN &&
	    htc_batt_info.rep.status != POWER_SUPPLY_STATUS_FULL)
		wake_lock(&htc_batt_info.charger_exist_lock);
	else
		wake_unlock(&htc_batt_info.charger_exist_lock);

	/* STEP 5: set the charger contorl depends on current status
	 * if charging source exist, determine charging_enable
	 */
	if ((int)htc_batt_info.rep.charging_source >
	    POWER_SUPPLY_TYPE_BATTERY) {
		/* STEP 5.1 determin charging_eanbled for charger control */
		if (g_chg_dis_reason)
			charging_enabled = 0;
		else
			charging_enabled = 1;

		/* STEP 5.2 determin pwrsrc_eanbled for charger control */
		if (g_pwrsrc_dis_reason)
			pwrsrc_enabled = 0;
		else
			pwrsrc_enabled = 1;

	} else {
		if ((htc_batt_info.prev.charging_source !=
		     htc_batt_info.rep.charging_source) ||
		     s_first) {
			charging_enabled = 0;
			pwrsrc_enabled = 0;
		}
	}

	gs_prev_charging_enabled = charging_enabled;
	s_prev_pwrsrc_enabled = pwrsrc_enabled;
	s_first = 0;

	if (g_pwrsrc_dis_reason || g_chg_dis_reason) {
		htc_batt_info.rep.chg_en = 0;
	} else {
		src = htc_batt_info.rep.charging_source;
		if (src == POWER_SUPPLY_TYPE_UNKNOWN)
			htc_batt_info.rep.chg_en = 0;
		else if (src == POWER_SUPPLY_TYPE_USB ||
			 src == POWER_SUPPLY_TYPE_USB_CDP)
			htc_batt_info.rep.chg_en = 1;
		else
			htc_batt_info.rep.chg_en = 2;
	}

	BATT_EMBEDDED("ID=%d, level=%d, vol=%dmV, temp=%d, current(mA)=%d\n",
		      htc_batt_info.rep.batt_id,
		      htc_batt_info.rep.level,
		      htc_batt_info.rep.batt_vol,
		      htc_batt_info.rep.batt_temp,
		      (htc_batt_info.rep.batt_current / 1000));
	BATT_LOG(" chg_name=%s, chg_src=%d, chg_en=%d, health=%d\n",
		 g_chr_src[htc_batt_info.rep.charging_source],
		 htc_batt_info.rep.chg_src,
		 htc_batt_info.rep.chg_en,
		 htc_batt_info.rep.health);
	BATT_LOG(" vbus(mV)=%d, MAX_IUSB(mA)=%d, MAX_PD_IUSB(mA)=%d\n",
		 (htc_batt_info.vbus / 1000),
		 get_property(htc_batt_info.usb_psy,
			      POWER_SUPPLY_PROP_CURRENT_MAX) / 1000,
		 get_property(htc_batt_info.usb_psy,
			      POWER_SUPPLY_PROP_PD_CURRENT_MAX) / 1000);
	BATT_LOG(" MAX_IBAT(mA)=%d, iusb_now(mA)=%d, AICL=%d\n",
		 get_property(
			htc_batt_info.batt_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX) / 1000,
		 get_property(htc_batt_info.usb_psy,
			      POWER_SUPPLY_PROP_INPUT_CURRENT_NOW) / 1000,
		 get_property(htc_batt_info.usb_psy,
			      POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED) / 1000);
	BATT_LOG(" batt_id(Kohms)=%d, status=%d, pwrsrc_dis_reason=%d\n",
		 get_property(htc_batt_info.bms_psy,
			      POWER_SUPPLY_PROP_RESISTANCE_ID) / 1000,
		 htc_batt_info.rep.status,
		 g_pwrsrc_dis_reason);
	BATT_LOG(" chg_dis_reason=%d, batt_state=%d, charger_temp=%d\n",
		 g_chg_dis_reason,
		 htc_batt_info.state,
		 htc_batt_info.rep.chg_src ?
		 get_property(htc_batt_info.batt_psy,
			      POWER_SUPPLY_PROP_CHARGER_TEMP) : -1);
	BATT_LOG(" CC_uAh=%d\n",
		 get_property(htc_batt_info.bms_psy,
			      POWER_SUPPLY_PROP_CHARGE_NOW_RAW)
		);

	wake_unlock(&htc_batt_timer.battery_lock);
}

static void htc_battery_update_work(struct work_struct *work)
{
	int intval = 0, present = 0, latest_chg_src = 0;
	bool info_update = false;

	/* POWER_SUPPLY_PROP_TYPE */
	present = get_property(htc_batt_info.usb_psy,
			       POWER_SUPPLY_PROP_PRESENT);
	if (present >= 0) {
		if (present)
			latest_chg_src = get_property(htc_batt_info.usb_psy,
						      POWER_SUPPLY_PROP_TYPE);
		else
			latest_chg_src = POWER_SUPPLY_TYPE_UNKNOWN;
		if (htc_batt_info.rep.charging_source != latest_chg_src)
			info_update = true;
	}

	/* POWER_SUPPLY_PROP_STATUS */
	intval = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_STATUS);
	if ((intval >= 0) && (htc_batt_info.rep.status != intval))
		info_update = true;

	/* POWER_SUPPLY_PROP_CAPACITY */
	intval = get_property(htc_batt_info.batt_psy,
			      POWER_SUPPLY_PROP_CAPACITY);
	if ((intval >= 0) && (htc_batt_info.rep.level != intval))
		info_update = true;

	/* POWER_SUPPLY_PROP_HEALTH */
	intval = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_HEALTH);
	if ((intval >= 0) && (htc_batt_info.rep.health != intval))
		info_update = true;

	/* Update batt_worker */
	if (info_update)
		htc_batt_schedule_batt_info_update();
}

static int htc_notifier_batt_callback(struct notifier_block *nb,
				      unsigned long ev, void *v)
{
	struct power_supply *psy = v;

	if (ev != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (!strcmp(psy->desc->name, "battery") ||
	    !strcmp(psy->desc->name, "usb")) {
		if (work_pending(&htc_batt_info.batt_update_work))
			return NOTIFY_OK;
		schedule_work(&htc_batt_info.batt_update_work);
	}

	return NOTIFY_OK;
}

#define WALLEYE_BATT_ID_1	"walleye 1"
#define WALLEYE_BATT_ID_2	"walleye 2"
#define MUSKIE_BATT_ID_1	"muskie 1"
#define MUSKIE_BATT_ID_2	"muskie 2"
#define LOADING_BATT_TYPE	"Loading Battery"
static int htc_battery_probe_process(void)
{
	union power_supply_propval ret = {0, };
	int rc = 0;

	htc_batt_info.batt_psy = power_supply_get_by_name("battery");
	if (!htc_batt_info.batt_psy)
		return -EPROBE_DEFER;
	htc_batt_info.bms_psy = power_supply_get_by_name("bms");
	if (!htc_batt_info.bms_psy)
		return -EPROBE_DEFER;
	htc_batt_info.usb_psy = power_supply_get_by_name("usb");
	if (!htc_batt_info.usb_psy)
		return -EPROBE_DEFER;

	htc_batt_info.htc_batt_cb.notifier_call = htc_notifier_batt_callback;
	rc = power_supply_reg_notifier(&htc_batt_info.htc_batt_cb);
	if (rc < 0) {
		BATT_ERR("Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	rc = power_supply_get_property(htc_batt_info.bms_psy,
				       POWER_SUPPLY_PROP_BATTERY_TYPE, &ret);
	if (rc < 0) {
		BATT_ERR("Unable to read battery-type rc=%d\n", rc);
		htc_batt_info.rep.batt_id = BATT_ID_UNKNOWN;
	} else {
		if (!strncmp(ret.strval,
			     LOADING_BATT_TYPE, sizeof(LOADING_BATT_TYPE)))
			return -EPROBE_DEFER;
		else if ((!strncmp(ret.strval,
				   WALLEYE_BATT_ID_1,
				   sizeof(WALLEYE_BATT_ID_1))) ||
			 (!strncmp(ret.strval,
				   MUSKIE_BATT_ID_1, sizeof(MUSKIE_BATT_ID_1))))
			htc_batt_info.rep.batt_id = BATT_ID_1;
		else if ((!strncmp(ret.strval,
				   WALLEYE_BATT_ID_2,
				   sizeof(WALLEYE_BATT_ID_2))) ||
			 (!strncmp(ret.strval,
				   MUSKIE_BATT_ID_2, sizeof(MUSKIE_BATT_ID_2))))
			htc_batt_info.rep.batt_id = BATT_ID_2;
		else
			htc_batt_info.rep.batt_id = BATT_ID_UNKNOWN;
	}

	BATT_LOG("Probe process done.\n");

	return 0;
}

static void batt_regular_timer_handler(unsigned long data)
{
	htc_batt_schedule_batt_info_update();
}

static enum alarmtimer_restart
batt_check_alarm_handler(struct alarm *alarm, ktime_t time)
{
	/* BATT_LOG("alarm handler, but do nothing."); */
	return 0;
}

static int htc_battery_prepare(struct device *dev)
{
	ktime_t interval;
	struct timespec xtime;
	unsigned long cur_jiffies;
	s64 next_alarm_sec = 0;
	int check_time = 0;

	xtime = CURRENT_TIME;
	cur_jiffies = jiffies;
	htc_batt_timer.total_time_ms += (cur_jiffies -
			htc_batt_timer.batt_system_jiffies) * MSEC_PER_SEC / HZ;
	htc_batt_timer.batt_system_jiffies = cur_jiffies;
	htc_batt_timer.batt_suspend_ms = xtime.tv_sec * MSEC_PER_SEC +
					xtime.tv_nsec / NSEC_PER_MSEC;

	check_time = BATT_SUSPEND_CHECK_TIME;

	interval = ktime_set(check_time - htc_batt_timer.total_time_ms / 1000,
			     0);
	next_alarm_sec = div_s64(interval.tv64, NSEC_PER_SEC);

	/* check if alarm is over time or in 1 second near future */
	if (next_alarm_sec <= 1) {
		BATT_LOG(
			"%s: passing time:%lu ms, trigger batt_work immediately.\n",
			__func__, htc_batt_timer.total_time_ms);
		htc_batt_schedule_batt_info_update();

		return -EBUSY;
	}

	BATT_EMBEDDED(
		"%s: passing time:%lu ms, alarm will be triggered after %lld sec\n",
		__func__, htc_batt_timer.total_time_ms, next_alarm_sec);
	BATT_LOG("htc_batt_info.state=0x%x\n", htc_batt_info.state);

	return 0;
}

static void htc_battery_complete(struct device *dev)
{
	struct timespec xtime;
	unsigned long resume_ms;
	unsigned long sr_time_period_ms;
	unsigned long check_time;
	int batt_vol;

	xtime = CURRENT_TIME;
	htc_batt_timer.batt_system_jiffies = jiffies;
	resume_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;
	sr_time_period_ms = resume_ms - htc_batt_timer.batt_suspend_ms;
	htc_batt_timer.total_time_ms += sr_time_period_ms;

	BATT_EMBEDDED("%s: sr_time_period=%lu ms; total passing time=%lu ms.",
		      __func__,
		      sr_time_period_ms, htc_batt_timer.total_time_ms);

	check_time = BATT_SUSPEND_CHECK_TIME * MSEC_PER_SEC;

	check_time -= CHECH_TIME_TOLERANCE_MS;

	/*
	 * When kernel resumes, battery driver should check total time to
	 * decide if do battery information update or just ignore.
	 */
	batt_vol = get_property(htc_batt_info.bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW) / 1000;
	if ((htc_batt_timer.total_time_ms >= check_time) || (batt_vol < 3400)) {
		BATT_LOG("trigger batt_work while resume. (batt_vol=%d)\n",
			 batt_vol);
		htc_batt_schedule_batt_info_update();
	}
}

static const struct dev_pm_ops htc_battery_pm_ops = {
	.prepare = htc_battery_prepare,
	.complete = htc_battery_complete,
};

static int htc_battery_fb_register(void)
{
	int rc = 0;

	BATT_LOG("%s in", __func__);
	htc_batt_info.fb_notif.notifier_call = fb_notifier_callback;
	rc = fb_register_client(&htc_batt_info.fb_notif);
	if (rc < 0) {
		BATT_ERR("[warning]:Unable to register fb_notifier: %d\n", rc);
		return rc;
	}
	return 0;
}

static int htc_battery_probe(struct platform_device *pdev)
{
	int rc = 0;

	INIT_WORK(&htc_batt_timer.batt_work, batt_worker);
	INIT_WORK(&htc_batt_info.batt_update_work, htc_battery_update_work);
	init_timer(&htc_batt_timer.batt_timer);
	htc_batt_timer.batt_timer.function = batt_regular_timer_handler;
	alarm_init(&htc_batt_timer.batt_check_wakeup_alarm, ALARM_REALTIME,
		   batt_check_alarm_handler);
	htc_batt_timer.batt_wq = create_singlethread_workqueue("batt_timer");

	htc_batt_timer.time_out = BATT_TIMER_UPDATE_TIME;
	batt_set_check_timer(htc_batt_timer.time_out);

	rc = htc_battery_probe_process();
	if (rc < 0)
		return rc;

	rc = htc_battery_fb_register();
	if (rc < 0)
		return rc;

	g_htc_battery_probe_done = true;

	return 0;
}

static struct platform_device htc_battery_pdev = {
	.name = HTC_BATT_NAME,
	.id = -1,
};

static struct platform_driver htc_battery_driver = {
	.probe	= htc_battery_probe,
	.driver	= {
		.name	= HTC_BATT_NAME,
		.owner	= THIS_MODULE,
		.pm = &htc_battery_pm_ops,
	},
};

static int __init htc_battery_init(void)
{
	wake_lock_init(&htc_batt_timer.battery_lock,
		       WAKE_LOCK_SUSPEND, "htc_battery");
	wake_lock_init(&htc_batt_info.charger_exist_lock,
		       WAKE_LOCK_SUSPEND, "charger_exist_lock");

	/* init battery parameters. */
	htc_batt_info.rep.batt_vol = 4000;
	htc_batt_info.rep.batt_id = 1;
	htc_batt_info.rep.batt_temp = 280;
	htc_batt_info.rep.batt_current = 0;
	htc_batt_info.rep.charging_source = POWER_SUPPLY_TYPE_UNKNOWN;
	htc_batt_info.rep.level = 33;
	htc_batt_info.rep.level_raw = 33;
	htc_batt_info.rep.full_level = 100;
	htc_batt_info.rep.status = POWER_SUPPLY_STATUS_UNKNOWN;
	htc_batt_info.rep.full_level_dis_batt_chg = 100;
	htc_batt_info.rep.overload = 0;
	htc_batt_info.rep.over_vchg = 0;
	htc_batt_info.rep.is_full = false;
	htc_batt_info.rep.health = POWER_SUPPLY_HEALTH_UNKNOWN;
	htc_batt_info.smooth_chg_full_delay_min = 1;
	htc_batt_info.decreased_batt_level_check = 1;
	htc_batt_info.critical_low_voltage_mv = 3200;
	htc_batt_info.batt_full_voltage_mv = 4350;
	htc_batt_info.batt_full_current_ma = 300;
	htc_batt_info.overload_curr_thr_ma = 0;
	htc_batt_info.batt_fcc_ma = 3000;
	htc_batt_info.batt_capacity_mah = 3000;
	htc_batt_info.batt_thermal_limit_vol = 4200;
	htc_batt_info.vbus = 0;

	platform_device_register(&htc_battery_pdev);
	platform_driver_register(&htc_battery_driver);

	BATT_LOG("htc_battery_init done.\n");

	return 0;
}

static void __exit htc_battery_exit(void)
{
	platform_device_unregister(&htc_battery_pdev);
	platform_driver_unregister(&htc_battery_driver);
}

module_init(htc_battery_init);
module_exit(htc_battery_exit);
MODULE_LICENSE("GPL");
