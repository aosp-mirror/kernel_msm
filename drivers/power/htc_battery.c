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

/*
 * Protects access to the following static data:
 *
 * htc_batt_info
 * htc_batt_timer
 * g_htc_battery_probe_done
 */
DEFINE_MUTEX(htc_battery_lock);

#define DEFAULT_CHARGE_STOP_LEVEL 100
#define DEFAULT_CHARGE_START_LEVEL 0

static int charge_stop_level = DEFAULT_CHARGE_STOP_LEVEL;
static int charge_start_level = DEFAULT_CHARGE_START_LEVEL;

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
	int next_temp;
	int recover_temp;
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
	int v_elvdd_dis_en;
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
	unsigned int time_out;
};

static struct htc_battery_info htc_batt_info;
static struct htc_battery_timer htc_batt_timer;

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

/* accesses htc_batt_timer, needs htc_battery_lock */
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

/* accesses htc_batt_info, needs htc_battery_lock */
static int set_batt_psy_property(enum power_supply_property prop, int value)
{
	union power_supply_propval ret = {0, };
	int rc = -1;

	if (htc_batt_info.batt_psy) {
		BATT_EMBEDDED("%s value(%d) prop(%d)",
			      __func__, value, prop);
		ret.intval = value;
		rc = power_supply_set_property(htc_batt_info.batt_psy,
					       prop, &ret);
	}

	return rc;
}

/* accesses htc_batt_info, needs htc_battery_lock */
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

/* accesses htc_batt_info, needs htc_battery_lock */
static void batt_update_info_from_gauge(void)
{
	htc_batt_info.rep.level = get_property(htc_batt_info.batt_psy,
					       POWER_SUPPLY_PROP_CAPACITY);
}

/* accesses htc_batt_info, needs htc_battery_lock */
static int is_bounding_fully_charged_level(void)
{
	static int s_pingpong = 1;
	int is_batt_chg_off_by_bounding = 0;
	int upperbd = charge_stop_level;
	int lowerbd = charge_start_level;
	int current_level = htc_batt_info.rep.level;

	if ((upperbd == DEFAULT_CHARGE_STOP_LEVEL) &&
	    (lowerbd == DEFAULT_CHARGE_START_LEVEL))
		return 0;

	if ((upperbd > lowerbd) &&
	    (upperbd <= DEFAULT_CHARGE_STOP_LEVEL) &&
	    (lowerbd >= DEFAULT_CHARGE_START_LEVEL)) {
		if (lowerbd < 0)
			lowerbd = 0;

		if (s_pingpong == 1 && upperbd <= current_level) {
			BATT_LOG(
				"%s: lowerbd=%d, upperbd=%d, current=%d, pingpong:1->0 turn off\n",
				__func__, lowerbd, upperbd, current_level);
			is_batt_chg_off_by_bounding = 1;
			s_pingpong = 0;
		} else if (s_pingpong == 0 && lowerbd < current_level) {
			BATT_LOG(
				"%s: lowerbd=%d, upperbd=%d, current=%d, toward 0, turn off\n",
				__func__, lowerbd, upperbd, current_level);
			is_batt_chg_off_by_bounding = 1;
		} else if (s_pingpong == 0 && current_level <= lowerbd) {
			BATT_LOG(
				"%s: lowerbd=%d, upperbd=%d, current=%d, pingpong:0->1 turn on\n",
				__func__, lowerbd, upperbd, current_level);
			s_pingpong = 1;
		} else {
			BATT_LOG(
				"%s: lowerbd=%d, upperbd=%d, current=%d, toward %d, turn on\n",
				__func__, lowerbd, upperbd, current_level,
				s_pingpong);
		}
	}

	return is_batt_chg_off_by_bounding;
}

/* accesses htc_batt_info, needs htc_battery_lock */
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

#define HYST_MV	200
#define MAX_IDX		4
#define HEALTH_LEVELS	6
static int ibat_map_ac[MAX_IDX][HEALTH_LEVELS] = {
/* IBAT_IDX =  < VOLTAGE-LIMITED, DISPLAY-ON > */
/*  #0 (00) */ {   30,   50,   100,   80,   50,   50},
/*  #1 (01) */ {   30,   50,   100,   50, 1000, 1000},
/*  #2 (10) */ {   15,   30,    50,   50,    0,    0},
/*  #3 (11) */ {   15,   30,    50,   50,    0,    0},
};

static struct htc_thermal_stage thermal_stage[] = {
	{	-INT_MAX,	120},
	{	100,		220},
	{	420,		200},	/* Good */
	{	450,		400},
	{	480,		430},
	{	INT_MAX,	460}
};

static int thermal_limit_vol[] = {
	4200, 4200, 4200, 4200, 4300, 4100 };

enum {
	HEALTH_COOL2 = 0,
	HEALTH_COOL1,
	HEALTH_GOOD,
	HEALTH_WARM1,
	HEALTH_WARM2,
	HEALTH_WARM3,
};

/* accesses htc_batt_info, needs htc_battery_lock */
static int update_ibat_setting(void)
{
	static int batt_thermal = HEALTH_GOOD;
	static bool is_vol_limited;
	int idx = 0;
	bool is_screen_on = true;
	int batt_temp = htc_batt_info.rep.batt_temp;
	int batt_vol = htc_batt_info.rep.batt_vol;
	int chg_type = htc_batt_info.rep.charging_source;
	int ibat_ma = 0;

	/* Step 1: Update Health status*/
	while (1) {
		if (batt_thermal >= HEALTH_GOOD) {	/* normal & warm */
			if (batt_temp >= thermal_stage[batt_thermal].next_temp)
				batt_thermal++;
			else if (batt_temp <=
				 thermal_stage[batt_thermal].recover_temp)
				batt_thermal--;
			else
				break;
		} else {				/* cool */
			if (batt_temp <= thermal_stage[batt_thermal].next_temp)
				batt_thermal--;
			else if (batt_temp >=
				 thermal_stage[batt_thermal].recover_temp)
				batt_thermal++;
			else
				break;
		}
	}
	/* Step 2: Update Voltage status */
	if (is_vol_limited || batt_vol > thermal_limit_vol[batt_thermal])
		is_vol_limited = true;
	if (!is_vol_limited ||
	    batt_vol < (thermal_limit_vol[batt_thermal] - HYST_MV))
		is_vol_limited = false;

	/* Step 3: Apply Screen ON configuration */
	is_screen_on = !(htc_batt_info.state & STATE_SCREEN_OFF);

	/* Step 4: Get mapping index */
	idx = ((2 * (is_vol_limited ? 1 : 0)) +
		(1 * (is_screen_on ? 1 : 0)));

	switch (chg_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
	case POWER_SUPPLY_TYPE_USB_HVDCP:
	case POWER_SUPPLY_TYPE_USB_DCP:
	case POWER_SUPPLY_TYPE_USB_PD:
		if (ibat_map_ac[idx][batt_thermal] <= 100)
			ibat_ma = htc_batt_info.batt_fcc_ma *
					ibat_map_ac[idx][batt_thermal] / 100;
		else
			ibat_ma = ibat_map_ac[idx][batt_thermal];
		break;
	default:
		ibat_ma = htc_batt_info.batt_fcc_ma;
		break;
	}

	ibat_ma = (ibat_ma / 25) * 25;

	if (ibat_ma * 1000 !=
		get_property(
			htc_batt_info.batt_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX)) {
		BATT_LOG(
			"%s: thermal=%d,temp=%d,fcc=%d,is_vol_limited(>%dmV)=%d,is_screen_on=%d,idx=%d,ibat_ma=%d.\n",
			__func__, batt_thermal, batt_temp,
			htc_batt_info.batt_fcc_ma,
			thermal_limit_vol[batt_thermal], is_vol_limited,
			is_screen_on, idx, ibat_ma);
	}

	if (g_chg_dis_reason)
		ibat_ma = 0;

	return ibat_ma * 1000;
}

/*
 * Accesses htc_batt_timer and g_htc_battery_probe_done.
 *
 * Caller must hold htc_battery_lock.
 */
int htc_batt_schedule_batt_info_update(void)
{
	if (!g_htc_battery_probe_done)
		return 1;

	if (!work_pending(&htc_batt_timer.batt_work))
		queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
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
	static int s_chg_present;
	int pwrsrc_enabled = s_prev_pwrsrc_enabled;
	int charging_enabled = gs_prev_charging_enabled;
	int src = 0, online = 0, chg_present = 0, ex_otg = 0;
	int ibat = 0;
	int ibat_new = 0;
	unsigned long time_since_last_update_ms;
	unsigned long cur_jiffies;

	mutex_lock(&htc_battery_lock);

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
	online = get_property(htc_batt_info.usb_psy,
			      POWER_SUPPLY_PROP_ONLINE);
	if (online)
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
	chg_present = get_property(htc_batt_info.usb_psy,
				   POWER_SUPPLY_PROP_PRESENT);
	ex_otg = get_property(htc_batt_info.usb_psy,
			      POWER_SUPPLY_PROP_USE_EXTERNAL_VBUS_OUTPUT);
	if (((int)htc_batt_info.rep.charging_source >
	    POWER_SUPPLY_TYPE_BATTERY) ||
	    (chg_present && !ex_otg)) {
		if (is_bounding_fully_charged_level())
			g_chg_dis_reason |= HTC_BATT_CHG_DIS_BIT_MFG;
		else
			g_chg_dis_reason &= ~HTC_BATT_CHG_DIS_BIT_MFG;
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

		/* STEP 5.3 check ibat setting */
		ibat = get_property(
				htc_batt_info.batt_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX);
		ibat_new = update_ibat_setting();

		if (ibat != ibat_new) {
			BATT_EMBEDDED("set ibat(%d)", ibat_new);
			set_batt_psy_property(
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				ibat_new);
		}

		if (s_chg_present != chg_present) {
			BATT_EMBEDDED(
				     "set pwrsrc_enable(%d) when plug-in",
				     pwrsrc_enabled);
			set_batt_psy_property(
					     POWER_SUPPLY_PROP_INPUT_SUSPEND,
					     !pwrsrc_enabled);
		} else if (pwrsrc_enabled != s_prev_pwrsrc_enabled) {
			BATT_EMBEDDED("set pwrsrc_enable(%d)", pwrsrc_enabled);
			set_batt_psy_property(
					     POWER_SUPPLY_PROP_INPUT_SUSPEND,
					     !pwrsrc_enabled);
		}

		if (charging_enabled != gs_prev_charging_enabled) {
			BATT_EMBEDDED("set charging_enable(%d)",
				      charging_enabled);
			set_batt_psy_property(
					     POWER_SUPPLY_PROP_CHARGE_DISABLE,
					     !charging_enabled);
		}
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
	s_chg_present = chg_present;
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
	BATT_LOG(" CC_uAh=%d, otg=%d\n",
		 get_property(htc_batt_info.bms_psy,
			      POWER_SUPPLY_PROP_CHARGE_NOW_RAW),
		 get_property(htc_batt_info.usb_psy,
			      POWER_SUPPLY_PROP_VBUS_OUTPUT_STATUS)
		);

	/* WA for display flickering */
	if (htc_batt_info.v_elvdd_dis_en > 0 &&
	    gpio_is_valid(htc_batt_info.v_elvdd_dis_en)) {
		if ((int)htc_batt_info.rep.charging_source >
		    POWER_SUPPLY_TYPE_BATTERY) {
			if (htc_batt_info.rep.level == 100)
				gpio_direction_output(
					htc_batt_info.v_elvdd_dis_en, 1);
			else
				gpio_direction_output(
					htc_batt_info.v_elvdd_dis_en, 0);
		} else {
			gpio_direction_output(htc_batt_info.v_elvdd_dis_en, 0);
		}
	}
	BATT_LOG(" v_elvdd_dis_en=%d\n",
		 gpio_get_value(htc_batt_info.v_elvdd_dis_en));

	mutex_unlock(&htc_battery_lock);
}

static void htc_battery_update_work(struct work_struct *work)
{
	int intval = 0, present = 0, latest_chg_src = 0;
	bool info_update = false;

	mutex_lock(&htc_battery_lock);

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

	mutex_unlock(&htc_battery_lock);
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

static int set_full_level_dis_chg(const char *val,
				  const struct kernel_param *kp)
{
	int rc;
	int old_val = charge_stop_level;

	rc = param_set_int(val, kp);
	if (rc) {
		BATT_ERR("Unable to set charge_stop_level: %d\n", rc);
		return rc;
	}

	if (charge_stop_level != old_val) {
		charge_start_level = charge_stop_level - 5;
		htc_batt_schedule_batt_info_update();
	}

	return 0;
}

static struct kernel_param_ops disable_charge_ops = {
	.set = set_full_level_dis_chg,
	.get = param_get_int,
};
module_param_cb(full_level_dis_chg, &disable_charge_ops,
		&charge_stop_level, 0644);

static int set_charge_stop_level(const char *val,
				 const struct kernel_param *kp)
{
	int rc;
	int old_val = charge_stop_level;

	rc = param_set_int(val, kp);
	if (rc) {
		BATT_ERR("Unable to set charge_stop_level: %d\n", rc);
		return rc;
	}

	if (charge_stop_level != old_val)
		htc_batt_schedule_batt_info_update();

	return 0;
}

static struct kernel_param_ops charge_stop_ops = {
	.set = set_charge_stop_level,
	.get = param_get_int,
};
module_param_cb(charge_stop_level, &charge_stop_ops,
		&charge_stop_level, 0644);

static int set_charge_start_level(const char *val,
				  const struct kernel_param *kp)
{
	int rc;
	int old_val = charge_start_level;

	rc = param_set_int(val, kp);
	if (rc) {
		BATT_ERR("Unable to set charge_start_level: %d\n", rc);
		return rc;
	}

	if (charge_start_level != old_val)
		htc_batt_schedule_batt_info_update();

	return 0;
}

static struct kernel_param_ops charge_start_ops = {
	.set = set_charge_start_level,
	.get = param_get_int,
};
module_param_cb(charge_start_level, &charge_start_ops,
		&charge_start_level, 0644);

#define WALLEYE_BATT_ID_1	"walleye 1"
#define WALLEYE_BATT_ID_2	"walleye 2"
#define MUSKIE_BATT_ID_1	"muskie 1"
#define MUSKIE_BATT_ID_2	"muskie 2"
#define LOADING_BATT_TYPE	"Loading Battery"
/* accesses htc_batt_info, needs htc_battery_lock */
static int htc_battery_probe_process(void)
{
	union power_supply_propval ret = {0, };
	int rc = 0, id_kohms = 0;
	struct device_node *node;
	struct device_node *profile_node;

	htc_batt_info.batt_psy = power_supply_get_by_name("battery");
	if (!htc_batt_info.batt_psy)
		return -EPROBE_DEFER;
	htc_batt_info.bms_psy = power_supply_get_by_name("bms");
	if (!htc_batt_info.bms_psy)
		return -EPROBE_DEFER;
	htc_batt_info.usb_psy = power_supply_get_by_name("usb");
	if (!htc_batt_info.usb_psy)
		return -EPROBE_DEFER;

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

	id_kohms = get_property(
			htc_batt_info.bms_psy,
			POWER_SUPPLY_PROP_RESISTANCE_ID) / 1000;
	node = of_find_node_by_name(NULL, "qcom,battery-data");
	if (!node) {
		BATT_LOG("%s: No batterydata available\n", __func__);
	} else {
		profile_node = of_batterydata_get_best_profile(
					node, id_kohms, NULL);
		if (!profile_node) {
			BATT_LOG(
				"%s: couldn't find profile handle\n",
				__func__);
		} else {
			rc = of_property_read_u32(
					profile_node,
					"htc,fastchg-current-ma",
					&htc_batt_info.batt_fcc_ma);
			if (rc < 0) {
				BATT_LOG(
					"%s: error reading htc,fastchg-current-ma. %d\n",
					__func__, rc);
				htc_batt_info.batt_fcc_ma = 2600;
			}
			/* read battery design capacity from DT, unit is mAh */
			rc = of_property_read_u32(
					profile_node,
					"qcom,nom-batt-capacity-mah",
					&htc_batt_info.batt_capacity_mah);
			if (rc < 0) {
				BATT_LOG(
					"%s: error reading qcom,nom-batt-capacity-mah. %d\n",
					__func__, rc);
				htc_batt_info.batt_capacity_mah = 2600;
			}
			/*
			 * read battery thermal limit threshold voltage from DT,
			 * unit is mV
			 */
			rc = of_property_read_u32(
					profile_node,
					"qcom,batt-thermal-limit-vol",
					&htc_batt_info.batt_thermal_limit_vol);
			if (rc < 0) {
				BATT_LOG(
					"%s: error reading qcom,batt-thermal-limit-vol. %d\n",
					__func__, rc);
				htc_batt_info.batt_thermal_limit_vol = 4200;
			}
		}
	}
	BATT_LOG("%s: catch name %s, set batt id=%d, fcc_ma=%d, capacity=%d\n",
		 __func__, ret.strval, htc_batt_info.rep.batt_id,
		 htc_batt_info.batt_fcc_ma, htc_batt_info.batt_capacity_mah);

	/* WA for display flickering */
	node = of_find_node_by_name(NULL, "htc,battery-node");
	if (!node) {
		BATT_LOG("%s: No htc,battery-node available\n", __func__);
	} else {
		htc_batt_info.v_elvdd_dis_en =
			of_get_named_gpio(node, "htc,v-elvdd-dis-en", 0);
		if (!gpio_is_valid(htc_batt_info.v_elvdd_dis_en)) {
			BATT_LOG(
				"%s: error to reading htc,v-elvdd-dis-en.\n",
				__func__);
			htc_batt_info.v_elvdd_dis_en = 0;
		} else {
			BATT_LOG("%s: htc,v-elvdd-dis-en = %d\n",
				 __func__, htc_batt_info.v_elvdd_dis_en);
			rc = gpio_request(htc_batt_info.v_elvdd_dis_en,
					  "V_ELVDD_DIS_EN");
			if (rc < 0) {
				BATT_LOG(
					"%s: fail to request V_ELVDD_DIS_EN, rc=%dn",
					__func__, rc);
			} else {
				gpio_direction_output(
					htc_batt_info.v_elvdd_dis_en, 0);
			}
		}
	}

	BATT_LOG("Probe process done.\n");

	return 0;
}

/*
 * htc_batt_timer.batt_timer can only be started after htc_batt_timer.batt_wq
 * is initialized and g_htc_battery_probe_done is set to true, so the timer
 * callback is properly synchronized with code that acquires htc_battery_lock.
 *
 * This timer callback *cannot* acquire htc_battery_lock because otherwise it
 * will deadlock with the del_timer_sync() in batt_worker().
 */
static void batt_regular_timer_handler(unsigned long data)
{
	htc_batt_schedule_batt_info_update();
}

/* accesses htc_batt_info, needs htc_battery_lock */
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

	mutex_lock(&htc_battery_lock);

	rc = htc_battery_probe_process();
	if (rc < 0)
		goto err_unlock;

	INIT_WORK(&htc_batt_timer.batt_work, batt_worker);
	INIT_WORK(&htc_batt_info.batt_update_work, htc_battery_update_work);
	setup_timer(&htc_batt_timer.batt_timer, batt_regular_timer_handler, 0);
	htc_batt_timer.batt_wq = create_singlethread_workqueue("batt_timer");
	if (!htc_batt_timer.batt_wq) {
		BATT_LOG("%s: create_singlethread_workqueue failed.\n",
			 __func__);
		goto err_unlock;
	}

	htc_batt_timer.time_out = BATT_TIMER_UPDATE_TIME;

	htc_batt_info.htc_batt_cb.notifier_call = htc_notifier_batt_callback;
	rc = power_supply_reg_notifier(&htc_batt_info.htc_batt_cb);
	if (rc  < 0) {
		BATT_LOG("%s: power_supply_reg_notifier failed.\n", __func__);
		goto err_destroy_workqueue;
	}

	rc = htc_battery_fb_register();
	if (rc  < 0) {
		BATT_LOG("%s: htc_battery_fb_register failed.\n", __func__);
		goto err_power_supply_unreg_notifier;
	}

	g_htc_battery_probe_done = true;
	batt_set_check_timer(htc_batt_timer.time_out);
	goto err_unlock;

err_power_supply_unreg_notifier:
	power_supply_unreg_notifier(&htc_batt_info.htc_batt_cb);
err_destroy_workqueue:
	destroy_workqueue(htc_batt_timer.batt_wq);
err_unlock:
	mutex_unlock(&htc_battery_lock);
	return rc;
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
	},
};

static int __init htc_battery_init(void)
{
	int ret;

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

	ret = platform_device_register(&htc_battery_pdev);
	if (ret < 0) {
		BATT_LOG("%s: device registration failed.\n", __func__);
		return ret;
	}

	ret = platform_driver_register(&htc_battery_driver);
	if (ret < 0) {
		BATT_LOG("%s: driver registration failed.\n", __func__);
		return ret;
	}

	BATT_LOG("%s done.\n", __func__);

	return 0;
}

static void __exit htc_battery_exit(void)
{
	platform_device_unregister(&htc_battery_pdev);
	platform_driver_unregister(&htc_battery_driver);
	BATT_LOG("%s done.\n", __func__);
}

module_init(htc_battery_init);
module_exit(htc_battery_exit);
MODULE_LICENSE("GPL");
