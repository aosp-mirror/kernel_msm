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

#ifdef CONFIG_PM_SLEEP
/* disabled for now, will need to keep track of sleep to potentially change
 * wake mask etc.
 * #define SUPPORT_PM_SLEEP
 */
#endif

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/pmic-voter.h>
#include <linux/thermal.h>
#include "google_bms.h"
#include "google_psy.h"
#include "qmath.h"
#include "logbuffer.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#define BATT_DELAY_INIT_MS 		250
#define BATT_WORK_ERROR_RETRY_MS	1000

#define DEFAULT_BATT_FAKE_CAPACITY		50
#define DEFAULT_BATT_UPDATE_INTERVAL		30000
#define DEFAULT_BATT_DRV_RL_SOC_THRESHOLD	97

#define MSC_ERROR_UPDATE_INTERVAL		5000
#define MSC_DEFAULT_UPDATE_INTERVAL		30000

/* qual time is 15 minutes of charge or 15% increase in SOC */
#define DEFAULT_CHG_STATS_MIN_QUAL_TIME		(15 * 60)
#define DEFAULT_CHG_STATS_MIN_DELTA_SOC		15

#define UICURVE_MAX	3

#if (GBMS_CCBIN_BUCKET_COUNT < 1) || (GBMS_CCBIN_BUCKET_COUNT > 100)
#error "GBMS_CCBIN_BUCKET_COUNT needs to be a value from 1-100"
#endif

struct ssoc_uicurve {
	qnum_t real;
	qnum_t ui;
};

enum batt_rl_status {
	BATT_RL_STATUS_NONE = 0,
	BATT_RL_STATUS_DISCHARGE = -1,
	BATT_RL_STATUS_RECHARGE = 1,
};

#define SSOC_STATE_BUF_SZ 128

struct batt_ssoc_state {
	/* output of gauge data filter */
	qnum_t ssoc_gdf;
	/*  UI Curves */
	int ssoc_curve_type;    /*<0 dsg, >0 chg, 0? */
	struct ssoc_uicurve ssoc_curve[UICURVE_MAX];
	qnum_t ssoc_uic;
	/* output of rate limiter */
	qnum_t ssoc_rl;

	/* recharge logic */
	int rl_soc_threshold;
	enum batt_rl_status rl_status;

	/* buff */
	char ssoc_state_cstr[SSOC_STATE_BUF_SZ];
};

struct gbatt_ccbin_data {
	u16 count[GBMS_CCBIN_BUCKET_COUNT];
	char cyc_ctr_cstr[GBMS_CCBIN_CSTR_SIZE];
	struct mutex lock;
	int prev_soc;
};

#define DEFAULT_RES_TEMP_HIGH	390
#define DEFAULT_RES_TEMP_LOW	350
#define DEFAULT_RES_SSOC_THR	75
#define DEFAULT_RES_FILT_LEN	10

struct batt_res {
	bool estimate_requested;

	/* samples */
	int sample_accumulator;
	int sample_count;

	/* registers */
	int filter_count;
	int resistance_avg;

	/* configuration */
	int estimate_filter;
	int ssoc_threshold;
	int res_temp_low;
	int res_temp_high;
};

struct batt_drv {
	struct device *device;
	struct power_supply *psy;

	const char *fg_psy_name;
	struct power_supply *fg_psy;
	struct notifier_block fg_nb;

	struct delayed_work init_work;
	struct delayed_work batt_work;
	struct wakeup_source batt_ws;

	/* TODO: b/111407333, will likely need to adjust SOC% on wakeup */
	bool init_complete;
	bool resume_complete;
	bool batt_present;

	struct mutex batt_lock;
	struct mutex chg_lock;

	/* battery work */
	u32 batt_update_interval;
	/* triger for recharge logic next update from charger */
	bool batt_full;
	struct batt_ssoc_state ssoc_state;
	/* bin count */
	struct gbatt_ccbin_data cc_data;
	struct power_supply *ccbin_psy;

	/* props */
	int soh;
	int fake_capacity;
	int buck_enabled;

	/* temp outside the charge table */
	int jeita_stop_charging;

	/* MSC charging */
	u32 battery_capacity;
	struct gbms_chg_profile chg_profile;

	union gbms_charger_state chg_state;

	int temp_idx;
	int vbatt_idx;
	int checked_cv_cnt;
	int checked_ov_cnt;
	int checked_tier_switch_cnt;

	int fv_uv;
	int cc_max;
	int msc_update_interval;

	struct votable	*msc_interval_votable;
	struct votable	*fcc_votable;
	struct votable	*fv_votable;

	/* stats */
	int msc_state;
	struct mutex stats_lock;
	struct gbms_charging_event ce_data;
	struct gbms_charging_event ce_qual;

	/* logging */
	struct logbuffer *log;

	/* thermal */
	struct thermal_zone_device *tz_dev;

	/* Resistance */
	struct batt_res res_state;
};

static int google_battery_tz_get_cycle_count(void *data, int *cycle_count)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	struct power_supply *fg_psy = batt_drv->fg_psy;

	if (!cycle_count) {
		pr_err("Cycle Count NULL");
		return -EINVAL;
	}
	if (!fg_psy)
		*cycle_count = 0;
	else
		*cycle_count = GPSY_GET_PROP(fg_psy,
					     POWER_SUPPLY_PROP_CYCLE_COUNT);
	return 0;
}

static int psy_changed(struct notifier_block *nb,
		       unsigned long action, void *data)
{
	struct power_supply *psy = data;
	struct batt_drv *batt_drv = container_of(nb, struct batt_drv, fg_nb);

	pr_debug("name=%s evt=%lu\n", psy->desc->name, action);

	if ((action != PSY_EVENT_PROP_CHANGED) ||
	    (psy == NULL) || (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (action == PSY_EVENT_PROP_CHANGED &&
	    (!strcmp(psy->desc->name, batt_drv->fg_psy_name))) {
		cancel_delayed_work(&batt_drv->batt_work);
		schedule_delayed_work(&batt_drv->batt_work, 0);
	}

	return NOTIFY_OK;
}

/* ------------------------------------------------------------------------- */

#define SSOC_TRUE 15
#define SSOC_SPOOF 95
#define SSOC_FULL 100
#define UICURVE_BUF_SZ	(UICURVE_MAX * 15 + 1)

enum ssoc_uic_type {
	SSOC_UIC_TYPE_DSG  = -1,
	SSOC_UIC_TYPE_NONE = 0,
	SSOC_UIC_TYPE_CHG  = 1,
};

const qnum_t ssoc_point_true = qnum_rconst(SSOC_TRUE);
const qnum_t ssoc_point_spoof = qnum_rconst(SSOC_SPOOF);
const qnum_t ssoc_point_full = qnum_rconst(SSOC_FULL);

static struct ssoc_uicurve chg_curve[UICURVE_MAX] = {
	{ ssoc_point_true, ssoc_point_true },
	{ ssoc_point_spoof, ssoc_point_spoof },
	{ ssoc_point_full, ssoc_point_full },
};

static struct ssoc_uicurve dsg_curve[UICURVE_MAX] = {
	{ ssoc_point_true, ssoc_point_true },
	{ ssoc_point_spoof, ssoc_point_full },
	{ ssoc_point_full, ssoc_point_full },
};

static char *ssoc_uicurve_cstr(char *buff, size_t size,
			       struct ssoc_uicurve *curve)
{
	int i, len = 0;

	for (i = 0; i < UICURVE_MAX ; i++) {
		len += snprintf(&buff[len], size - len,
				"[" QNUM_CSTR_FMT " " QNUM_CSTR_FMT "]",
				qnum_toint(curve[i].real),
				qnum_fracdgt(curve[i].real),
				qnum_toint(curve[i].ui),
				qnum_fracdgt(curve[i].ui));
		if (len >= size)
			break;
	}

	buff[len] = 0;
	return buff;
}

/* NOTE: no bounds checks on this one */
static int ssoc_uicurve_find(qnum_t real, struct ssoc_uicurve *curve)
{
	int i;

	for (i = 1; i < UICURVE_MAX ; i++) {
		if (real == curve[i].real)
			return i;
		if (real > curve[i].real)
			continue;
		break;
	}

	return i-1;
}

static qnum_t ssoc_uicurve_map(qnum_t real, struct ssoc_uicurve *curve)
{
	qnum_t slope = 0, delta_ui, delta_re;
	int i;

	if (real < curve[0].real)
		return real;
	if (real >= curve[UICURVE_MAX - 1].ui)
		return curve[UICURVE_MAX - 1].ui;

	i = ssoc_uicurve_find(real, curve);
	if (curve[i].real == real)
		return curve[i].ui;

	delta_ui = curve[i + 1].ui - curve[i].ui;
	delta_re =  curve[i + 1].real - curve[i].real;
	if (delta_re)
		slope = qnum_div(delta_ui, delta_re);

	return curve[i].ui + qnum_mul(slope, (real - curve[i].real));
}

/* "optimized" to work on 3 element curves */
static void ssoc_uicurve_splice(struct ssoc_uicurve *curve, qnum_t real,
				qnum_t ui)
{
	if (real < curve[0].real || real > curve[2].real)
		return;

#if UICURVE_MAX != 3
#error ssoc_uicurve_splice() only support UICURVE_MAX == 3
#endif

	/* splice only when real is within the curve range */
	curve[1].real = real;
	curve[1].ui = ui;
}

static void ssoc_uicurve_dup(struct ssoc_uicurve *dst,
			     struct ssoc_uicurve *curve)
{
	if (dst != curve)
		memcpy(dst, curve, sizeof(*dst)*UICURVE_MAX);
}


/* ------------------------------------------------------------------------- */

/* TODO: b/111407333, apply rate limiter to UIC */
static qnum_t ssoc_apply_rl(struct batt_ssoc_state *ssoc)
{
	return ssoc->ssoc_uic;
}

/* ------------------------------------------------------------------------- */

/* a statement :-) */
static int ssoc_get_real(const struct batt_ssoc_state *ssoc)
{
	return qnum_toint(ssoc->ssoc_gdf);
}

static qnum_t ssoc_get_capacity_raw(struct batt_ssoc_state *ssoc)
{
	return ssoc->ssoc_rl;
}

/* reported to userspace: call while holding batt_lock */
static int ssoc_get_capacity(struct batt_ssoc_state *ssoc)
{
	const qnum_t raw = ssoc_get_capacity_raw(ssoc);
	return qnum_roundint(raw, 0.005);
}

/* ------------------------------------------------------------------------- */

void dump_ssoc_state(struct batt_ssoc_state *ssoc_state, struct logbuffer *log)
{
	char buff[UICURVE_BUF_SZ] = { 0 };

	snprintf(ssoc_state->ssoc_state_cstr,
		 sizeof(ssoc_state->ssoc_state_cstr),
		 "SSOC: l=%d%% gdf=%d.%02d uic=%d.%02d rl=%d.%02d ct=%d curve:%s rls=%d",
		 ssoc_get_capacity(ssoc_state),
		 qnum_toint(ssoc_state->ssoc_gdf),
		 qnum_fracdgt(ssoc_state->ssoc_gdf),
		 qnum_toint(ssoc_state->ssoc_uic),
		 qnum_fracdgt(ssoc_state->ssoc_uic),
		 qnum_toint(ssoc_state->ssoc_rl),
		 qnum_fracdgt(ssoc_state->ssoc_rl),
		 ssoc_state->ssoc_curve_type,
		 ssoc_uicurve_cstr(buff, sizeof(buff), ssoc_state->ssoc_curve),
		 ssoc_state->rl_status);

	if (log) {
		logbuffer_log(log, "%s", ssoc_state->ssoc_state_cstr);
	} else {
		pr_info("%s\n", ssoc_state->ssoc_state_cstr);
	}
}

/* ------------------------------------------------------------------------- */

/* call while holding batt_lock
 * NOTE: for Maxim could need:
 *	1fh AvCap
 *	10h FullCap
 *	23h FullCapNom
 */
static void ssoc_update(struct batt_ssoc_state *ssoc, qnum_t soc)
{
	/* low pass filter */
	ssoc->ssoc_gdf = soc;
	/* spoof UI @ EOC */
	ssoc->ssoc_uic = ssoc_uicurve_map(ssoc->ssoc_gdf, ssoc->ssoc_curve);
	/*  monotonicity and rate of change */
	ssoc->ssoc_rl = ssoc_apply_rl(ssoc);
}

static int ssoc_work(struct batt_ssoc_state *ssoc_state,
		     struct power_supply *fg_psy)
{
	int soc_q8_8;
	qnum_t soc_raw;

	/* TODO: POWER_SUPPLY_PROP_CAPACITY_RAW should return a qnum_t */
	soc_q8_8 = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_CAPACITY_RAW);
	if (soc_q8_8 < 0)
		return -EINVAL;

	soc_raw = qnum_from_q8_8(soc_q8_8);

	ssoc_update(ssoc_state, soc_raw);
	return 0;
}

/* Called on connect and disconnect to adjust the UI curve. no op when type is
 * the same as ssoc_curve_type and when in recharge logic.
 */
void ssoc_change_curve(struct batt_ssoc_state *ssoc_state,
		       enum ssoc_uic_type type)
{
	qnum_t ssoc_level = ssoc_get_capacity_raw(ssoc_state);
	struct ssoc_uicurve *new_curve;

	if (ssoc_state->ssoc_curve_type == type)
		return;

	/* force dsg curve when showing 100% (includes recharge logic) */
	if (ssoc_level >= ssoc_point_full)
		type = SSOC_UIC_TYPE_DSG;

	new_curve = (type == SSOC_UIC_TYPE_DSG) ? dsg_curve : chg_curve;
	ssoc_uicurve_dup(ssoc_state->ssoc_curve, new_curve);
	ssoc_state->ssoc_curve_type = type;

	/* this really means do not splice the (DSG) curve on disconnect */
	if (ssoc_state->rl_status != BATT_RL_STATUS_NONE
	    || ssoc_level >= ssoc_point_full)
		return;

	/* splice at (->ssoc_gdf,->ssoc_rl) because past spoof */
	ssoc_uicurve_splice(ssoc_state->ssoc_curve,
			    ssoc_state->ssoc_gdf,
			    ssoc_level);
}

/* ------------------------------------------------------------------------- */

/* enter recharge logic in BATT_RL_STATUS_DISCHARGE on charger_DONE,
 * enter BATT_RL_STATUS_RECHARGE on Fuel Gauge FULL
 * NOTE: batt_rl_update_status() doesn't call this, it flip from DISCHARGE
 * to recharge on its own.
 * NOTE: call holding chg_lock
 * @pre rl_status != BATT_RL_STATUS_NONE
 */
static bool batt_rl_enter(struct batt_ssoc_state *ssoc_state,
			  enum batt_rl_status rl_status)
{
	const int rl_current = ssoc_state->rl_status;

	/* NOTE: NO_OP when RL=DISCHARGE since batt_rl_update_status() flip
	 * between BATT_RL_STATUS_DISCHARGE and BATT_RL_STATUS_RECHARGE
	 * directly.
	 */
	if (rl_current == rl_status || rl_current == BATT_RL_STATUS_DISCHARGE)
		return false;

	/* NOTE: rl_status transition from *->DISCHARGE on charger FULL (during
	 * charge or at the end of recharge) and transition from
	 * NONE->RECHARGE when battery is full (SOC==100%) before charger is.
	 */
	if (rl_status == BATT_RL_STATUS_DISCHARGE) {
		ssoc_uicurve_dup(ssoc_state->ssoc_curve, dsg_curve);
		ssoc_state->ssoc_curve_type = SSOC_UIC_TYPE_DSG;
	}

	ssoc_update(ssoc_state, ssoc_state->ssoc_gdf);
	ssoc_state->rl_status = rl_status;

	return true;
}

/* NOTE: might need to use SOC from bootloader as starting point to avoid UI
 * SSOC jumping around or taking long time to coverge. Could technically read
 * charger voltage and estimate SOC% based on empty and full voltage.
 */
static int ssoc_init(struct batt_ssoc_state *ssoc_state,
		     struct power_supply *fg_psy)
{
	int ret, capacity;

	/* ssoc_work() needs a curve: start with the charge curve to prevent
	 * SSOC% from increasing after a reboot. Curve type must be NONE until
	 * battery knows the charger BUCK_EN state.
	 */
	ssoc_uicurve_dup(ssoc_state->ssoc_curve, chg_curve);
	ssoc_state->ssoc_curve_type = SSOC_UIC_TYPE_NONE;

	ret = ssoc_work(ssoc_state, fg_psy);
	if (ret < 0)
		return -EIO;

	capacity = ssoc_get_capacity(ssoc_state);
	if (capacity >= SSOC_FULL) {
		/* consistent behavior when booting without adapter */
		ssoc_uicurve_dup(ssoc_state->ssoc_curve, dsg_curve);
	} else if (capacity < SSOC_SPOOF) {
		/* mark the initial point if under spoof */
		ssoc_uicurve_splice(ssoc_state->ssoc_curve,
						ssoc_state->ssoc_gdf,
						ssoc_state->ssoc_rl);

	}

	return 0;
}

/* ------------------------------------------------------------------------- */

/* just reset state, no PS notifications no changes in the UI curve. This is
 * called on startup and on disconnect when the charge driver state is reset
 * NOTE: call holding chg_lock
 */
static void batt_rl_reset(struct batt_drv *batt_drv)
{
	batt_drv->ssoc_state.rl_status = BATT_RL_STATUS_NONE;
}

/* RL recharge: after SSOC work, restart charging.
 * NOTE: call holding chg_lock
 */
static void batt_rl_update_status(struct batt_drv *batt_drv)
{
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int soc;

	/* already in _RECHARGE or _NONE, done */
	if (ssoc_state->rl_status != BATT_RL_STATUS_DISCHARGE)
		return;

	/* recharge logic work on real soc */
	soc = ssoc_get_real(ssoc_state);
	if (ssoc_state->rl_soc_threshold &&
	    soc <= ssoc_state->rl_soc_threshold) {

		/* change state (will restart charge) on trigger */
		ssoc_state->rl_status = BATT_RL_STATUS_RECHARGE;
		if (batt_drv->psy)
			power_supply_changed(batt_drv->psy);
	}

}

/* ------------------------------------------------------------------------- */

#define get_boot_sec() div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC)

static void batt_chg_stats_init(struct gbms_charging_event *ce_data)
{
	int i;

	memset(ce_data, 0, sizeof(*ce_data));

	ce_data->charging_stats.voltage_in = -1;
	ce_data->charging_stats.ssoc_in = -1;
	ce_data->charging_stats.voltage_out = -1;
	ce_data->charging_stats.ssoc_out = -1;

	for (i = 0; i < GBMS_STATS_TIER_COUNT ; i++) {
		ce_data->tier_stats[i].voltage_tier_idx = i;
		ce_data->tier_stats[i].soc_in = -1;
	}

}

static void batt_chg_stats_start(struct batt_drv *batt_drv)
{
	union gbms_ce_adapter_details ad;
	struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	const time_t now = get_boot_sec();
	int vin, cc_in;

	mutex_lock(&batt_drv->stats_lock);
	ad.v = batt_drv->ce_data.adapter_details.v;
	batt_chg_stats_init(ce_data);
	batt_drv->ce_data.adapter_details.v = ad.v;

	vin = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW);
	ce_data->charging_stats.voltage_in = (vin < 0) ? -1 : vin / 1000;
	ce_data->charging_stats.ssoc_in =
				ssoc_get_capacity(&batt_drv->ssoc_state);
	cc_in = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_CHARGE_COUNTER);
	ce_data->charging_stats.cc_in = (cc_in < 0) ? -1 : cc_in / 1000;

	ce_data->charging_stats.ssoc_out = -1;
	ce_data->charging_stats.voltage_out = -1;

	ce_data->first_update = now;
	ce_data->last_update = now;

	mutex_unlock(&batt_drv->stats_lock);
}

/* call holding stats_lock */
static bool batt_chg_stats_qual(const struct gbms_charging_event *ce_data)
{
	const long elap = ce_data->last_update - ce_data->first_update;
	const long ssoc_delta = ce_data->charging_stats.ssoc_out -
				ce_data->charging_stats.ssoc_in;

	return elap >= ce_data->chg_sts_qual_time ||
	    ssoc_delta >= ce_data->chg_sts_delta_soc;
}

/* call holding the log */
static void batt_chg_stats_update(struct batt_drv *batt_drv,
				  int msc_state, int vbatt_idx,
				  int ibatt_ma, int temp)
{
	const time_t now = get_boot_sec();
	const time_t elap = now - batt_drv->ce_data.last_update;
	const uint16_t icl_settled = batt_drv->chg_state.f.icl;
	struct gbms_ce_tier_stats *tier =
				&batt_drv->ce_data.tier_stats[vbatt_idx];

	if (msc_state == -1 || vbatt_idx == -1) {
		pr_err("MSC_STAT error msc_state=%d, vbatt_idx=%d\n",
				msc_state, vbatt_idx);
		return;
	}

	if (tier->soc_in == -1) {
		int soc_in, cc_in;

		soc_in = GPSY_GET_PROP(batt_drv->fg_psy,
					POWER_SUPPLY_PROP_CAPACITY_RAW);
		cc_in = GPSY_GET_PROP(batt_drv->fg_psy,
					POWER_SUPPLY_PROP_CHARGE_COUNTER);
		if (soc_in < 0 || cc_in < 0) {
			pr_info("MSC_STAT cannot read soc_in=%d or cc_in=%d\n",
				soc_in, cc_in);
			return;
		}

		tier->temp_in = temp;
		tier->temp_min = temp;
		tier->temp_max = temp;

		tier->ibatt_min = ibatt_ma;
		tier->ibatt_max = ibatt_ma;

		tier->icl_min = icl_settled;
		tier->icl_max = icl_settled;

		tier->soc_in = soc_in;
		tier->cc_in = cc_in / 1000;
	} else {
		const u8 flags = batt_drv->chg_state.f.flags;

		if (flags & GBMS_CS_FLAG_CC) {
			tier->time_fast += elap;
		} else if (flags & GBMS_CS_FLAG_CV) {
			tier->time_taper += elap;
		} else {
			tier->time_other += elap;
		}

		if (temp < tier->temp_min)
			tier->temp_min = temp;
		if (temp > tier->temp_max)
			tier->temp_max = temp;

		if (ibatt_ma < tier->ibatt_min)
			tier->ibatt_min = ibatt_ma;
		if (ibatt_ma > tier->ibatt_max)
			tier->ibatt_max = ibatt_ma;

		if (icl_settled < tier->icl_min)
			tier->icl_min = icl_settled;
		if (icl_settled > tier->icl_max)
			tier->icl_max = icl_settled;

		/* averages: temp < 100. icl_settled < 3000, sum(ibatt)
		 * is bound to battery capacity, elap in seconds, sums
		 * are stored in an s64. For icl_settled I need a tier
		 * to last for more than ~97M years.
		 */
		tier->msc_cnt[batt_drv->msc_state] += 1;
		tier->msc_elap[batt_drv->msc_state] += elap;
		tier->icl_sum += icl_settled * elap;
		tier->temp_sum += temp * elap;
		tier->ibatt_sum += ibatt_ma * elap;
	}

	tier->sample_count += 1;
	/* nex update will book to msc_state */
	batt_drv->msc_state = msc_state;
	batt_drv->ce_data.last_update = now;
}

/* Only the qualified copy gets the timestamp and the exit voltage.
 */
static void batt_chg_stats_pub(struct batt_drv *batt_drv,
			       char *reason,
			       bool force)
{
	bool publish;
	const int vout = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW);
	const int cc_out = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_CHARGE_COUNTER);

	mutex_lock(&batt_drv->stats_lock);

	/* book last period to the current tier */
	if (batt_drv->msc_state != -1 && batt_drv->vbatt_idx != -1)
		batt_chg_stats_update(batt_drv, batt_drv->msc_state,
				      batt_drv->vbatt_idx, 0, 0);

	/* record the closing in data (and qual) */
	batt_drv->ce_data.charging_stats.voltage_out =
				(vout < 0) ? -1 : vout / 1000;
	batt_drv->ce_data.charging_stats.ssoc_out =
				ssoc_get_capacity(&batt_drv->ssoc_state);
	batt_drv->ce_data.charging_stats.cc_out =
				(cc_out < 0) ? -1 : cc_out / 1000;

	/* TODO: add a field to ce_data to qual weird charge sessions */
	publish = force || batt_chg_stats_qual(&batt_drv->ce_data);
	if (publish) {
		struct gbms_charging_event *ce_qual = &batt_drv->ce_qual;

		memcpy(ce_qual, &batt_drv->ce_data, sizeof(*ce_qual));

		pr_info("MSC_STAT %s: elap=%ld ssoc=%d->%d v=%d->%d c=%d->%d\n",
			reason,
			ce_qual->last_update - ce_qual->first_update,
			ce_qual->charging_stats.ssoc_in,
			ce_qual->charging_stats.ssoc_out,
			ce_qual->charging_stats.voltage_in,
			ce_qual->charging_stats.voltage_out,
			ce_qual->charging_stats.cc_in,
			ce_qual->charging_stats.cc_out);
	}

	mutex_unlock(&batt_drv->stats_lock);

	if (publish)
		power_supply_changed(batt_drv->psy);
}


static int batt_chg_stats_cstr(char *buff, int size,
				const struct gbms_charging_event *ce_data,
				bool verbose)
{
	int i, j, len = 0;
	static char *codes[] = { "n", "s", "d", "l", "v", "vo", "p", "f", "t",
				"dl", "st", "r", "w", "rs", "n", "ny" };

	if (verbose) {
		const char *adapter_name =
			gbms_chg_ev_adapter_s(ce_data->adapter_details.ad_type);

		len += snprintf(&buff[len], size - len, "A: %s,",
			adapter_name);
	}

	len += snprintf(&buff[len], size - len, "%d,%d,%d",
				ce_data->adapter_details.ad_type,
				ce_data->adapter_details.ad_voltage * 100,
				ce_data->adapter_details.ad_amperage * 100);

	len += snprintf(&buff[len], size - len, "%s%hu,%hu, %hu,%hu",
				(verbose) ?  "\nS: " : ", ",
				ce_data->charging_stats.ssoc_in,
				ce_data->charging_stats.voltage_in,
				ce_data->charging_stats.ssoc_out,
				ce_data->charging_stats.voltage_out);


	if (verbose) {
		len += snprintf(&buff[len], size - len, " %hu,%hu",
				ce_data->charging_stats.cc_in,
				ce_data->charging_stats.cc_out);

		len += snprintf(&buff[len], size - len, " %ld,%ld",
				ce_data->first_update,
				ce_data->last_update);
	}

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++) {
		const long elap = ce_data->tier_stats[i].time_fast +
				  ce_data->tier_stats[i].time_taper +
				  ce_data->tier_stats[i].time_other;

		if (!elap)
			continue;

		len += snprintf(&buff[len], size - len, "\n%d%c ",
			ce_data->tier_stats[i].voltage_tier_idx,
			(verbose) ? ':' : ',');

		len += snprintf(&buff[len], size - len,
			"%d.%d,%d,%d, %d,%d,%d, %d,%ld,%d, %d,%ld,%d, %d,%ld,%d",
				ce_data->tier_stats[i].soc_in >> 8,
				ce_data->tier_stats[i].soc_in & 0xff,
				ce_data->tier_stats[i].cc_in,
				ce_data->tier_stats[i].temp_in,
				ce_data->tier_stats[i].time_fast,
				ce_data->tier_stats[i].time_taper,
				ce_data->tier_stats[i].time_other,
				ce_data->tier_stats[i].temp_min,
				ce_data->tier_stats[i].temp_sum / elap,
				ce_data->tier_stats[i].temp_max,
				ce_data->tier_stats[i].ibatt_min,
				ce_data->tier_stats[i].ibatt_sum / elap,
				ce_data->tier_stats[i].ibatt_max,
				ce_data->tier_stats[i].icl_min,
				ce_data->tier_stats[i].icl_sum / elap,
				ce_data->tier_stats[i].icl_max);

		if (!verbose)
			continue;

		/* time spent in every multi step charging state */
		len += snprintf(&buff[len], size - len, "\n%d:",
				ce_data->tier_stats[i].voltage_tier_idx);

		for (j = 0; j < MSC_STATES_COUNT; j++)
			len += snprintf(&buff[len], size - len, " %s=%d",
				codes[j], ce_data->tier_stats[i].msc_elap[j]);

		/* count spent in each step charging state */
		len += snprintf(&buff[len], size - len, "\n%d:",
				ce_data->tier_stats[i].voltage_tier_idx);

		for (j = 0; j < MSC_STATES_COUNT; j++)
			len += snprintf(&buff[len], size - len, " %s=%d",
				codes[j], ce_data->tier_stats[i].msc_cnt[j]);
	}

	len += snprintf(&buff[len], size - len, "\n");

	return len;
}

/* ------------------------------------------------------------------------- */
static void batt_res_dump_logs(struct batt_res *rstate)
{
	pr_info("RES: req:%d, sample:%d[%d], filt_cnt:%d, res_avg:%d\n",
		rstate->estimate_requested, rstate->sample_accumulator,
		rstate->sample_count, rstate->filter_count,
		rstate->resistance_avg);
}

static void batt_res_state_set(struct batt_res *rstate, bool breq)
{
	rstate->estimate_requested = breq;
	rstate->sample_accumulator = 0;
	rstate->sample_count = 0;
	batt_res_dump_logs(rstate);
}

static void batt_res_store_data(struct batt_res *rstate,
				struct power_supply *fg_psy)
{
	int ret = 0;
	int filter_estimate = 0;
	int total_estimate = 0;
	long new_estimate = 0;
	union power_supply_propval val;

	new_estimate = rstate->sample_accumulator / rstate->sample_count;
	filter_estimate = rstate->resistance_avg * rstate->filter_count;

	rstate->filter_count++;
	if (rstate->filter_count > rstate->estimate_filter) {
		rstate->filter_count = rstate->estimate_filter;
		filter_estimate -= rstate->resistance_avg;
	}
	total_estimate = filter_estimate + new_estimate;
	rstate->resistance_avg = total_estimate / rstate->filter_count;

	/* Save to NVRam*/
	val.intval = rstate->resistance_avg;
	ret = power_supply_set_property(fg_psy,
					POWER_SUPPLY_PROP_RESISTANCE_AVG,
					&val);
	if (ret < 0)
		pr_err("failed to write resistance_avg\n");

	val.intval = rstate->filter_count;
	ret = power_supply_set_property(fg_psy,
					POWER_SUPPLY_PROP_RES_FILTER_COUNT,
					&val);
	if (ret < 0)
		pr_err("failed to write resistenace filt_count\n");

	batt_res_dump_logs(rstate);
}

static int batt_res_load_data(struct batt_res *rstate,
			      struct power_supply *fg_psy)
{
	union power_supply_propval val;
	int ret = 0;

	ret = power_supply_get_property(fg_psy,
					POWER_SUPPLY_PROP_RESISTANCE_AVG,
					&val);
	if (ret < 0) {
		pr_err("failed to get resistance_avg(%d)\n", ret);
		return ret;
	}
	rstate->resistance_avg = val.intval;

	ret = power_supply_get_property(fg_psy,
					POWER_SUPPLY_PROP_RES_FILTER_COUNT,
					&val);
	if (ret < 0) {
		rstate->resistance_avg = 0;
		pr_err("failed to get resistance filt_count(%d)\n", ret);
		return ret;
	}
	rstate->filter_count = val.intval;

	batt_res_dump_logs(rstate);
	return 0;
}

static void batt_res_work(struct batt_drv *batt_drv)
{
	int temp, ret, resistance;
	struct batt_res *rstate = &batt_drv->res_state;
	const int ssoc_threshold = rstate->ssoc_threshold;
	const int res_temp_low = rstate->res_temp_low;
	const int res_temp_high = rstate->res_temp_high;

	temp = GPSY_GET_INT_PROP(batt_drv->fg_psy,
				 POWER_SUPPLY_PROP_TEMP, &ret);
	if (ret < 0 || temp < res_temp_low || temp > res_temp_high) {
		if (ssoc_get_real(&batt_drv->ssoc_state) > ssoc_threshold) {
			if (rstate->sample_count > 0) {
				/* update the filter */
				batt_res_store_data(&batt_drv->res_state,
						    batt_drv->fg_psy);
				batt_res_state_set(rstate, false);
			}
		}
		return;
	}

	resistance = GPSY_GET_INT_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_RESISTANCE, &ret);
	if (ret < 0)
		return;

	if (ssoc_get_real(&batt_drv->ssoc_state) < ssoc_threshold) {
		rstate->sample_accumulator += resistance / 100;
		rstate->sample_count++;
		batt_res_dump_logs(rstate);
	} else {
		if (rstate->sample_count > 0) {
			/* update the filter here */
			batt_res_store_data(&batt_drv->res_state,
					    batt_drv->fg_psy);
		}
		batt_res_state_set(rstate, false);
	}
}

/* ------------------------------------------------------------------------- */

/* should not reset rl state */
static inline void batt_reset_chg_drv_state(struct batt_drv *batt_drv)
{
	/* algo */
	batt_drv->temp_idx = -1;
	batt_drv->vbatt_idx = -1;
	batt_drv->fv_uv = -1;
	batt_drv->cc_max = -1;
	batt_drv->msc_update_interval = -1;
	batt_drv->jeita_stop_charging = -1;
	/* timers */
	batt_drv->checked_cv_cnt = 0;
	batt_drv->checked_ov_cnt = 0;
	batt_drv->checked_tier_switch_cnt = 0;
	/* stats */
	batt_drv->msc_state = -1;
}

/* software JEITA, disable charging when outside the charge table.
 * TODO: need to be able to disable (leave to HW)
 */
static bool msc_logic_soft_jeita(struct batt_drv *batt_drv, int temp)
{
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;

	if (temp < profile->temp_limits[0] ||
	    temp > profile->temp_limits[profile->temp_nb_limits - 1]) {
		if (batt_drv->jeita_stop_charging < 0) {
			pr_info("MSC_JEITA temp=%d off limits, do not enable charging\n",
				temp);
		} else if (batt_drv->jeita_stop_charging == 0) {
			pr_info("MSC_JEITA temp=%d off limits, disabling charging\n",
				temp);
			batt_reset_chg_drv_state(batt_drv);
		}

		return 1;
	}

	if (batt_drv->jeita_stop_charging) {
		pr_info("MSC_JEITA temp=%d ok, enabling charging\n", temp);
		batt_drv->jeita_stop_charging = false;
	}

	return 0;
}

static int msc_logic_internal(struct batt_drv *batt_drv)
{
	int msc_state = MSC_NONE;
	struct power_supply *fg_psy = batt_drv->fg_psy;
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	int vbatt_idx = batt_drv->vbatt_idx, fv_uv = batt_drv->fv_uv, temp_idx;
	int temp, ibatt, vbatt, vchrg, chg_type, ioerr;
	int update_interval = MSC_DEFAULT_UPDATE_INTERVAL;
	bool sw_jeita;

	temp = GPSY_GET_INT_PROP(fg_psy, POWER_SUPPLY_PROP_TEMP,&ioerr);
	if (ioerr < 0)
		return -EIO;

	sw_jeita = msc_logic_soft_jeita(batt_drv, temp);
	if (sw_jeita)
		return 0;

	ibatt = GPSY_GET_INT_PROP(fg_psy, POWER_SUPPLY_PROP_CURRENT_NOW,
					  &ioerr);
	if (ioerr < 0)
		return -EIO;

	vbatt = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (vbatt < 0)
		return -EIO;

	/* invalid or 0 vchg disable IDROP compensation in FAST */
	chg_type = batt_drv->chg_state.f.chg_type;
	vchrg = batt_drv->chg_state.f.vchrg;
	if (vchrg <= 0)
		vchrg = vbatt;

	/* Multi Step Charging with IRDROP compensation when vchrg is != 0
	 * vbatt_idx = batt_drv->vbatt_idx, fv_uv = batt_drv->fv_uv
	 */
	temp_idx = gbms_msc_temp_idx(profile, temp);
	if (temp_idx != batt_drv->temp_idx || batt_drv->fv_uv == -1 ||
		batt_drv->vbatt_idx == -1) {

		msc_state = MSC_SEED;

		/* seed voltage only when really needed */
		if (batt_drv->vbatt_idx == -1)
			vbatt_idx = gbms_msc_voltage_idx(profile, vbatt);

		pr_info("MSC_SEED temp=%d vbatt=%d temp_idx:%d->%d, vbatt_idx:%d->%d\n",
			temp, vbatt, batt_drv->temp_idx, temp_idx,
			batt_drv->vbatt_idx, vbatt_idx);

		/* Debounce tier switch only when not already switching */
		if (batt_drv->checked_tier_switch_cnt == 0)
			batt_drv->checked_cv_cnt = profile->cv_debounce_cnt;
	} else if (ibatt > 0) {
		/* Track battery voltage if discharging is due to system load,
		 * low ILIM or lack of headroom; stop charging work and reset
		 * batt_drv state() when discharging is due to disconnect.
		 * NOTE: POWER_SUPPLY_PROP_STATUS return *_DISCHARGING only on
		 * disconnect.
		 * NOTE: same vbat_idx will not change fv_uv
		 */
		msc_state = MSC_DSG;
		vbatt_idx = gbms_msc_voltage_idx(profile, vbatt);

		pr_info("MSC_DSG vbatt_idx:%d->%d vbatt=%d ibatt=%d fv_uv=%d cv_cnt=%d ov_cnt=%d\n",
			batt_drv->vbatt_idx, vbatt_idx,
			vbatt, ibatt, fv_uv,
			batt_drv->checked_cv_cnt,
			batt_drv->checked_ov_cnt);
	} else if (batt_drv->vbatt_idx == profile->volt_nb_limits - 1) {
		/* will not adjust charger voltage only in the configured
		 * last tier.
		 * NOTE: might not be the "real" last tier since can I have
		 * tiers with max charge current == 0.
		 * NOTE: should I use a voltage limit instead?
		 */

		msc_state = MSC_LAST;
		pr_info("MSC_LAST vbatt=%d ibatt=%d fv_uv=%d\n",
			vbatt, ibatt, fv_uv);
	} else {
		const int vtier = profile->volt_limits[vbatt_idx];
		const int utv_margin = profile->cv_range_accuracy;
		const int otv_margin = profile->cv_otv_margin;
		const int switch_cnt = profile->cv_tier_switch_cnt;
		const int cc_next_max = GBMS_CCCM_LIMITS(profile, temp_idx,
						    vbatt_idx + 1);

		if ((vbatt - vtier) > otv_margin) {
		/* OVER: vbatt over vtier for more than margin (usually 0) */
			const int cc_max =
				GBMS_CCCM_LIMITS(profile, temp_idx, vbatt_idx);

			/* pullback when over tier voltage, fast poll, penalty
			 * on TAPER_RAISE and no cv debounce (so will consider
			 * switching voltage tiers if the current is right).
			 * NOTE: lowering voltage might cause a small drop in
			 * current (we should remain  under next tier)
			 */
			fv_uv = gbms_msc_round_fv_uv(profile, vtier,
				fv_uv - profile->fv_uv_resolution);
			if (fv_uv < vtier)
				fv_uv = vtier;

			update_interval = profile->cv_update_interval;
			batt_drv->checked_ov_cnt = profile->cv_tier_ov_cnt;
			batt_drv->checked_cv_cnt = 0;

			if (batt_drv->checked_tier_switch_cnt > 0) {
			/* no pullback, next tier if already counting */
				msc_state = MSC_VSWITCH;
				vbatt_idx = batt_drv->vbatt_idx + 1;

				pr_info("MSC_VSWITCH vt=%d vb=%d ibatt=%d\n",
					vtier, vbatt, ibatt);
			} else if (-ibatt == cc_max) {
			/* pullback, double penalty if at full current */
				msc_state = MSC_VOVER;
				batt_drv->checked_ov_cnt *= 2;

				pr_info("MSC_VOVER vt=%d  vb=%d ibatt=%d fv_uv=%d->%d\n",
					vtier, vbatt, ibatt,
					batt_drv->fv_uv, fv_uv);
			} else {
				msc_state = MSC_PULLBACK;
				pr_info("MSC_PULLBACK vt=%d vb=%d ibatt=%d fv_uv=%d->%d\n",
					vtier, vbatt, ibatt,
					batt_drv->fv_uv, fv_uv);
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
			msc_state = MSC_FAST;
			if (vchrg && vchrg > vbatt) {
				fv_uv = gbms_msc_round_fv_uv(profile, vtier,
					vtier + (vchrg - vbatt));
			} else {
				/* could keep it steady instead */
				fv_uv = vtier;
			}

			/* no tier switch during fast charge */
			if (batt_drv->checked_cv_cnt == 0)
				batt_drv->checked_cv_cnt = 1;

			pr_info("MSC_FAST vt=%d vb=%d fv_uv=%d->%d vchrg=%d cv_cnt=%d\n",
				vtier, vbatt, batt_drv->fv_uv, fv_uv,
				vchrg, batt_drv->checked_cv_cnt);

		} else if (chg_type != POWER_SUPPLY_CHARGE_TYPE_TAPER) {
		/* Not fast or taper: set checked_cv_cnt=0 to make sure we test
		 * for current and avoid early termination in case of lack of
		 * headroom (Vfloat ~= Vbatt)
		 * NOTE: this can cause early switch on low ilim
		 */
			msc_state = MSC_TYPE;
			update_interval = profile->cv_update_interval;
			batt_drv->checked_cv_cnt = 0;

			pr_info("MSC_TYPE vt=%d vb=%d fv_uv=%d chg_type=%d\n",
				vtier, vbatt, fv_uv, chg_type);

		} else if (batt_drv->checked_cv_cnt +
			   batt_drv->checked_ov_cnt) {
		/* TAPER_DLY: countdown to raise fv_uv and/or check
		 * for tier switch, will keep steady...
		 */
			pr_info("MSC_DLY vt=%d vb=%d fv_uv=%d margin=%d cv_cnt=%d, ov_cnt=%d\n",
				vtier, vbatt, fv_uv, profile->cv_range_accuracy,
				batt_drv->checked_cv_cnt,
				batt_drv->checked_ov_cnt);

			msc_state = MSC_DLY;
			update_interval = profile->cv_update_interval;
			if (batt_drv->checked_cv_cnt)
				batt_drv->checked_cv_cnt -= 1;
			if (batt_drv->checked_ov_cnt)
				batt_drv->checked_ov_cnt -= 1;

		} else if ((vtier - vbatt) < utv_margin) {
		/* TAPER_STEADY: close enough to tier, don't need to adjust */

			msc_state = MSC_STEADY;
			update_interval = profile->cv_update_interval;

			pr_info("MSC_STEADY vt=%d vb=%d fv_uv=%d margin=%d\n",
				vtier, vbatt, fv_uv,
				profile->cv_range_accuracy);
		} else {
		/* TAPER_RAISE: under tier vlim, raise one click & debounce
		 * taper (see above handling of "close enough")
		 */
			msc_state = MSC_RAISE;
			fv_uv = gbms_msc_round_fv_uv(profile, vtier,
				fv_uv + profile->fv_uv_resolution);
			update_interval = profile->cv_update_interval;

			/* debounce next taper voltage adjustment */
			batt_drv->checked_cv_cnt = profile->cv_debounce_cnt;

			pr_info("MSC_RAISE vt=%d vb=%d fv_uv=%d->%d\n",
				vtier, vbatt, batt_drv->fv_uv, fv_uv);
		}

		if (batt_drv->checked_cv_cnt > 0) {
		/* debounce period on tier switch */
			msc_state = MSC_WAIT;
			pr_info("MSC_WAIT vt=%d vb=%d fv_uv=%d ibatt=%d cv_cnt=%d ov_cnt=%d\n",
				vtier, vbatt, fv_uv, ibatt,
				batt_drv->checked_cv_cnt,
				batt_drv->checked_ov_cnt);
		} else if (-ibatt > cc_next_max) {
		/* current over next tier, reset tier switch count */
			msc_state = MSC_RSTC;
			batt_drv->checked_tier_switch_cnt = 0;

			pr_info("MSC_RSTC vt=%d vb=%d fv_uv=%d ibatt=%d cc_next_max=%d t_cnt=%d\n",
				vtier, vbatt, fv_uv, ibatt, cc_next_max,
				batt_drv->checked_tier_switch_cnt);
		} else if (batt_drv->checked_tier_switch_cnt >= switch_cnt) {
		/* next tier, fv_uv detemined at MSC_SET */
			msc_state = MSC_NEXT;
			vbatt_idx = batt_drv->vbatt_idx + 1;

			pr_info("MSC_NEXT tier vb=%d ibatt=%d vbatt_idx=%d->%d\n",
				vbatt, ibatt, batt_drv->vbatt_idx, vbatt_idx);
		} else {
		/* current under next tier, increase tier switch count */
			msc_state = MSC_NYET;
			batt_drv->checked_tier_switch_cnt++;

			pr_info("MSC_NYET ibatt=%d cc_next_max=%d t_cnt=%d\n",
				ibatt, cc_next_max,
				batt_drv->checked_tier_switch_cnt);
		}

	}

	/* update fv or cc will change in last tier... */

	/* need a new fv_uv only on a new voltage tier */
	if (vbatt_idx != batt_drv->vbatt_idx) {
		fv_uv = profile->volt_limits[vbatt_idx];
		batt_drv->checked_tier_switch_cnt = 0;
		batt_drv->checked_ov_cnt = 0;
	}

	/* book elapsed time to previous tier */
	if (vbatt_idx != -1 && vbatt_idx < GBMS_STATS_TIER_COUNT) {
		int tier_idx = batt_drv->vbatt_idx;

		if (tier_idx == -1)
			tier_idx = vbatt_idx;

		mutex_lock(&batt_drv->stats_lock);
		batt_chg_stats_update(batt_drv, msc_state,
				      tier_idx, ibatt / 1000, temp);
		mutex_unlock(&batt_drv->stats_lock);
	}


	batt_drv->vbatt_idx = vbatt_idx;
	batt_drv->temp_idx = temp_idx;
	batt_drv->cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, vbatt_idx);
	batt_drv->fv_uv = fv_uv;

	/* next update */
	batt_drv->msc_update_interval = update_interval;

	pr_info("MSC_DATA cv_cnt=%d ov_cnt=%d temp_idx:%d->%d, vbatt_idx:%d->%d, fv=%d->%d, cc_max=%d\n",
		batt_drv->checked_cv_cnt, batt_drv->checked_ov_cnt,
		batt_drv->temp_idx, temp_idx, batt_drv->vbatt_idx,
		vbatt_idx, batt_drv->fv_uv, fv_uv,
		batt_drv->cc_max);

	return 0;
}

#define MSC_LOGIC_VOTER "msc_logic"

/* called holding chg_lock */
static int msc_logic(struct batt_drv *batt_drv)
{
	int err = 0;
	bool changed = false;
	union gbms_charger_state *chg_state = &batt_drv->chg_state;

	if (!batt_drv->chg_profile.cccm_limits)
		return -EINVAL;

	__pm_stay_awake(&batt_drv->batt_ws);

	pr_info("MSC_DIN chg_state=%lx f=0x%x chg_s=%s chg_t=%s vchg=%d icl=%d\n",
		(unsigned long)chg_state->v,
		chg_state->f.flags,
		gbms_chg_status_s(chg_state->f.chg_status),
		gbms_chg_type_s(chg_state->f.chg_type),
		chg_state->f.vchrg,
		chg_state->f.icl);

	if ((batt_drv->chg_state.f.flags & GBMS_CS_FLAG_BUCK_EN) == 0) {
	/* here on: disconnect */

		if (batt_drv->buck_enabled == 0)
			goto msc_logic_exit;

		batt_chg_stats_pub(batt_drv, "disconnect", false);

		batt_res_state_set(&batt_drv->res_state, false);

		/* change curve before changing the state */
		ssoc_change_curve(&batt_drv->ssoc_state, SSOC_UIC_TYPE_DSG);
		batt_reset_chg_drv_state(batt_drv);
		batt_rl_reset(batt_drv);

		err = GPSY_SET_PROP(batt_drv->fg_psy,
				    POWER_SUPPLY_PROP_BATT_CE_CTRL,
				    false);
		if (err < 0)
			pr_err("Cannot set the BATT_CE_CTRL.\n");

		batt_drv->buck_enabled = 0;
		changed = true;

		goto msc_logic_done;
	}

	/* here when connected to power supply */
	if (!batt_drv->buck_enabled) {

		ssoc_change_curve(&batt_drv->ssoc_state, SSOC_UIC_TYPE_CHG);

		if (batt_drv->res_state.estimate_filter)
			batt_res_state_set(&batt_drv->res_state, true);

		batt_chg_stats_start(batt_drv);
		err = GPSY_SET_PROP(batt_drv->fg_psy,
				    POWER_SUPPLY_PROP_BATT_CE_CTRL,
				    true);
		if (err < 0)
			pr_err("Cannot set the BATT_CE_CTRL.\n");

		batt_drv->buck_enabled = 1;
		changed = true;
	}

	/* enter RL in DISCHARGE on charger DONE and enter RL in RECHARGE on
	 * battery FULL (i.e. SSOC==100%). charger DONE forces the discharge
	 * curve while RECHARGE will not modify the current curve.
	 */
	if ((batt_drv->chg_state.f.flags & GBMS_CS_FLAG_DONE) != 0) {
		changed = batt_rl_enter(&batt_drv->ssoc_state,
						BATT_RL_STATUS_DISCHARGE);
	} else if (batt_drv->batt_full) {
		changed = batt_rl_enter(&batt_drv->ssoc_state,
						BATT_RL_STATUS_RECHARGE);
		if (changed)
			batt_chg_stats_pub(batt_drv, "100%", false);

	}

	err = msc_logic_internal(batt_drv);
	if (err == 0) {
		pr_info("MSC_DOUT fv_uv=%d cc_max=%d update_interval=%d\n",
			batt_drv->fv_uv,
			batt_drv->cc_max,
			batt_drv->msc_update_interval);
	} else {
		/* NOTE: google charger will poll again. */
		batt_drv->msc_update_interval = -1;
		pr_err("MSC_DOUT ERROR=%d fv_uv=%d cc_max=%d update_interval=%d\n",
			err, batt_drv->fv_uv, batt_drv->cc_max,
			batt_drv->msc_update_interval);
	}

msc_logic_done:
	/* NOTE: google_charger has voted(0) on msc_interval_votable */
	if (!batt_drv->fv_votable)
		batt_drv->fv_votable = find_votable(VOTABLE_MSC_FV);
	if (batt_drv->fv_votable)
		vote(batt_drv->fv_votable, MSC_LOGIC_VOTER,
			batt_drv->fv_uv != -1,
			batt_drv->fv_uv);
	if (!batt_drv->fcc_votable)
		batt_drv->fcc_votable = find_votable(VOTABLE_MSC_FCC);
	if (batt_drv->fcc_votable)
		vote(batt_drv->fcc_votable, MSC_LOGIC_VOTER,
			batt_drv->cc_max != -1,
			batt_drv->cc_max);

	if (!batt_drv->msc_interval_votable)
		batt_drv->msc_interval_votable =
			find_votable(VOTABLE_MSC_INTERVAL);
	if (batt_drv->msc_interval_votable)
		vote(batt_drv->msc_interval_votable, MSC_LOGIC_VOTER,
			 batt_drv->msc_update_interval != -1,
			 batt_drv->msc_update_interval);
msc_logic_exit:

	if (changed) {
		dump_ssoc_state(&batt_drv->ssoc_state, batt_drv->log);
		if (batt_drv->psy)
			power_supply_changed(batt_drv->psy);
	}

	__pm_relax(&batt_drv->batt_ws);
	return err;
}

/* charge profile not in battery */
static int batt_init_chg_profile(struct batt_drv *batt_drv)
{
	struct device_node *node = batt_drv->device->of_node;
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	int ret = 0;

	/* handle retry */
	if (!profile->cccm_limits) {
		ret = gbms_init_chg_profile(profile, node);
		if (ret < 0)
			return -EINVAL;
	}

	ret = of_property_read_u32(node, "google,chg-battery-capacity",
				   &batt_drv->battery_capacity);
	if (ret < 0)
		pr_warn("read chg-battery-capacity from gauge\n");

	/* use battery FULL design when is not specified in DT. When battery is
	 * not present use default capacity from DT (if present) or disable
	 * charging altogether.
	 */
	if (batt_drv->battery_capacity == 0) {
		u32 fc = 0;
		struct power_supply *fg_psy = batt_drv->fg_psy;

		if (batt_drv->batt_present) {
			fc = GPSY_GET_PROP(fg_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
			if (fc == -EAGAIN)
				return -EPROBE_DEFER;
			if (fc > 0) {
				pr_info("successfully read charging profile:\n");
				/* convert uA to mAh*/
				batt_drv->battery_capacity = fc / 1000;
			}

		}

		if (batt_drv->battery_capacity == 0) {
			struct device_node *node = batt_drv->device->of_node;

			ret = of_property_read_u32(node,
					"google,chg-battery-default-capacity",
						&batt_drv->battery_capacity);
			if (ret < 0)
				pr_warn("battery not present, no default capacity, zero charge table\n");
			else
				pr_warn("battery not present, using default capacity:\n");
		}
	}

	/* NOTE: with NG charger tolerance is applied from "charger" */
	gbms_init_chg_table(&batt_drv->chg_profile, batt_drv->battery_capacity);

	return 0;
}

/* ------------------------------------------------------------------------- */

/* call holding mutex_unlock(&ccd->lock); */
static int batt_cycle_count_store(struct gbatt_ccbin_data *ccd,
				  struct power_supply *psy)
{
	union power_supply_propval val = { .strval = ccd->cyc_ctr_cstr };
	int ret;

	if (!psy)
		return -ENODATA;

	(void)gbms_cycle_count_cstr(ccd->cyc_ctr_cstr,
				    sizeof(ccd->cyc_ctr_cstr),
				    ccd->count);

	ret = power_supply_set_property(psy,
		POWER_SUPPLY_PROP_CYCLE_COUNTS, &val);
	if (ret < 0) {
		pr_err("failed to set cycle counts prop ret=%d\n", ret);
		return ret;
	}

	return 0;
}

/* call holding mutex_unlock(&ccd->lock); */
static int batt_cycle_count_load(struct gbatt_ccbin_data *ccd,
				 struct power_supply *psy)
{
	union power_supply_propval val;
	int ret = 0;

	if (!psy)
		return -ENODATA;

	ret = power_supply_get_property(psy,
		POWER_SUPPLY_PROP_CYCLE_COUNTS, &val);
	if (ret < 0) {
		pr_err("failed to get cycle counts prop ret=%d\n", ret);
		return ret;
	}

	ccd->prev_soc = -1;

	ret = gbms_cycle_count_sscan(ccd->count, val.strval);
	if (ret < 0)
		pr_err("invalid cycle count string ret=%d\n", ret);
	else
		strlcpy(ccd->cyc_ctr_cstr, val.strval,
					sizeof(ccd->cyc_ctr_cstr));

	return ret;
}

/* update only when SSOC is increasing, not need to check charging */
static void batt_cycle_count_update(struct batt_drv *batt_drv, int soc)
{
	struct gbatt_ccbin_data *ccd = &batt_drv->cc_data;

	if (soc < 0 || soc > 100)
		return;

	mutex_lock(&ccd->lock);

	if (ccd->prev_soc != -1 && soc > ccd->prev_soc) {
		int bucket, cnt;

		for (cnt = soc ; cnt > ccd->prev_soc ; cnt--) {
			/* cnt decremented by 1 for bucket symmetry */
			bucket = (cnt - 1) * GBMS_CCBIN_BUCKET_COUNT / 100;
			ccd->count[bucket]++;
		}

		/* NOTE: could store on FULL or disconnect instead */
		(void)batt_cycle_count_store(ccd, batt_drv->ccbin_psy);
	}

	ccd->prev_soc = soc;

	mutex_unlock(&ccd->lock);
}

/* ------------------------------------------------------------------------- */

#define BATTERY_DEBUG_ATTRIBUTE(name, fn_read, fn_write) \
static const struct file_operations name = {	\
	.open	= simple_open,			\
	.llseek	= no_llseek,			\
	.read	= fn_read,			\
	.write	= fn_write,			\
}

static ssize_t batt_cycle_count_set_bins(struct file *filp,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	char buf[GBMS_CCBIN_CSTR_SIZE];
	int ret;

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	mutex_lock(&batt_drv->cc_data.lock);

	ret = gbms_cycle_count_sscan(batt_drv->cc_data.count, buf);
	if (ret == 0 && batt_drv->ccbin_psy) {
		union power_supply_propval val = { .strval = buf };

		ret = power_supply_set_property(batt_drv->ccbin_psy,
						POWER_SUPPLY_PROP_CYCLE_COUNTS,
						&val);
		if (ret < 0)
			pr_err("failed to write cycle counts (%d)\n", ret);
	}

	if (ret == 0)
		ret = count;

	mutex_unlock(&batt_drv->cc_data.lock);

	return ret;
}

BATTERY_DEBUG_ATTRIBUTE(cycle_count_bins_fops,
				NULL, batt_cycle_count_set_bins);


static int cycle_count_bins_store(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	int ret;

	mutex_lock(&batt_drv->cc_data.lock);
	ret = batt_cycle_count_store(&batt_drv->cc_data, batt_drv->ccbin_psy);
	if (ret < 0)
		pr_err("cannot store bin count ret=%d\n", ret);
	mutex_unlock(&batt_drv->cc_data.lock);

	return ret;
}

static int cycle_count_bins_reload(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	int ret;

	mutex_lock(&batt_drv->cc_data.lock);
	ret = batt_cycle_count_load(&batt_drv->cc_data, batt_drv->ccbin_psy);
	if (ret < 0)
		pr_err("cannot restore bin count ret=%d\n", ret);
	mutex_unlock(&batt_drv->cc_data.lock);
	*val = ret;

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(cycle_count_bins_sync_fops,
				cycle_count_bins_reload,
				cycle_count_bins_store, "%llu\n");


static int debug_get_ssoc_gdf(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	*val = batt_drv->ssoc_state.ssoc_gdf;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ssoc_gdf_fops, debug_get_ssoc_gdf, NULL, "%u\n");


static int debug_get_ssoc_uic(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	*val = batt_drv->ssoc_state.ssoc_uic;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ssoc_uic_fops, debug_get_ssoc_uic, NULL, "%u\n");

static int debug_get_ssoc_rls(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	mutex_lock(&batt_drv->chg_lock);
	*val = batt_drv->ssoc_state.rl_status;
	mutex_unlock(&batt_drv->chg_lock);

	return 0;
}

static int debug_set_ssoc_rls(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (val < 0 || val > 2)
		return -EINVAL;

	mutex_lock(&batt_drv->chg_lock);
	batt_drv->ssoc_state.rl_status = val;
	mutex_unlock(&batt_drv->chg_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ssoc_rls_fops,
				debug_get_ssoc_rls, debug_set_ssoc_rls, "%u\n");

static ssize_t debug_get_ssoc_uicurve(struct file *filp,
					   char __user *buf,
					   size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	char tmp[UICURVE_BUF_SZ] = { 0 };

	mutex_lock(&batt_drv->chg_lock);
	ssoc_uicurve_cstr(tmp, sizeof(tmp), batt_drv->ssoc_state.ssoc_curve);
	mutex_unlock(&batt_drv->chg_lock);

	return simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));
}

static ssize_t debug_set_ssoc_uicurve(struct file *filp,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	int ret, curve_type;
	char buf[8];

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	mutex_lock(&batt_drv->chg_lock);

	curve_type = (int)simple_strtoull(buf, NULL, 10);
	if (curve_type >= -1 && curve_type <= 1)
		ssoc_change_curve(&batt_drv->ssoc_state, curve_type);
	else
		ret = -EINVAL;

	mutex_unlock(&batt_drv->chg_lock);

	if (ret == 0)
		ret = count;

	return 0;
}

BATTERY_DEBUG_ATTRIBUTE(debug_ssoc_uicurve_cstr_fops,
					debug_get_ssoc_uicurve,
					debug_set_ssoc_uicurve);

static ssize_t batt_ctl_chg_stats_actual(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	if (count < 1)
		return -ENODATA;

	switch (buf[0]) {
	case 'p': /* publish data to qual */
	case 'P': /* force publish data to qual */
		batt_chg_stats_pub(batt_drv, "debug cmd", buf[0] == 'P');
		break;
	default:
		count = -EINVAL;
		break;
	}

	return count;
}

static ssize_t batt_show_chg_stats_actual(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	int len;

	mutex_lock(&batt_drv->stats_lock);
	len = batt_chg_stats_cstr(buf, PAGE_SIZE, &batt_drv->ce_data, false);
	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

static const DEVICE_ATTR(charge_stats_actual, 0664,
					     batt_show_chg_stats_actual,
					     batt_ctl_chg_stats_actual);

static ssize_t batt_ctl_chg_stats(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	if (count < 1)
		return -ENODATA;

	mutex_lock(&batt_drv->stats_lock);
	switch (buf[0]) {
	case '0': /* invalidate current qual */
		batt_chg_stats_init(&batt_drv->ce_qual);
		break;
	}
	mutex_unlock(&batt_drv->stats_lock);

	return count;
}

static ssize_t batt_show_chg_stats(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	int len = -ENODATA;

	mutex_lock(&batt_drv->stats_lock);

	if (batt_drv->ce_qual.first_update != 0)
		len = batt_chg_stats_cstr(buf,
					  PAGE_SIZE,
					  &batt_drv->ce_qual, false);

	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

static const DEVICE_ATTR(charge_stats, 0664, batt_show_chg_stats,
					     batt_ctl_chg_stats);

static ssize_t batt_show_chg_details(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	int len = 0;

	mutex_lock(&batt_drv->stats_lock);

	len += batt_chg_stats_cstr(&buf[len], PAGE_SIZE - len,
				   &batt_drv->ce_data, true);

	if (batt_drv->ce_qual.first_update)
		len += batt_chg_stats_cstr(&buf[len], PAGE_SIZE - len,
					   &batt_drv->ce_qual, true);

	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

static const DEVICE_ATTR(charge_details, 0444, batt_show_chg_details,
					       NULL);


static int batt_init_fs(struct batt_drv *batt_drv)
{
	struct dentry *de = NULL;
	int ret;

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charge_stats);
	if (ret)
		dev_err(&batt_drv->psy->dev,
				"Failed to create charge_stats\n");

	ret = device_create_file(&batt_drv->psy->dev,
						&dev_attr_charge_stats_actual);
	if (ret)
		dev_err(&batt_drv->psy->dev,
				"Failed to create charge_stats_actual\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charge_details);
	if (ret)
		dev_err(&batt_drv->psy->dev,
				"Failed to create charge_details\n");

#ifdef CONFIG_DEBUG_FS
	de = debugfs_create_dir("google_battery", 0);
	if (de) {
		debugfs_create_file("cycle_count_bins", 0400, de,
				    batt_drv, &cycle_count_bins_fops);
		debugfs_create_file("cycle_count_sync", 0600, de,
				    batt_drv, &cycle_count_bins_sync_fops);
		debugfs_create_file("ssoc_gdf", 0600, de,
				    batt_drv, &debug_ssoc_gdf_fops);
		debugfs_create_file("ssoc_uic", 0600, de,
				    batt_drv, &debug_ssoc_uic_fops);
		debugfs_create_file("ssoc_rls", 0400, de,
				    batt_drv, &debug_ssoc_rls_fops);
		debugfs_create_file("ssoc_uicurve", 0600, de,
				    batt_drv, &debug_ssoc_uicurve_cstr_fops);
	}
#endif

	return ret;
}

/* ------------------------------------------------------------------------- */

/* poll the battery, run SOC% etc, scheduled from psy_changed and from timer */
static void google_battery_work(struct work_struct *work)
{
	struct batt_drv *batt_drv =
	    container_of(work, struct batt_drv, batt_work.work);
	struct power_supply *fg_psy = batt_drv->fg_psy;
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int update_interval = batt_drv->batt_update_interval;
	const int prev_ssoc = ssoc_get_capacity(ssoc_state);
	int fg_status, ret;

	pr_debug("battery work item\n");

	__pm_stay_awake(&batt_drv->batt_ws);

	mutex_lock(&batt_drv->chg_lock);
	fg_status = GPSY_GET_INT_PROP(fg_psy, POWER_SUPPLY_PROP_STATUS, &ret);
	if (ret < 0)
		goto reschedule;

	mutex_lock(&batt_drv->batt_lock);
	ret = ssoc_work(ssoc_state, fg_psy);
	if (ret < 0) {
		update_interval = BATT_WORK_ERROR_RETRY_MS;
	} else {
		int ssoc;

		/* handle charge/recharge */
		batt_rl_update_status(batt_drv);

		ssoc = ssoc_get_capacity(ssoc_state);
		if (prev_ssoc != ssoc)
			power_supply_changed(batt_drv->psy);

		/* fuel gauge triggered recharge logic. */
		batt_drv->batt_full = (ssoc == SSOC_FULL);
	}

	/* TODO: poll other data here if needed */

	mutex_unlock(&batt_drv->batt_lock);

reschedule:
	mutex_unlock(&batt_drv->chg_lock);

	if (batt_drv->res_state.estimate_requested)
		batt_res_work(batt_drv);


	batt_cycle_count_update(batt_drv, ssoc_get_real(ssoc_state));
	dump_ssoc_state(ssoc_state, batt_drv->log);

	if (update_interval) {
		pr_debug("rerun battery work in %d ms\n", update_interval);
		schedule_delayed_work(&batt_drv->batt_work,
				      msecs_to_jiffies(update_interval));
	}

	__pm_relax(&batt_drv->batt_ws);
}

/* ------------------------------------------------------------------------- */


static enum power_supply_property gbatt_battery_props[] = {
	POWER_SUPPLY_PROP_ADAPTER_DETAILS,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE,
	POWER_SUPPLY_PROP_CHARGE_DONE,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CYCLE_COUNTS,
	POWER_SUPPLY_PROP_FCC_STEPPER_ENABLE,		/* compat */
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,	/* compat */
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_RECHARGE_SOC,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_SOH,
	POWER_SUPPLY_PROP_CHARGE_FULL_ESTIMATE,
	POWER_SUPPLY_PROP_RESISTANCE_AVG,
};

/* . status is DISCHARGING when not connected.
 * . FULL when in recharge logic or GG report full and SSOC is @ 100% (possibly
 *   just when 100%).
 * . same as FG state otherwise
 */
static int gbatt_get_status(struct batt_drv *batt_drv,
			    union power_supply_propval *val)
{
	int err;

	if (!batt_drv->buck_enabled) {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	if (batt_drv->ssoc_state.rl_status != BATT_RL_STATUS_NONE) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	if (!batt_drv->fg_psy)
		return -EINVAL;

	err = power_supply_get_property(batt_drv->fg_psy,
					POWER_SUPPLY_PROP_STATUS,
					val);
	if (err != 0)
		return err;

	if (val->intval == POWER_SUPPLY_STATUS_FULL) {
		const int ssoc = ssoc_get_capacity(&batt_drv->ssoc_state);

		/* ->buck_enabled = true, device is connected */
		if (ssoc != SSOC_FULL)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
	}

	return 0;
}

static int gbatt_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	int err = 0;
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;

	pm_runtime_get_sync(batt_drv->device);
	if (!batt_drv->init_complete || !batt_drv->resume_complete) {
		pm_runtime_put_sync(batt_drv->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(batt_drv->device);

	switch (psp) {
	case POWER_SUPPLY_PROP_ADAPTER_DETAILS:
		val->intval = batt_drv->ce_data.adapter_details.v;
	break;

	case POWER_SUPPLY_PROP_CYCLE_COUNTS:
		mutex_lock(&batt_drv->cc_data.lock);
		(void)gbms_cycle_count_cstr(batt_drv->cc_data.cyc_ctr_cstr,
				sizeof(batt_drv->cc_data.cyc_ctr_cstr),
				batt_drv->cc_data.count);
		val->strval = batt_drv->cc_data.cyc_ctr_cstr;
		mutex_unlock(&batt_drv->cc_data.lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (batt_drv->fake_capacity >= 0 &&
				batt_drv->fake_capacity <= 100)
			val->intval = batt_drv->fake_capacity;
		else {
			mutex_lock(&batt_drv->batt_lock);
			val->intval = ssoc_get_capacity(ssoc_state);
			mutex_unlock(&batt_drv->batt_lock);
		}
		break;

	/* ng charging:
	 * 1) write to POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE,
	 * 2) read POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT and
	 *    POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE
	 */
	case POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE:
		val->intval = batt_drv->chg_state.v;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		mutex_lock(&batt_drv->chg_lock);
		if (batt_drv->jeita_stop_charging ||
		    ssoc_state->rl_status == BATT_RL_STATUS_DISCHARGE) {
			val->intval = 0;
		} else {
			val->intval = batt_drv->cc_max;
		}
		mutex_unlock(&batt_drv->chg_lock);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = batt_drv->fv_uv;
		mutex_unlock(&batt_drv->chg_lock);
		break;

	/* compat, I need this when you run w/o b/118820788 */
	case POWER_SUPPLY_PROP_SW_JEITA_ENABLED:
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_QNOVO_ENABLE:
		val->intval = 0;
		break;

	/* POWER_SUPPLY_PROP_CHARGE_DONE comes from the charger BUT battery
	 * has also an idea about it. Now using a software state: charge is
	 * DONE when we are in the discharge phase of the recharge logic.
	 * NOTE: might change to keep DONE while rl_status != NONE
	 */
	case POWER_SUPPLY_PROP_CHARGE_DONE:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = (ssoc_state->rl_status ==
					BATT_RL_STATUS_DISCHARGE);
		mutex_unlock(&batt_drv->chg_lock);
		break;
	/* POWER_SUPPLY_PROP_CHARGE_TYPE comes from the charger so using the
	 * last value reported from the CHARGER. This (of course) means that
	 * NG charging needs to be enabled.
	 */
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = batt_drv->chg_state.f.chg_type;
		mutex_unlock(&batt_drv->chg_lock);
		break;

	/* compat, for *_CURRENT_LIMITED could return this one:
	 * 	(batt_drv->chg_state.f.flags & GBMS_CS_FLAG_ILIM)
	 */
	case POWER_SUPPLY_PROP_FCC_STEPPER_ENABLE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		err = gbatt_get_status(batt_drv, val);
		break;

	case POWER_SUPPLY_PROP_RECHARGE_SOC:
		val->intval = ssoc_state->rl_soc_threshold;
		break;

	/* health */
	case POWER_SUPPLY_PROP_HEALTH:
		if (!batt_drv->fg_psy)
			return -EINVAL;
		err = power_supply_get_property(batt_drv->fg_psy, psp, val);
		if (err == 0)
			batt_drv->soh = val->intval;
		break;
	/* define this better */
	case POWER_SUPPLY_PROP_SOH:
		val->intval = batt_drv->soh;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_ESTIMATE:
		if (!batt_drv->fg_psy)
			return -EINVAL;
		err = power_supply_get_property(batt_drv->fg_psy, psp, val);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_AVG:
		if (batt_drv->res_state.filter_count <
			batt_drv->res_state.estimate_filter)
			val->intval = 0;
		else
			val->intval = batt_drv->res_state.resistance_avg;
		break;
	/* TODO: "charger" will expose this but I'd rather use an API from
	 * google_bms.h. Right now route it to fg_psy: just make sure that
	 * fg_psy doesn't look it up in google_battery
	 */
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		/* fall through */
	default:
		if (!batt_drv->fg_psy)
			return -EINVAL;
		err = power_supply_get_property(batt_drv->fg_psy, psp, val);
		break;
	}

	if (err < 0) {
		pr_debug("gbatt: get_prop cannot read psp=%d\n", psp);
		return err;
	}

	return 0;
}

static int gbatt_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int ret = 0;

	pm_runtime_get_sync(batt_drv->device);
	if (!batt_drv->init_complete || !batt_drv->resume_complete) {
		pm_runtime_put_sync(batt_drv->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(batt_drv->device);

	switch (psp) {
	case POWER_SUPPLY_PROP_ADAPTER_DETAILS:
		mutex_lock(&batt_drv->stats_lock);
		batt_drv->ce_data.adapter_details.v = val->intval;
		mutex_unlock(&batt_drv->stats_lock);
	break;

	/* NG Charging, where it all begins */
	case POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE:
		mutex_lock(&batt_drv->chg_lock);
		batt_drv->chg_state.v = val->int64val;

		ret = msc_logic(batt_drv);
		mutex_unlock(&batt_drv->chg_lock);
		break;

	/* TODO: b/118843345, just a switch to disable step charging */
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SW_JEITA_ENABLED:
		pr_err("cannot write to psp=%d\n", psp);
		return -EINVAL;

	/* This is a software implementation of the recharge threshold: I don't
	 * see big advantages in using the hardware controlled one since we will
	 * likely wakeup on dSOC changes anyway.
	 * NOTE: the HW controlled recharge might rely on chipset specific
	 * understanding of FG SOC which (generally) won't match the SOC
	 * reported from an external FG. Voltage would work.
	 * NOTE: qc set this in smb5_init_hw() via smblib:
	 * int smblib_set_prop_rechg_soc_thresh(struct smb_charger *chg,
	 *		const union power_supply_propval *val) { }
	 */
	case POWER_SUPPLY_PROP_RECHARGE_SOC:
		if (val->intval < 0 || val->intval > 100) {
			pr_err("recharge-soc is incorrect\n");
			ret = -EINVAL;
		} else if (ssoc_state->rl_soc_threshold != val->intval) {
			ssoc_state->rl_soc_threshold = val->intval;
			if (batt_drv->psy)
				power_supply_changed(batt_drv->psy);
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		batt_drv->fake_capacity = val->intval;
		if (batt_drv->psy)
			power_supply_changed(batt_drv->psy);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		pr_debug("gbatt: get_prop cannot write psp=%d\n", psp);
		return ret;
	}


	return 0;
}

static int gbatt_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE:
	case POWER_SUPPLY_PROP_RECHARGE_SOC:
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SW_JEITA_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_ADAPTER_DETAILS:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct power_supply_desc gbatt_psy_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = gbatt_get_property,
	.set_property = gbatt_set_property,
	.property_is_writeable = gbatt_property_is_writeable,
	.properties = gbatt_battery_props,
	.num_properties = ARRAY_SIZE(gbatt_battery_props),
};

static void google_battery_init_work(struct work_struct *work)
{
	struct power_supply *fg_psy;
	struct batt_drv *batt_drv = container_of(work, struct batt_drv,
						 init_work.work);
	int ret = 0;

	batt_rl_reset(batt_drv);
	batt_drv->buck_enabled = -1;
	batt_reset_chg_drv_state(batt_drv);
	mutex_init(&batt_drv->chg_lock);
	mutex_init(&batt_drv->batt_lock);
	mutex_init(&batt_drv->stats_lock);
	batt_chg_stats_init(&batt_drv->ce_data);
	batt_chg_stats_init(&batt_drv->ce_qual);

	fg_psy = power_supply_get_by_name(batt_drv->fg_psy_name);
	if (!fg_psy) {
		pr_info("failed to get \"%s\" power supply, retrying...\n",
			batt_drv->fg_psy_name);
		goto retry_init_work;
	}

	batt_drv->fg_psy = fg_psy;
	/* uncomment this to enable loading/storing data to maxfg
	 *	batt_drv->ccbin_psy = batt_drv->fg_psy;
	 */

	if (!batt_drv->batt_present) {
		ret = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_PRESENT);
		if (ret == -EAGAIN)
			goto retry_init_work;

		batt_drv->batt_present = (ret > 0);
		if (!batt_drv->batt_present)
			pr_warn("battery not present (ret=%d)\n", ret);
	}

	ret = of_property_read_u32(batt_drv->device->of_node,
			"google,recharge-soc-threshold",
			&batt_drv->ssoc_state.rl_soc_threshold);
	if (ret < 0)
		batt_drv->ssoc_state.rl_soc_threshold =
				DEFAULT_BATT_DRV_RL_SOC_THRESHOLD;

	ret = ssoc_init(&batt_drv->ssoc_state, fg_psy);
	if (ret < 0 && batt_drv->batt_present)
		goto retry_init_work;

	dump_ssoc_state(&batt_drv->ssoc_state, batt_drv->log);

	ret = batt_init_chg_profile(batt_drv);
	if (ret == -EPROBE_DEFER)
		goto retry_init_work;

	if (ret < 0) {
		pr_err("charging profile disabled, ret=%d\n", ret);
	} else if (batt_drv->battery_capacity) {
		gbms_dump_chg_profile(&batt_drv->chg_profile);
	}

	batt_drv->fg_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&batt_drv->fg_nb);
	if (ret < 0)
		pr_err("cannot register power supply notifer, ret=%d\n",
			ret);

	wakeup_source_init(&batt_drv->batt_ws, gbatt_psy_desc.name);

	mutex_lock(&batt_drv->cc_data.lock);
	ret = batt_cycle_count_load(&batt_drv->cc_data, batt_drv->ccbin_psy);
	if (ret < 0)
		pr_err("cannot restore bin count ret=%d\n", ret);
	mutex_unlock(&batt_drv->cc_data.lock);

	batt_drv->fake_capacity = (batt_drv->batt_present) ? -EINVAL
						: DEFAULT_BATT_FAKE_CAPACITY;

	(void)batt_init_fs(batt_drv);

	pr_info("init_work done\n");

	batt_res_load_data(&batt_drv->res_state, batt_drv->fg_psy);

	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,chg-stats-qual-time",
				   &batt_drv->ce_data.chg_sts_qual_time);
	if (ret < 0)
		batt_drv->ce_data.chg_sts_qual_time =
					DEFAULT_CHG_STATS_MIN_QUAL_TIME;

	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,chg-stats-delta-soc",
				   &batt_drv->ce_data.chg_sts_delta_soc);
	if (ret < 0)
		batt_drv->ce_data.chg_sts_delta_soc =
					DEFAULT_CHG_STATS_MIN_DELTA_SOC;

	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,update-interval",
				   &batt_drv->batt_update_interval);
	if (ret < 0)
		batt_drv->batt_update_interval = DEFAULT_BATT_UPDATE_INTERVAL;

	batt_drv->init_complete = true;
	batt_drv->resume_complete = true;

	schedule_delayed_work(&batt_drv->batt_work, 0);

	return;

retry_init_work:
	schedule_delayed_work(&batt_drv->init_work,
			      msecs_to_jiffies(BATT_DELAY_INIT_MS));
}

static struct thermal_zone_of_device_ops google_battery_tz_ops = {
	.get_temp = google_battery_tz_get_cycle_count,
};

static int google_battery_probe(struct platform_device *pdev)
{
	const char *fg_psy_name, *psy_name = NULL;
	struct batt_drv *batt_drv;
	int ret;
	struct power_supply_config psy_cfg = {};

	batt_drv = devm_kzalloc(&pdev->dev, sizeof(*batt_drv), GFP_KERNEL);
	if (!batt_drv)
		return -ENOMEM;

	batt_drv->device = &pdev->dev;

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,fg-psy-name", &fg_psy_name);
	if (ret != 0) {
		pr_err("cannot read google,fg-psy-name, ret=%d\n", ret);
		return -EINVAL;
	}

	batt_drv->fg_psy_name =
	    devm_kstrdup(&pdev->dev, fg_psy_name, GFP_KERNEL);
	if (!batt_drv->fg_psy_name)
		return -ENOMEM;

	/* change name and type for debug/test */
	if (of_property_read_bool(pdev->dev.of_node, "google,psy-type-unknown"))
		gbatt_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,psy-name", &psy_name);
	if (ret == 0) {
		gbatt_psy_desc.name =
		    devm_kstrdup(&pdev->dev, psy_name, GFP_KERNEL);
	}

	INIT_DELAYED_WORK(&batt_drv->init_work, google_battery_init_work);
	INIT_DELAYED_WORK(&batt_drv->batt_work, google_battery_work);
	platform_set_drvdata(pdev, batt_drv);

	psy_cfg.drv_data = batt_drv;
	psy_cfg.of_node = pdev->dev.of_node;

	batt_drv->psy = devm_power_supply_register(batt_drv->device,
						   &gbatt_psy_desc, &psy_cfg);
	if (IS_ERR(batt_drv->psy)) {
		ret = PTR_ERR(batt_drv->psy);
		dev_err(batt_drv->device,
			"Couldn't register as power supply, ret=%d\n", ret);
		/* TODO: fail with -ENODEV */
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}

	batt_drv->log = debugfs_logbuffer_register("ssoc");
	if (IS_ERR(batt_drv->log)) {
		ret = PTR_ERR(batt_drv->log);
		dev_err(batt_drv->device,
			"failed to obtain logbuffer instance, ret=%d\n", ret);
		batt_drv->log = NULL;
	}

	/* Resistance Estimation configuration */
	ret = of_property_read_u32(pdev->dev.of_node, "google,res-temp-hi",
				   &batt_drv->res_state.res_temp_high);
	if (ret < 0)
		batt_drv->res_state.res_temp_high = DEFAULT_RES_TEMP_HIGH;

	ret = of_property_read_u32(pdev->dev.of_node, "google,res-temp-lo",
				   &batt_drv->res_state.res_temp_low);
	if (ret < 0)
		batt_drv->res_state.res_temp_low = DEFAULT_RES_TEMP_LOW;

	ret = of_property_read_u32(pdev->dev.of_node, "google,res-soc-thresh",
				   &batt_drv->res_state.ssoc_threshold);
	if (ret < 0)
		batt_drv->res_state.ssoc_threshold = DEFAULT_RES_SSOC_THR;

	ret = of_property_read_u32(pdev->dev.of_node, "google,res-filt-length",
				   &batt_drv->res_state.estimate_filter);
	if (ret < 0)
		batt_drv->res_state.estimate_filter = DEFAULT_RES_FILT_LEN;

	batt_drv->tz_dev = thermal_zone_of_sensor_register(batt_drv->device,
				0, batt_drv, &google_battery_tz_ops);
	if (IS_ERR(batt_drv->tz_dev)) {
		pr_err("battery tz register failed. err:%ld\n",
			PTR_ERR(batt_drv->tz_dev));
		ret = PTR_ERR(batt_drv->tz_dev);
		batt_drv->tz_dev = NULL;
	} else {
		thermal_zone_device_update(batt_drv->tz_dev, THERMAL_DEVICE_UP);
	}
	/* give time to fg driver to start */
	schedule_delayed_work(&batt_drv->init_work,
					msecs_to_jiffies(BATT_DELAY_INIT_MS));

	return 0;
}

static int google_battery_remove(struct platform_device *pdev)
{
	struct batt_drv *batt_drv = platform_get_drvdata(pdev);

	if (batt_drv) {
		if (batt_drv->fg_psy)
			power_supply_put(batt_drv->fg_psy);

		gbms_free_chg_profile(&batt_drv->chg_profile);

		wakeup_source_trash(&batt_drv->batt_ws);

		if (batt_drv->log)
			debugfs_logbuffer_unregister(batt_drv->log);
		if (batt_drv->tz_dev)
			thermal_zone_of_sensor_unregister(batt_drv->device,
					batt_drv->tz_dev);
	}

	return 0;
}

#ifdef SUPPORT_PM_SLEEP
static int google_battery_pm_suspend(struct device *dev)
{
	struct max1720x_chip *chip = i2c_get_clientdata(client);

	pm_runtime_get_sync(chip->dev);
	chip->resume_complete = false;
	pm_runtime_put_sync(chip->dev);

	return 0;
}

static int google_battery_pm_resume(struct device *dev)
{
	struct max1720x_chip *chip = i2c_get_clientdata(client);

	pm_runtime_get_sync(chip->dev);
	chip->resume_complete = true;
	pm_runtime_put_sync(chip->dev);

	return 0;
}

static const struct dev_pm_ops google_battery_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(google_battery_pm_suspend,
		google_battery_resume)
};
#endif


static const struct of_device_id google_charger_of_match[] = {
	{.compatible = "google,battery"},
	{},
};
MODULE_DEVICE_TABLE(of, google_charger_of_match);


static struct platform_driver google_battery_driver = {
	.driver = {
		   .name = "google,battery",
		   .owner = THIS_MODULE,
		   .of_match_table = google_charger_of_match,
#ifdef SUPPORT_PM_SLEEP
		   //.pm = google_battery_pm_ops,
#endif
		   //.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		   },
	.probe = google_battery_probe,
	.remove = google_battery_remove,
};

static int __init google_battery_init(void)
{
	int ret;

	ret = platform_driver_register(&google_battery_driver);
	if (ret < 0) {
		pr_err("device registration failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static void __init google_battery_exit(void)
{
	platform_driver_unregister(&google_battery_driver);
	pr_info("unregistered platform driver\n");
}

module_init(google_battery_init);
module_exit(google_battery_exit);
MODULE_DESCRIPTION("Google Battery Driver");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
