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
// disabled for now, will need to keep track of sleep to potentially change
// wake mask etc.
//#define SUPPORT_PM_SLEEP
#endif

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>

/* TODO: b/117899012, move to google_bms.h */
#define CHG_TEMP_NB_LIMITS_MAX 10
#define CHG_VOLT_NB_LIMITS_MAX 5

#define BATT_DELAY_INIT_MS 250
#define DEFAULT_BATT_DRV_UPDATE_INTERVAL 30000

/* TODO: b/117899012, move to google_bms.h */
struct chg_profile {
	u32 battery_capacity;
	int temp_nb_limits;
	s32 temp_limits[CHG_TEMP_NB_LIMITS_MAX];
	int volt_nb_limits;
	s32 volt_limits[CHG_VOLT_NB_LIMITS_MAX];
	/* Array of constant current limits */
	s32 *cccm_limits;
	u32 fv_uv_resolution;
	u32 fv_uv_margin_dpct;
	u32 cv_range_accuracy;
	u32 cv_otv_margin;
	u32 cv_debounce_cnt;
	u32 cv_update_interval;
	u32 cv_tier_ov_cnt;
	u32 cv_tier_switch_cnt;
};

/* TODO: b/117899012,  move to google_bms.h */
struct batt_ssoc_state {
	int ssoc_gdf;	/* output of gauge data filter */
	int ssoc_uic;	/* output of UI Curves */
	int ssoc_rl;	/* output of rate limiter */
};

/* TODO: share with google charger in google_bms.c */
union chg_charger_state {
	uint64_t v;
	struct {
		uint8_t chg_type;
		uint8_t pad0[3];
		uint16_t vchrg;
		uint8_t pad1[2];
	} f;
};

enum batt_rl_status {
	BATT_RL_STATUS_NONE = 0,
	BATT_RL_STATUS_DISCHARGE = 1,
	BATT_RL_STATUS_RECHARGE = 2,
};
struct batt_drv {
	struct device *device;
	struct power_supply *psy;

	const char *fg_psy_name;
	struct power_supply *fg_psy;
	struct notifier_block fg_nb;

	struct delayed_work init_work;
	struct delayed_work batt_work;
	struct wakeup_source chg_ws;

	/* TODO: b/111407333, will likely need to adjust SOC% on wakeup */
	bool init_complete;
	bool resume_complete;

	struct mutex batt_lock;
	struct mutex chg_lock;

	/* battery work */
	u32 update_interval;
	struct batt_ssoc_state ssoc_state;

	/* props */
	int soh;

	/* NG charging */
	bool stop_charging;
	int disable_charging;
	struct chg_profile chg_profile;
	union chg_charger_state chg_state;
	int temp_idx;
	int vbatt_idx;
	int checked_cv_cnt;
	int checked_ov_cnt;
	int checked_tier_switch_cnt;
	int chg_mode;
	int fv_uv;
	int fcc;
	/* recharge logic */
	enum batt_rl_status rl_status;
	int fg_status;
};

/* Used as left operand also */
#define CCCM_LIMITS(profile, ti, vi) \
	profile->cccm_limits[(ti * profile->volt_nb_limits) + vi]

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

/* TODO: b/117899012,  move to google_bms.c, google_bms.h */
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

/* TODO: b/117899012,  move to google_bms.c, google_bms.h */
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

static inline void reset_chg_drv_state(struct batt_drv *batt_drv)
{
	batt_drv->temp_idx = -1;
	batt_drv->vbatt_idx = -1;
	batt_drv->fv_uv = -1;
	batt_drv->checked_cv_cnt = 0;
	batt_drv->checked_ov_cnt = 0;
	batt_drv->checked_tier_switch_cnt = 0;
	batt_drv->disable_charging = 0;
	batt_drv->stop_charging = true;
}

/* 1. charge profile idx based on the battery temperature
 * TODO: b/117899012,  move to google_bms.c, google_bms.h
 */
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
 * TODO: b/117899012,  move to google_bms.c, google_bms.h
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
		const int headr = profile->fv_uv_resolution * 3;

		if ((vt - vbatt) < headr)
			vbatt_idx += 1;
	}

	return vbatt_idx;
}

/* Cap to fv_uv_margin_pct of VTIER if needed
 * TODO: b/117899012,  move to google_bms.c, google_bms.h
 */
static int msc_round_fv_uv(struct chg_profile *profile, int vtier, int fv_uv)
{
	int result = fv_uv;
	const unsigned int fv_uv_max = (vtier / 1000) *
					profile->fv_uv_margin_dpct;

	if (fv_uv_max != 0 && fv_uv > fv_uv_max)
		result = fv_uv_max;

	if (fv_uv_max != 0)
		pr_info("MSC_ROUND: vtier=%d fv_uv_max=%d fv_uv=%d -> %d\n",
			vtier, fv_uv_max, fv_uv, result);

	return result;
}

/* poll the battery, run SOC% etc, scheduled from psy_changed and from timer */
static void google_battery_work(struct work_struct *work)
{
	struct batt_drv *batt_drv =
	    container_of(work, struct batt_drv, batt_work.work);
	struct power_supply *fg_psy = batt_drv->fg_psy;
	int update_interval = batt_drv->update_interval;
	int soc, fg_status;

	pr_debug("battery work item\n");

	__pm_stay_awake(&batt_drv->chg_ws);

	/* TODO: implement recharge logic, missing disconect/reconnect cases
	 * and corner cases.
	 */
	mutex_lock(&batt_drv->chg_lock);
	fg_status = PSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_STATUS);
	if (fg_status < 0) {
		goto reschedule;
	} else if (fg_status == POWER_SUPPLY_STATUS_FULL &&
			fg_status != batt_drv->fg_status) {
		/* trigger recharge logic state machine */
		/* if (batt_drv->rl_status != BATT_RL_STATUS_RECHARGE) {
		 *     Change the UI curve if not changed already.
		 * }
		 */
		batt_drv->rl_status = BATT_RL_STATUS_DISCHARGE;
	}
	mutex_unlock(&batt_drv->chg_lock);

	batt_drv->fg_status = fg_status;

	mutex_lock(&batt_drv->batt_lock);
	soc = PSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_CAPACITY);

	/* TODO: b/111407333 software state of charge, separate to functions */
	batt_drv->ssoc_state.ssoc_gdf = soc;
	batt_drv->ssoc_state.ssoc_uic = soc;
	batt_drv->ssoc_state.ssoc_rl = soc;

	mutex_unlock(&batt_drv->batt_lock);

	if (batt_drv->rl_status != BATT_RL_STATUS_DISCHARGE) {
		/* in recharge or outside RL zone */
	} else if (batt_drv->ssoc_state.ssoc_gdf > 95) {
		/* TODO: voltage or SOC threshold in DT */
		batt_drv->rl_status = BATT_RL_STATUS_RECHARGE;

		if (batt_drv->psy)
			power_supply_changed(batt_drv->psy);
	}

	pr_info("batt_work: soc=%d\n", soc);

reschedule:
	if (update_interval) {
		pr_debug("rerun battery work in %d ms\n", update_interval);
		schedule_delayed_work(&batt_drv->batt_work,
				      msecs_to_jiffies(update_interval));
	}

	__pm_relax(&batt_drv->chg_ws);
}


static int google_charge_logic_internal(struct batt_drv *batt_drv)
{
	struct power_supply *fg_psy = batt_drv->fg_psy;
	struct chg_profile *profile = &batt_drv->chg_profile;
	int vbatt_idx = batt_drv->vbatt_idx, fv_uv = batt_drv->fv_uv, temp_idx;
	int temp, ibatt, vbatt, vchrg, chg_type;
	int update_interval = 0; /* TODO: route this to power supply somehow */

	temp = PSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_TEMP);
	if (temp == -EINVAL)
		return -EINVAL;
	/* JEITA, update _SOH, take out from here */
	if (temp < profile->temp_limits[0] ||
	    temp > profile->temp_limits[profile->temp_nb_limits - 1]) {
		if (!batt_drv->stop_charging) {
			pr_info("batt. temp. off limits, disabling charging\n");
			reset_chg_drv_state(batt_drv);
		}

		return 0;
	}

	if (batt_drv->stop_charging) {
		pr_info("batt. temp. ok, enabling charging\n");
		batt_drv->stop_charging = false;
	}

	ibatt = PSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	vbatt = PSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (ibatt == -EINVAL || vbatt == -EINVAL)
		return -EINVAL;

	/* from charger state */
	vchrg = batt_drv->chg_state.f.vchrg;
	if (vchrg <= 0)
		vchrg = vbatt;
	chg_type = batt_drv->chg_state.f.chg_type;

	/* Multi Step Charging with IRDROP compensation when vchrg is != 0
	 * vbatt_idx = batt_drv->vbatt_idx, fv_uv = batt_drv->fv_uv
	 */
	temp_idx = msc_temp_idx(profile, temp);
	if (temp_idx != batt_drv->temp_idx || batt_drv->fv_uv == -1 ||
		batt_drv->vbatt_idx == -1) {

		/* seed voltage only when really needed */
		if (batt_drv->vbatt_idx == -1)
			vbatt_idx = msc_voltage_idx(profile, vbatt);

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
		vbatt_idx = msc_voltage_idx(profile, vbatt);
		update_interval = profile->cv_update_interval;

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

		pr_info("MSC_LAST vbatt=%d ibatt=%d fv_uv=%d\n",
			vbatt, ibatt, fv_uv);
	} else {
		const int vtier = profile->volt_limits[vbatt_idx];
		const int utv_margin = profile->cv_range_accuracy;
		const int otv_margin = profile->cv_otv_margin;
		const int switch_cnt = profile->cv_tier_switch_cnt;
		const int cc_next_max = CCCM_LIMITS(profile, temp_idx,
						    vbatt_idx + 1);

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
				fv_uv - profile->fv_uv_resolution);
			if (fv_uv < vtier)
				fv_uv = vtier;

			update_interval = profile->cv_update_interval;
			batt_drv->checked_ov_cnt = profile->cv_tier_ov_cnt;
			batt_drv->checked_cv_cnt = 0;

			if (batt_drv->checked_tier_switch_cnt > 0) {
			/* no pullback, next tier if already counting */
				vbatt_idx = batt_drv->vbatt_idx + 1;

				pr_info("MSC_VSWITCH vt=%d vb=%d ibatt=%d\n",
					vtier, vbatt, ibatt);
			} else if (-ibatt == cc_max) {
			/* pullback, double penalty if at full current */
				batt_drv->checked_ov_cnt *= 2;

				pr_info("MSC_VOVER vt=%d  vb=%d ibatt=%d fv_uv=%d->%d\n",
					vtier, vbatt, ibatt,
					batt_drv->fv_uv, fv_uv);
			} else {
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
			if (vchrg && vchrg > vbatt) {
				fv_uv = msc_round_fv_uv(profile, vtier,
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
			update_interval = profile->cv_update_interval;
			batt_drv->checked_cv_cnt = 0;

			pr_info("MSC_TYPE vt=%d vb=%d fv_uv=%d chg_type=%d\n",
				vtier, vbatt, fv_uv, chg_type);

		} else if (batt_drv->checked_cv_cnt +
			   batt_drv->checked_ov_cnt) {
		/* TAPER_COUNTDOWN: countdown to raise fv_uv and/or check
		 * for tier switch, will keep steady...
		 */
			pr_info("MSC_DLY vt=%d vb=%d fv_uv=%d margin=%d cv_cnt=%d, ov_cnt=%d\n",
				vtier, vbatt, fv_uv, profile->cv_range_accuracy,
				batt_drv->checked_cv_cnt,
				batt_drv->checked_ov_cnt);

			update_interval = profile->cv_update_interval;
			if (batt_drv->checked_cv_cnt)
				batt_drv->checked_cv_cnt -= 1;
			if (batt_drv->checked_ov_cnt)
				batt_drv->checked_ov_cnt -= 1;

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
				fv_uv + profile->fv_uv_resolution);
			update_interval = profile->cv_update_interval;

			/* debounce next taper voltage adjustment */
			batt_drv->checked_cv_cnt = profile->cv_debounce_cnt;

			pr_info("MSC_RAISE vt=%d vb=%d fv_uv=%d->%d\n",
				vtier, vbatt, batt_drv->fv_uv, fv_uv);
		}

		if (batt_drv->checked_cv_cnt > 0) {
		/* debounce period on tier switch */
			pr_info("MSC_WAIT vt=%d vb=%d fv_uv=%d ibatt=%d cv_cnt=%d ov_cnt=%d\n",
				vtier, vbatt, fv_uv, ibatt,
				batt_drv->checked_cv_cnt,
				batt_drv->checked_ov_cnt);
		} else if (-ibatt > cc_next_max) {
		/* current over next tier, reset tier switch count */
			batt_drv->checked_tier_switch_cnt = 0;

			pr_info("MSC_RSTC vt=%d vb=%d fv_uv=%d ibatt=%d cc_next_max=%d t_cnt=%d\n",
				vtier, vbatt, fv_uv, ibatt, cc_next_max,
				batt_drv->checked_tier_switch_cnt);
		} else if (batt_drv->checked_tier_switch_cnt >= switch_cnt) {
		/* next tier, fv_uv detemined at MSC_SET */
			vbatt_idx = batt_drv->vbatt_idx + 1;

			pr_info("MSC_NEXT tier vb=%d ibatt=%d vbatt_idx=%d->%d\n",
				vbatt, ibatt, batt_drv->vbatt_idx, vbatt_idx);
		} else {
		/* current under next tier, increase tier switch count */
			batt_drv->checked_tier_switch_cnt++;

			pr_info("MSC_NYET ibatt=%d cc_next_max=%d t_cnt=%d\n",
				ibatt, cc_next_max,
				batt_drv->checked_tier_switch_cnt);
		}

	}

	/* update fv or cc will change in last tier... */
	if ((vbatt_idx != batt_drv->vbatt_idx)
		|| (temp_idx != batt_drv->temp_idx)
		|| (fv_uv != batt_drv->fv_uv)) {
		const int cc_max = CCCM_LIMITS(profile, temp_idx, vbatt_idx);

		/* need a new fv_uv only on a new voltage tier */
		if (vbatt_idx != batt_drv->vbatt_idx) {
			fv_uv = profile->volt_limits[vbatt_idx];
			batt_drv->checked_tier_switch_cnt = 0;
			batt_drv->checked_ov_cnt = 0;
		}

		pr_info("MSC_SET cv_cnt=%d ov_cnt=%d temp_idx:%d->%d, vbatt_idx:%d->%d, fv=%d->%d, cc_max=%d\n",
			batt_drv->checked_cv_cnt, batt_drv->checked_ov_cnt,
			batt_drv->temp_idx, temp_idx, batt_drv->vbatt_idx,
			vbatt_idx, batt_drv->fv_uv, fv_uv, cc_max);

		batt_drv->vbatt_idx = vbatt_idx;
		batt_drv->temp_idx = temp_idx;
		batt_drv->fv_uv = fv_uv;
		batt_drv->fcc = cc_max;
	}

	return 0;
}

static void google_charge_logic(struct batt_drv *batt_drv)
{
	int err;

	if (!batt_drv->chg_profile.cccm_limits)
		return;

	__pm_stay_awake(&batt_drv->chg_ws);
	pr_debug("battery charging item\n");

	/* TODO: reset batt_drv->rl_status to BATT_RL_STATUS_NONE and
	 * set batt->fg_status to proper value on disconnect.
	 */
	err = google_charge_logic_internal(batt_drv);
	if (err == 0) {
		pr_info("chg_ng: fv_uv=%d fcc=%d\n",
			batt_drv->fv_uv, batt_drv->fcc);
	} else {
		pr_err("chg_ng: ERROR fv_uv=%d fcc=%d\n",
			batt_drv->fv_uv, batt_drv->fcc);
	}

	__pm_relax(&batt_drv->chg_ws);
}

/* TODO: move to google_bms.c */
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

/* TODO: move to google_bms.c ? */
static void batt_init_chg_table(struct chg_profile *profile)
{
	u32 ccm;
	int vi, ti;

	/* chg-battery-capacity is in mAh, chg-cc-limits relative to 100 */
	for (ti = 0; ti < profile->temp_nb_limits - 1; ti++) {
		for (vi = 0; vi < profile->volt_nb_limits; vi++) {
			ccm = CCCM_LIMITS(profile, ti, vi);
			ccm *= profile->battery_capacity * 10;
			// round to the nearest resolution the PMIC can handle
			CCCM_LIMITS(profile, ti, vi) = ccm;
		}
	}
}

/* TODO: factor with google_charger and move to google_bms.c */
static int batt_init_chg_profile(struct batt_drv *batt_drv)
{
	struct device *dev = batt_drv->device;
	struct device_node *node = dev->of_node;
	struct chg_profile *profile = &batt_drv->chg_profile;
	u32 cccm_array_size;
	int ret = 0, vi;

	/* handle retry */
	if (profile->cccm_limits)
		return 0;

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
		profile->volt_limits[vi] = profile->volt_limits[vi];

	cccm_array_size =
	    (profile->temp_nb_limits - 1) * profile->volt_nb_limits;
	profile->cccm_limits = devm_kzalloc(dev,
					    sizeof(s32) * cccm_array_size,
					    GFP_KERNEL);

	ret = of_property_read_u32_array(node, "google,chg-cc-limits",
					 profile->cccm_limits, cccm_array_size);
	if (ret < 0) {
		pr_err("cannot read chg-cc-limits table, ret=%d\n", ret);
		devm_kfree(dev, profile->cccm_limits);
		profile->cccm_limits = 0;
		return ret;
	}


	ret = of_property_read_u32(node, "google,chg-battery-capacity",
				   &profile->battery_capacity);
	if (ret < 0)
		pr_warn("cannot read chg-battery-capacity, ret=%d\n", ret);

	/* taper step, */
	ret = of_property_read_u32(node, "google,fv-uv-resolution",
				   &profile->fv_uv_resolution);
	if (ret < 0)
		profile->fv_uv_resolution = 25000;

	/* IEEE1725, default to 0, 1030 for 3% of VTIER */
	ret = of_property_read_u32(node, "google,fv-uv-margin-dpct",
				   &profile->fv_uv_margin_dpct);
	if (ret < 0)
		profile->fv_uv_margin_dpct = 1020;

	/* how close to a voltage is close enough */
	ret = of_property_read_u32(node, "google,cv-range-accuracy",
				   &profile->cv_range_accuracy);
	if (ret < 0)
		profile->cv_range_accuracy = profile->fv_uv_resolution / 2;

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

	return 0;
}

static int init_debugfs(struct batt_drv *batt_drv)
{
	return 0;

}

static enum power_supply_property gbatt_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CYCLE_COUNTS,
	POWER_SUPPLY_PROP_PRESENT,
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
};

static int gbatt_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	int err = 0;
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

#ifdef SUPPORT_PM_SLEEP
	pm_runtime_get_sync(chip->device);
	if (!chip->init_complete || !chip->resume_complete) {
		pm_runtime_put_sync(chip->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(chip->device);
#endif
	switch (psp) {

	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		/* TODO: route to bms */
		val->intval = -ENODATA;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNTS:
		val->strval = NULL;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&batt_drv->batt_lock);
		val->intval = batt_drv->ssoc_state.ssoc_rl;
		mutex_unlock(&batt_drv->batt_lock);
		break;

	/* ng charging */
	case POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE:
		val->intval = batt_drv->chg_state.v;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		mutex_lock(&batt_drv->chg_lock);
		if (batt_drv->disable_charging || batt_drv->stop_charging ||
		    batt_drv->rl_status == BATT_RL_STATUS_DISCHARGE) {
			val->intval = 0;
		} else {
			val->intval = batt_drv->fv_uv;
		}
		mutex_unlock(&batt_drv->chg_lock);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = batt_drv->fcc;
		mutex_unlock(&batt_drv->chg_lock);
		break;

	/* health */
	case POWER_SUPPLY_PROP_HEALTH:
		if (!batt_drv->fg_psy)
			return -EINVAL;
		err = power_supply_get_property(batt_drv->fg_psy, psp, val);
		if (err == 0)
			batt_drv->soh = val->intval;
		break;
	case POWER_SUPPLY_PROP_SOH:
		val->intval = batt_drv->soh;
		break;
	default:
		if (!batt_drv->fg_psy)
			return -EINVAL;
		err = power_supply_get_property(batt_drv->fg_psy, psp, val);
		break;
	}

	if (err < 0)
		return err;

	return 0;
}

static int gbatt_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

#ifdef SUPPORT_PM_SLEEP
	pm_runtime_get_sync(chip->device);
	if (!chip->init_complete || !chip->resume_complete) {
		pm_runtime_put_sync(chip->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(chip->device);
#endif
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE:
		mutex_lock(&batt_drv->chg_lock);
		batt_drv->chg_state.v = val->intval;

		pr_info("MSC_STATE: val=%x, chg_type=%x vchrg=%d\n",
			batt_drv->chg_state.v,
			batt_drv->chg_state.f.chg_type,
			batt_drv->chg_state.f.vchrg);

		google_charge_logic(batt_drv);
		mutex_unlock(&batt_drv->chg_lock);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int gbatt_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE:
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
	struct power_supply_config psy_cfg = { .drv_data = batt_drv };
	int ret = 0;

	reset_chg_drv_state(batt_drv);
	mutex_init(&batt_drv->chg_lock);
	mutex_init(&batt_drv->batt_lock);


	fg_psy = power_supply_get_by_name(batt_drv->fg_psy_name);
	if (!fg_psy) {
		pr_info("failed to get \"%s\" power supply, retrying...\n",
			batt_drv->fg_psy_name);
		goto retry_init_work;
	}

	batt_drv->fg_psy = fg_psy;

	ret = batt_init_chg_profile(batt_drv);
	if (ret < 0) {
		// No support for charge table
		pr_err("charging profile disabled, ret=%d\n", ret);
	} else {
		struct chg_profile *profile = &batt_drv->chg_profile;

		/* use battery FULL design when capacity is not specified */
		if (profile->battery_capacity == 0) {
			u32 fc;

			fc = PSY_GET_PROP(fg_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
			if (fc == -EINVAL)
				goto retry_init_work;

			profile->battery_capacity = fc / 1000;
		}

		pr_info("successfully read charging profile:\n");
		batt_init_chg_table(profile);
		dump_profile(profile);
	}

	batt_drv->fg_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&batt_drv->fg_nb);
	if (ret < 0)
		pr_err("google_battery: cannot register power supply notifer, ret=%d\n",
			ret);

	wakeup_source_init(&batt_drv->chg_ws, gbatt_psy_desc.name);

	batt_drv->psy = devm_power_supply_register(batt_drv->device,
					       &gbatt_psy_desc, &psy_cfg);
	if (IS_ERR(batt_drv->psy)) {
		ret = PTR_ERR(batt_drv->psy);
		dev_err(batt_drv->device,
			"Couldn't register as power supply, ret=%d\n", ret);
	}

	pr_info("google_battery: init_work done\n");

	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,update-interval",
				   &batt_drv->update_interval);
	if (ret < 0)
		batt_drv->update_interval = DEFAULT_BATT_DRV_UPDATE_INTERVAL;

	schedule_delayed_work(&batt_drv->batt_work, 0);

	return;

retry_init_work:
	schedule_delayed_work(&batt_drv->init_work,
			      msecs_to_jiffies(BATT_DELAY_INIT_MS));
}

static int google_battery_probe(struct platform_device *pdev)
{
	const char *fg_psy_name, *psy_name = NULL;
	struct batt_drv *batt_drv;
	int ret;

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

	init_debugfs(batt_drv);
	INIT_DELAYED_WORK(&batt_drv->init_work, google_battery_init_work);
	INIT_DELAYED_WORK(&batt_drv->batt_work, google_battery_work);
	platform_set_drvdata(pdev, batt_drv);

	schedule_delayed_work(&batt_drv->init_work, 0);

	return 0;
}

static int google_battery_remove(struct platform_device *pdev)
{
	struct batt_drv *batt_drv = platform_get_drvdata(pdev);

	if (batt_drv) {
		if (batt_drv->fg_psy)
			power_supply_put(batt_drv->fg_psy);

		wakeup_source_trash(&batt_drv->chg_ws);
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
