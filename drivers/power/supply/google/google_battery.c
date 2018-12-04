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
#include "google_bms.h"
#include "google_psy.h"


#define BATT_DELAY_INIT_MS 250

#define DEFAULT_BATT_UPDATE_INTERVAL	30000
#define BATT_WORK_ERROR_RETRY_MS		1000

#define DEFAULT_BATT_DRV_RL_SOC_THRESHOLD	97

#define MSC_ERROR_UPDATE_INTERVAL		5000
#define MSC_DEFAULT_UPDATE_INTERVAL		30000

struct batt_ssoc_state {
	int ssoc_gdf;	/* output of gauge data filter */
	int ssoc_uic;	/* output of UI Curves */
	int ssoc_rl;	/* output of rate limiter */
	int ssoc;	/* level% */
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
	struct wakeup_source batt_ws;

	/* TODO: b/111407333, will likely need to adjust SOC% on wakeup */
	bool init_complete;
	bool resume_complete;

	struct mutex batt_lock;
	struct mutex chg_lock;

	/* battery work */
	u32 batt_update_interval;
	/* triger for recharge logic next update from charger */
	bool batt_full;
	struct batt_ssoc_state ssoc_state;

	/* props */
	int soh;

	/* MSC charging */
	bool msc_stop_charging; /* temp outside the charge table */
	struct gbms_chg_profile chg_profile;
	u32 battery_capacity;

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

	/* recharge logic */
	enum batt_rl_status rl_status;
	int batt_rl_soc_threshold;
};

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

static int ssoc_get_real(const struct batt_ssoc_state *ssoc)
{
	return ssoc->ssoc_gdf;
}

/* TODO: b/111407333, apply UI curve to GDF */
static int ssoc_apply_uic(struct batt_ssoc_state *ssoc)
{
	return ssoc->ssoc_gdf;
}

/* TODO: b/111407333, apply rate limiter to UIC */
static int ssoc_apply_rl(struct batt_ssoc_state *ssoc)
{
	return ssoc->ssoc_uic;
}

/* TODO: b/111407333, software state of charge
 * call while holding batt_lock
 */
static int ssoc_work(struct batt_ssoc_state *ssoc, struct power_supply *fg_psy)
{
	int soc;

	/* gauge data filter: make sense of gauge data */
	soc = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_CAPACITY);
	if (soc < 0)
		return -EINVAL;
	ssoc->ssoc_gdf = soc;

	/* apply UI curves: spoof UI @ EOC */
	ssoc->ssoc_uic = ssoc_apply_uic(ssoc);
	/* apply rate limiter: monotonicity and rate of change */
	ssoc->ssoc_rl = ssoc_apply_rl(ssoc);

	return 0;
}

/* reported to userspace
 * call while holding batt_lock
 */
static int ssoc_get_capacity(const struct batt_drv *batt_drv)
{
	/* FULL while in recharge logic, hack until b/111407333 */
	if (batt_drv->rl_status != BATT_RL_STATUS_NONE)
		return 100;
	/* TODO: round to 1% */
	return batt_drv->ssoc_state.ssoc_rl;
}

/* enter recharge logic on charger_DONE or Gauge FULL.
 * NOTE: call holding chg_lock
 * @pre rl_status != BATT_RL_STATUS_NONE
 */
static void batt_rl_enter(struct batt_drv *batt_drv,
			  enum batt_rl_status rl_status)
{
	const int rl_current = batt_drv->rl_status;

	/* enter on discharge is a NO_OP because batt_rl_update_status takes
	 * care of flipping betwee DISCHARGE and RECHARGE */
	if (rl_current == rl_status || rl_current == BATT_RL_STATUS_DISCHARGE)
		return;

	if (rl_current == BATT_RL_STATUS_NONE) {
		/* TODO: modify UI curve for  SSOC=100% @ RAW=95%
		 * NOTE: might need to adjust the curve depending on the entry
		 * rl_status.
		 */
	} else if (rl_current == BATT_RL_STATUS_RECHARGE) {
		/* RECHARGE->DISCHARGE if battery declared 100% before charger.
		 * NOTE: might need to adjust UI curve if the UI curve for enter
		 * on RECHARGE (from battery) is different from the cuve for
		 * entering from charger (in DISCHARGE).
		 */
	}

	batt_drv->rl_status = rl_status;

	/* TODO: should I trigger a ps change? */
}

/* just reset state, no PS notifications.
 * NOTE: call holding chg_lock
 */
static void batt_rl_reset(struct batt_drv *batt_drv)
{
	batt_drv->rl_status = BATT_RL_STATUS_NONE;
}

/* RL recharge: after SSOC work, restart charging.
 * NOTE: call holding chg_lock
 */
static void batt_rl_update_status(struct batt_drv *batt_drv)
{
	struct batt_ssoc_state *ssoc = &batt_drv->ssoc_state;
	int soc;

	/* already in _RECHARGE or _NONE, done */
	if (batt_drv->rl_status != BATT_RL_STATUS_DISCHARGE)
		return;

	/* recharge logic work on real soc */
	soc = ssoc_get_real(ssoc);
	if (batt_drv->batt_rl_soc_threshold &&
	    soc <= batt_drv->batt_rl_soc_threshold) {

		batt_drv->rl_status = BATT_RL_STATUS_RECHARGE;
		if (batt_drv->psy)
			power_supply_changed(batt_drv->psy);
	}

}

/* ------------------------------------------------------------------------- */

static inline void batt_reset_chg_drv_state(struct batt_drv *batt_drv)
{
	batt_drv->temp_idx = -1;
	batt_drv->vbatt_idx = -1;
	batt_drv->fv_uv = -1;
	batt_drv->cc_max = -1;
	batt_drv->checked_cv_cnt = 0;
	batt_drv->checked_ov_cnt = 0;
	batt_drv->checked_tier_switch_cnt = 0;
	batt_drv->msc_stop_charging = true;
	batt_drv->msc_update_interval = -1;
	batt_rl_reset(batt_drv);
}

static int msc_logic_internal(struct batt_drv *batt_drv)
{
	struct power_supply *fg_psy = batt_drv->fg_psy;
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	int vbatt_idx = batt_drv->vbatt_idx, fv_uv = batt_drv->fv_uv, temp_idx;
	int temp, ibatt, vbatt, vchrg, chg_type;
	int update_interval = MSC_DEFAULT_UPDATE_INTERVAL;

	temp = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_TEMP);
	if (temp == -EINVAL)
		return -EIO;

	/* sort of software JEITA: need to be able to disable (leave to HW) */
	if (temp < profile->temp_limits[0] ||
	    temp > profile->temp_limits[profile->temp_nb_limits - 1]) {
		if (!batt_drv->msc_stop_charging) {
			pr_info("batt. temp=%d off limits, disabling charging\n",
				temp);
			batt_reset_chg_drv_state(batt_drv);
		}

		return 0;
	}

	if (batt_drv->msc_stop_charging) {
		pr_info("batt. temp=%d ok, enabling charging\n", temp);
		batt_drv->msc_stop_charging = false;
	}

	ibatt = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	vbatt = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (ibatt == -EINVAL || vbatt == -EINVAL)
		return -EIO;

	/* invalid vchg disable IDROP compensation in FAST */
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

	/* need a new fv_uv only on a new voltage tier */
	if (vbatt_idx != batt_drv->vbatt_idx) {
		fv_uv = profile->volt_limits[vbatt_idx];
		batt_drv->checked_tier_switch_cnt = 0;
		batt_drv->checked_ov_cnt = 0;
	}

	batt_drv->vbatt_idx = vbatt_idx;
	batt_drv->temp_idx = temp_idx;
	batt_drv->cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, vbatt_idx);
	batt_drv->fv_uv = fv_uv;
	/* next update */
	batt_drv->msc_update_interval = update_interval;

	pr_info("MSC_SET cv_cnt=%d ov_cnt=%d temp_idx:%d->%d, vbatt_idx:%d->%d, fv=%d->%d, cc_max=%d\n",
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
	union gbms_charger_state *chg_state = &batt_drv->chg_state;

	if (!batt_drv->chg_profile.cccm_limits)
		return -EINVAL;

	__pm_stay_awake(&batt_drv->batt_ws);

	pr_info("MSC_DIN chg_state=%lx f=0x%x chg_s=%s chg_t=%s vchg=%d\n",
		(unsigned long)chg_state->v,
		chg_state->f.flags,
		gbms_chg_status_s(chg_state->f.chg_status),
		gbms_chg_type_s(chg_state->f.chg_type),
		chg_state->f.vchrg);

	/* here google_charger trigger the recharge logic and get out of it. */
	if ((batt_drv->chg_state.f.flags & GBMS_CS_FLAG_BUCK_EN) == 0) {
		/* also reset recharge logic */
		batt_reset_chg_drv_state(batt_drv);
		goto exit_msc_logic;
	} else if ((batt_drv->chg_state.f.flags & GBMS_CS_FLAG_DONE) != 0) {
		/* normal enter on full */
		batt_rl_enter(batt_drv, BATT_RL_STATUS_DISCHARGE);
	} else if (batt_drv->batt_full) {
		/* will enter RL only when state is NONE */
		batt_rl_enter(batt_drv, BATT_RL_STATUS_RECHARGE);
	}

	err = msc_logic_internal(batt_drv);
	if (err == 0) {
		pr_info("MSC_OUT fv_uv=%d cc_max=%d update_interval=%d\n",
			batt_drv->fv_uv,
			batt_drv->cc_max,
			batt_drv->msc_update_interval);
	} else {
		/* NOTE: google charger will poll again. */
		batt_drv->msc_update_interval = -1;
		pr_err("MSC_OUT ERROR=%d fv_uv=%d cc_max=%d update_interval=%d\n",
			err, batt_drv->fv_uv, batt_drv->cc_max,
			batt_drv->msc_update_interval);
	}

exit_msc_logic:

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

	/* NOTE: might trigger google charger */
	if (!batt_drv->msc_interval_votable)
		batt_drv->msc_interval_votable =
			find_votable(VOTABLE_MSC_INTERVAL);
	if (batt_drv->msc_interval_votable)
		vote(batt_drv->msc_interval_votable, MSC_LOGIC_VOTER,
			 batt_drv->msc_update_interval != -1,
			 batt_drv->msc_update_interval);

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
	if (profile->cccm_limits)
		return 0;

	ret = gbms_init_chg_profile(profile, node);
	if (ret < 0)
		return -EINVAL;

	ret = of_property_read_u32(node, "google,chg-battery-capacity",
				   &batt_drv->battery_capacity);
	if (ret < 0)
		pr_warn("read chg-battery-capacity from gauge\n");

	return 0;
}

/* ------------------------------------------------------------------------- */

/* poll the battery, run SOC% etc, scheduled from psy_changed and from timer */
static void google_battery_work(struct work_struct *work)
{
	struct batt_drv *batt_drv =
	    container_of(work, struct batt_drv, batt_work.work);
	struct power_supply *fg_psy = batt_drv->fg_psy;
	struct batt_ssoc_state *ssoc = &batt_drv->ssoc_state;
	int update_interval = batt_drv->batt_update_interval;
	int fg_status, ret;

	pr_debug("battery work item\n");

	__pm_stay_awake(&batt_drv->batt_ws);

	mutex_lock(&batt_drv->chg_lock);
	fg_status = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_STATUS);
	if (fg_status < 0)
		goto reschedule;

	/* fuel gauge triggered recharge logic */
	batt_drv->batt_full = (fg_status == POWER_SUPPLY_STATUS_FULL);

	mutex_lock(&batt_drv->batt_lock);
	ret = ssoc_work(ssoc, fg_psy);

	pr_info("SSOC: [ssoc=%d%% gdf=%d, uic=%d, rl=%d] rls=%d\n",
		ssoc_get_capacity(batt_drv),
		ssoc->ssoc_gdf, ssoc->ssoc_uic, ssoc->ssoc_rl,
		batt_drv->rl_status);

	/* TODO: poll other data here */

	mutex_unlock(&batt_drv->batt_lock);

	if (ret < 0) {
		update_interval = BATT_WORK_ERROR_RETRY_MS;
	} else {
		/* handle charge/recharge */
		batt_rl_update_status(batt_drv);
	}

reschedule:
	mutex_unlock(&batt_drv->chg_lock);

	if (update_interval) {
		pr_debug("rerun battery work in %d ms\n", update_interval);
		schedule_delayed_work(&batt_drv->batt_work,
				      msecs_to_jiffies(update_interval));
	}

	__pm_relax(&batt_drv->batt_ws);
}

/* ------------------------------------------------------------------------- */


static enum power_supply_property gbatt_battery_props[] = {
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
};

static int gbatt_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	int err = 0;
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);

#ifdef SUPPORT_PM_SLEEP
	pm_runtime_get_sync(chip->device);
	if (!chip->init_complete || !chip->resume_complete) {
		pm_runtime_put_sync(chip->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(chip->device);
#endif
	switch (psp) {

	case POWER_SUPPLY_PROP_CYCLE_COUNTS:
		val->strval = NULL;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&batt_drv->batt_lock);
		val->intval = ssoc_get_capacity(batt_drv);
		mutex_unlock(&batt_drv->batt_lock);
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
		if (batt_drv->msc_stop_charging ||
		    batt_drv->rl_status == BATT_RL_STATUS_DISCHARGE) {
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
	 */
	case POWER_SUPPLY_PROP_CHARGE_DONE:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = (batt_drv->rl_status == BATT_RL_STATUS_DISCHARGE);
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

	case POWER_SUPPLY_PROP_RECHARGE_SOC:
		val->intval = batt_drv->batt_rl_soc_threshold;
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
	int ret = 0;

#ifdef SUPPORT_PM_SLEEP
	pm_runtime_get_sync(chip->device);
	if (!chip->init_complete || !chip->resume_power_supply_get_by_name) {
		pm_runtime_put_sync(chip->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(chip->device);
#endif
	switch (psp) {
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
		} else if (batt_drv->batt_rl_soc_threshold != val->intval) {
			batt_drv->batt_rl_soc_threshold = val->intval;
			if (batt_drv->psy)
				power_supply_changed(batt_drv->psy);
		}
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
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SW_JEITA_ENABLED:
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

	batt_reset_chg_drv_state(batt_drv);
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
		/* No support for charge table, legacy */
		pr_err("charging profile disabled, ret=%d\n", ret);
	} else {
		struct gbms_chg_profile *profile = &batt_drv->chg_profile;

		/* use battery FULL design when capacity is not specified */
		if (batt_drv->battery_capacity == 0) {
			u32 fc;

			fc = GPSY_GET_PROP(fg_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
			if (fc == -EINVAL)
				goto retry_init_work;

			/* convert uA to mAh*/
			batt_drv->battery_capacity = fc / 1000;
		}

		/* NOTE: with NG charger tolerance is applied from "charger" */
		gbms_init_chg_table(profile, batt_drv->battery_capacity);
		pr_info("successfully read charging profile:\n");
		gbms_dump_chg_profile(profile);

		/* recharge logic */
		ret = of_property_read_u32(batt_drv->device->of_node,
					"google,recharge-soc-threshold",
					&batt_drv->batt_rl_soc_threshold);
		if (ret < 0)
			batt_drv->batt_rl_soc_threshold =
				DEFAULT_BATT_DRV_RL_SOC_THRESHOLD;

	}

	batt_drv->fg_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&batt_drv->fg_nb);
	if (ret < 0)
		pr_err("google_battery: cannot register power supply notifer, ret=%d\n",
			ret);

	wakeup_source_init(&batt_drv->batt_ws, gbatt_psy_desc.name);

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
				   &batt_drv->batt_update_interval);
	if (ret < 0)
		batt_drv->batt_update_interval = DEFAULT_BATT_UPDATE_INTERVAL;

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

		gbms_free_chg_profile(&batt_drv->chg_profile);

		wakeup_source_trash(&batt_drv->batt_ws);
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
