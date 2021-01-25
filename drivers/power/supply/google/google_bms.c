/*
 * Google Battery Management System
 *
 * Copyright (C) 2018 Google Inc.
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

#define gbms_owner(p)	((p)->owner_name ? (p)->owner_name : "google_bms")

#define gbms_info(p, fmt, ...)	\
	pr_info("%s: " fmt, gbms_owner(p), ##__VA_ARGS__)
#define gbms_warn(p, fmt, ...)	\
	pr_warn("%s: " fmt, gbms_owner(p), ##__VA_ARGS__)
#define gbms_err(p, fmt, ...)	\
	pr_err("%s: " fmt, gbms_owner(p), ##__VA_ARGS__)

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/power_supply.h>

#include "google_bms.h"

#define GBMS_DEFAULT_FV_UV_RESOLUTION   25000
#define GBMS_DEFAULT_FV_UV_MARGIN_DPCT  1020
#define GBMS_DEFAULT_CV_DEBOUNCE_CNT    3
#define GBMS_DEFAULT_CV_UPDATE_INTERVAL 2000
#define GBMS_DEFAULT_CV_TIER_OV_CNT     10
#define GBMS_DEFAULT_CV_TIER_SWITCH_CNT 3
#define GBMS_DEFAULT_CV_OTV_MARGIN      0
#define GBMS_DEFAULT_CHG_LAST_TIER_DEC_CURRENT  50000
#define GBMS_DEFAULT_CHG_LAST_TIER_TERM_CURRENT 150000

static const char *psy_chgt_str[] = {
	"Unknown", "None", "Trickle", "Fast", "Taper"
};

const char *gbms_chg_type_s(int cgh_type)
{
	if (cgh_type < 0 || cgh_type > ARRAY_SIZE(psy_chgt_str))
		return "<err>";
	return psy_chgt_str[cgh_type];
}

static const char *psy_chgs_str[] = {
	"Unknown", "Charging", "Discharging", "Not Charging", "Full"
};

const char *gbms_chg_status_s(int chg_status)
{
	if (chg_status < 0 || chg_status > ARRAY_SIZE(psy_chgs_str))
		return "<err>";
	return psy_chgs_str[chg_status];
}


const char *gbms_chg_ev_adapter_s(int adapter)
{
	static char *chg_ev_adapter_type_str[] = {
		FOREACH_CHG_EV_ADAPTER(CHG_EV_ADAPTER_STRING)
	};

	if (adapter < 0 || adapter > ARRAY_SIZE(chg_ev_adapter_type_str))
		return "<err>";
	return chg_ev_adapter_type_str[adapter];
}

/* convert C rates to current. Caller can account for tolerances reducing
 * battery_capacity. fv_uv_resolution is used to create discrete steps.
 * NOTE: the call covert C rates to chanrge currents IN PLACE, ie you cannot
 * call this twice.
 */
void gbms_init_chg_table(struct gbms_chg_profile *profile, u32 capacity_ma)
{
	u32 ccm;
	int vi, ti;
	const int fv_uv_step = profile->fv_uv_resolution;

	profile->capacity_ma = capacity_ma;

	/* chg-battery-capacity is in mAh, chg-cc-limits relative to 100 */
	for (ti = 0; ti < profile->temp_nb_limits - 1; ti++) {
		for (vi = 0; vi < profile->volt_nb_limits; vi++) {
			ccm = GBMS_CCCM_LIMITS(profile, ti, vi);
			ccm *= capacity_ma * 10;

			/* round to the nearest resolution */
			if (fv_uv_step)
				ccm = DIV_ROUND_CLOSEST(ccm, fv_uv_step)
					* fv_uv_step;

			GBMS_CCCM_LIMITS(profile, ti, vi) = ccm;
		}
	}
}

/* configure standard device charge profile properties */
static int gbms_read_cccm_limits(struct gbms_chg_profile *profile,
				 struct device_node *node)
{
	int ret;

	profile->temp_nb_limits =
	    of_property_count_elems_of_size(node, "google,chg-temp-limits",
					    sizeof(u32));
	if (profile->temp_nb_limits <= 0) {
		ret = profile->temp_nb_limits;
		gbms_err(profile, "cannot read chg-temp-limits, ret=%d\n", ret);
		return ret;
	}
	if (profile->temp_nb_limits > GBMS_CHG_TEMP_NB_LIMITS_MAX) {
		gbms_err(profile, "chg-temp-nb-limits exceeds driver max: %d\n",
		       GBMS_CHG_TEMP_NB_LIMITS_MAX);
		return -EINVAL;
	}
	ret = of_property_read_u32_array(node, "google,chg-temp-limits",
					 (u32 *)profile->temp_limits,
					 profile->temp_nb_limits);
	if (ret < 0) {
		gbms_err(profile, "cannot read chg-temp-limits table, ret=%d\n",
			 ret);
		return ret;
	}

	profile->volt_nb_limits =
	    of_property_count_elems_of_size(node, "google,chg-cv-limits",
					    sizeof(u32));
	if (profile->volt_nb_limits <= 0) {
		ret = profile->volt_nb_limits;
		gbms_err(profile, "cannot read chg-cv-limits, ret=%d\n", ret);
		return ret;
	}
	if (profile->volt_nb_limits > GBMS_CHG_VOLT_NB_LIMITS_MAX) {
		gbms_err(profile, "chg-cv-nb-limits exceeds driver max: %d\n",
		       GBMS_CHG_VOLT_NB_LIMITS_MAX);
		return -EINVAL;
	}
	ret = of_property_read_u32_array(node, "google,chg-cv-limits",
					 (u32 *)profile->volt_limits,
					 profile->volt_nb_limits);
	if (ret < 0) {
		gbms_err(profile, "cannot read chg-cv-limits table, ret=%d\n",
			 ret);
		return ret;
	}

	return 0;
}

int gbms_init_chg_profile_internal(struct gbms_chg_profile *profile,
			  struct device_node *node,
			  const char *owner_name)
{
	int ret, vi;
	u32 cccm_array_size, mem_size;

	profile->owner_name = owner_name;

	ret = gbms_read_cccm_limits(profile, node);
	if (ret < 0)
		return ret;

	cccm_array_size = (profile->temp_nb_limits - 1)
			  * profile->volt_nb_limits;
	mem_size = sizeof(s32) * cccm_array_size;

	profile->cccm_limits = kzalloc(mem_size, GFP_KERNEL);
	if (!profile->cccm_limits)
		return -ENOMEM;

	/* load C rates into profile->cccm_limits */
	ret = of_property_read_u32_array(node, "google,chg-cc-limits",
					 profile->cccm_limits,
					 cccm_array_size);
	if (ret < 0) {
		gbms_err(profile, "cannot read chg-cc-limits table, ret=%d\n",
			 ret);
		kfree(profile->cccm_limits);
		profile->cccm_limits = 0;
		return -EINVAL;
	}

	/* for irdrop compensation in taper step */
	ret = of_property_read_u32(node, "google,fv-uv-resolution",
				   &profile->fv_uv_resolution);
	if (ret < 0)
		profile->fv_uv_resolution = GBMS_DEFAULT_FV_UV_RESOLUTION;

	/* how close to tier voltage is close enough */
	ret = of_property_read_u32(node, "google,cv-range-accuracy",
				   &profile->cv_range_accuracy);
	if (ret < 0)
		profile->cv_range_accuracy = profile->fv_uv_resolution / 2;

	/* IEEE1725, default to 1020, cap irdrop offset */
	ret = of_property_read_u32(node, "google,fv-uv-margin-dpct",
				   &profile->fv_uv_margin_dpct);
	if (ret < 0)
		profile->fv_uv_margin_dpct = GBMS_DEFAULT_FV_UV_MARGIN_DPCT;

	/* debounce tier switch */
	ret = of_property_read_u32(node, "google,cv-debounce-cnt",
				   &profile->cv_debounce_cnt);
	if (ret < 0)
		profile->cv_debounce_cnt = GBMS_DEFAULT_CV_DEBOUNCE_CNT;

	/* how fast to poll in taper */
	ret = of_property_read_u32(node, "google,cv-update-interval",
				   &profile->cv_update_interval);
	if (ret < 0)
		profile->cv_update_interval = GBMS_DEFAULT_CV_UPDATE_INTERVAL;

	/* tier over voltage penalty */
	ret = of_property_read_u32(node, "google,cv-tier-ov-cnt",
				   &profile->cv_tier_ov_cnt);
	if (ret < 0)
		profile->cv_tier_ov_cnt = GBMS_DEFAULT_CV_TIER_OV_CNT;

	/* how many samples under next tier to wait before switching */
	ret = of_property_read_u32(node, "google,cv-tier-switch-cnt",
				   &profile->cv_tier_switch_cnt);
	if (ret < 0)
		profile->cv_tier_switch_cnt = GBMS_DEFAULT_CV_TIER_SWITCH_CNT;

	/* allow being "a little" over tier voltage, experimental */
	ret = of_property_read_u32(node, "google,cv-otv-margin",
				   &profile->cv_otv_margin);
	if (ret < 0)
		profile->cv_otv_margin = GBMS_DEFAULT_CV_OTV_MARGIN;

	/* sanity on voltages (should warn?) */
	for (vi = 0; vi < profile->volt_nb_limits; vi++)
		profile->volt_limits[vi] = profile->volt_limits[vi] /
		    profile->fv_uv_resolution * profile->fv_uv_resolution;

	ret = of_property_read_u32(node,
			    "google,chg-last-tier-vpack-tolerance",
			    &profile->chg_last_tier_vpack_tolerance);
	if (ret < 0)
		profile->chg_last_tier_vpack_tolerance = 0;

	if (profile->chg_last_tier_vpack_tolerance > 0) {
		ret = of_property_read_u32(node,
			    "google,chg-last-tier-dec-current",
			    &profile->chg_last_tier_dec_current);
		if (ret < 0)
			profile->chg_last_tier_dec_current =
				    GBMS_DEFAULT_CHG_LAST_TIER_DEC_CURRENT;
		ret = of_property_read_u32(node,
			    "google,chg-last-tier-term-current",
			    &profile->chg_last_tier_term_current);
		if (ret < 0)
			profile->chg_last_tier_term_current =
				    GBMS_DEFAULT_CHG_LAST_TIER_TERM_CURRENT;
	}

	return 0;
}

void gbms_free_chg_profile(struct gbms_chg_profile *profile)
{
	kfree(profile->cccm_limits);
	profile->cccm_limits = 0;
}

/* NOTE: I should really pass the scale */
void gbms_dump_raw_profile(const struct gbms_chg_profile *profile, int scale)
{
	const int tscale = (scale == 1) ? 1 : 10;
	/* with scale == 1 voltage takes 7 bytes, add 7 bytes of temperature */
	char buff[GBMS_CHG_VOLT_NB_LIMITS_MAX * 9 + 7];
	int ti, vi, count, len = sizeof(buff);

	gbms_info(profile, "Profile constant charge limits:\n");
	count = 0;
	for (vi = 0; vi < profile->volt_nb_limits; vi++) {
		count += scnprintf(buff + count, len - count, "  %4d",
				   profile->volt_limits[vi] / scale);
	}
	gbms_info(profile, "|T \\ V%s\n", buff);

	for (ti = 0; ti < profile->temp_nb_limits - 1; ti++) {
		count = 0;
		count += scnprintf(buff + count, len - count, "|%2d:%2d",
				   profile->temp_limits[ti] / tscale,
				   profile->temp_limits[ti + 1] / tscale);
		for (vi = 0; vi < profile->volt_nb_limits; vi++) {
			count += scnprintf(buff + count, len - count, "  %4d",
					   GBMS_CCCM_LIMITS(profile, ti, vi)
					   / scale);
		}
		gbms_info(profile, "%s\n", buff);
	}
}

int gbms_msc_round_fv_uv(const struct gbms_chg_profile *profile,
			   int vtier, int fv_uv)
{
	int result;
	const unsigned int fv_uv_max = (vtier / 1000)
					* profile->fv_uv_margin_dpct;

	if (fv_uv_max != 0 && fv_uv > fv_uv_max)
		fv_uv = fv_uv_max;

	result = fv_uv - (fv_uv % profile->fv_uv_resolution);

	if (fv_uv_max != 0)
		gbms_info(profile, "MSC_ROUND: fv_uv=%d vtier=%d fv_uv_max=%d -> %d\n",
			fv_uv, vtier, fv_uv_max, result);

	return result;
}

/* charge profile idx based on the battery temperature
 * TODO: return -1 when temperature is lower than profile->temp_limits[0] or
 * higher than profile->temp_limits[profile->temp_nb_limits - 1]
 */
int gbms_msc_temp_idx(const struct gbms_chg_profile *profile, int temp)
{
	int temp_idx = 0;

	while (temp_idx < profile->temp_nb_limits - 1 &&
	       temp >= profile->temp_limits[temp_idx + 1])
		temp_idx++;

	return temp_idx;
}

/* Compute the step index given the battery voltage
 * When selecting an index need to make sure that headroom for the tier voltage
 * will allow to send to the battery _at least_ next tier max FCC current and
 * well over charge termination current.
 */
int gbms_msc_voltage_idx(const struct gbms_chg_profile *profile, int vbatt)
{
	int vbatt_idx = 0;

	while (vbatt_idx < profile->volt_nb_limits - 1 &&
	       vbatt > profile->volt_limits[vbatt_idx])
		vbatt_idx++;

	/* assumes that 3 times the hardware resolution is ok
	 * TODO: make it configurable? tune?
	 */
	if (vbatt_idx != profile->volt_nb_limits - 1) {
		const int vt = profile->volt_limits[vbatt_idx];
		const int headr = profile->fv_uv_resolution * 3;

		if ((vt - vbatt) < headr)
			vbatt_idx += 1;
	}

	return vbatt_idx;
}

uint8_t gbms_gen_chg_flags(int chg_status, int chg_type)
{
	uint8_t flags = 0;

	if (chg_status != POWER_SUPPLY_STATUS_DISCHARGING) {
		flags |= GBMS_CS_FLAG_BUCK_EN;

		/* FULL makes sense only when charging is enabled */
		if (chg_status == POWER_SUPPLY_STATUS_FULL)
			flags |= GBMS_CS_FLAG_DONE;
	}
	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST)
		flags |= GBMS_CS_FLAG_CC;
	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_TAPER)
		flags |= GBMS_CS_FLAG_CV;

	return flags;
}

/* ------------------------------------------------------------------------- */

/* convert cycle counts array to string */
int gbms_cycle_count_cstr_bc(char *buf, size_t size,
				const u16 *ccount, int bcnt)
{
	int len = 0, i;

	for (i = 0; i < bcnt; i++)
		len += scnprintf(buf + len, size - len, "%d ", ccount[i]);
	buf[len - 1] = '\n';

	return len;
}

/* parse the result of gbms_cycle_count_cstr_bc() back to array */
int gbms_cycle_count_sscan_bc(u16 *ccount, int bcnt, const char *buff)
{
	int i, val[10];

	/* sscanf has 10 fixed conversions */
	if (bcnt != GBMS_CCBIN_BUCKET_COUNT)
		return -ERANGE;

	if (sscanf(buff, "%d %d %d %d %d %d %d %d %d %d",
			&val[0], &val[1], &val[2], &val[3], &val[4],
			&val[5], &val[6], &val[7], &val[8], &val[9])
			!= bcnt)
		return -EINVAL;

	for (i = 0; i < bcnt ; i++)
		if (val[i] >= 0 && val[i] < U16_MAX)
			ccount[i] = val[i];

	return 0;
}

bool gbms_temp_defend_dry_run(bool update, bool dry_run)
{
	static bool is_dry_run;

	if (update)
		is_dry_run = dry_run;

	return is_dry_run;
}
