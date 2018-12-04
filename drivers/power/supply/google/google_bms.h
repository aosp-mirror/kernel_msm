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

#ifndef __GOOGLE_BMS_H_
#define __GOOGLE_BMS_H_

#include <linux/types.h>

struct device_node;

#define GBMS_CHG_TEMP_NB_LIMITS_MAX 10
#define GBMS_CHG_VOLT_NB_LIMITS_MAX 5

struct gbms_chg_profile {
	const char *owner_name;

	int temp_nb_limits;
	s32 temp_limits[GBMS_CHG_TEMP_NB_LIMITS_MAX];
	int volt_nb_limits;
	s32 volt_limits[GBMS_CHG_VOLT_NB_LIMITS_MAX];
	/* Array of constant current limits */
	s32 *cccm_limits;

	/* behavior */
	u32 fv_uv_margin_dpct;
	u32 cv_range_accuracy;
	u32 cv_debounce_cnt;
	u32 cv_update_interval;
	u32 cv_tier_ov_cnt;
	u32 cv_tier_switch_cnt;
	/* taper step */
	u32 fv_uv_resolution;
	/* experimental */
	u32 cv_otv_margin;
};


#define GBMS_CCCM_LIMITS(profile, ti, vi) \
	profile->cccm_limits[(ti * profile->volt_nb_limits) + vi]

/* newgen charging */
#define GBMS_CS_FLAG_BUCK_EN    (1 << 0)
#define GBMS_CS_FLAG_DONE       (1 << 1)
#define GBMS_CS_FLAG_CC       	(1 << 2)
#define GBMS_CS_FLAG_CV       	(1 << 3)
#define GBMS_CS_FLAG_ILIM       (1 << 4)

union gbms_charger_state {
	uint64_t v;
	struct {
		uint8_t flags;
		uint8_t pad;
		uint8_t chg_status;
		uint8_t chg_type;
		uint16_t vchrg;
		uint8_t pad1[2];
	} f;
};

int gbms_init_chg_profile_internal(struct gbms_chg_profile *profile,
			  struct device_node *node, const char *owner_name);
#define gbms_init_chg_profile(p, n) \
	gbms_init_chg_profile_internal(p, n, KBUILD_MODNAME)

void gbms_init_chg_table(struct gbms_chg_profile *profile, u32 capacity);

void gbms_free_chg_profile(struct gbms_chg_profile *profile);

void gbms_dump_raw_profile(const struct gbms_chg_profile *profile, int scale);
#define gbms_dump_chg_profile(profile) gbms_dump_raw_profile(profile, 1000)

/* newgen charging: charge profile */
int gbms_msc_temp_idx(const struct gbms_chg_profile *profile, int temp);
int gbms_msc_voltage_idx(const struct gbms_chg_profile *profile, int vbatt);
int gbms_msc_round_fv_uv(const struct gbms_chg_profile *profile,
			   int vtier, int fv_uv);

/* newgen charging: charger flags  */
uint8_t gbms_gen_chg_flags(int chg_status, int chg_type);

/* debug/print */
const char *gbms_chg_type_s(int chg_type);
const char *gbms_chg_status_s(int chg_status);


/* Votables */
#define VOTABLE_MSC_INTERVAL	"MSC_INTERVAL"
#define VOTABLE_MSC_FCC		"MSC_FCC"
#define VOTABLE_MSC_FV		"MSC_FV"

/* Voters */

#endif  /* __GOOGLE_BMS_H_ */
