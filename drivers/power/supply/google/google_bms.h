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
#include "qmath.h"

struct device_node;

#define GBMS_CHG_TEMP_NB_LIMITS_MAX 10
#define GBMS_CHG_VOLT_NB_LIMITS_MAX 6

struct gbms_chg_profile {
	const char *owner_name;

	int temp_nb_limits;
	s32 temp_limits[GBMS_CHG_TEMP_NB_LIMITS_MAX];
	int volt_nb_limits;
	s32 volt_limits[GBMS_CHG_VOLT_NB_LIMITS_MAX];
	/* Array of constant current limits */
	s32 *cccm_limits;
	/* used to fill table  */
	u32 capacity_ma;

	/* behavior */
	u32 fv_uv_margin_dpct;
	u32 cv_range_accuracy;
	u32 cv_debounce_cnt;
	u32 cv_update_interval;
	u32 cv_tier_ov_cnt;
	u32 cv_tier_switch_cnt;
	u32 chg_last_tier_vpack_tolerance;
	u32 chg_last_tier_dec_current;
	u32 chg_last_tier_term_current;
	/* taper step */
	u32 fv_uv_resolution;
	/* experimental */
	u32 cv_otv_margin;
};

#define WLC_BPP_THRESHOLD_UV	700000
#define WLC_EPP_THRESHOLD_UV	1100000

#define FOREACH_CHG_EV_ADAPTER(S)		\
	S(UNKNOWN), 	\
	S(USB),		\
	S(USB_SDP),	\
	S(USB_DCP),	\
	S(USB_CDP),	\
	S(USB_ACA),	\
	S(USB_C),	\
	S(USB_PD),	\
	S(USB_PD_DRP),	\
	S(USB_PD_PPS),	\
	S(USB_BRICKID),	\
	S(USB_HVDCP),	\
	S(USB_HVDCP3),	\
	S(USB_FLOAT),	\
	S(WLC),		\
	S(WLC_EPP),	\
	S(WLC_SPP),	\

#define CHG_EV_ADAPTER_STRING(s)	#s
#define _CHG_EV_ADAPTER_PRIMITIVE_CAT(a, ...) a ## __VA_ARGS__

/* Enums will start with CHG_EV_ADAPTER_TYPE_ */
#define CHG_EV_ADAPTER_ENUM(e)	\
			_CHG_EV_ADAPTER_PRIMITIVE_CAT(CHG_EV_ADAPTER_TYPE_,e)

enum chg_ev_adapter_type_t {
	FOREACH_CHG_EV_ADAPTER(CHG_EV_ADAPTER_ENUM)
};

enum gbms_msc_states_t {
	MSC_NONE = 0,
	MSC_SEED,
	MSC_DSG,
	MSC_LAST,
	MSC_VSWITCH,
	MSC_VOVER,
	MSC_PULLBACK,
	MSC_FAST,
	MSC_TYPE,
	MSC_DLY,	/* in taper */
	MSC_STEADY,	/* in taper */
	MSC_TIERCNTING, /* in taper */
	MSC_RAISE,	/* in taper */
	MSC_WAIT,	/* in taper */
	MSC_RSTC,	/* in taper */
	MSC_NEXT,	/* in taper */
	MSC_NYET,	/* in taper */
	MSC_HEALTH,
	MSC_STATES_COUNT,
};

union gbms_ce_adapter_details {
	uint32_t	v;
	struct {
		uint8_t		ad_type;
		uint8_t		pad;
		uint8_t 	ad_voltage;
		uint8_t 	ad_amperage;
	};
};

struct gbms_ce_stats {
	uint16_t 	voltage_in;
	uint16_t	ssoc_in;
	uint16_t	cc_in;
	uint16_t 	voltage_out;
	uint16_t 	ssoc_out;
	uint16_t	cc_out;
};

struct ttf_tier_stat {
	int16_t soc_in;
	int	cc_in;
	int	cc_total;
	time_t	avg_time;
};

struct gbms_ce_tier_stats {
	int8_t		temp_idx;
	int8_t		vtier_idx;

	int16_t		soc_in;		/* 8.8 */
	uint16_t	cc_in;
	uint16_t	cc_total;

	uint32_t	time_fast;
	uint32_t	time_taper;
	uint32_t	time_other;

	int16_t		temp_in;
	int16_t		temp_min;
	int16_t		temp_max;

	int16_t		ibatt_min;
	int16_t		ibatt_max;

	uint16_t	icl_min;
	uint16_t	icl_max;

	int64_t		icl_sum;
	int64_t		temp_sum;
	int64_t		ibatt_sum;
	uint32_t 	sample_count;

	uint16_t 	msc_cnt[MSC_STATES_COUNT];
	uint32_t 	msc_elap[MSC_STATES_COUNT];
};

#define GBMS_STATS_TIER_COUNT	3
#define GBMS_SOC_STATS_LEN	101

/* time to full */

/* collected in charging event */
struct ttf_soc_stats {
	int ti[GBMS_SOC_STATS_LEN];		/* charge tier at each soc */
	int cc[GBMS_SOC_STATS_LEN];		/* coulomb count at each soc */
	time_t elap[GBMS_SOC_STATS_LEN];	/* time spent at soc */
};

/* reference data for soc estimation  */
struct ttf_adapter_stats {
	u32 *soc_table;
	u32 *elap_table;
	int table_count;
};

/* updated when the device publish the charge stats
 * NOTE: soc_stats and tier_stats are only valid for the given chg_profile
 * since tier, coulumb count and elap time spent at each SOC depends on the
 * maximum amout of current that can be pushed to the battery.
 */
struct batt_ttf_stats {
	time_t ttf_fake;

	struct ttf_soc_stats soc_ref;	/* gold: soc->elap,cc */
	int ref_temp_idx;
	int ref_watts;

	struct ttf_soc_stats soc_stats; /* rolling */
	struct ttf_tier_stat tier_stats[GBMS_STATS_TIER_COUNT];

	struct logbuffer *ttf_log;
};

/*
 * health based changing can be enabled from userspace with a deadline
 *
 * initial state:
 *	deadline = 0, rest_state = CHG_HEALTH_INACTIVE
 *
 * deadline = -1 from userspace
 *	CHG_HEALTH_* -> CHG_HEALTH_USER_DISABLED (settings disabled)
 * on deadline = 0 from userspace
 *	CHG_HEALTH_* -> CHG_HEALTH_USER_DISABLED (alarm, plug or misc. disabled)
 * on deadline > 0 from userspace
 *	CHG_HEALTH_* -> CHG_HEALTH_ENABLED
 *
 *  from CHG_HEALTH_ENABLED, msc_logic_health() can change the state to
 *	CHG_HEALTH_ENABLED  <-> CHG_HEALTH_ACTIVE
 *	CHG_HEALTH_ENABLED  -> CHG_HEALTH_DISABLED
 *
 * from CHG_HEALTH_ACTIVE, msc_logic_health() can change the state to
 *	CHG_HEALTH_ACTIVE   <-> CHG_HEALTH_ENABLED
 *	CHG_HEALTH_ACTIVE   -> CHG_HEALTH_DISABLED
 *	CHG_HEALTH_ACTIVE   -> CHG_HEALTH_DONE
 */
enum chg_health_state {
	CHG_HEALTH_CCLVL_DISABLED = -6,
	CHG_HEALTH_BD_DISABLED = -5,
	CHG_HEALTH_USER_DISABLED = -3,
	CHG_HEALTH_DISABLED = -2,
	CHG_HEALTH_DONE = -1,
	CHG_HEALTH_INACTIVE = 0,
	CHG_HEALTH_ENABLED,
	CHG_HEALTH_ACTIVE,
};

/* tier index used to log the session */
enum gbms_stats_tier_idx_t {
	GBMS_STATS_AC_TI_DEFENDER = -5,
	GBMS_STATS_AC_TI_DISABLE_SETTING_STOP = -4,
	GBMS_STATS_AC_TI_DISABLE_MISC = -3,
	GBMS_STATS_AC_TI_DISABLE_SETTING = -2,
	GBMS_STATS_AC_TI_INVALID = -1,

	/* Regular charge tiers 0 -> 9 */
	GBMS_STATS_AC_TI_VALID = 10,
	GBMS_STATS_AC_TI_DISABLED,
	GBMS_STATS_AC_TI_ENABLED,
	GBMS_STATS_AC_TI_ACTIVE,
	GBMS_STATS_AC_TI_ENABLED_AON,
	GBMS_STATS_AC_TI_ACTIVE_AON,

	/* TODO: rename, these are not really related to AC */
	GBMS_STATS_AC_TI_FULL_CHARGE = 100,
	GBMS_STATS_AC_TI_HIGH_SOC = 101,

	/* Defender TEMP or DWELL */
	GBMS_STATS_BD_TI_OVERHEAT_TEMP = 110,
	GBMS_STATS_BD_TI_CUSTOM_LEVELS = 111,
};

/* health state */
struct batt_chg_health {
	int rest_soc;		/* entry criteria */
	int rest_voltage;	/* entry criteria */
	int always_on_soc;	/* entry criteria */

	time_t rest_deadline;	/* full by this in seconds */
	int rest_rate;		/* centirate once enter */

	enum chg_health_state rest_state;
	int rest_cc_max;
	int rest_fv_uv;
};

#define CHG_HEALTH_REST_IS_ACTIVE(rest) \
	((rest)->rest_state == CHG_HEALTH_ACTIVE)

#define CHG_HEALTH_REST_SOC(rest) (((rest)->always_on_soc != -1) ? \
			(rest)->always_on_soc : (rest)->rest_soc)

/* reset on every charge session */
struct gbms_charging_event {
	union gbms_ce_adapter_details	adapter_details;

	/* profile used for this charge event */
	const struct gbms_chg_profile *chg_profile;
	/* charge event and tier tracking */
	struct gbms_ce_stats		charging_stats;
	struct gbms_ce_tier_stats	tier_stats[GBMS_STATS_TIER_COUNT];

	/* soc tracking for time to full */
	struct ttf_soc_stats soc_stats;
	int last_soc;

	time_t first_update;
	time_t last_update;
	uint32_t chg_sts_qual_time;
	uint32_t chg_sts_delta_soc;

	/* health based charging */
	struct batt_chg_health		ce_health;	/* updated on close */
	struct gbms_ce_tier_stats	health_stats;	/* updated in HC */

	/* other stats */
	struct gbms_ce_tier_stats full_charge_stats;
	struct gbms_ce_tier_stats high_soc_stats;

	struct gbms_ce_tier_stats overheat_stats;
	struct gbms_ce_tier_stats cc_lvl_stats;
};

#define GBMS_CCCM_LIMITS(profile, ti, vi) \
	profile->cccm_limits[(ti * profile->volt_nb_limits) + vi]

/* newgen charging */
#define GBMS_CS_FLAG_BUCK_EN	BIT(0)
#define GBMS_CS_FLAG_DONE	BIT(1)
#define GBMS_CS_FLAG_CC		BIT(2)
#define GBMS_CS_FLAG_CV		BIT(3)
#define GBMS_CS_FLAG_ILIM	BIT(4)
#define GBMS_CS_FLAG_CCLVL	BIT(5)

union gbms_charger_state {
	uint64_t v;
	struct {
		uint8_t flags;
		uint8_t pad;
		uint8_t chg_status;
		uint8_t chg_type;
		uint16_t vchrg;
		uint16_t icl;
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
const char *gbms_chg_ev_adapter_s(int adapter);

/* Votables */
#define VOTABLE_MSC_CHG_DISABLE	"MSC_CHG_DISABLE"
#define VOTABLE_MSC_PWR_DISABLE	"MSC_PWR_DISABLE"
#define VOTABLE_MSC_INTERVAL	"MSC_INTERVAL"
#define VOTABLE_MSC_FCC		"MSC_FCC"
#define VOTABLE_MSC_FV		"MSC_FV"

/* Binned cycle count */
#define GBMS_CCBIN_BUCKET_COUNT	10

#ifdef CONFIG_QPNP_QG
#undef GBMS_CCBIN_BUCKET_COUNT
#define GBMS_CCBIN_BUCKET_COUNT	8
#endif

#define GBMS_CCBIN_CSTR_SIZE	(GBMS_CCBIN_BUCKET_COUNT * 6 + 2)

int gbms_cycle_count_sscan_bc(u16 *ccount, int bcnt, const char *buff);
int gbms_cycle_count_cstr_bc(char *buff, size_t size,
					const u16 *ccount, int bcnt);

#define gbms_cycle_count_sscan(cc, buff) \
	gbms_cycle_count_sscan_bc(cc, GBMS_CCBIN_BUCKET_COUNT, buff)

#define gbms_cycle_count_cstr(buff, size, cc)	\
	gbms_cycle_count_cstr_bc(buff, size, cc, GBMS_CCBIN_BUCKET_COUNT)

/* Time to full */
int ttf_soc_cstr(char *buff, int size, const struct ttf_soc_stats *soc_stats,
		 int start, int end);

int ttf_soc_estimate(time_t *res,
		     const struct batt_ttf_stats *stats,
		     const struct gbms_charging_event *ce_data,
		     qnum_t soc, qnum_t last);

void ttf_soc_init(struct ttf_soc_stats *dst);

int ttf_tier_cstr(char *buff, int size, const struct ttf_tier_stat *t_stat);

int ttf_tier_estimate(time_t *res,
		      const struct batt_ttf_stats *ttf_stats,
		      int temp_idx, int vbatt_idx,
		      int capacity, int full_capacity);

int ttf_stats_init(struct batt_ttf_stats *stats,
		   struct device *device,
		   int capacity_ma);

void ttf_stats_update(struct batt_ttf_stats *stats,
	 	      struct gbms_charging_event *ce_data,
		      bool force);

int ttf_stats_cstr(char *buff, int size, const struct batt_ttf_stats *stats,
		   bool verbose);

int ttf_stats_sscan(struct batt_ttf_stats *stats,
		    const char *buff, size_t size);

struct batt_ttf_stats *ttf_stats_dup(struct batt_ttf_stats *dst,
				     const struct batt_ttf_stats *src);

void ttf_log(const struct batt_ttf_stats *stats, const char *fmt, ...);

ssize_t ttf_dump_details(char *buf, int max_size,
			 const struct batt_ttf_stats *ttf_stats,
			 int last_soc);
/**
 * GBMS Storage API
 * The API provides functions to access to data stored in the persistent and
 * semi-persistent storage of a device in a cross-platform and
 * location-independent fashion. Clients in kernel and userspace use this
 * directly and indirectly to retrieve battery serial number, cell chemistry
 * type, cycle bin count, battery lifetime history and other battery related
 * data.
 */

#define GBMS_STORAGE_ADDR_INVALID	-1
#define GBMS_STORAGE_INDEX_INVALID	-1

/* Battery Google Part Number */
#define GBMS_BGPN_LEN	10
/* Battery manufacturer info length */
#define GBMS_MINF_LEN	32
/* Battery device info length */
#define GBMS_DINF_LEN	32
/* Battery cycle count bin length */
#define GBMS_CNTB_LEN	16

/**
 * Tags are u32 constants: hardcoding as hex since characters constants of more
 * than one byte such as 'BGCE' are frown upon.
 */
typedef uint32_t gbms_tag_t;

enum gbms_tags {
	GBMS_TAG_BGCE = 0x42474345,
	GBMS_TAG_BCNT = 0x42434e54,
	GBMS_TAG_BRES = 0x42524553,
	GBMS_TAG_SNUM = 0x534e554d,
	GBMS_TAG_HIST = 0x48495354,
	GBMS_TAG_BRID = 0x42524944,
	GBMS_TAG_DSNM = 0x44534e4d,
	GBMS_TAG_MINF = 0x4d494e46,
	GBMS_TAG_DINF = 0x44494e46,
	GBMS_TAG_BGPN = 0x4247504e,
	GBMS_TAG_CNTB = 0x434e5442,
};

/**
 * struct gbms_storage_desc - callbacks for a GBMS storage provider.
 *
 * Fields not used should be initialized with NULL. The provider name and the
 * iter callback are optional but strongly recommended. The write, fetch, store
 * and flush callbacks are optional, descriptors with a non NULL write/store
 * callback should have a non NULL read/fetch callback.
 *
 * The iterator callback (iter) is used to list the tags stored in the provider
 * and can be used to detect duplicates. The list of tags exported from iter
 * can be expected to be static (i.e. tags can be enumerated once on
 * registration).
 *
 * The read and write callbacks transfer the data associated with a tag. The
 * calls must return -ENOENT when called with a tag that is not known to the
 * provider, a negative number on every other error or the number of bytes
 * read or written to the device. The tag lookup for the read and write
 * callbacks must be very efficient (i.e. consider implementation that use hash
 * or switch statements).
 *
 * Fetch and store callbacks are used to grant non-mediated access to a range
 * of consecutive addresses in storage space. The implementation must return a
 * negative number on error or the number of bytes transferred with the
 * operation. Support caching of the tag data location requires non NULL fetch
 * and not NULL info callbacks.
 *
 * The read_data and write_data callbacks transfer the data associated with an
 * enumerator. The calls must return -ENOENT when called with a tag that is not
 * known to the provider, a negative number on every other error or the number
 * of bytes read or written to the device during data transfers.
 *
 * Clients can only access keys that are available on a device (i.e. clients
 * cannot create new tags) and the API returns -ENOENT when trying to access a
 * tag that is not available on a device, -EGAIN while the storage is not fully
 * initialized.
 *
 * @iter: callback, return the tags known from this provider
 * @info: callback, return address and size for tags (used for caching)
 * @read: callback, read data from a tag
 * @write: callback, write data to a tag
 * @fetch: callback, read up to count data bytes from an address
 * @store: callback, write up to count data bytes to an address
 * @flush: callback, request a fush of data to permanent storage
 * @read_data: callback, read the elements of an enumerations
 * @write_data: callback, write to the elements of an enumeration
 */
struct gbms_storage_desc {
	int (*iter)(int index, gbms_tag_t *tag, void *ptr);
	int (*info)(gbms_tag_t tag, size_t *addr, size_t *size, void *ptr);
	int (*read)(gbms_tag_t tag, void *data, size_t count, void *ptr);
	int (*write)(gbms_tag_t tag, const void *data, size_t count, void *ptr);
	int (*fetch)(void *data, size_t addr, size_t count, void *ptr);
	int (*store)(const void *data, size_t addr, size_t count, void *ptr);
	int (*flush)(bool force, void *ptr);

	int (*read_data)(gbms_tag_t tag, void *data, size_t count, int idx,
			 void *ptr);
	int (*write_data)(gbms_tag_t tag, const void *data, size_t count,
			  int idx, void *ptr);
};

int gbms_storage_register(struct gbms_storage_desc *desc, const char *name,
			  void *ptr);
int gbms_storage_read(gbms_tag_t tag, void *data, size_t count);
int gbms_storage_write(gbms_tag_t tag, const void *data, size_t count);

int gbms_storage_read_data(gbms_tag_t tag, void *data, size_t count, int idx);
int gbms_storage_write_data(gbms_tag_t tag, const void *data, size_t count,
			    int idx);
int gbms_storage_flush(gbms_tag_t tag);
int gbms_storage_flush_all(void);

struct gbms_storage_device;

/* standard device implementation that read data from an enumeration */
struct gbms_storage_device *gbms_storage_create_device(const char *name,
						       gbms_tag_t tag);
void gbms_storage_cleanup_device(struct gbms_storage_device *gdev);


#endif  /* __GOOGLE_BMS_H_ */
