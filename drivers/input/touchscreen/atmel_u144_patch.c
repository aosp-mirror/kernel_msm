/*
 * Atmel maXTouch Touchscreen patch driver
 *
 * Copyright (C) 2013 Atmel Corporation
 * Copyright (C) 2013-2016 LG Electronics, Inc.
 *
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/slab.h>

#include "atmel_u144.h"
#include "atmel_u144_patch.h"

static u8 t255_user[MXT_PATCH_USER_DATA_MAX];

struct touch_pos tpos_data;
struct touch_supp tsupp_data;

static void mxt_patch_init_userdata(void)
{
	memset(t255_user, 0, MXT_PATCH_USER_DATA_MAX);
}

static void mxt_patch_calibration(struct mxt_data *data)
{
	mxt_write_object(data, MXT_GEN_COMMAND_T6, MXT_COMMAND_CALIBRATE, 1);
}

static int mxt_patch_start_timer(struct mxt_data *data, u16 period)
{
	struct mxt_object* object = NULL;
	int ret = 0;
	u8 t61_reg[5] = {3, 1, 0, 0, 0};

	object = mxt_get_object(data, MXT_SPT_TIMER_T61);
	if (!object) {
		__mxt_patch_debug(data, "TIMER NOT SUPPORTED\n");
		return 0;
	}

	t61_reg[3] = period & 0xFF;
	t61_reg[4] = (period >> 8) & 0xFF;

	ret = mxt_write_mem(data, object->start_address + (5 * data->patch.timer_id), 5, t61_reg);
	if (!ret) {
		__mxt_patch_debug(data, "START STAGE: %d TIMER[%d] %dms\n",
			data->patch.cur_stage, data->patch.timer_id, period);
	}

	return ret;
}

static int mxt_patch_stop_timer(struct mxt_data *data)
{
	struct mxt_object* object = NULL;
	int ret = 0;
	u8 t61_reg[5] = {3, 2, 0, 0, 0};

	object = mxt_get_object(data, MXT_SPT_TIMER_T61);
	if (!object) {
		__mxt_patch_debug(data, "TIMER NOT SUPPORTED\n");
		return 0;
	}

	ret = mxt_write_mem(data, object->start_address + (5 * data->patch.timer_id), 5, t61_reg);
	if (!ret) {
		__mxt_patch_debug(data, "STOP TIMER[%d]\n", data->patch.timer_id);
	}

	return ret;
}

static int mxt_patch_write_stage_cfg(struct mxt_data *data,
		struct stage_cfg* pscfg, bool do_action)
{
	if (!do_action)
		return 0;

	__mxt_patch_debug(data, "|- SCFG_WRITE: OBJECT_TYPE:%d OFFSET:%d VAL:%d OPT:%d\n",
		pscfg->obj_type, pscfg->offset, pscfg->val, pscfg->option);

	if (pscfg->obj_type == 255)
		t255_user[pscfg->offset] = pscfg->val;
	else
		mxt_write_object(data, pscfg->obj_type, pscfg->offset, pscfg->val);

	return 0;
}

static int mxt_patch_write_action_cfg(struct mxt_data *data,
		struct action_cfg* pacfg, bool do_action)
{
	if (!do_action)
		return 0;

	__mxt_patch_debug(data, "|-- ACFG_WRITE: OBJECT_TYPE:%d OFFSET:%d VAL:%d OPT:%d\n",
		pacfg->obj_type, pacfg->offset, pacfg->val, pacfg->option);

	if (pacfg->obj_type == 255)
		t255_user[pacfg->offset] = pacfg->val;
	else
		mxt_write_object(data, pacfg->obj_type, pacfg->offset, pacfg->val);

	return 0;
}

static int mxt_patch_write_trigger_cfg(struct mxt_data *data,
		struct trigger_cfg* ptcfg, bool do_action)
{
	if (!do_action)
		return 0;

	__mxt_patch_debug(data, "|-- TCFG_WRITE: OBJECT_TYPE:%d OFFSET:%d VAL:%d\n",
		ptcfg->obj_type, ptcfg->offset, ptcfg->val);

	if (ptcfg->obj_type == 255)
		t255_user[ptcfg->offset] = ptcfg->val;
	else
		mxt_write_object(data, ptcfg->obj_type, ptcfg->offset, ptcfg->val);

	return 0;
}

static int mxt_patch_write_event_cfg(struct mxt_data *data,
		struct event_cfg* pecfg, bool do_action)
{
	if (!do_action)
		return 0;

	__mxt_patch_debug(data, "|-- ECFG_WRITE: OBJECT_TYPE:%d OFFSET:%d VAL:%d\n",
		pecfg->obj_type, pecfg->offset, pecfg->val);

	if (pecfg->obj_type == 99) {
		data->patch.start = false;
		data->patch.skip_test = 1;
		data->patch.run_stage = false;
		data->patch.start_stage = pecfg->val;
		__mxt_patch_debug(data, "Start stage change:%d\n", pecfg->val);
	} else if(pecfg->obj_type == 255) {
		t255_user[pecfg->offset] = pecfg->val;
	} else {
		return mxt_write_object(data, pecfg->obj_type, pecfg->offset, pecfg->val);
	}

	return 0;
}

static int mxt_patch_predefined_action(struct mxt_data *data, u8 action_id,
		u16 action_val, bool do_action)
{
	if (!do_action)
		return 0;

	switch (action_id) {
		case MXT_PATCH_ACTION_NONE:
			__mxt_patch_debug(data, "|-- ACTION NONE\n");
			break;
		case MXT_PATCH_ACTION_CAL:
			__mxt_patch_debug(data, "|-- ACTION CALIBRATE: %d\n", action_val);
			mxt_patch_calibration(data);
			data->patch.start = false; // Wait Restart
			data->patch.start_stage = action_val;
			break;
		case MXT_PATCH_ACTION_EXTEND_TIMER:
			__mxt_patch_debug(data, "|-- ACTION EXTEND TIMER: %d\n", action_val);
			mxt_patch_start_timer(data, action_val);
			break;
		case MXT_PATCH_ACTION_GOTO_STAGE:
			__mxt_patch_debug(data, "|-- ACTION GOTO STAGE: %d\n", action_val);
			data->patch.skip_test = 1;
			data->patch.cur_stage = action_val;
			data->patch.run_stage = false;
			break;
		case MXT_PATCH_ACTION_CHANGE_START:
			__mxt_patch_debug(data, "|-- ACTION CHANGE START STAGE: %d\n", action_val);
			data->patch.start_stage = action_val;
			break;
		default:
			__mxt_patch_debug(data, "@@ INVALID ACTION ID=%d !!\n", action_id);
			return -1;
	}

	return 0;
}

static void mxt_patch_init_tpos(struct mxt_data *data, struct touch_pos* tpos)
{
	int i = 0;

	for (i = 0; i < MXT_MAX_FINGER; i++) {
		tpos->tcount[i] = 0;
		tpos->initx[i] = 0;
		tpos->inity[i] = 0;
		tpos->oldx[i] = 0;
		tpos->oldy[i] = 0;
	}
	tpos->locked_id = 0xff;
	tpos->moved_cnt = 0;
}

static bool mxt_patch_check_locked(struct mxt_data *data,
		struct touch_pos* tpos, u8 tid, u16 x, u16 y)
{
	s16 diffx = 0, diffy = 0;
	u32 distance = 0;

	/* OLD DIFF */
	diffx = x - tpos->oldx[tid];
	diffy = y - tpos->oldy[tid];
	distance = abs(diffx) + abs(diffy);

	/* INIT DIFF */
	if ((tpos->initx[tid] != 0) && (tpos->inity[tid] != 0)) {
		diffx = x - tpos->initx[tid];
		diffy = y - tpos->inity[tid];
		__mxt_patch_ddebug(data, "[TPOS] INITDIFF[%d] ABS X=%d, ABS Y=%d\n", tid, (int)abs(diffx), (int)abs(diffy));
	}

	if ((tpos->initx[tid] == 0) && (tpos->inity[tid] == 0)) {
		__mxt_patch_ddebug(data, "[TPOS] INITSET[%d] X=%d, Y=%d\n", tid, x, y);
		tpos->initx[tid] = x;
		tpos->inity[tid] = y;
		tpos->moved_cnt = 0;
	} else {
		/* OLD DIFF vs INIT DIFF */
		if ((distance < tpos->jitter) && ((abs(diffx) > tpos->maxdiff) || (abs(diffy) > tpos->maxdiff))) {
			tpos->moved_cnt++;
		}
	}

	if (tpos->moved_cnt > tpos->reset_cnt) {
		__mxt_patch_ddebug(data, "[TPOS] RESET[%d] X=%d, Y=%d\n", tid, x, y);
		tpos->initx[tid] = x;
		tpos->inity[tid] = y;
		tpos->moved_cnt = 0;
	}

	if ((distance < tpos->distance) &&
			(abs(diffx) < tpos->maxdiff) &&
			(abs(diffy) < tpos->maxdiff)) {
		return true;
	} else {
		return false;
	}

	return false;
}

static void mxt_patch_check_pattern(struct mxt_data *data,
		struct touch_pos* tpos, u8 tid, u16 x, u16 y, u8 finger_cnt)
{
	bool cal_condition = false;
	int error = 0;

	if (!finger_cnt) {
		return;
	}

	if (mxt_patch_check_locked(data, tpos, tid, x, y)) {
		tpos->tcount[tid] = tpos->tcount[tid]+1;
	} else {
		tpos->tcount[tid] = 0;
	}

	tpos->oldx[tid] = x;
	tpos->oldy[tid] = y;

	if (finger_cnt == 1) {
		if (tpos->tcount[tid] > tpos->locked_cnt) {
			__mxt_patch_debug(data, "[TPOS] ONE TOUCH LOCKED\n");
			mxt_patch_init_tpos(data, tpos);
			cal_condition = true;
		}
	} else {
		if ((tpos->tcount[tid] > tpos->locked_cnt) && tpos->locked_id != tid && tpos->locked_id != 0xff) {
			__mxt_patch_debug(data, "[TPOS] TWO TOUCH LOCKED [%d, %d]\n", tid, tpos->locked_id);
			mxt_patch_init_tpos(data, tpos);
			cal_condition = true;
		}

		if (tpos->tcount[tid] > tpos->locked_cnt) {
			tpos->locked_id = tid;
			if(tpos->tcount[tid] >= 0xFF){
				__mxt_patch_debug(data, "[TPOS] OVER LOCKED\n");
				mxt_patch_init_tpos(data, tpos);
				cal_condition = true;
			}
		}
	}

	if (cal_condition) {
		error = mxt_read_object(data, MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71,
			MXT_PATCH_T71_PTN_CAL, &tpos->cal_enable);
		if (error) {
			TOUCH_PATCH_INFO_MSG( "%s: Error read T71 [%d]\n", __func__, error);
		} else {
			if (tpos->cal_enable) {
				__mxt_patch_debug(data, "[TPOS] CAL\n");
				mxt_patch_calibration(data);

				error = mxt_read_object(data, MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71,
					MXT_PATCH_T71_PTN_OPT, &tpos->option);

				if (!error) {
					if (tpos->option & 0x01) { // Onetime
						mxt_write_object(data, MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71,
							MXT_PATCH_T71_PTN_CAL, 0);
						__mxt_patch_debug(data, "[TPOS] DISABLE T71[2]\n");
					}
				}
			} else{
				__mxt_patch_debug(data, "[TPOS] SKIP CAL T71[2]=0\n");
			}
		}
	}
}

static void mxt_patch_init_supp(struct mxt_data *data, struct touch_supp* tsup)
{
	tsup->old_time = jiffies_to_msecs(jiffies);
	tsup->repeat_cnt = 0;
}

static void mxt_patch_check_supp(struct mxt_data *data, struct touch_supp* tsup)
{
	u32 curr_time = jiffies_to_msecs(jiffies);
	u32 time_diff = 0;

	time_diff = TIME_WRAP_AROUND(tsup->old_time, curr_time);

	if (time_diff < tsup->time_gap*100) {
		__mxt_patch_debug(data, "[TSUP] Abnormal suppress %d\n", tsup->repeat_cnt);

		if (tsup->repeat_cnt++ > tsup->repeat_max) {
			__mxt_patch_debug(data, "[TSUP] Abnormal suppress detected\n");
			mxt_patch_calibration(data);
		}
	} else {
		tsup->repeat_cnt = 0;
		__mxt_patch_debug(data, "[TSUP] Normal suppress\n");
	}

	tsup->old_time = curr_time;
	mxt_patch_dump_source(data, true);
}

static void mxt_patch_load_t71data(struct mxt_data *data)
{
	struct mxt_object *obj = NULL;
	u8 buf[MXT_PATCH_T71_DATA_MAX] = {0};
	struct touch_pos* tpos = &tpos_data;
	int error = 0;

	obj = mxt_get_object(data, MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71);
	if (obj) {
		error = mxt_read_mem(data,
				obj->start_address,MXT_PATCH_T71_DATA_MAX, buf);

		if (!error) {
			tpos->option = buf[MXT_PATCH_T71_PTN_OPT];
			tpos->cal_enable = buf[MXT_PATCH_T71_PTN_CAL];
			tpos->reset_cnt = buf[3];
			tpos->distance = buf[4];
			tpos->maxdiff = buf[5];
			tpos->locked_cnt = buf[6];
			tpos->jitter = buf[7];
			tpos->amp = buf[8];
			tpos->area = buf[9];
			tpos->sum_size_t57 = buf[10];
			tpos->tch_count_t57 = buf[11];
			tpos->atch_count_t57 = buf[12];
			tpos->amp_2finger_min = buf[13];
			tpos->area_2finger_min = buf[14];
			tpos->sum_size_t57_2finger_min = buf[15];
			tpos->tch_count_t57_2finger_min = buf[16];
			tpos->atch_count_t57_2finger_min = buf[17];
			tpos->amp_2finger_max = buf[18];
			tpos->area_2finger_max = buf[19];
			tpos->sum_size_t57_2finger_max = buf[20];
			tpos->tch_count_t57_2finger_max = buf[21];
			tpos->atch_count_t57_2finger_max = buf[22];

			tpos->amp_3finger_min = buf[23];
			tpos->area_3finger_min = buf[24];
			tpos->sum_size_t57_3finger_min = buf[25];
			tpos->tch_count_t57_3finger_min = buf[26];
			tpos->atch_count_t57_3finger_min = buf[27];
			tpos->amp_3finger_max = buf[28];
			tpos->area_3finger_max = buf[29];
			tpos->sum_size_t57_3finger_max = buf[30];
			tpos->tch_count_t57_3finger_max = buf[31];
			tpos->atch_count_t57_3finger_max = buf[32];

			tpos->amp_mfinger_min = buf[33];
			tpos->area_mfinger_min = buf[34];
			tpos->sum_size_t57_mfinger_min = buf[35];
			tpos->tch_count_t57_mfinger_min = buf[36];
			tpos->atch_count_t57_mfinger_min = buf[37];
			tpos->amp_mfinger_max = buf[38];
			tpos->area_mfinger_max = buf[39];
			tpos->sum_size_t57_mfinger_max = buf[40];
			tpos->tch_count_t57_mfinger_max = buf[41];
			tpos->atch_count_t57_mfinger_max = buf[42];

			tpos->xlo_limit = buf[43];
			tpos->xhi_limit = (buf[44]<<8) | buf[45];
			tpos->ylo_limit = buf[46];
			tpos->yhi_limit = (buf[47]<<8) | buf[48];
			__mxt_patch_debug(data, "PTN CAL %d RST %d DST %d DIF %d CNT %d JIT %d\n",
				tpos->cal_enable, tpos->reset_cnt, tpos->distance, tpos->maxdiff, tpos->locked_cnt, tpos->jitter);

			TOUCH_PATCH_INFO_MSG("PTN CAL %d RST %d DST %d DIF %d CNT %d JIT %d\n",
				tpos->cal_enable, tpos->reset_cnt, tpos->distance, tpos->maxdiff, tpos->locked_cnt, tpos->jitter);

			tsupp_data.time_gap = buf[49];
			tsupp_data.repeat_max = buf[50];

			__mxt_patch_debug(data, "SUPP GAP %d*100ms CNT %d\n",
				tsupp_data.time_gap, tsupp_data.repeat_max);
		}
	}
}

const char* mxt_patch_src_item_name(u8 src_id)
{
	const char* src_item_name[MXT_PATCH_MAX_TYPE] = {
		MXT_XML_SRC_NONE,	//MXT_PATCH_ITEM_NONE		0
		MXT_XML_SRC_CHRG,	//MXT_PATCH_ITEM_CHARGER	1
		MXT_XML_SRC_FCNT,	//MXT_PATCH_ITEM_FINGER_CNT	2
		MXT_XML_SRC_AREA,	//MXT_PATCH_ITEM_T9_AREA	3
		MXT_XML_SRC_AMP,	//MXT_PATCH_ITEM_T9_AMP		4
		MXT_XML_SRC_SUM,	//MXT_PATCH_ITEM_T57_SUM	5
		MXT_XML_SRC_TCH,	//MXT_PATCH_ITEM_T57_TCH	6
		MXT_XML_SRC_ATCH,	//MXT_PATCH_ITEM_T57_ATCH	7
		MXT_XML_SRC_KCNT,	//MXT_PATCH_ITEM_KCNT		8
		MXT_XML_SRC_KVAL,	//MXT_PATCH_ITEM_KVAL		9
		MXT_XML_SRC_T9STATUS,	//MXT_PATCH_ITEM_T9STATUS	10
		MXT_XML_SRC_USER1,
		MXT_XML_SRC_USER2,
		MXT_XML_SRC_USER3,
		MXT_XML_SRC_USER4,
		MXT_XML_SRC_USER5,
	};

	if (MXT_PATCH_ITEM_NONE <= src_id && src_id < MXT_PATCH_ITEM_END) {
		return src_item_name[src_id];
	}

	return "ERR";
}

const char* mxt_patch_cond_name(u8 con_id)
{
	const char* cond_name[MXT_PATCH_MAX_CON] = {
		MXT_XML_CON_NONE,	//MXT_PATCH_CON_NONE	0
		MXT_XML_CON_EQUAL,	//MXT_PATCH_CON_EQUAL	1
		MXT_XML_CON_BELOW,	//MXT_PATCH_CON_BELOW	2
		MXT_XML_CON_ABOVE,	//MXT_PATCH_CON_ABOVE	3
		MXT_XML_CON_PLUS,	//MXT_PATCH_CON_PLUS	4
		MXT_XML_CON_MINUS,	//MXT_PATCH_CON_MINUS	5
		MXT_XML_CON_MUL,	//MXT_PATCH_CON_MUL	6
		MXT_XML_CON_DIV,	//MXT_PATCH_CON_DIV	7
		MXT_XML_CON_MASK,	//MXT_PATCH_CON_MASK	8
	};

	if (MXT_PATCH_CON_NONE <= con_id && con_id < MXT_PATCH_CON_END) {
		return cond_name[con_id];
	}

	return "ERR";
}

static int mxt_patch_item_lval(struct mxt_data *data, u16* psrc_item, u8 src_id)
{
	if (!psrc_item)
		return 0;

	if (MXT_PATCH_ITEM_NONE <= src_id && src_id < MXT_PATCH_ITEM_END) {
		return psrc_item[src_id];
	} else {
		__mxt_patch_debug(data, "@@ INVALID ITEM ID=%d !!\n", src_id);
	}

	return 0;
}

static int mxt_patch_item_rval(struct mxt_data *data, u16* psrc_item, struct item_val ival)
{
	int lval = mxt_patch_item_lval(data, psrc_item, ival.val_id);
	int rval = ival.val;

	switch (ival.val_eq) {
		case MXT_PATCH_CON_NONE:
			return lval ? lval : rval;
		case MXT_PATCH_CON_PLUS:
			lval += rval;
			break;
		case MXT_PATCH_CON_MINUS:
			lval -= rval;
			break;
		case MXT_PATCH_CON_MUL:
			lval *= rval;
			break;
		case MXT_PATCH_CON_DIV:
			lval /= rval;
			break;
		default:
			if (psrc_item) {
				__mxt_patch_debug(data,
					"@@ INVALID VAL_EQ=%d (LVAL=%d) => RVAL=%d !!\n",
					ival.val_eq, lval, rval);
			}
			return rval;
	}
	return lval;
}

static int mxt_patch_item_check(struct mxt_data *data, u16* psrc_item,
		struct test_item* ptitem, bool do_action)
{
	int lval = mxt_patch_item_lval(data, psrc_item, ptitem->src_id);
	int rval = mxt_patch_item_rval(data, psrc_item, ptitem->ival);

	if (!do_action) {
		__mxt_patch_debug(data, "|-- ITEM SRC_ID:%s COND:%s VAL_ID:%s EQ:%s VAL:%d\n",
			mxt_patch_src_item_name(ptitem->src_id), mxt_patch_cond_name(ptitem->cond),
			mxt_patch_src_item_name(ptitem->ival.val_id), mxt_patch_cond_name(ptitem->ival.val_eq), ptitem->ival.val);
	}

	if (psrc_item) {
		switch (ptitem->cond) {
			case MXT_PATCH_CON_EQUAL:
				__mxt_patch_ddebug(data, "|--- IF %s: %d == %d = %d\n",
					mxt_patch_src_item_name(ptitem->src_id), lval, rval, lval == rval ? 1 : 0);
				return lval == rval ? 1 : 0;
			case MXT_PATCH_CON_BELOW:
				__mxt_patch_ddebug(data, "|--- IF %s: %d < %d = %d\n",
					mxt_patch_src_item_name(ptitem->src_id), lval, rval, lval < rval ? 1 : 0);
				return lval < rval ? 1 : 0;
			case MXT_PATCH_CON_ABOVE:
				__mxt_patch_ddebug(data, "|--- IF %s: %d > %d = %d\n",
					mxt_patch_src_item_name(ptitem->src_id), lval, rval, lval > rval ? 1 : 0);
				return lval > rval ? 1 : 0;
			case MXT_PATCH_CON_MASK:
				__mxt_patch_ddebug(data, "|--- IF %s: %d & %d = %d\n",
					mxt_patch_src_item_name(ptitem->src_id), lval, rval, lval & rval ? 1 : 0);
				return lval & rval ? 1 : 0;
			default:
				__mxt_patch_debug(data, "@@ INVALID TEST COND=%d !!\n", ptitem->cond);
				return -1;
		}
	}
	return -1;
}

static int mxt_patch_stage_timer(struct mxt_data *data, u16 period, bool do_action)
{
	int ret = 0;
	u32 time = period * 10;

	if (do_action) {
		ret = mxt_patch_start_timer(data, time);
		if (!ret) {
			data->patch.period = period;
		}
	}
	return 0;
}

void mxt_patch_dump_source(struct mxt_data *data, bool do_action)
{
	if (do_action) {
		__mxt_patch_debug(data, "TA:%d FCNT:%d AREA:%d AMP:%d"
			" SUM:%d TCH:%d ATCH:%d KCNT:%d KVAL:%d S:%d U1:%d U2:%d U3:%d U4:%d U5:%d U6:%d Charger : %d\n",
			data->patch.src_item[1], data->patch.src_item[2],
			data->patch.src_item[3], data->patch.src_item[4],
			data->patch.src_item[5], data->patch.src_item[6],
			data->patch.src_item[7], data->patch.src_item[8],
			data->patch.src_item[9], data->patch.src_item[10],
			data->patch.src_item[11], data->patch.src_item[12],
			data->patch.src_item[13], data->patch.src_item[14],
			data->patch.src_item[15], data->patch.src_item[16], data->charging_mode);
	}
}

static int mxt_patch_parse_test_line(struct mxt_data *data, u8* ppatch,
		u16* psrc_item, u16* check_cnt, bool do_action)
{
	struct test_line* ptline = NULL;
	struct test_item* ptitem = NULL;
	struct action_cfg* pacfg = NULL;
	u32 i = 0, ulpos = 0;
	u8 test_result = 0;
	bool test_action = false;

	ptline = (struct test_line*)ppatch;

	if (!do_action) {
		__mxt_patch_debug(data, "|- TEST_LINE:%X OPT:%d CHK_CNT:%d ITEM_CNT:%d CFG_CNT:%d ACTION:%d VAL:%d \n",
			ptline->test_id, ptline->option, ptline->check_cnt, ptline->item_cnt, ptline->cfg_cnt, ptline->act_id, ptline->act_val);
	}

	ulpos += sizeof(struct test_line);

	test_result = 0;
	test_action = false;

	for (i = 0; i < ptline->item_cnt; i++) { /* Test Item Parsing */
		ptitem = (struct test_item*)(ppatch+ulpos);

		if (mxt_patch_item_check(data, psrc_item,ptitem, do_action) > 0) {
			test_result++;

			if (test_result == ptline->item_cnt) {
				if (check_cnt != NULL) {
					*check_cnt = *check_cnt + 1;

					if (*check_cnt == ptline->check_cnt) {
						test_action = true;
						TOUCH_PATCH_INFO_MSG("STAGE:%d TEST %d MATCHED\n", data->patch.cur_stage, ptline->test_id);
						TOUCH_PATCH_INFO_MSG("TA:%d FCNT:%d AREA:%d AMP:%d"
									" SUM:%d TCH:%d ATCH:%d KCNT:%d KVAL:%d S:%d U1:%d U2:%d U3:%d U4:%d U5:%d U6:%d Charger : %d\n",
									data->patch.src_item[1], data->patch.src_item[2],
									data->patch.src_item[3], data->patch.src_item[4],
									data->patch.src_item[5], data->patch.src_item[6],
									data->patch.src_item[7], data->patch.src_item[8],
									data->patch.src_item[9], data->patch.src_item[10],
									data->patch.src_item[11], data->patch.src_item[12],
									data->patch.src_item[13], data->patch.src_item[14],
									data->patch.src_item[15], data->patch.src_item[16], data->charging_mode);
						mxt_patch_dump_source(data, test_action);

						if (ptline->option & 0x01) {
							*check_cnt = 0;
							__mxt_patch_ddebug(data, "CHEK CNT CLEAR\n");
						}
					}
				}
			}
		} else {
			if (data->patch.option & 0x04) {
				if (do_action && psrc_item) {// Skip if any item was failed
					__mxt_patch_ddebug(data, "SKIP REMAINED ITEMS %d\n", i);
					return 0;
				}
			}
		}
		ulpos += sizeof(struct test_item);
	}

	for(i = 0; i <ptline->cfg_cnt; i++) { /* Test Line Action config */
		pacfg = (struct action_cfg*)(ppatch+ulpos);
		if (!do_action) {
			__mxt_patch_debug(data, "|-- ACTION_CFG: OBJ:%d OFFSET:%d VAL:%d OPT:%d\n",
				pacfg->obj_type, pacfg->offset, pacfg->val, pacfg->option);
		}
		mxt_patch_write_action_cfg(data, pacfg, test_action);
		ulpos += sizeof(struct action_cfg);
	}
	mxt_patch_predefined_action(data, ptline->act_id, ptline->act_val, test_action);

	return ulpos;
}

static int mxt_patch_parse_stage(struct mxt_data *data, u8* ppatch,
		u16* ptline_addr, u8* ptline_cnt, bool do_action)
{
	struct stage_def* psdef = NULL;
	struct stage_cfg* pscfg = NULL;
	u32 i = 0, ulpos = 0;

	psdef = (struct stage_def*)ppatch;

	if (!do_action) {
		__mxt_patch_debug(data,
			"STAGE_ID:%d OPT:%d PERIOD:%d CFG_CNT:%d TST_CNT:%d RESET:%d\n",
			psdef->stage_id, psdef->option, psdef->stage_period,
			psdef->cfg_cnt, psdef->test_cnt, psdef->reset_period);
	}

	mxt_patch_stage_timer(data, psdef->stage_period, do_action);
	ulpos += sizeof(struct stage_def);

	for(i = 0; i < psdef->cfg_cnt; i++) { /* Stage Config Parsing */
		pscfg = (struct stage_cfg*)(ppatch+ulpos);

		if (!do_action) {
			__mxt_patch_debug(data, "|- STAGE_CFG: OBJ:%d OFFSET:%d VAL:%d OPT:%d\n",
				pscfg->obj_type, pscfg->offset, pscfg->val, pscfg->option);
		}
		mxt_patch_write_stage_cfg(data, pscfg, do_action);
		ulpos += sizeof(struct stage_cfg);
	}

	for(i = 0; i < psdef->test_cnt; i++) { /* Test Line Parsing */
		if (ptline_addr != NULL) {
			ptline_addr[i] = (u16)ulpos;
		}
		ulpos += mxt_patch_parse_test_line(data, ppatch+ulpos, NULL, NULL, do_action);
	}

	if (ptline_cnt != NULL)
		*ptline_cnt = psdef->test_cnt;

	return ulpos;
}

static u16 mxt_patch_match_lval(struct mxt_data *data, u8* pmsg, u8 offset, u16 mask)
{
	u16 lval = 0;
	u8 msg[MXT_PATCH_MAX_MSG_SIZE+1] = {0};

	if (pmsg) {
		if (offset >= 200 && offset <= 255) {
			return t255_user[offset-200];
		}
		memcpy(msg, pmsg, MXT_PATCH_MAX_MSG_SIZE);
		if (0 <= offset && offset < MXT_PATCH_MAX_MSG_SIZE) {
			lval = msg[offset] | (msg[offset+1] << 8);
			return mask ? lval & mask : lval;
		} else {
			__mxt_patch_debug(data, "@@ INVALID OFFSET=%d !!\n", offset);
		}
	}

	return 0;
}

static int mxt_patch_match_check(struct mxt_data *data, u8* pmsg,
		struct match* pmatch, bool do_action)
{
	u16 lval = mxt_patch_match_lval(data, pmsg, pmatch->offset, pmatch->mask);
	u16 rval = pmatch->val;

	if (pmsg) {
		switch(pmatch->cond) {
			case MXT_PATCH_CON_EQUAL:
				__mxt_patch_ddebug(data, "|--- IF %d == %d = %d\n", lval, rval, lval == rval ? 1 : 0);
				return lval == rval ? 1 : 0;
			case MXT_PATCH_CON_BELOW:
				__mxt_patch_ddebug(data, "|--- IF %d < %d = %d\n", lval, rval, lval < rval ? 1 : 0);
				return lval < rval ? 1 : 0;
			case MXT_PATCH_CON_ABOVE:
				__mxt_patch_ddebug(data, "|--- IF %d > %d = %d\n", lval, rval, lval > rval ? 1 : 0);
				return lval > rval ? 1 : 0;
			default:
				__mxt_patch_debug(data, "@@ INVALID MATCH COND=%d !!\n", pmatch->cond);
				return -1;
		}
	}

	return -1;
}

static int mxt_patch_trigger_check(struct mxt_data *data, u8 object, u8 index, u8* pmsg)
{
	u8 reportid = pmsg[0];
	u8 type = 0, id = 0;

	type = data->reportids[reportid].type;
	id = data->reportids[reportid].index;

	if ((type == object) && (id == index))
		return 0;

	return 1;
}

static int mxt_patch_parse_trigger(struct mxt_data *data, u8* ppatch, u8* pmsg, bool do_action, u8 option)
{
	struct trigger* ptrgg = NULL;
	struct match* pmatch = NULL;
	struct trigger_cfg* ptcfg = NULL;
	u32 i = 0, ulpos = 0;
	u8 match_result = 0;
	u8 trigger_action = 0;

	ptrgg = (struct trigger*)ppatch;

	if (!do_action) {
		__mxt_patch_debug(data, "TRIGGER ID:%d OPT:%d OBJ:%d IDX:%d MATCH:%d CFG:%d ACT:%d VAL:%d\n",
			ptrgg->tid, ptrgg->option, ptrgg->object, ptrgg->index, ptrgg->match_cnt, ptrgg->cfg_cnt, ptrgg->act_id, ptrgg->act_val);
	}

	ulpos += sizeof(struct trigger);

	// Message Filter
	if (do_action) {
		if (mxt_patch_trigger_check(data, ptrgg->object, ptrgg->index, pmsg))
			return 1;
	}

	// Match Parsing
	match_result=0;
	trigger_action=false;
	for(i = 0; i < ptrgg->match_cnt; i++) {
		pmatch = (struct match*)(ppatch+ulpos);
		if (!do_action) {
			__mxt_patch_debug(data, "|- MATCH:%d OFFSET:%d MASK:%d COND:%s VAL:%d\n",
				i, pmatch->offset, pmatch->mask, mxt_patch_cond_name(pmatch->cond), pmatch->val);
		}

		if (mxt_patch_match_check(data, pmsg, pmatch, do_action) > 0) {
			match_result++;
			if (match_result == ptrgg->match_cnt) {
				if (option == ptrgg->option)
					trigger_action = true;
			}
		}
		ulpos += sizeof(struct match);
	}

	// Trigger Config Parsing
	for(i = 0; i < ptrgg->cfg_cnt; i++) {
		ptcfg = (struct trigger_cfg*)(ppatch+ulpos);

		if (!do_action) {
			__mxt_patch_debug(data, "|- TRIGGER_CFG: OBJECT_TYPE:%d OFFSET:%d VAL:%d\n",
				ptcfg->obj_type, ptcfg->offset, ptcfg->val);
		}
		mxt_patch_write_trigger_cfg(data, ptcfg, trigger_action);
		ulpos += sizeof(struct trigger_cfg);
	}
	// Predefined Action
	mxt_patch_predefined_action(data, ptrgg->act_id, ptrgg->act_val, trigger_action);

	return ulpos;
}

int mxt_patch_parse_event(struct mxt_data *data, u8* ppatch, bool do_action)
{
	struct user_event* pevent = NULL;
	struct event_cfg* pecfg = NULL;
	u32 i = 0, ulpos = 0;
	int error = 0;

	pevent = (struct user_event*)ppatch;

	if (!do_action) {
		__mxt_patch_debug(data, "EVENT ID:%d OPT:%d CFG:%d\n",
			pevent->eid, pevent->option, pevent->cfg_cnt);
	}
	ulpos += sizeof(struct user_event);

	// Event Config Parsing
	for(i = 0; i < pevent->cfg_cnt; i++) {
		pecfg = (struct event_cfg*)(ppatch+ulpos);
		if (!do_action) {
			__mxt_patch_debug(data, "|- EVENT_CFG: OBJECT_TYPE:%d OFFSET:%d VAL:%d\n",
				pecfg->obj_type, pecfg->offset, pecfg->val);
		}
		error = mxt_patch_write_event_cfg(data, pecfg, do_action);
		if (error)
			i = pevent->cfg_cnt+1;

		ulpos += sizeof(struct event_cfg);
	}

	return ulpos;
}

static int mxt_patch_parse_header(struct mxt_data *data, u8* ppatch, u16* pstage_addr, u16* ptrigger_addr, u16* pevent_addr)
{
	struct patch_header* ppheader = NULL;
	u32 i = 0, ulpos = 0;

	ppheader = (struct patch_header*)ppatch;

	TOUCH_PATCH_INFO_MSG("%s \n", __func__);

	TOUCH_PATCH_INFO_MSG( "PATCH MAGIC:%X SIZE:%d DATE:%d VER:%d OPT:%d DBG:%d TMR:%d STG:%d TRG:%d EVT:%d\n",
		ppheader->magic, ppheader->size, ppheader->date, ppheader->version, ppheader->option, ppheader->debug,
		ppheader->timer_id, ppheader->stage_cnt, ppheader->trigger_cnt, ppheader->event_cnt);

	if (ppheader->version != MXT_PATCH_VERSION) {
		TOUCH_PATCH_INFO_MSG( "MXT_PATCH_VERSION ERR\n");
	}

	ulpos = sizeof(struct patch_header);

	for(i = 0; i < ppheader->stage_cnt; i++) { /* Stage Def Parsing */
		if (pstage_addr != NULL) {
			pstage_addr[i] = (u16)ulpos;
		}
		ulpos += mxt_patch_parse_stage(data, ppatch+ulpos, NULL, NULL, false);
	}

	for(i = 0; i < ppheader->trigger_cnt; i++) { /* Trigger Parsing */
		if (ptrigger_addr != NULL) {
			ptrigger_addr[i] = (u16)ulpos;
		}
		ulpos += mxt_patch_parse_trigger(data, ppatch+ulpos, NULL, false, 0);
	}

	for(i = 0; i < ppheader->event_cnt; i++) { /* Event */
		if (pevent_addr != NULL) {
			pevent_addr[i] = (u16)ulpos;
		}
		ulpos += mxt_patch_parse_event(data, ppatch+ulpos, false);
	}

	if (ppheader->size != ulpos) { /* Patch Size Check */
		TOUCH_PATCH_INFO_MSG("Size Error %d != %d \n", ppheader->size, ulpos);
		return 0;
	} else{
		TOUCH_PATCH_INFO_MSG("Size OK= %d \n", ulpos);
	}

	return ulpos;
}

int mxt_patch_run_stage(struct mxt_data *data)
{
	struct stage_def* psdef = NULL;
	u8* ppatch = data->patch.patch;
	u16* pstage_addr = data->patch.stage_addr;
	u16 tline_addr[MXT_PATCH_MAX_TLINE] = {0};
	u8 tline_cnt = 0;
	u8 cur_stage = data->patch.cur_stage;

	__mxt_patch_debug(data, "RUN STAGE:%d\n", cur_stage);

	if (unlikely(!ppatch || !pstage_addr)) {
		TOUCH_PATCH_INFO_MSG( "%s pstage_addr is null\n", __func__);
		return 1;
	}

	psdef = (struct stage_def*)(ppatch+pstage_addr[cur_stage]);
	data->patch.cur_stage_opt = psdef->option;

	mxt_patch_parse_stage(data, (u8*)psdef, tline_addr, &tline_cnt, true);

	if (unlikely(!data->patch.tline_addr)) {
		data->patch.tline_addr = kzalloc(MXT_PATCH_MAX_TLINE, GFP_KERNEL);
	}

	if (unlikely(!data->patch.check_cnt)) {
		data->patch.check_cnt = kzalloc(MXT_PATCH_MAX_TLINE, GFP_KERNEL);
	}

	if (unlikely(!data->patch.tline_addr || !data->patch.check_cnt)) {
		TOUCH_PATCH_INFO_MSG( "tline_addr alloc error\n");
		return 1;
	}

	memcpy(data->patch.tline_addr, tline_addr, tline_cnt*sizeof(u16));
	memset(data->patch.check_cnt, 0, tline_cnt*sizeof(u16));
	data->patch.tline_cnt = tline_cnt;
	data->patch.run_stage = 1;
	data->patch.skip_test = 0;
	data->patch.stage_timestamp = jiffies_to_msecs(jiffies);
	__mxt_patch_ddebug(data, "Stage[%d] %d\n", cur_stage, data->patch.stage_timestamp);

	return 0;
}

static int mxt_patch_test_source(struct mxt_data *data, u16* psrc_item)
{
	int i = 0;
	u8* ppatch = data->patch.patch;
	u16* pstage_addr = data->patch.stage_addr;
	u8	cur_stage = data->patch.cur_stage;
	u32 curr_time = jiffies_to_msecs(jiffies);
	u32 time_diff = TIME_WRAP_AROUND(data->patch.stage_timestamp, curr_time);
	struct stage_def* psdef = NULL;
	u16* ptline_addr = NULL;
	u16* pcheck_cnt = NULL;

	if (!ppatch || !pstage_addr) {
		TOUCH_PATCH_INFO_MSG( "%s pstage_addr is null\n", __func__);
		return 1;
	}

	if (!data->patch.run_stage) {
		mxt_patch_run_stage(data);
	}

	if (!data->patch.run_stage)
		return 0;

	for(i = 0; i< data->patch.tline_cnt; i++) {
		ptline_addr = data->patch.tline_addr;
		pcheck_cnt = data->patch.check_cnt;

		if (!ptline_addr || !pcheck_cnt) {
			TOUCH_PATCH_INFO_MSG( "ptline_addr is null\n");
			return 1;
		}

		__mxt_patch_ddebug(data, "STAGE:%d, TEST:%d\n", cur_stage, i);

		mxt_patch_parse_test_line(data,
				ppatch+pstage_addr[cur_stage]+ptline_addr[i],
				psrc_item, &pcheck_cnt[i], true);

		psdef = (struct stage_def*)(ppatch+pstage_addr[cur_stage]);
		if (psdef->reset_period) {
			if (time_diff > psdef->reset_period*10) {
				pcheck_cnt[i] = 0;
				__mxt_patch_ddebug(data,
					"RESET CNT STAGE:%d, TEST:%d RESET:%d DIF:%d\n",
					cur_stage, i, psdef->reset_period, time_diff);
				data->patch.stage_timestamp = jiffies_to_msecs(jiffies);
			}
		}

		if (data->patch.skip_test) {
			__mxt_patch_debug(data, "REMAINED TEST SKIP\n");
			return 0;
		}
	}

	return 0;
}

static void mxt_patch_init_tsrc(struct test_src* tsrc)
{
	tsrc->charger = -1;
	tsrc->finger_cnt = -1;
	tsrc->area = -1;
	tsrc->amp = -1;
	tsrc->sum_size = -1;
	tsrc->tch_ch = -1;
	tsrc->atch_ch = -1;
	tsrc->key_cnt = -1;
	tsrc->key_val = -1;
	tsrc->status = -1;
	tsrc->user1 = t255_user[0];
	tsrc->user2 = t255_user[1];
	tsrc->user3 = t255_user[2];
	tsrc->user4 = t255_user[3];
	tsrc->user5 = t255_user[4];
	tsrc->user6 = t255_user[5];
}

static int mxt_patch_make_source(struct mxt_data *data, struct test_src* tsrc)
{
	if (tsrc->charger >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_CHARG]= tsrc->charger;
	if (tsrc->finger_cnt >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_FCNT]= tsrc->finger_cnt;
	if (tsrc->area >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_AREA]= tsrc->area;
	if (tsrc->amp >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_AMP]= tsrc->amp;
	if (tsrc->sum_size >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_SUM]= tsrc->sum_size;
	if (tsrc->tch_ch >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_TCH]= tsrc->tch_ch;
	if (tsrc->atch_ch >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_ATCH]= tsrc->atch_ch;
	if (tsrc->key_cnt >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_KCNT]= tsrc->key_cnt;
	if (tsrc->key_val >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_KVAL]= tsrc->key_val;
	if (tsrc->status >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_T9STATUS]= tsrc->status;
	if (tsrc->user1 >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_USER1] = tsrc->user1;
	if (tsrc->user2 >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_USER2] = tsrc->user2;
	if (tsrc->user3 >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_USER3] = tsrc->user3;
	if (tsrc->user5 >= 0)
		data->patch.src_item[MXT_PATCH_ITEM_USER5] = tsrc->user5;

	return 0;
}

static int mxt_patch_start_stage(struct mxt_data *data)
{
	if (data->patch.patch) {
		mxt_patch_stop_timer(data);
		data->patch.start = true;
		data->patch.cur_stage = 0;
		data->patch.run_stage = false;

		if (data->patch.start_stage) {
			data->patch.cur_stage = data->patch.start_stage;
		}
		__mxt_patch_debug(data, "PATCH: START STAGE %d\n", data->patch.cur_stage);

		mxt_patch_init_tpos(data, &tpos_data);

		mxt_patch_init_supp(data, &tsupp_data);

		return 0;
	}

	return 1;
}

static int mxt_patch_test_trigger(struct mxt_data *data,
		struct mxt_message *message, u8 option)
{
	int i = 0;
	u8* ppatch = data->patch.patch;
	u16* ptrigger_addr = data->patch.trigger_addr;
	u8	trigger_cnt = data->patch.trigger_cnt;
	u8	tmsg[MXT_PATCH_MAX_MSG_SIZE] = {0};

	if (!ppatch || !ptrigger_addr) {
		TOUCH_PATCH_INFO_MSG( "%s ptrigger_addr is null\n", __func__);
		return 1;
	}

	memset(tmsg, 0, MXT_PATCH_MAX_MSG_SIZE);
	tmsg[0] =  message->reportid;
	memcpy(&tmsg[1], message->message, 8);

	for(i = 0; i< trigger_cnt; i++) {
		mxt_patch_parse_trigger(data, ppatch+ptrigger_addr[i], tmsg, true, option);
	}

	return 0;
}

int mxt_patch_event(struct mxt_data *data, u8 event_id)
{
	u8* ppatch = NULL;
	u16* pevent_addr = NULL;

	TOUCH_INFO_MSG("Patch event %d\n", event_id);

	if (!data) {
		TOUCH_PATCH_INFO_MSG("%s addr is null\n", __func__);
		return 1;
	}

	ppatch = data->patch.patch;
	pevent_addr = data->patch.event_addr;

	if (!ppatch || !pevent_addr) {
		TOUCH_PATCH_INFO_MSG("%s addr is null\n", __func__);
		return 1;
	}

	if (event_id < data->patch.event_cnt) {
		mxt_patch_parse_event(data, ppatch+pevent_addr[event_id], true);
	}

	return 0;
}

static void mxt_patch_T6_object(struct mxt_data *data,
		struct mxt_message *message)
{
	/* Normal mode */
	if (message->message[0] == 0x00) {
		__mxt_patch_debug(data, "PATCH: NORMAL\n");
		if (data->patch.cal_flag == 1) {
			mxt_patch_start_stage(data);
			data->patch.cal_flag = 0;
		}
	}

	/* Calibration */
	if (message->message[0] & 0x10) {
		__mxt_patch_debug(data, "PATCH: CAL\n");
		data->patch.cal_flag = 1;
	}

	/* Reset */
	if (message->message[0] & 0x80) {
		__mxt_patch_debug(data, "PATCH: RESET\n");
		data->patch.start_stage = 0;
		data->patch.cal_flag = 1;
	}
}

static void mxt_patch_T9_object(struct mxt_data *data,
		struct mxt_message *message)
{
	int id = 0;
	u8 *msg = message->message;
	struct test_src tsrc = {0};

	id = data->reportids[message->reportid].index;

	mxt_patch_init_tsrc(&tsrc);

	data->fingers[id].x = ((msg[1] << 4) | ((msg[3] >> 4) & 0xf));
	data->fingers[id].y = ((msg[2] << 4) | (msg[3] & 0xf));
	data->fingers[id].x >>= 2;
	data->fingers[id].y >>= 2;

	tsrc.area = msg[4];
	tsrc.amp = msg[5];
	tsrc.status = msg[0];

	if (data->patch.start) {
		mxt_patch_make_source(data, &tsrc);

		if (data->patch.cur_stage_opt&0x02) {
			if ((msg[0] & MXT_DETECT_MSG_MASK) !=
					MXT_DETECT_MSG_MASK) {
				if (msg[0] & MXT_SUPPRESS_MSG_MASK) {
					mxt_patch_check_supp(data, &tsupp_data);
				}
			}
		}
	}
}

static void mxt_patch_T15_object(struct mxt_data *data, struct mxt_message *message)
{
	struct test_src tsrc = {0};
	unsigned long keystates = message->message[MXT_MSG_T15_KEYSTATE];
	u8 key_cnt = 0;
	int i = 0;

	for(i = 0; i < 8; i++) {
		if (test_bit(i, &keystates)) {
			key_cnt++;
		}
	}
	mxt_patch_init_tsrc(&tsrc);
	tsrc.key_cnt = key_cnt;
	tsrc.key_val = keystates;

	if (data->patch.start) {
		mxt_patch_make_source(data, &tsrc);
		if (data->patch.option & 0x02)
			mxt_patch_test_source(data, data->patch.src_item);
	}
}

static u8 check_pattern_tracking_condition(struct mxt_data *data,
		struct test_src* tsrc, struct touch_pos* tpos)
{
	u8 rtn = 0;
	int i = 0;

	/* 1. checking edge area for ghost touches */
	if (tsrc->finger_cnt) {
		for (i = 0; i < MXT_MAX_FINGER; i++) {
			if ((data->fingers[i].state != MXT_STATE_INACTIVE) &&
			    (data->fingers[i].state != MXT_STATE_RELEASE)) {
				if ((data->fingers[i].x < tpos_data.xlo_limit) ||
				    (data->fingers[i].x > tpos_data.xhi_limit) ||
				    (data->fingers[i].y < tpos_data.ylo_limit) ||
				    (data->fingers[i].y > tpos_data.yhi_limit)){
					rtn = 1;
				}
			}
		}
	}

	/* 2. checking amp, size, t57 messages for ghost touches */
	switch (tsrc->finger_cnt) {
		case 0:
			rtn = 0;
			break;
		case 1:
			if ((tsrc->amp < tpos_data.amp) &&
			    (tsrc->area < tpos_data.area) &&
			    (tsrc->sum_size < tpos_data.sum_size_t57) &&
			    (tsrc->tch_ch < tpos_data.tch_count_t57) &&
			    (tsrc->atch_ch < tpos_data.atch_count_t57)) {
				rtn = 1;
			}
			break;
		case 2:
			if ((tsrc->amp > tpos_data.amp_2finger_min) &&
			    (tsrc->amp < tpos_data.amp_2finger_max) &&
			    (tsrc->area > tpos_data.area_2finger_min) &&
			    (tsrc->area < tpos_data.area_2finger_max) &&
			    (tsrc->sum_size > tpos_data.sum_size_t57_2finger_min) &&
			    (tsrc->sum_size < tpos_data.sum_size_t57_2finger_max) &&
			    (tsrc->tch_ch > tpos_data.tch_count_t57_2finger_min) &&
			    (tsrc->tch_ch < tpos_data.tch_count_t57_2finger_max) &&
			    (tsrc->atch_ch > tpos_data.atch_count_t57_2finger_min) &&
			    (tsrc->atch_ch < tpos_data.atch_count_t57_2finger_max)) {
				rtn = 1;
			}
			break;
		case 3:
			if ((tsrc->amp > tpos_data.amp_3finger_min) &&
			    (tsrc->amp < tpos_data.amp_3finger_max) &&
			    (tsrc->area > tpos_data.area_3finger_min) &&
			    (tsrc->area < tpos_data.area_3finger_max) &&
			    (tsrc->sum_size > tpos_data.sum_size_t57_3finger_min) &&
			    (tsrc->sum_size < tpos_data.sum_size_t57_3finger_max) &&
			    (tsrc->tch_ch > tpos_data.tch_count_t57_3finger_min) &&
			    (tsrc->tch_ch < tpos_data.tch_count_t57_3finger_max) &&
			    (tsrc->atch_ch > tpos_data.atch_count_t57_3finger_min) &&
			    (tsrc->atch_ch < tpos_data.atch_count_t57_3finger_max)) {
				rtn = 1;
			}
			break;
		default: /* over 4 touches */
			if ((tsrc->amp > tpos_data.amp_mfinger_min) &&
			    (tsrc->amp < tpos_data.amp_mfinger_max) &&
			    (tsrc->area > tpos_data.area_mfinger_min) &&
			    (tsrc->area < tpos_data.area_mfinger_max) &&
			    (tsrc->sum_size > tpos_data.sum_size_t57_mfinger_min) &&
			    (tsrc->sum_size < tpos_data.sum_size_t57_mfinger_max) &&
			    (tsrc->tch_ch > tpos_data.tch_count_t57_mfinger_min) &&
			    (tsrc->tch_ch < tpos_data.tch_count_t57_mfinger_max) &&
			    (tsrc->atch_ch > tpos_data.atch_count_t57_mfinger_min) &&
			    (tsrc->atch_ch < tpos_data.atch_count_t57_mfinger_max)) {
				rtn = 1;
			}
			break;
	}

	return rtn;
}

static void mxt_patch_T57_object(struct mxt_data *data,
		struct mxt_message *message)
{
	struct test_src tsrc = {0};
	u8 *msg = message->message;
	u8 finger_cnt = 0;
	int i = 0;

	mxt_patch_init_tsrc(&tsrc);

	for (i = 0; i < MXT_MAX_FINGER; i++) {
		if ((data->fingers[i].state != MXT_STATE_INACTIVE) &&
		    (data->fingers[i].state != MXT_STATE_RELEASE))
			finger_cnt++;
	}

	tsrc.finger_cnt = finger_cnt;

	tsrc.sum_size = msg[0] | (msg[1] << 8);
	tsrc.tch_ch = msg[2] | (msg[3] << 8);
	tsrc.atch_ch = msg[4] | (msg[5] << 8);

	tsrc.area = data->patch.src_item[MXT_PATCH_ITEM_AREA];
	tsrc.amp = data->patch.src_item[MXT_PATCH_ITEM_AMP];

	if (data->patch.start) {
		if ((data->patch.option & 0x01)== 0x01 && !finger_cnt)
			return;

		mxt_patch_make_source(data, &tsrc);
		mxt_patch_test_source(data, data->patch.src_item);
	}

	if ((data->patch.cur_stage_opt & 0x01) &&
	     check_pattern_tracking_condition(data, &tsrc, &tpos_data)
	     && finger_cnt) {
		for (i = 0; i < MXT_MAX_FINGER; i++) {
			if ((data->fingers[i].state != MXT_STATE_INACTIVE) &&
			    (data->fingers[i].state != MXT_STATE_RELEASE)) {
				mxt_patch_check_pattern(data,
					&tpos_data, i, data->fingers[i].x,
					data->fingers[i].y, finger_cnt);
			}
		}
	}

	if (finger_cnt == 0) {
		mxt_patch_init_tpos(data, &tpos_data);
	}

}

static void mxt_patch_T61_object(struct mxt_data *data, struct mxt_message *message)
{
	int id = 0;
	u8 *msg = message->message;
	id = data->reportids[message->reportid].index;

	if ((id != data->patch.timer_id) || ((msg[0] & 0xa0) != 0xa0))
		return;

	__mxt_patch_debug(data, "END STAGE %d TIMER\n", data->patch.cur_stage);

	if ((data->patch.cur_stage+1) == data->patch.stage_cnt) {
		if (data->patch.period == 0) {
			__mxt_patch_debug(data, "EX-STAGE\n");
		} else {
			data->patch.start = false;
			__mxt_patch_debug(data, "END ALL STAGE\n");
		}
	} else {
		data->patch.cur_stage++;
		data->patch.run_stage = false;
	}

	if (!data->patch.run_stage) {
		mxt_patch_run_stage(data);
	}
}

static void mxt_patch_T100_object(struct mxt_data *data,
		struct mxt_message *message)
{
	u8 id, index;
	u8 *msg = message->message;
	u8 touch_type = 0, touch_event = 0, touch_detect = 0;
	u16 x, y;
	struct test_src tsrc;

	index = data->reportids[message->reportid].index;

	mxt_patch_init_tsrc(&tsrc);

	/* Treate screen messages */
	if (index < MXT_T100_SCREEN_MESSAGE_NUM_RPT_ID) {
		if (index == MXT_T100_SCREEN_MSG_FIRST_RPT_ID){
			data->patch.finger_cnt = msg[1];
			tsrc.finger_cnt = data->patch.finger_cnt;
			tsrc.tch_ch = (msg[3] << 8) | msg[2];
			tsrc.atch_ch = (msg[5] << 8) | msg[4];
			tsrc.sum_size = (msg[7] << 8) | msg[6];

			if(data->patch.start){
				mxt_patch_make_source(data, &tsrc);
				if((data->patch.option & 0x08) == 0x08) {
					mxt_patch_test_source(data, data->patch.src_item);
				}
			}
			return;
		}
	}

	if(index >= MXT_T100_SCREEN_MESSAGE_NUM_RPT_ID){
		//u8 i=0, stylus_cnt=0, large_cnt=0;

		/* Treate touch status messages */
		id = index - MXT_T100_SCREEN_MESSAGE_NUM_RPT_ID;
		touch_detect = msg[0] >> MXT_T100_DETECT_MSG_MASK;
		touch_type = (msg[0] & 0x70) >> 4;
		touch_event = msg[0] & 0x0F;

		switch (touch_type)	{
			case MXT_T100_TYPE_PATCH_FINGER:
			case MXT_T100_TYPE_PASSIVE_STYLUS:
				x = msg[1] | (msg[2] << 8);
				y = msg[3] | (msg[4] << 8);

				tsrc.amp = msg[5];
				tsrc.area2 = msg[6]&0x3f;
				tsrc.area = msg[7]&0x1f;

				if(data->patch.start){
					if((data->patch.option & 0x01)== 0x01 && !touch_detect)
						return;

					mxt_patch_make_source(data, &tsrc);
					mxt_patch_test_source(data, data->patch.src_item);
				}

			break;
		}
	}
}

void mxt_patch_message(struct mxt_data *data, struct mxt_message *message)
{
	u8 reportid = 0, type = 0;
	reportid = message->reportid;

	if (reportid > data->max_reportid)
		return;

	type = data->reportids[reportid].type;
	switch (type) {
		case MXT_GEN_COMMAND_T6:
			mxt_patch_T6_object(data, message);
			break;
		case MXT_TOUCH_MULTI_T9:
			mxt_patch_T9_object(data, message);
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			mxt_patch_T15_object(data, message);
			break;
		case MXT_PROCI_EXTRATOUCHSCREENDATA_T57:
			mxt_patch_T57_object(data, message);
			break;
		case MXT_SPT_TIMER_T61:
			mxt_patch_T61_object(data, message);
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			mxt_patch_T100_object(data, message);
			break;
	}

	if (data->patch.trigger_cnt && type) {
		mxt_patch_test_trigger(data, message, data->charging_mode);
	}
}

int mxt_patch_init(struct mxt_data *data, u8* ppatch)
{
	struct mxt_patch *patch_info = &data->patch;
	struct patch_header *ppheader = NULL;
	u16 stage_addr[64] = {0};
	u16 trigger_addr[64] = {0};
	u16 event_addr[64] = {0};
	u32 patch_size = 0;
	int ret;

	TOUCH_PATCH_INFO_MSG("%s \n", __func__);

	if (!ppatch) {
		TOUCH_PATCH_INFO_MSG("%s patch file error\n", __func__);
		return 1;
	}

	patch_size = mxt_patch_parse_header(data, ppatch,
			stage_addr, trigger_addr, event_addr);
	if (!patch_size) {
		TOUCH_PATCH_INFO_MSG("%s patch_size error\n", __func__);
		return 1;
	}

	ppheader = (struct patch_header*)ppatch;
	patch_info->timer_id = ppheader->timer_id;
	patch_info->option = ppheader->option;
	patch_info->debug = 0;
	patch_info->stage_cnt = ppheader->stage_cnt;
	patch_info->trigger_cnt = ppheader->trigger_cnt;
	patch_info->event_cnt = ppheader->event_cnt;
	patch_info->date = ppheader->date;

	if (data->patch.src_item)
		kfree(data->patch.src_item);
	patch_info->src_item = kzalloc(MXT_PATCH_ITEM_END * sizeof(u16),
			GFP_KERNEL);
	if (!patch_info->src_item) {
		TOUCH_PATCH_INFO_MSG("%s No mem(src_item)\n", __func__);
		ret = 1;
		goto error;
	}

	if (patch_info->stage_cnt) {
		if (patch_info->stage_addr)
			kfree(patch_info->stage_addr);

		patch_info->stage_addr = kzalloc(
			patch_info->stage_cnt * sizeof(u16), GFP_KERNEL);
		if (!patch_info->stage_addr) {
			TOUCH_PATCH_INFO_MSG("stage_addr alloc error\n");
			ret = 1;
			goto error;
		}
		memcpy(patch_info->stage_addr, stage_addr,
				patch_info->stage_cnt*sizeof(u16));
	}

	if (patch_info->trigger_cnt) {
		if (patch_info->trigger_addr)
			kfree(patch_info->trigger_addr);

		patch_info->trigger_addr = kzalloc(
			patch_info->trigger_cnt*sizeof(u16), GFP_KERNEL);
		if (!patch_info->trigger_addr) {
			TOUCH_PATCH_INFO_MSG("trigger_addr alloc error\n");
			ret = 1;
			goto error;
		}
		memcpy(patch_info->trigger_addr, trigger_addr,
				patch_info->trigger_cnt*sizeof(u16));
	}

	if (patch_info->event_cnt) {
		if (patch_info->event_addr) {
			kfree(patch_info->event_addr);
		}
		patch_info->event_addr = kzalloc(
			patch_info->event_cnt * sizeof(u16), GFP_KERNEL);
		if (!patch_info->event_addr) {
			TOUCH_PATCH_INFO_MSG("event_addr alloc error\n");
			ret = 1;
			goto error;
		}
		memcpy(patch_info->event_addr,
				event_addr, patch_info->event_cnt*sizeof(u16));
	}

	mxt_patch_load_t71data(data);
	mxt_patch_init_userdata();

	return 0;

error:
	if (patch_info->src_item) {
		kfree(patch_info->src_item);
		patch_info->src_item = NULL;
	}
	if (patch_info->stage_addr) {
		kfree(patch_info->stage_addr);
		patch_info->stage_addr = NULL;
	}
	if (patch_info->trigger_addr) {
		kfree(patch_info->trigger_addr);
		patch_info->trigger_addr = NULL;
	}
	return ret;
}

