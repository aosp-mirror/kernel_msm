/*
 * cyttsp5_mt_common.c
 * Cypress TrueTouch(TM) Standard Product V5 Multi-Touch Reports Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2014 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include "cyttsp5_regs.h"
#include <linux/input/mt.h>


#define MT_PARAM_SIGNAL(md, sig_ost) PARAM_SIGNAL(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_MIN(md, sig_ost) PARAM_MIN(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_MAX(md, sig_ost) PARAM_MAX(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_FUZZ(md, sig_ost) PARAM_FUZZ(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_FLAT(md, sig_ost) PARAM_FLAT(md->pdata->frmwrk, sig_ost)

extern int gesture_id;
extern u32 cyttsp_gesture_count[GESTURE_MAX];

static void cyttsp5_mt_lift_all(struct cyttsp5_mt_data *md)
{
	int max = md->si->tch_abs[CY_TCH_T].max;

	if (md->num_prv_rec != 0) {
		if (md->mt_function.report_slot_liftoff)
			md->mt_function.report_slot_liftoff(md, max);
		input_sync(md->input);
		md->num_prv_rec = 0;
	}
}

static void cyttsp5_get_touch_axis(struct cyttsp5_mt_data *md,
	int *axis, int size, int max, u8 *xy_data, int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		tp_log_vdebug(
			"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p xy_data[%d]=%02X(%d) bofs=%d\n",
			__func__, *axis, *axis, size, max, xy_data, next,
			xy_data[next], xy_data[next], bofs);
		*axis = *axis + ((xy_data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;

	tp_log_vdebug(
		"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p xy_data[%d]=%02X(%d)\n",
		__func__, *axis, *axis, size, max, xy_data, next,
		xy_data[next], xy_data[next]);
}

static void cyttsp5_get_touch_hdr(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch, u8 *xy_mode)
{
	struct cyttsp5_sysinfo *si = md->si;
	enum cyttsp5_tch_hdr hdr;

	for (hdr = CY_TCH_TIME; hdr < CY_TCH_NUM_HDR; hdr++) {
		if (!si->tch_hdr[hdr].report)
			continue;
		cyttsp5_get_touch_axis(md, &touch->hdr[hdr],
			si->tch_hdr[hdr].size,
			si->tch_hdr[hdr].max,
			xy_mode + si->tch_hdr[hdr].ofs,
			si->tch_hdr[hdr].bofs);
		tp_log_vdebug( "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_hdr_string[hdr],
			touch->hdr[hdr], touch->hdr[hdr]);
	}

	tp_log_debug(
		"%s: time=%X tch_num=%d lo=%d noise=%d counter=%d\n",
		__func__,
		touch->hdr[CY_TCH_TIME],
		touch->hdr[CY_TCH_NUM],
		touch->hdr[CY_TCH_LO],
		touch->hdr[CY_TCH_NOISE],
		touch->hdr[CY_TCH_COUNTER]);
}

static void cyttsp5_get_touch_record(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch, u8 *xy_data)
{
	struct cyttsp5_sysinfo *si = md->si;
	enum cyttsp5_tch_abs abs;

	for (abs = CY_TCH_X; abs < CY_TCH_NUM_ABS; abs++) {
		if (!si->tch_abs[abs].report)
			continue;
		cyttsp5_get_touch_axis(md, &touch->abs[abs],
			si->tch_abs[abs].size,
			si->tch_abs[abs].max,
			xy_data + si->tch_abs[abs].ofs,
			si->tch_abs[abs].bofs);

		tp_log_vdebug( "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_abs_string[abs],
			touch->abs[abs], touch->abs[abs]);
	}
}

static void cyttsp5_mt_process_touch(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch)
{
	struct cyttsp5_sysinfo *si = md->si;
	int tmp;
	bool flipped;


	/* Orientation is signed */
	touch->abs[CY_TCH_OR] = (int8_t)touch->abs[CY_TCH_OR];

	if (md->pdata->flags & CY_MT_FLAG_FLIP) {
		tmp = touch->abs[CY_TCH_X];
		touch->abs[CY_TCH_X] = touch->abs[CY_TCH_Y];
		touch->abs[CY_TCH_Y] = tmp;
		if (touch->abs[CY_TCH_OR] > 0)
			touch->abs[CY_TCH_OR] =
				md->or_max - touch->abs[CY_TCH_OR];
		else
			touch->abs[CY_TCH_OR] =
				md->or_min - touch->abs[CY_TCH_OR];
		flipped = true;
	} else
		flipped = false;

	if (md->pdata->flags & CY_MT_FLAG_INV_X) {
		if (flipped)
			touch->abs[CY_TCH_X] = si->sensing_conf_data.res_y -
				touch->abs[CY_TCH_X];
		else
			touch->abs[CY_TCH_X] = si->sensing_conf_data.res_x -
				touch->abs[CY_TCH_X];
		touch->abs[CY_TCH_OR] *= -1;
	}
	if (md->pdata->flags & CY_MT_FLAG_INV_Y) {
		if (flipped)
			touch->abs[CY_TCH_Y] = si->sensing_conf_data.res_x -
				touch->abs[CY_TCH_Y];
		else
			touch->abs[CY_TCH_Y] = si->sensing_conf_data.res_y -
				touch->abs[CY_TCH_Y];
		touch->abs[CY_TCH_OR] *= -1;
	}

	/* Convert MAJOR/MINOR from mm to resolution */
	tmp = touch->abs[CY_TCH_MAJ] * 100 * si->sensing_conf_data.res_x;
	touch->abs[CY_TCH_MAJ] = tmp / si->sensing_conf_data.len_x;
	tmp = touch->abs[CY_TCH_MIN] * 100 * si->sensing_conf_data.res_x;
	touch->abs[CY_TCH_MIN] = tmp / si->sensing_conf_data.len_x;

	tp_log_vdebug( "%s: flip=%s inv-x=%s inv-y=%s x=%04X(%d) y=%04X(%d)\n",
		__func__, flipped ? "true" : "false",
		md->pdata->flags & CY_MT_FLAG_INV_X ? "true" : "false",
		md->pdata->flags & CY_MT_FLAG_INV_Y ? "true" : "false",
		touch->abs[CY_TCH_X], touch->abs[CY_TCH_X],
		touch->abs[CY_TCH_Y], touch->abs[CY_TCH_Y]);
}

static void cyttsp5_report_event(struct cyttsp5_mt_data *md, int event,
		int value)
{
	int sig = MT_PARAM_SIGNAL(md, event);

	if (sig != CY_IGNORE_VALUE)
		input_report_abs(md->input, sig, value);
}

static void cyttsp5_check_and_save(struct cyttsp5_core_data *cd,
	enum cyttsp5_gesture_num gesture,
	unsigned int *reprot_gesture_key_value)
{
	if(cd == NULL)
		return;

	if (IS_GESTURE_ENABLE(gesture, cd->easy_wakeup_gesture)) {
		*reprot_gesture_key_value =	cd->wakeup_keys->keys[gesture];
		cyttsp_gesture_count[gesture]++;
		tp_log_debug("%s: Gesture detected,cyttsp_gesture_count[%d]=%d!\n",__func__, gesture,cyttsp_gesture_count[gesture]);
	} else {
		tp_log_warning("%s: TP Gesture: %d not enabled!!\n",__func__, gesture);
	}
}
static void cyttsp5_mt_gesture_report(struct cyttsp5_core_data *cd)
{
	unsigned int reprot_gesture_key_value = 0;
	unsigned long lTimerCur = jiffies;
	static unsigned long lTimerPre = 0;
	static unsigned long lTimerNext = 0;
	static int first_int_flag = true;

	if (true == first_int_flag) {
		lTimerPre = jiffies;
		lTimerCur = jiffies;
		first_int_flag = false;
		tp_log_debug("%s:Gesture first interrupt!\n",__func__);
	}

	if (time_before(lTimerCur, lTimerPre + lTimerNext)) {
		tp_log_debug("%s:Gesture Timer Interval is less than 1 senond!\n",__func__);
		return;
	} else {
		lTimerNext = GESTURE_TIMER_INTERVAL;
		lTimerPre = jiffies;
	}

	switch(gesture_id) {
		case CYTTSP5_GESTURE_DOUBLE_CLICK:
			cyttsp5_check_and_save(cd,
				GESTURE_DOUBLE_CLICK,
				&reprot_gesture_key_value);
			tp_log_debug("%s:double click detected! gesture_id = 0x%x\n", __func__ , gesture_id);
			break;
		case CYTTSP5_GESTURE_SINGLE_CLICK:
		case CYTTSP5_GESTURE_SLIDE:
			cyttsp5_check_and_save(cd,
				GESTURE_SINGLE_CLICK,
				&reprot_gesture_key_value);
			tp_log_debug("%s:sigdel click or slide detected! gesture_id = 0x%x\n", __func__ , gesture_id);
			break;
		case CYTTSP5_GESTURE_LETTER_C:
			cyttsp5_check_and_save(cd,
				GESTURE_LETTER_c,
				&reprot_gesture_key_value);
			tp_log_debug("%s:letter c detected! gesture_id = 0x%x\n", __func__ , gesture_id);
			break;
		case CYTTSP5_GESTURE_LETTER_E:
			cyttsp5_check_and_save(cd,
				GESTURE_LETTER_e,
				&reprot_gesture_key_value);
			tp_log_debug("%s:letter e detected! gesture_id = 0x%x\n", __func__ , gesture_id);
			break;
		case CYTTSP5_GESTURE_LETTER_M:
			cyttsp5_check_and_save(cd,
				GESTURE_LETTER_m,
				&reprot_gesture_key_value);
			tp_log_debug("%s:letter m detected! gesture_id = 0x%x\n", __func__ , gesture_id);
			break;
		case CYTTSP5_GESTURE_LETTER_W:
			cyttsp5_check_and_save(cd,
				GESTURE_LETTER_w,
				&reprot_gesture_key_value);
			tp_log_debug("%s:letter w detected! gesture_id = 0x%x\n", __func__ , gesture_id);
			break;
		default:
			tp_log_warning("%s:unknow gesture detected! gesture_id = 0x%x\n", __func__ , gesture_id);
			return;
	}

	/*Gesture detected, report the event.*/
	if (KEY_WAKEUP == reprot_gesture_key_value) {
		tp_log_warning("%s:reprot_gesture_key_value = %d\n", __func__, reprot_gesture_key_value);

		input_mt_slot(cd->md.input, 0);
		input_mt_report_slot_state(cd->md.input, MT_TOOL_FINGER, 1);
		input_sync(cd->md.input);
		input_mt_slot(cd->md.input, 0);
		input_mt_report_slot_state(cd->md.input, MT_TOOL_FINGER, 0);
		input_sync(cd->md.input);
	}

	return;
}

static int cyttp5_palm_report(struct cyttsp5_mt_data *md,int value)
{
	struct device *dev = md->input->dev.parent;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	unsigned int reprot_gesture_key_value = 0;

	if (APP_ENABLE_GESTURE(GESTURE_PALM_COVERED) & cd->easy_wakeup_gesture)
	{
		tp_log_warning("%s:GESTURE_PALM_COVERED detected!",__func__);
		reprot_gesture_key_value = cd->wakeup_keys->keys[GESTURE_PALM_COVERED];
		goto report_key;
	}

	tp_log_info("%s:palm cover is not enabled = %d\n", __func__, cd->easy_wakeup_gesture);
	return 0;

report_key:

	tp_log_debug("%s:reprot_gesture_key_value = %d",__func__, reprot_gesture_key_value);
	input_report_key(cd->md.input, reprot_gesture_key_value, value);
	input_sync(cd->md.input);

	return 1;
}

static void cyttsp5_get_mt_touches(struct cyttsp5_mt_data *md,
		struct cyttsp5_touch *tch, int num_cur_tch)
{
	struct cyttsp5_sysinfo *si = md->si;
	int sig;
	int i, j, t = 0;

	DECLARE_BITMAP(ids, si->tch_abs[CY_TCH_T].max);
	int mt_sync_count = 0;
	u8 *tch_addr;

	bitmap_zero(ids, si->tch_abs[CY_TCH_T].max);
	memset(tch->abs, 0, sizeof(tch->abs));

	for (i = 0; i < num_cur_tch; i++) {
		tch_addr = si->xy_data + (i * si->desc.tch_record_size);
		cyttsp5_get_touch_record(md, tch, tch_addr);

		/*  Discard proximity event */
		if (tch->abs[CY_TCH_O] == CY_OBJ_PROXIMITY) {
			tp_log_vdebug( "%s: Discarding proximity event\n",
					__func__);
			continue;
		} else if (tch->abs[CY_TCH_O] == CY_OBJ_HOVER) {
			tch->abs[CY_TCH_P] = 0;
		}

		/* Validate track_id */
		t = tch->abs[CY_TCH_T];
		if (t < md->t_min || t > md->t_max) {
			tp_log_err( "%s: tch=%d -> bad trk_id=%d max_id=%d\n",
				__func__, i, t, md->t_max);
			if (md->mt_function.input_sync)
				md->mt_function.input_sync(md->input);
			mt_sync_count++;
			continue;
		}

		/* Lift-off */
		if (tch->abs[CY_TCH_E] == CY_EV_LIFTOFF) {
			tp_log_debug( "%s: t=%d e=%d lift-off\n",
				__func__, t, tch->abs[CY_TCH_E]);
			goto cyttsp5_get_mt_touches_pr_tch;
		}

		/* Process touch */
		cyttsp5_mt_process_touch(md, tch);

		/* use 0 based track id's */
		t -= md->t_min;

		sig = MT_PARAM_SIGNAL(md, CY_ABS_ID_OST);
		if (sig != CY_IGNORE_VALUE) {
			if (md->mt_function.input_report)
				md->mt_function.input_report(md->input, sig,
						t, tch->abs[CY_TCH_O]);
			__set_bit(t, ids);
		}

		/* If touch type is hover, send P as distance, reset P */
		if (tch->abs[CY_TCH_O] == CY_OBJ_HOVER) {
			/* CY_ABS_D_OST signal must be in touch framework */
			cyttsp5_report_event(md, CY_ABS_D_OST,
					tch->abs[CY_TCH_P]);
			tch->abs[CY_TCH_P] = 0;
		}

		/* all devices: position and pressure fields */
		for (j = 0; j <= CY_ABS_W_OST; j++) {
			if (!si->tch_abs[j].report)
				continue;
			cyttsp5_report_event(md, CY_ABS_X_OST + j,
					tch->abs[CY_TCH_X + j]);
		}

		/* Get the extended touch fields */
		for (j = 0; j < CY_NUM_EXT_TCH_FIELDS; j++) {
			if (!si->tch_abs[j].report)
				continue;
			cyttsp5_report_event(md, CY_ABS_MAJ_OST + j,
					tch->abs[CY_TCH_MAJ + j]);
		}
		if (md->mt_function.input_sync)
			md->mt_function.input_sync(md->input);
		mt_sync_count++;

cyttsp5_get_mt_touches_pr_tch:
		tp_log_debug(
			"%s: t=%d x=%d y=%d z=%d M=%d m=%d o=%d e=%d obj=%d tip=%d\n",
			__func__, t,
			tch->abs[CY_TCH_X],
			tch->abs[CY_TCH_Y],
			tch->abs[CY_TCH_P],
			tch->abs[CY_TCH_MAJ],
			tch->abs[CY_TCH_MIN],
			tch->abs[CY_TCH_OR],
			tch->abs[CY_TCH_E],
			tch->abs[CY_TCH_O],
			tch->abs[CY_TCH_TIP]);
	}

	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input,
				si->tch_abs[CY_TCH_T].max, mt_sync_count, ids);

	md->num_prv_rec = num_cur_tch;
}

/* read xy_data for all current touches */
static int cyttsp5_xy_worker(struct cyttsp5_mt_data *md)
{
	struct cyttsp5_sysinfo *si = md->si;
	int max_tch = si->sensing_conf_data.max_tch;
	struct cyttsp5_touch tch;
	u8 num_cur_tch;
	int rc = 0;

	cyttsp5_get_touch_hdr(md, &tch, si->xy_mode + 3);

	num_cur_tch = tch.hdr[CY_TCH_NUM];
	if (num_cur_tch > max_tch) {
		tp_log_err( "%s: Num touch err detected (n=%d)\n",
			__func__, num_cur_tch);
		num_cur_tch = max_tch;
	}

	if (tch.hdr[CY_TCH_LO]&&!md->palm_down) {
		tp_log_info( "%s: Large area detected\n", __func__);
		num_cur_tch = 0;
		rc = cyttp5_palm_report(md,1);
		md->palm_down = true;
		if(1 == rc){
			return 0;
		}
	} else if (!tch.hdr[CY_TCH_LO]&&md->palm_down){
		tp_log_info( "%s: Large area removed\n", __func__);
		rc = cyttp5_palm_report(md,0);
		md->palm_down = false;
		if(1 == rc){
			return 0;
		}
	}

	if (num_cur_tch == 0 && md->num_prv_rec == 0)
		goto cyttsp5_xy_worker_exit;

	/* extract xy_data for all currently reported touches */
	tp_log_vdebug( "%s: extract data num_cur_tch=%d\n", __func__,
		num_cur_tch);
	if (num_cur_tch)
		cyttsp5_get_mt_touches(md, &tch, num_cur_tch);
	else
		cyttsp5_mt_lift_all(md);

	rc = 0;

cyttsp5_xy_worker_exit:
	return rc;
}
/* no need dummy for gesture*/
static int cyttsp5_mt_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;
	int rc;

	if (!md->si) {
		md->si = _cyttsp5_request_sysinfo(dev);
		if (!md->si) {
			tp_log_err("%s:md->si request again fail\n",__func__);
			return -EINVAL;
		}
	}

	if (md->si->xy_mode[2] !=  md->si->desc.tch_report_id)
		return 0;

	/* core handles handshake */
	mutex_lock(&md->mt_lock);
	rc = cyttsp5_xy_worker(md);
	mutex_unlock(&md->mt_lock);
	if (rc < 0)
		tp_log_err( "%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

static int cyttsp5_mt_wake_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	mutex_lock(&md->mt_lock);
	cyttsp5_mt_gesture_report(cd);
	mutex_unlock(&md->mt_lock);
	return 0;
}

static int cyttsp5_startup_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	tp_log_info("%s:in \n",__func__);
	if (!md->si) {
		tp_log_info("%s:md->si is null, request again\n",__func__);
		md->si = _cyttsp5_request_sysinfo(dev);
		if (!md->si) {
			tp_log_err("%s:md->si request again fail\n",__func__);
			return -EINVAL;
		}
	}

	mutex_lock(&md->mt_lock);
	cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return 0;
}

static int cyttsp5_suspend_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	if (!md->si) {
		tp_log_info("%s:md->si is null, request again\n",__func__);
		md->si = _cyttsp5_request_sysinfo(dev);
		if (!md->si) {
			tp_log_err("%s:md->si request again fail\n",__func__);
			return -EINVAL;
		}
	}
	mutex_lock(&md->mt_lock);
	/* release all touch */
	cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return 0;
}

static int cyttsp5_mt_open(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);
	cd->number_of_open_input_device++;
	cd->pm_runtime_usage_count++;

	tp_log_vdebug( "%s: setup subscriptions\n", __func__);

	/* set up touch call back */
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_IRQ, CY_MODULE_MT,
		cyttsp5_mt_attention, CY_MODE_OPERATIONAL);

	/* set up startup call back */
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_STARTUP, CY_MODULE_MT,
		cyttsp5_startup_attention, 0);

	/* set up suspend release touch call back */
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_SUSPEND, CY_MODULE_MT,
		cyttsp5_suspend_attention, 0);

	/* set up wakeup call back */
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_WAKE, CY_MODULE_MT,
		cyttsp5_mt_wake_attention, 0);

	return 0;
}

static void cyttsp5_mt_close(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_IRQ, CY_MODULE_MT,
		cyttsp5_mt_attention, CY_MODE_OPERATIONAL);

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_STARTUP, CY_MODULE_MT,
		cyttsp5_startup_attention, 0);

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_SUSPEND, CY_MODULE_MT,
		cyttsp5_suspend_attention, 0);

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_WAKE, CY_MODULE_MT,
		cyttsp5_mt_wake_attention, 0);

	if (cd->pm_runtime_usage_count > 0) {
		pm_runtime_put(dev);
		cd->pm_runtime_usage_count--;
	}
	cd->number_of_open_input_device--;
}

static int init_easy_wakeup_key_value(struct device *dev)
{
	int i = 0;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if(NULL  == cd->wakeup_keys)
	{
		tp_log_err("%s: wakeup keys is NULL!",__func__);
		return -1;
	}

	if(cd->wakeup_keys->size > GESTURE_MAX)
	{
		tp_log_err("%s: wakeup keys size not match!",__func__);
		return -1;
	}

	set_bit(KEY_POWER, cd->md.input->keybit);

	for (i = 0; i < cd->wakeup_keys->size; ++i){
		tp_log_debug("%s: wakeup_keys[%d] = %d",__func__, i, cd->wakeup_keys->keys[i]);
		set_bit(cd->wakeup_keys->keys[i], cd->md.input->keybit);
	}

	return 0;
}

static int cyttsp5_setup_input_device(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;
	int signal = CY_IGNORE_VALUE;
	int max_x, max_y, max_p, min, max, slots_max;
	int max_x_tmp, max_y_tmp;
	int i;
	int rc;

	tp_log_vdebug( "%s: Initialize event signals\n", __func__);
	__set_bit(EV_ABS, md->input->evbit);
	__set_bit(EV_REL, md->input->evbit);
	__set_bit(EV_KEY, md->input->evbit);
#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, md->input->propbit);
#endif

	rc = init_easy_wakeup_key_value(dev);
	if (rc) {
		tp_log_err("%s:init easy wakeup key value fail, rc = %d\n", __func__, rc);
	}

	if ( md->si ){
		max_p = md->si->sensing_conf_data.max_z;
		slots_max = md->si->tch_abs[CY_TCH_T].max;
	} else {
		/* only input_register_early config true will run this code*/
		max_p = md->pdata->p_max;
		slots_max = md->pdata->slots_max;
		tp_log_info("%s:input_register_early config true,use p_max=%d slots_max=%d\n",
						__func__,max_p,slots_max);
	}

	/* If virtualkeys enabled, don't use all screen */
	if (md->pdata->flags & CY_MT_FLAG_VKEYS) {
		max_x_tmp = md->pdata->vkeys_x;
		max_y_tmp = md->pdata->vkeys_y;
	} else {
		if ( md->si ) {
			max_x_tmp = md->si->sensing_conf_data.res_x;
			max_y_tmp = md->si->sensing_conf_data.res_y;
		} else {
			/* only input_register_early config true will run this code*/
			max_x_tmp = md->pdata->x_max;
			max_y_tmp = md->pdata->y_max;
			tp_log_info("%s:input_register_early config true,use x_max=%d y_max=%d\n",
							__func__,max_x_tmp,max_y_tmp);
		}
	}

	/* get maximum values from the sysinfo data */
	if (md->pdata->flags & CY_MT_FLAG_FLIP) {
		max_x = max_y_tmp - 1;
		max_y = max_x_tmp - 1;
	} else {
		max_x = max_x_tmp - 1;
		max_y = max_y_tmp - 1;
	}

	/* set event signal capabilities */
	for (i = 0; i < NUM_SIGNALS(md->pdata->frmwrk); i++) {
		signal = MT_PARAM_SIGNAL(md, i);
		if (signal != CY_IGNORE_VALUE) {
			__set_bit(signal, md->input->absbit);

			min = MT_PARAM_MIN(md, i);
			max = MT_PARAM_MAX(md, i);
			if (i == CY_ABS_ID_OST) {
				/* shift track ids down to start at 0 */
				max = max - min;
				min = min - min;
			} else if (i == CY_ABS_X_OST)
				max = max_x;
			else if (i == CY_ABS_Y_OST)
				max = max_y;
			else if (i == CY_ABS_P_OST)
				max = max_p;

			input_set_abs_params(md->input, signal, min, max,
				MT_PARAM_FUZZ(md, i), MT_PARAM_FLAT(md, i));
			tp_log_debug( "%s: register signal=%02X min=%d max=%d\n",
				__func__, signal, min, max);
		}
	}

	md->or_min = MT_PARAM_MIN(md, CY_ABS_OR_OST);
	md->or_max = MT_PARAM_MAX(md, CY_ABS_OR_OST);

	md->t_min = MT_PARAM_MIN(md, CY_ABS_ID_OST);
	md->t_max = MT_PARAM_MAX(md, CY_ABS_ID_OST);

	rc = md->mt_function.input_register_device(md->input,slots_max);
	if (rc < 0)
		tp_log_err( "%s: Error, failed register input device r=%d\n",
			__func__, rc);
	else
		md->input_device_registered = true;

	return rc;
}

static int cyttsp5_setup_input_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;
	int rc;

	md->si = _cyttsp5_request_sysinfo(dev);
	if (!md->si)
		return -EINVAL;

	rc = cyttsp5_setup_input_device(dev);

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_STARTUP, CY_MODULE_MT,
		cyttsp5_setup_input_attention, 0);

	return rc;
}

int cyttsp5_mt_probe(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	struct cyttsp5_mt_platform_data *mt_pdata;
	int rc = 0;

	if (!pdata || !pdata->mt_pdata) {
		tp_log_err("%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	mt_pdata = pdata->mt_pdata;

	cyttsp5_init_function_ptrs(md);

	mutex_init(&md->mt_lock);
	md->dev = dev;
	md->pdata = mt_pdata;

	/* Create the input device and register it. */
	tp_log_info("%s: Create the input device and register it\n",
		__func__);
	md->input = input_allocate_device();
	if (!md->input) {
		tp_log_err( "%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENOSYS;
		goto error_alloc_failed;
	}

	if (md->pdata->inp_dev_name) {
		md->input->name = md->pdata->inp_dev_name;
	} else {
		md->input->name = CYTTSP5_MT_NAME;
	}

	scnprintf(md->phys, sizeof(md->phys), "%s/input%d", dev_name(dev),
			cd->phys_num++);
	md->input->phys = md->phys;
	md->input->dev.parent = md->dev;
	md->input->open = cyttsp5_mt_open;
	md->input->close = cyttsp5_mt_close;
	md->palm_down = false;
	input_set_drvdata(md->input, md);

	/* get sysinfo */
	md->si = _cyttsp5_request_sysinfo(dev);

	if (md->si) {
		rc = cyttsp5_setup_input_device(dev);
		if (rc) {
			tp_log_err("%s %d:cyttsp5_setup_input_device fail, rc = %d\n",
						__func__, __LINE__, rc);
			goto error_init_input;
		}
	} else {
		if (cd->pdata->core_pdata->input_register_early) {
			tp_log_info("%s:input_register_early config true,will register input devices nowait FW upgrade\n",__func__);
			rc = cyttsp5_setup_input_device(dev);
			if (rc) {
				tp_log_err("%s %d:cyttsp5_setup_input_device fail, rc = %d\n",
							__func__, __LINE__, rc);
				goto error_init_input;
			}
		} else {
			tp_log_err( "%s: Fail get sysinfo pointer from core p=%p\n",
				__func__, md->si);
			_cyttsp5_subscribe_attention(dev, CY_ATTEN_STARTUP,
				CY_MODULE_MT, cyttsp5_setup_input_attention, 0);
		}
	}

	tp_log_info("%s: mt probe successful.\n", __func__);
	return 0;

error_init_input:
	input_free_device(md->input);
error_alloc_failed:
error_no_pdata:
	tp_log_err( "%s failed.\n", __func__);
	return rc;
}

int cyttsp5_mt_release(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	if (md->input_device_registered) {
		input_unregister_device(md->input);
	} else {
		input_free_device(md->input);
		_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_STARTUP,
			CY_MODULE_MT, cyttsp5_setup_input_attention, 0);
	}

	return 0;
}
