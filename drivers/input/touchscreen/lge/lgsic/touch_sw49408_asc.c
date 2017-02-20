/* touch_sw49408_asc.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: daehyun.gil@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define TS_MODULE "[asc]"

#include <touch_core.h>
#include "touch_sw49408.h"

#define TOUCH_SHOW(ret, buf, fmt, args...) \
	(ret += snprintf(buf + ret, PAGE_SIZE - ret, fmt, ##args))

static char asc_str[3][8] = {"NORMAL", "ACUTE", "OBTUSE"};
static char onhand_str[2][20] = {"NOT_IN_HAND", "IN_HAND"};

bool sw49408_asc_usable(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);

	return ((d->asc.use_asc == ASC_ON) ? true : false);
}

bool sw49408_asc_delta_chk_usable(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	struct sw49408_asc_info *asc = &(d->asc);

	if (!sw49408_asc_usable(dev))
		return false;

	if ((asc->max_delta_cnt < MIN_DELTA_CNT) ||
			(asc->max_delta_cnt > MAX_DELTA_CNT))
		return false;

	return ((asc->use_delta_chk == DELTA_CHK_ON) ? true : false);
}

static int sw49408_asc(struct device *dev, u32 code, u32 value)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 rdata = 0;
	u32 wdata = 0;
	int ret = 0;

	switch (code) {
	case ASC_READ_MAX_DELTA:
		sw49408_reg_read(dev, MAX_DELTA, &rdata, sizeof(rdata));

		if (rdata < LOW_DELTA_LIMIT) {
			TOUCH_I("%s : rdata(%d) is too low\n", __func__,
					rdata);
			rdata = LOW_DELTA_LIMIT;
		} else if (rdata > HIGH_DELTA_LIMIT) {
			TOUCH_I("%s : rdata(%d) is too high\n", __func__,
					rdata);
			rdata = HIGH_DELTA_LIMIT;
		}

		ret = (int)rdata;
		break;
	case ASC_GET_FW_SENSITIVITY:
		sw49408_reg_read(dev, TOUCH_MAX_R, &rdata, sizeof(rdata));

		if ((rdata < SENSITIVITY_LOW_LIMIT) ||
				(rdata > SENSITIVITY_HIGH_LIMIT)) {
			TOUCH_I("%s : rdata(%d) is abnormal\n", __func__,
					rdata);
				rdata = TEMP_SENSITIVITY;
		}

		if (!(d->asc.s_updated))
			d->asc.s_updated = true;

		d->asc.normal_s = rdata;
		d->asc.acute_s = SENSITIVITY_LOW_LIMIT;
		d->asc.obtuse_s = SENSITIVITY_HIGH_LIMIT;
		TOUCH_I("%s : normal_s = %d, acute_s = %d, obtuse_s = %d\n",
				__func__, d->asc.normal_s,
				d->asc.acute_s, d->asc.obtuse_s);
		break;
	case ASC_WRITE_SENSITIVITY:
#if 1
		if (!(d->asc.s_updated)) {
			TOUCH_I("%s : fw_sensitivity is not updated",
					__func__);
			break;
		}

		sw49408_reg_read(dev, TOUCH_MAX_R, &rdata, sizeof(rdata));

		if (value == NORMAL_SENSITIVITY)
			wdata = d->asc.normal_s;
		else if (value == ACUTE_SENSITIVITY)
			wdata = d->asc.acute_s;
		else if (value == OBTUSE_SENSITIVITY)
			wdata = d->asc.obtuse_s;
		else
			wdata = rdata;

		sw49408_reg_write(dev, TOUCH_MAX_W, &wdata, sizeof(wdata));
		TOUCH_I("%s : TOUCH_MAX Register value = %d->%d\n", __func__,
				rdata, wdata);
#else
		TOUCH_I("%s : ASC_WRITE_SENSITIVITY is not used\n", __func__);
#endif
		break;
	case ASC_WRITE_ONHAND:
		if ((value == NOT_IN_HAND) || (value == IN_HAND)) {
			sw49408_reg_read(dev, ONHAND_R, &rdata, sizeof(rdata));
			wdata = value;
			if (rdata == wdata)
				break;
			sw49408_reg_write(dev, ONHAND_W, &wdata, sizeof(wdata));
			TOUCH_I("%s : ONHAND Register value = %d->%d\n",
					__func__, rdata, wdata);
		} else {
			TOUCH_I("%s : unknown onhand value (%d)\n", __func__,
					value);
		}
		break;
	default:
		break;
	}

	return ret;
}

static void sw49408_asc_get_delta(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct sw49408_asc_info *asc = &(d->asc);
	int delta = 0;
	int i = 0;

	if (!sw49408_asc_delta_chk_usable(dev)) {
		TOUCH_I("%s : sw49408_asc_delta_chk_usable() error\n", __func__);
		return;
	}

	if (atomic_read(&ts->state.fb) != FB_RESUME) {
		TOUCH_I("%s : fb state is not FB_RESUME\n", __func__);
		return;
	}

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		TOUCH_I("%s : core state is not CORE_NORMAL\n", __func__);
		return;
	}

	mutex_lock(&ts->lock);
	delta = sw49408_asc(dev, ASC_READ_MAX_DELTA, 0);
	mutex_unlock(&ts->lock);

	if (asc->curr_delta_cnt < asc->max_delta_cnt) {
		asc->delta[(asc->curr_delta_cnt)++] = delta;
	} else {
		for (i = 0; i < asc->max_delta_cnt - 1; i++)
			asc->delta[i] = asc->delta[i + 1];
		asc->delta[i] = delta;
	}

	asc->delta_updated = true;

	return;
}

static void sw49408_asc_change_sensitivity(struct device *dev,
		int next_sensitivity)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	struct sw49408_asc_info *asc = &(d->asc);

	if (!sw49408_asc_usable(dev)) {
		TOUCH_I("%s : sw49408_asc_usable() error\n", __func__);
		return;
	}

	if (asc->curr_sensitivity == next_sensitivity)
		return;

	TOUCH_I("%s : sensitity(curr->next) = (%s->%s)\n", __func__,
			asc_str[asc->curr_sensitivity],
			asc_str[next_sensitivity]);

	sw49408_asc(dev, ASC_WRITE_SENSITIVITY, next_sensitivity);

	asc->curr_sensitivity = next_sensitivity;

	return;
}

static void sw49408_asc_update_sensitivity(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct sw49408_asc_info *asc = &(d->asc);
	int next_sensitivity = NORMAL_SENSITIVITY;
	int i = 0;
	int delta_avr = 0;
	unsigned char str[100] = {0, };
	int ret = 0;

	if (!sw49408_asc_delta_chk_usable(dev)) {
		TOUCH_I("%s : sw49408_asc_delta_chk_usable() error\n", __func__);
		return;
	}

	if (!(asc->delta_updated)) {
		TOUCH_I("%s : delta is not updated\n", __func__);
		return;
	}

	if (asc->curr_delta_cnt < asc->max_delta_cnt) {
		TOUCH_I("%s : curr_celta_cnt(%d) < max_delta_cnt(%d)\n",
				__func__, asc->curr_delta_cnt,
				asc->max_delta_cnt);
		return;
	}

	for (i = 0; i < asc->max_delta_cnt; i++) {
		delta_avr += asc->delta[i];
		ret += snprintf(str + ret, 100 - ret, "%4d ", asc->delta[i]);
	}

	delta_avr /= asc->max_delta_cnt;

	TOUCH_I("%s : delta_avr = %4d [%s]\n", __func__, delta_avr, str);

	if (delta_avr < asc->low_delta_thres)
		next_sensitivity = ACUTE_SENSITIVITY;
	else if (delta_avr > asc->high_delta_thres)
		next_sensitivity = OBTUSE_SENSITIVITY;
	else
		next_sensitivity = NORMAL_SENSITIVITY;

	asc->delta_updated = false;

	mutex_lock(&ts->lock);
	sw49408_asc_change_sensitivity(dev, next_sensitivity);
	mutex_unlock(&ts->lock);

	return;
}

static void sw49408_asc_finger_input_check_work_func(
		struct work_struct *finger_input_work)
{
	struct sw49408_asc_info *asc =
		container_of(to_delayed_work(finger_input_work),
				struct sw49408_asc_info, finger_input_work);
	struct sw49408_data *d = asc->d;
	struct touch_core_data *ts = to_touch_core(d->dev);

	if (!sw49408_asc_delta_chk_usable(d->dev)) {
		TOUCH_I("%s : sw49408_asc_delta_chk_usable() error\n", __func__);
		return;
	}

	if ((ts->tcount == 1) && !(asc->delta_updated))
		sw49408_asc_get_delta(d->dev);

	if ((ts->tcount == 0) && asc->delta_updated) {
		if (asc->curr_delta_cnt < asc->max_delta_cnt)
			asc->delta_updated = false;
		else
			sw49408_asc_update_sensitivity(d->dev);
	}

	return;
}

void sw49408_asc_toggle_delta_check(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct sw49408_asc_info *asc = &(d->asc);
	int connect_status = atomic_read(&ts->state.connect);
	int wireless_status = atomic_read(&ts->state.wireless);
	int call_status = atomic_read(&ts->state.incoming_call);
	int onhand_status = atomic_read(&ts->state.onhand);
	int qcover_status = ts->lpwg.qcover;

	if (!sw49408_asc_usable(dev)) {
		TOUCH_I("%s : sw49408_asc_usable() error\n", __func__);
		return;
	}

	TOUCH_I("%s : connect = %d, wireless = %d, call = %d, onhand = %d, qcover = %d\n",
			__func__, connect_status, wireless_status,
			call_status, onhand_status, qcover_status);

	if ((connect_status != CONNECT_INVALID) ||
			(wireless_status != 0) ||
			(call_status != INCOMING_CALL_IDLE) ||
			(qcover_status != HALL_FAR)) {
		asc->use_delta_chk = DELTA_CHK_OFF;
		sw49408_asc_change_sensitivity(dev, NORMAL_SENSITIVITY);
	} else {
		switch (onhand_status) {
		case NOT_IN_HAND:
			asc->use_delta_chk = DELTA_CHK_ON;
			sw49408_asc_change_sensitivity(dev, ACUTE_SENSITIVITY);
			break;
		case IN_HAND:
			asc->use_delta_chk = DELTA_CHK_ON;
			sw49408_asc_change_sensitivity(dev, NORMAL_SENSITIVITY);
			break;
		default:
			TOUCH_I("%s : unknown onhand_status\n", __func__);
			break;
		}
	}

	TOUCH_I("%s : curr_sensitivity = %s, use_delta_chk = %d\n", __func__,
			asc_str[asc->curr_sensitivity], asc->use_delta_chk);

	return;
}

void sw49408_asc_update_qcover_status(struct device *dev, int new_qcover)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int old_qcover = ts->lpwg.qcover;

	if (old_qcover != new_qcover) {
		TOUCH_I("%s : qcover status : %d -> %d\n", __func__,
				old_qcover, new_qcover);
		ts->lpwg.qcover = new_qcover;
		sw49408_asc_toggle_delta_check(dev);
	}

	return;
}

static void sw49408_asc_init_works(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);

	INIT_DELAYED_WORK(&(d->asc.finger_input_work),
			sw49408_asc_finger_input_check_work_func);

	return;
}

void sw49408_asc_write_onhand(struct device *dev, u32 value)
{
	sw49408_asc(dev, ASC_WRITE_ONHAND, value);

	return;
}

void sw49408_asc_control(struct device *dev, bool on_off)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	struct sw49408_asc_info *asc = &(d->asc);
	int i = 0;

	if (asc->use_asc == on_off) {
		TOUCH_I("%s : ASC is already %s\n", __func__,
				(on_off == ASC_ON) ? "ON" : "OFF");
		return;
	}

	TOUCH_I("%s : ASC %s start\n", __func__,
			(on_off == ASC_ON) ? "ON" : "OFF");

	if (on_off == ASC_ON) {
		sw49408_asc(dev, ASC_GET_FW_SENSITIVITY, 0);

		asc->use_delta_chk = DELTA_CHK_OFF;
		asc->delta_updated = false;
		asc->curr_delta_cnt = 0;
		for (i = 0; i < MAX_DELTA_CNT; i++)
			asc->delta[i] = 0;

		asc->use_asc = ASC_ON;
		sw49408_asc_toggle_delta_check(dev);
	} else {
		asc->use_asc = ASC_OFF;
		touch_msleep(50);
		sw49408_asc(dev, ASC_WRITE_SENSITIVITY, NORMAL_SENSITIVITY);
		asc->curr_sensitivity = NORMAL_SENSITIVITY;
	}

	TOUCH_I("%s : ASC %s end\n", __func__,
			(on_off == ASC_ON) ? "ON" : "OFF");

	return;
}

static ssize_t show_sw49408_asc_param(struct device *dev, char *buf)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	struct sw49408_asc_info *asc = &(d->asc);
	int ret = 0;

	TOUCH_SHOW(ret, buf, "1. use_asc = %d\n",
			asc->use_asc);
	TOUCH_SHOW(ret, buf, "2. low_delta_thres = %d\n",
			asc->low_delta_thres);
	TOUCH_SHOW(ret, buf, "3. high_delta_thres = %d\n",
			asc->high_delta_thres);
	TOUCH_SHOW(ret, buf, "4. max_delta_cnt = %d\n",
			asc->max_delta_cnt);

	return ret;
}

static ssize_t store_sw49408_asc_param(struct device *dev, const char *buf,
		size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct sw49408_asc_info *asc = &(d->asc);
	int variable = 0;
	int value = 0;

	if (sscanf(buf, "%d %d", &variable, &value) <= 0)
		return count;

	switch (variable) {
	case 1:
		if ((value == ASC_OFF) || (value == ASC_ON)) {
			mutex_lock(&ts->lock);
			sw49408_asc_control(dev, value);
			mutex_unlock(&ts->lock);
		}
		break;
	case 2:
		if ((value >= LOW_DELTA_THRES_LIMIT) &&
				(value <= HIGH_DELTA_THRES_LIMIT))
			asc->low_delta_thres = value;
		break;
	case 3:
		if ((value >= LOW_DELTA_THRES_LIMIT) &&
				(value <= HIGH_DELTA_THRES_LIMIT))
			asc->high_delta_thres = value;
		break;
	case 4:
		if ((value >= MIN_DELTA_CNT) && (value <= MAX_DELTA_CNT))
			asc->max_delta_cnt = value;
		break;
	default:
		break;
	}

	return count;
}

static ssize_t show_sw49408_asc_onhand(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	value = atomic_read(&ts->state.onhand);
	TOUCH_I("%s : %s(%d)\n", __func__, onhand_str[value], value);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", value);

	return ret;
}

static ssize_t store_sw49408_asc_onhand(struct device *dev, const char *buf,
		size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;
	int ret = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	switch (value) {
	case DPC_UNKNOWN:
	case DPC_FLAT_STATIC:
	case DPC_HIDDEN:
		value = NOT_IN_HAND;
		break;
	case DPC_IN_HAND:
	case DPC_FACING:
		value = IN_HAND;
		break;
	default:
		TOUCH_I("%s : Unknown %d\n", __func__, value);
		return count;
	}

	if (atomic_read(&ts->state.onhand) == value)
		return count;

	atomic_set(&ts->state.onhand, value);
	ret = touch_blocking_notifier_call(NOTIFY_ONHAND_STATE,
			&ts->state.onhand);
	TOUCH_I("%s : %s(%d), ret = %d\n",
			__func__, onhand_str[value], value, ret);

	return count;
}

static TOUCH_ATTR(asc, show_sw49408_asc_param, store_sw49408_asc_param);
static TOUCH_ATTR(onhand, show_sw49408_asc_onhand, store_sw49408_asc_onhand);

static struct attribute *sw49408_asc_attribute_list[] = {
	&touch_attr_asc.attr,
	&touch_attr_onhand.attr,
	NULL,
};

static const struct attribute_group sw49408_asc_attribute_group = {
	.attrs = sw49408_asc_attribute_list,
};

void sw49408_asc_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	ret = sysfs_create_group(&ts->kobj, &sw49408_asc_attribute_group);

	if (ret < 0)
		TOUCH_E("failed to create sysfs (ret = %d)\n", ret);

	return;
}

static void sw49408_asc_get_dts(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct device_node *np = ts->dev->of_node;
	struct sw49408_asc_info *asc = &(d->asc);

	PROPERTY_BOOL(np, "use_asc", asc->use_asc);
	PROPERTY_U32(np, "low_delta_thres", asc->low_delta_thres);
	PROPERTY_U32(np, "high_delta_thres", asc->high_delta_thres);
	PROPERTY_U32(np, "max_delta_cnt", asc->max_delta_cnt);

	if (!(asc->use_asc == ASC_OFF) && !(asc->use_asc == ASC_ON))
		asc->use_asc = ASC_OFF;

	if ((asc->low_delta_thres < LOW_DELTA_THRES_LIMIT) ||
			(asc->low_delta_thres > HIGH_DELTA_THRES_LIMIT))
		asc->low_delta_thres = LOW_DELTA_THRES_LIMIT;

	if ((asc->high_delta_thres < LOW_DELTA_THRES_LIMIT) ||
			(asc->high_delta_thres > HIGH_DELTA_THRES_LIMIT))
		asc->high_delta_thres = HIGH_DELTA_THRES_LIMIT;

	if ((asc->max_delta_cnt < MIN_DELTA_CNT) ||
			(asc->max_delta_cnt > MAX_DELTA_CNT))
		asc->max_delta_cnt = MIN_DELTA_CNT;

	TOUCH_I("%s : use_asc = %d, low_delta_thres = %d, high_delta_thres = %d, max_delta_cnt = %d\n",
			__func__, asc->use_asc, asc->low_delta_thres,
			asc->high_delta_thres, asc->max_delta_cnt);

	return;
}

void sw49408_asc_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 old_dbg_mask = atomic_read(&ts->state.debug_option_mask);
	u32 asc_dbg_mask = DEBUG_OPTION_1;

	TOUCH_I("%s : start\n", __func__);

	if (d == NULL) {
		TOUCH_E("d is NULL\n");
		return;
	}

	memset(&(d->asc), 0, sizeof(struct sw49408_asc_info));

	d->asc.d = d;

	sw49408_asc_init_works(dev);
	sw49408_asc_get_dts(dev);

	if (sw49408_asc_usable(dev))
		atomic_set(&ts->state.debug_option_mask,
				(old_dbg_mask | asc_dbg_mask));

	TOUCH_I("%s : end\n", __func__);

	return;
}

