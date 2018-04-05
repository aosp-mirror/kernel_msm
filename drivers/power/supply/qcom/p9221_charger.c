/*
 * P9221 Wireless Charger Driver
 *
 * Copyright (C) 2017 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/device.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include "p9221_charger.h"

static const u32 p9221_ov_set_lut[] = {
	17000000, 20000000, 15000000, 13000000,
	11000000, 11000000, 11000000, 11000000};

static int p9221_reg_read_n(struct p9221_charger_data *charger, u16 reg,
			    void *buf, size_t n)
{
	int ret;
	struct i2c_msg msg[2];
	u8 wbuf[2];

	msg[0].addr = charger->client->addr;
	msg[0].flags = charger->client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = wbuf;

	wbuf[0] = (reg & 0xFF00) >> 8;
	wbuf[1] = (reg & 0xFF);

	msg[1].addr = charger->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = n;
	msg[1].buf = buf;

	mutex_lock(&charger->io_lock);
	ret = i2c_transfer(charger->client->adapter, msg, 2);
	mutex_unlock(&charger->io_lock);

	if (ret < 0) {
		/*
		 * Treat -ENOTCONN as -ENODEV to suppress the get/set
		 * prop warnings.
		 */
		int nret = (ret == -ENOTCONN) ? -ENODEV : ret;

		dev_err(&charger->client->dev,
			"i2c read error, reg:%x, ret:%d (%d)\n",
			reg, ret, nret);
		return nret;
	}

	return (ret == 2) ? 0 : -EIO;
}

static int p9221_reg_read_16(struct p9221_charger_data *charger, u16 reg,
			     u16 *val)
{
	u8 buf[2];
	int ret;

	ret = p9221_reg_read_n(charger, reg, buf, 2);
	*val = (buf[1] << 8) | buf[0];
	return ret;
}

static int p9221_reg_read_8(struct p9221_charger_data *charger,
			    u16 reg, u8 *val)
{
	return p9221_reg_read_n(charger, reg, val, 1);
}

static bool p9221_reg_is_8_bit(struct p9221_charger_data *charger, u16 reg)
{
	if (charger->cust_id == P9221R5_CUSTOMER_ID_VAL) {
		switch (reg) {
		case P9221_CHIP_REVISION_REG:
		case P9221R5_VOUT_SET_REG:
		case P9221R5_ILIM_SET_REG:
		case P9221R5_CHARGE_STAT_REG:
		case P9221R5_EPT_REG:
		case P9221R5_SYSTEM_MODE_REG:
		case P9221R5_COM_CHAN_RESET_REG:
		case P9221R5_COM_CHAN_SEND_SIZE_REG:
		case P9221R5_COM_CHAN_SEND_IDX_REG:
		case P9221R5_COM_CHAN_RECV_SIZE_REG:
		case P9221R5_COM_CHAN_RECV_IDX_REG:
		case P9221R5_DEBUG_REG:
		case P9221R5_EPP_Q_FACTOR_REG:
		case P9221R5_EPP_TX_GUARANTEED_POWER_REG:
		case P9221R5_EPP_TX_POTENTIAL_POWER_REG:
		case P9221R5_EPP_TX_CAPABILITY_FLAGS_REG:
		case P9221R5_EPP_RENEGOTIATION_REG:
		case P9221R5_EPP_CUR_RPP_HEADER_REG:
		case P9221R5_EPP_CUR_NEGOTIATED_POWER_REG:
		case P9221R5_EPP_CUR_MAXIMUM_POWER_REG:
		case P9221R5_EPP_CUR_FSK_MODULATION_REG:
		case P9221R5_EPP_REQ_RPP_HEADER_REG:
		case P9221R5_EPP_REQ_NEGOTIATED_POWER_REG:
		case P9221R5_EPP_REQ_MAXIMUM_POWER_REG:
		case P9221R5_EPP_REQ_FSK_MODULATION_REG:
		case P9221R5_VRECT_TARGET_REG:
		case P9221R5_VRECT_KNEE_REG:
		case P9221R5_FOD_SECTION_REG:
		case P9221R5_VRECT_ADJ_REG:
		case P9221R5_ALIGN_X_ADC_REG:
		case P9221R5_ALIGN_Y_ADC_REG:
		case P9221R5_ASK_MODULATION_DEPTH_REG:
		case P9221R5_OVSET_REG:
		case P9221R5_EPP_TX_SPEC_REV_REG:
			return true;
		default:
			return false;
		}
	}

	switch (reg) {
	case P9221_CHIP_REVISION_REG:
	case P9221_CUSTOMER_ID_REG:
	case P9221_CHARGE_STAT_REG:
	case P9221_EPT_REG:
	case P9221_VOUT_SET_REG:
	case P9221_ILIM_SET_REG:
	case P9221_OP_MODE_REG:
	case P9221_COM_REG:
	case P9221_FW_SWITCH_KEY_REG:
	case P9221_ALIGN_X_ADC_REG:
	case P9221_ALIGN_Y_ADC_REG:
		return true;
	default:
		return false;
	}
}

/*
 * Read the reg and return the cooked value. (R5)
 */
static int p9221_reg_read_cooked_r5(struct p9221_charger_data *charger, u16 reg,
				    u16 raw_data, u32 *val)
{
	/* Do the appropriate conversion */
	switch (reg) {
	/* The following raw values */
	case P9221R5_ALIGN_X_ADC_REG:
	case P9221R5_ALIGN_Y_ADC_REG:
		*val = raw_data;
		break;

	/* The following are 12-bit ADC raw values */
	case P9221R5_VOUT_ADC_REG:
	case P9221R5_IOUT_ADC_REG:
	case P9221R5_DIE_TEMP_ADC_REG:
	case P9221R5_EXT_TEMP_REG:
		*val = raw_data & 0xFFF;
		break;

	/* The following are in 0.1 mill- and need to go to micro- */
	case P9221R5_VOUT_SET_REG:	/* 100mV -> uV */
		raw_data *= 100;
		/* Fall through */

	/* The following are in milli- and need to go to micro- */
	case P9221R5_IOUT_REG:		/* mA -> uA */
	case P9221R5_VRECT_REG:		/* mV -> uV */
	case P9221R5_VOUT_REG:		/* mV -> uV */
		/* Fall through */

	/* The following are in kilo- and need to go to their base */
	case P9221R5_OP_FREQ_REG:	/* kHz -> Hz */
	case P9221R5_TX_PINGFREQ_REG:	/* kHz -> Hz */
		*val = raw_data * 1000;
		break;

	case P9221R5_ILIM_SET_REG:
		/* 100mA -> uA, 200mA offset */
		*val = ((raw_data * 100) + 200) * 1000;
		break;

	case P9221R5_OVSET_REG:
		/* uV */
		raw_data &= P9221R5_OVSET_MASK;
		*val = p9221_ov_set_lut[raw_data];
		break;

	default:
		return -ENOENT;
	}

	return 0;
}

/*
 * Read the reg and return the cooked value. (Rx, x != 5)
 */
static int p9221_reg_read_cooked_rx(struct p9221_charger_data *charger, u16 reg,
				    u16 raw_data, u32 *val)
{
	/* Do the appropriate conversion */
	switch (reg) {
	/* The following raw values */
	case P9221_ALIGN_X_ADC_REG:
	case P9221_ALIGN_Y_ADC_REG:
		*val = raw_data;
		break;

	case P9221_RX_IOUT_REG:
		/* uA */
		*val = raw_data * 1000;
		break;

	case P9221_RX_RAWIOUT_REG:
		/* uA */
		*val = (raw_data & P9221_RX_RAWIOUT_MASK) * 1000;
		break;

	case P9221_ILIM_SET_REG:
		/* uA */
		/* raw_data is in 100mA steps */
		*val = (raw_data * 100) * 1000;
		break;

	case P9221_VOUT_SET_REG:
		/* uV */
		/* raw_data is in 100mV steps, 3500mV offset */
		*val = (3500 + (raw_data * 100)) * 1000;
		break;

	case P9221_RX_PINGFREQ_REG:
		/* Hz */
		*val = (raw_data &= P9221_RX_PINGFREQ_MASK) * 1000;
		break;

	case P9221_DIE_TEMP_ADC_REG:
		/* tenths of deg C */
		raw_data &= P9221_DIE_TEMP_ADC_MASK;
		*val = ((((u32)(raw_data * 10) - 13500) *
			830) / 4440) - 2730;
		break;

	case P9221_VOUT_ADC_REG:
		/* uV */
		raw_data &= P9221_VOUT_ADC_MASK;
		*val = ((u64)raw_data * 6 * 2100000) / 4095;
		break;

	case P9221_VRECT_ADC_REG:
		/* uV */
		raw_data &= P9221_VRECT_ADC_MASK;
		*val = ((u64)raw_data * 10 * 2100000) / 4095;
		break;

	case P9221_OP_FREQ_REG:
		/* Hz */
		*val = (64 * 6000 * 1000) / raw_data;
		break;

	default:
		return -ENOENT;
	}

	return 0;
}

/*
 * Read the reg and return the cooked value.
 */
static int p9221_reg_read_cooked(struct p9221_charger_data *charger,
				 u16 reg, u32 *val)
{
	int ret = 0;
	u16 data;

	if (p9221_reg_is_8_bit(charger, reg)) {
		u8 data8 = 0;
		ret = p9221_reg_read_8(charger, reg, &data8);
		data = data8;
	} else {
		ret = p9221_reg_read_16(charger, reg, &data);
	}
	if (ret)
		return ret;

	if (charger->cust_id == P9221R5_CUSTOMER_ID_VAL)
		return p9221_reg_read_cooked_r5(charger, reg, data, val);
	else
		return p9221_reg_read_cooked_rx(charger, reg, data, val);
}

static int p9221_reg_write_n(struct p9221_charger_data *charger, u16 reg,
			     void *buf, size_t n)
{
	int ret;
	u8 *data;
	int datalen = 2 + n;

	data = kmalloc(datalen, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data[0] = reg >> 8;
	data[1] = reg & 0xFF;
	memcpy(&data[2], buf, n);

	mutex_lock(&charger->io_lock);
	ret = i2c_master_send(charger->client, data, datalen);
	mutex_unlock(&charger->io_lock);
	kfree(data);

	if (ret < datalen) {
		/*
		 * Treat -ENOTCONN as -ENODEV to suppress the get/set
		 * prop warnings.
		 */
		int nret = (ret == -ENOTCONN) ? -ENODEV : -EIO;

		dev_err(&charger->client->dev,
			"%s: i2c write error, reg: 0x%x, n: %zd ret: %d (%d)\n",
			__func__, reg, n, ret, nret);
		return nret;
	}

	return 0;
}

static int p9221_reg_write_16(struct p9221_charger_data *charger, u16 reg,
			      u16 val)
{
	return p9221_reg_write_n(charger, reg, &val, 2);
}

static int p9221_reg_write_8(struct p9221_charger_data *charger, u16 reg, u8 val)
{
	return p9221_reg_write_n(charger, reg, &val, 1);
}

/*
 * Uncook the values and write to register
 */
static int p9221_reg_write_cooked_r5(struct p9221_charger_data *charger,
				     u16 reg, u32 val)
{
	int ret = 0;
	u16 data;
	int i;

	/* Do the appropriate conversion */
	switch (reg) {
	case P9221R5_ILIM_SET_REG:
		/* uA -> 0.1A, offset 0.2A */
		if ((val < 2000000) || (val > 1600000))
			return -EINVAL;
		data = (val / (100 * 1000)) - 2;
		break;

	case P9221R5_VOUT_SET_REG:
		/* uV -> 0.1V */
		val /= 1000;
		if (val < 3500 || val > charger->pdata->max_vout_mv)
			return -EINVAL;
		data = val / 100;
		break;

	case P9221R5_OVSET_REG:
		/* uV */
		for (i = 0; i < ARRAY_SIZE(p9221_ov_set_lut); i++) {
			if (val == p9221_ov_set_lut[i])
				break;
		}
		if (i == ARRAY_SIZE(p9221_ov_set_lut))
			return -EINVAL;
		data = i;
		break;
	default:
		return -ENOENT;
	}

	if (p9221_reg_is_8_bit(charger, reg))
		ret = p9221_reg_write_8(charger, reg, data);
	else
		ret = p9221_reg_write_16(charger, reg, data);

	return ret;
}

/*
 * Uncook the values and write to register
 */
static int p9221_reg_write_cooked_rx(struct p9221_charger_data *charger,
				     u16 reg, u32 val)
{
	int ret = 0;
	u16 data;
	int i;

	/* Do the appropriate conversion */
	switch (reg) {
		case P9221_ILIM_SET_REG:
			/* uA */
			if (val > 1500000)
				return -EINVAL;
			data = val / (100 * 1000);
			break;

		case P9221_VOUT_SET_REG:
			/* uV */
			val /= 1000;
			if (val < 3500 || val > charger->pdata->max_vout_mv)
				return -EINVAL;
			data = (val - 3500) / 100;
			break;

		case P9221_OVSET_REG:
			/* uV */
			for (i = 0; i < ARRAY_SIZE(p9221_ov_set_lut); i++) {
				if (val == p9221_ov_set_lut[i])
					break;
			}
			if (i == ARRAY_SIZE(p9221_ov_set_lut))
				return -EINVAL;
			data = i << P9221_OVSET_SHIFT;
			break;
		default:
			return -ENOENT;
	}

	if (p9221_reg_is_8_bit(charger, reg))
		ret = p9221_reg_write_8(charger, reg, data);
	else
		ret = p9221_reg_write_16(charger, reg, data);

	return ret;
}

/*
 * Uncook the values and write to register
 */
static int p9221_reg_write_cooked(struct p9221_charger_data *charger, u16 reg,
				  u32 val)
{
	if (charger->cust_id == P9221R5_CUSTOMER_ID_VAL)
		return p9221_reg_write_cooked_r5(charger, reg, val);
	else
		return p9221_reg_write_cooked_rx(charger, reg, val);
}

static int p9221_write_fod(struct p9221_charger_data *charger)
{
	int ret;
	u16 fod_reg = P9221_FOD_REG;

	if (charger->cust_id == P9221R5_CUSTOMER_ID_VAL)
		fod_reg = P9221R5_FOD_REG;

	ret = p9221_reg_write_n(charger, fod_reg, charger->pdata->fod,
				P9221_NUM_FOD);

	if (ret)
		dev_err(&charger->client->dev,
			"Could not write FOD: %d\n", ret);
	return ret;
}

struct p9221_prop_reg_map_entry p9221_prop_reg_map[] = {
	/* property			register			g, s */
	{POWER_SUPPLY_PROP_CURRENT_NOW,	P9221_RX_IOUT_REG,		1, 0},
	{POWER_SUPPLY_PROP_CURRENT_MAX,	P9221_ILIM_SET_REG,		1, 1},
	{POWER_SUPPLY_PROP_VOLTAGE_NOW,	P9221_VOUT_ADC_REG,		1, 0},
	{POWER_SUPPLY_PROP_VOLTAGE_MAX, P9221_VOUT_SET_REG,		1, 1},
	{POWER_SUPPLY_PROP_TEMP,	P9221_DIE_TEMP_ADC_REG,		1, 0},
};

struct p9221_prop_reg_map_entry p9221_prop_reg_map_r5[] = {
	/* property			register			g, s */
	{POWER_SUPPLY_PROP_CURRENT_NOW,	P9221R5_IOUT_REG,		1, 0},
	{POWER_SUPPLY_PROP_CURRENT_MAX,	P9221R5_ILIM_SET_REG,		1, 1},
	{POWER_SUPPLY_PROP_VOLTAGE_NOW,	P9221R5_VOUT_REG,		1, 0},
	{POWER_SUPPLY_PROP_VOLTAGE_MAX, P9221R5_VOUT_SET_REG,		1, 1},
	{POWER_SUPPLY_PROP_TEMP,	P9221R5_DIE_TEMP_ADC_REG,	1, 0},
};

static struct p9221_prop_reg_map_entry *p9221_get_map_entry(
				struct p9221_charger_data *charger,
				enum power_supply_property prop, bool set)
{
	int i;
	struct p9221_prop_reg_map_entry *p;
	int map_size;

	if (charger->cust_id == P9221R5_CUSTOMER_ID_VAL) {
		p = p9221_prop_reg_map_r5;
		map_size = ARRAY_SIZE(p9221_prop_reg_map_r5);
	} else {
		p = p9221_prop_reg_map;
		map_size = ARRAY_SIZE(p9221_prop_reg_map);
	}


	for (i = 0; i < map_size; i++) {
		if (((set && p->set) || (!set && p->get)) &&
		    (p->prop == prop))
			return p;
		p++;
	}
	return NULL;
}

static int p9221_get_property_reg(struct p9221_charger_data *charger,
				  enum power_supply_property prop,
				  union power_supply_propval *val)
{
	int ret;
	struct p9221_prop_reg_map_entry *p;
	u32 data;

	p = p9221_get_map_entry(charger, prop, false);
	if (p == NULL)
		return -EINVAL;

	if (!charger->online)
		return -ENODEV;

	ret = p9221_reg_read_cooked(charger, p->reg, &data);
	if (ret)
		return ret;

	val->intval = data;
	return 0;
}

static int p9221_set_property_reg(struct p9221_charger_data *charger,
				  enum power_supply_property prop,
				  const union power_supply_propval *val)
{
	struct p9221_prop_reg_map_entry *p;

	p = p9221_get_map_entry(charger, prop, true);
	if (p == NULL)
		return -EINVAL;

	if (!charger->online)
		return -ENODEV;

	return p9221_reg_write_cooked(charger, p->reg, val->intval);
}

static int p9221_get_property(struct power_supply *psy,
			      enum power_supply_property prop,
			      union power_supply_propval *val)
{
	struct p9221_charger_data *charger = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->online;
		break;
	default:
		ret = p9221_get_property_reg(charger, prop, val);
		break;
	}

	if (ret)
		dev_dbg(&charger->client->dev,
			"Couldn't get prop %d, ret=%d\n", prop, ret);
	return ret;
}

static int p9221_set_property(struct power_supply *psy,
			      enum power_supply_property prop,
			      const union power_supply_propval *val)
{
	struct p9221_charger_data *charger = power_supply_get_drvdata(psy);

	return p9221_set_property_reg(charger, prop, val);
}

static int p9221_prop_is_writeable(struct power_supply *psy,
				   enum power_supply_property prop)
{
	struct p9221_charger_data *charger = power_supply_get_drvdata(psy);

	if (p9221_get_map_entry(charger, prop, true))
		return 1;

	return 0;
}

static bool p9221_dc_psy_initialized(struct p9221_charger_data *charger)
{
	if (charger->dc_psy)
		return true;

	charger->dc_psy = power_supply_get_by_name("dc");

	if (!charger->dc_psy)
		return false;

	return true;
}

static int p9221_notifier_cb(struct notifier_block *nb, unsigned long event,
			     void *data)
{
	struct power_supply *psy = data;
	struct p9221_charger_data *charger =
		container_of(nb, struct p9221_charger_data, nb);
	union power_supply_propval prop;
	int ret;

	if (event != PSY_EVENT_PROP_CHANGED)
		goto out;

	if (strcmp(psy->desc->name, "dc") != 0)
		goto out;

	if (!p9221_dc_psy_initialized(charger))
		goto out;

	ret = power_supply_get_property(charger->dc_psy,
				        POWER_SUPPLY_PROP_PRESENT, &prop);
	if (ret) {
		dev_err(&charger->client->dev,
			"Error getting charging status: %d\n", ret);
		goto out;
	}

	if (charger->online != prop.intval) {
		charger->online = prop.intval;
		dev_info(&charger->client->dev,
			 "dc status is %d, trigger wc changed\n", prop.intval);
		power_supply_changed(charger->wc_psy);
	}

	pm_stay_awake(charger->dev);

	if (!schedule_delayed_work(&charger->notifier_work,
				   msecs_to_jiffies(100)))
		pm_relax(charger->dev);

out:
	return NOTIFY_OK;
}

/*
 * Enable interrupts on the P9221, note we don't really need to disable
 * interrupts since when the device goes out of field, the P9221 is reset.
 */
static int p9221_enable_interrupts(struct p9221_charger_data *charger)
{
	u16 mask = 0;
	int ret;

	if (charger->cust_id == P9221R5_CUSTOMER_ID_VAL)
		mask = P9221R5_STAT_LIMIT_MASK | P9221R5_STAT_CC_MASK;
	else
		mask = P9221_STAT_LIMIT_MASK;

	ret = p9221_reg_write_8(charger, P9221_INT_ENABLE_REG, mask);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not enable interrupts: %d\n", ret);
	return ret;
}

static void p9221_notifier_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, notifier_work.work);
	int ret;
	u8 cid = 0;

	if (!charger->online)
		goto out;

	ret = p9221_reg_read_8(charger, P9221_CUSTOMER_ID_REG, &cid);
	if (ret)
		dev_err(&charger->client->dev, "Could not get ID: %d\n", ret);
	else
		charger->cust_id = cid;

	ret = p9221_enable_interrupts(charger);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not enable interrupts: %d\n", ret);

	if (charger->pdata->fod_set)
		p9221_write_fod(charger);

out:
	pm_relax(charger->dev);
}

static size_t p9221_hex_str(u8 *data, size_t len, char *buf, size_t max_buf,
			    bool msbfirst)
{
	int i;
	int blen = 0;
	u8 val;

	for (i = 0; i < len; i++) {
		if (msbfirst)
			val = data[len - 1 - i];
		else
			val = data[i];
		blen += scnprintf(buf + (i * 3), max_buf - (i * 3),
				  "%02x ", val);
	}
	return blen;
}

static ssize_t p9221_add_reg_buffer(struct p9221_charger_data *charger,
				    char *buf, size_t count, u16 reg, int width,
				    bool cooked, const char *name, char *fmt)
{
	u32 val;
	int ret;

	if (cooked)
		ret = p9221_reg_read_cooked(charger, reg, &val);
	else if (width == 16) {
		u16 val16 = 0;
		ret = p9221_reg_read_16(charger, reg, &val16);
		val = val16;
	} else {
		u8 val8 = 0;
		ret = p9221_reg_read_8(charger, reg, &val8);
		val = val8;
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "%s", name);
	if (ret)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "err %d\n", ret);
	else
		count += scnprintf(buf + count, PAGE_SIZE - count, fmt, val);

	return count;
}

static ssize_t p9221_show_version(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int count = 0;
	int i;
	int ret;
	u8 val8 = 0;

	if (!charger->online)
		return -ENODEV;

	count = p9221_add_reg_buffer(charger, buf, count, P9221_CHIP_ID_REG, 16,
				     0, "chip id    : ", "%04x\n");
	count = p9221_add_reg_buffer(charger, buf, count,
				     P9221_CHIP_REVISION_REG, 8, 0,
				     "chip rev   : ", "%02x\n");
	count = p9221_add_reg_buffer(charger, buf, count,
				     P9221_CUSTOMER_ID_REG, 8, 0,
				     "cust id    : ", "%02x\n");
	count = p9221_add_reg_buffer(charger, buf, count,
				     P9221_OTP_FW_MAJOR_REV_REG, 16, 0,
				     "otp fw maj : ", "%04x\n");
	count = p9221_add_reg_buffer(charger, buf, count,
				     P9221_OTP_FW_MINOR_REV_REG, 16, 0,
				     "otp fw min : ", "%04x\n");

	count += scnprintf(buf + count, PAGE_SIZE - count, "otp fw date: ");
	for (i = 0; i < 12; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_OTP_FW_DATE_REG + i, &val8);
		count += scnprintf(buf + count, PAGE_SIZE - count, "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\notp fw time: ");
	for (i = 0; i < 12; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_OTP_FW_TIME_REG + i, &val8);
		count += scnprintf(buf + count, PAGE_SIZE - count, "%c", val8);
	}

	count = p9221_add_reg_buffer(charger, buf, count,
				     P9221_SRAM_FW_MAJOR_REV_REG, 16, 0,
				     "\nram fw maj : ", "%04x\n");
	count = p9221_add_reg_buffer(charger, buf, count,
				     P9221_SRAM_FW_MINOR_REV_REG, 16, 0,
				     "ram fw min : ", "%04x\n");

	count += scnprintf(buf + count, PAGE_SIZE - count, "ram fw date: ");
	for (i = 0; i < 12; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_SRAM_FW_DATE_REG + i, &val8);
		count += scnprintf(buf + count, PAGE_SIZE - count, "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\nram fw time: ");
	for (i = 0; i < 12; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_SRAM_FW_TIME_REG + i, &val8);
		count += scnprintf(buf + count, PAGE_SIZE - count, "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	return count;
}

static DEVICE_ATTR(version, 0444, p9221_show_version, NULL);

static ssize_t p9221_show_status(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int count = 0;

	if (!charger->online)
		return -ENODEV;

	count = p9221_add_reg_buffer(charger, buf, count,
				     P9221_STATUS_REG, 16, 0,
				     "status      : ", "%04x\n");

	count = p9221_add_reg_buffer(charger, buf, count,
				     P9221_INT_REG, 16, 0,
				     "int         : ", "%04x\n");

	count = p9221_add_reg_buffer(charger, buf, count,
				     P9221_INT_ENABLE_REG, 16, 0,
				     "int_enable  : ", "%04x\n");

	if (charger->cust_id == P9221R5_CUSTOMER_ID_VAL) {
		count = p9221_add_reg_buffer(charger, buf, count,
					     P9221R5_VOUT_REG, 16, 1,
					     "vout        : ", "%d uV\n");

		count = p9221_add_reg_buffer(charger, buf, count,
					     P9221R5_VRECT_REG, 16, 1,
					     "vrect       : ", "%d uV\n");

		count = p9221_add_reg_buffer(charger, buf, count,
					     P9221R5_IOUT_REG, 16, 1,
					     "iout        : ", "%d uA\n");

		count = p9221_add_reg_buffer(charger, buf, count,
					     P9221R5_OP_FREQ_REG, 16, 1,
					     "freq        : ", "%d hz\n");

	} else {
		count = p9221_add_reg_buffer(charger, buf, count,
					     P9221_VOUT_ADC_REG, 16, 1,
					     "vout        : ", "%d uV\n");

		count = p9221_add_reg_buffer(charger, buf, count,
					     P9221_VRECT_ADC_REG, 16, 1,
					     "vrect       : ", "%d uV\n");

		count = p9221_add_reg_buffer(charger, buf, count,
					     P9221_RX_IOUT_REG, 16, 1,
					     "iout        : ", "%d uA\n");

		count = p9221_add_reg_buffer(charger, buf, count,
					     P9221_OP_FREQ_REG, 16, 1,
					     "freq        : ", "%d hz\n");
	}

	return count;
}

static DEVICE_ATTR(status, 0444, p9221_show_status, NULL);

static ssize_t p9221_show_count(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%u\n", charger->count);
}

static ssize_t p9221_store_count(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u8 cnt;

	ret = kstrtou8(buf, 0, &cnt);
	if (ret < 0)
		return ret;
	charger->count = cnt;
	return count;
}

static DEVICE_ATTR(count, 0644, p9221_show_count, p9221_store_count);

static ssize_t p9221_show_addr(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%04x\n", charger->addr);
}

static ssize_t p9221_store_addr(struct device *dev,
			        struct device_attribute *attr,
			        const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u16 addr;

	ret = kstrtou16(buf, 16, &addr);
	if (ret < 0)
		return ret;
	charger->addr = addr;
	return count;
}

static DEVICE_ATTR(addr, 0644, p9221_show_addr, p9221_store_addr);

static ssize_t p9221_show_data(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	u8 reg[256];
	int ret;
	int i;
	ssize_t len = 0;

	if (!charger->count || (charger->addr > (0xFFFF - charger->count)))
		return -EINVAL;

	if (!charger->online)
		return -ENODEV;

	ret = p9221_reg_read_n(charger, charger->addr, reg, charger->count);
	if (ret)
		return ret;

	for (i = 0; i < charger->count; i++) {
		len += snprintf(buf + (i * 7), PAGE_SIZE - (i * 7),
				"%02x: %02x\n", charger->addr + i, reg[i]);
	}
	return len;
}

static ssize_t p9221_store_data(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	u8 reg[256];
	int i = 0;
	int ret = 0;
	char *data;

	if (!charger->count || (charger->addr > (0xFFFF - charger->count)))
		return -EINVAL;

	if (!charger->online)
		return -ENODEV;

	data = kmalloc(strlen(buf) + 1, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	strcpy(data, buf);
	while(data && i < charger->count) {
		char *d = strsep(&data, " ");
		if (*d) {
			ret = kstrtou8(d, 16, &reg[i]);
			if (ret)
				break;
			i++;
		}
	}
	if ((i != charger->count) || ret) {
		ret = -EINVAL;
		goto out;
	}

	ret = p9221_reg_write_n(charger, charger->addr, reg, charger->count);
	if (ret)
		goto out;
	ret = count;

out:
	kfree(data);
	return ret;
}

static DEVICE_ATTR(data, 0644, p9221_show_data, p9221_store_data);

static struct attribute *p9221_attributes[] = {
	&dev_attr_version.attr,
	&dev_attr_status.attr,
	&dev_attr_addr.attr,
	&dev_attr_count.attr,
	&dev_attr_data.attr,
	NULL
};

static const struct attribute_group p9221_attr_group = {
	.attrs = p9221_attributes,
};

static int p9221_set_cmd_reg(struct p9221_charger_data *charger, u8 cmd)
{
	u8 cur_cmd = 0;
	int retry;
	int ret;

	for (retry = 0; retry < 5; retry++) {
		ret = p9221_reg_write_8(charger, P9221_COM_REG, cmd);
		if (ret == 0)
			break;

		msleep(100);

		ret = p9221_reg_read_8(charger, P9221_COM_REG, &cur_cmd);
		if (ret == 0 && cur_cmd == 0)
			break;

		dev_info(&charger->client->dev,
			 "Failed to set cmd reg %02x: %d\n", cur_cmd, ret);

		msleep(100);
	}
	if (retry == 5) {
		dev_err(&charger->client->dev,
			"Failed to set cmd reg %02x: %d\n", cur_cmd, ret);
		return -EBUSY;
	}

	return 0;
}

static int p9221_send_eop(struct p9221_charger_data *charger, u8 reason)
{
	int ret;
	u8 cmd;

	if (reason > 8)
		return -EINVAL;

	dev_info(&charger->client->dev, "Send EOP reason=%d\n", reason);

	if (charger->cust_id == P9221R5_CUSTOMER_ID_VAL) {
		ret = p9221_reg_write_8(charger, P9221R5_EPT_REG, reason);
		cmd = P9221R5_COM_SENDEPT;
	} else {
		ret = p9221_reg_write_8(charger, P9221_EPT_REG, reason);
		cmd = P9221_COM_SEND_EOP_MASK;
	}
	if (ret)
		return ret;

	return p9221_set_cmd_reg(charger, cmd);
}

static irqreturn_t p9221_irq_thread(int irq, void *irq_data)
{
	struct p9221_charger_data *charger = irq_data;

	int ret = IRQ_HANDLED;
	u8 irq_src;
	u8 reason = 0;

	if (!charger->online)
		goto done;

	ret = p9221_reg_read_8(charger, P9221_INT_REG, &irq_src);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to read INT reg: %d\n", ret);
		ret = IRQ_NONE;
		goto done;
	}

	ret = p9221_reg_write_8(charger, P9221_INT_CLEAR_REG, irq_src);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to clear INT reg: %d\n", ret);
		ret = IRQ_NONE;
		goto done;
	}

	ret = p9221_reg_write_8(charger, P9221_COM_REG,
				P9221_COM_CLEAR_INT_MASK);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to reset INT: %d\n", ret);
		ret = IRQ_NONE;
		goto done;
	}

	if (irq_src & P9221_STAT_LIMIT_MASK) {
		dev_err(&charger->client->dev, "Received OVER INT: %02x\n",
			irq_src);
		if (irq_src & P9221_STAT_OV_TEMP)
			reason = P9221_EOP_OVER_TEMP;
		else if (irq_src & P9221_STAT_OV_VOLT)
			reason = P9221_EOP_OVER_VOLT;
		else
			reason = P9221_EOP_OVER_CURRENT;

		ret = p9221_send_eop(charger, reason);
		if (ret) {
			dev_err(&charger->client->dev,
				"Failed to send EOP %d: %d\n", reason, ret);
		}
		ret = IRQ_HANDLED;
	}

done:
	return ret;
}

static int p9221_parse_dt(struct device *dev,
			  struct p9221_charger_platform_data *pdata)
{
	int ret = 0;
	u32 data;
	struct device_node *node = dev->of_node;
	enum of_gpio_flags irq_gpio_flags;

	ret = of_get_named_gpio_flags(node, "idt,irq_gpio", 0, &irq_gpio_flags);
	if (ret < 0) {
		dev_err(dev, "unable to read idt,irq_gpio from dt: %d\n", ret);
		return ret;
	}
	pdata->irq_gpio = ret;
	pdata->irq_int = gpio_to_irq(pdata->irq_gpio);
	dev_info(dev, "gpio:%d, gpio_irq:%d\n", pdata->irq_gpio,
		 pdata->irq_int);

	/* Optional VOUT max */
	pdata->max_vout_mv = P9221_MAX_VOUT_SET_MV_DEFAULT;
	ret = of_property_read_u32(node, "max_vout_mv", &data);
	if (ret == 0) {
		if (data < 3500 || data > 29000)
			dev_err(dev, "max_vout_mv out of range %d\n", data);
		else
			pdata->max_vout_mv = data;
	}

	/* Optional FOD data */
	pdata->fod_set = false;
	ret = of_property_read_u8_array(node, "fod", pdata->fod, P9221_NUM_FOD);
	if (ret == 0) {
		char buf[P9221_NUM_FOD * 3 + 1];

		pdata->fod_set = true;
		p9221_hex_str(pdata->fod, P9221_NUM_FOD, buf,
			      P9221_NUM_FOD * 3 + 1, false);
		dev_info(dev, "dt fod: %s\n", buf);
	}

	return 0;
}

static enum power_supply_property p9221_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc p9221_psy_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = p9221_props,
	.num_properties = ARRAY_SIZE(p9221_props),
	.get_property = p9221_get_property,
	.set_property = p9221_set_property,
	.property_is_writeable = p9221_prop_is_writeable,
};

static int p9221_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device_node *of_node = client->dev.of_node;
	struct p9221_charger_data *charger;
	struct p9221_charger_platform_data *pdata = client->dev.platform_data;
	struct power_supply_config psy_cfg = {};
	int ret = 0;
	u16 chip_id;

	ret = i2c_check_functionality(client->adapter,
				      I2C_FUNC_SMBUS_BYTE_DATA |
				      I2C_FUNC_SMBUS_WORD_DATA |
				      I2C_FUNC_SMBUS_I2C_BLOCK);
	if (!ret) {
		ret = i2c_get_functionality(client->adapter);
		dev_err(&client->dev, "I2C adapter not compatible %x\n", ret);
		return -ENOSYS;
	}

	if (of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate pdata\n");
			return -ENOMEM;
		}
		ret = p9221_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Failed to parse dt\n");
			return ret;
		}
	}

	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (charger == NULL) {
		dev_err(&client->dev, "Failed to allocate charger\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, charger);
	charger->dev = &client->dev;
	charger->client = client;
	charger->pdata = pdata;
	mutex_init(&charger->io_lock);

	psy_cfg.drv_data = charger;
	psy_cfg.of_node = charger->dev->of_node;
	charger->wc_psy = devm_power_supply_register(charger->dev,
						     &p9221_psy_desc,
						     &psy_cfg);
	if (IS_ERR(charger->wc_psy)) {
		dev_err(&client->dev, "Fail to register supply: %d\n", ret);
		return PTR_ERR(charger->wc_psy);
	}

	ret = devm_request_threaded_irq(
		&client->dev, charger->pdata->irq_int, NULL,
		p9221_irq_thread, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"p9221-irq", charger);
	if (ret) {
		dev_err(&client->dev, "Failed to request IRQ\n");
		return ret;
	}
	device_init_wakeup(charger->dev, true);
	enable_irq_wake(charger->pdata->irq_int);

	charger->count = 1;
	ret = sysfs_create_group(&charger->dev->kobj, &p9221_attr_group);
	if (ret) {
		dev_info(&client->dev, "sysfs_create_group failed\n");
	}

	ret = p9221_reg_read_16(charger, P9221_CHIP_ID_REG, &chip_id);
	if (ret == 0) {
		u8 cid = 0;

		charger->online = 1;

		ret = p9221_reg_read_8(charger, P9221_CUSTOMER_ID_REG, &cid);
		if (ret)
			dev_err(&charger->client->dev,
				"Could not get ID: %d\n", ret);
		else
			charger->cust_id = cid;

		dev_info(&client->dev, "P9221 online, id: %04x cid: %02x\n",
			 chip_id, cid);
	}

	/*
	 * Register notifier so we can detect changes on DC_IN
	 */
	INIT_DELAYED_WORK(&charger->notifier_work, p9221_notifier_work);
	charger->nb.notifier_call = p9221_notifier_cb;
	ret = power_supply_reg_notifier(&charger->nb);
	if (ret) {
		dev_err(&client->dev, "Fail to register notifier: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "P9221 Charger Driver Loaded\n");

	return 0;
}

static int p9221_charger_remove(struct i2c_client *client)
{
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	device_init_wakeup(charger->dev, false);
	cancel_delayed_work_sync(&charger->notifier_work);
	power_supply_unreg_notifier(&charger->nb);
	mutex_destroy(&charger->io_lock);
	return 0;
}

static const struct i2c_device_id p9221_charger_id_table[] = {
	{ "p9221", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, p9221_charger_id_table);

#ifdef CONFIG_OF
static struct of_device_id p9221_charger_match_table[] = {
	{ .compatible = "idt,p9221",},
	{},
};
#else
#define p9221_charger_match_table NULL
#endif

static struct i2c_driver p9221_charger_driver = {
	.driver = {
		.name		= "p9221",
		.owner		= THIS_MODULE,
		.of_match_table = p9221_charger_match_table,
	},
	.probe		= p9221_charger_probe,
	.remove		= p9221_charger_remove,
	.id_table	= p9221_charger_id_table,
};

static int __init p9221_charger_init(void)
{
	return i2c_add_driver(&p9221_charger_driver);
}

static void __exit p9221_charger_exit(void)
{
	i2c_del_driver(&p9221_charger_driver);
}

module_init(p9221_charger_init);
module_exit(p9221_charger_exit);

MODULE_DESCRIPTION("IDT P9221 Wireless Power Receiver Driver");
MODULE_AUTHOR("Patrick Tjin <pattjin@google.com>");
MODULE_LICENSE("GPL");
