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
#include <linux/pmic-voter.h>
#include <linux/alarmtimer.h>
#include "p9221_charger.h"

#define P9221_TX_TIMEOUT_MS		(20 * 1000)
#define P9221_DCIN_TIMEOUT_MS		(2 * 1000)
#define P9221_VRECT_TIMEOUT_MS		(2 * 1000)
#define P9221_NOTIFIER_DELAY_MS		80
#define P9221R5_ILIM_MAX_UA		(1600 * 1000)
#define P9221R5_OVER_CHECK_NUM		3

#define OVC_LIMIT			1
#define OVC_THRESHOLD			1400000
#define OVC_BACKOFF_LIMIT		900000
#define OVC_BACKOFF_AMOUNT		100000

static void p9221_icl_ramp_reset(struct p9221_charger_data *charger);
static void p9221_icl_ramp_start(struct p9221_charger_data *charger);

static const u32 p9221_ov_set_lut[] = {
	17000000, 20000000, 15000000, 13000000,
	11000000, 11000000, 11000000, 11000000};

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
	if (ret == 0)
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
	switch (reg) {
	case P9221_CHIP_REVISION_REG:
	case P9221_CUSTOMER_ID_REG:
	case P9221_COM_REG:
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

/*
 * Cook the value according to the register (R5+)
 */
static int p9221_cook_reg(struct p9221_charger_data *charger, u16 reg,
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
 * Read the reg and return the cooked value.
 */
static int p9221_reg_read_cooked(struct p9221_charger_data *charger,
				 u16 reg, u32 *val)
{
	int ret = 0;
	u16 data = 0;

	if (p9221_reg_is_8_bit(charger, reg)) {
		u8 data8 = 0;
		ret = p9221_reg_read_8(charger, reg, &data8);
		data = data8;
	} else {
		ret = p9221_reg_read_16(charger, reg, &data);
	}
	if (ret)
		return ret;

	return p9221_cook_reg(charger, reg, data, val);
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
static int p9221_reg_write_cooked(struct p9221_charger_data *charger,
				  u16 reg, u32 val)
{
	int ret = 0;
	u16 data;
	int i;

	/* Do the appropriate conversion */
	switch (reg) {
	case P9221R5_ILIM_SET_REG:
		/* uA -> 0.1A, offset 0.2A */
		if ((val < 200000) || (val > 1600000))
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

static bool p9221_is_epp(struct p9221_charger_data *charger)
{
	int ret;
	u32 vout_uv;
	uint8_t reg;

	ret = p9221_reg_read_8(charger, P9221R5_SYSTEM_MODE_REG, &reg);
	if (ret == 0)
		return (reg & P9221R5_SYSTEM_MODE_EXTENDED_MASK) > 0;

	dev_err(&charger->client->dev, "Could not read mode: %d\n",
		ret);

	/* Check based on power supply voltage */
	ret = p9221_reg_read_cooked(charger, P9221R5_VOUT_ADC_REG, &vout_uv);
	if (ret) {
		dev_err(&charger->client->dev, "Could read VOUT_ADC, %d\n",
			ret);
		goto out;
	}

	dev_info(&charger->client->dev, "Voltage is %duV\n", vout_uv);
	if (vout_uv > P9221_EPP_THRESHOLD_UV)
		return true;

out:
	/* Default to BPP otherwise */
	return false;
}

static void p9221_write_fod(struct p9221_charger_data *charger)
{
	bool epp = false;
	u8 *fod = NULL;
	int fod_count = charger->pdata->fod_num;
	int ret;
	int retries = 3;

	if (!charger->pdata->fod_num && !charger->pdata->fod_epp_num)
		goto no_fod;

	/* Default to BPP FOD */
	if (charger->pdata->fod_num)
		fod = charger->pdata->fod;

	if (p9221_is_epp(charger) && charger->pdata->fod_epp_num) {
		fod = charger->pdata->fod_epp;
		fod_count = charger->pdata->fod_epp_num;
		epp = true;
	}

	if (!fod)
		goto no_fod;

	while (retries) {
		char s[fod_count * 3 + 1];
		u8 fod_read[fod_count];

		dev_info(&charger->client->dev, "Writing %s FOD (n=%d reg=%02x try=%d)\n",
			 epp ? "EPP" : "BPP", fod_count, P9221R5_FOD_REG,
			 retries);

		ret = p9221_reg_write_n(charger, P9221R5_FOD_REG, fod,
					fod_count);
		if (ret) {
			dev_err(&charger->client->dev,
				"Could not write FOD: %d\n", ret);
			return;
		}

		/* Verify the FOD has been written properly */
		ret = p9221_reg_read_n(charger, P9221R5_FOD_REG, fod_read,
				       fod_count);
		if (ret) {
			dev_err(&charger->client->dev,
				"Could not read back FOD: %d\n", ret);
			return;
		}

		if (memcmp(fod, fod_read, fod_count) == 0)
			return;

		p9221_hex_str(fod_read, fod_count, s, sizeof(s), 0);
		dev_err(&charger->client->dev,
			"FOD verify error, read: %s\n", s);

		retries--;
		msleep(100);
	}

no_fod:
	dev_warn(&charger->client->dev, "FOD not set! bpp:%d epp:%d r:%d\n",
		 charger->pdata->fod_num, charger->pdata->fod_epp_num, retries);
}

static int p9221_set_cmd_reg(struct p9221_charger_data *charger, u8 cmd)
{
	u8 cur_cmd = 0;
	int retry;
	int ret;

	for (retry = 0; retry < P9221_COM_CHAN_RETRIES; retry++) {
		ret = p9221_reg_read_8(charger, P9221_COM_REG, &cur_cmd);
		if (ret == 0 && cur_cmd == 0)
			break;
		msleep(25);
	}

	if (retry >= P9221_COM_CHAN_RETRIES) {
		dev_err(&charger->client->dev,
			"Failed to wait for cmd free %02x\n", cur_cmd);
		return -EBUSY;
	}

	ret = p9221_reg_write_8(charger, P9221_COM_REG, cmd);
	if (ret)
		dev_err(&charger->client->dev,
			"Failed to set cmd reg %02x: %d\n", cmd, ret);

	return ret;
}

static int p9221_send_data(struct p9221_charger_data *charger)
{
	int ret;

	if (charger->tx_busy)
		return -EBUSY;

	if (!charger->tx_len || charger->tx_len > P9221R5_DATA_SEND_BUF_SIZE)
		return -EINVAL;

	charger->tx_busy = true;

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_n(charger, P9221R5_DATA_SEND_BUF_START,
				charger->tx_buf, charger->tx_len);
	if (ret) {
		dev_err(&charger->client->dev, "Failed to load tx %d\n", ret);
		goto error;
	}

	ret = p9221_reg_write_8(charger, P9221R5_COM_CHAN_SEND_SIZE_REG,
				charger->tx_len);
	if (ret) {
		dev_err(&charger->client->dev, "Failed to load txsz %d\n", ret);
		goto error;
	}

	ret = p9221_set_cmd_reg(charger, P9221R5_COM_CCACTIVATE);
	if (ret)
		goto error;

	mutex_unlock(&charger->cmd_lock);
	return ret;

error:
	mutex_unlock(&charger->cmd_lock);
	charger->tx_busy = false;
	return ret;
}

static int p9221_send_csp(struct p9221_charger_data *charger, u8 stat)
{
	int ret;

	dev_info(&charger->client->dev, "Send CSP status=%d\n", stat);

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_8(charger, P9221R5_CHARGE_STAT_REG, stat);
	if (ret == 0)
		ret = p9221_set_cmd_reg(charger, P9221R5_COM_SENDCSP);

	mutex_unlock(&charger->cmd_lock);
	return ret;
}

static int p9221_send_eop(struct p9221_charger_data *charger, u8 reason)
{
	int ret;

	dev_info(&charger->client->dev, "Send EOP reason=%d\n", reason);

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_8(charger, P9221R5_EPT_REG, reason);
	if (ret == 0)
		ret = p9221_set_cmd_reg(charger, P9221R5_COM_SENDEPT);

	mutex_unlock(&charger->cmd_lock);
	return ret;
}

static int p9221_send_ccreset(struct p9221_charger_data *charger)
{
	int ret;

	dev_info(&charger->client->dev, "Send CC reset\n");

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_8(charger, P9221R5_COM_CHAN_RESET_REG,
				P9221R5_COM_CHAN_CCRESET);
	if (ret == 0)
		ret = p9221_set_cmd_reg(charger, P9221R5_COM_CCACTIVATE);

	mutex_unlock(&charger->cmd_lock);
	return ret;
}

struct p9221_prop_reg_map_entry p9221_prop_reg_map[] = {
	/* property			register			g, s */
	{POWER_SUPPLY_PROP_CURRENT_NOW,	P9221R5_IOUT_REG,		1, 0},
	{POWER_SUPPLY_PROP_VOLTAGE_NOW,	P9221R5_VOUT_REG,		1, 0},
	{POWER_SUPPLY_PROP_VOLTAGE_MAX, P9221R5_VOUT_SET_REG,		1, 1},
	{POWER_SUPPLY_PROP_TEMP,	P9221R5_DIE_TEMP_ADC_REG,	1, 0},
	{POWER_SUPPLY_PROP_CAPACITY,	0,				1, 1},
	{POWER_SUPPLY_PROP_ONLINE,	0,				1, 1},
};

static struct p9221_prop_reg_map_entry *p9221_get_map_entry(
				struct p9221_charger_data *charger,
				enum power_supply_property prop, bool set)
{
	int i;
	struct p9221_prop_reg_map_entry *p;
	int map_size;

	p = p9221_prop_reg_map;
	map_size = ARRAY_SIZE(p9221_prop_reg_map);

	for (i = 0; i < map_size; i++) {
		if (p->prop == prop) {
			if ((set && p->set) || (!set && p->get))
				return p;
		}
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

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(charger->dev);

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

static void p9221_abort_transfers(struct p9221_charger_data *charger)
{
	/* Abort all transfers */
	cancel_delayed_work(&charger->tx_work);
	charger->tx_busy = false;
	charger->tx_done = true;
	charger->rx_done = true;
	charger->rx_len = 0;
	sysfs_notify(&charger->dev->kobj, NULL, "txbusy");
	sysfs_notify(&charger->dev->kobj, NULL, "txdone");
	sysfs_notify(&charger->dev->kobj, NULL, "rxdone");
}

/*
 * Put the default ICL back to BPP, reset OCP voter
 * @pre charger && charger->dc_icl_votable && charger->client->dev
 */
static void p9221_vote_defaults(struct p9221_charger_data *charger)
{
	int ret;

	if (!charger->dc_icl_votable) {
		dev_err(&charger->client->dev,
			"Could not vote DC_ICL - no votable\n");
		return;
	}

	ret = vote(charger->dc_icl_votable, P9221_WLC_VOTER, true,
			P9221_DC_ICL_BPP_UA);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not vote DC_ICL %d\n", ret);

	ret = vote(charger->dc_icl_votable, P9221_OCP_VOTER, true,
			P9221_DC_ICL_EPP_UA);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not reset OCP DC_ICL voter %d\n", ret);
}

static void p9221_set_offline(struct p9221_charger_data *charger)
{
	dev_info(&charger->client->dev, "Set offline\n");

	charger->online = false;

	/* Reset PP buf so we can get a new serial number next time around */
	charger->pp_buf_valid = false;

	p9221_abort_transfers(charger);
	cancel_delayed_work(&charger->dcin_work);
	p9221_icl_ramp_reset(charger);
	del_timer(&charger->vrect_timer);

	p9221_vote_defaults(charger);
}

static void p9221_tx_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, tx_work.work);

	dev_info(&charger->client->dev, "timeout waiting for tx complete\n");

	charger->tx_busy = false;
	charger->tx_done = true;
	sysfs_notify(&charger->dev->kobj, NULL, "txbusy");
	sysfs_notify(&charger->dev->kobj, NULL, "txdone");
}

static void p9221_vrect_timer_handler(unsigned long data)
{
	struct p9221_charger_data *charger = (struct p9221_charger_data *)data;

	dev_info(&charger->client->dev,
		 "timeout waiting for VRECT, online=%d\n", charger->online);
	pm_relax(charger->dev);
}

static void p9221_dcin_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, dcin_work.work);

	dev_info(&charger->client->dev,
		 "timeout waiting for dc-in, online=%d\n", charger->online);

	if (charger->online)
		p9221_set_offline(charger);

	power_supply_changed(charger->wc_psy);

	pm_relax(charger->dev);
}

static const char *p9221_get_tx_id_str(struct p9221_charger_data *charger)
{
	int ret;
	uint32_t tx_id = 0;

	if (!charger->online)
		return NULL;

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		return NULL;
	}
	pm_runtime_put_sync(charger->dev);

	if (p9221_is_epp(charger)) {
		ret = p9221_reg_read_n(charger, P9221R5_PROP_TX_ID_REG,
				       &tx_id, sizeof(tx_id));
		if (ret)
			dev_err(&charger->client->dev,
				"Failed to read txid %d\n", ret);
	} else {
		/*
		 * If pp_buf_valid is true, we have received a serial
		 * number from the Tx, copy it to tx_id. (pp_buf_valid
		 * is left true here until we go offline as we may
		 * read this multiple times.)
		 */
		if (charger->pp_buf_valid &&
		    sizeof(tx_id) <= P9221R5_MAX_PP_BUF_SIZE)
			memcpy(&tx_id, &charger->pp_buf[1],
			       sizeof(tx_id));
	}
	scnprintf(charger->tx_id_str, sizeof(charger->tx_id_str),
		  "%08x", tx_id);
	return charger->tx_id_str;
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
		val->intval = charger->online && charger->enabled;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = p9221_get_tx_id_str(charger);
		if (val->strval == NULL)
			return -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (charger->last_capacity > 0)
			val->intval = charger->last_capacity;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (!charger->dc_icl_votable)
			return -EAGAIN;

		ret = get_effective_result(charger->dc_icl_votable);
		if (ret < 0)
			break;

		val->intval = ret;

		/* success */
		ret = 0;
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
	int ret = 0;
	bool changed = false;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		if ((val->intval < 0) || (val->intval > 1)) {
			ret = -EINVAL;
			break;
		}

		if (charger->enabled == val->intval)
			break;

		/*
		 * Asserting the enable line will automatically take bring
		 * us online if we are in field.  De-asserting the enable
		 * line will automatically take us offline if we are in field.
		 * This is due to the fact that DC in will change state
		 * appropriately when we change the state of this line.
		 */
		charger->enabled = val->intval;

		dev_warn(&charger->client->dev, "Set enable %d\n",
			 charger->enabled);

		if (charger->pdata->qien_gpio >= 0)
			gpio_set_value(charger->pdata->qien_gpio,
				       charger->enabled ? 0 : 1);

		changed = true;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (!charger->online || charger->last_capacity == val->intval)
			break;

		charger->last_capacity = val->intval;
		ret = p9221_send_csp(charger, charger->last_capacity);
		if (ret)
			dev_err(&charger->client->dev,
				"Could send csp: %d\n", ret);
		changed = true;

		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (val->intval < 0) {
			ret = -EINVAL;
			break;
		}

		if (!charger->dc_icl_votable) {
			ret = -EAGAIN;
			break;
		}

		ret = vote(charger->dc_icl_votable, P9221_USER_VOTER, true,
			   val->intval);

		changed = true;
		break;
	default:
		ret = p9221_set_property_reg(charger, prop, val);
		if (ret == 0)
			changed = true;
		break;
	}

	if (ret)
		dev_dbg(&charger->client->dev,
			"Couldn't set prop %d, ret=%d\n", prop, ret);

	if (changed)
		power_supply_changed(psy);

	return ret;
}

static int p9221_prop_is_writeable(struct power_supply *psy,
				   enum power_supply_property prop)
{
	struct p9221_charger_data *charger = power_supply_get_drvdata(psy);

	if (p9221_get_map_entry(charger, prop, true))
		return 1;

	return 0;
}

static int p9221_notifier_cb(struct notifier_block *nb, unsigned long event,
			     void *data)
{
	struct power_supply *psy = data;
	struct p9221_charger_data *charger =
		container_of(nb, struct p9221_charger_data, nb);

	if (event != PSY_EVENT_PROP_CHANGED)
		goto out;

	if (strcmp(psy->desc->name, "dc") == 0) {
		charger->dc_psy = psy;
		charger->check_dc = true;
	}

	if (!charger->check_dc)
		goto out;

	pm_stay_awake(charger->dev);

	if (!schedule_delayed_work(&charger->notifier_work,
				   msecs_to_jiffies(P9221_NOTIFIER_DELAY_MS)))
		pm_relax(charger->dev);

out:
	return NOTIFY_OK;
}

static int p9221_clear_interrupts(struct p9221_charger_data *charger, u16 mask)
{
	int ret;

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_16(charger, P9221R5_INT_CLEAR_REG, mask);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to clear INT reg: %d\n", ret);
		goto out;
	}

	ret = p9221_set_cmd_reg(charger, P9221_COM_CLEAR_INT_MASK);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to reset INT: %d\n", ret);
	}
out:
	mutex_unlock(&charger->cmd_lock);
	return ret;
}

/*
 * Enable interrupts on the P9221, note we don't really need to disable
 * interrupts since when the device goes out of field, the P9221 is reset.
 */
static int p9221_enable_interrupts(struct p9221_charger_data *charger)
{
	u16 mask;
	int ret;

	dev_dbg(&charger->client->dev, "Enable interrupts\n");

	mask = P9221R5_STAT_LIMIT_MASK | P9221R5_STAT_CC_MASK |
	       P9221_STAT_VRECT;

	ret = p9221_clear_interrupts(charger, mask);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not clear interrupts: %d\n", ret);

	ret = p9221_reg_write_8(charger, P9221_INT_ENABLE_REG, mask);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not enable interrupts: %d\n", ret);
	return ret;
}

static int p9221_set_dc_icl(struct p9221_charger_data *charger)
{
	int icl;
	int ret;

	if (!charger->dc_icl_votable) {
		charger->dc_icl_votable = find_votable("DC_ICL");
		if (!charger->dc_icl_votable) {
			dev_err(&charger->client->dev,
				"Could not get votable: DC_ICL\n");
			return -ENODEV;
		}
	}

	/* Default to BPP ICL */
	icl = P9221_DC_ICL_BPP_UA;

	if (charger->icl_ramp)
		icl = charger->icl_ramp_ua;

	if (p9221_is_epp(charger))
		icl = P9221_DC_ICL_EPP_UA;

	dev_info(&charger->client->dev, "Setting ICL %duA ramp=%d\n", icl,
		 charger->icl_ramp);
	ret = vote(charger->dc_icl_votable, P9221_WLC_VOTER, true, icl);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not vote DC_ICL %d\n", ret);

	/* Increase the IOUT limit */
	ret = p9221_reg_write_cooked(charger, P9221R5_ILIM_SET_REG,
				     P9221R5_ILIM_MAX_UA);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not set rx_iout limit reg: %d\n", ret);

	return ret;
}

static enum alarmtimer_restart p9221_icl_ramp_alarm_cb(struct alarm *alarm,
						       ktime_t now)
{
	struct p9221_charger_data *charger =
			container_of(alarm, struct p9221_charger_data,
				     icl_ramp_alarm);

	dev_info(&charger->client->dev, "ICL ramp alarm, ramp=%d\n",
		 charger->icl_ramp);

	/* Alarm is in atomic context, schedule work to complete the task */
	pm_stay_awake(charger->dev);
	schedule_delayed_work(&charger->icl_ramp_work, msecs_to_jiffies(100));

	return ALARMTIMER_NORESTART;
}

static void p9221_icl_ramp_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, icl_ramp_work.work);

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		schedule_delayed_work(&charger->icl_ramp_work,
				      msecs_to_jiffies(100));
		dev_dbg(&charger->client->dev, "Ramp reschedule\n");
		return;
	}
	pm_runtime_put_sync(charger->dev);

	dev_info(&charger->client->dev, "ICL ramp work, ramp=%d\n",
		 charger->icl_ramp);

	charger->icl_ramp = true;
	p9221_set_dc_icl(charger);

	pm_relax(charger->dev);
}

static void p9221_icl_ramp_reset(struct p9221_charger_data *charger)
{
	dev_info(&charger->client->dev, "ICL ramp reset, ramp=%d\n",
		 charger->icl_ramp);

	charger->icl_ramp = false;

	if (alarm_try_to_cancel(&charger->icl_ramp_alarm) < 0)
		dev_warn(&charger->client->dev, "Couldn't cancel icl_ramp_alarm\n");
	cancel_delayed_work(&charger->icl_ramp_work);
}

static void p9221_icl_ramp_start(struct p9221_charger_data *charger)
{
	/* Only ramp on BPP at this time */
	if (p9221_is_epp(charger))
		return;

	p9221_icl_ramp_reset(charger);

	dev_info(&charger->client->dev, "ICL ramp set alarm %dms, %dua, ramp=%d\n",
		 charger->icl_ramp_delay_ms, charger->icl_ramp_ua,
		 charger->icl_ramp);

	alarm_start_relative(&charger->icl_ramp_alarm,
			     ms_to_ktime(charger->icl_ramp_delay_ms));
}

static void p9221_set_online(struct p9221_charger_data *charger)
{
	int ret;
	u8 cid = 5;

	dev_info(&charger->client->dev, "Set online\n");

	charger->online = true;
	charger->tx_busy = false;
	charger->tx_done = true;
	charger->rx_done = false;
	charger->last_capacity = -1;

	ret = p9221_reg_read_8(charger, P9221_CUSTOMER_ID_REG, &cid);
	if (ret)
		dev_err(&charger->client->dev, "Could not get ID: %d\n", ret);
	else
		charger->cust_id = cid;

	dev_info(&charger->client->dev, "P9221 cid: %02x\n", charger->cust_id);

	ret = p9221_enable_interrupts(charger);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not enable interrupts: %d\n", ret);

	/* NOTE: depends on _is_epp() which is not valid until DC_IN */
	p9221_write_fod(charger);
}

/* 2 * P9221_NOTIFIER_DELAY_MS from VRECTON */
static void p9221_notifier_check_dc(struct p9221_charger_data *charger)
{
	int ret;
	union power_supply_propval prop;

	charger->check_dc = false;

	if (!charger->dc_psy)
		return;

	ret = power_supply_get_property(charger->dc_psy,
					POWER_SUPPLY_PROP_PRESENT, &prop);
	if (ret) {
		dev_err(&charger->client->dev,
			"Error getting charging status: %d\n", ret);
		return;
	}

	dev_info(&charger->client->dev, "dc status is %d\n", prop.intval);

	/*
	 * We now have confirmation from DC_IN, kill the timer, charger->online
	 * will be set by this function.
	 */
	cancel_delayed_work(&charger->dcin_work);
	del_timer(&charger->vrect_timer);

	/*
	 * Always write FOD, check dc_icl, send CSP
	 */
	if (prop.intval) {
		p9221_set_dc_icl(charger);
		p9221_write_fod(charger);
		if (charger->last_capacity > 0)
			p9221_send_csp(charger, charger->last_capacity);
		p9221_icl_ramp_start(charger);
	}

	/* We may have already gone online during check_det */
	if (charger->online == prop.intval)
		goto out;

	if (prop.intval)
		p9221_set_online(charger);
	else
		p9221_set_offline(charger);

out:
	dev_info(&charger->client->dev, "trigger wc changed on:%d in:%d\n",
		 charger->online, prop.intval);
	power_supply_changed(charger->wc_psy);
}

/* P9221_NOTIFIER_DELAY_MS from VRECTON */
bool p9221_notifier_check_det(struct p9221_charger_data *charger)
{
	bool relax = true;

	del_timer(&charger->vrect_timer);

	if (charger->online)
		goto done;

	dev_info(&charger->client->dev, "detected wlc, trigger wc changed\n");
	/* send out a FOD but is_epp() is still invalid */
	p9221_set_online(charger);
	power_supply_changed(charger->wc_psy);

	/* Give the dc-in 2 seconds to come up. */
	dev_info(&charger->client->dev, "start dc-in timer\n");
	cancel_delayed_work_sync(&charger->dcin_work);
	schedule_delayed_work(&charger->dcin_work,
			      msecs_to_jiffies(P9221_DCIN_TIMEOUT_MS));
	relax = false;

done:
	charger->check_det = false;

	return relax;
}

static void p9221_notifier_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, notifier_work.work);
	bool relax = true;

	dev_info(&charger->client->dev, "Notifier work: on:%d dc:%d det:%d\n",
		 charger->online, charger->check_dc, charger->check_det);

	if (charger->check_det)
		relax = p9221_notifier_check_det(charger);

	if (charger->check_dc)
		p9221_notifier_check_dc(charger);

	if (relax)
		pm_relax(charger->dev);
}

static ssize_t p9221_add_reg_buffer(struct p9221_charger_data *charger,
				    char *buf, size_t count, u16 reg, int width,
				    bool cooked, const char *name, char *fmt)
{
	u32 val;
	int ret;
	int added = 0;

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

	added += scnprintf(buf + count, PAGE_SIZE - count, "%s", name);
	count += added;
	if (ret)
		added += scnprintf(buf + count, PAGE_SIZE - count,
				   "err %d\n", ret);
	else
		added += scnprintf(buf + count, PAGE_SIZE - count, fmt, val);

	return added;
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

	count += p9221_add_reg_buffer(charger, buf, count, P9221_CHIP_ID_REG,
				      16, 0, "chip id    : ", "%04x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_CHIP_REVISION_REG, 8, 0,
				      "chip rev   : ", "%02x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_CUSTOMER_ID_REG, 8, 0,
				      "cust id    : ", "%02x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_OTP_FW_MAJOR_REV_REG, 16, 0,
				      "otp fw maj : ", "%04x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_OTP_FW_MINOR_REV_REG, 16, 0,
				      "otp fw min : ", "%04x\n");

	count += scnprintf(buf + count, PAGE_SIZE - count, "otp fw date: ");
	for (i = 0; i < P9221_OTP_FW_DATE_SIZE; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_OTP_FW_DATE_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\notp fw time: ");
	for (i = 0; i < P9221_OTP_FW_TIME_SIZE; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_OTP_FW_TIME_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_SRAM_FW_MAJOR_REV_REG, 16, 0,
				      "\nram fw maj : ", "%04x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_SRAM_FW_MINOR_REV_REG, 16, 0,
				      "ram fw min : ", "%04x\n");

	count += scnprintf(buf + count, PAGE_SIZE - count, "ram fw date: ");
	for (i = 0; i < P9221_SRAM_FW_DATE_SIZE; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_SRAM_FW_DATE_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\nram fw time: ");
	for (i = 0; i < P9221_SRAM_FW_TIME_SIZE; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_SRAM_FW_TIME_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
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
	int ret;
	u8 tmp[P9221R5_NUM_FOD];
	uint32_t tx_id = 0;

	if (!charger->online)
		return -ENODEV;

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_STATUS_REG, 16, 0,
				      "status      : ", "%04x\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_INT_REG, 16, 0,
				      "int         : ", "%04x\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_INT_ENABLE_REG, 16, 0,
				      "int_enable  : ", "%04x\n");


	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_SYSTEM_MODE_REG, 8, 0,
				      "mode        : ", "%02x\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_VOUT_REG, 16, 1,
				      "vout        : ", "%d uV\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_VRECT_REG, 16, 1,
				      "vrect       : ", "%d uV\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_IOUT_REG, 16, 1,
				      "iout        : ", "%d uA\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_ILIM_SET_REG, 16, 1,
				      "ilim        : ", "%d uA\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_OP_FREQ_REG, 16, 1,
				      "freq        : ", "%d hz\n");
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_busy     : %d\n", charger->tx_busy);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_done     : %d\n", charger->tx_done);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "rx_done     : %d\n", charger->rx_done);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_len      : %d\n", charger->tx_len);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "rx_len      : %d\n", charger->rx_len);
	p9221_reg_read_n(charger, P9221R5_PROP_TX_ID_REG, &tx_id,
			 sizeof(tx_id));
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_id       : %08x (%s)\n", tx_id,
			   p9221_get_tx_id_str(charger));

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_ALIGN_X_ADC_REG, 8, 1,
				      "align_x     : ", "%d\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_ALIGN_Y_ADC_REG, 8, 1,
				      "align_y     : ", "%d\n");

	/* FOD Register */
	ret = p9221_reg_read_n(charger, P9221R5_FOD_REG, tmp, P9221R5_NUM_FOD);
	count += scnprintf(buf + count, PAGE_SIZE - count, "fod         : ");
	if (ret)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "err %d\n", ret);
	else {
		count += p9221_hex_str(tmp, P9221R5_NUM_FOD, buf + count, count,
				       false);
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	/* Device tree FOD entries */
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "dt fod      : (n=%d) ", charger->pdata->fod_num);
	count += p9221_hex_str(charger->pdata->fod, charger->pdata->fod_num,
			       buf + count, PAGE_SIZE - count, false);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "\ndt fod-epp  : (n=%d) ",
			   charger->pdata->fod_epp_num);
	count += p9221_hex_str(charger->pdata->fod_epp,
			       charger->pdata->fod_epp_num,
			       buf + count, PAGE_SIZE - count, false);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "\npp buf      : (v=%d) ", charger->pp_buf_valid);
	count += p9221_hex_str(charger->pp_buf, sizeof(charger->pp_buf),
			       buf + count, PAGE_SIZE - count, false);

	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	return count;
}

static DEVICE_ATTR(status, 0444, p9221_show_status, NULL);

static ssize_t p9221_show_count(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%u\n", charger->count);
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

static ssize_t p9221_show_icl_ramp_delay_ms(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->icl_ramp_delay_ms);
}

static ssize_t p9221_store_icl_ramp_delay_ms(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u32 ms;

	ret = kstrtou32(buf, 10, &ms);
	if (ret < 0)
		return ret;
	charger->icl_ramp_delay_ms = ms;
	return count;
}

static DEVICE_ATTR(icl_ramp_delay_ms, 0644,
		   p9221_show_icl_ramp_delay_ms,
		   p9221_store_icl_ramp_delay_ms);

static ssize_t p9221_show_icl_ramp_ua(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->icl_ramp_ua);
}

static ssize_t p9221_store_icl_ramp_ua(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u32 ua;

	ret = kstrtou32(buf, 10, &ua);
	if (ret < 0)
		return ret;
	charger->icl_ramp_ua = ua;
	return count;
}

static DEVICE_ATTR(icl_ramp_ua, 0644,
		   p9221_show_icl_ramp_ua, p9221_store_icl_ramp_ua);

static ssize_t p9221_show_addr(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%04x\n", charger->addr);
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
		len += scnprintf(buf + len, PAGE_SIZE - len, "%02x: %02x\n",
				 charger->addr + i, reg[i]);
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
	char *tmp_buf;

	if (!charger->count || (charger->addr > (0xFFFF - charger->count)))
		return -EINVAL;

	if (!charger->online)
		return -ENODEV;

	tmp_buf = kmalloc(strlen(buf) + 1, GFP_KERNEL);
	data = tmp_buf;
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
	kfree(tmp_buf);
	return ret;
}

static DEVICE_ATTR(data, 0644, p9221_show_data, p9221_store_data);

static ssize_t p9221_store_ccreset(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;

	ret = p9221_send_ccreset(charger);
	if (ret)
		return ret;
	return count;
}

static DEVICE_ATTR(ccreset, 0200, NULL, p9221_store_ccreset);

static ssize_t p9221_show_rxdone(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->rx_done ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static DEVICE_ATTR(rxdone, 0444, p9221_show_rxdone, NULL);

static ssize_t p9221_show_rxlen(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%hu\n", charger->rx_len);
}

static DEVICE_ATTR(rxlen, 0444, p9221_show_rxlen, NULL);

static ssize_t p9221_show_txdone(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->tx_done ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static DEVICE_ATTR(txdone, 0444, p9221_show_txdone, NULL);

static ssize_t p9221_show_txbusy(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->tx_busy ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static DEVICE_ATTR(txbusy, 0444, p9221_show_txbusy, NULL);

static ssize_t p9221_store_txlen(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u16 len;

	ret = kstrtou16(buf, 16, &len);
	if (ret < 0)
		return ret;
	if (ret > P9221R5_COM_CHAN_SEND_SIZE_REG)
		return -EINVAL;

	cancel_delayed_work_sync(&charger->tx_work);

	charger->tx_len = len;
	charger->tx_done = false;
	ret = p9221_send_data(charger);
	if (ret) {
		charger->tx_done = true;
		return ret;
	}

	schedule_delayed_work(&charger->tx_work,
			      msecs_to_jiffies(P9221_TX_TIMEOUT_MS));

	return count;
}

static DEVICE_ATTR(txlen, 0200, NULL, p9221_store_txlen);

static struct attribute *p9221_attributes[] = {
	&dev_attr_version.attr,
	&dev_attr_status.attr,
	&dev_attr_addr.attr,
	&dev_attr_count.attr,
	&dev_attr_data.attr,
	&dev_attr_ccreset.attr,
	&dev_attr_txbusy.attr,
	&dev_attr_txdone.attr,
	&dev_attr_txlen.attr,
	&dev_attr_rxlen.attr,
	&dev_attr_rxdone.attr,
	&dev_attr_icl_ramp_ua.attr,
	&dev_attr_icl_ramp_delay_ms.attr,
	NULL
};

static ssize_t p9221_rxdata_read(struct file *filp, struct kobject *kobj,
				 struct bin_attribute *bin_attr,
				 char *buf, loff_t pos, size_t size)
{
	struct p9221_charger_data *charger;
	charger = dev_get_drvdata(container_of(kobj, struct device, kobj));

	memcpy(buf, &charger->rx_buf[pos], size);
	charger->rx_done = false;
	return size;
}

static struct bin_attribute bin_attr_rxdata = {
	.attr = {
		.name = "rxdata",
		.mode = 0400,
	},
	.read = p9221_rxdata_read,
	.size = P9221R5_DATA_RECV_BUF_SIZE,
};

static ssize_t p9221_txdata_read(struct file *filp, struct kobject *kobj,
				 struct bin_attribute *bin_attr,
				 char *buf, loff_t pos, size_t size)
{
	struct p9221_charger_data *charger;
	charger = dev_get_drvdata(container_of(kobj, struct device, kobj));

	memcpy(buf, &charger->tx_buf[pos], size);
	return size;
}

static ssize_t p9221_txdata_write(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buf, loff_t pos, size_t size)
{
	struct p9221_charger_data *charger;
	charger = dev_get_drvdata(container_of(kobj, struct device, kobj));

	memcpy(&charger->tx_buf[pos], buf, size);
	return size;
}

static struct bin_attribute bin_attr_txdata = {
	.attr = {
		.name = "txdata",
		.mode = 0600,
	},
	.read = p9221_txdata_read,
	.write = p9221_txdata_write,
	.size  = P9221R5_DATA_SEND_BUF_SIZE,
};

static struct bin_attribute *p9221_bin_attributes[] = {
	&bin_attr_txdata,
	&bin_attr_rxdata,
	NULL,
};

static const struct attribute_group p9221_attr_group = {
	.attrs		= p9221_attributes,
	.bin_attrs	= p9221_bin_attributes,
};

static void print_current_samples(struct p9221_charger_data *charger,
					u32 *iout_val, int count)
{
	int i;
	char temp[P9221R5_OVER_CHECK_NUM * 9 + 1] = { 0 };

	for (i = 0; i < count ; i++)
		scnprintf(temp + i * 9, sizeof(temp) - i * 9,
			  "%08x ", iout_val[i]);

	dev_info(&charger->client->dev, "OVER IOUT_SAMPLES: %s\n", temp);
}

/*
 * Number of times to poll the status to see if the current limit condition
 * was transient or not.
 */
static void p9221_over_handle(struct p9221_charger_data *charger,
			      u16 irq_src)
{
	u8 reason = 0;
	int i;
	int ret;
	int ovc_count = 0;
	u32 iout_val[P9221R5_OVER_CHECK_NUM] = { 0 };

	dev_err(&charger->client->dev, "Received OVER INT: %02x\n", irq_src);

	if (irq_src & P9221R5_STAT_OVV) {
		reason = P9221_EOP_OVER_VOLT;
		goto send_eop;
	}

	if (irq_src & P9221R5_STAT_OVT) {
		reason = P9221_EOP_OVER_TEMP;
		goto send_eop;
	}

	if ((irq_src & P9221R5_STAT_UV) && !(irq_src & P9221R5_STAT_OVC))
		return;

	/* Overcurrent, reduce ICL and poll to absorb any transients */

	if (charger->dc_icl_votable) {
		int icl;

		icl = get_effective_result_locked(charger->dc_icl_votable);
		if (icl < 0) {
			dev_err(&charger->client->dev,
				"Failed to read ICL (%d)\n", icl);
		} else if (icl > OVC_BACKOFF_LIMIT) {
			icl -= OVC_BACKOFF_AMOUNT;

			ret = vote(charger->dc_icl_votable,
				   P9221_OCP_VOTER, true,
				   icl);
			dev_err(&charger->client->dev,
				"Reduced ICL to %d (%d)\n", icl, ret);
		}
	}

	reason = P9221_EOP_OVER_CURRENT;
	for (i = 0; i < P9221R5_OVER_CHECK_NUM; i++) {
		ret = p9221_clear_interrupts(charger,
					     irq_src & P9221R5_STAT_LIMIT_MASK);
		msleep(50);
		if (ret)
			continue;

		ret = p9221_reg_read_cooked(charger, P9221R5_IOUT_REG,
			&iout_val[i]);
		if (ret) {
			dev_err(&charger->client->dev,
				"Failed to read IOUT[%d]: %d\n", i, ret);
			continue;
		} else if (iout_val[i] > OVC_THRESHOLD) {
			ovc_count++;
		}

		ret = p9221_reg_read_16(charger, P9221_STATUS_REG, &irq_src);
		if (ret) {
			dev_err(&charger->client->dev,
				"Failed to read status: %d\n", ret);
			continue;
		}

		if ((irq_src & P9221R5_STAT_OVC) == 0) {
			print_current_samples(charger, iout_val, i + 1);
			dev_info(&charger->client->dev,
				 "OVER condition %04x cleared after %d tries\n",
				 irq_src, i);
			return;
		}

		dev_err(&charger->client->dev,
			"OVER status is still %04x, retry\n", irq_src);
	}

	if (ovc_count < OVC_LIMIT) {
		print_current_samples(charger, iout_val,
				      P9221R5_OVER_CHECK_NUM);
		dev_info(&charger->client->dev,
			 "ovc_threshold=%d, ovc_count=%d, ovc_limit=%d\n",
			 OVC_THRESHOLD, ovc_count, OVC_LIMIT);
		return;
	}

send_eop:
	dev_err(&charger->client->dev,
		"OVER is %04x, sending EOP %d\n", irq_src, reason);

	ret = p9221_send_eop(charger, reason);
	if (ret)
		dev_err(&charger->client->dev,
			"Failed to send EOP %d: %d\n", reason, ret);
}

/* Handler for R5 and R7 chips */
static void p9221_irq_handler(struct p9221_charger_data *charger, u16 irq_src)
{
	int res;

	if (irq_src & P9221R5_STAT_LIMIT_MASK)
		p9221_over_handle(charger, irq_src);

	/* Receive complete */
	if (irq_src & P9221R5_STAT_CCDATARCVD) {
		uint8_t rxlen = 0;
		res = p9221_reg_read_8(charger, P9221R5_COM_CHAN_RECV_SIZE_REG,
				       &rxlen);
		if (res) {
			dev_err(&charger->client->dev,
				"Failed to read len: %d\n", res);
			rxlen = 0;
		}
		if (rxlen) {
			res = p9221_reg_read_n(charger,
					       P9221R5_DATA_RECV_BUF_START,
					       charger->rx_buf, rxlen);
			if (res)
				dev_err(&charger->client->dev,
					"Failed to read len: %d\n", res);

			charger->rx_len = rxlen;
			charger->rx_done = true;
			sysfs_notify(&charger->dev->kobj, NULL, "rxdone");
		}
	}

	/* Send complete */
	if (irq_src & P9221R5_STAT_CCSENDBUSY) {
		charger->tx_busy = false;
		charger->tx_done = true;
		cancel_delayed_work(&charger->tx_work);
		sysfs_notify(&charger->dev->kobj, NULL, "txbusy");
		sysfs_notify(&charger->dev->kobj, NULL, "txdone");
	}

	/* Proprietary packet */
	if (irq_src & P9221R5_STAT_PPRCVD) {
		const size_t maxsz = sizeof(charger->pp_buf) * 3 + 1;
		char s[maxsz];

		res = p9221_reg_read_n(charger,
				       P9221R5_DATA_RECV_BUF_START,
				       charger->pp_buf,
				       sizeof(charger->pp_buf));
		if (res)
			dev_err(&charger->client->dev,
				"Failed to read PP len: %d\n", res);

		/* We only care about PP which come with 0x4F header */
		charger->pp_buf_valid = (charger->pp_buf[0] == 0x4F);

		p9221_hex_str(charger->pp_buf, sizeof(charger->pp_buf),
			      s, maxsz, false);
		dev_info(&charger->client->dev, "Received PP: %s\n", s);
	}

	/* CC Reset complete */
	if (irq_src & P9221R5_STAT_CCRESET)
		p9221_abort_transfers(charger);
}

static irqreturn_t p9221_irq_thread(int irq, void *irq_data)
{
	struct p9221_charger_data *charger = irq_data;
	int ret;
	u16 irq_src = 0;

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(charger->dev);

	ret = p9221_reg_read_16(charger, P9221_INT_REG, &irq_src);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to read INT reg: %d\n", ret);
		goto out;
	}

	dev_info(&charger->client->dev, "INT: %04x\n", irq_src);
	if (!irq_src)
		goto out;

	ret = p9221_clear_interrupts(charger, irq_src);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to clear INT reg: %d\n", ret);
		goto out;
	}

	if (irq_src & P9221_STAT_VRECT) {
		dev_info(&charger->client->dev,
			"Received VRECTON, online=%d\n", charger->online);
		if (!charger->online) {
			charger->check_det = true;
			pm_stay_awake(charger->dev);

			if (!schedule_delayed_work(&charger->notifier_work,
				msecs_to_jiffies(P9221_NOTIFIER_DELAY_MS))) {
				pm_relax(charger->dev);
			}
		}
	}

	p9221_irq_handler(charger, irq_src);

out:
	return IRQ_HANDLED;
}

static irqreturn_t p9221_irq_det_thread(int irq, void *irq_data)
{
	struct p9221_charger_data *charger = irq_data;

	/* If we are already online, just ignore the interrupt. */
	if (charger->online)
		return IRQ_HANDLED;

	/*
	 * This interrupt will wake the device if it's suspended,
	 * but it is not reliable enough to trigger the charging indicator.
	 * Give ourselves 2 seconds for the VRECTON interrupt to appear
	 * before we put up the charging indicator.
	 */
	mod_timer(&charger->vrect_timer,
		  jiffies + msecs_to_jiffies(P9221_VRECT_TIMEOUT_MS));
	pm_stay_awake(charger->dev);

	return IRQ_HANDLED;
}

static int p9221_parse_dt(struct device *dev,
			  struct p9221_charger_platform_data *pdata)
{
	int ret = 0;
	u32 data;
	struct device_node *node = dev->of_node;

	/* Enable */
	ret = of_get_named_gpio(node, "idt,gpio_qien", 0);
	pdata->qien_gpio = ret;
	if (ret < 0)
		dev_warn(dev, "unable to read idt,gpio_qien from dt: %d\n",
			 ret);
	else
		dev_info(dev, "enable gpio:%d", pdata->qien_gpio);

	/* Main IRQ */
	ret = of_get_named_gpio(node, "idt,irq_gpio", 0);
	if (ret < 0) {
		dev_err(dev, "unable to read idt,irq_gpio from dt: %d\n", ret);
		return ret;
	}
	pdata->irq_gpio = ret;
	pdata->irq_int = gpio_to_irq(pdata->irq_gpio);
	dev_info(dev, "gpio:%d, gpio_irq:%d\n", pdata->irq_gpio,
		 pdata->irq_int);

	/* Optional Detect IRQ */
	ret = of_get_named_gpio(node, "idt,irq_det_gpio", 0);
	pdata->irq_det_gpio = ret;
	if (ret < 0) {
		dev_warn(dev, "unable to read idt,irq_det_gpio from dt: %d\n",
			 ret);
	} else {
		pdata->irq_det_int = gpio_to_irq(pdata->irq_det_gpio);
		dev_info(dev, "det gpio:%d, det gpio_irq:%d\n",
			 pdata->irq_det_gpio, pdata->irq_det_int);
	}

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
	pdata->fod_num =
	    of_property_count_elems_of_size(node, "fod", sizeof(u8));
	if (pdata->fod_num <= 0) {
		dev_err(dev, "No dt fod provided (%d)\n", pdata->fod_num);
		pdata->fod_num = 0;
	} else {
		if (pdata->fod_num > P9221R5_NUM_FOD) {
			dev_err(dev,
			    "Incorrect num of FOD %d, using first %d\n",
			    pdata->fod_num, P9221R5_NUM_FOD);
			pdata->fod_num = P9221R5_NUM_FOD;
		}
		ret = of_property_read_u8_array(node, "fod", pdata->fod,
						pdata->fod_num);
		if (ret == 0) {
			char buf[pdata->fod_num * 3 + 1];

			p9221_hex_str(pdata->fod, pdata->fod_num, buf,
				      pdata->fod_num * 3 + 1, false);
			dev_info(dev, "dt fod: %s (%d)\n", buf, pdata->fod_num);
		}
	}

	pdata->fod_epp_num =
	    of_property_count_elems_of_size(node, "fod_epp", sizeof(u8));
	if (pdata->fod_epp_num <= 0) {
		dev_err(dev, "No dt fod epp provided (%d)\n",
			pdata->fod_epp_num);
		pdata->fod_epp_num = 0;
	} else {
		if (pdata->fod_epp_num > P9221R5_NUM_FOD) {
			dev_err(dev,
			    "Incorrect num of EPP FOD %d, using first %d\n",
			    pdata->fod_epp_num, P9221R5_NUM_FOD);
			pdata->fod_epp_num = P9221R5_NUM_FOD;
		}
		ret = of_property_read_u8_array(node, "fod_epp", pdata->fod_epp,
						pdata->fod_epp_num);
		if (ret == 0) {
			char buf[pdata->fod_epp_num * 3 + 1];

			p9221_hex_str(pdata->fod_epp, pdata->fod_epp_num, buf,
				      pdata->fod_epp_num * 3 + 1, false);
			dev_info(dev, "dt fod_epp: %s (%d)\n", buf,
				 pdata->fod_epp_num);
		}
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
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CAPACITY,
};

static const struct power_supply_desc p9221_psy_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = p9221_props,
	.num_properties = ARRAY_SIZE(p9221_props),
	.get_property = p9221_get_property,
	.set_property = p9221_set_property,
	.property_is_writeable = p9221_prop_is_writeable,
	.no_thermal = true,
};

static int p9221_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device_node *of_node = client->dev.of_node;
	struct p9221_charger_data *charger;
	struct p9221_charger_platform_data *pdata = client->dev.platform_data;
	struct power_supply_config psy_cfg = {};
	int ret = 0;
	u16 chip_id = 0;

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
	charger->resume_complete = true;
	mutex_init(&charger->io_lock);
	mutex_init(&charger->cmd_lock);
	setup_timer(&charger->vrect_timer, p9221_vrect_timer_handler,
		    (unsigned long)charger);
	INIT_DELAYED_WORK(&charger->dcin_work, p9221_dcin_work);
	INIT_DELAYED_WORK(&charger->tx_work, p9221_tx_work);
	INIT_DELAYED_WORK(&charger->icl_ramp_work, p9221_icl_ramp_work);
	alarm_init(&charger->icl_ramp_alarm, ALARM_BOOTTIME,
		   p9221_icl_ramp_alarm_cb);

	/* Default enable */
	charger->enabled = true;
	if (charger->pdata->qien_gpio >= 0)
		gpio_direction_output(charger->pdata->qien_gpio, 0);

	/* Default to R5+ */
	charger->cust_id = 5;

	psy_cfg.drv_data = charger;
	psy_cfg.of_node = charger->dev->of_node;
	charger->wc_psy = devm_power_supply_register(charger->dev,
						     &p9221_psy_desc,
						     &psy_cfg);
	if (IS_ERR(charger->wc_psy)) {
		dev_err(&client->dev, "Fail to register supply: %d\n", ret);
		return PTR_ERR(charger->wc_psy);
	}

	/*
	 * Find the DC_ICL votable, we use this to limit the current that
	 * is taken from the wireless charger.
	 */
	charger->dc_icl_votable = find_votable("DC_ICL");
	if (!charger->dc_icl_votable)
		dev_warn(&charger->client->dev, "Could not find DC_ICL votable\n");

	charger->icl_ramp_ua = P9221_DC_ICL_BPP_RAMP_DEFAULT_UA;
	charger->icl_ramp_delay_ms = P9221_DC_ICL_BPP_RAMP_DELAY_DEFAULT_MS;

	/* Test to see if the charger is online */
	ret = p9221_reg_read_16(charger, P9221_CHIP_ID_REG, &chip_id);
	if (ret == 0 && chip_id == P9221_CHIP_ID) {
		dev_info(&client->dev, "Charger online id:%04x\n", chip_id);
		/* set charger->online=true, will ignore first VRECTON IRQ */
		p9221_set_online(charger);
	} else {
		/* disconnected, (likely err!=0) vote for BPP */
		p9221_vote_defaults(charger);
	}

	ret = devm_request_threaded_irq(
		&client->dev, charger->pdata->irq_int, NULL,
		p9221_irq_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
		"p9221-irq", charger);
	if (ret) {
		dev_err(&client->dev, "Failed to request IRQ\n");
		return ret;
	}
	device_init_wakeup(charger->dev, true);

	/*
	 * We will receive a VRECTON after enabling IRQ if the device is
	 * if the device is already in-field when the driver is probed.
	 */
	enable_irq_wake(charger->pdata->irq_int);

	if (gpio_is_valid(charger->pdata->irq_det_gpio)) {
		ret = devm_request_threaded_irq(
			&client->dev, charger->pdata->irq_det_int, NULL,
			p9221_irq_det_thread,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "p9221-irq-det",
			charger);
		if (ret) {
			dev_err(&client->dev, "Failed to request IRQ_DET\n");
			return ret;
		}

		ret = devm_gpio_request_one(&client->dev,
					    charger->pdata->irq_det_gpio,
					    GPIOF_DIR_IN, "p9221-det-gpio");
		if (ret) {
			dev_err(&client->dev, "Failed to request GPIO_DET\n");
			return ret;
		}
		enable_irq_wake(charger->pdata->irq_det_int);
	}

	charger->last_capacity = -1;
	charger->count = 1;
	ret = sysfs_create_group(&charger->dev->kobj, &p9221_attr_group);
	if (ret) {
		dev_info(&client->dev, "sysfs_create_group failed\n");
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

	if (chip_id == P9221_CHIP_ID) {
		charger->dc_psy = power_supply_get_by_name("dc");
		if (charger->dc_psy)
			power_supply_changed(charger->dc_psy);
	}

	return 0;
}

static int p9221_charger_remove(struct i2c_client *client)
{
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&charger->dcin_work);
	cancel_delayed_work_sync(&charger->tx_work);
	cancel_delayed_work_sync(&charger->icl_ramp_work);
	alarm_try_to_cancel(&charger->icl_ramp_alarm);
	del_timer_sync(&charger->vrect_timer);
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

#ifdef CONFIG_PM_SLEEP
static int p9221_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	pm_runtime_get_sync(charger->dev);
	charger->resume_complete = false;
	pm_runtime_put_sync(charger->dev);

	return 0;
}

static int p9221_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	pm_runtime_get_sync(charger->dev);
	charger->resume_complete = true;
	pm_runtime_put_sync(charger->dev);

	return 0;
}
#endif
static const struct dev_pm_ops p9221_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(p9221_pm_suspend, p9221_pm_resume)
};

static struct i2c_driver p9221_charger_driver = {
	.driver = {
		.name		= "p9221",
		.owner		= THIS_MODULE,
		.of_match_table = p9221_charger_match_table,
		.pm		= &p9221_pm_ops,
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
