/*
 * Wireless charger driver for IDTP9017
 *
 * Copyright (C) 2016 LGE Inc.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/power/idtp9017_wireless_charger.h>

#define WLC_GET_INFO_DELAY_MS     (10*1000)
#define WLC_SET_ENV_INTERVAL_MS   (10*1000)
#define WLC_SET_ENV_DELAY_MS       500
#define WLC_CHECK_STATUS_DELAY_MS  500
#define WLC_ONLINE_CHK_DELAY_MS   (10*1000)
#define WLC_TX_OFF_LATENCY         3
#define WLC_ACTIVE_CHECK_COUNT     20
#define WLC_ACTIVE_CHECK_SLEEP_MS  200

struct idtp9017_chip {
	struct i2c_client *client;
	struct device *dev;
	int wlc_enable_gpio;
	int wlc_full_chg_gpio;
	int wlc_off_gpio;
	int set_out_voltage;
	int set_limit_current_ma;
	int x_axis;
	int y_axis;
	int fod1_gain;
	int fod2_gain;
	int die_shdn_off;
	int die_shdn_hys;
	int die_temp_off;
	int die_temp_hys;
	int mode_depth;
	bool online;
	bool psy_chg_en;
	bool wlc_chg_en;
	struct power_supply wlc_psy;
	struct delayed_work wlc_status_work;
	struct delayed_work set_env_work;
	struct timespec prev_time;
	struct mutex off_gpio_access_lock;
	bool set_env_complete;
	bool is_wlc_off_control;
	struct delayed_work wlc_online_check_work;
};

/* OFF PIN should be control from HIGH to LOW with 3sec latency.
 * Otherwise, LOW state does not work, so wireless Tx does not charging.
 * This func checks the time of OFF PIN HIGH control.
 * Then LOW control also checks if having 3sec latency. */
static inline void idtp9017_off_gpio_high_time_set(struct idtp9017_chip *chip)
{
	get_monotonic_boottime(&chip->prev_time);
	chip->prev_time.tv_sec += WLC_TX_OFF_LATENCY;
}

static void idtp9017_off_gpio_ctrl(struct idtp9017_chip *chip, bool value)
{
	struct timespec now;

	if (!gpio_is_valid(chip->wlc_off_gpio))
		return;

	mutex_lock(&chip->off_gpio_access_lock);
	if (chip->is_wlc_off_control == value) {
		mutex_unlock(&chip->off_gpio_access_lock);
		return;
	}

	chip->is_wlc_off_control = value;
	if (value) {
		/* Tx off case */
		cancel_delayed_work_sync(&chip->set_env_work);
		cancel_delayed_work_sync(&chip->wlc_status_work);
		 /* marking the time of OFF PIN HIGH control */
		disable_irq_wake(chip->client->irq);
		disable_irq(chip->client->irq);
		idtp9017_off_gpio_high_time_set(chip);
		gpio_set_value(chip->wlc_off_gpio, 1);
		schedule_delayed_work(&chip->wlc_online_check_work,
				round_jiffies_relative(
				msecs_to_jiffies(WLC_ONLINE_CHK_DELAY_MS)));
	} else {
		/* Tx on case */
		cancel_delayed_work_sync(&chip->wlc_online_check_work);
		get_monotonic_boottime(&now);
		/* checking if LOW control has 3sec latency from the time of
		 * OFF control.If not, should do 3sec sleep.
		 */
		if (timespec_compare(&now, &chip->prev_time) < 0)
			msleep(WLC_TX_OFF_LATENCY * 1000);

		enable_irq(chip->client->irq);
		enable_irq_wake(chip->client->irq);
		gpio_set_value(chip->wlc_off_gpio, 0);
		//irq_thread will start set_env_work and wlc_status_work after wireless active
	}
	dev_info(chip->dev, "Set WLC TX Off: %d\n", !chip->psy_chg_en);
	mutex_unlock(&chip->off_gpio_access_lock);
}

static int idtp9017_check_wlc_present(struct idtp9017_chip *chip)
{
	struct timespec now;
	int active_n = 0; /* Initial present condition */
	int i;

	if (!gpio_is_valid(chip->wlc_off_gpio))
		return -1;

	if (!gpio_is_valid(chip->wlc_enable_gpio))
		return -1;

	if (gpio_get_value(chip->wlc_off_gpio)) {
		/* TX off case */
		get_monotonic_boottime(&now);
		/* checking if LOW control has 3sec latency from the time of
		 * OFF control. If not, should return Tx present state for
		 * next workqueue's checking function.
		 * Normally below check logic runs with 10sec latency through
		 * the workqueue. But in abnormal IRQ storm case that having
		 * a lot of suspend and resume call, below logic is able to
		 * run within 3sec. */
		if (timespec_compare(&now, &chip->prev_time) >= 0) {
			gpio_set_value(chip->wlc_off_gpio, 0);
			for (i = 0; i < WLC_ACTIVE_CHECK_COUNT; i++) {
				active_n = gpio_get_value(chip->wlc_enable_gpio);
				msleep(WLC_ACTIVE_CHECK_SLEEP_MS);
				if (!active_n)
					break;
			}
			/* marking the time of OFF PIN HIGH control */
			idtp9017_off_gpio_high_time_set(chip);
			gpio_set_value(chip->wlc_off_gpio, 1);
		}
	}

	return !active_n;
}

static void
idtp9017_wlc_online_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct idtp9017_chip *chip = container_of(dwork,
				struct idtp9017_chip, wlc_online_check_work);
	int wlc_present;

	/* after setting present, charger's external_power_changed function
	 * turn on Wireless Tx through charging_enabled psy's prop. */
	wlc_present = idtp9017_check_wlc_present(chip);
	if (!wlc_present) {
		power_supply_set_present(&chip->wlc_psy, 0);
		return;
	}

	schedule_delayed_work(&chip->wlc_online_check_work,
			round_jiffies_relative(msecs_to_jiffies(
			WLC_ONLINE_CHK_DELAY_MS)));
}

static enum power_supply_property pm_power_props_wireless[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
};

static int pm_power_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		return 1;
	default:
		break;
	}

	return 0;
}

static char *pm_power_supplied_to[] = {
	"battery",
};

static int pm_power_get_property_wireless(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct idtp9017_chip *chip =
			container_of(psy, struct idtp9017_chip, wlc_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = psy->type;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->psy_chg_en;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pm_power_set_property_wireless(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct idtp9017_chip *chip =
		container_of(psy, struct idtp9017_chip, wlc_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		chip->online = val->intval;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		psy->type = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		chip->psy_chg_en = !!val->intval;
		idtp9017_off_gpio_ctrl(chip, !chip->psy_chg_en);
		break;
	default:
		return -EINVAL;
	}

	power_supply_changed(&chip->wlc_psy);
	return 0;
}

static int idtp9017_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev,
				"i2c read fail: can't read from %02x\n",
				reg);
		return ret;
	}

	*val = (u8)ret;
	return 0;
}

static int idtp9017_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev,
				"i2c write fail: can't write %02x to %02x\n",
				val, reg);
		return ret;
	}
	return 0;
}

static int idtp9017_masked_write(struct i2c_client *client,
		u8 reg, u8 mask, u8 val)
{
	s32 ret;
	u8 temp = 0x00;

	ret = idtp9017_read_reg(client, reg, &temp);
	if (ret < 0)
		return ret;

	temp &= ~mask;
	temp |= val & mask;

	ret = idtp9017_write_reg(client, reg, temp);
	if (ret < 0)
		return ret;

	return ret;
}

static int idtp9017_wlc_status(struct idtp9017_chip *chip, int *res)
{
	struct i2c_client *client;
	u8 reg_val_H = 0x00;
	u8 reg_val_L = 0x00;
	int ret = 0;
	int enable = 0;

	if (!chip)
		return 0;

	client = chip->client;

	ret = idtp9017_read_reg(client, RDST_6A_H, &reg_val_H);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to read RDST_6A_H\n");
		return ret;
	}

	dev_dbg(chip->dev, "Read reg_status_H: 0x%02x\n", reg_val_H);

	reg_val_H &= (ABNM_RAW_15 | ABNM_RAW_14 | ABNM_RAW_10);
	if (reg_val_H) {
		enable = 0;
		if (reg_val_H & ABNM_RAW_15)
			dev_warn(chip->dev, "TX not detected\n");
		if (reg_val_H & ABNM_RAW_14)
			dev_warn(chip->dev, "Too low freq\n");
		if (reg_val_H & ABNM_RAW_10)
			dev_warn(chip->dev, "Vrect over 8.5V\n");
	} else {
		enable = 1;
	}

	msleep(200);

	ret = idtp9017_read_reg(client, RDST_6A_L, &reg_val_L);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to read RDST_6A_L\n");
		return ret;
	}

	dev_dbg(chip->dev, "Read reg_status_L: 0x%02x\n", reg_val_L);

	reg_val_L &= (ABNM_RAW_2 | ABNM_RAW_0);
	if (reg_val_L & ABNM_RAW_2) {
		dev_info(chip->dev, "Charging complete\n");
		enable = 2;
	} else if (reg_val_L & ABNM_RAW_0) {
		dev_warn(chip->dev, "Charger disabled\n");
		enable = 0;
	} else {
		enable = 1;
	}

	*res = enable;

	return 0;
}

static int idtp9017_get_chg_mode(struct idtp9017_chip *chip, int *res)
{
	struct i2c_client *client;
	u8 reg_val = 0x00;
	int ret = 0;
	bool wpc_mode = false;

	if (!chip)
		return 0;

	client = chip->client;

	ret = idtp9017_read_reg(client, REG_CHG_MODE, &reg_val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read REG_CHG_MODE\n");
		return ret;
	}

	dev_dbg(chip->dev, "Charging mode val: 0x%02x\n", reg_val);

	if (reg_val >= 0x01)
		wpc_mode = true;
	else
		wpc_mode = false;

	*res = wpc_mode;

	return 0;
}

static int idtp9017_get_out_voltage(struct idtp9017_chip *chip, int *res)
{
	u8 reg_val_1 = 0x00;
	u8 reg_val_2 = 0x00;
	u16 reg_sum = 0x00;
	int ret = 0;
	int read_voltage = 0;

	ret = idtp9017_read_reg(chip->client, RDST_32_H, &reg_val_1);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to Vout rdst_32_h reg\n");
		return ret;
	}

	reg_val_1 &= ADC_4BIT;
	reg_sum = reg_val_1 << SHIFT_FOR_ADC;

	ret = idtp9017_read_reg(chip->client, RDST_32_L, &reg_val_2);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to Vout rdst_32_L reg\n");
		return ret;
	}

	reg_sum |= reg_val_2;
	read_voltage = (reg_sum * 25) / 10;
	dev_dbg(chip->dev, "Read Vout: %d\n", read_voltage);

	*res = read_voltage;

	return 0;
}

static int idtp9017_get_out_current(struct idtp9017_chip *chip, int *res)
{
	u8 reg_val_1 = 0x00;
	u8 reg_val_2 = 0x00;
	u16 reg_sum = 0x0000;
	int ret = 0;
	int read_current = 0;

	ret = idtp9017_read_reg(chip->client, RDST_31_H, &reg_val_1);
	if (ret < 0) {
		pr_err("Fail to Vout rdst_31_h reg\n");
		return ret;
	}
	reg_val_1 &= ADC_4BIT;
	reg_sum = reg_val_1 << SHIFT_FOR_ADC;
	ret = idtp9017_read_reg(chip->client, RDST_31_L, &reg_val_2);
	if (ret < 0) {
		pr_err("Fail to Vout rdst_31_L reg\n");
		return ret;
	}

	reg_sum |= reg_val_2;
	read_current = ((int)reg_sum * 5) / 10;
	pr_debug("Read Iout : %d\n", read_current);

	*res = read_current;

	return 0;
}

static int idtp9017_get_voltage_rect(struct idtp9017_chip *chip, int *res)
{
	u8 reg_val_1 = 0x00;
	u8 reg_val_2 = 0x00;
	u16 reg_sum = 0x0000;
	int ret = 0;
	int read_voltage = 0;

	ret = idtp9017_read_reg(chip->client, RDST_30_H, &reg_val_1);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to Vout rdst_30_h reg\n");
		return ret;
	}

	reg_val_1 &= ADC_4BIT;
	reg_sum = reg_val_1 << SHIFT_FOR_ADC;
	ret = idtp9017_read_reg(chip->client, RDST_30_L, &reg_val_2);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to Vout rdst_30_L reg\n");
		return ret;
	}

	reg_sum |= reg_val_2;
	read_voltage = (reg_sum * 25) / 10;
	dev_dbg(chip->dev, "Read Vrect: %d\n", read_voltage);

	*res = read_voltage;

	return 0;
}

static int idtp9017_get_die_temperature(struct idtp9017_chip *chip, int *res)
{
	u8 reg_val = 0x00;
	int ret = 0;
	int read_temperature = 0;

	/* order in which to read the temperature
	 * 1st : only read RDSS_33_H reg.
	 * 2nd : read  die_temp of RDSS_33_L reg. */
	ret = idtp9017_read_reg(chip->client, RDST_33_H, &reg_val);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to Vout rdst_33_H reg\n");
		return ret;
	}

	ret = idtp9017_read_reg(chip->client, RDST_33_L, &reg_val);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to Vout rdst_33_L reg\n");
		return ret;
	}

	read_temperature = ((reg_val * 100) / 107) - 55;
	dev_dbg(chip->dev, "Read die_temp :%d val: 0x%02x\n",
			read_temperature, reg_val);

	*res =  read_temperature;

	return 0;
}

static int idtp9017_get_align_axis(struct idtp9017_chip *chip)
{
	u8 x, y;
	int ret = 0;

	ret = idtp9017_read_reg(chip->client, RDST_36_L, &x);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to Vout rdst_36_h reg\n");
		return ret;
	}

	ret = idtp9017_read_reg(chip->client, RDST_37_L, &y);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to Vout rdst_37_h reg\n");
		return ret;
	}

	chip->x_axis = (int)(x & ADC_7BIT);
	chip->y_axis = (int)(y & ADC_7BIT);

	/*
	 * check sign bit: 1 negative, 0 positive
	 */
	if (x & SIGN_BIT)
		chip->x_axis = -chip->x_axis;
	if (y & SIGN_BIT)
		chip->y_axis = -chip->x_axis;

	dev_dbg(chip->dev, "%s: x axis: %d, y axis: %d\n", __func__,
			chip->x_axis, chip->y_axis);

	return 0;
}

/* Select select_fod_reg "1" or "2" */
static int idtp9017_get_fod_gain(struct idtp9017_chip *chip,
		int select_fod_reg, int *res)
{
	u8 reg_val = 0x00;
	int ret = 0;
	long int fod_gain;

	if (select_fod_reg == 1) {
		ret = idtp9017_read_reg(chip->client, RDST_38_L, &reg_val);
		if (ret < 0) {
			dev_err(chip->dev, "Fail to Vout rdst_33_h reg\n");
			return ret;
		}

		reg_val &= ADC_4BIT;
		fod_gain = (int)((reg_val * 78) + 8828);
	} else if (select_fod_reg == 2) {
		ret = idtp9017_read_reg(chip->client, RDST_37_L, &reg_val);
		if (ret < 0) {
			dev_err(chip->dev, "Fail to Vout rdst_33_h reg\n");
			return ret;
		}

		reg_val &= ADC_5BIT;
		fod_gain = (int)((reg_val * 3904) - 58560);
	}

	*res = fod_gain;

	return 0;
}

static int idtp9017_set_fod_gain(struct idtp9017_chip *chip,
		int select_fod_reg, long int gain)
{
	u8 reg_val = 0x00;
	u8 set_en = 0x00;
	int ret = 0;

	set_en = 1 << SHIFT_EN;

	dev_dbg(chip->dev, "%s: enable: 0x%02x, val: 0x%02x\n", __func__,
			set_en, reg_val);

	if (select_fod_reg == 1) {
		reg_val = (u8)((gain - 8828)/78);
		ret = idtp9017_masked_write(chip->client, REG_18_H,
				FOD1_EN, set_en);
		if (ret < 0) {
			dev_err(chip->dev, "Fail to Vout rdst_33_h reg\n");
			return ret;
		}

		ret = idtp9017_masked_write(chip->client, REG_18_H,
				FOD1_VALUE, reg_val);
		if (ret < 0) {
			dev_err(chip->dev, "Fail to Vout rdst_33_h reg\n");
			return ret;
		}
	} else if (select_fod_reg == 2) {
		reg_val = (u8)((gain + 58560) / 3904);
		ret = idtp9017_masked_write(chip->client, REG_18_L,
				FOD2_EN, set_en);
		if (ret < 0) {
			dev_err(chip->dev, "Fail to Vout rdst_33_h reg\n");
			return ret;
		}

		ret = idtp9017_masked_write(chip->client, REG_18_L,
				FOD2_VALUE, reg_val);
		if (ret < 0) {
			dev_err(chip->dev, "Fail to Vout rdst_33_h reg\n");
			return ret;
		}
	} else {
		dev_warn(chip->dev, "Not set FOD1 and FOD2 Gain\n");
	}

	return 0;
}

#define DEFAULT_CURRENT 1600
static int idtp9017_get_i_limit(struct idtp9017_chip *chip, int *res)
{
	u8 reg_val = 0x00;
	int ret = 0;
	int read_i_limit = 0;
	int i;

	if (chip->set_limit_current_ma != 0) {
		ret = idtp9017_read_reg(chip->client, RDST_3A_L, &reg_val);
		if (ret < 0) {
			dev_err(chip->dev, "Fail to Vout rdst_3a_l reg\n");
			return ret;
		}
		reg_val &= ADC_5BIT;

		for (i = ARRAY_SIZE(icl_ma_table) - 2; i >= 0; i--) {
			if (icl_ma_table[i].value == reg_val) {
				read_i_limit = icl_ma_table[i].icl_ma;
				dev_dbg(chip->dev, "I : %d, read_i : %d\n",
						i, read_i_limit);
				break;
			} else if (icl_ma_table[i].value < reg_val) {
				read_i_limit = icl_ma_table[i+1].icl_ma;
				dev_dbg(chip->dev, "I : %d, read_i : %d\n",
						i, read_i_limit);
				break;
			}
		}
	} else {
		read_i_limit = DEFAULT_CURRENT;
		dev_warn(chip->dev, "Not yet set, use default current(%d mA)\n",
				read_i_limit);
	}

	*res = read_i_limit;

	return 0;
}

static int idtp9017_get_target_voltage(struct idtp9017_chip *chip, int *res)
{
	u8 reg_val = 0x00;
	int ret = 0;
	int read_voltage = 0;

	ret = idtp9017_read_reg(chip->client, RDST_3B_L, &reg_val);
	if (ret < 0) {
		dev_err(chip->dev, "Fail to Vout rdst_3b_l reg\n");
		return ret;
	}

	reg_val &= ADC_5BIT;
	read_voltage = ((int)(reg_val & VSET_VALUE) * 100) + 4100;

	dev_dbg(chip->dev, "target_voltage: val: 0x%02x voltage: %d mV\n",
			reg_val, read_voltage);

	*res = read_voltage;

	return 0;
}

static int idtp9017_enable_i_limit(struct idtp9017_chip *chip,
		bool enable)
{
	u8 reg_val = 0x00;
	int ret = 0;

	reg_val = enable << 7;

	ret = idtp9017_masked_write(chip->client, REG_19_H,
			ILIM_EN, reg_val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to enable i_limit\n");
		return ret;
	}

	return 0;
}

static int idtp9017_set_i_limit(struct idtp9017_chip *chip,
		int set_current)
{
	u8 reg_val = 0x00;
	int ret = 0;
	int i;

	set_current = 1000;
	for (i = ARRAY_SIZE(icl_ma_table) - 5; i >= 0; i--) {
		if (icl_ma_table[i].icl_ma == set_current) {
			reg_val = icl_ma_table[i].value;
			dev_dbg(chip->dev,
					"i : %d, table_ma : %d,"
					" table_value : 0x%02x\n",
					i, icl_ma_table[i].icl_ma,
					icl_ma_table[i].value);
			break;
		}
	}

	ret = idtp9017_masked_write(chip->client, REG_19_H,
			ILIM_VALUE, reg_val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to set i_limit\n");
		return ret;
	}

	dev_info(chip->dev, "set_i_limit: val: 0x%02x, current: %d\n",
			reg_val, set_current);

	return 0;
}

static int idtp9017_enable_out_voltage(struct idtp9017_chip *chip,
		bool enable)
{
	u8 reg_val = 0x00;
	int ret = 0;

	reg_val = enable << 7;

	ret = idtp9017_masked_write(chip->client, REG_19_L,
			VSET_EN, reg_val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to enable out_voltage\n");
		return ret;
	}
	dev_info(chip->dev, "enable_voltage: val: 0x%02x, enable: %d\n",
			reg_val, enable);

	return 0;
}

static int idtp9017_set_out_voltage(struct idtp9017_chip *chip,
		int voltage)
{
	u8 reg_val = 0x00;
	int ret = 0;

	reg_val = (voltage - 4100) / 100;
	ret = idtp9017_masked_write(chip->client, REG_19_L,
			VSET_VALUE, reg_val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to set out_voltage\n");
		return ret;
	}
	dev_info(chip->dev, "out_voltage: val: 0x%02x, voltage: %d\n",
			reg_val, voltage);

	return 0;
}

static int die_shdn_control(struct idtp9017_chip *chip,
		int shdn_value, int hys_value)
{
	u8 reg_val = 0x00;
	int ret = 0;

	reg_val = (u8)shdn_value << SHIFT_THRESHOLD_OFF;
	if (shdn_value != 0) {
		reg_val = (u8)hys_value << SHIFT_THRESHOLD_HYS;

		ret = idtp9017_masked_write(chip->client, REG_04_H,
				(TDIE_SHDN_OFF | TDIE_SHDN_HYS), reg_val);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to write shdn_off\n");
			return ret;
		}
	} else {
		ret = idtp9017_masked_write(chip->client, REG_04_H,
				TDIE_SHDN_OFF, reg_val);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to write shdn_off\n");
			return ret;
		}
	}

	return 0;
}

static int die_temp_control(struct idtp9017_chip *chip,
		int temp_value, int hys_value)
{
	u8 reg_val = 0x00;
	int ret = 0;

	reg_val = (u8)temp_value << SHIFT_THRESHOLD_OFF;
	if (temp_value != 0) {
		reg_val = (u8)hys_value << SHIFT_THRESHOLD_HYS;

		ret = idtp9017_masked_write(chip->client, REG_04_L,
				(TDIE_THMR_OFF | TDIE_THMR_HYS), reg_val);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to write shdn_off\n");
			return ret;
		}
	} else {
		ret = idtp9017_masked_write(chip->client, REG_04_L,
				TDIE_THMR_OFF, reg_val);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to write shdn_off\n");
			return ret;
		}
	}

	return 0;
}

static int idtp9017_get_operate_freq_in_khz(struct idtp9017_chip *chip, int *res)
{
	u8 reg_val = 0x00;
	int reg_sum = 0;
	int ret = 0;
	int freq_in_khz = 0;

	ret = idtp9017_read_reg(chip->client, REG_3F_H, &reg_val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed get freq\n");
		return ret;
	}

	reg_sum = (reg_val & ADC_4BIT) << SHIFT_FOR_ADC;

	ret = idtp9017_read_reg(chip->client, REG_3F_L, &reg_val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed get freq\n");
		return ret;
	}
	reg_sum |= reg_val;
	/* freq = 1 / (reg_sum * 3.125ns) */
	freq_in_khz = 1000000 / ((reg_sum * 3125) / 1000);

	dev_dbg(chip->dev, "reg_val : 0x%03x, freq : %d KHz\n",
			reg_sum, freq_in_khz);

	*res = freq_in_khz;

	return 0;
}

static void idtp9017_set_enviroment(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct idtp9017_chip *chip = container_of(dwork, struct idtp9017_chip,
			set_env_work);
	struct device *dev = chip->dev;
	int ret = 0;
	static int delayed_counter;

	if (chip->set_env_complete)
		return;

	chip->wlc_chg_en = !(gpio_get_value(chip->wlc_enable_gpio));
	if (chip->wlc_chg_en)
		delayed_counter++;

	if (chip->wlc_chg_en && delayed_counter >= 2) {
		if (chip->set_limit_current_ma != 0) {
			ret = idtp9017_enable_i_limit(chip, true);
			if (ret < 0) {
				dev_err(dev, "Failed enable limit_currnet\n");
				goto error;
			}

			ret = idtp9017_set_i_limit(chip,
					chip->set_limit_current_ma);
			if (ret < 0) {
				dev_err(dev, "Failed set limit_currnet\n");
				goto error;
			}
		} else {
			dev_err(dev, "Not set limit_current\n");
			ret = idtp9017_enable_i_limit(chip,
					chip->set_limit_current_ma);
			if (ret < 0) {
				dev_err(dev, "Failed enable limit_currnet\n");
				goto error;
			}
		}

		if (chip->set_out_voltage != 0) {
			ret = idtp9017_enable_out_voltage(chip, true);
			if (ret < 0) {
				dev_err(dev, "Failed enable out_voltage\n");
				goto error;
			}

			ret = idtp9017_set_out_voltage(chip,
					chip->set_out_voltage);
			if (ret < 0) {
				dev_err(dev, "Failed set out_voltage\n");
				goto error;
			}
		} else {
			dev_err(dev, "Not set out_voltage\n");
			ret = idtp9017_enable_out_voltage(chip, false);
			if (ret < 0) {
				dev_err(dev, "Failed enable out_voltage\n");
				goto error;
			}
		}

		if (chip->fod1_gain != 0) {
			ret = idtp9017_set_fod_gain(chip, 1, chip->fod1_gain);
			if (ret < 0) {
				dev_err(dev, "Failed set fod1\n");
				goto error;
			}
		} else {
			dev_dbg(dev, "Not set fod1_gain\n");
		}

		if (chip->fod2_gain != 0) {
			ret = idtp9017_set_fod_gain(chip, 2, chip->fod2_gain);
			if (ret < 0) {
				dev_err(dev, "Failed set fod1\n");
				goto error;
			}
		} else {
			dev_dbg(dev, "Not set fod2_gain\n");
		}

		if (chip->die_shdn_off != 0) {
			ret = die_shdn_control(chip, chip->die_shdn_off,
					chip->die_shdn_hys);
			if (ret < 0) {
				dev_err(dev, "Failed set die_shdn_off\n");
				goto error;
			}
		}

		if (chip->die_temp_off) {
			ret = die_temp_control(chip, chip->die_temp_off,
					chip->die_temp_hys);
			if (ret < 0) {
				dev_err(dev, "Failed set die_temp_off\n");
				goto error;
			}
		}

		dev_info(dev, "Complete to set enviroment\n");
		chip->set_env_complete = true;
	} else {
		if (chip->wlc_chg_en)
			dev_warn(dev, "Waiting, not yet set enviroment\n");
		else
			delayed_counter = 0;

		schedule_delayed_work(&chip->set_env_work,
			round_jiffies_relative(
				msecs_to_jiffies(WLC_SET_ENV_INTERVAL_MS)));
	}

	return;

error:
	schedule_delayed_work(&chip->set_env_work,
		round_jiffies_relative(
			msecs_to_jiffies(WLC_SET_ENV_DELAY_MS )));
}

static void wlc_info_worker(struct work_struct *work)
{
	static int check_counter;
	struct idtp9017_chip *chip = container_of(work, struct idtp9017_chip,
			wlc_status_work.work);
	int limit_cur_ma = 0;
	int out_cur_ma = 0;
	int out_vol_mv = 0;
	int target_vol = 0;
	int rect_vol_mv = 0;
	int wpc_mode = 0;
	int die_temp = 0;
	int delay = WLC_GET_INFO_DELAY_MS;
	int fod1_gain = 0;
	int fod2_gain = 0;
	int wlc_status = 0;
	int op_freq_in_khz = 0;
	int chg_done = 0;

	chip->wlc_chg_en = !(gpio_get_value(chip->wlc_enable_gpio));
	if (chip->wlc_chg_en)
		check_counter++;
	else
		check_counter = 0;

	if (check_counter >= 2 && chip->set_env_complete) {
		idtp9017_get_i_limit(chip, &limit_cur_ma);
		idtp9017_get_out_current(chip, &out_cur_ma);
		idtp9017_get_out_voltage(chip, &out_vol_mv);
		idtp9017_get_target_voltage(chip, &target_vol);
		idtp9017_get_die_temperature(chip, &die_temp);
		idtp9017_get_voltage_rect(chip, &rect_vol_mv);
		idtp9017_get_chg_mode(chip, &wpc_mode);
		idtp9017_get_fod_gain(chip, 1, &fod1_gain);
		idtp9017_get_fod_gain(chip, 2, &fod2_gain);
		idtp9017_get_align_axis(chip);
		idtp9017_get_operate_freq_in_khz(chip, &op_freq_in_khz);
		idtp9017_wlc_status(chip, &wlc_status);
		if (wlc_status == 2)
			chg_done = 1;

		dev_info(chip->dev, "chg_en: %s, chg_mode: %s, chg_done: %s\n",
				wpc_mode ? "wpc_mode" : "pma_mode",
				wlc_status ? "enable" : "disable",
				chg_done ? "Done" : "Not yet");
		dev_info(chip->dev, "Op_freq: %d KHz, Limit_cur: %d mA,"
				" Out_cur %dmA\n",
				op_freq_in_khz, limit_cur_ma, out_cur_ma);
		dev_info(chip->dev, "target_vol: %dmV, Out_vol: %dmV,"
				" Vrect: %dmV\n",
				target_vol, out_vol_mv, rect_vol_mv);
		dev_info(chip->dev, "Die_temperature: %d, Axis(%d, %d)\n",
				die_temp, chip->x_axis, chip->y_axis);
		dev_info(chip->dev, "Fod1_gain: %d.%d%%, Fod2_gain: %d.%dmW\n",
				fod1_gain/100, fod1_gain%100, fod2_gain/100,
				fod2_gain%100);
		if (chg_done)
			delay = delay * 100;
		else
			delay = delay * 50;
	}

	if (check_counter)
		schedule_delayed_work(&chip->wlc_status_work,
			round_jiffies_relative(msecs_to_jiffies(delay)));
	else
		dev_warn(chip->dev, "WLC is not connected\n");
}

static irqreturn_t idtp9017_irq_thread(int irq, void *handle)
{
	struct idtp9017_chip *chip = (struct idtp9017_chip *)handle;

	chip->wlc_chg_en = !(gpio_get_value(chip->wlc_enable_gpio));

	dev_dbg(chip->dev, "%s: chg_en: %d\n", __func__, chip->wlc_chg_en);

	if (!chip->is_wlc_off_control) {
		if (chip->wlc_chg_en) {
			schedule_delayed_work(&chip->set_env_work,
				round_jiffies_relative(
					msecs_to_jiffies(WLC_SET_ENV_DELAY_MS)));
			schedule_delayed_work(&chip->wlc_status_work,
				round_jiffies_relative(
					msecs_to_jiffies(WLC_CHECK_STATUS_DELAY_MS)));
		} else {
			cancel_delayed_work_sync(&chip->wlc_status_work);
		}
	}

	return IRQ_HANDLED;
}

static int idtp9017_parse_dt(struct device_node *dev_node,
		struct idtp9017_chip *chip)
{
	int ret;
	struct device *dev = chip->dev;

	chip->wlc_enable_gpio = of_get_named_gpio(dev_node,
			"idt,wlc-enable-gpio", 0);
	if (chip->wlc_enable_gpio < 0) {
		dev_err(dev, "Fail to get wlc-enable-gpio\n");
		return chip->wlc_enable_gpio;
	}
	dev_info(dev, "Get wlc-enable-gpio: %d\n",
				chip->wlc_enable_gpio);

	chip->wlc_full_chg_gpio = of_get_named_gpio(dev_node,
			"idt,wlc-full-chg-gpio", 0);
	if (chip->wlc_full_chg_gpio < 0) {
		dev_err(dev, "Fail to get wlc-full-chg-gpio\n");
		return chip->wlc_full_chg_gpio;
	}
	dev_info(dev, "Get wlc-full-chg-gpio: %d\n",
				chip->wlc_full_chg_gpio);

	chip->wlc_off_gpio = of_get_named_gpio(dev_node,
			"idt,wlc-off-gpio", 0);
	if (chip->wlc_off_gpio < 0) {
		dev_err(dev, "Fail to get wlc-off-gpio\n");
		return chip->wlc_off_gpio;
	}
	dev_info(dev, "Get wlc-off-gpio: %d\n",
				chip->wlc_off_gpio);

	ret = of_property_read_u32(dev_node,
			"idt,mode-depth", &chip->mode_depth);
	if (ret)
		dev_warn(dev, "Not exist mode_depth paramaeter\n");
	dev_info(dev, "Get mode-depth: %d\n", chip->mode_depth);

	ret = of_property_read_u32(dev_node,
			"idt,fod1-gain", &chip->fod1_gain);
	if (ret)
		dev_warn(dev, "Not exist fod1_gain paramaeter\n");
	dev_info(dev, "Get fod1-gain: %d\n", chip->fod1_gain);

	ret = of_property_read_u32(dev_node,
			"idt,fod2-gain", &chip->fod2_gain);
	if (ret)
		dev_warn(dev, "Not exist fod2_gain paramaeter\n");
	dev_info(dev, "Get fod2-gain: %d\n", chip->fod2_gain);

	ret = of_property_read_u32(dev_node,
			"idt,die-shdn-off", &chip->die_shdn_off);
	if (ret)
		dev_warn(dev, "Not exist die_shdn_off paramaeter\n");
	dev_info(dev, "Get die-shdn-off: %d\n", chip->die_shdn_off);

	ret = of_property_read_u32(dev_node,
			"idt,die-shdn-hys", &chip->die_shdn_hys);
	if (ret)
		dev_warn(dev, "Not exist die-shdn-hys paramaeter\n");
	dev_info(dev, "Get die-shdn-hys: %d\n", chip->die_shdn_hys);

	ret = of_property_read_u32(dev_node,
			"idt,die-temp-off", &chip->die_temp_off);
	if (ret)
		dev_warn(dev, "Not exist die-temp-off paramaeter\n");
	dev_info(dev, "Get die-temp-off: %d\n", chip->die_temp_off);

	ret = of_property_read_u32(dev_node,
			"idt,die-temp-hys", &chip->die_temp_hys);
	if (ret)
		dev_warn(dev, "Not exist die-temp-hys paramaeter\n");
	dev_info(dev, "Get die-temp-hys: %d\n", chip->die_temp_hys);

	ret = of_property_read_u32(dev_node,
			"idt,limit-current", &chip->set_limit_current_ma);
	if (ret)
		dev_warn(dev, "Not exist limit-current paramaeter\n");
	dev_info(dev, "Get limit-current: %d\n", chip->set_limit_current_ma);

	ret = of_property_read_u32(dev_node,
			"idt,out-voltage", &chip->set_out_voltage);
	if (ret)
		dev_err(dev, "Not exist out-voltage paramaeter\n");
	dev_info(dev, "Get out-voltage: %d\n", chip->set_out_voltage);

	return 0;
}

static int idtp9017_init_gpio(struct idtp9017_chip *chip)
{
	int ret = 0;

	ret = devm_gpio_request_one(chip->dev,
			chip->wlc_enable_gpio, GPIOF_DIR_IN,
			"wlc_enable_gpio");
	if (ret < 0) {
		dev_err(chip->dev, "Fail to request wlc_enable_gpio\n");
		return ret;
	}

	ret = devm_gpio_request_one(chip->dev,
			chip->wlc_full_chg_gpio, GPIOF_OUT_INIT_LOW,
			"wlc_full_chg_gpio");
	if (ret < 0) {
		dev_err(chip->dev, "Fail to request wlc_full_chg_gpio\n");
		return ret;
	}

	ret = devm_gpio_request_one(chip->dev,
			chip->wlc_off_gpio, GPIOF_OUT_INIT_LOW,
			"wlc_off_gpio");
	if (ret < 0) {
		dev_err(chip->dev, "Fail to request wlc_off_gpio\n");
		return ret;
	}

	return 0;
}

static int idtp9017_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct idtp9017_chip *chip;
	struct device_node *dev_node = client->dev.of_node;
	int ret;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("i2c func fail.\n");
		return -EIO;
	}

	chip = devm_kzalloc(&client->dev,
			sizeof(struct idtp9017_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "No memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->psy_chg_en = 1;

	/* need dts parser */
	if (dev_node) {
		ret = idtp9017_parse_dt(dev_node, chip);
		if (ret < 0) {
			dev_err(&client->dev, "Fail to read parse_dt\n");
			return ret;
		}
	}

	ret = idtp9017_init_gpio(chip);
	if (ret < 0) {
		dev_err(&client->dev, "Fail to request GPIOs\n");
		return ret;
	}

	i2c_set_clientdata(client, chip);

	/*Set Power Supply type for wlc*/
	chip->wlc_psy.name = "wireless";
	chip->wlc_psy.type = POWER_SUPPLY_TYPE_WIRELESS;
	chip->wlc_psy.supplied_to = pm_power_supplied_to;
	chip->wlc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	chip->wlc_psy.properties = pm_power_props_wireless;
	chip->wlc_psy.num_properties = ARRAY_SIZE(pm_power_props_wireless);
	chip->wlc_psy.get_property = pm_power_get_property_wireless;
	chip->wlc_psy.set_property = pm_power_set_property_wireless;
	chip->wlc_psy.property_is_writeable = pm_power_property_is_writeable;
	ret = power_supply_register(&chip->client->dev, &chip->wlc_psy);
	if (ret < 0) {
		dev_err(&client->dev,
			"Couldn't register power supply for wireless\n");
		return ret;
	}

	mutex_init(&chip->off_gpio_access_lock);
	INIT_DELAYED_WORK(&chip->wlc_status_work, wlc_info_worker);
	INIT_DELAYED_WORK(&chip->set_env_work, idtp9017_set_enviroment);
	INIT_DELAYED_WORK(&chip->wlc_online_check_work,
			idtp9017_wlc_online_check_work);

	/* Explicitly call irq thread for initial queuing on wireless booting case */
	idtp9017_irq_thread(client->irq, (void *)chip);
	ret = devm_request_threaded_irq(&client->dev,
		client->irq, NULL, idtp9017_irq_thread,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"idtp9017_irq", chip);
	if (ret) {
		dev_err(&client->dev, "failed to reqeust IRQ\n");
		return ret;
	}
	enable_irq_wake(client->irq);

	dev_info(&client->dev, "IDTP9017 probed\n");

	return 0;
}

static void idtp9017_resume(struct device *dev)
{
	struct idtp9017_chip *chip = dev_get_drvdata(dev);

	if (!chip->psy_chg_en)
		schedule_delayed_work(&chip->wlc_online_check_work, 0);

	if (!chip->is_wlc_off_control && chip->wlc_chg_en)
		schedule_delayed_work(&chip->wlc_status_work,
			round_jiffies_relative(
				msecs_to_jiffies(WLC_CHECK_STATUS_DELAY_MS)));
}

static int idtp9017_suspend(struct device *dev)
{
	struct idtp9017_chip *chip = dev_get_drvdata(dev);

	if (!chip->psy_chg_en)
		cancel_delayed_work_sync(&chip->wlc_online_check_work);

	if (!chip->is_wlc_off_control && chip->wlc_chg_en)
		cancel_delayed_work_sync(&chip->wlc_status_work);

	return 0;
}

static const struct dev_pm_ops idtp9017_pm_ops = {
	.prepare  = idtp9017_suspend,
	.complete = idtp9017_resume,
};

static int idtp9017_remove(struct i2c_client *client)
{
	return 0;
}

static struct of_device_id idt_idtp9017_table[] = {
	{ .compatible = "idt,idtp9017", },
	{},
};

static const struct i2c_device_id idtp9017_id[] = {
	{IDTP9017_NAME, 0},
	{}
};

static struct i2c_driver idtp9017_driver = {
	.driver = {
		.name	= IDTP9017_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = idt_idtp9017_table,
		.pm	= &idtp9017_pm_ops,
	},
	.probe		= idtp9017_probe,
	.id_table	= idtp9017_id,
	.remove		= idtp9017_remove,
};

static int __init idtp9017_init(void)
{
	return i2c_add_driver(&idtp9017_driver);
}
module_init(idtp9017_init);

static void __exit idtp9017_exit(void)
{
	return i2c_del_driver(&idtp9017_driver);
}
module_exit(idtp9017_exit);

MODULE_DESCRIPTION("idtp9017");
MODULE_LICENSE("GPL V2");
