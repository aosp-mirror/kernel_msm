/*
 * Fuel gauge driver for Maxim 17055
 *
 * Copyright (C) 2016 Maxim Integrated
 * Bo Yang <Bo.Yang@maximintegrated.com>
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
 *
 * This driver is based on max17042_battery.c
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/power/max17055_battery.h>
#include <linux/of.h>
#include <linux/regmap.h>

/* Status register bits */
#define STATUS_POR_BIT             (1 << 1)
#define STATUS_BST_BIT             (1 << 3)
#define STATUS_SOCI_BIT            (1 << 7)
#define STATUS_VMN_BIT             (1 << 8)
#define STATUS_TMN_BIT             (1 << 9)
#define STATUS_SMN_BIT             (1 << 10)
#define STATUS_BI_BIT              (1 << 11)
#define STATUS_VMX_BIT             (1 << 12)
#define STATUS_TMX_BIT             (1 << 13)
#define STATUS_SMX_BIT             (1 << 14)
#define STATUS_BR_BIT              (1 << 15)

#define STATUS2_HIB_BIT            (1 << 1)

#define CONFIG2_POR_CMD_BIT        (1 << 0)
#define CONFIG2_LDMDL_BIT          (1 << 5)

#define HIBCFG_ENHIB_BIT           (1 << 15)

#define VFSOC0_LOCK                0x0000
#define VFSOC0_UNLOCK              0x0080
#define MODEL_UNLOCK1              0X0059
#define MODEL_UNLOCK2              0X00C4
#define MODEL_LOCK1                0X0000
#define MODEL_LOCK2                0X0000

#define MAX17055_VMAX_TOLERANCE    50			/* 50 mV */
#define MAX17055_IC_VERSION_A      0x4000
#define MAX17055_IC_VERSION_B      0x4010
#define MAX17055_DRIVER_VERSION    0x101f

#if CONFIG_HUAWEI_SAWSHARK
#define MAX17055_CAPACITY_CARRY              127
#define MAX17055_CAPACITY_CARRY_THRESHOLD    1
#define MAX17055_SOCHOLD                     0xD3
#define SOCHOLD_ENHOLD_BIT                   (1 << 12)
#endif

const static u16 dPaccVals[2][2] = {
	{0x0159, 0x0190},
	{0x0ac7, 0x0c80}
};

struct max17055_chip {
	struct i2c_client *client;
	struct regmap *regmap;
	struct power_supply battery;
	enum max17055_chip_type chip_type;
	struct max17055_platform_data *pdata;
	struct work_struct work;
	int    init_complete;
};

#if CONFIG_HUAWEI_SAWSHARK
struct max17055_chip *global_max17055 = NULL;
#endif

static enum power_supply_property max17055_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_DEVICE_NAME,
};

#if CONFIG_HUAWEI_SAWSHARK
#define DEFAULT_TEMP 250
#endif
static int max17055_get_temperature(struct max17055_chip *chip, int *temp)
{
#if CONFIG_HUAWEI_SAWSHARK
	union power_supply_propval ret = {0, };

	if (!chip->pdata->chg_psy && chip->pdata->chg_psy_name)
	{
		pr_info("get battery power supply\n");
		chip->pdata->chg_psy = power_supply_get_by_name((char *)chip->pdata->chg_psy_name);
	}

	if (chip->pdata->chg_psy)
	{
		chip->pdata->chg_psy->get_property(chip->pdata->chg_psy,
					POWER_SUPPLY_PROP_TEMP, &ret);
		*temp = ret.intval;
	}
	else
	{
		*temp = DEFAULT_TEMP;
	}

	return 0;
#else
	int ret;
	u32 data;
	struct regmap *map = chip->regmap;

	ret = regmap_read(map, MAX17055_Temp, &data);
	if (ret < 0)
		return ret;

	*temp = data;
	/* The value is signed. */
	if (*temp & 0x8000) {
		*temp = (0x7fff & ~*temp) + 1;
		*temp *= -1;
	}

	/* The value is converted into deci-centigrade scale */
	/* Units of LSB = 1 / 256 degree Celsius */
	*temp = *temp * 10 / 256;
	return 0;
#endif
}

static int max17055_get_battery_health(struct max17055_chip *chip, int *health)
{
	int temp, vavg, vbatt, ret;
	u32 val;

	ret = regmap_read(chip->regmap, MAX17055_AvgVCell, &val);
	if (ret < 0)
		goto health_error;

	/* bits [0-3] unused */
	vavg = val * 625 / 8;
	/* Convert to millivolts */
	vavg /= 1000;

	ret = regmap_read(chip->regmap, MAX17055_VCell, &val);
	if (ret < 0)
		goto health_error;

	/* bits [0-3] unused */
	vbatt = val * 625 / 8;
	/* Convert to millivolts */
	vbatt /= 1000;

	if (vavg < chip->pdata->vmin) {
		*health = POWER_SUPPLY_HEALTH_DEAD;
		goto out;
	}

	if (vbatt > chip->pdata->vmax + MAX17055_VMAX_TOLERANCE) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		goto out;
	}

	ret = max17055_get_temperature(chip, &temp);
	if (ret < 0)
		goto health_error;

	if (temp <= chip->pdata->temp_min) {
		*health = POWER_SUPPLY_HEALTH_COLD;
		goto out;
	}

	if (temp >= chip->pdata->temp_max) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		goto out;
	}

	*health = POWER_SUPPLY_HEALTH_GOOD;

out:
	return 0;

health_error:
	return ret;
}

#if CONFIG_HUAWEI_SAWSHARK
int max17055_global_get_real_capacity(void)
{
	struct regmap *map = global_max17055->regmap;
	int ret = -1;
	u32 data = -1;
	u32 cur_soc = 0;

	if (!global_max17055)
	{
		pr_err("max17055 chip can not register\n");
		return -1;
	}

	ret = regmap_read(map, MAX17055_RepSOC, &data);
	if (ret < 0)
	{
		return ret;
	}

	/* rounding if capacity is greater or equal to 1 */
	cur_soc = (data >> 8);
	if (cur_soc >= MAX17055_CAPACITY_CARRY_THRESHOLD)
	{
		return ((data + MAX17055_CAPACITY_CARRY) >> 8);
	}
	else
	{
		return (data >> 8);
	}
}

extern bool mp2661_global_get_repeat_charging_detect_flag(void);
#define DEFAULT_BATT_AUTO_RECHARGE_CAPACITY   100
#endif

static int max17055_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max17055_chip *chip = container_of(psy,
				struct max17055_chip, battery);
	struct regmap *map = chip->regmap;
	int ret;
	u32 data;
#if CONFIG_HUAWEI_SAWSHARK
	u32 cur_soc = 0;
#endif

	if (!chip->init_complete)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(map, MAX17055_Status, &data);
		if (ret < 0)
			return ret;

		if (data & STATUS_BST_BIT)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(map, MAX17055_Status2, &data);
		if (ret < 0)
			return ret;

		if (data & STATUS2_HIB_BIT)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = regmap_read(map, MAX17055_Cycles, &data);
		if (ret < 0)
			return ret;

		val->intval = data;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = regmap_read(map, MAX17055_MaxMinVolt, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 8;
		val->intval *= 20000; /* Units of LSB = 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = regmap_read(map, MAX17055_VEmpty, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 7;
		val->intval *= 10000; /* Units of LSB = 10mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(map, MAX17055_VCell, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = regmap_read(map, MAX17055_AvgVCell, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = regmap_read(map, MAX17055_VFOCV, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = regmap_read(map, MAX17055_RepSOC, &data);
		if (ret < 0)
			return ret;

#if CONFIG_HUAWEI_SAWSHARK
		if (mp2661_global_get_repeat_charging_detect_flag())
		{
			/* report 100% if repeat charging detect flag is true */
			val->intval = DEFAULT_BATT_AUTO_RECHARGE_CAPACITY;
		}
		else
		{
			/* rounding if capacity is greater or equal to 1 */
			cur_soc = (data >> 8);
			if (cur_soc >= MAX17055_CAPACITY_CARRY_THRESHOLD)
			{
				val->intval = ((data + MAX17055_CAPACITY_CARRY) >> 8);
			}
			else
			{
				val->intval = (data >> 8);
			}
		}
#else
		val->intval = data >> 8;
#endif
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = regmap_read(map, MAX17055_FullCapRep, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 5000 / chip->pdata->r_sns;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = regmap_read(map, MAX17055_RepCap, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 5000 / chip->pdata->r_sns;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = max17055_get_temperature(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = regmap_read(map, MAX17055_TAlrtTh, &data);
		if (ret < 0)
			return ret;
		/* LSB is Alert Minimum. In deci-centigrade */
		val->intval = (data & 0xff) * 10;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = regmap_read(map, MAX17055_TAlrtTh, &data);
		if (ret < 0)
			return ret;
		/* MSB is Alert Maximum. In deci-centigrade */
		val->intval = (data >> 8) * 10;
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
		val->intval = chip->pdata->temp_min;
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		val->intval = chip->pdata->temp_max;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max17055_get_battery_health(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (chip->pdata->enable_current_sense) {
			ret = regmap_read(map, MAX17055_Current, &data);
			if (ret < 0)
				return ret;

			val->intval = data;
			if (val->intval & 0x8000) {
				/* Negative */
				val->intval = ~val->intval & 0x7fff;
				val->intval++;
				val->intval *= -1;
			}
			val->intval *= 1562500 / chip->pdata->r_sns;
		} else {
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (chip->pdata->enable_current_sense) {
			ret = regmap_read(map, MAX17055_AvgCurrent, &data);
			if (ret < 0)
				return ret;

			val->intval = data;
			if (val->intval & 0x8000) {
				/* Negative */
				val->intval = ~val->intval & 0x7fff;
				val->intval++;
				val->intval *= -1;
			}
			val->intval *= 1562500 / chip->pdata->r_sns;
		} else {
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_DEVICE_NAME:
		ret = regmap_read(map, MAX17055_DevName, &data);
		if (ret < 0)
		{
			return ret;
		}

		val->intval = data;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

#if CONFIG_HUAWEI_SAWSHARK
#define SOC_MAX    100
#define SOC_MIN    0
static int max17055_write_verify_reg(struct regmap *map, u8 reg, u32 value);
#endif
static int max17055_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct max17055_chip *chip = container_of(psy,
				struct max17055_chip, battery);
	struct regmap *map = chip->regmap;
	int ret = 0;
	u32 data;
	int8_t temp;

	switch (psp) {
#if CONFIG_HUAWEI_SAWSHARK
	case POWER_SUPPLY_PROP_CAPACITY:
		/* update fullcaprep to repcap */
		if ((val->intval <= SOC_MAX) && (val->intval >= SOC_MIN))
		{
			ret = regmap_read(map, MAX17055_RepCap, &data);
			if (ret < 0)
			{
				return ret;
			}

			max17055_write_verify_reg(map, MAX17055_FullCapRep, data);
			power_supply_changed(&chip->battery);
		}
		break;
#endif
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = regmap_read(map, MAX17055_TAlrtTh, &data);
		if (ret < 0)
			return ret;

		/* Input in deci-centigrade, convert to centigrade */
		temp = val->intval / 10;
		/* force min < max */
		if (temp >= (int8_t)(data >> 8))
			temp = (int8_t)(data >> 8) - 1;
		/* Write both MAX and MIN ALERT */
		data = (data & 0xff00) + temp;
		ret = regmap_write(map, MAX17055_TAlrtTh, data);
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = regmap_read(map, MAX17055_TAlrtTh, &data);
		if (ret < 0)
			return ret;

		/* Input in Deci-Centigrade, convert to centigrade */
		temp = val->intval / 10;
		/* force max > min */
		if (temp <= (int8_t)(data & 0xff))
			temp = (int8_t)(data & 0xff) + 1;
		/* Write both MAX and MIN ALERT */
		data = (data & 0xff) + (temp << 8);
		ret = regmap_write(map, MAX17055_TAlrtTh, data);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int max17055_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;

	switch (psp) {
#if CONFIG_HUAWEI_SAWSHARK
	case POWER_SUPPLY_PROP_CAPACITY:
#endif
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static int max17055_wait_on_bits(struct regmap *map, u8 reg, u16 bits)
{
	int wait = 100, data = 0;

	do {
		wait--;
		msleep(10);
		regmap_read(map, reg, &data);
	}while((wait > 0) && (data & bits));

	pr_info("max17055 wait %d, reg 0x%02x val 0x%04x\n", wait, reg, data);

	return wait;
}

static int max17055_write_verify_reg(struct regmap *map, u8 reg, u32 value)
{
	int retries = 8;
	int ret;
	u32 read_value;

	do {
		ret = regmap_write(map, reg, value);
		regmap_read(map, reg, &read_value);
		if (read_value != value) {
			ret = -EIO;
			retries--;
		}
	} while (retries && read_value != value);

	if (ret < 0)
		pr_err("%s: reg 0x%02x err %d\n", __func__, reg, ret);

	return ret;
}

static int max17055_write_volatile_reg(struct regmap *map, u8 reg, u32 value)
{
	int retries = 8;
	int ret;
	u32 read_value;

	do {
		ret = regmap_write(map, reg, value);
		mdelay(5);
		regmap_read(map, reg, &read_value);
		if (read_value != value) {
			ret = -EIO;
			retries--;
		}
	} while (retries && read_value != value);

	if (ret < 0)
		pr_err("%s: reg 0x%02x err %d\n", __func__, reg, ret);

	return ret;
}

static inline void max17055_override_por(struct regmap *map,
					 u8 reg, u16 value)
{
	if (value)
		regmap_write(map, reg, value);
}

static inline void max10742_unlock_model(struct max17055_chip *chip)
{
	struct regmap *map = chip->regmap;

	regmap_write(map, MAX17055_MLockReg1, MODEL_UNLOCK1);
	regmap_write(map, MAX17055_MLockReg2, MODEL_UNLOCK2);
}

static inline void max10742_lock_model(struct max17055_chip *chip)
{
	struct regmap *map = chip->regmap;

	regmap_write(map, MAX17055_MLockReg1, MODEL_LOCK1);
	regmap_write(map, MAX17055_MLockReg2, MODEL_LOCK2);
}

static inline void max17055_write_model_data(struct max17055_chip *chip,
					u8 addr, int size)
{
	struct regmap *map = chip->regmap;
	int i;

	for (i = 0; i < size; i++)
		regmap_write(map, addr + i,
			chip->pdata->config_data->cell_char_tbl[i]);
}

static inline void max17055_read_model_data(struct max17055_chip *chip,
					u8 addr, u32 *data, int size)
{
	struct regmap *map = chip->regmap;
	int i;

	for (i = 0; i < size; i++)
		regmap_read(map, addr + i, &data[i]);
}

static inline int max17055_model_data_compare(struct max17055_chip *chip,
					u16 *data1, u32 *data2, int size)
{
	int result = 0, i;

	for (i = 0; i < size; i++) {
		if (data1[i] != data2[i]){
			result = -EINVAL;
			break;
		}
	}

	if (result) {
		dev_err(&chip->client->dev, "%s compare failed\n", __func__);
		for (i = 0; i < size; i++)
			dev_info(&chip->client->dev, "0x%x, 0x%x",
				data1[i], data2[i]);
		dev_info(&chip->client->dev, "\n");
	}

	return result;
}

static int max17055_init_model(struct max17055_chip *chip)
{
	int ret;
	int table_size = ARRAY_SIZE(chip->pdata->config_data->cell_char_tbl);
	u32 *temp_data;

	temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
	if (!temp_data)
		return -ENOMEM;

	max10742_unlock_model(chip);
	max17055_write_model_data(chip, MAX17055_ModelChrTbl,
				table_size);
	max17055_read_model_data(chip, MAX17055_ModelChrTbl, temp_data,
				table_size);

	ret = max17055_model_data_compare(
		chip,
		chip->pdata->config_data->cell_char_tbl,
		temp_data,
		table_size);

	max10742_lock_model(chip);
	kfree(temp_data);

	return ret;
}

static int max17055_verify_model_lock(struct max17055_chip *chip)
{
	int i;
	int table_size = ARRAY_SIZE(chip->pdata->config_data->cell_char_tbl);
	u32 *temp_data;
	int ret = 0;

	temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
	if (!temp_data)
		return -ENOMEM;

	max17055_read_model_data(chip, MAX17055_ModelChrTbl, temp_data,
				table_size);
	for (i = 0; i < table_size; i++)
		if (temp_data[i])
			ret = -EINVAL;

	kfree(temp_data);
	return ret;
}

static void  max17055_write_custom_regs(struct max17055_chip *chip)
{
	struct max17055_config_data *config = chip->pdata->config_data;
	struct regmap *map = chip->regmap;

	max17055_write_verify_reg(map, MAX17055_RComp0, config->rcomp0);
	max17055_write_verify_reg(map, MAX17055_TempCo,	config->tcompc0);
	max17055_write_verify_reg(map, MAX17055_QRTbl00, config->qrtbl00);
	max17055_write_verify_reg(map, MAX17055_QRTbl10, config->qrtbl10);

	/* In the INI file QTable20 and QTable30 are optional */
	max17055_override_por(map, MAX17055_QRTbl20, config->qrtbl20);
	max17055_override_por(map, MAX17055_QRTbl30, config->qrtbl30);
}

static void max17055_update_model_regs(struct max17055_chip *chip)
{
	struct max17055_config_data *config = chip->pdata->config_data;
	struct regmap *map = chip->regmap;
	u32 vfSoc, dq_acc = config->design_cap / 16;

	max17055_write_verify_reg(map, MAX17055_DesignCap, config->design_cap);
	max17055_write_verify_reg(map, MAX17055_dQacc, dq_acc);
	max17055_write_verify_reg(map, MAX17055_dPacc, dPaccVals[1][1]);
	max17055_write_verify_reg(map, MAX17055_IChgTerm, config->ichgt_term);
	max17055_write_verify_reg(map, MAX17055_VEmpty, config->vempty);

	max17055_write_custom_regs(chip);

	max17055_write_volatile_reg(map, MAX17055_RepCap, 0);

	/* Update VFSOC */
	regmap_read(map, MAX17055_VFSOC, &vfSoc);
	regmap_write(map, MAX17055_Command, VFSOC0_UNLOCK);
	max17055_write_verify_reg(map, MAX17055_VFSOC0, vfSoc);
	regmap_write(map, MAX17055_Command, VFSOC0_LOCK);

	max17055_write_verify_reg(map, MAX17055_FullCapRep, config->design_cap);
	max17055_write_verify_reg(map, MAX17055_FullCapNom, config->design_cap);

	/* LearnCfg is optional */
	max17055_override_por(map, MAX17055_LearnCfg, config->learn_cfg);
}

/*
 * Block write all the override values coming from platform data.
 * This function MUST be called before the POR initialization proceedure
 * specified by maxim.
 */
static inline void max17055_override_por_values(struct max17055_chip *chip)
{
	struct regmap *map = chip->regmap;
	struct max17055_config_data *config = chip->pdata->config_data;

	max17055_override_por(map, MAX17055_TGain, config->tgain);
	max17055_override_por(map, MAx17055_TOff, config->toff);
	max17055_override_por(map, MAX17055_CGain, config->cgain);
	max17055_override_por(map, MAX17055_COff, config->coff);
	max17055_override_por(map, MAX17055_TCurve, config->tcurve);

	max17055_override_por(map, MAX17055_IAlrtTh, config->ialrt_thresh);
	max17055_override_por(map, MAX17055_VAlrtTh, config->valrt_thresh);
	max17055_override_por(map, MAX17055_TAlrtTh, config->talrt_thresh);
	max17055_override_por(map, MAX17055_SAlrtTh, config->salrt_thresh);
	max17055_override_por(map, MAX17055_Config, config->config);
	max17055_override_por(map, MAX17055_Config2, config->config2);
	max17055_override_por(map, MAX17055_ShdnTimer, config->shdntimer);

	max17055_override_por(map, MAX17055_AtRate, config->at_rate);
	max17055_override_por(map, MAX17055_FilterCfg, config->filter_cfg);
	max17055_override_por(map, MAX17055_RelaxCfg, config->relax_cfg);
	max17055_override_por(map, MAX17055_MiscCfg, config->misc_cfg);
	max17055_override_por(map, MAX17055_HibCfg, config->hib_cfg);

	max17055_override_por(map, MAX17055_FullSocThr, config->full_soc_thresh);
	max17055_override_por(map, MAX17055_IAvgEmpty, config->iavg_empty);

	/* Bring chip out of hibernate mode */
	regmap_write(map, MAX17055_Command, 0x0090);
	regmap_update_bits(map, MAX17055_HibCfg, HIBCFG_ENHIB_BIT, 0);
	regmap_write(map, MAX17055_Command, 0);
}

static void max17055_config_simple(struct max17055_chip *chip, bool with_ini)
{
	struct regmap *map = chip->regmap;
	struct max17055_config_data *config = chip->pdata->config_data;
	int dQacc_shrink = with_ini ? 16 : 128;
	bool bat_4v275 = !!(config->model_cfg & 0x400);

	if (MAXIM_DEVICE_TYPE_MAX17055A == chip->chip_type)
		max17055_write_volatile_reg(map, MAX17055_RepCap, 0);

	max17055_write_verify_reg(map, MAX17055_DesignCap, config->design_cap);
	max17055_write_verify_reg(map, MAX17055_dQacc, config->design_cap / dQacc_shrink);
	max17055_write_verify_reg(map, MAX17055_IChgTerm, config->ichgt_term);
	max17055_write_verify_reg(map, MAX17055_VEmpty, config->vempty);

	/* LearnCfg is optional */
	max17055_override_por(map, MAX17055_LearnCfg, config->learn_cfg);
	max17055_write_verify_reg(map, MAX17055_dPacc, dPaccVals[with_ini][bat_4v275]);
	regmap_write(map, MAX17055_ModelCfg, config->model_cfg);

	/*waiting for model loading to be complete*/
	max17055_wait_on_bits(map, MAX17055_ModelCfg, 0x8000);

	if (with_ini)
		max17055_write_custom_regs(chip);
}

static int max17055_init_chip(struct max17055_chip *chip)
{
	struct regmap *map = chip->regmap;
	int chip_config = chip->pdata->config_type;
	int ret;

	pr_info("%s Config chip with option %d\n", __func__, chip_config);

	max17055_override_por_values(chip);

	/* After Power up, waiting for FStat.DNR to be cleared*/
	max17055_wait_on_bits(map, MAX17055_FStat, 0x0001);

	switch (chip_config){
	case MODELGAUGE_CONFIG_TYPE_EZ:
		max17055_config_simple(chip, false);
		break;

	case MODELGAUGE_CONFIG_TYPE_SHORT:
		max17055_config_simple(chip, true);
		break;

	case MODELGAUGE_CONFIG_TYPE_FULL:
		/* write cell characterization data */
		ret = max17055_init_model(chip);
		if (ret) {
			dev_err(&chip->client->dev, "%s init failed\n",
				__func__);
			return -EIO;
		}

		ret = max17055_verify_model_lock(chip);
		if (ret) {
			dev_err(&chip->client->dev, "%s lock verify failed\n",
				__func__);
			return -EIO;
		}

		/* update all required registers for model loading */
		max17055_update_model_regs(chip);

		/* Fire Model Loading Command */
		regmap_update_bits(map, MAX17055_Config2, CONFIG2_LDMDL_BIT, CONFIG2_LDMDL_BIT);

		max17055_wait_on_bits(map, MAX17055_Config2, CONFIG2_LDMDL_BIT);

		break;

	default: /* Do nothing */

		break;
	}

	/* Enable Hibernate */
	regmap_update_bits(map, MAX17055_HibCfg, HIBCFG_ENHIB_BIT, HIBCFG_ENHIB_BIT);

	/* Store driver revision number */
	regmap_write(map, MAX17055_UserMem1, MAX17055_DRIVER_VERSION);

	/* Init complete, Clear the POR bit */
	regmap_update_bits(map, MAX17055_Status, STATUS_POR_BIT, 0x0);
	return 0;
}

static irqreturn_t max17055_thread_handler(int id, void *dev)
{
	struct max17055_chip *chip = dev;
	u32 val = 0;

	regmap_read(chip->regmap, MAX17055_Status, &val);
	if (val){
		if (val & STATUS_BST_BIT) {
			dev_info(&chip->client->dev, "Battery absent\n");
		}

		if (val & STATUS_SOCI_BIT) {
			dev_info(&chip->client->dev, "Capacity change INTR\n");
		}

		if (val & STATUS_BI_BIT) {
			dev_info(&chip->client->dev, "Battery inserted\n");
		}

		if (val & STATUS_BR_BIT) {
			dev_info(&chip->client->dev, "Battery removed\n");
		}

		/*Clear the alert bits*/
		regmap_update_bits(chip->regmap, MAX17055_Status, STATUS_BST_BIT | \
			STATUS_SOCI_BIT | STATUS_BI_BIT | STATUS_BR_BIT, 0);

		power_supply_changed(&chip->battery);
	}
	return IRQ_HANDLED;
}

static void max17055_init_worker(struct work_struct *work)
{
	struct max17055_chip *chip = container_of(work,
				struct max17055_chip, work);
	int ret;

	/* Initialize registers according to values from the platform data */
	if (chip->pdata->enable_por_init && chip->pdata->config_data) {
		ret = max17055_init_chip(chip);
		if (ret)
			return;
	}

	chip->init_complete = 1;
}

#ifdef CONFIG_OF
static struct max17055_platform_data *
max17055_get_pdata(struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 prop;
	struct max17055_platform_data *pdata;
#if CONFIG_HUAWEI_SAWSHARK
	int rc;
#endif
	if (!np)
		return dev->platform_data;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;
#if CONFIG_HUAWEI_SAWSHARK
	rc = of_property_read_string(np, "qcom,chg-psy-name",
                            &pdata->chg_psy_name);
	if (rc)
	{
		pdata->chg_psy_name = NULL;
	}
#endif
	/*
	 * Require current sense resistor value to be specified for
	 * current-sense functionality to be enabled at all.
	 */
	if (of_property_read_u32(np, "maxim,rsns-microohm", &prop) == 0) {
		pdata->r_sns = prop;
		pdata->enable_current_sense = true;
	}

	if (of_property_read_u32(np, "maxim,config-type", &pdata->config_type))
		pdata->config_type = MODELGAUGE_CONFIG_TYPE_EZ;
	if (of_property_read_u32(np, "maxim,cold-temp", &pdata->temp_min))
		pdata->temp_min = INT_MIN;
	if (of_property_read_u32(np, "maxim,over-heat-temp", &pdata->temp_max))
		pdata->temp_max = INT_MAX;
	if (of_property_read_u32(np, "maxim,dead-volt", &pdata->vmin))
		pdata->vmin = INT_MIN;
	if (of_property_read_u32(np, "maxim,over-volt", &pdata->vmax))
		pdata->vmax = INT_MAX;

	if (of_property_read_bool(np, "maxim,enable-por-init"))
		pdata->enable_por_init = true;

	pdata->config_data = devm_kzalloc(dev, sizeof(*(pdata->config_data)), GFP_KERNEL);
	if (pdata->config_data)
	{
		of_property_read_u16_array(np, "maxim,config-data",
			(u16 *)pdata->config_data, sizeof(*(pdata->config_data))/sizeof(u16));
	}

	return pdata;
}
#else
static struct max17055_platform_data *
max17055_get_pdata(struct device *dev)
{
	return dev->platform_data;
}
#endif

static const struct regmap_config max17055_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = MAX17055_VFSOC,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

#define VALUE_MAX_LENGTH (20)

static ssize_t max17055_data_logging_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return 0;
}

static ssize_t max17055_data_logging_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        struct max17055_chip *chip = dev_get_drvdata(dev);
		struct timespec now;
		u32 reg, data;
		ssize_t total = 0, size;

		/* Logging time */
		now = current_kernel_time();;
		size = snprintf(buf, VALUE_MAX_LENGTH, "%li ", now.tv_sec);
		buf += size;
		total += size;

		/* Logging registers */
		for (reg = MAX17055_Status; reg <= MAX17055_VFSOC; reg++){
			regmap_read(chip->regmap, reg, &data);
			size = snprintf(buf, VALUE_MAX_LENGTH, "%04x ", data);

			buf += size;
			total += size;

			/* ignore some registers */
			if (reg == 0x4f) reg += 0x60;
			if (reg == 0xbf) reg += 0x10;
		}

		total += snprintf(buf, VALUE_MAX_LENGTH, "\n");

        return total;
}

static struct device_attribute max17055_data_logging_attr =
        __ATTR(data_logging, S_IRUGO | S_IWUSR | S_IWGRP,
                        max17055_data_logging_show,
                        max17055_data_logging_store);

static int max17055_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17055_chip *chip;
	int ret;
	int i;
	u32 val, driver_version;


	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->regmap = devm_regmap_init_i2c(client, &max17055_regmap_config);
	if (IS_ERR(chip->regmap)) {
		dev_err(&client->dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	chip->pdata = max17055_get_pdata(&client->dev);
	if (!chip->pdata) {
		dev_err(&client->dev, "no platform data provided\n");
		return -EINVAL;
	}

	i2c_set_clientdata(client, chip);

	regmap_read(chip->regmap, MAX17055_DevName, &val);
	if (val == MAX17055_IC_VERSION_A) {
		dev_info(&client->dev, "chip type max17055A detected\n");
		chip->chip_type = MAXIM_DEVICE_TYPE_MAX17055A;
	}
	else if (val == MAX17055_IC_VERSION_B){
		dev_info(&client->dev, "chip type max17055B detected\n");
		chip->chip_type = MAXIM_DEVICE_TYPE_MAX17055B;
	}
	else {
		dev_err(&client->dev, "device version mismatch: %x\n", val);
		return -EIO;
	}

#if CONFIG_HUAWEI_SAWSHARK
	/* Enable Sochold */
	regmap_update_bits(chip->regmap, MAX17055_SOCHOLD, SOCHOLD_ENHOLD_BIT, SOCHOLD_ENHOLD_BIT);
#endif

	chip->battery.name		= "bms";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17055_get_property;
	chip->battery.set_property	= max17055_set_property;
	chip->battery.property_is_writeable = max17055_property_is_writeable;
	chip->battery.properties	= max17055_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17055_battery_props);

	/* When current is not measured,
	 * CURRENT_NOW and CURRENT_AVG properties should be invisible. */
	if (!chip->pdata->enable_current_sense)
		chip->battery.num_properties -= 2;

	if (chip->pdata->r_sns == 0)
		chip->pdata->r_sns = MAX17055_DEFAULT_SNS_RESISTOR;

	if (chip->pdata->init_data)
		for (i = 0; i < chip->pdata->num_init_data; i++)
			regmap_write(chip->regmap,
					chip->pdata->init_data[i].addr,
					chip->pdata->init_data[i].data);

	if (!chip->pdata->enable_current_sense) {
		regmap_write(chip->regmap, MAX17055_CGain, 0x0000);
		regmap_write(chip->regmap, MAX17055_MiscCfg, 0x0003);
		regmap_write(chip->regmap, MAX17055_LearnCfg, 0x0007);
	}

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		return ret;
	}

	regmap_read(chip->regmap, MAX17055_Status, &val);

	if (client->irq) {
		/* Clear interrupt state bits but left POR */
		regmap_write(chip->regmap, MAX17055_Status, val & STATUS_POR_BIT);

		ret = request_threaded_irq(client->irq, NULL,
					max17055_thread_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					chip->battery.name, chip);
		if (ret){
			client->irq = 0;
			dev_err(&client->dev, "%s(): cannot get IRQ\n",
				__func__);
		}
	}

	INIT_WORK(&chip->work, max17055_init_worker);

	regmap_read(chip->regmap, MAX17055_UserMem1, &driver_version);
	if (val & STATUS_POR_BIT){
		schedule_work(&chip->work);
	}
	else if (MAX17055_DRIVER_VERSION != driver_version) {
		schedule_work(&chip->work);
	}
	else {
		chip->init_complete = 1;
	}

#if CONFIG_HUAWEI_SAWSHARK
	global_max17055 = chip;
#endif

	ret = device_create_file(&client->dev, &max17055_data_logging_attr);
	if (ret) {
			dev_err(&client->dev, "failed: data logging sysfs register\n");
			return ret;
	}

	return 0;
}

static int max17055_remove(struct i2c_client *client)
{
	struct max17055_chip *chip = i2c_get_clientdata(client);

	if (client->irq)
		free_irq(client->irq, chip);

	if (chip->pdata && chip->pdata->config_data){
		devm_kfree(&client->dev, chip->pdata->config_data);
		devm_kfree(&client->dev, chip->pdata);
	}

	power_supply_unregister(&chip->battery);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max17055_suspend(struct device *dev)
{
	struct max17055_chip *chip = dev_get_drvdata(dev);

	/*
	 * disable the irq and enable irq_wake
	 * capability to the interrupt line.
	 */
	if (chip->client->irq) {
		disable_irq(chip->client->irq);
		enable_irq_wake(chip->client->irq);
	}

	return 0;
}

static int max17055_resume(struct device *dev)
{
	struct max17055_chip *chip = dev_get_drvdata(dev);

	if (chip->client->irq) {
		disable_irq_wake(chip->client->irq);
		enable_irq(chip->client->irq);
	}

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(max17055_pm_ops, max17055_suspend,
			max17055_resume);

#ifdef CONFIG_OF
static const struct of_device_id max17055_dt_match[] = {
	{ .compatible = "maxim,max17055" },
	{ },
};
MODULE_DEVICE_TABLE(of, max17055_dt_match);
#endif

static const struct i2c_device_id max17055_id[] = {
	{ "max17055", MAXIM_DEVICE_TYPE_MAX17055A },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17055_id);

static struct i2c_driver max17055_i2c_driver = {
	.driver	= {
		.name	= "max17055",
		.of_match_table = of_match_ptr(max17055_dt_match),
		.pm	= &max17055_pm_ops,
	},
	.probe		= max17055_probe,
	.remove		= max17055_remove,
	.id_table	= max17055_id,
};
module_i2c_driver(max17055_i2c_driver);

MODULE_AUTHOR("Bo Yang <bo.yang@maximintegrated.com>");
MODULE_DESCRIPTION("MAX17055 Fuel Gauge");
MODULE_LICENSE("GPL");
