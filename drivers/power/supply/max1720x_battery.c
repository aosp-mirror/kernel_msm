/*
 * Fuel gauge driver for Maxim 17201/17205
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

#define pr_fmt(fmt) KBUILD_MODNAME ":%s " fmt, __func__

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/time.h>

#define MAX1720X_TRECALL_MS 5
#define MAX1720X_TPOR_MS 150
#define MAX1720X_TNV_WRITE_MS 1000
#define MAX1720X_TBLOCK_MS 1000
#define MAX170X_I2C_DRIVER_NAME "max1720x_fg_irq"

enum max1720x_register {
	/* ModelGauge m5 Register */
	MAX1720X_Status = 0x00,
	MAX1720X_VAlrtTh = 0x01,
	MAX1720X_TAlrtTh = 0x02,
	MAX1720X_SAlrtTh = 0x03,
	MAX1720X_AtRate = 0x04,
	MAX1720X_RepCap = 0x05,
	MAX1720X_RepSOC = 0x06,
	MAX1720X_Age = 0x07,
	MAX1720X_Temp = 0x08,
	MAX1720X_VCell = 0x09,
	MAX1720X_Current = 0x0A,
	MAX1720X_AvgCurrent = 0x0B,
	MAX1720X_QResidual = 0x0C,
	MAX1720X_MixSOC = 0x0D,
	MAX1720X_AvSOC = 0x0E,
	MAX1720X_MixCap = 0x0F,

	MAX1720X_FullCAP = 0x10,
	MAX1720X_TTE = 0x11,
	MAX1720X_QRTable00 = 0x12,
	MAX1720X_FullSocThr = 0x13,
	MAX1720X_RCell = 0x14,
	MAX1720X_RFast = 0x15,
	MAX1720X_AvgTA = 0x16,
	MAX1720X_Cycles = 0x17,
	MAX1720X_DesignCap = 0x18,
	MAX1720X_AvgVCell = 0x19,
	MAX1720X_MaxMinTemp = 0x1A,
	MAX1720X_MaxMinVolt = 0x1B,
	MAX1720X_MaxMinCurr = 0x1C,
	MAX1720X_Config = 0x1D,
	MAX1720X_IChgTerm = 0x1E,
	MAX1720X_AvCap = 0x1F,

	MAX1720X_TTF = 0x20,
	MAX1720X_DevName = 0x21,
	MAX1720X_QRTable10 = 0x22,
	MAX1720X_FullCAPNom = 0x23,
	MAX1720X_AIN0 = 0x27,
	MAX1720X_LearnCfg = 0x28,
	MAX1720X_FilterCfg = 0x29,
	MAX1720X_RelaxCfg = 0x2A,
	MAX1720X_MiscCfg = 0x2B,
	MAX1720X_TGain = 0x2C,
	Max1720x_TOff = 0x2D,
	MAX1720X_CGain = 0x2E,
	MAX1720X_COff = 0x2F,

	MAX1720X_QRTable20 = 0x32,
	MAX1720X_FullCapRep = 0x35,
	MAX1720X_IAvgEmpty = 0x36,
	MAX1720X_RComp0 = 0x38,
	MAX1720X_TempCo = 0x39,
	MAX1720X_VEmpty = 0x3A,
	MAX1720X_FStat = 0x3D,
	MAX1720X_Timer = 0x3E,
	MAX1720X_ShdnTimer = 0x3F,

	MAX1720X_QRTable30 = 0x42,
	MAX1720X_dQAcc = 0x45,
	MAX1720X_dPAcc = 0x46,
	MAX1720X_VFRemCap = 0x4A,
	MAX1720X_QH = 0x4D,

	MAX1720X_Status2 = 0xB0,
	MAX1720X_IAlrtTh = 0xB4,
	MAX1720X_VShdnCfg = 0xB8,
	MAX1720X_AgeForecast = 0xB9,
	MAX1720X_HibCfg = 0xBA,
	MAX1720X_Config2 = 0xBB,
	MAX1720X_VRipple = 0xBC,
	MAX1720X_PackCfg = 0xBD,
	MAX1720X_TimerH = 0xBE,

	MAX1720X_AvgCell4 = 0xD1,
	MAX1720X_AvgCell3 = 0xD2,
	MAX1720X_AvgCell2 = 0xD3,
	MAX1720X_AvgCell1 = 0xD4,
	MAX1720X_Cell4 = 0xD5,
	MAX1720X_Cell3 = 0xD6,
	MAX1720X_Cell2 = 0xD7,
	MAX1720X_Cell1 = 0xD8,
	MAX1720X_CellX = 0xD9,
	MAX1720X_Batt = 0xDA,
	MAX1720X_AtQResidual = 0xDC,
	MAX1720X_AtTTE = 0xDD,
	MAX1720X_AtAvSOC = 0xDE,
	MAX1720X_AtAvCap = 0xDF,

	/* Individual Registers */
	MAX1720X_COMMAND = 0x60,
	MAX1720X_CommStat = 0x61,
	MAX1720X_Lock = 0x7F,
	MAX1720X_ODSCTh = 0xF2,
	MAX1720X_ODSCCfg = 0xF3,
	MAX1720X_VFOCV = 0xFB,
	MAX1720X_VFSOC = 0xFF,
};

enum MAX1720X_Status_bits {
	MAX1720X_STATUS_POR = BIT(1),
	MAX1720X_STATUS_IMN = BIT(2),
	MAX1720X_STATUS_BST = BIT(3),
	MAX1720X_STATUS_IMX = BIT(6),
	MAX1720X_STATUS_DSOCI = BIT(7),
	MAX1720X_STATUS_VMN = BIT(8),
	MAX1720X_STATUS_TMN = BIT(9),
	MAX1720X_STATUS_SMN = BIT(10),
	MAX1720X_STATUS_BI = BIT(11),
	MAX1720X_STATUS_VMX = BIT(12),
	MAX1720X_STATUS_TMX = BIT(13),
	MAX1720X_STATUS_SMX = BIT(14),
	MAX1720X_STATUS_BR = BIT(15),
};

enum MAX1720X_CommStat_bits {
	MAX1720X_COMMSTAT_NVBUSY = BIT(1),
	MAX1720X_COMMSTAT_NVERROR = BIT(2),
};

enum MAX1720X_Config_bits {
	MAX1720X_CONFIG_BER = BIT(0),
	MAX1720X_CONFIG_BEI = BIT(1),
	MAX1720X_CONFIG_AEN = BIT(2),
	MAX1720X_CONFIG_FTHRM = BIT(3),
	MAX1720X_CONFIG_ETHRM = BIT(4),
	MAX1720X_CONFIG_COMMSH = BIT(6),
	MAX1720X_CONFIG_SHDN = BIT(7),
	MAX1720X_CONFIG_TEX = BIT(8),
	MAX1720X_CONFIG_TEN = BIT(9),
	MAX1720X_CONFIG_AINSH = BIT(10),
	MAX1720X_CONFIG_ALRTP = BIT(11),
	MAX1720X_CONFIG_VS = BIT(12),
	MAX1720X_CONFIG_TS = BIT(13),
	MAX1720X_CONFIG_SS = BIT(14),
};

enum MAX1720X_COMMAND_bits {
	MAX1720X_COMMAND_FUEL_GAUGE_RESET = 0x0001,
	MAX1720X_COMMAND_HARDWARE_RESET = 0x000F,
	MAX1720X_COMMAND_QUERY_REMAINING_UPDATES = 0xE2FA,
	MAX1720X_COMMAND_COPY_NV_BLOCK = 0xE904,
};

/** NV RAM is never manipulated directly but rather shadow RAM.
 * We use NVRAM in this driver also code is reading/writing from/to shadow RAM.
 * max1720x_copy_nv_block will copy shadow RAM content to NV RAM.
 */
enum max1720x_nvram {
	MAX1720X_NVRAM_START = 0x80,
	MAX1720X_NVRAM_LEARNCFG = 0x9F,
	MAX1720X_NVRAM_QRTABLE00 = 0xA0,
	MAX1720X_NVRAM_CONFIG = 0xB0,
	MAX1720X_NVRAM_PACKCFG = 0xB5,
	MAX1720X_NVRAM_NVCFG0 = 0xB8,
	MAX1720X_NVRAM_SBSCFG = 0xBB,
	MAX1720X_NVRAM_CGAIN = 0xC8,
	MAX1720X_NVRAM_RSENSE = 0xCF,
	MAX1720X_NVRAM_DEVICENAME4 = 0xDF,
	MAX1720X_NVRAM_END = 0xE0,
	MAX1720X_NVRAM_REMAINING_UPDATES = 0xED,
	MAX1720X_NVRAM_HISTORY_END = 0xF0,
};
#define NVRAM_SHORT_INDEX(reg) (reg - MAX1720X_NVRAM_START)
#define NVRAM_BYTE_INDEX(reg) (NVRAM_SHORT_INDEX(reg) * sizeof(short))
#define NVRAM_BYTE_INDEX_FROM(reg, base) ((reg - base) * sizeof(short))
#define MAX1720X_NVRAM_SHORT_SIZE NVRAM_SHORT_INDEX(MAX1720X_NVRAM_END)
#define MAX1720X_NVRAM_SIZE NVRAM_BYTE_INDEX(MAX1720X_NVRAM_END)

#define MAX1720X_NPACKCFG_IDX NVRAM_SHORT_INDEX(MAX1720X_NVRAM_PACKCFG)
#define MAX1720X_NVCFG0_IDX NVRAM_SHORT_INDEX(MAX1720X_NVRAM_NVCFG0)

struct max1720x_chip {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct work_struct work;
	struct i2c_client *primary;
	struct i2c_client *secondary;
	struct regmap *regmap_nvram;
	u16 dt_nvRAM_cfg[MAX1720X_NVRAM_SHORT_SIZE];
	u16 RSense;
	u16 RConfig;
	bool init_complete;
};

#define REGMAP_READ(regmap, what, dst)				\
	do {							\
		unsigned int reg;				\
		int rtn;					\
		rtn = regmap_read(regmap, what, &reg);		\
		if (rtn) {					\
			pr_err("Failed to read %s\n", #what);	\
			dst = -1;				\
		} else						\
			dst = reg;				\
	} while (0)

#define REGMAP_WRITE(regmap, what, data)			\
	do {							\
		int rtn;					\
		rtn = regmap_write(regmap, what, data);		\
		if (rtn) {					\
			pr_err("Failed to write %s\n", #what);	\
		}						\
	} while (0)

bool max1720x_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00 ... 0x4F:
	case 0xB0 ... 0xDF:
	case MAX1720X_COMMAND:
	case MAX1720X_CommStat:
	case MAX1720X_Lock:
	case MAX1720X_ODSCTh:
	case MAX1720X_ODSCCfg:
	case MAX1720X_VFOCV:
	case MAX1720X_VFSOC:
		return true;
	}
	return false;
}

static const struct regmap_config max1720x_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX1720X_VFSOC,
	.readable_reg = max1720x_is_reg,
	.volatile_reg = max1720x_is_reg,

};

bool max1720x_is_nvram_reg(struct device *dev, unsigned int reg)
{
	return (reg >= MAX1720X_NVRAM_START
		&& reg <= MAX1720X_NVRAM_HISTORY_END);
}

static const struct regmap_config max1720x_regmap_nvram_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX1720X_NVRAM_HISTORY_END,
	.readable_reg = max1720x_is_nvram_reg,
	.volatile_reg = max1720x_is_nvram_reg,
};

static inline int reg_to_percentage(u16 val)
{
	/* LSB: 1/256% */
	return val >> 8;
}

static inline int reg_to_micro_amp_h(u16 val, u16 rsense)
{
	/* LSB: 5.0μVh/RSENSE ; Rsense LSB is 10μΩ */
	return div_s64((u64) val * 500000, rsense);
}

static inline int reg_to_micro_volt(u16 val)
{
	/* LSB: 0.078125mV */
	return div_u64((u64) val * 78125, 1000);
}

static inline int reg_to_micro_amp(s16 val, u16 rsense)
{
	/* LSB: 1.5625μV/RSENSE ; Rsense LSB is 10μΩ */
	return div_s64((s64) val * 156250, rsense);
}

static inline int reg_to_deci_deg_cel(s16 val)
{
	/* LSB: 1/256°C */
	return div_s64((s64) val * 10, 256);
}

static inline int reg_to_resistance_micro_ohms(s16 val, u16 rsense)
{
	/* LSB: 1/4096 Ohm */
	return div_s64((s64) val * 1000 * rsense, 4096);
}

static inline int reg_to_cycles(s16 val)
{
	/* LSB: 16% of one cycle */
	return val * 16 / 100;
}

static inline int reg_to_seconds(s16 val)
{
	/* LSB: 5.625 seconds */
	return val * 5625 / 1000;
}

static enum power_supply_property max1720x_battery_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_RAW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int max1720x_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct max1720x_chip *chip = power_supply_get_drvdata(psy);
	struct regmap *map = chip->regmap;
	u16 data;

	if (!chip->init_complete)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		REGMAP_READ(map, MAX1720X_RepSOC, data);
		val->intval = reg_to_percentage(data);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_RAW:
		REGMAP_READ(map, MAX1720X_DesignCap, data);
		val->intval = reg_to_percentage(data);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		REGMAP_READ(map, MAX1720X_RepCap, data);
		val->intval = reg_to_micro_amp_h(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		REGMAP_READ(map, MAX1720X_FullCapRep, data);
		val->intval = reg_to_micro_amp_h(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		REGMAP_READ(map, MAX1720X_DesignCap, data);
		val->intval = reg_to_micro_amp_h(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		REGMAP_READ(map, MAX1720X_AvgCurrent, data);
		/* current is positive value when flowing to device */
		val->intval = -reg_to_micro_amp(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		REGMAP_READ(map, MAX1720X_Current, data);
		/* current is positive value when flowing to device */
		val->intval = -reg_to_micro_amp(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		REGMAP_READ(map, MAX1720X_Cycles, data);
		val->intval = reg_to_cycles(data);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		REGMAP_READ(map, MAX1720X_Status, data);
		val->intval = (((u16) data) & MAX1720X_STATUS_BST) ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		REGMAP_READ(map, MAX1720X_RCell, data);
		val->intval = reg_to_resistance_micro_ohms(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		REGMAP_READ(map, MAX1720X_Temp, data);
		val->intval = reg_to_deci_deg_cel(data);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		REGMAP_READ(map, MAX1720X_TTE, data);
		val->intval = reg_to_seconds(data);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		REGMAP_READ(map, MAX1720X_TTF, data);
		val->intval = reg_to_seconds(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		REGMAP_READ(map, MAX1720X_AvgVCell, data);
		val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		REGMAP_READ(map, MAX1720X_MaxMinVolt, data);
		/* LSB: 20mV */
		val->intval = ((data >> 8) & 0xF) * 20000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		REGMAP_READ(map, MAX1720X_MaxMinVolt, data);
		/* LSB: 20mV */
		val->intval = (data & 0xF) * 20000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		REGMAP_READ(map, MAX1720X_VCell, data);
		val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		REGMAP_READ(map, MAX1720X_VFOCV, data);
		val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max1720x_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	return -EINVAL;
}

static int max1720x_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	return 0;
}

/*
 * A full reset restores the ICs to their power-up state the same as if power
 * had been cycled.
 */
static int max1720x_full_reset(struct max1720x_chip *chip)
{
	REGMAP_WRITE(chip->regmap, MAX1720X_COMMAND,
		     MAX1720X_COMMAND_HARDWARE_RESET);

	msleep(MAX1720X_TPOR_MS);

	return 0;
}

/*
 * A fuel gauge reset resets only the fuel gauge operation without resetting IC
 * hardware. This is useful for testing different configurations without writing
 * nonvolatile memory.
 */
static int max1720x_fg_reset(struct max1720x_chip *chip)
{
	REGMAP_WRITE(chip->regmap, MAX1720X_Config2,
		     MAX1720X_COMMAND_FUEL_GAUGE_RESET);

	msleep(MAX1720X_TPOR_MS);

	return 0;
}

static int max1720x_copy_nv_block(struct max1720x_chip *chip)
{
	u16 data;

	REGMAP_READ(chip->regmap, MAX1720X_CommStat, data);
	if (data & MAX1720X_COMMSTAT_NVERROR)
		REGMAP_WRITE(chip->regmap, MAX1720X_CommStat, 0);

	REGMAP_WRITE(chip->regmap, MAX1720X_COMMAND,
		     MAX1720X_COMMAND_COPY_NV_BLOCK);
	msleep(MAX1720X_TBLOCK_MS);
	REGMAP_READ(chip->regmap, MAX1720X_CommStat, data);
	if (data & MAX1720X_COMMSTAT_NVERROR) {
		dev_err(chip->dev, "CommStat.NVError is set\n");
		return -EINVAL;
	}

	return 0;
}

static int max1720x_load_dt_config_to_nvram(struct max1720x_chip *chip)
{
	if (regmap_raw_write(chip->regmap_nvram, MAX1720X_NVRAM_START,
			     chip->dt_nvRAM_cfg, MAX1720X_NVRAM_SIZE)) {
		dev_err(chip->dev, "Failed to write config to shadow RAM\n");
		return -EINVAL;
	}
	return 0;
}

static int max1720x_load_dt_config(struct max1720x_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	const char *data;
	int len;

	data = of_get_property(node, "maxim,fg-config", &len);
	if (!data) {
		dev_err(chip->dev, "No fg config available\n");
		return -ENODATA;
	}

	if (len != MAX1720X_NVRAM_SIZE) {
		dev_err(chip->dev, "invalid config size: %d\n", len);
		return -EINVAL;
	}
	memcpy(chip->dt_nvRAM_cfg, data, len);

	return 0;
}

/** Compare configuration:
 * Return 0 if matching else return 1 and update dst with learned from src.
 */
static int max1720x_compare_config(const u16 *cfg_src, u16 *cfg_dst)
{
	/* Locations A0h to AFh are updated by the ICs each time it learns */
	/* The ROM ID is unique to each IC and cannot be changed by the user. */

	if (memcmp(cfg_src, cfg_dst,
		   NVRAM_BYTE_INDEX(MAX1720X_NVRAM_LEARNCFG + 1)))
		goto update_dst;

	if (memcmp(cfg_src + NVRAM_SHORT_INDEX(MAX1720X_NVRAM_CONFIG),
		   cfg_dst + NVRAM_SHORT_INDEX(MAX1720X_NVRAM_CONFIG),
		   NVRAM_BYTE_INDEX_FROM(MAX1720X_NVRAM_SBSCFG + 1,
					 MAX1720X_NVRAM_CONFIG)))
		goto update_dst;

	return 0;
update_dst:
	memcpy(cfg_dst + NVRAM_SHORT_INDEX(MAX1720X_NVRAM_QRTABLE00),
	       cfg_src + NVRAM_SHORT_INDEX(MAX1720X_NVRAM_QRTABLE00),
	       NVRAM_BYTE_INDEX_FROM(MAX1720X_NVRAM_CONFIG,
				     MAX1720X_NVRAM_QRTABLE00));
	cfg_dst[NVRAM_SHORT_INDEX(MAX1720X_NVRAM_CGAIN)] =
	    cfg_src[NVRAM_SHORT_INDEX(MAX1720X_NVRAM_CGAIN)];

	return 1;
}


static void max1720x_handle_dt_config(struct max1720x_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	u16 data, dt_nPackCfg, nPackCfg;
	bool force_flash_config, force_load_config, config_differ;
	u16 nvRAM_cfg[MAX1720X_NVRAM_SHORT_SIZE];

	force_flash_config =
	    of_property_read_bool(node, "maxim,force-flash-config");
	force_load_config =
	    of_property_read_bool(node, "maxim,force-load-config");

	if (regmap_raw_read(chip->regmap_nvram, MAX1720X_NVRAM_START,
			    nvRAM_cfg, MAX1720X_NVRAM_SIZE)) {
		dev_err(chip->dev, "Failed to read config from shadow RAM\n");
		return;
	}

	config_differ = max1720x_compare_config(nvRAM_cfg, chip->dt_nvRAM_cfg);

	REGMAP_WRITE(chip->regmap, MAX1720X_COMMAND,
		     MAX1720X_COMMAND_QUERY_REMAINING_UPDATES);
	msleep(MAX1720X_TRECALL_MS);
	REGMAP_READ(chip->regmap_nvram, MAX1720X_NVRAM_REMAINING_UPDATES, data);
	dev_info(chip->dev, "Remaining config memory updates: 0x%04x\n", data);

	REGMAP_READ(chip->regmap, MAX1720X_Lock, data);
	dev_info(chip->dev, "NV RAM lock status: 0x%02x\n", data & 0x1F);

	REGMAP_READ(chip->regmap, MAX1720X_PackCfg, nPackCfg);
	dt_nPackCfg = chip->dt_nvRAM_cfg[MAX1720X_NPACKCFG_IDX];

	if ((dt_nPackCfg != nPackCfg) ||
	    (config_differ && force_flash_config)) {
		if (dt_nPackCfg != nPackCfg)
			dev_info(chip->dev, "nPackCfg 0x%04x != DT config 0x%04x\n",
				 nPackCfg, dt_nPackCfg);

		dev_info(chip->dev, "Flashing DT config to NVRAM\n");

		if (max1720x_load_dt_config_to_nvram(chip))
			goto check_force_load_config;

		if (max1720x_copy_nv_block(chip))
			goto check_force_load_config;

		max1720x_full_reset(chip);
	}

check_force_load_config:
	if (force_load_config) {
		dev_info(chip->dev, "Loading DT config to shadow RAM\n");
		max1720x_load_dt_config_to_nvram(chip);
		max1720x_fg_reset(chip);
	}
}

static irqreturn_t max1720x_fg_irq_thread_fn(int irq, void *obj)
{
	struct max1720x_chip *chip = obj;
	u16 fg_status;
	u32 data;
	int rtn;

	if (!chip || irq != chip->primary->irq) {
		WARN_ON_ONCE(1);
		return IRQ_NONE;
	}

	rtn = regmap_read(chip->regmap, MAX1720X_Status, &data);
	if (rtn) {
		pr_err("Failed to read MAX1720X_Status\n");
		return IRQ_NONE;
	}
	fg_status = (u16) data;
	if (fg_status & MAX1720X_STATUS_POR) {
		fg_status &= ~MAX1720X_STATUS_POR;
		pr_debug("POR is set");
	}
	if (fg_status & MAX1720X_STATUS_IMN)
		pr_debug("IMN is set");

	if (fg_status & MAX1720X_STATUS_BST)
		pr_debug("BST is set");

	if (fg_status & MAX1720X_STATUS_IMX)
		pr_debug("IMX is set");

	if (fg_status & MAX1720X_STATUS_DSOCI) {
		fg_status &= ~MAX1720X_STATUS_DSOCI;
		pr_debug("DSOCI is set");
	}
	if (fg_status & MAX1720X_STATUS_VMN) {
		if (chip->RConfig & MAX1720X_CONFIG_VS)
			fg_status &= ~MAX1720X_STATUS_VMN;
		pr_debug("VMN is set");
	}
	if (fg_status & MAX1720X_STATUS_TMN) {
		if (chip->RConfig & MAX1720X_CONFIG_TS)
			fg_status &= ~MAX1720X_STATUS_TMN;
		pr_debug("TMN is set");
	}
	if (fg_status & MAX1720X_STATUS_SMN) {
		if (chip->RConfig & MAX1720X_CONFIG_SS)
			fg_status &= ~MAX1720X_STATUS_SMN;
		pr_debug("SMN is set");
	}
	if (fg_status & MAX1720X_STATUS_BI)
		pr_debug("BI is set");

	if (fg_status & MAX1720X_STATUS_VMX) {
		if (chip->RConfig & MAX1720X_CONFIG_VS)
			fg_status &= ~MAX1720X_STATUS_VMX;
		pr_debug("VMX is set");
	}
	if (fg_status & MAX1720X_STATUS_TMX) {
		if (chip->RConfig & MAX1720X_CONFIG_TS)
			fg_status &= ~MAX1720X_STATUS_TMX;
		pr_debug("TMX is set");
	}
	if (fg_status & MAX1720X_STATUS_SMX) {
		if (chip->RConfig & MAX1720X_CONFIG_SS)
			fg_status &= ~MAX1720X_STATUS_SMX;
		pr_debug("SMX is set");
	}

	if (fg_status & MAX1720X_STATUS_BR)
		pr_debug("BR is set");

	REGMAP_WRITE(chip->regmap, MAX1720X_Status, fg_status);
	if (chip->psy)
		power_supply_changed(chip->psy);

	return IRQ_HANDLED;
}

static void max1720x_init_chip(struct max1720x_chip *chip)
{
	u16 data;

	REGMAP_READ(chip->regmap, MAX1720X_Status, data);
	if (data & MAX1720X_STATUS_BR) {
		dev_info(chip->dev, "Clearing Battery Removal bit\n");
		regmap_update_bits(chip->regmap, MAX1720X_Status,
				   MAX1720X_STATUS_BR, 0x0);
	}
	if (data & MAX1720X_STATUS_BI) {
		dev_info(chip->dev, "Clearing Battery Insertion bit\n");
		regmap_update_bits(chip->regmap, MAX1720X_Status,
				   MAX1720X_STATUS_BI, 0x0);
	}
	if (data & MAX1720X_STATUS_POR) {
		dev_info(chip->dev, "Clearing Power-On Reset bit\n");
		regmap_update_bits(chip->regmap, MAX1720X_Status,
				   MAX1720X_STATUS_POR, 0x0);
	}

	if (max1720x_load_dt_config(chip) == 0)
		max1720x_handle_dt_config(chip);

	REGMAP_READ(chip->regmap_nvram, MAX1720X_NVRAM_RSENSE, chip->RSense);
	dev_info(chip->dev, "RSense value %d micro Ohm\n", chip->RSense * 10);
	REGMAP_READ(chip->regmap, MAX1720X_Config, chip->RConfig);
	dev_info(chip->dev, "Config after POR: 0x%04x\n", chip->RConfig);
	REGMAP_READ(chip->regmap, MAX1720X_IChgTerm, data);
	dev_info(chip->dev, "IChgTerm: %d\n",
		 reg_to_micro_amp(data, chip->RSense));
	REGMAP_READ(chip->regmap, MAX1720X_VEmpty, data);
	dev_info(chip->dev, "VEmpty: VE=%dmV VR=%dmV\n",
		 ((data >> 7) & 0x1ff) * 10, (data & 0x7f) * 40);
}

static void max1720x_init_worker(struct work_struct *work)
{
	struct max1720x_chip *chip =
	    container_of(work, struct max1720x_chip, work);
	int rtn;

	max1720x_init_chip(chip);
	if (chip->primary->irq) {
		rtn = request_threaded_irq(chip->primary->irq, NULL,
					   max1720x_fg_irq_thread_fn,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   MAX170X_I2C_DRIVER_NAME, chip);
		if (rtn != 0) {
			dev_err(chip->dev, "Unable to register IRQ handler\n");
			return;
		}
	}
	chip->init_complete = 1;
}

static struct power_supply_desc max1720x_psy_desc = {
	.name = "maxfg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = max1720x_get_property,
	.set_property = max1720x_set_property,
	.property_is_writeable = max1720x_property_is_writeable,
	.properties = max1720x_battery_props,
	.num_properties = ARRAY_SIZE(max1720x_battery_props),
};

static int max1720x_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct max1720x_chip *chip;
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {
	};
	int ret = 0;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	i2c_set_clientdata(client, chip);
	chip->primary = client;
	chip->secondary = i2c_new_secondary_device(client, "nvram", 0xb);
	if (chip->secondary == NULL) {
		dev_err(dev, "Failed to initialize secondary i2c device\n");
		ret = -EINVAL;
		goto out;
	}
	i2c_set_clientdata(chip->secondary, chip);
	chip->regmap = devm_regmap_init_i2c(client, &max1720x_regmap_cfg);
	if (IS_ERR(chip->regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		ret = -EINVAL;
		goto i2c_unregister;
	}

	chip->regmap_nvram =
	    devm_regmap_init_i2c(chip->secondary, &max1720x_regmap_nvram_cfg);
	if (IS_ERR(chip->regmap_nvram)) {
		dev_err(dev, "Failed to initialize nvram regmap\n");
		ret = -EINVAL;
		goto i2c_unregister;
	}

	if (of_property_read_bool(dev->of_node, "maxim,psy-type-unknown"))
		max1720x_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	psy_cfg.drv_data = chip;
	chip->psy = devm_power_supply_register(dev,
					       &max1720x_psy_desc, &psy_cfg);
	if (IS_ERR(chip->psy)) {
		dev_err(dev, "Couldn't register as power supply\n");
		ret = PTR_ERR(chip->psy);
		goto i2c_unregister;
	}

	INIT_WORK(&chip->work, max1720x_init_worker);
	schedule_work(&chip->work);
	goto out;

i2c_unregister:
	i2c_unregister_device(chip->secondary);
out:
	return ret;
}

static const struct of_device_id max1720x_of_match[] = {
	{ .compatible = "maxim,max1720x"},
	{},
};
MODULE_DEVICE_TABLE(of, max1720x_of_match);

static const struct i2c_device_id max1720x_id[] = {
	{"max1720x", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max1720x_id);

static struct i2c_driver max1720x_i2c_driver = {
	.driver = {
		   .name = "max1720x",
		   .of_match_table = max1720x_of_match,
		   },
	.id_table = max1720x_id,
	.probe = max1720x_probe,
};

module_i2c_driver(max1720x_i2c_driver);
MODULE_AUTHOR("Thierry Strudel <tstrudel@google.com>");
MODULE_DESCRIPTION("MAX17201/MAX17205 Fuel Gauge");
MODULE_LICENSE("GPL");
