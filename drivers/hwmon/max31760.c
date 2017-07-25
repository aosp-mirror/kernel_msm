/* Maxim Integrated MAX31760 Precision Fan-Speed Controller driver
 *
 * Copyright (C) 2017 Google, Inc.
 * Author: muirj
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

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define DRIVER_NAME "max31760"

/*
 * MAX31760 registers.
 * Indentation helps identify how these constants apply:
 *      Register number.
 *          Per-register bit fields.
 *              Values for multi-bit fields.
 */
#define MAX31760_REG_CR1   0x00 /* Control Register 1 */
#define     MAX31760_CR1_TIS    0x01 /* Temperature Index Source */
#define     MAX31760_CR1_MTI    0x02 /* Maximum Temperature Index */
#define     MAX31760_CR1_PPS    0x04 /* PWM Polarity */
#define     MAX31760_CR1_DRV    0x18 /* PWM Frequency */
#define         MAX31760_DRV_33HZ   0x00
#define         MAX31760_DRV_150HZ  0x08
#define         MAX31760_DRV_1500HZ 0x10
#define         MAX31760_DRV_25KHZ  MAX31760_CR1_DRV
#define     MAX31760_CR1_HYST   0x20 /* Lookup Table Hysteresis: 2C or 4C */
#define     MAX31760_CR1_POR    0x40 /* Software Power-On Reset */
#define     MAX31760_CR1_ALTMSK 0x80 /* Alert Mask */
#define MAX31760_REG_CR2   0x01 /* Control Register 2 */
#define     MAX31760_CR2_DFC    0x01 /* Direct Fan Control */
#define     MAX31760_CR2_FSST   0x02 /* Fan Sense Signal Type */
#define     MAX31760_CR2_RDPS   0x04 /* RD Polarity Selection */
#define     MAX31760_CR2_FSEN   0x08 /* FS Input Enable */
#define     MAX31760_CR2_FFMODE 0x10 /* FF Functionality Selection */
#define     MAX31760_CR2_SPEN   0x20 /* Spin-up Enable */
#define     MAX31760_CR2_ALERTS 0x40 /* Alerts Functionality Selection */
#define     MAX31760_CR2_STBY   0x80 /* Standby Mode Enable */
#define MAX31760_REG_CR3   0x02 /* Control Register 3 */
#define     MAX31760_CR3_TACH1E 0x01 /* Tachometer 1 Enable */
#define     MAX31760_CR3_TACH2E 0x02 /* Tachometer 2 Enable */
#define     MAX31760_CR3_PSEN   0x04 /* Pulse Stretch Enable */
#define     MAX31760_CR3_TACHFL 0x08 /* Fan Fail When 100% Duty Cycle Only */
#define     MAX31760_CR3_RAMP   0x30 /* PWM Duty-Cycle Ramp Rate */
#define         MAX31760_RAMP_SLOW 0x00
#define         MAX31760_RAMP_SMED 0x10
#define         MAX31760_RAMP_MEDF 0x20
#define         MAX31760_RAMP_FAST MAX31760_CR3_RAMP
#define     MAX31760_CR3_FF_0   0x40 /* 0 Duty-Cycle Fan-Fail Detection */
#define     MAX31760_CR3_CLR_FF 0x80 /* Clear Fan Fail */
#define MAX31760_REG_FFDC  0x03 /* Fan Fault Duty Cycle */
#define MAX31760_REG_MASK  0x04 /* Alert Mask Register */
#define     MAX31760_MASK_TACH1AM 0x01 /* TACH1 Alarm Mask */
#define     MAX31760_MASK_TACH2AM 0x02 /* TACH2 Alarm Mask */
#define     MAX31760_MASK_ROTAM   0x04 /* Remote Overtemperature Alarm Mask */
#define     MAX31760_MASK_RHAM    0x08 /* Remote High Temperature Alarm Mask */
#define     MAX31760_MASK_LOTAM   0x10 /* Local Overtemperature Alarm Mask */
#define     MAX31760_MASK_LHAM    0x20 /* Local High Temperature Alarm Mask */
#define MAX31760_REG_IFR   0x05 /* Ideality Factor Register */
#define     MAX31760_IFR_MASK 0x3f /* Mask for value of the IFR */
#define MAX31760_REG_RHSH  0x06 /* Remote High Set-point MSB */
#define MAX31760_REG_RHSL  0x07 /* Remote High Set-point LSB */
#define MAX31760_REG_LOTSH 0x08 /* Local Overtemperature Set-point MSB */
#define MAX31760_REG_LOTSL 0x09 /* Local Overtemperature Set-point LSB */
#define MAX31760_REG_ROTSH 0x0a /* Remote Overtemperature Set-point MSB */
#define MAX31760_REG_ROTSL 0x0b /* Remote Overtemperature Set-point LSB */
#define MAX31760_REG_LHSH  0x0c /* Local High Set-point MSB */
#define MAX31760_REG_LHSL  0x0d /* Local High Set-point LSB */
#define MAX31760_REG_TCTH  0x0e /* TACH Count Threshold Register, MSB */
#define MAX31760_REG_TCTL  0x0f /* TACH Count Threshold Register, LSB */
#define MAX31760_REG_USER  0x10 /* 8 bytes General Purpose User Memory */
#define MAX31760_REG_USER0 0x10 /* Custom Control Register USER0 */
#define     MAX31760_USER0_PULSE1 0x07 /* Fan1 Pulses per revolution */
#define     MAX31760_USER0_PULSE2 0x38 /* Fan2 Pulses per revolution */
#define MAX31760_REG_USER1_PWMR 0x11 /* Manual mode PWM value for resume */
#define MAX31760_REG_LUT   0x20 /* 48-Byte Lookup Table (LUT) */
#define     MAX31760_LUT_COUNT 48
#define MAX31760_REG_PWMR  0x50 /* Direct Duty-Cycle Control Register */

#define MAX31760_REG_PWMV  0x51 /* Current PWM Duty-Cycle Register */
#define MAX31760_REG_TC1H  0x52 /* TACH1 Count Register, MSB */
#define MAX31760_REG_TC1L  0x53 /* TACH1 Count Register, LSB */
#define MAX31760_REG_TC2H  0x54 /* TACH2 Count Register, MSB */
#define MAX31760_REG_TC2L  0x55 /* TACH2 Count Register, LSB */
#define MAX31760_REG_RTH   0x56 /* Remote Temperature Reading Register, MSB */
#define MAX31760_REG_RTL   0x57 /* Remote Temperature Reading Register, LSB */
#define MAX31760_REG_LTH   0x58 /* Local Temperature Reading Register, MSB */
#define MAX31760_REG_LTL   0x59 /* Local Temperature Reading Register, LSB */
#define MAX31760_REG_SR	   0x5a /* Status Register */
#define     MAX31760_SR_TACH1A 0x01 /* TACH1 Alarm */
#define     MAX31760_SR_TACH2A 0x02 /* TACH2 Alarm */
#define     MAX31760_SR_ROTA   0x04 /* Remote Overtemperature Alarm */
#define     MAX31760_SR_RHA    0x08 /* Remote High Temperature Alarm */
#define     MAX31760_SR_LOTA   0x10 /* Local Overtemperature Alarm */
#define     MAX31760_SR_LHA    0x20 /* Local High Temperature Alarm */
#define     MAX31760_SR_RDFA   0x40 /* Remote Diode Fault Alarm */
#define     MAX31760_SR_PC     0x80 /* Program Corrupt Bit */

#define MAX31760_REG_EEX   0x5b /* Load EEPROM to RAM; Write RAM to EEPROM */
#define     MAX31760_EEX_LW    0x80 /* Load from or write to EEPROM */
#define     MAX31760_EEX_BLKS  0x1F /* Blocks to load/write */

#define MAX31760_TEMP_MIN_MC -40000   /* Minimum Millicelcius */
#define MAX31760_TEMP_MAX_MC 127875   /* Maximum Millicelcius */
#define MAX31760_TEMP_HIGH_HYST 1000  /* 1C hysteresis on high temp alarms. */
#define MAX31760_TEMP_OVER_HYST 10000 /* 10C hysteresis on over temp alarms. */
#define MAX31760_LUT_HYST_CLEAR 2000  /* LUT hysteresis: bit clear. */
#define MAX31760_LYT_HYST_THRESH 3000 /* LUT hysteresis: store threshold. */
#define MAX31760_LUT_HYST_SET 4000    /* LUT hysteresis: bit set. */

#define MAX31760_NUM_TEMPS 2          /* Number of temperature sensors. */
#define MAX31760_NUM_FANS 2           /* Number of fans. */
#define MAX31760_FAN_PULSES_DEF 2     /* Default number of fan pulses. */
#define MAX31760_FAN_PULSES_MAX 8     /* Maximum number of fan pulses. */
#define MAX31760_PWM_ENABLE_FULL 0    /* pwmX_enable: Set PWM at full power. */
#define MAX31760_PWM_ENABLE_MANUAL 1  /* pwmX_enable: Set manual mode. */
#define MAX31760_PWM_ENABLE_AUTO 2    /* pwmX_enable: Set automatic mode. */

#define MAX31760_LUT_AUTO_ATTRS 3     /* Number of LUT auto-point attributes. */
#define MAX31760_LUT_AUTO_ATTR_COUNT (MAX31760_LUT_COUNT * \
				      MAX31760_LUT_AUTO_ATTRS)
#define MAX31760_LUT_NAME_SIZE 32     /* Fit: pwm1_auto_pointXX_temp_hyst\0. */
#define MAX31760_SUPPLY_NAME_SIZE 12  /* Fit: maxim,fan1\0. */

/*
 * struct max31760_dev_attr - for generated device attributes
 * @sdattr:	Sensor device attribute.
 * @name:	Name of this attribute.
 */
struct max31760_dev_attr {
	struct sensor_device_attribute sdattr;
	char name[MAX31760_LUT_NAME_SIZE];
};

/*
 * struct max31760_fan - fan device data
 * @pulses:      Number of tach pulses per rotation.
 * @enabled:     True when the fan is enabled.
 * @label:       Label if provided in open firmware.
 * @enable_gpio: GPIO that enables this fan.
 * @supply:      Power supply for this fan.
 */
struct max31760_fan {
	const char *label;
	int pulses;
	bool enabled;
	struct gpio_desc *enable_gpio;
	struct regulator *supply;
};

/*
 * struct max31760 - device data
 * @regmap:	Register map.
 * @fan:	Fan data.
 * @vdd_supply: Optional regulator that supplies VDD.
 * @temp_label: Labels for the temperature sensors if provided in open firmware.
 * @lut_dev_attrs:
 *		Device attributes for the temperature to PWM lookup table.
 * @lut_attrs:  Pointers to the struct attribute in each lut_dev_attr.
 * @lut_group:  Attribute group for the LUT attributes.
 * @attr_groups:Sysfs attribute groups for this device.
 */
struct max31760 {
	struct regmap *regmap;
	struct max31760_fan fan[MAX31760_NUM_FANS];
	struct regulator *vdd_supply;
	const char *temp_label[MAX31760_NUM_TEMPS];
	struct max31760_dev_attr lut_dev_attrs[MAX31760_LUT_AUTO_ATTR_COUNT];
	struct attribute *lut_attrs[MAX31760_LUT_AUTO_ATTR_COUNT + 1];
	struct attribute_group lut_group;
	const struct attribute_group *attr_groups[4];
};

static bool max31760_readable_reg(struct device *dev, unsigned int reg)
{
	return reg != MAX31760_REG_EEX;
}

static bool max31760_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX31760_REG_PWMV ... MAX31760_REG_SR:
		return false;
	default:
		return true;
	}
}

static bool max31760_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX31760_REG_MASK:
	case MAX31760_REG_PWMR ... MAX31760_REG_EEX:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config max31760_regmap_config = {
	/*
	 * Device has an EEPROM to store the register values, so don't define
	 * reg_defaults: read the current values from the hardware.
	 */
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX31760_REG_EEX,
	.writeable_reg = max31760_writeable_reg,
	.readable_reg = max31760_readable_reg,
	.volatile_reg = max31760_volatile_reg,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

/* Convert 11-bit MAX31760 register value to milliCelsius */
static inline int max31760_temp_reg_to_mC(s16 val)
{
	return (val & ~0x0f) * 1000 / 256;
}

/* Convert milliCelsius to left adjusted 11-bit MAX31760 register value */
static inline u16 max31760_mC_to_temp_reg(int val)
{
	return (val * 256) / 1000;
}

/* Convert tachometer value to RPM. */
static inline long max31760_rpm_from_tach(u16 tach_count, int pulses)
{
	return 60L * 100000L / (long)tach_count / (long)pulses;
}

/* Convert RPM to tachometer value. */
static inline u16 max31760_tach_from_rpm(long rpm, int pulses)
{
	long tach = 60L * 100000L / rpm / (long)pulses;

	if (tach < 0)
		tach = 0;
	else if (tach > (long)USHRT_MAX)
		tach = USHRT_MAX;

	return tach;
}

/*
 * Read two subsequent registers into a 16-bit word, treating the first as the
 * most significant byte.
 */
static int max31760_read_word(struct regmap *regmap, unsigned int regmsb,
			      u16 *word)
{
	int err;
	unsigned int msb_val;
	unsigned int lsb_val;

	err = regmap_read(regmap, regmsb, &msb_val);
	if (err < 0)
		return err;
	err = regmap_read(regmap, regmsb + 1, &lsb_val);
	if (err < 0)
		return err;

	*word = ((msb_val << 8) & 0xff00) | (lsb_val & 0xff);
	return 0;
}

/*
 * Write a 16-bit word into two subsequent registers, treating the first as the
 * most significant byte.
 */
static int max31760_write_word(struct regmap *regmap, unsigned int regmsb,
			       u16 word)
{
	int err;
	unsigned int val;

	val = (word >> 8) & 0xff;
	err = regmap_write(regmap, regmsb, val);
	if (err < 0)
		return err;

	val = word & 0xff;
	return regmap_write(regmap, regmsb + 1, val);
}

/*
 * Read an alarm which may be flagged in the status register, or masked in the
 * alarm mask register. Reading from the status register will cause the bit in
 * the mask register to be set.
 */
static int max31760_read_alarm(struct device *dev, unsigned int srflag,
			       unsigned int maskflag, long *val)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int srval;
	unsigned int maskval;
	int err;

	err = regmap_read(max31760->regmap, MAX31760_REG_SR, &srval);
	if (err < 0)
		return err;
	err = regmap_read(max31760->regmap, MAX31760_REG_MASK, &maskval);
	if (err < 0)
		return err;

	*val = !!((srval & srflag) | (maskval & maskflag));
	return 0;
}

static int max31760_read_temp(struct device *dev, u32 attr, int channel,
			      long *val)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int reg;
	unsigned int regval;
	unsigned int srflag;
	unsigned int maskflag;
	u16 temp;
	int err;
	int hyst = 0;

	switch (attr) {
	case hwmon_temp_emergency_hyst:
		hyst = MAX31760_TEMP_OVER_HYST;
		/* fallthrough */
	case hwmon_temp_max_hyst:
		if (attr == hwmon_temp_max_hyst)
			hyst = MAX31760_TEMP_HIGH_HYST;
		/* fallthrough */
	case hwmon_temp_input:
	case hwmon_temp_max:
	case hwmon_temp_emergency:
		switch (attr) {
		case hwmon_temp_input:
			reg = channel ? MAX31760_REG_RTH : MAX31760_REG_LTH;
			break;
		case hwmon_temp_max_hyst:
		case hwmon_temp_max:
			reg = channel ? MAX31760_REG_RHSH : MAX31760_REG_LHSH;
			break;
		case hwmon_temp_emergency_hyst:
		case hwmon_temp_emergency:
			reg = channel ? MAX31760_REG_ROTSH : MAX31760_REG_LOTSH;
			break;
		}
		err = max31760_read_word(max31760->regmap, reg, &temp);
		if (err < 0)
			return err;
		*val = max31760_temp_reg_to_mC(temp) - hyst;
		break;
	case hwmon_temp_max_alarm:
	case hwmon_temp_emergency_alarm:
		switch (attr) {
		case hwmon_temp_max_alarm:
			srflag = channel ? MAX31760_SR_RHA : MAX31760_SR_LHA;
			maskflag = channel ? MAX31760_MASK_RHAM :
					     MAX31760_MASK_LHAM;
			break;
		case hwmon_temp_emergency_alarm:
			srflag = channel ? MAX31760_SR_ROTA : MAX31760_SR_LOTA;
			maskflag = channel ? MAX31760_MASK_RHAM :
					     MAX31760_MASK_LHAM;
			break;
		}
		return max31760_read_alarm(dev, srflag, maskflag, val);
	case hwmon_temp_fault:
		err = regmap_read(max31760->regmap, MAX31760_REG_SR, &regval);
		if (err < 0)
			return err;
		*val = !!(regval & MAX31760_SR_RDFA);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int max31760_read_fan(struct device *dev, u32 attr, int channel,
			     long *val)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	u16 tach_count;
	unsigned int reg;
	unsigned int regval;
	unsigned int srflag;
	unsigned int maskflag;
	int err;

	switch (attr) {
	case hwmon_fan_input:
	case hwmon_fan_min:
		switch (attr) {
		case hwmon_fan_input:
			reg = channel ? MAX31760_REG_TC2H : MAX31760_REG_TC1H;
			break;
		case hwmon_fan_min:
			reg = MAX31760_REG_TCTH;
			break;
		}
		err = max31760_read_word(max31760->regmap, reg, &tach_count);
		if (err)
			return err;
		*val = max31760_rpm_from_tach(tach_count,
					      max31760->fan[channel].pulses);
		break;
	case hwmon_fan_min_alarm:
		srflag = channel ? MAX31760_SR_TACH2A : MAX31760_SR_TACH1A;
		maskflag = channel ? MAX31760_MASK_TACH2AM :
				     MAX31760_MASK_TACH1AM;
		return max31760_read_alarm(dev, srflag, maskflag, val);
	case hwmon_fan_pulses:
		err = regmap_read(max31760->regmap, MAX31760_REG_USER0,
				  &regval);
		if (err)
			return err;
		if (channel)
			*val = (regval & MAX31760_USER0_PULSE2) >> 3;
		else
			*val = regval & MAX31760_USER0_PULSE1;
		if (*val == 0)
			*val = MAX31760_FAN_PULSES_DEF;
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int max31760_read_pwm(struct device *dev, u32 attr, int channel,
			     long *val)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int regval;
	int err;

	switch (attr) {
	case hwmon_pwm_input:
		/* Note that this is the current value, not the value stored to
		 * the duty-cycle register.
		 */
		err = regmap_read(max31760->regmap, MAX31760_REG_PWMV, &regval);
		if (err)
			return err;
		*val = regval;
		break;
	case hwmon_pwm_enable:
		err = regmap_read(max31760->regmap, MAX31760_REG_CR2, &regval);
		if (err)
			return err;
		if (regval & MAX31760_CR2_DFC)
			*val = MAX31760_PWM_ENABLE_MANUAL;
		else
			*val = MAX31760_PWM_ENABLE_AUTO;
		break;
	case hwmon_pwm_freq:
		err = regmap_read(max31760->regmap, MAX31760_REG_CR1, &regval);
		if (err)
			return err;
		switch (regval & MAX31760_CR1_DRV) {
		case MAX31760_DRV_33HZ:
		default:
			*val = 33;
			break;
		case MAX31760_DRV_150HZ:
			*val = 150;
			break;
		case MAX31760_DRV_1500HZ:
			*val = 1500;
			break;
		case MAX31760_DRV_25KHZ:
			*val = 25000;
			break;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int max31760_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_temp:
		return max31760_read_temp(dev, attr, channel, val);
	case hwmon_fan:
		return max31760_read_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return max31760_read_pwm(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int max31760_read_string(struct device *dev,
				enum hwmon_sensor_types type, u32 attr,
				int channel, const char **str)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		if (attr != hwmon_temp_label)
			return -EOPNOTSUPP;
		*str = max31760->temp_label[channel];
		break;
	case hwmon_fan:
		if (attr != hwmon_fan_label)
			return -EOPNOTSUPP;
		*str = max31760->fan[channel].label;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/* Write a temperature to the two adjacent registers starting at regmsb. */
static int max31760_write_temp_reg(struct regmap *regmap,
				   unsigned int regmsb, long temp)
{
	u16 word;

	temp = clamp_val(temp, MAX31760_TEMP_MIN_MC, MAX31760_TEMP_MAX_MC);
	word = max31760_mC_to_temp_reg(temp);

	return max31760_write_word(regmap, regmsb, word);
}

static int max31760_write_temp(struct device *dev, u32 attr, int channel,
			       long val)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_temp_max:
		return max31760_write_temp_reg(max31760->regmap,
					       channel ? MAX31760_REG_RHSH :
							 MAX31760_REG_LHSH,
					       val);
	case hwmon_temp_emergency:
		return max31760_write_temp_reg(max31760->regmap,
					       channel ? MAX31760_REG_ROTSH :
							 MAX31760_REG_LOTSH,
					       val);
	default:
		return -EOPNOTSUPP;
	}
}

/* Update the quick access fan pulses value. */
static void max31760_update_fan_pulses(struct max31760 *max31760, int channel,
				       int pulses)
{
	if (pulses > MAX31760_FAN_PULSES_MAX)
		pulses = MAX31760_FAN_PULSES_MAX;
	else if (pulses <= 0)
		pulses = MAX31760_FAN_PULSES_DEF;
	max31760->fan[channel].pulses = pulses;
}

static int max31760_write_fan(struct device *dev, u32 attr, int channel,
			      long val)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int regval;
	unsigned int mask;
	u16 tach;

	switch (attr) {
	case hwmon_fan_min:
		tach = max31760_tach_from_rpm(val,
					      max31760->fan[channel].pulses);
		return max31760_write_word(max31760->regmap, MAX31760_REG_TCTH,
					   tach);
	case hwmon_fan_pulses:
		max31760_update_fan_pulses(max31760, channel, val);
		regval = (unsigned int)max31760->fan[channel].pulses;
		if (channel) {
			regval <<= 3;
			mask = MAX31760_USER0_PULSE2;
		} else {
			mask = MAX31760_USER0_PULSE1;
		}
		return regmap_update_bits(max31760->regmap, MAX31760_REG_USER0,
					  mask, regval);
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int max31760_write_pwm(struct device *dev, u32 attr, int channel,
			      long val)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int regval;
	int err;

	switch (attr) {
	case hwmon_pwm_input:
		regval = (unsigned int)val & 0xff;
		err = regmap_write(max31760->regmap, MAX31760_REG_PWMR, regval);
		if (err)
			return err;
		return regmap_write(max31760->regmap, MAX31760_REG_USER1_PWMR,
				    regval);
	case hwmon_pwm_enable:
		switch (val) {
		case MAX31760_PWM_ENABLE_FULL:
			err = regmap_write(max31760->regmap, MAX31760_REG_PWMR,
					   0xff);
			if (err)
				return err;
			/* fallthrough */
		case MAX31760_PWM_ENABLE_MANUAL:
			return regmap_update_bits(max31760->regmap,
						  MAX31760_REG_CR2,
						  MAX31760_CR2_DFC,
						  MAX31760_CR2_DFC);
		default:
		case MAX31760_PWM_ENABLE_AUTO:
			return regmap_update_bits(max31760->regmap,
						  MAX31760_REG_CR2,
						  MAX31760_CR2_DFC, 0);
		}
		break;
	case hwmon_pwm_freq:
		if (val < 91)
			regval = MAX31760_DRV_33HZ;
		else if (val < 825)
			regval = MAX31760_DRV_150HZ;
		else if (val < 11000)
			regval = MAX31760_DRV_1500HZ;
		else
			regval = MAX31760_DRV_25KHZ;
		return regmap_update_bits(max31760->regmap, MAX31760_REG_CR1,
					  MAX31760_CR1_DRV, regval);
	}
	return 0;
}

static int max31760_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_temp:
		return max31760_write_temp(dev, attr, channel, val);
	case hwmon_fan:
		return max31760_write_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return max31760_write_pwm(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t max31760_is_visible(const void *dvrdata,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	struct max31760 *max31760 = (struct max31760 *)dvrdata;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_max_hyst:
		case hwmon_temp_max_alarm:
		case hwmon_temp_emergency_hyst:
		case hwmon_temp_emergency_alarm:
		case hwmon_temp_fault:
			return 0444;
		case hwmon_temp_label:
			if (max31760->temp_label[channel])
				return 0444;
			return 0;
		case hwmon_temp_max:
		case hwmon_temp_emergency:
			return 0644;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
		case hwmon_fan_min_alarm:
			return 0444;
		case hwmon_fan_label:
			if (max31760->fan[channel].label)
				return 0444;
			return 0;
		case hwmon_fan_min:
		case hwmon_fan_pulses:
			return 0644;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
		case hwmon_pwm_enable:
		case hwmon_pwm_freq:
			return 0644;
		}
		break;
	default:
		break;
	}

	return 0;
}

static u32 max31760_chip_config[] = {
	HWMON_C_REGISTER_TZ,
	0
};

static const struct hwmon_channel_info max31760_chip = {
	.type = hwmon_chip,
	.config = max31760_chip_config,
};

static u32 max31760_temp_config[] = {
	/*
	 * Local temperature sensor:
	 *     Local high set point (LHS) -> MAX,
	 *     Local over-temperature set point (LOTS) -> EMERGENCY
	 *     There is no fault flag for this temperature sensor.
	 */
	HWMON_T_INPUT | HWMON_T_LABEL | HWMON_T_MAX | HWMON_T_EMERGENCY |
		HWMON_T_MAX_ALARM | HWMON_T_EMERGENCY_ALARM |
		HWMON_T_MAX_HYST | HWMON_T_EMERGENCY_HYST,
	/*
	 * Remote temperature sensor:
	 *     Remote high set point (RHS) -> MAX,
	 *     Remote over-temperature set point (ROTS) -> EMERGENCY
	 */
	HWMON_T_INPUT | HWMON_T_LABEL | HWMON_T_MAX | HWMON_T_EMERGENCY |
		HWMON_T_MAX_ALARM | HWMON_T_EMERGENCY_ALARM |
		HWMON_T_MAX_HYST | HWMON_T_EMERGENCY_HYST | HWMON_T_FAULT,
	0
};

static const struct hwmon_channel_info max31760_temp = {
	.type = hwmon_temp,
	.config = max31760_temp_config,
};

static u32 max31760_fan_config[] = {
	HWMON_F_INPUT | HWMON_F_MIN | HWMON_F_MIN_ALARM | HWMON_F_PULSES |
		HWMON_F_LABEL,
	HWMON_F_INPUT | HWMON_F_MIN | HWMON_F_MIN_ALARM | HWMON_F_PULSES |
		HWMON_F_LABEL,
	0
};

static const struct hwmon_channel_info max31760_fan = {
	.type = hwmon_fan,
	.config = max31760_fan_config,
};

static u32 max31760_pwm_config[] = {
	HWMON_PWM_INPUT | HWMON_PWM_ENABLE | HWMON_PWM_FREQ,
	0
};

static const struct hwmon_channel_info max31760_pwm = {
	.type = hwmon_pwm,
	.config = max31760_pwm_config,
};

static const struct hwmon_channel_info *max31760_info[] = {
	&max31760_chip,
	&max31760_temp,
	&max31760_fan,
	&max31760_pwm,
	NULL
};

static const struct hwmon_ops max31760_hwmon_ops = {
	.is_visible = max31760_is_visible,
	.read = max31760_read,
	.read_string = max31760_read_string,
	.write = max31760_write,
};

static const struct hwmon_chip_info max31760_chip_info = {
	.ops = &max31760_hwmon_ops,
	.info = max31760_info,
};

/* Show which temperature sensors are used to drive the PWM lookup table. */
static ssize_t max31760_pwm_auto_channels_temp_show(
		struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int regval;
	int channels;
	int err;

	err = regmap_read(max31760->regmap, MAX31760_REG_CR1, &regval);
	if (err < 0)
		return err;

	/*
	 * Auto channels is a bit-field. TIS bit clear: temp1 (local) is used
	 * for the LUT. TIS bit set: temp2 (remote) is used for the LUT.
	 * MTI bit set: maximum temp from both is used, TIS bit is ignored.
	 */
	if (regval & MAX31760_CR1_MTI)
		channels = 3;
	else if (regval & MAX31760_CR1_TIS)
		channels = 2;
	else
		channels = 1;

	return snprintf(buf, PAGE_SIZE, "%d\n", channels);
}

/* Store which temperature sensors are used to drive the PWM lookup table. */
static ssize_t max31760_pwm_auto_channels_temp_store(
		struct device *dev, struct device_attribute *devattr,
		const char *buf, size_t count)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int regval;
	unsigned int mask;
	unsigned long channels;
	int err;

	err = kstrtoul(buf, 10, &channels);
	if (err < 0)
		return err;

	switch (channels & 0x3) {
	case 3:
		mask = MAX31760_CR1_MTI;
		regval = MAX31760_CR1_MTI;
		break;
	case 1:
		mask = MAX31760_CR1_TIS | MAX31760_CR1_MTI;
		regval = 0;
		break;
	default:
	case 2:
		mask = MAX31760_CR1_TIS | MAX31760_CR1_MTI;
		regval = MAX31760_CR1_TIS;
		break;
	}

	err = regmap_update_bits(max31760->regmap, MAX31760_REG_CR1, mask,
				 regval);
	if (err < 0)
		return err;

	return count;
}

/* Show the PWM value at a lookup table index. */
static ssize_t max31760_pwm_auto_point_pwm_show(
		struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	struct sensor_device_attribute *sensor_dev_attr =
						to_sensor_dev_attr(devattr);
	unsigned int reg = MAX31760_REG_LUT + sensor_dev_attr->index;
	unsigned int regval;
	int err;

	err = regmap_read(max31760->regmap, reg, &regval);
	if (err < 0)
		return err;

	return snprintf(buf, PAGE_SIZE, "%u\n", regval);
}

/* Store the PWM value at a lookup table index. */
static ssize_t max31760_pwm_auto_point_pwm_store(
		struct device *dev, struct device_attribute *devattr,
		const char *buf, size_t count)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	struct sensor_device_attribute *sensor_dev_attr =
						to_sensor_dev_attr(devattr);
	unsigned int reg = MAX31760_REG_LUT + sensor_dev_attr->index;
	unsigned int regval;
	unsigned long pwm;
	int err;

	err = kstrtoul(buf, 10, &pwm);
	if (err < 0)
		return err;
	regval = pwm & 0xff;

	err = regmap_write(max31760->regmap, reg, regval);
	if (err < 0)
		return err;

	return count;
}

/* Returns the temperature for the given PWM lookup table index. */
static int max31760_pwm_auto_point_temp(int index)
{
	if (index == 0)
		return MAX31760_TEMP_MIN_MC;
	else
		return (16 + index * 2) * 1000;
}

/* Show the temperature for a PWM lookup table index. */
static ssize_t max31760_pwm_auto_point_temp_show(
		struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct sensor_device_attribute *sensor_dev_attr =
						to_sensor_dev_attr(devattr);
	int temp = max31760_pwm_auto_point_temp(sensor_dev_attr->index);

	return snprintf(buf, PAGE_SIZE, "%d\n", temp);
}

/* Show the temperature hysteresis for a PWM lookup table index. */
static ssize_t max31760_pwm_auto_point_temp_hyst_show(
		struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	struct sensor_device_attribute *sensor_dev_attr =
						to_sensor_dev_attr(devattr);
	int temp = max31760_pwm_auto_point_temp(sensor_dev_attr->index);
	unsigned int regval;
	int err;

	err = regmap_read(max31760->regmap, MAX31760_REG_CR1, &regval);
	if (err < 0)
		return err;

	if (regval & MAX31760_CR1_HYST)
		temp -= MAX31760_LUT_HYST_SET;
	else
		temp -= MAX31760_LUT_HYST_CLEAR;

	return snprintf(buf, PAGE_SIZE, "%d\n", temp);
}

/* Store the temperature hysteresis for a PWM lookup table index. */
static ssize_t max31760_pwm_auto_point_temp_hyst_store(
		struct device *dev, struct device_attribute *devattr,
		const char *buf, size_t count)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	struct sensor_device_attribute *sensor_dev_attr =
						to_sensor_dev_attr(devattr);
	int temp = max31760_pwm_auto_point_temp(sensor_dev_attr->index);
	unsigned int regval;
	long hyst;
	int err;

	err = kstrtol(buf, 10, &hyst);
	if (err < 0)
		return err;

	temp -= hyst;
	if (temp >= MAX31760_LYT_HYST_THRESH)
		regval = MAX31760_CR1_HYST;
	else
		regval = 0;
	err = regmap_update_bits(max31760->regmap, MAX31760_REG_CR1,
				 MAX31760_CR1_HYST, regval);
	if (err < 0)
		return err;
	return count;
}

static SENSOR_DEVICE_ATTR(pwm1_auto_channels_temp, 0644,
			  max31760_pwm_auto_channels_temp_show,
			  max31760_pwm_auto_channels_temp_store, 0);
static struct attribute *max31760_attrs[] = {
	&sensor_dev_attr_pwm1_auto_channels_temp.dev_attr.attr,
	NULL
};
static const struct attribute_group max31760_group = {
	.attrs = max31760_attrs,
};

/* Writes to the 'config_load' attribute. */
static ssize_t max31760_config_load_store(struct device *dev,
					  struct device_attribute *devattr,
					  const char *buf, size_t count)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned long sections;
	unsigned int regval;
	int err;

	err = kstrtoul(buf, 10, &sections);
	if (err < 0)
		return err;

	if (sections == 0)
		regval = MAX31760_EEX_BLKS;
	else
		regval = sections & MAX31760_EEX_BLKS;
	regval |= MAX31760_EEX_LW;

	err = regmap_write(max31760->regmap, MAX31760_REG_EEX, regval);
	if (err < 0)
		return err;

	return count;
}

/* Writes to the 'config_store' attribute. */
static ssize_t max31760_config_store_store(struct device *dev,
					   struct device_attribute *devattr,
					   const char *buf, size_t count)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned long sections;
	unsigned int regval;
	int err;

	err = kstrtoul(buf, 10, &sections);
	if (err < 0)
		return err;

	if (sections == 0)
		regval = MAX31760_EEX_BLKS;
	else
		regval = sections & MAX31760_EEX_BLKS;

	err = regmap_write(max31760->regmap, MAX31760_REG_EEX, regval);
	if (err < 0)
		return err;

	return count;
}

/* Reads from the 'pwmX_fallback' attribute. */
static ssize_t max31760_pwm_fallback_show(struct device *dev,
					  struct device_attribute *devattr,
					  char *buf)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int regval;
	int err;

	err = regmap_read(max31760->regmap, MAX31760_REG_FFDC, &regval);
	if (err < 0)
		return err;

	return snprintf(buf, PAGE_SIZE, "%u\n", regval);
}

/* Writes to the 'pwmX_fallback' attribute. */
static ssize_t max31760_pwm_fallback_store(struct device *dev,
					   struct device_attribute *devattr,
					   const char *buf, size_t count)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int regval;
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;
	regval = val & 0xff;

	err = regmap_write(max31760->regmap, MAX31760_REG_FFDC, regval);
	if (err < 0)
		return err;
	return count;
}

/* Reads from the 'pwmX_ramp_rate' attribute. */
static ssize_t max31760_pwm_ramp_rate_show(struct device *dev,
					   struct device_attribute *devattr,
					   char *buf)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int regval;
	int err;

	err = regmap_read(max31760->regmap, MAX31760_REG_CR3, &regval);
	if (err < 0)
		return err;
	switch (regval & MAX31760_CR3_RAMP) {
	case MAX31760_RAMP_SLOW:
		regval = 8;
		break;
	case MAX31760_RAMP_SMED:
		regval = 16;
		break;
	case MAX31760_RAMP_MEDF:
		regval = 32;
		break;
	case MAX31760_RAMP_FAST:
		regval = 255;
		break;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", regval);
}

/* Writes to the 'pwmX_ramp_rate' attribute. */
static ssize_t max31760_pwm_ramp_rate_store(struct device *dev,
					    struct device_attribute *devattr,
					    const char *buf, size_t count)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int regval;
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;
	if (val <= 12)
		regval = MAX31760_RAMP_SLOW;
	else if (val <= 24)
		regval = MAX31760_RAMP_SMED;
	else if (val <= 143)
		regval = MAX31760_RAMP_MEDF;
	else
		regval = MAX31760_RAMP_FAST;

	err = regmap_update_bits(max31760->regmap, MAX31760_REG_CR3,
				 MAX31760_CR3_RAMP, regval);
	if (err < 0)
		return err;
	return count;
}

static SENSOR_DEVICE_ATTR(config_load, 0200, NULL, max31760_config_load_store,
			  0);
static SENSOR_DEVICE_ATTR(config_store, 0200, NULL, max31760_config_store_store,
			  0);
static SENSOR_DEVICE_ATTR(pwm1_fallback, 0644, max31760_pwm_fallback_show,
			  max31760_pwm_fallback_store, 0);
static SENSOR_DEVICE_ATTR(pwm1_ramp_rate, 0644, max31760_pwm_ramp_rate_show,
			  max31760_pwm_ramp_rate_store, 0);

static struct attribute *max31760_custom_attrs[] = {
	&sensor_dev_attr_config_load.dev_attr.attr,
	&sensor_dev_attr_config_store.dev_attr.attr,
	&sensor_dev_attr_pwm1_fallback.dev_attr.attr,
	&sensor_dev_attr_pwm1_ramp_rate.dev_attr.attr,
	NULL
};
static const struct attribute_group max31760_custom_group = {
	.name = "custom",
	.attrs = max31760_custom_attrs,
};

/* Generate auto_point sensor attributes. */
static void max31760_setup_attr_groups(struct device *dev)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	struct max31760_dev_attr *lut_dev_attr = max31760->lut_dev_attrs;
	struct device_attribute *dev_attr;
	int attr_index = 0;
	int i;

	for (i = 0; i < MAX31760_LUT_COUNT; i++, lut_dev_attr++) {
		snprintf(lut_dev_attr->name, MAX31760_LUT_NAME_SIZE,
			 "pwm1_auto_point%02d_pwm", i);
		lut_dev_attr->sdattr.index = i;
		dev_attr = &lut_dev_attr->sdattr.dev_attr;
		dev_attr->attr.name = lut_dev_attr->name;
		dev_attr->attr.mode = 0644;
		dev_attr->show = max31760_pwm_auto_point_pwm_show;
		dev_attr->store = max31760_pwm_auto_point_pwm_store;
		max31760->lut_attrs[attr_index++] =
			&lut_dev_attr->sdattr.dev_attr.attr;
	}

	for (i = 0; i < MAX31760_LUT_COUNT; i++, lut_dev_attr++) {
		snprintf(lut_dev_attr->name, MAX31760_LUT_NAME_SIZE,
			 "pwm1_auto_point%02d_temp", i);
		lut_dev_attr->sdattr.index = i;
		dev_attr = &lut_dev_attr->sdattr.dev_attr;
		dev_attr->attr.name = lut_dev_attr->name;
		dev_attr->attr.mode = 0444;
		dev_attr->show = max31760_pwm_auto_point_temp_show;
		max31760->lut_attrs[attr_index++] =
			&lut_dev_attr->sdattr.dev_attr.attr;
	}

	for (i = 0; i < MAX31760_LUT_COUNT; i++, lut_dev_attr++) {
		snprintf(lut_dev_attr->name, MAX31760_LUT_NAME_SIZE,
			 "pwm1_auto_point%02d_temp_hyst", i);
		lut_dev_attr->sdattr.index = i;
		dev_attr = &lut_dev_attr->sdattr.dev_attr;
		dev_attr->attr.name = lut_dev_attr->name;
		dev_attr->attr.mode = 0644;
		dev_attr->show = max31760_pwm_auto_point_temp_hyst_show;
		dev_attr->store = max31760_pwm_auto_point_temp_hyst_store;
		max31760->lut_attrs[attr_index++] =
			&lut_dev_attr->sdattr.dev_attr.attr;
	}

	max31760->lut_group.attrs = max31760->lut_attrs;
	max31760->attr_groups[0] = &max31760->lut_group;
	max31760->attr_groups[1] = &max31760_group;
	max31760->attr_groups[2] = &max31760_custom_group;
}

/* Update internal storage for the current register values. */
static int max31760_update_from_registers(struct device *dev)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	unsigned int regval;
	long val;
	int i;
	int err;

	/* Restore fan pulses values. */
	for (i = 0; i < MAX31760_NUM_FANS; i++) {
		err = max31760_read_fan(dev, hwmon_fan_pulses, i, &val);
		if (err)
			return err;
		max31760->fan[i].pulses = val;
	}

	/* Clear standby bit in case it is set. */
	err = regmap_update_bits(max31760->regmap, MAX31760_REG_CR2,
				 MAX31760_CR2_STBY, 0);
	if (err)
		return err;

	/* Restore last stored manual PWM value. */
	err = regmap_read(max31760->regmap, MAX31760_REG_USER1_PWMR, &regval);
	if (err)
		return err;
	return regmap_write(max31760->regmap, MAX31760_REG_PWMR, regval);
}

/* Read properties of a fan node. */
static int max31760_of_init_fan(struct device *dev,
				struct device_node *fan_node, u32 reg)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	char supply_name[MAX31760_SUPPLY_NAME_SIZE];
	struct max31760_fan *fan;
	int err;

	if (reg >= MAX31760_NUM_FANS) {
		dev_err(dev, "invalid reg on fan: %s", fan_node->name);
		return -EINVAL;
	}

	fan = &max31760->fan[reg];
	err = of_property_read_string(fan_node, "label", &fan->label);
	if (err)
		fan->label = NULL;

	fan->enabled = of_device_is_available(fan_node);

	fan->enable_gpio = devm_get_gpiod_from_child(dev, "maxim,enable",
						     &fan_node->fwnode);
	err = PTR_ERR(fan->enable_gpio);
	if (err == -ENOENT || err == -ENODEV) {
		fan->enable_gpio = NULL;
	} else if (err == -EPROBE_DEFER) {
		dev_dbg(dev, "Defer due to fan enable gpio.");
		return err;
	} else if (IS_ERR(fan->enable_gpio)) {
		dev_err(dev, "Error getting fan enable gpio: %d", err);
		return err;
	}

	if (fan->enable_gpio) {
		err = gpiod_direction_output(fan->enable_gpio, fan->enabled);
		if (err) {
			dev_err(dev, "Failed to set fan%d gpio output %s.", reg,
				fan->enabled ? "enabled" : "disabled");
			return err;
		}
	}

	snprintf(supply_name, MAX31760_SUPPLY_NAME_SIZE, "maxim,fan%u", reg);
	fan->supply = devm_regulator_get_optional(dev, supply_name);
	err = PTR_ERR(fan->supply);
	if (err == -ENODEV) {
		fan->supply = NULL;
	} else if (err == -EPROBE_DEFER) {
		dev_dbg(dev, "Defer due to %s-supply.", supply_name);
		return err;
	} else if (err == -ENOENT) {
		dev_err(dev, "Could not find regulator for %s-supply.",
			supply_name);
		return err;
	} else if (IS_ERR(fan->supply)) {
		dev_err(dev, "Get '%s-supply' error: %d", supply_name, err);
		return err;
	}

	if (fan->supply && fan->enabled) {
		err = regulator_enable(fan->supply);
		if (err) {
			dev_err(dev, "Failed to enable %s-supply: %d",
				supply_name, err);
			return err;
		}
	}

	return 0;
}

/* Read properties of a temp node. */
static int max31760_of_init_temp(struct device *dev,
				 struct device_node *temp_node, u32 reg)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	const char *label;
	int err;
	u32 val;

	if (reg >= MAX31760_NUM_TEMPS) {
		dev_err(dev, "invalid reg on temp: %s", temp_node->name);
		return -EINVAL;
	}

	err = of_property_read_string(temp_node, "label", &label);
	if (!err)
		max31760->temp_label[reg] = label;

	err = of_property_read_u32(temp_node, "maxim,ideality", &val);
	if (!err && reg == 1) {
		/* Only external temp sensor has ideality. */
		err = regmap_write(max31760->regmap, MAX31760_REG_IFR,
				   val & 0x3f);
	}

	return 0;
}

/* Configure registers which have associated device properties. */
static int max31760_of_init(struct device *dev)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	struct device_node *node;
	int err;
	u32 reg;
	u32 val;

	for_each_available_child_of_node(dev->of_node, node) {
		err = of_property_read_u32(node, "reg", &reg);
		if (err) {
			dev_err(dev, "invalid reg on sub-node: %s", node->name);
			return err;
		}

		if (!strcmp(node->name, "fan"))
			err = max31760_of_init_fan(dev, node, reg);
		else if (!strcmp(node->name, "temp"))
			err = max31760_of_init_temp(dev, node, reg);
		else
			dev_err(dev, "invalid subnode with name: %s",
				node->name);
		if (err) {
			dev_err(dev, "child '%s' err: %d", node->name, err);
			return err;
		}
	}

	val = 0;
	if (of_property_read_bool(dev->of_node, "maxim,pwm-polarity-negative"))
		val |= MAX31760_CR1_PPS;
	err = regmap_update_bits(max31760->regmap, MAX31760_REG_CR1,
				 MAX31760_CR1_PPS, val);
	if (err)
		return err;

	/* Put ALERT pin into comparator mode: interrupts aren't supported. */
	val = MAX31760_CR2_ALERTS | MAX31760_CR2_FFMODE;
	if (of_property_read_bool(dev->of_node, "maxim,fan-spin-up-enabled"))
		val |= MAX31760_CR2_SPEN;
	if (of_property_read_bool(dev->of_node, "maxim,fan-rd-signal"))
		val |= MAX31760_CR2_FSST;
	if (of_property_read_bool(dev->of_node, "maxim,fan-rd-polarity-high"))
		val |= MAX31760_CR2_RDPS;
	if (of_property_read_bool(dev->of_node, "maxim,fan-signal-enabled"))
		val |= MAX31760_CR2_FSEN;
	/*
	 * Future properties:
	 *     maxim,fan-fail-interrupt -> remove MAX31760_CR2_FFMODE
	 *     maxim,temp-alert-interrupt -> remove MAX31760_CR2_ALERTS
	 */

	err = regmap_update_bits(max31760->regmap, MAX31760_REG_CR2,
				 MAX31760_CR2_SPEN | MAX31760_CR2_FSST |
				 MAX31760_CR2_RDPS | MAX31760_CR2_FSEN |
				 MAX31760_CR2_ALERTS | MAX31760_CR2_FFMODE,
				 val);
	if (err)
		return err;

	val = (max31760->fan[0].enabled ? MAX31760_CR3_TACH1E : 0) |
	      (max31760->fan[1].enabled ? MAX31760_CR3_TACH2E : 0);
	if (of_property_read_bool(dev->of_node, "maxim,fan-fail-full-only"))
		val |= MAX31760_CR3_TACHFL;
	if (of_property_read_bool(dev->of_node,
				  "maxim,pwm-pulse-stretch-enabled"))
		val |= MAX31760_CR3_PSEN;
	if (of_property_read_bool(dev->of_node, "maxim,pwm-zero-fan-can-fail"))
		val |= MAX31760_CR3_FF_0;

	return regmap_update_bits(max31760->regmap, MAX31760_REG_CR3,
				  MAX31760_CR3_TACH1E | MAX31760_CR3_TACH2E |
				  MAX31760_CR3_TACHFL | MAX31760_CR3_PSEN |
				  MAX31760_CR3_FF_0, val);
}

static int max31760_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct max31760 *max31760;
	int err;

	max31760 = devm_kzalloc(dev, sizeof(*max31760), GFP_KERNEL);
	if (!max31760)
		return -ENOMEM;

	dev_set_drvdata(dev, max31760);

	max31760->vdd_supply = devm_regulator_get_optional(dev, "vdd");
	err = PTR_ERR(max31760->vdd_supply);
	if (err == -ENODEV) {
		max31760->vdd_supply = NULL;
	} else if (err == -EPROBE_DEFER) {
		dev_dbg(dev, "Defer due to vdd-supply.");
		return err;
	} else if (err == -ENOENT) {
		dev_err(dev, "Could not find regulator for vdd-supply.");
		return err;
	} else if (IS_ERR(max31760->vdd_supply)) {
		dev_err(dev, "Unhandled error %d getting vdd-supply.", err);
		return err;
	}

	if (max31760->vdd_supply) {
		err = regulator_enable(max31760->vdd_supply);
		if (err) {
			dev_err(dev, "Failed to enable vdd-supply: %d", err);
			return err;
		}
	}

	max31760->regmap = devm_regmap_init_i2c(client,
						&max31760_regmap_config);
	if (IS_ERR(max31760->regmap)) {
		err = PTR_ERR(max31760->regmap);
		dev_err(dev, "regmap init failed: %d", err);
		goto err_disable_regulator;
	}

	err = max31760_of_init(dev);
	if (err) {
		dev_err(dev, "failed to initialize from firmware: %d", err);
		goto err_disable_regulator;
	}

	err = max31760_update_from_registers(dev);
	if (err) {
		dev_err(dev, "failed to update from registers: %d", err);
		goto err_disable_regulator;
	}

	max31760_setup_attr_groups(dev);
	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 max31760,
							 &max31760_chip_info,
							 max31760->attr_groups);
	err = PTR_ERR_OR_ZERO(hwmon_dev);
	if (err)
		goto err_disable_regulator;
	return 0;

err_disable_regulator:
	if (max31760->vdd_supply)
		regulator_disable(max31760->vdd_supply);
	return err;
}

/* Toggle the fan GPIOs and regulators to match enable state. */
static int max31760_set_enabled(struct device *dev, bool enable)
{
	struct max31760 *max31760 = dev_get_drvdata(dev);
	struct max31760_fan *fan;
	bool is_enabled;
	int last_err;
	int err = 0;
	int i;

	if (!enable) {
		last_err = regmap_update_bits(max31760->regmap, MAX31760_REG_CR2,
					      MAX31760_CR2_STBY, MAX31760_CR2_STBY);
		if (last_err) {
			err = last_err;
			dev_err(dev, "Could not set Standby bit: %d", err);
		}
	}

	for (i = 0; i < MAX31760_NUM_FANS; i++) {
		fan = &max31760->fan[i];
		is_enabled = enable && fan->enabled;

		if (fan->enable_gpio)
			gpiod_set_value_cansleep(fan->enable_gpio, is_enabled);

		if (fan->supply) {
			if (is_enabled)
				last_err = regulator_enable(fan->supply);
			else
				last_err = regulator_disable(fan->supply);
			if (last_err) {
				err = last_err;
				dev_err(dev, "Failed to %s fan %d vdd-supply",
					is_enabled ? "enabled" : "disable", i);
			}
		}
	}

	if (max31760->vdd_supply) {
		if (enable)
			last_err = regulator_enable(max31760->vdd_supply);
		else if (regulator_is_enabled(max31760->vdd_supply))
			last_err = regulator_disable(max31760->vdd_supply);
		else
			last_err = 0;
		if (last_err) {
			err = last_err;
			dev_err(dev, "Failed to %s vdd-supply",
				enable ? "enable" : "disable");
		}
	}

	if (enable) {
		last_err = regmap_update_bits(max31760->regmap, MAX31760_REG_CR2,
					      MAX31760_CR2_STBY, 0);
		if (last_err) {
			err = last_err;
			dev_err(dev, "Could not clear Standby bit: %d", err);
		}
	}

	return err;
}

static int max31760_remove(struct i2c_client *client)
{
	struct max31760 *max31760 = i2c_get_clientdata(client);

	if (!max31760)
		return 0;

	return max31760_set_enabled(&client->dev, false);
}

static int __maybe_unused max31760_suspend(struct device *dev)
{
	return max31760_set_enabled(dev, false);
}

static int __maybe_unused max31760_resume(struct device *dev)
{
	int err;

	err = max31760_set_enabled(dev, true);
	if (err)
		return err;

	err = max31760_update_from_registers(dev);
	if (err)
		dev_err(dev, "failed to update from registers: %d", err);
	return err;
}

static SIMPLE_DEV_PM_OPS(max31760_dev_pm_ops, max31760_suspend,
			 max31760_resume);

static const struct i2c_device_id max31760_i2c_ids[] = {
	{ "max31760", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max31760_i2c_ids);

#ifdef CONFIG_OF
static const struct of_device_id max31760_of_ids[] = {
	{ .compatible = "maxim,max31760", },
	{}
};
MODULE_DEVICE_TABLE(of, max31760_of_ids);
#endif

static struct i2c_driver max31760_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.pm	= &max31760_dev_pm_ops,
		.of_match_table = of_match_ptr(max31760_of_ids),
	},
	.probe		= max31760_probe,
	.remove		= max31760_remove,
	.id_table	= max31760_i2c_ids,
};

module_i2c_driver(max31760_driver);

MODULE_AUTHOR("John Muir <john@jmuir.com>");
MODULE_DESCRIPTION("Maxim Integrated MAX31760 Precision Fan-Speed Controller Driver");
MODULE_LICENSE("GPL");
