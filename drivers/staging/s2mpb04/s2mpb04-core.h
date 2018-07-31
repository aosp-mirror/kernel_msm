/*
 * Copyright (C) 2017 Google, Inc.
 *
 * Author: Trevor Bunker <trevorbunker@google.com>
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

#ifndef S2MPB04_REGULATOR_H
#define S2MPB04_REGULATOR_H

/* registers */
#define S2MPB04_REG_CHIP_ID 0x00
#define S2MPB04_REG_INT1 0x01
#define S2MPB04_REG_INT2 0x02
#define S2MPB04_REG_INT3 0x03
#define S2MPB04_REG_INT4 0x04
#define S2MPB04_REG_INT1M 0x05
#define S2MPB04_REG_INT2M 0x06
#define S2MPB04_REG_INT3M 0x07
#define S2MPB04_REG_INT4M 0x08
#define S2MPB04_REG_STATUS1 0x09
#define S2MPB04_REG_STATUS2 0x0A
#define S2MPB04_REG_STATUS3 0x0B
#define S2MPB04_REG_CFG_PM 0x0C
#define S2MPB04_REG_BUCK1_CTRL 0x0D
#define S2MPB04_REG_BUCK1_OUT 0x0E
#define S2MPB04_REG_BUCK2_CTRL 0x0F
#define S2MPB04_REG_BUCK2_OUT 0x10
#define S2MPB04_REG_BUCK3_CTRL 0x11
#define S2MPB04_REG_BUCK3_OUT 0x12
#define S2MPB04_REG_LDO1_CTRL 0x13
#define S2MPB04_REG_LDO2_CTRL 0x14
#define S2MPB04_REG_LDO3_CTRL 0x15
#define S2MPB04_REG_LDO4_CTRL 0x16
#define S2MPB04_REG_LDO5_CTRL 0x17
#define S2MPB04_REG_BUCK1_OUT_DVS 0x18
#define S2MPB04_REG_LDO3_CTRL_DVS 0x19
#define S2MPB04_REG_LDO3_DVS 0x1A
#define S2MPB04_REG_NORM_THRES 0x1B
#define S2MPB04_REG_BOOST_THRES 0x1C
#define S2MPB04_REG_ADC_CTRL 0x1D
#define S2MPB04_REG_ADC_CTRL2 0x1E
#define S2MPB04_REG_ADC_CTRL3 0x1F
#define S2MPB04_REG_ADC0DATA 0x20
#define S2MPB04_REG_ADC1DATA 0x21
#define S2MPB04_REG_ADC2DATA 0x22
#define S2MPB04_REG_ADC3DATA 0x23
#define S2MPB04_REG_MUXSEL0 0x24
#define S2MPB04_REG_MUXSEL1 0x25
#define S2MPB04_REG_MUXSEL2 0x26
#define S2MPB04_REG_MUXSEL3 0x27
#define S2MPB04_REG_ADC_THRES_CTRL1 0x28
#define S2MPB04_REG_ADC_THRES_CTRL2 0x29
#define S2MPB04_REG_ADC_THRES_CTRL3 0x2A
#define S2MPB04_REG_ADC_THRES_CTRL4 0x2B
#define S2MPB04_REG_ADC_THRES_CTRL5 0x2C
#define S2MPB04_REG_ADC_THRES_CTRL6 0x2D
#define S2MPB04_REG_ADC_THRES_CTRL7 0x2E
#define S2MPB04_REG_ADC_THRES_CTRL8 0x2F
#define S2MPB04_REG_ADC_DBNC_CTRL1 0x30
#define S2MPB04_REG_ADC_DBNC_CTRL2 0x31
#define S2MPB04_REG_ADC_CAL_OFS1 0x32
#define S2MPB04_REG_ADC_CAL_OFS2 0x33
#define S2MPB04_REG_ADC_MON_TEMP 0x34
#define S2MPB04_REG_ADC_MON_VBAT 0x35
#define S2MPB04_REG_GPIO_CTRL 0x36
#define S2MPB04_REG_GPIO_A 0x37
#define S2MPB04_REG_GPIO_Y 0x38

/* adc inputs */
#define S2MPB04_ADC_I_SMPS1_PH1     0x2
#define S2MPB04_ADC_I_SMPS1_PH2     0x3
#define S2MPB04_ADC_I_SMPS1_SUM     0x4
#define S2MPB04_ADC_I_SMPS2         0x5
#define S2MPB04_ADC_V_SMPS1         0x6
#define S2MPB04_ADC_V_SMPS2         0x7
#define S2MPB04_ADC_I_LDO1          0x8
#define S2MPB04_ADC_I_LDO2          0x9
#define S2MPB04_ADC_V_LDO1          0xA
#define S2MPB04_ADC_V_LDO2          0xB
#define S2MPB04_ADC_PTAT            0xC
#define S2MPB04_ADC_VBAT            0xD

/* full scale of 8-bit adc slots */
#define S2MPB04_ADC_SCALE_V_SMPS1   4688  /* in uV/lsb */
#define S2MPB04_ADC_SCALE_V_SMPS2   6250  /* in uV/lsb */
#define S2MPB04_ADC_SCALE_I_SMPS1   31250 /* in uA/lsb */
#define S2MPB04_ADC_SCALE_I_SMPS2   15625 /* in uA/lsb */
#define S2MPB04_ADC_SCALE_V_LDO     9375  /* in uA/lsb */
#define S2MPB04_ADC_SCALE_I_LDO     586   /* in uA/lsb */
#define S2MPB04_ADC_SCALE_VBAT      23438 /* in uV/lsb */

/* macros for PTAT-to-temperature conversion */
#define TEMP_TO_PTAT_CODE(X)  ((238260 - (X)) / 1167)
#define PTAT_CODE_TO_TEMP(X)  (238260 - ((X) * 1167))

/* interrupt flags */
#define S2MPB04_INT_PONR              0
#define S2MPB04_INT_PONF              1
#define S2MPB04_INT_ADC_CH0           2
#define S2MPB04_INT_ADC_CH1           3
#define S2MPB04_INT_ADC_CH2           4
#define S2MPB04_INT_ADC_CH3           5
#define S2MPB04_INT_ADC_DONE          6
#define S2MPB04_INT_WATCHDOG          7
#define S2MPB04_INT_SMPS1_OI          8
#define S2MPB04_INT_SMPS2_OI          9
#define S2MPB04_INT_SMPS1_UV          10
#define S2MPB04_INT_LDO1_OI           11
#define S2MPB04_INT_LDO2_OI           12
#define S2MPB04_INT_TSD               13
#define S2MPB04_INT_TH_TRIPR          14
#define S2MPB04_INT_TH_TRIPF          15
#define S2MPB04_INT_TH_TINT           23

/* regulator names */
#define S2MPB04_REGLTR_NAME_SMPS1 "s2mpb04_smps1"
#define S2MPB04_REGLTR_NAME_SMPS2 "s2mpb04_smps2"
#define S2MPB04_REGLTR_NAME_SMPS3 "s2mpb04_smps3"
#define S2MPB04_REGLTR_NAME_LDO1  "s2mpb04_ldo1"
#define S2MPB04_REGLTR_NAME_LDO2  "s2mpb04_ldo2"
#define S2MPB04_REGLTR_NAME_LDO3  "s2mpb04_ldo3"
#define S2MPB04_REGLTR_NAME_LDO4  "s2mpb04_ldo4"
#define S2MPB04_REGLTR_NAME_LDO5  "s2mpb04_ldo5"

/* silicon versions */
#define S2MPB04_REV_ES 0

#define S2MPB04_NUM_GPIOS 2

/* regulator id enum */
enum s2mpb04_regulator_ids {
	S2MPB04_ID_SMPS1,
	S2MPB04_ID_SMPS2,
	S2MPB04_ID_SMPS3,
	S2MPB04_ID_LDO1,
	S2MPB04_ID_LDO2,
	S2MPB04_ID_LDO3,
	S2MPB04_ID_LDO4,
	S2MPB04_ID_LDO5,
	S2MPB04_ID_ALL,
	S2MPB04_NUM_REGULATORS = S2MPB04_ID_ALL,
};

/* driver data structure */
struct s2mpb04_core {
	struct device *dev;
	struct regmap *regmap;
	struct s2mpb04_platform_data *pdata;
	u8 rev_id;

	/* kernel thread for waiting for reset after shutdown */
	struct work_struct reset_work;

	/* completion used for initialization */
	struct completion init_complete;

	/* completion used for unexpected resets */
	struct completion reset_complete;

	/* flags used for serialization */
	unsigned long adc_conv_busy;

	/* completion used for signaling end of adc conversion */
	struct completion adc_conv_complete;
};

/* platform data structure */
struct s2mpb04_platform_data {
	int pon_gpio;
	int resetb_gpio;
	int intb_gpio;
	unsigned int resetb_irq;
	unsigned int intb_irq;
};

int s2mpb04_read_byte(struct s2mpb04_core *ddata, u8 addr, u8 *data);
int s2mpb04_write_byte(struct s2mpb04_core *ddata, u8 addr, u8 data);
int s2mpb04_update_bits(struct s2mpb04_core *ddata, u8 addr, unsigned int mask,
			u8 data);
int s2mpb04_toggle_pon(struct s2mpb04_core *ddata);
int s2mpb04_dump_regs(struct s2mpb04_core *ddata);
int s2mpb04_read_adc_chan(struct s2mpb04_core *ddata, int chan_num,
			  u8 *chan_data);

/* s2mpb04-sysfs.c */
void s2mpb04_config_sysfs(struct device *dev);

/* s2mpb04-regulator.c */
void s2mpb04_regulator_notify(enum s2mpb04_regulator_ids rid,
			      unsigned long event);

#endif /* S2MPB04_REGULATOR_H */
