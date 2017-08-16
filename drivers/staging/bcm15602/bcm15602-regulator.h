/*
 * Copyright (C) 2016 Google, Inc.
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

#ifndef BCM15602_REGULATOR_H
#define BCM15602_REGULATOR_H

/* registers */
#define BCM15602_REG_SYS_PMIC_ID_LSB                  0x00
#define BCM15602_REG_SYS_PMIC_ID_MSB                  0x01
#define BCM15602_REG_SYS_PMIC_REV                     0x02
#define BCM15602_REG_SYS_PMIC_GID                     0x03
#define BCM15602_REG_SYS_WRLOCKEY                     0x04
#define BCM15602_REG_SYS_RCOTRIM_LSB                  0x05
#define BCM15602_REG_SYS_RCOTRIM_MSB                  0x06
#define BCM15602_REG_SYS_GPIO_CTRL                    0x07
#define BCM15602_REG_SYS_GPIO_OUT_CTRL                0x08
#define BCM15602_REG_SYS_GPIO_IN                      0x09
#define BCM15602_REG_I2C_CSR_I2CCTRL1                 0x10
#define BCM15602_REG_I2C_CSR_I2CCTRL2                 0x11
#define BCM15602_REG_I2C_CSR_I2CFIFO                  0x12
#define BCM15602_REG_I2C_I2CCTRL3                     0x20
#define BCM15602_REG_I2C_I2C_SLAVE_ADDR               0x21
#define BCM15602_REG_INT_INTEN1                       0x28
#define BCM15602_REG_INT_INTEN2                       0x29
#define BCM15602_REG_INT_INTEN3                       0x2a
#define BCM15602_REG_INT_INTEN4                       0x2b
#define BCM15602_REG_INT_INTSET1                      0x2c
#define BCM15602_REG_INT_INTSET2                      0x2d
#define BCM15602_REG_INT_INTSET3                      0x2e
#define BCM15602_REG_INT_INTSET4                      0x2f
#define BCM15602_REG_INT_INTFLAG1                     0x30
#define BCM15602_REG_INT_INTFLAG2                     0x31
#define BCM15602_REG_INT_INTFLAG3                     0x32
#define BCM15602_REG_INT_INTFLAG4                     0x33
#define BCM15602_REG_PSM_PSM_CTRL1                    0x38
#define BCM15602_REG_PSM_PSM_CTRL2                    0x39
#define BCM15602_REG_PSM_PSM_CTRL3                    0x3a
#define BCM15602_REG_PSM_PSM_CTRL4                    0x3b
#define BCM15602_REG_PSM_PSM_STS                      0x3c
#define BCM15602_REG_PSM_PSM_ENV                      0x3d
#define BCM15602_REG_PMC_PWR_GRP_CTRL1                0x40
#define BCM15602_REG_PMC_PWR_GRP_CTRL2                0x41
#define BCM15602_REG_PMC_PWR_GRP_CONFG                0x42
#define BCM15602_REG_PMC_PMC_TEST                     0x43
#define BCM15602_REG_BUCK_SDSR_VOCTRL                 0x50
#define BCM15602_REG_BUCK_SDSR_VMAX                   0x51
#define BCM15602_REG_BUCK_SDSR_CTRL0                  0x52
#define BCM15602_REG_BUCK_SDSR_CTRL1                  0x53
#define BCM15602_REG_BUCK_SDSR_CTRL2                  0x54
#define BCM15602_REG_BUCK_SDSR_CTRL3                  0x55
#define BCM15602_REG_BUCK_SDSR_CTRL4                  0x56
#define BCM15602_REG_BUCK_SDSR_CTRL5                  0x57
#define BCM15602_REG_BUCK_SDSR_TSET_CTRL1             0x58
#define BCM15602_REG_BUCK_ASR_VOCTRL                  0x59
#define BCM15602_REG_BUCK_ASR_VMAX                    0x5a
#define BCM15602_REG_BUCK_ASR_VOTRIM                  0x5b
#define BCM15602_REG_BUCK_ASR_CTRL0                   0x5c
#define BCM15602_REG_BUCK_ASR_CTRL1                   0x5d
#define BCM15602_REG_BUCK_ASR_CTRL2                   0x5e
#define BCM15602_REG_BUCK_ASR_CTRL3                   0x5f
#define BCM15602_REG_BUCK_ASR_CTRL4                   0x60
#define BCM15602_REG_BUCK_ASR_CTRL5                   0x61
#define BCM15602_REG_BUCK_ASR_CTRL6                   0x62
#define BCM15602_REG_BUCK_ASR_CTRL7                   0x63
#define BCM15602_REG_BUCK_ASR_CTRL8                   0x64
#define BCM15602_REG_BUCK_ASR_CTRL9                   0x65
#define BCM15602_REG_BUCK_ASR_CTRL10                  0x66
#define BCM15602_REG_BUCK_ASR_CTRL11                  0x67
#define BCM15602_REG_BUCK_ASR_TSET_CTRL1              0x68
#define BCM15602_REG_BUCK_ASR_TSET_CTRL2              0x69
#define BCM15602_REG_BUCK_SDSR_OI_CTRL                0x6a
#define BCM15602_REG_LDO_IOLDO_VOCTRL                 0x70
#define BCM15602_REG_LDO_IOLDO_VMAX                   0x71
#define BCM15602_REG_LDO_IOLDO_ENCTRL                 0x72
#define BCM15602_REG_LDO_IOLDO_OVRI_CNT1              0x73
#define BCM15602_REG_LDO_IOLDO_OVRI_CNT2              0x74
#define BCM15602_REG_LDO_SDLDO_VOCTRL                 0x75
#define BCM15602_REG_LDO_SDLDO_VMAX                   0x76
#define BCM15602_REG_LDO_SDLDO_ENCTRL                 0x77
#define BCM15602_REG_LDO_SDLDO_OVRI_CNT1              0x78
#define BCM15602_REG_LDO_SDLDO_OVRI_CNT2              0x79
#define BCM15602_REG_LDO_LDO_TEST_CTRL1               0x7a
#define BCM15602_REG_ADC_MAN_CTRL                     0x80
#define BCM15602_REG_ADC_MAN_CONV_CHNUM               0x81
#define BCM15602_REG_ADC_MAN_RESULT_L                 0x82
#define BCM15602_REG_ADC_MAN_RESULT_H                 0x83
#define BCM15602_REG_ADC_DIE_TEMP_TRIM                0x84
#define BCM15602_REG_ADC_ISNS_RTRIM                   0x85
#define BCM15602_REG_ADC_ADC_OFFSET_TRIM              0x86
#define BCM15602_REG_ADC_ADC_GAIN_TRIM                0x87
#define BCM15602_REG_ADC_ISNS_GAIN_TRIM_LSB           0x88
#define BCM15602_REG_ADC_SDLDO_ISNS_GAIN_TRIMH        0x89
#define BCM15602_REG_ADC_SDLDO_ISNS_OFFSET_TRIM       0x8a
#define BCM15602_REG_ADC_HKCTRL                       0x8b
#define BCM15602_REG_ADC_HYST                         0x8c
#define BCM15602_REG_ADC_SLOTCTRL0                    0x8d
#define BCM15602_REG_ADC_SLOTCTRL1                    0x8e
#define BCM15602_REG_ADC_SLOTCTRL2                    0x8f
#define BCM15602_REG_ADC_SLOTCTRL3                    0x90
#define BCM15602_REG_ADC_SLOTCTRL4                    0x91
#define BCM15602_REG_ADC_SLOTCTRL5                    0x92
#define BCM15602_REG_ADC_SLOTCTRL6                    0x93
#define BCM15602_REG_ADC_SLOTCTRL7                    0x94
#define BCM15602_REG_ADC_SLOTCTRL8                    0x95
#define BCM15602_REG_ADC_SLOTCTRL9                    0x96
#define BCM15602_REG_ADC_SLOTCTRL10                   0x97
#define BCM15602_REG_ADC_SLOTCTRL11                   0x98
#define BCM15602_REG_ADC_SLOTTHL0                     0x99
#define BCM15602_REG_ADC_SLOTTHH0                     0x9a
#define BCM15602_REG_ADC_SLOTTHL1                     0x9b
#define BCM15602_REG_ADC_SLOTTHH1                     0x9c
#define BCM15602_REG_ADC_SLOTTHL2                     0x9d
#define BCM15602_REG_ADC_SLOTTHH2                     0x9e
#define BCM15602_REG_ADC_SLOTTHL3                     0x9f
#define BCM15602_REG_ADC_SLOTTHH3                     0xa0
#define BCM15602_REG_ADC_SLOTTHL4                     0xa1
#define BCM15602_REG_ADC_SLOTTHH4                     0xa2
#define BCM15602_REG_ADC_SLOTTHL5                     0xa3
#define BCM15602_REG_ADC_SLOTTHH5                     0xa4
#define BCM15602_REG_ADC_SLOTTHL6                     0xa5
#define BCM15602_REG_ADC_SLOTTHH6                     0xa6
#define BCM15602_REG_ADC_SLOTTHL7                     0xa7
#define BCM15602_REG_ADC_SLOTTHH7                     0xa8
#define BCM15602_REG_ADC_SLOTTHL8                     0xa9
#define BCM15602_REG_ADC_SLOTTHH8                     0xaa
#define BCM15602_REG_ADC_SLOTTHL9                     0xab
#define BCM15602_REG_ADC_SLOTTHH9                     0xac
#define BCM15602_REG_ADC_SLOTDATA0                    0xad
#define BCM15602_REG_ADC_SLOTDATA1                    0xae
#define BCM15602_REG_ADC_SLOTDATA2                    0xaf
#define BCM15602_REG_ADC_SLOTDATA3                    0xb0
#define BCM15602_REG_ADC_SLOTDATA4                    0xb1
#define BCM15602_REG_ADC_SLOTDATA5                    0xb2
#define BCM15602_REG_ADC_SLOTDATA6                    0xb3
#define BCM15602_REG_ADC_SLOTDATA7                    0xb4
#define BCM15602_REG_ADC_SLOTDATA8                    0xb5
#define BCM15602_REG_ADC_SLOTDATA9                    0xb6
#define BCM15602_REG_ADC_SLOTDATA10                   0xb7
#define BCM15602_REG_ADC_SLOTDATA11                   0xb8
#define BCM15602_REG_ADC_SLOTDATA3_0_LSB              0xb9
#define BCM15602_REG_ADC_SLOTDATA7_4_LSB              0xba
#define BCM15602_REG_ADC_SLOTDATA11_8_LSB             0xbb
#define BCM15602_REG_ADC_SLOTSTAL                     0xbc
#define BCM15602_REG_ADC_SLOTSTAH                     0xbd
#define BCM15602_REG_ADC_SLOTMSKL                     0xbe
#define BCM15602_REG_ADC_SLOTMSKH                     0xbf
#define BCM15602_REG_ADC_SLOTDATA_READINGL            0xc0
#define BCM15602_REG_ADC_SLOTDATA_READINGH            0xc1
#define BCM15602_REG_ADC_BGCTRL                       0xc2
#define BCM15602_REG_ADC_BG_OFFSET_TRIM               0xc3
#define BCM15602_REG_ADC_HOTTEMP_THRES                0xc4
#define BCM15602_REG_ADC_LBG_TC_TRIM                  0xc5
#define BCM15602_REG_ADC_TCTRIM                       0xc6
#define BCM15602_REG_ADC_IOLDO_ISNS_GAIN_TRIMH        0xc8
#define BCM15602_REG_ADC_IOLDO_ISNS_OFFSET_TRIM       0xc9
#define BCM15602_REG_ADC_ASR_ISNS_GAIN_TRIMH          0xca
#define BCM15602_REG_ADC_ASR_ISNS_OFFSET_TRIM         0xcb
#define BCM15602_REG_ADC_ASR_MSTR_ISNS_GAIN_TRIMH     0xcc
#define BCM15602_REG_ADC_ASR_MSTR_ISNS_OFFSET_TRIM    0xcd
#define BCM15602_REG_ADC_SDSR_ISNS_GAIN_TRIMH         0xce
#define BCM15602_REG_ADC_SDSR_ISNS_OFFSET_TRIM        0xcf
#define BCM15602_REG_WDT_WDTCTRL1                     0xd0
#define BCM15602_REG_WDT_WDTCTRL2                     0xd1
#define BCM15602_REG_WDT_WDTSTST                      0xd2

/* adc logical channel number */
#define BCM15602_ADC_SDLDO_CURR      0
#define BCM15602_ADC_IOLDO_CURR      1
#define BCM15602_ADC_ASR_SLV_CURR    2
#define BCM15602_ADC_ASR_MSTR_CURR   3
#define BCM15602_ADC_SDSR_CURR       4
#define BCM15602_ADC_RESERVED1       5
#define BCM15602_ADC_PTAT            6
#define BCM15602_ADC_RESERVED2       7
#define BCM15602_ADC_VIO             8
#define BCM15602_ADC_VSRC            9
#define BCM15602_ADC_VBAT            10
#define BCM15602_ADC_ANATEST         11

/* full scale of 10-bit adc slots */
#define BCM15602_ADC_SCALE_ASR_CURR     (3512000 / 1023) /* in uA/lsb */
#define BCM15602_ADC_SCALE_SDSR_CURR    (2106000 / 1023) /* in uA/lsb */
#define BCM15602_ADC_SCALE_SDLDO_CURR   (523000 / 1023)  /* in uA/lsb */
#define BCM15602_ADC_SCALE_IOLDO_CURR   (523000 / 1023)  /* in uA/lsb */
#define BCM15602_ADC_SCALE_VBAT         (4800000 / 1023) /* in uV/lsb */
#define BCM15602_ADC_SCALE_PTAT         (1200000 / 1023) /* in uV/lsb */

/* housekeeping slot numbers */
#define BCM15602_HK_ASR_MSTR_CURR_OVERI   0
#define BCM15602_HK_ASR_SLV_CURR_OVERI    1
#define BCM15602_HK_SDSR_CURR_OVERI       2
#define BCM15602_HK_SDLDO_CURR_OVERI      3
#define BCM15602_HK_IOLDO_CURR_OVERI      4
#define BCM15602_HK_VBAT_UNDERV0          5
#define BCM15602_HK_VBAT_UNDERV1          6
#define BCM15602_HK_VBAT_UNDERV2          7
#define BCM15602_HK_PTAT_OVERT            8
#define BCM15602_HK_PTAT_UNDERT           9
#define BCM15602_HK_NUM_SLOTS             10

/* macros for PTAT-to-temperature conversion */
#define TEMP_TO_PTAT_CODE(X)  ((X + 273150) / 563)
#define PTAT_CODE_TO_TEMP(X)  (X * 563 - 273150)

/* threshold for housekeeping slots */
#define BCM15602_HK_TH_ASR_MSTR_CURR   \
	(2500000 / BCM15602_ADC_SCALE_ASR_CURR)
#define BCM15602_HK_TH_ASR_SLV_CURR    \
	(2500000 / BCM15602_ADC_SCALE_ASR_CURR)
#define BCM15602_HK_TH_SDSR_CURR       \
	(1200000 / BCM15602_ADC_SCALE_SDSR_CURR)
#define BCM15602_HK_TH_SDLDO_CURR      \
	(100000 / BCM15602_ADC_SCALE_SDLDO_CURR)
#define BCM15602_HK_TH_IOLDO_CURR      \
	(100000 / BCM15602_ADC_SCALE_IOLDO_CURR)
#define BCM15602_HK_TH_VBAT_UNDERV0    \
	(3200000 / BCM15602_ADC_SCALE_VBAT)
#define BCM15602_HK_TH_VBAT_UNDERV1    \
	(3000000 / BCM15602_ADC_SCALE_VBAT)
#define BCM15602_HK_TH_VBAT_UNDERV2    \
	(2800000 / BCM15602_ADC_SCALE_VBAT)
#define BCM15602_HK_TH_PTAT_OVERT      TEMP_TO_PTAT_CODE(65000)
#define BCM15602_HK_TH_PTAT_UNDERT     TEMP_TO_PTAT_CODE(45000)

/* adc config state */
#define BCM15602_ADC_STATE_OFF      0
#define BCM15602_ADC_STATE_ACTIVE   1

/* adc config oversampling */
#define BCM15602_ADC_OVSP_1    0x00
#define BCM15602_ADC_OVSP_2    0x10
#define BCM15602_ADC_OVSP_4    0x20
#define BCM15602_ADC_OVSP_8    0x30
#define BCM15602_ADC_OVSP_16   0x40

/* adc config over/under threshold */
#define BCM15602_ADC_UNDER_TH   0
#define BCM15602_ADC_OVER_TH    1

/* interrupt flags */
#define BCM15602_INT_ASR_OVERI         0
#define BCM15602_INT_ASR_SHUTDOWN      1
#define BCM15602_INT_ASR_UV            2
#define BCM15602_INT_SDSR_OVERI        3
#define BCM15602_INT_SDSR_SHUTDOWN     4
#define BCM15602_INT_IOLDO_OVERI       8
#define BCM15602_INT_IOLDO_SHUTDOWN    9
#define BCM15602_INT_SDLDO_OVERI       11
#define BCM15602_INT_SDLDO_SHUTDOWN    12
#define BCM15602_INT_ADC_HK_INT        16
#define BCM15602_INT_ADC_CONV_DONE     17
#define BCM15602_INT_OTP_ECC_FAULT     24
#define BCM15602_INT_WDT_EXP           25
#define BCM15602_INT_WDT_ALARM         26
#define BCM15602_INT_THERM_TRIP        27
#define BCM15602_INT_THERM_TRIP_DONE   28

/* regulator names */
#define BCM15602_REGLTR_NAME_SDLDO "bcm15602_sdldo"
#define BCM15602_REGLTR_NAME_IOLDO "bcm15602_ioldo"
#define BCM15602_REGLTR_NAME_ASR "bcm15602_asr"
#define BCM15602_REGLTR_NAME_SDSR "bcm15602_sdsr"

/* silicon versions */
#define BCM15602_REV_ES 0
#define BCM15602_REV_A0 1
#define BCM15602_REV_A1 2

#define BCM15602_NUM_GPIOS 2

/* regulator id enum */
enum bcm15602_regulator_ids {
	BCM15602_ID_SDLDO,
	BCM15602_ID_IOLDO,
	BCM15602_ID_ASR,
	BCM15602_ID_SDSR,
	BCM15602_ID_ALL,
	BCM15602_NUM_REGULATORS = BCM15602_ID_ALL,
};

/* driver data structure */
struct bcm15602_chip {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_dev **rdevs;
	struct bcm15602_platform_data *pdata;
	u8 rev_id;
	u16 hk_status;
	bool wdt_enabled;

	/* kernel thread for waiting for reset after shutdown */
	struct work_struct reset_work;

	/* completion used for initialization */
	struct completion reset_complete;

	/* bitmask for tracking enabled regulators */
	unsigned long reg_enabled_mask;

	/* flags used for serialization */
	unsigned long adc_conv_busy;
	unsigned long hk_read_busy;

	/* completion used for signaling end of adc conversion */
	struct completion adc_conv_complete;
};

/* platform data structure */
struct bcm15602_platform_data {
	int pon_gpio;
	int resetb_gpio;
	int intb_gpio;
	unsigned int resetb_irq;
	unsigned int intb_irq;
};

/* adc channel configuration */
struct bcm15602_adc_ctrl {
	union {
		struct {
			u8 chan_num : 4;
			u8 ovsp     : 3;
			u8 state    : 1;
		};
		u8 ctrl;
	};
	union {
		struct  {
			u16 threshold : 10;
			u8 ot_ut      : 1;
			u8 deb_en     : 1;
			u8 th_en      : 1;
			u8 reserved   : 3;
		};
		u16 th_ctrl;
	};
};

int bcm15602_read_byte(struct bcm15602_chip *ddata, u8 addr, u8 *data);
int bcm15602_write_byte(struct bcm15602_chip *ddata, u8 addr, u8 data);
int bcm15602_update_bits(struct bcm15602_chip *ddata, u8 addr,
			 unsigned int mask, u8 data);
int bcm15602_read_adc_chan(struct bcm15602_chip *ddata,
			   int chan_num, u16 *chan_data);
int bcm15602_read_hk_slot(struct bcm15602_chip *ddata,
			  int slot_num, u16 *slot_data);

void bcm15602_config_sysfs(struct device *dev);

#endif /* BCM15602_REGULATOR_H */
