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
 */

#ifndef __MAX17055_BATTERY_H_
#define __MAX17055_BATTERY_H_

#define MAX17055_DEFAULT_SNS_RESISTOR	(10000)
#define MAX17055_CHARACTERIZATION_DATA_SIZE 48

enum max17055_register {
	MAX17055_Status		= 0x00,
	MAX17055_VAlrtTh	= 0x01,
	MAX17055_TAlrtTh	= 0x02,
	MAX17055_SAlrtTh	= 0x03,
	MAX17055_AtRate		= 0x04,
	MAX17055_RepCap		= 0x05,
	MAX17055_RepSOC		= 0x06,
	MAX17055_Age		= 0x07,
	MAX17055_Temp		= 0x08,
	MAX17055_VCell		= 0x09,
	MAX17055_Current	= 0x0A,
	MAX17055_AvgCurrent	= 0x0B,

	MAX17055_MixSOC		= 0x0D,
	MAX17055_AvSOC		= 0x0E,
	MAX17055_MixCap		= 0x0F,
	MAX17055_FullCapRep	= 0x10,
	MAX17055_TTE		= 0x11,
	MAX17055_QRTbl00	= 0x12,
	MAX17055_FullSocThr = 0x13,
	MAX17055_RCell		= 0x14,

	MAX17055_AvgTA		= 0x16,
	MAX17055_Cycles		= 0x17,
	MAX17055_DesignCap	= 0x18,
	MAX17055_AvgVCell	= 0x19,
	MAX17055_MaxMinTemp	= 0x1A,
	MAX17055_MaxMinVolt	= 0x1B,
	MAX17055_MaxMinCurr	= 0x1C,
	MAX17055_Config		= 0x1D,
	MAX17055_IChgTerm	= 0x1E,
	MAX17055_AvCap		= 0x1F,
	MAX17055_TTF		= 0x20,
	MAX17055_DevName	= 0x21,
	MAX17055_QRTbl10	= 0x22,
	MAX17055_FullCapNom	= 0x23,

	MAX17055_AIN0		= 0x27,
	MAX17055_LearnCfg	= 0x28,
	MAX17055_FilterCfg	= 0x29,
	MAX17055_RelaxCfg	= 0x2A,
	MAX17055_MiscCfg	= 0x2B,
	MAX17055_TGain		= 0x2C,
	MAx17055_TOff		= 0x2D,
	MAX17055_CGain		= 0x2E,
	MAX17055_COff		= 0x2F,
	MAX17055_QRTbl20	= 0x32,
	MAX17055_DieTemp	= 0x34,
	MAX17055_FullCap    = 0x35,
	MAX17055_IAvgEmpty	= 0x36,
	MAX17055_FCTC		= 0x37,
	MAX17055_RComp0		= 0x38,
	MAX17055_TempCo		= 0x39,
	MAX17055_VEmpty		= 0x3A,
	MAX17055_TaskPeriod	= 0x3C,
	MAX17055_FStat		= 0x3D,
	MAX17055_Timer		= 0x3E,
	MAX17055_ShdnTimer	= 0x3F,
	MAX17055_UserMem1	= 0x40,
	MAX17055_QRTbl30	= 0x42,
	MAX17055_dQacc		= 0x45,
	MAX17055_dPacc		= 0x46,
	MAX17055_VFSOC0		= 0x48,
	MAX17055_ConvgCfg	= 0x49,
	MAX17055_QH0		= 0x4C,
	MAX17055_QH			= 0x4D,
	MAX17055_QL			= 0x4E,
	MAX17055_Command	= 0x60,
	MAX17055_MLockReg1		= 0x62,
	MAX17055_MLockReg2		= 0x63,
	MAX17055_ModelChrTbl	= 0x80,
	MAX17055_Status2	= 0xB0,
	MAX17055_Power		= 0xB1,
	MAX17055_AvgPower	= 0xB3,
	MAX17055_IAlrtTh	= 0xB4,
	MAX17055_TCurve		= 0xB9,
	MAX17055_HibCfg		= 0xBA,
	MAX17055_Config2	= 0xBB,
	MAX17055_ModelCfg	= 0xDB,
	MAX17055_VFOCV		= 0xFB,
	MAX17055_VFSOC		= 0xFF,
};

enum max17055_chip_type {
	MAXIM_DEVICE_TYPE_UNKNOWN	= 0,
	MAXIM_DEVICE_TYPE_MAX17055A,
	MAXIM_DEVICE_TYPE_MAX17055B,

	MAXIM_DEVICE_TYPE_NUM
};

enum max17055_config_type {
	MODELGAUGE_CONFIG_TYPE_UNKNOWN	= 0,
	MODELGAUGE_CONFIG_TYPE_EZ,
	MODELGAUGE_CONFIG_TYPE_SHORT,
	MODELGAUGE_CONFIG_TYPE_FULL
};

/*
 * used for setting a register to a desired value
 * addr : address for a register
 * data : setting value for the register
 */
struct max17055_reg_data {
	u8 addr;
	u16 data;
};

struct max17055_config_data {
	/* A/D measurement */
	u16	tgain;		/* 0x2C */
	u16	toff;		/* 0x2D */
	u16 tcurve;		/* 0xB9 */
	u16	cgain;		/* 0x2E */
	u16	coff;		/* 0x2F */

	/* Alert / Status */
	u16 ialrt_thresh;		/* 0xB4 */
	u16	valrt_thresh;		/* 0x01 */
	u16	talrt_thresh;		/* 0x02 */
	u16	salrt_thresh;		/* 0x03 */
	u16	config;				/* 0x1D */
	u16 config2;			/* 0xBB */
	u16	shdntimer;			/* 0x3F */

	/* App data */
	u16	full_soc_thresh;	/* 0x13 */
	u16 rep_cap;			/* 0x05 */
	u16	design_cap;			/* 0x18 */
	u16	ichgt_term;			/* 0x1E */
	u16	iavg_empty;			/* 0x36 */
	u16	vempty;				/* 0x3A */
	u16 model_cfg;			/* 0xDB */

	/* config */
	u16	at_rate;	/* 0x04 */
	u16	learn_cfg;	/* 0x28 */
	u16	filter_cfg;	/* 0x29 */
	u16	relax_cfg;	/* 0x2A */
	u16	misc_cfg;	/* 0x2B */
	u16 hib_cfg;	/* 0xBA */
	u16 convg_cfg;  /* 0x49 */

	/* Cell Data */
	u16	rcomp0;		/* 0x38 */
	u16	tcompc0;	/* 0x39 */
	u16	qrtbl00;	/* 0x12 */
	u16	qrtbl10;	/* 0x22 */
	u16	qrtbl20;	/* 0x32 */
	u16	qrtbl30;	/* 0x42 */

	/* Temp Config */
	u16 qrtbl00n10;
	u16 qrtbl10n10;
	u16 qrtbl00n20;
	u16 qrtbl10n20;

	/* Model Data */
	u16	cell_char_tbl[MAX17055_CHARACTERIZATION_DATA_SIZE];
} __packed;

struct max17055_platform_data {
	struct max17055_reg_data *init_data;
	struct max17055_config_data *config_data;
	int num_init_data; /* Number of enties in init_data array */
	int config_type; /* The configuration options */
	bool enable_current_sense;
	bool enable_por_init; /* Use POR init from Maxim appnote */

	/*
	 * R_sns in micro-ohms.
	 * default 10000 (if r_sns = 0) as it is the recommended value by
	 * the datasheet although it can be changed by board designers.
	 */
	unsigned int r_sns;
	int         vmin;	/* in millivolts */
	int         vmax;	/* in millivolts */
	int         temp_min;	/* in tenths of degree Celsius */
	int         temp_max;	/* in tenths of degree Celsius */
};

#endif /* __MAX17055_BATTERY_H_ */
