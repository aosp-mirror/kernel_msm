/*
 * Wireless Power Receiver driver for IDTP9017
 *
 * Copyright (C) 2014-2016 LGE Inc.
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

#ifndef __IDTP9017_WIRELESS_CHARGER_H
#define __IDTP9017_WIRELESS_CHARGER_H

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#ifndef BIT
#define BIT(x) (1 << (x))
#endif

#define IDTP9017_NAME		"idtp9017"

/* Register List */
/* Read abnormal state */
#define RDST_6A_H 	0xD4
#define RDST_6A_L 	0xD5

/* Die Temperature control */
#define REG_04_H 	0x08
#define REG_04_L 	0x09
/* Rectifier control */
#define REG_11_H 	0x22
#define REG_11_L 	0x23
#define REG_12_H 	0x24
#define REG_12_L 	0x25
/* Modulation Depth set */
#define REG_16_L 	0x2D

/* Charging state register */
#define REG_CHGSTS 	0xF7

/* FOD Gain and offset */
#define REG_18_H 	0x30
#define REG_18_L 	0x31
/* Control Register */
#define REG_19_H 	0x32
#define REG_19_L 	0x33
/* Read ADC register */
/* Read Vrect register */
#define RDST_30_H 	0x60
#define RDST_30_L 	0x61
/* Read Output Current register */
#define RDST_31_H 	0x62
#define RDST_31_L 	0x63
/* Read Output Voltage register */
#define RDST_32_H 	0x64
#define RDST_32_L 	0x65
/* Read Die temperature register */
#define RDST_33_H 	0x66
#define RDST_33_L 	0x67
/* Read Temperature sensor1 register */
#define RDST_34_L 	0x69
/* Read Temperature sensor1 register */
#define RDST_35_L 	0x6B
/* Read Align x-axis register */
#define RDST_36_L 	0x6D
/* Read Align y-axis register */
#define RDST_37_L 	0x6F

/* Read FOD1 register */
#define RDST_38_L 	0x71
/* Read FOD2 register */
#define RDST_39_L 	0x73
/* Read Limite Current register */
#define RDST_3A_L 	0x75
/* Read Target Out Voltage register */
#define RDST_3B_H 	0x76
#define RDST_3B_L 	0x77
/* Read Operating freq. */
#define REG_3F_H 	0x7E
#define REG_3F_L 	0x7F
/* Read Charging mode */
#define REG_CHG_MODE 	0x88

/* Write MASK NAME */
/* REG_04_H */
#define TDIE_SHDN_OFF 	(BIT(7) | BIT(6))
#define TDIE_SHDN_HYS 	BIT(5)
/* REG_04_L */
#define TDIE_THMR_OFF 	(BIT(7) | BIT(6))
#define TDIE_THMR_HYS 	BIT(5)

#define SHIFT_THRESHOLD_OFF 6
#define SHIFT_THRESHOLD_HYS 5

/* RDST_6A_H ~ RDST_6A_L */
#define ABNM_RAW_15 	BIT(7)	/* TX detection is fail */
#define ABNM_RAW_14 	BIT(6)  /* Too low freq */
#define ABNM_RAW_10 	BIT(2)  /* Over Vrect 8.5V */
#define ABNM_RAW_2 		BIT(2)  /* EOC state */
#define ABNM_RAW_0  	BIT(0)  /* Disable */


/* Retifier REG_11_H ~ REG_12_L */
#define DV_VMIN_PMA 	(BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define DV_VMIN_WPC 	(BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define DV_PARA1 		(BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define DV_PARA2 		(BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define RECT_MGN_PMA 	(BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define RECT_MGN_WPC 	(BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define RECT_VMAX 		(BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define RECT_VMIN 		(BIT(3) | BIT(2) | BIT(1) | BIT(0))

#define SHIFT_RECT 		4

/* REG_16_L */
#define MOD_DEP_WPC 	(BIT(7) | BIT(6) | BIT (5) | BIT(4))
#define MOD_DEP_PMA 	(BIT(3) | BIT(2) | BIT (1) | BIT(0))

#define SHIFT_WPC 		4

/* REG_CHGSTS */
#define CHGSTS_FRC (BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))

/* REG_18_H */
#define FOD1_EN 		BIT(7)
#define FOD1_VALUE 		(BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define FOD2_EN 		BIT(7)
#define FOD2_VALUE 		(BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))

#define SHIFT_EN 		7

/* REG_19_H */
#define ILIM_EN 		BIT(7)
#define ILIM_VALUE 		(BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
/* REG_19_L */
#define VSET_EN 		BIT(7)
#define VSET_VALUE 		(BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))

#define SHIFT_FOR_ADC 	8

/* RDST_30_H ~ RDST_3B_L */
#define SIGN_BIT 		BIT(7)
#define ADC_4BIT 		(BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define ADC_5BIT 		(BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define ADC_6BIT 		(BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define ADC_7BIT 		(BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))

struct input_ma_limit_entry {
	int  icl_ma;
	u8  value;
};

static struct input_ma_limit_entry icl_ma_table[] = {
	{1000, 0x00},
	{100, 0x01},
	{200, 0x02},
	{300, 0x03},
	{400, 0x04},
	{500, 0x05},
	{600, 0x06},
	{700, 0x08},
	{800, 0x10},
	{900, 0x12},
	{1100, 0x14},
	{1200, 0x17},
	{1300, 0x21},
	{1400, 0x25},
	{1500, 0x29},
	{1600, 0x31},
};
#endif
