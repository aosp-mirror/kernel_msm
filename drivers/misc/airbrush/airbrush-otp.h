/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR OTP description and helper functions.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _AIRBRUSH_DDR_OTP_H_
#define _AIRBRUSH_DDR_OTP_H_

#include "airbrush-regs.h"

enum ddr_otp_idx {
	/* Important: The below are contigious registers in OTP_WRAPPER starting
	 * from offset 0x7000
	 */
	o_DREX_1CHIP_MASKING_0,
	o_DREX_1CHIP_MASKING_1,
	o_DREX_1CHIP_MASKING_2,
	o_DREX_CONCONTROL_0,
	o_DREX_CONCONTROL_1,
	o_DREX_DFIRSTCONTROL_0,
	o_DREX_DFIRSTCONTROL_1,
	o_DREX_DIRECTCMD_0,
	o_DREX_DIRECTCMD_1,
	o_DREX_DIRECTCMD_2,
	o_DREX_DIRECTCMD_3,
	o_DREX_DIRECTCMD_4,
	o_DREX_DIRECTCMD_5,
	o_DREX_DIRECTCMD_6,
	o_DREX_DIRECTCMD_7,
	o_DREX_DIRECTCMD_8,
	o_DREX_DIRECTCMD_9,
	o_DREX_DIRECTCMD_10,
	o_DREX_DIRECTCMD_11,
	o_DREX_DIRECTCMD_12,
	o_DREX_DIRECTCMD_13,
	o_DREX_DIRECTCMD_14,
	o_DREX_DIRECTCMD_15,
	o_DREX_DIRECTCMD_16,
	o_DREX_DIRECTCMD_17,
	o_DREX_DIRECTCMD_18,
	o_DREX_DIRECTCMD_19,
	o_DREX_DIRECTCMD_20,
	o_DREX_DIRECTCMD_21,
	o_DREX_DIRECTCMD_22,
	o_DREX_DIRECTCMD_23,
	o_DREX_DIRECTCMD_24,
	o_DREX_DIRECTCMD_25,
	o_DREX_DIRECTCMD_26,
	o_DREX_DIRECTCMD_27,
	o_DREX_DIRECTCMD_28,
	o_DREX_DIRECTCMD_29,
	o_DREX_DIRECTCMD_30,
	o_DPHY_CAL_CON0_0,
	o_DPHY_CAL_CON0_1,
	o_DPHY_CAL_CON0_2,
	o_DPHY_CAL_CON0_3,
	o_DPHY_CAL_CON0_4,
	o_DPHY_CAL_CON0_5,
	o_DPHY_CAL_CON2_0,
	o_DPHY_CAL_CON2_1,
	o_DPHY_CAL_CON2_2,
	o_DPHY_CAL_CON2_3,
	o_DPHY_CAL_CON2_4,
	o_DPHY_CAL_CON2_5,
	o_DPHY_CAL_WR_PATTERN_CON4_0,
	o_DPHY_CAL_WR_PATTERN_CON4_1,
	o_DPHY_GNR_CON0_0,
	o_DPHY_GNR_CON0_1,
	o_DPHY_GNR_CON0_2,
	o_DPHY_GNR_CON0_3,
	o_DPHY_LP_CON0_0,
	o_DPHY_LP_CON0_1,
	o_DPHY_LP_CON0_2,
	o_DPHY_MDLL_CON0_0,
	o_DPHY_MDLL_CON0_1,
	o_DPHY_MDLL_CON0_2,
	o_DPHY_MDLL_CON0_3,
	o_DPHY_OFFSETD_CON0_0,
	o_DPHY_OFFSETD_CON0_1,
	o_DPHY_ZQ_CON0_0,
	o_DPHY_ZQ_CON0_1,
	o_DPHY_ZQ_CON0_2,
	o_DPHY_ZQ_CON0_3,
	o_DPHY_ZQ_CON0_4,
	o_DPHY_ZQ_CON0_5,
	o_DPHY_ZQ_CON0_6,
	o_DPHY_ZQ_CON0_7,
	o_DPHY_ZQ_CON0_8,
	o_DPHY_ZQ_CON0_9,
	o_DPHY_ZQ_CON0_10,
	o_DPHY_ZQ_CON0_11,
	o_DPHY_ZQ_CON0_12,
	o_DPHY_ZQ_CON0_13,
	o_DPHY_ZQ_CON0_14,
	o_DPHY_ZQ_CON0_15,
	o_DPHY_ZQ_CON0_16,
	o_DPHY_ZQ_CON3_0,
	o_DPHY_ZQ_CON3_1,
	o_DPHY_ZQ_CON3_2,
	o_DPHY_ZQ_CON3_3,
	o_DPHY_ZQ_CON6_0,
	o_DPHY_ZQ_CON6_1,
	o_DPHY_ZQ_CON6_2,
	o_DPHY_ZQ_CON9_0,
	o_DPHY_ZQ_CON9_1,
	o_DPHY_ZQ_CON9_2,
	o_DPHY_ZQ_CON9_3,
	o_DREX_INIT_TRAIN_CONFIG_0,
	o_DREX_INIT_TRAIN_CONFIG_1,
	o_DREX_INIT_TRAIN_CONFIG_2,
	o_DREX_INIT_TRAIN_CONFIG_3,
	o_DREX_INIT_TRAIN_CONTROL_0,
	o_DREX_INIT_TRAIN_CONTROL_1,
	o_DREX_PHYCONTROL0_0,
	o_DREX_PHYCONTROL0_1,
	o_DREX_PRECHCONFIG0_0,
	o_DREX_PRECHCONFIG0_1,
	o_DREX_TIMINGDATA0_0,
	o_DREX_TIMINGDATA0_1,
	o_DREX_TIMINGDATA1_0,
	o_DREX_TIMINGDATA1_1,
	o_Reserved_DDR_INIT_0,
	o_Reserved_DDR_INIT_1,
	o_Reserved_DDR_INIT_2,
	o_Reserved_DDR_INIT_3,
	o_Reserved_DDR_INIT_4,
	o_Reserved_DDR_INIT_5,
	o_Reserved_DDR_INIT_6,
	o_Reserved_DDR_INIT_7,
	o_Reserved_DDR_INIT_8,
	o_Reserved_DDR_INIT_9,
	o_Reserved_DDR_INIT_10,
	o_Reserved_DDR_INIT_11,
	o_Reserved_DDR_INIT_12,
	o_Reserved_DDR_INIT_13,
	o_Reserved_DDR_INIT_14,
	o_Reserved_DDR_INIT_15,
	o_Reserved_DDR_INIT_16,
	o_Reserved_DDR_INIT_17,
	o_Reserved_DDR_INIT_18,
	o_Reserved_DDR_INIT_19,
	o_Reserved_DDR_INIT_20,

	/* Important: The below are contigious registers in OTP_WRAPPER starting
	 * from offset 0x7488
	 */
	o_PCIe_reg_75,
	o_PCIe_reg_address_75,
	o_PCIe_reg_76,
	o_PCIe_reg_address_76,
	o_PCIe_reg_77,
	o_PCIe_reg_address_77,
	o_PCIe_reg_78,
	o_PCIe_reg_address_78,
	o_PCIe_reg_79,
	o_PCIe_reg_address_79,

	/* Important: The below are contigious registers in OTP_WRAPPER starting
	 * from offset 0x6000
	 */
	o_SECURE_JTAG0,
	o_SECURE_JTAG1,
	o_SECURE_JTAG2,
	o_SECURE_JTAG3,
	o_SECURE_JTAG4,
	o_SECURE_JTAG5,

	o_DDR_OTP_MAX,
};

/* ddr_otp_values[] is used to simulate the OTP memory when the OTP is not
 * actually loaded to system.
 */
static uint32_t ddr_otp_values[o_DDR_OTP_MAX] __attribute__ ((section(".otpddr")))= {
	[o_DREX_1CHIP_MASKING_0] = 0x0,
	[o_DREX_1CHIP_MASKING_1] = 0x1,
	[o_DREX_1CHIP_MASKING_2] = 0x2,
	[o_DREX_CONCONTROL_0] = 0x8fff0102,
	[o_DREX_CONCONTROL_1] = 0x9fff8102,
	[o_DREX_DFIRSTCONTROL_0] = 0x0,
	[o_DREX_DFIRSTCONTROL_1] = 0x1,
	[o_DREX_DIRECTCMD_0] = 0x10c90,
	[o_DREX_DIRECTCMD_1] = 0x11070,
	[o_DREX_DIRECTCMD_10] = 0x21850,
	[o_DREX_DIRECTCMD_11] = 0x300013c,
	[o_DREX_DIRECTCMD_12] = 0x3000144,
	[o_DREX_DIRECTCMD_13] = 0x311070,
	[o_DREX_DIRECTCMD_14] = 0x311838,
	[o_DREX_DIRECTCMD_15] = 0x4000000,
	[o_DREX_DIRECTCMD_16] = 0x40154,
	[o_DREX_DIRECTCMD_17] = 0x5b8,
	[o_DREX_DIRECTCMD_18] = 0x418,
	[o_DREX_DIRECTCMD_19] = 0x502a8,
	[o_DREX_DIRECTCMD_2] = 0x111070,
	[o_DREX_DIRECTCMD_20] = 0x6000000,
	[o_DREX_DIRECTCMD_21] = 0x7000000,
	[o_DREX_DIRECTCMD_22] = 0x8000000,
	[o_DREX_DIRECTCMD_23] = 0x8d8,
	[o_DREX_DIRECTCMD_24] = 0x86c,
	[o_DREX_DIRECTCMD_25] = 0x9000000,
	[o_DREX_DIRECTCMD_26] = 0x9001400,
	[o_DREX_DIRECTCMD_27] = 0x9010000,
	[o_DREX_DIRECTCMD_28] = 0x9101400,
	[o_DREX_DIRECTCMD_29] = 0x9201400,
	[o_DREX_DIRECTCMD_3] = 0x111838,
	[o_DREX_DIRECTCMD_30] = 0xfc4,
	[o_DREX_DIRECTCMD_4] = 0x11420,
	[o_DREX_DIRECTCMD_5] = 0x11838,
	[o_DREX_DIRECTCMD_6] = 0x11d54,
	[o_DREX_DIRECTCMD_7] = 0x211070,
	[o_DREX_DIRECTCMD_8] = 0x211838,
	[o_DREX_DIRECTCMD_9] = 0x212a8,
	[o_DPHY_CAL_CON0_0] = 0x780806c0,
	[o_DPHY_CAL_CON0_1] = 0x780806c1,
	[o_DPHY_CAL_CON0_2] = 0x780806c4,
	[o_DPHY_CAL_CON0_3] = 0x780806c5,
	[o_DPHY_CAL_CON0_4] = 0x780806cc,
	[o_DPHY_CAL_CON0_5] = 0x780806ec,
	[o_DPHY_CAL_CON2_0] = 0xaa200000,
	[o_DPHY_CAL_CON2_1] = 0xaa201000,
	[o_DPHY_CAL_CON2_2] = 0xaa250000,
	[o_DPHY_CAL_CON2_3] = 0xaa251000,
	[o_DPHY_CAL_CON2_4] = 0xaa270000,
	[o_DPHY_CAL_CON2_5] = 0x44050000,
	[o_DPHY_CAL_WR_PATTERN_CON4_0] = 0x55aa55aa,
	[o_DPHY_CAL_WR_PATTERN_CON4_1] = 0x5555,
	[o_DPHY_GNR_CON0_0] = 0x43005020,
	[o_DPHY_GNR_CON0_1] = 0x43106024,
	[o_DPHY_GNR_CON0_2] = 0x47106024,
	[o_DPHY_GNR_CON0_3] = 0x47106016,
	[o_DPHY_LP_CON0_0] = 0x400c,
	[o_DPHY_LP_CON0_1] = 0x400f,
	[o_DPHY_LP_CON0_2] = 0xf,
	[o_DPHY_MDLL_CON0_0] = 0x10000130,
	[o_DPHY_MDLL_CON0_1] = 0x10000150,
	[o_DPHY_MDLL_CON0_2] = 0x10000170,
	[o_DPHY_MDLL_CON0_3] = 0x10800170,
	[o_DPHY_OFFSETD_CON0_0] = 0x1000000,
	[o_DPHY_OFFSETD_CON0_1] = 0x11000000,
	[o_DPHY_ZQ_CON0_0] =  0xff807304,
	[o_DPHY_ZQ_CON0_1] =  0xff807304,
	[o_DPHY_ZQ_CON0_10] = 0xff847304,
	[o_DPHY_ZQ_CON0_11] = 0xff847306,
	[o_DPHY_ZQ_CON0_12] = 0xff047304,
	[o_DPHY_ZQ_CON0_13] = 0xff807304,
	[o_DPHY_ZQ_CON0_14] = 0xff807306,
	[o_DPHY_ZQ_CON0_15] = 0xff847304,
	[o_DPHY_ZQ_CON0_16] = 0xff847306,
	[o_DPHY_ZQ_CON0_2] =  0xff807304,
	[o_DPHY_ZQ_CON0_3] =  0xff047304,
	[o_DPHY_ZQ_CON0_4] =  0xff007300,
	[o_DPHY_ZQ_CON0_5] =  0xff007302,
	[o_DPHY_ZQ_CON0_6] =  0xff047300,
	[o_DPHY_ZQ_CON0_7] =  0xff047302,
	[o_DPHY_ZQ_CON0_8] =  0xff807304,
	[o_DPHY_ZQ_CON0_9] =  0xff807306,
	[o_DPHY_ZQ_CON3_0] =  0x003f3f3f,
	[o_DPHY_ZQ_CON3_1] =  0x003f3f3f,
	[o_DPHY_ZQ_CON3_2] =  0x003f3f3f,
	[o_DPHY_ZQ_CON3_3] =  0x003f3f3f,
	[o_DPHY_ZQ_CON6_0] = 0x101,
	[o_DPHY_ZQ_CON6_1] = 0x2020,
	[o_DPHY_ZQ_CON6_2] = 0x2121,
	[o_DPHY_ZQ_CON9_0] = 0x4040,
	[o_DPHY_ZQ_CON9_1] = 0x5454,
	[o_DPHY_ZQ_CON9_2] = 0x1414,
	[o_DREX_INIT_TRAIN_CONFIG_0] = 0x0,
	[o_DREX_INIT_TRAIN_CONFIG_2] = 0x2,
	[o_DREX_INIT_TRAIN_CONFIG_3] = 0x4,
	[o_DREX_INIT_TRAIN_CONTROL_0] = 0x0,
	[o_DREX_INIT_TRAIN_CONTROL_1] = 0x1,
	[o_DREX_PHYCONTROL0_0] = 0x80,
	[o_DREX_PHYCONTROL0_1] = 0x88,
	[o_DREX_PRECHCONFIG0_0] = 0x0,
	[o_DREX_PRECHCONFIG0_1] = 0xf0000,
	[o_DREX_TIMINGDATA0_0] = 0x6a408464,
	[o_DREX_TIMINGDATA1_0] = 0x6a408464,
	[o_Reserved_DDR_INIT_0] = 0xff,
	[o_Reserved_DDR_INIT_1] = 0x82470300,
	[o_Reserved_DDR_INIT_10] = 0x402844,
	[o_Reserved_DDR_INIT_11] = 0x432323,
	[o_Reserved_DDR_INIT_12] = 0x0,
	[o_Reserved_DDR_INIT_13] = 0xc02844,
	[o_Reserved_DDR_INIT_14] = 0x8c02844,
	[o_Reserved_DDR_INIT_15] = 0x100,
	[o_Reserved_DDR_INIT_16] = 0x1,
	[o_Reserved_DDR_INIT_17] = 0xea201000,
	[o_Reserved_DDR_INIT_18] = 0xfc7f9800,
	[o_Reserved_DDR_INIT_19] = 0x55aa55aa,
	[o_Reserved_DDR_INIT_2] = 0x82470310,
	[o_Reserved_DDR_INIT_20] = 0x7f08004d,
	[o_Reserved_DDR_INIT_3] = 0x1,
	[o_Reserved_DDR_INIT_4] = 0x1,
	[o_Reserved_DDR_INIT_5] = 0x274a,
	[o_Reserved_DDR_INIT_6] = 0x23a5,
	[o_Reserved_DDR_INIT_7] = 0x3370,
	[o_Reserved_DDR_INIT_8] = 0x6330504,
	[o_Reserved_DDR_INIT_9] = 0x6330504,

	[o_PCIe_reg_75] = 0x55aa55aa,
	[o_PCIe_reg_76] = 0x55aa55aa,
	[o_PCIe_reg_77] = 0x33,
	[o_PCIe_reg_78] = 0x80F00510,
	[o_PCIe_reg_79] = 0x0,
	[o_PCIe_reg_address_75] = 0x55aa55aa,
	[o_PCIe_reg_address_76] = 0x8000,
	[o_PCIe_reg_address_77] = 0x80F00500,
	[o_PCIe_reg_address_78] = 0x3,
	[o_PCIe_reg_address_79] = 0x542010c,

	[o_SECURE_JTAG0] = 0x81f40306,
	[o_SECURE_JTAG1] = 0x81f40316,
	[o_SECURE_JTAG2] = 0x0,
	[o_SECURE_JTAG3] = 0x00c3004f,
	[o_SECURE_JTAG4] = 0x11,
	[o_SECURE_JTAG5] = 0x3,
};

/* --------------------------------------------------
 * 1.7.1.65 AP_HW_OPTION_1
 *    Base Address: 0x10BB_0000
 *    Address = Base Address + 0x4014, Reset Value = 0x0000_0000
 *    OTP_DBG_DIS[2] (OTP_FW_PATCH_DIS)
 *    OTP_UART_PRINT_DISABLE[3] (Disable UART prints when set)
 *    OTP_DDR_OTP_FLASHED[7]
 *    OTP_AP_DDR [9] (0: M0_DDR_INIT, 1: HOST_DDR_INIT)
 * -------------------------------------------------- */
#define OTP_AP_HW_OPTION_1()	(RD_REG(ABC_BASE_OTP_WRAPPER + 0x4014))
#define IS_OTP_FW_PATCH_DIS()	(!!(OTP_AP_HW_OPTION_1() & (1 << 2)))
#define IS_UART_PRINT_DISABLE() (!!(OTP_AP_HW_OPTION_1() & (1 << 3)))
#define IS_DDR_OTP_FLASHED()	(!!(OTP_AP_HW_OPTION_1() & (1 << 7)))
#define IS_M0_DDR_INIT()	( !(OTP_AP_HW_OPTION_1() & (1 << 9)))

static inline uint32_t read_otp_wrapper(uint32_t offset)
{
	if (offset < o_PCIe_reg_75)
		return RD_REG(ABC_BASE_OTP_WRAPPER + 0x7000 + (offset * 4));
	else if (offset < o_SECURE_JTAG0)
		return RD_REG(ABC_BASE_OTP_WRAPPER + 0x7488 + ((offset - o_PCIe_reg_75) * 4));
	else
		return RD_REG(ABC_BASE_OTP_WRAPPER + 0x6000 + ((offset - o_SECURE_JTAG0) * 4));
}

static inline uint32_t read_otp_array(uint32_t offset)
{
	return ddr_otp_values[offset];
}

#endif /* _AIRBRUSH_DDR_OTP_H_ */
