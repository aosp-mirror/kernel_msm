/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR header.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _AIRBRUSH_DDR_INTERNAL_H_
#define _AIRBRUSH_DDR_INTERNAL_H_

#include <linux/mfd/abc-pcie.h>
#include "airbrush-otp.h"

/* ------------------------------------------------------------------------ */
/* Depending on the usage, please comment/uncomment the below config macros */
/* ------------------------------------------------------------------------ */
//#define CONFIG_DDR_VREF_DISABLE
//#define CONFIG_DDR_BOOT_TEST
/* ------------------------------------------------------------------------ */

#define pll_locktime_pll_phy_mif	0x10510008
#define pll_con0_pll_phy_mif		0x10510140
#define clk_con_div_dfi_div2		0x10511800
#define clk_con_div_div2_pllclk_mif	0x10511804
#define clk_con_div_div4_pllclk_mif	0x10511808
#define phy0_init_ctrl_reg		0x10530400
#define phy0_rst_ctrl_reg		0x10530404
#define phy1_init_ctrl_reg		0x10530500
#define phy1_rst_ctrl_reg		0x10530504
#define mif_pll_wrap_ctrl_reg		0x10530510

#define DREX_BASE_ADDR			0x10580000
#define DPHY_BASE_ADDR			0x105b0000
#define DPHY2_BASE_ADDR			0x105c0000

#define DREX_CONCONTROL			0x10580000
#define DREX_MEMCONTROL			0x10580004
#define DREX_DIRECTCMD			0x10580010
#define DREX_PRECHCONFIG0		0x10580014
#define DREX_PHYCONTROL0		0x10580018
#define DREX_TIMINGRFCPB		0x10580020
#define DREX_PWRDNCONFIG		0x10580028
#define DREX_TIMINGROW0			0x10580034
#define DREX_TIMINGDATA0		0x10580038
#define DREX_TIMINGPOWER0		0x1058003c
#define DREX_PHYSTATUS			0x10580040
#define DREX_ETCTIMING			0x10580044
#define DREX_CHIPSTATUS			0x10580048
#define DREX_RDFETCH0			0x1058004c
#define DREX_RDFETCH1			0x10580050
#define DREX_MRSTATUS			0x10580054
#define DREX_TIMINGSETSW		0x105800e0
#define DREX_TIMINGROW1			0x105800e4
#define DREX_TIMINGDATA1		0x105800e8
#define DREX_TIMINGPOWER1		0x105800ec
#define DREX_ALL_INIT_INDI		0x10580400
#define DREX_INIT_TRAIN_CONFIG		0x10580430
#define DREX_INIT_TRAIN_CONTROL		0x10580434
#define DREX_WRTRA_PATTERN0		0x10580460
#define DREX_WRTRA_PATTERN1		0x10580464
#define DREX_WRTRA_PATTERN2		0x10580468
#define DREX_DFIRSTCONTROL		0x10580708
#define DREX_ACTIVATE_AXI_READY		0x10580714
#define DREX_1CHIP_MASKING		0x1058076c
#define DREX_ASP_MEMBASECONFIG0		0x10590f00
#define DREX_ASP_MEMCONFIG0		0x10590f10
#define DREX_ASP_CHIP0SIZECONFIG	0x10590f20
#define DPHY_GNR_CON0			0x105b0000
#define DPHY_CAL_CON0			0x105b0004
#define DPHY_CAL_CON1			0x105b0008
#define DPHY_CAL_CON2			0x105b000c
#define DPHY_CAL_CON3			0x105b0010
#define DPHY_CAL_CON4			0x105b0014
#define DPHY_LP_CON0			0x105b0018
#define DPHY_GATE_CON0			0x105b001c
#define DPHY_OFFSETD_CON0		0x105b0050
#define DPHY_CAL_WR_PATTERN_CON0	0x105b0098
#define DPHY_CAL_WR_PATTERN_CON1	0x105b009c
#define DPHY_CAL_WR_PATTERN_CON2	0x105b00a0
#define DPHY_CAL_WR_PATTERN_CON3	0x105b00a4
#define DPHY_CAL_WR_PATTERN_CON4	0x105b00a8
#define DPHY_CAL_RD_PATTERN_CON0	0x105b00ac
#define DPHY_MDLL_CON0			0x105b00b0
#define DPHY_MDLL_CON1			0x105b00b4
#define DPHY_DVFS_CON			0x105b00b8
#define DPHY_ZQ_CON0			0x105b03c8
#define DPHY_ZQ_CON1			0x105b03cc
#define DPHY_ZQ_CON3			0x105b03d4
#define DPHY_ZQ_CON6			0x105b03e0
#define DPHY_ZQ_CON9			0x105b03ec
#define DPHY_TESTIRCV_CON0		0x105b0400
#define DPHY_CBT_CON0			0x105b0608
#define DPHY_PRBS_CON0			0x105b0684
#define DPHY_PRBS_CON1			0x105b0688
#define DPHY_PRBS_CON4			0x105b0694
#define DPHY_PRBS_CON5			0x105b0698
#define DPHY_PRBS_CON6			0x105b069c
#define DPHY_PRBS_CON7			0x105b06a0
#define DPHY_PRBS_CON8			0x105b06a4
#define DPHY_MON_CON0			0x105b0700

#define DPHY2_GNR_CON0			0x105c0000
#define DPHY2_CAL_CON0			0x105c0004
#define DPHY2_CAL_CON1			0x105c0008
#define DPHY2_CAL_CON2			0x105c000c
#define DPHY2_CAL_CON3			0x105c0010
#define DPHY2_CAL_CON4			0x105c0014
#define DPHY2_LP_CON0			0x105c0018
#define DPHY2_GATE_CON0			0x105c001c
#define DPHY2_OFFSETD_CON0		0x105c0050
#define DPHY2_CAL_WR_PATTERN_CON0	0x105c0098
#define DPHY2_CAL_WR_PATTERN_CON1	0x105c009c
#define DPHY2_CAL_WR_PATTERN_CON2	0x105c00a0
#define DPHY2_CAL_WR_PATTERN_CON3	0x105c00a4
#define DPHY2_CAL_WR_PATTERN_CON4	0x105c00a8
#define DPHY2_CAL_RD_PATTERN_CON0	0x105c00ac
#define DPHY2_MDLL_CON0			0x105c00b0
#define DPHY2_MDLL_CON1			0x105c00b4
#define DPHY2_DVFS_CON			0x105c00b8
#define DPHY2_ZQ_CON0			0x105c03c8
#define DPHY2_ZQ_CON1			0x105c03cc
#define DPHY2_ZQ_CON3			0x105c03d4
#define DPHY2_ZQ_CON6			0x105c03e0
#define DPHY2_ZQ_CON9			0x105c03ec
#define DPHY2_TESTIRCV_CON0		0x105c0400
#define DPHY2_CBT_CON0			0x105c0608
#define DPHY2_PRBS_CON0			0x105c0684
#define DPHY2_PRBS_CON1			0x105c0688
#define DPHY2_PRBS_CON4			0x105c0694
#define DPHY2_PRBS_CON5			0x105c0698
#define DPHY2_PRBS_CON6			0x105c069c
#define DPHY2_PRBS_CON7			0x105c06a0
#define DPHY2_PRBS_CON8			0x105c06a4
#define DPHY2_MON_CON0			0x105c0700

/* Register offsets used during PRBS training */
#define DPHY_OFFSET_PRBS_CON6		0x69c
#define DPHY_OFFSET_PRBS_CON7		0x6a0

#define pll_locktime_pll_aon		0x10b10000
#define pll_con0_pll_aon		0x10b10100
#define clk_con_div_div4_pllclk		0x10b11800
#define clk_con_div_div_otp		0x10b11804
#define clk_con_div_div_tmu		0x10b11808
#define clk_con_div_pll_aon_clk		0x10b1180c
#define clk_con_div_shared_div_aon_pll	0x10b11810
#define clk_con_div_shared_div_mif	0x10b11814

enum ddr_poll_index {
	p_pll_con0_pll_phy_mif,
	p_DPHY_ZQ_CON1,
	p_DPHY_MDLL_CON1,
	p_DREX_PHYSTATUS_dfi,
	p_DREX_PHYSTATUS_train,
	p_DREX_CHIPSTATUS_sr_enter,
	p_DREX_CHIPSTATUS_sr_exit,
	p_DPHY_PRBS_CON0_prbs_done,
	p_DPHY_PRBS_CON0_prbs_disable,
};

struct ddr_reg_poll_t {
	uint32_t mask;
	uint32_t val;
	uint32_t usec_timeout;
};

enum ddr_train_save_index {
	s_DPHY_MDLL_CON1,
	s_DPHY2_MDLL_CON1,
	s_DPHY_CA_DESKEW_CON0,
	s_DPHY_CA_DESKEW_CON1,
	s_DPHY_CA_DESKEW_CON2,
	s_DPHY2_CA_DESKEW_CON0,
	s_DPHY2_CA_DESKEW_CON1,
	s_DPHY2_CA_DESKEW_CON2,
	s_DPHY_RD_DESKEW_CENTER_CS0_CON_DM,
	s_DPHY_RD_DESKEW_CENTER_CS0_CON0,
	s_DPHY_RD_DESKEW_CENTER_CS0_CON1,
	s_DPHY_RD_DESKEW_CENTER_CS0_CON2,
	s_DPHY_RD_DESKEW_CENTER_CS0_CON3,
	s_DPHY_RD_DESKEW_CENTER_CS0_CON4,
	s_DPHY_RD_DESKEW_CENTER_CS0_CON5,
	s_DPHY_RD_DESKEW_CENTER_CS0_CON6,
	s_DPHY_RD_DESKEW_CENTER_CS0_CON7,
	s_DPHY_RD_DESKEW_CENTER_CS1_CON_DM,
	s_DPHY_RD_DESKEW_CENTER_CS1_CON0,
	s_DPHY_RD_DESKEW_CENTER_CS1_CON1,
	s_DPHY_RD_DESKEW_CENTER_CS1_CON2,
	s_DPHY_RD_DESKEW_CENTER_CS1_CON3,
	s_DPHY_RD_DESKEW_CENTER_CS1_CON4,
	s_DPHY_RD_DESKEW_CENTER_CS1_CON5,
	s_DPHY_RD_DESKEW_CENTER_CS1_CON6,
	s_DPHY_RD_DESKEW_CENTER_CS1_CON7,
	s_DPHY_RD_DESKEW_LEFT_CS0_CON_DM,
	s_DPHY_RD_DESKEW_LEFT_CS0_CON0,
	s_DPHY_RD_DESKEW_LEFT_CS0_CON1,
	s_DPHY_RD_DESKEW_LEFT_CS0_CON2,
	s_DPHY_RD_DESKEW_LEFT_CS0_CON3,
	s_DPHY_RD_DESKEW_LEFT_CS0_CON4,
	s_DPHY_RD_DESKEW_LEFT_CS0_CON5,
	s_DPHY_RD_DESKEW_LEFT_CS0_CON6,
	s_DPHY_RD_DESKEW_LEFT_CS0_CON7,
	s_DPHY_RD_DESKEW_LEFT_CS1_CON_DM,
	s_DPHY_RD_DESKEW_LEFT_CS1_CON0,
	s_DPHY_RD_DESKEW_LEFT_CS1_CON1,
	s_DPHY_RD_DESKEW_LEFT_CS1_CON2,
	s_DPHY_RD_DESKEW_LEFT_CS1_CON3,
	s_DPHY_RD_DESKEW_LEFT_CS1_CON4,
	s_DPHY_RD_DESKEW_LEFT_CS1_CON5,
	s_DPHY_RD_DESKEW_LEFT_CS1_CON6,
	s_DPHY_RD_DESKEW_LEFT_CS1_CON7,
	s_DPHY_RD_DQS_VWMC_CS0_CON0,
	s_DPHY_RD_DQS_VWMC_CS1_CON0,
	s_DPHY_RD_DQS_VWML_CS0_CON0,
	s_DPHY_RD_DQS_VWML_CS1_CON0,
	s_DPHY2_RD_DESKEW_CENTER_CS0_CON_DM,
	s_DPHY2_RD_DESKEW_CENTER_CS0_CON0,
	s_DPHY2_RD_DESKEW_CENTER_CS0_CON1,
	s_DPHY2_RD_DESKEW_CENTER_CS0_CON2,
	s_DPHY2_RD_DESKEW_CENTER_CS0_CON3,
	s_DPHY2_RD_DESKEW_CENTER_CS0_CON4,
	s_DPHY2_RD_DESKEW_CENTER_CS0_CON5,
	s_DPHY2_RD_DESKEW_CENTER_CS0_CON6,
	s_DPHY2_RD_DESKEW_CENTER_CS0_CON7,
	s_DPHY2_RD_DESKEW_CENTER_CS1_CON_DM,
	s_DPHY2_RD_DESKEW_CENTER_CS1_CON0,
	s_DPHY2_RD_DESKEW_CENTER_CS1_CON1,
	s_DPHY2_RD_DESKEW_CENTER_CS1_CON2,
	s_DPHY2_RD_DESKEW_CENTER_CS1_CON3,
	s_DPHY2_RD_DESKEW_CENTER_CS1_CON4,
	s_DPHY2_RD_DESKEW_CENTER_CS1_CON5,
	s_DPHY2_RD_DESKEW_CENTER_CS1_CON6,
	s_DPHY2_RD_DESKEW_CENTER_CS1_CON7,
	s_DPHY2_RD_DESKEW_LEFT_CS0_CON_DM,
	s_DPHY2_RD_DESKEW_LEFT_CS0_CON0,
	s_DPHY2_RD_DESKEW_LEFT_CS0_CON1,
	s_DPHY2_RD_DESKEW_LEFT_CS0_CON2,
	s_DPHY2_RD_DESKEW_LEFT_CS0_CON3,
	s_DPHY2_RD_DESKEW_LEFT_CS0_CON4,
	s_DPHY2_RD_DESKEW_LEFT_CS0_CON5,
	s_DPHY2_RD_DESKEW_LEFT_CS0_CON6,
	s_DPHY2_RD_DESKEW_LEFT_CS0_CON7,
	s_DPHY2_RD_DESKEW_LEFT_CS1_CON_DM,
	s_DPHY2_RD_DESKEW_LEFT_CS1_CON0,
	s_DPHY2_RD_DESKEW_LEFT_CS1_CON1,
	s_DPHY2_RD_DESKEW_LEFT_CS1_CON2,
	s_DPHY2_RD_DESKEW_LEFT_CS1_CON3,
	s_DPHY2_RD_DESKEW_LEFT_CS1_CON4,
	s_DPHY2_RD_DESKEW_LEFT_CS1_CON5,
	s_DPHY2_RD_DESKEW_LEFT_CS1_CON6,
	s_DPHY2_RD_DESKEW_LEFT_CS1_CON7,
	s_DPHY2_RD_DQS_VWMC_CS0_CON0,
	s_DPHY2_RD_DQS_VWMC_CS1_CON0,
	s_DPHY2_RD_DQS_VWML_CS0_CON0,
	s_DPHY2_RD_DQS_VWML_CS1_CON0,
	s_DPHY_WR_DESKEWC_CS0_CON0,
	s_DPHY_WR_DESKEWC_CS0_CON1,
	s_DPHY_WR_DESKEWC_CS0_CON2,
	s_DPHY_WR_DESKEWC_CS0_CON3,
	s_DPHY_WR_DESKEWC_CS0_CON4,
	s_DPHY_WR_DESKEWC_CS0_CON5,
	s_DPHY_WR_DESKEWC_CS0_CON6,
	s_DPHY_WR_DESKEWC_CS0_CON7,
	s_DPHY_DM_DESKEWC_CS0_CON0,
	s_DPHY_WR_DESKEWC_CS1_CON0,
	s_DPHY_WR_DESKEWC_CS1_CON1,
	s_DPHY_WR_DESKEWC_CS1_CON2,
	s_DPHY_WR_DESKEWC_CS1_CON3,
	s_DPHY_WR_DESKEWC_CS1_CON4,
	s_DPHY_WR_DESKEWC_CS1_CON5,
	s_DPHY_WR_DESKEWC_CS1_CON6,
	s_DPHY_WR_DESKEWC_CS1_CON7,
	s_DPHY_DM_DESKEWC_CS1_CON0,
	s_DPHY_WR_DESKEWL_CS0_CON0,
	s_DPHY_WR_DESKEWL_CS0_CON1,
	s_DPHY_WR_DESKEWL_CS0_CON2,
	s_DPHY_WR_DESKEWL_CS0_CON3,
	s_DPHY_WR_DESKEWL_CS0_CON4,
	s_DPHY_WR_DESKEWL_CS0_CON5,
	s_DPHY_WR_DESKEWL_CS0_CON6,
	s_DPHY_WR_DESKEWL_CS0_CON7,
	s_DPHY_DM_DESKEWL_CS0_CON0,
	s_DPHY_WR_DESKEWL_CS1_CON0,
	s_DPHY_WR_DESKEWL_CS1_CON1,
	s_DPHY_WR_DESKEWL_CS1_CON2,
	s_DPHY_WR_DESKEWL_CS1_CON3,
	s_DPHY_WR_DESKEWL_CS1_CON4,
	s_DPHY_WR_DESKEWL_CS1_CON5,
	s_DPHY_WR_DESKEWL_CS1_CON6,
	s_DPHY_WR_DESKEWL_CS1_CON7,
	s_DPHY_DM_DESKEWL_CS1_CON0,
	s_DPHY2_WR_DESKEWC_CS0_CON0,
	s_DPHY2_WR_DESKEWC_CS0_CON1,
	s_DPHY2_WR_DESKEWC_CS0_CON2,
	s_DPHY2_WR_DESKEWC_CS0_CON3,
	s_DPHY2_WR_DESKEWC_CS0_CON4,
	s_DPHY2_WR_DESKEWC_CS0_CON5,
	s_DPHY2_WR_DESKEWC_CS0_CON6,
	s_DPHY2_WR_DESKEWC_CS0_CON7,
	s_DPHY2_DM_DESKEWC_CS0_CON0,
	s_DPHY2_WR_DESKEWC_CS1_CON0,
	s_DPHY2_WR_DESKEWC_CS1_CON1,
	s_DPHY2_WR_DESKEWC_CS1_CON2,
	s_DPHY2_WR_DESKEWC_CS1_CON3,
	s_DPHY2_WR_DESKEWC_CS1_CON4,
	s_DPHY2_WR_DESKEWC_CS1_CON5,
	s_DPHY2_WR_DESKEWC_CS1_CON6,
	s_DPHY2_WR_DESKEWC_CS1_CON7,
	s_DPHY2_DM_DESKEWC_CS1_CON0,
	s_DPHY2_WR_DESKEWL_CS0_CON0,
	s_DPHY2_WR_DESKEWL_CS0_CON1,
	s_DPHY2_WR_DESKEWL_CS0_CON2,
	s_DPHY2_WR_DESKEWL_CS0_CON3,
	s_DPHY2_WR_DESKEWL_CS0_CON4,
	s_DPHY2_WR_DESKEWL_CS0_CON5,
	s_DPHY2_WR_DESKEWL_CS0_CON6,
	s_DPHY2_WR_DESKEWL_CS0_CON7,
	s_DPHY2_DM_DESKEWL_CS0_CON0,
	s_DPHY2_WR_DESKEWL_CS1_CON0,
	s_DPHY2_WR_DESKEWL_CS1_CON1,
	s_DPHY2_WR_DESKEWL_CS1_CON2,
	s_DPHY2_WR_DESKEWL_CS1_CON3,
	s_DPHY2_WR_DESKEWL_CS1_CON4,
	s_DPHY2_WR_DESKEWL_CS1_CON5,
	s_DPHY2_WR_DESKEWL_CS1_CON6,
	s_DPHY2_WR_DESKEWL_CS1_CON7,
	s_DPHY2_DM_DESKEWL_CS1_CON0,
	s_DPHY_PRBS_CON2,
	s_DPHY_PRBS_CON3,
	s_DPHY2_PRBS_CON2,
	s_DPHY2_PRBS_CON3,
	s_DPHY_ZQ_CON9,
	s_DPHY2_ZQ_CON9,
	s_train_max_index,
};

enum ddr_train_restore_read_index {
	r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON_DM,
	r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON0,
	r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON1,
	r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON2,
	r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON3,
	r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON4,
	r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON5,
	r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON6,
	r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON7,
	r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON_DM,
	r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON0,
	r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON1,
	r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON2,
	r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON3,
	r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON4,
	r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON5,
	r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON6,
	r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON7,
	r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON_DM,
	r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON0,
	r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON1,
	r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON2,
	r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON3,
	r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON4,
	r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON5,
	r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON6,
	r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON7,
	r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON_DM,
	r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON0,
	r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON1,
	r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON2,
	r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON3,
	r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON4,
	r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON5,
	r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON6,
	r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON7,
	r_DPHY_SW_RD_DQS_VWMC_CS0_CON0,
	r_DPHY_SW_RD_DQS_VWMC_CS1_CON0,
	r_DPHY_SW_RD_DQS_VWML_CS0_CON0,
	r_DPHY_SW_RD_DQS_VWML_CS1_CON0,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON_DM,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON0,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON1,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON2,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON3,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON4,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON5,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON6,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON7,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON_DM,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON0,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON1,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON2,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON3,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON4,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON5,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON6,
	r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON7,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON_DM,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON0,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON1,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON2,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON3,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON4,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON5,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON6,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON7,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON_DM,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON0,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON1,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON2,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON3,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON4,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON5,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON6,
	r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON7,
	r_DPHY2_SW_RD_DQS_VWMC_CS0_CON0,
	r_DPHY2_SW_RD_DQS_VWMC_CS1_CON0,
	r_DPHY2_SW_RD_DQS_VWML_CS0_CON0,
	r_DPHY2_SW_RD_DQS_VWML_CS1_CON0,
};

enum ddr_train_restore_write_index {
	r_DPHY_SW_WR_DESKEWC_CS0_CON0,
	r_DPHY_SW_WR_DESKEWC_CS0_CON1,
	r_DPHY_SW_WR_DESKEWC_CS0_CON2,
	r_DPHY_SW_WR_DESKEWC_CS0_CON3,
	r_DPHY_SW_WR_DESKEWC_CS0_CON4,
	r_DPHY_SW_WR_DESKEWC_CS0_CON5,
	r_DPHY_SW_WR_DESKEWC_CS0_CON6,
	r_DPHY_SW_WR_DESKEWC_CS0_CON7,
	r_DPHY_SW_DM_DESKEWC_CS0_CON0,
	r_DPHY_SW_WR_DESKEWC_CS1_CON0,
	r_DPHY_SW_WR_DESKEWC_CS1_CON1,
	r_DPHY_SW_WR_DESKEWC_CS1_CON2,
	r_DPHY_SW_WR_DESKEWC_CS1_CON3,
	r_DPHY_SW_WR_DESKEWC_CS1_CON4,
	r_DPHY_SW_WR_DESKEWC_CS1_CON5,
	r_DPHY_SW_WR_DESKEWC_CS1_CON6,
	r_DPHY_SW_WR_DESKEWC_CS1_CON7,
	r_DPHY_SW_DM_DESKEWC_CS1_CON0,
	r_DPHY_SW_WR_DESKEWL_CS0_CON0,
	r_DPHY_SW_WR_DESKEWL_CS0_CON1,
	r_DPHY_SW_WR_DESKEWL_CS0_CON2,
	r_DPHY_SW_WR_DESKEWL_CS0_CON3,
	r_DPHY_SW_WR_DESKEWL_CS0_CON4,
	r_DPHY_SW_WR_DESKEWL_CS0_CON5,
	r_DPHY_SW_WR_DESKEWL_CS0_CON6,
	r_DPHY_SW_WR_DESKEWL_CS0_CON7,
	r_DPHY_SW_DM_DESKEWL_CS0_CON0,
	r_DPHY_SW_WR_DESKEWL_CS1_CON0,
	r_DPHY_SW_WR_DESKEWL_CS1_CON1,
	r_DPHY_SW_WR_DESKEWL_CS1_CON2,
	r_DPHY_SW_WR_DESKEWL_CS1_CON3,
	r_DPHY_SW_WR_DESKEWL_CS1_CON4,
	r_DPHY_SW_WR_DESKEWL_CS1_CON5,
	r_DPHY_SW_WR_DESKEWL_CS1_CON6,
	r_DPHY_SW_WR_DESKEWL_CS1_CON7,
	r_DPHY_SW_DM_DESKEWL_CS1_CON0,
	r_DPHY2_SW_WR_DESKEWC_CS0_CON0,
	r_DPHY2_SW_WR_DESKEWC_CS0_CON1,
	r_DPHY2_SW_WR_DESKEWC_CS0_CON2,
	r_DPHY2_SW_WR_DESKEWC_CS0_CON3,
	r_DPHY2_SW_WR_DESKEWC_CS0_CON4,
	r_DPHY2_SW_WR_DESKEWC_CS0_CON5,
	r_DPHY2_SW_WR_DESKEWC_CS0_CON6,
	r_DPHY2_SW_WR_DESKEWC_CS0_CON7,
	r_DPHY2_SW_DM_DESKEWC_CS0_CON0,
	r_DPHY2_SW_WR_DESKEWC_CS1_CON0,
	r_DPHY2_SW_WR_DESKEWC_CS1_CON1,
	r_DPHY2_SW_WR_DESKEWC_CS1_CON2,
	r_DPHY2_SW_WR_DESKEWC_CS1_CON3,
	r_DPHY2_SW_WR_DESKEWC_CS1_CON4,
	r_DPHY2_SW_WR_DESKEWC_CS1_CON5,
	r_DPHY2_SW_WR_DESKEWC_CS1_CON6,
	r_DPHY2_SW_WR_DESKEWC_CS1_CON7,
	r_DPHY2_SW_DM_DESKEWC_CS1_CON0,
	r_DPHY2_SW_WR_DESKEWL_CS0_CON0,
	r_DPHY2_SW_WR_DESKEWL_CS0_CON1,
	r_DPHY2_SW_WR_DESKEWL_CS0_CON2,
	r_DPHY2_SW_WR_DESKEWL_CS0_CON3,
	r_DPHY2_SW_WR_DESKEWL_CS0_CON4,
	r_DPHY2_SW_WR_DESKEWL_CS0_CON5,
	r_DPHY2_SW_WR_DESKEWL_CS0_CON6,
	r_DPHY2_SW_WR_DESKEWL_CS0_CON7,
	r_DPHY2_SW_DM_DESKEWL_CS0_CON0,
	r_DPHY2_SW_WR_DESKEWL_CS1_CON0,
	r_DPHY2_SW_WR_DESKEWL_CS1_CON1,
	r_DPHY2_SW_WR_DESKEWL_CS1_CON2,
	r_DPHY2_SW_WR_DESKEWL_CS1_CON3,
	r_DPHY2_SW_WR_DESKEWL_CS1_CON4,
	r_DPHY2_SW_WR_DESKEWL_CS1_CON5,
	r_DPHY2_SW_WR_DESKEWL_CS1_CON6,
	r_DPHY2_SW_WR_DESKEWL_CS1_CON7,
	r_DPHY2_SW_DM_DESKEWL_CS1_CON0,
};

#define DDR_POLL_USLEEP_MIN		100
/* Minimum RESET_n LOW time after completion of voltage ramp */
#define DDR_INIT_TIMING_tINIT1_USEC	400
/* Minimum CKE low time after RESET_n high */
#define DDR_INIT_TIMING_tINIT3_USEC	2000
/* CKE High to CS delay */
#define DDR_INIT_CKE2CS_DELAY_USEC	100

#define ddr_usleep(usec)	(usleep_range(usec, usec + 1))

static inline uint32_t ddr_reg_rd(uint32_t addr)
{
	uint32_t data = 0xffffffff;

	/* TODO(b/121225073): Add synchronization and fail check */
	WARN_ON(abc_pcie_config_read(addr & 0xFFFFFF, 0x4, &data));

	return data;
}

static inline void ddr_reg_wr(uint32_t addr, uint32_t data)
{
	/* TODO(b/121225073): Add synchronization and fail check */
	WARN_ON(abc_pcie_config_write(addr & 0xFFFFFF, 0x4, data));
}

static inline void ddr_reg_set(uint32_t addr, uint32_t mask)
{
	ddr_reg_wr(addr, ddr_reg_rd(addr) | mask);
}

static inline void ddr_reg_clr(uint32_t addr, uint32_t mask)
{
	ddr_reg_wr(addr, ddr_reg_rd(addr) & (~mask));
}

static inline uint32_t ddr_mem_rd(uint32_t addr)
{
	uint32_t data = 0xffffffff;

	/* TODO(b/121225073): Add synchronization and fail check */
	WARN_ON(memory_config_read(addr, 0x4, &data));

	return data;
}

static inline void ddr_mem_wr(uint32_t addr, uint32_t data)
{
	/* TODO(b/121225073): Add synchronization and fail check */
	WARN_ON(memory_config_write(addr, 0x4, data));
}

void ddr_prbs_training_init(void);

#define VREF_REF_NUM		0x05
#define PHY_VREF_LEVELS		64
#define DRAM_VREF_LEVELS	51
#define VREF_FROM		0x0
#define VREF_STEP		0x1

#define VREF_PRBS_TIMEOUT_USEC	5000

enum vref_operation_t {
	VREF_READ = 0,
	VREF_WRITE
};

enum vref_byte_t {
	VREF_BYTE0 = 0,
	VREF_BYTE1,
	VREF_BYTE_ALL
};

enum vref_error_t {
	VREF_ERROR = DDR_FAIL,
	VREF_SUCCESS = DDR_SUCCESS,
	VREF_TIMEOUT
};

enum vref_prbs_t {
	VREF_PRBS_SUCCESS = 0,
	VREF_PRBS_TIMEOUT
};

enum phy_type_t {
	PHY0 = 0,
	PHY1
};

/* Little Endian representation of the register fields */
union dphy_cal_con0_t {
	uint32_t n;
	struct {
		 uint32_t wrlvl_mode                : (0 - 0 + 1);
		 uint32_t gate_cal_mode             : (1 - 1 + 1);
		 uint32_t ca_cal_mode               : (2 - 2 + 1);
		 uint32_t rd_cal_mode               : (3 - 3 + 1);
		 uint32_t lock_average_en           : (4 - 4 + 1);
		 uint32_t wr_cal_mode               : (5 - 5 + 1);
		 uint32_t rdlvl_dqs_edge_en         : (7 - 6 + 1);
		 uint32_t wrlvl_start               : (8 - 8 + 1);
		 uint32_t wrtrn_dqs_edge_en         : (10 - 9 + 1);
		 uint32_t lock_sample_condition     : (12 - 11 + 1);
		 uint32_t ctrl_upd_interval         : (14 - 13 + 1);
		 uint32_t reserved_15_15            : (15 - 15 + 1);
		 uint32_t wrlvl_resp                : (16 - 16 + 1);
		 uint32_t reserved_17_17            : (17 - 17 + 1);
		 uint32_t wr_per_rank_en            : (19 - 18 + 1);
		 uint32_t byte_rdlvl_en             : (20 - 20 + 1);
		 uint32_t ca_swap_mode              : (21 - 21 + 1);
		 uint32_t cal_vtc_en                : (22 - 22 + 1);
		 uint32_t freq_offset_en            : (23 - 23 + 1);
		 uint32_t avg_window_size           : (25 - 24 + 1);
		 uint32_t dvfs_wr_train_en          : (26 - 26 + 1);
		 uint32_t gate_rdchk_en             : (27 - 27 + 1);
		 uint32_t cs_default                : (31 - 28 + 1);
	} bits;
};

union dphy_zq_con9_t {
	uint32_t n;
	struct {
		 uint32_t zq_ds0_vref               : (5 - 0 + 1);
		 uint32_t zq_ds0_vref_fsbst         : (6 - 6 + 1);
		 uint32_t zq_ds0_vref_pd            : (7 - 7 + 1);
		 uint32_t zq_ds1_vref               : (13 - 8 + 1);
		 uint32_t zq_ds1_vref_fsbst         : (14 - 14 + 1);
		 uint32_t zq_ds1_vref_pd            : (15 - 15 + 1);
		 uint32_t reserved_16_31            : (31 - 16 + 1);
	} bits;
};

union dphy_prbs_con0_t {
	uint32_t n;
	struct {
		 uint32_t prbs_done                 : (0 - 0 + 1);
		 uint32_t prbs_read_start           : (1 - 1 + 1);
		 uint32_t prbs_write_start          : (2 - 2 + 1);
		 uint32_t reserved_3_15             : (15 - 3 + 1);
		 uint32_t prbs_pattern              : (31 - 16 + 1);
	} bits;
};

union dphy_prbs_con1_t {
	uint32_t n;
	struct {
		 uint32_t prbs_tresync              : (4 - 0 + 1);
		 uint32_t prbs_trddata_en_adj       : (8 - 5 + 1);
		 uint32_t reserved_9_12             : (12 - 9 + 1);
		 uint32_t prbs_twr2rd               : (17 - 13 + 1);
		 uint32_t prbs_wrlat                : (23 - 18 + 1);
		 uint32_t reserved_24_25            : (25 - 24 + 1);
		 uint32_t prbs_dbi_en               : (26 - 26 + 1);
		 uint32_t reserved_27_27            : (27 - 27 + 1);
		 uint32_t prbs_cs                   : (31 - 28 + 1);
	} bits;
};

union dphy_prbs_con6_t {
	uint32_t n;
	struct {
		 uint32_t prbs_offset_left0         : (8 - 0 + 1);
		 uint32_t reserved_9_15             : (15 - 9 + 1);
		 uint32_t prbs_offset_left1         : (24 - 16 + 1);
		 uint32_t reserved_31_25            : (31 - 25 + 1);
	} bits;
};

union dphy_prbs_con7_t {
	uint32_t n;
	struct {
		 uint32_t prbs_offset_right0        : (8 - 0 + 1);
		 uint32_t reserved_9_15             : (15 - 9 + 1);
		 uint32_t prbs_offset_right1        : (24 - 16 + 1);
		 uint32_t reserved_31_25            : (31 - 25 + 1);
	} bits;
};

struct ab_ddr_context {
	struct ab_state_context *ab_state_ctx;
	enum ddr_state ddr_state;
	uint32_t ddr_train_save_value[s_train_max_index];
};

int32_t ddrphy_run_vref_training(struct ab_ddr_context *ctx);

#endif /* _AIRBRUSH_DDR_INTERNAL_H_ */
