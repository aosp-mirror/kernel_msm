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
//#define CONFIG_ZEBU_EMULATION
//#define CONFIG_IGNORE_DFI_STATUS
//#define CONFIG_DDR_VREF_DISABLE
//#define CONFIG_DDR_BOOT_TEST
/* ------------------------------------------------------------------------ */

#define __CONST 	const

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
#define reg_CONCONTROL			0x10580000
#define reg_MEMCONTROL			0x10580004
#define reg_DIRECTCMD			0x10580010
#define reg_PRECHCONFIG0		0x10580014
#define reg_PHYCONTROL0			0x10580018
#define reg_TIMINGRFCPB			0x10580020
#define reg_PWRDNCONFIG			0x10580028
#define reg_TIMINGROW0			0x10580034
#define reg_TIMINGDATA0			0x10580038
#define reg_TIMINGPOWER0		0x1058003c
#define reg_PHYSTATUS			0x10580040
#define reg_ETCTIMING			0x10580044
#define reg_CHIPSTATUS			0x10580048
#define reg_RDFETCH0			0x1058004c
#define reg_RDFETCH1			0x10580050
#define reg_MRSTATUS			0x10580054
#define reg_TIMINGSETSW			0x105800e0
#define reg_TIMINGROW1			0x105800e4
#define reg_TIMINGDATA1			0x105800e8
#define reg_TIMINGPOWER1		0x105800ec
#define reg_ALL_INIT_INDI		0x10580400
#define reg_INIT_TRAIN_CONFIG		0x10580430
#define reg_INIT_TRAIN_CONTROL		0x10580434
#define reg_WRTRA_PATTERN0		0x10580460
#define reg_WRTRA_PATTERN1		0x10580464
#define reg_WRTRA_PATTERN2		0x10580468
#define reg_DFIRSTCONTROL		0x10580708
#define reg_ACTIVATE_AXI_READY		0x10580714
#define reg_1CHIP_MASKING		0x1058076c
#define reg_ASP_MEMBASECONFIG0		0x10590f00
#define reg_ASP_MEMCONFIG0		0x10590f10
#define reg_ASP_CHIP0SIZECONFIG		0x10590f20
#define reg_DPHY_GNR_CON0		0x105b0000
#define reg_DPHY_CAL_CON0		0x105b0004
#define reg_DPHY_CAL_CON2		0x105b000c
#define reg_DPHY_CAL_CON3		0x105b0010
#define reg_DPHY_CAL_CON4		0x105b0014
#define reg_DPHY_LP_CON0		0x105b0018
#define reg_DPHY_GATE_CON0		0x105b001c
#define reg_DPHY_OFFSETD_CON0		0x105b0050
#define reg_DPHY_CAL_WR_PATTERN_CON0	0x105b0098
#define reg_DPHY_CAL_WR_PATTERN_CON1	0x105b009c
#define reg_DPHY_CAL_WR_PATTERN_CON2	0x105b00a0
#define reg_DPHY_CAL_WR_PATTERN_CON3	0x105b00a4
#define reg_DPHY_CAL_WR_PATTERN_CON4	0x105b00a8
#define reg_DPHY_CAL_RD_PATTERN_CON0	0x105b00ac
#define reg_DPHY_MDLL_CON0		0x105b00b0
#define reg_DPHY_MDLL_CON1		0x105b00b4
#define reg_DPHY_DVFS_CON		0x105b00b8
#define reg_DPHY_ZQ_CON0		0x105b03c8
#define reg_DPHY_ZQ_CON1		0x105b03cc
#define reg_DPHY_ZQ_CON3		0x105b03d4
#define reg_DPHY_ZQ_CON6		0x105b03e0
#define reg_DPHY_ZQ_CON9		0x105b03ec
#define reg_DPHY_TESTIRCV_CON0		0x105b0400
#define reg_DPHY_CBT_CON0		0x105b0608
#define reg_DPHY_PRBS_CON0		0x105b0684
#define reg_DPHY_PRBS_CON1		0x105b0688
#define reg_DPHY_PRBS_CON4		0x105b0694
#define reg_DPHY_PRBS_CON5		0x105b0698
#define reg_DPHY_PRBS_CON6		0x105b069c
#define reg_DPHY_PRBS_CON7		0x105b06a0
#define reg_DPHY_PRBS_CON8		0x105b06a4
#define reg_DPHY_MON_CON0		0x105b0700
#define reg_DPHY2_GNR_CON0		0x105c0000
#define reg_DPHY2_CAL_CON0		0x105c0004
#define reg_DPHY2_CAL_CON2		0x105c000c
#define reg_DPHY2_CAL_CON3		0x105c0010
#define reg_DPHY2_CAL_CON4		0x105c0014
#define reg_DPHY2_LP_CON0		0x105c0018
#define reg_DPHY2_GATE_CON0		0x105c001c
#define reg_DPHY2_OFFSETD_CON0		0x105c0050
#define reg_DPHY2_CAL_WR_PATTERN_CON0	0x105c0098
#define reg_DPHY2_CAL_WR_PATTERN_CON1	0x105c009c
#define reg_DPHY2_CAL_WR_PATTERN_CON2	0x105c00a0
#define reg_DPHY2_CAL_WR_PATTERN_CON3	0x105c00a4
#define reg_DPHY2_CAL_WR_PATTERN_CON4	0x105c00a8
#define reg_DPHY2_CAL_RD_PATTERN_CON0	0x105c00ac
#define reg_DPHY2_MDLL_CON0		0x105c00b0
#define reg_DPHY2_MDLL_CON1		0x105c00b4
#define reg_DPHY2_DVFS_CON		0x105c00b8
#define reg_DPHY2_ZQ_CON0		0x105c03c8
#define reg_DPHY2_ZQ_CON1		0x105c03cc
#define reg_DPHY2_ZQ_CON3		0x105c03d4
#define reg_DPHY2_ZQ_CON6		0x105c03e0
#define reg_DPHY2_ZQ_CON9		0x105c03ec
#define reg_DPHY2_TESTIRCV_CON0		0x105c0400
#define reg_DPHY2_CBT_CON0		0x105c0608
#define reg_DPHY2_PRBS_CON0		0x105c0684
#define reg_DPHY2_PRBS_CON1		0x105c0688
#define reg_DPHY2_PRBS_CON4		0x105c0694
#define reg_DPHY2_PRBS_CON5		0x105c0698
#define reg_DPHY2_PRBS_CON6		0x105c069c
#define reg_DPHY2_PRBS_CON7		0x105c06a0
#define reg_DPHY2_PRBS_CON8		0x105c06a4
#define reg_DPHY2_MON_CON0		0x105c0700
#define pll_locktime_pll_aon		0x10b10000
#define pll_con0_pll_aon		0x10b10100
#define clk_con_div_div4_pllclk		0x10b11800
#define clk_con_div_div_otp		0x10b11804
#define clk_con_div_div_tmu		0x10b11808
#define clk_con_div_pll_aon_clk		0x10b1180c
#define clk_con_div_shared_div_aon_pll	0x10b11810
#define clk_con_div_shared_div_mif	0x10b11814
#define reg_PMU_CONTROL			0x10ba0004

enum ddr_poll_index {
	p_pll_con0_pll_phy_mif,
	p_reg_DPHY_ZQ_CON1,
	p_reg_DPHY2_ZQ_CON1,
	p_reg_DPHY_MDLL_CON1,
	p_reg_DPHY2_MDLL_CON1,
	p_reg_MRSTATUS,
	p_reg_PHYSTATUS_dfi,
	p_reg_PHYSTATUS_train,
	p_reg_CHIPSTATUS_sr_enter,
	p_reg_CHIPSTATUS_sr_exit,
	p_reg_DPHY_PRBS_CON0_prbs_done,
	p_reg_DPHY_PRBS_CON0_prbs_disable,
};

typedef struct __ddr_reg_control {
	uint32_t flags;
	uint32_t reg;
	uint32_t val;
} ddr_reg_control_t;

typedef struct __reg_ddr_poll {
	uint32_t mask;
	uint32_t val;
	uint32_t usec_timeout;
} ddr_reg_poll_t;

typedef struct __reg_ddr_blk_reg_control {
	__CONST ddr_reg_control_t 	*blk;
} reg_ddr_blk_reg_control_t;

#define FLG_OTP			(1 << 0)
#define FLG_POLL		(1 << 1)
#define FLG_WAIT		(1 << 2)	/* wait time in usec */
#define FLG_SET			(1 << 3)
#define FLG_RESET		(1 << 4)
#define FLG_STATE_SUSPEND	(1 << 5)
#define FLG_NON_DIRECT		(0x1F)		//(FLG_OTP | FLG_POLL | FLG_WAIT | FLG_SET | FLG_RESET)

typedef enum __reg_ddr_blks {
	b_config_PLL_MIF_via_CMU_MIF_for_LowFreq,
	b_config_PLL_MIF_via_CMU_MIF_for_HighFreq,
	b_ungate_PHY_clock,
	b_deassert_PHY_reset_while_PHY_is_gated,
	b_initialize_PHY_pre,
	b_initialize_PHY,
	b_initialize_DFI,
	b_DRAM_reset_sequence,
	b_Power_down_exit_sequence,
	b_MRWs_Set_VREF_ODT_etc,
	b_ZQ_Calibration,
	b_IO_Initialization,
	b_Enable_DLL,
	b_Set_DREX_timing_parameters,
	b_Set_DREX_address_parameters,
	b_prepare_training,
	b_CA_training,
	b_ODT_training,
	b_AutoDQS_clean_Gate_training,
	b_Read_DQ_Calibration,
	b_Write_DQ_Calibration,
	b_PRBS_training_init,
	b_PRBS_training_read,
	b_PRBS_training_write,
	b_AXI_Enable_After_All_training,
	b_Enter_Self_Refresh_mode,
	b_Exit_Self_Refresh_mode,
} reg_ddr_blks_t;

__CONST ddr_reg_poll_t ddr_reg_poll[] = {
	[p_pll_con0_pll_phy_mif]	= { 0x20000000, 0x20000000, 5000 },
#ifdef CONFIG_ZEBU_EMULATION
	[p_reg_DPHY_ZQ_CON1]		= { 0x00000000, 0x00000000, 0 },	/* zq_done[0] bit */
	[p_reg_DPHY2_ZQ_CON1]		= { 0x00000000, 0x00000000, 0 },	/* zq_done[0] bit */
#else
	[p_reg_DPHY_ZQ_CON1]		= { 0x00000001, 0x00000001, 5000 },	/* zq_done[0] bit */
	[p_reg_DPHY2_ZQ_CON1]		= { 0x00000001, 0x00000001, 5000 },	/* zq_done[0] bit */
#endif
	[p_reg_DPHY_MDLL_CON1]		= { 0x00040000, 0x00040000, 5000 },	/* ctrl_locked[18] bit */
	[p_reg_DPHY2_MDLL_CON1]		= { 0x00040000, 0x00040000, 5000 },	/* ctrl_locked[18] bit */
	[p_reg_MRSTATUS]		= { 0x00000000, 0x00000000, 5000 },	/* JUST READ THE REGISTER ??? */
#ifdef CONFIG_IGNORE_DFI_STATUS
	[p_reg_PHYSTATUS_dfi]		= { 0x00000000, 0x00000000, 0 },	/* dfi_init_complete[4:3] */
#else
	[p_reg_PHYSTATUS_dfi]		= { 0x00000018, 0x00000018, 5000 },	/* dfi_init_complete[4:3] */
#endif
	[p_reg_PHYSTATUS_train]		= { 0x0000C000, 0x0000C000, 5000 },	/* train_complete[15:14] */
	[p_reg_CHIPSTATUS_sr_enter]	= { 0x00000100, 0x00000100, 5000 },	/* chip_sref_state[8]=1 */
	[p_reg_CHIPSTATUS_sr_exit]	= { 0x00000100, 0x00000000, 5000 },	/* chip_sref_state[8]=0 */
	[p_reg_DPHY_PRBS_CON0_prbs_done]	= { 0x00000001, 0x00000001, 5000 },	/* prbs_done[0] = 1 */
	[p_reg_DPHY_PRBS_CON0_prbs_disable]	= { 0x00000001, 0x00000000, 5000 },	/* prbs_disable[0] = 0 */
};

__CONST ddr_reg_control_t config_PLL_MIF_via_CMU_MIF_for_LowFreq[] = {
	{ FLG_OTP,	pll_locktime_pll_phy_mif,	o_Reserved_DDR_INIT_0 },
	{ FLG_OTP,	pll_con0_pll_phy_mif,		o_SECURE_JTAG0 },
	{ FLG_POLL,	pll_con0_pll_phy_mif,		p_pll_con0_pll_phy_mif },
	{ FLG_OTP,	pll_con0_pll_phy_mif,		o_SECURE_JTAG1 },
	{ FLG_OTP,	clk_con_div_div4_pllclk_mif,	o_Reserved_DDR_INIT_3 },
	{ FLG_OTP,	clk_con_div_div2_pllclk_mif,	o_Reserved_DDR_INIT_4 },
	{ FLG_OTP,	clk_con_div_dfi_div2,		o_Reserved_DDR_INIT_3 },
	{ 0, 		0,				0 },
};

__CONST ddr_reg_control_t config_PLL_MIF_via_CMU_MIF_for_HighFreq[] = {
	{ FLG_SET,	reg_MEMCONTROL,		0x00000001 },
	{ 0x0,		mif_pll_wrap_ctrl_reg,	0x0000000a },
	{ 0x0,		pll_con0_pll_phy_mif,	0x0 },
	{ FLG_OTP,	pll_con0_pll_phy_mif,	o_Reserved_DDR_INIT_1 },
	{ FLG_POLL,	pll_con0_pll_phy_mif,	p_pll_con0_pll_phy_mif },
	{ FLG_OTP,	pll_con0_pll_phy_mif,	o_Reserved_DDR_INIT_2 },
	{ 0x0,		mif_pll_wrap_ctrl_reg,	0x0000000e },
	{ FLG_RESET,	reg_MEMCONTROL,		0x00000001 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t ungate_PHY_clock[] = {
	{ 0x0,		mif_pll_wrap_ctrl_reg, 	0x00000008 },
	{ 0x0,		mif_pll_wrap_ctrl_reg,	0x0000000a },
	{ 0x0,		mif_pll_wrap_ctrl_reg,	0x0000000e },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t deassert_PHY_reset_while_PHY_is_gated[] = {
	{ 0x0,		phy0_init_ctrl_reg,	0x00008021 },
	{ 0x0,		phy1_init_ctrl_reg,	0x00008021 },
	{ 0x0,		phy0_rst_ctrl_reg,	0x00008080 },
	{ 0x0,		phy1_rst_ctrl_reg,	0x00008080 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t initialize_PHY_pre[] = {
	{ FLG_OTP,	reg_ACTIVATE_AXI_READY,	o_Reserved_DDR_INIT_12 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0 },
	{ FLG_OTP,	reg_CONCONTROL,		o_reg_CONCONTROL_0 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t initialize_PHY[] = {
	{ FLG_OTP,	reg_DPHY_MON_CON0, 	o_PCIe_reg_address_76 },
	{ FLG_OTP,	reg_DPHY2_MON_CON0,	o_PCIe_reg_address_76 },
	{ FLG_OTP,	reg_CONCONTROL,		o_reg_CONCONTROL_0 },
	{ FLG_OTP,	reg_DPHY_DVFS_CON,	o_Reserved_DDR_INIT_5 },
	{ FLG_OTP,	reg_DPHY2_DVFS_CON,	o_Reserved_DDR_INIT_5 },
	{ FLG_OTP,	reg_DPHY_GNR_CON0,	o_reg_DPHY_GNR_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_GNR_CON0,	o_reg_DPHY_GNR_CON0_0 },
	{ FLG_OTP,	reg_DPHY_CAL_CON0,	o_reg_DPHY_CAL_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON0,	o_reg_DPHY_CAL_CON0_0 },
	{ FLG_OTP,	reg_DPHY_CAL_CON2,	o_Reserved_DDR_INIT_17 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON2,	o_Reserved_DDR_INIT_17 },
	{ FLG_OTP,	reg_DPHY_LP_CON0,	o_reg_DPHY_LP_CON0_2 },
	{ FLG_OTP,	reg_DPHY2_LP_CON0,	o_reg_DPHY_LP_CON0_2 },
	{ FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_12 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_12 },
	{ FLG_OTP,	reg_DPHY_ZQ_CON3,	o_reg_DPHY_ZQ_CON3_2 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON3,	o_reg_DPHY_ZQ_CON3_2 },
	{ FLG_OTP,	reg_DPHY_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_1 },
	{ FLG_OTP,	reg_DPHY2_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_1 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t initialize_DFI[] = {
	{ FLG_OTP,	reg_CONCONTROL,		o_reg_CONCONTROL_1 },
	{ FLG_POLL,	reg_PHYSTATUS,		p_reg_PHYSTATUS_dfi },
	{ FLG_RESET,	reg_CONCONTROL,		0x10008000 },
	{ FLG_OTP,	reg_DPHY_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_1 },
	{ FLG_OTP,	reg_DPHY2_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_1 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t DRAM_reset_sequence[] = {
	/* RESET_DRAM */
	{ FLG_OTP,	reg_DFIRSTCONTROL,	o_reg_DFIRSTCONTROL_1 },
	{ FLG_WAIT,	400,			400 },
	{ FLG_OTP,	reg_DFIRSTCONTROL,	o_reg_DFIRSTCONTROL_0 },
	{ FLG_WAIT,	2000,			2000 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t Power_down_exit_sequence[] = {
	/* ExitPD start */
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_21 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t MRWs_Set_VREF_ODT_etc[] = {
	/* LPDDR4_chip_Init */
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_17 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_23 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_30 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_0 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_4 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_10 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_27 },
	{ 0, 		0,			0 },
};


/* From OFF -> ACTIVE the ZQ_Calibration is always required.
 * From SLEEP -> ACTIVE the ZQ_Calibration is not needed.
 * From SUSPEND -> ACTIVE the ZQ_Calibration is always required.
 */
__CONST ddr_reg_control_t ZQ_Calibration[] = {
	/* LPDDR4_ZQ_Cal */
	{ FLG_OTP,	reg_1CHIP_MASKING,	o_reg_1CHIP_MASKING_2 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_11 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_12 },
	{ FLG_OTP,	reg_1CHIP_MASKING,	o_reg_1CHIP_MASKING_0 },
	{ FLG_OTP,	reg_1CHIP_MASKING,	o_reg_1CHIP_MASKING_1 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_11 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_12 },
	{ FLG_OTP,	reg_1CHIP_MASKING,	o_reg_1CHIP_MASKING_0 },

	/* DRAM DCTL Resync */
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_1 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t IO_Initialization[] = {
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_3 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_12 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_1 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_12 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_12 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_12 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_15 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_3 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_12 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_1 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_12 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_12 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_12 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_15 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_15 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_15 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_15 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_15 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_16 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_16 },

	/* wait zq calibration */
	{ FLG_STATE_SUSPEND | FLG_POLL,	reg_DPHY_ZQ_CON1,	p_reg_DPHY_ZQ_CON1 },
	{ FLG_STATE_SUSPEND | FLG_POLL,	reg_DPHY2_ZQ_CON1,	p_reg_DPHY2_ZQ_CON1 },

	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_14 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_14 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_13 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_13 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_CONCONTROL,		o_reg_CONCONTROL_0 },
	{ FLG_OTP,			reg_DIRECTCMD,		o_reg_DIRECTCMD_0 },
	{ FLG_OTP,			reg_DIRECTCMD,		o_reg_DIRECTCMD_30 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON3,	o_reg_DPHY_ZQ_CON3_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON3,	o_reg_DPHY_ZQ_CON3_3 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON3,	o_reg_DPHY_ZQ_CON3_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON3,	o_reg_DPHY_ZQ_CON3_3 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_1 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON3,	o_reg_DPHY_ZQ_CON3_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON3,	o_reg_DPHY_ZQ_CON3_1 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_1 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON3,	o_reg_DPHY_ZQ_CON3_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON3,	o_reg_DPHY_ZQ_CON3_1 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_8 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_8 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_10 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_10 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_11 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_11 },

	/* wait zq calibration */
	{ FLG_STATE_SUSPEND | FLG_POLL,	reg_DPHY_ZQ_CON1,	p_reg_DPHY_ZQ_CON1 },
	{ FLG_STATE_SUSPEND | FLG_POLL,	reg_DPHY2_ZQ_CON1,	p_reg_DPHY2_ZQ_CON1 },

	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_9 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_9 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_8 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_8 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_CONCONTROL,		o_reg_CONCONTROL_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON9,	o_reg_DPHY_ZQ_CON9_1 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON9,	o_reg_DPHY_ZQ_CON9_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON9,	o_reg_DPHY_ZQ_CON9_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON9,	o_reg_DPHY_ZQ_CON9_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON9,	o_reg_DPHY_ZQ_CON9_1 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON9,	o_reg_DPHY_ZQ_CON9_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON9,	o_reg_DPHY_ZQ_CON9_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON9,	o_reg_DPHY_ZQ_CON9_2 },
	{ FLG_OTP,			reg_DIRECTCMD,		o_reg_DIRECTCMD_1 },
	{ FLG_OTP,			reg_DIRECTCMD,		o_reg_DIRECTCMD_2 },
	{ FLG_OTP,			reg_DIRECTCMD,		o_reg_DIRECTCMD_7 },
	{ FLG_OTP,			reg_DIRECTCMD,		o_reg_DIRECTCMD_13 },
	{ FLG_OTP,			reg_DIRECTCMD,		o_reg_DIRECTCMD_5 },
	{ FLG_OTP,			reg_DIRECTCMD,		o_reg_DIRECTCMD_3 },
	{ FLG_OTP,			reg_DIRECTCMD,		o_reg_DIRECTCMD_8 },
	{ FLG_OTP,			reg_DIRECTCMD,		o_reg_DIRECTCMD_14 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_8 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_8 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_10 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_10 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_11 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_11 },

	/* wait zq calibration */
	{ FLG_STATE_SUSPEND | FLG_POLL,	reg_DPHY_ZQ_CON1,	p_reg_DPHY_ZQ_CON1 },
	{ FLG_STATE_SUSPEND | FLG_POLL,	reg_DPHY2_ZQ_CON1,	p_reg_DPHY2_ZQ_CON1 },

	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_9 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_9 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_8 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_8 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_CONCONTROL,		o_reg_CONCONTROL_0 },

	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY_ZQ_CON9,	o_reg_DPHY_ZQ_CON9_2 },
	{ FLG_STATE_SUSPEND | FLG_OTP,	reg_DPHY2_ZQ_CON9,	o_reg_DPHY_ZQ_CON9_2 },
	{ 0, 				0,			0 },
};

__CONST ddr_reg_control_t Enable_DLL[] = {
	{ FLG_OTP,	reg_DPHY_GNR_CON0,	o_reg_DPHY_GNR_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_GNR_CON0,	o_reg_DPHY_GNR_CON0_0 },
	{ 0x0,		reg_DPHY_GATE_CON0,	0xf00ffff },
	{ 0x0,		reg_DPHY2_GATE_CON0, 	0xf00ffff },
	{ FLG_OTP,	reg_DPHY_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_2 },
	{ FLG_OTP,	reg_DPHY2_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_2 },
	{ FLG_OTP,	reg_DPHY_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_0 },
	{ FLG_OTP,	reg_DPHY_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_2 },
	{ FLG_OTP,	reg_DPHY2_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_2 },

	/* Poll for ctrl_lock */
	{ FLG_POLL,	reg_DPHY_MDLL_CON1, 	p_reg_DPHY_MDLL_CON1 },
	{ FLG_POLL,	reg_DPHY2_MDLL_CON1,	p_reg_DPHY2_MDLL_CON1 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t Set_DREX_timing_parameters[] = {

	/* SET_DREX */
	{ FLG_OTP,	reg_PRECHCONFIG0,	o_reg_PRECHCONFIG0_0 },
	{ 0x0,		reg_PWRDNCONFIG,	0xffff00ff },
	{ 0x0,		reg_TIMINGSETSW,	0x1 },
	{ 0x0,		reg_TIMINGRFCPB,	0x4242 },
	{ 0x0,		reg_TIMINGROW0,		0x836ba815 },
	{ 0x0,		reg_TIMINGROW1,		0x836ba815 },
	{ FLG_OTP,	reg_TIMINGDATA0,	o_reg_TIMINGDATA0_0 },
	{ FLG_OTP,	reg_TIMINGDATA1,	o_reg_TIMINGDATA1_0 },
	{ 0x0,		reg_TIMINGPOWER0,	0x4c880471 },
	{ 0x0,		reg_TIMINGPOWER1,	0x4c880471 },
	{ FLG_OTP,	reg_ETCTIMING,		o_Reserved_DDR_INIT_7 },
	{ FLG_OTP,	reg_RDFETCH0,		o_Reserved_DDR_INIT_8 },
	{ FLG_OTP,	reg_RDFETCH1,		o_Reserved_DDR_INIT_9 },

	/* DRAM DCTL Resync */
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_CONCONTROL,		o_reg_CONCONTROL_0 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_1 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t Set_DREX_address_parameters[] = {
	/* DRAM Density Check */
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_25 },
	{ FLG_POLL,	reg_MRSTATUS,		p_reg_MRSTATUS },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_1 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_26 },
	{ FLG_POLL,	reg_MRSTATUS,		p_reg_MRSTATUS },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_29 },
	{ FLG_POLL,	reg_MRSTATUS,		p_reg_MRSTATUS },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_1 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_28 },
	{ FLG_POLL,	reg_MRSTATUS,		p_reg_MRSTATUS },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_1 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_27 },
	{ FLG_POLL,	reg_MRSTATUS,		p_reg_MRSTATUS },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_1 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_27 },
	{ FLG_OTP,	reg_DPHY_GNR_CON0,	o_reg_DPHY_GNR_CON0_2 },
	{ FLG_OTP,	reg_DPHY2_GNR_CON0,	o_reg_DPHY_GNR_CON0_2 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_1 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },

	/* DRAM Density Setting */
	{ FLG_OTP,	reg_MEMCONTROL,		o_Reserved_DDR_INIT_10 },
	{ 0x0,		reg_ASP_MEMBASECONFIG0,	0x20009 },
	{ FLG_OTP,	reg_ASP_MEMCONFIG0,	o_Reserved_DDR_INIT_11 },
	{ 0x0,		reg_ASP_CHIP0SIZECONFIG,0x1 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t prepare_training[] = {
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_1 },
	{ FLG_OTP,	reg_DPHY_DVFS_CON,	o_Reserved_DDR_INIT_5 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_1 },
	{ FLG_OTP,	reg_DPHY2_DVFS_CON,	o_Reserved_DDR_INIT_5 },
	{ FLG_OTP,	reg_DPHY_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_3 },
	{ FLG_OTP,	reg_DPHY2_MDLL_CON0,	o_reg_DPHY_MDLL_CON0_3 },
	{ 0, 		0,			0 },
};

__CONST ddr_reg_control_t CA_training[] = {

	/* S/W Command Bus Training */
	{ FLG_OTP,	reg_DPHY_CBT_CON0,	o_Reserved_DDR_INIT_20 },
	{ FLG_OTP,	reg_DPHY2_CBT_CON0,	o_Reserved_DDR_INIT_20 },
	{ FLG_OTP,	reg_DPHY_LP_CON0,	o_reg_DPHY_LP_CON0_2 },
	{ FLG_OTP,	reg_DPHY2_LP_CON0,	o_reg_DPHY_LP_CON0_2 },
	{ FLG_OTP,	reg_DPHY_CAL_CON0,	o_reg_DPHY_CAL_CON0_1 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON0,	o_reg_DPHY_CAL_CON0_1 },
	{ FLG_OTP,	reg_DPHY_CAL_CON0,	o_reg_DPHY_CAL_CON0_3 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON0,	o_reg_DPHY_CAL_CON0_3 },

	/* DRAMDCTLResync:  start */
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_1 },
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },

	{ FLG_OTP,	reg_DPHY_CAL_CON0,	o_reg_DPHY_CAL_CON0_2 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON0,	o_reg_DPHY_CAL_CON0_2 },
	{ FLG_OTP,	reg_DPHY_CBT_CON0,	o_Reserved_DDR_INIT_20 },
	{ FLG_OTP,	reg_DPHY2_CBT_CON0,	o_Reserved_DDR_INIT_20 },
	{ FLG_OTP,	reg_DPHY_CBT_CON0,	o_Reserved_DDR_INIT_20 },
	{ FLG_OTP,	reg_DPHY2_CBT_CON0,	o_Reserved_DDR_INIT_20 },
	{ FLG_OTP,	reg_DPHY_LP_CON0,	o_reg_DPHY_LP_CON0_1 },
	{ FLG_OTP,	reg_DPHY2_LP_CON0,	o_reg_DPHY_LP_CON0_1 },
	{ 0, 		0,			0 },
};


__CONST ddr_reg_control_t ODT_training[] = {
	/* On_Die_Termination */
	{ FLG_OTP,	reg_DPHY_LP_CON0,	o_reg_DPHY_LP_CON0_0 },
	{ FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_4 },
	{ FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_4 },
	{ FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_4 },
	{ FLG_OTP,	reg_DPHY_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_OTP,	reg_DPHY_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_OTP,	reg_DPHY_CAL_CON2,	o_reg_DPHY_CAL_CON2_1 },
	{ FLG_OTP,	reg_DPHY_CAL_CON2,	o_reg_DPHY_CAL_CON2_0 },
	{ FLG_OTP,	reg_DPHY_CAL_CON2,	o_reg_DPHY_CAL_CON2_1 },
	{ FLG_OTP,	reg_DPHY_CAL_CON2,	o_reg_DPHY_CAL_CON2_1 },
	{ FLG_OTP,	reg_DPHY_CAL_CON2,	o_reg_DPHY_CAL_CON2_3 },
	{ FLG_OTP,	reg_DPHY_GNR_CON0,	o_reg_DPHY_GNR_CON0_1 },
	{ FLG_OTP,	reg_DPHY2_LP_CON0,	o_reg_DPHY_LP_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_4 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_4 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_4 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON6,	o_reg_DPHY_ZQ_CON6_2 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON2,	o_reg_DPHY_CAL_CON2_1 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON2,	o_reg_DPHY_CAL_CON2_0 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON2,	o_reg_DPHY_CAL_CON2_1 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON2,	o_reg_DPHY_CAL_CON2_1 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON2,	o_reg_DPHY_CAL_CON2_3 },
	{ FLG_OTP,	reg_DPHY2_GNR_CON0,	o_reg_DPHY_GNR_CON0_1 },

	/* ZQ_Calibration */
	{ FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_4 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_4 },
	{ FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_6 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_6 },
	{ FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_7 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_7 },

	/* wait zq calibration */
	{ FLG_POLL,	reg_DPHY_ZQ_CON1,	p_reg_DPHY_ZQ_CON1 },
	{ FLG_POLL,	reg_DPHY2_ZQ_CON1,	p_reg_DPHY2_ZQ_CON1 },

	{ FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_5 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_5 },
	{ FLG_OTP,	reg_DPHY_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_4 },
	{ FLG_OTP,	reg_DPHY2_ZQ_CON0,	o_reg_DPHY_ZQ_CON0_4 },
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0 },
	{ FLG_OTP,	reg_CONCONTROL,		o_reg_CONCONTROL_0 },
	{ 0, 		0,			0 },
};

__CONST ddr_reg_control_t AutoDQS_clean_Gate_training[] = {
	{ FLG_OTP,	reg_DPHY_CAL_CON4,	o_SECURE_JTAG3 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON4,	o_SECURE_JTAG3 },
	{ FLG_OTP,	reg_DPHY_GNR_CON0,	o_reg_DPHY_GNR_CON0_2 },
	{ FLG_OTP,	reg_DPHY_CAL_CON2,	o_reg_DPHY_CAL_CON2_2 },
	{ FLG_OTP,	reg_DPHY_CAL_CON2,	o_reg_DPHY_CAL_CON2_4 },
	{ FLG_OTP,	reg_DPHY_CAL_CON2,	o_reg_DPHY_CAL_CON2_4 },
	{ FLG_OTP,	reg_DPHY_TESTIRCV_CON0,	o_PCIe_reg_77 },
	{ FLG_OTP,	reg_DPHY2_GNR_CON0,	o_reg_DPHY_GNR_CON0_2 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON2,	o_reg_DPHY_CAL_CON2_2 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON2,	o_reg_DPHY_CAL_CON2_4 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON2,	o_reg_DPHY_CAL_CON2_4 },
	{ FLG_OTP,	reg_DPHY2_TESTIRCV_CON0,o_PCIe_reg_77 },
	{ FLG_OTP,	reg_DPHY_CAL_CON0,	o_reg_DPHY_CAL_CON0_2 },
	{ FLG_OTP,	reg_DPHY_CAL_CON3,	o_Reserved_DDR_INIT_18 },
	{ FLG_OTP,	reg_DPHY_CAL_CON2,	o_reg_DPHY_CAL_CON2_4 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON0,	o_reg_DPHY_CAL_CON0_2 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON3,	o_Reserved_DDR_INIT_18 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON2,	o_reg_DPHY_CAL_CON2_4 },
	{ 0, 		0,			0 },
};

__CONST ddr_reg_control_t Read_DQ_Calibration[] = {
	{ FLG_OTP,	reg_DPHY_CAL_RD_PATTERN_CON0,	o_Reserved_DDR_INIT_19 },
	{ FLG_OTP,	reg_DIRECTCMD,			o_reg_DIRECTCMD_16 },
	{ FLG_OTP,	reg_DIRECTCMD,			o_reg_DIRECTCMD_19 },
	{ FLG_OTP,	reg_DIRECTCMD,			o_reg_DIRECTCMD_9 },
	{ FLG_OTP,	reg_DIRECTCMD,			o_reg_DIRECTCMD_6 },
	{ FLG_OTP,	reg_DPHY_CAL_CON0,		o_reg_DPHY_CAL_CON0_4 },
	{ FLG_OTP,	reg_DPHY2_CAL_RD_PATTERN_CON0,	o_Reserved_DDR_INIT_19 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON0,		o_reg_DPHY_CAL_CON0_4 },
	{ FLG_OTP,	reg_INIT_TRAIN_CONFIG,		o_reg_INIT_TRAIN_CONFIG_0 },
	{ FLG_OTP,	reg_INIT_TRAIN_CONFIG,		o_reg_INIT_TRAIN_CONFIG_2 },
	{ FLG_OTP,	reg_INIT_TRAIN_CONTROL,		o_reg_INIT_TRAIN_CONTROL_1 },

	/* poll for train complete	*/
	{ FLG_POLL,	reg_PHYSTATUS,			p_reg_PHYSTATUS_train },

	{ FLG_OTP,	reg_INIT_TRAIN_CONTROL,		o_reg_INIT_TRAIN_CONTROL_0 },
	{ 0, 		0,				0 },
};

__CONST ddr_reg_control_t Write_DQ_Calibration[] = {
	{ FLG_OTP,	reg_DPHY_CAL_WR_PATTERN_CON0,	o_PCIe_reg_75 },
	{ FLG_OTP,	reg_DPHY_CAL_WR_PATTERN_CON1,	o_PCIe_reg_address_75 },
	{ FLG_OTP,	reg_DPHY_CAL_WR_PATTERN_CON2,	o_PCIe_reg_76 },
	{ FLG_OTP,	reg_DPHY_CAL_WR_PATTERN_CON3,	o_reg_DPHY_CAL_WR_PATTERN_CON4_0 },
	{ FLG_OTP,	reg_DPHY_CAL_WR_PATTERN_CON4,	o_reg_DPHY_CAL_WR_PATTERN_CON4_1 },
	{ 0x0,		reg_WRTRA_PATTERN0,		0xaa55aa55 },
	{ 0x0,		reg_WRTRA_PATTERN1,		0xaa55aa55 },
	{ 0x0,		reg_WRTRA_PATTERN2,		0x5555 },
	{ FLG_OTP,	reg_DPHY_CAL_CON0,		o_reg_DPHY_CAL_CON0_5 },
	{ FLG_OTP,	reg_DPHY2_CAL_WR_PATTERN_CON0,	o_PCIe_reg_75 },
	{ FLG_OTP,	reg_DPHY2_CAL_WR_PATTERN_CON1,	o_PCIe_reg_address_75 },
	{ FLG_OTP,	reg_DPHY2_CAL_WR_PATTERN_CON2,	o_PCIe_reg_76 },
	{ FLG_OTP,	reg_DPHY2_CAL_WR_PATTERN_CON3,	o_reg_DPHY_CAL_WR_PATTERN_CON4_0 },
	{ FLG_OTP,	reg_DPHY2_CAL_WR_PATTERN_CON4,	o_reg_DPHY_CAL_WR_PATTERN_CON4_1 },
	{ 0x0,		reg_WRTRA_PATTERN0,		0xaa55aa55 },
	{ 0x0,		reg_WRTRA_PATTERN1,		0xaa55aa55 },
	{ 0x0,		reg_WRTRA_PATTERN2,		0x5555 },
	{ FLG_OTP,	reg_DPHY2_CAL_CON0,		o_reg_DPHY_CAL_CON0_5 },
	{ FLG_OTP,	reg_INIT_TRAIN_CONFIG,		o_reg_INIT_TRAIN_CONFIG_0 },
	{ FLG_OTP,	reg_INIT_TRAIN_CONFIG,		o_reg_INIT_TRAIN_CONFIG_3 },
	{ FLG_OTP,	reg_INIT_TRAIN_CONTROL,		o_reg_INIT_TRAIN_CONTROL_1 },

	/* poll for train complete	*/
	{ FLG_POLL,	reg_PHYSTATUS,			p_reg_PHYSTATUS_train },

	{ FLG_OTP,	reg_INIT_TRAIN_CONTROL,		o_reg_INIT_TRAIN_CONTROL_0 },
	{ 0, 		0,				0 },
};

__CONST ddr_reg_control_t PRBS_training_init[] = {

	/* DDRPHY_SET_PRBS_TRAINING_INIT */
	{ 0x0,		reg_DPHY_PRBS_CON0,	0x50000	},
	{ 0x0,		reg_DPHY2_PRBS_CON0,	0x50000	},
	{ FLG_OTP,	reg_DPHY_PRBS_CON1,	o_PCIe_reg_address_79 },
	{ FLG_OTP,	reg_DPHY2_PRBS_CON1,	o_PCIe_reg_address_79 },
	{ 0, 		0,			0 },
};

__CONST ddr_reg_control_t PRBS_training_read[] = {

	/* DDRPHY_RUN_PRBS_TRAINING - READ */
	{ 0x0,	reg_DPHY_PRBS_CON0,		0x50002	},
	{ 0x0,	reg_DPHY2_PRBS_CON0,		0x50002	},
	{ FLG_POLL,	reg_DPHY_PRBS_CON0,	p_reg_DPHY_PRBS_CON0_prbs_done },
	{ FLG_POLL,	reg_DPHY2_PRBS_CON0,	p_reg_DPHY_PRBS_CON0_prbs_done },
	{ 0x0,	reg_DPHY_PRBS_CON0,		0x50001	},
	{ 0x0,	reg_DPHY2_PRBS_CON0,		0x50001	},
	{ FLG_POLL,	reg_DPHY_PRBS_CON0,	p_reg_DPHY_PRBS_CON0_prbs_disable },
	{ FLG_POLL,	reg_DPHY2_PRBS_CON0,	p_reg_DPHY_PRBS_CON0_prbs_disable },

	/* DRAMDCTLResync:  start */
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_1},
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0},
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0},
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0},
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0},
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0},
	{ 0, 		0,			0 },
};

__CONST ddr_reg_control_t PRBS_training_write[] = {

	/* DDRPHY_RUN_PRBS_TRAINING - WRITE */
	{ 0x0,		reg_DPHY_PRBS_CON0,	0x50004	},
	{ 0x0,		reg_DPHY2_PRBS_CON0,	0x50004	},
	{ FLG_POLL,	reg_DPHY_PRBS_CON0,	p_reg_DPHY_PRBS_CON0_prbs_done },
	{ FLG_POLL,	reg_DPHY2_PRBS_CON0,	p_reg_DPHY_PRBS_CON0_prbs_done },
	{ 0x0,		reg_DPHY_PRBS_CON0,	0x50001	},
	{ 0x0,		reg_DPHY2_PRBS_CON0,	0x50001	},
	{ FLG_POLL,	reg_DPHY_PRBS_CON0,	p_reg_DPHY_PRBS_CON0_prbs_disable },
	{ FLG_POLL,	reg_DPHY2_PRBS_CON0,	p_reg_DPHY_PRBS_CON0_prbs_disable },

	/* DRAMDCTLResync:  start */
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_1},
	{ FLG_OTP,	reg_PHYCONTROL0,	o_reg_PHYCONTROL0_0},
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0},
	{ FLG_OTP,	reg_DPHY_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0},
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0},
	{ FLG_OTP,	reg_DPHY2_OFFSETD_CON0,	o_reg_DPHY_OFFSETD_CON0_0},
	{ 0, 		0,			0 },
};

__CONST ddr_reg_control_t AXI_Enable_After_All_training[] = {
	{ FLG_OTP,	reg_MEMCONTROL,		o_Reserved_DDR_INIT_13 },
	{ FLG_OTP,	reg_MEMCONTROL,		o_Reserved_DDR_INIT_14 },
	{ FLG_OTP,	reg_DFIRSTCONTROL,	o_Reserved_DDR_INIT_15 },
	{ FLG_OTP,	reg_CONCONTROL,		o_reg_CONCONTROL_0 },
	{ FLG_SET,	reg_CONCONTROL,		0x1 << 5 },
	{ 0x0,		reg_ALL_INIT_INDI,	0x1 },
	{ FLG_OTP,	reg_ACTIVATE_AXI_READY,	o_Reserved_DDR_INIT_16 },
	{ 0, 		0,			0 },
};

__CONST ddr_reg_control_t Enter_Self_Refresh_mode[] = {
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_15 },
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_20 },

	/* poll for self refresh entry */
	{ FLG_POLL,	reg_CHIPSTATUS,		p_reg_CHIPSTATUS_sr_enter },
	{ 0, 		0,			0 },
};

__CONST ddr_reg_control_t Exit_Self_Refresh_mode[] = {
	{ FLG_OTP,	reg_DIRECTCMD,		o_reg_DIRECTCMD_22 },

	/* poll for self refresh exit */
	{ FLG_POLL,	reg_CHIPSTATUS,		p_reg_CHIPSTATUS_sr_exit },
	{ 0, 		0,			0 },
};

__CONST ddr_reg_control_t *ddr_blk_reg_control[] = {
	[b_config_PLL_MIF_via_CMU_MIF_for_LowFreq] = config_PLL_MIF_via_CMU_MIF_for_LowFreq,
	[b_config_PLL_MIF_via_CMU_MIF_for_HighFreq] = config_PLL_MIF_via_CMU_MIF_for_HighFreq,
	[b_ungate_PHY_clock] = ungate_PHY_clock,
	[b_deassert_PHY_reset_while_PHY_is_gated] = deassert_PHY_reset_while_PHY_is_gated,
	[b_initialize_PHY_pre] = initialize_PHY_pre,
	[b_initialize_PHY] = initialize_PHY,
	[b_initialize_DFI] = initialize_DFI,
	[b_DRAM_reset_sequence] = DRAM_reset_sequence,
	[b_Power_down_exit_sequence] = Power_down_exit_sequence,
	[b_MRWs_Set_VREF_ODT_etc] = MRWs_Set_VREF_ODT_etc,
	[b_ZQ_Calibration] = ZQ_Calibration,
	[b_IO_Initialization] = IO_Initialization,
	[b_Enable_DLL] = Enable_DLL,
	[b_Set_DREX_timing_parameters] = Set_DREX_timing_parameters,
	[b_Set_DREX_address_parameters] = Set_DREX_address_parameters,
	[b_prepare_training] = prepare_training,
	[b_CA_training] = CA_training,
	[b_ODT_training] = ODT_training,
	[b_AutoDQS_clean_Gate_training] = AutoDQS_clean_Gate_training,
	[b_Read_DQ_Calibration] = Read_DQ_Calibration,
	[b_Write_DQ_Calibration] = Write_DQ_Calibration,
	[b_PRBS_training_init] = PRBS_training_init,
	[b_PRBS_training_read] = PRBS_training_read,
	[b_PRBS_training_write] = PRBS_training_write,
	[b_AXI_Enable_After_All_training] = AXI_Enable_After_All_training,
	[b_Enter_Self_Refresh_mode] = Enter_Self_Refresh_mode,
	[b_Exit_Self_Refresh_mode] = Exit_Self_Refresh_mode,
};

enum ddr_train_save_index {
	s_reg_DPHY_MDLL_CON1,
	s_reg_DPHY2_MDLL_CON1,
	s_reg_DPHY_CA_DESKEW_CON0,
	s_reg_DPHY_CA_DESKEW_CON1,
	s_reg_DPHY_CA_DESKEW_CON2,
	s_reg_DPHY2_CA_DESKEW_CON0,
	s_reg_DPHY2_CA_DESKEW_CON1,
	s_reg_DPHY2_CA_DESKEW_CON2,
	s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON_DM,
	s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON0,
	s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON1,
	s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON2,
	s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON3,
	s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON4,
	s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON5,
	s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON6,
	s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON7,
	s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON_DM,
	s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON0,
	s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON1,
	s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON2,
	s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON3,
	s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON4,
	s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON5,
	s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON6,
	s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON7,
	s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON_DM,
	s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON0,
	s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON1,
	s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON2,
	s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON3,
	s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON4,
	s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON5,
	s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON6,
	s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON7,
	s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON_DM,
	s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON0,
	s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON1,
	s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON2,
	s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON3,
	s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON4,
	s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON5,
	s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON6,
	s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON7,
	s_reg_DPHY_RD_DQS_VWMC_CS0_CON0,
	s_reg_DPHY_RD_DQS_VWMC_CS1_CON0,
	s_reg_DPHY_RD_DQS_VWML_CS0_CON0,
	s_reg_DPHY_RD_DQS_VWML_CS1_CON0,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON_DM,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON0,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON1,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON2,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON3,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON4,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON5,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON6,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON7,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON_DM,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON0,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON1,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON2,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON3,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON4,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON5,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON6,
	s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON7,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON_DM,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON0,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON1,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON2,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON3,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON4,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON5,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON6,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON7,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON_DM,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON0,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON1,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON2,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON3,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON4,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON5,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON6,
	s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON7,
	s_reg_DPHY2_RD_DQS_VWMC_CS0_CON0,
	s_reg_DPHY2_RD_DQS_VWMC_CS1_CON0,
	s_reg_DPHY2_RD_DQS_VWML_CS0_CON0,
	s_reg_DPHY2_RD_DQS_VWML_CS1_CON0,
	s_reg_DPHY_WR_DESKEWC_CS0_CON0,
	s_reg_DPHY_WR_DESKEWC_CS0_CON1,
	s_reg_DPHY_WR_DESKEWC_CS0_CON2,
	s_reg_DPHY_WR_DESKEWC_CS0_CON3,
	s_reg_DPHY_WR_DESKEWC_CS0_CON4,
	s_reg_DPHY_WR_DESKEWC_CS0_CON5,
	s_reg_DPHY_WR_DESKEWC_CS0_CON6,
	s_reg_DPHY_WR_DESKEWC_CS0_CON7,
	s_reg_DPHY_DM_DESKEWC_CS0_CON0,
	s_reg_DPHY_WR_DESKEWC_CS1_CON0,
	s_reg_DPHY_WR_DESKEWC_CS1_CON1,
	s_reg_DPHY_WR_DESKEWC_CS1_CON2,
	s_reg_DPHY_WR_DESKEWC_CS1_CON3,
	s_reg_DPHY_WR_DESKEWC_CS1_CON4,
	s_reg_DPHY_WR_DESKEWC_CS1_CON5,
	s_reg_DPHY_WR_DESKEWC_CS1_CON6,
	s_reg_DPHY_WR_DESKEWC_CS1_CON7,
	s_reg_DPHY_DM_DESKEWC_CS1_CON0,
	s_reg_DPHY_WR_DESKEWL_CS0_CON0,
	s_reg_DPHY_WR_DESKEWL_CS0_CON1,
	s_reg_DPHY_WR_DESKEWL_CS0_CON2,
	s_reg_DPHY_WR_DESKEWL_CS0_CON3,
	s_reg_DPHY_WR_DESKEWL_CS0_CON4,
	s_reg_DPHY_WR_DESKEWL_CS0_CON5,
	s_reg_DPHY_WR_DESKEWL_CS0_CON6,
	s_reg_DPHY_WR_DESKEWL_CS0_CON7,
	s_reg_DPHY_DM_DESKEWL_CS0_CON0,
	s_reg_DPHY_WR_DESKEWL_CS1_CON0,
	s_reg_DPHY_WR_DESKEWL_CS1_CON1,
	s_reg_DPHY_WR_DESKEWL_CS1_CON2,
	s_reg_DPHY_WR_DESKEWL_CS1_CON3,
	s_reg_DPHY_WR_DESKEWL_CS1_CON4,
	s_reg_DPHY_WR_DESKEWL_CS1_CON5,
	s_reg_DPHY_WR_DESKEWL_CS1_CON6,
	s_reg_DPHY_WR_DESKEWL_CS1_CON7,
	s_reg_DPHY_DM_DESKEWL_CS1_CON0,
	s_reg_DPHY2_WR_DESKEWC_CS0_CON0,
	s_reg_DPHY2_WR_DESKEWC_CS0_CON1,
	s_reg_DPHY2_WR_DESKEWC_CS0_CON2,
	s_reg_DPHY2_WR_DESKEWC_CS0_CON3,
	s_reg_DPHY2_WR_DESKEWC_CS0_CON4,
	s_reg_DPHY2_WR_DESKEWC_CS0_CON5,
	s_reg_DPHY2_WR_DESKEWC_CS0_CON6,
	s_reg_DPHY2_WR_DESKEWC_CS0_CON7,
	s_reg_DPHY2_DM_DESKEWC_CS0_CON0,
	s_reg_DPHY2_WR_DESKEWC_CS1_CON0,
	s_reg_DPHY2_WR_DESKEWC_CS1_CON1,
	s_reg_DPHY2_WR_DESKEWC_CS1_CON2,
	s_reg_DPHY2_WR_DESKEWC_CS1_CON3,
	s_reg_DPHY2_WR_DESKEWC_CS1_CON4,
	s_reg_DPHY2_WR_DESKEWC_CS1_CON5,
	s_reg_DPHY2_WR_DESKEWC_CS1_CON6,
	s_reg_DPHY2_WR_DESKEWC_CS1_CON7,
	s_reg_DPHY2_DM_DESKEWC_CS1_CON0,
	s_reg_DPHY2_WR_DESKEWL_CS0_CON0,
	s_reg_DPHY2_WR_DESKEWL_CS0_CON1,
	s_reg_DPHY2_WR_DESKEWL_CS0_CON2,
	s_reg_DPHY2_WR_DESKEWL_CS0_CON3,
	s_reg_DPHY2_WR_DESKEWL_CS0_CON4,
	s_reg_DPHY2_WR_DESKEWL_CS0_CON5,
	s_reg_DPHY2_WR_DESKEWL_CS0_CON6,
	s_reg_DPHY2_WR_DESKEWL_CS0_CON7,
	s_reg_DPHY2_DM_DESKEWL_CS0_CON0,
	s_reg_DPHY2_WR_DESKEWL_CS1_CON0,
	s_reg_DPHY2_WR_DESKEWL_CS1_CON1,
	s_reg_DPHY2_WR_DESKEWL_CS1_CON2,
	s_reg_DPHY2_WR_DESKEWL_CS1_CON3,
	s_reg_DPHY2_WR_DESKEWL_CS1_CON4,
	s_reg_DPHY2_WR_DESKEWL_CS1_CON5,
	s_reg_DPHY2_WR_DESKEWL_CS1_CON6,
	s_reg_DPHY2_WR_DESKEWL_CS1_CON7,
	s_reg_DPHY2_DM_DESKEWL_CS1_CON0,
	s_reg_DPHY_PRBS_CON2,
	s_reg_DPHY_PRBS_CON3,
	s_reg_DPHY2_PRBS_CON2,
	s_reg_DPHY2_PRBS_CON3,
	s_reg_DPHY_ZQ_CON9,
	s_reg_DPHY2_ZQ_CON9,
	s_train_max_index,
};

__CONST uint32_t g_ddr_train_save_address[] = {
	[s_reg_DPHY_MDLL_CON1] = 0x105c00b4,
	[s_reg_DPHY2_MDLL_CON1] = 0x105c00b4,
	[s_reg_DPHY_CA_DESKEW_CON0] = 0x105b007c,
	[s_reg_DPHY_CA_DESKEW_CON1] = 0x105b0080,
	[s_reg_DPHY_CA_DESKEW_CON2] = 0x105b0084,
	[s_reg_DPHY2_CA_DESKEW_CON0] = 0x105c007c,
	[s_reg_DPHY2_CA_DESKEW_CON1] = 0x105c0080,
	[s_reg_DPHY2_CA_DESKEW_CON2] = 0x105c0084,

	[s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON_DM] = 0x105b018c,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON0] = 0x105b0190,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON1] = 0x105b019c,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON2] = 0x105b01a8,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON3] = 0x105b01b4,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON4] = 0x105b01c0,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON5] = 0x105b01cc,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON6] = 0x105b01d8,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON7] = 0x105b01e4,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON_DM] = 0x105b076c,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON0] = 0x105b0770,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON1] = 0x105b0774,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON2] = 0x105b0778,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON3] = 0x105b077c,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON4] = 0x105b0780,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON5] = 0x105b0784,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON6] = 0x105b0788,
	[s_reg_DPHY_RD_DESKEW_CENTER_CS1_CON7] = 0x105b078c,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON_DM] = 0x105b0610,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON0] = 0x105b0614,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON1] = 0x105b0620,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON2] = 0x105b062c,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON3] = 0x105b0638,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON4] = 0x105b0644,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON5] = 0x105b0650,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON6] = 0x105b065c,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS0_CON7] = 0x105b0668,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON_DM] = 0x105b0790,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON0] = 0x105b0794,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON1] = 0x105b0798,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON2] = 0x105b079c,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON3] = 0x105b07a0,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON4] = 0x105b07a4,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON5] = 0x105b07a8,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON6] = 0x105b07ac,
	[s_reg_DPHY_RD_DESKEW_LEFT_CS1_CON7] = 0x105b07b0,
	[s_reg_DPHY_RD_DQS_VWMC_CS0_CON0] = 0x105b0580,
	[s_reg_DPHY_RD_DQS_VWMC_CS1_CON0] = 0x105b0768,
	[s_reg_DPHY_RD_DQS_VWML_CS0_CON0] = 0x105b0574,
	[s_reg_DPHY_RD_DQS_VWML_CS1_CON0] = 0x105b0764,

	[s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON_DM] = 0x105c018c,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON0] = 0x105c0190,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON1] = 0x105c019c,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON2] = 0x105c01a8,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON3] = 0x105c01b4,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON4] = 0x105c01c0,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON5] = 0x105c01cc,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON6] = 0x105c01d8,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS0_CON7] = 0x105c01e4,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON_DM] = 0x105c076c,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON0] = 0x105c0770,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON1] = 0x105c0774,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON2] = 0x105c0778,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON3] = 0x105c077c,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON4] = 0x105c0780,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON5] = 0x105c0784,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON6] = 0x105c0788,
	[s_reg_DPHY2_RD_DESKEW_CENTER_CS1_CON7] = 0x105c078c,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON_DM] = 0x105c0610,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON0] = 0x105c0614,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON1] = 0x105c0620,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON2] = 0x105c062c,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON3] = 0x105c0638,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON4] = 0x105c0644,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON5] = 0x105c0650,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON6] = 0x105c065c,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS0_CON7] = 0x105c0668,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON_DM] = 0x105c0790,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON0] = 0x105c0794,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON1] = 0x105c0798,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON2] = 0x105c079c,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON3] = 0x105c07a0,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON4] = 0x105c07a4,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON5] = 0x105c07a8,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON6] = 0x105c07ac,
	[s_reg_DPHY2_RD_DESKEW_LEFT_CS1_CON7] = 0x105c07b0,
	[s_reg_DPHY2_RD_DQS_VWMC_CS0_CON0] = 0x105c0580,
	[s_reg_DPHY2_RD_DQS_VWMC_CS1_CON0] = 0x105c0768,
	[s_reg_DPHY2_RD_DQS_VWML_CS0_CON0] = 0x105c0574,
	[s_reg_DPHY2_RD_DQS_VWML_CS1_CON0] = 0x105c0764,

	[s_reg_DPHY_WR_DESKEWC_CS0_CON0] = 0x105b01f0,
	[s_reg_DPHY_WR_DESKEWC_CS0_CON1] = 0x105b01fc,
	[s_reg_DPHY_WR_DESKEWC_CS0_CON2] = 0x105b0208,
	[s_reg_DPHY_WR_DESKEWC_CS0_CON3] = 0x105b0214,
	[s_reg_DPHY_WR_DESKEWC_CS0_CON4] = 0x105b0220,
	[s_reg_DPHY_WR_DESKEWC_CS0_CON5] = 0x105b022c,
	[s_reg_DPHY_WR_DESKEWC_CS0_CON6] = 0x105b0238,
	[s_reg_DPHY_WR_DESKEWC_CS0_CON7] = 0x105b0244,
	[s_reg_DPHY_DM_DESKEWC_CS0_CON0] = 0x105b0250,
	[s_reg_DPHY_WR_DESKEWC_CS1_CON0] = 0x105b0410,
	[s_reg_DPHY_WR_DESKEWC_CS1_CON1] = 0x105b041c,
	[s_reg_DPHY_WR_DESKEWC_CS1_CON2] = 0x105b0428,
	[s_reg_DPHY_WR_DESKEWC_CS1_CON3] = 0x105b0434,
	[s_reg_DPHY_WR_DESKEWC_CS1_CON4] = 0x105b0440,
	[s_reg_DPHY_WR_DESKEWC_CS1_CON5] = 0x105b044c,
	[s_reg_DPHY_WR_DESKEWC_CS1_CON6] = 0x105b0458,
	[s_reg_DPHY_WR_DESKEWC_CS1_CON7] = 0x105b0464,
	[s_reg_DPHY_DM_DESKEWC_CS1_CON0] = 0x105b0470,
	[s_reg_DPHY_WR_DESKEWL_CS0_CON0] = 0x105b0490,
	[s_reg_DPHY_WR_DESKEWL_CS0_CON1] = 0x105b049c,
	[s_reg_DPHY_WR_DESKEWL_CS0_CON2] = 0x105b04a8,
	[s_reg_DPHY_WR_DESKEWL_CS0_CON3] = 0x105b04b4,
	[s_reg_DPHY_WR_DESKEWL_CS0_CON4] = 0x105b04c0,
	[s_reg_DPHY_WR_DESKEWL_CS0_CON5] = 0x105b04cc,
	[s_reg_DPHY_WR_DESKEWL_CS0_CON6] = 0x105b04d8,
	[s_reg_DPHY_WR_DESKEWL_CS0_CON7] = 0x105b04e4,
	[s_reg_DPHY_DM_DESKEWL_CS0_CON0] = 0x105b04f0,
	[s_reg_DPHY_WR_DESKEWL_CS1_CON0] = 0x105b0500,
	[s_reg_DPHY_WR_DESKEWL_CS1_CON1] = 0x105b050c,
	[s_reg_DPHY_WR_DESKEWL_CS1_CON2] = 0x105b0518,
	[s_reg_DPHY_WR_DESKEWL_CS1_CON3] = 0x105b0524,
	[s_reg_DPHY_WR_DESKEWL_CS1_CON4] = 0x105b0530,
	[s_reg_DPHY_WR_DESKEWL_CS1_CON5] = 0x105b053c,
	[s_reg_DPHY_WR_DESKEWL_CS1_CON6] = 0x105b0548,
	[s_reg_DPHY_WR_DESKEWL_CS1_CON7] = 0x105b0554,
	[s_reg_DPHY_DM_DESKEWL_CS1_CON0] = 0x105b0560,

	[s_reg_DPHY2_WR_DESKEWC_CS0_CON0] = 0x105c01f0,
	[s_reg_DPHY2_WR_DESKEWC_CS0_CON1] = 0x105c01fc,
	[s_reg_DPHY2_WR_DESKEWC_CS0_CON2] = 0x105c0208,
	[s_reg_DPHY2_WR_DESKEWC_CS0_CON3] = 0x105c0214,
	[s_reg_DPHY2_WR_DESKEWC_CS0_CON4] = 0x105c0220,
	[s_reg_DPHY2_WR_DESKEWC_CS0_CON5] = 0x105c022c,
	[s_reg_DPHY2_WR_DESKEWC_CS0_CON6] = 0x105c0238,
	[s_reg_DPHY2_WR_DESKEWC_CS0_CON7] = 0x105c0244,
	[s_reg_DPHY2_DM_DESKEWC_CS0_CON0] = 0x105c0250,
	[s_reg_DPHY2_WR_DESKEWC_CS1_CON0] = 0x105c0410,
	[s_reg_DPHY2_WR_DESKEWC_CS1_CON1] = 0x105c041c,
	[s_reg_DPHY2_WR_DESKEWC_CS1_CON2] = 0x105c0428,
	[s_reg_DPHY2_WR_DESKEWC_CS1_CON3] = 0x105c0434,
	[s_reg_DPHY2_WR_DESKEWC_CS1_CON4] = 0x105c0440,
	[s_reg_DPHY2_WR_DESKEWC_CS1_CON5] = 0x105c044c,
	[s_reg_DPHY2_WR_DESKEWC_CS1_CON6] = 0x105c0458,
	[s_reg_DPHY2_WR_DESKEWC_CS1_CON7] = 0x105c0464,
	[s_reg_DPHY2_DM_DESKEWC_CS1_CON0] = 0x105c0470,
	[s_reg_DPHY2_WR_DESKEWL_CS0_CON0] = 0x105c0490,
	[s_reg_DPHY2_WR_DESKEWL_CS0_CON1] = 0x105c049c,
	[s_reg_DPHY2_WR_DESKEWL_CS0_CON2] = 0x105c04a8,
	[s_reg_DPHY2_WR_DESKEWL_CS0_CON3] = 0x105c04b4,
	[s_reg_DPHY2_WR_DESKEWL_CS0_CON4] = 0x105c04c0,
	[s_reg_DPHY2_WR_DESKEWL_CS0_CON5] = 0x105c04cc,
	[s_reg_DPHY2_WR_DESKEWL_CS0_CON6] = 0x105c04d8,
	[s_reg_DPHY2_WR_DESKEWL_CS0_CON7] = 0x105c04e4,
	[s_reg_DPHY2_DM_DESKEWL_CS0_CON0] = 0x105c04f0,
	[s_reg_DPHY2_WR_DESKEWL_CS1_CON0] = 0x105c0500,
	[s_reg_DPHY2_WR_DESKEWL_CS1_CON1] = 0x105c050c,
	[s_reg_DPHY2_WR_DESKEWL_CS1_CON2] = 0x105c0518,
	[s_reg_DPHY2_WR_DESKEWL_CS1_CON3] = 0x105c0524,
	[s_reg_DPHY2_WR_DESKEWL_CS1_CON4] = 0x105c0530,
	[s_reg_DPHY2_WR_DESKEWL_CS1_CON5] = 0x105c053c,
	[s_reg_DPHY2_WR_DESKEWL_CS1_CON6] = 0x105c0548,
	[s_reg_DPHY2_WR_DESKEWL_CS1_CON7] = 0x105c0554,
	[s_reg_DPHY2_DM_DESKEWL_CS1_CON0] = 0x105c0560,
	[s_reg_DPHY_PRBS_CON2] = 0x105b068c,
	[s_reg_DPHY_PRBS_CON3] = 0x105b0690,
	[s_reg_DPHY2_PRBS_CON2] = 0x105c068c,
	[s_reg_DPHY2_PRBS_CON3] = 0x105c0690,
	[s_reg_DPHY_ZQ_CON9] = 0x105b03ec,
	[s_reg_DPHY2_ZQ_CON9] = 0x105c03ec,
};

uint32_t g_ddr_train_save_value[s_train_max_index];

enum ddr_train_restore_read_index {
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON_DM,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON0,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON1,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON2,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON3,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON4,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON5,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON6,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON7,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON_DM,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON0,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON1,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON2,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON3,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON4,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON5,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON6,
	r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON7,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON_DM,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON0,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON1,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON2,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON3,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON4,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON5,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON6,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON7,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON_DM,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON0,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON1,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON2,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON3,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON4,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON5,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON6,
	r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON7,
	r_reg_DPHY_SW_RD_DQS_VWMC_CS0_CON0,
	r_reg_DPHY_SW_RD_DQS_VWMC_CS1_CON0,
	r_reg_DPHY_SW_RD_DQS_VWML_CS0_CON0,
	r_reg_DPHY_SW_RD_DQS_VWML_CS1_CON0,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON_DM,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON0,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON1,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON2,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON3,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON4,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON5,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON6,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON7,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON_DM,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON0,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON1,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON2,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON3,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON4,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON5,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON6,
	r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON7,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON_DM,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON0,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON1,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON2,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON3,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON4,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON5,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON6,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON7,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON_DM,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON0,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON1,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON2,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON3,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON4,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON5,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON6,
	r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON7,
	r_reg_DPHY2_SW_RD_DQS_VWMC_CS0_CON0,
	r_reg_DPHY2_SW_RD_DQS_VWMC_CS1_CON0,
	r_reg_DPHY2_SW_RD_DQS_VWML_CS0_CON0,
	r_reg_DPHY2_SW_RD_DQS_VWML_CS1_CON0,
};

__CONST uint32_t g_ddr_train_restore_read_address[] = {
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON_DM] = 0x105b07c8,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON0] = 0x105b07cc,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON1] = 0x105b07d0,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON2] = 0x105b07d4,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON3] = 0x105b07d8,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON4] = 0x105b07dc,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON5] = 0x105b07e0,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON6] = 0x105b07e4,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS0_CON7] = 0x105b07e8,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON_DM] = 0x105b0820,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON0] = 0x105b0824,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON1] = 0x105b0828,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON2] = 0x105b082c,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON3] = 0x105b0830,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON4] = 0x105b0834,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON5] = 0x105b0838,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON6] = 0x105b083c,
	[r_reg_DPHY_SW_RD_DESKEW_CENTER_CS1_CON7] = 0x105b0840,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON_DM] = 0x105b07f0,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON0] = 0x105b07f4,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON1] = 0x105b07f8,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON2] = 0x105b07fc,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON3] = 0x105b0800,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON4] = 0x105b0804,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON5] = 0x105b0808,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON6] = 0x105b080c,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS0_CON7] = 0x105b0810,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON_DM] = 0x105b0850,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON0] = 0x105b0854,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON1] = 0x105b0858,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON2] = 0x105b085c,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON3] = 0x105b0860,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON4] = 0x105b0864,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON5] = 0x105b0868,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON6] = 0x105b086c,
	[r_reg_DPHY_SW_RD_DESKEW_LEFT_CS1_CON7] = 0x105b0870,
	[r_reg_DPHY_SW_RD_DQS_VWMC_CS0_CON0] = 0x105b07c4,
	[r_reg_DPHY_SW_RD_DQS_VWMC_CS1_CON0] = 0x105b0818,
	[r_reg_DPHY_SW_RD_DQS_VWML_CS0_CON0] = 0x105b07c0,
	[r_reg_DPHY_SW_RD_DQS_VWML_CS1_CON0] = 0x105b0814,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON_DM] = 0x105c07c8,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON0] = 0x105c07cc,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON1] = 0x105c07d0,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON2] = 0x105c07d4,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON3] = 0x105c07d8,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON4] = 0x105c07dc,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON5] = 0x105c07e0,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON6] = 0x105c07e4,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON7] = 0x105c07e8,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON_DM] = 0x105c0820,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON0] = 0x105c0824,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON1] = 0x105c0828,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON2] = 0x105c082c,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON3] = 0x105c0830,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON4] = 0x105c0834,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON5] = 0x105c0838,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON6] = 0x105c083c,
	[r_reg_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON7] = 0x105c0840,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON_DM] = 0x105c07f0,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON0] = 0x105c07f4,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON1] = 0x105c07f8,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON2] = 0x105c07fc,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON3] = 0x105c0800,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON4] = 0x105c0804,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON5] = 0x105c0808,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON6] = 0x105c080c,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON7] = 0x105c0810,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON_DM] = 0x105c0850,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON0] = 0x105c0854,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON1] = 0x105c0858,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON2] = 0x105c085c,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON3] = 0x105c0860,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON4] = 0x105c0864,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON5] = 0x105c0868,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON6] = 0x105c086c,
	[r_reg_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON7] = 0x105c0870,
	[r_reg_DPHY2_SW_RD_DQS_VWMC_CS0_CON0] = 0x105c07c4,
	[r_reg_DPHY2_SW_RD_DQS_VWMC_CS1_CON0] = 0x105c0818,
	[r_reg_DPHY2_SW_RD_DQS_VWML_CS0_CON0] = 0x105c07c0,
	[r_reg_DPHY2_SW_RD_DQS_VWML_CS1_CON0] = 0x105c0814,
};

enum ddr_train_restore_write_index {
	r_reg_DPHY_SW_WR_DESKEWC_CS0_CON0,
	r_reg_DPHY_SW_WR_DESKEWC_CS0_CON1,
	r_reg_DPHY_SW_WR_DESKEWC_CS0_CON2,
	r_reg_DPHY_SW_WR_DESKEWC_CS0_CON3,
	r_reg_DPHY_SW_WR_DESKEWC_CS0_CON4,
	r_reg_DPHY_SW_WR_DESKEWC_CS0_CON5,
	r_reg_DPHY_SW_WR_DESKEWC_CS0_CON6,
	r_reg_DPHY_SW_WR_DESKEWC_CS0_CON7,
	r_reg_DPHY_SW_DM_DESKEWC_CS0_CON0,
	r_reg_DPHY_SW_WR_DESKEWC_CS1_CON0,
	r_reg_DPHY_SW_WR_DESKEWC_CS1_CON1,
	r_reg_DPHY_SW_WR_DESKEWC_CS1_CON2,
	r_reg_DPHY_SW_WR_DESKEWC_CS1_CON3,
	r_reg_DPHY_SW_WR_DESKEWC_CS1_CON4,
	r_reg_DPHY_SW_WR_DESKEWC_CS1_CON5,
	r_reg_DPHY_SW_WR_DESKEWC_CS1_CON6,
	r_reg_DPHY_SW_WR_DESKEWC_CS1_CON7,
	r_reg_DPHY_SW_DM_DESKEWC_CS1_CON0,
	r_reg_DPHY_SW_WR_DESKEWL_CS0_CON0,
	r_reg_DPHY_SW_WR_DESKEWL_CS0_CON1,
	r_reg_DPHY_SW_WR_DESKEWL_CS0_CON2,
	r_reg_DPHY_SW_WR_DESKEWL_CS0_CON3,
	r_reg_DPHY_SW_WR_DESKEWL_CS0_CON4,
	r_reg_DPHY_SW_WR_DESKEWL_CS0_CON5,
	r_reg_DPHY_SW_WR_DESKEWL_CS0_CON6,
	r_reg_DPHY_SW_WR_DESKEWL_CS0_CON7,
	r_reg_DPHY_SW_DM_DESKEWL_CS0_CON0,
	r_reg_DPHY_SW_WR_DESKEWL_CS1_CON0,
	r_reg_DPHY_SW_WR_DESKEWL_CS1_CON1,
	r_reg_DPHY_SW_WR_DESKEWL_CS1_CON2,
	r_reg_DPHY_SW_WR_DESKEWL_CS1_CON3,
	r_reg_DPHY_SW_WR_DESKEWL_CS1_CON4,
	r_reg_DPHY_SW_WR_DESKEWL_CS1_CON5,
	r_reg_DPHY_SW_WR_DESKEWL_CS1_CON6,
	r_reg_DPHY_SW_WR_DESKEWL_CS1_CON7,
	r_reg_DPHY_SW_DM_DESKEWL_CS1_CON0,
	r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON0,
	r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON1,
	r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON2,
	r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON3,
	r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON4,
	r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON5,
	r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON6,
	r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON7,
	r_reg_DPHY2_SW_DM_DESKEWC_CS0_CON0,
	r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON0,
	r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON1,
	r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON2,
	r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON3,
	r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON4,
	r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON5,
	r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON6,
	r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON7,
	r_reg_DPHY2_SW_DM_DESKEWC_CS1_CON0,
	r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON0,
	r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON1,
	r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON2,
	r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON3,
	r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON4,
	r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON5,
	r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON6,
	r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON7,
	r_reg_DPHY2_SW_DM_DESKEWL_CS0_CON0,
	r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON0,
	r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON1,
	r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON2,
	r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON3,
	r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON4,
	r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON5,
	r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON6,
	r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON7,
	r_reg_DPHY2_SW_DM_DESKEWL_CS1_CON0,
};

__CONST uint32_t g_ddr_train_restore_write_address[] = {
	[r_reg_DPHY_SW_WR_DESKEWC_CS0_CON0] = 0x105b0880,
	[r_reg_DPHY_SW_WR_DESKEWC_CS0_CON1] = 0x105b0884,
	[r_reg_DPHY_SW_WR_DESKEWC_CS0_CON2] = 0x105b0888,
	[r_reg_DPHY_SW_WR_DESKEWC_CS0_CON3] = 0x105b088c,
	[r_reg_DPHY_SW_WR_DESKEWC_CS0_CON4] = 0x105b0890,
	[r_reg_DPHY_SW_WR_DESKEWC_CS0_CON5] = 0x105b0894,
	[r_reg_DPHY_SW_WR_DESKEWC_CS0_CON6] = 0x105b0898,
	[r_reg_DPHY_SW_WR_DESKEWC_CS0_CON7] = 0x105b089c,
	[r_reg_DPHY_SW_DM_DESKEWC_CS0_CON0] = 0x105b08a0,
	[r_reg_DPHY_SW_WR_DESKEWC_CS1_CON0] = 0x105b08e0,
	[r_reg_DPHY_SW_WR_DESKEWC_CS1_CON1] = 0x105b08e4,
	[r_reg_DPHY_SW_WR_DESKEWC_CS1_CON2] = 0x105b08e8,
	[r_reg_DPHY_SW_WR_DESKEWC_CS1_CON3] = 0x105b08ec,
	[r_reg_DPHY_SW_WR_DESKEWC_CS1_CON4] = 0x105b08f0,
	[r_reg_DPHY_SW_WR_DESKEWC_CS1_CON5] = 0x105b08f4,
	[r_reg_DPHY_SW_WR_DESKEWC_CS1_CON6] = 0x105b08f8,
	[r_reg_DPHY_SW_WR_DESKEWC_CS1_CON7] = 0x105b08fc,
	[r_reg_DPHY_SW_DM_DESKEWC_CS1_CON0] = 0x105b0900,
	[r_reg_DPHY_SW_WR_DESKEWL_CS0_CON0] = 0x105b08b0,
	[r_reg_DPHY_SW_WR_DESKEWL_CS0_CON1] = 0x105b08b4,
	[r_reg_DPHY_SW_WR_DESKEWL_CS0_CON2] = 0x105b08b8,
	[r_reg_DPHY_SW_WR_DESKEWL_CS0_CON3] = 0x105b08bc,
	[r_reg_DPHY_SW_WR_DESKEWL_CS0_CON4] = 0x105b08c0,
	[r_reg_DPHY_SW_WR_DESKEWL_CS0_CON5] = 0x105b08c4,
	[r_reg_DPHY_SW_WR_DESKEWL_CS0_CON6] = 0x105b08c8,
	[r_reg_DPHY_SW_WR_DESKEWL_CS0_CON7] = 0x105b08cc,
	[r_reg_DPHY_SW_DM_DESKEWL_CS0_CON0] = 0x105b08d0,
	[r_reg_DPHY_SW_WR_DESKEWL_CS1_CON0] = 0x105b0910,
	[r_reg_DPHY_SW_WR_DESKEWL_CS1_CON1] = 0x105b0914,
	[r_reg_DPHY_SW_WR_DESKEWL_CS1_CON2] = 0x105b0918,
	[r_reg_DPHY_SW_WR_DESKEWL_CS1_CON3] = 0x105b091c,
	[r_reg_DPHY_SW_WR_DESKEWL_CS1_CON4] = 0x105b0920,
	[r_reg_DPHY_SW_WR_DESKEWL_CS1_CON5] = 0x105b0924,
	[r_reg_DPHY_SW_WR_DESKEWL_CS1_CON6] = 0x105b0928,
	[r_reg_DPHY_SW_WR_DESKEWL_CS1_CON7] = 0x105b092c,
	[r_reg_DPHY_SW_DM_DESKEWL_CS1_CON0] = 0x105b0930,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON0] = 0x105c0880,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON1] = 0x105c0884,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON2] = 0x105c0888,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON3] = 0x105c088c,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON4] = 0x105c0890,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON5] = 0x105c0894,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON6] = 0x105c0898,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS0_CON7] = 0x105c089c,
	[r_reg_DPHY2_SW_DM_DESKEWC_CS0_CON0] = 0x105c08a0,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON0] = 0x105c08e0,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON1] = 0x105c08e4,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON2] = 0x105c08e8,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON3] = 0x105c08ec,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON4] = 0x105c08f0,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON5] = 0x105c08f4,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON6] = 0x105c08f8,
	[r_reg_DPHY2_SW_WR_DESKEWC_CS1_CON7] = 0x105c08fc,
	[r_reg_DPHY2_SW_DM_DESKEWC_CS1_CON0] = 0x105c0900,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON0] = 0x105c08b0,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON1] = 0x105c08b4,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON2] = 0x105c08b8,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON3] = 0x105c08bc,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON4] = 0x105c08c0,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON5] = 0x105c08c4,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON6] = 0x105c08c8,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS0_CON7] = 0x105c08cc,
	[r_reg_DPHY2_SW_DM_DESKEWL_CS0_CON0] = 0x105c08d0,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON0] = 0x105c0910,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON1] = 0x105c0914,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON2] = 0x105c0918,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON3] = 0x105c091c,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON4] = 0x105c0920,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON5] = 0x105c0924,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON6] = 0x105c0928,
	[r_reg_DPHY2_SW_WR_DESKEWL_CS1_CON7] = 0x105c092c,
	[r_reg_DPHY2_SW_DM_DESKEWL_CS1_CON0] = 0x105c0930,
};

#define VREF_REF_NUM	0x05
#define PHY_MAX_VREF	0x3f
#define DRAM_MAX_VREF	0x32
#define VREF_FROM	0x0
#define VREF_STEP	0x1

#define DDRPHY_VREF_RD32(base, offset)		(RD_REG(base + offset))
#define DDRPHY_VREF_WR32(base, offset, data)	(WR_REG(base + offset, data))

#define VREF_PRBS_TIMEOUT_USEC	5000

typedef enum {
	VREF_READ = 0,
	VREF_WRITE
} eVref_op;

typedef enum {
	VREF_BYTE0 = 0,
	VREF_BYTE1,
	VREF_BYTE_ALL
} eVref_byte;

typedef enum {
	VREF_ERROR = -1,
	VREF_SUCCESS = 0,
	VREF_TIMEOUT
} eVref_error;

typedef enum {
	VREF_PRBS_SUCCESS = 0,
	VREF_PRBS_TIMEOUT
} eVref_prbs;

typedef enum {
	PHY0 = 0,
	PHY1
} ePHY;

#define DREX_BASE_ADDR		0x10580000
#define DDRPHY0_BASE_ADDR	0x105b0000
#define DDRPHY1_BASE_ADDR	0x105c0000

#define rDIRECTCMD_offset	0x10

#define rCAL_CON0_offset	0x004
#define rCAL_CON3_offset	0x010
#define rZQ_CON9_offset		0x3EC
#define rPRBS_CON0_offset	0x684
#define rPRBS_CON1_offset	0x688
#define rPRBS_CON4_offset	0x694
#define rPRBS_CON5_offset	0x698
#define rPRBS_CON6_offset	0x69c
#define rPRBS_CON7_offset	0x6a0

/* Little Endian representation of the register fields */
typedef union {
	uint32_t n;
	struct {
		 uint32_t wrlvl_mode                : ( 0 - 0 + 1 );
		 uint32_t gate_cal_mode             : ( 1 - 1 + 1 );
		 uint32_t ca_cal_mode               : ( 2 - 2 + 1 );
		 uint32_t rd_cal_mode               : ( 3 - 3 + 1 );
		 uint32_t lock_average_en           : ( 4 - 4 + 1 );
		 uint32_t wr_cal_mode               : ( 5 - 5 + 1 );
		 uint32_t rdlvl_dqs_edge_en         : ( 7 - 6 + 1 );
		 uint32_t wrlvl_start               : ( 8 - 8 + 1 );
		 uint32_t wrtrn_dqs_edge_en         : ( 10 - 9 + 1 );
		 uint32_t lock_sample_condition     : ( 12 - 11 + 1 );
		 uint32_t ctrl_upd_interval         : ( 14 - 13 + 1 );
		 uint32_t reserved_15_15            : ( 15 - 15 + 1 );
		 uint32_t wrlvl_resp                : ( 16 - 16 + 1 );
		 uint32_t reserved_17_17            : ( 17 - 17 + 1 );
		 uint32_t wr_per_rank_en            : ( 19 - 18 + 1 );
		 uint32_t byte_rdlvl_en             : ( 20 - 20 + 1 );
		 uint32_t ca_swap_mode              : ( 21 - 21 + 1 );
		 uint32_t cal_vtc_en                : ( 22 - 22 + 1 );
		 uint32_t freq_offset_en            : ( 23 - 23 + 1 );
		 uint32_t avg_window_size           : ( 25 - 24 + 1 );
		 uint32_t dvfs_wr_train_en          : ( 26 - 26 + 1 );
		 uint32_t gate_rdchk_en             : ( 27 - 27 + 1 );
		 uint32_t cs_default                : ( 31 - 28 + 1 );
	} bits;
} sCAL_CON0; //0x0004

typedef union {
	uint32_t n;
	struct {
		uint32_t pcfg_mode                 : ( 2 - 0 + 1 );
		uint32_t reserved_3_3              : ( 3 - 3 + 1 );
		uint32_t min_locktime_adj          : ( 5 - 4 + 1 );
		uint32_t prbs_sw_mode              : ( 6 - 6 + 1 );
		uint32_t wrlvl_sw_mode             : ( 7 - 7 + 1 );
		uint32_t rd_sw_mode                : ( 8 - 8 + 1 );
		uint32_t wr_sw_mode                : ( 9 - 9 + 1 );
		uint32_t gt_sw_mode                : ( 10 - 10 + 1 );
		uint32_t auto_dqs_clean            : ( 11 - 11 + 1 );
		uint32_t upd_ack_cycle             : ( 15 - 12 + 1 );
		uint32_t mcupd_req_cycle           : ( 21 - 16 + 1 );
		uint32_t dvfs_wait_cycle           : ( 25 - 22 + 1 );
		uint32_t phyupd_req_cycle          : ( 31 - 26 + 1 );
	} bits;
} sCAL_CON3; //0x0010

typedef union {
	uint32_t n;
	struct {
		 uint32_t zq_ds0_vref               : ( 5 - 0 + 1 );
		 uint32_t zq_ds0_vref_fsbst         : ( 6 - 6 + 1 );
		 uint32_t zq_ds0_vref_pd            : ( 7 - 7 + 1 );
		 uint32_t zq_ds1_vref               : ( 13 - 8 + 1 );
		 uint32_t zq_ds1_vref_fsbst         : ( 14 - 14 + 1 );
		 uint32_t zq_ds1_vref_pd            : ( 15 - 15 + 1 );
		 uint32_t reserved_16_31            : ( 31 - 16 + 1 );
	} bits;
} sZQ_CON9; //0x03EC

typedef union {
	uint32_t n;
	struct {
		 uint32_t prbs_done                 : ( 0 - 0 + 1 );
		 uint32_t prbs_read_start           : ( 1 - 1 + 1 );
		 uint32_t prbs_write_start          : ( 2 - 2 + 1 );
		 uint32_t reserved_3_15             : ( 15 - 3 + 1 );
		 uint32_t prbs_pattern              : ( 31 - 16 + 1 );
	} bits;
} sPRBS_CON0; //0x0684

typedef union {
	uint32_t n;
	struct {
		 uint32_t prbs_tresync              : ( 4 - 0 + 1 );
		 uint32_t prbs_trddata_en_adj       : ( 8 - 5 + 1 );
		 uint32_t reserved_9_12             : ( 12 - 9 + 1 );
		 uint32_t prbs_twr2rd               : ( 17 - 13 + 1 );
		 uint32_t prbs_wrlat                : ( 23 - 18 + 1 );
		 uint32_t reserved_24_25            : ( 25 - 24 + 1 );
		 uint32_t prbs_dbi_en               : ( 26 - 26 +1 );
		 uint32_t reserved_27_27            : ( 27 - 27 + 1 );
		 uint32_t prbs_cs                   : ( 31 - 28 +1 );
	} bits;
} sPRBS_CON1; //0x0688

typedef union {
	uint32_t n;
	struct {
		 uint32_t sw_prbs_offsetr0          : ( 8 - 0 + 1 );
		 uint32_t reserved_9_15             : ( 15 - 9 + 1 );
		 uint32_t sw_prbs_offsetr1          : ( 24 - 16 + 1 );
		 uint32_t reserved_31_25            : ( 31 - 25 + 1 );
	} bits;
} sPRBS_CON4; //0x0694

typedef union {
	uint32_t n;
	struct {
		 uint32_t sw_prbs_offsetw0          : ( 8 - 0 + 1 );
		 uint32_t reserved_9_15             : ( 15 - 9 + 1 );
		 uint32_t sw_prbs_offsetw1          : ( 24 - 16 + 1 );
		 uint32_t reserved_31_25            : ( 31 - 25 + 1 );
	} bits;
} sPRBS_CON5; //0x0698

typedef union {
	uint32_t n;
	struct {
		 uint32_t prbs_offset_left0         : ( 8 - 0 + 1 );
		 uint32_t reserved_9_15             : ( 15 - 9 + 1 );
		 uint32_t prbs_offset_left1         : ( 24 - 16 + 1 );
		 uint32_t reserved_31_25            : ( 31 - 25 + 1 );
	} bits;
} sPRBS_CON6; //0x069c

typedef union {
	uint32_t n;
	struct {
		 uint32_t prbs_offset_right0        : ( 8 - 0 + 1 );
		 uint32_t reserved_9_15             : ( 15 - 9 + 1 );
		 uint32_t prbs_offset_right1        : ( 24 - 16 + 1 );
		 uint32_t reserved_31_25            : ( 31 - 25 + 1 );
	} bits;
} sPRBS_CON7; //0x06a0

#endif /* _AIRBRUSH_DDR_INTERNAL_H_ */
