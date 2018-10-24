/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR Initialization and Training sequence.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "airbrush-ddr.h"
#include "airbrush-ddr-internal.h"
#include "airbrush-pmic-ctrl.h"
#include "airbrush-regs.h"

/* In case the DDR Initialization/Training OTPs are already fused to the OTP
 * array, then the below function pointer is updated with the address of the
 * function which reads OTPs from the fused OTP memory.
 *
 * In case the OTPs are not fused, this will point to the function which reads
 * the OTPs from the global array stored in the data segment.
 */
static uint32_t (*ddr_otp_rd)(uint32_t);

static uint32_t g_ddr_train_save_regs[] = {
	[s_DPHY_MDLL_CON1] = 0x105b00b4,
	[s_DPHY2_MDLL_CON1] = 0x105c00b4,
	[s_DPHY_CA_DESKEW_CON0] = 0x105b007c,
	[s_DPHY_CA_DESKEW_CON1] = 0x105b0080,
	[s_DPHY_CA_DESKEW_CON2] = 0x105b0084,
	[s_DPHY2_CA_DESKEW_CON0] = 0x105c007c,
	[s_DPHY2_CA_DESKEW_CON1] = 0x105c0080,
	[s_DPHY2_CA_DESKEW_CON2] = 0x105c0084,

	[s_DPHY_RD_DESKEW_CENTER_CS0_CON_DM] = 0x105b018c,
	[s_DPHY_RD_DESKEW_CENTER_CS0_CON0] = 0x105b0190,
	[s_DPHY_RD_DESKEW_CENTER_CS0_CON1] = 0x105b019c,
	[s_DPHY_RD_DESKEW_CENTER_CS0_CON2] = 0x105b01a8,
	[s_DPHY_RD_DESKEW_CENTER_CS0_CON3] = 0x105b01b4,
	[s_DPHY_RD_DESKEW_CENTER_CS0_CON4] = 0x105b01c0,
	[s_DPHY_RD_DESKEW_CENTER_CS0_CON5] = 0x105b01cc,
	[s_DPHY_RD_DESKEW_CENTER_CS0_CON6] = 0x105b01d8,
	[s_DPHY_RD_DESKEW_CENTER_CS0_CON7] = 0x105b01e4,
	[s_DPHY_RD_DESKEW_CENTER_CS1_CON_DM] = 0x105b076c,
	[s_DPHY_RD_DESKEW_CENTER_CS1_CON0] = 0x105b0770,
	[s_DPHY_RD_DESKEW_CENTER_CS1_CON1] = 0x105b0774,
	[s_DPHY_RD_DESKEW_CENTER_CS1_CON2] = 0x105b0778,
	[s_DPHY_RD_DESKEW_CENTER_CS1_CON3] = 0x105b077c,
	[s_DPHY_RD_DESKEW_CENTER_CS1_CON4] = 0x105b0780,
	[s_DPHY_RD_DESKEW_CENTER_CS1_CON5] = 0x105b0784,
	[s_DPHY_RD_DESKEW_CENTER_CS1_CON6] = 0x105b0788,
	[s_DPHY_RD_DESKEW_CENTER_CS1_CON7] = 0x105b078c,
	[s_DPHY_RD_DESKEW_LEFT_CS0_CON_DM] = 0x105b0610,
	[s_DPHY_RD_DESKEW_LEFT_CS0_CON0] = 0x105b0614,
	[s_DPHY_RD_DESKEW_LEFT_CS0_CON1] = 0x105b0620,
	[s_DPHY_RD_DESKEW_LEFT_CS0_CON2] = 0x105b062c,
	[s_DPHY_RD_DESKEW_LEFT_CS0_CON3] = 0x105b0638,
	[s_DPHY_RD_DESKEW_LEFT_CS0_CON4] = 0x105b0644,
	[s_DPHY_RD_DESKEW_LEFT_CS0_CON5] = 0x105b0650,
	[s_DPHY_RD_DESKEW_LEFT_CS0_CON6] = 0x105b065c,
	[s_DPHY_RD_DESKEW_LEFT_CS0_CON7] = 0x105b0668,
	[s_DPHY_RD_DESKEW_LEFT_CS1_CON_DM] = 0x105b0790,
	[s_DPHY_RD_DESKEW_LEFT_CS1_CON0] = 0x105b0794,
	[s_DPHY_RD_DESKEW_LEFT_CS1_CON1] = 0x105b0798,
	[s_DPHY_RD_DESKEW_LEFT_CS1_CON2] = 0x105b079c,
	[s_DPHY_RD_DESKEW_LEFT_CS1_CON3] = 0x105b07a0,
	[s_DPHY_RD_DESKEW_LEFT_CS1_CON4] = 0x105b07a4,
	[s_DPHY_RD_DESKEW_LEFT_CS1_CON5] = 0x105b07a8,
	[s_DPHY_RD_DESKEW_LEFT_CS1_CON6] = 0x105b07ac,
	[s_DPHY_RD_DESKEW_LEFT_CS1_CON7] = 0x105b07b0,
	[s_DPHY_RD_DQS_VWMC_CS0_CON0] = 0x105b0580,
	[s_DPHY_RD_DQS_VWMC_CS1_CON0] = 0x105b0768,
	[s_DPHY_RD_DQS_VWML_CS0_CON0] = 0x105b0574,
	[s_DPHY_RD_DQS_VWML_CS1_CON0] = 0x105b0764,

	[s_DPHY2_RD_DESKEW_CENTER_CS0_CON_DM] = 0x105c018c,
	[s_DPHY2_RD_DESKEW_CENTER_CS0_CON0] = 0x105c0190,
	[s_DPHY2_RD_DESKEW_CENTER_CS0_CON1] = 0x105c019c,
	[s_DPHY2_RD_DESKEW_CENTER_CS0_CON2] = 0x105c01a8,
	[s_DPHY2_RD_DESKEW_CENTER_CS0_CON3] = 0x105c01b4,
	[s_DPHY2_RD_DESKEW_CENTER_CS0_CON4] = 0x105c01c0,
	[s_DPHY2_RD_DESKEW_CENTER_CS0_CON5] = 0x105c01cc,
	[s_DPHY2_RD_DESKEW_CENTER_CS0_CON6] = 0x105c01d8,
	[s_DPHY2_RD_DESKEW_CENTER_CS0_CON7] = 0x105c01e4,
	[s_DPHY2_RD_DESKEW_CENTER_CS1_CON_DM] = 0x105c076c,
	[s_DPHY2_RD_DESKEW_CENTER_CS1_CON0] = 0x105c0770,
	[s_DPHY2_RD_DESKEW_CENTER_CS1_CON1] = 0x105c0774,
	[s_DPHY2_RD_DESKEW_CENTER_CS1_CON2] = 0x105c0778,
	[s_DPHY2_RD_DESKEW_CENTER_CS1_CON3] = 0x105c077c,
	[s_DPHY2_RD_DESKEW_CENTER_CS1_CON4] = 0x105c0780,
	[s_DPHY2_RD_DESKEW_CENTER_CS1_CON5] = 0x105c0784,
	[s_DPHY2_RD_DESKEW_CENTER_CS1_CON6] = 0x105c0788,
	[s_DPHY2_RD_DESKEW_CENTER_CS1_CON7] = 0x105c078c,
	[s_DPHY2_RD_DESKEW_LEFT_CS0_CON_DM] = 0x105c0610,
	[s_DPHY2_RD_DESKEW_LEFT_CS0_CON0] = 0x105c0614,
	[s_DPHY2_RD_DESKEW_LEFT_CS0_CON1] = 0x105c0620,
	[s_DPHY2_RD_DESKEW_LEFT_CS0_CON2] = 0x105c062c,
	[s_DPHY2_RD_DESKEW_LEFT_CS0_CON3] = 0x105c0638,
	[s_DPHY2_RD_DESKEW_LEFT_CS0_CON4] = 0x105c0644,
	[s_DPHY2_RD_DESKEW_LEFT_CS0_CON5] = 0x105c0650,
	[s_DPHY2_RD_DESKEW_LEFT_CS0_CON6] = 0x105c065c,
	[s_DPHY2_RD_DESKEW_LEFT_CS0_CON7] = 0x105c0668,
	[s_DPHY2_RD_DESKEW_LEFT_CS1_CON_DM] = 0x105c0790,
	[s_DPHY2_RD_DESKEW_LEFT_CS1_CON0] = 0x105c0794,
	[s_DPHY2_RD_DESKEW_LEFT_CS1_CON1] = 0x105c0798,
	[s_DPHY2_RD_DESKEW_LEFT_CS1_CON2] = 0x105c079c,
	[s_DPHY2_RD_DESKEW_LEFT_CS1_CON3] = 0x105c07a0,
	[s_DPHY2_RD_DESKEW_LEFT_CS1_CON4] = 0x105c07a4,
	[s_DPHY2_RD_DESKEW_LEFT_CS1_CON5] = 0x105c07a8,
	[s_DPHY2_RD_DESKEW_LEFT_CS1_CON6] = 0x105c07ac,
	[s_DPHY2_RD_DESKEW_LEFT_CS1_CON7] = 0x105c07b0,
	[s_DPHY2_RD_DQS_VWMC_CS0_CON0] = 0x105c0580,
	[s_DPHY2_RD_DQS_VWMC_CS1_CON0] = 0x105c0768,
	[s_DPHY2_RD_DQS_VWML_CS0_CON0] = 0x105c0574,
	[s_DPHY2_RD_DQS_VWML_CS1_CON0] = 0x105c0764,

	[s_DPHY_WR_DESKEWC_CS0_CON0] = 0x105b01f0,
	[s_DPHY_WR_DESKEWC_CS0_CON1] = 0x105b01fc,
	[s_DPHY_WR_DESKEWC_CS0_CON2] = 0x105b0208,
	[s_DPHY_WR_DESKEWC_CS0_CON3] = 0x105b0214,
	[s_DPHY_WR_DESKEWC_CS0_CON4] = 0x105b0220,
	[s_DPHY_WR_DESKEWC_CS0_CON5] = 0x105b022c,
	[s_DPHY_WR_DESKEWC_CS0_CON6] = 0x105b0238,
	[s_DPHY_WR_DESKEWC_CS0_CON7] = 0x105b0244,
	[s_DPHY_DM_DESKEWC_CS0_CON0] = 0x105b0250,
	[s_DPHY_WR_DESKEWC_CS1_CON0] = 0x105b0410,
	[s_DPHY_WR_DESKEWC_CS1_CON1] = 0x105b041c,
	[s_DPHY_WR_DESKEWC_CS1_CON2] = 0x105b0428,
	[s_DPHY_WR_DESKEWC_CS1_CON3] = 0x105b0434,
	[s_DPHY_WR_DESKEWC_CS1_CON4] = 0x105b0440,
	[s_DPHY_WR_DESKEWC_CS1_CON5] = 0x105b044c,
	[s_DPHY_WR_DESKEWC_CS1_CON6] = 0x105b0458,
	[s_DPHY_WR_DESKEWC_CS1_CON7] = 0x105b0464,
	[s_DPHY_DM_DESKEWC_CS1_CON0] = 0x105b0470,
	[s_DPHY_WR_DESKEWL_CS0_CON0] = 0x105b0490,
	[s_DPHY_WR_DESKEWL_CS0_CON1] = 0x105b049c,
	[s_DPHY_WR_DESKEWL_CS0_CON2] = 0x105b04a8,
	[s_DPHY_WR_DESKEWL_CS0_CON3] = 0x105b04b4,
	[s_DPHY_WR_DESKEWL_CS0_CON4] = 0x105b04c0,
	[s_DPHY_WR_DESKEWL_CS0_CON5] = 0x105b04cc,
	[s_DPHY_WR_DESKEWL_CS0_CON6] = 0x105b04d8,
	[s_DPHY_WR_DESKEWL_CS0_CON7] = 0x105b04e4,
	[s_DPHY_DM_DESKEWL_CS0_CON0] = 0x105b04f0,
	[s_DPHY_WR_DESKEWL_CS1_CON0] = 0x105b0500,
	[s_DPHY_WR_DESKEWL_CS1_CON1] = 0x105b050c,
	[s_DPHY_WR_DESKEWL_CS1_CON2] = 0x105b0518,
	[s_DPHY_WR_DESKEWL_CS1_CON3] = 0x105b0524,
	[s_DPHY_WR_DESKEWL_CS1_CON4] = 0x105b0530,
	[s_DPHY_WR_DESKEWL_CS1_CON5] = 0x105b053c,
	[s_DPHY_WR_DESKEWL_CS1_CON6] = 0x105b0548,
	[s_DPHY_WR_DESKEWL_CS1_CON7] = 0x105b0554,
	[s_DPHY_DM_DESKEWL_CS1_CON0] = 0x105b0560,

	[s_DPHY2_WR_DESKEWC_CS0_CON0] = 0x105c01f0,
	[s_DPHY2_WR_DESKEWC_CS0_CON1] = 0x105c01fc,
	[s_DPHY2_WR_DESKEWC_CS0_CON2] = 0x105c0208,
	[s_DPHY2_WR_DESKEWC_CS0_CON3] = 0x105c0214,
	[s_DPHY2_WR_DESKEWC_CS0_CON4] = 0x105c0220,
	[s_DPHY2_WR_DESKEWC_CS0_CON5] = 0x105c022c,
	[s_DPHY2_WR_DESKEWC_CS0_CON6] = 0x105c0238,
	[s_DPHY2_WR_DESKEWC_CS0_CON7] = 0x105c0244,
	[s_DPHY2_DM_DESKEWC_CS0_CON0] = 0x105c0250,
	[s_DPHY2_WR_DESKEWC_CS1_CON0] = 0x105c0410,
	[s_DPHY2_WR_DESKEWC_CS1_CON1] = 0x105c041c,
	[s_DPHY2_WR_DESKEWC_CS1_CON2] = 0x105c0428,
	[s_DPHY2_WR_DESKEWC_CS1_CON3] = 0x105c0434,
	[s_DPHY2_WR_DESKEWC_CS1_CON4] = 0x105c0440,
	[s_DPHY2_WR_DESKEWC_CS1_CON5] = 0x105c044c,
	[s_DPHY2_WR_DESKEWC_CS1_CON6] = 0x105c0458,
	[s_DPHY2_WR_DESKEWC_CS1_CON7] = 0x105c0464,
	[s_DPHY2_DM_DESKEWC_CS1_CON0] = 0x105c0470,
	[s_DPHY2_WR_DESKEWL_CS0_CON0] = 0x105c0490,
	[s_DPHY2_WR_DESKEWL_CS0_CON1] = 0x105c049c,
	[s_DPHY2_WR_DESKEWL_CS0_CON2] = 0x105c04a8,
	[s_DPHY2_WR_DESKEWL_CS0_CON3] = 0x105c04b4,
	[s_DPHY2_WR_DESKEWL_CS0_CON4] = 0x105c04c0,
	[s_DPHY2_WR_DESKEWL_CS0_CON5] = 0x105c04cc,
	[s_DPHY2_WR_DESKEWL_CS0_CON6] = 0x105c04d8,
	[s_DPHY2_WR_DESKEWL_CS0_CON7] = 0x105c04e4,
	[s_DPHY2_DM_DESKEWL_CS0_CON0] = 0x105c04f0,
	[s_DPHY2_WR_DESKEWL_CS1_CON0] = 0x105c0500,
	[s_DPHY2_WR_DESKEWL_CS1_CON1] = 0x105c050c,
	[s_DPHY2_WR_DESKEWL_CS1_CON2] = 0x105c0518,
	[s_DPHY2_WR_DESKEWL_CS1_CON3] = 0x105c0524,
	[s_DPHY2_WR_DESKEWL_CS1_CON4] = 0x105c0530,
	[s_DPHY2_WR_DESKEWL_CS1_CON5] = 0x105c053c,
	[s_DPHY2_WR_DESKEWL_CS1_CON6] = 0x105c0548,
	[s_DPHY2_WR_DESKEWL_CS1_CON7] = 0x105c0554,
	[s_DPHY2_DM_DESKEWL_CS1_CON0] = 0x105c0560,
	[s_DPHY_PRBS_CON2] = 0x105b068c,
	[s_DPHY_PRBS_CON3] = 0x105b0690,
	[s_DPHY2_PRBS_CON2] = 0x105c068c,
	[s_DPHY2_PRBS_CON3] = 0x105c0690,
	[s_DPHY_ZQ_CON9] = 0x105b03ec,
	[s_DPHY2_ZQ_CON9] = 0x105c03ec,
};

static uint32_t g_ddr_train_restore_read_regs[] = {
	[r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON_DM] = 0x105b07c8,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON0] = 0x105b07cc,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON1] = 0x105b07d0,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON2] = 0x105b07d4,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON3] = 0x105b07d8,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON4] = 0x105b07dc,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON5] = 0x105b07e0,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON6] = 0x105b07e4,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS0_CON7] = 0x105b07e8,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON_DM] = 0x105b0820,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON0] = 0x105b0824,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON1] = 0x105b0828,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON2] = 0x105b082c,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON3] = 0x105b0830,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON4] = 0x105b0834,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON5] = 0x105b0838,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON6] = 0x105b083c,
	[r_DPHY_SW_RD_DESKEW_CENTER_CS1_CON7] = 0x105b0840,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON_DM] = 0x105b07f0,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON0] = 0x105b07f4,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON1] = 0x105b07f8,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON2] = 0x105b07fc,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON3] = 0x105b0800,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON4] = 0x105b0804,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON5] = 0x105b0808,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON6] = 0x105b080c,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS0_CON7] = 0x105b0810,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON_DM] = 0x105b0850,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON0] = 0x105b0854,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON1] = 0x105b0858,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON2] = 0x105b085c,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON3] = 0x105b0860,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON4] = 0x105b0864,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON5] = 0x105b0868,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON6] = 0x105b086c,
	[r_DPHY_SW_RD_DESKEW_LEFT_CS1_CON7] = 0x105b0870,
	[r_DPHY_SW_RD_DQS_VWMC_CS0_CON0] = 0x105b07c4,
	[r_DPHY_SW_RD_DQS_VWMC_CS1_CON0] = 0x105b0818,
	[r_DPHY_SW_RD_DQS_VWML_CS0_CON0] = 0x105b07c0,
	[r_DPHY_SW_RD_DQS_VWML_CS1_CON0] = 0x105b0814,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON_DM] = 0x105c07c8,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON0] = 0x105c07cc,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON1] = 0x105c07d0,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON2] = 0x105c07d4,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON3] = 0x105c07d8,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON4] = 0x105c07dc,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON5] = 0x105c07e0,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON6] = 0x105c07e4,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS0_CON7] = 0x105c07e8,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON_DM] = 0x105c0820,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON0] = 0x105c0824,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON1] = 0x105c0828,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON2] = 0x105c082c,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON3] = 0x105c0830,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON4] = 0x105c0834,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON5] = 0x105c0838,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON6] = 0x105c083c,
	[r_DPHY2_SW_RD_DESKEW_CENTER_CS1_CON7] = 0x105c0840,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON_DM] = 0x105c07f0,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON0] = 0x105c07f4,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON1] = 0x105c07f8,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON2] = 0x105c07fc,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON3] = 0x105c0800,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON4] = 0x105c0804,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON5] = 0x105c0808,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON6] = 0x105c080c,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS0_CON7] = 0x105c0810,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON_DM] = 0x105c0850,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON0] = 0x105c0854,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON1] = 0x105c0858,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON2] = 0x105c085c,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON3] = 0x105c0860,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON4] = 0x105c0864,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON5] = 0x105c0868,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON6] = 0x105c086c,
	[r_DPHY2_SW_RD_DESKEW_LEFT_CS1_CON7] = 0x105c0870,
	[r_DPHY2_SW_RD_DQS_VWMC_CS0_CON0] = 0x105c07c4,
	[r_DPHY2_SW_RD_DQS_VWMC_CS1_CON0] = 0x105c0818,
	[r_DPHY2_SW_RD_DQS_VWML_CS0_CON0] = 0x105c07c0,
	[r_DPHY2_SW_RD_DQS_VWML_CS1_CON0] = 0x105c0814,
};

static uint32_t g_ddr_train_restore_write_regs[] = {
	[r_DPHY_SW_WR_DESKEWC_CS0_CON0] = 0x105b0880,
	[r_DPHY_SW_WR_DESKEWC_CS0_CON1] = 0x105b0884,
	[r_DPHY_SW_WR_DESKEWC_CS0_CON2] = 0x105b0888,
	[r_DPHY_SW_WR_DESKEWC_CS0_CON3] = 0x105b088c,
	[r_DPHY_SW_WR_DESKEWC_CS0_CON4] = 0x105b0890,
	[r_DPHY_SW_WR_DESKEWC_CS0_CON5] = 0x105b0894,
	[r_DPHY_SW_WR_DESKEWC_CS0_CON6] = 0x105b0898,
	[r_DPHY_SW_WR_DESKEWC_CS0_CON7] = 0x105b089c,
	[r_DPHY_SW_DM_DESKEWC_CS0_CON0] = 0x105b08a0,
	[r_DPHY_SW_WR_DESKEWC_CS1_CON0] = 0x105b08e0,
	[r_DPHY_SW_WR_DESKEWC_CS1_CON1] = 0x105b08e4,
	[r_DPHY_SW_WR_DESKEWC_CS1_CON2] = 0x105b08e8,
	[r_DPHY_SW_WR_DESKEWC_CS1_CON3] = 0x105b08ec,
	[r_DPHY_SW_WR_DESKEWC_CS1_CON4] = 0x105b08f0,
	[r_DPHY_SW_WR_DESKEWC_CS1_CON5] = 0x105b08f4,
	[r_DPHY_SW_WR_DESKEWC_CS1_CON6] = 0x105b08f8,
	[r_DPHY_SW_WR_DESKEWC_CS1_CON7] = 0x105b08fc,
	[r_DPHY_SW_DM_DESKEWC_CS1_CON0] = 0x105b0900,
	[r_DPHY_SW_WR_DESKEWL_CS0_CON0] = 0x105b08b0,
	[r_DPHY_SW_WR_DESKEWL_CS0_CON1] = 0x105b08b4,
	[r_DPHY_SW_WR_DESKEWL_CS0_CON2] = 0x105b08b8,
	[r_DPHY_SW_WR_DESKEWL_CS0_CON3] = 0x105b08bc,
	[r_DPHY_SW_WR_DESKEWL_CS0_CON4] = 0x105b08c0,
	[r_DPHY_SW_WR_DESKEWL_CS0_CON5] = 0x105b08c4,
	[r_DPHY_SW_WR_DESKEWL_CS0_CON6] = 0x105b08c8,
	[r_DPHY_SW_WR_DESKEWL_CS0_CON7] = 0x105b08cc,
	[r_DPHY_SW_DM_DESKEWL_CS0_CON0] = 0x105b08d0,
	[r_DPHY_SW_WR_DESKEWL_CS1_CON0] = 0x105b0910,
	[r_DPHY_SW_WR_DESKEWL_CS1_CON1] = 0x105b0914,
	[r_DPHY_SW_WR_DESKEWL_CS1_CON2] = 0x105b0918,
	[r_DPHY_SW_WR_DESKEWL_CS1_CON3] = 0x105b091c,
	[r_DPHY_SW_WR_DESKEWL_CS1_CON4] = 0x105b0920,
	[r_DPHY_SW_WR_DESKEWL_CS1_CON5] = 0x105b0924,
	[r_DPHY_SW_WR_DESKEWL_CS1_CON6] = 0x105b0928,
	[r_DPHY_SW_WR_DESKEWL_CS1_CON7] = 0x105b092c,
	[r_DPHY_SW_DM_DESKEWL_CS1_CON0] = 0x105b0930,
	[r_DPHY2_SW_WR_DESKEWC_CS0_CON0] = 0x105c0880,
	[r_DPHY2_SW_WR_DESKEWC_CS0_CON1] = 0x105c0884,
	[r_DPHY2_SW_WR_DESKEWC_CS0_CON2] = 0x105c0888,
	[r_DPHY2_SW_WR_DESKEWC_CS0_CON3] = 0x105c088c,
	[r_DPHY2_SW_WR_DESKEWC_CS0_CON4] = 0x105c0890,
	[r_DPHY2_SW_WR_DESKEWC_CS0_CON5] = 0x105c0894,
	[r_DPHY2_SW_WR_DESKEWC_CS0_CON6] = 0x105c0898,
	[r_DPHY2_SW_WR_DESKEWC_CS0_CON7] = 0x105c089c,
	[r_DPHY2_SW_DM_DESKEWC_CS0_CON0] = 0x105c08a0,
	[r_DPHY2_SW_WR_DESKEWC_CS1_CON0] = 0x105c08e0,
	[r_DPHY2_SW_WR_DESKEWC_CS1_CON1] = 0x105c08e4,
	[r_DPHY2_SW_WR_DESKEWC_CS1_CON2] = 0x105c08e8,
	[r_DPHY2_SW_WR_DESKEWC_CS1_CON3] = 0x105c08ec,
	[r_DPHY2_SW_WR_DESKEWC_CS1_CON4] = 0x105c08f0,
	[r_DPHY2_SW_WR_DESKEWC_CS1_CON5] = 0x105c08f4,
	[r_DPHY2_SW_WR_DESKEWC_CS1_CON6] = 0x105c08f8,
	[r_DPHY2_SW_WR_DESKEWC_CS1_CON7] = 0x105c08fc,
	[r_DPHY2_SW_DM_DESKEWC_CS1_CON0] = 0x105c0900,
	[r_DPHY2_SW_WR_DESKEWL_CS0_CON0] = 0x105c08b0,
	[r_DPHY2_SW_WR_DESKEWL_CS0_CON1] = 0x105c08b4,
	[r_DPHY2_SW_WR_DESKEWL_CS0_CON2] = 0x105c08b8,
	[r_DPHY2_SW_WR_DESKEWL_CS0_CON3] = 0x105c08bc,
	[r_DPHY2_SW_WR_DESKEWL_CS0_CON4] = 0x105c08c0,
	[r_DPHY2_SW_WR_DESKEWL_CS0_CON5] = 0x105c08c4,
	[r_DPHY2_SW_WR_DESKEWL_CS0_CON6] = 0x105c08c8,
	[r_DPHY2_SW_WR_DESKEWL_CS0_CON7] = 0x105c08cc,
	[r_DPHY2_SW_DM_DESKEWL_CS0_CON0] = 0x105c08d0,
	[r_DPHY2_SW_WR_DESKEWL_CS1_CON0] = 0x105c0910,
	[r_DPHY2_SW_WR_DESKEWL_CS1_CON1] = 0x105c0914,
	[r_DPHY2_SW_WR_DESKEWL_CS1_CON2] = 0x105c0918,
	[r_DPHY2_SW_WR_DESKEWL_CS1_CON3] = 0x105c091c,
	[r_DPHY2_SW_WR_DESKEWL_CS1_CON4] = 0x105c0920,
	[r_DPHY2_SW_WR_DESKEWL_CS1_CON5] = 0x105c0924,
	[r_DPHY2_SW_WR_DESKEWL_CS1_CON6] = 0x105c0928,
	[r_DPHY2_SW_WR_DESKEWL_CS1_CON7] = 0x105c092c,
	[r_DPHY2_SW_DM_DESKEWL_CS1_CON0] = 0x105c0930,
};

static struct ddr_reg_poll_t ddr_reg_poll_array[] = {
	[p_pll_con0_pll_phy_mif]	= { 0x20000000, 0x20000000, 5000 },
	/* zq_done[0] bit */
	[p_DPHY_ZQ_CON1]		= { 0x00000001, 0x00000001, 5000 },
	/* ctrl_locked[18] bit */
	[p_DPHY_MDLL_CON1]		= { 0x00040000, 0x00040000, 5000 },
	/* dfi_init_complete[4:3] */
	[p_DREX_PHYSTATUS_dfi]		= { 0x00000018, 0x00000018, 5000 },
	/* train_complete[15:14] */
	[p_DREX_PHYSTATUS_train]	= { 0x0000C000, 0x0000C000, 5000 },
	/* chip_sref_state[8]=1 */
	[p_DREX_CHIPSTATUS_sr_enter]	= { 0x00000100, 0x00000100, 5000 },
	/* chip_sref_state[8]=0 */
	[p_DREX_CHIPSTATUS_sr_exit]	= { 0x00000100, 0x00000000, 5000 },
	/* prbs_done[0] = 1 */
	[p_DPHY_PRBS_CON0_prbs_done]	= { 0x00000001, 0x00000001, 5000 },
	/* prbs_disable[0] = 0 */
	[p_DPHY_PRBS_CON0_prbs_disable]	= { 0x00000001, 0x00000000, 5000 },
};

static inline void ddr_reg_wr_otp(uint32_t addr, uint32_t otp_idx)
{
	ddr_reg_wr(addr, ddr_otp_rd(otp_idx));
}

static int ddr_reg_poll(uint32_t reg, uint32_t poll_idx)
{
	struct ddr_reg_poll_t *poll = &ddr_reg_poll_array[poll_idx];
	uint32_t poll_multiplier;
	unsigned long timeout;

	poll_multiplier = ddr_otp_rd(o_SECURE_JTAG2) + 1;

	timeout = jiffies +
		  usecs_to_jiffies(poll->usec_timeout * poll_multiplier);
	while (((ddr_reg_rd(reg) & poll->mask) != poll->val) &&
			time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	if ((ddr_reg_rd(reg) & poll->mask) != poll->val)
		return DDR_FAIL;

	return DDR_SUCCESS;
}

static int ddr_config_cmu_mif_lowfreq(void)
{
	ddr_reg_wr_otp(pll_locktime_pll_phy_mif, o_Reserved_DDR_INIT_0);
	ddr_reg_wr_otp(pll_con0_pll_phy_mif, o_SECURE_JTAG0);

	if (ddr_reg_poll(pll_con0_pll_phy_mif, p_pll_con0_pll_phy_mif))
		return DDR_FAIL;

	ddr_reg_wr_otp(pll_con0_pll_phy_mif, o_SECURE_JTAG1);
	ddr_reg_wr_otp(clk_con_div_div4_pllclk_mif, o_Reserved_DDR_INIT_3);
	ddr_reg_wr_otp(clk_con_div_div2_pllclk_mif, o_Reserved_DDR_INIT_4);
	ddr_reg_wr_otp(clk_con_div_dfi_div2, o_Reserved_DDR_INIT_3);

	return DDR_SUCCESS;
}

static int ddr_config_cmu_mif_highfreq(void)
{
	ddr_reg_set(DREX_MEMCONTROL, 0x00000001);
	ddr_reg_wr(mif_pll_wrap_ctrl_reg, 0x0000000a);
	ddr_reg_wr(pll_con0_pll_phy_mif, 0x0);
	ddr_reg_wr_otp(pll_con0_pll_phy_mif, o_Reserved_DDR_INIT_1);

	if (ddr_reg_poll(pll_con0_pll_phy_mif, p_pll_con0_pll_phy_mif))
		return DDR_FAIL;

	ddr_reg_wr_otp(pll_con0_pll_phy_mif, o_Reserved_DDR_INIT_2);
	ddr_reg_wr(mif_pll_wrap_ctrl_reg, 0x0000000e);
	ddr_reg_clr(DREX_MEMCONTROL, 0x00000001);

	return DDR_SUCCESS;
}

static void ddr_ungate_phy_clock(void)
{
	ddr_reg_wr(mif_pll_wrap_ctrl_reg, 0x00000008);
	ddr_reg_wr(mif_pll_wrap_ctrl_reg, 0x0000000a);
	ddr_reg_wr(mif_pll_wrap_ctrl_reg, 0x0000000e);
}

static void ddr_deassert_phy_reset_while_phy_is_gated(void)
{
	ddr_reg_wr(phy0_init_ctrl_reg, 0x00008021);
	ddr_reg_wr(phy1_init_ctrl_reg, 0x00008021);
	ddr_reg_wr(phy0_rst_ctrl_reg, 0x00008080);
	ddr_reg_wr(phy1_rst_ctrl_reg, 0x00008080);
}

static void ddr_initialize_phy_pre(void)
{
	ddr_reg_wr_otp(DREX_ACTIVATE_AXI_READY, o_Reserved_DDR_INIT_12);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);
}

static void ddr_initialize_phy(void)
{
	ddr_reg_wr_otp(DPHY_MON_CON0, o_PCIe_reg_address_76);
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);
	ddr_reg_wr_otp(DPHY_DVFS_CON, o_Reserved_DDR_INIT_5);
	ddr_reg_wr_otp(DPHY_GNR_CON0, o_DPHY_GNR_CON0_0);
	ddr_reg_wr_otp(DPHY_CAL_CON0, o_DPHY_CAL_CON0_0);
	ddr_reg_wr_otp(DPHY_CAL_CON2, o_Reserved_DDR_INIT_17);
	ddr_reg_wr_otp(DPHY_LP_CON0, o_DPHY_LP_CON0_2);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_2);
	ddr_reg_wr_otp(DPHY_MDLL_CON0, o_DPHY_MDLL_CON0_1);

	ddr_reg_wr_otp(DPHY2_MON_CON0, o_PCIe_reg_address_76);
	ddr_reg_wr_otp(DPHY2_DVFS_CON, o_Reserved_DDR_INIT_5);
	ddr_reg_wr_otp(DPHY2_GNR_CON0, o_DPHY_GNR_CON0_0);
	ddr_reg_wr_otp(DPHY2_CAL_CON0, o_DPHY_CAL_CON0_0);
	ddr_reg_wr_otp(DPHY2_CAL_CON2, o_Reserved_DDR_INIT_17);
	ddr_reg_wr_otp(DPHY2_LP_CON0, o_DPHY_LP_CON0_2);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_2);
	ddr_reg_wr_otp(DPHY2_MDLL_CON0, o_DPHY_MDLL_CON0_1);
}

static int ddr_initialize_dfi(void)
{
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_1);

	if (ddr_reg_poll(DREX_PHYSTATUS, p_DREX_PHYSTATUS_dfi))
		return DDR_FAIL;

	ddr_reg_clr(DREX_CONCONTROL, 0x10008000);
	ddr_reg_wr_otp(DPHY_MDLL_CON0, o_DPHY_MDLL_CON0_1);
	ddr_reg_wr_otp(DPHY2_MDLL_CON0, o_DPHY_MDLL_CON0_1);

	return DDR_SUCCESS;
}

static void ddr_dram_reset_sequence(void)
{
	/* RESET_DRAM */
	ddr_reg_wr_otp(DREX_DFIRSTCONTROL, o_DREX_DFIRSTCONTROL_1);
	ddr_usleep(DDR_INIT_TIMING_tINIT1_USEC);
	ddr_reg_wr_otp(DREX_DFIRSTCONTROL, o_DREX_DFIRSTCONTROL_0);
	ddr_usleep(DDR_INIT_TIMING_tINIT3_USEC);
}

static void ddr_power_down_exit_sequence(void)
{
	/* ExitPD start */
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_21);
}

static void ddr_mrw_set_vref_odt_etc(void)
{
	/* LPDDR4_chip_Init */
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_17);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_23);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_30);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_0);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_4);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_10);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_27);
}


/* From OFF -> ACTIVE the ZQ_Calibration is always required.
 * From SLEEP -> ACTIVE the ZQ_Calibration is not needed.
 * From SUSPEND -> ACTIVE the ZQ_Calibration is always required.
 */
static void ddr_zq_calibration(void)
{
	/* LPDDR4_ZQ_Cal */
	ddr_reg_wr_otp(DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_2);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_11);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_12);
	ddr_reg_wr_otp(DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_0);
	ddr_reg_wr_otp(DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_1);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_11);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_12);
	ddr_reg_wr_otp(DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_0);

	/* DRAM DCTL Resync */
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_1);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
}

static int ddr_io_initialization(int from_suspend)
{
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_3);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_1);
	ddr_reg_wr_otp(DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_15);
	ddr_reg_wr_otp(DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_0);
	ddr_reg_wr_otp(DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_2);

	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_3);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_1);
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_15);
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);

	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_15);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_15);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_16);

	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_15);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_15);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_16);

	/* wait zq calibration */
	if (ddr_reg_poll(DPHY_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;

	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_14);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_13);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);

	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_14);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_13);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);

	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);

	if (!from_suspend) {
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_0);
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_30);
	}

	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_0);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_2);
	ddr_reg_wr_otp(DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_0);
	ddr_reg_wr_otp(DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_3);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_0);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_1);
	ddr_reg_wr_otp(DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_0);
	ddr_reg_wr_otp(DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_1);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_10);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_11);

	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_2);
	ddr_reg_wr_otp(DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_3);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_1);
	ddr_reg_wr_otp(DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_1);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_10);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_11);

	/* wait zq calibration */
	if (ddr_reg_poll(DPHY_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;

	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_9);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_9);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);

	ddr_reg_wr_otp(DPHY_ZQ_CON9, o_DPHY_ZQ_CON9_1);
	ddr_reg_wr_otp(DPHY_ZQ_CON9, o_DPHY_ZQ_CON9_0);
	ddr_reg_wr_otp(DPHY_ZQ_CON9, o_DPHY_ZQ_CON9_2);
	ddr_reg_wr_otp(DPHY_ZQ_CON9, o_DPHY_ZQ_CON9_2);

	ddr_reg_wr_otp(DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_1);
	ddr_reg_wr_otp(DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_2);
	ddr_reg_wr_otp(DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_2);

	if (!from_suspend) {
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_1);
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_2);
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_7);
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_13);
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_5);
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_3);
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_8);
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_14);
	}

	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_10);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_11);

	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_10);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_11);

	/* wait zq calibration */
	if (ddr_reg_poll(DPHY_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;

	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_9);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_9);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);

	ddr_reg_wr_otp(DPHY_ZQ_CON9, o_DPHY_ZQ_CON9_2);
	ddr_reg_wr_otp(DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_2);

	return DDR_SUCCESS;
}

static int ddr_enable_dll(void)
{
	ddr_reg_wr_otp(DPHY_GNR_CON0, o_DPHY_GNR_CON0_0);
	ddr_reg_wr_otp(DPHY2_GNR_CON0, o_DPHY_GNR_CON0_0);
	ddr_reg_wr(DPHY_GATE_CON0, 0xf00ffff);
	ddr_reg_wr(DPHY2_GATE_CON0, 0xf00ffff);
	ddr_reg_wr_otp(DPHY_MDLL_CON0, o_DPHY_MDLL_CON0_2);
	ddr_reg_wr_otp(DPHY2_MDLL_CON0, o_DPHY_MDLL_CON0_2);
	ddr_reg_wr_otp(DPHY_MDLL_CON0, o_DPHY_MDLL_CON0_0);
	ddr_reg_wr_otp(DPHY2_MDLL_CON0, o_DPHY_MDLL_CON0_0);
	ddr_reg_wr_otp(DPHY_MDLL_CON0, o_DPHY_MDLL_CON0_2);
	ddr_reg_wr_otp(DPHY2_MDLL_CON0, o_DPHY_MDLL_CON0_2);

	/* Poll for ctrl_lock */
	if (ddr_reg_poll(DPHY_MDLL_CON1, p_DPHY_MDLL_CON1))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_MDLL_CON1, p_DPHY_MDLL_CON1))
		return DDR_FAIL;

	return DDR_SUCCESS;
}

static void ddr_set_drex_timing_parameters(void)
{
	/* SET_DREX */
	ddr_reg_wr_otp(DREX_PRECHCONFIG0, o_DREX_PRECHCONFIG0_0);
	ddr_reg_wr(DREX_PWRDNCONFIG, 0xffff00ff);
	ddr_reg_wr(DREX_TIMINGSETSW, 0x1);
	ddr_reg_wr(DREX_TIMINGRFCPB, 0x4242);
	ddr_reg_wr(DREX_TIMINGROW0, 0x836ba815);
	ddr_reg_wr(DREX_TIMINGROW1, 0x836ba815);
	ddr_reg_wr_otp(DREX_TIMINGDATA0, o_DREX_TIMINGDATA0_0);
	ddr_reg_wr_otp(DREX_TIMINGDATA1, o_DREX_TIMINGDATA1_0);
	ddr_reg_wr(DREX_TIMINGPOWER0, 0x4c880471);
	ddr_reg_wr(DREX_TIMINGPOWER1, 0x4c880471);
	ddr_reg_wr_otp(DREX_ETCTIMING, o_Reserved_DDR_INIT_7);
	ddr_reg_wr_otp(DREX_RDFETCH0, o_Reserved_DDR_INIT_8);
	ddr_reg_wr_otp(DREX_RDFETCH1, o_Reserved_DDR_INIT_9);

	/* DRAM DCTL Resync */
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_1);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
}

static int ddr_set_drex_address_parameters(void)
{
	/* DRAM Density Check */
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_25);
	ddr_reg_rd(DREX_MRSTATUS);

	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_1);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_26);
	ddr_reg_rd(DREX_MRSTATUS);

	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_29);
	ddr_reg_rd(DREX_MRSTATUS);

	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_1);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_28);
	ddr_reg_rd(DREX_MRSTATUS);

	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_1);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_27);
	ddr_reg_rd(DREX_MRSTATUS);

	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_1);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_27);
	ddr_reg_wr_otp(DPHY_GNR_CON0, o_DPHY_GNR_CON0_2);
	ddr_reg_wr_otp(DPHY2_GNR_CON0, o_DPHY_GNR_CON0_2);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_1);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);

	/* DRAM Density Setting */
	ddr_reg_wr_otp(DREX_MEMCONTROL, o_Reserved_DDR_INIT_10);
	ddr_reg_wr(DREX_ASP_MEMBASECONFIG0, 0x20009);
	ddr_reg_wr_otp(DREX_ASP_MEMCONFIG0, o_Reserved_DDR_INIT_11);
	ddr_reg_wr(DREX_ASP_CHIP0SIZECONFIG, 0x1);

	return DDR_SUCCESS;
}

static void ddr_prepare_training(void)
{
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_1);
	ddr_reg_wr_otp(DPHY_DVFS_CON, o_Reserved_DDR_INIT_5);
	ddr_reg_wr_otp(DPHY_MDLL_CON0, o_DPHY_MDLL_CON0_3);

	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_1);
	ddr_reg_wr_otp(DPHY2_DVFS_CON, o_Reserved_DDR_INIT_5);
	ddr_reg_wr_otp(DPHY2_MDLL_CON0, o_DPHY_MDLL_CON0_3);
}

static void ddr_ca_training(void)
{
	/* S/W Command Bus Training */
	ddr_reg_wr_otp(DPHY_CBT_CON0, o_Reserved_DDR_INIT_20);
	ddr_reg_wr_otp(DPHY_LP_CON0, o_DPHY_LP_CON0_2);
	ddr_reg_wr_otp(DPHY_CAL_CON0, o_DPHY_CAL_CON0_1);
	ddr_reg_wr_otp(DPHY_CAL_CON0, o_DPHY_CAL_CON0_3);
	ddr_reg_wr_otp(DPHY2_CBT_CON0, o_Reserved_DDR_INIT_20);
	ddr_reg_wr_otp(DPHY2_LP_CON0, o_DPHY_LP_CON0_2);
	ddr_reg_wr_otp(DPHY2_CAL_CON0, o_DPHY_CAL_CON0_1);
	ddr_reg_wr_otp(DPHY2_CAL_CON0, o_DPHY_CAL_CON0_3);

	/* DRAMDCTLResync:  start */
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_1);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);

	ddr_reg_wr_otp(DPHY_CAL_CON0, o_DPHY_CAL_CON0_2);
	ddr_reg_wr_otp(DPHY_CBT_CON0, o_Reserved_DDR_INIT_20);
	ddr_reg_wr_otp(DPHY_CBT_CON0, o_Reserved_DDR_INIT_20);
	ddr_reg_wr_otp(DPHY_LP_CON0, o_DPHY_LP_CON0_1);
	ddr_reg_wr_otp(DPHY2_CAL_CON0, o_DPHY_CAL_CON0_2);
	ddr_reg_wr_otp(DPHY2_CBT_CON0, o_Reserved_DDR_INIT_20);
	ddr_reg_wr_otp(DPHY2_CBT_CON0, o_Reserved_DDR_INIT_20);
	ddr_reg_wr_otp(DPHY2_LP_CON0, o_DPHY_LP_CON0_1);
}

static int ddr_odt_training(void)
{
	/* On_Die_Termination */
	ddr_reg_wr_otp(DPHY_LP_CON0, o_DPHY_LP_CON0_0);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_4);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_4);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_4);
	ddr_reg_wr_otp(DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY_CAL_CON2, o_DPHY_CAL_CON2_1);
	ddr_reg_wr_otp(DPHY_CAL_CON2, o_DPHY_CAL_CON2_0);
	ddr_reg_wr_otp(DPHY_CAL_CON2, o_DPHY_CAL_CON2_1);
	ddr_reg_wr_otp(DPHY_CAL_CON2, o_DPHY_CAL_CON2_1);
	ddr_reg_wr_otp(DPHY_CAL_CON2, o_DPHY_CAL_CON2_3);
	ddr_reg_wr_otp(DPHY_GNR_CON0, o_DPHY_GNR_CON0_1);

	ddr_reg_wr_otp(DPHY2_LP_CON0, o_DPHY_LP_CON0_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_4);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_4);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_4);
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY2_CAL_CON2, o_DPHY_CAL_CON2_1);
	ddr_reg_wr_otp(DPHY2_CAL_CON2, o_DPHY_CAL_CON2_0);
	ddr_reg_wr_otp(DPHY2_CAL_CON2, o_DPHY_CAL_CON2_1);
	ddr_reg_wr_otp(DPHY2_CAL_CON2, o_DPHY_CAL_CON2_1);
	ddr_reg_wr_otp(DPHY2_CAL_CON2, o_DPHY_CAL_CON2_3);
	ddr_reg_wr_otp(DPHY2_GNR_CON0, o_DPHY_GNR_CON0_1);

	/* ZQ_Calibration */
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_4);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_6);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_7);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_4);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_6);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_7);

	/* wait zq calibration */
	if (ddr_reg_poll(DPHY_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;

	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_5);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_4);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_5);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_4);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);

	return DDR_SUCCESS;
}

static void ddr_autodqs_clean_gate_training(void)
{
	ddr_reg_wr_otp(DPHY_CAL_CON4, o_SECURE_JTAG3);
	ddr_reg_wr_otp(DPHY_GNR_CON0, o_DPHY_GNR_CON0_2);
	ddr_reg_wr_otp(DPHY_CAL_CON2, o_DPHY_CAL_CON2_2);
	ddr_reg_wr_otp(DPHY_CAL_CON2, o_DPHY_CAL_CON2_4);
	ddr_reg_wr_otp(DPHY_CAL_CON2, o_DPHY_CAL_CON2_4);
	ddr_reg_wr_otp(DPHY_TESTIRCV_CON0, o_PCIe_reg_77);
	ddr_reg_wr_otp(DPHY_CAL_CON0, o_DPHY_CAL_CON0_2);
	ddr_reg_wr_otp(DPHY_CAL_CON3, o_Reserved_DDR_INIT_18);
	ddr_reg_wr_otp(DPHY_CAL_CON2, o_DPHY_CAL_CON2_4);

	ddr_reg_wr_otp(DPHY2_CAL_CON4, o_SECURE_JTAG3);
	ddr_reg_wr_otp(DPHY2_GNR_CON0, o_DPHY_GNR_CON0_2);
	ddr_reg_wr_otp(DPHY2_CAL_CON2, o_DPHY_CAL_CON2_2);
	ddr_reg_wr_otp(DPHY2_CAL_CON2, o_DPHY_CAL_CON2_4);
	ddr_reg_wr_otp(DPHY2_CAL_CON2, o_DPHY_CAL_CON2_4);
	ddr_reg_wr_otp(DPHY2_TESTIRCV_CON0, o_PCIe_reg_77);
	ddr_reg_wr_otp(DPHY2_CAL_CON0, o_DPHY_CAL_CON0_2);
	ddr_reg_wr_otp(DPHY2_CAL_CON3, o_Reserved_DDR_INIT_18);
	ddr_reg_wr_otp(DPHY2_CAL_CON2, o_DPHY_CAL_CON2_4);
}

static int ddr_read_dq_calibration(void)
{
	ddr_reg_wr_otp(DPHY_CAL_RD_PATTERN_CON0, o_Reserved_DDR_INIT_19);
	ddr_reg_wr_otp(DPHY2_CAL_RD_PATTERN_CON0, o_Reserved_DDR_INIT_19);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_16);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_19);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_9);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_6);
	ddr_reg_wr_otp(DPHY_CAL_CON0, o_DPHY_CAL_CON0_4);
	ddr_reg_wr_otp(DPHY2_CAL_CON0, o_DPHY_CAL_CON0_4);
	ddr_reg_wr_otp(DREX_INIT_TRAIN_CONFIG, o_DREX_INIT_TRAIN_CONFIG_0);
	ddr_reg_wr_otp(DREX_INIT_TRAIN_CONFIG, o_DREX_INIT_TRAIN_CONFIG_2);
	ddr_reg_wr_otp(DREX_INIT_TRAIN_CONTROL, o_DREX_INIT_TRAIN_CONTROL_1);

	/* poll for train complete	*/
	if (ddr_reg_poll(DREX_PHYSTATUS, p_DREX_PHYSTATUS_train))
		return DDR_FAIL;

	ddr_reg_wr_otp(DREX_INIT_TRAIN_CONTROL, o_DREX_INIT_TRAIN_CONTROL_0);

	return DDR_SUCCESS;
}

static int ddr_write_dq_calibration(void)
{
	ddr_reg_wr_otp(DPHY_CAL_WR_PATTERN_CON0, o_PCIe_reg_75);
	ddr_reg_wr_otp(DPHY_CAL_WR_PATTERN_CON1, o_PCIe_reg_address_75);
	ddr_reg_wr_otp(DPHY_CAL_WR_PATTERN_CON2, o_PCIe_reg_76);
	ddr_reg_wr_otp(DPHY_CAL_WR_PATTERN_CON3, o_DPHY_CAL_WR_PATTERN_CON4_0);
	ddr_reg_wr_otp(DPHY_CAL_WR_PATTERN_CON4, o_DPHY_CAL_WR_PATTERN_CON4_1);
	ddr_reg_wr(DREX_WRTRA_PATTERN0, 0xaa55aa55);
	ddr_reg_wr(DREX_WRTRA_PATTERN1, 0xaa55aa55);
	ddr_reg_wr(DREX_WRTRA_PATTERN2, 0x5555);
	ddr_reg_wr_otp(DPHY_CAL_CON0, o_DPHY_CAL_CON0_5);
	ddr_reg_wr_otp(DPHY2_CAL_WR_PATTERN_CON0, o_PCIe_reg_75);
	ddr_reg_wr_otp(DPHY2_CAL_WR_PATTERN_CON1, o_PCIe_reg_address_75);
	ddr_reg_wr_otp(DPHY2_CAL_WR_PATTERN_CON2, o_PCIe_reg_76);
	ddr_reg_wr_otp(DPHY2_CAL_WR_PATTERN_CON3, o_DPHY_CAL_WR_PATTERN_CON4_0);
	ddr_reg_wr_otp(DPHY2_CAL_WR_PATTERN_CON4, o_DPHY_CAL_WR_PATTERN_CON4_1);
	ddr_reg_wr(DREX_WRTRA_PATTERN0, 0xaa55aa55);
	ddr_reg_wr(DREX_WRTRA_PATTERN1, 0xaa55aa55);
	ddr_reg_wr(DREX_WRTRA_PATTERN2, 0x5555);
	ddr_reg_wr_otp(DPHY2_CAL_CON0, o_DPHY_CAL_CON0_5);
	ddr_reg_wr_otp(DREX_INIT_TRAIN_CONFIG, o_DREX_INIT_TRAIN_CONFIG_0);
	ddr_reg_wr_otp(DREX_INIT_TRAIN_CONFIG, o_DREX_INIT_TRAIN_CONFIG_3);
	ddr_reg_wr_otp(DREX_INIT_TRAIN_CONTROL, o_DREX_INIT_TRAIN_CONTROL_1);

	/* poll for train complete	*/
	if (ddr_reg_poll(DREX_PHYSTATUS, p_DREX_PHYSTATUS_train))
		return DDR_FAIL;

	ddr_reg_wr_otp(DREX_INIT_TRAIN_CONTROL, o_DREX_INIT_TRAIN_CONTROL_0);

	return DDR_SUCCESS;
}

void ddr_prbs_training_init(void)
{
	/* DDRPHY_SET_PRBS_TRAINING_INIT */
	ddr_reg_wr(DPHY_PRBS_CON0, 0x50000);
	ddr_reg_wr(DPHY2_PRBS_CON0, 0x50000);
	ddr_reg_wr_otp(DPHY_PRBS_CON1, o_PCIe_reg_address_79);
	ddr_reg_wr_otp(DPHY2_PRBS_CON1, o_PCIe_reg_address_79);
}

static int ddr_prbs_training_read(void)
{
	/* DDRPHY_RUN_PRBS_TRAINING - READ */
	ddr_reg_wr(DPHY_PRBS_CON0, 0x50002);
	ddr_reg_wr(DPHY2_PRBS_CON0, 0x50002);
	if (ddr_reg_poll(DPHY_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;

	ddr_reg_wr(DPHY_PRBS_CON0, 0x50001);
	ddr_reg_wr(DPHY2_PRBS_CON0, 0x50001);
	if (ddr_reg_poll(DPHY_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;

	/* DRAMDCTLResync:  start */
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_1);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);

	return DDR_SUCCESS;
}

static int ddr_prbs_training_write(void)
{
	/* DDRPHY_RUN_PRBS_TRAINING - WRITE */
	ddr_reg_wr(DPHY_PRBS_CON0, 0x50004);
	ddr_reg_wr(DPHY2_PRBS_CON0, 0x50004);

	if (ddr_reg_poll(DPHY_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;

	ddr_reg_wr(DPHY_PRBS_CON0, 0x50001);
	ddr_reg_wr(DPHY2_PRBS_CON0, 0x50001);

	if (ddr_reg_poll(DPHY_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;

	/* DRAMDCTLResync:  start */
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_1);
	ddr_reg_wr_otp(DREX_PHYCONTROL0, o_DREX_PHYCONTROL0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);

	return DDR_SUCCESS;
}

static void ddr_axi_enable_after_all_training(void)
{
	ddr_reg_wr_otp(DREX_MEMCONTROL, o_Reserved_DDR_INIT_13);
	ddr_reg_wr_otp(DREX_MEMCONTROL, o_Reserved_DDR_INIT_14);
	ddr_reg_wr_otp(DREX_DFIRSTCONTROL, o_Reserved_DDR_INIT_15);
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);
	ddr_reg_set(DREX_CONCONTROL, 0x1 << 5);
	ddr_reg_wr(DREX_ALL_INIT_INDI, 0x1);
	ddr_reg_wr_otp(DREX_ACTIVATE_AXI_READY, o_Reserved_DDR_INIT_16);
}

static int ddr_enter_self_refresh_mode(void)
{
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_15);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_20);

	/* poll for self refresh entry */
	if (ddr_reg_poll(DREX_CHIPSTATUS, p_DREX_CHIPSTATUS_sr_enter))
		return DDR_FAIL;

	return DDR_SUCCESS;
}

static int ddr_exit_self_refresh_mode(void)
{
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_22);

	/* poll for self refresh exit */
	if (ddr_reg_poll(DREX_CHIPSTATUS, p_DREX_CHIPSTATUS_sr_exit))
		return DDR_FAIL;

	return DDR_SUCCESS;
}

static void ddr_train_save_configuration(struct ab_ddr_context *ddr_ctx)
{
	int i;

	for (i = 0; i < s_train_max_index; i++)
		ddr_ctx->ddr_train_save_value[i] =
			ddr_reg_rd(g_ddr_train_save_regs[i]);
}

static void ddr_train_restore_configuration(uint32_t *ddr_train_save_value)
{
	int save_idx, restore_idx;

	ddr_reg_wr(DPHY_CAL_CON1, 0x112001);
	ddr_reg_wr(DPHY_CAL_CON4, 0x83004f);
	ddr_reg_wr(DPHY_CAL_CON1, 0x112000);

	ddr_reg_wr(DPHY2_CAL_CON1, 0x112001);
	ddr_reg_wr(DPHY2_CAL_CON4, 0x83004f);
	ddr_reg_wr(DPHY2_CAL_CON1, 0x112000);

	/* Auto DQS clean / Gate training */
	ddr_autodqs_clean_gate_training();

	ddr_reg_wr(DPHY_CAL_RD_PATTERN_CON0, 0x55AA55AA);
	ddr_reg_wr(DPHY2_CAL_RD_PATTERN_CON0, 0x55AA55AA);

	ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON0, 0x55aa55aa);
	ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON1, 0x55aa55aa);
	ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON2, 0x55aa55aa);
	ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON3, 0x55aa55aa);
	ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON4, 0x5555);
	ddr_reg_wr(DREX_WRTRA_PATTERN0, 0xaa55aa55);
	ddr_reg_wr(DREX_WRTRA_PATTERN1, 0xaa55aa55);
	ddr_reg_wr(DREX_WRTRA_PATTERN2, 0x5555);

	ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON0, 0x55aa55aa);
	ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON1, 0x55aa55aa);
	ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON2, 0x55aa55aa);
	ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON3, 0x55aa55aa);
	ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON4, 0x5555);
	ddr_reg_wr(DREX_WRTRA_PATTERN0, 0xaa55aa55);
	ddr_reg_wr(DREX_WRTRA_PATTERN1, 0xaa55aa55);
	ddr_reg_wr(DREX_WRTRA_PATTERN2, 0x5555);

	/* Restore Read Training Results */
	ddr_reg_wr(DPHY_CAL_CON0, 0x780806c8);
	ddr_reg_wr(DPHY_CAL_CON3, 0xfc7f9900);
	ddr_reg_wr(DPHY2_CAL_CON0, 0x780806c8);
	ddr_reg_wr(DPHY2_CAL_CON3, 0xfc7f9900);

	restore_idx = 0;
	for (save_idx = s_DPHY_RD_DESKEW_CENTER_CS0_CON_DM;
		save_idx <= s_DPHY2_RD_DQS_VWML_CS1_CON0; save_idx++) {
		ddr_reg_wr(g_ddr_train_restore_read_regs[restore_idx],
				ddr_train_save_value[save_idx]);
		restore_idx++;
	}

	ddr_reg_wr(DPHY_CAL_CON3, 0xfc7f9800);
	ddr_reg_wr(DPHY2_CAL_CON3, 0xfc7f9800);

	/* Restore Write Training Results */
	ddr_reg_wr(DPHY_CAL_CON0, 0x780806e8);
	ddr_reg_wr(DPHY_CAL_CON3, 0xfc7f9a00);
	ddr_reg_wr(DPHY2_CAL_CON0, 0x780806e8);
	ddr_reg_wr(DPHY2_CAL_CON3, 0xfc7f9a00);

	restore_idx = 0;
	for (save_idx = s_DPHY_WR_DESKEWC_CS0_CON0;
		save_idx <= s_DPHY2_DM_DESKEWL_CS1_CON0; save_idx++) {
		ddr_reg_wr(g_ddr_train_restore_write_regs[restore_idx],
				ddr_train_save_value[save_idx]);
		restore_idx++;
	}

	ddr_reg_wr(DPHY_CAL_CON3, 0xfc7f9800);
	ddr_reg_wr(DPHY2_CAL_CON3, 0xfc7f9800);

	/* Restore PRBS training result */
	ddr_reg_wr(DPHY_CAL_CON3, 0xfc7f9840);
	ddr_reg_wr(DPHY_PRBS_CON4, ddr_train_save_value[s_DPHY_PRBS_CON2]);
	ddr_reg_wr(DPHY_PRBS_CON5, ddr_train_save_value[s_DPHY_PRBS_CON3]);
	ddr_reg_wr(DPHY_CAL_CON3, 0xfc7f9800);

	ddr_reg_wr(DPHY2_CAL_CON3, 0xfc7f9840);
	ddr_reg_wr(DPHY2_PRBS_CON4, ddr_train_save_value[s_DPHY2_PRBS_CON2]);
	ddr_reg_wr(DPHY2_PRBS_CON5, ddr_train_save_value[s_DPHY2_PRBS_CON3]);
	ddr_reg_wr(DPHY2_CAL_CON3, 0xfc7f9800);

	/* Restore ctrl_lock_value_init after restoring training result */
	ddr_reg_wr(DPHY_MDLL_CON1, ddr_train_save_value[s_DPHY_MDLL_CON1]);
	ddr_reg_wr(DPHY2_MDLL_CON1, ddr_train_save_value[s_DPHY2_MDLL_CON1]);

	ddr_reg_wr(DPHY_MDLL_CON1, 0x7c6310);
	ddr_reg_wr(DPHY2_MDLL_CON1, 0x7c6310);
	ddr_reg_wr(DPHY_MDLL_CON1, 0x5c6310);
	ddr_reg_wr(DPHY2_MDLL_CON1, 0x5c6310);

#ifndef CONFIG_DDR_VREF_DISABLE
	ddr_reg_wr(DPHY_ZQ_CON9, ddr_train_save_value[s_DPHY_ZQ_CON9]);
	ddr_reg_wr(DPHY2_ZQ_CON9, ddr_train_save_value[s_DPHY2_ZQ_CON9]);
#endif
}

#define DDR_BOOT_TEST_READ (0x1 << 0)
#define DDR_BOOT_TEST_WRITE (0x1 << 1)
#define DDR_BOOT_TEST_READ_WRITE (DDR_BOOT_TEST_READ | DDR_BOOT_TEST_WRITE)
#define DMA_SZ (16 * 1024 * 1024)

static DECLARE_COMPLETION(dma_completion);

static int dma_callback(uint8_t chan, enum dma_data_direction dir,
			enum abc_dma_trans_status status)
{
	complete(&dma_completion);

	return 0;
}

/* TODO(b/118829034): Move to a separate test file */
void ab_ddr_read_write_test(int read_write)
{
	size_t size = DMA_SZ;
	char *host_vaddr;
	dma_addr_t host_paddr, ab_paddr;
	struct dma_element_t desc;
	int i, j;

	host_vaddr = abc_alloc_coherent(size, &host_paddr);
	if (!host_vaddr) {
		pr_err("%s: buffer allocation failed\n", __func__);
		return;
	}

	for (i = 0; i < DMA_SZ; i++)
		host_vaddr[i] = i & 0xff;

	(void)abc_reg_dma_irq_callback(&dma_callback, 0);

	desc.len = size;
	desc.chan = 0;

	if (DDR_BOOT_TEST_WRITE & read_write) {
		desc.src_addr = (uint32_t)(host_paddr & 0xFFFFFFFF);
		desc.src_u_addr = (uint32_t)(host_paddr >> 32);

		ab_paddr = 0x20000000;

		for (j = 0; j < 32; j++) {
			desc.dst_addr = (uint32_t)(ab_paddr & 0xFFFFFFFF);
			desc.dst_u_addr = (uint32_t)(ab_paddr >> 32);

			reinit_completion(&dma_completion);

			(void)dma_sblk_start(0, DMA_TO_DEVICE, &desc);

			wait_for_completion(&dma_completion);

			ab_paddr = ab_paddr + DMA_SZ;
		}
	}

	if (DDR_BOOT_TEST_READ & read_write) {
		desc.dst_addr = (uint32_t)(host_paddr & 0xFFFFFFFF);
		desc.dst_u_addr = (uint32_t)(host_paddr >> 32);

		ab_paddr = 0x20000000;

		for (j = 0; j < 32; j++) {
			desc.src_addr = (uint32_t)(ab_paddr & 0xFFFFFFFF);
			desc.src_u_addr = (uint32_t)(ab_paddr >> 32);

			reinit_completion(&dma_completion);

			(void)dma_sblk_start(0, DMA_FROM_DEVICE, &desc);

			wait_for_completion(&dma_completion);

			for (i = 0; i < DMA_SZ; i++)
				if (host_vaddr[i] != (i & 0xff))
					pr_err("%s: mismatch for byte %d\n",
							__func__, i);

			ab_paddr = ab_paddr + DMA_SZ;
		}
	}

	(void)abc_reg_dma_irq_callback(NULL, 0);

	abc_free_coherent(size, host_vaddr, host_paddr);
}

static int ab_ddr_train(struct ab_ddr_context *ddr_ctx, uint32_t DDR_SR)
{
	/* Check ddr_reg_rd(TRN_ADDR) value and if non zero, the previous
	 * training results are stored in here. Restore the Training parameters.
	 */
	if (GET_REG_TRN_ADDR()) {

		/* Self-refresh entry sequence */
		if (ddr_enter_self_refresh_mode())
			goto ddr_train_fail;

		/* Set upd_mode = 0. Taken care in other sequence */

		/* Copy training results from TRN_ADDR to PHY/DRAM */
		ddr_train_restore_configuration((uint32_t *)(uint64_t)
							GET_REG_TRN_ADDR());

		/* Self-refresh exit sequence */
		if (ddr_exit_self_refresh_mode())
			goto ddr_train_fail;

		ddr_axi_enable_after_all_training();

	} else {

		if (!DDR_SR) {
			/* Self-refresh entry sequence */
			if (ddr_enter_self_refresh_mode())
				goto ddr_train_fail;
		}

		/* configure PLL_MIF via CMU_MIF_for_Highfreq and wait for
		 * pll lock
		 */
		if (ddr_config_cmu_mif_highfreq())
			goto ddr_train_fail;

		/* for CKE High to CS delay */
		ddr_usleep(DDR_INIT_CKE2CS_DELAY_USEC);

		/* Enable DLL */
		if (ddr_enable_dll())
			goto ddr_train_fail;

		/* Power-down exit sequence */
		ddr_power_down_exit_sequence();

		ddr_set_drex_timing_parameters();

		if (ddr_set_drex_address_parameters())
			goto ddr_train_fail;

		/* CA Training */
		ddr_prepare_training();

		ddr_ca_training();

		/* ODT Training */
		if (ddr_odt_training())
			goto ddr_train_fail;

		/* Auto DQS clean / Gate training */
		ddr_autodqs_clean_gate_training();

		/* Calibrate / Level DQ for Read */
		if (ddr_read_dq_calibration())
			goto ddr_train_fail;

		/* Calibrate / Level DQ for Write */
		if (ddr_write_dq_calibration())
			goto ddr_train_fail;

		/* Reset the prbs_dram_act_enable */
		ddr_reg_clr(DPHY_PRBS_CON8, 1 << 31);
		ddr_reg_clr(DPHY2_PRBS_CON8, 1 << 31);

		/* PRBS training init */
		ddr_prbs_training_init();

		/* PRBS training - Read */
		if (ddr_prbs_training_read())
			goto ddr_train_fail;

		/* PRBS training - Write */
		if (ddr_prbs_training_write())
			goto ddr_train_fail;

#ifndef CONFIG_DDR_VREF_DISABLE
		if (!DDR_SR) {
#ifdef CONFIG_ZEBU_EMULATION
			/* PRBS vref training for both DPHY0/1 */
			ddrphy_run_vref_training(ddr_ctx);
#else
			/* PRBS vref training for both DPHY0/1 */
			if (ddrphy_run_vref_training(ddr_ctx))
				goto ddr_train_fail;
#endif
			/* CA Training */
			ddr_prepare_training();

			ddr_ca_training();

			/* ODT Training */
			if (ddr_odt_training())
				goto ddr_train_fail;

			/* Auto DQS clean / Gate training */
			ddr_autodqs_clean_gate_training();

			/* Calibrate / Level DQ for Read */
			if (ddr_read_dq_calibration())
				goto ddr_train_fail;

			/* Calibrate / Level DQ for Write */
			if (ddr_write_dq_calibration())
				goto ddr_train_fail;

			/* PRBS training init */
			ddr_prbs_training_init();

			/* PRBS training - Read */
			if (ddr_prbs_training_read())
				goto ddr_train_fail;

			/* PRBS training - Write */
			if (ddr_prbs_training_write())
				goto ddr_train_fail;
		}
#endif /* CONFIG_DDR_VREF_DISABLE */

		/* Self-refresh exit sequence */
		if (ddr_exit_self_refresh_mode())
			goto ddr_train_fail;

		/* Set the prbs_dram_act_enable */
		ddr_reg_set(DPHY_PRBS_CON8, (1 << 31));
		ddr_reg_set(DPHY2_PRBS_CON8, (1 << 31));

		ddr_axi_enable_after_all_training();

		/* Save training results to local array */
		ddr_train_save_configuration(ddr_ctx);
	}

	return DDR_SUCCESS;

ddr_train_fail:
	return DDR_FAIL;
}

static int32_t ab_ddr_init_internal_isolation(struct ab_ddr_context *ddr_ctx)
{
	uint32_t DDR_SR;

	/* Read the DDR_SR */
	DDR_SR = GPIO_DDR_SR();

	/* deassert PHY reset while PHY clock is gated */
	ddr_deassert_phy_reset_while_phy_is_gated();

	/* configure PLL_MIF via CMU_MIF_for_Lowfreq and wait for pll lock */
	if (ddr_config_cmu_mif_lowfreq())
		goto ddr_init_fail;

	/* Ungate PHY Clock */
	ddr_ungate_phy_clock();

	/* Deassert Internal Isolation
	 * SYSREG_Central_PMU ::
	 * PMU_CONTROL[0x4] :: PHY_RET_ON[7:7] = 0
	 */
	PMU_CONTROL_PHY_RET_OFF();

	/* Initialize PHY */
	ddr_initialize_phy_pre();

	ddr_initialize_phy();

	/* Initialize DFI Interface */
	if (ddr_initialize_dfi())
		goto ddr_init_fail;

	if (DDR_SR) { /* SUSPEND to ACTIVE sequence */

		/* IO Initialization for suspend */
		if (ddr_io_initialization(1))
			goto ddr_init_fail;

	} else { /* OFF to ACTIVE sequence */

		/* DRAM reset sequence */
		ddr_dram_reset_sequence();

		/* Power-down exit sequence */
		ddr_power_down_exit_sequence();

		/* IO Initialization */
		if (ddr_io_initialization(0))
			goto ddr_init_fail;

		/* MRWs (Set VREF, ODT, etc) */
		ddr_mrw_set_vref_odt_etc();

		/* DRAM ZQ Calibration */
		ddr_zq_calibration();
	}

	/* DDR training */
	if (ab_ddr_train(ddr_ctx, DDR_SR))
		goto ddr_init_fail;

	/* Now M0 can enter WFI for GPIO_DDR_TRAIN or REG_DDR_TRAIN */
	return DDR_SUCCESS;

ddr_init_fail:
	/* [TODO] Anything need to be done in case of failure ?? */
	return DDR_FAIL;
}

/* Perform ddr re-training or train result restore */
void ab_ddr_train_gpio(struct ab_state_context *sc)
{
	uint32_t DDR_SR;
	struct ab_ddr_context *ddr_ctx;

	if (!sc || !sc->ddr_data) {
		pr_err("%s, error: ab_ddr_setup() is not called", __func__);
		return;
	}

	ddr_ctx = (struct ab_ddr_context *)sc->ddr_data;

	/* Read the DDR_SR */
	DDR_SR = GPIO_DDR_SR();

	while (GPIO_CKE_IN_SENSE())
		;

	/* self-refresh exit sequence */
	ddr_exit_self_refresh_mode();

	ab_ddr_train(ddr_ctx, DDR_SR);
}

/* Perform ddr re-training */
void ab_ddr_train_sysreg(struct ab_state_context *sc)
{
	uint32_t DDR_SR;
	struct ab_ddr_context *ddr_ctx;

	if (!sc || !sc->ddr_data) {
		pr_err("%s, error: ab_ddr_setup() is not called", __func__);
		return;
	}

	ddr_ctx = (struct ab_ddr_context *)sc->ddr_data;

	/* Read the DDR_SR */
	DDR_SR = GPIO_DDR_SR();

	SET_REG_TRN_ADDR(0);

	ab_ddr_train(ddr_ctx, DDR_SR);
}

int32_t ab_ddr_resume(struct ab_state_context *sc)
{
	/* TBD: HACK
	 * Remove this after waiting for the DDR Initialization interrupts
	 */
	msleep(200);

	if (!IS_HOST_DDR_INIT()) {
		/* TBD: The below code is not part of ABC A0 BootROM.
		 * For B0 BootROM, we can remove this code.
		 */
		/* Self-refresh exit sequence */
		if (ddr_exit_self_refresh_mode()) {
			pr_err("ERROR!!! Exit from Self-Refresh is fail\n");
			return DDR_FAIL;
		}

		ddr_axi_enable_after_all_training();
	}

	/* Disable the DDR_SR GPIO */
	ab_gpio_disable_ddr_sr(sc);

#ifdef CONFIG_DDR_BOOT_TEST
	ab_ddr_read_write_test(DDR_BOOT_TEST_READ);
#endif
	return DDR_SUCCESS;
}
EXPORT_SYMBOL(ab_ddr_resume);

int32_t ab_ddr_suspend(struct ab_state_context *sc)
{
#ifdef CONFIG_DDR_BOOT_TEST
	ab_ddr_read_write_test(DDR_BOOT_TEST_WRITE);
#endif
	/* Block AXI Before entering self-refresh */
	ddr_reg_wr(DREX_ACTIVATE_AXI_READY, 0x0);

	/* Self-refresh entry sequence */
	if (ddr_enter_self_refresh_mode())
		goto ddr_suspend_fail;

	/* Enable the PMU Retention */
	PMU_CONTROL_PHY_RET_ON();

	/* Enable GPIOs to inform DDR is in suspend mode */
	ab_gpio_enable_ddr_sr(sc);
	ab_gpio_enable_ddr_iso(sc);

	return DDR_SUCCESS;

ddr_suspend_fail:
	return DDR_FAIL;
}
EXPORT_SYMBOL(ab_ddr_suspend);

int32_t ab_ddr_selfrefresh_exit(struct ab_state_context *sc)
{
	/* Self-refresh exit sequence */
	if (ddr_exit_self_refresh_mode())
		return DDR_FAIL;

	/* Allow AXI after exiting from self-refresh */
	ddr_reg_wr(DREX_ACTIVATE_AXI_READY, 0x1);

#ifdef CONFIG_DDR_BOOT_TEST
	ab_ddr_read_write_test(DDR_BOOT_TEST_READ);
#endif
	return DDR_SUCCESS;
}
EXPORT_SYMBOL(ab_ddr_selfrefresh_exit);

int32_t ab_ddr_selfrefresh_enter(struct ab_state_context *sc)
{
#ifdef CONFIG_DDR_BOOT_TEST
	ab_ddr_read_write_test(DDR_BOOT_TEST_WRITE);
#endif
	/* Block AXI Before entering self-refresh */
	ddr_reg_wr(DREX_ACTIVATE_AXI_READY, 0x0);

	/* Self-refresh entry sequence */
	if (ddr_enter_self_refresh_mode())
		return DDR_FAIL;

	return DDR_SUCCESS;
}
EXPORT_SYMBOL(ab_ddr_selfrefresh_enter);

static int ab_ddr_set_state(const struct block_property *prop_from,
			const struct block_property *prop_to,
			enum chip_state chip_state_id, void *data)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;
	struct ab_ddr_context *ddr_ctx;

	if (!sc || !prop_from || !prop_to)
		return -EINVAL;

	if (!sc->ddr_data) {
		pr_err("%s, error: ab_ddr_setup() is not called", __func__);
		return -EINVAL;
	}

	ddr_ctx = (struct ab_ddr_context *)sc->ddr_data;

	switch (chip_state_id) {
	case CHIP_STATE_0_0 ... CHIP_STATE_2_6:
		if (ddr_ctx->ddr_state == DDR_SLEEP)
			ab_ddr_selfrefresh_exit(sc);
		else if (ddr_ctx->ddr_state == DDR_SUSPEND)
			ab_ddr_resume(sc);
		ddr_ctx->ddr_state = DDR_ON;
		break;

	case CHIP_STATE_3_0 ... CHIP_STATE_4_0:
		/* ddr sleep/deep-sleep functionality */
		if (ddr_ctx->ddr_state != DDR_ON)
			return -EINVAL;

		ab_ddr_selfrefresh_enter(sc);

		ddr_ctx->ddr_state = DDR_SLEEP;
		break;

	case CHIP_STATE_5_0:
		/* ddr suspend functionality */
		if (ddr_ctx->ddr_state == DDR_SUSPEND)
			return -EINVAL;
		ab_ddr_suspend(sc);

		ddr_ctx->ddr_state = DDR_SUSPEND;
		break;

	case CHIP_STATE_6_0:
		if (ddr_ctx->ddr_state == DDR_SUSPEND) {
			ab_gpio_disable_ddr_sr(sc);
			ab_gpio_disable_ddr_iso(sc);
		}

		ddr_ctx->ddr_state = DDR_OFF;
		break;

	default:
		break;
	}

	/* Based on the state, call the corresponding DDR functionality */
	return 0;
}

int32_t ab_ddr_setup(struct ab_state_context *sc)
{
	int data;
	struct platform_device *pdev;
	struct ab_ddr_context *ddr_ctx;

	if (!sc || !sc->pdev)
		return -EFAULT;

	/* ab_ddr_setup is already called */
	if (sc->ddr_data)
		return DDR_SUCCESS;

	ddr_ctx = kzalloc(sizeof(struct ab_ddr_context), GFP_KERNEL);
	if (ddr_ctx == NULL)
		return -ENOMEM;

	pdev = sc->pdev;

	/* If DDR OTPs are not fused, use the OTP array instead */
	if (of_property_read_u32(pdev->dev.of_node, "ddr-otp-flashed", &data)) {
		dev_info(sc->dev, "%s: DDR OTPs NOT fused. Use local array.\n",
			 __func__);
		ddr_otp_rd = &read_otp_array;
	} else {
		dev_info(sc->dev, "%s: DDR OTPs fused\n", __func__);
		ddr_otp_rd = &read_otp_wrapper;
	}

#ifdef DEBUG
	for (data = 0; data < o_DDR_OTP_MAX; data++)
		dev_dbg(sc->dev, "%d: 0x%x\n", data, (data));
#endif

	/* In A0 BootROM Auto Refresh setting is missing. To fix this issue,
	 * set the Auto-Refresh enable during the ddr setup.
	 */
	if (!IS_HOST_DDR_INIT())
		ddr_reg_set(DREX_CONCONTROL, 0x1 << 5);

	/* Register the Airbrush State Manager (ASM) callback */
	ab_sm_register_blk_callback(DRAM, &ab_ddr_set_state, sc);

	/* ddr_ctx can be accessed outside this file with the help of
	 * ab_state_context
	 */
	ddr_ctx->ab_state_ctx = sc;
	sc->ddr_data = ddr_ctx;

	return DDR_SUCCESS;
}

int32_t ab_ddr_init(struct ab_state_context *sc)
{
	int32_t ret;
	struct ab_ddr_context *ddr_ctx;
#ifdef CONFIG_DDR_BOOT_TEST
	uint32_t DDR_SR;
#endif

	if (!sc || !sc->ddr_data) {
		pr_err("%s, error: ddr setup is not called", __func__);
		return DDR_FAIL;
	}

	ddr_ctx = (struct ab_ddr_context *)sc->ddr_data;
	ddr_ctx->ddr_state = DDR_ON;

	ret = ab_ddr_init_internal_isolation(ddr_ctx);

#ifdef CONFIG_DDR_BOOT_TEST
	/* Read the DDR_SR */
	DDR_SR = GPIO_DDR_SR();

	if (!DDR_SR && !ret)
		ab_ddr_read_write_test(DDR_BOOT_TEST_READ_WRITE);
#endif
	return ret;
}
EXPORT_SYMBOL(ab_ddr_init);
