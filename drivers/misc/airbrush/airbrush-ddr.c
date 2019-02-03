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
#include <linux/airbrush-sm-notifier.h>
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

static const struct ddr_train_save_restore_t *
			get_train_save_restore_regs(unsigned int idx)
{
	static const struct ddr_train_save_restore_t
				train_save_restore_regs[] = {
		[s_DPHY_MDLL_CON1] = {
			DPHY_MDLL_CON1,
			DPHY_MDLL_CON1 },
		[s_DPHY_CA_DESKEW_CON0] = {
			DPHY_CA_DESKEW_CON0,
			DPHY_CA_DESKEW_CON0 },
		[s_DPHY_CA_DESKEW_CON1] = {
			DPHY_CA_DESKEW_CON1,
			DPHY_CA_DESKEW_CON1 },
		[s_DPHY_CA_DESKEW_CON2] = {
			DPHY_CA_DESKEW_CON2,
			DPHY_CA_DESKEW_CON2 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON_DM] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON_DM,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON_DM },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON0] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON0,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON0 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON1] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON1,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON1 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON2] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON2,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON2 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON3] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON3,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON3 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON4] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON4,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON4 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON5] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON5,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON5 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON6] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON6,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON6 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON7] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON7,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON7 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON_DM] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON_DM,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON_DM },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON0] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON0,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON0 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON1] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON1,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON1 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON2] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON2,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON2 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON3] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON3,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON3 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON4] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON4,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON4 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON5] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON5,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON5 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON6] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON6,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON6 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON7] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON7,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON7 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON_DM] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON_DM,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON_DM },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON0] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON0,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON0 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON1] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON1,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON1 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON2] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON2,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON2 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON3] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON3,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON3 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON4] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON4,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON4 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON5] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON5,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON5 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON6] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON6,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON6 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON7] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON7,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON7 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON_DM] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON_DM,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON_DM },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON0] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON0,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON0 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON1] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON1,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON1 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON2] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON2,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON2 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON3] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON3,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON3 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON4] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON4,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON4 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON5] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON5,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON5 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON6] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON6,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON6 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON7] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON7,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON7 },
		[s_DPHY_RD_DQS_VWMC_CS0_CON0] = {
			DPHY_RD_DQS_VWMC_CS0_CON0,
			DPHY_SW_RD_DQS_VWMC_CS0_CON0 },
		[s_DPHY_RD_DQS_VWMC_CS1_CON0] = {
			DPHY_RD_DQS_VWMC_CS1_CON0,
			DPHY_SW_RD_DQS_VWMC_CS1_CON0 },
		[s_DPHY_RD_DQS_VWML_CS0_CON0] = {
			DPHY_RD_DQS_VWML_CS0_CON0,
			DPHY_SW_RD_DQS_VWML_CS0_CON0 },
		[s_DPHY_RD_DQS_VWML_CS1_CON0] = {
			DPHY_RD_DQS_VWML_CS1_CON0,
			DPHY_SW_RD_DQS_VWML_CS1_CON0 },
		[s_DPHY_WR_DESKEWC_CS0_CON0] = {
			DPHY_WR_DESKEWC_CS0_CON0,
			DPHY_SW_WR_DESKEWC_CS0_CON0 },
		[s_DPHY_WR_DESKEWC_CS0_CON1] = {
			DPHY_WR_DESKEWC_CS0_CON1,
			DPHY_SW_WR_DESKEWC_CS0_CON1 },
		[s_DPHY_WR_DESKEWC_CS0_CON2] = {
			DPHY_WR_DESKEWC_CS0_CON2,
			DPHY_SW_WR_DESKEWC_CS0_CON2 },
		[s_DPHY_WR_DESKEWC_CS0_CON3] = {
			DPHY_WR_DESKEWC_CS0_CON3,
			DPHY_SW_WR_DESKEWC_CS0_CON3 },
		[s_DPHY_WR_DESKEWC_CS0_CON4] = {
			DPHY_WR_DESKEWC_CS0_CON4,
			DPHY_SW_WR_DESKEWC_CS0_CON4 },
		[s_DPHY_WR_DESKEWC_CS0_CON5] = {
			DPHY_WR_DESKEWC_CS0_CON5,
			DPHY_SW_WR_DESKEWC_CS0_CON5 },
		[s_DPHY_WR_DESKEWC_CS0_CON6] = {
			DPHY_WR_DESKEWC_CS0_CON6,
			DPHY_SW_WR_DESKEWC_CS0_CON6 },
		[s_DPHY_WR_DESKEWC_CS0_CON7] = {
			DPHY_WR_DESKEWC_CS0_CON7,
			DPHY_SW_WR_DESKEWC_CS0_CON7 },
		[s_DPHY_DM_DESKEWC_CS0_CON0] = {
			DPHY_DM_DESKEWC_CS0_CON0,
			DPHY_SW_DM_DESKEWC_CS0_CON0 },
		[s_DPHY_WR_DESKEWC_CS1_CON0] = {
			DPHY_WR_DESKEWC_CS1_CON0,
			DPHY_SW_WR_DESKEWC_CS1_CON0 },
		[s_DPHY_WR_DESKEWC_CS1_CON1] = {
			DPHY_WR_DESKEWC_CS1_CON1,
			DPHY_SW_WR_DESKEWC_CS1_CON1 },
		[s_DPHY_WR_DESKEWC_CS1_CON2] = {
			DPHY_WR_DESKEWC_CS1_CON2,
			DPHY_SW_WR_DESKEWC_CS1_CON2 },
		[s_DPHY_WR_DESKEWC_CS1_CON3] = {
			DPHY_WR_DESKEWC_CS1_CON3,
			DPHY_SW_WR_DESKEWC_CS1_CON3 },
		[s_DPHY_WR_DESKEWC_CS1_CON4] = {
			DPHY_WR_DESKEWC_CS1_CON4,
			DPHY_SW_WR_DESKEWC_CS1_CON4 },
		[s_DPHY_WR_DESKEWC_CS1_CON5] = {
			DPHY_WR_DESKEWC_CS1_CON5,
			DPHY_SW_WR_DESKEWC_CS1_CON5 },
		[s_DPHY_WR_DESKEWC_CS1_CON6] = {
			DPHY_WR_DESKEWC_CS1_CON6,
			DPHY_SW_WR_DESKEWC_CS1_CON6 },
		[s_DPHY_WR_DESKEWC_CS1_CON7] = {
			DPHY_WR_DESKEWC_CS1_CON7,
			DPHY_SW_WR_DESKEWC_CS1_CON7 },
		[s_DPHY_DM_DESKEWC_CS1_CON0] = {
			DPHY_DM_DESKEWC_CS1_CON0,
			DPHY_SW_DM_DESKEWC_CS1_CON0 },
		[s_DPHY_WR_DESKEWL_CS0_CON0] = {
			DPHY_WR_DESKEWL_CS0_CON0,
			DPHY_SW_WR_DESKEWL_CS0_CON0 },
		[s_DPHY_WR_DESKEWL_CS0_CON1] = {
			DPHY_WR_DESKEWL_CS0_CON1,
			DPHY_SW_WR_DESKEWL_CS0_CON1 },
		[s_DPHY_WR_DESKEWL_CS0_CON2] = {
			DPHY_WR_DESKEWL_CS0_CON2,
			DPHY_SW_WR_DESKEWL_CS0_CON2 },
		[s_DPHY_WR_DESKEWL_CS0_CON3] = {
			DPHY_WR_DESKEWL_CS0_CON3,
			DPHY_SW_WR_DESKEWL_CS0_CON3 },
		[s_DPHY_WR_DESKEWL_CS0_CON4] = {
			DPHY_WR_DESKEWL_CS0_CON4,
			DPHY_SW_WR_DESKEWL_CS0_CON4 },
		[s_DPHY_WR_DESKEWL_CS0_CON5] = {
			DPHY_WR_DESKEWL_CS0_CON5,
			DPHY_SW_WR_DESKEWL_CS0_CON5 },
		[s_DPHY_WR_DESKEWL_CS0_CON6] = {
			DPHY_WR_DESKEWL_CS0_CON6,
			DPHY_SW_WR_DESKEWL_CS0_CON6 },
		[s_DPHY_WR_DESKEWL_CS0_CON7] = {
			DPHY_WR_DESKEWL_CS0_CON7,
			DPHY_SW_WR_DESKEWL_CS0_CON7 },
		[s_DPHY_DM_DESKEWL_CS0_CON0] = {
			DPHY_DM_DESKEWL_CS0_CON0,
			DPHY_SW_DM_DESKEWL_CS0_CON0 },
		[s_DPHY_WR_DESKEWL_CS1_CON0] = {
			DPHY_WR_DESKEWL_CS1_CON0,
			DPHY_SW_WR_DESKEWL_CS1_CON0 },
		[s_DPHY_WR_DESKEWL_CS1_CON1] = {
			DPHY_WR_DESKEWL_CS1_CON1,
			DPHY_SW_WR_DESKEWL_CS1_CON1 },
		[s_DPHY_WR_DESKEWL_CS1_CON2] = {
			DPHY_WR_DESKEWL_CS1_CON2,
			DPHY_SW_WR_DESKEWL_CS1_CON2 },
		[s_DPHY_WR_DESKEWL_CS1_CON3] = {
			DPHY_WR_DESKEWL_CS1_CON3,
			DPHY_SW_WR_DESKEWL_CS1_CON3 },
		[s_DPHY_WR_DESKEWL_CS1_CON4] = {
			DPHY_WR_DESKEWL_CS1_CON4,
			DPHY_SW_WR_DESKEWL_CS1_CON4 },
		[s_DPHY_WR_DESKEWL_CS1_CON5] = {
			DPHY_WR_DESKEWL_CS1_CON5,
			DPHY_SW_WR_DESKEWL_CS1_CON5 },
		[s_DPHY_WR_DESKEWL_CS1_CON6] = {
			DPHY_WR_DESKEWL_CS1_CON6,
			DPHY_SW_WR_DESKEWL_CS1_CON6 },
		[s_DPHY_WR_DESKEWL_CS1_CON7] = {
			DPHY_WR_DESKEWL_CS1_CON7,
			DPHY_SW_WR_DESKEWL_CS1_CON7 },
		[s_DPHY_DM_DESKEWL_CS1_CON0] = {
			DPHY_DM_DESKEWL_CS1_CON0,
			DPHY_SW_DM_DESKEWL_CS1_CON0 },
		[s_DPHY_PRBS_CON2] = {
			DPHY_PRBS_CON2,
			DPHY_PRBS_CON4 },
		[s_DPHY_PRBS_CON3] = {
			DPHY_PRBS_CON3,
			DPHY_PRBS_CON5 },
		[s_DPHY_ZQ_CON9] = {
			DPHY_ZQ_CON9,
			DPHY_ZQ_CON9 },

		[s_DPHY2_MDLL_CON1] = {
			DPHY2_MDLL_CON1,
			DPHY2_MDLL_CON1 },
		[s_DPHY2_CA_DESKEW_CON0] = {
			DPHY2_CA_DESKEW_CON0,
			DPHY2_CA_DESKEW_CON0 },
		[s_DPHY2_CA_DESKEW_CON1] = {
			DPHY2_CA_DESKEW_CON1,
			DPHY2_CA_DESKEW_CON1 },
		[s_DPHY2_CA_DESKEW_CON2] = {
			DPHY2_CA_DESKEW_CON2,
			DPHY2_CA_DESKEW_CON2 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON_DM] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON_DM,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON_DM },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON0] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON0,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON0 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON1] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON1,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON1 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON2] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON2,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON2 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON3] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON3,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON3 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON4] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON4,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON4 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON5] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON5,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON5 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON6] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON6,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON6 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON7] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON7,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON7 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON_DM] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON_DM,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON_DM },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON0] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON0,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON0 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON1] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON1,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON1 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON2] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON2,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON2 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON3] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON3,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON3 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON4] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON4,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON4 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON5] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON5,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON5 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON6] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON6,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON6 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON7] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON7,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON7 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON_DM] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON_DM,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON_DM },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON0] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON0,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON0 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON1] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON1,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON1 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON2] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON2,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON2 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON3] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON3,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON3 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON4] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON4,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON4 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON5] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON5,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON5 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON6] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON6,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON6 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON7] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON7,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON7 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON_DM] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON_DM,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON_DM },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON0] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON0,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON0 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON1] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON1,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON1 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON2] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON2,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON2 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON3] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON3,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON3 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON4] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON4,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON4 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON5] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON5,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON5 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON6] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON6,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON6 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON7] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON7,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON7 },
		[s_DPHY2_RD_DQS_VWMC_CS0_CON0] = {
			DPHY2_RD_DQS_VWMC_CS0_CON0,
			DPHY2_SW_RD_DQS_VWMC_CS0_CON0 },
		[s_DPHY2_RD_DQS_VWMC_CS1_CON0] = {
			DPHY2_RD_DQS_VWMC_CS1_CON0,
			DPHY2_SW_RD_DQS_VWMC_CS1_CON0 },
		[s_DPHY2_RD_DQS_VWML_CS0_CON0] = {
			DPHY2_RD_DQS_VWML_CS0_CON0,
			DPHY2_SW_RD_DQS_VWML_CS0_CON0 },
		[s_DPHY2_RD_DQS_VWML_CS1_CON0] = {
			DPHY2_RD_DQS_VWML_CS1_CON0,
			DPHY2_SW_RD_DQS_VWML_CS1_CON0 },
		[s_DPHY2_WR_DESKEWC_CS0_CON0] = {
			DPHY2_WR_DESKEWC_CS0_CON0,
			DPHY2_SW_WR_DESKEWC_CS0_CON0 },
		[s_DPHY2_WR_DESKEWC_CS0_CON1] = {
			DPHY2_WR_DESKEWC_CS0_CON1,
			DPHY2_SW_WR_DESKEWC_CS0_CON1 },
		[s_DPHY2_WR_DESKEWC_CS0_CON2] = {
			DPHY2_WR_DESKEWC_CS0_CON2,
			DPHY2_SW_WR_DESKEWC_CS0_CON2 },
		[s_DPHY2_WR_DESKEWC_CS0_CON3] = {
			DPHY2_WR_DESKEWC_CS0_CON3,
			DPHY2_SW_WR_DESKEWC_CS0_CON3 },
		[s_DPHY2_WR_DESKEWC_CS0_CON4] = {
			DPHY2_WR_DESKEWC_CS0_CON4,
			DPHY2_SW_WR_DESKEWC_CS0_CON4 },
		[s_DPHY2_WR_DESKEWC_CS0_CON5] = {
			DPHY2_WR_DESKEWC_CS0_CON5,
			DPHY2_SW_WR_DESKEWC_CS0_CON5 },
		[s_DPHY2_WR_DESKEWC_CS0_CON6] = {
			DPHY2_WR_DESKEWC_CS0_CON6,
			DPHY2_SW_WR_DESKEWC_CS0_CON6 },
		[s_DPHY2_WR_DESKEWC_CS0_CON7] = {
			DPHY2_WR_DESKEWC_CS0_CON7,
			DPHY2_SW_WR_DESKEWC_CS0_CON7 },
		[s_DPHY2_DM_DESKEWC_CS0_CON0] = {
			DPHY2_DM_DESKEWC_CS0_CON0,
			DPHY2_SW_DM_DESKEWC_CS0_CON0 },
		[s_DPHY2_WR_DESKEWC_CS1_CON0] = {
			DPHY2_WR_DESKEWC_CS1_CON0,
			DPHY2_SW_WR_DESKEWC_CS1_CON0 },
		[s_DPHY2_WR_DESKEWC_CS1_CON1] = {
			DPHY2_WR_DESKEWC_CS1_CON1,
			DPHY2_SW_WR_DESKEWC_CS1_CON1 },
		[s_DPHY2_WR_DESKEWC_CS1_CON2] = {
			DPHY2_WR_DESKEWC_CS1_CON2,
			DPHY2_SW_WR_DESKEWC_CS1_CON2 },
		[s_DPHY2_WR_DESKEWC_CS1_CON3] = {
			DPHY2_WR_DESKEWC_CS1_CON3,
			DPHY2_SW_WR_DESKEWC_CS1_CON3 },
		[s_DPHY2_WR_DESKEWC_CS1_CON4] = {
			DPHY2_WR_DESKEWC_CS1_CON4,
			DPHY2_SW_WR_DESKEWC_CS1_CON4 },
		[s_DPHY2_WR_DESKEWC_CS1_CON5] = {
			DPHY2_WR_DESKEWC_CS1_CON5,
			DPHY2_SW_WR_DESKEWC_CS1_CON5 },
		[s_DPHY2_WR_DESKEWC_CS1_CON6] = {
			DPHY2_WR_DESKEWC_CS1_CON6,
			DPHY2_SW_WR_DESKEWC_CS1_CON6 },
		[s_DPHY2_WR_DESKEWC_CS1_CON7] = {
			DPHY2_WR_DESKEWC_CS1_CON7,
			DPHY2_SW_WR_DESKEWC_CS1_CON7 },
		[s_DPHY2_DM_DESKEWC_CS1_CON0] = {
			DPHY2_DM_DESKEWC_CS1_CON0,
			DPHY2_SW_DM_DESKEWC_CS1_CON0 },
		[s_DPHY2_WR_DESKEWL_CS0_CON0] = {
			DPHY2_WR_DESKEWL_CS0_CON0,
			DPHY2_SW_WR_DESKEWL_CS0_CON0 },
		[s_DPHY2_WR_DESKEWL_CS0_CON1] = {
			DPHY2_WR_DESKEWL_CS0_CON1,
			DPHY2_SW_WR_DESKEWL_CS0_CON1 },
		[s_DPHY2_WR_DESKEWL_CS0_CON2] = {
			DPHY2_WR_DESKEWL_CS0_CON2,
			DPHY2_SW_WR_DESKEWL_CS0_CON2 },
		[s_DPHY2_WR_DESKEWL_CS0_CON3] = {
			DPHY2_WR_DESKEWL_CS0_CON3,
			DPHY2_SW_WR_DESKEWL_CS0_CON3 },
		[s_DPHY2_WR_DESKEWL_CS0_CON4] = {
			DPHY2_WR_DESKEWL_CS0_CON4,
			DPHY2_SW_WR_DESKEWL_CS0_CON4 },
		[s_DPHY2_WR_DESKEWL_CS0_CON5] = {
			DPHY2_WR_DESKEWL_CS0_CON5,
			DPHY2_SW_WR_DESKEWL_CS0_CON5 },
		[s_DPHY2_WR_DESKEWL_CS0_CON6] = {
			DPHY2_WR_DESKEWL_CS0_CON6,
			DPHY2_SW_WR_DESKEWL_CS0_CON6 },
		[s_DPHY2_WR_DESKEWL_CS0_CON7] = {
			DPHY2_WR_DESKEWL_CS0_CON7,
			DPHY2_SW_WR_DESKEWL_CS0_CON7 },
		[s_DPHY2_DM_DESKEWL_CS0_CON0] = {
			DPHY2_DM_DESKEWL_CS0_CON0,
			DPHY2_SW_DM_DESKEWL_CS0_CON0 },
		[s_DPHY2_WR_DESKEWL_CS1_CON0] = {
			DPHY2_WR_DESKEWL_CS1_CON0,
			DPHY2_SW_WR_DESKEWL_CS1_CON0 },
		[s_DPHY2_WR_DESKEWL_CS1_CON1] = {
			DPHY2_WR_DESKEWL_CS1_CON1,
			DPHY2_SW_WR_DESKEWL_CS1_CON1 },
		[s_DPHY2_WR_DESKEWL_CS1_CON2] = {
			DPHY2_WR_DESKEWL_CS1_CON2,
			DPHY2_SW_WR_DESKEWL_CS1_CON2 },
		[s_DPHY2_WR_DESKEWL_CS1_CON3] = {
			DPHY2_WR_DESKEWL_CS1_CON3,
			DPHY2_SW_WR_DESKEWL_CS1_CON3 },
		[s_DPHY2_WR_DESKEWL_CS1_CON4] = {
			DPHY2_WR_DESKEWL_CS1_CON4,
			DPHY2_SW_WR_DESKEWL_CS1_CON4 },
		[s_DPHY2_WR_DESKEWL_CS1_CON5] = {
			DPHY2_WR_DESKEWL_CS1_CON5,
			DPHY2_SW_WR_DESKEWL_CS1_CON5 },
		[s_DPHY2_WR_DESKEWL_CS1_CON6] = {
			DPHY2_WR_DESKEWL_CS1_CON6,
			DPHY2_SW_WR_DESKEWL_CS1_CON6 },
		[s_DPHY2_WR_DESKEWL_CS1_CON7] = {
			DPHY2_WR_DESKEWL_CS1_CON7,
			DPHY2_SW_WR_DESKEWL_CS1_CON7 },
		[s_DPHY2_DM_DESKEWL_CS1_CON0] = {
			DPHY2_DM_DESKEWL_CS1_CON0,
			DPHY2_SW_DM_DESKEWL_CS1_CON0 },
		[s_DPHY2_PRBS_CON2] = {
			DPHY2_PRBS_CON2,
			DPHY2_PRBS_CON4 },
		[s_DPHY2_PRBS_CON3] = {
			DPHY2_PRBS_CON3,
			DPHY2_PRBS_CON5 },
		[s_DPHY2_ZQ_CON9] = {
			DPHY2_ZQ_CON9,
			DPHY2_ZQ_CON9 },
	};

	return &train_save_restore_regs[idx];
}

static unsigned int ddr_freq_param(enum ddr_freq_t freq, unsigned int index)
{
	static const unsigned int ddr_reg_freq[f_reg_max][AB_DRAM_FREQ_MAX] = {
	    /* f_DPHY_DVFS_CON */
	    { DVFS_CON(1866), DVFS_CON(1600), DVFS_CON(1200),
				DVFS_CON(933), DVFS_CON(800)},
	    /* f_DPHY_CAL_CON2 : TODO(b/122059867) remove all magic numbers */
	    { 0x84070000, 0x84070000, 0x84070000, 0x04050000, 0x04050000 },
	    /* f_DPHY_GNR_CON0_NODBI */
	    { 0x43005024, 0x43005020, 0x4300501c, 0x43005016, 0x43005010 },
	    /* f_DPHY_GNR_CON0_DBI */
	    { 0x47106024, 0x47106020, 0x4710601c, 0x47106016, 0x47106010 },
	    /* f_DREX_TIMINGRFCPB */
	    { TMGPBR_1866, TMGPBR_1600, TMGPBR_1200, TMGPBR_933, TMGPBR_800 },
	    /* f_DREX_TIMINGROW */
	    { TMGROW_1866, TMGROW_1600, TMGROW_1200, TMGROW_933, TMGROW_800 },
	    /* f_DREX_TIMINGDATA */
	    { TMGDTA_1866, TMGDTA_1600, TMGDTA_1200, TMGDTA_933, TMGDTA_800 },
	    /* f_DREX_TIMINGPOWER */
	    { TMGPWR_1866, TMGPWR_1600, TMGPWR_1200, TMGPWR_933, TMGPWR_800 },
	};

	return ddr_reg_freq[index][freq];
};

static inline void ddr_reg_wr_otp(uint32_t addr, uint32_t otp_idx)
{
	ddr_reg_wr(addr, ddr_otp_rd(otp_idx));
}

static int ddr_reg_poll(uint32_t reg, uint32_t poll_idx)
{
	static const struct ddr_reg_poll_t ddr_reg_poll_array[] = {
		[p_pll_con0_pll_phy_mif]	= { PLL_STABLE,
						    PLL_STABLE,
						    POLL_TIMEOUT_USEC },
		/* zq_done[0] bit */
		[p_DPHY_ZQ_CON1]		= { ZQ_DONE,
						    ZQ_DONE,
						    POLL_TIMEOUT_USEC },
		/* ctrl_locked[18] bit */
		[p_DPHY_MDLL_CON1]		= { CTRL_LOCKED,
						    CTRL_LOCKED,
						    POLL_TIMEOUT_USEC },
		/* dfi_init_complete[4:3] */
		[p_DREX_PHYSTATUS_dfi]		= { DFI_INIT_COMPLETE_ALL,
						    DFI_INIT_COMPLETE_ALL,
						    POLL_TIMEOUT_USEC },
		/* train_complete[15:14] */
		[p_DREX_PHYSTATUS_train]	= { TRAIN_COMPLETE_ALL,
						    TRAIN_COMPLETE_ALL,
						    POLL_TIMEOUT_USEC },
		/* chip_sref_state[8]=1 */
		[p_DREX_CHIPSTATUS_sr_enter]	= { CHIP_SREF_STATE(0),
						    CHIP_SREF_ENTRY(0),
						    POLL_TIMEOUT_USEC },
		/* chip_sref_state[8]=0 */
		[p_DREX_CHIPSTATUS_sr_exit]	= { CHIP_SREF_STATE(0),
						    CHIP_SREF_EXIT(0),
						    POLL_TIMEOUT_USEC },
		/* prbs_done[0] = 1 */
		[p_DPHY_PRBS_CON0_prbs_done]	= { PRBS_DONE,
						    PRBS_DONE,
						    POLL_TIMEOUT_USEC },
		/* prbs_disable[0] = 0 */
		[p_DPHY_PRBS_CON0_prbs_disable]	= { PRBS_DONE,
						    0x00000000,
						    POLL_TIMEOUT_USEC},
	};

	const struct ddr_reg_poll_t *poll = &ddr_reg_poll_array[poll_idx];
	uint32_t poll_multiplier;
	uint32_t reg_val;
	unsigned long timeout;

	/* Reading the register before calculating the timeout makes sure the
	 * previous write request to DREX or DPHY (for example training or
	 * mdll lock request) is completed.
	 * As ddr_reg_rd() translates to pcie_config_read() (which is a
	 * non-posted pcie transaction), the previous pcie_config_write() has
	 * to be completed (which is posted pcie transaction) before completing
	 * the current read request.
	 */
	reg_val = ddr_reg_rd(reg);

	poll_multiplier = ddr_otp_rd(o_SECURE_JTAG2) + 1;
	timeout = jiffies +
		  usecs_to_jiffies(poll->usec_timeout * poll_multiplier);

	while (((reg_val & poll->mask) != poll->val) &&
			time_before(jiffies, timeout)) {
		ddr_usleep(DDR_POLL_USLEEP_MIN);
		reg_val = ddr_reg_rd(reg);
	}

	/* There is a chance that the above loop can exit because of the current
	 * task is scheduled out for more than the "timeout" jiffies. In this
	 * case the reg_val can contain the old value.
	 *
	 * If the poll condition doesn't meet, then read the register one more
	 * time which will handle the above scenario.
	 */
	if ((reg_val & poll->mask) != poll->val)
		reg_val = ddr_reg_rd(reg);

	if ((reg_val & poll->mask) != poll->val) {
		pr_err("ddr status poll failed for idx: 0x%x\n", poll_idx);
		pr_err("reg: 0x%x, val: 0x%x, poll_msk: 0x%x, poll_val: 0x%x\n",
			reg, reg_val, poll->mask, poll->val);
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

static int ddr_config_cmu_mif_lowfreq(void)
{
	ddr_reg_wr_otp(PLL_LOCKTIME_PLL_PHY_MIF, o_Reserved_DDR_INIT_0);
	ddr_reg_wr_otp(PLL_CON0_PLL_PHY_MIF, o_SECURE_JTAG0);

	if (ddr_reg_poll(PLL_CON0_PLL_PHY_MIF, p_pll_con0_pll_phy_mif)) {
		pr_err("%s, mif pll lock failed\n", __func__);
		return DDR_FAIL;
	}

	ddr_reg_wr_otp(PLL_CON0_PLL_PHY_MIF, o_SECURE_JTAG1);
	ddr_reg_wr_otp(CLK_CON_DIV_DIV4_PLLCLK_MIF, o_Reserved_DDR_INIT_3);
	ddr_reg_wr_otp(CLK_CON_DIV_DIV2_PLLCLK_MIF, o_Reserved_DDR_INIT_4);
	ddr_reg_wr_otp(CLK_CON_DIV_DFI_DIV2, o_Reserved_DDR_INIT_3);

	return DDR_SUCCESS;
}

static void ddr_set_pll_to_oscillator(void)
{
	ddr_reg_set(DREX_MEMCONTROL, CLK_STOP_EN);
	ddr_reg_clr(MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);

	/* Select the oscillator clock from SYSREG_MIF */
	ddr_reg_clr(MIF_PLL_WRAP_CTRL_REG, SEL_CLKMUX_PLL);

	/* Disable PLL and move PLL to oscillator clock */
	ddr_reg_wr(PLL_CON0_PLL_PHY_MIF, PLL_MUX_SEL(PLL_MUX_SEL_OSCCLK));

	ddr_reg_set(MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);
	ddr_reg_clr(DREX_MEMCONTROL, CLK_STOP_EN);
}

static int ddr_set_pll_freq(enum ddr_freq_t freq)
{
	static const struct airbrush_ddr_pll_t
				ab_ddr_pll_pms_table[AB_DRAM_FREQ_MAX] = {
		[AB_DRAM_FREQ_MHZ_1866] = { 3, 583, 0},
		[AB_DRAM_FREQ_MHZ_1600] = { 3, 500, 0},
		[AB_DRAM_FREQ_MHZ_1200] = { 3, 375, 0},
		[AB_DRAM_FREQ_MHZ_933]  = { 3, 583, 1},
		[AB_DRAM_FREQ_MHZ_800]  = { 3, 500, 1},
	};
	const struct airbrush_ddr_pll_t *pms;

	if (freq >= AB_DRAM_FREQ_MAX)
		return -EINVAL;

	/* With the given PMS values ab_ddr_pll_pms_table[freq],
	 * try to configure the PLL for MIF.
	 */
	pms = &ab_ddr_pll_pms_table[freq];

	ddr_reg_set(DREX_MEMCONTROL, CLK_STOP_EN);
	ddr_reg_clr(MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);

	/* Select the oscillator clock from SYSREG_MIF */
	ddr_reg_clr(MIF_PLL_WRAP_CTRL_REG, SEL_CLKMUX_PLL);

	ddr_reg_wr(PLL_CON0_PLL_PHY_MIF, PLL_MUX_SEL(PLL_MUX_SEL_OSCCLK));
	ddr_reg_clr_set(PLL_CON0_PLL_PHY_MIF, PLL_PMS_MSK,
			PLL_PMS(pms->p, pms->m, pms->s) | PLL_ENABLE);

	if (ddr_reg_poll(PLL_CON0_PLL_PHY_MIF, p_pll_con0_pll_phy_mif)) {
		pr_err("%s, mif pll lock failed\n", __func__);
		return DDR_FAIL;
	}

	ddr_reg_set(PLL_CON0_PLL_PHY_MIF, PLL_MUX_SEL(PLL_MUX_SEL_PLLOUT));

	/* Select the mif pll fout from SYSREG_MIF */
	ddr_reg_set(MIF_PLL_WRAP_CTRL_REG, SEL_CLKMUX_PLL);

	ddr_reg_set(MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);
	ddr_reg_clr(DREX_MEMCONTROL, CLK_STOP_EN);

	return DDR_SUCCESS;
}

static void ddr_ungate_phy_clock(void)
{
	ddr_reg_set(MIF_PLL_WRAP_CTRL_REG, WRAP_PLL_IS_STABLE);
	ddr_reg_set(MIF_PLL_WRAP_CTRL_REG, SEL_CLKMUX_PLL);
	ddr_reg_set(MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);
}

static void ddr_deassert_phy_reset_while_phy_is_gated(void)
{
	ddr_reg_set(PHY0_INIT_CTRL_REG, INIT_PLL_IS_STABLE);
	ddr_reg_set(PHY1_INIT_CTRL_REG, INIT_PLL_IS_STABLE);
	ddr_reg_set(PHY0_RST_CTRL_REG, RST_N | DIV_RST_N);
	ddr_reg_set(PHY1_RST_CTRL_REG, RST_N | DIV_RST_N);
}

static int ddr_block_axi_transactions(void)
{
	unsigned long timeout;

	timeout = jiffies + usecs_to_jiffies(DDR_AXI_BLOCK_TIMEOUT);
	ddr_reg_clr(DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);

	while (((ddr_reg_rd(DREX_EMPTY_STATE) & IDLE_STATE_MASK) !=
	       IDLE_STATE_MASK) && time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	/* check for the ddr empty state */
	if ((ddr_reg_rd(DREX_EMPTY_STATE) & IDLE_STATE_MASK) !=
	    IDLE_STATE_MASK) {
		pr_err("Block AXI: Error!! DDR AXI Block timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void ddr_initialize_phy_pre(void)
{
	ddr_reg_clr(DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);
	ddr_reg_wr(DREX_PHYCONTROL0, PAUSE_NO_RELOCK);

	/* set ignore_dic[31] (ignore dfi_init_complete) &
	 * pclk_async PCLK Async
	 */
	ddr_reg_clr(DREX_DFIRSTCONTROL, PB_WA_EN);
	ddr_reg_clr(DREX_MEMCONTROL, PB_REF_EN);
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);
}

static void ddr_initialize_phy(enum ddr_freq_t freq)
{
	ddr_reg_set(DPHY_MON_CON0, MDLL_MONITOR_EN);
	ddr_reg_set(DPHY2_MON_CON0, MDLL_MONITOR_EN);
	ddr_reg_clr(DREX_CONCONTROL, DFI_INIT_START | DFI_INIT_START_PHY2);

	ddr_reg_clr_set(DPHY_DVFS_CON, DVFS_CON_MSK,
			ddr_freq_param(freq, f_DPHY_DVFS_CON));
	ddr_reg_wr(DPHY_GNR_CON0, ddr_freq_param(freq, f_DPHY_GNR_CON0_NODBI));
	ddr_reg_wr_otp(DPHY_CAL_CON0, o_DPHY_CAL_CON0_0);
	ddr_reg_wr(DPHY_CAL_CON2, ddr_freq_param(freq, f_DPHY_CAL_CON2));
	ddr_reg_wr_otp(DPHY_LP_CON0, o_DPHY_LP_CON0_2);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_2);

	ddr_reg_clr_set(DPHY2_DVFS_CON, DVFS_CON_MSK,
			ddr_freq_param(freq, f_DPHY_DVFS_CON));
	ddr_reg_wr(DPHY2_GNR_CON0, ddr_freq_param(freq, f_DPHY_GNR_CON0_NODBI));
	ddr_reg_wr_otp(DPHY2_CAL_CON0, o_DPHY_CAL_CON0_0);
	ddr_reg_wr(DPHY2_CAL_CON2, ddr_freq_param(freq, f_DPHY_CAL_CON2));
	ddr_reg_wr_otp(DPHY2_LP_CON0, o_DPHY_LP_CON0_2);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_2);

	/* Disable power features till ddr training completes */
	ddr_reg_clr(DREX_CGCONTROL, PHY_CG_EN);
	ddr_reg_clr(DPHY_LP_CON0, PCL_PD | MDLL_CG_EN | DS_IO_PD);
	ddr_reg_clr(DPHY2_LP_CON0, PCL_PD | MDLL_CG_EN | DS_IO_PD);
	ddr_reg_clr(DREX_MEMCONTROL, DPWRDN_EN);
	ddr_reg_clr(DREX_MEMCONTROL, CLK_STOP_EN);
}

static int ddr_initialize_dfi(int is_init)
{
	ddr_reg_wr_otp(DPHY_MDLL_CON0, o_DPHY_MDLL_CON0_1);
	ddr_reg_wr_otp(DPHY2_MDLL_CON0, o_DPHY_MDLL_CON0_1);

	if (is_init) {
		ddr_reg_set(DREX_CONCONTROL,
			    DFI_INIT_START_PHY2 | DFI_INIT_START);
		if (ddr_reg_poll(DREX_PHYSTATUS, p_DREX_PHYSTATUS_dfi))
			return DDR_FAIL;

		ddr_reg_clr(DREX_CONCONTROL,
			    DFI_INIT_START_PHY2 | DFI_INIT_START);
	}

	ddr_reg_clr(DPHY_MDLL_CON0, CTRL_DLL_ON);
	ddr_reg_clr(DPHY2_MDLL_CON0, CTRL_DLL_ON);

	return DDR_SUCCESS;
}

static void ddr_dram_reset_sequence(void)
{
	/* RESET_DRAM */
	ddr_reg_set(DREX_DFIRSTCONTROL, DFI_RESET_CONTROL);
	ddr_usleep(DDR_INIT_TIMING_tINIT1_USEC);
	ddr_reg_clr(DREX_DFIRSTCONTROL, DFI_RESET_CONTROL);
	ddr_usleep(DDR_INIT_TIMING_tINIT3_USEC);
}

static void ddr_power_down_exit_sequence(void)
{
	/* ExitPD start */
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_21);
}

static void ddr_mrw_set_vref_odt_etc(int freq)
{
	/* MRW Settings for each frequency supported */
	static const struct airbrush_ddr_mrw_set_t
				ab_ddr_mrw_set_table[AB_DRAM_FREQ_MAX] = {
		[AB_DRAM_FREQ_MHZ_1866] = { 0x5b8, 0x8d8, 0xfc4, 0x10c90,
					    MRW13_DEFAULT, 0x21850, 0x9010000},
		[AB_DRAM_FREQ_MHZ_1600] = { 0x578, 0x8b4, 0xfc4, 0x10c90,
					    MRW13_DEFAULT, 0x21850, 0x9010000},
		[AB_DRAM_FREQ_MHZ_1200] = { 0x538, 0x890, 0xfc4, 0x10c90,
					    MRW13_DEFAULT, 0x21850, 0x9010000},
		[AB_DRAM_FREQ_MHZ_933]  = { 0x4d8, 0x86c, 0xfc4, 0x10c90,
					    MRW13_DEFAULT, 0x21850, 0x9010000 },
		[AB_DRAM_FREQ_MHZ_800]  = { 0x498, 0x848, 0xfc4, 0x10c90,
					    MRW13_DEFAULT, 0x21850, 0x9010000 },
	};

	/* LPDDR4_chip_Init */
	ddr_reg_wr(DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr1);
	ddr_reg_wr(DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr2);
	ddr_reg_wr(DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr3);
	ddr_reg_wr(DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr11);
	ddr_reg_wr(DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr13);
	ddr_reg_wr(DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr22);
	ddr_reg_wr(DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr8);
}

static void ddr_dphy_ctrl_resync(void)
{
	/* When CTRL_RESYNC bit transits from LOW to HIGH, pointers of FIFO
	 * within PHY and all of the DLL information (Read/Write/CA/CS DLL)
	 * is updated.
	 */
	ddr_reg_set(DPHY_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(DPHY_OFFSETD_CON0, CTRL_RESYNC);

	ddr_reg_set(DPHY2_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(DPHY2_OFFSETD_CON0, CTRL_RESYNC);
}

static void ddr_dram_dctrl_resync(void)
{
	/* force DLL resync enable/disable */
	ddr_reg_set(DREX_PHYCONTROL0, FP_RESYNC);
	ddr_reg_clr(DREX_PHYCONTROL0, FP_RESYNC);

	ddr_dphy_ctrl_resync();
}

static void ddr_dram_zq_calibration(void)
{
	/* lpddr4 zq calibration */
	ddr_reg_wr_otp(DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_2);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_11);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_12);
	ddr_reg_wr_otp(DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_0);
	ddr_reg_wr_otp(DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_1);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_11);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_12);
	ddr_reg_wr_otp(DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_0);

	ddr_dram_dctrl_resync();
}

static int ddr_zq_calibration(void)
{
	/* zq calibration is implemented as one-time long calibration */
	ddr_reg_clr_set(DPHY_ZQ_CON2, CTRL_ZQ_CLK_DIV_MSK, CTRL_ZQ_CLK_DIV(7));
	ddr_reg_set(DPHY_ZQ_CON0, ZQ_CLK_EN);
	ddr_reg_clr_set(DPHY_ZQ_CON0, ZQ_MANUAL_MODE_MSK,
			ZQ_MANUAL_MODE(ZQ_MANUAL_MODE_LONG));
	ddr_reg_set(DPHY_ZQ_CON0, ZQ_CLK_DIV_EN);
	ddr_reg_set(DPHY_ZQ_CON0, ZQ_MANUAL_STR);

	ddr_reg_clr_set(DPHY2_ZQ_CON2, CTRL_ZQ_CLK_DIV_MSK, CTRL_ZQ_CLK_DIV(7));
	ddr_reg_set(DPHY2_ZQ_CON0, ZQ_CLK_EN);
	ddr_reg_clr_set(DPHY2_ZQ_CON0, ZQ_MANUAL_MODE_MSK,
			ZQ_MANUAL_MODE(ZQ_MANUAL_MODE_LONG));
	ddr_reg_set(DPHY2_ZQ_CON0, ZQ_CLK_DIV_EN);
	ddr_reg_set(DPHY2_ZQ_CON0, ZQ_MANUAL_STR);

	/* wait zq calibration */
	if (ddr_reg_poll(DPHY_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;

	ddr_reg_clr(DPHY_ZQ_CON0, ZQ_MANUAL_STR);
	ddr_reg_clr(DPHY_ZQ_CON0, ZQ_CLK_DIV_EN);
	ddr_reg_clr(DPHY2_ZQ_CON0, ZQ_MANUAL_STR);
	ddr_reg_clr(DPHY2_ZQ_CON0, ZQ_CLK_DIV_EN);

	ddr_reg_clr(DPHY_OFFSETD_CON0, UPD_MODE);
	ddr_reg_clr(DPHY2_OFFSETD_CON0, UPD_MODE);

	ddr_reg_clr(DREX_CONCONTROL, UPDATE_MODE);

	return DDR_SUCCESS;
}

static int ddr_io_initialization(int from_suspend)
{
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_3);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_1);
	ddr_reg_wr_otp(DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
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
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_15);
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_0);
	ddr_reg_wr_otp(DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);

	/* perform ZQ calibration */
	if (ddr_zq_calibration()) {
		pr_err("%s: ddr zq calibration failed\n", __func__);
		return DDR_FAIL;
	}

	if (!from_suspend) {
		/* set dram odt settings */
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_0);
		/* set dram drive-strength */
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_30);
	}

	/* set host drive strength */
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

	ddr_reg_wr_otp(DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_1);
	ddr_reg_wr_otp(DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_0);
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
	ddr_reg_wr(DPHY_GATE_CON0, 0xf00ffff);
	ddr_reg_wr(DPHY2_GATE_CON0, 0xf00ffff);

	ddr_reg_set(DPHY_MDLL_CON0, CTRL_DLL_ON);
	ddr_reg_set(DPHY2_MDLL_CON0, CTRL_DLL_ON);

	ddr_reg_clr(DPHY_MDLL_CON0, CTRL_START);
	ddr_reg_clr(DPHY2_MDLL_CON0, CTRL_START);

	udelay(DDR_DLL_CTRL_OFF_ON_UDELAY);

	ddr_reg_set(DPHY_MDLL_CON0, CTRL_START);
	ddr_reg_set(DPHY2_MDLL_CON0, CTRL_START);

	/* Poll for ctrl_lock */
	if (ddr_reg_poll(DPHY_MDLL_CON1, p_DPHY_MDLL_CON1)) {
		pr_err("%s: phy dll stable lock fail\n", __func__);
		return DDR_FAIL;
	}

	if (ddr_reg_poll(DPHY2_MDLL_CON1, p_DPHY_MDLL_CON1)) {
		pr_err("%s: phy2 dll stable lock fail\n", __func__);
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

static void ddr_set_drex_timing_parameters(enum ddr_freq_t freq)
{
	ddr_reg_clr_set(DREX_PRECHCONFIG0,
			TP_EN_MSK | PORT_POLICY_MSK,
			PORT_POLICY(PORT_POLICY_OPEN_PAGE));

	ddr_reg_wr(DREX_PWRDNCONFIG, 0xffff00ff);
	ddr_reg_wr(DREX_TIMINGSETSW, 0x1);
	ddr_reg_wr(DREX_TIMINGRFCPB, ddr_freq_param(freq, f_DREX_TIMINGRFCPB));
	ddr_reg_wr(DREX_TIMINGROW0, ddr_freq_param(freq, f_DREX_TIMINGROW));
	ddr_reg_wr(DREX_TIMINGROW1, ddr_freq_param(freq, f_DREX_TIMINGROW));
	ddr_reg_wr(DREX_TIMINGDATA0, ddr_freq_param(freq, f_DREX_TIMINGDATA));
	ddr_reg_wr(DREX_TIMINGDATA1, ddr_freq_param(freq, f_DREX_TIMINGDATA));
	ddr_reg_wr(DREX_TIMINGPOWER0, ddr_freq_param(freq, f_DREX_TIMINGPOWER));
	ddr_reg_wr(DREX_TIMINGPOWER1, ddr_freq_param(freq, f_DREX_TIMINGPOWER));

	/* Set the all-bank and per-bank auto refresh timings */
	ddr_reg_wr(DREX_TIMINGARE, T_REFI_DEFAULT | T_REFIPB_DEFAULT);

	ddr_reg_wr_otp(DREX_ETCTIMING, o_Reserved_DDR_INIT_7);
	ddr_reg_wr_otp(DREX_RDFETCH0, o_Reserved_DDR_INIT_8);
	ddr_reg_wr_otp(DREX_RDFETCH1, o_Reserved_DDR_INIT_9);

	/* DRAM DCTL Resync */
	ddr_reg_wr_otp(DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);

	/* DRAM DCTL Resync */
	ddr_dram_dctrl_resync();
}

static int ddr_set_drex_address_parameters(enum ddr_freq_t freq)
{
	/* DRAM Density Check */
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_25);
	ddr_reg_rd(DREX_MRSTATUS);

	ddr_dram_dctrl_resync();

	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_26);
	ddr_reg_rd(DREX_MRSTATUS);

	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_29);
	ddr_reg_rd(DREX_MRSTATUS);
	ddr_dram_dctrl_resync();

	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_28);
	ddr_reg_rd(DREX_MRSTATUS);
	ddr_dram_dctrl_resync();

	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_27);
	ddr_reg_rd(DREX_MRSTATUS);
	ddr_dram_dctrl_resync();

	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_27);
	ddr_reg_wr(DPHY_GNR_CON0, ddr_freq_param(freq, f_DPHY_GNR_CON0_DBI));
	ddr_reg_wr(DPHY2_GNR_CON0, ddr_freq_param(freq, f_DPHY_GNR_CON0_DBI));
	ddr_dram_dctrl_resync();

	/* DRAM Density Setting */
	ddr_reg_wr_otp(DREX_MEMCONTROL, o_Reserved_DDR_INIT_10);
	ddr_reg_wr(DREX_ASP_MEMBASECONFIG0, CHUNK_START_END);
	ddr_reg_wr_otp(DREX_ASP_MEMCONFIG0, o_Reserved_DDR_INIT_11);
	ddr_reg_wr(DREX_ASP_CHIP0SIZECONFIG, CHIP_SIZE_512MB);

	/* Enable dbi_en at controller side. MR3 is already updated with
	 * read and write DBI enable
	 */
	ddr_reg_set(DREX_MEMCONTROL, DBI_EN);

	return DDR_SUCCESS;
}

static void ddr_prepare_training(enum ddr_freq_t freq)
{
	ddr_reg_clr(DPHY_OFFSETD_CON0, UPD_MODE);
	ddr_reg_clr_set(DPHY_DVFS_CON, DVFS_CON_MSK,
			ddr_freq_param(freq, f_DPHY_DVFS_CON));
	ddr_reg_set(DPHY_MDLL_CON0, CLKM_CG_EN_SW);

	ddr_reg_clr(DPHY2_OFFSETD_CON0, UPD_MODE);
	ddr_reg_clr_set(DPHY2_DVFS_CON, DVFS_CON_MSK,
			ddr_freq_param(freq, f_DPHY_DVFS_CON));
	ddr_reg_set(DPHY2_MDLL_CON0, CLKM_CG_EN_SW);
}

static void ddr_ca_training(void)
{
	ddr_reg_clr(DPHY_LP_CON0, DS_IO_PD);
	ddr_reg_clr(DPHY2_LP_CON0, DS_IO_PD);

	ddr_reg_clr_set(DPHY_CBT_CON0, CBT_CS_MSK, CBT_CS(CBT_CS_RANK0));
	ddr_reg_set(DPHY_CAL_CON0, WRLVL_MODE);
	ddr_reg_set(DPHY_CAL_CON0, CA_CAL_MODE);

	ddr_reg_clr_set(DPHY2_CBT_CON0, CBT_CS_MSK, CBT_CS(CBT_CS_RANK0));
	ddr_reg_set(DPHY2_CAL_CON0, WRLVL_MODE);
	ddr_reg_set(DPHY2_CAL_CON0, CA_CAL_MODE);

	ddr_dram_dctrl_resync();

	ddr_reg_clr(DPHY_CAL_CON0, WRLVL_MODE | CA_CAL_MODE);
	ddr_reg_clr(DPHY_CBT_CON0, CBT_CA_VREF_MODE_DS0 | CBT_CA_VREF_MODE_DS1);
	ddr_reg_clr(DPHY2_CAL_CON0, WRLVL_MODE | CA_CAL_MODE);
	ddr_reg_clr(DPHY2_CBT_CON0, CBT_CA_VREF_MODE_DS0 |
				    CBT_CA_VREF_MODE_DS1);

	ddr_reg_set(DPHY_LP_CON0, DS_IO_PD);
	ddr_reg_set(DPHY2_LP_CON0, DS_IO_PD);
}

static int ddr_odt_training(void)
{
	/* On_Die_Termination */
	ddr_reg_clr(DPHY_LP_CON0, CTRL_PULLD_DQS);
	ddr_reg_clr_set(DPHY_ZQ_CON0, ZQ_MODE_NOTERM | ZQ_RGDDR3, ZQ_MODE_LP4);
	ddr_reg_clr(DPHY_ZQ_CON6, ZQ_DS0_NOTERM | ZQ_DS1_NOTERM);
	ddr_reg_clr(DPHY_CAL_CON2, CTRL_RODT_DISABLE);
	ddr_reg_clr_set(DPHY_CAL_CON2, CTRL_READADJ_MSK | CTRL_READDURADJ_MSK,
			CTRL_READADJ(1) | CTRL_READDURADJ(5));
	ddr_reg_clr(DPHY_GNR_CON0, CTRL_DFDQS);

	ddr_reg_clr(DPHY2_LP_CON0, CTRL_PULLD_DQS);
	ddr_reg_clr_set(DPHY2_ZQ_CON0, ZQ_MODE_NOTERM | ZQ_RGDDR3, ZQ_MODE_LP4);
	ddr_reg_clr(DPHY2_ZQ_CON6, ZQ_DS0_NOTERM | ZQ_DS1_NOTERM);
	ddr_reg_clr(DPHY2_CAL_CON2, CTRL_RODT_DISABLE);
	ddr_reg_clr_set(DPHY2_CAL_CON2, CTRL_READADJ_MSK | CTRL_READDURADJ_MSK,
			CTRL_READADJ(1) | CTRL_READDURADJ(5));
	ddr_reg_clr(DPHY2_GNR_CON0, CTRL_DFDQS);

	/* ZQ One-time forced calibration */
	if (ddr_zq_calibration()) {
		pr_err("%s: one time forced zq calibration failed\n", __func__);
		return DDR_FAIL;
	}

	ddr_dphy_ctrl_resync();
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);

	return DDR_SUCCESS;
}

static void ddr_autodqs_clean_gate_training(enum ddr_freq_t freq)
{
	if (is_low_freq(freq)) {
		ddr_reg_clr(DPHY_CAL_CON4, GLITCH_REMOVAL_EN);
		ddr_reg_clr(DPHY_CAL_CON3, AUTO_DQS_CLEAN);
		ddr_reg_clr_set(DPHY_CAL_CON2,
				CTRL_READADJ_MSK |
				CTRL_READDURADJ_MSK |
				CTRL_GATEADJ_MSK |
				CTRL_GATEDURADJ_MSK |
				CTRL_SHGATE,
				CTRL_READADJ(0) |
				CTRL_READDURADJ(5) |
				CTRL_GATEADJ(0) |
				CTRL_GATEDURADJ(4));
		ddr_reg_clr(DPHY_GNR_CON0, CTRL_DFDQS);
		ddr_reg_clr_set(DPHY_TESTIRCV_CON0,
				DQS0_TESTIRCV_MSK | DQS1_TESTIRCV_MSK,
				DQS0_TESTIRCV(0x0) | DQS1_TESTIRCV(0x0));
		ddr_reg_clr(DPHY_CAL_CON0, GATE_CAL_MODE);
		ddr_reg_clr(DPHY_CAL_CON2, CTRL_RODT_DISABLE);

		ddr_reg_clr(DPHY2_CAL_CON4, GLITCH_REMOVAL_EN);
		ddr_reg_clr(DPHY2_CAL_CON3, AUTO_DQS_CLEAN);
		ddr_reg_clr_set(DPHY2_CAL_CON2,
				CTRL_READADJ_MSK |
				CTRL_READDURADJ_MSK |
				CTRL_GATEADJ_MSK |
				CTRL_GATEDURADJ_MSK |
				CTRL_SHGATE,
				CTRL_READADJ(0) |
				CTRL_READDURADJ(5) |
				CTRL_GATEADJ(0) |
				CTRL_GATEDURADJ(4));
		ddr_reg_clr(DPHY2_GNR_CON0, CTRL_DFDQS);
		ddr_reg_clr_set(DPHY2_TESTIRCV_CON0,
				DQS0_TESTIRCV_MSK | DQS1_TESTIRCV_MSK,
				DQS0_TESTIRCV(0x0) | DQS1_TESTIRCV(0x0));
		ddr_reg_clr(DPHY2_CAL_CON0, GATE_CAL_MODE);
		ddr_reg_clr(DPHY2_CAL_CON2, CTRL_RODT_DISABLE);
	} else {
		ddr_reg_set(DPHY_CAL_CON4, GLITCH_REMOVAL_EN);
		ddr_reg_set(DPHY_GNR_CON0, CTRL_DFDQS);
		ddr_reg_clr_set(DPHY_CAL_CON2,
				CTRL_READADJ_MSK |
				CTRL_READDURADJ_MSK,
				CTRL_READADJ(0) |
				CTRL_READDURADJ(7));
		ddr_reg_clr_set(DPHY_TESTIRCV_CON0,
				DQS0_TESTIRCV_MSK | DQS1_TESTIRCV_MSK,
				DQS0_TESTIRCV(0x3) | DQS1_TESTIRCV(0x3));
		ddr_reg_clr(DPHY_CAL_CON0, GATE_CAL_MODE);
		ddr_reg_set(DPHY_CAL_CON3, AUTO_DQS_CLEAN);
		ddr_reg_clr(DPHY_CAL_CON2, CTRL_RODT_DISABLE);

		ddr_reg_set(DPHY2_CAL_CON4, GLITCH_REMOVAL_EN);
		ddr_reg_set(DPHY2_GNR_CON0, CTRL_DFDQS);
		ddr_reg_clr_set(DPHY2_CAL_CON2,
				CTRL_READADJ_MSK |
				CTRL_READDURADJ_MSK,
				CTRL_READADJ(0) |
				CTRL_READDURADJ(7));
		ddr_reg_clr_set(DPHY2_TESTIRCV_CON0,
				DQS0_TESTIRCV_MSK | DQS1_TESTIRCV_MSK,
				DQS0_TESTIRCV(0x3) | DQS1_TESTIRCV(0x3));
		ddr_reg_clr(DPHY2_CAL_CON0, GATE_CAL_MODE);
		ddr_reg_set(DPHY2_CAL_CON3, AUTO_DQS_CLEAN);
		ddr_reg_clr(DPHY2_CAL_CON2, CTRL_RODT_DISABLE);
	}
}

static int ddr_read_dq_calibration(void)
{
	ddr_reg_wr(DPHY_CAL_RD_PATTERN_CON0, PATTERN_55AA55AA);
	ddr_reg_wr(DPHY2_CAL_RD_PATTERN_CON0, PATTERN_55AA55AA);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_16);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_19);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_9);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_6);
	ddr_reg_set(DPHY_CAL_CON0, RD_CAL_MODE);
	ddr_reg_set(DPHY2_CAL_CON0, RD_CAL_MODE);

	ddr_reg_wr(DREX_INIT_TRAIN_CONFIG, 0x0);
	ddr_reg_set(DREX_INIT_TRAIN_CONFIG, INIT_READ_TRAIN_CHIP0);
	ddr_reg_set(DREX_INIT_TRAIN_CONTROL, INIT_TRAIN_START);

	/* poll for train complete	*/
	if (ddr_reg_poll(DREX_PHYSTATUS, p_DREX_PHYSTATUS_train))
		return DDR_FAIL;

	ddr_reg_clr(DREX_INIT_TRAIN_CONTROL, INIT_TRAIN_START);

	return DDR_SUCCESS;
}

static int ddr_write_dq_calibration(void)
{
	ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON0, PATTERN_55AA55AA);
	ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON1, PATTERN_55AA55AA);
	ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON2, PATTERN_55AA55AA);
	ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON3, PATTERN_55AA55AA);
	ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON4, PATTERN_5555);

	ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON0, PATTERN_55AA55AA);
	ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON1, PATTERN_55AA55AA);
	ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON2, PATTERN_55AA55AA);
	ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON3, PATTERN_55AA55AA);
	ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON4, PATTERN_5555);

	ddr_reg_wr(DREX_WRTRA_PATTERN0, PATTERN_AA55AA55);
	ddr_reg_wr(DREX_WRTRA_PATTERN1, PATTERN_AA55AA55);
	ddr_reg_wr(DREX_WRTRA_PATTERN2, PATTERN_5555);

	ddr_reg_set(DPHY_CAL_CON0, WR_CAL_MODE);
	ddr_reg_set(DPHY2_CAL_CON0, WR_CAL_MODE);

	ddr_reg_wr(DREX_INIT_TRAIN_CONFIG, 0x0);
	ddr_reg_set(DREX_INIT_TRAIN_CONFIG, INIT_WRITE_TRAIN_CHIP0);
	ddr_reg_set(DREX_INIT_TRAIN_CONTROL, INIT_TRAIN_START);

	/* poll for train complete	*/
	if (ddr_reg_poll(DREX_PHYSTATUS, p_DREX_PHYSTATUS_train))
		return DDR_FAIL;

	ddr_reg_clr(DREX_INIT_TRAIN_CONTROL, INIT_TRAIN_START);

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
	ddr_reg_set(DPHY_PRBS_CON0, PRBS_READ_START);
	ddr_reg_set(DPHY2_PRBS_CON0, PRBS_READ_START);
	if (ddr_reg_poll(DPHY_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;

	ddr_reg_clr(DPHY_PRBS_CON0, PRBS_READ_START);
	ddr_reg_clr(DPHY2_PRBS_CON0, PRBS_READ_START);
	if (ddr_reg_poll(DPHY_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;

	/* DRAMDCTLResync:  start */
	ddr_dram_dctrl_resync();

	return DDR_SUCCESS;
}

static int ddr_prbs_training_write(void)
{
	/* DDRPHY_RUN_PRBS_TRAINING - WRITE */
	ddr_reg_set(DPHY_PRBS_CON0, PRBS_WRITE_START);
	ddr_reg_set(DPHY2_PRBS_CON0, PRBS_WRITE_START);

	if (ddr_reg_poll(DPHY_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;

	ddr_reg_clr(DPHY_PRBS_CON0, PRBS_WRITE_START);
	ddr_reg_clr(DPHY2_PRBS_CON0, PRBS_WRITE_START);

	if (ddr_reg_poll(DPHY_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;
	if (ddr_reg_poll(DPHY2_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;

	/* DRAMDCTLResync start */
	ddr_dram_dctrl_resync();

	return DDR_SUCCESS;
}

static void ddr_axi_enable_after_all_training(void)
{
	ddr_reg_clr(DPHY_MDLL_CON0, CLKM_CG_EN_SW);
	ddr_reg_clr(DPHY2_MDLL_CON0, CLKM_CG_EN_SW);

	/* Enable all power saving features */
	ddr_reg_set(DREX_CGCONTROL, PHY_CG_EN);
	ddr_reg_set(DPHY_LP_CON0, PCL_PD | MDLL_CG_EN | DS_IO_PD);
	ddr_reg_set(DPHY2_LP_CON0, PCL_PD | MDLL_CG_EN | DS_IO_PD);
	ddr_reg_clr_set(DREX_MEMCONTROL, DPWRDN_TYPE_MSK,
			DPWRDN_TYPE(FORCED_PRECHARGE_PD) | DPWRDN_EN);
	ddr_reg_set(DREX_MEMCONTROL, CLK_STOP_EN);

	/* enable refresh and axi access controls */
	ddr_reg_set(DREX_MEMCONTROL, PB_REF_EN | DBI_EN);
	ddr_reg_set(DREX_DFIRSTCONTROL, PB_WA_EN);
	ddr_reg_clr(DREX_CONCONTROL, DFI_INIT_START_PHY2 | DFI_INIT_START);
	ddr_reg_set(DREX_CONCONTROL, AREF_EN);
	ddr_reg_set(DREX_ALL_INIT_INDI, ALL_INIT_DONE);
	ddr_reg_set(DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);
}

static int ddr_enter_self_refresh_mode(void)
{
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_15);
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_20);

	/* poll for self refresh entry */
	if (ddr_reg_poll(DREX_CHIPSTATUS, p_DREX_CHIPSTATUS_sr_enter)) {
		pr_err("%s: self-refresh entry failed.\n", __func__);
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

static int ddr_exit_self_refresh_mode(void)
{
	ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_22);

	/* poll for self refresh exit */
	if (ddr_reg_poll(DREX_CHIPSTATUS, p_DREX_CHIPSTATUS_sr_exit)) {
		pr_err("%s: self-refresh exit failed.\n", __func__);
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

static void ddr_copy_train_results_from_sram(struct ab_ddr_context *ddr_ctx)
{
	int i;
	unsigned int sram_train_base;
	int freq = ddr_ctx->cur_freq;

	sram_train_base = ddr_ctx->ddr_train_sram_location;
	pr_info("freq: %d, sram_train_base: 0x%x\n", freq, sram_train_base);

	for (i = 0; i < s_train_max_index; i++)
		ddr_ctx->ddr_train_save_value[freq][i] =
				ddr_mem_rd(sram_train_base + (i * 4));
}

static void ddr_train_save_configuration(struct ab_ddr_context *ddr_ctx)
{
	int i;
	int freq = ddr_ctx->cur_freq;
	const struct ddr_train_save_restore_t *train_reg;

	for (i = 0; i < s_train_max_index; i++) {
		train_reg = get_train_save_restore_regs(i);
		ddr_ctx->ddr_train_save_value[freq][i] =
				ddr_reg_rd(train_reg->reg_save);
	}
}

static void ddr_train_restore_configuration(struct ab_ddr_context *ddr_ctx,
					enum ddr_freq_t freq, int restore_mode)
{
	int idx;
	const struct ddr_train_save_restore_t *train_reg;
	uint32_t *ddr_train_save_value =
		ddr_ctx->ddr_train_save_value[freq];

	/* Training related setting to be applied before Restore */
	ddr_reg_clr_set(DPHY_CAL_CON1, RDLVL_PASS_ADJ_MSK,
			RDLVL_PASS_ADJ(0x4));
	ddr_reg_clr_set(DPHY_CAL_CON1, GLVL_PERIODIC_INCR_ADJ_MSK,
			GLVL_PERIODIC_INCR_ADJ(0x40));
	ddr_reg_clr_set(DPHY_CAL_CON4, GLVL_PERIODIC_FINE_INCR_ADJ_MSK,
			GLVL_PERIODIC_FINE_INCR_ADJ(0x0));
	ddr_reg_clr_set(DPHY_CAL_CON1, RDLVL_PERIODIC_INCR_ADJ_MSK,
			RDLVL_PERIODIC_INCR_ADJ(0x1));

	ddr_reg_clr_set(DPHY2_CAL_CON1, RDLVL_PASS_ADJ_MSK,
			RDLVL_PASS_ADJ(0x4));
	ddr_reg_clr_set(DPHY2_CAL_CON1, GLVL_PERIODIC_INCR_ADJ_MSK,
			GLVL_PERIODIC_INCR_ADJ(0x40));
	ddr_reg_clr_set(DPHY2_CAL_CON4, GLVL_PERIODIC_FINE_INCR_ADJ_MSK,
			GLVL_PERIODIC_FINE_INCR_ADJ(0x0));
	ddr_reg_clr_set(DPHY2_CAL_CON1, RDLVL_PERIODIC_INCR_ADJ_MSK,
			RDLVL_PERIODIC_INCR_ADJ(0x1));

	/* Restore command bus training result */
	ddr_reg_set(DPHY_CAL_CON0, CA_CAL_MODE);
	ddr_reg_set(DPHY2_CAL_CON0, CA_CAL_MODE);

	ddr_reg_wr(DPHY_CA_DESKEW_CON0,
		   ddr_train_save_value[s_DPHY_CA_DESKEW_CON0]);
	ddr_reg_wr(DPHY_CA_DESKEW_CON1,
		   ddr_train_save_value[s_DPHY_CA_DESKEW_CON1]);
	ddr_reg_wr(DPHY_CA_DESKEW_CON2,
		   ddr_train_save_value[s_DPHY_CA_DESKEW_CON2]);

	ddr_reg_wr(DPHY2_CA_DESKEW_CON0,
		   ddr_train_save_value[s_DPHY2_CA_DESKEW_CON0]);
	ddr_reg_wr(DPHY2_CA_DESKEW_CON1,
		   ddr_train_save_value[s_DPHY2_CA_DESKEW_CON1]);
	ddr_reg_wr(DPHY2_CA_DESKEW_CON2,
		   ddr_train_save_value[s_DPHY2_CA_DESKEW_CON2]);

	ddr_reg_clr(DPHY_CAL_CON0, CA_CAL_MODE);
	ddr_reg_clr(DPHY2_CAL_CON0, CA_CAL_MODE);

	if (restore_mode == AB_RESTORE_FULL) {

		/* Auto DQS clean / Gate training */
		ddr_autodqs_clean_gate_training(freq);

		ddr_reg_wr(DPHY_CAL_RD_PATTERN_CON0, PATTERN_55AA55AA);
		ddr_reg_wr(DPHY2_CAL_RD_PATTERN_CON0, PATTERN_55AA55AA);

		ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON0, PATTERN_55AA55AA);
		ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON1, PATTERN_55AA55AA);
		ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON2, PATTERN_55AA55AA);
		ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON3, PATTERN_55AA55AA);
		ddr_reg_wr(DPHY_CAL_WR_PATTERN_CON4, PATTERN_5555);
		ddr_reg_wr(DREX_WRTRA_PATTERN0, PATTERN_AA55AA55);
		ddr_reg_wr(DREX_WRTRA_PATTERN1, PATTERN_AA55AA55);
		ddr_reg_wr(DREX_WRTRA_PATTERN2, PATTERN_5555);

		ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON0, PATTERN_55AA55AA);
		ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON1, PATTERN_55AA55AA);
		ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON2, PATTERN_55AA55AA);
		ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON3, PATTERN_55AA55AA);
		ddr_reg_wr(DPHY2_CAL_WR_PATTERN_CON4, PATTERN_5555);
		ddr_reg_wr(DREX_WRTRA_PATTERN0, PATTERN_AA55AA55);
		ddr_reg_wr(DREX_WRTRA_PATTERN1, PATTERN_AA55AA55);
		ddr_reg_wr(DREX_WRTRA_PATTERN2, PATTERN_5555);

		/* Restore Read Training Results */
		ddr_reg_set(DPHY_CAL_CON0, RD_CAL_MODE);
		ddr_reg_set(DPHY_CAL_CON3, RD_SW_MODE);
		ddr_reg_set(DPHY2_CAL_CON0, RD_CAL_MODE);
		ddr_reg_set(DPHY2_CAL_CON3, RD_SW_MODE);

		for (idx = s_DPHY_RD_DESKEW_CENTER_CS0_CON_DM;
			idx <= s_DPHY2_RD_DQS_VWML_CS1_CON0; idx++) {
			train_reg = get_train_save_restore_regs(idx);
			ddr_reg_wr(train_reg->reg_restore,
				   ddr_train_save_value[idx]);
		}

		ddr_reg_clr(DPHY_CAL_CON3, RD_SW_MODE);
		ddr_reg_clr(DPHY2_CAL_CON3, RD_SW_MODE);

		/* Restore Write Training Results */
		ddr_reg_set(DPHY_CAL_CON0, WR_CAL_MODE);
		ddr_reg_set(DPHY_CAL_CON3, WR_SW_MODE);
		ddr_reg_set(DPHY2_CAL_CON0, WR_CAL_MODE);
		ddr_reg_set(DPHY2_CAL_CON3, WR_SW_MODE);

		for (idx = s_DPHY_WR_DESKEWC_CS0_CON0;
			idx <= s_DPHY2_DM_DESKEWL_CS1_CON0; idx++) {
			train_reg = get_train_save_restore_regs(idx);
			ddr_reg_wr(train_reg->reg_restore,
				   ddr_train_save_value[idx]);
		}

		ddr_reg_clr(DPHY_CAL_CON3, WR_SW_MODE);
		ddr_reg_clr(DPHY2_CAL_CON3, WR_SW_MODE);
	}

	/* Restore PRBS training result */
	ddr_reg_set(DPHY_CAL_CON3, PRBS_SW_MODE);
	ddr_reg_wr(DPHY_PRBS_CON4, ddr_train_save_value[s_DPHY_PRBS_CON2]);
	ddr_reg_wr(DPHY_PRBS_CON5, ddr_train_save_value[s_DPHY_PRBS_CON3]);
	ddr_reg_clr(DPHY_CAL_CON3, PRBS_SW_MODE);

	ddr_reg_set(DPHY2_CAL_CON3, PRBS_SW_MODE);
	ddr_reg_wr(DPHY2_PRBS_CON4, ddr_train_save_value[s_DPHY2_PRBS_CON2]);
	ddr_reg_wr(DPHY2_PRBS_CON5, ddr_train_save_value[s_DPHY2_PRBS_CON3]);
	ddr_reg_clr(DPHY2_CAL_CON3, PRBS_SW_MODE);

	ddr_reg_wr(DPHY_ZQ_CON9, ddr_train_save_value[s_DPHY_ZQ_CON9]);
	ddr_reg_wr(DPHY2_ZQ_CON9, ddr_train_save_value[s_DPHY2_ZQ_CON9]);
}

static int ab_ddr_initial_trainings(struct ab_ddr_context *ddr_ctx)
{
	ddr_prepare_training(ddr_ctx->cur_freq);

	ddr_ca_training();

	if (ddr_odt_training()) {
		pr_err("%s: ddr on-die-termination training failed.\n",
			__func__);
		goto ddr_init_train_fail;
	}

	ddr_autodqs_clean_gate_training(ddr_ctx->cur_freq);

	if (ddr_read_dq_calibration()) {
		pr_err("%s: ddr read dq calibration failed.\n", __func__);
		goto ddr_init_train_fail;
	}

	if (ddr_write_dq_calibration()) {
		pr_err("%s: ddr write dq calibration failed.\n", __func__);
		goto ddr_init_train_fail;
	}

	/* reset the prbs_dram_act_enable */
	ddr_reg_clr(DPHY_PRBS_CON8, PRBS_DRAM_ACT_ENABLE);
	ddr_reg_clr(DPHY2_PRBS_CON8, PRBS_DRAM_ACT_ENABLE);

	ddr_prbs_training_init();

	if (ddr_prbs_training_read()) {
		pr_err("%s: ddr prbs read training failed.\n", __func__);
		goto ddr_init_train_fail;
	}

	if (ddr_prbs_training_write()) {
		pr_err("%s: ddr prbs write training failed.\n", __func__);
		goto ddr_init_train_fail;
	}

	/* Set the prbs_dram_act_enable */
	ddr_reg_set(DPHY_PRBS_CON8, PRBS_DRAM_ACT_ENABLE);
	ddr_reg_set(DPHY2_PRBS_CON8, PRBS_DRAM_ACT_ENABLE);

	return DDR_SUCCESS;

ddr_init_train_fail:
	pr_err("Error!!! ddr initial training sequence failed\n");
	return DDR_FAIL;
}

static int ab_ddr_train(struct ab_ddr_context *ddr_ctx, uint32_t ddr_sr)
{
	/* Check ddr_reg_rd(TRN_ADDR) value and if non zero, the previous
	 * training results are stored in here. Restore the Training parameters.
	 */
	if (GET_REG_TRN_ADDR()) {

		if (ddr_enter_self_refresh_mode())
			goto ddr_train_fail;

		/* Copy training results from TRN_ADDR to PHY/DRAM */
		ddr_train_restore_configuration(ddr_ctx,
						AB_DRAM_FREQ_MHZ_1866,
						AB_RESTORE_FULL);

		if (ddr_exit_self_refresh_mode())
			goto ddr_train_fail;

		ddr_axi_enable_after_all_training();

	} else {

		if (!ddr_sr) {
			if (ddr_enter_self_refresh_mode())
				goto ddr_train_fail;
		}

		if (ddr_set_pll_freq(AB_DRAM_FREQ_MHZ_1866)) {
			pr_err("%s: setting pll freq [1866MHz] failed.\n",
				__func__);
			goto ddr_train_fail;
		}

		/* for CKE High to CS delay */
		udelay(DDR_INIT_CKE2CS_DELAY_USEC);

		if (ddr_enable_dll()) {
			pr_err("%s: enable dll failed.\n", __func__);
			goto ddr_train_fail;
		}

		ddr_power_down_exit_sequence();

		ddr_set_drex_timing_parameters(ddr_ctx->cur_freq);

		if (ddr_set_drex_address_parameters(ddr_ctx->cur_freq)) {
			pr_err("%s: drex address parameter settings failed.\n",
				__func__);
			goto ddr_train_fail;
		}

		if (ab_ddr_initial_trainings(ddr_ctx)) {
			pr_err("%s: initial training failed.\n", __func__);
			goto ddr_train_fail;
		}

		if (!ddr_sr) {
			if (ddrphy_run_vref_training(ddr_ctx)) {
				pr_err("%s: vref training failed.\n", __func__);
				goto ddr_train_fail;
			}

			if (ab_ddr_initial_trainings(ddr_ctx)) {
				pr_err("%s: re-initial training failed.\n",
						__func__);
				goto ddr_train_fail;
			}
		}

		if (ddr_exit_self_refresh_mode())
			goto ddr_train_fail;

		ddr_axi_enable_after_all_training();

		/* Save training results to local array */
		ddr_train_save_configuration(ddr_ctx);
		ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_1866] = 1;
	}

	return DDR_SUCCESS;

ddr_train_fail:
	pr_err("%s: error!! ddr training failed\n", __func__);
	return DDR_FAIL;
}

static int32_t ab_ddr_init_internal_isolation(struct ab_ddr_context *ddr_ctx)
{
	uint32_t ddr_sr;

	ddr_sr = GPIO_DDR_SR();

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

	ddr_initialize_phy(ddr_ctx->cur_freq);

	/* Initialize DFI Interface */
	if (ddr_initialize_dfi(1)) {
		pr_err("%s: DFI initialization failed\n", __func__);
		goto ddr_init_fail;
	}

	if (ddr_sr) { /* SUSPEND to ACTIVE sequence */

		/* IO Initialization for suspend */
		if (ddr_io_initialization(1)) {
			pr_err("%s: ddr IO init from suspend failed\n",
				__func__);
			goto ddr_init_fail;
		}
	} else { /* OFF to ACTIVE sequence */

		/* DRAM reset sequence */
		ddr_dram_reset_sequence();

		/* Power-down exit sequence */
		ddr_power_down_exit_sequence();

		/* IO Initialization */
		if (ddr_io_initialization(0)) {
			pr_err("%s: ddr IO initialization failed\n", __func__);
			goto ddr_init_fail;
		}

		/* MRWs (Set VREF, ODT, etc) */
		ddr_mrw_set_vref_odt_etc(AB_DRAM_FREQ_MHZ_1866);

		/* DRAM ZQ Calibration */
		ddr_dram_zq_calibration();
	}

	/* DDR training */
	if (ab_ddr_train(ddr_ctx, ddr_sr))
		goto ddr_init_fail;

	/* Now M0 can enter WFI for GPIO_DDR_TRAIN or REG_DDR_TRAIN */
	return DDR_SUCCESS;

ddr_init_fail:
	pr_err("%s: error!! ddr initialization failed\n", __func__);
	return DDR_FAIL;
}

static int32_t ab_ddr_init_freq_change(struct ab_ddr_context *ddr_ctx)
{
	/* Initialize PHY */
	ddr_initialize_phy_pre();

	ddr_initialize_phy(ddr_ctx->cur_freq);

	/* Initialize DFI Interface */
	if (ddr_initialize_dfi(0))
		goto ddr_init_fail;

	/* Power-down exit sequence */
	ddr_power_down_exit_sequence();

	/* configure MRWs (Set VREF, ODT, etc) as per the frequency */
	ddr_mrw_set_vref_odt_etc(ddr_ctx->cur_freq);

	/* Enable DLL */
	if (ddr_enable_dll()) {
		pr_err("%s: enable dll failed.\n", __func__);
		goto ddr_init_fail;
	}

	ddr_set_drex_timing_parameters(ddr_ctx->cur_freq);

	if (ddr_set_drex_address_parameters(ddr_ctx->cur_freq))
		goto ddr_init_fail;

	return DDR_SUCCESS;

ddr_init_fail:
	pr_err("%s: Error!! init frequency change failed.\n", __func__);
	return DDR_FAIL;
}

static int ddr_set_mif_freq(void *ctx, enum ddr_freq_t freq)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (ddr_ctx->cur_freq == freq)
		return DDR_SUCCESS;

	ddr_ctx->cur_freq = freq;

#ifdef CONFIG_DDR_BOOT_TEST
	ab_ddr_read_write_test(ctx, DDR_TEST_PCIE_DMA_WRITE(512));
#endif
	/* Block AXI Before entering self-refresh */
	if (ddr_block_axi_transactions())
		return -ETIMEDOUT;

	/* Disable PB Refresh & Auto Refresh */
	ddr_reg_clr(DREX_MEMCONTROL, PB_REF_EN);
	ddr_reg_clr(DREX_DFIRSTCONTROL, PB_WA_EN);
	ddr_reg_clr(DREX_CONCONTROL, AREF_EN);

	/* safe to send a manual all bank refresh command */
	ddr_reg_wr(DREX_DIRECTCMD, 0x5000000);

	/* Self-refresh entry sequence */
	if (ddr_enter_self_refresh_mode()) {
		pr_err("%s: self-refresh entry failed\n", __func__);
		return DDR_FAIL;
	}

	/* Configure the PLL for the required frequency */
	if (ddr_set_pll_freq(freq)) {
		pr_err("%s: pll setting failed\n", __func__);
		return DDR_FAIL;
	}

	/* DDR Init flow (reduced) without training */
	if (ab_ddr_init_freq_change(ddr_ctx)) {
		pr_err("%s: frequency switch failed\n", __func__);
		return DDR_FAIL;
	}

	/* The below settings were taken care duing different training
	 * sequences like io initialization, odt, zq calibration during normal
	 * ddr initialization and training. As part of frequency change, we are
	 * not executing above sequences. So, the below settings are required
	 * for normal ddr operation.
	 */
	ddr_reg_clr(DPHY_LP_CON0, CTRL_PULLD_DQ | CTRL_PULLD_DQS);
	ddr_reg_clr(DPHY_ZQ_CON0, ZQ_CLK_DIV_EN);
	ddr_reg_clr_set(DPHY_ZQ_CON0, ZQ_MODE_TERM_MSK,
			ZQ_MODE_TERM(ZQ_MODE_TERM_60_OHM));

	ddr_reg_clr(DPHY2_LP_CON0, CTRL_PULLD_DQ | CTRL_PULLD_DQS);
	ddr_reg_clr(DPHY2_ZQ_CON0, ZQ_CLK_DIV_EN);
	ddr_reg_clr_set(DPHY2_ZQ_CON0, ZQ_MODE_TERM_MSK,
			ZQ_MODE_TERM(ZQ_MODE_TERM_60_OHM));

	if (ddr_ctx->ddr_train_completed[freq]) {
		ddr_train_restore_configuration(ddr_ctx, freq, AB_RESTORE_FULL);
	} else {
		/* RESTORE the training results of CBT, PRBS and VREF training
		 * of 1866MHz from SRAM location
		 */
		ddr_train_restore_configuration(ddr_ctx, AB_DRAM_FREQ_MHZ_1866,
						AB_RESTORE_DVFS);

		/* Prepare Training */
		ddr_prepare_training(freq);

		/* autodqs clean gate training for low freq */
		ddr_autodqs_clean_gate_training(freq);

		/* Read & Write dq calibration for modified frequency */
		if (ddr_read_dq_calibration()) {
			pr_err("Error!! read dq calibration failed\n");
			goto freq_change_fail;
		}

		if (ddr_write_dq_calibration()) {
			pr_err("Error!! write dq calibration failed\n");
			goto freq_change_fail;
		}
	}

	/* Self-refresh exit sequence */
	if (ddr_exit_self_refresh_mode()) {
		pr_err("%s: self-refresh exit failed\n", __func__);
		goto freq_change_fail;
	}

	ddr_axi_enable_after_all_training();

	/* save the training results to local array */
	if (!ddr_ctx->ddr_train_completed[freq])
		ddr_train_save_configuration(ddr_ctx);

	ddr_ctx->ddr_train_completed[freq] = 1;

#ifdef CONFIG_DDR_BOOT_TEST
	/* Run MEMTESTER for the data integrity */
	ab_ddr_read_write_test(ctx, DDR_TEST_PCIE_DMA_READ(512));
#endif
	return DDR_SUCCESS;

freq_change_fail:
	pr_err("%s: ddr frequency switch failed!!\n", __func__);
	return DDR_FAIL;
}

/* Perform ddr re-training or train result restore */
int32_t ab_ddr_train_gpio(void *ctx)
{
	uint32_t ddr_sr;
	unsigned long timeout;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	ddr_sr = GPIO_DDR_SR();

	timeout = jiffies + usecs_to_jiffies(DDR_CLK_IN_SENSE_TIMEOUT);
	while (GPIO_CKE_IN_SENSE() && time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	if (GPIO_CKE_IN_SENSE()) {
		pr_err("error!! CLK_IN_SENSE transition timeout\n");
		return DDR_FAIL;
	}

	/* self-refresh exit sequence */
	if (ddr_exit_self_refresh_mode()) {
		pr_err("%s: self-refresh exit failed\n", __func__);
		return DDR_FAIL;
	}

	if (ab_ddr_train(ddr_ctx, ddr_sr)) {
		pr_err("%s: ddr training failed\n", __func__);
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

/* Perform ddr re-training */
int32_t ab_ddr_train_sysreg(void *ctx)
{
	uint32_t ddr_sr;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("error: ddr setup is not called\n");
		return -EAGAIN;
	}

	/* Read the DDR_SR */
	ddr_sr = GPIO_DDR_SR();

	/* Set the train address to zero to make sure re-train happens instead
	 * of training restore
	 */
	SET_REG_TRN_ADDR(0);

	if (ab_ddr_train(ddr_ctx, ddr_sr)) {
		pr_err("train_sysreg: ddr training failed\n");
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

static int ddr_enable_power_features(void)
{
	/* Block AXI Before entering self-refresh */
	if (ddr_block_axi_transactions())
		return -ETIMEDOUT;

	/* Disable PB Refresh & Auto Refresh */
	ddr_reg_clr(DREX_MEMCONTROL, PB_REF_EN);
	ddr_reg_clr(DREX_DFIRSTCONTROL, PB_WA_EN);
	ddr_reg_clr(DREX_CONCONTROL, AREF_EN);

	/* safe to send a manual all bank refresh command */
	ddr_reg_wr(DREX_DIRECTCMD, 0x5000000);

	/* Self-refresh entry sequence */
	if (ddr_enter_self_refresh_mode()) {
		pr_err("self-refresh entry fail during pwr features enable\n");
		ddr_reg_set(DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);
		return -ETIMEDOUT;
	}

	/* Set MR13 register to control below fields for power optimization
	 * VRCG (VREF Current Generator) OP[3]
	 *       0B: Normal Operation (default)
	 *       1B: VREF Fast Response (high current) mode 3
	 * RRO Refresh rate option OP[4]
	 *       0B: Disable codes 001 and 010 in MR4 OP[2:0]
	 *       1B: Enable all codes in MR4 OP[2:0]
	 */
	ddr_reg_wr(DREX_DIRECTCMD, MRW13_DEFAULT);

	/* Set the all-bank and per-bank auto refresh timings */
	ddr_reg_wr(DREX_TIMINGARE, T_REFI_DEFAULT | T_REFIPB_DEFAULT);

	/* Enabling DDR Power features */
	ddr_reg_set(DREX_CGCONTROL, PHY_CG_EN);
	ddr_reg_set(DPHY_LP_CON0, PCL_PD | MDLL_CG_EN | DS_IO_PD);
	ddr_reg_set(DPHY2_LP_CON0, PCL_PD | MDLL_CG_EN | DS_IO_PD);
	ddr_reg_clr_set(DREX_MEMCONTROL, DPWRDN_TYPE_MSK,
			DPWRDN_TYPE(FORCED_PRECHARGE_PD) | DPWRDN_EN);
	ddr_reg_set(DREX_MEMCONTROL, CLK_STOP_EN);

	/* Self-refresh exit sequence */
	if (ddr_exit_self_refresh_mode()) {
		pr_err("self-refresh exit fail during power features enable\n");
		ddr_reg_set(DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);
		return -ETIMEDOUT;
	}

	/* safe to send a manual all bank refresh command */
	ddr_reg_wr(DREX_DIRECTCMD, 0x5000000);

	/* Enable PB Refresh & Auto Refresh */
	ddr_reg_set(DREX_MEMCONTROL, PB_REF_EN);
	ddr_reg_set(DREX_DFIRSTCONTROL, PB_WA_EN);
	ddr_reg_set(DREX_CONCONTROL, AREF_EN);

	ddr_reg_set(DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);

	return 0;
}

static int __ab_ddr_wait_for_ddr_init(struct ab_ddr_context *ddr_ctx)
{
	unsigned long timeout;

	timeout = jiffies + AB_DDR_INIT_TIMEOUT;
	while ((!(ddr_reg_rd(REG_DDR_TRAIN_STATUS) & DDR_TRAIN_COMPLETE)) &&
			time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	/* check for the ddr training failure condition */
	if (ddr_reg_rd(REG_DDR_TRAIN_STATUS) & DDR_TRAIN_FAIL) {
		pr_err("%s: DDR Training failed during M0 boot\n",
		       __func__);
		return -EIO;
	}

	/* enable the ddr power features */
	if (ddr_enable_power_features())
		return -ETIMEDOUT;

	/* In A0 BootROM Auto Refresh setting is missing. To fix this issue,
	 * set the Auto-Refresh enable immediately after DDR init.
	 */
	if (ab_get_chip_id(ddr_ctx->ab_state_ctx) == CHIP_ID_A0)
		ddr_reg_set(DREX_CONCONTROL, AREF_EN);

	/* DDR training is completed as part of bootrom code execution.
	 * Copy the training results to the local array for future use.
	 */
	ddr_ctx->ddr_train_sram_location = ddr_reg_rd(SYSREG_REG_TRN_ADDR);
	ddr_copy_train_results_from_sram(ddr_ctx);

	/* Try to use latest training results from M0 resume sequence */
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_1866] = 1;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_1600] = 0;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_1200] = 0;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_933] = 0;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_800] = 0;

	/* set 1866MHz ddr clock during airbrush normal and resume boot */
	ddr_ctx->cur_freq = AB_DRAM_FREQ_MHZ_1866;

	return 0;
}

int ab_ddr_wait_for_ddr_init(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("wait_for_ddr_init: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	/* wait is only required when M0 performs ddr init */
	if (IS_HOST_DDR_INIT() || (ddr_ctx->ddr_state == DDR_SUSPEND))
		return 0;

	return __ab_ddr_wait_for_ddr_init(ddr_ctx);
}

static int32_t ab_ddr_resume(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	struct ab_state_context *sc = ddr_ctx->ab_state_ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("ddr_resume: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	/* Wait till the ddr init & training is completed */
	if (__ab_ddr_wait_for_ddr_init(ddr_ctx))
		return DDR_FAIL;

	if (IS_DDR_OTP_FLASHED() && (ab_get_chip_id(sc) == CHIP_ID_A0) &&
	    IS_M0_DDR_INIT() && !sc->alternate_boot) {

		/* IMPORTANT:
		 * ------------------------------------------------------------
		 * This path should not be HIT as there are high chances
		 * of data loss because of the below reasons. Please avoid
		 * the below combination.
		 *  [A0 SAMPLE + DDR OTP Flashed + NORMAL BOOT + M0_DDR_INIT]
		 *
		 * 1] In A0 BootROM, self-refresh exit sequence is called way
		 *    before Auto Refresh settings are enabled.
		 * 2] From  Suspend -> Resume scenario,
		 *    "AXI_Enable_After_All_training" sequence is skipped.
		 *
		 * Issue[1] can't be hadled. To fix Issue[2] call
		 * "AXI_Enable_After_All_training" sequence here.
		 */
		ddr_axi_enable_after_all_training();

		pr_err("%s: Error!! This path should not be used\n", __func__);
		WARN_ON(1);
		return DDR_FAIL;
	}

	/* Disable the DDR_SR GPIO */
	ab_gpio_disable_ddr_sr(sc);

#ifdef CONFIG_DDR_BOOT_TEST
	ab_ddr_read_write_test(ctx, DDR_TEST_PCIE_DMA_READ(512));
#endif
	return DDR_SUCCESS;
}

static int32_t ab_ddr_suspend(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	struct ab_state_context *sc = ddr_ctx->ab_state_ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("ddr_suspend: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	if (ddr_ctx->ddr_state == DDR_ON) {
#ifdef CONFIG_DDR_BOOT_TEST
		ab_ddr_read_write_test(ctx, DDR_TEST_PCIE_DMA_WRITE(512));
#endif
		/* Block AXI Before entering self-refresh */
		if (ddr_block_axi_transactions())
			return -ETIMEDOUT;

		/* Disable PB Refresh & Auto Refresh */
		ddr_reg_clr(DREX_MEMCONTROL, PB_REF_EN);
		ddr_reg_clr(DREX_DFIRSTCONTROL, PB_WA_EN);
		ddr_reg_clr(DREX_CONCONTROL, AREF_EN);

		/* safe to send a manual all bank refresh command */
		ddr_reg_wr(DREX_DIRECTCMD, 0x5000000);

		/* Self-refresh entry sequence */
		if (ddr_enter_self_refresh_mode())
			goto ddr_suspend_fail;

	} else if ((ddr_ctx->ddr_state == DDR_SLEEP) &&
		   (ddr_ctx->cur_freq != AB_DRAM_FREQ_MHZ_1866)) {

		/* This sections handles the case, when ddr suspend is requested
		 * from the sleep state. During sleep state, MIF clock is set to
		 * Oscillator. But to configure the MR settings, we need to move
		 * the MIF clock to PLL. Configuring MR registers may fail when
		 * MIF clock is set to lower frequencies like Oscillator clock.
		 */

		/* Move MIF clock to PLL to configure MR registers @1866MHz */
		if (ddr_set_pll_freq(ddr_ctx->cur_freq)) {
			pr_err("ddr suspend: mif pll config failed.\n");
			return DDR_FAIL;
		}
	}

	/* No need to configure MR registers if current frequency is already
	 * set to 1866MHz.
	 */
	if (ddr_ctx->cur_freq != AB_DRAM_FREQ_MHZ_1866) {
		/* MR Register configuration is not allowed when DRAM is in
		 * Power Down Entry mode. But during self-refresh entry command,
		 * DREX sends both Self-Refresh and Power Down Entry commands
		 * to DRAM.
		 *
		 * For the above reason, we need to bring the DRAM out of Power
		 * Down Entry mode.
		 */
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_21);

		/* During suspend -> resume (DDR_SR = 1), M0 bootrom will not
		 * update the MR registers specific to 1866MHz. As the ddr
		 * initialization happens at 1866MHz, make sure the MR registers
		 * are udpated for 1866MHz before entering to suspend state.
		 * Otherwise the read/write trainings will fail during ddr
		 * initialization (during suspend -> resume) in BootROM
		 */
		ddr_mrw_set_vref_odt_etc(AB_DRAM_FREQ_MHZ_1866);

		/* As we are going to suspend state, keep the DRAM in
		 * Power Down Entry Mode.
		 */
		ddr_reg_wr_otp(DREX_DIRECTCMD, o_DREX_DIRECTCMD_20);
	}

	/* Move the MIF clock to oscillator and switch off the PLL for
	 * better power savings.
	 */
	ddr_set_pll_to_oscillator();

	/* Enable the PMU Retention */
	PMU_CONTROL_PHY_RET_ON();

	/* Enable GPIOs to inform DDR is in suspend mode */
	ab_gpio_enable_ddr_sr(sc);
	ab_gpio_enable_ddr_iso(sc);

	/* Airbrush will resume with 1866MHz */
	ddr_ctx->cur_freq = AB_DRAM_FREQ_MHZ_1866;

	return DDR_SUCCESS;

ddr_suspend_fail:
	return DDR_FAIL;
}

int32_t ab_ddr_selfrefresh_exit(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("SRX: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	/* Config MIF_PLL and move the MIF clock to PLL output */
	if (ddr_set_pll_freq(ddr_ctx->cur_freq)) {
		pr_err("%s: mif pll config failed.\n", __func__);
		return DDR_FAIL;
	}

	/* Self-refresh exit sequence */
	if (ddr_exit_self_refresh_mode()) {
		pr_err("%s: self-refresh exit failed\n", __func__);
		return DDR_FAIL;
	}

	/* safe to send a manual all bank refresh command */
	ddr_reg_wr(DREX_DIRECTCMD, 0x5000000);

	/* Enable PB Refresh & Auto Refresh */
	ddr_reg_set(DREX_MEMCONTROL, PB_REF_EN);
	ddr_reg_set(DREX_DFIRSTCONTROL, PB_WA_EN);
	ddr_reg_set(DREX_CONCONTROL, AREF_EN);

	/* Allow AXI after exiting from self-refresh */
	ddr_reg_wr(DREX_ACTIVATE_AXI_READY, 0x1);

#ifdef CONFIG_DDR_BOOT_TEST
	ab_ddr_read_write_test(ctx, DDR_TEST_PCIE_DMA_READ(512));
#endif
	return DDR_SUCCESS;
}

int32_t ab_ddr_selfrefresh_enter(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("SRE: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

#ifdef CONFIG_DDR_BOOT_TEST
	ab_ddr_read_write_test(ctx, DDR_TEST_PCIE_DMA_WRITE(512));
#endif
	/* Block AXI Before entering self-refresh */
	if (ddr_block_axi_transactions())
		return -ETIMEDOUT;

	/* Disable PB Refresh & Auto Refresh */
	ddr_reg_clr(DREX_MEMCONTROL, PB_REF_EN);
	ddr_reg_clr(DREX_DFIRSTCONTROL, PB_WA_EN);
	ddr_reg_clr(DREX_CONCONTROL, AREF_EN);

	/* safe to send a manual all bank refresh command */
	ddr_reg_wr(DREX_DIRECTCMD, 0x5000000);

	/* Self-refresh entry sequence */
	if (ddr_enter_self_refresh_mode()) {
		pr_err("%s: self-refresh entry failed\n", __func__);
		return DDR_FAIL;
	}

	/* Move the MIF clock to oscillator */
	ddr_set_pll_to_oscillator();

	return DDR_SUCCESS;
}

static int ab_ddr_get_freq(void *ctx, u64 *val)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	u64 freq = 0;
	int ret = DDR_SUCCESS;

	if (!ddr_ctx->is_setup_done) {
		pr_err("get_freq: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	switch (ddr_ctx->cur_freq) {
	case AB_DRAM_FREQ_MHZ_1866:
		freq = DRAM_CLK_1866MHZ;
		break;
	case AB_DRAM_FREQ_MHZ_1600:
		freq = DRAM_CLK_1600MHZ;
		break;
	case AB_DRAM_FREQ_MHZ_1200:
		freq = DRAM_CLK_1200MHZ;
		break;
	case AB_DRAM_FREQ_MHZ_933:
		freq = DRAM_CLK_933MHZ;
		break;
	case AB_DRAM_FREQ_MHZ_800:
		freq = DRAM_CLK_800MHZ;
		break;
	default:
		ret = DDR_FAIL;
		break;
	}

	if ((ddr_ctx->ddr_state == DDR_SLEEP) ||
	    (ddr_ctx->ddr_state == DDR_SUSPEND)) {
		freq = DRAM_CLK_OSC;
	} else if (ddr_ctx->ddr_state == DDR_OFF) {
		freq = 0;
	}

	*val = freq;

	return ret;
}

static int ab_ddr_set_freq(void *ctx, u64 val)
{
	int ret = DDR_FAIL;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("set_freq: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	switch (val) {
	case DRAM_CLK_1866MHZ:
		ret = ddr_set_mif_freq(ctx, AB_DRAM_FREQ_MHZ_1866);
		break;
	case DRAM_CLK_1600MHZ:
		ret = ddr_set_mif_freq(ctx, AB_DRAM_FREQ_MHZ_1600);
		break;
	case DRAM_CLK_1200MHZ:
		ret = ddr_set_mif_freq(ctx, AB_DRAM_FREQ_MHZ_1200);
		break;
	case DRAM_CLK_933MHZ:
		ret = ddr_set_mif_freq(ctx, AB_DRAM_FREQ_MHZ_933);
		break;
	case DRAM_CLK_800MHZ:
	case DRAM_CLK_200MHZ:
		ret = ddr_set_mif_freq(ctx, AB_DRAM_FREQ_MHZ_800);
		break;
	default:
		break;
	}

	return ret;
}

static int ab_ddr_set_state(const struct block_property *prop_from,
			const struct block_property *prop_to,
			enum block_state block_state_id, void *data)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)data;
	unsigned long old_rate;
	unsigned long new_rate;
	unsigned long extra_pre_notify_flag = 0;
	unsigned long extra_post_notify_flag = 0;

	if (!ddr_ctx->is_setup_done) {
		pr_err("set_state: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	if (!prop_from || !prop_to)
		return -EINVAL;

	/*
	 * If DRAM is going to enter block_state_0 (power off), data will
	 * be lost.  Notify subscribers in advance both pre-rate and post-rate.
	 */
	if (block_state_id == BLOCK_STATE_0) {
		extra_pre_notify_flag = AB_DRAM_DATA_PRE_OFF;
		extra_post_notify_flag = AB_DRAM_DATA_POST_OFF;
	}

	old_rate = prop_from->clk_frequency;
	new_rate = prop_to->clk_frequency;
	ab_sm_clk_notify(AB_DRAM_PRE_RATE_CHANGE | extra_pre_notify_flag,
			 old_rate, new_rate);

	switch (block_state_id) {
	case BLOCK_STATE_300 ... BLOCK_STATE_305:
		if (ddr_ctx->ddr_state == DDR_SLEEP)
			ab_ddr_selfrefresh_exit(ddr_ctx);
		else if (ddr_ctx->ddr_state == DDR_SUSPEND)
			ab_ddr_resume(ddr_ctx);

		ddr_ctx->prev_ddr_state = ddr_ctx->ddr_state;
		ddr_ctx->ddr_state = DDR_ON;
		break;

	case BLOCK_STATE_101:
		/* ddr sleep/deep-sleep functionality */
		if (ddr_ctx->ddr_state != DDR_ON) {
			ab_sm_clk_notify(AB_DRAM_ABORT_RATE_CHANGE,
					 old_rate, new_rate);
			return -EINVAL;
		}

		ab_ddr_selfrefresh_enter(ddr_ctx);

		ddr_ctx->prev_ddr_state = ddr_ctx->ddr_state;
		ddr_ctx->ddr_state = DDR_SLEEP;
		break;

	case BLOCK_STATE_100:
		/* ddr suspend functionality */
		if ((ddr_ctx->ddr_state == DDR_SUSPEND) ||
			(ddr_ctx->ddr_state == DDR_OFF)) {
			ab_sm_clk_notify(AB_DRAM_ABORT_RATE_CHANGE,
					 old_rate, new_rate);
			return -EINVAL;
		}

		ab_ddr_suspend(ddr_ctx);

		ddr_ctx->prev_ddr_state = ddr_ctx->ddr_state;
		ddr_ctx->ddr_state = DDR_SUSPEND;
		break;

	case BLOCK_STATE_0:
		if (ddr_ctx->ddr_state == DDR_SUSPEND) {
			ab_gpio_disable_ddr_sr(ddr_ctx->ab_state_ctx);
			ab_gpio_disable_ddr_iso(ddr_ctx->ab_state_ctx);
		}

		ddr_ctx->prev_ddr_state = ddr_ctx->ddr_state;
		ddr_ctx->ddr_state = DDR_OFF;
		break;

	default:
		break;
	}

	if (ddr_ctx->ddr_state == DDR_ON)
		ab_ddr_set_freq(ddr_ctx, prop_to->clk_frequency);

	ab_sm_clk_notify(AB_DRAM_POST_RATE_CHANGE | extra_post_notify_flag,
			 old_rate, new_rate);

	/* Based on the state, call the corresponding DDR functionality */
	return 0;
}

static int32_t ab_ddr_setup(void *ctx, void *ab_ctx)
{
#ifdef DEBUG
	int otp_idx;
#endif
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	struct ab_state_context *sc = (struct ab_state_context *)ab_ctx;

	if (!sc) {
		pr_err("ddr_setup: Invalid state context\n");
		return -EINVAL;
	}

	/* ab_ddr_setup is already called */
	if (ddr_ctx->is_setup_done)
		return DDR_SUCCESS;

	/* Incase of OTPs are flashed, get all information from OTPs.
	 * If ddr OTPs are not flashed, depend on dt properties
	 */
	if (IS_DDR_OTP_FLASHED()) {
		/* DDR OTPs are flashed, use OTP wrapper to read ddr OTPs*/
		dev_dbg(ddr_ctx->dev, "ddr_setup: DDR OTPs fused\n");
		ddr_otp_rd = &read_otp_wrapper;

		if (ab_get_chip_id(sc) == CHIP_ID_A0) {
			dev_info(ddr_ctx->dev, "Airbrush Revision: A0\n");
		} else if (ab_get_chip_id(sc) == CHIP_ID_B0) {
			dev_info(ddr_ctx->dev, "Airbrush Revision: B0\n");
		} else {
			/* add support for other revisions */
			pr_err("%s: Invalid version ID\n", __func__);
			WARN_ON(1);
		}
	} else {

		/* If DDR OTP is not flashed, then use the OTP array instead */
		dev_warn(ddr_ctx->dev,
			 "%s: DDR OTPs NOT fused. Use local array.\n",
			 __func__);
		ddr_otp_rd = &read_otp_array;
	}

#ifdef DEBUG
	for (otp_idx = 0; otp_idx < o_DDR_OTP_MAX; otp_idx++)
		dev_dbg(ddr_ctx->dev,
			"%d: 0x%x\n", otp_idx, ddr_otp_rd(otp_idx));
#endif

	/* Keeps track of setup call.
	 * Should be checked in all other dram_ops callbacks
	 */
	ddr_ctx->is_setup_done = 1;
	ddr_ctx->ab_state_ctx = sc;

	return DDR_SUCCESS;
}

static int32_t ab_ddr_init(void *ctx)
{
	int32_t ret = 0;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
#ifdef CONFIG_DDR_BOOT_TEST
	uint32_t ddr_sr;
#endif

	if (!ddr_ctx->is_setup_done) {
		pr_err("%s, error: ddr setup is not called", __func__);
		return -EAGAIN;
	}

	ddr_ctx->cur_freq = AB_DRAM_FREQ_MHZ_1866;

	ret = ab_ddr_init_internal_isolation(ddr_ctx);
	if (ret)
		pr_err("%s: error!! ddr initialization failed\n", __func__);

#ifdef CONFIG_DDR_BOOT_TEST
	/* Read the DDR_SR */
	ddr_sr = GPIO_DDR_SR();

	if (!ddr_sr && !ret)
		ab_ddr_read_write_test(ctx, DDR_TEST_PCIE_DMA_READ_WRITE(512));
#endif
	return ret;
}

static struct ab_sm_dram_ops dram_ops = {
	.ctx = NULL,

	.setup = &ab_ddr_setup,
	.wait_for_init = &ab_ddr_wait_for_ddr_init,
	.init = &ab_ddr_init,
	.get_freq = &ab_ddr_get_freq,
	.set_freq = &ab_ddr_set_freq,
	.suspend = &ab_ddr_suspend,
	.resume = &ab_ddr_resume,
	.sref_enter = &ab_ddr_selfrefresh_enter,
	.sref_exit = &ab_ddr_selfrefresh_exit,
	.rw_test = &ab_ddr_read_write_test,
	.eye_margin = &ab_ddr_eye_margin,
	.eye_margin_plot = &ab_ddr_eye_margin_plot,
	.ppc_set_event = &ab_ddr_ppc_set_event,
	.ppc_ctrl = &ab_ddr_ppc_ctrl,
};

static int ab_ddr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ab_ddr_context *ddr_ctx;

	ddr_ctx = kzalloc(sizeof(struct ab_ddr_context), GFP_KERNEL);
	if (ddr_ctx == NULL)
		return -ENOMEM;

	ddr_ctx->eye_data = vmalloc(sizeof(struct ddr_eyemargin_data));
	if (ddr_ctx->eye_data == NULL)
		pr_err("ab_ddr: memory alloc for eyemargin data failed\n");

	/* initialize ddr state to off */
	ddr_ctx->ddr_state = DDR_OFF;

	ddr_ctx->dev = dev;

	mutex_init(&ddr_ctx->ddr_lock);

	dram_ops.ctx = ddr_ctx;
	ab_sm_register_dram_ops(&dram_ops);

	/* Register the Airbrush State Manager (ASM) callback */
	ab_sm_register_blk_callback(DRAM, &ab_ddr_set_state, ddr_ctx);

	return 0;
}

static int ab_ddr_remove(struct platform_device *pdev)
{
	ab_sm_unregister_dram_ops();
	return 0;
}

static const struct of_device_id ab_ddr_of_match[] = {
		{ .compatible = "abc,airbrush-ddr", },
		{ },
};

static struct platform_driver ab_ddr_driver = {
	.probe = ab_ddr_probe,
	.remove = ab_ddr_remove,
	.driver = {
		.name = "airbrush-ddr",
		.of_match_table = ab_ddr_of_match,
	},
};
module_platform_driver(ab_ddr_driver);
