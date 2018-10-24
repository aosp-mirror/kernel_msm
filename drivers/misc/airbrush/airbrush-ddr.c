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
	unsigned long timeout;

	poll_multiplier = ddr_otp_rd(o_SECURE_JTAG2) + 1;

	timeout = jiffies +
		  usecs_to_jiffies(poll->usec_timeout * poll_multiplier);
	while (((ddr_reg_rd(reg) & poll->mask) != poll->val) &&
			time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	if ((ddr_reg_rd(reg) & poll->mask) != poll->val) {
		pr_err("%s, status poll failed for reg: 0x%x, idx: 0x%x\n",
		       __func__, reg, poll_idx);
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

static int ddr_config_cmu_mif_highfreq(void)
{
	ddr_reg_set(DREX_MEMCONTROL, CLK_STOP_EN);
	ddr_reg_clr(MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);
	ddr_reg_wr(PLL_CON0_PLL_PHY_MIF, 0x0);
	ddr_reg_wr_otp(PLL_CON0_PLL_PHY_MIF, o_Reserved_DDR_INIT_1);

	if (ddr_reg_poll(PLL_CON0_PLL_PHY_MIF, p_pll_con0_pll_phy_mif)) {
		pr_err("%s, mif pll lock failed\n", __func__);
		return DDR_FAIL;
	}

	ddr_reg_wr_otp(PLL_CON0_PLL_PHY_MIF, o_Reserved_DDR_INIT_2);
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

static void ddr_initialize_phy(void)
{
	/* mdll_monitor_en[15] will be set to 0x1 */
	ddr_reg_wr_otp(DPHY_MON_CON0, o_PCIe_reg_address_76);

	/* set ignore_dic[31] (ignore dfi_init_complete) &
	 * pclk_async PCLK Async
	 */
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

	/* DRAM DCTL Resync */
	ddr_dram_dctrl_resync();
}

static int ddr_set_drex_address_parameters(void)
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
	ddr_reg_wr_otp(DPHY_GNR_CON0, o_DPHY_GNR_CON0_2);
	ddr_reg_wr_otp(DPHY2_GNR_CON0, o_DPHY_GNR_CON0_2);
	ddr_dram_dctrl_resync();

	/* DRAM Density Setting */
	ddr_reg_wr_otp(DREX_MEMCONTROL, o_Reserved_DDR_INIT_10);
	ddr_reg_wr(DREX_ASP_MEMBASECONFIG0, CHUNK_START_END);
	ddr_reg_wr_otp(DREX_ASP_MEMCONFIG0, o_Reserved_DDR_INIT_11);
	ddr_reg_wr(DREX_ASP_CHIP0SIZECONFIG, CHIP_SIZE_512MB);

	return DDR_SUCCESS;
}

static void ddr_prepare_training(void)
{
	ddr_reg_clr(DPHY_OFFSETD_CON0, UPD_MODE);
	ddr_reg_wr_otp(DPHY_DVFS_CON, o_Reserved_DDR_INIT_5);
	ddr_reg_set(DPHY_MDLL_CON0, CLKM_CG_EN_SW);

	ddr_reg_clr(DPHY2_OFFSETD_CON0, UPD_MODE);
	ddr_reg_wr_otp(DPHY2_DVFS_CON, o_Reserved_DDR_INIT_5);
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

static void ddr_autodqs_clean_gate_training(void)
{
	ddr_reg_set(DPHY_CAL_CON4, GLITCH_REMOVAL_EN);
	ddr_reg_set(DPHY_GNR_CON0, CTRL_DFDQS);
	ddr_reg_clr_set(DPHY_TESTIRCV_CON0,
			DQS0_TESTIRCV_MSK | DQS1_TESTIRCV_MSK,
			DQS0_TESTIRCV(0x3) | DQS1_TESTIRCV(0x3));
	ddr_reg_clr(DPHY_CAL_CON0, GATE_CAL_MODE);
	ddr_reg_set(DPHY_CAL_CON3, AUTO_DQS_CLEAN);
	ddr_reg_clr(DPHY_CAL_CON2, CTRL_RODT_DISABLE);

	ddr_reg_set(DPHY2_CAL_CON4, GLITCH_REMOVAL_EN);
	ddr_reg_set(DPHY2_GNR_CON0, CTRL_DFDQS);
	ddr_reg_clr_set(DPHY2_TESTIRCV_CON0,
			DQS0_TESTIRCV_MSK | DQS1_TESTIRCV_MSK,
			DQS0_TESTIRCV(0x3) | DQS1_TESTIRCV(0x3));
	ddr_reg_clr(DPHY2_CAL_CON0, GATE_CAL_MODE);
	ddr_reg_set(DPHY2_CAL_CON3, AUTO_DQS_CLEAN);
	ddr_reg_clr(DPHY2_CAL_CON2, CTRL_RODT_DISABLE);
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

	/* DRAMDCTLResync:  start */
	ddr_dram_dctrl_resync();

	return DDR_SUCCESS;
}

static void ddr_axi_enable_after_all_training(void)
{
	ddr_reg_wr_otp(DREX_MEMCONTROL, o_Reserved_DDR_INIT_13);
	ddr_reg_wr_otp(DREX_MEMCONTROL, o_Reserved_DDR_INIT_14);
	ddr_reg_wr_otp(DREX_DFIRSTCONTROL, o_Reserved_DDR_INIT_15);
	ddr_reg_wr_otp(DREX_CONCONTROL, o_DREX_CONCONTROL_0);
	ddr_reg_set(DREX_CONCONTROL, AREF_EN);
	ddr_reg_set(DREX_ALL_INIT_INDI, ALL_INIT_DONE);
	ddr_reg_wr_otp(DREX_ACTIVATE_AXI_READY, o_Reserved_DDR_INIT_16);
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

static void ddr_train_save_configuration(struct ab_ddr_context *ddr_ctx)
{
	int i;
	const struct ddr_train_save_restore_t *train_reg;

	for (i = 0; i < s_train_max_index; i++) {
		train_reg = get_train_save_restore_regs(i);
		ddr_ctx->ddr_train_save_value[i] =
				ddr_reg_rd(train_reg->reg_save);
	}
}

static void ddr_train_restore_configuration(uint32_t *ddr_train_save_value)
{
	int idx;
	const struct ddr_train_save_restore_t *train_reg;

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

	/* Auto DQS clean / Gate training */
	ddr_autodqs_clean_gate_training();

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
		ddr_reg_wr(train_reg->reg_restore, ddr_train_save_value[idx]);
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
		ddr_reg_wr(train_reg->reg_restore, ddr_train_save_value[idx]);
	}

	ddr_reg_clr(DPHY_CAL_CON3, WR_SW_MODE);
	ddr_reg_clr(DPHY2_CAL_CON3, WR_SW_MODE);

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

	/* Enabling lock_value_init_override */
	ddr_reg_set(DPHY_MDLL_CON1, LOCK_VALUE_INIT_OVERRIDE);
	ddr_reg_set(DPHY2_MDLL_CON1, LOCK_VALUE_INIT_OVERRIDE);

	/* Restore ctrl_lock_value_init after restoring training result */
	ddr_reg_clr_set(DPHY_MDLL_CON1, CTRL_LOCK_VALUE_INIT_MSK,
		CTRL_LOCK_VALUE_INIT(ddr_train_save_value[s_DPHY_MDLL_CON1]));
	ddr_reg_clr_set(DPHY2_MDLL_CON1, CTRL_LOCK_VALUE_INIT_MSK,
		CTRL_LOCK_VALUE_INIT(ddr_train_save_value[s_DPHY2_MDLL_CON1]));

	/* Disabling lock_value_init_override */
	ddr_reg_clr(DPHY_MDLL_CON1, LOCK_VALUE_INIT_OVERRIDE);
	ddr_reg_clr(DPHY2_MDLL_CON1, LOCK_VALUE_INIT_OVERRIDE);
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

static int ab_ddr_initial_trainings(void)
{
	ddr_prepare_training();

	ddr_ca_training();

	if (ddr_odt_training()) {
		pr_err("%s: ddr on-die-termination training failed.\n",
			__func__);
		goto ddr_init_train_fail;
	}

	ddr_autodqs_clean_gate_training();

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
		ddr_train_restore_configuration(
				&ddr_ctx->ddr_train_save_value[0]);

		if (ddr_exit_self_refresh_mode())
			goto ddr_train_fail;

		ddr_axi_enable_after_all_training();

	} else {

		if (!ddr_sr) {
			if (ddr_enter_self_refresh_mode())
				goto ddr_train_fail;
		}

		/* configure PLL_MIF via CMU_MIF_for_Highfreq and wait for
		 * pll lock
		 */
		if (ddr_config_cmu_mif_highfreq())
			goto ddr_train_fail;

		/* for CKE High to CS delay */
		udelay(DDR_INIT_CKE2CS_DELAY_USEC);

		if (ddr_enable_dll()) {
			pr_err("%s: enable dll failed.\n", __func__);
			goto ddr_train_fail;
		}

		ddr_power_down_exit_sequence();

		ddr_set_drex_timing_parameters();

		if (ddr_set_drex_address_parameters()) {
			pr_err("%s: drex address parameter settings failed.\n",
				__func__);
			goto ddr_train_fail;
		}

		if (ab_ddr_initial_trainings()) {
			pr_err("%s: initial training failed.\n", __func__);
			goto ddr_train_fail;
		}

		if (!ddr_sr) {
			if (ddrphy_run_vref_training(ddr_ctx)) {
				pr_err("%s: vref training failed.\n", __func__);
				goto ddr_train_fail;
			}

			if (ab_ddr_initial_trainings()) {
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

	ddr_initialize_phy();

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
		ddr_mrw_set_vref_odt_etc();

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

/* Perform ddr re-training or train result restore */
int32_t ab_ddr_train_gpio(struct ab_state_context *sc)
{
	uint32_t ddr_sr;
	struct ab_ddr_context *ddr_ctx;
	unsigned long timeout;

	if (!sc || !sc->ddr_data) {
		pr_err("%s, error: ab_ddr_setup() is not called", __func__);
		return DDR_FAIL;
	}

	ddr_ctx = (struct ab_ddr_context *)sc->ddr_data;

	ddr_sr = GPIO_DDR_SR();

	timeout = jiffies + usecs_to_jiffies(DDR_CLK_IN_SENSE_TIMEOUT);
	while (GPIO_CKE_IN_SENSE() && time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	if (GPIO_CKE_IN_SENSE()) {
		pr_err("%s, error!! CLK_IN_SENSE transition timeout\n",
			__func__);
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
int32_t ab_ddr_train_sysreg(struct ab_state_context *sc)
{
	uint32_t ddr_sr;
	struct ab_ddr_context *ddr_ctx;

	if (!sc || !sc->ddr_data) {
		pr_err("%s, error: ab_ddr_setup() is not called", __func__);
		return DDR_FAIL;
	}

	ddr_ctx = (struct ab_ddr_context *)sc->ddr_data;

	/* Read the DDR_SR */
	ddr_sr = GPIO_DDR_SR();

	/* Set the train address to zero to make sure re-train happens instead
	 * of training restore
	 */
	SET_REG_TRN_ADDR(0);

	if (ab_ddr_train(ddr_ctx, ddr_sr)) {
		pr_err("%s: ddr training failed\n", __func__);
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
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
			pr_err("%s: self-refresh exit failed\n", __func__);
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
	if (ddr_enter_self_refresh_mode()) {
		pr_err("%s: self-refresh entry failed\n", __func__);
		goto ddr_suspend_fail;
	}

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
	if (ddr_exit_self_refresh_mode()) {
		pr_err("%s: self-refresh exit failed\n", __func__);
		return DDR_FAIL;
	}

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
	if (ddr_enter_self_refresh_mode()) {
		pr_err("%s: self-refresh entry failed\n", __func__);
		return DDR_FAIL;
	}

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

	if (!sc || !sc->pdev) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -EFAULT;
	}

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
	uint32_t ddr_sr;
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
	ddr_sr = GPIO_DDR_SR();

	if (!ddr_sr && !ret)
		ab_ddr_read_write_test(DDR_BOOT_TEST_READ_WRITE);
#endif
	return ret;
}
EXPORT_SYMBOL(ab_ddr_init);
