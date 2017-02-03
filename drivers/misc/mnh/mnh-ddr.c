/*
*
* MNH DDR Driver
* Copyright (c) 2016, Intel Corporation.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
*/

#include "mnh-hwio.h"
#include "mnh-hwio-bases.h"
#include "mnh-hwio-ddr-ctl.h"
#include "mnh-hwio-ddr-pi.h"
#include "mnh-hwio-ddr-phy.h"
#include "mnh-hwio-scu.h"
#include "mnh-ddr.h"
#include "mnh-pcie.h"

#define MNH_DDR_CTL_IN(reg) \
	HW_IN(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, reg)
#define MNH_DDR_CTL_INf(reg, fld) \
	HW_INf(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, reg, fld)
#define MNH_DDR_CTL_OUTf(reg, fld, val) \
	HW_OUTf(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, reg, fld, val)

#define MNH_DDR_PI_INf(reg, fld) \
	HW_INf(HWIO_DDR_PI_BASE_ADDR, DDR_PI, reg, fld)
#define MNH_DDR_PI_OUTf(reg, fld, val) \
	HW_OUTf(HWIO_DDR_PI_BASE_ADDR, DDR_PI, reg, fld, val)
#define MNH_DDR_PI_OUT(reg, val) \
	HW_OUT(HWIO_DDR_PI_BASE_ADDR, DDR_PI, reg, val)

#define MNH_DDR_PHY_INf(reg, fld) \
	HW_INf(HWIO_DDR_PHY_BASE_ADDR, DDR_PHY, reg, fld)
#define MNH_DDR_PHY_OUTf(reg, fld, val) \
	HW_OUTf(HWIO_DDR_PHY_BASE_ADDR, DDR_PHY, reg, fld, val)
#define MNH_DDR_PHY_OUT(reg, val) \
	HW_OUT(HWIO_DDR_PHY_BASE_ADDR, DDR_PHY, reg, val)

#define MNH_SCU_INf(reg, fld) \
	HW_INf(HWIO_SCU_BASE_ADDR, SCU, reg, fld)
#define MNH_SCU_OUTf(reg, fld, val) \
	HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, reg, fld, val)
#define MNH_SCU_OUT(reg, val) \
	HW_OUT(HWIO_SCU_BASE_ADDR, SCU, reg, val)
#define MNH_SCU_OUTx(reg, inst, val) \
	HW_OUTx(HWIO_SCU_BASE_ADDR, SCU, reg, inst, val)

#define MNH_RSTC_INf(fld) \
	HW_INf(HWIO_SCU_BASE_ADDR, SCU, RSTC, fld)
#define MNH_RSTC_OUTf(fld, val) \
	HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, RSTC, fld, val)

#define WRITE_DDR_REG_CONFIG(ddrblock, regindex) \
	mnh_reg_write(state->bases.ddrblock##_base + (regindex * sizeof(u32)), \
		      state->config->ddrblock[regindex])

int mnh_ddr_po_init(struct device *dev,
		    struct mnh_ddr_state *state)
{
	int index;
	int i;

	dev_info(dev, "%s start.", __func__);

	/* MNH_PLL_PASSCODE_SET */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x4CD9);

	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 1);
	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, PD, 0);
	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FOUTPOSTDIVPD, 0);
	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 0);

	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LP4_FSP_SW_OVERRIDE, 0);

	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, 0, state->fsps[0]);
	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, 1, state->fsps[1]);
	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, 2, state->fsps[2]);
	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, 3, state->fsps[3]);

	/* MNH_PLL_PASSCODE_CLR */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x0);

	for (index = 0; index < MNH_DDR_NUM_CTL_REG; index++)
		WRITE_DDR_REG_CONFIG(ctl, index);

	for (index = 0; index < MNH_DDR_NUM_PI_REG; index++)
		WRITE_DDR_REG_CONFIG(pi, index);

#ifndef MNH_DDR_DO_FSP
	MNH_DDR_PHY_OUTf(1024, PHY_FREQ_SEL, 2);
	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_MULTICAST_EN, 1);
#else
	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_MULTICAST_EN, 1);
	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 0);
#endif
	for (index = 0; index < MNH_DDR_NUM_PHY_REG; index++)
		WRITE_DDR_REG_CONFIG(phy, index);

#ifdef MNH_DDR_DO_FSP
	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_MULTICAST_EN, 0);
	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 1);

	/* a */
	MNH_DDR_PHY_OUTf(83, PHY_RDDQS_LATENCY_ADJUST_0, 0x03);
	MNH_DDR_PHY_OUTf(83, PHY_RDDQS_GATE_SLAVE_DELAY_0, 0x011a);
	MNH_DDR_PHY_OUTf(85, PHY_GTLVL_LAT_ADJ_START_0, 0x01);
	MNH_DDR_PHY_OUTf(90, PHY_RDDATA_EN_TSEL_DLY_0, 0x02);
	MNH_DDR_PHY_OUTf(90, PHY_RDDATA_EN_DLY_0, 0x03);
	MNH_DDR_PHY_OUTf(92, PHY_RPTR_UPDATE_0, 0x07);

	MNH_DDR_PHY_OUTf(211, PHY_RDDQS_LATENCY_ADJUST_1, 0x03);
	MNH_DDR_PHY_OUTf(211, PHY_RDDQS_GATE_SLAVE_DELAY_1, 0x011a);
	MNH_DDR_PHY_OUTf(213, PHY_GTLVL_LAT_ADJ_START_1, 0x01);
	MNH_DDR_PHY_OUTf(218, PHY_RDDATA_EN_TSEL_DLY_1, 0x02);
	MNH_DDR_PHY_OUTf(218, PHY_RDDATA_EN_DLY_1, 0x03);
	MNH_DDR_PHY_OUTf(220, PHY_RPTR_UPDATE_1, 0x07);

	MNH_DDR_PHY_OUTf(339, PHY_RDDQS_LATENCY_ADJUST_2, 0x03);
	MNH_DDR_PHY_OUTf(339, PHY_RDDQS_GATE_SLAVE_DELAY_2, 0x011a);
	MNH_DDR_PHY_OUTf(341, PHY_GTLVL_LAT_ADJ_START_2, 0x01);
	MNH_DDR_PHY_OUTf(346, PHY_RDDATA_EN_TSEL_DLY_2, 0x02);
	MNH_DDR_PHY_OUTf(346, PHY_RDDATA_EN_DLY_2, 0x03);
	MNH_DDR_PHY_OUTf(348, PHY_RPTR_UPDATE_2, 0x07);

	MNH_DDR_PHY_OUTf(467, PHY_RDDQS_LATENCY_ADJUST_3, 0x03);
	MNH_DDR_PHY_OUTf(467, PHY_RDDQS_GATE_SLAVE_DELAY_3, 0x011a);
	MNH_DDR_PHY_OUTf(469, PHY_GTLVL_LAT_ADJ_START_3, 0x01);
	MNH_DDR_PHY_OUTf(474, PHY_RDDATA_EN_TSEL_DLY_3, 0x02);
	MNH_DDR_PHY_OUTf(474, PHY_RDDATA_EN_DLY_3, 0x03);
	MNH_DDR_PHY_OUTf(476, PHY_RPTR_UPDATE_3, 0x07);

	MNH_DDR_PHY_OUTf(1045, PHY_PLL_CTRL_TOP, 0x0122);
	MNH_DDR_PHY_OUTf(1045, PHY_PLL_CTRL, 0x1102);
	MNH_DDR_PHY_OUTf(1046, PHY_PLL_CTRL_CA, 0x0122);

	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 2);

	/* b */
	MNH_DDR_PHY_OUTf(83, PHY_RDDQS_GATE_SLAVE_DELAY_0, 0x0119);
	MNH_DDR_PHY_OUTf(90, PHY_RDDATA_EN_TSEL_DLY_0, 0x01);
	MNH_DDR_PHY_OUTf(90, PHY_RDDATA_EN_DLY_0, 0x02);

	MNH_DDR_PHY_OUTf(211, PHY_RDDQS_GATE_SLAVE_DELAY_1, 0x0119);
	MNH_DDR_PHY_OUTf(218, PHY_RDDATA_EN_TSEL_DLY_1, 0x01);
	MNH_DDR_PHY_OUTf(218, PHY_RDDATA_EN_DLY_1, 0x02);

	MNH_DDR_PHY_OUTf(339, PHY_RDDQS_GATE_SLAVE_DELAY_2, 0x0119);
	MNH_DDR_PHY_OUTf(346, PHY_RDDATA_EN_TSEL_DLY_2, 0x01);
	MNH_DDR_PHY_OUTf(346, PHY_RDDATA_EN_DLY_2, 0x02);

	MNH_DDR_PHY_OUTf(467, PHY_RDDQS_GATE_SLAVE_DELAY_3, 0x0119);
	MNH_DDR_PHY_OUTf(474, PHY_RDDATA_EN_TSEL_DLY_3, 0x01);
	MNH_DDR_PHY_OUTf(474, PHY_RDDATA_EN_DLY_3, 0x02);

	MNH_DDR_PHY_OUTf(1045, PHY_PLL_CTRL_TOP, 0x0122);
	MNH_DDR_PHY_OUTf(1045, PHY_PLL_CTRL, 0x1102);
	MNH_DDR_PHY_OUTf(1046, PHY_PLL_CTRL_CA, 0x0122);
#endif

	dev_info(dev, "%s begin training,", __func__);
	MNH_DDR_PI_OUTf(00, PI_START, 1);
	MNH_DDR_CTL_OUTf(00, START, 1);

	i = 0;
	while ((i < 1000) && !(MNH_DDR_CTL_IN(227) & 0x00000010)) {
		udelay(10);
		i++;
	}

	if (i == 1000)
		dev_err(dev, "%s: ddr training failed\n", __func__);
	else
		dev_info(dev, "%s done.", __func__);
#ifdef MNH_DDR_DO_FSP
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F0_0, 1);
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F1_0, 1);
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F2_0, 1);
	MNH_DDR_CTL_OUTf(166, MR_FSP_DATA_VALID_F3_0, 1);
#endif
	return 0;
}
EXPORT_SYMBOL(mnh_ddr_po_init);
