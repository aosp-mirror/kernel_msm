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
#include "mnh-ddr-33-300-600-400.h"
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
	mnh_reg_write(_state.ddrblock##_base + (regindex * sizeof(u32)), \
		      _state.ddrblock[regindex])

#define WRITE_DDR_PHY_CONFIG(fsp, regindex)    \
	mnh_reg_write(_state.phy_base + (regindex * sizeof(u32)),\
		      _state.phy[fsp][regindex])

#define WRITE_SCU_FSP(fsp) \
	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, fsp, _state.fsps[fsp])

#define SAVE_DDR_REG_CONFIG(ddrblock, regindex) \
	_state.ddrblock[regindex] = \
		mnh_reg_read(_state.ddrblock##_base + (regindex * sizeof(u32)))

#define SAVE_DDR_PHY_REG_CONFIG(fsp, regindex) \
	_state.phy[fsp][regindex] = \
		mnh_reg_read(_state.phy_base + (regindex * sizeof(u32)))

#define CLR_START(ddrblock) (_state.ddrblock[0] &= (0xFFFFFFFE))

static struct mnh_ddr_state mnh_ddr_po_config = {
	.bases = {
		HWIO_DDR_CTL_BASE_ADDR,
		HWIO_DDR_PHY_BASE_ADDR,
		HWIO_DDR_PI_BASE_ADDR
	},
	.fsps = {
		0x100B007D,
		0x0433007D,
		0x0311007D,
		0x0422007D
	},
	&mnh_ddr_33_300_600_400,
};

struct mnh_ddr_internal_state _state;

static void mnh_ddr_init_internal_state(struct mnh_ddr_state *state)
{
	_state.ctl_base = state->bases.ctl_base;
	_state.phy_base = state->bases.phy_base;
	_state.pi_base = state->bases.pi_base;
	memcpy(&(_state.ctl[0]),
		 &(state->config->ctl[0]),
		 MNH_DDR_NUM_CTL_REG * sizeof(u32));

	memcpy(&(_state.phy[0][0]),
		&(state->config->phy[0]),
		MNH_DDR_NUM_PHY_REG * sizeof(u32));
	memcpy(&(_state.phy[1][0]),
		&(state->config->phy[0]),
		MNH_DDR_NUM_PHY_REG * sizeof(u32));
	memcpy(&(_state.phy[2][0]),
		&(state->config->phy[0]),
		MNH_DDR_NUM_PHY_REG * sizeof(u32));

	memcpy(&(_state.pi[0]),
		&(state->config->pi[0]),
		MNH_DDR_NUM_PI_REG * sizeof(u32));

	memcpy(&(_state.fsps[0]),
		&(state->fsps[0]),
		MNH_DDR_NUM_FSPS * sizeof(u32));
}

void mnh_ddr_init_clocks(struct device *dev)
{
	int timeout = 0;

	/* MNH_PLL_PASSCODE_SET */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x4CD9);

	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 1);
	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, PD, 0);
	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FOUTPOSTDIVPD, 0);
	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 0);

	dev_info(dev, "%s waiting for lpddr4 pll lock", __func__);
	while ((timeout < 20) && (!MNH_SCU_INf(LPDDR4_REFCLK_PLL_STS, LOCK))) {
		udelay(1);
		timeout++;
	}

	if (timeout == 20)
		dev_err(dev, "%s lpddr4 pll lock failed", __func__);
	else
		dev_info(dev, "%s lpddr4 pll locked after %d iterations",
			 __func__, timeout);

	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LP4_FSP_SW_OVERRIDE, 0);

	WRITE_SCU_FSP(0);
	WRITE_SCU_FSP(1);
	WRITE_SCU_FSP(2);
	WRITE_SCU_FSP(3);

	/* MNH_PLL_PASSCODE_CLR */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x0);
}

int mnh_ddr_suspend(struct device *dev, struct gpio_desc *iso_n)
{
	int index, fsp;
	int timeout = 0;

#ifdef PARTIAL_REFRESH
	/* refresh only first segment */
	MNH_DDR_CTL_OUTf(166, MR17_DATA_0, 0xFE);
#endif

	for (index = 0; index < MNH_DDR_NUM_CTL_REG; index++)
		SAVE_DDR_REG_CONFIG(ctl, index);
	CLR_START(ctl);

	for (index = 0; index < MNH_DDR_NUM_PI_REG; index++)
		SAVE_DDR_REG_CONFIG(pi, index);
	CLR_START(pi);

	for (fsp = 0; fsp < MNH_DDR_PHY_NUM_FSPS; fsp++) {
		MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, fsp);

		for (index = 0; index < MNH_DDR_NUM_PHY_REG; index++)
			SAVE_DDR_PHY_REG_CONFIG(fsp, index);
	}

	/* Enable clock gating */
	MNH_SCU_OUTf(CCU_CLK_CTL, HALT_LP4CG_EN, 1);
	MNH_SCU_OUTf(CCU_CLK_CTL, HALT_LP4_PLL_BYPCLK_CG_EN, 1);
	MNH_SCU_OUTf(MEM_PWR_MGMNT, HALT_LP4CMEM_PD_EN, 1);

	MNH_DDR_CTL_OUTf(112, LP_CMD, 0xFE);
	dev_info(dev, "%s waiting for LP complete.", __func__);
	while ((timeout < 10) && !(MNH_DDR_CTL_IN(227) & 0x00000020)) {
		udelay(1);
		timeout++;
	}
	if (timeout == 10)
		dev_info(dev, "%s: failed to get LP complete\n", __func__);
	else
		dev_info(dev, "%s got it after %d iterations. 121 is 0x%x",
			 __func__, timeout, MNH_DDR_CTL_INf(121, LP_STATE));

	gpiod_set_value_cansleep(iso_n, 0);
	udelay(1000);

	dev_info(dev, "%s done.", __func__);

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_suspend);

int mnh_ddr_resume(struct device *dev, struct gpio_desc *iso_n)
{
	int index, fsp;
	int timeout = 0;

	mnh_ddr_init_clocks(dev);

	for (index = 0; index < MNH_DDR_NUM_CTL_REG; index++)
		WRITE_DDR_REG_CONFIG(ctl, index);

#ifdef PARTIAL_REFRESH
	/* refresh no segments */
	MNH_DDR_CTL_OUTf(166, MR17_DATA_0, 0xFF);
#endif

	for (index = 0; index < MNH_DDR_NUM_PI_REG; index++)
		WRITE_DDR_REG_CONFIG(pi, index);

	for (fsp = 0; fsp < MNH_DDR_PHY_NUM_FSPS; fsp++) {
		MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_MULTICAST_EN, 0);
		MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, fsp);

		for (index = 0; index < MNH_DDR_NUM_PHY_REG; index++) {
			if (index != 1025)
				WRITE_DDR_PHY_CONFIG(fsp, index);
		}
	}

	gpiod_set_value_cansleep(iso_n, 1);
	udelay(1000);
	MNH_DDR_CTL_OUTf(81, PWRUP_SREFRESH_EXIT, 1);

	while ((timeout < 10) && (MNH_DDR_CTL_INf(450, MEM_RST_VALID) == 0)) {
		udelay(1000);
		timeout++;
	}

	if (timeout == 10)
		dev_err(dev, "%s: failed to see reset valid\n", __func__);
	else
		dev_info(dev, "%s: observed reset valid after %d iterations\n",
			 __func__, timeout);

	/*MNH_DDR_PHY_OUTf(1051, PHY_SET_DFI_INPUT_RST_PAD, 1);*/
	udelay(1000);

	/*MNH_DDR_PI_OUTf(00, PI_START, 1);*/
	MNH_DDR_CTL_OUTf(00, START, 1);

	dev_info(dev, "%s waiting for init done.", __func__);

	/*
	* INT_STATUS: bit 4 init done
	*/
	timeout = 0;
	while ((timeout < 10) && !(MNH_DDR_CTL_IN(227) & 0x00000010)) {
		udelay(1);
		timeout++;
	}
	if (timeout == 10)
		dev_err(dev, "%s: init failed", __func__);
	else
		dev_info(dev, "%s: init done after %d iterations.",
			 __func__, timeout);

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_resume);


int mnh_ddr_po_init(struct device *dev)
{
	int index;
	int timeout = 0;
	struct mnh_ddr_state *state = &mnh_ddr_po_config;

	mnh_ddr_init_internal_state(state);

	dev_info(dev, "%s start.", __func__);

	mnh_ddr_init_clocks(dev);

	for (index = 0; index < MNH_DDR_NUM_CTL_REG; index++)
		WRITE_DDR_REG_CONFIG(ctl, index);

	for (index = 0; index < MNH_DDR_NUM_PI_REG; index++)
		WRITE_DDR_REG_CONFIG(pi, index);

	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_MULTICAST_EN, 1);
	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 0);

	for (index = 0; index < MNH_DDR_NUM_PHY_REG; index++)
		WRITE_DDR_PHY_CONFIG(0, index);

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

	dev_info(dev, "%s begin training,", __func__);
	MNH_DDR_PI_OUTf(00, PI_START, 1);
	MNH_DDR_CTL_OUTf(00, START, 1);

	while ((timeout < 1000) && !(MNH_DDR_CTL_IN(227) & 0x00000010)) {
		udelay(10);
		timeout++;
	}

	if (timeout == 1000)
		dev_err(dev, "%s: ddr training failed\n", __func__);
	else
		dev_info(dev, "%s done after %d iterations.",
			 __func__, timeout);

	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F0_0, 1);
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F1_0, 1);
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F2_0, 1);
	MNH_DDR_CTL_OUTf(166, MR_FSP_DATA_VALID_F3_0, 1);

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_po_init);
