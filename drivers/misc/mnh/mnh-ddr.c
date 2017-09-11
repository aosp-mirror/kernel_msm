/*
*
* MNH DDR Driver
* Copyright (c) 2016-2017, Intel Corporation.
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

/* #define DEBUG */

#include "mnh-clk.h"
#include "mnh-hwio.h"
#include "mnh-hwio-bases.h"
#include "mnh-hwio-ddr-ctl.h"
#include "mnh-hwio-ddr-pi.h"
#include "mnh-hwio-ddr-phy.h"
#include "mnh-hwio-scu.h"
#include "mnh-ddr-33-300-600-400.h"
#include "mnh-ddr.h"
#include "mnh-pcie.h"
#include "mnh-sm.h"

#define MNH_DDR_CTL_IN(reg) \
	HW_IN(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, reg)
#define MNH_DDR_CTL_INf(reg, fld) \
	HW_INf(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, reg, fld)
#define MNH_DDR_CTL_OUT(reg, val) \
	HW_OUT(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, reg, val)
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

#define MNH_SCU_IN(reg) \
	HW_IN(HWIO_SCU_BASE_ADDR, SCU, reg)
#define MNH_SCU_INf(reg, fld) \
	HW_INf(HWIO_SCU_BASE_ADDR, SCU, reg, fld)
#define MNH_SCU_INx(reg, inst) \
	HW_INx(HWIO_SCU_BASE_ADDR, SCU, reg, inst)
#define MNH_SCU_INxf(reg, inst, fld) \
	HW_INxf(HWIO_SCU_BASE_ADDR, SCU, reg, inst, fld)
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
	_state.fsps[fsp] &= 0xFFFFFF00;\
	_state.fsps[fsp] |= 0x7d;\
	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, fsp, _state.fsps[fsp])

#define SAVE_CURRENT_FSP() \
	_state.suspend_fsp = \
		MNH_SCU_INf(LPDDR4_LOW_POWER_STS, LPDDR4_CUR_FSP); \
	dev_dbg(dev, "%s: saved fsp: %d\n", __func__, _state.suspend_fsp)

#define SAVED_FSP() _state.suspend_fsp

#define WRITE_CLK_FROM_FSP(fsp) \
do { \
	if (fsp < (MNH_DDR_NUM_FSPS)) {	     \
		MNH_SCU_OUTf(CCU_CLK_DIV, LPDDR4_REFCLK_DIV, \
			MNH_SCU_INxf(LPDDR4_FSP_SETTING, fsp, \
				FSP_LPDDR4_REFCLK_DIV)); \
		MNH_SCU_OUTf(CCU_CLK_DIV, AXI_FABRIC_CLK_DIV, \
			MNH_SCU_INxf(LPDDR4_FSP_SETTING, fsp, \
				FSP_AXI_FABRIC_CLK_DIV)); \
		MNH_SCU_OUTf(CCU_CLK_DIV, PCIE_AXI_CLK_DIV, \
			MNH_SCU_INxf(LPDDR4_FSP_SETTING, fsp, \
				FSP_PCIE_AXI_CLK_DIV)); \
		MNH_SCU_OUTf(CCU_CLK_CTL, LP4_AXI_SYS200_MODE, \
			MNH_SCU_INxf(LPDDR4_FSP_SETTING, fsp, \
				FSP_SYS200_MODE)); \
	} else \
		dev_err(dev, "%s: invalid fsp 0x%x", __func__, fsp); \
} while (0)

#define SAVE_DDR_REG_CONFIG(ddrblock, regindex) \
	_state.ddrblock[regindex] = \
		mnh_reg_read(_state.ddrblock##_base + (regindex * sizeof(u32)))

#define SAVE_DDR_PHY_REG_CONFIG(fsp, regindex) \
	_state.phy[fsp][regindex] = \
		mnh_reg_read(_state.phy_base + (regindex * sizeof(u32)))

#define CLR_START(ddrblock) (_state.ddrblock[0] &= (0xFFFFFFFE))

/* timeout for training all FSPs */
#define TRAINING_TIMEOUT msecs_to_jiffies(45)

#define LP_CMD_EXIT_LP 0x81
#define LP_CMD_DSRPD 0xFE

/* INT status bits */
#define MR_WRITE_SBIT 26
#define MR_READ_SBIT 23
#define LP_CMD_SBIT 5
#define INIT_DONE_SBIT 4

static struct mnh_ddr_state mnh_ddr_po_config = {
	.bases = {
		HWIO_DDR_CTL_BASE_ADDR,
		HWIO_DDR_PHY_BASE_ADDR,
		HWIO_DDR_PI_BASE_ADDR
	},
	.fsps = {
		0x131B007D,
		0x1311007D,
		0x0311007D,
		0x0422007D
	},
	&mnh_ddr_33_300_600_400,
};

struct mnh_ddr_internal_state _state;
/* read entire int_status */
u64 mnh_ddr_int_status(struct device *dev)
{
	u64 int_stat = ((u64)MNH_DDR_CTL_IN(228) << 32) | MNH_DDR_CTL_IN(227);
	return int_stat;
}
EXPORT_SYMBOL(mnh_ddr_int_status);

/* clear entire int_status */
int mnh_ddr_clr_int_status(struct device *dev)
{
	u64 stat = 0;

	MNH_DDR_CTL_OUT(230, 0x0F);
	MNH_DDR_CTL_OUT(229, 0xFFFFFFFF);
	stat = mnh_ddr_int_status(dev);
	if (stat) {
		dev_err(dev, "%s: int stat not all clear: %llx\n", __func__,
			stat);
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL(mnh_ddr_clr_int_status);

/* read single bit in int_status */
static u32 mnh_ddr_int_status_bit(u8 sbit)
{
	u32 status = 0;
	u32 upper = 0;
	const u32 max_int_status_bit = 35;
	const u32 first_upper_bit = 32;

	if (sbit > max_int_status_bit)
		return -EINVAL;

	/*
	 * docs refer to int status by bit numbers 0-35,
	 * but we're only reading 32 bits at a time.
	 */
	upper = (sbit >= first_upper_bit) ? 1 : 0;
	sbit -= (upper) ? first_upper_bit : 0;

	status = (upper) ? MNH_DDR_CTL_IN(228) : MNH_DDR_CTL_IN(227);
	status &= (1 << sbit);
	return status;
}

/* clear single bit in int_status */
static int mnh_ddr_clr_int_status_bit(struct device *dev, u8 sbit)
{
	const u32 max_int_status_bit = 35;
	const u32 first_upper_bit = 32;

	if (sbit > max_int_status_bit)
		return -EINVAL;

	if (sbit >= first_upper_bit)
		MNH_DDR_CTL_OUT(230, 1 << (sbit - first_upper_bit));
	else
		MNH_DDR_CTL_OUT(229, 1 << sbit);

	if (mnh_ddr_int_status_bit(sbit)) {
		dev_err(dev, "%s: bit %d is still set.\n", __func__, sbit);
		return -EIO;
	}
	return 0;
}

static int mnh_ddr_send_lp_cmd(struct device *dev, u8 cmd)
{
	u32 timeout = 100000;

	dev_dbg(dev, "%s sending cmd: 0x%x\n", __func__, cmd);
	MNH_DDR_CTL_OUTf(112, LP_CMD, cmd);

	while (!mnh_ddr_int_status_bit(LP_CMD_SBIT) && --timeout)
		udelay(1);

	if (!mnh_ddr_int_status_bit(LP_CMD_SBIT))
		return -ETIMEDOUT;

	return mnh_ddr_clr_int_status_bit(dev, LP_CMD_SBIT);
}

static void mnh_ddr_enable_lp(void)
{
	MNH_DDR_CTL_OUTf(124, LP_AUTO_SR_MC_GATE_IDLE, 0xFF);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_MEM_GATE_EN, 0x4);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_ENTRY_EN, 0x4);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_EXIT_EN, 0xF);
}

static void mnh_ddr_disable_lp(struct device *dev)
{
	MNH_DDR_CTL_OUTf(124, LP_AUTO_SR_MC_GATE_IDLE, 0x00);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_MEM_GATE_EN, 0x0);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_ENTRY_EN, 0x0);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_EXIT_EN, 0x0);
	mnh_ddr_send_lp_cmd(dev, LP_CMD_EXIT_LP);
}

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

	memcpy(&(_state.pi[0]),
		&(state->config->pi[0]),
		MNH_DDR_NUM_PI_REG * sizeof(u32));

	memcpy(&(_state.fsps[0]),
		&(state->fsps[0]),
		MNH_DDR_NUM_FSPS * sizeof(u32));
	_state.suspend_fsp = 0;
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

	dev_dbg(dev, "%s waiting for lpddr4 pll lock", __func__);
	while ((timeout < 20) && (!MNH_SCU_INf(LPDDR4_REFCLK_PLL_STS, LOCK))) {
		udelay(1);
		timeout++;
	}

	if (timeout == 20)
		dev_err(dev, "%s lpddr4 pll lock failed", __func__);
	else
		dev_dbg(dev, "%s lpddr4 pll locked after %d iterations",
			 __func__, timeout);

	WRITE_SCU_FSP(0);
	WRITE_SCU_FSP(1);
	WRITE_SCU_FSP(2);
	WRITE_SCU_FSP(3);

	WRITE_CLK_FROM_FSP(SAVED_FSP());
	dev_dbg(dev, "%s lpddr4 pll locked", __func__);
	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LP4_FSP_SW_OVERRIDE, 0);
	/* MNH_PLL_PASSCODE_CLR */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x0);
}

static void mnh_ddr_pull_config(void)
{
	int index, fsp;
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
}

int mnh_ddr_suspend(struct device *dev, struct gpio_desc *iso_n)
{
	mnh_ddr_disable_lp(dev);

	/* resume to fsp2 */
	mnh_lpddr_freq_change(LPDDR_FREQ_FSP2);
	SAVE_CURRENT_FSP();
	mnh_ddr_pull_config();

	if (mnh_ddr_send_lp_cmd(dev, LP_CMD_DSRPD))
		dev_err(dev, "%s: failed to get LP complete\n", __func__);

	/* Enable clock gating */
	MNH_SCU_OUTf(CCU_CLK_CTL, HALT_LP4CG_EN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, HALT_LP4_PLL_BYPCLK_CG_EN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, LP4PHY_PLL_BYPASS_CLKEN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, LP4_REFCLKEN, 0);

	udelay(1);
	gpiod_set_value_cansleep(iso_n, 0);
	udelay(1);

	dev_dbg(dev, "%s done.", __func__);

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

	MNH_DDR_CTL_OUTf(23, DFIBUS_FREQ_INIT, SAVED_FSP());
	MNH_DDR_CTL_OUTf(23, DFIBUS_BOOT_FREQ, 0);

	MNH_DDR_CTL_OUTf(23, PHY_INDEP_TRAIN_MODE, 0);
	MNH_DDR_CTL_OUTf(23, CDNS_INTRL0, 1);

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

	udelay(1000);

	MNH_DDR_CTL_OUTf(00, START, 1);

	dev_dbg(dev, "%s waiting for init done.", __func__);

	timeout = 0;
	while ((timeout < 1000) && (!mnh_ddr_int_status_bit(INIT_DONE_SBIT))) {
		udelay(1);
		timeout++;
	}

	if (!mnh_ddr_int_status_bit(INIT_DONE_SBIT)) {
		dev_err(dev, "%s time out on init done %llx.\n",
			__func__, mnh_ddr_int_status(dev));
		return -ETIMEDOUT;
	}

	/* need to clear PWRUP_SREFRESH_EXIT to clear interrupt status bit 0 */
	MNH_DDR_CTL_OUTf(81, PWRUP_SREFRESH_EXIT, 0);
	dev_dbg(dev, "%s got init done %llx.\n", __func__,
		mnh_ddr_int_status(dev));
	mnh_ddr_clr_int_status(dev);
	mnh_lpddr_freq_change(SAVED_FSP());
	mnh_ddr_enable_lp();

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_resume);

int mnh_ddr_po_init(struct device *dev, struct gpio_desc *iso_n)
{
	int index;
	unsigned long timeout;
	struct mnh_ddr_state *state = &mnh_ddr_po_config;

	mnh_ddr_init_internal_state(state);

	dev_dbg(dev, "%s start.", __func__);

	/* Gate CPU clock while initializing DDR */
	MNH_SCU_OUTf(CCU_CLK_CTL, HALT_CPUCG_EN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, CPU_CLKEN, 0);

	/* deassert iso_n */
	gpiod_set_value_cansleep(iso_n, 1);

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

	dev_dbg(dev, "%s begin training,", __func__);
	MNH_DDR_PI_OUTf(00, PI_START, 1);
	MNH_DDR_CTL_OUTf(00, START, 1);

	timeout = jiffies + TRAINING_TIMEOUT;
	while (time_before(jiffies, timeout) &&
	       (!mnh_ddr_int_status_bit(INIT_DONE_SBIT)))
		usleep_range(100, 200);

	if (!mnh_ddr_int_status_bit(INIT_DONE_SBIT)) {
		dev_err(dev, "%s timed out on init done.\n", __func__);
		return -ETIMEDOUT;
	}

	dev_dbg(dev, "%s got init done %llx.\n", __func__,
		 mnh_ddr_int_status(dev));

	mnh_ddr_clr_int_status(dev);
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F0_0, 1);
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F1_0, 1);
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F2_0, 1);
	MNH_DDR_CTL_OUTf(166, MR_FSP_DATA_VALID_F3_0, 1);

	mnh_ddr_enable_lp();

	/* Enable FSP2 => 2400 */
	mnh_lpddr_freq_change(LPDDR_FREQ_FSP2);

	/* Enable CPU clock after DDR init is done */
	MNH_SCU_OUTf(CCU_CLK_CTL, CPU_CLKEN, 1);

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_po_init);
