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

#include "mnh-clk.h"
#include "mnh-hwio.h"
#include "mnh-hwio-bases.h"
#include "mnh-hwio-ddr-ctl.h"
#include "mnh-hwio-ddr-pi.h"
#include "mnh-hwio-ddr-phy.h"
#include "mnh-hwio-scu.h"
#include "mnh-ddr-33-100-400-600.h"
#include "mnh-ddr.h"
#include "mnh-pcie.h"
#include "mnh-sm.h"

#define MNH_DDR_CTL_IN(reg) \
	HW_IN(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, reg)
#define MNH_DDR_CTL_INf(...) \
	HW_INf(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, __VA_ARGS__)
#define MNH_DDR_CTL_OUT(...) \
	HW_OUT(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, __VA_ARGS__)
#define MNH_DDR_CTL_OUTf(...) \
	HW_OUTf(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, __VA_ARGS__)

#define MNH_DDR_PI_INf(...) \
	HW_INf(HWIO_DDR_PI_BASE_ADDR, DDR_PI, __VA_ARGS__)
#define MNH_DDR_PI_OUTf(...) \
	HW_OUTf(HWIO_DDR_PI_BASE_ADDR, DDR_PI, __VA_ARGS__)
#define MNH_DDR_PI_OUT(...) \
	HW_OUT(HWIO_DDR_PI_BASE_ADDR, DDR_PI, __VA_ARGS__)

#define MNH_DDR_PHY_INf(...) \
	HW_INf(HWIO_DDR_PHY_BASE_ADDR, DDR_PHY, __VA_ARGS__)
#define MNH_DDR_PHY_OUTf(...) \
	HW_OUTf(HWIO_DDR_PHY_BASE_ADDR, DDR_PHY, __VA_ARGS__)
#define MNH_DDR_PHY_OUT(...) \
	HW_OUT(HWIO_DDR_PHY_BASE_ADDR, DDR_PHY, __VA_ARGS__)

#define MNH_SCU_IN(reg) \
	HW_IN(HWIO_SCU_BASE_ADDR, SCU, reg)
#define MNH_SCU_INf(...) \
	HW_INf(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)
#define MNH_SCU_INx(...) \
	HW_INx(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)
#define MNH_SCU_INxf(...) \
	HW_INxf(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)
#define MNH_SCU_OUTf(...) \
	HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)
#define MNH_SCU_OUT(...) \
	HW_OUT(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)
#define MNH_SCU_OUTx(...) \
	HW_OUTx(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)

#define MNH_RSTC_INf(fld) \
	HW_INf(HWIO_SCU_BASE_ADDR, SCU, RSTC, fld)
#define MNH_RSTC_OUTf(...) \
	HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, RSTC, __VA_ARGS__)

#define WRITE_DDR_REG_CONFIG(ddrblock, regindex) \
do { \
	if (_state->ddrblock[regindex]) { \
		mnh_reg_write(_state->ddrblock##_base + \
				(regindex * sizeof(u32)), \
			      _state->ddrblock[regindex]); \
	} \
} while (0)

#define WRITE_DDR_PHY_CONFIG(fsp, regindex)    \
do { \
	if (_state->phy[fsp][regindex]) { \
		mnh_reg_write(_state->phy_base + (regindex * sizeof(u32)), \
			      _state->phy[fsp][regindex]); \
	} \
} while (0)

#define WRITE_SET_ELEMENT(regindex, regvalue)	\
	mnh_reg_write(_state->phy_base + (regindex * sizeof(u32)),\
		regvalue)

#define WRITE_SCU_FSP(fsp) \
do { \
	_state->fsps[fsp] &= 0xFFFFFF00;\
	_state->fsps[fsp] |= 0x7d;\
	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, fsp, _state->fsps[fsp]); \
} while (0)

#define SAVE_CURRENT_FSP() \
do { \
	_state->suspend_fsp = \
		MNH_SCU_INf(LPDDR4_LOW_POWER_STS, LPDDR4_CUR_FSP); \
	dev_dbg(dev, "%s: saved fsp: %d\n", __func__, _state->suspend_fsp); \
} while (0)

#define SAVED_FSP() _state->suspend_fsp

#define WRITE_CLK_FROM_FSP(fsp) \
do { \
	if (fsp < (MNH_DDR_NUM_FSPS)) { \
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
	_state->ddrblock[regindex] = \
		mnh_reg_read(_state->ddrblock##_base + (regindex * sizeof(u32)))

#define SAVE_DDR_PHY_REG_CONFIG(fsp, regindex) \
	_state->phy[fsp][regindex] = \
		mnh_reg_read(_state->phy_base + (regindex * sizeof(u32)))

#define CLR_START(ddrblock) (_state->ddrblock[0] &= (0xFFFFFFFE))

/* timeout for training all FSPs */
#define TRAINING_TIMEOUT msecs_to_jiffies(45)

#define LP_CMD_EXIT_LP 0x81
#define LP_CMD_DSRPD 0xFE

/* INT status bits */
#define MR_WRITE_SBIT 26
#define MR_READ_SBIT 23
#define BIST_SBIT 6
#define LP_CMD_SBIT 5
#define INIT_DONE_SBIT 4

static struct mnh_ddr_internal_state *_state;

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

static void mnh_ddr_init_internal_state(const struct mnh_ddr_reg_config *cfg)
{
	_state->ctl_base = HWIO_DDR_CTL_BASE_ADDR;
	_state->pi_base = HWIO_DDR_PI_BASE_ADDR;
	_state->phy_base = HWIO_DDR_PHY_BASE_ADDR;

	memcpy(&(_state->fsps[0]),
		&(cfg->fsps[0]),
		MNH_DDR_NUM_FSPS * sizeof(u32));

	memcpy(&(_state->ctl[0]),
		 &(cfg->ctl[0]),
		 MNH_DDR_NUM_CTL_REG * sizeof(u32));

	memcpy(&(_state->phy[0][0]),
		&(cfg->phy[0]),
		MNH_DDR_NUM_PHY_REG * sizeof(u32));

	memcpy(&(_state->pi[0]),
		&(cfg->pi[0]),
		MNH_DDR_NUM_PI_REG * sizeof(u32));

	_state->suspend_fsp = 0;
	_state->tref[0] = cfg->ctl[56] & 0xFFFF;
	_state->tref[1] = cfg->ctl[57] & 0xFFFF;
	_state->tref[2] = cfg->ctl[58] & 0xFFFF;
	_state->tref[3] = cfg->ctl[59] & 0xFFFF;
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

	for (fsp = 0; fsp < MNH_DDR_NUM_FSPS; fsp++) {
		MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, fsp);
		for (index = 0; index < MNH_DDR_NUM_PHY_REG; index++)
			SAVE_DDR_PHY_REG_CONFIG(fsp, index);
	}
}

int mnh_ddr_suspend(struct device *dev, struct gpio_desc *iso_n)
{
	mnh_ddr_disable_lp(dev);

	dev_dbg(dev, "%s: tref 0x%04x 0x%04x 0x%04x 0x%04x\n",
		__func__, MNH_DDR_CTL_INf(56, TREF_F0),
		MNH_DDR_CTL_INf(57, TREF_F1), MNH_DDR_CTL_INf(58, TREF_F2),
		MNH_DDR_CTL_INf(59, TREF_F3));

	/*
	 * restore hot tref settings, rather than what was
	 * saved from the MNH to handle the case where
	 * we're suspending in a cold temp and resuming at hot.
	 * If we're not actually hot, the MNH side will adjust
	 * the rate downward.
	 */
	MNH_DDR_CTL_OUTf(56, TREF_F0, _state->tref[0]);
	MNH_DDR_CTL_OUTf(57, TREF_F1, _state->tref[1]);
	MNH_DDR_CTL_OUTf(58, TREF_F2, _state->tref[2]);
	MNH_DDR_CTL_OUTf(59, TREF_F3, _state->tref[3]);

	/* resume to fsp3 */
	mnh_lpddr_freq_change(LPDDR_FREQ_FSP3);
	SAVE_CURRENT_FSP();
	mnh_ddr_pull_config();

	mnh_ddr_send_lp_cmd(dev, LP_CMD_DSRPD);
	dev_dbg(dev, "%s LP_STATE is 0x%x", __func__,
		MNH_DDR_CTL_INf(121, LP_STATE));

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

	for (fsp = 0; fsp < MNH_DDR_NUM_FSPS; fsp++) {
		MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_MULTICAST_EN, 0);
		MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, fsp);

		for (index = 0; index < MNH_DDR_NUM_PHY_REG; index++) {
			if (index != 1025)
				WRITE_DDR_PHY_CONFIG(fsp, index);
		}
		MNH_DDR_PHY_OUTf(1084, PHY_CAL_CLK_SELECT_0, 0x4);
	}

	MNH_DDR_CTL_OUTf(81, PWRUP_SREFRESH_EXIT, 1);

	while ((timeout < 10) && (MNH_DDR_CTL_INf(450, MEM_RST_VALID) == 0)) {
		udelay(1000);
		timeout++;
	}

	if (timeout == 10)
		dev_err(dev, "%s: failed to see reset valid\n", __func__);

	udelay(1000);
	/* extra delay after getting reset valid */
	udelay(1000);
	MNH_DDR_PHY_OUTf(1051, PHY_SET_DFI_INPUT_RST_PAD, 1);

	gpiod_set_value_cansleep(iso_n, 1);
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

	dev_dbg(dev, "%s: tref 0x%04x 0x%04x 0x%04x 0x%04x\n",
		__func__, MNH_DDR_CTL_INf(56, TREF_F0),
		MNH_DDR_CTL_INf(57, TREF_F1), MNH_DDR_CTL_INf(58, TREF_F2),
		MNH_DDR_CTL_INf(59, TREF_F3));

	mnh_ddr_enable_lp();

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_resume);

int mnh_ddr_po_init(struct device *dev, struct gpio_desc *iso_n)
{
	int index, setindex;
	unsigned long timeout;
	const struct mnh_ddr_reg_config *cfg = &mnh_ddr_33_100_400_600;

	_state = devm_kzalloc(dev, sizeof(struct mnh_ddr_internal_state),
			      GFP_KERNEL);
	if (!_state)
		return -ENOMEM;

	mnh_ddr_init_internal_state(cfg);

	dev_dbg(dev, "%s start.", __func__);

	/* deassert iso_n */
	gpiod_set_value_cansleep(iso_n, 1);

	mnh_ddr_init_clocks(dev);

	for (index = 0; index < MNH_DDR_NUM_CTL_REG; index++)
		WRITE_DDR_REG_CONFIG(ctl, index);

	/* Make sure DRAM will request refresh rate adjustments */
	MNH_DDR_CTL_OUTf(164, MR13_DATA_0, 0xD0);

	for (index = 0; index < MNH_DDR_NUM_PI_REG; index++)
		WRITE_DDR_REG_CONFIG(pi, index);

	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_MULTICAST_EN, 1);
	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 0);

	for (index = 0; index < MNH_DDR_NUM_PHY_REG; index++)
		WRITE_DDR_PHY_CONFIG(0, index);

	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_MULTICAST_EN, 0);
	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 1);

	/* a */
	setindex = 0;
	while ((setindex < MNH_DDR_PHY_SET_SIZE) &&
		(cfg->phy_setA[setindex][0] != 0xFFFFFFFF)) {
		WRITE_SET_ELEMENT(cfg->phy_setA[setindex][0],
			cfg->phy_setA[setindex][1]);
		setindex++;
	}

	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 2);

	/* b */
	setindex = 0;
	while ((setindex < MNH_DDR_PHY_SET_SIZE) &&
		(cfg->phy_setB[setindex][0] != 0xFFFFFFFF)) {
		WRITE_SET_ELEMENT(cfg->phy_setB[setindex][0],
			cfg->phy_setB[setindex][1]);
		setindex++;
	}

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

	dev_dbg(dev, "%s: tref 0x%04x 0x%04x 0x%04x 0x%04x\n",
		__func__, MNH_DDR_CTL_INf(56, TREF_F0),
		MNH_DDR_CTL_INf(57, TREF_F1), MNH_DDR_CTL_INf(58, TREF_F2),
		MNH_DDR_CTL_INf(59, TREF_F3));

	mnh_ddr_enable_lp();

	/* Enable FSP3 => 2400 */
	mnh_lpddr_freq_change(LPDDR_FREQ_FSP3);

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_po_init);

u32 mnh_ddr_mbist(struct device *dev, enum mnh_ddr_bist_type bist_type)
{
	u32 result = 0;
	u32 timeout = 1000000;
	const u32 pattern[] = {
		0x55555555, 0x33333333, 0x0f0f0f0f, 0x00ff00ff };

	u32 old_in_order_accept;

	switch (bist_type) {
	case MOVI1_3N:
	case LIMITED_MOVI1_3N:
		dev_info(dev, "%s: type %d\n", __func__, bist_type);
		break;
	default:
		dev_err(dev, "%s: type %d is not supported\n",
			__func__, bist_type);
		return 0;
	}

	mnh_ddr_disable_lp(dev);

	old_in_order_accept = MNH_DDR_CTL_INf(223, IN_ORDER_ACCEPT);
	MNH_DDR_CTL_OUTf(223, IN_ORDER_ACCEPT, 1);

	/* 512MB */
	MNH_DDR_CTL_OUTf(171, ADDR_SPACE, 0x1D);
	MNH_DDR_CTL_OUTf(173, BIST_START_ADDRESS, 0);

	MNH_DDR_CTL_OUTf(171, BIST_DATA_CHECK, 1);
	MNH_DDR_CTL_OUTf(172, BIST_ADDR_CHECK, 0);

	/* if limited movi, use these patterns */
	MNH_DDR_CTL_OUTf(178, BIST_DATA_PATTERN, pattern[0]);
	MNH_DDR_CTL_OUTf(179, BIST_DATA_PATTERN, pattern[1]);
	MNH_DDR_CTL_OUTf(180, BIST_DATA_PATTERN, pattern[2]);
	MNH_DDR_CTL_OUTf(181, BIST_DATA_PATTERN, pattern[3]);

	MNH_DDR_CTL_OUTf(175, BIST_DATA_MASK, 0);

	MNH_DDR_CTL_OUTf(177, BIST_TEST_MODE, bist_type);

	MNH_DDR_CTL_OUTf(171, BIST_GO, 1);
	dev_info(dev, "%s: waiting to finish BIST\n", __func__);

	while (!mnh_ddr_int_status_bit(BIST_SBIT) && --timeout)
		msleep(20);

	if (!mnh_ddr_int_status_bit(BIST_SBIT)) {
		dev_err(dev, "%s: BIST timedout: %llx\n",
			__func__, mnh_ddr_int_status(dev));
	} else {
		result = MNH_DDR_CTL_INf(171, BIST_RESULT);
		dev_info(dev, "%s: result 0x%02x\n", __func__, result);
	}

	MNH_DDR_CTL_OUTf(171, BIST_GO, 0);
	mnh_ddr_clr_int_status(dev);

	MNH_DDR_CTL_OUTf(223, IN_ORDER_ACCEPT,
		old_in_order_accept);

	mnh_ddr_enable_lp();
	return result;
}
EXPORT_SYMBOL(mnh_ddr_mbist);
