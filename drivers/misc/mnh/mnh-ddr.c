/*
*
* MNH DDR Driver
* Copyright (c) 2016-2018, Intel Corporation.
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
#define MNH_DDR_PHY_IN(reg) \
	HW_IN(HWIO_DDR_PHY_BASE_ADDR, DDR_PHY, reg)
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
#define MNH_SCU_OUTxf(reg, inst, fld, val) \
	HW_OUTxf(HWIO_SCU_BASE_ADDR, SCU, reg, inst, fld, val)

#define MNH_RSTC_INf(fld) \
	HW_INf(HWIO_SCU_BASE_ADDR, SCU, RSTC, fld)
#define MNH_RSTC_OUTf(fld, val) \
	HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, RSTC, fld, val)

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

#define WRITE_CLK_FROM_FSP(fsp) mnh_ddr_write_clk_from_fsp(fsp, 1)
#define WRITE_CLK_FROM_FSP_NO_LOCK(fsp) mnh_ddr_write_clk_from_fsp(fsp, 0)

#define SAVE_DDR_REG_CONFIG(ddrblock, regindex) \
do { \
	_state->ddrblock[regindex] = \
		mnh_reg_read(_state->ddrblock##_base + \
					((regindex) * sizeof(u32))); \
} while (0)

#define SAVE_DDR_PHY_REG_CONFIG(fsp, regindex) \
do { \
	_state->phy[fsp][regindex] = \
		mnh_reg_read(_state->phy_base + ((regindex) * sizeof(u32))); \
} while (0)

#define CLR_START(ddrblock) (_state->ddrblock[0] &= (0xFFFFFFFE))

/* timeout for training all FSPs */
#define TRAINING_TIMEOUT msecs_to_jiffies(45)
#define RESUME_TIMEOUT msecs_to_jiffies(7)

#define MNH_DDR_ASSERT_ISO_N() \
	do { \
		gpiod_set_value_cansleep(_state->iso_n, 0); \
		udelay(20); \
	} while (0)

#define MNH_DDR_DEASSERT_ISO_N() \
	do { \
		gpiod_set_value_cansleep(_state->iso_n, 1); \
		udelay(20); \
	} while (0)

#define LP_CMD_FREQ_SWITCH 0x8A
#define LP_CMD_EXIT_LP 0x81
#define LP_CMD_DSRPD 0xFE
#define LP_CMD_SRPD 0x3A
#define LP_CMD_EXIT_SRPD 0x01

/* INT status bits */
#define DFS_COMPLETE_SBIT 31
#define DFI_STATE_CHANGE_SBIT 28
#define INHIBIT_DRAM_CMD_SBIT 27
#define MR_WRITE_SBIT 26
#define MR_READ_SBIT 23
#define DFI_UPDATE_ERROR_SBIT 13
#define BIST_SBIT 6
#define LP_CMD_SBIT 5
#define INIT_DONE_SBIT 4

/* PI_INT_STATUS bits */
#define PI_CONTROL_ERROR_BIT 1
#define PI_INIT_DONE_BIT 0

static struct mnh_ddr_internal_state *_state;

static void mnh_ddr_disable_lp(void);
static void mnh_ddr_enable_lp(void);

/*
 * Write the clk dividers from given FSP index.
 *  fsp: the fsp index [0-3]
 *  pll_freeze: define whether the function uses HW PLL freeze funtionality
 *  inside the function to set all the dividers in one operation (with unfreeze)
 *  0 to set it off
 *  1 to set it on
 */
void mnh_ddr_write_clk_from_fsp(unsigned int fsp, int pll_freeze)
{
	if (fsp >= MNH_DDR_NUM_FSPS) {
		pr_err("%s invalid fsp 0x%x", __func__, fsp);
		return;
	}

	if (pll_freeze) {
		MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x4CD9);
		MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 1);
	}
	MNH_SCU_OUTf(CCU_CLK_DIV, LPDDR4_REFCLK_DIV,
		MNH_SCU_INxf(LPDDR4_FSP_SETTING, fsp, FSP_LPDDR4_REFCLK_DIV));
	MNH_SCU_OUTf(CCU_CLK_DIV, AXI_FABRIC_CLK_DIV,
		MNH_SCU_INxf(LPDDR4_FSP_SETTING, fsp, FSP_AXI_FABRIC_CLK_DIV));
	MNH_SCU_OUTf(CCU_CLK_DIV, PCIE_AXI_CLK_DIV,
		MNH_SCU_INxf(LPDDR4_FSP_SETTING, fsp, FSP_PCIE_AXI_CLK_DIV));
	MNH_SCU_OUTf(CCU_CLK_CTL, LP4_AXI_SYS200_MODE,
		MNH_SCU_INxf(LPDDR4_FSP_SETTING, fsp, FSP_SYS200_MODE));
	if (pll_freeze) {
		MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 0);
		MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0);
	}
}

static u32 mnh_ddr_sanity_check(void)
{
	/* just verify comm is actually up */
	u32 val = MNH_DDR_CTL_IN(00);

	/*
	 * If above register reads either 0 or all FF, it indicates that
	 * access to DDR CTL registers are not ready.
	 */
	if ((val == 0) || (val == 0xFFFFFFFF))
		return 0;
	else
		return 1;
}

/* read entire int_status */
u64 mnh_ddr_int_status(void)
{
	u64 int_stat = ((u64)MNH_DDR_CTL_IN(228) << 32) | MNH_DDR_CTL_IN(227);
	return int_stat;
}
EXPORT_SYMBOL(mnh_ddr_int_status);

int mnh_ddr_print_phy_status(void)
{
	int ret = 0;
	/* call from a context where lp has already been disabled
	 * so ctl and phy reg can be accessed. don't do it here
	 */
	if ((MNH_DDR_PHY_IN(00) != 0x76543210) ||
		(MNH_DDR_PHY_IN(256) != 0x76543210) ||
		(MNH_DDR_PHY_IN(384) != 0x76543210)) {
		pr_err("%s ERROR PHY 00: 0x%08x 256: 0x%08x 384: 0x%08x\n",
			__func__,
			MNH_DDR_PHY_IN(00),
			MNH_DDR_PHY_IN(256),
			MNH_DDR_PHY_IN(384));
		ret = -1;
	}

	if (MNH_DDR_PHY_INf(1099, PHY_AC_INIT_COMPLETE_OBS) != 0x000003f1) {
		pr_info("%s PHY_AC_INIT_COMPLETE_OBS: 0x%08x\n",
			__func__, MNH_DDR_PHY_INf(1099,
					PHY_AC_INIT_COMPLETE_OBS));
		ret = -1;
	}
	if (MNH_DDR_PHY_INf(1100, PHY_DS_INIT_COMPLETE_OBS) != 0x0000000f) {
		pr_info("%s PHY_DS_INIT_COMPLETE_OBS: 0x%08x\n",
			__func__, MNH_DDR_PHY_INf(1100,
					PHY_DS_INIT_COMPLETE_OBS));
		ret = -1;
	}
	return ret;
}
EXPORT_SYMBOL(mnh_ddr_print_phy_status);


/* clear entire int_status */
int mnh_ddr_clr_int_status(void)
{
	u64 stat = 0;

	MNH_DDR_CTL_OUT(229, 0xFFFFFFFF);
	MNH_DDR_CTL_OUT(230, 0x0F);
	stat = mnh_ddr_int_status();
	if (stat) {
		pr_err("%s: int stat not all clear: %llx\n",
			__func__, stat);
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
static int mnh_ddr_clr_int_status_bit(u8 sbit)
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
		pr_err("%s: bit %d is still set.\n", __func__, sbit);
		return -EIO;
	}
	return 0;
}

/* read single bit in PI_INT_STATUS */
static u32 mnh_ddr_pi_int_status_bit(u8 sbit)
{
	u32 status = 0;
	const u32 max_int_status_bit = 24;

	if (sbit > max_int_status_bit)
		return -EINVAL;

	status = MNH_DDR_PI_INf(172, PI_INT_STATUS);
	pr_debug("%s: PI_INT_STATUS = 0x%x.\n", __func__, status);

	status &= (1 << sbit);
	return status;
}

static int mnh_ddr_send_lp_cmd(u8 cmd)
{
	u32 timeout = 100000;

	pr_debug("%s sending cmd: 0x%x\n", __func__, cmd);
	MNH_DDR_CTL_OUTf(112, LP_CMD, cmd);

	while (!mnh_ddr_int_status_bit(LP_CMD_SBIT) && --timeout)
		udelay(1);

	if (!mnh_ddr_int_status_bit(LP_CMD_SBIT))
		return -ETIMEDOUT;

	return mnh_ddr_clr_int_status_bit(LP_CMD_SBIT);
}

/*
 * Both chip 0 and chip 1 are written.
 */
int mnh_ddr_write_mode_reg(u8 modereg, u8 modevalue)
{
	const u64 writeable = 0x0000010101D3FE1E;
	u32 val = 0;
	unsigned long timeout = 0;
	int ret = 0;

	if ((modereg >= 64) ||
		((writeable & (1ULL << modereg)) == 0)) {
		pr_err("%s %d is not writeable.\n",
			__func__, modereg);
		return -EIO;
	}

	pr_debug("%s LP_STATE is 0x%x\n",
		__func__, MNH_DDR_CTL_INf(121, LP_STATE));
	val = 0xFF & modereg;

	/*
	 * bit 24 indicates all chip selects
	 * bit 23 indicates indicates a single mode reg
	 */
	val |= (1 << 23) | (1 << 24);
	MNH_DDR_CTL_OUTf(140, WRITE_MODEREG, val);
	MNH_DDR_CTL_OUTf(160, MRSINGLE_DATA_0, modevalue);
	/* trigger write */
	val |= (1 << 25);
	MNH_DDR_CTL_OUTf(140, WRITE_MODEREG, val);

	timeout = jiffies + msecs_to_jiffies(500);
	while (!mnh_ddr_int_status_bit(MR_WRITE_SBIT) &&
	       time_before(jiffies, timeout)) {
		udelay(100);
	}

	if (mnh_ddr_int_status_bit(MR_WRITE_SBIT)) {
		mnh_ddr_clr_int_status_bit(MR_WRITE_SBIT);
	} else {
		pr_err("%s timeout on MR write done. %llx.",
			__func__, mnh_ddr_int_status());
		ret = -EIO;
	}
	val = MNH_DDR_CTL_INf(141, MRW_STATUS);
	if (val) {
		pr_err("%s ERROR status: 0x%x", __func__, val);
		ret = -EIO;
	}

	return ret;
}

static void mnh_ddr_enable_lp(void)
{
	MNH_DDR_CTL_OUTf(124, LP_AUTO_SR_MC_GATE_IDLE, 0xFF);
	if (MNH_DDR_CTL_INf(122, LP_AUTO_EXIT_EN) == 0)
		MNH_DDR_CTL_OUTf(122, LP_AUTO_EXIT_EN, 0xF);
	if (MNH_DDR_CTL_INf(122, LP_AUTO_MEM_GATE_EN) == 0)
		MNH_DDR_CTL_OUTf(122, LP_AUTO_MEM_GATE_EN, 0x4);
	if (MNH_DDR_CTL_INf(122, LP_AUTO_ENTRY_EN) == 0)
		MNH_DDR_CTL_OUTf(122, LP_AUTO_ENTRY_EN, 0x4);
}

static void mnh_ddr_disable_lp(void)
{
	MNH_DDR_CTL_OUTf(122, LP_AUTO_ENTRY_EN, 0x0);
	mnh_ddr_send_lp_cmd(LP_CMD_EXIT_LP);
}

static void mnh_ddr_init_internal_state(struct mnh_ddr_internal_state *_state,
					const struct mnh_ddr_reg_config *cfg,
					struct gpio_desc *iso_n)
{
	_state->iso_n = iso_n;
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

	_state->tref[0] = cfg->ctl[56] & 0xFFFF;
	_state->tref[1] = cfg->ctl[57] & 0xFFFF;
	_state->tref[2] = cfg->ctl[58] & 0xFFFF;
	_state->tref[3] = cfg->ctl[59] & 0xFFFF;
}

static void mnh_ddr_init_clocks(struct device *dev, int fsp)
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

	/* WA for old HW bug to keep AXI, PCIE_AXI clk divs as nonzero
	 * during the first FSP switch
	 */
	MNH_SCU_OUTf(CCU_CLK_DIV, AXI_FABRIC_CLK_DIV, 0x1);
	MNH_SCU_OUTf(CCU_CLK_DIV, PCIE_AXI_CLK_DIV, 0x3);

	WRITE_SCU_FSP(0);
	WRITE_SCU_FSP(1);
	WRITE_SCU_FSP(2);
	WRITE_SCU_FSP(3);

	WRITE_CLK_FROM_FSP(fsp);
	dev_dbg(dev, "%s lpddr4 pll locked", __func__);
	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LP4_FSP_SW_OVERRIDE, 0);
	/* MNH_PLL_PASSCODE_CLR */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x0);
}

static void mnh_ddr_clear_lpc_status(void)
{
	/* Paranoia */
	if (MNH_SCU_INf(SCU_IRQ_STATUS, LP4_FREQ_CHG_REQ)) {
		pr_info("%s LP4_FREQ_CHG_REQ already set, clearing", __func__);
		MNH_SCU_OUTf(SCU_IRQ_STATUS, LP4_FREQ_CHG_REQ, 1);
	}
	if (MNH_SCU_INf(SCU_IRQ_STATUS, LP4_LPC_CMD_DONE)) {
		pr_info("%s LP4_LPC_CMD_DONE already set, clearing", __func__);
		MNH_SCU_OUTf(SCU_IRQ_STATUS, LP4_LPC_CMD_DONE, 1);
	}
	if (MNH_SCU_INf(LPDDR4_LOW_POWER_STS, LPC_CMD_RSP)) {
		pr_info("%s LPC_CMD_RSP already set, clearing", __func__);
		MNH_SCU_OUTf(LPDDR4_LOW_POWER_STS, LPC_CMD_RSP, 1);
	}
	if (MNH_SCU_INf(LPDDR4_LOW_POWER_STS, LPC_CMD_DONE)) {
		pr_info("%s LPC_CMD_DONE already set, clearing", __func__);
		MNH_SCU_OUTf(LPDDR4_LOW_POWER_STS, LPC_CMD_DONE, 1);
	}
}

#define MR_TABLE_LEN 5
const u8 mrw_fsps[MNH_DDR_NUM_FSPS][MR_TABLE_LEN][2] = {
	{
		{  1, 0x06 },
		{  2, 0x00 },
		{  3, 0x31 },
		{ 11, 0x00 },
		{ 22, 0x00 }
	},
	{
		{  1, 0x06 },
		{  2, 0x00 },
		{  3, 0x31 },
		{ 11, 0x00 },
		{ 22, 0x00 },
	},
	{
		{  1, 0x26 },
		{  2, 0x12 },
		{  3, 0x31 },
		{ 11, 0x00 },
		{ 22, 0x00 },
	},
	{
		{  1, 0x46 },
		{  2, 0x24 },
		{  3, 0x31 },
		{ 11, 0x00 },
		{ 22, 0x00 },
	}
};

int mnh_ddr_sw_switch(int index)
{
	static int iteration;
	u8 fsop, fswr;
	u8 mr13val;
	int old_lpi_wakeup_en, show_log, timeout;
	int i, ret = -EIO;

	uint16_t upd_high[MNH_DDR_NUM_FSPS];
	uint16_t upd_norm[MNH_DDR_NUM_FSPS];

	if ((index < 0) || (index >= MNH_DDR_NUM_FSPS)) {
		pr_err("%s %d is not a valid FSP\n",
			__func__, index);
		return -EINVAL;
	} else if (MNH_DDR_CTL_INf(133, CURRENT_REG_COPY) == index) {
		pr_info("%s %d is already in use - skipping\n",
			__func__, index);
		return 0;
	}
	show_log = iteration++ % 1000;
	if ((show_log == 0) || (show_log == 1)) {
		pr_info("%s #%d fsp %d -> %d DLL RESET\n",
			__func__, (iteration - 1),
			MNH_DDR_CTL_INf(133, CURRENT_REG_COPY), index);
	}

	if (!MNH_SCU_INxf(LPDDR4_FSP_SETTING, index, FSP_SYS200_MODE))
		mnh_lpddr_sys200_mode(false);

	old_lpi_wakeup_en = MNH_DDR_CTL_INf(120, LPI_WAKEUP_EN);
	/* need to make sure to disable lp, so phy regs
	 * and DRAM mode regs can be accessed.
	 */
	mnh_ddr_disable_lp();
	mnh_ddr_clr_int_status();

	/* set software control */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x4CD9);
	MNH_SCU_OUTxf(LPDDR4_FSP_SETTING, index, FSP_SW_CTRL, 1);
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0);

	mnh_ddr_clear_lpc_status();

	/* step 0 */
	MNH_DDR_CTL_OUTf(221, INHIBIT_DRAM_CMD, 3);
	MNH_DDR_CTL_OUTf(120, LPI_WAKEUP_EN, 0);
	udelay(10);
	MNH_DDR_CTL_OUTf(54, AREFRESH, 1);
	/* CDNS added this line */
	MNH_DDR_PHY_OUTf(1098, PHY_INIT_UPDATE_CONFIG, 0);

	timeout = 1000;
	udelay(1); /* CDNS addition start */
	MNH_DDR_CTL_OUTf(112, LP_CMD, 1);
	while (!mnh_ddr_int_status_bit(LP_CMD_SBIT) &&
		timeout-- > 0) {
		udelay(1);
	}
	if (mnh_ddr_clr_int_status_bit(LP_CMD_SBIT)) {
		pr_err("%s %d LP_CMD not clearing\n",
			__func__, __LINE__);
		ret = -EIO;
		goto sw_switch_error_exit;
	}  /* CDNS addition end */

	timeout = 1000;
	while (!mnh_ddr_int_status_bit(INHIBIT_DRAM_CMD_SBIT) && timeout-- > 0)
		udelay(1);
	if (mnh_ddr_clr_int_status_bit(INHIBIT_DRAM_CMD_SBIT)) {
		pr_err("%s %d INHIBIT not clearing\n",
			__func__, __LINE__);
		ret = -EIO;
		goto sw_switch_error_exit;
	}

	MNH_DDR_CTL_OUTf(223, CTRLUPD_REQ_PER_AREF_EN, 0);
	MNH_DDR_CTL_OUTf(525, CTRLUPD_AREF_HP_ENABLE, 0);
	MNH_DDR_PHY_OUTf(1100, PHY_UPDATE_MASK, 1);
	/* step 1 */
	upd_high[0] = MNH_DDR_CTL_INf(88, UPD_CTRLUPD_HIGH_THRESHOLD_F0);
	upd_norm[0] = MNH_DDR_CTL_INf(88, UPD_CTRLUPD_NORM_THRESHOLD_F0);
	upd_high[1] = MNH_DDR_CTL_INf(91, UPD_CTRLUPD_HIGH_THRESHOLD_F1);
	upd_norm[1] = MNH_DDR_CTL_INf(90, UPD_CTRLUPD_NORM_THRESHOLD_F1);
	upd_high[2] = MNH_DDR_CTL_INf(93, UPD_CTRLUPD_HIGH_THRESHOLD_F2);
	upd_norm[2] = MNH_DDR_CTL_INf(93, UPD_CTRLUPD_NORM_THRESHOLD_F2);
	upd_high[3] = MNH_DDR_CTL_INf(96, UPD_CTRLUPD_HIGH_THRESHOLD_F3);
	upd_norm[3] = MNH_DDR_CTL_INf(95, UPD_CTRLUPD_NORM_THRESHOLD_F3);

	MNH_DDR_CTL_OUTf(88, UPD_CTRLUPD_HIGH_THRESHOLD_F0, 0);
	MNH_DDR_CTL_OUTf(88, UPD_CTRLUPD_NORM_THRESHOLD_F0, 0);
	MNH_DDR_CTL_OUTf(91, UPD_CTRLUPD_HIGH_THRESHOLD_F1, 0);
	MNH_DDR_CTL_OUTf(90, UPD_CTRLUPD_NORM_THRESHOLD_F1, 0);
	MNH_DDR_CTL_OUTf(93, UPD_CTRLUPD_HIGH_THRESHOLD_F2, 0);
	MNH_DDR_CTL_OUTf(93, UPD_CTRLUPD_NORM_THRESHOLD_F2, 0);
	MNH_DDR_CTL_OUTf(96, UPD_CTRLUPD_HIGH_THRESHOLD_F3, 0);
	MNH_DDR_CTL_OUTf(95, UPD_CTRLUPD_NORM_THRESHOLD_F3, 0);

	fsop = MNH_DDR_CTL_INf(169, FSP_OP_CURRENT);
	fswr = !MNH_DDR_CTL_INf(169, FSP_WR_CURRENT);

	mr13val = (fsop << 7) | (fswr << 6) | (1 << 4);
	if (mnh_ddr_write_mode_reg(13, mr13val)) {
		pr_err("%s %d error writing MR13\n",
			__func__, __LINE__);
		ret = -EIO;
		goto sw_switch_error_exit;
	}

	/* step 2 */
	for (i = 0; i < MR_TABLE_LEN; i++) {
		if (mnh_ddr_write_mode_reg(mrw_fsps[index][i][0],
				mrw_fsps[index][i][1])) {
			pr_err("%s %d error (%d %d) writing mr: 0x%02x\n",
				__func__, __LINE__, index, i,
				mrw_fsps[index][i][0]);
			ret = -EIO;
			goto sw_switch_error_exit;
		}
	}

	/* step 3 */
	fsop = (fsop == 1) ? 0 : 1;
	mr13val = (fsop << 7) | (fswr << 6) | (1 << 4) | (1 << 3);
	if (mnh_ddr_write_mode_reg(13, mr13val)) {
		pr_err("%s %d error writing MR13\n",
			__func__, __LINE__);
		ret = -EIO;
		goto sw_switch_error_exit;
	}
	fswr = (fswr == 1) ? 0 : 1;

	/* step 4 */
	mnh_ddr_send_lp_cmd(LP_CMD_SRPD);
	/* step 5 */
	/* removed using iso for switch */
	/* step 6 moved to step 0 */
	/* step 7 */
	MNH_DDR_PHY_OUTf(1099, PHY_DLL_RST_EN, 1);

	/* step 8 */
	/* Prepare memory controller for switch */
	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LPC_FREQ_CHG_COPY_NUM, index);
	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LPC_EXT_CMD, LP_CMD_FREQ_SWITCH);
	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LPC_EXT_CMD_REQ, 1);

	timeout = 1000;
	while (!MNH_SCU_INf(SCU_IRQ_STATUS, LP4_FREQ_CHG_REQ) &&
		(timeout-- > 0))
		udelay(1);

	if (!MNH_SCU_INf(SCU_IRQ_STATUS, LP4_FREQ_CHG_REQ)) {
		pr_err("%s: Missed SCU_IRQ_STATUS.LP4_FREQ_CHG_REQ!\n",
			__func__);
		ret = -ETIME;
		goto sw_switch_error_exit;
	}

	/* clear it */
	MNH_SCU_OUTf(SCU_IRQ_STATUS, LP4_FREQ_CHG_REQ, 1);
	/* load clock settings from the fsp of interest */
	WRITE_CLK_FROM_FSP(index);
	/* effect the clock change */
	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LP4_FSP_SW_OVERRIDE, 1);
	udelay(100);
	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LP4_FSP_SW_OVERRIDE, 0);

	/* step 9 */
	MNH_DDR_PHY_OUTf(1099, PHY_DLL_RST_EN, 2);
	/* step 10 */
	/* inform memory controller freq change is done */
	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LP4_FREQ_CHG_ACK, 1);
	timeout = 1000;
	while (!MNH_SCU_INf(LPDDR4_LOW_POWER_STS, LPC_CMD_DONE) &&
		(timeout-- > 0))
		udelay(1);

	if (!MNH_SCU_INf(LPDDR4_LOW_POWER_STS, LPC_CMD_DONE)) {
		pr_err("%s Missed: LPDDR4_LOW_POWER_STS.LPC_CMD_DONE\n",
			__func__);
		ret = -ETIME;
		goto sw_switch_error_exit;
	}

	/* steps 11 - 13 removed because we're not using iso */

	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LP4_FSP_SW_OVERRIDE, 0);
	/* clear done */
	MNH_SCU_OUTf(LPDDR4_LOW_POWER_STS, LPC_CMD_DONE, 1);
	/* step 14 */
	/* undo previous changes. */
	MNH_DDR_CTL_OUTf(223, CTRLUPD_REQ_PER_AREF_EN, 1);
	MNH_DDR_CTL_OUTf(525, CTRLUPD_AREF_HP_ENABLE, 1);
	/* CDNS added this line */
	MNH_DDR_PHY_OUTf(1098, PHY_INIT_UPDATE_CONFIG, 7);
	MNH_DDR_PHY_OUTf(1100, PHY_UPDATE_MASK, 0);
	MNH_DDR_CTL_OUTf(221, INHIBIT_DRAM_CMD, 0);
	MNH_DDR_CTL_OUTf(88, UPD_CTRLUPD_HIGH_THRESHOLD_F0, upd_high[0]);
	MNH_DDR_CTL_OUTf(88, UPD_CTRLUPD_NORM_THRESHOLD_F0, upd_norm[0]);
	MNH_DDR_CTL_OUTf(91, UPD_CTRLUPD_HIGH_THRESHOLD_F1, upd_high[1]);
	MNH_DDR_CTL_OUTf(90, UPD_CTRLUPD_NORM_THRESHOLD_F1, upd_norm[1]);
	MNH_DDR_CTL_OUTf(93, UPD_CTRLUPD_HIGH_THRESHOLD_F2, upd_high[2]);
	MNH_DDR_CTL_OUTf(93, UPD_CTRLUPD_NORM_THRESHOLD_F2, upd_norm[2]);
	MNH_DDR_CTL_OUTf(96, UPD_CTRLUPD_HIGH_THRESHOLD_F3, upd_high[3]);
	MNH_DDR_CTL_OUTf(95, UPD_CTRLUPD_NORM_THRESHOLD_F3, upd_norm[3]);

	MNH_DDR_CTL_OUTf(120, LPI_WAKEUP_EN, old_lpi_wakeup_en);
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x4CD9);
	MNH_SCU_OUTxf(LPDDR4_FSP_SETTING, index, FSP_SW_CTRL, 0);
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0);

	mnh_ddr_clr_int_status_bit(DFI_UPDATE_ERROR_SBIT);
	mnh_ddr_clr_int_status_bit(DFS_COMPLETE_SBIT);
	mnh_ddr_clr_int_status_bit(DFI_STATE_CHANGE_SBIT);
	ret = 0;

	if (MNH_SCU_INxf(LPDDR4_FSP_SETTING, index, FSP_SYS200_MODE))
		mnh_lpddr_sys200_mode(true);

sw_switch_error_exit:
	if (mnh_ddr_print_phy_status())
		ret = -EIO;

	return ret;
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

	for (fsp = 0; fsp < MNH_DDR_NUM_BANKED_FSPS; fsp++) {
		MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, fsp);
		for (index = 0; index < MNH_DDR_NUM_PHY_REG; index++)
			SAVE_DDR_PHY_REG_CONFIG(fsp, index);
	}
}

int mnh_ddr_suspend(struct device *dev)
{
	if (WARN_ON(!_state))
		return -ENOMEM;

	mnh_ddr_disable_lp();

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

	if (MNH_DDR_CTL_INf(133, CURRENT_REG_COPY) !=
		LPDDR_FREQ_FSP3) {
		/* resume to fsp3 */
		if (mnh_lpddr_freq_change(LPDDR_FREQ_FSP3)) {
			dev_err(dev, "%s ERROR suspend clock switch failed.",
				__func__);
			return -EIO;
		}
	}
	mnh_ddr_pull_config();

	mnh_ddr_send_lp_cmd(LP_CMD_DSRPD);
	dev_dbg(dev, "%s LP_STATE is 0x%x", __func__,
		MNH_DDR_CTL_INf(121, LP_STATE));

	/* Enable clock gating */
	MNH_SCU_OUTf(CCU_CLK_CTL, HALT_LP4CG_EN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, HALT_LP4_PLL_BYPCLK_CG_EN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, LP4PHY_PLL_BYPASS_CLKEN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, LP4_REFCLKEN, 0);

	udelay(1);
	MNH_DDR_ASSERT_ISO_N();
	dev_dbg(dev, "%s iso is asserted", __func__);

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_suspend);

int mnh_ddr_resume(struct device *dev)
{
	int index, fsp;
	unsigned long timeout = 0;
	unsigned long start_jiff, end_jiff;

	if (WARN_ON(!_state))
		return -ENOMEM;

	mnh_ddr_init_clocks(dev, LPDDR_FREQ_FSP3);

	if (!mnh_ddr_sanity_check())
		return -EIO;

	for (index = 0; index < MNH_DDR_NUM_CTL_REG; index++)
		WRITE_DDR_REG_CONFIG(ctl, index);

	for (index = 0; index < MNH_DDR_NUM_PI_REG; index++)
		WRITE_DDR_REG_CONFIG(pi, index);

	for (fsp = 0; fsp < MNH_DDR_NUM_BANKED_FSPS; fsp++) {
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

	MNH_DDR_DEASSERT_ISO_N();
	udelay(1000);
	MNH_DDR_CTL_OUTf(00, START, 1);
	start_jiff = jiffies;
	timeout = start_jiff + RESUME_TIMEOUT;

	dev_dbg(dev, "%s waiting for ctl init done.", __func__);
	while ((!mnh_ddr_int_status_bit(INIT_DONE_SBIT)) &&
			time_before(jiffies, timeout))
		udelay(10);

	end_jiff = jiffies;
	dev_dbg(dev, "%s time elapsed is %u ms",
		__func__, jiffies_to_msecs(end_jiff - start_jiff));

	if (!mnh_ddr_int_status_bit(INIT_DONE_SBIT)) {
		dev_err(dev, "%s time out on init done %llx.\n",
			__func__, mnh_ddr_int_status());
		return -ETIMEDOUT;
	}

	/* need to clear PWRUP_SREFRESH_EXIT to clear interrupt status bit 0 */
	MNH_DDR_CTL_OUTf(81, PWRUP_SREFRESH_EXIT, 0);
	dev_dbg(dev, "%s got init done %llx.\n", __func__,
		mnh_ddr_int_status());
	mnh_ddr_clr_int_status();
	mnh_ddr_enable_lp();

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_resume);

int mnh_ddr_po_init(struct device *dev, struct gpio_desc *iso_n)
{
	int index, setindex, pi_step;
	unsigned long timeout;
	const struct mnh_ddr_reg_config *cfg = &mnh_ddr_33_100_400_600;

	if (WARN_ON(!_state))
		return -ENOMEM;

	mnh_ddr_init_internal_state(_state, cfg, iso_n);
	dev_dbg(dev, "%s start.", __func__);

	/* deassert iso_n */
	MNH_DDR_DEASSERT_ISO_N();
	mnh_ddr_init_clocks(dev, LPDDR_FREQ_FSP0);

	if (!mnh_ddr_sanity_check())
		return -EIO;

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

	/* set the index back to 1 to enable PI WA training */
	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 1);
	/* Add SCU register change to enable SW switching for all 4 FSPs. */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x4CD9);
	MNH_SCU_OUTxf(LPDDR4_FSP_SETTING, 0, FSP_SW_CTRL, 1);
	MNH_SCU_OUTxf(LPDDR4_FSP_SETTING, 1, FSP_SW_CTRL, 1);
	MNH_SCU_OUTxf(LPDDR4_FSP_SETTING, 2, FSP_SW_CTRL, 1);
	MNH_SCU_OUTxf(LPDDR4_FSP_SETTING, 3, FSP_SW_CTRL, 1);
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x0);

	dev_dbg(dev, "%s begin training,", __func__);
	MNH_DDR_PI_OUTf(00, PI_START, 1);
	MNH_DDR_CTL_OUTf(00, START, 1);

	/*
	 * The 11 DFS events during PI training are to be tracked,
	 * and apply WA at correct points.
	 * 1.	FSP0 to FSP1
	 * 2.	FSP1 to FSP2
	 * 3.	FSP2 to FSP1
	 * 4.	FSP1 to FSP2
	 * 5.	FSP2 to FSP1 - Program POSTDIV of FSP2 to 1 at
	 * pi_freq_change_req event
	 * 6.	FSP1 to FSP2 - Program POSTDIV of FSP2 to 0 at
	 * pi_freq_change_req event, provide 400 MHz, set pi_freq_change_ack = 1
	 * 7.	FSP2 to FSP3
	 * 8.	FSP3 to FSP2
	 * 9.	FSP2 to FSP3
	 * 10.	FSP3 to FSP2 - Program POSTDIV of FSP2&FSP3 to 1 at
	 * pi_freq_change_req event, Provide 300 MHz clock instead of usual
	 * 400 MHz clock, set pi_freq_change_ack = 1
	 * 11.	FSP2 to FSP3 - Program POSTDIV of FSP2&FSP3 to 0 at
	 * pi_freq_change_req event, provide 600 MHz, set pi_freq_change_ack = 1
	 *
	 */
	dev_info(dev, "%s PI WA training,", __func__);
	for (pi_step = 1; pi_step <= 11; pi_step++) {
		timeout = 10000;
		while (!MNH_SCU_INf(SCU_IRQ_STATUS, LP4_FREQ_CHG_REQ) &&
			  (timeout-- > 0))
			udelay(1);
		if (!MNH_SCU_INf(SCU_IRQ_STATUS, LP4_FREQ_CHG_REQ)) {
			dev_err(dev, "%s: Missed SCU_IRQ_STATUS.LP4_FREQ_CHG_REQ! pi_step=%d, LPDDR4_REQ_FSP=0x%x\n",
			__func__,
			pi_step,
			MNH_SCU_INf(LPDDR4_LOW_POWER_STS, LPDDR4_REQ_FSP));
			return -ETIME;
		}
		dev_dbg(dev, "%s: pi_step=%d, LPDDR4_REQ_FSP=0x%x, INDEX=%d\n",
			__func__,
			pi_step,
			MNH_SCU_INf(LPDDR4_LOW_POWER_STS, LPDDR4_REQ_FSP),
			MNH_DDR_PHY_INf(1025, PHY_FREQ_SEL_INDEX));
		/* clear it */
		MNH_SCU_OUTf(SCU_IRQ_STATUS, LP4_FREQ_CHG_REQ, 1);

		/* Change PHY PLL POSTDIV at Step 5,6,10,11 */
		switch (pi_step) {
		case 5:
			/* change postdiv of CA PLL */
			MNH_DDR_PHY_OUTf(1046, PHY_PLL_CTRL_CA, 0x322);
			break;
		case  6:
			/* change postdiv of CA PLL for f2 register set */
			MNH_DDR_PHY_OUTf(1046, PHY_PLL_CTRL_CA, 0x122);
			break;
		case 10:
			/* change postdiv of CA PLL for f2/f3 register set */
			MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 1);
			MNH_DDR_PHY_OUTf(1046, PHY_PLL_CTRL_CA, 0x322);
			MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 2);
			MNH_DDR_PHY_OUTf(1046, PHY_PLL_CTRL_CA, 0x322);
			break;
		case 11:
			/* change postdiv of CA PLL for f2 register set */
			MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 1);
			MNH_DDR_PHY_OUTf(1046, PHY_PLL_CTRL_CA, 0x122);
			MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_INDEX, 2);
			MNH_DDR_PHY_OUTf(1046, PHY_PLL_CTRL_CA, 0x122);
			break;
		}

		/* Freeze the divider settings till all registers are updated */
		MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x4CD9);
		MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 1);
		MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x0);
		/* load clock settings from the fsp of interest based on PI's
		 * fsp request
		 */
		WRITE_CLK_FROM_FSP_NO_LOCK((MNH_SCU_INf(LPDDR4_LOW_POWER_STS,
						LPDDR4_REQ_FSP)));
		/* Provide 300 MHz clock */
		if (pi_step == 10)
			/* 1200 MHz div by 4 */
			MNH_SCU_OUTf(CCU_CLK_DIV, LPDDR4_REFCLK_DIV, 3);
		/* UnFreeze the divider settings till all regs are updated */
		MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x4CD9);
		MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 0);
		MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x0);
		/* effect the clock change */
		MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG,
				 LP4_FSP_SW_OVERRIDE, 1);
		/* inform memory controller freq change is done */
		MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG,
				 LP4_FREQ_CHG_ACK, 1);
		MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG,
				 LP4_FSP_SW_OVERRIDE, 0);
	}

	timeout = jiffies + msecs_to_jiffies(50);
	while (!(mnh_ddr_int_status_bit(INIT_DONE_SBIT) &&
			mnh_ddr_pi_int_status_bit(PI_INIT_DONE_BIT)) &&
			time_before(jiffies, timeout))
		udelay(100);

	if (!mnh_ddr_int_status_bit(INIT_DONE_SBIT) ||
		!mnh_ddr_pi_int_status_bit(PI_INIT_DONE_BIT)) {
		dev_err(dev, "%s timed out on init done. 0x%llx 0x%08x\n",
			__func__,
			mnh_ddr_int_status(),
			MNH_DDR_PI_INf(172, PI_INT_STATUS));
		return -ETIMEDOUT;
	}

	dev_dbg(dev, "%s got init done %llx.\n", __func__,
		 mnh_ddr_int_status());

	mnh_ddr_clr_int_status();
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F0_0, 1);
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F1_0, 1);
	MNH_DDR_CTL_OUTf(165, MR_FSP_DATA_VALID_F2_0, 1);
	MNH_DDR_CTL_OUTf(166, MR_FSP_DATA_VALID_F3_0, 1);

	dev_dbg(dev, "%s done\n", __func__);
	/* settings to take effect on resume */
	MNH_DDR_CTL_OUTf(23, PHY_INDEP_TRAIN_MODE, 0);
	MNH_DDR_CTL_OUTf(23, CDNS_INTRL0, 1);
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

	mnh_ddr_disable_lp();

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
			__func__, mnh_ddr_int_status());
	} else {
		result = MNH_DDR_CTL_INf(171, BIST_RESULT);
		dev_info(dev, "%s: result 0x%02x\n", __func__, result);
	}

	MNH_DDR_CTL_OUTf(171, BIST_GO, 0);
	mnh_ddr_clr_int_status();

	MNH_DDR_CTL_OUTf(223, IN_ORDER_ACCEPT,
		old_in_order_accept);

	mnh_ddr_enable_lp();
	return result;
}
EXPORT_SYMBOL(mnh_ddr_mbist);

int mnh_ddr_platform_init(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);

	_state = devm_kzalloc(dev, sizeof(struct mnh_ddr_internal_state),
			      GFP_KERNEL);
	if (!_state)
		return -ENOMEM;

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_ddr_platform_init);
