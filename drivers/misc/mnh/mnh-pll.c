/*
*
* MNH PLL APIs
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
#include "mnh-hwio-scu.h"
#include "mnh-pll.h"


/***********************************/
/* shared pll macros and functions */
/*             begin               */
/*                                 */
/***********************************/
/* pllname : PCIE_REFCLK , LPDDR4_REFCLK , CPU_IPU , IPU */
/* subreg: FRAC_DIV , INTGR_DIV , CTRL, STS */
#define MNH_PLL_INf(pllname, subreg, fld)	\
HW_INf(HWIO_SCU_BASE_ADDR, SCU, pllname##_PLL_##subreg, fld)
#define MNH_PLL_OUTf(pllname, subreg, fld, val)	\
HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, pllname##_PLL_##subreg, fld, val)

#define MNH_CLK_CTL_INf(fld) \
HW_INf(HWIO_SCU_BASE_ADDR, SCU, CCU_CLK_CTL, fld)
#define MNH_CLK_CTL_OUTf(fld, val) \
HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, CCU_CLK_CTL, fld, val)

/* set (unlock) passcode, clr (relock) passcode */
#define MNH_PLL_PASSCODE_SET() \
HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, PLL_PASSCODE, PASSCODE, 0x4CD9)
#define MNH_PLL_PASSCODE_CLR()  \
HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, PLL_PASSCODE, PASSCODE, 0)

/*******************************/
/*           begin             */
/* lpddr4 macros and functions */
/*******************************/
#define MNH_LPC_INf(fld)	\
HW_INf(HWIO_SCU_BASE_ADDR, SCU, LPDDR4_LOW_POWER_CFG, fld)
#define MNH_LPC_OUTf(fld, val)	\
HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, LPDDR4_LOW_POWER_CFG, fld, val)

#define MNH_FSP_INx(inst) \
HW_INx(HWIO_SCU_BASE_ADDR, SCU, LPDDR4_FSP_SETTING, inst)
#define MNH_FSP_OUTx(inst, val) \
HW_OUTx(HWIO_SCU_BASE_ADDR, SCU, LPDDR4_FSP_SETTING, inst, val)

#define MNH_FSP_SHFT(fld, val) \
((val << HWIO_SCU_LPDDR4_FSP_SETTING_FSP_##fld##_FLDSHFT) & \
HWIO_SCU_LPDDR4_FSP_SETTING_FSP_##fld##_FLDMASK)

#define MNH_FSP_MASK(fld) HWIO_SCU_LPDDR4_FSP_SETTING_FSP_##fld##_FLDMASK


static int mnh_pll_is_valid_fsp(int fsp)
{
	if ((fsp >= 0) && (fsp < MNH_NUM_FSPS))
		return 1;
	else
		return 0;
}

static int mnh_pll_is_valid_lpddr4_freq(int freq)
{
	switch (freq) {
	case MNH_LP4_FREQ_BOOT:
	case MNH_LP4_FREQ_800:
	case MNH_LP4_FREQ_1200:
	case MNH_LP4_FREQ_1600:
	case MNH_LP4_FREQ_1920:
	case MNH_LP4_FREQ_2400:
		return 1;
	default:
		return 0;
	}
}

static int mnh_pll_set_fsp_freq(int fsp, int freq)
{
	u32 fsp_val = 0;
	u32 fsp_mask =
		MNH_FSP_MASK(SW_CTRL) |
		MNH_FSP_MASK(SYS200_MODE) |
		MNH_FSP_MASK(PCIE_AXI_CLK_DIV) |
		MNH_FSP_MASK(AXI_FABRIC_CLK_DIV) |
		MNH_FSP_MASK(LPDDR4_REFCLK_DIV) |
		MNH_FSP_MASK(FBDIV);

	const struct mnh_fsp_setting freqs[] = {
		/*
		sw_ctrl, sys200,
		fbdiv, pcie_axi_clk_div,
		axi_fab_clk_div, lpddr4_refclk_div;
		*/
		{ 0, 1, 125, 0, 0, 0 }, /* BOOT */
		{ 0, 0, 125, 2, 2, 2 }, /*  200   800 */
		{ 0, 0, 125, 4, 3, 3 }, /*  300  1200 */
		{ 0, 0, 125, 4, 2, 2 }, /*  400  1600 */
		{ 0, 0, 125, 1, 0, 0 }, /*  480  1920 */
		{ 0, 0, 125, 3, 1, 1 }, /*  600  2400 */
	};

	if (mnh_pll_is_valid_lpddr4_freq(freq) &&
		mnh_pll_is_valid_fsp(fsp)) {
		fsp_val = MNH_FSP_INx(fsp) & ~fsp_mask;
		fsp_val |=
			MNH_FSP_SHFT(SW_CTRL,		 freqs[freq].sw_ctrl) |
			MNH_FSP_SHFT(SYS200_MODE,	 freqs[freq].sys200) |
			MNH_FSP_SHFT(PCIE_AXI_CLK_DIV,	 freqs[freq].pcie_axi_clk_div) |
			MNH_FSP_SHFT(AXI_FABRIC_CLK_DIV, freqs[freq].axi_fab_clk_div) |
			MNH_FSP_SHFT(LPDDR4_REFCLK_DIV,	 freqs[freq].lpddr4_refclk_div) |
			MNH_FSP_SHFT(FBDIV,		 freqs[freq].fbdiv);
		MNH_FSP_OUTx(fsp, fsp_val);
		return 0;
	} else {
		return -1;
	}
}

int mnh_pll_lpddr4_is_locked(void)
{
	return MNH_PLL_INf(LPDDR4_REFCLK, STS, LOCK);
}

/* LPDDR4 PI Frequency Switching - Training at desired frequency ...*/
int mnh_pll_lpddr4_boot_settings(int freq0, int freq1, int freq2, int freq3)
{
	int ret = 0;
	if (mnh_pll_is_valid_lpddr4_freq(freq0) &&
		mnh_pll_is_valid_lpddr4_freq(freq1) &&
		mnh_pll_is_valid_lpddr4_freq(freq2) &&
		mnh_pll_is_valid_lpddr4_freq(freq3)) {
		/* 1. */
		MNH_LPC_OUTf(LP4_FSP_SW_OVERRIDE, 0);
		/* 2. */
		MNH_LPC_OUTf(LP4_PI_FREQCHG_EN, 1);
		/* 3. */
		MNH_PLL_PASSCODE_SET();
		/* 4. validate power on defaults? */

		/* 5. */
		ret |= mnh_pll_set_fsp_freq(0, freq0);
		ret |= mnh_pll_set_fsp_freq(1, freq1);
		ret |= mnh_pll_set_fsp_freq(2, freq2);
		ret |= mnh_pll_set_fsp_freq(3, freq3);

		/* 6. */
		MNH_LPC_OUTf(LP4_FSP_SW_OVERRIDE, 1);
		MNH_PLL_OUTf(LPDDR4_REFCLK, CTRL, FRZ_PLL_IN, 1);

		/* keep HW defaults in INTGR_DIV for now */

		MNH_PLL_OUTf(LPDDR4_REFCLK, CTRL, FOUTPOSTDIVPD, 1);
		MNH_PLL_OUTf(LPDDR4_REFCLK, CTRL, FRZ_PLL_IN, 0);
		MNH_PLL_OUTf(LPDDR4_REFCLK, CTRL, PD, 0);

		do {
			udelay(10);
		} while (!MNH_PLL_INf(LPDDR4_REFCLK, STS, LOCK));

		MNH_PLL_OUTf(LPDDR4_REFCLK, CTRL, FOUTPOSTDIVPD, 0);
		/* 7. */
		MNH_PLL_PASSCODE_CLR();
	} else {
		ret = -1;
	}

	return ret;
}

void mnh_pll_set_lpddr4_ref_clock_en(u32 val)
{
	val = (val == 0) ? 0 : 1;
	MNH_CLK_CTL_OUTf(LP4_REFCLKEN, val);
}

/*******************************/
/* lpddr4 macros and functions */
/*           end             */
/*******************************/
