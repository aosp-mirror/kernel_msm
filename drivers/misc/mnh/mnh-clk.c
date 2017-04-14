/*
 *
 * MNH Clock Driver
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


#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include "mnh-hwio.h"
#include "mnh-hwio-bases.h"
#include "mnh-hwio-scu.h"
#include "mnh-clk.h"
#include "mnh-ddr.h"


#define PLL_UNLOCK 0x4CD9
#define REF_FREQ_SEL 0x8
#define MAX_STR_COPY 9
#define LP4_LPC_FREQ_SWITCH 0x8A
#define PLL_LOCK_TIMEOUT 100
#define REF_CLK_KHZ 19200
#define SYS200_CLK_KHZ 200000

enum mnh_clk_type {
	CPU_CLK = 0,
	IPU_CLK
};

enum mnh_lpddr_lpc_rsp_type {
	LPC_CMD_NOERR = 0,
	LPC_CMD_ERR = 1
};

enum mnh_ipu_clk_src {
	CPU_IPU_PLL = 0,/*Both CPU and IPU Clock is derived from same PLL */
	IPU_PLL	/* IPU Clock is derived from IPU PLL */
};

struct freq_reg_table {
	int fbdiv;
	int postdiv1;
	int postdiv2;
	int clk_div;
};

/* SYS200 frequency calculation tables
 * SYS200  FBDIV FBDIV	POSTDIV1 POSTDIV2 FOUTPOSTDIV CLKDIV CLKFreq
 * CPU	 200	 104	 2	  1	   200.000	0	 200.000
 * IPU	 200	 104	 2	  1	   200.000	1	 100.000
 */
static struct freq_reg_table sys200_reg_tables[] = {
	{104, 2, 1, 0},	/* CPU, 200 MHz */
	{104, 2, 1, 1}	/* IPU, 100 MHz */
};

/* CPU clock frequency calculation tables
 * Refclk  FBDIV VCO	POSTDIV1 POSTDIV2 FOUTPOSTDIV CLKDIV CLKFreq
 * SYS200                                                        200.000
 * 19.2	 125	 2400	 6	  1	   400.000	0	 400.000
 * 19.2	 125	 2400	 4	  1	   600.000	0	 600.000
 * 19.2	 125	 2400	 3	  1	   800.000	0	 800.000
 * 19.2	 99	 1900.8  2	  1	   950.400	0	 950.400
 */
static struct freq_reg_table cpu_reg_tables[] = {
	{125, 6, 2, 0},	/* 200 MHz */
	{125, 6, 1, 0},	/* 400 MHz */
	{125, 4, 1, 0},	/* 600 MHz */
	{125, 3, 1, 0},	/* 800 MHz */
	{99, 2, 1, 0}	/* 950 MHz */
};

/* If IPU clock is driven by CPU_IPU PLL
 * calculate IPU divider based on CPU clk and divider
 * CPU CLK < 850: IPU_CLK_DIV = ((CPU_CLK_DIV+1)*2-1)
 * CPU CLK > 850: IPU_CLK_DIV = ((CPU_CLK_DIV+1)*2)
 */
static inline int mnh_clk_get_ipu_div(int cpu_freq, int cpu_div)
{
	if (cpu_freq < 850)
		return ((cpu_div + 1) * 2) - 1;
	else
		return (cpu_div + 1) * 2;
}

/* IPU clock frequency calculation tables
 * Refclk  FBDIV  VCO	POSTDIV1 POSTDIV2 FOUTPOSTDIV CLKDIV CLKFreq
 * 19.2	 125	 2400	  6	    2	  400.000	1	 100.000
 * 19.2	 125	 2400	  6	    1	  400.000	1	 200.000
 * 19.2	 125	 2400	  4	    1	  600.000	1	 300.000
 * 19.2	 125	 2400	  6	    1	  400.000	0	 400.000
 * 19.2	 133	 2553.6   6	    1	  425.600	0	 425.600
 */
static struct freq_reg_table ipu_reg_tables[] = {
	{125, 6, 2, 1},	/* 100 MHz */
	{125, 6, 1, 1},	/* 200 MHz */
	{125, 4, 1, 1},	/* 300 MHz */
	{125, 6, 1, 0},	/* 400 MHz */
	{133, 6, 1, 0}	/* 425 MHz */
};

struct mnh_freq_cooling_device {
	struct device *dev;
	uint32_t regs;
	enum mnh_ipu_clk_src ipu_clk_src;
	enum mnh_cpu_freq_type cpu_freq;
	enum mnh_ipu_freq_type ipu_freq;
	enum mnh_lpddr_freq_type ddr_freq;
};

static struct mnh_freq_cooling_device *mnh_dev;

/**
 * CPU clock controller
 * @index: int with frquency table index info.
 * Return: 0 on success, an error code otherwise.
 *
 * 1PLL(CPU_IPU PLL) for CPU/IPU clocks. Since CPU and IPU clocks are derived
 * from same PLL in this mode, there would be restriction on achedivable clock
 * frequencies for CPU and IPU clocks. IPU clock would be half of CPU clock. Any
 * frequency changes is achieved by changing FBDIV(integer feedback division) of
 * the PLL(PLL output = FBDIV * REFCLK frequency).
 * Default CPU_CLK_DIV : 1, IPU_CLK_DIV: 2
 * CLK = (REFCLK FREQ * FBDIV) / ((POSTDIV1 * POSTDIV2) * (CLK_DIV + 1))
 */
int mnh_cpu_freq_change(int index)
{
	int lock = 0;
	int ipu_div = 0;
	int timeout = 0;
	int ret = 0;

	if (index < CPU_FREQ_MIN || index > CPU_FREQ_MAX)
		return -EINVAL;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, index);

	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to SYS200 mode */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0x1);

	/* Read current IPU clock source */
	mnh_dev->ipu_clk_src =
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	/* Latch current settings going to PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure FBDIV first and set POSTDIV1, POSTDIV2
	* Compute dividers based on REF_FREQ_SEL hardware strap
	* Check FBDIV * REFCLK is witin VCO range (950-3800MHz)
	*/
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, FBDIV,
		cpu_reg_tables[index].fbdiv);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV1,
			cpu_reg_tables[index].postdiv1);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV2,
		cpu_reg_tables[index].postdiv2);

	/* Set FOUTPOSTDIVPD = 1 to avoid glitches to output */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* Apply the updated PLL configurations  */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	/* Power up PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, PD, 0);

	/* Wait for minimum 128REFCLK, 6.7usec for 19.2MHz refclk
	* before checking PLL lock
	*/
	udelay(7);

	/* Check PLL is locked */
	do {
		lock = HW_INf(mnh_dev->regs,
			SCU, CPU_IPU_PLL_STS, LOCK);
		timeout++;
	} while ((timeout < PLL_LOCK_TIMEOUT) && (lock != 1));

	if ((timeout == PLL_LOCK_TIMEOUT) && (lock != 1)) {
		dev_err(mnh_dev->dev, "%s: failed to lock PLL\n", __func__);

		ret = -EBUSY;

		/* Power down CPU_IPU PLL */
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, PD, 1);
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

		goto fail_pll_lock;
	}

	/* Set FOUTPOSTDIVPD = 0 to ensure clk output is un-gated */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);

	/* Configure CPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV,
		cpu_reg_tables[index].clk_div);

	/* If IPU clock is driven by CPU_IPU PLL,
	*  configure IPU divider based on CPU divider value
	*  to make sure IPU clock does not go over its limit
	*/
	if (mnh_dev->ipu_clk_src == CPU_IPU_PLL) {
		if (index > CPU_FREQ_800)
			ipu_div = mnh_clk_get_ipu_div(950,
				cpu_reg_tables[index].clk_div);
		else
			ipu_div = mnh_clk_get_ipu_div(0,
				cpu_reg_tables[index].clk_div);

		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV, ipu_div);
	}

	/* Go back to CPU_IPU PLL output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL,
			CPU_IPU_SYS200_MODE, 0);

	mnh_dev->cpu_freq = index;
	if (mnh_dev->ipu_clk_src == CPU_IPU_PLL)
		mnh_dev->ipu_freq = index;

fail_pll_lock:
	/* Lock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	return ret;
}
EXPORT_SYMBOL(mnh_cpu_freq_change);

/**
 * IPU clock controller
 * @index: int with frquency table index info.
 * Return: 0 on success, an error code otherwise.
 *
 * Until IPU clock is configured, IPU clock is driven from PCIe or CPU_IPU PLL,
 * and once it is configured by driver, IPU PLL is used to control IPU clock.
 * To turn off IPU PLL, CPU frequency needs to be set to 200MHz to put
 * both CPU and IPU into SYS200 mode.
 */
int mnh_ipu_freq_change(int index)
{
	int lock = 0;

	if (index < IPU_FREQ_MIN || index > IPU_FREQ_MAX)
		return -EINVAL;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, index);

	/* Disable IPU_PLL clock output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLKEN, 0x0);

	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to stable clock before freq switch to avoid glitches */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC, CPU_IPU_PLL);

	/* Set FRZ_PLL_IN=1 to latch the current settings going to PLL */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure FBDIV first and set POSTDIV1, POSTDIV2 */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, FBDIV,
		ipu_reg_tables[index].fbdiv);
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV1,
		ipu_reg_tables[index].postdiv1);
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV2,
		ipu_reg_tables[index].postdiv2);

	/* Set FOUTPOSTDIVPD = 1 to avoid glitches to output */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* Apply the updated PLL configurations  */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	/* Power up PLL */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, PD, 0);

	/* Wait for minimum 128REFCLK, 6.7usec for 19.2MHz refclk
	* before checking PLL lock
	*/
	udelay(7);

	/* Check PLL is locked */
	do {
		lock = HW_INf(mnh_dev->regs, SCU, IPU_PLL_STS, LOCK);
	} while (lock != 1);

	/* Set FOUTPOSTDIVPD = 0 to ensure clk output is un-gated */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);

	/* Configure IPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV,
	ipu_reg_tables[index].clk_div);

	/* Go back to IPU PLL output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC, IPU_PLL);

	mnh_dev->ipu_freq = index;
	mnh_dev->ipu_clk_src =
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	/* Lock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	/* Enable IPU_PLL clock output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLKEN, 0x1);

	return 0;
}
EXPORT_SYMBOL(mnh_ipu_freq_change);

/**
 * LPDDR clock control driver
 * @index: int with frquency table index info.
 * Return: 0 on success, an error code otherwise.
 *
 * LPDDR clock is controlled by LPC instead of using direct PLL configuration.
 * Precondition:
 * LPDDR refclk pll should be enabled at cold boot and resume
 * LPDDR FSPx registers should be configured at cold boot and resume
 */
int mnh_lpddr_freq_change(int index)
{
	int status = 0;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, index);

	if (index < LPDDR_FREQ_MIN || index > LPDDR_FREQ_MAX)
		return -EINVAL;

	/* Check the requested FSP is already in use */
	mnh_dev->ddr_freq = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
		LPDDR4_CUR_FSP);
	if (mnh_dev->ddr_freq == index) {
		dev_dbg(mnh_dev->dev, "%s: requested fsp%d is in use\n",
			__func__, index);
		return 0;
	}

	/* Disable LPC SW override */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG,
		LP4_FSP_SW_OVERRIDE, 0);

	/* Configure FSP index */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG,
		LPC_FREQ_CHG_COPY_NUM, index);

	/* Configure LPC cmd for frequency switch */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG,
		LPC_EXT_CMD, LP4_LPC_FREQ_SWITCH);

	/* Initiate LPC cmd to LPDDR controller */
	dev_dbg(mnh_dev->dev, "%s: lpddr freq switching from fsp%d to fsp%d\n",
		__func__, mnh_dev->ddr_freq, index);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG, LPC_EXT_CMD_REQ, 1);

	/* Wait until LPC cmd process is done */
	do {
		status = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
			LPC_CMD_DONE);
	} while (status != 1);

	/* Clear LPC cmd status */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS, LPC_CMD_DONE, 1);

	/* Check LPC error status */
	if (HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS, LPC_CMD_RSP)
		== LPC_CMD_ERR) {
		/* Clear error status */
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
			LPC_CMD_RSP, 1);
		dev_err(mnh_dev->dev, "Failed to process lpc cmd:0x%x\n",
			LP4_LPC_FREQ_SWITCH);
		return -EIO;
	}

	/* Check FSPx switch status */
	if (HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS, LPDDR4_CUR_FSP)
		!= index) {
		dev_err(mnh_dev->dev, "Failed to switch to fsp%d\n", index);
		return -EIO;
	}

	mnh_dev->ddr_freq = index;
	mnh_ddr_clr_int_status(mnh_dev->dev);
	return 0;
}
EXPORT_SYMBOL(mnh_lpddr_freq_change);

/**
 * LPDDR clock control driver
 * Return: 0 on success, an error code otherwise.
 *
 * LPDDR clock is derived from sys200 clk instead of separate lpddr clk
 */
int mnh_lpddr_sys200_mode(void)
{
	/* Switch lpddr to SYS200 mode */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, LP4_AXI_SYS200_MODE, 0x1);

	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Power down LPDDR PLL */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 1);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, PD, 1);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, FOUTPOSTDIVPD, 1);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, BYPASS, 1);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 0);

	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL(mnh_lpddr_sys200_mode);

/**
 * CPU and IPU SYS200 clock control driver
 * Return: 0 on success, an error code otherwise.
 *
 * CPU and IPU clock is derived from sys200 clk instead of separate plls
 */
int mnh_cpu_ipu_sys200_mode(void)
{
	dev_dbg(mnh_dev->dev, "%s\n", __func__);

	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to SYS200 mode */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0x1);

	/* Read current IPU clock source */
	mnh_dev->ipu_clk_src =
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	if (mnh_dev->ipu_clk_src == IPU_PLL) {
		/* Change clk source to CPU_IPU_PLL */
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL,
			IPU_CLK_SRC, CPU_IPU_PLL);

		/* IPU: Latch current settings to go into PLL */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 1);

		/* IPU: Power down PLL */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, PD, 1);
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL,
			FOUTPOSTDIVPD, 1);

		/* IPU: Apply PLL configurations */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 0);
	}

	/* Configure IPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV,
	sys200_reg_tables[IPU_CLK].clk_div);

	/* Configure CPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV,
		sys200_reg_tables[CPU_CLK].clk_div);

	/* CPU_IPU: Latch current settings to go into PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* CPU_IPU: Power down PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, PD, 1);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* CPU_IPU: Apply PLL configurations */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	mnh_dev->cpu_freq = 0;
	mnh_dev->ipu_freq = 0;

	/* Lock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL(mnh_cpu_ipu_sys200_mode);

/* Frequency calculation by PLL configuration
 * @cfg: struct freq_reg_table with pll config information.
 * Return: freq MHz.
 *
 * This returns current frequency in MHz unit
 * freq = (refclk*fbdiv)/((postdiv1*postdiv2)*(clk_div+1))
 */
static int mnh_freq_get_by_pll(struct freq_reg_table cfg, int sys200)
{
	uint32_t freq_khz;

	if (sys200)
		freq_khz = (SYS200_CLK_KHZ)/(cfg.clk_div+1);
	else
		freq_khz = (REF_CLK_KHZ*cfg.fbdiv)/
			((cfg.postdiv1*cfg.postdiv2)*(cfg.clk_div+1));

	return (freq_khz/1000);
}

static const char * const cpu_freq_str[] = {"200", "400", "600", "800", "950"};
static ssize_t cpu_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct freq_reg_table pll_cfg;
	int sys200;

	/* Check CPU is in SYS200 mode */
	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);

	/* Calculate frequency by PLL configuration */
	pll_cfg.fbdiv =
		HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, FBDIV);
	pll_cfg.postdiv1 =
		HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV1);
	pll_cfg.postdiv2 =
		HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV2);
	pll_cfg.clk_div = HW_INf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV);

	return snprintf(buf, PAGE_SIZE, "%dMHz\n",
			mnh_freq_get_by_pll(pll_cfg, sys200));
}

static ssize_t cpu_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int i, err;

	dev_dbg(mnh_dev->dev, "%s: %s\n", __func__, buf);
	for (i = 0; i < ARRAY_SIZE(cpu_freq_str); i++) {
		if (!strncmp(buf, cpu_freq_str[i], strlen(cpu_freq_str[i]))) {
			err = mnh_cpu_freq_change(i);
			if (!err)
				return count;
			else
				return -EIO;
		}
	}

	dev_err(mnh_dev->dev, "invalid freq: %s\n", buf);
	return -EINVAL;

}

static const char * const ipu_freq_str[] = {"100", "200", "300", "400", "425"};
static ssize_t ipu_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct freq_reg_table pll_cfg;
	int sys200, clk_src;

	/* Check IPU is in SYS200 mode */
	clk_src = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);
	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200 && (clk_src == IPU_PLL))
		sys200 = 0;

	/* Calculate frequency by PLL configuration */
	if (clk_src == CPU_IPU_PLL) {
		pll_cfg.fbdiv = HW_INf(mnh_dev->regs, SCU,
				CPU_IPU_PLL_INTGR_DIV, FBDIV);
		pll_cfg.postdiv1 = HW_INf(mnh_dev->regs, SCU,
				CPU_IPU_PLL_INTGR_DIV, POSTDIV1);
		pll_cfg.postdiv2 = HW_INf(mnh_dev->regs, SCU,
				CPU_IPU_PLL_INTGR_DIV, POSTDIV2);
	} else {
		pll_cfg.fbdiv = HW_INf(mnh_dev->regs, SCU,
				IPU_PLL_INTGR_DIV, FBDIV);
		pll_cfg.postdiv1 = HW_INf(mnh_dev->regs, SCU,
				IPU_PLL_INTGR_DIV, POSTDIV1);
		pll_cfg.postdiv2 = HW_INf(mnh_dev->regs, SCU,
				IPU_PLL_INTGR_DIV, POSTDIV2);
	}
	pll_cfg.clk_div = HW_INf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV);
	return snprintf(buf, PAGE_SIZE, "%dMHz\n",
			mnh_freq_get_by_pll(pll_cfg, sys200));
}

static ssize_t ipu_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int i, err;

	dev_dbg(mnh_dev->dev, "%s: %s\n", __func__, buf);
	for (i = 0; i < ARRAY_SIZE(ipu_freq_str); i++) {
		if (!strncmp(buf, ipu_freq_str[i], strlen(ipu_freq_str[i]))) {
			err = mnh_ipu_freq_change(i);
			if (!err)
				return count;
			else
				return -EIO;
		}
	}

	dev_err(mnh_dev->dev, "invalid freq: %s\n", buf);
	return -EINVAL;
}

static ssize_t lpddr_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	uint32_t var = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
				LPDDR4_CUR_FSP);

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
	return snprintf(buf, PAGE_SIZE, "FSP%d\n", var);
}

static ssize_t lpddr_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
	if (!mnh_lpddr_freq_change(var))
		return count;
	else
		return -EIO;
}

static ssize_t ipu_clk_src_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	mnh_dev->ipu_clk_src =
		HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	return scnprintf(buf, MAX_STR_COPY, "%s\n",
		(mnh_dev->ipu_clk_src == CPU_IPU_PLL) ? "CPU_IPU":"IPU");
}

static ssize_t sys200_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int sys200, clk_src;

	clk_src = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);
	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200 && (clk_src == IPU_PLL))
		sys200 = 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", sys200);
}

static ssize_t sys200_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	if (var == 1) {
		dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
		if (!mnh_cpu_ipu_sys200_mode())
			return count;
	}
	return -EIO;
}

static DEVICE_ATTR(cpu_freq, S_IWUSR | S_IRUGO,
		cpu_freq_get, cpu_freq_set);
static DEVICE_ATTR(ipu_freq, S_IWUSR | S_IRUGO,
		ipu_freq_get, ipu_freq_set);
static DEVICE_ATTR(lpddr_freq, S_IWUSR | S_IRUGO,
		lpddr_freq_get, lpddr_freq_set);
static DEVICE_ATTR(ipu_clk_src, S_IRUGO,
		ipu_clk_src_get, NULL);
static DEVICE_ATTR(sys200, S_IWUSR | S_IRUGO,
		sys200_freq_get, sys200_freq_set);

static struct attribute *freq_dev_attributes[] = {
	&dev_attr_cpu_freq.attr,
	&dev_attr_ipu_freq.attr,
	&dev_attr_lpddr_freq.attr,
	&dev_attr_ipu_clk_src.attr,
	&dev_attr_sys200.attr,
	NULL
};

static struct attribute_group mnh_freq_cooling_group = {
	.name = "mnh_freq_cool",
	.attrs = freq_dev_attributes
};



static int init_sysfs(struct device *dev, struct kobject *sysfs_kobj)
{
	int ret;

	ret = sysfs_create_group(sysfs_kobj, &mnh_freq_cooling_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs\n");
		return -EINVAL;
	}

	return 0;
}

static void clean_sysfs(struct kobject *sysfs_kobj)
{
	sysfs_remove_group(sysfs_kobj, &mnh_freq_cooling_group);
}

int mnh_clk_init(struct device *dev, uint32_t baseadress)
{
	dev_dbg(dev, "%s\n", __func__);

	mnh_dev = devm_kzalloc(dev, sizeof(*mnh_dev),
			GFP_KERNEL);
	if (!mnh_dev)
		return -ENOMEM;

	/* Set baseadress for SCU */
	mnh_dev->regs = baseadress;
	mnh_dev->dev = dev;

	/* TBD - Acquire current frequency */

	init_sysfs(dev, &dev->kobj);

	return 0;
}

void mnh_clk_clean(struct device *dev)
{
	clean_sysfs(&dev->kobj);
}
