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

#define SCU_IN(...) \
	HW_IN(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)
#define SCU_INf(...) \
	HW_INf(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)
#define SCU_INxf(...) \
	HW_INxf(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)
#define SCU_OUT(...) \
	HW_OUT(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)
#define SCU_OUTf(...) \
	HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, __VA_ARGS__)

#define PLL_UNLOCK 0x4CD9
#define LP4_LPC_FREQ_SWITCH 0x8A
#define REF_CLK_KHZ 19200
#define SYS200_CLK_KHZ 200000
#define PLL_LOCK_TIMEOUT 100

enum mnh_clk_type {
	CPU_CLK = 0,
	IPU_CLK
};

enum mnh_lpddr_lpc_rsp_type {
	LPC_CMD_NOERR = 0,
	LPC_CMD_ERR = 1
};

enum mnh_ipu_clk_src {
	CPU_IPU_PLL = 0,
	IPU_PLL
};

struct freq_reg_table {
	char *freq_str;
	int fbdiv;
	int postdiv1;
	int postdiv2;
	int clk_div;
};

/* SYS200 frequency calculation table */
static struct freq_reg_table sys200_reg_table[] = {
	{"200", 104, 2, 1, 0}, /* CPU_CLK */
	{"100", 104, 2, 1, 1}  /* IPU_CLK */
};

/* CPU clock frequency calculation table */
static struct freq_reg_table cpu_reg_table[] = {
	{"200", 125, 6, 2, 0},
	{"400", 125, 6, 1, 0},
	{"600", 125, 4, 1, 0},
	{"800", 125, 3, 1, 0},
	{"950",  99, 2, 1, 0}
};

/* IPU clock frequency calculation table */
static struct freq_reg_table ipu_reg_table[] = {
	{"100", 125, 6, 2, 1},
	{"200", 125, 6, 1, 1},
	{"300", 125, 4, 1, 1},
	{"400", 125, 6, 1, 0},
	{"425", 133, 6, 1, 0}
};

struct mnh_clk {
	struct device *dev;
};

static struct mnh_clk *mnh_clk;

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

/* Frequency calculation by PLL configuration
 * @cfg: struct freq_reg_table with pll config information.
 * Return: freq MHz.
 *
 * This returns current frequency in MHz unit
 */
static int mnh_freq_get_by_pll(struct freq_reg_table cfg, int sys200)
{
	uint32_t freq_khz;

	if (sys200)
		freq_khz = (SYS200_CLK_KHZ) / (cfg.clk_div + 1);
	else
		freq_khz = (REF_CLK_KHZ * cfg.fbdiv) /
			((cfg.postdiv1 * cfg.postdiv2) * (cfg.clk_div + 1));

	return (freq_khz / 1000);
}

static int mnh_get_cpu_pllcfg(struct freq_reg_table *pllcfg)
{
	int sys200;

	sys200 = SCU_INf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE);

	pllcfg->fbdiv = SCU_INf(CPU_IPU_PLL_INTGR_DIV, FBDIV);
	pllcfg->postdiv1 = SCU_INf(CPU_IPU_PLL_INTGR_DIV, POSTDIV1);
	pllcfg->postdiv2 = SCU_INf(CPU_IPU_PLL_INTGR_DIV, POSTDIV2);
	pllcfg->clk_div = SCU_INf(CCU_CLK_DIV, CPU_CLK_DIV);

	return mnh_freq_get_by_pll(*pllcfg, sys200);
}

static int mnh_get_ipu_pllcfg(struct freq_reg_table *pllcfg)
{
	int sys200, clk_src;

	clk_src = SCU_INf(CCU_CLK_CTL, IPU_CLK_SRC);
	sys200 = SCU_INf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200 && (clk_src == IPU_PLL))
		sys200 = 0;

	if (clk_src == CPU_IPU_PLL) {
		pllcfg->fbdiv = SCU_INf(CPU_IPU_PLL_INTGR_DIV, FBDIV);
		pllcfg->postdiv1 = SCU_INf(CPU_IPU_PLL_INTGR_DIV, POSTDIV1);
		pllcfg->postdiv2 = SCU_INf(CPU_IPU_PLL_INTGR_DIV, POSTDIV2);
	} else {
		pllcfg->fbdiv = SCU_INf(IPU_PLL_INTGR_DIV, FBDIV);
		pllcfg->postdiv1 = SCU_INf(IPU_PLL_INTGR_DIV, POSTDIV1);
		pllcfg->postdiv2 = SCU_INf(IPU_PLL_INTGR_DIV, POSTDIV2);
	}
	pllcfg->clk_div = SCU_INf(CCU_CLK_DIV, IPU_CLK_DIV);

	return mnh_freq_get_by_pll(*pllcfg, sys200);
}

static int mnh_get_cpu_freq(void)
{
	struct freq_reg_table pllcfg;

	return mnh_get_cpu_pllcfg(&pllcfg);
}

static int mnh_get_ipu_freq(void)
{
	struct freq_reg_table pllcfg;

	return mnh_get_ipu_pllcfg(&pllcfg);
}

int mnh_cpu_freq_to_index(void)
{
	int fbdiv, postdiv1, postdiv2, clk_div;
	int sys200, i;

	sys200 = SCU_INf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200)
		return 0;

	fbdiv = SCU_INf(CPU_IPU_PLL_INTGR_DIV, FBDIV);
	postdiv1 = SCU_INf(CPU_IPU_PLL_INTGR_DIV, POSTDIV1);
	postdiv2 = SCU_INf(CPU_IPU_PLL_INTGR_DIV, POSTDIV2);
	clk_div = SCU_INf(CCU_CLK_DIV, CPU_CLK_DIV);

	for (i = 0; i < ARRAY_SIZE(cpu_reg_table); i++) {
		if ((fbdiv == cpu_reg_table[i].fbdiv) &&
		    (postdiv1 == cpu_reg_table[i].postdiv1) &&
		    (postdiv2 == cpu_reg_table[i].postdiv2) &&
		    (clk_div == cpu_reg_table[i].clk_div))
			return i;
	}

	return -EINVAL;
}

int mnh_ipu_freq_to_index(void)
{
	int fbdiv, postdiv1, postdiv2, clk_div;
	int sys200, ipu_clk_src;
	int i;
	const struct freq_reg_table *table = ipu_reg_table;

	sys200 = SCU_INf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200)
		return 0;

	ipu_clk_src = SCU_INf(CCU_CLK_CTL, IPU_CLK_SRC);
	if (ipu_clk_src == CPU_IPU_PLL)
		return mnh_cpu_freq_to_index();

	fbdiv = SCU_INf(IPU_PLL_INTGR_DIV, FBDIV);
	postdiv1 = SCU_INf(IPU_PLL_INTGR_DIV, POSTDIV1);
	postdiv2 = SCU_INf(IPU_PLL_INTGR_DIV, POSTDIV2);
	clk_div = SCU_INf(CCU_CLK_DIV, IPU_CLK_DIV);

	for (i = 0; i < ARRAY_SIZE(ipu_reg_table); i++) {
		if ((fbdiv == table[i].fbdiv) &&
		    (postdiv1 == table[i].postdiv1) &&
		    (postdiv2 == table[i].postdiv2) &&
		    (clk_div == table[i].clk_div))
			return i;
	}

	return -EINVAL;
}

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
	int timeout = 0;
	int sys200, ipu_div = 0;
	const struct freq_reg_table *pllcfg;
	bool use_sys200;
	int ipu_clk_src;
	int cpu_freq, old_cpu_freq;
	int clk_div;

	if (!mnh_clk)
		return -ENODEV;

	if (index < CPU_FREQ_MIN || index > CPU_FREQ_MAX)
		return -EINVAL;

	dev_dbg(mnh_clk->dev, "%s: %d\n", __func__, index);

	/* Get PLL config from reg table */
	pllcfg = &cpu_reg_table[index];

	/* Get current SYS200 mode */
	sys200 = SCU_INf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE);

	/* Calculate frequency from PLL config */
	old_cpu_freq = mnh_get_cpu_freq();
	cpu_freq = mnh_freq_get_by_pll(*pllcfg, 0);

	/* Check to see if we need to change the frequency */
	if (old_cpu_freq == cpu_freq) {
		dev_dbg(mnh_clk->dev, "%s: already at desired frequency\n",
			__func__);
		return 0;
	}

	/* Check to see if we should use SYS200 mode */
	if (cpu_freq == 200) {
		use_sys200 = true;
		clk_div = 0;
	} else {
		use_sys200 = false;
		clk_div = pllcfg->clk_div;
	}

	/* Unlock PLL access */
	SCU_OUTf(PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to SYS200 mode */
	SCU_OUTf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0x1);

	/* Read current IPU clock source */
	ipu_clk_src = SCU_INf(CCU_CLK_CTL, IPU_CLK_SRC);

	/* Latch current settings going to PLL */
	SCU_OUTf(CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure CPU PLL */
	if (use_sys200) {
		/* Power down PLL and PLL output */
		SCU_OUTf(CPU_IPU_PLL_CTRL, PD, 1);
		SCU_OUTf(CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);
		SCU_OUTf(CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);
	} else {
		/* Configure FBDIV first and set POSTDIV1, POSTDIV2
		* Compute dividers based on REF_FREQ_SEL hardware strap
		* Check FBDIV * REFCLK is witin VCO range (950-3800MHz)
		*/
		SCU_OUTf(CPU_IPU_PLL_INTGR_DIV, FBDIV, pllcfg->fbdiv);
		SCU_OUTf(CPU_IPU_PLL_INTGR_DIV, POSTDIV1, pllcfg->postdiv1);
		SCU_OUTf(CPU_IPU_PLL_INTGR_DIV, POSTDIV2, pllcfg->postdiv2);

		/* Set FOUTPOSTDIVPD = 1 to avoid glitches to output */
		SCU_OUTf(CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

		/* Apply the updated PLL configurations  */
		SCU_OUTf(CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

		/* Power up PLL */
		SCU_OUTf(CPU_IPU_PLL_CTRL, PD, 0);

		/* Wait for minimum 128REFCLK, 6.7usec for 19.2MHz refclk
		* before checking PLL lock
		*/
		udelay(7);

		/* Check PLL is locked */
		do {
			lock = SCU_INf(CPU_IPU_PLL_STS, LOCK);
		} while ((++timeout < PLL_LOCK_TIMEOUT) && (lock != 1));

		if ((timeout >= PLL_LOCK_TIMEOUT) && (lock != 1))
			goto fail_cpu_pll_lock;

		/* Set FOUTPOSTDIVPD = 0 to ensure clk output is un-gated */
		SCU_OUTf(CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);
	}

	/* Configure CPU_CLK_DIV */
	SCU_OUTf(CCU_CLK_DIV, CPU_CLK_DIV, clk_div);

	/* If IPU clock is driven by CPU_IPU PLL,
	*  configure IPU divider based on CPU divider value
	*  to make sure IPU clock does not go over its limit
	*/
	if (ipu_clk_src == CPU_IPU_PLL) {
		ipu_div = mnh_clk_get_ipu_div(cpu_freq, clk_div);
		SCU_OUTf(CCU_CLK_DIV, IPU_CLK_DIV, ipu_div);
	}

	/* Set final CPU clock source */
	SCU_OUTf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE, use_sys200);

	/* Lock PLL access */
	SCU_OUTf(PLL_PASSCODE, PASSCODE, 0);

	return 0;

fail_cpu_pll_lock:
	dev_err(mnh_clk->dev, "%s: failed to lock CPU PLL\n", __func__);

	/* Power down the PLL */
	SCU_OUTf(CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);
	SCU_OUTf(CPU_IPU_PLL_CTRL, PD, 1);
	SCU_OUTf(CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);
	SCU_OUTf(CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	/* Lock PLL access */
	SCU_OUTf(PLL_PASSCODE, PASSCODE, 0);

	return -EIO;
}
EXPORT_SYMBOL_GPL(mnh_cpu_freq_change);

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
	int timeout = 0;
	int lock = 0;
	const struct freq_reg_table *pllcfg;
	bool use_cpu_clk_src;
	int ipu_freq, old_ipu_freq, cpu_freq;
	int clk_div;
	int ipu_clken;

	if (!mnh_clk)
		return -ENODEV;

	if (index < IPU_FREQ_MIN || index > IPU_FREQ_MAX)
		return -EINVAL;

	dev_dbg(mnh_clk->dev, "%s: %d\n", __func__, index);

	/* Get PLL config from reg table */
	pllcfg = &ipu_reg_table[index];

	/* Calculate frequency from PLL config */
	old_ipu_freq = mnh_get_ipu_freq();
	cpu_freq = mnh_get_cpu_freq();
	ipu_freq = mnh_freq_get_by_pll(*pllcfg, 0);

	/* Check to see if we need to change the frequency */
	if (old_ipu_freq == ipu_freq) {
		dev_dbg(mnh_clk->dev, "%s: already at desired frequency\n",
			__func__);
		return 0;
	}

	/* TODO: determine if we can get to IPU frequency from CPU frequency */
	if ((cpu_freq == 200) && (ipu_freq == 100)) {
		use_cpu_clk_src = true;
		clk_div = 1;
	} else if ((cpu_freq == 200) && (ipu_freq == 200)) {
		use_cpu_clk_src = true;
		clk_div = 0;
	} else {
		use_cpu_clk_src = false;
		clk_div = pllcfg->clk_div;
	}

	/* Disable IPU_PLL clock output */
	ipu_clken = SCU_INf(CCU_CLK_CTL, IPU_CLKEN);
	SCU_OUTf(CCU_CLK_CTL, IPU_CLKEN, 0x0);

	/* Unlock PLL access */
	SCU_OUTf(PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to stable clock before freq switch to avoid glitches */
	SCU_OUTf(CCU_CLK_CTL, IPU_CLK_SRC, CPU_IPU_PLL);

	/* Set FRZ_PLL_IN=1 to latch the current settings going to PLL */
	SCU_OUTf(IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure IPU PLL */
	if (use_cpu_clk_src) {
		SCU_OUTf(IPU_PLL_CTRL, PD, 1);
		SCU_OUTf(IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);
		SCU_OUTf(IPU_PLL_CTRL, FRZ_PLL_IN, 0);
	} else {
		/* Configure FBDIV first and set POSTDIV1, POSTDIV2 */
		SCU_OUTf(IPU_PLL_INTGR_DIV, FBDIV, pllcfg->fbdiv);
		SCU_OUTf(IPU_PLL_INTGR_DIV, POSTDIV1, pllcfg->postdiv1);
		SCU_OUTf(IPU_PLL_INTGR_DIV, POSTDIV2, pllcfg->postdiv2);

		/* Set FOUTPOSTDIVPD = 1 to avoid glitches to output */
		SCU_OUTf(IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

		/* Apply the updated PLL configurations  */
		SCU_OUTf(IPU_PLL_CTRL, FRZ_PLL_IN, 0);

		/* Power up PLL */
		SCU_OUTf(IPU_PLL_CTRL, PD, 0);

		/* Wait for minimum 128REFCLK, 6.7usec for 19.2MHz refclk
		* before checking PLL lock
		*/
		udelay(7);

		/* Check PLL is locked */
		do {
			lock = SCU_INf(IPU_PLL_STS, LOCK);
		} while ((++timeout < PLL_LOCK_TIMEOUT) && (lock != 1));

		if ((timeout >= PLL_LOCK_TIMEOUT) && (lock != 1))
			goto fail_ipu_pll_lock;

		/* Set FOUTPOSTDIVPD = 0 to ensure clk output is un-gated */
		SCU_OUTf(IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);
	}

	/* Configure IPU_CLK_DIV */
	SCU_OUTf(CCU_CLK_DIV, IPU_CLK_DIV, clk_div);

	/* Go back to IPU PLL output */
	if (!use_cpu_clk_src)
		SCU_OUTf(CCU_CLK_CTL, IPU_CLK_SRC, IPU_PLL);

	/* Lock PLL access */
	SCU_OUTf(PLL_PASSCODE, PASSCODE, 0);

	/* Enable IPU_PLL clock output */
	SCU_OUTf(CCU_CLK_CTL, IPU_CLKEN, ipu_clken);

	return 0;

fail_ipu_pll_lock:
	dev_err(mnh_clk->dev, "%s: failed to lock IPU PLL\n", __func__);

	/* Power down IPU PLL */
	SCU_OUTf(IPU_PLL_CTRL, FRZ_PLL_IN, 1);
	SCU_OUTf(IPU_PLL_CTRL, PD, 1);
	SCU_OUTf(IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);
	SCU_OUTf(IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	/* Lock PLL access */
	SCU_OUTf(PLL_PASSCODE, PASSCODE, 0);

	return -EBUSY;
}
EXPORT_SYMBOL_GPL(mnh_ipu_freq_change);

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
	int timeout = 0;
	enum mnh_lpddr_freq_type ddr_freq;

	if (!mnh_clk)
		return -ENODEV;

	dev_dbg(mnh_clk->dev, "%s: %d\n", __func__, index);

	if (index < LPDDR_FREQ_MIN || index > LPDDR_FREQ_MAX)
		return -EINVAL;

	/* Check the requested FSP is already in use */
	ddr_freq = SCU_INf(LPDDR4_LOW_POWER_STS, LPDDR4_CUR_FSP);
	if (ddr_freq == index) {
		dev_dbg(mnh_clk->dev, "%s: requested fsp%d is in use\n",
			__func__, index);
		return 0;
	}

	/* Power up LPDDR PLL if the FSP setting uses it */
	if (!SCU_INxf(LPDDR4_FSP_SETTING, index, FSP_SYS200_MODE))
		mnh_lpddr_sys200_mode(false);

	/* Disable LPC SW override */
	SCU_OUTf(LPDDR4_LOW_POWER_CFG, LP4_FSP_SW_OVERRIDE, 0);

	/* Configure FSP index */
	SCU_OUTf(LPDDR4_LOW_POWER_CFG, LPC_FREQ_CHG_COPY_NUM, index);

	/* Configure LPC cmd for frequency switch */
	SCU_OUTf(LPDDR4_LOW_POWER_CFG, LPC_EXT_CMD, LP4_LPC_FREQ_SWITCH);

	/* Initiate LPC cmd to LPDDR controller */
	dev_dbg(mnh_clk->dev, "%s: lpddr freq switching from fsp%d to fsp%d\n",
		__func__, ddr_freq, index);
	SCU_OUTf(LPDDR4_LOW_POWER_CFG, LPC_EXT_CMD_REQ, 1);

	/* Wait until LPC cmd process is done */
	do {
		status = SCU_INf(LPDDR4_LOW_POWER_STS, LPC_CMD_DONE);
	} while ((++timeout < PLL_LOCK_TIMEOUT) && (status != 1));

	/* Clear LPC cmd status */
	SCU_OUTf(LPDDR4_LOW_POWER_STS, LPC_CMD_DONE, 1);

	/* Check LPC error status */
	if (SCU_INf(LPDDR4_LOW_POWER_STS, LPC_CMD_RSP) == LPC_CMD_ERR) {
		/* Clear error status */
		SCU_OUTf(LPDDR4_LOW_POWER_STS, LPC_CMD_RSP, 1);
		dev_err(mnh_clk->dev, "Failed to process lpc cmd:0x%x\n",
			LP4_LPC_FREQ_SWITCH);
		return -EIO;
	}

	/* Check FSPx switch status */
	if (SCU_INf(LPDDR4_LOW_POWER_STS, LPDDR4_CUR_FSP) != index) {
		dev_err(mnh_clk->dev, "Failed to switch to fsp%d\n", index);
		return -EIO;
	}

	/* Power down LPDDR PLL if the FSP setting doesn't use it */
	if (SCU_INxf(LPDDR4_FSP_SETTING, index, FSP_SYS200_MODE))
		mnh_lpddr_sys200_mode(true);

	mnh_ddr_clr_int_status(mnh_clk->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_lpddr_freq_change);

/**
 * LPDDR clock control driver
 * Return: 0 on success, an error code otherwise.
 *
 * LPDDR clock is derived from sys200 clk instead of separate lpddr clk
 */
int mnh_lpddr_sys200_mode(bool enable)
{
	int lock;
	int timeout = 0;

	/* Switch lpddr to SYS200 mode */
	SCU_OUTf(CCU_CLK_CTL, LP4_AXI_SYS200_MODE, 0x1);

	dev_dbg(mnh_clk->dev, "%s: %d\n", __func__, enable);
	/* Unlock PLL access */
	SCU_OUTf(PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	if (enable) {
		/* Power down LPDDR PLL */
		SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, PD, 1);
		SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FOUTPOSTDIVPD, 1);
		SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, BYPASS, 1);
	} else {
		/* Power up LPDDR PLL */
		SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 1);
		SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, PD, 0);
		SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FOUTPOSTDIVPD, 0);
		SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, BYPASS, 0);
		SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 0);

		/* Check PLL is locked */
		do {
			lock = SCU_INf(LPDDR4_REFCLK_PLL_STS, LOCK);
		} while ((++timeout < PLL_LOCK_TIMEOUT) && (lock != 1));
	}

	/* Lock PLL */
	SCU_OUTf(PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_lpddr_sys200_mode);

/**
 * CPU and IPU SYS200 clock control driver
 * Return: 0 on success, an error code otherwise.
 *
 * CPU and IPU clock is derived from sys200 clk instead of separate plls
 */
int mnh_cpu_ipu_sys200_mode(void)
{
	int ipu_clk_src;

	if (!mnh_clk)
		return -ENODEV;

	dev_dbg(mnh_clk->dev, "%s\n", __func__);

	/* Unlock PLL access */
	SCU_OUTf(PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to SYS200 mode */
	SCU_OUTf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0x1);

	/* Read current IPU clock source */
	ipu_clk_src =
		SCU_INf(CCU_CLK_CTL, IPU_CLK_SRC);

	if (ipu_clk_src == IPU_PLL) {
		/* Change clk source to CPU_IPU_PLL */
		SCU_OUTf(CCU_CLK_CTL, IPU_CLK_SRC, CPU_IPU_PLL);

		/* IPU: Latch current settings to go into PLL */
		SCU_OUTf(IPU_PLL_CTRL, FRZ_PLL_IN, 1);

		/* IPU: Power down PLL */
		SCU_OUTf(IPU_PLL_CTRL, PD, 1);
		SCU_OUTf(IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

		/* IPU: Apply PLL configurations */
		SCU_OUTf(IPU_PLL_CTRL, FRZ_PLL_IN, 0);
	}

	/* Configure IPU_CLK_DIV */
	SCU_OUTf(CCU_CLK_DIV, IPU_CLK_DIV, sys200_reg_table[IPU_CLK].clk_div);

	/* Configure CPU_CLK_DIV */
	SCU_OUTf(CCU_CLK_DIV, CPU_CLK_DIV, sys200_reg_table[CPU_CLK].clk_div);

	/* CPU_IPU: Latch current settings to go into PLL */
	SCU_OUTf(CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* CPU_IPU: Power down PLL */
	SCU_OUTf(CPU_IPU_PLL_CTRL, PD, 1);
	SCU_OUTf(CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* CPU_IPU: Apply PLL configurations */
	SCU_OUTf(CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	/* Lock PLL access */
	SCU_OUTf(PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_cpu_ipu_sys200_mode);

/**
 * Set the SCU clock gating at init
 * Return: 0 on success, an error code otherwise.
 *
 * enable == 1 sets the SCU to enable clock gating in general when CPU enters
 * L2 WFI state.
 */
int mnh_clock_init_gating(bool enabled)
{
	if (!mnh_clk)
		return -ENODEV;

	dev_dbg(mnh_clk->dev, "%s:%d\n", __func__, __LINE__);

	/* Add periph clk gates */
	SCU_OUTf(RSTC, PERI_DMA_RST, enabled);

	SCU_OUTf(PERIPH_CLK_CTRL, PERI_DMA_CLKEN_SW, !enabled);
	SCU_OUTf(MEM_PWR_MGMNT, HALT_BTROM_PD_EN, enabled);
	SCU_OUTf(MEM_PWR_MGMNT, HALT_BTSRAM_PD_EN, enabled);
	SCU_OUTf(MEM_PWR_MGMNT, BTROM_SLP, enabled);
	SCU_OUTf(MEM_PWR_MGMNT, BTSRAM_DS, enabled);
	SCU_OUTf(CCU_CLK_CTL, HALT_BTSRAMCG_EN, enabled);
	SCU_OUTf(CCU_CLK_CTL, HALT_BTROMCG_EN, enabled);

	SCU_OUTf(CCU_CLK_CTL, HALT_LP4_PLL_BYPCLK_CG_EN, enabled);
	SCU_OUTf(CCU_CLK_CTL, LP4PHY_PLL_BYPASS_CLKEN, enabled);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_clock_init_gating);

int mnh_bypass_clock_gating(bool enabled)
{
	if (!mnh_clk)
		return -ENODEV;

	dev_dbg(mnh_clk->dev, "%s:%d\n", __func__, __LINE__);

	if (enabled) {
		SCU_OUTf(CCU_CLK_CTL, HALT_CPUCG_EN, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, HALT_CPUMEM_PD_EN, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, CPU_L2MEM_DS, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, CPU_L1MEM_DS, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, HALT_LP4CMEM_PD_EN, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, LP4C_MEM_DS, enabled);
	} else {
		SCU_OUTf(MEM_PWR_MGMNT, CPU_L2MEM_DS, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, CPU_L1MEM_DS, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, HALT_CPUMEM_PD_EN, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, LP4C_MEM_DS, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, HALT_LP4CMEM_PD_EN, enabled);
		SCU_OUTf(CCU_CLK_CTL, HALT_CPUCG_EN, enabled);
	}

	SCU_OUTf(RSTC, WDT_RST, enabled);

	SCU_OUTf(PERIPH_CLK_CTRL, PVT_CLKEN, !enabled);
	SCU_OUTf(PERIPH_CLK_CTRL, WDT_CLKEN_SW, !enabled);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_bypass_clock_gating);

/**
 * Set the SCU clock gating for bypass mode
 * Return: 0 on success, an error code otherwise.
 *
 * enable == 1 sets the SCU to enable clock gating in general when CPU enters
 * L2 WFI state.
 */
int mnh_ipu_clock_gating(bool enabled)
{
	if (!mnh_clk)
		return -ENOENT;

	dev_dbg(mnh_clk->dev, "%s\n", __func__);

	if (enabled) {
		SCU_OUTf(CCU_CLK_CTL, IPU_CLKEN, !enabled);
		SCU_OUTf(MEM_PWR_MGMNT, IPU_MEM_DS, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, IPU_MEM_SD, enabled);
	} else {
		SCU_OUTf(MEM_PWR_MGMNT, IPU_MEM_SD, enabled);
		SCU_OUTf(MEM_PWR_MGMNT, IPU_MEM_DS, enabled);
		SCU_OUTf(CCU_CLK_CTL, IPU_CLKEN, !enabled);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_ipu_clock_gating);

int mnh_ipu_reset(void)
{
	dev_dbg(mnh_clk->dev, "%s\n", __func__);
	if (!mnh_clk)
		return -ENOENT;

	SCU_OUTf(RSTC, IPU_RST, 1);
	SCU_OUTf(RSTC, IPU_RST, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_ipu_reset);

static ssize_t cpu_freq_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%dMHz\n", mnh_get_cpu_freq());
}

static ssize_t cpu_freq_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	int i, err;

	dev_dbg(mnh_clk->dev, "%s: %s\n", __func__, buf);
	for (i = 0; i < ARRAY_SIZE(cpu_reg_table); i++) {
		if (!strncmp(buf, cpu_reg_table[i].freq_str,
			strlen(cpu_reg_table[i].freq_str))) {
			err = mnh_cpu_freq_change(i);
			if (!err)
				return count;
			else
				return -EIO;
		}
	}

	dev_err(mnh_clk->dev, "invalid freq: %s\n", buf);
	return -EINVAL;

}
static DEVICE_ATTR_RW(cpu_freq);

static ssize_t ipu_freq_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%dMHz\n", mnh_get_ipu_freq());
}

static ssize_t ipu_freq_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	int i, err;

	dev_dbg(mnh_clk->dev, "%s: %s\n", __func__, buf);
	for (i = 0; i < ARRAY_SIZE(ipu_reg_table); i++) {
		if (!strncmp(buf, ipu_reg_table[i].freq_str,
			strlen(ipu_reg_table[i].freq_str))) {
			err = mnh_ipu_freq_change(i);
			if (!err)
				return count;
			else
				return -EIO;
		}
	}

	dev_err(mnh_clk->dev, "invalid freq: %s\n", buf);
	return -EINVAL;
}
static DEVICE_ATTR_RW(ipu_freq);

static ssize_t lpddr_freq_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	uint32_t var = SCU_INf(LPDDR4_LOW_POWER_STS,
				LPDDR4_CUR_FSP);

	dev_dbg(mnh_clk->dev, "%s: %d\n", __func__, var);
	return snprintf(buf, PAGE_SIZE, "FSP%d\n", var);
}

static ssize_t lpddr_freq_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int var;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_clk->dev, "%s: %d\n", __func__, var);
	if (!mnh_lpddr_freq_change(var))
		return count;
	else
		return -EIO;
}
static DEVICE_ATTR_RW(lpddr_freq);

static ssize_t ipu_clk_src_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int ipu_clk_src = SCU_INf(CCU_CLK_CTL, IPU_CLK_SRC);

	return snprintf(buf, PAGE_SIZE, "%s\n",
		(ipu_clk_src == CPU_IPU_PLL) ? "CPU_IPU":"IPU");
}
static DEVICE_ATTR_RO(ipu_clk_src);

static ssize_t sys200_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int sys200, clk_src;

	clk_src = SCU_INf(CCU_CLK_CTL, IPU_CLK_SRC);
	sys200 = SCU_INf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200 && (clk_src == IPU_PLL))
		sys200 = 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", sys200);
}

static ssize_t sys200_mode_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	bool var;
	int ret;

	ret = kstrtobool(buf, &var);
	if (ret < 0)
		return ret;

	if (var) {
		dev_dbg(mnh_clk->dev, "%s: %d\n", __func__, var);
		if (!mnh_cpu_ipu_sys200_mode())
			return count;
	}

	return -EIO;
}
static DEVICE_ATTR_RW(sys200_mode);

static ssize_t ipu_clock_gating_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int clk_gated;

	clk_gated = !SCU_INf(CCU_CLK_CTL, IPU_CLKEN);

	return snprintf(buf, PAGE_SIZE, "%d\n", clk_gated);
}

static ssize_t ipu_clock_gating_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	bool var;
	int ret;

	ret = kstrtobool(buf, &var);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_clk->dev, "%s: %d\n", __func__, var);
	if (!mnh_ipu_clock_gating(var))
		return count;

	return -EIO;
}

static DEVICE_ATTR_RW(ipu_clock_gating);

static ssize_t bypass_clock_gating_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int clk_gated;

	clk_gated = SCU_INf(CCU_CLK_CTL, HALT_CPUCG_EN);

	return snprintf(buf, PAGE_SIZE, "%d\n", clk_gated);
}

static ssize_t bypass_clock_gating_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	bool var;
	int ret;

	ret = kstrtobool(buf, &var);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_clk->dev, "%s: %d\n", __func__, var);
	if (!mnh_bypass_clock_gating(var))
		return count;

	return -EIO;
}
static DEVICE_ATTR_RW(bypass_clock_gating);

static struct attribute *mnh_clk_attrs[] = {
	&dev_attr_cpu_freq.attr,
	&dev_attr_ipu_freq.attr,
	&dev_attr_lpddr_freq.attr,
	&dev_attr_ipu_clk_src.attr,
	&dev_attr_sys200_mode.attr,
	&dev_attr_ipu_clock_gating.attr,
	&dev_attr_bypass_clock_gating.attr,
	NULL
};

static struct attribute_group mnh_clk_attr_group = {
	.name = "mnh_clk",
	.attrs = mnh_clk_attrs
};

static int init_sysfs(struct device *dev, struct kobject *sysfs_kobj)
{
	int ret;

	ret = sysfs_create_group(sysfs_kobj, &mnh_clk_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group (%d)\n", ret);
		return -EINVAL;
	}

	return 0;
}

static void clean_sysfs(struct kobject *sysfs_kobj)
{
	sysfs_remove_group(sysfs_kobj, &mnh_clk_attr_group);
}

int mnh_clk_init(struct device *dev, uint32_t baseadress)
{
	dev_dbg(dev, "%s\n", __func__);

	mnh_clk = devm_kzalloc(dev, sizeof(*mnh_clk), GFP_KERNEL);
	if (!mnh_clk)
		return -ENOMEM;

	mnh_clk->dev = dev;

	init_sysfs(dev, &dev->kobj);

	return 0;
}

void mnh_clk_clean(struct device *dev)
{
	clean_sysfs(&dev->kobj);
}
