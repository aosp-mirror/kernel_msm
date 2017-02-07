/*
 *
 * MNH Clock Driver
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


#include <linux/delay.h>
#include "mnh-hwio.h"
#include "mnh-hwio-bases.h"
#include "mnh-hwio-scu.h"

#define MNH_SCU_INf(reg, fld) \
HW_INf(HWIO_SCU_BASE_ADDR, SCU, reg, fld)
#define MNH_SCU_OUTf(reg, fld, val) \
HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, reg, fld, val)
#define MNH_SCU_OUT(reg, val) \
HW_OUT(HWIO_SCU_BASE_ADDR, SCU, reg, val)

#define PLL_UNLOCK 0x4CD9
#define REF_FREQ_SEL 0x8
#define MAX_STR_COPY 9
#define LP4_LPC_FREQ_SWITCH 0x8A
#define PLL_LOCK_TIMEOUT 100

enum mnh_cpu_freq_type {
	CPU_FREQ_MIN = 0,
	CPU_FREQ_200 = CPU_FREQ_MIN,
	CPU_FREQ_400,
	CPU_FREQ_600,
	CPU_FREQ_800,
	CPU_FREQ_950,
	CPU_FREQ_MAX = CPU_FREQ_950
};

enum mnh_ipu_clk_src {
	CPU_IPU_PLL = 0,/*Both CPU and IPU Clock is derived from same PLL */
	IPU_PLL	/* IPU Clock is derived from IPU PLL */
};

struct freq_reg_table {
	int fidiv;
	int postdiv1;
	int postdiv2;
	int clk_div;
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
	{0, 0, 0, 0},	/* 200 MHz in SYS200 */
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

/* CPU clock 200-1000MHZ (IPU clock 100-500MHz)
 * 1PLL(CPU_IPU PLL) for CPU/IPU clocks. Since CPU and IPU clocks are derived
 * from same PLL in this mode, there would be restriction on achedivable clock
 * frequencies for CPU and IPU clocks. IPU clock would be half of CPU clock. Any
 * frequency changes is achieved by changing FBDIV(integer feedback division) of
 * the PLL(PLL output = FBDIV * REFCLK frequency).
 * Default CPU_CLK_DIV : 1, IPU_CLK_DIV: 2
 * CLK = (REFCLK FREQ * FBDIV) / [ (POSTDIV1 * POSTDIV2) * [?_CLK_DIV[3:0] + 1])
 */
int mnh_sm_cpu_set_clk_freq(struct device *dev, int index)
{
	int lock = 0;
	int ipu_div = 0;
	int ipu_clk_src = 0;
	int timeout = 0;
	int ret = 0;

	if (index < CPU_FREQ_MIN || index > CPU_FREQ_MAX)
		return -EINVAL;

	dev_dbg(dev, "%s: %d\n", __func__, index);

	/* Unlock PLL access */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to SYS200 mode */
	MNH_SCU_OUTf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0x1);

	/* Read current IPU clock source */
	ipu_clk_src =
		MNH_SCU_INf(CCU_CLK_CTL, IPU_CLK_SRC);

	if (index == CPU_FREQ_200) {
		/* Power down CPU_IPU PLL */
		MNH_SCU_OUTf(CPU_IPU_PLL_CTRL, PD, 1);
		MNH_SCU_OUTf(CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

		goto pll_acc_lock;
	}

	/* Latch current settings going to PLL */
	MNH_SCU_OUTf(CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure FBDIV first and set POSTDIV1, POSTDIV2
	* Compute dividers based on REF_FREQ_SEL hardware strap
	* Check FBDIV * REFCLK is witin VCO range (950-3800MHz)
	*/
	MNH_SCU_OUTf(CPU_IPU_PLL_INTGR_DIV, FBDIV,
		cpu_reg_tables[index].fidiv);
	MNH_SCU_OUTf(CPU_IPU_PLL_INTGR_DIV, POSTDIV1,
			cpu_reg_tables[index].postdiv1);
	MNH_SCU_OUTf(CPU_IPU_PLL_INTGR_DIV, POSTDIV2,
		cpu_reg_tables[index].postdiv2);

	/* Set FOUTPOSTDIVPD = 1 to avoid glitches to output */
	MNH_SCU_OUTf(CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* Apply the updated PLL configurations  */
	MNH_SCU_OUTf(CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	/* Power up PLL */
	MNH_SCU_OUTf(CPU_IPU_PLL_CTRL, PD, 0);

	/* Wait for minimum 128REFCLK, 6.7usec for 19.2MHz refclk
	* before checking PLL lock
	*/
	udelay(7);

	/* Check PLL is locked */
	do {
		lock = MNH_SCU_INf(CPU_IPU_PLL_STS, LOCK);
		timeout++;
	} while ((timeout < PLL_LOCK_TIMEOUT) && (lock != 1));

	if ((timeout == PLL_LOCK_TIMEOUT) && (lock != 1)) {
		dev_err(dev, "%s: failed to lock PLL\n", __func__);

		ret = -EBUSY;

		/* Power down CPU_IPU PLL */
		MNH_SCU_OUTf(CPU_IPU_PLL_CTRL, PD, 1);
		MNH_SCU_OUTf(CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

		goto pll_acc_lock;
	}

	/* Set FOUTPOSTDIVPD = 0 to ensure clk output is un-gated */
	MNH_SCU_OUTf(CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);

	/* Configure CPU_CLK_DIV */
	MNH_SCU_OUTf(CCU_CLK_DIV, CPU_CLK_DIV,
		cpu_reg_tables[index].clk_div);

	/* If IPU clock is driven by CPU_IPU PLL,
	*  configure IPU divider based on CPU divider value
	*  to make sure IPU clock does not go over its limit
	*/
	if (ipu_clk_src == CPU_IPU_PLL) {
		if (index > CPU_FREQ_800)
			ipu_div = mnh_clk_get_ipu_div(950,
				cpu_reg_tables[index].clk_div);
		else
			ipu_div = mnh_clk_get_ipu_div(0,
				cpu_reg_tables[index].clk_div);

		MNH_SCU_OUTf(CCU_CLK_DIV, IPU_CLK_DIV, ipu_div);
	}

	/* Go back to CPU_IPU PLL output */
	MNH_SCU_OUTf(CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0);

pll_acc_lock:

	/* Lock PLL access */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0);

	return ret;
}
EXPORT_SYMBOL(mnh_sm_cpu_set_clk_freq);


