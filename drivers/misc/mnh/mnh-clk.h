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

#ifndef __MNH_CLK
#define __MNH_CLK

enum mnh_cpu_freq_type {
	CPU_FREQ_MIN = 0,
	CPU_FREQ_200 = CPU_FREQ_MIN,
	CPU_FREQ_400,
	CPU_FREQ_600,
	CPU_FREQ_800,
	CPU_FREQ_950,
	CPU_FREQ_MAX = CPU_FREQ_950
};

/**
 * API to set the CPU clock rate.
 * CPU clock 200-1000MHZ (IPU clock 100-500MHz)
 * 1PLL(CPU_IPU PLL) for CPU/IPU clocks. Since CPU and IPU clocks are derived
 * from same PLL in this mode, there would be restriction on achedivable clock
 * frequencies for CPU and IPU clocks. IPU clock would be half of CPU clock. Any
 * frequency changes is achieved by changing FBDIV(integer feedback division) of
 * the PLL(PLL output = FBDIV * REFCLK frequency).
 * Default CPU_CLK_DIV : 1, IPU_CLK_DIV: 2
 * CLK = (REFCLK FREQ * FBDIV) / [ (POSTDIV1 * POSTDIV2) * [?_CLK_DIV[3:0] + 1])
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_cpu_set_clk_freq(struct device *dev, int index);

#endif /* __MNH_CLK */
