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

#ifndef __MNH_PLL_H__
#define __MNH_PLL_H__

#include <linux/types.h>

#define MNH_NUM_FSPS 4

enum mnh_lpddr4_freq {
	/* used as an index */
	MNH_LP4_FREQ_BOOT	=   0,
	MNH_LP4_FREQ_400,
	MNH_LP4_FREQ_800,
	MNH_LP4_FREQ_1200,
	MNH_LP4_FREQ_1600,
	MNH_LP4_FREQ_1920,
	MNH_LP4_FREQ_2400,
};

struct mnh_fsp_setting {
	u8 sw_ctrl;
	u8 sys200;
	u16 fbdiv;
	u8 pcie_axi_clk_div;
	u8 axi_fab_clk_div;
	u8 lpddr4_refclk_div;

};

int mnh_pll_lpddr4_boot_settings(int freq0, int freq1, int freq2, int freq3);
int mnh_pll_lpddr4_is_locked(void);
void mnh_pll_set_lpddr4_ref_clock_en(u32 val);
#endif /* __MNH_PLL_H__ */
