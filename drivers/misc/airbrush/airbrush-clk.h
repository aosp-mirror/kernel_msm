/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 * Author: Raman Kumar Banka <raman.k2@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _AIRBRUSH_CLK_H_
#define _AIRBRUSH_CLK_H_

#include <linux/airbrush-sm-ctrl.h>
#include <linux/clk-provider.h>
#include <linux/of.h>

#define MIF_PLL_TIMEOUT		1000

#define MIF_PLL_CONTROL0	0x10510140

struct ab_clk_context {
	struct device *dev;

	struct clk *ipu_pll;
	struct clk *ipu_pll_mux;
	struct clk *ipu_pll_div;
	struct clk *ipu_switch_mux;
	struct clk *tpu_pll;
	struct clk *tpu_pll_mux;
	struct clk *tpu_pll_div;
	struct clk *tpu_switch_mux;
	struct clk *osc_clk;
	struct clk *shared_div_aon_pll;
	struct clk *aon_pll;
	struct clk *aon_pll_mux;
};

void abc_clk_aon_init(struct device_node *np);
void abc_clk_core_init(struct device_node *np);
void abc_clk_fsys_init(struct device_node *np);
void abc_clk_mif_init(struct device_node *np);
void abc_clk_ipu_init(struct device_node *np);
void abc_clk_tpu_init(struct device_node *np);

#endif //_AIRBRUSH_CLK_H_
