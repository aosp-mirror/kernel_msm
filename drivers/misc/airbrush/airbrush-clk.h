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

struct ab_clk_context {
	struct device *dev;

	struct clk *ipu_pll;
	struct clk *ipu_pll_mux;
	struct clk *ipu_pll_div;
	struct clk *ipu_switch_mux;
	struct clk *ipu_gate_clk;
	struct clk *tpu_pll;
	struct clk *tpu_pll_mux;
	struct clk *tpu_pll_div;
	struct clk *tpu_switch_mux;
	struct clk *tpu_gate_clk;
	struct clk *osc_clk;
	struct clk *shared_div_aon_pll;
	struct clk *aon_pll;
	struct clk *aon_pll_mux;

	struct mutex pcie_link_lock;
	bool pcie_link_ready; /* Guarded by pcie_link_lock */
	struct notifier_block pcie_link_blocking_nb;
};

void abc_clk_aon_init(struct device_node *np);
void abc_clk_core_init(struct device_node *np);
void abc_clk_fsys_init(struct device_node *np);
void abc_clk_mif_init(struct device_node *np);
void abc_clk_ipu_init(struct device_node *np);
void abc_clk_tpu_init(struct device_node *np);

#endif //_AIRBRUSH_CLK_H_
