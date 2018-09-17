/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *
 * Author: Raman Kumar Banka <raman.k2@samsung.com>
 *
 * Clock controller for airbrush state manager
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include<linux/mfd/abc-pcie.h>
#include <linux/airbrush-sm-ctrl.h>

#define OSC_RATE 19200000

void ipu_pll_enable(struct device *dev)
{
	struct clk *ipu_pll;

	ipu_pll = clk_get(dev, "ipu_pll");
	clk_prepare(ipu_pll);
	clk_enable(ipu_pll);
}

void ipu_pll_disable(struct device *dev)
{
	struct clk *ipu_pll;

	ipu_pll = clk_get(dev, "ipu_pll");
	clk_disable(ipu_pll);
	clk_unprepare(ipu_pll);
}

u64 ipu_set_rate(struct device *dev, u64 rate)
{
	struct clk *ipu_pll_mux;
	struct clk *ipu_pll;
	struct clk *ipu_pll_div;
	struct clk *ipu_switch_mux;
	struct clk *osc_clk;
	struct clk *shared_div_aon_pll;

	if (rate == 0)
		rate = OSC_RATE;

	ipu_pll_mux = clk_get(dev, "ipu_pll_mux");
	ipu_pll = clk_get(dev, "ipu_pll");
	ipu_pll_div = clk_get(dev, "ipu_pll_div");
	ipu_switch_mux = clk_get(dev, "ipu_switch_mux");
	osc_clk = clk_get(dev, "osc_clk");
	shared_div_aon_pll = clk_get(dev, "shared_div_aon_pll");

	if (rate == OSC_RATE) {
		clk_set_parent(ipu_pll_mux, osc_clk);
		clk_set_rate(ipu_pll_div, OSC_RATE);
		clk_set_parent(ipu_switch_mux, ipu_pll_div);
		return clk_get_rate(ipu_switch_mux);
	}

	clk_set_parent(ipu_pll_mux, ipu_pll);
	clk_set_parent(ipu_switch_mux, shared_div_aon_pll);
	clk_set_rate(ipu_pll, rate);
	clk_set_rate(ipu_pll_div, rate);
	clk_set_parent(ipu_switch_mux, ipu_pll_div);

	return clk_get_rate(ipu_switch_mux);
}

void tpu_pll_enable(struct device *dev)
{
	struct clk *tpu_pll;

	tpu_pll = clk_get(dev, "tpu_pll");
	clk_prepare(tpu_pll);
	clk_enable(tpu_pll);
}

void tpu_pll_disable(struct device *dev)
{
	struct clk *tpu_pll;

	tpu_pll = clk_get(dev, "tpu_pll");
	clk_disable(tpu_pll);
	clk_unprepare(tpu_pll);
}

u64 tpu_set_rate(struct device *dev, u64 rate)
{
	struct clk *tpu_pll_mux;
	struct clk *tpu_pll;
	struct clk *tpu_pll_div;
	struct clk *tpu_switch_mux;
	struct clk *osc_clk;
	struct clk *shared_div_aon_pll;

	if (rate == 0)
		rate = OSC_RATE;

	tpu_pll_mux = clk_get(dev, "tpu_pll_mux");
	tpu_pll = clk_get(dev, "tpu_pll");
	tpu_pll_div = clk_get(dev, "tpu_pll_div");
	tpu_switch_mux = clk_get(dev, "tpu_switch_mux");
	osc_clk = clk_get(dev, "osc_clk");
	shared_div_aon_pll = clk_get(dev, "shared_div_aon_pll");

	if (rate == OSC_RATE) {
		clk_set_parent(tpu_pll_mux, osc_clk);
		clk_set_rate(tpu_pll_div, OSC_RATE);
		clk_set_parent(tpu_switch_mux, tpu_pll_div);
		return clk_get_rate(tpu_switch_mux);
	}

	clk_set_parent(tpu_pll_mux, tpu_pll);
	clk_set_parent(tpu_switch_mux, shared_div_aon_pll);
	clk_set_rate(tpu_pll, rate);
	clk_set_rate(tpu_pll_div, rate);
	clk_set_parent(tpu_switch_mux, tpu_pll_div);

	return clk_get_rate(tpu_switch_mux);
}

void abc_clk_register(struct ab_state_context *ab_ctx)
{
	struct platform_device *pdev = ab_ctx->pdev;
	struct device_node *child = NULL;

	/* Registering CMUs to Common Clock Framework.
	 * Parse through the airbrush device node and scan for cmu nodes.
	 * once found, register the same with the common clock framework
	 */
	for_each_child_of_node(pdev->dev.of_node, child) {
		if (of_device_is_compatible(child,
					"diablo,abc-clock-aon")) {
			abc_clk_aon_init(child);
		} else if (of_device_is_compatible(child,
					"diablo,abc-clock-core")) {
			abc_clk_core_init(child);
		} else if (of_device_is_compatible(child,
					"diablo,abc-clock-fsys")) {
			abc_clk_fsys_init(child);
		} else if (of_device_is_compatible(child,
					"diablo,abc-clock-mif")) {
			abc_clk_mif_init(child);
		} else if (of_device_is_compatible(child,
					"diablo,abc-clock-ipu")) {
			abc_clk_ipu_init(child);
		} else if (of_device_is_compatible(child,
					"diablo,abc-clock-tpu")) {
			abc_clk_tpu_init(child);
		}
	}
}
