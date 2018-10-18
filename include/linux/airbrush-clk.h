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

void abc_clk_aon_init(struct device_node *np);
void abc_clk_core_init(struct device_node *np);
void abc_clk_fsys_init(struct device_node *np);
void abc_clk_mif_init(struct device_node *np);
void abc_clk_ipu_init(struct device_node *np);
void abc_clk_tpu_init(struct device_node *np);

/* TODO: Move away from using state manager context
 * pointer. This is a temporary fix to kernel crashing.
 */
void ipu_gate(struct ab_state_context *ab_ctx);
void ipu_ungate(struct ab_state_context *ab_ctx);
int ipu_pll_enable(struct ab_state_context *ab_ctx);
void ipu_pll_disable(struct ab_state_context *ab_ctx);
u64 ipu_set_rate(struct ab_state_context *ab_ctx, u64 rate);

void tpu_gate(struct ab_state_context *ab_ctx);
void tpu_ungate(struct ab_state_context *ab_ctx);
int tpu_pll_enable(struct ab_state_context *ab_ctx);
void tpu_pll_disable(struct ab_state_context *ab_ctx);
u64 tpu_set_rate(struct ab_state_context *ab_ctx, u64 rate);

#endif //_AIRBRUSH_CLK_H_
