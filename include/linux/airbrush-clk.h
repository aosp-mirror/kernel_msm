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

void abc_clk_aon_init(struct device_node *np);
void abc_clk_core_init(struct device_node *np);
void abc_clk_fsys_init(struct device_node *np);
void abc_clk_mif_init(struct device_node *np);
void abc_clk_ipu_init(struct device_node *np);
void abc_clk_tpu_init(struct device_node *np);

/* TODO: Move away from using state manager context
 * pointer. This is a temporary fix to kernel crashing.
 */
int ipu_gate(void *ctx);
int ipu_ungate(void *ctx);
int ipu_pll_enable(struct ab_state_context *ab_ctx);
void ipu_pll_disable(struct ab_state_context *ab_ctx);
u64 ipu_set_rate(struct ab_state_context *ab_ctx, u64 rate);

int tpu_gate(void *ctx);
int tpu_ungate(void *ctx);
int tpu_pll_enable(struct ab_state_context *ab_ctx);
void tpu_pll_disable(struct ab_state_context *ab_ctx);
u64 tpu_set_rate(struct ab_state_context *ab_ctx, u64 rate);

u64 aon_set_rate(struct ab_state_context *sc, u64 rate);

int attach_mif_clk_ref(void *ctx);
int deattach_mif_clk_ref(void *ctx);

#endif //_AIRBRUSH_CLK_H_
