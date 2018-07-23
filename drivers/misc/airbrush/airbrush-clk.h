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

#include <linux/clk-provider.h>
#include <linux/of.h>

void abc_clk_aon_init(struct device_node *np);
void abc_clk_core_init(struct device_node *np);
void abc_clk_fsys_init(struct device_node *np);
void abc_clk_mif_init(struct device_node *np);
void abc_clk_ipu_init(struct device_node *np);
void abc_clk_tpu_init(struct device_node *np);

unsigned long ipu_set_rate(struct device *dev, unsigned long rate);
unsigned long tpu_set_rate(struct device *dev, unsigned long rate);
#endif
