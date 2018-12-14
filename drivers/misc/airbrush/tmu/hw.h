/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _AIRBRUSH_TMU_HW_
#define _AIRBRUSH_TMU_HW_

#include <linux/types.h>

struct ab_tmu_hw;
struct device;

struct ab_tmu_hw *devm_ab_tmu_hw_create(struct device *dev, u32 base);
void devm_ab_tmu_hw_destroy(struct ab_tmu_hw *hw);

#endif /* _AIRBRUSH_TMU_HW_ */
