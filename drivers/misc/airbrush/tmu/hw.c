// SPDX-License-Identifier: GPL-2.0
/*
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
#include "hw.h"

#include <linux/device.h>
#include <linux/mfd/abc-pcie.h>

struct ab_tmu_hw {
	struct device *dev;
	u32 base;
};

static int ab_tmu_hw_init(struct ab_tmu_hw *hw, struct device *dev, u32 base)
{
	hw->dev = dev;
	hw->base = base;
	return 0;
}

static void ab_tmu_hw_exit(struct ab_tmu_hw *hw)
{}

static void devm_ab_tmu_hw_release(struct device *dev, void *res)
{
	struct ab_tmu_hw *hw = res;

	ab_tmu_hw_exit(hw);
}

struct ab_tmu_hw *devm_ab_tmu_hw_create(struct device *dev, u32 base)
{
	struct ab_tmu_hw *hw;
	int err;

	hw = devres_alloc(devm_ab_tmu_hw_release, sizeof(struct ab_tmu_hw),
			GFP_KERNEL);
	if (!hw)
		return ERR_PTR(-ENOMEM);

	err = ab_tmu_hw_init(hw, dev, base);
	if (err < 0) {
		devres_free(hw);
		return ERR_PTR(err);
	}

	devres_add(dev, hw);
	return hw;
}

static int
devm_ab_tmu_hw_match(struct device *dev, void *res, void *data)
{
	return res == data;
}

void devm_ab_tmu_hw_destroy(struct ab_tmu_hw *hw)
{
	devres_release(hw->dev, devm_ab_tmu_hw_release, devm_ab_tmu_hw_match,
		hw);
}
