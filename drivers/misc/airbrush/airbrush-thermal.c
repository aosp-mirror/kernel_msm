// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "airbrush-thermal.h"
#include "airbrush-cooling.h"

#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/thermal.h>

static enum throttle_state to_throttle_state(unsigned long state)
{
	switch (state) {
	case 0:
		return THROTTLE_NONE;
	case 1:
		return THROTTLE_TO_MID;
	case 2:
		return THROTTLE_TO_LOW;
	default:
		return THROTTLE_TO_MIN;
	}
}

struct ab_thermal {
	struct device *dev;

	struct ab_thermal_ops ops;
	void *op_data;

	struct ab_cooling *cooling;
};

static void ab_thermal_exit(struct ab_thermal *thermal)
{
	if (!IS_ERR_OR_NULL(thermal->cooling))
		ab_cooling_unregister(thermal->cooling);
}

static void ab_thermal_cooling_op_state_updated(
		const struct ab_cooling *cooling, unsigned long old_state,
		unsigned long new_state, void *cooling_op_data)
{
	struct ab_thermal *thermal = cooling_op_data;
	enum throttle_state state;

	state = to_throttle_state(new_state);
	thermal->ops.throttle_state_updated(state, thermal->op_data);
}

const static struct ab_cooling_ops ab_thermal_cooling_ops = {
	.state_updated = ab_thermal_cooling_op_state_updated,
};

static int ab_thermal_init(struct ab_thermal *thermal, struct device *dev,
		const struct ab_thermal_ops *ops, void *op_data)
{
	int err;
	struct device_node *cooling_node;

	thermal->dev = dev;
	thermal->ops = *ops;
	thermal->op_data = op_data;
	thermal->cooling = NULL;

	cooling_node = of_find_node_by_name(NULL, "abc-cooling");
	if (!cooling_node)
		dev_warn(dev, "Cannot find OF node for self cooling device. Cooling cannot trigger by airbrush itself.");

	thermal->cooling = ab_cooling_register(cooling_node,
			AB_CDEV_NAME, &ab_thermal_cooling_ops, thermal);
	if (IS_ERR(thermal->cooling)) {
		err = PTR_ERR(thermal->cooling);
		ab_thermal_exit(thermal);
		return err;
	}

	return 0;
}

static void devm_ab_thermal_release(struct device *dev, void *res)
{
	struct ab_thermal *thermal = res;

	ab_thermal_exit(thermal);
}

struct ab_thermal *devm_ab_thermal_create(struct device *dev,
		const struct ab_thermal_ops *ops, void *op_data)
{
	struct ab_thermal *thermal;
	int err;

	thermal = devres_alloc(devm_ab_thermal_release,
			sizeof(struct ab_thermal), GFP_KERNEL);
	if (!thermal)
		return ERR_PTR(-ENOMEM);

	err = ab_thermal_init(thermal, dev, ops, op_data);
	if (err < 0) {
		devres_free(thermal);
		return ERR_PTR(err);
	}

	devres_add(dev, thermal);
	return thermal;
}

static int devm_ab_thermal_match(struct device *dev, void *res, void *data)
{
	struct ab_thermal *r = res;
	struct ab_thermal *d = data;

	return r == d;
}

void devm_ab_thermal_destroy(struct ab_thermal *thermal)
{
	devres_release(thermal->dev, devm_ab_thermal_release,
			devm_ab_thermal_match, thermal);
}
