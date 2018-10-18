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
#include "airbrush-cooling.h"

#include <linux/slab.h>
#include <linux/thermal.h>

struct ab_cooling {
	struct thermal_cooling_device *cdev;

	struct ab_cooling_ops ops;
	void *op_data;

	unsigned long cur_state;
};

static int ab_cooling_op_get_max_state(struct thermal_cooling_device *cdev,
		unsigned long *state)
{
	*state = AIRBRUSH_COOLING_STATE_MAX;
	return 0;
}

static int ab_cooling_op_get_cur_state(struct thermal_cooling_device *cdev,
		unsigned long *state)
{
	struct ab_cooling *cooling = cdev->devdata;

	*state = cooling->cur_state;
	return 0;
}

static int ab_cooling_op_set_cur_state(struct thermal_cooling_device *cdev,
		unsigned long state)
{
	struct ab_cooling *cooling = cdev->devdata;
	unsigned long old_state = cooling->cur_state;

	if (old_state == state)
		return 0;

	cooling->cur_state = state;
	cooling->ops.state_updated(cooling, old_state, state,
		cooling->op_data);
	return 0;
}

static const struct thermal_cooling_device_ops ab_cooling_ops = {
	.get_max_state = ab_cooling_op_get_max_state,
	.get_cur_state = ab_cooling_op_get_cur_state,
	.set_cur_state = ab_cooling_op_set_cur_state,
};

struct ab_cooling *ab_cooling_register(struct device_node *np, char *type,
		const struct ab_cooling_ops *ops, void *op_data)
{
	struct ab_cooling *cooling;
	int err;

	cooling = kmalloc(sizeof(struct ab_cooling), GFP_KERNEL);
	if (!cooling)
		return ERR_PTR(-ENOMEM);

	cooling->ops = *ops;
	cooling->op_data = op_data;

	cooling->cur_state = 0;
	cooling->cdev = thermal_of_cooling_device_register(np, type,
			cooling, &ab_cooling_ops);
	if (IS_ERR(cooling->cdev)) {
		err = PTR_ERR(cooling->cdev);
		kfree(cooling);
		return ERR_PTR(err);
	}

	return cooling;
}

void ab_cooling_unregister(struct ab_cooling *cooling)
{
	thermal_cooling_device_unregister(cooling->cdev);
	kfree(cooling);
}
