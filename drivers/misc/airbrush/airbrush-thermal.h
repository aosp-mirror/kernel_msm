/* SPDX-License-Identifier: GPL-2.0
 *
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
#ifndef _AIRBRUSH_THERMAL_H
#define _AIRBRUSH_THERMAL_H

#include <linux/airbrush-sm-ctrl.h>

#define AB_CDEV_NAME "ab"
#define AB_CDEV_INTERNAL_NAME "ab-internal"

struct ab_thermal;
struct device;

struct ab_thermal_ops {
	void (*throttle_state_updated)(
			enum throttle_state throttle_state_id, void *op_data);
};

void ab_thermal_set_ops(struct ab_thermal *thermal,
		const struct ab_thermal_ops *ops, void *op_data);

#endif /* _AIRBRUSH_THERMAL_H */
