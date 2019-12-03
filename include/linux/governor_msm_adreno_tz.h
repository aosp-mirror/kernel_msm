/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __GOVERNOR_MSM_ADRENO_TZ_H
#define __GOVERNOR_MSM_ADRENO_TZ_H

struct devfreq_notifiers {
	int (*add) (struct device *dev, struct notifier_block *nb);
	int (*delete) (struct device *dev, struct notifier_block *nb);
};

extern struct devfreq_notifiers msm_adreno_tz_notifiers;

#endif /* __GOVERNOR_MSM_ADRENO_TZ_H */
