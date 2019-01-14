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
#ifndef _AIRBRUSH_TMU_DRVDATA_
#define _AIRBRUSH_TMU_DRVDATA_

#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "hw.h"

struct ab_tmu_sensor;

/**
 * struct ab_tmu_drvdata : hold the private data of the TMU driver
 * @irq_work: pointer to the irq work structure.
 * @hw: handle to hw register access and pcie link event.
 * @sensor: handles to sensor operation, the id of sensor is the same as
 * its index at this array.
 */
struct ab_tmu_drvdata {
	struct ab_tmu_hw *hw;
	int irq;
	struct notifier_block tmu_nb;
	struct work_struct irq_work;
	spinlock_t sensor_irq_lock;
	u32 sensor_irq[AB_TMU_NUM_ALL_PROBE];
	struct ab_tmu_sensor *sensor[AB_TMU_NUM_ALL_PROBE];
};

#endif /* _AIRBRUSH_TMU_DRVDATA_ */
