/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Thiagu Ramalingam(thiagu.r@samsung.com)
 *
 * Airbrush power states driver .
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _AIRBRUSH_PMU_H_
#define _AIRBRUSH_PMU_H_

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/exynos5-pmu.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/types.h>

#include <linux/airbrush-sm-ctrl.h>

int ab_set_pm_state(struct ab_state_context *sc,
			uint32_t set_state, uint32_t device);
#endif
