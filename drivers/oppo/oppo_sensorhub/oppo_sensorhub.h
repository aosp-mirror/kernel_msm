/*
 * oppo_sensorhub.h - Linux kernel modules for OPPO SensorHub
 *
 * Copyright (C), 2008-2019, OPPO Mobile Comm Corp., Ltd.
 * Author: Zeng Zhaoxiu <zengzhaoxiu@oppo.com>
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#ifndef __LINUX_OPPO_SENSORHUB_H
#define __LINUX_OPPO_SENSORHUB_H

#include <linux/notifier.h>

struct oppo_sensorhub_platform_data {
    int gpio_reset;
    bool gpio_reset_active_low;
};

enum {
    SNSHUB_PRE_RESET,
    SNSHUB_POST_RESET,
};

int oppo_sensorhub_reset_notifier_register(struct device *dev, struct notifier_block *nb);
int oppo_sensorhub_reset_notifier_unregister(struct device *dev, struct notifier_block *nb);

void oppo_sensorhub_assert_reset(struct device *dev);
void oppo_sensorhub_deassert_reset(struct device *dev);
void oppo_sensorhub_reset(struct device *dev);

#endif
