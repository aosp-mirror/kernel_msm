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
#include "sensor.h"

#include <linux/slab.h>
#include <linux/thermal.h>
#include "../../../thermal/thermal_core.h"

#include "hw.h"
#include "trim.h"

#define MCELSIUS	1000

static const u16 no_trimming_error1[] = {287, 286, 287, 287, 286, 286, 286};
static const u16 no_trimming_error2[] = {346, 346, 347, 347, 347, 346, 346};

struct ab_tmu_sensor {
	struct device *dev;
	struct ab_tmu_hw *hw;
	int id;
	struct ab_tmu_trim trim;
	struct thermal_zone_device *tzd;
};

static int ab_tmu_sensor_get_temp(void *data, int *temp)
{
	struct ab_tmu_sensor *sensor = data;
	struct ab_tmu_hw *hw = sensor->hw;
	bool pcie_link_ready;
	u32 code;

	if (!data)
		return -EINVAL;

	pcie_link_ready = ab_tmu_hw_pcie_link_lock(hw);
	if (pcie_link_ready) {
		code = ab_tmu_hw_read_current_temp(hw, sensor->id);
		*temp = ab_tmu_trim_raw_to_cel(&sensor->trim, code) * MCELSIUS;
	} else {
		/*
		 * Return 0 degree Celsius while TMU is not ready to
		 * suppress the error log while reading the temperature.
		 */
		*temp = 0;
	}
	ab_tmu_hw_pcie_link_unlock(hw);
	return 0;
}

static void ab_tmu_enable_emulate(struct ab_tmu_hw *hw)
{
	u32 emul_con;

	emul_con = ab_tmu_hw_read(hw, AB_TMU_EMUL_CON);
	emul_con |= (1 << AB_TMU_EMUL_ENABLE_SHIFT);

	ab_tmu_hw_write(hw, AB_TMU_EMUL_CON, emul_con);
}

static void ab_tmu_disable_emulate(struct ab_tmu_hw *hw)
{
	u32 emul_con;

	emul_con = ab_tmu_hw_read(hw, AB_TMU_EMUL_CON);
	emul_con &= ~(1 << AB_TMU_EMUL_ENABLE_SHIFT);

	ab_tmu_hw_write(hw, AB_TMU_EMUL_CON, emul_con);
}

static void ab_tmu_set_emulate_data(struct ab_tmu_hw *hw, u16 next_time,
		u16 next_data)
{
	u32 emul_con;

	next_time &= 0xFFFF;
	next_data &= 0x1FF;
	if (next_time == 0x0)
		next_time = 0x1;
	emul_con = (ab_tmu_hw_read(hw, AB_TMU_EMUL_CON) >> 0) & 0x1;
	emul_con |= (next_time << AB_TMU_EMUL_NEXTTIME_SHIFT) |
		(next_data << AB_TMU_EMUL_DATA_SHIFT);

	ab_tmu_hw_write(hw, AB_TMU_EMUL_CON, emul_con);
}

static int ab_tmu_sensor_set_emul_temp(void *data, int temp)
{
	struct ab_tmu_sensor *sensor = data;
	struct ab_tmu_hw *hw = sensor->hw;
	bool pcie_link_ready;
	u16 code;
	int ret;

	pcie_link_ready = ab_tmu_hw_pcie_link_lock(hw);
	if (pcie_link_ready) {
		if (temp != 0) {
			ab_tmu_enable_emulate(hw);
			code = ab_tmu_trim_cel_to_raw(&sensor->trim,
					temp / MCELSIUS);
			ab_tmu_set_emulate_data(hw, 1, code);
		} else {
			ab_tmu_disable_emulate(hw);
		}
		ret = 0;
	} else {
		ret = -ENODEV;
	}
	ab_tmu_hw_pcie_link_unlock(hw);

	return ret;
}

static struct thermal_zone_of_device_ops ab_tmu_sensor_ops = {
	.get_temp = ab_tmu_sensor_get_temp,
	.set_emul_temp = ab_tmu_sensor_set_emul_temp,
};

struct ab_tmu_sensor *devm_ab_tmu_sensor_create(struct device *dev,
		struct ab_tmu_hw *hw, int id)
{
	int err;
	struct ab_tmu_sensor *sensor;

	if (id < 0 || id >= AB_TMU_NUM_ALL_PROBE)
		return ERR_PTR(-EINVAL);

	sensor = devm_kzalloc(dev, sizeof(struct ab_tmu_sensor), GFP_KERNEL);
	if (!sensor)
		return ERR_PTR(-ENOMEM);

	sensor->dev = dev;
	sensor->hw = hw;
	sensor->id = id;
	sensor->trim.error1 = no_trimming_error1[id];
	sensor->trim.error2 = no_trimming_error2[id];
	sensor->tzd = devm_thermal_zone_of_sensor_register(dev, id, sensor,
			&ab_tmu_sensor_ops);
	if (IS_ERR(sensor->tzd)) {
		err = PTR_ERR(sensor->tzd);
		devm_kfree(dev, sensor);
		return ERR_PTR(err);
	}

	return sensor;
}

void ab_tmu_sensor_load_trim_info(struct ab_tmu_sensor *sensor)
{
	struct ab_tmu_hw *hw = sensor->hw;
	u32 trim_info;

	trim_info = ab_tmu_hw_read(hw, AB_TMU_TRIMINFO(sensor->id));

	sensor->trim.cal_type = (trim_info >> AB_TMU_CAL_SHIFT) &
			AB_TMU_CAL_MASK;
	if (sensor->trim.cal_type == AB_TMU_NO_TRIMMING)
		return;

	sensor->trim.error1 = trim_info & AB_TMU_TEMP_MASK;
	sensor->trim.error2 = (trim_info >> AB_TMU_TEMP_SHIFT) &
			AB_TMU_TEMP_MASK;
}

void ab_tmu_sensor_save_threshold(struct ab_tmu_sensor *sensor)
{
	struct thermal_zone_device *tz = sensor->tzd;
	struct ab_tmu_hw *hw = sensor->hw;
	unsigned int reg_off, bit_off;
	unsigned int rising_threshold = 0, falling_threshold = 0;
	int temp, temp_hist;
	int threshold_code;
	int j;

	for (j = (of_thermal_get_ntrips(tz) - 1); j >= 0; j--) {
		reg_off = ((7 - j) / 2) * 4;
		bit_off = ((8 - j) % 2);

		tz->ops->get_trip_temp(tz, j, &temp);
		temp /= MCELSIUS;

		tz->ops->get_trip_hyst(tz, j, &temp_hist);
		temp_hist = temp - (temp_hist / MCELSIUS);

		/* Set 9-bit temp code for rising threshold levels */
		threshold_code = ab_tmu_trim_cel_to_raw(&sensor->trim, temp);
		rising_threshold = ab_tmu_hw_read(hw,
				AB_TMU_THD_TEMP_RISE7_6_P(sensor->id) +
				reg_off);
		rising_threshold &= ~(AB_TMU_TEMP_MASK <<
				(16 * bit_off));
		rising_threshold |= threshold_code << (16 * bit_off);

		/* Set 9-bit temp code for falling threshold levels */
		threshold_code = ab_tmu_trim_cel_to_raw(&sensor->trim,
				temp_hist);
		falling_threshold = ab_tmu_hw_read(hw,
				AB_TMU_THD_TEMP_FALL7_6_P(sensor->id) +
				reg_off);
		falling_threshold &= ~(AB_TMU_TEMP_MASK <<
				(16 * bit_off));
		falling_threshold |= threshold_code << (16 * bit_off);

		ab_tmu_hw_write(hw, AB_TMU_THD_TEMP_RISE7_6_P(sensor->id) +
				reg_off, rising_threshold);
		ab_tmu_hw_write(hw, AB_TMU_THD_TEMP_FALL7_6_P(sensor->id) +
				reg_off, falling_threshold);
	}
}

void ab_tmu_sensor_update(struct ab_tmu_sensor *sensor)
{
	struct thermal_zone_device *tzd = sensor->tzd;
	int i, temp;
	char *envp[2];

	thermal_zone_device_update(tzd, THERMAL_EVENT_UNSPECIFIED);

	mutex_lock(&tzd->lock);
	/* Find the level for which trip happened */
	for (i = 0; i < of_thermal_get_ntrips(tzd); i++) {
		tzd->ops->get_trip_temp(tzd, i, &temp);
		if (tzd->last_temperature < temp)
			break;
	}

	envp[0] = kasprintf(GFP_KERNEL, "%d", i);
	envp[1] = NULL;
	kobject_uevent_env(&tzd->device.kobj, KOBJ_CHANGE, envp);
	kfree(envp[0]);
	mutex_unlock(&tzd->lock);
}

int ab_tmu_sensor_raw_to_cel(const struct ab_tmu_sensor *sensor, u16 raw)
{
	return ab_tmu_trim_raw_to_cel(&sensor->trim, raw);
}
