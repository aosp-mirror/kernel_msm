/*
 * Copyright (C) 2017 Mobvoi, Inc.
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

#ifndef _NANOHUB_FUEL_GAUGE_H
#define _NANOHUB_FUEL_GAUGE_H

#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

#include "main.h"

#define FUEL_GAUGE_USE_FAKE_CAPACITY  0

struct bq27x00_reg_cache {
	uint8_t status;
	uint8_t present;

	uint16_t control;
	int16_t temperature;
	uint16_t voltage;
	uint16_t flags;
	uint16_t FullAvailableCapacity;
	uint16_t RemainingCapacity;
	uint16_t FullChargeCapacity;
	uint16_t AverageCurrent;
	/*uint16_t StateOfCharge;*/
	uint16_t RemainingCapacityUnfiltered;
	uint16_t FullChargeCapacityUnfiltered;
	uint16_t FullChargeCapacityFiltered;
	uint16_t StateOfChargeUnfiltered;

	/*uint16_t time_to_empty;
	uint16_t time_to_empty_avg;
	uint16_t time_to_full;*/
	uint32_t charge_full;
	/*uint16_t cycle_count;*/
	uint16_t capacity;
	/*uint16_t energy;*/
	uint16_t power_avg;
	uint16_t health;

	uint16_t charge_design_full;
	uint16_t NominalAvailableCapacity;/*in uAh*/
} __packed;

struct Nanohub_FuelGauge_Info {
	struct mutex lock;
	struct bq27x00_reg_cache cache;
	struct power_supply bat;
	struct delayed_work work;
	struct delayed_work request_delayed_work;
	struct wake_lock wakelock_report;
	struct nanohub_data *hub_data;
	struct device *dev;
	struct power_supply	*usb_psy;

	uint64_t ts_wakelock_report;
	uint64_t wakelock_active_time;
	unsigned long last_update;

	unsigned int  pre_interval;
	int charge_design_full;
#if FUEL_GAUGE_USE_FAKE_CAPACITY
	int fake_capacity;
#endif
	uint16_t last_capacity;
	bool charger_online;
	bool requested;
};



int bq27x00_powersupply_init(struct device *dev,
			struct nanohub_data *hub_data);
void bq27x00_powersupply_unregister(void);

int is_fuel_gauge_data(struct nanohub_buf *buf, int len);
int handle_fuelgauge_data(struct nanohub_buf *buf, int len);

#endif
