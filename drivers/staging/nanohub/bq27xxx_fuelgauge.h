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


struct bq27x00_reg_cache {
	uint8_t status;
	uint8_t present;

	/*uint16_t control;*/
	uint16_t temperature;
	uint16_t voltage;
	uint16_t flags;
	uint16_t FullAvailableCapacity;
	uint16_t RemainingCapacity;
	uint16_t FullChargeCapacity;
	uint16_t AverageCurrent;
	/*uint16_t StateOfCharge;*/
	uint16_t RemainingCapacityUnfiltered;
	uint16_t FullChargeCapacityUnfiltered;
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
	struct device *dev;
	struct mutex lock;
	struct bq27x00_reg_cache cache;
	int charge_design_full;
	unsigned long last_update;
	struct power_supply bat;
	unsigned int  pre_interval;
	struct nanohub_data *hub_data;
	struct delayed_work work;
	bool requested;
};



int bq27x00_powersupply_init(struct device *dev,
			struct nanohub_data *hub_data);
void bq27x00_powersupply_unregister(void);

int enable_fuelgauge(struct nanohub_data *data, int on);
int is_fuel_gauge_data(struct nanohub_buf *buf, int len);


#endif

