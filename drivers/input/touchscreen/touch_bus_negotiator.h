/* drivers/input/touchscreen/touch_bus_negotiator.h
 *
 * Touch Bus Negotiator for Google Pixel devices.
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
 *
 */

#ifndef TOUCHSCREEN_BUS_NEGOTIATOR_H
#define TOUCHSCREEN_BUS_NEGOTIATOR_H

#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include "qmi_tbn_v01.h"

#define TBN_REQUEST_BUS_TIMEOUT_MS 500
#define TBN_RELEASE_BUS_TIMEOUT_MS 500

struct tbn_context {
	struct device *dev;
	struct qmi_handle *qmi_handle;
	struct notifier_block server_event_notifier;
	struct workqueue_struct *event_processor;
	struct work_struct on_server_arrive;
	struct work_struct on_server_exit;
	struct work_struct on_event;
	struct mutex service_lock;
};

struct tbn_context *tbn_init(struct device *dev);
void tbn_cleanup(struct tbn_context *);
int tbn_request_bus(struct tbn_context *);
int tbn_release_bus(struct tbn_context *);


#endif /* TOUCHSCREEN_BUS_NEGOTIATOR_H */
