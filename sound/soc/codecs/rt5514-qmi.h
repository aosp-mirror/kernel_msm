// SPDX-License-Identifier: GPL-2.0
/* sound/soc/codecs/rt5514-qmi.h
 *
 * RT5514 qmi driver for Google Pixel devices.
 *
 * Copyright (C) 2020 Google, Inc.
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

#ifndef RT5514_QMI_H
#define RT5514_QMI_H

#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include "rtk_spi_v01.h"

typedef void (rt5514_qmi_callback)(uint8_t error_code);

struct rt5514_qmi_instance {
	struct device *dev;
	struct qmi_handle qmi_handle;
	struct mutex service_lock;
	struct work_struct svc_arrive_work;
	struct workqueue_struct *qmi_wq;
	bool connected;
	rt5514_qmi_callback *func_cb;
};

struct rt5514_qmi_instance *rt5514_qmi_init(struct device *dev,
					    rt5514_qmi_callback *cb);
void rt5514_qmi_cleanup(struct rt5514_qmi_instance *instance);

#endif /* RT5514_QMI_H */
