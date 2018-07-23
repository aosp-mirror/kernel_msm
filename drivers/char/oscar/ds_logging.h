/* Common logging utilities for the DS driver framework.
 *
 * Copyright (C) 2017 Google, Inc.
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

#include <linux/pci.h>
#include <linux/printk.h>

#ifndef _DS_LOGGING_H_
#define _DS_LOGGING_H_

/* Base macro; other logging can/should be built on top of this. */
#define ds_dev_log(level, device, pci_dev, format, arg...)                     \
	{                                                                      \
		if (pci_dev) {                                                 \
			dev_##level(&(pci_dev)->dev, "%s: " format "\n",       \
				__func__, ##arg);                              \
		} else {                                                       \
			ds_nodev_log(level, format, ##arg);                    \
		}                                                              \
	}

/* "No-device" logging macros. */
#define ds_nodev_log(level, format, arg...)                                    \
	pr_##level("ds: %s: " format "\n", __func__, ##arg)

#define ds_nodev_debug(format, arg...) ds_nodev_log(debug, format, ##arg)

#define ds_nodev_info(format, arg...) ds_nodev_log(info, format, ##arg)

#define ds_nodev_warn(format, arg...) ds_nodev_log(warn, format, ##arg)

#define ds_nodev_error(format, arg...) ds_nodev_log(err, format, ##arg)

/* ds_dev logging macros */
#define ds_log_info(ds_dev, format, arg...)                                    \
	ds_dev_log(info, (ds_dev)->dev_info.device, (ds_dev)->pci_dev, format, \
		##arg)

#define ds_log_warn(ds_dev, format, arg...)                                    \
	ds_dev_log(warn, (ds_dev)->dev_info.device, (ds_dev)->pci_dev, format, \
		##arg)

#define ds_log_error(ds_dev, format, arg...)                                   \
	ds_dev_log(err, (ds_dev)->dev_info.device, (ds_dev)->pci_dev, format,  \
		##arg)

#define ds_log_debug(ds_dev, format, arg...)                                   \
	{                                                                      \
		if ((ds_dev)->pci_dev) {                                       \
			dev_dbg(&((ds_dev)->pci_dev)->dev, "%s: " format "\n", \
				__func__, ##arg);                              \
		} else {                                                       \
			ds_nodev_log(debug, format, ##arg);                    \
		}                                                              \
	}

#endif
