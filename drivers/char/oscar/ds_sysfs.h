/* Set of common sysfs utilities.
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

/* The functions described here are a set of utilities to allow each file in the
 * DS driver framework to manage their own set of sysfs entries, instead of
 * centralizing all that work in one file.
 *
 * The goal of these utilities is to allow for sysfs entries to be easily
 * created without causing a proliferation of sysfs "show" functions. This
 * requires O(N) string lookups during show function execution, but as reading
 * sysfs entries is rarely performance-critical, this is likely acceptible.
 */
#ifndef __DS_SYSFS_H__
#define __DS_SYSFS_H__

#include "ds_constants.h"
#include "ds_generic.h"
#include <linux/device.h>
#include <linux/stringify.h>
#include <linux/sysfs.h>

/* The maximum number of mappings/devices a driver needs to support. */
#define DS_SYSFS_NUM_MAPPINGS (DS_FRAMEWORK_DESC_MAX * DS_DEV_MAX)

/* The maximum number of sysfs nodes in a directory.
 */
#define DS_SYSFS_MAX_NODES 196

/* End markers for DS sysfs struct arrays. */
#define DS_ARRAY_END_TOKEN DS_RESERVED_ARRAY_END
#define DS_ARRAY_END_MARKER __stringify(DS_ARRAY_END_TOKEN)

/*
 * Terminator struct for a ds_sysfs_attr array. Must be at the end of
 * all ds_sysfs_attribute arrays.
 */
#define DS_END_OF_ATTR_ARRAY                                                   \
	{                                                                      \
		__ATTR(DS_ARRAY_END_TOKEN, S_IRUGO, NULL, NULL), 0, 0          \
	}

/*
 * struct ds_sysfs_attribute: Pairing of sysfs attribute and user data.
 * Used in lookups in sysfs "show" functions to return attribute metadata.
 */
struct ds_sysfs_attribute {
	struct device_attribute attr;
	ulong data;
	ulong index;
};

#define DS_SYSFS_RO(_name, _disp_function, _data)                              \
	{                                                                      \
		__ATTR(_name, S_IRUGO, _disp_function, NULL), _data            \
	}
#define DS_SYSFS_REG(_name, _offset, _bar)                                     \
	{                                                                      \
		__ATTR(_name, S_IRUGO, ds_sysfs_register_show, NULL)           \
		, _offset, _bar                                                \
	}

/*
 * ds_sysfs_create_mapping: Create an entry in mapping_data between a device
 * and a ds_dev struct.
 * @device: Device struct to map ds_dev to.
 * @ds_dev: The ds_dev struct associated with the driver controlling @device.
 *
 * Description: This function maps a ds_dev* to a device*. This mapping can
 * be used in sysfs_show functions to get a handle to the ds_dev struct
 * controlling the device node.
 *
 * If this function is not called before ds_sysfs_create_entries, a ds_warn
 * will be logged.
 */
int ds_sysfs_create_mapping(struct device *device, struct ds_dev *mapping_data);

/*
 * ds_sysfs_create_entries: Creates a file's entries in sysfs.
 * @sysfs_data: Pointer to file's/user's sysfs context.
 * @device: Kernel device structure.
 * @attrs: List of attributes/sysfs entries to create.
 *
 * Description: Creates each sysfs entry described in "attrs". Can be called
 * multiple times for a given @device.
 */
int ds_sysfs_create_entries(
	struct device *device, const struct ds_sysfs_attribute *attrs);

/*
 * ds_sysfs_remove_mapping: Removes a device*-ds_dev* mapping from the global
 * table.
 * @device: Device to unmap.
 *
 * Description: Sets both device and the ds_dev mapped to it out of the table.
 */
void ds_sysfs_remove_mapping(struct device *device);

/*
 * ds_sysfs_get_device_data: User data lookup based on kernel device structure.
 * @device: Kernel device structure.
 *
 * Description: Returns the user data associated with "device" in a prior call
 * to ds_sysfs_create_entries. Returns NULL if no mapping can be found.
 */
struct ds_dev *ds_sysfs_get_device_data(struct device *device);

/*
 * ds_sysfs_get_attr_data: User data lookup based on attribute structure.
 * @attrs: List of attributes/sysfs entries.
 * @attr: Device attribute to look up.
 *
 * Description: Returns the user data associated with "attr" in the input
 * "attrs" array. This lookup could be done entirely in user code; it's provided
 * here as a common utility.
 */
ulong ds_sysfs_get_attr_data(
	struct device *device, struct device_attribute *attr);

/*
 * ds_sysfs_get_attr_index: User data lookup based on attribute structure.
 * @attrs: List of attributes/sysfs entries.
 * @attr: Device attribute to look up.
 *
 * Description: Returns the user data associated with "attr" in the input
 * "attrs" array. This lookup could be done entirely in user code; it's provided
 * here as a common utility.
 */
ulong ds_sysfs_get_attr_index(
	struct device *device, struct device_attribute *attr);

/*
 * ds_sysfs_register_show: Display a register as a sysfs node.
 */
ssize_t ds_sysfs_register_show(
	struct device *device, struct device_attribute *attr, char *buf);

#endif /* __DS_SYSFS_H__ */
