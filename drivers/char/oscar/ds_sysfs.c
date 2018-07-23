/* Copyright (C) 2017 Google, Inc.
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
#include "ds_sysfs.h"

#include "ds_generic.h"
#include "ds_logging.h"

/* struct ds_sysfs_mapping: Pair of kernel device and user-specified pointer.
 * Used in lookups in sysfs "show" functions to return user data.
 */

struct ds_sysfs_mapping {
	struct device *device;
	struct ds_dev *data;
	struct ds_sysfs_attribute *attributes;
	int attribute_count;
};

/*
 * struct ds_sysfs_mapping: Data needed to manage users of this sysfs utility.
 * Currently has a fixed size; if space is a concern, this can be dynamically
 * allocated.
 *
 * 'Global' (file-scoped) pointer to the backing sysfs mappings
 * between devices and ds_data pointers. This removes the requirement
 * to have a ds_sysfs_mapping handle in all files.
 */

static struct ds_sysfs_mapping dev_mappings[DS_SYSFS_NUM_MAPPINGS];

static int get_mapping_index(struct device *device)
{
	int i;

	for (i = 0; i < DS_SYSFS_NUM_MAPPINGS; i++) {
		if (dev_mappings[i].device == device)
			return i;
	}
	ds_nodev_info("Unable to find mapping for 0x%p", device);
	return -EINVAL;
}

int ds_sysfs_create_mapping(struct device *device, struct ds_dev *mapping_data)
{
	int ret;
	int map_idx = -1;

	ds_nodev_info(
		"Creating sysfs entries for device pointer 0x%p.", device);

	/* Check that the device we're initting hasn't already been initted. */
	ret = get_mapping_index(device);
	if (ret >= 0) {
		ds_nodev_error(
			"Attempting to re-initialize sysfs mapping for device 0x%p.",
			device);
		return -EINVAL;
	}

	/* Find the first empty entry in the array. */
	for (map_idx = 0; map_idx < DS_SYSFS_NUM_MAPPINGS; ++map_idx) {
		if (dev_mappings[map_idx].device == NULL)
			break;
	}
	if (map_idx == DS_SYSFS_NUM_MAPPINGS) {
		ds_nodev_error("All mappings have been exhausted");
		return -ENOMEM;
	}

	dev_mappings[map_idx].device = device;
	dev_mappings[map_idx].data = mapping_data;
	dev_mappings[map_idx].attributes = kzalloc(
		DS_SYSFS_MAX_NODES *
			sizeof(*(dev_mappings[map_idx].attributes)),
		GFP_KERNEL);
	dev_mappings[map_idx].attribute_count = 0;
	if (!dev_mappings[map_idx].attributes) {
		ds_nodev_error("Unable to allocate sysfs attribute array.");
		return -ENOMEM;
	}

	return 0;
}

int ds_sysfs_create_entries(
	struct device *device, const struct ds_sysfs_attribute *attrs)
{
	int i;
	int ret;
	struct ds_sysfs_mapping *mapping = NULL;

	ret = get_mapping_index(device);
	if (ret < 0) {
		ds_nodev_error(
			"Creating entries for device 0x%p without first initializing mapping.",
			device);
		return -EINVAL;
	}

	mapping = &(dev_mappings[ret]);
	for (i = 0; strcmp(attrs[i].attr.attr.name, DS_ARRAY_END_MARKER); i++) {
		if (mapping->attribute_count == DS_SYSFS_MAX_NODES) {
			ds_nodev_error(
				"Maximum number of sysfs nodes reached for device.");
			return -ENOMEM;
		}

		ret = device_create_file(device, &(attrs[i].attr));
		if (ret) {
			ds_nodev_error("Unable to create device entries");
			return ret;
		}

		mapping->attributes[mapping->attribute_count] = attrs[i];
		++mapping->attribute_count;
	}

	return 0;
}
EXPORT_SYMBOL(ds_sysfs_create_entries);

void ds_sysfs_remove_mapping(struct device *device)
{
	int i;
	int ret;
	struct ds_sysfs_mapping *mapping = NULL;

	ret = get_mapping_index(device);
	if (ret < 0) {
		ds_nodev_error(
			"Attempted to remove non-existent sysfs mapping to device 0x%p",
			device);
		return;
	}

	mapping = &(dev_mappings[ret]);
	/* First, we need to remove any created nodes. */
	for (i = 0; i < mapping->attribute_count; ++i)
		device_remove_file(device, &(mapping->attributes[i].attr));

	/* Clean up the actual mapping struct. */
	kfree(mapping->attributes);
	mapping->attributes = NULL;
	mapping->attribute_count = 0;
	mapping->data = NULL;
	mapping->device = NULL;
}

struct ds_dev *ds_sysfs_get_device_data(struct device *device)
{
	int ret;

	ret = get_mapping_index(device);
	if (ret < 0) {
		ds_nodev_error("device %p not registered.", device);
		return NULL;
	}

	return dev_mappings[ret].data;
}
EXPORT_SYMBOL(ds_sysfs_get_device_data);

ulong ds_sysfs_get_attr_data(
	struct device *device, struct device_attribute *attr)
{
	int i;
	int map_idx;
	int num_attrs;
	struct ds_sysfs_attribute *attrs = NULL;

	map_idx = get_mapping_index(device);
	if (map_idx < 0) {
		ds_nodev_error("No mapping for device 0x%p", device);
		/* This is less likely to be used as an enum or register value
		 * than 0.
		 */
		return ULONG_MAX;
	}

	attrs = dev_mappings[map_idx].attributes;
	num_attrs = dev_mappings[map_idx].attribute_count;
	for (i = 0; i < num_attrs; ++i) {
		if (!strcmp(attrs[i].attr.attr.name, attr->attr.name))
			return attrs[i].data;
	}

	ds_nodev_error("Unable to find match for device_attribute %s",
		attr->attr.name);
	/* This is less likely to be used as an enum or register value than 0.
	 */
	return ULONG_MAX;
}
EXPORT_SYMBOL(ds_sysfs_get_attr_data);

ulong ds_sysfs_get_attr_index(
	struct device *device, struct device_attribute *attr)
{
	int i;
	int map_idx;
	int num_attrs;
	struct ds_sysfs_attribute *attrs = NULL;

	map_idx = get_mapping_index(device);
	if (map_idx < 0) {
		ds_nodev_error("No mapping for device 0x%p", device);
		/* This is less likely to be used as an enum or register value
		 * than 0.
		 */
		return ULONG_MAX;
	}

	attrs = dev_mappings[map_idx].attributes;
	num_attrs = dev_mappings[map_idx].attribute_count;
	for (i = 0; i < num_attrs; ++i) {
		if (!strcmp(attrs[i].attr.attr.name, attr->attr.name))
			return attrs[i].index;
	}

	ds_nodev_error("Unable to find match for device_attribute %s",
		attr->attr.name);
	/* This is less likely to be used as an enum or register value than 0.
	 */
	return ULONG_MAX;
}

ssize_t ds_sysfs_register_show(
	struct device *device, struct device_attribute *attr, char *buf)
{
	ulong reg_address;
	ulong reg_bar;
	ulong reg_value;
	struct ds_dev *ds_dev =
		(struct ds_dev *)ds_sysfs_get_device_data(device);
	reg_address = ds_sysfs_get_attr_data(device, attr);
	reg_bar = ds_sysfs_get_attr_index(device, attr);

	if (ds_dev == NULL) {
		ds_nodev_error(
			"No sysfs mapping found for pointer 0x%p", device);
		return 0;
	}

	reg_value = ds_dev_read_64(ds_dev, reg_bar, reg_address);
	return snprintf(buf, PAGE_SIZE, "0x%lX\n", reg_value);
}
EXPORT_SYMBOL(ds_sysfs_register_show);
