/* DS generic driver framework. This file contains the implementation
 * for the DS generic driver framework - the functionality that is common
 * across DS devices.
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
#include "ds_generic.h"

#include "ds_interrupt.h"
#include "ds_ioctl.h"
#include "ds_logging.h"
#include "ds_page_table.h"
#include "ds_sysfs.h"
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/iommu.h>

#ifdef DS_KERNEL_TRACE_SUPPORT
#define CREATE_TRACE_POINTS
#include <trace/events/ds_mmap.h>
#else
#define trace_ds_mmap_exit(x)
#define trace_ds_mmap_entry(x, ...)
#endif

/*
 * struct ds_internal_desc: "Private" members of ds_driver_desc.
 *
 * Contains internal per-device type tracking data, i.e., data not appropriate
 * as part of the public interface for the generic framework.
 */
struct ds_internal_desc {
	/* Device-specific-driver-provided configuration information. */
	struct ds_driver_desc *driver_desc;

	/* Protects access to per-driver data (i.e. this structure). */
	struct mutex mutex;

	/* Kernel-internal device class. */
	struct class *class;

	/* PCI subsystem metadata associated with this driver. */
	struct pci_driver pci;

	/* Instantiated / present devices of this type. */
	struct ds_dev *devs[DS_DEV_MAX];
};

/*
 * ds_cdev_get_data: Retrieve device-specific data via cdev pointer.
 * @cdev_ptr: Character device pointer associated with the device.
 *
 * This function returns the pointer to the device-specific data allocated in
 * add_dev_cb for the device associated with cdev_ptr.
 */
static struct ds_cdev_info *ds_cdev_get_info(struct cdev *cdev_ptr)
{
	return container_of(cdev_ptr, struct ds_cdev_info, cdev);
}

/*
 * ds_owned_by_current_tgid: Returns nonzero if the ds_cdev_info is owned by
 * the current thread group ID.
 * @info: Device node info.
 */
static int ds_owned_by_current_tgid(struct ds_cdev_info *info)
{
	return (info->ownership.is_owned &&
		(info->ownership.owner == current->tgid));
}

/* Function declarations; comments are with definitions. */
static int __init ds_init(void);
static void __exit ds_exit(void);
#ifndef PLATFORM_INTERFACE
static int ds_pci_probe(
	struct pci_dev *pci_dev, const struct pci_device_id *id);
static void ds_pci_remove(struct pci_dev *pci_dev);

static int ds_setup_pci(struct pci_dev *pci_dev, struct ds_dev *dev);
static void ds_cleanup_pci(struct ds_dev *dev);

static int ds_map_pci_bar(struct ds_dev *dev, int bar_num);
static void ds_unmap_pci_bar(struct ds_dev *dev, int bar_num);
#endif

static int ds_alloc_dev(struct ds_internal_desc *internal_desc,
	struct device *dev, struct ds_dev **pdev, const char *kobj_name);
static void ds_free_dev(struct ds_dev *dev);

static int ds_find_dev_slot(
	struct ds_internal_desc *internal_desc, const char *kobj_name);

static int ds_add_cdev(struct ds_cdev_info *dev_info,
	const struct file_operations *file_ops, struct module *owner);

static int ds_enable_dev(
	struct ds_internal_desc *internal_desc, struct ds_dev *dev);
static void ds_disable_dev(struct ds_dev *dev);

#ifndef PLATFORM_INTERFACE
static struct ds_internal_desc *lookup_internal_desc(struct pci_dev *pci_dev);
#endif
static struct ds_internal_desc *lookup_internal_platform_desc(
	struct platform_device *pdev);

static ssize_t ds_sysfs_data_show(
	struct device *device, struct device_attribute *attr, char *buf);

static int ds_mmap(struct file *filp, struct vm_area_struct *vma);
static int ds_open(struct inode *inode, struct file *file);
static int ds_release(struct inode *inode, struct file *file);
static long ds_ioctl(struct file *filp, uint cmd, ulong arg);

static int ds_get_hw_status(struct ds_dev *dev);

static int dwn_platform_driver_probe(struct platform_device *pdev);

/* Global data definitions. */
/* Mutex - only for framework-wide data. Other data should be protected by
 * finer-grained locks.
 */
static DEFINE_MUTEX(g_mutex);

/* List of all registered device descriptions & their supporting data. */
static struct ds_internal_desc g_descs[DS_FRAMEWORK_DESC_MAX];

/* status name table. Must end with {0,NULL}. */
const struct ds_num_name ds_status_name_table[] = {
	{ DS_STATUS_DEAD, "DEAD" },
	{ DS_STATUS_ALIVE, "ALIVE" },
	{ DS_STATUS_LAMED, "LAMED" },
	{ DS_STATUS_DRIVER_EXIT, "DRIVER_EXITING" },
	{ 0, NULL },
};

/* enum sysfs_attribute_type: enumeration of the automatic ds framework
 * sysfs nodes.
 */
enum ds_sysfs_attribute_type {
	ATTR_DRIVER_VERSION,
	ATTR_FRAMEWORK_VERSION,
	ATTR_DEVICE_TYPE,
	ATTR_HARDWARE_REVISION,
	ATTR_PCI_ADDRESS,
	ATTR_STATUS,
	ATTR_IS_DEVICE_OWNED,
	ATTR_DEVICE_OWNER,
	ATTR_WRITE_OPEN_COUNT,
	ATTR_RESET_COUNT,
	ATTR_USER_MEM_RANGES
};

/* File operations for all ds devices. */
static const struct file_operations ds_file_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.mmap = ds_mmap,
	.open = ds_open,
	.release = ds_release,
	.unlocked_ioctl = ds_ioctl,
};

/* These attributes apply to all ds driver instances. */
static const struct ds_sysfs_attribute ds_sysfs_generic_attrs[] = {
	DS_SYSFS_RO(driver_version, ds_sysfs_data_show, ATTR_DRIVER_VERSION),
	DS_SYSFS_RO(
		framework_version, ds_sysfs_data_show, ATTR_FRAMEWORK_VERSION),
	DS_SYSFS_RO(device_type, ds_sysfs_data_show, ATTR_DEVICE_TYPE),
	DS_SYSFS_RO(revision, ds_sysfs_data_show, ATTR_HARDWARE_REVISION),
	DS_SYSFS_RO(pci_address, ds_sysfs_data_show, ATTR_PCI_ADDRESS),
	DS_SYSFS_RO(status, ds_sysfs_data_show, ATTR_STATUS),
	DS_SYSFS_RO(is_device_owned, ds_sysfs_data_show, ATTR_IS_DEVICE_OWNED),
	DS_SYSFS_RO(device_owner, ds_sysfs_data_show, ATTR_DEVICE_OWNER),
	DS_SYSFS_RO(
		write_open_count, ds_sysfs_data_show, ATTR_WRITE_OPEN_COUNT),
	DS_SYSFS_RO(reset_count, ds_sysfs_data_show, ATTR_RESET_COUNT),
	DS_SYSFS_RO(user_mem_ranges, ds_sysfs_data_show, ATTR_USER_MEM_RANGES),
	DS_END_OF_ATTR_ARRAY
};

MODULE_DESCRIPTION("Google DS driver framework");
MODULE_VERSION(DS_FRAMEWORK_VERSION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mark Omernick <momernick@google.com>");
module_init(ds_init);
module_exit(ds_exit);
/*
 * ds_check_and_invoke_callback: Perform a standard DS callback.
 * @ds_dev: Device specific pointer to forward.
 * @cb_function: Standard callback to perform.
 *
 */
static inline int ds_check_and_invoke_callback(
	struct ds_dev *ds_dev, int (*cb_function)(struct ds_dev *))
{
	int ret = 0;

	ds_nodev_error("ds_check_and_invoke_callback %p", ds_dev);
	if (cb_function) {
		mutex_lock(&ds_dev->mutex);
		ret = cb_function(ds_dev);
		mutex_unlock(&ds_dev->mutex);
	}
	return ret;
}

/*
 * ds_check_and_invoke_callback_nolock: Perform a standard DS callback
 * without grabbing ds_dev->mutex.
 * @ds_dev: Device specific pointer to forward.
 * @cb_function: Standard callback to perform.
 *
 */
static inline int ds_check_and_invoke_callback_nolock(
	struct ds_dev *ds_dev, int (*cb_function)(struct ds_dev *))
{
	int ret = 0;

	if (cb_function) {
		ds_log_info(ds_dev, "Invoking device-specific callback.");
		ret = cb_function(ds_dev);
	}
	return ret;
}

static int __init ds_init(void)
{
	int i;

	ds_nodev_info("Performing one-time init of the DS framework.");
	/* Check for duplicates and find a free slot. */
	mutex_lock(&g_mutex);

	for (i = 0; i < DS_FRAMEWORK_DESC_MAX; i++) {
		g_descs[i].driver_desc = NULL;
		mutex_init(&g_descs[i].mutex);
	}

	mutex_unlock(&g_mutex);
	return 0;
}

static void __exit ds_exit(void)
{
	ds_nodev_info("Removing DS framework module.");
}

static int dwn_platform_driver_probe(struct platform_device *pdev)
{
	struct resource *r;

	int ret;
	const char *kobj_name = dev_name(&pdev->dev);
	struct ds_internal_desc *internal_desc;
	struct ds_dev *dev;
	struct ds_driver_desc *driver_desc;
	struct device *parent;

	struct abc_device *abc_dev = pdev->dev.platform_data;

	ds_nodev_info("add ds platform device %s dma_ops %p %p", kobj_name,
		pdev, &(pdev->dev));

	mutex_lock(&g_mutex);
	internal_desc = lookup_internal_platform_desc(pdev);
	mutex_unlock(&g_mutex);
	if (!internal_desc) {
		ds_nodev_info(
			"Platform Device probe called for unknown driver type");
		return -ENODEV;
	}

	driver_desc = internal_desc->driver_desc;

/* Get the IO memory resource for this device, this corresponds
 * to the TN BAR, at index DW_TN_BAR_INDEX
 * TODO: sane way to share the BAR index with driver specific code
 */
#define DW_TN_BAR_INDEX 0
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "cannot get mem resource\n");
		return -ENODEV;
	}

	parent = &(pdev->dev);

	ret = ds_alloc_dev(internal_desc, parent, &dev, kobj_name);
	if (ret)
		return ret;
	if (IS_ERR_OR_NULL(dev->dev_info.device)) {
		ds_nodev_error("cannot create %s device %s [ret = %ld]",
			driver_desc->name, dev->dev_info.name,
			PTR_ERR(dev->dev_info.device));
		ret = -ENODEV;
		goto fail1;
	}
	dev->platform_dev = pdev;

	/* Map BARs */
	dev->bar_data[DW_TN_BAR_INDEX].phys_base = r->start;
	dev->bar_data[DW_TN_BAR_INDEX].length_bytes = r->end - r->start;

	/* TODO(vandwalle)
	 * Check the mem reqion is released properly in the error path.
	 * This doesn't seem to be done.
	 *
	if (!request_mem_region(dev->bar_data[DW_TN_BAR_INDEX].phys_base,
		    dev->bar_data[DW_TN_BAR_INDEX].length_bytes,
		    dev->dev_info.name)) {
		ds_log_error(dev, "cannot request BAR %d memory region ",
			DW_TN_BAR_INDEX);
		ret = -EINVAL;
		goto fail1;
	}
	*/

	dev->bar_data[DW_TN_BAR_INDEX].virt_base = abc_dev->tpu_config;
	if (!dev->bar_data[DW_TN_BAR_INDEX].virt_base) {
		ds_log_error(dev, "cannot remap BAR %d memory region",
			DW_TN_BAR_INDEX);
		ret = -ENOMEM;
		goto fail1;
	}
	ds_nodev_info("dwn platform device %s: remapped BAR2 at %p", kobj_name,
		dev->bar_data[DW_TN_BAR_INDEX].virt_base);

	ret = ds_check_and_invoke_callback(dev, driver_desc->add_dev_cb);
	if (ret) {
		ds_log_error(dev, "Error in add device cb: %d", ret);
		goto fail2;
	}
	/* TODO(vandwalle)
	 * Check if fail3 is correct here, since ds_sysfs_remove_mapping()
	 * would get to be called.
	 */
	ret = ds_sysfs_create_mapping(dev->dev_info.device, dev);
	if (ret)
		goto fail3;

	/*
	 * Once we've created the mapping structures successfully, attempt to
	 * create a symlink to the parent directory of this object.
	 */
	ret = sysfs_create_link(&dev->dev_info.device->kobj, &parent->kobj,
		dev_name(&pdev->dev));
	if (ret) {
		ds_log_error(dev, "Cannot create sysfs platform link: %d", ret);
		goto fail3;
	}

	ret = ds_sysfs_create_entries(
		dev->dev_info.device, ds_sysfs_generic_attrs);
	if (ret)
		goto fail4;

	ret = ds_check_and_invoke_callback(dev, driver_desc->sysfs_setup_cb);
	if (ret) {
		ds_log_error(dev, "Error in sysfs setup cb: %d", ret);
		goto fail5;
	}

	if (driver_desc->device_reset_cb) {
		/* Perform a device reset. */
		ret = driver_desc->device_reset_cb(dev, 0);
		if (ret)
			ds_log_error(dev, "Device reset cb returned %d.", ret);
	}

	ret = ds_enable_dev(internal_desc, dev);
	if (ret) {
		ds_nodev_error("cannot setup %s device", driver_desc->name);
		goto fail6;
	}
	if (driver_desc->device_close_cb) {
		/* Perform a device cleanup, so as to place it in Power Save
		 * Mode.
		 */
		ret = driver_desc->device_close_cb(dev);
		if (ret)
			ds_log_error(
				dev, "Device cleanup cb returned %d.", ret);
	}

	return 0;

fail6:
	ds_disable_dev(dev);
fail5:
	ds_check_and_invoke_callback(dev, driver_desc->sysfs_cleanup_cb);
fail4:
fail3:
	ds_sysfs_remove_mapping(dev->dev_info.device);
fail2:
	ds_check_and_invoke_callback(dev, driver_desc->remove_dev_cb);
	device_destroy(internal_desc->class, dev->dev_info.devt);
fail1:
	ds_free_dev(dev);
	return ret;

	return 0;
}

/*
 * TODO: Abstract this function since it has 99% common code with pci_remove
 */
static int dwn_platform_driver_remove(struct platform_device *pdev)
{
	int i;
	struct ds_internal_desc *internal_desc;
	struct ds_dev *dev = NULL;
	struct ds_driver_desc *driver_desc;

	/* Find the device desc. */
	mutex_lock(&g_mutex);
	internal_desc = lookup_internal_platform_desc(pdev);
	if (!internal_desc) {
		mutex_unlock(&g_mutex);
		return 0;
	}
	driver_desc = internal_desc->driver_desc;
	mutex_unlock(&g_mutex);

	/* Now find the specific device */
	mutex_lock(&internal_desc->mutex);
	for (i = 0; i < DS_USER_DEV_MAX; i++) {
		if (internal_desc->devs[i] &&
			internal_desc->devs[i]->platform_dev == pdev) {
			dev = internal_desc->devs[i];
			break;
		}
	}
	mutex_unlock(&internal_desc->mutex);

	if (!dev)
		return 0;

	ds_disable_dev(dev);

	iounmap(dev->bar_data[DW_TN_BAR_INDEX].virt_base);
	release_mem_region(dev->bar_data[DW_TN_BAR_INDEX].phys_base,
		dev->bar_data[DW_TN_BAR_INDEX].length_bytes);

	ds_check_and_invoke_callback(dev, driver_desc->sysfs_cleanup_cb);
	ds_sysfs_remove_mapping(dev->dev_info.device);

	ds_check_and_invoke_callback(dev, driver_desc->remove_dev_cb);

	device_destroy(internal_desc->class, dev->dev_info.devt);
	ds_free_dev(dev);

	return 0;
}

#undef DW_TN_BAR_INDEX

static const struct of_device_id dwn_of_match[] = {
	{
		.compatible = "google,knoll",
	},
	{},
};
MODULE_DEVICE_TABLE(of, dwn_of_match);

static struct platform_driver dwn_platform_driver = {
		.probe = dwn_platform_driver_probe,
		.remove = dwn_platform_driver_remove,
		.driver = {
			.name = DRV_NAME_ABC_PCIE_TPU,
			.of_match_table = dwn_of_match,
		} };

/* See ds_generic.h for description. */
int ds_register_device(struct ds_driver_desc *driver_desc)
{
	int i, ret;
	int desc_idx = -1;
	struct ds_internal_desc *internal;

	ds_nodev_info("Initializing DS framework device");
	/* Check for duplicates and find a free slot. */
	mutex_lock(&g_mutex);

	for (i = 0; i < DS_FRAMEWORK_DESC_MAX; i++) {
		if (g_descs[i].driver_desc == driver_desc) {
			ds_nodev_error("%s driver already loaded/registered",
				driver_desc->name);
			mutex_unlock(&g_mutex);
			return -EBUSY;
		}
	}

	/* This and the above loop could be combined, but this reads easier. */
	for (i = 0; i < DS_FRAMEWORK_DESC_MAX; i++) {
		if (!g_descs[i].driver_desc) {
			g_descs[i].driver_desc = driver_desc;
			desc_idx = i;
			break;
		}
	}
	mutex_unlock(&g_mutex);

	ds_nodev_info("loaded %s driver, framework version %s",
		driver_desc->name, DS_FRAMEWORK_VERSION);
	if (desc_idx == -1) {
		ds_nodev_error("too many ds drivers loaded: %d\n",
			DS_FRAMEWORK_DESC_MAX);
		return -EBUSY;
	}

	/* Internal structure setup. */
	ds_nodev_info("Performing initial internal structure setup.");
	internal = &g_descs[desc_idx];
	mutex_init(&internal->mutex);
	memset(internal->devs, 0, sizeof(struct ds_dev *) * DS_DEV_MAX);
	memset(&internal->pci, 0, sizeof(internal->pci));

	if (driver_desc->pci_id_table != NULL) {
#ifndef PLATFORM_INTERFACE
		internal->pci.name = driver_desc->name;
		internal->pci.id_table = driver_desc->pci_id_table;
		internal->pci.probe = ds_pci_probe;
		internal->pci.remove = ds_pci_remove;
		internal->class =
			class_create(driver_desc->module, driver_desc->name);

		if (IS_ERR_OR_NULL(internal->class)) {
			ds_nodev_error("cannot register %s class [ret=%ld]",
				driver_desc->name, PTR_ERR(internal->class));
			return PTR_ERR(internal->class);
		}

		ds_nodev_info("Registering PCI driver.");
		/*
		 * Not using pci_register_driver() (without underscores), as it
		 * depends on KBUILD_MODNAME, and this is a shared file.
		 */
		ret = __pci_register_driver(
			&internal->pci, driver_desc->module, driver_desc->name);
		if (ret) {
			ds_nodev_error(
				"cannot register pci driver [ret=%d]", ret);
			goto fail1;
		}
#else
		ds_nodev_error("cannot register pci driver, need CONFIG_PCI");
		ret = -1;
		goto fail1;
#endif
	} else {
		ds_nodev_info("Registering Platform driver.");
		internal->class =
			class_create(driver_desc->module, driver_desc->name);

		ret = platform_driver_register(&dwn_platform_driver);
		if (ret) {
			ds_nodev_error("Cannot register platform "
				       "driver [ret=%d]",
				ret);
			goto fail1;
		}
	}

	ds_nodev_info("Registering char driver.");
	ret = register_chrdev_region(
		MKDEV(driver_desc->major, driver_desc->minor), DS_DEV_MAX,
		driver_desc->name);
	if (ret) {
		ds_nodev_error("cannot register char driver [ret=%d]", ret);
		goto fail2;
	}

	ds_nodev_info("Driver registered successfully.");
	return 0;

fail2:
#ifndef PLATFORM_INTERFACE
	if (driver_desc->pci_id_table != NULL)
		pci_unregister_driver(&internal->pci);
#endif

fail1:
	class_destroy(internal->class);

	g_descs[desc_idx].driver_desc = NULL;
	return ret;
}
EXPORT_SYMBOL(ds_register_device);

/* See ds_generic.h for description. */
void ds_unregister_device(struct ds_driver_desc *driver_desc)
{
	int i, desc_idx;
	struct ds_internal_desc *internal_desc = NULL;

	mutex_lock(&g_mutex);
	for (i = 0; i < DS_FRAMEWORK_DESC_MAX; i++) {
		if (g_descs[i].driver_desc == driver_desc) {
			internal_desc = &g_descs[i];
			desc_idx = i;
			break;
		}
	}
	mutex_unlock(&g_mutex);

	if (internal_desc == NULL) {
		ds_nodev_error("request to unregister unknown desc: %s, %d:%d",
			driver_desc->name, driver_desc->major,
			driver_desc->minor);
		return;
	}

	unregister_chrdev_region(
		MKDEV(driver_desc->major, driver_desc->minor), DS_DEV_MAX);

	if (driver_desc->pci_id_table != NULL) {
#ifndef PLATFORM_INTERFACE
		pci_unregister_driver(&internal_desc->pci);
#endif
	} else {
		ds_nodev_info("Unregistering Platform driver.");
		platform_driver_unregister(&dwn_platform_driver);
	}

	class_destroy(internal_desc->class);

	/* Finally, effectively "remove" the driver. */
	g_descs[desc_idx].driver_desc = NULL;

	ds_nodev_info("removed %s driver", driver_desc->name);
}
EXPORT_SYMBOL(ds_unregister_device);

/**
 * ds_alloc_dev - Allocate a DS device.
 * @internal_desc: Pointer to the internal data for the device driver.
 * @pdev: Pointer to the DS device pointer, the allocated device.
 * @kobj_name: PCIe name for the device
 *
 * Description: Allocates and initializes a DS device structure.
 *              Adds the device to the device list.
 *
 * Returns 0 if successful, a negative error code otherwise.
 */
static int ds_alloc_dev(struct ds_internal_desc *internal_desc,
	struct device *parent, struct ds_dev **pdev, const char *kobj_name)
{
	int dev_idx;
	struct ds_driver_desc *driver_desc = internal_desc->driver_desc;
	struct ds_dev *dev;
	struct ds_cdev_info *dev_info;

	ds_nodev_info("Allocating a ds device %s.", kobj_name);

	*pdev = NULL;

	dev_idx = ds_find_dev_slot(internal_desc, kobj_name);
	if (dev_idx < 0)
		return dev_idx;

	dev = *pdev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ds_nodev_error("no memory for device");
		return -ENOMEM;
	}
	internal_desc->devs[dev_idx] = dev;

	mutex_init(&dev->mutex);

	dev->internal_desc = internal_desc;
	dev->dev_idx = dev_idx;
	snprintf(dev->kobj_name, DS_NAME_MAX, "%s", kobj_name);
	/* ds_bar_data is uninitialized. */
	dev->num_page_tables = driver_desc->num_page_tables;
	/* max_page_table_size and *page table are uninit'ed */
	/* interrupt_data is not initialized. */
	/* status is 0, or DS_STATUS_DEAD */

	dev_info = &(dev->dev_info);
	snprintf(dev_info->name, DS_NAME_MAX, "%s_%u", driver_desc->name,
		dev->dev_idx);
	dev_info->devt =
		MKDEV(driver_desc->major, driver_desc->minor + dev->dev_idx);
	dev_info->device = device_create(internal_desc->class, parent,
		dev_info->devt, dev, dev_info->name);

	ds_nodev_info("platform device create %p.", dev_info->device);

	/* cdev has not yet been added; cdev_added is 0 */
	dev_info->ds_dev_ptr = dev;
	/* ownership is all 0, indicating no owner or opens. */

	return 0;
}

/*
 * ds_find_dev_slot: Finds the next free ds_internal_dev_slot.
 *
 * Returns the located slot number on success or a negative number on failure.
 */
static int ds_find_dev_slot(
	struct ds_internal_desc *internal_desc, const char *kobj_name)
{
	int i;

	mutex_lock(&internal_desc->mutex);

	/* Search for a previous instance of this device. */
	for (i = 0; i < DS_DEV_MAX; i++) {
		if (internal_desc->devs[i] &&
			strcmp(internal_desc->devs[i]->kobj_name, kobj_name) ==
				0) {
			ds_nodev_error("duplicate device %s", kobj_name);
			mutex_unlock(&internal_desc->mutex);
			return -EBUSY;
		}
	}

	/* Find a free device slot. */
	for (i = 0; i < DS_USER_DEV_MAX; i++) {
		if (!internal_desc->devs[i])
			break;
	}

	if (i == DS_USER_DEV_MAX) {
		ds_nodev_info(
			"Too many registered devices; max %d", DS_USER_DEV_MAX);
		mutex_unlock(&internal_desc->mutex);
		return -EBUSY;
	}

	mutex_unlock(&internal_desc->mutex);
	return i;
}

/*
 * ds_free_dev - Free a DS device.
 * @internal_dev: DS device pointer; the device to unregister and free.
 *
 * Description: Removes the device from the device list and frees
 *              the DS device structure.
 */
static void ds_free_dev(struct ds_dev *dev)
{
	struct ds_internal_desc *internal_desc = dev->internal_desc;

	mutex_lock(&internal_desc->mutex);
	internal_desc->devs[dev->dev_idx] = NULL;
	mutex_unlock(&internal_desc->mutex);

	kfree(dev);
}

#ifndef PLATFORM_INTERFACE
/**
 * ds_pci_probe: PCI subsystem probe function.
 * @pci_dev: PCI device pointer to the new device.
 * @id: PCI device id structure pointer, the vendor and device ids.
 *
 * Called when a DS device is found. Allocates device metadata, maps device
 * memory, and calls ds_enable_dev to prepare the device for active use.
 *
 * Returns 0 if successful and a negative value otherwise.
 */
static int ds_pci_probe(struct pci_dev *pci_dev, const struct pci_device_id *id)
{
#ifndef PLATFORM_INTERFACE
	int ret;
	int read_attribs;
	const char *kobj_name = dev_name(&pci_dev->dev);
	struct ds_internal_desc *internal_desc;
	struct ds_dev *dev;
	struct ds_driver_desc *driver_desc;
	struct device *parent;

	ds_nodev_info("add ds device %s", kobj_name);

	mutex_lock(&g_mutex);
	internal_desc = lookup_internal_desc(pci_dev);
	mutex_unlock(&g_mutex);
	if (!internal_desc) {
		ds_nodev_info("PCI probe called for unknown driver type");
		return -ENODEV;
	}

	driver_desc = internal_desc->driver_desc;

	parent = &(pci_dev->dev);
	ret = ds_alloc_dev(internal_desc, parent, &dev, kobj_name);
	if (ret)
		return ret;
	if (IS_ERR_OR_NULL(dev->dev_info.device)) {
		ds_nodev_error("cannot create %s device %s [ret = %ld]",
			driver_desc->name, dev->dev_info.name,
			PTR_ERR(dev->dev_info.device));
		ret = -ENODEV;
		goto fail1;
	}
	dev->pci_dev = pci_dev;

	ret = ds_setup_pci(pci_dev, dev);
	if (ret)
		goto fail2;

	ret = ds_check_and_invoke_callback(dev, driver_desc->add_dev_cb);
	if (ret) {
		ds_log_error(dev, "Error in add device cb: %d", ret);
		goto fail2;
	}

	ret = ds_sysfs_create_mapping(dev->dev_info.device, dev);
	if (ret)
		goto fail3;

	/*
	 * Once we've created the mapping structures successfully, attempt to
	 * create a symlink to the pci directory of this object.
	 */
	ret = sysfs_create_link(&dev->dev_info.device->kobj, &pci_dev->dev.kobj,
		dev_name(&pci_dev->dev));
	if (ret) {
		ds_log_error(dev, "Cannot create sysfs pci link: %d", ret);
		goto fail3;
	}
	ret = ds_sysfs_create_entries(
		dev->dev_info.device, ds_sysfs_generic_attrs);
	if (ret)
		goto fail4;

	ret = ds_check_and_invoke_callback(dev, driver_desc->sysfs_setup_cb);
	if (ret) {
		ds_log_error(dev, "Error in sysfs setup cb: %d", ret);
		goto fail5;
	}

	ret = ds_enable_dev(internal_desc, dev);
	if (ret) {
		ds_nodev_error("cannot setup %s device", driver_desc->name);
		goto fail6;
	}
	if (dev->pci_dev) {

		/* Create IOMMU/SMMU maping, Attach IOMMU */
		dev->dma_mapping = arm_iommu_create_mapping(
			dev->pci_dev->dev.bus, (u32) 0, (SZ_1G * 64ULL));
		if (!dev->dma_mapping) {
			ds_log_error(dev, "IOMMU create mapping failed\n");
			goto fail6;
		}

		/* Set IOMMU attributes: atomic S1 bypass */
		ret = iommu_domain_set_attr(dev->dma_mapping->domain,
				DOMAIN_ATTR_ATOMIC, &read_attribs);
		if (ret) {
			ds_log_error(dev,
				"IOMMU set attr atomic failed %d\n", ret);
			goto fail7;
		}

		ret = iommu_domain_set_attr(dev->dma_mapping->domain,
				DOMAIN_ATTR_S1_BYPASS, &read_attribs);
		if (ret) {
			ds_log_error(dev,
				"IOMMU set attr atomic failed %d\n", ret);
			goto fail7;
		}

		ret = arm_iommu_attach_device(&(dev->pci_dev->dev),
				dev->dma_mapping);
		if (ret) {
			ds_log_error(dev,
				"IOMMU attach failed (%d)\n", ret);
			goto fail7;
		}
		ds_nodev_info("IOMMU attached successfully");
	}
	return 0;
fail7:
	arm_iommu_release_mapping(dev->dma_mapping);
fail6:
	ds_disable_dev(dev);
fail5:
	ds_check_and_invoke_callback(dev, driver_desc->sysfs_cleanup_cb);
fail4:
fail3:
	ds_sysfs_remove_mapping(dev->dev_info.device);
fail2:
	ds_cleanup_pci(dev);
	ds_check_and_invoke_callback(dev, driver_desc->remove_dev_cb);
	device_destroy(internal_desc->class, dev->dev_info.devt);
fail1:
	ds_free_dev(dev);
	return ret;
#endif
	return 0;
}

/*
 * ds_pci_remove: PCI subsystem remove function.
 * @pci_dev: PCI device pointer; the device to remove.
 *
 * Called to remove a DS device. Finds the device in the device list and
 * cleans up metadata.
 */
static void ds_pci_remove(struct pci_dev *pci_dev)
{
	int i;
	struct ds_internal_desc *internal_desc;
	struct ds_dev *dev = NULL;
	struct ds_driver_desc *driver_desc;
	/* Find the device desc. */
	mutex_lock(&g_mutex);
	internal_desc = lookup_internal_desc(pci_dev);
	if (!internal_desc) {
		mutex_unlock(&g_mutex);
		return;
	}
	driver_desc = internal_desc->driver_desc;
	mutex_unlock(&g_mutex);

	/* Now find the specific device */
	mutex_lock(&internal_desc->mutex);
	for (i = 0; i < DS_USER_DEV_MAX; i++) {
		if (internal_desc->devs[i] &&
			internal_desc->devs[i]->pci_dev == pci_dev) {
			dev = internal_desc->devs[i];
			break;
		}
	}
	mutex_unlock(&internal_desc->mutex);

	if (!dev)
		return;

	ds_nodev_info("remove %s device %s", internal_desc->driver_desc->name,
		dev->kobj_name);

	ds_disable_dev(dev);
	ds_cleanup_pci(dev);

	ds_check_and_invoke_callback(dev, driver_desc->sysfs_cleanup_cb);
	ds_sysfs_remove_mapping(dev->dev_info.device);

	ds_check_and_invoke_callback(dev, driver_desc->remove_dev_cb);

	device_destroy(internal_desc->class, dev->dev_info.devt);
	ds_free_dev(dev);
}

/*
 * ds_setup_pci: Setup PCI & set up memory mapping for the specified device.
 * @pci_dev: pointer to the particular PCI device.
 * @internal_dev: Corresponding DS device pointer.
 *
 * Enables the PCI device, reads the BAR registers and sets up pointers to the
 * device's memory mapped IO space.
 *
 * Returns 0 on success and a negative value otherwise.
 */
static int ds_setup_pci(struct pci_dev *pci_dev, struct ds_dev *dev)
{
	int i, mapped_bars, ret;

	/* TODO @(mahdi)
	 * Disable PCIe timeout, must be removed after bringup
	 */
	struct pci_dev *pci_dev_trav;

	dev->pci_dev = pci_dev;
	ret = pci_enable_device(pci_dev);
	if (ret) {
		ds_log_error(dev, "cannot enable PCI device");
		return ret;
	}

	pci_set_master(pci_dev);

	for (i = 0; i < DS_NUM_BARS; i++) {
		ret = ds_map_pci_bar(dev, i);
		if (ret) {
			mapped_bars = i;
			goto fail;
		}
	}

	/* TODO @(mahdi)
	 * Disable PCIe timeout, must be removed after bringup
	 */
	pci_dev_trav = NULL;
	while (true) {
		pci_dev_trav = pci_get_device(PCI_ANY_ID,
				PCI_ANY_ID, pci_dev_trav);
		if (pci_dev_trav == NULL)
			break;
		pcie_capability_clear_and_set_word(pci_dev_trav,
			PCI_EXP_DEVCTL2, PCI_EXP_DEVCTL2_COMP_TIMEOUT, 0x1d);
	}
	return 0;

fail:
	for (i = 0; i < mapped_bars; i++)
		ds_unmap_pci_bar(dev, i);

	pci_disable_device(pci_dev);
	return -ENOMEM;
}

/* ds_cleanup_pci: Unmaps memory and cleans up PCI for the specified device. */
static void ds_cleanup_pci(struct ds_dev *dev)
{

#ifndef PLATFORM_INTERFACE
	int i;

	for (i = 0; i < DS_NUM_BARS; i++)
		ds_unmap_pci_bar(dev, i);

	/* Detach IOMMU and then release its mapping */
	arm_iommu_detach_device(&(dev->pci_dev->dev));
	ds_nodev_info("iommu detached");

	arm_iommu_release_mapping(dev->dma_mapping);
	ds_nodev_info("mapping released");


	pci_disable_device(dev->pci_dev);
#endif
}

/*
 * ds_map_pci_bar: Maps the specified bar into kernel space.
 * @internal_dev: Device possessing the BAR to map.
 * @bar_num: The BAR to map.
 *
 * Returns 0 on success, a negative error code otherwise.
 * A zero-sized BAR will not be mapped, but is not an error.
 */
static int ds_map_pci_bar(struct ds_dev *dev, int bar_num)
{
	struct ds_internal_desc *internal_desc = dev->internal_desc;
	struct ds_driver_desc *driver_desc = internal_desc->driver_desc;
	ulong desc_bytes = driver_desc->bar_descriptions[bar_num].size;
	int ret;

	if (desc_bytes == 0)
		return 0;

	if (driver_desc->bar_descriptions[bar_num].type != PCI_BAR) {
		/* not PCI: skip this entry */
		return 0;
	}
	/*
	 * pci_resource_start and pci_resource_len return a "resource_size_t",
	 * which is safely castable to ulong (which itself is the arg to
	 * request_mem_region).
	 */
	dev->bar_data[bar_num].phys_base =
		(ulong)pci_resource_start(dev->pci_dev, bar_num);
	if (!dev->bar_data[bar_num].phys_base) {
		ds_log_error(dev, "cannot get PCI BAR%u base address", bar_num);
		return -EINVAL;
	}

	dev->bar_data[bar_num].length_bytes =
		(ulong)pci_resource_len(dev->pci_dev, bar_num);
	if (dev->bar_data[bar_num].length_bytes < desc_bytes) {
		ds_log_error(dev,
			"PCI BAR %u space is too small: %lu; expected >= %lu",
			bar_num, dev->bar_data[bar_num].length_bytes,
			desc_bytes);
		return -ENOMEM;
	}

	if (!request_mem_region(dev->bar_data[bar_num].phys_base,
		    dev->bar_data[bar_num].length_bytes, dev->dev_info.name)) {
		ds_log_error(dev, "cannot get BAR %d memory region %pR",
			bar_num, &dev->pci_dev->resource[bar_num]);
		return -EINVAL;
	}

	dev->bar_data[bar_num].virt_base =
		ioremap_nocache(dev->bar_data[bar_num].phys_base,
			dev->bar_data[bar_num].length_bytes);
	if (!dev->bar_data[bar_num].virt_base) {
		ds_log_error(dev, "cannot remap BAR %d memory region %pR",
			bar_num, &dev->pci_dev->resource[bar_num]);
		ret = -ENOMEM;
		goto fail2;
	}

	dma_set_mask(&dev->pci_dev->dev, DMA_BIT_MASK(64));
	dma_set_coherent_mask(&dev->pci_dev->dev, DMA_BIT_MASK(64));

	return 0;

fail2:
	iounmap(dev->bar_data[bar_num].virt_base);
	release_mem_region(dev->bar_data[bar_num].phys_base,
		dev->bar_data[bar_num].length_bytes);
	return ret;
}

/*
 * ds_unmap_pci_bar: Releases PCI BAR mapping.
 * @internal_dev: Device possessing the BAR to unmap.
 *
 * A zero-sized or not-mapped BAR will not be unmapped, but is not an error.
 */
static void ds_unmap_pci_bar(struct ds_dev *dev, int bar_num)
{
	ulong base, bytes;
	struct ds_internal_desc *internal_desc = dev->internal_desc;
	struct ds_driver_desc *driver_desc = internal_desc->driver_desc;

	if (driver_desc->bar_descriptions[bar_num].size == 0 ||
		!dev->bar_data[bar_num].virt_base)
		return;

	if (driver_desc->bar_descriptions[bar_num].type != PCI_BAR)
		return;

	iounmap(dev->bar_data[bar_num].virt_base);
	dev->bar_data[bar_num].virt_base = NULL;

	base = pci_resource_start(dev->pci_dev, bar_num);
	if (!base) {
		ds_log_error(dev, "cannot get PCI BAR%u base address", bar_num);
		return;
	}

	bytes = pci_resource_len(dev->pci_dev, bar_num);
	release_mem_region(base, bytes);
}
#endif

/*
 * ds_add_cdev: Handle adding a char device and related info.
 * @dev_info: Pointer to the dev_info struct for this device.
 * @file_ops: The file operations for this device.
 * @owner: The owning module for this device.
 *
 */
static int ds_add_cdev(struct ds_cdev_info *dev_info,
	const struct file_operations *file_ops, struct module *owner)
{
	int ret;

	cdev_init(&dev_info->cdev, file_ops);
	dev_info->cdev.owner = owner;
	ret = cdev_add(&(dev_info->cdev), dev_info->devt, 1);
	if (ret) {
		ds_log_error(dev_info->ds_dev_ptr,
			"cannot add char device [ret=%d]", ret);
		return ret;
	}
	dev_info->cdev_added = 1;

	return 0;
}

/*
 * ds_enable_dev: Performs final init and mark the device as active.
 * @internal_desc: Pointer to DS [internal] driver descriptor structure.
 * @internal_dev: Pointer to DS [internal] device structure.
 *
 * Currently forwards all work to device-specific callback; a future phase will
 * extract elements of character device registration here.
 */
static int ds_enable_dev(
	struct ds_internal_desc *internal_desc, struct ds_dev *dev)
{
	int i;
	int ret;
	bool has_dma_ops;
	struct device *ddev;

	struct ds_driver_desc *driver_desc = internal_desc->driver_desc;

	ret = ds_interrupt_init(dev, driver_desc->name,
		driver_desc->interrupt_type, driver_desc->interrupts,
		driver_desc->num_interrupts, driver_desc->interrupt_pack_width,
		driver_desc->interrupt_bar_index,
		driver_desc->wire_interrupt_offsets);
	if (ret) {
		ds_log_error(dev, "Critical failure to allocate interrupts: %d",
			ret);
		ds_interrupt_cleanup(dev);
		return ret;
	}

	/* This lets individual driver specify which dma_ops they need to
	 * operate with, for instance platform_drivers on emulators may
	 * not be able to work with the dma_ops that are set automatically
	 * by kernel, or driver with specific iommu might have to set their own
	 *
	 * TODO(vandwalle): Sanitize driver_desc structure, i.e. we shouldn't
	 * rely on pci_id_table pointer, but rely on a per-driver setting.
	 */
	if (driver_desc->pci_id_table)
		has_dma_ops = true;
	else
		has_dma_ops = true;

	for (i = 0; i < driver_desc->num_page_tables; i++) {
		ds_log_info(dev, "Initializing page table %d bar index %d.", i,
			driver_desc->page_table_bar_index);
		ddev = ds_get_device(dev);
		if (!ddev) {
			ds_log_error(
				dev, "no physical device!!");
			WARN_ON(1);
		}
		ret = ds_page_table_init(&(dev->page_table[i]),
			&(dev->bar_data[driver_desc->page_table_bar_index]),
			&(driver_desc->page_table_offsets[i]),
			driver_desc->page_table_extended_bit, ddev,
			has_dma_ops);
		if (ret) {
			ds_log_error(
				dev, "Couldn't init page table %d: %d", i, ret);
			return ret;
		}
		/* Make sure that the page table is clear and set to simple
		 * addresses.
		 */
		ds_page_table_reset(dev->page_table[i]);
	}

	/* hardware_revision_cb returns a positive integer (the rev) if
	 * successful.)
	 */
	ret = ds_check_and_invoke_callback(
		dev, driver_desc->hardware_revision_cb);
	if (ret < 0) {
		ds_log_error(dev, "Error getting hardware revision: %d", ret);
		return ret;
	}
	dev->hardware_revision = ret;

	ret = ds_check_and_invoke_callback(dev, driver_desc->enable_dev_cb);
	if (ret) {
		ds_log_error(dev, "Error in enable device cb: %d", ret);
		return ret;
	}

	/* device_status_cb returns a device status, not an error code. */
	dev->status = ds_get_hw_status(dev);
	if (dev->status == DS_STATUS_DEAD)
		ds_log_error(dev, "Device reported as unhealthy.");

	ret = ds_add_cdev(&dev->dev_info, &ds_file_ops, driver_desc->module);
	if (ret)
		return ret;

	return 0;
}

/*
 * ds_disable_dev: Disable device operations.
 * @internal_dev: Pointer to DS [internal] device structure.
 *
 * Currently forwards all work to device-specific callback; a future phase will
 * extract elements of character device unregistration here.
 */
static void ds_disable_dev(struct ds_dev *dev)
{
	struct ds_driver_desc *driver_desc = dev->internal_desc->driver_desc;
	int i;

	/* Only delete the device if it has been successfully added. */
	if (dev->dev_info.cdev_added)
		cdev_del(&dev->dev_info.cdev);

	dev->status = DS_STATUS_DEAD;

	ds_interrupt_cleanup(dev);

	for (i = 0; i < driver_desc->num_page_tables; ++i) {
		if (dev->page_table[i] != NULL) {
			ds_page_table_reset(dev->page_table[i]);
			ds_page_table_cleanup(dev->page_table[i]);
		}
	}

	ds_check_and_invoke_callback(dev, driver_desc->disable_dev_cb);
}

/**
 * lookup_internal_platform_desc: Registered descriptor lookup.
 *
 * Precondition: Called with g_mutex held (to avoid a race on return).
 * Returns NULL if no matching device was found.
 */
static struct ds_internal_desc *lookup_internal_platform_desc(
	struct platform_device *pdev)
{
	int i;

	__must_hold(&g_mutex);
	for (i = 0; i < DS_FRAMEWORK_DESC_MAX; i++) {
		if (g_descs[i].driver_desc &&
			!g_descs[i].driver_desc->pci_id_table)
			/* TODO : chip specific matching */
			return &g_descs[i];
	}

	return NULL;
}

#ifndef PLATFORM_INTERFACE
/**
 * lookup_internal_desc: Registered descriptor lookup.
 *
 * Precondition: Called with g_mutex held (to avoid a race on return).
 * Returns NULL if no matching device was found.
 */
static struct ds_internal_desc *lookup_internal_desc(struct pci_dev *pci_dev)
{
	int i;

	__must_hold(&g_mutex);
	for (i = 0; i < DS_FRAMEWORK_DESC_MAX; i++) {
		if (g_descs[i].driver_desc &&
			g_descs[i].driver_desc->pci_id_table &&
			pci_match_id(
				g_descs[i].driver_desc->pci_id_table, pci_dev))
			return &g_descs[i];
	}

	return NULL;
}
#endif

/**
 * ds_num_name_lookup - Lookup a name by number in a num_name table.
 * @num: Number to lookup.
 * @table: Array of num_name structures, the table for the lookup.
 *
 * Description: Searches for num in the table.  If found, the
 *		corresponding name is returned; otherwise NULL
 *		is returned.
 *
 *		The table must have a NULL name pointer at the end.
 */
const char *ds_num_name_lookup(uint num, const struct ds_num_name *table)
{
	uint i = 0;

	while (table[i].snn_name) {
		if (num == table[i].snn_num)
			break;
		++i;
	}

	return table[i].snn_name;
}
EXPORT_SYMBOL(ds_num_name_lookup);

/**
 * ds_open - Open of the char device file.
 * @inode: Inode structure pointer of the device file.
 * @file: File structure pointer.
 *
 * Description: Called on an open of the device file.  If the open is for
 *              writing, and the device is not owned, this process becomes
 *              the owner.  If the open is for writing and the device is
 *              already owned by some other process, it is an error.  If
 *              this process is the owner, increment the open count.
 *
 *              Returns 0 if successful, a negative error number otherwise.
 */
static int ds_open(struct inode *inode, struct file *file)
{
	int ret;
	struct ds_dev *ds_dev;
	struct ds_driver_desc *driver_desc;
	struct ds_ownership *ownership;
	char task_name[TASK_COMM_LEN];
	struct ds_cdev_info *dev_info = ds_cdev_get_info(inode->i_cdev);

	if (!dev_info) {
		ds_nodev_error("Unable to retrieve device data");
		return -EINVAL;
	}
	ds_dev = dev_info->ds_dev_ptr;
	driver_desc = ds_dev->internal_desc->driver_desc;
	ownership = &(dev_info->ownership);
	get_task_comm(task_name, current);
	file->private_data = ds_dev;
	inode->i_size = 0;

	ds_log_info(ds_dev,
		"Attempting to open with tgid %u (%s) (f_mode: 0%03o, "
		"fmode_write: %d is_root: %u)",
		current->tgid, task_name, file->f_mode,
		(file->f_mode & FMODE_WRITE), capable(CAP_SYS_ADMIN));

	/* Always allow non-writing accesses. */
	if (!(file->f_mode & FMODE_WRITE)) {
		ds_log_info(ds_dev, "Allowing read-only opening.");
		return 0;
	}

	mutex_lock(&ds_dev->mutex);

	ds_log_info(ds_dev, "Current write open count (owning tgid %u): %d.",
		ownership->owner, ownership->write_open_count);

	/* Opening a node owned by another TGID is an error (even root.) */
	if (ownership->is_owned && ownership->owner != current->tgid) {
		ds_log_error(ds_dev, "Process %u is opening a node held by %u.",
			current->tgid, ownership->owner);
		mutex_unlock(&ds_dev->mutex);
		return -EPERM;
	}

	/* If the node is not owned, assign it to the current TGID. */
	if (!ownership->is_owned) {
		ret = ds_check_and_invoke_callback_nolock(
			ds_dev, driver_desc->device_open_cb);
		if (ret) {
			ds_log_error(
				ds_dev, "Error in device open cb: %d", ret);
			mutex_unlock(&ds_dev->mutex);
			return ret;
		}
		ownership->is_owned = 1;
		ownership->owner = current->tgid;
		ds_log_info(ds_dev, "Device owner is now tgid %u",
			ownership->owner);
	}

	ownership->write_open_count++;

	ds_log_info(ds_dev, "Success: New open count (owning tgid %u): %d",
		ownership->owner, ownership->write_open_count);

	mutex_unlock(&ds_dev->mutex);
	return 0;
}

/**
 * ds_release - Close of the char device file.
 * @inode: Inode structure pointer of the device file.
 * @file: File structure pointer.
 *
 * Description: Called on a close of the device file.  If this process
 *              is the owner, decrement the open count.  On last close
 *              by the owner, free up buffers and eventfd contexts, and
 *              release ownership.
 *
 *              Returns 0 if successful, a negative error number otherwise.
 */
static int ds_release(struct inode *inode, struct file *file)
{
	int i;
	struct ds_ownership *ownership;
	struct ds_dev *ds_dev;
	struct ds_driver_desc *driver_desc;
	char task_name[TASK_COMM_LEN];
	struct ds_cdev_info *dev_info =
		(struct ds_cdev_info *)ds_cdev_get_info(inode->i_cdev);
	if (!dev_info) {
		ds_nodev_error("Unable to retrieve device data");
		return -EINVAL;
	}
	ds_dev = dev_info->ds_dev_ptr;
	driver_desc = ds_dev->internal_desc->driver_desc;
	ownership = &(dev_info->ownership);
	get_task_comm(task_name, current);
	mutex_lock(&ds_dev->mutex);

	ds_log_info(ds_dev,
		"Releasing device node. Call origin: tgid %u (%s) (f_mode: 0%03o, "
		"fmode_write: %d, is_root: %u)",
		current->tgid, task_name, file->f_mode,
		(file->f_mode & FMODE_WRITE), capable(CAP_SYS_ADMIN));
	ds_log_info(ds_dev, "Current open count (owning tgid %u): %d",
		ownership->owner, ownership->write_open_count);

	if (file->f_mode & FMODE_WRITE) {
		ownership->write_open_count--;
		if (ownership->write_open_count == 0) {
			ds_log_info(ds_dev, "Device is now free");
			ownership->is_owned = 0;
			ownership->owner = 0;

			/* Forces chip reset before we unmap the page tables. */
			driver_desc->device_reset_cb(ds_dev, 0);

			for (i = 0; i < driver_desc->num_page_tables; ++i) {
				ds_page_table_unmap_all(ds_dev->page_table[i]);
				ds_page_table_garbage_collect(
					ds_dev->page_table[i]);
				ds_free_coherent_memory_all(ds_dev, i);
			}

			/* Closes device, enters power save. */
			ds_check_and_invoke_callback_nolock(
				ds_dev, driver_desc->device_close_cb);
		}
	}

	ds_log_info(ds_dev, "New open count (owning tgid %u): %d",
		ownership->owner, ownership->write_open_count);
	mutex_unlock(&ds_dev->mutex);
	return 0;
}

/*
 * ds_mmap_has_permissions: Permission and validity checking for mmap ops.
 * @ds_dev: DS device information structure.
 * @vma: Standard virtual memory area descriptor.
 *
 * Verifies that the user has permissions to perform the requested mapping and
 * that the provided descriptor/range is of adequate size to hold the range to
 * be mapped.
 */
static int ds_mmap_has_permissions(
	struct ds_dev *ds_dev, struct vm_area_struct *vma, int bar_permissions)
{
	int requested_permissions;
	/* Always allow sysadmin to write */
	if (capable(CAP_SYS_ADMIN))
		return 1;

	/* Never allow non-sysadmins to write to a dead device. */
	if (ds_dev->status != DS_STATUS_ALIVE) {
		ds_log_info(ds_dev, "Device is dead.");
		return 0;
	}

	/* Make sure that no wrong flags are set. */
	requested_permissions =
		(vma->vm_flags & (VM_WRITE | VM_READ | VM_EXEC));
	if (requested_permissions & ~(bar_permissions)) {
		ds_log_info(ds_dev,
			"Attempting to map a region with requested permissions 0x%x, "
			"but region has permissions 0x%x.",
			requested_permissions, bar_permissions);
		return 0;
	}

	/* Do not allow a non-owner to write. */
	if ((vma->vm_flags & VM_WRITE) &&
		!ds_owned_by_current_tgid(&(ds_dev->dev_info))) {
		ds_log_info(ds_dev,
			"Attempting to mmap a region for write without owning device.");
		return 0;
	}

	return 1;
}

/*
 * ds_is_coherent_region: Checks if an address is within the region allocated
 * for coherent buffer
 * @driver_desc: driver description.
 * @address: offset of address to check
 *
 * Verifies that the input address is within the region allocated to coherent
 * buffer
 */

static bool ds_is_coherent_region(struct ds_driver_desc *driver_desc,
		ulong address)
{
	struct ds_coherent_buffer_desc coh_buff_desc =
		driver_desc->coherent_buffer_description;

	if (coh_buff_desc.permissions != DS_NOMAP) {
		if ((address >= coh_buff_desc.base) &&
				(address < (coh_buff_desc.base +
					    coh_buff_desc.size))) {
			return true;
		}
	}
	/* If we haven't found the address by now, it is invalid. */
	return false;
}


static int ds_get_bar_index(struct ds_driver_desc *driver_desc, ulong address)
{
	int i;

	for (i = 0; i < DS_NUM_BARS; ++i) {
		struct ds_bar_desc bar_desc = driver_desc->bar_descriptions[i];

		if (bar_desc.permissions != DS_NOMAP) {
			if ((address >= bar_desc.base) &&
				(address < (bar_desc.base + bar_desc.size))) {
				return i;
			}
		}
	}
	/* If we haven't found the address by now, it is invalid. */
	return -EINVAL;
}

/*
 * ds_mmap: Maps a device's BARs into user space.
 * @filp: File structure pointer describing this node usage session.
 * @vma: Standard virtual memory area descriptor.
 *
 * Maps the entirety of each of the device's BAR ranges into the user memory
 * range specified by vma.
 *
 * Returns 0 on success, a negative errno on error.
 */
static int ds_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int i, ret;
	int bar_index;
	int has_mapped = 0;
	ulong phys_offset, mapping_offset, map_length;
	ulong raw_offset, requested_offset, requested_length;
	ulong range_start, range_length, range_end;
	ulong permissions;
	bool is_coherent_region;
	struct ds_driver_desc *driver_desc;


	struct ds_dev *ds_dev = (struct ds_dev *)filp->private_data;
	struct ds_bar_data *bar_data;
	struct ds_bar_desc *bar_desc;

	if (!ds_dev) {
		ds_nodev_error("Unable to retrieve ds device data");
		trace_ds_mmap_exit(-EINVAL);
		return -EINVAL;
	}
	driver_desc = ds_dev->internal_desc->driver_desc;

	/* Calculate the offset of this range into physical mem. */
	raw_offset = (vma->vm_pgoff << PAGE_SHIFT) +
		     driver_desc->legacy_mmap_address_offset;
	requested_length = vma->vm_end - vma->vm_start;
	trace_ds_mmap_entry(
		ds_dev->dev_info.name, raw_offset, requested_length);

	/* check if index is within a bar region, if not,
	 * check coherent buffer
	 */
	bar_index = ds_get_bar_index(driver_desc, raw_offset);
	is_coherent_region = ds_is_coherent_region(driver_desc, raw_offset);
	if (bar_index < 0 && !is_coherent_region) {
		ds_log_error(ds_dev,
			"Unable to find matching bar for address 0x%lx",
			raw_offset);
		trace_ds_mmap_exit(bar_index);
		return bar_index;
	}
	if (bar_index > 0 && is_coherent_region) {
		ds_log_error(ds_dev,
			"double matching bar and coherent"
			"buffers for address 0x%lx",
			raw_offset);
		trace_ds_mmap_exit(bar_index);
		return bar_index;
	}

	/* Subtract the base of the bar from the raw offset to get the memory
	 * location within the bar to map.
	 */
	if (is_coherent_region) {
		requested_offset = raw_offset -
			driver_desc->coherent_buffer_description.base;
		vma->vm_private_data = ds_dev;
		permissions = driver_desc
			->coherent_buffer_description.permissions;
	} else {
		requested_offset = raw_offset -
			driver_desc->bar_descriptions[bar_index].base;
		bar_data = &ds_dev->bar_data[bar_index];
		vma->vm_private_data = ds_dev;
		permissions = driver_desc
			->bar_descriptions[bar_index].permissions;
		bar_desc = &(driver_desc->bar_descriptions[bar_index]);
	}
	if (!ds_mmap_has_permissions(ds_dev, vma, permissions)) {
		ds_log_error(ds_dev, "Permission checking failed.");
		trace_ds_mmap_exit(-EPERM);
		return -EPERM;
	}

	if (vma->vm_start & (PAGE_SIZE - 1)) {
		ds_log_error(ds_dev, "Base address not page-aligned: 0x%p\n",
			(void *)vma->vm_start);
		trace_ds_mmap_exit(-EINVAL);
		return -EINVAL;
	}

	if (is_coherent_region) {
		if (requested_length == 0 || requested_length >
				ds_dev->coherent_buffer.length_bytes) {
			trace_ds_mmap_exit(-EINVAL);
			return -EINVAL;
		}

		/* Keep track if we've mapped any memory at all. */
		has_mapped = 1;
		// TODO (mahdih): needs investigation
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

		ret = remap_pfn_range(vma, vma->vm_start,
			(ds_dev->coherent_buffer.phys_base) >> PAGE_SHIFT,
			requested_length, vma->vm_page_prot);
		if (ret) {
			ds_log_error(ds_dev,
				"Error remapping PFN range err=%d.", ret);
			trace_ds_mmap_exit(ret);
			return ret;
		}
		/* Record the user virtual to dma_address mapping that was
		 * created by the kernel
		 */
		ds_log_error(ds_dev,
				"mapping coh buffer %lu.", requested_length);
		ds_set_user_virt(ds_dev, requested_length,
			ds_dev->coherent_buffer.phys_base, vma->vm_start);
	} else {
		/* Note that all address cases (incl. above and below mappable
		 *  region) are handled in the address scanning code below.
		 */
		for (i = 0; i < bar_desc->num_mappable_regions; i++) {
			range_start = bar_desc->mappable_regions[i].start;
			range_length =
				bar_desc->mappable_regions[i].length_bytes;
			range_end = range_start + range_length;
			if (requested_offset + requested_length < range_start) {
				/*
				 * If the requested region is completely below
				 * the range, there is nothing to map.
				 */
				continue;

			} else if (requested_offset <= range_start) {
				/* If the requested offset is below this range's
				 * start but the requested length continues into
				 * it: 1) Only map starting from the beginning
				 * of this range's phys. offset, so we don't map
				 * unmappable memory. 2) The length of the
				 * virtual memory to not map is the delta
				 * between the requested offset and the mappable
				 * start (and since the mappable start is
				 *	bigger, start - req.)
				 * 3) The map length is the minimum of the
				 * mappable requested length
				 *	(requested_length-mapping_offset) and
				 * the actual mappable length of the range.
				 */
				phys_offset = range_start;
				mapping_offset = range_start - requested_offset;
				map_length =
					min(requested_length - mapping_offset,
						range_length);
			} else if (requested_offset > range_start &&
				   requested_offset < range_end) {
				/*
				 * If the requested offset is within this range:
				 * 1) Map starting from the requested offset.
				 * 2) Because there is no forbidden memory
				 * between the requested offset and the range
				 * start, mapping_offset is 0. 3) The map length
				 * is the minimum of the requested length and
				 * the remaining length in the buffer
				 *	(range_end-requested_offset)
				 */
				phys_offset = requested_offset;
				mapping_offset = 0;
				map_length = min(requested_length,
					range_end - requested_offset);
			} else {
				/*
				 * If the requested offset is above this range,
				 * there is nothing to map.
				 */
				continue;
			}

			/* Keep track if we've mapped any memory at all. */
			has_mapped = 1;
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			ret = io_remap_pfn_range(vma,
				vma->vm_start + mapping_offset,
				(bar_data->phys_base + phys_offset) >>
					PAGE_SHIFT,
				map_length, vma->vm_page_prot);
			if (ret) {
				ds_log_error(ds_dev,
					"Error remapping PFN range err=%d.",
					ret);
				trace_ds_mmap_exit(ret);
				return ret;
			}
		}
	}

	/* If we could not map any memory, the request was invalid. */
	if (!has_mapped) {
		ds_log_error(
			ds_dev, "Map request did not contain a valid region.");
		trace_ds_mmap_exit(-EINVAL);
		return -EINVAL;
	}

	trace_ds_mmap_exit(0);
	return 0;
}

/*
 * ds_get_hw_status: Determine the health of the DS device.
 * @ds_dev: DS device structure.
 *
 * Checks the underlying device health (via the device_status_cb)
 * and the status of initialized ds code systems (currently
 * only interrupts), then returns a ds_status appropriately.
 */
static int ds_get_hw_status(struct ds_dev *ds_dev)
{
	int status;
	int i;
	struct ds_driver_desc *driver_desc = ds_dev->internal_desc->driver_desc;

	status = ds_check_and_invoke_callback_nolock(
		ds_dev, driver_desc->device_status_cb);
	if (status != DS_STATUS_ALIVE) {
		ds_log_info(ds_dev, "Hardware reported status %d.", status);
		return status;
	}

	status = ds_interrupt_system_status(ds_dev);
	if (status != DS_STATUS_ALIVE) {
		ds_log_info(
			ds_dev, "Interrupt system reported status %d.", status);
		return status;
	}

	for (i = 0; i < driver_desc->num_page_tables; ++i) {
		status = ds_page_table_system_status(ds_dev->page_table[i]);
		if (status != DS_STATUS_ALIVE) {
			ds_log_info(ds_dev, "Page table %d reported status %d.",
				i, status);
			return status;
		}
	}

	return DS_STATUS_ALIVE;
}

/*
 * ds_ioctl: DS ioctl dispatch function.
 * @filp: File structure pointer describing this node usage session.
 * @cmd: ioctl number to handle.
 * @arg: ioctl-specific data pointer.
 *
 * First, checks if the ioctl is a DS generic ioctl. If not, it passes
 * the ioctl to the ioctl_handler_cb registered in the driver description.
 * If the ioctl is a DS generic ioctl, the function passes it to the
 * ds_ioctl_handler in ds_ioctl.c.
 */
static long ds_ioctl(struct file *filp, uint cmd, ulong arg)
{
	struct ds_dev *ds_dev;
	struct ds_driver_desc *driver_desc;

	ds_dev = (struct ds_dev *)filp->private_data;
	driver_desc = ds_dev->internal_desc->driver_desc;

	if (!ds_is_supported_ioctl(cmd)) {
		/*
		 * The ioctl handler is not a standard ds callback, since
		 * it requires different arguments. This means we can't use
		 * ds_check_and_invoke_callback.
		 */
		if (driver_desc->ioctl_handler_cb)
			return driver_desc->ioctl_handler_cb(filp, cmd, arg);

		ds_log_error(ds_dev, "Received unknown ioctl 0x%x", cmd);
		return -EINVAL;
	}

	return ds_handle_ioctl(filp, cmd, arg);
}

int ds_reset(struct ds_dev *ds_dev, uint reset_type)
{
	int ret;

	mutex_lock(&ds_dev->mutex);
	ret = ds_reset_nolock(ds_dev, reset_type);
	mutex_unlock(&ds_dev->mutex);
	return ret;
}
EXPORT_SYMBOL(ds_reset);

int ds_reset_nolock(struct ds_dev *ds_dev, uint reset_type)
{
	int ret;
	int i;
	struct ds_driver_desc *driver_desc;

	driver_desc = ds_dev->internal_desc->driver_desc;
	if (!driver_desc->device_reset_cb) {
		ds_log_error(
			ds_dev, "No device reset callback was registered.");
		return -EINVAL;
	}

	/* Perform a device reset of the requested type. */
	ret = driver_desc->device_reset_cb(ds_dev, reset_type);
	if (ret)
		ds_log_error(ds_dev, "Device reset cb returned %d.", ret);

	/* Reinitialize the page tables and interrupt framework. */
	for (i = 0; i < driver_desc->num_page_tables; ++i)
		ds_page_table_reset(ds_dev->page_table[i]);

	ret = ds_interrupt_reinit(ds_dev);
	if (ret) {
		ds_log_error(ds_dev, "Unable to reinit interrupts: %d.", ret);
		return ret;
	}

	/* Get current device health. */
	ds_dev->status = ds_get_hw_status(ds_dev);
	if (ds_dev->status == DS_STATUS_DEAD) {
		ds_log_error(ds_dev, "Device reported as dead.");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(ds_reset_nolock);

static ssize_t ds_write_mappable_regions(
	char *buf, struct ds_driver_desc *driver_desc, int bar_index)
{
	int i;
	ssize_t written;
	ssize_t total_written = 0;
	ulong min_addr, max_addr;
	struct ds_bar_desc bar_desc = driver_desc->bar_descriptions[bar_index];

	if (bar_desc.permissions == DS_NOMAP)
		return 0;
	for (i = 0; (i < bar_desc.num_mappable_regions) &&
		    (total_written < PAGE_SIZE);
		i++) {
		min_addr = bar_desc.mappable_regions[i].start -
			   driver_desc->legacy_mmap_address_offset;
		max_addr = bar_desc.mappable_regions[i].start -
			   driver_desc->legacy_mmap_address_offset +
			   bar_desc.mappable_regions[i].length_bytes;
		written = scnprintf(buf, PAGE_SIZE - total_written,
			"0x%08lx-0x%08lx\n", min_addr, max_addr);
		total_written += written;
		buf += written;
	}
	return total_written;
}

static ssize_t ds_sysfs_data_show(
	struct device *device, struct device_attribute *attr, char *buf)
{
	int j;
	ssize_t total_written = 0;
	struct ds_driver_desc *driver_desc;
	struct ds_dev *ds_dev =
		(struct ds_dev *)ds_sysfs_get_device_data(device);
	enum ds_sysfs_attribute_type sysfs_type =
		(enum ds_sysfs_attribute_type)ds_sysfs_get_attr_data(
			device, attr);
	if (ds_dev == NULL) {
		ds_nodev_error(
			"No sysfs mapping found for pointer 0x%p", device);
		return 0;
	}

	driver_desc = ds_dev->internal_desc->driver_desc;

	switch (sysfs_type) {
	case ATTR_DRIVER_VERSION:
		return snprintf(buf, PAGE_SIZE, "%s\n",
			ds_dev->internal_desc->driver_desc->driver_version);
	case ATTR_FRAMEWORK_VERSION:
		return snprintf(buf, PAGE_SIZE, "%s\n", DS_FRAMEWORK_VERSION);
	case ATTR_DEVICE_TYPE:
		return snprintf(buf, PAGE_SIZE, "%s\n",
			ds_dev->internal_desc->driver_desc->name);
	case ATTR_HARDWARE_REVISION:
		return snprintf(
			buf, PAGE_SIZE, "%d\n", ds_dev->hardware_revision);
	case ATTR_PCI_ADDRESS:
		return snprintf(buf, PAGE_SIZE, "%s\n", ds_dev->kobj_name);
	case ATTR_STATUS:
		return snprintf(buf, PAGE_SIZE, "%s\n",
			ds_num_name_lookup(
				ds_dev->status, ds_status_name_table));
	case ATTR_IS_DEVICE_OWNED:
		return snprintf(buf, PAGE_SIZE, "%d\n",
			ds_dev->dev_info.ownership.is_owned);
	case ATTR_DEVICE_OWNER:
		return snprintf(buf, PAGE_SIZE, "%d\n",
			ds_dev->dev_info.ownership.owner);
	case ATTR_WRITE_OPEN_COUNT:
		return snprintf(buf, PAGE_SIZE, "%d\n",
			ds_dev->dev_info.ownership.write_open_count);
	case ATTR_RESET_COUNT:
		return snprintf(buf, PAGE_SIZE, "%d\n", ds_dev->reset_count);
	case ATTR_USER_MEM_RANGES:
		for (j = 0; j < DS_NUM_BARS; ++j) {
			total_written +=
				ds_write_mappable_regions(buf, driver_desc, j);
		}
		return total_written;
	default:
		ds_log_error(ds_dev, "Unknown attribute: %s", attr->attr.name);
		return 0;
	}
}

/* ds_get_driver_desc: Get the driver structure for a given ds_dev
 * @dev: pointer to ds_dev, implementing the requested driver.
 */
struct ds_driver_desc *ds_get_driver_desc(struct ds_dev *dev)
{
	return dev->internal_desc->driver_desc;
}

/* ds_get_device: Get the device structure for a given ds_dev
 * @dev: pointer to ds_dev, implementing the requested driver.
 */
struct device *ds_get_device(struct ds_dev *dev)
{
	if (dev->pci_dev)
		return &(dev->pci_dev->dev);
	else if (dev->platform_dev) {
		struct abc_device *abc_dev = dev->
			platform_dev->dev.platform_data;

		return &(abc_dev->pdev->dev);
	}
	return NULL;
}

/**
 * ds_wait_sync - Synchronously waits on device.
 * @ds_dev: Device struct.
 * @bar: Bar
 * @offset: Register offset
 * @mask: Register mask
 * @val: Expected value
 * @timeout_ns: Timeout in nanoseconds
 *
 * Description: Busy waits for a specific combination of bits to be set
 * on a DS register.
 **/
int ds_wait_sync(struct ds_dev *ds_dev, int bar, u64 offset, u64 mask, u64 val,
	u64 timeout_ns)
{
	u64 reg;
	struct timespec start_time, cur_time;
	u64 diff_nanosec;
	int count = 0;

	reg = ds_dev_read_64(ds_dev, bar, offset);
	start_time = current_kernel_time();
	while ((reg & mask) != val) {
		count++;
		cur_time = current_kernel_time();
		diff_nanosec = (u64)(cur_time.tv_sec - start_time.tv_sec) *
				       1000000000LL +
			       (u64)(cur_time.tv_nsec) -
			       (u64)(start_time.tv_nsec);
		if (diff_nanosec > timeout_ns) {
			ds_log_error(ds_dev,
			"dw_wait_sync timeout: reg %llx count %x dma %lld ns\n",
			offset, count, diff_nanosec);
			return -EBUSY;
		}
		reg = ds_dev_read_64(ds_dev, bar, offset);
	}
	return 0;
}
EXPORT_SYMBOL(ds_wait_sync);

/**
 * ds_wait_async - Asynchronously waits on device.
 * @ds_dev: Device struct.
 * @bar: Bar
 * @offset: Register offset
 * @mask: Register mask
 * @val: Expected value
 * @max_retries: number of sleep periods
 * @delay_ms: Timeout in milliseconds
 *
 * Description: Busy waits for a specific combination of bits to be set
 * on a DS register.
 **/
int ds_wait_async(struct ds_dev *ds_dev, int bar, u64 offset, u64 mask, u64 val,
	u64 max_retries, u64 delay_ms)
{
	u64 retries = 0;
	u64 tmp;

	while (retries < max_retries) {
		tmp = ds_dev_read_64(ds_dev, bar, offset);
		if ((tmp & mask) == val)
			break;
		schedule_timeout(msecs_to_jiffies(delay_ms));
		retries++;
	}
	if (retries == max_retries) {
		ds_log_error(ds_dev,
			"dw_wait_sync timeout: reg %llx timeout (%llu ms)",
			offset, max_retries * delay_ms);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(ds_wait_async);
