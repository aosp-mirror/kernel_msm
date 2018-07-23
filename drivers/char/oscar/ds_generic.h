/* DS generic driver. Defines the set of data types and functions necessary
 * to define a driver using the DS generic driver framework.
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

#ifndef __DS_GENERIC_H__
#define __DS_GENERIC_H__

#ifdef OSCAR_AIRBRUSH
#define PLATFORM_INTERFACE
#endif

#include <linux/cdev.h>
#include <linux/compiler.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/slab.h>
#ifndef PLATFORM_INTERFACE
#include <asm/dma-iommu.h>
#endif
#include <linux/mfd/abc-pcie.h>

#include "ds_constants.h"

/**
 * struct ds_num_name - Map numbers to names.
 * @ein_num: Number.
 * @ein_name: Name associated with the number, a char pointer.
 *
 * This structure maps numbers to names. It is used to provide printable enum
 * names, e.g {0, "DEAD"} or {1, "ALIVE"}.
 */
struct ds_num_name {
	uint snn_num;
	const char *snn_name;
};

/* Description of the packing attribute for an interrupt struct */
enum ds_interrupt_packing {
	PACK_0 = 0,
	PACK_1 = 1,
	PACK_2 = 2,
	PACK_3 = 3,
	UNPACKED = 4,
};

/* Type of the interrupt supported by the device. */
enum ds_interrupt_type {
	PCI_MSIX = 0,
	PCI_MSI = 1,
	PLATFORM_WIRE = 2,
};

/* Used to describe a DS interrupt. Contains an interrupt index, a register,
 * and packing data for that interrupt. The register and packing data
 * fields is relevant only for PCI_MSIX interrupt type and can be
 * set to 0 for everything else.
 */
struct ds_interrupt_desc {
	int index;
	u64 reg;
	int packing;
};

/* struct ds_wire_interrupt_offsets : offsets to the wire interrupt
 * handling registers
 */
struct ds_wire_interrupt_offsets {
	u64 pending_bit_array;
	u64 mask_array;
};

/* This enum is used to identify memory regions being part of the phys
 * memory that belongs to a device.
 */
enum mappable_area_type {
	PCI_BAR = 0, /* Default */
	BUS_REGION,  /* For SYSBUS devices, i.e. AXI etc... */
	COHERENT_MEMORY
};

/* struct ds_bar_data: Metadata for each device specific memory mapping. *
 * This struct is used so as to track PCI memory, I/O space, AXI and coherent
 * memory area... i.e. memory objects which can be referenced in the device's
 * mmap function.
 */
struct ds_bar_data {
	/* Virtual base address. */
	u8 __iomem *virt_base;

	/* Physical base address. */
	ulong phys_base;

	/* Length of the mapping. */
	ulong length_bytes;

	/* Type of mappable area */
	enum mappable_area_type type;
};

/* struct ds_ownership: Maintains device open ownership data. */
struct ds_ownership {
	/* 1 if the device is owned, 0 otherwise. */
	int is_owned;

	/* TGID of the owner. */
	pid_t owner;

	/* Count of the number of writable opens by the owner. */
	int write_open_count;
};

/* struct ds_page_table_offsets: Offsets to page table registers. One per PT. */
struct ds_page_table_offsets {
	/* Register containing the length of this page table. */
	int page_table_size_reg;

	/* Register containing the simple page table. */
	int page_table_reg;

	/* Register containing the extended page table. */
	int extended_page_table_reg;
};

/* struct ds_cdev_info: Maintains information about a device node. */
struct ds_cdev_info {
	/* The internal name of this device. */
	char name[DS_NAME_MAX];

	/* Device number. */
	dev_t devt;

	/* Kernel-internal device structure. */
	struct device *device;

	/* Character device for real. */
	struct cdev cdev;

	/* Flag indicating if cdev_add has been called for the devices. */
	int cdev_added;

	/* Pointer to pointer to the overall ds_dev struct for this device. */
	struct ds_dev *ds_dev_ptr;

	/* Ownership data for the device in question. */
	struct ds_ownership ownership;
};

/*
 * struct ds_mappable_region: Describes the offset and length of mmapable
 * regions in the device's BAR space.
 *
 */
struct ds_mappable_region {
	u64 start;
	u64 length_bytes;
};

/*
 * struct ds_bar_desc: Describe the offset, size, and permissions for a
 * mappable device bar.
 */
struct ds_bar_desc {
	/*
	 * The size of each PCI BAR range, in bytes. If a value is 0, that BAR
	 * will not be mapped into kernel space at all.
	 * For devices with 64 bit BARs, only elements 0, 2, and 4 should be
	 * populated, and 1, 3, and 5 should be set to 0.
	 * For example, for a device mapping 1M in each of the first two 64-bit
	 * BARs, this field would be set as { 0x100000, 0, 0x100000, 0, 0, 0 }
	 * (one number per bar_desc struct.)
	 */
	u64 size;
	/* The permissions for this bar. (Should be VM_WRITE/VM_READ/VM_EXEC,
	 * and can be or'd.) If set to DS_NOMAP, the bar will
	 * not be used for mmaping.
	 */
	ulong permissions;
	/* The memory address corresponding to the base of this bar, if used. */
	u64 base;
	/* The number of mappable regions in this bar. */
	int num_mappable_regions;

	/* The mappable subregions of this bar. */
	const struct ds_mappable_region *mappable_regions;

	/* Type of mappable area */
	enum mappable_area_type type;
};

struct ds_coherent_buffer_desc {
	/*
	 * The size of coherent Buffer
	 */
	u64 size;
	/* The permissions for this bar. (Should be VM_WRITE/VM_READ/VM_EXEC,
	 * and can be or'd.) If set to DS_NOMAP, the bar will
	 * not be used for mmaping.
	 */
	ulong permissions;

	/* device side address. */
	u64 base;
};


struct ds_coherent_buffer {
	u64 base;

	/* Virtual base address. */
	u8 __iomem *virt_base;

	/* Physical base address. */
	ulong phys_base;

	/* Length of the mapping. */
	ulong length_bytes;
};


/* Description of DS-specific permissions in the mmap field. */
enum ds_mapping_options { DS_NOMAP = 0 };

/* This struct represents an undefined bar that should never be mapped. */
#define DS_UNUSED_BAR                                                          \
	{                                                                      \
		0, DS_NOMAP, 0, 0, NULL, 0                                     \
	}

/*
 * struct ds_internal_desc: Internal data for a DS device. See ds_generic.c
 * for more information.
 *
 */
struct ds_internal_desc;

#define MAX_NUM_COHERENT_PAGES 16

/*
 * struct ds_dev: Maintains device data for DS devices.
 *
 * This structure contains all the data required to handle a DS
 * device.
 *
 */
struct ds_dev {
	/* Pointer to the internal driver description for this device. */
	struct ds_internal_desc *internal_desc;

	/* The pointer to either pci_dev or platform_dev is the only
	 * field that identifies the ds_dev as either PCI or SYSBUS
	 * One of those two pointer must be NULL.
	 * It is used only by log functions.
	 *
	 * TODO(vandwalle): build a better abstraction.
	 */

	/* PCI subsystem metadata. */
	struct pci_dev *pci_dev;

	/* Platform driver metadata. */
	struct platform_device *platform_dev;

	/* This device's index into internal_desc->devs. */
	int dev_idx;

	/* The name of this device, as reported by the kernel. */
	char kobj_name[DS_NAME_MAX];

	/* Virtual address of mapped BAR memory range. */
	struct ds_bar_data bar_data[DS_NUM_BARS];

	/* Coherent Buffer */
	struct ds_coherent_buffer coherent_buffer;

	/* Number of page tables for this device. */
	int num_page_tables;

	/* Address translations. Page tables have a private implementation. */
	struct ds_page_table *page_table[DS_MAX_NUM_PAGE_TABLES];

	/* Interrupt data for this device. */
	struct ds_interrupt_data *interrupt_data;

	/* Status for this device - DS_STATUS_ALIVE or _DEAD. */
	uint status;

	/* Number of times this device has been reset. */
	uint reset_count;

	/* Dev information for the cdev node. */
	struct ds_cdev_info dev_info;

	/* Hardware revision value for this device. */
	int hardware_revision;

	/*
	 * Device-specific data; allocated in ds_driver_desc.add_dev_cb() and
	 * freed in ds_driver_desc.remove_dev_cb().
	 */
	void *cb_data;

	/* Protects access to per-device data (i.e. this structure). */
	struct mutex mutex;

	/* DMA IOMMU/SMMU Mapping for oscar
	 */
	struct dma_iommu_mapping *dma_mapping;
};
/*
 * struct ds_driver_desc: Device type descriptor.
 *
 * This structure contains device-specific data needed to identify and address a
 * type of device to be administered via the DS generic driver.
 *
 * Device IDs are per-driver. In other words, two drivers using the DS
 * framework will each have a distinct device 0 (for example).
 */
struct ds_driver_desc {
	/* The name of this device class. */
	const char *name;

	/* The version of this driver: "1.0.0", "2.1.3", etc. */
	const char *driver_version;

	/* Major and minor numbers identifying the device. */
	int major, minor;

	/* Module structure for this driver. */
	struct module *module;

	/* PCI ID table, NULL for non PCI devices. */
	const struct pci_device_id *pci_id_table;

	/* The number of page tables handled by this driver. */
	int num_page_tables;

	/* The index of the bar containing the page tables. */
	int page_table_bar_index;

	/* Registers used to control each page table. */
	const struct ds_page_table_offsets *page_table_offsets;

	/* The bit index indicating whether a PT entry is extended. */
	int page_table_extended_bit;

	/*
	 * Legacy mmap address adjusment. Should be zero for all DS devices.
	 */
	ulong legacy_mmap_address_offset;

	/* Set of 6 bar descriptions that describe all PCIe bars.
	 * Note that BUS/AXI devices (i.e. non PCI devices) use those.
	 */
	struct ds_bar_desc bar_descriptions[DS_NUM_BARS];

	/* Coherent buffer description
	 */
	struct ds_coherent_buffer_desc coherent_buffer_description;

	/* Offset of wire interrupt registers. */
	const struct ds_wire_interrupt_offsets *wire_interrupt_offsets;

	/* Interrupt type. (One of ds_interrupt_type). */
	int interrupt_type;

	/* Index of the bar containing the interrupt registers to program. */
	int interrupt_bar_index;

	/* Number of interrupts in the ds_interrupt_desc array */
	int num_interrupts;

	/* Description of the interrupts for this device. */
	const struct ds_interrupt_desc *interrupts;

	/* Bit width of a single interrupt in a packed vector. */
	int interrupt_pack_width;

	/* Driver callback functions - all may be NULL */
	/*
	 * add_dev_cb: Callback when a device is found.
	 * @dev: The ds_dev struct for this driver instance.
	 *
	 * This callback should initialize the device-specific cb_data.
	 * Called when a device is found by the driver,
	 * before any BAR ranges have been mapped. If this call fails (returns
	 * nonzero), remove_dev_cb will be called.
	 *
	 */
	int (*add_dev_cb)(struct ds_dev *dev);

	/*
	 * remove_dev_cb: Callback for when a device is removed from the system.
	 * @dev: The ds_dev struct for this driver instance.
	 *
	 * This callback should free data allocated in add_dev_cb.
	 * Called immediately before a device is unregistered by the driver.
	 * All framework-managed resources will have been cleaned up by the time
	 * this callback is invoked (PCI BARs, character devices, ...).
	 */
	int (*remove_dev_cb)(struct ds_dev *dev);

	/*
	 * device_open_cb: Callback for when a device node is opened for
	 * the first time.
	 * @dev: The ds_dev struct for this driver instance.
	 *
	 * This callback should perform device-specific setup that needs to
	 * occur only once when a device is first opened.
	 */
	int (*device_open_cb)(struct ds_dev *dev);

	/*
	 * device_close_cb: Callback for when a device node is closed for the
	 * last time,
	 * @dev: The ds_dev struct for this driver instance.
	 *
	 * This callback should perform device-specific cleanup that only
	 * needs to occur when the last reference to a device node is closed.
	 */
	int (*device_close_cb)(struct ds_dev *dev);

	/*
	 * enable_dev_cb: Callback immediately before enabling the device.
	 * @dev: Pointer to the ds_dev struct for this driver instance.
	 *
	 * This callback is invoked after the device has been added and all BAR
	 * spaces mapped, immediately before registering and enabling the
	 * [character] device via cdev_add. If this call fails (returns
	 * nonzero), disable_dev_cb will be called.
	 *
	 * Note that cdev are initialized but not active
	 * (cdev_add has not yet been called) when this callback is invoked.
	 */
	int (*enable_dev_cb)(struct ds_dev *dev);

	/*
	 * disable_dev_cb: Callback immediately after disabling the device.
	 * @dev: Pointer to the ds_dev struct for this driver instance.
	 *
	 * Called during device shutdown, immediately after disabling device
	 * operations via cdev_del.
	 */
	int (*disable_dev_cb)(struct ds_dev *dev);

	/*
	 * sysfs_setup_cb: Callback to set up driver-specific sysfs nodes.
	 * @dev: Pointer to the ds_dev struct for this device.
	 *
	 * Called just before enable_dev_cb.
	 *
	 */
	int (*sysfs_setup_cb)(struct ds_dev *dev);

	/*
	 * sysfs_cleanup_cb: Callback to clean up driver-specific sysfs nodes.
	 * @dev: Pointer to the ds_dev struct for this device.
	 *
	 * Called just before disable_dev_cb.
	 *
	 */
	int (*sysfs_cleanup_cb)(struct ds_dev *dev);

	/*
	 * ioctl_handler_cb: Callback to handle device-specific ioctls.
	 * @filp: File structure pointer describing this node usage session.
	 * @cmd: ioctl number to handle.
	 * @arg: ioctl-specific data pointer.
	 *
	 * Invoked whenever an ioctl is called that the generic ds
	 * framework doesn't support. If no cb is registered, unknown ioctls
	 * return -EINVAL. Should return an error status (either -EINVAL or
	 * the error result of the ioctl being handled).
	 *
	 */
	long (*ioctl_handler_cb)(struct file *filp, uint cmd, ulong arg);

	/*
	 * device_status_cb: Callback to determine device health.
	 * @dev: Pointer to the ds_dev struct for this device.
	 *
	 * Called to determine if the device is healthy or not. Should return
	 * a member of the ds_status_type enum.
	 *
	 */
	int (*device_status_cb)(struct ds_dev *dev);

	/*
	 * hardware_revision_cb: Get the device's hardware revision.
	 * @dev: Pointer to the ds_dev struct for this device.
	 *
	 * Called to determine the reported rev of the physical hardware.
	 * Revision should be >0. A negative return value is an error.
	 */
	int (*hardware_revision_cb)(struct ds_dev *dev);

	/*
	 * device_reset_cb: Reset the hardware in question.
	 * @dev: Pointer to the ds_dev structure for this device.
	 * @type: Integer representing reset type. (All
	 * ds resets have an integer representing their type
	 * defined in (device)_ioctl.h; the specific resets are
	 * device-dependent, but are handled in the device-specific
	 * callback anyways.)
	 *
	 * Called by reset ioctls. This function should not
	 * lock the ds_dev mutex. It should return 0 on success
	 * and an error on failure.
	 */
	int (*device_reset_cb)(struct ds_dev *dev, uint reset_type);
};

/*
 * ds_register_device: Register the specified device type with the framework.
 * @desc: Populated/initialized device type descriptor.
 *
 * This function does _not_ take ownership of desc; the underlying struct must
 * exist until the matching call to ds_unregister_device.
 * This function should be called from your driver's module_init function.
 */
int ds_register_device(struct ds_driver_desc *desc);

/*
 * ds_unregister_device: Remove the specified device type from the framework.
 * @desc: Descriptor for the device type to unregister; it should have been
 *        passed to ds_register_device in a previous call.
 *
 * This function should be called from your driver's module_exit function.
 */
void ds_unregister_device(struct ds_driver_desc *desc);

/*
 * ds_reset(_nolock): Reset the DS device.
 * @ds_dev: DS device struct.
 * @reset_type: Uint representing requested reset type. Should be
 * valid in the underlying callback.
 *
 * Calls device_reset_cb. Returns 0 on success and an error code othewrise.
 * ds_reset_nolock will not lock the mutex, ds_reset will.
 *
 */
int ds_reset(struct ds_dev *ds_dev, uint reset_type);
int ds_reset_nolock(struct ds_dev *ds_dev, uint reset_type);

/*
 * ds_num_name_lookup - Lookup a name by number in a num_name table.
 * @num: Number to lookup.
 * @table: Array of num_name structures, the table for the lookup.
 *
 */
const char *ds_num_name_lookup(uint num, const struct ds_num_name *table);

static inline ulong ds_dev_read_64(struct ds_dev *dev, int bar, ulong location)
{
	/*TODO(momernick): Add endianness support here. */
#ifdef OSCAR_AIRBRUSH
	u32 temp;

	tpu_config_read(location, 0, &temp);
	return (ulong)temp;
#else
	return readq(&dev->bar_data[bar].virt_base[location]);
#endif
}


static inline void ds_dev_write_64(
	struct ds_dev *dev, u64 value, int bar, ulong location)
{
#ifdef OSCAR_AIRBRUSH
	tpu_config_write(location, 0, (u32)value);
#else
	writeq(value, &dev->bar_data[bar].virt_base[location]);
#endif
}

static inline void ds_dev_write_32(
	struct ds_dev *dev, u32 value, int bar, ulong location)
{
#ifdef OSCAR_AIRBRUSH
	tpu_config_write(location, 0, value);
#else
	writel_relaxed(value, &dev->bar_data[bar].virt_base[location]);
#endif
}

static inline u32 ds_dev_read_32(struct ds_dev *dev, int bar, ulong location)
{
	u32 temp;

#ifdef OSCAR_AIRBRUSH
	tpu_config_read(location, 0, &temp);
	return temp;
#else
	return readl_relaxed(&dev->bar_data[bar].virt_base[location]);
#endif
}

static inline void ds_read_modify_write_64(struct ds_dev *dev, int bar,
	ulong location, u64 value, u64 mask_width, u64 mask_shift)
{
	u64 mask, tmp;

	tmp = ds_dev_read_64(dev, bar, location);
	mask = ((1 << mask_width) - 1) << mask_shift;
	tmp = (tmp & ~mask) | (value << mask_shift);
	ds_dev_write_64(dev, tmp, bar, location);
}

static inline void ds_read_modify_write_32(struct ds_dev *dev, int bar,
	ulong location, u32 value, u32 mask_width, u32 mask_shift)
{
	u32 mask, tmp;

	tmp = ds_dev_read_32(dev, bar, location);
	mask = ((1 << mask_width) - 1) << mask_shift;
	tmp = (tmp & ~mask) | (value << mask_shift);
	ds_dev_write_32(dev, tmp, bar, location);
}

/* Get the DS driver structure for a given device. */
extern struct ds_driver_desc *ds_get_driver_desc(struct ds_dev *dev);

/* Get the device structure for a given device. */
extern struct device *ds_get_device(struct ds_dev *dev);

/* Helper function, Synchronous waits on a given set of bits. */
extern int ds_wait_sync(struct ds_dev *ds_dev, int bar, u64 offset, u64 mask,
	u64 val, u64 timeout_ns);

/* Helper function, Asynchronous waits on a given set of bits. */
extern int ds_wait_async(struct ds_dev *ds_dev, int bar, u64 offset, u64 mask,
	u64 val, u64 max_retries, u64 delay_ms);
#endif /* __DS_GENERIC_H__ */
