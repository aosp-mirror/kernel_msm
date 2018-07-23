/* Driver for the Oscar chip.
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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "../../mfd/abc-pcie-private.h"
#include "linux/mfd/abc-pcie.h"
#include "dw_ioctl.h"

#include "ds_generic.h"
#include "ds_interrupt.h"
#include "ds_logging.h"
#include "ds_page_table.h"
#include "ds_sysfs.h"

/* Constants */
#define DW_DEVICE_NAME "Oscar"
#define DW_DRIVER_VERSION "0.1"

#define DW_PCI_VENDOR_ID 0x1556
#define DW_PCI_DEVICE_ID 0x1111

/* Oscar is on BAR 4 and 5. */
#define DW_BAR_BYTES 0x100000

/* Number of bytes allocated for coherent memory. */
#define DW_CH_MEM_BYTES (PAGE_SIZE * MAX_NUM_COHERENT_PAGES)

/* Oscar uses BAR 4/5. */
#define DW_BAR_INDEX 0

/* The number of user-mappable memory ranges in Oscar BAR. */
#define NUM_BAR_RANGES 3

/* Bar Offsets. */
#define DW_BAR_OFFSET 0
#define DW_CM_OFFSET 0x1000000

/* enum sysfs_attribute_type: enumeration of the supported sysfs entries. */
enum sysfs_attribute_type {
	ATTR_KERNEL_HIB_PAGE_TABLE_SIZE,
	ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE,
	ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES,
};

/*
 * Register offsets into BAR 4/5 memory.
 * Only values necessary for driver implementation are defined.
 */
enum dw_bar_regs {
	DW_BAR_REG_HIB_PAGE_TABLE_SIZE = 0x6000,
	DW_BAR_REG_KERNEL_HIB_EXTENDED_TABLE = 0x6008,
	DW_BAR_REG_KERNEL_HIB_TRANSLATION_ENABLE = 0x6010,
	DW_BAR_REG_KERNEL_HIB_DMA_PAUSE = 0x46050,
	DW_BAR_REG_KERNEL_HIB_DMA_PAUSE_MASK = 0x6058,
	DW_BAR_REG_HIB_PAGE_TABLE_INIT = 0x6078,
	DW_BAR_REG_USER_HIB_DMA_PAUSE = 0x86D8,
	DW_BAR_REG_USER_HIB_DMA_PAUSED = 0x86E0,
	DW_BAR_REG_HIB_PAGE_TABLE = 0x10000,

	/* Top Level Registers. */
	DW_BAR_REG_AON_RESET = 0x20000,
	DW_BAR_REG_AON_CLOCK_ENABLE = 0x20008,
	DW_BAR_REG_AON_LOGIC_SHUTDOWN_PRE = 0x00020010,
	DW_BAR_REG_AON_LOGIC_SHUTDOWN_ALL = 0x00020018,
	DW_BAR_REG_AON_MEM_SHUTDOWN = 0x00020020,
	DW_BAR_REG_AON_MEM_POWERDOWN = 0x00020028,
	DW_BAR_REG_AON_CLAMP_ENABLE = 0x00020038,
	DW_BAR_REG_AON_FORCE_QUIESCE = 0x00020040,
	DW_BAR_REG_AON_IDLE = 0x00020050,
};

/* For now map the entire BAR2 into user space. (This helps debugging when
 * running test vectors from user land)
 * In production driver we want to exclude the kernel HIB.
 */
static struct ds_page_table_offsets dw_page_table_offsets[] = {
	{ DW_BAR_REG_HIB_PAGE_TABLE_SIZE, DW_BAR_REG_HIB_PAGE_TABLE,
		DW_BAR_REG_KERNEL_HIB_EXTENDED_TABLE },
};

/* Function declarations */

static int __init dw_init(void);
static void dw_exit(void);

static int dw_add_dev_cb(struct ds_dev *ds_dev);
static int dw_remove_dev_cb(struct ds_dev *ds_dev);

static int dw_sysfs_setup_cb(struct ds_dev *ds_dev);

static int dw_device_cleanup(struct ds_dev *ds_dev);

static int dw_device_open_cb(struct ds_dev *ds_dev);

static ssize_t sysfs_show(
	struct device *device, struct device_attribute *attr, char *buf);

static int dw_reset(struct ds_dev *ds_dev, uint type);

static int dw_get_status(struct ds_dev *ds_dev);

static uint dw_ioctl_check_permissions(struct file *file, uint cmd);

static long dw_ioctl(struct file *file, uint cmd, ulong arg);

static long dw_clock_gating(struct ds_dev *ds_dev, ulong arg);

static int dw_enter_reset(struct ds_dev *ds_dev, uint type);
static int dw_quit_reset(struct ds_dev *ds_dev, uint type);

/* Data definitions */

/* The data necessary to display this file's sysfs entries. */
static struct ds_sysfs_attribute dw_sysfs_attrs[] = {
	DS_SYSFS_RO(node_0_page_table_entries, sysfs_show,
		ATTR_KERNEL_HIB_PAGE_TABLE_SIZE),
	DS_SYSFS_RO(node_0_simple_page_table_entries, sysfs_show,
		ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE),
	DS_SYSFS_RO(node_0_num_mapped_pages, sysfs_show,
		ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES),
	DS_END_OF_ATTR_ARRAY
};

static const struct pci_device_id dw_pci_ids[] = {
	{ PCI_DEVICE(DW_PCI_VENDOR_ID, DW_PCI_DEVICE_ID) },
	{ 0 }
};

/* The regions in the BAR5 space that can be mapped into user space. */
static const struct ds_mappable_region tn_mappable_regions[NUM_BAR_RANGES] = {
	{ 0x0000, 0x1000 },
	{ 0x4000, 0x1000 },
	{ 0x8000, 0x1000 },
};

static const struct ds_mappable_region cm_mappable_regions[1] = {
	{ 0x00000, DW_CH_MEM_BYTES }
};

/* Interrupt descriptors for DW. */
static struct ds_interrupt_desc dw_interrupts[] = {
	{ABC_MSI_4_TPU_IRQ0, },
	{ABC_MSI_5_TPU_IRQ1, },
	{ABC_MSI_AON_INTNC, },
};

static struct ds_driver_desc dw_desc = {
	.name = DRV_NAME_ABC_PCIE_TPU,
	.driver_version = DW_DRIVER_VERSION,
	.major = 120,
	.minor = 0,
	.module = THIS_MODULE,
	.pci_id_table = NULL,

	.num_page_tables = 1,
	.page_table_bar_index = DW_BAR_INDEX,
	.page_table_offsets = dw_page_table_offsets,
	.page_table_extended_bit = DW_EXTENDED_SHIFT,

	.bar_descriptions = {
		{
			DW_BAR_BYTES,
			(VM_WRITE | VM_READ),
			DW_BAR_OFFSET,
			NUM_BAR_RANGES,
			tn_mappable_regions,
			PCI_BAR
		},
		DS_UNUSED_BAR,
		DS_UNUSED_BAR,
		DS_UNUSED_BAR,
		DS_UNUSED_BAR,
		DS_UNUSED_BAR,
	},
	.coherent_buffer_description = {
		DW_CH_MEM_BYTES,
		(VM_WRITE | VM_READ),
		DW_CM_OFFSET,
	},
	.interrupt_type = PLATFORM_WIRE,
	.interrupt_bar_index = DW_BAR_INDEX,
	.num_interrupts = 3,
	.interrupts = dw_interrupts,

	.add_dev_cb = dw_add_dev_cb,
	.remove_dev_cb = dw_remove_dev_cb,

	.enable_dev_cb = NULL,
	.disable_dev_cb = NULL,

	.sysfs_setup_cb = dw_sysfs_setup_cb,
	.sysfs_cleanup_cb = NULL,

	.device_open_cb = dw_device_open_cb,
	.device_close_cb = dw_device_cleanup,

	.ioctl_handler_cb = dw_ioctl,
	.device_status_cb = dw_get_status,
	.hardware_revision_cb = NULL,
	.device_reset_cb = dw_reset,
};

/* Module registration boilerplate */
MODULE_DESCRIPTION("Google Oscar driver");
MODULE_VERSION(DW_DRIVER_VERSION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Joseph <jnjoseph@google.com>");
MODULE_DEVICE_TABLE(pci, dw_pci_ids);
module_init(dw_init);
module_exit(dw_exit);

/* Act as if only GCB is instantiated. */
static int bypass_top_level;

module_param(bypass_top_level, int, 0644);

static int __init dw_init(void)
{
	return ds_register_device(&dw_desc);
}

static void dw_exit(void)
{
	ds_unregister_device(&dw_desc);
}

static int dw_add_dev_cb(struct ds_dev *ds_dev)
{
	ulong page_table_ready, msix_table_ready;
	int retries = 0;


	dw_reset(ds_dev, 0);

	while (retries < DW_RESET_RETRY) {
		page_table_ready = ds_dev_read_64(
			ds_dev, DW_BAR_INDEX, DW_BAR_REG_HIB_PAGE_TABLE_INIT);
		/* TODO(jnjoseph): Update when interrupts are enabled. */
		msix_table_ready = 1;
		if (page_table_ready && msix_table_ready)
			break;
		schedule_timeout(msecs_to_jiffies(DW_RESET_DELAY));
		retries++;
	}

	if (retries == DW_RESET_RETRY) {
		if (!page_table_ready)
			ds_log_error(ds_dev, "Page table init timed out.");
		if (!msix_table_ready)
			ds_log_error(ds_dev, "MSI-X table init timed out.");
		return -ETIMEDOUT;
	}

	return 0;
}

static int dw_remove_dev_cb(struct ds_dev *ds_dev)
{
	return 0;
}

static int dw_sysfs_setup_cb(struct ds_dev *ds_dev)
{
	return ds_sysfs_create_entries(ds_dev->dev_info.device, dw_sysfs_attrs);
}

/* On device open, we want to perform a core reinit reset. */
static int dw_device_open_cb(struct ds_dev *ds_dev)
{
	return ds_reset_nolock(ds_dev, DW_CHIP_REINIT_RESET);
}

/**
 * dw_get_status - Set device status.
 * @dev: Oscar device struct.
 *
 * Description: Check the device status registers and set the driver status
 *		to ALIVE or DEAD.
 *
 *		Returns 0 if status is ALIVE, a negative error number otherwise.
 */
static int dw_get_status(struct ds_dev *ds_dev)
{

	/* Oscar, always returns ALIVE for now */
	return DS_STATUS_ALIVE;
}

/**
 * dw_device_cleanup - Clean up Oscar HW after close.
 * @ds_dev: DS device pointer.
 *
 * Description: Resets the Oscar hardware. Called on final close via
 * device_close_cb.
 */
static int dw_device_cleanup(struct ds_dev *ds_dev)
{
	return dw_enter_reset(ds_dev, DW_CHIP_REINIT_RESET);
}

/**
 * dw_reset - Quits reset.
 * @ds_dev: DS device pointer.
 *
 * Description: Resets the hardware, then quits reset.
 * Called on device open.
 *
 */
static int dw_reset(struct ds_dev *ds_dev, uint type)
{
	int ret = 0;

	if (bypass_top_level)
		return 0;

	ds_log_error(ds_dev, "dw_reset.\n");

	ret = dw_enter_reset(ds_dev, type);
	if (ret < 0)
		return ret;
	return dw_quit_reset(ds_dev, type);
}

/*
 * Enters GCB reset state.
 */
static int dw_enter_reset(struct ds_dev *ds_dev, uint type)
{
	if (bypass_top_level)
		return 0;

	ds_log_debug(ds_dev, "dw_enter_reset.");

	/* 1. Check whether we are already in reset to guard HIB access. */
	if (ds_dev_read_64(ds_dev, DW_BAR_INDEX, DW_BAR_REG_AON_RESET) == 0) {
		/* 1a. Enable DMA Pause. */
		ds_dev_write_64(
			ds_dev, 1, DW_BAR_INDEX, DW_BAR_REG_USER_HIB_DMA_PAUSE);
		/* 1b. Wait for DMA Pause to complete. */
		if (ds_wait_async(ds_dev, DW_BAR_INDEX,
			    DW_BAR_REG_USER_HIB_DMA_PAUSED, 1, 1,
			    DW_RESET_DELAY, DW_RESET_RETRY)) {
			ds_log_error(ds_dev,
				"DMA pause failed after timeout (%d ms)",
				DW_RESET_RETRY * DW_RESET_DELAY);
			return -EINVAL;
		}
	}

	/* 2. Enable Quiesce. */
	ds_dev_write_64(ds_dev, 1, DW_BAR_INDEX, DW_BAR_REG_AON_FORCE_QUIESCE);

	/* 3. Enable Reset. */
	ds_dev_write_64(ds_dev, 1, DW_BAR_INDEX, DW_BAR_REG_AON_RESET);

	/* 4. Disable Clock Enable.
	 *  - clock_enable = 0.
	 *  - cb_idle_override = 1.
	 */
	ds_dev_write_64(ds_dev, 2, DW_BAR_INDEX, DW_BAR_REG_AON_CLOCK_ENABLE);

	/* 5. Enable Clamp. */
	ds_dev_write_64(
		ds_dev, 0x1ffff, DW_BAR_INDEX, DW_BAR_REG_AON_CLAMP_ENABLE);

	/* 6. Enable Memory shutdown. */
	ds_dev_write_64(
		ds_dev, 0x1ffff, DW_BAR_INDEX, DW_BAR_REG_AON_MEM_SHUTDOWN);
	ds_dev_write_64(
		ds_dev, 0x1ffff, DW_BAR_INDEX, DW_BAR_REG_AON_MEM_POWERDOWN);

	/* 7. Enable Logic shutdown. */
	ds_dev_write_64(ds_dev, 0x1ffff, DW_BAR_INDEX,
		DW_BAR_REG_AON_LOGIC_SHUTDOWN_ALL);
	ds_dev_write_64(ds_dev, 0x1ffff, DW_BAR_INDEX,
		DW_BAR_REG_AON_LOGIC_SHUTDOWN_PRE);
	return 0;
}

/*
 * Quits GCB reset state.
 */
static int dw_quit_reset(struct ds_dev *ds_dev, uint type)
{
	if (bypass_top_level)
		return 0;

	ds_log_debug(ds_dev, "dw_quit_reset.");

	/* 1. Enable Logic shutdown. */
	ds_dev_write_64(
		ds_dev, 0, DW_BAR_INDEX, DW_BAR_REG_AON_LOGIC_SHUTDOWN_PRE);
	ds_dev_write_64(
		ds_dev, 0, DW_BAR_INDEX, DW_BAR_REG_AON_LOGIC_SHUTDOWN_ALL);

	/* 2. Enable Clock Enable, and set idle_override to force the clock on.
	 * - clock_enable = 1.
	 *  - cb_idle_override = 1.
	 */
	ds_dev_write_64(ds_dev, 3, DW_BAR_INDEX, DW_BAR_REG_AON_CLOCK_ENABLE);

	/* 3. Disable Clock Enable.
	 *  - clock_enable = 0.
	 *  - cb_idle_override = 1.
	 */
	ds_dev_write_64(ds_dev, 2, DW_BAR_INDEX, DW_BAR_REG_AON_CLOCK_ENABLE);

	/* 4. Disable Memory shutdown. */
	ds_dev_write_64(ds_dev, 0, DW_BAR_INDEX, DW_BAR_REG_AON_MEM_SHUTDOWN);
	ds_dev_write_64(ds_dev, 0, DW_BAR_INDEX, DW_BAR_REG_AON_MEM_POWERDOWN);

	/* 5. Enable Clock Enable, with dynamic activity based clock gating.
	 *  - clock_enable = 1.
	 *  - cb_idle_override = 0.
	 */
	ds_dev_write_64(ds_dev, 3, DW_BAR_INDEX, DW_BAR_REG_AON_CLOCK_ENABLE);

	/* 6. Disable Clamp. */
	ds_dev_write_64(ds_dev, 0, DW_BAR_INDEX, DW_BAR_REG_AON_CLAMP_ENABLE);

	/* 7. Disable Reset. */
	ds_dev_write_64(ds_dev, 0, DW_BAR_INDEX, DW_BAR_REG_AON_RESET);

	/* 8. Disable Quiesce. */
	ds_dev_write_64(ds_dev, 0, DW_BAR_INDEX, DW_BAR_REG_AON_FORCE_QUIESCE);

	return 0;
}

/*
 * DW_ioctl_check_permissions: Check permissions for Oscar ioctls.
 * @file: File pointer from ioctl.
 * @cmd: ioctl command.
 *
 * Returns 1 if the current user may execute this ioctl, and 0 otherwise.
 */
static uint dw_ioctl_check_permissions(struct file *filp, uint cmd)
{
	struct ds_dev *ds_dev = filp->private_data;
	int root = capable(CAP_SYS_ADMIN);
	int is_owner = ds_dev->dev_info.ownership.is_owned &&
		       current->tgid == ds_dev->dev_info.ownership.owner;

	if (root || is_owner)
		return 1;
	return 0;
}

/*
 * dw_ioctl: Oscar-specific ioctl handler.
 */
static long dw_ioctl(struct file *filp, uint cmd, ulong arg)
{
	struct ds_dev *ds_dev = filp->private_data;

	if (!dw_ioctl_check_permissions(filp, cmd))
		return -EPERM;

	switch (cmd) {
	case DW_IOCTL_GATE_CLOCK:
		return dw_clock_gating(ds_dev, arg);
	default:
		return -ENOTTY; /* unknown command */
	}
}

/*
 * dw_clock_gating: Gates or un-gates Oscar clock.
 * @ds_dev: DS device pointer.
 * @arg: User ioctl arg, in this case to a dw_gate_clock_ioctl struct.
 */
static long dw_clock_gating(struct ds_dev *ds_dev, ulong arg)
{
	return 0;
}

/*
 * sysfs_show: Display driver sysfs entries.
 * @device: Kernel device structure.
 * @attr: Attribute to display.
 * @buf: Buffer to which to write output.
 *
 * Description: Looks up the driver data and file-specific attribute data (the
 * type of the attribute), then fills "buf" accordingly.
 */
static ssize_t sysfs_show(
	struct device *device, struct device_attribute *attr, char *buf)
{
	struct ds_dev *ds_dev =
		(struct ds_dev *)ds_sysfs_get_device_data(device);
	enum sysfs_attribute_type type =
		(enum sysfs_attribute_type)ds_sysfs_get_attr_data(device, attr);
	if (ds_dev == NULL) {
		ds_nodev_error("No DW device sysfs mapping found");
		return 0;
	}

	switch (type) {
	case ATTR_KERNEL_HIB_PAGE_TABLE_SIZE:
		return scnprintf(buf, PAGE_SIZE, "%u\n",
			ds_page_table_num_entries(ds_dev->page_table[0]));
	case ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE:
		return scnprintf(buf, PAGE_SIZE, "%u\n",
			ds_page_table_num_entries(ds_dev->page_table[0]));
	case ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES:
		return scnprintf(buf, PAGE_SIZE, "%u\n",
			ds_page_table_num_active_pages(ds_dev->page_table[0]));
	default:
		ds_log_error(ds_dev, "Unknown attribute: %s", attr->attr.name);
		return 0;
	}
}
