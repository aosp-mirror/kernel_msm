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

#include "ds_interrupt.h"

#include "ds_constants.h"
#include "ds_generic.h"
#include "ds_logging.h"
#include "ds_sysfs.h"
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/mfd/abc-pcie.h>
#include "../../mfd/abc-pcie-private.h"

#if !defined(CONFIG_PCI) && !defined(PLATFORM_INTERFACE)
#error HOW DWN IS INTERFACING THE SYSTEM?
#endif

#ifdef DS_KERNEL_TRACE_SUPPORT
#define CREATE_TRACE_POINTS
#include <trace/events/ds_interrupt.h>
#else
#define trace_ds_interrupt_event(x, ...)
#endif
/* Retry attempts if the requested number of interrupts aren't available. */
#define MSIX_RETRY_COUNT 3

/* struct ds_interrupt_data: Instance interrupt management data. */
struct ds_interrupt_data {

	/* The name associated with this interrupt data. */
	const char *name;

	/* Interrupt type. See ds_interrupt_type in ds_generic.h */
	int type;

	/* The PCI device [if any] associated with the owning device.
	 * This field is used to distinguish between platform devices and PCI
	 * devices.
	 */
	struct pci_dev *pci_dev;

	/* Set to 1 if interrupt initalization is complete, 0 otherwise. */
	int init_complete;

	/* The number of interrupts requested by the owning device. */
	int num_interrupts;

	/* The number of times each interrupt has been called. */
	ulong interrupt_counts[DS_MAX_INTERRUPTS];

	/* A pointer to the interrupt descriptor struct for this device. */
	const struct ds_interrupt_desc *interrupts;

	/* The index of the bar into which interrupts should be mapped. */
	int interrupt_bar_index;

	/* The width of a single interrupt in a packed interrupt register. */
	int pack_width;

	/* offset of wire interrupt registers */
	const struct ds_wire_interrupt_offsets *wire_interrupt_offsets;

	/*
	 * Design-wise, these three elements should be bundled together, but
	 * pci_enable_msix's interface requires that they be managed
	 * individually (requires array of struct msix_entry).
	 */

	/* The number of successfully configured interrupts. */
	int num_configured;

#ifndef PLATFORM_INTERFACE
	/* The MSI-X data for each requested/configured interrupt. */
	struct msix_entry msix_entries[DS_MAX_INTERRUPTS];
#endif

	/* The eventfd "callback" data for each interrupt. */
	struct eventfd_ctx *eventfd_ctxs[DS_MAX_INTERRUPTS];

	/*
	 * Linux IRQ number.
	 */
	int irq;
};

/* Function definitions. */
static ssize_t interrupt_sysfs_show(
	struct device *device, struct device_attribute *attr, char *buf);
static int ds_platform_interrupt_handler(uint32_t irq, void *dev_id);
static void ds_platform_free_handler(struct ds_dev *ds_dev);
static int ds_platform_register_handler(struct ds_dev *ds_dev);

#ifndef PLATFORM_INTERFACE
static irqreturn_t ds_msix_interrupt_handler(int irq, void *dev_id);
static irqreturn_t ds_msi_interrupt_handler(int irq, void *dev_id);
#endif

/* Structures to display interrupt counts in sysfs. */
enum interrupt_sysfs_attribute_type {
	ATTR_INTERRUPT_COUNTS,
};

static struct ds_sysfs_attribute interrupt_sysfs_attrs[] = {
	DS_SYSFS_RO(
		interrupt_counts, interrupt_sysfs_show, ATTR_INTERRUPT_COUNTS),
	DS_END_OF_ATTR_ARRAY,
};

/*
 * ds_interrupt_setup: Set up device registers for interrupt handling.
 * @ds_dev: The DS information structure for this device.
 *
 * Sets up the device registers with the correct indices for the relevant
 * interrupts.
 */
static void ds_interrupt_setup(struct ds_dev *ds_dev);

#ifndef PLATFORM_INTERFACE
/* MSIX init and cleanup. */
static int ds_interrupt_msix_init(
		struct ds_dev *dev, struct ds_interrupt_data *interrupt_data);
static void ds_interrupt_msix_cleanup(struct ds_interrupt_data *interrupt_data);

/* MSI init and cleanup. */
static int ds_interrupt_msi_init(
		struct ds_dev *dev, struct ds_interrupt_data *interrupt_data);
static void ds_interrupt_msi_cleanup(struct ds_interrupt_data *interrupt_data);
#endif

int ds_interrupt_init(struct ds_dev *ds_dev, const char *name, int type,
	const struct ds_interrupt_desc *interrupts, int num_interrupts,
	int pack_width, int bar_index,
	const struct ds_wire_interrupt_offsets *wire_int_offsets)
{
	int ret;
	struct ds_interrupt_data *interrupt_data;

	if (num_interrupts >= DS_MAX_INTERRUPTS) {
		ds_nodev_error("Too many interrupts requested: %d; max %d\n",
			num_interrupts, DS_MAX_INTERRUPTS);
		return -EINVAL;
	}

	interrupt_data = kzalloc(sizeof(struct ds_interrupt_data), GFP_KERNEL);
	if (!interrupt_data)
		return -ENOMEM;
	ds_dev->interrupt_data = interrupt_data;
	interrupt_data->name = name;
	interrupt_data->type = type;
	interrupt_data->pci_dev = ds_dev->pci_dev;
	interrupt_data->num_interrupts = num_interrupts;
	memset(interrupt_data->interrupt_counts, 0,
		DS_MAX_INTERRUPTS * sizeof(*interrupt_data->interrupt_counts));
	interrupt_data->interrupts = interrupts;
	interrupt_data->interrupt_bar_index = bar_index;
	interrupt_data->pack_width = pack_width;
	interrupt_data->num_configured = 0;
	interrupt_data->wire_interrupt_offsets = wire_int_offsets;

	switch (interrupt_data->type) {
#ifndef PLATFORM_INTERFACE
	case PCI_MSIX:
		ret = ds_interrupt_msix_init(ds_dev, interrupt_data);
		break;

	case PCI_MSI:
		ret = ds_interrupt_msi_init(ds_dev, interrupt_data);
		break;
#endif
	case PLATFORM_WIRE:
		ret = ds_platform_register_handler(ds_dev);
		break;

	default:
		ds_nodev_error("Cannot handle unsuport interrupt type %d.",
			       interrupt_data->type);
		ret = -EINVAL;
	};

	if (ret) {
		/* Failing to setup interrupts will cause the device to report
		 * DS_STATUS_LAMED. But it is not fatal.
		 */
		ds_log_warn(ds_dev, "Couldn't initialize interrupts: %d", ret);
		return 0;
	}

	ds_interrupt_setup(ds_dev);
	ds_sysfs_create_entries(ds_dev->dev_info.device, interrupt_sysfs_attrs);

	return 0;
}

static int ds_platform_register_handler(struct ds_dev *ds_dev)
{
	struct device *dev;
	int ret, i;

	WARN_ON(!ds_dev);
	WARN_ON(!ds_dev->interrupt_data);
	dev = ds_get_device(ds_dev);
	for (i = 0; i < ds_dev->interrupt_data->num_interrupts; i++) {
		ret = abc_reg_irq_callback2(&ds_platform_interrupt_handler,
				ds_dev->interrupt_data->interrupts[i].index,
				ds_dev);
		if (ret) {
			ds_nodev_error("Cannot get IRQ for index %d\n",  i);
			return ret;
		}
		ds_dev->interrupt_data->num_configured++;
	}
	ds_dev->interrupt_data->init_complete = 1;

	return 0;
}

static void ds_platform_free_handler(struct ds_dev *ds_dev)
{

	struct device *dev;

	WARN_ON(!ds_dev);
	WARN_ON(!ds_dev->interrupt_data);

	dev = ds_get_device(ds_dev);
	WARN_ON(!dev);

	ds_dev->interrupt_data->irq = -1;
	ds_dev->interrupt_data->init_complete = 0;
}

#ifndef PLATFORM_INTERFACE
static int ds_interrupt_msix_init(
	struct ds_dev *dev, struct ds_interrupt_data *interrupt_data)
{
	int ret;
	int i;

	for (i = 0; i < interrupt_data->num_interrupts; i++) {
		interrupt_data->msix_entries[i].entry = i;
		interrupt_data->msix_entries[i].vector = 0;
		interrupt_data->eventfd_ctxs[i] = NULL;
	}
	/* Retry MSIX_RETRY_COUNT times if not enough IRQs are available. */
	ret = interrupt_data->num_interrupts;
	for (i = 0; i < MSIX_RETRY_COUNT && ret > 0; i++)
		ret = pci_enable_msix(interrupt_data->pci_dev,
			interrupt_data->msix_entries, ret);

	if (ret)
		return ret > 0 ? -EBUSY : ret;
	interrupt_data->init_complete = 1;

	for (i = 0; i < interrupt_data->num_interrupts; i++) {
		ret = request_irq(interrupt_data->msix_entries[i].vector,
			ds_msix_interrupt_handler, 0, interrupt_data->name,
			&interrupt_data->msix_entries[i]);
		if (ret) {
			ds_nodev_error(
				"Cannot get IRQ %d for interrupt %d; %d\n",
				interrupt_data->msix_entries[i].vector, i, ret);
			return ret;
		}

		interrupt_data->num_configured++;
	}

/* On QCM DragonBoard, we exit ds_interrupt_msix_init()
 * and kernel interrupt setup code with MSIX vectors masked.
 * This is wrong because nothing
 * else in the driver will normally touch the msix vectors.
 * As a temporary hack, force unmasking there.
 * TODO(vandalle) figure out why QCM kernel doesn't unmask
 * the MSIX vectors, after ds_interrupt_msix_init(), and remove
 * this code.
 * b/35955848 MSIX vectors not unmasked by QCM810 kernel
 */
#define MSIX_VECTOR_SIZE 16
#define MSIX_MASK_BIT_OFFSET 12
#define DW_BAR2_REG_KERNEL_HIB_MSIX_TABLE 0x46800
	for (i = 0; i < dev->interrupt_data->num_configured; i++) {
		/* check if the MSIX vector is unmasked */
		ulong location = DW_BAR2_REG_KERNEL_HIB_MSIX_TABLE +
				 MSIX_MASK_BIT_OFFSET + i * MSIX_VECTOR_SIZE;
		u32 mask = ds_dev_read_32(dev,
			dev->interrupt_data->interrupt_bar_index, location);
		if (!(mask & 1))
			continue;
		/* Unmask the msix vector (clear 32 bits) */
		ds_dev_write_32(dev, 0,
			dev->interrupt_data->interrupt_bar_index, location);
	}
#undef MSIX_VECTOR_SIZE
#undef MSIX_MASK_BIT_OFFSET
#undef DW_BAR2_REG_KERNEL_HIB_MSIX_TABLE
	return 0;
}


static int ds_interrupt_msi_init(
	struct ds_dev *dev, struct ds_interrupt_data *interrupt_data)
{
	int ret;
	int i;

	for (i = 0; i < interrupt_data->num_interrupts; i++)
		interrupt_data->eventfd_ctxs[i] = NULL;

	/* NOTE(mahdih): pci_enable_msi_range is depricated, use
	 * pci_enable_msi_exact instead
	 * NOTE(jnjoseph): For some reason, interrupts gets enabled
	 * only if enable *all* 32 interrupts.
	 * Support for pci_enable_msi_block was dropped in 3.16 in favor of
	 * pci_enable_msi_range which was implemented in 3.13.x kernels
	 * (but not 3.13.0).
	 */
	ret = pci_enable_msi_exact(dev->pci_dev, 32);
	if (ret != 0) {
		ds_nodev_error("pci_msi_range failed with ret=%d\n", ret);
		return ret > 0 ? -EBUSY : ret;
	}

	interrupt_data->init_complete = 1;
	interrupt_data->irq = dev->pci_dev->irq;

	for (i = 0; i < interrupt_data->num_interrupts; i++) {
		ret = request_irq(dev->pci_dev->irq + i,
				  ds_msi_interrupt_handler, IRQF_SHARED,
				  interrupt_data->name,
				  interrupt_data);
		if (ret) {
			ds_nodev_error(
				"Cannot get IRQ %d for interrupt %d; %d\n",
				dev->pci_dev->irq + i, i, ret);
			return ret;
		}

		interrupt_data->num_configured++;
	}

	return 0;
}

static void ds_interrupt_msix_cleanup(struct ds_interrupt_data *interrupt_data)
{
	int i;

	for (i = 0; i < interrupt_data->num_configured; i++) {
		free_irq(interrupt_data->msix_entries[i].vector,
			&interrupt_data->msix_entries[i]);
	}
	interrupt_data->num_configured = 0;

	if (interrupt_data->init_complete)
		pci_disable_msix(interrupt_data->pci_dev);
	interrupt_data->init_complete = 0;
}


static void ds_interrupt_msi_cleanup(struct ds_interrupt_data *interrupt_data)
{
	int i;

	for (i = 0; i < interrupt_data->num_configured; i++)
		free_irq(interrupt_data->pci_dev->irq + i, interrupt_data);
	interrupt_data->num_configured = 0;

	if (interrupt_data->init_complete)
		pci_disable_msi(interrupt_data->pci_dev);
	interrupt_data->init_complete = 0;
}
#endif


int ds_interrupt_reinit(struct ds_dev *ds_dev)
{
	int ret;

	if (!ds_dev->interrupt_data) {
		ds_log_error(ds_dev,
			"Attempted to reinit uninitialized interrupt data.");
		return -EINVAL;
	}

	switch (ds_dev->interrupt_data->type) {
#ifndef PLATFORM_INTERFACE
	case PCI_MSIX:
		ds_interrupt_msix_cleanup(ds_dev->interrupt_data);
		ret = ds_interrupt_msix_init(ds_dev, ds_dev->interrupt_data);
		break;

	case PCI_MSI:
		ds_interrupt_msi_cleanup(ds_dev->interrupt_data);
		ret = ds_interrupt_msi_init(ds_dev, ds_dev->interrupt_data);
		break;
#endif
	case PLATFORM_WIRE:
		/* Nothing needs to be done for platform devices. */
		ret = 0;
		break;

	default:
		ds_nodev_error("Cannot handle unsuport interrupt type %d.",
			       ds_dev->interrupt_data->type);
		ret = -EINVAL;
	};

	if (ret) {
		/* Failing to setup interrupts will cause the device to report
		 * DS_STATUS_LAMED. But it is not fatal.
		 */
		ds_log_warn(ds_dev, "Couldn't re-initialize interrupts: %d",
			    ret);
		return 0;
	}

	ds_interrupt_setup(ds_dev);

	return 0;
}

/* See ds_interrupt.h for description. */
int ds_interrupt_reset_counts(struct ds_dev *ds_dev)
{
	ds_log_info(ds_dev, "Clearing interrupt counts.");
	memset(ds_dev->interrupt_data->interrupt_counts, 0,
		DS_MAX_INTERRUPTS *
			sizeof(*ds_dev->interrupt_data->interrupt_counts));
	return 0;
}

static void ds_interrupt_setup(struct ds_dev *ds_dev)
{
	int i;
	int pack_shift;
	ulong mask;
	ulong value;
	struct ds_interrupt_data *interrupt_data = ds_dev->interrupt_data;

	if (!interrupt_data) {
		ds_log_error(ds_dev, "Interrupt data is not initialized.");
		return;
	}

	ds_log_info(ds_dev, "Running interrupt setup.");

	if (interrupt_data->type == PLATFORM_WIRE ||
	    interrupt_data->type == PCI_MSI) {
		/* Nothing needs to be done for platform or PCI devices. */
		return;
	}

	if (interrupt_data->type != PCI_MSIX) {
		ds_nodev_error("Cannot handle unsuport interrupt type %d.",
			       interrupt_data->type);
		return;

	}

	/* Setup the MSIX table. */

	for (i = 0; i < interrupt_data->num_interrupts; i++) {
		/*
		 * If the interrupt is not packed, we can write the index into
		 * the register directly. If not, we need to deal with a read-
		 * modify-write and shift based on the packing index.
		 */
		ds_nodev_info("Setting up interrupt index %d with index 0x%llx "
			      "and packing %d",
			interrupt_data->interrupts[i].index,
			interrupt_data->interrupts[i].reg,
			interrupt_data->interrupts[i].packing);
		if (interrupt_data->interrupts[i].packing == UNPACKED) {
			value = interrupt_data->interrupts[i].index;
		} else {
			switch (interrupt_data->interrupts[i].packing) {
			case PACK_0:
				pack_shift = 0;
				break;
			case PACK_1:
				pack_shift = interrupt_data->pack_width;
				break;
			case PACK_2:
				pack_shift = 2 * interrupt_data->pack_width;
				break;
			case PACK_3:
				pack_shift = 3 * interrupt_data->pack_width;
				break;
			default:
				ds_nodev_error("Found interrupt description "
					"with unknown enum %d.",
					interrupt_data->interrupts[i].packing);
				return;
			}

			mask = ~(0xFFFF << pack_shift);
			value = ds_dev_read_64(ds_dev,
					interrupt_data->interrupt_bar_index,
					interrupt_data->interrupts[i].reg) &
				mask;
			value |= interrupt_data->interrupts[i].index
				 << pack_shift;
		}
		ds_dev_write_64(ds_dev, value,
			interrupt_data->interrupt_bar_index,
			interrupt_data->interrupts[i].reg);
	}
}

/* See ds_interrupt.h for description. */
void ds_interrupt_pause(struct ds_dev *ds_dev, int enable_pause)
{
	uint64_t offset;

	WARN_ON(!ds_dev);

	if (!ds_dev->interrupt_data)
		return; /* nothing to do */

	if (ds_dev->interrupt_data->type == PCI_MSI ||
	    ds_dev->interrupt_data->type == PCI_MSIX) {
		/* Nothing to be done for MSI/MSIX just yet. */
	}

	WARN_ON(ds_dev->interrupt_data->type != PLATFORM_WIRE);

	if (enable_pause && ds_dev->interrupt_data->init_complete) {
		WARN_ON(!ds_dev->interrupt_data->wire_interrupt_offsets);
		WARN_ON(!ds_dev->interrupt_data->wire_interrupt_offsets
		->mask_array);

		/* Mask wire interrupt. */
		offset = ds_dev->interrupt_data
			->wire_interrupt_offsets->mask_array;
		ds_dev_write_64(ds_dev, -1LL,
				ds_dev->interrupt_data->interrupt_bar_index,
				offset);
		ds_platform_free_handler(ds_dev);
	} else if (!enable_pause &&
		   !ds_dev->interrupt_data->init_complete) {
		WARN_ON(!ds_dev->interrupt_data->wire_interrupt_offsets);
		WARN_ON(!ds_dev->interrupt_data->wire_interrupt_offsets
		->mask_array);

		/* Un-mask wire interrupt. */
		offset = ds_dev->interrupt_data->wire_interrupt_offsets
			->mask_array;
		ds_dev_write_64(ds_dev, 0,
				ds_dev->interrupt_data->interrupt_bar_index,
				offset);

		ds_platform_register_handler(ds_dev);
	}
}
EXPORT_SYMBOL(ds_interrupt_pause);

void ds_interrupt_cleanup(struct ds_dev *ds_dev)
{
	struct ds_interrupt_data *interrupt_data = ds_dev->interrupt_data;
	/*
	 * It is possible to get an error code from ds_interrupt_init
	 * before interrupt_data has been allocated, so check it.
	 */
	if (!interrupt_data)
		return;

	switch (interrupt_data->type) {
#ifndef PLATFORM_INTERFACE
	case PCI_MSIX:
		ds_interrupt_msix_cleanup(interrupt_data);
		break;

	case PCI_MSI:
		ds_interrupt_msi_cleanup(interrupt_data);
		break;
#endif
	case PLATFORM_WIRE:
		ds_platform_free_handler(ds_dev);
		break;

	default:
		ds_nodev_error("Cannot handle unsuport interrupt type %d.",
			       interrupt_data->type);
	};

	kfree(interrupt_data);
	ds_dev->interrupt_data = NULL;
}

int ds_interrupt_system_status(struct ds_dev *ds_dev)
{
	if (!ds_dev->interrupt_data) {
		ds_nodev_info("Interrupt data is null.");
		return DS_STATUS_DEAD;
	}

	if (!ds_dev->interrupt_data->init_complete) {
		ds_nodev_info("Interrupt not initialized.");
		return DS_STATUS_LAMED;
	}

	if (ds_dev->interrupt_data->num_configured !=
	    ds_dev->interrupt_data->num_interrupts) {
		ds_nodev_info("Not all interrupts were configured.");
		return DS_STATUS_LAMED;
	}

	return DS_STATUS_ALIVE;
}

int ds_interrupt_set_eventfd(
	struct ds_interrupt_data *interrupt_data, int interrupt, int event_fd)
{
	struct eventfd_ctx *ctx = eventfd_ctx_fdget(event_fd);

	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	if (interrupt > interrupt_data->num_interrupts)
		return -EINVAL;

	interrupt_data->eventfd_ctxs[interrupt] = ctx;
	return 0;
}

int ds_interrupt_clear_eventfd(
	struct ds_interrupt_data *interrupt_data, int interrupt)
{
	if (interrupt < 0 || interrupt > interrupt_data->num_interrupts)
		return -EINVAL;

	interrupt_data->eventfd_ctxs[interrupt] = NULL;
	return 0;
}

int ds_interrupt_trigger_eventfd(
	struct ds_interrupt_data *interrupt_data, int interrupt)
{
	struct eventfd_ctx *ctx = interrupt_data->eventfd_ctxs[interrupt];

	if (!ctx)
		return -EINVAL;

	eventfd_signal(ctx, 1);
	return 0;
}

#ifndef PLATFORM_INTERFACE
struct msix_entry *ds_interrupt_get_msix_entries(
	struct ds_interrupt_data *interrupt_data)
{
	return interrupt_data->msix_entries;
}
#endif

struct eventfd_ctx **ds_interrupt_get_eventfd_ctxs(
	struct ds_interrupt_data *interrupt_data)
{
	return interrupt_data->eventfd_ctxs;
}
EXPORT_SYMBOL(ds_interrupt_get_eventfd_ctxs);

static ssize_t interrupt_sysfs_show(
	struct device *device, struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t written = 0, total_written = 0;
	struct ds_interrupt_data *interrupt_data;
	struct ds_dev *ds_dev =
		(struct ds_dev *)ds_sysfs_get_device_data(device);
	enum interrupt_sysfs_attribute_type sysfs_type =
		(enum interrupt_sysfs_attribute_type)ds_sysfs_get_attr_data(
			device, attr);
	if (ds_dev == NULL) {
		ds_nodev_error(
			"No sysfs mapping found for pointer 0x%p", device);
		return 0;
	}
	interrupt_data = ds_dev->interrupt_data;
	switch (sysfs_type) {
	case ATTR_INTERRUPT_COUNTS:
		for (i = 0; i < interrupt_data->num_interrupts; ++i) {
			written = scnprintf(buf, PAGE_SIZE - total_written,
				"0x%02x: %ld\n", i,
				interrupt_data->interrupt_counts[i]);
			total_written += written;
			buf += written;
		}
		return total_written;
	default:
		ds_log_error(ds_dev, "Unknown attribute: %s", attr->attr.name);
		return 0;
	}
}

#ifndef PLATFORM_INTERFACE

/*
 * MSIX interrupt handler, used with PCI driver.
 */
static irqreturn_t ds_msix_interrupt_handler(int irq, void *dev_id)
{
	struct eventfd_ctx *ctx;
	struct ds_interrupt_data *interrupt_data;
	struct msix_entry *msix_entry = dev_id;
	int interrupt = msix_entry->entry;

	interrupt_data = container_of(
		msix_entry, struct ds_interrupt_data, msix_entries[interrupt]);
	trace_ds_interrupt_event(interrupt_data->name, interrupt);

	ctx = interrupt_data->eventfd_ctxs[interrupt];
	if (ctx)
		eventfd_signal(ctx, 1);

	++(interrupt_data->interrupt_counts[interrupt]);

	return IRQ_HANDLED;
}

static irqreturn_t ds_msi_interrupt_handler(int irq, void *dev_id)
{
	struct eventfd_ctx *ctx;
	struct ds_interrupt_data *interrupt_data =
		(struct ds_interrupt_data *) dev_id;
	int interrupt = irq - interrupt_data->irq;

	trace_ds_interrupt_event(interrupt_data->name, interrupt);
	ctx = interrupt_data->eventfd_ctxs[interrupt];
	if (ctx)
		eventfd_signal(ctx, 1);

	++(interrupt_data->interrupt_counts[interrupt]);

	return IRQ_HANDLED;
}

#endif

/*
 * Platform interrupt handler, used with platform driver.
 */
static int ds_platform_interrupt_handler(uint32_t irq, void *dev_id)
{
	struct eventfd_ctx *ctx;
	struct ds_interrupt_data *interrupt_data;
	struct ds_dev *dev;
	u64 offset;
	u64 pending_bits;
	u64 masked_bits;
	int i;
	/* platform driver has only one interrupt line */

	dev = (struct ds_dev *)dev_id;
	interrupt_data = dev->interrupt_data;
	trace_ds_interrupt_event(interrupt_data->name, interrupt);

	if (interrupt_data->wire_interrupt_offsets) {
		/* Read the pending bit and set the corresponding mask.
		 * Interrupts are processed by the run-time stack in user land,
		 * which is responsible for clearing the mask.
		 */
		offset = interrupt_data->wire_interrupt_offsets
				 ->pending_bit_array;
		pending_bits = ds_dev_read_64(
			dev, interrupt_data->interrupt_bar_index, offset);
		offset = interrupt_data->wire_interrupt_offsets->mask_array;
		masked_bits = ds_dev_read_64(
			dev, interrupt_data->interrupt_bar_index, offset);
		ds_dev_write_64(dev, pending_bits | masked_bits,
			interrupt_data->interrupt_bar_index, offset);
	}

	for (i = 0; i < interrupt_data->num_interrupts; i++) {
		if (irq == interrupt_data->interrupts[i].index) {
			ctx = interrupt_data->eventfd_ctxs[i];
			if (ctx) {
				ds_log_info(dev,
					"handler for %d is not NULL.", irq);
				eventfd_signal(ctx, 1);
				++(interrupt_data->interrupt_counts[i]);
				return IRQ_HANDLED;
			}
		}
	}
	ds_log_info(dev, "handler for %d is NULL.", irq);
	return -EPERM;
}
