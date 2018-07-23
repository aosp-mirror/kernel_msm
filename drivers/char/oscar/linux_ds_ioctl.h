/* Common DS device kernel and user space declarations.
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
#ifndef __LINUX_DS_IOCTL_H__
#define __LINUX_DS_IOCTL_H__

#include <linux/ioctl.h>
#include <linux/types.h>
#ifndef __KERNEL__
#include <stdint.h>
#endif

/* ioctl structure declarations */

/* Ioctl structures are padded to a multiple of 64 bits */
/* and padded to put 64 bit values on 64 bit boundaries. */
/* Unsigned 64 bit integers are used to hold pointers. */
/* This helps compatibility between 32 and 64 bits. */

/*
 * Common structure for ioctls associating an eventfd with a device interrupt,
 * when using the DS interrupt module.
 */
struct ds_interrupt_eventfd {
	uint64_t interrupt;
	uint64_t event_fd;
};

/*
 * Common structure for ioctls mapping and unmapping buffers when using the
 * DS page_table module.
 */
struct ds_page_table_ioctl {
	uint64_t page_table_index;
	uint64_t size;
	uint64_t host_address;
	uint64_t device_address;
};

/*
 * Common structure for ioctls mapping and unmapping buffers when using the
 * DS page_table module.
 * dma_address: phys addr start of coherent memory, allocated by kernel
 */
struct ds_coherent_alloc_config_ioctl {
	uint64_t page_table_index;
	uint64_t enable;
	uint64_t size;
	uint64_t dma_address;
};

/* Base number for all DS-common IOCTLs */
#define DS_IOCTL_BASE 0xDC

/*
 * DS_IOCTL_RESET: Reset the device using the specified reset type.
 */
// NOLINTNEXTLINE
#define DS_IOCTL_RESET _IOW(DS_IOCTL_BASE, 0, unsigned long)

/*
 * DS_IOCTL_SET_EVENTFD: Associate the specified [event]fd with the specified
 * interrupt.
 */
// NOLINTNEXTLINE
#define DS_IOCTL_SET_EVENTFD _IOW(DS_IOCTL_BASE, 1, struct ds_interrupt_eventfd)

/*
 * JF_IOCTL_CLEAR_EVENTFD: Clears any eventfd associated with the specified
 * interrupt. The (ulong) argument is the interrupt number to clear.
 */
// NOLINTNEXTLINE
#define DS_IOCTL_CLEAR_EVENTFD _IOW(DS_IOCTL_BASE, 2, unsigned long)

/*
 * JF_IOCTL_LOOPBACK_RAISE: [Loopbacks only] Requests that the loopback
 * device send the specified interrupt to the host. The (ulong) argument is the
 * number of the interrupt to send.
 */
// NOLINTNEXTLINE
#define DS_IOCTL_LOOPBACK_INTERRUPT _IOW(DS_IOCTL_BASE, 3, unsigned long)

/*
 * DS_IOCTL_NUMBER_PAGE_TABLES: Queries the kernel for the number of page tables
 * supported by the device.
 */
#define DS_IOCTL_NUMBER_PAGE_TABLES _IOR(DS_IOCTL_BASE, 4, uint64_t)

/*
 * DS_IOCTL_PAGE_TABLE_SIZE: Queries the kernel for the maximum size of the
 * page table.  Only the size and page_table_index fields are used from the
 * struct ds_page_table_ioctl.
 */
#define DS_IOCTL_PAGE_TABLE_SIZE                                               \
	_IOWR(DS_IOCTL_BASE, 5, struct ds_page_table_ioctl)

/*
 * DS_IOCTL_SIMPLE_PAGE_TABLE_SIZE: Queries the kernel for the current simple
 * page table size.  Only the size and page_table_index fields are used from
 * the struct ds_page_table_ioctl.
 */
#define DS_IOCTL_SIMPLE_PAGE_TABLE_SIZE                                        \
	_IOWR(DS_IOCTL_BASE, 6, struct ds_page_table_ioctl)

/*
 * DS_IOCTL_PARTITION_PAGE_TABLE: Tells the kernel to change the split between
 * the number of simple and extended entries in the given page table.  Only the
 * size and page_table_index fields are used from the struct
 * ds_page_table_ioctl.
 */
#define DS_IOCTL_PARTITION_PAGE_TABLE                                          \
	_IOW(DS_IOCTL_BASE, 7, struct ds_page_table_ioctl)

/*
 * DS_IOCTL_MAP_BUFFER: Tells the kernel to map size bytes at host_address to
 * device_address in page_table_index page table.
 */
#define DS_IOCTL_MAP_BUFFER _IOW(DS_IOCTL_BASE, 8, struct ds_page_table_ioctl)

/*
 * DS_IOCTL_UNMAP_BUFFER: Tells the kernel to unmap size bytes at host_address
 * from device_address in page_table_index page table.
 */
#define DS_IOCTL_UNMAP_BUFFER _IOW(DS_IOCTL_BASE, 9, struct ds_page_table_ioctl)

/*
 * DS_IOCTL_CLEAR_INTERRUPT_COUNTS: Clear the interrupt counts stored for this
 * device.
 */
#define DS_IOCTL_CLEAR_INTERRUPT_COUNTS _IO(DS_IOCTL_BASE, 10)

/*
 * DS_IOCTL_CONFIG_COHERENT_ALLOCATOR: Enable/Disable and configure the coherent
 * allocator.
 */
#define DS_IOCTL_CONFIG_COHERENT_ALLOCATOR                                     \
	_IOWR(DS_IOCTL_BASE, 11, struct ds_coherent_alloc_config_ioctl)

#endif /* __LINUX_DS_IOCTL_H__ */
