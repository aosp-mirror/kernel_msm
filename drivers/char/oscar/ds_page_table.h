/* DS Page Table functionality. This file describes the address
 * translation/paging functionality supported by the DS driver framework.
 * As much as possible, internal details are hidden to simplify use -
 * all calls are thread-safe (protected by an internal mutex) except where
 * indicated otherwise.
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

#ifndef __DS_ADDR_TRNSL_H__
#define __DS_ADDR_TRNSL_H__

#include <linux/pci.h>
#include <linux/types.h>

#include "ds_constants.h"
#include "ds_generic.h"

/*
 * Structure used for managing address translation on a device. All details are
 * internal to the implementation.
 */
struct ds_page_table;

/*
 * ds_init: Allocate and init address translation data.
 * @ppage_table: Pointer to DS page table pointer. Set by this call.
 * @att_base_reg: [Mapped] pointer to the first entry in the device's address
 *                translation table.
 * @extended_offset_reg: [Mapped] pointer to the device's register containing
 *                       the starting index of the extended translation table.
 * @extended_bit_location: The index of the bit indicating whether an address
 *                         is extended.
 * @total_entries: The total number of entries in the device's address
 *                 translation table.
 * @device: Device structure for the underlying device. Only used for logging.
 * @bool has_dma_ops: Whether the page table uses arch specific dma_ops or
 * whether the driver will supply its own.
 *
 * Description: Allocates and initializes data to track DS chip address
 * translation - simple and extended page table metadata. Initially, the page
 * table is partitioned such that all addresses are "simple" (single-level
 * lookup). ds_partition_page_table can be called to change this paritioning.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int ds_page_table_init(struct ds_page_table **ppg_tbl,
	const struct ds_bar_data *bar_data,
	const struct ds_page_table_offsets *page_table_offsets,
	uint extended_bit_location, struct device *device, bool dma_ops);

/*
 * ds_cleanup: Deallocate and cleanup address translation data.
 * @page_table: DS page table pointer.
 *
 * Description: The inverse of ds_init; frees page_table and its contained
 *              elements.
 *
 *	        Because this call destroys the page table, it cannot be
 *	        thread-safe (mutex-protected)!
 */
void ds_page_table_cleanup(struct ds_page_table *page_table);

/*
 * ds_partition_page_table: Sets the size of the simple page table.
 * @page_table: DS page table pointer.
 * @num_simple_entries: Desired size of the simple page table (in entries).
 *
 * Description: ds_partition_page_table checks to see if the simple page
 *              size can be changed (i.e., if there are no active extended
 *              mappings in the new simple size range), and, if so,
 *              sets the new simple and extended page table sizes.
 *
 *              Returns 0 if successful, or non-zero if the page table entries
 *              are not free.
 */
int ds_page_table_partition(
	struct ds_page_table *page_table, uint num_simple_entries);

/*
 * ds_page_table_map: Get and map [host] user space pages into device memory.
 * @page_table: DS page table pointer.
 * @host_addr: Starting host virtual memory address of the pages.
 * @dev_addr: Starting device address of the pages.
 * @num_pages: Number of [4kB] pages to map.
 *
 * Description: Maps the "num_pages" pages of host memory pointed to by
 *              host_addr to the address "dev_addr" in device memory.
 *
 *              The caller is responsible for checking the addresses ranges.
 *
 *              Returns 0 if successful or a non-zero error number otherwise.
 *              If there is an error, no pages are mapped.
 */
int ds_page_table_map(struct ds_page_table *page_table, ulong host_addr,
	ulong dev_addr, uint num_pages);

/*
 * ds_page_table_unmap: Un-map host pages from device memory.
 * @page_table: DS page table pointer.
 * @dev_addr: Starting device address of the pages to unmap.
 * @num_pages: The number of [4kB] pages to unmap.
 *
 * Description: The inverse of ds_map_pages. Unmaps pages from the device.
 */
void ds_page_table_unmap(
	struct ds_page_table *page_table, ulong dev_addr, uint num_pages);

/*
 * ds_page_table_unmap_all: Unmap ALL host pages from device memory.
 * @page_table: DS page table pointer.
 */
void ds_page_table_unmap_all(struct ds_page_table *page_table);

/*
 * void ds_page_table_reset: Unmap all host pages from device memory and reset
 * the table to fully simple addressing.
 * @page_table: DS page table pointer.
 */
void ds_page_table_reset(struct ds_page_table *page_table);

/*
 * ds_garbage_collect: Reclaims unused page table memory.
 * @page_table: DS page table pointer.
 *
 * Description: Examines the page table and frees any currently-unused
 *              allocations. Called internally on ds_cleanup().
 */
void ds_page_table_garbage_collect(struct ds_page_table *page_table);

/*
 * ds_page_table_lookup_page: Retrieve the backing page for a device address.
 * @page_table: DS page table pointer.
 * @dev_addr: DS device address.
 * @ppage: Pointer to a page pointer for the returned page.
 * @poffset: Pointer to an unsigned long for the returned offset.
 *
 * Description: Interprets the address and looks up the corresponding page
 *              in the page table and the offset in that page.  (We need an
 *              offset because the host page may be larger than the DS chip
 *              page it contains.)
 *
 *              Returns 0 if successful, -1 for an error.  The page pointer
 *              and offset are returned through the pointers, if successful.
 */
int ds_page_table_lookup_page(struct ds_page_table *page_table, ulong dev_addr,
	struct page **page, ulong *poffset);

/*
 * ds_page_table_is_mapping_bad: Checks validity for input addrs and size.
 * @page_table: DS page table pointer.
 * @host_addr: Host address to check.
 * @dev_addr: DS device address.
 * @bytes: Size of the range to check (in bytes).
 *
 * Description: This call performs a number of checks to verify that the ranges
 * specified by both addresses and the size are valid for mapping pages into
 * DS device memory.
 *
 * Returns 1 if true - if the mapping is bad, 0 otherwise.
 */
int ds_page_table_are_addrs_bad(struct ds_page_table *page_table,
	ulong host_addr, ulong dev_addr, ulong bytes);

/*
 * ds_page_table_is_mapping_bad: Checks validity for input dev addr and size.
 * @page_table: DS page table pointer.
 * @dev_addr: DS device address.
 * @bytes: Size of the range to check (in bytes).
 *
 * Description: This call performs a number of checks to verify that the range
 * specified by the device address and the size is valid for mapping pages into
 * DS device memory.
 *
 * Returns 1 if true - if the address is bad, 0 otherwise.
 */
int ds_page_table_is_dev_addr_bad(
	struct ds_page_table *page_table, ulong dev_addr, ulong bytes);

/*
 * ds_page_table_max_size: Gets maximum size for the given page table.
 * @page_table: DS page table pointer.
 */
uint ds_page_table_max_size(struct ds_page_table *page_table);

/*
 * ds_page_table_num_entries: Gets the total number of entries in the arg.
 * @page_table: DS page table pointer.
 */
uint ds_page_table_num_entries(struct ds_page_table *page_table);

/*
 * ds_page_table_num_simple_entries: Gets the number of simple entries.
 * @page_table: DS page table pointer.
 */
uint ds_page_table_num_simple_entries(struct ds_page_table *page_table);

/*
 * ds_page_table_num_extended_entries: Gets the number of extended entries.
 * @page_table: DS page table pointer.
 */
uint ds_page_table_num_extended_entries(struct ds_page_table *page_table);

/*
 * ds_page_table_num_active_pages: Gets the number of actively pinned pages.
 * @page_table: DS page table pointer.
 */
uint ds_page_table_num_active_pages(struct ds_page_table *page_table);

/*
 * ds_page_table_system_status: Get status of page table managed by @page_table.
 * @page_table: DS page table pointer.
 */
int ds_page_table_system_status(struct ds_page_table *page_table);

/*
 * ds_alloc_coherent_memory: Allocate a block of coherent memory.
 * @ds_dev: DS Device.
 * @size: Size of the memory block.
 * @dma_address: Dma address allocated by the kernel.
 * @index: Index of the ds_page_table within this DS device
 *
 * Description: Allocate a contiguous coherent memory block, DMA'ble
 * by this device.
 */
int ds_alloc_coherent_memory(struct ds_dev *ds_dev, uint64_t size,
	dma_addr_t *dma_address, uint64_t index);
/*
 * Release a block of contiguous coherent memory, in use by a device.
 */
int ds_free_coherent_memory(struct ds_dev *ds_dev, uint64_t size,
	dma_addr_t dma_address, uint64_t index);

/*
 * ds_free_coherent_memory_all: Release all coherent memory.
 */
void ds_free_coherent_memory_all(struct ds_dev *ds_dev, uint64_t index);

/*
 * ds_set_user_virt: Records the host_addr to coherent dma memory mapping.
 * @ds_dev: DS Device.
 * @size: Size of the virtual address range to map.
 * @dma_address: Dma address within the coherent memory range.
 * @vma: Virtual address we wish to map to coherent memory.
 *
 * Description: For each page in the virtual address range, record the
 * coherent page mapping.
 *
 * Does not perform validity checking.
 */
int ds_set_user_virt(struct ds_dev *ds_dev, uint64_t size,
	dma_addr_t dma_address, ulong vma);

#endif
