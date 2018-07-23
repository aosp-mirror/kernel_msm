/* Implementation of DS page table support.
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

/*
 * The static functions in this file aren't prefixed with ds_page_table to
 * save precious horizontal space.
 *
 * This file assumes 4kB pages throughout; can be factored out when necessary.
 *
 * DS chip address format format is as follows.
 * Simple addresses - those whose containing pages are directly placed in the
 * device's address translation registers - are laid out as:
 * [ 63 - 40: Unused | 39 - 28: 0 | 27 - 12: page index | 11 - 0: page offset ]
 * page index:  The index of the containing page in the device's address
 *              translation registers.
 * page offset: The index of the address into the containing page.
 *
 * Extended address - those whose containing pages are contained in a second-
 * level page table whose address is present in the device's address translation
 * registers - are laid out as:
 * [ 63 - 40: Unused | 39: flag | 38 - 37: 0 | 36 - 21: dev/level 0 index |
 *   20 - 12: host/level 1 index | 11 - 0: page offset ]
 * flag:        Marker indicating that this is an extended address. Always 1.
 * dev index:   The index of the first-level page in the device's extended
 *              address translation registers.
 * host index:  The index of the containing page in the [host-resident] second-
 *              level page table.
 * page offset: The index of the address into the containing [second-level]
 *              page.
 */
#include "ds_page_table.h"

#include <linux/file.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pagemap.h>
#include <linux/vmalloc.h>
#include <linux/mfd/abc-pcie.h>
#include "ds_constants.h"
#include "ds_generic.h"
#include "ds_logging.h"

/* Constants & utility macros */
/* The number of pages that can be mapped into each second-level page table. */
#define DS_PAGES_PER_SUBTABLE 512

/* The starting position of the page index in a simple virtual address. */
#define DS_SIMPLE_PAGE_SHIFT 12

/* Flag indicating that a [device] slot is valid for use. */
#define DS_VALID_SLOT_FLAG 1

/*
 * The starting position of the level 0 page index (i.e., the entry in the
 * device's extended address registers) in an extended address.
 * Also can be thought of as (log2(PAGE_SIZE) + log2(PAGES_PER_SUBTABLE)),
 * or (12 + 9).
 */
#define DS_EXTENDED_LVL0_SHIFT 21

/*
 * Number of first level pages that DS chips support. Equivalent to
 * log2(NUM_LVL0_PAGE_TABLES)
 *
 * At a maximum, allowing for a 34 bits address space (or 16GB)
 *     = DS_EXTENDED_LVL0_WIDTH + (log2(PAGE_SIZE) + log2(PAGES_PER_SUBTABLE)
 * or, = 13 + 9 + 12
 */
#define DS_EXTENDED_LVL0_WIDTH 13

/*
 * The starting position of the level 1 page index (i.e., the entry in the
 * host second-level/sub- table) in an extended address.
 */
#define DS_EXTENDED_LVL1_SHIFT 12

/* Page-table specific error logging. */
#define ds_pg_tbl_error(pg_tbl, format, arg...)                                \
	ds_dev_log(err, (pg_tbl)->device, (struct pci_dev *)NULL, format, ##arg)

/* Type declarations */
/* enum pte_status: Valid states for a struct ds_page_table_entry. */
enum pte_status {
	PTE_FREE,
	PTE_INUSE,
};

/*
 * struct ds_page_table_entry: Mapping metadata for a single page.
 *
 * In this file, host-side page table entries are referred to as that (or PTEs).
 * Where device vs. host entries are differentiated, device-side or -visible
 * entries are called "slots". A slot may be either an entry in the device's
 * address translation table registers or an entry in a second-level page
 * table ("subtable").
 *
 * The full data in this structure is visible on the host [of course]. Only
 * the address contained in dma_addr is communicated to the device; that points
 * to the actual page mapped and described by this structure.
 */
struct ds_page_table_entry {
	/* The status of this entry/slot: free or in use. */
	enum pte_status status;

	/* Address of the page in DMA space. */
	dma_addr_t dma_addr;

	/* Linux page descriptor for the page described by this structure. */
	struct page *page;

	/*
	 * Index for alignment into host vaddrs.
	 * When a user specifies a host address for a mapping, that address may
	 * not be page-aligned. Offset is the index into the containing page of
	 * the host address (i.e., host_vaddr & (PAGE_SIZE - 1)).
	 * This is necessary for translating between user-specified addresses
	 * and page-aligned addresses.
	 */
	int offset;

	/*
	 * If this is an extended and first-level entry, sublevel points
	 * to the second-level entries underneath this entry.
	 */
	struct ds_page_table_entry *sublevel;
};

/*
 * struct ds_coherent_page_entry: maintains virtual to physical address
 * mapping for a coherent page that is allocated by this module for a given
 * device.
 * Note that coherent pages mappings virt mapping cannot be tracked by the
 * Linux kernel, and coherent pages don't have a struct page associated,
 * hence Linux kernel cannot perform a get_user_page_xx() on a phys address
 * that was allocated coherent.
 * This structure trivially implements this mechanism.
 *
 * TODO(vandwalle): we should be able to track multiple user virtual address, in
 * case multiple processes request the same coherent page.
 */
struct ds_coherent_page_entry {
	/* Phys address, dma'able by the owner device */
	dma_addr_t paddr;

	/* Kernel virtual address */
	uint64_t user_virt;

	/* User virtual address that was mapped by the mmap kernel subsystem */
	uint64_t kernel_virt;

	/* Whether this page has been mapped into a user land process virtual
	 * space
	 */
	uint32_t in_use;
};

/*
 * struct ds_page_table: [Host-side] page table descriptor.
 *
 * This structure tracks the metadata necessary to manage both simple and
 * extended page tables.
 */
struct ds_page_table {
	/* The total number of entries in the page table. */
	uint total_entries;

	/* The number of simple (single-level) entries in the page table. */
	uint num_simple_entries;

	/* The number of extended (two-level) entries in the page table. */
	uint num_extended_entries;

	/* Array of [host-side] page table entries. */
	struct ds_page_table_entry *entries;

	/* Number of actively mapped kernel pages in this table. */
	uint num_active_pages;

	/* Device register: base of/first slot in the page table. */
	u64 __iomem *base_slot;

	/* Device register: holds the offset indicating the start of the
	 * extended address region of the device's address translation table.
	 */
	u64 __iomem *extended_offset_reg;

	/* Device structure for the underlying device. */
	struct device *device;

	/* Location of the extended address bit for this DS device. */
	u64 extended_flag;

	/* Mutex to protect page table internals. */
	struct mutex mutex;

	/* Number of coherent pages accessible thru by this page table */
	int num_coherent_pages;

	/*
	 * List of coherent memory (physical) allocated for a device.
	 *
	 * This structure also remembers the user virtual mapping, this is
	 * hacky, but we need to do this because the kernel doesn't keep track
	 * of the user coherent pages (pfn pages), and virt to coherent page
	 * mapping.
	 * TODO: use find_vma() APIs to convert host address to vm_area, to
	 * dma_addr_t instead of storing user virtu address in
	 * ds_coherent_page_entry
	 *
	 * Note that the user virtual mapping is created by the driver, in
	 * ds_mmap function, so user_virt belongs in the driver anyhow.
	 */
	struct ds_coherent_page_entry *coherent_pages;

	/* Whether the page table uses arch specific dma_ops or
	 * whether the driver is supplying its own.
	 */
	bool dma_ops;
};

/* Mapping declarations */
static int ds_map_simple_pages(struct ds_page_table *pg_tbl, ulong host_addr,
	ulong dev_addr, uint num_pages);
static int ds_map_extended_pages(struct ds_page_table *pg_tbl, ulong host_addr,
	ulong dev_addr, uint num_pages);
static int ds_perform_mapping(struct ds_page_table *pg_tbl,
	struct ds_page_table_entry *pte_base, u64 __iomem *att_base,
	ulong host_addr, uint num_pages, int is_simple_mapping);

static int ds_alloc_simple_entries(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_pages);
static int ds_alloc_extended_entries(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_entries);
static int ds_alloc_extended_subtable(struct ds_page_table *pg_tbl,
	struct ds_page_table_entry *pte, u64 __iomem *att_reg);

/* Unmapping declarations */
static void ds_page_table_unmap_nolock(
	struct ds_page_table *pg_tbl, ulong start_addr, uint num_pages);
static void ds_page_table_unmap_all_nolock(struct ds_page_table *pg_tbl);
static void ds_unmap_simple_pages(
	struct ds_page_table *pg_tbl, ulong start_addr, uint num_pages);
static void ds_unmap_extended_pages(
	struct ds_page_table *pg_tbl, ulong start_addr, uint num_pages);
static void ds_perform_unmapping(struct ds_page_table *pg_tbl,
	struct ds_page_table_entry *pte_base, u64 __iomem *att_base,
	uint num_pages, int is_simple_mapping);

static void ds_free_extended_subtable(struct ds_page_table *pg_tbl,
	struct ds_page_table_entry *pte, u64 __iomem *att_reg);
static int ds_release_page(struct page *page);

/* Other/utility declarations */
static inline int ds_addr_is_simple(struct ds_page_table *pg_tbl, ulong addr);
static int ds_is_simple_dev_addr_bad(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_pages);
static int ds_is_extended_dev_addr_bad(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_pages);
static int ds_is_pte_range_free(
	struct ds_page_table_entry *pte, uint num_entries);
static void ds_page_table_garbage_collect_nolock(struct ds_page_table *pg_tbl);

/* Address format declarations */
static ulong ds_components_to_dev_address(struct ds_page_table *pg_tbl,
	int is_simple, uint page_index, uint offset);
static int ds_simple_page_idx(struct ds_page_table *pg_tbl, ulong dev_addr);
static ulong ds_extended_lvl0_page_idx(
	struct ds_page_table *pg_tbl, ulong dev_addr);
static ulong ds_extended_lvl1_page_idx(
	struct ds_page_table *pg_tbl, ulong dev_addr);

/* is_coherent: Determines whether a host buffer was mapped coherent
 * @pg_tbl: ds_page_table structure tracking the host buffer mapping
 * @host_addr: User virtual address within a host buffer
 */
static int is_coherent(struct ds_page_table *pg_tbl, ulong host_addr);

/* Public/exported functions */
/* See ds_page_table.h for description. */
int ds_page_table_init(struct ds_page_table **ppg_tbl,
	const struct ds_bar_data *bar_data,
	const struct ds_page_table_offsets *page_table_offsets,
	uint extended_bit_location, struct device *device, bool has_dma_ops)
{
	ulong bytes;
	ulong total_entries;
	struct ds_page_table *pg_tbl;

	total_entries = readq(&(
		bar_data->virt_base[page_table_offsets->page_table_size_reg]));

	if (total_entries == ULONG_MAX) {
		ds_nodev_error("Error reading page table size. "
			"Initializing page table with size 0.");
		total_entries = 0;
	}

	ds_nodev_info("Attempting to initialize page table of size 0x%lx.",
		total_entries);

	*ppg_tbl = kzalloc(sizeof(**ppg_tbl), GFP_KERNEL);
	if (!*ppg_tbl) {
		ds_nodev_error("No memory for page table.");
		return -ENOMEM;
	}

	pg_tbl = *ppg_tbl;
	bytes = total_entries * sizeof(struct ds_page_table_entry);
	if (bytes != 0) {
		pg_tbl->entries = vmalloc(bytes);
		if (!pg_tbl->entries) {
			ds_nodev_error(
				"No memory for address translation metadata.");
			kfree(pg_tbl);
			*ppg_tbl = NULL;
			return -ENOMEM;
		}
		memset(pg_tbl->entries, 0, bytes);
	}

	mutex_init(&pg_tbl->mutex);
	pg_tbl->total_entries = total_entries;
	pg_tbl->num_simple_entries = total_entries;
	pg_tbl->num_extended_entries = 0;
	pg_tbl->num_active_pages = 0;
	pg_tbl->base_slot = (u64 __iomem *)&(
		bar_data->virt_base[page_table_offsets->page_table_reg]);
	pg_tbl->extended_offset_reg = (u64 __iomem *)&(
		bar_data->virt_base[page_table_offsets
					    ->extended_page_table_reg]);
	pg_tbl->device = device;
	pg_tbl->extended_flag = (1ull << extended_bit_location);
	pg_tbl->dma_ops = has_dma_ops;
	ds_nodev_info("Page table initialized successfully.");

	return 0;
}

/* See ds_page_table.h for description. */
void ds_page_table_cleanup(struct ds_page_table *pg_tbl)
{
	/* Deallocate free second-level tables. */
	ds_page_table_garbage_collect(pg_tbl);

	/* TODO(rspringer): Check that all PTEs have been freed? */

	vfree(pg_tbl->entries);
	pg_tbl->entries = NULL;

	kfree(pg_tbl);
}

/* See ds_page_table.h for description. */
int ds_page_table_partition(
	struct ds_page_table *pg_tbl, uint num_simple_entries)
{
	int i, start;

	mutex_lock(&pg_tbl->mutex);
	if (num_simple_entries > pg_tbl->total_entries) {
		mutex_unlock(&pg_tbl->mutex);
		return -EINVAL;
	}

	ds_page_table_garbage_collect_nolock(pg_tbl);

	start = min(pg_tbl->num_simple_entries, num_simple_entries);

	for (i = start; i < pg_tbl->total_entries; i++) {
		if (pg_tbl->entries[i].status != PTE_FREE) {
			ds_pg_tbl_error(pg_tbl, "entry %d is not free", i);
			mutex_unlock(&pg_tbl->mutex);
			return -EBUSY;
		}
	}

	pg_tbl->num_simple_entries = num_simple_entries;
	pg_tbl->num_extended_entries =
		pg_tbl->total_entries - num_simple_entries;
	writeq(num_simple_entries, pg_tbl->extended_offset_reg);

	mutex_unlock(&pg_tbl->mutex);
	return 0;
}
EXPORT_SYMBOL(ds_page_table_partition);

/*
 * See ds_page_table.h for general description.
 *
 * ds_page_table_map calls either ds_map_simple_pages() or
 * ds_map_extended_pages() to actually perform the mapping.
 *
 * The page table mutex is held for the entire operation.
 */
int ds_page_table_map(struct ds_page_table *pg_tbl, ulong host_addr,
	ulong dev_addr, uint num_pages)
{
	int ret;

	if (!num_pages)
		return 0;

	mutex_lock(&pg_tbl->mutex);

	if (ds_addr_is_simple(pg_tbl, dev_addr)) {

		ret = ds_map_simple_pages(
			pg_tbl, host_addr, dev_addr, num_pages);
	} else
		ret = ds_map_extended_pages(
			pg_tbl, host_addr, dev_addr, num_pages);

	mutex_unlock(&pg_tbl->mutex);

	ds_nodev_debug("ds_page_table_map done: ha %llx daddr %llx num %d "
		       "ret %d\n",
		(unsigned long long int)host_addr,
		(unsigned long long int)dev_addr, num_pages, ret);
	return ret;
}
EXPORT_SYMBOL(ds_page_table_map);

/*
 * See ds_page_table.h for general description.
 *
 * ds_page_table_unmap takes the page table lock and calls either
 * ds_unmap_simple_pages() or ds_unmap_extended_pages() to
 * actually unmap the pages from device space.
 *
 * The page table mutex is held for the entire operation.
 */
void ds_page_table_unmap(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_pages)
{
	if (!num_pages)
		return;

	mutex_lock(&pg_tbl->mutex);
	ds_page_table_unmap_nolock(pg_tbl, dev_addr, num_pages);
	mutex_unlock(&pg_tbl->mutex);
}
EXPORT_SYMBOL(ds_page_table_unmap);

static void ds_page_table_unmap_all_nolock(struct ds_page_table *pg_tbl)
{
	ds_unmap_simple_pages(pg_tbl,
		ds_components_to_dev_address(pg_tbl, 1, 0, 0),
		pg_tbl->num_simple_entries);
	ds_unmap_extended_pages(pg_tbl,
		ds_components_to_dev_address(pg_tbl, 0, 0, 0),
		pg_tbl->num_extended_entries * DS_PAGES_PER_SUBTABLE);
}

/* See ds_page_table.h for description. */
void ds_page_table_unmap_all(struct ds_page_table *pg_tbl)
{
	mutex_lock(&pg_tbl->mutex);
	ds_page_table_unmap_all_nolock(pg_tbl);
	mutex_unlock(&pg_tbl->mutex);
}
EXPORT_SYMBOL(ds_page_table_unmap_all);

/* See ds_page_table.h for description. */
void ds_page_table_reset(struct ds_page_table *pg_tbl)
{
	mutex_lock(&pg_tbl->mutex);
	ds_page_table_unmap_all_nolock(pg_tbl);
	writeq(pg_tbl->total_entries, pg_tbl->extended_offset_reg);
	mutex_unlock(&pg_tbl->mutex);
}

/* See ds_page_table.h for description. */
void ds_page_table_garbage_collect(struct ds_page_table *pg_tbl)
{
	mutex_lock(&pg_tbl->mutex);
	ds_page_table_garbage_collect_nolock(pg_tbl);
	mutex_unlock(&pg_tbl->mutex);
}

/* See ds_page_table.h for description. */
int ds_page_table_lookup_page(struct ds_page_table *pg_tbl, ulong dev_addr,
	struct page **ppage, ulong *poffset)
{
	uint page_num;
	struct ds_page_table_entry *pte;

	mutex_lock(&pg_tbl->mutex);
	if (ds_addr_is_simple(pg_tbl, dev_addr)) {
		page_num = ds_simple_page_idx(pg_tbl, dev_addr);
		if (page_num >= pg_tbl->num_simple_entries)
			goto fail;

		pte = pg_tbl->entries + page_num;
		if (pte->status != PTE_INUSE)
			goto fail;
	} else {
		/* Find the level 0 entry, */
		page_num = ds_extended_lvl0_page_idx(pg_tbl, dev_addr);
		if (page_num >= pg_tbl->num_extended_entries)
			goto fail;

		pte = pg_tbl->entries + pg_tbl->num_simple_entries + page_num;
		if (pte->status != PTE_INUSE)
			goto fail;

		/* and its contained level 1 entry. */
		page_num = ds_extended_lvl1_page_idx(pg_tbl, dev_addr);
		pte = pte->sublevel + page_num;
		if (pte->status != PTE_INUSE)
			goto fail;
	}

	*ppage = pte->page;
	*poffset = pte->offset;
	mutex_unlock(&pg_tbl->mutex);
	return 0;

fail:
	*ppage = NULL;
	*poffset = 0;
	mutex_unlock(&pg_tbl->mutex);
	return -ENOMEM;
}

/* See ds_page_table.h for description. */
int ds_page_table_are_addrs_bad(struct ds_page_table *pg_tbl, ulong host_addr,
	ulong dev_addr, ulong bytes)
{
	if (host_addr & (PAGE_SIZE - 1)) {
		ds_pg_tbl_error(pg_tbl,
			"host mapping address 0x%lx must be page aligned",
			host_addr);
		return 1;
	}

	return ds_page_table_is_dev_addr_bad(pg_tbl, dev_addr, bytes);
}
EXPORT_SYMBOL(ds_page_table_are_addrs_bad);

/* See ds_page_table.h for description. */
int ds_page_table_is_dev_addr_bad(
	struct ds_page_table *pg_tbl, ulong dev_addr, ulong bytes)
{
	int num_pages = bytes / PAGE_SIZE;

	if (bytes & (PAGE_SIZE - 1)) {
		ds_pg_tbl_error(pg_tbl,
			"mapping size 0x%lX must be page aligned", bytes);
		return 1;
	}

	if (num_pages == 0) {
		ds_pg_tbl_error(pg_tbl,
			"requested mapping is less than one page: %lu / %lu",
			bytes, PAGE_SIZE);
		return 1;
	}

	if (ds_addr_is_simple(pg_tbl, dev_addr))
		return ds_is_simple_dev_addr_bad(pg_tbl, dev_addr, num_pages);
	else
		return ds_is_extended_dev_addr_bad(pg_tbl, dev_addr, num_pages);
}
EXPORT_SYMBOL(ds_page_table_is_dev_addr_bad);

/* See ds_page_table.h for description. */
uint ds_page_table_max_size(struct ds_page_table *page_table)
{
	if (!page_table) {
		ds_nodev_error("Passed a null page table.");
		return 0;
	}
	return page_table->total_entries;
}
EXPORT_SYMBOL(ds_page_table_max_size);

/* See ds_page_table.h for description. */
uint ds_page_table_num_entries(struct ds_page_table *pg_tbl)
{
	if (!pg_tbl) {
		ds_nodev_error("Passed a null page table.");
		return 0;
	}

	return pg_tbl->num_simple_entries + pg_tbl->num_extended_entries;
}
EXPORT_SYMBOL(ds_page_table_num_entries);

/* See ds_page_table.h for description. */
uint ds_page_table_num_simple_entries(struct ds_page_table *pg_tbl)
{
	if (!pg_tbl) {
		ds_nodev_error("Passed a null page table.");
		return 0;
	}

	return pg_tbl->num_simple_entries;
}
EXPORT_SYMBOL(ds_page_table_num_simple_entries);

/* See ds_page_table.h for description. */
uint ds_page_table_num_extended_entries(struct ds_page_table *pg_tbl)
{
	if (!pg_tbl) {
		ds_nodev_error("Passed a null page table.");
		return 0;
	}

	return pg_tbl->num_extended_entries;
}
EXPORT_SYMBOL(ds_page_table_num_extended_entries);

uint ds_page_table_num_active_pages(struct ds_page_table *pg_tbl)
{
	if (!pg_tbl) {
		ds_nodev_error("Passed a null page table.");
		return 0;
	}

	return pg_tbl->num_active_pages;
}
EXPORT_SYMBOL(ds_page_table_num_active_pages);

/* See ds_page_table.h */
int ds_page_table_system_status(struct ds_page_table *page_table)
{
	if (!page_table) {
		ds_nodev_error("Passed a null page table.");
		return DS_STATUS_LAMED;
	}

	if (ds_page_table_num_entries(page_table) == 0) {
		ds_nodev_error("Page table size is 0.");
		return DS_STATUS_LAMED;
	}

	return DS_STATUS_ALIVE;
}

/* Internal functions */

/* Mapping functions */
/*
 * ds_map_simple_pages - Allocate and map pages to simple addresses.
 * @pg_tbl: DS page table pointer.
 * @host_addr: Starting host virtual memory address of the pages.
 * @dev_addr: Starting device address of the pages.
 * @cnt: Count of the number of device pages to map.
 *
 * Description: ds_map_simple_pages calls ds_simple_alloc_pages() to allocate
 *		the page table slots, then calls ds_perform_mapping() to
 *		actually do the work of mapping the pages into the the simple
 *		page table (device translation table registers).
 *
 *		The sd_mutex must be held when ds_map_simple_pages() is
 *		called.
 *
 *		Returns 0 if successful or a non-zero error number otherwise.
 *		If there is an error, no pages are mapped.
 */
static int ds_map_simple_pages(struct ds_page_table *pg_tbl, ulong host_addr,
	ulong dev_addr, uint num_pages)
{
	int ret;
	uint slot_idx = ds_simple_page_idx(pg_tbl, dev_addr);

	ret = ds_alloc_simple_entries(pg_tbl, dev_addr, num_pages);
	if (ret) {
		ds_pg_tbl_error(pg_tbl,
			"page table slots %u to %u are not available", slot_idx,
			slot_idx + num_pages - 1);
		return ret;
	}

	ret = ds_perform_mapping(pg_tbl, pg_tbl->entries + slot_idx,
		pg_tbl->base_slot + slot_idx, host_addr, num_pages, 1);

	if (ret) {
		ds_page_table_unmap_nolock(pg_tbl, dev_addr, num_pages);
		ds_pg_tbl_error(pg_tbl, "ds_perform_mapping %d.", ret);
	}
	return ret;
}

/*
 * ds_map_extended_pages - Get and map buffers to extended addresses.
 * @pg_tbl: DS page table pointer.
 * @host_addr: Starting host virtual memory address of the pages.
 * @dev_addr: Starting device address of the pages.
 * @num_pages: The number of device pages to map.
 *
 * Description: ds_map_extended_buffers calls ds_alloc_extended_entries() to
 *		allocate the page table slots, then loops over the level 0
 *		page table entries, and for each calls ds_perform_mapping()
 *		to map the buffers into the level 1 page table for that level
 *		0 entry.
 *
 *		The page table mutex must be held when ds_map_extended_pages()
 *		is called.
 *
 *		Returns 0 if successful or a non-zero error number otherwise.
 *		If there is an error, no pages are mapped.
 */
static int ds_map_extended_pages(struct ds_page_table *pg_tbl, ulong host_addr,
	ulong dev_addr, uint num_pages)
{
	int ret;
	ulong dev_addr_end;
	uint slot_idx, remain, len;
	struct ds_page_table_entry *pte;
	u64 __iomem *slot_base;

	ret = ds_alloc_extended_entries(pg_tbl, dev_addr, num_pages);
	if (ret) {
		dev_addr_end = dev_addr + (num_pages / PAGE_SIZE) - 1;
		ds_pg_tbl_error(pg_tbl,
			"page table slots (%lu,%lu) to (%lu,%lu) are not available",
			ds_extended_lvl0_page_idx(pg_tbl, dev_addr),
			ds_extended_lvl1_page_idx(pg_tbl, dev_addr),
			ds_extended_lvl0_page_idx(pg_tbl, dev_addr_end),
			ds_extended_lvl1_page_idx(pg_tbl, dev_addr_end));
		return ret;
	}

	remain = num_pages;
	slot_idx = ds_extended_lvl1_page_idx(pg_tbl, dev_addr);
	pte = pg_tbl->entries + pg_tbl->num_simple_entries +
	      ds_extended_lvl0_page_idx(pg_tbl, dev_addr);

	while (remain > 0) {
		len = min(remain, DS_PAGES_PER_SUBTABLE - slot_idx);

		slot_base =
			(u64 __iomem *)(page_address(pte->page) + pte->offset);
		ret = ds_perform_mapping(pg_tbl, pte->sublevel + slot_idx,
			slot_base + slot_idx, host_addr, len, 0);
		if (ret) {
			ds_page_table_unmap_nolock(pg_tbl, dev_addr, num_pages);
			return ret;
		}

		remain -= len;
		slot_idx = 0;
		pte++;
		host_addr += len * PAGE_SIZE;
	}

	return 0;
}

/*
 * TODO(vandwalle)
 * dma_map_page() is not plugged properly when runnig under qemu. i.e. dma_ops
 * are not set properly, whihc causes the kernel to assert.
 *
 * This temporary hack allows the driver to work on qemu, but need to be fixed:
 * - either manually set the dma_ops for the architecture (which incidentally
 * can't be done in an out-of-tree module) - or get qemu to fill the device tree
 * properly so as linux plug the proper dma_ops or so as the driver can detect
 * that it is runnig on qemu
 */
static inline dma_addr_t _no_op_dma_map_page(struct device *dev,
	struct page *page, size_t offset, size_t size,
	enum dma_data_direction dir)
{

	/* struct dma_map_ops *ops = get_dma_ops(dev);
	 * dma_addr_t addr;

	 * kmemcheck_mark_initialized(page_address(page) + offset, size);
	 * BUG_ON(!valid_dma_direction(dir));
	 * addr = ops->map_page(dev, page, offset, size, dir, NULL);
	 * debug_dma_map_page(dev, page, offset, size, dir, addr, false);
	 */

	return page_to_phys(page);
}

/*
 * ds_perform_mapping - Get and map last level page table buffers.
 * @pg_tbl: DS page table pointer.
 * @ptes: Array of page table entries to describe this mapping, one per
 *        page to map.
 * @slots: Location(s) to write device-mapped page address. If this is a simple
 *	   mapping, these will be address translation registers. If this is
 *	   an extended mapping, these will be within a second-level page table
 *	   allocated by the host and so must have their __iomem attribute
 *	   casted away.
 * @host_addr: Starting [host] virtual memory address of the buffers.
 * @num_pages: The number of device pages to map.
 * @is_simple_mapping: 1 if this is a simple mapping, 0 otherwise.
 *
 * Description: ds_perform_mapping calls get_user_pages() to get pages
 *		of user memory and pin them.  It then calls dma_map_page() to
 *		map them for DMA.  Finally, the mapped DMA addresses are written
 *		into the page table.
 *
 *		This function expects that the page table entries are
 *		already allocated.  The level argument determines how the
 *		final page table entries are written: either into PCIe memory
 *		mapped space for a level 0 page table or into kernel memory
 *		for a level 1 page table.
 *
 *		The page pointers are saved for later releasing the pages.
 *
 *		Returns 0 if successful or a non-zero error number otherwise.
 */
static int ds_perform_mapping(struct ds_page_table *pg_tbl,
	struct ds_page_table_entry *ptes, u64 __iomem *slots, ulong host_addr,
	uint num_pages, int is_simple_mapping)
{
	int ret;
	ulong offset;
	struct page *page;
	dma_addr_t dma_addr;
	ulong page_addr;
	int i;

	for (i = 0; i < num_pages; i++) {
		page_addr = host_addr + i * PAGE_SIZE;
		offset = page_addr & (PAGE_SIZE - 1);
		ds_nodev_debug("ds_perform_mapping i %d\n", i);
		if (is_coherent(pg_tbl, host_addr)) {

			uint64_t off =
				(uint64_t)host_addr -
				(uint64_t)pg_tbl->coherent_pages[0].user_virt;
			ptes[i].page = 0;
			ptes[i].offset = offset;
			ptes[i].dma_addr = pg_tbl->coherent_pages[0].paddr +
					   off + i * PAGE_SIZE;
		} else {

			ret = get_user_pages_fast(
				page_addr - offset, 1, 1, &page);

			if (ret <= 0) {
				ds_pg_tbl_error(pg_tbl,
					"get user pages failed for addr=0x%lx, "
					"offset=0x%lx [ret=%d]",
					page_addr, offset, ret);
				return ret ? ret : -ENOMEM;
			}
			++pg_tbl->num_active_pages;

			ptes[i].page = page;
			ptes[i].offset = offset;

			/* Map the page into DMA space. */
			if (pg_tbl->dma_ops) {
				/* hook in to kernel map functions */
				ptes[i].dma_addr = dma_map_page(pg_tbl->device,
					page, 0, PAGE_SIZE, DMA_BIDIRECTIONAL);
			} else {
				ptes[i].dma_addr = _no_op_dma_map_page(
					pg_tbl->device, page, 0, PAGE_SIZE,
					DMA_BIDIRECTIONAL);
			}

			ds_nodev_debug("    ds_perform_mapping dev %p i %d "
				       "pte %p pfn %p -> "
				       "mapped %llx\n",
				pg_tbl->device, i, &ptes[i],
				(void *)(page_to_pfn(page)),
				(unsigned long long int)ptes[i].dma_addr);

			if (ptes[i].dma_addr == -1) {
				ds_nodev_error("ds_perform_mapping i %d -> "
					       "fail to map page %llx "
					       "[pfn %p ohys %p]\n",
					i,
					(unsigned long long int)ptes[i]
						.dma_addr,
					(void *)(page_to_pfn(page)),
					(void *)(page_to_phys(page)));
				return -ENOMEM;
			}
			/* Wait until the page is mapped. */
			mb();
		}

		/* Make the DMA-space address available to the device. */
		dma_addr = (ptes[i].dma_addr + offset) | DS_VALID_SLOT_FLAG;

		if (is_simple_mapping) {
			writeq(dma_addr, &slots[i]);
		} else {
			((u64 __force *)slots)[i] = dma_addr;
			/* Extended page table vectors are in DRAM,
			 * and so need to be synced each time they are updated.
			 */
			dma_map_single(pg_tbl->device,
				(void *)&((u64 __force *)slots)[i],
				sizeof(uint64_t), DMA_TO_DEVICE);
		}
		ptes[i].status = PTE_INUSE;
	}
	return 0;
}

/**
 * ds_alloc_simple_entries - Allocate page table entries in a simple table.
 * @pg_tbl: DS page table pointer.
 * @dev_addr: Starting device address for the (eventual) mappings.
 * @num_pages: Count of pages to be mapped.
 *
 * Description: ds_alloc_simple_entries checks to see if a range of page
 *		table slots are available.  As long as the sd_mutex is
 *		held, the slots will be available.
 *
 *		The page table mutex must be held when
 *		ds_alloc_simple entries() is called.
 *
 *		Returns 0 if successful, or non-zero if the requested device
 *		addresses are not available.
 */
static int ds_alloc_simple_entries(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_pages)
{
	if (!ds_is_pte_range_free(
		    pg_tbl->entries + ds_simple_page_idx(pg_tbl, dev_addr),
		    num_pages))
		return -EBUSY;

	return 0;
}

/**
 * ds_alloc_extended_entries - Allocate slots in an extended page table.
 * @pg_tbl: DS page table pointer.
 * @dev_addr: Starting device address for the (eventual) mappings.
 * @num_pages: Count of pages to be mapped.
 *
 * Description: ds_alloc_extended_entries checks to see if a range of page
 *		table slots are available. If necessary, memory is allocated for
 *		second level page tables.
 *
 *		Note that memory for second level page tables is allocated
 *		as needed, but that memory is only freed on the final close
 *		of the device file, when the page tables are repartitioned,
 *		or the the device is removed.  If there is an error or if
 *		the full range of slots is not available, any memory
 *		allocated for second level page tables remains allocated
 *		until final close, repartition, or device removal.
 *
 *		The page table mutex must be held when
 *		ds_alloc_extended_entries() is called.
 *
 *		Returns 0 if successful, or non-zero if the slots are
 *		not available.
 */
static int ds_alloc_extended_entries(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_entries)
{
	int ret = 0;
	uint remain, subtable_slot_idx, len;
	struct ds_page_table_entry *pte;
	u64 __iomem *slot;

	remain = num_entries;
	subtable_slot_idx = ds_extended_lvl1_page_idx(pg_tbl, dev_addr);
	pte = pg_tbl->entries + pg_tbl->num_simple_entries +
	      ds_extended_lvl0_page_idx(pg_tbl, dev_addr);
	slot = pg_tbl->base_slot + pg_tbl->num_simple_entries +
	       ds_extended_lvl0_page_idx(pg_tbl, dev_addr);

	while (remain > 0) {
		len = min(remain, DS_PAGES_PER_SUBTABLE - subtable_slot_idx);

		if (pte->status == PTE_FREE) {
			ret = ds_alloc_extended_subtable(pg_tbl, pte, slot);
			if (ret) {
				ds_pg_tbl_error(pg_tbl,
					"no memory for extended addr subtable");
				return ret;
			}
		} else {
			if (!ds_is_pte_range_free(
				    pte->sublevel + subtable_slot_idx, len))
				return -EBUSY;
		}

		remain -= len;
		subtable_slot_idx = 0;
		pte++;
		slot++;
	}

	return 0;
}

/*
 * ds_alloc_extended_subtable - Allocate a second level page table.
 * @pg_tbl: DS page table pointer.
 * @pte: Extended page table entry under/for which to allocate a second level.
 * @slot: [Device] slot corresponding to pte.
 *
 * Description: Allocate the memory for a second level page table (subtable) at
 *	        the given level 0 entry.  Then call dma_map_page() to map the
 *		second level page table for DMA.  Finally, write the
 *		mapped DMA address into the device page table.
 *
 *		The page table mutex must be held when
 *		ds_alloc_extended_subtable() is called.
 *
 *		Returns 0 if successful, or a non-zero error otherwise.
 */
static int ds_alloc_extended_subtable(struct ds_page_table *pg_tbl,
	struct ds_page_table_entry *pte, u64 __iomem *slot)
{
	ulong page_addr, subtable_bytes;
	dma_addr_t dma_addr;

	/* XXX FIX ME XXX this is inefficient for non-4K page sizes */

	/* GFP_DMA flag must be passed to architectures for which
	 * part of the memory range is not considered DMA'able.
	 * This seems to be the case for Juno board with 4.5.0 Linaro kernel
	 */
	page_addr = get_zeroed_page(GFP_KERNEL | GFP_DMA);
	if (!page_addr)
		return -ENOMEM;
	pte->page = virt_to_page((void *)page_addr);
	pte->offset = 0;

	subtable_bytes =
		sizeof(struct ds_page_table_entry) * DS_PAGES_PER_SUBTABLE;
	pte->sublevel = vmalloc(subtable_bytes);
	if (!pte->sublevel) {
		free_page(page_addr);
		memset(pte, 0, sizeof(struct ds_page_table_entry));
		return -ENOMEM;
	}
	memset(pte->sublevel, 0, subtable_bytes);

	/* Map the page into DMA space. */
	if (pg_tbl->dma_ops) {
		pte->dma_addr = dma_map_page(pg_tbl->device, pte->page, 0,
			PAGE_SIZE, DMA_BIDIRECTIONAL);

	} else {
		pte->dma_addr = _no_op_dma_map_page(pg_tbl->device, pte->page,
			0, PAGE_SIZE, DMA_BIDIRECTIONAL);
	}
	/* Wait until the page is mapped. */
	mb();

	/* make the addresses available to the device */
	dma_addr = (pte->dma_addr + pte->offset) | DS_VALID_SLOT_FLAG;
	writeq(dma_addr, slot);

	pte->status = PTE_INUSE;

	return 0;
}

/* Unmapping functions */
/*
 * ds_page_table_unmap_nolock: Non-locking entry to unmapping routines.
 * @pg_tbl: DS page table structure.
 * @dev_addr: Starting device address of the pages to unmap.
 * @num_pages: The number of device pages to unmap.
 *
 * Description: Version of ds_unmap_pages that assumes the page table lock
 *              is held.
 */
static void ds_page_table_unmap_nolock(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_pages)
{
	if (!num_pages)
		return;

	if (ds_addr_is_simple(pg_tbl, dev_addr))
		ds_unmap_simple_pages(pg_tbl, dev_addr, num_pages);
	else
		ds_unmap_extended_pages(pg_tbl, dev_addr, num_pages);
}
/*
 * ds_unmap_simple_pages - Unmap and release pages mapped to simple addresses.
 * @pg_tbl: DS page table pointer.
 * @dev_addr: Starting device address of the buffers.
 * @num_pages: The number of device pages to unmap.
 *
 * Description: ds_simple_unmap_pages calls ds_perform_unmapping() to unmap
 *		and release the buffers in the level 0 page table.
 *
 *		The sd_mutex must be held when ds_unmap_simple_pages() is
 *		called.
 */
static void ds_unmap_simple_pages(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_pages)
{
	uint slot = ds_simple_page_idx(pg_tbl, dev_addr);

	ds_perform_unmapping(pg_tbl, pg_tbl->entries + slot,
		pg_tbl->base_slot + slot, num_pages, 1);
}

/**
 * ds_unmap_extended_pages - Unmap and release buffers to extended addresses.
 * @pg_tbl: DS page table pointer.
 * @dev_addr: Starting device address of the pages to unmap.
 * @addr: Starting device address of the buffers.
 * @num_pages: The number of device pages to unmap.
 *
 * Description: ds_extended_unmap_pages loops over the level 0 page table
 *		entries, and for each calls ds_perform_unmapping() to unmap
 *		the buffers from the level 1 page [sub]table for that level 0
 *		entry.
 *
 *		The page table mutex must be held when
 *		ds_unmap_extended_pages() is called.
 */
static void ds_unmap_extended_pages(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_pages)
{
	uint slot_idx, remain, len;
	struct ds_page_table_entry *pte;
	u64 __iomem *slot_base;

	remain = num_pages;
	slot_idx = ds_extended_lvl1_page_idx(pg_tbl, dev_addr);
	pte = pg_tbl->entries + pg_tbl->num_simple_entries +
	      ds_extended_lvl0_page_idx(pg_tbl, dev_addr);

	while (remain > 0) {
		/* TODO(rspringer): Add check to ensure pte remains valid? */
		len = min(remain, DS_PAGES_PER_SUBTABLE - slot_idx);

		if (pte->status == PTE_INUSE) {
			slot_base = (u64 __iomem *)(page_address(pte->page) +
						    pte->offset);
			ds_perform_unmapping(pg_tbl, pte->sublevel + slot_idx,
				slot_base + slot_idx, len, 0);
		}

		remain -= len;
		slot_idx = 0;
		pte++;
	}
}

/*
 * ds_perform_unmapping -- Unmap and release mapped pages.
 * @pg_tbl: DS page table pointer.
 * @ptes: Array of page table entries to describe the mapped range, one per
 *        page to unmap.
 * @slots: Device slots corresponding to the mappings described by "ptes".
 *         As with ptes, one element per page to unmap.
 *         If these are simple mappings, these will be address translation
 *         registers. If these are extended mappings, these will be witin a
 *         second-level page table allocated on the host, and so must have
 *	   their __iomem attribute casted away.
 * @num_pages: Number of pages to unmap.
 * @is_simple_mapping: 1 if this is a simple mapping, 0 otherwise.
 *
 * Description: ds_perform_unmapping() loops through the metadata entries
 *		in a last level page table (simple table or extended subtable),
 *		and for each page:
 *		 - Unmaps the page from DMA space (dma_unmap_page),
 *		 - Returns the page to the OS (ds_release_page),
 *		The entry in the page table is written to 0. The metadata
 *		type is set to PTE_FREE and the metadata is all reset
 *		to 0.
 *
 *		The page table mutex must be held when this function is called.
 */
static void ds_perform_unmapping(struct ds_page_table *pg_tbl,
	struct ds_page_table_entry *ptes, u64 __iomem *slots, uint num_pages,
	int is_simple_mapping)
{
	int i;
	/* For each page table entry and corresponding entry in the device's
	 * address translation table:
	 */
	for (i = 0; i < num_pages; i++) {
		/* release the address from the device, */
		if (is_simple_mapping || ptes[i].status == PTE_INUSE)
			writeq(0, &slots[i]);
		else
			((u64 __force *)slots)[i] = 0;
		/* Force sync around the address release. */
		mb();

		/* release the address from the driver, */
		if (ptes[i].status == PTE_INUSE) {

			/* TODO @(mahdih) second condition seems redundant */
			if (ptes[i].dma_addr && ptes[i].page) {
				dma_unmap_page(pg_tbl->device, ptes[i].dma_addr,
					PAGE_SIZE, DMA_FROM_DEVICE);
			}
			if (ds_release_page(ptes[i].page))
				--pg_tbl->num_active_pages;
		}
		ptes[i].status = PTE_FREE;

		/* and clear the PTE. */
		memset(&ptes[i], 0, sizeof(struct ds_page_table_entry));
	}
}

/*
 * ds_free_extended_subtable - Free a second level page [sub]table.
 * @pg_tbl: DS page table pointer.
 * @pte: Page table entry _pointing_to_ the subtable to free.
 * @slot: Device slot holding a pointer to the sublevel's contents.
 *
 * Description: Safely deallocates a second-level [sub]table by:
 *  - Marking the containing first-level PTE as free
 *  - Setting the corresponding [extended] device slot as NULL
 *  - Unmapping the PTE from DMA space.
 *  - Freeing the subtable's memory.
 *  - Deallocating the page and clearing out the PTE.
 *
 * The page table mutex must be held before this call.
 */
static void ds_free_extended_subtable(struct ds_page_table *pg_tbl,
	struct ds_page_table_entry *pte, u64 __iomem *slot)
{
	/* Release the page table from the driver */
	pte->status = PTE_FREE;

	/* Release the page table from the device */
	writeq(0, slot);
	/* Force sync around the address release. */
	mb();

	if (pte->dma_addr)
		dma_unmap_page(pg_tbl->device, pte->dma_addr, PAGE_SIZE,
			DMA_BIDIRECTIONAL);

	vfree(pte->sublevel);

	if (pte->page)
		free_page((ulong)page_address(pte->page));

	memset(pte, 0, sizeof(struct ds_page_table_entry));
}

/*
 * ds_release_page: Safely return a page to the OS.
 * @page: The page to return to the OS.
 * Returns 1 if the page was released, 0 if it was
 * ignored.
 */
static int ds_release_page(struct page *page)
{
	if (!page)
		return 0;

	if (!PageReserved(page))
		SetPageDirty(page);
	put_page(page);

	return 1;
}

/* Evaluates to nonzero if the specified virtual address is simple. */
static inline int ds_addr_is_simple(struct ds_page_table *pg_tbl, ulong addr)
{
	return !((addr) & (pg_tbl)->extended_flag);
}

/*
 * ds_is_simple_dev_addr_bad: Validity checking for simple addresses.
 * @pg_tbl: DS page table pointer.
 * @dev_addr: The device address to which the pages will be mapped.
 * @num_pages: The number of pages in the range to consider.
 *
 * Description: This call verifies that address translation commutes (from
 * address to/from page + offset) and that the requested page range starts and
 * ends within the set of currently-partitioned simple pages.
 */
static int ds_is_simple_dev_addr_bad(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_pages)
{
	uint page_offset = dev_addr & (PAGE_SIZE - 1);
	uint page_index = (dev_addr / PAGE_SIZE) & (pg_tbl->total_entries - 1);

	if (ds_components_to_dev_address(pg_tbl, 1, page_index, page_offset) !=
		dev_addr) {
		ds_pg_tbl_error(pg_tbl, "address is invalid, 0x%lX", dev_addr);
		return 1;
	}

	if (page_index >= pg_tbl->num_simple_entries) {
		ds_pg_tbl_error(pg_tbl,
			"starting slot at %u is too large, max is < %u",
			page_index, pg_tbl->num_simple_entries);
		return 1;
	}

	if (page_index + num_pages > pg_tbl->num_simple_entries) {
		ds_pg_tbl_error(pg_tbl,
			"ending slot at %u is too large, max is <= %u",
			page_index + num_pages, pg_tbl->num_simple_entries);
		return 1;
	}

	return 0;
}

/*
 * ds_is_extended_dev_addr_bad:
 * @pg_tbl: DS page table pointer.
 * @dev_addr: The device address to which the pages will be mapped.
 * @num_pages: The number of second-level/sub pages in the range to consider.
 *
 * Description: This call verifies that address translation commutes (from
 * address to/from page + offset) and that the requested page range starts and
 * ends within the set of currently-partitioned simple pages.
 */
static int ds_is_extended_dev_addr_bad(
	struct ds_page_table *pg_tbl, ulong dev_addr, uint num_pages)
{
	/* Starting byte index of dev_addr into the first mapped page */
	uint page_offset;
	uint page_global_idx, page_lvl0_idx;
	uint num_lvl0_pages;
	ulong addr;

	page_offset = dev_addr & (PAGE_SIZE - 1);

	/* check if the device address is out of bound */
	addr = dev_addr & ~((pg_tbl)->extended_flag);
	if (addr >> (DS_EXTENDED_LVL0_WIDTH + DS_EXTENDED_LVL0_SHIFT)) {
		ds_pg_tbl_error(pg_tbl, "device address out of bound, 0x%p",
			(void *)dev_addr);
		return 1;
	}

	/* Find the starting sub-page index in the space of all sub-pages. */
	page_global_idx = (dev_addr / PAGE_SIZE) &
			  (pg_tbl->total_entries * DS_PAGES_PER_SUBTABLE - 1);

	/* Find the starting level 0 index. */
	page_lvl0_idx = ds_extended_lvl0_page_idx(pg_tbl, dev_addr);

	/* Get the count of affected level 0 pages. */
	num_lvl0_pages =
		(num_pages + DS_PAGES_PER_SUBTABLE - 1) / DS_PAGES_PER_SUBTABLE;

	if (ds_components_to_dev_address(
		    pg_tbl, 0, page_global_idx, page_offset) != dev_addr) {
		ds_pg_tbl_error(
			pg_tbl, "address is invalid, 0x%p", (void *)dev_addr);
		return 1;
	}

	if (page_lvl0_idx >= pg_tbl->num_extended_entries) {
		ds_pg_tbl_error(pg_tbl,
			"starting level 0 slot at %u is too large, max is < %u",
			page_lvl0_idx, pg_tbl->num_extended_entries);
		return 1;
	}

	if (page_lvl0_idx + num_lvl0_pages > pg_tbl->num_extended_entries) {
		ds_pg_tbl_error(pg_tbl,
			"ending level 0 slot at %u is too large, max is <= %u",
			page_lvl0_idx + num_lvl0_pages,
			pg_tbl->num_extended_entries);
		return 1;
	}

	return 0;
}

/*
 * ds_is_pte_range_free: Checks if a range of PTEs is free.
 * @ptes: The set of PTEs to check.
 * @num_entries: The number of PTEs to check.
 *
 * Description: Iterates over the input PTEs to determine if all have been
 * marked as FREE or if any are INUSE. In the former case, 1/true is returned.
 * Otherwise, 0/false is returned.
 *
 * The page table mutex must be held before this call.
 */
static int ds_is_pte_range_free(
	struct ds_page_table_entry *ptes, uint num_entries)
{
	int i;

	for (i = 0; i < num_entries; i++) {
		if (ptes[i].status != PTE_FREE)
			return 0;
	}

	return 1;
}

/*
 * ds_page_table_garbage_collect_nolock: Actually perform collection.
 * @pg_tbl: DS page table structure.
 *
 * Description: Version of ds_page_table_garbage_collect that assumes the page
 *              table lock is held.
 */
static void ds_page_table_garbage_collect_nolock(struct ds_page_table *pg_tbl)
{
	struct ds_page_table_entry *pte;
	u64 __iomem *slot;

	/* XXX FIX ME XXX -- more efficient to keep a usage count */
	/* rather than scanning the second level page tables */

	for (pte = pg_tbl->entries + pg_tbl->num_simple_entries,
	    slot = pg_tbl->base_slot + pg_tbl->num_simple_entries;
		pte < pg_tbl->entries + pg_tbl->total_entries; pte++, slot++) {
		if (pte->status == PTE_INUSE) {
			if (ds_is_pte_range_free(
				    pte->sublevel, DS_PAGES_PER_SUBTABLE))
				ds_free_extended_subtable(pg_tbl, pte, slot);
		}
	}
}

/*
 * ds_components_to_dev_address: Converts components to a device address.
 * @pg_tbl: DS page table structure.
 * @is_simple: nonzero if this should be a simple entry, zero otherwise.
 * @page_index: The page index into the respective table.
 * @offset: The offset within the requested page.
 *
 * Simple utility function to convert (simple, page, offset) into a device
 * address.
 * Examples:
 * Simple page 0, offset 32:
 *  Input (0, 0, 32), Output 0x20
 * Simple page 1000, offset 511:
 *  Input (0, 1000, 512), Output 0x3E81FF
 * Extended page 0, offset 32:
 *  Input (0, 0, 32), Output 0x8000000020
 * Extended page 1000, offset 511:
 *  Input (1, 1000, 512), Output 0x8003E81FF
 */
static ulong ds_components_to_dev_address(struct ds_page_table *pg_tbl,
	int is_simple, uint page_index, uint offset)
{
	ulong lvl0_index, lvl1_index;

	if (is_simple) {
		/* Return simple addresses directly. */
		lvl0_index = page_index & (pg_tbl->total_entries - 1);
		return (lvl0_index << DS_SIMPLE_PAGE_SHIFT) | offset;
	}

	/*
	 * This could be compressed into fewer statements, but
	 * A) the compiler should optimize it
	 * B) this is not slow
	 * C) this is an uncommon operation
	 * D) this is actually readable this way.
	 */
	lvl0_index = page_index / DS_PAGES_PER_SUBTABLE;
	lvl1_index = page_index & (DS_PAGES_PER_SUBTABLE - 1);
	return (pg_tbl)->extended_flag |
	       (lvl0_index << DS_EXTENDED_LVL0_SHIFT) |
	       (lvl1_index << DS_EXTENDED_LVL1_SHIFT) | offset;
}

/*
 * ds_simple_page_idx: Gets the index of the address' page in the simple table.
 * @pg_tbl: DS page table structure.
 * @dev_addr: The address whose page index to retrieve.
 *
 * Description: Treats the input address as a simple address and determines the
 * index of its underlying page in the simple page table (i.e., device address
 * translation registers.
 *
 * Does not perform validity checking.
 */
static int ds_simple_page_idx(struct ds_page_table *pg_tbl, ulong dev_addr)
{
	return (dev_addr >> DS_SIMPLE_PAGE_SHIFT) & (pg_tbl->total_entries - 1);
}

/*
 * ds_extended_lvl0_page_idx: Gets the level 0 page index for the given address.
 * @pg_tbl: DS page table structure.
 * @dev_addr: The address whose page index to retrieve.
 *
 * Description: Treats the input address as an extended address and determines
 * the index of its underlying page in the first-level extended page table
 * (i.e., device extended address translation registers).
 *
 * Does not perform validity checking.
 */
static ulong ds_extended_lvl0_page_idx(
	struct ds_page_table *pg_tbl, ulong dev_addr)
{
	return (dev_addr >> DS_EXTENDED_LVL0_SHIFT) &
	       ((1 << DS_EXTENDED_LVL0_WIDTH) - 1);
}

/*
 * ds_extended_lvl1_page_idx: Gets the level 1 page index for the given address.
 * @pg_tbl: DS page table structure.
 * @dev_addr: The address whose page index to retrieve.
 *
 * Description: Treats the input address as an extended address and determines
 * the index of its underlying page in the second-level extended page table
 * (i.e., host memory pointed to by a first-level page table entry).
 *
 * Does not perform validity checking.
 */
static ulong ds_extended_lvl1_page_idx(
	struct ds_page_table *pg_tbl, ulong dev_addr)
{
	return (dev_addr >> DS_EXTENDED_LVL1_SHIFT) &
	       (DS_PAGES_PER_SUBTABLE - 1);
}

/* is_coherent: Determines whether a host buffer was mapped coherent
 * @pg_tbl: ds_page_table structure tracking the host buffer mapping
 * @host_addr: user virtual address within a host buffer
 *
 * Description: A ds page_table currently support one contiguous
 * dma range, mapped to one contiguous virtual memory range. Check if the
 * host_addr is within start of page 0, and end of last page, for that range.
 */
static int is_coherent(struct ds_page_table *pg_tbl, ulong host_addr)
{
	uint64_t min, max;

	/* whether the host address is within user virt range */
	if (pg_tbl->coherent_pages == NULL)
		return 0;
	min = (uint64_t)pg_tbl->coherent_pages[0].user_virt;
	max = min + PAGE_SIZE * pg_tbl->num_coherent_pages;

	return min <= host_addr && host_addr < max;
}

/*
 * ds_set_user_virt: Records the host_addr to coherent dma memory mapping.
 * @ds_dev: DS Device.
 * @size: Size of the virtual address range to map.
 * @dma_address: Dma address within the coherent memory range.
 * @vma: Virtual address we wish to map to coherent memory.
 *
 * Description: For each page in the virtual address range, record the
 * coherent page mds_pretapping.
 */
int ds_set_user_virt(
	struct ds_dev *ds_dev, uint64_t size, dma_addr_t dma_address, ulong vma)
{
	int j;
	struct ds_page_table *pg_tbl;

	unsigned int num_pages = size / PAGE_SIZE;

	/* TODO: for future chipset, better handling of the case where
	 * multiple page tables are supported on a given Device
	 */
	pg_tbl = ds_dev->page_table[0];
	if (pg_tbl == NULL) {
		ds_nodev_error("ds_set_user_virt: invalid page table index");
		return 0;
	}
	for (j = 0; j < num_pages; j++) {
		pg_tbl->coherent_pages[j].user_virt =
			(uint64_t)vma + j * PAGE_SIZE;
	}
	return 0;
}

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
	dma_addr_t *dma_address, uint64_t index)
{

	dma_addr_t handle;
	void *mem;
	int  j;
	unsigned int num_pages;
	struct ds_driver_desc *driver_desc;

	driver_desc = ds_get_driver_desc(ds_dev);
	num_pages = (size + PAGE_SIZE - 1) / (PAGE_SIZE);
	if (ds_dev->page_table[index] == NULL)
		return -EFAULT;

	if (num_pages == 0)
		return -EINVAL;

	mem = dma_alloc_coherent(ds_get_device(ds_dev), num_pages * PAGE_SIZE,
			&handle, 0);
	if (!mem) {
		ds_nodev_info("alloc coherent failed");
		goto nomem;
	}
	ds_nodev_info("alloc coherent succeed, physical 0x%llx, "
			"virtual 0x%p", handle, mem);
	ds_dev->page_table[index]->num_coherent_pages = num_pages;

	/* allocate the physical memory block */
	ds_dev->page_table[index]->coherent_pages = kzalloc(
		num_pages * sizeof(struct ds_coherent_page_entry), GFP_KERNEL);
	if (!ds_dev->page_table[index]->coherent_pages)
		goto nomem;
	*dma_address = 0;


	ds_dev->coherent_buffer.length_bytes =
		PAGE_SIZE * (num_pages);
	ds_dev->coherent_buffer.phys_base = handle;
	ds_dev->coherent_buffer.virt_base = mem;

	*dma_address = driver_desc->coherent_buffer_description.base;
	for (j = 0; j < num_pages; j++) {
		ds_dev->page_table[index]
			->coherent_pages[j]
			.paddr = handle +  j * PAGE_SIZE;
		ds_dev->page_table[index]
			->coherent_pages[j]
			.kernel_virt =
			(uint64_t)mem +   j * PAGE_SIZE;
	}

	if (*dma_address == 0)
		goto nomem;
	return 0;

nomem:
	if (mem) {
		dma_free_coherent(ds_get_device(ds_dev), num_pages * PAGE_SIZE,
			mem, handle);
	}

	kfree(ds_dev->page_table[index]->coherent_pages);
	ds_dev->page_table[index]->coherent_pages = 0;
	ds_dev->page_table[index]->num_coherent_pages = 0;
	return -ENOMEM;
}

/*
 * ds_free_coherent_memory: free a block of coherent memory.
 * @ds_dev: DS Device.
 * @size: Size of the memory block.
 * @dma_address: Dma address allocated by the kernel.
 * @index: Index of the ds_page_table within this DS device
 *
 * Description: Release memory allocated thru ds_alloc_coherent_memory.
 */
int ds_free_coherent_memory(struct ds_dev *ds_dev, uint64_t size,
	dma_addr_t dma_address, uint64_t index)
{
	struct ds_driver_desc *driver_desc;

	if (ds_dev->page_table[index] == NULL)
		return -EFAULT;

	driver_desc = ds_get_driver_desc(ds_dev);

	if (driver_desc->coherent_buffer_description.base == dma_address) {
		if (ds_dev->coherent_buffer.length_bytes) {
			dma_free_coherent(ds_get_device(ds_dev),
					ds_dev->coherent_buffer.length_bytes,
					ds_dev->coherent_buffer.virt_base,
					ds_dev->coherent_buffer.phys_base
					);
			ds_dev->coherent_buffer.length_bytes = 0;
			ds_dev->coherent_buffer.virt_base = 0;
			ds_dev->coherent_buffer.phys_base = 0;
		}
		return 0;
	}
	return -EADDRNOTAVAIL;
}

/*
 * ds_free_coherent_memory_all: Release all coherent memory.
 * @ds_dev: DS Device.
 * @index: Index of the ds_page_table within this DS device
 *
 * Description: Release all memory allocated thru ds_alloc_coherent_memory.
 */
void ds_free_coherent_memory_all(struct ds_dev *ds_dev, uint64_t index)
{

	if (ds_dev->page_table[index] == NULL)
		return;

	if (ds_dev->coherent_buffer.length_bytes) {
		dma_free_coherent(ds_get_device(ds_dev),
					ds_dev->coherent_buffer.length_bytes,
					ds_dev->coherent_buffer.virt_base,
					ds_dev->coherent_buffer.phys_base);
		ds_dev->coherent_buffer.length_bytes = 0;
		ds_dev->coherent_buffer.virt_base = 0;
		ds_dev->coherent_buffer.phys_base = 0;
	}
}
