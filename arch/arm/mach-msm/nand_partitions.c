/* arch/arm/mach-msm/nand_partitions.c
 *
 * Code to extract partition information from ATAG set up by the
 * bootloader.
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/mach/flash.h>
#include <asm/io.h>

#include <asm/setup.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <mach/msm_iomap.h>

#include <mach/board.h>


/* configuration tags specific to msm */

#define ATAG_MSM_PARTITION 0x4d534D70 /* MSMp */

struct msm_ptbl_entry
{
	char name[16];
	__u32 offset;
	__u32 size;
	__u32 flags;
};

#define MSM_MAX_PARTITIONS 8

static struct mtd_partition msm_nand_partitions[MSM_MAX_PARTITIONS];
static char msm_nand_names[MSM_MAX_PARTITIONS * 16];

extern struct flash_platform_data msm_nand_data;

static int __init parse_tag_msm_partition(const struct tag *tag)
{
	struct mtd_partition *ptn = msm_nand_partitions;
	char *name = msm_nand_names;
	struct msm_ptbl_entry *entry = (void *) &tag->u;
	unsigned count, n;
	unsigned have_kpanic = 0;

	count = (tag->hdr.size - 2) /
		(sizeof(struct msm_ptbl_entry) / sizeof(__u32));

	if (count > MSM_MAX_PARTITIONS)
		count = MSM_MAX_PARTITIONS;

	for (n = 0; n < count; n++) {
		memcpy(name, entry->name, 15);
		name[15] = 0;

		if (!strcmp(name, "kpanic"))
			have_kpanic = 1;

		ptn->name = name;
		ptn->offset = entry->offset;
		ptn->size = entry->size;

		name += 16;
		entry++;
		ptn++;
	}

#ifdef CONFIG_VIRTUAL_KPANIC_PARTITION
	if (!have_kpanic) {
		int i;
		uint64_t kpanic_off = 0;

		if (count == MSM_MAX_PARTITIONS) {
			printk("Cannot create virtual 'kpanic' partition\n");
			goto out;
		}

		for (i = 0; i < count; i++) {
			ptn = &msm_nand_partitions[i];
			if (!strcmp(ptn->name, CONFIG_VIRTUAL_KPANIC_SRC)) {
				ptn->size -= CONFIG_VIRTUAL_KPANIC_PSIZE;
				kpanic_off = ptn->offset + ptn->size;
				break;
			}
		}
		if (i == count) {
			printk(KERN_ERR "Partition %s not found\n",
			       CONFIG_VIRTUAL_KPANIC_SRC);
			goto out;
		}

		ptn = &msm_nand_partitions[count];
		ptn->name ="kpanic";
		ptn->offset = kpanic_off;
		ptn->size = CONFIG_VIRTUAL_KPANIC_PSIZE;

		printk("Virtual mtd partition '%s' created @%llx (%llu)\n",
		       ptn->name, ptn->offset, ptn->size);

		count++;
	}
out:
#endif /* CONFIG_VIRTUAL_KPANIC_SRC */
	msm_nand_data.nr_parts = count;
	msm_nand_data.parts = msm_nand_partitions;

	return 0;
}

__tagtable(ATAG_MSM_PARTITION, parse_tag_msm_partition);
