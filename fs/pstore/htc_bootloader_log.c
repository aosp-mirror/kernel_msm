/* Copyright (c) 2013,2017 HTC Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/fsnotify.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/htc_debug_tools.h>
#include <linux/uaccess.h>


#define RAMLOG_COMPATIBLE_NAME "htc,bldr_log"
#define RAMLOG_LAST_RSE_NAME "bl_old_log"
#define RAMLOG_CUR_RSE_NAME "bl_log"

#define BOOT_DEBUG_MAGIC		0xAACCBBDD

/* Header structure defined in LK */
struct bldr_log_header {
	__u32 magic;
	__u32 offset;
	__u32 rotate_flag;
};

static char *bl_last_log_buf, *bl_cur_log_buf;
static unsigned long bl_last_log_buf_size, bl_cur_log_buf_size;

static int bldr_log_check_header(struct bldr_log_header *header,
	unsigned long bldr_log_size)
{
	/* Check magic first */
	if (header->magic == BOOT_DEBUG_MAGIC) {
		/* Check offset range */
		if (header->offset >= sizeof(struct bldr_log_header) &&
		    header->offset <  sizeof(struct bldr_log_header) +
		    bldr_log_size)
			return 1;
	}
	return 0;
}

static void bldr_log_parser(const char *bldr_log, char *bldr_log_buf,
	unsigned long bldr_log_size, unsigned long *bldr_log_buf_size)
{
	struct bldr_log_header *header = (struct bldr_log_header *)bldr_log;

	if (!bldr_log_check_header(header, bldr_log_size)) {
		pr_warn("bldr_log_parser: bldr_log invalid in %p\n", bldr_log);
		return;
	}

	if (header->rotate_flag) {
		char *bldr_log_buf_ptr = bldr_log_buf;
		unsigned long bldr_log_bottom_size, bldr_log_top_size;

		/* Bottom half part  */
		bldr_log_bottom_size = bldr_log_size - header->offset;
		memcpy(bldr_log_buf_ptr, bldr_log + header->offset,
			bldr_log_bottom_size);
		bldr_log_buf_ptr += bldr_log_bottom_size;

		/* Top half part  */
		bldr_log_top_size = header->offset - sizeof(*header);
		memcpy(bldr_log_buf_ptr, bldr_log + sizeof(*header),
			bldr_log_top_size);

		*bldr_log_buf_size = bldr_log_bottom_size + bldr_log_top_size;
	} else {
		*bldr_log_buf_size = header->offset - sizeof(*header);

		memcpy(bldr_log_buf, bldr_log + sizeof(*header),
			*bldr_log_buf_size);
	}

	pr_debug("bldr_log_parser: size %ld\n", *bldr_log_buf_size);
}

ssize_t bldr_last_log_read_once(char __user *userbuf, ssize_t klog_size)
{
	ssize_t len = 0;

	len = (klog_size > bl_last_log_buf_size) ?
		(ssize_t)bl_last_log_buf_size : 0;

	if (len > 0) {
		if (copy_to_user(userbuf, bl_last_log_buf, len)) {
			pr_warn("bldr_last_log_read_once, copy_to_user failed\n");
			return -EFAULT;
		}
	}
	return len;
}

ssize_t bldr_log_read_once(char __user *userbuf, ssize_t klog_size)
{
	ssize_t len = 0;

	len = (klog_size > bl_cur_log_buf_size) ?
		(ssize_t)bl_cur_log_buf_size : 0;

	if (len > 0) {
		if (copy_to_user(userbuf, bl_cur_log_buf, len)) {
			pr_warn("bldr_log_read_once, copy_to_user failed\n");
			return -EFAULT;
		}
	}
	return len;
}

/**
 * Read last bootloader logs, kernel logs, current bootloader logs in order.
 *
 * Handle reads that overlap different regions so the file appears like one
 * contiguous file to the reader.
 */
ssize_t bldr_log_read(const void *lastk_buf, ssize_t lastk_size,
	char __user *userbuf, size_t count, loff_t *ppos)
{
	loff_t pos;
	ssize_t total_len = 0;
	ssize_t len;
	int i;

	struct {
		const char *buf;
		const ssize_t size;
	} log_regions[] = {
		{ .buf = bl_last_log_buf,    .size = bl_last_log_buf_size },
		{ .buf = lastk_buf,	     .size = lastk_size },
		{ .buf = bl_cur_log_buf,     .size = bl_cur_log_buf_size },
	};

	pos = *ppos;
	if (pos < 0)
		return -EINVAL;

	if (!count)
		return 0;

	for (i = 0; i < ARRAY_SIZE(log_regions); ++i) {
		if (pos < log_regions[i].size && log_regions[i].buf != NULL) {
			len = simple_read_from_buffer(userbuf, count, &pos,
				log_regions[i].buf, log_regions[i].size);
			if (len < 0)
				return len;
			count -= len;
			userbuf += len;
			total_len += len;
		}
		pos -= log_regions[i].size;
		if (pos < 0)
			break;
	}

	*ppos += total_len;
	return total_len;
}

ssize_t bldr_log_total_size(void)
{
	return bl_last_log_buf_size + bl_cur_log_buf_size;
}

int bldr_log_setup(phys_addr_t bldr_phy_addr, size_t bldr_log_size,
	bool is_last_bldr)
{
	char *bldr_base;
	int ret = 0;

	if (!bldr_log_size) {
		ret = EINVAL;
		goto _out;
	}

	bldr_base = ioremap(bldr_phy_addr, bldr_log_size);
	if (!bldr_base) {
		pr_warn("%s: failed to map last bootloader log buffer\n",
			__func__);
		ret = -ENOMEM;
		goto _out;
	}

	if (is_last_bldr) {
		bl_last_log_buf = kmalloc(bldr_log_size, GFP_KERNEL);
		if (!bl_last_log_buf) {
			pr_warn("%s: failed to alloc last bootloader log buffer\n",
				__func__);
			goto _unmap;
		} else {
			pr_info("bootloader_log: allocate for last bootloader log, size: %zu\n",
				bldr_log_size);
			bldr_log_parser(bldr_base, bl_last_log_buf,
				bldr_log_size, &bl_last_log_buf_size);
		}
	} else {
		bl_cur_log_buf = kmalloc(bldr_log_size, GFP_KERNEL);
		if (!bl_cur_log_buf) {
			pr_warn("%s: failed to alloc last bootloader log buffer\n",
				__func__);
			goto _unmap;
		} else {
			pr_info("bootloader_log: allocate buffer for bootloader log, size: %zu\n",
				bldr_log_size);
			bldr_log_parser(bldr_base, bl_cur_log_buf,
				bldr_log_size, &bl_cur_log_buf_size);
		}
	}


_unmap:
	iounmap(bldr_base);
_out:
	return ret;
}

int bldr_log_init(void)
{
	struct device_node *np;
	struct resource temp_res;
	int num_reg = 0;

	np = of_find_compatible_node(NULL, NULL, RAMLOG_COMPATIBLE_NAME);

	if (np) {
		while (of_address_to_resource(np, num_reg, &temp_res) == 0) {
			if (!strcmp(temp_res.name, RAMLOG_LAST_RSE_NAME))
				bldr_log_setup(temp_res.start,
					resource_size(&temp_res), true);
			else if (!strcmp(temp_res.name, RAMLOG_CUR_RSE_NAME))
				bldr_log_setup(temp_res.start,
					resource_size(&temp_res), false);
			else
				pr_warn("%s: unknown bldr resource %s\n",
					__func__, temp_res.name);

			num_reg++;
		}
		if (!num_reg)
			pr_warn("%s: can't find address resource\n", __func__);
	} else {
		pr_warn("%s: can't find compatible '%s'\n",
			__func__, RAMLOG_COMPATIBLE_NAME);
	}

	return num_reg;
}

void bldr_log_release(void)
{
	kfree(bl_last_log_buf);
	kfree(bl_cur_log_buf);
}
