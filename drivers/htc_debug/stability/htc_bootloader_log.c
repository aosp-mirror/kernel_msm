/* Copyright (c) 2013, HTC Corporation. All rights reserved.
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
#include <asm/uaccess.h>


#define RAMLOG_COMPATIBLE_NAME "htc,bldr_log"
#define RAMLOG_LAST_RSE_NAME "bl_old_log"
#define RAMLOG_CUR_RSE_NAME "bl_log"
#define BLDR_LAST_TZ_LOG_START_TAG "------ TZBSP DIAG RING BUFF, each line for an individual log ------"
#define BLDR_LAST_TZ_LOG_END_TAG "---------------- END Extracted TZBSP Log --------------------------"

#define BOOT_DEBUG_MAGIC		0xAACCBBDD

/* Header structure defined in LK */
struct bldr_log_header {
	uint32_t magic;
	uint32_t offset;
	uint32_t rotate_flag;
};

char *bl_last_log_buf, *bl_cur_log_buf;
char *bl_last_tz_log_buf, *uart_status_buf;
unsigned long bl_last_log_buf_size, bl_cur_log_buf_size;
size_t bl_last_tz_log_buf_size, uart_status_buf_size;
bool uart_on = false;
static const char *uart_on_str ="\nConsole is ENABLED";
static const char *uart_off_str ="\nConsole is DISABLED";

static int bldr_log_check_header(struct bldr_log_header *header, unsigned long bldr_log_size)
{
	/* Check magic first */
	if (header->magic == BOOT_DEBUG_MAGIC) {
		/* Check offset range */
		if (header->offset >= sizeof(struct bldr_log_header) &&
		    header->offset <  sizeof(struct bldr_log_header) + bldr_log_size)
			return 1;
	}
	return 0;
}

static void bldr_log_parser(const char *bldr_log, char *bldr_log_buf, unsigned long bldr_log_size, unsigned long *bldr_log_buf_size)
{
	struct bldr_log_header *header = (struct bldr_log_header *)bldr_log;

	if (!bldr_log_check_header(header, bldr_log_size))
	{
		pr_warn("bldr_log_parser: bldr_log is invalid in %p\n", bldr_log);
		return;
	}

	if (header->rotate_flag)
	{
		char *bldr_log_buf_ptr = bldr_log_buf;
		unsigned long bldr_log_bottom_size, bldr_log_top_size;

		/* Bottom half part  */
		bldr_log_bottom_size = bldr_log_size - header->offset;
		memcpy(bldr_log_buf_ptr, bldr_log + header->offset, bldr_log_bottom_size);
		bldr_log_buf_ptr += bldr_log_bottom_size;

		/* Top half part  */
		bldr_log_top_size = header->offset - sizeof(struct bldr_log_header);
		memcpy(bldr_log_buf_ptr, bldr_log + sizeof(struct bldr_log_header), bldr_log_top_size);

		*bldr_log_buf_size = bldr_log_bottom_size + bldr_log_top_size;
	}
	else
	{
		*bldr_log_buf_size = header->offset - sizeof(struct bldr_log_header);

		memcpy(bldr_log_buf, bldr_log + sizeof(struct bldr_log_header), *bldr_log_buf_size);
	}

	pr_info("bldr_log_parser: size %ld\n", *bldr_log_buf_size);
}

ssize_t bldr_last_log_read_once(char __user *userbuf, ssize_t klog_size)
{
	ssize_t len = 0;

	len = (klog_size > bl_last_log_buf_size)? (ssize_t)bl_last_log_buf_size : 0;

	if (0 < len) {
		if (copy_to_user(userbuf, bl_last_log_buf, len)) {
			pr_warn("bldr_last_log_read_once, copy_to_user failed\n");
			return 0;
		}
	}
	return len;
}

ssize_t bldr_log_read_once(char __user *userbuf, ssize_t klog_size)
{
	ssize_t len = 0;

	len = (klog_size > bl_cur_log_buf_size)? (ssize_t)bl_cur_log_buf_size : 0;

	if (0 < len) {
		if(copy_to_user(userbuf, bl_cur_log_buf, len)) {
			pr_warn("bldr_log_read_once, copy_to_user failed\n");
			return 0;
		}
	}
	return len;
}

/**
 * Update uart_on flag when finding "uart=1" in kernel command line
 */
static int __init uarton_status(char *str)
{
	uart_on = true;
	return 1;
}
__setup("uart=1", uarton_status);

/**
 * Init the value of uart_status_buf.
 *
 * uart on, set "Console is ENABLED"
 * uart off, set "Console is DISABLED"
 */
static int uart_status_setup(void)
{
	const char *uart_status_str;

	if (uart_on) {
		uart_status_str = uart_on_str;
	} else {
		uart_status_str = uart_off_str;
	}

	uart_status_buf_size = strlen(uart_status_str) + 1;
	uart_status_buf = kmalloc(uart_status_buf_size, GFP_KERNEL);
	if (!uart_status_buf) {
		uart_status_buf_size = 0;
		pr_warn("%s: failed to alloc uart status buffer\n", __func__);
		return -ENOMEM;
	}
	memcpy(uart_status_buf, uart_status_str, uart_status_buf_size);
	return 0;
}

/**
 * Read last bootloader logs, current bootloader logs, kernel logs,
 * last bootloader TZ logs, uart status in that order.
 *
 * Handle reads that overlap different regions so the file appears like one
 * file to the reader.
 */
ssize_t bldr_log_read(const void *lastk_buf, ssize_t lastk_size, char __user *userbuf,
						size_t count, loff_t *ppos)
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
		{ .buf = bl_cur_log_buf,     .size = bl_cur_log_buf_size, },
		{ .buf = lastk_buf,	     .size = lastk_size },
		{ .buf = bl_last_tz_log_buf, .size = bl_last_tz_log_buf_size },
		{ .buf = uart_status_buf,    .size = uart_status_buf_size }
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

int bldr_log_setup(phys_addr_t bldr_phy_addr, size_t bldr_log_size, bool is_last_bldr)
{
	char *bldr_base;
	int ret = 0;
	char *bl_last_tz_log_start;
	char *bl_last_tz_log_end;

	if (!bldr_log_size) {
		ret = EINVAL;
		goto _out;
	}

	bldr_base = ioremap(bldr_phy_addr, bldr_log_size);
	if (!bldr_base) {
		pr_warn("%s: failed to map last bootloader log buffer\n", __func__);
		ret = -ENOMEM;
		goto _out;
	}

	if (is_last_bldr) {
		bl_last_log_buf = kmalloc(bldr_log_size, GFP_KERNEL);

		if (!bl_last_log_buf) {
			pr_warn("%s: failed to alloc last bootloader log buffer\n", __func__);
			ret = -ENOMEM;
			goto _unmap;
		} else {
			pr_info("bootloader_log: allocate buffer for last bootloader log, size: %zu\n", bldr_log_size);
			bldr_log_parser(bldr_base, bl_last_log_buf, bldr_log_size, &bl_last_log_buf_size);

			bl_last_tz_log_start = strnstr(bl_last_log_buf,
				BLDR_LAST_TZ_LOG_START_TAG, bldr_log_size);
			if (bl_last_tz_log_start == NULL)
				goto _unmap;

			bl_last_tz_log_end = strnstr(bl_last_log_buf,
				BLDR_LAST_TZ_LOG_END_TAG, bldr_log_size);
			if (bl_last_tz_log_end == NULL)
				goto _unmap;

			bl_last_tz_log_end += strlen(BLDR_LAST_TZ_LOG_END_TAG);
			if (bl_last_tz_log_start >= bl_last_tz_log_end)
				goto _unmap;

			bl_last_tz_log_buf_size = bl_last_tz_log_end - bl_last_tz_log_start;
			bl_last_tz_log_buf = kmalloc(bl_last_tz_log_buf_size, GFP_KERNEL);
			if (!bl_last_tz_log_buf) {
				bl_last_tz_log_buf_size = 0;
				goto _unmap;
			}

			memcpy(bl_last_tz_log_buf, bl_last_tz_log_start,
			       bl_last_tz_log_buf_size);
		}
	} else {
		bl_cur_log_buf = kmalloc(bldr_log_size, GFP_KERNEL);
		if (!bl_cur_log_buf) {
			pr_warn("%s: failed to alloc last bootloader log buffer\n", __func__);
			goto _unmap;
		} else {
			pr_info("bootloader_log: allocate buffer for bootloader log, size: %zu\n", bldr_log_size);
			bldr_log_parser(bldr_base, bl_cur_log_buf, bldr_log_size, &bl_cur_log_buf_size);
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
				bldr_log_setup(temp_res.start, resource_size(&temp_res), true);
			else if (!strcmp(temp_res.name, RAMLOG_CUR_RSE_NAME))
				bldr_log_setup(temp_res.start, resource_size(&temp_res), false);
			else
				pr_warn("%s: unknown bldr resource %s\n", __func__, temp_res.name);

			num_reg++;
		}
		if (!num_reg)
			pr_warn("%s: can't find address resource\n", __func__);

		uart_status_setup();
	} else
		pr_warn("%s: can't find compatible '%s'\n", __func__, RAMLOG_COMPATIBLE_NAME);

	return num_reg;
}

void bldr_log_release(void)
{
	if (bl_last_log_buf)
		kfree(bl_last_log_buf);

	if (bl_cur_log_buf)
		kfree(bl_cur_log_buf);

	kfree(uart_status_buf);
}
