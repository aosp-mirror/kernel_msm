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
#ifndef __HTC_DEBUG_TOOLS_H__
#define __HTC_DEBUG_TOOLS_H__

#include <linux/types.h>

#if defined(CONFIG_HTC_DEBUG_BOOTLOADER_LOG)
ssize_t bldr_log_read_once(char __user *userbuf, ssize_t klog_size);
ssize_t bldr_last_log_read_once(char __user *userbuf, ssize_t klog_size);
ssize_t bldr_log_read(const void *lastk_buf, ssize_t lastk_size,
	char __user *userbuf, size_t count, loff_t *ppos);
ssize_t bldr_log_total_size(void);
int bldr_log_init(void);
void bldr_log_release(void);
#else
static inline ssize_t bldr_log_read_once(char __user *userbuf,
	ssize_t klog_size) { return 0; }
static inline ssize_t bldr_last_log_read_once(char __user *userbuf,
	ssize_t klog_size) { return 0; }
static inline ssize_t bldr_log_read(const void *lastk_buf, ssize_t lastk_size,
	char __user *userbuf, size_t count, loff_t *ppos) { return 0; }
static inline ssize_t bldr_log_total_size(void) { return 0; }
static inline int bldr_log_init(void) { return 0; }
static inline void bldr_log_release(void) { return; }
#endif


#endif /* __HTC_DEBUG_TOOLS_H__ */
