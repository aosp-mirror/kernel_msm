/*
 * (C) Copyright Advanced Micro Devices, Inc. 2002, 2008
 * Copyright (c) 2008-2009 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */
#include <linux/debugfs.h>
#include "kgsl_log.h"
#include "kgsl_ringbuffer.h"
#include "kgsl_device.h"
#include "kgsl.h"

/*default log levels is error for everything*/
#define KGSL_LOG_LEVEL_DEFAULT 3
#define KGSL_LOG_LEVEL_MAX     7
unsigned int kgsl_drv_log = KGSL_LOG_LEVEL_DEFAULT;
unsigned int kgsl_cmd_log = KGSL_LOG_LEVEL_DEFAULT;
unsigned int kgsl_ctxt_log = KGSL_LOG_LEVEL_DEFAULT;
unsigned int kgsl_mem_log = KGSL_LOG_LEVEL_DEFAULT;

unsigned int kgsl_cache_enable;

#ifdef CONFIG_DEBUG_FS
static int kgsl_log_set(unsigned int *log_val, void *data, u64 val)
{
	*log_val = min((unsigned int)val, (unsigned int)KGSL_LOG_LEVEL_MAX);
	return 0;
}

static int kgsl_drv_log_set(void *data, u64 val)
{
	return kgsl_log_set(&kgsl_drv_log, data, val);
}

static int kgsl_drv_log_get(void *data, u64 *val)
{
	*val = kgsl_drv_log;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(kgsl_drv_log_fops, kgsl_drv_log_get,
			kgsl_drv_log_set, "%llu\n");

static int kgsl_cmd_log_set(void *data, u64 val)
{
	return kgsl_log_set(&kgsl_cmd_log, data, val);
}

static int kgsl_cmd_log_get(void *data, u64 *val)
{
	*val = kgsl_cmd_log;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(kgsl_cmd_log_fops, kgsl_cmd_log_get,
			kgsl_cmd_log_set, "%llu\n");

static int kgsl_ctxt_log_set(void *data, u64 val)
{
	return kgsl_log_set(&kgsl_ctxt_log, data, val);
}

static int kgsl_ctxt_log_get(void *data, u64 *val)
{
	*val = kgsl_ctxt_log;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(kgsl_ctxt_log_fops, kgsl_ctxt_log_get,
			kgsl_ctxt_log_set, "%llu\n");

static int kgsl_mem_log_set(void *data, u64 val)
{
	return kgsl_log_set(&kgsl_mem_log, data, val);
}

static int kgsl_mem_log_get(void *data, u64 *val)
{
	*val = kgsl_mem_log;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(kgsl_mem_log_fops, kgsl_mem_log_get,
			kgsl_mem_log_set, "%llu\n");

#ifdef DEBUG
static ssize_t rb_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t rb_regs_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	const int debug_bufmax = 4096;
	static char buffer[4096];
	int n = 0;
	struct kgsl_device *device = NULL;
	struct kgsl_ringbuffer *rb = NULL;
	struct kgsl_rb_debug rb_debug;

	device = &kgsl_driver.yamato_device;

	rb = &device->ringbuffer;

	kgsl_ringbuffer_debug(rb, &rb_debug);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"rbbm_status %08x mem_rptr %08x mem_wptr_poll %08x\n",
			rb_debug.rbbm_status,
			rb_debug.mem_rptr,
			rb_debug.mem_wptr_poll);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"rb_base %08x rb_cntl %08x rb_rptr_addr %08x"
			" rb_rptr %08x rb_rptr_wr %08x\n",
			rb_debug.cp_rb_base,
			rb_debug.cp_rb_cntl,
			rb_debug.cp_rb_rptr_addr,
			rb_debug.cp_rb_rptr,
			rb_debug.cp_rb_rptr_wr);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"rb_wptr %08x rb_wptr_delay %08x rb_wptr_base %08x"
			" ib1_base %08x ib1_bufsz %08x\n",
			rb_debug.cp_rb_wptr,
			rb_debug.cp_rb_wptr_delay,
			rb_debug.cp_rb_wptr_base,
			rb_debug.cp_ib1_base,
			rb_debug.cp_ib1_bufsz);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"ib2_base  %08x ib2_bufsz %08x st_base %08x"
			" st_bufsz %08x cp_me_cntl %08x cp_me_status %08x\n",
			rb_debug.cp_ib2_base,
			rb_debug.cp_ib2_bufsz,
			rb_debug.cp_st_base,
			rb_debug.cp_st_bufsz,
			rb_debug.cp_me_cntl,
			rb_debug.cp_me_status);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"csq_cp_rb %08x csq_cp_ib1 %08x csq_cp_ib2 %08x\n",
			rb_debug.cp_csq_rb_stat,
			rb_debug.cp_csq_ib1_stat,
			rb_debug.cp_csq_ib2_stat);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"cp_debug %08x cp_stat %08x cp_int_status %08x"
			" cp_int_cntl %08x\n",
			rb_debug.cp_debug,
			rb_debug.cp_stat,
			rb_debug.cp_int_status,
			rb_debug.cp_int_cntl);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"sop_timestamp: %0d eop_timestamp: %d\n",
			rb_debug.sop_timestamp,
			rb_debug.eop_timestamp);
	n++;
	buffer[n] = 0;
	return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static struct file_operations kgsl_rb_regs_fops = {
	.read = rb_regs_read,
	.open = rb_regs_open,
};
#endif /*DEBUG*/

#ifdef DEBUG
static ssize_t mmu_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t mmu_regs_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	const int debug_bufmax = 4096;
	static char buffer[4096];
	int n = 0;
	struct kgsl_device *device = NULL;
	struct kgsl_mmu *mmu = NULL;
	struct kgsl_mmu_debug mmu_debug;

	device = &kgsl_driver.yamato_device;

	mmu = &device->mmu;

	kgsl_mmu_debug(mmu, &mmu_debug);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"config %08x mpu_base %08x mpu_end %08x\n",
			mmu_debug.config,
			mmu_debug.mpu_base,
			mmu_debug.mpu_end);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"va_range %08x pt_base %08x\n",
			mmu_debug.va_range,
			mmu_debug.pt_base);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"page_fault %08x trans_error %08x axi_error %08x\n",
			mmu_debug.page_fault,
			mmu_debug.trans_error,
			mmu_debug.axi_error);

	n += scnprintf(buffer + n, debug_bufmax - n,
			"interrupt_mask %08x interrupt_status %08x\n",
			mmu_debug.interrupt_mask,
			mmu_debug.interrupt_status);

	n++;
	buffer[n] = 0;
	return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static struct file_operations kgsl_mmu_regs_fops = {
	.read = mmu_regs_read,
	.open = mmu_regs_open,
};
#endif /*DEBUG*/

#ifdef CONFIG_MSM_KGSL_MMU
static int kgsl_cache_enable_set(void *data, u64 val)
{
	kgsl_cache_enable = (val != 0);
	return 0;
}

static int kgsl_cache_enable_get(void *data, u64 *val)
{
	*val = kgsl_cache_enable;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(kgsl_cache_enable_fops, kgsl_cache_enable_get,
			kgsl_cache_enable_set, "%llu\n");
#endif

#endif /* CONFIG_DEBUG_FS */

int kgsl_debug_init(void)
{
#ifdef CONFIG_DEBUG_FS
	struct dentry *dent;
	dent = debugfs_create_dir("kgsl", 0);
	if (IS_ERR(dent))
		return 0;

	debugfs_create_file("log_level_cmd", 0644, dent, 0,
				&kgsl_cmd_log_fops);
	debugfs_create_file("log_level_ctxt", 0644, dent, 0,
				&kgsl_ctxt_log_fops);
	debugfs_create_file("log_level_drv", 0644, dent, 0,
				&kgsl_drv_log_fops);
	debugfs_create_file("log_level_mem", 0644, dent, 0,
				&kgsl_mem_log_fops);
#ifdef DEBUG
	debugfs_create_file("rb_regs", 0444, dent, 0,
				&kgsl_rb_regs_fops);
#endif

#ifdef DEBUG
	debugfs_create_file("mmu_regs", 0444, dent, 0,
				&kgsl_mmu_regs_fops);
#endif

#ifdef CONFIG_MSM_KGSL_MMU
	debugfs_create_file("cache_enable", 0644, dent, 0,
			    &kgsl_cache_enable_fops);
#endif

#endif /* CONFIG_DEBUG_FS */
	return 0;
}
