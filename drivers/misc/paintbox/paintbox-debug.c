/*
 * Debug support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
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

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "paintbox-bif.h"
#include "paintbox-bus.h"
#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-dma-debug.h"
#include "paintbox-io.h"
#include "paintbox-lbp-debug.h"
#include "paintbox-power.h"
#include "paintbox-stp-debug.h"
#include "paintbox-regs.h"

#define REG_NAME_COLUMN_NUMBER 8
#define REG_VALUE_COLUMN_NUMBER 44

int dump_ipu_vprintf(struct paintbox_data *pb, char *buf, int *written,
		size_t len, const char *format, va_list args)
{
	int ret = 0;

	if (buf && written) {
		ret = vsnprintf(buf + *written, len - *written, format, args);

		if (ret < 0) {
			dev_err(pb->dev, "%s: dump printf error, err = %d",
					__func__, ret);
			return ret;
		}

		*written += ret;
	} else {
		struct va_format vaf;

		vaf.fmt = format;
		vaf.va = &args;

		/* TODO:  Add support for levels */

		pr_info("%pV", &vaf);
	}

	return ret;
}

int dump_ipu_printf(struct paintbox_data *pb, char *buf, int *written,
		size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	if (ret < 0) {
		dev_err(pb->dev, "%s: dump printf error, err = %d",
				__func__, ret);
		return ret;
	}

	return ret;
}

int dump_ipu_register(struct paintbox_data *pb, uint32_t group_base,
		uint32_t reg_offset, const char *reg_name, char *buf,
		int *written, size_t len)
{
	return dump_ipu_printf(pb, buf, written, len,
			"0x%04lx: %-*s0x%016llx\n", group_base + reg_offset,
			REG_VALUE_COLUMN_NUMBER - REG_NAME_COLUMN_NUMBER,
			reg_name ? reg_name : REG_UNUSED,
			paintbox_readq(pb->dev, group_base + reg_offset));
}

int dump_ipu_register_with_value(struct paintbox_data *pb, uint32_t group_base,
		uint32_t reg_offset, uint64_t reg_value, const char *reg_name,
		char *buf, int *written, size_t len)
{
	return dump_ipu_printf(pb, buf, written, len,
			"0x%04lx: %-*s0x%016llx\n", group_base + reg_offset,
			REG_VALUE_COLUMN_NUMBER - REG_NAME_COLUMN_NUMBER,
			reg_name ? reg_name : REG_UNUSED, reg_value);
}

#ifdef CONFIG_PAINTBOX_DEBUG
static int debug_stats_show(struct seq_file *s, void *unused)
{
	struct paintbox_debug *debug = s->private;
	struct paintbox_data *pb = debug->pb;
	char *buf;
	size_t len;
	int ret = 0, written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	if (!debug->stats_dump) {
		mutex_unlock(&pb->lock);
		return 0;
	}

	if (debug->resource_id >= 0)
		written = snprintf(buf, len, "%s%u: ", debug->name,
				debug->resource_id);
	else
		written = snprintf(buf, len, "%s: ", debug->name);
	if (written < 0) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev, "%s: error dumping registers %d", __func__,
				ret);
		return written;
	}

	ret = debug->stats_dump(debug, buf + written, len - written);

	mutex_unlock(&pb->lock);

	if (ret < 0)
		return ret;

	written += ret;

	seq_commit(s, written);

	return 0;
}

static int debug_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_stats_show, inode->i_private);
}

static const struct file_operations debug_stats_fops = {
	.open = debug_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_debug *debug = s->private;
	struct paintbox_data *pb = debug->pb;
	char *buf;
	size_t len;
	int ret, written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	if (debug->resource_id >= 0)
		ret = snprintf(buf, len, "%s%u:\n", debug->name,
				debug->resource_id);
	else
		ret = snprintf(buf, len, "%s:\n", debug->name);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev, "%s: error dumping registers %d", __func__,
				ret);
		return ret;
	}

	written = ret;

	ret = debug->register_dump(debug, buf + written, len - written);

	mutex_unlock(&pb->lock);

	if (ret < 0)
		return ret;

	written += ret;

	seq_commit(s, written);

	return 0;
}

static int debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_regs_show, inode->i_private);
}

static const struct file_operations debug_regs_fops = {
	.open = debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void paintbox_debug_free_entry(struct paintbox_debug *debug)
{
	debugfs_remove(debug->stats_dump_dentry);
	debugfs_remove(debug->reg_dump_dentry);
	debugfs_remove(debug->debug_dir);
}

void paintbox_debug_create_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, struct dentry *debug_root,
		const char *name, unsigned int resource_id,
		register_dump_t register_dump, stats_dump_t stats_dump,
		void *arg)
{
	char resource_name[RESOURCE_NAME_LEN];
	int ret;

	debug->pb = pb;
	debug->name = name;
	debug->resource_id = resource_id;
	debug->register_dump = register_dump;
	debug->stats_dump = stats_dump;

	if (debug->resource_id >= 0)
		ret = snprintf(resource_name, RESOURCE_NAME_LEN, "%s%u", name,
				resource_id);
	else
		ret = snprintf(resource_name, RESOURCE_NAME_LEN, "%s", name);
	if (ret < 0) {
		dev_err(pb->dev, "%s: error creating debugs entry %d\b",
				__func__, ret);
		return;
	}

	debug->debug_dir = debugfs_create_dir(resource_name, debug_root);
	if (IS_ERR(debug->debug_dir)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(debug->debug_dir));
		return;
	}

	debug->reg_dump_dentry = debugfs_create_file("regs", 0640,
			debug->debug_dir, debug, &debug_regs_fops);
	if (IS_ERR(debug->reg_dump_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(debug->reg_dump_dentry));
		return;
	}

	debug->stats_dump_dentry = debugfs_create_file("stats", 0640,
			debug->debug_dir, debug, &debug_stats_fops);
	if (IS_ERR(debug->stats_dump_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(debug->stats_dump_dentry));
		return;
	}
}

static int reg_entry_show(struct seq_file *s, void *p)
{
	struct paintbox_debug_reg_entry *reg_entry = s->private;

	seq_printf(s, "0x%016llx\n", reg_entry->read(reg_entry));
	return 0;
}

static int reg_entry_open(struct inode *inode, struct file *file)
{
	return single_open(file, reg_entry_show, inode->i_private);
}

static ssize_t reg_entry_write(struct file *file, const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct paintbox_debug_reg_entry *reg_entry = s->private;
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;
	uint64_t val;
	int ret;

	ret = kstrtou64_from_user(user_buf, count, 0, &val);
	if (ret < 0) {
		dev_err(pb->dev, "%s: invalid value, err = %d", __func__, ret);
		return ret;
	}

	reg_entry->write(reg_entry, val);

	return count;
}

static const struct file_operations reg_entry_fops = {
	.open = reg_entry_open,
	.write = reg_entry_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

int paintbox_debug_alloc_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, size_t reg_count)
{
	debug->reg_entries = kcalloc(reg_count,
			sizeof(struct paintbox_debug_reg_entry), GFP_KERNEL);
	if (!debug->reg_entries) {
		debug->num_reg_entries = 0;
		return -ENOMEM;
	}

	debug->num_reg_entries = reg_count;
	debug->pb = pb;

	return 0;
}

void paintbox_debug_free_reg_entries(struct paintbox_debug *debug)
{
	unsigned int i;

	for (i = 0; i < debug->num_reg_entries; i++)
		debugfs_remove(debug->reg_entries[i].debug_dentry);

	kfree(debug->reg_entries);

	debug->reg_entries = NULL;
	debug->num_reg_entries = 0;
}

int paintbox_debug_create_reg_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, unsigned int index,
		const char *reg_name, uint32_t reg_offset,
		register_write_t reg_write, register_read_t reg_read)
{
	struct paintbox_debug_reg_entry *reg_entry = &debug->reg_entries[index];

	reg_entry->debug_dentry = debugfs_create_file(reg_name, 0640,
			debug->debug_dir, reg_entry, &reg_entry_fops);
	if (IS_ERR(reg_entry->debug_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(reg_entry->debug_dentry));
		return PTR_ERR(reg_entry->debug_dentry);
	}

	reg_entry->reg_offset = reg_offset;
	reg_entry->write = reg_write;
	reg_entry->read = reg_read;
	reg_entry->debug = debug;

	return 0;
}

void paintbox_debug_create_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, const char **reg_names,
		size_t reg_count, register_write_t reg_write,
		register_read_t reg_read)
{
	int i, ret;

	ret = paintbox_debug_alloc_reg_entries(pb, debug, reg_count);
	if (ret < 0)
		return;

	for (i = 0; i < reg_count; i++) {
		if (!reg_names[i])
			continue;

		ret = paintbox_debug_create_reg_entry(pb, debug, i,
				reg_names[i], i * IPU_REG_WIDTH, reg_write,
				reg_read);
		if (ret < 0) {
			paintbox_debug_free_reg_entries(debug);
			return;
		}
	}
}

#define PB_IOCTL_NAME_ENTRY(c)[_IOC_NR(c)] = #c

static const char *ioctl_names[PB_NUM_IOCTLS] = {
	PB_IOCTL_NAME_ENTRY(PB_GET_IPU_CAPABILITIES),
	PB_IOCTL_NAME_ENTRY(PB_ALLOCATE_DMA_CHANNEL),
	PB_IOCTL_NAME_ENTRY(PB_SETUP_DMA_TRANSFER),
	PB_IOCTL_NAME_ENTRY(PB_START_DMA_TRANSFER),
	PB_IOCTL_NAME_ENTRY(PB_BIND_DMA_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_UNBIND_DMA_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_RELEASE_DMA_CHANNEL),
	PB_IOCTL_NAME_ENTRY(PB_ALLOCATE_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_WAIT_FOR_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_RELEASE_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_ALLOCATE_LINE_BUFFER_POOL),
	PB_IOCTL_NAME_ENTRY(PB_SETUP_LINE_BUFFER),
	PB_IOCTL_NAME_ENTRY(PB_RESET_LINE_BUFFER_POOL),
	PB_IOCTL_NAME_ENTRY(PB_RESET_LINE_BUFFER),
	PB_IOCTL_NAME_ENTRY(PB_RELEASE_LINE_BUFFER_POOL),
	PB_IOCTL_NAME_ENTRY(PB_ALLOCATE_PROCESSOR),
	PB_IOCTL_NAME_ENTRY(PB_SETUP_PROCESSOR),
	PB_IOCTL_NAME_ENTRY(PB_START_PROCESSOR),
	PB_IOCTL_NAME_ENTRY(PB_RELEASE_PROCESSOR),
	PB_IOCTL_NAME_ENTRY(PB_GET_PROCESSOR_IDLE),
	PB_IOCTL_NAME_ENTRY(PB_WAIT_FOR_ALL_PROCESSOR_IDLE),
	PB_IOCTL_NAME_ENTRY(PB_WRITE_LBP_MEMORY),
	PB_IOCTL_NAME_ENTRY(PB_READ_LBP_MEMORY),
	PB_IOCTL_NAME_ENTRY(PB_WRITE_STP_MEMORY),
	PB_IOCTL_NAME_ENTRY(PB_READ_STP_MEMORY),
	PB_IOCTL_NAME_ENTRY(PB_STOP_PROCESSOR),
	PB_IOCTL_NAME_ENTRY(PB_RESUME_PROCESSOR),
	PB_IOCTL_NAME_ENTRY(PB_RESET_PROCESSOR),
	PB_IOCTL_NAME_ENTRY(PB_RESET_ALL_PROCESSORS),
	PB_IOCTL_NAME_ENTRY(PB_GET_ALL_PROCESSOR_STATES),
	PB_IOCTL_NAME_ENTRY(PB_GET_PROGRAM_STATE),
	PB_IOCTL_NAME_ENTRY(PB_WRITE_VECTOR_SRAM_COORDINATES),
	PB_IOCTL_NAME_ENTRY(PB_WRITE_VECTOR_SRAM_REPLICATE),
	PB_IOCTL_NAME_ENTRY(PB_READ_VECTOR_SRAM_COORDINATES),
	PB_IOCTL_NAME_ENTRY(PB_READ_DMA_TRANSFER),
	PB_IOCTL_NAME_ENTRY(PB_ALLOCATE_MIPI_IN_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_RELEASE_MIPI_IN_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_SETUP_MIPI_IN_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_ENABLE_MIPI_IN_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_DISABLE_MIPI_IN_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_GET_MIPI_IN_FRAME_NUMBER),
	PB_IOCTL_NAME_ENTRY(PB_CLEANUP_MIPI_IN_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_ALLOCATE_MIPI_OUT_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_RELEASE_MIPI_OUT_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_SETUP_MIPI_OUT_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_ENABLE_MIPI_OUT_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_DISABLE_MIPI_OUT_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_CLEANUP_MIPI_OUT_STREAM),
	PB_IOCTL_NAME_ENTRY(PB_BIND_MIPI_IN_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_UNBIND_MIPI_IN_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_BIND_MIPI_OUT_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_UNBIND_MIPI_OUT_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_GET_COMPLETED_UNREAD_COUNT),
	PB_IOCTL_NAME_ENTRY(PB_BIND_STP_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_UNBIND_STP_INTERRUPT),
	PB_IOCTL_NAME_ENTRY(PB_STOP_DMA_TRANSFER),
	PB_IOCTL_NAME_ENTRY(PB_FLUSH_DMA_TRANSFERS),
	PB_IOCTL_NAME_ENTRY(PB_STP_PC_HISTOGRAM_ENABLE),
	PB_IOCTL_NAME_ENTRY(PB_STP_PC_HISTOGRAM_READ),
	PB_IOCTL_NAME_ENTRY(PB_STP_PC_HISTOGRAM_CLEAR),
	PB_IOCTL_NAME_ENTRY(PB_FLUSH_INTERRUPTS),
	PB_IOCTL_NAME_ENTRY(PB_FLUSH_ALL_INTERRUPTS),
	PB_IOCTL_NAME_ENTRY(PB_WAIT_FOR_MIPI_INPUT_QUIESCENCE)
};

void paintbox_debug_log_non_ioctl_stats(struct paintbox_data *pb,
		enum non_ioctl_stats_type stats_type, ktime_t start,
		ktime_t end, size_t transfer_len)
{
	struct paintbox_ioctl_stat *entry;
	ktime_t duration;
	unsigned long irqflags = 0;

	if (in_interrupt())
		spin_lock(&pb->stats.stats_lock);
	else
		spin_lock_irqsave(&pb->stats.stats_lock, irqflags);

	duration = ktime_sub(end, start);

	entry = &pb->stats.non_ioctl_entries[stats_type];

	if (ktime_after(duration, entry->max_time))
		entry->max_time = duration;

	if (ktime_before(duration, entry->min_time))
		entry->min_time = duration;

	entry->total_time = ktime_add(entry->total_time, duration);

	if (stats_type == PB_STATS_DMA_MALLOC) {
		if (transfer_len > pb->stats.dma_malloc_max_transfer_len)
			pb->stats.dma_malloc_max_transfer_len = transfer_len;
	}

	entry->count++;

	if (in_interrupt())
		spin_unlock(&pb->stats.stats_lock);
	else
		spin_unlock_irqrestore(&pb->stats.stats_lock, irqflags);
}

void paintbox_debug_log_ioctl_stats(struct paintbox_data *pb, unsigned int cmd,
		ktime_t start, ktime_t end)
{
	struct paintbox_ioctl_stat *entry;
	ktime_t duration;

	if (_IOC_TYPE(cmd) != 'p')
		return;

	mutex_lock(&pb->stats.ioctl_lock);

	duration = ktime_sub(end, start);

	entry = &pb->stats.ioctl_entries[_IOC_NR(cmd)];

	if (ktime_after(duration, entry->max_time))
		entry->max_time = duration;

	if (ktime_before(duration, entry->min_time))
		entry->min_time = duration;

	entry->total_time = ktime_add(entry->total_time, duration);

	entry->count++;

	mutex_unlock(&pb->stats.ioctl_lock);
}

static int paintbox_ioctl_time_stats_show(struct seq_file *s, void *p)
{
	struct paintbox_data *pb = s->private;
	unsigned int i;
	struct paintbox_ioctl_stat *entry;

	seq_printf(s, "probe %lldus\n", ktime_to_us(pb->stats.probe_time));

	entry = &pb->stats.non_ioctl_entries[PB_STATS_OPEN];
	if (entry->count)
		seq_printf(s, "open min %lldns max %lldns avg %lldns tot %lldns cnt %u\n",
				ktime_to_ns(entry->min_time),
				ktime_to_ns(entry->max_time),
				ktime_to_ns(entry->total_time) /
				entry->count,
				ktime_to_ns(entry->total_time),
				entry->count);

	entry = &pb->stats.non_ioctl_entries[PB_STATS_CLOSE];
	if (entry->count)
		seq_printf(s, "close min %lldns max %lldns avg %lldns tot %lldns cnt %u\n",
				ktime_to_ns(entry->min_time),
				ktime_to_ns(entry->max_time),
				ktime_to_ns(entry->total_time) /
				entry->count,
				ktime_to_ns(entry->total_time),
				entry->count);

	entry = &pb->stats.non_ioctl_entries[PB_STATS_IO_INTERRUPT_HANDLE];
	if (entry->count)
		seq_printf(s, "interrupt min %lldns max %lldns avg %lldns tot %lldns cnt %u\n",
				ktime_to_ns(entry->min_time),
				ktime_to_ns(entry->max_time),
				ktime_to_ns(entry->total_time) /
				entry->count,
				ktime_to_ns(entry->total_time),
				entry->count);

	entry = &pb->stats.non_ioctl_entries[PB_STATS_DMA_SETUP];
	if (entry->count)
		seq_printf(s, "setup total min %lldns max %lldns avg %lldns tot %lldns cnt %u\n",
				ktime_to_ns(entry->min_time),
				ktime_to_ns(entry->max_time),
				ktime_to_ns(entry->total_time) /
				entry->count,
				ktime_to_ns(entry->total_time),
				entry->count);

	entry = &pb->stats.non_ioctl_entries[PB_STATS_CACHE_OP];
	if (entry->count)
		seq_printf(s, "\tdma buf map (+cache op) min %lldns max %lldns avg %lldns tot %lldns cnt %u\n",
				ktime_to_ns(entry->min_time),
				ktime_to_ns(entry->max_time),
				ktime_to_ns(entry->total_time) /
				entry->count,
				ktime_to_ns(entry->total_time),
				entry->count);

	entry = &pb->stats.non_ioctl_entries[PB_STATS_DMA_MALLOC];
	if (entry->count)
		seq_printf(s, "\tuser space copy min %lldns max %lldns avg %lldns tot %lldns max len %zu cnt %u\n",
				ktime_to_ns(entry->min_time),
				ktime_to_ns(entry->max_time),
				ktime_to_ns(entry->total_time) /
				entry->count,
				ktime_to_ns(entry->total_time),
				pb->stats.dma_malloc_max_transfer_len,
				entry->count);

	entry = &pb->stats.non_ioctl_entries[PB_STATS_DMA_ENQ];
	if (entry->count)
		seq_printf(s, "transfer queue min %lldns max %lldns avg %lldns tot %lldns cnt %u\n",
				ktime_to_ns(entry->min_time),
				ktime_to_ns(entry->max_time),
				ktime_to_ns(entry->total_time) /
				entry->count,
				ktime_to_ns(entry->total_time),
				entry->count);

	mutex_lock(&pb->stats.ioctl_lock);

	if (!pb->stats.ioctl_entries) {
		mutex_unlock(&pb->stats.ioctl_lock);
		return 0;
	}

	for (i = 0; i < PB_NUM_IOCTLS; i++) {
		entry = &pb->stats.ioctl_entries[i];

		if (entry->count == 0)
			continue;

		if (i == _IOC_NR(PB_WAIT_FOR_INTERRUPT)) {
			entry = &pb->stats.non_ioctl_entries[PB_STATS_WAIT_PRE];
			seq_printf(s, "PB_WAIT_FOR_INTERRUPT(pre) min %lldns max %lldns avg %lldns tot %lldns cnt %u\n",
					ktime_to_ns(entry->min_time),
					ktime_to_ns(entry->max_time),
					ktime_to_ns(entry->total_time) /
					entry->count,
					ktime_to_ns(entry->total_time),
					entry->count);

			entry = &pb->stats.non_ioctl_entries[
					PB_STATS_WAIT_POST];

			seq_printf(s, "PB_WAIT_FOR_INTERRUPT(post) min %lldns max %lldns avg %lldns tot %lldns cnt %u\n",
					ktime_to_ns(entry->min_time),
					ktime_to_ns(entry->max_time),
					ktime_to_ns(entry->total_time) /
					entry->count,
					ktime_to_ns(entry->total_time),
					entry->count);
			continue;
		}

		seq_printf(s, "%s min %lldns max %lldns avg %lldns tot %lldns cnt %u\n",
				ioctl_names[i] ? ioctl_names[i] : "Unknown",
				ktime_to_ns(entry->min_time),
				ktime_to_ns(entry->max_time),
				ktime_to_ns(entry->total_time) / entry->count,
				ktime_to_ns(entry->total_time),
				entry->count);
	}

	mutex_unlock(&pb->stats.ioctl_lock);

	return 0;
}

static int paintbox_ioctl_time_stats_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, paintbox_ioctl_time_stats_show,
			inode->i_private);
}

static ssize_t paintbox_ioctl_time_stats_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct paintbox_data *pb = s->private;
	unsigned int val;
	int ret;

	ret = kstrtouint_from_user(user_buf, count, 0, &val);
	if (ret == 0) {
		mutex_lock(&pb->stats.ioctl_lock);

		if (!pb->stats.ioctl_time_enabled && val == 1) {
			unsigned int i;

			if (WARN_ON(pb->stats.ioctl_entries)) {
				mutex_unlock(&pb->stats.ioctl_lock);
				return -EPERM;
			}

			pb->stats.ioctl_entries = kcalloc(PB_NUM_IOCTLS,
					sizeof(struct paintbox_ioctl_stat),
					GFP_KERNEL);
			if (!pb->stats.ioctl_entries) {
				mutex_unlock(&pb->stats.ioctl_lock);
				return -ENOMEM;
			}

			if (WARN_ON(pb->stats.non_ioctl_entries)) {
				mutex_unlock(&pb->stats.ioctl_lock);
				return -EPERM;
			}

			pb->stats.non_ioctl_entries =
				kcalloc(PB_STATS_NUM_NON_IOCTL_STATS_TYPE,
					sizeof(struct paintbox_ioctl_stat),
					GFP_KERNEL);
			if (!pb->stats.non_ioctl_entries) {
				mutex_unlock(&pb->stats.ioctl_lock);
				return -ENOMEM;
			}

			for (i = 0; i < PB_STATS_NUM_NON_IOCTL_STATS_TYPE; i++)
				pb->stats.non_ioctl_entries[i].min_time =
						ktime_set(KTIME_SEC_MAX, 0);
			pb->stats.dma_malloc_max_transfer_len = 0;

			for (i = 0; i < PB_NUM_IOCTLS; i++)
				pb->stats.ioctl_entries[i].min_time =
						ktime_set(KTIME_SEC_MAX, 0);
			pb->stats.ioctl_time_enabled = true;
		} else if (pb->stats.ioctl_time_enabled && val == 0) {
			pb->stats.ioctl_time_enabled = false;

			kfree(pb->stats.ioctl_entries);
			pb->stats.ioctl_entries = NULL;

			kfree(pb->stats.non_ioctl_entries);
			pb->stats.ioctl_entries = NULL;
		}

		mutex_unlock(&pb->stats.ioctl_lock);

		return count;
	}

	dev_err(pb->dev, "%s: invalid value, err = %d", __func__, ret);

	return ret;
}

static const struct file_operations ioctl_time_stats_fops = {
	.open = paintbox_ioctl_time_stats_open,
	.write = paintbox_ioctl_time_stats_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int paintbox_reg_dump_show(struct seq_file *s, void *unused)
{
	struct paintbox_data *pb = s->private;
	unsigned int i, j;
	char *buf;
	size_t len;
	int ret, written = 0;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);

	ret = paintbox_pm_dump_registers(&pb->power.debug, buf + written,
			len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	ret = paintbox_dump_io_apb_registers(&pb->io.apb_debug, buf + written,
			len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	ret = paintbox_dump_bif_registers(&pb->io.axi_debug, buf + written,
			len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	ret = paintbox_dump_dma_registers(&pb->dma.debug, buf + written,
			len - written);
	if (ret < 0)
		goto err_exit;

	written += ret;

	for (i = 0; i < pb->dma.num_channels; i++) {
		ret = paintbox_dump_dma_channel_registers(
				&pb->dma.channels[i].debug, buf + written,
				len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;
	}

	for (i = 0; i < pb->stp.num_stps; i++) {
		ret = paintbox_dump_stp_registers(&pb->stp.stps[i].debug,
				buf + written, len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;
	}

	for (i = 0; i < pb->lbp.num_lbps; i++) {
		ret = paintbox_dump_lbp_registers(&pb->lbp.lbps[i].debug,
				buf + written, len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;

		for (j = 0; j < pb->lbp.max_lbs; j++) {
			ret = paintbox_dump_lb_registers(
					&pb->lbp.lbps[i].lbs[j].debug,
					buf + written, len - written);
			if (ret < 0)
				goto err_exit;

			written += ret;
		}
	}

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return 0;

err_exit:
	mutex_unlock(&pb->lock);
	dev_err(pb->dev, "%s: register dump error, err = %d", __func__, ret);
	return ret;
}

static int paintbox_reg_dump_open(struct inode *inode, struct file *file)
{
	struct paintbox_data *pb = inode->i_private;
	size_t len;

	/* TODO:  Add V1 support for AON registers */
	len = IO_APB_NUM_REGS * REG_DEBUG_BUFFER_SIZE;
	len += IO_AXI_NUM_REGS * REG_DEBUG_BUFFER_SIZE;
	len += pb->dma.num_channels * DMA_DEBUG_BUFFER_SIZE;
	len += pb->stp.num_stps * STP_NUM_REGS * REG_DEBUG_BUFFER_SIZE;
	len += pb->lbp.num_lbps * LBP_NUM_REGS * REG_DEBUG_BUFFER_SIZE;
	len += pb->lbp.num_lbps * pb->lbp.max_lbs * LB_NUM_REGS *
			REG_DEBUG_BUFFER_SIZE;

	return single_open_size(file, paintbox_reg_dump_show, inode->i_private,
			len);
}

static const struct file_operations reg_dump_fops = {
	.open = paintbox_reg_dump_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void paintbox_debug_init(struct paintbox_data *pb)
{
	pb->debug_root = paintbox_bus_get_debug_root(pb->dev);

	pb->regs_dentry = debugfs_create_file("regs", 0640, pb->debug_root, pb,
			&reg_dump_fops);
	if (IS_ERR(pb->regs_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(pb->regs_dentry));
		return;
	}

	pb->stats.ioctl_time_stats_dentry = debugfs_create_file("driver_stats",
			0640, pb->debug_root, pb, &ioctl_time_stats_fops);
	if (IS_ERR(pb->stats.ioctl_time_stats_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(pb->stats.ioctl_time_stats_dentry));
		return;
	}

	mutex_init(&pb->stats.ioctl_lock);
	spin_lock_init(&pb->stats.stats_lock);
}

void paintbox_debug_remove(struct paintbox_data *pb)
{
	debugfs_remove(pb->stats.ioctl_time_stats_dentry);
	mutex_destroy(&pb->stats.ioctl_lock);

	debugfs_remove(pb->regs_dentry);
}
#endif
