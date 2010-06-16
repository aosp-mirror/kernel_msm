/* arch/arm/mach-msm/clock.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2010, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/ctype.h>
#include <linux/pm_qos_params.h>
#include <linux/device.h>
#include <linux/seq_file.h>
#include <mach/clk.h>

#include "clock.h"
#include "proc_comm.h"
#include "clock-7x30.h"

static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clocks_lock);
static HLIST_HEAD(clocks);
struct clk *msm_clocks;
unsigned msm_num_clocks;

/*
 * Bitmap of enabled clocks, excluding ACPU which is always
 * enabled
 */
static DECLARE_BITMAP(clock_map_enabled, NR_CLKS);
static DEFINE_SPINLOCK(clock_map_lock);

static struct clk *clk_allocate_handle(struct clk *sclk)
{
	unsigned long flags;
	struct clk_handle *clkh = kzalloc(sizeof(*clkh), GFP_KERNEL);
	if (!clkh)
		return ERR_PTR(ENOMEM);
	clkh->clk.flags = CLKFLAG_HANDLE;
	clkh->source = sclk;

	spin_lock_irqsave(&clocks_lock, flags);
	hlist_add_head(&clkh->clk.list, &sclk->handles);
	spin_unlock_irqrestore(&clocks_lock, flags);
	return &clkh->clk;
}

static struct clk *source_clk(struct clk *clk)
{
	struct clk_handle *clkh;

	if (clk->flags & CLKFLAG_HANDLE) {
		clkh = container_of(clk, struct clk_handle, clk);
		clk = clkh->source;
	}
	return clk;
}

/*
 * Standard clock functions defined in include/linux/clk.h
 */
struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *clk;
	struct hlist_node *pos;

	mutex_lock(&clocks_mutex);

	hlist_for_each_entry(clk, pos, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == dev)
			goto found_it;

	hlist_for_each_entry(clk, pos, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == NULL)
			goto found_it;

	clk = ERR_PTR(-ENOENT);
found_it:
	if (!IS_ERR(clk) && (clk->flags & CLKFLAG_SHARED))
		clk = clk_allocate_handle(clk);
	mutex_unlock(&clocks_mutex);
	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
	struct clk_handle *clkh;
	unsigned long flags;

	if (WARN_ON(IS_ERR(clk)))
		return;

	if (!(clk->flags & CLKFLAG_HANDLE))
		return;

	clk_set_rate(clk, 0);

	spin_lock_irqsave(&clocks_lock, flags);
	clkh = container_of(clk, struct clk_handle, clk);
	hlist_del(&clk->list);
	kfree(clkh);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&clocks_lock, flags);
	clk = source_clk(clk);
	clk->count++;
	if (clk->count == 1) {
		clk->ops->enable(clk->id);
		spin_lock(&clock_map_lock);
		clock_map_enabled[BIT_WORD(clk->id)] |= BIT_MASK(clk->id);
		spin_unlock(&clock_map_lock);
	}
	spin_unlock_irqrestore(&clocks_lock, flags);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&clocks_lock, flags);
	clk = source_clk(clk);
	BUG_ON(clk->count == 0);
	clk->count--;
	if (clk->count == 0) {
		clk->ops->disable(clk->id);
		spin_lock(&clock_map_lock);
		clock_map_enabled[BIT_WORD(clk->id)] &= ~BIT_MASK(clk->id);
		spin_unlock(&clock_map_lock);
	}
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

int clk_reset(struct clk *clk, enum clk_reset_action action)
{
	if (!clk->ops->reset)
		clk->ops->reset = &pc_clk_reset;
	return clk->ops->reset(clk->remote_id, action);
}
EXPORT_SYMBOL(clk_reset);

unsigned long clk_get_rate(struct clk *clk)
{
	clk = source_clk(clk);
	return clk->ops->get_rate(clk->id);
}
EXPORT_SYMBOL(clk_get_rate);

static unsigned long clk_find_min_rate_locked(struct clk *clk)
{
	unsigned long rate = 0;
	struct clk_handle *clkh;
	struct hlist_node *pos;

	hlist_for_each_entry(clkh, pos, &clk->handles, clk.list)
		if (clkh->rate > rate)
			rate = clkh->rate;
	return rate;
}

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	if (clk->flags & CLKFLAG_HANDLE) {
		struct clk_handle *clkh;
		clkh = container_of(clk, struct clk_handle, clk);
		clkh->rate = rate;
		clk = clkh->source;
		rate = clk_find_min_rate_locked(clk);
	}

	if (clk->flags & CLKFLAG_MAX) {
		ret = clk->ops->set_max_rate(clk->id, rate);
		if (ret)
			goto err;
	}
	if (clk->flags & CLKFLAG_MIN) {
		ret = clk->ops->set_min_rate(clk->id, rate);
		if (ret)
			goto err;
	}

	if (!(clk->flags & (CLKFLAG_MAX | CLKFLAG_MIN)))
		ret = clk->ops->set_rate(clk->id, rate);
err:
	spin_unlock_irqrestore(&clocks_lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->round_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_min_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->set_min_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_min_rate);

int clk_set_max_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->set_max_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_max_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	return -ENOSYS;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	return ERR_PTR(-ENOSYS);
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_flags(struct clk *clk, unsigned long flags)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;
	clk = source_clk(clk);
	return clk->ops->set_flags(clk->id, flags);
}
EXPORT_SYMBOL(clk_set_flags);

void clk_enter_sleep(int from_idle)
{
}

void clk_exit_sleep(void)
{
}

int clks_print_running(void)
{
	struct clk *clk;
	int clk_on_count = 0;
	struct hlist_node *pos;
	char buf[100];
	char *pbuf = buf;
	int size = sizeof(buf);
	int wr;
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);

	hlist_for_each_entry(clk, pos, &clocks, list) {
		if (clk->count) {
			clk_on_count++;
			wr = snprintf(pbuf, size, " %s", clk->name);
			if (wr >= size)
				break;
			pbuf += wr;
			size -= wr;
		}
	}
	if (clk_on_count)
		pr_info("clocks on:%s\n", buf);

	spin_unlock_irqrestore(&clocks_lock, flags);
	return !clk_on_count;
}
EXPORT_SYMBOL(clks_print_running);

static void __init set_clock_ops(struct clk *clk)
{
	if (!clk->ops) {
		clk->ops = &clk_ops_pcom;
		clk->id = clk->remote_id;
	}
}

void __init msm_clock_init(struct clk *clock_tbl, unsigned num_clocks)
{
	unsigned n;

	spin_lock_init(&clocks_lock);
	mutex_lock(&clocks_mutex);
	msm_clocks = clock_tbl;
	msm_num_clocks = num_clocks;
	for (n = 0; n < msm_num_clocks; n++) {
		set_clock_ops(&msm_clocks[n]);
		hlist_add_head(&msm_clocks[n].list, &clocks);
	}
	mutex_unlock(&clocks_mutex);
}

#if defined(CONFIG_DEBUG_FS)
static struct clk *msm_clock_get_nth(unsigned index)
{
	if (index < msm_num_clocks)
		return msm_clocks + index;
	else
		return 0;
}

static int clock_debug_rate_set(void *data, u64 val)
{
	struct clk *clock = data;
	int ret;

	/* Only increases to max rate will succeed, but that's actually good
	 * for debugging purposes. So we don't check for error. */
	if (clock->flags & CLK_MAX)
		clk_set_max_rate(clock, val);
	if (clock->flags & CLK_MIN)
		ret = clk_set_min_rate(clock, val);
	else
		ret = clk_set_rate(clock, val);
	if (ret != 0)
		printk(KERN_ERR "clk_set%s_rate failed (%d)\n",
			(clock->flags & CLK_MIN) ? "_min" : "", ret);
	return ret;
}

static int clock_debug_rate_get(void *data, u64 *val)
{
	struct clk *clock = data;
	*val = clk_get_rate(clock);
	return 0;
}

static int clock_debug_enable_set(void *data, u64 val)
{
	struct clk *clock = data;
	int rc = 0;

	if (val)
		rc = clock->ops->enable(clock->id);
	else
		clock->ops->disable(clock->id);

	return rc;
}

static int clock_debug_enable_get(void *data, u64 *val)
{
	struct clk *clock = data;

	*val = clock->ops->is_enabled(clock->id);

	return 0;
}

static int clock_debug_local_get(void *data, u64 *val)
{
	struct clk *clock = data;

	*val = clock->ops != &clk_ops_pcom;

	return 0;
}

static void *clk_info_seq_start(struct seq_file *seq, loff_t *ppos)
{
	struct hlist_node *pos;
	int i = *ppos;
	mutex_lock(&clocks_mutex);
	hlist_for_each(pos, &clocks)
		if (i-- == 0)
			return hlist_entry(pos, struct clk, list);
	return NULL;
}

static void *clk_info_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct clk *clk = v;
	++*pos;
	return hlist_entry(clk->list.next, struct clk, list);
}

static void clk_info_seq_stop(struct seq_file *seq, void *v)
{
	mutex_unlock(&clocks_mutex);
}

static int clk_info_seq_show(struct seq_file *seq, void *v)
{
	struct clk *clk = v;
	unsigned long flags;
	struct clk_handle *clkh;
	struct hlist_node *pos;

	seq_printf(seq, "Clock %s\n", clk->name);
	seq_printf(seq, "  Id          %d\n", clk->id);
	seq_printf(seq, "  Count       %d\n", clk->count);
	seq_printf(seq, "  Flags       %x\n", clk->flags);
	seq_printf(seq, "  Dev         %p %s\n",
			clk->dev, clk->dev ? dev_name(clk->dev) : "");
	seq_printf(seq, "  Handles     %p\n", clk->handles.first);
	spin_lock_irqsave(&clocks_lock, flags);
	hlist_for_each_entry(clkh, pos, &clk->handles, clk.list)
		seq_printf(seq, "    Requested rate    %ld\n", clkh->rate);
	spin_unlock_irqrestore(&clocks_lock, flags);

	seq_printf(seq, "  Enabled     %d\n", clk->ops->is_enabled(clk->id));
	seq_printf(seq, "  Rate        %ld\n", clk_get_rate(clk));

	seq_printf(seq, "\n");
	return 0;
}

static struct seq_operations clk_info_seqops = {
	.start = clk_info_seq_start,
	.next = clk_info_seq_next,
	.stop = clk_info_seq_stop,
	.show = clk_info_seq_show,
};

static int clk_info_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &clk_info_seqops);
}

static const struct file_operations clk_info_fops = {
	.owner = THIS_MODULE,
	.open = clk_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

DEFINE_SIMPLE_ATTRIBUTE(clock_rate_fops, clock_debug_rate_get,
			clock_debug_rate_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(clock_enable_fops, clock_debug_enable_get,
			clock_debug_enable_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(clock_local_fops, clock_debug_local_get,
			NULL, "%llu\n");

static int __init clock_debug_init(void)
{
	struct dentry *dent_rate, *dent_enable, *dent_local;
	struct clk *clock;
	struct hlist_node *pos;
	char temp[50], *ptr;

	dent_rate = debugfs_create_dir("clk_rate", 0);
	if (IS_ERR(dent_rate))
		return PTR_ERR(dent_rate);

	dent_enable = debugfs_create_dir("clk_enable", 0);
	if (IS_ERR(dent_enable))
		return PTR_ERR(dent_enable);

	dent_local = debugfs_create_dir("clk_local", NULL);
	if (IS_ERR(dent_local))
		return PTR_ERR(dent_local);

	debugfs_create_file("clk_info", 0x444, 0, NULL, &clk_info_fops);

	mutex_lock(&clocks_mutex);
	hlist_for_each_entry(clock, pos, &clocks, list) {
		strncpy(temp, clock->dbg_name, ARRAY_SIZE(temp)-1);
		for (ptr = temp; *ptr; ptr++)
			*ptr = tolower(*ptr);
		debugfs_create_file(temp, 0644, dent_rate,
				    clock, &clock_rate_fops);
		debugfs_create_file(temp, 0644, dent_enable,
				    clock, &clock_enable_fops);
		debugfs_create_file(temp, S_IRUGO, dent_local,
				    clock, &clock_local_fops);
	}
	mutex_unlock(&clocks_mutex);
	return 0;
}

late_initcall(clock_debug_init);
#endif

/* The bootloader and/or AMSS may have left various clocks enabled.
 * Disable any clocks that belong to us (CLKFLAG_AUTO_OFF) but have
 * not been explicitly enabled by a clk_enable() call.
 */
static int __init clock_late_init(void)
{
	unsigned long flags;
	struct clk *clk;
	struct hlist_node *pos;
	unsigned count = 0;

	mutex_lock(&clocks_mutex);
	hlist_for_each_entry(clk, pos, &clocks, list) {
		if (clk->flags & CLKFLAG_AUTO_OFF) {
			spin_lock_irqsave(&clocks_lock, flags);
			if (!clk->count) {
				count++;
				clk->ops->auto_off(clk->id);
			}
			spin_unlock_irqrestore(&clocks_lock, flags);
		}
	}
	mutex_unlock(&clocks_mutex);
	pr_info("clock_late_init() disabled %d unused clocks\n", count);
	return 0;
}

late_initcall(clock_late_init);

