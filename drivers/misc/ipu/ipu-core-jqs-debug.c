/*
 * JQS management debug support for the Paintbox programmable IPU
 *
 * Copyright (C) 2018 Google, Inc.
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

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/ipu-core.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/types.h>

#include "ipu-core-internal.h"
#include "ipu-core-jqs.h"
#include "ipu-core-jqs-debug.h"


static inline bool ipu_core_jqs_debug_is_running(struct paintbox_bus *bus)
{
	return bus->fw_status == JQS_FW_STATUS_RAM_RUNNING ||
			bus->fw_status == JQS_FW_STATUS_ROM_RUNNING;
}

static int ipu_core_jqs_debug_enable_show(struct seq_file *s, void *p)
{
	struct paintbox_bus *bus = s->private;

	seq_printf(s, "%d\n", ipu_core_jqs_debug_is_running(bus));
	return 0;
}

static int ipu_core_jqs_debug_enable_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, ipu_core_jqs_debug_enable_show,
			inode->i_private);
}

static ssize_t ipu_core_jqs_debug_enable_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct paintbox_bus *bus = s->private;
	int ret, val;

	ret = kstrtoint_from_user(user_buf, count, 0, &val);
	if (ret == 0) {
		if (ipu_core_jqs_debug_is_running(bus) && val == 0)
			ipu_core_jqs_disable_firmware(bus);
		else if (!ipu_core_jqs_debug_is_running(bus) && val == 1)
			ipu_core_jqs_enable_firmware(bus);

		return count;
	}

	dev_err(bus->parent_dev, "%s: invalid value, err = %d", __func__, ret);

	return ret;
}

static const struct file_operations ipu_core_jqs_enable_fops = {
	.open = ipu_core_jqs_debug_enable_open,
	.write = ipu_core_jqs_debug_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

void ipu_core_jqs_debug_init(struct paintbox_bus *bus)
{
	bus->fw_enable_dentry = debugfs_create_file("fw_state", 0640,
			bus->debug_root, bus, &ipu_core_jqs_enable_fops);
	if (IS_ERR(bus->fw_enable_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->fw_enable_dentry));
	}
}

void ipu_core_jqs_debug_remove(struct paintbox_bus *bus)
{
	debugfs_remove(bus->fw_enable_dentry);
	debugfs_remove(bus->fw_enq_dentry);
}
