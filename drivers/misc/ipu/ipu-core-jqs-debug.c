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

static int ipu_core_jqs_fw_state_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;

	if (val > JQS_FW_STATUS_RUNNING) {
		dev_err(bus->parent_dev, "%s: invalid value, val = %llu",
				__func__, val);
		return -EINVAL;
	}

	/* Check if a higher readiness state was requested. */
	while (val > bus->fw_status) {
		int ret;

		switch (bus->fw_status) {
		case JQS_FW_STATUS_INIT:
			ret = ipu_core_jqs_load_firmware(bus);
			if (ret < 0)
				return ret;
			break;
		case JQS_FW_STATUS_REQUESTED:
			ret = ipu_core_jqs_stage_firmware(bus);
			if (ret < 0)
				return ret;
			break;
		case JQS_FW_STATUS_STAGED:
			ret = ipu_core_jqs_enable_firmware(bus);
			if (ret < 0)
				return ret;
			break;
		case JQS_FW_STATUS_RUNNING:
			return 0;
		}
	}

	/* Check if a lower readiness state was requested. */
	while (val < bus->fw_status) {
		switch (bus->fw_status) {
		case JQS_FW_STATUS_RUNNING:
			ipu_core_jqs_disable_firmware(bus);
			break;
		case JQS_FW_STATUS_STAGED:
			ipu_core_jqs_unstage_firmware(bus);
			break;
		case JQS_FW_STATUS_REQUESTED:
			ipu_core_jqs_unload_firmware(bus);
			break;
		case JQS_FW_STATUS_INIT:
			return 0;
		}
	}

	return 0;
}

static int ipu_core_jqs_fw_state_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = bus->fw_status;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_fw_state_fops, ipu_core_jqs_fw_state_get,
			ipu_core_jqs_fw_state_set, "%llu\n");

void ipu_core_jqs_debug_init(struct paintbox_bus *bus)
{
	bus->fw_state_dentry = debugfs_create_file("fw_state", 0640,
			bus->debug_root, bus, &ipu_core_jqs_fw_state_fops);
	if (IS_ERR(bus->fw_state_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->fw_state_dentry));
	}
}

void ipu_core_jqs_debug_remove(struct paintbox_bus *bus)
{
	debugfs_remove(bus->fw_state_dentry);
}
