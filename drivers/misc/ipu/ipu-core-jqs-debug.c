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


static int ipu_core_jqs_log_level_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	enum jqs_log_level requested_level = (enum jqs_log_level)val;
	int ret = 0;

	if (requested_level > JQS_LOG_LEVEL_INFO)
		return -ERANGE;

	mutex_lock(&bus->jqs.lock);

	bus->jqs.log_level = requested_level;

	/* If the JQS is ready then commit the new log configuration */
	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_set_log_info(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_log_level_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = (uint64_t)bus->jqs.log_level;

	return 0;
}

static int ipu_core_jqs_trigger_level_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	enum jqs_log_level requested_level = (enum jqs_log_level)val;
	int ret = 0;

	if (requested_level > JQS_LOG_LEVEL_INFO)
		return -ERANGE;

	mutex_lock(&bus->jqs.lock);

	bus->jqs.log_trigger_level = requested_level;

	/* If the JQS is ready then commit the new log configuration */
	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_set_log_info(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_trigger_level_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = (uint64_t)bus->jqs.log_trigger_level;

	return 0;
}

static int ipu_core_jqs_kernel_log_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	int ret = 0;

	mutex_lock(&bus->jqs.lock);

	if (val)
		bus->jqs.log_sink_mask |= JQS_LOG_SINK_MESSAGE;
	else
		bus->jqs.log_sink_mask &= ~JQS_LOG_SINK_MESSAGE;

	/* If the JQS is ready then commit the new log configuration */
	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_set_log_info(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_kernel_log_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = !!(bus->jqs.log_sink_mask & JQS_LOG_SINK_MESSAGE);

	return 0;
}

static int ipu_core_jqs_uart_log_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	int ret = 0;

	mutex_lock(&bus->jqs.lock);

	if (val)
		bus->jqs.log_sink_mask |= JQS_LOG_SINK_UART;
	else
		bus->jqs.log_sink_mask &= ~JQS_LOG_SINK_UART;

	/* If the JQS is ready then commit the new log configuration */
	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_set_log_info(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_uart_log_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = !!(bus->jqs.log_sink_mask & JQS_LOG_SINK_UART);

	return 0;
}

static int ipu_core_jqs_uart_baud_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	int ret = 0;

	mutex_lock(&bus->jqs.lock);

	bus->jqs.uart_baud = (uint32_t)val;

	/* If the JQS is ready then commit the new log configuration */
	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_set_log_info(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_uart_baud_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = bus->jqs.uart_baud;

	return 0;
}

static int ipu_core_jqs_fw_state_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	int ret = 0;

	if (val > JQS_FW_STATUS_RUNNING) {
		dev_err(bus->parent_dev, "%s: invalid value, val = %llu",
				__func__, val);
		return -EINVAL;
	}

	mutex_lock(&bus->jqs.lock);

	/* Check if a higher readiness state was requested. */
	while (val > bus->jqs.status && !ret) {
		switch (bus->jqs.status) {
		case JQS_FW_STATUS_INIT:
			ret = ipu_core_jqs_load_firmware(bus);
			break;
		case JQS_FW_STATUS_REQUESTED:
			ret = ipu_core_jqs_stage_firmware(bus);
			break;
		case JQS_FW_STATUS_STAGED:
			ret = ipu_core_jqs_enable_firmware(bus);
			break;
		case JQS_FW_STATUS_RUNNING:
			break;
		}
	}

	/* Check if a lower readiness state was requested. */
	while (val < bus->jqs.status) {
		switch (bus->jqs.status) {
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
			break;
		}
	}

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_fw_state_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = bus->jqs.status;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_log_level_fops, ipu_core_jqs_log_level_get,
			ipu_core_jqs_log_level_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_log_trigger_level_fops,
			ipu_core_jqs_trigger_level_get,
			ipu_core_jqs_trigger_level_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_kernel_log_fops,
			ipu_core_jqs_kernel_log_get,
			ipu_core_jqs_kernel_log_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_uart_log_fops,
			ipu_core_jqs_uart_log_get,
			ipu_core_jqs_uart_log_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_uart_baud_fops,
			ipu_core_jqs_uart_baud_get,
			ipu_core_jqs_uart_baud_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_fw_state_fops, ipu_core_jqs_fw_state_get,
			ipu_core_jqs_fw_state_set, "%llu\n");

void ipu_core_jqs_debug_init(struct paintbox_bus *bus)
{
	bus->jqs.jqs_dentry = debugfs_create_dir("jqs", bus->debug_root);
	if (IS_ERR(bus->jqs.jqs_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.jqs_dentry));
		return;
	}

	bus->jqs.fw_state_dentry = debugfs_create_file("fw_state", 0640,
			bus->jqs.jqs_dentry, bus, &ipu_core_jqs_fw_state_fops);
	if (IS_ERR(bus->jqs.fw_state_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.fw_state_dentry));
		return;
	}

	bus->jqs.log_level_dentry = debugfs_create_file("log_level", 0640,
			bus->jqs.jqs_dentry, bus, &ipu_core_jqs_log_level_fops);
	if (IS_ERR(bus->jqs.log_level_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.log_level_dentry));
		return;
	}

	bus->jqs.trigger_level_dentry = debugfs_create_file("log_trigger_level",
			0640, bus->jqs.jqs_dentry, bus,
			&ipu_core_jqs_log_trigger_level_fops);
	if (IS_ERR(bus->jqs.trigger_level_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.trigger_level_dentry));
		return;
	}

	bus->jqs.kernel_log_dentry = debugfs_create_file("kernel_log_en", 0640,
			bus->jqs.jqs_dentry, bus,
			&ipu_core_jqs_kernel_log_fops);
	if (IS_ERR(bus->jqs.kernel_log_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.kernel_log_dentry));
		return;
	}

	bus->jqs.uart_log_dentry = debugfs_create_file("uart_log_en", 0640,
			bus->jqs.jqs_dentry, bus, &ipu_core_jqs_uart_log_fops);
	if (IS_ERR(bus->jqs.uart_log_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.uart_log_dentry));
	}

	bus->jqs.uart_baud_dentry = debugfs_create_file("uart_baud", 0640,
			bus->jqs.jqs_dentry, bus, &ipu_core_jqs_uart_baud_fops);
	if (IS_ERR(bus->jqs.uart_baud_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.uart_baud_dentry));
	}
}

void ipu_core_jqs_debug_remove(struct paintbox_bus *bus)
{
	debugfs_remove_recursive(bus->jqs.jqs_dentry);
}
