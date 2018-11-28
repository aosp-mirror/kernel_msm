/*
 * Core JQS management support for the Paintbox programmable IPU
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

#ifndef __IPU_CORE_JQS_H__
#define __IPU_CORE_JQS_H__

#include <linux/ipu-core.h>
#include <linux/types.h>

/* The caller to these functions must hold bus->jqs.lock */
int ipu_core_jqs_enable_firmware(struct paintbox_bus *bus);
void ipu_core_jqs_disable_firmware(struct paintbox_bus *bus, int reason_code);
int ipu_core_jqs_load_firmware(struct paintbox_bus *bus);
void ipu_core_jqs_unload_firmware(struct paintbox_bus *bus);
int ipu_core_jqs_stage_firmware(struct paintbox_bus *bus);
void ipu_core_jqs_unstage_firmware(struct paintbox_bus *bus);

void ipu_core_jqs_enable_clock(struct paintbox_bus *bus,
		uint64_t clock_rate_hz);
void ipu_core_jqs_disable_clock(struct paintbox_bus *bus);

int ipu_core_jqs_send_set_log_info(struct paintbox_bus *bus);

int ipu_core_jqs_init(struct paintbox_bus *bus);
void ipu_core_jqs_remove(struct paintbox_bus *bus);

bool ipu_core_jqs_is_ready(struct paintbox_bus *bus);

#define IPU_CORE_JQS_CLOCK_RATE_SLEEP_OR_SUSPEND 19200000 /* Hz */

static inline void ipu_core_jqs_disable_firmware_normal(
		struct paintbox_bus *bus)
{
		/* Firmware disable requests initiated by the AP are reported as
		 * aborts on any queue that is still active when the disable
		 * request is made.
		 */
		ipu_core_jqs_disable_firmware(bus, -ECONNABORTED);
}

static inline void ipu_core_jqs_disable_firmware_error(
		struct paintbox_bus *bus)
{
		/* JQS or PCIe link failures are reported as connection resets
		 */
		ipu_core_jqs_disable_firmware(bus, -ECONNRESET);
}

#endif /* __IPU_CORE_JQS_H__ */
