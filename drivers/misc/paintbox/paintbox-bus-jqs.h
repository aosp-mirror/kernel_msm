/*
 * JQS management support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_BUS_JQS_H__
#define __PAINTBOX_BUS_JQS_H__

#include <linux/types.h>

#include "paintbox-bus-impl.h"

int paintbox_bus_jqs_enable_firmware(struct paintbox_bus *bus);
void paintbox_bus_jqs_disable_firmware(struct paintbox_bus *bus);

void paintbox_bus_jqs_release(struct paintbox_bus *bus);

#ifdef CONFIG_PAINTBOX_DEBUG
void paintbox_bus_jqs_debug_init(struct paintbox_bus *bus);
void paintbox_bus_jqs_debug_remove(struct paintbox_bus *bus);
#endif

#endif /* __PAINTBOX_BUS_JQS_H__ */
