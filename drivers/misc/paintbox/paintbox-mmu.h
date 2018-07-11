/*
 * MMU support for the Paintbox programmable IPU
 *
 * Copyright (C) 2017 Google, Inc.
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

#ifndef __PAINTBOX_MMU_H__
#define __PAINTBOX_MMU_H__

#include <linux/kernel.h>
#include <linux/types.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_dump_mmu_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
#endif

void paintbox_mmu_interrupt(struct paintbox_data *pb);

int paintbox_mmu_init(struct paintbox_data *pb);

/* All sessions must be released before remove can be called. */
void paintbox_mmu_remove(struct paintbox_data *pb);

#endif  /* __PAINTBOX_MMU_H__ */
