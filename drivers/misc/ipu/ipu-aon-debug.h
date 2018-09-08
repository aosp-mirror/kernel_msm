/*
 * AON debug support for the Paintbox programmable IPU
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

#ifndef __IPU_AON_H__
#define __IPU_AON_H__

#include <linux/types.h>

#include "ipu-client.h"

#if IS_ENABLED(CONFIG_IPU_DEBUG)
int ipu_aon_dump_registers(struct paintbox_debug *debug, char *buf,
		size_t len);

void ipu_aon_debug_init(struct paintbox_data *pb);
void ipu_aon_debug_remove(struct paintbox_data *pb);
#else
static inline int ipu_aon_dump_registers(struct paintbox_debug *debug,
		char *buf, size_t len)
{
	return 0;
}

static inline void ipu_aon_debug_init(struct paintbox_data *pb) {}
static inline void ipu_aon_debug_remove(struct paintbox_data *pb) {}
#endif

#endif /* __IPU_AON_H__ */
