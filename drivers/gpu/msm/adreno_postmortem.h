/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ADRENO_POSTMORTEM_H
#define __ADRENO_POSTMORTEM_H

struct kgsl_device;

#define IB_LIST_SIZE    64
struct ib_list {
	int count;
	uint32_t bases[IB_LIST_SIZE];
	uint32_t sizes[IB_LIST_SIZE];
	uint32_t offsets[IB_LIST_SIZE];
};

int adreno_postmortem_dump(struct kgsl_device *device, int manual);

void dump_ib(struct kgsl_device *device, char *buffId, uint32_t pt_base,
	uint32_t base_offset, uint32_t ib_base, uint32_t ib_size, bool dump);

void dump_ib1(struct kgsl_device *device, uint32_t pt_base,
			uint32_t base_offset,
			uint32_t ib1_base, uint32_t ib1_size,
			struct ib_list *ib_list, bool dump);

#endif /* __ADRENO_POSTMORTEM_H */
