/*
* Copyright (c) 2008-2009 QUALCOMM USA, INC.
* 
* All source code in this file is licensed under the following license
* 
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, you can find it at http://www.fsf.org
*/

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_cmdstream.h"
#include "kgsl_sharedmem.h"

int kgsl_cmdstream_init(struct kgsl_device *device)
{
	return 0;
}

int kgsl_cmdstream_close(struct kgsl_device *device)
{
	return 0;
}

uint32_t
kgsl_cmdstream_readtimestamp(struct kgsl_device *device,
			     enum kgsl_timestamp_type type)
{
	uint32_t timestamp = 0;

	KGSL_CMD_VDBG("enter (device_id=%d, type=%d)\n", device->id, type);

	if (type == KGSL_TIMESTAMP_CONSUMED)
		KGSL_CMDSTREAM_GET_SOP_TIMESTAMP(device,
						 (unsigned int *)&timestamp);
	else if (type == KGSL_TIMESTAMP_RETIRED)
		KGSL_CMDSTREAM_GET_EOP_TIMESTAMP(device,
						 (unsigned int *)&timestamp);

	KGSL_CMD_VDBG("return %d\n", timestamp);

	return timestamp;
}

int kgsl_cmdstream_check_timestamp(struct kgsl_device *device,
				   unsigned int timestamp)
{
	unsigned int ts_processed;

	ts_processed = kgsl_cmdstream_readtimestamp(device,
						    KGSL_TIMESTAMP_RETIRED);
	return timestamp_cmp(ts_processed, timestamp);
}

void kgsl_cmdstream_memqueue_drain(struct kgsl_device *device)
{
	struct kgsl_mem_entry *entry, *entry_tmp;
	uint32_t ts_processed;
	struct kgsl_ringbuffer *rb = &device->ringbuffer;

	/* get current EOP timestamp */
	ts_processed =
	    kgsl_cmdstream_readtimestamp(device, KGSL_TIMESTAMP_RETIRED);

	list_for_each_entry_safe(entry, entry_tmp, &rb->memqueue, free_list) {
		/*NOTE: this assumes that the free list is sorted by
		 * timestamp, but I'm not yet sure that it is a valid
		 * assumption
		 */
		if (!timestamp_cmp(ts_processed, entry->free_timestamp))
			break;
		KGSL_MEM_DBG("ts_processed %d ts_free %d gpuaddr %x)\n",
			     ts_processed, entry->free_timestamp,
			     entry->memdesc.gpuaddr);
		kgsl_remove_mem_entry(entry);
	}
}

int
kgsl_cmdstream_freememontimestamp(struct kgsl_device *device,
				  struct kgsl_mem_entry *entry,
				  uint32_t timestamp,
				  enum kgsl_timestamp_type type)
{
	struct kgsl_ringbuffer *rb = &device->ringbuffer;
	KGSL_MEM_DBG("enter (dev %p gpuaddr %x ts %d)\n",
		     device, entry->memdesc.gpuaddr, timestamp);
	(void)type;		/* unref. For now just use EOP timestamp */

	list_add_tail(&entry->free_list, &rb->memqueue);
	entry->free_timestamp = timestamp;

	return 0;
}
