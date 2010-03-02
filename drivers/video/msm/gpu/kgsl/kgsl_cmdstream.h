#ifndef __KGSL_CMDSTREAM_H
#define __KGSL_CMDSTREAM_H

#include <linux/msm_kgsl.h>
#include "kgsl_device.h"
#include "kgsl_log.h"

#ifdef KGSL_DEVICE_SHADOW_MEMSTORE_TO_USER
#define KGSL_CMDSTREAM_USE_MEM_TIMESTAMP
#endif /* KGSL_DEVICE_SHADOW_MEMSTORE_TO_USER */

#ifdef KGSL_CMDSTREAM_USE_MEM_TIMESTAMP
#define KGSL_CMDSTREAM_GET_SOP_TIMESTAMP(device, data) 	\
		kgsl_sharedmem_read(&device->memstore, (data),	\
				KGSL_DEVICE_MEMSTORE_OFFSET(soptimestamp), 4)
#else
#define KGSL_CMDSTREAM_GET_SOP_TIMESTAMP(device, data)	\
		kgsl_yamato_regread(device, REG_CP_TIMESTAMP, (data))
#endif /* KGSL_CMDSTREAM_USE_MEM_TIMESTAMP */

#define KGSL_CMDSTREAM_GET_EOP_TIMESTAMP(device, data)	\
		kgsl_sharedmem_read(&device->memstore, (data),	\
				KGSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp), 4)

/* Flags to control command packet settings */
#define KGSL_CMD_FLAGS_PMODE			0x00000001
#define KGSL_CMD_FLAGS_NO_TS_CMP		0x00000002

int kgsl_cmdstream_init(struct kgsl_device *device);

int kgsl_cmdstream_close(struct kgsl_device *device);

void kgsl_cmdstream_memqueue_drain(struct kgsl_device *device);

uint32_t
kgsl_cmdstream_readtimestamp(struct kgsl_device *device,
			     enum kgsl_timestamp_type type);

int kgsl_cmdstream_check_timestamp(struct kgsl_device *device,
				   unsigned int timestamp);

int
kgsl_cmdstream_freememontimestamp(struct kgsl_device *device,
				  struct kgsl_mem_entry *entry,
				  uint32_t timestamp,
				  enum kgsl_timestamp_type type);

static inline bool timestamp_cmp(unsigned int new, unsigned int old)
{
	int ts_diff = new - old;
	return (ts_diff >= 0) || (ts_diff < -20000);
}

#endif /* __KGSL_CMDSTREAM_H */
