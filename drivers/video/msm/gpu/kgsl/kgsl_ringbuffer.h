/*
 * (C) Copyright Advanced Micro Devices, Inc. 2002, 2007
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
#ifndef __GSL_RINGBUFFER_H
#define __GSL_RINGBUFFER_H

#include <linux/types.h>
#include <linux/msm_kgsl.h>
#include <linux/mutex.h>
#include "kgsl_log.h"
#include "kgsl_sharedmem.h"
#include "yamato_reg.h"

#define GSL_STATS_RINGBUFFER

#define GSL_RB_USE_MEM_RPTR
#define GSL_RB_USE_MEM_TIMESTAMP
#define GSL_DEVICE_SHADOW_MEMSTORE_TO_USER

/* ringbuffer sizes log2quadword */
#define GSL_RB_SIZE_8	 	0
#define GSL_RB_SIZE_16		1
#define GSL_RB_SIZE_32		2
#define GSL_RB_SIZE_64		3
#define GSL_RB_SIZE_128		4
#define GSL_RB_SIZE_256		5
#define GSL_RB_SIZE_512		6
#define GSL_RB_SIZE_1K  	7
#define GSL_RB_SIZE_2K  	8
#define GSL_RB_SIZE_4K  	9
#define GSL_RB_SIZE_8K  	10
#define GSL_RB_SIZE_16K 	11
#define GSL_RB_SIZE_32K 	12
#define GSL_RB_SIZE_64K 	13
#define GSL_RB_SIZE_128K	14
#define GSL_RB_SIZE_256K	15
#define GSL_RB_SIZE_512K	16
#define GSL_RB_SIZE_1M		17
#define GSL_RB_SIZE_2M		18
#define GSL_RB_SIZE_4M		19

/* Yamato ringbuffer config*/
static const unsigned int kgsl_cfg_rb_sizelog2quadwords = GSL_RB_SIZE_32K;
static const unsigned int kgsl_cfg_rb_blksizequadwords  = GSL_RB_SIZE_16;

/* CP timestamp register */
#define	REG_CP_TIMESTAMP		 REG_SCRATCH_REG0


struct kgsl_device;
struct kgsl_drawctxt;
struct kgsl_ringbuffer;

struct kgsl_rb_debug {
	unsigned int pm4_ucode_rel;
	unsigned int pfp_ucode_rel;
	unsigned int mem_wptr_poll;
	unsigned int mem_rptr;
	unsigned int cp_rb_base;
	unsigned int cp_rb_cntl;
	unsigned int cp_rb_rptr_addr;
	unsigned int cp_rb_rptr;
	unsigned int cp_rb_rptr_wr;
	unsigned int cp_rb_wptr;
	unsigned int cp_rb_wptr_delay;
	unsigned int cp_rb_wptr_base;
	unsigned int cp_ib1_base;
	unsigned int cp_ib1_bufsz;
	unsigned int cp_ib2_base;
	unsigned int cp_ib2_bufsz;
	unsigned int cp_st_base;
	unsigned int cp_st_bufsz;
	unsigned int cp_csq_rb_stat;
	unsigned int cp_csq_ib1_stat;
	unsigned int cp_csq_ib2_stat;
	unsigned int scratch_umsk;
	unsigned int scratch_addr;
	unsigned int cp_me_cntl;
	unsigned int cp_me_status;
	unsigned int cp_debug;
	unsigned int cp_stat;
	unsigned int cp_int_status;
	unsigned int cp_int_cntl;
	unsigned int rbbm_status;
	unsigned int rbbm_int_status;
	unsigned int sop_timestamp;
	unsigned int eop_timestamp;
};
#ifdef DEBUG
void kgsl_ringbuffer_debug(struct kgsl_ringbuffer *rb,
				struct kgsl_rb_debug *rb_debug);

void kgsl_ringbuffer_dump(struct kgsl_ringbuffer *rb);
#else
static inline void kgsl_ringbuffer_debug(struct kgsl_ringbuffer *rb,
					struct kgsl_rb_debug *rb_debug)
{
}

static inline void kgsl_ringbuffer_dump(struct kgsl_ringbuffer *rb)
{
}
#endif

struct kgsl_rbwatchdog {
	uint32_t   flags;
	unsigned int  rptr_sample;
};

#define GSL_RB_MEMPTRS_SCRATCH_COUNT	 8
struct kgsl_rbmemptrs {
	volatile int  rptr;
	volatile int  wptr_poll;
} __attribute__ ((packed));

#define GSL_RB_MEMPTRS_RPTR_OFFSET \
	(offsetof(struct kgsl_rbmemptrs, rptr))

#define GSL_RB_MEMPTRS_WPTRPOLL_OFFSET \
	(offsetof(struct kgsl_rbmemptrs, wptr_poll))

struct kgsl_rbstats {
	int64_t issues;
	int64_t words_total;
};


struct kgsl_ringbuffer {
	struct kgsl_device *device;
	uint32_t flags;

	struct kgsl_memdesc buffer_desc;

	struct kgsl_memdesc memptrs_desc;
	struct kgsl_rbmemptrs *memptrs;

	/*ringbuffer size */
	unsigned int sizedwords;
	unsigned int blksizequadwords;

	unsigned int wptr; /* write pointer offset in dwords from baseaddr */
	unsigned int rptr; /* read pointer offset in dwords from baseaddr */
	uint32_t timestamp;

	/* queue of memfrees pending timestamp elapse */
	struct list_head memqueue;

	struct kgsl_rbwatchdog watchdog;

#ifdef GSL_STATS_RINGBUFFER
	struct kgsl_rbstats stats;
#endif /* GSL_STATS_RINGBUFFER */

};

/* dword base address of the GFX decode space */
#define GSL_HAL_SUBBLOCK_OFFSET(reg) ((unsigned int)((reg) - (0x2000)))

#define GSL_RB_WRITE(ring, data) \
	do { \
		mb(); \
		writel(data, ring); \
		ring++; \
	} while (0)

/* timestamp */
#ifdef GSL_DEVICE_SHADOW_MEMSTORE_TO_USER
#define GSL_RB_USE_MEM_TIMESTAMP
#endif /* GSL_DEVICE_SHADOW_MEMSTORE_TO_USER */

#ifdef GSL_RB_USE_MEM_TIMESTAMP
/* enable timestamp (...scratch0) memory shadowing */
#define GSL_RB_MEMPTRS_SCRATCH_MASK 0x1
#define GSL_RB_INIT_TIMESTAMP(rb)

#else
#define GSL_RB_MEMPTRS_SCRATCH_MASK 0x0
#define GSL_RB_INIT_TIMESTAMP(rb) \
		kgsl_yamato_regwrite((rb)->device->id, REG_CP_TIMESTAMP, 0)

#endif /* GSL_RB_USE_MEMTIMESTAMP */

/* mem rptr */
#ifdef GSL_RB_USE_MEM_RPTR
#define GSL_RB_CNTL_NO_UPDATE 0x0 /* enable */
#define GSL_RB_GET_READPTR(rb, data) \
	do { \
		*(data) = (rb)->memptrs->rptr; \
	} while (0)
#else
#define GSL_RB_CNTL_NO_UPDATE 0x1 /* disable */
#define GSL_RB_GET_READPTR(rb, data) \
	do { \
		kgsl_yamato_regread((rb)->device->id, REG_CP_RB_RPTR, (data)); \
	} while (0)
#endif /* GSL_RB_USE_MEMRPTR */

/* wptr polling */
#ifdef GSL_RB_USE_WPTR_POLLING
#define GSL_RB_CNTL_POLL_EN 0x1 /* enable */
#define GSL_RB_UPDATE_WPTR_POLLING(rb) \
	do { (rb)->memptrs->wptr_poll = (rb)->wptr; } while (0)
#else
#define GSL_RB_CNTL_POLL_EN 0x0 /* disable */
#define GSL_RB_UPDATE_WPTR_POLLING(rb)
#endif	/* GSL_RB_USE_WPTR_POLLING */

/* stats */
#ifdef GSL_STATS_RINGBUFFER
#define GSL_RB_STATS(x) x
#else
#define GSL_RB_STATS(x)
#endif /* GSL_STATS_RINGBUFFER */

struct kgsl_pmem_entry;

int kgsl_ringbuffer_issueibcmds(struct kgsl_device *, int drawctxt_index,
				uint32_t ibaddr, int sizedwords,
				uint32_t *timestamp,
				unsigned int flags);

int kgsl_ringbuffer_init(struct kgsl_device *device);

int kgsl_ringbuffer_close(struct kgsl_ringbuffer *rb);

uint32_t kgsl_ringbuffer_issuecmds(struct kgsl_device *device,
					int pmodeoff,
					unsigned int *cmdaddr,
					int sizedwords);

int kgsl_ringbuffer_gettimestampshadow(struct kgsl_device *device,
					unsigned int *sopaddr,
					unsigned int *eopaddr);

void kgsl_ringbuffer_watchdog(void);

void kgsl_cp_intrcallback(struct kgsl_device *device);

#endif  /* __GSL_RINGBUFFER_H */
