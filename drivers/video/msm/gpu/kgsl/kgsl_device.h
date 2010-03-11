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
#ifndef _KGSL_DEVICE_H
#define _KGSL_DEVICE_H

#include <asm/atomic.h>

#include <linux/types.h>
#include <linux/irqreturn.h>
#include <linux/wait.h>
#include <linux/msm_kgsl.h>

#include "kgsl_drawctxt.h"
#include "kgsl_mmu.h"
#include "kgsl_ringbuffer.h"

#define KGSL_CONTEXT_MAX        8

#define KGSL_TIMEOUT_NONE       0
#define KGSL_TIMEOUT_DEFAULT    0xFFFFFFFF

#define KGSL_DEV_FLAGS_INITIALIZED0	0x00000001
#define KGSL_DEV_FLAGS_INITIALIZED	0x00000002
#define KGSL_DEV_FLAGS_STARTED		0x00000004
#define KGSL_DEV_FLAGS_ACTIVE		0x00000008

#define KGSL_CHIPID_YAMATODX_REV21  0x20100
#define KGSL_CHIPID_YAMATODX_REV211 0x20101

/* Private memory flags for use with memdesc->priv feild */
#define KGSL_MEMFLAGS_MEM_REQUIRES_FLUSH    0x00000001
#define KGSL_MEMFLAGS_VMALLOC_MEM           0x00000002

#define KGSL_GRAPHICS_MEMORY_LOW_WATERMARK  0x1000000
#define KGSL_IS_PAGE_ALIGNED(addr) (!((addr) & (~PAGE_MASK)))

struct kgsl_device;
struct platform_device;


struct kgsl_memregion {
	unsigned char  *mmio_virt_base;
	unsigned int   mmio_phys_base;
	uint32_t      gpu_base;
	unsigned int   sizebytes;
};

struct kgsl_device {

	unsigned int	  refcnt;
	uint32_t       flags;
	enum kgsl_deviceid    id;
	unsigned int      chip_id;
	struct kgsl_memregion regspace;
	struct kgsl_memdesc memstore;

	struct kgsl_mmu 	  mmu;
	struct kgsl_memregion gmemspace;
	struct kgsl_ringbuffer ringbuffer;
	unsigned int      drawctxt_count;
	struct kgsl_drawctxt *drawctxt_active;
	struct kgsl_drawctxt drawctxt[KGSL_CONTEXT_MAX];

	wait_queue_head_t ib1_wq;
};

struct kgsl_devconfig {
	struct kgsl_memregion regspace;

	unsigned int     mmu_config;
	uint32_t        mpu_base;
	int              mpu_range;
	uint32_t        va_base;
	unsigned int     va_range;

	struct kgsl_memregion gmemspace;
};

int kgsl_yamato_start(struct kgsl_device *device, uint32_t flags);

int kgsl_yamato_stop(struct kgsl_device *device);

bool kgsl_yamato_is_idle(struct kgsl_device *device);

int kgsl_yamato_idle(struct kgsl_device *device, unsigned int timeout);

int kgsl_yamato_getproperty(struct kgsl_device *device,
				enum kgsl_property_type type, void *value,
				unsigned int sizebytes);

int kgsl_yamato_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value);

int kgsl_yamato_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value);

int kgsl_yamato_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp, unsigned int timeout);


int kgsl_yamato_init(struct kgsl_device *, struct kgsl_devconfig *);

int kgsl_yamato_close(struct kgsl_device *device);

int kgsl_yamato_runpending(struct kgsl_device *device);

int __init kgsl_yamato_config(struct kgsl_devconfig *,
				struct platform_device *pdev);

void kgsl_register_dump(struct kgsl_device *device);

int kgsl_yamato_setup_pt(struct kgsl_device *device,
			 struct kgsl_pagetable *pagetable);
int kgsl_yamato_cleanup_pt(struct kgsl_device *device,
			   struct kgsl_pagetable *pagetable);
#ifdef CONFIG_MSM_KGSL_MMU
int kgsl_yamato_setstate(struct kgsl_device *device, uint32_t flags);
#else
static inline int kgsl_yamato_setstate(struct kgsl_device *device, uint32_t flags)
{ return 0; }
#endif

irqreturn_t kgsl_yamato_isr(int irq, void *data);

#endif  /* _KGSL_DEVICE_H */
