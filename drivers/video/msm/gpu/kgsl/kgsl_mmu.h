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
#ifndef __GSL_MMU_H
#define __GSL_MMU_H
#include <linux/types.h>
#include <linux/msm_kgsl.h>
#include "kgsl_log.h"
#include "kgsl_sharedmem.h"

#define GSL_PT_SUPER_PTE 8
#define GSL_PT_PAGE_WV		0x00000001
#define GSL_PT_PAGE_RV		0x00000002
#define GSL_PT_PAGE_DIRTY	0x00000004
/* MMU Flags */
#define KGSL_MMUFLAGS_TLBFLUSH         0x10000000
#define KGSL_MMUFLAGS_PTUPDATE         0x20000000

extern unsigned int kgsl_cache_enable;

struct kgsl_device;

struct kgsl_mmu_debug {
	unsigned int  config;
	unsigned int  mpu_base;
	unsigned int  mpu_end;
	unsigned int  va_range;
	unsigned int  pt_base;
	unsigned int  page_fault;
	unsigned int  trans_error;
	unsigned int  axi_error;
	unsigned int  interrupt_mask;
	unsigned int  interrupt_status;
};

struct kgsl_ptstats {
	int64_t  maps;
	int64_t  unmaps;
	int64_t  superpteallocs;
	int64_t  superptefrees;
	int64_t  ptswitches;
	int64_t  tlbflushes[KGSL_DEVICE_MAX];
};

struct kgsl_pagetable {
	unsigned int   refcnt;
	struct kgsl_mmu *mmu;
	struct kgsl_memdesc  base;
	uint32_t      va_base;
	unsigned int   va_range;
	unsigned int   last_superpte;
	unsigned int   max_entries;
	struct gen_pool *pool;
};

struct kgsl_mmu {
	unsigned int     refcnt;
	uint32_t      flags;
	struct kgsl_device     *device;
	unsigned int     config;
	uint32_t        mpu_base;
	int              mpu_range;
	uint32_t        va_base;
	unsigned int     va_range;
	struct kgsl_memdesc    dummyspace;
	/* current page table object being used by device mmu */
	struct kgsl_pagetable  *defaultpagetable;
	struct kgsl_pagetable  *hwpagetable;
};


static inline int
kgsl_mmu_isenabled(struct kgsl_mmu *mmu)
{
	return ((mmu)->flags & KGSL_FLAGS_STARTED) ? 1 : 0;
}


int kgsl_mmu_init(struct kgsl_device *device);

int kgsl_mmu_close(struct kgsl_device *device);

struct kgsl_pagetable *kgsl_mmu_createpagetableobject(struct kgsl_mmu *mmu);

int kgsl_mmu_destroypagetableobject(struct kgsl_pagetable *pagetable);

int kgsl_mmu_setstate(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable);

#ifdef CONFIG_MSM_KGSL_MMU
int kgsl_mmu_map(struct kgsl_pagetable *pagetable,
		 unsigned int address,
		 int range,
		 unsigned int protflags,
		 unsigned int *gpuaddr,
		 unsigned int flags);

int kgsl_mmu_unmap(struct kgsl_pagetable *pagetable,
					unsigned int gpuaddr, int range);

pte_t *kgsl_get_pte_from_vaddr(unsigned int vaddr);
#else
static inline int kgsl_mmu_map(struct kgsl_pagetable *pagetable,
			       unsigned int address,
			       int range,
			       unsigned int protflags,
			       unsigned int *gpuaddr,
			       unsigned int flags)
{
	*gpuaddr = address;
	return 0;
}

static inline int kgsl_mmu_unmap(struct kgsl_pagetable *pagetable,
				 unsigned int gpuaddr, int range) { return 0; }

static inline pte_t *kgsl_get_pte_from_vaddr(unsigned int vaddr) {return NULL;}
#endif

int kgsl_mmu_querystats(struct kgsl_pagetable *pagetable,
			struct kgsl_ptstats *stats);

void kgsl_mh_intrcallback(struct kgsl_device *device);

#ifdef DEBUG
void kgsl_mmu_debug(struct kgsl_mmu *, struct kgsl_mmu_debug*);
#else
static inline void kgsl_mmu_debug(struct kgsl_mmu *mmu,
				  struct kgsl_mmu_debug *mmu_debug) { }
#endif /* DEBUG */

#endif /* __GSL_MMU_H */
