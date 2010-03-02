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
#ifndef __GSL_SHAREDMEM_H
#define __GSL_SHAREDMEM_H

#include <linux/types.h>
#include <linux/msm_kgsl.h>

#define KGSL_PAGESIZE           0x1000
#define KGSL_PAGESIZE_SHIFT     12
#define KGSL_PAGEMASK           (~(KGSL_PAGESIZE - 1))

struct kgsl_pagetable;

struct platform_device;
struct gen_pool;

/* memory allocation flags */
#define KGSL_MEMFLAGS_ANY	0x00000000 /*dont care*/

#define KGSL_MEMFLAGS_APERTUREANY 0x00000000
#define KGSL_MEMFLAGS_EMEM	0x00000000
#define KGSL_MEMFLAGS_CONPHYS 	0x00001000

#define KGSL_MEMFLAGS_ALIGNANY	0x00000000
#define KGSL_MEMFLAGS_ALIGN32	0x00000000
#define KGSL_MEMFLAGS_ALIGN64	0x00060000
#define KGSL_MEMFLAGS_ALIGN128	0x00070000
#define KGSL_MEMFLAGS_ALIGN256	0x00080000
#define KGSL_MEMFLAGS_ALIGN512	0x00090000
#define KGSL_MEMFLAGS_ALIGN1K	0x000A0000
#define KGSL_MEMFLAGS_ALIGN2K	0x000B0000
#define KGSL_MEMFLAGS_ALIGN4K	0x000C0000
#define KGSL_MEMFLAGS_ALIGN8K	0x000D0000
#define KGSL_MEMFLAGS_ALIGN16K	0x000E0000
#define KGSL_MEMFLAGS_ALIGN32K	0x000F0000
#define KGSL_MEMFLAGS_ALIGN64K	0x00100000
#define KGSL_MEMFLAGS_ALIGNPAGE	KGSL_MEMFLAGS_ALIGN4K

/* fail the alloc if the flags cannot be honored */
#define KGSL_MEMFLAGS_STRICTREQUEST 0x80000000

#define KGSL_MEMFLAGS_APERTURE_MASK	0x0000F000
#define KGSL_MEMFLAGS_ALIGN_MASK 	0x00FF0000

#define KGSL_MEMFLAGS_APERTURE_SHIFT	12
#define KGSL_MEMFLAGS_ALIGN_SHIFT	16


/* shared memory allocation */
struct kgsl_memdesc {
	struct kgsl_pagetable *pagetable;
	void  *hostptr;
	unsigned int gpuaddr;
	unsigned int physaddr;
	unsigned int size;
	unsigned int priv;
};

struct kgsl_sharedmem {
	void *baseptr;
	unsigned int physbase;
	unsigned int size;
	struct gen_pool *pool;
};

int kgsl_sharedmem_alloc(uint32_t flags, int size,
			struct kgsl_memdesc *memdesc);

/*TODO: add protection flags */
int kgsl_sharedmem_import(struct kgsl_pagetable *,
				uint32_t phys_addr,
				uint32_t size,
				struct kgsl_memdesc *memdesc);


void kgsl_sharedmem_free(struct kgsl_memdesc *memdesc);


int kgsl_sharedmem_read(const struct kgsl_memdesc *memdesc, void *dst,
			unsigned int offsetbytes, unsigned int sizebytes);

int kgsl_sharedmem_write(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes, void *value,
			unsigned int sizebytes);

int kgsl_sharedmem_set(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes, unsigned int value,
			unsigned int sizebytes);

int kgsl_sharedmem_init(struct kgsl_sharedmem *shmem);

int kgsl_sharedmem_close(struct kgsl_sharedmem *shmem);

#endif /* __GSL_SHAREDMEM_H */
