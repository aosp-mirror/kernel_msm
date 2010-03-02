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
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>

#include "kgsl_sharedmem.h"
#include "kgsl_device.h"
#include "kgsl.h"
#include "kgsl_log.h"

/*  block alignment shift count */
static inline unsigned int
kgsl_memarena_get_order(uint32_t flags)
{
	unsigned int alignshift;
	alignshift = ((flags & KGSL_MEMFLAGS_ALIGN_MASK)
		      >> KGSL_MEMFLAGS_ALIGN_SHIFT);
	return alignshift;
}

/*  block alignment shift count */
static inline unsigned int
kgsl_memarena_align(unsigned int address, unsigned int shift)
{
	unsigned int alignedbaseaddr = ((address) >> shift) << shift;
	if (alignedbaseaddr < address)
		alignedbaseaddr += (1 << shift);

	return alignedbaseaddr;
}

int
kgsl_sharedmem_init(struct kgsl_sharedmem *shmem)
{
	int result = -EINVAL;

	if (!request_mem_region(shmem->physbase, shmem->size, DRIVER_NAME)) {
		KGSL_MEM_ERR("request_mem_region failed\n");
		goto error;
	}

	shmem->baseptr = ioremap(shmem->physbase, shmem->size);
	KGSL_MEM_INFO("ioremap(shm) = %p\n", shmem->baseptr);

	if (shmem->baseptr == NULL) {
		KGSL_MEM_ERR("ioremap failed for address %08x size %d\n",
				shmem->physbase, shmem->size);
		result = -ENODEV;
		goto error_release_mem;
	}

	shmem->pool = gen_pool_create(KGSL_PAGESIZE_SHIFT, -1);
	if (shmem->pool == NULL) {
		KGSL_MEM_ERR("gen_pool_create failed\n");
		result = -ENOMEM;
		goto error_iounmap;
	}

	if (gen_pool_add(shmem->pool, shmem->physbase, shmem->size, -1)) {
		KGSL_MEM_ERR("gen_pool_create failed\n");
		result = -ENOMEM;
		goto error_pool_destroy;
	}
	result = 0;
	KGSL_MEM_INFO("physbase 0x%08x size 0x%08x baseptr 0x%p\n",
			shmem->physbase, shmem->size, shmem->baseptr);
	return 0;

error_pool_destroy:
	gen_pool_destroy(shmem->pool);
error_iounmap:
	iounmap(shmem->baseptr);
	shmem->baseptr = NULL;
error_release_mem:
	release_mem_region(shmem->physbase, shmem->size);
error:
	return result;
}

int
kgsl_sharedmem_close(struct kgsl_sharedmem *shmem)
{
	if (shmem->pool) {
		gen_pool_destroy(shmem->pool);
		shmem->pool = NULL;
	}

	if (shmem->baseptr != NULL) {
		KGSL_MEM_INFO("iounmap(shm) = %p\n", shmem->baseptr);
		iounmap(shmem->baseptr);
		shmem->baseptr = NULL;
		release_mem_region(shmem->physbase, shmem->size);
	}

	return 0;
}
/*
* get the host mapped address for a hardware device address
*/
static void *kgsl_memarena_gethostptr(struct kgsl_sharedmem *shmem,
					uint32_t physaddr)
{
	void *result;

	KGSL_MEM_VDBG("enter (memarena=%p, physaddr=0x%08x)\n",
			shmem, physaddr);

	BUG_ON(shmem == NULL);

	/* check address range */
	if (physaddr < shmem->physbase)
		return NULL;

	if (physaddr >= shmem->physbase + shmem->size)
		return NULL;

	if (shmem->baseptr == NULL) {
		KGSL_MEM_VDBG("return: %p\n", NULL);
		return NULL;
	}

	result = ((physaddr - shmem->physbase) + shmem->baseptr);

	KGSL_MEM_VDBG("return: %p\n", result);

	return result;
}


int
kgsl_sharedmem_alloc(uint32_t flags, int size,
			struct kgsl_memdesc *memdesc)
{
	struct kgsl_sharedmem *shmem;
	int result = -ENOMEM;
	unsigned int blksize;
	unsigned int baseaddr;
	unsigned int alignshift;
	unsigned int alignedbaseaddr;

	KGSL_MEM_VDBG("enter (flags=0x%08x, size=%d, memdesc=%p)\n",
					flags, size, memdesc);

	shmem = &kgsl_driver.shmem;
	BUG_ON(memdesc == NULL);
	BUG_ON(size <= 0);

	alignshift = kgsl_memarena_get_order(flags);

	size = ALIGN(size, KGSL_PAGESIZE);
	blksize = size;
	if (alignshift > KGSL_PAGESIZE_SHIFT)
		blksize += (1 << alignshift) - KGSL_PAGESIZE;

	baseaddr = gen_pool_alloc(shmem->pool, blksize);
	if (baseaddr == 0) {
		KGSL_MEM_ERR("gen_pool_alloc failed\n");
		result = -ENOMEM;
		goto done;
	}
	result = 0;

	if (alignshift > KGSL_PAGESIZE_SHIFT) {
		alignedbaseaddr = ALIGN(baseaddr, (1 << alignshift));

		KGSL_MEM_VDBG("ba %x al %x as %d m->as %d bs %x s %x\n",
				baseaddr, alignedbaseaddr, alignshift,
				KGSL_PAGESIZE_SHIFT, blksize, size);
		if (alignedbaseaddr > baseaddr) {
			KGSL_MEM_VDBG("physaddr %x free before %x size %x\n",
					alignedbaseaddr,
					baseaddr, alignedbaseaddr - baseaddr);
			gen_pool_free(shmem->pool, baseaddr,
					alignedbaseaddr - baseaddr);
			blksize -= alignedbaseaddr - baseaddr;
		}
		if (blksize > size) {
			KGSL_MEM_VDBG("physaddr %x free after %x size %x\n",
					alignedbaseaddr,
					alignedbaseaddr + size,
					blksize - size);
			gen_pool_free(shmem->pool,
					alignedbaseaddr + size,
					blksize - size);
		}
	} else {
		alignedbaseaddr = baseaddr;
	}

	memdesc->physaddr = alignedbaseaddr;
	memdesc->hostptr = kgsl_memarena_gethostptr(shmem, memdesc->physaddr);
	memdesc->size = size;

	KGSL_MEM_VDBG("ashift %d m->ashift %d blksize %d base %x abase %x\n",
			alignshift, KGSL_PAGESIZE_SHIFT, blksize, baseaddr,
			alignedbaseaddr);

done:
	if (result)
		memset(memdesc, 0, sizeof(*memdesc));


	KGSL_MEM_VDBG("return: %d\n", result);
	return result;
}

void
kgsl_sharedmem_free(struct kgsl_memdesc *memdesc)
{
	struct kgsl_sharedmem  *shmem =  &kgsl_driver.shmem;

	KGSL_MEM_VDBG("enter (shmem=%p, memdesc=%p, physaddr=%08x, size=%d)\n",
			shmem, memdesc, memdesc->physaddr, memdesc->size);

	BUG_ON(memdesc == NULL);
	BUG_ON(memdesc->size <= 0);
	BUG_ON(shmem->physbase > memdesc->physaddr);
	BUG_ON((shmem->physbase + shmem->size)
	       < (memdesc->physaddr + memdesc->size));

	gen_pool_free(shmem->pool, memdesc->physaddr, memdesc->size);

	memset(memdesc, 0, sizeof(struct kgsl_memdesc));
	KGSL_MEM_VDBG("return\n");
}

int
kgsl_sharedmem_read(const struct kgsl_memdesc *memdesc, void *dst,
			unsigned int offsetbytes, unsigned int sizebytes)
{
	if (memdesc == NULL || memdesc->hostptr == NULL || dst == NULL) {
		KGSL_MEM_ERR("bad ptr memdesc %p hostptr %p dst %p\n",
				memdesc,
				(memdesc ? memdesc->hostptr : NULL),
				dst);
		return -EINVAL;
	}
	if (offsetbytes + sizebytes > memdesc->size) {
		KGSL_MEM_ERR("bad range: offset %d size %d memdesc %d\n",
				offsetbytes, sizebytes, memdesc->size);
		return -ERANGE;
	}
	memcpy(dst, memdesc->hostptr + offsetbytes, sizebytes);
	return 0;
}

int
kgsl_sharedmem_write(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes,
			void *value, unsigned int sizebytes)
{
	if (memdesc == NULL || memdesc->hostptr == NULL) {
		KGSL_MEM_ERR("bad ptr memdesc %p hostptr %p\n", memdesc,
				(memdesc ? memdesc->hostptr : NULL));
		return -EINVAL;
	}
	if (offsetbytes + sizebytes > memdesc->size) {
		KGSL_MEM_ERR("bad range: offset %d size %d memdesc %d\n",
				offsetbytes, sizebytes, memdesc->size);
		return -ERANGE;
	}
	memcpy(memdesc->hostptr + offsetbytes, value, sizebytes);
	return 0;
}

int
kgsl_sharedmem_set(const struct kgsl_memdesc *memdesc, unsigned int offsetbytes,
			unsigned int value, unsigned int sizebytes)
{
	if (memdesc == NULL || memdesc->hostptr == NULL) {
		KGSL_MEM_ERR("bad ptr memdesc %p hostptr %p\n", memdesc,
				(memdesc ? memdesc->hostptr : NULL));
		return -EINVAL;
	}
	if (offsetbytes + sizebytes > memdesc->size) {
		KGSL_MEM_ERR("bad range: offset %d size %d memdesc %d\n",
				offsetbytes, sizebytes, memdesc->size);
		return -ERANGE;
	}
	memset(memdesc->hostptr + offsetbytes, value, sizebytes);
	return 0;
}

