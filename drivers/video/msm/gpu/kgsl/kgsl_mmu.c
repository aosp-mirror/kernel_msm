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
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>

#include <asm/pgalloc.h>
#include <asm/pgtable.h>

#include "kgsl_mmu.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "yamato_reg.h"

struct kgsl_pte_debug {
	unsigned int read:1;
	unsigned int write:1;
	unsigned int dirty:1;
	unsigned int reserved:9;
	unsigned int phyaddr:20;
};

#define GSL_PTE_SIZE	4
#define GSL_PT_EXTRA_ENTRIES	16


#define GSL_PT_PAGE_BITS_MASK	0x00000007
#define GSL_PT_PAGE_ADDR_MASK	(~(KGSL_PAGESIZE - 1))

#define GSL_MMU_INT_MASK \
	(MH_INTERRUPT_MASK__AXI_READ_ERROR | \
	 MH_INTERRUPT_MASK__AXI_WRITE_ERROR)

uint32_t kgsl_pt_entry_get(struct kgsl_pagetable *pt, uint32_t va)
{
	return (va - pt->va_base) >> KGSL_PAGESIZE_SHIFT;
}

uint32_t kgsl_pt_map_get(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return baseptr[pte];
}

void kgsl_pt_map_set(struct kgsl_pagetable *pt, uint32_t pte, uint32_t val)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] = val;
}
#define GSL_PT_MAP_DEBUG(pte)	((struct kgsl_pte_debug *) \
		&gsl_pt_map_get(pagetable, pte))

void kgsl_pt_map_setbits(struct kgsl_pagetable *pt, uint32_t pte, uint32_t bits)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] |= bits;
}

void kgsl_pt_map_setaddr(struct kgsl_pagetable *pt, uint32_t pte,
					uint32_t pageaddr)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	uint32_t val = baseptr[pte];
	val &= ~GSL_PT_PAGE_ADDR_MASK;
	val |= (pageaddr & GSL_PT_PAGE_ADDR_MASK);
	baseptr[pte] = val;
}

void kgsl_pt_map_resetall(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] &= GSL_PT_PAGE_DIRTY;
}

void kgsl_pt_map_resetbits(struct kgsl_pagetable *pt, uint32_t pte,
				uint32_t bits)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] &= ~(bits & GSL_PT_PAGE_BITS_MASK);
}

int kgsl_pt_map_isdirty(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return baseptr[pte] & GSL_PT_PAGE_DIRTY;
}

uint32_t kgsl_pt_map_getaddr(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return baseptr[pte] & GSL_PT_PAGE_ADDR_MASK;
}

void kgsl_mh_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;
	unsigned int reg;
	unsigned int axi_error;
	struct kgsl_mmu_debug dbg;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	kgsl_yamato_regread(device, REG_MH_INTERRUPT_STATUS, &status);

	if (status & MH_INTERRUPT_MASK__AXI_READ_ERROR) {
		kgsl_yamato_regread(device, REG_MH_AXI_ERROR, &axi_error);
		KGSL_MEM_FATAL("axi read error interrupt (%08x)\n", axi_error);
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else if (status & MH_INTERRUPT_MASK__AXI_WRITE_ERROR) {
		kgsl_yamato_regread(device, REG_MH_AXI_ERROR, &axi_error);
		KGSL_MEM_FATAL("axi write error interrupt (%08x)\n", axi_error);
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else if (status & MH_INTERRUPT_MASK__MMU_PAGE_FAULT) {
		kgsl_yamato_regread(device, REG_MH_MMU_PAGE_FAULT, &reg);
		KGSL_MEM_FATAL("mmu page fault interrupt: %08x\n", reg);
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else {
		KGSL_MEM_DBG("bad bits in REG_MH_INTERRUPT_STATUS %08x\n",
			     status);
	}

	kgsl_yamato_regwrite(device, REG_MH_INTERRUPT_CLEAR, status);

	/*TODO: figure out how to handle errror interupts.
	* specifically, page faults should probably nuke the client that
	* caused them, but we don't have enough info to figure that out yet.
	*/

	KGSL_MEM_VDBG("return\n");
}

#ifdef DEBUG
void kgsl_mmu_debug(struct kgsl_mmu *mmu, struct kgsl_mmu_debug *regs)
{
	memset(regs, 0, sizeof(struct kgsl_mmu_debug));

	kgsl_yamato_regread(mmu->device, REG_MH_MMU_CONFIG,
			    &regs->config);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_MPU_BASE,
			    &regs->mpu_base);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_MPU_END,
			    &regs->mpu_end);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_VA_RANGE,
			    &regs->va_range);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_PT_BASE,
			    &regs->pt_base);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_PAGE_FAULT,
			    &regs->page_fault);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_TRAN_ERROR,
			    &regs->trans_error);
	kgsl_yamato_regread(mmu->device, REG_MH_AXI_ERROR,
			    &regs->axi_error);
	kgsl_yamato_regread(mmu->device, REG_MH_INTERRUPT_MASK,
			    &regs->interrupt_mask);
	kgsl_yamato_regread(mmu->device, REG_MH_INTERRUPT_STATUS,
			    &regs->interrupt_status);

	KGSL_MEM_DBG("mmu config %08x mpu_base %08x mpu_end %08x\n",
		     regs->config, regs->mpu_base, regs->mpu_end);
	KGSL_MEM_DBG("mmu va_range %08x pt_base %08x \n",
		     regs->va_range, regs->pt_base);
	KGSL_MEM_DBG("mmu page_fault %08x tran_err %08x\n",
		     regs->page_fault, regs->trans_error);
	KGSL_MEM_DBG("mmu int mask %08x int status %08x\n",
			regs->interrupt_mask, regs->interrupt_status);
}
#endif

struct kgsl_pagetable *kgsl_mmu_createpagetableobject(struct kgsl_mmu *mmu)
{
	int status = 0;
	struct kgsl_pagetable *pagetable = NULL;
	uint32_t flags;

	KGSL_MEM_VDBG("enter (mmu=%p)\n", mmu);

	pagetable = kzalloc(sizeof(struct kgsl_pagetable), GFP_KERNEL);
	if (pagetable == NULL) {
		KGSL_MEM_ERR("Unable to allocate pagetable object.\n");
		return NULL;
	}

	pagetable->mmu = mmu;
	pagetable->va_base = mmu->va_base;
	pagetable->va_range = mmu->va_range;
	pagetable->last_superpte = 0;
	pagetable->max_entries = (mmu->va_range >> KGSL_PAGESIZE_SHIFT)
				 + GSL_PT_EXTRA_ENTRIES;

	pagetable->pool = gen_pool_create(KGSL_PAGESIZE_SHIFT, -1);
	if (pagetable->pool == NULL) {
		KGSL_MEM_ERR("Unable to allocate virtualaddr pool.\n");
		goto err_gen_pool_create;
	}

	if (gen_pool_add(pagetable->pool, pagetable->va_base,
				pagetable->va_range, -1)) {
		KGSL_MEM_ERR("gen_pool_create failed for pagetable %p\n",
				pagetable);
		goto err_gen_pool_add;
	}

	/* allocate page table memory */
	flags = (KGSL_MEMFLAGS_ALIGN4K | KGSL_MEMFLAGS_CONPHYS
		 | KGSL_MEMFLAGS_STRICTREQUEST);
	status = kgsl_sharedmem_alloc(flags,
				      pagetable->max_entries * GSL_PTE_SIZE,
				      &pagetable->base);

	if (status) {
		KGSL_MEM_ERR("cannot alloc page tables\n");
		goto err_kgsl_sharedmem_alloc;
	}

	/* reset page table entries
	 * -- all pte's are marked as not dirty initially
	 */
	kgsl_sharedmem_set(&pagetable->base, 0, 0, pagetable->base.size);
	pagetable->base.gpuaddr = pagetable->base.physaddr;

	KGSL_MEM_VDBG("return %p\n", pagetable);

	return pagetable;

err_kgsl_sharedmem_alloc:
err_gen_pool_add:
	gen_pool_destroy(pagetable->pool);
err_gen_pool_create:
	kfree(pagetable);
	return NULL;
}

int kgsl_mmu_destroypagetableobject(struct kgsl_pagetable *pagetable)
{
	KGSL_MEM_VDBG("enter (pagetable=%p)\n", pagetable);

	if (pagetable) {
		if (pagetable->base.gpuaddr)
			kgsl_sharedmem_free(&pagetable->base);

		if (pagetable->pool) {
			gen_pool_destroy(pagetable->pool);
			pagetable->pool = NULL;
		}

		kfree(pagetable);

	}
	KGSL_MEM_VDBG("return 0x%08x\n", 0);

	return 0;
}

int kgsl_mmu_setstate(struct kgsl_device *device,
				struct kgsl_pagetable *pagetable)
{
	int status = 0;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p, pagetable=%p)\n", device, pagetable);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		/* page table not current, then setup mmu to use new
		 *  specified page table
		 */
		KGSL_MEM_INFO("from %p to %p\n", mmu->hwpagetable, pagetable);
		if (mmu->hwpagetable != pagetable) {
			mmu->hwpagetable = pagetable;

			/* call device specific set page table */
			status = kgsl_yamato_setstate(mmu->device,
				KGSL_MMUFLAGS_TLBFLUSH |
				KGSL_MMUFLAGS_PTUPDATE);
		}
	}

	KGSL_MEM_VDBG("return %d\n", status);

	return status;
}

int kgsl_mmu_init(struct kgsl_device *device)
{
	/*
	 * intialize device mmu
	 *
	 * call this with the global lock held
	 */
	int status;
	uint32_t flags;
	struct kgsl_mmu *mmu = &device->mmu;
#ifdef _DEBUG
	struct kgsl_mmu_debug regs;
#endif /* _DEBUG */

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_INITIALIZED0) {
		KGSL_MEM_INFO("MMU already initialized.\n");
		return 0;
	}

	mmu->device = device;

#ifndef CONFIG_MSM_KGSL_MMU
	mmu->config = 0x00000000;
#endif

	/* setup MMU and sub-client behavior */
	kgsl_yamato_regwrite(device, REG_MH_MMU_CONFIG, mmu->config);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK);
	kgsl_yamato_regwrite(device, REG_MH_INTERRUPT_MASK, GSL_MMU_INT_MASK);

	mmu->flags |= KGSL_FLAGS_INITIALIZED0;

	/* MMU not enabled */
	if ((mmu->config & 0x1) == 0) {
		KGSL_MEM_VDBG("return %d\n", 0);
		return 0;
	}

	/* idle device */
	kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);

	/* make sure aligned to pagesize */
	BUG_ON(mmu->mpu_base & (KGSL_PAGESIZE - 1));
	BUG_ON((mmu->mpu_base + mmu->mpu_range) & (KGSL_PAGESIZE - 1));

	/* define physical memory range accessible by the core */
	kgsl_yamato_regwrite(device, REG_MH_MMU_MPU_BASE,
				mmu->mpu_base);
	kgsl_yamato_regwrite(device, REG_MH_MMU_MPU_END,
				mmu->mpu_base + mmu->mpu_range);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);
	kgsl_yamato_regwrite(device, REG_MH_INTERRUPT_MASK,
			GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);

	mmu->flags |= KGSL_FLAGS_INITIALIZED;

	/* sub-client MMU lookups require address translation */
	if ((mmu->config & ~0x1) > 0) {
		/*make sure virtual address range is a multiple of 64Kb */
		BUG_ON(mmu->va_range & ((1 << 16) - 1));

		/* allocate memory used for completing r/w operations that
		 * cannot be mapped by the MMU
		 */
		flags = (KGSL_MEMFLAGS_ALIGN4K | KGSL_MEMFLAGS_CONPHYS
			 | KGSL_MEMFLAGS_STRICTREQUEST);
		status = kgsl_sharedmem_alloc(flags, 64, &mmu->dummyspace);
		if (status != 0) {
			KGSL_MEM_ERR
			    ("Unable to allocate dummy space memory.\n");
			kgsl_mmu_close(device);
			return status;
		}

		kgsl_sharedmem_set(&mmu->dummyspace, 0, 0,
				   mmu->dummyspace.size);
		/* TRAN_ERROR needs a 32 byte (32 byte aligned) chunk of memory
		 * to complete transactions in case of an MMU fault. Note that
		 * we'll leave the bottom 32 bytes of the dummyspace for other
		 * purposes (e.g. use it when dummy read cycles are needed
		 * for other blocks */
		kgsl_yamato_regwrite(device,
				     REG_MH_MMU_TRAN_ERROR,
				     mmu->dummyspace.physaddr + 32);

		mmu->defaultpagetable = kgsl_mmu_createpagetableobject(mmu);
		if (!mmu->defaultpagetable) {
			KGSL_MEM_ERR("Failed to create global page table\n");
			kgsl_mmu_close(device);
			return -ENOMEM;
		}
		mmu->hwpagetable = mmu->defaultpagetable;
		kgsl_yamato_regwrite(device, REG_MH_MMU_PT_BASE,
					mmu->hwpagetable->base.gpuaddr);
		kgsl_yamato_regwrite(device, REG_MH_MMU_VA_RANGE,
				(mmu->hwpagetable->va_base |
				(mmu->hwpagetable->va_range >> 16)));
		status = kgsl_yamato_setstate(device, KGSL_MMUFLAGS_TLBFLUSH);
		if (status) {
			kgsl_mmu_close(device);
			return status;
		}

		mmu->flags |= KGSL_FLAGS_STARTED;
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}

#ifdef CONFIG_MSM_KGSL_MMU
pte_t *kgsl_get_pte_from_vaddr(unsigned int vaddr)
{
	pgd_t *pgd_ptr = NULL;
	pmd_t *pmd_ptr = NULL;
	pte_t *pte_ptr = NULL;

	pgd_ptr = pgd_offset(current->mm, vaddr);
	if (pgd_none(*pgd) || pgd_bad(*pgd)) {
		KGSL_MEM_ERR
		    ("Invalid pgd entry found while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}

	pmd_ptr = pmd_offset(pgd_ptr, vaddr);
	if (pmd_none(*pmd_ptr) || pmd_bad(*pmd_ptr)) {
		KGSL_MEM_ERR
		    ("Invalid pmd entry found while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}

	pte_ptr = pte_offset_map(pmd_ptr, vaddr);
	if (!pte_ptr) {
		KGSL_MEM_ERR
		    ("Unable to map pte entry while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}
	return pte_ptr;
}

int kgsl_mmu_map(struct kgsl_pagetable *pagetable,
				unsigned int address,
				int range,
				unsigned int protflags,
				unsigned int *gpuaddr,
				unsigned int flags)
{
	int numpages;
	unsigned int pte, superpte, ptefirst, ptelast, physaddr;
	int flushtlb, alloc_size;
	struct kgsl_mmu *mmu = NULL;
	int phys_contiguous = flags & KGSL_MEMFLAGS_CONPHYS;
	unsigned int align = flags & KGSL_MEMFLAGS_ALIGN_MASK;

	KGSL_MEM_VDBG("enter (pt=%p, physaddr=%08x, range=%08d, gpuaddr=%p)\n",
		      pagetable, address, range, gpuaddr);

	mmu = pagetable->mmu;

	BUG_ON(mmu == NULL);
	BUG_ON(protflags & ~(GSL_PT_PAGE_RV | GSL_PT_PAGE_WV));
	BUG_ON(protflags == 0);
	BUG_ON(range <= 0);

	/* Only support 4K and 8K alignment for now */
	if (align != KGSL_MEMFLAGS_ALIGN8K && align != KGSL_MEMFLAGS_ALIGN4K) {
		KGSL_MEM_ERR("Cannot map memory according to "
			     "requested flags: %08x\n", flags);
		return -EINVAL;
	}

	/* Make sure address being mapped is at 4K boundary */
	if (!IS_ALIGNED(address, KGSL_PAGESIZE) || range & ~KGSL_PAGEMASK) {
		KGSL_MEM_ERR("Cannot map address not aligned "
			     "at page boundary: address: %08x, range: %08x\n",
			     address, range);
		return -EINVAL;
	}
	alloc_size = range;
	if (align == KGSL_MEMFLAGS_ALIGN8K)
		alloc_size += KGSL_PAGESIZE;

	*gpuaddr = gen_pool_alloc(pagetable->pool, alloc_size);
	if (*gpuaddr == 0) {
		KGSL_MEM_ERR("gen_pool_alloc failed: %d\n", alloc_size);
		return -ENOMEM;
	}

	if (align == KGSL_MEMFLAGS_ALIGN8K) {
		if (*gpuaddr & ((1 << 13) - 1)) {
			/* Not 8k aligned, align it */
			gen_pool_free(pagetable->pool, *gpuaddr, KGSL_PAGESIZE);
			*gpuaddr = *gpuaddr + KGSL_PAGESIZE;
		} else
			gen_pool_free(pagetable->pool, *gpuaddr + range,
				      KGSL_PAGESIZE);
	}

	numpages = (range >> KGSL_PAGESIZE_SHIFT);

	ptefirst = kgsl_pt_entry_get(pagetable, *gpuaddr);
	ptelast = ptefirst + numpages;

	pte = ptefirst;
	flushtlb = 0;

	superpte = ptefirst & (GSL_PT_SUPER_PTE - 1);
	for (pte = superpte; pte < ptefirst; pte++) {
		/* tlb needs to be flushed only when a dirty superPTE
		   gets backed */
		if (kgsl_pt_map_isdirty(pagetable, pte)) {
			flushtlb = 1;
			break;
		}
	}

	for (pte = ptefirst; pte < ptelast; pte++) {
#ifdef VERBOSE_DEBUG
		/* check if PTE exists */
		uint32_t val = kgsl_pt_map_getaddr(pagetable, pte);
		BUG_ON(val != 0 && val != GSL_PT_PAGE_DIRTY);
#endif
		if (kgsl_pt_map_isdirty(pagetable, pte))
			flushtlb = 1;
		/* mark pte as in use */
		if (phys_contiguous)
			physaddr = address;
		else {
			physaddr = vmalloc_to_pfn((void *)address);
			physaddr <<= PAGE_SHIFT;
		}

		if (physaddr)
			kgsl_pt_map_set(pagetable, pte, physaddr | protflags);
		else {
			KGSL_MEM_ERR
			("Unable to find physaddr for vmallloc address: %x\n",
			     address);
			kgsl_mmu_unmap(pagetable, *gpuaddr, range);
			return -EFAULT;
		}
		address += KGSL_PAGESIZE;
	}

	/* set superpte to end of next superpte */
	superpte = (ptelast + (GSL_PT_SUPER_PTE - 1))
			& (GSL_PT_SUPER_PTE - 1);
	for (pte = ptelast; pte < superpte; pte++) {
		/* tlb needs to be flushed only when a dirty superPTE
		   gets backed */
		if (kgsl_pt_map_isdirty(pagetable, pte)) {
			flushtlb = 1;
			break;
		}
	}
	KGSL_MEM_INFO("pt %p p %08x g %08x pte f %d l %d n %d f %d\n",
		      pagetable, address, *gpuaddr, ptefirst, ptelast,
		      numpages, flushtlb);

	dmb();

	/* Invalidate tlb only if current page table used by GPU is the
	 * pagetable that we used to allocate */
	if (pagetable == mmu->hwpagetable)
		kgsl_yamato_setstate(mmu->device, KGSL_MMUFLAGS_TLBFLUSH);


	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}

int
kgsl_mmu_unmap(struct kgsl_pagetable *pagetable, unsigned int gpuaddr,
		int range)
{
	unsigned int numpages;
	unsigned int pte, ptefirst, ptelast;

	KGSL_MEM_VDBG("enter (pt=%p, gpuaddr=0x%08x, range=%d)\n",
			pagetable, gpuaddr, range);

	BUG_ON(range <= 0);

	numpages = (range >> KGSL_PAGESIZE_SHIFT);
	if (range & (KGSL_PAGESIZE - 1))
		numpages++;

	ptefirst = kgsl_pt_entry_get(pagetable, gpuaddr);
	ptelast = ptefirst + numpages;

	KGSL_MEM_INFO("pt %p gpu %08x pte first %d last %d numpages %d\n",
		      pagetable, gpuaddr, ptefirst, ptelast, numpages);

	for (pte = ptefirst; pte < ptelast; pte++) {
#ifdef VERBOSE_DEBUG
		/* check if PTE exists */
		BUG_ON(!kgsl_pt_map_getaddr(pagetable, pte));
#endif
		kgsl_pt_map_set(pagetable, pte, GSL_PT_PAGE_DIRTY);
	}

	dmb();

	/* Invalidate tlb only if current page table used by GPU is the
	 * pagetable that we used to allocate */
	if (pagetable == pagetable->mmu->hwpagetable)
		kgsl_yamato_setstate(pagetable->mmu->device,
					KGSL_MMUFLAGS_TLBFLUSH);

	gen_pool_free(pagetable->pool, gpuaddr, range);

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}
#endif

int kgsl_mmu_close(struct kgsl_device *device)
{
	/*
	 *  close device mmu
	 *
	 *  call this with the global lock held
	 */
	struct kgsl_mmu *mmu = &device->mmu;
#ifdef _DEBUG
	int i;
#endif /* _DEBUG */

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_INITIALIZED0) {
		/* disable mh interrupts */
		KGSL_MEM_DBG("disabling mmu interrupts\n");
		kgsl_yamato_regwrite(device, REG_MH_INTERRUPT_MASK, 0);

		/* disable MMU */
		kgsl_yamato_regwrite(device, REG_MH_MMU_CONFIG, 0x00000000);

		if (mmu->dummyspace.gpuaddr)
			kgsl_sharedmem_free(&mmu->dummyspace);

		mmu->flags &= ~KGSL_FLAGS_STARTED;
		mmu->flags &= ~KGSL_FLAGS_INITIALIZED;
		mmu->flags &= ~KGSL_FLAGS_INITIALIZED0;
		kgsl_mmu_destroypagetableobject(mmu->defaultpagetable);
		mmu->defaultpagetable = NULL;
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}
