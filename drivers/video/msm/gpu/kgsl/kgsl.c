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
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/android_pmem.h>
#include <linux/highmem.h>
#include <linux/vmalloc.h>
#include <asm/cacheflush.h>

#include <asm/atomic.h>

#include "kgsl.h"
#include "kgsl_drawctxt.h"
#include "kgsl_ringbuffer.h"
#include "kgsl_cmdstream.h"
#include "kgsl_log.h"

struct kgsl_file_private {
	struct list_head	list;
	struct list_head	mem_list;
	uint32_t		ctxt_id_mask;
	struct kgsl_pagetable	*pagetable;
	unsigned long		vmalloc_size;
};

static void kgsl_put_phys_file(struct file *file);

#ifdef CONFIG_MSM_KGSL_MMU
static long flush_l1_cache_range(unsigned long addr, int size)
{
	struct page *page;
	pte_t *pte_ptr;
	unsigned long end;

	for (end = addr; end < (addr + size); end += KGSL_PAGESIZE) {
		pte_ptr = kgsl_get_pte_from_vaddr(end);
		if (!pte_ptr)
			return -EINVAL;

		page = pte_page(pte_val(*pte_ptr));
		if (!page) {
			KGSL_DRV_ERR("could not find page for pte\n");
			pte_unmap(pte_ptr);
			return -EINVAL;
		}

		pte_unmap(pte_ptr);
		flush_dcache_page(page);
	}

	return 0;
}

static long flush_l1_cache_all(struct kgsl_file_private *private)
{
	int result = 0;
	struct kgsl_mem_entry *entry = NULL;

	kgsl_yamato_runpending(&kgsl_driver.yamato_device);
	list_for_each_entry(entry, &private->mem_list, list) {
		if (KGSL_MEMFLAGS_MEM_REQUIRES_FLUSH & entry->memdesc.priv) {
			result =
			    flush_l1_cache_range((unsigned long)entry->
						 memdesc.hostptr,
						 entry->memdesc.size);
			if (result)
				goto done;
		}
	}
done:
	return result;
}
#else
static inline long flush_l1_cache_range(unsigned long addr, int size)
{ return 0; }

static inline long flush_l1_cache_all(struct kgsl_file_private *private)
{ return 0; }
#endif

/*this is used for logging, so that we can call the dev_printk
 functions without export struct kgsl_driver everywhere*/
struct device *kgsl_driver_getdevnode(void)
{
	BUG_ON(kgsl_driver.pdev == NULL);
	return &kgsl_driver.pdev->dev;
}

/* the hw and clk enable/disable funcs must be either called from softirq or
 * with mutex held */
static void kgsl_clk_enable(void)
{
	clk_set_rate(kgsl_driver.ebi1_clk, 128000000);
	clk_enable(kgsl_driver.imem_clk);
	if (kgsl_driver.grp_pclk)
		clk_enable(kgsl_driver.grp_pclk);
	clk_enable(kgsl_driver.grp_clk);
}

static void kgsl_clk_disable(void)
{
	clk_disable(kgsl_driver.grp_clk);
	if (kgsl_driver.grp_pclk)
		clk_disable(kgsl_driver.grp_pclk);
	clk_disable(kgsl_driver.imem_clk);
	clk_set_rate(kgsl_driver.ebi1_clk, 0);
}

static void kgsl_hw_disable(void)
{
	kgsl_driver.active = false;
	disable_irq(kgsl_driver.interrupt_num);
	kgsl_clk_disable();
	pr_debug("kgsl: hw disabled\n");
	wake_unlock(&kgsl_driver.wake_lock);
}

static void kgsl_hw_enable(void)
{
	wake_lock(&kgsl_driver.wake_lock);
	kgsl_clk_enable();
	enable_irq(kgsl_driver.interrupt_num);
	kgsl_driver.active = true;
	pr_debug("kgsl: hw enabled\n");
}

static void kgsl_hw_get_locked(void)
{
	/* active_cnt is protected by driver mutex */
	if (kgsl_driver.active_cnt++ == 0) {
		if (kgsl_driver.active) {
			del_timer_sync(&kgsl_driver.standby_timer);
			barrier();
		}
		if (!kgsl_driver.active)
			kgsl_hw_enable();
	}
}

static void kgsl_hw_put_locked(bool start_timer)
{
	if ((--kgsl_driver.active_cnt == 0) && start_timer) {
		mod_timer(&kgsl_driver.standby_timer,
			  jiffies + msecs_to_jiffies(20));
	}
}

static void kgsl_do_standby_timer(unsigned long data)
{
	if (kgsl_yamato_is_idle(&kgsl_driver.yamato_device))
		kgsl_hw_disable();
	else
		mod_timer(&kgsl_driver.standby_timer,
			  jiffies + msecs_to_jiffies(10));
}

/* file operations */
static int kgsl_first_open_locked(void)
{
	int result = 0;

	BUG_ON(kgsl_driver.active);
	BUG_ON(kgsl_driver.active_cnt);

	kgsl_clk_enable();

	/* init memory apertures */
	result = kgsl_sharedmem_init(&kgsl_driver.shmem);
	if (result != 0)
		goto done;

	/* init devices */
	result = kgsl_yamato_init(&kgsl_driver.yamato_device,
					&kgsl_driver.yamato_config);
	if (result != 0)
		goto done;

	result = kgsl_yamato_start(&kgsl_driver.yamato_device, 0);
	if (result != 0)
		goto done;

done:
	kgsl_clk_disable();
	return result;
}

static int kgsl_last_release_locked(void)
{
	BUG_ON(kgsl_driver.active_cnt);

	disable_irq(kgsl_driver.interrupt_num);

	kgsl_yamato_stop(&kgsl_driver.yamato_device);

	/* close devices */
	kgsl_yamato_close(&kgsl_driver.yamato_device);

	/* shutdown memory apertures */
	kgsl_sharedmem_close(&kgsl_driver.shmem);

	kgsl_clk_disable();
	kgsl_driver.active = false;
	wake_unlock(&kgsl_driver.wake_lock);

	return 0;
}

static int kgsl_release(struct inode *inodep, struct file *filep)
{
	int result = 0;
	unsigned int i;
	struct kgsl_mem_entry *entry, *entry_tmp;
	struct kgsl_file_private *private = NULL;

	mutex_lock(&kgsl_driver.mutex);

	private = filep->private_data;
	BUG_ON(private == NULL);
	filep->private_data = NULL;
	list_del(&private->list);

	kgsl_hw_get_locked();

	for (i = 0; i < KGSL_CONTEXT_MAX; i++)
		if (private->ctxt_id_mask & (1 << i))
			kgsl_drawctxt_destroy(&kgsl_driver.yamato_device, i);

	list_for_each_entry_safe(entry, entry_tmp, &private->mem_list, list)
		kgsl_remove_mem_entry(entry);

	if (private->pagetable != NULL) {
		kgsl_yamato_cleanup_pt(&kgsl_driver.yamato_device,
					private->pagetable);
		kgsl_mmu_destroypagetableobject(private->pagetable);
		private->pagetable = NULL;
	}

	kfree(private);

	if (atomic_dec_return(&kgsl_driver.open_count) == 0) {
		KGSL_DRV_VDBG("last_release\n");
		kgsl_hw_put_locked(false);
		result = kgsl_last_release_locked();
	} else
		kgsl_hw_put_locked(true);

	mutex_unlock(&kgsl_driver.mutex);

	return result;
}

static int kgsl_open(struct inode *inodep, struct file *filep)
{
	int result = 0;
	struct kgsl_file_private *private = NULL;

	KGSL_DRV_DBG("file %p pid %d\n", filep, task_pid_nr(current));


	if (filep->f_flags & O_EXCL) {
		KGSL_DRV_ERR("O_EXCL not allowed\n");
		return -EBUSY;
	}

	private = kzalloc(sizeof(*private), GFP_KERNEL);
	if (private == NULL) {
		KGSL_DRV_ERR("cannot allocate file private data\n");
		return -ENOMEM;
	}

	mutex_lock(&kgsl_driver.mutex);

	private->ctxt_id_mask = 0;
	INIT_LIST_HEAD(&private->mem_list);

	filep->private_data = private;

	list_add(&private->list, &kgsl_driver.client_list);

	if (atomic_inc_return(&kgsl_driver.open_count) == 1) {
		result = kgsl_first_open_locked();
		if (result != 0)
			goto done;
	}

	kgsl_hw_get_locked();

	/*NOTE: this must happen after first_open */
	private->pagetable =
		kgsl_mmu_createpagetableobject(&kgsl_driver.yamato_device.mmu);
	if (private->pagetable == NULL) {
		result = -ENOMEM;
		goto done;
	}
	result = kgsl_yamato_setup_pt(&kgsl_driver.yamato_device,
					private->pagetable);
	if (result) {
		kgsl_mmu_destroypagetableobject(private->pagetable);
		private->pagetable = NULL;
		goto done;
	}
	private->vmalloc_size = 0;
done:
	kgsl_hw_put_locked(true);
	mutex_unlock(&kgsl_driver.mutex);
	if (result != 0)
		kgsl_release(inodep, filep);
	return result;
}


/*call with driver locked */
static struct kgsl_mem_entry *
kgsl_sharedmem_find(struct kgsl_file_private *private, unsigned int gpuaddr)
{
	struct kgsl_mem_entry *entry = NULL, *result = NULL;

	BUG_ON(private == NULL);

	list_for_each_entry(entry, &private->mem_list, list) {
		if (entry->memdesc.gpuaddr == gpuaddr) {
			result = entry;
			break;
		}
	}
	return result;
}

/*call with driver locked */
struct kgsl_mem_entry *
kgsl_sharedmem_find_region(struct kgsl_file_private *private,
				unsigned int gpuaddr,
				size_t size)
{
	struct kgsl_mem_entry *entry = NULL, *result = NULL;

	BUG_ON(private == NULL);

	list_for_each_entry(entry, &private->mem_list, list) {
		if (gpuaddr >= entry->memdesc.gpuaddr &&
		    ((gpuaddr + size) <=
			(entry->memdesc.gpuaddr + entry->memdesc.size))) {
			result = entry;
			break;
		}
	}

	return result;
}

/*call all ioctl sub functions with driver locked*/

static long kgsl_ioctl_device_getproperty(struct kgsl_file_private *private,
					 void __user *arg)
{
	int result = 0;
	struct kgsl_device_getproperty param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
	result = kgsl_yamato_getproperty(&kgsl_driver.yamato_device,
					 param.type,
					 param.value, param.sizebytes);
done:
	return result;
}

static long kgsl_ioctl_device_regread(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_device_regread param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
	result = kgsl_yamato_regread(&kgsl_driver.yamato_device,
				     param.offsetwords, &param.value);
	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}


static long kgsl_ioctl_device_waittimestamp(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_device_waittimestamp param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	/* Don't wait forever, set a max value for now */
	if (param.timeout == -1)
		param.timeout = 10 * MSEC_PER_SEC;
	result = kgsl_yamato_waittimestamp(&kgsl_driver.yamato_device,
				     param.timestamp,
				     param.timeout);

	kgsl_yamato_runpending(&kgsl_driver.yamato_device);
done:
	return result;
}

static long kgsl_ioctl_rb_issueibcmds(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_ringbuffer_issueibcmds param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.drawctxt_id >= KGSL_CONTEXT_MAX
		|| (private->ctxt_id_mask & 1 << param.drawctxt_id) == 0) {
		result = -EINVAL;
		KGSL_DRV_ERR("invalid drawctxt drawctxt_id %d\n",
			      param.drawctxt_id);
		result = -EINVAL;
		goto done;
	}

	if (kgsl_sharedmem_find_region(private, param.ibaddr,
				param.sizedwords*sizeof(uint32_t)) == NULL) {
		KGSL_DRV_ERR("invalid cmd buffer ibaddr %08x sizedwords %d\n",
			      param.ibaddr, param.sizedwords);
		result = -EINVAL;
		goto done;

	}

	result = kgsl_ringbuffer_issueibcmds(&kgsl_driver.yamato_device,
					     param.drawctxt_id,
					     param.ibaddr,
					     param.sizedwords,
					     &param.timestamp,
					     param.flags);
	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}

static long kgsl_ioctl_cmdstream_readtimestamp(struct kgsl_file_private
						*private, void __user *arg)
{
	int result = 0;
	struct kgsl_cmdstream_readtimestamp param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	param.timestamp =
		kgsl_cmdstream_readtimestamp(&kgsl_driver.yamato_device,
							param.type);
	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}

static long kgsl_ioctl_cmdstream_freememontimestamp(struct kgsl_file_private
						*private, void __user *arg)
{
	int result = 0;
	struct kgsl_cmdstream_freememontimestamp param;
	struct kgsl_mem_entry *entry = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	entry = kgsl_sharedmem_find(private, param.gpuaddr);
	if (entry == NULL) {
		KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
		result = -EINVAL;
		goto done;
	}

	if (entry->memdesc.priv & KGSL_MEMFLAGS_VMALLOC_MEM)
		entry->memdesc.priv &= ~KGSL_MEMFLAGS_MEM_REQUIRES_FLUSH;

	result = kgsl_cmdstream_freememontimestamp(&kgsl_driver.yamato_device,
							entry,
							param.timestamp,
							param.type);

	kgsl_yamato_runpending(&kgsl_driver.yamato_device);

done:
	return result;
}

static long kgsl_ioctl_drawctxt_create(struct kgsl_file_private *private,
				      void __user *arg)
{
	int result = 0;
	struct kgsl_drawctxt_create param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	result = kgsl_drawctxt_create(&kgsl_driver.yamato_device,
					private->pagetable,
					param.flags,
					&param.drawctxt_id);
	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	private->ctxt_id_mask |= 1 << param.drawctxt_id;

done:
	return result;
}

static long kgsl_ioctl_drawctxt_destroy(struct kgsl_file_private *private,
				       void __user *arg)
{
	int result = 0;
	struct kgsl_drawctxt_destroy param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.drawctxt_id >= KGSL_CONTEXT_MAX
		|| (private->ctxt_id_mask & 1 << param.drawctxt_id) == 0) {
		result = -EINVAL;
		goto done;
	}

	result = kgsl_drawctxt_destroy(&kgsl_driver.yamato_device,
					param.drawctxt_id);
	if (result == 0)
		private->ctxt_id_mask &= ~(1 << param.drawctxt_id);

done:
	return result;
}

void kgsl_remove_mem_entry(struct kgsl_mem_entry *entry)
{
	kgsl_mmu_unmap(entry->memdesc.pagetable,
		       entry->memdesc.gpuaddr & KGSL_PAGEMASK,
		       entry->memdesc.size);
	if (KGSL_MEMFLAGS_VMALLOC_MEM & entry->memdesc.priv) {
		vfree((void *)entry->memdesc.physaddr);
		entry->priv->vmalloc_size -= entry->memdesc.size;
	} else
		kgsl_put_phys_file(entry->pmem_file);
	list_del(&entry->list);

	if (entry->free_list.prev)
		list_del(&entry->free_list);

	kfree(entry);

}

static long kgsl_ioctl_sharedmem_free(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_sharedmem_free param;
	struct kgsl_mem_entry *entry = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	entry = kgsl_sharedmem_find(private, param.gpuaddr);
	if (entry == NULL) {
		KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
		result = -EINVAL;
		goto done;
	}

	kgsl_remove_mem_entry(entry);
done:
	return result;
}

#ifdef CONFIG_MSM_KGSL_MMU
static int kgsl_ioctl_sharedmem_from_vmalloc(struct kgsl_file_private *private,
					     void __user *arg)
{
	int result = 0, len;
	struct kgsl_sharedmem_from_vmalloc param;
	struct kgsl_mem_entry *entry = NULL;
	void *vmalloc_area;
	struct vm_area_struct *vma;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto error;
	}

	if (!param.hostptr) {
		KGSL_DRV_ERR
		    ("Invalid host pointer of malloc passed: param.hostptr "
		     "%08x\n", param.hostptr);
		result = -EINVAL;
		goto error;
	}

	vma = find_vma(current->mm, param.hostptr);
	if (!vma) {
		KGSL_MEM_ERR("Could not find vma for address %x\n",
			     param.hostptr);
		result = -EINVAL;
		goto error;
	}
	len = vma->vm_end - vma->vm_start;
	if (vma->vm_pgoff || !IS_ALIGNED(len, PAGE_SIZE)
	    || !IS_ALIGNED(vma->vm_start, PAGE_SIZE)) {
		KGSL_MEM_ERR
		("kgsl vmalloc mapping must be at offset 0 and page aligned\n");
		result = -EINVAL;
		goto error;
	}
	if (vma->vm_start != param.hostptr) {
		KGSL_MEM_ERR
		    ("vma start address is not equal to mmap address\n");
		result = -EINVAL;
		goto error;
	}

	if ((private->vmalloc_size + len) > KGSL_GRAPHICS_MEMORY_LOW_WATERMARK
	    && !param.force_no_low_watermark) {
		result = -ENOMEM;
		goto error;
	}

	entry = kzalloc(sizeof(struct kgsl_mem_entry), GFP_KERNEL);
	if (entry == NULL) {
		result = -ENOMEM;
		goto error;
	}

	/* allocate memory and map it to user space */
	vmalloc_area = vmalloc_user(len);
	if (!vmalloc_area) {
		KGSL_MEM_ERR("vmalloc failed\n");
		result = -ENOMEM;
		goto error_free_entry;
	}
	if (!kgsl_cache_enable) {
		/* If we are going to map non-cached, make sure to flush the
		 * cache to ensure that previously cached data does not
		 * overwrite this memory */
		dmac_flush_range(vmalloc_area, vmalloc_area + len);
		KGSL_MEM_INFO("Caching for memory allocation turned off\n");
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	} else {
		KGSL_MEM_INFO("Caching for memory allocation turned on\n");
	}

	result = remap_vmalloc_range(vma, vmalloc_area, 0);
	if (result) {
		KGSL_MEM_ERR("remap_vmalloc_range returned %d\n", result);
		goto error_free_vmalloc;
	}

	result =
	    kgsl_mmu_map(private->pagetable, (unsigned long)vmalloc_area, len,
			 GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
			 &entry->memdesc.gpuaddr, KGSL_MEMFLAGS_ALIGN4K);

	if (result != 0)
		goto error_free_vmalloc;

	entry->memdesc.pagetable = private->pagetable;
	entry->memdesc.size = len;
	entry->memdesc.hostptr = (void *)param.hostptr;
	entry->memdesc.priv = KGSL_MEMFLAGS_VMALLOC_MEM |
	    KGSL_MEMFLAGS_MEM_REQUIRES_FLUSH;
	entry->memdesc.physaddr = (unsigned long)vmalloc_area;
	entry->priv = private;

	param.gpuaddr = entry->memdesc.gpuaddr;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto error_unmap_entry;
	}
	private->vmalloc_size += len;
	list_add(&entry->list, &private->mem_list);

	return 0;

error_unmap_entry:
	kgsl_mmu_unmap(private->pagetable, entry->memdesc.gpuaddr,
		       entry->memdesc.size);

error_free_vmalloc:
	vfree(vmalloc_area);

error_free_entry:
	kfree(entry);

error:
	return result;
}
#else
static inline int kgsl_ioctl_sharedmem_from_vmalloc(
			struct kgsl_file_private *private, void __user *arg)
{
	return -ENOSYS;
}
#endif

static int kgsl_get_phys_file(int fd, unsigned long *start, unsigned long *len,
			      struct file **filep)
{
	struct file *fbfile;
	int put_needed;
	unsigned long vstart = 0;
	int ret = 0;
	dev_t rdev;
	struct fb_info *info;

	*filep = NULL;
	if (!get_pmem_file(fd, start, &vstart, len, filep))
		return 0;

	fbfile = fget_light(fd, &put_needed);
	if (fbfile == NULL)
		return -1;

	rdev = fbfile->f_dentry->d_inode->i_rdev;
	info = MAJOR(rdev) == FB_MAJOR ? registered_fb[MINOR(rdev)] : NULL;
	if (info) {
		*start = info->fix.smem_start;
		*len = info->fix.smem_len;
		ret = 0;
	} else
		ret = -1;
	fput_light(fbfile, put_needed);

	return ret;
}

static void kgsl_put_phys_file(struct file *file)
{
	KGSL_DRV_DBG("put phys file %p\n", file);
	if (file)
		put_pmem_file(file);
}

static int kgsl_ioctl_sharedmem_from_pmem(struct kgsl_file_private *private,
						void __user *arg)
{
	int result = 0;
	struct kgsl_sharedmem_from_pmem param;
	struct kgsl_mem_entry *entry = NULL;
	unsigned long start = 0, len = 0;
	struct file *pmem_file = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto error;
	}

	if (kgsl_get_phys_file(param.pmem_fd, &start, &len, &pmem_file)) {
		result = -EINVAL;
		goto error;
	} else if (param.offset + param.len > len) {
		KGSL_DRV_ERR("%s: region too large 0x%x + 0x%x >= 0x%lx\n",
			     __func__, param.offset, param.len, len);
		result = -EINVAL;
		goto error_put_pmem;
	}

	KGSL_MEM_INFO("get phys file %p start 0x%lx len 0x%lx\n",
		      pmem_file, start, len);
	KGSL_DRV_DBG("locked phys file %p\n", pmem_file);

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (entry == NULL) {
		result = -ENOMEM;
		goto error_put_pmem;
	}

	entry->pmem_file = pmem_file;

	entry->memdesc.pagetable = private->pagetable;

	/* Any MMU mapped memory must have a length in multiple of PAGESIZE */
	entry->memdesc.size = ALIGN(param.len, PAGE_SIZE);

	/*we shouldn't need to write here from kernel mode */
	entry->memdesc.hostptr = NULL;

	/* ensure that MMU mappings are at page boundary */
	entry->memdesc.physaddr = start + (param.offset & KGSL_PAGEMASK);
	result = kgsl_mmu_map(private->pagetable, entry->memdesc.physaddr,
			entry->memdesc.size, GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
			&entry->memdesc.gpuaddr,
			KGSL_MEMFLAGS_ALIGN4K | KGSL_MEMFLAGS_CONPHYS);
	if (result)
		goto error_free_entry;

	/* If the offset is not at 4K boundary then add the correct offset
	 * value to gpuaddr */
	entry->memdesc.gpuaddr += (param.offset & ~KGSL_PAGEMASK);
	param.gpuaddr = entry->memdesc.gpuaddr;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto error_unmap_entry;
	}
	list_add(&entry->list, &private->mem_list);
	return result;

error_unmap_entry:
	kgsl_mmu_unmap(entry->memdesc.pagetable,
		       entry->memdesc.gpuaddr & KGSL_PAGEMASK,
		       entry->memdesc.size);
error_free_entry:
	kfree(entry);

error_put_pmem:
	kgsl_put_phys_file(pmem_file);

error:
	return result;
}

#ifdef CONFIG_MSM_KGSL_MMU
/*This function flushes a graphics memory allocation from CPU cache
 *when caching is enabled with MMU*/
static int kgsl_ioctl_sharedmem_flush_cache(struct kgsl_file_private *private,
				       void __user *arg)
{
	int result = 0;
	struct kgsl_mem_entry *entry;
	struct kgsl_sharedmem_free param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	entry = kgsl_sharedmem_find(private, param.gpuaddr);
	if (!entry) {
		KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
		result = -EINVAL;
		goto done;
	}
	result = flush_l1_cache_range((unsigned long)entry->memdesc.hostptr,
				      entry->memdesc.size);
	/* Mark memory as being flushed so we don't flush it again */
	entry->memdesc.priv &= ~KGSL_MEMFLAGS_MEM_REQUIRES_FLUSH;
done:
	return result;
}
#else
static int kgsl_ioctl_sharedmem_flush_cache(struct kgsl_file_private *private,
					    void __user *arg)
{
	return -ENOSYS;
}
#endif


static long kgsl_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int result = 0;
	struct kgsl_file_private *private = filep->private_data;
	struct kgsl_drawctxt_set_bin_base_offset binbase;

	BUG_ON(private == NULL);

	KGSL_DRV_VDBG("filep %p cmd 0x%08x arg 0x%08lx\n", filep, cmd, arg);

	mutex_lock(&kgsl_driver.mutex);

	kgsl_hw_get_locked();

	switch (cmd) {

	case IOCTL_KGSL_DEVICE_GETPROPERTY:
		result =
		    kgsl_ioctl_device_getproperty(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_DEVICE_REGREAD:
		result = kgsl_ioctl_device_regread(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_DEVICE_WAITTIMESTAMP:
		result = kgsl_ioctl_device_waittimestamp(private,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS:
		if (kgsl_cache_enable)
			flush_l1_cache_all(private);
		result = kgsl_ioctl_rb_issueibcmds(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_CMDSTREAM_READTIMESTAMP:
		result =
		    kgsl_ioctl_cmdstream_readtimestamp(private,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_CMDSTREAM_FREEMEMONTIMESTAMP:
		result =
		    kgsl_ioctl_cmdstream_freememontimestamp(private,
						    (void __user *)arg);
		break;

	case IOCTL_KGSL_DRAWCTXT_CREATE:
		result = kgsl_ioctl_drawctxt_create(private,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_DRAWCTXT_DESTROY:
		result =
		    kgsl_ioctl_drawctxt_destroy(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_SHAREDMEM_FREE:
		result = kgsl_ioctl_sharedmem_free(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_SHAREDMEM_FROM_VMALLOC:
		kgsl_yamato_runpending(&kgsl_driver.yamato_device);
		result = kgsl_ioctl_sharedmem_from_vmalloc(private,
							   (void __user *)arg);
		break;

	case IOCTL_KGSL_SHAREDMEM_FLUSH_CACHE:
		if (kgsl_cache_enable)
			result = kgsl_ioctl_sharedmem_flush_cache(private,
						       (void __user *)arg);
		break;
	case IOCTL_KGSL_SHAREDMEM_FROM_PMEM:
		kgsl_yamato_runpending(&kgsl_driver.yamato_device);
		result = kgsl_ioctl_sharedmem_from_pmem(private,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_DRAWCTXT_SET_BIN_BASE_OFFSET:
		if (copy_from_user(&binbase, (void __user *)arg,
				   sizeof(binbase))) {
			result = -EFAULT;
			break;
		}

		if (private->ctxt_id_mask & (1 << binbase.drawctxt_id)) {
			result = kgsl_drawctxt_set_bin_base_offset(
					&kgsl_driver.yamato_device,
					binbase.drawctxt_id,
					binbase.offset);
		} else {
			result = -EINVAL;
			KGSL_DRV_ERR("invalid drawctxt drawctxt_id %d\n",
				     binbase.drawctxt_id);
		}
		break;

	default:
		KGSL_DRV_ERR("invalid ioctl code %08x\n", cmd);
		result = -EINVAL;
		break;
	}

	kgsl_hw_put_locked(true);
	mutex_unlock(&kgsl_driver.mutex);
	KGSL_DRV_VDBG("result %d\n", result);
	return result;
}

static struct file_operations kgsl_fops = {
	.owner = THIS_MODULE,
	.release = kgsl_release,
	.open = kgsl_open,
	.unlocked_ioctl = kgsl_ioctl,
};


struct kgsl_driver kgsl_driver = {
	.misc = {
		 .name = DRIVER_NAME,
		 .minor = MISC_DYNAMIC_MINOR,
		 .fops = &kgsl_fops,
	 },
	.open_count = ATOMIC_INIT(0),
	.mutex = __MUTEX_INITIALIZER(kgsl_driver.mutex),
};

static void kgsl_driver_cleanup(void)
{

	wake_lock_destroy(&kgsl_driver.wake_lock);

	if (kgsl_driver.interrupt_num > 0) {
		if (kgsl_driver.have_irq) {
			free_irq(kgsl_driver.interrupt_num, NULL);
			kgsl_driver.have_irq = 0;
		}
		kgsl_driver.interrupt_num = 0;
	}

	if (kgsl_driver.grp_clk) {
		clk_put(kgsl_driver.grp_clk);
		kgsl_driver.grp_clk = NULL;
	}

	if (kgsl_driver.imem_clk != NULL) {
		clk_put(kgsl_driver.imem_clk);
		kgsl_driver.imem_clk = NULL;
	}

	if (kgsl_driver.ebi1_clk != NULL) {
		clk_put(kgsl_driver.ebi1_clk);
		kgsl_driver.ebi1_clk = NULL;
	}

	kgsl_driver.pdev = NULL;

}


static int __devinit kgsl_platform_probe(struct platform_device *pdev)
{
	int result = 0;
	struct clk *clk;
	struct resource *res = NULL;

	kgsl_debug_init();

	INIT_LIST_HEAD(&kgsl_driver.client_list);

	/*acquire clocks */
	BUG_ON(kgsl_driver.grp_clk != NULL);
	BUG_ON(kgsl_driver.imem_clk != NULL);
	BUG_ON(kgsl_driver.ebi1_clk != NULL);

	kgsl_driver.pdev = pdev;

	setup_timer(&kgsl_driver.standby_timer, kgsl_do_standby_timer, 0);
	wake_lock_init(&kgsl_driver.wake_lock, WAKE_LOCK_SUSPEND, "kgsl");

	clk = clk_get(&pdev->dev, "grp_clk");
	if (IS_ERR(clk)) {
		result = PTR_ERR(clk);
		KGSL_DRV_ERR("clk_get(grp_clk) returned %d\n", result);
		goto done;
	}
	kgsl_driver.grp_clk = clk;

	clk = clk_get(&pdev->dev, "grp_pclk");
	if (IS_ERR(clk)) {
		KGSL_DRV_ERR("no grp_pclk, continuing\n");
		clk = NULL;
	}
	kgsl_driver.grp_pclk = clk;

	clk = clk_get(&pdev->dev, "imem_clk");
	if (IS_ERR(clk)) {
		result = PTR_ERR(clk);
		KGSL_DRV_ERR("clk_get(imem_clk) returned %d\n", result);
		goto done;
	}
	kgsl_driver.imem_clk = clk;

	clk = clk_get(&pdev->dev, "ebi1_clk");
	if (IS_ERR(clk)) {
		result = PTR_ERR(clk);
		KGSL_DRV_ERR("clk_get(ebi1_clk) returned %d\n", result);
		goto done;
	}
	kgsl_driver.ebi1_clk = clk;

	/*acquire interrupt */
	kgsl_driver.interrupt_num = platform_get_irq(pdev, 0);
	if (kgsl_driver.interrupt_num <= 0) {
		KGSL_DRV_ERR("platform_get_irq() returned %d\n",
			       kgsl_driver.interrupt_num);
		result = -EINVAL;
		goto done;
	}

	result = request_irq(kgsl_driver.interrupt_num, kgsl_yamato_isr,
				IRQF_TRIGGER_HIGH, DRIVER_NAME, NULL);
	if (result) {
		KGSL_DRV_ERR("request_irq(%d) returned %d\n",
			      kgsl_driver.interrupt_num, result);
		goto done;
	}
	kgsl_driver.have_irq = 1;
	disable_irq(kgsl_driver.interrupt_num);

	result = kgsl_yamato_config(&kgsl_driver.yamato_config, pdev);
	if (result != 0)
		goto done;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "kgsl_phys_memory");
	if (res == NULL) {
		result = -EINVAL;
		goto done;
	}

	kgsl_driver.shmem.physbase = res->start;
	kgsl_driver.shmem.size = resource_size(res);

done:
	if (result)
		kgsl_driver_cleanup();
	else
		result = misc_register(&kgsl_driver.misc);

	return result;
}

static int kgsl_platform_remove(struct platform_device *pdev)
{

	kgsl_driver_cleanup();
	misc_deregister(&kgsl_driver.misc);

	return 0;
}

static int kgsl_platform_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	mutex_lock(&kgsl_driver.mutex);
	if (atomic_read(&kgsl_driver.open_count) > 0) {
		if (kgsl_driver.active)
			pr_err("%s: Suspending while active???\n", __func__);
	}
	mutex_unlock(&kgsl_driver.mutex);
	return 0;
}

static struct platform_driver kgsl_platform_driver = {
	.probe = kgsl_platform_probe,
	.remove = __devexit_p(kgsl_platform_remove),
	.suspend = kgsl_platform_suspend,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME
	}
};

static int __init kgsl_mod_init(void)
{
	return platform_driver_register(&kgsl_platform_driver);
}

static void __exit kgsl_mod_exit(void)
{
	platform_driver_unregister(&kgsl_platform_driver);
}

module_init(kgsl_mod_init);
module_exit(kgsl_mod_exit);

MODULE_AUTHOR("QUALCOMM");
MODULE_DESCRIPTION("3D graphics driver for QSD8x50 and MSM7x27");
MODULE_VERSION("1.0");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:kgsl");
