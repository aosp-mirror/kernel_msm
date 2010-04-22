/* arch/arm/mach-msm/hw3d.c
 *
 * Register/Interrupt access for userspace 3D library.
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 * Heavily modified: Dima Zavin <dima@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* #define DEBUG */
/* #define VERBOSE */

#include <linux/clk.h>
#include <linux/earlysuspend.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/msm_hw3d.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <asm/io.h>

#include <mach/board.h>

#if defined(VERBOSE)
#define VDBG(x...)	pr_debug(x)
#else
#define VDBG(x...)	do {} while(0)
#endif

struct mem_region {
	unsigned long		pbase;
	unsigned long		size;
	void __iomem		*vbase;
};

/* Device minor numbers for master and client */
#define MINOR_MASTER 0
#define MINOR_CLIENT 1

struct hw3d_info {
	dev_t devno;
	struct cdev	master_cdev;
	struct cdev	client_cdev;

	struct clk		*grp_clk;
	struct clk		*imem_clk;
	int			irq;

	struct mem_region	regions[HW3D_NUM_REGIONS];

	wait_queue_head_t	irq_wq;
	bool			irq_pending;
	bool			irq_en;
	bool			suspending;
	bool			revoking;
	bool			enabled;

	struct timer_list	revoke_timer;
	wait_queue_head_t	revoke_wq;
	wait_queue_head_t	revoke_done_wq;
	unsigned int		waiter_cnt;

	spinlock_t		lock;

	struct file		*client_file;
	struct task_struct	*client_task;

	struct early_suspend	early_suspend;
	struct wake_lock	wake_lock;
};
static struct hw3d_info *hw3d_info;

struct hw3d_data {
	struct vm_area_struct	*vmas[HW3D_NUM_REGIONS];
	struct mutex		mutex;
	bool			closing;
};

#define REGION_PAGE_ID(addr)		\
	((((uint32_t)(addr)) >> (28 - PAGE_SHIFT)) & 0xf)
#define REGION_PAGE_OFFS(addr)		\
	((((uint32_t)(addr)) & ~(0xf << (28 - PAGE_SHIFT))))

static int hw3d_open(struct inode *, struct file *);
static int hw3d_release(struct inode *, struct file *);
static int hw3d_mmap(struct file *, struct vm_area_struct *);
static int hw3d_flush(struct file *, fl_owner_t);
static long hw3d_ioctl(struct file *, unsigned int, unsigned long);

static void hw3d_vma_open(struct vm_area_struct *);
static void hw3d_vma_close(struct vm_area_struct *);

static struct file_operations hw3d_fops = {
	.open		= hw3d_open,
	.release	= hw3d_release,
	.mmap		= hw3d_mmap,
	.flush		= hw3d_flush,
	.unlocked_ioctl	= hw3d_ioctl,
};

static struct vm_operations_struct hw3d_vm_ops = {
	.open = hw3d_vma_open,
	.close = hw3d_vma_close,
};

static bool is_master(struct hw3d_info *info, struct file *file)
{
	int fmin = MINOR(file->f_dentry->d_inode->i_rdev);
	return fmin == MINOR(info->master_cdev.dev);
}

static bool is_client(struct hw3d_info *info, struct file *file)
{
	int fmin = MINOR(file->f_dentry->d_inode->i_rdev);
	return fmin == MINOR(info->client_cdev.dev);
}

inline static void locked_hw3d_irq_disable(struct hw3d_info *info)
{
	if (info->irq_en) {
		disable_irq_nosync(info->irq);
		info->irq_en = 0;
	}
}

inline static void locked_hw3d_irq_enable(struct hw3d_info *info)
{
	if (!info->irq_en) {
		enable_irq(info->irq);
		info->irq_en = 1;
	}
}

static void hw3d_disable_interrupt(struct hw3d_info *info)
{
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	locked_hw3d_irq_disable(info);
	spin_unlock_irqrestore(&info->lock, flags);
}

static irqreturn_t hw3d_irq_handler(int irq, void *data)
{
	struct hw3d_info *info = data;
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	locked_hw3d_irq_disable(info);
	info->irq_pending = 1;
	spin_unlock_irqrestore(&info->lock, flags);

	wake_up(&info->irq_wq);

	return IRQ_HANDLED;
}

static long hw3d_wait_for_interrupt(struct hw3d_info *info, struct file *filp)
{
	struct hw3d_data *data = filp->private_data;
	unsigned long flags;
	int ret;

	if (is_master(info, filp)) {
		pr_err("%s: cannot wait for irqs on master node\n", __func__);
		return -EPERM;
	}

	for (;;) {
		spin_lock_irqsave(&info->lock, flags);
		if (info->irq_pending) {
			info->irq_pending = 0;
			spin_unlock_irqrestore(&info->lock, flags);
			return 0;
		}
		locked_hw3d_irq_enable(info);
		spin_unlock_irqrestore(&info->lock, flags);

		ret = wait_event_interruptible(info->irq_wq,
					       info->irq_pending ||
					       info->revoking ||
					       data->closing);
		/* always make sure the irq gets disabled */
		if (ret == 0 && !info->irq_pending)
			ret = -EPIPE;
		if (ret < 0) {
			hw3d_disable_interrupt(info);
			return ret;
		}
	}

	return 0;
}

static long hw3d_wait_for_revoke(struct hw3d_info *info, struct file *filp)
{
	struct hw3d_data *data = filp->private_data;
	int ret;

	if (is_master(info, filp)) {
		pr_err("%s: cannot revoke on master node\n", __func__);
		return -EPERM;
	}

	ret = wait_event_interruptible(info->revoke_wq,
				       info->revoking ||
				       data->closing);
	if (ret == 0 && data->closing)
		ret = -EPIPE;
	if (ret < 0)
		return ret;
	return 0;
}

static void locked_hw3d_client_done(struct hw3d_info *info, int had_timer)
{
	if (info->enabled) {
		pr_debug("hw3d: was enabled\n");
		info->enabled = 0;
		clk_disable(info->grp_clk);
		clk_disable(info->imem_clk);
	}
	info->revoking = 0;

	/* double check that the irqs are disabled */
	locked_hw3d_irq_disable(info);

	if (had_timer)
		wake_unlock(&info->wake_lock);
	wake_up(&info->revoke_done_wq);
}

static void do_force_revoke(struct hw3d_info *info)
{
	unsigned long flags;

	/* at this point, the task had a chance to relinquish the gpu, but
	 * it hasn't. So, we kill it */
	spin_lock_irqsave(&info->lock, flags);
	pr_debug("hw3d: forcing revoke\n");
	locked_hw3d_irq_disable(info);
	if (info->client_task) {
		pr_info("hw3d: force revoke from pid=%d\n",
			info->client_task->pid);
		force_sig(SIGKILL, info->client_task);
		put_task_struct(info->client_task);
		info->client_task = NULL;
	}
	locked_hw3d_client_done(info, 1);
	pr_debug("hw3d: done forcing revoke\n");
	spin_unlock_irqrestore(&info->lock, flags);
}

#define REVOKE_TIMEOUT		(2 * HZ)
static void locked_hw3d_revoke(struct hw3d_info *info)
{
	/* force us to wait to suspend until the revoke is done. If the
	 * user doesn't release the gpu, the timer will turn off the gpu,
	 * and force kill the process. */
	wake_lock(&info->wake_lock);
	info->revoking = 1;
	wake_up(&info->revoke_wq);
	mod_timer(&info->revoke_timer, jiffies + REVOKE_TIMEOUT);
}

bool is_msm_hw3d_file(struct file *file)
{
	struct hw3d_info *info = hw3d_info;
	if (MAJOR(file->f_dentry->d_inode->i_rdev) == MAJOR(info->devno) &&
	    (is_master(info, file) || is_client(info, file)))
		return 1;
	return 0;
}

void put_msm_hw3d_file(struct file *file)
{
	if (!is_msm_hw3d_file(file))
		return;
	fput(file);
}

int get_msm_hw3d_file(int fd, uint32_t *offs, unsigned long *pbase,
		      unsigned long *len, struct file **filp)
{
	struct hw3d_info *info = hw3d_info;
	struct file *file;
	struct hw3d_data *data;
	uint32_t offset = HW3D_OFFSET_IN_REGION(*offs);
	int region = HW3D_REGION_ID(*offs);
	int ret = 0;

	if (unlikely(region >= HW3D_NUM_REGIONS)) {
		VDBG("hw3d: invalid region %d requested\n", region);
		return -EINVAL;
	} else if (unlikely(offset >= info->regions[region].size)) {
		VDBG("hw3d: offset %08x outside of the requested region %d\n",
		     offset, region);
		return -EINVAL;
	}

	file = fget(fd);
	if (unlikely(file == NULL)) {
		pr_info("%s: requested data from file descriptor that doesn't "
			"exist.", __func__);
		return -EINVAL;
	} else if (!is_msm_hw3d_file(file)) {
		ret = -1;
		goto err;
	}

	data = file->private_data;
	if (unlikely(!data)) {
		VDBG("hw3d: invalid file\n");
		ret = -EINVAL;
		goto err;
	}

	mutex_lock(&data->mutex);
	if (unlikely(!data->vmas[region])) {
		mutex_unlock(&data->mutex);
		VDBG("hw3d: requested hw3d region is not mapped\n");
		ret = -ENOENT;
		goto err;
	}

	*offs = offset;
	*pbase = info->regions[region].pbase;
	*filp = file;
	*len = data->vmas[region]->vm_end - data->vmas[region]->vm_start;
	mutex_unlock(&data->mutex);
	return 0;

err:
	fput(file);
	return ret;
}

static int hw3d_flush(struct file *filp, fl_owner_t id)
{
	struct hw3d_info *info = hw3d_info;
	struct hw3d_data *data = filp->private_data;

	if (!data) {
		pr_err("%s: no private data\n", __func__);
		return -EINVAL;
	}

	if (is_master(info, filp))
		return 0;
	pr_debug("hw3d: closing\n");
	/* releases any blocked ioctls */
	data->closing = 1;
	wake_up(&info->revoke_wq);
	wake_up(&info->irq_wq);
	return 0;
}

static int should_wakeup(struct hw3d_info *info, unsigned int cnt)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&info->lock, flags);
	ret = (cnt != info->waiter_cnt) ||
		(!info->suspending && !info->client_file);
	spin_unlock_irqrestore(&info->lock, flags);

	return ret;
}

static int locked_open_wait_for_gpu(struct hw3d_info *info,
				    unsigned long *flags)
{
	unsigned int my_cnt;
	int ret;

	my_cnt = ++info->waiter_cnt;
	pr_debug("%s: wait_for_open %d\n", __func__, my_cnt);

	/* in case there are other waiters, wake and release them. */
	wake_up(&info->revoke_done_wq);

	if (info->suspending)
		pr_info("%s: suspended, waiting (%d %d)\n", __func__,
			current->group_leader->pid, current->pid);
	if (info->client_file)
		pr_info("%s: has client, waiting (%d %d)\n", __func__,
			current->group_leader->pid, current->pid);
	spin_unlock_irqrestore(&info->lock, *flags);

	ret = wait_event_interruptible(info->revoke_done_wq,
				       should_wakeup(info, my_cnt));

	spin_lock_irqsave(&info->lock, *flags);
	pr_debug("%s: woke up (%d %d %p)\n", __func__,
		 info->waiter_cnt, info->suspending, info->client_file);

	if (ret >= 0) {
		if (my_cnt != info->waiter_cnt) {
			pr_info("%s: someone else asked for gpu after us %d:%d"
				"(%d %d)\n", __func__,
				current->group_leader->pid, current->pid,
				my_cnt, info->waiter_cnt);
			ret = -EBUSY;
		} else if (info->suspending || info->client_file) {
			pr_err("%s: couldn't get the gpu for %d:%d (%d %p)\n",
			       __func__, current->group_leader->pid,
			       current->pid, info->suspending,
			       info->client_file);
			ret = -EBUSY;
		} else
			ret = 0;
	}
	return ret;
}

static int hw3d_open(struct inode *inode, struct file *file)
{
	struct hw3d_info *info = hw3d_info;
	struct hw3d_data *data;
	unsigned long flags;
	int ret = 0;

	pr_info("%s: pid %d tid %d opening %s node\n", __func__,
		current->group_leader->pid, current->pid,
		is_master(info, file) ? "master" : "client");

	if (file->private_data != NULL)
		return -EINVAL;

	data = kzalloc(sizeof(struct hw3d_data), GFP_KERNEL);
	if (!data) {
		pr_err("%s: unable to allocate memory for hw3d_data.\n",
		       __func__);
		return -ENOMEM;
	}

	mutex_init(&data->mutex);
	file->private_data = data;

	/* master always succeeds, so we are done */
	if (is_master(info, file))
		return 0;

	spin_lock_irqsave(&info->lock, flags);
	if (info->client_file) {
		pr_debug("hw3d: have client_file, need revoke\n");
		locked_hw3d_revoke(info);
	}

	ret = locked_open_wait_for_gpu(info, &flags);
	if (ret < 0) {
		spin_unlock_irqrestore(&info->lock, flags);
		goto err;
	}
	pr_info("%s: pid %d tid %d got gpu\n", __func__,
		current->group_leader->pid, current->pid);

	info->client_file = file;
	get_task_struct(current->group_leader);
	info->client_task = current->group_leader;

	/* XXX: we enable these clocks if the client connects..
	 * probably not right? Should only turn the clocks on when the user
	 * tries to map the registers? */
	clk_enable(info->imem_clk);
	clk_enable(info->grp_clk);
	info->enabled = 1;

	spin_unlock_irqrestore(&info->lock, flags);
	return 0;

err:
	file->private_data = NULL;
	kfree(data);
	return ret;

}

static int hw3d_release(struct inode *inode, struct file *file)
{
	struct hw3d_info *info = hw3d_info;
	struct hw3d_data *data = file->private_data;
	unsigned long flags;

	BUG_ON(!data);

	file->private_data = NULL;

	if (is_master(info, file))
		goto done;

	pr_info("%s: in release for pid=%d tid=%d\n", __func__,
		current->group_leader->pid, current->pid);
	spin_lock_irqsave(&info->lock, flags);

	if (info->client_task && info->client_task == current->group_leader) {
		pr_debug("hw3d: releasing %d\n", info->client_task->pid);
		put_task_struct(info->client_task);
		info->client_task = NULL;
	}

	if (info->client_file && info->client_file == file) {
		int pending;
		/* this will be true if we are still the "owner" of the gpu */
		pr_debug("hw3d: had file\n");
		pending = del_timer(&info->revoke_timer);
		locked_hw3d_client_done(info, pending);
		info->client_file = NULL;
	} else
		pr_warning("hw3d: release without client_file.\n");
	spin_unlock_irqrestore(&info->lock, flags);

done:
	kfree(data);
	return 0;
}

static void hw3d_vma_open(struct vm_area_struct *vma)
{
	/* XXX: should the master be allowed to fork and keep the mappings? */

	/* TODO: remap garbage page into here.
	 *
	 * For now, just pull the mapping. The user shouldn't be forking
	 * and using it anyway. */
	zap_page_range(vma, vma->vm_start, vma->vm_end - vma->vm_start, NULL);
}

static void hw3d_vma_close(struct vm_area_struct *vma)
{
	struct file *file = vma->vm_file;
	struct hw3d_data *data = file->private_data;
	int i;

	pr_debug("hw3d: current %u ppid %u file %p count %ld\n",
		 current->pid, current->parent->pid, file, file_count(file));

	BUG_ON(!data);

	mutex_lock(&data->mutex);
	for (i = 0; i < HW3D_NUM_REGIONS; ++i) {
		if (data->vmas[i] == vma) {
			data->vmas[i] = NULL;
			goto done;
		}
	}
	pr_warning("%s: vma %p not of ours during vma_close\n", __func__, vma);
done:
	mutex_unlock(&data->mutex);
}

static int hw3d_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct hw3d_info *info = hw3d_info;
	struct hw3d_data *data = file->private_data;
	unsigned long vma_size = vma->vm_end - vma->vm_start;
	int ret = 0;
	int region = REGION_PAGE_ID(vma->vm_pgoff);

	if (region >= HW3D_NUM_REGIONS) {
		pr_err("%s: Trying to mmap unknown region %d\n", __func__,
		       region);
		return -EINVAL;
	} else if (vma_size > info->regions[region].size) {
		pr_err("%s: VMA size %ld exceeds region %d size %ld\n",
			__func__, vma_size, region,
			info->regions[region].size);
		return -EINVAL;
	} else if (REGION_PAGE_OFFS(vma->vm_pgoff) != 0 ||
		   (vma_size & ~PAGE_MASK)) {
		pr_err("%s: Can't remap part of the region %d\n", __func__,
		       region);
		return -EINVAL;
	} else if (!is_master(info, file) &&
		   current->group_leader != info->client_task) {
		pr_err("%s: current(%d) != client_task(%d)\n", __func__,
		       current->group_leader->pid, info->client_task->pid);
		return -EPERM;
	} else if (!is_master(info, file) &&
		   (info->revoking || info->suspending)) {
		pr_err("%s: cannot mmap while revoking(%d) or suspending(%d)\n",
		       __func__, info->revoking, info->suspending);
		return -EPERM;
	}

	mutex_lock(&data->mutex);
	if (data->vmas[region] != NULL) {
		pr_err("%s: Region %d already mapped (pid=%d tid=%d)\n",
		       __func__, region, current->group_leader->pid,
		       current->pid);
		ret = -EBUSY;
		goto done;
	}

	/* our mappings are always noncached */
#ifdef pgprot_noncached
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#endif

	ret = io_remap_pfn_range(vma, vma->vm_start,
				 info->regions[region].pbase >> PAGE_SHIFT,
				 vma_size, vma->vm_page_prot);
	if (ret) {
		pr_err("%s: Cannot remap page range for region %d!\n", __func__,
		       region);
		ret = -EAGAIN;
		goto done;
	}

	/* Prevent a malicious client from stealing another client's data
	 * by forcing a revoke on it and then mmapping the GPU buffers.
	 */
	if (region != HW3D_REGS)
		memset(info->regions[region].vbase, 0,
		       info->regions[region].size);

	vma->vm_ops = &hw3d_vm_ops;

	/* mark this region as mapped */
	data->vmas[region] = vma;

done:
	mutex_unlock(&data->mutex);
	return ret;
}

static long hw3d_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct hw3d_info *info = hw3d_info;
	struct hw3d_region regions[HW3D_NUM_REGIONS];
	int i;

	if (!file->private_data)
		return -EINVAL;

	switch (cmd) {
	case HW3D_WAIT_FOR_REVOKE:
		return hw3d_wait_for_revoke(info, file);

	case HW3D_WAIT_FOR_INTERRUPT:
		return hw3d_wait_for_interrupt(info, file);

	case HW3D_GET_REGIONS:
		for (i = 0; i < HW3D_NUM_REGIONS; ++i) {
			regions[i].phys = info->regions[i].pbase;
			regions[i].map_offset = HW3D_REGION_OFFSET(i);
			regions[i].len = info->regions[i].size;
		}
		if (copy_to_user((void __user *)arg, regions, sizeof(regions)))
			return -EFAULT;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void hw3d_early_suspend(struct early_suspend *h)
{
	unsigned long flags;
	struct hw3d_info *info;
	info = container_of(h, struct hw3d_info, early_suspend);

	spin_lock_irqsave(&info->lock, flags);
	info->suspending = 1;
	if (info->client_file) {
		pr_info("hw3d: Requesting revoke for suspend\n");
		locked_hw3d_revoke(info);
	}
	spin_unlock_irqrestore(&info->lock, flags);
}

static void hw3d_late_resume(struct early_suspend *h)
{
	unsigned long flags;
	struct hw3d_info *info;
	info = container_of(h, struct hw3d_info, early_suspend);

	spin_lock_irqsave(&info->lock, flags);
	if (info->suspending)
		pr_info("%s: resuming\n", __func__);
	info->suspending = 0;
	wake_up(&info->revoke_done_wq);
	spin_unlock_irqrestore(&info->lock, flags);
}

static int hw3d_resume(struct platform_device *pdev)
{
	struct hw3d_info *info = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	if (info->suspending)
		pr_info("%s: resuming\n", __func__);
	info->suspending = 0;
	spin_unlock_irqrestore(&info->lock, flags);
	return 0;
}

static int __init hw3d_probe(struct platform_device *pdev)
{
	struct hw3d_info *info;
#define DEV_MASTER MKDEV(MAJOR(info->devno), MINOR_MASTER)
#define DEV_CLIENT MKDEV(MAJOR(info->devno), MINOR_CLIENT)
	struct class *hw3d_class;
	struct device *master_dev;
	struct device *client_dev;
	struct resource *res[HW3D_NUM_REGIONS];
	int i;
	int irq;
	int ret = 0;

	res[HW3D_REGS] = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						      "regs");
	res[HW3D_SMI] = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "smi");
	res[HW3D_EBI] = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "ebi");
	irq = platform_get_irq(pdev, 0);
	if (!res[HW3D_REGS] || !res[HW3D_SMI] || !res[HW3D_EBI] || irq < 0) {
		pr_err("%s: incomplete resources\n", __func__);
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct hw3d_info), GFP_KERNEL);
	if (info == NULL) {
		pr_err("%s: Cannot allocate memory for hw3d_info\n", __func__);
		ret = -ENOMEM;
		goto err_alloc;
	}

	info->irq = irq;
	wake_lock_init(&info->wake_lock, WAKE_LOCK_SUSPEND, "hw3d_revoke_lock");
	spin_lock_init(&info->lock);
	init_waitqueue_head(&info->irq_wq);
	init_waitqueue_head(&info->revoke_wq);
	init_waitqueue_head(&info->revoke_done_wq);
	setup_timer(&info->revoke_timer,
		    (void (*)(unsigned long))do_force_revoke,
		    (unsigned long)info);

	platform_set_drvdata(pdev, info);

	info->grp_clk = clk_get(NULL, "grp_clk");
	if (IS_ERR(info->grp_clk)) {
		pr_err("%s: Cannot get grp_clk\n", __func__);
		ret = PTR_ERR(info->grp_clk);
		goto err_get_grp_clk;
	}

	info->imem_clk = clk_get(NULL, "imem_clk");
	if (IS_ERR(info->imem_clk)) {
		pr_err("%s: Cannot get imem_clk\n", __func__);
		ret = PTR_ERR(info->imem_clk);
		goto err_get_imem_clk;
	}

	for (i = 0; i < HW3D_NUM_REGIONS; ++i) {
		info->regions[i].pbase = res[i]->start;
		info->regions[i].size = res[i]->end - res[i]->start + 1;
		info->regions[i].vbase = ioremap(info->regions[i].pbase,
						 info->regions[i].size);
		if (info->regions[i].vbase == 0) {
			pr_err("%s: Cannot remap region %d\n", __func__, i);
			goto err_remap_region;
		}
	}

	hw3d_class = class_create(THIS_MODULE, "msm_hw3d");
	if (IS_ERR(hw3d_class))
		goto err_fail_create_class;

	ret = alloc_chrdev_region(&info->devno, 0, 2, "msm_hw3d");
	if (ret < 0)
		goto err_fail_alloc_region;

	/* register the master/client devices */
	master_dev = device_create(hw3d_class, &pdev->dev,
			DEV_MASTER, "%s", "msm_hw3dm");
	if (IS_ERR(master_dev))
		goto err_dev_master;
	cdev_init(&info->master_cdev, &hw3d_fops);
	info->master_cdev.owner = THIS_MODULE;
	ret = cdev_add(&info->master_cdev, DEV_MASTER, 1);
	if (ret < 0) {
		pr_err("%s: Cannot register master device node\n", __func__);
		goto err_reg_master;
	}

	client_dev = device_create(hw3d_class, &pdev->dev,
			DEV_CLIENT, "%s", "msm_hw3dc");
	if (IS_ERR(client_dev))
		goto err_dev_client;
	cdev_init(&info->client_cdev, &hw3d_fops);
	info->client_cdev.owner = THIS_MODULE;
	ret = cdev_add(&info->client_cdev, DEV_CLIENT, 1);
	if (ret < 0) {
		pr_err("%s: Cannot register client device node\n", __func__);
		goto err_reg_client;
	}

	info->early_suspend.suspend = hw3d_early_suspend;
	info->early_suspend.resume = hw3d_late_resume;
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&info->early_suspend);

	info->irq_en = 1;
	ret = request_irq(info->irq, hw3d_irq_handler, IRQF_TRIGGER_HIGH,
			  "hw3d", info);
	if (ret != 0) {
		pr_err("%s: Cannot request irq\n", __func__);
		goto err_req_irq;
	}
	hw3d_disable_interrupt(info);

	hw3d_info = info;

	return 0;

err_req_irq:
	unregister_early_suspend(&info->early_suspend);
	cdev_del(&info->client_cdev);
err_reg_client:
	device_destroy(hw3d_class, DEV_CLIENT);
err_dev_client:
	cdev_del(&info->master_cdev);
err_reg_master:
	device_destroy(hw3d_class, DEV_MASTER);
err_dev_master:
	unregister_chrdev_region(info->devno, 2);
err_fail_alloc_region:
	class_unregister(hw3d_class);
err_fail_create_class:
err_remap_region:
	for (i = 0; i < HW3D_NUM_REGIONS; ++i)
		if (info->regions[i].vbase != 0)
			iounmap(info->regions[i].vbase);
	clk_put(info->imem_clk);
err_get_imem_clk:
	clk_put(info->grp_clk);
err_get_grp_clk:
	wake_lock_destroy(&info->wake_lock);
	kfree(info);
	platform_set_drvdata(pdev, NULL);
err_alloc:
	hw3d_info = NULL;
	return ret;
}

static struct platform_driver msm_hw3d_driver = {
	.probe		= hw3d_probe,
	.resume		= hw3d_resume,
	.driver		= {
		.name = "msm_hw3d",
		.owner = THIS_MODULE,
	},
};

static int __init hw3d_init(void)
{
	return platform_driver_register(&msm_hw3d_driver);
}

device_initcall(hw3d_init);
