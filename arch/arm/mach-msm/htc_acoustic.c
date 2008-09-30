/* arch/arm/mach-msm/htc_acoustic.c
 *
 * Copyright (C) 2007-2008 HTC Corporation
 * Author: Laurence Chen <Laurence_Chen@htc.com>
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

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>

#define HTC_ACOUSTIC_TABLE_BASE_PHY_ADDR_START	(0x01FE0000)
#define HTC_ACOUSTIC_TABLE_SIZE			(0x10000)

#define ACOUSTICE(x...) printk(KERN_ERR "[ACOUSTIC] " x)

static int acoustic_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long pgoff;
	size_t size = vma->vm_end - vma->vm_start;
	
	if (vma->vm_pgoff != 0)
		return -EINVAL;
	
	if (size <= HTC_ACOUSTIC_TABLE_SIZE)
		pgoff = HTC_ACOUSTIC_TABLE_BASE_PHY_ADDR_START >> PAGE_SHIFT;
	else
		return -EINVAL;

	vma->vm_flags |= VM_IO | VM_RESERVED;

	if (io_remap_pfn_range(vma, vma->vm_start, pgoff,
			      size, vma->vm_page_prot))
		return -EAGAIN;
	
	return 0;
}

static int acoustic_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int acoustic_release(struct inode *inode, struct file *file)
{
	return 0;
}
 
static struct file_operations acoustic_fops = {
	.owner = THIS_MODULE,
	.open = acoustic_open,
	.release = acoustic_release,
	.mmap = acoustic_mmap,
};

static struct miscdevice acoustic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-acoustic",
	.fops = &acoustic_fops,
};

static int __init acoustic_init(void)
{
	int ret;

	ret = misc_register(&acoustic_misc);
	if (ret < 0) {
		ACOUSTICE("failed to register misc device!\n");
		return ret;
	}
	
	return 0;
}

static void __exit acoustic_exit(void)
{
	int ret;

	ret = misc_deregister(&acoustic_misc);
	if (ret < 0)
		ACOUSTICE("failed to unregister misc device!\n");
}

module_init(acoustic_init);
module_exit(acoustic_exit);
 
MODULE_AUTHOR("Laurence Chen <Laurence_Chen@htc.com>");
MODULE_DESCRIPTION("HTC acoustic driver");
MODULE_LICENSE("GPL");
