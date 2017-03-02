/* Copyright (c) 2017, HTC Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/uio_driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <linux/cma.h>
#include <soc/qcom/secure_buffer.h>
#include <linux/slab.h>
#include <linux/remote_spinlock.h>
#include <soc/qcom/smem.h>
#include <soc/qcom/ramdump.h>
#include <soc/qcom/subsystem_notif.h>
#include "sharedmem_qmi.h"

#define CLIENT_ID_PROP "qcom,client-id"

#define MPSS_RMTS_CLIENT_ID 1

#define DRIVER_NAME "rtel" /* runtime_embedded_log */

struct restart_notifier_block {
	unsigned processor;
	char *name;
	struct notifier_block nb;
};

static struct ramdump_segment *rtel_ramdump_segments;
static void *rtel_ramdump_dev;

static void *uio_vaddr;
static struct device *uio_dev;

static int setup_shared_ram_perms(u32 client_id, phys_addr_t addr, u32 size);
static int clear_shared_ram_perms(u32 client_id, phys_addr_t addr, u32 size);

static int restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data)
{
	if (code == SUBSYS_RAMDUMP_NOTIFICATION) {
		struct restart_notifier_block *notifier;

		notifier = container_of(this,
					struct restart_notifier_block, nb);
		pr_info("[rtel] %s: ssrestart for processor %d ('%s')\n",
				__func__, notifier->processor,
				notifier->name);

		if (rtel_ramdump_dev && rtel_ramdump_segments) {
			int ret;

			pr_info("[rtel] %s: saving runtime embedded log ramdump.\n",
					__func__);
			ret = do_ramdump(rtel_ramdump_dev,
					rtel_ramdump_segments, 1);
			if (ret < 0)
				pr_err("[rtel]%s: unable to dump runtime embedded log %d\n",
						__func__, ret);
		}
	}

	return NOTIFY_DONE;
}

static struct restart_notifier_block restart_notifiers[] = {
	{SMEM_MODEM, "modem", .nb.notifier_call = restart_notifier_cb},
};

static ssize_t rtel_show(struct device *d,
			struct device_attribute *attr,
			char *buf)
{
	struct uio_info *info = dev_get_drvdata(d);

	return snprintf(buf, PAGE_SIZE, "size:0x%x paddr:0x%x vaddr:0x%p\n",
					(unsigned int)info->mem[0].size,
					(unsigned int)info->mem[0].addr,
					uio_vaddr);
}

static ssize_t rtel_store(struct device *d,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int ret = 0;
	u64 rmtfs_addr = 0, rtel_addr = 0;
	u32 rmtfs_size = 0, rtel_size = 0;
	struct uio_info *info = dev_get_drvdata(d);
	void *handle;
	struct restart_notifier_block *nb;

	pr_info("[rtel] Entering rtel store.\n");

	if (!info) {
		pr_err("[rtel] Can't get correct uio info.\n");
		return count;
	}

	get_uio_addr_size_by_name("rmtfs", &rmtfs_addr, &rmtfs_size);
	rtel_addr = info->mem[0].addr;
	rtel_size = info->mem[0].size;

	if (!rmtfs_addr || !rmtfs_size || !rtel_addr || !rtel_size) {
		pr_err("[rtel] Can't get rmtfs/rtel address or size data.\n");
		return count;
	}

	/* Debug purpose, input userdebug/release to switch mem protect area */
	if ((buf[0] == '1' && !uio_vaddr) || !strncmp(buf, "enable", 6)) {
		uio_vaddr = dma_alloc_writecombine(uio_dev, rtel_size,
						&rtel_addr, GFP_KERNEL);

		if (uio_vaddr == NULL) {
			pr_err("[rtel] Shared memalloc fail, client=%s, size=%x\n",
				info->name, rtel_size);
			return count;
		}

		clear_shared_ram_perms(MPSS_RMTS_CLIENT_ID, rmtfs_addr,
					rmtfs_size);
		ret = setup_shared_ram_perms(MPSS_RMTS_CLIENT_ID, rtel_addr,
					rtel_size);

		if (ret)
			pr_err("[rtel] %s setup_shared_ram_perms fail.\n",
				info->name);

		if (rtel_ramdump_dev)
			return count;

		rtel_ramdump_segments = kzalloc(sizeof(struct ramdump_segment),
						GFP_KERNEL);
		if (IS_ERR_OR_NULL(rtel_ramdump_segments)) {
			pr_err("[rtel] %s:rtel_ramdump_segments alloc fail\n",
					__func__);
			rtel_ramdump_segments = NULL;
		} else {
			rtel_ramdump_segments->address = rtel_addr +
							rmtfs_size;
			rtel_ramdump_segments->size = rtel_size - rmtfs_size;
			rtel_ramdump_segments->v_address = uio_vaddr +
							rmtfs_size;

			pr_info("[rtel] rtel addr= 0x%lx, size= 0x%lx, v_addr= 0x%p\n",
					rtel_ramdump_segments->address,
					rtel_ramdump_segments->size,
					rtel_ramdump_segments->v_address);
		}

		rtel_ramdump_dev = create_ramdump_device("rtel", NULL);
		if (IS_ERR_OR_NULL(rtel_ramdump_dev)) {
			pr_err("[rtel] %s: Unable to create rtel ramdump device.\n",
					__func__);
			rtel_ramdump_dev = NULL;
		}

		if (rtel_ramdump_segments && rtel_ramdump_dev) {
			nb = &restart_notifiers[0];
			handle = subsys_notif_register_notifier(nb->name,
				    &nb->nb);
			if (IS_ERR_OR_NULL(handle)) {
				pr_err("[rtel] %s: Unable to register subsys notify.\n",
					__func__);
			} else
				pr_info("[rtel] registering notif for '%s', handle=0x%p\n",
					nb->name, handle);
		}
	} else if (uio_vaddr && !strncmp(buf, "disable", 7)) {
		clear_shared_ram_perms(MPSS_RMTS_CLIENT_ID, rtel_addr,
		    rtel_size);
		ret = setup_shared_ram_perms(MPSS_RMTS_CLIENT_ID, rmtfs_addr,
			rmtfs_size);

		if (ret)
			pr_err("[rtel] %s setup_shared_ram_perms fail!!\n",
					info->name);

		dma_free_writecombine(uio_dev, rtel_size, uio_vaddr, rtel_addr);
		uio_vaddr = NULL;
	} else {
		pr_err("[rtel] No action, input buf %s", buf);
		return count;
	}

	return count;
}

static DEVICE_ATTR(rtel, 0664, rtel_show, rtel_store);
static struct attribute *rtel_attributes[] = {
	&dev_attr_rtel.attr,
	NULL
};

static const struct attribute_group rtel_group = {
	.name  = "rtel",
	.attrs = rtel_attributes,
};


static int uio_get_mem_index(struct uio_info *info, struct vm_area_struct *vma)
{
	if (vma->vm_pgoff >= MAX_UIO_MAPS)
		return -EINVAL;

	if (info->mem[vma->vm_pgoff].size == 0)
		return -EINVAL;

	return (int)vma->vm_pgoff;
}

static int sharedmem_mmap(struct uio_info *info, struct vm_area_struct *vma)
{
	int result;
	struct uio_mem *mem;
	int mem_index = uio_get_mem_index(info, vma);

	if (mem_index < 0) {
		pr_err("[rtel] mem_index is invalid errno %d\n", mem_index);
		return mem_index;
	}

	mem = info->mem + mem_index;

	if (vma->vm_end - vma->vm_start > mem->size) {
		pr_err("vm_end[%lu] - vm_start[%lu] [%lu] > mem->size[%lu]\n",
			(unsigned long) vma->vm_end,
			(unsigned long) vma->vm_start,
			(unsigned long) (vma->vm_end - vma->vm_start),
			(unsigned long) mem->size);
		return -EINVAL;
	}
	pr_debug("[rtel] Attempting to setup mmap.\n");

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	result = remap_pfn_range(vma, vma->vm_start,
				mem->addr >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot);
	if (result != 0)
		pr_err("[rtel] mmap Failed with errno %d\n", result);
	else
		pr_debug("[rtel] mmap success\n");

	return result;
}

/* Setup the shared ram permissions.
 * This function currently supports the mpss client only.
 */
static int setup_shared_ram_perms(u32 client_id, phys_addr_t addr, u32 size)
{
	int ret;
	u32 source_vmlist[1] = {VMID_HLOS};
	int dest_vmids[2] = {VMID_HLOS, VMID_MSS_MSA};
	int dest_perms[2] = {PERM_READ|PERM_WRITE, PERM_READ|PERM_WRITE};

	if (client_id != MPSS_RMTS_CLIENT_ID)
		return -EINVAL;

	ret = hyp_assign_phys(addr, size, source_vmlist, 1, dest_vmids,
				dest_perms, 2);
	if (ret)
		pr_err("hyp_assign_phys failed addr=0x%pa size=%x err=%d\n",
			&addr, size, ret);
	return ret;
}

static int clear_shared_ram_perms(u32 client_id, phys_addr_t addr, u32 size)
{
	int ret;
	u32 source_vmlist[2] = {VMID_HLOS, VMID_MSS_MSA};
	int dest_vmids[1] = {VMID_HLOS};
	int dest_perms[1] = {PERM_READ|PERM_WRITE};

	if (client_id != MPSS_RMTS_CLIENT_ID)
		return -EINVAL;

	ret = hyp_assign_phys(addr, size, source_vmlist, 2, dest_vmids,
				dest_perms, 1);
	if (ret)
		pr_err("hyp_assign_phys failed addr=0x%pa size=%x err=%d\n",
			&addr, size, ret);
	return ret;
}

static int rtel_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct uio_info *info = NULL;
	struct resource *clnt_res = NULL;
	u32 client_id = ((u32)~0U);
	u32 shared_mem_size = 0;
	phys_addr_t shared_mem_pyhsical = 0;
	bool is_addr_dynamic = false;
	struct sharemem_qmi_entry qmi_entry;

	/* Get the addresses from platform-data */
	if (!pdev->dev.of_node) {
		pr_err("[rtel] Node not found\n");
		ret = -ENODEV;
		goto out;
	}
	clnt_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!clnt_res) {
		pr_err("[rtel] Resource not found\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(pdev->dev.of_node, CLIENT_ID_PROP,
				   &client_id);
	if (ret) {
		client_id = ((u32)~0U);
		pr_warn("[rtel] qcom,client-id property not found\n");
	}

	info = devm_kzalloc(&pdev->dev, sizeof(struct uio_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	shared_mem_size = resource_size(clnt_res);
	shared_mem_pyhsical = clnt_res->start;

	if (shared_mem_size == 0) {
		pr_err("[rtel] Shared memory size is zero\n");
		return -EINVAL;
	}

	/* Setup device */
	info->mmap = sharedmem_mmap; /* Custom mmap function. */
	info->name = clnt_res->name;
	info->version = "1.0";
	info->mem[0].addr = shared_mem_pyhsical;
	info->mem[0].size = shared_mem_size;
	info->mem[0].memtype = UIO_MEM_PHYS;
	uio_dev = &pdev->dev;
	uio_vaddr = NULL;

	ret = uio_register_device(&pdev->dev, info);
	if (ret) {
		pr_err("[rtel] uio register failed ret=%d\n", ret);
		goto out;
	}

	dev_set_drvdata(&pdev->dev, info);

	ret = sysfs_create_group(&pdev->dev.kobj, &rtel_group);
	if (ret)
		pr_err("[rtel] Failed to register rtel sysfs\n");

	qmi_entry.client_id = client_id;
	qmi_entry.client_name = info->name;
	qmi_entry.address = info->mem[0].addr;
	qmi_entry.size = info->mem[0].size;
	qmi_entry.is_addr_dynamic = is_addr_dynamic;

	sharedmem_qmi_add_entry(&qmi_entry);
	pr_info("[rtel] Device created for client '%s'\n", clnt_res->name);

out:
	return ret;
}

static int rtel_remove(struct platform_device *pdev)
{
	struct uio_info *info = dev_get_drvdata(&pdev->dev);

	uio_unregister_device(info);

	return 0;
}

static const struct of_device_id rtel_of_match[] = {
	{.compatible = "htc,rtel-uio",},
	{}
};
MODULE_DEVICE_TABLE(of, rtel_of_match);

static struct platform_driver rtel_driver = {
	.probe	= rtel_probe,
	.remove	= rtel_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = rtel_of_match,
	},
};

static int __init rtel_init(void)
{
	int result;

	result = platform_driver_register(&rtel_driver);
	if (result != 0) {
		pr_err("Platform driver rtel_driver registration failed\n");
		return result;
	}
	return 0;
}

static void __exit rtel_exit(void)
{
	platform_driver_unregister(&rtel_driver);
}

module_init(rtel_init);
module_exit(rtel_exit);

MODULE_LICENSE("GPL v2");
