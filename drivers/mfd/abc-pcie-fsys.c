/*
 * Airbrush PCIe FSYS  driver
 *
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com
 *
 * Author: Sayanta Pattanayak <sayanta.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <linux/mfd/abc-pcie.h>

#include "abc-pcie-private.h"

struct abc_pcie_fsys {
	struct class *dev_class;
	unsigned int dev_major;
	struct cdev c_dev;
	struct device *dev;
	dev_t devt;
};

struct abc_pcie_fsys abc_fsys;

#define BUF_SIZE	4096

static int dma_callback(uint8_t chan, enum dma_data_direction dir,
		enum abc_dma_trans_status status)
{
	pr_info(" In Dma_Callback for chan=%d, dir=%d, status=%d\n",
			chan, dir, status);
	return 0;
}

static int wdt_callback(uint32_t irq)
{
	pr_info(" In Wdt_Callback for irq %d\n", irq);
	return 0;
}

static int abc_fsys_open(struct inode *ip, struct file *fp)
{
	struct abc_device *abc_dev;

	abc_dev = dev_get_drvdata(abc_fsys.dev);
	fp->private_data = abc_dev;

	return 0;
}

static long abc_fsys_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	struct abc_device *abc_dev = fp->private_data;
	struct config_write cw;
	struct config_read  cr;
	struct outb_region  or;
	struct inb_region   ir;
	struct abc_dma_desc dr;
	struct abc_dma_desc dw;
	struct dma_element_t de;
	u32	data;
	int ret = 0;

	switch (cmd) {
	case ABC_PCIE_CONFIG_READ:
		if (copy_from_user(&cr, (void __user *) arg, sizeof(cr))) {
			pr_err("Error in CR Copying\n");
			return -1;
		}
		ret = pcie_config_read(cr.offset, cr.len, &data);
		if (copy_to_user((void __user *)cr.data, &data, sizeof(data))) {
			pr_err("Error copying into CR data\n");
			return -EFAULT;
		}
		break;
	case ABC_PCIE_CONFIG_WRITE:
		if (copy_from_user(&cw, (void __user *) arg, sizeof(cw))) {
			pr_err("Error in CW Copying\n");
			return -1;
		}
		ret = pcie_config_write(cw.offset, cw.len, cw.data);
		break;
	case ABC_PCIE_SET_IB_IATU:
		if (copy_from_user(&ir, (void __user *) arg, sizeof(ir))) {
			pr_err("Error in IB Copying\n");
			return -1;
		}
		ret = set_inbound_iatu(ir);
		break;
	case ABC_PCIE_SET_OB_IATU:
		if (copy_from_user(&or, (void __user *) arg, sizeof(or))) {
			pr_err("Error in OB Copying\n");
			return -1;
		}
		ret = set_outbound_iatu(or);
		break;
	case ABC_PCIE_ALLOC_BUF:
		if (abc_dev->wr_buf || abc_dev->rd_buf)
			return ret;
		abc_dev->wr_buf = abc_alloc_coherent(arg,
						     &abc_dev->wr_buf_addr);
		abc_dev->rd_buf = abc_dev->wr_buf + BUF_SIZE;
		abc_dev->rd_buf_addr = abc_dev->wr_buf_addr + BUF_SIZE;
		break;
	case ABC_PCIE_SET_RD_DMA:
		if (copy_from_user(&dr, (void __user *) arg, sizeof(dr))) {
			pr_err("Error in DR Copying\n");
			return -1;
		}
		de.src_addr = abc_dev->rd_buf_addr;
		de.src_u_addr = 0x0;
		de.dst_addr = dr.buf_addr;
		de.dst_u_addr = 0x0;
		de.len      = dr.len;
		de.chan	    = dr.chan;
		ret = dma_sblk_start(de.chan, DMA_TO_DEVICE, &de);
		break;
	case ABC_PCIE_SET_WR_DMA:
		if (copy_from_user(&dw, (void __user *) arg, sizeof(dw))) {
			pr_err("Error in DR Copying\n");
			return -1;
		}
		de.src_addr = dw.buf_addr;
		de.src_u_addr = 0x0;
		de.dst_addr = abc_dev->wr_buf_addr;
		de.dst_u_addr = 0x0;
		de.len      = dw.len;
		de.chan	    = dw.chan;
		ret = dma_sblk_start(de.chan, DMA_FROM_DEVICE, &de);
		break;
	}

	return ret;

}

static int abc_mmap(struct file *fp, struct vm_area_struct *area)
{
	struct abc_device *abc_dev = fp->private_data;
	int ret;

	ret = dma_mmap_coherent(abc_dev->dev, area,
			abc_dev->wr_buf, abc_dev->wr_buf_addr,
			area->vm_end - area->vm_start);
	return ret;
}

static int abc_fsys_close(struct inode *ip, struct file *fp)
{
	//todo..need to do closing acitivity.
	return 0;
}

static const struct file_operations abc_fsys_fops = {
	.owner		= THIS_MODULE,
	.open		= abc_fsys_open,
	.release	= abc_fsys_close,
	.unlocked_ioctl = abc_fsys_ioctl,
	.mmap           = abc_mmap,
};

static int abc_pcie_fsys_drv_probe(struct platform_device *pdev)
{
	struct abc_device *abc_dev = pdev->dev.platform_data;
	int ret;
	u32 dma_chan;

#if 0
	/*Keeping below code as alternate way of getting MEM resource
	 * and performing ioremap on the same for direct readl()/writel()
	 * on respective module address.
	 */

	struct resource *res;

	/* MEM resource can be obtained here and IOREMAP can be perfomed
	 * on the same.In that case pci_ioremap shouldn't be performed in
	 * abc-pcie function driver.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOMEM;

	if (!request_mem_region(res->start, resource_size(res),
				"abc-pcie-fsys"))
		return -EBUSY;

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		release_mem_region(res->start, resource_size(res));
		return -ENOMEM;
	}

	abc_dev->fsys_config = base;
	abc_dev->pcie_config = base + (1024 * 1024);
#endif

	abc_fsys.dev_class = class_create(THIS_MODULE, "abc-pcie-fsys");
	if (IS_ERR(abc_fsys.dev_class))
		ret = PTR_ERR(abc_fsys.dev_class);

	ret = alloc_chrdev_region(&abc_fsys.devt, 0, MAX_MINOR_COUNT,
				  DRV_NAME_ABC_PCIE_BLK_FSYS);
	if (ret)
		dev_err(&pdev->dev, "Could not allocate chrdev region\n");

	abc_fsys.dev_major = MAJOR(abc_fsys.devt);

	cdev_init(&abc_fsys.c_dev, &abc_fsys_fops);
	abc_fsys.c_dev.owner = THIS_MODULE;

	ret = cdev_add(&abc_fsys.c_dev,
		       MKDEV(abc_fsys.dev_major, FSYS_MINOR_NUMBER),
		       MAX_MINOR_COUNT);
	if (ret)
		dev_err(&pdev->dev, "Could not add cdev\n");

	abc_fsys.dev = device_create(abc_fsys.dev_class, &pdev->dev,
			MKDEV(abc_fsys.dev_major, FSYS_MINOR_NUMBER),
				     abc_dev, "abcpciefsys");
		if (IS_ERR(abc_fsys.dev))
			dev_err(&pdev->dev, "Could not create files\n");


	/* DMA Callback registration */
	for (dma_chan = 0; dma_chan < 8; dma_chan++)
		abc_reg_dma_irq_callback(&dma_callback, dma_chan);

	/* Register callback with callback function & MSI_IRQ Number */
	abc_reg_irq_callback(&wdt_callback, ABC_MSI_9_WDT0);

	return 0;
}

static int abc_pcie_fsys_drv_remove(struct platform_device *pdev)
{
	device_destroy(abc_fsys.dev_class,
			MKDEV(abc_fsys.dev_major, FSYS_MINOR_NUMBER));
	cdev_del(&abc_fsys.c_dev);
	unregister_chrdev_region(abc_fsys.devt, MAX_MINOR_COUNT);
	class_destroy(abc_fsys.dev_class);
	return 0;
}

static const struct platform_device_id abc_pcie_fsys_ids[] = {
	{
		.name   = DRV_NAME_ABC_PCIE_BLK_FSYS,
	}, {
		/* todo */
	}
};
MODULE_DEVICE_TABLE(platform, abc_pcie_fsys_ids);

static struct platform_driver abc_pcie_fsys_driver = {
	.probe          = abc_pcie_fsys_drv_probe,
	.remove         = abc_pcie_fsys_drv_remove,
	.id_table       = abc_pcie_fsys_ids,
	.driver         = {
		.name   = DRV_NAME_ABC_PCIE_BLK_FSYS,
	},
};
module_platform_driver(abc_pcie_fsys_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sayanta Pattanayak <sayanta.p@samsung.com>");
MODULE_DESCRIPTION("Airbrush FSYS Driver");
