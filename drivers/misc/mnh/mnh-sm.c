/*
 *
 * MNH State Manager HOST Driver
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include <linux/delay.h>

#include "mnh-pcie.h"
#include "hw-mnh-regs.h"
#include "mnh-sm.h"

#define MAX_STR_COPY 32
#define SUCCESS 0
#define DEVICE_NAME "mnh_sm"
#define CLASS_NAME "mnh_sm"

#define SGL_SIZE 64

#define INIT_DONE 0x1

/* Firmware download address */
#define HW_MNH_SBL_DOWNLOAD		0x40000000
#define HW_MNH_SBL_DOWNLOAD_EXE		0x00101000
#define HW_MNH_UBOOT_DOWNLOAD		0x40020000
#define HW_MNH_KERNEL_DOWNLOAD		0x40080000
#define HW_MNH_DT_DOWNLOAD		0x40880000
#define HW_MNH_RAMDISK_DOWNLOAD		0x40890000
#define IMG_DOWNLOAD_MAX_SIZE		(2000 * 1024)
#define FIP_IMG_SBL_SIZE_OFFSET		0x28
#define FIP_IMG_SBL_ADDR_OFFSET		0x20
#define FIP_IMG_UBOOT_SIZE_OFFSET	0x50
#define FIP_IMG_UBOOT_ADDR_OFFSET	0x48

/* SM GPIO definitions */
#define MNH_POWER_ON_GPIO 393
#define MNH_PCIE_CHAN_0 0

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct mnh_sm_device {
	enum fw_image_state	image_loaded;
};

static struct cdev *mcdev;
static dev_t dev_num;
static struct class *mclass;
static struct device *mdevice;
static struct mnh_sm_device *mnh_sm_dev;

static ssize_t mnh_sm_poweron_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
    ssize_t strlen = 0;
    int gpio_value = 0;
    printk("Entering mnh_sm_poweron_show...\n");
    gpio_request(MNH_POWER_ON_GPIO, "MNH PWRON");
    gpio_value = gpio_export(MNH_POWER_ON_GPIO,true);
    gpio_direction_output(MNH_POWER_ON_GPIO, 1);
    return strlen;
}

static ssize_t mnh_sm_poweron_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
    printk("Entering mnh_sm_poweron_store...\n");
    gpio_set_value(MNH_POWER_ON_GPIO,1);
    return -EINVAL;
}

static DEVICE_ATTR(poweron, S_IWUSR | S_IRUGO,
		mnh_sm_poweron_show, mnh_sm_poweron_store);

static ssize_t mnh_sm_poweroff_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
    ssize_t strlen = 0;
    int gpio_value = 0;

    printk("Entering mnh_sm_poweroff_show...\n");
    gpio_request(MNH_POWER_ON_GPIO, "MNH PWRON");
    gpio_value = gpio_export(MNH_POWER_ON_GPIO,true);
    gpio_direction_output(MNH_POWER_ON_GPIO, 1);
    printk("MNH_POWER_ON_GPIO = %d", gpio_value);

    return strlen;
}

static ssize_t mnh_sm_poweroff_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
    printk("Entering mnh_sm_poweroff_store...\n");

    gpio_set_value(MNH_POWER_ON_GPIO,0);
    return -EINVAL;
}

static DEVICE_ATTR(poweroff, S_IWUSR | S_IRUGO,
		mnh_sm_poweroff_show, mnh_sm_poweroff_store);

static int dma_callback(uint8_t chan, enum mnh_dma_chan_dir_t dir,
		enum mnh_dma_trans_status_t status)
{
	printk("DMA_CALLBACK: ch:%d, dir:%s, status:%s\n",
		chan, (dir == DMA_AP2EP)?"READ(AP2EP)":"WRITE(EP2AP)",
		(status == DMA_DONE)?"DONE":"ABORT");


	if ( chan == MNH_PCIE_CHAN_0 && dir == DMA_AP2EP )
	{
		if (mnh_sm_dev->image_loaded == FW_IMAGE_DOWNLOADING) {
			if (status == DMA_DONE)
				mnh_sm_dev->image_loaded = FW_IMAGE_DOWNLOAD_SUCCESS;
			else if (status == DMA_ABORT)
				mnh_sm_dev->image_loaded = FW_IMAGE_DOWNLOAD_FAIL;
		}
	}
	else
	{
		printk("DMA_CALLBACK: incorrect channel and direction");
		return -EINVAL;
	}

	return 0;
}

static int mnh_firmware_waitdownloaded(void)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(5000);

	do {
		if (mnh_sm_dev->image_loaded == FW_IMAGE_DOWNLOAD_SUCCESS) {
			printk("Firmware loaded!\n");
			return 0;
		} else if (mnh_sm_dev->image_loaded == FW_IMAGE_DOWNLOAD_FAIL) {
			printk("Firmware load fail!\n");
			return -EIO;
		}
		msleep(20);
	} while (time_before(jiffies, timeout));

	printk("Fail to Download Firmware, timeout!!.\n");
	return -EIO;
}

int mnh_transfer_firmware(size_t fw_size, const uint8_t *fw_data,
	uint64_t dst_addr)
{
	uint32_t *buf = NULL;
	struct mnh_dma_element_t dma_blk;
	int err = -EINVAL;
	size_t sent = 0, size, remaining;

	remaining = fw_size;

	while (remaining > 0) {
		size = MIN(remaining, IMG_DOWNLOAD_MAX_SIZE);

		buf = (uint32_t *)kmalloc(size, GFP_KERNEL);
		if (!buf) {
			printk("Fail to alloc!\n");
			return -EIO;
		}
		memcpy(buf, fw_data + sent, size);

		dma_blk.dst_addr = dst_addr + sent;
		dma_blk.len = size;
		dma_blk.src_addr = virt_to_phys(buf);

		printk("FW download - AP(:0x%llx) to EP(:0x%llx), size(%d)\n",
			dma_blk.src_addr, dma_blk.dst_addr, dma_blk.len);

		mnh_sm_dev->image_loaded = FW_IMAGE_DOWNLOADING;
		mnh_dma_sblk_start(MNH_PCIE_CHAN_0, DMA_AP2EP, &dma_blk);

		sent += size;
		remaining -= size;
		printk("Sent:%zd, Remaining:%zd\n", sent, remaining);

		err = mnh_firmware_waitdownloaded();
		kfree(buf);

		if (err)
			break;
	}

	return err;

}

int mnh_download_firmware(void)
{
	const struct firmware *dt_img, *kernel_img, *ram_img;
	const struct firmware *fip_img;
	int err;
	uint32_t size, addr;

	mnh_sm_dev->image_loaded = FW_IMAGE_NONE;

	/* Register DMA callback */
	mnh_reg_irq_callback(0, 0, dma_callback);

	err = request_firmware(&fip_img, "mnh/fip.bin", mdevice);
	if (err) {
		printk("request fip_image failed - %d\n", err);
		return -EIO;
	}

	err = request_firmware(&kernel_img, "mnh/Image", mdevice);
	if (err) {
		printk("request kernel failed - %d\n", err);
		goto free_uboot;
	}

	err = request_firmware(&dt_img, "mnh/mnh.dtb", mdevice);
	if (err) {
		printk("request kernel failed - %d\n", err);
		goto free_kernel;
	}

	err = request_firmware(&ram_img, "mnh/ramdisk.img", mdevice);
	if (err) {
		printk("request kernel failed - %d\n", err);
		goto free_dt;
	}

	/* DMA transfer for SBL */
	memcpy(&size, (uint8_t *)(fip_img->data + FIP_IMG_SBL_SIZE_OFFSET), 4);
	memcpy(&addr, (uint8_t *)(fip_img->data + FIP_IMG_SBL_ADDR_OFFSET), 4);
	printk("sbl size :0x%x", size);
	printk("sbl data addr :0x%x", addr);

	printk("DOWNLOADING SBL...size:0x%x\n", size);
	if (mnh_transfer_firmware(size, fip_img->data + addr,
			HW_MNH_SBL_DOWNLOAD))
		goto fail_downloading;
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_4, 4,
		HW_MNH_SBL_DOWNLOAD);
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_5, 4,
		HW_MNH_SBL_DOWNLOAD_EXE);
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_6, 4,
		size);

	/* DMA transfer for UBOOT */
	memcpy(&size,
		(uint8_t *)(fip_img->data + FIP_IMG_UBOOT_SIZE_OFFSET), 4);
	memcpy(&addr,
		(uint8_t *)(fip_img->data + FIP_IMG_UBOOT_ADDR_OFFSET), 4);
	printk("uboot size :0x%x", size);
	printk("uboot data addr:0x%x", addr);

	/* DMA transfer for UBOOT */
	printk("DOWNLOADING UBOOT...size:0x%x\n", size);
	if (mnh_transfer_firmware(size, fip_img->data + addr,
			HW_MNH_UBOOT_DOWNLOAD))
		goto fail_downloading;
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_7, 4,
		HW_MNH_UBOOT_DOWNLOAD);

	/* DMA transfer for device tree */
	printk("DOWNLOADING DT...size:%zd\n", dt_img->size);
	if (mnh_transfer_firmware(dt_img->size, dt_img->data,
			HW_MNH_DT_DOWNLOAD))
		goto fail_downloading;

	/* DMA transfer for ramdisk */
	printk("DOWNLOADING RAMDISK...size:%zd\n", ram_img->size);
	if (mnh_transfer_firmware(ram_img->size, ram_img->data,
			HW_MNH_RAMDISK_DOWNLOAD))
		goto fail_downloading;

	/* DMA transfer for Kernel image */
	printk("DOWNLOADING KERNEL...size:%zd\n", kernel_img->size);
	if (mnh_transfer_firmware(kernel_img->size, kernel_img->data,
			HW_MNH_KERNEL_DOWNLOAD))
		goto fail_downloading;

	release_firmware(fip_img);
	release_firmware(kernel_img);
	release_firmware(dt_img);
	release_firmware(ram_img);

	/* Unregister DMA callback */
        mnh_reg_irq_callback(NULL, NULL, NULL);
	return 0;

fail_downloading:
	printk("FW downloading fails\n");
	mnh_sm_dev->image_loaded = FW_IMAGE_DOWNLOAD_FAIL;
	release_firmware(ram_img);
free_dt:
	release_firmware(dt_img);
free_kernel:
	release_firmware(kernel_img);
free_uboot:
	release_firmware(fip_img);

	/* Unregister DMA callback */
        mnh_reg_irq_callback(NULL, NULL, NULL);
	return -EIO;
}

static ssize_t mnh_sm_download_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
    ssize_t strlen = 0;

    printk("MNH PM mnh_pm_download_show...\n");

    mnh_sm_download();

    return strlen;
}

static ssize_t mnh_sm_download_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
    return -EINVAL;
}

static DEVICE_ATTR(download, S_IWUSR | S_IRUGO,
		mnh_sm_download_show, mnh_sm_download_store);


static ssize_t mnh_sm_suspend_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
    ssize_t strlen = 0;
    return strlen;
}

static ssize_t mnh_sm_suspend_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
    return -EINVAL;
}

static DEVICE_ATTR(suspend, S_IWUSR | S_IRUGO,
		mnh_sm_suspend_show, mnh_sm_suspend_store);

static ssize_t mnh_sm_resume_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
    ssize_t strlen = 0;
    return strlen;
}

static ssize_t mnh_sm_resume_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
    return -EINVAL;
}

static DEVICE_ATTR(resume, S_IRUGO | S_IWUSR | S_IWGRP,
		mnh_sm_resume_show, mnh_sm_resume_store);

static ssize_t mnh_sm_reset_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
    ssize_t strlen = 0;
    return strlen;
}

static ssize_t mnh_sm_reset_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
    return -EINVAL;
}

static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR | S_IWGRP,
		mnh_sm_reset_show, mnh_sm_reset_store);


static struct attribute *mnh_sm_dev_attributes[] = {
	&dev_attr_poweron.attr,
	&dev_attr_poweroff.attr,
	&dev_attr_download.attr,
	&dev_attr_suspend.attr,
	&dev_attr_resume.attr,
	&dev_attr_reset.attr,
	NULL
};

static struct attribute_group mnh_sm_group = {
	.name = "mnh_sm",
	.attrs = mnh_sm_dev_attributes
};

/*******************************************************************************
 *
 *	APIs
 *
 ******************************************************************************/


/**
 * API to initialize Power and clocks to MNH, MIPI, DDR, DDR training,
 * and PCIE.
 * @param[in] Structure argument to configure each boot component.
 *            This structure will be populated within the kernel module.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_poweron( struct mnh_sm_configuration* mnh_sm_boot_args )
{
    return 0;
}
EXPORT_SYMBOL(mnh_sm_poweron);

/**
 * API to obtain the state of monette hill.
 * @return the power states of mnh(ex: On, Off, Active, Suspend, Bypass).
 * 	MNH_HW_INIT - MNH is on, Kernel not executing, and before FW download.
 * 	MNH_HW_OFF - MNH is powered off
 * 	MNH_HW_ACTIVE: MNH is on and flashed. Kernel is running.
 * 	MNH_HW_SUSPEND_SELF_REFRESH: DDR is self refreshing.
 *                                   All other components are off.
 * 	MNH_HW_SUSPEND_HIBERNATE: Hibernation image stored in AP RAM
 *                           	  over PCIe outbound and MNH is powered down.
 */
int mnh_sm_get_state(void)
{
    return 0;
}
EXPORT_SYMBOL(mnh_sm_get_state);

/**
 * API to power monette hill.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_poweroff(void)
{
    return 0;
}
EXPORT_SYMBOL(mnh_sm_poweroff);

/**
 * API to download the binary images(SBL, UBoot, Kernel, Ramdisk) for mnh.
 * The location of the binaries will be located in the AP file system.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_download(void)
{
    uint32_t  magic;

    if (mnh_download_firmware() == 0) {
        /* Magic number setting to notify MNH that PCIE initialization
	is done on Host side */
	if (mnh_config_read(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET +
	                    HW_MNH_PCIE_GP_0,
			    sizeof(uint32_t), &magic) == SUCCESS && magic == 0)
        {
	    mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET +
	                     HW_MNH_PCIE_GP_0,
	                     sizeof(uint32_t), INIT_DONE);
	} else {
	    printk("Read GP0 register fail or GP0 is not 0:%d",
	            magic);
	}
    }
    return 0;
}

EXPORT_SYMBOL(mnh_sm_download);

/**
 * API to put MNH in suspend state.  In suspend mode the DDR will be isolated
 * and put in self refresh while the CPU is powered down.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_suspend(void)
{
    return 0;
}
EXPORT_SYMBOL(mnh_sm_suspend);

/**
 * API to put MNH into active state.
 * The resume call flow should be similar to normal bootflow except for DDR
 * initializations. Since the binaries are already saved on the DDR while MNH
 * is in suspend, ESM will not need to download the binaries again during
 * resume.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_resume(void)
{
    return 0;
}
EXPORT_SYMBOL(mnh_sm_resume);

static int __exit mnh_sm_exit(void)
{
	pr_debug("MNHPM Un-initializing\n");
	sysfs_remove_group(kernel_kobj, &mnh_sm_group);
	cdev_del(mcdev);
	unregister_chrdev_region(dev_num, 1);
	device_destroy(mclass, MKDEV(MAJOR(dev_num), 0));
	class_destroy(mclass);
        gpio_free(MNH_POWER_ON_GPIO);

	return 0;
}

static int __init mnh_sm_init(void)
{
	int error = 0;

	pr_debug("MNH SM initializing...\n");

	mnh_sm_dev = kzalloc(sizeof(struct mnh_sm_device), GFP_KERNEL);

	if (mnh_sm_dev == NULL) {
		printk("Fail to alloc mnh_sm_dev");
		return error;
	}

	error = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
	if (error < 0) {
		return error;
	}

	mcdev = cdev_alloc();
	mcdev->owner = THIS_MODULE;

	error = cdev_add(mcdev, dev_num, 1);
	if (error) {
		unregister_chrdev_region(dev_num, 1);
		return error;
	}

	mclass = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(mclass)) {
		cdev_del(mcdev);
		unregister_chrdev_region(dev_num, 1);
		error = PTR_ERR(mclass);
		return error;
	}
	mdevice = device_create(mclass, NULL, MKDEV(MAJOR(dev_num), 0),
			NULL, DEVICE_NAME);
	if (IS_ERR(mdevice)) {
		cdev_del(mcdev);
		unregister_chrdev_region(dev_num, 1);
		class_destroy(mclass);
		error = PTR_ERR(mdevice);
		return error;
	}

	pr_debug("char driver %s added", DEVICE_NAME);

	error = sysfs_create_group(kernel_kobj, &mnh_sm_group);
	if (error)
		pr_debug("failed to create /sys/kernel/mnh_sm\n");


	printk("MNH SM initialized successfully\n");

	return error;

}


module_init(mnh_sm_init);
module_exit(mnh_sm_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("MNH State Manager HOST DRIVER");
MODULE_LICENSE("GPL v2");
