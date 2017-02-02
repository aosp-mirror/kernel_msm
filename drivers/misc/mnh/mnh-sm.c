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

#define DEBUG

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "mnh-pcie.h"
#include "hw-mnh-regs.h"
#include "mnh-sm-config.h"
#include "mnh-sm-config-a.h"
#include "mnh-sm.h"
#include "mnh-mipi.h"

#define MAX_STR_COPY 32
#define SUCCESS 0
#define DEVICE_NAME "mnh_sm"
#define CLASS_NAME "mnh_sm"

#define SGL_SIZE 64

#define INIT_DONE 0x1

/* Firmware download address */
/* #define HW_MNH_SBL_DOWNLOAD		0x40000000 */
#define HW_MNH_SBL_DOWNLOAD		0x00101000 /* push directly to sram */
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

/* PCIe */
#define MNH_PCIE_CHAN_0 0

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct mnh_sm_device {
	struct platform_device *pdev;
	struct device *dev;

	/* char device resources */
	struct cdev cdev;
	dev_t dev_num;

	enum fw_image_state	image_loaded;
};

static struct mnh_sm_device *mnh_sm_dev;
static hotplug_cb_t mnh_hotplug_cb;
static int mnh_state;
static int mnh_sm_uboot;

static ssize_t mnh_sm_poweron_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	ssize_t strlen = 0;

	dev_info(dev, "Entering mnh_sm_poweron_show...\n");
	mnh_sm_poweron(sm_config_1);
	return strlen;
}

static ssize_t mnh_sm_poweron_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	dev_info(dev, "Entering mnh_sm_poweron_store...\n");
	return -EINVAL;
}

static DEVICE_ATTR(poweron, S_IWUSR | S_IRUSR | S_IRGRP,
		mnh_sm_poweron_show, mnh_sm_poweron_store);

static ssize_t mnh_sm_poweroff_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	ssize_t strlen = 0;

	dev_info(dev, "Entering mnh_sm_poweroff_show...\n");
	mnh_sm_poweroff(sm_config_1);

	return strlen;
}

static ssize_t mnh_sm_poweroff_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	dev_info(dev, "Entering mnh_sm_poweroff_store...\n");

	return -EINVAL;
}

static DEVICE_ATTR(poweroff, S_IWUSR | S_IRUSR | S_IRGRP,
		mnh_sm_poweroff_show, mnh_sm_poweroff_store);



static ssize_t mnh_sm_config_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	ssize_t strlen = 0;

	dev_info(dev, "Entering mnh_sm_config_show...\n");
	mnh_sm_config(sm_config_1);
	return strlen;
}

static DEVICE_ATTR(config, S_IRUGO,
		mnh_sm_config_show, NULL);


static ssize_t mnh_sm_state_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	dev_info(dev, "Entering mnh_sm_state_show...\n");

	return scnprintf(buf, MAX_STR_COPY, "0x%x\n", mnh_state);
}

static ssize_t mnh_sm_state_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	unsigned long int val = 0;
	uint8_t *token;
	const char *delim = ";";

	dev_info(dev, "Entering mnh_sm_state_store...\n");
	token = strsep((char **)&buf, delim);
	if ((token) && (!(kstrtoul(token, 0, &val)))
	    && (val >= MNH_HW_INIT) && (val <= MNH_HW_SUSPEND_HIBERNATE)) {
		mnh_state = val;
		return mnh_state;
	}
	return -EINVAL;
}

static DEVICE_ATTR(state, S_IWUSR | S_IRUGO,
		mnh_sm_state_show, mnh_sm_state_store);

static int dma_callback(uint8_t chan, enum mnh_dma_chan_dir_t dir,
		enum mnh_dma_trans_status_t status)
{
	dev_info(mnh_sm_dev->dev, "DMA_CALLBACK: ch:%d, dir:%s, status:%s\n",
		 chan, (dir == DMA_AP2EP)?"READ(AP2EP)":"WRITE(EP2AP)",
		 (status == DMA_DONE)?"DONE":"ABORT");


	if (chan == MNH_PCIE_CHAN_0 && dir == DMA_AP2EP) {
		if (mnh_sm_dev->image_loaded == FW_IMAGE_DOWNLOADING) {
			if (status == DMA_DONE)
				mnh_sm_dev->image_loaded =
				    FW_IMAGE_DOWNLOAD_SUCCESS;
			else if (status == DMA_ABORT)
				mnh_sm_dev->image_loaded =
				    FW_IMAGE_DOWNLOAD_FAIL;
		}
	} else {
		dev_err(mnh_sm_dev->dev, "DMA_CALLBACK: incorrect channel and direction");
		return -EINVAL;
	}

	return 0;
}

static int mnh_firmware_waitdownloaded(void)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(5000);

	do {
		if (mnh_sm_dev->image_loaded == FW_IMAGE_DOWNLOAD_SUCCESS) {
			dev_info(mnh_sm_dev->dev, "Firmware loaded!\n");
			return 0;
		} else if (mnh_sm_dev->image_loaded == FW_IMAGE_DOWNLOAD_FAIL) {
			dev_err(mnh_sm_dev->dev, "Firmware load fail!\n");
			return -EIO;
		}
		msleep(20);
	} while (time_before(jiffies, timeout));

	dev_err(mnh_sm_dev->dev, "Fail to Download Firmware, timeout!!\n");
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

		buf = kmalloc(size, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
		memcpy(buf, fw_data + sent, size);

		dma_blk.dst_addr = dst_addr + sent;
		dma_blk.len = size;
		dma_blk.src_addr = mnh_map_mem(buf, size, DMA_TO_DEVICE);

		if (!dma_blk.src_addr) {
			dev_err(mnh_sm_dev->dev,
				"Could not map dma buffer for FW download\n");
			kfree(buf);
			return -ENOMEM;
		}

		dev_info(mnh_sm_dev->dev, "FW download - AP(:0x%llx) to EP(:0x%llx), size(%d)\n",
			 dma_blk.src_addr, dma_blk.dst_addr, dma_blk.len);

		mnh_sm_dev->image_loaded = FW_IMAGE_DOWNLOADING;
		mnh_dma_sblk_start(MNH_PCIE_CHAN_0, DMA_AP2EP, &dma_blk);

		sent += size;
		remaining -= size;
		dev_info(mnh_sm_dev->dev, "Sent:%zd, Remaining:%zd\n",
			 sent, remaining);

		err = mnh_firmware_waitdownloaded();
		mnh_unmap_mem(dma_blk.src_addr, size, DMA_TO_DEVICE);
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
	err = mnh_reg_irq_callback(0, 0, dma_callback);
	if (err) {
		dev_err(mnh_sm_dev->dev, "register irq callback failed - %d\n",
			err);
		return err;
	}

	err = request_firmware(&fip_img, "easel/fip.bin", mnh_sm_dev->dev);
	if (err) {
		dev_err(mnh_sm_dev->dev, "request fip_image failed - %d\n",
			err);
		return -EIO;
	}

	err = request_firmware(&kernel_img, "easel/Image", mnh_sm_dev->dev);
	if (err) {
		dev_err(mnh_sm_dev->dev, "request kernel failed - %d\n", err);
		goto free_uboot;
	}

	err = request_firmware(&dt_img, "easel/mnh.dtb", mnh_sm_dev->dev);
	if (err) {
		dev_err(mnh_sm_dev->dev, "request kernel failed - %d\n", err);
		goto free_kernel;
	}

	err = request_firmware(&ram_img, "easel/ramdisk.img", mnh_sm_dev->dev);
	if (err) {
		dev_err(mnh_sm_dev->dev, "request kernel failed - %d\n", err);
		goto free_dt;
	}

	/* DMA transfer for SBL */
	memcpy(&size, (uint8_t *)(fip_img->data + FIP_IMG_SBL_SIZE_OFFSET), 4);
	memcpy(&addr, (uint8_t *)(fip_img->data + FIP_IMG_SBL_ADDR_OFFSET), 4);
	dev_info(mnh_sm_dev->dev, "sbl size :0x%x", size);
	dev_info(mnh_sm_dev->dev, "sbl data addr :0x%x", addr);

	dev_info(mnh_sm_dev->dev, "DOWNLOADING SBL...size:0x%x\n", size);
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
	dev_info(mnh_sm_dev->dev, "uboot size :0x%x", size);
	dev_info(mnh_sm_dev->dev, "uboot data addr:0x%x", addr);

	/* DMA transfer for UBOOT */
	dev_info(mnh_sm_dev->dev, "DOWNLOADING UBOOT...size:0x%x\n", size);
	if (mnh_transfer_firmware(size, fip_img->data + addr,
			HW_MNH_UBOOT_DOWNLOAD))
		goto fail_downloading;

	/* DMA transfer for device tree */
	dev_info(mnh_sm_dev->dev, "DOWNLOADING DT...size:%zd\n", dt_img->size);
	if (mnh_transfer_firmware(dt_img->size, dt_img->data,
			HW_MNH_DT_DOWNLOAD))
		goto fail_downloading;

	/* DMA transfer for ramdisk */
	dev_info(mnh_sm_dev->dev, "DOWNLOADING RAMDISK...size:%zd\n",
		 ram_img->size);
	if (mnh_transfer_firmware(ram_img->size, ram_img->data,
			HW_MNH_RAMDISK_DOWNLOAD))
		goto fail_downloading;

	/* DMA transfer for Kernel image */
	dev_info(mnh_sm_dev->dev, "DOWNLOADING KERNEL...size:%zd\n",
		 kernel_img->size);
	if (mnh_transfer_firmware(kernel_img->size, kernel_img->data,
			HW_MNH_KERNEL_DOWNLOAD))
		goto fail_downloading;

	/* PC */
	if (mnh_sm_uboot)
		mnh_config_write(
			HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_7, 4,
			HW_MNH_UBOOT_DOWNLOAD);
	else
		mnh_config_write(
			HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_7, 4,
			HW_MNH_KERNEL_DOWNLOAD);

	/* sbl needs this for its own operation and arg0 for kernel */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_3, 4,
			 HW_MNH_DT_DOWNLOAD);

	release_firmware(fip_img);
	release_firmware(kernel_img);
	release_firmware(dt_img);
	release_firmware(ram_img);

	/* Unregister DMA callback */
	mnh_reg_irq_callback(NULL, NULL, NULL);

	if (mnh_hotplug_cb)
		mnh_hotplug_cb(MNH_HOTPLUG_IN);

	return 0;

fail_downloading:
	dev_err(mnh_sm_dev->dev, "FW downloading fails\n");
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

	dev_info(mnh_sm_dev->dev, "MNH PM mnh_pm_download_show...\n");

	mnh_sm_download();

	return strlen;
}

static DEVICE_ATTR(download, S_IRUSR | S_IRGRP,
		mnh_sm_download_show, NULL);


static ssize_t mnh_sm_suspend_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	ssize_t strlen = 0;

	mnh_sm_suspend(sm_config_1);
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

	mnh_sm_resume(sm_config_1);
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
	&dev_attr_config.attr,
	&dev_attr_state.attr,
	&dev_attr_download.attr,
	&dev_attr_suspend.attr,
	&dev_attr_resume.attr,
	&dev_attr_reset.attr,
	NULL
};

static struct attribute_group mnh_sm_group = {
	.attrs = mnh_sm_dev_attributes
};

/*******************************************************************************
 *
 *      APIs
 *
 ******************************************************************************/

/**
 * API to register hotplug callback to receive MNH up/down notifications
 * @param[in] hotplug_cb  handler for hotplug in/out events
 * @return 0
 */
int mnh_sm_reg_hotplug_callback(hotplug_cb_t hotplug_cb)
{
	mnh_hotplug_cb = hotplug_cb;
	return 0;
}
EXPORT_SYMBOL(mnh_sm_reg_hotplug_callback);

/**
 * API to initialize Power and clocks to MNH.
 * @param[in] Structure argument to configure power and clock component.
 *            This structure will be populated within the kernel module.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_poweron(struct mnh_sm_configuration mnh_sm_boot_args)
{
	mnh_state = MNH_HW_INIT;
	/* Initialize MNH Power */
	mnh_pwr_set_state(MNH_PWR_S0);

	return 0;
}
EXPORT_SYMBOL(mnh_sm_poweron);


/**
 * API to power monette hill.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_poweroff(struct mnh_sm_configuration mnh_sm_boot_args)
{
	mnh_state = MNH_HW_OFF;
	/* Power down MNH */
	mnh_pwr_set_state(MNH_PWR_S4);
	return 0;
}
EXPORT_SYMBOL(mnh_sm_poweroff);

/**
 * API to initialize MIPI, DDR, DDR training,
 * and PCIE.
 * @param[in] Structure argument to configure each boot component.
 *            This structure will be populated within the kernel module.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_config(struct mnh_sm_configuration mnh_sm_boot_args)
{
	/* Initialize DDR */

	/* Initialize DDR training */

	/* Initialze MIPI bypass */
	/* TODO hardcode the to use the first config */
	mnh_sm_mipi_bypass_init(&mnh_sm_boot_args);

	mnh_ddr_po_init(mnh_sm_dev->dev, mnh_sm_boot_args.ddr_config);
	return 0;
}
EXPORT_SYMBOL(mnh_sm_config);

/**
 * API to obtain the state of monette hill.
 * @return the power states of mnh(ex: On, Off, Active, Suspend, Bypass).
 *      MNH_HW_INIT - MNH is on, Kernel not executing, and before FW download.
 *      MNH_HW_OFF - MNH is powered off
 *      MNH_HW_ACTIVE: MNH is on and flashed. Kernel is running.
 *      MNH_HW_SUSPEND_SELF_REFRESH: DDR is self refreshing.
 *                                   All other components are off.
 *      MNH_HW_SUSPEND_HIBERNATE: Hibernation image stored in AP RAM
 *                                over PCIe outbound and MNH is powered down.
 */
int mnh_sm_get_state(void)
{
	return mnh_state;
}
EXPORT_SYMBOL(mnh_sm_get_state);

/**
 * API to set the state of monette hill.
 * @param[in] Set the power states of mnh(ex: On, Off, Active, Suspend, Bypass).
 *      MNH_HW_INIT - MNH is on, Kernel not executing, and before FW download.
 *      MNH_HW_OFF - MNH is powered off
 *      MNH_HW_ACTIVE: MNH is on and flashed. Kernel is running.
 *      MNH_HW_SUSPEND_SELF_REFRESH: DDR is self refreshing.
 *                                   All other components are off.
 *      MNH_HW_SUSPEND_HIBERNATE: Hibernation image stored in AP RAM
 *                                over PCIe outbound and MNH is powered down.
 */
int mnh_sm_set_state(int state)
{
	mnh_state = state;
	return 0;
}
EXPORT_SYMBOL(mnh_sm_set_state);

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
		 * is done on Host side
		 */
		if (mnh_config_read(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET +
				    HW_MNH_PCIE_GP_0,
				    sizeof(uint32_t), &magic) == SUCCESS &&
				    magic == 0) {
			mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET +
					 HW_MNH_PCIE_GP_0,
					 sizeof(uint32_t), INIT_DONE);
		} else {
			dev_err(mnh_sm_dev->dev, "Read GP0 register fail or GP0 is not 0:%d",
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
int mnh_sm_suspend(struct mnh_sm_configuration mnh_sm_boot_args)
{
	mnh_state = MNH_HW_SUSPEND_SELF_REFRESH;
	/* Suspend MNH power */
	mnh_pwr_set_state(MNH_PWR_S3);
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
int mnh_sm_resume(struct mnh_sm_configuration mnh_sm_boot_args)
{
	mnh_state = MNH_HW_ACTIVE;
	/* Initialize MNH Power */
	mnh_pwr_set_state(MNH_PWR_S0);
	return 0;
}
EXPORT_SYMBOL(mnh_sm_resume);

static int mnh_sm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int error = 0;

	dev_dbg(dev, "MNH SM initializing...\n");

	/* allocate device memory */
	mnh_sm_dev = devm_kzalloc(dev, sizeof(struct mnh_sm_device),
				  GFP_KERNEL);
	if (mnh_sm_dev == NULL)
		return -ENOMEM;

	/* add device data to platform device */
	mnh_sm_dev->pdev = pdev;
	mnh_sm_dev->dev = dev;
	dev_set_drvdata(dev, mnh_sm_dev);

	/* allocate character device region */
	error = alloc_chrdev_region(&mnh_sm_dev->dev_num, 0, 1, DEVICE_NAME);
	if (error < 0) {
		dev_err(dev, "alloc_chrdev_region failed (%d)\n", error);
		return error;
	}

	/* make cdev */
	cdev_init(&mnh_sm_dev->cdev, NULL);
	error = cdev_add(&mnh_sm_dev->cdev, mnh_sm_dev->dev_num, 1);
	if (error) {
		dev_err(dev, "cdev_add failed (%d)\n", error);
		goto fail_cdev_add;
	}

	dev_dbg(dev, "char driver %s added", DEVICE_NAME);

	/* create sysfs group */
	error = sysfs_create_group(&dev->kobj, &mnh_sm_group);
	if (error) {
		dev_err(dev, "failed to create /sys/kernel/mnh_sm\n");
		goto fail_sysfs_create_group;
	}

	/* initialize mnh-pwr and get resources there */
	error = mnh_pwr_init(dev);
	if (error) {
		dev_err(dev, "failed to initialize mnh-pwr (%d)\n", error);
		goto fail_mnh_pwr_init;
	}

	dev_info(dev, "MNH SM initialized successfully\n");

	/* TODO: Is this necessary? */
	mnh_sm_poweroff(sm_config_1);

	return 0;

fail_mnh_pwr_init:
	sysfs_remove_group(&dev->kobj, &mnh_sm_group);
fail_sysfs_create_group:
	cdev_del(&mnh_sm_dev->cdev);
fail_cdev_add:
	unregister_chrdev_region(mnh_sm_dev->dev_num, 1);
	devm_kfree(dev, mnh_sm_dev);

	return error;
}

static const struct of_device_id mnh_sm_ids[] = {
	{ .compatible = "intel,mnh-sm", },
	{ },
};

MODULE_DEVICE_TABLE(of, esm_dt_ids);
static struct platform_driver mnh_sm = {
	.probe = mnh_sm_probe,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = mnh_sm_ids,
	},
};

module_platform_driver(mnh_sm);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("MNH State Manager HOST DRIVER");
MODULE_LICENSE("GPL v2");
