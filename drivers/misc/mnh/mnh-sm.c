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

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ion.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/msm_ion.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "hw-mnh-regs.h"
#include "mnh-clk.h"
#include "mnh-ddr.h"
#include "mnh-hwio.h"
#include "mnh-hwio-bases.h"
#include "mnh-hwio-pcie-ss.h"
#include "mnh-hwio-scu.h"
#include "mnh-mipi.h"
#include "mnh-pcie.h"
#include "mnh-pwr.h"
#include "mnh-sm.h"

#define MNH_SCU_INf(reg, fld) \
HW_INf(HWIO_SCU_BASE_ADDR, SCU, reg, fld)
#define MNH_SCU_OUTf(reg, fld, val) \
HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, reg, fld, val)
#define MNH_SCU_OUT(reg, val) \
HW_OUT(HWIO_SCU_BASE_ADDR, SCU, reg, val)
#define MNH_SCU_OUTx(reg, inst, val) \
HW_OUTx(HWIO_SCU_BASE_ADDR, SCU, reg, inst, val)

#define MNH_PCIE_IN(reg) \
HW_IN(HWIO_PCIE_SS_BASE_ADDR, PCIE_SS, reg)
#define MNH_PCIE_INf(reg, fld) \
HW_INf(HWIO_PCIE_SS_BASE_ADDR, PCIE_SS, reg, fld)
#define MNH_PCIE_OUTf(reg, fld, val) \
HW_OUTf(HWIO_PCIE_SS_BASE_ADDR, PCIE_SS, reg, fld, val)
#define MNH_PCIE_OUT(reg, val) \
HW_OUT(HWIO_PCIE_SS_BASE_ADDR, PCIE_SS, reg, val)
#define MNH_PCIE_OUTx(reg, inst, val) \
HW_OUTx(HWIO_PCIE_SS_BASE_ADDR, PCIE_SS, reg, inst, val)

#define MAX_STR_COPY 32
#define SUCCESS 0
#define DEVICE_NAME "mnh_sm"

#define SGL_SIZE 64

#define INIT_DONE 0x1
#define INIT_RESUME 0x2

/* Timeout for waiting for PCIe firmware download */
#define PCIE_FIRMWARE_DOWNLOAD_TIMEOUT msecs_to_jiffies(500)

/* Timeout for waiting for MNH to be powered */
#define POWERED_COMPLETE_TIMEOUT msecs_to_jiffies(5000)

/* Timeout for waiting for MNH to suspend after issuing request */
#define SUSPEND_COMPLETE_TIMEOUT msecs_to_jiffies(5000)

/* Timeout for waiting for MNH set_state to complete */
#define STATE_CHANGE_COMPLETE_TIMEOUT msecs_to_jiffies(5000)

/* Timeout for MNH_HOTPLUG_IN when uart is enabled (in ms) */
#define HOTPLUG_IN_LOOSE_TIMEOUT_MS 15000

/* PCIe */
#define MNH_PCIE_CHAN_0 0

/* Allow partial active mode where PCIe link is up, but Easel is not booted */
#define ALLOW_PARTIAL_ACTIVE 1

/* Firmware update mask, defines userspace updatable slots */
#define FW_SLOT_UPDATE_MASK ((1 << MNH_FW_SLOT_RAMDISK) |\
	(1 << MNH_FW_SLOT_SBL) |\
	(1 << MNH_FW_SLOT_KERNEL) |\
	(1 << MNH_FW_SLOT_DTB))

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

enum fw_image_state {
	FW_IMAGE_NONE = 0,
	FW_IMAGE_DOWNLOADING,
	FW_IMAGE_DOWNLOAD_SUCCESS,
	FW_IMAGE_DOWNLOAD_FAIL
};

enum mnh_ddr_status {
	MNH_DDR_OFF = 0, /* powered off */
	MNH_DDR_ACTIVE, /* powered on, active */
	MNH_DDR_SELF_REFRESH, /* powered on, in self-refresh */
};

enum fw_image_partition_index {
	FW_PART_PRI = 0,
	FW_PART_SEC,
	FW_PART_MAX
};

struct mnh_sm_device {
	struct platform_device *pdev;
	struct miscdevice *misc_dev;
	struct device *dev;
	atomic_t cdev_ctr;
	struct device *chardev;
	struct class *dev_class;
	uint32_t *firmware_buf[2];
	size_t firmware_buf_size[2];
	enum fw_image_state image_loaded;
	int state;

	/* flag for when driver has completed initialization */
	bool initialized;

	/* mutex to synchronize state transitions */
	struct mutex lock;

	/* kernel thread for state transitions from ioctls */
	struct kthread_worker worker;
	struct kthread_work set_state_work;
	struct task_struct *thread;
	int next_state;

	/* completion used to signal when mnh is powered */
	struct completion powered_complete;
	bool powered;

	/* completion used for signaling kthread work is complete */
	struct completion work_complete;
	int work_ret;

	/* pins used for boot and power state transitions */
	struct gpio_desc *boot_mode_gpio;
	struct gpio_desc *ready_gpio;

	/* irq for ready_gpio */
	unsigned int ready_irq;

	/* completion used for synchronizing with secondary bootloader */
	struct completion suspend_complete;

	/* size of the SBL, PBL copies it from DDR to SRAM */
	uint32_t sbl_size;

	/* resume address of kernel, updated before suspending */
	uint32_t resume_addr;

	/* state of the ddr channel */
	enum mnh_ddr_status ddr_status;

	/* pin used for ddr pad isolation */
	struct gpio_desc *ddr_pad_iso_n_pin;

	/* flag to know if firmware has already been downloaded */
	bool firmware_downloaded;

	/* flag indicating a valid update buffer */
	bool pending_update;

	/* node to carveout memory, owned by mnh_sm_device */
	struct mnh_ion *ion[FW_PART_MAX];
};

static struct mnh_sm_device *mnh_sm_dev;
static int mnh_mipi_debug;
static int mnh_freeze_state;

/*
 * bit [0] - bootloader console enabled
 * bit [1] - kernel console enabled
 * bit [2] - kernel stdout-to-console enabled
 */
enum {
	MNH_BOOTARGS_BL_CONSOLE_ENABLE     = 1 << 0,
	MNH_BOOTARGS_KERNEL_CONSOLE_ENABLE = 1 << 1,
	MNH_BOOTARGS_STDOUT_CONSOLE_ENABLE = 1 << 2,
};
#define MNH_UART_ENABLE (MNH_BOOTARGS_BL_CONSOLE_ENABLE | \
			 MNH_BOOTARGS_KERNEL_CONSOLE_ENABLE | \
			 MNH_BOOTARGS_STDOUT_CONSOLE_ENABLE)
static unsigned int mnh_boot_args;

/* 32-bit flag mask to SCU for boot power options */
enum {
	MNH_POWER_MODE_CLKPM_ENABLE  = 1 << 0,
	MNH_POWER_MODE_L1_2_ENABLE   = 1 << 1,
	MNH_POWER_MODE_AXI_CG_ENABLE = 1 << 2,
};
static uint32_t mnh_power_mode = MNH_POWER_MODE_CLKPM_ENABLE;

/* flag for boot mode options */
static enum mnh_boot_mode mnh_boot_mode = MNH_BOOT_MODE_PCIE;

/* callback when easel enters and leaves the active state */
static hotplug_cb_t mnh_hotplug_cb;

static int mnh_sm_get_val_from_buf(const char *buf, unsigned long *val)
{
	uint8_t *token;
	const char *delim = ";";

	token = strsep((char **)&buf, delim);
	if ((token) && (!(kstrtoul(token, 0, val))))
		return 0;
	else
		return -EINVAL;
}

static ssize_t stage_fw_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	dev_dbg(mnh_sm_dev->dev, "Entering mnh_sm_stage_fw_show...\n");

	return scnprintf(buf, MAX_STR_COPY, "%s\n",
			 mnh_sm_dev->ion[FW_PART_PRI]->is_fw_ready
			 ? "Firmware staged to ION"
			 : "Firmware not staged");

	return 0;
}

static ssize_t stage_fw_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{

	unsigned long val = 0;
	int ret;

	dev_dbg(mnh_sm_dev->dev, "Entering mnh_sm_stage_fw_store...\n");

	ret = mnh_sm_get_val_from_buf(buf, &val);
	if (!ret) {
		if (val)
			mnh_ion_stage_firmware(mnh_sm_dev->ion[FW_PART_PRI]);
		return count;
	}

	return -EINVAL;
}

static DEVICE_ATTR_RW(stage_fw);

static ssize_t poweron_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	ssize_t strlen = 0;

	dev_dbg(dev, "Entering mnh_sm_poweron_show...\n");
	mnh_sm_set_state(MNH_STATE_ACTIVE);
	return strlen;
}

static DEVICE_ATTR_RO(poweron);

static ssize_t poweroff_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	ssize_t strlen = 0;

	dev_dbg(dev, "Entering mnh_sm_poweroff_show...\n");
	mnh_sm_set_state(MNH_STATE_OFF);

	return strlen;
}

static DEVICE_ATTR_RO(poweroff);

static ssize_t state_show(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	dev_dbg(dev, "Entering mnh_sm_state_show...\n");

	return scnprintf(buf, MAX_STR_COPY, "%d\n", mnh_sm_dev->state);
}

static ssize_t state_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf,
			   size_t count)
{
	unsigned long val = 0;
	int ret;

	dev_dbg(dev, "Entering mnh_sm_state_store...\n");

	ret = mnh_sm_get_val_from_buf(buf, &val);
	mnh_sm_set_state((int)val);
	return count;
}

static DEVICE_ATTR_RW(state);

static int dma_callback(uint8_t chan, enum mnh_dma_chan_dir_t dir,
		enum mnh_dma_trans_status_t status)
{
	dev_dbg(mnh_sm_dev->dev, "DMA_CALLBACK: ch:%d, dir:%s, status:%s\n",
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
	unsigned long timeout = jiffies + PCIE_FIRMWARE_DOWNLOAD_TIMEOUT;

	do {
		if (mnh_sm_dev->image_loaded == FW_IMAGE_DOWNLOAD_SUCCESS) {
			dev_dbg(mnh_sm_dev->dev, "Firmware loaded!\n");
			return 0;
		} else if (mnh_sm_dev->image_loaded == FW_IMAGE_DOWNLOAD_FAIL) {
			dev_err(mnh_sm_dev->dev, "Firmware load fail!\n");
			return -EIO;
		}
		usleep_range(2000, 5000);
	} while (time_before(jiffies, timeout));

	dev_err(mnh_sm_dev->dev, "Fail to Download Firmware, timeout!!\n");
	return -EIO;
}

static size_t mnh_get_firmware_buf(struct device *dev, uint32_t **buf)
{
	size_t size = IMG_DOWNLOAD_MAX_SIZE;

	while (size > 0) {
		*buf = devm_kmalloc(dev, size, GFP_KERNEL);
		if (*buf)
			break;

		size >>= 1;
	}

	return size;
}

static int mnh_transfer_firmware_contig(size_t fw_size, dma_addr_t src_addr,
	uint64_t dst_addr)
{
	struct mnh_dma_element_t dma_blk;
	int err = -EINVAL;

	dma_blk.dst_addr = dst_addr;
	dma_blk.len = fw_size;
	dma_blk.src_addr = src_addr;

	dev_dbg(mnh_sm_dev->dev,
		"FW download - AP(:0x%llx) to EP(:0x%llx), size(%d)\n",
		dma_blk.src_addr, dma_blk.dst_addr, dma_blk.len);

	mnh_sm_dev->image_loaded = FW_IMAGE_DOWNLOADING;
	mnh_dma_sblk_start(MNH_PCIE_CHAN_0, DMA_AP2EP, &dma_blk);

	err = mnh_firmware_waitdownloaded();

	return err;
}

static int mnh_transfer_firmware(size_t fw_size, const uint8_t *fw_data,
	uint64_t dst_addr)
{
	uint32_t *buf;
	size_t buf_size;
	int buf_index = 0;
	struct mnh_dma_element_t dma_blk;
	int err = -EINVAL;
	size_t sent = 0, size, remaining;

	remaining = fw_size;

	while (remaining > 0) {
		buf = mnh_sm_dev->firmware_buf[buf_index];
		buf_size = mnh_sm_dev->firmware_buf_size[buf_index];

		size = MIN(remaining, buf_size);

		memcpy(buf, fw_data + sent, size);

		if (mnh_sm_dev->image_loaded == FW_IMAGE_DOWNLOADING) {
			err = mnh_firmware_waitdownloaded();
			mnh_unmap_mem(dma_blk.src_addr, size, DMA_TO_DEVICE);
			if (err)
				break;
		}

		dma_blk.dst_addr = dst_addr + sent;
		dma_blk.len = size;
		dma_blk.src_addr = mnh_map_mem(buf, size, DMA_TO_DEVICE);

		if (!dma_blk.src_addr) {
			dev_err(mnh_sm_dev->dev,
				"Could not map dma buffer for FW download\n");
			return -ENOMEM;
		}

		dev_dbg(mnh_sm_dev->dev, "FW download - AP(:0x%llx) to EP(:0x%llx), size(%d)\n",
			 dma_blk.src_addr, dma_blk.dst_addr, dma_blk.len);

		mnh_sm_dev->image_loaded = FW_IMAGE_DOWNLOADING;
		mnh_dma_sblk_start(MNH_PCIE_CHAN_0, DMA_AP2EP, &dma_blk);

		sent += size;
		remaining -= size;
		buf_index = (buf_index + 1) & 0x1;
		dev_dbg(mnh_sm_dev->dev, "Sent:%zd, Remaining:%zd\n",
			 sent, remaining);
	}

	if (mnh_sm_dev->image_loaded == FW_IMAGE_DOWNLOADING) {
		err = mnh_firmware_waitdownloaded();
		mnh_unmap_mem(dma_blk.src_addr, size, DMA_TO_DEVICE);
	}

	return err;
}

int mnh_transfer_slot_ion(struct mnh_ion *ion, int slot)
{
	int err;
	uint32_t sbl_size, sbl_offset;

	dev_dbg(mnh_sm_dev->dev, "ION downloading fw[%d]...\n", slot);
	if (slot == MNH_FW_SLOT_SBL) {
		/* extract SBL from fip.bin image */
		memcpy(&sbl_size, (uint8_t *)(ion->vaddr
			+ ion->fw_array[slot].ap_offs
			+ FIP_IMG_SBL_SIZE_OFFSET),
			sizeof(sbl_size));
		memcpy(&sbl_offset, (uint8_t *)(ion->vaddr
			+ ion->fw_array[slot].ap_offs
			+ FIP_IMG_SBL_ADDR_OFFSET),
			sizeof(sbl_offset));
		dev_dbg(mnh_sm_dev->dev,
			"SBL offset %d, SBL size %d\n",
			sbl_offset, sbl_size);
		mnh_sm_dev->sbl_size = sbl_size;

		err = mnh_transfer_firmware_contig(
			sbl_size,
			ion->fw_array[slot].ap_addr + sbl_offset,
			ion->fw_array[slot].ep_addr);
	} else {
		err = mnh_transfer_firmware_contig(
			ion->fw_array[slot].size,
			ion->fw_array[slot].ap_addr,
			ion->fw_array[slot].ep_addr);
	}
	return err;
}

int mnh_download_firmware_ion(struct mnh_ion *ion[FW_PART_MAX])
{
	int err;
	int i;

	if (!ion)
		return -ENODEV;

	if (!ion[FW_PART_PRI]->is_fw_ready)
		return -EAGAIN;

	mnh_sm_dev->image_loaded = FW_IMAGE_NONE;

	/* Register DMA callback */
	err = mnh_reg_irq_callback(0, 0, dma_callback);
	if (err) {
		dev_err(mnh_sm_dev->dev,
			"register irq callback failed - %d\n", err);
		return err;
	}

	for (i = 0; i < MAX_NR_MNH_FW_SLOTS; i++) {
		/* only upload from primary ION buffer */
		err = mnh_transfer_slot_ion(ion[FW_PART_PRI], i);
		if (err) {
			/* Unregister DMA callback */
			dev_err(mnh_sm_dev->dev,
				"failed FW transfer of slot %d: %d\n", i, err);
			mnh_reg_irq_callback(NULL, NULL, NULL);
			return err;
		}
	}

	/* Configure sbl addresses and size */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_4, 4,
		HW_MNH_SBL_DOWNLOAD);
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_5, 4,
		HW_MNH_SBL_DOWNLOAD_EXE);
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_6, 4,
			 mnh_sm_dev->sbl_size);

	/* Configure post sbl entry address */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_7, 4,
			 HW_MNH_KERNEL_DOWNLOAD);

	/* sbl needs this for its own operation and arg0 for kernel */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_3, 4,
			 HW_MNH_DT_DOWNLOAD);
	/* Unregister DMA callback */
	mnh_reg_irq_callback(NULL, NULL, NULL);

	mnh_sm_dev->firmware_downloaded = true;

	return 0;
}

int mnh_download_firmware_legacy(void)
{
	const struct firmware *dt_img, *kernel_img, *ram_img;
	const struct firmware *fip_img;
	int err;
	uint32_t size, addr;
	int i;

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
		goto free_fip;
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

	/* get double buffers for transferring firmware */
	for (i = 0; i < 2; i++) {
		mnh_sm_dev->firmware_buf_size[i] =
			mnh_get_firmware_buf(mnh_sm_dev->dev,
					     &mnh_sm_dev->firmware_buf[i]);
		if (!mnh_sm_dev->firmware_buf_size[i]) {
			dev_err(mnh_sm_dev->dev,
				"%s: could not allocate a buffer for firmware transfers\n",
				__func__);
			return -ENOMEM;
		}
	}

	/* DMA transfer for SBL */
	memcpy(&size, (uint8_t *)(fip_img->data + FIP_IMG_SBL_SIZE_OFFSET),
		sizeof(size));
	memcpy(&addr, (uint8_t *)(fip_img->data + FIP_IMG_SBL_ADDR_OFFSET),
		sizeof(addr));
	mnh_sm_dev->sbl_size = size;

	dev_dbg(mnh_sm_dev->dev, "sbl data addr :0x%x", addr);
	dev_dbg(mnh_sm_dev->dev, "DOWNLOADING SBL...size:0x%x\n", size);
	if (mnh_transfer_firmware(size, fip_img->data + addr,
			HW_MNH_SBL_DOWNLOAD))
		goto fail_downloading;

	/* DMA transfer for device tree */
	dev_dbg(mnh_sm_dev->dev, "DOWNLOADING DT...size:%zd\n", dt_img->size);
	if (mnh_transfer_firmware(dt_img->size, dt_img->data,
			HW_MNH_DT_DOWNLOAD))
		goto fail_downloading;

	/* DMA transfer for ramdisk */
	dev_dbg(mnh_sm_dev->dev, "DOWNLOADING RAMDISK...size:%zd\n",
		 ram_img->size);
	if (mnh_transfer_firmware(ram_img->size, ram_img->data,
			HW_MNH_RAMDISK_DOWNLOAD))
		goto fail_downloading;

	/* DMA transfer for Kernel image */
	dev_dbg(mnh_sm_dev->dev, "DOWNLOADING KERNEL...size:%zd\n",
		 kernel_img->size);
	if (mnh_transfer_firmware(kernel_img->size, kernel_img->data,
			HW_MNH_KERNEL_DOWNLOAD))
		goto fail_downloading;

	/* Configure sbl addresses and size */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_4, 4,
		HW_MNH_SBL_DOWNLOAD);
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_5, 4,
		HW_MNH_SBL_DOWNLOAD_EXE);
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_6, 4,
		size);

	/* Configure post sbl entry address */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_7, 4,
			 HW_MNH_KERNEL_DOWNLOAD);

	/* sbl needs this for its own operation and arg0 for kernel */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_3, 4,
			 HW_MNH_DT_DOWNLOAD);

	for (i = 0; i < 2; i++)
		devm_kfree(mnh_sm_dev->dev, mnh_sm_dev->firmware_buf[i]);

	release_firmware(fip_img);
	release_firmware(kernel_img);
	release_firmware(dt_img);
	release_firmware(ram_img);

	/* Unregister DMA callback */
	mnh_reg_irq_callback(NULL, NULL, NULL);

	mnh_sm_dev->firmware_downloaded = true;

	return 0;

fail_downloading:
	dev_err(mnh_sm_dev->dev, "FW downloading fails\n");
	mnh_sm_dev->image_loaded = FW_IMAGE_DOWNLOAD_FAIL;
	release_firmware(ram_img);
free_dt:
	release_firmware(dt_img);
free_kernel:
	release_firmware(kernel_img);
free_fip:
	release_firmware(fip_img);

	/* Unregister DMA callback */
	mnh_reg_irq_callback(NULL, NULL, NULL);
	return -EIO;
}

int mnh_download_firmware(void)
{
	int err;

	/* Prefer to download from ION buffer if it's ready */
	err = mnh_download_firmware_ion(mnh_sm_dev->ion);
	if (!err) {
		dev_dbg(mnh_sm_dev->dev,
			"%s: ION download successful\n", __func__);
		return 0;
	}

	/* Otherwise fall back to legacy mode */
	dev_err(mnh_sm_dev->dev, "%s: Fallback to legacy mode\n", __func__);
	return mnh_download_firmware_legacy();
}

/**
 * Verify the integrity of the images in the secondary ion buffer
 * and populate the fw_array structure
 * @return 0 if success or -1 on failure to verify the image integrity
 */
int mnh_validate_update_buf(struct mnh_update_configs configs)
{
	int i, slot;
	int err_mask = 0;
	struct mnh_update_config config;

	if (!mnh_sm_dev->ion[FW_PART_SEC])
		return -ENOMEM;

	/* Initialize update buffer struct */
	for (i = 0; i < MAX_NR_MNH_FW_SLOTS; i++) {
		mnh_sm_dev->ion[FW_PART_SEC]->fw_array[i].ap_offs = 0;
		mnh_sm_dev->ion[FW_PART_SEC]->fw_array[i].size = 0;
	}
	/* Parse update slots */
	for (i = 0; i < MAX_NR_MNH_FW_SLOTS; i++) {
		config = configs.config[i];
		slot = config.slot_type;
		/* only configure slots masked by FW_SLOT_UPDATE_MASK */
		if ((FW_SLOT_UPDATE_MASK & (1 << slot)) &&
		    (config.size > 0)) {
			mnh_sm_dev->ion[FW_PART_SEC]->fw_array[slot].ap_offs =
				config.offset;
			mnh_sm_dev->ion[FW_PART_SEC]->fw_array[slot].size =
				config.size;
			dev_dbg(mnh_sm_dev->dev,
				"%s: Validating slot[%d]: size %zd offs %lu\n",
				__func__, slot, config.size, config.offset);
			/* perform signature check */
			dev_dbg(mnh_sm_dev->dev, "%s: Performing sig. check\n",
				__func__);
			err_mask = 0; /* todo: b/36782736 add sig.check here */
		}
	}

	/* all update slots must verify positive or they are discarded */
	if (err_mask) {
		dev_err(mnh_sm_dev->dev, "%s: Update firmware tainted\n",
			__func__);
		mnh_sm_dev->ion[FW_PART_SEC]->is_fw_ready = false;
		mnh_sm_dev->pending_update = false;
		return err_mask;
	}
	dev_dbg(mnh_sm_dev->dev, "%s: Update firmware validated\n",
		__func__);
	mnh_sm_dev->ion[FW_PART_SEC]->is_fw_ready = true;
	mnh_sm_dev->pending_update = true;
	return 0;
}

static ssize_t download_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	ssize_t strlen = 0;

	dev_dbg(mnh_sm_dev->dev, "MNH PM mnh_pm_download_show...\n");

	mnh_sm_set_state(MNH_STATE_ACTIVE);

	return strlen;
}

static DEVICE_ATTR_RO(download);

static ssize_t suspend_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	ssize_t strlen = 0;

	mnh_sm_set_state(MNH_STATE_SUSPEND);

	return strlen;
}

static DEVICE_ATTR_RO(suspend);

int mnh_resume_firmware(void)
{
	dev_dbg(mnh_sm_dev->dev, "%s sbl dl:0x%x ex:0x%x size:0x%x\n", __func__,
		HW_MNH_SBL_DOWNLOAD, HW_MNH_SBL_DOWNLOAD_EXE,
		mnh_sm_dev->sbl_size);
	dev_dbg(mnh_sm_dev->dev, "%s resume:0x%x dt:0x%x\n", __func__,
		mnh_sm_dev->resume_addr, HW_MNH_DT_DOWNLOAD);

	/* Configure sbl addresses and size */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_4, 4,
		HW_MNH_SBL_DOWNLOAD);
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_5, 4,
		HW_MNH_SBL_DOWNLOAD_EXE);
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_6, 4,
		mnh_sm_dev->sbl_size);

	/* Configure resume entry address */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_7, 4,
		mnh_sm_dev->resume_addr);

	/* sbl needs this for its own operation and arg0 for kernel */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_3, 4,
			 HW_MNH_DT_DOWNLOAD);

	/* Configure boot command to resume */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_0,
		sizeof(uint32_t), INIT_RESUME);

	return 0;
}

static ssize_t resume_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	ssize_t strlen = 0;

	mnh_sm_set_state(MNH_STATE_ACTIVE);

	return strlen;
}

static DEVICE_ATTR_RO(resume);

static ssize_t reset_show(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	ssize_t strlen = 0;
	return strlen;
}

static DEVICE_ATTR_RO(reset);

static ssize_t cpu_clk_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t count)
{
	unsigned long val = 0;
	int ret;

	dev_dbg(mnh_sm_dev->dev, "Entering mnh_sm_cpu_clk_store...\n");

	if (count > 10)
		return -EINVAL;

	ret = mnh_sm_get_val_from_buf(buf, &val);
	if (!ret && (val >= CPU_FREQ_MIN) && (val <= CPU_FREQ_MAX)) {
		mnh_cpu_freq_change(val);
		return count;
	}

	return -EINVAL;
}

static DEVICE_ATTR_WO(cpu_clk);

static ssize_t debug_mipi_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	dev_dbg(mnh_sm_dev->dev, "Entering mnh_sm_debug_mipi_show...\n");

	return scnprintf(buf, MAX_STR_COPY, "%d\n", mnh_mipi_debug);
}

static ssize_t debug_mipi_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val = 0;
	int ret;

	dev_dbg(mnh_sm_dev->dev, "Entering mnh_sm_debug_mipi_store...\n");

	ret = mnh_sm_get_val_from_buf(buf, &val);
	if (!ret) {
		mnh_mipi_debug = val;
		mnh_mipi_set_debug(val);
		return count;
	}

	return -EINVAL;
}

static DEVICE_ATTR_RW(debug_mipi);

static ssize_t freeze_state_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	dev_dbg(mnh_sm_dev->dev, "Entering mnh_sm_freeze_state_show...\n");

	return scnprintf(buf, MAX_STR_COPY, "%d\n", mnh_freeze_state);
}

static ssize_t freeze_state_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	unsigned long val = 0;
	int ret;

	dev_dbg(mnh_sm_dev->dev, "Entering mnh_sm_freeze_state_store...\n");

	ret = mnh_sm_get_val_from_buf(buf, &val);
	if (!ret) {
		mnh_freeze_state = val;
		return count;
	}

	return -EINVAL;
}

static DEVICE_ATTR_RW(freeze_state);

/* temporary sysfs hook to test mipi shutdown of the available channels */
static ssize_t mipi_stop_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	struct mnh_mipi_config cfg;
	unsigned long val = 0;
	int ret;

	dev_dbg(mnh_sm_dev->dev, "%s: Entering mnh_sm_mipi_store...\n",
		__func__);

	ret = mnh_sm_get_val_from_buf(buf, &val);
	if (!ret) {
		if (val < 3) {
			cfg.rxdev = cfg.txdev = val;
			dev_dbg(mnh_sm_dev->dev,
				"%s: Shutting down mipi tx dev %d, rx dev %d\n",
				__func__, cfg.txdev, cfg.rxdev);
			mnh_mipi_stop(mnh_sm_dev->dev, cfg);
		} else {
			dev_dbg(mnh_sm_dev->dev, "%s: Invalid MIPI channel\n",
				__func__);
		}
		return count;
	}
	dev_err(mnh_sm_dev->dev, "%s: Usage: echo\"<channel>\">mipi_stop\n",
		__func__);
	return -EINVAL;
}

static DEVICE_ATTR_WO(mipi_stop);

static ssize_t boot_args_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	dev_dbg(mnh_sm_dev->dev, "Entering mnh_sm_boot_args_show...\n");

	return scnprintf(buf, MAX_STR_COPY, "%d\n", mnh_boot_args);
}

static ssize_t boot_args_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	unsigned long val = 0;
	int ret;

	dev_dbg(mnh_sm_dev->dev, "Entering mnh_sm_boot_args_store...\n");

	ret = mnh_sm_get_val_from_buf(buf, &val);
	if (!ret) {
		mnh_boot_args = val;
		return count;
	}

	return -EINVAL;
}

static DEVICE_ATTR_RW(boot_args);

static ssize_t boot_trace_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int err;
	uint32_t val;

	err = mnh_config_read(MNH_BOOT_TRACE, sizeof(val), &val);
	if (!err)
		return scnprintf(buf, MAX_STR_COPY, "%x\n", val);
	return scnprintf(buf, MAX_STR_COPY, "Unavailable\n");
}

static DEVICE_ATTR_RO(boot_trace);

static ssize_t enable_uart_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, MAX_STR_COPY, "%d\n", mnh_boot_args &
			 MNH_UART_ENABLE);
}

static ssize_t enable_uart_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	unsigned long val = 0;
	int ret;

	ret = mnh_sm_get_val_from_buf(buf, &val);
	if (!ret) {
		if (val)
			mnh_boot_args |= MNH_UART_ENABLE;
		else
			mnh_boot_args &= ~MNH_UART_ENABLE;
		return count;
	}

	return -EINVAL;
}

static DEVICE_ATTR_RW(enable_uart);

static ssize_t power_mode_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	return scnprintf(buf, MAX_STR_COPY, "%d\n", mnh_power_mode);
}

static ssize_t power_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int val = 0;
	int ret;

	ret = kstrtoint(buf, 0, &val);
	if (!ret) {
		mnh_power_mode = val;
		return count;
	}

	return -EINVAL;
}

static DEVICE_ATTR_RW(power_mode);

static int mnh_sm_read_mipi_interrupts(void)
{
	int i, int_event = 0;
	struct mipi_device_irq_st dev_ints;
	struct mipi_host_irq_st host_ints;

	memset((void *)&dev_ints, 0, sizeof(dev_ints));
	memset((void *)&host_ints, 0, sizeof(host_ints));

	dev_info(mnh_sm_dev->dev, "%s: Querying MIPI interrupts\n", __func__);
	/* Read device & top interrupts */
	for (i = 0; i <= 1; i++) {
		dev_ints.dev = i;
		mnh_mipi_get_device_interrupts(mnh_sm_dev->dev, &dev_ints);
		if (dev_ints.main || dev_ints.fifo_overflow)
			int_event = 1;
	}
	/* Read host interrupts */
	for (i = 0; i <= 2; i++) {
		host_ints.dev = i;
		mnh_mipi_get_host_interrupts(mnh_sm_dev->dev, &host_ints);
		if (host_ints.main)
			int_event = 1;
	}

	return int_event;
}

static ssize_t mipi_int_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	return scnprintf(buf, MAX_STR_COPY, "%d\n",
			 mnh_sm_read_mipi_interrupts());
}

static DEVICE_ATTR_RO(mipi_int);

static ssize_t spi_boot_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, MAX_STR_COPY, "%d\n", mnh_boot_mode);
}

static ssize_t spi_boot_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int val = 0;
	int ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret || (val < MNH_BOOT_MODE_PCIE) || (val > MNH_BOOT_MODE_SPI))
		return -EINVAL;

	if (val != mnh_boot_mode) {
		mnh_pwr_set_state(MNH_PWR_S4);
		gpiod_set_value(mnh_sm_dev->boot_mode_gpio, val);
		mnh_boot_mode = val;

		if (val == MNH_BOOT_MODE_SPI)
			mnh_pwr_set_state(MNH_PWR_S0);
	}

	return count;
}

static DEVICE_ATTR_RW(spi_boot_mode);

static ssize_t error_event_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, MAX_STR_COPY, "%d\n", 0);
}

static DEVICE_ATTR_RO(error_event);

static struct attribute *mnh_sm_attrs[] = {
	&dev_attr_stage_fw.attr,
	&dev_attr_poweron.attr,
	&dev_attr_poweroff.attr,
	&dev_attr_state.attr,
	&dev_attr_download.attr,
	&dev_attr_suspend.attr,
	&dev_attr_resume.attr,
	&dev_attr_reset.attr,
	&dev_attr_cpu_clk.attr,
	&dev_attr_debug_mipi.attr,
	&dev_attr_freeze_state.attr,
	&dev_attr_mipi_stop.attr,
	&dev_attr_boot_args.attr,
	&dev_attr_enable_uart.attr,
	&dev_attr_power_mode.attr,
	&dev_attr_mipi_int.attr,
	&dev_attr_spi_boot_mode.attr,
	&dev_attr_boot_trace.attr,
	&dev_attr_error_event.attr,
	NULL
};
ATTRIBUTE_GROUPS(mnh_sm);

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

static int mnh_sm_hotplug_callback(enum mnh_hotplug_event_t event)
{
	if (!mnh_hotplug_cb)
		return -EFAULT;

	if ((event == MNH_HOTPLUG_IN) && (mnh_boot_args & MNH_UART_ENABLE)) {
		dev_info(mnh_sm_dev->dev,
			 "%s: allow %d secs for MNH_HOTPLUG_IN\n",
			 __func__, HOTPLUG_IN_LOOSE_TIMEOUT_MS / 1000);
		return mnh_hotplug_cb(event,
				      (void *)HOTPLUG_IN_LOOSE_TIMEOUT_MS);
	}

	return mnh_hotplug_cb(event, NULL);
}

/**
 * API to initialize Power and clocks to MNH.
 * @param[in] Structure argument to configure power and clock component.
 *            This structure will be populated within the kernel module.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
static int mnh_sm_poweron(void)
{
	int ret;

	/* Initialize MNH Power */
	ret = mnh_pwr_set_state(MNH_PWR_S0);
	if (ret) {
		dev_err(mnh_sm_dev->dev,
			"%s: failed to power on device (%d)\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

/**
 * API to power monette hill.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
static int mnh_sm_poweroff(void)
{
	mnh_pwr_set_state(MNH_PWR_S4);
	mnh_sm_dev->ddr_status = MNH_DDR_OFF;
	mnh_sm_dev->firmware_downloaded = false;
	return 0;
}

static int mnh_sm_config_ddr(void)
{
	int ret;

	/* Initialize DDR */
	ret = mnh_ddr_po_init(mnh_sm_dev->dev, mnh_sm_dev->ddr_pad_iso_n_pin);
	if (ret) {
		dev_err(mnh_sm_dev->dev, "%s: ddr training failed (%d)\n",
			__func__, ret);
		return ret;
	}

	mnh_sm_dev->ddr_status = MNH_DDR_ACTIVE;
	return 0;
}

static int mnh_sm_resume_ddr(void)
{
	/* deassert pad isolation, take ddr out of self-refresh mode */
	mnh_ddr_resume(mnh_sm_dev->dev, mnh_sm_dev->ddr_pad_iso_n_pin);
	mnh_sm_dev->ddr_status = MNH_DDR_ACTIVE;
	return 0;
}

static int mnh_sm_suspend_ddr(void)
{
	/* put ddr into self-refresh mode, assert pad isolation */
	mnh_ddr_suspend(mnh_sm_dev->dev, mnh_sm_dev->ddr_pad_iso_n_pin);
	mnh_sm_dev->ddr_status = MNH_DDR_SELF_REFRESH;
	return 0;
}

/**
 * API to download the binary images(SBL, UBoot, Kernel, Ramdisk) for mnh.
 * The location of the binaries will be located in the AP file system.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
static int mnh_sm_download(void)
{
	uint32_t  magic;
	int ret;
	ktime_t start, end;
	static int iter;

	iter++;
	start = ktime_get();
	ret = mnh_download_firmware();
	end = ktime_get();
	dev_dbg(mnh_sm_dev->dev, "iter:%d took %d ms to download firmware\n",
		 iter, (unsigned)ktime_to_ms(ktime_sub(end, start)));

	if (ret) {
		dev_err(mnh_sm_dev->dev,
			"%s: firmware download failed\n", __func__);
		return ret;
	}

	/* set the boot_args mask */
	mnh_config_write(HWIO_SCU_GP_ADDR(HWIO_SCU_BASE_ADDR, 2), 4,
			 mnh_boot_args);

	/* set the default power mode */
	MNH_SCU_OUT(GP_POWER_MODE, mnh_power_mode);

	/*
	 * Magic number setting to notify MNH that PCIE initialization
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
		dev_err(mnh_sm_dev->dev,
			"Read GP0 register fail or GP0 is not 0:%d",
			magic);
	}

	return 0;
}

/**
 * API to put MNH in suspend state.  In suspend mode the DDR will be isolated
 * and put in self refresh while the CPU is powered down.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
static int mnh_sm_suspend(void)
{
	unsigned long timeout;
	int ret;

	dev_dbg(mnh_sm_dev->dev, "%s: waiting for suspend signal\n", __func__);

	/* Wait until MNH is ready to go to suspend */
	timeout = wait_for_completion_timeout(&mnh_sm_dev->suspend_complete,
					      SUSPEND_COMPLETE_TIMEOUT);
	if (!timeout) {
		dev_err(mnh_sm_dev->dev,
			"%s: timeout waiting for device to suspend kernel\n",
			__func__);
		return -ETIMEDOUT;
	}

	dev_dbg(mnh_sm_dev->dev, "%s: suspend successful.\n", __func__);

	/* Read resume entry address */
	ret = mnh_config_read(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET +
			      HW_MNH_PCIE_GP_7, sizeof(uint32_t),
			      &mnh_sm_dev->resume_addr);
	if (ret != SUCCESS) {
		dev_err(mnh_sm_dev->dev, "%s: resume entry addr read failed\n",
			__func__);
		return -EIO;
	}
	dev_dbg(mnh_sm_dev->dev, "%s: resume entry: 0x%x\n", __func__,
		mnh_sm_dev->resume_addr);

	mnh_sm_suspend_ddr();

	/* Suspend MNH power */
	mnh_pwr_set_state(MNH_PWR_S3);
	return 0;
}

/**
 * API to put MNH into active state.
 * The resume call flow should be similar to normal bootflow except for DDR
 * initializations. Since the binaries are already saved on the DDR while MNH
 * is in suspend, ESM will not need to download the binaries again during
 * resume.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
static int mnh_sm_resume(void)
{
	mnh_resume_firmware();
	return 0;
}

/**
 * API to obtain the state of monette hill.
 * @return the power states of mnh
 */
int mnh_sm_get_state(void)
{
	int state;

	if (!mnh_sm_dev)
		return MNH_STATE_OFF;

	mutex_lock(&mnh_sm_dev->lock);
	state = mnh_sm_dev->state;
	mutex_unlock(&mnh_sm_dev->lock);

	return state;
}
EXPORT_SYMBOL(mnh_sm_get_state);

static void mnh_sm_print_boot_trace(struct device (*dev))
{
	int err;
	uint32_t val;

	err = mnh_config_read(MNH_BOOT_TRACE, sizeof(val), &val);

	if (err) {
		dev_err(dev,
			"%s: failed reading MNH_BOOT_TRACE (%d)\n",
			__func__, err);
		return;
	}

	dev_info(dev, "%s: MNH_BOOT_TRACE = 0x%x\n", __func__, val);
}

/*
 * NOTE (b/64372955): Put Easel into a low-power mode for MIPI bypass. Ideally,
 * this would be done from Easel kernel, but if the Easel kernel fails for some
 * reason, we can disable a lot of clocks on Easel to reduce power.
 */
static void mnh_sm_enter_low_power_mode(void)
{
	MNH_SCU_OUTf(CCU_CLK_CTL, CPU_CLKEN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, BTSRAM_CLKEN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, BTROM_CLKEN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, LP4_REFCLKEN, 0);
	MNH_SCU_OUTf(CCU_CLK_CTL, IPU_CLKEN, 0);
}

static int mnh_sm_set_state_locked(int state)
{
	struct device *dev = mnh_sm_dev->dev;
	int ret = 0;

	if (state == mnh_sm_dev->state) {
		dev_dbg(dev, "%s: already in state %d\n", __func__, state);
		return 0;
	}

	switch (state) {
	case MNH_STATE_OFF:
		mnh_sm_hotplug_callback(MNH_HOTPLUG_OUT);

		/* toggle powered flag and clear completion */
		mnh_sm_dev->powered = false;
		reinit_completion(&mnh_sm_dev->powered_complete);

		/* stage firmware copy to ION if valid update was received */
		if (mnh_sm_dev->pending_update) {
			dev_dbg(mnh_sm_dev->dev,
				"%s: staging firmware update",
				__func__);
			mnh_ion_stage_firmware_update(
				mnh_sm_dev->ion[FW_PART_PRI],
				mnh_sm_dev->ion[FW_PART_SEC]);
			mnh_sm_dev->pending_update = false;
			mnh_ion_destroy_buffer(mnh_sm_dev->ion[FW_PART_SEC]);
		}

		ret = mnh_sm_poweroff();

		disable_irq(mnh_sm_dev->ready_irq);
		break;
	case MNH_STATE_ACTIVE:
		enable_irq(mnh_sm_dev->ready_irq);

		ret = mnh_sm_poweron();
		if (ret)
			break;

		/* toggle powered flag and notify any waiting threads */
		mnh_sm_dev->powered = true;
		complete(&mnh_sm_dev->powered_complete);

		/* make sure ddr is configured */
		if (mnh_sm_dev->ddr_status == MNH_DDR_OFF)
			ret = mnh_sm_config_ddr();
		else if (mnh_sm_dev->ddr_status == MNH_DDR_SELF_REFRESH)
			ret = mnh_sm_resume_ddr();
		if (ret)
			break;

		/* use max CPU frequency for fast boot */
		mnh_cpu_freq_change(CPU_FREQ_950);

		/* have we downloaded firmware already? */
		if (mnh_sm_dev->firmware_downloaded)
			ret = mnh_sm_resume();
		else
			ret = mnh_sm_download();
		if (ret)
			break;

		ret = mnh_sm_hotplug_callback(MNH_HOTPLUG_IN);
		if (ret)
			mnh_sm_print_boot_trace(mnh_sm_dev->dev);

		break;
	case MNH_STATE_SUSPEND:
		ret = mnh_sm_set_state_locked(MNH_STATE_ACTIVE);
		if (!ret) {
			mnh_sm_hotplug_callback(MNH_HOTPLUG_OUT);

			/* toggle powered flag and clear completion */
			mnh_sm_dev->powered = false;
			reinit_completion(&mnh_sm_dev->powered_complete);

			ret = mnh_sm_suspend();
		}
		break;
	default:
		dev_err(mnh_sm_dev->dev,
			 "%s: invalid state %d\n", __func__, state);
		ret = -EINVAL;
	}

	if (ret) {
		/*
		 * NOTE (b/64372955): the minimum requirements for MIPI bypass
		 * are power and PCIe. A successful boot is only necessary for
		 * easelcomm communication. Return a special code to indicate
		 * that we reached the minimum requirements, but couldn't
		 * establish host communication. Also, disable a few clocks on
		 * Easel to reduce power.
		 */
#if ALLOW_PARTIAL_ACTIVE
		if ((state == MNH_STATE_ACTIVE) && (mnh_sm_dev->powered)) {
			dev_warn(mnh_sm_dev->dev,
				 "%s: failed to fully transition to state %d (%d), allow partial active\n",
				 __func__, state, ret);
			mnh_sm_enter_low_power_mode();
			ret = -EHOSTUNREACH;
		} else {
#endif
			dev_err(mnh_sm_dev->dev,
				 "%s: failed to transition to state %d (%d)\n",
				 __func__, state, ret);

			if (state == MNH_STATE_ACTIVE) {
				mnh_sm_dev->powered = false;
				reinit_completion(
					&mnh_sm_dev->powered_complete);
				mnh_sm_poweroff();
				disable_irq(mnh_sm_dev->ready_irq);
				mnh_sm_dev->state = MNH_STATE_OFF;
			}

			return ret;
#if ALLOW_PARTIAL_ACTIVE
		}
#endif
	}

	mnh_sm_dev->state = state;

	return ret;
}

/**
 * API to set the state of monette hill.
 * @param[in] Set the power states of mnh
 */
int mnh_sm_set_state(int state)
{
	int ret;

	if (!mnh_sm_dev)
		return -ENODEV;

	dev_info(mnh_sm_dev->dev, "%s: request state %d\n", __func__, state);

	if (mnh_freeze_state) {
		dev_info(mnh_sm_dev->dev,
			"%s: ignoring requested state %d because freeze_state is set\n",
			__func__, mnh_sm_dev->next_state);
		return -EBUSY;
	}

	mutex_lock(&mnh_sm_dev->lock);

	ret = mnh_sm_set_state_locked(state);

	mutex_unlock(&mnh_sm_dev->lock);

	if (ret)
		return ret;

	dev_info(mnh_sm_dev->dev, "%s: finished state %d\n", __func__,
		 mnh_sm_dev->state);

	return 0;
}
EXPORT_SYMBOL(mnh_sm_set_state);

int mnh_sm_is_present(void)
{
	return (mnh_sm_dev != NULL) ? 1 : 0;
}
EXPORT_SYMBOL(mnh_sm_is_present);

int mnh_sm_pwr_error_cb(void)
{
	dev_err(mnh_sm_dev->dev,
		"%s: observed mnh-pwr error, switching state to off\n",
		__func__);
	sysfs_notify(&mnh_sm_dev->dev->kobj, NULL, "error_event");
	return mnh_sm_set_state(MNH_STATE_OFF);
}
EXPORT_SYMBOL(mnh_sm_pwr_error_cb);

enum mnh_boot_mode mnh_sm_get_boot_mode(void)
{
	return mnh_boot_mode;
}
EXPORT_SYMBOL(mnh_sm_get_boot_mode);

static int mnh_sm_open(struct inode *inode, struct file *filp)
{
	int dev_ctr;

	if (!mnh_sm_dev)
		return -ENODEV;

	if (!mnh_sm_dev->initialized)
		return -EBUSY;

	filp->private_data = mnh_sm_dev; /* for other methods */

	dev_ctr = atomic_inc_return(&mnh_sm_dev->cdev_ctr);
	dev_dbg(mnh_sm_dev->dev, "%s: opening mnh_sm: mnh_sm_dev->cdev_ctr %d\n",
		__func__, dev_ctr);

	/* only stage fw transf. when the first handle to the cdev is opened */
	if (dev_ctr == 1) {
		if (mnh_sm_dev->ion &&
		    !mnh_sm_dev->ion[FW_PART_PRI]->is_fw_ready) {
			/* Request firmware and stage them to carveout buf. */
			dev_dbg(mnh_sm_dev->dev, "%s: staging firmware\n",
				__func__);
			mnh_ion_stage_firmware(mnh_sm_dev->ion[FW_PART_PRI]);
		}
	}
	return 0;
}

static int mnh_sm_close(struct inode *inode, struct file *filp)
{
	if (!mnh_sm_dev)
		return -ENODEV;

	if (!mnh_sm_dev->initialized)
		return -EBUSY;

	/* Only shut mnh down when there is no active handle to the cdev */
	if (atomic_dec_and_test(&mnh_sm_dev->cdev_ctr)) {
		mnh_sm_set_state(MNH_STATE_OFF);
		dev_dbg(mnh_sm_dev->dev, "%s: closing mnh_sm\n", __func__);
	}

	return 0;
}

static void mnh_sm_set_state_work(struct kthread_work *data)
{
	mnh_sm_dev->work_ret = mnh_sm_set_state(mnh_sm_dev->next_state);
	complete(&mnh_sm_dev->work_complete);
}

static int mnh_sm_wait_for_power(void)
{
	unsigned long timeout;

	if (mnh_sm_dev->powered)
		return 0;

	timeout = wait_for_completion_timeout(&mnh_sm_dev->powered_complete,
		POWERED_COMPLETE_TIMEOUT);
	if (!timeout) {
		dev_err(mnh_sm_dev->dev,
			"%s: timeout waiting for mnh to power on\n",
			__func__);
		return -ETIMEDOUT;
	}

	return 0;
}

static int mnh_sm_wait_for_state(int state)
{
	unsigned long timeout;

	if (mnh_sm_dev->state == state)
		return 0;

	if (mnh_sm_dev->next_state != state)
		return -EINVAL;

	timeout = wait_for_completion_timeout(&mnh_sm_dev->work_complete,
		STATE_CHANGE_COMPLETE_TIMEOUT);
	if (!timeout) {
		dev_err(mnh_sm_dev->dev,
			"%s: timeout waiting for state transition\n",
			__func__);
		return -ETIMEDOUT;
	}

	return mnh_sm_dev->work_ret;
}

static long mnh_sm_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	int err = 0;
	int fd = -1; /* handle for ion buffer sharing */
	struct mnh_mipi_config mipi_config;
	struct mnh_update_configs update_configs;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
				  _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg,
				  _IOC_SIZE(cmd));
	if (err) {
		dev_err(mnh_sm_dev->dev, "%s: access error!\n", __func__);
		return -EFAULT;
	}

	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok(  )
	 */

	if (_IOC_TYPE(cmd) != MNH_SM_IOC_MAGIC) {
		dev_err(mnh_sm_dev->dev,
			"%s: invalid ioctl type %c (0x%08x)\n",
			__func__, _IOC_TYPE(cmd), cmd);
		return -ENOTTY;
	}

	switch (cmd) {
	case MNH_SM_IOC_GET_STATE:
		err = copy_to_user((void __user *)arg, &mnh_sm_dev->state,
				   sizeof(mnh_sm_dev->state));
		if (err) {
			dev_err(mnh_sm_dev->dev,
				"%s: failed to copy to userspace (%d)\n",
				__func__, err);
			return err;
		}
		break;
	case MNH_SM_IOC_SET_STATE:
		mnh_sm_dev->next_state = (int)arg;
		reinit_completion(&mnh_sm_dev->work_complete);
		queue_kthread_work(&mnh_sm_dev->worker,
				   &mnh_sm_dev->set_state_work);
		break;
	case MNH_SM_IOC_WAIT_FOR_POWER:
		err = mnh_sm_wait_for_power();
		break;
	case MNH_SM_IOC_WAIT_FOR_STATE:
		err = mnh_sm_wait_for_state((int)arg);
		break;
	case MNH_SM_IOC_CONFIG_MIPI:
	case MNH_SM_IOC_STOP_MIPI:
		err = copy_from_user(&mipi_config, (void __user *)arg,
				     sizeof(struct mnh_mipi_config));
		if (err) {
			dev_err(mnh_sm_dev->dev,
				"%s: failed to copy mipi config from userspace (%d)\n",
				__func__, err);
			return err;
		}
		if (!mnh_sm_dev->powered)
			return -EIO;
		if (cmd == MNH_SM_IOC_CONFIG_MIPI)
			mnh_mipi_config(mnh_sm_dev->dev, mipi_config);
		else
			mnh_mipi_stop(mnh_sm_dev->dev, mipi_config);
		break;
	case MNH_SM_IOC_GET_UPDATE_BUF:
		mnh_sm_dev->ion[FW_PART_SEC]->is_fw_ready = false;
		if (mnh_sm_dev->ion[FW_PART_SEC]) {
			if (mnh_ion_create_buffer(mnh_sm_dev->ion[FW_PART_SEC],
						  MNH_ION_BUFFER_SIZE,
						  ION_SYSTEM_HEAP_ID))
				dev_warn(mnh_sm_dev->dev,
					 "%s: cannot claim ION buffer\n",
					 __func__);
		}
		fd = ion_share_dma_buf_fd(mnh_sm_dev->ion[FW_PART_SEC]->client,
					  mnh_sm_dev->ion[FW_PART_SEC]->handle);
		err = copy_to_user((void __user *)arg, &fd, sizeof(fd));
		if (err) {
			mnh_ion_destroy_buffer(mnh_sm_dev->ion[FW_PART_SEC]);
			dev_err(mnh_sm_dev->dev,
				"%s: failed to copy to userspace (%d)\n",
				__func__, err);
			return err;
		}
		break;
	case MNH_SM_IOC_POST_UPDATE_BUF:
		err = copy_from_user(&update_configs, (void __user *)arg,
				     sizeof(update_configs));
		if (err) {
			dev_err(mnh_sm_dev->dev,
				"%s: failed to copy from userspace (%d)\n",
				__func__, err);
			return err;
		}

		err = mnh_validate_update_buf(update_configs);
		if (err) {
			mnh_ion_destroy_buffer(mnh_sm_dev->ion[FW_PART_SEC]);

			dev_err(mnh_sm_dev->dev,
				"%s: failed to validate slots (%d)\n",
				__func__, err);
			return err;
		}
		break;
	default:
		dev_err(mnh_sm_dev->dev,
			"%s: unknown ioctl %c, dir=%d, #%d (0x%08x)\n",
			__func__, _IOC_TYPE(cmd), _IOC_DIR(cmd), _IOC_NR(cmd),
			cmd);
		break;
	}

	return err;
}

static irqreturn_t mnh_sm_ready_irq_handler(int irq, void *cookie)
{
	if (gpiod_get_value(mnh_sm_dev->ready_gpio)) {
		dev_info(mnh_sm_dev->dev, "%s: mnh device is ready to boot\n",
			 __func__);
		reinit_completion(&mnh_sm_dev->suspend_complete);
	} else {
		dev_dbg(mnh_sm_dev->dev, "%s: mnh device is ready to suspend\n",
			__func__);
		complete(&mnh_sm_dev->suspend_complete);
	}

	return IRQ_HANDLED;
}

static long mnh_sm_compat_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	int ret;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(MNH_SM_IOC_STOP_MIPI):
	case _IOC_NR(MNH_SM_IOC_CONFIG_MIPI):
	case _IOC_NR(MNH_SM_IOC_GET_STATE):
	case _IOC_NR(MNH_SM_IOC_POST_UPDATE_BUF):
	case _IOC_NR(MNH_SM_IOC_GET_UPDATE_BUF):
		cmd &= ~(_IOC_SIZEMASK << _IOC_SIZESHIFT);
		cmd |= sizeof(void *) << _IOC_SIZESHIFT;
		ret = mnh_sm_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
		break;
	default:
		ret = mnh_sm_ioctl(file, cmd, arg);
		break;
	}

	return ret;
}

static const struct file_operations mnh_sm_fops = {
	.open = mnh_sm_open,
	.unlocked_ioctl = mnh_sm_ioctl,
	.compat_ioctl = mnh_sm_compat_ioctl,
	.release = mnh_sm_close
};

static struct miscdevice mnh_sm_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &mnh_sm_fops,
	.groups = mnh_sm_groups,
};

static int mnh_sm_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct sched_param param = { .sched_priority = 5 };

	int error = 0;
	int i;

	pr_debug("%s: MNH SM initializing...\n", __func__);

	/* create char driver */
	error = misc_register(&mnh_sm_miscdevice);
	if (error) {
		pr_err("%s: failed to create char device (%d)\n",
		       __func__, error);
		return error;
	}
	dev = mnh_sm_miscdevice.this_device;

	/* allocate device memory */
	mnh_sm_dev = devm_kzalloc(dev, sizeof(struct mnh_sm_device),
				  GFP_KERNEL);
	if (mnh_sm_dev == NULL) {
		error = -ENOMEM;
		goto fail_probe_0;
	}

	/* add device data to platform device */
	mnh_sm_dev->pdev = pdev;
	mnh_sm_dev->misc_dev = &mnh_sm_miscdevice;
	mnh_sm_dev->dev = dev;

	/* initialize kthread work queue */
	init_kthread_worker(&mnh_sm_dev->worker);
	init_kthread_work(&mnh_sm_dev->set_state_work, mnh_sm_set_state_work);
	mnh_sm_dev->thread = kthread_run(kthread_worker_fn,
					 &mnh_sm_dev->worker,
					 "set_state_work");
	if (IS_ERR(mnh_sm_dev->thread)) {
		dev_err(dev, "%s: unable to start work thread\n", __func__);
		mnh_sm_dev->thread = NULL;
		error = -ENOMEM;
		goto fail_probe_1;
	}
	sched_setscheduler(mnh_sm_dev->thread, SCHED_FIFO, &param);

	/* initialize driver structures */
	mutex_init(&mnh_sm_dev->lock);
	init_completion(&mnh_sm_dev->powered_complete);
	init_completion(&mnh_sm_dev->work_complete);
	init_completion(&mnh_sm_dev->suspend_complete);

	/* Allocate primary ION buffer (from carveout region) */
	mnh_sm_dev->ion[FW_PART_PRI] = devm_kzalloc(dev,
						    sizeof(struct mnh_ion),
						    GFP_KERNEL);
	if (mnh_sm_dev->ion[FW_PART_PRI]) {
		mnh_sm_dev->ion[FW_PART_PRI]->device = dev;
		if (mnh_ion_create_buffer(mnh_sm_dev->ion[FW_PART_PRI],
					  MNH_ION_BUFFER_SIZE,
					  ION_GOOGLE_HEAP_ID))
			dev_warn(dev, "%s: cannot claim ION buffer\n",
				 __func__);
	}

	/* Initialize ION structure of firmware update buffer */
	mnh_sm_dev->ion[FW_PART_SEC] = devm_kzalloc(dev,
						    sizeof(struct mnh_ion),
						    GFP_KERNEL);
	if (mnh_sm_dev->ion[FW_PART_SEC])
		mnh_sm_dev->ion[FW_PART_SEC]->device = dev;

	/* get boot mode gpio */
	mnh_sm_dev->boot_mode_gpio = devm_gpiod_get(&pdev->dev, "boot-mode",
						    GPIOD_OUT_LOW);
	if (IS_ERR(mnh_sm_dev->boot_mode_gpio)) {
		dev_err(dev, "%s: could not get boot_mode gpio (%ld)\n",
			__func__, PTR_ERR(mnh_sm_dev->boot_mode_gpio));
		error = PTR_ERR(mnh_sm_dev->boot_mode_gpio);
		goto fail_probe_2;
	}

	/* get ready gpio */
	mnh_sm_dev->ready_gpio = devm_gpiod_get(&pdev->dev, "ready", GPIOD_IN);
	if (IS_ERR(mnh_sm_dev->ready_gpio)) {
		dev_err(dev, "%s: could not get ready gpio (%ld)\n",
			__func__, PTR_ERR(mnh_sm_dev->ready_gpio));
		error = PTR_ERR(mnh_sm_dev->ready_gpio);
		goto fail_probe_2;
	}
	mnh_sm_dev->ready_irq = gpiod_to_irq(mnh_sm_dev->ready_gpio);

	/* request ready gpio irq */
	irq_set_status_flags(mnh_sm_dev->ready_irq, IRQ_DISABLE_UNLAZY);
	error = devm_request_threaded_irq(dev, mnh_sm_dev->ready_irq, NULL,
					  mnh_sm_ready_irq_handler,
					  IRQF_TRIGGER_FALLING |
					  IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					  "mnh-ready", mnh_sm_dev);
	if (error) {
		dev_err(dev, "%s: could not get ready irq (%d)\n", __func__,
			error);
		goto fail_probe_2;
	}
	disable_irq(mnh_sm_dev->ready_irq);

	/* request ddr pad isolation pin */
	mnh_sm_dev->ddr_pad_iso_n_pin = devm_gpiod_get(&pdev->dev,
						       "ddr-pad-iso-n",
						       GPIOD_OUT_HIGH);
	if (IS_ERR(mnh_sm_dev->ddr_pad_iso_n_pin)) {
		dev_err(dev, "%s: could not get ddr_pad_iso_n gpio (%ld)\n",
			__func__, PTR_ERR(mnh_sm_dev->ddr_pad_iso_n_pin));
		if (PTR_ERR(mnh_sm_dev->ddr_pad_iso_n_pin) == -EPROBE_DEFER)
			error = -ENODEV;
		else
			error = PTR_ERR(mnh_sm_dev->ddr_pad_iso_n_pin);
		goto fail_probe_2;
	}

	/* initialize mnh-pwr and get resources there */
	enable_irq(mnh_sm_dev->ready_irq);
	error = mnh_pwr_init(pdev, dev);
	disable_irq(mnh_sm_dev->ready_irq);
	if (error) {
		dev_err(dev, "failed to initialize mnh-pwr (%d)\n", error);
		goto fail_probe_2;
	}

	/* initialize mnh-clk driver */
	mnh_clk_init(dev, HWIO_SCU_BASE_ADDR);

	mnh_sm_dev->initialized = true;
	dev_info(dev, "MNH SM initialized successfully\n");

	return 0;

fail_probe_2:
	for (i = 0; i < FW_PART_MAX; i++) {
		mnh_ion_destroy_buffer(mnh_sm_dev->ion[i]);
		devm_kfree(dev, mnh_sm_dev->ion[i]);
		mnh_sm_dev->ion[i] = NULL;
	}
fail_probe_1:
	devm_kfree(dev, mnh_sm_dev);
	mnh_sm_dev = NULL;
fail_probe_0:
	misc_deregister(&mnh_sm_miscdevice);

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

/* Make sure mnh_sm driver is initialized after bcm15602-gpio */
static int __init mnh_sm_init(void)
{
	return platform_driver_register(&mnh_sm);
}
late_initcall(mnh_sm_init);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("MNH State Manager HOST DRIVER");
MODULE_LICENSE("GPL v2");
