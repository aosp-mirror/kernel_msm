/*
 * MNH Firmware on ION buffer
 *
 * Copyright 2017 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/msm_ion.h>
#include "hw-mnh-regs.h"
#include "mnh-sm.h"

#define MSM_ION_EASEL_MEM_HEAP_NAME	"easel_mem"

/**
 * mnh_ion_fw_copy - Copy data to ION buffer and flush cache
 * Return 0		on success
 *        -ENOMEM	if exceeding totoal buffer size
 * mnh_ion_fw_copy must be called in slot incrementing order:
 * 1. MNH_FW_SLOT_SBL, 2. MNH_FW_SLOT_KERNEL,
 * 3. MNH_FW_SLOT_DTB, 4. MNH_FW_SLOT_RAMDISK
 */
static int mnh_ion_fw_copy(struct mnh_ion *ion, int slot,
				uint64_t ep_addr, size_t size,
				const u8 *data)
{
	dma_addr_t ion_addr;	/* Base bus address of ION buffer */
	unsigned long offset;	/* offset inside ION buffer */
	void *buf;

	ion_addr = ion->sg->sgl->dma_address;
	offset = (slot == 0) ? 0 : ion->fw_array[slot-1].ap_offs +
		ion->fw_array[slot-1].size;

	buf = ion->vaddr + offset;

	if (offset + size >= ion->total_size) {
		dev_err(ion->device,
			"%s: slot=%d offset=0x%lx + size=0x%zx >= total=0x%zx\n",
			__func__, slot, offset, size, ion->total_size);
		return -ENOMEM;
	}

	ion->fw_array[slot].ep_addr = ep_addr;
	ion->fw_array[slot].size = size;
	ion->fw_array[slot].ap_addr = ion_addr + offset;
	ion->fw_array[slot].ap_offs = offset;

	memcpy(buf, data, size);

	msm_ion_do_cache_op(ion->client, ion->handle, buf, size,
			    ION_IOC_CLEAN_CACHES);

	return 0;
}

/**
 * mnh_ion_fw_copy_by_name - Request firmware from disk and copy to ION
 * Returns 0		on success
 *         negative	on error
 */
static int mnh_ion_fw_copy_by_name(struct mnh_ion *ion, int slot,
				   uint64_t ep_addr, const char *name)
{
	int err = 0;
	const struct firmware *fw_img;

	dev_dbg(ion->device, "%s: request %s for slot %d\n",
		__func__, name, slot);

	err = request_firmware(&fw_img, name, ion->device);
	if (err) {
		dev_err(ion->device, "%s: request %s failed (%d)\n",
			__func__, name, err);
		return err;
	}
	err = mnh_ion_fw_copy(ion, slot, ep_addr, fw_img->size, fw_img->data);
	if (err) {
		dev_err(ion->device, "%s: copy %s to ION failed (%d)\n",
			__func__, name, err);
		/* Do not return yet */
	}
	release_firmware(fw_img);

	return err;
}

/**
 * mnh_ion_fw_copy_try_ion_sec - Try update from secondary ION buffer
 * Returns 0		on success
 *         negative	no update at this slot, or failed to copy
 */
static int mnh_ion_fw_copy_try_ion_sec(struct mnh_ion *ion,
				       struct mnh_ion *ion_sec, int slot,
				       uint64_t ep_addr)
{
	int err = 0;

	if (!ion_sec || !ion)
		return -EINVAL;

	if (ion_sec->fw_array[slot].size <= 0)
		return -EINVAL;

	dev_dbg(ion->device, "%s: Using update buffer for slot %d\n",
		__func__, slot);

	err = mnh_ion_fw_copy(ion, slot, ep_addr,
			      ion_sec->fw_array[slot].size,
			      ion_sec->vaddr +
			      ion_sec->fw_array[slot].ap_offs);

	return err;
}

/**
 * mnh_ion_fw_update_request - For each slot, try to update from secondary
 *     ION buffer first.  If failed, try update that slot from disk instead.
 * Returns 0		on success
 *         negative	on error
 */
static int mnh_ion_fw_update_request(struct mnh_ion *ion,
				     struct mnh_ion *ion_sec)
{
	int err;

	err = mnh_ion_fw_copy_try_ion_sec(ion, ion_sec, MNH_FW_SLOT_SBL,
					  HW_MNH_SBL_DOWNLOAD);
	if (err) {
		err = mnh_ion_fw_copy_by_name(ion, MNH_FW_SLOT_SBL,
					      HW_MNH_SBL_DOWNLOAD,
					      "easel/fip.bin");
		if (err)
			return err;
	}

	err = mnh_ion_fw_copy_try_ion_sec(ion, ion_sec, MNH_FW_SLOT_KERNEL,
					  HW_MNH_KERNEL_DOWNLOAD);
	if (err) {
		err = mnh_ion_fw_copy_by_name(ion, MNH_FW_SLOT_KERNEL,
					      HW_MNH_KERNEL_DOWNLOAD,
					      "easel/Image");
		if (err)
			return err;
	}

	err = mnh_ion_fw_copy_try_ion_sec(ion, ion_sec, MNH_FW_SLOT_DTB,
					  HW_MNH_DT_DOWNLOAD);
	if (err) {
		err = mnh_ion_fw_copy_by_name(ion, MNH_FW_SLOT_DTB,
					      HW_MNH_DT_DOWNLOAD,
					      "easel/mnh.dtb");
		if (err)
			return err;
	}

	err = mnh_ion_fw_copy_try_ion_sec(ion, ion_sec, MNH_FW_SLOT_RAMDISK,
					  HW_MNH_RAMDISK_DOWNLOAD);
	if (err) {
		err = mnh_ion_fw_copy_by_name(ion, MNH_FW_SLOT_RAMDISK,
					      HW_MNH_RAMDISK_DOWNLOAD,
					      "easel/ramdisk.img");
		if (err)
			return err;
	}

	return 0;
}

/**
 * mnh_ion_fw_request - For each slot, update firmware from disk
 * Returns 0		on success
 *         negative	on error
 */
static int mnh_ion_fw_request(struct mnh_ion *ion)
{
	int err;

	err = mnh_ion_fw_copy_by_name(ion, MNH_FW_SLOT_SBL,
				      HW_MNH_SBL_DOWNLOAD,
				      "easel/fip.bin");
	if (err)
		return err;

	err = mnh_ion_fw_copy_by_name(ion, MNH_FW_SLOT_KERNEL,
				      HW_MNH_KERNEL_DOWNLOAD,
				      "easel/Image");
	if (err)
		return err;

	err = mnh_ion_fw_copy_by_name(ion, MNH_FW_SLOT_DTB,
				      HW_MNH_DT_DOWNLOAD,
				      "easel/mnh.dtb");
	if (err)
		return err;

	err = mnh_ion_fw_copy_by_name(ion, MNH_FW_SLOT_RAMDISK,
				      HW_MNH_RAMDISK_DOWNLOAD,
				      "easel/ramdisk.img");
	if (err)
		return err;

	return 0;
}

int mnh_ion_stage_firmware(struct mnh_ion *ion)
{
	int ret;

	if (!ion || !ion->client)
		return -ENODEV;

	ion->is_fw_ready = false;
	ret = mnh_ion_fw_request(ion);
	if (ret)
		return ret;

	ion->is_fw_ready = true;

	return 0;
}
EXPORT_SYMBOL(mnh_ion_stage_firmware);

int mnh_ion_stage_firmware_update(struct mnh_ion *ion, struct mnh_ion *ion_sec)
{
	int err;

	dev_dbg(ion->device, "%s: staging update\n", __func__);
	if (!ion || !ion_sec || !ion->client ||
	    !ion_sec->client || !ion_sec->is_fw_ready)
		return -ENODEV;

	ion->is_fw_ready = false;
	err = mnh_ion_fw_update_request(ion, ion_sec);
	if (err)
		return err;

	ion->is_fw_ready = true;

	return 0;
}
EXPORT_SYMBOL(mnh_ion_stage_firmware_update);

long mnh_ion_create_buffer(struct mnh_ion *ion, size_t size,
			   enum ion_heap_ids heap_id)
{
	long err = 0;
	struct device *dev = ion->device;

	if (!ion)
		return -EINVAL;

	dev_dbg(dev, "%s: Claiming %zuMB ION buffer...\n",
		 __func__, size >> 20);

	/* Create ION client */
	ion->client = msm_ion_client_create(MSM_ION_EASEL_MEM_HEAP_NAME);
	if (IS_ERR(ion->client)) {
		err = PTR_ERR(ion->client);
		dev_err(dev, "%s: Ion cannot create client \"%s\" (%ld)\n",
			__func__, MSM_ION_EASEL_MEM_HEAP_NAME, err);
		return err;
	}

	/* Allocate an ION buffer */
	ion->handle = ion_alloc(ion->client, size, PAGE_SIZE /* align */,
				    ION_HEAP(heap_id),
				    ION_FLAG_CACHED);
	if (IS_ERR(ion->handle)) {
		err = PTR_ERR(ion->handle);
		dev_err(dev, "%s: Ion cannot allocate buffer (%ld)\n",
			__func__, err);
		goto fail_ion_alloc;
	}

	/* Map ION buffer to kernel space */
	ion->vaddr = ion_map_kernel(ion->client, ion->handle);
	if (IS_ERR(ion->vaddr)) {
		err = PTR_ERR(ion->vaddr);
		dev_err(dev, "%s: Cannot map ion buffer to kernel (%ld)\n",
			__func__, err);
		goto fail_ion_map_kernel;
	}

	/* Map ION buffer to IOMMU and get sg_table */
	ion->sg = ion_sg_table(ion->client, ion->handle);
	if (IS_ERR(ion->sg)) {
		err = PTR_ERR(ion->sg);
		dev_err(dev, "%s: Cannot get ion sg_table (%ld)\n",
			__func__, err);
		goto fail_ion_sg_table;
	}

	dev_dbg(dev, "sg->nents=%d\n", ion->sg->nents);
	dev_dbg(dev, "sg->orig_nents=%d\n", ion->sg->orig_nents);
	dev_dbg(dev, "sgl->dma_address=%pa\n", &(ion->sg->sgl->dma_address));
	dev_dbg(dev, "sgl->length=0x%x\n", ion->sg->sgl->length);

	ion->total_size = size;

	return 0;

fail_ion_sg_table:
	ion_unmap_kernel(ion->client, ion->handle);
	ion->vaddr = NULL;
fail_ion_map_kernel:
	ion_free(ion->client, ion->handle);
	ion->handle = NULL;
fail_ion_alloc:
	ion_client_destroy(ion->client);
	ion->client = NULL;
	return err;
}
EXPORT_SYMBOL(mnh_ion_create_buffer);

void mnh_ion_destroy_buffer(struct mnh_ion *ion)
{
	if (!ion || !(ion->client))
		return;

	dev_dbg(ion->device, "%s: Freeing %zuMB ION buffer...\n",
		 __func__, ion->total_size >> 20);

	if (!IS_ERR_OR_NULL(ion->handle)) {
		ion_unmap_kernel(ion->client, ion->handle);
		ion->vaddr = NULL;
		ion_free(ion->client, ion->handle);
		ion->handle = NULL;
	}
	ion_client_destroy(ion->client);
	ion->client = NULL;

	ion->is_fw_ready = false;
}
EXPORT_SYMBOL(mnh_ion_destroy_buffer);
