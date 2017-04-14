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

#ifndef __MNH_SM_HOST
#define __MNH_SM_HOST

#include <linux/msm_ion.h>
#include <uapi/linux/mnh-sm.h>
#include "mnh-pcie.h"

/* Firmware download address */
#define HW_MNH_DRAM_BASE		0x40000000
#define HW_MNH_SBL_DOWNLOAD		0x40000000
#define HW_MNH_SBL_DOWNLOAD_EXE		0x00101000
#define HW_MNH_KERNEL_DOWNLOAD		0x40080000
#define HW_MNH_DT_DOWNLOAD		0x40880000
#define HW_MNH_RAMDISK_DOWNLOAD		0x40890000
/* Firmware download related definitions */
#define IMG_DOWNLOAD_MAX_SIZE		(4000 * 1024)
#define FIP_IMG_SBL_SIZE_OFFSET		0x28
#define FIP_IMG_SBL_ADDR_OFFSET		0x20

enum mnh_boot_mode {
	MNH_BOOT_MODE_PCIE,
	MNH_BOOT_MODE_SPI,
};

struct mnh_ion_fw_conf {
	dma_addr_t ap_addr;	/* AP side addr (dma) */
	unsigned long ap_offs;  /* Slot's offset in the ion buffer */
	uint64_t ep_addr;	/* EP side addr (phys) */
	size_t size;		/* size of firmware */
};

struct mnh_ion {
	struct ion_client  *client;
	struct ion_handle *handle;
	struct sg_table *sg;
	void *vaddr;
	size_t total_size;	/* size in bytes */
	bool is_fw_ready;	/* whether fw loaded in ION */
	struct mnh_ion_fw_conf fw_array[MAX_NR_MNH_FW_SLOTS];
	struct device *device;	/* Linux basic device associated with parent */
};

/** API to register hotplug callback to receive MNH up/down notifications
 * @param[in] hotplug_cb  handler for hotplug in/out events
 * @return 0
 */
int mnh_sm_reg_hotplug_callback(hotplug_cb_t hotplug_cb);

/**
 * API to obtain the state of monette hill.
 * @return the power states of mnh.
 */
int mnh_sm_get_state(void);

/**
 * API to set the state of monette hill.
 * @param[in] Set the power states of mnh
 */
int mnh_sm_set_state(int state);

int mnh_sm_is_present(void);

/**
 * API to claim ION buffer for mnh.
 * @param[in] dev       Linux device structure assocatiated with mnh_sm
 * @param[in] ion       Pre-allocated mnh_ion struct
 * @param[in] size      Size of ION buffer to claim (in bytes)
 * @param[in] heap_id   Heap ID to be used for the allocation
 *
 * @return 0            on success
 * @return -negative    on failure
 */
long mnh_ion_create_buffer(struct mnh_ion *ion, size_t size,
			   enum ion_heap_ids heap_id);

/**
 * API to release previously claimed ION buffer.
 * @param[in] ion      Pre-allocated mnh_ion struct
 *                     Caller is responsible for freeing ion.
 */
void mnh_ion_destroy_buffer(struct mnh_ion *ion);

/**
 * API to request MNH firmware and stage them to ION buffer.
 * @param[in] ion      Pre-allocated mnh_ion struct
 *                     Caller is responsible for freeing ion.
 *
 * @return 0            on success
 * @return -negative    on failure
 */
int mnh_ion_stage_firmware(struct mnh_ion *ion);

/**
 * API to stage Play Store firmware update to ION buffer.
 * @param[in] ion      Pre-allocated mnh_ion struct
 *                     Caller is responsible for freeing ion.
 * @param[in] ion_sec  Temporary allocated buffer holding
 *                     firmware updated from Play Store channel
 *
 * @return 0            on success
 * @return -negative    on failure
 */
int mnh_ion_stage_firmware_update(struct mnh_ion *ion,
				  struct mnh_ion *ion_sec);

/*
 * Callback from mnh-pwr when there is a failure event.
 *
 * @return 0           on success
 * @return -errno      on failure
 */
int mnh_sm_pwr_error_cb(void);

enum mnh_boot_mode mnh_sm_get_boot_mode(void);

#endif /* __MNH_SM_HOST */

