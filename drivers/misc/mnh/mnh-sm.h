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

#include "mnh-pcie.h"
#include "mnh-sm-config.h"

enum mnh_sm_state {
	MNH_HW_INIT, /* powered on, unconfigured */
	MNH_HW_CONFIG, /* powered on, mipi and ddr configured */
	MNH_HW_OFF, /* powered off */
	MNH_HW_ACTIVE, /* powered on and booted */
	MNH_HW_SUSPEND_SELF_REFRESH, /* suspended, ddr in self-refresh */
	MNH_HW_SUSPEND_HIBERNATE, /* suspended, kernel image in AP DRAM */
	MNH_HW_UNKNOWN /* on init */
};

/** API to register hotplug callback to receive MNH up/down notifications
 * @param[in] hotplug_cb  handler for hotplug in/out events
 * @return 0
 */
int mnh_sm_reg_hotplug_callback(hotplug_cb_t hotplug_cb);

/**
 * API to initialize Power and clocks to MNH, MIPI, DDR, DDR training,
 * and PCIE.
 * @param[in] Structure argument to configure each boot component.
 *            This structure will be populated within the kernel module.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_poweron(struct mnh_sm_configuration mnh_sm_boot_args);


/**
 * API to power monette hill.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_poweroff(struct mnh_sm_configuration mnh_sm_boot_args);

/**
 * API to initialize MIPI, DDR, DDR training,
 * and PCIE.
 * @param[in] Structure argument to configure each boot component.
 *            This structure will be populated within the kernel module.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_config(struct mnh_sm_configuration mnh_sm_boot_args);


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
int mnh_sm_get_state(void);

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
int mnh_sm_set_state(int state);

/**
 * API to download the binary images(SBL, UBoot, Kernel, Ramdisk) for mnh.
 * The location of the binaries will be located in the AP file system.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_download(void);

/**
 * API to put MNH in suspend state.  In suspend mode the DDR will be isolated
 * and put in self refresh while the CPU is powered down.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_suspend(struct mnh_sm_configuration mnh_sm_boot_args);

/**
 * API to put MNH into active state.
 * The resume call flow should be similar to normal bootflow except for DDR
 * initializations. Since the binaries are already saved on the DDR while MNH
 * is in suspend, ESM will not need to download the binaries again during
 * resume.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_sm_resume(struct mnh_sm_configuration mnh_sm_boot_args);

int mnh_sm_is_present(void);

void mnh_sm_get_default_config(struct mnh_sm_configuration *mnh_sm_boot_args);

#endif /* __MNH_SM_HOST */

