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

enum mnh_sm_state {
	MNH_STATE_OFF, /* powered off */
	MNH_STATE_INIT, /* powered on, unconfigured */
	MNH_STATE_CONFIG_MIPI, /* powered on, mipi configured */
	MNH_STATE_CONFIG_DDR, /* powered on, ddr configured */
	MNH_STATE_ACTIVE, /* powered on and booted */
	MNH_STATE_SUSPEND_SELF_REFRESH, /* suspended, ddr in self-refresh */
	MNH_STATE_SUSPEND_HIBERNATE, /* suspended, kernel image in AP DRAM */
	MNH_STATE_MAX,
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

#endif /* __MNH_SM_HOST */

