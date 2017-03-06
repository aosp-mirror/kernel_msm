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

