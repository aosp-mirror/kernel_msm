/*
 * Driver interface for the AB State Manager
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __UAPI_AB_SM_H__
#define __UAPI_AB_SM_H__

#include <linux/ioctl.h>


#define AB_SM_IOCTL_MAGIC	'a'

/* First call after open will immediately return the current
 * state. Subsequent calls will block until next state change,
 * then return the new state.
 *
 * Parameter int *:
 *	Pass pointer to integer to be filled in with new state value
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_ASYNC_NOTIFY	_IOR(AB_SM_IOCTL_MAGIC, 0, int *)

/*
 * Parameter int:
 *	Pass new state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_STATE		_IOW(AB_SM_IOCTL_MAGIC, 1, int)

/*
 * Parameter int *:
 *	Pass pointer to integer to be filled in with state value
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_GET_STATE		_IOR(AB_SM_IOCTL_MAGIC, 2, int *)

/* On success will return 0, otherwise will return error < 0. */
#define AB_SM_ENTER_EL2		_IO(AB_SM_IOCTL_MAGIC, 3)

/* On success will return 0, otherwise will return error < 0. */
#define AB_SM_EXIT_EL2		_IO(AB_SM_IOCTL_MAGIC, 4)

#define AB_CHIP_ID_UNKNOWN	-1
#define AB_CHIP_ID_A0		0
#define AB_CHIP_ID_B0		1


#endif /* __UAPI_AB_SM_H__ */
