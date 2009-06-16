/* linux/include/asm-arm/arch-msm/hsusb.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_HSUSB_H
#define __ASM_ARCH_MSM_HSUSB_H

#include <linux/types.h>

/* platform device data for msm_hsusb driver */

struct msm_hsusb_platform_data {
	/* hard reset the ULPI PHY */
	void (*phy_reset)(void);

	/* (de)assert the reset to the usb core */
	void (*hw_reset)(bool enable);

	/* for notification when USB is connected or disconnected */
	void (*usb_connected)(int);

	/* val, reg pairs terminated by -1 */
	int *phy_init_seq;
};

#endif
