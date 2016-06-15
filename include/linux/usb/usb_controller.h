/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __USB_CONTROLLER_H__
#define __USB_CONTROLLER_H__

struct usb_controller {
	int (*notify_attached_source)(struct usb_controller *uc, int value);
	int (*pd_vbus_ctrl)(int on, bool isPRSwap);
	bool (*vbus_boost_enabled)(void);
};

extern int usb_controller_register(struct device* parent, struct usb_controller *uc);

#endif

