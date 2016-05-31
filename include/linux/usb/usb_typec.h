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

#ifndef __USB_TYPEC_H__
#define __USB_TYPEC_H__

struct usb_typec_ctrl {
	u8 sink_current;
};

extern int usb_typec_ctrl_register(struct device* parent, struct usb_typec_ctrl *utc);

#endif

