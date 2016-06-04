
/* Copyright (c) 2015-2016 The Linux Foundation. All rights reserved.
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

enum{
	STATUS_NORMAL           = 0,
	STATUS_FULL_CHECK	= 1,
	STATUS_FULL_WARM	= 2,
	STATUS_FULL             = 3,
	STATUS_OVER_TEMP        = 4,
	STATUS_BATT_OV		= 5,
};

extern int cei_smb231_flag;
extern int smb23x_disable_input_current(bool disable);
extern int usb_insertion(void);
extern int usb_pre_removal(void);
extern int usb_removal(void);
