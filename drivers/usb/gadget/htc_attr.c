/*
 * Copyright (C) 2015 HTC, Inc.
 * Author: Dyson Lee <Dyson@intel.com>
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

static bool enable_htc_radio_debug_func = false;

const char * add_usb_radio_debug_function(const char *buff) {
	pr_info("%s : switch to radio debug function\n", __func__);
	if (!strcmp(buff, "adb") || !strcmp(buff, "mtp,adb"))
		return "diag,adb,serial,rmnet,mass_storage";
	else if(!strcmp(buff, "rndis,adb"))
		return "rndis,diag,adb";
	return buff;
}

/* Change the PID for radio flag 8 20000 */
void check_usb_vid_pid(struct usb_composite_dev *cdev) {
	switch(cdev->desc.idProduct) {
		case 0x4ed2:
		case 0x4ed7:
		case 0x4ee2:
		case 0x4ee7:
			cdev->desc.idVendor = 0x05c6;
			cdev->desc.idProduct = 0x9025;
			break;
		case 0x4ed4:
		case 0x4ee4:
			cdev->desc.idVendor = 0x05c6;
			cdev->desc.idProduct = 0x902d;
			break;
		default:
			break;
	}
	return;
}

static int __init htc_radio_debug_func_set(char *str)
{
	enable_htc_radio_debug_func = true;
	pr_info("set enable_htc_radio_debug_function\n");
	return 0;
}
__setup("htc_radio_debug_func=1", htc_radio_debug_func_set);
