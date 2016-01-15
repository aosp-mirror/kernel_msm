/* include/linux/touch_fw_update.h
 *
 * Copyright (c)2014 HTC, Inc.
 *
 * Driver Version: 1.0.0
 * Release Date: Aug 28, 2014
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

#ifndef __TOUCH_FWU_H
#define __TOUCH_FWU_H

struct touch_fwu_notifier {
	int (*fwupdate)(struct firmware *fw);
	u32 flash_timeout;
	char fw_vendor[20];
	char fw_ver[20];
};
int register_fw_update(struct touch_fwu_notifier *notifier);
void unregister_fw_update(void);
void touch_fw_update_progress(int percentage);

#endif //__TOUCH_FWU_H
