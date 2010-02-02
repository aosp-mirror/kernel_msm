/* include/linux/tpa2018d1.h - tpa2018d1 speaker amplifier driver
 *
 * Copyright (C) 2009 HTC Corporation.
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

#ifndef _LINUX_TPA2018D1_H
#define _LINUX_TPA2018D1_H

#include <linux/ioctl.h>

enum tpa2018d1_mode {
	TPA2018_MODE_OFF,
	TPA2018_MODE_PLAYBACK,
	TPA2018_MODE_RINGTONE,
	TPA2018_MODE_VOICE_CALL,
	TPA2018_NUM_MODES,
};

#define TPA2018_IOCTL_MAGIC	'a'
#define TPA2018_SET_CONFIG	_IOW(TPA2018_IOCTL_MAGIC, 1, unsigned)
#define TPA2018_READ_CONFIG	_IOR(TPA2018_IOCTL_MAGIC, 2, unsigned)
#define TPA2018_SET_PARAM	_IOW(TPA2018_IOCTL_MAGIC, 3, unsigned)
#define TPA2018_SET_MODE	_IOW(TPA2018_IOCTL_MAGIC, 4, unsigned)

#endif

