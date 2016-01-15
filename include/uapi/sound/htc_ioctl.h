/*
 *
 * Copyright (C) 2012 HTC Corporation.
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

#define ACOUSTIC_IOCTL_MAGIC 'p'
#define ACOUSTIC_GET_HW_COMPONENT               _IOR(ACOUSTIC_IOCTL_MAGIC, 45, unsigned)
#define ACOUSTIC_UPDATE_BEATS_STATUS            _IOW(ACOUSTIC_IOCTL_MAGIC, 47, unsigned)
#define ACOUSTIC_UPDATE_DQ_STATUS               _IOW(ACOUSTIC_IOCTL_MAGIC, 52, unsigned)
#define ACOUSTIC_NOTIFIY_FM_SSR                 _IOW(ACOUSTIC_IOCTL_MAGIC, 53, unsigned)
#define ACOUSTIC_CONTROL_WAKELOCK               _IOW(ACOUSTIC_IOCTL_MAGIC, 77, unsigned)
#define ACOUSTIC_ADSP_CMD                       _IOW(ACOUSTIC_IOCTL_MAGIC, 98, unsigned)
#define ACOUSTIC_GET_TFA_VER                    _IOR(ACOUSTIC_IOCTL_MAGIC, 99, unsigned)

#define AUD_HW_NUM            10
#define HTC_AUDIO_TPA2051     0x01
#define HTC_AUDIO_TPA2026     0x02
#define HTC_AUDIO_TPA2028     0x04
#define HTC_AUDIO_A1028       0x08
#define HTC_AUDIO_TPA6185     0x10
#define HTC_AUDIO_RT5501      0x20
#define HTC_AUDIO_TFA9887     0x40
#define HTC_AUDIO_TFA9887L    0x80
#define HTC_AUDIO_TFA9888     0x100
#define HTC_AUDIO_RT5503      0x200
