/* include/asm/mach-msm/htc_acoustic_alsa.h
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
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/sched.h>
#include <linux/qdsp6v2/apr.h>
#include <sound/htc_ioctl.h>
#include <linux/device.h>

#ifndef _ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_QCT_ALSA_H_
#define _ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_QCT_ALSA_H_


#ifdef CONFIG_HTC_HEADSET_MGR
enum HS_NOTIFY_TYPE {
	HS_AMP_N = 0,
	HS_CODEC_N,
	HS_N_MAX,
};

struct hs_notify_t {
	int used;
	void *private_data;
	int (*callback_f)(void*,int);
};
#endif

enum HTC_FEATURE {
	HTC_Q6_EFFECT = 0,
	HTC_AUD_24BIT,
};

struct avcs_crash_params {
    struct apr_hdr  hdr;
    uint32_t crash_type;
};

struct acoustic_ops {
	void (*set_q6_effect)(int mode);
	int (*get_htc_revision)(void);
	int (*get_hw_component)(void);
	char* (*get_mid)(void);
	int (*enable_digital_mic)(void);
	int (*enable_24b_audio)(void);
	int (*get_q6_effect) (void);
};

void htc_acoustic_register_ops(struct acoustic_ops *ops);
void register_htc_ftm_dev(struct device *);
void set_pinctrl_ftm_mode(int enable);
void htc_acoustic_register_spk_version(void (*spk_func)(unsigned char*));

#ifdef CONFIG_HTC_HEADSET_MGR
void htc_acoustic_register_hs_notify(enum HS_NOTIFY_TYPE type, struct hs_notify_t *notify);
#endif

/* To query if feature is enable */
int htc_acoustic_query_feature(enum HTC_FEATURE feature);

#endif

