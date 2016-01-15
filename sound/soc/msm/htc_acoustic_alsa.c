/* arch/arm/mach-msm/htc_acoustic_alsa.c
 *
 * Copyright (C) 2012 HTC Corporation
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <sound/htc_acoustic_alsa.h>
#include "qdsp6v2/msm-pcm-routing-v2.h"
#ifdef CONFIG_HTC_HEADSET_MGR
#include <linux/htc_headset_mgr.h>
#endif

#define AVCS_CMD_ADSP_CRASH                      0x0001FFFF
#define D(fmt, args...) printk(KERN_INFO "[AUD] htc-acoustic: "fmt, ##args)
#define E(fmt, args...) printk(KERN_ERR "[AUD] htc-acoustic: "fmt, ##args)

static DEFINE_MUTEX(api_lock);
static struct acoustic_ops *the_ops = NULL;
static struct switch_dev sdev_beats;
static struct switch_dev sdev_dq;
static struct switch_dev sdev_fm;
extern struct wake_lock compr_lpa_q6_cb_wakelock;
static struct wake_lock htc_acoustic_wakelock;
static struct wake_lock htc_acoustic_wakelock_timeout;
struct avcs_ctl {
       atomic_t ref_cnt;
       void *apr;
};
static struct device* ftm_dev = NULL;
static struct pinctrl *ftm_pinctrl = NULL;
static struct pinctrl_state *ftm_state = NULL;
static struct avcs_ctl this_avcs;
static int ftm_init = 0;
static void (*htc_spk_version)(unsigned char*) = NULL;

#ifdef CONFIG_HTC_HEADSET_MGR
static struct hs_notify_t hs_plug_nt[HS_N_MAX] = {{0, NULL, NULL}};
static DEFINE_MUTEX(hs_nt_lock);
#endif

void htc_acoustic_register_spk_version(void (*spk_func)(unsigned char*)) {
	htc_spk_version = spk_func;
}

void register_htc_ftm_dev(struct device* dev)
{
	if (dev != NULL)
		ftm_dev = dev;
	else {
		pr_err("%s: Unable to find PRI AUX device.\n", __func__);
		return;
	}

	ftm_pinctrl = devm_pinctrl_get(ftm_dev);
	if (IS_ERR_OR_NULL(ftm_pinctrl)) {
		pr_err("%s: Unable to get ftm_pinctrl handle\n", __func__);
		return;
	}

	ftm_state = pinctrl_lookup_state(ftm_pinctrl,
					"ftm");
	if (IS_ERR_OR_NULL(ftm_state)) {
		pr_err("%s: could not get ftm pinstate\n", __func__);
		return;
	}

	ftm_init = 1;
}

void set_pinctrl_ftm_mode(int ftm_enable)
{
	int ret = 0;
	int aux_pcm_active = 0;
	struct msm_pcm_routing_bdai_data bedai;

	msm_pcm_routing_get_bedai_info(MSM_BACKEND_DAI_AUXPCM_RX, &bedai);
	aux_pcm_active = bedai.active;

	if (!ftm_init) {
		pr_err("%s: unable to set ftm control due to init fail.\n", __func__);
		return;
	}

	if (ftm_enable) {
		//ftm enable, go ftm control
		ret = pinctrl_select_state(ftm_pinctrl, ftm_state);
	}
	else if(aux_pcm_active){
		//AUX PCM active, go default control
		ret = pinctrl_pm_select_default_state(ftm_dev);
	}
	else {
		//AUX PCM non-active, go sleep control
		ret = pinctrl_pm_select_sleep_state(ftm_dev);
	}

	if (ret)
		pr_err("%s select pinctrl state fail!", __func__);
}

void htc_acoustic_register_ops(struct acoustic_ops *ops)
{
	D("acoustic_register_ops \n");
	mutex_lock(&api_lock);
	the_ops = ops;
	mutex_unlock(&api_lock);
}

static int32_t avcs_callback(struct apr_client_data *data, void *priv)
{
	if (data->opcode == RESET_EVENTS) {
		pr_debug("%s: Reset event is received: %d %d apr[%p]\n",
				__func__,
				data->reset_event,
				data->reset_proc,
				this_avcs.apr);
		apr_reset(this_avcs.apr);
		atomic_set(&this_avcs.ref_cnt, 0);
		this_avcs.apr = NULL;
		return 0;
	}
	pr_info("%s: avcs command send done.\n", __func__);
	return 0;
}

#ifdef CONFIG_HTC_HEADSET_MGR
void htc_acoustic_register_hs_notify(enum HS_NOTIFY_TYPE type, struct hs_notify_t *notify)
{
	pr_info("%s enter, type %d\n", __func__, (int)type);

	if (notify == NULL) {
		pr_info("%s: notify is null\n", __func__);
		return;
	}

	mutex_lock(&hs_nt_lock);
	if (hs_plug_nt[type].used) {
		pr_err("%s: hs notification %d is reigstered\n", __func__, (int)type);
	} else {
		hs_plug_nt[type].private_data = notify->private_data;
		hs_plug_nt[type].callback_f = notify->callback_f;
		hs_plug_nt[type].used = 1;
	}
	mutex_unlock(&hs_nt_lock);
}

static int htc_acoustic_hsnotify(int on)
{
	int i = 0;

	mutex_lock(&hs_nt_lock);
	for (i = 0; i < HS_N_MAX; i++) {
		if (hs_plug_nt[i].used && hs_plug_nt[i].callback_f)
			hs_plug_nt[i].callback_f(hs_plug_nt[i].private_data, on);
	}
	mutex_unlock(&hs_nt_lock);

	return 0;
}
#endif

int htc_acoustic_query_feature(enum HTC_FEATURE feature)
{
	int ret = -1;
	mutex_lock(&api_lock);
	switch(feature) {
		case HTC_Q6_EFFECT:
			if(the_ops && the_ops->get_q6_effect)
				ret = the_ops->get_q6_effect();
			break;
		case HTC_AUD_24BIT:
			if(the_ops && the_ops->enable_24b_audio)
				ret = the_ops->enable_24b_audio();
			break;
		default:
			break;

	};
	mutex_unlock(&api_lock);
	return ret;
}

static int acoustic_open(struct inode *inode, struct file *file)
{
	D("open\n");
	return 0;
}

static int acoustic_release(struct inode *inode, struct file *file)
{
	D("release\n");
	return 0;
}

static long
acoustic_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	unsigned char *buf = NULL;
	unsigned int us32_size = 0;
	int s32_value = 0;
	void __user *argp = (void __user *)arg;

	if (_IOC_TYPE(cmd) != ACOUSTIC_IOCTL_MAGIC)
		return -ENOTTY;

//	if (_IOC_SIZE(cmd) > sizeof(struct tfa9895_i2c_buffer))
//		return -EINVAL;

	us32_size = _IOC_SIZE(cmd);

	buf = kzalloc(us32_size, GFP_KERNEL);

	if (buf == NULL) {
		E("%s %d: allocate kernel buffer failed.\n", __func__, __LINE__);
		return -EFAULT;
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		rc = copy_from_user(buf, argp, us32_size);
		if (rc) {
			E("%s %d: copy_from_user fail.\n", __func__, __LINE__);
			rc = -EFAULT;
		}
		else {
			D("%s %d: copy_from_user ok. size=%#x\n", __func__, __LINE__, us32_size);
		}
	}

	mutex_lock(&api_lock);
	switch (cmd) {
		case ACOUSTIC_GET_HW_COMPONENT: {
			if (the_ops && the_ops->get_hw_component) {
				s32_value = the_ops->get_hw_component();
			} else {
				s32_value = 0;
			}
			if(sizeof(s32_value) <= us32_size) {
				memcpy((void*)buf, (void*)&s32_value, sizeof(s32_value));
				D("%s %d: ACOUSTIC_GET_HW_COMPONENT %#x\n", __func__, __LINE__, s32_value);
			} else {
				E("%s %d: ACOUSTIC_GET_HW_COMPONENT error.\n", __func__, __LINE__);
				rc = -EINVAL;
			}
			break;
		}
		case ACOUSTIC_UPDATE_BEATS_STATUS: {
			if(sizeof(s32_value) <= us32_size) {
				memcpy((void*)&s32_value, (void*)buf, sizeof(s32_value));
				D("%s %d: ACOUSTIC_UPDATE_BEATS_STATUS %#x\n", __func__, __LINE__, s32_value);
				if (s32_value < -1 || s32_value > 1) {
					rc = -EINVAL;
					break;
				}
				sdev_beats.state = -1;
				switch_set_state(&sdev_beats, s32_value);
			} else {
				E("%s %d: ACOUSTIC_UPDATE_BEATS_STATUS error.\n", __func__, __LINE__);
				rc = -EINVAL;
			}
			break;
		}
		case ACOUSTIC_UPDATE_DQ_STATUS: {
			if(sizeof(s32_value) <= us32_size) {
				memcpy((void*)&s32_value, (void*)buf, sizeof(s32_value));
				D("%s %d: ACOUSTIC_UPDATE_DQ_STATUS %#x\n", __func__, __LINE__, s32_value);
				if (s32_value < -1 || s32_value > 1) {
					rc = -EINVAL;
					break;
				}
				sdev_dq.state = -1;
				switch_set_state(&sdev_dq, s32_value);
			} else {
				E("%s %d: ACOUSTIC_UPDATE_DQ_STATUS error.\n", __func__, __LINE__);
				rc = -EINVAL;
			}
			break;
		}
		case ACOUSTIC_NOTIFIY_FM_SSR: {
			int new_state = -1;

			if (copy_from_user(&new_state, (void *)arg, sizeof(new_state))) {
				rc = -EFAULT;
				break;
			}
			D("Update FM SSR Status : %d\n", new_state);
			if (new_state < -1 || new_state > 1) {
				E("Invalid FM status update");
				rc = -EINVAL;
				break;
			}

			sdev_fm.state = -1;
			switch_set_state(&sdev_fm, new_state);
			break;
		}
		case ACOUSTIC_CONTROL_WAKELOCK: {
			if(sizeof(s32_value) <= us32_size) {
				memcpy((void*)&s32_value, (void*)buf, sizeof(s32_value));
				D("%s %d: ACOUSTIC_CONTROL_WAKELOCK %#x\n", __func__, __LINE__, s32_value);
				if (s32_value < -1 || s32_value > 1) {
					rc = -EINVAL;
					break;
				}
				if (s32_value == 1) {
					wake_lock_timeout(&htc_acoustic_wakelock, 60*HZ);
					D("%s %d: wake_unlock compr_lpa_q6_cb_wakelock\n", __func__, __LINE__);
					wake_unlock(&compr_lpa_q6_cb_wakelock );
				} else {
					wake_lock_timeout(&htc_acoustic_wakelock_timeout, 1*HZ);
					wake_unlock(&htc_acoustic_wakelock);
				}
			} else {
				E("%s %d: ACOUSTIC_CONTROL_WAKELOCK error.\n", __func__, __LINE__);
				rc = -EINVAL;
			}
			break;
		}
		case  ACOUSTIC_ADSP_CMD: {
			//uint16_t cmd_size;
			struct avcs_crash_params config;
			config.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
			config.hdr.pkt_size = sizeof(struct avcs_crash_params);
			config.hdr.src_port = 0;
			config.hdr.dest_port = 0;
			config.hdr.token = 0;
			config.hdr.opcode = AVCS_CMD_ADSP_CRASH;
			config.crash_type = 0;

			if(sizeof(s32_value) <= us32_size) {
				memcpy((void*)&s32_value, (void *)buf, sizeof(s32_value));
				D("%s %d: ACOUSTIC_ADSP_CMD %#x\n", __func__, __LINE__, s32_value);
				if (s32_value <= 0) {
					pr_err("%s %d: copy to user failed\n", __func__, __LINE__);
					rc = -EFAULT;
					break;
				}
				if ((atomic_read(&this_avcs.ref_cnt) == 0) ||
					(this_avcs.apr == NULL)) {
					this_avcs.apr = apr_register("ADSP", "CORE", avcs_callback,
						0xFFFFFFFF, NULL);

					if (this_avcs.apr == NULL) {
						pr_err("%s: Unable to register apr\n", __func__);
						rc = -ENODEV;
						break;
					}

					atomic_inc(&this_avcs.ref_cnt);
				}

				rc = apr_send_pkt(this_avcs.apr, (uint32_t *)(&config));
				if (rc < 0) {
					pr_err("%s: send ADSP CMD failed, %d.\n", __func__, rc);
				} else {
					pr_info("%s: send ADSP CMD success.\n", __func__);
					rc = 0;
				}
			}
			break;
		}
		case  ACOUSTIC_GET_TFA_VER: {
			unsigned char version[2] = {0x00,0x00};
			if(htc_spk_version) {
				htc_spk_version(version);
			}
			memcpy((void*)buf, (void*)version, sizeof(version));
			break;
		}
		default: {
			rc= -EINVAL;
			break;
		}
	}

	if(0 == rc) {
		if (_IOC_DIR(cmd) & _IOC_READ) {
			rc = copy_to_user(argp, buf, us32_size);
			if (rc) {
				E("%s %d: copy_to_user fail.\n", __func__, __LINE__);
				rc = -EFAULT;
			}
			else {
				D("%s %d: copy_to_user ok. size=%#x\n", __func__, __LINE__, us32_size);
			}
		}
	}

	kfree(buf);
	mutex_unlock(&api_lock);
	return rc;
}

static ssize_t beats_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Beats\n");
}

static ssize_t dq_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "DQ\n");
}

static ssize_t fm_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "FM\n");
}

static struct file_operations acoustic_fops = {
	.owner = THIS_MODULE,
	.open = acoustic_open,
	.release = acoustic_release,
	.unlocked_ioctl = acoustic_ioctl,
	.compat_ioctl = acoustic_ioctl,
};

static struct miscdevice acoustic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-acoustic",
	.fops = &acoustic_fops,
};

static int __init acoustic_init(void)
{
	int ret = 0;
#ifdef CONFIG_HTC_HEADSET_MGR
	struct headset_notifier notifier;
#endif

	ret = misc_register(&acoustic_misc);
	wake_lock_init(&htc_acoustic_wakelock, WAKE_LOCK_SUSPEND, "htc_acoustic");
	wake_lock_init(&htc_acoustic_wakelock_timeout, WAKE_LOCK_SUSPEND, "htc_acoustic_timeout");

	if (ret < 0) {
		pr_err("failed to register misc device!\n");
		return ret;
	}

	sdev_beats.name = "Beats";
	sdev_beats.print_name = beats_print_name;

	ret = switch_dev_register(&sdev_beats);
	if (ret < 0) {
		pr_err("failed to register beats switch device!\n");
		return ret;
	}

	sdev_dq.name = "DQ";
	sdev_dq.print_name = dq_print_name;

	ret = switch_dev_register(&sdev_dq);
	if (ret < 0) {
		pr_err("failed to register DQ switch device!\n");
		return ret;
	}

	sdev_fm.name = "FM";
	sdev_fm.print_name = fm_print_name;

	ret = switch_dev_register(&sdev_fm);
	if (ret < 0) {
		pr_err("failed to register FM switch device!\n");
		return ret;
	}

#ifdef CONFIG_HTC_HEADSET_MGR
	notifier.id = HEADSET_REG_MIC_BIAS2;
	notifier.func = htc_acoustic_hsnotify;
	headset_notifier_register(&notifier);
#endif

	return 0;
}

static void __exit acoustic_exit(void)
{
	misc_deregister(&acoustic_misc);
}

module_init(acoustic_init);
module_exit(acoustic_exit);
