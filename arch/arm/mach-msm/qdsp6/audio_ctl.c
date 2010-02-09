/* arch/arm/mach-msm/qdsp6/audio_ctrl.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation
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
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/msm_audio.h>

#include <mach/msm_qdsp6_audio.h>

#define BUFSZ (0)

static DEFINE_MUTEX(voice_lock);
static int voice_started;

static struct audio_client *voc_tx_clnt;
static struct audio_client *voc_rx_clnt;

static int q6_voice_start(uint32_t rx_acdb_id, uint32_t tx_acdb_id)
{
	int rc = 0;

	mutex_lock(&voice_lock);

	if (voice_started) {
		pr_err("voice: busy\n");
		rc = -EBUSY;
		goto done;
	}

	voc_rx_clnt = q6voice_open(AUDIO_FLAG_WRITE, rx_acdb_id);
	if (!voc_rx_clnt) {
		pr_err("voice: open voice rx failed.\n");
		rc = -ENOMEM;
		goto done;
	}

	voc_tx_clnt = q6voice_open(AUDIO_FLAG_READ, tx_acdb_id);
	if (!voc_tx_clnt) {
		pr_err("voice: open voice tx failed.\n");
		q6voice_close(voc_rx_clnt);
		rc = -ENOMEM;
	}

	voice_started = 1;
done:
	mutex_unlock(&voice_lock);
	return rc;
}

static int q6_voice_stop(void)
{
	mutex_lock(&voice_lock);
	if (voice_started) {
		q6voice_close(voc_tx_clnt);
		q6voice_close(voc_rx_clnt);
		voice_started = 0;
	}
	mutex_unlock(&voice_lock);
	return 0;
}

static int q6_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int q6_ioctl(struct inode *inode, struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	int rc;
	uint32_t n;
	uint32_t id[2];
	char filename[64];

	switch (cmd) {
	case AUDIO_SWITCH_DEVICE:
		rc = copy_from_user(&id, (void *)arg, sizeof(id));
		if (!rc)
			rc = q6audio_do_routing(id[0], id[1]);
		break;
	case AUDIO_SET_VOLUME:
		rc = copy_from_user(&n, (void *)arg, sizeof(n));
		if (!rc)
			rc = q6audio_set_rx_volume(n);
		break;
	case AUDIO_SET_MUTE:
		rc = copy_from_user(&n, (void *)arg, sizeof(n));
		if (!rc)
			rc = q6audio_set_tx_mute(n);
		break;
	case AUDIO_UPDATE_ACDB:
		rc = copy_from_user(&id, (void *)arg, sizeof(id));
		if (!rc)
			rc = q6audio_update_acdb(id[0], id[1]);
		break;
	case AUDIO_START_VOICE:
		if (arg == 0) {
			id[0] = id[1] = 0;
		} else if (copy_from_user(&id, (void*) arg, sizeof(id))) {
			pr_info("voice: copy acdb_id from user failed\n");
			rc = -EFAULT;
			break;
		}
		rc = q6_voice_start(id[0], id[1]);
		break;
	case AUDIO_STOP_VOICE:
		rc = q6_voice_stop();
		break;
	case AUDIO_REINIT_ACDB:
		rc = copy_from_user(&filename, (void *)arg, sizeof(filename));
		if (!rc)
			rc = q6audio_reinit_acdb(filename);
		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}


static int q6_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations q6_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= q6_open,
	.ioctl		= q6_ioctl,
	.release	= q6_release,
};

struct miscdevice q6_control_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_audio_ctl",
	.fops	= &q6_dev_fops,
};


static int __init q6_audio_ctl_init(void) {
	return misc_register(&q6_control_device);
}

device_initcall(q6_audio_ctl_init);
