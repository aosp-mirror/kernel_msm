/* arch/arm/mach-msm/qdsp6/mp3.c
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
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include <linux/msm_audio.h>

#include <mach/msm_qdsp6_audio.h>

#define BUFSZ (8192)
#define DMASZ (BUFSZ * 2)

struct mp3 {
	struct mutex lock;
	struct audio_client *ac;
	uint32_t sample_rate;
	uint32_t channel_count;
};

static long mp3_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct mp3 *mp3 = file->private_data;
	int rc = 0;

	if (cmd == AUDIO_GET_STATS) {
		struct msm_audio_stats stats;
		memset(&stats, 0, sizeof(stats));
		if (copy_to_user((void*) arg, &stats, sizeof(stats)))
			return -EFAULT;
		return 0;
	}

	mutex_lock(&mp3->lock);
	switch (cmd) {
	case AUDIO_SET_VOLUME: {
		int vol;
		if (copy_from_user(&vol, (void*) arg, sizeof(vol))) {
			rc = -EFAULT;
			break;
		}
		rc = q6audio_set_stream_volume(mp3->ac, vol);
		break;
	}
	case AUDIO_START: {
		uint32_t acdb_id;
		if (arg == 0) {
			acdb_id = 0;
		} else if (copy_from_user(&acdb_id, (void*) arg, sizeof(acdb_id))) {
			pr_info("pcm_out: copy acdb_id from user failed\n");
			rc = -EFAULT;
			break;
		}
		if (mp3->ac) {
			rc = -EBUSY;
		} else {
			mp3->ac = q6audio_open_mp3(BUFSZ,
				mp3->sample_rate, mp3->channel_count, acdb_id);
			if (!mp3->ac)
				rc = -ENOMEM;
		}
		break;
	}
	case AUDIO_STOP:
		break;
	case AUDIO_FLUSH:
		break;
	case AUDIO_SET_CONFIG: {
		struct msm_audio_config config;
		if (mp3->ac) {
			rc = -EBUSY;
			break;
		}
		if (copy_from_user(&config, (void*) arg, sizeof(config))) {
			rc = -EFAULT;
			break;
		}
		if (config.channel_count < 1 || config.channel_count > 2) {
			rc = -EINVAL;
			break;
		}
		mp3->sample_rate = config.sample_rate;
		mp3->channel_count = config.channel_count;
		break;
	}
	case AUDIO_GET_CONFIG: {
		struct msm_audio_config config;
		config.buffer_size = BUFSZ;
		config.buffer_count = 2;
		config.sample_rate = mp3->sample_rate;
		config.channel_count = mp3->channel_count;
		config.unused[0] = 0;
		config.unused[1] = 0;
		config.unused[2] = 0;
		if (copy_to_user((void*) arg, &config, sizeof(config))) {
			rc = -EFAULT;
		}
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&mp3->lock);
	return rc;
}

static int mp3_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	struct mp3 *mp3;
	mp3 = kzalloc(sizeof(struct mp3), GFP_KERNEL);

	if (!mp3)
		return -ENOMEM;

	mutex_init(&mp3->lock);
	mp3->channel_count = 2;
	mp3->sample_rate = 44100;

	file->private_data = mp3;
	return rc;
}

static ssize_t mp3_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	struct mp3 *mp3 = file->private_data;
	struct audio_client *ac;
	struct audio_buffer *ab;
	const char __user *start = buf;
	int xfer;

	if (!mp3->ac)
		mp3_ioctl(file, AUDIO_START, 0);

	ac = mp3->ac;
	if (!ac)
		return -ENODEV;

	while (count > 0) {
		ab = ac->buf + ac->cpu_buf;

		if (ab->used)
			wait_event(ac->wait, (ab->used == 0));

		xfer = count;
		if (xfer > ab->size)
			xfer = ab->size;

		if (copy_from_user(ab->data, buf, xfer))
			return -EFAULT;

		buf += xfer;
		count -= xfer;

		ab->used = xfer;
		q6audio_write(ac, ab);
		ac->cpu_buf ^= 1;
	}

	return buf - start;
}

static int mp3_fsync(struct file *f, struct dentry *dentry, int datasync)
{
	struct mp3 *mp3 = f->private_data;
	if (mp3->ac)
		return q6audio_async(mp3->ac);
	return -ENODEV;
}

static int mp3_release(struct inode *inode, struct file *file)
{
	struct mp3 *mp3 = file->private_data;
	if (mp3->ac)
		q6audio_mp3_close(mp3->ac);
	kfree(mp3);
	return 0;
}

static struct file_operations mp3_fops = {
	.owner		= THIS_MODULE,
	.open		= mp3_open,
	.write		= mp3_write,
	.fsync		= mp3_fsync,
	.release	= mp3_release,
	.unlocked_ioctl	= mp3_ioctl,
};

struct miscdevice mp3_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_mp3",
	.fops	= &mp3_fops,
};

static int __init mp3_init(void) {
	return misc_register(&mp3_misc);
}

device_initcall(mp3_init);
