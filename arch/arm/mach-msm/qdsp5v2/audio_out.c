/* arch/arm/mach-msm/qdsp5v2/audio_out.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include <linux/delay.h>

#include <linux/msm_audio.h>

#include "adsp.h"
#include "adsp_audio.h"

#include "adsp_module_afe.h"


void adie_enable(void);

struct audio_buffer {
	dma_addr_t phys;
	void *data;
	uint32_t size;
	uint32_t used;
};
	
struct audio {
	struct audio_buffer buf[2];

	int cpu_buf;
	int dsp_buf;
	int running;
	int session;

	wait_queue_head_t wait;
	struct audplay *audplay;
	void *data;
	dma_addr_t phys;
};

static void audio_send_data(void *cookie)
{
	struct audio *audio = cookie;
	struct audio_buffer *ab = audio->buf + audio->dsp_buf;

	if (ab->used) {
		ab->used = 0;
		audio->dsp_buf ^= 1;
		wake_up(&audio->wait);
	}
}


static int need_init = 1;

static int audio_open(struct inode *inode, struct file *file)
{
	int ret;
	struct audio *audio;

	if (need_init) {
		msm_codec_output(1);
		afe_enable(AFE_DEVICE_MI2S_CODEC_RX, 48000, 2);
		adie_enable();
		need_init = 0;
	}

#if 0
	msleep(5000);
	afe_disable(AFE_DEVICE_MI2S_CODEC_RX);
	msm_codec_output(0);

	msleep(5000);
	msm_codec_output(1);
	afe_enable(AFE_DEVICE_MI2S_CODEC_RX, 48000, 2);
	return 0;
#endif

	audio = kzalloc(sizeof(*audio), GFP_KERNEL);
	if (!audio)
		return -ENOMEM;

	audio->data = dma_alloc_coherent(NULL, 8192, &audio->phys, GFP_KERNEL);
	if (!audio->data) {
		pr_err("audio: could not allocate DMA buffers\n");
		kfree(audio);
		return -ENOMEM;
	}

	init_waitqueue_head(&audio->wait);

	audio->buf[0].phys = audio->phys;
	audio->buf[0].data = audio->data;
	audio->buf[0].size = 4096;
	audio->buf[0].used = 0;
	audio->buf[1].phys = audio->phys + 4096;
	audio->buf[1].data = audio->data + 4096;
	audio->buf[1].size = 4096;
	audio->buf[1].used = 0;

	audio->audplay = audplay_get(audio_send_data, audio);
	if (!audio->audplay) {
		kfree(audio);
		return -ENODEV;
	}

	audplay_dsp_config(audio->audplay, 1);

	audplay_config_pcm(audio->audplay, 44100, 16, 2);

	audplay_mix_select(audio->audplay, 1);
	audplay_volume_pan(audio->audplay, 0x2000, 0);

	file->private_data = audio;
	return 0;
}

static ssize_t audio_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	struct audio *audio = file->private_data;
	struct audio_buffer *ab;
	const char __user *start = buf;
	int xfer;

	while (count > 0) {
		ab = audio->buf + audio->cpu_buf;

		if (ab->used)
			if (!wait_event_timeout(audio->wait,
						(ab->used == 0), 5*HZ)) {
				pr_err("audio_write: timeout. dsp dead?\n");
				return -EIO;
			}

		xfer = count;
		if (xfer > ab->size)
			xfer = ab->size;

		if (copy_from_user(ab->data, buf, xfer)) 
			return -EFAULT;

		buf += xfer;
		count -= xfer;

		ab->used = xfer;
		audplay_send_data(audio->audplay, ab->phys, ab->used);
		audio->cpu_buf ^= 1;
	}

	return buf - start;
}

static int audio_release(struct inode *inode, struct file *file)
{
	struct audio *audio = file->private_data;
	pr_info("audio_release()\n");
	audplay_dsp_config(audio->audplay, 0);
	audplay_put(audio->audplay);
	kfree(audio);
	return 0;
}

static long audio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct audio *audio = file->private_data;
	int rc = 0;

	switch (cmd) {
	case AUDIO_SET_VOLUME: {
		int vol;
		if (copy_from_user(&vol, (void*) arg, sizeof(vol))) {
			rc = -EFAULT;
			break;
		}
		pr_info("audio_out: volume %d\n", vol);
		break;
	}
	case AUDIO_GET_STATS:
	case AUDIO_START:
	case AUDIO_STOP:
	case AUDIO_FLUSH:
	case AUDIO_SET_CONFIG:
		/* implement me! */
		break;
	case AUDIO_GET_CONFIG: {
		struct msm_audio_config config;
		config.buffer_size = 4096;
		config.buffer_count = 2;
		config.sample_rate = 44100;
		config.channel_count = 2;
		config.unused[0] = 0;
		config.unused[1] = 0;
		config.unused[2] = 0;
		if (copy_to_user((void*) arg, &config, sizeof(config))) {
			rc = -EFAULT;
		}
		break;
	}
	}
	return rc;
}


static const struct file_operations audio_out_fops = {
	.owner		= THIS_MODULE,
	.open		= audio_open,
	.release	= audio_release,
	.write		= audio_write,
	.unlocked_ioctl	= audio_ioctl,
};

struct miscdevice audio_out_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_pcm_out",
	.fops	= &audio_out_fops,
};

static int __init audio_init(void)
{
	adsp_audio_init();
	return misc_register(&audio_out_misc);
}

device_initcall(audio_init);
