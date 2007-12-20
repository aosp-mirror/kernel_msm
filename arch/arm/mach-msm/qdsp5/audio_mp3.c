/* arch/arm/mach-msm/qdsp5/audio_mp3.c
 * 
 * mp3 audio output device
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2008 HTC Corporation
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>

#include <linux/delay.h>

#include <asm/atomic.h>
#include <asm/ioctls.h>
#include <mach/msm_adsp.h>

#include <linux/msm_audio.h>

#include "audmgr.h"

#include <mach/qdsp5/qdsp5audppcmdi.h>
#include <mach/qdsp5/qdsp5audppmsg.h>
#include <mach/qdsp5/qdsp5audplaycmdi.h>
#include <mach/qdsp5/qdsp5audplaymsg.h>

/* for queue ids - should be relative to module number*/
#include "adsp.h"

#define BUFSZ 32768
#define DMASZ (BUFSZ * 2)

#define AUDPLAY_INVALID_READ_PTR_OFFSET	0xFFFF
#define AUDDEC_DEC_MP3 2

/* Decoder status received from AUDPPTASK */
typedef enum {
  AUDPP_DEC_STATUS_SLEEP,
  AUDPP_DEC_STATUS_INIT,
  AUDPP_DEC_STATUS_CFG,
  AUDPP_DEC_STATUS_PLAY
} audpp_dec_status_type;

struct buffer {
	void *data;
	unsigned size;
	unsigned used;
	unsigned addr;
};

struct audio {
	struct buffer out[2];

	spinlock_t dsp_lock;

	uint8_t out_head;
	uint8_t out_tail;
	uint8_t out_needed; /* number of buffers the dsp is waiting for */

	atomic_t out_bytes;

	struct mutex lock;
	struct mutex write_lock;
	wait_queue_head_t wait;

	struct msm_adsp_module *audplay;

	/* configuration to use on next enable */
	uint32_t out_sample_rate;
	uint32_t out_channel_mode;

	struct audmgr audmgr;

	/* data allocated for various buffers */
	char *data;
	dma_addr_t phys;

	int opened;
	int enabled;
	int running;
	int stopped; /* set when stopped, cleared on flush */
	unsigned volume;

	uint16_t dec_id;
	uint32_t read_ptr_offset;
};

static int auddec_dsp_config(struct audio *audio, int enable);
static void audpp_cmd_cfg_adec_params(struct audio *audio);
static void audplay_send_data(struct audio *audio, unsigned needed);

static void audio_dsp_event(void *private, unsigned id, uint16_t *msg);

/* must be called with audio->lock held */
static int audio_enable(struct audio *audio)
{
	struct audmgr_config cfg;
	int rc;

	pr_info("audio_enable()\n");

	if (audio->enabled)
		return 0;

	audio->out_tail = 0;
	audio->out_needed = 0;

	cfg.tx_rate = RPC_AUD_DEF_SAMPLE_RATE_NONE;
	cfg.rx_rate = RPC_AUD_DEF_SAMPLE_RATE_48000;
	cfg.def_method = RPC_AUD_DEF_METHOD_PLAYBACK;
	cfg.codec = RPC_AUD_DEF_CODEC_MP3;
	cfg.snd_method = RPC_SND_METHOD_MIDI;

	rc = audmgr_enable(&audio->audmgr, &cfg);
	if (rc < 0)
		return rc;

	if (audpp_enable(audio->dec_id, audio_dsp_event, audio)) {
		pr_err("audio: audpp_enable() failed\n");
		audmgr_disable(&audio->audmgr);
		return -ENODEV;
	}

	if (msm_adsp_enable(audio->audplay)) {
		pr_err("audio: msm_adsp_enable(audplay) failed\n");
		audmgr_disable(&audio->audmgr);
		return -ENODEV;
	}

	audio->enabled = 1;
	return 0;
}

/* must be called with audio->lock held */
static int audio_disable(struct audio *audio)
{
	pr_info("audio_disable()\n");
	if (audio->enabled) {
		audio->enabled = 0;
		auddec_dsp_config(audio, 0);
		wake_up(&audio->wait);
		msm_adsp_disable(audio->audplay);
		audpp_disable(audio->dec_id, audio);
		audmgr_disable(&audio->audmgr);
		audio->out_needed = 0;
	}
	return 0;
}

/* ------------------- dsp --------------------- */

static void audplay_dsp_event(void *data, unsigned id, size_t len,
			    void (*getevent)(void *ptr, size_t len))
{
	struct audio *audio = data;
	uint32_t msg[28];
	getevent(msg, sizeof(msg));

	switch (id) {
	case AUDPLAY_MSG_DEC_NEEDS_DATA:
		audplay_send_data(audio, 1);
		break;
	default:
		pr_err("unexpected message from decoder \n");
		break;
	}
}

static void audio_dsp_event(void *private, unsigned id, uint16_t *msg)
{
	struct audio *audio = private;

	switch (id) {
	case AUDPP_MSG_STATUS_MSG: {
		unsigned status = msg[1];

		switch (status) {
		case AUDPP_DEC_STATUS_SLEEP:
			pr_info("decoder status: sleep \n");
			break;
		case AUDPP_DEC_STATUS_INIT: {
			pr_info("decoder status: init \n");
			audpp_cmd_cfg_adec_params(audio);
			break;
		}
		case AUDPP_DEC_STATUS_CFG:
			pr_info("decoder status: cfg \n");
			break;
		case AUDPP_DEC_STATUS_PLAY:
			pr_info("decoder status: play \n");
			break;
		default:
			pr_err("unknown decoder status \n");
			break;
		}
		break;
	}
	case AUDPP_MSG_CFG_MSG:
		if (msg[0] == AUDPP_MSG_ENA_ENA) {
			pr_info("audio_dsp_event: CFG_MSG ENABLE\n");
			auddec_dsp_config(audio, 1);
			audio->out_needed = 0;
			audio->running = 1;
			audpp_set_volume_and_pan(audio->dec_id, audio->volume, 0);
			audpp_avsync(audio->dec_id, 22050);
		} else if (msg[0] == AUDPP_MSG_ENA_DIS) {
			pr_info("audio_dsp_event: CFG_MSG DISABLE\n");
			audpp_avsync(audio->dec_id, 0);
			audio->running = 0;
		} else {
			pr_err("audio_dsp_event: CFG_MSG %d?\n", msg[0]);
		}
		break;
	default:
		pr_err("audio_dsp_event: UNKNOWN (%d)\n", id);
	}

}


struct msm_adsp_ops audplay_adsp_ops = {
	.event = audplay_dsp_event,
};


#define audplay_send_queue0(audio, cmd, len) \
	msm_adsp_write(audio->audplay, QDSP_uPAudPlay0BitStreamCtrlQueue, cmd, len)

static int auddec_dsp_config(struct audio *audio, int enable)
{
	audpp_cmd_cfg_dec_type cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd_id = AUDPP_CMD_CFG_DEC_TYPE;
	if (enable)
		cmd.dec0_cfg = AUDPP_CMD_UPDATDE_CFG_DEC |
			       AUDPP_CMD_ENA_DEC_V |
			       AUDDEC_DEC_MP3;
	else
		cmd.dec0_cfg = AUDPP_CMD_UPDATDE_CFG_DEC |
			       AUDPP_CMD_DIS_DEC_V;

	return audpp_send_queue1(&cmd, sizeof(cmd));
}

static void audpp_cmd_cfg_adec_params(struct audio *audio)
{
	audpp_cmd_cfg_adec_params_mp3 cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.common.cmd_id = AUDPP_CMD_CFG_ADEC_PARAMS;
	cmd.common.length = AUDPP_CMD_CFG_ADEC_PARAMS_MP3_LEN;
	cmd.common.dec_id = audio->dec_id;
	cmd.common.input_sampling_frequency = audio->out_sample_rate;

	audpp_send_queue2(&cmd, sizeof(cmd));
}

static int audplay_dsp_send_data_avail(struct audio *audio,
					unsigned idx, unsigned len)
{
	audplay_cmd_bitstream_data_avail cmd;

	cmd.cmd_id		= AUDPLAY_CMD_BITSTREAM_DATA_AVAIL;
	cmd.decoder_id		= audio->dec_id;
	cmd.buf_ptr		= audio->out[idx].addr;
	cmd.buf_size		= len/2;
	cmd.partition_number	= 0;
	return audplay_send_queue0(audio, &cmd, sizeof(cmd));
}

static void audplay_send_data(struct audio *audio, unsigned needed)
{
	struct buffer *frame;
	unsigned long flags;

	spin_lock_irqsave(&audio->dsp_lock, flags);
	if (!audio->running)
		goto done;

	if (needed) {
		/* We were called from the callback because the DSP
		 * requested more data.  Note that the DSP does want
		 * more data, and if a buffer was in-flight, mark it
		 * as available (since the DSP must now be done with
		 * it).
		 */
		audio->out_needed = 1;
		frame = audio->out + audio->out_tail;
		if (frame->used == 0xffffffff) {
			frame->used = 0;
			audio->out_tail ^= 1;
			wake_up(&audio->wait);
		}
	}

	if (audio->out_needed) {
		/* If the DSP currently wants data and we have a
		 * buffer available, we will send it and reset
		 * the needed flag.  We'll mark the buffer as in-flight
		 * so that it won't be recycled until the next buffer
		 * is requested
		 */
			
		frame = audio->out + audio->out_tail;
		if (frame->used) {
			BUG_ON(frame->used == 0xffffffff);
			audplay_dsp_send_data_avail(audio, audio->out_tail, frame->used);
			frame->used = 0xffffffff;
			audio->out_needed = 0;
		}
	}
done:
	spin_unlock_irqrestore(&audio->dsp_lock, flags);
}

/* ------------------- device --------------------- */

static void audio_flush(struct audio *audio)
{
	audio->out[0].used = 0;
	audio->out[1].used = 0;
	audio->out_head = 0;
	audio->out_tail = 0;
	audio->stopped = 0;
	atomic_set(&audio->out_bytes, 0);
}

static long audio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct audio *audio = file->private_data;
	int rc;

	if (cmd == AUDIO_GET_STATS) {
		struct msm_audio_stats stats;
		stats.byte_count = audpp_avsync_byte_count(audio->dec_id);
		stats.sample_count = audpp_avsync_sample_count(audio->dec_id);
		if (copy_to_user((void *) arg, &stats, sizeof(stats)))
			return -EFAULT;
		return 0;
	}
	if (cmd == AUDIO_SET_VOLUME) {
		unsigned long flags;
		spin_lock_irqsave(&audio->dsp_lock, flags);
		audio->volume = arg;
		if (audio->running)
			audpp_set_volume_and_pan(audio->dec_id, arg, 0);
		spin_unlock_irqrestore(&audio->dsp_lock, flags);
	}
	mutex_lock(&audio->lock);
	switch (cmd) {
	case AUDIO_START:
		rc = audio_enable(audio);
		break;
	case AUDIO_STOP:
		rc = audio_disable(audio);
		audio->stopped = 1;
		break;
	case AUDIO_FLUSH:
		if (audio->stopped) {
			/* Make sure we're stopped and we wake any threads
			 * that might be blocked holding the write_lock.
			 * While audio->stopped write threads will always
			 * exit immediately.
			 */
			wake_up(&audio->wait);
			mutex_lock(&audio->write_lock);
			audio_flush(audio);
			mutex_unlock(&audio->write_lock);
		}
	case AUDIO_SET_CONFIG: {
		struct msm_audio_config config;
		if (copy_from_user(&config, (void *) arg, sizeof(config))) {
			rc = -EFAULT;
			break;
		}
		if (config.channel_count == 1) {
			config.channel_count = AUDPP_CMD_PCM_INTF_MONO_V;
		} else if (config.channel_count == 2) {
			config.channel_count = AUDPP_CMD_PCM_INTF_STEREO_V;
		} else {
			rc = -EINVAL;
			break;
		}
		audio->out_sample_rate = config.sample_rate;
		audio->out_channel_mode = config.channel_count;
		rc = 0;
		break;
	}
	case AUDIO_GET_CONFIG: {
		struct msm_audio_config config;
		config.buffer_size = BUFSZ;
		config.buffer_count = 2;
		config.sample_rate = audio->out_sample_rate;
		if (audio->out_channel_mode == AUDPP_CMD_PCM_INTF_MONO_V) {
			config.channel_count = 1;
		} else {
			config.channel_count = 2;
		}
		config.unused[0] = 0;
		config.unused[1] = 0;
		config.unused[2] = 0;
		config.unused[3] = 0;
		if (copy_to_user((void *) arg, &config, sizeof(config))) {
			rc = -EFAULT;
		} else {
			rc = 0;
		}
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&audio->lock);
	return rc;
}

static ssize_t audio_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	return -EINVAL;
}

static ssize_t audio_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	struct audio *audio = file->private_data;
	const char __user *start = buf;
	struct buffer *frame;
	size_t xfer;
	int rc = 0;

	if (count & 1)
		return -EINVAL;

	mutex_lock(&audio->write_lock);
	while (count > 0) {
		frame = audio->out + audio->out_head;
		rc = wait_event_interruptible(audio->wait,
					      (frame->used == 0) || (audio->stopped));
		if (rc < 0)
			break;
		if (audio->stopped) {
			rc = -EBUSY;
			break;
		}
		xfer = (count > frame->size) ? frame->size : count;
		if (copy_from_user(frame->data, buf, xfer)) {
			rc = -EFAULT;
			break;
		}

		frame->used = xfer;
		audio->out_head ^= 1;
		count -= xfer;
		buf += xfer;

		audplay_send_data(audio, 0);

	}
	mutex_unlock(&audio->write_lock);
	if (buf > start)
		return buf - start;
	return rc;
}

static int audio_release(struct inode *inode, struct file *file)
{
	struct audio *audio = file->private_data;

	mutex_lock(&audio->lock);
	audio_disable(audio);
	audio_flush(audio);
	msm_adsp_put(audio->audplay);
	audio->audplay = NULL;
	audio->opened = 0;
	mutex_unlock(&audio->lock);
	return 0;
}

struct audio the_mp3_audio;

static int audio_open(struct inode *inode, struct file *file)
{
	struct audio *audio = &the_mp3_audio;
	int rc;

	mutex_lock(&audio->lock);

	if (audio->opened) {
		pr_err("audio: busy\n");
		rc = -EBUSY;
		goto done;
	}

	if (!audio->data) {
		audio->data = dma_alloc_coherent(NULL, DMASZ,
						 &audio->phys, GFP_KERNEL);
		if (!audio->data) {
			pr_err("audio: could not allocate DMA buffers\n");
			rc = -ENOMEM;
			goto done;
		}
	}

	rc = audmgr_open(&audio->audmgr);
	if (rc)
		goto done;

	rc = msm_adsp_get("AUDPLAY0TASK", &audio->audplay, &audplay_adsp_ops, audio);
	if (rc) {
		pr_err("audio: failed to get audplay0 dsp module\n");
		goto done;
	}
	audio->out_sample_rate = 44100;
	audio->out_channel_mode = AUDPP_CMD_PCM_INTF_STEREO_V;
	audio->dec_id = 0;

	audio->out[0].data = audio->data + 0;
	audio->out[0].addr = audio->phys + 0;
	audio->out[0].size = BUFSZ;

	audio->out[1].data = audio->data + BUFSZ;
	audio->out[1].addr = audio->phys + BUFSZ;
	audio->out[1].size = BUFSZ;

	audio->volume = 0x2000; /* Q13 1.0 */

	audio_flush(audio);

	file->private_data = audio;
	audio->opened = 1;
	rc = 0;
done:
	mutex_unlock(&audio->lock);
	return rc;
}

static struct file_operations audio_mp3_fops = {
	.owner		= THIS_MODULE,
	.open		= audio_open,
	.release	= audio_release,
	.read		= audio_read,
	.write		= audio_write,
	.unlocked_ioctl	= audio_ioctl,
};

struct miscdevice audio_mp3_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_mp3",
	.fops	= &audio_mp3_fops,
};

static int __init audio_init(void)
{
	mutex_init(&the_mp3_audio.lock);
	mutex_init(&the_mp3_audio.write_lock);
	spin_lock_init(&the_mp3_audio.dsp_lock);
	init_waitqueue_head(&the_mp3_audio.wait);
	return misc_register(&audio_mp3_misc);
}

device_initcall(audio_init);
