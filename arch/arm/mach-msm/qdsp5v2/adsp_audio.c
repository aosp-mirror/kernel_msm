/* arch/arm/mach-msm/qdsp5v2/adsp_audio.c
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

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>

#include "adsp.h"
#include "adsp_module_afe.h"
#include "adsp_module_audpp.h"
#include "adsp_module_audplay.h"

#include "adsp_audio.h"


#define AUDDEC_DEC_PCM 0

/* Decoder status received from AUDPPTASK */
#define  STATUS_SLEEP	0
#define  STATUS_INIT	1
#define  STATUS_CONFIG	2
#define  STATUS_PLAY	3


#define MAX_AUDPLAY_TASKS 5

struct audplay {
	struct msm_adsp_module *module;
	wait_queue_head_t wait;
	int q1;
	int active;
	int id;
	int status;
	struct audpp *audpp;

	void (*callback)(void *cookie);
	void *cookie;
};

struct audpp {
	struct msm_adsp_module *module;
	wait_queue_head_t wait;
	struct mutex lock;
	int q1, q2, q3;
	unsigned count;
	struct audplay audplay[MAX_AUDPLAY_TASKS];
};

struct afe_info {
	struct msm_adsp_module *module;
	wait_queue_head_t wait;
	struct mutex lock;
	unsigned count;
	u8 active[AFE_DEVICE_ID_MAX + 1];
};

struct afe_info the_afe_info;
static struct audpp the_audpp;


static void afe_callback(unsigned id, void *event, size_t len, void *cookie)
{
	struct afe_info *afe = cookie;
	struct afe_msg_codec_config_ack *msg = event;

	printk("afe_callback id=%d len=%d\n", id, len);
	
	if (id != AFE_MSG_CODEC_CONFIG_ACK)
		return;

	if (msg->device_id > AFE_DEVICE_ID_MAX)
		return;

	if (msg->device_activity == AFE_MSG_CODEC_CONFIG_ENABLED)
		afe->active[msg->device_id] = 1;
	else
		afe->active[msg->device_id] = 0;

	wake_up(&afe->wait);
}

int afe_enable(unsigned device, unsigned rate, unsigned channels)
{
	struct afe_info *afe = &the_afe_info;
	struct afe_cmd_codec_config cmd;
	int ret = 0;

	/* rate must be one of the following:
	 * 8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
	 */
	cmd.cmd_id = AFE_CMD_CODEC_CONFIG_CMD;
	cmd.device_id = device;
	cmd.activity = 1;
	cmd.sample_rate = rate / 1000;
	cmd.channel_mode = channels;
	cmd.volume = AFE_VOLUME_UNITY;
	cmd.reserved = 0;

	mutex_lock(&afe->lock);

	if (!afe->module) {
		ret = msm_adsp_get("AFE", &afe->module, afe_callback, afe);
		if (ret)
			goto done;
	}

	if (afe->active[device]) {
		pr_err("afe_enable: device %d already enabled\n", device);
		ret = -EBUSY;
		goto done;
	}

	if (++afe->count == 1) {
		pr_info("AFE ENABLE!\n");
		ret = msm_adsp_enable(afe->module);
		if (ret < 0) {
			pr_err("afe_enable: cannot enable module\n");
			afe->count--;
			goto done;
		}
	}

	ret = msm_adsp_write(afe->module, 0, &cmd, sizeof(cmd));
	if (ret < 0) {
		printk("afe_enable: command write failed\n");
		goto done;
	}

	ret = wait_event_timeout(afe->wait, afe->active[device], 5 * HZ);
	if (!ret) {
		pr_err("afe_enable: command timeout\n");
		ret = -EIO;
	} else {
		printk("afe_enable: device %d active\n", cmd.device_id);
	}
done:
	mutex_unlock(&afe->lock);
	return ret;
}

int afe_disable(unsigned device)
{
	struct afe_info *afe = &the_afe_info;
	struct afe_cmd_codec_config cmd;
	int ret = 0;

	memset(&cmd, sizeof(cmd), 0);
	cmd.cmd_id = AFE_CMD_CODEC_CONFIG_CMD;
	cmd.device_id = device;
	cmd.activity = 0;

	mutex_lock(&afe->lock);

	if (!afe->active[device]) {
		pr_err("afe_disable: device %d already disabled\n", device);
		goto done;
	}

	ret = msm_adsp_write(afe->module, 0, &cmd, sizeof(cmd));
	if (ret < 0) {
		printk("afe_disable: command write failed\n");
		goto done;
	}

	ret = wait_event_timeout(afe->wait, !afe->active[device], 5 * HZ);
	if (!ret) {
		pr_err("afe_disable: command timeout\n");
		ret = -EIO;
	} else {
		printk("afe_disable: device %d inactive\n", cmd.device_id);
		if (--afe->count == 0) {
			pr_info("AFE DISABLE!\n");
			msm_adsp_disable(afe->module);
		}
	}
done:
	mutex_unlock(&afe->lock);
	return ret;
}

static void audpp_callback(unsigned id, void *event, size_t len, void *cookie)
{
	struct audpp *audpp = cookie;

	if (id == AUDPP_MSG_STATUS_MSG) {
		struct audpp_msg_status_msg *msg = event;
		pr_info("audpp STATUS id=%d status=%d reason=%d\n",
			msg->dec_id, msg->status, msg->reason);
		if (msg->dec_id < MAX_AUDPLAY_TASKS) {
			audpp->audplay[msg->dec_id].status = msg->status;
			wake_up(&audpp->audplay[msg->dec_id].wait);
		}
			
	} else {
		pr_info("audpp cb %d %d\n", id, len);
	}
}

static int audpp_get(struct audpp *audpp)
{
	int ret = 0;

	if (++audpp->count > 1)
		return 0;

	ret = msm_adsp_get("AUDPP", &audpp->module, audpp_callback, audpp);
	if (ret < 0) {
		pr_err("audpp_get: could not get AUDPP\n");
		goto fail_get_module;
	}

	audpp->q1 = msm_adsp_lookup_queue(audpp->module, "AudPPCmd1");
	audpp->q2 = msm_adsp_lookup_queue(audpp->module, "AudPPCmd2");
	audpp->q3 = msm_adsp_lookup_queue(audpp->module, "AudPPCmd3");
	if ((audpp->q1 < 0) || (audpp->q2 < 0) || (audpp->q3 < 0)) {
		pr_err("audpp_get: could not get queues\n");
		ret = -ENODEV;
		goto fail_enable_module;
	}

	ret = msm_adsp_enable(audpp->module);
	if (ret < 0)
		goto fail_enable_module;

	return 0;

fail_enable_module:
	msm_adsp_put(audpp->module);
	audpp->module = NULL;
fail_get_module:
	audpp->count--;
	return ret;
}

static void audpp_put(struct audpp *audpp)
{
	if (--audpp->count > 0)
		return;

	msm_adsp_disable(audpp->module);
	msm_adsp_put(audpp->module);
	audpp->module = NULL;
}


static void audplay_callback(unsigned id, void *event, size_t len, void *cookie)
{
	struct audplay *audplay = cookie;
	if (id == AUDPLAY_MSG_DEC_NEEDS_DATA) {
#if 0
		struct audplay_msg_dec_needs_data *msg = event;
		pr_info("audplay NEEDDATA id=%d off=%d sz=%d %d %d %d %d\n",
			msg->dec_id, msg->adecDataReadPtrOffset,
			msg->adecDataBufSize, msg->bitstream_free_len,
			msg->bitstream_write_ptr, msg->bitstream_buf_start,
			msg->bitstream_buf_len);
#endif
		audplay->callback(audplay->cookie);
	} else {
		pr_info("audplay cb %d %d\n", id, len);
	}
}

struct audplay *audplay_get(void (*cb)(void *cookie), void *cookie)
{
	struct audpp *audpp = &the_audpp;
	struct audplay *audplay = 0;
	char buf[32];
	unsigned n;
	int ret;

	mutex_lock(&audpp->lock);

	for (n = 0; n < MAX_AUDPLAY_TASKS; n++)
		if (audpp->audplay[n].active == 0) break;

	if (n == MAX_AUDPLAY_TASKS)
		goto done;

	if (audpp_get(audpp))
		goto done;

	audplay = audpp->audplay + n;
	sprintf(buf, "AUDPLAY%d", n);
	ret = msm_adsp_get(buf, &audplay->module, audplay_callback, audplay);
	if (ret < 0)
		goto fail_audplay_get;

	sprintf(buf,"AudPlay%dBitStreamCtrl", n);
	audplay->q1 = msm_adsp_lookup_queue(audplay->module, buf);
	if (audplay->q1 < 0)
		goto fail_audplay_enable;

	ret = msm_adsp_enable(audplay->module);
	if (ret < 0)
		goto fail_audplay_enable;

	audplay->active = 1;
	audplay->callback = cb;
	audplay->cookie = cookie;
	goto done;

fail_audplay_enable:
	msm_adsp_put(audplay->module);
	audplay->module = NULL;
	audplay->callback = NULL;
fail_audplay_get:
	audplay = NULL;
	audpp_put(audpp);
done:
	mutex_unlock(&audpp->lock);
	return audplay;
}

void audplay_put(struct audplay *audplay)
{
	mutex_lock(&audplay->audpp->lock);
	audplay->active = 0;
	msm_adsp_disable(audplay->module);
	msm_adsp_put(audplay->module);
	audplay->module = NULL;
	audplay->callback = NULL;
	audpp_put(audplay->audpp);
	mutex_unlock(&audplay->audpp->lock);
}

static void inline audplay_send_q1(struct audplay *audplay, void *cmd, int len) 
{
	msm_adsp_write(audplay->module, audplay->q1, cmd, len);
}

static void inline audpp_send_q1(struct audpp *audpp, void *cmd, int len) 
{
	msm_adsp_write(audpp->module, audpp->q1, cmd, len);
}

static void inline audpp_send_q2(struct audpp *audpp, void *cmd, int len) 
{
	msm_adsp_write(audpp->module, audpp->q2, cmd, len);
}

static void inline audpp_send_q3(struct audpp *audpp, void *cmd, int len) 
{
	msm_adsp_write(audpp->module, audpp->q3, cmd, len);
}


void audplay_config_pcm(struct audplay *audplay,
			unsigned rate, unsigned width, unsigned channels)
{
	struct audpp_cmd_cfg_adec_params_wav cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.common.cmd_id = AUDPP_CMD_CFG_ADEC_PARAMS;
	cmd.common.length = AUDPP_CMD_CFG_ADEC_PARAMS_WAV_LEN >> 1;
	cmd.common.dec_id = audplay->id;
	cmd.common.input_sampling_frequency = rate;
	cmd.stereo_cfg = channels;
	cmd.pcm_width = 1;
	cmd.sign = 0; /* really? */
	audpp_send_q2(audplay->audpp, &cmd, sizeof(cmd)); /* sizeof(cmd)?!*/
}

void audplay_dsp_config(struct audplay *audplay, int enable)
{
	struct audpp_cmd_cfg_dec_type cmd;
	int next;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd_id = AUDPP_CMD_CFG_DEC_TYPE;
	cmd.dec_cfg = AUDPP_CMD_UPDATDE_CFG_DEC;
	if (enable) {
		cmd.dec_cfg |= AUDPP_CMD_ENA_DEC_V | AUDDEC_DEC_PCM;
		next = STATUS_INIT;
	} else {
		cmd.dec_cfg |= AUDPP_CMD_DIS_DEC_V;
		next = STATUS_SLEEP;
	}
	cmd.dm_mode = 0;
	cmd.stream_id = audplay->id;

	mutex_lock(&audplay->audpp->lock);
	audpp_send_q1(audplay->audpp, &cmd, sizeof(cmd));
	wait_event_timeout(audplay->wait, audplay->status == next, 5 * HZ);
	mutex_unlock(&audplay->audpp->lock);
}

void audplay_send_data(struct audplay *audplay, unsigned phys, unsigned len)
{
	struct audplay_cmd_bitstream_data_avail cmd;

	cmd.cmd_id = AUDPLAY_CMD_BITSTREAM_DATA_AVAIL;
	cmd.decoder_id = audplay->id;
	cmd.buf_ptr = phys;
	cmd.buf_size = len/2;
	cmd.partition_number = 0;

	mutex_lock(&audplay->audpp->lock);
	audplay_send_q1(audplay, &cmd, sizeof(cmd));
	wait_event_timeout(audplay->wait, audplay->status == STATUS_PLAY, 5 * HZ);
	mutex_unlock(&audplay->audpp->lock);
}

void audplay_mix_select(struct audplay *audplay, unsigned mix)
{
	struct audpp_cmd_cfg_dev_mixer_params cmd;
	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd_id = AUDPP_CMD_CFG_DEV_MIXER;
	cmd.stream_id = audplay->id;
	cmd.mixer_cmd = mix;
	audpp_send_q1(audplay->audpp, &cmd, sizeof(cmd));
}

void audplay_volume_pan(struct audplay *audplay, unsigned volume, unsigned pan)
{
#define AUDPP_CMD_VOLUME_PAN		0
#define AUDPP_CMD_CFG_OBJ_UPDATE 0x8000
	uint16_t cmd[7];
	cmd[0] = AUDPP_CMD_CFG_OBJECT_PARAMS;
	cmd[1] = AUDPP_CMD_POPP_STREAM;
	cmd[2] = audplay->id;
	cmd[3] = AUDPP_CMD_CFG_OBJ_UPDATE;
	cmd[4] = AUDPP_CMD_VOLUME_PAN;
	cmd[5] = volume;
	cmd[6] = pan;
	audpp_send_q3(audplay->audpp, cmd, sizeof(cmd));
}



void adsp_audio_init(void)
{
	struct afe_info *afe = &the_afe_info;
	struct audpp *audpp = &the_audpp;
	int n;

	mutex_init(&audpp->lock);
	init_waitqueue_head(&audpp->wait);
	for (n = 0; n < MAX_AUDPLAY_TASKS; n++) {
		struct audplay *audplay = audpp->audplay + n;
		audplay->id = n;
		audplay->audpp = audpp;
		init_waitqueue_head(&audplay->wait);
	}

	mutex_init(&afe->lock);
	init_waitqueue_head(&afe->wait);
}
