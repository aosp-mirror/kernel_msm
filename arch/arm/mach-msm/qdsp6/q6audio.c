/* arch/arm/mach-msm/qdsp6/q6audio.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>

#include "../dal.h"
#include "dal_audio.h"
#include "dal_audio_format.h"
#include "dal_acdb.h"
#include "dal_adie.h"
#include <mach/msm_qdsp6_audio.h>

#include <linux/gpio.h>

#include "q6audio_devices.h"

#if 0
#define TRACE(x...) pr_info("Q6: "x)
#else
#define TRACE(x...) do{}while(0)
#endif

static struct q6_hw_info q6_audio_hw[Q6_HW_COUNT] = {
	[Q6_HW_HANDSET] = {
		.min_gain = -2000,
		.max_gain = 0,
	},
	[Q6_HW_HEADSET] = {
		.min_gain = -2000,
		.max_gain = 0,
	},
	[Q6_HW_SPEAKER] = {
		.min_gain = -1500,
		.max_gain = 0,
	},
	[Q6_HW_TTY] = {
		.min_gain = -2000,
		.max_gain = 0,
	},
	[Q6_HW_BT_SCO] = {
		.min_gain = -2000,
		.max_gain = 0,
	},
	[Q6_HW_BT_A2DP] = {
		.min_gain = -2000,
		.max_gain = 0,
	},
};

static struct wake_lock wakelock;
static struct wake_lock idlelock;
static int idlecount;
static DEFINE_MUTEX(idlecount_lock);

void audio_prevent_sleep(void)
{
	mutex_lock(&idlecount_lock);
	if (++idlecount == 1) {
		wake_lock(&wakelock);
		wake_lock(&idlelock);
	}
	mutex_unlock(&idlecount_lock);
}

void audio_allow_sleep(void)
{
	mutex_lock(&idlecount_lock);
	if (--idlecount == 0) {
		wake_unlock(&idlelock);
		wake_unlock(&wakelock);
	}
	mutex_unlock(&idlecount_lock);
}

static struct clk *icodec_rx_clk;
static struct clk *icodec_tx_clk;
static struct clk *ecodec_clk;
static struct clk *sdac_clk;

static struct q6audio_analog_ops default_analog_ops;
static struct q6audio_analog_ops *analog_ops = &default_analog_ops;
static uint32_t tx_clk_freq = 8000;
static int tx_mute_status = 0;
static int rx_vol_level = 100;
static char acdb_file[64] = "default.acdb";
static uint32_t tx_acdb = 0;
static uint32_t rx_acdb = 0;

void q6audio_register_analog_ops(struct q6audio_analog_ops *ops)
{
	analog_ops = ops;
}

void q6audio_set_acdb_file(char* filename)
{
	if (filename)
		strncpy(acdb_file, filename, sizeof(acdb_file)-1);
}

static struct q6_device_info *q6_lookup_device(uint32_t device_id)
{
	struct q6_device_info *di = q6_audio_devices;
	for (;;) {
		if (di->id == device_id)
			return di;
		if (di->id == 0) {
			pr_err("q6_lookup_device: bogus id 0x%08x\n",
			       device_id);
			return di;
		}
		di++;
	}
}

static uint32_t q6_device_to_codec(uint32_t device_id)
{
	struct q6_device_info *di = q6_lookup_device(device_id);
	return di->codec;
}

static uint32_t q6_device_to_dir(uint32_t device_id)
{
	struct q6_device_info *di = q6_lookup_device(device_id);
	return di->dir;
}

static uint32_t q6_device_to_cad_id(uint32_t device_id)
{
	struct q6_device_info *di = q6_lookup_device(device_id);
	return di->cad_id;
}

static uint32_t q6_device_to_path(uint32_t device_id)
{
	struct q6_device_info *di = q6_lookup_device(device_id);
	return di->path;
}

static uint32_t q6_device_to_rate(uint32_t device_id)
{
	struct q6_device_info *di = q6_lookup_device(device_id);
	return di->rate;
}

int q6_device_volume(uint32_t device_id, int level)
{
	struct q6_device_info *di = q6_lookup_device(device_id);
	if (analog_ops->get_rx_vol)
		return analog_ops->get_rx_vol(di->hw, level);
	else {
		struct q6_hw_info *hw;
		hw = &q6_audio_hw[di->hw];
		return hw->min_gain + ((hw->max_gain - hw->min_gain) * level) / 100;
	}
}

static inline int adie_open(struct dal_client *client) 
{
	return dal_call_f0(client, DAL_OP_OPEN, 0);
}

static inline int adie_close(struct dal_client *client) 
{
	return dal_call_f0(client, DAL_OP_CLOSE, 0);
}

static inline int adie_set_path(struct dal_client *client,
				uint32_t id, uint32_t path_type)
{
	return dal_call_f1(client, ADIE_OP_SET_PATH, id, path_type);
}

static inline int adie_set_path_freq_plan(struct dal_client *client,
                                         uint32_t path_type, uint32_t plan)
{
	return dal_call_f1(client, ADIE_OP_SET_PATH_FREQUENCY_PLAN,
			   path_type, plan);
}

static inline int adie_proceed_to_stage(struct dal_client *client,
					uint32_t path_type, uint32_t stage)
{
	return dal_call_f1(client, ADIE_OP_PROCEED_TO_STAGE,
			   path_type, stage);
}

static inline int adie_mute_path(struct dal_client *client,
				 uint32_t path_type, uint32_t mute_state)
{
	return dal_call_f1(client, ADIE_OP_MUTE_PATH, path_type, mute_state);
}

static int adie_refcount;

static struct dal_client *adie;
static struct dal_client *adsp;
static struct dal_client *acdb;

static int adie_enable(void)
{
	adie_refcount++;
	if (adie_refcount == 1)
		adie_open(adie);
	return 0;
}

static int adie_disable(void)
{
	adie_refcount--;
	if (adie_refcount == 0)
		adie_close(adie);
	return 0;
}

/* 4k DMA scratch page used for exchanging acdb device config tables
 * and stream format descriptions with the DSP.
 */
static void *audio_data;
static dma_addr_t audio_phys;

#define SESSION_MIN 0
#define SESSION_MAX 64

static DEFINE_MUTEX(session_lock);
static DEFINE_MUTEX(audio_lock);

static struct audio_client *session[SESSION_MAX];

static int session_alloc(struct audio_client *ac)
{
	int n;

	mutex_lock(&session_lock);
	for (n = SESSION_MIN; n < SESSION_MAX; n++) {
		if (!session[n]) {
			session[n] = ac;
			mutex_unlock(&session_lock);
			return n;
		}
	}
	mutex_unlock(&session_lock);
	return -ENOMEM;
}

static void session_free(int n, struct audio_client *ac)
{
	mutex_lock(&session_lock);
	if (session[n] == ac)
		session[n] = 0;
	mutex_unlock(&session_lock);
}

static void audio_client_free(struct audio_client *ac)
{
	session_free(ac->session, ac);

	if (ac->buf[0].data)
		dma_free_coherent(NULL, ac->buf[0].size,
				  ac->buf[0].data, ac->buf[0].phys);
	if (ac->buf[1].data)
		dma_free_coherent(NULL, ac->buf[1].size,
				  ac->buf[1].data, ac->buf[1].phys);
	kfree(ac);
}

static struct audio_client *audio_client_alloc(unsigned bufsz)
{
	struct audio_client *ac;
	int n;

	ac = kzalloc(sizeof(*ac), GFP_KERNEL);
	if (!ac)
		return 0;

	n = session_alloc(ac);
	if (n < 0)
		goto fail_session;
	ac->session = n;

	if (bufsz > 0) {
		ac->buf[0].data = dma_alloc_coherent(NULL, bufsz,
						&ac->buf[0].phys, GFP_KERNEL);
		if (!ac->buf[0].data)
			goto fail;
		ac->buf[1].data = dma_alloc_coherent(NULL, bufsz,
						&ac->buf[1].phys, GFP_KERNEL);
		if (!ac->buf[1].data)
			goto fail;

		ac->buf[0].size = bufsz;
		ac->buf[1].size = bufsz;
	}

	init_waitqueue_head(&ac->wait);
	ac->client = adsp;

	return ac;

fail:
	session_free(n, ac);
fail_session:
	audio_client_free(ac);
	return 0;
}

void audio_client_dump(struct audio_client *ac)
{
	dal_trace_dump(ac->client);
}

static int audio_ioctl(struct audio_client *ac, void *ptr, uint32_t len)
{
	struct adsp_command_hdr *hdr = ptr;
	uint32_t tmp;
	int r;

	hdr->size = len - sizeof(u32);
	hdr->dst = AUDIO_ADDR(ac->session, 0, AUDIO_DOMAIN_DSP);
	hdr->src = AUDIO_ADDR(ac->session, 0, AUDIO_DOMAIN_MODEM);
	hdr->context = ac->session;
	ac->cb_status = -EBUSY;
	r = dal_call(ac->client, AUDIO_OP_CONTROL, 5, ptr, len, &tmp, sizeof(tmp));
	if (r != 4)
		return -EIO;
	if (!wait_event_timeout(ac->wait, (ac->cb_status != -EBUSY), 5*HZ)) {
		dal_trace_dump(ac->client);
		pr_err("audio_ioctl: timeout. dsp dead?\n");
		q6audio_dsp_not_responding();
	}
	return ac->cb_status;
}

static int audio_command(struct audio_client *ac, uint32_t cmd)
{
	struct adsp_command_hdr rpc;
	memset(&rpc, 0, sizeof(rpc));
	rpc.opcode = cmd;
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

static int audio_open_control(struct audio_client *ac)
{
	struct adsp_open_command rpc;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_OPEN_DEVICE;
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

static int audio_out_open(struct audio_client *ac, uint32_t bufsz,
			  uint32_t rate, uint32_t channels)
{
	struct adsp_open_command rpc;

	memset(&rpc, 0, sizeof(rpc));

	rpc.format.standard.format = ADSP_AUDIO_FORMAT_PCM;
	rpc.format.standard.channels = channels;
	rpc.format.standard.bits_per_sample = 16;
	rpc.format.standard.sampling_rate = rate;
	rpc.format.standard.is_signed = 1;
	rpc.format.standard.is_interleaved = 1;

	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_OPEN_WRITE;
	rpc.device = ADSP_AUDIO_DEVICE_ID_DEFAULT;
	rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_PLAYBACK;
	rpc.buf_max_size = bufsz;

	TRACE("open out %p\n", ac);
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

static int audio_in_open(struct audio_client *ac, uint32_t bufsz,
			 uint32_t rate, uint32_t channels)
{
	struct adsp_open_command rpc;

	memset(&rpc, 0, sizeof(rpc));

	rpc.format.standard.format = ADSP_AUDIO_FORMAT_PCM;
	rpc.format.standard.channels = channels;
	rpc.format.standard.bits_per_sample = 16;
	rpc.format.standard.sampling_rate = rate;
	rpc.format.standard.is_signed = 1;
	rpc.format.standard.is_interleaved = 1;

	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_OPEN_READ;
	rpc.device = ADSP_AUDIO_DEVICE_ID_DEFAULT;
	rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_RECORD;
	rpc.buf_max_size = bufsz;

	TRACE("%p: open in\n", ac);
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

static int audio_mp3_open(struct audio_client *ac, uint32_t bufsz,
			  uint32_t rate, uint32_t channels)
{
	struct adsp_open_command rpc;

	memset(&rpc, 0, sizeof(rpc));

	rpc.format.standard.format = ADSP_AUDIO_FORMAT_MP3;
	rpc.format.standard.channels = channels;
	rpc.format.standard.bits_per_sample = 16;
	rpc.format.standard.sampling_rate = rate;
	rpc.format.standard.is_signed = 1;
	rpc.format.standard.is_interleaved = 0;

	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_OPEN_WRITE;
	rpc.device = ADSP_AUDIO_DEVICE_ID_DEFAULT;
	rpc.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_PLAYBACK;
	rpc.buf_max_size = bufsz;

	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

static int audio_close(struct audio_client *ac)
{
	TRACE("%p: close\n", ac);
	audio_command(ac, ADSP_AUDIO_IOCTL_CMD_STREAM_STOP);
	audio_command(ac, ADSP_AUDIO_IOCTL_CMD_CLOSE);
	return 0;
}

static int audio_set_table(struct audio_client *ac,
			   uint32_t device_id, int size)
{
	struct adsp_set_dev_cfg_table_command rpc;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_SET_DEVICE_CONFIG_TABLE;
	if (q6_device_to_dir(device_id) == Q6_TX)
		rpc.hdr.data = tx_clk_freq;
	rpc.device_id = device_id;
	rpc.phys_addr = audio_phys;
	rpc.phys_size = size;
	rpc.phys_used = size;

	TRACE("control: set table %x\n", device_id);
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

int q6audio_read(struct audio_client *ac, struct audio_buffer *ab)
{
	struct adsp_buffer_command rpc;
	uint32_t res;
	int r;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.size = sizeof(rpc) - sizeof(u32);
	rpc.hdr.dst = AUDIO_ADDR(ac->session, 0, AUDIO_DOMAIN_DSP);
	rpc.hdr.src = AUDIO_ADDR(ac->session, 0, AUDIO_DOMAIN_MODEM);
	rpc.hdr.context = ac->session;
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_DATA_TX;
	rpc.buffer.addr = ab->phys;
	rpc.buffer.max_size = ab->size;
	rpc.buffer.actual_size = ab->used;

	TRACE("%p: read\n", ac);
	r = dal_call(ac->client, AUDIO_OP_DATA, 5, &rpc, sizeof(rpc),
		     &res, sizeof(res));
	return 0;
}

int q6audio_write(struct audio_client *ac, struct audio_buffer *ab)
{
	struct adsp_buffer_command rpc;
	uint32_t res;
	int r;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.size = sizeof(rpc) - sizeof(u32);
	rpc.hdr.dst = AUDIO_ADDR(ac->session, 0, AUDIO_DOMAIN_DSP);
	rpc.hdr.src = AUDIO_ADDR(ac->session, 0, AUDIO_DOMAIN_MODEM);
	rpc.hdr.context = ac->session;
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_DATA_RX;
	rpc.buffer.addr = ab->phys;
	rpc.buffer.max_size = ab->size;
	rpc.buffer.actual_size = ab->used;

	TRACE("%p: write\n", ac);
	r = dal_call(ac->client, AUDIO_OP_DATA, 5, &rpc, sizeof(rpc),
		     &res, sizeof(res));
	return 0;
}

static int audio_rx_volume(struct audio_client *ac, uint32_t dev_id, int32_t volume)
{
	struct adsp_set_dev_volume_command rpc;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL;
	rpc.device_id = dev_id;
	rpc.path = ADSP_PATH_RX;
	rpc.volume = volume;
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

static int audio_rx_mute(struct audio_client *ac, uint32_t dev_id, int mute)
{
	struct adsp_set_dev_mute_command rpc;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE;
	rpc.device_id = dev_id;
	rpc.path = ADSP_PATH_RX;
	rpc.mute = !!mute;
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

static int audio_tx_volume(struct audio_client *ac, uint32_t dev_id, int32_t volume)
{
	struct adsp_set_dev_volume_command rpc;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL;
	rpc.device_id = dev_id;
	rpc.path = ADSP_PATH_TX;
	rpc.volume = volume;
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

static int audio_tx_mute(struct audio_client *ac, uint32_t dev_id, int mute)
{
	struct adsp_set_dev_mute_command rpc;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE;
	rpc.device_id = dev_id;
	rpc.path = ADSP_PATH_TX;
	rpc.mute = !!mute;
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

static int audio_stream_volume(struct audio_client *ac, int volume)
{
	struct adsp_set_volume_command rpc;
	int rc;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_SET_STREAM_VOL;
	rpc.volume = volume;
	rc = audio_ioctl(ac, &rpc, sizeof(rpc));
	return rc;
}

static int audio_stream_mute(struct audio_client *ac, int mute)
{
	struct adsp_set_mute_command rpc;
	int rc;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_SET_STREAM_MUTE;
	rpc.mute = mute;
	rc = audio_ioctl(ac, &rpc, sizeof(rpc));
	return rc;
}

static void callback(void *data, int len, void *cookie)
{
	struct adsp_event_hdr *e = data;
	struct audio_client *ac;


	if (e->context >= SESSION_MAX) {
		pr_err("audio callback: bogus session %d\n",
		       e->context);
		return;
	}
	ac = session[e->context];
	if (!ac) {
		pr_err("audio callback: unknown session %d\n",
		       e->context);
		return;
	}

	if (e->event_id == ADSP_AUDIO_IOCTL_CMD_STREAM_EOS) {
		TRACE("%p: CB stream eos\n", ac);
		if (e->status)
			pr_err("playback status %d\n", e->status);
		if (ac->cb_status == -EBUSY) {
			ac->cb_status = e->status;
			wake_up(&ac->wait);
		}
		return;
	}

	if (e->event_id == ADSP_AUDIO_EVT_STATUS_BUF_DONE) {
		TRACE("%p: CB done (%d)\n", ac, e->status);
		if (e->status)
			pr_err("buffer status %d\n", e->status);
		ac->buf[ac->dsp_buf].used = 0;
		ac->dsp_buf ^= 1;
		wake_up(&ac->wait);
		return;
	}

	TRACE("%p: CB %08x status %d\n", ac, e->event_id, e->status);
	if (e->status)
		pr_warning("audio_cb: s=%d e=%08x status=%d\n",
			   e->context, e->event_id, e->status);
	if (ac->cb_status == -EBUSY) {
		ac->cb_status = e->status;
		wake_up(&ac->wait);
	}
}

static void audio_init(struct dal_client *client)
{
	u32 tmp[3];

	tmp[0] = 2 * sizeof(u32);
	tmp[1] = 1;
	tmp[2] = 0;
	dal_call(client, AUDIO_OP_INIT, 5, tmp, sizeof(tmp),
		 tmp, sizeof(u32));
}

static struct audio_client *ac_control;

static int q6audio_init(void)
{
	struct audio_client *ac = 0;
	int res;

	mutex_lock(&audio_lock);
	if (ac_control) {
		res = 0;
		goto done;
	}

	pr_info("audio: init: codecs\n");
	icodec_rx_clk = clk_get(0, "icodec_rx_clk");
	icodec_tx_clk = clk_get(0, "icodec_tx_clk");
	ecodec_clk = clk_get(0, "ecodec_clk");
	sdac_clk = clk_get(0, "sdac_clk");
	audio_data = dma_alloc_coherent(NULL, 4096, &audio_phys, GFP_KERNEL);

	adsp = dal_attach(AUDIO_DAL_DEVICE, AUDIO_DAL_PORT,
			  callback, 0);
	if (!adsp) {
		pr_err("audio_init: cannot attach to adsp\n");
		res = -ENODEV;
		goto done;
	}
	pr_info("audio: init: INIT\n");
	audio_init(adsp);
	dal_trace(adsp);

	ac = audio_client_alloc(0);
	if (!ac) {
		pr_err("audio_init: cannot allocate client\n");
		res = -ENOMEM;
		goto done;
	}

	pr_info("audio: init: OPEN control\n");
	if (audio_open_control(ac)) {
		pr_err("audio_init: cannot open control channel\n");
		res = -ENODEV;
		goto done;
	}

	pr_info("audio: init: attach ACDB\n");
	acdb = dal_attach(ACDB_DAL_DEVICE, ACDB_DAL_PORT, 0, 0);
	if (!acdb) {
		pr_err("audio_init: cannot attach to acdb channel\n");
		res = -ENODEV;
		goto done;
	}

	pr_info("audio: init: attach ADIE\n");
	adie = dal_attach(ADIE_DAL_DEVICE, ADIE_DAL_PORT, 0, 0);
	if (!adie) {
		pr_err("audio_init: cannot attach to adie\n");
		res = -ENODEV;
		goto done;
	}
	if (analog_ops->init)
		analog_ops->init();

	res = 0;
	ac_control = ac;

	wake_lock_init(&idlelock, WAKE_LOCK_IDLE, "audio_pcm_idle");
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "audio_pcm_suspend");
done:
	if ((res < 0) && ac)
		audio_client_free(ac);
	mutex_unlock(&audio_lock);

	return res;
}

struct audio_config_data {
	uint32_t device_id;
	uint32_t sample_rate;
	uint32_t offset;
	uint32_t length;
};

struct audio_config_database {
	uint8_t magic[8];
	uint32_t entry_count;
	uint32_t unused;
	struct audio_config_data entry[0];
};

void *acdb_data;
const struct firmware *acdb_fw;
extern struct miscdevice q6_control_device;

static int acdb_init(char *filename)
{
	const struct audio_config_database *db;
	const struct firmware *fw;
	int n;

	pr_info("acdb: load '%s'\n", filename);
	if (request_firmware(&fw, filename, q6_control_device.this_device) < 0) {
		pr_err("acdb: load 'default.acdb' failed...\n");
		return -ENODEV;
	}
	db = (void*) fw->data;

	if (fw->size < sizeof(struct audio_config_database)) {
		pr_err("acdb: undersized database\n");
		goto fail;
	}
	if (strcmp(db->magic, "ACDB1.0")) {
		pr_err("acdb: invalid magic\n");
		goto fail;
	}
	if (db->entry_count > 1024) {
		pr_err("acdb: too many entries\n");
		goto fail;
	}
	if (fw->size < (sizeof(struct audio_config_database) +
			db->entry_count * sizeof(struct audio_config_data))) {
		pr_err("acdb: undersized TOC\n");
		goto fail;
	}
	for (n = 0; n < db->entry_count; n++) {
		if (db->entry[n].length > 4096) {
			pr_err("acdb: entry %d too large (%d)\n",
			       n, db->entry[n].length);
			goto fail;
		}
		if ((db->entry[n].offset + db->entry[n].length) > fw->size) {
			pr_err("acdb: entry %d outside of data\n", n);
			goto fail;
		}
	}
	if (acdb_data)
		release_firmware(acdb_fw);
	acdb_data = (void*) fw->data;
	acdb_fw = fw;
	return 0;
fail:
	release_firmware(fw);
	return -ENODEV;
}

static int acdb_get_config_table(uint32_t device_id, uint32_t sample_rate)
{
	struct audio_config_database *db;
	int n, res;

	if (q6audio_init())
		return 0;

	if (!acdb_data) {
		res = acdb_init(acdb_file);
		if (res)
			return res;
	}

	db = acdb_data;
	for (n = 0; n < db->entry_count; n++) {
		if (db->entry[n].device_id != device_id)
			continue;
		if (db->entry[n].sample_rate != sample_rate)
			continue;
		break;
	}

	if (n == db->entry_count) {
		pr_err("acdb: no entry for device %d, rate %d.\n",
		       device_id, sample_rate);
		return 0;
	}

	pr_info("acdb: %d bytes for device %d, rate %d.\n",
		db->entry[n].length, device_id, sample_rate);

	memcpy(audio_data, acdb_data + db->entry[n].offset, db->entry[n].length);
	return db->entry[n].length;
}

static uint32_t audio_rx_path_id = ADIE_PATH_HANDSET_RX;
static uint32_t audio_rx_device_id = ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR;
static uint32_t audio_rx_device_group = -1;
static uint32_t audio_tx_path_id = ADIE_PATH_HANDSET_TX;
static uint32_t audio_tx_device_id = ADSP_AUDIO_DEVICE_ID_HANDSET_MIC;
static uint32_t audio_tx_device_group = -1;

static int qdsp6_devchg_notify(struct audio_client *ac,
			       uint32_t dev_type, uint32_t dev_id)
{
	struct adsp_device_switch_command rpc;

	if (dev_type != ADSP_AUDIO_RX_DEVICE &&
	    dev_type != ADSP_AUDIO_TX_DEVICE)
		return -EINVAL;

	memset(&rpc, 0, sizeof(rpc));
	rpc.hdr.opcode = ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_PREPARE;
	if (dev_type == ADSP_AUDIO_RX_DEVICE) {
		rpc.old_device = audio_rx_device_id;
		rpc.new_device = dev_id;
	} else {
		rpc.old_device = audio_tx_device_id;
		rpc.new_device = dev_id;
	}
	rpc.device_class = 0;
	rpc.device_type = dev_type;
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}

static int qdsp6_standby(struct audio_client *ac)
{
	return audio_command(ac, ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_STANDBY);
}

static int qdsp6_start(struct audio_client *ac)
{
	return audio_command(ac, ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_COMMIT);
}

static void audio_rx_analog_enable(int en)
{
	switch (audio_rx_device_id) {
	case ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_MONO:
	case ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_STEREO:
	case ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_SPKR:
		if (analog_ops->headset_enable)
			analog_ops->headset_enable(en);
		break;
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_MONO_HEADSET:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_STEREO_HEADSET:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_MONO_HEADSET:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_STEREO_HEADSET:
		if (analog_ops->headset_enable)
			analog_ops->headset_enable(en);
		if (analog_ops->speaker_enable)
			analog_ops->speaker_enable(en);
		break;
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO:
		if (analog_ops->speaker_enable)
			analog_ops->speaker_enable(en);
		break;
	case ADSP_AUDIO_DEVICE_ID_BT_SCO_SPKR:
		if (analog_ops->bt_sco_enable)
			analog_ops->bt_sco_enable(en);
		break;
	case ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR:
		if (analog_ops->receiver_enable)
			analog_ops->receiver_enable(en);
		break;
	}
}

static void audio_tx_analog_enable(int en)
{
	switch (audio_tx_device_id) {
	case ADSP_AUDIO_DEVICE_ID_HANDSET_MIC:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MIC:
		if (analog_ops->int_mic_enable)
			analog_ops->int_mic_enable(en);
		break;
	case ADSP_AUDIO_DEVICE_ID_HEADSET_MIC:
	case ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_MIC:
		if (analog_ops->ext_mic_enable)
			analog_ops->ext_mic_enable(en);
		break;
	case ADSP_AUDIO_DEVICE_ID_BT_SCO_MIC:
		if (analog_ops->bt_sco_enable)
			analog_ops->bt_sco_enable(en);
		break;
	}
}

static int audio_update_acdb(uint32_t adev, uint32_t acdb_id)
{
	uint32_t sample_rate;
	int sz = -1;

	sample_rate = q6_device_to_rate(adev);

	if (q6_device_to_dir(adev) == Q6_RX)
		rx_acdb = acdb_id;
	else
		tx_acdb = acdb_id;

	if (acdb_id != 0)
		sz = acdb_get_config_table(acdb_id, sample_rate);

	if (sz <= 0) {
		acdb_id = q6_device_to_cad_id(adev);
		sz = acdb_get_config_table(acdb_id, sample_rate);
		if (sz <= 0)
			return -EINVAL;
	}

	audio_set_table(ac_control, adev, sz);
	return 0;
}

static void _audio_rx_path_enable(int reconf, uint32_t acdb_id)
{
	adie_enable();
	adie_set_path(adie, audio_rx_path_id, ADIE_PATH_RX);
	adie_set_path_freq_plan(adie, ADIE_PATH_RX, 48000);

	adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_READY);
	adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_ANALOG_READY);

	audio_update_acdb(audio_rx_device_id, acdb_id);
	if (!reconf)
		qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, audio_rx_device_id);
	qdsp6_standby(ac_control);
	qdsp6_start(ac_control);

	audio_rx_analog_enable(1);
}

static void _audio_tx_path_enable(int reconf, uint32_t acdb_id)
{
	audio_tx_analog_enable(1);

	adie_enable();
	adie_set_path(adie, audio_tx_path_id, ADIE_PATH_TX);

	if (tx_clk_freq > 8000)
		adie_set_path_freq_plan(adie, ADIE_PATH_TX, 48000);
	else
		adie_set_path_freq_plan(adie, ADIE_PATH_TX, 8000);

	adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_DIGITAL_READY);
	adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_DIGITAL_ANALOG_READY);

	audio_update_acdb(audio_tx_device_id, acdb_id);

	if (!reconf)
		qdsp6_devchg_notify(ac_control, ADSP_AUDIO_TX_DEVICE, audio_tx_device_id);
	qdsp6_standby(ac_control);
	qdsp6_start(ac_control);

	audio_tx_mute(ac_control, audio_tx_device_id, tx_mute_status);
}

static void _audio_rx_path_disable(void)
{
	audio_rx_analog_enable(0);

	adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_ANALOG_OFF);
	adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_OFF);
	adie_disable();
}

static void _audio_tx_path_disable(void)
{
	audio_tx_analog_enable(0);

	adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_ANALOG_OFF);
	adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_DIGITAL_OFF);
	adie_disable();
}

static int icodec_rx_clk_refcount;
static int icodec_tx_clk_refcount;
static int ecodec_clk_refcount;
static int sdac_clk_refcount;

static void _audio_rx_clk_enable(void)
{
	uint32_t device_group = q6_device_to_codec(audio_rx_device_id);

	switch(device_group) {
	case Q6_ICODEC_RX:
		icodec_rx_clk_refcount++;
		if (icodec_rx_clk_refcount == 1) {
			clk_set_rate(icodec_rx_clk, 12288000);
			clk_enable(icodec_rx_clk);
		}
		break;
	case Q6_ECODEC_RX:
		ecodec_clk_refcount++;
		if (ecodec_clk_refcount == 1) {
			clk_set_rate(ecodec_clk, 2048000);
			clk_enable(ecodec_clk);
		}
		break;
	case Q6_SDAC_RX:
		sdac_clk_refcount++;
		if (sdac_clk_refcount == 1) {
			clk_set_rate(sdac_clk, 12288000);
			clk_enable(sdac_clk);
		}
		break;
	default:
		return;
	}
	audio_rx_device_group = device_group;
}

static void _audio_tx_clk_enable(void)
{
	uint32_t device_group = q6_device_to_codec(audio_tx_device_id);

	switch (device_group) {
	case Q6_ICODEC_TX:
		icodec_tx_clk_refcount++;
		if (icodec_tx_clk_refcount == 1) {
			clk_set_rate(icodec_tx_clk, tx_clk_freq * 256);
			clk_enable(icodec_tx_clk);
		}
		break;
	case Q6_ECODEC_TX:
		ecodec_clk_refcount++;
		if (ecodec_clk_refcount == 1) {
			clk_set_rate(ecodec_clk, 2048000);
			clk_enable(ecodec_clk);
		}
		break;
	case Q6_SDAC_TX:
		/* TODO: In QCT BSP, clk rate was set to 20480000 */
		sdac_clk_refcount++;
		if (sdac_clk_refcount == 1) {
			clk_set_rate(sdac_clk, 12288000);
			clk_enable(sdac_clk);
		}
		break;
	default:
		return;
	}
	audio_tx_device_group = device_group;
}

static void _audio_rx_clk_disable(void)
{
	switch (audio_rx_device_group) {
	case Q6_ICODEC_RX:
		icodec_rx_clk_refcount--;
		if (icodec_rx_clk_refcount == 0) {
			clk_disable(icodec_rx_clk);
			audio_rx_device_group = -1;
		}
		break;
	case Q6_ECODEC_RX:
		ecodec_clk_refcount--;
		if (ecodec_clk_refcount == 0) {
			clk_disable(ecodec_clk);
			audio_rx_device_group = -1;
		}
		break;
	case Q6_SDAC_RX:
		sdac_clk_refcount--;
		if (sdac_clk_refcount == 0) {
			clk_disable(sdac_clk);
			audio_rx_device_group = -1;
		}
		break;
	default:
		pr_err("audiolib: invalid rx device group %d\n",
			audio_rx_device_group);
		break;
	}
}

static void _audio_tx_clk_disable(void)
{
	switch (audio_tx_device_group) {
	case Q6_ICODEC_TX:
		icodec_tx_clk_refcount--;
		if (icodec_tx_clk_refcount == 0) {
			clk_disable(icodec_tx_clk);
			audio_tx_device_group = -1;
		}
		break;
	case Q6_ECODEC_TX:
		ecodec_clk_refcount--;
		if (ecodec_clk_refcount == 0) {
			clk_disable(ecodec_clk);
			audio_tx_device_group = -1;
		}
		break;
	case Q6_SDAC_TX:
		sdac_clk_refcount--;
		if (sdac_clk_refcount == 0) {
			clk_disable(sdac_clk);
			audio_tx_device_group = -1;
		}
		break;
	default:
		pr_err("audiolib: invalid tx device group %d\n",
			audio_tx_device_group);
		break;
	}
}

static void _audio_rx_clk_reinit(uint32_t rx_device)
{
	uint32_t device_group = q6_device_to_codec(rx_device);

	if (device_group != audio_rx_device_group)
		_audio_rx_clk_disable();

	audio_rx_device_id = rx_device;
	audio_rx_path_id = q6_device_to_path(rx_device);

	if (device_group != audio_rx_device_group)
		_audio_rx_clk_enable();

}

static void _audio_tx_clk_reinit(uint32_t tx_device)
{
	uint32_t device_group = q6_device_to_codec(tx_device);

	if (device_group != audio_tx_device_group)
		_audio_tx_clk_disable();

	audio_tx_device_id = tx_device;
	audio_tx_path_id = q6_device_to_path(tx_device);

	if (device_group != audio_tx_device_group)
		_audio_tx_clk_enable();
}

static DEFINE_MUTEX(audio_path_lock);
static int audio_rx_path_refcount;
static int audio_tx_path_refcount;

static int audio_rx_path_enable(int en, uint32_t acdb_id)
{
	mutex_lock(&audio_path_lock);
	if (en) {
		audio_rx_path_refcount++;
		if (audio_rx_path_refcount == 1) {
			_audio_rx_clk_enable();
			_audio_rx_path_enable(0, acdb_id);
		}
	} else {
		audio_rx_path_refcount--;
		if (audio_rx_path_refcount == 0) {
			_audio_rx_path_disable();
			_audio_rx_clk_disable();
		}
	}
	mutex_unlock(&audio_path_lock);
	return 0;
}

static int audio_tx_path_enable(int en, uint32_t acdb_id)
{
	mutex_lock(&audio_path_lock);
	if (en) {
		audio_tx_path_refcount++;
		if (audio_tx_path_refcount == 1) {
			_audio_tx_clk_enable();
			_audio_tx_path_enable(0, acdb_id);
		}
	} else {
		audio_tx_path_refcount--;
		if (audio_tx_path_refcount == 0) {
			_audio_tx_path_disable();
			_audio_tx_clk_disable();
		}
	}
	mutex_unlock(&audio_path_lock);
	return 0;
}

int q6audio_reinit_acdb(char* filename) {
	int res;

	if (q6audio_init())
		return 0;

	mutex_lock(&audio_path_lock);
	if (strlen(filename) < 0 || !strcmp(filename, acdb_file)) {
		res = -EINVAL;
		goto done;
	}
	res = acdb_init(filename);
	if (!res)
		strcpy(acdb_file, filename);
done:
	mutex_unlock(&audio_path_lock);
	return res;

}

int q6audio_update_acdb(uint32_t id_src, uint32_t id_dst)
{
	int res;

	if (q6audio_init())
		return 0;

	mutex_lock(&audio_path_lock);
	res = audio_update_acdb(id_dst, id_src);
	if (res)
		goto done;

	if (q6_device_to_dir(id_dst) == Q6_RX)
		qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, id_dst);
	else
		qdsp6_devchg_notify(ac_control, ADSP_AUDIO_TX_DEVICE, id_dst);
	qdsp6_standby(ac_control);
	qdsp6_start(ac_control);
done:
	mutex_unlock(&audio_path_lock);
	return res;
}

int q6audio_set_tx_mute(int mute)
{
	uint32_t adev;

	if (q6audio_init())
		return 0;

	mutex_lock(&audio_path_lock);

	if (mute == tx_mute_status) {
		mutex_unlock(&audio_path_lock);
		return 0;
	}

	adev = audio_tx_device_id;
	audio_tx_mute(ac_control, adev, mute);
	tx_mute_status = mute;
	mutex_unlock(&audio_path_lock);
	return 0;
}

int q6audio_set_stream_volume(struct audio_client *ac, int vol)
{
	if (vol > 1200 || vol < -4000) {
		pr_err("unsupported volume level %d\n", vol);
		return -EINVAL;
	}
	mutex_lock(&audio_path_lock);
	audio_stream_mute(ac, 0);
	audio_stream_volume(ac, vol);
	mutex_unlock(&audio_path_lock);
	return 0;
}

int q6audio_set_rx_volume(int level)
{
	uint32_t adev;
	int vol;

	if (q6audio_init())
		return 0;

	if (level < 0 || level > 100)
		return -EINVAL;

	mutex_lock(&audio_path_lock);
	adev = ADSP_AUDIO_DEVICE_ID_VOICE;
	vol = q6_device_volume(audio_rx_device_id, level);
	audio_rx_mute(ac_control, adev, 0);
	audio_rx_volume(ac_control, adev, vol);
	rx_vol_level = level;
	mutex_unlock(&audio_path_lock);
	return 0;
}

static void do_rx_routing(uint32_t device_id, uint32_t acdb_id)
{
	if (device_id == audio_rx_device_id) {
		if (acdb_id != rx_acdb) {
			audio_update_acdb(device_id, acdb_id);
			qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, device_id);
			qdsp6_standby(ac_control);
			qdsp6_start(ac_control);
		}
		return;
	}

	if (audio_rx_path_refcount > 0) {
		qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, device_id);
		_audio_rx_path_disable();
		_audio_rx_clk_reinit(device_id);
		_audio_rx_path_enable(1, acdb_id);
	} else {
		audio_rx_device_id = device_id;
		audio_rx_path_id = q6_device_to_path(device_id);
	}
}

static void do_tx_routing(uint32_t device_id, uint32_t acdb_id)
{
	if (device_id == audio_tx_device_id) {
		if (acdb_id != tx_acdb) {
			audio_update_acdb(device_id, acdb_id);
			qdsp6_devchg_notify(ac_control, ADSP_AUDIO_TX_DEVICE, device_id);
			qdsp6_standby(ac_control);
			qdsp6_start(ac_control);
		}
		return;
	}

	if (audio_tx_path_refcount > 0) {
		qdsp6_devchg_notify(ac_control, ADSP_AUDIO_TX_DEVICE, device_id);
		_audio_tx_path_disable();
		_audio_tx_clk_reinit(device_id);
		_audio_tx_path_enable(1, acdb_id);
	} else {
		audio_tx_device_id = device_id;
		audio_tx_path_id = q6_device_to_path(device_id);
	}
}

int q6audio_do_routing(uint32_t device_id, uint32_t acdb_id)
{
	if (q6audio_init())
		return 0;

	mutex_lock(&audio_path_lock);

	switch(q6_device_to_dir(device_id)) {
	case Q6_RX:
		do_rx_routing(device_id, acdb_id);
		break;
	case Q6_TX:
		do_tx_routing(device_id, acdb_id);
		break;
	}

	mutex_unlock(&audio_path_lock);
	return 0;
}

int q6audio_set_route(const char *name)
{
	uint32_t route;
	if (!strcmp(name, "speaker")) {
		route = ADIE_PATH_SPEAKER_STEREO_RX;
	} else if (!strcmp(name, "headphones")) {
		route = ADIE_PATH_HEADSET_STEREO_RX;
	} else if (!strcmp(name, "handset")) {
		route = ADIE_PATH_HANDSET_RX;
	} else {
		return -EINVAL;
	}

	mutex_lock(&audio_path_lock);
	if (route == audio_rx_path_id)
		goto done;

	audio_rx_path_id = route;

	if (audio_rx_path_refcount > 0) {
		_audio_rx_path_disable();
		_audio_rx_path_enable(1, 0);
	}
	if (audio_tx_path_refcount > 0) {
		_audio_tx_path_disable();
		_audio_tx_path_enable(1, 0);
	}
done:
	mutex_unlock(&audio_path_lock);
	return 0;
}

struct audio_client *q6audio_open_pcm(uint32_t bufsz, uint32_t rate,
				      uint32_t channels, uint32_t flags, uint32_t acdb_id)
{
	int rc, retry = 5;
	struct audio_client *ac;

	if (q6audio_init())
		return 0;

	ac = audio_client_alloc(bufsz);
	if (!ac)
		return 0;

	ac->flags = flags;

	mutex_lock(&audio_path_lock);

	if (ac->flags & AUDIO_FLAG_WRITE) {
		audio_rx_path_refcount++;
		if (audio_rx_path_refcount == 1) {
			_audio_rx_clk_enable();
			audio_update_acdb(audio_rx_device_id, acdb_id);
			qdsp6_devchg_notify(ac_control, ADSP_AUDIO_RX_DEVICE, audio_rx_device_id);
			qdsp6_standby(ac_control);
			qdsp6_start(ac_control);
		}
	} else {
		/* TODO: consider concurrency with voice call */
		tx_clk_freq = rate;
		audio_tx_path_refcount++;
		if (audio_tx_path_refcount == 1) {
			_audio_tx_clk_enable();
			_audio_tx_path_enable(0, acdb_id);
		}
	}

	for (retry = 5;;retry--) {
		if (ac->flags & AUDIO_FLAG_WRITE)
			rc = audio_out_open(ac, bufsz, rate, channels);
		else
			rc = audio_in_open(ac, bufsz, rate, channels);
		if (rc == 0)
			break;
		if (retry == 0)
			q6audio_dsp_not_responding();
		pr_err("q6audio: open pcm error %d, retrying\n", rc);
		msleep(1);
	}

	if (ac->flags & AUDIO_FLAG_WRITE) {
		if (audio_rx_path_refcount == 1) {
			adie_enable();
			adie_set_path(adie, audio_rx_path_id, ADIE_PATH_RX);
			adie_set_path_freq_plan(adie, ADIE_PATH_RX, 48000);

			adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_READY);
			adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_ANALOG_READY);

			audio_rx_analog_enable(1);
		}
	}

	mutex_unlock(&audio_path_lock);

	for (retry = 5;;retry--) {
		rc = audio_command(ac, ADSP_AUDIO_IOCTL_CMD_SESSION_START);
		if (rc == 0)
			break;
		if (retry == 0)
			q6audio_dsp_not_responding();
		pr_err("q6audio: stream start error %d, retrying\n", rc);
	}

	if (!(ac->flags & AUDIO_FLAG_WRITE)) {
		ac->buf[0].used = 1;
		ac->buf[1].used = 1;
		q6audio_read(ac, &ac->buf[0]);
		q6audio_read(ac, &ac->buf[1]);
	}

	audio_prevent_sleep();
	return ac;
}

int q6audio_close(struct audio_client *ac)
{
	audio_close(ac);
	if (ac->flags & AUDIO_FLAG_WRITE)
		audio_rx_path_enable(0, 0);
	else
		audio_tx_path_enable(0, 0);

	audio_client_free(ac);
	audio_allow_sleep();
	return 0;
}

struct audio_client *q6voice_open(uint32_t flags, uint32_t acdb_id)
{
	struct audio_client *ac;

	if (q6audio_init())
		return 0;

	ac = audio_client_alloc(0);
	if (!ac)
		return 0;

	ac->flags = flags;
	if (ac->flags & AUDIO_FLAG_WRITE)
		audio_rx_path_enable(1, acdb_id);
	else {
		tx_clk_freq = 8000;
		audio_tx_path_enable(1, acdb_id);
	}

	return ac;
}

int q6voice_close(struct audio_client *ac)
{
	if (ac->flags & AUDIO_FLAG_WRITE)
		audio_rx_path_enable(0, 0);
	else
		audio_tx_path_enable(0, 0);

	audio_client_free(ac);
	return 0;
}

struct audio_client *q6audio_open_mp3(uint32_t bufsz, uint32_t rate,
				      uint32_t channels, uint32_t acdb_id)
{
	struct audio_client *ac;

	printk("q6audio_open_mp3()\n");

	if (q6audio_init())
		return 0;

	ac = audio_client_alloc(bufsz);
	if (!ac)
		return 0;

	ac->flags = AUDIO_FLAG_WRITE;
	audio_rx_path_enable(1, acdb_id);

	audio_mp3_open(ac, bufsz, rate, channels);
	audio_command(ac, ADSP_AUDIO_IOCTL_CMD_SESSION_START);

	return ac;
}

int q6audio_mp3_close(struct audio_client *ac)
{
	audio_close(ac);
	audio_rx_path_enable(0, 0);
	audio_client_free(ac);
	return 0;
}

int q6audio_async(struct audio_client *ac)
{
	struct adsp_command_hdr rpc;
	memset(&rpc, 0, sizeof(rpc));
	rpc.opcode = ADSP_AUDIO_IOCTL_CMD_STREAM_EOS;
	rpc.response_type = ADSP_AUDIO_RESPONSE_ASYNC;
	return audio_ioctl(ac, &rpc, sizeof(rpc));
}
