/* arch/arm/mach-msm/qdsp5v2/adsp_audio.h
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

#ifndef _MSM_ADSP_AUDIO_H_
#define _MSM_ADSP_AUDIO_H_

struct audplay;
struct audpp;

struct audplay *audplay_get(void (*cb)(void *cookie), void *cookie);
void audplay_put(struct audplay *audplay);

void audplay_send_data(struct audplay *audplay, unsigned phys, unsigned len);

void audplay_dsp_config(struct audplay *audplay, int enable);
void audplay_config_pcm(struct audplay *audplay,
			unsigned rate, unsigned width, unsigned channels);


void audplay_mix_select(struct audplay *audplay, unsigned mix);
void audplay_volume_pan(struct audplay *audplay, unsigned volume, unsigned pan);


int afe_enable(unsigned device, unsigned rate, unsigned channels);
int afe_disable(unsigned device);


int msm_codec_output(int enable);

void adsp_audio_init(void);

#endif
