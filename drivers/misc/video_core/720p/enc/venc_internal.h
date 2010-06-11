/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef VENC_INTERNAL_H
#define VENC_INTERNAL_H

#include <linux/msm_vidc_enc.h>
#include <linux/cdev.h>

#include "video_core_init.h"

#define VID_ENC_MAX_ENCODER_CLIENTS 16
#define VID_ENC_MAX_NUM_OF_BUFF 100

enum venc_buffer_dir {
	VEN_BUFFER_TYPE_INPUT,
	VEN_BUFFER_TYPE_OUTPUT
};

struct vid_enc_msg {
	struct list_head list;
	struct venc_msg venc_msg_info;
};

struct vid_enc_dev {

	struct cdev cdev;
	struct device *device;
	resource_size_t phys_base;
	void __iomem *virt_base;
	unsigned int irq;
	struct clk *hclk;
	struct clk *hclk_div2;
	struct clk *pclk;
	unsigned long hclk_rate;
	struct mutex lock;
	s32 device_handle;
	struct video_client_ctx venc_clients[VID_ENC_MAX_ENCODER_CLIENTS];
	u32 num_clients;
};

u32 vid_enc_set_get_base_cfg(struct video_client_ctx *client_ctx,
	struct venc_basecfg *base_config, u32 set_flag);

u32 vid_enc_set_get_inputformat(struct video_client_ctx *client_ctx,
	u32 *input_format, u32 set_flag);

u32 vid_enc_set_get_codec(struct video_client_ctx *client_ctx, u32 *codec_type,
	u32 set_flag);

u32 vid_enc_set_get_framesize(struct video_client_ctx *client_ctx,
	u32 *height, u32 *width, u32 set_flag);

u32 vid_enc_set_get_bitrate(struct video_client_ctx *client_ctx,
	struct venc_targetbitrate *venc_bitrate, u32 set_flag);

u32 vid_enc_set_get_framerate(struct video_client_ctx *client_ctx,
	struct venc_framerate *frame_rate, u32 set_flag);

u32 vid_enc_set_get_live_mode(struct video_client_ctx *client_ctx,
	struct venc_switch *encoder_switch, u32 set_flag);

u32 vid_enc_set_get_short_header(struct video_client_ctx *client_ctx,
	struct venc_switch *encoder_switch, u32 set_flag);

u32 vid_enc_set_get_profile(struct video_client_ctx *client_ctx,
	struct venc_profile *profile, u32 set_flag);

u32 vid_enc_set_get_profile_level(struct video_client_ctx *client_ctx,
	struct ven_profilelevel *profile_level, u32 set_flag);

u32 vid_enc_set_get_session_qp(struct video_client_ctx *client_ctx,
	struct venc_sessionqp *session_qp, u32 set_flag);

u32 vid_enc_set_get_intraperiod(struct video_client_ctx *client_ctx,
	struct venc_intraperiod *intraperiod, u32 set_flag);

u32 vid_enc_request_iframe(struct video_client_ctx *client_ctx);

u32 vid_enc_get_sequence_header(struct video_client_ctx *client_ctx,
	struct venc_seqheader *seq_header);

u32 vid_enc_set_get_entropy_cfg(struct video_client_ctx *client_ctx,
	struct venc_entropycfg *entropy_cfg, u32 set_flag);

u32 vid_enc_set_get_dbcfg(struct video_client_ctx *client_ctx,
	struct venc_dbcfg *dbcfg, u32 set_flag);

u32 vid_enc_set_get_intrarefresh(struct video_client_ctx *client_ctx,
	struct venc_intrarefresh *intrarefresh,	u32 set_flag);

u32 vid_enc_set_get_multiclicecfg(struct video_client_ctx *client_ctx,
	struct venc_multiclicecfg *multiclicecfg, u32 set_flag);

u32 vid_enc_set_get_ratectrlcfg(struct video_client_ctx *client_ctx,
	struct venc_ratectrlcfg *ratectrlcfg, u32 set_flag);

u32 vid_enc_set_get_voptimingcfg(struct video_client_ctx *client_ctx,
	struct  venc_voptimingcfg *voptimingcfg, u32 set_flag);

u32 vid_enc_set_get_headerextension(struct video_client_ctx *client_ctx,
	struct venc_headerextension *headerextension, u32 set_flag);

u32 vid_enc_set_get_qprange(struct video_client_ctx *client_ctx,
	struct venc_qprange *qprange, u32 set_flag);

u32 vid_enc_start(struct video_client_ctx *client_ctx);

u32 vid_enc_stop(struct video_client_ctx *client_ctx);

u32 vid_enc_pause(struct video_client_ctx *client_ctx);

u32 vid_enc_resume(struct video_client_ctx *client_ctx);

u32 vid_enc_flush(struct video_client_ctx *client_ctx,
	struct venc_bufferflush *bufferflush);

u32 vid_enc_get_buffer_req(struct video_client_ctx *client_ctx,
	struct venc_allocatorproperty *venc_buf_req, u32 input_dir);

u32 vid_enc_set_buffer_req(struct video_client_ctx *client_ctx,
	struct venc_allocatorproperty *venc_buf_req, u32 input_dir);

u32 vid_enc_set_buffer(struct video_client_ctx *client_ctx,
	struct venc_bufferpayload *buffer_info,
	enum venc_buffer_dir buffer_type);

u32 vid_enc_encode_frame(struct video_client_ctx *client_ctx,
	struct venc_buffer *input_frame_info);

u32 vid_enc_fill_output_buffer(struct video_client_ctx *client_ctx,
	struct venc_buffer *output_frame_info);

#endif
