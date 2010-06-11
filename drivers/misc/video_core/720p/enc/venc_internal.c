/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/android_pmem.h>
#include <linux/clk.h>

#include "video_core_type.h"
#include "vcd_api.h"
#include "venc_internal.h"
#include "video_core_init.h"

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define ERR(x...) printk(KERN_ERR x)

u32 vid_enc_set_get_base_cfg(struct video_client_ctx *client_ctx,
	struct venc_basecfg *config, u32 set_flag)
{
	struct venc_targetbitrate venc_bitrate;
	struct venc_framerate frame_rate;
	u32 current_codec_type;

	if (!client_ctx || !config)
		return false;

	if (!vid_enc_set_get_codec(client_ctx, &current_codec_type, false))
			return false;

	DBG("%s: Current Codec Type = %u\n", __func__, current_codec_type);
	if (current_codec_type != config->codectype) {
		if (!vid_enc_set_get_codec(client_ctx, &config->codectype,
				set_flag))
			return false;
	}

	if (!vid_enc_set_get_inputformat(client_ctx, &config->inputformat,
			set_flag))
		return false;

	if (!vid_enc_set_get_framesize(client_ctx, &config->input_height,
			&config->input_width, set_flag))
		return false;

	if (set_flag)
		venc_bitrate.target_bitrate = config->targetbitrate;

	if (!vid_enc_set_get_bitrate(client_ctx, &venc_bitrate, set_flag))
		return false;

	if (!set_flag)
		config->targetbitrate = venc_bitrate.target_bitrate;

	if (set_flag) {
		frame_rate.fps_denominator = config->fps_den;
		frame_rate.fps_numerator = config->fps_num;
	}

	if (!vid_enc_set_get_framerate(client_ctx, &frame_rate, set_flag))
		return false;

	if (!set_flag) {
		config->fps_den = frame_rate.fps_denominator;
		config->fps_num = frame_rate.fps_numerator;
	}

	return true;
}

u32 vid_enc_set_get_inputformat(struct video_client_ctx *client_ctx,
	u32 *input_format, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_buffer_format format_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = true;

	if (!client_ctx || !input_format)
		return false;

	vcd_property_hdr.id = VCD_I_BUFFER_FORMAT;
	vcd_property_hdr.sz = sizeof(struct vcd_property_buffer_format);

	if (set_flag) {
		switch (*input_format) {
		case VEN_INPUTFMT_NV12:
			format_type.buffer_format = VCD_BUFFER_FORMAT_NV12;
			break;
		case VEN_INPUTFMT_NV21:
			format_type.buffer_format = VCD_BUFFER_FORMAT_TILE_4x2;
			break;
		default:
			status = false;
			break;
		}

		if (!status)
			return status;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &format_type);
		if (vcd_status) {
			status = false;
			ERR("%s(): Set VCD_I_BUFFER_FORMAT Failed\n", __func__);
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &format_type);

		if (vcd_status) {
			status = false;
			ERR("%s(): Get VCD_I_BUFFER_FORMAT Failed\n", __func__);
			return status;
		}
		switch (format_type.buffer_format) {
		case VCD_BUFFER_FORMAT_NV12:
			*input_format = VEN_INPUTFMT_NV12;
			break;
		case VCD_BUFFER_FORMAT_TILE_4x2:
			*input_format = VEN_INPUTFMT_NV21;
			break;
		default:
			status = false;
			break;
		}
	}
	return status;
}

u32 vid_enc_set_get_codec(struct video_client_ctx *client_ctx, u32 *codec_type,
	u32 set_flag)
{
	struct vcd_property_codec vcd_property_codec;
	struct vcd_property_hdr vcd_property_hdr;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = true;

	if (!client_ctx || !codec_type)
		return false;

	vcd_property_hdr.id = VCD_I_CODEC;
	vcd_property_hdr.sz = sizeof(struct vcd_property_codec);

	if (set_flag) {
		switch (*codec_type) {
		case VEN_CODEC_MPEG4:
			vcd_property_codec.codec = VCD_CODEC_MPEG4;
			break;
		case VEN_CODEC_H263:
			vcd_property_codec.codec = VCD_CODEC_H263;
			break;
		case VEN_CODEC_H264:
			vcd_property_codec.codec = VCD_CODEC_H264;
			break;
		default:
			status = false;
			break;
		}

		if (!status)
			return status;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &vcd_property_codec);
		if (vcd_status) {
			status = false;
			ERR("%s: Set VCD_I_CODEC Failed\n", __func__);
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &vcd_property_codec);

		if (vcd_status) {
			status = false;
			ERR("%s(): Get VCD_I_CODEC Failed\n", __func__);
			return status;
		}
		switch (vcd_property_codec.codec) {
		case VCD_CODEC_H263:
			*codec_type = VEN_CODEC_H263;
			break;
		case VCD_CODEC_H264:
			*codec_type = VEN_CODEC_H264;
			break;
		case VCD_CODEC_MPEG4:
			*codec_type = VEN_CODEC_MPEG4;
			break;
		case VCD_CODEC_DIVX_3:
		case VCD_CODEC_DIVX_4:
		case VCD_CODEC_DIVX_5:
		case VCD_CODEC_DIVX_6:
		case VCD_CODEC_MPEG1:
		case VCD_CODEC_MPEG2:
		case VCD_CODEC_VC1:
		case VCD_CODEC_VC1_RCV:
		case VCD_CODEC_XVID:
		default:
			status = false;
			break;
		}
	}
	return status;
}

u32 vid_enc_set_get_framesize(struct video_client_ctx *client_ctx, u32 *height,
	u32 *width, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_frame_size frame_size;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !height || !width)
		return false;

	vcd_property_hdr.id = VCD_I_FRAME_SIZE;
	vcd_property_hdr.sz = sizeof(struct vcd_property_frame_size);

	if (set_flag) {
		frame_size.height = *height;
		frame_size.width = *width;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &frame_size);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_FRAME_SIZE Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &frame_size);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_FRAME_SIZE Failed\n", __func__);
			return false;
		}
		*height = frame_size.height;
		*width = frame_size.width;
	}
	return true;
}

u32 vid_enc_set_get_bitrate(struct video_client_ctx *client_ctx,
	struct venc_targetbitrate *venc_bitrate, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_target_bitrate bit_rate;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !venc_bitrate)
		return false;

	vcd_property_hdr.id = VCD_I_TARGET_BITRATE;
	vcd_property_hdr.sz = sizeof(struct vcd_property_target_bitrate);
	if (set_flag) {
		bit_rate.target_bitrate = venc_bitrate->target_bitrate;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &bit_rate);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_TARGET_BITRATE Failed\n",
				__func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &bit_rate);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_TARGET_BITRATE Failed\n",
				__func__);
			return false;
		}
		venc_bitrate->target_bitrate = bit_rate.target_bitrate;
	}
	return true;
}

u32 vid_enc_set_get_framerate(struct video_client_ctx *client_ctx,
	struct venc_framerate *frame_rate, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_frame_rate vcd_frame_rate;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !frame_rate)
		return false;

	vcd_property_hdr.id = VCD_I_FRAME_RATE;
	vcd_property_hdr.sz = sizeof(struct vcd_property_frame_rate);

	if (set_flag) {
		vcd_frame_rate.fps_denominator = frame_rate->fps_denominator;
		vcd_frame_rate.fps_numerator = frame_rate->fps_numerator;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &vcd_frame_rate);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_FRAME_RATE Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &vcd_frame_rate);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_FRAME_RATE Failed\n", __func__);
			return false;
		}
		frame_rate->fps_denominator = vcd_frame_rate.fps_denominator;
		frame_rate->fps_numerator = vcd_frame_rate.fps_numerator;
	}
	return true;
}

u32 vid_enc_set_get_live_mode(struct video_client_ctx *client_ctx,
	struct venc_switch *encoder_switch, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_live live_mode;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx)
		return false;

	vcd_property_hdr.id = VCD_I_LIVE;
	vcd_property_hdr.sz = sizeof(struct vcd_property_live);

	if (set_flag) {
		live_mode.live = 1;
		if (!encoder_switch->status)
			live_mode.live = 0;

		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &live_mode);
		if (vcd_status) {
			ERR("%s(): Set VCD_I_LIVE Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &live_mode);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_LIVE Failed\n", __func__);
			return false;
		}
		encoder_switch->status = 1;
		if (!live_mode.live)
			encoder_switch->status = 0;
	}
	return true;
}

u32 vid_enc_set_get_short_header(struct video_client_ctx *client_ctx,
	struct venc_switch *encoder_switch, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_short_header short_header;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !encoder_switch)
		return false;

	vcd_property_hdr.id = VCD_I_SHORT_HEADER;
	vcd_property_hdr.sz = sizeof(struct vcd_property_short_header);

	if (set_flag) {
		short_header.short_header = (u32) encoder_switch->status;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &short_header);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_SHORT_HEADER Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &short_header);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_SHORT_HEADER Failed\n", __func__);
			return false;
		}
		encoder_switch->status = (u8)short_header.short_header;
	}
	return true;
}

u32 vid_enc_set_get_profile(struct video_client_ctx *client_ctx,
	struct venc_profile *profile, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_profile profile_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = true;

	if (!client_ctx || !profile)
		return false;

	vcd_property_hdr.id = VCD_I_PROFILE;
	vcd_property_hdr.sz = sizeof(struct vcd_property_profile);

	if (set_flag) {
		switch (profile->profile) {
		case VEN_PROFILE_MPEG4_SP:
			profile_type.profile = VCD_PROFILE_MPEG4_SP;
			break;
		case VEN_PROFILE_MPEG4_ASP:
			profile_type.profile = VCD_PROFILE_MPEG4_ASP;
			break;
		case VEN_PROFILE_H264_BASELINE:
			profile_type.profile = VCD_PROFILE_H264_BASELINE;
			break;
		case VEN_PROFILE_H264_MAIN:
			profile_type.profile = VCD_PROFILE_H264_MAIN;
			break;
		case VEN_PROFILE_H264_HIGH:
			profile_type.profile = VCD_PROFILE_H264_HIGH;
			break;
		case VEN_PROFILE_H263_BASELINE:
			profile_type.profile = VCD_PROFILE_H263_BASELINE;
			break;
		default:
			status = false;
			break;
		}

		if (!status)
			return status;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &profile_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_PROFILE Failed\n",	__func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &profile_type);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_PROFILE Failed\n", __func__);
			return false;
		}
		switch (profile_type.profile) {
		case VCD_PROFILE_H263_BASELINE:
			profile->profile = VEN_PROFILE_H263_BASELINE;
			break;
		case VCD_PROFILE_H264_BASELINE:
			profile->profile = VEN_PROFILE_H264_BASELINE;
			break;
		case VCD_PROFILE_H264_HIGH:
			profile->profile = VEN_PROFILE_H264_HIGH;
			break;
		case VCD_PROFILE_H264_MAIN:
			profile->profile = VEN_PROFILE_H264_MAIN;
			break;
		case VCD_PROFILE_MPEG4_ASP:
			profile->profile = VEN_PROFILE_MPEG4_ASP;
			break;
		case VCD_PROFILE_MPEG4_SP:
			profile->profile = VEN_PROFILE_MPEG4_SP;
			break;
		default:
			status = false;
			break;
		}
	}
	return status;
}

u32 vid_enc_set_get_profile_level(struct video_client_ctx *client_ctx,
	struct ven_profilelevel *profile_level,	u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_level level_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !profile_level)
		return false;

	vcd_property_hdr.id = VCD_I_LEVEL;
	vcd_property_hdr.sz = sizeof(struct vcd_property_level);

	if (set_flag) {
		switch (profile_level->level) {
		//TODO: collapse this crap
		case VEN_LEVEL_MPEG4_0:
			level_type.level = VCD_LEVEL_MPEG4_0;
			break;
		case VEN_LEVEL_MPEG4_1:
			level_type.level = VCD_LEVEL_MPEG4_1;
			break;
		case VEN_LEVEL_MPEG4_2:
			level_type.level = VCD_LEVEL_MPEG4_2;
			break;
		case VEN_LEVEL_MPEG4_3:
			level_type.level = VCD_LEVEL_MPEG4_3;
			break;
		case VEN_LEVEL_MPEG4_4:
			level_type.level = VCD_LEVEL_MPEG4_4;
			break;
		case VEN_LEVEL_MPEG4_5:
			level_type.level = VCD_LEVEL_MPEG4_5;
			break;
		case VEN_LEVEL_MPEG4_3b:
			level_type.level = VCD_LEVEL_MPEG4_3b;
			break;
		case VEN_LEVEL_MPEG4_6:
			level_type.level = VCD_LEVEL_MPEG4_6;
			break;
		case VEN_LEVEL_H264_1:
			level_type.level = VCD_LEVEL_H264_1;
			break;
		case VEN_LEVEL_H264_1b:
			level_type.level = VCD_LEVEL_H264_1b;
			break;
		case VEN_LEVEL_H264_1p1:
			level_type.level = VCD_LEVEL_H264_1p1;
			break;
		case VEN_LEVEL_H264_1p2:
			level_type.level = VCD_LEVEL_H264_1p2;
			break;
		case VEN_LEVEL_H264_1p3:
			level_type.level = VCD_LEVEL_H264_1p3;
			break;
		case VEN_LEVEL_H264_2:
			level_type.level = VCD_LEVEL_H264_2;
			break;
		case VEN_LEVEL_H264_2p1:
			level_type.level = VCD_LEVEL_H264_2p1;
			break;
		case VEN_LEVEL_H264_2p2:
			level_type.level = VCD_LEVEL_H264_2p2;
			break;
		case VEN_LEVEL_H264_3:
			level_type.level = VCD_LEVEL_H264_3;
			break;
		case VEN_LEVEL_H264_3p1:
			level_type.level = VCD_LEVEL_H264_3p1;
			break;
		case VEN_LEVEL_H263_10:
			level_type.level = VCD_LEVEL_H263_10;
			break;
		case VEN_LEVEL_H263_20:
			level_type.level = VCD_LEVEL_H263_20;
			break;
		case VEN_LEVEL_H263_30:
			level_type.level = VCD_LEVEL_H263_30;
			break;
		case VEN_LEVEL_H263_40:
			level_type.level = VCD_LEVEL_H263_40;
			break;
		case VEN_LEVEL_H263_45:
			level_type.level = VCD_LEVEL_H263_45;
			break;
		case VEN_LEVEL_H263_50:
			level_type.level = VCD_LEVEL_H263_50;
			break;
		case VEN_LEVEL_H263_60:
			level_type.level = VCD_LEVEL_H263_60;
			break;
		case VEN_LEVEL_H263_70:
			level_type.level = VCD_LEVEL_H263_70;
			break;
		default:
			return false;
		}
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &level_type);

		if (vcd_status) {
			ERR("%s: Set VCD_I_LEVEL Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &level_type);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_LEVEL Failed\n", __func__);
			return false;
		}
		switch (level_type.level) {
		case VCD_LEVEL_MPEG4_0:
			profile_level->level = VEN_LEVEL_MPEG4_0;
			break;
		case VCD_LEVEL_MPEG4_1:
			profile_level->level = VEN_LEVEL_MPEG4_1;
			break;
		case VCD_LEVEL_MPEG4_2:
			profile_level->level = VEN_LEVEL_MPEG4_2;
			break;
		case VCD_LEVEL_MPEG4_3:
			profile_level->level = VEN_LEVEL_MPEG4_3;
			break;
		case VCD_LEVEL_MPEG4_4:
			profile_level->level = VEN_LEVEL_MPEG4_4;
			break;
		case VCD_LEVEL_MPEG4_5:
			profile_level->level = VEN_LEVEL_MPEG4_5;
			break;
		case VCD_LEVEL_MPEG4_3b:
			profile_level->level = VEN_LEVEL_MPEG4_3b;
			break;
		case VCD_LEVEL_H264_1:
			profile_level->level = VEN_LEVEL_H264_1;
			break;
		case VCD_LEVEL_H264_1b:
			profile_level->level = VEN_LEVEL_H264_1b;
			break;
		case VCD_LEVEL_H264_1p1:
			profile_level->level = VEN_LEVEL_H264_1p1;
			break;
		case VCD_LEVEL_H264_1p2:
			profile_level->level = VEN_LEVEL_H264_1p2;
			break;
		case VCD_LEVEL_H264_1p3:
			profile_level->level = VEN_LEVEL_H264_1p3;
			break;
		case VCD_LEVEL_H264_2:
			profile_level->level = VEN_LEVEL_H264_2;
			break;
		case VCD_LEVEL_H264_2p1:
			profile_level->level = VEN_LEVEL_H264_2p1;
			break;
		case VCD_LEVEL_H264_2p2:
			profile_level->level = VEN_LEVEL_H264_2p2;
			break;
		case VCD_LEVEL_H264_3:
			profile_level->level = VEN_LEVEL_H264_3;
			break;
		case VCD_LEVEL_H264_3p1:
			profile_level->level = VEN_LEVEL_H264_3p1;
			break;
		case VCD_LEVEL_H264_3p2:
		case VCD_LEVEL_H264_4:
			return false;
		case VCD_LEVEL_H263_10:
			profile_level->level = VEN_LEVEL_H263_10;
			break;
		case VCD_LEVEL_H263_20:
			profile_level->level = VEN_LEVEL_H263_20;
			break;
		case VCD_LEVEL_H263_30:
			profile_level->level = VEN_LEVEL_H263_30;
			break;
		case VCD_LEVEL_H263_40:
			profile_level->level = VEN_LEVEL_H263_40;
			break;
		case VCD_LEVEL_H263_45:
			profile_level->level = VEN_LEVEL_H263_45;
			break;
		case VCD_LEVEL_H263_50:
			profile_level->level = VEN_LEVEL_H263_50;
			break;
		case VCD_LEVEL_H263_60:
			profile_level->level = VEN_LEVEL_H263_60;
			break;
		case VCD_LEVEL_H263_70:
		default:
			return false;
		}
	}
	return true;
}

u32 vid_enc_set_get_session_qp(struct video_client_ctx *client_ctx,
	struct venc_sessionqp *session_qp, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_session_qp qp_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !session_qp)
		return false;

	vcd_property_hdr.id = VCD_I_SESSION_QP;
	vcd_property_hdr.sz = sizeof(struct vcd_property_session_qp);

	if (set_flag) {
		qp_type.iframe_qp = session_qp->iframeqp;
		qp_type.frame_qp = session_qp->pframqp;

		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &qp_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_SESSION_QP Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &qp_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_SESSION_QP Failed\n", __func__);
			return false;
		}
		session_qp->iframeqp = qp_type.iframe_qp;
		session_qp->pframqp = qp_type.frame_qp;
	}
	return true;
}

u32 vid_enc_set_get_intraperiod(struct video_client_ctx *client_ctx,
	struct venc_intraperiod *intraperiod, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_i_period period_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !intraperiod)
		return false;

	vcd_property_hdr.id = VCD_I_INTRA_PERIOD;
	vcd_property_hdr.sz = sizeof(struct vcd_property_i_period);

	if (set_flag) {
		period_type.frames = intraperiod->num_pframes;
		period_type.bframes = 0;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &period_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_INTRA_PERIOD Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &period_type);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_INTRA_PERIOD Failed\n", __func__);
			return false;
		}
		intraperiod->num_pframes = period_type.frames;
	}
	return true;
}

u32 vid_enc_request_iframe(struct video_client_ctx *client_ctx)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_req_i_frame request;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx)
		return false;

	vcd_property_hdr.id = VCD_I_REQ_IFRAME;
	vcd_property_hdr.sz = sizeof(struct vcd_property_req_i_frame);
	request.req_i_frame = 1;

	vcd_status = vcd_set_property(client_ctx->vcd_handle, &vcd_property_hdr,
		&request);

	if (vcd_status) {
		ERR("%s(): Set VCD_I_REQ_IFRAME Failed\n", __func__);
		return false;
	}
	return true;
}

u32 vid_enc_get_sequence_header(struct video_client_ctx *client_ctx,
	struct venc_seqheader *seq_header)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_sequence_hdr hdr_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !seq_header || !seq_header->buf_sz)
		return false;

	vcd_property_hdr.id = VCD_I_SEQ_HEADER;
	vcd_property_hdr.sz = sizeof(struct vcd_sequence_hdr);

	hdr_type.addr = kzalloc(seq_header->buf_sz, GFP_KERNEL);
	seq_header->buf = hdr_type.addr;
	if (!hdr_type.addr)
		return false;

	hdr_type.sz = seq_header->buf_sz;
	vcd_status = vcd_get_property(client_ctx->vcd_handle, &vcd_property_hdr,
		&hdr_type);

	if (vcd_status) {
		ERR("%s: Get VCD_I_SEQ_HEADER Failed\n", __func__);
		return false;
	}
	return true;
}

u32 vid_enc_set_get_entropy_cfg(struct video_client_ctx *client_ctx,
	struct venc_entropycfg *entropy_cfg, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_entropy_control control_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !entropy_cfg)
		return false;

	vcd_property_hdr.id = VCD_I_ENTROPY_CTRL;
	vcd_property_hdr.sz = sizeof(struct vcd_property_entropy_control);

	if (set_flag) {
		switch (entropy_cfg->cabacmodel) {
		case VEN_ENTROPY_MODEL_CAVLC:
			control_type.entropy_sel = VCD_ENTROPY_SEL_CAVLC;
			break;
		case VEN_ENTROPY_MODEL_CABAC:
			control_type.entropy_sel = VCD_ENTROPY_SEL_CABAC;
			break;
		default:
			return false;
		}

		if (entropy_cfg->cabacmodel == VCD_ENTROPY_SEL_CABAC) {
			switch (entropy_cfg->cabacmodel) {
			case VEN_CABAC_MODEL_0:
				control_type.cabac_model =
					VCD_CABAC_MODEL_NUMBER_0;
				break;
			case VEN_CABAC_MODEL_1:
				control_type.cabac_model =
					VCD_CABAC_MODEL_NUMBER_1;
				break;
			case VEN_CABAC_MODEL_2:
				control_type.cabac_model =
					VCD_CABAC_MODEL_NUMBER_2;
				break;
			default:
				return false;
			}
		}

		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s(): Set VCD_I_ENTROPY_CTRL Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s(): Get VCD_I_ENTROPY_CTRL Failed\n", __func__);
			return false;
		}

		switch (control_type.entropy_sel) {
		case VCD_ENTROPY_SEL_CABAC:
			entropy_cfg->cabacmodel = VEN_ENTROPY_MODEL_CABAC;
			break;
		case VCD_ENTROPY_SEL_CAVLC:
			entropy_cfg->cabacmodel = VEN_ENTROPY_MODEL_CAVLC;
			break;
		default:
			return false;
		}

		if (control_type.entropy_sel ==	VCD_ENTROPY_SEL_CABAC) {
			switch (control_type.cabac_model) {
			case VCD_CABAC_MODEL_NUMBER_0:
				entropy_cfg->cabacmodel = VEN_CABAC_MODEL_0;
				break;
			case VCD_CABAC_MODEL_NUMBER_1:
				entropy_cfg->cabacmodel = VEN_CABAC_MODEL_1;
				break;
			case VCD_CABAC_MODEL_NUMBER_2:
				entropy_cfg->cabacmodel = VEN_CABAC_MODEL_2;
				break;
			default:
				return false;
			}
		}
	}
	return true;
}

u32 vid_enc_set_get_dbcfg(struct video_client_ctx *client_ctx,
	struct venc_dbcfg *dbcfg, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_db_config control_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !dbcfg)
		return false;

	vcd_property_hdr.id = VCD_I_DEBLOCKING;
	vcd_property_hdr.sz = sizeof(struct vcd_property_db_config);

	if (set_flag) {
		switch (dbcfg->db_mode) {
		case VEN_DB_DISABLE:
			control_type.db_config = VCD_DB_DISABLE;
			break;
		case VEN_DB_ALL_BLKG_BNDRY:
			control_type.db_config = VCD_DB_ALL_BLOCKING_BOUNDARY;
			break;
		case VEN_DB_SKIP_SLICE_BNDRY:
			control_type.db_config = VCD_DB_SKIP_SLICE_BOUNDARY;
			break;
		default:
			return false;
		}

		control_type.slice_alpha_offset = dbcfg->slicealpha_offset;
		control_type.slice_beta_offset = dbcfg->slicebeta_offset;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s: Set VCD_I_DEBLOCKING Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s: Get VCD_I_DEBLOCKING Failed\n", __func__);
			return false;
		}
		switch (control_type.db_config) {
		case VCD_DB_ALL_BLOCKING_BOUNDARY:
			dbcfg->db_mode = VEN_DB_ALL_BLKG_BNDRY;
			break;
		case VCD_DB_DISABLE:
			dbcfg->db_mode = VEN_DB_DISABLE;
			break;
		case VCD_DB_SKIP_SLICE_BOUNDARY:
			dbcfg->db_mode = VEN_DB_SKIP_SLICE_BNDRY;
			break;
		default:
			return false;
		}
		dbcfg->slicealpha_offset = control_type.slice_alpha_offset;
		dbcfg->slicebeta_offset = control_type.slice_beta_offset;
	}
	return true;
}

u32 vid_enc_set_get_intrarefresh(struct video_client_ctx *client_ctx,
	struct venc_intrarefresh *intrarefresh, u32 set_flag)
{
	struct vcd_property_hdr prop_hdr;
	struct vcd_property_intra_refresh_mb_number control_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !intrarefresh)
		return false;

	prop_hdr.id = VCD_I_INTRA_REFRESH;
	prop_hdr.sz = sizeof(struct vcd_property_intra_refresh_mb_number);

	if (set_flag) {
		control_type.cir_mb_number = intrarefresh->mbcount;
		vcd_status = vcd_set_property(client_ctx->vcd_handle, &prop_hdr,
			&control_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_INTRA_REFRESH Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle, &prop_hdr,
			&control_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_INTRA_REFRESH Failed\n", __func__);
			return false;
		}
		intrarefresh->mbcount = control_type.cir_mb_number;
	}
	return true;
}

u32 vid_enc_set_get_multiclicecfg(struct video_client_ctx *client_ctx,
	struct venc_multiclicecfg *multiclicecfg, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_multi_slice control_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !multiclicecfg)
		return false;

	vcd_property_hdr.id = VCD_I_MULTI_SLICE;
	vcd_property_hdr.sz = sizeof(struct vcd_property_multi_slice);

	if (set_flag) {
		switch (multiclicecfg->mslice_mode) {
		case VEN_MSLICE_OFF:
			control_type.m_slice_sel = VCD_MSLICE_OFF;
			break;
		case VEN_MSLICE_CNT_MB:
			control_type.m_slice_sel = VCD_MSLICE_BY_MB_COUNT;
			break;
		case VEN_MSLICE_CNT_BYTE:
			control_type.m_slice_sel = VCD_MSLICE_BY_BYTE_COUNT;
			break;
		case VEN_MSLICE_GOB:
			control_type.m_slice_sel = VCD_MSLICE_BY_GOB;
			break;
		default:
			return false;
		}

		control_type.m_slice_size = multiclicecfg->mslice_size;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s: Set VCD_I_MULTI_SLICE Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s: Get VCD_I_MULTI_SLICE Failed\n", __func__);
			return false;
		}
		multiclicecfg->mslice_size = control_type.m_slice_size;
		switch (control_type.m_slice_sel) {
		case VCD_MSLICE_OFF:
			multiclicecfg->mslice_mode = VEN_MSLICE_OFF;
			break;
		case VCD_MSLICE_BY_MB_COUNT:
			multiclicecfg->mslice_mode = VEN_MSLICE_CNT_MB;
			break;
		case VCD_MSLICE_BY_BYTE_COUNT:
			multiclicecfg->mslice_mode = VEN_MSLICE_CNT_BYTE;
			break;
		case VCD_MSLICE_BY_GOB:
			multiclicecfg->mslice_mode = VEN_MSLICE_GOB;
			break;
		default:
			return false;
		}
	}
	return true;
}

u32 vid_enc_set_get_ratectrlcfg(struct video_client_ctx *client_ctx,
	struct venc_ratectrlcfg *ratectrlcfg, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_rate_control control_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !ratectrlcfg)
		return false;

	vcd_property_hdr.id = VCD_I_RATE_CONTROL;
	vcd_property_hdr.sz = sizeof(struct vcd_property_rate_control);

	if (set_flag) {
		switch (ratectrlcfg->rcmode) {
		case VEN_RC_OFF:
			control_type.rate_control = VCD_RATE_CONTROL_OFF;
			break;
		case VEN_RC_CBR_VFR:
			control_type.rate_control = VCD_RATE_CONTROL_CBR_VFR;
			break;
		case VEN_RC_VBR_CFR:
			control_type.rate_control = VCD_RATE_CONTROL_VBR_CFR;
			break;
		case VEN_RC_VBR_VFR:
			control_type.rate_control = VCD_RATE_CONTROL_VBR_VFR;
			break;
		default:
			return false;
		}

		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s(): Set VCD_I_RATE_CONTROL Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_RATE_CONTROL Failed\n", __func__);
			return false;
		}

		switch (control_type.rate_control) {
		case VCD_RATE_CONTROL_OFF:
			ratectrlcfg->rcmode = VEN_RC_OFF;
			break;
		case VCD_RATE_CONTROL_CBR_VFR:
			ratectrlcfg->rcmode = VEN_RC_CBR_VFR;
			break;
		case VCD_RATE_CONTROL_VBR_CFR:
			ratectrlcfg->rcmode = VEN_RC_VBR_CFR;
			break;
		case VCD_RATE_CONTROL_VBR_VFR:
			ratectrlcfg->rcmode = VEN_RC_VBR_VFR;
			break;
		default:
			return false;
		}
	}
	return true;
}

u32 vid_enc_set_get_voptimingcfg(struct video_client_ctx *client_ctx,
	struct venc_voptimingcfg *venc_timing, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_vop_timing vcd_timing;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !venc_timing)
		return false;

	vcd_property_hdr.id = VCD_I_VOP_TIMING;
	vcd_property_hdr.sz = sizeof(struct vcd_property_vop_timing);

	if (set_flag) {
		vcd_timing.vop_time_resolution =
			venc_timing->voptime_resolution;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &vcd_timing);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_VOP_TIMING Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &vcd_timing);
		if (vcd_status) {
			ERR("%s(): Get VCD_I_VOP_TIMING Failed\n", __func__);
			return false;
		}
		venc_timing->voptime_resolution =
			vcd_timing.vop_time_resolution;
	}
	return true;
}

u32 vid_enc_set_get_headerextension(struct video_client_ctx *client_ctx,
	struct venc_headerextension *headerextension, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	u32 control_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !headerextension)
		return false;

	vcd_property_hdr.id = VCD_I_HEADER_EXTENSION;
	vcd_property_hdr.sz = sizeof(u32);

	if (set_flag) {
		control_type = headerextension->header_extension;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s: Set VCD_I_HEADER_EXTENSION Fail\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s: Get VCD_I_HEADER_EXTENSION Fail\n", __func__);
			return false;
		}
		headerextension->header_extension = control_type;
	}
	return true;
}

u32 vid_enc_set_get_qprange(struct video_client_ctx *client_ctx,
	struct venc_qprange *qprange, u32 set_flag)
{
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_qp_range control_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !qprange)
		return false;

	vcd_property_hdr.id = VCD_I_QP_RANGE;
	vcd_property_hdr.sz =	sizeof(struct vcd_property_qp_range);

	if (set_flag) {
		control_type.max_qp = qprange->maxqp;
		control_type.min_qp = qprange->minqp;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_QP_RANGE Failed\n", __func__);
			return false;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s(): Get VCD_I_QP_RANGE Failed\n", __func__);
			return false;
		}
		qprange->maxqp = control_type.max_qp;
		qprange->minqp = control_type.min_qp;
	}
	return true;
}

u32 vid_enc_start(struct video_client_ctx *client_ctx)
{
	u32 vcd_status;

	if (!client_ctx)
		return false;

	vcd_status = vcd_encode_start(client_ctx->vcd_handle);
	if (vcd_status) {
		ERR("%s: vcd_encode_start failed %u\n",	__func__, vcd_status);
		return false;
	}
	return true;
}


u32 vid_enc_stop(struct video_client_ctx *client_ctx)
{
	u32 vcd_status;

	if (!client_ctx)
		return false;
	vcd_status = vcd_stop(client_ctx->vcd_handle);
	if (vcd_status) {
		ERR("%s: vcd_stop failed %u\n", __func__, vcd_status);
		return false;
	}
	DBG("Send STOP_DONE message to client = %p\n", client_ctx);
	return true;
}

u32 vid_enc_pause(struct video_client_ctx *client_ctx)
{
	u32 vcd_status;

	if (!client_ctx)
		return false;

	DBG("PAUSE command from client = %p\n", client_ctx);
	vcd_status = vcd_pause(client_ctx->vcd_handle);
	if (vcd_status)
		return false;
	return true;
}

u32 vid_enc_resume(struct video_client_ctx *client_ctx)
{
	u32 vcd_status;

	if (!client_ctx)
		return false;

	DBG("Resume command from client = %p\n", client_ctx);
	vcd_status = vcd_resume(client_ctx->vcd_handle);
	if (vcd_status)
		return false;
	return true;
}

u32 vid_enc_flush(struct video_client_ctx *client_ctx,
	struct venc_bufferflush *bufferflush)
{
	u32 mode;
	u32 vcd_status;

	if (!client_ctx || !bufferflush)
		return false;

	switch (bufferflush->flush_mode) {
	case VEN_FLUSH_INPUT:
		mode = VCD_FLUSH_INPUT;
		break;
	case VEN_FLUSH_OUTPUT:
		mode = VCD_FLUSH_OUTPUT;
		break;
	case VEN_FLUSH_ALL:
		mode = VCD_FLUSH_ALL;
		break;
	default:
		return false;
		break;
	}
	vcd_status = vcd_flush(client_ctx->vcd_handle, mode);
	if (vcd_status)
		return false;
	return true;
}

u32 vid_enc_get_buffer_req(struct video_client_ctx *client_ctx,
	struct venc_allocatorproperty *venc_buf_req, u32 input_dir)
{
	enum vcd_buffer_type buffer;
	struct vcd_buffer_requirement buffer_req;
	u32 vcd_status;

	if (!client_ctx || !venc_buf_req)
		return false;

	buffer = VCD_BUFFER_OUTPUT;
	if (input_dir)
		buffer = VCD_BUFFER_INPUT;

	vcd_status = vcd_get_buffer_requirements(client_ctx->vcd_handle,
		buffer, &buffer_req);
	if (vcd_status)
		return false;

	venc_buf_req->actualcount = buffer_req.actual_count;
	venc_buf_req->alignment = buffer_req.align;
	venc_buf_req->datasize = buffer_req.size;
	venc_buf_req->mincount = buffer_req.min_count;
	venc_buf_req->maxcount = buffer_req.max_count;
	venc_buf_req->alignment = buffer_req.align;
	venc_buf_req->bufpoolid = buffer_req.buf_pool_id;
	venc_buf_req->suffixsize = 0;

	return true;
}

u32 vid_enc_set_buffer_req(struct video_client_ctx *client_ctx,
	struct venc_allocatorproperty *venc_buf_req, u32 input_dir)
{
	enum vcd_buffer_type buffer;
	struct vcd_buffer_requirement buffer_req;
	u32 vcd_status;

	if (!client_ctx || !venc_buf_req)
		return false;

	buffer = VCD_BUFFER_OUTPUT;
	if (input_dir)
		buffer = VCD_BUFFER_INPUT;

	buffer_req.actual_count = venc_buf_req->actualcount;
	buffer_req.align = venc_buf_req->alignment;
	buffer_req.size = venc_buf_req->datasize;
	buffer_req.min_count = venc_buf_req->mincount;
	buffer_req.max_count = venc_buf_req->maxcount;
	buffer_req.align = venc_buf_req->alignment;
	buffer_req.buf_pool_id = 0;

	vcd_status = vcd_set_buffer_requirements(client_ctx->vcd_handle,
		buffer, &buffer_req);

	if (vcd_status)
		return false;
	return true;
}

u32 vid_enc_set_buffer(struct video_client_ctx *client_ctx,
	struct venc_bufferpayload *buf_info, enum venc_buffer_dir buf_type)
{
	u32 vcd_status = VCD_ERR_FAIL;
	enum vcd_buffer_type buffer_vcd_type;
	enum buffer_dir dir_buffer = BUFFER_TYPE_INPUT;
	void __user *user_addr;
	void *kern_addr;
	phys_addr_t phys_addr;
	unsigned long len;
	int pmem_fd;
	struct file *file;
	struct buf_addr_table *buf_addr_table;

	s32 buf_index = -1;

	if (!client_ctx || !buf_info)
		return false;

	user_addr = buf_info->buffer;

	if (buf_type == VEN_BUFFER_TYPE_OUTPUT)
		dir_buffer = BUFFER_TYPE_OUTPUT;

	/* if buffer already set, ignore */
	if (vid_c_lookup_addr_table(client_ctx, dir_buffer, true, &user_addr,
			&kern_addr, &phys_addr, &pmem_fd, &file,
			&buf_index)) {

		DBG("%s: user_addr = %p is already set\n", __func__, user_addr);
		return true;
	}

	if (get_pmem_file(buf_info->fd, (unsigned long *)&phys_addr,
			(unsigned long *)&kern_addr,
			&len, &file)) {
		ERR("%s: get_pmem_file failed\n", __func__);
		return false;
	}
	put_pmem_file(file);
	if (buf_type == VEN_BUFFER_TYPE_INPUT) {
		buffer_vcd_type = VCD_BUFFER_INPUT;
		client_ctx->num_of_input_buffers++;

		if (client_ctx->num_of_input_buffers > VID_ENC_MAX_NUM_OF_BUFF
				) {
			ERR("%s: num_of_input_buffers reached max value"
				" VID_ENC_MAX_NUM_OF_BUFF\n", __func__);
			client_ctx->num_of_input_buffers--;
			return false;
		}

		buf_index = client_ctx->num_of_input_buffers - 1;
		buf_addr_table = &client_ctx->input_buf_addr_table[buf_index];
		buf_addr_table->user_addr = buf_info->buffer;
		kern_addr = (u8 *)kern_addr + buf_info->offset;
		phys_addr += buf_info->offset;
		buf_addr_table->kern_addr = kern_addr;
		buf_addr_table->phys_addr = phys_addr;
		buf_addr_table->pmem_fd = buf_info->fd;
		buf_addr_table->file = file;
	} else {
		buffer_vcd_type = VCD_BUFFER_OUTPUT;

		client_ctx->num_of_output_buffers++;

		if (client_ctx->num_of_output_buffers >
				VID_ENC_MAX_NUM_OF_BUFF) {
			ERR("%s: num_of_outut_buffers reached max value"
				" VID_ENC_MAX_NUM_OF_BUFF\n", __func__);
			client_ctx->num_of_output_buffers--;
			return false;
		}

		buf_index = client_ctx->num_of_output_buffers - 1;

		buf_addr_table = &client_ctx->output_buf_addr_table[buf_index];
		kern_addr = (u8 *)kern_addr + buf_info->offset;
		phys_addr += buf_info->offset;
		buf_addr_table->user_addr = buf_info->buffer;
		buf_addr_table->kern_addr = kern_addr;
		buf_addr_table->phys_addr = phys_addr;
		buf_addr_table->pmem_fd = buf_info->fd;
		buf_addr_table->file = file;
	}

	vcd_status = vcd_set_buffer(client_ctx->vcd_handle, buffer_vcd_type,
		kern_addr, buf_info->sz);

	if (!vcd_status)
		return true;
	else
		return false;
}

u32 vid_enc_encode_frame(struct video_client_ctx *client_ctx,
	struct venc_buffer *input_frame_info)
{
	struct vcd_frame_data vcd_input_buffer;
	void __user *user_addr;
	void *kern_addr;
	phys_addr_t phys_addr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;

	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !input_frame_info)
		return false;

	user_addr = input_frame_info->addr;

	if (!vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_INPUT, true,
			&user_addr, &kern_addr, &phys_addr, &pmem_fd, &file,
			&buffer_index)) {
		ERR("%s: kernel_vaddr not found\n", __func__);
		return false;
	}

	/* kern_addr  is found. send the frame to VCD */
	memset((void *)&vcd_input_buffer, 0, sizeof(vcd_input_buffer));

	vcd_input_buffer.virt_addr = (u8 *)kern_addr + input_frame_info->offset;
	vcd_input_buffer.offset = input_frame_info->offset;
	vcd_input_buffer.client_data = input_frame_info->clientdata;
	vcd_input_buffer.ip_frm_tag = (u32)input_frame_info->clientdata;
	vcd_input_buffer.data_len = input_frame_info->len;
	vcd_input_buffer.time_stamp = input_frame_info->timestamp;

	/* Rely on VCD using the same flags as OMX */
	vcd_input_buffer.flags = input_frame_info->flags;

	vcd_status = vcd_encode_frame(client_ctx->vcd_handle,
		&vcd_input_buffer);

	if (vcd_status) {
		ERR("%s: vcd_encode_frame failed = %u\n", __func__, vcd_status);
		return false;
	}
	return true;
}

u32 vid_enc_fill_output_buffer(struct video_client_ctx *client_ctx,
	struct venc_buffer *output_frame_info)
{
	void __user *user_addr;
	void *kern_addr;
	phys_addr_t phys_addr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;
	u32 vcd_status = VCD_ERR_FAIL;

	struct vcd_frame_data vcd_frame;

	if (!client_ctx || !output_frame_info)
		return false;

	user_addr = output_frame_info->addr;

	if (!vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_OUTPUT,
			true, &user_addr, &kern_addr, &phys_addr, &pmem_fd,
			&file, &buffer_index)) {
		ERR("%s: kernel_vaddr not found\n", __func__);
		return false;
	}

	memset((void *)&vcd_frame, 0, sizeof(vcd_frame));
	vcd_frame.virt_addr = kern_addr;
	vcd_frame.client_data = output_frame_info->clientdata;
	vcd_frame.alloc_len = output_frame_info->sz;

	vcd_status = vcd_fill_output_buffer(client_ctx->vcd_handle, &vcd_frame);
	if (vcd_status) {
		ERR("%s: vcd_fill_output_buffer %u\n", __func__, vcd_status);
		return false;
	}
	return true;
}
