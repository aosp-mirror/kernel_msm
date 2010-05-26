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
		struct venc_basecfg *base_config, u32 set_flag)
{
	struct venc_targetbitrate venc_bitrate;
	struct venc_framerate frame_rate;
	u32 current_codec_type;

	if (!client_ctx || !base_config)
		return FALSE;

	if (!vid_enc_set_get_codec(client_ctx, &current_codec_type, FALSE))
			return FALSE;

	DBG("%s(): Current Codec Type = %u\n", __func__, current_codec_type);
	if (current_codec_type != base_config->codectype) {
		if (!vid_enc_set_get_codec(client_ctx,
				(u32 *)&base_config->codectype, set_flag))
			return FALSE;
	}

	if (!vid_enc_set_get_inputformat(client_ctx,
			(u32 *)&base_config->inputformat, set_flag))
		return FALSE;

	if (!vid_enc_set_get_framesize(client_ctx,
			(u32 *)&base_config->input_height,
			(u32 *)&base_config->input_width, set_flag))
		return FALSE;

	if (set_flag)
		venc_bitrate.target_bitrate = base_config->targetbitrate;

	if (!vid_enc_set_get_bitrate(client_ctx, &venc_bitrate, set_flag))
		return FALSE;

	if (!set_flag)
		base_config->targetbitrate = venc_bitrate.target_bitrate;

	if (set_flag) {
		frame_rate.fps_denominator = base_config->fps_den;
		frame_rate.fps_numerator = base_config->fps_num;
	}

	if (!vid_enc_set_get_framerate(client_ctx, &frame_rate, set_flag))
		return FALSE;

	if (!set_flag) {
		base_config->fps_den = frame_rate.fps_denominator;
		base_config->fps_num = frame_rate.fps_numerator;
	}

	return TRUE;
}

u32 vid_enc_set_get_inputformat(struct video_client_ctx *client_ctx,
		u32 *input_format, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_buffer_format_type format_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !input_format)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_BUFFER_FORMAT;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_buffer_format_type);

	if (set_flag) {
		switch (*input_format) {
		case VEN_INPUTFMT_NV12:
			format_type.e_buffer_format = VCD_BUFFER_FORMAT_NV12;
			break;
		case VEN_INPUTFMT_NV21:
			format_type.e_buffer_format =
				VCD_BUFFER_FORMAT_TILE_4x2;
			break;
		default:
			status = FALSE;
			break;
		}

		if (status) {
			vcd_status = vcd_set_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &format_type);
			if (vcd_status) {
				status = FALSE;
				ERR("%s(): Set VCD_I_BUFFER_FORMAT Failed\n",
						 __func__);
			}
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &format_type);

		if (vcd_status) {
			status = FALSE;
			ERR("%s(): Get VCD_I_BUFFER_FORMAT Failed\n", __func__);
		}	else {
			switch (format_type.e_buffer_format) {
			case VCD_BUFFER_FORMAT_NV12:
				*input_format = VEN_INPUTFMT_NV12;
				break;
			case VCD_BUFFER_FORMAT_TILE_4x2:
				*input_format = VEN_INPUTFMT_NV21;
				break;
			default:
				status = FALSE;
				break;
			}
		}
	}
	return status;
}

u32 vid_enc_set_get_codec(struct video_client_ctx *client_ctx, u32 *codec_type,
		u32 set_flag)
{
	struct vcd_property_codec_type vcd_property_codec;
	struct vcd_property_hdr_type vcd_property_hdr;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !codec_type)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_CODEC;
	vcd_property_hdr.n_size = sizeof(struct vcd_property_codec_type);

	if (set_flag) {
		switch (*codec_type) {
		case VEN_CODEC_MPEG4:
			vcd_property_codec.e_codec = VCD_CODEC_MPEG4;
			break;
		case VEN_CODEC_H263:
			vcd_property_codec.e_codec = VCD_CODEC_H263;
			break;
		case VEN_CODEC_H264:
			vcd_property_codec.e_codec = VCD_CODEC_H264;
			break;
		default:
			status = FALSE;
			break;
		}

		if (status) {
			vcd_status = vcd_set_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &vcd_property_codec);
			if (vcd_status) {
				status = FALSE;
				ERR("%s(): Set VCD_I_CODEC Failed\n", __func__);
			}
		}
	}	else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &vcd_property_codec);

		if (vcd_status) {
			status = FALSE;
			ERR("%s(): Get VCD_I_CODEC Failed\n",
					 __func__);
		} else {
			switch (vcd_property_codec.e_codec) {
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
				status = FALSE;
				break;
			}
		}
	}
	return status;
}

u32 vid_enc_set_get_framesize(struct video_client_ctx *client_ctx,
		u32 *height, u32 *width, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_frame_size_type frame_size;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !height || !width)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_FRAME_SIZE;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_frame_size_type);

	if (set_flag) {
		frame_size.n_height = *height;
		frame_size.n_width = *width;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &frame_size);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_FRAME_SIZE Failed\n",
					__func__);
			return FALSE;
		}
	}	else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
						&vcd_property_hdr, &frame_size);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_FRAME_SIZE Failed\n",
					__func__);
			return FALSE;
		}
		*height = frame_size.n_height;
		*width = frame_size.n_width;
	}
	return TRUE;
}

u32 vid_enc_set_get_bitrate(struct video_client_ctx *client_ctx,
		struct venc_targetbitrate *venc_bitrate, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_target_bitrate_type bit_rate;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !venc_bitrate)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_TARGET_BITRATE;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_target_bitrate_type);
	if (set_flag) {
		bit_rate.n_target_bitrate = venc_bitrate->target_bitrate;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &bit_rate);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_TARGET_BITRATE Failed\n",
					__func__);
			return FALSE;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &bit_rate);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_TARGET_BITRATE Failed\n",
					__func__);
			return FALSE;
		}
		venc_bitrate->target_bitrate = bit_rate.n_target_bitrate;
	}
	return TRUE;
}

u32 vid_enc_set_get_framerate(struct video_client_ctx *client_ctx,
		struct venc_framerate *frame_rate, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_frame_rate_type vcd_frame_rate;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !frame_rate)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_FRAME_RATE;
	vcd_property_hdr.n_size =
				sizeof(struct vcd_property_frame_rate_type);

	if (set_flag) {
		vcd_frame_rate.n_fps_denominator = frame_rate->fps_denominator;
		vcd_frame_rate.n_fps_numerator = frame_rate->fps_numerator;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &vcd_frame_rate);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_FRAME_RATE Failed\n",
					__func__);
			return FALSE;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &vcd_frame_rate);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_FRAME_RATE Failed\n",
					__func__);
			return FALSE;
		}
		frame_rate->fps_denominator = vcd_frame_rate.n_fps_denominator;
		frame_rate->fps_numerator = vcd_frame_rate.n_fps_numerator;
	}
	return TRUE;
}

u32 vid_enc_set_get_live_mode(struct video_client_ctx *client_ctx,
		struct venc_switch *encoder_switch, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_live_type live_mode;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_LIVE;
	vcd_property_hdr.n_size =
				sizeof(struct vcd_property_live_type);

	if (set_flag) {
		live_mode.b_live = 1;
		if (!encoder_switch->status)
			live_mode.b_live = 0;

		vcd_status = vcd_set_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &live_mode);
		if (vcd_status) {
			ERR("%s(): Set VCD_I_LIVE Failed\n",
					__func__);
			return FALSE;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &live_mode);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_LIVE Failed\n",
					__func__);
			return FALSE;
		}	else {
			encoder_switch->status = 1;
			if (!live_mode.b_live)
				encoder_switch->status = 0;
		}
	}
	return TRUE;
}

u32 vid_enc_set_get_short_header(struct video_client_ctx *client_ctx,
		struct venc_switch *encoder_switch,	u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_short_header_type short_header;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !encoder_switch)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_SHORT_HEADER;
	vcd_property_hdr.n_size =
				sizeof(struct vcd_property_short_header_type);

	if (set_flag) {
		short_header.b_short_header = (u32) encoder_switch->status;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &short_header);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_SHORT_HEADER Failed\n",
					__func__);
			return FALSE;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &short_header);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_SHORT_HEADER Failed\n",
					__func__);
			return FALSE;
		}	else {
			encoder_switch->status =
				(u8) short_header.b_short_header;
		}
	}
	return TRUE;
}

u32 vid_enc_set_get_profile(struct video_client_ctx *client_ctx,
		struct venc_profile *profile, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_profile_type profile_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !profile)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_PROFILE;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_profile_type);

	if (set_flag) {
		switch (profile->profile) {
		case VEN_PROFILE_MPEG4_SP:
			profile_type.e_profile = VCD_PROFILE_MPEG4_SP;
			break;
		case VEN_PROFILE_MPEG4_ASP:
			profile_type.e_profile = VCD_PROFILE_MPEG4_ASP;
			break;
		case VEN_PROFILE_H264_BASELINE:
			profile_type.e_profile = VCD_PROFILE_H264_BASELINE;
			break;
		case VEN_PROFILE_H264_MAIN:
			profile_type.e_profile = VCD_PROFILE_H264_MAIN;
			break;
		case VEN_PROFILE_H264_HIGH:
			profile_type.e_profile = VCD_PROFILE_H264_HIGH;
			break;
		case VEN_PROFILE_H263_BASELINE:
			profile_type.e_profile = VCD_PROFILE_H263_BASELINE;
			break;
		default:
			status = FALSE;
			break;
		}

		if (status) {
			vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &profile_type);

			if (vcd_status) {
				ERR("%s(): Set VCD_I_PROFILE Failed\n",
						__func__);
				return FALSE;
			}
		}
	}	else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &profile_type);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_PROFILE Failed\n",
					__func__);
			return FALSE;
		} else {
			switch (profile_type.e_profile) {
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
				status = FALSE;
				break;
			}
		}
	}
	return status;
}

u32 vid_enc_set_get_profile_level(struct video_client_ctx *client_ctx,
		struct ven_profilelevel *profile_level,	u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_level_type level_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !profile_level)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_LEVEL;
	vcd_property_hdr.n_size =
			sizeof(struct vcd_property_level_type);

	if (set_flag) {
		switch (profile_level->level) {
		case VEN_LEVEL_MPEG4_0:
			level_type.e_level = VCD_LEVEL_MPEG4_0;
			break;
		case VEN_LEVEL_MPEG4_1:
			level_type.e_level = VCD_LEVEL_MPEG4_1;
			break;
		case VEN_LEVEL_MPEG4_2:
			level_type.e_level = VCD_LEVEL_MPEG4_2;
			break;
		case VEN_LEVEL_MPEG4_3:
			level_type.e_level = VCD_LEVEL_MPEG4_3;
			break;
		case VEN_LEVEL_MPEG4_4:
			level_type.e_level = VCD_LEVEL_MPEG4_4;
			break;
		case VEN_LEVEL_MPEG4_5:
			level_type.e_level = VCD_LEVEL_MPEG4_5;
			break;
		case VEN_LEVEL_MPEG4_3b:
			level_type.e_level = VCD_LEVEL_MPEG4_3b;
			break;
		case VEN_LEVEL_MPEG4_6:
			level_type.e_level = VCD_LEVEL_MPEG4_6;
			break;
		case VEN_LEVEL_H264_1:
			level_type.e_level = VCD_LEVEL_H264_1;
			break;
		case VEN_LEVEL_H264_1b:
			level_type.e_level = VCD_LEVEL_H264_1b;
			break;
		case VEN_LEVEL_H264_1p1:
			level_type.e_level = VCD_LEVEL_H264_1p1;
			break;
		case VEN_LEVEL_H264_1p2:
			level_type.e_level = VCD_LEVEL_H264_1p2;
			break;
		case VEN_LEVEL_H264_1p3:
			level_type.e_level = VCD_LEVEL_H264_1p3;
			break;
		case VEN_LEVEL_H264_2:
			level_type.e_level = VCD_LEVEL_H264_2;
			break;
		case VEN_LEVEL_H264_2p1:
			level_type.e_level = VCD_LEVEL_H264_2p1;
			break;
		case VEN_LEVEL_H264_2p2:
			level_type.e_level = VCD_LEVEL_H264_2p2;
			break;
		case VEN_LEVEL_H264_3:
			level_type.e_level = VCD_LEVEL_H264_3;
			break;
		case VEN_LEVEL_H264_3p1:
			level_type.e_level = VCD_LEVEL_H264_3p1;
			break;

		case VEN_LEVEL_H263_10:
			level_type.e_level = VCD_LEVEL_H263_10;
			break;
		case VEN_LEVEL_H263_20:
			level_type.e_level = VCD_LEVEL_H263_20;
			break;
		case VEN_LEVEL_H263_30:
			level_type.e_level = VCD_LEVEL_H263_30;
			break;
		case VEN_LEVEL_H263_40:
			level_type.e_level = VCD_LEVEL_H263_40;
			break;
		case VEN_LEVEL_H263_45:
			level_type.e_level = VCD_LEVEL_H263_45;
			break;
		case VEN_LEVEL_H263_50:
			level_type.e_level = VCD_LEVEL_H263_50;
			break;
		case VEN_LEVEL_H263_60:
			level_type.e_level = VCD_LEVEL_H263_60;
			break;
		case VEN_LEVEL_H263_70:
			level_type.e_level = VCD_LEVEL_H263_70;
			break;
		default:
			status = FALSE;
			break;
		}
		if (status) {
			vcd_status = vcd_set_property(client_ctx->vcd_handle,
						&vcd_property_hdr, &level_type);

			if (vcd_status) {
				ERR("%s(): Set VCD_I_LEVEL Failed\n",
						__func__);
				return FALSE;
			}
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &level_type);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_LEVEL Failed\n",
					__func__);
			return FALSE;
		} else {
			switch (level_type.e_level) {
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
				status = FALSE;
				break;
			case VCD_LEVEL_H264_4:
				status = FALSE;
				break;
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
				status = FALSE;
				break;
			default:
				status = FALSE;
				break;
			}
		}
	}
	return status;
}

u32 vid_enc_set_get_session_qp(struct video_client_ctx *client_ctx,
		struct venc_sessionqp *session_qp, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_session_qp_type qp_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !session_qp)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_SESSION_QP;
	vcd_property_hdr.n_size =
			sizeof(struct vcd_property_session_qp_type);

	if (set_flag) {
		qp_type.n_i_frame_qp = session_qp->iframeqp;
		qp_type.n_p_frame_qp = session_qp->pframqp;

		vcd_status = vcd_set_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &qp_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_SESSION_QP Failed\n",
					__func__);
			return FALSE;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &qp_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_SESSION_QP Failed\n",
					__func__);
			return FALSE;
		} else {
			session_qp->iframeqp = qp_type.n_i_frame_qp;
			session_qp->pframqp = qp_type.n_p_frame_qp;
		}
	}
	return TRUE;
}

u32 vid_enc_set_get_intraperiod(struct video_client_ctx *client_ctx,
		struct venc_intraperiod *intraperiod,	u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_i_period_type period_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !intraperiod)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_INTRA_PERIOD;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_i_period_type);

	if (set_flag) {
		period_type.n_p_frames = intraperiod->num_pframes;
		period_type.n_b_frames = 0;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &period_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_INTRA_PERIOD Failed\n",
					__func__);
			return FALSE;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &period_type);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_INTRA_PERIOD Failed\n",
					__func__);
			return FALSE;
		} else
			intraperiod->num_pframes = period_type.n_p_frames;
	}
	return TRUE;
}

u32 vid_enc_request_iframe(struct video_client_ctx *client_ctx)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_req_i_frame_type request;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_REQ_IFRAME;
	vcd_property_hdr.n_size =
				sizeof(struct vcd_property_req_i_frame_type);
	request.b_req_i_frame = 1;

	vcd_status = vcd_set_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &request);

	if (vcd_status) {
		ERR("%s(): Set VCD_I_REQ_IFRAME Failed\n",
				__func__);
		return FALSE;
	}
	return status;
}

u32 vid_enc_get_sequence_header(struct video_client_ctx *client_ctx,
		struct venc_seqheader	*seq_header)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_sequence_hdr_type hdr_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx ||
			!seq_header || !seq_header->bufsize)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_SEQ_HEADER;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_sequence_hdr_type);

	hdr_type.p_sequence_header =
		kzalloc(seq_header->bufsize, GFP_KERNEL);
	seq_header->hdrbufptr = hdr_type.p_sequence_header;

	if (!hdr_type.p_sequence_header)
		return FALSE;
	hdr_type.n_sequence_header_len = seq_header->bufsize;
	vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &hdr_type);

	if (vcd_status) {
		ERR("%s(): Get VCD_I_SEQ_HEADER Failed\n",
				__func__);
		status = FALSE;
	}
	return TRUE;
}

u32 vid_enc_set_get_entropy_cfg(struct video_client_ctx *client_ctx,
		struct venc_entropycfg *entropy_cfg, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_entropy_control_type control_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !entropy_cfg)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_ENTROPY_CTRL;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_entropy_control_type);

	if (set_flag) {
		switch (entropy_cfg->cabacmodel) {
		case VEN_ENTROPY_MODEL_CAVLC:
			control_type.e_entropy_sel = VCD_ENTROPY_SEL_CAVLC;
			break;
		case VEN_ENTROPY_MODEL_CABAC:
			control_type.e_entropy_sel = VCD_ENTROPY_SEL_CABAC;
			break;
		default:
			status = FALSE;
			break;
		}

		if (status && entropy_cfg->cabacmodel ==
				VCD_ENTROPY_SEL_CABAC) {
			switch (entropy_cfg->cabacmodel) {
			case VEN_CABAC_MODEL_0:
				control_type.e_cabac_model =
					VCD_CABAC_MODEL_NUMBER_0;
				break;
			case VEN_CABAC_MODEL_1:
				control_type.e_cabac_model =
					VCD_CABAC_MODEL_NUMBER_1;
				break;
			case VEN_CABAC_MODEL_2:
				control_type.e_cabac_model =
					VCD_CABAC_MODEL_NUMBER_2;
				break;
			default:
				status = FALSE;
				break;
			}
		}
		if (status) {
			vcd_status = vcd_set_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &control_type);

			if (vcd_status) {
				ERR("%s(): Set VCD_I_ENTROPY_CTRL Failed\n",
						__func__);
				status = FALSE;
			}
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_ENTROPY_CTRL Failed\n",
					__func__);
			status = FALSE;
		} else {
			switch (control_type.e_entropy_sel) {
			case VCD_ENTROPY_SEL_CABAC:
				entropy_cfg->cabacmodel =
					VEN_ENTROPY_MODEL_CABAC;
				break;
			case VCD_ENTROPY_SEL_CAVLC:
				entropy_cfg->cabacmodel =
					VEN_ENTROPY_MODEL_CAVLC;
				break;
			default:
				status = FALSE;
				break;
			}

			if (status && control_type.e_entropy_sel ==
					VCD_ENTROPY_SEL_CABAC) {
				switch (control_type.e_cabac_model) {
				case VCD_CABAC_MODEL_NUMBER_0:
					entropy_cfg->cabacmodel =
						VEN_CABAC_MODEL_0;
					break;
				case VCD_CABAC_MODEL_NUMBER_1:
					entropy_cfg->cabacmodel =
						VEN_CABAC_MODEL_1;
					break;
				case VCD_CABAC_MODEL_NUMBER_2:
					entropy_cfg->cabacmodel =
						VEN_CABAC_MODEL_2;
					break;
				default:
					status = FALSE;
					break;
				}
			}
		}
	}
	return status;
}

u32 vid_enc_set_get_dbcfg(struct video_client_ctx *client_ctx,
		struct venc_dbcfg *dbcfg, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_db_config_type control_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !dbcfg)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_DEBLOCKING;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_db_config_type);

	if (set_flag) {
		switch (dbcfg->db_mode) {
		case VEN_DB_DISABLE:
			control_type.e_db_config = VCD_DB_DISABLE;
			break;
		case VEN_DB_ALL_BLKG_BNDRY:
			control_type.e_db_config = VCD_DB_ALL_BLOCKING_BOUNDARY;
			break;
		case VEN_DB_SKIP_SLICE_BNDRY:
			control_type.e_db_config = VCD_DB_SKIP_SLICE_BOUNDARY;
			break;
		default:
			status = FALSE;
			break;
		}

		if (status) {
			control_type.n_slice_alpha_offset =
				dbcfg->slicealpha_offset;
			control_type.n_slice_beta_offset =
				dbcfg->slicebeta_offset;
			vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);
			if (vcd_status) {
				ERR("%s(): Set VCD_I_DEBLOCKING Failed\n",
						__func__);
				status = FALSE;
			}
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s(): Get VCD_I_DEBLOCKING Failed\n",
					__func__);
			status = FALSE;
		} else {
			switch (control_type.e_db_config) {
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
				status = FALSE;
				break;
			}
			dbcfg->slicealpha_offset =
				control_type.n_slice_alpha_offset;
			dbcfg->slicebeta_offset =
				control_type.n_slice_beta_offset;
		}
	}
	return status;
}

u32 vid_enc_set_get_intrarefresh(struct video_client_ctx *client_ctx,
		struct venc_intrarefresh *intrarefresh, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_intra_refresh_mb_number_type control_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !intrarefresh)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_INTRA_REFRESH;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_intra_refresh_mb_number_type);

	if (set_flag) {
		control_type.n_cir_mb_number = intrarefresh->mbcount;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
				&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_INTRA_REFRESH Failed\n",
					__func__);
			return FALSE;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_INTRA_REFRESH Failed\n",
					__func__);
			return FALSE;
		} else
			intrarefresh->mbcount = control_type.n_cir_mb_number;
	}
	return TRUE;
}

u32 vid_enc_set_get_multiclicecfg(struct video_client_ctx *client_ctx,
		struct venc_multiclicecfg *multiclicecfg,	u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_multi_slice_type control_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !multiclicecfg)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_MULTI_SLICE;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_multi_slice_type);

	if (set_flag) {
		switch (multiclicecfg->mslice_mode) {
		case VEN_MSLICE_OFF:
			control_type.e_m_slice_sel =
				VCD_MSLICE_OFF;
			break;
		case VEN_MSLICE_CNT_MB:
			control_type.e_m_slice_sel =
				VCD_MSLICE_BY_MB_COUNT;
			break;
		case VEN_MSLICE_CNT_BYTE:
			control_type.e_m_slice_sel =
				VCD_MSLICE_BY_BYTE_COUNT;
			break;
		case VEN_MSLICE_GOB:
			control_type.e_m_slice_sel =
				VCD_MSLICE_BY_GOB;
			break;
		default:
			status = FALSE;
			break;
		}

		if (status) {
			control_type.n_m_slice_size =
				multiclicecfg->mslice_size;
			vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);

			if (vcd_status) {
				ERR("%s(): Set VCD_I_MULTI_SLICE Failed\n",
						__func__);
				status = FALSE;
			}
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_MULTI_SLICE Failed\n",
					__func__);
			status = FALSE;
		} else {
			multiclicecfg->mslice_size =
				control_type.n_m_slice_size;
			switch (control_type.e_m_slice_sel) {
			case VCD_MSLICE_OFF:
				multiclicecfg->mslice_mode = VEN_MSLICE_OFF;
				break;
			case VCD_MSLICE_BY_MB_COUNT:
				multiclicecfg->mslice_mode = VEN_MSLICE_CNT_MB;
				break;
			case VCD_MSLICE_BY_BYTE_COUNT:
				multiclicecfg->mslice_mode =
					VEN_MSLICE_CNT_BYTE;
				break;
			case VCD_MSLICE_BY_GOB:
				multiclicecfg->mslice_mode =
					VEN_MSLICE_GOB;
				break;
			default:
				status = FALSE;
				break;
			}
		}
	}
	return status;
}

u32 vid_enc_set_get_ratectrlcfg(struct video_client_ctx *client_ctx,
		struct venc_ratectrlcfg *ratectrlcfg,	u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_rate_control_type control_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !ratectrlcfg)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_RATE_CONTROL;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_rate_control_type);

	if (set_flag) {
		switch (ratectrlcfg->rcmode) {
		case VEN_RC_OFF:
			control_type.e_rate_control = VCD_RATE_CONTROL_OFF;
			break;
		case VEN_RC_CBR_VFR:
			control_type.e_rate_control = VCD_RATE_CONTROL_CBR_VFR;
			break;
		case VEN_RC_VBR_CFR:
			control_type.e_rate_control = VCD_RATE_CONTROL_VBR_CFR;
			break;
		case VEN_RC_VBR_VFR:
			control_type.e_rate_control = VCD_RATE_CONTROL_VBR_VFR;
			break;
		default:
			status = FALSE;
			break;
		}

		if (status) {
			vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &control_type);
			if (vcd_status) {
				ERR("%s(): Set VCD_I_RATE_CONTROL Failed\n",
						__func__);
				status = FALSE;
			}
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
		&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s(): Get VCD_I_RATE_CONTROL Failed\n",
					__func__);
			status = FALSE;
		} else {
			switch (control_type.e_rate_control) {
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
				status = FALSE;
				break;
			}
		}
	}
	return status;
}

u32 vid_enc_set_get_voptimingcfg(struct video_client_ctx *client_ctx,
		struct	venc_voptimingcfg *voptimingcfg, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_vop_timing_type control_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !voptimingcfg)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_VOP_TIMING;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_vop_timing_type);

	if (set_flag) {
		control_type.n_vop_time_resolution =
		voptimingcfg->voptime_resolution;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
		&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_VOP_TIMING Failed\n",
					__func__);
			status = FALSE;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
		&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s(): Get VCD_I_VOP_TIMING Failed\n",
					__func__);
			status = FALSE;
		} else
			voptimingcfg->voptime_resolution =
			control_type.n_vop_time_resolution;
	}
	return status;
}

u32 vid_enc_set_get_headerextension(struct video_client_ctx *client_ctx,
		struct venc_headerextension *headerextension, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	u32 control_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !headerextension)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_HEADER_EXTENSION;
	vcd_property_hdr.n_size = sizeof(u32);

	if (set_flag) {
		control_type = headerextension->header_extension;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
		&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s(): Set VCD_I_HEADER_EXTENSION Failed\n",
					__func__);
			status = FALSE;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
		&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s(): Get VCD_I_HEADER_EXTENSION Failed\n",
					__func__);
			status = FALSE;
		} else {
			headerextension->header_extension = control_type;
		}
	}
	return status;
}

u32 vid_enc_set_get_qprange(struct video_client_ctx *client_ctx,
		struct venc_qprange *qprange, u32 set_flag)
{
	struct vcd_property_hdr_type vcd_property_hdr;
	struct vcd_property_qp_range_type control_type;
	u32 vcd_status = VCD_ERR_FAIL;
	u32 status = TRUE;

	if (!client_ctx || !qprange)
		return FALSE;

	vcd_property_hdr.prop_id = VCD_I_QP_RANGE;
	vcd_property_hdr.n_size =
		sizeof(struct vcd_property_qp_range_type);

	if (set_flag) {
		control_type.n_max_qp = qprange->maxqp;
		control_type.n_min_qp = qprange->minqp;
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
		&vcd_property_hdr, &control_type);

		if (vcd_status) {
			ERR("%s(): Set VCD_I_QP_RANGE Failed\n",
					__func__);
			status = FALSE;
		}
	} else {
		vcd_status = vcd_get_property(client_ctx->vcd_handle,
					&vcd_property_hdr, &control_type);
		if (vcd_status) {
			ERR("%s(): Get VCD_I_QP_RANGE Failed\n",
					__func__);
			status = FALSE;
		} else {
			qprange->maxqp = control_type.n_max_qp;
			qprange->minqp = control_type.n_min_qp;
		}
	}
	return status;
}

u32 vid_enc_start_stop(struct video_client_ctx *client_ctx, u32 start)
{
	u32 vcd_status;

	if (!client_ctx)
		return FALSE;

	if (start) {
			vcd_status = vcd_encode_start(client_ctx->vcd_handle);

			if (vcd_status) {
				ERR("%s(): vcd_encode_start failed."
				" vcd_status = %u\n", __func__, vcd_status);
				return FALSE;
			}
	} else {
		vcd_status = vcd_stop(client_ctx->vcd_handle);
		if (vcd_status) {
			ERR("%s(): vcd_stop failed.  vcd_status = %u\n",
		__func__, vcd_status);
			return FALSE;
		}
		DBG("Send STOP_DONE message to client = %p\n",
				client_ctx);
	}
	return TRUE;
}

u32 vid_enc_pause_resume(struct video_client_ctx *client_ctx, u32 pause)
{
	u32 vcd_status;

	if (!client_ctx)
		return FALSE;

	if (pause) {
		DBG("PAUSE command from client = %p\n",
				client_ctx);
		vcd_status = vcd_pause(client_ctx->vcd_handle);
	} else {
		DBG("Resume command from client = %p\n",
				client_ctx);
		vcd_status = vcd_resume(client_ctx->vcd_handle);
	}

	if (vcd_status)
		return FALSE;

	return TRUE;
}

u32 vid_enc_flush(struct video_client_ctx *client_ctx,
		struct venc_bufferflush *bufferflush)
{
	u32 status = TRUE, n_mode, vcd_status;

	if (!client_ctx || !bufferflush)
		return FALSE;

	switch (bufferflush->flush_mode) {
	case VEN_FLUSH_INPUT:
		n_mode = VCD_FLUSH_INPUT;
		break;
	case VEN_FLUSH_OUTPUT:
		n_mode = VCD_FLUSH_OUTPUT;
		break;
	case VEN_FLUSH_ALL:
		n_mode = VCD_FLUSH_ALL;
		break;
	default:
		status = FALSE;
		break;
	}
	if (status) {
		vcd_status = vcd_flush(client_ctx->vcd_handle, n_mode);
		if (vcd_status)
			status = FALSE;
	}
	return status;
}

u32 vid_enc_get_buffer_req(struct video_client_ctx *client_ctx,
		struct venc_allocatorproperty *venc_buf_req, u32 input_dir)
{
	enum vcd_buffer_type e_buffer;
	struct vcd_buffer_requirement_type buffer_req;
	u32 status = TRUE;
	u32 vcd_status;

	if (!client_ctx || !venc_buf_req)
		return FALSE;

	e_buffer = VCD_BUFFER_OUTPUT;
	if (input_dir)
		e_buffer = VCD_BUFFER_INPUT;

	vcd_status = vcd_get_buffer_requirements(client_ctx->vcd_handle,
							e_buffer, &buffer_req);

	if (vcd_status)
		status = FALSE;

	if (status) {
		venc_buf_req->actualcount = buffer_req.n_actual_count;
		venc_buf_req->alignment = buffer_req.n_align;
		venc_buf_req->datasize = buffer_req.n_size;
		venc_buf_req->mincount = buffer_req.n_min_count;
		venc_buf_req->maxcount = buffer_req.n_max_count;
		venc_buf_req->alignment = buffer_req.n_align;
		venc_buf_req->bufpoolid = buffer_req.n_buf_pool_id;
		venc_buf_req->suffixsize = 0;
	}
	return status;
}

u32 vid_enc_set_buffer_req(struct video_client_ctx *client_ctx,
		struct venc_allocatorproperty *venc_buf_req, u32 input_dir)
{
	enum vcd_buffer_type e_buffer;
	struct vcd_buffer_requirement_type buffer_req;
	u32 status = TRUE;
	u32 vcd_status;

	if (!client_ctx || !venc_buf_req)
		return FALSE;

	e_buffer = VCD_BUFFER_OUTPUT;
	if (input_dir)
		e_buffer = VCD_BUFFER_INPUT;

	buffer_req.n_actual_count = venc_buf_req->actualcount;
	buffer_req.n_align = venc_buf_req->alignment;
	buffer_req.n_size = venc_buf_req->datasize;
	buffer_req.n_min_count = venc_buf_req->mincount;
	buffer_req.n_max_count = venc_buf_req->maxcount;
	buffer_req.n_align = venc_buf_req->alignment;
	buffer_req.n_buf_pool_id = 0;

	vcd_status = vcd_set_buffer_requirements(client_ctx->vcd_handle,
				e_buffer, &buffer_req);

	if (vcd_status)
		status = FALSE;
	return status;
}

u32 vid_enc_set_buffer(struct video_client_ctx *client_ctx,
		struct venc_bufferpayload *buffer_info,
		enum venc_buffer_dir buffer_type)
{
	u32 vcd_status = VCD_ERR_FAIL;
	enum vcd_buffer_type buffer_vcd_type;
	enum buffer_dir dir_buffer = BUFFER_TYPE_INPUT;
	unsigned long user_vaddr, kernel_vaddr, phy_addr, len;
	int pmem_fd;
	struct file *file;
	struct buf_addr_table *buf_addr_table;

	s32 buffer_index = -1;

	if (!client_ctx || !buffer_info)
		return FALSE;

	user_vaddr = (unsigned long)buffer_info->pbuffer;

	if (buffer_type == VEN_BUFFER_TYPE_OUTPUT)
		dir_buffer = BUFFER_TYPE_OUTPUT;

	/*If buffer already set, ignore */
	if (vid_c_lookup_addr_table(client_ctx, dir_buffer,
			TRUE, &user_vaddr, &kernel_vaddr,
			&phy_addr, &pmem_fd, &file,
			&buffer_index)) {

		DBG("%s() : user_virt_addr = 0x%08lx is already set.",
				__func__, user_vaddr);
		return TRUE;
	}

	if (get_pmem_file(buffer_info->fd,
				&phy_addr, &kernel_vaddr, &len, &file)) {
		ERR("%s(): get_pmem_file failed\n", __func__);
		return FALSE;
	}
	put_pmem_file(file);
	if (buffer_type == VEN_BUFFER_TYPE_INPUT) {
		buffer_vcd_type = VCD_BUFFER_INPUT;
		client_ctx->num_of_input_buffers++;

		if (client_ctx->num_of_input_buffers >
				VID_ENC_MAX_NUM_OF_BUFF) {
			ERR("%s(): num_of_input_buffers reached max value"
			" VID_ENC_MAX_NUM_OF_BUFF \n", __func__);
			client_ctx->num_of_input_buffers--;
			return FALSE;
		}

		buffer_index = client_ctx->num_of_input_buffers - 1;
		buf_addr_table =
				&client_ctx->input_buf_addr_table[buffer_index];

		buf_addr_table->user_vaddr =
			(unsigned long)buffer_info->pbuffer;
		kernel_vaddr += (unsigned long)buffer_info->offset;
		phy_addr += (unsigned long)buffer_info->offset;
		buf_addr_table->kernel_vaddr = kernel_vaddr;
		buf_addr_table->phy_addr = phy_addr;
		buf_addr_table->pmem_fd = buffer_info->fd;
		buf_addr_table->file = file;
	} else {
		buffer_vcd_type = VCD_BUFFER_OUTPUT;

		client_ctx->num_of_output_buffers++;

		if (client_ctx->num_of_output_buffers >
				VID_ENC_MAX_NUM_OF_BUFF) {
			ERR("%s(): num_of_outut_buffers reached max value"
			" VID_ENC_MAX_NUM_OF_BUFF \n", __func__);
			client_ctx->num_of_output_buffers--;
			return FALSE;
		}

		buffer_index = client_ctx->num_of_output_buffers - 1;

		buf_addr_table =
			&client_ctx->output_buf_addr_table[buffer_index];
		kernel_vaddr += (unsigned long)buffer_info->offset;
		phy_addr += (unsigned long)buffer_info->offset;
		buf_addr_table->user_vaddr =
			(unsigned long)buffer_info->pbuffer;
		buf_addr_table->kernel_vaddr = kernel_vaddr;
		buf_addr_table->phy_addr = phy_addr;
		buf_addr_table->pmem_fd = buffer_info->fd;
		buf_addr_table->file = file;
	}

	vcd_status = vcd_set_buffer(client_ctx->vcd_handle,
	buffer_vcd_type, (u8 *) kernel_vaddr,
	buffer_info->nsize);

	if (!vcd_status)
		return TRUE;
	else
		return FALSE;
}

u32 vid_enc_encode_frame(struct video_client_ctx *client_ctx,
		struct venc_buffer *input_frame_info)
{
	struct vcd_frame_data_type vcd_input_buffer;
	unsigned long kernel_vaddr, phy_addr, user_vaddr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;

	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !input_frame_info)
		return FALSE;

	user_vaddr = (unsigned long)input_frame_info->ptrbuffer;

	if (vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_INPUT,
			TRUE, &user_vaddr, &kernel_vaddr,
			&phy_addr, &pmem_fd, &file,
			&buffer_index)) {

		/* kernel_vaddr  is found. send the frame to VCD */
		memset((void *)&vcd_input_buffer, 0,
					sizeof(struct vcd_frame_data_type));

		vcd_input_buffer.p_virtual =
		(u8 *) (kernel_vaddr + input_frame_info->offset);

		vcd_input_buffer.n_offset = input_frame_info->offset;
		vcd_input_buffer.n_frm_clnt_data =
				(u32) input_frame_info->clientdata;
		vcd_input_buffer.n_ip_frm_tag =
				(u32) input_frame_info->clientdata;
		vcd_input_buffer.n_data_len = input_frame_info->len;
		vcd_input_buffer.time_stamp = input_frame_info->timestamp;

		/* Rely on VCD using the same flags as OMX */
		vcd_input_buffer.n_flags = input_frame_info->flags;

		vcd_status = vcd_encode_frame(client_ctx->vcd_handle,
		&vcd_input_buffer);
		if (!vcd_status)
			return TRUE;
		else {
			ERR("%s(): vcd_encode_frame failed = %u\n",
			__func__, vcd_status);
			return FALSE;
		}

	} else {
		ERR("%s(): kernel_vaddr not found\n",
				__func__);
		return FALSE;
	}
}

u32 vid_enc_fill_output_buffer(struct video_client_ctx *client_ctx,
		struct venc_buffer *output_frame_info)
{
	unsigned long kernel_vaddr, phy_addr, user_vaddr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;
	u32 vcd_status = VCD_ERR_FAIL;

	struct vcd_frame_data_type vcd_frame;

	if (!client_ctx || !output_frame_info)
		return FALSE;

	user_vaddr = (unsigned long)output_frame_info->ptrbuffer;

	if (vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_OUTPUT,
			TRUE, &user_vaddr, &kernel_vaddr,
			&phy_addr, &pmem_fd, &file,
			&buffer_index)) {

		memset((void *)&vcd_frame, 0,
					 sizeof(struct vcd_frame_data_type));
		vcd_frame.p_virtual = (u8 *) kernel_vaddr;
		vcd_frame.n_frm_clnt_data = (u32) output_frame_info->clientdata;
		vcd_frame.n_alloc_len = output_frame_info->size;

		vcd_status = vcd_fill_output_buffer(client_ctx->vcd_handle,
								&vcd_frame);
		if (!vcd_status)
			return TRUE;
		else {
			ERR("%s(): vcd_fill_output_buffer failed = %u\n",
					__func__, vcd_status);
			return FALSE;
		}
	} else {
		ERR("%s(): kernel_vaddr not found\n", __func__);
		return FALSE;
	}
}
