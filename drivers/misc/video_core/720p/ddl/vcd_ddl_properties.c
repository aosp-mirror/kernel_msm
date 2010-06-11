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

#include "video_core_type.h"

#include "vcd_ddl_utils.h"
#include "vcd_ddl_metadata.h"

static u32 ddl_set_dec_property(struct ddl_client_context *pddl,
	struct vcd_property_hdr *hdr, void *value);
static u32 ddl_set_enc_property(struct ddl_client_context *pddl,
	struct vcd_property_hdr *hdr, void *value);
static u32 ddl_get_dec_property(struct ddl_client_context *pddl,
	struct vcd_property_hdr *hdr, void *value);
static u32 ddl_get_enc_property(struct ddl_client_context *pddl,
	struct vcd_property_hdr *hdr, void *value);
static u32 ddl_set_enc_dynamic_property(struct ddl_encoder_data *enc,
	struct vcd_property_hdr *hdr, void *value);
static void ddl_set_default_enc_property(struct ddl_client_context *ddl);
static void ddl_set_default_enc_profile(struct ddl_encoder_data *enc);
static void ddl_set_default_enc_level(struct ddl_encoder_data *enc);
static void ddl_set_default_enc_vop_timing(struct ddl_encoder_data *enc);
static void ddl_set_default_enc_intra_period(struct ddl_encoder_data *enc);
static void ddl_set_default_enc_rc_params(struct ddl_encoder_data *enc);
static u32 ddl_valid_buffer_requirement(struct vcd_buffer_requirement
	*orig, struct vcd_buffer_requirement *req);
static u32 ddl_decoder_min_num_dpb(struct ddl_decoder_data *dec);
static u32 ddl_set_dec_buffers(struct ddl_decoder_data *dec,
	struct ddl_property_dec_pic_buffers *dpb);

u32 ddl_set_property(u32 *ddl_handle, struct vcd_property_hdr *hdr,
	void *value)
{
	u32 vcd_status;
	struct ddl_context *ddl_context;
	struct ddl_client_context *ddl = (struct ddl_client_context *)
		ddl_handle;

	if (!hdr || !value) {
		pr_err("ddl_set_prop:Bad_argument\n");
		return VCD_ERR_ILLEGAL_PARM;
	}
	ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(ddl_context)) {
		pr_err("ddl_set_prop:Not_inited\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (!ddl) {
		pr_err("ddl_set_prop:Bad_handle\n");
		return VCD_ERR_BAD_HANDLE;
	}
	if (ddl->decoding)
		vcd_status = ddl_set_dec_property(ddl, hdr, value);
	else
		vcd_status = ddl_set_enc_property(ddl, hdr, value);
	if (vcd_status)
		pr_err("ddl_set_prop:FAILED\n");

	return vcd_status;
}

u32 ddl_get_property(u32 *ddl_handle, struct vcd_property_hdr *hdr, void *value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_context *ddl_context;
	struct ddl_client_context *ddl = (struct ddl_client_context *)
		ddl_handle;

	if (!hdr || !value)
		return VCD_ERR_ILLEGAL_PARM;

	if (hdr->id == DDL_I_CAPABILITY) {
		struct ddl_property_capability *cap;
		if (sizeof(*cap) == hdr->sz) {
			cap = value;
			cap->max_num_client = VCD_MAX_NO_CLIENT;
			cap->exclusive = VCD_COMMAND_EXCLUSIVE;
			cap->frame_command_depth = VCD_FRAME_COMMAND_DEPTH;
			cap->general_command_depth = VCD_GENERAL_COMMAND_DEPTH;
			cap->ddl_time_out_in_ms = DDL_HW_TIMEOUT_IN_MS;
			vcd_status = VCD_S_SUCCESS;
		}
		return vcd_status;
	}
	ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(ddl_context))
		return VCD_ERR_ILLEGAL_OP;

	if (!ddl)
		return VCD_ERR_BAD_HANDLE;

	if (ddl->decoding)
		vcd_status = ddl_get_dec_property(ddl, hdr, value);
	else
		vcd_status = ddl_get_enc_property(ddl, hdr, value);
	if (vcd_status)
		pr_err("ddl_get_prop:FAILED\n");

	return vcd_status;
}

u32 ddl_decoder_ready_to_start(struct ddl_client_context *ddl,
	struct vcd_phys_sequence_hdr *seq_hdr)
{
	struct ddl_decoder_data *dec = &ddl->codec_data.decoder;
	if (!dec->codec_type.codec) {
		pr_err("ddl_dec_start_check:Codec_not_set\n");
		return false;
	}
	if (!seq_hdr && (!dec->client_frame_size.height ||
			!dec->client_frame_size.width)) {
		pr_err("ddl_dec_start_check:"
			"Client_height_width_default\n");
		return false;
	}
	return true;
}

u32 ddl_encoder_ready_to_start(struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *enc = &ddl->codec_data.encoder;

	if (!enc->codec_type.codec || !enc->frame_size.height ||
			!enc->frame_size.width ||
			!enc->frame_rate.fps_denominator ||
			!enc->frame_rate.fps_numerator ||
			!enc->target_bit_rate.target_bitrate) {
		return false;
	}
	return true;
}

static u32 ddl_set_dec_property(struct ddl_client_context *ddl,
		struct vcd_property_hdr *hdr, void *value) {
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_decoder_data *dec = &ddl->codec_data.decoder;
	switch (hdr->id) {
	case DDL_I_DPB_RELEASE:
		if (sizeof(struct ddl_frame_data_tag) == hdr->sz &&
				dec->dp_buf.no_of_dec_pic_buf) {
			vcd_status = ddl_decoder_dpb_transact(dec, value,
				DDL_DPB_OP_MARK_FREE);
		}
		break;
	case DDL_I_DPB:
	{
		struct ddl_property_dec_pic_buffers *dpb = value;
		if (sizeof(*dpb) == hdr->sz &&
				(DDLCLIENT_STATE_IS(ddl,
				DDL_CLIENT_WAIT_FOR_INITCODEC) ||
				DDLCLIENT_STATE_IS(ddl,
				DDL_CLIENT_WAIT_FOR_DPB)) &&
				dpb->no_of_dec_pic_buf >=
				dec->client_output_buf_req.actual_count) {
			vcd_status = ddl_set_dec_buffers(dec, dpb);
		}
		break;
	}
	case DDL_I_REQ_OUTPUT_FLUSH:
		if (sizeof(u32) == hdr->sz) {
			dec->dynamic_prop_change |= DDL_DEC_REQ_OUTPUT_FLUSH;
			dec->dpb_mask.client_mask = 0;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case DDL_I_INPUT_BUF_REQ:
	{
		struct vcd_buffer_requirement *buf_req = value;
		if (sizeof(*buf_req) == hdr->sz &&
				ddl_valid_buffer_requirement(
				&dec->min_input_buf_req, buf_req)) {
			dec->client_input_buf_req = *buf_req;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case DDL_I_OUTPUT_BUF_REQ:
	{
		struct vcd_buffer_requirement *buf_req = value;
		if (sizeof(*buf_req) == hdr->sz &&
				ddl_valid_buffer_requirement(
				&dec->min_output_buf_req, buf_req)) {
			dec->client_output_buf_req = *buf_req;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_CODEC:
	{
		struct vcd_property_codec *codec = value;
		if (sizeof(*codec) == hdr->sz &&
				DDLCLIENT_STATE_IS(ddl,	DDL_CLIENT_OPEN)) {
			if (!vcd_fw_is_codec_supported(true, codec->codec)) {
				vcd_status = VCD_ERR_NOT_SUPPORTED;
				break;
			}
			dec->codec_type = *codec;
			ddl_set_default_dec_property(ddl);
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_POST_FILTER:
		if (sizeof(struct vcd_property_post_filter) == hdr->sz &&
				DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN) &&
				(dec->codec_type.codec == VCD_CODEC_MPEG4 ||
				dec->codec_type.codec == VCD_CODEC_MPEG2)) {
			dec->post_filter = *(struct vcd_property_post_filter *)
				value;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_FRAME_SIZE:
	{
		struct vcd_property_frame_size *frame_size = value;
		if ((sizeof(*frame_size) == hdr->sz) &&
				DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)) {
			if (dec->client_frame_size.height != frame_size->height
					|| dec->client_frame_size.width !=
					frame_size->width) {
				dec->client_frame_size = *frame_size;
				ddl_calculate_stride(&dec->client_frame_size,
					!dec->progressive_only);
				ddl_set_default_decoder_buffer_req(dec, true);
			}
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_BUFFER_FORMAT:
	{
		struct vcd_property_buffer_format *tile = value;
		if (sizeof(*tile) == hdr->sz &&
				DDLCLIENT_STATE_IS(ddl,	DDL_CLIENT_OPEN) &&
				(tile->buffer_format ==	VCD_BUFFER_FORMAT_NV12
				|| tile->buffer_format ==
				VCD_BUFFER_FORMAT_TILE_4x2)) {
			if (tile->buffer_format !=
					dec->buf_format.buffer_format) {
				dec->buf_format = *tile;
				ddl_set_default_decoder_buffer_req(dec,	true);
			}
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		vcd_status = ddl_set_metadata_params(ddl, hdr, value);
		break;
	default:
		vcd_status = VCD_ERR_ILLEGAL_OP;
		break;
	}
	return vcd_status;
}

static u32 ddl_set_enc_property(struct ddl_client_context *ddl,
	struct vcd_property_hdr *hdr, void *value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_encoder_data *enc = &ddl->codec_data.encoder;

	if (DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME)) {
		vcd_status = ddl_set_enc_dynamic_property(enc, hdr, value);
		return vcd_status;
	}

	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)) {
		pr_err("ddl_set_enc_property:"
				"Fails_as_not_in_open_state\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	switch (hdr->id) {
	case VCD_I_TARGET_BITRATE:
	{
		struct vcd_property_target_bitrate *bitrate = value;
		if (sizeof(*bitrate) == hdr->sz &&
				bitrate->target_bitrate) {
			enc->target_bit_rate = *bitrate;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_FRAME_RATE:
	{
		struct vcd_property_frame_rate *framerate = value;
		if (sizeof(*framerate) == hdr->sz &&
				framerate->fps_denominator &&
				framerate->fps_numerator) {
			enc->frame_rate = *framerate;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_FRAME_SIZE:
	{
		struct vcd_property_frame_size *framesize = value;
		if (sizeof(*framesize) == hdr->sz &&
				DDL_ALLOW_ENC_FRAMESIZE(framesize->width,
				framesize->height)) {
			enc->frame_size = *framesize;
			ddl_calculate_stride(&enc->frame_size, false);
			ddl_set_default_encoder_buffer_req(enc);
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_CODEC:
	{
		struct vcd_property_codec *codec = value;
		if (sizeof(*codec) == hdr->sz) {
			if (!vcd_fw_is_codec_supported(false, codec->codec)) {
				vcd_status = VCD_ERR_NOT_SUPPORTED;
				break;
			}
			enc->codec_type = *codec;
			ddl_set_default_enc_property(ddl);
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_REQ_IFRAME:
		vcd_status = VCD_S_SUCCESS;
		break;
	case VCD_I_INTRA_PERIOD:
	{
		struct vcd_property_i_period *iperiod = value;
		if (sizeof(*iperiod) == hdr->sz && !iperiod->bframes) {
			enc->period = *iperiod;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_PROFILE:
	{
		struct vcd_property_profile *profile = value;
		if (sizeof(*profile) == hdr->sz &&
				((enc->codec_type.codec == VCD_CODEC_MPEG4 &&
				(profile->profile == VCD_PROFILE_MPEG4_SP ||
				profile->profile == VCD_PROFILE_MPEG4_ASP)) ||
				((enc->codec_type.codec == VCD_CODEC_H264 &&
				profile->profile >= VCD_PROFILE_H264_BASELINE &&
				profile->profile <= VCD_PROFILE_H264_HIGH)) ||
				(enc->codec_type.codec == VCD_CODEC_H263 &&
				profile->profile == VCD_PROFILE_H263_BASELINE))
				) {
			enc->profile = *profile;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_LEVEL:
	{
		struct vcd_property_level *level = value;
		if (sizeof(*level) == hdr->sz &&
				((enc->codec_type.codec == VCD_CODEC_MPEG4 &&
				level->level >= VCD_LEVEL_MPEG4_0 &&
				level->level <= VCD_LEVEL_MPEG4_6) ||
				(enc->codec_type.codec == VCD_CODEC_H264 &&
				level->level >= VCD_LEVEL_H264_1 &&
				level->level <= VCD_LEVEL_H264_3p1) ||
				(enc->codec_type.codec == VCD_CODEC_H263 &&
				level->level >= VCD_LEVEL_H263_10 &&
				level->level <= VCD_LEVEL_H263_70))) {
			enc->level = *level;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_MULTI_SLICE:
	{
		struct vcd_property_multi_slice *multislice = value;
		switch (multislice->m_slice_sel) {
		case VCD_MSLICE_OFF:
			vcd_status = VCD_S_SUCCESS;
			break;
		case VCD_MSLICE_BY_GOB:
			if (enc->codec_type.codec == VCD_CODEC_H263)
				vcd_status = VCD_S_SUCCESS;
			 break;
		case VCD_MSLICE_BY_MB_COUNT:
			if (multislice->m_slice_size >= 1 &&
					(multislice->m_slice_size <=
					(enc->frame_size.height
					* enc->frame_size.width	/ 16 / 16))) {
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		case VCD_MSLICE_BY_BYTE_COUNT:
			if (multislice->m_slice_size <
					DDL_MINIMUM_BYTE_PER_SLICE) {
				vcd_status = VCD_S_SUCCESS;
				break;
			}
		default:
			break;
		}
		if (sizeof(struct vcd_property_multi_slice) == hdr->sz &&
				!vcd_status) {
			enc->multi_slice = *multislice;
		}
		break;
	}
	case VCD_I_RATE_CONTROL:
	{
		struct vcd_property_rate_control *ratecontrol_type = value;
		if (sizeof(*ratecontrol_type) == hdr->sz &&
				ratecontrol_type->rate_control >=
				VCD_RATE_CONTROL_OFF &&
				ratecontrol_type->rate_control <=
				VCD_RATE_CONTROL_CBR_CFR) {
			enc->rc_type = *ratecontrol_type;
			ddl_set_default_enc_rc_params(enc);
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_SHORT_HEADER:
		if (sizeof(struct vcd_property_short_header) ==	hdr->sz &&
				enc->codec_type.codec == VCD_CODEC_MPEG4) {
			enc->short_header =
				*(struct vcd_property_short_header *)value;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_VOP_TIMING:
	{
		struct vcd_property_vop_timing *voptime = value;
		if (sizeof(*voptime) == hdr->sz && enc->frame_rate.fps_numerator
				<= voptime->vop_time_resolution) {
			enc->vop_timing = *voptime;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_HEADER_EXTENSION:
		if (sizeof(u32) == hdr->sz && enc->codec_type.codec ==
				VCD_CODEC_MPEG4) {
			enc->hdr_ext_control = *(u32 *)value;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_ENTROPY_CTRL:
	{
		struct vcd_property_entropy_control *entropy = value;
		if (sizeof(*entropy) == hdr->sz &&
				enc->codec_type.codec == VCD_CODEC_H264 &&
				entropy->entropy_sel >= VCD_ENTROPY_SEL_CAVLC &&
				entropy->entropy_sel <= VCD_ENTROPY_SEL_CABAC) {
			enc->entropy_control = *entropy;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_DEBLOCKING:
	{
		struct vcd_property_db_config *db = value;
		if (sizeof(*db) == hdr->sz &&
				enc->codec_type.codec == VCD_CODEC_H264	&&
				db->db_config >= VCD_DB_ALL_BLOCKING_BOUNDARY &&
				db->db_config <= VCD_DB_SKIP_SLICE_BOUNDARY) {
			enc->db_control = *db;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_QP_RANGE:
	{
		struct vcd_property_qp_range *qp = value;
		if (sizeof(*qp) == hdr->sz && qp->min_qp <= qp->max_qp &&
				((enc->codec_type.codec == VCD_CODEC_H264 &&
				qp->max_qp <= DDL_MAX_H264_QP) ||
				qp->max_qp <= DDL_MAX_MPEG4_QP)) {
			enc->qp_range = *qp;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_SESSION_QP:
	{
		struct vcd_property_session_qp *qp = value;
		if ((sizeof(*qp) == hdr->sz) &&
				qp->iframe_qp >= enc->qp_range.min_qp &&
				qp->iframe_qp <= enc->qp_range.max_qp &&
				qp->frame_qp >= enc->qp_range.min_qp &&
				qp->frame_qp <= enc->qp_range.max_qp) {
			enc->session_qp = *qp;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_RC_LEVEL_CONFIG:
	{
		struct vcd_property_rc_level *rc_level = value;
		if (sizeof(*rc_level) == hdr->sz &&
				(enc->rc_type.rate_control >=
				VCD_RATE_CONTROL_VBR_VFR ||
				enc->rc_type.rate_control <=
				VCD_RATE_CONTROL_CBR_VFR) &&
				(!rc_level->mb_level_rc ||
				enc->codec_type.codec == VCD_CODEC_H264)) {
			enc->rc_level = *rc_level;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_FRAME_LEVEL_RC:
	{
		struct vcd_property_frame_level_rc_params *rc = value;
		if (sizeof(*rc) == hdr->sz && rc->reaction_coeff &&
				enc->rc_level.frame_level_rc) {
			enc->frame_level_rc = *rc;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_ADAPTIVE_RC:
	{
		struct vcd_property_adaptive_rc_params *rc = value;
		if (sizeof(*rc) == hdr->sz && enc->codec_type.codec ==
				VCD_CODEC_H264 && enc->rc_level.mb_level_rc) {
			enc->adaptive_rc = *rc;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_INTRA_REFRESH:
	{
		struct vcd_property_intra_refresh_mb_number *mbnum = value;
		u32 frame_mbnum = (enc->frame_size.width / 16) *
			(enc->frame_size.height / 16);
		if (sizeof(*mbnum) == hdr->sz && mbnum->cir_mb_number <=
				frame_mbnum) {
			enc->intra_refresh = *mbnum;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_BUFFER_FORMAT:
	{
		struct vcd_property_buffer_format *tile = value;
		if (sizeof(*tile) == hdr->sz && tile->buffer_format ==
				VCD_BUFFER_FORMAT_NV12) {
			enc->buf_format = *tile;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case DDL_I_INPUT_BUF_REQ:
	{
		struct vcd_buffer_requirement *buf_req = value;
		if (sizeof(*buf_req) == hdr->sz && ddl_valid_buffer_requirement(
				&enc->input_buf_req, buf_req)) {
			enc->client_input_buf_req = *buf_req;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case DDL_I_OUTPUT_BUF_REQ:
	{
		struct vcd_buffer_requirement *buf_req = value;
		if (sizeof(*buf_req) == hdr->sz && ddl_valid_buffer_requirement(
				&enc->output_buf_req, buf_req)) {
			enc->client_output_buf_req = *buf_req;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		vcd_status = ddl_set_metadata_params(ddl, hdr, value);
		break;
	default:
		vcd_status = VCD_ERR_ILLEGAL_OP;
		break;
	}
	return vcd_status;
}

static u32 ddl_get_dec_property(struct ddl_client_context *ddl,
		struct vcd_property_hdr *hdr, void *value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_decoder_data *dec = &ddl->codec_data.decoder;

	switch (hdr->id) {
	case VCD_I_FRAME_SIZE:
		if (sizeof(struct vcd_property_frame_size) == hdr->sz) {
			if (dec->client_frame_size.width) {
				struct vcd_property_frame_size *size = value;
				*size = dec->client_frame_size;
				vcd_status = VCD_S_SUCCESS;
			} else {
				vcd_status = VCD_ERR_ILLEGAL_OP;
			}
		}
		break;
	case VCD_I_PROFILE:
		if (sizeof(struct vcd_property_profile) == hdr->sz) {
			*(struct vcd_property_profile *)value = dec->profile;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_LEVEL:
		if (sizeof(struct vcd_property_level) == hdr->sz) {
			*(struct vcd_property_level *)value = dec->level;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_PROGRESSIVE_ONLY:
		if (sizeof(u32) == hdr->sz) {
			*(u32 *)value = dec->progressive_only;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case DDL_I_INPUT_BUF_REQ:
		if (sizeof(struct vcd_buffer_requirement) == hdr->sz) {
			if (dec->client_input_buf_req.size) {
				*(struct vcd_buffer_requirement *)value =
					dec->client_input_buf_req;
				vcd_status = VCD_S_SUCCESS;
			} else {
				vcd_status = VCD_ERR_ILLEGAL_OP;
			}
		}
		break;
	case DDL_I_OUTPUT_BUF_REQ:
		if (sizeof(struct vcd_buffer_requirement) == hdr->sz) {
			if (dec->client_output_buf_req.size) {
				*(struct vcd_buffer_requirement *)value =
					dec->client_output_buf_req;
				vcd_status = VCD_S_SUCCESS;
			} else {
				vcd_status = VCD_ERR_ILLEGAL_OP;
			}
		}
		break;
	case VCD_I_CODEC:
		if (sizeof(struct vcd_property_codec) == hdr->sz) {
			if (dec->codec_type.codec) {
				*(struct vcd_property_codec *)value =
					dec->codec_type;
				vcd_status = VCD_S_SUCCESS;
			} else {
				vcd_status = VCD_ERR_ILLEGAL_OP;
			}
		}
		break;
	case VCD_I_BUFFER_FORMAT:
		if (sizeof(struct vcd_property_buffer_format) == hdr->sz) {
			*(struct vcd_property_buffer_format *)value =
				dec->buf_format;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_POST_FILTER:
		if (sizeof(struct vcd_property_post_filter) == hdr->sz) {
			*(struct vcd_property_post_filter *)value =
				dec->post_filter;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case DDL_I_SEQHDR_ALIGN_BYTES:
		if (sizeof(u32) == hdr->sz) {
			*(u32 *)value = DDL_LINEAR_BUFFER_ALIGN_BYTES;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case DDL_I_FRAME_PROC_UNITS:
		if (sizeof(u32) == hdr->sz && dec->client_frame_size.width &&
				dec->client_frame_size.height) {
			*(u32 *)value = ((dec->client_frame_size.width >> 4) *
				(dec->client_frame_size.height >> 4));
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case DDL_I_DPB_RETRIEVE:
		if (sizeof(struct ddl_frame_data_tag) == hdr->sz) {
			vcd_status = ddl_decoder_dpb_transact(dec,
				(struct ddl_frame_data_tag *)value,
				DDL_DPB_OP_RETRIEVE);
		}
		break;
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		vcd_status = ddl_get_metadata_params(ddl, hdr, value);
		break;
	default:
		vcd_status = VCD_ERR_ILLEGAL_OP;
		break;
	}
	return vcd_status;
}

static u32 ddl_get_enc_property(struct ddl_client_context *ddl,
		struct vcd_property_hdr *hdr, void *value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_encoder_data *enc = &ddl->codec_data.encoder;

	struct vcd_property_entropy_control *entropy_control;
	struct vcd_property_intra_refresh_mb_number *intra_refresh;

	switch (hdr->id) {
	case VCD_I_CODEC:
		if (sizeof(struct vcd_property_codec) == hdr->sz) {
			*(struct vcd_property_codec *)value = enc->codec_type;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_FRAME_SIZE:
		if (sizeof(struct vcd_property_frame_size) == hdr->sz) {
			*(struct vcd_property_frame_size *)value =
				enc->frame_size;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_FRAME_RATE:
		if (sizeof(struct vcd_property_frame_rate) == hdr->sz) {
			*(struct vcd_property_frame_rate *)value =
				enc->frame_rate;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_TARGET_BITRATE:
		if (sizeof(struct vcd_property_target_bitrate) == hdr->sz) {
			*(struct vcd_property_target_bitrate *)value =
				enc->target_bit_rate;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_RATE_CONTROL:
		if (sizeof(struct vcd_property_rate_control) == hdr->sz) {
			*(struct vcd_property_rate_control *)value =
				enc->rc_type;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_PROFILE:
		if (sizeof(struct vcd_property_profile) == hdr->sz) {
			*(struct vcd_property_profile *)value = enc->profile;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_LEVEL:
		if (sizeof(struct vcd_property_level) == hdr->sz) {
			*(struct vcd_property_level *)value = enc->level;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_MULTI_SLICE:
		if (sizeof(struct vcd_property_multi_slice) == hdr->sz) {
			*(struct vcd_property_multi_slice *)value =
				enc->multi_slice;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_SEQ_HEADER:
	{
		struct vcd_sequence_hdr *seq_hdr = value;
		if (enc->seq_header.size && sizeof(struct vcd_sequence_hdr) ==
				hdr->sz && enc->seq_header.size <=
				seq_hdr->sz) {
			memcpy(seq_hdr->addr, enc->seq_header.virt_addr,
				enc->seq_header.size);
			seq_hdr->sz = enc->seq_header.size;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case DDL_I_SEQHDR_PRESENT:
		if (sizeof(u32) == hdr->sz) {
			if ((enc->codec_type.codec == VCD_CODEC_MPEG4 &&
					!enc->short_header.short_header) ||
					enc->codec_type.codec ==
					VCD_CODEC_H264)
				*(u32 *)value = 0x1;
			else
				*(u32 *)value = 0x0;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_VOP_TIMING:
		if (sizeof(struct vcd_property_vop_timing) == hdr->sz) {
			*(struct vcd_property_vop_timing *)value =
				enc->vop_timing;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_SHORT_HEADER:
		if (sizeof(struct vcd_property_short_header) == hdr->sz) {
			if (enc->codec_type.codec == VCD_CODEC_MPEG4) {
				*(struct vcd_property_short_header *)value =
					enc->short_header;
				vcd_status = VCD_S_SUCCESS;
			} else {
				vcd_status = VCD_ERR_ILLEGAL_OP;
			}
		}
		break;
	case VCD_I_ENTROPY_CTRL:
		entropy_control = value;
		if (sizeof(struct vcd_property_entropy_control) == hdr->sz) {
			if (enc->codec_type.codec == VCD_CODEC_H264) {
				*entropy_control = enc->entropy_control;
				vcd_status = VCD_S_SUCCESS;
			} else {
				vcd_status = VCD_ERR_ILLEGAL_OP;
			}
		}
		break;
	case VCD_I_DEBLOCKING:
		if (sizeof(struct vcd_property_db_config) == hdr->sz) {
			if (enc->codec_type.codec == VCD_CODEC_H264) {
				*(struct vcd_property_db_config *)value =
					enc->db_control;
				vcd_status = VCD_S_SUCCESS;
			} else {
				vcd_status = VCD_ERR_ILLEGAL_OP;
			}
		}
		break;
	case VCD_I_INTRA_PERIOD:
		if (sizeof(struct vcd_property_i_period) == hdr->sz) {
			*(struct vcd_property_i_period *)value = enc->period;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_QP_RANGE:
		if (sizeof(struct vcd_property_qp_range) == hdr->sz) {
			*(struct vcd_property_qp_range *)value = enc->qp_range;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_SESSION_QP:
		if (sizeof(struct vcd_property_session_qp) == hdr->sz) {
			*(struct vcd_property_session_qp *)value =
				enc->session_qp;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_RC_LEVEL_CONFIG:
		if (sizeof(struct vcd_property_rc_level) == hdr->sz) {
			*(struct vcd_property_rc_level *)value = enc->rc_level;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_FRAME_LEVEL_RC:
		if (sizeof(struct vcd_property_frame_level_rc_params) ==
				hdr->sz) {
			*(struct vcd_property_frame_level_rc_params *)value =
				enc->frame_level_rc;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_ADAPTIVE_RC:
		if (sizeof(struct vcd_property_adaptive_rc_params) ==
				hdr->sz) {
			*(struct vcd_property_adaptive_rc_params *)value =
				enc->adaptive_rc;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_INTRA_REFRESH:
		intra_refresh = value;
		if (sizeof(struct vcd_property_intra_refresh_mb_number) ==
				hdr->sz) {
			*intra_refresh = enc->intra_refresh;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case DDL_I_INPUT_BUF_REQ:
		if (sizeof(struct vcd_buffer_requirement) == hdr->sz) {
			if (enc->output_buf_req.size) {
				*(struct vcd_buffer_requirement *)value =
					enc->client_input_buf_req;
				vcd_status = VCD_S_SUCCESS;
			} else {
				vcd_status = VCD_ERR_ILLEGAL_OP;
			}
		}
		break;
	case DDL_I_OUTPUT_BUF_REQ:
		if (sizeof(struct vcd_buffer_requirement) == hdr->sz) {
			if (enc->output_buf_req.size) {
				*(struct vcd_buffer_requirement *)value =
					enc->client_output_buf_req;
				vcd_status = VCD_S_SUCCESS;
			} else {
				vcd_status = VCD_ERR_ILLEGAL_OP;
			}
		}
		break;
	case VCD_I_BUFFER_FORMAT:
		if (sizeof(struct vcd_property_buffer_format) == hdr->sz) {
			*(struct vcd_property_buffer_format *)value =
				enc->buf_format;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case DDL_I_FRAME_PROC_UNITS:
		if (sizeof(u32) == hdr->sz && enc->frame_size.width &&
				enc->frame_size.height) {
			*(u32 *)value = ((enc->frame_size.width >> 4) *
				(enc->frame_size.height >> 4));
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_HEADER_EXTENSION:
		if (sizeof(u32) == hdr->sz && enc->codec_type.codec ==
				VCD_CODEC_MPEG4) {
			*(u32 *)value = enc->hdr_ext_control;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		vcd_status = ddl_get_metadata_params(ddl, hdr, value);
		break;
	default:
		vcd_status = VCD_ERR_ILLEGAL_OP;
		break;
	}
	return vcd_status;
}

static u32 ddl_set_enc_dynamic_property(struct ddl_encoder_data *enc,
		struct vcd_property_hdr *hdr, void *value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	switch (hdr->id) {
	case VCD_I_REQ_IFRAME:
		if (sizeof(struct vcd_property_req_i_frame) == hdr->sz) {
			enc->dynamic_prop_change |= DDL_ENC_REQ_IFRAME;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_TARGET_BITRATE:
		if (sizeof(struct vcd_property_target_bitrate) == hdr->sz) {
			enc->target_bit_rate =
				*(struct vcd_property_target_bitrate *)value;
			enc->dynamic_prop_change |= DDL_ENC_CHANGE_BITRATE;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_INTRA_PERIOD:
	{
		struct vcd_property_i_period *iperiod = value;
		if (sizeof(struct vcd_property_i_period) == hdr->sz &&
				!iperiod->bframes) {
			enc->period = *iperiod;
			enc->dynamic_prop_change |= DDL_ENC_CHANGE_IPERIOD;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_FRAME_RATE:
	{
		struct vcd_property_frame_rate *frame_rate = value;
		if (sizeof(struct vcd_property_frame_rate) == hdr->sz &&
				frame_rate->fps_denominator &&
				frame_rate->fps_numerator &&
				frame_rate->fps_denominator <=
				frame_rate->fps_numerator) {
			enc->frame_rate = *frame_rate;
			enc->dynamic_prop_change |= DDL_ENC_CHANGE_FRAMERATE;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	default:
		vcd_status = VCD_ERR_ILLEGAL_OP;
		break;
	}
	return vcd_status;
}

void ddl_set_default_dec_property(struct ddl_client_context *ddl)
{
	struct ddl_decoder_data *dec = &(ddl->codec_data.decoder);

	if (dec->codec_type.codec == VCD_CODEC_MPEG4 ||
			dec->codec_type.codec == VCD_CODEC_MPEG2) {
		dec->post_filter.post_filter = true;
	} else {
		dec->post_filter.post_filter = false;
	}
	dec->buf_format.buffer_format = VCD_BUFFER_FORMAT_NV12;
	dec->client_frame_size.height = 144;
	dec->client_frame_size.width = 176;
	dec->client_frame_size.stride = 176;
	dec->client_frame_size.scan_lines = 144;
	dec->progressive_only = 1;
	ddl_set_default_metadata_flag(ddl);

	ddl_set_default_decoder_buffer_req(dec, true);
}

static void ddl_set_default_enc_property(struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *enc = &(ddl->codec_data.encoder);

	ddl_set_default_enc_profile(enc);
	ddl_set_default_enc_level(enc);

	enc->rc_type.rate_control = VCD_RATE_CONTROL_VBR_VFR;
	ddl_set_default_enc_rc_params(enc);

	ddl_set_default_enc_intra_period(enc);

	enc->intra_refresh.cir_mb_number = 0;
	ddl_set_default_enc_vop_timing(enc);

	enc->multi_slice.m_slice_size = VCD_MSLICE_OFF;
	enc->short_header.short_header = false;

	enc->entropy_control.entropy_sel = VCD_ENTROPY_SEL_CAVLC;
	enc->entropy_control.cabac_model = VCD_CABAC_MODEL_NUMBER_0;
	enc->db_control.db_config = VCD_DB_ALL_BLOCKING_BOUNDARY;
	enc->db_control.slice_alpha_offset = 0;
	enc->db_control.slice_beta_offset = 0;

	enc->re_con_buf_format.buffer_format = VCD_BUFFER_FORMAT_TILE_4x2;

	enc->buf_format.buffer_format = VCD_BUFFER_FORMAT_NV12;

	enc->hdr_ext_control = 0;

	ddl_set_default_metadata_flag(ddl);

	ddl_set_default_encoder_buffer_req(enc);
}

static void ddl_set_default_enc_profile(struct ddl_encoder_data *enc)
{
	enum vcd_codec codec = enc->codec_type.codec;
	if (codec == VCD_CODEC_MPEG4)
		enc->profile.profile = VCD_PROFILE_MPEG4_SP;
	else if (codec == VCD_CODEC_H264)
		enc->profile.profile = VCD_PROFILE_H264_BASELINE;
	else
		enc->profile.profile = VCD_PROFILE_H263_BASELINE;
}

static void ddl_set_default_enc_level(struct ddl_encoder_data *enc)
{
	enum vcd_codec codec = enc->codec_type.codec;
	if (codec == VCD_CODEC_MPEG4)
		enc->level.level = VCD_LEVEL_MPEG4_1;
	else if (codec == VCD_CODEC_H264)
		enc->level.level = VCD_LEVEL_H264_1;
	else
		enc->level.level = VCD_LEVEL_H263_10;
}

static void ddl_set_default_enc_vop_timing(struct ddl_encoder_data *enc)
{
	enc->vop_timing.vop_time_resolution = (2 *
		enc->frame_rate.fps_numerator) /
		enc->frame_rate.fps_denominator;
}

static void ddl_set_default_enc_intra_period(struct ddl_encoder_data *enc)
{
	switch (enc->rc_type.rate_control) {
	default:
	case VCD_RATE_CONTROL_VBR_VFR:
	case VCD_RATE_CONTROL_VBR_CFR:
	case VCD_RATE_CONTROL_CBR_VFR:
	case VCD_RATE_CONTROL_OFF:
		enc->period.frames = ((enc->frame_rate.fps_numerator << 1) /
			enc->frame_rate.fps_denominator) - 1;
		break;
	case VCD_RATE_CONTROL_CBR_CFR:
		enc->period.frames = ((enc->frame_rate.fps_numerator >> 1) /
			enc->frame_rate.fps_denominator) - 1;
		break;
	}
	enc->period.bframes = 0;
}

static void ddl_set_default_enc_rc_params(struct ddl_encoder_data *enc)
{
	enum vcd_codec codec = enc->codec_type.codec;

	enc->rc_level.frame_level_rc = true;
	enc->qp_range.min_qp = 0x1;

	if (codec == VCD_CODEC_H264) {
		enc->qp_range.max_qp = 0x33;
		enc->session_qp.iframe_qp = 0x19;
		enc->session_qp.frame_qp = 0x19;

		enc->rc_level.mb_level_rc = true;
		enc->adaptive_rc.activity_region_flag = true;
		enc->adaptive_rc.dark_region_as_flag = true;
		enc->adaptive_rc.smooth_region_as_flag = true;
		enc->adaptive_rc.static_region_as_flag = true;
	} else {
		enc->qp_range.max_qp = 0x1f;
		enc->session_qp.iframe_qp = 0x14;
		enc->session_qp.frame_qp = 0x14;
		enc->rc_level.mb_level_rc = false;
	}

	switch (enc->rc_type.rate_control) {
	default:
	case VCD_RATE_CONTROL_VBR_VFR:
		enc->r_cframe_skip = 1;
		enc->frame_level_rc.reaction_coeff = 0x1f4;
		break;
	case VCD_RATE_CONTROL_VBR_CFR:
		enc->r_cframe_skip = 0;
		enc->frame_level_rc.reaction_coeff = 0x1f4;
		break;
	case VCD_RATE_CONTROL_CBR_VFR:
		enc->r_cframe_skip = 1;
		if (codec != VCD_CODEC_H264) {
			enc->session_qp.iframe_qp = 0xf;
			enc->session_qp.frame_qp = 0xf;
		}

		enc->frame_level_rc.reaction_coeff = 0x6;
		break;
	case VCD_RATE_CONTROL_CBR_CFR:
		enc->r_cframe_skip = 0;
		enc->frame_level_rc.reaction_coeff = 0x6;
		break;
	case VCD_RATE_CONTROL_OFF:
		enc->r_cframe_skip = 0;
		enc->rc_level.frame_level_rc = false;
		enc->rc_level.mb_level_rc = false;
		break;
	}
}

void ddl_set_default_encoder_buffer_req(struct ddl_encoder_data *enc)
{
	u32 y_cb_cr_size;

	y_cb_cr_size = ddl_get_yuv_buffer_size(&enc->frame_size,
		&enc->buf_format, false);

	memset(&enc->input_buf_req, 0, sizeof(struct vcd_buffer_requirement));

	enc->input_buf_req.min_count = 1;
	enc->input_buf_req.actual_count = enc->input_buf_req.min_count;
	enc->input_buf_req.max_count = DDL_MAX_BUFFER_COUNT;
	enc->input_buf_req.size = y_cb_cr_size;
	enc->input_buf_req.align = DDL_LINEAR_BUFFER_ALIGN_BYTES;

	enc->client_input_buf_req = enc->input_buf_req;

	memset(&enc->output_buf_req, 0, sizeof(struct vcd_buffer_requirement));

	enc->output_buf_req.min_count = 2;
	enc->output_buf_req.actual_count = enc->output_buf_req.min_count;
	enc->output_buf_req.max_count = DDL_MAX_BUFFER_COUNT;
	enc->output_buf_req.align = DDL_LINEAR_BUFFER_ALIGN_BYTES;
	enc->output_buf_req.size = y_cb_cr_size;
	ddl_set_default_encoder_metadata_buffer_size(enc);
	enc->client_output_buf_req = enc->output_buf_req;
}

void ddl_set_default_decoder_buffer_req(struct ddl_decoder_data *dec,
	u32 estimate)
{
	size_t y_cb_cr_size;
	u32 min_dpb;
	struct vcd_property_frame_size *frame_size;
	struct vcd_buffer_requirement *output_buf_req, *input_buf_req;

	if (!dec->codec_type.codec)
		return;

	if (estimate) {
		frame_size = &dec->client_frame_size;
		output_buf_req = &dec->client_output_buf_req;
		input_buf_req = &dec->client_input_buf_req;
		min_dpb = ddl_decoder_min_num_dpb(dec);
		y_cb_cr_size = ddl_get_yuv_buffer_size(frame_size,
			&dec->buf_format, !dec->progressive_only);
	} else {
		frame_size = &dec->frame_size;
		output_buf_req = &dec->actual_output_buf_req;
		input_buf_req = &dec->actual_input_buf_req;
		y_cb_cr_size = dec->y_cb_cr_size;
		min_dpb = dec->min_dpb_num;
	}

	memset(output_buf_req, 0, sizeof(struct vcd_buffer_requirement));

	output_buf_req->min_count = min_dpb;
	output_buf_req->actual_count = output_buf_req->min_count;
	output_buf_req->max_count = DDL_MAX_BUFFER_COUNT;
	output_buf_req->size = y_cb_cr_size;
	if (dec->buf_format.buffer_format != VCD_BUFFER_FORMAT_NV12)
		output_buf_req->align = DDL_TILE_BUFFER_ALIGN_BYTES;
	else
		output_buf_req->align = DDL_LINEAR_BUFFER_ALIGN_BYTES;

	ddl_set_default_decoder_metadata_buffer_size(dec, frame_size,
		output_buf_req);

	dec->min_output_buf_req = *output_buf_req;

	memset(input_buf_req, 0, sizeof(struct vcd_buffer_requirement));

	input_buf_req->min_count = 1;
	input_buf_req->actual_count = input_buf_req->min_count;
	input_buf_req->max_count = DDL_MAX_BUFFER_COUNT;
	input_buf_req->size = y_cb_cr_size;

	if (input_buf_req->size >= (1280 * 720 * 3) >> 1)
		input_buf_req->size >>= 1;

	input_buf_req->align = DDL_LINEAR_BUFFER_ALIGN_BYTES;

	dec->min_input_buf_req = *input_buf_req;
}

size_t ddl_get_yuv_buffer_size(struct vcd_property_frame_size *frame_size,
	struct vcd_property_buffer_format *buf_format, u32 interlace)
{
	u32 width = frame_size->stride;
	u32 height = frame_size->scan_lines;
	size_t sz;

	if (buf_format->buffer_format != VCD_BUFFER_FORMAT_NV12) {
		size_t component_sz;
		u32 width_round_up;
		u32 height_round_up;
		u32 height_chroma = (height >> 1);

		width_round_up = DDL_TILE_ALIGN(width, DDL_TILE_ALIGN_WIDTH);
		height_round_up = DDL_TILE_ALIGN(height, DDL_TILE_ALIGN_HEIGHT);

		component_sz = width_round_up * height_round_up;
		component_sz = DDL_TILE_ALIGN(component_sz,
			DDL_TILE_MULTIPLY_FACTOR);

		sz = (component_sz + DDL_TILE_BUF_ALIGN_GUARD_BYTES) &
			DDL_TILE_BUF_ALIGN_MASK;

		height_round_up = DDL_TILE_ALIGN(height_chroma,
			DDL_TILE_ALIGN_HEIGHT);
		component_sz = width_round_up * height_round_up;
		component_sz = DDL_TILE_ALIGN(component_sz,
			DDL_TILE_MULTIPLY_FACTOR);
		sz += component_sz;
	} else {
		sz = height * width;
		sz += sz >> 1;
	}
	return sz;
}

void ddl_calculate_stride(struct vcd_property_frame_size *frame_size,
	u32 interlace)
{
	frame_size->stride = ((frame_size->width + 15) >> 4) << 4;

	if (interlace)
		frame_size->scan_lines = ((frame_size->height + 31) >> 5) << 5;
	else
		frame_size->scan_lines = ((frame_size->height + 15) >> 4) << 4;
}

static u32 ddl_valid_buffer_requirement(struct vcd_buffer_requirement
	*orig, struct vcd_buffer_requirement *req)
{
	u32 status = false;
	if (orig->max_count >= req->actual_count &&
			orig->actual_count <= req->actual_count &&
			orig->align <= req->align && orig->size <= req->size) {
		status = true;
	} else {
		pr_err("ddl_valid_buf_req:Failed\n");
	}
	return status;
}

static u32 ddl_decoder_min_num_dpb(struct ddl_decoder_data *dec)
{
	u32 min_dpb = 0;
	switch (dec->codec_type.codec) {
	default:
	case VCD_CODEC_MPEG4:
	case VCD_CODEC_MPEG2:
	case VCD_CODEC_DIVX_4:
	case VCD_CODEC_DIVX_5:
	case VCD_CODEC_DIVX_6:
	case VCD_CODEC_XVID:
		min_dpb = 3;
		break;
	case VCD_CODEC_H263:
		min_dpb = 2;
		break;
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		min_dpb = 4;
		break;
	case VCD_CODEC_H264:
	{
		u32 yuv_size = (dec->client_frame_size.height *
			dec->client_frame_size.width * 3) >> 1;
		min_dpb = 6912000 / yuv_size;
		if (min_dpb > 16)
			min_dpb = 16;

		min_dpb += 2;
		break;
	}
	}
	return min_dpb;
}

static u32 ddl_set_dec_buffers(struct ddl_decoder_data *dec,
	struct ddl_property_dec_pic_buffers *dpb)
{
	u32 vcd_status = VCD_S_SUCCESS;
	u32 i;
	for (i = 0; !vcd_status && i < dpb->no_of_dec_pic_buf; ++i) {
		if (!IS_ALIGNED(dpb->dec_pic_buffers[i].vcd_frm.phys_addr,
				dec->client_output_buf_req.align) ||
				dpb->dec_pic_buffers[i].vcd_frm.alloc_len <
				dec->client_output_buf_req.size) {
			vcd_status = VCD_ERR_ILLEGAL_PARM;
			pr_err("ddl_set_prop:"
				"Dpb_align_fail_or_alloc_size_small\n");
			return vcd_status;
		}
	}

	if (dec->dp_buf.no_of_dec_pic_buf) {
		kfree(dec->dp_buf.dec_pic_buffers);
		dec->dp_buf.dec_pic_buffers = NULL;
		dec->dp_buf.no_of_dec_pic_buf = 0;
	}
	dec->dp_buf.dec_pic_buffers = kmalloc(dpb->no_of_dec_pic_buf *
		sizeof(struct ddl_frame_data_tag), GFP_KERNEL);

	if (!dec->dp_buf.dec_pic_buffers) {
		pr_err("ddl_dec_set_prop:"
			"Dpb_container_alloc_failed\n");
		return VCD_ERR_ALLOC_FAIL;
	}
	dec->dp_buf.no_of_dec_pic_buf = dpb->no_of_dec_pic_buf;
	for (i = 0; i < dpb->no_of_dec_pic_buf; ++i)
		dec->dp_buf.dec_pic_buffers[i] = dpb->dec_pic_buffers[i];

	dec->dpb_mask.client_mask = 0;
	dec->dpb_mask.hw_mask = 0;
	dec->dynamic_prop_change = 0;
	return VCD_S_SUCCESS;
}

void ddl_set_initial_default_values(struct ddl_client_context *ddl)
{
	if (ddl->decoding) {
		ddl->codec_data.decoder.codec_type.codec = VCD_CODEC_MPEG4;
		ddl_set_default_dec_property(ddl);
	} else {
		struct ddl_encoder_data *enc = &(ddl->codec_data.encoder);
		enc->codec_type.codec = VCD_CODEC_MPEG4;

		enc->target_bit_rate.target_bitrate = 64000;
		enc->frame_size.width = 176;
		enc->frame_size.height = 144;
		enc->frame_size.stride = 176;
		enc->frame_size.scan_lines = 144;
		enc->frame_rate.fps_numerator = 30;
		enc->frame_rate.fps_denominator = 1;
		ddl_set_default_enc_property(ddl);
	}
}
