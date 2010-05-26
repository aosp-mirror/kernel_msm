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

static u32 ddl_set_dec_property(struct ddl_client_context_type *pddl,
				struct vcd_property_hdr_type *p_property_hdr,
				void *p_property_value);
static u32 ddl_set_enc_property(struct ddl_client_context_type *pddl,
				struct vcd_property_hdr_type *p_property_hdr,
				void *p_property_value);
static u32 ddl_get_dec_property(struct ddl_client_context_type *pddl,
				struct vcd_property_hdr_type *p_property_hdr,
				void *p_property_value);
static u32 ddl_get_enc_property(struct ddl_client_context_type *pddl,
				struct vcd_property_hdr_type *p_property_hdr,
				void *p_property_value);
static u32 ddl_set_enc_dynamic_property(struct ddl_encoder_data_type *p_encoder,
					struct vcd_property_hdr_type
					*p_property_hdr,
					void *p_property_value);
static void ddl_set_default_enc_property(struct ddl_client_context_type *p_ddl);
static void ddl_set_default_enc_profile(struct ddl_encoder_data_type
					*p_encoder);
static void ddl_set_default_enc_level(struct ddl_encoder_data_type *p_encoder);
static void ddl_set_default_enc_vop_timing(struct ddl_encoder_data_type
					   *p_encoder);
static void ddl_set_default_enc_intra_period(struct ddl_encoder_data_type
					     *p_encoder);
static void ddl_set_default_enc_rc_params(struct ddl_encoder_data_type
					  *p_encoder);
static u32 ddl_valid_buffer_requirement(struct vcd_buffer_requirement_type
					*original_buf_req,
					struct vcd_buffer_requirement_type
					*req_buf_req);
static u32 ddl_decoder_min_num_dpb(struct ddl_decoder_data_type *p_decoder);
static u32 ddl_set_dec_buffers
    (struct ddl_decoder_data_type *p_decoder,
     struct ddl_property_dec_pic_buffers_type *p_dpb);

u32 ddl_set_property(u32 *ddl_handle,
     struct vcd_property_hdr_type *p_property_hdr, void *p_property_value)
{
	u32 vcd_status;
	struct ddl_context_type *p_ddl_context;
	struct ddl_client_context_type *p_ddl =
	    (struct ddl_client_context_type *)ddl_handle;

	if (!p_property_hdr || !p_property_value) {
		VIDC_LOGERR_STRING("ddl_set_prop:Bad_argument");
		return VCD_ERR_ILLEGAL_PARM;
	}
	p_ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_set_prop:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (!p_ddl) {
		VIDC_LOGERR_STRING("ddl_set_prop:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (p_ddl->b_decoding) {
		vcd_status =
		    ddl_set_dec_property(p_ddl, p_property_hdr,
					 p_property_value);
	} else {
		vcd_status =
		    ddl_set_enc_property(p_ddl, p_property_hdr,
					 p_property_value);
	}
	if (vcd_status)
		VIDC_LOGERR_STRING("ddl_set_prop:FAILED");

	return vcd_status;
}

u32 ddl_get_property(u32 *ddl_handle,
     struct vcd_property_hdr_type *p_property_hdr, void *p_property_value)
{

	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_context_type *p_ddl_context;
	struct ddl_client_context_type *p_ddl =
	    (struct ddl_client_context_type *)ddl_handle;

	if (!p_property_hdr || !p_property_value)
		return VCD_ERR_ILLEGAL_PARM;

	if (p_property_hdr->prop_id == DDL_I_CAPABILITY) {
		if (sizeof(struct ddl_property_capability_type) ==
		    p_property_hdr->n_size) {
			struct ddl_property_capability_type *p_ddl_capability =
			    (struct ddl_property_capability_type *)
			    p_property_value;
			p_ddl_capability->n_max_num_client = VCD_MAX_NO_CLIENT;
			p_ddl_capability->b_exclusive =
				VCD_COMMAND_EXCLUSIVE;
			p_ddl_capability->n_frame_command_depth =
				VCD_FRAME_COMMAND_DEPTH;
			p_ddl_capability->n_general_command_depth =
				VCD_GENERAL_COMMAND_DEPTH;
			p_ddl_capability->n_ddl_time_out_in_ms =
				DDL_HW_TIMEOUT_IN_MS;
			vcd_status = VCD_S_SUCCESS;
		}
		return vcd_status;
	}
	p_ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(p_ddl_context))
		return VCD_ERR_ILLEGAL_OP;

	if (!p_ddl)
		return VCD_ERR_BAD_HANDLE;

	if (p_ddl->b_decoding) {
		vcd_status =
		    ddl_get_dec_property(p_ddl, p_property_hdr,
					 p_property_value);
	} else {
		vcd_status =
		    ddl_get_enc_property(p_ddl, p_property_hdr,
					 p_property_value);
	}
	if (vcd_status)
		VIDC_LOGERR_STRING("ddl_get_prop:FAILED");

	return vcd_status;
}

u32 ddl_decoder_ready_to_start(struct ddl_client_context_type *p_ddl,
     struct vcd_sequence_hdr_type *p_header)
{
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	if (!p_decoder->codec_type.e_codec) {
		VIDC_LOGERR_STRING("ddl_dec_start_check:Codec_not_set");
		return FALSE;
	}
	if ((!p_header) &&
	    (!p_decoder->client_frame_size.n_height ||
	     !p_decoder->client_frame_size.n_width)
	    ) {
		VIDC_LOGERR_STRING
		    ("ddl_dec_start_check:Client_height_width_default");
		return FALSE;
	}
	return TRUE;
}

u32 ddl_encoder_ready_to_start(struct ddl_client_context_type *p_ddl)
{
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);

	if (!p_encoder->codec_type.e_codec ||
	    !p_encoder->frame_size.n_height ||
	    !p_encoder->frame_size.n_width ||
	    !p_encoder->frame_rate.n_fps_denominator ||
	    !p_encoder->frame_rate.n_fps_numerator ||
	    !p_encoder->target_bit_rate.n_target_bitrate) {
		return FALSE;
	}
	return TRUE;
}

static u32 ddl_set_dec_property
    (struct ddl_client_context_type *p_ddl,
     struct vcd_property_hdr_type *p_property_hdr, void *p_property_value) {
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	switch (p_property_hdr->prop_id) {
	case DDL_I_DPB_RELEASE:
		{
			if (sizeof(struct ddl_frame_data_type_tag) ==
			    p_property_hdr->n_size
			    && p_decoder->dp_buf.n_no_of_dec_pic_buf) {
				vcd_status =
				    ddl_decoder_dpb_transact(p_decoder,
					     (struct ddl_frame_data_type_tag *)
					     p_property_value,
					     DDL_DPB_OP_MARK_FREE);
			}
			break;
		}
	case DDL_I_DPB:
		{
			struct ddl_property_dec_pic_buffers_type *p_dpb =
			    (struct ddl_property_dec_pic_buffers_type *)
			    p_property_value;

			if (sizeof(struct ddl_property_dec_pic_buffers_type) ==
			    p_property_hdr->n_size &&
			    (DDLCLIENT_STATE_IS
			     (p_ddl, DDL_CLIENT_WAIT_FOR_INITCODEC)
			     || DDLCLIENT_STATE_IS(p_ddl,
						   DDL_CLIENT_WAIT_FOR_DPB)
			    ) &&
			    p_dpb->n_no_of_dec_pic_buf >=
			    p_decoder->client_output_buf_req.n_actual_count) {
				vcd_status =
				    ddl_set_dec_buffers(p_decoder, p_dpb);
			}
			break;
		}
	case DDL_I_REQ_OUTPUT_FLUSH:
		{
			if (sizeof(u32) == p_property_hdr->n_size) {
				p_decoder->n_dynamic_prop_change |=
				    DDL_DEC_REQ_OUTPUT_FLUSH;
				p_decoder->dpb_mask.n_client_mask = 0;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_INPUT_BUF_REQ:
		{
			struct vcd_buffer_requirement_type *p_buffer_req =
			    (struct vcd_buffer_requirement_type *)
			    p_property_value;
			if (sizeof(struct vcd_buffer_requirement_type) ==
			    p_property_hdr->n_size &&
			    (ddl_valid_buffer_requirement(
						&p_decoder->min_input_buf_req,
						p_buffer_req))) {
				p_decoder->client_input_buf_req = *p_buffer_req;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_OUTPUT_BUF_REQ:
		{
			struct vcd_buffer_requirement_type *p_buffer_req =
			    (struct vcd_buffer_requirement_type *)
			    p_property_value;
			if (sizeof(struct vcd_buffer_requirement_type) ==
			    p_property_hdr->n_size &&
			    (ddl_valid_buffer_requirement(
						&p_decoder->min_output_buf_req,
						p_buffer_req))) {
				p_decoder->client_output_buf_req =
				    *p_buffer_req;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}

	case VCD_I_CODEC:
		{
			struct vcd_property_codec_type *p_codec =
			    (struct vcd_property_codec_type *)p_property_value;
			if (sizeof(struct vcd_property_codec_type) ==
			    p_property_hdr->n_size
			    && DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_OPEN)
			    ) {
				u32 b_return;
				vcd_fw_transact(FALSE, TRUE,
					p_decoder->codec_type.e_codec);
				b_return = vcd_fw_transact(TRUE, TRUE,
					p_codec->e_codec);
				if (b_return) {
					p_decoder->codec_type = *p_codec;
					ddl_set_default_dec_property(p_ddl);
					vcd_status = VCD_S_SUCCESS;
				} else {
					b_return = vcd_fw_transact(TRUE, TRUE,
						p_decoder->codec_type.e_codec);
					vcd_status = VCD_ERR_NOT_SUPPORTED;
				}
			}
			break;
		}
	case VCD_I_POST_FILTER:
		{
			if (sizeof(struct vcd_property_post_filter_type) ==
			    p_property_hdr->n_size
			    && DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_OPEN) &&
			    (p_decoder->codec_type.e_codec == VCD_CODEC_MPEG4 ||
			     p_decoder->codec_type.e_codec == VCD_CODEC_MPEG2)
			    ) {
				p_decoder->post_filter =
				    *(struct vcd_property_post_filter_type *)
				    p_property_value;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_SIZE:
		{
			struct vcd_property_frame_size_type *p_frame_size =
			    (struct vcd_property_frame_size_type *)
			    p_property_value;

			if ((sizeof(struct vcd_property_frame_size_type) ==
					p_property_hdr->n_size) &&
				(DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_OPEN))) {
				if (p_decoder->client_frame_size.n_height !=
				    p_frame_size->n_height
				    || p_decoder->client_frame_size.n_width !=
				    p_frame_size->n_width) {
					p_decoder->client_frame_size =
					    *p_frame_size;
					ddl_calculate_stride(
					&p_decoder->client_frame_size,
					!p_decoder->n_progressive_only);
					ddl_set_default_decoder_buffer_req
					    (p_decoder, TRUE);
				}
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_BUFFER_FORMAT:
		{
			struct vcd_property_buffer_format_type *p_tile =
			    (struct vcd_property_buffer_format_type *)
			    p_property_value;
			if (sizeof(struct vcd_property_buffer_format_type) ==
			    p_property_hdr->n_size &&
			    DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_OPEN) &&
			    (p_tile->e_buffer_format == VCD_BUFFER_FORMAT_NV12
			     || p_tile->e_buffer_format ==
			     VCD_BUFFER_FORMAT_TILE_4x2)
			    ) {
				if (p_tile->e_buffer_format !=
				    p_decoder->buf_format.e_buffer_format) {
					p_decoder->buf_format = *p_tile;
					ddl_set_default_decoder_buffer_req
					    (p_decoder, TRUE);
				}
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		{
			vcd_status = ddl_set_metadata_params(p_ddl,
							     p_property_hdr,
							     p_property_value);
			break;
		}
	default:
		{
			vcd_status = VCD_ERR_ILLEGAL_OP;
			break;
		}
	}
	return vcd_status;
}

static u32 ddl_set_enc_property(struct ddl_client_context_type *p_ddl,
	struct vcd_property_hdr_type *p_property_hdr, void *p_property_value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);

	if (DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME)) {
		vcd_status = ddl_set_enc_dynamic_property(p_encoder,
			p_property_hdr, p_property_value);
		return vcd_status;
	}

	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_OPEN)) {
		VIDC_LOGERR_STRING
			("ddl_set_enc_property:Fails_as_not_in_open_state");
		return VCD_ERR_ILLEGAL_OP;
	}

	switch (p_property_hdr->prop_id) {
	case VCD_I_TARGET_BITRATE:
		{
			struct vcd_property_target_bitrate_type *p_bitrate =
				(struct vcd_property_target_bitrate_type *)
				p_property_value;
			if (sizeof(struct vcd_property_target_bitrate_type) ==
				p_property_hdr->n_size &&
				p_bitrate->n_target_bitrate) {
				p_encoder->target_bit_rate = *p_bitrate;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}

	case VCD_I_FRAME_RATE:
		{
			struct vcd_property_frame_rate_type *p_framerate =
				(struct vcd_property_frame_rate_type *)
				p_property_value;
			if (sizeof(struct vcd_property_frame_rate_type)
				== p_property_hdr->n_size &&
				p_framerate->n_fps_denominator &&
				p_framerate->n_fps_numerator) {
				p_encoder->frame_rate = *p_framerate;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_SIZE:
		{
			struct vcd_property_frame_size_type *p_framesize =
				(struct vcd_property_frame_size_type *)
				p_property_value;

			if ((sizeof(struct vcd_property_frame_size_type)
				== p_property_hdr->n_size) &&
				(DDL_ALLOW_ENC_FRAMESIZE(p_framesize->n_width,
				p_framesize->n_height))
				) {
				p_encoder->frame_size = *p_framesize;
				ddl_calculate_stride(&p_encoder->frame_size,
					FALSE);
				ddl_set_default_encoder_buffer_req(p_encoder);
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_CODEC:
		{
			struct vcd_property_codec_type *p_codec =
				(struct vcd_property_codec_type *)
				p_property_value;
			if (sizeof(struct vcd_property_codec_type) ==
				p_property_hdr->n_size) {
				u32 b_return;

				vcd_fw_transact(FALSE, FALSE,
					p_encoder->codec_type.e_codec);

				b_return = vcd_fw_transact(TRUE, FALSE,
					p_codec->e_codec);
				if (b_return) {
					p_encoder->codec_type = *p_codec;
					ddl_set_default_enc_property(p_ddl);
					vcd_status = VCD_S_SUCCESS;
				} else {
					b_return = vcd_fw_transact(TRUE, FALSE,
						p_encoder->codec_type.e_codec);
					vcd_status = VCD_ERR_NOT_SUPPORTED;
				}
			}
			break;
		}
	case VCD_I_REQ_IFRAME:
		{
			vcd_status = VCD_S_SUCCESS;
			break;
		}
	case VCD_I_INTRA_PERIOD:
		{
			struct vcd_property_i_period_type *p_iperiod =
				(struct vcd_property_i_period_type *)
				p_property_value;
			if ((sizeof(struct vcd_property_i_period_type) ==
				p_property_hdr->n_size) &&
				(!p_iperiod->n_b_frames)) {
				p_encoder->i_period = *p_iperiod;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_PROFILE:
		{
			struct vcd_property_profile_type *p_profile =
				(struct vcd_property_profile_type *)
				p_property_value;
			if (
				(sizeof(struct vcd_property_profile_type) ==
				p_property_hdr->n_size) &&
				(
				 (
				  (p_encoder->codec_type.
					e_codec == VCD_CODEC_MPEG4) &&
				  (
				   p_profile->e_profile == VCD_PROFILE_MPEG4_SP
				   || p_profile->e_profile ==
				   VCD_PROFILE_MPEG4_ASP
				   )
				  ) ||
				 (
				  (
				  (p_encoder->codec_type.
				   e_codec == VCD_CODEC_H264) &&
				   (p_profile->e_profile >=
					VCD_PROFILE_H264_BASELINE)
				   && (p_profile->e_profile <=
					VCD_PROFILE_H264_HIGH)
				   )
				  ) ||
				 (
				  (p_encoder->codec_type.
				   e_codec == VCD_CODEC_H263) &&
				  (p_profile->e_profile ==
				   VCD_PROFILE_H263_BASELINE)
				  )
				 )
				) {
				p_encoder->profile = *p_profile;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_LEVEL:
		{
			struct vcd_property_level_type *p_level =
				(struct vcd_property_level_type *)
				p_property_value;
			if (
				(sizeof(struct vcd_property_level_type) ==
				 p_property_hdr->n_size
				) &&
				(
				(
				(p_encoder->codec_type.
				 e_codec == VCD_CODEC_MPEG4) &&
				(p_level->e_level >= VCD_LEVEL_MPEG4_0) &&
				(p_level->e_level <= VCD_LEVEL_MPEG4_6)
				) ||
				(
				(p_encoder->codec_type.
				 e_codec == VCD_CODEC_H264) &&
				(p_level->e_level >= VCD_LEVEL_H264_1) &&
				(p_level->e_level <= VCD_LEVEL_H264_3p1)
				) ||
				(
				(p_encoder->codec_type.
				 e_codec == VCD_CODEC_H263) &&
				(p_level->e_level >= VCD_LEVEL_H263_10) &&
				(p_level->e_level <= VCD_LEVEL_H263_70)
				)
				)
				) {
				p_encoder->level = *p_level;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_MULTI_SLICE:
		{
			struct vcd_property_multi_slice_type *p_multislice =
				(struct vcd_property_multi_slice_type *)
				p_property_value;
			switch (p_multislice->e_m_slice_sel) {
			case VCD_MSLICE_OFF:
				{
					vcd_status = VCD_S_SUCCESS;
					break;
				}
			case VCD_MSLICE_BY_GOB:
				{
					if (p_encoder->codec_type.e_codec ==
						VCD_CODEC_H263)
						vcd_status = VCD_S_SUCCESS;
					 break;
				}
			case VCD_MSLICE_BY_MB_COUNT:
				{
					if (p_multislice->n_m_slice_size
						>= 1 && (p_multislice->
						n_m_slice_size <=
						(p_encoder->frame_size.n_height
						* p_encoder->frame_size.n_width
						/ 16 / 16))
						) {
						vcd_status = VCD_S_SUCCESS;
					}
					break;
				  }
			case VCD_MSLICE_BY_BYTE_COUNT:
				{
					if (p_multislice->n_m_slice_size <
						DDL_MINIMUM_BYTE_PER_SLICE) {
						vcd_status = VCD_S_SUCCESS;
						break;
					}
				}
			default:
				{
					break;
				}
			}
			if (sizeof(struct vcd_property_multi_slice_type) ==
				p_property_hdr->n_size &&
				!vcd_status) {
				p_encoder->multi_slice = *p_multislice;
			}
			break;
		}
	case VCD_I_RATE_CONTROL:
		{
			struct vcd_property_rate_control_type
				*p_ratecontrol_type =
				(struct vcd_property_rate_control_type *)
				p_property_value;
			if (sizeof(struct vcd_property_rate_control_type) ==
				p_property_hdr->n_size &&
				p_ratecontrol_type->
				e_rate_control >= VCD_RATE_CONTROL_OFF &&
				p_ratecontrol_type->
				e_rate_control <= VCD_RATE_CONTROL_CBR_CFR
				) {
				p_encoder->rc_type = *p_ratecontrol_type;
				ddl_set_default_enc_rc_params(p_encoder);
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_SHORT_HEADER:
		{

		if (sizeof(struct vcd_property_short_header_type) ==
			p_property_hdr->n_size &&
			p_encoder->codec_type.e_codec == VCD_CODEC_MPEG4) {
			p_encoder->short_header =
				*(struct vcd_property_short_header_type *)
				p_property_value;
			vcd_status = VCD_S_SUCCESS;
		}

			break;
		}
	case VCD_I_VOP_TIMING:
		{
			struct vcd_property_vop_timing_type *p_voptime =
				(struct vcd_property_vop_timing_type *)
				p_property_value;
			if (
				(sizeof(struct vcd_property_vop_timing_type) ==
					  p_property_hdr->n_size
				) &&
				(p_encoder->frame_rate.n_fps_numerator <=
					p_voptime->n_vop_time_resolution)
				) {
				p_encoder->vop_timing = *p_voptime;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_HEADER_EXTENSION:
		{
			if (sizeof(u32) == p_property_hdr->n_size &&
				p_encoder->codec_type.e_codec == VCD_CODEC_MPEG4
				) {
				p_encoder->n_hdr_ext_control = *(u32 *)
					p_property_value;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_ENTROPY_CTRL:
		{
			struct vcd_property_entropy_control_type
				*p_entropy_control =
				(struct vcd_property_entropy_control_type *)
				p_property_value;
			if (sizeof(struct vcd_property_entropy_control_type) ==
				p_property_hdr->n_size &&
				p_encoder->codec_type.e_codec == VCD_CODEC_H264
				&& p_entropy_control->
				e_entropy_sel >= VCD_ENTROPY_SEL_CAVLC &&
				p_entropy_control->e_entropy_sel <=
				VCD_ENTROPY_SEL_CABAC) {
				p_encoder->entropy_control = *p_entropy_control;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_DEBLOCKING:
		{
			struct vcd_property_db_config_type *p_dbconfig =
				(struct vcd_property_db_config_type *)
				p_property_value;
			if (sizeof(struct vcd_property_db_config_type) ==
				p_property_hdr->n_size &&
				p_encoder->codec_type.e_codec == VCD_CODEC_H264
				&& p_dbconfig->e_db_config >=
				VCD_DB_ALL_BLOCKING_BOUNDARY
				&& p_dbconfig->e_db_config <=
				VCD_DB_SKIP_SLICE_BOUNDARY
				) {
				p_encoder->db_control = *p_dbconfig;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_QP_RANGE:
		{
			struct vcd_property_qp_range_type *p_qp =
				(struct vcd_property_qp_range_type *)
				p_property_value;
			if ((sizeof(struct vcd_property_qp_range_type) ==
				p_property_hdr->n_size) &&
				(p_qp->n_min_qp <= p_qp->n_max_qp) &&
				(
				(p_encoder->codec_type.e_codec == VCD_CODEC_H264
				&& p_qp->n_max_qp <= DDL_MAX_H264_QP) ||
				(p_qp->n_max_qp <= DDL_MAX_MPEG4_QP)
				)
				) {
				p_encoder->qp_range = *p_qp;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_SESSION_QP:
		{
			struct vcd_property_session_qp_type *p_qp =
				(struct vcd_property_session_qp_type *)
				p_property_value;

		if ((sizeof(struct vcd_property_session_qp_type) ==
			p_property_hdr->n_size) &&
			(p_qp->n_i_frame_qp >= p_encoder->qp_range.n_min_qp) &&
			(p_qp->n_i_frame_qp <= p_encoder->qp_range.n_max_qp) &&
			(p_qp->n_p_frame_qp >= p_encoder->qp_range.n_min_qp) &&
			(p_qp->n_p_frame_qp <= p_encoder->qp_range.n_max_qp)
			) {
			p_encoder->session_qp = *p_qp;
			vcd_status = VCD_S_SUCCESS;
		}

			break;
		}
	case VCD_I_RC_LEVEL_CONFIG:
		{
			struct vcd_property_rc_level_type *p_rc_level =
				(struct vcd_property_rc_level_type *)
				p_property_value;
			if (sizeof(struct vcd_property_rc_level_type) ==
				p_property_hdr->n_size &&
				(
				p_encoder->rc_type.
				e_rate_control >= VCD_RATE_CONTROL_VBR_VFR ||
				p_encoder->rc_type.
				e_rate_control <= VCD_RATE_CONTROL_CBR_VFR
				) &&
				(!p_rc_level->b_mb_level_rc ||
				p_encoder->codec_type.e_codec == VCD_CODEC_H264
				)
				) {
				p_encoder->rc_level = *p_rc_level;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_LEVEL_RC:
		{

		struct vcd_property_frame_level_rc_params_type
			*p_frame_levelrc =
			(struct vcd_property_frame_level_rc_params_type *)
			p_property_value;

			if ((sizeof(struct
				vcd_property_frame_level_rc_params_type)
				== p_property_hdr->n_size) &&
				(p_frame_levelrc->n_reaction_coeff) &&
				(p_encoder->rc_level.b_frame_level_rc)
				) {
				p_encoder->frame_level_rc = *p_frame_levelrc;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_ADAPTIVE_RC:
		{

		if ((sizeof(struct
			vcd_property_adaptive_rc_params_type)
			== p_property_hdr->n_size) &&
			(p_encoder->codec_type.
			e_codec == VCD_CODEC_H264) &&
			(p_encoder->rc_level.b_mb_level_rc)) {

			p_encoder->adaptive_rc =
				*(struct vcd_property_adaptive_rc_params_type *)
				p_property_value;

			vcd_status = VCD_S_SUCCESS;
		}

			break;
		}
	case VCD_I_INTRA_REFRESH:
		{

		struct vcd_property_intra_refresh_mb_number_type
			*p_intra_refresh_mbnum =
			(struct	vcd_property_intra_refresh_mb_number_type *)
			p_property_value;

			u32 n_frame_mbnum =
				(p_encoder->frame_size.n_width / 16) *
				(p_encoder->frame_size.n_height / 16);
			if (sizeof(struct
				vcd_property_intra_refresh_mb_number_type)
				== p_property_hdr->n_size &&
				p_intra_refresh_mbnum->n_cir_mb_number <=
				n_frame_mbnum) {
				p_encoder->intra_refresh =
					*p_intra_refresh_mbnum;
				vcd_status = VCD_S_SUCCESS;
			}

			break;
		}
	case VCD_I_BUFFER_FORMAT:
		{
			struct vcd_property_buffer_format_type *p_tile =
				(struct vcd_property_buffer_format_type *)
				p_property_value;
			if (sizeof(struct vcd_property_buffer_format_type) ==
				p_property_hdr->n_size &&
				p_tile->e_buffer_format ==
				VCD_BUFFER_FORMAT_NV12) {
				p_encoder->buf_format = *p_tile;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_INPUT_BUF_REQ:
		{
			struct vcd_buffer_requirement_type *p_buffer_req =
				(struct vcd_buffer_requirement_type *)
				p_property_value;
			if (sizeof(struct vcd_buffer_requirement_type) ==
				p_property_hdr->n_size &&
				(ddl_valid_buffer_requirement(
				&p_encoder->input_buf_req, p_buffer_req))
				) {
				p_encoder->client_input_buf_req = *p_buffer_req;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_OUTPUT_BUF_REQ:
		{
			struct vcd_buffer_requirement_type *p_buffer_req =
				(struct vcd_buffer_requirement_type *)
				p_property_value;
			if (sizeof(struct vcd_buffer_requirement_type) ==
				p_property_hdr->n_size &&
				(ddl_valid_buffer_requirement(
				&p_encoder->output_buf_req, p_buffer_req))
				) {
				p_encoder->client_output_buf_req =
					*p_buffer_req;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		{
			vcd_status = ddl_set_metadata_params(
				p_ddl, p_property_hdr, p_property_value);
			break;
		}
	default:
		{
			vcd_status = VCD_ERR_ILLEGAL_OP;
			break;
		}
	}
	return vcd_status;
}

static u32 ddl_get_dec_property
    (struct ddl_client_context_type *p_ddl,
     struct vcd_property_hdr_type *p_property_hdr, void *p_property_value) {
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_decoder_data_type *p_decoder = &p_ddl->codec_data.decoder;

	switch (p_property_hdr->prop_id) {
	case VCD_I_FRAME_SIZE:
		{
			if (sizeof(struct vcd_property_frame_size_type) ==
			    p_property_hdr->n_size) {
				if (p_decoder->client_frame_size.n_width) {
					*(struct vcd_property_frame_size_type *)
					    p_property_value =
					    p_decoder->client_frame_size;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case VCD_I_PROFILE:
		{
			if (sizeof(struct vcd_property_profile_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_profile_type *)
				    p_property_value = p_decoder->profile;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_LEVEL:
		{
			if (sizeof(struct vcd_property_level_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_level_type *)
				    p_property_value = p_decoder->level;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_PROGRESSIVE_ONLY:
		{
			if (sizeof(u32) == p_property_hdr->n_size) {
				*(u32 *) p_property_value =
				    p_decoder->n_progressive_only;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_INPUT_BUF_REQ:
		{
			if (sizeof(struct vcd_buffer_requirement_type) ==
			    p_property_hdr->n_size) {
				if (p_decoder->
						client_input_buf_req.n_size) {
					*(struct vcd_buffer_requirement_type *)
					    p_property_value =
					    p_decoder->client_input_buf_req;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case DDL_I_OUTPUT_BUF_REQ:
		{
			if (sizeof(struct vcd_buffer_requirement_type) ==
			    p_property_hdr->n_size) {
				if (p_decoder->client_output_buf_req.n_size) {
					*(struct vcd_buffer_requirement_type *)
					    p_property_value =
					    p_decoder->client_output_buf_req;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case VCD_I_CODEC:
		{
			if (sizeof(struct vcd_property_codec_type) ==
			    p_property_hdr->n_size) {
				if (p_decoder->codec_type.e_codec) {
					*(struct vcd_property_codec_type *)
					    p_property_value =
					    p_decoder->codec_type;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case VCD_I_BUFFER_FORMAT:
		{
			if (sizeof(struct vcd_property_buffer_format_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_buffer_format_type *)
				    p_property_value = p_decoder->buf_format;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_POST_FILTER:
		{
			if (sizeof(struct vcd_property_post_filter_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_post_filter_type *)
				    p_property_value = p_decoder->post_filter;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_SEQHDR_ALIGN_BYTES:
		{
			if (sizeof(u32) == p_property_hdr->n_size) {
				*(u32 *) p_property_value =
				    DDL_LINEAR_BUFFER_ALIGN_BYTES;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_FRAME_PROC_UNITS:
		{
			if (sizeof(u32) == p_property_hdr->n_size &&
			    p_decoder->client_frame_size.n_width &&
			    p_decoder->client_frame_size.n_height) {
				*(u32 *) p_property_value =
				    ((p_decoder->client_frame_size.
				      n_width >> 4) *
				     (p_decoder->client_frame_size.
				      n_height >> 4)
				    );
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_DPB_RETRIEVE:
		{
			if (sizeof(struct ddl_frame_data_type_tag) ==
			    p_property_hdr->n_size) {
				vcd_status =
				    ddl_decoder_dpb_transact(p_decoder,
					 (struct ddl_frame_data_type_tag *)
					     p_property_value,
					     DDL_DPB_OP_RETRIEVE);
			}
			break;
		}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		{
			vcd_status = ddl_get_metadata_params(
						   p_ddl,
						   p_property_hdr,
						   p_property_value);
			break;
		}
	default:
		{
			vcd_status = VCD_ERR_ILLEGAL_OP;
			break;
		}
	}
	return vcd_status;
}

static u32 ddl_get_enc_property
    (struct ddl_client_context_type *p_ddl,
     struct vcd_property_hdr_type *p_property_hdr, void *p_property_value) {
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_encoder_data_type *p_encoder = &p_ddl->codec_data.encoder;

	struct vcd_property_entropy_control_type *entropy_control;
	struct vcd_property_intra_refresh_mb_number_type *intra_refresh;

	switch (p_property_hdr->prop_id) {
	case VCD_I_CODEC:
		{
			if (sizeof(struct vcd_property_codec_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_codec_type *)
					p_property_value =
					p_encoder->codec_type;
		    vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_SIZE:
		{
			if (sizeof(struct vcd_property_frame_size_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_frame_size_type *)
					p_property_value =
					p_encoder->frame_size;

				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_RATE:
		{
			if (sizeof(struct vcd_property_frame_rate_type) ==
				p_property_hdr->n_size) {

				*(struct vcd_property_frame_rate_type *)
					p_property_value =
					p_encoder->frame_rate;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_TARGET_BITRATE:
		{

			if (sizeof(struct vcd_property_target_bitrate_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_target_bitrate_type *)
					p_property_value =
					p_encoder->target_bit_rate;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_RATE_CONTROL:
		{
			if (sizeof(struct vcd_property_rate_control_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_rate_control_type *)
				    p_property_value = p_encoder->rc_type;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_PROFILE:
		{
			if (sizeof(struct vcd_property_profile_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_profile_type *)
				    p_property_value = p_encoder->profile;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_LEVEL:
		{
			if (sizeof(struct vcd_property_level_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_level_type *)
				    p_property_value = p_encoder->level;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_MULTI_SLICE:
		{
			if (sizeof(struct vcd_property_multi_slice_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_multi_slice_type *)
				    p_property_value = p_encoder->multi_slice;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_SEQ_HEADER:
		{
			struct vcd_sequence_hdr_type *p_seq_hdr =
			    (struct vcd_sequence_hdr_type *)p_property_value;
			if (p_encoder->seq_header.n_buffer_size &&
			    sizeof(struct vcd_sequence_hdr_type) ==
			    p_property_hdr->n_size
			    && p_encoder->seq_header.n_buffer_size <=
			    p_seq_hdr->n_sequence_header_len) {
				DDL_MEMCPY(p_seq_hdr->p_sequence_header,
					   p_encoder->seq_header.
					   p_align_virtual_addr,
					   p_encoder->seq_header.n_buffer_size);
				p_seq_hdr->n_sequence_header_len =
				    p_encoder->seq_header.n_buffer_size;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_SEQHDR_PRESENT:
		{
			if (sizeof(u32) == p_property_hdr->n_size) {
				if ((p_encoder->codec_type.
					e_codec == VCD_CODEC_MPEG4 &&
					!p_encoder->short_header.b_short_header)
					|| p_encoder->codec_type.e_codec ==
					VCD_CODEC_H264) {
					*(u32 *)p_property_value = 0x1;
				} else {
					*(u32 *)p_property_value = 0x0;
				}
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_VOP_TIMING:
		{
			if (sizeof(struct vcd_property_vop_timing_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_vop_timing_type *)
				    p_property_value = p_encoder->vop_timing;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_SHORT_HEADER:
		{
			if (sizeof(struct vcd_property_short_header_type) ==
			    p_property_hdr->n_size) {
				if (p_encoder->codec_type.e_codec ==
					VCD_CODEC_MPEG4) {
					*(struct vcd_property_short_header_type
					  *)p_property_value =
						p_encoder->short_header;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case VCD_I_ENTROPY_CTRL:
		{
			entropy_control = p_property_value;
			if (sizeof(struct vcd_property_entropy_control_type) ==
			    p_property_hdr->n_size) {
				if (p_encoder->codec_type.e_codec ==
					VCD_CODEC_H264) {
					*entropy_control =
				     p_encoder->entropy_control;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case VCD_I_DEBLOCKING:
		{
			if (sizeof(struct vcd_property_db_config_type) ==
			    p_property_hdr->n_size) {
				if (p_encoder->codec_type.e_codec ==
					VCD_CODEC_H264) {
					*(struct vcd_property_db_config_type *)
					    p_property_value =
					    p_encoder->db_control;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case VCD_I_INTRA_PERIOD:
		{
			if (sizeof(struct vcd_property_i_period_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_i_period_type *)
				    p_property_value = p_encoder->i_period;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_QP_RANGE:
		{
			if (sizeof(struct vcd_property_qp_range_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_qp_range_type *)
				    p_property_value = p_encoder->qp_range;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_SESSION_QP:
		{
			if (sizeof(struct vcd_property_session_qp_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_session_qp_type *)
				    p_property_value = p_encoder->session_qp;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_RC_LEVEL_CONFIG:
		{
			if (sizeof(struct vcd_property_rc_level_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_rc_level_type *)
				    p_property_value = p_encoder->rc_level;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_LEVEL_RC:
		{
			if (sizeof
			    (struct vcd_property_frame_level_rc_params_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_frame_level_rc_params_type
				 *)p_property_value =
				 p_encoder->frame_level_rc;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_ADAPTIVE_RC:
		{
			if (sizeof(struct vcd_property_adaptive_rc_params_type)
			    == p_property_hdr->n_size) {
				*(struct vcd_property_adaptive_rc_params_type *)
				    p_property_value = p_encoder->adaptive_rc;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_INTRA_REFRESH:
		{
			intra_refresh = p_property_value;
			if (sizeof
			    (struct vcd_property_intra_refresh_mb_number_type)
			    == p_property_hdr->n_size) {
				*intra_refresh = p_encoder->intra_refresh;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_INPUT_BUF_REQ:
		{
			if (sizeof(struct vcd_buffer_requirement_type) ==
			    p_property_hdr->n_size) {
				if (p_encoder->output_buf_req.n_size) {
					*(struct vcd_buffer_requirement_type *)
					    p_property_value =
					    p_encoder->client_input_buf_req;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case DDL_I_OUTPUT_BUF_REQ:
		{
			if (sizeof(struct vcd_buffer_requirement_type) ==
			    p_property_hdr->n_size) {
				if (p_encoder->output_buf_req.n_size) {
					*(struct vcd_buffer_requirement_type *)
					    p_property_value =
					    p_encoder->client_output_buf_req;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case VCD_I_BUFFER_FORMAT:
		{
			if (sizeof(struct vcd_property_buffer_format_type) ==
			    p_property_hdr->n_size) {
				*(struct vcd_property_buffer_format_type *)
				    p_property_value = p_encoder->buf_format;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_FRAME_PROC_UNITS:
		{
			if (sizeof(u32) == p_property_hdr->n_size &&
			    p_encoder->frame_size.n_width &&
			    p_encoder->frame_size.n_height) {
				*(u32 *) p_property_value =
				    ((p_encoder->frame_size.n_width >> 4) *
				     (p_encoder->frame_size.n_height >> 4)
				    );
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_HEADER_EXTENSION:
		{
			if (sizeof(u32) == p_property_hdr->n_size &&
			    p_encoder->codec_type.e_codec == VCD_CODEC_MPEG4) {
				*(u32 *) p_property_value =
				    p_encoder->n_hdr_ext_control;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		{
			vcd_status = ddl_get_metadata_params(
						   p_ddl,
						   p_property_hdr,
						   p_property_value);
			break;
		}
	default:
		{
			vcd_status = VCD_ERR_ILLEGAL_OP;
			break;
		}
	}
	return vcd_status;
}

static u32 ddl_set_enc_dynamic_property
    (struct ddl_encoder_data_type *p_encoder,
     struct vcd_property_hdr_type *p_property_hdr, void *p_property_value) {
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	switch (p_property_hdr->prop_id) {
	case VCD_I_REQ_IFRAME:
		{
			if (sizeof(struct vcd_property_req_i_frame_type) ==
			    p_property_hdr->n_size) {
				p_encoder->n_dynamic_prop_change |=
				    DDL_ENC_REQ_IFRAME;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_TARGET_BITRATE:
		{
			if (sizeof(struct vcd_property_target_bitrate_type) ==
			    p_property_hdr->n_size) {
				p_encoder->target_bit_rate =
				    *(struct vcd_property_target_bitrate_type *)
				    p_property_value;
				p_encoder->n_dynamic_prop_change |=
				    DDL_ENC_CHANGE_BITRATE;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_INTRA_PERIOD:
		{
			struct vcd_property_i_period_type *p_iperiod =
				(struct vcd_property_i_period_type *)
				p_property_value;
			if (sizeof(struct vcd_property_i_period_type) ==
				p_property_hdr->n_size &&
				!p_iperiod->n_b_frames) {
				p_encoder->i_period = *p_iperiod;
				p_encoder->n_dynamic_prop_change |=
					DDL_ENC_CHANGE_IPERIOD;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_RATE:
		{
			struct vcd_property_frame_rate_type *p_frame_rate =
			    (struct vcd_property_frame_rate_type *)
			    p_property_value;
			if (sizeof(struct vcd_property_frame_rate_type)
			    == p_property_hdr->n_size &&
			    p_frame_rate->n_fps_denominator &&
			    p_frame_rate->n_fps_numerator &&
			    p_frame_rate->n_fps_denominator <=
			    p_frame_rate->n_fps_numerator) {
				p_encoder->frame_rate = *p_frame_rate;
				p_encoder->n_dynamic_prop_change |=
				    DDL_ENC_CHANGE_FRAMERATE;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	default:
		{
			vcd_status = VCD_ERR_ILLEGAL_OP;
			break;
		}
	}
	return vcd_status;
}

void ddl_set_default_dec_property(struct ddl_client_context_type *p_ddl)
{
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);

	if (p_decoder->codec_type.e_codec == VCD_CODEC_MPEG4 ||
	    p_decoder->codec_type.e_codec == VCD_CODEC_MPEG2) {
		p_decoder->post_filter.b_post_filter = TRUE;
	} else {
		p_decoder->post_filter.b_post_filter = FALSE;
	}
	p_decoder->buf_format.e_buffer_format = VCD_BUFFER_FORMAT_NV12;
	p_decoder->client_frame_size.n_height = 144;
	p_decoder->client_frame_size.n_width = 176;
	p_decoder->client_frame_size.n_stride = 176;
	p_decoder->client_frame_size.n_scan_lines = 144;
	p_decoder->n_progressive_only = 1;
	ddl_set_default_metadata_flag(p_ddl);

	ddl_set_default_decoder_buffer_req(p_decoder, TRUE);

}

static void ddl_set_default_enc_property(struct ddl_client_context_type *p_ddl)
{
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);

	ddl_set_default_enc_profile(p_encoder);
	ddl_set_default_enc_level(p_encoder);

	p_encoder->rc_type.e_rate_control = VCD_RATE_CONTROL_VBR_VFR;
	ddl_set_default_enc_rc_params(p_encoder);

	ddl_set_default_enc_intra_period(p_encoder);

	p_encoder->intra_refresh.n_cir_mb_number = 0;
	ddl_set_default_enc_vop_timing(p_encoder);

	p_encoder->multi_slice.n_m_slice_size = VCD_MSLICE_OFF;
	p_encoder->short_header.b_short_header = FALSE;

	p_encoder->entropy_control.e_entropy_sel = VCD_ENTROPY_SEL_CAVLC;
	p_encoder->entropy_control.e_cabac_model = VCD_CABAC_MODEL_NUMBER_0;
	p_encoder->db_control.e_db_config = VCD_DB_ALL_BLOCKING_BOUNDARY;
	p_encoder->db_control.n_slice_alpha_offset = 0;
	p_encoder->db_control.n_slice_beta_offset = 0;

	p_encoder->re_con_buf_format.e_buffer_format =
		VCD_BUFFER_FORMAT_TILE_4x2;

	p_encoder->buf_format.e_buffer_format = VCD_BUFFER_FORMAT_NV12;

	p_encoder->n_hdr_ext_control = 0;

	ddl_set_default_metadata_flag(p_ddl);

	ddl_set_default_encoder_buffer_req(p_encoder);
}

static void ddl_set_default_enc_profile(struct ddl_encoder_data_type *p_encoder)
{
	enum vcd_codec_type e_codec = p_encoder->codec_type.e_codec;
	if (e_codec == VCD_CODEC_MPEG4)
		p_encoder->profile.e_profile = VCD_PROFILE_MPEG4_SP;
	else if (e_codec == VCD_CODEC_H264)
		p_encoder->profile.e_profile = VCD_PROFILE_H264_BASELINE;
	else
		p_encoder->profile.e_profile = VCD_PROFILE_H263_BASELINE;
}

static void ddl_set_default_enc_level(struct ddl_encoder_data_type *p_encoder)
{
	enum vcd_codec_type e_codec = p_encoder->codec_type.e_codec;
	if (e_codec == VCD_CODEC_MPEG4)
		p_encoder->level.e_level = VCD_LEVEL_MPEG4_1;
	else if (e_codec == VCD_CODEC_H264)
		p_encoder->level.e_level = VCD_LEVEL_H264_1;
	else
		p_encoder->level.e_level = VCD_LEVEL_H263_10;
}

static void ddl_set_default_enc_vop_timing
    (struct ddl_encoder_data_type *p_encoder)
{
	p_encoder->vop_timing.n_vop_time_resolution =
	    (2 * p_encoder->frame_rate.n_fps_numerator) /
	    p_encoder->frame_rate.n_fps_denominator;
}

static void ddl_set_default_enc_intra_period(
		struct ddl_encoder_data_type *p_encoder)
{
	switch (p_encoder->rc_type.e_rate_control) {
	default:
	case VCD_RATE_CONTROL_VBR_VFR:
	case VCD_RATE_CONTROL_VBR_CFR:
	case VCD_RATE_CONTROL_CBR_VFR:
	case VCD_RATE_CONTROL_OFF:
		{
			p_encoder->i_period.n_p_frames =
			    ((p_encoder->frame_rate.n_fps_numerator << 1) /
			     p_encoder->frame_rate.n_fps_denominator) - 1;
			break;
		}
	case VCD_RATE_CONTROL_CBR_CFR:
		{
			p_encoder->i_period.n_p_frames =
			    ((p_encoder->frame_rate.n_fps_numerator >> 1) /
			     p_encoder->frame_rate.n_fps_denominator) - 1;
			break;
		}
	}
	p_encoder->i_period.n_b_frames = 0;
}

static void ddl_set_default_enc_rc_params(
		struct ddl_encoder_data_type *p_encoder)
{
	enum vcd_codec_type e_codec = p_encoder->codec_type.e_codec;

	p_encoder->rc_level.b_frame_level_rc = TRUE;
	p_encoder->qp_range.n_min_qp = 0x1;

	if (e_codec == VCD_CODEC_H264) {
		p_encoder->qp_range.n_max_qp = 0x33;
		p_encoder->session_qp.n_i_frame_qp = 0x19;
		p_encoder->session_qp.n_p_frame_qp = 0x19;

		p_encoder->rc_level.b_mb_level_rc = TRUE;
		p_encoder->adaptive_rc.b_activity_region_flag = TRUE;
		p_encoder->adaptive_rc.b_dark_region_as_flag = TRUE;
		p_encoder->adaptive_rc.b_smooth_region_as_flag = TRUE;
		p_encoder->adaptive_rc.b_static_region_as_flag = TRUE;
	} else {
		p_encoder->qp_range.n_max_qp = 0x1f;
		p_encoder->session_qp.n_i_frame_qp = 0x14;
		p_encoder->session_qp.n_p_frame_qp = 0x14;
		p_encoder->rc_level.b_mb_level_rc = FALSE;
	}

	switch (p_encoder->rc_type.e_rate_control) {
	default:
	case VCD_RATE_CONTROL_VBR_VFR:
		{
			p_encoder->n_r_cframe_skip = 1;
			p_encoder->frame_level_rc.n_reaction_coeff = 0x1f4;
			break;
		}
	case VCD_RATE_CONTROL_VBR_CFR:
		{
			p_encoder->n_r_cframe_skip = 0;
			p_encoder->frame_level_rc.n_reaction_coeff = 0x1f4;
			break;
		}
	case VCD_RATE_CONTROL_CBR_VFR:
		{
			p_encoder->n_r_cframe_skip = 1;
			if (e_codec != VCD_CODEC_H264) {
				p_encoder->session_qp.n_i_frame_qp = 0xf;
				p_encoder->session_qp.n_p_frame_qp = 0xf;
			}

			p_encoder->frame_level_rc.n_reaction_coeff = 0x6;
			break;
		}
	case VCD_RATE_CONTROL_CBR_CFR:
		{
			p_encoder->n_r_cframe_skip = 0;
			p_encoder->frame_level_rc.n_reaction_coeff = 0x6;
			break;
		}
	case VCD_RATE_CONTROL_OFF:
		{
			p_encoder->n_r_cframe_skip = 0;
			p_encoder->rc_level.b_frame_level_rc = FALSE;
			p_encoder->rc_level.b_mb_level_rc = FALSE;
			break;
		}
	}
}

void ddl_set_default_encoder_buffer_req(struct ddl_encoder_data_type *p_encoder)
{
	u32 n_y_cb_cr_size;

	n_y_cb_cr_size = ddl_get_yuv_buffer_size(&p_encoder->frame_size,
		&p_encoder->buf_format, FALSE);

	memset(&p_encoder->input_buf_req, 0,
	       sizeof(struct vcd_buffer_requirement_type));

	p_encoder->input_buf_req.n_min_count = 1;
	p_encoder->input_buf_req.n_actual_count =
	    p_encoder->input_buf_req.n_min_count;
	p_encoder->input_buf_req.n_max_count = DDL_MAX_BUFFER_COUNT;
	p_encoder->input_buf_req.n_size = n_y_cb_cr_size;
	p_encoder->input_buf_req.n_align = DDL_LINEAR_BUFFER_ALIGN_BYTES;

	p_encoder->client_input_buf_req = p_encoder->input_buf_req;

	memset(&p_encoder->output_buf_req, 0,
	       sizeof(struct vcd_buffer_requirement_type));

	p_encoder->output_buf_req.n_min_count = 2;
	p_encoder->output_buf_req.n_actual_count =
	    p_encoder->output_buf_req.n_min_count;
	p_encoder->output_buf_req.n_max_count = DDL_MAX_BUFFER_COUNT;
	p_encoder->output_buf_req.n_align = DDL_LINEAR_BUFFER_ALIGN_BYTES;
	p_encoder->output_buf_req.n_size = n_y_cb_cr_size;
	ddl_set_default_encoder_metadata_buffer_size(p_encoder);
	p_encoder->client_output_buf_req = p_encoder->output_buf_req;
}

void ddl_set_default_decoder_buffer_req(struct ddl_decoder_data_type *p_decoder,
		u32 b_estimate)
{
	u32 n_y_cb_cr_size, n_min_dpb;
	struct vcd_property_frame_size_type  *p_frame_size;
	struct vcd_buffer_requirement_type *p_output_buf_req, *p_input_buf_req;

	if (!p_decoder->codec_type.e_codec)
		return;

	if (b_estimate) {
		p_frame_size = &p_decoder->client_frame_size;
		p_output_buf_req = &p_decoder->client_output_buf_req;
		p_input_buf_req = &p_decoder->client_input_buf_req;
		n_min_dpb = ddl_decoder_min_num_dpb(p_decoder);
		 n_y_cb_cr_size = ddl_get_yuv_buffer_size(p_frame_size,
			&p_decoder->buf_format,
			(!p_decoder->n_progressive_only));
	} else {
		p_frame_size = &p_decoder->frame_size;
		p_output_buf_req = &p_decoder->actual_output_buf_req;
		p_input_buf_req = &p_decoder->actual_input_buf_req;
		n_y_cb_cr_size = p_decoder->n_y_cb_cr_size;
		n_min_dpb = p_decoder->n_min_dpb_num;
	}

	memset(p_output_buf_req, 0, sizeof(struct vcd_buffer_requirement_type));

	p_output_buf_req->n_min_count = n_min_dpb;
	p_output_buf_req->n_actual_count = p_output_buf_req->n_min_count;
	p_output_buf_req->n_max_count = DDL_MAX_BUFFER_COUNT;
	p_output_buf_req->n_size = n_y_cb_cr_size;
	if (p_decoder->buf_format.e_buffer_format != VCD_BUFFER_FORMAT_NV12)
		p_output_buf_req->n_align = DDL_TILE_BUFFER_ALIGN_BYTES;
	else
		p_output_buf_req->n_align = DDL_LINEAR_BUFFER_ALIGN_BYTES;

	ddl_set_default_decoder_metadata_buffer_size(p_decoder,
		p_frame_size, p_output_buf_req);

	p_decoder->min_output_buf_req = *p_output_buf_req;

	memset(p_input_buf_req, 0, sizeof(struct vcd_buffer_requirement_type));

	p_input_buf_req->n_min_count = 1;
	p_input_buf_req->n_actual_count = p_input_buf_req->n_min_count;
	p_input_buf_req->n_max_count = DDL_MAX_BUFFER_COUNT;
	p_input_buf_req->n_size = n_y_cb_cr_size;

	if (p_input_buf_req->n_size >= ((1280*720*3) >> 1))
		p_input_buf_req->n_size = (p_input_buf_req->n_size >> 1);

	p_input_buf_req->n_align = DDL_LINEAR_BUFFER_ALIGN_BYTES;

	p_decoder->min_input_buf_req = *p_input_buf_req;

}

u32 ddl_get_yuv_buffer_size(struct vcd_property_frame_size_type *p_frame_size,
     struct vcd_property_buffer_format_type *p_buf_format, u32 inter_lace)
{
	u32 n_width = p_frame_size->n_stride;
	u32 n_height = p_frame_size->n_scan_lines;
	u32 n_total_memory_size;

	if (p_buf_format->e_buffer_format != VCD_BUFFER_FORMAT_NV12) {
		u32 n_component_mem_size;
		u32 n_width_round_up;
		u32 n_height_round_up;
		u32 n_height_chroma = (n_height >> 1);

		n_width_round_up =
		    DDL_TILE_ALIGN(n_width, DDL_TILE_ALIGN_WIDTH);
		n_height_round_up =
		    DDL_TILE_ALIGN(n_height, DDL_TILE_ALIGN_HEIGHT);

		n_component_mem_size = n_width_round_up * n_height_round_up;
		n_component_mem_size = DDL_TILE_ALIGN(n_component_mem_size,
						      DDL_TILE_MULTIPLY_FACTOR);

		n_total_memory_size = ((n_component_mem_size +
					 DDL_TILE_BUF_ALIGN_GUARD_BYTES) &
					DDL_TILE_BUF_ALIGN_MASK);

		n_height_round_up =
		    DDL_TILE_ALIGN(n_height_chroma, DDL_TILE_ALIGN_HEIGHT);
		n_component_mem_size = n_width_round_up * n_height_round_up;
		n_component_mem_size = DDL_TILE_ALIGN(n_component_mem_size,
						      DDL_TILE_MULTIPLY_FACTOR);
		n_total_memory_size += n_component_mem_size;
	} else {
		n_total_memory_size = n_height * n_width;
		n_total_memory_size += (n_total_memory_size >> 1);
	}
	return n_total_memory_size;
}

void ddl_calculate_stride(struct vcd_property_frame_size_type *p_frame_size,
						 u32 b_interlace)
{
	p_frame_size->n_stride = ((p_frame_size->n_width + 15) >> 4) << 4;

	if (b_interlace) {
		p_frame_size->n_scan_lines =
			((p_frame_size->n_height + 31) >> 5) << 5;
	} else {
		p_frame_size->n_scan_lines =
			((p_frame_size->n_height + 15) >> 4) << 4;
	}

}

static u32 ddl_valid_buffer_requirement
	(struct vcd_buffer_requirement_type *original_buf_req,
	struct vcd_buffer_requirement_type *req_buf_req)
{
	u32 b_status = FALSE;
	if (
		   original_buf_req->n_max_count >= req_buf_req->n_actual_count
		   && original_buf_req->n_actual_count <=
		   req_buf_req->n_actual_count &&
		   original_buf_req->n_align <= req_buf_req->n_align &&
		   original_buf_req->n_size <= req_buf_req->n_size) {
		b_status = TRUE;
	} else {
		VIDC_LOGERR_STRING("ddl_valid_buf_req:Failed");
	}
	return b_status;
}

static u32 ddl_decoder_min_num_dpb(struct ddl_decoder_data_type *p_decoder)
{
	u32 n_min_dpb = 0;
	switch (p_decoder->codec_type.e_codec) {
	default:
	case VCD_CODEC_MPEG4:
	case VCD_CODEC_MPEG2:
	case VCD_CODEC_DIVX_4:
	case VCD_CODEC_DIVX_5:
	case VCD_CODEC_DIVX_6:
	case VCD_CODEC_XVID:
		{
			n_min_dpb = 3;
			break;
		}
	case VCD_CODEC_H263:
		{
			n_min_dpb = 2;
			break;
		}
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		{
			n_min_dpb = 4;
			break;
		}
	case VCD_CODEC_H264:
		{
			u32 n_yuv_size =
			    ((p_decoder->client_frame_size.n_height *
			      p_decoder->client_frame_size.n_width * 3) >> 1);
			n_min_dpb = 6912000 / n_yuv_size;
			if (n_min_dpb > 16)
				n_min_dpb = 16;

			n_min_dpb += 2;
			break;
		}
	}
	return n_min_dpb;
}

static u32 ddl_set_dec_buffers
    (struct ddl_decoder_data_type *p_decoder,
     struct ddl_property_dec_pic_buffers_type *p_dpb) {
	u32 vcd_status = VCD_S_SUCCESS;
	u32 n_loopc;
	for (n_loopc = 0; !vcd_status &&
	     n_loopc < p_dpb->n_no_of_dec_pic_buf; ++n_loopc) {
		if ((!DDL_ADDR_IS_ALIGNED
		     (p_dpb->a_dec_pic_buffers[n_loopc].vcd_frm.p_physical,
		      p_decoder->client_output_buf_req.n_align)
		    )
		    || (p_dpb->a_dec_pic_buffers[n_loopc].vcd_frm.n_alloc_len <
			p_decoder->client_output_buf_req.n_size)
		    ) {
			vcd_status = VCD_ERR_ILLEGAL_PARM;
		}
	}
	if (vcd_status) {
		VIDC_LOGERR_STRING
		    ("ddl_set_prop:Dpb_align_fail_or_alloc_size_small");
		return vcd_status;
	}
	if (p_decoder->dp_buf.n_no_of_dec_pic_buf) {
		DDL_FREE(p_decoder->dp_buf.a_dec_pic_buffers);
		p_decoder->dp_buf.n_no_of_dec_pic_buf = 0;
	}
	p_decoder->dp_buf.a_dec_pic_buffers =
	    DDL_MALLOC(p_dpb->n_no_of_dec_pic_buf *
		       sizeof(struct ddl_frame_data_type_tag));

	if (!p_decoder->dp_buf.a_dec_pic_buffers) {
		VIDC_LOGERR_STRING
		    ("ddl_dec_set_prop:Dpb_container_alloc_failed");
		return VCD_ERR_ALLOC_FAIL;
	}
	p_decoder->dp_buf.n_no_of_dec_pic_buf = p_dpb->n_no_of_dec_pic_buf;
	for (n_loopc = 0; n_loopc < p_dpb->n_no_of_dec_pic_buf; ++n_loopc) {
		p_decoder->dp_buf.a_dec_pic_buffers[n_loopc] =
		    p_dpb->a_dec_pic_buffers[n_loopc];
	}
	p_decoder->dpb_mask.n_client_mask = 0;
	p_decoder->dpb_mask.n_hw_mask = 0;
	p_decoder->n_dynamic_prop_change = 0;
	return VCD_S_SUCCESS;
}

void ddl_set_initial_default_values(struct ddl_client_context_type *p_ddl)
{
	if (p_ddl->b_decoding) {
		p_ddl->codec_data.decoder.codec_type.e_codec = VCD_CODEC_MPEG4;
		vcd_fw_transact(TRUE, TRUE,
			p_ddl->codec_data.decoder.codec_type.e_codec);
		ddl_set_default_dec_property(p_ddl);
	} else {
		struct ddl_encoder_data_type *p_encoder =
		    &(p_ddl->codec_data.encoder);
		p_encoder->codec_type.e_codec = VCD_CODEC_MPEG4;
		vcd_fw_transact(TRUE, FALSE,
			p_encoder->codec_type.e_codec);

		p_encoder->target_bit_rate.n_target_bitrate = 64000;
		p_encoder->frame_size.n_width = 176;
		p_encoder->frame_size.n_height = 144;
		p_encoder->frame_size.n_stride = 176;
		p_encoder->frame_size.n_scan_lines = 144;
		p_encoder->frame_rate.n_fps_numerator = 30;
		p_encoder->frame_rate.n_fps_denominator = 1;
		ddl_set_default_enc_property(p_ddl);
	}

	return;
}
