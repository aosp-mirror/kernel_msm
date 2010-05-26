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

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

void ddl_core_init(struct ddl_context_type *p_ddl_context)
{
	char *psz_version;
	struct vcd_fw_details_type fw_details;
	u32 fw_endianness;
	enum vidc_720p_endian_type e_dma_endian;
	u32 b_interrupt_off;
	enum vidc_720p_interrupt_level_selection_type e_interrupt_sel;
	u32 intr_mask = 0x0;

	vcd_get_fw_property(VCD_FW_BOOTCODE, &fw_details);
	vcd_get_fw_property(VCD_FW_ENDIAN, &fw_endianness);
	if (fw_endianness == VCD_FW_BIG_ENDIAN)
		e_dma_endian = VIDC_720P_BIG_ENDIAN;
	else
		e_dma_endian = VIDC_720P_LITTLE_ENDIAN;

	b_interrupt_off = FALSE;
	e_interrupt_sel = VIDC_720P_INTERRUPT_LEVEL_SEL;

	intr_mask |= VIDC_720P_INTR_BUFFER_FULL;
	intr_mask |= VIDC_720P_INTR_FW_DONE;
	intr_mask |= VIDC_720P_INTR_DMA_DONE;
	intr_mask |= VIDC_720P_INTR_FRAME_DONE;

	vidc_720p_do_sw_reset();

	vidc_720p_init(&psz_version,
			fw_details.n_fw_size,
			fw_details.p_fw_buffer_addr,
			e_dma_endian,
			b_interrupt_off, e_interrupt_sel, intr_mask);
	return;
}

void ddl_core_start_cpu(struct ddl_context_type *p_ddl_context)
{
	u32 fw_endianness;
	enum vidc_720p_endian_type e_dma_endian;
	u32 dbg_core_dump_buf_size = 0;

	vcd_get_fw_property(VCD_FW_ENDIAN, &fw_endianness);
	if (fw_endianness == VCD_FW_BIG_ENDIAN)
		e_dma_endian = VIDC_720P_LITTLE_ENDIAN;
	else
		e_dma_endian = VIDC_720P_BIG_ENDIAN;

	ddl_move_command_state(p_ddl_context, DDL_CMD_CPU_RESET);

	DBG("VSP_BUF_ADDR_SIZE %d",
		p_ddl_context->context_buf_addr.n_buffer_size);
	if (p_ddl_context->enable_dbg_core_dump) {
		dbg_core_dump_buf_size = p_ddl_context->dbg_core_dump.
			n_buffer_size;
	}

	vidc_720p_start_cpu(e_dma_endian,
		p_ddl_context->context_buf_addr.p_align_physical_addr,
		p_ddl_context->dbg_core_dump.p_align_physical_addr,
		dbg_core_dump_buf_size);

	VIDC_DEBUG_REGISTER_LOG;
}

void ddl_channel_set(struct ddl_client_context_type *p_ddl)
{
	enum vidc_720p_enc_dec_selection_type e_enc_dec_sel;
	enum vidc_720p_codec_type e_codec;
	enum vcd_codec_type *p_codec;
	u32 fw_property_id;
	struct vcd_fw_details_type fw_details;

	if (p_ddl->b_decoding) {
		e_enc_dec_sel = VIDC_720P_DECODER;
		fw_property_id = VCD_FW_DECODE;
		p_codec = &(p_ddl->codec_data.decoder.codec_type.e_codec);
	} else {
		e_enc_dec_sel = VIDC_720P_ENCODER;
		fw_property_id = VCD_FW_ENCODE;
		p_codec = &(p_ddl->codec_data.encoder.codec_type.e_codec);
	}
	switch (*p_codec) {
	default:
	case VCD_CODEC_MPEG4:
		{
			e_codec = VIDC_720P_MPEG4;

		if (p_ddl->b_decoding) {
			vidc_720p_decode_set_mpeg4_data_partitionbuffer
				(p_ddl->p_ddl_context->data_partition_tempbuf.
				 p_align_physical_addr);
		}

			break;
		}
	case VCD_CODEC_H264:
		{
			e_codec = VIDC_720P_H264;
			break;
		}
	case VCD_CODEC_DIVX_4:
	case VCD_CODEC_DIVX_5:
	case VCD_CODEC_DIVX_6:
		{
			e_codec = VIDC_720P_DIVX;
			break;
		}
	case VCD_CODEC_XVID:
		{
			e_codec = VIDC_720P_XVID;
			break;
		}
	case VCD_CODEC_H263:
		{
			e_codec = VIDC_720P_H263;
			break;
		}
	case VCD_CODEC_MPEG2:
		{
			e_codec = VIDC_720P_MPEG2;
			break;
		}
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		{
			e_codec = VIDC_720P_VC1;
			break;
		}
	}

	fw_details.e_codec = *p_codec;
	vcd_get_fw_property(fw_property_id, &fw_details);
	VIDC_DEBUG_REGISTER_LOG;

	ddl_move_command_state(p_ddl->p_ddl_context, DDL_CMD_CHANNEL_SET);
	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_CHDONE);

	vidc_720p_set_channel(p_ddl->n_channel_id,
			       e_enc_dec_sel,
			       e_codec,
			       fw_details.p_fw_buffer_addr,
			       fw_details.n_fw_size);
}

void ddl_decode_init_codec(struct ddl_client_context_type *p_ddl)
{
	u32 n_seq_h = 0, n_seq_e = 0, n_start_byte_num = 0;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vcd_sequence_hdr_type *p_seq_hdr = &p_decoder->decode_config;
	enum vidc_720p_memory_access_method_type mem_access_method;

	ddl_metadata_enable(p_ddl);

	vidc_720p_decode_set_error_control(TRUE);

	vidc_720p_decode_set_mpeg4Post_filter(p_decoder->post_filter.
					       b_post_filter);
	if (p_decoder->codec_type.e_codec == VCD_CODEC_VC1_RCV) {
		vidc_720p_set_frame_size(p_decoder->client_frame_size.n_width,
			p_decoder->client_frame_size.n_height);
	} else {
		vidc_720p_set_frame_size(0x0, 0x0);
	}

	switch (p_decoder->buf_format.e_buffer_format) {
	default:
	case VCD_BUFFER_FORMAT_NV12:
		{
			mem_access_method = VIDC_720P_TILE_LINEAR;
			break;
		}
	case VCD_BUFFER_FORMAT_TILE_4x2:
		{
			mem_access_method = VIDC_720P_TILE_64x32;
			break;
		}
	}
	VIDC_LOG_STRING("HEADER-PARSE-START");
	VIDC_DEBUG_REGISTER_LOG;
	n_seq_h = (u32) p_seq_hdr->p_sequence_header;
	n_start_byte_num = 8 - (n_seq_h & DDL_STREAMBUF_ALIGN_GUARD_BYTES);
	n_seq_e = n_seq_h + p_seq_hdr->n_sequence_header_len;
	n_seq_h &= ~(DDL_STREAMBUF_ALIGN_GUARD_BYTES);
	DDL_PADDING_HACK(n_seq_e);

	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODECDONE);
	ddl_move_command_state(p_ddl->p_ddl_context, DDL_CMD_HEADER_PARSE);

	vidc_720p_decode_bitstream_header(p_ddl->n_channel_id,
		   p_seq_hdr->n_sequence_header_len,
		   n_start_byte_num,
		   n_seq_h,
		   n_seq_e,
		   mem_access_method);
}

void ddl_decode_dynamic_property(struct ddl_client_context_type *p_ddl,
				 u32 b_enable)
{
	uint8_t *p_temp = NULL;
	u32 n_extra_datastart = 0;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vcd_frame_data_type *p_bit_stream =
	    &(p_ddl->input_frame.vcd_frm);

	if (!b_enable) {
		if (p_decoder->b_dynmic_prop_change_req) {
			p_decoder->b_dynmic_prop_change_req = FALSE;
			vidc_720p_decode_dynamic_req_reset();
		}
		return;
	}
	if ((p_decoder->n_dynamic_prop_change &
				DDL_DEC_REQ_OUTPUT_FLUSH)) {
		p_decoder->b_dynmic_prop_change_req = TRUE;
		p_decoder->n_dynamic_prop_change &= ~(DDL_DEC_REQ_OUTPUT_FLUSH);
		p_decoder->dpb_mask.n_hw_mask = 0;
		vidc_720p_decode_dynamic_req_set(VIDC_720P_FLUSH_REQ);
	}
	if (((p_decoder->n_meta_data_enable_flag & VCD_METADATA_PASSTHROUGH))
	    && ((VCD_FRAME_FLAG_EXTRADATA & p_bit_stream->n_flags))
	    ) {

		p_temp = ((uint8_t *)p_bit_stream->p_physical +
					p_bit_stream->n_offset +
					p_bit_stream->n_data_len + 3);

		n_extra_datastart = (u32) ((u32)p_temp & ~3);
		p_decoder->b_dynmic_prop_change_req = TRUE;

		vidc_720p_decode_setpassthrough_start(n_extra_datastart);

		vidc_720p_decode_dynamic_req_set(VIDC_720P_EXTRADATA);
	}
}

void ddl_encode_dynamic_property(struct ddl_client_context_type *p_ddl,
				 u32 b_enable)
{
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);
	u32 n_enc_param_change = 0;

	if (!b_enable) {
		if (p_encoder->b_dynmic_prop_change_req) {
			p_encoder->b_dynmic_prop_change_req = FALSE;
			p_encoder->n_ext_enc_control_val &=
				~(VIDC_720P_ENC_IFRAME_REQ);
			vidc_720p_encode_set_control_param
			(p_encoder->n_ext_enc_control_val);
			vidc_720p_encoder_set_param_change(n_enc_param_change);
		}
		return;
	}
	if ((p_encoder->n_dynamic_prop_change & DDL_ENC_REQ_IFRAME)) {
		p_encoder->n_dynamic_prop_change &= ~(DDL_ENC_REQ_IFRAME);
		p_encoder->n_ext_enc_control_val |= VIDC_720P_ENC_IFRAME_REQ;
		vidc_720p_encode_set_control_param
		(p_encoder->n_ext_enc_control_val);
	}
	if ((p_encoder->n_dynamic_prop_change & DDL_ENC_CHANGE_BITRATE)) {
		vidc_720p_encode_set_bit_rate(
		p_encoder->target_bit_rate.n_target_bitrate);
		n_enc_param_change |= VIDC_720P_ENC_BITRATE_CHANGE;
		p_encoder->n_dynamic_prop_change &= ~(DDL_ENC_CHANGE_BITRATE);
	}
	if ((p_encoder->n_dynamic_prop_change & DDL_ENC_CHANGE_IPERIOD)) {
		vidc_720p_encode_set_i_period
			(p_encoder->i_period.n_p_frames);
		n_enc_param_change |= VIDC_720P_ENC_IPERIOD_CHANGE;
		p_encoder->n_dynamic_prop_change &= ~(DDL_ENC_CHANGE_IPERIOD);
	}
	if ((p_encoder->n_dynamic_prop_change &
				DDL_ENC_CHANGE_FRAMERATE)) {
		vidc_720p_encode_set_fps
		    ((p_encoder->frame_rate.n_fps_numerator * 1000) /
		     p_encoder->frame_rate.n_fps_denominator);
		n_enc_param_change |= VIDC_720P_ENC_FRAMERATE_CHANGE;
		p_encoder->n_dynamic_prop_change &= ~(DDL_ENC_CHANGE_FRAMERATE);
	}
	if (n_enc_param_change)
		vidc_720p_encoder_set_param_change(n_enc_param_change);
}

static void ddl_encode_set_profile_level(struct ddl_client_context_type *p_ddl)
{
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);
	u32 profile;
	u32 n_level;

	switch (p_encoder->profile.e_profile) {
	default:
	case VCD_PROFILE_MPEG4_SP:
		{
			profile = VIDC_720P_PROFILE_MPEG4_SP;
			break;
		}
	case VCD_PROFILE_MPEG4_ASP:
		{
			profile = VIDC_720P_PROFILE_MPEG4_ASP;
			break;
		}
	case VCD_PROFILE_H264_BASELINE:
		{
			profile = VIDC_720P_PROFILE_H264_BASELINE;
			break;
		}
	case VCD_PROFILE_H264_MAIN:
		{
			profile = VIDC_720P_PROFILE_H264_MAIN;
			break;
		}
	case VCD_PROFILE_H264_HIGH:
		{
			profile = VIDC_720P_PROFILE_H264_HIGH;
			break;
		}
	case VCD_PROFILE_H263_BASELINE:
		{
			profile = VIDC_720P_PROFILE_H263_BASELINE;
			break;
		}
	}
	switch (p_encoder->level.e_level) {
	default:
	case VCD_LEVEL_MPEG4_0:
		{
			n_level = VIDC_720P_MPEG4_LEVEL0;
			break;
		}
	case VCD_LEVEL_MPEG4_0b:
		{
			n_level = VIDC_720P_MPEG4_LEVEL0b;
			break;
		}
	case VCD_LEVEL_MPEG4_1:
		{
			n_level = VIDC_720P_MPEG4_LEVEL1;
			break;
		}
	case VCD_LEVEL_MPEG4_2:
		{
			n_level = VIDC_720P_MPEG4_LEVEL2;
			break;
		}
	case VCD_LEVEL_MPEG4_3:
		{
			n_level = VIDC_720P_MPEG4_LEVEL3;
			break;
		}
	case VCD_LEVEL_MPEG4_3b:
		{
			n_level = VIDC_720P_MPEG4_LEVEL3b;
			break;
		}

	case VCD_LEVEL_MPEG4_4:
	case VCD_LEVEL_MPEG4_4a:
		{
			n_level = VIDC_720P_MPEG4_LEVEL4a;
			break;
		}
	case VCD_LEVEL_MPEG4_5:
		{
			n_level = VIDC_720P_MPEG4_LEVEL5;
			break;
		}
	case VCD_LEVEL_MPEG4_6:
		{
			n_level = VIDC_720P_MPEG4_LEVEL6;
			break;
		}
	case VCD_LEVEL_H264_1:
		{
			n_level = VIDC_720P_H264_LEVEL1;
			break;
		}
	case VCD_LEVEL_H264_1b:
		{
			n_level = VIDC_720P_H264_LEVEL1b;
			break;
		}
	case VCD_LEVEL_H264_1p1:
		{
			n_level = VIDC_720P_H264_LEVEL1p1;
			break;
		}
	case VCD_LEVEL_H264_1p2:
		{
			n_level = VIDC_720P_H264_LEVEL1p2;
			break;
		}
	case VCD_LEVEL_H264_1p3:
		{
			n_level = VIDC_720P_H264_LEVEL1p3;
			break;
		}
	case VCD_LEVEL_H264_2:
		{
			n_level = VIDC_720P_H264_LEVEL2;
			break;
		}
	case VCD_LEVEL_H264_2p1:
		{
			n_level = VIDC_720P_H264_LEVEL2p1;
			break;
		}
	case VCD_LEVEL_H264_2p2:
		{
			n_level = VIDC_720P_H264_LEVEL2p2;
			break;
		}
	case VCD_LEVEL_H264_3:
		{
			n_level = VIDC_720P_H264_LEVEL3;
			break;
		}
	case VCD_LEVEL_H264_3p1:
		{
			n_level = VIDC_720P_H264_LEVEL3p1;
			break;
		}
	case VCD_LEVEL_H263_10:
		{
			n_level = VIDC_720P_H263_LEVEL10;
			break;
		}
	case VCD_LEVEL_H263_20:
		{
			n_level = VIDC_720P_H263_LEVEL20;
			break;
		}
	case VCD_LEVEL_H263_30:
		{
			n_level = VIDC_720P_H263_LEVEL30;
			break;
		}
	case VCD_LEVEL_H263_40:
		{
			n_level = VIDC_720P_H263_LEVEL40;
			break;
		}
	case VCD_LEVEL_H263_45:
		{
			n_level = VIDC_720P_H263_LEVEL45;
			break;
		}
	case VCD_LEVEL_H263_50:
		{
			n_level = VIDC_720P_H263_LEVEL50;
			break;
		}
	case VCD_LEVEL_H263_60:
		{
			n_level = VIDC_720P_H263_LEVEL60;
			break;
		}
	case VCD_LEVEL_H263_70:
		{
			n_level = VIDC_720P_H263_LEVEL70;
			break;
		}
	}
	vidc_720p_encode_set_profile(profile, n_level);
}

void ddl_encode_init_codec(struct ddl_client_context_type *p_ddl)
{
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);
	enum vidc_720p_memory_access_method_type mem_access_method;
	enum vidc_720p_DBConfig_type e_db_config;
	enum vidc_720p_MSlice_selection_type e_m_slice_sel;

	ddl_encode_set_profile_level(p_ddl);

	vidc_720p_set_frame_size
	    (p_encoder->frame_size.n_width, p_encoder->frame_size.n_height);
	vidc_720p_encode_set_qp_params
	    (p_encoder->qp_range.n_max_qp, p_encoder->qp_range.n_min_qp);
	vidc_720p_encode_set_rc_config
	    (p_encoder->rc_level.b_frame_level_rc,
	     p_encoder->rc_level.b_mb_level_rc,
	     p_encoder->session_qp.n_i_frame_qp,
	     p_encoder->session_qp.n_p_frame_qp);

	if (p_encoder->n_r_cframe_skip) {
		if (p_encoder->n_vb_vbuffer_size) {
			p_encoder->n_ext_enc_control_val = (0x2 << 0x2) |
			(p_encoder->n_vb_vbuffer_size << 0x10);
		} else
			p_encoder->n_ext_enc_control_val = (0x1 << 2);
	} else
		p_encoder->n_ext_enc_control_val = 0;

	vidc_720p_encode_set_fps
	    ((p_encoder->frame_rate.n_fps_numerator * 1000) /
	     p_encoder->frame_rate.n_fps_denominator);

	vidc_720p_encode_set_vop_time(
			p_encoder->vop_timing.n_vop_time_resolution, 0);

	if (p_encoder->rc_level.b_frame_level_rc) {
		vidc_720p_encode_set_bit_rate
		    (p_encoder->target_bit_rate.n_target_bitrate);

		vidc_720p_encode_set_frame_level_rc_params
		    (p_encoder->frame_level_rc.n_reaction_coeff);
	}
	if (p_encoder->rc_level.b_mb_level_rc) {
		vidc_720p_encode_set_mb_level_rc_params
		    (p_encoder->adaptive_rc.b_dark_region_as_flag,
		     p_encoder->adaptive_rc.b_smooth_region_as_flag,
		     p_encoder->adaptive_rc.b_static_region_as_flag,
		     p_encoder->adaptive_rc.b_activity_region_flag);
	}
	if (p_encoder->codec_type.e_codec == VCD_CODEC_MPEG4) {
		vidc_720p_encode_set_short_header
		    (p_encoder->short_header.b_short_header);

		if (p_encoder->n_hdr_ext_control) {
			vidc_720p_encode_set_hec_period
			(p_encoder->n_hdr_ext_control);
			p_encoder->n_ext_enc_control_val |= (0x1 << 0x1);
		}
	}
	/* set extended encoder control settings */
	vidc_720p_encode_set_control_param
	(p_encoder->n_ext_enc_control_val);

	if (p_encoder->codec_type.e_codec == VCD_CODEC_H264) {
		enum vidc_720p_entropy_sel_type e_entropy_sel;
		enum vidc_720p_cabac_model_type e_cabac_model_number;
		switch (p_encoder->entropy_control.e_entropy_sel) {
		default:
		case VCD_ENTROPY_SEL_CAVLC:
			{
				e_entropy_sel = VIDC_720P_ENTROPY_SEL_CAVLC;
				break;
			}
		case VCD_ENTROPY_SEL_CABAC:
			{
				e_entropy_sel = VIDC_720P_ENTROPY_SEL_CABAC;
				break;
			}
		}
		switch (p_encoder->entropy_control.e_cabac_model) {
		default:
		case VCD_CABAC_MODEL_NUMBER_0:
			{
				e_cabac_model_number =
				    VIDC_720P_CABAC_MODEL_NUMBER_0;
				break;
			}
		case VCD_CABAC_MODEL_NUMBER_1:
			{
				e_cabac_model_number =
				    VIDC_720P_CABAC_MODEL_NUMBER_1;
				break;
			}
		case VCD_CABAC_MODEL_NUMBER_2:
			{
				e_cabac_model_number =
				    VIDC_720P_CABAC_MODEL_NUMBER_2;
				break;
			}
		}
		vidc_720p_encode_set_entropy_control
		    (e_entropy_sel, e_cabac_model_number);
		switch (p_encoder->db_control.e_db_config) {
		default:
		case VCD_DB_ALL_BLOCKING_BOUNDARY:
			{
				e_db_config =
				    VIDC_720P_DB_ALL_BLOCKING_BOUNDARY;
				break;
			}
		case VCD_DB_DISABLE:
			{
				e_db_config =
				    VIDC_720P_DB_DISABLE;
				break;
			}
		case VCD_DB_SKIP_SLICE_BOUNDARY:
			{
				e_db_config =
				    VIDC_720P_DB_SKIP_SLICE_BOUNDARY;
				break;
			}
		}
		vidc_720p_encode_set_db_filter_control
		    (e_db_config,
		     p_encoder->db_control.n_slice_alpha_offset,
		     p_encoder->db_control.n_slice_beta_offset);
	}

	vidc_720p_encode_set_intra_refresh_mb_number
	    (p_encoder->intra_refresh.n_cir_mb_number);

	switch (p_encoder->multi_slice.e_m_slice_sel) {
	default:
	case VCD_MSLICE_OFF:
		e_m_slice_sel = VIDC_720P_MSLICE_OFF;
		break;
	case VCD_MSLICE_BY_MB_COUNT:
		{
			e_m_slice_sel = VIDC_720P_MSLICE_BY_MB_COUNT;
			break;
		}
	case VCD_MSLICE_BY_BYTE_COUNT:
		{
			e_m_slice_sel = VIDC_720P_MSLICE_BY_BYTE_COUNT;
			break;
		}
	case VCD_MSLICE_BY_GOB:
		{
			e_m_slice_sel = VIDC_720P_MSLICE_BY_GOB;
			break;
		}
	}
	vidc_720p_encode_set_multi_slice_info
	    (e_m_slice_sel, p_encoder->multi_slice.n_m_slice_size);

	vidc_720p_encode_set_dpb_buffer
	    (p_encoder->enc_dpb_addr.p_align_physical_addr,
			 p_encoder->enc_dpb_addr.n_buffer_size);

	VIDC_LOG1("ENC_DPB_ADDR_SIZE", p_encoder->enc_dpb_addr.n_buffer_size);

	vidc_720p_encode_set_i_period(p_encoder->i_period.n_p_frames);

	ddl_metadata_enable(p_ddl);

	if (p_encoder->seq_header.p_virtual_base_addr) {
		u32 n_ext_buffer_start, n_ext_buffer_end, n_start_byte_num;
		n_ext_buffer_start =
		    (u32) p_encoder->seq_header.p_align_physical_addr;
		n_ext_buffer_end =
		    n_ext_buffer_start + p_encoder->seq_header.n_buffer_size;
		n_start_byte_num =
		    (n_ext_buffer_start & DDL_STREAMBUF_ALIGN_GUARD_BYTES);
		n_ext_buffer_start &= ~(DDL_STREAMBUF_ALIGN_GUARD_BYTES);
		n_ext_buffer_end &= ~(DDL_STREAMBUF_ALIGN_GUARD_BYTES);
		VIDC_LOG1("ENC_SEQHDR_ALLOC_SIZE",
			   p_encoder->seq_header.n_buffer_size);
		vidc_720p_encode_set_seq_header_buffer(n_ext_buffer_start,
							n_ext_buffer_end,
							n_start_byte_num);
	}

	if (p_encoder->re_con_buf_format.e_buffer_format ==
		VCD_BUFFER_FORMAT_NV12)
		mem_access_method = VIDC_720P_TILE_LINEAR;
	else
		mem_access_method = VIDC_720P_TILE_16x16;

	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODECDONE);
	ddl_move_command_state(p_ddl->p_ddl_context, DDL_CMD_INIT_CODEC);

	vidc_720p_encode_init_codec(p_ddl->n_channel_id, mem_access_method);
}

void ddl_channel_end(struct ddl_client_context_type *p_ddl)
{
	VIDC_DEBUG_REGISTER_LOG;

	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_CHEND);
	ddl_move_command_state(p_ddl->p_ddl_context, DDL_CMD_CHANNEL_END);

	vidc_720p_submit_command(p_ddl->n_channel_id, VIDC_720P_CMD_CHEND);
}

void ddl_encode_frame_run(struct ddl_client_context_type *p_ddl)
{
	u32 n_ext_buffer_start, n_ext_buffer_end;
	u32 n_y_addr, n_c_addr;
	u32 n_start_byte_number = 0;
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);
	struct vcd_frame_data_type *p_stream = &(p_ddl->output_frame.vcd_frm);

	n_ext_buffer_start = (u32) p_stream->p_physical + p_stream->n_offset;
	n_ext_buffer_end = ddl_encode_set_metadata_output_buf(p_ddl);
	n_start_byte_number =
	    (n_ext_buffer_start & DDL_STREAMBUF_ALIGN_GUARD_BYTES);
	if (n_start_byte_number) {
		u32 n_upper_data, n_lower_data;
		u32 *p_align_virtual_addr;
		n_ext_buffer_start &= ~(DDL_STREAMBUF_ALIGN_GUARD_BYTES);
		p_align_virtual_addr = (u32 *) (((u32) p_stream->p_virtual +
						 p_stream->n_offset) -
						n_start_byte_number);
		n_upper_data = *p_align_virtual_addr;
		p_align_virtual_addr++;
		n_lower_data = *p_align_virtual_addr;
		vidc_720p_encode_unalign_bitstream(n_upper_data, n_lower_data);
	}

	n_y_addr = (u32) p_ddl->input_frame.vcd_frm.p_physical +
	    p_ddl->input_frame.vcd_frm.n_offset;
	n_c_addr = (n_y_addr + (p_encoder->frame_size.n_height *
				p_encoder->frame_size.n_width));
	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE);
	ddl_move_command_state(p_ddl->p_ddl_context, DDL_CMD_ENCODE_FRAME);

	if (p_encoder->n_dynamic_prop_change) {
		p_encoder->b_dynmic_prop_change_req = TRUE;
		ddl_encode_dynamic_property(p_ddl, TRUE);
	}
	vidc_720p_encode_set_vop_time(
			p_encoder->vop_timing.n_vop_time_resolution,
			p_ddl->input_frame.n_frm_delta
	    );

	vidc_720p_encode_frame(p_ddl->n_channel_id,
			n_ext_buffer_start,
				n_ext_buffer_end,
				n_start_byte_number, n_y_addr, n_c_addr);
}

u32 ddl_decode_set_buffers(struct ddl_client_context_type *p_ddl)
{
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	u32 n_comv_buf_size = DDL_COMV_BUFLINE_NO, n_comv_buf_no = 0;
	u32 n_ref_buf_no = 0;

	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_DPB)) {
		VIDC_LOG_STRING("STATE-CRITICAL");
		return VCD_ERR_FAIL;
	}

	switch (p_decoder->codec_type.e_codec) {
	default:
	case VCD_CODEC_DIVX_4:
	case VCD_CODEC_DIVX_5:
	case VCD_CODEC_DIVX_6:
	case VCD_CODEC_XVID:
	case VCD_CODEC_MPEG2:
	case VCD_CODEC_MPEG4:
		{
			n_comv_buf_no = DDL_MPEG_COMV_BUF_NO;
			n_ref_buf_no = DDL_MPEG_REFBUF_COUNT;
			break;
		}
	case VCD_CODEC_H263:
		{
			n_comv_buf_no = DDL_H263_COMV_BUF_NO;
			break;
		}
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		{
			n_comv_buf_no =
			    p_decoder->client_output_buf_req.n_actual_count + 1;
			n_comv_buf_size = DDL_VC1_COMV_BUFLINE_NO;
			break;
		}
	case VCD_CODEC_H264:
		{
			n_comv_buf_no =
			    p_decoder->client_output_buf_req.n_actual_count;
			break;
		}
	}

	if (n_comv_buf_no) {
		n_comv_buf_size *= (n_comv_buf_no *
				    (((p_decoder->client_frame_size.
				     n_width + 15) >> 4)) *
				    (((p_decoder->client_frame_size.
				      n_height + 15) >> 4) + 1));
		ddl_pmem_alloc(&p_decoder->dpb_comv_buffer, n_comv_buf_size,
			       DDL_LINEAR_BUFFER_ALIGN_BYTES);
		if (!p_decoder->dpb_comv_buffer.p_virtual_base_addr) {
			VIDC_LOGERR_STRING
			    ("Dec_set_buf:Comv_buf_alloc_failed");
			return VCD_ERR_ALLOC_FAIL;
		}
		vidc_720p_decode_set_comv_buffer(p_decoder->dpb_comv_buffer.
						  p_align_physical_addr,
						  p_decoder->dpb_comv_buffer.
						  n_buffer_size);
	}
	p_decoder->ref_buffer.p_align_physical_addr = NULL;
	if (n_ref_buf_no) {
		u32 n_size, n_yuv_size, n_align_bytes;
		n_yuv_size = ddl_get_yuv_buffer_size(&p_decoder->
			client_frame_size, &p_decoder->buf_format,
			(!p_decoder->n_progressive_only));
		n_size = n_yuv_size * n_ref_buf_no;
		if (p_decoder->buf_format.e_buffer_format ==
			VCD_BUFFER_FORMAT_NV12)
			n_align_bytes = DDL_LINEAR_BUFFER_ALIGN_BYTES;
		else
			n_align_bytes = DDL_TILE_BUFFER_ALIGN_BYTES;

		ddl_pmem_alloc(&p_decoder->ref_buffer, n_size, n_align_bytes);
		if (!p_decoder->ref_buffer.p_virtual_base_addr) {
			ddl_pmem_free(p_decoder->dpb_comv_buffer);
			VIDC_LOGERR_STRING
			    ("Dec_set_buf:mpeg_ref_buf_alloc_failed");
			return VCD_ERR_ALLOC_FAIL;
		}
	}
	ddl_decode_set_metadata_output(p_decoder);

	ddl_decoder_dpb_transact(p_decoder, NULL, DDL_DPB_OP_INIT);

	if (p_decoder->codec_type.e_codec == VCD_CODEC_H264) {
		vidc_720p_decode_setH264VSPBuffer(p_decoder->
						   h264Vsp_temp_buffer.
						   p_align_physical_addr);
		VIDC_LOG1("VSP_BUF_ADDR_SIZE",
			   p_decoder->h264Vsp_temp_buffer.n_buffer_size);
	}

	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_DPBDONE);
	ddl_move_command_state(p_ddl->p_ddl_context, DDL_CMD_DECODE_SET_DPB);

	vidc_720p_submit_command(p_ddl->n_channel_id,
		VIDC_720P_CMD_INITBUFFERS);
	return VCD_S_SUCCESS;
}

void ddl_decode_frame_run(struct ddl_client_context_type *p_ddl)
{
	u32 n_ext_buffer_start = 0, n_ext_buffer_end = 0;
	u32 n_start_byte_num = 8;
	struct ddl_decoder_data_type *p_decoder = &p_ddl->codec_data.decoder;
	struct vcd_frame_data_type *p_bit_stream =
	    &(p_ddl->input_frame.vcd_frm);

	if (!p_bit_stream->n_data_len ||
		!p_bit_stream->p_physical) {
		ddl_decode_eos_run(p_ddl);
		return;
	}

	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE);

	ddl_decode_dynamic_property(p_ddl, TRUE);

	ddl_decoder_dpb_transact(p_decoder, NULL, DDL_DPB_OP_SET_MASK);

	n_ext_buffer_start = (u32)p_bit_stream->p_physical +
		p_bit_stream->n_offset;
	n_start_byte_num = 8 - (n_ext_buffer_start &
		DDL_STREAMBUF_ALIGN_GUARD_BYTES);
	n_ext_buffer_end = n_ext_buffer_start + p_bit_stream->n_data_len;
	n_ext_buffer_start &= ~(DDL_STREAMBUF_ALIGN_GUARD_BYTES);
	DDL_PADDING_HACK(n_ext_buffer_end);

	ddl_move_command_state(p_ddl->p_ddl_context, DDL_CMD_DECODE_FRAME);

	vidc_720p_decode_frame(p_ddl->n_channel_id,
			n_ext_buffer_start,
			n_ext_buffer_end,
			p_bit_stream->n_data_len,
			n_start_byte_num, p_bit_stream->n_ip_frm_tag);
}

void  ddl_decode_eos_run(struct ddl_client_context_type *p_ddl)
{
	struct ddl_decoder_data_type *p_decoder = &p_ddl->codec_data.decoder;

	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE);

	ddl_decode_dynamic_property(p_ddl, TRUE);

	ddl_decoder_dpb_transact(p_decoder, NULL, DDL_DPB_OP_SET_MASK);

	p_decoder->b_dynmic_prop_change_req = TRUE;

	ddl_move_command_state(p_ddl->p_ddl_context, DDL_CMD_EOS);

	vidc_720p_issue_eos(p_ddl->n_channel_id);
}

u32 ddl_hal_engine_reset(struct ddl_context_type *p_ddl_context)
{
	u32 b_eng_reset;
	u32 n_channel_id = 0;
	u32 fw_endianness;
	enum vidc_720p_endian_type e_dma_endian;
	enum vidc_720p_interrupt_level_selection_type e_interrupt_sel;
	u32 intr_mask = 0x0;

	if (p_ddl_context->p_current_ddl)
		n_channel_id = p_ddl_context->p_current_ddl->n_channel_id;

	e_interrupt_sel = VIDC_720P_INTERRUPT_LEVEL_SEL;
	/* Enable all the supported interrupt */
	intr_mask |= VIDC_720P_INTR_BUFFER_FULL;
	intr_mask |= VIDC_720P_INTR_FW_DONE;
	intr_mask |= VIDC_720P_INTR_DMA_DONE;
	intr_mask |= VIDC_720P_INTR_FRAME_DONE;

	vcd_get_fw_property(VCD_FW_ENDIAN, &fw_endianness);
	/* Reverse the endianness settings after boot code download */
	if (fw_endianness == VCD_FW_BIG_ENDIAN)
		e_dma_endian = VIDC_720P_LITTLE_ENDIAN;
	else
		e_dma_endian = VIDC_720P_BIG_ENDIAN;

	/* Need to reset MFC silently */
	b_eng_reset = vidc_720p_engine_reset(
		n_channel_id,
		e_dma_endian, e_interrupt_sel,
		intr_mask);
	if (!b_eng_reset) {
		/* call the hw fatal callback if engine reset fails */
		ddl_hw_fatal_cb(p_ddl_context);
	}
	return b_eng_reset ;
}
