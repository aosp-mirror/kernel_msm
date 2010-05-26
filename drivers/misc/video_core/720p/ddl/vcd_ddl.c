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

u32 ddl_device_init(struct ddl_init_config_type *p_ddl_init_config,
		    void *p_client_data)
{
	struct ddl_context_type *p_ddl_context;
	u32 status = VCD_S_SUCCESS;

	if ((!p_ddl_init_config) ||
	    (!p_ddl_init_config->ddl_callback) ||
	    (!p_ddl_init_config->p_core_virtual_base_addr)
	    ) {
		VIDC_LOGERR_STRING("ddl_dev_init:Bad_argument");
		return VCD_ERR_ILLEGAL_PARM;
	}

	p_ddl_context = ddl_get_context();

	if (DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_dev_init:Multiple_init");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_dev_init:Ddl_busy");
		return VCD_ERR_BUSY;
	}

	DDL_MEMSET(p_ddl_context, 0, sizeof(struct ddl_context_type));

	DDL_BUSY(p_ddl_context);

	p_ddl_context->ddl_callback = p_ddl_init_config->ddl_callback;
	p_ddl_context->pf_interrupt_clr = p_ddl_init_config->pf_interrupt_clr;
	p_ddl_context->p_core_virtual_base_addr =
	    p_ddl_init_config->p_core_virtual_base_addr;
	p_ddl_context->p_client_data = p_client_data;

	p_ddl_context->intr_status = DDL_INVALID_INTR_STATUS;

	vidc_720p_set_device_virtual_base(p_ddl_context->
					   p_core_virtual_base_addr);

	p_ddl_context->p_current_ddl = NULL;
	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);

	ddl_client_transact(DDL_INIT_CLIENTS, NULL);

	ddl_pmem_alloc(&p_ddl_context->context_buf_addr,
		       DDL_CONTEXT_MEMORY, DDL_LINEAR_BUFFER_ALIGN_BYTES);
	if (!p_ddl_context->context_buf_addr.p_virtual_base_addr) {
		VIDC_LOGERR_STRING("ddl_dev_init:Context_alloc_fail");
		status = VCD_ERR_ALLOC_FAIL;
	}
	if (!status) {
		ddl_pmem_alloc(&p_ddl_context->db_line_buffer,
			       DDL_DB_LINE_BUF_SIZE,
			       DDL_TILE_BUFFER_ALIGN_BYTES);
		if (!p_ddl_context->db_line_buffer.p_virtual_base_addr) {
			VIDC_LOGERR_STRING("ddl_dev_init:Line_buf_alloc_fail");
			status = VCD_ERR_ALLOC_FAIL;
		}
	}

	if (!status) {
		ddl_pmem_alloc(&p_ddl_context->data_partition_tempbuf,
					   DDL_MPEG4_DATA_PARTITION_BUF_SIZE,
					   DDL_TILE_BUFFER_ALIGN_BYTES);
		if (p_ddl_context->data_partition_tempbuf.p_virtual_base_addr \
			== NULL) {
			VIDC_LOGERR_STRING
				("ddl_dev_init:Data_partition_buf_alloc_fail");
			status = VCD_ERR_ALLOC_FAIL;
		}
   }

   if (!status) {

		ddl_pmem_alloc(&p_ddl_context->metadata_shared_input,
					   DDL_METADATA_TOTAL_INPUTBUFSIZE,
					   DDL_LINEAR_BUFFER_ALIGN_BYTES);
		if (!p_ddl_context->metadata_shared_input.p_virtual_base_addr) {
			VIDC_LOGERR_STRING
			("ddl_dev_init:metadata_shared_input_alloc_fail");
			status = VCD_ERR_ALLOC_FAIL;
		}
	 }

	if (!status) {
		ddl_pmem_alloc(&p_ddl_context->dbg_core_dump, \
					   DDL_DBG_CORE_DUMP_SIZE,  \
					   DDL_LINEAR_BUFFER_ALIGN_BYTES);
		if (!p_ddl_context->dbg_core_dump.p_virtual_base_addr) {
			VIDC_LOGERR_STRING
				("ddl_dev_init:dbg_core_dump_alloc_failed");
			status = VCD_ERR_ALLOC_FAIL;
		}
		p_ddl_context->enable_dbg_core_dump = 0;
	}

	if (!status && !vcd_fw_init()) {
		VIDC_LOGERR_STRING("ddl_dev_init:fw_init_failed");
		status = VCD_ERR_ALLOC_FAIL;
	}
	if (status) {
		ddl_release_context_buffers(p_ddl_context);
		DDL_IDLE(p_ddl_context);
		return status;
	}

	ddl_move_command_state(p_ddl_context, DDL_CMD_DMA_INIT);

	ddl_core_init(p_ddl_context);

	return status;
}

u32 ddl_device_release(void *p_client_data)
{
	struct ddl_context_type *p_ddl_context;

	p_ddl_context = ddl_get_context();

	if (DDL_IS_BUSY(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_dev_rel:Ddl_busy");
		return VCD_ERR_BUSY;
	}

	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_dev_rel:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (!ddl_client_transact(DDL_ACTIVE_CLIENT, NULL)) {
		VIDC_LOGERR_STRING("ddl_dev_rel:Client_present_err");
		return VCD_ERR_CLIENT_PRESENT;
	}
	DDL_BUSY(p_ddl_context);

	p_ddl_context->n_device_state = DDL_DEVICE_NOTINIT;
	p_ddl_context->p_client_data = p_client_data;
	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	vidc_720p_stop_fw();

	VIDC_LOG_STRING("FW_ENDDONE");
	ddl_release_context_buffers(p_ddl_context);

	DDL_IDLE(p_ddl_context);

	return VCD_S_SUCCESS;
}

u32 ddl_open(u32 **p_ddl_handle, u32 b_decoding)
{
	struct ddl_context_type *p_ddl_context;
	struct ddl_client_context_type *p_ddl;
	u32 status;

	if (!p_ddl_handle) {
		VIDC_LOGERR_STRING("ddl_open:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}

	p_ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_open:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}

	status = ddl_client_transact(DDL_GET_CLIENT, &p_ddl);

	if (status) {
		VIDC_LOGERR_STRING("ddl_open:Client_trasac_failed");
		return status;
	}

	ddl_move_client_state(p_ddl, DDL_CLIENT_OPEN);

	p_ddl->codec_data.hdr.b_decoding = b_decoding;
	p_ddl->b_decoding = b_decoding;

	ddl_set_default_meta_data_hdr(p_ddl);

	ddl_set_initial_default_values(p_ddl);

	*p_ddl_handle = (u32 *) p_ddl;
	return VCD_S_SUCCESS;
}

u32 ddl_close(u32 **p_ddl_handle)
{
	struct ddl_context_type *p_ddl_context;
	struct ddl_client_context_type **pp_ddl =
	    (struct ddl_client_context_type **)p_ddl_handle;

	if (!pp_ddl || !*pp_ddl) {
		VIDC_LOGERR_STRING("ddl_close:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}

	p_ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_close:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (!DDLCLIENT_STATE_IS(*pp_ddl, DDL_CLIENT_OPEN)) {
		VIDC_LOGERR_STRING("ddl_close:Not_in_open_state");
		return VCD_ERR_ILLEGAL_OP;
	}

	ddl_move_client_state(*pp_ddl, DDL_CLIENT_INVALID);
	if ((*pp_ddl)->b_decoding) {
		vcd_fw_transact(FALSE, TRUE,
			(*pp_ddl)->codec_data.decoder.codec_type.e_codec);
	} else {
		vcd_fw_transact(FALSE, FALSE,
			(*pp_ddl)->codec_data.encoder.codec_type.e_codec);
	}
	ddl_client_transact(DDL_FREE_CLIENT, pp_ddl);

	return VCD_S_SUCCESS;
}

u32 ddl_encode_start(u32 *ddl_handle, void *p_client_data)
{
	struct ddl_client_context_type *p_ddl =
	    (struct ddl_client_context_type *)ddl_handle;
	struct ddl_context_type *p_ddl_context;
	struct ddl_encoder_data_type *p_encoder;
	u32 n_dpb_size;

	p_ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_enc_start:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_enc_start:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || p_ddl->b_decoding) {
		VIDC_LOGERR_STRING("ddl_enc_start:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}

	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_OPEN)) {
		VIDC_LOGERR_STRING("ddl_enc_start:Not_opened");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (!ddl_encoder_ready_to_start(p_ddl)) {
		VIDC_LOGERR_STRING("ddl_enc_start:Err_param_settings");
		return VCD_ERR_ILLEGAL_OP;
	}

	p_encoder = &p_ddl->codec_data.encoder;

	n_dpb_size = ddl_get_yuv_buffer_size(&p_encoder->frame_size,
					&p_encoder->re_con_buf_format, FALSE);

	n_dpb_size *= DDL_ENC_NUM_DPB_BUFFERS;
	ddl_pmem_alloc(&p_encoder->enc_dpb_addr,
		       n_dpb_size, DDL_TILE_BUFFER_ALIGN_BYTES);
	if (!p_encoder->enc_dpb_addr.p_virtual_base_addr) {
		VIDC_LOGERR_STRING("ddl_enc_start:Dpb_alloc_failed");
		return VCD_ERR_ALLOC_FAIL;
	}

	if ((p_encoder->codec_type.e_codec == VCD_CODEC_MPEG4 &&
	     !p_encoder->short_header.b_short_header) ||
	    p_encoder->codec_type.e_codec == VCD_CODEC_H264) {
		ddl_pmem_alloc(&p_encoder->seq_header,
			       DDL_ENC_SEQHEADER_SIZE,
			       DDL_LINEAR_BUFFER_ALIGN_BYTES);
		if (!p_encoder->seq_header.p_virtual_base_addr) {
			ddl_pmem_free(p_encoder->enc_dpb_addr);
			VIDC_LOGERR_STRING
			    ("ddl_enc_start:Seq_hdr_alloc_failed");
			return VCD_ERR_ALLOC_FAIL;
		}
	} else {
		p_encoder->seq_header.n_buffer_size = 0;
		p_encoder->seq_header.p_virtual_base_addr = 0;
	}

	DDL_BUSY(p_ddl_context);

	p_ddl_context->p_current_ddl = p_ddl;
	p_ddl_context->p_client_data = p_client_data;
	ddl_channel_set(p_ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_decode_start(u32 *ddl_handle,
     struct vcd_sequence_hdr_type *p_header, void *p_client_data)
{
	struct ddl_client_context_type *p_ddl =
	    (struct ddl_client_context_type *)ddl_handle;
	struct ddl_context_type *p_ddl_context;
	struct ddl_decoder_data_type *p_decoder;

	p_ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_dec_start:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_dec_start:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || !p_ddl->b_decoding) {
		VIDC_LOGERR_STRING("ddl_dec_start:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_OPEN)) {
		VIDC_LOGERR_STRING("ddl_dec_start:Not_in_opened_state");
		return VCD_ERR_ILLEGAL_OP;
	}

	if ((p_header) &&
	    ((!p_header->n_sequence_header_len) ||
	     (!p_header->p_sequence_header)
	    )
	    ) {
		VIDC_LOGERR_STRING("ddl_dec_start:Bad_param_seq_header");
		return VCD_ERR_ILLEGAL_PARM;
	}

	if (!ddl_decoder_ready_to_start(p_ddl, p_header)) {
		VIDC_LOGERR_STRING("ddl_dec_start:Err_param_settings");
		return VCD_ERR_ILLEGAL_OP;
	}

	DDL_BUSY(p_ddl_context);

	p_decoder = &p_ddl->codec_data.decoder;
	if (p_header) {
		p_decoder->b_header_in_start = TRUE;
		p_decoder->decode_config = *p_header;
	} else {
		p_decoder->b_header_in_start = FALSE;
		p_decoder->decode_config.n_sequence_header_len = 0;
	}

	if (p_decoder->codec_type.e_codec == VCD_CODEC_H264) {
		ddl_pmem_alloc(&p_decoder->h264Vsp_temp_buffer,
			       DDL_DECODE_H264_VSPTEMP_BUFSIZE,
			       DDL_LINEAR_BUFFER_ALIGN_BYTES);
		if (!p_decoder->h264Vsp_temp_buffer.p_virtual_base_addr) {
			DDL_IDLE(p_ddl_context);
			VIDC_LOGERR_STRING
			    ("ddl_dec_start:H264Sps_alloc_failed");
			return VCD_ERR_ALLOC_FAIL;
		}
	}

	p_ddl_context->p_current_ddl = p_ddl;
	p_ddl_context->p_client_data = p_client_data;

	ddl_channel_set(p_ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_decode_frame(u32 *ddl_handle,
     struct ddl_frame_data_type_tag *p_input_bits, void *p_client_data)
{
	u32 vcd_status = VCD_S_SUCCESS;
	struct ddl_client_context_type *p_ddl =
	    (struct ddl_client_context_type *)ddl_handle;
	struct ddl_context_type *p_ddl_context = ddl_get_context();

#ifdef CORE_TIMING_INFO
	ddl_get_core_start_time(0);
#endif

	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_dec_frame:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_dec_frame:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || !p_ddl->b_decoding) {
		VIDC_LOGERR_STRING("ddl_dec_frame:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!p_input_bits ||
	    ((!p_input_bits->vcd_frm.p_physical ||
	      !p_input_bits->vcd_frm.n_data_len) &&
	     (!(VCD_FRAME_FLAG_EOS & p_input_bits->vcd_frm.n_flags))
	    )
	    ) {
		VIDC_LOGERR_STRING("ddl_dec_frame:Bad_input_param");
		return VCD_ERR_ILLEGAL_PARM;
	}

	DDL_BUSY(p_ddl_context);

	p_ddl_context->p_current_ddl = p_ddl;
	p_ddl_context->p_client_data = p_client_data;

	p_ddl->input_frame = *p_input_bits;

	if (DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME)) {
		ddl_decode_frame_run(p_ddl);
	} else {
		if (!p_ddl->codec_data.decoder.dp_buf.n_no_of_dec_pic_buf) {
			VIDC_LOGERR_STRING("ddl_dec_frame:Dpbs_requied");
			vcd_status = VCD_ERR_ILLEGAL_OP;
		} else if (DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_DPB)) {
			vcd_status = ddl_decode_set_buffers(p_ddl);
		} else
		    if (DDLCLIENT_STATE_IS
			(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODEC)) {
			p_ddl->codec_data.decoder.decode_config.
			    p_sequence_header =
			    p_ddl->input_frame.vcd_frm.p_physical;
			p_ddl->codec_data.decoder.decode_config.
			    n_sequence_header_len =
			    p_ddl->input_frame.vcd_frm.n_data_len;
			ddl_decode_init_codec(p_ddl);
		} else {
			VIDC_LOGERR_STRING("Dec_frame:Wrong_state");
			vcd_status = VCD_ERR_ILLEGAL_OP;
		}
		if (vcd_status)
			DDL_IDLE(p_ddl_context);
	}
	return vcd_status;
}

u32 ddl_encode_frame(u32 *ddl_handle,
     struct ddl_frame_data_type_tag *p_input_frame,
     struct ddl_frame_data_type_tag *p_output_bit, void *p_client_data)
{
	struct ddl_client_context_type *p_ddl =
	    (struct ddl_client_context_type *)ddl_handle;
	struct ddl_context_type *p_ddl_context = ddl_get_context();

#ifdef CORE_TIMING_INFO
	ddl_get_core_start_time(1);
#endif

	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_enc_frame:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_enc_frame:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || p_ddl->b_decoding) {
		VIDC_LOGERR_STRING("ddl_enc_frame:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!p_input_frame ||
	    !p_input_frame->vcd_frm.p_physical ||
	    p_ddl->codec_data.encoder.input_buf_req.n_size !=
	    p_input_frame->vcd_frm.n_data_len) {
		VIDC_LOGERR_STRING("ddl_enc_frame:Bad_input_params");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if ((((u32) p_input_frame->vcd_frm.p_physical +
		   p_input_frame->vcd_frm.n_offset) &
		  (DDL_STREAMBUF_ALIGN_GUARD_BYTES)
	    )
	    ) {
		VIDC_LOGERR_STRING
		    ("ddl_enc_frame:Un_aligned_yuv_start_address");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if (!p_output_bit ||
	    !p_output_bit->vcd_frm.p_physical ||
	    !p_output_bit->vcd_frm.n_alloc_len) {
		VIDC_LOGERR_STRING("ddl_enc_frame:Bad_output_params");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if ((p_ddl->codec_data.encoder.output_buf_req.n_size +
	     p_output_bit->vcd_frm.n_offset) >
	    p_output_bit->vcd_frm.n_alloc_len) {
		VIDC_LOGERR_STRING
		    ("ddl_enc_frame:n_offset_large, Exceeds_min_buf_size");
	}
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME)) {
		VIDC_LOGERR_STRING("ddl_enc_frame:Wrong_state");
		return VCD_ERR_ILLEGAL_OP;
	}

	DDL_BUSY(p_ddl_context);

	p_ddl_context->p_current_ddl = p_ddl;
	p_ddl_context->p_client_data = p_client_data;

	p_ddl->input_frame = *p_input_frame;
	p_ddl->output_frame = *p_output_bit;

	ddl_encode_frame_run(p_ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_decode_end(u32 *ddl_handle, void *p_client_data)
{
	struct ddl_client_context_type *p_ddl =
	    (struct ddl_client_context_type *)ddl_handle;
	struct ddl_context_type *p_ddl_context;

	p_ddl_context = ddl_get_context();

#ifdef CORE_TIMING_INFO
	ddl_reset_time_variables(0);
#endif

	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_dec_end:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_dec_end:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || !p_ddl->b_decoding) {
		VIDC_LOGERR_STRING("ddl_dec_end:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODEC) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_DPB) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_FATAL_ERROR)
	    ) {
		VIDC_LOGERR_STRING("ddl_dec_end:Wrong_state");
		return VCD_ERR_ILLEGAL_OP;
	}
	DDL_BUSY(p_ddl_context);

	p_ddl_context->p_current_ddl = p_ddl;
	p_ddl_context->p_client_data = p_client_data;

	ddl_channel_end(p_ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_encode_end(u32 *ddl_handle, void *p_client_data)
{
	struct ddl_client_context_type *p_ddl =
	    (struct ddl_client_context_type *)ddl_handle;
	struct ddl_context_type *p_ddl_context;

	p_ddl_context = ddl_get_context();

#ifdef CORE_TIMING_INFO
	ddl_reset_time_variables(1);
#endif

	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_enc_end:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		VIDC_LOGERR_STRING("ddl_enc_end:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || p_ddl->b_decoding) {
		VIDC_LOGERR_STRING("ddl_enc_end:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODEC) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_FATAL_ERROR)) {
		VIDC_LOGERR_STRING("ddl_enc_end:Wrong_state");
		return VCD_ERR_ILLEGAL_OP;
	}
	DDL_BUSY(p_ddl_context);

	p_ddl_context->p_current_ddl = p_ddl;
	p_ddl_context->p_client_data = p_client_data;

	ddl_channel_end(p_ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_reset_hw(u32 n_mode)
{
	struct ddl_context_type *p_ddl_context;
	struct ddl_client_context_type *p_ddl;
	int i_client_num;

	VIDC_LOG_STRING("ddl_reset_hw:called");
	p_ddl_context = ddl_get_context();
	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	DDL_BUSY(p_ddl_context);

	if (p_ddl_context->p_core_virtual_base_addr)
		vidc_720p_do_sw_reset();

	p_ddl_context->n_device_state = DDL_DEVICE_NOTINIT;
	for (i_client_num = 0; i_client_num < VCD_MAX_NO_CLIENT;
			++i_client_num) {
		p_ddl = p_ddl_context->a_ddl_clients[i_client_num];
		p_ddl_context->a_ddl_clients[i_client_num] = NULL;
		if (p_ddl) {
			ddl_release_client_internal_buffers(p_ddl);
			ddl_client_transact(DDL_FREE_CLIENT, &p_ddl);
		}
	}

	ddl_release_context_buffers(p_ddl_context);
	DDL_MEMSET(p_ddl_context, 0, sizeof(struct ddl_context_type));

	VIDC_LOG_BUFFER_INIT;
	return TRUE;
}
