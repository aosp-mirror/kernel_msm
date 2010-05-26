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

static void ddl_decoder_input_done_callback(
	struct	ddl_client_context_type *p_ddl, u32 b_frame_transact_end);
static void ddl_decoder_ouput_done_callback(
	struct ddl_client_context_type *p_ddl, u32 b_frame_transact_end);

static u32 ddl_get_frame_type
    (struct vcd_frame_data_type *p_frame, u32 n_frame_type);

static void ddl_getdec_profilelevel
(struct ddl_decoder_data_type   *p_decoder, u32 n_profile, u32 n_level);

static void ddl_dma_done_callback(struct ddl_context_type *p_ddl_context)
{
	if (!DDLCOMMAND_STATE_IS(p_ddl_context, DDL_CMD_DMA_INIT)) {
		VIDC_LOG_STRING("UNKWN_DMADONE");
		return;
	}
	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	VIDC_LOG_STRING("DMA_DONE");
	ddl_core_start_cpu(p_ddl_context);
}

static void ddl_cpu_started_callback(struct ddl_context_type *p_ddl_context)
{
	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	VIDC_LOG_STRING("CPU-STARTED");

	if (!vidc_720p_cpu_start()) {
		ddl_hw_fatal_cb(p_ddl_context);
		return;
	}

	vidc_720p_set_deblock_line_buffer(
			p_ddl_context->db_line_buffer.p_align_physical_addr,
			p_ddl_context->db_line_buffer.n_buffer_size);
	p_ddl_context->n_device_state = DDL_DEVICE_INITED;
	p_ddl_context->ddl_callback(VCD_EVT_RESP_DEVICE_INIT, VCD_S_SUCCESS,
			NULL, 0, NULL, p_ddl_context->p_client_data);
	DDL_IDLE(p_ddl_context);
}


static void ddl_eos_done_callback(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl = p_ddl_context->p_current_ddl;
	u32 n_displaystatus;

	if (!DDLCOMMAND_STATE_IS(p_ddl_context, DDL_CMD_EOS)) {
		VIDC_LOG_STRING("UNKWN_EOSDONE");
		ddl_client_fatal_cb(p_ddl_context);
		return;
	}

	if (!p_ddl ||
	    !p_ddl->b_decoding ||
	    !DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE)
	    ) {
		VIDC_LOG_STRING("STATE-CRITICAL-EOSDONE");
		ddl_client_fatal_cb(p_ddl_context);
		return;
	}
	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);

	vidc_720p_eos_info(&n_displaystatus);
	if ((enum vidc_720p_display_status_type)n_displaystatus
		!= VIDC_720P_EMPTY_BUFFER) {
		VIDC_LOG_STRING("EOSDONE-EMPTYBUF-ISSUE");
	}

	ddl_decode_dynamic_property(p_ddl, FALSE);
	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME);
	VIDC_LOG_STRING("EOS_DONE");
	p_ddl_context->ddl_callback(VCD_EVT_RESP_EOS_DONE, VCD_S_SUCCESS,
		NULL, 0, (u32 *) p_ddl, p_ddl_context->p_client_data);

	DDL_IDLE(p_ddl_context);
}

static u32 ddl_channel_set_callback(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl = p_ddl_context->p_current_ddl;
	u32 return_status = FALSE;

	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	VIDC_DEBUG_REGISTER_LOG;

	if (!p_ddl ||
	    !DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_CHDONE)
	    ) {
		VIDC_LOG_STRING("STATE-CRITICAL-CHSET");
		DDL_IDLE(p_ddl_context);
		return return_status;
	}
	VIDC_LOG_STRING("Channel-set");
	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODEC);

	if (p_ddl->b_decoding) {
		if (p_ddl->codec_data.decoder.b_header_in_start) {
			ddl_decode_init_codec(p_ddl);
		} else {
			p_ddl_context->ddl_callback(VCD_EVT_RESP_START,
					VCD_S_SUCCESS, NULL,
					    0, (u32 *) p_ddl,
						    p_ddl_context->
						    p_client_data);

			DDL_IDLE(p_ddl_context);
			return_status = TRUE;
		}
	} else {
		ddl_encode_init_codec(p_ddl);
	}
	return return_status;
}

static void ddl_init_codec_done_callback(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl = p_ddl_context->p_current_ddl;
	struct ddl_encoder_data_type *p_encoder;

	if (!p_ddl ||
	    p_ddl->b_decoding ||
	    !DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODECDONE)
	    ) {
		VIDC_LOG_STRING("STATE-CRITICAL-INITCODEC");
		ddl_client_fatal_cb(p_ddl_context);
		return;
	}
	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME);
	VIDC_LOG_STRING("INIT_CODEC_DONE");

	p_encoder = &p_ddl->codec_data.encoder;
	if (p_encoder->seq_header.p_virtual_base_addr) {
		vidc_720p_encode_get_header(&p_encoder->seq_header.
					     n_buffer_size);
	}

	p_ddl_context->ddl_callback(VCD_EVT_RESP_START, VCD_S_SUCCESS, NULL,
		0, (u32 *) p_ddl, p_ddl_context->p_client_data);

	DDL_IDLE(p_ddl_context);
}

static u32 ddl_header_done_callback(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl = p_ddl_context->p_current_ddl;
	struct ddl_decoder_data_type *p_decoder;
	struct vidc_720p_seq_hdr_info_type seq_hdr_info;
	u32 vcd_event = VCD_EVT_RESP_START;
	u32 vcd_status = VCD_S_SUCCESS;
	u32 b_req_cb = TRUE, bret = TRUE;

	if (!DDLCOMMAND_STATE_IS(p_ddl_context, DDL_CMD_HEADER_PARSE)) {
		VIDC_LOG_STRING("UNKWN_HEADERDONE");
		ddl_client_fatal_cb(p_ddl_context);
		return TRUE;
	}

	if (!p_ddl ||
	    !p_ddl->b_decoding ||
	    !DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODECDONE)
	    ) {
		VIDC_LOG_STRING("STATE-CRITICAL-HDDONE");
		ddl_client_fatal_cb(p_ddl_context);
		return TRUE;
	}
	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_DPB);
	VIDC_LOG_STRING("HEADER_DONE");
	VIDC_DEBUG_REGISTER_LOG;

	vidc_720p_decode_get_seq_hdr_info(&seq_hdr_info);

	p_decoder = &(p_ddl->codec_data.decoder);
	p_decoder->frame_size.n_width = seq_hdr_info.n_img_size_x;
	p_decoder->frame_size.n_height = seq_hdr_info.n_img_size_y;
	p_decoder->n_min_dpb_num = seq_hdr_info.n_min_num_dpb;
	p_decoder->n_y_cb_cr_size = seq_hdr_info.n_min_dpb_size;
	p_decoder->n_progressive_only = 1 - seq_hdr_info.n_progressive;
	ddl_getdec_profilelevel(p_decoder, seq_hdr_info.n_profile,
		seq_hdr_info.n_level);
	ddl_calculate_stride(&p_decoder->frame_size,
			!p_decoder->n_progressive_only);
	if (seq_hdr_info.n_crop_exists)	{
		p_decoder->frame_size.n_width -=
		(seq_hdr_info.n_crop_right_offset
		+ seq_hdr_info.n_crop_left_offset);
		p_decoder->frame_size.n_height -=
		(seq_hdr_info.n_crop_top_offset +
		seq_hdr_info.n_crop_bottom_offset);
	}
	ddl_set_default_decoder_buffer_req(p_decoder, FALSE);
	if (seq_hdr_info.n_data_partitioned == 0x1 &&
		p_decoder->codec_type.e_codec == VCD_CODEC_MPEG4 &&
		seq_hdr_info.n_img_size_x > DDL_MAX_DP_FRAME_WIDTH &&
		seq_hdr_info.n_img_size_y > DDL_MAX_DP_FRAME_HEIGHT)	{
		ddl_client_fatal_cb(p_ddl_context);
		return TRUE;
	}


	if (p_decoder->b_header_in_start) {
		p_decoder->client_frame_size = p_decoder->frame_size;
		p_decoder->client_output_buf_req =
			p_decoder->actual_output_buf_req;
		if ((p_decoder->frame_size.n_width *
			 p_decoder->frame_size.n_height) >= (800*480)) {
			if ((p_decoder->actual_output_buf_req.\
				 n_actual_count + 2) < 10)
				p_decoder->client_output_buf_req.\
				n_actual_count = 10;
			else
				p_decoder->client_output_buf_req.\
				n_actual_count += 2;
		} else
			p_decoder->client_output_buf_req.\
			n_actual_count =
			p_decoder->actual_output_buf_req.\
			n_actual_count + 5;

		p_decoder->client_input_buf_req =
		    p_decoder->actual_input_buf_req;
	} else {
		DBG("%s(): n_width = %d client_frame_size.n_width = %d\n",
		    __func__, p_decoder->frame_size.n_width,
		    p_decoder->client_frame_size.n_width);
		DBG("%s(): n_height = %d client_frame_size.n_height = %d\n",
		    __func__, p_decoder->frame_size.n_height,
		    p_decoder->client_frame_size.n_height);
		DBG("%s(): n_size = %d client_frame_size n_size = %d\n",
		    __func__, p_decoder->actual_output_buf_req.n_size,
		    p_decoder->client_output_buf_req.n_size);
		DBG("%s(): n_min_dpb_num = %d n_actual_count = %d\n", __func__,
		    p_decoder->n_min_dpb_num,
		    p_decoder->client_output_buf_req.n_actual_count);

		bret = FALSE;

		if (p_decoder->frame_size.n_width ==
		    p_decoder->client_frame_size.n_width
		    && p_decoder->frame_size.n_height ==
		    p_decoder->client_frame_size.n_height
		    && p_decoder->actual_output_buf_req.n_size <=
		    p_decoder->client_output_buf_req.n_size
		    && p_decoder->n_min_dpb_num <=
		    p_decoder->client_output_buf_req.n_actual_count) {
			vcd_status = ddl_decode_set_buffers(p_ddl);
			if (!vcd_status)
				b_req_cb = FALSE;
			else{
				ddl_client_fatal_cb(p_ddl_context);
				b_req_cb = TRUE;
			}
		} else {
			p_decoder->client_frame_size = p_decoder->frame_size;
			p_decoder->client_output_buf_req =
			    p_decoder->actual_output_buf_req;
			p_decoder->client_input_buf_req =
			    p_decoder->actual_input_buf_req;
			VIDC_LOGERR_STRING
			    ("ddlhdr_done_cb:Decode_reconfig_not_supported");
			vcd_event = VCD_EVT_IND_RECONFIG;
		}
	}

	if (b_req_cb) {
		p_ddl_context->ddl_callback(vcd_event, vcd_status,
			NULL, 0, (u32 *) p_ddl,	p_ddl_context->p_client_data);

		DDL_IDLE(p_ddl_context);
	}
	return bret;
}

static u32 ddl_dpb_buffers_set_done_callback(struct ddl_context_type
					      *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl = p_ddl_context->p_current_ddl;

	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	if (!p_ddl ||
	    !DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_DPBDONE)
	    ) {
		VIDC_LOG_STRING("STATE-CRITICAL-DPBDONE");
		ddl_client_fatal_cb(p_ddl_context);
		return TRUE;
	}
	VIDC_LOG_STRING("INTR_DPBDONE");
	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME);
	ddl_decode_frame_run(p_ddl);
	return FALSE;
}

static void ddl_encoder_frame_run_callback(struct ddl_context_type
					   *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl = p_ddl_context->p_current_ddl;
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);
	u32 b_eos_present = FALSE;

	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE)
	    ) {
		VIDC_LOG_STRING("STATE-CRITICAL-ENCFRMRUN");
		ddl_client_fatal_cb(p_ddl_context);
		return;
	}

	VIDC_LOG_STRING("ENC_FRM_RUN_DONE");

	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	vidc_720p_enc_frame_info(&p_encoder->enc_frame_info);

	p_ddl->output_frame.vcd_frm.n_ip_frm_tag =
	    p_ddl->input_frame.vcd_frm.n_ip_frm_tag;
	p_ddl->output_frame.vcd_frm.n_data_len =
	    p_encoder->enc_frame_info.n_enc_size;
	p_ddl->output_frame.vcd_frm.n_flags |= VCD_FRAME_FLAG_ENDOFFRAME;
	ddl_get_frame_type
	    (&(p_ddl->output_frame.vcd_frm),
	     p_encoder->enc_frame_info.n_frame_type);
	ddl_process_encoder_metadata(p_ddl);

	ddl_encode_dynamic_property(p_ddl, FALSE);

	p_ddl->input_frame.b_frm_trans_end = FALSE;
	p_ddl_context->ddl_callback(VCD_EVT_RESP_INPUT_DONE, VCD_S_SUCCESS,
		&(p_ddl->input_frame), sizeof(struct ddl_frame_data_type_tag),
		(u32 *) p_ddl, p_ddl_context->p_client_data);

#ifdef CORE_TIMING_INFO
	ddl_calc_core_time(1);
#endif
	/* check the presence of EOS */
   b_eos_present =
      ((VCD_FRAME_FLAG_EOS & p_ddl->input_frame.vcd_frm.n_flags));

	p_ddl->output_frame.b_frm_trans_end = !b_eos_present;
	p_ddl_context->ddl_callback(VCD_EVT_RESP_OUTPUT_DONE, VCD_S_SUCCESS,
		&(p_ddl->output_frame),	sizeof(struct ddl_frame_data_type_tag),
		(u32 *) p_ddl, p_ddl_context->p_client_data);

	if (b_eos_present) {
		VIDC_LOG_STRING("ENC-EOS_DONE");
		p_ddl_context->ddl_callback(VCD_EVT_RESP_EOS_DONE,
				VCD_S_SUCCESS, NULL, 0,	(u32 *)p_ddl,
				p_ddl_context->p_client_data);
	}

	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME);
	DDL_IDLE(p_ddl_context);
}

static u32 ddl_decoder_frame_run_callback(struct ddl_context_type
					   *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl = p_ddl_context->p_current_ddl;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vidc_720p_dec_disp_info_type *p_dec_disp_info =
	    &(p_decoder->dec_disp_info);
	u32 b_callback_end = FALSE;
	u32 status = TRUE, eos_present = FALSE;;

	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE)) {
		VIDC_LOG_STRING("STATE-CRITICAL-DECFRMRUN");
		ddl_client_fatal_cb(p_ddl_context);
		return TRUE;
	}

	VIDC_LOG_STRING("DEC_FRM_RUN_DONE");

	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);

	vidc_720p_decode_display_info(p_dec_disp_info);

	ddl_decode_dynamic_property(p_ddl, FALSE);

	if (p_dec_disp_info->n_resl_change) {
		VIDC_LOGERR_STRING
			("ddl_dec_frm_done:Dec_reconfig_no_tsupported");
	}

	if ((VCD_FRAME_FLAG_EOS & p_ddl->input_frame.vcd_frm.n_flags)) {
		b_callback_end = FALSE;
		eos_present = TRUE;
	}


	if (p_dec_disp_info->e_disp_status == VIDC_720P_DECODE_ONLY ||
		p_dec_disp_info->e_disp_status
			== VIDC_720P_DECODE_AND_DISPLAY) {
		if (!eos_present)
			b_callback_end = (p_dec_disp_info->e_disp_status
					== VIDC_720P_DECODE_ONLY);

	  ddl_decoder_input_done_callback(p_ddl, b_callback_end);
	}

	if (p_dec_disp_info->e_disp_status == VIDC_720P_DECODE_AND_DISPLAY
		|| p_dec_disp_info->e_disp_status == VIDC_720P_DISPLAY_ONLY) {
		if (!eos_present)
			b_callback_end =
			(p_dec_disp_info->e_disp_status
				== VIDC_720P_DECODE_AND_DISPLAY);

		ddl_decoder_ouput_done_callback(p_ddl, b_callback_end);
	}

	if (p_dec_disp_info->e_disp_status ==  VIDC_720P_DISPLAY_ONLY) {
		/* send the same input once again for decoding */
		ddl_decode_frame_run(p_ddl);
		/* client need to ignore the interrupt */
		status = FALSE;
	} else if (eos_present) {
		/* send EOS command to HW */
		ddl_decode_eos_run(p_ddl);
		/* client need to ignore the interrupt */
		status = FALSE;
	} else {
		ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME);
		/* move to Idle */
		DDL_IDLE(p_ddl_context);
	}
	return status;
}

static u32 ddl_eos_frame_done_callback(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl = p_ddl_context->p_current_ddl;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vidc_720p_dec_disp_info_type *p_dec_disp_info =
		&(p_decoder->dec_disp_info);

	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
		VIDC_LOGERR_STRING("STATE-CRITICAL-EOSFRMRUN");
		ddl_client_fatal_cb(p_ddl_context);
		return TRUE;
	}
	VIDC_LOG_STRING("EOS_FRM_RUN_DONE");

	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);

	vidc_720p_decode_display_info(p_dec_disp_info);

	ddl_decode_dynamic_property(p_ddl, FALSE);

	if (p_ddl_context->n_op_failed == 0x1)
		VIDC_LOGERR_STRING("ddl_eos_frm_done:OPFAILED!!");
	else if (p_dec_disp_info->n_resl_change)
		VIDC_LOGERR_STRING("ddl_eos_frm_done:Dec_reconfig!!");

	if (p_dec_disp_info->e_disp_status == VIDC_720P_DISPLAY_ONLY)
		ddl_decoder_ouput_done_callback(p_ddl, FALSE);
	else
		VIDC_LOG_STRING("STATE-CRITICAL-WRONG-DISP-STATUS");

	ddl_decoder_dpb_transact(p_decoder, NULL, DDL_DPB_OP_SET_MASK);
	ddl_move_command_state(p_ddl_context, DDL_CMD_EOS);
	vidc_720p_submit_command(p_ddl->n_channel_id,
		VIDC_720P_CMD_FRAMERUN);
	return FALSE;
}

static void ddl_channel_end_callback(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl;

	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	VIDC_LOG_STRING("CH_END_DONE");

	p_ddl = p_ddl_context->p_current_ddl;
	if (!p_ddl ||
	    !DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_CHEND)
	    ) {
		VIDC_LOG_STRING("STATE-CRITICAL-CHEND");
		DDL_IDLE(p_ddl_context);
		return;
	}

	ddl_release_client_internal_buffers(p_ddl);
	p_ddl_context->ddl_callback(VCD_EVT_RESP_STOP, VCD_S_SUCCESS,
		NULL, 0, (u32 *) p_ddl,	p_ddl_context->p_client_data);
	ddl_move_client_state(p_ddl, DDL_CLIENT_OPEN);
	DDL_IDLE(p_ddl_context);
}

static u32 ddl_operation_done_callback(struct ddl_context_type *p_ddl_context)
{
	u32 b_return_status = TRUE;

	switch (p_ddl_context->e_cmd_state) {
	case DDL_CMD_DECODE_FRAME:
		{
			b_return_status = ddl_decoder_frame_run_callback(
				p_ddl_context);
			break;
		}
	case DDL_CMD_ENCODE_FRAME:
		{
			ddl_encoder_frame_run_callback(p_ddl_context);
			break;
		}
	case DDL_CMD_CHANNEL_SET:
		{
			b_return_status = ddl_channel_set_callback(
				p_ddl_context);
			break;
		}
	case DDL_CMD_INIT_CODEC:
		{
			ddl_init_codec_done_callback(p_ddl_context);
			break;
		}
	case DDL_CMD_HEADER_PARSE:
		{
			b_return_status = ddl_header_done_callback(
				p_ddl_context);
			break;
		}
	case DDL_CMD_DECODE_SET_DPB:
		{
			b_return_status = ddl_dpb_buffers_set_done_callback(
				p_ddl_context);
			break;
		}
	case DDL_CMD_CHANNEL_END:
		{
			ddl_channel_end_callback(p_ddl_context);
			break;
		}
	case DDL_CMD_EOS:
		{
			b_return_status = ddl_eos_frame_done_callback(
				p_ddl_context);
			break;
		}
	case DDL_CMD_CPU_RESET:
		{
			ddl_cpu_started_callback(p_ddl_context);
			break;
		}
	default:
		{
			VIDC_LOG_STRING("UNKWN_OPDONE");
			b_return_status = FALSE;
			break;
		}
	}
	return b_return_status;
}

static u32 ddl_process_intr_status(struct ddl_context_type *p_ddl_context,
			u32 int_status)
{
	u32 b_status = TRUE;
	switch (int_status) {
	case VIDC_720P_INTR_FRAME_DONE:
		 {
			b_status = ddl_operation_done_callback(p_ddl_context);
			break;
		 }
	case VIDC_720P_INTR_DMA_DONE:
		 {
			ddl_dma_done_callback(p_ddl_context);
			b_status = FALSE;
			break;
		 }
	case VIDC_720P_INTR_FW_DONE:
		 {
			ddl_eos_done_callback(p_ddl_context);
			break;
		 }
	case VIDC_720P_INTR_BUFFER_FULL:
		 {
			VIDC_LOGERR_STRING("BUF_FULL_INTR");
			break;
		 }
	default:
		 {
			VIDC_LOGERR_STRING("UNKWN_INTR");
			break;
		 }
	}
	return b_status;
}

void ddl_read_and_clear_interrupt(void)
{
	struct ddl_context_type *p_ddl_context;

	p_ddl_context = ddl_get_context();
	if (!p_ddl_context->p_core_virtual_base_addr) {
		VIDC_LOGERR_STRING("SPURIOUS_INTERRUPT");
		return;
	}
	vidc_720p_get_interrupt_status(&p_ddl_context->intr_status,
		&p_ddl_context->n_cmd_err_status,
		&p_ddl_context->n_disp_pic_err_status,
		&p_ddl_context->n_op_failed
	);

	vidc_720p_interrupt_done_clear();

}

u32 ddl_process_core_response(void)
{
	struct ddl_context_type *p_ddl_context;
	u32 b_return_status = TRUE;

	p_ddl_context = ddl_get_context();
	if (!p_ddl_context->p_core_virtual_base_addr) {
		VIDC_LOGERR_STRING("UNKWN_INTR");
		return FALSE;
	}
	if (p_ddl_context->intr_status == DDL_INVALID_INTR_STATUS) {
		VIDC_LOGERR_STRING("INTERRUPT_NOT_READ");
		return FALSE;
	}

	if (!ddl_handle_core_errors(p_ddl_context)) {
		b_return_status = ddl_process_intr_status(p_ddl_context,
			p_ddl_context->intr_status);
	}

	if (p_ddl_context->pf_interrupt_clr)
		(*p_ddl_context->pf_interrupt_clr)();

	p_ddl_context->intr_status = DDL_INVALID_INTR_STATUS;
	return b_return_status;
}

static void ddl_decoder_input_done_callback(
	struct	ddl_client_context_type *p_ddl, u32 b_frame_transact_end)
{
	struct vidc_720p_dec_disp_info_type *p_dec_disp_info =
		&(p_ddl->codec_data.decoder.dec_disp_info);
	struct vcd_frame_data_type *p_input_vcd_frm =
		&(p_ddl->input_frame.vcd_frm);
	ddl_get_frame_type(p_input_vcd_frm, p_dec_disp_info->
		n_input_frame_type);

	p_input_vcd_frm->b_interlaced = (p_dec_disp_info->
		n_input_is_interlace);

	p_input_vcd_frm->n_offset += p_dec_disp_info->n_input_bytes_consumed;
	p_input_vcd_frm->n_data_len -= p_dec_disp_info->n_input_bytes_consumed;

	p_ddl->input_frame.b_frm_trans_end = b_frame_transact_end;
	p_ddl->p_ddl_context->ddl_callback(
		VCD_EVT_RESP_INPUT_DONE,
		VCD_S_SUCCESS,
		&p_ddl->input_frame,
		sizeof(struct ddl_frame_data_type_tag),
		(void *)p_ddl,
		p_ddl->p_ddl_context->p_client_data);
}

static void ddl_decoder_ouput_done_callback(
	struct ddl_client_context_type *p_ddl,
	u32 b_frame_transact_end)
{
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vidc_720p_dec_disp_info_type *p_dec_disp_info =
		&(p_decoder->dec_disp_info);
	struct ddl_frame_data_type_tag *p_output_frame =
		&p_ddl->output_frame;
	struct vcd_frame_data_type *p_output_vcd_frm =
		&(p_output_frame->vcd_frm);
	u32 vcd_status;
	u32 n_free_luma_dpb = 0;

	p_output_vcd_frm->p_physical = (u8 *)p_dec_disp_info->n_y_addr;

	if (p_decoder->codec_type.e_codec == VCD_CODEC_MPEG4 ||
		p_decoder->codec_type.e_codec == VCD_CODEC_VC1 ||
		p_decoder->codec_type.e_codec == VCD_CODEC_VC1_RCV){
		vidc_720p_decode_skip_frm_details(&n_free_luma_dpb);
		if (n_free_luma_dpb)
			p_output_vcd_frm->p_physical = (u8 *) n_free_luma_dpb;
	}


	vcd_status = ddl_decoder_dpb_transact(
			p_decoder,
			p_output_frame,
			DDL_DPB_OP_MARK_BUSY);


	p_output_vcd_frm->n_ip_frm_tag =  p_dec_disp_info->n_tag_top;
	if (p_dec_disp_info->n_crop_exists == 0x1) {
		p_output_vcd_frm->dec_op_prop.disp_frm.n_left =
			p_dec_disp_info->n_crop_left_offset;
		p_output_vcd_frm->dec_op_prop.disp_frm.n_top =
			p_dec_disp_info->n_crop_top_offset;
		p_output_vcd_frm->dec_op_prop.disp_frm.n_right =
			p_dec_disp_info->n_img_size_x -
			p_dec_disp_info->n_crop_right_offset;
		p_output_vcd_frm->dec_op_prop.disp_frm.n_bottom =
			p_dec_disp_info->n_img_size_y -
			p_dec_disp_info->n_crop_bottom_offset;
	} else {
		p_output_vcd_frm->dec_op_prop.disp_frm.n_left = 0;
		p_output_vcd_frm->dec_op_prop.disp_frm.n_top = 0;
		p_output_vcd_frm->dec_op_prop.disp_frm.n_right =
			p_dec_disp_info->n_img_size_x;
		p_output_vcd_frm->dec_op_prop.disp_frm.n_bottom =
			p_dec_disp_info->n_img_size_y;
	}
	if (!p_dec_disp_info->n_disp_is_interlace) {
		p_output_vcd_frm->b_interlaced = FALSE;
		p_output_frame->n_intrlcd_ip_frm_tag = VCD_FRAMETAG_INVALID;
	} else {
		p_output_vcd_frm->b_interlaced = TRUE;
		p_output_frame->n_intrlcd_ip_frm_tag =
			p_dec_disp_info->n_tag_bottom;
	}

	p_output_vcd_frm->n_offset = 0;
	p_output_vcd_frm->n_data_len = p_decoder->n_y_cb_cr_size;

	if (n_free_luma_dpb)
		p_output_vcd_frm->n_data_len = 0;

	p_output_vcd_frm->n_flags |= VCD_FRAME_FLAG_ENDOFFRAME;

	if (!vcd_status)
		ddl_process_decoder_metadata(p_ddl);
	p_output_frame->b_frm_trans_end = b_frame_transact_end;

#ifdef CORE_TIMING_INFO
	ddl_calc_core_time(0);
#endif

	p_ddl->p_ddl_context->ddl_callback(
		VCD_EVT_RESP_OUTPUT_DONE,
		vcd_status,
		p_output_frame,
		sizeof(struct ddl_frame_data_type_tag),
		(void *)p_ddl,
		p_ddl->p_ddl_context->p_client_data);
}

static u32 ddl_get_frame_type
    (struct vcd_frame_data_type *p_frame, u32 n_frame_type) {
	enum vidc_720p_frame_type e_frame_type =
	    (enum vidc_720p_frame_type)n_frame_type;
	u32 b_status = TRUE;

	switch (e_frame_type) {
	case VIDC_720P_IFRAME:
		{
			p_frame->n_flags |= VCD_FRAME_FLAG_SYNCFRAME;
			p_frame->e_frame_type = VCD_FRAME_I;
			break;
		}
	case VIDC_720P_PFRAME:
		{
			p_frame->e_frame_type = VCD_FRAME_P;
			break;
		}
	case VIDC_720P_BFRAME:
		{
			p_frame->e_frame_type = VCD_FRAME_B;
			break;
		}
	case VIDC_720P_NOTCODED:
		{
			p_frame->n_data_len = 0;
			break;
		}
	default:
		{
			VIDC_LOG_STRING("CRITICAL-FRAMETYPE");
			b_status = FALSE;
			break;
		}
	}
	return b_status;
}

static void ddl_getmpeg4_declevel(enum vcd_codec_level_type *p_level,
	u32 n_level)
{
	switch (n_level) {
	case VIDC_720P_MPEG4_LEVEL0:
		{
			*p_level = VCD_LEVEL_MPEG4_0;
			break;
		}
	case VIDC_720P_MPEG4_LEVEL0b:
		{
			*p_level = VCD_LEVEL_MPEG4_0b;
			break;
		}
	case VIDC_720P_MPEG4_LEVEL1:
		{
			*p_level = VCD_LEVEL_MPEG4_1;
			break;
		}
	case VIDC_720P_MPEG4_LEVEL2:
		{
			*p_level = VCD_LEVEL_MPEG4_2;
			break;
		}
	case VIDC_720P_MPEG4_LEVEL3:
		{
			*p_level = VCD_LEVEL_MPEG4_3;
			break;
		}
	case VIDC_720P_MPEG4_LEVEL3b:
		{
			*p_level = VCD_LEVEL_MPEG4_3b;
			break;
		}
	case VIDC_720P_MPEG4_LEVEL4a:
		{
			*p_level = VCD_LEVEL_MPEG4_4a;
			break;
		}
	case VIDC_720P_MPEG4_LEVEL5:
		{
			*p_level = VCD_LEVEL_MPEG4_5;
			break;
		}
	case VIDC_720P_MPEG4_LEVEL6:
		{
			*p_level = VCD_LEVEL_MPEG4_6;
			break;
		}
	}
}

static void ddl_geth264_declevel(enum vcd_codec_level_type *p_level,
	u32 n_level)
{
	switch (n_level) {
	case VIDC_720P_H264_LEVEL1:
		{
			*p_level = VCD_LEVEL_H264_1;
			break;
		}
	case VIDC_720P_H264_LEVEL1b:
		{
			*p_level = VCD_LEVEL_H264_1b;
			break;
		}
	case VIDC_720P_H264_LEVEL1p1:
		{
			*p_level = VCD_LEVEL_H264_1p1;
			break;
		}
	case VIDC_720P_H264_LEVEL1p2:
		{
			*p_level = VCD_LEVEL_H264_1p2;
			break;
		}
	case VIDC_720P_H264_LEVEL1p3:
		{
			*p_level = VCD_LEVEL_H264_1p3;
			break;
		}
	case VIDC_720P_H264_LEVEL2:
		{
			*p_level = VCD_LEVEL_H264_2;
			break;
		}
	case VIDC_720P_H264_LEVEL2p1:
		{
			*p_level = VCD_LEVEL_H264_2p1;
			break;
		}
	case VIDC_720P_H264_LEVEL2p2:
		{
			*p_level = VCD_LEVEL_H264_2p2;
			break;
		}
	case VIDC_720P_H264_LEVEL3:
		{
			*p_level = VCD_LEVEL_H264_3;
			break;
		}
	case VIDC_720P_H264_LEVEL3p1:
		{
			*p_level = VCD_LEVEL_H264_3p1;
			break;
		}
	case VIDC_720P_H264_LEVEL3p2:
	{
		*p_level = VCD_LEVEL_H264_3p2;
		break;
	}

	}
}

static void ddl_get_vc1_dec_level(
	enum vcd_codec_level_type *p_level, u32 level,
	enum vcd_codec_profile_type vc1_profile)
{
	if (vc1_profile == VCD_PROFILE_VC1_ADVANCE)	{
		switch (level) {
		case VIDC_720P_VC1_LEVEL0:
			{
				*p_level = VCD_LEVEL_VC1_0;
				break;
			}
		case VIDC_720P_VC1_LEVEL1:
			{
				*p_level = VCD_LEVEL_VC1_1;
				break;
			}
		case VIDC_720P_VC1_LEVEL2:
			{
				*p_level = VCD_LEVEL_VC1_2;
				break;
			}
		case VIDC_720P_VC1_LEVEL3:
			{
				*p_level = VCD_LEVEL_VC1_3;
				break;
			}
		case VIDC_720P_VC1_LEVEL4:
			{
				*p_level = VCD_LEVEL_VC1_4;
				break;
			}
		}
		return;
	}

	/* now determine the Main and Simple profile level */
	switch (level) {
	case VIDC_720P_VC1_LEVEL_LOW:
		{
			*p_level = VCD_LEVEL_VC1_LOW;
			break;
		}
	case VIDC_720P_VC1_LEVEL_MED:
		{
			*p_level = VCD_LEVEL_VC1_MEDIUM;
			break;
		}
	case VIDC_720P_VC1_LEVEL_HIGH:
		{
			*p_level = VCD_LEVEL_VC1_HIGH;
			break;
		}
	}
}

static void ddl_get_mpeg2_dec_level(enum vcd_codec_level_type *p_level,
								 u32 level)
{
	switch (level) {
	case VIDCL_720P_MPEG2_LEVEL_LOW:
		{
			*p_level = VCD_LEVEL_MPEG2_LOW;
			break;
		}
	case VIDCL_720P_MPEG2_LEVEL_MAIN:
		{
			*p_level = VCD_LEVEL_MPEG2_MAIN;
			break;
		}
	case VIDCL_720P_MPEG2_LEVEL_HIGH14:
		{
			*p_level = VCD_LEVEL_MPEG2_HIGH_14;
			break;
		}
	}
}

static void ddl_getdec_profilelevel(struct ddl_decoder_data_type *p_decoder,
		u32 n_profile, u32 n_level)
{
	enum vcd_codec_profile_type profile = VCD_PROFILE_UNKNOWN;
	enum vcd_codec_level_type level = VCD_LEVEL_UNKNOWN;

	switch (p_decoder->codec_type.e_codec) {
	case VCD_CODEC_MPEG4:
		{
			if (n_profile == VIDC_720P_PROFILE_MPEG4_SP)
				profile = VCD_PROFILE_MPEG4_SP;
			else if (n_profile == VIDC_720P_PROFILE_MPEG4_ASP)
				profile = VCD_PROFILE_MPEG4_ASP;

			ddl_getmpeg4_declevel(&level, n_level);
			break;
		}
	case VCD_CODEC_H264:
		{
			if (n_profile == VIDC_720P_PROFILE_H264_BASELINE)
				profile = VCD_PROFILE_H264_BASELINE;
			else if (n_profile == VIDC_720P_PROFILE_H264_MAIN)
				profile = VCD_PROFILE_H264_MAIN;
			else if (n_profile == VIDC_720P_PROFILE_H264_HIGH)
				profile = VCD_PROFILE_H264_HIGH;
			ddl_geth264_declevel(&level, n_level);
			break;
		}
	default:
	case VCD_CODEC_H263:
		{
			break;
		}
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		{
			if (n_profile == VIDC_720P_PROFILE_VC1_SP)
				profile = VCD_PROFILE_VC1_SIMPLE;
			else if (n_profile == VIDC_720P_PROFILE_VC1_MAIN)
				profile = VCD_PROFILE_VC1_MAIN;
			else if (n_profile == VIDC_720P_PROFILE_VC1_ADV)
				profile = VCD_PROFILE_VC1_ADVANCE;
			ddl_get_vc1_dec_level(&level, n_level, profile);
			break;
		}
	case VCD_CODEC_MPEG2:
		{
			if (n_profile == VIDC_720P_PROFILE_MPEG2_MAIN)
				profile = VCD_PROFILE_MPEG2_MAIN;
			else if (n_profile == VIDC_720P_PROFILE_MPEG2_SP)
				profile = VCD_PROFILE_MPEG2_SIMPLE;
			ddl_get_mpeg2_dec_level(&level, n_level);
			break;
		}
	}

	p_decoder->profile.e_profile = profile;
	p_decoder->level.e_level = level;
}
