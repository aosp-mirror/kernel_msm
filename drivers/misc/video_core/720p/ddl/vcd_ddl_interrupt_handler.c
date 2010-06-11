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

static void ddl_decoder_input_done_callback(struct ddl_client_context *ddl,
		u32 frame_transact_end);
static void ddl_decoder_ouput_done_callback(struct ddl_client_context *ddl,
		u32 frame_transact_end);

static u32 ddl_get_frame_type(struct vcd_frame_data *frame, u32 frame_type);

static void ddl_getdec_profilelevel(struct ddl_decoder_data *dec,
		u32 profile, u32 level);

static void ddl_dma_done_callback(struct ddl_context *ddl_context)
{
	if (!DDLCOMMAND_STATE_IS(ddl_context, DDL_CMD_DMA_INIT)) {
		pr_debug("UNKNOWN_DMADONE\n");
		return;
	}
	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);
	pr_debug("DMA_DONE");
	ddl_core_start_cpu(ddl_context);
}

static void ddl_cpu_started_callback(struct ddl_context *ddl_context)
{
	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);
	pr_debug("CPU-STARTED");

	if (!vidc_720p_cpu_start()) {
		ddl_hw_fatal_cb(ddl_context);
		return;
	}

	vidc_720p_set_deblock_line_buffer(ddl_context->db_line_buffer.phys_addr,
			ddl_context->db_line_buffer.size);
	ddl_context->device_state = DDL_DEVICE_INITED;
	ddl_context->ddl_callback(VCD_EVT_RESP_DEVICE_INIT, VCD_S_SUCCESS,
			NULL, 0, NULL, ddl_context->client_data);
	DDL_IDLE(ddl_context);
}


static void ddl_eos_done_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl = ddl_context->current_ddl;
	u32 displaystatus;

	if (!DDLCOMMAND_STATE_IS(ddl_context, DDL_CMD_EOS)) {
		pr_debug("UNKWN_EOSDONE");
		ddl_client_fatal_cb(ddl_context);
		return;
	}

	if (!ddl || !ddl->decoding || !DDLCLIENT_STATE_IS(ddl,
			DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
		pr_debug("STATE-CRITICAL-EOSDONE");
		ddl_client_fatal_cb(ddl_context);
		return;
	}
	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);

	vidc_720p_eos_info(&displaystatus);
	if ((enum vidc_720p_display_status_type) displaystatus
		!= VIDC_720P_EMPTY_BUFFER) {
		pr_debug("EOSDONE-EMPTYBUF-ISSUE");
	}

	ddl_decode_dynamic_property(ddl, false);
	ddl_move_client_state(ddl, DDL_CLIENT_WAIT_FOR_FRAME);
	pr_debug("EOS_DONE");
	ddl_context->ddl_callback(VCD_EVT_RESP_EOS_DONE, VCD_S_SUCCESS, NULL,
			0, (u32 *) ddl, ddl_context->client_data);

	DDL_IDLE(ddl_context);
}

static u32 ddl_channel_set_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl = ddl_context->current_ddl;
	u32 return_status = false;

	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);

	if (!ddl || !DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_CHDONE)) {
		pr_debug("STATE-CRITICAL-CHSET");
		DDL_IDLE(ddl_context);
		return return_status;
	}
	pr_debug("Channel-set");
	ddl_move_client_state(ddl, DDL_CLIENT_WAIT_FOR_INITCODEC);

	if (ddl->decoding) {
		if (ddl->codec_data.decoder.header_in_start) {
			ddl_decode_init_codec(ddl);
		} else {
			ddl_context->ddl_callback(VCD_EVT_RESP_START,
					VCD_S_SUCCESS, NULL, 0, (u32 *) ddl,
					ddl_context->client_data);
			DDL_IDLE(ddl_context);
			return_status = true;
		}
	} else {
		ddl_encode_init_codec(ddl);
	}
	return return_status;
}

static void ddl_init_codec_done_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl = ddl_context->current_ddl;
	struct ddl_encoder_data *encoder;

	if (!ddl || ddl->decoding || !DDLCLIENT_STATE_IS(ddl,
			DDL_CLIENT_WAIT_FOR_INITCODECDONE)) {
		pr_debug("STATE-CRITICAL-INITCODEC");
		ddl_client_fatal_cb(ddl_context);
		return;
	}
	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);
	ddl_move_client_state(ddl, DDL_CLIENT_WAIT_FOR_FRAME);
	pr_debug("INIT_CODEC_DONE");

	encoder = &ddl->codec_data.encoder;
	if (encoder->seq_header.virt_addr)
		vidc_720p_encode_get_header(&encoder->seq_header.size);

	ddl_context->ddl_callback(VCD_EVT_RESP_START, VCD_S_SUCCESS, NULL, 0,
			(u32 *) ddl, ddl_context->client_data);

	DDL_IDLE(ddl_context);
}

static u32 ddl_header_done_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl = ddl_context->current_ddl;
	struct ddl_decoder_data *dec;
	struct vidc_720p_seq_hdr_info_type seq_hdr_info;
	u32 vcd_event = VCD_EVT_RESP_START;
	u32 vcd_status = VCD_S_SUCCESS;
	u32 req_cb = true, bret = true;

	if (!DDLCOMMAND_STATE_IS(ddl_context, DDL_CMD_HEADER_PARSE)) {
		pr_debug("UNKWN_HEADERDONE");
		ddl_client_fatal_cb(ddl_context);
		return true;
	}

	if (!ddl || !ddl->decoding || !DDLCLIENT_STATE_IS(ddl,
			DDL_CLIENT_WAIT_FOR_INITCODECDONE)) {
		pr_debug("STATE-CRITICAL-HDDONE");
		ddl_client_fatal_cb(ddl_context);
		return true;
	}
	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);
	ddl_move_client_state(ddl, DDL_CLIENT_WAIT_FOR_DPB);
	pr_debug("HEADER_DONE");

	vidc_720p_decode_get_seq_hdr_info(&seq_hdr_info);

	dec = &(ddl->codec_data.decoder);
	dec->frame_size.width = seq_hdr_info.img_size_x;
	dec->frame_size.height = seq_hdr_info.img_size_y;
	dec->min_dpb_num = seq_hdr_info.min_num_dpb;
	dec->y_cb_cr_size = seq_hdr_info.min_dpb_size;
	dec->progressive_only = 1 - seq_hdr_info.progressive;
	ddl_getdec_profilelevel(dec, seq_hdr_info.profile, seq_hdr_info.level);
	ddl_calculate_stride(&dec->frame_size, !dec->progressive_only);
	if (seq_hdr_info.crop_exists) {
		dec->frame_size.width -= (seq_hdr_info.crop_right_offset +
				seq_hdr_info.crop_left_offset);
		dec->frame_size.height -= (seq_hdr_info.crop_top_offset +
				seq_hdr_info.crop_bottom_offset);
	}
	ddl_set_default_decoder_buffer_req(dec, false);
	if (seq_hdr_info.data_partitioned == 0x1 &&
			dec->codec_type.codec == VCD_CODEC_MPEG4 &&
			seq_hdr_info.img_size_x > DDL_MAX_DP_FRAME_WIDTH &&
			seq_hdr_info.img_size_y > DDL_MAX_DP_FRAME_HEIGHT) {
		ddl_client_fatal_cb(ddl_context);
		return true;
	}


	if (dec->header_in_start) {
		dec->client_frame_size = dec->frame_size;
		dec->client_output_buf_req = dec->actual_output_buf_req;
		if ((dec->frame_size.width * dec->frame_size.height) >=
				(800*480)) {
			if ((dec->actual_output_buf_req.actual_count + 2) < 10)
				dec->client_output_buf_req.actual_count = 10;
			else
				dec->client_output_buf_req.actual_count += 2;
		} else
			dec->client_output_buf_req.actual_count =
				dec->actual_output_buf_req.actual_count + 5;

		dec->client_input_buf_req = dec->actual_input_buf_req;
	} else {
		DBG("%s(): width = %d client_frame_size.width = %d\n",
			__func__, dec->frame_size.width,
			dec->client_frame_size.width);
		DBG("%s(): height = %d client_frame_size.height = %d\n",
			__func__, dec->frame_size.height,
			dec->client_frame_size.height);
		DBG("%s(): size = %d client_frame_size size = %d\n",
			__func__, dec->actual_output_buf_req.size,
			dec->client_output_buf_req.size);
		DBG("%s(): min_dpb_num = %d actual_count = %d\n", __func__,
			dec->min_dpb_num,
			dec->client_output_buf_req.actual_count);

		bret = false;

		if (dec->frame_size.width == dec->client_frame_size.width &&
				dec->frame_size.height ==
				dec->client_frame_size.height &&
				dec->actual_output_buf_req.size <=
				dec->client_output_buf_req.size &&
				dec->min_dpb_num <=
				dec->client_output_buf_req.actual_count) {
			vcd_status = ddl_decode_set_buffers(ddl);
			if (!vcd_status) {
				req_cb = false;
			} else {
				ddl_client_fatal_cb(ddl_context);
				req_cb = true;
			}
		} else {
			dec->client_frame_size = dec->frame_size;
			dec->client_output_buf_req = dec->actual_output_buf_req;
			dec->client_input_buf_req = dec->actual_input_buf_req;
			pr_err("%s:Decode_reconfig_not_supported\n", __func__);
			vcd_event = VCD_EVT_IND_RECONFIG;
		}
	}

	if (req_cb) {
		ddl_context->ddl_callback(vcd_event, vcd_status, NULL, 0,
				(u32 *) ddl, ddl_context->client_data);
		DDL_IDLE(ddl_context);
	}
	return bret;
}

static u32 ddl_dpb_buffers_set_done_callback(struct ddl_context
					      *ddl_context)
{
	struct ddl_client_context *ddl = ddl_context->current_ddl;

	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);
	if (!ddl || !DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_DPBDONE)) {
		pr_debug("STATE-CRITICAL-DPBDONE\n");
		ddl_client_fatal_cb(ddl_context);
		return true;
	}
	pr_debug("INTR_DPBDONE\n");
	ddl_move_client_state(ddl, DDL_CLIENT_WAIT_FOR_FRAME);
	ddl_decode_frame_run(ddl);
	return false;
}

static void ddl_encoder_frame_run_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl = ddl_context->current_ddl;
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);
	u32 eos_present = false;

	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE)) {
		pr_debug("STATE-CRITICAL-ENCFRMRUN\n");
		ddl_client_fatal_cb(ddl_context);
		return;
	}

	pr_debug("ENC_FRM_RUN_DONE\n");

	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);
	vidc_720p_enc_frame_info(&encoder->enc_frame_info);

	ddl->output_frame.vcd_frm.ip_frm_tag =
		ddl->input_frame.vcd_frm.ip_frm_tag;
	ddl->output_frame.vcd_frm.data_len =
		encoder->enc_frame_info.enc_size;
	ddl->output_frame.vcd_frm.flags |= VCD_FRAME_FLAG_ENDOFFRAME;
	ddl_get_frame_type(&(ddl->output_frame.vcd_frm),
		encoder->enc_frame_info.frame_type);
	ddl_process_encoder_metadata(ddl);

	ddl_encode_dynamic_property(ddl, false);

	ddl->input_frame.frm_trans_end = false;
	ddl_context->ddl_callback(VCD_EVT_RESP_INPUT_DONE, VCD_S_SUCCESS,
		&(ddl->input_frame), sizeof(struct ddl_frame_data_tag),
		(u32 *) ddl, ddl_context->client_data);

#ifdef CORE_TIMING_INFO
	ddl_calc_core_time(1);
#endif
	/* check the presence of EOS */
	eos_present = VCD_FRAME_FLAG_EOS & ddl->input_frame.vcd_frm.flags;

	ddl->output_frame.frm_trans_end = !eos_present;
	ddl_context->ddl_callback(VCD_EVT_RESP_OUTPUT_DONE, VCD_S_SUCCESS,
		&(ddl->output_frame), sizeof(struct ddl_frame_data_tag),
		(u32 *) ddl, ddl_context->client_data);

	if (eos_present) {
		pr_debug("ENC-EOS_DONE\n");
		ddl_context->ddl_callback(VCD_EVT_RESP_EOS_DONE,
				VCD_S_SUCCESS, NULL, 0,	(u32 *)ddl,
				ddl_context->client_data);
	}

	ddl_move_client_state(ddl, DDL_CLIENT_WAIT_FOR_FRAME);
	DDL_IDLE(ddl_context);
}

static u32 ddl_decoder_frame_run_callback(struct ddl_context
					   *ddl_context)
{
	struct ddl_client_context *ddl = ddl_context->current_ddl;
	struct ddl_decoder_data *dec = &(ddl->codec_data.decoder);
	struct vidc_720p_dec_disp_info *dec_disp_info = &(dec->dec_disp_info);
	u32 callback_end = false;
	u32 status = true, eos_present = false;;

	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE)) {
		pr_debug("STATE-CRITICAL-DECFRMRUN\n");
		ddl_client_fatal_cb(ddl_context);
		return true;
	}

	pr_debug("DEC_FRM_RUN_DONE\n");

	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);

	vidc_720p_decode_display_info(dec_disp_info);

	ddl_decode_dynamic_property(ddl, false);

	if (dec_disp_info->resl_change) {
		pr_err("ddl_dec_frm_done:"
			"Dec_reconfig_no_tsupported\n");
	}

	if ((VCD_FRAME_FLAG_EOS & ddl->input_frame.vcd_frm.flags)) {
		callback_end = false;
		eos_present = true;
	}

	if (dec_disp_info->disp_status == VIDC_720P_DECODE_ONLY ||
			dec_disp_info->disp_status ==
			VIDC_720P_DECODE_AND_DISPLAY) {
		if (!eos_present)
			callback_end = (dec_disp_info->disp_status ==
				VIDC_720P_DECODE_ONLY);

		ddl_decoder_input_done_callback(ddl, callback_end);
	}

	if (dec_disp_info->disp_status == VIDC_720P_DECODE_AND_DISPLAY ||
			dec_disp_info->disp_status == VIDC_720P_DISPLAY_ONLY) {
		if (!eos_present)
			callback_end = (dec_disp_info->disp_status ==
				VIDC_720P_DECODE_AND_DISPLAY);

		ddl_decoder_ouput_done_callback(ddl, callback_end);
	}

	if (dec_disp_info->disp_status ==  VIDC_720P_DISPLAY_ONLY) {
		/* send the same input once again for decoding */
		ddl_decode_frame_run(ddl);
		/* client need to ignore the interrupt */
		status = false;
	} else if (eos_present) {
		/* send EOS command to HW */
		ddl_decode_eos_run(ddl);
		/* client need to ignore the interrupt */
		status = false;
	} else {
		ddl_move_client_state(ddl, DDL_CLIENT_WAIT_FOR_FRAME);
		/* move to Idle */
		DDL_IDLE(ddl_context);
	}
	return status;
}

static u32 ddl_eos_frame_done_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl = ddl_context->current_ddl;
	struct ddl_decoder_data *dec = &(ddl->codec_data.decoder);
	struct vidc_720p_dec_disp_info *dec_disp_info =
		&(dec->dec_disp_info);

	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
		pr_err("STATE-CRITICAL-EOSFRMRUN\n");
		ddl_client_fatal_cb(ddl_context);
		return true;
	}
	pr_debug("EOS_FRM_RUN_DONE\n");

	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);

	vidc_720p_decode_display_info(dec_disp_info);

	ddl_decode_dynamic_property(ddl, false);

	if (ddl_context->op_failed == 0x1)
		pr_err("ddl_eos_frm_done:OPFAILED!!\n");
	else if (dec_disp_info->resl_change)
		pr_err("ddl_eos_frm_done:Dec_reconfig!!\n");

	if (dec_disp_info->disp_status == VIDC_720P_DISPLAY_ONLY)
		ddl_decoder_ouput_done_callback(ddl, false);
	else
		pr_debug("STATE-CRITICAL-WRONG-DISP-STATUS\n");

	ddl_decoder_dpb_transact(dec, NULL, DDL_DPB_OP_SET_MASK);
	ddl_move_command_state(ddl_context, DDL_CMD_EOS);
	vidc_720p_submit_command(ddl->channel_id, VIDC_720P_CMD_FRAMERUN);
	return false;
}

static void ddl_channel_end_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl;

	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);
	pr_debug("CH_END_DONE\n");

	ddl = ddl_context->current_ddl;
	if (!ddl || !DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_CHEND)) {
		pr_debug("STATE-CRITICAL-CHEND\n");
		DDL_IDLE(ddl_context);
		return;
	}

	ddl_release_client_internal_buffers(ddl);
	ddl_context->ddl_callback(VCD_EVT_RESP_STOP, VCD_S_SUCCESS, NULL, 0,
		(u32 *) ddl, ddl_context->client_data);
	ddl_move_client_state(ddl, DDL_CLIENT_OPEN);
	DDL_IDLE(ddl_context);
}

static u32 ddl_operation_done_callback(struct ddl_context *ddl_context)
{
	u32 return_status = true;

	switch (ddl_context->cmd_state) {
	case DDL_CMD_DECODE_FRAME:
		return_status = ddl_decoder_frame_run_callback(ddl_context);
		break;
	case DDL_CMD_ENCODE_FRAME:
		ddl_encoder_frame_run_callback(ddl_context);
		break;
	case DDL_CMD_CHANNEL_SET:
		return_status = ddl_channel_set_callback(ddl_context);
		break;
	case DDL_CMD_INIT_CODEC:
		ddl_init_codec_done_callback(ddl_context);
		break;
	case DDL_CMD_HEADER_PARSE:
		return_status = ddl_header_done_callback(ddl_context);
		break;
	case DDL_CMD_DECODE_SET_DPB:
		return_status = ddl_dpb_buffers_set_done_callback(ddl_context);
		break;
	case DDL_CMD_CHANNEL_END:
		ddl_channel_end_callback(ddl_context);
		break;
	case DDL_CMD_EOS:
		return_status = ddl_eos_frame_done_callback(ddl_context);
		break;
	case DDL_CMD_CPU_RESET:
		ddl_cpu_started_callback(ddl_context);
		break;
	default:
		pr_debug("UNKWN_OPDONE\n");
		return_status = false;
		break;
	}
	return return_status;
}

static u32 ddl_process_intr_status(struct ddl_context *ddl_context,
			u32 int_status)
{
	u32 status = true;
	switch (int_status) {
	case VIDC_720P_INTR_FRAME_DONE:
		status = ddl_operation_done_callback(ddl_context);
		break;
	case VIDC_720P_INTR_DMA_DONE:
		ddl_dma_done_callback(ddl_context);
		status = false;
		break;
	case VIDC_720P_INTR_FW_DONE:
		ddl_eos_done_callback(ddl_context);
		break;
	case VIDC_720P_INTR_BUFFER_FULL:
		pr_err("BUF_FULL_INTR\n");
		break;
	default:
		pr_err("UNKWN_INTR\n");
		break;
	}
	return status;
}

void ddl_read_and_clear_interrupt(void)
{
	struct ddl_context *ddl_context;

	ddl_context = ddl_get_context();
	if (!ddl_context->core_virtual_base_addr) {
		pr_err("SPURIOUS_INTERRUPT\n");
		return;
	}
	vidc_720p_get_interrupt_status(&ddl_context->intr_status,
		&ddl_context->cmd_err_status,
		&ddl_context->disp_pic_err_status,
		&ddl_context->op_failed);

	vidc_720p_interrupt_done_clear();
}

u32 ddl_process_core_response(void)
{
	struct ddl_context *ddl_context;
	u32 return_status = true;

	ddl_context = ddl_get_context();
	if (!ddl_context->core_virtual_base_addr) {
		pr_err("UNKWN_INTR\n");
		return false;
	}
	if (ddl_context->intr_status == DDL_INVALID_INTR_STATUS) {
		pr_err("INTERRUPT_NOT_READ\n");
		return false;
	}

	if (!ddl_handle_core_errors(ddl_context)) {
		return_status = ddl_process_intr_status(ddl_context,
			ddl_context->intr_status);
	}

	if (ddl_context->pf_interrupt_clr)
		(*ddl_context->pf_interrupt_clr)();

	ddl_context->intr_status = DDL_INVALID_INTR_STATUS;
	return return_status;
}

static void ddl_decoder_input_done_callback(
	struct	ddl_client_context *ddl, u32 frame_transact_end)
{
	struct vidc_720p_dec_disp_info *dec_disp_info =
		&(ddl->codec_data.decoder.dec_disp_info);
	struct vcd_frame_data *input_vcd_frm = &(ddl->input_frame.vcd_frm);
	ddl_get_frame_type(input_vcd_frm, dec_disp_info->input_frame_type);

	input_vcd_frm->interlaced = (dec_disp_info->input_is_interlace);

	input_vcd_frm->offset += dec_disp_info->input_bytes_consumed;
	input_vcd_frm->data_len -= dec_disp_info->input_bytes_consumed;

	ddl->input_frame.frm_trans_end = frame_transact_end;
	ddl->ddl_context->ddl_callback(VCD_EVT_RESP_INPUT_DONE,
		VCD_S_SUCCESS,
		&ddl->input_frame,
		sizeof(struct ddl_frame_data_tag),
		(void *)ddl,
		ddl->ddl_context->client_data);
}

static void ddl_decoder_ouput_done_callback(struct ddl_client_context *ddl,
	u32 frame_transact_end)
{
	struct ddl_decoder_data *dec = &ddl->codec_data.decoder;
	struct vidc_720p_dec_disp_info *dec_disp_info =	&dec->dec_disp_info;
	struct ddl_frame_data_tag *output_frame = &ddl->output_frame;
	struct vcd_frame_data *output_vcd_frm = &output_frame->vcd_frm;
	u32 vcd_status;
	phys_addr_t free_luma_dpb = 0;

	output_vcd_frm->phys_addr = dec_disp_info->y_addr;

	if (dec->codec_type.codec == VCD_CODEC_MPEG4 ||
			dec->codec_type.codec == VCD_CODEC_VC1 ||
			dec->codec_type.codec == VCD_CODEC_VC1_RCV) {
		vidc_720p_decode_skip_frm_details(&free_luma_dpb);
		if (free_luma_dpb)
			output_vcd_frm->phys_addr = free_luma_dpb;
	}

	vcd_status = ddl_decoder_dpb_transact(dec, output_frame,
		DDL_DPB_OP_MARK_BUSY);

	output_vcd_frm->ip_frm_tag =  dec_disp_info->tag_top;
	if (dec_disp_info->crop_exists == 0x1) {
		output_vcd_frm->dec_op_prop.disp_frm.left =
			dec_disp_info->crop_left_offset;
		output_vcd_frm->dec_op_prop.disp_frm.top =
			dec_disp_info->crop_top_offset;
		output_vcd_frm->dec_op_prop.disp_frm.right =
			dec_disp_info->img_size_x -
			dec_disp_info->crop_right_offset;
		output_vcd_frm->dec_op_prop.disp_frm.bottom =
			dec_disp_info->img_size_y -
			dec_disp_info->crop_bottom_offset;
	} else {
		output_vcd_frm->dec_op_prop.disp_frm.left = 0;
		output_vcd_frm->dec_op_prop.disp_frm.top = 0;
		output_vcd_frm->dec_op_prop.disp_frm.right =
			dec_disp_info->img_size_x;
		output_vcd_frm->dec_op_prop.disp_frm.bottom =
			dec_disp_info->img_size_y;
	}
	if (!dec_disp_info->disp_is_interlace) {
		output_vcd_frm->interlaced = false;
		output_frame->intrlcd_ip_frm_tag = VCD_FRAMETAG_INVALID;
	} else {
		output_vcd_frm->interlaced = true;
		output_frame->intrlcd_ip_frm_tag = dec_disp_info->tag_bottom;
	}

	output_vcd_frm->offset = 0;
	output_vcd_frm->data_len = dec->y_cb_cr_size;

	if (free_luma_dpb)
		output_vcd_frm->data_len = 0;

	output_vcd_frm->flags |= VCD_FRAME_FLAG_ENDOFFRAME;

	if (!vcd_status)
		ddl_process_decoder_metadata(ddl);
	output_frame->frm_trans_end = frame_transact_end;

#ifdef CORE_TIMING_INFO
	ddl_calc_core_time(0);
#endif

	ddl->ddl_context->ddl_callback(VCD_EVT_RESP_OUTPUT_DONE,
		vcd_status, output_frame, sizeof(struct ddl_frame_data_tag),
		(void *)ddl, ddl->ddl_context->client_data);
}

static u32 ddl_get_frame_type(struct vcd_frame_data *frame, u32 frame_type)
{
	enum vidc_720p_frame_type e_frame_type =
		(enum vidc_720p_frame_type)frame_type;
	u32 status = true;

	switch (e_frame_type) {
	case VIDC_720P_IFRAME:
		frame->flags |= VCD_FRAME_FLAG_SYNCFRAME;
		frame->frame_type = VCD_FRAME_I;
		break;
	case VIDC_720P_PFRAME:
		frame->frame_type = VCD_FRAME_P;
		break;
	case VIDC_720P_BFRAME:
		frame->frame_type = VCD_FRAME_B;
		break;
	case VIDC_720P_NOTCODED:
		frame->data_len = 0;
		break;
	default:
		pr_debug("CRITICAL-FRAMETYPE\n");
		status = false;
		break;
	}
	return status;
}

static void ddl_getmpeg4_declevel(enum vcd_codec_level_type *vcd_level,
	u32 level)
{
	switch (level) {
	case VIDC_720P_MPEG4_LEVEL0:
		*vcd_level = VCD_LEVEL_MPEG4_0;
		break;
	case VIDC_720P_MPEG4_LEVEL0b:
		*vcd_level = VCD_LEVEL_MPEG4_0b;
		break;
	case VIDC_720P_MPEG4_LEVEL1:
		*vcd_level = VCD_LEVEL_MPEG4_1;
		break;
	case VIDC_720P_MPEG4_LEVEL2:
		*vcd_level = VCD_LEVEL_MPEG4_2;
		break;
	case VIDC_720P_MPEG4_LEVEL3:
		*vcd_level = VCD_LEVEL_MPEG4_3;
		break;
	case VIDC_720P_MPEG4_LEVEL3b:
		*vcd_level = VCD_LEVEL_MPEG4_3b;
		break;
	case VIDC_720P_MPEG4_LEVEL4a:
		*vcd_level = VCD_LEVEL_MPEG4_4a;
		break;
	case VIDC_720P_MPEG4_LEVEL5:
		*vcd_level = VCD_LEVEL_MPEG4_5;
		break;
	case VIDC_720P_MPEG4_LEVEL6:
		*vcd_level = VCD_LEVEL_MPEG4_6;
		break;
	}
}

static void ddl_geth264_declevel(enum vcd_codec_level_type *vcd_level,
		u32 level)
{
	switch (level) {
	case VIDC_720P_H264_LEVEL1:
		*vcd_level = VCD_LEVEL_H264_1;
		break;
	case VIDC_720P_H264_LEVEL1b:
		*vcd_level = VCD_LEVEL_H264_1b;
		break;
	case VIDC_720P_H264_LEVEL1p1:
		*vcd_level = VCD_LEVEL_H264_1p1;
		break;
	case VIDC_720P_H264_LEVEL1p2:
		*vcd_level = VCD_LEVEL_H264_1p2;
		break;
	case VIDC_720P_H264_LEVEL1p3:
		*vcd_level = VCD_LEVEL_H264_1p3;
		break;
	case VIDC_720P_H264_LEVEL2:
		*vcd_level = VCD_LEVEL_H264_2;
		break;
	case VIDC_720P_H264_LEVEL2p1:
		*vcd_level = VCD_LEVEL_H264_2p1;
		break;
	case VIDC_720P_H264_LEVEL2p2:
		*vcd_level = VCD_LEVEL_H264_2p2;
		break;
	case VIDC_720P_H264_LEVEL3:
		*vcd_level = VCD_LEVEL_H264_3;
		break;
	case VIDC_720P_H264_LEVEL3p1:
		*vcd_level = VCD_LEVEL_H264_3p1;
		break;
	case VIDC_720P_H264_LEVEL3p2:
		*vcd_level = VCD_LEVEL_H264_3p2;
		break;
	}
}

static void ddl_get_vc1_dec_level(enum vcd_codec_level_type *vcd_level,
		u32 level, enum vcd_codec_profile_type vc1_profile)
{
	if (vc1_profile == VCD_PROFILE_VC1_ADVANCE) {
		switch (level) {
		case VIDC_720P_VC1_LEVEL0:
			*vcd_level = VCD_LEVEL_VC1_0;
			break;
		case VIDC_720P_VC1_LEVEL1:
			*vcd_level = VCD_LEVEL_VC1_1;
			break;
		case VIDC_720P_VC1_LEVEL2:
			*vcd_level = VCD_LEVEL_VC1_2;
			break;
		case VIDC_720P_VC1_LEVEL3:
			*vcd_level = VCD_LEVEL_VC1_3;
			break;
		case VIDC_720P_VC1_LEVEL4:
			*vcd_level = VCD_LEVEL_VC1_4;
			break;
		}
		return;
	}

	/* now determine the Main and Simple profile level */
	switch (level) {
	case VIDC_720P_VC1_LEVEL_LOW:
		*vcd_level = VCD_LEVEL_VC1_LOW;
		break;
	case VIDC_720P_VC1_LEVEL_MED:
		*vcd_level = VCD_LEVEL_VC1_MEDIUM;
		break;
	case VIDC_720P_VC1_LEVEL_HIGH:
		*vcd_level = VCD_LEVEL_VC1_HIGH;
		break;
	}
}

static void ddl_get_mpeg2_dec_level(enum vcd_codec_level_type *vcd_level,
		u32 level)
{
	switch (level) {
	case VIDCL_720P_MPEG2_LEVEL_LOW:
		*vcd_level = VCD_LEVEL_MPEG2_LOW;
		break;
	case VIDCL_720P_MPEG2_LEVEL_MAIN:
		*vcd_level = VCD_LEVEL_MPEG2_MAIN;
		break;
	case VIDCL_720P_MPEG2_LEVEL_HIGH14:
		*vcd_level = VCD_LEVEL_MPEG2_HIGH_14;
		break;
	}
}

static void ddl_getdec_profilelevel(struct ddl_decoder_data *dec,
		u32 profile, u32 level)
{
	enum vcd_codec_profile_type vcd_profile = VCD_PROFILE_UNKNOWN;
	enum vcd_codec_level_type vcd_level = VCD_LEVEL_UNKNOWN;

	switch (dec->codec_type.codec) {
	case VCD_CODEC_MPEG4:
		if (profile == VIDC_720P_PROFILE_MPEG4_SP)
			vcd_profile = VCD_PROFILE_MPEG4_SP;
		else if (profile == VIDC_720P_PROFILE_MPEG4_ASP)
			vcd_profile = VCD_PROFILE_MPEG4_ASP;

		ddl_getmpeg4_declevel(&vcd_level, level);
		break;
	case VCD_CODEC_H264:
		if (profile == VIDC_720P_PROFILE_H264_BASELINE)
			vcd_profile = VCD_PROFILE_H264_BASELINE;
		else if (profile == VIDC_720P_PROFILE_H264_MAIN)
			vcd_profile = VCD_PROFILE_H264_MAIN;
		else if (profile == VIDC_720P_PROFILE_H264_HIGH)
			vcd_profile = VCD_PROFILE_H264_HIGH;
		ddl_geth264_declevel(&vcd_level, level);
		break;
	default:
	case VCD_CODEC_H263:
		break;
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		if (profile == VIDC_720P_PROFILE_VC1_SP)
			vcd_profile = VCD_PROFILE_VC1_SIMPLE;
		else if (profile == VIDC_720P_PROFILE_VC1_MAIN)
			vcd_profile = VCD_PROFILE_VC1_MAIN;
		else if (profile == VIDC_720P_PROFILE_VC1_ADV)
			vcd_profile = VCD_PROFILE_VC1_ADVANCE;
		ddl_get_vc1_dec_level(&vcd_level, level, vcd_profile);
		break;
	case VCD_CODEC_MPEG2:
		if (profile == VIDC_720P_PROFILE_MPEG2_MAIN)
			vcd_profile = VCD_PROFILE_MPEG2_MAIN;
		else if (profile == VIDC_720P_PROFILE_MPEG2_SP)
			vcd_profile = VCD_PROFILE_MPEG2_SIMPLE;
		ddl_get_mpeg2_dec_level(&vcd_level, level);
		break;
	}

	dec->profile.profile = vcd_profile;
	dec->level.level = vcd_level;
}
