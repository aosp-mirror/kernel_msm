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

u32 ddl_device_init(struct ddl_init_config *ddl_init_config, void *client_data)
{
	struct ddl_context *ddl_ctxt;
	u32 status = VCD_S_SUCCESS;

	if (!ddl_init_config || !ddl_init_config->ddl_callback ||
			!ddl_init_config->core_virtual_base_addr) {
		pr_err("ddl_dev_init:Bad_argument\n");
		return VCD_ERR_ILLEGAL_PARM;
	}

	ddl_ctxt = ddl_get_context();

	if (DDL_IS_INITIALIZED(ddl_ctxt)) {
		pr_err("ddl_dev_init:Multiple_init\n");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(ddl_ctxt)) {
		pr_err("ddl_dev_init:Ddl_busy\n");
		return VCD_ERR_BUSY;
	}

	memset(ddl_ctxt, 0, sizeof(struct ddl_context));

	DDL_BUSY(ddl_ctxt);

	ddl_ctxt->ddl_callback = ddl_init_config->ddl_callback;
	ddl_ctxt->pf_interrupt_clr = ddl_init_config->pf_interrupt_clr;
	ddl_ctxt->core_virtual_base_addr =
		ddl_init_config->core_virtual_base_addr;
	ddl_ctxt->client_data = client_data;

	ddl_ctxt->intr_status = DDL_INVALID_INTR_STATUS;

	vidc_720p_set_device_virtual_base(ddl_ctxt->core_virtual_base_addr);

	ddl_ctxt->current_ddl = NULL;
	ddl_move_command_state(ddl_ctxt, DDL_CMD_INVALID);

	ddl_client_transact(DDL_INIT_CLIENTS, NULL);

	if (!ddl_dma_alloc(&ddl_ctxt->context_buf_addr, DDL_CONTEXT_MEMORY, npelly_context)) {
		pr_err("ddl_dev_init:Context_alloc_fail\n");
		status = VCD_ERR_ALLOC_FAIL;
		goto out;
	}
	if (!ddl_dma_alloc(&ddl_ctxt->db_line_buffer, DDL_DB_LINE_BUF_SIZE, npelly_dbl)) {
		pr_err("ddl_dev_init:Line_buf_alloc_fail\n");
		status = VCD_ERR_ALLOC_FAIL;
		goto out;
	}
	if (!ddl_dma_alloc(&ddl_ctxt->data_partition_tempbuf,
			DDL_MPEG4_DATA_PARTITION_BUF_SIZE, npelly_mpeg4)) {
		pr_err("ddl_dev_init:"
			"Data_partition_buf_alloc_fail\n");
		status = VCD_ERR_ALLOC_FAIL;
		goto out;
	}
	if (!ddl_dma_alloc(&ddl_ctxt->metadata_shared_input,
			DDL_METADATA_TOTAL_INPUTBUFSIZE, npelly_meta)) {
		pr_err("ddl_dev_init:"
			"metadata_shared_input_alloc_fail\n");
		status = VCD_ERR_ALLOC_FAIL;
		goto out;
	}
	if (!ddl_dma_alloc(&ddl_ctxt->dbg_core_dump, DDL_DBG_CORE_DUMP_SIZE, npelly_debug)) {
		pr_err("ddl_dev_init:"
			"dbg_core_dump_alloc_failed\n");
		status = VCD_ERR_ALLOC_FAIL;
		ddl_ctxt->enable_dbg_core_dump = 0;
		goto out;
	}

out:
	if (status) {
		ddl_release_context_buffers(ddl_ctxt);
		DDL_IDLE(ddl_ctxt);
		return status;
	}

	ddl_move_command_state(ddl_ctxt, DDL_CMD_DMA_INIT);

	ddl_core_init(ddl_ctxt);

	return status;
}

u32 ddl_device_release(void *client_data)
{
	struct ddl_context *ddl_ctxt;

	ddl_ctxt = ddl_get_context();

	if (DDL_IS_BUSY(ddl_ctxt)) {
		pr_err("ddl_dev_rel:Ddl_busy\n");
		return VCD_ERR_BUSY;
	}

	if (!DDL_IS_INITIALIZED(ddl_ctxt)) {
		pr_err("ddl_dev_rel:Not_inited\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (!ddl_client_transact(DDL_ACTIVE_CLIENT, NULL)) {
		pr_err("ddl_dev_rel:Client_present_err\n");
		return VCD_ERR_CLIENT_PRESENT;
	}
	DDL_BUSY(ddl_ctxt);

	ddl_ctxt->device_state = DDL_DEVICE_NOTINIT;
	ddl_ctxt->client_data = client_data;
	ddl_move_command_state(ddl_ctxt, DDL_CMD_INVALID);
	vidc_720p_stop_fw();

	pr_debug("FW_ENDDONE\n");
	ddl_release_context_buffers(ddl_ctxt);

	DDL_IDLE(ddl_ctxt);

	return VCD_S_SUCCESS;
}

u32 ddl_open(u32 **ddl_handle, u32 decoding)
{
	struct ddl_context *ddl_context;
	struct ddl_client_context *ddl;
	u32 status;

	if (!ddl_handle) {
		pr_err("ddl_open:Bad_handle\n");
		return VCD_ERR_BAD_HANDLE;
	}

	ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(ddl_context)) {
		pr_err("ddl_open:Not_inited\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	status = ddl_client_transact(DDL_GET_CLIENT, &ddl);

	if (status) {
		pr_err("ddl_open:Client_trasac_failed\n");
		return status;
	}

	ddl_move_client_state(ddl, DDL_CLIENT_OPEN);

	ddl->codec_data.hdr.decoding = decoding;
	ddl->decoding = decoding;

	ddl_set_default_meta_data_hdr(ddl);

	ddl_set_initial_default_values(ddl);

	*ddl_handle = (u32 *) ddl;
	return VCD_S_SUCCESS;
}

u32 ddl_close(u32 **ddl_handle)
{
	struct ddl_context *ddl_context;
	struct ddl_client_context **pp_ddl = (struct ddl_client_context **)
		ddl_handle;

	if (!pp_ddl || !*pp_ddl) {
		pr_err("ddl_close:Bad_handle\n");
		return VCD_ERR_BAD_HANDLE;
	}

	ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(ddl_context)) {
		pr_err("ddl_close:Not_inited\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (!DDLCLIENT_STATE_IS(*pp_ddl, DDL_CLIENT_OPEN)) {
		pr_err("ddl_close:Not_in_open_state\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	ddl_move_client_state(*pp_ddl, DDL_CLIENT_INVALID);

	ddl_client_transact(DDL_FREE_CLIENT, pp_ddl);

	return VCD_S_SUCCESS;
}

u32 ddl_encode_start(u32 *ddl_handle, void *client_data)
{
	struct ddl_client_context *ddl =
		(struct ddl_client_context *)ddl_handle;
	struct ddl_context *ddl_context;
	struct ddl_encoder_data *enc;
	u32 dpb_size;

	ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(ddl_context)) {
		pr_err("ddl_enc_start:Not_inited\n");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(ddl_context)) {
		pr_err("ddl_enc_start:Ddl_busy\n");
		return VCD_ERR_BUSY;
	}
	if (!ddl || ddl->decoding) {
		pr_err("ddl_enc_start:Bad_handle\n");
		return VCD_ERR_BAD_HANDLE;
	}

	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)) {
		pr_err("ddl_enc_start:Not_opened\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (!ddl_encoder_ready_to_start(ddl)) {
		pr_err("ddl_enc_start:Err_param_settings\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	enc = &ddl->codec_data.encoder;

	dpb_size = ddl_get_yuv_buffer_size(&enc->frame_size,
					&enc->re_con_buf_format, false);

	dpb_size *= DDL_ENC_NUM_DPB_BUFFERS;
	if (!ddl_dma_alloc(&enc->enc_dpb_addr, dpb_size, npelly_enc_dpb)) {
		pr_err("ddl_enc_start:Dpb_alloc_failed\n");
		return VCD_ERR_ALLOC_FAIL;
	}

	if ((enc->codec_type.codec == VCD_CODEC_MPEG4 &&
			!enc->short_header.short_header) ||
			enc->codec_type.codec == VCD_CODEC_H264) {
		if (!ddl_dma_alloc(&enc->seq_header, DDL_ENC_SEQHEADER_SIZE, npelly_enc_seq)) {
			ddl_dma_free(&enc->enc_dpb_addr);
			pr_err("ddl_enc_start:Seq_hdr_alloc_failed\n");
			return VCD_ERR_ALLOC_FAIL;
		}
	} else {
		enc->seq_header.size = 0;
		enc->seq_header.virt_addr = NULL;
	}

	DDL_BUSY(ddl_context);

	ddl_context->current_ddl = ddl;
	ddl_context->client_data = client_data;
	ddl_channel_set(ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_decode_start(u32 *ddl_handle, struct vcd_phys_sequence_hdr *hdr,
		void *client_data)
{
	struct ddl_client_context *ddl = (struct ddl_client_context *)
		ddl_handle;
	struct ddl_context *ddl_context;
	struct ddl_decoder_data *decoder;

	ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(ddl_context)) {
		pr_err("ddl_dec_start:Not_inited\n");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(ddl_context)) {
		pr_err("ddl_dec_start:Ddl_busy\n");
		return VCD_ERR_BUSY;
	}
	if (!ddl || !ddl->decoding) {
		pr_err("ddl_dec_start:Bad_handle\n");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)) {
		pr_err("ddl_dec_start:Not_in_opened_state\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (hdr && (!hdr->sz || !hdr->addr)) {
		pr_err("ddl_dec_start:Bad_param_seq_header\n");
		return VCD_ERR_ILLEGAL_PARM;
	}

	if (!ddl_decoder_ready_to_start(ddl, hdr)) {
		pr_err("ddl_dec_start:Err_param_settings\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	DDL_BUSY(ddl_context);

	decoder = &ddl->codec_data.decoder;
	if (hdr) {
		decoder->header_in_start = true;
		decoder->decode_config = *hdr;
	} else {
		decoder->header_in_start = false;
		decoder->decode_config.sz = 0;
	}

	if (decoder->codec_type.codec == VCD_CODEC_H264) {
		if (!ddl_dma_alloc(&decoder->h264Vsp_temp_buffer,
			       DDL_DECODE_H264_VSPTEMP_BUFSIZE,
				npelly_dec_h264)) {
			DDL_IDLE(ddl_context);
			pr_err("ddl_dec_start:H264Sps_alloc_failed\n");
			return VCD_ERR_ALLOC_FAIL;
		}
	}

	ddl_context->current_ddl = ddl;
	ddl_context->client_data = client_data;

	ddl_channel_set(ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_decode_frame(u32 *ddl_handle, struct ddl_frame_data_tag *in_bits,
		void *client_data)
{
	u32 vcd_status = VCD_S_SUCCESS;
	struct ddl_client_context *ddl = (struct ddl_client_context *)
		ddl_handle;
	struct ddl_context *ddl_context = ddl_get_context();

#ifdef CORE_TIMING_INFO
	ddl_get_core_start_time(0);
#endif

	if (!DDL_IS_INITIALIZED(ddl_context)) {
		pr_err("ddl_dec_frame:Not_inited\n");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(ddl_context)) {
		pr_err("ddl_dec_frame:Ddl_busy\n");
		return VCD_ERR_BUSY;
	}
	if (!ddl || !ddl->decoding) {
		pr_err("ddl_dec_frame:Bad_handle\n");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!in_bits || ((!in_bits->vcd_frm.phys_addr ||
			!in_bits->vcd_frm.data_len) &&
			!(VCD_FRAME_FLAG_EOS & in_bits->vcd_frm.flags))) {
		pr_err("ddl_dec_frame:Bad_input_param\n");
		return VCD_ERR_ILLEGAL_PARM;
	}

	DDL_BUSY(ddl_context);

	ddl_context->current_ddl = ddl;
	ddl_context->client_data = client_data;

	ddl->input_frame = *in_bits;

	if (DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME)) {
		ddl_decode_frame_run(ddl);
	} else {
		if (!ddl->codec_data.decoder.dp_buf.no_of_dec_pic_buf) {
			pr_err("ddl_dec_frame:Dpbs_requied\n");
			vcd_status = VCD_ERR_ILLEGAL_OP;
		} else if (DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_DPB)) {
			vcd_status = ddl_decode_set_buffers(ddl);
		} else if (DDLCLIENT_STATE_IS(ddl,
				DDL_CLIENT_WAIT_FOR_INITCODEC)) {
			ddl->codec_data.decoder.decode_config.addr =
				ddl->input_frame.vcd_frm.phys_addr;
			ddl->codec_data.decoder.decode_config.sz =
				ddl->input_frame.vcd_frm.data_len;
			ddl_decode_init_codec(ddl);
		} else {
			pr_err("Dec_frame:Wrong_state\n");
			vcd_status = VCD_ERR_ILLEGAL_OP;
		}
		if (vcd_status)
			DDL_IDLE(ddl_context);
	}
	return vcd_status;
}

u32 ddl_encode_frame(u32 *ddl_handle, struct ddl_frame_data_tag *input_frame,
		struct ddl_frame_data_tag *out_bits, void *client_data)
{
	struct ddl_client_context *ddl = (struct ddl_client_context *)
		ddl_handle;
	struct ddl_context *ddl_context = ddl_get_context();

#ifdef CORE_TIMING_INFO
	ddl_get_core_start_time(1);
#endif

	if (!DDL_IS_INITIALIZED(ddl_context)) {
		pr_err("ddl_encode_frame:Not_inited\n");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(ddl_context)) {
		pr_err("ddl_encode_frame:Ddl_busy\n");
		return VCD_ERR_BUSY;
	}
	if (!ddl || ddl->decoding) {
		pr_err("ddl_encode_frame:Bad_handle\n");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!input_frame || !input_frame->vcd_frm.phys_addr ||
			ddl->codec_data.encoder.input_buf_req.size !=
			input_frame->vcd_frm.data_len) {
		pr_err("ddl_encode_frame:Bad_input_params\n");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if ((input_frame->vcd_frm.phys_addr + input_frame->vcd_frm.offset) &
			DDL_STREAMBUF_ALIGN_GUARD_BYTES) {
		pr_err("ddl_encode_frame:unaligned_yuv_start_addr\n");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if (!out_bits || !out_bits->vcd_frm.phys_addr ||
			!out_bits->vcd_frm.alloc_len) {
		pr_err("ddl_encode_frame:Bad_output_params\n");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if ((ddl->codec_data.encoder.output_buf_req.size +
			out_bits->vcd_frm.offset) >
			out_bits->vcd_frm.alloc_len) {
		pr_err("ddl_encode_frame:offset > min_buf_size\n");
	}
	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME)) {
		pr_err("ddl_encode_frame:Wrong_state\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	DDL_BUSY(ddl_context);

	ddl_context->current_ddl = ddl;
	ddl_context->client_data = client_data;

	ddl->input_frame = *input_frame;
	ddl->output_frame = *out_bits;

	ddl_encode_frame_run(ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_decode_end(u32 *ddl_handle, void *client_data)
{
	struct ddl_client_context *ddl = (struct ddl_client_context *)
		ddl_handle;
	struct ddl_context *ddl_context;

	ddl_context = ddl_get_context();

#ifdef CORE_TIMING_INFO
	ddl_reset_time_variables(0);
#endif

	if (!DDL_IS_INITIALIZED(ddl_context)) {
		pr_err("ddl_dec_end:Not_inited\n");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(ddl_context)) {
		pr_err("ddl_dec_end:Ddl_busy\n");
		return VCD_ERR_BUSY;
	}
	if (!ddl || !ddl->decoding) {
		pr_err("ddl_dec_end:Bad_handle\n");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME) &&
			!DDLCLIENT_STATE_IS(ddl,
			DDL_CLIENT_WAIT_FOR_INITCODEC) &&
			!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_DPB) &&
			!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_FATAL_ERROR)) {
		pr_err("ddl_dec_end:Wrong_state\n");
		return VCD_ERR_ILLEGAL_OP;
	}
	DDL_BUSY(ddl_context);

	ddl_context->current_ddl = ddl;
	ddl_context->client_data = client_data;

	ddl_channel_end(ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_encode_end(u32 *ddl_handle, void *client_data)
{
	struct ddl_client_context *ddl = (struct ddl_client_context *)
		ddl_handle;
	struct ddl_context *ddl_context;

	ddl_context = ddl_get_context();

#ifdef CORE_TIMING_INFO
	ddl_reset_time_variables(1);
#endif

	if (!DDL_IS_INITIALIZED(ddl_context)) {
		pr_err("ddl_enc_end:Not_inited\n");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(ddl_context)) {
		pr_err("ddl_enc_end:Ddl_busy\n");
		return VCD_ERR_BUSY;
	}
	if (!ddl || ddl->decoding) {
		pr_err("ddl_enc_end:Bad_handle\n");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME) &&
		!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_INITCODEC) &&
		!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_FATAL_ERROR)) {
		pr_err("ddl_enc_end:Wrong_state\n");
		return VCD_ERR_ILLEGAL_OP;
	}
	DDL_BUSY(ddl_context);

	ddl_context->current_ddl = ddl;
	ddl_context->client_data = client_data;

	ddl_channel_end(ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_reset_hw(u32 mode)
{
	struct ddl_context *ddl_context;
	struct ddl_client_context *ddl;
	int client_num;

	pr_debug("ddl_reset_hw:called\n");
	ddl_context = ddl_get_context();
	ddl_move_command_state(ddl_context, DDL_CMD_INVALID);
	DDL_BUSY(ddl_context);

	if (ddl_context->core_virtual_base_addr)
		vidc_720p_do_sw_reset();

	ddl_context->device_state = DDL_DEVICE_NOTINIT;
	for (client_num = 0; client_num < VCD_MAX_NO_CLIENT; ++client_num) {
		ddl = ddl_context->ddl_clients[client_num];
		ddl_context->ddl_clients[client_num] = NULL;
		if (ddl) {
			ddl_release_client_internal_buffers(ddl);
			ddl_client_transact(DDL_FREE_CLIENT, &ddl);
		}
	}

	ddl_release_context_buffers(ddl_context);
	memset(ddl_context, 0, sizeof(struct ddl_context));

	return true;
}
