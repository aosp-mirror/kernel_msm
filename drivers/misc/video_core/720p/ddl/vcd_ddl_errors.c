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
#include "vcd_ddl.h"

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define ERR(x...) printk(KERN_ERR x)

#define INVALID_CHANNEL_NUMBER  1
#define INVALID_COMMAND_ID 2
#define CHANNEL_ALREADY_IN_USE 3
#define CHANNEL_NOT_SET_BEFORE_CHANNEL_CLOSE 4
#define CHANNEL_SET_ERROR_INIT_CODEC 5
#define INIT_CODEC_ALREADY_CALLED 6
#define CHANNEL_SET_ERROR_INIT_BUFFERS 7
#define INIT_CODEC_ERROR_INIT_BUFFERS 8
#define INIT_BUFFER_ALREADY_CALLED  9
#define CHANNEL_SET_ERROR_FRAME_RUN 10
#define INIT_CODEC_ERROR_FRAME_RUN 11
#define INIT_BUFFERS_ERROR_FRAME_RUN 12
#define CODEC_LIMIT_EXCEEDED 13
#define FIRMWARE_SIZE_ZERO 14
#define FIRMWARE_ADDRESS_EXT_ZERO 15
#define CONTEXT_DMA_IN_ERROR 16
#define CONTEXT_DMA_OUT_ERROR 17
#define PROGRAM_DMA_ERROR 18
#define CONTEXT_STORE_EXT_ADD_ZERO 19
#define MEM_ALLOCATION_FAILED 20


#define UNSUPPORTED_FEATURE_IN_PROFILE 27
#define RESOLUTION_NOT_SUPPORTED 28
#define HEADER_NOT_FOUND 52
#define MB_NUM_INVALID 61
#define FRAME_RATE_NOT_SUPPORTED 62
#define INVALID_QP_VALUE 63
#define INVALID_RC_REACTION_COEFFICIENT 64
#define INVALID_CPB_SIZE_AT_GIVEN_LEVEL 65

#define ALLOC_DPB_SIZE_NOT_SUFFICIENT 71
#define ALLOC_DB_SIZE_NOT_SUFFICIENT 72
#define ALLOC_COMV_SIZE_NOT_SUFFICIENT 73
#define NUM_BUF_OUT_OF_RANGE 74
#define NULL_CONTEXT_POINTER 75
#define NULL_COMAMND_CONTROL_COMM_POINTER 76
#define NULL_METADATA_INPUT_POINTER 77
#define NULL_DPB_POINTER 78
#define NULL_DB_POINTER 79
#define NULL_COMV_POINTER 80

#define DIVIDE_BY_ZERO 81
#define BIT_STREAM_BUF_EXHAUST 82
#define DMA_NOT_STOPPED 83
#define DMA_TX_NOT_COMPLETE 84

#define MB_HEADER_NOT_DONE  85
#define MB_COEFF_NOT_DONE 86
#define CODEC_SLICE_NOT_DONE 87
#define VME_NOT_READY 88
#define VC1_BITPLANE_DECODE_ERR 89


#define VSP_NOT_READY 90
#define BUFFER_FULL_STATE 91

#define RESOLUTION_MISMATCH 112
#define NV_QUANT_ERR 113
#define SYNC_MARKER_ERR 114
#define FEATURE_NOT_SUPPORTED 115
#define MEM_CORRUPTION  116
#define INVALID_REFERENCE_FRAME  117
#define PICTURE_CODING_TYPE_ERR  118
#define MV_RANGE_ERR  119
#define PICTURE_STRUCTURE_ERR 120
#define SLICE_ADDR_INVALID  121
#define NON_PAIRED_FIELD_NOT_SUPPORTED  122
#define NON_FRAME_DATA_RECEIVED 123
#define INCOMPLETE_FRAME  124
#define NO_BUFFER_RELEASED_FROM_HOST  125
#define PICTURE_MANAGEMENT_ERROR  128
#define INVALID_MMCO  129
#define INVALID_PIC_REORDERING 130
#define INVALID_POC_TYPE 131
#define ACTIVE_SPS_NOT_PRESENT 132
#define ACTIVE_PPS_NOT_PRESENT 133
#define INVALID_SPS_ID 134
#define INVALID_PPS_ID 135


#define METADATA_NO_SPACE_QP 151
#define METADATA_NO_SAPCE_CONCEAL_MB 152
#define METADATA_NO_SPACE_VC1_PARAM 153
#define METADATA_NO_SPACE_SEI 154
#define METADATA_NO_SPACE_VUI 155
#define METADATA_NO_SPACE_EXTRA 156
#define METADATA_NO_SPACE_DATA_NONE 157
#define FRAME_RATE_UNKNOWN 158
#define ASPECT_RATIO_UNKOWN 159
#define COLOR_PRIMARIES_UNKNOWN 160
#define TRANSFER_CHAR_UNKWON 161
#define MATRIX_COEFF_UNKNOWN 162
#define NON_SEQ_SLICE_ADDR 163
#define BROKEN_LINK 164
#define FRAME_CONCEALED 165
#define PROFILE_UNKOWN 166
#define LEVEL_UNKOWN 167
#define BIT_RATE_NOT_SUPPORTED 168
#define COLOR_DIFF_FORMAT_NOT_SUPPORTED 169
#define NULL_EXTRA_METADATA_POINTER  170
#define SYNC_POINT_NOT_RECEIVED_STARTED_DECODING  171
#define NULL_FW_DEBUG_INFO_POINTER  172
#define ALLOC_DEBUG_INFO_SIZE_INSUFFICIENT  173
#define MAX_STAGE_COUNTER_EXCEEDED 174

#define METADATA_NO_SPACE_MB_INFO 180
#define METADATA_NO_SPACE_SLICE_SIZE 181
#define RESOLUTION_WARNING 182

void ddl_hw_fatal_cb(struct ddl_context_type *p_ddl_context)
{
	/* Invalidate the command state */
	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);
	p_ddl_context->n_device_state = DDL_DEVICE_HWFATAL;

	/* callback to the client to indicate hw fatal error */
	p_ddl_context->ddl_callback(VCD_EVT_IND_HWERRFATAL,
					VCD_ERR_HW_FATAL, NULL, 0,
					(void *)p_ddl_context->p_current_ddl,
					p_ddl_context->p_client_data);

	DDL_IDLE(p_ddl_context);
}

static u32 ddl_handle_hw_fatal_errors(struct ddl_context_type
			*p_ddl_context)
{
	u32 b_status = FALSE;

	switch (p_ddl_context->n_cmd_err_status) {

	case INVALID_CHANNEL_NUMBER:
	case INVALID_COMMAND_ID:
	case CHANNEL_ALREADY_IN_USE:
	case CHANNEL_NOT_SET_BEFORE_CHANNEL_CLOSE:
	case CHANNEL_SET_ERROR_INIT_CODEC:
	case INIT_CODEC_ALREADY_CALLED:
	case CHANNEL_SET_ERROR_INIT_BUFFERS:
	case INIT_CODEC_ERROR_INIT_BUFFERS:
	case INIT_BUFFER_ALREADY_CALLED:
	case CHANNEL_SET_ERROR_FRAME_RUN:
	case INIT_CODEC_ERROR_FRAME_RUN:
	case INIT_BUFFERS_ERROR_FRAME_RUN:
	case CODEC_LIMIT_EXCEEDED:
	case FIRMWARE_SIZE_ZERO:
	case FIRMWARE_ADDRESS_EXT_ZERO:

	case CONTEXT_DMA_IN_ERROR:
	case CONTEXT_DMA_OUT_ERROR:
	case PROGRAM_DMA_ERROR:
	case CONTEXT_STORE_EXT_ADD_ZERO:
	case MEM_ALLOCATION_FAILED:

	case DIVIDE_BY_ZERO:
	case DMA_NOT_STOPPED:
	case DMA_TX_NOT_COMPLETE:

	case VSP_NOT_READY:
	case BUFFER_FULL_STATE:
		ERR("HW FATAL ERROR");
		ddl_hw_fatal_cb(p_ddl_context);
		b_status = TRUE;
		break;
	}
	return b_status;
}

void ddl_client_fatal_cb(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type  *p_ddl =
		p_ddl_context->p_current_ddl;

	if (p_ddl_context->e_cmd_state == DDL_CMD_DECODE_FRAME)
		ddl_decode_dynamic_property(p_ddl, FALSE);
	else if (p_ddl_context->e_cmd_state == DDL_CMD_ENCODE_FRAME)
		ddl_encode_dynamic_property(p_ddl, FALSE);

	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);

	ddl_move_client_state(p_ddl, DDL_CLIENT_FATAL_ERROR);

	p_ddl_context->ddl_callback
	(
		VCD_EVT_IND_HWERRFATAL,
		VCD_ERR_CLIENT_FATAL,
		NULL,
		0,
		(void *)p_ddl,
		p_ddl_context->p_client_data
	);

	DDL_IDLE(p_ddl_context);
}

static u32 ddl_handle_client_fatal_errors(struct ddl_context_type
			*p_ddl_context)
{
	u32 b_status = FALSE;

	switch (p_ddl_context->n_cmd_err_status) {
	case UNSUPPORTED_FEATURE_IN_PROFILE:
	case RESOLUTION_NOT_SUPPORTED:
	case HEADER_NOT_FOUND:
	case INVALID_SPS_ID:
	case INVALID_PPS_ID:

	case MB_NUM_INVALID:
	case FRAME_RATE_NOT_SUPPORTED:
	case INVALID_QP_VALUE:
	case INVALID_RC_REACTION_COEFFICIENT:
	case INVALID_CPB_SIZE_AT_GIVEN_LEVEL:

	case ALLOC_DPB_SIZE_NOT_SUFFICIENT:
	case ALLOC_DB_SIZE_NOT_SUFFICIENT:
	case ALLOC_COMV_SIZE_NOT_SUFFICIENT:
	case NUM_BUF_OUT_OF_RANGE:
	case NULL_CONTEXT_POINTER:
	case NULL_COMAMND_CONTROL_COMM_POINTER:
	case NULL_METADATA_INPUT_POINTER:
	case NULL_DPB_POINTER:
	case NULL_DB_POINTER:
	case NULL_COMV_POINTER:
		{
			b_status = TRUE;
			break;
		}
	}

	if (!b_status)
		ERR("UNKNOWN-OP-FAILED");

	ddl_client_fatal_cb(p_ddl_context);

	return TRUE;
}

static void ddl_input_failed_cb(struct ddl_context_type *p_ddl_context,
			u32 vcd_event, u32 vcd_status)
{
	struct ddl_client_context_type  *p_ddl = p_ddl_context->p_current_ddl;

	ddl_move_command_state(p_ddl_context, DDL_CMD_INVALID);

	if (p_ddl->b_decoding)
		ddl_decode_dynamic_property(p_ddl, FALSE);
	else
		ddl_encode_dynamic_property(p_ddl, FALSE);

	p_ddl_context->ddl_callback(vcd_event,
		vcd_status, &p_ddl->input_frame,
		sizeof(struct ddl_frame_data_type_tag),
		(void *)p_ddl, p_ddl_context->p_client_data);

	ddl_move_client_state(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME);
}

static u32 ddl_handle_core_recoverable_errors(struct ddl_context_type \
			*p_ddl_context)
{
	struct ddl_client_context_type  *p_ddl = p_ddl_context->p_current_ddl;
	u32   vcd_status = VCD_S_SUCCESS;
	u32   vcd_event = VCD_EVT_RESP_INPUT_DONE;
	u32   b_eos = FALSE, pending_display = 0, release_mask = 0;

	if (p_ddl_context->e_cmd_state != DDL_CMD_DECODE_FRAME &&
		p_ddl_context->e_cmd_state != DDL_CMD_ENCODE_FRAME) {
		return FALSE;
	}
	switch (p_ddl_context->n_cmd_err_status) {
	case NON_PAIRED_FIELD_NOT_SUPPORTED:
		{
		vcd_status = VCD_ERR_INTRLCD_FIELD_DROP;
		break;
		}
	case NO_BUFFER_RELEASED_FROM_HOST:
		{
			/* lets check sanity of this error */
			release_mask =
				p_ddl->codec_data.decoder.dpb_mask.n_hw_mask;
			while (release_mask > 0) {
				if ((release_mask & 0x1))
					pending_display += 1;
				release_mask >>= 1;
			}

			if (pending_display >=
				p_ddl->codec_data.decoder.n_min_dpb_num) {
				DBG("FWISSUE-REQBUF!!");
				/* callback to client for client fatal error */
				ddl_client_fatal_cb(p_ddl_context);
				return TRUE ;
			}
		vcd_event = VCD_EVT_RESP_OUTPUT_REQ;
		break;
		}
	case BIT_STREAM_BUF_EXHAUST:
	case MB_HEADER_NOT_DONE:
	case MB_COEFF_NOT_DONE:
	case CODEC_SLICE_NOT_DONE:
	case VME_NOT_READY:
	case VC1_BITPLANE_DECODE_ERR:
		{
			u32 b_reset_core;
			/* need to reset the internal core hw engine */
			b_reset_core = ddl_hal_engine_reset(p_ddl_context);
			if (!b_reset_core)
				return TRUE;
			/* fall through to process bitstream error handling */
		}
	case RESOLUTION_MISMATCH:
	case NV_QUANT_ERR:
	case SYNC_MARKER_ERR:
	case FEATURE_NOT_SUPPORTED:
	case MEM_CORRUPTION:
	case INVALID_REFERENCE_FRAME:
	case PICTURE_CODING_TYPE_ERR:
	case MV_RANGE_ERR:
	case PICTURE_STRUCTURE_ERR:
	case SLICE_ADDR_INVALID:
	case NON_FRAME_DATA_RECEIVED:
	case INCOMPLETE_FRAME:
	case PICTURE_MANAGEMENT_ERROR:
	case INVALID_MMCO:
	case INVALID_PIC_REORDERING:
	case INVALID_POC_TYPE:
	case ACTIVE_SPS_NOT_PRESENT:
	case ACTIVE_PPS_NOT_PRESENT:
		{
			vcd_status = VCD_ERR_BITSTREAM_ERR;
			break;
		}
	}

	if (!vcd_status && vcd_event == VCD_EVT_RESP_INPUT_DONE)
		return FALSE;

	p_ddl->input_frame.b_frm_trans_end = TRUE;

	b_eos = ((vcd_event == VCD_EVT_RESP_INPUT_DONE) &&
		((VCD_FRAME_FLAG_EOS & p_ddl->input_frame.
				vcd_frm.n_flags)));

	if ((p_ddl->b_decoding && b_eos) ||
		(!p_ddl->b_decoding))
		p_ddl->input_frame.b_frm_trans_end = FALSE;

	if (vcd_event == VCD_EVT_RESP_INPUT_DONE &&
		p_ddl->b_decoding &&
		!p_ddl->codec_data.decoder.b_header_in_start &&
		!p_ddl->codec_data.decoder.dec_disp_info.n_img_size_x &&
		!p_ddl->codec_data.decoder.dec_disp_info.n_img_size_y
		) {
		/* this is first frame seq. header only case */
		vcd_status = VCD_S_SUCCESS;
		p_ddl->input_frame.vcd_frm.n_flags |=
			VCD_FRAME_FLAG_CODECCONFIG;
		p_ddl->input_frame.b_frm_trans_end = !b_eos;
		/* put just some non - zero value */
		p_ddl->codec_data.decoder.dec_disp_info.n_img_size_x = 0xff;
	}
	/* inform client about input failed */
	ddl_input_failed_cb(p_ddl_context, vcd_event, vcd_status);

	/* for Encoder case, we need to send output done also */
	if (!p_ddl->b_decoding) {
		/* transaction is complete after this callback */
		p_ddl->output_frame.b_frm_trans_end = !b_eos;
		/* error case: NO data present */
		p_ddl->output_frame.vcd_frm.n_data_len = 0;
		/* call back to client for output frame done */
		p_ddl_context->ddl_callback(VCD_EVT_RESP_OUTPUT_DONE,
		VCD_ERR_FAIL, &(p_ddl->output_frame),
			sizeof(struct ddl_frame_data_type_tag),
			(void *)p_ddl, p_ddl_context->p_client_data);

		if (b_eos) {
			DBG("ENC-EOS_DONE");
			/* send client EOS DONE callback */
			p_ddl_context->ddl_callback(VCD_EVT_RESP_EOS_DONE,
				VCD_S_SUCCESS, NULL, 0, (void *)p_ddl,
				p_ddl_context->p_client_data);
		}
	}

	/* if it is decoder EOS case */
	if (p_ddl->b_decoding && b_eos)
		ddl_decode_eos_run(p_ddl);
	else
		DDL_IDLE(p_ddl_context);

	return TRUE;
}

static u32 ddl_handle_core_warnings(u32 n_err_status)
{
	u32 b_status = FALSE;

	switch (n_err_status) {
	case FRAME_RATE_UNKNOWN:
	case ASPECT_RATIO_UNKOWN:
	case COLOR_PRIMARIES_UNKNOWN:
	case TRANSFER_CHAR_UNKWON:
	case MATRIX_COEFF_UNKNOWN:
	case NON_SEQ_SLICE_ADDR:
	case BROKEN_LINK:
	case FRAME_CONCEALED:
	case PROFILE_UNKOWN:
	case LEVEL_UNKOWN:
	case BIT_RATE_NOT_SUPPORTED:
	case COLOR_DIFF_FORMAT_NOT_SUPPORTED:
	case NULL_EXTRA_METADATA_POINTER:
	case SYNC_POINT_NOT_RECEIVED_STARTED_DECODING:

	case NULL_FW_DEBUG_INFO_POINTER:
	case ALLOC_DEBUG_INFO_SIZE_INSUFFICIENT:
	case MAX_STAGE_COUNTER_EXCEEDED:

	case METADATA_NO_SPACE_MB_INFO:
	case METADATA_NO_SPACE_SLICE_SIZE:
	case RESOLUTION_WARNING:

	/* decoder warnings */
	case METADATA_NO_SPACE_QP:
	case METADATA_NO_SAPCE_CONCEAL_MB:
	case METADATA_NO_SPACE_VC1_PARAM:
	case METADATA_NO_SPACE_SEI:
	case METADATA_NO_SPACE_VUI:
	case METADATA_NO_SPACE_EXTRA:
	case METADATA_NO_SPACE_DATA_NONE:
		{
			b_status = TRUE;
			DBG("CMD-WARNING-IGNORED!!");
			break;
		}
	}
	return b_status;
}

u32 ddl_handle_core_errors(struct ddl_context_type *p_ddl_context)
{
	u32 b_status = FALSE;

	if (!p_ddl_context->n_cmd_err_status &&
		!p_ddl_context->n_disp_pic_err_status)
		return FALSE;

	if (p_ddl_context->e_cmd_state == DDL_CMD_INVALID) {
		DBG("SPURIOUS_INTERRUPT_ERROR");
		return TRUE;
	}

	if (!p_ddl_context->n_op_failed) {
		u32 b_disp_status;
		b_status = ddl_handle_core_warnings(p_ddl_context->
			n_cmd_err_status);
		b_disp_status = ddl_handle_core_warnings(
			p_ddl_context->n_disp_pic_err_status);
		if (!b_status && !b_disp_status)
			DBG("ddl_warning:Unknown");

		return FALSE;
	}

	ERR("\n %s(): OPFAILED!!", __func__);
	ERR("\n CMD_ERROR_STATUS = %u, DISP_ERR_STATUS = %u",
		p_ddl_context->n_cmd_err_status,
		p_ddl_context->n_disp_pic_err_status);

	b_status = ddl_handle_hw_fatal_errors(p_ddl_context);

	if (!b_status)
		b_status = ddl_handle_core_recoverable_errors(p_ddl_context);

	if (!b_status)
		b_status = ddl_handle_client_fatal_errors(p_ddl_context);

	return b_status;
}
