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

struct ddl_context *ddl_get_context(void)
{
	static struct ddl_context ddl_context;
	return &ddl_context;
}

void ddl_move_client_state(struct ddl_client_context *ddl,
	enum ddl_client_state client_state)
{
	ddl->client_state = client_state;
}

void ddl_move_command_state(struct ddl_context *ddl_context,
	enum ddl_cmd_state command_state)
{
	ddl_context->cmd_state = command_state;
}

u32 ddl_client_transact(u32 operation, struct ddl_client_context **pddl_client)
{
	u32 ret_status = VCD_ERR_FAIL;
	u32 i;
	struct ddl_context *ddl_context;

	ddl_context = ddl_get_context();
	switch (operation) {
	case DDL_FREE_CLIENT:
		if (pddl_client && *pddl_client) {
			u32 channel_id;
			channel_id = (*pddl_client)->channel_id;
			if (channel_id < VCD_MAX_NO_CLIENT)
				ddl_context->ddl_clients[channel_id] = NULL;
			else
				pr_warn("CHID_CORRUPTION\n");
			kfree(*pddl_client);
			*pddl_client = NULL;
			ret_status = VCD_S_SUCCESS;
		}
		break;
	case DDL_GET_CLIENT:
		ret_status = VCD_ERR_MAX_CLIENT;
		for (i = 0; i < VCD_MAX_NO_CLIENT && ret_status ==
				VCD_ERR_MAX_CLIENT; ++i) {
			if (!ddl_context->ddl_clients[i]) {
				*pddl_client = (struct ddl_client_context *)
					kzalloc((sizeof(
					struct ddl_client_context)),
					GFP_KERNEL);
				if (!*pddl_client) {
					ret_status = VCD_ERR_ALLOC_FAIL;
					break;
				}
				ddl_context->ddl_clients[i] = *pddl_client;
				(*pddl_client)->channel_id = i;
				(*pddl_client)->ddl_context = ddl_context;
				ret_status = VCD_S_SUCCESS;
			}
		}
		break;
	case DDL_INIT_CLIENTS:
		for (i = 0; i < VCD_MAX_NO_CLIENT; ++i)
			ddl_context->ddl_clients[i] = NULL;
		ret_status = VCD_S_SUCCESS;
		break;
	case DDL_ACTIVE_CLIENT:
		for (i = 0; i < VCD_MAX_NO_CLIENT; ++i) {
			if (ddl_context->ddl_clients[i]) {
				ret_status = VCD_S_SUCCESS;
				break;
			}
		}
		break;
	default:
		ret_status = VCD_ERR_ILLEGAL_PARM;
		break;
	}
	return ret_status;
}

u32 ddl_decoder_dpb_transact(struct ddl_decoder_data *dec,
	struct ddl_frame_data_tag *in_out_frame, u32 operation)
{
	u32 vcd_status = VCD_S_SUCCESS;
	u32 i;
	struct ddl_frame_data_tag *found_frame = NULL;
	struct ddl_mask *dpb_mask = &dec->dpb_mask;

	switch (operation) {
	case DDL_DPB_OP_MARK_BUSY:
	case DDL_DPB_OP_MARK_FREE:
		for (i = 0; i < dec->dp_buf.no_of_dec_pic_buf; ++i) {
			if (in_out_frame->vcd_frm.phys_addr == dec->dp_buf.
					dec_pic_buffers[i].vcd_frm.phys_addr) {
				found_frame = &dec->dp_buf.dec_pic_buffers[i];
				break;
			}
		}

		if (!found_frame) {
			in_out_frame->vcd_frm.phys_addr = 0;
			vcd_status = VCD_ERR_BAD_POINTER;
			pr_debug("BUF_NOT_FOUND\n");
			break;
		}
		if (operation == DDL_DPB_OP_MARK_BUSY) {
			dpb_mask->hw_mask &= ~(0x1 << i);
			*in_out_frame = *found_frame;
		} else if (operation == DDL_DPB_OP_MARK_FREE) {
			dpb_mask->client_mask |= 0x1 << i;
			*found_frame = *in_out_frame;
		}

		break;
	case DDL_DPB_OP_SET_MASK:
		dpb_mask->hw_mask |= dpb_mask->client_mask;
		dpb_mask->client_mask = 0;
		vidc_720p_decode_set_dpb_release_buffer_mask(dpb_mask->hw_mask);
		break;
	case DDL_DPB_OP_INIT:
	{
		size_t dpb_size = !dec->meta_data_offset ?
			dec->dp_buf.dec_pic_buffers[0].vcd_frm.alloc_len :
			dec->meta_data_offset;
		vidc_720p_decode_set_dpb_details(dec->dp_buf.no_of_dec_pic_buf,
			dpb_size, dec->ref_buffer.phys_addr);
		for (i = 0; i < dec->dp_buf.no_of_dec_pic_buf; ++i) {
			vidc_720p_decode_set_dpb_buffers(i, dec->dp_buf.
				dec_pic_buffers[i].vcd_frm.phys_addr);
			pr_debug("DEC_DPB_BUFn_SIZE %u\n", dec->dp_buf.
				dec_pic_buffers[i].vcd_frm.alloc_len);
		}
		break;
	}
	case DDL_DPB_OP_RETRIEVE:
	{
		u32 position;
		u32 *mask;
		if (dpb_mask->client_mask) {
			mask = &dpb_mask->client_mask;
		} else if (dpb_mask->hw_mask) {
			mask = &dpb_mask->hw_mask;
		} else {
			in_out_frame->vcd_frm.phys_addr = 0;
			break;
		}
		position = 0x1;
		for (i = 0; i < dec->dp_buf.no_of_dec_pic_buf; ++i) {
			if (*mask & position) {
				found_frame = &dec->dp_buf.dec_pic_buffers[i];
				*mask &= ~position;
				*in_out_frame = *found_frame;
				break;
			}
			position <<= 1;
		}
		if (!found_frame)
			in_out_frame->vcd_frm.phys_addr = 0;
		break;
	}
	}
	return vcd_status;
}

void ddl_release_context_buffers(struct ddl_context *ddl_context)
{
	ddl_dma_free(&ddl_context->context_buf_addr);
	ddl_dma_free(&ddl_context->db_line_buffer);
	ddl_dma_free(&ddl_context->data_partition_tempbuf);
	ddl_dma_free(&ddl_context->metadata_shared_input);
	ddl_dma_free(&ddl_context->dbg_core_dump);
}

void ddl_release_client_internal_buffers(struct ddl_client_context *ddl)
{
	if (ddl->decoding) {
		struct ddl_decoder_data *dec = &(ddl->codec_data.decoder);
		ddl_dma_free(&dec->h264Vsp_temp_buffer);
		ddl_dma_free(&dec->dpb_comv_buffer);
		ddl_dma_free(&dec->ref_buffer);
		kfree(dec->dp_buf.dec_pic_buffers);
		dec->dp_buf.dec_pic_buffers = NULL;
		ddl_decode_dynamic_property(ddl, false);
		dec->decode_config.sz = 0;
		dec->decode_config.addr = 0;
		dec->dpb_mask.client_mask = 0;
		dec->dpb_mask.hw_mask = 0;
		dec->dp_buf.no_of_dec_pic_buf = 0;
		dec->dynamic_prop_change = 0;

	} else {
		struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);
		ddl_dma_free(&encoder->enc_dpb_addr);
		ddl_dma_free(&encoder->seq_header);
		ddl_encode_dynamic_property(ddl, false);
		encoder->dynamic_prop_change = 0;
	}
}
