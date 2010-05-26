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

static u32 *ddl_metadata_hdr_entry(struct ddl_client_context_type *p_ddl,
				   u32 n_meta_data_type)
{
	u32 n_skip_words = 0;
	u32 *p_buffer;

	if (p_ddl->b_decoding) {
		p_buffer = (u32 *)
		    p_ddl->codec_data.decoder.meta_data_input.
		    p_align_virtual_addr;
		n_skip_words = 32 + 1;
		p_buffer += n_skip_words;

		switch (n_meta_data_type) {
		default:
		case VCD_METADATA_DATANONE:
			{
				n_skip_words = 0;
				break;
			}
		case VCD_METADATA_QPARRAY:
			{
				n_skip_words = 3;
				break;
			}
		case VCD_METADATA_CONCEALMB:
			{
				n_skip_words = 6;
				break;
			}
		case VCD_METADATA_VC1:
			{
				n_skip_words = 9;
				break;
			}
		case VCD_METADATA_SEI:
			{
				n_skip_words = 12;
				break;
			}
		case VCD_METADATA_VUI:
			{
				n_skip_words = 15;
				break;
			}
		case VCD_METADATA_PASSTHROUGH:
			{
				n_skip_words = 18;
				break;
			}
		case VCD_METADATA_QCOMFILLER:
			{
				n_skip_words = 21;
				break;
			}
		}
	} else {
		p_buffer = (u32 *)
		    p_ddl->codec_data.encoder.meta_data_input.
		    p_align_virtual_addr;
		n_skip_words = 2;
		p_buffer += n_skip_words;

		switch (n_meta_data_type) {
		default:
		case VCD_METADATA_DATANONE:
			{
				n_skip_words = 0;
				break;
			}
		case VCD_METADATA_ENC_SLICE:
			{
				n_skip_words = 3;
				break;
			}
		case VCD_METADATA_QCOMFILLER:
			{
				n_skip_words = 6;
				break;
			}
		}

	}

	p_buffer += n_skip_words;
	return p_buffer;
}

void ddl_set_default_meta_data_hdr(struct ddl_client_context_type *p_ddl)
{
	struct ddl_buf_addr_type *p_main_buffer =
	    &p_ddl->p_ddl_context->metadata_shared_input;
	struct ddl_buf_addr_type *p_client_buffer;
	u32 *p_hdr_entry;

	if (p_ddl->b_decoding)
		p_client_buffer = &(p_ddl->codec_data.decoder.meta_data_input);
	else
		p_client_buffer = &(p_ddl->codec_data.encoder.meta_data_input);

	DDL_METADATA_CLIENT_INPUTBUF(p_main_buffer, p_client_buffer,
				     p_ddl->n_channel_id);

	p_hdr_entry = ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_QCOMFILLER);
	p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] = 1;
	p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX] = 1;
	p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_QCOMFILLER;

	p_hdr_entry = ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_DATANONE);
	p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] = 2;
	p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX] = 2;
	p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_DATANONE;

	if (p_ddl->b_decoding) {
		p_hdr_entry =
		    ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_QPARRAY);
		p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] = 3;
		p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX] = 3;
		p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_QPARRAY;

		p_hdr_entry =
		    ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_CONCEALMB);
		p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] = 4;
		p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX] = 4;
		p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] =
		    VCD_METADATA_CONCEALMB;

		p_hdr_entry = ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_SEI);
		p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] = 5;
		p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX] = 5;
		p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_SEI;

		p_hdr_entry = ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_VUI);
		p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] = 6;
		p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX] = 6;
		p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_VUI;

		p_hdr_entry = ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_VC1);
		p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] = 7;
		p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX] = 7;
		p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_VC1;

		p_hdr_entry =
		    ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_PASSTHROUGH);
		p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] = 8;
		p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX] = 8;
		p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] =
		    VCD_METADATA_PASSTHROUGH;

	} else {
		p_hdr_entry =
		    ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_ENC_SLICE);
		p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] = 9;
		p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX] = 9;
		p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] =
		    VCD_METADATA_ENC_SLICE;
	}
}

static u32 ddl_supported_metadata_flag(struct ddl_client_context_type *p_ddl)
{
	u32 n_flag = 0;

	if (p_ddl->b_decoding) {
		enum vcd_codec_type e_codec =
		    p_ddl->codec_data.decoder.codec_type.e_codec;

		n_flag |= (VCD_METADATA_CONCEALMB |
			   VCD_METADATA_PASSTHROUGH | VCD_METADATA_QPARRAY);
		if (e_codec == VCD_CODEC_H264) {
			n_flag |= (VCD_METADATA_SEI | VCD_METADATA_VUI);
		} else if (e_codec == VCD_CODEC_VC1 ||
			   e_codec == VCD_CODEC_VC1_RCV) {
			n_flag |= VCD_METADATA_VC1;
		}
	} else {
		n_flag |= VCD_METADATA_ENC_SLICE;
	}

	return n_flag;
}

void ddl_set_default_metadata_flag(struct ddl_client_context_type *p_ddl)
{
	if (p_ddl->b_decoding)
		p_ddl->codec_data.decoder.n_meta_data_enable_flag = 0;
	else
		p_ddl->codec_data.encoder.n_meta_data_enable_flag = 0;
}

void ddl_set_default_decoder_metadata_buffer_size(
	struct ddl_decoder_data_type *p_decoder,
	struct vcd_property_frame_size_type *p_frame_size,
	struct vcd_buffer_requirement_type *p_output_buf_req)
{
	u32 n_flag = p_decoder->n_meta_data_enable_flag;
	u32 n_suffix = 0;
	u32 n_size = 0;

	if (!n_flag) {
		p_decoder->n_suffix = 0;
		return;
	}

	if (n_flag & VCD_METADATA_QPARRAY) {
		u32 n_num_of_mb =
		    ((p_frame_size->n_width * p_frame_size->n_height) >> 8);
		n_size = DDL_METADATA_HDR_SIZE;
		n_size += n_num_of_mb;
		DDL_METADATA_ALIGNSIZE(n_size);
		n_suffix += n_size;
	}
	if (n_flag & VCD_METADATA_CONCEALMB) {
		u32 n_num_of_mb =
		    ((p_frame_size->n_width * p_frame_size->n_height) >> 8);
		n_size = DDL_METADATA_HDR_SIZE;
		n_size *= (4 * n_num_of_mb / 2);
		DDL_METADATA_ALIGNSIZE(n_size);
		n_suffix += n_size;
	}
	if (n_flag & VCD_METADATA_VC1) {
		n_size = DDL_METADATA_HDR_SIZE;
		n_size += DDL_METADATA_VC1_PAYLOAD_SIZE;
		DDL_METADATA_ALIGNSIZE(n_size);
		n_suffix += n_size;
	}
	if (n_flag & VCD_METADATA_SEI) {
		n_size = DDL_METADATA_HDR_SIZE;
		n_size += DDL_METADATA_SEI_PAYLOAD_SIZE;
		DDL_METADATA_ALIGNSIZE(n_size);
		n_suffix += (n_size * DDL_METADATA_SEI_MAX);
	}
	if (n_flag & VCD_METADATA_VUI) {
		n_size = DDL_METADATA_HDR_SIZE;
		n_size += DDL_METADATA_VUI_PAYLOAD_SIZE;
		DDL_METADATA_ALIGNSIZE(n_size);
		n_suffix += (n_size);
	}
	if (n_flag & VCD_METADATA_PASSTHROUGH) {
		n_size = DDL_METADATA_HDR_SIZE;
		n_size += DDL_METADATA_PASSTHROUGH_PAYLOAD_SIZE;
		DDL_METADATA_ALIGNSIZE(n_size);
		n_suffix += (n_size);
	}
	n_size = DDL_METADATA_EXTRADATANONE_SIZE;
	DDL_METADATA_ALIGNSIZE(n_size);
	n_suffix += (n_size);

	n_suffix += DDL_METADATA_EXTRAPAD_SIZE;
	DDL_METADATA_ALIGNSIZE(n_suffix);

	p_decoder->n_suffix = n_suffix;
	p_output_buf_req->n_size += n_suffix;
	return;
}

void ddl_set_default_encoder_metadata_buffer_size(struct ddl_encoder_data_type
						  *p_encoder)
{
	u32 n_flag = p_encoder->n_meta_data_enable_flag;
	u32 n_suffix = 0;
	u32 n_size = 0;

	if (!n_flag) {
		p_encoder->n_suffix = 0;
		return;
	}

	if (n_flag & VCD_METADATA_ENC_SLICE) {
		u32 n_num_of_mb = (p_encoder->frame_size.n_width *
				   p_encoder->frame_size.n_height / 16 / 16);
		n_size = DDL_METADATA_HDR_SIZE;

		n_size += 4;

		n_size += (8 * n_num_of_mb);
		DDL_METADATA_ALIGNSIZE(n_size);
		n_suffix += n_size;
	}

	n_size = DDL_METADATA_EXTRADATANONE_SIZE;
	DDL_METADATA_ALIGNSIZE(n_size);
	n_suffix += (n_size);

	n_suffix += DDL_METADATA_EXTRAPAD_SIZE;
	DDL_METADATA_ALIGNSIZE(n_suffix);

	p_encoder->n_suffix = n_suffix;
	p_encoder->output_buf_req.n_size += n_suffix;
}

u32 ddl_set_metadata_params(struct ddl_client_context_type *p_ddl,
			    struct vcd_property_hdr_type *p_property_hdr,
			    void *p_property_value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	if (p_property_hdr->prop_id == VCD_I_METADATA_ENABLE) {
		struct vcd_property_meta_data_enable_type *p_meta_data_enable =
		    (struct vcd_property_meta_data_enable_type *)
		    p_property_value;
		u32 *p_meta_data_enable_flag;
		enum vcd_codec_type e_codec;
		if (p_ddl->b_decoding) {
			p_meta_data_enable_flag =
			    &(p_ddl->codec_data.decoder.
			      n_meta_data_enable_flag);
			e_codec = p_ddl->codec_data.decoder.codec_type.e_codec;
		} else {
			p_meta_data_enable_flag =
			    &(p_ddl->codec_data.encoder.
			      n_meta_data_enable_flag);
			e_codec = p_ddl->codec_data.encoder.codec_type.e_codec;
		}
		if (sizeof(struct vcd_property_meta_data_enable_type) ==
		    p_property_hdr->n_size &&
		    DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_OPEN) &&
					e_codec) {
			if (!p_meta_data_enable->n_meta_data_enable_flag) {
				*p_meta_data_enable_flag = 0;
				if (p_ddl->b_decoding) {
					ddl_set_default_decoder_buffer_req
					    (&p_ddl->codec_data.decoder, TRUE);
				} else {
					ddl_set_default_encoder_buffer_req
					    (&p_ddl->codec_data.encoder);
				}

			} else {
				u32 n_flag = ddl_supported_metadata_flag(p_ddl);
				n_flag &=
					(p_meta_data_enable->
					 n_meta_data_enable_flag);
				if (n_flag) {
					n_flag |= DDL_METADATA_MANDATORY;
					if (n_flag !=
						*p_meta_data_enable_flag) {
						*p_meta_data_enable_flag =
						    n_flag;

				if (p_ddl->b_decoding) {
					ddl_set_default_decoder_buffer_req
						(&p_ddl->codec_data.
						decoder, TRUE);
				} else {
					ddl_set_default_encoder_buffer_req
						(&p_ddl->codec_data.
						 encoder);
				}

					}
				}
			}
			vcd_status = VCD_S_SUCCESS;
		}
	} else if (p_property_hdr->prop_id == VCD_I_METADATA_HEADER) {
		struct vcd_property_metadata_hdr_type *p_hdr =
		    (struct vcd_property_metadata_hdr_type *)p_property_value;
		if (sizeof(struct vcd_property_metadata_hdr_type) ==
		    p_property_hdr->n_size) {
			u32 n_flag = ddl_supported_metadata_flag(p_ddl);
			n_flag |= DDL_METADATA_MANDATORY;
			n_flag &= p_hdr->n_meta_data_id_type;
			if (!(n_flag & (n_flag - 1))) {
				u32 *p_hdr_entry =
				    ddl_metadata_hdr_entry(p_ddl, n_flag);
				p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] =
				    p_hdr->n_version;
				p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX] =
				    p_hdr->n_port_index;
				p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] =
				    p_hdr->e_type;
				vcd_status = VCD_S_SUCCESS;
			}
		}
	}
	return vcd_status;
}

u32 ddl_get_metadata_params(struct ddl_client_context_type *p_ddl,
	struct vcd_property_hdr_type *p_property_hdr,
	void	*p_property_value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM ;
	if (p_property_hdr->prop_id == VCD_I_METADATA_ENABLE &&
		sizeof(struct vcd_property_meta_data_enable_type)
		== p_property_hdr->n_size) {
		struct vcd_property_meta_data_enable_type *p_meta_data_enable =
			(struct vcd_property_meta_data_enable_type *)
			p_property_value;
		p_meta_data_enable->n_meta_data_enable_flag =
			((p_ddl->b_decoding) ?
			(p_ddl->codec_data.decoder.n_meta_data_enable_flag)
			: (p_ddl->codec_data.encoder.n_meta_data_enable_flag));
		vcd_status = VCD_S_SUCCESS;
	} else if (p_property_hdr->prop_id == VCD_I_METADATA_HEADER &&
		sizeof(struct vcd_property_metadata_hdr_type) ==
		p_property_hdr->n_size) {
		struct vcd_property_metadata_hdr_type *p_hdr =
			(struct vcd_property_metadata_hdr_type *)
			p_property_value;
		u32 n_flag = ddl_supported_metadata_flag(p_ddl);
		n_flag |= DDL_METADATA_MANDATORY;
		n_flag &= p_hdr->n_meta_data_id_type;
		if (!(n_flag & (n_flag - 1))) {
			u32 *p_hdr_entry = ddl_metadata_hdr_entry(p_ddl,
				n_flag);
			p_hdr->n_version =
			p_hdr_entry[DDL_METADATA_HDR_VERSION_INDEX];
			p_hdr->n_port_index =
			p_hdr_entry[DDL_METADATA_HDR_PORT_INDEX];
			p_hdr->e_type =
				p_hdr_entry[DDL_METADATA_HDR_TYPE_INDEX];
			vcd_status = VCD_S_SUCCESS;
		}
	}
	return vcd_status;
}

void ddl_metadata_enable(struct ddl_client_context_type *p_ddl)
{
	u32 n_flag, n_hal_flag = 0;
	u32 *p_metadata_input;
	if (p_ddl->b_decoding) {
		n_flag = p_ddl->codec_data.decoder.n_meta_data_enable_flag;
		p_metadata_input =
		    p_ddl->codec_data.decoder.meta_data_input.
		    p_align_physical_addr;
	} else {
		n_flag = p_ddl->codec_data.encoder.n_meta_data_enable_flag;
		p_metadata_input =
		    p_ddl->codec_data.encoder.meta_data_input.
		    p_align_physical_addr;
	}
	if (n_flag) {
		if (n_flag & VCD_METADATA_QPARRAY)
			n_hal_flag |= VIDC_720P_METADATA_ENABLE_QP;
		if (n_flag & VCD_METADATA_CONCEALMB)
			n_hal_flag |= VIDC_720P_METADATA_ENABLE_CONCEALMB;
		if (n_flag & VCD_METADATA_VC1)
			n_hal_flag |= VIDC_720P_METADATA_ENABLE_VC1;
		if (n_flag & VCD_METADATA_SEI)
			n_hal_flag |= VIDC_720P_METADATA_ENABLE_SEI;
		if (n_flag & VCD_METADATA_VUI)
			n_hal_flag |= VIDC_720P_METADATA_ENABLE_VUI;
		if (n_flag & VCD_METADATA_ENC_SLICE)
			n_hal_flag |= VIDC_720P_METADATA_ENABLE_ENCSLICE;
		if (n_flag & VCD_METADATA_PASSTHROUGH)
			n_hal_flag |= VIDC_720P_METADATA_ENABLE_PASSTHROUGH;
	} else {
		p_metadata_input = 0;
	}
	vidc_720p_metadata_enable(n_hal_flag, p_metadata_input);
}

u32 ddl_encode_set_metadata_output_buf(struct ddl_client_context_type *p_ddl)
{
	struct ddl_encoder_data_type *p_encoder = &p_ddl->codec_data.encoder;
	u32 *p_buffer;
	struct vcd_frame_data_type *p_stream = &(p_ddl->output_frame.vcd_frm);
	u32 n_ext_buffer_end, n_hw_metadata_start;

	n_ext_buffer_end = (u32) p_stream->p_physical + p_stream->n_alloc_len;
	if (!p_encoder->n_meta_data_enable_flag) {
		n_ext_buffer_end &= ~(DDL_STREAMBUF_ALIGN_GUARD_BYTES);
		return n_ext_buffer_end;
	}
	n_hw_metadata_start = (n_ext_buffer_end - p_encoder->n_suffix) &
	    ~(DDL_STREAMBUF_ALIGN_GUARD_BYTES);

	n_ext_buffer_end = (n_hw_metadata_start - 1) &
	    ~(DDL_STREAMBUF_ALIGN_GUARD_BYTES);

	p_buffer = p_encoder->meta_data_input.p_align_virtual_addr;

	*p_buffer++ = p_encoder->n_suffix;

	*p_buffer = n_hw_metadata_start;

	p_encoder->n_meta_data_offset =
	    n_hw_metadata_start - (u32) p_stream->p_physical;

	return n_ext_buffer_end;
}

void ddl_decode_set_metadata_output(struct ddl_decoder_data_type *p_decoder)
{
	u32 *p_buffer;
	u32 n_loopc;

	if (!p_decoder->n_meta_data_enable_flag) {
		p_decoder->n_meta_data_offset = 0;
		return;
	}

	p_decoder->n_meta_data_offset = ddl_get_yuv_buffer_size(
		&p_decoder->client_frame_size, &p_decoder->buf_format,
		(!p_decoder->n_progressive_only));

	p_buffer = p_decoder->meta_data_input.p_align_virtual_addr;

	*p_buffer++ = p_decoder->n_suffix;

	for (n_loopc = 0; n_loopc < p_decoder->dp_buf.n_no_of_dec_pic_buf;
	     ++n_loopc) {
		*p_buffer++ = (u32) (p_decoder->n_meta_data_offset + (u8 *)
				     p_decoder->dp_buf.
				     a_dec_pic_buffers[n_loopc].vcd_frm.
				     p_physical);
	}
}

void ddl_process_encoder_metadata(struct ddl_client_context_type *p_ddl)
{
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);
	struct vcd_frame_data_type *p_out_frame =
	    &(p_ddl->output_frame.vcd_frm);
	u32 *p_qfiller_hdr, *p_qfiller, n_start_addr;
	u32 n_qfiller_size;

	if (!p_encoder->n_meta_data_enable_flag) {
		p_out_frame->n_flags &= ~(VCD_FRAME_FLAG_EXTRADATA);
		return;
	}

	if (!p_encoder->enc_frame_info.n_metadata_exists) {
		p_out_frame->n_flags &= ~(VCD_FRAME_FLAG_EXTRADATA);
		return;
	}
	p_out_frame->n_flags |= VCD_FRAME_FLAG_EXTRADATA;

	n_start_addr = (u32) ((u8 *) p_out_frame->p_virtual +
			      p_out_frame->n_offset);
	p_qfiller = (u32 *) ((p_out_frame->n_data_len + n_start_addr + 3) & ~3);

	n_qfiller_size = (u32) ((p_encoder->n_meta_data_offset +
				 (u8 *) p_out_frame->p_virtual) -
				(u8 *) p_qfiller);

	p_qfiller_hdr = ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_QCOMFILLER);

	*p_qfiller++ = n_qfiller_size;
	*p_qfiller++ = p_qfiller_hdr[DDL_METADATA_HDR_VERSION_INDEX];
	*p_qfiller++ = p_qfiller_hdr[DDL_METADATA_HDR_PORT_INDEX];
	*p_qfiller++ = p_qfiller_hdr[DDL_METADATA_HDR_TYPE_INDEX];
	*p_qfiller = (u32) (n_qfiller_size - DDL_METADATA_HDR_SIZE);
}

void ddl_process_decoder_metadata(struct ddl_client_context_type *p_ddl)
{
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vcd_frame_data_type *p_output_frame =
	    &(p_ddl->output_frame.vcd_frm);
	u32 *p_qfiller_hdr, *p_qfiller;
	u32 n_qfiller_size;

	if (!p_decoder->n_meta_data_enable_flag) {
		p_output_frame->n_flags &= ~(VCD_FRAME_FLAG_EXTRADATA);
		return;
	}

	if (!p_decoder->dec_disp_info.n_metadata_exists) {
		p_output_frame->n_flags &= ~(VCD_FRAME_FLAG_EXTRADATA);
		return;
	}
	p_output_frame->n_flags |= VCD_FRAME_FLAG_EXTRADATA;

	if (p_output_frame->n_data_len != p_decoder->n_meta_data_offset) {
		p_qfiller = (u32 *) ((u32) ((p_output_frame->n_data_len +
					     p_output_frame->n_offset +
					     (u8 *) p_output_frame->p_virtual) +
					    3) & ~3);

		n_qfiller_size = (u32) ((p_decoder->n_meta_data_offset +
					 (u8 *) p_output_frame->p_virtual) -
					(u8 *) p_qfiller);

		p_qfiller_hdr =
		    ddl_metadata_hdr_entry(p_ddl, VCD_METADATA_QCOMFILLER);
		*p_qfiller++ = n_qfiller_size;
		*p_qfiller++ = p_qfiller_hdr[DDL_METADATA_HDR_VERSION_INDEX];
		*p_qfiller++ = p_qfiller_hdr[DDL_METADATA_HDR_PORT_INDEX];
		*p_qfiller++ = p_qfiller_hdr[DDL_METADATA_HDR_TYPE_INDEX];
		*p_qfiller = (u32) (n_qfiller_size - DDL_METADATA_HDR_SIZE);
	}
}
