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

static u32 *ddl_metadata_hdr_entry(struct ddl_client_context *ddl,
	u32 meta_data_type)
{
	u32 skip_words;
	u32 *buffer;

	if (ddl->decoding) {
		buffer = ddl->codec_data.decoder.meta_data_input.virt_addr;
		skip_words = 32 + 1;
		buffer += skip_words;

		switch (meta_data_type) {
		default:
		case VCD_METADATA_DATANONE:
			skip_words = 0;
			break;
		case VCD_METADATA_QPARRAY:
			skip_words = 3;
			break;
		case VCD_METADATA_CONCEALMB:
			skip_words = 6;
			break;
		case VCD_METADATA_VC1:
			skip_words = 9;
			break;
		case VCD_METADATA_SEI:
			skip_words = 12;
			break;
		case VCD_METADATA_VUI:
			skip_words = 15;
			break;
		case VCD_METADATA_PASSTHROUGH:
			skip_words = 18;
			break;
		case VCD_METADATA_QCOMFILLER:
			skip_words = 21;
			break;
		}
	} else {
		buffer = ddl->codec_data.encoder.meta_data_input.virt_addr;
		skip_words = 2;
		buffer += skip_words;

		switch (meta_data_type) {
		default:
		case VCD_METADATA_DATANONE:
			skip_words = 0;
			break;
		case VCD_METADATA_ENC_SLICE:
			skip_words = 3;
			break;
		case VCD_METADATA_QCOMFILLER:
			skip_words = 6;
			break;
		}
	}

	buffer += skip_words;
	return buffer;
}

void ddl_set_default_meta_data_hdr(struct ddl_client_context *ddl)
{
	struct ddl_dma_buffer *main_buffer =
		&ddl->ddl_context->metadata_shared_input;
	struct ddl_dma_buffer *b;
	u32 *hdr;

	if (ddl->decoding)
		b = &ddl->codec_data.decoder.meta_data_input;
	else
		b = &ddl->codec_data.encoder.meta_data_input;

	b->phys_addr = main_buffer->phys_addr +
		DDL_METADATA_CLIENT_INPUTBUFSIZE * ddl->channel_id;
	b->virt_addr = (void *)((u8 *)main_buffer->virt_addr +
		DDL_METADATA_CLIENT_INPUTBUFSIZE * ddl->channel_id);

	hdr = ddl_metadata_hdr_entry(ddl, VCD_METADATA_QCOMFILLER);
	hdr[DDL_METADATA_HDR_VERSION_INDEX] = 1;
	hdr[DDL_METADATA_HDR_PORT_INDEX] = 1;
	hdr[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_QCOMFILLER;

	hdr = ddl_metadata_hdr_entry(ddl, VCD_METADATA_DATANONE);
	hdr[DDL_METADATA_HDR_VERSION_INDEX] = 2;
	hdr[DDL_METADATA_HDR_PORT_INDEX] = 2;
	hdr[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_DATANONE;

	if (ddl->decoding) {
		hdr = ddl_metadata_hdr_entry(ddl, VCD_METADATA_QPARRAY);
		hdr[DDL_METADATA_HDR_VERSION_INDEX] = 3;
		hdr[DDL_METADATA_HDR_PORT_INDEX] = 3;
		hdr[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_QPARRAY;

		hdr = ddl_metadata_hdr_entry(ddl, VCD_METADATA_CONCEALMB);
		hdr[DDL_METADATA_HDR_VERSION_INDEX] = 4;
		hdr[DDL_METADATA_HDR_PORT_INDEX] = 4;
		hdr[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_CONCEALMB;

		hdr = ddl_metadata_hdr_entry(ddl, VCD_METADATA_SEI);
		hdr[DDL_METADATA_HDR_VERSION_INDEX] = 5;
		hdr[DDL_METADATA_HDR_PORT_INDEX] = 5;
		hdr[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_SEI;

		hdr = ddl_metadata_hdr_entry(ddl, VCD_METADATA_VUI);
		hdr[DDL_METADATA_HDR_VERSION_INDEX] = 6;
		hdr[DDL_METADATA_HDR_PORT_INDEX] = 6;
		hdr[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_VUI;

		hdr = ddl_metadata_hdr_entry(ddl, VCD_METADATA_VC1);
		hdr[DDL_METADATA_HDR_VERSION_INDEX] = 7;
		hdr[DDL_METADATA_HDR_PORT_INDEX] = 7;
		hdr[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_VC1;

		hdr = ddl_metadata_hdr_entry(ddl, VCD_METADATA_PASSTHROUGH);
		hdr[DDL_METADATA_HDR_VERSION_INDEX] = 8;
		hdr[DDL_METADATA_HDR_PORT_INDEX] = 8;
		hdr[DDL_METADATA_HDR_TYPE_INDEX] =
			VCD_METADATA_PASSTHROUGH;

	} else {
		hdr = ddl_metadata_hdr_entry(ddl, VCD_METADATA_ENC_SLICE);
		hdr[DDL_METADATA_HDR_VERSION_INDEX] = 9;
		hdr[DDL_METADATA_HDR_PORT_INDEX] = 9;
		hdr[DDL_METADATA_HDR_TYPE_INDEX] = VCD_METADATA_ENC_SLICE;
	}
}

static u32 ddl_supported_metadata_flag(struct ddl_client_context *ddl)
{
	u32 flag = 0;

	if (ddl->decoding) {
		enum vcd_codec codec =
			ddl->codec_data.decoder.codec_type.codec;

		flag |= VCD_METADATA_CONCEALMB | VCD_METADATA_PASSTHROUGH |
				VCD_METADATA_QPARRAY;
		if (codec == VCD_CODEC_H264)
			flag |= VCD_METADATA_SEI | VCD_METADATA_VUI;
		else if (codec == VCD_CODEC_VC1 || codec == VCD_CODEC_VC1_RCV)
			flag |= VCD_METADATA_VC1;
	} else {
		flag |= VCD_METADATA_ENC_SLICE;
	}

	return flag;
}

void ddl_set_default_metadata_flag(struct ddl_client_context *ddl)
{
	if (ddl->decoding)
		ddl->codec_data.decoder.meta_data_enable_flag = 0;
	else
		ddl->codec_data.encoder.meta_data_enable_flag = 0;
}

void ddl_set_default_decoder_metadata_buffer_size(struct ddl_decoder_data *dec,
	struct vcd_property_frame_size *frame_size,
	struct vcd_buffer_requirement *output_buf_req)
{
	u32 flag = dec->meta_data_enable_flag;
	u32 suffix = 0;
	u32 size = 0;

	if (!flag) {
		dec->suffix = 0;
		return;
	}

	if (flag & VCD_METADATA_QPARRAY) {
		u32 num_of_mb = ((frame_size->width * frame_size->height) >> 8);
		size = DDL_METADATA_HDR_SIZE;
		size += num_of_mb;
		DDL_METADATA_ALIGNSIZE(size);
		suffix += size;
	}
	if (flag & VCD_METADATA_CONCEALMB) {
		u32 num_of_mb = ((frame_size->width * frame_size->height) >> 8);
		size = DDL_METADATA_HDR_SIZE;
		size *= (4 * num_of_mb / 2);
		DDL_METADATA_ALIGNSIZE(size);
		suffix += size;
	}
	if (flag & VCD_METADATA_VC1) {
		size = DDL_METADATA_HDR_SIZE;
		size += DDL_METADATA_VC1_PAYLOAD_SIZE;
		DDL_METADATA_ALIGNSIZE(size);
		suffix += size;
	}
	if (flag & VCD_METADATA_SEI) {
		size = DDL_METADATA_HDR_SIZE;
		size += DDL_METADATA_SEI_PAYLOAD_SIZE;
		DDL_METADATA_ALIGNSIZE(size);
		suffix += (size * DDL_METADATA_SEI_MAX);
	}
	if (flag & VCD_METADATA_VUI) {
		size = DDL_METADATA_HDR_SIZE;
		size += DDL_METADATA_VUI_PAYLOAD_SIZE;
		DDL_METADATA_ALIGNSIZE(size);
		suffix += (size);
	}
	if (flag & VCD_METADATA_PASSTHROUGH) {
		size = DDL_METADATA_HDR_SIZE;
		size += DDL_METADATA_PASSTHROUGH_PAYLOAD_SIZE;
		DDL_METADATA_ALIGNSIZE(size);
		suffix += (size);
	}
	size = DDL_METADATA_EXTRADATANONE_SIZE;
	DDL_METADATA_ALIGNSIZE(size);
	suffix += (size);

	suffix += DDL_METADATA_EXTRAPAD_SIZE;
	DDL_METADATA_ALIGNSIZE(suffix);

	dec->suffix = suffix;
	output_buf_req->size += suffix;
	return;
}

void ddl_set_default_encoder_metadata_buffer_size(struct ddl_encoder_data *enc)
{
	u32 flag = enc->meta_data_enable_flag;
	u32 suffix = 0;
	u32 size = 0;

	if (!flag) {
		enc->suffix = 0;
		return;
	}

	if (flag & VCD_METADATA_ENC_SLICE) {
		u32 num_of_mb = enc->frame_size.width * enc->frame_size.height /
			16 / 16;
		size = DDL_METADATA_HDR_SIZE + 4 + 8 * num_of_mb;
		DDL_METADATA_ALIGNSIZE(size);
		suffix += size;
	}

	size = DDL_METADATA_EXTRADATANONE_SIZE;
	DDL_METADATA_ALIGNSIZE(size);
	suffix += (size);

	suffix += DDL_METADATA_EXTRAPAD_SIZE;
	DDL_METADATA_ALIGNSIZE(suffix);

	enc->suffix = suffix;
	enc->output_buf_req.size += suffix;
}

static u32 ddl_set_metadata_enable_client_open(struct ddl_client_context *ddl,
	struct vcd_property_meta_data_enable *meta_data_enable,
	u32 *meta_data_enable_flag)
{
	if (!meta_data_enable->meta_data_enable_flag) {
		*meta_data_enable_flag = 0;
		if (ddl->decoding) {
			ddl_set_default_decoder_buffer_req(
				&ddl->codec_data.decoder, true);
		} else {
			ddl_set_default_encoder_buffer_req(
				&ddl->codec_data.encoder);
		}

	} else {
		u32 flag = ddl_supported_metadata_flag(ddl);
		flag &= meta_data_enable->meta_data_enable_flag;
		if (flag) {
			flag |= DDL_METADATA_MANDATORY;
			if (flag != *meta_data_enable_flag) {
				*meta_data_enable_flag = flag;

				if (ddl->decoding) {
					ddl_set_default_decoder_buffer_req(
						&ddl->codec_data.decoder, true);
				} else {
					ddl_set_default_encoder_buffer_req(
						&ddl->codec_data.encoder);
				}

			}
		}
	}
	return VCD_S_SUCCESS;
}

static u32 ddl_set_metadata_header(struct ddl_client_context *ddl,
	struct vcd_property_hdr *property_hdr,
	struct vcd_property_metadata_hdr *hdr)
{
	u32 flag;
	if (sizeof(struct vcd_property_metadata_hdr) != property_hdr->sz)
		return VCD_ERR_ILLEGAL_PARM;

	flag = ddl_supported_metadata_flag(ddl);
	flag |= DDL_METADATA_MANDATORY;
	flag &= hdr->meta_data_id_type;
	if (!(flag & (flag - 1))) {
		u32 *hdr_entry = ddl_metadata_hdr_entry(ddl, flag);
		hdr_entry[DDL_METADATA_HDR_VERSION_INDEX] = hdr->version;
		hdr_entry[DDL_METADATA_HDR_PORT_INDEX] = hdr->port_index;
		hdr_entry[DDL_METADATA_HDR_TYPE_INDEX] = hdr->type;
		return VCD_S_SUCCESS;
	}
	return VCD_ERR_ILLEGAL_PARM;
}

u32 ddl_set_metadata_params(struct ddl_client_context *ddl,
	struct vcd_property_hdr *prop, void *value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	if (prop->id == VCD_I_METADATA_ENABLE) {
		struct vcd_property_meta_data_enable *meta_data_enable =
			(struct vcd_property_meta_data_enable *)value;
		u32 *meta_data_enable_flag;
		enum vcd_codec codec;
		if (ddl->decoding) {
			meta_data_enable_flag = &ddl->codec_data.decoder.
				meta_data_enable_flag;
			codec = ddl->codec_data.decoder.codec_type.codec;
		} else {
			meta_data_enable_flag = &ddl->codec_data.encoder.
				meta_data_enable_flag;
			codec = ddl->codec_data.encoder.codec_type.codec;
		}
		if (sizeof(struct vcd_property_meta_data_enable) == prop->sz &&
				DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN) &&
				codec) {
			vcd_status = ddl_set_metadata_enable_client_open(ddl,
				meta_data_enable, meta_data_enable_flag);
		}
	} else if (prop->id == VCD_I_METADATA_HEADER) {
		vcd_status = ddl_set_metadata_header(ddl, prop, value);
	}
	return vcd_status;
}

u32 ddl_get_metadata_params(struct ddl_client_context *ddl,
	struct vcd_property_hdr *prop, void *value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct vcd_property_meta_data_enable *enable;
	struct vcd_property_metadata_hdr *hdr;

	if (prop->id == VCD_I_METADATA_ENABLE && prop->sz == sizeof(*enable)) {
		enable = value;
		enable->meta_data_enable_flag = ddl->decoding ?
			ddl->codec_data.decoder.meta_data_enable_flag :
			ddl->codec_data.encoder.meta_data_enable_flag;
		vcd_status = VCD_S_SUCCESS;
	} else if (prop->id == VCD_I_METADATA_HEADER &&
			sizeof(*hdr) == prop->sz) {
		u32 flag = ddl_supported_metadata_flag(ddl);
		hdr = value;
		flag |= DDL_METADATA_MANDATORY;
		flag &= hdr->meta_data_id_type;
		if (!(flag & (flag - 1))) {
			u32 *hdr_entry = ddl_metadata_hdr_entry(ddl, flag);
			hdr->version =
				hdr_entry[DDL_METADATA_HDR_VERSION_INDEX];
			hdr->port_index =
				hdr_entry[DDL_METADATA_HDR_PORT_INDEX];
			hdr->type = hdr_entry[DDL_METADATA_HDR_TYPE_INDEX];
			vcd_status = VCD_S_SUCCESS;
		}
	}
	return vcd_status;
}

void ddl_metadata_enable(struct ddl_client_context *ddl)
{
	u32 flag, hal_flag = 0;
	phys_addr_t input;
	if (ddl->decoding) {
		flag = ddl->codec_data.decoder.meta_data_enable_flag;
		input = ddl->codec_data.decoder.meta_data_input.phys_addr;
	} else {
		flag = ddl->codec_data.encoder.meta_data_enable_flag;
		input = ddl->codec_data.encoder.meta_data_input.phys_addr;
	}
	if (flag) {
		if (flag & VCD_METADATA_QPARRAY)
			hal_flag |= VIDC_720P_METADATA_ENABLE_QP;
		if (flag & VCD_METADATA_CONCEALMB)
			hal_flag |= VIDC_720P_METADATA_ENABLE_CONCEALMB;
		if (flag & VCD_METADATA_VC1)
			hal_flag |= VIDC_720P_METADATA_ENABLE_VC1;
		if (flag & VCD_METADATA_SEI)
			hal_flag |= VIDC_720P_METADATA_ENABLE_SEI;
		if (flag & VCD_METADATA_VUI)
			hal_flag |= VIDC_720P_METADATA_ENABLE_VUI;
		if (flag & VCD_METADATA_ENC_SLICE)
			hal_flag |= VIDC_720P_METADATA_ENABLE_ENCSLICE;
		if (flag & VCD_METADATA_PASSTHROUGH)
			hal_flag |= VIDC_720P_METADATA_ENABLE_PASSTHROUGH;
	} else {
		input = 0;
	}
	vidc_720p_metadata_enable(hal_flag, input);
}

phys_addr_t ddl_encode_set_metadata_output_buf(struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *encoder = &ddl->codec_data.encoder;
	u32 *buffer;
	struct vcd_frame_data *stream = &(ddl->output_frame.vcd_frm);
	phys_addr_t ext_buffer_end;
	phys_addr_t hw_metadata_start;

	ext_buffer_end = stream->phys_addr + stream->alloc_len;
	if (!encoder->meta_data_enable_flag) {
		ext_buffer_end &= ~DDL_STREAMBUF_ALIGN_GUARD_BYTES;
		return ext_buffer_end;
	}
	hw_metadata_start = (ext_buffer_end - encoder->suffix) &
		~DDL_STREAMBUF_ALIGN_GUARD_BYTES;

	ext_buffer_end = (hw_metadata_start - 1) &
		~DDL_STREAMBUF_ALIGN_GUARD_BYTES;

	buffer = encoder->meta_data_input.virt_addr;

	*buffer++ = encoder->suffix;

	*buffer = hw_metadata_start;

	encoder->meta_data_offset = hw_metadata_start - stream->phys_addr;

	return ext_buffer_end;
}

void ddl_decode_set_metadata_output(struct ddl_decoder_data *dec)
{
	int i;
	u32 *buffer;

	if (!dec->meta_data_enable_flag) {
		dec->meta_data_offset = 0;
		return;
	}

	dec->meta_data_offset = ddl_get_yuv_buffer_size(&dec->client_frame_size,
		&dec->buf_format, !dec->progressive_only);

	buffer = dec->meta_data_input.virt_addr;

	*buffer++ = dec->suffix;

	for (i = 0; i < dec->dp_buf.no_of_dec_pic_buf; ++i)
		*buffer++ = dec->dp_buf.dec_pic_buffers[i].vcd_frm.phys_addr +
			dec->meta_data_offset;
}

//TOOD consider combining ddl_process_xxx_metadata
void ddl_process_encoder_metadata(struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *enc = &ddl->codec_data.encoder;
	struct vcd_frame_data *frm = &ddl->output_frame.vcd_frm;
	u32 *qfill_hdr;
	u32 *qfill;
	unsigned long tmp;
	size_t qfill_sz;

	if (!enc->meta_data_enable_flag) {
		frm->flags &= ~VCD_FRAME_FLAG_EXTRADATA;
		return;
	}

	if (!enc->enc_frame_info.metadata_exists) {
		frm->flags &= ~VCD_FRAME_FLAG_EXTRADATA;
		return;
	}
	frm->flags |= VCD_FRAME_FLAG_EXTRADATA;

	tmp = (unsigned long)frm->virt_addr + frm->offset + frm->data_len;
	qfill = (u32 *)ALIGN(tmp, 4);

	qfill_sz = enc->meta_data_offset + (u8 *)frm->virt_addr - (u8 *)qfill;

	qfill_hdr = ddl_metadata_hdr_entry(ddl, VCD_METADATA_QCOMFILLER);

	*qfill++ = qfill_sz;
	*qfill++ = qfill_hdr[DDL_METADATA_HDR_VERSION_INDEX];
	*qfill++ = qfill_hdr[DDL_METADATA_HDR_PORT_INDEX];
	*qfill++ = qfill_hdr[DDL_METADATA_HDR_TYPE_INDEX];
	*qfill = qfill_sz - DDL_METADATA_HDR_SIZE;
}

void ddl_process_decoder_metadata(struct ddl_client_context *ddl)
{
	struct ddl_decoder_data *dec = &ddl->codec_data.decoder;
	struct vcd_frame_data *frm = &ddl->output_frame.vcd_frm;
	u32 *qfill_hdr;
	u32 *qfill;
	size_t qfill_sz;
	unsigned long tmp;

	if (!dec->meta_data_enable_flag) {
		frm->flags &= ~VCD_FRAME_FLAG_EXTRADATA;
		return;
	}
	if (!dec->dec_disp_info.metadata_exists) {
		frm->flags &= ~VCD_FRAME_FLAG_EXTRADATA;
		return;
	}
	frm->flags |= VCD_FRAME_FLAG_EXTRADATA;

	if (frm->data_len == dec->meta_data_offset)
		return;

	tmp = (unsigned long)frm->virt_addr + frm->offset + frm->data_len;
	qfill = (u32 *)ALIGN(tmp, 4);

	qfill_sz = dec->meta_data_offset + (u8 *)frm->virt_addr - (u8 *)qfill;

	qfill_hdr = ddl_metadata_hdr_entry(ddl,	VCD_METADATA_QCOMFILLER);

	*qfill++ = qfill_sz;
	*qfill++ = qfill_hdr[DDL_METADATA_HDR_VERSION_INDEX];
	*qfill++ = qfill_hdr[DDL_METADATA_HDR_PORT_INDEX];
	*qfill++ = qfill_hdr[DDL_METADATA_HDR_TYPE_INDEX];
	*qfill = qfill_sz - DDL_METADATA_HDR_SIZE;
}
