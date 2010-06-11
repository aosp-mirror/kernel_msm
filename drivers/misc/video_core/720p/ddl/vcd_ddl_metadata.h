/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _VCD_DDL_METADATA_H_
#define _VCD_DDL_METADATA_H_

#define DDL_MAX_DEC_METADATATYPE  (8)
#define DDL_MAX_ENC_METADATATYPE  (3)

#define DDL_METADATA_EXTRAPAD_SIZE (256)
#define DDL_METADATA_HDR_SIZE (20)

#define DDL_METADATA_EXTRADATANONE_SIZE (24)

#define DDL_METADATA_ALIGNSIZE(x) ((x) = (((x) + 0x7) & ~0x7))

#define DDL_METADATA_MANDATORY (VCD_METADATA_DATANONE | VCD_METADATA_QCOMFILLER)

#define DDL_METADATA_VC1_PAYLOAD_SIZE (38*4)

#define DDL_METADATA_SEI_PAYLOAD_SIZE (100)
#define DDL_METADATA_SEI_MAX (5)

#define DDL_METADATA_VUI_PAYLOAD_SIZE (256)

#define DDL_METADATA_PASSTHROUGH_PAYLOAD_SIZE  (68)

#define DDL_METADATA_CLIENT_INPUTBUFSIZE  (256)
#define DDL_METADATA_TOTAL_INPUTBUFSIZE \
	(DDL_METADATA_CLIENT_INPUTBUFSIZE * VCD_MAX_NO_CLIENT)

#define DDL_METADATA_HDR_VERSION_INDEX 0
#define DDL_METADATA_HDR_PORT_INDEX    1
#define DDL_METADATA_HDR_TYPE_INDEX    2


void ddl_set_default_meta_data_hdr(struct ddl_client_context *ddl);
u32 ddl_get_metadata_params(struct ddl_client_context	*ddl,
	struct vcd_property_hdr *property_hdr, void *property_value);
u32 ddl_set_metadata_params(struct ddl_client_context *ddl,
	struct vcd_property_hdr *property_hdr, void *property_value);
void ddl_set_default_metadata_flag(struct ddl_client_context *ddl);
void ddl_set_default_decoder_metadata_buffer_size(struct ddl_decoder_data *dec,
	struct vcd_property_frame_size *frame_size,
	struct vcd_buffer_requirement *output_buf_req);
void ddl_set_default_encoder_metadata_buffer_size(struct ddl_encoder_data *enc);
void ddl_metadata_enable(struct ddl_client_context *ddl);
phys_addr_t ddl_encode_set_metadata_output_buf(struct ddl_client_context *ddl);
void ddl_decode_set_metadata_output(struct ddl_decoder_data *decoder);
void ddl_process_encoder_metadata(struct ddl_client_context *ddl);
void ddl_process_decoder_metadata(struct ddl_client_context *ddl);

#endif
