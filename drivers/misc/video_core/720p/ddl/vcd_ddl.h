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
#ifndef _VCD_DDL_H_
#define _VCD_DDL_H_
#include "vcd_ddl_api.h"
#include "vcd_ddl_utils.h"
#include "vcd_ddl_firmware.h"
#include "video_core_720p.h"

#undef DDL_INLINE
#define DDL_INLINE

#define DDL_BUSY_STATE 1
#define DDL_IDLE_STATE 0
#define DDL_ERROR_STATE 2
#define DDL_IS_BUSY(p_ddl_context) \
	(((p_ddl_context)->n_ddl_busy != DDL_IDLE_STATE))
#define DDL_BUSY(p_ddl_context) \
	((p_ddl_context)->n_ddl_busy = DDL_BUSY_STATE)
#define DDL_IDLE(p_ddl_context) \
	((p_ddl_context)->n_ddl_busy = DDL_IDLE_STATE)
#define DDL_ERROR(p_ddl_context) \
	((p_ddl_context)->n_ddl_busy = DDL_ERROR_STATE)

#define DDL_DEVICE_NOTINIT  0
#define DDL_DEVICE_INITED   1
#define DDL_DEVICE_HWFATAL  2
#define DDL_IS_INITIALIZED(p_ddl_context)  \
(p_ddl_context->n_device_state == DDL_DEVICE_INITED)

#define DDLCOMMAND_STATE_IS(p_ddl_context, command_state) \
(command_state == (p_ddl_context)->e_cmd_state)

#define DDLCLIENT_STATE_IS(p_ddl, client_state) \
(client_state == (p_ddl)->e_client_state)

#define DDL_DPB_OP_INIT       1
#define DDL_DPB_OP_MARK_FREE  2
#define DDL_DPB_OP_MARK_BUSY  3
#define DDL_DPB_OP_SET_MASK   4
#define DDL_DPB_OP_RETRIEVE   5

#define DDL_INIT_CLIENTS     0
#define DDL_GET_CLIENT       1
#define DDL_FREE_CLIENT      2
#define DDL_ACTIVE_CLIENT    3

#define DDL_INVALID_CHANNEL_ID  ((u32)~0)
#define DDL_INVALID_CODEC_TYPE ((u32)~0)
#define DDL_INVALID_INTR_STATUS ((u32)~0)

#define DDL_ENC_REQ_IFRAME                      0x1
#define DDL_ENC_CHANGE_IPERIOD                  0x2
#define DDL_ENC_CHANGE_BITRATE                  0x4
#define DDL_ENC_CHANGE_FRAMERATE                0x8

#define DDL_DEC_REQ_OUTPUT_FLUSH                0x1

struct ddl_buf_addr_type {
	u32 *p_physical_base_addr;
	u32 *p_virtual_base_addr;
	u32 *p_align_physical_addr;
	u32 *p_align_virtual_addr;
	u32 n_buffer_size;
};

enum ddl_cmd_state_type {
	DDL_CMD_INVALID = 0x0,
	DDL_CMD_DMA_INIT = 0x1,
	DDL_CMD_CPU_RESET = 0x2,
	DDL_CMD_CHANNEL_SET = 0x3,
	DDL_CMD_INIT_CODEC = 0x4,
	DDL_CMD_HEADER_PARSE = 0x5,
	DDL_CMD_DECODE_SET_DPB = 0x6,
	DDL_CMD_DECODE_FRAME = 0x7,
	DDL_CMD_ENCODE_FRAME = 0x8,
	DDL_CMD_EOS = 0x9,
	DDL_CMD_CHANNEL_END = 0xA,
	DDL_CMD_32BIT = 0x7FFFFFFF
};

enum ddl_client_state_type {
	DDL_CLIENT_INVALID = 0x0,
	DDL_CLIENT_OPEN = 0x1,
	DDL_CLIENT_WAIT_FOR_CHDONE = 0x2,
	DDL_CLIENT_WAIT_FOR_INITCODEC = 0x3,
	DDL_CLIENT_WAIT_FOR_INITCODECDONE = 0x4,
	DDL_CLIENT_WAIT_FOR_DPB = 0x5,
	DDL_CLIENT_WAIT_FOR_DPBDONE = 0x6,
	DDL_CLIENT_WAIT_FOR_FRAME = 0x7,
	DDL_CLIENT_WAIT_FOR_FRAME_DONE = 0x8,
	DDL_CLIENT_WAIT_FOR_EOS_DONE = 0x9,
	DDL_CLIENT_WAIT_FOR_CHEND = 0xA,
	DDL_CLIENT_FATAL_ERROR = 0xB,
	DDL_CLIENT_32BIT = 0x7FFFFFFF
};

struct ddl_mask_type {
	u32 n_client_mask;
	u32 n_hw_mask;
};

struct ddl_context_type;

struct ddl_client_context_type;

struct ddl_codec_data_hdr_type {
	u32 b_decoding;
};

struct ddl_encoder_data_type {
	struct ddl_codec_data_hdr_type hdr;
	struct vcd_property_codec_type codec_type;
	struct vcd_property_frame_size_type frame_size;
	struct vcd_property_frame_rate_type frame_rate;
	struct vcd_property_target_bitrate_type target_bit_rate;
	struct vcd_property_profile_type profile;
	struct vcd_property_level_type level;
	struct vcd_property_rate_control_type rc_type;
	struct vcd_property_multi_slice_type multi_slice;
	u32 n_meta_data_enable_flag;
	u32 n_suffix;
	struct ddl_buf_addr_type meta_data_input;
	u32 n_meta_data_offset;
	struct vcd_property_short_header_type short_header;
	struct vcd_property_vop_timing_type vop_timing;
	u32 n_hdr_ext_control;
	struct vcd_property_db_config_type db_control;
	struct vcd_property_entropy_control_type entropy_control;
	struct vcd_property_i_period_type i_period;
	struct vcd_property_session_qp_type session_qp;
	struct vcd_property_qp_range_type qp_range;
	struct vcd_property_rc_level_type rc_level;
	u32 n_r_cframe_skip;
	u32 n_vb_vbuffer_size;
	struct vcd_property_frame_level_rc_params_type frame_level_rc;
	struct vcd_property_adaptive_rc_params_type adaptive_rc;
	struct vcd_property_intra_refresh_mb_number_type intra_refresh;
	struct vcd_property_buffer_format_type buf_format;
	struct vcd_property_buffer_format_type re_con_buf_format;
	u32 n_dynamic_prop_change;
	u32 b_dynmic_prop_change_req;
	u32 n_ext_enc_control_val;
	struct vidc_720p_enc_frame_info_type enc_frame_info;
	struct ddl_buf_addr_type enc_dpb_addr;
	struct ddl_buf_addr_type seq_header;
	struct vcd_buffer_requirement_type input_buf_req;
	struct vcd_buffer_requirement_type output_buf_req;
	struct vcd_buffer_requirement_type client_input_buf_req;
	struct vcd_buffer_requirement_type client_output_buf_req;
};

struct ddl_decoder_data_type {
	struct ddl_codec_data_hdr_type hdr;
	struct vcd_property_codec_type codec_type;
	struct vcd_property_buffer_format_type buf_format;
	struct vcd_property_frame_size_type frame_size;
	struct vcd_property_frame_size_type client_frame_size;
	struct vcd_property_profile_type profile;
	struct vcd_property_level_type level;
	u32 n_progressive_only;
	u32 n_meta_data_enable_flag;
	u32 n_suffix;
	struct ddl_buf_addr_type meta_data_input;
	struct ddl_buf_addr_type ref_buffer;
	u32 n_meta_data_offset;
	struct vcd_property_post_filter_type post_filter;
	struct vcd_sequence_hdr_type decode_config;
	u32 b_header_in_start;
	u32 n_min_dpb_num;
	u32 n_y_cb_cr_size;
	struct ddl_property_dec_pic_buffers_type dp_buf;
	struct ddl_mask_type dpb_mask;
	u32 n_dynamic_prop_change;
	u32 b_dynmic_prop_change_req;
	struct vidc_720p_dec_disp_info_type dec_disp_info;
	struct ddl_buf_addr_type dpb_comv_buffer;
	struct ddl_buf_addr_type h264Vsp_temp_buffer;
	struct vcd_buffer_requirement_type actual_input_buf_req;
	struct vcd_buffer_requirement_type min_input_buf_req;
	struct vcd_buffer_requirement_type client_input_buf_req;
	struct vcd_buffer_requirement_type actual_output_buf_req;
	struct vcd_buffer_requirement_type min_output_buf_req;
	struct vcd_buffer_requirement_type client_output_buf_req;
};

union ddl_codec_data_type {
	struct ddl_codec_data_hdr_type hdr;
	struct ddl_decoder_data_type decoder;
	struct ddl_encoder_data_type encoder;
};

struct ddl_context_type {
	u8 *p_core_virtual_base_addr;
	void (*ddl_callback) (u32 event, u32 status, void *payload, u32 size,
			      u32 *p_ddl_handle, void *const p_client_data);
	void *p_client_data;
	void (*pf_interrupt_clr) (void);
	enum ddl_cmd_state_type e_cmd_state;
	struct ddl_client_context_type *p_current_ddl;
	struct ddl_buf_addr_type context_buf_addr;
	struct ddl_buf_addr_type db_line_buffer;
	struct ddl_buf_addr_type data_partition_tempbuf;
	struct ddl_buf_addr_type metadata_shared_input;
	struct ddl_buf_addr_type dbg_core_dump;
	u32 enable_dbg_core_dump;
	struct ddl_client_context_type *a_ddl_clients[VCD_MAX_NO_CLIENT];
	u32 n_device_state;
	u32 n_ddl_busy;
	u32  intr_status;
	u32 n_cmd_err_status;
	u32 n_disp_pic_err_status;
	u32 n_op_failed;
};

struct ddl_client_context_type {
	struct ddl_context_type *p_ddl_context;
	enum ddl_client_state_type e_client_state;
	u32 b_decoding;
	u32 n_channel_id;
	struct ddl_frame_data_type_tag input_frame;
	struct ddl_frame_data_type_tag output_frame;
	union ddl_codec_data_type codec_data;
};

DDL_INLINE struct ddl_context_type *ddl_get_context(void);
DDL_INLINE void ddl_move_command_state(struct ddl_context_type *p_ddl_context,
				       enum ddl_cmd_state_type e_command_state);
DDL_INLINE void ddl_move_client_state(struct ddl_client_context_type *p_ddl,
				      enum ddl_client_state_type client_state);
void ddl_core_init(struct ddl_context_type *);
void ddl_core_start_cpu(struct ddl_context_type *);
void ddl_channel_set(struct ddl_client_context_type *);
void ddl_channel_end(struct ddl_client_context_type *);
void ddl_encode_init_codec(struct ddl_client_context_type *);
void ddl_decode_init_codec(struct ddl_client_context_type *);
void ddl_encode_frame_run(struct ddl_client_context_type *);
void ddl_decode_frame_run(struct ddl_client_context_type *);
void  ddl_decode_eos_run(struct ddl_client_context_type *);
void ddl_release_context_buffers(struct ddl_context_type *);
void ddl_release_client_internal_buffers(struct ddl_client_context_type *p_ddl);
u32 ddl_decode_set_buffers(struct ddl_client_context_type *);
u32 ddl_decoder_dpb_transact(struct ddl_decoder_data_type *p_decoder,
			     struct ddl_frame_data_type_tag *p_in_out_frame,
			     u32 n_operation);
u32 ddl_client_transact(u32, struct ddl_client_context_type **);
void ddl_set_default_decoder_buffer_req
    (struct ddl_decoder_data_type *p_decoder, u32 b_estimate);
void ddl_set_default_encoder_buffer_req
    (struct ddl_encoder_data_type *p_encoder);
void ddl_set_default_dec_property(struct ddl_client_context_type *);
u32 ddl_encoder_ready_to_start(struct ddl_client_context_type *);
u32 ddl_decoder_ready_to_start(struct ddl_client_context_type *,
			       struct vcd_sequence_hdr_type *);
u32 ddl_get_yuv_buffer_size
    (struct vcd_property_frame_size_type *p_frame_size,
     struct vcd_property_buffer_format_type *p_buf_format, u32 inter_lace);
void ddl_calculate_stride(struct vcd_property_frame_size_type *p_frame_size,
						  u32 inter_lace);
void ddl_encode_dynamic_property(struct ddl_client_context_type *p_ddl,
				 u32 b_enable);
void ddl_decode_dynamic_property(struct ddl_client_context_type *p_ddl,
				 u32 b_enable);
void ddl_set_initial_default_values(struct ddl_client_context_type *p_ddl);
u32 ddl_handle_core_errors(struct ddl_context_type *p_ddl_context);
void ddl_client_fatal_cb(struct ddl_context_type *p_ddl_context);
void ddl_hw_fatal_cb(struct ddl_context_type *p_ddl_context);
u32 ddl_hal_engine_reset(struct ddl_context_type *p_ddl_context);
#endif
