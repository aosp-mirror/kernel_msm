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
#ifndef VID_C_720P_H
#define VID_C_720P_H

#include <linux/io.h>
#include <linux/delay.h>
#include <asm/system.h>

/** List all the levels and their register values */

#define VIDC_720P_PROFILE_MPEG4_SP      0
#define VIDC_720P_PROFILE_MPEG4_ASP     1
#define VIDC_720P_PROFILE_H264_BASELINE 0
#define VIDC_720P_PROFILE_H264_MAIN     1
#define VIDC_720P_PROFILE_H264_HIGH     2
#define VIDC_720P_PROFILE_H263_BASELINE 0

#define VIDC_720P_PROFILE_VC1_SP        0
#define VIDC_720P_PROFILE_VC1_MAIN      1
#define VIDC_720P_PROFILE_VC1_ADV       2
#define VIDC_720P_PROFILE_MPEG2_MAIN    4
#define VIDC_720P_PROFILE_MPEG2_SP      5

#define VIDC_720P_MPEG4_LEVEL0  0
#define VIDC_720P_MPEG4_LEVEL0b 9
#define VIDC_720P_MPEG4_LEVEL1  1
#define VIDC_720P_MPEG4_LEVEL2  2
#define VIDC_720P_MPEG4_LEVEL3  3
#define VIDC_720P_MPEG4_LEVEL3b 7
#define VIDC_720P_MPEG4_LEVEL4a 4
#define VIDC_720P_MPEG4_LEVEL5  5
#define VIDC_720P_MPEG4_LEVEL6  6

#define VIDC_720P_H264_LEVEL1     10
#define VIDC_720P_H264_LEVEL1b    9
#define VIDC_720P_H264_LEVEL1p1   11
#define VIDC_720P_H264_LEVEL1p2   12
#define VIDC_720P_H264_LEVEL1p3   13
#define VIDC_720P_H264_LEVEL2     20
#define VIDC_720P_H264_LEVEL2p1   21
#define VIDC_720P_H264_LEVEL2p2   22
#define VIDC_720P_H264_LEVEL3     30
#define VIDC_720P_H264_LEVEL3p1   31
#define VIDC_720P_H264_LEVEL3p2   32

#define VIDC_720P_H263_LEVEL10    10
#define VIDC_720P_H263_LEVEL20    20
#define VIDC_720P_H263_LEVEL30    30
#define VIDC_720P_H263_LEVEL40    40
#define VIDC_720P_H263_LEVEL45    45
#define VIDC_720P_H263_LEVEL50    50
#define VIDC_720P_H263_LEVEL60    60
#define VIDC_720P_H263_LEVEL70    70

#define VIDC_720P_VC1_LEVEL_LOW    0
#define VIDC_720P_VC1_LEVEL_MED    2
#define VIDC_720P_VC1_LEVEL_HIGH   4
#define VIDC_720P_VC1_LEVEL0       0
#define VIDC_720P_VC1_LEVEL1       1
#define VIDC_720P_VC1_LEVEL2       2
#define VIDC_720P_VC1_LEVEL3       3
#define VIDC_720P_VC1_LEVEL4       4

#define VIDCL_720P_MPEG2_LEVEL_LOW 10
#define VIDCL_720P_MPEG2_LEVEL_MAIN 8
#define VIDCL_720P_MPEG2_LEVEL_HIGH14 6

#define VIDC_720P_CMD_CHSET               0x0
#define VIDC_720P_CMD_CHEND               0x2
#define VIDC_720P_CMD_INITCODEC           0x3
#define VIDC_720P_CMD_FRAMERUN            0x4
#define VIDC_720P_CMD_INITBUFFERS         0x5
#define VIDC_720P_CMD_FRAMERUN_REALLOCATE 0x6
#define VIDC_720P_CMD_MFC_ENGINE_RESET 0x7

enum vidc_720p_endian_type {
	VIDC_720P_BIG_ENDIAN = 0x0,
	VIDC_720P_LITTLE_ENDIAN = 0x1
};

enum vidc_720p_memory_access_method_type {
	VIDC_720P_TILE_LINEAR = 0,
	VIDC_720P_TILE_16x16 = 2,
	VIDC_720P_TILE_64x32 = 3
};

enum vidc_720p_interrupt_control_mode_type {
	VIDC_720P_INTERRUPT_MODE = 0,
	VIDC_720P_POLL_MODE = 1
};

enum vidc_720p_interrupt_level_selection_type {
	VIDC_720P_INTERRUPT_LEVEL_SEL = 0,
	VIDC_720P_INTERRUPT_PULSE_SEL = 1
};

#define VIDC_720P_INTR_BUFFER_FULL             0x002
#define VIDC_720P_INTR_FW_DONE                 0x020
#define VIDC_720P_INTR_HEADER_DONE             0x040
#define VIDC_720P_INTR_DMA_DONE                0x080
#define VIDC_720P_INTR_FRAME_DONE              0x100

enum vidc_720p_enc_dec_selection_type {
	VIDC_720P_DECODER = 0,
	VIDC_720P_ENCODER = 1
};

enum vidc_720p_codec_type {
	VIDC_720P_MPEG4 = 0,
	VIDC_720P_H264 = 1,
	VIDC_720P_DIVX = 2,
	VIDC_720P_XVID = 3,
	VIDC_720P_H263 = 4,
	VIDC_720P_MPEG2 = 5,
	VIDC_720P_VC1 = 6
};

enum vidc_720p_frame_type {
	VIDC_720P_NOTCODED = 0,
	VIDC_720P_IFRAME = 1,
	VIDC_720P_PFRAME = 2,
	VIDC_720P_BFRAME = 3
};

enum vidc_720p_entropy_sel_type {
	VIDC_720P_ENTROPY_SEL_CAVLC = 0,
	VIDC_720P_ENTROPY_SEL_CABAC = 1
};

enum vidc_720p_cabac_model_type {
	VIDC_720P_CABAC_MODEL_NUMBER_0 = 0,
	VIDC_720P_CABAC_MODEL_NUMBER_1 = 1,
	VIDC_720P_CABAC_MODEL_NUMBER_2 = 2
};

enum vidc_720p_DBConfig_type {
	VIDC_720P_DB_ALL_BLOCKING_BOUNDARY = 0,
	VIDC_720P_DB_DISABLE = 1,
	VIDC_720P_DB_SKIP_SLICE_BOUNDARY = 2
};

enum vidc_720p_MSlice_selection_type {
	VIDC_720P_MSLICE_BY_MB_COUNT = 0,
	VIDC_720P_MSLICE_BY_BYTE_COUNT = 1,
	VIDC_720P_MSLICE_BY_GOB = 2,
	VIDC_720P_MSLICE_OFF = 3
};

enum vidc_720p_display_status_type {
	VIDC_720P_DECODE_ONLY = 0,
	VIDC_720P_DECODE_AND_DISPLAY = 1,
	VIDC_720P_DISPLAY_ONLY = 2,
	VIDC_720P_EMPTY_BUFFER = 3
};

#define VIDC_720P_ENC_IFRAME_REQ       0x1
#define VIDC_720P_ENC_IPERIOD_CHANGE   0x1
#define VIDC_720P_ENC_FRAMERATE_CHANGE 0x2
#define VIDC_720P_ENC_BITRATE_CHANGE   0x4

#define VIDC_720P_FLUSH_REQ     0x1
#define VIDC_720P_EXTRADATA     0x2

#define VIDC_720P_METADATA_ENABLE_QP           0x01
#define VIDC_720P_METADATA_ENABLE_CONCEALMB    0x02
#define VIDC_720P_METADATA_ENABLE_VC1          0x04
#define VIDC_720P_METADATA_ENABLE_SEI          0x08
#define VIDC_720P_METADATA_ENABLE_VUI          0x10
#define VIDC_720P_METADATA_ENABLE_ENCSLICE     0x20
#define VIDC_720P_METADATA_ENABLE_PASSTHROUGH  0x40

struct vidc_720p_dec_disp_info {
	enum vidc_720p_display_status_type disp_status;
	u32 resl_change;
	u32 reconfig_flush_done;
	u32 img_size_x;
	u32 img_size_y;
	phys_addr_t y_addr;
	phys_addr_t c_addr;
	u32 tag_top;
	u32 pic_time_top;
	u32 disp_is_interlace;
	u32 tag_bottom;
	u32 pic_time_bottom;
	u32 metadata_exists;
	u32 crop_exists;
	u32 crop_right_offset;
	u32 crop_left_offset;
	u32 crop_bottom_offset;
	u32 crop_top_offset;
	u32 input_frame_type;
	u32 input_bytes_consumed;
	u32 input_is_interlace;
	u32 input_frame_num;
};

struct vidc_720p_seq_hdr_info_type {
	u32 img_size_x;
	u32 img_size_y;
	u32 dec_frm_size;
	u32 min_num_dpb;
	u32 min_dpb_size;
	u32 profile;
	u32 level;
	u32 progressive;
	u32 data_partitioned;
	u32 crop_exists;
	u32 crop_right_offset;
	u32 crop_left_offset;
	u32 crop_bottom_offset;
	u32 crop_top_offset;
};

struct vidc_720p_enc_frame_info {
	u32 enc_size;
	u32 frame_type;
	u32 metadata_exists;
};

void vidc_720p_set_device_virtual_base(void *virt_addr);

void vidc_720p_init(char **ppsz_version, size_t sz, phys_addr_t phys_addr,
	enum vidc_720p_endian_type dma_endian, u32 interrupt_off,
	enum vidc_720p_interrupt_level_selection_type interrupt_sel,
	u32 interrupt_mask);

u32 vidc_720p_do_sw_reset(void);

u32 vidc_720p_reset_is_success(void);

void vidc_720p_start_cpu(enum vidc_720p_endian_type dma_endian,
	phys_addr_t icontext_bufferstart, phys_addr_t debug_core_dump_addr,
	size_t debug_buffer_size);

u32 vidc_720p_cpu_start(void);

void vidc_720p_stop_fw(void);

void vidc_720p_get_interrupt_status(u32 *interrupt_status, u32 *cmd_err_status,
	u32 *disp_pic_err_status, u32 *op_failed);

void vidc_720p_interrupt_done_clear(void);

void vidc_720p_submit_command(u32 ch_id, u32 cmd_id);


void vidc_720p_set_channel(u32 ch_id,
	enum vidc_720p_enc_dec_selection_type enc_dec_sel,
	enum vidc_720p_codec_type codec, dma_addr_t pi_fw,
	size_t firmware_size);

u32 vidc_720p_engine_reset(u32 ch_id,
	enum vidc_720p_endian_type dma_endian,
	enum vidc_720p_interrupt_level_selection_type interrupt_sel,
	u32 interrupt_mask
);

void vidc_720p_encode_set_profile(u32 profile, u32 level);

void vidc_720p_set_frame_size(u32 size_x, u32 size_y);

void vidc_720p_encode_set_fps(u32 rc_frame_rate);

void vidc_720p_encode_set_vop_time(u32 vop_time_resolution,
	u32 vop_time_increment);

void vidc_720p_encode_set_hec_period(u32 hec_period);

void vidc_720p_encode_set_short_header(u32 short_header);

void vidc_720p_encode_set_qp_params(u32 max_qp, u32 min_qp);

void vidc_720p_encode_set_rc_config(u32 enable_frame_level_rc,
	u32 enable_mb_level_rc_flag, u32 iframe_qp, u32 pframe_qp);

void vidc_720p_encode_set_bit_rate(u32 target_bitrate);

void vidc_720p_encoder_set_param_change(u32 enc_param_change);

void vidc_720p_encode_set_control_param(u32 param_val);

void vidc_720p_encode_set_frame_level_rc_params(u32 reaction_coeff);

void vidc_720p_encode_set_mb_level_rc_params(u32 dark_region_as_flag,
	u32 smooth_region_as_flag, u32 static_region_as_flag,
	u32 activity_region_flag);

void vidc_720p_encode_set_entropy_control(enum vidc_720p_entropy_sel_type
	entropy_sel, enum vidc_720p_cabac_model_type cabac_model_number);

void vidc_720p_encode_set_db_filter_control(enum vidc_720p_DBConfig_type
	db_config, u32 slice_alpha_offset, u32 slice_beta_offset);

void vidc_720p_encode_set_intra_refresh_mb_number(u32 cir_mb_number);

void vidc_720p_encode_set_multi_slice_info(enum vidc_720p_MSlice_selection_type
	m_slice_sel, u32 multi_slice_size);

void vidc_720p_encode_set_dpb_buffer(phys_addr_t pi_enc_dpb_addr,
	size_t alloc_len);

void vidc_720p_set_deblock_line_buffer(phys_addr_t pi_deblock_line_buffer_start,
	size_t alloc_len);

void vidc_720p_encode_set_i_period(u32 period);

void vidc_720p_encode_init_codec(u32 ch_id,
	enum vidc_720p_memory_access_method_type memory_access_model);

void vidc_720p_encode_unalign_bitstream(u32 upper_unalign_word,
	u32 lower_unalign_word);

void vidc_720p_encode_set_seq_header_buffer(phys_addr_t ext_buffer_start,
	phys_addr_t ext_buffer_end, u32 start_byte_num);

void vidc_720p_encode_frame(u32 ch_id, phys_addr_t ext_buffer_start,
	phys_addr_t ext_buffer_end, u32 start_byte_number, phys_addr_t y_addr,
	phys_addr_t c_addr);

void vidc_720p_encode_get_header(u32 *pi_enc_header_size);

void vidc_720p_enc_frame_info(struct vidc_720p_enc_frame_info *enc_frame_info);

void vidc_720p_decode_bitstream_header(u32 ch_id, u32 dec_unit_size,
	u32 start_byte_num, u32 ext_buffer_start, u32 ext_buffer_end,
	enum vidc_720p_memory_access_method_type memory_access_model);

void vidc_720p_decode_get_seq_hdr_info(struct vidc_720p_seq_hdr_info_type
	*seq_hdr_info);

void vidc_720p_decode_set_dpb_release_buffer_mask(u32 dpb_release_buffer_mask);

void vidc_720p_decode_set_dpb_buffers(u32 buf_index, phys_addr_t pi_dpb_buffer);

void vidc_720p_decode_set_comv_buffer(dma_addr_t pi_dpb_comv_buffer,
	size_t alloc_len);

void vidc_720p_decode_set_dpb_details(u32 num_dpb, size_t alloc_len,
	phys_addr_t ref_buffer);

void vidc_720p_decode_set_mpeg4Post_filter(u32 enable_post_filter);

void vidc_720p_decode_set_error_control(u32 enable_error_control);

void vidc_720p_decode_set_mpeg4_data_partitionbuffer(dma_addr_t vsp_buf_start);

void vidc_720p_decode_setH264VSPBuffer(dma_addr_t pi_vsp_temp_buffer_start);

void vidc_720p_decode_frame(u32 ch_id, phys_addr_t ext_buffer_start,
	phys_addr_t ext_buffer_end, size_t dec_unit_size, u32 start_byte_num,
	u32 input_frame_tag);

void vidc_720p_issue_eos(u32 ch_id);
void vidc_720p_eos_info(u32 *disp_status);

void vidc_720p_decode_display_info(struct vidc_720p_dec_disp_info *disp_info);

void vidc_720p_decode_skip_frm_details(phys_addr_t *free_luma_dpb);

void vidc_720p_metadata_enable(u32 flag, phys_addr_t input_buffer);

void vidc_720p_decode_dynamic_req_reset(void);

void vidc_720p_decode_dynamic_req_set(u32 property);

void vidc_720p_decode_setpassthrough_start(phys_addr_t pass_startaddr);

#endif
