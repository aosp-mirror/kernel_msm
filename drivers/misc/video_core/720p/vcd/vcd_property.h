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
#ifndef _VCD_DRIVER_PROPERTY_H_
#define _VCD_DRIVER_PROPERTY_H_

#define VCD_START_BASE       0x0
#define VCD_I_LIVE           (VCD_START_BASE + 0x1)
#define VCD_I_CODEC          (VCD_START_BASE + 0x2)
#define VCD_I_FRAME_SIZE     (VCD_START_BASE + 0x3)
#define VCD_I_METADATA_ENABLE  (VCD_START_BASE + 0x4)
#define VCD_I_METADATA_HEADER  (VCD_START_BASE + 0x5)
#define VCD_I_PROFILE        (VCD_START_BASE + 0x6)
#define VCD_I_LEVEL          (VCD_START_BASE + 0x7)
#define VCD_I_BUFFER_FORMAT  (VCD_START_BASE + 0x8)
#define VCD_I_FRAME_RATE  (VCD_START_BASE + 0x9)
#define VCD_I_TARGET_BITRATE (VCD_START_BASE + 0xA)
#define VCD_I_MULTI_SLICE    (VCD_START_BASE + 0xB)
#define VCD_I_ENTROPY_CTRL   (VCD_START_BASE + 0xC)
#define VCD_I_DEBLOCKING     (VCD_START_BASE + 0xD)
#define VCD_I_RATE_CONTROL   (VCD_START_BASE + 0xE)
#define VCD_I_QP_RANGE      (VCD_START_BASE + 0xF)
#define VCD_I_SESSION_QP    (VCD_START_BASE + 0x10)
#define VCD_I_INTRA_PERIOD   (VCD_START_BASE + 0x11)
#define VCD_I_VOP_TIMING     (VCD_START_BASE + 0x12)
#define VCD_I_SHORT_HEADER   (VCD_START_BASE + 0x13)
#define VCD_I_SEQ_HEADER    (VCD_START_BASE + 0x14)
#define VCD_I_HEADER_EXTENSION   (VCD_START_BASE + 0x15)
#define VCD_I_INTRA_REFRESH  (VCD_START_BASE + 0x16)
#define VCD_I_POST_FILTER    (VCD_START_BASE + 0x17)
#define VCD_I_PROGRESSIVE_ONLY (VCD_START_BASE + 0x18)

#define VCD_START_REQ      (VCD_START_BASE + 0x1000)
#define VCD_I_REQ_IFRAME   (VCD_START_REQ + 0x1)

#define VCD_I_RESERVED_BASE  (VCD_START_BASE + 0x10000)

struct vcd_property_hdr_type {
	u32    prop_id;
	u32    n_size;
};

struct vcd_property_live_type {
	u32             b_live;
};

enum vcd_codec_type {
	VCD_CODEC_H264      = 0x1,
	VCD_CODEC_H263      = 0x2,
	VCD_CODEC_MPEG1     = 0x3,
	VCD_CODEC_MPEG2     = 0x4,
	VCD_CODEC_MPEG4     = 0x5,
	VCD_CODEC_DIVX_3    = 0x6,
	VCD_CODEC_DIVX_4    = 0x7,
	VCD_CODEC_DIVX_5    = 0x8,
	VCD_CODEC_DIVX_6    = 0x9,
	VCD_CODEC_XVID      = 0xA,
	VCD_CODEC_VC1       = 0xB,
	VCD_CODEC_VC1_RCV   = 0xC
};

struct vcd_property_codec_type {
	enum vcd_codec_type       e_codec;
};

struct vcd_property_frame_size_type {
	u32              n_width;
	u32              n_height;
	u32              n_stride;
	u32              n_scan_lines;
};


#define VCD_METADATA_DATANONE       0x001
#define VCD_METADATA_QCOMFILLER     0x002
#define VCD_METADATA_QPARRAY        0x004
#define VCD_METADATA_CONCEALMB      0x008
#define VCD_METADATA_SEI            0x010
#define VCD_METADATA_VUI            0x020
#define VCD_METADATA_VC1            0x040
#define VCD_METADATA_PASSTHROUGH    0x080
#define VCD_METADATA_ENC_SLICE      0x100

struct vcd_property_meta_data_enable_type {
	u32 n_meta_data_enable_flag;
};

struct vcd_property_metadata_hdr_type {
	u32 n_meta_data_id_type;
	u32 n_version;
	u32 n_port_index;
	u32 e_type;
};

struct vcd_property_frame_rate_type {
	u32              n_fps_denominator;
	u32              n_fps_numerator;
};

struct vcd_property_target_bitrate_type {
	u32             n_target_bitrate;
};

enum vcd_yuv_buffer_format_type {
	VCD_BUFFER_FORMAT_NV12      = 0x1,
	VCD_BUFFER_FORMAT_TILE_4x2    = 0x2,
	VCD_BUFFER_FORMAT_NV12_16M2KA = 0x3
};

struct vcd_property_buffer_format_type {
	enum vcd_yuv_buffer_format_type  e_buffer_format;
};

struct vcd_property_post_filter_type {
	u32           b_post_filter;
};

enum vcd_codec_profile_type {
	VCD_PROFILE_UNKNOWN       = 0x0,
	VCD_PROFILE_MPEG4_SP      = 0x1,
	VCD_PROFILE_MPEG4_ASP     = 0x2,
	VCD_PROFILE_H264_BASELINE = 0x3,
	VCD_PROFILE_H264_MAIN     = 0x4,
	VCD_PROFILE_H264_HIGH     = 0x5,
	VCD_PROFILE_H263_BASELINE = 0x6,
	VCD_PROFILE_VC1_SIMPLE    = 0x7,
	VCD_PROFILE_VC1_MAIN      = 0x8,
	VCD_PROFILE_VC1_ADVANCE   = 0x9,
	VCD_PROFILE_MPEG2_MAIN    = 0xA,
	VCD_PROFILE_MPEG2_SIMPLE  = 0xB
};

struct vcd_property_profile_type {
	enum vcd_codec_profile_type       e_profile;
};

enum vcd_codec_level_type {
   VCD_LEVEL_UNKNOWN       = 0x0,
   VCD_LEVEL_MPEG4_0       = 0x1,
   VCD_LEVEL_MPEG4_0b      = 0x2,
   VCD_LEVEL_MPEG4_1       = 0x3,
   VCD_LEVEL_MPEG4_2       = 0x4,
   VCD_LEVEL_MPEG4_3       = 0x5,
   VCD_LEVEL_MPEG4_3b      = 0x6,
   VCD_LEVEL_MPEG4_4       = 0x7,
   VCD_LEVEL_MPEG4_4a      = 0x8,
   VCD_LEVEL_MPEG4_5       = 0x9,
   VCD_LEVEL_MPEG4_6       = 0xA,
   VCD_LEVEL_MPEG4_7       = 0xB,
   VCD_LEVEL_MPEG4_X       = 0xC,
   VCD_LEVEL_H264_1        = 0x10,
   VCD_LEVEL_H264_1b       = 0x11,
   VCD_LEVEL_H264_1p1      = 0x12,
   VCD_LEVEL_H264_1p2      = 0x13,
   VCD_LEVEL_H264_1p3      = 0x14,
   VCD_LEVEL_H264_2        = 0x15,
   VCD_LEVEL_H264_2p1      = 0x16,
   VCD_LEVEL_H264_2p2      = 0x17,
   VCD_LEVEL_H264_3        = 0x18,
   VCD_LEVEL_H264_3p1      = 0x19,
   VCD_LEVEL_H264_3p2      = 0x1A,
   VCD_LEVEL_H264_4        = 0x1B,
   VCD_LEVEL_H264_X        = 0x1C,
   VCD_LEVEL_H263_10       = 0x20,
   VCD_LEVEL_H263_20       = 0x21,
   VCD_LEVEL_H263_30       = 0x22,
   VCD_LEVEL_H263_40       = 0x23,
   VCD_LEVEL_H263_45       = 0x24,
   VCD_LEVEL_H263_50       = 0x25,
   VCD_LEVEL_H263_60       = 0x26,
   VCD_LEVEL_H263_70       = 0x27,
   VCD_LEVEL_H263_X        = 0x28,
   VCD_LEVEL_MPEG2_LOW     = 0x30,
   VCD_LEVEL_MPEG2_MAIN    = 0x31,
   VCD_LEVEL_MPEG2_HIGH_14 = 0x32,
   VCD_LEVEL_MPEG2_HIGH    = 0x33,
   VCD_LEVEL_MPEG2_X       = 0x34,
   VCD_LEVEL_VC1_LOW       = 0x40,
   VCD_LEVEL_VC1_MEDIUM    = 0x41,
   VCD_LEVEL_VC1_HIGH      = 0x42,
   VCD_LEVEL_VC1_0         = 0x43,
   VCD_LEVEL_VC1_1         = 0x44,
   VCD_LEVEL_VC1_2         = 0x45,
   VCD_LEVEL_VC1_3         = 0x46,
   VCD_LEVEL_VC1_4         = 0x47,
   VCD_LEVEL_VC1_X         = 0x48
};

struct vcd_property_level_type {
	enum vcd_codec_level_type   e_level;
};

enum vcd_m_slice_sel_type {
	VCD_MSLICE_OFF             = 0x1,
	VCD_MSLICE_BY_MB_COUNT     = 0x2,
	VCD_MSLICE_BY_BYTE_COUNT   = 0x3,
	VCD_MSLICE_BY_GOB          = 0x4
};

struct vcd_property_multi_slice_type {
	enum vcd_m_slice_sel_type   e_m_slice_sel;
	u32             n_m_slice_size;
};

enum vcd_entropy_sel_type {
	VCD_ENTROPY_SEL_CAVLC = 0x1,
	VCD_ENTROPY_SEL_CABAC = 0x2
};

enum vcd_cabac_model_type {
	VCD_CABAC_MODEL_NUMBER_0 = 0x1,
	VCD_CABAC_MODEL_NUMBER_1 = 0x2,
	VCD_CABAC_MODEL_NUMBER_2 = 0x3
};

struct vcd_property_entropy_control_type {
	enum vcd_entropy_sel_type  e_entropy_sel;
	enum vcd_cabac_model_type  e_cabac_model;
};

enum vcd_db_config_type {
	VCD_DB_ALL_BLOCKING_BOUNDARY = 0x1,
	VCD_DB_DISABLE               = 0x2,
	VCD_DB_SKIP_SLICE_BOUNDARY   = 0x3
};
struct vcd_property_db_config_type {
	enum vcd_db_config_type    e_db_config;
	u32             n_slice_alpha_offset;
	u32             n_slice_beta_offset;
};

enum vcd_rate_control_type {
	VCD_RATE_CONTROL_OFF      = 0x1,
	VCD_RATE_CONTROL_VBR_VFR  = 0x2,
	VCD_RATE_CONTROL_VBR_CFR  = 0x3,
	VCD_RATE_CONTROL_CBR_VFR  = 0x4,
	VCD_RATE_CONTROL_CBR_CFR  = 0x5
};

struct vcd_property_rate_control_type {
	enum vcd_rate_control_type     e_rate_control;
};

struct vcd_property_qp_range_type {
	u32              n_max_qp;
	u32              n_min_qp;
};

struct vcd_property_session_qp_type {
	u32              n_i_frame_qp;
	u32              n_p_frame_qp;
	u32		 		 n_b_frame_qp;
};

struct vcd_property_i_period_type {
	u32 n_p_frames;
	u32 n_b_frames;
};

struct vcd_property_vop_timing_type {
	u32   n_vop_time_resolution;
};

struct vcd_property_short_header_type {
	u32             b_short_header;
};

struct vcd_property_intra_refresh_mb_number_type {
	u32            n_cir_mb_number;
};

struct vcd_property_req_i_frame_type {
	u32        b_req_i_frame;
};

struct vcd_frame_rect_type{
   u32   n_left;
   u32   n_top;
   u32   n_right;
   u32   n_bottom;
};

struct vcd_property_dec_output_buffer_type {
	struct vcd_frame_rect_type   disp_frm;
};

#endif
