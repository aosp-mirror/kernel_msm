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
#ifndef _VCD_DDL_INTERNAL_PROPERTY_H_
#define _VCD_DDL_INTERNAL_PROPERTY_H_
#include "vcd_api.h"

#define VCD_EVT_RESP_DDL_BASE          0x3000
#define VCD_EVT_RESP_DEVICE_INIT       (VCD_EVT_RESP_DDL_BASE + 0x1)
#define VCD_EVT_RESP_OUTPUT_REQ        (VCD_EVT_RESP_DDL_BASE + 0x2)
#define VCD_EVT_RESP_EOS_DONE          (VCD_EVT_RESP_DDL_BASE + 0x3)
#define VCD_EVT_RESP_TRANSACTION_PENDING (VCD_EVT_RESP_DDL_BASE + 0x4)

#define VCD_S_DDL_ERR_BASE     0x90000000
#define VCD_ERR_MAX_NO_CODEC   (VCD_S_DDL_ERR_BASE + 0x1)
#define VCD_ERR_CLIENT_PRESENT (VCD_S_DDL_ERR_BASE + 0x2)
#define VCD_ERR_CLIENT_FATAL   (VCD_S_DDL_ERR_BASE + 0x3)

#define VCD_I_CUSTOM_BASE  (VCD_I_RESERVED_BASE)
#define VCD_I_RC_LEVEL_CONFIG (VCD_I_CUSTOM_BASE + 0x1)
#define VCD_I_FRAME_LEVEL_RC (VCD_I_CUSTOM_BASE + 0x2)
#define VCD_I_ADAPTIVE_RC    (VCD_I_CUSTOM_BASE + 0x3)
#define VCD_I_CUSTOM_DDL_BASE  (VCD_I_RESERVED_BASE + 0x100)
#define DDL_I_INPUT_BUF_REQ  (VCD_I_CUSTOM_DDL_BASE + 0x1)
#define DDL_I_OUTPUT_BUF_REQ (VCD_I_CUSTOM_DDL_BASE + 0x2)
#define DDL_I_DPB       (VCD_I_CUSTOM_DDL_BASE + 0x3)
#define DDL_I_DPB_RELEASE    (VCD_I_CUSTOM_DDL_BASE + 0x4)
#define DDL_I_DPB_RETRIEVE  (VCD_I_CUSTOM_DDL_BASE + 0x5)
#define DDL_I_REQ_OUTPUT_FLUSH   (VCD_I_CUSTOM_DDL_BASE + 0x6)
#define DDL_I_SEQHDR_ALIGN_BYTES (VCD_I_CUSTOM_DDL_BASE + 0x7)
#define DDL_I_SEQHDR_PRESENT (VCD_I_CUSTOM_DDL_BASE + 0xb)
#define DDL_I_CAPABILITY    (VCD_I_CUSTOM_DDL_BASE + 0x8)
#define DDL_I_FRAME_PROC_UNITS    (VCD_I_CUSTOM_DDL_BASE + 0x9)

struct vcd_property_rc_level_type {
	u32 b_frame_level_rc;
	u32 b_mb_level_rc;
};

struct vcd_property_frame_level_rc_params_type {
	u32 n_reaction_coeff;
};

struct vcd_property_adaptive_rc_params_type {
	u32 b_dark_region_as_flag;
	u32 b_smooth_region_as_flag;
	u32 b_static_region_as_flag;
	u32 b_activity_region_flag;
};

struct ddl_frame_data_type_tag;

struct ddl_property_dec_pic_buffers_type {
	struct ddl_frame_data_type_tag *a_dec_pic_buffers;
	u32 n_no_of_dec_pic_buf;
};

struct ddl_property_capability_type {
	u32 n_max_num_client;
	u32 n_general_command_depth;
	u32 n_frame_command_depth;
	u32 b_exclusive;
	u32   n_ddl_time_out_in_ms;
};

#endif
