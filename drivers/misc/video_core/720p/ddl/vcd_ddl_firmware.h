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
#ifndef _VCD_DDL_FIRMWARE_H_
#define _VCD_DDL_FIRMWARE_H_
#include "vcd_property.h"

#define VCD_FW_BIG_ENDIAN     0x0
#define VCD_FW_LITTLE_ENDIAN  0x1

struct vcd_fw_details_type {
	enum vcd_codec_type e_codec;
	u32 *p_fw_buffer_addr;
	u32 n_fw_size;
};

#define VCD_FW_PROP_BASE         0x0

#define VCD_FW_ENDIAN       (VCD_FW_PROP_BASE + 0x1)
#define VCD_FW_BOOTCODE     (VCD_FW_PROP_BASE + 0x2)
#define VCD_FW_DECODE     (VCD_FW_PROP_BASE + 0x3)
#define VCD_FW_ENCODE     (VCD_FW_PROP_BASE + 0x4)

extern unsigned char *vid_c_command_control_fw;
extern u32 vid_c_command_control_fw_size;
extern unsigned char *vid_c_mpg4_dec_fw;
extern u32 vid_c_mpg4_dec_fw_size;
extern unsigned char *vid_c_h263_dec_fw;
extern u32 vid_c_h263_dec_fw_size;
extern unsigned char *vid_c_h264_dec_fw;
extern u32 vid_c_h264_dec_fw_size;
extern unsigned char *vid_c_mpg4_enc_fw;
extern u32 vid_c_mpg4_enc_fw_size;
extern unsigned char *vid_c_h264_enc_fw;
extern u32 vid_c_h264_enc_fw_size;

u32 vcd_fw_init(void);
u32 vcd_get_fw_property(u32 prop_id, void *prop_details);
u32 vcd_fw_transact(u32 b_add, u32 b_decoding, enum vcd_codec_type e_codec);
void vcd_fw_release(void);

#endif
