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
#ifndef _VCD_DDL_API_H_
#define _VCD_DDL_API_H_
#include "vcd_ddl_internal_property.h"

struct ddl_init_config {
	u8 *core_virtual_base_addr;
	void (*pf_interrupt_clr) (void);
	void (*ddl_callback) (u32 event, u32 status, void *payload, u32 size,
		u32 *ddl_handle, void *const client_data);
};

struct ddl_frame_data_tag {
	struct vcd_frame_data vcd_frm;
	u32 intrlcd_ip_frm_tag;
	u32 frm_trans_end;
	u32 frm_delta;
};

u32 ddl_device_init(struct ddl_init_config *ddl_init_config, void *client_data);
u32 ddl_device_release(void *client_data);
u32 ddl_open(u32 **ddl_handle, u32 decoding);
u32 ddl_close(u32 **ddl_handle);
u32 ddl_encode_start(u32 *ddl_handle, void *client_data);
u32 ddl_encode_frame(u32 *ddl_handle, struct ddl_frame_data_tag *input_frame,
	struct ddl_frame_data_tag *output_bit, void *client_data);
u32 ddl_encode_end(u32 *ddl_handle, void *client_data);
u32 ddl_decode_start(u32 *ddl_handle, struct vcd_phys_sequence_hdr *header,
	void *client_data);
u32 ddl_decode_frame(u32 *ddl_handle, struct ddl_frame_data_tag *in_bits,
	void *client_data);
u32 ddl_decode_end(u32 *ddl_handle, void *client_data);
u32 ddl_set_property(u32 *ddl_handle, struct vcd_property_hdr *property_hdr,
	void *property_value);
u32 ddl_get_property(u32 *ddl_handle, struct vcd_property_hdr *property_hdr,
	void *property_value);
void ddl_read_and_clear_interrupt(void);
u32 ddl_process_core_response(void);
u32 ddl_reset_hw(u32 mode);

#endif
