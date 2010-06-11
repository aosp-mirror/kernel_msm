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
#ifndef _VCD_API_H_
#define _VCD_API_H_
#include "vcd_property.h"
#include "vcd_status.h"

#define VCD_FRAME_FLAG_EOS 0x00000001
#define VCD_FRAME_FLAG_ENDOFFRAME 0x00000010
#define VCD_FRAME_FLAG_SYNCFRAME 0x00000020
#define VCD_FRAME_FLAG_EXTRADATA 0x00000040
#define VCD_FRAME_FLAG_CODECCONFIG  0x00000080

#define VCD_FLUSH_INPUT   0x0001
#define VCD_FLUSH_OUTPUT  0x0002
#define VCD_FLUSH_ALL     0x0003

#define VCD_FRAMETAG_INVALID  0xffffffff

struct vcd_handle_container {
	void *handle;
};
struct vcd_flush_cmd {
	u32 mode;
};

enum vcd_frame {
	VCD_FRAME_YUV = 1,
	VCD_FRAME_I,
	VCD_FRAME_P,
	VCD_FRAME_B,
	VCD_FRAME_32BIT = 0x7fffffff
};

enum vcd_power_state {
	VCD_PWR_STATE_ON = 1,
	VCD_PWR_STATE_SLEEP,
};

struct vcd_frame_data {
	void *virt_addr;
	phys_addr_t phys_addr;
	size_t alloc_len;
	size_t data_len;
	size_t offset;
	s64 time_stamp;
	u32 flags;
	void *client_data;
	struct vcd_property_dec_output_buffer dec_op_prop;
	u32 interlaced;
	enum vcd_frame frame_type;
	u32 ip_frm_tag;
};

struct vcd_phys_sequence_hdr {
	phys_addr_t addr;
	size_t sz;
};

struct vcd_sequence_hdr {
	void *addr;
	size_t sz;
};

enum vcd_buffer_type {
	VCD_BUFFER_INPUT = 0x1,
	VCD_BUFFER_OUTPUT = 0x2,
	VCD_BUFFER_INVALID = 0x3,
	VCD_BUFFER_32BIT = 0x7FFFFFFF
};

struct vcd_buffer_requirement {
	u32 min_count;
	u32 actual_count;
	u32 max_count;
	size_t size;
	u32 align;
	u32 buf_pool_id;
};

struct vcd_init_config {
	void *device_name;
	void *(*pf_map_dev_base_addr) (void *device_name);
	void (*pf_un_map_dev_base_addr) (void);
	void (*pf_interrupt_clr) (void);
	void (*pf_register_isr) (void *device_name);
	void (*pf_deregister_isr) (void);
	u32  (*pf_timer_create) (void (*pf_timer_handler)(void *),
		void *user_data, void **pp_timer_handle);
	void (*pf_timer_release) (void *timer_handle);
	void (*pf_timer_start) (void *timer_handle, u32 time_out);
	void (*pf_timer_stop) (void *timer_handle);
};

u32 vcd_init(struct vcd_init_config *config, s32 *driver_handle);
u32 vcd_term(s32 driver_handle);
u32 vcd_open(s32 driver_handle, u32 decoding,
	void (*callback) (u32 event, u32 status, void *info, u32 size,
	void *handle, void *const client_data), void *client_data);
u32 vcd_close(void *handle);
u32 vcd_encode_start(void *handle);
u32 vcd_encode_frame(void *handle, struct vcd_frame_data *input_frame);
u32 vcd_decode_start(void *handle, struct vcd_sequence_hdr *seq_hdr);
u32 vcd_decode_frame(void *handle, struct vcd_frame_data *input_frame);
u32 vcd_pause(void *handle);
u32 vcd_resume(void *handle);
u32 vcd_flush(void *handle, u32 mode);
u32 vcd_stop(void *handle);
u32 vcd_set_property(void *handle, struct vcd_property_hdr *prop_hdr,
	void *prop_val);
u32 vcd_get_property(void *handle, struct vcd_property_hdr *prop_hdr,
	void *prop_val);
u32 vcd_set_buffer_requirements(void *handle, enum vcd_buffer_type buffer_type,
	struct vcd_buffer_requirement *buffer_req);
u32 vcd_get_buffer_requirements(void *handle, enum vcd_buffer_type buffer_type,
	struct vcd_buffer_requirement *buffer_req);
u32 vcd_set_buffer(void *handle, enum vcd_buffer_type buffer_type,
	void *buffer, size_t buf_size);
u32 vcd_allocate_buffer(void *handle, enum vcd_buffer_type buffer_type,
	size_t sz, void **virt_addr, phys_addr_t *phys_addr);
u32 vcd_free_buffer(void *handle, enum vcd_buffer_type buffer_type, void *buf);
u32 vcd_fill_output_buffer(void *handle, struct vcd_frame_data *buffer);
u32 vcd_set_device_power(s32 driver_handle, enum vcd_power_state pwr_state);
void vcd_read_and_clear_interrupt(void);
void vcd_response_handler(void);

#endif
