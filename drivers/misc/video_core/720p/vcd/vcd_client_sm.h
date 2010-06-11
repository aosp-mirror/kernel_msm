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
#ifndef _VCD_CLIENT_SM_H_
#define _VCD_CLIENT_SM_H_
#include "vcd_api.h"
#include "vcd_ddl_api.h"

struct vcd_clnt_state_table;
struct vcd_clnt_state_ctxt;
struct vcd_clnt_ctxt;

enum vcd_clnt_state_enum {
	VCD_CLIENT_STATE_NULL = 0,
	VCD_CLIENT_STATE_OPEN,
	VCD_CLIENT_STATE_STARTING,
	VCD_CLIENT_STATE_RUN,
	VCD_CLIENT_STATE_FLUSHING,
	VCD_CLIENT_STATE_PAUSING,
	VCD_CLIENT_STATE_PAUSED,
	VCD_CLIENT_STATE_STOPPING,
	VCD_CLIENT_STATE_EOS,
	VCD_CLIENT_STATE_INVALID,
	VCD_CLIENT_STATE_MAX,
	VCD_CLIENT_STATE_32BIT = 0x7FFFFFFF
};

#define CLIENT_STATE_EVENT_NUMBER(ppf) \
	((u32 *) (&(((struct vcd_clnt_state_table*)0)->ev_hdlr.ppf)) -  \
	(u32 *) (&(((struct vcd_clnt_state_table*)0)->ev_hdlr.pf_close)) + 1)

struct vcd_clnt_state_table {
	struct {
		u32(*pf_close) (struct vcd_clnt_ctxt *cctxt);
		u32(*pf_encode_start) (struct vcd_clnt_ctxt *cctxt);
		u32(*pf_encode_frame) (struct vcd_clnt_ctxt *cctxt,
			struct vcd_frame_data *input_frame);
		u32(*pf_decode_start) (struct vcd_clnt_ctxt *cctxt,
			struct vcd_sequence_hdr *seq_hdr);
		u32(*pf_decode_frame) (struct vcd_clnt_ctxt *cctxt,
			struct vcd_frame_data *input_frame);
		u32(*pf_pause) (struct vcd_clnt_ctxt *cctxt);
		u32(*pf_resume) (struct vcd_clnt_ctxt *cctxt);
		u32(*pf_flush) (struct vcd_clnt_ctxt *cctxt, u32 mode);
		u32(*pf_stop) (struct vcd_clnt_ctxt *cctxt);
		u32(*pf_set_property) (struct vcd_clnt_ctxt *cctxt,
			struct vcd_property_hdr *prop_hdr, void *prop);
		u32(*pf_get_property) (struct vcd_clnt_ctxt *cctxt,
			struct vcd_property_hdr *prop_hdr,
			void *prop);
		u32(*pf_set_buffer_requirements) (struct vcd_clnt_ctxt *cctxt,
			enum vcd_buffer_type vcd_buffer_type,
			struct vcd_buffer_requirement *buffer_req);
		u32(*pf_get_buffer_requirements) (struct vcd_clnt_ctxt *cctxt,
			  enum vcd_buffer_type vcd_buffer_type,
			  struct vcd_buffer_requirement *buffer_req);
		u32(*pf_set_buffer) (struct vcd_clnt_ctxt *cctxt,
			enum vcd_buffer_type vcd_buffer_type, void *buffer,
			size_t buf_size);
		u32(*pf_allocate_buffer) (struct vcd_clnt_ctxt *cctxt,
			enum vcd_buffer_type vcd_buffer_type, size_t sz,
			void **virt_addr, phys_addr_t *phys_addr);
		u32(*pf_free_buffer) (struct vcd_clnt_ctxt *cctxt,
			enum vcd_buffer_type vcd_buffer_type, void *buf);
		u32(*pf_fill_output_buffer) (struct vcd_clnt_ctxt *cctxt,
			struct vcd_frame_data *buffer);
		void (*pf_clnt_cb) (struct vcd_clnt_ctxt *cctxt, u32 event,
			u32 status, void *payload, u32 size, u32 *ddl_handle,
			void *const client_data);
	} ev_hdlr;

	void (*pf_entry) (struct vcd_clnt_ctxt *cctxt, s32 state_event_type);
	void (*pf_exit) (struct vcd_clnt_ctxt *cctxt, s32 state_event_type);
};

struct vcd_clnt_state_ctxt {
	const struct vcd_clnt_state_table *state_table;
	enum vcd_clnt_state_enum state;
};

extern void vcd_do_client_state_transition(struct vcd_clnt_ctxt *cctxt,
	enum vcd_clnt_state_enum to_state, u32 ev_code);

extern const struct vcd_clnt_state_table *vcd_get_client_state_table(
	enum vcd_clnt_state_enum state);

#endif
