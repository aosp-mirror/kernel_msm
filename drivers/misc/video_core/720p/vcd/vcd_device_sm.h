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
#ifndef _VCD_DEVICE_SM_H_
#define _VCD_DEVICE_SM_H_

#include "vcd_api.h"
#include "vcd_ddl_api.h"
#include "vcd_core.h"

struct vcd_dev_state_table_type_t;
struct vcd_dev_state_ctxt_type_t;
struct vcd_drv_ctxt_type_t;

enum vcd_dev_state_enum_type {
	VCD_DEVICE_STATE_NULL = 0,
	VCD_DEVICE_STATE_NOT_INIT,
	VCD_DEVICE_STATE_INITING,
	VCD_DEVICE_STATE_READY,
	VCD_DEVICE_STATE_INVALID,
	VCD_DEVICE_STATE_MAX,
	VCD_DEVICE_STATE_32BIT = 0x7FFFFFFF
};

struct vcd_dev_state_table_type_t {
	struct {
		u32(*pf_init) (struct vcd_drv_ctxt_type_t *p_drv_ctxt,
				struct vcd_init_config_type *p_config,
				s32 *p_driver_handle);

		u32(*pf_term) (struct vcd_drv_ctxt_type_t *p_drv_ctxt,
				s32 driver_handle);

		u32(*pf_open) (struct vcd_drv_ctxt_type_t *p_drv_ctxt,
				s32 driver_handle, u32 b_decoding,
				void (*callback) (u32 event, u32 status,
					void *p_info, u32 n_size, void *handle,
					void *const p_client_data),
				void *p_client_data);

		u32(*pf_close) (struct vcd_drv_ctxt_type_t *p_drv_ctxt,
				struct vcd_clnt_ctxt_type_t *p_cctxt);

		u32(*pf_resume) (struct vcd_drv_ctxt_type_t *p_drv_ctxt,
				struct vcd_clnt_ctxt_type_t *p_cctxt);

		u32(*pf_set_dev_pwr) (struct vcd_drv_ctxt_type_t *p_drv_ctxt,
				enum vcd_power_state_type e_pwr_state);

		void (*pf_dev_cb) (struct vcd_drv_ctxt_type_t *p_drv_ctxt,
				u32 event, u32 status, void *p_payload,
				u32 n_size, u32 *ddl_handle,
				void *const p_client_data);

		void (*pf_timeout) (struct vcd_drv_ctxt_type_t *p_drv_ctxt,
							void *p_user_data);
	} ev_hdlr;

	void (*pf_entry) (struct vcd_drv_ctxt_type_t *p_drv_ctxt,
			s32 n_state_event_type);
	void (*pf_exit) (struct vcd_drv_ctxt_type_t *p_drv_ctxt,
			s32 n_state_event_type);
};

#define   DEVICE_STATE_EVENT_NUMBER(ppf) \
	((u32 *) (&(((struct vcd_dev_state_table_type_t*)0)->ev_hdlr.ppf)) - \
	(u32 *) (&(((struct vcd_dev_state_table_type_t*)0)->ev_hdlr.pf_init)) \
	+ 1)

struct vcd_dev_state_ctxt_type_t {
	const struct vcd_dev_state_table_type_t *p_state_table;

	enum vcd_dev_state_enum_type e_state;
};

struct vcd_drv_ctxt_type_t {
	struct vcd_dev_state_ctxt_type_t dev_state;

	struct vcd_dev_ctxt_type dev_ctxt;

	u32 *dev_cs;
};


extern struct vcd_drv_ctxt_type_t *vcd_get_drv_context(void);

void vcd_continue(void);

#endif
