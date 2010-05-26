/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "video_core_type.h"
#include "vcd.h"

u32 vcd_init(struct vcd_init_config_type *p_config, s32 *p_driver_handle)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;

	VCD_MSG_MED("vcd_init:");

	if (!p_config ||
	    !p_driver_handle || !p_config->pf_map_dev_base_addr) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_ILLEGAL_PARM;
	}

	p_drv_ctxt = vcd_get_drv_context();

	if (!p_drv_ctxt->dev_cs)
		rc = vcd_critical_section_create(&p_drv_ctxt->dev_cs);

	if (rc)
		return VCD_ERR_ALLOC_FAIL;

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_drv_ctxt->dev_state.p_state_table->ev_hdlr.pf_init) {
		rc = p_drv_ctxt->dev_state.p_state_table->ev_hdlr.
		    pf_init(p_drv_ctxt, p_config, p_driver_handle);
	} else {
		VCD_MSG_ERROR("Unsupported API in device state %d",
			      p_drv_ctxt->dev_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_init);

u32 vcd_term(s32 driver_handle)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;

	VCD_MSG_MED("vcd_term:");

	p_drv_ctxt = vcd_get_drv_context();

	if (!p_drv_ctxt->dev_cs) {
		VCD_MSG_ERROR("No critical section object");

		return VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_drv_ctxt->dev_state.p_state_table->ev_hdlr.pf_term) {
		rc = p_drv_ctxt->dev_state.p_state_table->ev_hdlr.
		    pf_term(p_drv_ctxt, driver_handle);
	} else {
		VCD_MSG_ERROR("Unsupported API in device state %d",
			      p_drv_ctxt->dev_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	if (p_drv_ctxt->dev_state.e_state == VCD_DEVICE_STATE_NULL) {
		VCD_MSG_HIGH
		    ("Device in NULL state. Releasing critical section");

		vcd_critical_section_release(p_drv_ctxt->dev_cs);
		p_drv_ctxt->dev_cs = NULL;
	}

	return rc;

}
EXPORT_SYMBOL(vcd_term);

u32 vcd_open(s32 driver_handle, u32 b_decoding,
	void (*callback) (u32 event, u32 status, void *p_info, u32 n_size,
		       void *handle, void *const p_client_data),
	void *p_client_data)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;

	VCD_MSG_MED("vcd_open:");

	if (!callback) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_ILLEGAL_PARM;
	}

	p_drv_ctxt = vcd_get_drv_context();

	if (!p_drv_ctxt->dev_cs) {
		VCD_MSG_ERROR("No critical section object");

		return VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_drv_ctxt->dev_state.p_state_table->ev_hdlr.pf_open) {
		rc = p_drv_ctxt->dev_state.p_state_table->ev_hdlr.
		    pf_open(p_drv_ctxt, driver_handle, b_decoding, callback,
			    p_client_data);
	} else {
		VCD_MSG_ERROR("Unsupported API in device state %d",
			      p_drv_ctxt->dev_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_open);

u32 vcd_close(void *handle)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_close:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	p_drv_ctxt = vcd_get_drv_context();

	if (p_drv_ctxt->dev_state.p_state_table->ev_hdlr.pf_close) {
		rc = p_drv_ctxt->dev_state.p_state_table->ev_hdlr.
		    pf_close(p_drv_ctxt, p_cctxt);
	} else {
		VCD_MSG_ERROR("Unsupported API in device state %d",
			      p_drv_ctxt->dev_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	return rc;

}
EXPORT_SYMBOL(vcd_close);

u32 vcd_encode_start(void *handle)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_encode_start:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_encode_start &&
	    p_drv_ctxt->dev_ctxt.e_pwr_state != VCD_PWR_STATE_SLEEP) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_encode_start(p_cctxt);
	} else {
		VCD_MSG_ERROR
		    ("Unsupported API in dev power state %d OR client state %d",
		     p_drv_ctxt->dev_ctxt.e_pwr_state,
		     p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_encode_start);

u32 vcd_encode_frame(void *handle, struct vcd_frame_data_type *p_input_frame)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_encode_frame:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	if (!p_input_frame) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_BAD_POINTER;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_encode_frame) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_encode_frame(p_cctxt, p_input_frame);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_encode_frame);

u32 vcd_decode_start(void *handle, struct vcd_sequence_hdr_type *p_seq_hdr)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_decode_start:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_decode_start &&
	    p_drv_ctxt->dev_ctxt.e_pwr_state != VCD_PWR_STATE_SLEEP) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_decode_start(p_cctxt, p_seq_hdr);
	} else {
		VCD_MSG_ERROR
		    ("Unsupported API in dev power state %d OR client state %d",
		     p_drv_ctxt->dev_ctxt.e_pwr_state,
		     p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_decode_start);

u32 vcd_decode_frame(void *handle, struct vcd_frame_data_type *p_input_frame)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_decode_frame:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	if (!p_input_frame) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_BAD_POINTER;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_decode_frame) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_decode_frame(p_cctxt, p_input_frame);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_decode_frame);

u32 vcd_pause(void *handle)
{
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	u32 rc;

	VCD_MSG_MED("vcd_pause:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_pause) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_pause(p_cctxt);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_pause);

u32 vcd_resume(void *handle)
{
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	u32 rc;

	VCD_MSG_MED("vcd_resume:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_drv_ctxt->dev_state.p_state_table->ev_hdlr.pf_resume &&
	    p_drv_ctxt->dev_ctxt.e_pwr_state != VCD_PWR_STATE_SLEEP) {
		rc = p_drv_ctxt->dev_state.p_state_table->ev_hdlr.
		    pf_resume(p_drv_ctxt, p_cctxt);
	} else {
		VCD_MSG_ERROR
		    ("Unsupported API in dev power state %d OR client state %d",
		     p_drv_ctxt->dev_ctxt.e_pwr_state,
		     p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_resume);

u32 vcd_flush(void *handle, u32 n_mode)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_flush:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_flush) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_flush(p_cctxt, n_mode);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_flush);

u32 vcd_stop(void *handle)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_stop:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_stop &&
	    p_drv_ctxt->dev_ctxt.e_pwr_state != VCD_PWR_STATE_SLEEP) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_stop(p_cctxt);
	} else {
		VCD_MSG_ERROR
		    ("Unsupported API in dev power state %d OR client state %d",
		     p_drv_ctxt->dev_ctxt.e_pwr_state,
		     p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_stop);

u32 vcd_set_property(void *handle,
     struct vcd_property_hdr_type *p_prop_hdr, void *p_prop_val)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_set_property:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	if (!p_prop_hdr || !p_prop_val) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_BAD_POINTER;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_set_property) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_set_property(p_cctxt, p_prop_hdr, p_prop_val);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_set_property);

u32 vcd_get_property(void *handle,
     struct vcd_property_hdr_type *p_prop_hdr, void *p_prop_val)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_get_property:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	if (!p_prop_hdr || !p_prop_val) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_BAD_POINTER;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_get_property) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_get_property(p_cctxt, p_prop_hdr, p_prop_val);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_get_property);

u32 vcd_set_buffer_requirements(void *handle,
     enum vcd_buffer_type e_buffer,
     struct vcd_buffer_requirement_type *p_buffer_req)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_set_buffer_requirements:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	if (!p_buffer_req) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_BAD_POINTER;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.
	    pf_set_buffer_requirements) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_set_buffer_requirements(p_cctxt, e_buffer, p_buffer_req);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_set_buffer_requirements);

u32 vcd_get_buffer_requirements(void *handle,
     enum vcd_buffer_type e_buffer,
     struct vcd_buffer_requirement_type *p_buffer_req)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_get_buffer_requirements:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	if (!p_buffer_req) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_BAD_POINTER;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.
	    pf_get_buffer_requirements) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_get_buffer_requirements(p_cctxt, e_buffer, p_buffer_req);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_get_buffer_requirements);

u32 vcd_set_buffer(void *handle,
     enum vcd_buffer_type e_buffer, u8 *p_buffer, u32 n_buf_size)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_set_buffer:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	if (!p_buffer || !n_buf_size) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_BAD_POINTER;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_set_buffer) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_set_buffer(p_cctxt, e_buffer, p_buffer, n_buf_size);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_set_buffer);

u32 vcd_allocate_buffer(void *handle,
     enum vcd_buffer_type e_buffer,
     u32 n_buf_size, u8 **pp_vir_buf_addr, u8 **pp_phy_buf_addr)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_allocate_buffer:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	if (!pp_vir_buf_addr || !pp_phy_buf_addr
	    || !n_buf_size) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_BAD_POINTER;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_allocate_buffer) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_allocate_buffer(p_cctxt, e_buffer, n_buf_size,
				       pp_vir_buf_addr, pp_phy_buf_addr);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_allocate_buffer);

u32 vcd_free_buffer(void *handle, enum vcd_buffer_type e_buffer, u8 *p_buffer)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_free_buffer:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_free_buffer) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_free_buffer(p_cctxt, e_buffer, p_buffer);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_free_buffer);

u32 vcd_fill_output_buffer(void *handle, struct vcd_frame_data_type *p_buffer)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt =
	    (struct vcd_clnt_ctxt_type_t *)handle;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	u32 rc;

	VCD_MSG_MED("vcd_fill_output_buffer:");

	if (!p_cctxt || p_cctxt->n_signature != VCD_SIGNATURE) {
		VCD_MSG_ERROR("Bad client handle");

		return VCD_ERR_BAD_HANDLE;
	}

	if (!p_buffer) {
		VCD_MSG_ERROR("Bad parameters");

		return VCD_ERR_BAD_POINTER;
	}

	p_drv_ctxt = vcd_get_drv_context();

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_fill_output_buffer) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
		    pf_fill_output_buffer(p_cctxt, p_buffer);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
			      p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_fill_output_buffer);

u32 vcd_set_device_power(s32 driver_handle,
		enum vcd_power_state_type e_pwr_state)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;

	VCD_MSG_MED("vcd_set_device_power:");

	p_drv_ctxt = vcd_get_drv_context();

	if (!p_drv_ctxt->dev_cs) {
		VCD_MSG_ERROR("No critical section object");

		return VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (p_drv_ctxt->dev_state.p_state_table->ev_hdlr.pf_set_dev_pwr) {
		rc = p_drv_ctxt->dev_state.p_state_table->ev_hdlr.
		    pf_set_dev_pwr(p_drv_ctxt, e_pwr_state);
	} else {
		VCD_MSG_ERROR("Unsupported API in device state %d",
			      p_drv_ctxt->dev_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	vcd_critical_section_leave(p_drv_ctxt->dev_cs);

	return rc;

}
EXPORT_SYMBOL(vcd_set_device_power);

void vcd_read_and_clear_interrupt(void)
{
   VCD_MSG_LOW("vcd_read_and_clear_interrupt:");
   ddl_read_and_clear_interrupt();
}


void vcd_response_handler(void)
{
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;

	VCD_MSG_LOW("vcd_response_handler:");
  p_drv_ctxt = vcd_get_drv_context();

  vcd_critical_section_enter(p_drv_ctxt->dev_cs);

	if (!ddl_process_core_response()) {
		VCD_MSG_HIGH
		    ("ddl_process_core_response indicated no further"
		     "processing");
    vcd_critical_section_leave(p_drv_ctxt->dev_cs);
		return;
	}

	if (p_drv_ctxt->dev_ctxt.b_continue)
		vcd_continue();
	vcd_critical_section_leave(p_drv_ctxt->dev_cs);
}
EXPORT_SYMBOL(vcd_response_handler);






