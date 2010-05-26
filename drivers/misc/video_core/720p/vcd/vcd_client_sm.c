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

static const struct vcd_clnt_state_table_type_t *vcd_clnt_state_table[];

void vcd_clnt_handle_device_err_fatal(struct vcd_clnt_ctxt_type_t *p_cctxt,
								  u32 e_event)
{
	if (p_cctxt->clnt_state.e_state != VCD_CLIENT_STATE_INVALID) {
		p_cctxt->callback(e_event, VCD_ERR_HW_FATAL, NULL, 0,
			p_cctxt, p_cctxt->p_client_data);
		vcd_flush_buffers_in_err_fatal(p_cctxt);
		vcd_do_client_state_transition(p_cctxt,
			VCD_CLIENT_STATE_INVALID,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	}
}

static u32 vcd_close_in_open(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_close_in_open:");
	if (p_cctxt->in_buf_pool.n_allocated ||
		 p_cctxt->out_buf_pool.n_allocated) {
		VCD_MSG_ERROR("\n Allocated buffers are not freed yet");
		return VCD_ERR_ILLEGAL_OP;
	}
	vcd_destroy_client_context(p_cctxt);
	return rc;
}

static u32  vcd_close_in_invalid(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	VCD_MSG_LOW("vcd_close_in_invalid:");
	if (p_cctxt->in_buf_pool.n_allocated ||
		p_cctxt->out_buf_pool.n_allocated){
		VCD_MSG_ERROR("Allocated buffers are not freed yet");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (p_cctxt->status.b_cleaning_up)
		p_cctxt->status.b_close_pending = TRUE;
	else
		vcd_destroy_client_context(p_cctxt);
	return VCD_S_SUCCESS;
}

static u32 vcd_start_in_run_cmn(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	VCD_MSG_LOW("vcd_start_in_run_cmn:");
	p_cctxt->callback(VCD_EVT_RESP_START, VCD_S_SUCCESS, NULL, 0,
					  p_cctxt, p_cctxt->p_client_data);
	return VCD_S_SUCCESS;

}

static u32 vcd_encode_start_in_open(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_property_hdr_type prop_hdr;
	struct vcd_property_vop_timing_type timing;

	VCD_MSG_LOW("vcd_encode_start_in_open:");

	if (p_cctxt->b_decoding) {
		VCD_MSG_ERROR("vcd_encode_init for decoder client");

		return VCD_ERR_ILLEGAL_OP;
	}

	if (!p_cctxt->in_buf_pool.a_entries ||
	    !p_cctxt->out_buf_pool.a_entries ||
	    p_cctxt->in_buf_pool.n_validated != p_cctxt->in_buf_pool.n_count ||
	    p_cctxt->out_buf_pool.n_validated !=
	    p_cctxt->out_buf_pool.n_count) {
		VCD_MSG_ERROR("Buffer pool is not completely setup yet");

		return VCD_ERR_BAD_STATE;
	}

	rc = vcd_add_client_to_sched(p_cctxt);

	VCD_FAILED_RETURN(rc, "Failed: vcd_add_client_to_sched");

	prop_hdr.prop_id = VCD_I_VOP_TIMING;
	prop_hdr.n_size = sizeof(struct vcd_property_vop_timing_type);
	rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr, &timing);

	VCD_FAILED_RETURN(rc, "Failed: Get VCD_I_VOP_TIMING");
	if (!timing.n_vop_time_resolution) {
		VCD_MSG_ERROR("Vop_time_resolution value is zero");
		return VCD_ERR_FAIL;
	}
	p_cctxt->n_time_resoln = timing.n_vop_time_resolution;

	rc = vcd_process_cmd_sess_start(p_cctxt);

	if (!VCD_FAILED(rc)) {
		vcd_do_client_state_transition(p_cctxt,
					       VCD_CLIENT_STATE_STARTING,
					       CLIENT_STATE_EVENT_NUMBER
					       (pf_encode_start));
	}

	return rc;
}

static u32  vcd_encode_start_in_run(struct vcd_clnt_ctxt_type_t
	*p_cctxt)
{
	VCD_MSG_LOW("vcd_encode_start_in_run:");
	(void) vcd_start_in_run_cmn(p_cctxt);
	return VCD_S_SUCCESS;
}


static u32 vcd_encode_frame_cmn(struct vcd_clnt_ctxt_type_t *p_cctxt,
     struct vcd_frame_data_type *p_input_frame)
{
	VCD_MSG_LOW("vcd_encode_frame_cmn in %d:", p_cctxt->clnt_state.e_state);

	if (p_cctxt->b_decoding) {
		VCD_MSG_ERROR("vcd_encode_frame for decoder client");

		return VCD_ERR_ILLEGAL_OP;
	}

	return vcd_handle_input_frame(p_cctxt, p_input_frame);
}

static u32 vcd_decode_start_in_open
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     struct vcd_sequence_hdr_type *p_seq_hdr)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_decode_start_in_open:");

	if (!p_cctxt->b_decoding) {
		VCD_MSG_ERROR("vcd_decode_init for encoder client");

		return VCD_ERR_ILLEGAL_OP;
	}

	if (p_seq_hdr) {
		VCD_MSG_HIGH("Seq hdr supplied. len = %d",
			     p_seq_hdr->n_sequence_header_len);

		rc = vcd_store_seq_hdr(p_cctxt, p_seq_hdr);

	} else {
		VCD_MSG_HIGH("Seq hdr not supplied");

		p_cctxt->seq_hdr.n_sequence_header_len = 0;
		p_cctxt->seq_hdr.p_sequence_header = NULL;
	}

	VCD_FAILED_RETURN(rc, "Err processing seq hdr");

	rc = vcd_process_cmd_sess_start(p_cctxt);

	if (!VCD_FAILED(rc)) {
		vcd_do_client_state_transition(p_cctxt,
					       VCD_CLIENT_STATE_STARTING,
					       CLIENT_STATE_EVENT_NUMBER
					       (pf_decode_start));
	}

	return rc;
}

static u32 vcd_decode_start_in_run(struct vcd_clnt_ctxt_type_t *p_cctxt,
	struct vcd_sequence_hdr_type *p_seqhdr)
{
   VCD_MSG_LOW("vcd_decode_start_in_run:");
   (void) vcd_start_in_run_cmn(p_cctxt);
   return VCD_S_SUCCESS;
}

static u32 vcd_decode_frame_cmn
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     struct vcd_frame_data_type *p_input_frame)
{
	VCD_MSG_LOW("vcd_decode_frame_cmn in %d:", p_cctxt->clnt_state.e_state);

	if (!p_cctxt->b_decoding) {
		VCD_MSG_ERROR("Decode_frame api called for Encoder client");

		return VCD_ERR_ILLEGAL_OP;
	}

	return vcd_handle_input_frame(p_cctxt, p_input_frame);
}

static u32 vcd_pause_in_run(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_pause_in_run:");

	if (p_cctxt->b_sched_clnt_valid) {
		rc = vcd_map_sched_status(sched_suspend_resume_client
					  (p_cctxt->p_dev_ctxt->sched_hdl,
					   p_cctxt->sched_clnt_hdl, FALSE));
	}

	VCD_FAILED_RETURN(rc, "Failed: sched_suspend_resume_client");

	if (p_cctxt->status.n_frame_submitted > 0) {
		vcd_do_client_state_transition(p_cctxt,
					       VCD_CLIENT_STATE_PAUSING,
					       CLIENT_STATE_EVENT_NUMBER
					       (pf_pause));

	} else {
		VCD_MSG_HIGH("No client frames are currently being processed");

		vcd_do_client_state_transition(p_cctxt,
					       VCD_CLIENT_STATE_PAUSED,
					       CLIENT_STATE_EVENT_NUMBER
					       (pf_pause));

		p_cctxt->callback(VCD_EVT_RESP_PAUSE,
				  VCD_S_SUCCESS,
				  NULL, 0, p_cctxt, p_cctxt->p_client_data);

		rc = vcd_power_event(p_cctxt->p_dev_ctxt, p_cctxt,
				     VCD_EVT_PWR_CLNT_PAUSE);

		if (VCD_FAILED(rc))
			VCD_MSG_ERROR("VCD_EVT_PWR_CLNT_PAUSE_END failed");

	}

	return VCD_S_SUCCESS;
}

static u32 vcd_resume_in_paused(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_resume_in_paused:");

	if (p_cctxt->b_sched_clnt_valid) {

		rc = vcd_power_event(p_cctxt->p_dev_ctxt,
				     p_cctxt, VCD_EVT_PWR_CLNT_RESUME);

		if (VCD_FAILED(rc)) {
			VCD_MSG_ERROR("VCD_EVT_PWR_CLNT_RESUME failed");
		} else {

			rc = vcd_map_sched_status(sched_suspend_resume_client
						  (p_cctxt->p_dev_ctxt->
						   sched_hdl,
						   p_cctxt->sched_clnt_hdl,
						   TRUE));
			if (VCD_FAILED(rc)) {
				VCD_MSG_ERROR
				    ("rc = 0x%x. Failed: "
				     "sched_suspend_resume_client",
				     rc);
			}

		}
		if (!VCD_FAILED(rc))
			vcd_try_submit_frame(p_dev_ctxt);
	}

	if (!VCD_FAILED(rc)) {
		vcd_do_client_state_transition(p_cctxt,
					       VCD_CLIENT_STATE_RUN,
					       CLIENT_STATE_EVENT_NUMBER
					       (pf_resume));
	}

	return rc;
}

static u32 vcd_flush_cmn(struct vcd_clnt_ctxt_type_t *p_cctxt, u32 n_mode)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_flush_cmn in %d:", p_cctxt->clnt_state.e_state);

	rc = vcd_flush_buffers(p_cctxt, n_mode);

	VCD_FAILED_RETURN(rc, "Failed: vcd_flush_buffers");

	if (p_cctxt->status.n_frame_submitted > 0) {
		vcd_do_client_state_transition(p_cctxt,
					       VCD_CLIENT_STATE_FLUSHING,
					       CLIENT_STATE_EVENT_NUMBER
					       (pf_flush));
	} else {
		VCD_MSG_HIGH("All buffers are flushed");
		p_cctxt->status.n_flush_mode = n_mode;
		vcd_send_flush_done(p_cctxt, VCD_S_SUCCESS);
	}

	return rc;
}

static u32  vcd_flush_inopen(struct vcd_clnt_ctxt_type_t *p_cctxt,
	u32 n_mode)
{
   VCD_MSG_LOW("vcd_flush_inopen:");
   p_cctxt->status.n_flush_mode = n_mode;
   vcd_send_flush_done(p_cctxt, VCD_S_SUCCESS);
   return VCD_S_SUCCESS;
}

static u32 vcd_flush_in_flushing
    (struct vcd_clnt_ctxt_type_t *p_cctxt, u32 n_mode)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_flush_in_flushing:");

	rc = vcd_flush_buffers(p_cctxt, n_mode);

	return rc;
}

static u32 vcd_flush_in_eos(struct vcd_clnt_ctxt_type_t *p_cctxt,
	u32 n_mode)
{
	VCD_MSG_LOW("vcd_flush_in_eos:");

	if (n_mode > VCD_FLUSH_ALL || !n_mode) {
		VCD_MSG_ERROR("Invalid flush mode %d", n_mode);

		return VCD_ERR_ILLEGAL_PARM;
	}

	VCD_MSG_MED("Flush mode requested %d", n_mode);

	p_cctxt->status.n_flush_mode |= n_mode;

	return VCD_S_SUCCESS;
}

static u32 vcd_flush_in_invalid(struct vcd_clnt_ctxt_type_t *p_cctxt,
	u32 mode)
{
	u32 rc = VCD_S_SUCCESS;
	VCD_MSG_LOW("vcd_flush_in_invalid:");
	if (!p_cctxt->status.b_cleaning_up) {
		rc = vcd_flush_buffers(p_cctxt, mode);
		if (!VCD_FAILED(rc)) {
			VCD_MSG_HIGH("All buffers are flushed");
			p_cctxt->status.n_flush_mode = mode;
			vcd_send_flush_done(p_cctxt, VCD_S_SUCCESS);
		}
	}
	return rc;
}

static u32 vcd_stop_cmn(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	u32 rc = VCD_S_SUCCESS;
	struct vcd_transc_type *p_transc;

	VCD_MSG_LOW("vcd_stop_cmn in %d:", p_cctxt->clnt_state.e_state);

	rc = vcd_flush_buffers(p_cctxt, VCD_FLUSH_ALL);

	VCD_FAILED_RETURN(rc, "Failed: vcd_flush_buffers");

	if (!p_cctxt->status.n_frame_submitted) {

		if (vcd_get_command_channel(p_dev_ctxt, &p_transc)) {
			rc = vcd_power_event(p_dev_ctxt, p_cctxt,
				VCD_EVT_PWR_CLNT_CMD_BEGIN);

			if (!VCD_FAILED(rc)) {
				p_transc->e_type = VCD_CMD_CODEC_STOP;
				p_transc->p_cctxt = p_cctxt;

				rc = vcd_submit_cmd_sess_end(p_transc);
			} else {
				VCD_MSG_ERROR("Failed:"
					" VCD_EVT_PWR_CLNT_CMD_BEGIN");
			}

			if (VCD_FAILED(rc)) {
				vcd_release_command_channel(p_dev_ctxt,
							    p_transc);
			}

		} else {
			vcd_client_cmd_flush_and_en_q(p_cctxt,
						      VCD_CMD_CODEC_STOP);
		}
	}

	if (VCD_FAILED(rc)) {
		(void)vcd_power_event(p_dev_ctxt, p_cctxt,
				      VCD_EVT_PWR_CLNT_CMD_FAIL);
	} else {
		vcd_do_client_state_transition(p_cctxt,
					       VCD_CLIENT_STATE_STOPPING,
					       CLIENT_STATE_EVENT_NUMBER
					       (pf_stop));
	}

	return rc;
}


static u32  vcd_stop_inopen(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	VCD_MSG_LOW("vcd_stop_inopen:");

	p_cctxt->callback(VCD_EVT_RESP_STOP, VCD_S_SUCCESS,
					  NULL, 0, p_cctxt,
					  p_cctxt->p_client_data);

	return VCD_S_SUCCESS;
}

static u32 vcd_stop_in_run(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_stop_in_run:");

	rc = vcd_stop_cmn(p_cctxt);

	if (!VCD_FAILED(rc) && p_cctxt->status.b1st_frame_recvd) {
		rc = vcd_power_event(p_cctxt->p_dev_ctxt,
				     p_cctxt, VCD_EVT_PWR_CLNT_LAST_FRAME);
	}

	return rc;
}

static u32 vcd_stop_in_eos(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_stop_in_eos:");

	p_cctxt->status.b_stop_pending = TRUE;

	return rc;
}

static u32  vcd_stop_in_invalid(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	VCD_MSG_LOW("vcd_stop_in_invalid:");
	if (p_cctxt->status.b_cleaning_up) {
		p_cctxt->status.b_stop_pending = TRUE;
	} else {
		(void) vcd_flush_buffers(p_cctxt, VCD_FLUSH_ALL);
		p_cctxt->callback(VCD_EVT_RESP_STOP, VCD_S_SUCCESS, NULL,
			0, p_cctxt,	p_cctxt->p_client_data);
	}
	return VCD_S_SUCCESS;
}

static u32 vcd_set_property_cmn
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     struct vcd_property_hdr_type *p_prop_hdr, void *p_prop_val)
{
	u32 rc;

	VCD_MSG_LOW("vcd_set_property_cmn in %d:", p_cctxt->clnt_state.e_state);
	VCD_MSG_LOW("property Id = %d", p_prop_hdr->prop_id);

	if (!p_prop_hdr->n_size || !p_prop_hdr->prop_id) {
		VCD_MSG_MED("Bad parameters");

		return VCD_ERR_ILLEGAL_PARM;
	}

	rc = ddl_set_property(p_cctxt->ddl_handle, p_prop_hdr, p_prop_val);

	VCD_FAILED_RETURN(rc, "Failed: ddl_set_property");

	switch (p_prop_hdr->prop_id) {

	case VCD_I_LIVE:
		{
			struct vcd_property_live_type *p_live =
			    (struct vcd_property_live_type *)p_prop_val;

			p_cctxt->b_live = p_live->b_live;

			break;
		}

	case VCD_I_FRAME_RATE:
		{
			if (p_cctxt->b_sched_clnt_valid) {
				rc = vcd_set_frame_rate(p_cctxt,
					(struct vcd_property_frame_rate_type *)
					p_prop_val);
			}

			break;
		}

	case VCD_I_FRAME_SIZE:
		{
			if (p_cctxt->b_sched_clnt_valid) {
				rc = vcd_set_frame_size(p_cctxt,
					(struct vcd_property_frame_size_type *)
					p_prop_val);
			}

			break;
		}

	default:
		{
			break;
		}

	}

	return rc;
}

static u32 vcd_get_property_cmn
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     struct vcd_property_hdr_type *p_prop_hdr, void *p_prop_val)
{
	VCD_MSG_LOW("vcd_get_property_cmn in %d:", p_cctxt->clnt_state.e_state);
	VCD_MSG_LOW("property Id = %d", p_prop_hdr->prop_id);
	if (!p_prop_hdr->n_size || !p_prop_hdr->prop_id) {
		VCD_MSG_MED("Bad parameters");

		return VCD_ERR_ILLEGAL_PARM;
	}
	return ddl_get_property(p_cctxt->ddl_handle, p_prop_hdr, p_prop_val);
}

static u32 vcd_set_buffer_requirements_cmn
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     enum vcd_buffer_type e_buffer,
     struct vcd_buffer_requirement_type *p_buffer_req)
{
	struct vcd_property_hdr_type Prop_hdr;
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_pool_type *p_buf_pool;

	VCD_MSG_LOW("vcd_set_buffer_requirements_cmn in %d:",
		    p_cctxt->clnt_state.e_state);

	if (!p_cctxt->b_decoding &&
	    p_cctxt->clnt_state.e_state != VCD_CLIENT_STATE_OPEN) {
		VCD_MSG_ERROR("Bad state (%d) for encoder",
			      p_cctxt->clnt_state.e_state);

		return VCD_ERR_BAD_STATE;
	}

	VCD_MSG_MED("Buffer type = %d", e_buffer);

	if (e_buffer == VCD_BUFFER_INPUT) {
		Prop_hdr.prop_id = DDL_I_INPUT_BUF_REQ;
		p_buf_pool = &p_cctxt->in_buf_pool;
	} else if (e_buffer == VCD_BUFFER_OUTPUT) {
		Prop_hdr.prop_id = DDL_I_OUTPUT_BUF_REQ;
		p_buf_pool = &p_cctxt->out_buf_pool;
	} else {
		rc = VCD_ERR_ILLEGAL_PARM;
	}

	VCD_FAILED_RETURN(rc, "Invalid buffer type provided");

	if (p_buf_pool->n_validated > 0) {
		VCD_MSG_ERROR("Need to free allocated buffers");

		return VCD_ERR_ILLEGAL_OP;
	}

	Prop_hdr.n_size = sizeof(*p_buffer_req);

	rc = ddl_set_property(p_cctxt->ddl_handle, &Prop_hdr, p_buffer_req);

	VCD_FAILED_RETURN(rc, "Failed: ddl_set_property");

	if (p_buf_pool->a_entries) {
		VCD_MSG_MED("Resetting buffer requirements");

		vcd_free_buffer_pool_entries(p_buf_pool);
	}

	rc = vcd_alloc_buffer_pool_entries(p_buf_pool, p_buffer_req);

	return rc;

}

static u32 vcd_get_buffer_requirements_cmn
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     enum vcd_buffer_type e_buffer,
     struct vcd_buffer_requirement_type *p_buffer_req)
{
	struct vcd_property_hdr_type Prop_hdr;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_get_buffer_requirements_cmn in %d:",
		    p_cctxt->clnt_state.e_state);

	VCD_MSG_MED("Buffer type = %d", e_buffer);

	if (e_buffer == VCD_BUFFER_INPUT)
		Prop_hdr.prop_id = DDL_I_INPUT_BUF_REQ;
	else if (e_buffer == VCD_BUFFER_OUTPUT)
		Prop_hdr.prop_id = DDL_I_OUTPUT_BUF_REQ;
	else
		rc = VCD_ERR_ILLEGAL_PARM;

	VCD_FAILED_RETURN(rc, "Invalid buffer type provided");

	Prop_hdr.n_size = sizeof(*p_buffer_req);

	return ddl_get_property(p_cctxt->ddl_handle, &Prop_hdr, p_buffer_req);

}

static u32 vcd_set_buffer_cmn
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     enum vcd_buffer_type e_buffer, u8 *p_buffer, u32 n_buf_size)
{
	u32 rc;
	struct vcd_buffer_pool_type *p_buf_pool;

	VCD_MSG_LOW("vcd_set_buffer_cmn in %d:", p_cctxt->clnt_state.e_state);

	rc = vcd_common_allocate_set_buffer(p_cctxt, e_buffer, n_buf_size,
					    &p_buf_pool);

	if (!VCD_FAILED(rc)) {
		rc = vcd_set_buffer_internal(p_cctxt, p_buf_pool, p_buffer,
					     n_buf_size);
	}

	return rc;
}

static u32 vcd_allocate_buffer_cmn
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     enum vcd_buffer_type e_buffer,
     u32 n_buf_size, u8 **pp_vir_buf_addr, u8 **pp_phy_buf_addr)
{
	u32 rc;
	struct vcd_buffer_pool_type *p_buf_pool;

	VCD_MSG_LOW("vcd_allocate_buffer_cmn in %d:",
		    p_cctxt->clnt_state.e_state);

	rc = vcd_common_allocate_set_buffer(p_cctxt, e_buffer, n_buf_size,
					    &p_buf_pool);

	if (!VCD_FAILED(rc)) {
		rc = vcd_allocate_buffer_internal(p_cctxt,
						  p_buf_pool,
						  n_buf_size,
						  pp_vir_buf_addr,
						  pp_phy_buf_addr);
	}

	return rc;
}

static u32 vcd_free_buffer_cmn
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     enum vcd_buffer_type e_buffer, u8 *p_buffer)
{

	VCD_MSG_LOW("vcd_free_buffer_cmn in %d:", p_cctxt->clnt_state.e_state);

	return vcd_free_one_buffer_internal(p_cctxt, e_buffer, p_buffer);
}

static u32 vcd_fill_output_buffer_cmn
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     struct vcd_frame_data_type *p_buffer)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_entry_type *p_buf_entry;
	struct vcd_frame_data_type *p_frm_entry;
	u32 b_q_result = TRUE;

	VCD_MSG_LOW("vcd_fill_output_buffer_cmn in %d:",
		    p_cctxt->clnt_state.e_state);

	p_buf_entry = vcd_check_fill_output_buffer(p_cctxt, p_buffer);
	if (!p_buf_entry)
		return VCD_ERR_BAD_POINTER;

	b_q_result =
	    vcd_buffer_pool_entry_en_q(&p_cctxt->out_buf_pool, p_buf_entry);

	if (!b_q_result && !p_cctxt->b_decoding) {
		VCD_MSG_ERROR("Failed: vcd_buffer_pool_entry_en_q");

		return VCD_ERR_FAIL;
	}

	p_frm_entry = &p_buf_entry->frame;

	*p_frm_entry = *p_buffer;
	p_frm_entry->p_physical = p_buf_entry->p_physical;
	p_frm_entry->n_ip_frm_tag = VCD_FRAMETAG_INVALID;
	p_frm_entry->n_data_len = 0;

	if (p_cctxt->b_sched_clnt_valid) {
		if (p_cctxt->b_decoding && p_cctxt->status.b1st_frame_recvd) {
			struct vcd_property_hdr_type Prop_hdr;
			struct ddl_frame_data_type_tag ddl_frm;

			Prop_hdr.prop_id = DDL_I_DPB_RELEASE;
			Prop_hdr.n_size =
				sizeof(struct ddl_frame_data_type_tag);

			memset(&ddl_frm, 0, sizeof(ddl_frm));
			ddl_frm.vcd_frm = *p_frm_entry;
			ddl_frm.n_intrlcd_ip_frm_tag = VCD_FRAMETAG_INVALID;

			rc = ddl_set_property(p_cctxt->ddl_handle, &Prop_hdr,
					      &ddl_frm);

			if (VCD_FAILED(rc)) {
				VCD_MSG_ERROR("Error returning output buffer to"
						" HW. rc = 0x%x", rc);

				p_buf_entry->b_in_use = FALSE;
			} else {
				p_cctxt->out_buf_pool.n_in_use++;
				p_buf_entry->b_in_use = TRUE;
			}
		}

		if (!VCD_FAILED(rc)) {
			rc = vcd_map_sched_status(sched_update_client_o_tkn
						  (p_cctxt->p_dev_ctxt->
						   sched_hdl,
						   p_cctxt->sched_clnt_hdl,
						   TRUE,
						   p_cctxt->
						   n_sched_o_tkn_per_ip_frm));
		}

		if (!VCD_FAILED(rc))
			vcd_try_submit_frame(p_cctxt->p_dev_ctxt);

	}

	return rc;
}

static u32 vcd_fill_output_buffer_in_eos
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     struct vcd_frame_data_type *p_buffer)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_entry_type *p_buf_entry;

	VCD_MSG_LOW("vcd_fill_output_buffer_in_eos:");

	p_buf_entry = vcd_check_fill_output_buffer(p_cctxt, p_buffer);
	if (!p_buf_entry)
		return VCD_ERR_BAD_POINTER;

	if (p_cctxt->status.b_eos_wait_for_op_buf) {
		VCD_MSG_HIGH("Got an output buffer we were waiting for");

		p_buf_entry->frame = *p_buffer;

		p_buf_entry->frame.n_data_len = 0;
		p_buf_entry->frame.n_flags |= VCD_FRAME_FLAG_EOS;
		p_buf_entry->frame.n_ip_frm_tag =
		    p_cctxt->status.eos_trig_ip_frm.n_ip_frm_tag;
		p_buf_entry->frame.time_stamp =
		    p_cctxt->status.eos_trig_ip_frm.time_stamp;

		p_cctxt->callback(VCD_EVT_RESP_OUTPUT_DONE,
				  VCD_S_SUCCESS,
				  &p_buf_entry->frame,
				  sizeof(struct vcd_frame_data_type),
				  p_cctxt, p_cctxt->p_client_data);

		p_cctxt->status.b_eos_wait_for_op_buf = FALSE;

		vcd_do_client_state_transition(p_cctxt,
					       VCD_CLIENT_STATE_RUN,
					       CLIENT_STATE_EVENT_NUMBER
					       (pf_fill_output_buffer));

	} else {
		rc = vcd_fill_output_buffer_cmn(p_cctxt, p_buffer);
	}

	return rc;
}

static void vcd_clnt_cb_in_starting
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     u32 event, u32 status, void *p_payload, u32 size,
	 u32 *ddl_handle, void *const p_client_data)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	struct vcd_transc_type *p_transc =
		(struct vcd_transc_type *)p_client_data;
	VCD_MSG_LOW("vcd_clnt_cb_in_starting:");
	if (p_cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("vcd_clnt_cb_in_initing: Wrong DDL handle %p",
			ddl_handle);
		return;
	}

	switch (event) {
	case VCD_EVT_RESP_START:
		{
			vcd_handle_start_done(p_cctxt,
				(struct vcd_transc_type *)p_client_data,
				status);
			break;
		}
	case VCD_EVT_RESP_STOP:
		{
			vcd_handle_stop_done_in_starting(p_cctxt,
				(struct vcd_transc_type *)p_client_data,
				status);
			break;
		}
	case VCD_EVT_IND_HWERRFATAL:
		{
			p_cctxt->status.n_cmd_submitted--;
			vcd_mark_command_channel(p_cctxt->p_dev_ctxt, p_transc);
			vcd_handle_err_fatal(p_cctxt, VCD_EVT_RESP_START,
				status);
			break;
		}
	default:
		{
			VCD_MSG_ERROR("Unexpected callback event=%d status=%d "
				"from DDL",	event, status);
			p_dev_ctxt->b_continue = FALSE;
			break;
		}
	}
}

static void vcd_clnt_cb_in_run
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     u32 event,
     u32 status,
     void *p_payload, u32 n_size, u32 *ddl_handle, void *const p_client_data)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	if (p_cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch");

		return;
	}

	switch (event) {
	case VCD_EVT_RESP_INPUT_DONE:
		{
			rc = vcd_handle_input_done(p_cctxt, p_payload, event,
						   status);

			break;
		}

	case VCD_EVT_RESP_OUTPUT_DONE:
		{
			if (!p_cctxt->status.b1st_op_done_recvd) {
				if (!VCD_FAILED(status)) {
					rc = vcd_handle_first_frame_done
					    (p_cctxt, p_payload);

					if (VCD_FAILED(rc)) {
						VCD_MSG_ERROR
						    ("rc = 0x%x. Failed: "
						     "vcd_handle_first_frame_"
						     "done", rc);

						status = VCD_ERR_FAIL;
					} else {
						p_cctxt->status.
						    b1st_op_done_recvd = TRUE;
					}
				}
			}

			rc = vcd_handle_frame_done(p_cctxt, p_payload, event,
						   status);

			break;
		}
	case VCD_EVT_RESP_OUTPUT_REQ:
		{
			rc = vcd_handle_output_required(p_cctxt, p_payload,
				status);
			break;
		}

	case VCD_EVT_IND_RECONFIG:
		{
			break;
		}
	case VCD_EVT_RESP_TRANSACTION_PENDING:
		{
			 vcd_handle_trans_pending(p_cctxt);
			 break;
		}

	case VCD_EVT_IND_HWERRFATAL:
		{
			 vcd_handle_ind_hw_err_fatal(p_cctxt,
				VCD_EVT_IND_HWERRFATAL, status);
			 break;
		}
	default:
		{
			VCD_MSG_ERROR
			    ("Unexpected callback event=%d status=%d from DDL",
			     event, status);
			p_dev_ctxt->b_continue = FALSE;

			break;
		}
	}

	if (!VCD_FAILED(rc) &&
	    (event == VCD_EVT_RESP_INPUT_DONE ||
	     event == VCD_EVT_RESP_OUTPUT_DONE ||
	     event == VCD_EVT_RESP_OUTPUT_REQ)) {

		if (((struct ddl_frame_data_type_tag *)
					p_payload)->b_frm_trans_end)
			vcd_mark_frame_channel(p_cctxt->p_dev_ctxt);
	}
}

static void vcd_clnt_cb_in_eos
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     u32 event,
     u32 status,
     void *p_payload, u32 n_size, u32 *ddl_handle, void *const p_client_data) {
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	struct vcd_transc_type *p_transc = NULL;

	if (p_cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch");

		return;
	}

	switch (event) {
	case VCD_EVT_RESP_INPUT_DONE:
		{
			vcd_handle_input_done_in_eos(p_cctxt, p_payload,
						     status);

			break;
		}

	case VCD_EVT_RESP_OUTPUT_DONE:
		{
			vcd_handle_frame_done_in_eos(p_cctxt, p_payload,
						     status);

			break;
		}
	case VCD_EVT_RESP_OUTPUT_REQ:
		{
			(void)vcd_handle_output_required(p_cctxt, p_payload,
					status);
			break;
		}
	case VCD_EVT_RESP_EOS_DONE:
		{
			p_transc = (struct vcd_transc_type *)p_client_data;

			vcd_handle_eos_done(p_cctxt, p_transc, status);
			break;
		}
	case VCD_EVT_IND_HWERRFATAL:
		{
			vcd_handle_ind_hw_err_fatal(p_cctxt,
				VCD_EVT_IND_HWERRFATAL,	status);
			break;
		}
	default:
		{
			VCD_MSG_ERROR
			    ("Unexpected callback event=%d status=%d from DDL",
			     event, status);

			p_dev_ctxt->b_continue = FALSE;

			break;
		}

	}
	if (event == VCD_EVT_RESP_INPUT_DONE ||
		event == VCD_EVT_RESP_OUTPUT_DONE ||
		event == VCD_EVT_RESP_OUTPUT_REQ) {
		if (p_payload && ((struct ddl_frame_data_type_tag *)
			p_payload)->b_frm_trans_end) {
			vcd_mark_frame_channel(p_cctxt->p_dev_ctxt);
			if (!p_cctxt->status.n_frame_submitted)
				vcd_handle_eos_trans_end(p_cctxt);
		}
	}
}

static void vcd_clnt_cb_in_flushing
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     u32 event,
     u32 status,
     void *p_payload, u32 size, u32 *ddl_handle, void *const p_client_data) {
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_clnt_cb_in_flushing:");

	if (p_cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch");

		return;
	}

	switch (event) {
	case VCD_EVT_RESP_INPUT_DONE:
		{
			rc = vcd_handle_input_done(p_cctxt,
						   p_payload,
						   VCD_EVT_RESP_INPUT_FLUSHED,
						   status);

			break;
		}

	case VCD_EVT_RESP_OUTPUT_DONE:
		{

			rc = vcd_handle_frame_done(p_cctxt,
						   p_payload,
						   VCD_EVT_RESP_OUTPUT_FLUSHED,
						   status);

			break;
		}
	case VCD_EVT_RESP_OUTPUT_REQ:
		{
			rc = vcd_handle_output_required_in_flushing(p_cctxt,
				p_payload);
			break;
		}
	case VCD_EVT_IND_HWERRFATAL:
		{
			vcd_handle_ind_hw_err_fatal(p_cctxt,
				VCD_EVT_IND_HWERRFATAL,	status);
			break;
		}
	default:
		{
			VCD_MSG_ERROR
			    ("Unexpected callback event=%d status=%d from DDL",
			     event, status);

			p_dev_ctxt->b_continue = FALSE;

			break;
		}
	}
	if (!VCD_FAILED(rc) && ((event == VCD_EVT_RESP_INPUT_DONE ||
		event == VCD_EVT_RESP_OUTPUT_DONE) ||
			event == VCD_EVT_RESP_OUTPUT_REQ)) {
		if (((struct ddl_frame_data_type_tag *)p_payload)->
			b_frm_trans_end) {

			vcd_mark_frame_channel(p_cctxt->p_dev_ctxt);

			if (!p_cctxt->status.n_frame_submitted) {
				VCD_MSG_HIGH
				    ("All pending frames recvd from DDL");

				if (p_cctxt->status.
				    n_flush_mode & VCD_FLUSH_OUTPUT) {
					vcd_flush_output_buffers(p_cctxt);

					vcd_release_all_clnt_frm_transc
					    (p_cctxt);

				}

				vcd_send_flush_done(p_cctxt, VCD_S_SUCCESS);
				vcd_release_interim_frame_channels(p_dev_ctxt);
				VCD_MSG_HIGH("Flush complete");
				vcd_release_all_clnt_def_frm_transc(p_cctxt);
				vcd_do_client_state_transition(p_cctxt,
					VCD_CLIENT_STATE_RUN,
					CLIENT_STATE_EVENT_NUMBER
					(pf_clnt_cb));
			}
		}
	}
}

static void vcd_clnt_cb_in_stopping
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     u32 event,
     u32 status,
     void *p_payload, u32 n_size, u32 *ddl_handle, void *const p_client_data) {
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_clnt_cb_in_stopping:");

	if (p_cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch");

		return;
	}

	switch (event) {

	case VCD_EVT_RESP_INPUT_DONE:
		{
			rc = vcd_handle_input_done(p_cctxt,
						   p_payload,
						   VCD_EVT_RESP_INPUT_FLUSHED,
						   status);

			break;
		}

	case VCD_EVT_RESP_OUTPUT_DONE:
		{

			rc = vcd_handle_frame_done(p_cctxt,
						   p_payload,
						   VCD_EVT_RESP_OUTPUT_FLUSHED,
						   status);

			break;
		}
	case VCD_EVT_RESP_OUTPUT_REQ:
		{
			rc = vcd_handle_output_required_in_flushing(p_cctxt,
				p_payload);
			break;
		}
	case VCD_EVT_RESP_STOP:
		{
			vcd_handle_stop_done(p_cctxt,
					     (struct vcd_transc_type *)
					     p_client_data, status);

			break;
		}
	case VCD_EVT_IND_HWERRFATAL:
		{
			vcd_handle_ind_hw_err_fatal(p_cctxt, VCD_EVT_RESP_STOP,
				status);
			break;
		}

	default:
		{
			VCD_MSG_ERROR
			    ("Unexpected callback event=%d status=%d from DDL",
			     event, status);

			p_dev_ctxt->b_continue = FALSE;

			break;
		}
	}

	if (!VCD_FAILED(rc) && ((event == VCD_EVT_RESP_INPUT_DONE ||
		event == VCD_EVT_RESP_OUTPUT_DONE) ||
		event == VCD_EVT_RESP_OUTPUT_REQ)) {

		if (((struct ddl_frame_data_type_tag *)p_payload)->
			b_frm_trans_end) {

			vcd_mark_frame_channel(p_cctxt->p_dev_ctxt);

			if (!p_cctxt->status.n_frame_submitted) {
				VCD_MSG_HIGH
				    ("All pending frames recvd from DDL");

				vcd_flush_output_buffers(p_cctxt);

				p_cctxt->status.n_flush_mode = 0;

				vcd_release_all_clnt_frm_transc(p_cctxt);

				VCD_MSG_HIGH
				    ("All buffers flushed. Enqueuing stop cmd");

				vcd_client_cmd_flush_and_en_q(p_cctxt,
						VCD_CMD_CODEC_STOP);
			}

		}
	}
}

static void vcd_clnt_cb_in_pausing
    (struct vcd_clnt_ctxt_type_t *p_cctxt,
     u32 event,
     u32 status,
     void *p_payload, u32 size, u32 *ddl_handle, void *const p_client_data)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_clnt_cb_in_pausing:");

	if (p_cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch");

		return;
	}

	switch (event) {
	case VCD_EVT_RESP_INPUT_DONE:
		{
			rc = vcd_handle_input_done(p_cctxt, p_payload, event,
						   status);

			break;
		}

	case VCD_EVT_RESP_OUTPUT_DONE:
		{
			rc = vcd_handle_frame_done(p_cctxt, p_payload, event,
						   status);
			break;
		}
	case VCD_EVT_RESP_OUTPUT_REQ:
		{
			rc = vcd_handle_output_required(p_cctxt, p_payload,
				status);
			break;
		}
	case VCD_EVT_IND_HWERRFATAL:
		{
			vcd_handle_ind_hw_err_fatal(p_cctxt,
				VCD_EVT_RESP_PAUSE,	status);
			break;
		}
	default:
		{
			VCD_MSG_ERROR
			    ("Unexpected callback event=%d status=%d from DDL",
			     event, status);

			p_dev_ctxt->b_continue = FALSE;

			break;
		}

	}

	if (!VCD_FAILED(rc) && ((event == VCD_EVT_RESP_INPUT_DONE ||
	     event == VCD_EVT_RESP_OUTPUT_DONE) ||
		event == VCD_EVT_RESP_OUTPUT_REQ)) {

		if (((struct ddl_frame_data_type_tag *)p_payload)->
			b_frm_trans_end) {

			vcd_mark_frame_channel(p_cctxt->p_dev_ctxt);

			if (!p_cctxt->status.n_frame_submitted) {
				VCD_MSG_HIGH
				    ("All pending frames recvd from DDL");

				p_cctxt->callback(VCD_EVT_RESP_PAUSE,
						  VCD_S_SUCCESS,
						  NULL,
						  0,
						  p_cctxt,
						  p_cctxt->p_client_data);

				vcd_do_client_state_transition(p_cctxt,
						VCD_CLIENT_STATE_PAUSED,
						CLIENT_STATE_EVENT_NUMBER
							       (pf_clnt_cb));

				rc = vcd_power_event(p_cctxt->p_dev_ctxt,
						     p_cctxt,
						     VCD_EVT_PWR_CLNT_PAUSE);

				if (VCD_FAILED(rc)) {
					VCD_MSG_ERROR
					    ("VCD_EVT_PWR_CLNT_PAUSE_END"
					     "failed");
				}
			}
		}
	}
}

static void  vcd_clnt_cb_in_invalid(
   struct vcd_clnt_ctxt_type_t *p_cctxt, u32 event, u32 status,
   void *p_payload, u32 size, u32 *ddl_handle,
   void *const p_client_data
)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	VCD_MSG_LOW("vcd_clnt_cb_in_invalid:");
	if (p_cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch");
		return;
	}
	switch (event) {
	case VCD_EVT_RESP_STOP:
		{
			vcd_handle_stop_done_in_invalid(p_cctxt, status);
			break;
		}
	case VCD_EVT_RESP_INPUT_DONE:
	case VCD_EVT_RESP_OUTPUT_DONE:
	case VCD_EVT_RESP_OUTPUT_REQ:
	case VCD_EVT_RESP_TRANSACTION_PENDING:
		{
			break;
		}
	case VCD_EVT_IND_HWERRFATAL:
		{
			if (status == VCD_ERR_HW_FATAL)
				vcd_handle_stop_done_in_invalid(p_cctxt,
					status);

			break;
		}
	default:
		{
			VCD_MSG_ERROR("Unexpected callback event=%d status=%d"
				"from DDL",	event, status);
			p_dev_ctxt->b_continue = FALSE;
			break;
		}
	}
}

static void vcd_clnt_enter_open
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Entering CLIENT_STATE_OPEN on api %d", n_state_event_type);
}

static void vcd_clnt_enter_starting
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Entering CLIENT_STATE_STARTING on api %d",
		    n_state_event_type);
}

static void vcd_clnt_enter_run
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Entering CLIENT_STATE_RUN on api %d", n_state_event_type);
}

static void vcd_clnt_enter_flushing
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Entering CLIENT_STATE_FLUSHING on api %d",
		    n_state_event_type);
}

static void vcd_clnt_enter_stopping
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Entering CLIENT_STATE_STOPPING on api %d",
		    n_state_event_type);
}

static void vcd_clnt_enter_eos(struct vcd_clnt_ctxt_type_t *p_cctxt,
	s32 n_state_event_type)
{
   u32     rc;

   VCD_MSG_MED("Entering CLIENT_STATE_EOS on api %d", n_state_event_type);
	rc = vcd_map_sched_status(sched_suspend_resume_client(
			p_cctxt->p_dev_ctxt->sched_hdl,
			p_cctxt->sched_clnt_hdl, FALSE));
	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("Failed: sched_suspend_resume_client."
					  "rc=0x%x", rc);
}

static void vcd_clnt_enter_pausing
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Entering CLIENT_STATE_PAUSING on api %d",
		    n_state_event_type);
}

static void vcd_clnt_enter_paused
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type)
{
	VCD_MSG_MED("Entering CLIENT_STATE_PAUSED on api %d",
		n_state_event_type);
}

static void  vcd_clnt_enter_invalid(struct vcd_clnt_ctxt_type_t *p_cctxt,
	s32 n_state_event_type)
{
   VCD_MSG_MED("Entering CLIENT_STATE_INVALID on api %d",
		n_state_event_type);

   p_cctxt->b_ddl_hdl_valid = FALSE;
}

static void vcd_clnt_exit_open
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type)
{
	VCD_MSG_MED("Exiting CLIENT_STATE_OPEN on api %d", n_state_event_type);
}

static void vcd_clnt_exit_starting
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Exiting CLIENT_STATE_STARTING on api %d",
		    n_state_event_type);
}

static void vcd_clnt_exit_run
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Exiting CLIENT_STATE_RUN on api %d", n_state_event_type);
}

static void vcd_clnt_exit_flushing
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Exiting CLIENT_STATE_FLUSHING on api %d",
		    n_state_event_type);
}

static void vcd_clnt_exit_stopping
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Exiting CLIENT_STATE_STOPPING on api %d",
		    n_state_event_type);
}

static void vcd_clnt_exit_eos
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type)
{
	u32 rc;
	VCD_MSG_MED("Exiting CLIENT_STATE_EOS on api %d", n_state_event_type);
	rc = vcd_map_sched_status(sched_suspend_resume_client(
		p_cctxt->p_dev_ctxt->sched_hdl, p_cctxt->sched_clnt_hdl, TRUE));
	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("Failed: sched_suspend_resume_client. rc=0x%x",
			rc);
}

static void vcd_clnt_exit_pausing
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Exiting CLIENT_STATE_PAUSING on api %d",
		    n_state_event_type);
}

static void vcd_clnt_exit_paused
    (struct vcd_clnt_ctxt_type_t *p_cctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Exiting CLIENT_STATE_PAUSED on api %d",
		    n_state_event_type);
}

static void  vcd_clnt_exit_invalid(struct vcd_clnt_ctxt_type_t *p_cctxt,
	s32 n_state_event_type)
{
	VCD_MSG_MED("Exiting CLIENT_STATE_INVALID on api %d",
		n_state_event_type);
}

void vcd_do_client_state_transition(struct vcd_clnt_ctxt_type_t *p_cctxt,
     enum vcd_clnt_state_enum_type e_to_state, u32 n_ev_code)
{
	struct vcd_clnt_state_ctxt_type_t *p_state_ctxt;

	if (!p_cctxt || e_to_state >= VCD_CLIENT_STATE_MAX) {
		VCD_MSG_ERROR("Bad parameters. p_cctxt=%p, e_to_state=%d",
			      p_cctxt, e_to_state);
	}

	p_state_ctxt = &p_cctxt->clnt_state;

	if (p_state_ctxt->e_state == e_to_state) {
		VCD_MSG_HIGH("Client already in requested e_to_state=%d",
			     e_to_state);

		return;
	}

	VCD_MSG_MED("vcd_do_client_state_transition: C%d -> C%d, for api %d",
		    (int)p_state_ctxt->e_state, (int)e_to_state, n_ev_code);

	if (p_state_ctxt->p_state_table->pf_exit)
		p_state_ctxt->p_state_table->pf_exit(p_cctxt, n_ev_code);


	p_state_ctxt->e_state = e_to_state;
	p_state_ctxt->p_state_table = vcd_clnt_state_table[e_to_state];

	if (p_state_ctxt->p_state_table->pf_entry)
		p_state_ctxt->p_state_table->pf_entry(p_cctxt, n_ev_code);
}

const struct vcd_clnt_state_table_type_t *vcd_get_client_state_table
    (enum vcd_clnt_state_enum_type e_state) {
	return vcd_clnt_state_table[e_state];
}

static const struct vcd_clnt_state_table_type_t vcd_clnt_table_open = {
	{
	 vcd_close_in_open,
	 vcd_encode_start_in_open,
	 NULL,
	 vcd_decode_start_in_open,
	 NULL,
	 NULL,
	 NULL,
	 vcd_flush_inopen,
	 vcd_stop_inopen,
	 vcd_set_property_cmn,
	 vcd_get_property_cmn,
	 vcd_set_buffer_requirements_cmn,
	 vcd_get_buffer_requirements_cmn,
	 vcd_set_buffer_cmn,
	 vcd_allocate_buffer_cmn,
	 vcd_free_buffer_cmn,
	 vcd_fill_output_buffer_cmn,
	 NULL,
	 },
	vcd_clnt_enter_open,
	vcd_clnt_exit_open
};

static const struct vcd_clnt_state_table_type_t vcd_clnt_table_starting = {
	{
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 vcd_clnt_cb_in_starting,
	 },
	vcd_clnt_enter_starting,
	vcd_clnt_exit_starting
};

static const struct vcd_clnt_state_table_type_t vcd_clnt_table_run = {
	{
	 NULL,
	 vcd_encode_start_in_run,
	 vcd_encode_frame_cmn,
	 vcd_decode_start_in_run,
	 vcd_decode_frame_cmn,
	 vcd_pause_in_run,
	 NULL,
	 vcd_flush_cmn,
	 vcd_stop_in_run,
	 vcd_set_property_cmn,
	 vcd_get_property_cmn,
	 vcd_set_buffer_requirements_cmn,
	 vcd_get_buffer_requirements_cmn,
	 vcd_set_buffer_cmn,
	 vcd_allocate_buffer_cmn,
	 NULL,
	 vcd_fill_output_buffer_cmn,
	 vcd_clnt_cb_in_run,
	 },
	vcd_clnt_enter_run,
	vcd_clnt_exit_run
};

static const struct vcd_clnt_state_table_type_t vcd_clnt_table_flushing = {
	{
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 vcd_flush_in_flushing,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 vcd_clnt_cb_in_flushing,
	 },
	vcd_clnt_enter_flushing,
	vcd_clnt_exit_flushing
};

static const struct vcd_clnt_state_table_type_t vcd_clnt_table_stopping = {
	{
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 vcd_get_property_cmn,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 vcd_clnt_cb_in_stopping,
	 },
	vcd_clnt_enter_stopping,
	vcd_clnt_exit_stopping
};

static const struct vcd_clnt_state_table_type_t vcd_clnt_table_eos = {
	{
	 NULL,
	 NULL,
	 vcd_encode_frame_cmn,
	 NULL,
	 vcd_decode_frame_cmn,
	 NULL,
	 NULL,
	 vcd_flush_in_eos,
	 vcd_stop_in_eos,
	 NULL,
	 vcd_get_property_cmn,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 vcd_fill_output_buffer_in_eos,
	 vcd_clnt_cb_in_eos,
	 },
	vcd_clnt_enter_eos,
	vcd_clnt_exit_eos
};

static const struct vcd_clnt_state_table_type_t vcd_clnt_table_pausing = {
	{
	 NULL,
	 NULL,
	 vcd_encode_frame_cmn,
	 NULL,
	 vcd_decode_frame_cmn,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 vcd_get_property_cmn,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 vcd_fill_output_buffer_cmn,
	 vcd_clnt_cb_in_pausing,
	 },
	vcd_clnt_enter_pausing,
	vcd_clnt_exit_pausing
};

static const struct vcd_clnt_state_table_type_t vcd_clnt_table_paused = {
	{
	 NULL,
	 NULL,
	 vcd_encode_frame_cmn,
	 NULL,
	 vcd_decode_frame_cmn,
	 NULL,
	 vcd_resume_in_paused,
	 vcd_flush_cmn,
	 vcd_stop_cmn,
	 vcd_set_property_cmn,
	 vcd_get_property_cmn,
	 vcd_set_buffer_requirements_cmn,
	 vcd_get_buffer_requirements_cmn,
	 vcd_set_buffer_cmn,
	 vcd_allocate_buffer_cmn,
	 NULL,
	 vcd_fill_output_buffer_cmn,
	 NULL,
	 },
	vcd_clnt_enter_paused,
	vcd_clnt_exit_paused
};
static const struct vcd_clnt_state_table_type_t vcd_clnt_table_invalid = {
   {
      vcd_close_in_invalid,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      vcd_flush_in_invalid,
      vcd_stop_in_invalid,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      vcd_free_buffer_cmn,
      NULL,
      vcd_clnt_cb_in_invalid,
   },
   vcd_clnt_enter_invalid,
   vcd_clnt_exit_invalid
};

static const struct vcd_clnt_state_table_type_t *vcd_clnt_state_table[] = {
	NULL,
	&vcd_clnt_table_open,
	&vcd_clnt_table_starting,
	&vcd_clnt_table_run,
	&vcd_clnt_table_flushing,
	&vcd_clnt_table_pausing,
	&vcd_clnt_table_paused,
	&vcd_clnt_table_stopping,
	&vcd_clnt_table_eos,
   &vcd_clnt_table_invalid
};
