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

static const struct vcd_clnt_state_table *vcd_clnt_state_table[];

void vcd_clnt_handle_device_err_fatal(struct vcd_clnt_ctxt *cctxt, u32 event)
{
	if (cctxt->clnt_state.state != VCD_CLIENT_STATE_INVALID) {
		cctxt->callback(event, VCD_ERR_HW_FATAL, NULL, 0,
			cctxt, cctxt->client_data);
		vcd_flush_buffers_in_err_fatal(cctxt);
		vcd_do_client_state_transition(cctxt,
			VCD_CLIENT_STATE_INVALID,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	}
}

static u32 vcd_close_in_open(struct vcd_clnt_ctxt *cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_close_in_open:");
	if (cctxt->in_buf_pool.allocated || cctxt->out_buf_pool.allocated) {
		VCD_MSG_ERROR("Allocated buffers are not freed yet\n");
		return VCD_ERR_ILLEGAL_OP;
	}
	vcd_destroy_client_context(cctxt);
	return rc;
}

static u32  vcd_close_in_invalid(struct vcd_clnt_ctxt *cctxt)
{
	VCD_MSG_LOW("vcd_close_in_invalid:\n");
	if (cctxt->in_buf_pool.allocated || cctxt->out_buf_pool.allocated) {
		VCD_MSG_ERROR("Allocated buffers are not freed yet\n");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (cctxt->status.cleaning_up)
		cctxt->status.close_pending = true;
	else
		vcd_destroy_client_context(cctxt);
	return VCD_S_SUCCESS;
}

static u32 vcd_start_in_run_cmn(struct vcd_clnt_ctxt *cctxt)
{
	VCD_MSG_LOW("vcd_start_in_run_cmn:\n");
	cctxt->callback(VCD_EVT_RESP_START, VCD_S_SUCCESS, NULL, 0, cctxt,
		cctxt->client_data);
	return VCD_S_SUCCESS;
}

static u32 vcd_encode_start_in_open(struct vcd_clnt_ctxt *cctxt)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_property_hdr prop_hdr;
	struct vcd_property_vop_timing timing;

	VCD_MSG_LOW("vcd_encode_start_in_open:\n");

	if (cctxt->decoding) {
		VCD_MSG_ERROR("vcd_encode_init for decoder client\n");

		return VCD_ERR_ILLEGAL_OP;
	}

	if (!cctxt->in_buf_pool.entries || !cctxt->out_buf_pool.entries ||
			cctxt->in_buf_pool.validated !=
			cctxt->in_buf_pool.count ||
			cctxt->out_buf_pool.validated !=
			cctxt->out_buf_pool.count) {
		VCD_MSG_ERROR("Buffer pool is not completely setup yet\n");

		return VCD_ERR_BAD_STATE;
	}

	rc = vcd_add_client_to_sched(cctxt);

	VCD_FAILED_RETURN(rc, "Failed: vcd_add_client_to_sched\n");

	prop_hdr.id = VCD_I_VOP_TIMING;
	prop_hdr.sz = sizeof(struct vcd_property_vop_timing);
	rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr, &timing);

	VCD_FAILED_RETURN(rc, "Failed: Get VCD_I_VOP_TIMING\n");
	if (!timing.vop_time_resolution) {
		VCD_MSG_ERROR("Vop_time_resolution value is zero\n");
		return VCD_ERR_FAIL;
	}
	cctxt->time_resoln = timing.vop_time_resolution;

	rc = vcd_process_cmd_sess_start(cctxt);

	if (!VCD_FAILED(rc)) {
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_STARTING,
			CLIENT_STATE_EVENT_NUMBER(pf_encode_start));
	}

	return rc;
}

static u32  vcd_encode_start_in_run(struct vcd_clnt_ctxt *cctxt)
{
	VCD_MSG_LOW("vcd_encode_start_in_run:\n");
	vcd_start_in_run_cmn(cctxt);
	return VCD_S_SUCCESS;
}


static u32 vcd_encode_frame_cmn(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame)
{
	VCD_MSG_LOW("vcd_encode_frame_cmn in %d:", cctxt->clnt_state.state);

	if (cctxt->decoding) {
		VCD_MSG_ERROR("vcd_encode_frame for decoder client\n");

		return VCD_ERR_ILLEGAL_OP;
	}

	return vcd_handle_input_frame(cctxt, input_frame);
}

static u32 vcd_decode_start_in_open(struct vcd_clnt_ctxt *cctxt,
	struct vcd_sequence_hdr *seq_hdr)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_decode_start_in_open:\n");

	if (!cctxt->decoding) {
		VCD_MSG_ERROR("vcd_decode_init for encoder client\n");

		return VCD_ERR_ILLEGAL_OP;
	}

	if (seq_hdr) {
		VCD_MSG_HIGH("Seq hdr supplied. len = %d\n", seq_hdr->sz);
		rc = vcd_store_seq_hdr(cctxt, seq_hdr);

	} else {
		VCD_MSG_HIGH("Seq hdr not supplied\n");
		cctxt->seq_hdr.sz = 0;
		cctxt->seq_hdr.addr = NULL;
	}

	VCD_FAILED_RETURN(rc, "Err processing seq hdr\n");

	rc = vcd_process_cmd_sess_start(cctxt);

	if (!VCD_FAILED(rc)) {
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_STARTING,
			CLIENT_STATE_EVENT_NUMBER(pf_decode_start));
	}

	return rc;
}

static u32 vcd_decode_start_in_run(struct vcd_clnt_ctxt *cctxt,
	struct vcd_sequence_hdr *seqhdr)
{
	VCD_MSG_LOW("vcd_decode_start_in_run:\n");
	vcd_start_in_run_cmn(cctxt);
	return VCD_S_SUCCESS;
}

static u32 vcd_decode_frame_cmn(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame)
{
	VCD_MSG_LOW("vcd_decode_frame_cmn in %d:\n", cctxt->clnt_state.state);

	if (!cctxt->decoding) {
		VCD_MSG_ERROR("Decode_frame api called for Encoder client\n");

		return VCD_ERR_ILLEGAL_OP;
	}

	return vcd_handle_input_frame(cctxt, input_frame);
}

static u32 vcd_pause_in_run(struct vcd_clnt_ctxt *cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_pause_in_run:\n");

	if (cctxt->sched_clnt_valid) {
		rc = vcd_map_sched_status(sched_suspend_resume_client(
			cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
			false));
	}

	VCD_FAILED_RETURN(rc, "Failed: sched_suspend_resume_client\n");

	if (cctxt->status.frame_submitted > 0) {
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_PAUSING,
			CLIENT_STATE_EVENT_NUMBER(pf_pause));

	} else {
		VCD_MSG_HIGH("No client frames are currently being processed\n");

		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_PAUSED,
			CLIENT_STATE_EVENT_NUMBER(pf_pause));

		cctxt->callback(VCD_EVT_RESP_PAUSE, VCD_S_SUCCESS, NULL, 0,
			cctxt, cctxt->client_data);

		rc = vcd_power_event(cctxt->dev_ctxt, cctxt,
			VCD_EVT_PWR_CLNT_PAUSE);

		if (VCD_FAILED(rc))
			VCD_MSG_ERROR("VCD_EVT_PWR_CLNT_PAUSE_END failed\n");

	}

	return VCD_S_SUCCESS;
}

static u32 vcd_resume_in_paused(struct vcd_clnt_ctxt *cctxt)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_resume_in_paused:\n");

	if (cctxt->sched_clnt_valid) {

		rc = vcd_power_event(cctxt->dev_ctxt, cctxt,
			VCD_EVT_PWR_CLNT_RESUME);

		if (VCD_FAILED(rc)) {
			VCD_MSG_ERROR("VCD_EVT_PWR_CLNT_RESUME failed\n");
		} else {
			rc = vcd_map_sched_status(sched_suspend_resume_client(
				cctxt->dev_ctxt->sched_hdl,
				cctxt->sched_clnt_hdl, true));
			if (VCD_FAILED(rc)) {
				VCD_MSG_ERROR("rc = 0x%x. Failed: "
				     "sched_suspend_resume_client\n", rc);
			}

		}
		if (!VCD_FAILED(rc))
			vcd_try_submit_frame(dev_ctxt);
	}

	if (!VCD_FAILED(rc)) {
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_RUN,
			CLIENT_STATE_EVENT_NUMBER(pf_resume));
	}

	return rc;
}

static u32 vcd_flush_cmn(struct vcd_clnt_ctxt *cctxt, u32 mode)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_flush_cmn in %d:\n", cctxt->clnt_state.state);

	rc = vcd_flush_buffers(cctxt, mode);

	VCD_FAILED_RETURN(rc, "Failed: vcd_flush_buffers\n");

	if (cctxt->status.frame_submitted > 0) {
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_FLUSHING,
			CLIENT_STATE_EVENT_NUMBER(pf_flush));
	} else {
		VCD_MSG_HIGH("All buffers are flushed\n");
		cctxt->status.flush_mode = mode;
		vcd_send_flush_done(cctxt, VCD_S_SUCCESS);
	}

	return rc;
}

static u32  vcd_flush_inopen(struct vcd_clnt_ctxt *cctxt, u32 mode)
{
	VCD_MSG_LOW("vcd_flush_inopen:\n");
	cctxt->status.flush_mode = mode;
	vcd_send_flush_done(cctxt, VCD_S_SUCCESS);
	return VCD_S_SUCCESS;
}

static u32 vcd_flush_in_flushing(struct vcd_clnt_ctxt *cctxt, u32 mode)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_flush_in_flushing:\n");

	rc = vcd_flush_buffers(cctxt, mode);

	return rc;
}

static u32 vcd_flush_in_eos(struct vcd_clnt_ctxt *cctxt, u32 mode)
{
	VCD_MSG_LOW("vcd_flush_in_eos:\n");

	if (mode > VCD_FLUSH_ALL || !mode) {
		VCD_MSG_ERROR("Invalid flush mode %d\n", mode);

		return VCD_ERR_ILLEGAL_PARM;
	}

	VCD_MSG_MED("Flush mode requested %d\n", mode);

	cctxt->status.flush_mode |= mode;

	return VCD_S_SUCCESS;
}

static u32 vcd_flush_in_invalid(struct vcd_clnt_ctxt *cctxt, u32 mode)
{
	u32 rc = VCD_S_SUCCESS;
	VCD_MSG_LOW("vcd_flush_in_invalid:\n");
	if (!cctxt->status.cleaning_up) {
		rc = vcd_flush_buffers(cctxt, mode);
		if (!VCD_FAILED(rc)) {
			VCD_MSG_HIGH("All buffers are flushed\n");
			cctxt->status.flush_mode = mode;
			vcd_send_flush_done(cctxt, VCD_S_SUCCESS);
		}
	}
	return rc;
}

static u32 vcd_stop_cmn(struct vcd_clnt_ctxt *cctxt)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u32 rc = VCD_S_SUCCESS;
	struct vcd_transc *transc;

	VCD_MSG_LOW("vcd_stop_cmn in %d:\n", cctxt->clnt_state.state);

	rc = vcd_flush_buffers(cctxt, VCD_FLUSH_ALL);

	VCD_FAILED_RETURN(rc, "Failed: vcd_flush_buffers\n");

	if (!cctxt->status.frame_submitted) {

		if (vcd_get_command_channel(dev_ctxt, &transc)) {
			rc = vcd_power_event(dev_ctxt, cctxt,
				VCD_EVT_PWR_CLNT_CMD_BEGIN);

			if (!VCD_FAILED(rc)) {
				transc->type = VCD_CMD_CODEC_STOP;
				transc->cctxt = cctxt;

				rc = vcd_submit_cmd_sess_end(transc);
			} else {
				VCD_MSG_ERROR("Failed:"
					" VCD_EVT_PWR_CLNT_CMD_BEGIN\n");
			}

			if (VCD_FAILED(rc))
				vcd_release_command_channel(dev_ctxt, transc);

		} else {
			vcd_client_cmd_flush_and_en_q(cctxt,
				VCD_CMD_CODEC_STOP);
		}
	}

	if (VCD_FAILED(rc)) {
		vcd_power_event(dev_ctxt, cctxt, VCD_EVT_PWR_CLNT_CMD_FAIL);
	} else {
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_STOPPING,
			CLIENT_STATE_EVENT_NUMBER(pf_stop));
	}

	return rc;
}


static u32  vcd_stop_inopen(struct vcd_clnt_ctxt *cctxt)
{
	VCD_MSG_LOW("vcd_stop_inopen:\n");

	cctxt->callback(VCD_EVT_RESP_STOP, VCD_S_SUCCESS, NULL, 0, cctxt,
		cctxt->client_data);

	return VCD_S_SUCCESS;
}

static u32 vcd_stop_in_run(struct vcd_clnt_ctxt *cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_stop_in_run:\n");

	rc = vcd_stop_cmn(cctxt);

	if (!VCD_FAILED(rc) && cctxt->status.b1st_frame_recvd) {
		rc = vcd_power_event(cctxt->dev_ctxt, cctxt,
			VCD_EVT_PWR_CLNT_LAST_FRAME);
	}

	return rc;
}

static u32 vcd_stop_in_eos(struct vcd_clnt_ctxt *cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_stop_in_eos:\n");

	cctxt->status.stop_pending = true;

	return rc;
}

static u32  vcd_stop_in_invalid(struct vcd_clnt_ctxt *cctxt)
{
	VCD_MSG_LOW("vcd_stop_in_invalid:\n");
	if (cctxt->status.cleaning_up) {
		cctxt->status.stop_pending = true;
	} else {
		vcd_flush_buffers(cctxt, VCD_FLUSH_ALL);
		cctxt->callback(VCD_EVT_RESP_STOP, VCD_S_SUCCESS, NULL, 0,
			cctxt, cctxt->client_data);
	}
	return VCD_S_SUCCESS;
}

static u32 vcd_set_property_cmn(struct vcd_clnt_ctxt *cctxt,
	struct vcd_property_hdr *prop_hdr, void *prop_val)
{
	u32 rc;

	VCD_MSG_LOW("vcd_set_property_cmn in %d:\n", cctxt->clnt_state.state);
	VCD_MSG_LOW("property Id = %d\n", prop_hdr->id);

	if (!prop_hdr->sz || !prop_hdr->id) {
		VCD_MSG_MED("Bad parameters\n");

		return VCD_ERR_ILLEGAL_PARM;
	}

	rc = ddl_set_property(cctxt->ddl_handle, prop_hdr, prop_val);

	VCD_FAILED_RETURN(rc, "Failed: ddl_set_property\n");

	switch (prop_hdr->id) {
	case VCD_I_LIVE:
	{
		struct vcd_property_live *live = (struct vcd_property_live *)
			prop_val;
		cctxt->live = live->live;
		break;
	}
	case VCD_I_FRAME_RATE:
		if (cctxt->sched_clnt_valid) {
			rc = vcd_set_frame_rate(cctxt,
				(struct vcd_property_frame_rate *)prop_val);
		}
		break;
	case VCD_I_FRAME_SIZE:
		if (cctxt->sched_clnt_valid) {
			rc = vcd_set_frame_size(cctxt,
				(struct vcd_property_frame_size *)prop_val);
		}
		break;
	default:
		break;
	}

	return rc;
}

static u32 vcd_get_property_cmn(struct vcd_clnt_ctxt *cctxt,
	struct vcd_property_hdr *prop_hdr, void *prop_val)
{
	VCD_MSG_LOW("vcd_get_property_cmn in %d:\n", cctxt->clnt_state.state);
	VCD_MSG_LOW("property id = %d\n", prop_hdr->id);
	if (!prop_hdr->sz || !prop_hdr->id) {
		VCD_MSG_MED("Bad parameters\n");

		return VCD_ERR_ILLEGAL_PARM;
	}
	return ddl_get_property(cctxt->ddl_handle, prop_hdr, prop_val);
}

static u32 vcd_set_buffer_requirements_cmn(struct vcd_clnt_ctxt *cctxt,
	enum vcd_buffer_type vcd_buffer_type,
	struct vcd_buffer_requirement *buffer_req)
{
	struct vcd_property_hdr prop_hdr;
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_pool *buf_pool;

	VCD_MSG_LOW("vcd_set_buffer_requirements_cmn in %d:\n",
		    cctxt->clnt_state.state);

	if (!cctxt->decoding && cctxt->clnt_state.state !=
			VCD_CLIENT_STATE_OPEN) {
		VCD_MSG_ERROR("Bad state (%d) for encoder\n",
			cctxt->clnt_state.state);

		return VCD_ERR_BAD_STATE;
	}

	VCD_MSG_MED("Buffer type = %d\n", vcd_buffer_type);

	if (vcd_buffer_type == VCD_BUFFER_INPUT) {
		prop_hdr.id = DDL_I_INPUT_BUF_REQ;
		buf_pool = &cctxt->in_buf_pool;
	} else if (vcd_buffer_type == VCD_BUFFER_OUTPUT) {
		prop_hdr.id = DDL_I_OUTPUT_BUF_REQ;
		buf_pool = &cctxt->out_buf_pool;
	} else {
		rc = VCD_ERR_ILLEGAL_PARM;
	}

	VCD_FAILED_RETURN(rc, "Invalid buffer type provided\n");

	if (buf_pool->validated > 0) {
		VCD_MSG_ERROR("Need to free allocated buffers\n");

		return VCD_ERR_ILLEGAL_OP;
	}

	prop_hdr.sz = sizeof(*buffer_req);

	rc = ddl_set_property(cctxt->ddl_handle, &prop_hdr, buffer_req);

	VCD_FAILED_RETURN(rc, "Failed: ddl_set_property\n");

	if (buf_pool->entries) {
		VCD_MSG_MED("Resetting buffer requirements\n");

		vcd_free_buffer_pool_entries(buf_pool);
	}

	rc = vcd_alloc_buffer_pool_entries(buf_pool, buffer_req);

	return rc;
}

static u32 vcd_get_buffer_requirements_cmn(struct vcd_clnt_ctxt *cctxt,
	enum vcd_buffer_type vcd_buffer_type,
	struct vcd_buffer_requirement *buffer_req)
{
	struct vcd_property_hdr prop_hdr;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_get_buffer_requirements_cmn in %d:\n",
		    cctxt->clnt_state.state);

	VCD_MSG_MED("Buffer type = %d\n", vcd_buffer_type);

	if (vcd_buffer_type == VCD_BUFFER_INPUT)
		prop_hdr.id = DDL_I_INPUT_BUF_REQ;
	else if (vcd_buffer_type == VCD_BUFFER_OUTPUT)
		prop_hdr.id = DDL_I_OUTPUT_BUF_REQ;
	else
		rc = VCD_ERR_ILLEGAL_PARM;

	VCD_FAILED_RETURN(rc, "Invalid buffer type provided\n");

	prop_hdr.sz = sizeof(*buffer_req);

	return ddl_get_property(cctxt->ddl_handle, &prop_hdr, buffer_req);

}

static u32 vcd_set_buffer_cmn(struct vcd_clnt_ctxt *cctxt,
	enum vcd_buffer_type vcd_buffer_type, void *buf, size_t sz)
{
	u32 rc;
	struct vcd_buffer_pool *buf_pool;

	VCD_MSG_LOW("vcd_set_buffer_cmn in %d:\n", cctxt->clnt_state.state);

	rc = vcd_common_allocate_set_buffer(cctxt, vcd_buffer_type, sz,
		&buf_pool);

	if (!VCD_FAILED(rc)) {
		rc = vcd_set_buffer_internal(cctxt, buf_pool, buf, sz);
	}

	return rc;
}

static u32 vcd_allocate_buffer_cmn(struct vcd_clnt_ctxt *cctxt,
	enum vcd_buffer_type vcd_buffer_type, size_t sz, void **virt_addr,
	phys_addr_t *phys_addr)
{
	u32 rc;
	struct vcd_buffer_pool *buf_pool;

	VCD_MSG_LOW("vcd_allocate_buffer_cmn in %d:\n",
		    cctxt->clnt_state.state);

	rc = vcd_common_allocate_set_buffer(cctxt, vcd_buffer_type, sz,
		&buf_pool);

	if (!VCD_FAILED(rc)) {
		rc = vcd_allocate_buffer_internal(cctxt, buf_pool, sz,
			virt_addr, phys_addr);
	}

	return rc;
}

static u32 vcd_free_buffer_cmn(struct vcd_clnt_ctxt *cctxt,
	enum vcd_buffer_type vcd_buffer_type, void *buf)
{
	VCD_MSG_LOW("vcd_free_buffer_cmn in %d:\n", cctxt->clnt_state.state);

	return vcd_free_one_buffer_internal(cctxt, vcd_buffer_type, buf);
}

static u32 vcd_fill_output_buffer_cmn(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *buffer)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_entry *buf_entry;
	struct vcd_frame_data *frm_entry;
	u32 q_result = true;

	VCD_MSG_LOW("vcd_fill_output_buffer_cmn in %d:\n",
		    cctxt->clnt_state.state);

	buf_entry = vcd_check_fill_output_buffer(cctxt, buffer);
	if (!buf_entry)
		return VCD_ERR_BAD_POINTER;

	q_result = vcd_buffer_pool_entry_en_q(&cctxt->out_buf_pool, buf_entry);

	if (!q_result && !cctxt->decoding) {
		VCD_MSG_ERROR("Failed: vcd_buffer_pool_entry_en_q\n");

		return VCD_ERR_FAIL;
	}

	frm_entry = &buf_entry->frame;

	*frm_entry = *buffer;
	frm_entry->phys_addr = buf_entry->phys_addr;
	frm_entry->ip_frm_tag = VCD_FRAMETAG_INVALID;
	frm_entry->data_len = 0;

	if (cctxt->sched_clnt_valid) {
		if (cctxt->decoding && cctxt->status.b1st_frame_recvd) {
			struct vcd_property_hdr prop_hdr;
			struct ddl_frame_data_tag ddl_frm;

			prop_hdr.id = DDL_I_DPB_RELEASE;
			prop_hdr.sz = sizeof(struct ddl_frame_data_tag);

			memset(&ddl_frm, 0, sizeof(ddl_frm));
			ddl_frm.vcd_frm = *frm_entry;
			ddl_frm.intrlcd_ip_frm_tag = VCD_FRAMETAG_INVALID;

			rc = ddl_set_property(cctxt->ddl_handle, &prop_hdr,
				&ddl_frm);

			if (VCD_FAILED(rc)) {
				VCD_MSG_ERROR("Error returning output buffer to"
						" HW. rc = 0x%x\n", rc);

				buf_entry->in_use = false;
			} else {
				cctxt->out_buf_pool.in_use++;
				buf_entry->in_use = true;
			}
		}

		if (!VCD_FAILED(rc)) {
			rc = vcd_map_sched_status(sched_update_client_o_tkn(
				cctxt->dev_ctxt->sched_hdl,
				cctxt->sched_clnt_hdl, true,
				cctxt->sched_o_tkn_per_ip_frm));
		}

		if (!VCD_FAILED(rc))
			vcd_try_submit_frame(cctxt->dev_ctxt);

	}

	return rc;
}

static u32 vcd_fill_output_buffer_in_eos(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *buffer)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_entry *buf_entry;

	VCD_MSG_LOW("vcd_fill_output_buffer_in_eos:\n");

	buf_entry = vcd_check_fill_output_buffer(cctxt, buffer);
	if (!buf_entry)
		return VCD_ERR_BAD_POINTER;

	if (cctxt->status.eos_wait_for_op_buf) {
		VCD_MSG_HIGH("Got an output buffer we were waiting for\n");

		buf_entry->frame = *buffer;

		buf_entry->frame.data_len = 0;
		buf_entry->frame.flags |= VCD_FRAME_FLAG_EOS;
		buf_entry->frame.ip_frm_tag =
			cctxt->status.eos_trig_ip_frm.ip_frm_tag;
		buf_entry->frame.time_stamp =
			cctxt->status.eos_trig_ip_frm.time_stamp;

		cctxt->callback(VCD_EVT_RESP_OUTPUT_DONE, VCD_S_SUCCESS,
			&buf_entry->frame, sizeof(struct vcd_frame_data),
			cctxt, cctxt->client_data);

		cctxt->status.eos_wait_for_op_buf = false;

		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_RUN,
			CLIENT_STATE_EVENT_NUMBER(pf_fill_output_buffer));

	} else {
		rc = vcd_fill_output_buffer_cmn(cctxt, buffer);
	}

	return rc;
}

static void vcd_clnt_cb_in_starting(struct vcd_clnt_ctxt *cctxt, u32 event,
	u32 status, void *payload, u32 size, u32 *ddl_handle,
	void *const client_data)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	struct vcd_transc *transc = (struct vcd_transc *)client_data;
	VCD_MSG_LOW("vcd_clnt_cb_in_starting:\n");
	if (cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("vcd_clnt_cb_in_initing: Wrong DDL handle %p\n",
			ddl_handle);
		return;
	}

	switch (event) {
	case VCD_EVT_RESP_START:
		vcd_handle_start_done(cctxt, (struct vcd_transc *)client_data,
			status);
		break;
	case VCD_EVT_RESP_STOP:
		vcd_handle_stop_done_in_starting(cctxt, (struct vcd_transc *)
			client_data, status);
		break;
	case VCD_EVT_IND_HWERRFATAL:
		cctxt->status.cmd_submitted--;
		vcd_mark_command_channel(cctxt->dev_ctxt, transc);
		vcd_handle_err_fatal(cctxt, VCD_EVT_RESP_START,	status);
		break;
	default:
		VCD_MSG_ERROR("Unexpected callback event=%d status=%d "
			"from DDL\n", event, status);
		dev_ctxt->cont = false;
		break;
	}
}

static void vcd_clnt_cb_in_run(struct vcd_clnt_ctxt *cctxt, u32 event,
	u32 status, void *payload, u32 size, u32 *ddl_handle,
	void *const client_data)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	if (cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch\n");

		return;
	}

	switch (event) {
	case VCD_EVT_RESP_INPUT_DONE:
		rc = vcd_handle_input_done(cctxt, payload, event, status);
		break;
	case VCD_EVT_RESP_OUTPUT_DONE:
		if (!cctxt->status.b1st_op_done_recvd) {
			if (!VCD_FAILED(status)) {
				rc = vcd_handle_first_frame_done(cctxt,
					payload);

				if (VCD_FAILED(rc)) {
					VCD_MSG_ERROR("rc = 0x%x. Failed: "
						"vcd_handle_first_frame_"
						"done\n", rc);
					status = VCD_ERR_FAIL;
				} else {
					cctxt->status.b1st_op_done_recvd = true;
				}
			}
		}

		rc = vcd_handle_frame_done(cctxt, payload, event, status);

		break;
	case VCD_EVT_RESP_OUTPUT_REQ:
		rc = vcd_handle_output_required(cctxt, payload, status);
		break;
	case VCD_EVT_IND_RECONFIG:
		break;
	case VCD_EVT_RESP_TRANSACTION_PENDING:
		vcd_handle_trans_pending(cctxt);
		break;
	case VCD_EVT_IND_HWERRFATAL:
		vcd_handle_ind_hw_err_fatal(cctxt, VCD_EVT_IND_HWERRFATAL,
			status);
		break;
	default:
		VCD_MSG_ERROR("Unexpected callback event=%d status=%d from DDL\n",
			event, status);
		dev_ctxt->cont = false;
		break;
	}

	if (!VCD_FAILED(rc) && (event == VCD_EVT_RESP_INPUT_DONE ||
			event == VCD_EVT_RESP_OUTPUT_DONE ||
			event == VCD_EVT_RESP_OUTPUT_REQ)) {

		if (((struct ddl_frame_data_tag *)payload)->frm_trans_end)
			vcd_mark_frame_channel(cctxt->dev_ctxt);
	}
}

static void vcd_clnt_cb_in_eos(struct vcd_clnt_ctxt *cctxt, u32 event,
	u32 status, void *payload, u32 size, u32 *ddl_handle,
	void *const client_data)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	struct vcd_transc *transc = NULL;

	if (cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch\n");

		return;
	}

	switch (event) {
	case VCD_EVT_RESP_INPUT_DONE:
		vcd_handle_input_done_in_eos(cctxt, payload, status);
		break;
	case VCD_EVT_RESP_OUTPUT_DONE:
		vcd_handle_frame_done_in_eos(cctxt, payload, status);
		break;
	case VCD_EVT_RESP_OUTPUT_REQ:
		vcd_handle_output_required(cctxt, payload, status);
		break;
	case VCD_EVT_RESP_EOS_DONE:
		transc = (struct vcd_transc *)client_data;
		vcd_handle_eos_done(cctxt, transc, status);
		break;
	case VCD_EVT_IND_HWERRFATAL:
		vcd_handle_ind_hw_err_fatal(cctxt, VCD_EVT_IND_HWERRFATAL,
			status);
		break;
	default:
		VCD_MSG_ERROR("Unexpected callback event=%d status=%d from DDL\n",
			event, status);

		dev_ctxt->cont = false;
		break;
	}

	if (event == VCD_EVT_RESP_INPUT_DONE ||
			event == VCD_EVT_RESP_OUTPUT_DONE ||
			event == VCD_EVT_RESP_OUTPUT_REQ) {
		if (payload && ((struct ddl_frame_data_tag *)
				payload)->frm_trans_end) {
			vcd_mark_frame_channel(cctxt->dev_ctxt);
			if (!cctxt->status.frame_submitted)
				vcd_handle_eos_trans_end(cctxt);
		}
	}
}

static void vcd_clnt_cb_in_flushing(struct vcd_clnt_ctxt *cctxt, u32 event,
	u32 status, void *payload, u32 size, u32 *ddl_handle,
	void *const client_data)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_clnt_cb_in_flushing:\n");

	if (cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch\n");
		return;
	}

	switch (event) {
	case VCD_EVT_RESP_INPUT_DONE:
		rc = vcd_handle_input_done(cctxt, payload,
			VCD_EVT_RESP_INPUT_FLUSHED, status);
		break;
	case VCD_EVT_RESP_OUTPUT_DONE:
		rc = vcd_handle_frame_done(cctxt, payload,
			VCD_EVT_RESP_OUTPUT_FLUSHED, status);
		break;
	case VCD_EVT_RESP_OUTPUT_REQ:
		rc = vcd_handle_output_required_in_flushing(cctxt, payload);
		break;
	case VCD_EVT_IND_HWERRFATAL:
		vcd_handle_ind_hw_err_fatal(cctxt, VCD_EVT_IND_HWERRFATAL,
			status);
		break;
	default:
		VCD_MSG_ERROR("Unexpected callback event=%d status=%d from DDL\n",
			event, status);
		dev_ctxt->cont = false;
		break;
	}

	if (!VCD_FAILED(rc) && (event == VCD_EVT_RESP_INPUT_DONE ||
			event == VCD_EVT_RESP_OUTPUT_DONE ||
			event == VCD_EVT_RESP_OUTPUT_REQ) &&
			((struct ddl_frame_data_tag *)payload)->frm_trans_end) {

		vcd_mark_frame_channel(cctxt->dev_ctxt);

		if (!cctxt->status.frame_submitted) {
			VCD_MSG_HIGH("All pending frames recvd from DDL\n");

			if (cctxt->status.flush_mode & VCD_FLUSH_OUTPUT) {
				vcd_flush_output_buffers(cctxt);

				vcd_release_all_clnt_frm_transc(cctxt);
			}

			vcd_send_flush_done(cctxt, VCD_S_SUCCESS);
			vcd_release_interim_frame_channels(dev_ctxt);
			VCD_MSG_HIGH("Flush complete\n");
			vcd_release_all_clnt_def_frm_transc(cctxt);
			vcd_do_client_state_transition(cctxt,
				VCD_CLIENT_STATE_RUN,
				CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
		}
	}
}

static void vcd_clnt_cb_in_stopping(struct vcd_clnt_ctxt *cctxt, u32 event,
	u32 status, void *payload, u32 size, u32 *ddl_handle,
	void *const client_data)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_clnt_cb_in_stopping:\n");

	if (cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch\n");
		return;
	}

	switch (event) {
	case VCD_EVT_RESP_INPUT_DONE:
		rc = vcd_handle_input_done(cctxt, payload,
			VCD_EVT_RESP_INPUT_FLUSHED, status);

		break;
	case VCD_EVT_RESP_OUTPUT_DONE:
		rc = vcd_handle_frame_done(cctxt, payload,
			VCD_EVT_RESP_OUTPUT_FLUSHED, status);

		break;
	case VCD_EVT_RESP_OUTPUT_REQ:
		rc = vcd_handle_output_required_in_flushing(cctxt, payload);
		break;
	case VCD_EVT_RESP_STOP:
		vcd_handle_stop_done(cctxt, (struct vcd_transc *)client_data,
			status);
		break;
	case VCD_EVT_IND_HWERRFATAL:
		vcd_handle_ind_hw_err_fatal(cctxt, VCD_EVT_RESP_STOP, status);
		break;
	default:
		VCD_MSG_ERROR("Unexpected callback event=%d status=%d from DDL\n",
			event, status);

		dev_ctxt->cont = false;
		break;
	}

	if (!VCD_FAILED(rc) && (event == VCD_EVT_RESP_INPUT_DONE ||
			event == VCD_EVT_RESP_OUTPUT_DONE ||
			event == VCD_EVT_RESP_OUTPUT_REQ) &&
			((struct ddl_frame_data_tag *)payload)->frm_trans_end) {

		vcd_mark_frame_channel(cctxt->dev_ctxt);

		if (!cctxt->status.frame_submitted) {
			VCD_MSG_HIGH("All pending frames recvd from DDL\n");

			vcd_flush_output_buffers(cctxt);

			cctxt->status.flush_mode = 0;

			vcd_release_all_clnt_frm_transc(cctxt);

			VCD_MSG_HIGH("All buffers flushed. Enqueuing stop cmd\n");

			vcd_client_cmd_flush_and_en_q(cctxt,
				VCD_CMD_CODEC_STOP);
		}
	}
}

static void vcd_clnt_cb_in_pausing(struct vcd_clnt_ctxt *cctxt, u32 event,
	u32 status, void *payload, u32 size, u32 *ddl_handle,
	void *const client_data)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_clnt_cb_in_pausing:\n");

	if (cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch\n");

		return;
	}

	switch (event) {
	case VCD_EVT_RESP_INPUT_DONE:
		rc = vcd_handle_input_done(cctxt, payload, event, status);
		break;
	case VCD_EVT_RESP_OUTPUT_DONE:
		rc = vcd_handle_frame_done(cctxt, payload, event, status);
		break;
	case VCD_EVT_RESP_OUTPUT_REQ:
		rc = vcd_handle_output_required(cctxt, payload, status);
		break;
	case VCD_EVT_IND_HWERRFATAL:
		vcd_handle_ind_hw_err_fatal(cctxt, VCD_EVT_RESP_PAUSE, status);
		break;
	default:
		VCD_MSG_ERROR("Unexpected callback event=%d status=%d from DDL\n",
			event, status);

		dev_ctxt->cont = false;
		break;
	}

	if (!VCD_FAILED(rc) && (event == VCD_EVT_RESP_INPUT_DONE ||
			event == VCD_EVT_RESP_OUTPUT_DONE ||
			event == VCD_EVT_RESP_OUTPUT_REQ) &&
			((struct ddl_frame_data_tag *)payload)->frm_trans_end) {

		vcd_mark_frame_channel(cctxt->dev_ctxt);

		if (!cctxt->status.frame_submitted) {
			VCD_MSG_HIGH("All pending frames recvd from DDL\n");

			cctxt->callback(VCD_EVT_RESP_PAUSE, VCD_S_SUCCESS, NULL,
				0, cctxt, cctxt->client_data);

			vcd_do_client_state_transition(cctxt,
				VCD_CLIENT_STATE_PAUSED,
				CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));

			rc = vcd_power_event(cctxt->dev_ctxt, cctxt,
				VCD_EVT_PWR_CLNT_PAUSE);

			if (VCD_FAILED(rc)) {
				VCD_MSG_ERROR("VCD_EVT_PWR_CLNT_PAUSE_END "
					"failed\n");
			}
		}
	}
}

static void  vcd_clnt_cb_in_invalid(struct vcd_clnt_ctxt *cctxt, u32 event,
	u32 status, void *payload, u32 size, u32 *ddl_handle,
	void *const client_data)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	VCD_MSG_LOW("vcd_clnt_cb_in_invalid:\n");
	if (cctxt->ddl_handle != ddl_handle) {
		VCD_MSG_ERROR("ddl_handle mismatch\n");
		return;
	}
	switch (event) {
	case VCD_EVT_RESP_STOP:
		vcd_handle_stop_done_in_invalid(cctxt, status);
		break;
	case VCD_EVT_RESP_INPUT_DONE:
	case VCD_EVT_RESP_OUTPUT_DONE:
	case VCD_EVT_RESP_OUTPUT_REQ:
	case VCD_EVT_RESP_TRANSACTION_PENDING:
		break;
	case VCD_EVT_IND_HWERRFATAL:
		if (status == VCD_ERR_HW_FATAL)
			vcd_handle_stop_done_in_invalid(cctxt, status);

		break;
	default:
		VCD_MSG_ERROR("Unexpected callback event=%d status=%d from DDL\n",
			event, status);
		dev_ctxt->cont = false;
		break;
	}
}

static void vcd_clnt_enter_open(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Entering CLIENT_STATE_OPEN on api %d\n", ev_code);
}

static void vcd_clnt_enter_starting(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Entering CLIENT_STATE_STARTING on api %d\n",	ev_code);
}

static void vcd_clnt_enter_run(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Entering CLIENT_STATE_RUN on api %d\n", ev_code);
}

static void vcd_clnt_enter_flushing(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Entering CLIENT_STATE_FLUSHING on api %d\n",	ev_code);
}

static void vcd_clnt_enter_stopping(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Entering CLIENT_STATE_STOPPING on api %d\n", ev_code);
}

static void vcd_clnt_enter_eos(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	u32 rc;

	VCD_MSG_MED("Entering CLIENT_STATE_EOS on api %d\n", ev_code);
	rc = vcd_map_sched_status(sched_suspend_resume_client(
		cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl, false));
	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("Failed: sched_suspend_resume_client. rc=0x%x\n",
			rc);
}

static void vcd_clnt_enter_pausing(struct vcd_clnt_ctxt *cctxt,	s32 ev_code)
{
	VCD_MSG_MED("Entering CLIENT_STATE_PAUSING on api %d\n", ev_code);
}

static void vcd_clnt_enter_paused(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Entering CLIENT_STATE_PAUSED on api %d\n", ev_code);
}

static void  vcd_clnt_enter_invalid(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Entering CLIENT_STATE_INVALID on api %d\n", ev_code);
	cctxt->ddl_hdl_valid = false;
}

static void vcd_clnt_exit_open(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Exiting CLIENT_STATE_OPEN on api %d\n", ev_code);
}

static void vcd_clnt_exit_starting(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Exiting CLIENT_STATE_STARTING on api %d\n", ev_code);
}

static void vcd_clnt_exit_run(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Exiting CLIENT_STATE_RUN on api %d\n", ev_code);
}

static void vcd_clnt_exit_flushing(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Exiting CLIENT_STATE_FLUSHING on api %d\n", ev_code);
}

static void vcd_clnt_exit_stopping(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Exiting CLIENT_STATE_STOPPING on api %d\n", ev_code);
}

static void vcd_clnt_exit_eos(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	u32 rc;
	VCD_MSG_MED("Exiting CLIENT_STATE_EOS on api %d\n", ev_code);
	rc = vcd_map_sched_status(sched_suspend_resume_client(
		cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl, true));
	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("Failed: sched_suspend_resume_client. rc=0x%x\n",
			rc);
}

static void vcd_clnt_exit_pausing(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Exiting CLIENT_STATE_PAUSING on api %d\n", ev_code);
}

static void vcd_clnt_exit_paused(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Exiting CLIENT_STATE_PAUSED on api %d\n", ev_code);
}

static void  vcd_clnt_exit_invalid(struct vcd_clnt_ctxt *cctxt, s32 ev_code)
{
	VCD_MSG_MED("Exiting CLIENT_STATE_INVALID on api %d\n", ev_code);
}

void vcd_do_client_state_transition(struct vcd_clnt_ctxt *cctxt,
	enum vcd_clnt_state_enum to_state, u32 ev_code)
{
	struct vcd_clnt_state_ctxt *state_ctxt;

	if (!cctxt || to_state >= VCD_CLIENT_STATE_MAX) {
		VCD_MSG_ERROR("Bad parameters. cctxt=%p, to_state=%d\n", cctxt,
			to_state);
	}

	state_ctxt = &cctxt->clnt_state;

	if (state_ctxt->state == to_state) {
		VCD_MSG_HIGH("Client already in requested to_state=%d\n",
			to_state);
		return;
	}

	VCD_MSG_MED("vcd_do_client_state_transition: C%d -> C%d, for api %d\n",
		(int)state_ctxt->state, (int)to_state, ev_code);

	if (state_ctxt->state_table->pf_exit)
		state_ctxt->state_table->pf_exit(cctxt, ev_code);


	state_ctxt->state = to_state;
	state_ctxt->state_table = vcd_clnt_state_table[to_state];

	if (state_ctxt->state_table->pf_entry)
		state_ctxt->state_table->pf_entry(cctxt, ev_code);
}

const struct vcd_clnt_state_table *vcd_get_client_state_table(
	enum vcd_clnt_state_enum state)
{
	return vcd_clnt_state_table[state];
}

static const struct vcd_clnt_state_table vcd_clnt_table_open = {
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

static const struct vcd_clnt_state_table vcd_clnt_table_starting = {
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

static const struct vcd_clnt_state_table vcd_clnt_table_run = {
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

static const struct vcd_clnt_state_table vcd_clnt_table_flushing = {
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

static const struct vcd_clnt_state_table vcd_clnt_table_stopping = {
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

static const struct vcd_clnt_state_table vcd_clnt_table_eos = {
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

static const struct vcd_clnt_state_table vcd_clnt_table_pausing = {
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

static const struct vcd_clnt_state_table vcd_clnt_table_paused = {
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
static const struct vcd_clnt_state_table vcd_clnt_table_invalid = {
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

static const struct vcd_clnt_state_table *vcd_clnt_state_table[] = {
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
