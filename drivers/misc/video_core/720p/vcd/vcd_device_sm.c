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

static const struct vcd_dev_state_table_type_t *vcd_dev_state_table[];
static const struct vcd_dev_state_table_type_t vcd_dev_table_null;

struct vcd_drv_ctxt_type_t *vcd_get_drv_context(void)
{
	static struct vcd_drv_ctxt_type_t drv_context = {
		{&vcd_dev_table_null, VCD_DEVICE_STATE_NULL},
		{0},
		0
	};

	return &drv_context;

}

void vcd_do_device_state_transition(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 enum vcd_dev_state_enum_type e_to_state, u32 n_ev_code)
{
	struct vcd_dev_state_ctxt_type_t *p_state_ctxt;

	if (!p_drv_ctxt || e_to_state >= VCD_DEVICE_STATE_MAX) {
		VCD_MSG_ERROR("Bad parameters. p_drv_ctxt=%p, e_to_state=%d",
				  p_drv_ctxt, e_to_state);
	}

	p_state_ctxt = &p_drv_ctxt->dev_state;

	if (p_state_ctxt->e_state == e_to_state) {
		VCD_MSG_HIGH("Device already in requested e_to_state=%d",
				 e_to_state);

		return;
	}

	VCD_MSG_MED("vcd_do_device_state_transition: D%d -> D%d, for api %d",
			(int)p_state_ctxt->e_state, (int)e_to_state, n_ev_code);

	if (p_state_ctxt->p_state_table->pf_exit)
		p_state_ctxt->p_state_table->pf_exit(p_drv_ctxt, n_ev_code);


	p_state_ctxt->e_state = e_to_state;
	p_state_ctxt->p_state_table = vcd_dev_state_table[e_to_state];

	if (p_state_ctxt->p_state_table->pf_entry)
		p_state_ctxt->p_state_table->pf_entry(p_drv_ctxt, n_ev_code);
}

void vcd_hw_timeout_handler(void *p_user_data)
{
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;

	VCD_MSG_HIGH("vcd_hw_timeout_handler:");
	p_user_data = NULL;
	p_drv_ctxt = vcd_get_drv_context();
	vcd_critical_section_enter(p_drv_ctxt->dev_cs);
	if (p_drv_ctxt->dev_state.p_state_table->ev_hdlr.pf_timeout)
		p_drv_ctxt->dev_state.p_state_table->ev_hdlr.
			pf_timeout(p_drv_ctxt, p_user_data);
	else
		VCD_MSG_ERROR("hw_timeout unsupported in device state %d",
			p_drv_ctxt->dev_state.e_state);
	vcd_critical_section_leave(p_drv_ctxt->dev_cs);
}

void vcd_ddl_callback(u32 event, u32 status, void *p_payload,
	u32 n_size, u32 *ddl_handle, void *const p_client_data)
{
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	struct vcd_dev_ctxt_type *p_dev_ctxt;
	struct vcd_dev_state_ctxt_type_t *p_dev_state;
	struct vcd_clnt_ctxt_type_t *p_cctxt;
	struct vcd_transc_type *p_transc;

	VCD_MSG_LOW("vcd_ddl_callback:");

	VCD_MSG_LOW("event=0x%x status=0x%x", event, status);

	p_drv_ctxt = vcd_get_drv_context();
	p_dev_ctxt = &p_drv_ctxt->dev_ctxt;
	p_dev_state = &p_drv_ctxt->dev_state;

	p_dev_ctxt->b_continue = TRUE;
	vcd_device_timer_stop(p_dev_ctxt);

	switch (p_dev_state->e_state) {
	case VCD_DEVICE_STATE_NULL:
		{
			VCD_MSG_HIGH("Callback unexpected in NULL state");
			break;
		}

	case VCD_DEVICE_STATE_NOT_INIT:
		{
			VCD_MSG_HIGH("Callback unexpected in NOT_INIT state");
			break;
		}

	case VCD_DEVICE_STATE_INITING:
		{
			if (p_dev_state->p_state_table->ev_hdlr.pf_dev_cb) {
				p_dev_state->p_state_table->ev_hdlr.
					pf_dev_cb(p_drv_ctxt, event, status,
						  p_payload, n_size, ddl_handle,
						  p_client_data);
			} else {
				VCD_MSG_HIGH("No device handler in %d state",
						 p_dev_state->e_state);
			}
			break;
		}

	case VCD_DEVICE_STATE_READY:
		{
			p_transc = (struct vcd_transc_type *)p_client_data;

			if (!p_transc || !p_transc->b_in_use
				|| !p_transc->p_cctxt) {
				VCD_MSG_ERROR("Invalid clientdata "
							  "received from DDL ");
			} else {
				p_cctxt = p_transc->p_cctxt;

				if (p_cctxt->clnt_state.p_state_table->ev_hdlr.
					pf_clnt_cb) {
					p_cctxt->clnt_state.p_state_table->
						ev_hdlr.pf_clnt_cb(p_cctxt,
						event, status, p_payload,
						n_size,	ddl_handle,
						p_client_data);
				} else {
					VCD_MSG_HIGH
					("No client handler in"
					" (dsm:READY, csm:%d) state",
					(int)p_cctxt->clnt_state.e_state);

					if (VCD_FAILED(status)) {
						VCD_MSG_FATAL("DDL callback"
						" returned failure 0x%x",
						status);
					}
				}
			}
			break;
		}

	default:
		{
			VCD_MSG_ERROR("Unknown state");
			break;
		}

	}

}

u32 vcd_init_device_context(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
		u32 n_ev_code)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;
	struct sched_init_param_type sched_init;
	u32 rc;
	struct ddl_init_config_type ddl_init;

	VCD_MSG_LOW("vcd_init_device_context:");

	p_dev_ctxt->e_pending_cmd = VCD_CMD_NONE;

	rc = vcd_power_event(p_dev_ctxt, NULL, VCD_EVT_PWR_DEV_INIT_BEGIN);
	VCD_FAILED_RETURN(rc, "VCD_EVT_PWR_DEV_INIT_BEGIN failed");

	VCD_MSG_HIGH("Device powered ON and clocked");

	sched_init.n_perf_lvl = p_dev_ctxt->n_max_perf_lvl;
	rc = vcd_map_sched_status(sched_create
				  (&sched_init, &p_dev_ctxt->sched_hdl));

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: sched_create", rc);

		(void)vcd_power_event(p_dev_ctxt, NULL,
					  VCD_EVT_PWR_DEV_INIT_FAIL);

		return rc;
	}

	VCD_MSG_HIGH("Created scheduler instance.");

	ddl_init.p_core_virtual_base_addr = p_dev_ctxt->p_device_base_addr;
	ddl_init.pf_interrupt_clr = p_dev_ctxt->config.pf_interrupt_clr;
	ddl_init.ddl_callback = vcd_ddl_callback;

	rc = ddl_device_init(&ddl_init, NULL);

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: ddl_device_init", rc);

		(void)sched_destroy(p_dev_ctxt->sched_hdl);
		p_dev_ctxt->sched_hdl = NULL;

		(void)vcd_power_event(p_dev_ctxt, NULL,
					  VCD_EVT_PWR_DEV_INIT_FAIL);
	} else {
		vcd_device_timer_start(p_dev_ctxt);
		vcd_do_device_state_transition(p_drv_ctxt,
						   VCD_DEVICE_STATE_INITING,
						   n_ev_code);
	}

	return rc;
}

void vcd_handle_device_init_failed(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
		u32 status)
{
	struct vcd_clnt_ctxt_type_t *p_client;
	struct vcd_clnt_ctxt_type_t *p_tmp_client;

	VCD_MSG_ERROR("Device init failed. status = %d", status);

	p_client = p_drv_ctxt->dev_ctxt.p_cctxt_list_head;
	while (p_client) {
		p_client->callback(VCD_EVT_RESP_OPEN,
				   status, NULL, 0, 0, p_client->p_client_data);

		p_tmp_client = p_client;
		p_client = p_client->p_next;

		vcd_destroy_client_context(p_tmp_client);
	}
	if (ddl_device_release(NULL))
		VCD_MSG_ERROR("Failed: ddl_device_release");

	(void)sched_destroy(p_drv_ctxt->dev_ctxt.sched_hdl);
	p_drv_ctxt->dev_ctxt.sched_hdl = NULL;

	if (vcd_power_event(&p_drv_ctxt->dev_ctxt,
		NULL, VCD_EVT_PWR_DEV_INIT_FAIL))
		VCD_MSG_ERROR("VCD_EVT_PWR_DEV_INIT_FAIL failed");

	vcd_do_device_state_transition(p_drv_ctxt,
		VCD_DEVICE_STATE_NOT_INIT,
		DEVICE_STATE_EVENT_NUMBER(pf_dev_cb));
}

u32 vcd_deinit_device_context(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
		u32 n_ev_code)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_deinit_device_context:");

	rc = vcd_power_event(&p_drv_ctxt->dev_ctxt, NULL,
				 VCD_EVT_PWR_DEV_TERM_BEGIN);

	VCD_FAILED_RETURN(rc, "VCD_EVT_PWR_DEV_TERM_BEGIN failed");

	rc = ddl_device_release(NULL);

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: ddl_device_release", rc);

		(void)vcd_power_event(p_dev_ctxt, NULL,
					  VCD_EVT_PWR_DEV_TERM_FAIL);
	} else {
		(void)sched_destroy(p_dev_ctxt->sched_hdl);
		p_dev_ctxt->sched_hdl = NULL;

		(void) vcd_power_event(p_dev_ctxt, NULL,
			VCD_EVT_PWR_DEV_TERM_END);

		vcd_do_device_state_transition(p_drv_ctxt,
			VCD_DEVICE_STATE_NOT_INIT, n_ev_code);
	}
	return rc;
}

void vcd_term_driver_context(struct vcd_drv_ctxt_type_t *p_drv_ctxt)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;

	VCD_MSG_HIGH("All driver instances terminated");

	if (p_dev_ctxt->config.pf_deregister_isr)
		p_dev_ctxt->config.pf_deregister_isr();

	if (p_dev_ctxt->config.pf_un_map_dev_base_addr)
		p_dev_ctxt->config.pf_un_map_dev_base_addr();

	if (p_dev_ctxt->config.pf_timer_release)
		p_dev_ctxt->config.pf_timer_release(
			p_dev_ctxt->p_hw_timer_handle);

	vcd_free(p_dev_ctxt->a_trans_tbl);

	memset(p_dev_ctxt, 0, sizeof(struct vcd_dev_ctxt_type));

	vcd_do_device_state_transition(p_drv_ctxt,
					   VCD_DEVICE_STATE_NULL,
					   DEVICE_STATE_EVENT_NUMBER(pf_term));

}

u32 vcd_reset_device_context(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	u32 ev_code)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_reset_device_context:");
	vcd_reset_device_channels(p_dev_ctxt);
	rc = vcd_power_event(&p_drv_ctxt->dev_ctxt, NULL,
						 VCD_EVT_PWR_DEV_TERM_BEGIN);
	VCD_FAILED_RETURN(rc, "VCD_EVT_PWR_DEV_TERM_BEGIN failed");
	if (ddl_reset_hw(0))
		VCD_MSG_HIGH("HW Reset done");
	else
		VCD_MSG_FATAL("HW Reset failed");

	(void)vcd_power_event(p_dev_ctxt, NULL, VCD_EVT_PWR_DEV_TERM_END);

	return VCD_S_SUCCESS;
}

void vcd_handle_device_err_fatal(struct vcd_dev_ctxt_type *p_dev_ctxt,
	struct vcd_clnt_ctxt_type_t *p_trig_clnt)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt = p_dev_ctxt->p_cctxt_list_head;
	VCD_MSG_LOW("vcd_handle_device_err_fatal:");
	while (p_cctxt) {
		if (p_cctxt != p_trig_clnt) {
			vcd_clnt_handle_device_err_fatal(p_cctxt,
				VCD_EVT_IND_HWERRFATAL);
		}
		p_cctxt = p_cctxt->p_next;
	}
	p_dev_ctxt->e_pending_cmd = VCD_CMD_DEVICE_RESET;
	vcd_do_device_state_transition(vcd_get_drv_context(),
		VCD_DEVICE_STATE_INVALID,
		DEVICE_STATE_EVENT_NUMBER(pf_dev_cb));
}

void vcd_handle_for_last_clnt_close(
	struct vcd_dev_ctxt_type *p_dev_ctxt, u32 b_send_deinit)
{
	if (!p_dev_ctxt->p_cctxt_list_head) {
		VCD_MSG_HIGH("All clients are closed");
		if (b_send_deinit)
			(void) vcd_deinit_device_context(
				vcd_get_drv_context(),
				DEVICE_STATE_EVENT_NUMBER(pf_close));
		else
			p_dev_ctxt->e_pending_cmd =
			VCD_CMD_DEVICE_TERM;
	}
}
void vcd_continue(void)
{
	struct vcd_drv_ctxt_type_t *p_drv_ctxt;
	struct vcd_dev_ctxt_type *p_dev_ctxt;
	u32 b_continue;
	struct vcd_transc_type *p_transc;
	u32 rc;
	VCD_MSG_LOW("vcd_continue:");

	p_drv_ctxt = vcd_get_drv_context();
	p_dev_ctxt = &p_drv_ctxt->dev_ctxt;

	p_dev_ctxt->b_continue = FALSE;

	if (p_dev_ctxt->e_pending_cmd == VCD_CMD_DEVICE_INIT) {
		VCD_MSG_HIGH("VCD_CMD_DEVICE_INIT is pending");

		p_dev_ctxt->e_pending_cmd = VCD_CMD_NONE;

		(void)vcd_init_device_context(p_drv_ctxt,
			DEVICE_STATE_EVENT_NUMBER(pf_open));
	} else if (p_dev_ctxt->e_pending_cmd == VCD_CMD_DEVICE_TERM) {
		VCD_MSG_HIGH("VCD_CMD_DEVICE_TERM is pending");

		p_dev_ctxt->e_pending_cmd = VCD_CMD_NONE;

		(void)vcd_deinit_device_context(p_drv_ctxt,
			DEVICE_STATE_EVENT_NUMBER(pf_close));
	} else if (p_dev_ctxt->e_pending_cmd == VCD_CMD_DEVICE_RESET) {
		VCD_MSG_HIGH("VCD_CMD_DEVICE_RESET is pending");
		p_dev_ctxt->e_pending_cmd = VCD_CMD_NONE;
		(void)vcd_reset_device_context(p_drv_ctxt,
			DEVICE_STATE_EVENT_NUMBER(pf_dev_cb));
	} else {
		if (p_dev_ctxt->b_set_perf_lvl_pending) {
			rc = vcd_power_event(p_dev_ctxt, NULL,
						 VCD_EVT_PWR_DEV_SET_PERFLVL);

			if (VCD_FAILED(rc)) {
				VCD_MSG_ERROR
					("VCD_EVT_PWR_CLNT_SET_PERFLVL failed");
				VCD_MSG_HIGH
					("Not running at desired perf level."
					 "curr=%d, reqd=%d",
					 p_dev_ctxt->n_curr_perf_lvl,
					 p_dev_ctxt->n_reqd_perf_lvl);
			} else {
				p_dev_ctxt->b_set_perf_lvl_pending = FALSE;
			}
		}

		do {
			b_continue = FALSE;

			if (vcd_get_command_channel_in_loop
				(p_dev_ctxt, &p_transc)) {
				if (vcd_submit_command_in_continue(p_dev_ctxt,
					p_transc))
					b_continue = TRUE;
				else {
					VCD_MSG_MED
						("No more commands to submit");

					vcd_release_command_channel(p_dev_ctxt,
						p_transc);

					vcd_release_interim_command_channels
						(p_dev_ctxt);
				}
			}
		} while (b_continue);

		do {
			b_continue = FALSE;

			if (vcd_get_frame_channel_in_loop
				(p_dev_ctxt, &p_transc)) {
				if (vcd_try_submit_frame_in_continue(p_dev_ctxt,
					p_transc)) {
					b_continue = TRUE;
				} else {
					VCD_MSG_MED("No more frames to submit");

					vcd_release_frame_channel(p_dev_ctxt,
								  p_transc);

					vcd_release_interim_frame_channels
						(p_dev_ctxt);
				}
			}

		} while (b_continue);

		if (!vcd_core_is_busy(p_dev_ctxt)) {
			rc = vcd_power_event(p_dev_ctxt, NULL,
				VCD_EVT_PWR_CLNT_CMD_END);

			if (VCD_FAILED(rc))
				VCD_MSG_ERROR("Failed:"
					"VCD_EVT_PWR_CLNT_CMD_END");
		}
	}
}

static void vcd_pause_all_sessions(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt = p_dev_ctxt->p_cctxt_list_head;
	u32 rc;

	while (p_cctxt) {
		if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_pause) {
			rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
				pf_pause(p_cctxt);

			if (VCD_FAILED(rc))
				VCD_MSG_ERROR("Client pause failed");

		}

		p_cctxt = p_cctxt->p_next;
	}
}

static void vcd_resume_all_sessions(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt = p_dev_ctxt->p_cctxt_list_head;
	u32 rc;

	while (p_cctxt) {
		if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_resume) {
			rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
				pf_resume(p_cctxt);

			if (VCD_FAILED(rc))
				VCD_MSG_ERROR("Client resume failed");

		}

		p_cctxt = p_cctxt->p_next;
	}
}

static u32 vcd_init_cmn
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 struct vcd_init_config_type *p_config, s32 *p_driver_handle)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;
	s32 driver_id;

	if (p_dev_ctxt->config.pf_interrupt_clr !=
		p_config->pf_interrupt_clr
		|| p_dev_ctxt->config.pf_register_isr !=
		p_config->pf_register_isr
		|| p_dev_ctxt->config.pf_deregister_isr !=
		p_config->pf_deregister_isr
		|| p_dev_ctxt->config.pf_map_dev_base_addr !=
		p_config->pf_map_dev_base_addr
		|| p_dev_ctxt->config.pf_un_map_dev_base_addr !=
		p_config->pf_un_map_dev_base_addr) {
		VCD_MSG_ERROR("Device config mismatch");
		VCD_MSG_HIGH("VCD will be using config from 1st vcd_init");
	}

	*p_driver_handle = 0;

	driver_id = 0;
	while (driver_id < VCD_DRIVER_INSTANCE_MAX &&
		   p_dev_ctxt->b_driver_ids[driver_id]) {
		++driver_id;
	}

	if (driver_id == VCD_DRIVER_INSTANCE_MAX) {
		VCD_MSG_ERROR("Max driver instances reached");

		return VCD_ERR_FAIL;
	}

	++p_dev_ctxt->n_refs;
	p_dev_ctxt->b_driver_ids[driver_id] = TRUE;
	*p_driver_handle = driver_id + 1;

	VCD_MSG_HIGH("Driver_id = %d. No of driver instances = %d",
			 driver_id, p_dev_ctxt->n_refs);

	return VCD_S_SUCCESS;

}

static u32 vcd_init_in_null
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 struct vcd_init_config_type *p_config, s32 *p_driver_handle) {
	u32 rc = VCD_S_SUCCESS;
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;
	u32 b_done_create_timer = FALSE;
	VCD_MSG_LOW("vcd_init_in_dev_null:");


	p_dev_ctxt->config = *p_config;

	p_dev_ctxt->p_device_base_addr =
		(u8 *)p_config->pf_map_dev_base_addr(
			p_dev_ctxt->config.p_device_name);

	if (!p_dev_ctxt->p_device_base_addr) {
		VCD_MSG_ERROR("NULL Device_base_addr");

		return VCD_ERR_FAIL;
	}

	if (p_config->pf_register_isr) {
		p_config->pf_register_isr(p_dev_ctxt->config.
			p_device_name);
	}

	if (p_config->pf_timer_create) {
		if (p_config->pf_timer_create(vcd_hw_timeout_handler,
			NULL, &p_dev_ctxt->p_hw_timer_handle))
			b_done_create_timer = TRUE;
		else {
			VCD_MSG_ERROR("timercreate failed");
			return VCD_ERR_FAIL;
		}
	}


	rc = vcd_init_cmn(p_drv_ctxt, p_config, p_driver_handle);

	if (!VCD_FAILED(rc)) {
		vcd_do_device_state_transition(p_drv_ctxt,
						   VCD_DEVICE_STATE_NOT_INIT,
						   DEVICE_STATE_EVENT_NUMBER
						   (pf_init));
	} else {
		if (p_dev_ctxt->config.pf_un_map_dev_base_addr)
			p_dev_ctxt->config.pf_un_map_dev_base_addr();

		if (p_dev_ctxt->config.pf_deregister_isr)
			p_dev_ctxt->config.pf_deregister_isr();

		if (b_done_create_timer && p_dev_ctxt->config.pf_timer_release)
			p_dev_ctxt->config.pf_timer_release(p_dev_ctxt->
				p_hw_timer_handle);

	}

	return rc;

}

static u32 vcd_init_in_not_init
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 struct vcd_init_config_type *p_config, s32 *p_driver_handle)
{

	VCD_MSG_LOW("vcd_init_in_dev_not_init:");

	return vcd_init_cmn(p_drv_ctxt, p_config, p_driver_handle);

}

static u32 vcd_init_in_initing
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 struct vcd_init_config_type *p_config, s32 *p_driver_handle) {

	VCD_MSG_LOW("vcd_init_in_dev_initing:");

	return vcd_init_cmn(p_drv_ctxt, p_config, p_driver_handle);

}

static u32 vcd_init_in_ready
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 struct vcd_init_config_type *p_config, s32 *p_driver_handle)
{
	VCD_MSG_LOW("vcd_init_in_dev_ready:");

	return vcd_init_cmn(p_drv_ctxt, p_config, p_driver_handle);
}

static u32 vcd_term_cmn
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 driver_handle)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;

	if (!vcd_validate_driver_handle(p_dev_ctxt, driver_handle)) {
		VCD_MSG_ERROR("Invalid driver handle = %d", driver_handle);

		return VCD_ERR_BAD_HANDLE;
	}

	if (vcd_check_for_client_context(p_dev_ctxt,
				driver_handle - 1)) {
		VCD_MSG_ERROR("Driver has active client");

		return VCD_ERR_BAD_STATE;
	}

	--p_dev_ctxt->n_refs;
	p_dev_ctxt->b_driver_ids[driver_handle - 1] = FALSE;

	VCD_MSG_HIGH("Driver_id %d terminated. No of driver instances = %d",
			 driver_handle - 1, p_dev_ctxt->n_refs);

	return VCD_S_SUCCESS;
}

static u32 vcd_term_in_not_init
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 driver_handle)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;
	u32 rc;

	VCD_MSG_LOW("vcd_term_in_dev_not_init:");

	rc = vcd_term_cmn(p_drv_ctxt, driver_handle);

	if (!VCD_FAILED(rc) && !p_dev_ctxt->n_refs)
		vcd_term_driver_context(p_drv_ctxt);

	return rc;
}

static u32 vcd_term_in_initing
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 driver_handle)
{
	VCD_MSG_LOW("vcd_term_in_dev_initing:");

	return vcd_term_cmn(p_drv_ctxt, driver_handle);
}

static u32 vcd_term_in_ready
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 driver_handle)
{
	VCD_MSG_LOW("vcd_term_in_dev_ready:");

	return vcd_term_cmn(p_drv_ctxt, driver_handle);
}

static u32  vcd_term_in_invalid(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
							 s32  driver_handle)
{
	u32 rc;
	VCD_MSG_LOW("vcd_term_in_invalid:");
	rc = vcd_term_cmn(p_drv_ctxt, driver_handle);
	if (!VCD_FAILED(rc) && !p_drv_ctxt->dev_ctxt.n_refs)
		vcd_term_driver_context(p_drv_ctxt);

	return rc;
}

static u32 vcd_open_cmn
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 s32 driver_handle,
	 u32 b_decoding,
	 void (*callback) (u32 event, u32 status, void *p_info, u32 n_size,
			   void *handle, void *const p_client_data),
	 void *p_client_data, struct vcd_clnt_ctxt_type_t ** pp_cctxt)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;
	struct vcd_clnt_ctxt_type_t *p_cctxt;
	struct vcd_clnt_ctxt_type_t *p_client;

	if (!vcd_validate_driver_handle(p_dev_ctxt, driver_handle)) {
		VCD_MSG_ERROR("Invalid driver handle = %d", driver_handle);

		return VCD_ERR_BAD_HANDLE;
	}

	p_cctxt =
		(struct vcd_clnt_ctxt_type_t *)
		vcd_malloc(sizeof(struct vcd_clnt_ctxt_type_t));
	if (!p_cctxt) {
		VCD_MSG_ERROR("No memory for client ctxt");

		return VCD_ERR_ALLOC_FAIL;
	}

	memset(p_cctxt, 0, sizeof(struct vcd_clnt_ctxt_type_t));
	p_cctxt->p_dev_ctxt = p_dev_ctxt;
	p_cctxt->driver_id = driver_handle - 1;
	p_cctxt->b_decoding = b_decoding;
	p_cctxt->callback = callback;
	p_cctxt->p_client_data = p_client_data;

	p_client = p_dev_ctxt->p_cctxt_list_head;
	p_dev_ctxt->p_cctxt_list_head = p_cctxt;
	p_cctxt->p_next = p_client;

	*pp_cctxt = p_cctxt;

	return VCD_S_SUCCESS;

}

static u32 vcd_open_in_not_init
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 s32 driver_handle,
	 u32 b_decoding,
	 void (*callback) (u32 event, u32 status, void *p_info, u32 n_size,
			   void *handle, void *const p_client_data),
	 void *p_client_data)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt;
	u32 rc;

	VCD_MSG_LOW("vcd_open_in_dev_not_init:");

	rc = vcd_open_cmn(p_drv_ctxt, driver_handle, b_decoding, callback,
			  p_client_data, &p_cctxt);

	VCD_FAILED_RETURN(rc, "Failed: vcd_open_cmn");

	rc = vcd_init_device_context(p_drv_ctxt,
					 DEVICE_STATE_EVENT_NUMBER(pf_open));

	if (VCD_FAILED(rc))
		vcd_destroy_client_context(p_cctxt);

	return rc;
}

static u32 vcd_open_in_initing(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 s32 driver_handle, u32 b_decoding,
	 void (*callback) (u32 event, u32 status, void *p_info, u32 n_size,
			   void *handle, void *const p_client_data),
	 void *p_client_data)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt;

	VCD_MSG_LOW("vcd_open_in_dev_initing:");

	return vcd_open_cmn(p_drv_ctxt, driver_handle, b_decoding, callback,
				 p_client_data, &p_cctxt);
}

static u32 vcd_open_in_ready
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 s32 driver_handle,
	 u32 b_decoding,
	 void (*callback) (u32 event, u32 status, void *p_info, u32 n_size,
			   void *handle, void *const p_client_data),
	 void *p_client_data)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt;
	struct vcd_handle_container_type container;
	u32 rc;

	VCD_MSG_LOW("vcd_open_in_dev_ready:");

	rc = vcd_open_cmn(p_drv_ctxt, driver_handle, b_decoding, callback,
			  p_client_data, &p_cctxt);

	VCD_FAILED_RETURN(rc, "Failed: vcd_open_cmn");

	rc = vcd_init_client_context(p_cctxt);

	if (!VCD_FAILED(rc)) {
		container.handle = (void *)p_cctxt;

		callback(VCD_EVT_RESP_OPEN,
			 VCD_S_SUCCESS,
			 &container,
			 sizeof(container), container.handle, p_client_data);
	} else {
		VCD_MSG_ERROR("rc = 0x%x. Failed: vcd_init_client_context", rc);

		vcd_destroy_client_context(p_cctxt);
	}

	return rc;
}

static u32 vcd_close_in_ready
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 struct vcd_clnt_ctxt_type_t *p_cctxt) {
	u32 rc;

	VCD_MSG_LOW("vcd_close_in_dev_ready:");

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_close) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
			pf_close(p_cctxt);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
				  p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	if (!VCD_FAILED(rc))
		vcd_handle_for_last_clnt_close(&p_drv_ctxt->dev_ctxt, TRUE);

	return rc;
}

static u32  vcd_close_in_dev_invalid(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc;
	VCD_MSG_LOW("vcd_close_in_dev_invalid:");
	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_close) {
		rc = p_cctxt->clnt_state.p_state_table->
			ev_hdlr.pf_close(p_cctxt);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
					  p_cctxt->clnt_state.e_state);
		rc = VCD_ERR_BAD_STATE;
	}
	if (!VCD_FAILED(rc) && !p_drv_ctxt->dev_ctxt.
		p_cctxt_list_head) {
		VCD_MSG_HIGH("All INVALID clients are closed");
		vcd_do_device_state_transition(p_drv_ctxt,
			VCD_DEVICE_STATE_NOT_INIT,
			DEVICE_STATE_EVENT_NUMBER(pf_close));
	}
	return rc;
}

static u32 vcd_resume_in_ready
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 struct vcd_clnt_ctxt_type_t *p_cctxt) {
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_resume_in_ready:");

	if (p_cctxt->clnt_state.p_state_table->ev_hdlr.pf_resume) {
		rc = p_cctxt->clnt_state.p_state_table->ev_hdlr.
			pf_resume(p_cctxt);
	} else {
		VCD_MSG_ERROR("Unsupported API in client state %d",
				  p_cctxt->clnt_state.e_state);

		rc = VCD_ERR_BAD_STATE;
	}

	return rc;
}

static u32 vcd_set_dev_pwr_in_ready
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 enum vcd_power_state_type e_pwr_state)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;

	VCD_MSG_LOW("vcd_set_dev_pwr_in_ready:");

	switch (e_pwr_state) {
	case VCD_PWR_STATE_SLEEP:
		{
			vcd_pause_all_sessions(p_dev_ctxt);

			p_dev_ctxt->e_pwr_state = VCD_PWR_STATE_SLEEP;

			break;
		}

	case VCD_PWR_STATE_ON:
		{
			if (p_dev_ctxt->e_pwr_state == VCD_PWR_STATE_SLEEP)
				vcd_resume_all_sessions(p_dev_ctxt);


			p_dev_ctxt->e_pwr_state = VCD_PWR_STATE_ON;

			break;
		}

	default:
		{
			VCD_MSG_ERROR("Invalid power state requested %d",
					  e_pwr_state);
			break;
		}

	}

	return rc;
}

static void vcd_dev_cb_in_initing
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
	 u32 event,
	 u32 status,
	 void *p_payload, u32 size, u32 *ddl_handle, void *const p_client_data)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt;
	struct vcd_clnt_ctxt_type_t *p_client;
	struct vcd_clnt_ctxt_type_t *p_tmp_client;
	struct vcd_handle_container_type container;
	u32 rc = VCD_S_SUCCESS;
	u32 b_client_inited = FALSE;
	u32 b_fail_all_open = FALSE;

	VCD_MSG_LOW("vcd_dev_cb_in_initing:");

	if (event != VCD_EVT_RESP_DEVICE_INIT) {
		VCD_MSG_ERROR("vcd_dev_cb_in_initing: Unexpected event %d",
				  (int)event);
		return;
	}

	p_dev_ctxt = &p_drv_ctxt->dev_ctxt;

	p_dev_ctxt->b_continue = FALSE;

	if (VCD_FAILED(status)) {
		vcd_handle_device_init_failed(p_drv_ctxt, status);

		return;
	}

	vcd_do_device_state_transition(p_drv_ctxt,
					   VCD_DEVICE_STATE_READY,
					   DEVICE_STATE_EVENT_NUMBER(pf_open));

	if (!p_dev_ctxt->p_cctxt_list_head) {
		VCD_MSG_HIGH("All clients are closed");

		p_dev_ctxt->e_pending_cmd = VCD_CMD_DEVICE_TERM;

		return;
	}

	if (!p_dev_ctxt->n_ddl_cmd_ch_depth
		|| !p_dev_ctxt->a_trans_tbl)
		rc = vcd_setup_with_ddl_capabilities(p_dev_ctxt);


	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR
			("rc = 0x%x: Failed vcd_setup_with_ddl_capabilities",
			 rc);

		b_fail_all_open = TRUE;
	}

	p_client = p_dev_ctxt->p_cctxt_list_head;
	while (p_client) {
		if (!b_fail_all_open)
			rc = vcd_init_client_context(p_client);


		if (!VCD_FAILED(rc)) {
			container.handle = (void *)p_client;
			p_client->callback(VCD_EVT_RESP_OPEN,
					   VCD_S_SUCCESS,
					   &container,
					   sizeof(container),
					   container.handle,
					   p_client->p_client_data);

			p_client = p_client->p_next;

			b_client_inited = TRUE;
		} else {
			VCD_MSG_ERROR
				("rc = 0x%x, Failed: vcd_init_client_context",
				 rc);

			p_client->callback(VCD_EVT_RESP_OPEN,
					   rc,
					   NULL, 0, 0, p_client->p_client_data);

			p_tmp_client = p_client;
			p_client = p_client->p_next;

			vcd_destroy_client_context(p_tmp_client);
		}
	}

	if (!b_client_inited || b_fail_all_open) {
		VCD_MSG_ERROR("All client open requests failed");

		p_dev_ctxt->e_pending_cmd = VCD_CMD_DEVICE_TERM;
	} else {
		if (vcd_power_event(p_dev_ctxt, NULL,
					 VCD_EVT_PWR_DEV_INIT_END)) {
			VCD_MSG_ERROR("VCD_EVT_PWR_DEV_INIT_END failed");
		}
	}
}

static void  vcd_hw_timeout_cmn(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
							  void *p_user_data)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = &p_drv_ctxt->dev_ctxt;
	VCD_MSG_LOW("vcd_hw_timeout_cmn:");
	vcd_device_timer_stop(p_dev_ctxt);

	vcd_handle_device_err_fatal(p_dev_ctxt, NULL);

	/* Reset HW. */
	(void) vcd_reset_device_context(p_drv_ctxt,
		DEVICE_STATE_EVENT_NUMBER(pf_timeout));
}

static void vcd_dev_enter_null
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Entering DEVICE_STATE_NULL on api %d", n_state_event_type);

}

static void vcd_dev_enter_not_init
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Entering DEVICE_STATE_NOT_INIT on api %d",
			n_state_event_type);

}

static void vcd_dev_enter_initing
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Entering DEVICE_STATE_INITING on api %d",
			n_state_event_type);

}

static void vcd_dev_enter_ready
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Entering DEVICE_STATE_READY on api %d",
			n_state_event_type);
}

static void vcd_dev_enter_invalid(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
							   s32 state_event_type)
{
   VCD_MSG_MED("Entering DEVICE_STATE_INVALID on api %d", state_event_type);
}

static void vcd_dev_exit_null
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Exiting DEVICE_STATE_NULL on api %d", n_state_event_type);
}

static void vcd_dev_exit_not_init
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Exiting DEVICE_STATE_NOT_INIT on api %d",
			n_state_event_type);

}

static void vcd_dev_exit_initing
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Exiting DEVICE_STATE_INITING on api %d",
			n_state_event_type);
}

static void vcd_dev_exit_ready
	(struct vcd_drv_ctxt_type_t *p_drv_ctxt, s32 n_state_event_type) {
	VCD_MSG_MED("Exiting DEVICE_STATE_READY on api %d", n_state_event_type);
}

static void vcd_dev_exit_invalid(struct vcd_drv_ctxt_type_t *p_drv_ctxt,
							  s32 state_event_type)
{
   VCD_MSG_MED("Exiting DEVICE_STATE_INVALID on api %d", state_event_type);
}

static const struct vcd_dev_state_table_type_t vcd_dev_table_null = {
	{
	 vcd_init_in_null,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 },
	vcd_dev_enter_null,
	vcd_dev_exit_null
};

static const struct vcd_dev_state_table_type_t vcd_dev_table_not_init = {
	{
	 vcd_init_in_not_init,
	 vcd_term_in_not_init,
	 vcd_open_in_not_init,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 NULL,
	 },
	vcd_dev_enter_not_init,
	vcd_dev_exit_not_init
};

static const struct vcd_dev_state_table_type_t vcd_dev_table_initing = {
	{
	 vcd_init_in_initing,
	 vcd_term_in_initing,
	 vcd_open_in_initing,
	 NULL,
	 NULL,
	 NULL,
	 vcd_dev_cb_in_initing,
	 vcd_hw_timeout_cmn,
	 },
	vcd_dev_enter_initing,
	vcd_dev_exit_initing
};

static const struct vcd_dev_state_table_type_t vcd_dev_table_ready = {
	{
	 vcd_init_in_ready,
	 vcd_term_in_ready,
	 vcd_open_in_ready,
	 vcd_close_in_ready,
	 vcd_resume_in_ready,
	 vcd_set_dev_pwr_in_ready,
	 NULL,
	 vcd_hw_timeout_cmn,
	 },
	vcd_dev_enter_ready,
	vcd_dev_exit_ready
};

static const struct vcd_dev_state_table_type_t vcd_dev_table_in_invalid = {
	{
		NULL,
		vcd_term_in_invalid,
		NULL,
		vcd_close_in_dev_invalid,
		NULL,
		NULL,
		NULL,
		NULL,
	},
	vcd_dev_enter_invalid,
	vcd_dev_exit_invalid
};

static const struct vcd_dev_state_table_type_t *vcd_dev_state_table[] = {
	&vcd_dev_table_null,
	&vcd_dev_table_not_init,
	&vcd_dev_table_initing,
	&vcd_dev_table_ready,
	&vcd_dev_table_in_invalid
};
