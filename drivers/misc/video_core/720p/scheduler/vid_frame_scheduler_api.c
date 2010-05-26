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

#include "vid_frame_scheduler_api.h"
#include "vid_frame_scheduler.h"

enum sched_status_type sched_create(
	struct sched_init_param_type *p_init_param, void **p_handle)
{
	struct sched_ctx_type *p_sched_ctx;

	SCHED_MSG_HIGH("sched_create API");

	if (!p_handle || !p_init_param) {
		SCHED_MSG_ERR
		("Bad input parameters: p_handle=%p, p_init_param=%p",
		p_handle, p_init_param);
		return SCHED_S_EBADPARM;
	}

	if (!p_init_param->n_perf_lvl) {
		SCHED_MSG_ERR("Invalid Perf level=%u",
			p_init_param->n_perf_lvl);
		return SCHED_S_EBADPARM;
	}

	p_sched_ctx =
		(struct sched_ctx_type *)
		SCHED_MALLOC(sizeof(struct sched_ctx_type));

	if (!p_sched_ctx) {
		SCHED_MSG_ERR("Could not allocate sched ctx. Out of memory");
		return SCHED_S_ENOMEM;
	}

	SCHED_MEMSET(p_sched_ctx, 0, sizeof(struct sched_ctx_type));
	p_sched_ctx->n_perf_lvl = p_init_param->n_perf_lvl;

	if (SCHED_FAILED(SCHED_CRITSEC_CREATE(&p_sched_ctx->sched_cs))) {
		SCHED_FREE(p_sched_ctx);
		return SCHED_S_EFAIL;
	}

	*p_handle = p_sched_ctx;

	SCHED_MSG_MED("Sched instance created. All went well");

	return SCHED_S_OK;

}

enum sched_status_type sched_destroy(void *handle)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;

	SCHED_MSG_HIGH("sched_destroy API");

	if (!p_sched_ctx) {
		SCHED_MSG_ERR("Bad input parameters");
		return SCHED_S_EBADPARM;
	}
	(void)SCHED_CRITSEC_ENTER(p_sched_ctx->sched_cs);
	(void)sched_clear_clnt_list(p_sched_ctx->p_rt_head);
	(void)sched_clear_clnt_list(p_sched_ctx->p_n_rt_head);
	SCHED_MSG_MED("Sched clnt lists are cleared & released");
	(void)SCHED_CRITSEC_LEAVE(p_sched_ctx->sched_cs);
	(void)SCHED_CRITSEC_RELEASE(p_sched_ctx->sched_cs);
	SCHED_MEMSET(p_sched_ctx, 0, sizeof(struct sched_ctx_type));
	SCHED_FREE(p_sched_ctx);
	SCHED_MSG_MED("Sched instance deleted");
	return SCHED_S_OK;
}

enum sched_status_type sched_get_param(
	void *handle,
	enum sched_index_type param_index,
	union sched_value_type *p_param_value)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	enum sched_status_type status;

	SCHED_MSG_HIGH("sched_get_param API");

	if (!p_sched_ctx || !p_param_value) {
		SCHED_MSG_ERR
		("Bad input parameters: p_sched_ctx=%p, p_param_value=%p",
		p_sched_ctx, p_param_value);

		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(p_sched_ctx->sched_cs);

	status =
	sched_process_sched_lvl_get_param(p_sched_ctx, param_index,
					p_param_value);

	(void)SCHED_CRITSEC_LEAVE(p_sched_ctx->sched_cs);
	return status;
}

enum sched_status_type sched_set_param(
	void *handle,
	enum sched_index_type param_index,
	union sched_value_type *p_param_value)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	enum sched_status_type status;

	SCHED_MSG_HIGH("sched_set_param API");

	if (!p_sched_ctx || !p_param_value) {
		SCHED_MSG_ERR
		("Bad input parameters: p_sched_ctx=%p, p_param_value=%p",
		p_sched_ctx, p_param_value);
		return SCHED_S_EBADPARM;
	}
	(void)SCHED_CRITSEC_ENTER(p_sched_ctx->sched_cs);
	status =
		sched_process_sched_lvl_set_param(p_sched_ctx, param_index,
		p_param_value);
	(void)SCHED_CRITSEC_LEAVE(p_sched_ctx->sched_cs);
	return status;
}

enum sched_status_type sched_add_client(
	void *handle,
	struct sched_client_init_param_type *p_init_param,
	void **p_client_hdl)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	enum sched_status_type status = SCHED_S_OK;
	struct _sched_clnt_list_node_type *p_new_clnt;

	SCHED_MSG_HIGH("sched_add_client API");

	if (!p_sched_ctx || !p_init_param ||
			!p_client_hdl) {
		SCHED_MSG_ERR("Bad input parameters");

		return SCHED_S_EBADPARM;
	}

	p_new_clnt = (struct _sched_clnt_list_node_type *)
	SCHED_MALLOC(sizeof(struct _sched_clnt_list_node_type));
	if (!p_new_clnt) {
		SCHED_MSG_ERR("Could not allocate client ctx. Out of memory");
		return SCHED_S_ENOMEM;
	}
	(void)SCHED_CRITSEC_ENTER(p_sched_ctx->sched_cs);
	status = sched_process_add_clnt(p_sched_ctx, p_new_clnt, p_init_param);

	if (SCHED_FAILED(status)) {
		SCHED_MSG_ERR("Add_client failed with err=%d", status);
		sched_free_clnt_node(p_new_clnt);
		p_new_clnt = NULL;
	}

	(void)SCHED_CRITSEC_LEAVE(p_sched_ctx->sched_cs);
	*p_client_hdl = p_new_clnt;
	SCHED_MSG_MED("Sched client instance created. All went well");
	return status;
}

enum sched_status_type sched_remove_client(void *handle, void *client_hdl)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	struct _sched_clnt_list_node_type *p_clnt_node =
	(struct _sched_clnt_list_node_type *)client_hdl;
	enum sched_status_type status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_remove_client API");
	if (!p_sched_ctx || !p_clnt_node) {
		SCHED_MSG_ERR
		("Bad input parameters: p_sched_ctx=%p, p_clnt_node=%p",
		p_sched_ctx, p_clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(p_sched_ctx->sched_cs);
	status = sched_process_remove_clnt(p_sched_ctx, p_clnt_node);
	(void)SCHED_CRITSEC_LEAVE(p_sched_ctx->sched_cs);
	return status;
}

enum sched_status_type sched_flush_client_buffer(
	void *handle, void *client_hdl, void **pp_frm_data)
{

	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	struct _sched_clnt_list_node_type *p_clnt_node =
	(struct _sched_clnt_list_node_type *)client_hdl;
	enum sched_status_type status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_flush_client_buffer API");
	if (!p_sched_ctx || !p_clnt_node || !pp_frm_data) {
		SCHED_MSG_ERR
		("Bad input parameters: p_sched_ctx=%p, p_clnt_node=%p",
		p_sched_ctx, p_clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(p_clnt_node->data.clnt_cs);
	status =
	sched_process_flush_clnt_buff(p_sched_ctx, p_clnt_node,
					pp_frm_data);
	(void)SCHED_CRITSEC_LEAVE(p_clnt_node->data.clnt_cs);
	return status;
}

enum sched_status_type sched_mark_client_eof(void *handle, void *client_hdl)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	struct _sched_clnt_list_node_type *p_clnt_node =
	(struct _sched_clnt_list_node_type *)client_hdl;
	enum sched_status_type status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_mark_client_eof API");
	if (!p_sched_ctx || !p_clnt_node) {
		SCHED_MSG_ERR
		("Bad input parameters: p_sched_ctx=%p, p_clnt_node=%p",
		p_sched_ctx, p_clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(p_clnt_node->data.clnt_cs);
	status = sched_process_mark_clnt_eof(p_sched_ctx, p_clnt_node);
	(void)SCHED_CRITSEC_LEAVE(p_clnt_node->data.clnt_cs);
	return status;
}

enum sched_status_type sched_update_client_o_tkn(
	void *handle, void *client_hdl, u32 b_type, u32 n_o_tkn)
{

	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	struct _sched_clnt_list_node_type *p_clnt_node =
	(struct _sched_clnt_list_node_type *)client_hdl;
	enum sched_status_type status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_restore_client_o_tkn API");

	if (!p_sched_ctx || !p_clnt_node) {
		SCHED_MSG_ERR
		("Bad input parameters: p_sched_ctx=%p, p_clnt_node=%p",
		p_sched_ctx, p_clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(p_clnt_node->data.clnt_cs);
	status =
	sched_process_update_clnt_o_tkn(p_sched_ctx, p_clnt_node, b_type,
					n_o_tkn);
	(void)SCHED_CRITSEC_LEAVE(p_clnt_node->data.clnt_cs);
	return status;
}

enum sched_status_type sched_queue_frame(
	void *handle, void *client_hdl, void *p_frm_data)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	struct _sched_clnt_list_node_type *p_clnt_node =
	(struct _sched_clnt_list_node_type *)client_hdl;
	enum sched_status_type status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_queue_frame API");
	if (!p_sched_ctx || !p_clnt_node) {
		SCHED_MSG_ERR
		("Bad input parameters: p_sched_ctx=%p, p_clnt_node=%p",
		p_sched_ctx, p_clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(p_clnt_node->data.clnt_cs);
	status = sched_process_en_q_frm(p_sched_ctx, p_clnt_node, p_frm_data);
	(void)SCHED_CRITSEC_LEAVE(p_clnt_node->data.clnt_cs);
	return status;
}

enum sched_status_type sched_re_queue_frame(
void *handle, void *client_hdl, void *p_frm_data)
{
	struct sched_ctx_type* p_sched_ctx = (struct sched_ctx_type *)handle;
	struct _sched_clnt_list_node_type *p_clnt_node =
	(struct _sched_clnt_list_node_type *)client_hdl;
	enum sched_status_type status = SCHED_S_OK;

	SCHED_MSG_HIGH("\n sched_re_queue_frame API");
	if (!p_sched_ctx || !p_clnt_node) {
		SCHED_MSG_ERR("Bad input parameters:"
		"p_sched_ctx=%p, p_clnt_node=%p",
		p_sched_ctx, p_clnt_node);
		return SCHED_S_EBADPARM;
	}
	(void)SCHED_CRITSEC_ENTER(p_clnt_node->data.clnt_cs);
	status = sched_process_re_en_q_frm(p_sched_ctx, p_clnt_node,
			p_frm_data);
	(void)SCHED_CRITSEC_LEAVE(p_clnt_node->data.clnt_cs);
	return status;
}

enum sched_status_type sched_de_queue_frame(
	void *handle, void **pp_frm_data, void **pp_client_data)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	enum sched_status_type status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_de_queue_frame API");

	if (!p_sched_ctx || !pp_frm_data
		|| !pp_client_data) {
		SCHED_MSG_ERR("Bad input parameters: p_sched_ctx=%p, "
				"pp_frm_data=%p, pp_client_data=%p",
				p_sched_ctx, pp_frm_data,
					pp_client_data);
		return SCHED_S_EBADPARM;
	}
	(void)SCHED_CRITSEC_ENTER(p_sched_ctx->sched_cs);
	status =
	sched_process_de_q_frm(p_sched_ctx, pp_frm_data, pp_client_data);
	(void)SCHED_CRITSEC_LEAVE(p_sched_ctx->sched_cs);
	return status;
}

enum sched_status_type sched_get_client_param(
	void *handle, void *client_hdl,
	enum sched_index_type param_index,
	union sched_value_type *p_param_value)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	struct _sched_clnt_list_node_type *p_clnt_node =
	(struct _sched_clnt_list_node_type *)client_hdl;
	enum sched_status_type status;

	SCHED_MSG_HIGH("sched_get_client_param API");

	if (!p_sched_ctx || !p_clnt_node ||
			!p_param_value) {
		SCHED_MSG_ERR("Bad input parameters: p_sched_ctx=%p, "
				"p_clnt_node=%p, p_param_value=%p",
				p_sched_ctx, p_clnt_node,
				p_param_value);

		return SCHED_S_EBADPARM;
	}
	(void)SCHED_CRITSEC_ENTER(p_clnt_node->data.clnt_cs);
	status = sched_process_clnt_lvl_get_param(p_sched_ctx,
						&p_clnt_node->data,
						param_index, p_param_value);
	(void)SCHED_CRITSEC_LEAVE(p_clnt_node->data.clnt_cs);
	return status;
}

enum sched_status_type sched_set_client_param(
	void *handle, void *client_hdl,
	enum sched_index_type param_index,
	union sched_value_type *p_param_value)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	struct _sched_clnt_list_node_type *p_clnt_node =
	    (struct _sched_clnt_list_node_type *)client_hdl;
	enum sched_status_type status;

	SCHED_MSG_HIGH("sched_set_client_param API");

	if (!p_sched_ctx || !p_clnt_node ||
			!p_param_value) {
		SCHED_MSG_ERR("Bad input parameters: "
				"p_sched_ctx=%p, p_clnt_node=%p, "
				"p_param_value=%p", p_sched_ctx, p_clnt_node,
				p_param_value);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(p_sched_ctx->sched_cs);
	(void)SCHED_CRITSEC_ENTER(p_clnt_node->data.clnt_cs);

	status = sched_process_clnt_lvl_set_param(p_sched_ctx,
			&p_clnt_node->data, param_index, p_param_value);

	(void)SCHED_CRITSEC_LEAVE(p_clnt_node->data.clnt_cs);
	(void)SCHED_CRITSEC_LEAVE(p_sched_ctx->sched_cs);
	return status;
}

enum sched_status_type sched_suspend_resume_client(
	void *handle, void *client_hdl, u32 b_state)
{
	struct sched_ctx_type *p_sched_ctx = (struct sched_ctx_type *)handle;
	struct _sched_clnt_list_node_type *p_clnt_node =
	(struct _sched_clnt_list_node_type *)client_hdl;
	enum sched_status_type status;

	SCHED_MSG_HIGH("sched_client_suspend_resume API");

	if (!p_sched_ctx || !p_clnt_node) {
		SCHED_MSG_ERR
		("Bad input parameters: p_sched_ctx=%p, p_clnt_node=%p",
		p_sched_ctx, p_clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(p_clnt_node->data.clnt_cs);
	status =
	sched_process_suspend_resume_clnt(p_sched_ctx, p_clnt_node,
					b_state);
	(void)SCHED_CRITSEC_LEAVE(p_clnt_node->data.clnt_cs);
	return status;
}
