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

enum sched_status sched_create(
	struct sched_init_param *init_param, void **handle)
{
	struct sched_ctx *sched_ctx;

	SCHED_MSG_HIGH("sched_create API");

	if (!handle || !init_param) {
		SCHED_MSG_ERR
		("Bad input parameters: handle=%p, init_param=%p",
		handle, init_param);
		return SCHED_S_EBADPARM;
	}

	if (!init_param->perf_lvl) {
		SCHED_MSG_ERR("Invalid Perf level=%u",
			init_param->perf_lvl);
		return SCHED_S_EBADPARM;
	}

	sched_ctx =
		(struct sched_ctx *)
		SCHED_MALLOC(sizeof(struct sched_ctx));

	if (!sched_ctx) {
		SCHED_MSG_ERR("Could not allocate sched ctx. Out of memory");
		return SCHED_S_ENOMEM;
	}

	SCHED_MEMSET(sched_ctx, 0, sizeof(struct sched_ctx));
	sched_ctx->perf_lvl = init_param->perf_lvl;

	if (SCHED_FAILED(SCHED_CRITSEC_CREATE(&sched_ctx->sched_cs))) {
		SCHED_FREE(sched_ctx);
		return SCHED_S_EFAIL;
	}

	*handle = sched_ctx;

	SCHED_MSG_MED("Sched instance created. All went well");

	return SCHED_S_OK;

}

enum sched_status sched_destroy(void *handle)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;

	SCHED_MSG_HIGH("sched_destroy API");

	if (!sched_ctx) {
		SCHED_MSG_ERR("Bad input parameters");
		return SCHED_S_EBADPARM;
	}
	(void)SCHED_CRITSEC_ENTER(sched_ctx->sched_cs);
	(void)sched_clear_clnt_list(sched_ctx->rt_head);
	(void)sched_clear_clnt_list(sched_ctx->rt_head);
	SCHED_MSG_MED("Sched clnt lists are cleared & released");
	(void)SCHED_CRITSEC_LEAVE(sched_ctx->sched_cs);
	(void)SCHED_CRITSEC_RELEASE(sched_ctx->sched_cs);
	SCHED_MEMSET(sched_ctx, 0, sizeof(struct sched_ctx));
	SCHED_FREE(sched_ctx);
	SCHED_MSG_MED("Sched instance deleted");
	return SCHED_S_OK;
}

enum sched_status sched_get_param(
	void *handle,
	enum sched_index param_index,
	union sched_value_type *param_value)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	enum sched_status status;

	SCHED_MSG_HIGH("sched_get_param API");

	if (!sched_ctx || !param_value) {
		SCHED_MSG_ERR
		("Bad input parameters: sched_ctx=%p, param_value=%p",
		sched_ctx, param_value);

		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(sched_ctx->sched_cs);

	status =
	sched_process_sched_lvl_get_param(sched_ctx, param_index,
					param_value);

	(void)SCHED_CRITSEC_LEAVE(sched_ctx->sched_cs);
	return status;
}

enum sched_status sched_set_param(
	void *handle,
	enum sched_index param_index,
	union sched_value_type *param_value)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	enum sched_status status;

	SCHED_MSG_HIGH("sched_set_param API");

	if (!sched_ctx || !param_value) {
		SCHED_MSG_ERR
		("Bad input parameters: sched_ctx=%p, param_value=%p",
		sched_ctx, param_value);
		return SCHED_S_EBADPARM;
	}
	(void)SCHED_CRITSEC_ENTER(sched_ctx->sched_cs);
	status =
		sched_process_sched_lvl_set_param(sched_ctx, param_index,
		param_value);
	(void)SCHED_CRITSEC_LEAVE(sched_ctx->sched_cs);
	return status;
}

enum sched_status sched_add_client(
	void *handle,
	struct sched_client_init_param *init_param,
	void **client_hdl)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	enum sched_status status = SCHED_S_OK;
	struct _sched_clnt_list_node *new_clnt;

	SCHED_MSG_HIGH("sched_add_client API");

	if (!sched_ctx || !init_param ||
			!client_hdl) {
		SCHED_MSG_ERR("Bad input parameters");

		return SCHED_S_EBADPARM;
	}

	new_clnt = (struct _sched_clnt_list_node *)
	SCHED_MALLOC(sizeof(struct _sched_clnt_list_node));
	if (!new_clnt) {
		SCHED_MSG_ERR("Could not allocate client ctx. Out of memory");
		return SCHED_S_ENOMEM;
	}
	(void)SCHED_CRITSEC_ENTER(sched_ctx->sched_cs);
	status = sched_process_add_clnt(sched_ctx, new_clnt, init_param);

	if (SCHED_FAILED(status)) {
		SCHED_MSG_ERR("Add_client failed with err=%d", status);
		sched_free_clnt_node(new_clnt);
		new_clnt = NULL;
	}

	(void)SCHED_CRITSEC_LEAVE(sched_ctx->sched_cs);
	*client_hdl = new_clnt;
	SCHED_MSG_MED("Sched client instance created. All went well");
	return status;
}

enum sched_status sched_remove_client(void *handle, void *client_hdl)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	struct _sched_clnt_list_node *clnt_node =
	(struct _sched_clnt_list_node *)client_hdl;
	enum sched_status status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_remove_client API");
	if (!sched_ctx || !clnt_node) {
		SCHED_MSG_ERR
		("Bad input parameters: sched_ctx=%p, clnt_node=%p",
		sched_ctx, clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(sched_ctx->sched_cs);
	status = sched_process_remove_clnt(sched_ctx, clnt_node);
	(void)SCHED_CRITSEC_LEAVE(sched_ctx->sched_cs);
	return status;
}

enum sched_status sched_flush_client_buffer(
	void *handle, void *client_hdl, void **pp_frm_data)
{

	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	struct _sched_clnt_list_node *clnt_node =
	(struct _sched_clnt_list_node *)client_hdl;
	enum sched_status status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_flush_client_buffer API");
	if (!sched_ctx || !clnt_node || !pp_frm_data) {
		SCHED_MSG_ERR
		("Bad input parameters: sched_ctx=%p, clnt_node=%p",
		sched_ctx, clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(clnt_node->data.clnt_cs);
	status =
	sched_process_flush_clnt_buff(sched_ctx, clnt_node,
					pp_frm_data);
	(void)SCHED_CRITSEC_LEAVE(clnt_node->data.clnt_cs);
	return status;
}

enum sched_status sched_mark_client_eof(void *handle, void *client_hdl)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	struct _sched_clnt_list_node *clnt_node =
	(struct _sched_clnt_list_node *)client_hdl;
	enum sched_status status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_mark_client_eof API");
	if (!sched_ctx || !clnt_node) {
		SCHED_MSG_ERR
		("Bad input parameters: sched_ctx=%p, clnt_node=%p",
		sched_ctx, clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(clnt_node->data.clnt_cs);
	status = sched_process_mark_clnt_eof(sched_ctx, clnt_node);
	(void)SCHED_CRITSEC_LEAVE(clnt_node->data.clnt_cs);
	return status;
}

enum sched_status sched_update_client_o_tkn(
	void *handle, void *client_hdl, u32 type, u32 o_tkn)
{

	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	struct _sched_clnt_list_node *clnt_node =
	(struct _sched_clnt_list_node *)client_hdl;
	enum sched_status status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_restore_client_o_tkn API");

	if (!sched_ctx || !clnt_node) {
		SCHED_MSG_ERR
		("Bad input parameters: sched_ctx=%p, clnt_node=%p",
		sched_ctx, clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(clnt_node->data.clnt_cs);
	status =
	sched_process_update_clnt_o_tkn(sched_ctx, clnt_node, type,
					o_tkn);
	(void)SCHED_CRITSEC_LEAVE(clnt_node->data.clnt_cs);
	return status;
}

enum sched_status sched_queue_frame(
	void *handle, void *client_hdl, void *frm_data)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	struct _sched_clnt_list_node *clnt_node =
	(struct _sched_clnt_list_node *)client_hdl;
	enum sched_status status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_queue_frame API");
	if (!sched_ctx || !clnt_node) {
		SCHED_MSG_ERR
		("Bad input parameters: sched_ctx=%p, clnt_node=%p",
		sched_ctx, clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(clnt_node->data.clnt_cs);
	status = sched_process_en_q_frm(sched_ctx, clnt_node, frm_data);
	(void)SCHED_CRITSEC_LEAVE(clnt_node->data.clnt_cs);
	return status;
}

enum sched_status sched_re_queue_frame(
void *handle, void *client_hdl, void *frm_data)
{
	struct sched_ctx* sched_ctx = (struct sched_ctx *)handle;
	struct _sched_clnt_list_node *clnt_node =
	(struct _sched_clnt_list_node *)client_hdl;
	enum sched_status status = SCHED_S_OK;

	SCHED_MSG_HIGH("\n sched_re_queue_frame API");
	if (!sched_ctx || !clnt_node) {
		SCHED_MSG_ERR("Bad input parameters:"
		"sched_ctx=%p, clnt_node=%p",
		sched_ctx, clnt_node);
		return SCHED_S_EBADPARM;
	}
	(void)SCHED_CRITSEC_ENTER(clnt_node->data.clnt_cs);
	status = sched_process_re_en_q_frm(sched_ctx, clnt_node,
			frm_data);
	(void)SCHED_CRITSEC_LEAVE(clnt_node->data.clnt_cs);
	return status;
}

enum sched_status sched_de_queue_frame(
	void *handle, void **pp_frm_data, void **pp_client_data)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	enum sched_status status = SCHED_S_OK;

	SCHED_MSG_HIGH("sched_de_queue_frame API");

	if (!sched_ctx || !pp_frm_data
		|| !pp_client_data) {
		SCHED_MSG_ERR("Bad input parameters: sched_ctx=%p, "
				"pp_frm_data=%p, pp_client_data=%p",
				sched_ctx, pp_frm_data,
					pp_client_data);
		return SCHED_S_EBADPARM;
	}
	(void)SCHED_CRITSEC_ENTER(sched_ctx->sched_cs);
	status =
	sched_process_de_q_frm(sched_ctx, pp_frm_data, pp_client_data);
	(void)SCHED_CRITSEC_LEAVE(sched_ctx->sched_cs);
	return status;
}

enum sched_status sched_get_client_param(
	void *handle, void *client_hdl,
	enum sched_index param_index,
	union sched_value_type *param_value)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	struct _sched_clnt_list_node *clnt_node =
	(struct _sched_clnt_list_node *)client_hdl;
	enum sched_status status;

	SCHED_MSG_HIGH("sched_get_client_param API");

	if (!sched_ctx || !clnt_node ||
			!param_value) {
		SCHED_MSG_ERR("Bad input parameters: sched_ctx=%p, "
				"clnt_node=%p, param_value=%p",
				sched_ctx, clnt_node,
				param_value);

		return SCHED_S_EBADPARM;
	}
	(void)SCHED_CRITSEC_ENTER(clnt_node->data.clnt_cs);
	status = sched_process_clnt_lvl_get_param(sched_ctx,
						&clnt_node->data,
						param_index, param_value);
	(void)SCHED_CRITSEC_LEAVE(clnt_node->data.clnt_cs);
	return status;
}

enum sched_status sched_set_client_param(
	void *handle, void *client_hdl,
	enum sched_index param_index,
	union sched_value_type *param_value)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	struct _sched_clnt_list_node *clnt_node =
	    (struct _sched_clnt_list_node *)client_hdl;
	enum sched_status status;

	SCHED_MSG_HIGH("sched_set_client_param API");

	if (!sched_ctx || !clnt_node ||
			!param_value) {
		SCHED_MSG_ERR("Bad input parameters: "
				"sched_ctx=%p, clnt_node=%p, "
				"param_value=%p", sched_ctx, clnt_node,
				param_value);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(sched_ctx->sched_cs);
	(void)SCHED_CRITSEC_ENTER(clnt_node->data.clnt_cs);

	status = sched_process_clnt_lvl_set_param(sched_ctx,
			&clnt_node->data, param_index, param_value);

	(void)SCHED_CRITSEC_LEAVE(clnt_node->data.clnt_cs);
	(void)SCHED_CRITSEC_LEAVE(sched_ctx->sched_cs);
	return status;
}

enum sched_status sched_suspend_resume_client(
	void *handle, void *client_hdl, u32 state)
{
	struct sched_ctx *sched_ctx = (struct sched_ctx *)handle;
	struct _sched_clnt_list_node *clnt_node =
	(struct _sched_clnt_list_node *)client_hdl;
	enum sched_status status;

	SCHED_MSG_HIGH("sched_client_suspend_resume API");

	if (!sched_ctx || !clnt_node) {
		SCHED_MSG_ERR
		("Bad input parameters: sched_ctx=%p, clnt_node=%p",
		sched_ctx, clnt_node);
		return SCHED_S_EBADPARM;
	}

	(void)SCHED_CRITSEC_ENTER(clnt_node->data.clnt_cs);
	status =
	sched_process_suspend_resume_clnt(sched_ctx, clnt_node,
					state);
	(void)SCHED_CRITSEC_LEAVE(clnt_node->data.clnt_cs);
	return status;
}
