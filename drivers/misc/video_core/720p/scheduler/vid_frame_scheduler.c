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

static const u32 SCHED_TKNBKT_SIZE_FACTOR = 5;
static const u32 SCHED_TKNBKT_FILL_NORMLZ_SCALE = 100;
static const u32 SCHED_TIME_MAX = 0xffffffff;


SCHED_INLINE u32 SCHED_SUCCEEDED(enum sched_status_type status)
{
	SCHED_MSG_LOW("SCHED_SUCCEEDED check: status = %d", status);

	if (status == SCHED_S_OK)
		return TRUE;
	else
		return FALSE;

}

SCHED_INLINE u32 SCHED_FAILED(enum sched_status_type status)
{
	SCHED_MSG_LOW("SCHED_FAILED check: status = %d", status);

	if (status >= SCHED_S_EFAIL)
		return TRUE;
	else
		return FALSE;

}

static void sched_clear_clnt_ctx(struct sched_clnt_ctx_type *p_ctx)
{
	if (p_ctx->a_clnt_frm_q)
		SCHED_FREE(p_ctx->a_clnt_frm_q);
	(void)SCHED_CRITSEC_RELEASE(p_ctx->clnt_cs);
}

SCHED_INLINE void sched_free_clnt_node(
		struct _sched_clnt_list_node_type *p_clnt_node)
{
	sched_clear_clnt_ctx(&p_clnt_node->data);
	SCHED_FREE(p_clnt_node);

}

enum sched_status_type sched_clear_clnt_list(
	struct _sched_clnt_list_node_type *p_clnt_lst) {
	struct _sched_clnt_list_node_type *p_clnt_node;

	while (p_clnt_lst) {
		(void)SCHED_CRITSEC_ENTER(p_clnt_lst->data.clnt_cs);
		p_clnt_node = p_clnt_lst;
		p_clnt_lst = p_clnt_lst->p_next;
		sched_free_clnt_node(p_clnt_node);
	}
	return SCHED_S_OK;
}

static SCHED_INLINE enum sched_status_type sched_alloc_frm_q(
		struct sched_clnt_ctx_type *p_ctx)
{
	p_ctx->a_clnt_frm_q = (struct sched_clnt_q_elem *)
		SCHED_MALLOC(sizeof(struct sched_clnt_q_elem) *
			 p_ctx->n_max_queue_len);

	if (!p_ctx->a_clnt_frm_q) {
		SCHED_MSG_ERR("Could not allocate clnt frm Q. Out of memory");
		return SCHED_S_ENOMEM;
	}

	SCHED_MEMSET(p_ctx->a_clnt_frm_q,
	0, sizeof(struct sched_clnt_q_elem) * p_ctx->n_max_queue_len);
	p_ctx->n_q_head = 0;
	p_ctx->n_q_tail = -1;
	p_ctx->n_q_len = 0;
	SCHED_MSG_MED("Clnt frm Q allocted & initialized");
	return SCHED_S_OK;

}

static SCHED_INLINE void sched_de_q_head_frm
	(struct sched_clnt_ctx_type *p_ctx,
	 struct sched_clnt_q_elem *p_q_elem) {
	*p_q_elem = p_ctx->a_clnt_frm_q[p_ctx->n_q_head];

	memset(&p_ctx->a_clnt_frm_q[p_ctx->n_q_head], 0,
		sizeof(struct sched_clnt_q_elem));

	/*Update the circular queue head index.*/
	p_ctx->n_q_head = (p_ctx->n_q_head + 1) % p_ctx->n_max_queue_len;
	p_ctx->n_q_len--;
}

static SCHED_INLINE void sched_tkn_bkt_fill_normalize
	(struct sched_clnt_ctx_type *p_ctx)
{
	p_ctx->n_bkt_curr_tkns_nmlzd =
		(p_ctx->n_bkt_curr_tkns * SCHED_TKNBKT_FILL_NORMLZ_SCALE) /
		p_ctx->n_p_tkn_per_frm;
}

static void sched_tkn_bkt_config(struct sched_clnt_ctx_type *p_ctx)
{
	p_ctx->n_bkt_size = p_ctx->n_p_tkn_per_frm * SCHED_TKNBKT_SIZE_FACTOR;
	p_ctx->n_bkt_quies_cap = p_ctx->n_bkt_size;
	p_ctx->n_bkt_curr_tkns =
	SCHED_MIN(p_ctx->n_bkt_curr_tkns, p_ctx->n_bkt_size);
}

static void sched_tkn_bkt_supply(
	struct sched_clnt_ctx_type *p_ctx, u32 n_curr_time)
{
	u32 n_delta;
	u32 n_num_tkns;

	/*Check if there's time wrap-around since last token supply time.*/
	if (n_curr_time < p_ctx->n_bkt_lst_sup_time) {
		SCHED_MSG_HIGH("Current time wrap around detected");
		n_delta =
		SCHED_TIME_MAX - p_ctx->n_bkt_lst_sup_time + n_curr_time;
	} else
		n_delta = n_curr_time - p_ctx->n_bkt_lst_sup_time;

	/*Proceed only if there is any time elapsed since our last supply
	time.*/
	if (n_delta > 0) {
		/*Calculate the number of tokens that we can supply based on
		time elapsed and the client's token supply rate.*/
		n_num_tkns = n_delta * p_ctx->n_curr_p_tkn_rate / 1000;

		if (n_num_tkns > 0) {
			p_ctx->n_bkt_curr_tkns = SCHED_MIN(p_ctx->n_bkt_size,
					p_ctx->n_bkt_curr_tkns + n_num_tkns);

			if ((n_delta * p_ctx->n_curr_p_tkn_rate % 1000)) {
				n_delta = (n_num_tkns * 1000 +
						(p_ctx->n_curr_p_tkn_rate >> 1))
						/ p_ctx->n_curr_p_tkn_rate;
				if ((SCHED_TIME_MAX -
					p_ctx->n_bkt_lst_sup_time) < n_delta) {
					SCHED_MSG_HIGH
					("Handling for current time wrap "
					"around");

					p_ctx->n_bkt_lst_sup_time = n_delta -
					(SCHED_TIME_MAX -
					p_ctx->n_bkt_lst_sup_time);
				} else
					p_ctx->n_bkt_lst_sup_time += n_delta;
			} else
				p_ctx->n_bkt_lst_sup_time = n_curr_time;

			if (p_ctx->n_bkt_curr_tkns >
				(s32) p_ctx->n_bkt_quies_cap) {
				SCHED_MSG_HIGH
				("Client Quiesence detected. Capping "
				"n_bkt_curr_tkns");
				p_ctx->n_bkt_curr_tkns = p_ctx->n_p_tkn_per_frm;
			}
			sched_tkn_bkt_fill_normalize(p_ctx);
		}
	}
}

static SCHED_INLINE void sched_tkn_bkt_consume(
	struct sched_clnt_ctx_type *p_ctx) {
	p_ctx->n_bkt_curr_tkns -= p_ctx->n_p_tkn_per_frm;
}

static SCHED_INLINE u32 sched_clnt_frm_is_cnfmnt
	(struct sched_clnt_ctx_type *p_ctx)
{
	if (p_ctx->n_bkt_curr_tkns >= (s32) p_ctx->n_p_tkn_per_frm)
		return TRUE;
	else
		return FALSE;
}				/* end of sched_clnt_frm_is_conformant */

static struct sched_clnt_ctx_type *sched_elect_cnfmnt
	(struct sched_clnt_ctx_type *p_prov_elect,
	struct sched_clnt_ctx_type *p_new_cand) {

	/*If there is no provisional elect client then the new candidate
	becomes the first one.*/
	if (!p_prov_elect)
		return p_new_cand;


	/*Here we want to pick the client who has accumulated the most tokens
	from the time of attaining single frame conformance.
	Since we are comparing between clients we use the available normalized
	token bucket occupancy value.*/
	if (p_prov_elect->n_bkt_curr_tkns_nmlzd >=
	    p_new_cand->n_bkt_curr_tkns_nmlzd) {
		return p_prov_elect;
	} else {
	/*We had held on to this provisional elect conformant
	client critical section. Since new candidate has won the
	election leave critical section of earlier provisional
	elect.
	*/
		(void)SCHED_CRITSEC_LEAVE(p_prov_elect->clnt_cs);
		return p_new_cand;
	}
}

static struct sched_clnt_ctx_type *sched_elect_non_cnfmnt
	(struct sched_clnt_ctx_type *p_prov_elect,
	struct sched_clnt_ctx_type *p_new_cand) {

	/*If there is no provisional elect client then the new candidate
	becomes the first one.*/
	if (!p_prov_elect)
		return p_new_cand;
	/*Here we want to pick the client who is closest to attaining a single
	frame conformance.
	Since we are comparing between clients we use the available
	normalized token bucket occupancy value.
	Also if the provisional elect or the new contender (in that order)
	have an end of frame marker set we give it priority over deciding
	by frame conformance method mentiond earlier.*/
	if (p_prov_elect->n_eof_marker > 0) {
		return p_prov_elect;
	} else if (p_new_cand->n_eof_marker > 0) {
		/*We had held on to this provisional elect non conformant client
		critical section. Since new candidate has won the election
		leave critical section of earlier provisional elect.
		*/
		(void)SCHED_CRITSEC_LEAVE(p_prov_elect->clnt_cs);

		return p_new_cand;
	} else if (p_prov_elect->n_bkt_curr_tkns_nmlzd >=
		   p_new_cand->n_bkt_curr_tkns_nmlzd) {
		return p_prov_elect;
	} else {
	/*Similar to above case leave critical section of earlier
	provisional elect.*/
		(void)SCHED_CRITSEC_LEAVE(p_prov_elect->clnt_cs);
		return p_new_cand;
	}

}

static struct sched_clnt_ctx_type *sched_elect_non_rt
	(struct sched_ctx_type *p_sched_ctx) {
	struct _sched_clnt_list_node_type *p_node = NULL;
	struct _sched_clnt_list_node_type *p_start_node = NULL;
	u32 b_found = FALSE;

	/*For non real time clients we are using a round robin election
	algorithm.
	Based on the last scheduled client we find the next to schedule
	and return its context.
	We also need to skip the client if certain conditions (mentioned below)
	are not met*/
	if (!p_sched_ctx->p_n_rt_last_sched)
		p_start_node = p_node = p_sched_ctx->p_n_rt_head;
	else {
		if (!p_sched_ctx->p_n_rt_last_sched->p_next)
			p_start_node = p_sched_ctx->p_n_rt_head;
		else
			p_start_node = p_sched_ctx->p_n_rt_last_sched->p_next;

		p_node = p_start_node;
	}

	do {

		(void)SCHED_CRITSEC_ENTER(p_node->data.clnt_cs);

	/*Check if the client can be considered for this round of scheduling.*/
		if (sched_consider_clnt_for_sched(&p_node->data)) {
			b_found = TRUE;
			p_sched_ctx->p_n_rt_last_sched = p_node;
		}

	/*If this client is not the election winner then leave its critical
	section.
	If we have found a winner we want to hold on to its critical
	section. We would leave its critical section after we are done
	with dequeueing a frame from the client context.*/
		if (!b_found)
			(void)SCHED_CRITSEC_LEAVE(p_node->data.clnt_cs);

		if (!p_node->p_next)
			p_node = p_sched_ctx->p_n_rt_head;
		else
			p_node = p_node->p_next;

	} while (p_node != p_start_node);

	if (b_found) {
		SCHED_MSG_LOW("Non real time client selected");

		return &p_sched_ctx->p_n_rt_last_sched->data;
	} else {
		SCHED_MSG_MED
		    ("No non-real time client available for scheduling");

		return NULL;
	}

}

static enum sched_status_type sched_process_set_p_tkn_rate(
		struct sched_ctx_type *p_sched_ctx,
	struct sched_clnt_ctx_type *p_clnt_ctx,
	union sched_value_type *p_param_value) {
	u32 n_curr_time = 0;

	if (p_param_value->un_value == p_clnt_ctx->n_curr_p_tkn_rate)
		return SCHED_S_OK;


	if ((p_sched_ctx->n_total_clnt_bw - p_clnt_ctx->n_curr_p_tkn_rate +
	     p_param_value->un_value) > p_sched_ctx->n_perf_lvl) {
		SCHED_MSG_HIGH
		    ("Perf level insufficient for requested P Tkn rate");

	}

	/*Get current time. We need this for token supply.
	If we didn't get a valid current time value just return*/
	if (SCHED_FAILED(SCHED_GET_CURRENT_TIME(&n_curr_time))) {
		SCHED_MSG_ERR("Get current time failed");

		return SCHED_S_EFAIL;
	}

	/*Before we go ahead and update the Current p_tkn rate, we fill
	the token bucket upto current time instance.*/
	sched_tkn_bkt_supply(p_clnt_ctx, n_curr_time);

	/*Next, update the current value of total client bandwidth with
	the new p_tkn rate of the client.*/
	p_sched_ctx->n_total_clnt_bw = p_sched_ctx->n_total_clnt_bw -
	    p_clnt_ctx->n_curr_p_tkn_rate + p_param_value->un_value;
	p_clnt_ctx->n_curr_p_tkn_rate = p_param_value->un_value;

	/*Since the current Ptkn rate (i.e. current alloted bandwidth)
	of the client has changed we need to update client's token
	bucket configuration*/
	sched_tkn_bkt_config(p_clnt_ctx);
	return SCHED_S_OK;
}

static enum sched_status_type sched_process_add_rt_clnt(
		struct sched_ctx_type *p_sched_ctx,
	struct _sched_clnt_list_node_type *p_clnt_node) {
	enum sched_status_type status;
	struct sched_clnt_ctx_type *p_clnt_ctx = &p_clnt_node->data;
	struct _sched_clnt_list_node_type *p_tmp_node;

	/*Validate real time client specific parameters.*/
	if (!p_clnt_ctx->n_curr_p_tkn_rate)
		SCHED_MSG_HIGH("Allocated token rate is zero");

	/*Check if our performance level setting can sustain the new client*/
	if (p_sched_ctx->n_total_clnt_bw + p_clnt_ctx->n_curr_p_tkn_rate >
	    p_sched_ctx->n_perf_lvl) {
		SCHED_MSG_HIGH("Not enough bandwidth to support client");
		SCHED_MSG_HIGH
		    ("curr_perflvl=%d, curr_bw=%d, newclnt_ptknrate=%d",
		     p_sched_ctx->n_perf_lvl, p_sched_ctx->n_total_clnt_bw,
		     p_clnt_ctx->n_curr_p_tkn_rate);

	}
	/*Allocate the client frame queue*/
	status = sched_alloc_frm_q(p_clnt_ctx);

	if (SCHED_SUCCEEDED(status)) {
		/*Allocate the token bucket*/
		sched_tkn_bkt_config(p_clnt_ctx);
		/*We start with empty token bucket*/
		p_clnt_ctx->n_bkt_curr_tkns = 0;
		p_clnt_ctx->n_bkt_curr_tkns_nmlzd = 0;
		/*Add the client to the real time client list and increase the
		total client bandwidth.*/
		p_tmp_node = p_sched_ctx->p_rt_head;
		p_sched_ctx->p_rt_head = p_clnt_node;
		p_sched_ctx->p_rt_head->p_next = p_tmp_node;
		p_sched_ctx->n_rt_clnts++;
		p_sched_ctx->n_total_clnt_bw += p_clnt_ctx->n_curr_p_tkn_rate;
	}
	return status;
}

static enum sched_status_type sched_process_add_non_rt_clnt(
		struct sched_ctx_type *p_sched_ctx,
	struct _sched_clnt_list_node_type *p_clnt_node) {
	enum sched_status_type status;
	struct sched_clnt_ctx_type *p_clnt_ctx = &p_clnt_node->data;
	struct _sched_clnt_list_node_type *p_tmp_node;

	/*Allocate the client frame queue*/
	status = sched_alloc_frm_q(p_clnt_ctx);
	if (SCHED_SUCCEEDED(status)) {
		/*Add the client to the real time client list and increase the
		total client bandwidth.*/
		p_tmp_node = p_sched_ctx->p_n_rt_head;
		p_sched_ctx->p_n_rt_head = p_clnt_node;
		p_sched_ctx->p_n_rt_head->p_next = p_tmp_node;
		p_sched_ctx->n_n_rt_clnts++;
	}
	return status;
}

enum sched_status_type sched_process_add_clnt(
	struct sched_ctx_type *p_sched_ctx,
	struct _sched_clnt_list_node_type *p_clnt_node,
	struct sched_client_init_param_type *p_init_param) {
	enum sched_status_type status = SCHED_S_OK;

	SCHED_MEMSET(p_clnt_node, 0, sizeof(struct _sched_clnt_list_node_type));

	/*Validate all initialization parameters*/
	if (!p_init_param->n_p_tkn_per_frm ||
	    !p_init_param->frm_rate.n_numer ||
	    !p_init_param->frm_rate.n_denom ||
	    !p_init_param->n_max_queue_len ||
	    !p_init_param->n_o_tkn_max ||
	    !p_init_param->n_o_tkn_per_ip_frm ||
	    p_init_param->n_o_tkn_init > p_init_param->n_o_tkn_max ||
	    p_init_param->n_o_tkn_per_ip_frm > p_init_param->n_o_tkn_max) {
		SCHED_MSG_ERR("Bad initialization parameters");
		return SCHED_S_EBADPARM;
	}

	/*Store all initialization parameters*/
	p_clnt_node->data.client_ctgy = p_init_param->client_ctgy;
	p_clnt_node->data.n_curr_p_tkn_rate = p_init_param->n_alloc_p_tkn_rate;
	p_clnt_node->data.frm_rate = p_init_param->frm_rate;
	p_clnt_node->data.n_max_queue_len = p_init_param->n_max_queue_len;
	p_clnt_node->data.n_o_tkn_max = p_init_param->n_o_tkn_max;
	p_clnt_node->data.n_o_tkn_per_ip_frm = p_init_param->n_o_tkn_per_ip_frm;
	p_clnt_node->data.n_curr_o_tkns = p_init_param->n_o_tkn_init;
	p_clnt_node->data.n_p_tkn_per_frm = p_init_param->n_p_tkn_per_frm;
	p_clnt_node->data.p_client_data = p_init_param->p_client_data;
	p_clnt_node->data.b_sched_state = TRUE;

	SCHED_MSG_HIGH("Adding new client of category %d",
		       p_clnt_node->data.client_ctgy);
	SCHED_MSG_MED("Allocated P token rate (per sec) = %d",
		      p_clnt_node->data.n_curr_p_tkn_rate);
	SCHED_MSG_MED("Frame rate = %d / %d",
		      p_clnt_node->data.frm_rate.n_numer,
		      p_clnt_node->data.frm_rate.n_denom);
	SCHED_MSG_MED("Max_queue_len = %d", p_clnt_node->data.n_max_queue_len);
	SCHED_MSG_MED("Max O tokens = %d", p_clnt_node->data.n_o_tkn_max);
	SCHED_MSG_MED("O tokens threshold = %d",
		      p_clnt_node->data.n_o_tkn_per_ip_frm);
	SCHED_MSG_MED("P tokens per frame = %d",
		      p_clnt_node->data.n_p_tkn_per_frm);
	SCHED_MSG_MED("Client data ptr = %p", p_clnt_node->data.p_client_data);

	if (SCHED_FAILED(SCHED_CRITSEC_CREATE(&p_clnt_node->data.clnt_cs)))
		return SCHED_S_EFAIL;

	/*Configure the client context based on client category.*/
	switch (p_clnt_node->data.client_ctgy) {
	case SCHED_CLNT_RT_BUFF:
	case SCHED_CLNT_RT_NOBUFF:
		{
			status =
			    sched_process_add_rt_clnt(p_sched_ctx, p_clnt_node);
			break;
		}

	case SCHED_CLNT_NONRT:
		{
			status =
			    sched_process_add_non_rt_clnt(p_sched_ctx,
							  p_clnt_node);
			break;
		}

	default:
		{
			status = SCHED_S_EBADPARM;
			break;
		}

	}
	return status;
}

enum sched_status_type sched_process_remove_clnt(
		struct sched_ctx_type *p_sched_ctx,
	struct _sched_clnt_list_node_type *p_clnt_node) {

	(void)SCHED_CRITSEC_ENTER(p_clnt_node->data.clnt_cs);

	/*Handling if the client frame queue is not empty. Just return
	and let Codec driver dequeue all frames for this client
	before calling remove client*/
	if (p_clnt_node->data.n_q_len) {
		SCHED_MSG_ERR("Cannot remove client. Queue is not empty");
		return SCHED_S_EINVALST;
	}

	/*Based on client category, remove the client node from the
	appropriate scheduler client list*/
	switch (p_clnt_node->data.client_ctgy) {
	case SCHED_CLNT_RT_BUFF:
	case SCHED_CLNT_RT_NOBUFF:
		{

			sched_remove_node_from_list(&p_sched_ctx->p_rt_head,
						    p_clnt_node);
			p_sched_ctx->n_rt_clnts--;
			p_sched_ctx->n_total_clnt_bw -=
			    p_clnt_node->data.n_curr_p_tkn_rate;
			break;
		}

	case SCHED_CLNT_NONRT:
		{
			sched_remove_node_from_list(&p_sched_ctx->p_n_rt_head,
						    p_clnt_node);
			p_sched_ctx->n_n_rt_clnts--;
			break;
		}

	default:
		{
			SCHED_ASSERT(0);
			break;
		}
	}

	/*Now that client node is off the scheduler client list free up
	resources that its been using.*/
	SCHED_MSG_HIGH("Removing new client of category %d",
		       p_clnt_node->data.client_ctgy);
	SCHED_MSG_MED("Allocated P token rate (per sec) = %d",
		      p_clnt_node->data.n_curr_p_tkn_rate);
	SCHED_MSG_MED("Frame rate = %d / %d",
		      p_clnt_node->data.frm_rate.n_numer,
		      p_clnt_node->data.frm_rate.n_denom);
	SCHED_MSG_MED("Max_queue_len = %d", p_clnt_node->data.n_max_queue_len);
	SCHED_MSG_MED("Max O tokens = %d", p_clnt_node->data.n_o_tkn_max);
	SCHED_MSG_MED("P tokens per frame = %d",
		      p_clnt_node->data.n_p_tkn_per_frm);
	SCHED_MSG_MED("Client data ptr = %p", p_clnt_node->data.p_client_data);
	sched_free_clnt_node(p_clnt_node);
	return SCHED_S_OK;
}

enum sched_status_type sched_process_flush_clnt_buff(
		struct sched_ctx_type *p_sched_ctx,
	struct _sched_clnt_list_node_type *p_clnt_node, void **pp_frm_data) {
	struct sched_clnt_ctx_type *p_clnt_ctx;
	enum sched_status_type status = SCHED_S_OK;
	struct sched_clnt_q_elem q_elem;

	p_clnt_ctx = &p_clnt_node->data;

	/*If the client queue is empty just return an QEMPTY status*/
	if (!p_clnt_ctx->n_q_len) {
		status = SCHED_S_QEMPTY;
	} else {
		p_clnt_ctx->b_flushing = TRUE;

	/*If the client queue is not empty just remove and return the
	element at the front of the queue.*/
		sched_de_q_head_frm(p_clnt_ctx, &q_elem);
		*pp_frm_data = q_elem.p_frm_data;
	}

	/*If the Queue was orginially empty OR if it got empty after latest
	De_queue we reset the flushing and First_frame flags.
	Token bucket contents are also emptied.Queue pointers are reset.
	o_tkns are restored.*/
	if (!p_clnt_ctx->n_q_len) {
		p_clnt_ctx->b_flushing = FALSE;
		p_clnt_ctx->b_first_frm = FALSE;
		p_clnt_ctx->n_bkt_curr_tkns = 0;
		p_clnt_ctx->n_bkt_curr_tkns_nmlzd = 0;
		p_clnt_ctx->n_bkt_lst_sup_time = 0;
		p_clnt_ctx->n_q_head = 0;
		p_clnt_ctx->n_q_tail = -1;
		SCHED_MSG_HIGH
		    ("Client flushed and re-initialized. Client category %d",
		     p_clnt_ctx->client_ctgy);
		SCHED_MSG_MED("Client allocated P token rate (per sec) = %d",
			      p_clnt_ctx->n_curr_p_tkn_rate);
		SCHED_MSG_MED("Client frame rate = %d / %d",
			      p_clnt_ctx->frm_rate.n_numer,
			      p_clnt_ctx->frm_rate.n_denom);
		SCHED_MSG_MED("Client P tokens per frame = %d",
			      p_clnt_ctx->n_p_tkn_per_frm);
	}
	return status;
}

SCHED_INLINE enum sched_status_type sched_process_mark_clnt_eof(
		struct sched_ctx_type *p_sched_ctx,
	struct _sched_clnt_list_node_type *p_clnt_node) {

	if (!p_clnt_node->data.n_q_len)
		return SCHED_S_QEMPTY;


	if (!p_clnt_node->data.a_clnt_frm_q[p_clnt_node->data.n_q_tail].b_eof) {
		/*Just increment the EOF marker count in the client context.*/
		p_clnt_node->data.n_eof_marker++;
		p_clnt_node->data.a_clnt_frm_q[p_clnt_node->data.n_q_tail].
		b_eof = TRUE;
	} else
		SCHED_MSG_HIGH("Current frame is already marked EOF");

	SCHED_MSG_HIGH("Client marked for end of frames. Client category %d",
		       p_clnt_node->data.client_ctgy);
	SCHED_MSG_MED("Client allocated P token rate (per sec) = %d",
		      p_clnt_node->data.n_curr_p_tkn_rate);
	SCHED_MSG_MED("Client frame rate = %d / %d",
		      p_clnt_node->data.frm_rate.n_numer,
		      p_clnt_node->data.frm_rate.n_denom);
	SCHED_MSG_MED("Client P tokens per frame = %d",
		      p_clnt_node->data.n_p_tkn_per_frm);
	return SCHED_S_OK;
}

enum sched_status_type sched_process_update_clnt_o_tkn(
		struct sched_ctx_type *p_sched_ctx,
	struct _sched_clnt_list_node_type *p_clnt_node,
	u32 b_type, u32 n_o_tkn) {

	/*Act based on the type of update.*/

	if (b_type) {
		/*Just replenish the output tokens the client currently has with
		the provided number while not going over the max value.*/
		p_clnt_node->data.n_curr_o_tkns =
		    SCHED_MIN(p_clnt_node->data.n_curr_o_tkns + n_o_tkn,
			      p_clnt_node->data.n_o_tkn_max);
	} else {
	/*Just subtract the give number of output tokens from the count
	the client currently has while not going less than 0.*/
		if (n_o_tkn >= p_clnt_node->data.n_curr_o_tkns)
			p_clnt_node->data.n_curr_o_tkns = 0;
		else
			p_clnt_node->data.n_curr_o_tkns -= n_o_tkn;

	}

	SCHED_MSG_LOW("%d O tokens restored for client", n_o_tkn);
	SCHED_MSG_LOW("Client Curr_o_tkns = %d",
		      p_clnt_node->data.n_curr_o_tkns);
	SCHED_MSG_LOW("Client category = %d", p_clnt_node->data.client_ctgy);
	SCHED_MSG_LOW("Client allocated P token rate (per sec) = %d",
		      p_clnt_node->data.n_curr_p_tkn_rate);
	SCHED_MSG_LOW("Client frame rate = %d / %d",
		      p_clnt_node->data.frm_rate.n_numer,
		      p_clnt_node->data.frm_rate.n_denom);
	SCHED_MSG_LOW("Client P tokens per frame = %d",
		      p_clnt_node->data.n_p_tkn_per_frm);
	return SCHED_S_OK;
}

enum sched_status_type sched_process_en_q_frm(
		struct sched_ctx_type *p_sched_ctx,
	struct _sched_clnt_list_node_type *p_clnt_node, void *p_frm_data) {
	struct sched_clnt_ctx_type *p_clnt_ctx;
	u32 n_curr_time = 0;

	p_clnt_ctx = &p_clnt_node->data;

	/*Check if the client queue is full already*/
	if (p_clnt_ctx->n_q_len == p_clnt_ctx->n_max_queue_len) {
		SCHED_MSG_HIGH("Cannot enqueue. Client queue is full");

		return SCHED_S_QFULL;
	}

	/*Check if the client queue is being flushed.*/
	if (p_clnt_ctx->b_flushing) {
		SCHED_MSG_ERR("Cannot enqueue. Client queue is being flushed");

		return SCHED_S_EINVALST;
	}

	/*Reposition tail, increase Q length and add the frame data to Q*/
	p_clnt_ctx->n_q_tail =
	    (p_clnt_ctx->n_q_tail + 1) % p_clnt_ctx->n_max_queue_len;

	p_clnt_ctx->n_q_len++;

	p_clnt_ctx->a_clnt_frm_q[p_clnt_ctx->n_q_tail].p_frm_data = p_frm_data;
	p_clnt_ctx->a_clnt_frm_q[p_clnt_ctx->n_q_tail].b_eof = FALSE;

	/*If this is the first frame being queued for this client then,
	get current time. We now start the token supply clock for the client.
	Supply tokens required for a single frame processing while storing
	the current time as the last supply time and marking that first
	frame is received.*/
	if (!p_clnt_ctx->b_first_frm) {
		SCHED_MSG_HIGH("Client first frame enqueued");
		if (p_clnt_ctx->client_ctgy != SCHED_CLNT_NONRT) {
			if (SCHED_SUCCEEDED
			(SCHED_GET_CURRENT_TIME(&n_curr_time))) {
				p_clnt_ctx->n_bkt_curr_tkns =
					p_clnt_ctx->n_p_tkn_per_frm;
				p_clnt_ctx->n_bkt_lst_sup_time = n_curr_time;
				p_clnt_ctx->b_first_frm = TRUE;
			}
		} else
			p_clnt_ctx->b_first_frm = TRUE;
	}

	SCHED_MSG_LOW("Client frame enqueued. Queue fill status = %d / %d",
			p_clnt_ctx->n_q_len, p_clnt_ctx->n_max_queue_len);
	SCHED_MSG_LOW("Client category = %d", p_clnt_ctx->client_ctgy);
	SCHED_MSG_LOW("Client allocated P token rate (per sec) = %d",
		p_clnt_ctx->n_curr_p_tkn_rate);
	SCHED_MSG_LOW("Client frame rate = %d / %d",
		p_clnt_ctx->frm_rate.n_numer,
		p_clnt_ctx->frm_rate.n_denom);
	SCHED_MSG_LOW("Client P tokens per frame = %d",
		p_clnt_ctx->n_p_tkn_per_frm);

	return SCHED_S_OK;

}

enum sched_status_type sched_process_re_en_q_frm(
	struct sched_ctx_type *p_sched_ctx,
	struct _sched_clnt_list_node_type *p_clnt_node,
	void *p_frm_data)
{
	struct sched_clnt_ctx_type *p_clnt_ctx;
	u32 n_curr_time = 0;

	p_clnt_ctx = &p_clnt_node->data;

	if (p_clnt_ctx->n_q_len == p_clnt_ctx->n_max_queue_len) {
		SCHED_MSG_ERR("Cannot re-enqueue. Client queue is full");
		return SCHED_S_QFULL;
	}

	if (p_clnt_ctx->b_flushing) {
		SCHED_MSG_ERR("Cannot re-enqueue. Client"
					" queue is being flushed");
		return SCHED_S_EINVALST;
	}

	p_clnt_ctx->n_q_head =
		(p_clnt_ctx->n_q_head + p_clnt_ctx->n_max_queue_len - 1) %
		p_clnt_ctx->n_max_queue_len;

	p_clnt_ctx->n_q_len++;

	p_clnt_ctx->a_clnt_frm_q[p_clnt_ctx->n_q_head].p_frm_data =
		p_frm_data;
	p_clnt_ctx->a_clnt_frm_q[p_clnt_ctx->n_q_head].b_eof =
		FALSE;

	if (p_clnt_ctx->client_ctgy != SCHED_CLNT_NONRT) {
		if (!p_clnt_ctx->b_first_frm) {
			SCHED_MSG_HIGH("Client frame "
						"re-enqueued as first frame");
			if (SCHED_SUCCEEDED
			(SCHED_GET_CURRENT_TIME(&n_curr_time))) {
				p_clnt_ctx->n_bkt_curr_tkns =
					p_clnt_ctx->n_p_tkn_per_frm;
				p_clnt_ctx->n_bkt_lst_sup_time =
					n_curr_time;
				p_clnt_ctx->b_first_frm =
					TRUE;
			}
		} else
			p_clnt_ctx->n_bkt_curr_tkns +=
				p_clnt_ctx->n_p_tkn_per_frm;
	} else
		p_clnt_ctx->b_first_frm = TRUE;


	SCHED_MSG_LOW("Client frame re-enqueued. Queue fill status = %d / %d",
	p_clnt_ctx->n_q_len, p_clnt_ctx->n_max_queue_len);
	SCHED_MSG_LOW("Client category = %d", p_clnt_ctx->client_ctgy);
	SCHED_MSG_LOW("Client allocated P token rate (per sec) = %d",
	p_clnt_ctx->n_curr_p_tkn_rate);
	SCHED_MSG_LOW("Client frame rate = %d / %d",
			p_clnt_ctx->frm_rate.n_numer,
			p_clnt_ctx->frm_rate.n_denom);
	SCHED_MSG_LOW("Client P tokens per frame = %d",
			p_clnt_ctx->n_p_tkn_per_frm);

	return SCHED_S_OK;

}

enum sched_status_type sched_process_de_q_frm_rt_clnt(
		struct sched_ctx_type *p_sched_ctx,
	struct sched_clnt_ctx_type **pp_conf_elect_ctx,
	struct sched_clnt_ctx_type **pp_non_conf_elect_ctx) {
	u32 n_curr_time = 0;
	struct _sched_clnt_list_node_type *p_clnt_node;
	struct sched_clnt_ctx_type *p_clnt_ctx;

	*pp_conf_elect_ctx = NULL;
	*pp_non_conf_elect_ctx = NULL;

	/*Get current time. We need this for token supply.
	If we didn't get a valid current time value just return*/
	if (SCHED_FAILED(SCHED_GET_CURRENT_TIME(&n_curr_time))) {
		SCHED_MSG_ERR("Get current time failed");

		return SCHED_S_EFAIL;
	}

	/*Run through the list of real time clients.
	Consider only the clients that have queued atleast one frame since
	being admitted into the scheduler.
	Supply tokens equivalent to elapsed time since last supply.
	Also in this same pass, check if each client has a conformant
	frame or not.*/
	p_clnt_node = p_sched_ctx->p_rt_head;
	while (p_clnt_node) {
		p_clnt_ctx = &p_clnt_node->data;

		(void)SCHED_CRITSEC_ENTER(p_clnt_ctx->clnt_cs);

		if (sched_consider_clnt_for_sched(p_clnt_ctx)) {
			sched_tkn_bkt_supply(p_clnt_ctx, n_curr_time);
			if (sched_clnt_frm_is_cnfmnt(p_clnt_ctx)) {
				*pp_conf_elect_ctx =
					sched_elect_cnfmnt(*pp_conf_elect_ctx,
						p_clnt_ctx);
			} else {
				if (!*pp_conf_elect_ctx) {
					*pp_non_conf_elect_ctx =
					    sched_elect_non_cnfmnt
					    (*pp_non_conf_elect_ctx,
					     p_clnt_ctx);
				} else if (*pp_non_conf_elect_ctx) {
					(void)
					    SCHED_CRITSEC_LEAVE(
					    (*pp_non_conf_elect_ctx)->clnt_cs);
					*pp_non_conf_elect_ctx = NULL;

				}
			}
		}
		if (p_clnt_ctx != *pp_conf_elect_ctx
		    && p_clnt_ctx != *pp_non_conf_elect_ctx)
			(void)SCHED_CRITSEC_LEAVE(p_clnt_ctx->clnt_cs);
		p_clnt_node = p_clnt_node->p_next;
	}

	return SCHED_S_OK;

}

enum sched_status_type sched_process_de_q_frm(
		struct sched_ctx_type *p_sched_ctx,
	void **pp_frm_data, void **pp_client_data) {
	enum sched_status_type status;
	struct sched_clnt_ctx_type *p_sched_clnt_ctx = NULL;
	struct sched_clnt_ctx_type *p_conf_elect_ctx;
	struct sched_clnt_ctx_type *p_non_conf_elect_ctx;
	struct sched_clnt_q_elem q_elem;

	status = sched_process_de_q_frm_rt_clnt(p_sched_ctx,
						&p_conf_elect_ctx,
						&p_non_conf_elect_ctx);
	if (SCHED_FAILED(status)) {
		SCHED_MSG_ERR("sched_process_de_q_frm_rt_clnt ret err=%d",
			      status);

		return status;
	}

	/*At this point we have looked at all real time clients in the
	scheduler list and have run their elections.
	We used the following frame service order to pick the client to
	schedule:
	a) client with conformant frame
	b) client with non-conformant frame
	c) non real-time client*/
	if (p_conf_elect_ctx) {
		SCHED_MSG_LOW("Conformant frame client selected");
		sched_tkn_bkt_consume(p_conf_elect_ctx);
		p_sched_clnt_ctx = p_conf_elect_ctx;
	} else if (p_non_conf_elect_ctx) {
		SCHED_MSG_LOW("Non-Conformant frame client selected");
		sched_tkn_bkt_consume(p_non_conf_elect_ctx);
		p_sched_clnt_ctx = p_non_conf_elect_ctx;
	} else if (p_sched_ctx->n_n_rt_clnts)
		p_sched_clnt_ctx = sched_elect_non_rt(p_sched_ctx);

	/*If we have a client that we can schedule, then dequeue the frame
	at the head of its queue.*/
	if (p_sched_clnt_ctx) {
		*pp_client_data = p_sched_clnt_ctx->p_client_data;

		sched_de_q_head_frm(p_sched_clnt_ctx, &q_elem);

		*pp_frm_data = q_elem.p_frm_data;

		p_sched_clnt_ctx->n_curr_o_tkns -=
		    p_sched_clnt_ctx->n_o_tkn_per_ip_frm;

	/*If the dequeued frame was marked EOF we need to decrement the
	eof_marker count.*/
		if (q_elem.b_eof) {
			SCHED_MSG_MED
			    ("Last frame for EOF marked client dequeued");

			p_sched_clnt_ctx->n_eof_marker--;

			status = SCHED_S_EOF;
		}

		SCHED_MSG_LOW
		    ("Client frame Dequeued. Queue fill status = %d / %d",
		     p_sched_clnt_ctx->n_q_len,
		     p_sched_clnt_ctx->n_max_queue_len);
		SCHED_MSG_LOW("Client category = %d",
			      p_sched_clnt_ctx->client_ctgy);
		SCHED_MSG_LOW("Client allocated P token rate (per sec) = %d",
			      p_sched_clnt_ctx->n_curr_p_tkn_rate);
		SCHED_MSG_LOW("Client frame rate = %d / %d",
			      p_sched_clnt_ctx->frm_rate.n_numer,
			      p_sched_clnt_ctx->frm_rate.n_denom);
		SCHED_MSG_LOW("Client P tokens per frame = %d",
			      p_sched_clnt_ctx->n_p_tkn_per_frm);

	/*We had held on to the election winning client critical
	section. Leave client critical section before we exit.*/
		(void)SCHED_CRITSEC_LEAVE(p_sched_clnt_ctx->clnt_cs);
	} else {
		status = SCHED_S_QEMPTY;
	}

	return status;

}

enum sched_status_type sched_process_sched_lvl_get_param(
		struct sched_ctx_type *p_sched_ctx,
	enum sched_index_type param_index,
	union sched_value_type *p_param_value)
{
	enum sched_status_type status = SCHED_S_OK;

	switch (param_index) {
	case SCHED_I_PERFLEVEL:
		{
			p_param_value->un_value = p_sched_ctx->n_perf_lvl;
			break;
		}

	default:
		{
			status = SCHED_S_EBADPARM;
			break;
		}
	}
	return status;
}

enum sched_status_type sched_process_sched_lvl_set_param(
		struct sched_ctx_type *p_sched_ctx,
	enum sched_index_type param_index,
	union sched_value_type *p_param_value)
{
	enum sched_status_type status = SCHED_S_OK;

	SCHED_MSG_HIGH("Set_sched_param index = %u, value = %p",
		       param_index, (void *)p_param_value);

	switch (param_index) {
	case SCHED_I_PERFLEVEL:
		{
			if (p_sched_ctx->n_total_clnt_bw >
			    p_param_value->un_value) {
				SCHED_MSG_HIGH
				    ("Perf level being lowered than current "
				     "bandwidth");
				SCHED_MSG_HIGH
				    ("curr_perflvl=%d, new_perflvl=%d, "
				     "curr_bw=%d",
				     p_sched_ctx->n_perf_lvl,
				     p_param_value->un_value,
				     p_sched_ctx->n_total_clnt_bw);
			}

			p_sched_ctx->n_perf_lvl = p_param_value->un_value;

			break;
		}

	default:
		{
			status = SCHED_S_EBADPARM;
			break;
		}
	}
	return status;
}

enum sched_status_type sched_process_clnt_lvl_get_param(
		struct sched_ctx_type *p_sched_ctx,
	struct sched_clnt_ctx_type *p_clnt_ctx,
	enum sched_index_type param_index,
	union sched_value_type *p_param_value) {
	enum sched_status_type status = SCHED_S_OK;

	switch (param_index) {
	case SCHED_I_CLNT_CURRQLEN:
		{
			p_param_value->un_value = p_clnt_ctx->n_q_len;
			break;
		}

	case SCHED_I_CLNT_PTKNRATE:
		{
			p_param_value->un_value = p_clnt_ctx->n_curr_p_tkn_rate;
			break;
		}

	case SCHED_I_CLNT_PTKNPERFRM:
		{
			p_param_value->un_value = p_clnt_ctx->n_p_tkn_per_frm;
			break;
		}

	case SCHED_I_CLNT_FRAMERATE:
		{
			p_param_value->frm_rate = p_clnt_ctx->frm_rate;
			break;
		}

	case SCHED_I_CLNT_OTKNMAX:
		{
			p_param_value->un_value = p_clnt_ctx->n_o_tkn_max;
			break;
		}

	case SCHED_I_CLNT_OTKNPERIPFRM:
		{
			p_param_value->un_value =
			    p_clnt_ctx->n_o_tkn_per_ip_frm;
			break;
		}

	case SCHED_I_CLNT_OTKNCURRENT:
		{
			p_param_value->un_value = p_clnt_ctx->n_curr_o_tkns;
			break;
		}

	default:
		{
			status = SCHED_S_EBADPARM;
			break;
		}
	}
	return status;
}

enum sched_status_type sched_process_clnt_lvl_set_param(
		struct sched_ctx_type *p_sched_ctx,
	struct sched_clnt_ctx_type *p_clnt_ctx,
	enum sched_index_type param_index,
	union sched_value_type *p_param_value)
{
	enum sched_status_type status = SCHED_S_OK;

	SCHED_MSG_HIGH("Set_clnt_param index = %u, value = %p",
		       param_index, (void *)p_param_value);

	switch (param_index) {
	case SCHED_I_CLNT_CURRQLEN:
	case SCHED_I_CLNT_OTKNCURRENT:
		{
			status = SCHED_S_EINVALOP;
			break;
		}

	case SCHED_I_CLNT_PTKNRATE:
		{
			status =
			    sched_process_set_p_tkn_rate(p_sched_ctx,
							 p_clnt_ctx,
							 p_param_value);
			break;
		}

	case SCHED_I_CLNT_PTKNPERFRM:
		{

			p_clnt_ctx->n_p_tkn_per_frm = p_param_value->un_value;
			sched_tkn_bkt_config(p_clnt_ctx);
			break;
		}

	case SCHED_I_CLNT_FRAMERATE:
		{
			p_clnt_ctx->frm_rate = p_param_value->frm_rate;
			break;
		}

	case SCHED_I_CLNT_OTKNMAX:
		{
			if (p_param_value->un_value <
			    p_clnt_ctx->n_o_tkn_per_ip_frm) {
				status = SCHED_S_EBADPARM;
			} else {
				p_clnt_ctx->n_o_tkn_max =
				    p_param_value->un_value;

				p_clnt_ctx->n_curr_o_tkns =
				    SCHED_MIN(p_clnt_ctx->n_curr_o_tkns,
					      p_clnt_ctx->n_o_tkn_max);
			}
			break;
		}

	case SCHED_I_CLNT_OTKNPERIPFRM:
		{
			if (p_param_value->un_value > p_clnt_ctx->n_o_tkn_max) {
				status = SCHED_S_EBADPARM;
			} else {
				p_clnt_ctx->n_o_tkn_per_ip_frm =
				    p_param_value->un_value;
			}
			break;
		}

	default:
		{
			status = SCHED_S_EBADPARM;
			break;
		}
	}

	return status;

}

enum sched_status_type sched_process_suspend_resume_clnt(
		struct sched_ctx_type *p_sched_ctx,
	struct _sched_clnt_list_node_type *p_clnt_node, u32 b_state) {
	u32 n_curr_time;
	struct sched_clnt_ctx_type *p_clnt_ctx = &p_clnt_node->data;

	SCHED_MSG_HIGH("Current client sched_state=%d. Requested state=%d",
		       p_clnt_ctx->b_sched_state, b_state);

	if (p_clnt_ctx->b_sched_state == b_state)
		return SCHED_S_OK;


	p_clnt_ctx->b_sched_state = b_state;

	if (!SCHED_SUCCEEDED(SCHED_GET_CURRENT_TIME(&n_curr_time))) {
		SCHED_MSG_ERR("Get current time failed");

		return SCHED_S_OK;
	}

	/* RESUME */
	if (b_state) {
		p_clnt_ctx->n_bkt_lst_sup_time = n_curr_time;
	} else {		/* SUSPEND */
	/*As we are suspending the client we fill the token bucket upto
	current time instance.*/
		sched_tkn_bkt_supply(p_clnt_ctx, n_curr_time);
	}

	SCHED_MSG_MED("Client category %d", p_clnt_ctx->client_ctgy);
	SCHED_MSG_MED("Client allocated P token rate (per sec) = %d",
		      p_clnt_ctx->n_curr_p_tkn_rate);
	SCHED_MSG_MED("Client frame rate = %d / %d",
		      p_clnt_ctx->frm_rate.n_numer,
		      p_clnt_ctx->frm_rate.n_denom);
	SCHED_MSG_MED("Client P tokens per frame = %d",
		      p_clnt_ctx->n_p_tkn_per_frm);

	return SCHED_S_OK;

}

void sched_remove_node_from_list(
	struct _sched_clnt_list_node_type **pp_head,
	struct _sched_clnt_list_node_type *p_node)
{
	u32 b_found = FALSE;
	struct _sched_clnt_list_node_type *p_curr = *pp_head;

	if (!*pp_head || !p_node) {
		SCHED_MSG_ERR("Bad params. p_head %p, p_node %p", *pp_head,
			      p_node);
		return;
	}

	if (p_node == *pp_head) {
		*pp_head = p_node->p_next;
		return;
	}

	while (!b_found && p_curr) {
		if (p_node == p_curr->p_next) {
			p_curr->p_next = p_node->p_next;
			b_found = TRUE;
		}

		p_curr = p_curr->p_next;
	}

}

SCHED_INLINE u32 sched_consider_clnt_for_sched(
		struct sched_clnt_ctx_type *p_clnt_ctx)
{
	if (p_clnt_ctx->b_first_frm &&
	    p_clnt_ctx->b_sched_state &&
	    !p_clnt_ctx->b_flushing &&
	    p_clnt_ctx->n_q_len &&
	    p_clnt_ctx->n_curr_o_tkns >= p_clnt_ctx->n_o_tkn_per_ip_frm) {
		return TRUE;
	} else {
		return FALSE;
	}
}
