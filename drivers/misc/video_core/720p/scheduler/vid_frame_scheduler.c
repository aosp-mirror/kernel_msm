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


SCHED_INLINE u32 SCHED_SUCCEEDED(enum sched_status status)
{
	SCHED_MSG_LOW("SCHED_SUCCEEDED check: status = %d", status);

	if (status == SCHED_S_OK)
		return true;
	else
		return false;

}

SCHED_INLINE u32 SCHED_FAILED(enum sched_status status)
{
	SCHED_MSG_LOW("SCHED_FAILED check: status = %d", status);

	if (status >= SCHED_S_EFAIL)
		return true;
	else
		return false;

}

static void sched_clear_clnt_ctx(struct sched_clnt_ctx *ctx)
{
	if (ctx->clnt_frm_q)
		SCHED_FREE(ctx->clnt_frm_q);
	(void)SCHED_CRITSEC_RELEASE(ctx->clnt_cs);
}

SCHED_INLINE void sched_free_clnt_node(
		struct _sched_clnt_list_node *clnt_node)
{
	sched_clear_clnt_ctx(&clnt_node->data);
	SCHED_FREE(clnt_node);

}

enum sched_status sched_clear_clnt_list(
	struct _sched_clnt_list_node *clnt_lst) {
	struct _sched_clnt_list_node *clnt_node;

	while (clnt_lst) {
		(void)SCHED_CRITSEC_ENTER(clnt_lst->data.clnt_cs);
		clnt_node = clnt_lst;
		clnt_lst = clnt_lst->next;
		sched_free_clnt_node(clnt_node);
	}
	return SCHED_S_OK;
}

static SCHED_INLINE enum sched_status sched_alloc_frm_q(
		struct sched_clnt_ctx *ctx)
{
	ctx->clnt_frm_q = (struct sched_clnt_q_elem *)
		SCHED_MALLOC(sizeof(struct sched_clnt_q_elem) *
			 ctx->max_queue_len);

	if (!ctx->clnt_frm_q) {
		SCHED_MSG_ERR("Could not allocate clnt frm Q. Out of memory");
		return SCHED_S_ENOMEM;
	}

	SCHED_MEMSET(ctx->clnt_frm_q,
	0, sizeof(struct sched_clnt_q_elem) * ctx->max_queue_len);
	ctx->q_head = 0;
	ctx->q_tail = -1;
	ctx->q_len = 0;
	SCHED_MSG_MED("Clnt frm Q allocted & initialized");
	return SCHED_S_OK;

}

static SCHED_INLINE void sched_de_q_head_frm
	(struct sched_clnt_ctx *ctx,
	 struct sched_clnt_q_elem *q_elem) {
	*q_elem = ctx->clnt_frm_q[ctx->q_head];

	memset(&ctx->clnt_frm_q[ctx->q_head], 0,
		sizeof(struct sched_clnt_q_elem));

	/*Update the circular queue head index.*/
	ctx->q_head = (ctx->q_head + 1) % ctx->max_queue_len;
	ctx->q_len--;
}

static SCHED_INLINE void sched_tkn_bkt_fill_normalize
	(struct sched_clnt_ctx *ctx)
{
	ctx->bkt_curr_tkns_nmlzd =
		(ctx->bkt_curr_tkns * SCHED_TKNBKT_FILL_NORMLZ_SCALE) /
		ctx->tkn_per_frm;
}

static void sched_tkn_bkt_config(struct sched_clnt_ctx *ctx)
{
	ctx->bkt_size = ctx->tkn_per_frm * SCHED_TKNBKT_SIZE_FACTOR;
	ctx->bkt_quies_cap = ctx->bkt_size;
	ctx->bkt_curr_tkns =
	SCHED_MIN(ctx->bkt_curr_tkns, ctx->bkt_size);
}

static void sched_tkn_bkt_supply(
	struct sched_clnt_ctx *ctx, u32 curr_time)
{
	u32 delta;
	u32 num_tkns;

	/*Check if there's time wrap-around since last token supply time.*/
	if (curr_time < ctx->bkt_lst_sup_time) {
		SCHED_MSG_HIGH("Current time wrap around detected");
		delta =
		SCHED_TIME_MAX - ctx->bkt_lst_sup_time + curr_time;
	} else
		delta = curr_time - ctx->bkt_lst_sup_time;

	/*Proceed only if there is any time elapsed since our last supply
	time.*/
	if (delta > 0) {
		/*Calculate the number of tokens that we can supply based on
		time elapsed and the client's token supply rate.*/
		num_tkns = delta * ctx->curr_p_tkn_rate / 1000;

		if (num_tkns > 0) {
			ctx->bkt_curr_tkns = SCHED_MIN(ctx->bkt_size,
					ctx->bkt_curr_tkns + num_tkns);

			if ((delta * ctx->curr_p_tkn_rate % 1000)) {
				delta = (num_tkns * 1000 +
						(ctx->curr_p_tkn_rate >> 1))
						/ ctx->curr_p_tkn_rate;
				if ((SCHED_TIME_MAX -
					ctx->bkt_lst_sup_time) < delta) {
					SCHED_MSG_HIGH
					("Handling for current time wrap "
					"around");

					ctx->bkt_lst_sup_time = delta -
					(SCHED_TIME_MAX -
					ctx->bkt_lst_sup_time);
				} else
					ctx->bkt_lst_sup_time += delta;
			} else
				ctx->bkt_lst_sup_time = curr_time;

			if (ctx->bkt_curr_tkns >
				(s32) ctx->bkt_quies_cap) {
				SCHED_MSG_HIGH
				("Client Quiesence detected. Capping "
				"bkt_curr_tkns");
				ctx->bkt_curr_tkns = ctx->tkn_per_frm;
			}
			sched_tkn_bkt_fill_normalize(ctx);
		}
	}
}

static SCHED_INLINE void sched_tkn_bkt_consume(
	struct sched_clnt_ctx *ctx) {
	ctx->bkt_curr_tkns -= ctx->tkn_per_frm;
}

static SCHED_INLINE u32 sched_clnt_frm_is_cnfmnt
	(struct sched_clnt_ctx *ctx)
{
	if (ctx->bkt_curr_tkns >= (s32) ctx->tkn_per_frm)
		return true;
	else
		return false;
}				/* end of sched_clnt_frm_is_conformant */

static struct sched_clnt_ctx *sched_elect_cnfmnt
	(struct sched_clnt_ctx *prov_elect,
	struct sched_clnt_ctx *new_cand) {

	/*If there is no provisional elect client then the new candidate
	becomes the first one.*/
	if (!prov_elect)
		return new_cand;


	/*Here we want to pick the client who has accumulated the most tokens
	from the time of attaining single frame conformance.
	Since we are comparing between clients we use the available normalized
	token bucket occupancy value.*/
	if (prov_elect->bkt_curr_tkns_nmlzd >=
	    new_cand->bkt_curr_tkns_nmlzd) {
		return prov_elect;
	} else {
	/*We had held on to this provisional elect conformant
	client critical section. Since new candidate has won the
	election leave critical section of earlier provisional
	elect.
	*/
		(void)SCHED_CRITSEC_LEAVE(prov_elect->clnt_cs);
		return new_cand;
	}
}

static struct sched_clnt_ctx *sched_elect_non_cnfmnt
	(struct sched_clnt_ctx *prov_elect,
	struct sched_clnt_ctx *new_cand) {

	/*If there is no provisional elect client then the new candidate
	becomes the first one.*/
	if (!prov_elect)
		return new_cand;
	/*Here we want to pick the client who is closest to attaining a single
	frame conformance.
	Since we are comparing between clients we use the available
	normalized token bucket occupancy value.
	Also if the provisional elect or the new contender (in that order)
	have an end of frame marker set we give it priority over deciding
	by frame conformance method mentiond earlier.*/
	if (prov_elect->eof_marker > 0) {
		return prov_elect;
	} else if (new_cand->eof_marker > 0) {
		/*We had held on to this provisional elect non conformant client
		critical section. Since new candidate has won the election
		leave critical section of earlier provisional elect.
		*/
		(void)SCHED_CRITSEC_LEAVE(prov_elect->clnt_cs);

		return new_cand;
	} else if (prov_elect->bkt_curr_tkns_nmlzd >=
		   new_cand->bkt_curr_tkns_nmlzd) {
		return prov_elect;
	} else {
	/*Similar to above case leave critical section of earlier
	provisional elect.*/
		(void)SCHED_CRITSEC_LEAVE(prov_elect->clnt_cs);
		return new_cand;
	}

}

static struct sched_clnt_ctx *sched_elect_non_rt
	(struct sched_ctx *sched_ctx) {
	struct _sched_clnt_list_node *node = NULL;
	struct _sched_clnt_list_node *start_node = NULL;
	u32 found = false;

	/*For non real time clients we are using a round robin election
	algorithm.
	Based on the last scheduled client we find the next to schedule
	and return its context.
	We also need to skip the client if certain conditions (mentioned below)
	are not met*/
	if (!sched_ctx->non_rt_last_sched)
		start_node = node = sched_ctx->non_rt_head;
	else {
		if (!sched_ctx->non_rt_last_sched->next)
			start_node = sched_ctx->non_rt_head;
		else
			start_node = sched_ctx->non_rt_last_sched->next;

		node = start_node;
	}

	do {

		(void)SCHED_CRITSEC_ENTER(node->data.clnt_cs);

	/*Check if the client can be considered for this round of scheduling.*/
		if (sched_consider_clnt_for_sched(&node->data)) {
			found = true;
			sched_ctx->non_rt_last_sched = node;
		}

	/*If this client is not the election winner then leave its critical
	section.
	If we have found a winner we want to hold on to its critical
	section. We would leave its critical section after we are done
	with dequeueing a frame from the client context.*/
		if (!found)
			(void)SCHED_CRITSEC_LEAVE(node->data.clnt_cs);

		if (!node->next)
			node = sched_ctx->non_rt_head;
		else
			node = node->next;

	} while (node != start_node);

	if (found) {
		SCHED_MSG_LOW("Non real time client selected");

		return &sched_ctx->non_rt_last_sched->data;
	} else {
		SCHED_MSG_MED
		    ("No non-real time client available for scheduling");

		return NULL;
	}

}

static enum sched_status sched_process_set_p_tkn_rate(
		struct sched_ctx *sched_ctx,
	struct sched_clnt_ctx *clnt_ctx,
	union sched_value_type *param_value) {
	u32 curr_time = 0;

	if (param_value->un_value == clnt_ctx->curr_p_tkn_rate)
		return SCHED_S_OK;


	if ((sched_ctx->total_clnt_bw - clnt_ctx->curr_p_tkn_rate +
	     param_value->un_value) > sched_ctx->perf_lvl) {
		SCHED_MSG_HIGH
		    ("Perf level insufficient for requested P Tkn rate");

	}

	/*Get current time. We need this for token supply.
	If we didn't get a valid current time value just return*/
	if (SCHED_FAILED(SCHED_GET_CURRENT_TIME(&curr_time))) {
		SCHED_MSG_ERR("Get current time failed");

		return SCHED_S_EFAIL;
	}

	/*Before we go ahead and update the Current tkn rate, we fill
	the token bucket upto current time instance.*/
	sched_tkn_bkt_supply(clnt_ctx, curr_time);

	/*Next, update the current value of total client bandwidth with
	the new tkn rate of the client.*/
	sched_ctx->total_clnt_bw = sched_ctx->total_clnt_bw -
	    clnt_ctx->curr_p_tkn_rate + param_value->un_value;
	clnt_ctx->curr_p_tkn_rate = param_value->un_value;

	/*Since the current Ptkn rate (i.e. current alloted bandwidth)
	of the client has changed we need to update client's token
	bucket configuration*/
	sched_tkn_bkt_config(clnt_ctx);
	return SCHED_S_OK;
}

static enum sched_status sched_process_add_rt_clnt(
		struct sched_ctx *sched_ctx,
	struct _sched_clnt_list_node *clnt_node) {
	enum sched_status status;
	struct sched_clnt_ctx *clnt_ctx = &clnt_node->data;
	struct _sched_clnt_list_node *tmp_node;

	/*Validate real time client specific parameters.*/
	if (!clnt_ctx->curr_p_tkn_rate)
		SCHED_MSG_HIGH("Allocated token rate is zero");

	/*Check if our performance level setting can sustain the new client*/
	if (sched_ctx->total_clnt_bw + clnt_ctx->curr_p_tkn_rate >
	    sched_ctx->perf_lvl) {
		SCHED_MSG_HIGH("Not enough bandwidth to support client");
		SCHED_MSG_HIGH
		    ("curr_perflvl=%d, curr_bw=%d, newclnt_ptknrate=%d",
		     sched_ctx->perf_lvl, sched_ctx->total_clnt_bw,
		     clnt_ctx->curr_p_tkn_rate);

	}
	/*Allocate the client frame queue*/
	status = sched_alloc_frm_q(clnt_ctx);

	if (SCHED_SUCCEEDED(status)) {
		/*Allocate the token bucket*/
		sched_tkn_bkt_config(clnt_ctx);
		/*We start with empty token bucket*/
		clnt_ctx->bkt_curr_tkns = 0;
		clnt_ctx->bkt_curr_tkns_nmlzd = 0;
		/*Add the client to the real time client list and increase the
		total client bandwidth.*/
		tmp_node = sched_ctx->rt_head;
		sched_ctx->rt_head = clnt_node;
		sched_ctx->rt_head->next = tmp_node;
		sched_ctx->rt_clnts++;
		sched_ctx->total_clnt_bw += clnt_ctx->curr_p_tkn_rate;
	}
	return status;
}

static enum sched_status sched_process_add_non_rt_clnt(
		struct sched_ctx *sched_ctx,
	struct _sched_clnt_list_node *clnt_node) {
	enum sched_status status;
	struct sched_clnt_ctx *clnt_ctx = &clnt_node->data;
	struct _sched_clnt_list_node *tmp_node;

	/*Allocate the client frame queue*/
	status = sched_alloc_frm_q(clnt_ctx);
	if (SCHED_SUCCEEDED(status)) {
		/*Add the client to the real time client list and increase the
		total client bandwidth.*/
		tmp_node = sched_ctx->non_rt_head;
		sched_ctx->non_rt_head = clnt_node;
		sched_ctx->non_rt_head->next = tmp_node;
		sched_ctx->non_rt_clnts++;
	}
	return status;
}

enum sched_status sched_process_add_clnt(
	struct sched_ctx *sched_ctx,
	struct _sched_clnt_list_node *clnt_node,
	struct sched_client_init_param *init_param) {
	enum sched_status status = SCHED_S_OK;

	SCHED_MEMSET(clnt_node, 0, sizeof(struct _sched_clnt_list_node));

	/*Validate all initialization parameters*/
	if (!init_param->tkn_per_frm ||
	    !init_param->frm_rate.numer ||
	    !init_param->frm_rate.denom ||
	    !init_param->max_queue_len ||
	    !init_param->o_tkn_max ||
	    !init_param->o_tkn_per_ip_frm ||
	    init_param->o_tkn_init > init_param->o_tkn_max ||
	    init_param->o_tkn_per_ip_frm > init_param->o_tkn_max) {
		SCHED_MSG_ERR("Bad initialization parameters");
		return SCHED_S_EBADPARM;
	}

	/*Store all initialization parameters*/
	clnt_node->data.client_ctgy = init_param->client_ctgy;
	clnt_node->data.curr_p_tkn_rate = init_param->alloc_p_tkn_rate;
	clnt_node->data.frm_rate = init_param->frm_rate;
	clnt_node->data.max_queue_len = init_param->max_queue_len;
	clnt_node->data.o_tkn_max = init_param->o_tkn_max;
	clnt_node->data.o_tkn_per_ip_frm = init_param->o_tkn_per_ip_frm;
	clnt_node->data.curr_o_tkns = init_param->o_tkn_init;
	clnt_node->data.tkn_per_frm = init_param->tkn_per_frm;
	clnt_node->data.client_data = init_param->client_data;
	clnt_node->data.sched_state = true;

	SCHED_MSG_HIGH("Adding new client of category %d",
		       clnt_node->data.client_ctgy);
	SCHED_MSG_MED("Allocated P token rate (per sec) = %d",
		      clnt_node->data.curr_p_tkn_rate);
	SCHED_MSG_MED("Frame rate = %d / %d",
		      clnt_node->data.frm_rate.numer,
		      clnt_node->data.frm_rate.denom);
	SCHED_MSG_MED("Max_queue_len = %d", clnt_node->data.max_queue_len);
	SCHED_MSG_MED("Max O tokens = %d", clnt_node->data.o_tkn_max);
	SCHED_MSG_MED("O tokens threshold = %d",
		      clnt_node->data.o_tkn_per_ip_frm);
	SCHED_MSG_MED("P tokens per frame = %d",
		      clnt_node->data.tkn_per_frm);
	SCHED_MSG_MED("Client data ptr = %p", clnt_node->data.client_data);

	if (SCHED_FAILED(SCHED_CRITSEC_CREATE(&clnt_node->data.clnt_cs)))
		return SCHED_S_EFAIL;

	/*Configure the client context based on client category.*/
	switch (clnt_node->data.client_ctgy) {
	case SCHED_CLNT_RT_BUFF:
	case SCHED_CLNT_RT_NOBUFF:
		{
			status =
			    sched_process_add_rt_clnt(sched_ctx, clnt_node);
			break;
		}

	case SCHED_CLNT_NONRT:
		{
			status =
			    sched_process_add_non_rt_clnt(sched_ctx,
							  clnt_node);
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

enum sched_status sched_process_remove_clnt(
		struct sched_ctx *sched_ctx,
	struct _sched_clnt_list_node *clnt_node) {

	(void)SCHED_CRITSEC_ENTER(clnt_node->data.clnt_cs);

	/*Handling if the client frame queue is not empty. Just return
	and let Codec driver dequeue all frames for this client
	before calling remove client*/
	if (clnt_node->data.q_len) {
		SCHED_MSG_ERR("Cannot remove client. Queue is not empty");
		return SCHED_S_EINVALST;
	}

	/*Based on client category, remove the client node from the
	appropriate scheduler client list*/
	switch (clnt_node->data.client_ctgy) {
	case SCHED_CLNT_RT_BUFF:
	case SCHED_CLNT_RT_NOBUFF:
		{

			sched_remove_node_from_list(&sched_ctx->rt_head,
						    clnt_node);
			sched_ctx->rt_clnts--;
			sched_ctx->total_clnt_bw -=
			    clnt_node->data.curr_p_tkn_rate;
			break;
		}

	case SCHED_CLNT_NONRT:
		{
			sched_remove_node_from_list(&sched_ctx->non_rt_head,
						    clnt_node);
			sched_ctx->non_rt_clnts--;
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
		       clnt_node->data.client_ctgy);
	SCHED_MSG_MED("Allocated P token rate (per sec) = %d",
		      clnt_node->data.curr_p_tkn_rate);
	SCHED_MSG_MED("Frame rate = %d / %d",
		      clnt_node->data.frm_rate.numer,
		      clnt_node->data.frm_rate.denom);
	SCHED_MSG_MED("Max_queue_len = %d", clnt_node->data.max_queue_len);
	SCHED_MSG_MED("Max O tokens = %d", clnt_node->data.o_tkn_max);
	SCHED_MSG_MED("P tokens per frame = %d",
		      clnt_node->data.tkn_per_frm);
	SCHED_MSG_MED("Client data ptr = %p", clnt_node->data.client_data);
	sched_free_clnt_node(clnt_node);
	return SCHED_S_OK;
}

enum sched_status sched_process_flush_clnt_buff(
		struct sched_ctx *sched_ctx,
	struct _sched_clnt_list_node *clnt_node, void **pp_frm_data) {
	struct sched_clnt_ctx *clnt_ctx;
	enum sched_status status = SCHED_S_OK;
	struct sched_clnt_q_elem q_elem;

	clnt_ctx = &clnt_node->data;

	/*If the client queue is empty just return an QEMPTY status*/
	if (!clnt_ctx->q_len) {
		status = SCHED_S_QEMPTY;
	} else {
		clnt_ctx->flushing = true;

	/*If the client queue is not empty just remove and return the
	element at the front of the queue.*/
		sched_de_q_head_frm(clnt_ctx, &q_elem);
		*pp_frm_data = q_elem.frm_data;
	}

	/*If the Queue was orginially empty OR if it got empty after latest
	De_queue we reset the flushing and First_frame flags.
	Token bucket contents are also emptied.Queue pointers are reset.
	o_tkns are restored.*/
	if (!clnt_ctx->q_len) {
		clnt_ctx->flushing = false;
		clnt_ctx->first_frm = false;
		clnt_ctx->bkt_curr_tkns = 0;
		clnt_ctx->bkt_curr_tkns_nmlzd = 0;
		clnt_ctx->bkt_lst_sup_time = 0;
		clnt_ctx->q_head = 0;
		clnt_ctx->q_tail = -1;
		SCHED_MSG_HIGH
		    ("Client flushed and re-initialized. Client category %d",
		     clnt_ctx->client_ctgy);
		SCHED_MSG_MED("Client allocated P token rate (per sec) = %d",
			      clnt_ctx->curr_p_tkn_rate);
		SCHED_MSG_MED("Client frame rate = %d / %d",
			      clnt_ctx->frm_rate.numer,
			      clnt_ctx->frm_rate.denom);
		SCHED_MSG_MED("Client P tokens per frame = %d",
			      clnt_ctx->tkn_per_frm);
	}
	return status;
}

SCHED_INLINE enum sched_status sched_process_mark_clnt_eof(
		struct sched_ctx *sched_ctx,
	struct _sched_clnt_list_node *clnt_node) {

	if (!clnt_node->data.q_len)
		return SCHED_S_QEMPTY;


	if (!clnt_node->data.clnt_frm_q[clnt_node->data.q_tail].eof) {
		/*Just increment the EOF marker count in the client context.*/
		clnt_node->data.eof_marker++;
		clnt_node->data.clnt_frm_q[clnt_node->data.q_tail].
		eof = true;
	} else
		SCHED_MSG_HIGH("Current frame is already marked EOF");

	SCHED_MSG_HIGH("Client marked for end of frames. Client category %d",
		       clnt_node->data.client_ctgy);
	SCHED_MSG_MED("Client allocated P token rate (per sec) = %d",
		      clnt_node->data.curr_p_tkn_rate);
	SCHED_MSG_MED("Client frame rate = %d / %d",
		      clnt_node->data.frm_rate.numer,
		      clnt_node->data.frm_rate.denom);
	SCHED_MSG_MED("Client P tokens per frame = %d",
		      clnt_node->data.tkn_per_frm);
	return SCHED_S_OK;
}

enum sched_status sched_process_update_clnt_o_tkn(
		struct sched_ctx *sched_ctx,
	struct _sched_clnt_list_node *clnt_node,
	u32 type, u32 o_tkn) {

	/*Act based on the type of update.*/

	if (type) {
		/*Just replenish the output tokens the client currently has with
		the provided number while not going over the max value.*/
		clnt_node->data.curr_o_tkns =
		    SCHED_MIN(clnt_node->data.curr_o_tkns + o_tkn,
			      clnt_node->data.o_tkn_max);
	} else {
	/*Just subtract the give number of output tokens from the count
	the client currently has while not going less than 0.*/
		if (o_tkn >= clnt_node->data.curr_o_tkns)
			clnt_node->data.curr_o_tkns = 0;
		else
			clnt_node->data.curr_o_tkns -= o_tkn;

	}

	SCHED_MSG_LOW("%d O tokens restored for client", o_tkn);
	SCHED_MSG_LOW("Client Curr_o_tkns = %d",
		      clnt_node->data.curr_o_tkns);
	SCHED_MSG_LOW("Client category = %d", clnt_node->data.client_ctgy);
	SCHED_MSG_LOW("Client allocated P token rate (per sec) = %d",
		      clnt_node->data.curr_p_tkn_rate);
	SCHED_MSG_LOW("Client frame rate = %d / %d",
		      clnt_node->data.frm_rate.numer,
		      clnt_node->data.frm_rate.denom);
	SCHED_MSG_LOW("Client P tokens per frame = %d",
		      clnt_node->data.tkn_per_frm);
	return SCHED_S_OK;
}

enum sched_status sched_process_en_q_frm(
		struct sched_ctx *sched_ctx,
	struct _sched_clnt_list_node *clnt_node, void *frm_data) {
	struct sched_clnt_ctx *clnt_ctx;
	u32 curr_time = 0;

	clnt_ctx = &clnt_node->data;

	/*Check if the client queue is full already*/
	if (clnt_ctx->q_len == clnt_ctx->max_queue_len) {
		SCHED_MSG_HIGH("Cannot enqueue. Client queue is full");

		return SCHED_S_QFULL;
	}

	/*Check if the client queue is being flushed.*/
	if (clnt_ctx->flushing) {
		SCHED_MSG_ERR("Cannot enqueue. Client queue is being flushed");

		return SCHED_S_EINVALST;
	}

	/*Reposition tail, increase Q length and add the frame data to Q*/
	clnt_ctx->q_tail =
	    (clnt_ctx->q_tail + 1) % clnt_ctx->max_queue_len;

	clnt_ctx->q_len++;

	clnt_ctx->clnt_frm_q[clnt_ctx->q_tail].frm_data = frm_data;
	clnt_ctx->clnt_frm_q[clnt_ctx->q_tail].eof = false;

	/*If this is the first frame being queued for this client then,
	get current time. We now start the token supply clock for the client.
	Supply tokens required for a single frame processing while storing
	the current time as the last supply time and marking that first
	frame is received.*/
	if (!clnt_ctx->first_frm) {
		SCHED_MSG_HIGH("Client first frame enqueued");
		if (clnt_ctx->client_ctgy != SCHED_CLNT_NONRT) {
			if (SCHED_SUCCEEDED
			(SCHED_GET_CURRENT_TIME(&curr_time))) {
				clnt_ctx->bkt_curr_tkns =
					clnt_ctx->tkn_per_frm;
				clnt_ctx->bkt_lst_sup_time = curr_time;
				clnt_ctx->first_frm = true;
			}
		} else
			clnt_ctx->first_frm = true;
	}

	SCHED_MSG_LOW("Client frame enqueued. Queue fill status = %d / %d",
			clnt_ctx->q_len, clnt_ctx->max_queue_len);
	SCHED_MSG_LOW("Client category = %d", clnt_ctx->client_ctgy);
	SCHED_MSG_LOW("Client allocated P token rate (per sec) = %d",
		clnt_ctx->curr_p_tkn_rate);
	SCHED_MSG_LOW("Client frame rate = %d / %d",
		clnt_ctx->frm_rate.numer,
		clnt_ctx->frm_rate.denom);
	SCHED_MSG_LOW("Client P tokens per frame = %d",
		clnt_ctx->tkn_per_frm);

	return SCHED_S_OK;

}

enum sched_status sched_process_re_en_q_frm(
	struct sched_ctx *sched_ctx,
	struct _sched_clnt_list_node *clnt_node,
	void *frm_data)
{
	struct sched_clnt_ctx *clnt_ctx;
	u32 curr_time = 0;

	clnt_ctx = &clnt_node->data;

	if (clnt_ctx->q_len == clnt_ctx->max_queue_len) {
		SCHED_MSG_ERR("Cannot re-enqueue. Client queue is full");
		return SCHED_S_QFULL;
	}

	if (clnt_ctx->flushing) {
		SCHED_MSG_ERR("Cannot re-enqueue. Client"
					" queue is being flushed");
		return SCHED_S_EINVALST;
	}

	clnt_ctx->q_head =
		(clnt_ctx->q_head + clnt_ctx->max_queue_len - 1) %
		clnt_ctx->max_queue_len;

	clnt_ctx->q_len++;

	clnt_ctx->clnt_frm_q[clnt_ctx->q_head].frm_data =
		frm_data;
	clnt_ctx->clnt_frm_q[clnt_ctx->q_head].eof =
		false;

	if (clnt_ctx->client_ctgy != SCHED_CLNT_NONRT) {
		if (!clnt_ctx->first_frm) {
			SCHED_MSG_HIGH("Client frame "
						"re-enqueued as first frame");
			if (SCHED_SUCCEEDED
			(SCHED_GET_CURRENT_TIME(&curr_time))) {
				clnt_ctx->bkt_curr_tkns =
					clnt_ctx->tkn_per_frm;
				clnt_ctx->bkt_lst_sup_time =
					curr_time;
				clnt_ctx->first_frm =
					true;
			}
		} else
			clnt_ctx->bkt_curr_tkns +=
				clnt_ctx->tkn_per_frm;
	} else
		clnt_ctx->first_frm = true;


	SCHED_MSG_LOW("Client frame re-enqueued. Queue fill status = %d / %d",
	clnt_ctx->q_len, clnt_ctx->max_queue_len);
	SCHED_MSG_LOW("Client category = %d", clnt_ctx->client_ctgy);
	SCHED_MSG_LOW("Client allocated P token rate (per sec) = %d",
	clnt_ctx->curr_p_tkn_rate);
	SCHED_MSG_LOW("Client frame rate = %d / %d",
			clnt_ctx->frm_rate.numer,
			clnt_ctx->frm_rate.denom);
	SCHED_MSG_LOW("Client P tokens per frame = %d",
			clnt_ctx->tkn_per_frm);

	return SCHED_S_OK;

}

enum sched_status sched_process_de_q_frm_rt_clnt(
		struct sched_ctx *sched_ctx,
	struct sched_clnt_ctx **pp_conf_elect_ctx,
	struct sched_clnt_ctx **pp_non_conf_elect_ctx) {
	u32 curr_time = 0;
	struct _sched_clnt_list_node *clnt_node;
	struct sched_clnt_ctx *clnt_ctx;

	*pp_conf_elect_ctx = NULL;
	*pp_non_conf_elect_ctx = NULL;

	/*Get current time. We need this for token supply.
	If we didn't get a valid current time value just return*/
	if (SCHED_FAILED(SCHED_GET_CURRENT_TIME(&curr_time))) {
		SCHED_MSG_ERR("Get current time failed");

		return SCHED_S_EFAIL;
	}

	/*Run through the list of real time clients.
	Consider only the clients that have queued atleast one frame since
	being admitted into the scheduler.
	Supply tokens equivalent to elapsed time since last supply.
	Also in this same pass, check if each client has a conformant
	frame or not.*/
	clnt_node = sched_ctx->rt_head;
	while (clnt_node) {
		clnt_ctx = &clnt_node->data;

		(void)SCHED_CRITSEC_ENTER(clnt_ctx->clnt_cs);

		if (sched_consider_clnt_for_sched(clnt_ctx)) {
			sched_tkn_bkt_supply(clnt_ctx, curr_time);
			if (sched_clnt_frm_is_cnfmnt(clnt_ctx)) {
				*pp_conf_elect_ctx =
					sched_elect_cnfmnt(*pp_conf_elect_ctx,
						clnt_ctx);
			} else {
				if (!*pp_conf_elect_ctx) {
					*pp_non_conf_elect_ctx =
					    sched_elect_non_cnfmnt
					    (*pp_non_conf_elect_ctx,
					     clnt_ctx);
				} else if (*pp_non_conf_elect_ctx) {
					(void)
					    SCHED_CRITSEC_LEAVE(
					    (*pp_non_conf_elect_ctx)->clnt_cs);
					*pp_non_conf_elect_ctx = NULL;

				}
			}
		}
		if (clnt_ctx != *pp_conf_elect_ctx
		    && clnt_ctx != *pp_non_conf_elect_ctx)
			(void)SCHED_CRITSEC_LEAVE(clnt_ctx->clnt_cs);
		clnt_node = clnt_node->next;
	}

	return SCHED_S_OK;

}

enum sched_status sched_process_de_q_frm(
		struct sched_ctx *sched_ctx,
	void **pp_frm_data, void **pp_client_data) {
	enum sched_status status;
	struct sched_clnt_ctx *sched_clnt_ctx = NULL;
	struct sched_clnt_ctx *conf_elect_ctx;
	struct sched_clnt_ctx *non_conf_elect_ctx;
	struct sched_clnt_q_elem q_elem;

	status = sched_process_de_q_frm_rt_clnt(sched_ctx,
						&conf_elect_ctx,
						&non_conf_elect_ctx);
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
	if (conf_elect_ctx) {
		SCHED_MSG_LOW("Conformant frame client selected");
		sched_tkn_bkt_consume(conf_elect_ctx);
		sched_clnt_ctx = conf_elect_ctx;
	} else if (non_conf_elect_ctx) {
		SCHED_MSG_LOW("Non-Conformant frame client selected");
		sched_tkn_bkt_consume(non_conf_elect_ctx);
		sched_clnt_ctx = non_conf_elect_ctx;
	} else if (sched_ctx->non_rt_clnts)
		sched_clnt_ctx = sched_elect_non_rt(sched_ctx);

	/*If we have a client that we can schedule, then dequeue the frame
	at the head of its queue.*/
	if (sched_clnt_ctx) {
		*pp_client_data = sched_clnt_ctx->client_data;

		sched_de_q_head_frm(sched_clnt_ctx, &q_elem);

		*pp_frm_data = q_elem.frm_data;

		sched_clnt_ctx->curr_o_tkns -=
		    sched_clnt_ctx->o_tkn_per_ip_frm;

	/*If the dequeued frame was marked EOF we need to decrement the
	eof_marker count.*/
		if (q_elem.eof) {
			SCHED_MSG_MED
			    ("Last frame for EOF marked client dequeued");

			sched_clnt_ctx->eof_marker--;

			status = SCHED_S_EOF;
		}

		SCHED_MSG_LOW
		    ("Client frame Dequeued. Queue fill status = %d / %d",
		     sched_clnt_ctx->q_len,
		     sched_clnt_ctx->max_queue_len);
		SCHED_MSG_LOW("Client category = %d",
			      sched_clnt_ctx->client_ctgy);
		SCHED_MSG_LOW("Client allocated P token rate (per sec) = %d",
			      sched_clnt_ctx->curr_p_tkn_rate);
		SCHED_MSG_LOW("Client frame rate = %d / %d",
			      sched_clnt_ctx->frm_rate.numer,
			      sched_clnt_ctx->frm_rate.denom);
		SCHED_MSG_LOW("Client P tokens per frame = %d",
			      sched_clnt_ctx->tkn_per_frm);

	/*We had held on to the election winning client critical
	section. Leave client critical section before we exit.*/
		(void)SCHED_CRITSEC_LEAVE(sched_clnt_ctx->clnt_cs);
	} else {
		status = SCHED_S_QEMPTY;
	}

	return status;

}

enum sched_status sched_process_sched_lvl_get_param(
		struct sched_ctx *sched_ctx,
	enum sched_index param_index,
	union sched_value_type *param_value)
{
	enum sched_status status = SCHED_S_OK;

	switch (param_index) {
	case SCHED_I_PERFLEVEL:
		{
			param_value->un_value = sched_ctx->perf_lvl;
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

enum sched_status sched_process_sched_lvl_set_param(
		struct sched_ctx *sched_ctx,
	enum sched_index param_index,
	union sched_value_type *param_value)
{
	enum sched_status status = SCHED_S_OK;

	SCHED_MSG_HIGH("Set_sched_param index = %u, value = %p",
		       param_index, (void *)param_value);

	switch (param_index) {
	case SCHED_I_PERFLEVEL:
		{
			if (sched_ctx->total_clnt_bw >
			    param_value->un_value) {
				SCHED_MSG_HIGH
				    ("Perf level being lowered than current "
				     "bandwidth");
				SCHED_MSG_HIGH
				    ("curr_perflvl=%d, new_perflvl=%d, "
				     "curr_bw=%d",
				     sched_ctx->perf_lvl,
				     param_value->un_value,
				     sched_ctx->total_clnt_bw);
			}

			sched_ctx->perf_lvl = param_value->un_value;

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

enum sched_status sched_process_clnt_lvl_get_param(
		struct sched_ctx *sched_ctx,
	struct sched_clnt_ctx *clnt_ctx,
	enum sched_index param_index,
	union sched_value_type *param_value) {
	enum sched_status status = SCHED_S_OK;

	switch (param_index) {
	case SCHED_I_CLNT_CURRQLEN:
		{
			param_value->un_value = clnt_ctx->q_len;
			break;
		}

	case SCHED_I_CLNT_PTKNRATE:
		{
			param_value->un_value = clnt_ctx->curr_p_tkn_rate;
			break;
		}

	case SCHED_I_CLNT_PTKNPERFRM:
		{
			param_value->un_value = clnt_ctx->tkn_per_frm;
			break;
		}

	case SCHED_I_CLNT_FRAMERATE:
		{
			param_value->frm_rate = clnt_ctx->frm_rate;
			break;
		}

	case SCHED_I_CLNT_OTKNMAX:
		{
			param_value->un_value = clnt_ctx->o_tkn_max;
			break;
		}

	case SCHED_I_CLNT_OTKNPERIPFRM:
		{
			param_value->un_value =
			    clnt_ctx->o_tkn_per_ip_frm;
			break;
		}

	case SCHED_I_CLNT_OTKNCURRENT:
		{
			param_value->un_value = clnt_ctx->curr_o_tkns;
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

enum sched_status sched_process_clnt_lvl_set_param(
		struct sched_ctx *sched_ctx,
	struct sched_clnt_ctx *clnt_ctx,
	enum sched_index param_index,
	union sched_value_type *param_value)
{
	enum sched_status status = SCHED_S_OK;

	SCHED_MSG_HIGH("Set_clnt_param index = %u, value = %p",
		       param_index, (void *)param_value);

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
			    sched_process_set_p_tkn_rate(sched_ctx,
							 clnt_ctx,
							 param_value);
			break;
		}

	case SCHED_I_CLNT_PTKNPERFRM:
		{

			clnt_ctx->tkn_per_frm = param_value->un_value;
			sched_tkn_bkt_config(clnt_ctx);
			break;
		}

	case SCHED_I_CLNT_FRAMERATE:
		{
			clnt_ctx->frm_rate = param_value->frm_rate;
			break;
		}

	case SCHED_I_CLNT_OTKNMAX:
		{
			if (param_value->un_value <
			    clnt_ctx->o_tkn_per_ip_frm) {
				status = SCHED_S_EBADPARM;
			} else {
				clnt_ctx->o_tkn_max =
				    param_value->un_value;

				clnt_ctx->curr_o_tkns =
				    SCHED_MIN(clnt_ctx->curr_o_tkns,
					      clnt_ctx->o_tkn_max);
			}
			break;
		}

	case SCHED_I_CLNT_OTKNPERIPFRM:
		{
			if (param_value->un_value > clnt_ctx->o_tkn_max) {
				status = SCHED_S_EBADPARM;
			} else {
				clnt_ctx->o_tkn_per_ip_frm =
				    param_value->un_value;
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

enum sched_status sched_process_suspend_resume_clnt(
		struct sched_ctx *sched_ctx,
	struct _sched_clnt_list_node *clnt_node, u32 state) {
	u32 curr_time;
	struct sched_clnt_ctx *clnt_ctx = &clnt_node->data;

	SCHED_MSG_HIGH("Current client sched_state=%d. Requested state=%d",
		       clnt_ctx->sched_state, state);

	if (clnt_ctx->sched_state == state)
		return SCHED_S_OK;


	clnt_ctx->sched_state = state;

	if (!SCHED_SUCCEEDED(SCHED_GET_CURRENT_TIME(&curr_time))) {
		SCHED_MSG_ERR("Get current time failed");

		return SCHED_S_OK;
	}

	/* RESUME */
	if (state) {
		clnt_ctx->bkt_lst_sup_time = curr_time;
	} else {		/* SUSPEND */
	/*As we are suspending the client we fill the token bucket upto
	current time instance.*/
		sched_tkn_bkt_supply(clnt_ctx, curr_time);
	}

	SCHED_MSG_MED("Client category %d", clnt_ctx->client_ctgy);
	SCHED_MSG_MED("Client allocated P token rate (per sec) = %d",
		      clnt_ctx->curr_p_tkn_rate);
	SCHED_MSG_MED("Client frame rate = %d / %d",
		      clnt_ctx->frm_rate.numer,
		      clnt_ctx->frm_rate.denom);
	SCHED_MSG_MED("Client P tokens per frame = %d",
		      clnt_ctx->tkn_per_frm);

	return SCHED_S_OK;

}

void sched_remove_node_from_list(
	struct _sched_clnt_list_node **pp_head,
	struct _sched_clnt_list_node *node)
{
	u32 found = false;
	struct _sched_clnt_list_node *curr = *pp_head;

	if (!*pp_head || !node) {
		SCHED_MSG_ERR("Bad params. head %p, node %p", *pp_head,
			      node);
		return;
	}

	if (node == *pp_head) {
		*pp_head = node->next;
		return;
	}

	while (!found && curr) {
		if (node == curr->next) {
			curr->next = node->next;
			found = true;
		}

		curr = curr->next;
	}

}

SCHED_INLINE u32 sched_consider_clnt_for_sched(
		struct sched_clnt_ctx *clnt_ctx)
{
	if (clnt_ctx->first_frm &&
	    clnt_ctx->sched_state &&
	    !clnt_ctx->flushing &&
	    clnt_ctx->q_len &&
	    clnt_ctx->curr_o_tkns >= clnt_ctx->o_tkn_per_ip_frm) {
		return true;
	} else {
		return false;
	}
}
