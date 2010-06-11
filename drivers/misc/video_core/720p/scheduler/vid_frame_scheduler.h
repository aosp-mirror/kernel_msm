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
#ifndef _VID_FRAME_SCHEDULER_H_
#define _VID_FRAME_SCHEDULER_H_
#include "vid_frame_scheduler_utils.h"

struct sched_clnt_q_elem {
	void *frm_data;
	u32 eof;

};

struct sched_clnt_ctx {
	enum sched_client_ctgy client_ctgy;
	struct sched_client_frm_rate frm_rate;
	u32 tkn_per_frm;
	u32 curr_p_tkn_rate;
	u32 o_tkn_max;
	u32 o_tkn_per_ip_frm;
	u32 curr_o_tkns;
	u32 bkt_size;
	u32 bkt_quies_cap;
	s32 bkt_curr_tkns;
	s32 bkt_curr_tkns_nmlzd;
	u32 bkt_lst_sup_time;
	u32 max_queue_len;
	struct sched_clnt_q_elem *clnt_frm_q;
	s32 q_head;
	s32 q_tail;
	u32 q_len;
	u32 first_frm;
	u32 eof_marker;
	u32 flushing;
	u32 sched_state;
	void *client_data;
	u32 *clnt_cs;
};

struct _sched_clnt_list_node {
	struct sched_clnt_ctx data;
	struct _sched_clnt_list_node *next;

};

struct _sched_clnt_list_node;

struct sched_ctx {
	u32 perf_lvl;
	struct _sched_clnt_list_node *rt_head;
	u32 rt_clnts;
	struct _sched_clnt_list_node *non_rt_head;
	u32 non_rt_clnts;
	struct _sched_clnt_list_node *non_rt_last_sched;
	u32 total_clnt_bw;
	u32 *sched_cs;
};

SCHED_INLINE u32 SCHED_SUCCEEDED(enum sched_status status);
SCHED_INLINE u32 SCHED_FAILED(enum sched_status status);
SCHED_INLINE void sched_free_clnt_node
    (struct _sched_clnt_list_node *clnt_node);
enum sched_status sched_clear_clnt_list
    (struct _sched_clnt_list_node *clnt_lst);
enum sched_status sched_process_add_clnt
    (struct sched_ctx *sched_ctx,
     struct _sched_clnt_list_node *clnt_node,
     struct sched_client_init_param *init_param);
enum sched_status sched_process_remove_clnt
    (struct sched_ctx *sched_ctx,
     struct _sched_clnt_list_node *clnt_node);
enum sched_status sched_process_flush_clnt_buff
    (struct sched_ctx *sched_ctx,
     struct _sched_clnt_list_node *clnt_node, void **pp_frm_data);
SCHED_INLINE enum sched_status sched_process_mark_clnt_eof
    (struct sched_ctx *sched_ctx,
     struct _sched_clnt_list_node *clnt_node);
enum sched_status sched_process_update_clnt_o_tkn
    (struct sched_ctx *sched_ctx,
     struct _sched_clnt_list_node *clnt_node, u32 type, u32 o_tkn);
enum sched_status sched_process_en_q_frm
    (struct sched_ctx *sched_ctx,
     struct _sched_clnt_list_node *clnt_node, void *frm_data);
enum sched_status sched_process_re_en_q_frm
(struct sched_ctx	*sched_ctx,
  struct _sched_clnt_list_node *clnt_node, void *frm_data);
enum sched_status sched_process_de_q_frm
    (struct sched_ctx *sched_ctx,
     void **pp_frm_data, void **pp_client_data);
enum sched_status sched_process_sched_lvl_get_param
    (struct sched_ctx *sched_ctx,
     enum sched_index param_index, union sched_value_type *param_value);
enum sched_status sched_process_sched_lvl_set_param
    (struct sched_ctx *sched_ctx,
     enum sched_index param_index, union sched_value_type *param_value);
enum sched_status sched_process_clnt_lvl_get_param
    (struct sched_ctx *sched_ctx,
     struct sched_clnt_ctx *clnt_ctx,
     enum sched_index param_index, union sched_value_type *param_value);
enum sched_status sched_process_clnt_lvl_set_param
    (struct sched_ctx *sched_ctx,
     struct sched_clnt_ctx *clnt_ctx,
     enum sched_index param_index, union sched_value_type *param_value);
enum sched_status sched_process_suspend_resume_clnt
    (struct sched_ctx *sched_ctx,
     struct _sched_clnt_list_node *clnt_node, u32 state);
void sched_remove_node_from_list
    (struct _sched_clnt_list_node **pp_head,
     struct _sched_clnt_list_node *node);
SCHED_INLINE u32 sched_consider_clnt_for_sched
    (struct sched_clnt_ctx *clnt_ctx);

#endif
