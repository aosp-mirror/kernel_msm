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
#ifndef _VCD_CORE_H_
#define _VCD_CORE_H_

#include "vcd_api.h"
#include "vid_frame_scheduler_api.h"
#include "vcd_ddl_api.h"

#include "vcd_util.h"
#include "vcd_client_sm.h"
#include "vcd_power_sm.h"

#define VCD_SIGNATURE                        0x75017591U

#define VCD_MIN_PERF_LEVEL                   37900

#define VCD_MAX_SCHEDULER_QUEUE_DURATION     1

#define VCD_MAX_SCHEDULER_QUEUE_SIZE(n_fps_n, n_fps_d)          \
      (n_fps_n / n_fps_d * VCD_MAX_SCHEDULER_QUEUE_DURATION)

#define VCD_SCHEDULER_INITIAL_PERF_LEVEL        108000

#define VCD_SCHEDULER_ENC_DFLT_OTKN_PERFRM        1

#define VCD_SCHEDULER_DEC_DFLT_OTKN_PERFRM        1

#define VCD_DRIVER_INSTANCE_MAX              4

#define VCD_MAX_CLIENT_TRANSACTIONS          32

#define VCD_SEQ_HDR_PADDING_BYTES            256

#define VCD_DEC_NUM_INTERLACED_FIELDS        2

#define VCD_TIMESTAMP_RESOLUTION             1000000
#define VCD_DEC_INITIAL_FRAME_RATE           30

#define VCD_S_SCHED_STAT_BASE  0x20000000
#define VCD_S_SCHED_EOS        (VCD_S_SCHED_STAT_BASE + 0x1)
#define VCD_S_SCHED_SLEEP      (VCD_S_SCHED_STAT_BASE + 0x2)
#define VCD_S_SCHED_QEMPTY     (VCD_S_SCHED_STAT_BASE + 0x3)
#define VCD_S_SCHED_QFULL      (VCD_S_SCHED_STAT_BASE + 0x4)

enum vcd_command_type {
	VCD_CMD_NONE,
	VCD_CMD_DEVICE_INIT,
	VCD_CMD_DEVICE_TERM,
	VCD_CMD_DEVICE_RESET,
	VCD_CMD_CODEC_START,
	VCD_CMD_CODEC_STOP,
	VCD_CMD_CODE_FRAME,
	VCD_CMD_OUTPUT_FLUSH,
	VCD_CMD_CLIENT_CLOSE
};

struct vcd_cmd_q_element_type {
	enum vcd_command_type e_pending_cmd;
};

struct vcd_buffer_entry_type {
	u32 b_valid;
	u8 *p_alloc;
	u8 *p_virtual;
	u8 *p_physical;
	u32 n_size;
	u32 b_allocated;
	u32 b_in_use;
	struct vcd_frame_data_type frame;

};

struct vcd_buffer_pool_type {
	struct vcd_buffer_entry_type *a_entries;
	u32 n_count;
	struct vcd_buffer_requirement_type buf_req;
	u32 n_validated;
	u32 n_allocated;
	u32 n_in_use;
	struct vcd_buffer_entry_type **a_queue;
	u16 n_q_len;
	u16 n_q_head;
	u16 n_q_tail;

};

struct vcd_transc_type {
	u32 b_in_use;
	enum vcd_command_type e_type;
	struct vcd_clnt_ctxt_type_t *p_cctxt;

	struct vcd_buffer_entry_type *p_ip_buf_entry;

	s64 time_stamp;
	u32 n_ip_frm_tag;
	enum vcd_frame_type e_frame_type;

	struct vcd_buffer_entry_type *p_op_buf_entry;

	u32 b_input_done;
	u32 b_frame_done;
};

struct vcd_dev_ctxt_type {
	u32 b_ddl_cmd_concurrency;
	u32 n_ddl_frame_ch_depth;
	u32 n_ddl_cmd_ch_depth;
	u32 n_ddl_frame_ch_interim;
	u32 n_ddl_cmd_ch_interim;
	u32 n_ddl_frame_ch_free;
	u32 n_ddl_cmd_ch_free;

	void *sched_hdl;

	struct vcd_init_config_type config;

	u32 b_driver_ids[VCD_DRIVER_INSTANCE_MAX];
	u32 n_refs;
	u8 *p_device_base_addr;
	void *p_hw_timer_handle;
	u32               n_hw_time_out;
	struct vcd_clnt_ctxt_type_t *p_cctxt_list_head;

	enum vcd_command_type e_pending_cmd;

	u32 b_continue;

	struct vcd_transc_type *a_trans_tbl;
	u32 n_trans_tbl_size;

	enum vcd_power_state_type e_pwr_state;
	enum vcd_pwr_clk_state_type e_pwr_clk_state;
	u32 n_active_clnts;
	u32 n_max_perf_lvl;
	u32 n_reqd_perf_lvl;
	u32 n_curr_perf_lvl;
	u32 b_set_perf_lvl_pending;

};

struct vcd_clnt_status_type {
	u32 b_req_perf_lvl;

	u32 b1st_frame_recvd;
	u32 b1st_ip_done_recvd;
	u32 b1st_op_done_recvd;

	u32 n_frame_submitted;
	u32 n_frame_delayed;
	u32 n_cmd_submitted;

	u32 n_int_field_cnt;

	s64 first_ts;
	s64 prev_ts;
	u32 n_time_elapsed;

	u32 b_stop_pending;
	u32 n_flush_mode;

	u32 b_eos_wait_for_op_buf;
	struct vcd_frame_data_type eos_trig_ip_frm;

	u32 b_eos_prev_valid;
	struct ddl_frame_data_type_tag eos_prev_op_frm;
	u32	e_last_err;
	u32	e_last_evt;
	u32	b_cleaning_up;
	u32	b_close_pending;
};

struct vcd_clnt_ctxt_type_t {
	u32 n_signature;
	struct vcd_clnt_state_ctxt_type_t clnt_state;

	s32 driver_id;

	u32 b_live;
	u32 b_decoding;

	struct vcd_property_frame_rate_type frm_rate;
	u32 n_frm_p_units;
	u32 n_reqd_perf_lvl;
	u32 n_time_resoln;

	struct vcd_buffer_pool_type in_buf_pool;
	struct vcd_buffer_pool_type out_buf_pool;

	void (*callback) (u32 event, u32 status, void *p_info, u32 n_size,
			  void *handle, void *const p_client_data);
	void *p_client_data;

	u32 b_sched_clnt_valid;
	void *sched_clnt_hdl;
	u32 n_sched_o_tkn_per_ip_frm;
	u32	b_ddl_hdl_valid;
	u32 *ddl_handle;
	struct vcd_dev_ctxt_type *p_dev_ctxt;
	struct vcd_cmd_q_element_type cmd_q;
	struct vcd_sequence_hdr_type seq_hdr;
	u8 *p_seq_hdr_phy_addr;
	struct vcd_clnt_status_type status;

	struct vcd_clnt_ctxt_type_t *p_next;
};

#define VCD_BUFFERPOOL_INUSE_DECREMENT(val) \
do { \
	if ((val) > 0) \
		val--; \
	else { \
		VCD_MSG_ERROR("%s(): Inconsistent val given in " \
			" VCD_BUFFERPOOL_INUSE_DECREMENT\n", __func__); \
		vcd_assert(); \
	} \
} while (0)

#endif
