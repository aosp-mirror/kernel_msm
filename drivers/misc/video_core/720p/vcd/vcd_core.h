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

#define VCD_MAX_SCHEDULER_QUEUE_SIZE(fps_n, fps_d)          \
      (fps_n / fps_d * VCD_MAX_SCHEDULER_QUEUE_DURATION)

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

//TODO: remove this
struct vcd_cmd_q_element {
	enum vcd_command_type pending_cmd;
};

struct vcd_dma_buffer {
	void *virt_addr;
	phys_addr_t phys_addr;
	size_t size;
};

struct vcd_buffer_entry {
	u32 valid;
	struct vcd_dma_buffer buffer;
	void *virt_addr;
	phys_addr_t phys_addr;
	size_t size;
//	u8 *alloc;
//	u8 *virtual;  // aligned so == alloc
//	u8 *physical;
//	u32 size;
	u32 allocated;  // true when allocated
	u32 in_use;
	struct vcd_frame_data frame;

};

struct vcd_buffer_pool {
	struct vcd_buffer_entry *entries;
	u32 count;
	struct vcd_buffer_requirement buf_req;
	u32 validated;
	u32 allocated;
	u32 in_use;
	struct vcd_buffer_entry **queue;
	u16 q_len;
	u16 q_head;
	u16 q_tail;

};

struct vcd_transc {
	u32 in_use;
	enum vcd_command_type type;
	struct vcd_clnt_ctxt *cctxt;

	struct vcd_buffer_entry *ip_buf_entry;

	s64 time_stamp;
	u32 ip_frm_tag;
	enum vcd_frame frame_type;

	struct vcd_buffer_entry *op_buf_entry;

	u32 input_done;
	u32 frame_done;
};

struct vcd_dev_ctxt {
	u32 ddl_cmd_concurrency;
	u32 ddl_frame_ch_depth;
	u32 ddl_cmd_ch_depth;
	u32 ddl_frame_ch_interim;
	u32 ddl_cmd_ch_interim;
	u32 ddl_frame_ch_free;
	u32 ddl_cmd_ch_free;

	void *sched_hdl;

	struct vcd_init_config config;

	u32 driver_ids[VCD_DRIVER_INSTANCE_MAX];
	u32 refs;
	u8 *device_base_addr;
	void *hw_timer_handle;
	u32               hw_time_out;
	struct vcd_clnt_ctxt *cctxt_list_head;

	enum vcd_command_type pending_cmd;

	u32 cont;

	struct vcd_transc *trans_tbl;
	u32 trans_tbl_size;

	enum vcd_power_state pwr_state;
	enum vcd_pwr_clk_state_type pwr_clk_state;
	u32 active_clnts;
	u32 max_perf_lvl;
	u32 reqd_perf_lvl;
	u32 curr_perf_lvl;
	u32 set_perf_lvl_pending;

};

struct vcd_clnt_status {
	u32 req_perf_lvl;

	u32 b1st_frame_recvd;
	u32 b1st_ip_done_recvd;
	u32 b1st_op_done_recvd;

	u32 frame_submitted;
	u32 frame_delayed;
	u32 cmd_submitted;

	u32 int_field_cnt;

	s64 first_ts;
	s64 prev_ts;
	u32 time_elapsed;

	u32 stop_pending;
	u32 flush_mode;

	u32 eos_wait_for_op_buf;
	struct vcd_frame_data eos_trig_ip_frm;

	u32 eos_prev_valid;
	struct ddl_frame_data_tag eos_prev_op_frm;
	u32	last_err;
	u32	last_evt;
	u32	cleaning_up;
	u32	close_pending;
};

struct vcd_clnt_ctxt {
	u32 signature;
	struct vcd_clnt_state_ctxt clnt_state;

	s32 driver_id;

	u32 live;
	u32 decoding;

	struct vcd_property_frame_rate frm_rate;
	u32 frm_p_units;
	u32 reqd_perf_lvl;
	u32 time_resoln;

	struct vcd_buffer_pool in_buf_pool;
	struct vcd_buffer_pool out_buf_pool;

	void (*callback) (u32 event, u32 status, void *info, u32 size,
			  void *handle, void *const client_data);
	void *client_data;

	u32 sched_clnt_valid;
	void *sched_clnt_hdl;
	u32 sched_o_tkn_per_ip_frm;
	u32 ddl_hdl_valid;
	u32 *ddl_handle;
	struct vcd_dev_ctxt *dev_ctxt;
	struct vcd_cmd_q_element cmd_q;

	struct vcd_sequence_hdr seq_hdr;
	phys_addr_t seq_hdr_phys_addr;

	struct vcd_clnt_status status;

	struct vcd_clnt_ctxt *next;
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
