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
#ifndef _VCD_H_
#define _VCD_H_

#include "vcd_api.h"
#include "vid_frame_scheduler_api.h"
#include "vcd_ddl_api.h"
#include "vcd_res_tracker_api.h"
#include "vcd_util.h"
#include "vcd_client_sm.h"
#include "vcd_core.h"
#include "vcd_device_sm.h"

void vcd_reset_device_channels(struct vcd_dev_ctxt *dev_ctxt);

u32 vcd_get_command_channel(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc **pp_transc);

u32 vcd_get_command_channel_in_loop(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc **pp_transc);

void vcd_mark_command_channel(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc);

void vcd_release_command_channel(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc);

void vcd_release_multiple_command_channels(struct vcd_dev_ctxt *dev_ctxt,
	u32 channels);

void vcd_release_interim_command_channels(struct vcd_dev_ctxt *dev_ctxt);

u32 vcd_get_frame_channel(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc **pp_transc);

u32 vcd_get_frame_channel_in_loop(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc **pp_transc);

void vcd_mark_frame_channel(struct vcd_dev_ctxt *dev_ctxt);

void vcd_release_frame_channel(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc);

void vcd_release_multiple_frame_channels(struct vcd_dev_ctxt *dev_ctxt,
	u32 channels);

void vcd_release_interim_frame_channels(struct vcd_dev_ctxt *dev_ctxt);
u32 vcd_core_is_busy(struct vcd_dev_ctxt *dev_ctxt);

void vcd_device_timer_start(struct vcd_dev_ctxt *dev_ctxt);
void vcd_device_timer_stop(struct vcd_dev_ctxt *dev_ctxt);

u32 vcd_init_device_context(struct vcd_drv_ctxt *drv_ctxt, u32 ev_code);

u32 vcd_deinit_device_context(struct vcd_drv_ctxt *drv_ctxt, u32 ev_code);

u32 vcd_init_client_context(struct vcd_clnt_ctxt *cctxt);

void vcd_destroy_client_context(struct vcd_clnt_ctxt *cctxt);

u32 vcd_check_for_client_context(struct vcd_dev_ctxt *dev_ctxt, s32 driver_id);

u32 vcd_validate_driver_handle(struct vcd_dev_ctxt *dev_ctxt,
	s32 driver_handle);

void vcd_handle_for_last_clnt_close(struct vcd_dev_ctxt *dev_ctxt,
	u32 send_deinit);

u32 vcd_common_allocate_set_buffer(struct vcd_clnt_ctxt *cctxt,
	enum vcd_buffer_type buffer, size_t sz,
	struct vcd_buffer_pool **pp_buf_pool);

u32 vcd_set_buffer_internal(struct vcd_clnt_ctxt *cctxt,
	struct vcd_buffer_pool *buf_pool, void *buf, size_t sz);

u32 vcd_allocate_buffer_internal(struct vcd_clnt_ctxt *cctxt,
	struct vcd_buffer_pool *buf_pool, size_t buf_size, void **virt_addr,
	phys_addr_t *phys_addr);

u32 vcd_free_one_buffer_internal(struct vcd_clnt_ctxt *cctxt,
	enum vcd_buffer_type vcd_buffer_type, u8 *buffer);

u32 vcd_free_buffers_internal(struct vcd_clnt_ctxt *cctxt,
	struct vcd_buffer_pool *buf_pool);

u32 vcd_alloc_buffer_pool_entries(struct vcd_buffer_pool *buf_pool,
	struct vcd_buffer_requirement *buf_req);

void vcd_free_buffer_pool_entries(struct vcd_buffer_pool *buf_pool);

void vcd_flush_in_use_buffer_pool_entries(struct vcd_clnt_ctxt *cctxt,
	struct vcd_buffer_pool *buf_pool, u32 event);

void vcd_reset_buffer_pool_for_reuse(struct vcd_buffer_pool *buf_pool);

struct vcd_buffer_entry *vcd_get_free_buffer_pool_entry(
	struct vcd_buffer_pool *pool);

struct vcd_buffer_entry *vcd_find_buffer_pool_entry(struct vcd_buffer_pool
	*pool, void *virt_addr);

struct vcd_buffer_entry *vcd_buffer_pool_entry_de_q(
	struct vcd_buffer_pool *pool);

u32 vcd_buffer_pool_entry_en_q(struct vcd_buffer_pool *pool,
	struct vcd_buffer_entry *entry);

u32 vcd_client_cmd_en_q(struct vcd_clnt_ctxt *cctxt,
	enum vcd_command_type command);

void vcd_client_cmd_flush_and_en_q(struct vcd_clnt_ctxt *cctxt,
	enum vcd_command_type command);

u32 vcd_client_cmd_de_q(struct vcd_clnt_ctxt *cctxt,
	enum vcd_command_type *command);

u32 vcd_handle_recvd_eos(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame, u32 * pb_eos_handled);

u32 vcd_handle_first_decode_frame(struct vcd_clnt_ctxt *cctxt);

u32 vcd_add_client_to_sched(struct vcd_clnt_ctxt *cctxt);

u32 vcd_handle_input_frame(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame);

u32 vcd_store_seq_hdr(struct vcd_clnt_ctxt *cctxt,
	struct vcd_sequence_hdr *seq_hdr);

u32 vcd_set_frame_size(struct vcd_clnt_ctxt *cctxt,
	struct vcd_property_frame_size *frm_size);

u32 vcd_set_frame_rate(struct vcd_clnt_ctxt *cctxt,
	struct vcd_property_frame_rate *fps);

u32 vcd_calculate_frame_delta(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *frame);

struct vcd_buffer_entry *vcd_check_fill_output_buffer(
	struct vcd_clnt_ctxt *cctxt, struct vcd_frame_data *buffer);

u32 vcd_requeue_input_frame(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_clnt_ctxt *cctxt, struct vcd_buffer_entry *buf_entry);

u32 vcd_schedule_frame(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_clnt_ctxt **pp_cctxt,
	struct vcd_buffer_entry **pp_ip_buf_entry);

u32 vcd_map_sched_status(enum sched_status sched_status);

u32 vcd_submit_command_in_continue(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc);

u32 vcd_submit_cmd_sess_start(struct vcd_transc *transc);

u32 vcd_submit_cmd_sess_end(struct vcd_transc *transc);

void vcd_submit_cmd_client_close(struct vcd_clnt_ctxt *cctxt);

u32 vcd_submit_frame(struct vcd_dev_ctxt *dev_ctxt, struct vcd_transc *transc);

u32 vcd_try_submit_frame_in_continue(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc);

u32 vcd_process_cmd_sess_start(struct vcd_clnt_ctxt *cctxt);

void vcd_try_submit_frame(struct vcd_dev_ctxt *dev_ctxt);

u32 vcd_setup_with_ddl_capabilities(struct vcd_dev_ctxt *dev_ctxt);
void vcd_handle_submit_frame_failed(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc);

struct vcd_transc *vcd_get_free_trans_tbl_entry(struct vcd_dev_ctxt *dev_ctxt);

void vcd_release_trans_tbl_entry(struct vcd_transc *trans_entry);

void vcd_release_all_clnt_frm_transc(struct vcd_clnt_ctxt *cctxt);
void vcd_release_all_clnt_def_frm_transc(struct vcd_clnt_ctxt *cctxt);
void vcd_release_all_clnt_transc(struct vcd_clnt_ctxt *cctxt);

u32 vcd_handle_input_done(struct vcd_clnt_ctxt *cctxt, void *payload,
	u32 event, u32 status);

void vcd_handle_input_done_in_eos(struct vcd_clnt_ctxt *cctxt, void *payload,
	u32 status);

void vcd_handle_input_done_failed(struct vcd_clnt_ctxt *cctxt,
	struct vcd_transc *transc);

void vcd_handle_input_done_for_interlacing(struct vcd_clnt_ctxt *cctxt);

void vcd_handle_input_done_with_trans_end(struct vcd_clnt_ctxt *cctxt);

u32 vcd_handle_frame_done(struct vcd_clnt_ctxt *cctxt, void *payload,
	u32 event, u32 status);

void vcd_handle_frame_done_for_interlacing(struct vcd_clnt_ctxt *cctxt,
	struct vcd_transc *transc_ip1, struct ddl_frame_data_tag *op_frm,
	u32 status);

u32 vcd_handle_first_frame_done(struct vcd_clnt_ctxt *cctxt, void *payload);

void vcd_handle_frame_done_in_eos(struct vcd_clnt_ctxt *cctxt, void *payload,
	u32 status);

u32 vcd_handle_first_encode_frame_done(struct vcd_clnt_ctxt *cctxt,
	void *payload);

u32 vcd_handle_output_required(struct vcd_clnt_ctxt *cctxt, void *payload,
	u32 status);

u32 vcd_handle_output_required_in_flushing(struct vcd_clnt_ctxt *cctxt,
	void *payload);

u32 vcd_handle_output_req_tran_end_in_eos(struct vcd_clnt_ctxt *cctxt);

u32 vcd_validate_io_done_pyld(void *payload, u32 status);

void vcd_handle_eos_trans_end(struct vcd_clnt_ctxt *cctxt);

void vcd_handle_eos_done(struct vcd_clnt_ctxt *cctxt,
	struct vcd_transc *transc, u32 status);

void vcd_send_frame_done_in_eos(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame, u32 valid_opbuf);

void vcd_send_frame_done_in_eos_for_dec(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame);

void vcd_send_frame_done_in_eos_for_enc(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame);

void vcd_handle_start_done(struct vcd_clnt_ctxt *cctxt,
	struct vcd_transc *transc, u32 status);

void vcd_handle_stop_done(struct vcd_clnt_ctxt *cctxt,
	struct vcd_transc *transc, u32 status);

void vcd_handle_stop_done_in_starting(struct vcd_clnt_ctxt *cctxt,
	struct vcd_transc *transc, u32 status);

void vcd_handle_stop_done_in_invalid(struct vcd_clnt_ctxt *cctxt, u32 status);

void vcd_send_flush_done(struct vcd_clnt_ctxt *cctxt, u32 status);

void vcd_process_pending_flush_in_eos(struct vcd_clnt_ctxt *cctxt);

void vcd_process_pending_stop_in_eos(struct vcd_clnt_ctxt *cctxt);

void vcd_handle_trans_pending(struct vcd_clnt_ctxt *cctxt);

void vcd_flush_output_buffers(struct vcd_clnt_ctxt *cctxt);

u32 vcd_flush_buffers(struct vcd_clnt_ctxt *cctxt, u32 mode);
void vcd_flush_buffers_in_err_fatal(struct vcd_clnt_ctxt *cctxt);

u32 vcd_power_event(struct vcd_dev_ctxt *dev_ctxt, struct vcd_clnt_ctxt *cctxt,
	u32 event);

u32 vcd_device_power_event(struct vcd_dev_ctxt *dev_ctxt, u32 event,
	struct vcd_clnt_ctxt *cctxt);

u32 vcd_client_power_event(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_clnt_ctxt *cctxt, u32 event);

u32 vcd_enable_clock(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_clnt_ctxt *cctxt);

u32 vcd_disable_clock(struct vcd_dev_ctxt *dev_ctxt);

u32 vcd_set_perf_level(struct vcd_dev_ctxt *dev_ctxt, u32 perf_lvl,
	struct vcd_clnt_ctxt *cctxt);

u32 vcd_update_clnt_perf_lvl(struct vcd_clnt_ctxt *cctxt,
	struct vcd_property_frame_rate *fps, u32 frm_p_units);

u32 vcd_gate_clock(struct vcd_dev_ctxt *dev_ctxt);

u32 vcd_un_gate_clock(struct vcd_dev_ctxt *dev_ctxt);

void vcd_handle_err_fatal(struct vcd_clnt_ctxt *cctxt, u32 event, u32 status);

void vcd_handle_device_err_fatal(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_clnt_ctxt *cctxt);

void vcd_clnt_handle_device_err_fatal(struct vcd_clnt_ctxt *cctxt, u32 event);

void vcd_handle_err_in_starting(struct vcd_clnt_ctxt *cctxt, u32 status);

void vcd_handle_ind_hw_err_fatal(struct vcd_clnt_ctxt *cctxt, u32 event,
	u32 status);

#endif
