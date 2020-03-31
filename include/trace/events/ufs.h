/*
 * Copyright (c) 2013-2015, 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM ufs

#if !defined(_TRACE_UFS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_UFS_H

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(ufshcd_state_change_template,
	TP_PROTO(const char *dev_name, int state),

	TP_ARGS(dev_name, state),

	TP_STRUCT__entry(
		__string(dev_name, dev_name)
		__field(int, state)
	),

	TP_fast_assign(
		__assign_str(dev_name, dev_name);
		__entry->state = state;
	),

	TP_printk("%s: state changed to %s",
		__get_str(dev_name), __entry->state ? "ON" : "OFF")
);

DEFINE_EVENT_PRINT(ufshcd_state_change_template, ufshcd_clk_gating,
	TP_PROTO(const char *dev_name, int state),
	TP_ARGS(dev_name, state),
	TP_printk("%s: state changed to %s", __get_str(dev_name),
		__print_symbolic(__entry->state,
				{ CLKS_OFF, "CLKS_OFF" },
				{ CLKS_ON, "CLKS_ON" },
				{ REQ_CLKS_OFF, "REQ_CLKS_OFF" },
				{ REQ_CLKS_ON, "REQ_CLKS_ON" }))
);

DEFINE_EVENT_PRINT(ufshcd_state_change_template, ufshcd_hibern8_on_idle,
	TP_PROTO(const char *dev_name, int state),
	TP_ARGS(dev_name, state),
	TP_printk("%s: state changed to %s", __get_str(dev_name),
		__print_symbolic(__entry->state,
			{ HIBERN8_ENTERED, "HIBERN8_ENTER" },
			{ HIBERN8_EXITED, "HIBERN8_EXIT" },
			{ REQ_HIBERN8_ENTER, "REQ_HIBERN8_ENTER" },
			{ REQ_HIBERN8_EXIT, "REQ_HIBERN8_EXIT" }))
);

DEFINE_EVENT(ufshcd_state_change_template, ufshcd_auto_bkops_state,
	TP_PROTO(const char *dev_name, int state),
	TP_ARGS(dev_name, state));

TRACE_EVENT(ufshcd_clk_scaling,

	TP_PROTO(const char *dev_name, const char *state, const char *clk,
		u32 prev_state, u32 curr_state),

	TP_ARGS(dev_name, state, clk, prev_state, curr_state),

	TP_STRUCT__entry(
		__string(dev_name, dev_name)
		__string(state, state)
		__string(clk, clk)
		__field(u32, prev_state)
		__field(u32, curr_state)
	),

	TP_fast_assign(
		__assign_str(dev_name, dev_name);
		__assign_str(state, state);
		__assign_str(clk, clk);
		__entry->prev_state = prev_state;
		__entry->curr_state = curr_state;
	),

	TP_printk("%s: %s %s from %u to %u Hz",
		__get_str(dev_name), __get_str(state), __get_str(clk),
		__entry->prev_state, __entry->curr_state)
);

DECLARE_EVENT_CLASS(ufshcd_profiling_template,
	TP_PROTO(const char *dev_name, const char *profile_info, s64 time_us,
		 int err),

	TP_ARGS(dev_name, profile_info, time_us, err),

	TP_STRUCT__entry(
		__string(dev_name, dev_name)
		__string(profile_info, profile_info)
		__field(s64, time_us)
		__field(int, err)
	),

	TP_fast_assign(
		__assign_str(dev_name, dev_name);
		__assign_str(profile_info, profile_info);
		__entry->time_us = time_us;
		__entry->err = err;
	),

	TP_printk("%s: %s: took %lld usecs, err %d",
		__get_str(dev_name), __get_str(profile_info),
		__entry->time_us, __entry->err)
);

DEFINE_EVENT(ufshcd_profiling_template, ufshcd_profile_hibern8,
	TP_PROTO(const char *dev_name, const char *profile_info, s64 time_us,
		 int err),
	TP_ARGS(dev_name, profile_info, time_us, err));

DEFINE_EVENT(ufshcd_profiling_template, ufshcd_profile_clk_gating,
	TP_PROTO(const char *dev_name, const char *profile_info, s64 time_us,
		 int err),
	TP_ARGS(dev_name, profile_info, time_us, err));

DEFINE_EVENT(ufshcd_profiling_template, ufshcd_profile_clk_scaling,
	TP_PROTO(const char *dev_name, const char *profile_info, s64 time_us,
		 int err),
	TP_ARGS(dev_name, profile_info, time_us, err));

DECLARE_EVENT_CLASS(ufshcd_template,
	TP_PROTO(const char *dev_name, int err, s64 usecs,
		 int dev_state, int link_state),

	TP_ARGS(dev_name, err, usecs, dev_state, link_state),

	TP_STRUCT__entry(
		__field(s64, usecs)
		__field(int, err)
		__string(dev_name, dev_name)
		__field(int, dev_state)
		__field(int, link_state)
	),

	TP_fast_assign(
		__entry->usecs = usecs;
		__entry->err = err;
		__assign_str(dev_name, dev_name);
		__entry->dev_state = dev_state;
		__entry->link_state = link_state;
	),

	TP_printk(
		"%s: took %lld usecs, dev_state: %s, link_state: %s, err %d",
		__get_str(dev_name),
		__entry->usecs,
		__print_symbolic(__entry->dev_state,
			{ UFS_ACTIVE_PWR_MODE, "ACTIVE" },
			{ UFS_SLEEP_PWR_MODE, "SLEEP" },
			{ UFS_POWERDOWN_PWR_MODE, "POWERDOWN" }),
		__print_symbolic(__entry->link_state,
			{ UIC_LINK_OFF_STATE, "LINK_OFF" },
			{ UIC_LINK_ACTIVE_STATE, "LINK_ACTIVE" },
			{ UIC_LINK_HIBERN8_STATE, "LINK_HIBERN8" }),
		__entry->err
	)
);

DEFINE_EVENT(ufshcd_template, ufshcd_system_suspend,
	TP_PROTO(const char *dev_name, int err, s64 usecs,
		int dev_state, int link_state),
	TP_ARGS(dev_name, err, usecs, dev_state, link_state));

DEFINE_EVENT(ufshcd_template, ufshcd_system_resume,
	TP_PROTO(const char *dev_name, int err, s64 usecs,
		int dev_state, int link_state),
	TP_ARGS(dev_name, err, usecs, dev_state, link_state));

DEFINE_EVENT(ufshcd_template, ufshcd_runtime_suspend,
	TP_PROTO(const char *dev_name, int err, s64 usecs,
		int dev_state, int link_state),
	TP_ARGS(dev_name, err, usecs, dev_state, link_state));

DEFINE_EVENT(ufshcd_template, ufshcd_runtime_resume,
	TP_PROTO(const char *dev_name, int err, s64 usecs,
		int dev_state, int link_state),
	TP_ARGS(dev_name, err, usecs, dev_state, link_state));

DEFINE_EVENT(ufshcd_template, ufshcd_init,
	TP_PROTO(const char *dev_name, int err, s64 usecs,
		int dev_state, int link_state),
	TP_ARGS(dev_name, err, usecs, dev_state, link_state));

TRACE_EVENT(ufshcd_command,
	TP_PROTO(const char *dev_name, const char *str, unsigned int tag,
			u32 doorbell, u32 transfer_len, u32 intr, u64 lba,
			u8 opcode),

	TP_ARGS(dev_name, str, tag, doorbell, transfer_len, intr, lba, opcode),

	TP_STRUCT__entry(
		__string(dev_name, dev_name)
		__string(str, str)
		__field(unsigned int, tag)
		__field(u32, doorbell)
		__field(u32, transfer_len)
		__field(u32, intr)
		__field(u64, lba)
		__field(u8, opcode)
	),

	TP_fast_assign(
		__assign_str(dev_name, dev_name);
		__assign_str(str, str);
		__entry->tag = tag;
		__entry->doorbell = doorbell;
		__entry->transfer_len = transfer_len;
		__entry->intr = intr;
		__entry->lba = lba;
		__entry->opcode = opcode;
	),

	TP_printk(
		"%s: %14s: tag: %-2u cmd: 0x%-2x lba: %-9llu size: %-7u DB: 0x%-8x IS: 0x%x",
		__get_str(dev_name), __get_str(str), __entry->tag,
		(u32)__entry->opcode, __entry->lba, __entry->transfer_len,
		__entry->doorbell, __entry->intr
	)
);

TRACE_EVENT(ufs_stats,
	TP_PROTO(struct ufs_stats *ufs_stats, struct ufshcd_io_stat *prev_rstat,
			struct ufshcd_io_stat *prev_wstat, u64 *avg_time),

	TP_ARGS(ufs_stats, prev_rstat, prev_wstat, avg_time),

	TP_STRUCT__entry(
		__field(u64,	peak_read)
		__field(u64,	peak_write)
		__field(u64,	peak_flush)
		__field(u64,	peak_discard)
		__field(u64,	peak_qdepth)
		__field(u64,	avg_read)
		__field(u64,	avg_write)
		__field(u64,	avg_flush)
		__field(u64,	avg_discard)
		__field(u64,	r_rc_s)
		__field(u64,	r_tb_s)
		__field(u64,	w_rc_s)
		__field(u64,	w_tb_s)
		__field(u64,	r_rc_c)
		__field(u64,	r_tb_c)
		__field(u64,	w_rc_c)
		__field(u64,	w_tb_c)
		__field(u64,	r_rem)
		__field(u64,	w_rem)
	),

	TP_fast_assign(
		__entry->peak_read	= ufs_stats->peak_reqs[TS_READ];
		__entry->peak_write	= ufs_stats->peak_reqs[TS_WRITE];
		__entry->peak_flush	= ufs_stats->peak_reqs[TS_FLUSH];
		__entry->peak_discard	= ufs_stats->peak_reqs[TS_DISCARD];
		__entry->peak_qdepth	= ufs_stats->peak_queue_depth;
		__entry->avg_read	= avg_time[TS_READ];
		__entry->avg_write	= avg_time[TS_WRITE];
		__entry->avg_flush	= avg_time[TS_FLUSH];
		__entry->avg_discard	= avg_time[TS_DISCARD];
		__entry->r_rc_s	= ufs_stats->io_read.req_count_started
				- prev_rstat->req_count_started;
		__entry->r_tb_s	= ufs_stats->io_read.total_bytes_started
				- prev_rstat->total_bytes_started;
		__entry->w_rc_s	= ufs_stats->io_write.req_count_started
				- prev_wstat->req_count_started;
		__entry->w_tb_s	= ufs_stats->io_write.total_bytes_started
				- prev_wstat->total_bytes_started;
		__entry->r_rc_c	= ufs_stats->io_read.req_count_completed
				- prev_rstat->req_count_completed;
		__entry->r_tb_c	= ufs_stats->io_read.total_bytes_completed
				- prev_rstat->total_bytes_completed;
		__entry->w_rc_c	= ufs_stats->io_write.req_count_completed
				- prev_wstat->req_count_completed;
		__entry->w_tb_c	= ufs_stats->io_write.total_bytes_completed
				- prev_wstat->total_bytes_completed;
		__entry->r_rem	= ufs_stats->io_read.req_count_started
				- ufs_stats->io_read.req_count_completed;
		__entry->w_rem	= ufs_stats->io_write.req_count_started
				- ufs_stats->io_write.req_count_completed;
	),

	TP_printk(
		"avg/max(us): read(%llu/%llu) write(%llu/%llu) "
		"flush(%llu/%llu) discard(%llu/%llu), "
		"started_bytes/count: read(%llu/%llu) write(%llu/%llu), "
		"completed_bytes/count: read(%llu/%llu) write(%llu/%llu), "
		"in-flight_read/write: %llu/%llu, peak_queue_depth: %llu",
		__entry->avg_read, __entry->peak_read,
		__entry->avg_write, __entry->peak_write,
		__entry->avg_flush, __entry->peak_flush,
		__entry->avg_discard, __entry->peak_discard,
		__entry->r_tb_s, __entry->r_rc_s,
		__entry->w_tb_s, __entry->w_rc_s,
		__entry->r_tb_c, __entry->r_rc_c,
		__entry->w_tb_c, __entry->w_rc_c,
		__entry->r_rem, __entry->w_rem, __entry->peak_qdepth
	)
);

#endif /* if !defined(_TRACE_UFS_H) || defined(TRACE_HEADER_MULTI_READ) */

/* This part must be outside protection */
#include <trace/define_trace.h>
