/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2013-2020, The Linux Foundation. All rights reserved.
 */

#if !defined(_TRACE_BW_HWMON_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_BW_HWMON_TRACE_H

#undef TRACE_SYSTEM
#define TRACE_SYSTEM power

#include <linux/tracepoint.h>
#include <linux/trace_events.h>

TRACE_EVENT(bw_hwmon_meas,

	TP_PROTO(const char *name, unsigned long mbps,
		 unsigned long us, int wake),

	TP_ARGS(name, mbps, us, wake),

	TP_STRUCT__entry(
		__string(name,			name)
		__field(unsigned long,		mbps)
		__field(unsigned long,		us)
		__field(int,			wake)
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->mbps = mbps;
		__entry->us = us;
		__entry->wake = wake;
	),

	TP_printk("dev: %s, mbps = %lu, us = %lu, wake = %d",
		__get_str(name),
		__entry->mbps,
		__entry->us,
		__entry->wake)
);

TRACE_EVENT(bw_hwmon_update,

	TP_PROTO(const char *name, unsigned long mbps, unsigned long freq,
		 unsigned long up_thres, unsigned long down_thres),

	TP_ARGS(name, mbps, freq, up_thres, down_thres),

	TP_STRUCT__entry(
		__string(name,			name)
		__field(unsigned long,		mbps)
		__field(unsigned long,		freq)
		__field(unsigned long,		up_thres)
		__field(unsigned long,		down_thres)
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->mbps = mbps;
		__entry->freq = freq;
		__entry->up_thres = up_thres;
		__entry->down_thres = down_thres;
	),

	TP_printk("dev: %s, mbps = %lu, freq = %lu, up = %lu, down = %lu",
		__get_str(name),
		__entry->mbps,
		__entry->freq,
		__entry->up_thres,
		__entry->down_thres)
);

#endif /* _TRACE_BW_HWMON_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE governor_bw_hwmon_trace
#include <trace/define_trace.h>
