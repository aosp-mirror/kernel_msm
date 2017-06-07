#undef TRACE_SYSTEM
#define TRACE_SYSTEM critical

#if !defined(_TRACE_CRITICAL_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_CRITICAL_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>
#include <linux/string.h>

TRACE_EVENT(critical_start,

	TP_PROTO(unsigned long ip, unsigned long parent_ip),

	TP_ARGS(ip, parent_ip),

	TP_STRUCT__entry(
		__array(char, func, 16)
		__array(char, parent, 16)
	),

	TP_fast_assign(
		snprintf(__entry->func, 16, "%pf", (void *)ip);
		snprintf(__entry->parent, 16, "%pf", (void *)parent_ip);
	),

	TP_printk("caller=%s parent=%s\n",
		  __entry->func,
		  __entry->parent)
);

TRACE_EVENT(critical_end,

	TP_PROTO(unsigned long ip, unsigned long parent_ip),

	TP_ARGS(ip, parent_ip),

	TP_STRUCT__entry(
		__array(char, func, 16)
		__array(char, parent, 16)
	),

	TP_fast_assign(
		snprintf(__entry->func, 16, "%pf", (void *)ip);
		snprintf(__entry->parent, 16, "%pf", (void *)parent_ip);
	),

	TP_printk("caller=%s parent=%s\n",
		  __entry->func,
		  __entry->parent)
);
#endif /* _TRACE_CRITICAL_H */

/* This part ust be outside protection */
#include <trace/define_trace.h>
