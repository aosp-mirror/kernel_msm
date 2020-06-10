/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2014-2016, 2019, The Linux Foundation. All rights reserved.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM msm_bus

#if !defined(_TRACE_MSM_BUS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MSM_BUS_H

#include <linux/tracepoint.h>

TRACE_EVENT(bus_rules_matches,

	TP_PROTO(int node_id, int rule_id, unsigned long long node_ab,
		unsigned long long node_ib, unsigned long long node_clk),

	TP_ARGS(node_id, rule_id, node_ab, node_ib, node_clk),

	TP_STRUCT__entry(
		__field(int, node_id)
		__field(int, rule_id)
		__field(u64, node_ab)
		__field(u64, node_ib)
		__field(u64, node_clk)
	),

	TP_fast_assign(
		__entry->node_id = node_id;
		__entry->rule_id = rule_id;
		__entry->node_ab = node_ab;
		__entry->node_ib = node_ib;
		__entry->node_clk = node_clk;
	),

	TP_printk("Rule match node%d rule%d node-ab%llu:ib%llu:clk%llu",
		__entry->node_id, __entry->rule_id,
		(unsigned long long)__entry->node_ab,
		(unsigned long long)__entry->node_ib,
		(unsigned long long)__entry->node_clk)
);

#endif
#define TRACE_INCLUDE_FILE trace_msm_bus
#include <trace/define_trace.h>
