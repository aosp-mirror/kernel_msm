/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 Google LLC
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM msm_bus_dbg

#if !defined(_TRACE_MSM_BUS_DBG_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MSM_BUS_DBG_H

#include <linux/tracepoint.h>

TRACE_EVENT(bus_update_request,

	TP_PROTO(int sec, int nsec, const char *name, int src, int dest,
		unsigned long long ab, unsigned long long ib),

	TP_ARGS(sec, nsec, name, src, dest, ab, ib),

	TP_STRUCT__entry(
		__field(int, sec)
		__field(int, nsec)
		__string(name, name)
		__field(int, src)
		__field(int, dest)
		__field(u64, ab)
		__field(u64, ib)
	),

	TP_fast_assign(
		__entry->sec = sec;
		__entry->nsec = nsec;
		__assign_str(name, name);
		__entry->src = src;
		__entry->dest = dest;
		__entry->ab = ab;
		__entry->ib = ib;
	),

	TP_printk("time= %u.%09u name=%s src=%d dest=%d ab=%llu ib=%llu",
		__entry->sec,
		__entry->nsec,
		__get_str(name),
		__entry->src,
		__entry->dest,
		(unsigned long long)__entry->ab,
		(unsigned long long)__entry->ib)
);

TRACE_EVENT(bus_agg_bw,

	TP_PROTO(unsigned int node_id, int rpm_id, int ctx_set,
		unsigned long long agg_ab),

	TP_ARGS(node_id, rpm_id, ctx_set, agg_ab),

	TP_STRUCT__entry(
		__field(unsigned int, node_id)
		__field(int, rpm_id)
		__field(int, ctx_set)
		__field(u64, agg_ab)
	),

	TP_fast_assign(
		__entry->node_id = node_id;
		__entry->rpm_id = rpm_id;
		__entry->ctx_set = ctx_set;
		__entry->agg_ab = agg_ab;
	),

	TP_printk("node_id:%u rpm_id:%d rpm_ctx:%d agg_ab:%llu",
		__entry->node_id,
		__entry->rpm_id,
		__entry->ctx_set,
		(unsigned long long)__entry->agg_ab)
);

TRACE_EVENT(bus_client_status,

	TP_PROTO(const char *name, int src, int dest,
		unsigned long long ab, unsigned long long ib,
		int active_only, int vote_count),

	TP_ARGS(name, src, dest, ab, ib, active_only, vote_count),

	TP_STRUCT__entry(
		__string(name, name)
		__field(int, src)
		__field(int, dest)
		__field(u64, ab)
		__field(u64, ib)
		__field(int, active_only)
		__field(int, vote_count)
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->src = src;
		__entry->dest = dest;
		__entry->ab = ab;
		__entry->ib = ib;
		__entry->active_only = active_only;
		__entry->vote_count = vote_count;
	),

	TP_printk("name=%s src=%d dest=%d ab=%llu ib=%llu active_only=%d vote_count=%d",
		__get_str(name),
		__entry->src,
		__entry->dest,
		(unsigned long long)__entry->ab,
		(unsigned long long)__entry->ib,
		__entry->active_only,
		__entry->vote_count)
);

TRACE_EVENT(bus_bcm_client_status,

	TP_PROTO(const char *bcm, const char *client,
		unsigned long long act_ab, unsigned long long act_ib,
		unsigned long long slp_ab, unsigned long long slp_ib),

	TP_ARGS(bcm, client, act_ab, act_ib, slp_ab, slp_ib),

	TP_STRUCT__entry(
		__string(bcm, bcm)
		__string(client, client)
		__field(u64, act_ab)
		__field(u64, act_ib)
		__field(u64, slp_ab)
		__field(u64, slp_ib)
	),

	TP_fast_assign(
		__assign_str(bcm, bcm);
		 __assign_str(client, client);
		__entry->act_ab = act_ab;
		__entry->act_ib = act_ib;
		__entry->slp_ab = slp_ab;
		__entry->slp_ib = slp_ib;
	),

	TP_printk(
		"bcm=%s cl=%s act_ab=%llu act_ib=%llu slp_ab=%llu slp_ib=%llu",
		__get_str(bcm),
		__get_str(client),
		(unsigned long long)__entry->act_ab,
		(unsigned long long)__entry->act_ib,
		(unsigned long long)__entry->slp_ab,
		(unsigned long long)__entry->slp_ib)
);
#endif
#define TRACE_INCLUDE_FILE trace_msm_bus_dbg
#include <trace/define_trace.h>

