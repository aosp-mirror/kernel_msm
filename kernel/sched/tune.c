#include <linux/cgroup.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/percpu.h>
#include <linux/printk.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>

#include <trace/events/sched.h>

#include "sched.h"
#include "tune.h"

unsigned int sysctl_sched_cfs_boost __read_mostly = 0;

/* Performance Boost region (B) threshold params */
static int perf_boost_idx;

/* Performance Constraint region (C) threshold params */
static int perf_constrain_idx;

/**
 * Performance-Energy (P-E) Space thresholds constants
 */
struct threshold_params {
	int nrg_gain;
	int cap_gain;
};

/*
 * System specific P-E space thresholds constants
 */
static struct threshold_params
threshold_gains[] = {
	{ 0, 4 }, /* >=  0% */
	{ 1, 4 }, /* >= 10% */
	{ 2, 4 }, /* >= 20% */
	{ 3, 4 }, /* >= 30% */
	{ 4, 4 }, /* >= 40% */
	{ 4, 3 }, /* >= 50% */
	{ 4, 2 }, /* >= 60% */
	{ 4, 1 }, /* >= 70% */
	{ 4, 0 }, /* >= 80% */
	{ 4, 0 }  /* >= 90% */
};

/*
 * System energy normalization constants
 */
struct target_nrg {
	unsigned long min_power;
	unsigned long max_power;
	unsigned long nrg_shift;
	unsigned long nrg_mult;
};

/*
 * Target specific system energy normalization constants
 * NOTE: These values are specific for ARM TC2 and they are derived from the
 *       energy model data defined in: arch/arm/kernel/topology.c
 */
static struct target_nrg
schedtune_target_nrg = {

	/*
	 * TC2 Min CPUs power:
	 * all CPUs idle, all clusters in deep idle:
	 *   0 * 3 + 0 * 2 + 10 + 25
	 */
	.min_power = 35,

	/*
	 * TC2 Max CPUs power:
	 * all CPUs fully utilized while running at max OPP:
	 *   1024 * 3 + 6997 * 2 + 4905 + 15200
	 */

	.max_power = 37171,

	/*
	 * Fast integer division by constant:
	 *  Constant   : Max - Min       (C) = 37171 - 35 = 37136
	 *  Precision  : 0.1%            (P) = 0.1
	 *  Reference  : C * 100 / P     (R) = 3713600
	 *
	 * Thus:
	 *  Shift bifs : ceil(log(R,2))  (S) = 26
	 *  Mult const : round(2^S/C)    (M) = 1807
	 *
	 * This allows to compute the normalized energy:
	 *   system_energy / C
	 * as:
	 *   (system_energy * M) >> S
	 */
	.nrg_shift = 26,	/* S */
	.nrg_mult  = 1807,	/* M */
};

/*
 * System energy normalization
 * Returns the normalized value, in the range [0..SCHED_LOAD_SCALE],
 * corresponding to the specified energy variation.
 */
int
schedtune_normalize_energy(int energy_diff)
{
	long long normalized_nrg = energy_diff;
	int max_delta;

	/* Check for boundaries */
	max_delta  = schedtune_target_nrg.max_power;
	max_delta -= schedtune_target_nrg.min_power;
	WARN_ON(abs(energy_diff) >= max_delta);

	/* Scale by energy magnitude */
	normalized_nrg <<= SCHED_LOAD_SHIFT;

	/* Normalize on max energy for target platform */
	normalized_nrg  *= schedtune_target_nrg.nrg_mult;
	normalized_nrg >>= schedtune_target_nrg.nrg_shift;

	return normalized_nrg;
}

static int
__schedtune_accept_deltas(int nrg_delta, int cap_delta,
		int perf_boost_idx, int perf_constrain_idx) {
	int energy_payoff;

	/* Performance Boost (B) region */
	if (nrg_delta > 0 && cap_delta > 0) {
		/*
		 * energy_payoff criteria:
		 *    cap_delta / nrg_delta > cap_gain / nrg_gain
		 * which is:
		 *    nrg_delta * cap_gain < cap_delta * nrg_gain
		 */
		energy_payoff  = cap_delta * threshold_gains[perf_boost_idx].nrg_gain;
		energy_payoff -= nrg_delta * threshold_gains[perf_boost_idx].cap_gain;
#if 0
		// TJK: fix tracing
		trace_sched_tune_filter(
				threshold_gains[perf_boost_idx].nrg_gain,
				threshold_gains[perf_boost_idx].cap_gain,
				energy_payoff, 8);
#endif
		return energy_payoff;
	}

	/* Performance Constraint (C) region */
	if (nrg_delta < 0 && cap_delta < 0) {
		/*
		 * energy_payoff criteria:
		 *    cap_delta / nrg_delta < cap_gain / nrg_gain
		 * which is:
		 *    nrg_delta * cap_gain > cap_delta * nrg_gain
		 */
		energy_payoff  = nrg_delta * threshold_gains[perf_constrain_idx].cap_gain;
		energy_payoff -= cap_delta * threshold_gains[perf_constrain_idx].nrg_gain;

#if 0
		// TJK: fix tracing
		trace_sched_tune_filter(
				threshold_gains[perf_constrain_idx].nrg_gain,
				threshold_gains[perf_constrain_idx].cap_gain,
				energy_payoff, 6);
#endif
		return energy_payoff;
	}

	/* Default: reject schedule candidate */
	return -INT_MAX;
}

#ifdef CONFIG_CGROUP_SCHEDTUNE

/*
 * EAS scheduler tunables for task groups.
 */

/* SchdTune tunables for a group of tasks */
struct schedtune {
	/* SchedTune CGroup subsystem */
	struct cgroup_subsys_state css;

	/* Boost group allocated ID */
	int idx;

	/* Boost value for tasks on that SchedTune CGroup */
	int boost;

	/* Performance Boost (B) region threshold params */
	int perf_boost_idx;

	/* Performance Constraint (C) region threshold params */
	int perf_constrain_idx;
};

static inline struct schedtune *css_st(struct cgroup_subsys_state *css)
{
	return css ? container_of(css, struct schedtune, css) : NULL;
}

static inline struct schedtune *cgroup_st(struct cgroup *cgrp) {
	return container_of(cgroup_subsys_state(cgrp, schedtune_subsys_id), struct schedtune, css);
}

static inline struct schedtune *task_schedtune(struct task_struct *tsk)
{
	return cgroup_st(task_cgroup(tsk, schedtune_subsys_id));
}

#if 0
static inline struct schedtune *parent_st(struct schedtune *st)
{
	return css_st(st->css.parent);
}
#endif

/*
 * SchedTune root control group
 * The root control group is used to defined a system-wide boosting tuning,
 * which is applied to all tasks in the system.
 * Task specific boosting tuning could be specified by creating and
 * configuring a child control group under the root one.
 * By default, system-wide boosting is disabled, i.e. no boosting is applied
 * to all tasks not into a child control group.
 */
static struct schedtune
root_schedtune = {
	.boost			= 0,
	.perf_boost_idx 	= 0,
	.perf_constrain_idx 	= 0,
};

int
schedtune_accept_deltas(int nrg_delta, int cap_delta, struct task_struct *task) {
	struct schedtune *ct;
	int perf_boost_idx;
	int perf_constrain_idx;

	/* Optimal (O) region */
	if (nrg_delta < 0 && cap_delta > 0) {
		// TJK: fix tracing: trace_sched_tune_filter(0, 0, 1, 0);
		return INT_MAX;
	}

	/* Suboptimal (S) region */
	if (nrg_delta > 0 && cap_delta < 0) {
		// TJK: fix tracing trace_sched_tune_filter(0, 0, -1, 5);
		return -INT_MAX;
	}

	/* Get task specific perf Boost/Constraints indexes */
	rcu_read_lock();
	ct = task_schedtune(task);
	perf_boost_idx = ct->perf_boost_idx;
	perf_constrain_idx = ct->perf_constrain_idx;
	rcu_read_unlock();

	return __schedtune_accept_deltas(nrg_delta, cap_delta,
			perf_boost_idx, perf_constrain_idx);

}

/*
 * Maximum number of boost groups to support
 * When per-task boosting is used we still allows only limited number of
 * boost groups for two main reasons:
 * 1. on a real system we usually have only few classes of workloads which
 *    make sense to boost with different values (e.g. backgroud vs foreground
 *    tasks, interactive vs low-priority tasks)
 * 2. a limited number allows for a simpler and more memory/time efficient
 *    implementation especially for the computation of the per-CPU boost
 *    value
 */
#define BOOSTGROUPS_COUNT 16

/* Array of configured boostgroups */
static struct schedtune *allocated_group[BOOSTGROUPS_COUNT] = {
	&root_schedtune,
	NULL,
};

/* SchedTune boost groups
 * Each CPU in the system could be affected by multiple boost groups, for
 * example when a CPU has two RUNNABLE tasks beloging to two different boost
 * groups and thus likely with different boost values.
 * This data structure keep track of all the boost groups which could impact
 * on a CPU.
 * Since on each system we expect only a limited number of boost
 * groups, here we use a simple array to keep track of the metrics required to
 * compute the maximum per-CPU boosting value.
 */
struct boost_groups {
	/* Maximum boost value for all RUNNABLE tasks on a CPU */
	bool idle;
	unsigned boost_max;
	struct {
		/* The boost for tasks on that boost group */
		unsigned boost;
		/* Count of RUNNABLE tasks on that boost group */
		unsigned tasks;
	} group[BOOSTGROUPS_COUNT];
};

/* Boost groups affecting each CPU in the system */
DEFINE_PER_CPU(struct boost_groups, cpu_boost_groups);

static void
schedtune_cpu_update(int cpu)
{
	struct boost_groups *bg;
	unsigned boost_max;
	int idx;

	bg = &per_cpu(cpu_boost_groups, cpu);

	/* The root boost group is always active */
	boost_max = bg->group[0].boost;
	for (idx = 1; idx < BOOSTGROUPS_COUNT; ++idx) {
		/*
		 * A boost group affects a CPU only if it has
		 * RUNNABLE tasks on that CPU
		 */
		if (bg->group[idx].tasks == 0)
			continue;

		boost_max = max(boost_max, bg->group[idx].boost);
	}

	bg->boost_max = boost_max;
}

static int
schedtune_boostgroup_update(int idx, int boost)
{
	struct boost_groups *bg;
	int cur_boost_max;
	int old_boost;
	int cpu;

	/* Update per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);

		/*
		 * Keep track of current boost values to compute the per CPU
		 * maximum only when it has been affected by the new value of
		 * the updated boost group
		 */
		cur_boost_max = bg->boost_max;
		old_boost = bg->group[idx].boost;

		/* Update the boost value of this boost group */
		bg->group[idx].boost = boost;

		/* Check if this update increase current max */
		if (boost > cur_boost_max && bg->group[idx].tasks) {
			bg->boost_max = boost;
			// TJK: fix tracing: trace_sched_tune_boostgroup_update(cpu, 1, bg->boost_max);
			continue;
		}

		/* Check if this update has decreased current max */
		if (cur_boost_max == old_boost && old_boost > boost) {
			schedtune_cpu_update(cpu);
			// TJK: fix tracing: trace_sched_tune_boostgroup_update(cpu, -1, bg->boost_max);
			continue;
		}

		//TJK: fix tracing: trace_sched_tune_boostgroup_update(cpu, 0, bg->boost_max);
	}

	return 0;
}

void
schedtune_idle(int cpu)
{
	int idx;
	struct boost_groups *bg;

	bg = &per_cpu(cpu_boost_groups, cpu);
	if (!bg->idle) {
		bg->idle = true;
		for (idx = 1; idx < BOOSTGROUPS_COUNT; ++idx) {
			bg->group[idx].tasks = 0;
		}
	}
}

static inline void
schedtune_tasks_update(struct task_struct *p, int cpu, int idx, int task_count)
{
	struct boost_groups *bg;
	int tasks;

	bg = &per_cpu(cpu_boost_groups, cpu);
	bg->idle = false;

	/* Update boosted tasks count while avoiding to make it negative */
	if (task_count < 0 && bg->group[idx].tasks <= -task_count)
		bg->group[idx].tasks = 0;
	else
		bg->group[idx].tasks += task_count;

	/* Boost group activation or deactivation on that RQ */
	tasks = bg->group[idx].tasks;
	if (tasks == 1 || tasks == 0)
		schedtune_cpu_update(cpu);

#if 0
	// TJK: fix tracing
	trace_sched_tune_tasks_update(p, cpu, tasks, idx,
			bg->group[idx].boost, bg->boost_max);
#endif

}

/*
 * NOTE: This function must be called while holding the lock on the CPU RQ
 */
void schedtune_enqueue_task(struct task_struct *p, int cpu)
{
	struct schedtune *st;
	int idx;

	/*
	 * When a task is marked PF_EXITING by do_exit() it's going to be
	 * dequeued and enqueued multiple times in the exit path.
	 * Thus we avoid any further update, since we do not want to change
	 * CPU boosting while the task is exiting.
	 */
	if (p->flags & PF_EXITING)
		return;

	/* Get task boost group */
	rcu_read_lock();
	st = task_schedtune(p);
	idx = st->idx;
	rcu_read_unlock();

	schedtune_tasks_update(p, cpu, idx, 1);
}

/*
 * NOTE: This function must be called while holding the lock on the CPU RQ
 */
void schedtune_dequeue_task(struct task_struct *p, int cpu)
{
	struct schedtune *st;
	int idx;

	/*
	 * When a task is marked PF_EXITING by do_exit() it's going to be
	 * dequeued and enqueued multiple times in the exit path.
	 * Thus we avoid any further update, since we do not want to change
	 * CPU boosting while the task is exiting.
	 * The last dequeue will be done by cgroup exit() callback.
	 */
	if (p->flags & PF_EXITING)
		return;

	/* Get task boost group */
	rcu_read_lock();
	st = task_schedtune(p);
	idx = st->idx;
	rcu_read_unlock();

	schedtune_tasks_update(p, cpu, idx, -1);
}

int schedtune_taskgroup_boost(struct task_struct *p)
{
	struct schedtune *ct;
	int task_boost;

	rcu_read_lock();
	ct = task_schedtune(p);
	task_boost = ct->boost;
	rcu_read_unlock();

	return task_boost;
}

int schedtune_cpu_boost(int cpu)
{
	struct boost_groups *bg;
	bg = &per_cpu(cpu_boost_groups, cpu);
	return 	bg->boost_max;
}

static u64
boost_read(struct cgroup *cgrp, struct cftype *cft)
{
	struct schedtune *st = cgroup_st(cgrp);
	return st->boost;
}

static int
boost_write(struct cgroup *cgrp, struct cftype *cft,
			  u64 boost)
{
	struct schedtune *st = cgroup_st(cgrp);
	int err = 0;

	if (boost < 0 || boost > 100) {
		err = -EINVAL;
		goto out;
	}

	st->boost = boost;
	if (cgrp == (void *)&root_schedtune.css)
		sysctl_sched_cfs_boost = boost;

	if (boost == 100)
		boost = 99;

	/* Performance Boost (B) region threshold params */
	st->perf_boost_idx  = boost;
	st->perf_boost_idx /= 10;

	/* Performance Constraint (C) region threshold params */
	st->perf_constrain_idx  = 100 - boost;
	st->perf_constrain_idx /= 10;

	/* Update CPU boost */
	schedtune_boostgroup_update(st->idx, st->boost);
#if 0
	// TJK: fix tracing
	trace_sched_tune_config(st->boost,
			threshold_gains[st->perf_boost_idx].nrg_gain,
			threshold_gains[st->perf_boost_idx].cap_gain,
			threshold_gains[st->perf_constrain_idx].nrg_gain,
			threshold_gains[st->perf_constrain_idx].cap_gain);
#endif
out:
	return err;
}

static struct cftype files[] = {
	{
		.name = "boost",
		.read_u64 = boost_read,
		.write_u64 = boost_write,
	},
	{ }	/* terminate */
};

static int
schedtune_boostgroup_init(struct schedtune *st)
{
	struct boost_groups *bg;
	int cpu;

	/* Keep track of allocated boost group */
	allocated_group[st->idx] = st;

	/* Initialize the per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);
		bg->group[st->idx].boost = 0;
		bg->group[st->idx].tasks = 0;
	}

	return 0;
}

static int
schedtune_init(void)
{
	struct boost_groups *bg;
	int cpu;

	/* Initialize the per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);
		memset(bg, 0 , sizeof(struct boost_groups));
	}

	pr_info("  schedtune configured to support %d boost groups\n",
			BOOSTGROUPS_COUNT);
	return 0;
}

static struct cgroup_subsys_state *
schedtune_css_alloc(struct cgroup *cgrp)
{
	struct schedtune *st;
	int idx;

	if (!cgrp->parent) {
		schedtune_init();
		return &root_schedtune.css;
	}

	/* Allows only single level hierachies */
#if 0
	if (cgrp->parent != (void *)&root_schedtune.css) {
		pr_err("Nested SchedTune boosting groups not allowed\n");
		return ERR_PTR(-ENOMEM);
	}
#endif
	/* Allows only a limited number of boosting groups */
	for (idx = 1; idx < BOOSTGROUPS_COUNT; ++idx)
		if (allocated_group[idx] == NULL)
			break;
	if (idx == BOOSTGROUPS_COUNT) {
		pr_err("Trying to create more than %d SchedTune boosting groups\n",
				BOOSTGROUPS_COUNT);
		return ERR_PTR(-ENOSPC);
	}

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		goto out;

	/* Initialize per CPUs boost group support */
	st->idx = idx;
	if (schedtune_boostgroup_init(st))
		goto release;

	return &st->css;

release:
	kfree(st);
out:
	return ERR_PTR(-ENOMEM);
}

static void
schedtune_boostgroup_release(struct schedtune *st)
{
	/* Reset this group boost */
	schedtune_boostgroup_update(st->idx, 0);

	/* Keep track of allocated boost group */
	allocated_group[st->idx] = NULL;
}

static void
schedtune_css_free(struct cgroup *cgrp)
{
	struct schedtune *st = cgroup_st(cgrp);
	schedtune_boostgroup_release(st);
	kfree(st);
}

static void
schedtune_exit(struct cgroup *cgrp,
		struct cgroup *old_css,
		struct task_struct *tsk)
{
	struct schedtune *old_st = cgroup_st(old_css);
	int cpu = task_cpu(tsk);

	schedtune_tasks_update(tsk, cpu, old_st->idx, -1);
}

static int
schedtune_allow_attach(struct cgroup *cgrp, struct cgroup_taskset *tset) {
	// TJK: too permissive, but ok for now
	return 0;
}

struct cgroup_subsys schedtune_subsys = {
	.name           = "schedtune",
	.css_alloc	= schedtune_css_alloc,
	.css_free	= schedtune_css_free,
	.exit		= schedtune_exit,
	.allow_attach	= schedtune_allow_attach,
	.base_cftypes	= files,
	.subsys_id      = schedtune_subsys_id,
	.early_init	= 1,
};

#else /* CONFIG_CGROUP_SCHEDTUNE */

int
schedtune_accept_deltas(int nrg_delta, int cap_delta) {

	/* Optimal (O) region */
	if (nrg_delta < 0 && cap_delta > 0) {
		trace_printk("schedtune_filter: region=O ngain=0 pgain=0 nrg_payoff=-1");
		return INT_MAX;
	}

	/* Suboptimal (S) region */
	if (nrg_delta > 0 && cap_delta < 0) {
		trace_printk("schedtune_filter: region=S ngain=0 pgain=0 nrg_payoff=1");
		return -INT_MAX;
	}

	return __schedtune_accept_deltas(nrg_delta, cap_delta,
			perf_boost_idx, perf_constrain_idx);

}

#endif /* CONFIG_CGROUP_SCHEDTUNE */

int
sysctl_sched_cfs_boost_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	/* Performance Boost (B) region threshold params */
	perf_boost_idx  = sysctl_sched_cfs_boost;
	perf_boost_idx /= 10;

	/* Performance Constraint (C) region threshold params */
	perf_constrain_idx  = 100 - sysctl_sched_cfs_boost;
	perf_constrain_idx /= 10;

	return 0;
}

