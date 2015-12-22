/*
 *  Copyright (C)  2015 Michael Turquette <mturquette@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/percpu.h>
#include <linux/irq_work.h>

#include "sched.h"

#define THROTTLE_NSEC		20000000 /* 50ms default */

static DEFINE_PER_CPU(unsigned long, pcpu_capacity);
static DEFINE_PER_CPU(struct cpufreq_policy *, pcpu_policy);
static DEFINE_PER_CPU(int, governor_started);

/**
 * gov_data - per-policy data internal to the governor
 * @throttle: next throttling period expiry. Derived from throttle_nsec
 * @throttle_nsec: throttle period length in nanoseconds
 * @freq: new frequency stored in *_sched_update_cpu and used in *_sched_thread
 *
 * struct gov_data is the per-policy cpufreq_sched-specific data structure. A
 * per-policy instance of it is created when the cpufreq_sched governor receives
 * the CPUFREQ_GOV_START condition and a pointer to it exists in the gov_data
 * member of struct cpufreq_policy.
 *
 * Readers of this data must call down_read(policy->rwsem). Writers must
 * call down_write(policy->rwsem).
 */
struct gov_data {
	ktime_t throttle;
	unsigned int throttle_nsec;
	struct cpufreq_policy *policy;
	unsigned int freq;
	bool change_pending;
	struct list_head gov_list;
};

 /* worker thread for dvfs transition that may block/sleep */
static struct task_struct *freq_change_task;
static LIST_HEAD(sched_gov_list);
static struct mutex gov_list_lock;

/* irq_work: callback used to wake up worker thread */
static struct irq_work irq_work;

static void cpufreq_sched_try_driver_target(struct cpufreq_policy *policy, unsigned int freq)
{
	struct gov_data *gd;

	/* avoid race with cpufreq_sched_stop */
	if (!down_write_trylock(&policy->rwsem))
		return;

	gd = policy->governor_data;

	if (!gd)
		return;

	__cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_L);

	gd->throttle = ktime_add_ns(ktime_get(), gd->throttle_nsec);
	up_write(&policy->rwsem);
}

/*
 * we pass in struct cpufreq_policy. This is safe because changing out the
 * policy requires a call to __cpufreq_governor(policy, CPUFREQ_GOV_STOP),
 * which tears down all of the data structures and __cpufreq_governor(policy,
 * CPUFREQ_GOV_START) will do a full rebuild, including this kthread with the
 * new policy pointer
 */
static int cpufreq_sched_thread(void *data)
{
	struct sched_param param;
	int ret;

	param.sched_priority = 50;
	ret = sched_setscheduler_nocheck(current, SCHED_FIFO, &param);
	if (ret) {
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		do_exit(-EINVAL);
	} else {
		pr_debug("%s: kthread (%d) set to SCHED_FIFO\n",
				__func__, current->pid);
	}

	/* main loop of the frequency change kthread */
	do {
		struct cpufreq_policy *policy = NULL;
		unsigned int freq = 0;
		struct gov_data *gd = NULL;
		mutex_lock(&gov_list_lock);
		list_for_each_entry(gd, &sched_gov_list, gov_list) {
			if (gd->change_pending) {
				gd->change_pending = false;	
				freq = gd->freq;
				policy = gd->policy;
				break;
			}
		}
		mutex_unlock(&gov_list_lock);

		if (!policy) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
		} else {
			cpufreq_sched_try_driver_target(policy, freq);
		}

		if (kthread_should_stop())
			break;

	} while (!kthread_should_stop());

	do_exit(0);
}

static void cpufreq_sched_irq_work(struct irq_work *irq_work)
{
	wake_up_process(freq_change_task);
}

/**
 * cpufreq_sched_set_capacity - interface to scheduler for changing capacity values
 * @cpu: cpu whose capacity utilization has recently changed
 * @capacity: the new capacity requested by cpu
 *
 * cpufreq_sched_sched_capacity is an interface exposed to the scheduler so
 * that the scheduler may inform the governor of updates to capacity
 * utilization and make changes to cpu frequency. Currently this interface is
 * designed around PELT values in CFS. It can be expanded to other scheduling
 * classes in the future if needed.
 *
 * cpufreq_sched_set_capacity raises an IPI. The irq_work handler for that IPI
 * wakes up the thread that does the actual work, cpufreq_sched_thread.
 *
 * This functions bails out early if either condition is true:
 * 1) this cpu did not the new maximum capacity for its frequency domain
 * 2) no change in cpu frequency is necessary to meet the new capacity request
 */
void cpufreq_sched_set_cap(int cpu, unsigned long capacity)
{
	unsigned int freq_new, cpu_tmp;
	struct cpufreq_policy *policy;
	struct gov_data *gd;
	unsigned long capacity_max = 0;

	if (!per_cpu(governor_started, cpu))
		return;

	/* update per-cpu capacity request */
	per_cpu(pcpu_capacity, cpu) = capacity;

	policy = cpufreq_cpu_get(cpu);
	if (IS_ERR_OR_NULL(policy)) {
		return;
	}

	if (!policy->governor_data)
		goto out;

	gd = policy->governor_data;

	/* bail early if we are throttled */
	if (ktime_compare(ktime_get(), gd->throttle) < 0)
		goto out;

	/* find max capacity requested by cpus in this policy */
	for_each_cpu(cpu_tmp, policy->cpus)
		capacity_max = max(capacity_max, per_cpu(pcpu_capacity, cpu_tmp));

	/*
	 * We only change frequency if this cpu's capacity request represents a
	 * new max. If another cpu has requested a capacity greater than the
	 * previous max then we rely on that cpu to hit this code path and make
	 * the change. IOW, the cpu with the new max capacity is responsible
	 * for setting the new capacity/frequency.
	 *
	 * If this cpu is not the new maximum then bail
	 */
	if (capacity_max > capacity)
		goto out;

	/* Convert the new maximum capacity request into a cpu frequency */
	freq_new = (capacity * policy->max) / capacity_orig_of(cpu);

	/* No change in frequency? Bail and return current capacity. */
	if (freq_new == policy->cur)
		goto out;

	/* store the new frequency and perform the transition */
	gd->freq = freq_new;
	gd->change_pending = true;

	irq_work_queue(&irq_work);

out:
	cpufreq_cpu_put(policy);
	return;
}

/**
 * cpufreq_sched_reset_capacity - interface to scheduler for resetting capacity
 *                                requests
 * @cpu: cpu whose capacity request has to be reset
 *
 * This _wont trigger_ any capacity update.
 */
void cpufreq_sched_reset_cap(int cpu)
{
	per_cpu(pcpu_capacity, cpu) = 0;
}

static inline void set_sched_energy_freq(void)
{
	// TJK: must always bump the key so we can handle
	// 1 cluster going online/offline
	static_key_slow_inc(&__sched_energy_freq);
}

static inline void clear_sched_energy_freq(void)
{
	// TJK: must always bump the key so we can handle
	// 1 cluster going online/offline
	static_key_slow_dec(&__sched_energy_freq);
}

static int cpufreq_sched_policy_start(struct cpufreq_policy *policy)
{
	int cpu;

	/* initialize per-cpu data */
	for_each_cpu(cpu, policy->cpus) {
		pr_info("%s: start CPU%d\n", __func__, cpu);
		per_cpu(pcpu_capacity, cpu) = 0;
		per_cpu(pcpu_policy, cpu) = policy;
	}
	return 0;
}

static int cpufreq_sched_policy_stop(struct cpufreq_policy *policy)
{
	int cpu;
	if (!policy) {
		return 0;
	}
	for_each_cpu(cpu, policy->cpus) {
		pr_info("%s: stop CPU%d\n", __func__, cpu);
	}
	/*
	 * Nothing to do. The per_cpu fields will be re-initialized
	 * on the next START
	 */
	return 0;
}

static struct attribute_group sched_attr_group_gov_pol;
static struct attribute_group *get_sysfs_attr(void)
{
	return &sched_attr_group_gov_pol;
}

static int cpufreq_sched_policy_init(struct cpufreq_policy *policy)
{
	int rc;
	struct gov_data *gd;
	int cpu = cpumask_first(policy->cpus);

	if (cpu >= nr_cpu_ids) {
		return -EINVAL;
	}
	WARN_ON(policy->governor_data);
	/* prepare per-policy private data */
	gd = kzalloc(sizeof(*gd), GFP_KERNEL);
	pr_info("%s: init CPU%d cluster\n", __func__, cpu);

	if (!gd) {
		pr_debug("%s: failed to allocate private data\n", __func__);
		return -ENOMEM;
	}
	rc = sysfs_create_group(get_governor_parent_kobj(policy), get_sysfs_attr());
	if (rc) {
		pr_err("%s: couldn't create sysfs attributes: %d\n", __func__, rc);
		goto err;
	}

	/*
	 * Don't ask for freq changes at an higher rate than what
	 * the driver advertises as transition latency.
	 */
#ifdef THROTTLE_FROM_CPUFREQ_DRIVER_LATENCY
	gd->throttle_nsec = policy->cpuinfo.transition_latency ?
			    policy->cpuinfo.transition_latency :
			    THROTTLE_NSEC;
#else
	gd->throttle_nsec = THROTTLE_NSEC;
#endif
	pr_debug("%s: throttle threshold = %u [ns]\n",
		  __func__, gd->throttle_nsec);

	policy->governor_data = gd;
	gd->policy = policy;

	mutex_lock(&gov_list_lock);
	list_add_tail(&gd->gov_list, &sched_gov_list);
	mutex_unlock(&gov_list_lock);

	set_sched_energy_freq();

	for_each_cpu(cpu, policy->cpus)
		per_cpu(governor_started, cpu) = 1;

	return 0;

err:
	kfree(gd);
	return -ENOMEM;
}

static int cpufreq_sched_policy_exit(struct cpufreq_policy *policy)
{
	struct gov_data *gd = policy->governor_data;
	int cpu = cpumask_first(policy->cpus);
	pr_info("%s: exit CPU%d cluster\n", __func__, cpu);
	WARN_ON(!policy->governor_data);

	for_each_cpu(cpu, policy->cpus)
		per_cpu(governor_started, cpu) = 0;

	clear_sched_energy_freq();

	policy->governor_data = NULL;
	mutex_lock(&gov_list_lock);
	list_del(&gd->gov_list);
	mutex_unlock(&gov_list_lock);

	sysfs_remove_group(get_governor_parent_kobj(policy), get_sysfs_attr());
	
	/* FIXME replace with devm counterparts? */
	kfree(gd);
	return 0;
}

static int cpufreq_sched_setup(struct cpufreq_policy *policy, unsigned int event)
{
	switch (event) {
		case CPUFREQ_GOV_START:
			return cpufreq_sched_policy_start(policy);
		case CPUFREQ_GOV_STOP:
			return cpufreq_sched_policy_stop(policy);
		case CPUFREQ_GOV_POLICY_INIT:
			return cpufreq_sched_policy_init(policy);
		case CPUFREQ_GOV_POLICY_EXIT:
			return cpufreq_sched_policy_exit(policy);
		case CPUFREQ_GOV_LIMITS:	/* unused */
			break;
	}
	return 0;
}

/* Tunables */
static ssize_t show_throttle_ns(struct gov_data *gd, char *buf)
{
	return sprintf(buf, "%u\n", gd->throttle_nsec);
}

static ssize_t store_throttle_ns(struct gov_data *gd,
		const char *buf, size_t count)
{
	int ret;
	long unsigned int val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	gd->throttle_nsec = val;
	return count;
}
/*
 * Create show/store routines
 * - sys: One governor instance for complete SYSTEM
 * - pol: One governor instance per struct cpufreq_policy
 */
#define show_gov_pol_sys(file_name)					\
static ssize_t show_##file_name##_gov_pol				\
(struct cpufreq_policy *policy, char *buf)				\
{									\
	return show_##file_name(policy->governor_data, buf);		\
}

#define store_gov_pol_sys(file_name)					\
static ssize_t store_##file_name##_gov_pol				\
(struct cpufreq_policy *policy, const char *buf, size_t count)		\
{									\
	return store_##file_name(policy->governor_data, buf, count);	\
}

#define gov_pol_attr_rw(_name)						\
	static struct freq_attr _name##_gov_pol =				\
	__ATTR(_name, 0644, show_##_name##_gov_pol, store_##_name##_gov_pol)

#define show_store_gov_pol_sys(file_name)				\
	show_gov_pol_sys(file_name);						\
	store_gov_pol_sys(file_name)
#define tunable_handlers(file_name) \
	show_gov_pol_sys(file_name); \
	store_gov_pol_sys(file_name); \
	gov_pol_attr_rw(file_name)

tunable_handlers(throttle_ns);

/* Per policy governor instance */
static struct attribute *sched_attributes_gov_pol[] = {
	&throttle_ns_gov_pol.attr,
	NULL,
};

static struct attribute_group sched_attr_group_gov_pol = {
	.attrs = sched_attributes_gov_pol,
	.name = "sched",
};

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHED
static
#endif
struct cpufreq_governor cpufreq_gov_sched = {
	.name			= "sched",
	.governor		= cpufreq_sched_setup,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_sched_init(void)
{
	int cpu;

	for_each_cpu(cpu, cpu_possible_mask) {
		per_cpu(governor_started, cpu) = 0;
	}
	mutex_init(&gov_list_lock);
	freq_change_task = kthread_run(cpufreq_sched_thread, NULL, "kcpufreq_sched_task");
	if (IS_ERR_OR_NULL(freq_change_task)) {
			pr_err("%s: failed to create kcpufreq_sched_task thread\n", __func__);
		return -ENOMEM;
	}
	init_irq_work(&irq_work, cpufreq_sched_irq_work);
	return cpufreq_register_governor(&cpufreq_gov_sched);
}

static void __exit cpufreq_sched_exit(void)
{
	kthread_stop(freq_change_task);
	cpufreq_unregister_governor(&cpufreq_gov_sched);
}

/* Try to make this the default governor */
fs_initcall(cpufreq_sched_init);

MODULE_LICENSE("GPL v2");
