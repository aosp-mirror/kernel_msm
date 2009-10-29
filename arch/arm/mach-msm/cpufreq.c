/* arch/arm/mach-msm/cpufreq.c
 *
 * MSM architecture cpufreq driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 * Author: Mike A. Chan <mikechan@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/cpufreq.h>
#include <linux/earlysuspend.h>
#include <linux/init.h>
#include "acpuclock.h"

#ifdef CONFIG_MSM_CPU_FREQ_SCREEN
static void msm_early_suspend(struct early_suspend *handler) {
	acpuclk_set_rate(CONFIG_MSM_CPU_FREQ_SCREEN_OFF * 1000, 0);
}

static void msm_late_resume(struct early_suspend *handler) {
	acpuclk_set_rate(CONFIG_MSM_CPU_FREQ_SCREEN_ON * 1000, 0);
}

static struct early_suspend msm_power_suspend = {
	.suspend = msm_early_suspend,
	.resume = msm_late_resume,
};

static int __init clock_late_init(void)
{
	register_early_suspend(&msm_power_suspend);
	return 0;
}

late_initcall(clock_late_init);
#else

static int msm_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	int index;
	struct cpufreq_freqs freqs;
	struct cpufreq_frequency_table *table =
		cpufreq_frequency_get_table(smp_processor_id());

	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
			&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		return -EINVAL;
	}

	if (policy->cur == table[index].frequency)
		return 0;

#ifdef CONFIG_CPU_FREQ_DEBUG
	printk("msm_cpufreq_target %d r %d (%d-%d) selected %d\n", target_freq,
		relation, policy->min, policy->max, table[index].frequency);
#endif
	freqs.old = policy->cur;
	freqs.new = table[index].frequency;
	freqs.cpu = smp_processor_id();
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	acpuclk_set_rate(table[index].frequency * 1000, 0);
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	return 0;
}

static int msm_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
			policy->cpuinfo.max_freq);
	return 0;
}

static int __init msm_cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *table =
		cpufreq_frequency_get_table(smp_processor_id());

	BUG_ON(cpufreq_frequency_table_cpuinfo(policy, table));
	policy->cur = acpuclk_get_rate();
	policy->cpuinfo.transition_latency =
		acpuclk_get_switch_time() * NSEC_PER_USEC;
	return 0;
}

static struct freq_attr *msm_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver msm_cpufreq_driver = {
	/* lps calculations are handled here. */
	.flags		= CPUFREQ_STICKY | CPUFREQ_CONST_LOOPS,
	.init		= msm_cpufreq_init,
	.verify		= msm_cpufreq_verify,
	.target		= msm_cpufreq_target,
	.name		= "msm",
	.attr		= msm_cpufreq_attr,
};

static int __init msm_cpufreq_register(void)
{
	return cpufreq_register_driver(&msm_cpufreq_driver);
}

device_initcall(msm_cpufreq_register);
#endif
