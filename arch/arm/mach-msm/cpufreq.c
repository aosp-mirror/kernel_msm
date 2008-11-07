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

#include <linux/earlysuspend.h>
#include <linux/init.h>
#include "acpuclock.h"

#ifdef CONFIG_MSM_CPU_FREQ_ONDEMAND
#include <linux/cpufreq.h>
#endif

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
#elif defined(CONFIG_MSM_CPU_FREQ_ONDEMAND)

static int msm_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	unsigned int freq;
	switch(relation) {
		/* Lowest value at or above target frequency. */
		case CPUFREQ_RELATION_L:
		/* Highest value at or below target frequency. */
		case CPUFREQ_RELATION_H:
			if (target_freq < CONFIG_MSM_CPU_FREQ_ONDEMAND_MAX)
				freq = CONFIG_MSM_CPU_FREQ_ONDEMAND_MIN;
			else
				freq = CONFIG_MSM_CPU_FREQ_ONDEMAND_MAX;
			break;
		default:
			return -EINVAL;
	}
#ifdef CONFIG_CPU_FREQ_DEBUG
	printk("msm_cpufreq_target %d r %d (%d-%d) selected %d\n", target_freq,
		relation, policy->min, policy->max, freq);
#endif
	acpuclk_set_rate(freq * 1000, 0);
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
	policy->cur = acpuclk_get_rate();
	policy->min = CONFIG_MSM_CPU_FREQ_ONDEMAND_MIN;
	policy->max = CONFIG_MSM_CPU_FREQ_ONDEMAND_MAX;
	policy->cpuinfo.min_freq = CONFIG_MSM_CPU_FREQ_ONDEMAND_MIN;
	policy->cpuinfo.max_freq = CONFIG_MSM_CPU_FREQ_ONDEMAND_MAX;
	policy->cpuinfo.transition_latency =
		acpuclk_get_switch_time() * NSEC_PER_USEC;
	return 0;
}

static struct cpufreq_driver msm_cpufreq_driver = {
	/* lps calculstaions are handled here. */
	.flags		= CPUFREQ_STICKY | CPUFREQ_CONST_LOOPS,
	.init		= msm_cpufreq_init,
	.verify		= msm_cpufreq_verify,
	.target		= msm_cpufreq_target,
	.name		= "msm",
};

static int __init msm_cpufreq_register(void)
{
	return cpufreq_register_driver(&msm_cpufreq_driver);
}

device_initcall(msm_cpufreq_register);
#endif
