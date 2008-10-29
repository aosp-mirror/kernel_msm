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
