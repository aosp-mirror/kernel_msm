/*
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/sort.h>
#include <mach/board.h>

#include "acpuclock.h"

unsigned long acpuclk_power_collapse(void)
{
	return 0;
}

unsigned long acpuclk_wait_for_irq(void)
{
	return 0;
}

unsigned long acpuclk_get_wfi_rate(void)
{
	return 0;
}

int acpuclk_set_rate(unsigned long rate, int for_power_collapse)
{
	return 0;
}

unsigned long acpuclk_get_rate(void)
{
	return 0;
}

uint32_t acpuclk_get_switch_time(void)
{
	return 0;
}

static void __init acpuclk_init(void)
{
}

void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *clkdata)
{
	pr_info("acpu_clock_init()\n");
	acpuclk_init();
}
