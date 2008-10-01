/* arch/arm/mach-msm/htc_pwrsink.c
 *
 * Copyright (C) 2008 HTC Corporation
 * Copyright (C) 2008 Google, Inc.
 * Author: San Mehat <san@google.com>
 *         Kant Kang <kant_kang@htc.com>
 *         Eiven Peng <eiven_peng@htc.com>
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/earlysuspend.h>
#include <mach/msm_smd.h>
#include <mach/trout_pwrsink.h>

#include "smd_private.h"

static int initialized = 0;
static struct pwr_sink *sink_array[PWRSINK_LAST + 1];
static DEFINE_SPINLOCK(sink_lock);
static unsigned long total_sink;
static uint32_t *smem_total_sink;

int trout_pwrsink_set(pwrsink_id_type id, unsigned percent_utilized)
{
	unsigned long flags;

	if (!smem_total_sink)
		smem_total_sink = smem_alloc(SMEM_ID_VENDOR0, sizeof(uint32_t));

	if (!initialized)
		return -EAGAIN;

	if (id < 0 || id > PWRSINK_LAST)
		return -EINVAL;

	spin_lock_irqsave(&sink_lock, flags);

	if (!sink_array[id]) {
		spin_unlock_irqrestore(&sink_lock, flags);
		return -ENOENT;
	}

	if (sink_array[id]->percent_util == percent_utilized) {
		spin_unlock_irqrestore(&sink_lock, flags);
		return 0;
	}

	total_sink -= (sink_array[id]->ua_max *
		       sink_array[id]->percent_util / 100);
	sink_array[id]->percent_util = percent_utilized;
	total_sink += (sink_array[id]->ua_max *
		       sink_array[id]->percent_util / 100);

	if (smem_total_sink)
		*smem_total_sink = total_sink / 1000;

	pr_debug("trout_pwrsink: ID %d, Util %d%%, Total %lu uA %s\n",
		 id, percent_utilized, total_sink,
		 smem_total_sink ? "SET" : "");

	spin_unlock_irqrestore(&sink_lock, flags);

	return 0;
}
EXPORT_SYMBOL(trout_pwrsink_set);

void trout_pwrsink_suspend_early(struct early_suspend *h)
{
	trout_pwrsink_set(PWRSINK_SYSTEM_LOAD, 70);
}

int trout_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	trout_pwrsink_set(PWRSINK_SYSTEM_LOAD, 13);
	return 0;
}

int trout_pwrsink_resume_early(struct platform_device *pdev)
{
	trout_pwrsink_set(PWRSINK_SYSTEM_LOAD, 70);
	return 0;
}

void trout_pwrsink_resume_late(struct early_suspend *h)
{
	trout_pwrsink_set(PWRSINK_SYSTEM_LOAD, 100);
}

struct early_suspend trout_pwrsink_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
	.suspend = trout_pwrsink_suspend_early,
	.resume = trout_pwrsink_resume_late,
};

static int __init trout_pwrsink_probe(struct platform_device *pdev)
{
	struct pwr_sink_platform_data *pdata = pdev->dev.platform_data;
	int i;
	
	if (!pdata)
		return -EINVAL;

	total_sink = 0;
	for (i = 0; i < pdata->num_sinks; i++) {
		sink_array[pdata->sinks[i].id] = &pdata->sinks[i];
		total_sink += (pdata->sinks[i].ua_max * pdata->sinks[i].percent_util / 100);
	}

	initialized = 1;

	register_early_suspend(&trout_pwrsink_early_suspend);
	
	return 0;
}

static struct platform_driver trout_pwrsink_driver = {
	.probe = trout_pwrsink_probe,
	.suspend_late = trout_pwrsink_suspend_late,
	.resume_early = trout_pwrsink_resume_early,
	.driver = {
		.name = "trout_pwrsink",
		.owner = THIS_MODULE,
	},
};

static int __init trout_pwrsink_init(void)
{
	initialized = 0;
	memset(sink_array, 0, sizeof(sink_array));
	return platform_driver_register(&trout_pwrsink_driver);
}

module_init(trout_pwrsink_init);

