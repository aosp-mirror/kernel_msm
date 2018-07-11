/*
 * Power management support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PAINTBOX_POWER_H__
#define __PAINTBOX_POWER_H__

#include <linux/io.h>

#include "paintbox-common.h"

void paintbox_enable_mmu_bif_idle_clock_gating(struct paintbox_data *pb);
void paintbox_disable_mmu_bif_idle_clock_gating(struct paintbox_data *pb);

/* The caller to these functions must hold pb->dma.dma_lock */
void paintbox_pm_enable_dma_channel(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);
void paintbox_pm_disable_dma_channel(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);

/* The caller to these functions must hold pb->lock */
void paintbox_pm_stp_enable(struct paintbox_data *pb, struct paintbox_stp *stp);
void paintbox_pm_lbp_enable(struct paintbox_data *pb, struct paintbox_lbp *lbp);
void paintbox_pm_stp_disable(struct paintbox_data *pb,
		struct paintbox_stp *stp);
void paintbox_pm_lbp_disable(struct paintbox_data *pb,
		struct paintbox_lbp *lbp);

int paintbox_pm_init(struct paintbox_data *pb);
void paintbox_pm_remove(struct paintbox_data *pb);

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_pm_dump_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
#else
static inline int paintbox_pm_dump_registers(struct paintbox_debug *debug,
		char *buf, size_t len)
{
	return 0;
}
#endif

#endif /* __PAINTBOX_POWER_H__ */
