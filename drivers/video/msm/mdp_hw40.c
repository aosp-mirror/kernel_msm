/*
 * Copyright (C) 2010 Google, Inc.
 * Author: Dima Zavin <dima@android.com>
 *
 * Based on code from Code Aurora Forum.
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

#include <linux/io.h>

#include "mdp_hw.h"

int mdp_hw_init(struct mdp_info *mdp)
{
	mdp_writel(mdp, 0, MDP_INTR_ENABLE);
	mdp_writel(mdp, 0, MDP_DMA_P_HIST_INTR_ENABLE);

	/* TODO: Configure the VG/RGB pipes fetch data */

	/* this should work for any mdp_clk freq. 
	 * TODO: use different value for mdp_clk freqs >= 90Mhz */
	mdp_writel(mdp, 0x27, MDP_DMA_P_FETCH_CFG); /* 8 bytes-burst x 8 req */

	mdp_writel(mdp, 0x3, MDP_EBI2_PORTMAP_MODE);

	/* 3 pending requests */
	mdp_writel(mdp, 0x02222, MDP_MAX_RD_PENDING_CMD_CONFIG);

	/* no overlay processing, sw controls everything */
	mdp_writel(mdp, 0, MDP_LAYERMIXER_IN_CFG);
	mdp_writel(mdp, 1 << 3, MDP_OVERLAYPROC0_CFG);
	mdp_writel(mdp, 1 << 3, MDP_OVERLAYPROC1_CFG);

	/* XXX: HACK! hardcode to do mddi on primary */
	mdp_writel(mdp, 0x2, MDP_DISP_INTF_SEL);
	return 0;
}

