/*
 * Copyright (C) 2010 Google, Inc.
 * Author: Dima Zavin <dima@android.com>
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
#include "mdp_ppp.h"
#include "mdp_csc_table.h"

#define MDP_CMD_DEBUG_ACCESS_BASE (0x10000)

int mdp_hw_init(struct mdp_info *mdp)
{
	int n;

	mdp_writel(mdp, 0, MDP_INTR_ENABLE);

	/* debug interface write access */
	mdp_writel(mdp, 1, 0x60);
	mdp_writel(mdp, 1, MDP_EBI2_PORTMAP_MODE);

#ifndef CONFIG_MSM_MDP22
	/* disable lcdc */
	mdp_writel(mdp, 0, MDP_LCDC_EN);
	/* enable auto clock gating for all blocks by default */
	mdp_writel(mdp, 0xffffffff, MDP_CGC_EN);
	/* reset color/gamma correct parms */
	mdp_writel(mdp, 0, MDP_DMA_P_COLOR_CORRECT_CONFIG);
#endif

	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01f8);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01fc);
	mdp_writel(mdp, 1, 0x60);

	for (n = 0; n < ARRAY_SIZE(csc_color_lut); n++)
		mdp_writel(mdp, csc_color_lut[n].val, csc_color_lut[n].reg);

	/* clear up unused fg/main registers */
	/* comp.plane 2&3 ystride */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0120);

	/* unpacked pattern */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x012c);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0130);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0134);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0158);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x015c);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0160);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0170);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0174);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x017c);

	/* comp.plane 2 & 3 */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0114);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0118);

	/* clear unused bg registers */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01c8);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01d0);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01dc);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01e0);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01e4);

	for (n = 0; n < ARRAY_SIZE(csc_matrix_config_table); n++)
		mdp_writel(mdp, csc_matrix_config_table[n].val,
			   csc_matrix_config_table[n].reg);

	mdp_ppp_init_scale(mdp);

#ifndef CONFIG_MSM_MDP31
	mdp_writel(mdp, 0x04000400, MDP_COMMAND_CONFIG);
#endif

	return 0;
}
