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

#ifdef CONFIG_FB_MSM_MDDI
static void mdp_dma_to_mddi(void *priv, uint32_t addr, uint32_t stride,
			    uint32_t width, uint32_t height, uint32_t x,
			    uint32_t y)
{
	struct mdp_info *mdp = priv;
	uint32_t dma2_cfg;
	uint16_t ld_param = 0; /* 0=PRIM, 1=SECD, 2=EXT */

	dma2_cfg = DMA_PACK_TIGHT |
		DMA_PACK_ALIGN_LSB |
		DMA_OUT_SEL_AHB |
		DMA_IBUF_NONCONTIGUOUS;

	dma2_cfg |= mdp->dma_format;
	dma2_cfg |= mdp->dma_pack_pattern;

	dma2_cfg |= DMA_OUT_SEL_MDDI;

	dma2_cfg |= DMA_MDDI_DMAOUT_LCD_SEL_PRIMARY;

	dma2_cfg |= DMA_DITHER_EN;

	/* 666 18BPP */
	dma2_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;

#ifdef CONFIG_MSM_MDP22
	/* setup size, address, and stride */
	mdp_writel(mdp, (height << 16) | (width),
		   MDP_CMD_DEBUG_ACCESS_BASE + 0x0184);
	mdp_writel(mdp, addr, MDP_CMD_DEBUG_ACCESS_BASE + 0x0188);
	mdp_writel(mdp, stride, MDP_CMD_DEBUG_ACCESS_BASE + 0x018C);

	/* set y & x offset and MDDI transaction parameters */
	mdp_writel(mdp, (y << 16) | (x), MDP_CMD_DEBUG_ACCESS_BASE + 0x0194);
	mdp_writel(mdp, ld_param, MDP_CMD_DEBUG_ACCESS_BASE + 0x01a0);
	mdp_writel(mdp, (MDDI_VDO_PACKET_DESC << 16) | MDDI_VDO_PACKET_PRIM,
		   MDP_CMD_DEBUG_ACCESS_BASE + 0x01a4);

	mdp_writel(mdp, dma2_cfg, MDP_CMD_DEBUG_ACCESS_BASE + 0x0180);

	/* start DMA2 */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0044);
#else
	/* setup size, address, and stride */
	mdp_writel(mdp, (height << 16) | (width), MDP_DMA_P_SIZE);
	mdp_writel(mdp, addr, MDP_DMA_P_IBUF_ADDR);
	mdp_writel(mdp, stride, MDP_DMA_P_IBUF_Y_STRIDE);

	/* set y & x offset and MDDI transaction parameters */
	mdp_writel(mdp, (y << 16) | (x), MDP_DMA_P_OUT_XY);
	mdp_writel(mdp, ld_param, MDP_MDDI_PARAM_WR_SEL);
	mdp_writel(mdp, (MDDI_VDO_PACKET_DESC << 16) | MDDI_VDO_PACKET_PRIM,
		   MDP_MDDI_PARAM);

	mdp_writel(mdp, 0x1, MDP_MDDI_DATA_XFR);
	mdp_writel(mdp, dma2_cfg, MDP_DMA_P_CONFIG);
	mdp_writel(mdp, 0, MDP_DMA_P_START);
#endif
}
#endif

int mdp_hw_init(struct mdp_info *mdp)
{
	int n;

#ifdef CONFIG_FB_MSM_MDDI
	n = mdp_out_if_register(&mdp->mdp_dev, MSM_MDDI_PMDH_INTERFACE, mdp,
				  MDP_DMA_P_DONE, mdp_dma_to_mddi);
	if (n)
		return n;
#endif

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
