/* arch/arm/mach-msm/qdsp5v2/audio_glue.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/module.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

/* the audio codec control consists of
 * - mi2s transports (x3)
 * - lpa (low power audio) frontend for mi2s tx
 * - various related clocks
 */
struct msm_codec {
	void *tx_base;
	void *rx_base;
	void *lpa_base;

	struct clk *rx_mclk;
	struct clk *rx_sclk;
	struct clk *tx_mclk;
	struct clk *tx_sclk;

	struct clk *lpa_codec_clk;
	struct clk *lpa_core_clk;
	struct clk *lpa_pclk;

	struct clk *adsp_clk;
};

#define LPA_MAX_BUF_SIZE 		0x30000

#define LPA_CONTROL			0x00000000
#define LPA_CODEC			0x00000004
#define LPA_HLB_MIN_ADDR		0x00000008
#define LPA_HLB_MAX_ADDR		0x0000000C
#define LPA_HLB_WPTR			0x00000010
#define LPA_HLB_VOLUME_CONTROL 		0x00000014
#define LPA_LLB_MIN_ADDR		0x00000018
#define LPA_LLB_MAX_ADDR		0x0000001C
#define LPA_SB_MIN_ADDR			0x00000020
#define LPA_SB_MAX_ADDR			0x00000024
#define LPA_INTR_ENABLE			0x00000028
#define LPA_INTR_STATUS			0x0000002C
#define LPA_WMARK_ASSIGN		0x00000030
#define LPA_WMARK_0_LLB			0x00000034
#define LPA_WMARK_1_LLB			0x00000038
#define LPA_WMARK_2_LLB			0x0000003C
#define LPA_WMARK_3_LLB			0x00000040
#define LPA_WMARK_HLB			0x00000044
#define LPA_WMARK_SB			0x00000048
#define LPA_RDPTR_LLB			0x0000004C
#define LPA_RDPTR_HLB			0x00000050
#define LPA_WRPTR_SB			0x00000054
#define LPA_UTC_CONFIG			0x00000058
#define LPA_UTC_INTR_LOW		0x0000005C
#define LPA_UTC_INTR_HIGH		0x00000060
#define LPA_UTC_LOW			0x00000064
#define LPA_UTC_HIGH			0x00000068
#define LPA_MISR			0x0000006C
#define LPA_STATUS			0x00000070
#define LPA_ACK				0x00000074
#define LPA_MEMORY_CONTROL		0x00000078
#define LPA_MEMORY_STATUS		0x0000007C
#define LPA_MEMORY_TIME_CONTROL		0x00000080
#define LPA_ACC_LV			0x00000084
#define LPA_ACC_HV			0x0000008c
#define LPA_RESETS			0x00000090
#define LPA_TESTBUS			0x00000094

#define LPA_AICTL			0x00000100

/* OBUF_CODEC */
#define LPA_CODEC_LOAD			0x200000
#define LPA_CODEC_INTF_EN		0x100000
#define LPA_CODEC_CFG_MASK		0x0FC07F

#define LPA_SAMPLE_RATE_8KHZ		0x000000
#define LPA_SAMPLE_RATE_11P025KHZ	0x010000
#define LPA_SAMPLE_RATE_16KHZ		0x020000
#define LPA_SAMPLE_RATE_22P05KHZ	0x030000
#define LPA_SAMPLE_RATE_32KHZ		0x040000
#define LPA_SAMPLE_RATE_44P1KHZ		0x050000
#define LPA_SAMPLE_RATE_48KHZ		0x060000
#define LPA_SAMPLE_RATE_64KHZ		0x070000
#define LPA_SAMPLE_RATE_96KHZ		0x080000

#define LPA_BITS_PER_CHAN_16BITS	0x000000
#define LPA_BITS_PER_CHAN_24BITS	0x004000
#define LPA_BITS_PER_CHAN_32BITS	0x008000
#define LPA_BITS_PER_CHAN_RESERVED	0x00C000

#define LPA_INTF_SDAC			0x000010
#define LPA_INTF_MI2S			0x000020
#define LPA_INTF_WB_CODEC		0x000030

/* WB_CODEC & SDAC can only support 16bit mono/stereo.
 * MI2S can bit format and number of channel
 */
#define LPA_NUM_CHAN_MONO		0x000000
#define LPA_NUM_CHAN_STEREO		0x000001
#define LPA_NUM_CHAN_5P1		0x000002
#define LPA_NUM_CHAN_7P1		0x000003
#define LPA_NUM_CHAN_4_CHANNEL		0x000004

/* OBUF_CONTROL */
#define LPA_CONTROL_TEST_EN		0x100
#define LPA_CONTROL_LLB_CLR_CMD		0x080
#define LPA_CONTROL_SB_SAT_EN		0x040
#define LPA_CONTROL_LLB_SAT_EN		0x020
#define LPA_CONTROL_LLB_ACC_EN		0x008
#define LPA_CONTROL_HLB_EN		0x004
#define LPA_CONTROL_LLB_EN		0x002
#define LPA_CONTROL_SB_EN		0x001

/* OBUF_RESET definition */
#define LPA_RESETS_MISR			0x1
#define LPA_RESETS_OVERALL		0x2

/* OBUF_STATUS definition */
#define LPA_STATUS_RESET_DONE		0x80000
#define LPA_STATUS_LLB_CLR		0x40000

/* OBUF_HLB_MIN_ADDR definition */
#define LPA_HLB_MIN_ADDR_LOAD		0x40000
#define LPA_HLB_MIN_ADDR_SEG_MASK	0x3e000

/* OBUF_HLB_MAX_ADDR definition */
#define LPA_HLB_MAX_ADDR_SEG_MASK	0x3fff8

/* OBUF_LLB_MIN_ADDR definition */
#define LPA_LLB_MIN_ADDR_LOAD		0x40000
#define LPA_LLB_MIN_ADDR_SEG_BMSK	0x3e000

/* OBUF_LLB_MAX_ADDR definition */
#define LPA_LLB_MAX_ADDR_SEG_MASK	0x3ff8
#define LPA_LLB_MAX_ADDR_SEG_SHFT	0x3

/* OBUF_SB_MIN_ADDR definition */
#define LPA_SB_MIN_ADDR_LOAD		0x4000
#define LPA_SB_MIN_ADDR_SEG_BMSK	0x3e00

/* OBUF_SB_MAX_ADDR definition */
#define LPA_SB_MAX_ADDR_SEG_BMSK	0x3ff8

/* OBUF_MEMORY_CONTROL definition */
#define LPA_MEM_CTL_PWRUP		0xfff

/* OBUF_INTR_ENABLE definition */
#define LPA_INTR_EN			0x3

/* OBUF_WMARK_ASSIGN definition */
#define LPA_WMARK_ASSIGN_BMSK		0xF
#define LPA_WMARK_ASSIGN_DONE		0xF

/* OBUF_WMARK_n_LLB definition */
#define LPA_WMARK_n_LLB_ADDR(n)  (0x00000034 + 0x4 * (n))
#define LPA_WMARK_CTRL_MASK		0x0c0000
#define LPA_WMARK_CTRL_SHFT		0x12
#define LPA_WMARK_MAP_MASK		0xf00000
#define LPA_WMARK_MAP_SHFT		0x14

#define LPA_WMARK_CTL_DISABLED		0x0
#define LPA_WMARK_CTL_NON_BLOCK		0x1
#define LPA_WMARK_CTL_ZERO_INSERT	0x2
#define LPA_WMARK_CTL_RESERVED		0x3

/* OBUF_UTC_CONFIG definition */
#define LPA_UTC_CONFIG_MAP_MASK		0xf0
#define LPA_UTC_CONFIG_MAP_SHFT		0x4
#define LPA_UTC_CONFIG_EN		0x1
#define LPA_UTC_CONFIG_NO_INTR		0xF

/* OBUF_ACK definition */
#define LPA_ACK_RESET_DONE		0x80000


#define LPA_BUF_ID_HLB	0   /* HLB buffer */
#define LPA_BUF_ID_LLB	1   /* LLB buffer */
#define LPA_BUF_ID_SB	2   /* SB buffer */
#define LPA_BUF_ID_UTC	3


/* from board file in qct tree */

#define LPA_HLB_SIZE		0x2BFF8

#define LPA_ID_DSP		0
#define LPA_ID_APP		2

#if 0
#define CFG_LLB_MIN_ADDR	0x0000
#define CFG_LLB_MAX_ADDR	0x3ff8
#define CFG_SB_MIN_ADDR		0
#define CFG_SB_MAX_ADDR		0
#else
#define CFG_LLB_MIN_ADDR	0x0000
#define CFG_LLB_MAX_ADDR	0x37f8
#define CFG_SB_MIN_ADDR		0x3800
#define CFG_SB_MAX_ADDR		0x3ff8
#endif

#define CFG_HLB_MIN_ADDR	0x00000
#define CFG_HLB_MAX_ADDR	0x2BFF8

/* 7x30 MI2S Registers */

/* MI2S Registers are named from the MI2S block's point of view:
 * - TX = transmit from SoC to external codec
 * - RX = receive from external codec to SoC
 */
#define MI2S_RESET		0x00
#define MI2S_MODE		0x04
#define MI2S_TX_MODE		0x08
#define MI2S_RX_MODE		0x0C

#define MI2S_RESET_RESET	1

#define MI2S_MODE_MASTER	0x1000
#define MI2S_MODE_16BIT		0x0100
#define MI2S_MODE_24BIT		0x0200
#define MI2S_MODE_32BIT		0x0300
#define MI2S_MODE_EN_3		0x0080
#define MI2S_MODE_EN_2		0x0040
#define MI2S_MODE_EN_1		0x0020
#define MI2S_MODE_EN_0		0x0010
#define MI2S_MODE_TX_3		0x0008
#define MI2S_MODE_TX_2		0x0004
#define MI2S_MODE_TX_1		0x0002
#define MI2S_MODE_TX_0		0x0001

#define MI2S_TX_MODE_2CH	0x0000
#define MI2S_TX_MODE_4CH	0x0008
#define MI2S_TX_MODE_6CH	0x0010
#define MI2S_TX_MODE_8CH	0x0018
#define MI2S_TX_MODE_STEREO	0x0004
#define MI2S_TX_MODE_MONO_PACK	0x0002 /* 2 mono samples packed together */
#define MI2S_TX_MODE_DMA_SYNC	0x0001 /* sync dma ack clocks */

#define MI2S_RX_MODE_2CH	0x0000
#define MI2S_RX_MODE_4CH	0x0008
#define MI2S_RX_MODE_6CH	0x0010
#define MI2S_RX_MODE_8CH	0x0018
#define MI2S_RX_MODE_STEREO	0x0004
#define MI2S_RX_MODE_MONO_PACK	0x0002 /* 2 mono samples packed together */
#define MI2S_RX_MODE_DMA_SYNC	0x0001 /* sync dma ack clocks */

static int mi2s_set_output(struct msm_codec *mc,
			   unsigned channels, unsigned bitdepth)
{
	unsigned mode = 0;
	unsigned tx_mode = 0;

	if (channels != 2 || bitdepth != 16)
		return -EINVAL;

	/* TODO: support non stereo-16 (does the DSP even do that?) */

	mode |= MI2S_MODE_MASTER;
	mode |= MI2S_MODE_16BIT;
	mode |= MI2S_MODE_EN_0;
	mode |= MI2S_MODE_TX_0;

	tx_mode |= MI2S_TX_MODE_STEREO;
	tx_mode |= MI2S_TX_MODE_2CH;
	tx_mode |= MI2S_RX_MODE_DMA_SYNC;
	
	writel(1, mc->tx_base + MI2S_RESET);
	writel(mode, mc->tx_base + MI2S_MODE);
	writel(tx_mode, mc->tx_base + MI2S_TX_MODE);
	writel(0, mc->tx_base + MI2S_RESET);

	return 0;
}

static int mi2s_set_input(struct msm_codec *mc,
			  unsigned channels, unsigned bitdepth)
{
	unsigned mode = 0;
	unsigned rx_mode = 0;

	if (channels != 2 || bitdepth != 16)
		return -EINVAL;

	/* TODO: support non stereo-16 */
	/* TODO: packed mono mode? */

	mode |= MI2S_MODE_MASTER;
	mode |= MI2S_MODE_16BIT;
	mode |= MI2S_MODE_EN_0;

	rx_mode |= MI2S_RX_MODE_STEREO;
	rx_mode |= MI2S_RX_MODE_2CH;
	rx_mode |= MI2S_RX_MODE_DMA_SYNC;
	
	writel(1, mc->rx_base + MI2S_RESET);
	writel(mode, mc->rx_base + MI2S_MODE);
	writel(rx_mode, mc->rx_base + MI2S_RX_MODE);
	writel(0, mc->rx_base + MI2S_RESET);

	return 0;
}

void lpa_enable(struct msm_codec *mc)
{
	unsigned val;

	/* for "hardware reasons" we must ensure the
	 * adsp clock is on during this reset sequence.
	 */
	clk_enable(mc->adsp_clk);

	/* disable codec */
	writel(LPA_CODEC_LOAD, mc->lpa_base + LPA_CODEC);

	writel(LPA_RESETS_MISR | LPA_RESETS_OVERALL,
	       mc->lpa_base + LPA_RESETS);

	while (!(readl(mc->lpa_base + LPA_STATUS) & LPA_STATUS_RESET_DONE))
		;

	writel(LPA_ACK_RESET_DONE, mc->lpa_base + LPA_ACK);

	clk_disable(mc->adsp_clk);

	/* configure memory buffers */
	writel(CFG_LLB_MIN_ADDR | LPA_LLB_MIN_ADDR_LOAD,
	       mc->lpa_base + LPA_LLB_MIN_ADDR);
	writel(CFG_LLB_MAX_ADDR, mc->lpa_base + LPA_LLB_MAX_ADDR);

	writel(CFG_SB_MIN_ADDR | LPA_SB_MIN_ADDR_LOAD,
	       mc->lpa_base + LPA_SB_MIN_ADDR);
	writel(CFG_SB_MAX_ADDR, mc->lpa_base + LPA_SB_MAX_ADDR);

	writel(CFG_HLB_MIN_ADDR | LPA_HLB_MIN_ADDR_LOAD,
	       mc->lpa_base + LPA_HLB_MIN_ADDR);
	writel(CFG_HLB_MAX_ADDR, mc->lpa_base + LPA_HLB_MAX_ADDR);

	writel(LPA_MEM_CTL_PWRUP, mc->lpa_base + LPA_MEMORY_CONTROL);


	while (readl(mc->lpa_base + LPA_WMARK_ASSIGN) != LPA_WMARK_ASSIGN_DONE)
		;

	/* setup watermark ownership */
	writel(LPA_ID_DSP << LPA_WMARK_MAP_SHFT,
	       mc->lpa_base + LPA_WMARK_0_LLB);
	writel(LPA_ID_DSP << LPA_WMARK_MAP_SHFT,
	       mc->lpa_base + LPA_WMARK_1_LLB);
	writel(LPA_ID_APP << LPA_WMARK_MAP_SHFT,
	       mc->lpa_base + LPA_WMARK_2_LLB);
	writel(LPA_ID_APP << LPA_WMARK_MAP_SHFT,
	       mc->lpa_base + LPA_WMARK_3_LLB);
	writel(LPA_ID_DSP << LPA_WMARK_MAP_SHFT,
	       mc->lpa_base + LPA_WMARK_HLB);
	writel(LPA_ID_DSP << LPA_WMARK_MAP_SHFT,
	       mc->lpa_base + LPA_WMARK_SB);
	writel(0, mc->lpa_base + LPA_UTC_CONFIG);


	val = readl(mc->lpa_base + LPA_CONTROL);
	val |= LPA_CONTROL_LLB_EN;
	val |= LPA_CONTROL_LLB_SAT_EN;
	val |= LPA_CONTROL_SB_EN;
	val |= LPA_CONTROL_SB_SAT_EN;
	writel(val, mc->lpa_base + LPA_CONTROL);

	writel(1 << LPA_ID_DSP, mc->lpa_base + LPA_INTR_ENABLE);
}

void lpa_start(struct msm_codec *mc)
{
	unsigned val, codec;

	codec = LPA_CODEC_LOAD;
	codec |= LPA_NUM_CHAN_STEREO;
	codec |= LPA_SAMPLE_RATE_48KHZ;
	codec |= LPA_BITS_PER_CHAN_16BITS;
	codec |= LPA_INTF_WB_CODEC;
	writel(codec, mc->lpa_base + LPA_CODEC);

	/* clear LLB */
	val = readl(mc->lpa_base + LPA_CONTROL);
	writel(val | LPA_CONTROL_LLB_CLR_CMD, mc->lpa_base + LPA_CONTROL);

	while (!(readl(mc->lpa_base + LPA_STATUS) & LPA_STATUS_LLB_CLR))
		udelay(100);

	/* enable codec */
	codec |= LPA_CODEC_INTF_EN;
	writel(codec, mc->lpa_base + LPA_CODEC);
}

void lpa_disable(struct msm_codec *mc)
{
	writel(LPA_CODEC_LOAD, mc->lpa_base + LPA_CODEC);
}

int msm_codec_output_enable(struct msm_codec *mc)
{
	unsigned rate, val;

	pr_info("msm_codec_output_enable()\n");

	/* yes rx clks for tx codec -- the clocks
	 * are named from the opposite POV of the
	 * codec for some reason...
	 */


	/* bitrate * bits * channels * 8 */
	rate = 48000 * 16 * 2 * 8;
	clk_set_rate(mc->rx_mclk, rate);

	printk("RATE %d\n", clk_get_rate(mc->rx_mclk));

	clk_enable(mc->rx_mclk);
	clk_enable(mc->rx_sclk);

 	clk_enable(mc->lpa_pclk);
	clk_enable(mc->lpa_codec_clk);
	clk_enable(mc->lpa_core_clk);
	/* LPA init */

	lpa_enable(mc);

	/* interconnect reg -> LPA */
	val = readl(mc->lpa_base + LPA_AICTL);
	writel(val | 4, mc->lpa_base + LPA_AICTL);

	/* fire up mi2s transport */
	mi2s_set_output(mc, 2, 16);

	lpa_start(mc);

	/* AFE enable */

	/* ADIE enable */

	/* AMP enable */

	return 0;
}

int msm_codec_output_disable(struct msm_codec *mc)
{
	pr_info("msm_codec_output_disable()\n");
	/* AMP disable */
	/* ADIE disable */
	/* AFE disable */
	/* LPA disable */

	clk_disable(mc->lpa_core_clk);
	clk_disable(mc->lpa_codec_clk);
	clk_disable(mc->lpa_pclk);

	clk_disable(mc->rx_sclk);
	clk_disable(mc->rx_mclk);

	return 0;
}


static struct msm_codec the_msm_codec;

int msm_codec_output(int enable)
{
	struct msm_codec *mc = &the_msm_codec;
	if (enable)
		return msm_codec_output_enable(mc);
	else
		return msm_codec_output_disable(mc);
}

/* 7x30 memory map */

#define PHYS_ADDR_LPA		0xA5000000
#define PHYS_SIZE_LPA		0x00000800

#define PHYS_ADDR_MI2S_HDMI	0xAC900000
#define PHYS_ADDR_MI2S_CODEC_RX	0xAC940040
#define PHYS_ADDR_MI2S_CODEC_TX 0xAC980080
#define PHYS_SIZE_MI2S          0x00000040

int msm_codec_init(void)
{
	struct msm_codec *mc = &the_msm_codec;

	printk("msm_codec_init()\n");

	mc->rx_mclk = clk_get(NULL, "mi2s_codec_rx_mclk");
	if (IS_ERR(mc->rx_mclk))
		return -ENODEV;
	mc->rx_sclk = clk_get(NULL, "mi2s_codec_rx_sclk");
	if (IS_ERR(mc->rx_sclk))
		return -ENODEV;
	mc->tx_mclk = clk_get(NULL, "mi2s_codec_tx_mclk");
	if (IS_ERR(mc->tx_mclk))
		return -ENODEV;
	mc->tx_sclk = clk_get(NULL, "mi2s_codec_tx_sclk");
	if (IS_ERR(mc->tx_sclk))
		return -ENODEV;
	mc->lpa_codec_clk = clk_get(NULL, "lpa_codec_clk");
	if (IS_ERR(mc->lpa_codec_clk))
		return -ENODEV;
	mc->lpa_core_clk = clk_get(NULL, "lpa_core_clk");
	if (IS_ERR(mc->lpa_core_clk))
		return -ENODEV;
	mc->lpa_pclk = clk_get(NULL, "lpa_pclk");
	if (IS_ERR(mc->lpa_pclk))
		return -ENODEV;
	mc->adsp_clk = clk_get(NULL, "adsp_clk");
	if (IS_ERR(mc->adsp_clk))
		return -ENODEV;

	mc->lpa_base = ioremap(PHYS_ADDR_LPA, PHYS_SIZE_LPA);
	if (!mc->lpa_base)
		return -ENODEV;
	mc->rx_base = ioremap(PHYS_ADDR_MI2S_CODEC_RX, PHYS_SIZE_MI2S);
	if (!mc->rx_base)
		return -ENODEV;
	mc->tx_base = ioremap(PHYS_ADDR_MI2S_CODEC_TX, PHYS_SIZE_MI2S);
	if (!mc->tx_base)
		return -ENODEV;

	return 0;
}
