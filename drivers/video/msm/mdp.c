/* drivers/video/msm_fb/mdp.c
 *
 * MSM MDP Interface (used by framebuffer core)
 *
 * Copyright (C) 2007 QUALCOMM Incorporated
 * Copyright (C) 2007 Google Incorporated
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

#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/file.h>
#include <linux/android_pmem.h>
#include <linux/major.h>
#include <linux/msm_hw3d.h>
#include <linux/slab.h>

#include <mach/msm_iomap.h>
#include <mach/msm_fb.h>
#include <linux/platform_device.h>

#include "mdp_hw.h"
#include "mdp_ppp.h"

struct class *mdp_class;

#define MDP_CMD_DEBUG_ACCESS_BASE (0x10000)

static DECLARE_WAIT_QUEUE_HEAD(mdp_ppp_waitqueue);
static unsigned int mdp_irq_mask;
struct clk *mdp_clk_to_disable_later = 0;
DEFINE_MUTEX(mdp_mutex);

static int locked_enable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	BUG_ON(!mask);

	/* if the mask bits are already set return an error, this interrupt
	 * is already enabled */
	if (mdp_irq_mask & mask) {
		pr_err("mdp irq already on %x %x\n", mdp_irq_mask, mask);
		return -1;
	}
	/* if the mdp irq is not already enabled enable it */
	if (!mdp_irq_mask) {
		clk_set_rate(mdp->ebi1_clk, 128000000);
		clk_enable(mdp->clk);
		enable_irq(mdp->irq);
	}

	/* clear out any previous irqs for the requested mask*/
	mdp_writel(mdp, mask, MDP_INTR_CLEAR);

	/* update the irq mask to reflect the fact that the interrupt is
	 * enabled */
	mdp_irq_mask |= mask;
	mdp_writel(mdp, mdp_irq_mask, MDP_INTR_ENABLE);
	return 0;
}

static int enable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&mdp->lock, flags);
	ret = locked_enable_mdp_irq(mdp, mask);
	spin_unlock_irqrestore(&mdp->lock, flags);
	return ret;
}

static int locked_disable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	/* this interrupt is already disabled! */
	if (!(mdp_irq_mask & mask)) {
		printk(KERN_ERR "mdp irq already off %x %x\n",
		       mdp_irq_mask, mask);
		return -1;
	}
	/* update the irq mask to reflect the fact that the interrupt is
	 * disabled */
	mdp_irq_mask &= ~(mask);
	mdp_writel(mdp, mdp_irq_mask, MDP_INTR_ENABLE);

	/* if no one is waiting on the interrupt, disable it */
	if (!mdp_irq_mask) {
		disable_irq_nosync(mdp->irq);
		if (mdp->clk)
			clk_disable(mdp->clk);
		clk_set_rate(mdp->ebi1_clk, 0);
	}
	return 0;
}

int disable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	unsigned long irq_flags;
	int ret;

	spin_lock_irqsave(&mdp->lock, irq_flags);
	ret = locked_disable_mdp_irq(mdp, mask);
	spin_unlock_irqrestore(&mdp->lock, irq_flags);
	return ret;
}

static irqreturn_t mdp_isr(int irq, void *data)
{
	uint32_t status;
	unsigned long irq_flags;
	struct mdp_info *mdp = data;
	int i;

	spin_lock_irqsave(&mdp->lock, irq_flags);

	status = mdp_readl(mdp, MDP_INTR_STATUS);
	mdp_writel(mdp, status, MDP_INTR_CLEAR);

//	pr_info("%s: status=%08x (irq_mask=%08x)\n", __func__, status,
//		mdp_irq_mask);
	status &= mdp_irq_mask;

	for (i = 0; i < MSM_MDP_NUM_INTERFACES; ++i) {
		struct mdp_out_interface *out_if = &mdp->out_if[i];
		if (status & out_if->dma_mask) {
			if (out_if->dma_cb) {
				out_if->dma_cb->func(out_if->dma_cb);
				out_if->dma_cb = NULL;
			}
			wake_up(&out_if->dma_waitqueue);
		}
		if (status & out_if->irq_mask) {
			out_if->irq_cb->func(out_if->irq_cb);
			out_if->irq_cb = NULL;
		}
	}

	if (status & DL0_ROI_DONE)
		wake_up(&mdp_ppp_waitqueue);

	if (status)
		locked_disable_mdp_irq(mdp, status);

	spin_unlock_irqrestore(&mdp->lock, irq_flags);
	return IRQ_HANDLED;
}

static uint32_t mdp_check_mask(struct mdp_info *mdp, uint32_t mask)
{
	uint32_t ret;
	unsigned long irq_flags;

	spin_lock_irqsave(&mdp->lock, irq_flags);
	ret = mdp_irq_mask & mask;
	spin_unlock_irqrestore(&mdp->lock, irq_flags);
	return ret;
}

static int mdp_wait(struct mdp_info *mdp, uint32_t mask, wait_queue_head_t *wq)
{
	int ret = 0;
	unsigned long irq_flags;

//	pr_info("%s: WAITING for 0x%x\n", __func__, mask);
	wait_event_timeout(*wq, !mdp_check_mask(mdp, mask), HZ);

	spin_lock_irqsave(&mdp->lock, irq_flags);
	if (mdp_irq_mask & mask) {
		pr_warning("%s: timeout waiting for mdp to complete 0x%x\n",
			   __func__, mask);
		printk("GLBL_CLK_ENA: %08X\n", readl(MSM_CLK_CTL_BASE + 0x0000));
		printk("GLBL_CLK_STATE: %08X\n", readl(MSM_CLK_CTL_BASE + 0x0004));
		printk("GLBL_SLEEP_EN: %08X\n", readl(MSM_CLK_CTL_BASE + 0x001C));
		printk("GLBL_CLK_ENA_2: %08X\n", readl(MSM_CLK_CTL_BASE + 0x0220));
		printk("GLBL_CLK_STATE_2: %08X\n", readl(MSM_CLK_CTL_BASE + 0x0224));
		printk("GLBL_CLK_SLEEP_EN_2: %08X\n", readl(MSM_CLK_CTL_BASE + 0x023C));
		mdp_ppp_dump_debug(mdp);
		locked_disable_mdp_irq(mdp, mask);
		ret = -ETIMEDOUT;
	} else {
//		pr_info("%s: SUCCESS waiting for 0x%x\n", __func__, mask);
	}
	spin_unlock_irqrestore(&mdp->lock, irq_flags);

	return ret;
}

void mdp_dma_wait(struct mdp_device *mdp_dev, int interface)
{
#define MDP_MAX_TIMEOUTS 20
	static int timeout_count;
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	unsigned int mask = 0;
	wait_queue_head_t *wq;

	switch (interface) {
	case MSM_MDDI_PMDH_INTERFACE:
	case MSM_MDDI_EMDH_INTERFACE:
	case MSM_LCDC_INTERFACE:
		BUG_ON(!mdp->out_if[interface].registered);
		mask = mdp->out_if[interface].dma_mask;
		wq = &mdp->out_if[interface].dma_waitqueue;
		break;
	default:
		pr_err("%s: Unknown interface %d\n", __func__, interface);
		BUG();
	}

	if (mdp_wait(mdp, mask, wq) == -ETIMEDOUT)
		timeout_count++;
	else
		timeout_count = 0;

	if (timeout_count > MDP_MAX_TIMEOUTS) {
		printk(KERN_ERR "mdp: dma failed %d times, somethings wrong!\n",
		       MDP_MAX_TIMEOUTS);
		BUG();
	}
}

static int mdp_ppp_wait(struct mdp_info *mdp)
{
	return mdp_wait(mdp, DL0_ROI_DONE, &mdp_ppp_waitqueue);
}

static void mdp_dma_to_mddi(void *priv, uint32_t addr, uint32_t stride,
			    uint32_t width, uint32_t height, uint32_t x,
			    uint32_t y)
{
	struct mdp_info *mdp = priv;
	uint32_t dma2_cfg;
	uint16_t ld_param = 0; /* 0=PRIM, 1=SECD, 2=EXT */

	dma2_cfg = DMA_PACK_TIGHT |
		DMA_PACK_ALIGN_LSB |
		DMA_PACK_PATTERN_RGB |
		DMA_OUT_SEL_AHB |
		DMA_IBUF_NONCONTIGUOUS;

	dma2_cfg |= mdp->format;

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

	mdp_writel(mdp, dma2_cfg, MDP_DMA_P_CONFIG);
	mdp_writel(mdp, 0, MDP_DMA_P_START);
#endif
}

void mdp_dma(struct mdp_device *mdp_dev, uint32_t addr, uint32_t stride,
	     uint32_t width, uint32_t height, uint32_t x, uint32_t y,
	     struct msmfb_callback *callback, int interface)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	struct mdp_out_interface *out_if;
	unsigned long flags;

	if (interface < 0 || interface > MSM_MDP_NUM_INTERFACES ||
	    !mdp->out_if[interface].registered) {
		pr_err("%s: Unknown interface: %d\n", __func__, interface);
		BUG();
	}
	out_if = &mdp->out_if[interface];

	spin_lock_irqsave(&mdp->lock, flags);
	if (locked_enable_mdp_irq(mdp, out_if->dma_mask)) {
		pr_err("%s: busy\n", __func__);
		goto done;
	}

	out_if->dma_cb = callback;
	out_if->dma_start(out_if->priv, addr, stride, width, height, x, y);
done:
	spin_unlock_irqrestore(&mdp->lock, flags);
}

static int get_img(struct mdp_img *img, struct fb_info *info,
		   unsigned long *start, unsigned long *len,
		   struct file** filep)
{
	int put_needed, ret = 0;
	struct file *file;
	unsigned long vstart;

	if (!get_pmem_file(img->memory_id, start, &vstart, len, filep))
		return 0;
	else if (!get_msm_hw3d_file(img->memory_id, &img->offset, start, len,
				    filep))
		return 0;

	file = fget_light(img->memory_id, &put_needed);
	if (file == NULL)
		return -1;

	if (MAJOR(file->f_dentry->d_inode->i_rdev) == FB_MAJOR) {
		*start = info->fix.smem_start;
		*len = info->fix.smem_len;
		ret = 0;
	} else
		ret = -1;
	fput_light(file, put_needed);

	return ret;
}

static void put_img(struct file *file)
{
	if (file) {
		if (is_pmem_file(file))
			put_pmem_file(file);
		else if (is_msm_hw3d_file(file))
			put_msm_hw3d_file(file);
	}
}

void mdp_configure_dma(struct mdp_device *mdp_dev)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	uint32_t dma_cfg;

	if (!mdp->dma_config_dirty)
		return;
	dma_cfg = mdp_readl(mdp, MDP_DMA_P_CONFIG);
	dma_cfg &= ~DMA_IBUF_FORMAT_MASK;
	dma_cfg &= ~DMA_PACK_PATTERN_MASK;
	dma_cfg |= (mdp->format | mdp->pack_pattern);
	mdp_writel(mdp, dma_cfg, MDP_DMA_P_CONFIG);
	mdp->dma_config_dirty = false;

	return;
}

int mdp_check_output_format(struct mdp_device *mdp_dev, int bpp)
{
	switch (bpp) {
	case 16:
	case 24:
	case 32:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int mdp_set_output_format(struct mdp_device *mdp_dev, int bpp)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	uint32_t format, pack_pattern;

	switch (bpp) {
	case 16:
		format = DMA_IBUF_FORMAT_RGB565;
		pack_pattern = DMA_PACK_PATTERN_RGB;
		break;
#ifdef CONFIG_MSM_MDP22
	case 24:
	case 32:
		format = DMA_IBUF_FORMAT_RGB888_OR_ARGB8888;
		break;
#else
	case 24:
		format = DMA_IBUF_FORMAT_RGB888;
		pack_pattern = DMA_PACK_PATTERN_BGR;
		break;
	case 32:
		format = DMA_IBUF_FORMAT_XRGB8888;
		pack_pattern = DMA_PACK_PATTERN_BGR;
		break;
#endif
	default:
		return -EINVAL;
	}
	if (format != mdp->format || pack_pattern != mdp->pack_pattern) {
		mdp->format = format;
		mdp->pack_pattern = pack_pattern;
		mdp->dma_config_dirty = true;
	}

	return 0;
}

static void dump_req(struct mdp_blit_req *req,
	unsigned long src_start, unsigned long src_len,
	unsigned long dst_start, unsigned long dst_len)
{
	pr_err("flags: 0x%x\n", 	req->flags);
	pr_err("src_start:  0x%08lx\n", src_start);
	pr_err("src_len:    0x%08lx\n", src_len);
	pr_err("src.offset: 0x%x\n",    req->src.offset);
	pr_err("src.format: 0x%x\n",    req->src.format);
	pr_err("src.width:  %d\n",      req->src.width);
	pr_err("src.height: %d\n",      req->src.height);
	pr_err("src_rect.x: %d\n",      req->src_rect.x);
	pr_err("src_rect.y: %d\n",      req->src_rect.y);
	pr_err("src_rect.w: %d\n",      req->src_rect.w);
	pr_err("src_rect.h: %d\n",      req->src_rect.h);

	pr_err("dst_start:  0x%08lx\n", dst_start);
	pr_err("dst_len:    0x%08lx\n", dst_len);
	pr_err("dst.offset: 0x%x\n",    req->dst.offset);
	pr_err("dst.format: 0x%x\n",    req->dst.format);
	pr_err("dst.width:  %d\n",      req->dst.width);
	pr_err("dst.height: %d\n",      req->dst.height);
	pr_err("dst_rect.x: %d\n",      req->dst_rect.x);
	pr_err("dst_rect.y: %d\n",      req->dst_rect.y);
	pr_err("dst_rect.w: %d\n",      req->dst_rect.w);
	pr_err("dst_rect.h: %d\n",      req->dst_rect.h);
}

int mdp_blit_and_wait(struct mdp_info *mdp, struct mdp_blit_req *req,
	struct file *src_file, unsigned long src_start, unsigned long src_len,
	struct file *dst_file, unsigned long dst_start, unsigned long dst_len)
{
	int ret;
	enable_mdp_irq(mdp, DL0_ROI_DONE);
	ret = mdp_ppp_blit(mdp, req,
			src_file, src_start, src_len,
			dst_file, dst_start, dst_len);
	if (unlikely(ret)) {
		disable_mdp_irq(mdp, DL0_ROI_DONE);
		return ret;
	}
	ret = mdp_ppp_wait(mdp);
	if (unlikely(ret)) {
		printk(KERN_ERR "%s: failed!\n", __func__);
		pr_err("original request:\n");
		dump_req(mdp->req, src_start, src_len, dst_start, dst_len);
		pr_err("dead request:\n");
		dump_req(req, src_start, src_len, dst_start, dst_len);
		BUG();
		return ret;
	}
	return 0;
}

int mdp_blit(struct mdp_device *mdp_dev, struct fb_info *fb,
	     struct mdp_blit_req *req)
{
	int ret;
	unsigned long src_start = 0, src_len = 0, dst_start = 0, dst_len = 0;
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	struct file *src_file = 0, *dst_file = 0;

#ifdef CONFIG_MSM_MDP31
	if (req->flags & MDP_ROT_90) {
		if (unlikely(((req->dst_rect.h == 1) &&
			((req->src_rect.w != 1) ||
			(req->dst_rect.w != req->src_rect.h))) ||
			((req->dst_rect.w == 1) && ((req->src_rect.h != 1) ||
			(req->dst_rect.h != req->src_rect.w))))) {
			pr_err("mpd_ppp: error scaling when size is 1!\n");
			return -EINVAL;
		}
	} else {
		if (unlikely(((req->dst_rect.w == 1) &&
			((req->src_rect.w != 1) ||
			(req->dst_rect.h != req->src_rect.h))) ||
			((req->dst_rect.h == 1) && ((req->src_rect.h != 1) ||
			(req->dst_rect.h != req->src_rect.h))))) {
			pr_err("mpd_ppp: error scaling when size is 1!\n");
			return -EINVAL;
		}
	}
#endif

	/* WORKAROUND FOR HARDWARE BUG IN BG TILE FETCH */
	if (unlikely(req->src_rect.h == 0 ||
		     req->src_rect.w == 0)) {
		printk(KERN_ERR "mdp_ppp: src img of zero size!\n");
		return -EINVAL;
	}
	if (unlikely(req->dst_rect.h == 0 ||
		     req->dst_rect.w == 0))
		return -EINVAL;

	/* do this first so that if this fails, the caller can always
	 * safely call put_img */
	if (unlikely(get_img(&req->src, fb, &src_start, &src_len, &src_file))) {
		printk(KERN_ERR "mdp_ppp: could not retrieve src image from "
				"memory\n");
		return -EINVAL;
	}

	if (unlikely(get_img(&req->dst, fb, &dst_start, &dst_len, &dst_file))) {
		printk(KERN_ERR "mdp_ppp: could not retrieve dst image from "
				"memory\n");
		put_img(src_file);
		return -EINVAL;
	}
	mutex_lock(&mdp_mutex);

	/* transp_masking unimplemented */
	req->transp_mask = MDP_TRANSP_NOP;
	mdp->req = req;
#ifndef CONFIG_MSM_MDP31
	if (unlikely((req->transp_mask != MDP_TRANSP_NOP ||
		      req->alpha != MDP_ALPHA_NOP ||
		      HAS_ALPHA(req->src.format)) &&
		     (req->flags & MDP_ROT_90 &&
		      req->dst_rect.w <= 16 && req->dst_rect.h >= 16))) {
		int i;
		unsigned int tiles = req->dst_rect.h / 16;
		unsigned int remainder = req->dst_rect.h % 16;
		req->src_rect.w = 16*req->src_rect.w / req->dst_rect.h;
		req->dst_rect.h = 16;
		for (i = 0; i < tiles; i++) {
			ret = mdp_blit_and_wait(mdp, req,
						src_file, src_start, src_len,
						dst_file, dst_start, dst_len);
			if (ret)
				goto end;
			req->dst_rect.y += 16;
			req->src_rect.x += req->src_rect.w;
		}
		if (!remainder)
			goto end;
		req->src_rect.w = remainder*req->src_rect.w / req->dst_rect.h;
		req->dst_rect.h = remainder;
	}
#else
	/* Workarounds for MDP 3.1 hardware bugs */
	if (unlikely((mdp_get_bytes_per_pixel(req->dst.format) == 4) &&
		(req->dst_rect.w != 1) &&
		(((req->dst_rect.w % 8) == 6) ||
		((req->dst_rect.w % 32) == 3) ||
		((req->dst_rect.w % 32) == 1)))) {
		ret = mdp_ppp_blit_split_width(mdp, req,
			src_file, src_start, src_len,
			dst_file, dst_start, dst_len);
		goto end;
	} else if (unlikely((req->dst_rect.w != 1) && (req->dst_rect.h != 1) &&
		((req->dst_rect.h % 32) == 3 ||
		(req->dst_rect.h % 32) == 1))) {
		ret = mdp_ppp_blit_split_height(mdp, req,
			src_file, src_start, src_len,
			dst_file, dst_start, dst_len);
		goto end;
	}
#endif
	ret = mdp_blit_and_wait(mdp, req,
				src_file, src_start, src_len,
				dst_file, dst_start, dst_len);
end:
	put_img(src_file);
	put_img(dst_file);
	mutex_unlock(&mdp_mutex);
	return ret;
}

void mdp_set_grp_disp(struct mdp_device *mdp_dev, unsigned disp_id)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);

	disp_id &= 0xf;
	mdp_writel(mdp, disp_id, MDP_FULL_BYPASS_WORD43);
}

/* used by output interface drivers like mddi and lcdc */
int mdp_out_if_register(struct mdp_device *mdp_dev, int interface,
			void *private_data, uint32_t dma_mask,
			mdp_dma_start_func_t dma_start)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	unsigned long flags;
	int ret = 0;

	if (interface < 0 || interface >= MSM_MDP_NUM_INTERFACES) {
		pr_err("%s: invalid interface (%d)\n", __func__, interface);
		return -EINVAL;
	}

	spin_lock_irqsave(&mdp->lock, flags);

	if (mdp->out_if[interface].registered) {
		pr_err("%s: interface (%d) already registered\n", __func__,
		       interface);
		ret = -EINVAL;
		goto done;
	}

	init_waitqueue_head(&mdp->out_if[interface].dma_waitqueue);
	mdp->out_if[interface].registered = 1;
	mdp->out_if[interface].priv = private_data;
	mdp->out_if[interface].dma_mask = dma_mask;
	mdp->out_if[interface].dma_start = dma_start;
	mdp->out_if[interface].dma_cb = NULL;

done:
	spin_unlock_irqrestore(&mdp->lock, flags);
	return ret;
}

int mdp_out_if_req_irq(struct mdp_device *mdp_dev, int interface,
		       uint32_t mask, struct msmfb_callback *cb)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	unsigned long flags;
	int ret = 0;

	if (interface < 0 || interface >= MSM_MDP_NUM_INTERFACES) {
		pr_err("%s: invalid interface (%d)\n", __func__, interface);
		BUG();
	} else if (!mdp->out_if[interface].registered) {
		pr_err("%s: interface (%d) not registered\n", __func__,
		       interface);
		BUG();
	}

	spin_lock_irqsave(&mdp->lock, flags);

	if (mask) {
		ret = locked_enable_mdp_irq(mdp, mask);
		if (ret) {
			pr_err("%s: busy\n", __func__);
			goto done;
		}
		mdp->out_if[interface].irq_mask = mask;
		mdp->out_if[interface].irq_cb = cb;
	} else {
		locked_disable_mdp_irq(mdp, mask);
		mdp->out_if[interface].irq_mask = 0;
		mdp->out_if[interface].irq_cb = NULL;
	}

done:
	spin_unlock_irqrestore(&mdp->lock, flags);
	return ret;
}

int register_mdp_client(struct class_interface *cint)
{
	if (!mdp_class) {
		pr_err("mdp: no mdp_class when registering mdp client\n");
		return -ENODEV;
	}
	cint->class = mdp_class;
	return class_interface_register(cint);
}

#include "mdp_csc_table.h"

void mdp_hw_init(struct mdp_info *mdp)
{
	int n;

	mdp_irq_mask = 0;

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
}

int mdp_probe(struct platform_device *pdev)
{
	struct resource *resource;
	int ret;
	struct mdp_info *mdp;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		pr_err("mdp: can not get mdp mem resource!\n");
		return -ENOMEM;
	}

	mdp = kzalloc(sizeof(struct mdp_info), GFP_KERNEL);
	if (!mdp)
		return -ENOMEM;

	spin_lock_init(&mdp->lock);

	mdp->irq = platform_get_irq(pdev, 0);
	if (mdp->irq < 0) {
		pr_err("mdp: can not get mdp irq\n");
		ret = mdp->irq;
		goto error_get_irq;
	}

	mdp->base = ioremap(resource->start,
			    resource->end - resource->start);
	if (mdp->base == 0) {
		printk(KERN_ERR "msmfb: cannot allocate mdp regs!\n");
		ret = -ENOMEM;
		goto error_ioremap;
	}

	mdp->mdp_dev.dma = mdp_dma;
	mdp->mdp_dev.dma_wait = mdp_dma_wait;
	mdp->mdp_dev.blit = mdp_blit;
	mdp->mdp_dev.set_grp_disp = mdp_set_grp_disp;
	mdp->mdp_dev.set_output_format = mdp_set_output_format;
	mdp->mdp_dev.check_output_format = mdp_check_output_format;
	mdp->mdp_dev.configure_dma = mdp_configure_dma;

	ret = mdp_out_if_register(&mdp->mdp_dev, MSM_MDDI_PMDH_INTERFACE, mdp,
				  MDP_DMA_P_DONE, mdp_dma_to_mddi);
	if (ret)
		goto error_mddi_pmdh_register;

	mdp->clk = clk_get(&pdev->dev, "mdp_clk");
	if (IS_ERR(mdp->clk)) {
		printk(KERN_INFO "mdp: failed to get mdp clk");
		ret = PTR_ERR(mdp->clk);
		goto error_get_mdp_clk;
	}

	mdp->ebi1_clk = clk_get(NULL, "ebi1_clk");
	if (IS_ERR(mdp->ebi1_clk)) {
		pr_err("mdp: failed to get ebi1 clk\n");
		ret = PTR_ERR(mdp->ebi1_clk);
		goto error_get_ebi1_clk;
	}


	ret = request_irq(mdp->irq, mdp_isr, IRQF_DISABLED, "msm_mdp", mdp);
	if (ret)
		goto error_request_irq;
	disable_irq(mdp->irq);

	clk_enable(mdp->clk);
	mdp_clk_to_disable_later = mdp->clk;
	mdp_hw_init(mdp);

	/* register mdp device */
	mdp->mdp_dev.dev.parent = &pdev->dev;
	mdp->mdp_dev.dev.class = mdp_class;
	dev_set_name(&mdp->mdp_dev.dev, "mdp%d", pdev->id);

	/* if you can remove the platform device you'd have to implement
	 * this:
	mdp_dev.release = mdp_class; */

	ret = device_register(&mdp->mdp_dev.dev);
	if (ret)
		goto error_device_register;

	pr_info("%s: initialized\n", __func__);

	return 0;

error_device_register:
	free_irq(mdp->irq, mdp);
error_request_irq:
	clk_put(mdp->ebi1_clk);
error_get_ebi1_clk:
	clk_put(mdp->clk);
error_get_mdp_clk:
error_mddi_pmdh_register:
	iounmap(mdp->base);
error_ioremap:
error_get_irq:
	kfree(mdp);
	return ret;
}

static struct platform_driver msm_mdp_driver = {
	.probe = mdp_probe,
	.driver = {.name = "msm_mdp"},
};

static int __init mdp_lateinit(void)
{
	if (mdp_clk_to_disable_later)
		clk_disable(mdp_clk_to_disable_later);
	return 0;
}

static int __init mdp_init(void)
{
	mdp_class = class_create(THIS_MODULE, "msm_mdp");
	if (IS_ERR(mdp_class)) {
		printk(KERN_ERR "Error creating mdp class\n");
		return PTR_ERR(mdp_class);
	}
	return platform_driver_register(&msm_mdp_driver);
}

subsys_initcall(mdp_init);
late_initcall(mdp_lateinit);
