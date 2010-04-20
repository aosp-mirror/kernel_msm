/* drivers/video/msm/mdp_ppp.c
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
#include <linux/fb.h>
#include <linux/file.h>
#include <linux/delay.h>
#include <linux/major.h>
#include <linux/msm_hw3d.h>
#include <linux/msm_mdp.h>
#include <linux/mutex.h>
#include <linux/android_pmem.h>
#include <linux/wait.h>
#include <mach/msm_fb.h>

#include "mdp_hw.h"
#include "mdp_ppp.h"

#define PPP_DUMP_BLITS 0

#define PPP_DEBUG_MSGS 1
#if PPP_DEBUG_MSGS
#define DLOG(fmt,args...) \
	do { printk(KERN_INFO "[%s:%s:%d] "fmt, __FILE__, __func__, \
		    __LINE__, ##args); } \
	while (0)
#else
#define DLOG(x...) do {} while (0)
#endif

#define IMG_LEN(rect_h, w, rect_w, bpp) (((rect_h) * w) * bpp)

#define Y_TO_CRCB_RATIO(format) \
	((format == MDP_Y_CBCR_H2V2 || format == MDP_Y_CRCB_H2V2) ?  2 :\
	 (format == MDP_Y_CBCR_H2V1 || format == MDP_Y_CRCB_H2V1) ?  1 : 1)

static uint32_t pack_pattern[] = {
	PPP_ARRAY0(PACK_PATTERN)
};

static uint32_t src_img_cfg[] = {
	PPP_ARRAY1(CFG, SRC)
};

static uint32_t dst_img_cfg[] = {
	PPP_ARRAY1(CFG, DST)
};

static const uint32_t bytes_per_pixel[] = {
	[MDP_RGB_565] = 2,
	[MDP_XRGB_8888] = 4,
	[MDP_Y_CBCR_H2V2] = 1,
	[MDP_ARGB_8888] = 4,
	[MDP_RGB_888] = 3,
	[MDP_Y_CRCB_H2V2] = 1,
	[MDP_YCRYCB_H2V1] = 2,
	[MDP_Y_CRCB_H2V1] = 1,
	[MDP_Y_CBCR_H2V1] = 1,
	[MDP_RGBA_8888] = 4,
	[MDP_BGRA_8888] = 4,
	[MDP_RGBX_8888] = 4,
};

static uint32_t dst_op_chroma[] = {
	PPP_ARRAY1(CHROMA_SAMP, DST)
};

static uint32_t src_op_chroma[] = {
	PPP_ARRAY1(CHROMA_SAMP, SRC)
};

static uint32_t bg_op_chroma[] = {
	PPP_ARRAY1(CHROMA_SAMP, BG)
};

static DECLARE_WAIT_QUEUE_HEAD(mdp_ppp_waitqueue);
DEFINE_MUTEX(mdp_mutex);

static uint32_t get_luma_offset(struct mdp_img *img,
				struct mdp_rect *rect, uint32_t bpp)
{
#ifndef CONFIG_MSM_MDP31
	return (rect->x + (rect->y * img->width)) * bpp;
#else
	return 0;
#endif
}

static uint32_t get_chroma_offset(struct mdp_img *img,
				  struct mdp_rect *rect, uint32_t bpp)
{
#ifndef CONFIG_MSM_MDP31
	uint32_t compress_v = Y_TO_CRCB_RATIO(img->format);
	uint32_t compress_h = 2;
	uint32_t offset = 0;

	if (IS_PSEUDOPLNR(img->format)) {
		offset = (rect->x / compress_h) * compress_h;
		offset += rect->y == 0 ? 0 :
			  ((rect->y + 1) / compress_v) * img->width;
		offset *= bpp;
	}
	return offset;
#else
	return 0;
#endif
}

static void set_src_region(struct mdp_img *img, struct mdp_rect *rect,
			   struct ppp_regs *regs)
{
	regs->src_rect = (rect->h << 16) | (rect->w & 0x1fff);

#ifdef CONFIG_MSM_MDP31
	regs->src_xy = (rect->y << 16) | (rect->x & 0x1fff);
	regs->src_img_sz = (img->height << 16) | (img->width & 0x1fff);
#endif
}

static inline void set_dst_region(struct mdp_rect *rect, struct ppp_regs *regs)
{
	regs->dst_rect = (rect->h << 16) | (rect->w & 0xfff);

#ifdef CONFIG_MSM_MDP31
	regs->dst_xy = (rect->y << 16) | (rect->x & 0x1fff);
#endif
}

static void set_blend_region(struct mdp_img *img, struct mdp_rect *rect,
			     struct ppp_regs *regs)
{
#ifdef CONFIG_MSM_MDP31
	uint32_t rect_x = rect->x;
	uint32_t rect_y = rect->y;
	uint32_t img_w = img->width;
	uint32_t img_h = img->height;

	/* HW bug workaround */
	if (img->format == MDP_YCRYCB_H2V1) {
		regs->bg0 += (rect_x + (rect_y * img_w)) * regs->bg_bpp;
		rect_x = 0;
		rect_y = 0;
		img_w = rect->w;
		img_h = rect->h;
	}

	regs->bg_xy = (rect_y << 16) | (rect_x & 0x1fff);
	regs->bg_img_sz = (img_h << 16) | (img_w & 0x1fff);
#endif
}

static void rotate_dst_addr_x(struct mdp_blit_req *req,
			      struct ppp_regs *regs)
{
#ifndef CONFIG_MSM_MDP31
	regs->dst0 += (req->dst_rect.w -
		       min((uint32_t)16, req->dst_rect.w)) * regs->dst_bpp;
	regs->dst1 += (req->dst_rect.w -
		       min((uint32_t)16, req->dst_rect.w)) * regs->dst_bpp;
#endif
}

static void rotate_dst_addr_y(struct mdp_blit_req *req,
			      struct ppp_regs *regs)
{
#ifndef CONFIG_MSM_MDP31
	regs->dst0 += (req->dst_rect.h -
		       min((uint32_t)16, req->dst_rect.h)) *
		       regs->dst_ystride;
	regs->dst1 += (req->dst_rect.h -
		       min((uint32_t)16, req->dst_rect.h)) *
		       regs->dst_ystride;
#endif
}

static void blit_rotate(struct mdp_blit_req *req,
			struct ppp_regs *regs)
{
	if (req->flags == MDP_ROT_NOP)
		return;

	regs->op |= PPP_OP_ROT_ON;
	if ((req->flags & MDP_ROT_90 || req->flags & MDP_FLIP_LR) &&
	    !(req->flags & MDP_ROT_90 && req->flags & MDP_FLIP_LR))
		rotate_dst_addr_x(req, regs);
	if (req->flags & MDP_ROT_90)
		regs->op |= PPP_OP_ROT_90;
	if (req->flags & MDP_FLIP_UD) {
		regs->op |= PPP_OP_FLIP_UD;
		rotate_dst_addr_y(req, regs);
	}
	if (req->flags & MDP_FLIP_LR)
		regs->op |= PPP_OP_FLIP_LR;
}

static void blit_convert(struct mdp_blit_req *req, struct ppp_regs *regs)
{
	if (req->src.format == req->dst.format)
		return;
	if (IS_RGB(req->src.format) && IS_YCRCB(req->dst.format)) {
		regs->op |= PPP_OP_CONVERT_RGB2YCBCR | PPP_OP_CONVERT_ON;
#ifdef CONFIG_MSM_MDP31
		/* primary really means set1 */
		regs->op |= PPP_OP_CONVERT_MATRIX_PRIMARY;
		regs->csc_cfg = 0x1e;
#endif
	} else if (IS_YCRCB(req->src.format) && IS_RGB(req->dst.format)) {
		regs->op |= PPP_OP_CONVERT_YCBCR2RGB | PPP_OP_CONVERT_ON;
#ifdef CONFIG_MSM_MDP31
		/* secondary really means set2 */
		regs->op |= PPP_OP_CONVERT_MATRIX_SECONDARY;
		regs->csc_cfg = 0;
#endif
	}
}

#define GET_BIT_RANGE(value, high, low) \
	(((1 << (high - low + 1)) - 1) & (value >> low))
static uint32_t transp_convert(struct mdp_blit_req *req)
{
	uint32_t transp = 0;
	if (req->src.format == MDP_RGB_565) {
		/* pad each value to 8 bits by copying the high bits into the
		 * low end, convert RGB to RBG by switching low 2 components */
		transp |= ((GET_BIT_RANGE(req->transp_mask, 15, 11) << 3) |
			   (GET_BIT_RANGE(req->transp_mask, 15, 13))) << 16;

		transp |= ((GET_BIT_RANGE(req->transp_mask, 4, 0) << 3) |
			   (GET_BIT_RANGE(req->transp_mask, 4, 2))) << 8;

		transp |= (GET_BIT_RANGE(req->transp_mask, 10, 5) << 2) |
			  (GET_BIT_RANGE(req->transp_mask, 10, 9));
	} else {
		/* convert RGB to RBG */
		transp |= (GET_BIT_RANGE(req->transp_mask, 15, 8)) |
			  (GET_BIT_RANGE(req->transp_mask, 23, 16) << 16) |
			  (GET_BIT_RANGE(req->transp_mask, 7, 0) << 8);
	}
	return transp;
}
#undef GET_BIT_RANGE

static void blit_blend(struct mdp_blit_req *req, struct ppp_regs *regs)
{
	/* TRANSP BLEND */
	if (req->transp_mask != MDP_TRANSP_NOP) {
		req->transp_mask = transp_convert(req);
		if (req->alpha != MDP_ALPHA_NOP) {
			/* use blended transparancy mode
			 * pixel = (src == transp) ? dst : blend
			 * blend is combo of blend_eq_sel and
			 * blend_alpha_sel */
			regs->op |= PPP_OP_ROT_ON | PPP_OP_BLEND_ON |
				PPP_OP_BLEND_ALPHA_BLEND_NORMAL |
				PPP_OP_BLEND_CONSTANT_ALPHA |
				PPP_BLEND_ALPHA_TRANSP;
		} else {
			/* simple transparancy mode
			 * pixel = (src == transp) ? dst : src */
			regs->op |= PPP_OP_ROT_ON | PPP_OP_BLEND_ON |
				PPP_OP_BLEND_SRCPIXEL_TRANSP;
		}
	}

	req->alpha &= 0xff;
	/* ALPHA BLEND */
	if (HAS_ALPHA(req->src.format)) {
		regs->op |= PPP_OP_ROT_ON | PPP_OP_BLEND_ON;
		if (req->flags & MDP_BLEND_FG_PREMULT) {
#ifdef CONFIG_MSM_MDP31
			/* premultiplied alpha:
			 * bg_alpha = (1 - fg_alpha)
			 * fg_alpha = 0xff
			 */
			regs->bg_alpha_sel = PPP_BLEND_BG_USE_ALPHA_SEL |
				PPP_BLEND_BG_ALPHA_REVERSE |
				PPP_BLEND_BG_SRCPIXEL_ALPHA;
			regs->op |= PPP_OP_BLEND_CONSTANT_ALPHA;
			req->alpha = 0xff;
#endif
		} else {
			regs->op |= PPP_OP_BLEND_SRCPIXEL_ALPHA;
		}
	} else if (req->alpha < MDP_ALPHA_NOP) {
		/* just blend by alpha */
		regs->op |= PPP_OP_ROT_ON | PPP_OP_BLEND_ON |
			PPP_OP_BLEND_ALPHA_BLEND_NORMAL |
			PPP_OP_BLEND_CONSTANT_ALPHA;
	}

	regs->op |= bg_op_chroma[req->dst.format];

	/* since we always blend src + dst -> dst, copy most of the
	 * configuration from dest to bg */
	regs->bg0 = regs->dst0;
	regs->bg1 = regs->dst1;
	regs->bg_cfg = src_img_cfg[req->dst.format];
	regs->bg_bpp = regs->dst_bpp;
	regs->bg_pack = pack_pattern[req->dst.format];
	regs->bg_ystride = regs->dst_ystride;
	set_blend_region(&req->dst, &req->dst_rect, regs);
}

static int blit_scale(struct mdp_info *mdp, struct mdp_blit_req *req,
		      struct ppp_regs *regs)
{
	struct mdp_rect dst_rect;

	memcpy(&dst_rect, &req->dst_rect, sizeof(dst_rect));
	if (req->flags & MDP_ROT_90) {
		dst_rect.w = req->dst_rect.h;
		dst_rect.h = req->dst_rect.w;
	}

	if ((req->src_rect.w == dst_rect.w) && (req->src_rect.h == dst_rect.h)
	    && !(req->flags & MDP_BLUR)) {
		regs->phasex_init = 0;
		regs->phasey_init = 0;
		regs->phasex_step = 0;
		regs->phasey_step = 0;
		return 0;
	}

	if (mdp_ppp_cfg_scale(mdp, regs, &req->src_rect, &dst_rect,
				req->src.format, req->dst.format)) {
		DLOG("crap, bad scale\n");
		return -1;
	}

	regs->op |= (PPP_OP_SCALE_Y_ON | PPP_OP_SCALE_X_ON);
	return 0;
}

static void blit_blur(struct mdp_info *mdp, struct mdp_blit_req *req,
		      struct ppp_regs *regs)
{
	int ret;
	if (!(req->flags & MDP_BLUR))
		return;

	ret = mdp_ppp_load_blur(mdp);
	if (ret)
		return;

	regs->op |= (PPP_OP_SCALE_Y_ON | PPP_OP_SCALE_X_ON);
}

static void get_len(struct mdp_img *img, struct mdp_rect *rect, uint32_t bpp,
		    uint32_t *len0, uint32_t *len1)
{
	*len0 = IMG_LEN(rect->h, img->width, rect->w, bpp);
	if (IS_PSEUDOPLNR(img->format))
		*len1 = *len0/Y_TO_CRCB_RATIO(img->format);
	else
		*len1 = 0;
}

static int valid_src_dst(unsigned long src_start, unsigned long src_len,
			 unsigned long dst_start, unsigned long dst_len,
			 struct mdp_blit_req *req, struct ppp_regs *regs)
{
	unsigned long src_min_ok = src_start;
	unsigned long src_max_ok = src_start + src_len;
	unsigned long dst_min_ok = dst_start;
	unsigned long dst_max_ok = dst_start + dst_len;
	uint32_t src0_len, src1_len, dst0_len, dst1_len;
	get_len(&req->src, &req->src_rect, regs->src_bpp, &src0_len,
		 &src1_len);
	get_len(&req->dst, &req->dst_rect, regs->dst_bpp, &dst0_len,
		 &dst1_len);

	if (regs->src0 < src_min_ok || regs->src0 > src_max_ok ||
	    regs->src0 + src0_len > src_max_ok) {
		DLOG("invalid_src %x %x %lx %lx\n", regs->src0,
		      src0_len, src_min_ok, src_max_ok);
		return 0;
	}
	if (regs->src_cfg & PPP_SRC_PLANE_PSEUDOPLNR) {
		if (regs->src1 < src_min_ok || regs->src1 > src_max_ok ||
		    regs->src1 + src1_len > src_max_ok) {
			DLOG("invalid_src1");
			return 0;
		}
	}
	if (regs->dst0 < dst_min_ok || regs->dst0 > dst_max_ok ||
	    regs->dst0 + dst0_len > dst_max_ok) {
		DLOG("invalid_dst");
		return 0;
	}
	if (regs->dst_cfg & PPP_SRC_PLANE_PSEUDOPLNR) {
		if (regs->dst1 < dst_min_ok || regs->dst1 > dst_max_ok ||
		    regs->dst1 + dst1_len > dst_max_ok) {
			DLOG("invalid_dst1");
			return 0;
		}
	}
	return 1;
}

static void flush_imgs(struct mdp_blit_req *req, struct ppp_regs *regs,
		       struct file *src_file, struct file *dst_file)
{
#ifdef CONFIG_ANDROID_PMEM
	uint32_t src0_len, src1_len, dst0_len, dst1_len;

	/* flush src images to memory before dma to mdp */
	get_len(&req->src, &req->src_rect, regs->src_bpp, &src0_len,
		&src1_len);
	flush_pmem_file(src_file, req->src.offset, src0_len);
	if (IS_PSEUDOPLNR(req->src.format))
		flush_pmem_file(src_file, req->src.offset + src0_len,
				src1_len);

	/* flush dst images */
	get_len(&req->dst, &req->dst_rect, regs->dst_bpp, &dst0_len,
		&dst1_len);
	flush_pmem_file(dst_file, req->dst.offset, dst0_len);
	if (IS_PSEUDOPLNR(req->dst.format))
		flush_pmem_file(dst_file, req->dst.offset + dst0_len,
				dst1_len);
#endif
}

static uint32_t get_chroma_base(struct mdp_img *img, uint32_t base,
				uint32_t bpp)
{
	uint32_t addr = 0;

	if (IS_PSEUDOPLNR(img->format))
		addr = base + (img->width * img->height * bpp);
	return addr;
}

int mdp_get_bytes_per_pixel(int format)
{
	if (format < 0 || format >= MDP_IMGTYPE_LIMIT)
		return -1;
	return bytes_per_pixel[format];
}

#if PPP_DUMP_BLITS
#define mdp_writel_dbg(mdp, val, reg) do { \
		pr_info("%s: writing 0x%08x=0x%08x\n", __func__, (reg), (val));\
		mdp_writel((mdp), (val), (reg)); \
	} while (0)
#else
#define mdp_writel_dbg(mdp, val, reg) mdp_writel((mdp), (val), (reg))
#endif


static int send_blit(const struct mdp_info *mdp, struct mdp_blit_req *req,
		     struct ppp_regs *regs, struct file *src_file,
		     struct file *dst_file)
{
#if 0
	mdp_writel_dbg(mdp, 1, MDP_PPP_CMD_MODE);
#endif
	mdp_writel_dbg(mdp, regs->src_rect, PPP_ADDR_SRC_ROI);
	mdp_writel_dbg(mdp, regs->src0, PPP_ADDR_SRC0);
	mdp_writel_dbg(mdp, regs->src1, PPP_ADDR_SRC1);
	mdp_writel_dbg(mdp, regs->src_ystride, PPP_ADDR_SRC_YSTRIDE);
	mdp_writel_dbg(mdp, regs->src_cfg, PPP_ADDR_SRC_CFG);
	mdp_writel_dbg(mdp, regs->src_pack, PPP_ADDR_SRC_PACK_PATTERN);

	mdp_writel_dbg(mdp, regs->op, PPP_ADDR_OPERATION);
	mdp_writel_dbg(mdp, regs->phasex_init, PPP_ADDR_PHASEX_INIT);
	mdp_writel_dbg(mdp, regs->phasey_init, PPP_ADDR_PHASEY_INIT);
	mdp_writel_dbg(mdp, regs->phasex_step, PPP_ADDR_PHASEX_STEP);
	mdp_writel_dbg(mdp, regs->phasey_step, PPP_ADDR_PHASEY_STEP);

#ifdef CONFIG_MSM_MDP31
	mdp_writel_dbg(mdp, regs->scale_cfg, MDP_PPP_SCALE_CONFIG);
	mdp_writel_dbg(mdp, regs->csc_cfg, MDP_PPP_CSC_CONFIG);
	mdp_writel_dbg(mdp, regs->src_xy, MDP_PPP_SRC_XY);
	mdp_writel_dbg(mdp, regs->src_img_sz, MDP_PPP_SRC_IMAGE_SIZE);
	mdp_writel_dbg(mdp, regs->dst_xy, MDP_PPP_OUT_XY);
#else
	/* no edge conditions to set for MDP 3.1 */
	mdp_writel_dbg(mdp, regs->edge, PPP_ADDR_EDGE);
#endif

	mdp_writel_dbg(mdp, (req->alpha << 24) | (req->transp_mask & 0xffffff),
	       PPP_ADDR_ALPHA_TRANSP);

	mdp_writel_dbg(mdp, regs->dst_cfg, PPP_ADDR_DST_CFG);
	mdp_writel_dbg(mdp, regs->dst_pack, PPP_ADDR_DST_PACK_PATTERN);
	mdp_writel_dbg(mdp, regs->dst_rect, PPP_ADDR_DST_ROI);
	mdp_writel_dbg(mdp, regs->dst0, PPP_ADDR_DST0);
	mdp_writel_dbg(mdp, regs->dst1, PPP_ADDR_DST1);
	mdp_writel_dbg(mdp, regs->dst_ystride, PPP_ADDR_DST_YSTRIDE);

	if (regs->op & PPP_OP_BLEND_ON) {
		mdp_writel_dbg(mdp, regs->bg0, PPP_ADDR_BG0);
		mdp_writel_dbg(mdp, regs->bg1, PPP_ADDR_BG1);
		mdp_writel_dbg(mdp, regs->bg_ystride, PPP_ADDR_BG_YSTRIDE);
		mdp_writel_dbg(mdp, regs->bg_cfg, PPP_ADDR_BG_CFG);
		mdp_writel_dbg(mdp, regs->bg_pack, PPP_ADDR_BG_PACK_PATTERN);
#ifdef CONFIG_MSM_MDP31
		mdp_writel_dbg(mdp, regs->bg_xy, MDP_PPP_BG_XY);
		mdp_writel_dbg(mdp, regs->bg_img_sz, MDP_PPP_BG_IMAGE_SIZE);
		mdp_writel_dbg(mdp, regs->bg_alpha_sel,
			       MDP_PPP_BLEND_BG_ALPHA_SEL);
#endif
	}
	flush_imgs(req, regs, src_file, dst_file);
	mdp_writel_dbg(mdp, 0x1000, MDP_DISPLAY0_START);
	return 0;
}

#if PPP_DUMP_BLITS
static void mdp_dump_blit(struct mdp_blit_req *req)
{
	pr_info("%s: src: w=%d h=%d f=0x%x offs=0x%x mem_id=%d\n", __func__,
		req->src.width, req->src.height, req->src.format,
		req->src.offset, req->src.memory_id);
	pr_info("%s: dst: w=%d h=%d f=0x%x offs=0x%x mem_id=%d\n", __func__,
		req->dst.width, req->dst.height, req->dst.format,
		req->dst.offset, req->dst.memory_id);
	pr_info("%s: src_rect: x=%d y=%d w=%d h=%d\n", __func__,
		req->src_rect.x, req->src_rect.y, req->src_rect.w,
		req->src_rect.h);
	pr_info("%s: dst_rect: x=%d y=%d w=%d h=%d\n", __func__,
		req->dst_rect.x, req->dst_rect.y, req->dst_rect.w,
		req->dst_rect.h);
	pr_info("%s: alpha=0x%08x\n", __func__, req->alpha);
	pr_info("%s: transp_max=0x%08x\n", __func__, req->transp_mask);
	pr_info("%s: flags=%08x\n", __func__, req->flags);
}
#endif

static int process_blit(struct mdp_info *mdp, struct mdp_blit_req *req,
		 struct file *src_file, unsigned long src_start, unsigned long src_len,
		 struct file *dst_file, unsigned long dst_start, unsigned long dst_len)
{
	struct ppp_regs regs = {0};
	uint32_t luma_base;

#if PPP_DUMP_BLITS
	mdp_dump_blit(req);
#endif

	if (unlikely(req->src.format >= MDP_IMGTYPE_LIMIT ||
		     req->dst.format >= MDP_IMGTYPE_LIMIT)) {
		printk(KERN_ERR "mdp_ppp: img is of wrong format\n");
		return -EINVAL;
	}

	if (unlikely(req->src_rect.x > req->src.width ||
		     req->src_rect.y > req->src.height ||
		     req->dst_rect.x > req->dst.width ||
		     req->dst_rect.y > req->dst.height)) {
		printk(KERN_ERR "mdp_ppp: img rect is outside of img!\n");
		return -EINVAL;
	}

	if (unlikely(req->src_rect.x + req->src_rect.w > req->src.width ||
		     req->src_rect.y + req->src_rect.h > req->src.height ||
		     req->dst_rect.x + req->dst_rect.w > req->dst.width ||
		     req->dst_rect.y + req->dst_rect.h > req->dst.height)) {
		printk(KERN_ERR "mdp_ppp: img rect extends outside of img!\n");
		return -EINVAL;
	}

	/* set the src image configuration */
	regs.src_cfg = src_img_cfg[req->src.format];
	regs.src_cfg |= (req->src_rect.x & 0x1) ? PPP_SRC_BPP_ROI_ODD_X : 0;
	regs.src_cfg |= (req->src_rect.y & 0x1) ? PPP_SRC_BPP_ROI_ODD_Y : 0;
	regs.src_pack = pack_pattern[req->src.format];

	/* set the dest image configuration */
	regs.dst_cfg = dst_img_cfg[req->dst.format] | PPP_DST_OUT_SEL_AXI;
	regs.dst_pack = pack_pattern[req->dst.format];

	/* set src, bpp, start pixel and ystride */
	regs.src_bpp = mdp_get_bytes_per_pixel(req->src.format);
	luma_base = src_start + req->src.offset;
	regs.src0 = luma_base +
		get_luma_offset(&req->src, &req->src_rect, regs.src_bpp);
	regs.src1 = get_chroma_base(&req->src, luma_base, regs.src_bpp);
	regs.src1 += get_chroma_offset(&req->src, &req->src_rect, regs.src_bpp);
	regs.src_ystride = req->src.width * regs.src_bpp;
	set_src_region(&req->src, &req->src_rect, &regs);

	/* set dst, bpp, start pixel and ystride */
	regs.dst_bpp = mdp_get_bytes_per_pixel(req->dst.format);
	luma_base = dst_start + req->dst.offset;
	regs.dst0 = luma_base +
		get_luma_offset(&req->dst, &req->dst_rect, regs.dst_bpp);
	regs.dst1 = get_chroma_base(&req->dst, luma_base, regs.dst_bpp);
	regs.dst1 += get_chroma_offset(&req->dst, &req->dst_rect, regs.dst_bpp);
	regs.dst_ystride = req->dst.width * regs.dst_bpp;
	set_dst_region(&req->dst_rect, &regs);

	if (!valid_src_dst(src_start, src_len, dst_start, dst_len, req,
			   &regs)) {
		printk(KERN_ERR "mdp_ppp: final src or dst location is "
			"invalid, are you trying to make an image too large "
			"or to place it outside the screen?\n");
		return -EINVAL;
	}

	/* set up operation register */
	regs.op = 0;
	blit_rotate(req, &regs);
	blit_convert(req, &regs);
	if (req->flags & MDP_DITHER)
		regs.op |= PPP_OP_DITHER_EN;
	blit_blend(req, &regs);
	if (blit_scale(mdp, req, &regs)) {
		printk(KERN_ERR "mdp_ppp: error computing scale for img.\n");
		return -EINVAL;
	}
	blit_blur(mdp, req, &regs);
	regs.op |= dst_op_chroma[req->dst.format] |
		   src_op_chroma[req->src.format];

	/* if the image is YCRYCB, the x and w must be even */
	if (unlikely(req->src.format == MDP_YCRYCB_H2V1)) {
		req->src_rect.x = req->src_rect.x & (~0x1);
		req->src_rect.w = req->src_rect.w & (~0x1);
		req->dst_rect.x = req->dst_rect.x & (~0x1);
		req->dst_rect.w = req->dst_rect.w & (~0x1);
	}

	if (mdp_ppp_cfg_edge_cond(req, &regs))
		return -EINVAL;

	/* for simplicity, always write the chroma stride */
	regs.src_ystride &= 0x3fff;
	regs.src_ystride |= regs.src_ystride << 16;
	regs.dst_ystride &= 0x3fff;
	regs.dst_ystride |= regs.dst_ystride << 16;
	regs.bg_ystride &= 0x3fff;
	regs.bg_ystride |= regs.bg_ystride << 16;

#if PPP_DUMP_BLITS
	pr_info("%s: sending blit\n", __func__);
#endif
	send_blit(mdp, req, &regs, src_file, dst_file);
	return 0;
}

#define mdp_dump_register(mdp, reg) \
	printk(# reg ": %08x\n", mdp_readl((mdp), (reg)))

void mdp_ppp_dump_debug(const struct mdp_info *mdp)
{
	mdp_dump_register(mdp, MDP_TFETCH_STATUS);
	mdp_dump_register(mdp, MDP_TFETCH_TILE_COUNT);
	mdp_dump_register(mdp, MDP_TFETCH_FETCH_COUNT);
	mdp_dump_register(mdp, MDP_BGTFETCH_STATUS);
	mdp_dump_register(mdp, MDP_BGTFETCH_TILE_COUNT);
	mdp_dump_register(mdp, MDP_BGTFETCH_FETCH_COUNT);
	mdp_dump_register(mdp, MDP_PPP_SCALE_STATUS);
	mdp_dump_register(mdp, MDP_PPP_BLEND_STATUS);
	mdp_dump_register(mdp, MDP_INTR_STATUS);
	mdp_dump_register(mdp, MDP_INTR_ENABLE);
}

static int mdp_ppp_wait(struct mdp_info *mdp)
{
	int ret;

	ret = mdp_wait(mdp, DL0_ROI_DONE, &mdp_ppp_waitqueue);
	if (ret)
		mdp_ppp_dump_debug(mdp);
	return ret;
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

static void dump_req(struct mdp_blit_req *req,
	unsigned long src_start, unsigned long src_len,
	unsigned long dst_start, unsigned long dst_len)
{
	pr_err("flags: 0x%x\n",         req->flags);
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

int mdp_ppp_blit_and_wait(struct mdp_info *mdp, struct mdp_blit_req *req,
		struct file *src_file, unsigned long src_start, unsigned long src_len,
		struct file *dst_file, unsigned long dst_start, unsigned long dst_len)
{
	int ret;
	mdp->enable_irq(mdp, DL0_ROI_DONE);
	ret = process_blit(mdp, req, src_file, src_start, src_len,
			   dst_file, dst_start, dst_len);
	if (unlikely(ret)) {
		mdp->disable_irq(mdp, DL0_ROI_DONE);
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

int mdp_ppp_blit(struct mdp_info *mdp, struct fb_info *fb,
		 struct mdp_blit_req *req)
{
	int ret;
	unsigned long src_start = 0, src_len = 0, dst_start = 0, dst_len = 0;
	struct file *src_file = 0, *dst_file = 0;

	ret = mdp_ppp_validate_blit(mdp, req);
	if (ret)
		return ret;

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

	ret = mdp_ppp_do_blit(mdp, req, src_file, src_start, src_len,
			      dst_file, dst_start, dst_len);

	put_img(src_file);
	put_img(dst_file);
	mutex_unlock(&mdp_mutex);
	return ret;
}

void mdp_ppp_handle_isr(struct mdp_info *mdp, uint32_t mask)
{
	if (mask & DL0_ROI_DONE)
		wake_up(&mdp_ppp_waitqueue);
}


