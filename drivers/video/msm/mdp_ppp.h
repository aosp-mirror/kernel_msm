/* drivers/video/msm/mdp_ppp.h
 *
 * Copyright (C) 2009 Google Incorporated
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

#ifndef _VIDEO_MSM_MDP_PPP_H_
#define _VIDEO_MSM_MDP_PPP_H_

#include <linux/types.h>

struct ppp_regs {
	uint32_t src0;
	uint32_t src1;
	uint32_t dst0;
	uint32_t dst1;
	uint32_t src_cfg;
	uint32_t dst_cfg;
	uint32_t src_pack;
	uint32_t dst_pack;
	uint32_t src_rect;
	uint32_t dst_rect;
	uint32_t src_ystride;
	uint32_t dst_ystride;
	uint32_t op;
	uint32_t src_bpp;
	uint32_t dst_bpp;
	uint32_t edge;
	uint32_t phasex_init;
	uint32_t phasey_init;
	uint32_t phasex_step;
	uint32_t phasey_step;

	uint32_t bg0;
	uint32_t bg1;
	uint32_t bg_cfg;
	uint32_t bg_bpp;
	uint32_t bg_pack;
	uint32_t bg_ystride;

#ifdef CONFIG_MSM_MDP31
	uint32_t src_xy;
	uint32_t src_img_sz;
	uint32_t dst_xy;
	uint32_t bg_xy;
	uint32_t bg_img_sz;
	uint32_t bg_alpha_sel;

	uint32_t scale_cfg;
	uint32_t csc_cfg;
#endif
};

struct mdp_info;
struct mdp_rect;
struct mdp_blit_req;
struct fb_info;

#ifdef CONFIG_FB_MSM_MDP_PPP
int mdp_get_bytes_per_pixel(int format);
int mdp_ppp_blit(struct mdp_info *mdp, struct fb_info *fb,
		 struct mdp_blit_req *req);
void mdp_ppp_handle_isr(struct mdp_info *mdp, uint32_t mask);
int mdp_ppp_blit_and_wait(struct mdp_info *mdp, struct mdp_blit_req *req,
			  struct file *src_file, unsigned long src_start,
			  unsigned long src_len, struct file *dst_file,
			  unsigned long dst_start, unsigned long dst_len);

/* these must be provided by h/w specific ppp files */
void mdp_ppp_init_scale(struct mdp_info *mdp);
int mdp_ppp_cfg_scale(struct mdp_info *mdp, struct ppp_regs *regs,
		 struct mdp_rect *src_rect, struct mdp_rect *dst_rect,
		 uint32_t src_format, uint32_t dst_format);
int mdp_ppp_load_blur(struct mdp_info *mdp);
int mdp_ppp_cfg_edge_cond(struct mdp_blit_req *req, struct ppp_regs *regs);
int mdp_ppp_validate_blit(struct mdp_info *mdp, struct mdp_blit_req *req);
int mdp_ppp_do_blit(struct mdp_info *mdp, struct mdp_blit_req *req,
		    struct file *src_file, unsigned long src_start,
		    unsigned long src_len, struct file *dst_file,
		    unsigned long dst_start, unsigned long dst_len);

#else

static inline int mdp_get_bytes_per_pixel(int format) { return -1; }
static inline int mdp_ppp_blit(struct mdp_info *mdp, struct fb_info *fb,
			       struct mdp_blit_req *req) { return -EINVAL; }
static inline void mdp_ppp_handle_isr(struct mdp_info *mdp, uint32_t mask) {}
static inline int mdp_ppp_blit_and_wait(struct mdp_info *mdp,
		struct mdp_blit_req *req, struct file *src_file,
		unsigned long src_start, unsigned long src_len,
		struct file *dst_file, unsigned long dst_start,
		unsigned long dst_len) { return 0; }

static inline void mdp_ppp_init_scale(struct mdp_info *mdp) {}
static inline int mdp_ppp_cfg_scale(struct mdp_info *mdp, struct ppp_regs *regs,
		 struct mdp_rect *src_rect, struct mdp_rect *dst_rect,
		 uint32_t src_format, uint32_t dst_format) { return 0; }
static inline int mdp_ppp_load_blur(struct mdp_info *mdp) { return 0; }
static inline int mdp_ppp_cfg_edge_cond(struct mdp_blit_req *req, struct ppp_regs *regs) { return 0; }
static inline int mdp_ppp_validate_blit(struct mdp_info *mdp, struct mdp_blit_req *req) { return -EINVAL; }
static inline int mdp_ppp_do_blit(struct mdp_info *mdp,
		struct mdp_blit_req *req,
		struct file *src_file, unsigned long src_start,
		unsigned long src_len, struct file *dst_file,
		unsigned long dst_start, unsigned long dst_len) { return 0; }


#endif /* CONFIG_FB_MSM_MDP_PPP */

#endif /* _VIDEO_MSM_MDP_PPP_H_ */
