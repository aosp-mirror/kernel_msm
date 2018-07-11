/*
 * Line Buffer Pool Support for Paintbox IPU
 *
 * Copyright (C) 2015 Google, Inc.
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <uapi/paintbox_v2.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-lbp.h"
#include "paintbox-lbp-debug.h"
#include "paintbox-power.h"
#include "paintbox-regs.h"
#include "paintbox-sram.h"
#include "paintbox-stp.h"

/* The IPU uses a 4x4 block when transferring pixels */
#define LB_BLOCK_TRANSFER_HEIGHT 4
#define LB_BLOCK_TRANSFER_WIDTH  4

/* TODO:  Temporarily make the line buffer configuration validation a
 * debug only operation.  b/62353362
 */
#ifdef DEBUG
static int validate_lb_config(struct paintbox_data *pb,
		struct paintbox_lbp *lbp, struct line_buffer_config *lb_config)
{
	/* TODO: Need to figure out how the broadcast id will be
	 * expressed.
	 */
	if (lb_config->lb_id < 0 || lb_config->lb_id >= pb->lbp.max_lbs) {
		dev_err(pb->dev, "%s: invalid line buffer id %d\n",
				__func__, lb_config->lb_id);
		return -EINVAL;
	}

	if (lb_config->num_reuse_rows > LB_CTRL0_REUSE_ROWS_MAX) {
		dev_err(pb->dev,
				"%s: lb%u lb%u: invalid reuse rows, %u (max %llu)\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->num_reuse_rows,
				LB_CTRL0_REUSE_ROWS_MAX);
		return -EINVAL;
	}

	if (lb_config->num_read_ptrs > pb->lbp.max_rptrs) {
		dev_err(pb->dev,
				"%s: lb%u lb%u: invalid max read ptrs, %u (max %u)\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->num_read_ptrs,
				pb->lbp.max_rptrs);
		return -EINVAL;
	}

	if (lb_config->fb_rows > pb->lbp.max_fb_rows) {
		dev_err(pb->dev,
				"%s: lbp%u lb%u invalid fb_rows, %u (max %u)\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->fb_rows,
				pb->lbp.max_fb_rows);
		return -EINVAL;
	}

	if (lb_config->x_offset_pixels < LB_OFFSET_OFFSET_X_MIN ||
			lb_config->x_offset_pixels > LB_OFFSET_OFFSET_X_MAX) {
		dev_err(pb->dev,
				"%s: lbp%u lb%u x offset out of bounds, %d (min %d max %d)\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->x_offset_pixels,
				LB_OFFSET_OFFSET_X_MIN, LB_OFFSET_OFFSET_X_MAX);
		return -ERANGE;
	}

	if (lb_config->y_offset_pixels < LB_OFFSET_OFFSET_Y_MIN ||
			lb_config->y_offset_pixels > LB_OFFSET_OFFSET_Y_MAX) {
		dev_err(pb->dev,
				"%s: lbp%u lb%u y offset out of bounds, %d (min %d max %d)\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->y_offset_pixels,
				LB_OFFSET_OFFSET_Y_MIN, LB_OFFSET_OFFSET_Y_MAX);
		return -ERANGE;
	}

	if (lb_config->chan_offset_pixels > LB_OFFSET_OFFSET_CHAN_MAX) {
		dev_err(pb->dev,
				"%s: lbp%u lb%u invalid CHAN offset, %u (max %llu)\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->chan_offset_pixels,
				LB_OFFSET_OFFSET_CHAN_MAX);
		return -EINVAL;
	}

	if ((lb_config->ipu_fb_base_addr & LB_BASE_FB_BASE_ALIGN_MASK) != 0) {
		dev_err(pb->dev,
				"%s: lbp%u lb%u FB base alignment error, 0x%x\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->ipu_fb_base_addr);
		return -EINVAL;
	}

	if ((lb_config->ipu_sb_base_addr & LB_BASE_SB_BASE_ALIGN_MASK) != 0) {
		dev_err(pb->dev,
				"%s: lbp%u lb%u: SB base alignment error, 0x%x\n",
				__func__, lb_config->lb_pool_id,
				lb_config->lb_id, lb_config->ipu_fb_base_addr);
		return -EINVAL;
	}

	return 0;
}
#endif

/* The caller to this function must hold pb->lock */
int validate_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, int pool_id)
{
	if (pool_id >= pb->lbp.num_lbps) {
		dev_err(pb->dev, "%s: invalid lb pool id %d\n", __func__,
				pool_id);
		return -EINVAL;
	}

	if (pb->lbp.lbps[pool_id].session != session) {
		dev_err(pb->dev, "%s: access error, lb pool id %d\n", __func__,
				pool_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
struct paintbox_lbp *get_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, int pool_id, int *err)
{
	int ret;

	ret = validate_lbp(pb, session, pool_id);
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	*err = 0;
	return &pb->lbp.lbps[pool_id];
}

/* The caller to this function must hold pb->lock */
struct paintbox_lb *get_lb(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int lbp_id,
		unsigned int lb_id, int *err)
{
	struct paintbox_lbp *lbp;
	struct paintbox_lb *lb;
	int ret;

	lbp = get_lbp(pb, session, lbp_id, &ret);
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	if (lb_id >= pb->lbp.max_lbs) {
		dev_err(pb->dev, "%s: lbp%u invalid lb id %u\n", __func__,
				lbp_id, lb_id);
		*err = -EINVAL;
		return NULL;
	}

	lb = &lbp->lbs[lb_id];

	if (!lb->configured) {
		dev_err(pb->dev, "%s: lbp%u lb%u not configured\n", __func__,
				lbp_id, lb_id);
		*err = -EINVAL;
		return NULL;
	}

	*err = 0;
	return lb;
}

/* The caller to this function must hold pb->lock and must have set the LBP_SEL
 * register.
 */
static void reset_line_buffer(struct paintbox_data *pb, struct paintbox_lb *lb)
{
	uint64_t val, reset_mask;

	reset_mask = 1ULL << (lb->lb_id + LBP_CTRL_LB_RESET_SHIFT);

	/* This register is not self clearing, we need to clear the reset bit */
	val = lb->lbp->regs.lbp_ctrl;
	val |= reset_mask;
	paintbox_writeq(pb->dev, val, IPU_CSR_LBP_OFFSET + LBP_CTRL);
	val &= ~reset_mask;
	paintbox_writeq(pb->dev, val, IPU_CSR_LBP_OFFSET + LBP_CTRL);
}

/* The caller to this function must hold pb->lock */
void reset_lb(struct paintbox_data *pb, unsigned int lbp_id, unsigned int lb_id)
{
	paintbox_lb_select(pb, lbp_id, lb_id);
	reset_line_buffer(pb, &pb->lbp.lbps[lbp_id].lbs[lb_id]);
}

/* The caller to this function must hold pb->lock */
void release_lbp(struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_lbp *lbp)
{
	dev_dbg(pb->dev, "lbp%u release\n", lbp->pool_id);

	/* The LBP access control masks are not implemented on the V0 IPU.
	 */
	disable_stp_access_to_lbp(pb, session, lbp);

	/* Disable all line buffers within the pool. */
	paintbox_lbp_select(pb, lbp->pool_id);
	lbp->regs.lbp_ctrl = 0;
	paintbox_writeq(pb->dev, 0, IPU_CSR_LBP_OFFSET + LBP_CTRL);

#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
	paintbox_pm_lbp_disable(pb, lbp);
#endif

	/* Remove the line buffer pool from the session. */
	list_del(&lbp->session_entry);
#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
	if (WARN_ON(--session->lbp_count < 0))
		session->lbp_count = 0;
#endif
	lbp->session = NULL;
	pb->lbp.available_lbp_mask |= 1ULL << lbp->pool_id;
}

/* The caller to this function must hold pb->lock */
static void paintbox_lbp_init_regs(struct paintbox_data *pb,
		struct paintbox_lbp *lbp)
{
	unsigned int i;

	paintbox_lb_select(pb, lbp->pool_id, LBP_SEL_LB_SEL_M);

	paintbox_writeq(pb->dev, LB_CTRL0_DEF, IPU_CSR_LBP_OFFSET +
			LB_CTRL0);
	paintbox_writeq(pb->dev, LB_OFFSET_DEF, IPU_CSR_LBP_OFFSET +
			LB_OFFSET);
	paintbox_writel(pb->dev, LB_BDRY_DEF, IPU_CSR_LBP_OFFSET + LB_BDRY);
	paintbox_writel(pb->dev, LB_IMG_SIZE_DEF, IPU_CSR_LBP_OFFSET +
			LB_IMG_SIZE);
	paintbox_writel(pb->dev, LB_SB_SIZE_DEF, IPU_CSR_LBP_OFFSET +
			LB_SB_SIZE);
	paintbox_writel(pb->dev, LB_BASE_DEF, IPU_CSR_LBP_OFFSET + LB_BASE);
	paintbox_writel(pb->dev, LB_L_PARAM_DEF, IPU_CSR_LBP_OFFSET +
			LB_L_PARAM);

	for (i = 0; i < pb->lbp.max_lbs; i++) {
		struct paintbox_lb *lb = &lbp->lbs[i];

		lb->regs.lb_ctrl0 = LB_CTRL0_DEF;
		lb->regs.lb_offset = LB_OFFSET_DEF;
		lb->regs.lb_bdry = LB_BDRY_DEF;
		lb->regs.lb_img_size = LB_IMG_SIZE_DEF;
		lb->regs.lb_sb_size = LB_SB_SIZE_DEF;
		lb->regs.lb_base = LB_BASE_DEF;
		lb->regs.lb_l_param = LB_L_PARAM_DEF;
	}
}

/* The caller to this function must hold pb->lock */
int allocate_lbp(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned int pool_id)
{
	struct paintbox_lbp *lbp;

	lbp = &pb->lbp.lbps[pool_id];
	if (lbp->session) {
		dev_err(pb->dev, "%s: access error, lbp id %d\n", __func__,
				pool_id);
		return -EACCES;
	}

	lbp->session = session;
	list_add_tail(&lbp->session_entry, &session->lbp_list);
#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
	session->lbp_count++;
#endif

#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
	paintbox_pm_lbp_enable(pb, lbp);
#endif

	/* paintbox_lbp_init_regs selects the lbp and lb broadcast */
	paintbox_lbp_init_regs(pb, lbp);
	paintbox_writeq(pb->dev, LBP_CTRL_LBP_RESET_MASK,
			IPU_CSR_LBP_OFFSET + LBP_CTRL);
	lbp->regs.lbp_ctrl = 0;
	paintbox_writeq(pb->dev, 0, IPU_CSR_LBP_OFFSET + LBP_CTRL);

	/* The LBP access control masks are not implemented on the V0 IPU.
	 */
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	/* Grant access to all STPs in this session. */
	enable_stp_access_to_lbp(pb, session, lbp);
#endif
	pb->lbp.available_lbp_mask &= ~(1ULL << pool_id);
	dev_dbg(pb->dev, "lbp%u allocated\n", pool_id);

	return 0;
}

int allocate_lbp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret;
	unsigned int pool_id = (unsigned int)arg;

	if (pool_id >= pb->lbp.num_lbps) {
		dev_err(pb->dev, "%s: invalid lbp id %d\n", __func__, pool_id);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	ret = allocate_lbp(pb, session, pool_id);
	if (ret < 0)
		dev_err(pb->dev, "%s: allocate lbp id %d error %d\n",
				__func__, pool_id, ret);
	mutex_unlock(&pb->lock);

	return ret;
}

int release_lbp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int pool_id = (unsigned int)arg;
	struct paintbox_lbp *lbp;
	int ret = 0;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, pool_id, &ret);
	if (ret == 0) {
		release_lbp(pb, session, lbp);
		signal_completion_on_first_alloc_waiter(pb);
	}
	mutex_unlock(&pb->lock);

	return ret;
}

static int write_l_param_register(struct paintbox_data *pb,
		struct paintbox_lb *lb)
{
	unsigned int width_rounded, sb_cols_rounded;
	uint32_t l_inc, l_width, line_ratio, lb_l_param;

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	switch (lb->fb_rows) {
	case 1:
		line_ratio = 16;
		break;
	case 2:
		line_ratio = 8;
		break;
	default:
		line_ratio = 4;
	};
#else
	line_ratio = 4;
#endif

	width_rounded = (lb->width_pixels + line_ratio - 1) / line_ratio;
	sb_cols_rounded =  (lb->sb_cols + line_ratio - 1) / line_ratio;

	/* Linear address increment:
	 * (ROUND_UP(img_width, line_ratio) / line_ratio
	 */
	l_inc = width_rounded;
	if (l_inc > LB_L_PARAM_L_INC_MAX) {
		dev_err(pb->dev,
				"%s: lbp%u lb%u invalid l_inc valid %u (max %llu)\n",
				__func__, lb->lbp->pool_id, lb->lb_id, l_inc,
				LB_L_PARAM_L_INC_MAX);
		return -EINVAL;
	}

	/* Capacity of the linear space in 256 words.
	 * ROUND_UP(img_width, line_ratio) / line_ratio +
	 *   ROUND_UP(sb_cols, line_ratio) / line_ratio
	 */
	l_width = width_rounded + sb_cols_rounded;
	if (l_width > LB_L_PARAM_L_WIDTH_MAX) {
		dev_err(pb->dev,
				"%s: lbp%u lb%u: invalid l_width valid %u (max %llu)\n",
				__func__, lb->lbp->pool_id, lb->lb_id, l_width,
				LB_L_PARAM_L_WIDTH_MAX);
		return -EINVAL;
	}

	lb_l_param = l_inc | (l_width << LB_L_PARAM_L_WIDTH_SHIFT);
	if (lb_l_param != lb->regs.lb_l_param) {
		lb->regs.lb_l_param = lb_l_param;
		paintbox_writel(pb->dev, lb_l_param, IPU_CSR_LBP_OFFSET +
				LB_L_PARAM);
	}

	return 0;
}

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
static int write_lb_sb_delta_register(struct paintbox_data *pb,
		struct paintbox_lb *lb)
{
	unsigned int width_rounded, sb_cols_rounded;
	uint32_t lb_sb_delta;

	if (lb->sb_cols == 0) {
		paintbox_writel(pb->dev, 0, IPU_CSR_LBP_OFFSET +
				LB_SB_DELTA);
		return 0;
	}

	width_rounded = (lb->width_pixels + LB_BLOCK_TRANSFER_WIDTH - 1) /
			LB_BLOCK_TRANSFER_WIDTH;
	sb_cols_rounded = (lb->sb_cols + LB_BLOCK_TRANSFER_HEIGHT - 1) /
			LB_BLOCK_TRANSFER_HEIGHT;

	lb_sb_delta = sb_cols_rounded - width_rounded % sb_cols_rounded;

	if (lb_sb_delta > LB_SB_DELTA_SB_DELTA_M) {
		dev_err(pb->dev,
				"%s: lbp%u lb%u: invalid lb_sb_delta %u (max %llu)\n",
				__func__, lb->lbp->pool_id, lb->lb_id,
				lb_sb_delta, LB_SB_DELTA_SB_DELTA_M);
		return -EINVAL;
	}

	paintbox_writel(pb->dev, lb_sb_delta, IPU_CSR_LBP_OFFSET +
				LB_SB_DELTA);

	return 0;
}
#endif

static int paintbox_setup_lb(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct line_buffer_config_v2 *config)
{
	struct paintbox_lbp *lbp;
	struct paintbox_lb *lb;
	uint64_t ctrl, lb_ctrl0, lb_offset;
	uint32_t lb_bdry, lb_base, lb_img_size, lb_sb_size;
#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	uint32_t lb_l_param;
#endif
	int ret;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, config->base.lb_pool_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	lb = &lbp->lbs[config->base.lb_id];

#ifdef DEBUG
	ret = validate_lb_config(pb, lbp, &config->base);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}
#endif

	switch (config->base.padding.method) {
	case IPU_PADDING_DO_NOT_PAD:
		lb_bdry = LB_BDRY_BDRY_VAL_CLAMP;
		break;
	case IPU_PADDING_CONSTANT:
		lb_bdry = (config->base.padding.value_or_period <<
				LB_BDRY_BDRY_VAL_SHIFT) |
				LB_BDRY_BDRY_VAL_CLAMP;
		break;
	case IPU_PADDING_PERIODIC:
		lb_bdry = (config->base.padding.value_or_period <<
				LB_BDRY_BDRY_VAL_SHIFT) |
				LB_BDRY_BDRY_VAL_REPEAT;
		break;
	case IPU_PADDING_SYMMETRIC:
		lb_bdry = (config->base.padding.value_or_period <<
				LB_BDRY_BDRY_VAL_SHIFT) |
				LB_BDRY_BDRY_VAL_REFLECT;
		break;
	default:
		dev_err(pb->dev,
				"%s: lbp%u lb%u: invalid padding method, %d\n",
				__func__, config->base.lb_pool_id,
				config->base.lb_id,
				config->base.padding.method);
		mutex_unlock(&pb->lock);
		return ret;
	}

	lb->fb_rows = config->base.fb_rows;
	lb->num_read_ptrs = config->base.num_read_ptrs;
	lb->num_channels = config->base.num_channels;
	lb->width_pixels = config->base.width_pixels;
	lb->height_pixels = config->base.height_pixels;
	lb->sb_cols = config->base.sb_cols;
	lb->sb_rows = config->base.sb_rows;

	paintbox_lb_select(pb, config->base.lb_pool_id, config->base.lb_id);

	/* Disable the line buffer before configuring it in case there is an
	 * active configuration.  Setting the ENA count to the line buffer id
	 * will disable this line buffer and leave all the earlier ones
	 * enabled.
	 */
	ctrl = lbp->regs.lbp_ctrl;
	ctrl &= ~(1ULL << config->base.lb_id);
	ctrl |= 1ULL << (lb->lb_id + LBP_CTRL_LB_RESET_SHIFT);
	lbp->regs.lbp_ctrl = ctrl;
	paintbox_writeq(pb->dev, ctrl, IPU_CSR_LBP_OFFSET + LBP_CTRL);

	lb_ctrl0 = config->base.num_read_ptrs;
	lb_ctrl0 |= config->base.num_channels << LB_CTRL0_NUM_CHAN_SHIFT;
	lb_ctrl0 |= config->base.fb_rows << LB_CTRL0_FB_ROWS_SHIFT;
	lb_ctrl0 |= ((uint64_t)config->base.num_reuse_rows) <<
			LB_CTRL0_REUSE_ROWS_SHIFT;
	if (config->enable_chaining)
		lb_ctrl0 |= LB_CTRL0_EN_CHAIN_MASK;

	if (config->addr_mode_is_physical)
		lb_ctrl0 |= LB_CTRL0_ADDR_MODE_MASK;

	if (lb_ctrl0 != lb->regs.lb_ctrl0) {
		lb->regs.lb_ctrl0 = lb_ctrl0;
		paintbox_writeq(pb->dev, lb_ctrl0, IPU_CSR_LBP_OFFSET +
				LB_CTRL0);
	}

	lb_offset = (uint64_t)config->base.x_offset_pixels &
			LB_OFFSET_OFFSET_X_MASK;
	lb_offset |= ((uint64_t)config->base.y_offset_pixels &
			LB_OFFSET_OFFSET_Y_M) << LB_OFFSET_OFFSET_Y_SHIFT;
	lb_offset |= ((uint64_t)config->base.chan_offset_pixels &
			LB_OFFSET_OFFSET_CHAN_M) << LB_OFFSET_OFFSET_CHAN_SHIFT;
	lb_offset |= ((uint64_t)config->base.fb_offset_pixels &
			LB_OFFSET_FB_OFFSET_M) << LB_OFFSET_FB_OFFSET_SHIFT;
	if (lb_offset != lb->regs.lb_offset) {
		lb->regs.lb_offset = lb_offset;
		paintbox_writeq(pb->dev, lb_offset, IPU_CSR_LBP_OFFSET +
				LB_OFFSET);
	}

	if (lb_bdry != lb->regs.lb_bdry) {
		lb->regs.lb_bdry = lb_bdry;
		paintbox_writel(pb->dev, lb_bdry, IPU_CSR_LBP_OFFSET +
				LB_BDRY);
	}

	lb_img_size = config->base.height_pixels <<
			LB_IMG_SIZE_IMG_HEIGHT_SHIFT |
			config->base.width_pixels;
	if (lb_img_size != lb->regs.lb_img_size) {
		lb->regs.lb_img_size = lb_img_size;
		paintbox_writel(pb->dev, lb_img_size, IPU_CSR_LBP_OFFSET +
				LB_IMG_SIZE);
	}

	lb_sb_size = config->base.sb_rows << LB_SB_SIZE_SB_ROWS_SHIFT |
			config->base.sb_cols;
	if (lb_sb_size != lb->regs.lb_sb_size) {
		lb->regs.lb_sb_size = lb_sb_size;
		paintbox_writel(pb->dev, lb_sb_size, IPU_CSR_LBP_OFFSET +
				LB_SB_SIZE);
	}

	lb_base = config->base.ipu_fb_base_addr >> LB_BASE_FB_BASE_ALIGN_SHIFT;
	lb_base |= (config->base.ipu_sb_base_addr >>
			LB_BASE_SB_BASE_ALIGN_SHIFT) <<
			LB_BASE_SB_BASE_ADDR_SHIFT;
	if (lb_base != lb->regs.lb_base) {
		lb->regs.lb_base = lb_base;
		paintbox_writel(pb->dev, lb_base, IPU_CSR_LBP_OFFSET +
				LB_BASE);
	}

	ret = write_l_param_register(pb, lb);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = write_lb_sb_delta_register(pb, lb);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* Enable and initialize the line buffer
	 * The init bit is not self clearing, we need to clear the init bit.
	 */
	ctrl |= 1ULL << config->base.lb_id;
	ctrl &= ~(1ULL << (lb->lb_id + LBP_CTRL_LB_RESET_SHIFT));
	ctrl |= 1ULL << (config->base.lb_id + LBP_CTRL_LB_INIT_SHIFT);
	paintbox_writeq(pb->dev, ctrl, IPU_CSR_LBP_OFFSET + LBP_CTRL);
	ctrl &= ~(1ULL << (config->base.lb_id + LBP_CTRL_LB_INIT_SHIFT));
	lbp->regs.lbp_ctrl = ctrl;
	paintbox_writeq(pb->dev, ctrl, IPU_CSR_LBP_OFFSET + LBP_CTRL);

	lb->configured = true;

	LOG_LINE_BUFFER_SETUP(pb, &config->base);

	LOG_LBP_REGISTERS(pb, lbp, lb);

	mutex_unlock(&pb->lock);

	return 0;
}

int paintbox_setup_lb_ioctl_v0(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct line_buffer_config __user *user_config;
	struct line_buffer_config_v2 config;

	user_config = (struct line_buffer_config __user *)arg;

	/* The V2 line buffer configuration is a super-set of the v0 line buffer
	 * config, memset the structure to initialize the v2 portion of the
	 * structure to 0.
	 */
	memset(&config, 0, sizeof(config));

	/* Copy the v0 portion of the line buffer structure into the kernel
	 * buffer.
	 */
	if (copy_from_user(&config, user_config,
			sizeof(struct line_buffer_config)))
		return -EFAULT;

	return paintbox_setup_lb(pb, session, &config);
}

int paintbox_setup_lb_ioctl_v2(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct line_buffer_config_v2 __user *user_config;
	struct line_buffer_config_v2 config;

	user_config = (struct line_buffer_config_v2 __user *)arg;
	if (copy_from_user(&config, user_config, sizeof(config)))
		return -EFAULT;

	return paintbox_setup_lb(pb, session, &config);
}

/* The caller to this function must hold pb->lock */
static int lbp_sram_write_word(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config, const uint8_t *buf,
		uint32_t ram_ctrl_addr)
{
	unsigned int attempts = 0;

	paintbox_lbp_select(pb, sram_config->core_id);

	write_ram_data_registers(pb, buf, IPU_CSR_LBP_OFFSET + LBP_RAM_DATA0,
			LBP_DATA_REG_COUNT);

	paintbox_writel(pb->dev, LBP_RAM_CTRL_RUN_MASK |
			LBP_RAM_CTRL_WRITE_MASK | ram_ctrl_addr,
			IPU_CSR_LBP_OFFSET + LBP_RAM_CTRL);

	while (paintbox_readl(pb->dev, IPU_CSR_LBP_OFFSET + LBP_RAM_CTRL) &
			LBP_RAM_CTRL_RUN_MASK) {
		if (++attempts >= MAX_MEMORY_ACCESS_ATTEMPTS) {
			dev_err(pb->dev, "%s: write timeout\n", __func__);
			return -ETIMEDOUT;
		}

		usleep_range(MIN_RAM_ACCESS_SLEEP, MAX_RAM_ACCESS_SLEEP);
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
static int lbp_sram_read_word(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config, uint8_t *buf,
		uint32_t ram_ctrl_addr)
{
	unsigned int attempts = 0;

	paintbox_lbp_select(pb, sram_config->core_id);

	paintbox_writel(pb->dev, LBP_RAM_CTRL_RUN_MASK | ram_ctrl_addr,
			IPU_CSR_LBP_OFFSET + LBP_RAM_CTRL);

	while (paintbox_readl(pb->dev, IPU_CSR_LBP_OFFSET + LBP_RAM_CTRL) &
			LBP_RAM_CTRL_RUN_MASK) {
		if (++attempts >= MAX_MEMORY_ACCESS_ATTEMPTS) {
			dev_err(pb->dev, "%s: read timeout\n", __func__);
			return -ETIMEDOUT;
		}

		usleep_range(MIN_RAM_ACCESS_SLEEP, MAX_RAM_ACCESS_SLEEP);
	}

	read_ram_data_registers(pb, buf, IPU_CSR_LBP_OFFSET + LBP_RAM_DATA0,
			LBP_DATA_REG_COUNT);

	return 0;
}

static void create_lbp_sram_config(struct paintbox_sram_config *sram_config,
		unsigned int lbp_id, bool pad_to_align)
{
	sram_config->core_id = lbp_id;
	sram_config->ram_ctrl_target = 0;
	sram_config->ram_data_mode = RAM_DATA_MODE_NORMAL;
	sram_config->sram_word_bytes = LBP_DATA_REG_COUNT * IPU_REG_WIDTH_BYTES;
	sram_config->write_word = &lbp_sram_write_word;
	sram_config->read_word = &lbp_sram_read_word;
	sram_config->pad_to_align = pad_to_align;
}

static int validate_sram_transfer(struct paintbox_data *pb,
		struct paintbox_lbp *lbp, uint32_t sram_byte_addr,
		size_t len_bytes)
{
	if (sram_byte_addr + len_bytes > pb->lbp.mem_size_bytes) {
		dev_err(pb->dev,
				"%s: lbp%u memory transfer out of range: SRAM addr 0x%08x + %lu > %u bytes\n",
				__func__, lbp->pool_id, sram_byte_addr,
				len_bytes, pb->lbp.mem_size_bytes);
		return -ERANGE;
	}

	return 0;
}

int write_lbp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_write __user *user_req;
	struct ipu_sram_write req;
	struct paintbox_sram_config sram_config;
	struct paintbox_lbp *lbp;
	int ret;

	user_req = (struct ipu_sram_write __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, req.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = validate_sram_transfer(pb, lbp, req.sram_byte_addr,
			req.len_bytes);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	create_lbp_sram_config(&sram_config, req.id, req.pad_to_align);

	ret = sram_write_user_buffer(pb, &sram_config, req.sram_byte_addr,
			req.buf, req.len_bytes);
	if (ret < 0)
		dev_err(pb->dev,
				"%s: lbp%u write error addr: 0x%04x ram_ctrl 0x%08x err = %d\n",
				__func__, req.id, req.sram_byte_addr,
				paintbox_readl(pb->dev, IPU_CSR_LBP_OFFSET +
				LBP_RAM_CTRL), ret);

	mutex_unlock(&pb->lock);

	return ret;
}

int read_lbp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_read __user *user_req;
	struct ipu_sram_read req;
	struct paintbox_sram_config sram_config;
	struct paintbox_lbp *lbp;
	int ret;

	user_req = (struct ipu_sram_read __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, req.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = validate_sram_transfer(pb, lbp, req.sram_byte_addr,
			req.len_bytes);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	create_lbp_sram_config(&sram_config, req.id, false /* pad_to_align */);

	ret = sram_read_user_buffer(pb, &sram_config, req.sram_byte_addr,
			req.buf, req.len_bytes);
	if (ret < 0)
		dev_err(pb->dev,
				"%s: lbp%u read error addr: 0x%04x ram_ctrl 0x%08x err = %d\n",
				__func__, req.id, req.sram_byte_addr,
				paintbox_readl(pb->dev, IPU_CSR_LBP_OFFSET +
				LBP_RAM_CTRL), ret);

	mutex_unlock(&pb->lock);

	return ret;
}

int reset_lbp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int lbp_id = (unsigned int)arg;
	struct paintbox_lbp *lbp;
	uint64_t ctrl;
	int ret;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, lbp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(pb->dev, "lbp%u reset\n", lbp->pool_id);

	paintbox_lbp_select(pb, lbp->pool_id);

	ctrl = lbp->regs.lbp_ctrl;
	ctrl |= LBP_CTRL_LBP_RESET_MASK;
	paintbox_writeq(pb->dev, ctrl, IPU_CSR_LBP_OFFSET + LBP_CTRL);
	ctrl &= ~LBP_CTRL_LBP_RESET_MASK;
	paintbox_writeq(pb->dev, ctrl, IPU_CSR_LBP_OFFSET + LBP_CTRL);

	mutex_unlock(&pb->lock);

	return 0;
}

int reset_lb_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	struct line_buffer_reset __user *user_req;
	struct line_buffer_reset req;
	struct paintbox_lbp *lbp;
	uint64_t ctrl;
	int ret;

	user_req = (struct line_buffer_reset __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(struct line_buffer_reset)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	lbp = get_lbp(pb, session, req.lbp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	if (req.lb_id >= pb->lbp.max_lbs) {
		dev_err(pb->dev, "%s: lbp%u: invalid lb id %u, max %u\n",
				__func__, req.lbp_id, req.lb_id,
				pb->lbp.max_lbs);
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	dev_dbg(pb->dev, "lbp%u lb%u reset\n",  lbp->pool_id, req.lb_id);

	paintbox_lbp_select(pb, lbp->pool_id);

	ctrl = lbp->regs.lbp_ctrl;
	ctrl |= 1ULL << req.lb_id << LBP_CTRL_LB_RESET_SHIFT;
	paintbox_writeq(pb->dev, ctrl, IPU_CSR_LBP_OFFSET + LBP_CTRL);
	ctrl &= ~(1ULL << req.lb_id << LBP_CTRL_LB_RESET_SHIFT);
	paintbox_writeq(pb->dev, ctrl, IPU_CSR_LBP_OFFSET + LBP_CTRL);

	mutex_unlock(&pb->lock);

	return 0;
}

#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
static int validate_sram_transfer_for_broadcast(struct paintbox_data *pb,
		uint32_t sram_byte_addr, size_t len_bytes)
{
	if (sram_byte_addr + len_bytes > pb->lbp.mem_size_bytes) {
		dev_err(pb->dev,
				"%s: lbp memory transfer out of range: SRAM addr 0x%08x + %lu > %u bytes\n",
				__func__, sram_byte_addr, len_bytes,
				pb->lbp.mem_size_bytes);
		return -ERANGE;
	}

	return 0;
}

int lbp_test_broadcast_write_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_write_broadcast __user *user_req;
	struct ipu_sram_write_broadcast req;
	struct paintbox_sram_config sram_config;
	int ret;

	user_req = (struct ipu_sram_write_broadcast __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	ret = validate_sram_transfer_for_broadcast(pb,
			req.sram_byte_addr, req.len_bytes);
	if (ret < 0)
		return ret;

	create_lbp_sram_config(&sram_config,
			LBP_SEL_DEF & LBP_SEL_LBP_SEL_MASK, false);

	/* Entering broadcast path for LBP write when:
	 *	1) there is only 1 session in existence; or
	 *	2) one of the session has all LBPs
	 */
	mutex_lock(&pb->lock);

	if (pb->session_count != 1 && session->lbp_count != pb->lbp.num_lbps) {
		mutex_unlock(&pb->lock);
		return -EBUSY;
	}

	ret = sram_write_user_buffer(pb, &sram_config,
			req.sram_byte_addr, req.buf, req.len_bytes);
	if (ret < 0) {
		dev_err(pb->dev,
				"%s: write error addr: 0x%04x ram_ctrl 0x%08x err = %d\n",
				__func__, req.sram_byte_addr,
				paintbox_readl(pb->dev, IPU_CSR_LBP_OFFSET +
				LBP_RAM_CTRL), ret);
		mutex_unlock(&pb->lock);
		return ret;
	}

	mutex_unlock(&pb->lock);

	return 0;
}
#endif

/* The caller to this function must hold pb->lock and must have selected a pool
 * via paintbox_lbp_select
 */
void paintbox_lbp_set_pmon_rptr_id(struct paintbox_data *pb, uint64_t rptr_id)
{
	uint64_t lbp_ctrl;
	struct paintbox_lbp *lbp = &pb->lbp.lbps[pb->lbp.selected_lbp_id];

	lbp_ctrl = lbp->regs.lbp_ctrl;
	lbp_ctrl &= ~LBP_CTRL_PMON_RD_SEL_MASK;
	lbp_ctrl |= rptr_id << LBP_CTRL_PMON_RD_SEL_SHIFT;
	lbp->regs.lbp_ctrl = lbp_ctrl;
	paintbox_writeq(pb->dev, lbp_ctrl, IPU_CSR_LBP_OFFSET + LBP_CTRL);
}

/* The caller to this function must hold pb->lock */
void paintbox_lbp_release(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_lbp *lbp, *lbp_next;

	list_for_each_entry_safe(lbp, lbp_next, &session->lbp_list,
			session_entry)
		release_lbp(pb, session, lbp);
}

/* All sessions must be released before remove can be called. */
void paintbox_lbp_remove(struct paintbox_data *pb)
{
	unsigned int lbp_id, lb_id;

	for (lbp_id = 0; lbp_id < pb->lbp.num_lbps; lbp_id++) {
		struct paintbox_lbp *lbp = &pb->lbp.lbps[lbp_id];

		for (lb_id = 0; lb_id < pb->lbp.max_lbs; lb_id++) {
			struct paintbox_lb *lb = &lbp->lbs[lb_id];

			paintbox_lb_debug_remove(pb, lb);
		}

		paintbox_lbp_debug_remove(pb, lbp);

		kfree(pb->lbp.lbps[lbp_id].lbs);
	}

	kfree(pb->lbp.lbps);
}

/* Resets shadows and restores power state. */
void paintbox_lbp_post_ipu_reset(struct paintbox_data *pb)
{
	unsigned int i;
	struct paintbox_lbp *lbp;

	pb->lbp.selected_lbp_id = LBP_SEL_DEF & LBP_SEL_LBP_SEL_M;
	pb->lbp.selected_lb_id = (LBP_SEL_DEF & LBP_SEL_LB_SEL_MASK) >>
			LBP_SEL_LB_SEL_SHIFT;

	for (i = 0; i < pb->lbp.num_lbps; i++) {
		lbp = &pb->lbp.lbps[i];
		lbp->regs.lbp_ctrl = LBP_CTRL_DEF;

#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
		if (lbp->pm_enabled)
			paintbox_pm_lbp_enable(pb, lbp);
#endif

		paintbox_lbp_init_regs(pb, lbp);
	}
}

static int init_lbp(struct paintbox_data *pb, unsigned int lbp_index)
{
	struct paintbox_lbp *lbp;
	unsigned int i;

	lbp = &pb->lbp.lbps[lbp_index];

	/* Store pool id with object as a convenience to avoid doing a lookup
	 * later on.
	 */
	lbp->pool_id = lbp_index;

	lbp->lbs = kcalloc(pb->lbp.max_lbs, sizeof(struct paintbox_lb),
			GFP_KERNEL);
	if (!lbp->lbs)
		return -ENOMEM;

	lbp->regs.lbp_ctrl = LBP_CTRL_DEF;

	paintbox_lbp_debug_init(pb, lbp);

	for (i = 0; i < pb->lbp.max_lbs; i++) {
		struct paintbox_lb *lb = &lbp->lbs[i];

		lb->lbp = lbp;
		lb->lb_id = i;

		paintbox_lb_debug_init(pb, lbp, lb);
	}

	return 0;
}

int paintbox_lbp_init(struct paintbox_data *pb)
{
	unsigned int i;
	uint32_t caps;
	int ret;

	pb->lbp.lbps = kcalloc(pb->lbp.num_lbps, sizeof(struct paintbox_lbp),
			GFP_KERNEL);
	if (!pb->lbp.lbps)
		return -ENOMEM;

	pb->lbp.selected_lbp_id = LBP_SEL_DEF & LBP_SEL_LBP_SEL_M;
	pb->lbp.selected_lb_id = (LBP_SEL_DEF & LBP_SEL_LB_SEL_MASK) >>
			LBP_SEL_LB_SEL_SHIFT;

	/* Read LBP/LB capabilities from LBP0 since that is always powered.
	 * The capabilities are the same for the other LBPs.
	 */
	paintbox_lbp_select(pb, 0);
	caps = paintbox_readl(pb->dev, IPU_CSR_LBP_OFFSET + LBP_CAP0);

	pb->lbp.max_lbs = caps & LBP_CAP0_MAX_LB_MASK;
	pb->lbp.max_rptrs = (caps & LBP_CAP0_MAX_RPTR_MASK) >>
			LBP_CAP0_MAX_RPTR_SHIFT;
	pb->lbp.max_channels = (caps & LBP_CAP0_MAX_CHAN_MASK) >>
			LBP_CAP0_MAX_CHAN_SHIFT;
	pb->lbp.max_fb_rows = (caps & LBP_CAP0_MAX_FB_ROWS_MASK) >>
			LBP_CAP0_MAX_FB_ROWS_SHIFT;
	pb->lbp.mem_size_bytes = paintbox_readl(pb->dev, IPU_CSR_LBP_OFFSET +
			LBP_CAP1);

	for (i = 0; i < pb->lbp.num_lbps; i++) {
		ret = init_lbp(pb, i);
		if (ret < 0)
			return ret;
	}

	dev_dbg(pb->dev, "lbp: base 0x%08x len %lu max lbs%u\n",
			IPU_CSR_LBP_OFFSET, LBP_BLOCK_LEN, pb->lbp.max_lbs);
	dev_dbg(pb->dev, "\trptrs %u ch %u fbrows %u size %u bytes\n",
			pb->lbp.max_rptrs, pb->lbp.max_channels,
			pb->lbp.max_fb_rows, pb->lbp.mem_size_bytes);

	return 0;
}

