/*
 * Linebuffer Pool Support for Paintbox IPU
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

#ifndef __PAINTBOX_LBP_H__
#define __PAINTBOX_LBP_H__

#include <linux/io.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"

int allocate_lbp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int release_lbp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int paintbox_setup_lb_ioctl_v0(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int paintbox_setup_lb_ioctl_v2(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int reset_lbp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg);
int reset_lb_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg);
int write_lbp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int read_lbp_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
int lbp_test_broadcast_write_memory_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
#endif

/* The caller to these functions must hold pb->lock */
int validate_lbp(struct paintbox_data *pb, struct paintbox_session *session,
		int pool_id);
struct paintbox_lbp *get_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, int pool_id, int *err);
struct paintbox_lb *get_lb(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int lbp_id,
		unsigned int lb_id, int *err);
int allocate_lbp(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned int pool_id);
void release_lbp(struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_lbp *lbp);
void reset_lb(struct paintbox_data *pb, unsigned int lbp_id,
		unsigned int lb_id);

/* The caller to this function must hold pb->lock and must have selected a pool
 * via paintbox_lbp_select
 */
void paintbox_lbp_set_pmon_rptr_id(struct paintbox_data *pb, uint64_t rptr_id);

/* The caller to this function must hold pb->lock */
static inline void paintbox_lbp_select(struct paintbox_data *pb,
		unsigned int lbp_id)
{
	if (pb->lbp.selected_lbp_id == lbp_id)
		return;

	pb->lbp.selected_lbp_id = lbp_id;

	paintbox_writel(pb->dev, lbp_id | pb->lbp.selected_lb_id <<
			LBP_SEL_LB_SEL_SHIFT, IPU_CSR_LBP_OFFSET + LBP_SEL);
}

/* The caller to this function must hold pb->lock */
static inline void paintbox_lb_select(struct paintbox_data *pb,
		unsigned int lbp_id, unsigned int lb_id)
{
	if (pb->lbp.selected_lbp_id == lbp_id &&
			pb->lbp.selected_lb_id == lb_id)
		return;

	pb->lbp.selected_lbp_id = lbp_id;
	pb->lbp.selected_lb_id = lb_id;

	paintbox_writel(pb->dev, lbp_id | lb_id << LBP_SEL_LB_SEL_SHIFT,
			IPU_CSR_LBP_OFFSET + LBP_SEL);
}

void paintbox_lbp_post_ipu_reset(struct paintbox_data *pb);

int paintbox_lbp_init(struct paintbox_data *pb);

/* The caller to this function must hold pb->lock */
void paintbox_lbp_release(struct paintbox_data *pb,
		struct paintbox_session *session);

/* All sessions must be released before remove can be called. */
void paintbox_lbp_remove(struct paintbox_data *pb);

#endif /* __PAINTBOX_LBP_H__ */
