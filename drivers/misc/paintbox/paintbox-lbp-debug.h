/*
 * Linebuffer Pool Debug Support for Paintbox IPU
 *
 * Copyright (C) 2017 Google, Inc.
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

#ifndef __PAINTBOX_LBP_DEBUG_H__
#define __PAINTBOX_LBP_DEBUG_H__

#include <linux/io.h>

#include "paintbox-common.h"

#ifdef DEBUG
void paintbox_log_lb_setup(struct paintbox_data *pb,
		struct line_buffer_config *config);

#define LOG_LINE_BUFFER_SETUP(pb, config)				\
	paintbox_log_lb_setup(pb, config)
#else
#define LOG_LINE_BUFFER_SETUP(pb, config)				\
do { } while (0)
#endif

#ifdef VERBOSE_DEBUG
void paintbox_log_lbp_registers(struct paintbox_data *pb,
		struct paintbox_lbp *lbp, struct paintbox_lb *lb,
		const char *msg);

#define LOG_LBP_REGISTERS(pb, lbp, lb)		\
	paintbox_log_lbp_registers(pb, lbp, lb, __func__)

#else

#define LOG_LBP_REGISTERS(pb, lbp, lb)		\
do { } while (0)
#endif

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_dump_lbp_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
int paintbox_dump_lb_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
void paintbox_lbp_debug_init(struct paintbox_data *pb,
		struct paintbox_lbp *lbp);
void paintbox_lbp_debug_remove(struct paintbox_data *pb,
		struct paintbox_lbp *lbp);
void paintbox_lb_debug_init(struct paintbox_data *pb, struct paintbox_lbp *lbp,
		struct paintbox_lb *lb);
void paintbox_lb_debug_remove(struct paintbox_data *pb, struct paintbox_lb *lb);
#else
static inline void paintbox_lbp_debug_init(struct paintbox_data *pb,
		struct paintbox_lbp *lbp) { }
static inline void paintbox_lbp_debug_remove(struct paintbox_data *pb,
		struct paintbox_lbp *lbp) { }
static inline void paintbox_lb_debug_init(struct paintbox_data *pb,
		struct paintbox_lbp *lbp, struct paintbox_lb *lb) { }
static inline void paintbox_lb_debug_remove(struct paintbox_data *pb,
		struct paintbox_lb *lb) { }
#endif

#endif /* __PAINTBOX_LBP_DEBUG_H__ */
