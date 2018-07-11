/*
 * STP PC Histogram support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_STP_PC_HISTOGRAM_H__
#define __PAINTBOX_STP_PC_HISTOGRAM_H__

#include "paintbox-common.h"

void stp_pc_histogram_clear(struct paintbox_stp *stp,
		size_t inst_mem_size_in_instructions);

int stp_pc_histogram_clear_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long stp_mask);

int stp_pc_histogram_enable(struct paintbox_data *pb, uint64_t stp_mask);

int stp_pc_histogram_enable_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long stp_mask);

int stp_pc_histogram_read_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

#endif /* __PAINTBOX_STP_PC_HISTOGRAM_H__ */
