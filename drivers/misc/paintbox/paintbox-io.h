/*
 * IO support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
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

#ifndef __PAINTBOX_IO_H__
#define __PAINTBOX_IO_H__

#include <linux/io.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

void paintbox_enable_dma_channel_interrupt(struct paintbox_data *pb,
		unsigned int channel_id);
void paintbox_disable_dma_channel_interrupt(struct paintbox_data *pb,
		unsigned int channel_id);
void paintbox_enable_dma_channel_error_interrupt(struct paintbox_data *pb,
		unsigned int channel_id);
void paintbox_disable_dma_channel_error_interrupt(struct paintbox_data *pb,
		unsigned int channel_id);

void paintbox_enable_stp_interrupt(struct paintbox_data *pb,
		unsigned int stp_id);
void paintbox_disable_stp_interrupt(struct paintbox_data *pb,
		unsigned int stp_id);
void paintbox_enable_stp_error_interrupt(struct paintbox_data *pb,
		unsigned int stp_id);
void paintbox_disable_stp_error_interrupt(struct paintbox_data *pb,
		unsigned int stp_id);

void paintbox_io_apb_post_ipu_reset(struct paintbox_data *pb);

int paintbox_io_apb_init(struct paintbox_data *pb);

/* All sessions must be released before remove can be called. */
void paintbox_io_apb_remove(struct paintbox_data *pb);

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_dump_io_apb_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
#endif

#endif /* __PAINTBOX_IO_H__ */
