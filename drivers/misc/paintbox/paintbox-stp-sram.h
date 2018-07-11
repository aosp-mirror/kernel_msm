/*
 * STP SRAM support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_STP_SRAM_H__
#define __PAINTBOX_STP_SRAM_H__

#include "paintbox-common.h"
#include "paintbox-sram.h"

int create_scalar_sram_config(struct paintbox_sram_config *sram_config,
		unsigned int stp_id, enum sram_target_type sram_target,
		bool swap_data, bool pad_to_align);

int stp_sram_write_word(struct paintbox_data *pb,
		struct paintbox_sram_config *config, const uint8_t *buf,
		uint32_t ram_ctrl_mask);
int stp_sram_read_word(struct paintbox_data *pb,
		struct paintbox_sram_config *config, uint8_t *buf,
		uint32_t ram_ctrl_mask);

int write_stp_scalar_sram_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int read_stp_scalar_sram_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int write_stp_vector_sram_coordinates_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int write_stp_vector_sram_replicate_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int read_stp_vector_sram_coordinates_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

#endif /* __PAINTBOX_STP_SRAM_H__ */
