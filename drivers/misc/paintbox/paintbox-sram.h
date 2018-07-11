/*
 * Paintbox programmable SRAM support
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

#ifndef __PAINTBOX_SRAM_H__
#define __PAINTBOX_SRAM_H__

#include <linux/io.h>

#include "paintbox-common.h"

#define RAM_DATA_MODE_NORMAL    0

/* TODO:  Remove once b/30316979 is fixed.  RAM_DATA_MODE_SWAP is
 * needed because the assembler writes the instruction in reverse byte order
 * (due to an issue with the DV tools).
 */
#define RAM_DATA_MODE_SWAP      1

/* TODO:  The conversion to the vector SRAM's column major lane
 * ordering needs to be moved to the runtime so it can be used for both DMA and
 * PIO.
 */
#define RAM_DATA_MODE_COL_MAJOR 2

#define VECTOR_SRAM_LANE_GROUP_SIMD_COLS 4
#define VECTOR_SRAM_LANE_GROUP_SIMD_ROWS 8
#define VECTOR_SRAM_LANE_GROUP_HALO_COLS 1
#define VECTOR_SRAM_LANE_GROUP_HALO_ROWS 2

#define VECTOR_LANE_WIDTH 2
#define VECTOR_LANE_GROUP_WIDTH 4 /* lanes */
#define VECTOR_LANE_GROUP_HEIGHT 2 /* lanes */

#define MIN_RAM_ACCESS_SLEEP       10  /* us */
#define MAX_RAM_ACCESS_SLEEP       100 /* us */
#define MAX_MEMORY_ACCESS_ATTEMPTS 5


struct paintbox_sram_config {
	uint32_t ram_ctrl_target;
	unsigned int ram_data_mode;
	unsigned int core_id;
	size_t sram_word_bytes;
	bool pad_to_align;
	int (*write_word)(struct paintbox_data *pb,
			struct paintbox_sram_config *sram_config,
			const uint8_t *buf, uint32_t ram_ctrl_addr);
	int (*read_word)(struct paintbox_data *pb,
			struct paintbox_sram_config *sram_config, uint8_t *buf,
			uint32_t ram_ctrl_addr);
};

int sram_write_buffer(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config,
		uint32_t sram_byte_addr, const uint8_t *buf, size_t len_bytes);

int sram_write_user_buffer(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config,
		uint32_t sram_byte_addr, const void __user *user_buf,
		size_t len_bytes);

int sram_read_buffer(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config,
		uint32_t sram_byte_addr, uint8_t *buf, size_t len_bytes);

int sram_read_user_buffer(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config,
		uint32_t sram_byte_addr, void __user *user_buf,
		size_t len_bytes);

void write_ram_data_registers(struct paintbox_data *pb, const uint8_t *buf,
		uint32_t data_reg, unsigned int reg_count);

void write_ram_data_registers_swapped(struct paintbox_data *pb,
		const uint8_t *buf, uint32_t data_reg, unsigned int reg_count);

void write_ram_data_registers_column_major(struct paintbox_data *pb,
		const uint8_t *buf);

void read_ram_data_registers(struct paintbox_data *pb, uint8_t *buf,
		uint32_t data_reg, unsigned int reg_count);

void read_ram_data_registers_swapped(struct paintbox_data *pb,
		uint8_t *buf, uint32_t data_reg, unsigned int reg_count);

void read_ram_data_registers_column_major(struct paintbox_data *pb,
		uint8_t *buf);

int alloc_and_copy_from_user(struct paintbox_data *pb, uint8_t **buf,
			const void __user *user_buf, size_t len_bytes);

#endif /* __PAINTBOX_SRAM_H__ */
