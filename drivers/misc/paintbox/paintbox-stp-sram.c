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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"
#include "paintbox-sram.h"
#include "paintbox-stp.h"
#include "paintbox-stp-sram.h"


int stp_sram_write_word(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config, const uint8_t *buf,
		uint32_t ram_ctrl_addr)
{
	unsigned long irq_flags;
	unsigned int attempts = 0;

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, sram_config->core_id);

	/* TODO:  This can be removed once the SWAP and COL_MAJOR
	 * support is moved outside the driver.
	 */
	switch (sram_config->ram_data_mode) {
	case RAM_DATA_MODE_NORMAL:
		write_ram_data_registers(pb, buf, IPU_CSR_STP_OFFSET +
				STP_RAM_DATA0, STP_DATA_REG_COUNT);
		break;
	case RAM_DATA_MODE_SWAP:
		write_ram_data_registers_swapped(pb, buf, IPU_CSR_STP_OFFSET +
				STP_RAM_DATA0, STP_DATA_REG_COUNT);
		break;
	case RAM_DATA_MODE_COL_MAJOR:
		write_ram_data_registers_column_major(pb, buf);
		break;
	};

	paintbox_writel(pb->dev, STP_RAM_CTRL_RUN_MASK |
			STP_RAM_CTRL_WRITE_MASK | ram_ctrl_addr |
			sram_config->ram_ctrl_target,
			IPU_CSR_STP_OFFSET + STP_RAM_CTRL);

	while (paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET + STP_RAM_CTRL) &
			STP_RAM_CTRL_RUN_MASK) {
		spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

		if (++attempts >= MAX_MEMORY_ACCESS_ATTEMPTS) {
			dev_err(pb->dev, "%s: write timeout\n", __func__);
			return -ETIMEDOUT;
		}

		usleep_range(MIN_RAM_ACCESS_SLEEP, MAX_RAM_ACCESS_SLEEP);

		spin_lock_irqsave(&pb->stp.lock, irq_flags);

		/* Make sure the STP_SEL register is set to the correct STP ID
		 * before reading the STP_RAM_CTRL register.  If an interrupt
		 * occurred while stp_lock was not held then the STP_SEL
		 * register may have been changed.
		 */
		paintbox_stp_select(pb, sram_config->core_id);
	}

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	return 0;
}

int stp_sram_read_word(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config, uint8_t *buf,
		uint32_t ram_ctrl_addr)
{
	unsigned long irq_flags;
	unsigned int attempts = 0;

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, sram_config->core_id);

	paintbox_writel(pb->dev, STP_RAM_CTRL_RUN_MASK | ram_ctrl_addr |
			sram_config->ram_ctrl_target,
			IPU_CSR_STP_OFFSET + STP_RAM_CTRL);

	while (paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET + STP_RAM_CTRL) &
			STP_RAM_CTRL_RUN_MASK) {
		spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

		if (++attempts >= MAX_MEMORY_ACCESS_ATTEMPTS) {
			dev_err(pb->dev, "%s: read timeout\n", __func__);
			return -ETIMEDOUT;
		}

		usleep_range(MIN_RAM_ACCESS_SLEEP, MAX_RAM_ACCESS_SLEEP);

		spin_lock_irqsave(&pb->stp.lock, irq_flags);

		/* Make sure the STP_SEL register is set to the correct STP ID
		 * before reading the STP_RAM_CTRL register.  If an interrupt
		 * occurred while stp_lock was not held then the STP_SEL
		 * register may have been changed.
		 */
		paintbox_stp_select(pb, sram_config->core_id);
	}

	/* TODO:  This can be removed once the SWAP and COL_MAJOR
	 * support is moved outside the driver.
	 */
	switch (sram_config->ram_data_mode) {
	case RAM_DATA_MODE_NORMAL:
		read_ram_data_registers(pb, buf, IPU_CSR_STP_OFFSET +
				STP_RAM_DATA0, STP_DATA_REG_COUNT);
		break;
	case RAM_DATA_MODE_SWAP:
		read_ram_data_registers_swapped(pb, buf, IPU_CSR_STP_OFFSET +
				STP_RAM_DATA0, STP_DATA_REG_COUNT);
		break;
	case RAM_DATA_MODE_COL_MAJOR:
		read_ram_data_registers_column_major(pb, buf);
		break;
	};

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	return 0;
}

static int validate_sram_transfer(struct paintbox_data *pb,
		struct paintbox_stp *stp, uint32_t sram_byte_addr,
		enum sram_target_type sram_target, size_t len_bytes)
{
	size_t sram_len_bytes;

	switch (sram_target) {
	case SRAM_TARGET_STP_INSTRUCTION_RAM:
		sram_len_bytes = pb->stp.inst_mem_size_in_instructions *
				STP_INST_SRAM_INSTRUCTION_WIDTH_BYTES;
		break;
	case SRAM_TARGET_STP_CONSTANT_RAM:
		sram_len_bytes = pb->stp.const_mem_size_in_words *
				STP_CONST_SRAM_WORD_WIDTH_BYTES;
		break;
	case SRAM_TARGET_STP_SCALAR_RAM:
		sram_len_bytes = pb->stp.scalar_mem_size_in_words *
				STP_SCALAR_SRAM_WORD_WIDTH_BYTES;
		break;
	default:
		dev_err(pb->dev, "%s: stp%u: invalid ram transfer type\n",
				__func__, stp->stp_id);
		return -EINVAL;
	};

	if (sram_byte_addr + len_bytes > sram_len_bytes) {
		dev_err(pb->dev,
				"%s: stp%u: memory transfer out of range: SRAM target %d addr 0x%08x + %lu > %lu bytes\n",
				__func__, stp->stp_id, sram_target,
				sram_byte_addr, len_bytes, sram_len_bytes);
		return -ERANGE;
	}

	return 0;
}

int create_scalar_sram_config(struct paintbox_sram_config *sram_config,
		unsigned int stp_id, enum sram_target_type sram_target,
		bool swap_data, bool pad_to_align)
{
	switch (sram_target) {
	case SRAM_TARGET_STP_INSTRUCTION_RAM:
		sram_config->ram_ctrl_target = STP_RAM_TARG_INST_RAM <<
				STP_RAM_CTRL_RAM_TARG_SHIFT;
		break;
	case SRAM_TARGET_STP_CONSTANT_RAM:
		sram_config->ram_ctrl_target = STP_RAM_TARG_CNST_RAM <<
				STP_RAM_CTRL_RAM_TARG_SHIFT;
		break;
	case SRAM_TARGET_STP_SCALAR_RAM:
		sram_config->ram_ctrl_target = STP_RAM_TARG_DATA_RAM <<
				STP_RAM_CTRL_RAM_TARG_SHIFT;
		break;
	default:
		return -EINVAL;
	};

	sram_config->core_id = stp_id;
	sram_config->sram_word_bytes = STP_PIO_WORD_WIDTH_BYTES;
	sram_config->ram_data_mode = swap_data ? RAM_DATA_MODE_SWAP :
			RAM_DATA_MODE_NORMAL;
	sram_config->write_word = &stp_sram_write_word;
	sram_config->read_word = &stp_sram_read_word;
	sram_config->pad_to_align = pad_to_align;

	return 0;
}

static void create_vector_sram_config(struct paintbox_sram_config *sram_config,
		unsigned int stp_id)
{
	sram_config->core_id = stp_id;
	sram_config->ram_ctrl_target = 0;
	sram_config->sram_word_bytes = STP_PIO_WORD_WIDTH_BYTES;
	sram_config->ram_data_mode = RAM_DATA_MODE_COL_MAJOR;
	sram_config->write_word = &stp_sram_write_word;
	sram_config->read_word = &stp_sram_read_word;
	sram_config->pad_to_align = false;
}

static inline uint32_t get_vector_address(uint32_t lane_group_x,
		uint32_t lane_group_y, uint32_t sheet_slot, bool alu_registers)
{
	uint32_t target_base = alu_registers ? STP_RAM_TARG_ALU_IO_RF_0 :
			STP_RAM_TARG_ALU_IO_RAM_0;
	uint32_t ram_ctrl;

	/* RAM_ADDR field is composed of the bank y coordinate and the bank
	 * index
	 */
	ram_ctrl = ((lane_group_y << STP_RAM_ADDR_ROW_SHIFT) | sheet_slot) <<
			STP_RAM_CTRL_RAM_ADDR_SHIFT;
	ram_ctrl |= (lane_group_x + target_base) << STP_RAM_CTRL_RAM_TARG_SHIFT;

	return ram_ctrl;
}

static int validate_stp_vector_write_replicate_parameters(
		struct paintbox_data *pb, struct paintbox_stp *stp,
		struct sram_vector_replicate_write *req)
{

	size_t sram_len_bytes, sram_start_bytes, num_sheet_slots;

	if (!req->buf) {
		dev_err(pb->dev, "%s: invalid user buffer\n", __func__);
		return -EINVAL;
	}

	num_sheet_slots = req->write_halo_lanes ?
			pb->stp.halo_mem_size_in_words :
			pb->stp.vector_mem_size_in_words;

	sram_len_bytes = num_sheet_slots * STP_PIO_WORD_WIDTH_BYTES;
	sram_start_bytes = req->sheet_slot * STP_PIO_WORD_WIDTH_BYTES;

	if (req->len_bytes  > sram_len_bytes - sram_start_bytes) {
		dev_err(pb->dev, "%s: write length too long %lu > %lu\n",
				__func__, req->len_bytes, sram_len_bytes -
				sram_start_bytes);
		return -ERANGE;
	}

	if (req->sheet_slot > num_sheet_slots) {
		dev_err(pb->dev, "%s: sheet slot out of range %u > %lu\n",
				__func__, req->sheet_slot, num_sheet_slots);
		return -ERANGE;
	}

	if (req->byte_offset_in_lane_group > STP_PIO_WORD_WIDTH_BYTES) {
		dev_err(pb->dev,
				"%s: byte offset in lange group exceeds the size of a lange group %u > %u\n",
				__func__, req->byte_offset_in_lane_group,
				STP_PIO_WORD_WIDTH_BYTES);
		return -EINVAL;
	}

	return 0;
}

static int validate_stp_vector_write_coordinate_parameters(
		struct paintbox_data *pb, struct paintbox_stp *stp,
		struct sram_vector_coordinate_write *req)
{
	size_t sram_start_bytes, sram_len_bytes, num_sheet_slots, max_rows,
			max_cols;
	bool is_halo_write = false;

	if (!req->buf) {
		dev_err(pb->dev, "%s: invalid user buffer\n", __func__);
		return -EINVAL;
	}

	is_halo_write = (req->lane_group_x >=
			VECTOR_SRAM_LANE_GROUP_SIMD_COLS || req->lane_group_y >=
			VECTOR_SRAM_LANE_GROUP_SIMD_ROWS);

	num_sheet_slots = is_halo_write ? pb->stp.halo_mem_size_in_words :
			pb->stp.vector_mem_size_in_words;

	sram_len_bytes = num_sheet_slots * STP_PIO_WORD_WIDTH_BYTES;
	sram_start_bytes = req->sheet_slot * STP_PIO_WORD_WIDTH_BYTES;

	max_rows = VECTOR_SRAM_LANE_GROUP_SIMD_ROWS;
	max_cols = VECTOR_SRAM_LANE_GROUP_SIMD_COLS;

	if (is_halo_write) {
		max_rows += VECTOR_SRAM_LANE_GROUP_HALO_ROWS;
		max_cols += VECTOR_SRAM_LANE_GROUP_HALO_COLS;
	}

	if (req->len_bytes > sram_len_bytes - sram_start_bytes) {
		dev_err(pb->dev, "%s: write length too long %lu > %lu\n",
				__func__, req->len_bytes, sram_len_bytes -
				sram_start_bytes);
		return -ERANGE;
	}

	if (req->sheet_slot > num_sheet_slots) {
		dev_err(pb->dev, "%s: sheet slot out of range %u > %lu\n",
				__func__, req->sheet_slot, num_sheet_slots);
		return -ERANGE;
	}

	if (req->byte_offset_in_lane_group > STP_PIO_WORD_WIDTH_BYTES) {
		dev_err(pb->dev,
				"%s: byte offset in lange group exceeds the size of a lange group %u > %u\n",
				__func__, req->byte_offset_in_lane_group,
				STP_PIO_WORD_WIDTH_BYTES);
		return -EINVAL;
	}

	if (req->lane_group_x >= max_cols) {
		dev_err(pb->dev, "%s: lane group x out of range %u >= %lu\n",
				__func__, req->lane_group_x, max_cols);
		return -ERANGE;
	}

	if (req->lane_group_y >= max_rows) {
		dev_err(pb->dev, "%s: lane group y out of range %u >= %lu\n",
				__func__, req->lane_group_y, max_rows);
		return -ERANGE;
	}

	return 0;
}

static int validate_stp_vector_read_coordinate_parameters(
		struct paintbox_data *pb, struct paintbox_stp *stp,
		struct sram_vector_coordinate_read *req)
{
	size_t sram_start_bytes, sram_len_bytes, num_sheet_slots, max_rows,
			max_cols;
	bool is_halo_read = false;

	if (!req->buf) {
		dev_err(pb->dev, "%s: invalid user buffer\n", __func__);
		return -EINVAL;
	}

	is_halo_read = (req->lane_group_x >= VECTOR_SRAM_LANE_GROUP_SIMD_COLS ||
			req->lane_group_y >= VECTOR_SRAM_LANE_GROUP_SIMD_ROWS);

	num_sheet_slots = is_halo_read ? pb->stp.halo_mem_size_in_words :
			pb->stp.vector_mem_size_in_words;

	sram_len_bytes = num_sheet_slots * STP_PIO_WORD_WIDTH_BYTES;
	sram_start_bytes = req->sheet_slot * STP_PIO_WORD_WIDTH_BYTES;

	max_rows = VECTOR_SRAM_LANE_GROUP_SIMD_ROWS;
	max_cols = VECTOR_SRAM_LANE_GROUP_SIMD_COLS;

	if (is_halo_read) {
		max_rows += VECTOR_SRAM_LANE_GROUP_HALO_ROWS;
		max_cols += VECTOR_SRAM_LANE_GROUP_HALO_COLS;
	}

	if (req->len_bytes > sram_len_bytes - sram_start_bytes) {
		dev_err(pb->dev, "%s: read length too long %lu > %lu\n",
				__func__, req->len_bytes, sram_len_bytes -
				sram_start_bytes);
		return -ERANGE;
	}

	if (req->sheet_slot > num_sheet_slots) {
		dev_err(pb->dev, "%s: sheet slot out of range %u > %lu\n",
				__func__, req->sheet_slot, num_sheet_slots);
		return -ERANGE;
	}

	if (req->byte_offset_in_lane_group > STP_PIO_WORD_WIDTH_BYTES) {
		dev_err(pb->dev,
				"%s: byte offset in lange group exceeds the size of a lange group %u > %u\n",
				__func__, req->byte_offset_in_lane_group,
				STP_PIO_WORD_WIDTH_BYTES);
		return -EINVAL;
	}

	if (req->lane_group_x >= max_cols) {
		dev_err(pb->dev, "%s: lane group x out of range %u >= %lu\n",
				__func__, req->lane_group_x, max_cols);
		return -ERANGE;
	}

	if (req->lane_group_y >= max_rows) {
		dev_err(pb->dev, "%s: lane group y out of range %u >= %lu\n",
				__func__, req->lane_group_y, max_rows);
		return -ERANGE;
	}

	return 0;
}

int write_stp_scalar_sram_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_write __user *user_req;
	struct ipu_sram_write req;
	struct paintbox_sram_config sram_config;
	struct paintbox_stp *stp;
	int ret;

	user_req = (struct ipu_sram_write __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	if (!req.buf) {
		dev_err(pb->dev, "%s: invalid user buffer\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = validate_sram_transfer(pb, stp, req.sram_byte_addr,
			req.sram_target, req.len_bytes);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* HDR+ only updates stp scalar sram with this ioctl call. Use cache
	 * here to improve write to scalar sram performance by avoiding
	 * misaligned writes which triggers read-modify-write.
	 */
	if (req.sram_target == SRAM_TARGET_STP_CONSTANT_RAM) {
		if (copy_from_user(&stp->constant_ram[req.sram_byte_addr],
				req.buf, req.len_bytes)) {
			mutex_unlock(&pb->lock);
			return -EFAULT;
		}
	}

	ret = create_scalar_sram_config(&sram_config, stp->stp_id,
			req.sram_target, req.swap_data, req.pad_to_align);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev, "%s: stp%u: invalid ram target: 0x%04x err = %d\n",
				__func__, req.id, req.sram_target, ret);
		return ret;
	}

	if (req.sram_target == SRAM_TARGET_STP_CONSTANT_RAM) {
		/* calculate the closest 16-byte words that covers the entire
		 * sram write to avoid read-modify-write penalty.
		 */
		uint32_t sram_addr_lower;
		uint32_t sram_addr_upper;
		uint32_t len_bytes_aligned;

		sram_addr_lower = round_down(req.sram_byte_addr,
				sram_config.sram_word_bytes);
		sram_addr_upper = round_up(req.sram_byte_addr + req.len_bytes,
				sram_config.sram_word_bytes);
		len_bytes_aligned = sram_addr_upper - sram_addr_lower;
		ret = sram_write_buffer(pb,
				&sram_config,
				sram_addr_lower,
				&stp->constant_ram[sram_addr_lower],
				len_bytes_aligned);
		if (ret < 0) {
			dev_err(pb->dev,
					"%s: stp%u: write error addr: 0x%04x type: 0x%02x err = %d\n",
					__func__, req.id, sram_addr_lower,
					req.sram_target, ret);
		}
	} else {
		ret = sram_write_user_buffer(pb,
				&sram_config,
				req.sram_byte_addr,
				req.buf,
				req.len_bytes);
		if (ret < 0)
			dev_err(pb->dev,
					"%s: stp%u: write error addr: 0x%04x type: 0x%02x err = %d\n",
					__func__, req.id, req.sram_byte_addr,
					req.sram_target, ret);
	}
	mutex_unlock(&pb->lock);

	return ret;
}

int read_stp_scalar_sram_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_sram_read __user *user_req;
	struct ipu_sram_read req;
	struct paintbox_sram_config sram_config;
	struct paintbox_stp *stp;
	int ret;

	user_req = (struct ipu_sram_read __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	if (!req.buf) {
		dev_err(pb->dev, "%s: invalid user buffer\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = validate_sram_transfer(pb, stp, req.sram_byte_addr,
			req.sram_target, req.len_bytes);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = create_scalar_sram_config(&sram_config, stp->stp_id,
			req.sram_target, req.swap_data,
			false /* pad_to_align */);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev, "%s: stp%u: invalid ram target: 0x%04x err = %d\n",
				__func__, req.id, req.sram_target, ret);
		return ret;
	}

	ret = sram_read_user_buffer(pb, &sram_config, req.sram_byte_addr,
			req.buf, req.len_bytes);
	if (ret < 0)
		dev_err(pb->dev,
				"%s: stp%u: read error addr: 0x%04x type: 0x%02x err = %d\n",
				__func__, req.id, req.sram_byte_addr,
				req.sram_target, ret);

	mutex_unlock(&pb->lock);

	return ret;
}

static int sram_read_vector_buffer(struct paintbox_data *pb,
		unsigned int stp_id, uint8_t *buf, uint32_t lane_group_x,
		uint32_t lane_group_y, uint32_t sheet_slot_start,
		uint32_t byte_offest_in_lane_group, size_t len_bytes,
		bool read_alu_registers)
{
	struct paintbox_sram_config sram_config;
	size_t bytes_remaining = len_bytes;
	size_t bytes_read = 0;
	uint32_t sheet_slot = sheet_slot_start, ram_ctrl_addr;
	int ret;

	create_vector_sram_config(&sram_config, stp_id);

	/* If the transfer does not start at the beginning of a word then it
	 * will require special handling before we can get to the main body of
	 * the transfer.
	 */
	if (byte_offest_in_lane_group > 0) {
		uint8_t transfer_buf[STP_PIO_WORD_WIDTH_BYTES];
		size_t short_read_len = STP_PIO_WORD_WIDTH_BYTES -
				byte_offest_in_lane_group;

		ram_ctrl_addr = get_vector_address(lane_group_x, lane_group_y,
				sheet_slot, read_alu_registers);

		ret = stp_sram_read_word(pb, &sram_config, transfer_buf,
				ram_ctrl_addr);
		if (ret < 0)
			return ret;

		memcpy(buf, transfer_buf + byte_offest_in_lane_group,
				short_read_len);

		bytes_remaining -= short_read_len;
		bytes_read += short_read_len;
		sheet_slot++;
	}

	/* If the transfer data length is greater than or equal to an SRAM word
	 * then transfer the data in full word transfers.
	 */
	while (bytes_remaining && bytes_remaining >= STP_PIO_WORD_WIDTH_BYTES) {
		ram_ctrl_addr = get_vector_address(lane_group_x, lane_group_y,
				sheet_slot, read_alu_registers);

		ret = stp_sram_read_word(pb, &sram_config, buf + bytes_read,
				ram_ctrl_addr);
		if (ret < 0)
			return ret;

		bytes_remaining -= STP_PIO_WORD_WIDTH_BYTES;
		bytes_read += STP_PIO_WORD_WIDTH_BYTES;
		sheet_slot++;
	}

	/* Handle any remaining bytes that are shorter than an SRAM word */
	if (bytes_remaining > 0) {
		uint8_t transfer_buf[STP_PIO_WORD_WIDTH_BYTES];

		ram_ctrl_addr = get_vector_address(lane_group_x, lane_group_y,
				sheet_slot, read_alu_registers);

		ret = stp_sram_read_word(pb, &sram_config, transfer_buf,
				ram_ctrl_addr);
		if (ret < 0)
			return ret;

		memcpy(buf + bytes_read, transfer_buf, bytes_remaining);
	}

	return 0;
}

static int sram_write_vector_buffer(struct paintbox_data *pb,
		unsigned int stp_id, const uint8_t *buf, uint32_t lane_group_x,
		uint32_t lane_group_y, uint32_t sheet_slot_start,
		uint32_t byte_offset_in_lane_group, size_t len_bytes,
		bool write_alu_registers)
{
	struct paintbox_sram_config sram_config;
	size_t bytes_remaining = len_bytes;
	size_t bytes_written = 0;
	uint32_t sheet_slot = sheet_slot_start, ram_ctrl_addr;
	int ret;

	create_vector_sram_config(&sram_config, stp_id);

	/* If the transfer does not start at the beginning of a word then it
	 * will require special handling before we can get to the main body of
	 * the transfer.
	 */
	if (byte_offset_in_lane_group > 0) {
		uint8_t transfer_buf[STP_PIO_WORD_WIDTH_BYTES];
		size_t short_write_len = STP_PIO_WORD_WIDTH_BYTES -
				byte_offset_in_lane_group;

		ram_ctrl_addr = get_vector_address(lane_group_x, lane_group_y,
				sheet_slot, write_alu_registers);

		ret = stp_sram_read_word(pb, &sram_config, transfer_buf,
				ram_ctrl_addr);
		if (ret < 0)
			return ret;

		memcpy(transfer_buf + byte_offset_in_lane_group, buf,
				short_write_len);

		ret = stp_sram_write_word(pb, &sram_config, transfer_buf,
				ram_ctrl_addr);
		if (ret < 0)
			return ret;

		bytes_remaining -= short_write_len;
		bytes_written += short_write_len;
		sheet_slot++;
	}

	/* If the transfer data length is greater than or equal to the SRAM
	 * word then transfer the data in full word transfers.
	 */
	while (bytes_remaining && bytes_remaining >= STP_PIO_WORD_WIDTH_BYTES) {
		ram_ctrl_addr = get_vector_address(lane_group_x, lane_group_y,
				sheet_slot, write_alu_registers);

		ret = stp_sram_write_word(pb, &sram_config, buf + bytes_written,
				ram_ctrl_addr);
		if (ret < 0)
			return ret;

		bytes_remaining -= STP_PIO_WORD_WIDTH_BYTES;
		bytes_written += STP_PIO_WORD_WIDTH_BYTES;
		sheet_slot++;
	}

	/* Handle any remaining bytes that are shorter than an SRAM word. */
	if (bytes_remaining > 0) {
		uint8_t transfer_buf[STP_PIO_WORD_WIDTH_BYTES];

		ram_ctrl_addr = get_vector_address(lane_group_x, lane_group_y,
				sheet_slot, write_alu_registers);

		ret = stp_sram_read_word(pb, &sram_config, transfer_buf,
				ram_ctrl_addr);
		if (ret < 0)
			return ret;

		memcpy(transfer_buf, buf + bytes_written, bytes_remaining);

		ret = stp_sram_write_word(pb, &sram_config, transfer_buf,
				ram_ctrl_addr);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* Vector memory on the IPU is organized as a torus.  The lane group addresses
 * in the RAM access registers are physical addresses.  The following arrays map
 * the various lane groups of vector memory from their physical addresses to
 * their logical addresses.
 *
 * TODO:  Conversion between logical and physical addressing for the
 * lane groups may be moved to the runtime.  If that is the case then this code
 * can be remmoved.
 */
static const unsigned int logical_to_phys_row_map[10] = {
	7, 6, 3, 2, 0, 1, 4, 5, 8, 9
};

static const unsigned int logical_to_phys_col_map[5] = {
	1, 3, 4, 2, 0
};

int write_stp_vector_sram_coordinates_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct sram_vector_coordinate_write __user *user_req;
	struct sram_vector_coordinate_write req;
	struct paintbox_stp *stp;
	uint8_t *buf = NULL;
	int ret;

	user_req = (struct sram_vector_coordinate_write __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0)
		goto exit;

	ret = validate_stp_vector_write_coordinate_parameters(pb, stp, &req);
	if (ret < 0)
		goto exit;

	ret = alloc_and_copy_from_user(pb, &buf, req.buf, req.len_bytes);
	if (ret < 0)
		goto exit;

	ret = sram_write_vector_buffer(pb, stp->stp_id, buf,
			logical_to_phys_col_map[req.lane_group_x],
			logical_to_phys_row_map[req.lane_group_y],
			req.sheet_slot, req.byte_offset_in_lane_group,
			req.len_bytes, req.write_alu_registers);

exit:
	mutex_unlock(&pb->lock);

	kfree(buf);

	return ret;
}

int write_stp_vector_sram_replicate_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct sram_vector_replicate_write __user *user_req;
	struct sram_vector_replicate_write req;
	struct paintbox_stp *stp;
	unsigned int num_lane_group_cols, num_lane_group_rows;
	unsigned int lane_group_col, lane_group_row;
	unsigned int phys_col, phys_row;
	uint8_t *buf = NULL;
	int ret;

	user_req = (struct sram_vector_replicate_write __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0)
		goto exit;

	ret = validate_stp_vector_write_replicate_parameters(pb, stp, &req);
	if (ret < 0)
		goto exit;

	num_lane_group_rows = VECTOR_SRAM_LANE_GROUP_SIMD_ROWS;
	num_lane_group_cols = VECTOR_SRAM_LANE_GROUP_SIMD_COLS;

	if (req.write_halo_lanes) {
		num_lane_group_rows += VECTOR_SRAM_LANE_GROUP_HALO_ROWS;
		num_lane_group_cols += VECTOR_SRAM_LANE_GROUP_HALO_COLS;
	}

	ret = alloc_and_copy_from_user(pb, &buf, req.buf, req.len_bytes);
	if (ret < 0)
		goto exit;

	/* Iterate over all lane groups and write the supplied user buffer to
	 * those groups.
	 */
	for (lane_group_col = 0; lane_group_col < num_lane_group_cols;
			lane_group_col++) {
		phys_col = logical_to_phys_col_map[lane_group_col];

		for (lane_group_row = 0; lane_group_row < num_lane_group_rows;
				lane_group_row++) {
			phys_row = logical_to_phys_row_map[lane_group_row];

			ret = sram_write_vector_buffer(pb, stp->stp_id, buf,
					phys_col, phys_row, req.sheet_slot,
					req.byte_offset_in_lane_group,
					req.len_bytes, req.write_alu_registers);
			if (ret < 0)
				goto exit;
		}
	}

exit:
	mutex_unlock(&pb->lock);

	kfree(buf);

	return ret;
}

int read_stp_vector_sram_coordinates_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct sram_vector_coordinate_read __user *user_req;
	struct sram_vector_coordinate_read req;
	struct paintbox_stp *stp;
	uint8_t *buf = NULL;
	int ret;

	user_req = (struct sram_vector_coordinate_read __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.id, &ret);
	if (ret < 0)
		goto exit;

	ret = validate_stp_vector_read_coordinate_parameters(pb, stp, &req);
	if (ret < 0)
		goto exit;

	buf = kmalloc(req.len_bytes, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = sram_read_vector_buffer(pb, stp->stp_id, buf,
			logical_to_phys_col_map[req.lane_group_x],
			logical_to_phys_row_map[req.lane_group_y],
			req.sheet_slot, req.byte_offset_in_lane_group,
			req.len_bytes, req.read_alu_registers);
	if (ret < 0)
		goto exit;

	if (copy_to_user((void __user *)req.buf, buf, req.len_bytes))
		ret = -EFAULT;

exit:
	mutex_unlock(&pb->lock);

	kfree(buf);

	return ret;
}
