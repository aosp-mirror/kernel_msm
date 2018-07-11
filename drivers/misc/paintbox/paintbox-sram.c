/*
 * Paintbox programmable IPU SRAM Support
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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-bus.h"
#include "paintbox-common.h"
#include "paintbox-regs.h"
#include "paintbox-sram.h"

#ifdef DEBUG
void dump_registers(struct paintbox_data *pb, void __iomem *reg_start,
		size_t reg_count, const char *msg)
{
	unsigned int i;

	dev_info(pb->dev, "%s: reg addr %p len %lu\n", msg, reg_start,
			reg_count);
	for (i = 0; i < reg_count; i++)
		dev_info(pb->dev, "0x%016llx\n", readq(reg_start + i *
				IPU_REG_WIDTH_BYTES));
}

#define DUMP_REGISTERS(pb, reg, reg_count, msg)	\
	dump_registers(pb, reg, reg_count, msg)
#else
#define DUMP_REGISTERS(pb, reg, reg_count, msg)	\
do { } while (0)
#endif


int alloc_and_copy_from_user(struct paintbox_data *pb, uint8_t **buf,
		const void __user *user_buf, size_t len_bytes)
{
	*buf = kmalloc(len_bytes, GFP_KERNEL);
	if (!*buf) {
		dev_err(pb->dev, "%s: allocation failure\n", __func__);
		return -ENOMEM;
	}

	if (copy_from_user(*buf, user_buf, len_bytes)) {
		kfree(*buf);
		return -EFAULT;
	}

	return 0;
}

void write_ram_data_registers(struct paintbox_data *pb, const uint8_t *buf,
		uint32_t data_reg, unsigned int reg_count)
{
	unsigned int i;

	for (i = 0; i < reg_count * IPU_REG_WIDTH_BYTES; i +=
			IPU_REG_WIDTH_BYTES) {
		uint64_t data;

		data = ((uint64_t)buf[i + 0]) << 0;
		data |= ((uint64_t)buf[i + 1]) << 8;
		data |= ((uint64_t)buf[i + 2]) << 16;
		data |= ((uint64_t)buf[i + 3]) << 24;
		data |= ((uint64_t)buf[i + 4]) << 32;
		data |= ((uint64_t)buf[i + 5]) << 40;
		data |= ((uint64_t)buf[i + 6]) << 48;
		data |= ((uint64_t)buf[i + 7]) << 56;

		paintbox_writeq(pb->dev, data, data_reg + i);
	}

	DUMP_REGISTERS(pb, data_reg, reg_count, __func__);
}

/* TODO:  Remove once b/30316979 is fixed.  RAM_DATA_MODE_SWAP is
 * needed because the assembler writes the instruction in reverse byte order
 * (due to an issue with the DV tools).  Once that bug is fixed this function
 * can be removed.
 */
void write_ram_data_registers_swapped(struct paintbox_data *pb,
		const uint8_t *buf, uint32_t data_reg,
		unsigned int reg_count)
{
	unsigned int i, j;

	for (i = 0, j = reg_count - 1; i < reg_count * IPU_REG_WIDTH_BYTES;
			i += IPU_REG_WIDTH_BYTES, j--) {
		uint64_t data;

		/* Swap the byte order when loading the registers */
		data = ((uint64_t)buf[i + 0]) << 56;
		data |= ((uint64_t)buf[i + 1]) << 48;
		data |= ((uint64_t)buf[i + 2]) << 40;
		data |= ((uint64_t)buf[i + 3]) << 32;
		data |= ((uint64_t)buf[i + 4]) << 24;
		data |= ((uint64_t)buf[i + 5]) << 16;
		data |= ((uint64_t)buf[i + 6]) << 8;
		data |= ((uint64_t)buf[i + 7]) << 0;

		paintbox_writeq(pb->dev, data, data_reg + j *
				IPU_REG_WIDTH_BYTES);
	}

	DUMP_REGISTERS(pb, data_reg, reg_count, __func__);
}

/* TODO:  The conversion to the vector SRAM's column major lane
 * ordering needs to be moved to the runtime so it can be used for both DMA and
 * PIO.  This function can be removed once that support is ready.
 */
void write_ram_data_registers_column_major(struct paintbox_data *pb,
		const uint8_t *buf)
{
	unsigned int reg_index, row0_col, row1_col;

	/* Each vector bank is a 4 x 2 array of ALU lanes (lanes are 16 bits).
	 * The data provided to the HAL to be written into the vector bank is a
	 * uint8_t* buffer with the uint16_t lane values in row major order:
	 *
	 * 0                   8                15
	 * R0C0 ROC1 R0C2 R0C3 R1C0 R1C1 R1C2 R1C3
	 *
	 * The hardware on the other hand is column major (wiring optimization)
	 * and requires the data to be loaded into the data registers in this
	 * fashion:
	 *
	 * DATA_1_H   DATA_1_L   DATA_0_H   DATA_0_L
	 * R1C3 R0C3  R1C2 R0C2  R1C1 R0C1  R1C0 R0C0
	 */
	row0_col = 0;
	row1_col = VECTOR_LANE_GROUP_WIDTH * VECTOR_LANE_WIDTH;

	for (reg_index = 0; reg_index < STP_DATA_REG_COUNT; reg_index++,
			row0_col += VECTOR_GROUP_ROW_OFFSET_BYTES,
			row1_col += VECTOR_GROUP_ROW_OFFSET_BYTES) {
		uint64_t data;

		data = ((uint64_t)buf[row0_col]) << 0;
		data |= ((uint64_t)buf[row0_col + 1]) << 8;
		data |= ((uint64_t)buf[row1_col]) << 16;
		data |= ((uint64_t)buf[row1_col + 1]) << 24;
		data |= ((uint64_t)buf[row0_col + 2]) << 32;
		data |= ((uint64_t)buf[row0_col + 3]) << 40;
		data |= ((uint64_t)buf[row1_col + 2]) << 48;
		data |= ((uint64_t)buf[row1_col + 3]) << 56;

		paintbox_writeq(pb->dev, data, IPU_CSR_STP_OFFSET +
				STP_RAM_DATA0 + reg_index *
				IPU_REG_WIDTH_BYTES);
	}

	DUMP_REGISTERS(pb, pb->stp.reg_base + STP_RAM_DATA0, STP_DATA_REG_COUNT,
			__func__);
}

void read_ram_data_registers(struct paintbox_data *pb, uint8_t *buf,
		uint32_t data_reg, unsigned int reg_count)
{
	unsigned int i;

	DUMP_REGISTERS(pb, data_reg, reg_count, __func__);

	for (i = 0; i < reg_count * IPU_REG_WIDTH_BYTES; i +=
			IPU_REG_WIDTH_BYTES) {
		uint64_t data = paintbox_readq(pb->dev, data_reg + i);

		buf[i + 0] = (uint8_t)((data >> 0) & 0xff);
		buf[i + 1] = (uint8_t)((data >> 8) & 0xff);
		buf[i + 2] = (uint8_t)((data >> 16) & 0xff);
		buf[i + 3] = (uint8_t)((data >> 24) & 0xff);
		buf[i + 4] = (uint8_t)((data >> 32) & 0xff);
		buf[i + 5] = (uint8_t)((data >> 40) & 0xff);
		buf[i + 6] = (uint8_t)((data >> 48) & 0xff);
		buf[i + 7] = (uint8_t)((data >> 56) & 0xff);
	}
}

/* TODO:  Remove once b/30316979 is fixed.  RAM_DATA_MODE_SWAP is
 * needed because the assember writes the instruction in reverse byte order (due
 * to an issue with the DV tools).  Once that bug is fixed this function can be
 * removed.
 */
void read_ram_data_registers_swapped(struct paintbox_data *pb,
		uint8_t *buf, uint32_t data_reg, unsigned int reg_count)
{
	unsigned int i, j;

	DUMP_REGISTERS(pb, data_reg, reg_count, __func__);

	for (i = 0, j = reg_count - 1; i < reg_count * IPU_REG_WIDTH_BYTES;
			i += IPU_REG_WIDTH_BYTES, j--) {
		uint64_t data = paintbox_readq(pb->dev, data_reg + j *
				IPU_REG_WIDTH_BYTES);

		/* Swap the byte order when copying from the registers */
		buf[i + 0] = (uint8_t)((data >> 56) & 0xff);
		buf[i + 1] = (uint8_t)((data >> 48) & 0xff);
		buf[i + 2] = (uint8_t)((data >> 40) & 0xff);
		buf[i + 3] = (uint8_t)((data >> 32) & 0xff);
		buf[i + 4] = (uint8_t)((data >> 24) & 0xff);
		buf[i + 5] = (uint8_t)((data >> 16) & 0xff);
		buf[i + 6] = (uint8_t)((data >> 8) & 0xff);
		buf[i + 7] = (uint8_t)((data >> 0) & 0xff);
	}
}

/* TODO:  The conversion to the vector SRAM's column major lane
 * ordering needs to be moved to the runtime so it can be used for both DMA and
 * PIO.  This function can be removed once that support is ready.
 */
void read_ram_data_registers_column_major(struct paintbox_data *pb,
		uint8_t *buf)
{
	unsigned int reg_index, row0_col, row1_col;

	DUMP_REGISTERS(pb, pb->stp.reg_base + STP_RAM_DATA0, STP_DATA_REG_COUNT,
			__func__);

	/* Each vector bank is a 4 x 2 array of ALU lanes (lanes are 16 bits).
	 * The data provided to the HAL to be written into the vector bank is a
	 * uint8_t* buffer with the uint16_t lane values in row major order:
	 *
	 * 0                   8                15
	 * R0C0 ROC1 R0C2 R0C3 R1C0 R1C1 R1C2 R1C3
	 *
	 * The hardware on the other hand is column major (wiring optimization)
	 * and requires the data to be loaded into the data registers in this
	 * fashion:
	 *
	 * DATA_1_H   DATA_1_L   DATA_0_H   DATA_0_L
	 * R1C3 R0C3  R1C2 R0C2  R1C1 R0C1  R1C0 R0C0
	 */
	row0_col = 0;
	row1_col = VECTOR_LANE_GROUP_WIDTH * VECTOR_LANE_WIDTH;

	for (reg_index = 0; reg_index < STP_DATA_REG_COUNT; reg_index++,
			row0_col += VECTOR_GROUP_ROW_OFFSET_BYTES,
			row1_col += VECTOR_GROUP_ROW_OFFSET_BYTES) {
		uint64_t data = paintbox_readq(pb->dev, IPU_CSR_STP_OFFSET +
				STP_RAM_DATA0 + reg_index *
				IPU_REG_WIDTH_BYTES);

		buf[row0_col] = (uint8_t)((data >> 0) & 0xff);
		buf[row0_col + 1] = (uint8_t)((data >> 8) & 0xff);
		buf[row1_col] = (uint8_t)((data >> 16) & 0xff);
		buf[row1_col + 1] = (uint8_t)((data >> 24) & 0xff);
		buf[row0_col + 2] = (uint8_t)((data >> 32) & 0xff);
		buf[row0_col + 3] = (uint8_t)((data >> 40) & 0xff);
		buf[row1_col + 2] = (uint8_t)((data >> 48) & 0xff);
		buf[row1_col + 3] = (uint8_t)((data >> 56) & 0xff);
	}
}

static inline uint32_t get_scalar_address(uint32_t sram_word_addr)
{
	return (sram_word_addr & COMMON_RAM_ADDR) << COMMON_RAM_ADDR_SHIFT;
}

int sram_write_buffer(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config,
		uint32_t sram_byte_addr, const uint8_t *buf, size_t len_bytes)
{
	size_t bytes_remaining = len_bytes;
	size_t bytes_written = 0;
	unsigned int byte_offset_in_word;
	uint32_t sram_word_addr, ram_ctrl_addr;
	int ret;

	sram_word_addr = sram_byte_addr / sram_config->sram_word_bytes;
	byte_offset_in_word = sram_byte_addr % sram_config->sram_word_bytes;

	/* If the transfer does not start at the beginning of a word then it
	 * will require special handling before we can get to the main body of
	 * the transfer.
	 */
	if (byte_offset_in_word > 0) {
		uint8_t transfer_buf[sram_config->sram_word_bytes];
		size_t short_write_len = min(sram_config->sram_word_bytes -
				byte_offset_in_word, len_bytes);

		ram_ctrl_addr = get_scalar_address(sram_word_addr);

		if (sram_config->pad_to_align) {
			memset(transfer_buf, 0, sram_config->sram_word_bytes);
		} else {
			ret = sram_config->read_word(pb, sram_config,
					transfer_buf, ram_ctrl_addr);
			if (ret < 0)
				return ret;
		}

		memcpy(transfer_buf + byte_offset_in_word, buf,
				short_write_len);

		ret = sram_config->write_word(pb, sram_config, transfer_buf,
				ram_ctrl_addr);
		if (ret < 0)
			return ret;

		bytes_remaining -= short_write_len;
		bytes_written += short_write_len;
		sram_word_addr++;
	}

	/* If the transfer data length is greater than or equal to the SRAM
	 * word then transfer the data in full word transfers.
	 */
	while (bytes_remaining && bytes_remaining >=
			sram_config->sram_word_bytes) {
		ram_ctrl_addr = get_scalar_address(sram_word_addr);

		ret = sram_config->write_word(pb, sram_config, buf +
				bytes_written, ram_ctrl_addr);
		if (ret < 0)
			return ret;

		bytes_remaining -= sram_config->sram_word_bytes;
		bytes_written += sram_config->sram_word_bytes;
		sram_word_addr++;
	}

	/* Handle any remaining bytes that are shorter than an SRAM word. */
	if (bytes_remaining > 0) {
		uint8_t transfer_buf[sram_config->sram_word_bytes];

		ram_ctrl_addr = get_scalar_address(sram_word_addr);

		if (sram_config->pad_to_align) {
			memset(transfer_buf, 0, sram_config->sram_word_bytes);
		} else {
			ret = sram_config->read_word(pb, sram_config,
					transfer_buf, ram_ctrl_addr);
			if (ret < 0)
				return ret;
		}

		memcpy(transfer_buf, buf + bytes_written, bytes_remaining);

		ret = sram_config->write_word(pb, sram_config, transfer_buf,
				ram_ctrl_addr);
		if (ret < 0)
			return ret;
	}

	return 0;
}

int sram_write_user_buffer(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config,
		uint32_t sram_byte_addr, const void __user *user_buf,
		size_t len_bytes)
{
	uint8_t *buf;
	int ret;

	ret = alloc_and_copy_from_user(pb, &buf, user_buf, len_bytes);
	if (ret < 0)
		return ret;

	ret = sram_write_buffer(pb, sram_config, sram_byte_addr, buf,
			len_bytes);

	kfree(buf);

	return ret;
}

int sram_read_buffer(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config,
		uint32_t sram_byte_addr, uint8_t *buf, size_t len_bytes)
{
	size_t bytes_remaining = len_bytes;
	size_t bytes_read = 0;
	unsigned int byte_offset_in_word;
	uint32_t sram_word_addr;
	int ret = 0;

	sram_word_addr = sram_byte_addr / sram_config->sram_word_bytes;
	byte_offset_in_word = sram_byte_addr % sram_config->sram_word_bytes;

	/* If the transfer does not start at the beginning of a word then it
	 * will require special handling before we can get to the main body of
	 * the transfer.
	 */
	if (byte_offset_in_word > 0) {
		uint8_t transfer_buf[sram_config->sram_word_bytes];

		size_t short_read_len = min(sram_config->sram_word_bytes -
				byte_offset_in_word, len_bytes);

		ret = sram_config->read_word(pb, sram_config, transfer_buf,
				get_scalar_address(sram_word_addr));
		if (ret < 0)
			return ret;

		memcpy(buf, transfer_buf + byte_offset_in_word, short_read_len);

		bytes_remaining -= short_read_len;
		bytes_read += short_read_len;
		sram_word_addr++;
	}

	/* If the transfer data length is greater than or equal to an SRAM word
	 * then transfer the data in full word transfers.
	 */
	while (bytes_remaining && bytes_remaining >=
			sram_config->sram_word_bytes) {
		ret = sram_config->read_word(pb, sram_config, buf + bytes_read,
				get_scalar_address(sram_word_addr));
		if (ret < 0)
			return ret;

		bytes_remaining -= sram_config->sram_word_bytes;
		bytes_read += sram_config->sram_word_bytes;
		sram_word_addr++;
	}

	/* Handle any remaining bytes that are shorter than an SRAM word */
	if (bytes_remaining > 0) {
		uint8_t transfer_buf[sram_config->sram_word_bytes];

		ret = sram_config->read_word(pb, sram_config, transfer_buf,
				get_scalar_address(sram_word_addr));
		if (ret < 0)
			return ret;

		memcpy(buf + bytes_read, transfer_buf, bytes_remaining);
	}

	return 0;
}

int sram_read_user_buffer(struct paintbox_data *pb,
		struct paintbox_sram_config *sram_config,
		uint32_t sram_byte_addr, void __user *user_buf,
		size_t len_bytes)
{
	uint8_t *buf;
	int ret;

	buf = kmalloc(len_bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = sram_read_buffer(pb, sram_config, sram_byte_addr, buf, len_bytes);
	if (ret < 0)
		goto exit;

	if (copy_to_user((void __user *)user_buf, buf, len_bytes))
		ret = -EFAULT;

exit:
	kfree(buf);

	return ret;
}
