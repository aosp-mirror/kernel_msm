/*
 * STP debug support for the Paintbox programmable IPU
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

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-regs.h"
#include "ipu-stp-debug.h"

static inline void ipu_stp_select(struct paintbox_data *pb,
		unsigned int stp_id)
{
	ipu_writel(pb->dev, stp_id, IPU_CSR_STP_OFFSET + STP_SEL);
}

/* The caller to this function must hold pb->lock */
static uint64_t ipu_stp_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;

	/* TODO(b/115386014):  Move to a JQS message based system. */

	ipu_stp_select(pb, stp->stp_id);
	return ipu_readq(pb->dev, IPU_CSR_STP_OFFSET + reg_entry->reg_offset);
}

/* The caller to this function must hold pb->lock */
static void ipu_stp_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;

	/* TODO(b/115386014):  Move to a JQS message based system. */

	ipu_stp_select(pb, stp->stp_id);
	ipu_writeq(pb->dev, val, IPU_CSR_STP_OFFSET + reg_entry->reg_offset);
}

static const char *ipu_stp_reg_names[STP_NUM_REGS] = {
	REG_NAME_ENTRY(STP_SEL),
	REG_NAME_ENTRY(STP_CTRL),
	REG_NAME_ENTRY(STP_STAT),
	REG_NAME_ENTRY(STP_CAP),
	REG_NAME_ENTRY(STP_RAM_CTRL),
	REG_NAME_ENTRY(STP_RAM_DATA0),
	REG_NAME_ENTRY(STP_RAM_DATA1),
	REG_NAME_ENTRY(STP_PMON_CFG),
	REG_NAME_ENTRY(STP_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(STP_PMON_CNT_0),
	REG_NAME_ENTRY(STP_PMON_CNT_0_STS),
	REG_NAME_ENTRY(STP_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(STP_PMON_CNT_1),
	REG_NAME_ENTRY(STP_PMON_CNT_1_STS),
	REG_NAME_ENTRY(STP_START),
	REG_NAME_ENTRY(STP_MASK),
	REG_NAME_ENTRY(STP_IER),
	REG_NAME_ENTRY(STP_IMR),
	REG_NAME_ENTRY(STP_ISR),
	REG_NAME_ENTRY(STP_ISR_OVF),
	REG_NAME_ENTRY(STP_ITR),
	REG_NAME_ENTRY(STP_IRQ_LOG),
	REG_NAME_ENTRY(STP_ERR_LOG),
};

static inline int ipu_stp_dump_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len)
{
	const char *reg_name = ipu_stp_reg_names[REG_INDEX(reg_offset)];

	return ipu_debug_dump_register_with_value(pb, IPU_CSR_STP_OFFSET,
			reg_offset, reg_value, reg_name, buf, written, len);
}

static int ipu_stp_dump_reg_verbose(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = ipu_stp_dump_register(pb, reg_offset, reg_value, buf, written,
			len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = ipu_debug_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

static int ipu_stp_dump_stp_ctrl_register(struct paintbox_data *pb,
		uint64_t val, char *buf, int *written, size_t len)
{
	return ipu_stp_dump_reg_verbose(pb, STP_CTRL, val, buf, written,
			len,
			"\tTS_FR %d PC_FR %d RESUME %d RESET %d ENA %d\n",
			!!(val & STP_CTRL_TS_FR_MASK),
			!!(val & STP_CTRL_PC_FR_MASK),
			!!(val & STP_CTRL_RESUME_MASK),
			!!(val & STP_CTRL_RESET_MASK),
			!!(val & STP_CTRL_ENA_MASK));
}

static int ipu_stp_dump_stp_stat_register(struct paintbox_data *pb,
		uint64_t val, char *buf, int *written, size_t len)
{
	return ipu_stp_dump_reg_verbose(pb, STP_STAT, val, buf, written,
			len,
			"\tSTALLED %d PC 0x%04x\n",
			(val >> STP_STAT_STALLED_SHIFT) & STP_STAT_STALLED_M,
			val & STP_STAT_PC_MASK);
}

/* The caller to this function must hold pb->lock */
int ipu_stp_dump_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	uint64_t stp_registers[STP_NUM_REGS];
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;
	int ret, written = 0;
	unsigned int reg_offset;
	uint64_t val;

	ipu_stp_select(pb, stp->stp_id);

	for (reg_offset = 0; reg_offset < STP_BLOCK_LEN; reg_offset +=
			IPU_REG_WIDTH) {
		if (!ipu_stp_reg_names[REG_INDEX(reg_offset)])
			continue;

		stp_registers[REG_INDEX(reg_offset)] =
				ipu_readq(pb->dev, IPU_CSR_STP_OFFSET +
				reg_offset);
	}

	val = stp_registers[REG_INDEX(STP_SEL)];
	ret = ipu_stp_dump_reg_verbose(pb, STP_SEL, val, buf, &written,
			len, "\tSTP_SEL 0x%02x\n", val & STP_SEL_STP_SEL_MASK);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_CTRL)];
	ret = ipu_stp_dump_stp_ctrl_register(pb, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_STAT)];
	ret = ipu_stp_dump_stp_stat_register(pb, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_CAP)];
	ret = ipu_stp_dump_reg_verbose(pb, STP_CAP, val, buf, &written,
			len,
			"\tHALO_MEM %u words VECTOR_MEM %u words CONST MEM %u words SCALAR_MEM %u words INST_MEM %u instructions\n",
			(val & STP_CAP_HALO_MEM_MASK) >>
					STP_CAP_HALO_MEM_SHIFT,
			(val & STP_CAP_VECTOR_MEM_MASK) >>
					STP_CAP_VECTOR_MEM_SHIFT,
			(val & STP_CAP_CONST_MEM_MASK) >>
					STP_CAP_CONST_MEM_SHIFT,
			(val & STP_CAP_SCALAR_MEM_MASK) >>
					STP_CAP_SCALAR_MEM_SHIFT,
			val & STP_CAP_INST_MEM_MASK);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_START)];
	ret = ipu_stp_dump_register(pb, STP_START, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_MASK)];
	ret = ipu_stp_dump_register(pb, STP_MASK, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_IER)];
	ret = ipu_stp_dump_register(pb, STP_IER, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_IMR)];
	ret = ipu_stp_dump_register(pb, STP_IMR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_ISR)];
	ret = ipu_stp_dump_register(pb, STP_ISR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_ISR_OVF)];
	ret = ipu_stp_dump_register(pb, STP_ISR_OVF, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_ITR)];
	ret = ipu_stp_dump_register(pb, STP_ITR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	for (reg_offset = STP_RAM_CTRL; reg_offset < STP_BLOCK_LEN;
			reg_offset += IPU_REG_WIDTH) {
		if (!ipu_stp_reg_names[REG_INDEX(reg_offset)])
			continue;

		ret = ipu_stp_dump_register(pb, reg_offset,
				stp_registers[REG_INDEX(reg_offset)], buf,
				&written, len);
		if (ret < 0)
			return ret;
	}

	return written;
}

void ipu_stp_debug_init(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	ipu_debug_create_entry(pb, &stp->debug, pb->debug_root, "stp",
			stp->stp_id, ipu_stp_dump_registers, NULL, stp);

	ipu_debug_create_reg_entries(pb, &stp->debug, ipu_stp_reg_names,
			STP_NUM_REGS, ipu_stp_reg_entry_write,
			ipu_stp_reg_entry_read);
}

void ipu_stp_debug_remove(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	ipu_debug_free_reg_entries(&stp->debug);
	ipu_debug_free_entry(&stp->debug);
}
