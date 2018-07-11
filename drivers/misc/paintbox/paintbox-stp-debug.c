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

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

static uint64_t paintbox_stp_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;
	uint64_t val;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, stp->stp_id);
	val = paintbox_readq(pb->dev, IPU_CSR_STP_OFFSET +
			reg_entry->reg_offset);

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	mutex_unlock(&pb->lock);

	return val;
}

static void paintbox_stp_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, stp->stp_id);
	paintbox_writeq(pb->dev, val, IPU_CSR_STP_OFFSET +
			reg_entry->reg_offset);

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	mutex_unlock(&pb->lock);
}

static const char *stp_reg_names[STP_NUM_REGS] = {
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
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	REG_NAME_ENTRY(STP_START),
	REG_NAME_ENTRY(STP_MASK),
	REG_NAME_ENTRY(STP_IER),
	REG_NAME_ENTRY(STP_IMR),
	REG_NAME_ENTRY(STP_ISR),
	REG_NAME_ENTRY(STP_ISR_OVF),
	REG_NAME_ENTRY(STP_ITR),
	REG_NAME_ENTRY(STP_IRQ_LOG),
	REG_NAME_ENTRY(STP_ERR_LOG),
#endif
};

static inline int paintbox_dump_stp_reg(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len)
{
	const char *reg_name = stp_reg_names[REG_INDEX(reg_offset)];

	return dump_ipu_register_with_value(pb, IPU_CSR_STP_OFFSET,
			reg_offset, reg_value, reg_name, buf, written, len);
}

static int paintbox_dump_stp_reg_verbose(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = paintbox_dump_stp_reg(pb, reg_offset, reg_value, buf, written,
			len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
static int paintbox_dump_stp_ctrl(struct paintbox_data *pb, uint64_t val,
		char *buf, int *written, size_t len)
{
	return paintbox_dump_stp_reg_verbose(pb, STP_CTRL, val, buf, written,
			len,
			"\tTS_FR %d PC_FR %d RESUME %d RESET %d ENA %d\n",
			!!(val & STP_CTRL_TS_FR_MASK),
			!!(val & STP_CTRL_PC_FR_MASK),
			!!(val & STP_CTRL_RESUME_MASK),
			!!(val & STP_CTRL_RESET_MASK),
			!!(val & STP_CTRL_ENA_MASK));
}

static int paintbox_dump_stp_stat(struct paintbox_data *pb, uint64_t val,
		char *buf, int *written, size_t len)
{
	return paintbox_dump_stp_reg_verbose(pb, STP_STAT, val, buf, written,
			len,
			"\tSTALLED %d PC 0x%04x\n",
			(val >> STP_STAT_STALLED_SHIFT) & STP_STAT_STALLED_M,
			val & STP_STAT_PC_MASK);
}
#else
static int paintbox_dump_stp_ctrl(struct paintbox_data *pb, uint64_t val,
		char *buf, int *written, size_t len)
{
	return paintbox_dump_stp_reg_verbose(pb, STP_CTRL, val, buf, written,
			len,
			"\tLBP_MASK 0x%04x INT %d RESUME %d RESET %d ENA %d\n",
			(val & STP_CTRL_LBP_MASK_MASK) >>
			STP_CTRL_LBP_MASK_SHIFT, !!(val & STP_CTRL_INT_MASK),
			!!(val & STP_CTRL_RESUME_MASK),
			!!(val & STP_CTRL_RESET_MASK),
			!!(val & STP_CTRL_ENA_MASK));
}

static int paintbox_dump_stp_stat(struct paintbox_data *pb, uint64_t val,
		char *buf, int *written, size_t len)
{
	return paintbox_dump_stp_reg_verbose(pb, STP_STAT, val, buf, written,
			len,
			"\tSTALLED %d INT_CODE 0x%04x PC 0x%04x\n",
			(val >> STP_STAT_STALLED_SHIFT) & STP_STAT_STALLED_M,
			(val & STP_STAT_INT_CODE_MASK) >>
			STP_STAT_INT_CODE_SHIFT, val & STP_STAT_PC_MASK);
}
#endif

int paintbox_dump_stp_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	uint64_t stp_registers[STP_NUM_REGS];
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;
	int ret, written = 0;
	unsigned int reg_offset;
	uint64_t val;

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, stp->stp_id);

	for (reg_offset = 0; reg_offset < STP_BLOCK_LEN; reg_offset +=
			IPU_REG_WIDTH) {
		if (!stp_reg_names[REG_INDEX(reg_offset)])
			continue;

		stp_registers[REG_INDEX(reg_offset)] =
				paintbox_readq(pb->dev, IPU_CSR_STP_OFFSET +
				reg_offset);
	}

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	val = stp_registers[REG_INDEX(STP_SEL)];
	ret = paintbox_dump_stp_reg_verbose(pb, STP_SEL, val, buf, &written,
			len, "\tSTP_SEL 0x%02x\n", val & STP_SEL_STP_SEL_MASK);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_CTRL)];
	ret = paintbox_dump_stp_ctrl(pb, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_STAT)];
	ret = paintbox_dump_stp_stat(pb, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_CAP)];
	ret = paintbox_dump_stp_reg_verbose(pb, STP_CAP, val, buf, &written,
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
	ret = paintbox_dump_stp_reg(pb, STP_START, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_MASK)];
	ret = paintbox_dump_stp_reg(pb, STP_MASK, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_IER)];
	ret = paintbox_dump_stp_reg(pb, STP_IER, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_IMR)];
	ret = paintbox_dump_stp_reg(pb, STP_IMR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_ISR)];
	ret = paintbox_dump_stp_reg(pb, STP_ISR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_ISR_OVF)];
	ret = paintbox_dump_stp_reg(pb, STP_ISR_OVF, val, buf, &written, len);
	if (ret < 0)
		return ret;

	val = stp_registers[REG_INDEX(STP_ITR)];
	ret = paintbox_dump_stp_reg(pb, STP_ITR, val, buf, &written, len);
	if (ret < 0)
		return ret;

	for (reg_offset = STP_RAM_CTRL; reg_offset < STP_BLOCK_LEN;
			reg_offset += IPU_REG_WIDTH) {
		if (!stp_reg_names[REG_INDEX(reg_offset)])
			continue;

		ret = paintbox_dump_stp_reg(pb, reg_offset,
				stp_registers[REG_INDEX(reg_offset)], buf,
				&written, len);
		if (ret < 0)
			return ret;
	}

	return written;
}

int paintbox_dump_stp_stats(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_stp *stp = container_of(debug, struct paintbox_stp,
			debug);

	return scnprintf(buf, len, " interrupts: %u\n", stp->interrupt_count);
}

void paintbox_stp_debug_init(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	paintbox_debug_create_entry(pb, &stp->debug, pb->debug_root,
			"stp", stp->stp_id, paintbox_dump_stp_registers,
			paintbox_dump_stp_stats, stp);

	paintbox_debug_create_reg_entries(pb, &stp->debug, stp_reg_names,
			STP_NUM_REGS, paintbox_stp_reg_entry_write,
			paintbox_stp_reg_entry_read);
}

void paintbox_stp_debug_remove(struct paintbox_data *pb,
		struct paintbox_stp *stp)
{
	paintbox_debug_free_reg_entries(&stp->debug);
	paintbox_debug_free_entry(&stp->debug);
}
