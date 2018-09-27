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

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include "ipu-apb-debug.h"
#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-regs.h"

/* The caller to this function must hold pb->lock */
static uint64_t ipu_apb_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	return ipu_readq(pb->dev, IPU_CSR_APB_OFFSET + reg_entry->reg_offset);
}

/* The caller to this function must hold pb->lock */
static void ipu_apb_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	ipu_writeq(pb->dev, val, IPU_CSR_APB_OFFSET + reg_entry->reg_offset);
}

static const char *ipu_apb_reg_names[IO_APB_NUM_REGS] = {
	REG_NAME_ENTRY(IPU_ISR),
	REG_NAME_ENTRY(IPU_ITR),
	REG_NAME_ENTRY(IPU_IER),
	REG_NAME_ENTRY(DMA_CHAN_ISR),
	REG_NAME_ENTRY(DMA_CHAN_ITR),
	REG_NAME_ENTRY(DMA_CHAN_IER),
	REG_NAME_ENTRY(DMA_CHAN_IMR),
	REG_NAME_ENTRY(DMA_ERR_ISR),
	REG_NAME_ENTRY(DMA_ERR_IMR),
	REG_NAME_ENTRY(DMA_ERR_IER),
	REG_NAME_ENTRY(DMA_ERR_ITR),
	REG_NAME_ENTRY(IPU_STP_ISR),
	REG_NAME_ENTRY(IPU_STP_ITR),
	REG_NAME_ENTRY(IPU_STP_IER),
	REG_NAME_ENTRY(IPU_STP_IMR),
	REG_NAME_ENTRY(STP_ERR_ISR),
	REG_NAME_ENTRY(STP_ERR_IMR),
	REG_NAME_ENTRY(STP_ERR_IER),
	REG_NAME_ENTRY(STP_ERR_ITR),
	REG_NAME_ENTRY(IPU_STP_GRP_SEL),
};

static inline int ipu_apb_dump_reg(struct paintbox_data *pb,
		uint32_t reg_offset, char *buf, int *written, size_t len)
{
	const char *reg_name = ipu_apb_reg_names[REG_INDEX(reg_offset)];

	return ipu_debug_dump_register(pb, IPU_CSR_APB_OFFSET, reg_offset,
			reg_name, buf, written, len);
}

/* The caller to this function must hold pb->lock */
int ipu_apb_dump_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_data *pb = debug->pb;
	unsigned int i;
	int ret, written = 0;

	for (i = 0; i < IO_APB_NUM_REGS; i++) {
		if (ipu_apb_reg_names[i] == NULL)
			continue;

		ret = ipu_apb_dump_reg(pb, i * IPU_REG_WIDTH, buf, &written,
				len);
		if (ret < 0) {
			dev_err(pb->dev, "%s: register dump error, err = %d",
					__func__, ret);
			return ret;
		}
	}

	return written;
}

void ipu_apb_debug_init(struct paintbox_data *pb)
{
	ipu_debug_create_entry(pb, &pb->apb_debug, pb->debug_root, "apb", -1,
			ipu_apb_dump_registers, NULL, pb);

	ipu_debug_create_reg_entries(pb, &pb->apb_debug, ipu_apb_reg_names,
			IO_APB_NUM_REGS, ipu_apb_reg_entry_write,
			ipu_apb_reg_entry_read);
}

void ipu_apb_debug_remove(struct paintbox_data *pb)
{
	ipu_debug_free_reg_entries(&pb->apb_debug);
	ipu_debug_free_entry(&pb->apb_debug);
}
