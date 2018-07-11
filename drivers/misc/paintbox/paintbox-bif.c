/*
 * BIF support for the Paintbox programmable IPU
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
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include "paintbox-bif.h"
#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-regs.h"

#ifdef CONFIG_PAINTBOX_DEBUG
static uint64_t paintbox_bif_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	return paintbox_readq(pb->dev, IPU_CSR_AXI_OFFSET +
			reg_entry->reg_offset);
}

static void paintbox_bif_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	paintbox_writeq(pb->dev, val, IPU_CSR_AXI_OFFSET +
			reg_entry->reg_offset);
}

static const char *bif_reg_names[IO_AXI_NUM_REGS] = {
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA0),
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA1),
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA2),
	REG_NAME_ENTRY(BIF_AXI_CTRL_DMA3),
	REG_NAME_ENTRY(BIF_AXI_CTRL_MMU),
	REG_NAME_ENTRY(BIF_IMR),
	REG_NAME_ENTRY(BIF_ISR),
	REG_NAME_ENTRY(BIF_ISR_OVF),
	REG_NAME_ENTRY(BIF_TO_ERR_CFG),
	REG_NAME_ENTRY(BIF_ERR_LOG),
	REG_NAME_ENTRY(BIF_ERR_LOG_BUS_ADDR),
	REG_NAME_ENTRY(BIF_PMON_CFG),
	REG_NAME_ENTRY(BIF_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(BIF_PMON_CNT_0),
	REG_NAME_ENTRY(BIF_PMON_CNT_0_STS_ACC),
	REG_NAME_ENTRY(BIF_PMON_CNT_0_STS),
	REG_NAME_ENTRY(BIF_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(BIF_PMON_CNT_1),
	REG_NAME_ENTRY(BIF_PMON_CNT_1_STS_ACC),
	REG_NAME_ENTRY(BIF_PMON_CNT_1_STS),
	REG_NAME_ENTRY(BIF_IER),
	REG_NAME_ENTRY(BIF_ITR),
};

static inline int paintbox_dump_bif_reg(struct paintbox_data *pb,
		uint32_t reg_offset, char *buf, int *written, size_t len)
{
	const char *reg_name = bif_reg_names[REG_INDEX(reg_offset)];

	return dump_ipu_register(pb, IPU_CSR_AXI_OFFSET, reg_offset, reg_name,
			buf, written, len);
}

int paintbox_dump_bif_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_data *pb = debug->pb;
	unsigned int i;
	int ret, written = 0;

	for (i = 0; i < IO_AXI_NUM_REGS; i++) {
		if (bif_reg_names[i] != NULL) {
			ret = paintbox_dump_bif_reg(pb, i * IPU_REG_WIDTH, buf,
					&written, len);
			if (ret < 0) {
				dev_err(pb->dev, "%s: register dump error, %d",
						__func__, ret);
				return ret;
			}
		}
	}

	return written;
}
#endif

/* This function must be called in an interrupt context */
static void paintbox_bif_dump_fifo_state(struct paintbox_data *pb)
{
	uint64_t err_log;

	err_log = paintbox_readq(pb->dev, IPU_CSR_AXI_OFFSET + BIF_ERR_LOG);
	paintbox_writeq(pb->dev, err_log, IPU_CSR_AXI_OFFSET + BIF_ERR_LOG);

	dev_err(pb->dev, "%s: BIF FIFO state:\n", __func__);
	dev_err(pb->dev, "\tWIDs %d RIDs %d\n",
			!!(err_log & BIF_ERR_LOG_TO_ERR_WID_PEND_MASK),
			!!(err_log & BIF_ERR_LOG_TO_ERR_RID_PEND_MASK));
	dev_err(pb->dev, "\tMMU Read Out %d MMU Read RX %d\n",
			!!(err_log & BIF_ERR_LOG_TO_ERR_MMU_RD_OUT_PEND_MASK),
			!!(err_log & BIF_ERR_LOG_TO_ERR_MMU_RD_RX_PEND_MASK));
	dev_err(pb->dev,
			"\tDMA Write: WDATA FIFO %d WPULL FIFO %d WACK FIFO %d Input FIFO %d\n",
			!!(err_log & BIF_ERR_LOG_TO_ERR_DMA_WR_WDATA_PEND_MASK),
			!!(err_log & BIF_ERR_LOG_TO_ERR_DMA_WR_WPULL_PEND_MASK),
			!!(err_log & BIF_ERR_LOG_TO_ERR_DMA_WR_WACK_PEND_MASK),
			!!(err_log & BIF_ERR_LOG_TO_ERR_DMA_WR_IN_PEND_MASK));
	dev_err(pb->dev,
			"\tDMA Read: Input FIFO %d Output FIFO %d\n",
			!!(err_log & BIF_ERR_LOG_TO_ERR_DMA_RD_IN_PEND_MASK),
			!!(err_log & BIF_ERR_LOG_TO_ERR_DMA_RD_OUT_PEND_MASK));
	dev_err(pb->dev,
			"\tJQS Write: Input FIFO %d WDATA FIFO %d RSP FIFO %d PENDID %d\n",
			!!(err_log & BIF_ERR_LOG_TO_ERR_JQS_WR_IN_PEND_MASK),
			!!(err_log & BIF_ERR_LOG_TO_ERR_JQS_WR_WDATA_PEND_MASK),
			!!(err_log & BIF_ERR_LOG_TO_ERR_JQS_WR_RSP_PEND_MASK),
			!!(err_log &
			BIF_ERR_LOG_TO_ERR_JQS_WR_PENDID_PEND_MASK));
	dev_err(pb->dev,
			"\tJQS Read: Input FIFO %d Output FIFO %d\n",
			!!(err_log & BIF_ERR_LOG_TO_ERR_JQS_RD_IN_PEND_MASK),
			!!(err_log & BIF_ERR_LOG_TO_ERR_JQS_RD_OUT_PEND_MASK));
	dev_err(pb->dev, "\tChannel Write: TX FIFO %d\n",
			!!(err_log & BIF_ERR_LOG_TO_ERR_CHAN_WR_TX_PEND_MASK));
	dev_err(pb->dev, "\tChannel Read: TX FIFO %d RX FIFO %d\n",
			!!(err_log & BIF_ERR_LOG_TO_ERR_CHAN_RD_TX_PEND_MASK),
			!!(err_log & BIF_ERR_LOG_TO_ERR_CHAN_RD_RX_PEND_MASK));
}

/* This function must be called in an interrupt context */
static void paintbox_bif_dma_timeout_interrupt(struct paintbox_data *pb,
		const char *str, bool overflow)
{
	if (!overflow) {
		dev_err(pb->dev, "%s: %s timeout error\n", str, __func__);
		paintbox_bif_dump_fifo_state(pb);
	} else {
		dev_err(pb->dev, "%s: %s timeout error (int overflow)\n", str,
				__func__);
	}

	/* A BIF timeout error is a catastrophic error for the IPU.  Complete
	 * and report the error on all DMA channels.
	 */
	dma_report_error_all_channels(pb, -ENOTRECOVERABLE);

	/* TODO:  Initiate a reset of the IPU and block new operations
	 * until the IPU comes out of reset.  Note it might be necessary to
	 * reset the whole Easel chip if the AXI bus is messed up.  As part of
	 * the reset process any MIPI or STP waiters should be released.
	 * b/34518459
	 */
}

/* This function must be called in an interrupt context */
static void paintbox_bif_mmu_timeout_interrupt(struct paintbox_data *pb,
		bool overflow)
{
	if (!overflow) {
		dev_err(pb->dev, "%s: read rx timeout error\n", __func__);
		paintbox_bif_dump_fifo_state(pb);

	} else {
		dev_err(pb->dev, "%s: read rx timeout error (int overflow)\n",
				__func__);
	}

	/* A BIF timeout error is a catastrophic error for the IPU.  Complete
	 * and report the error on all DMA channels.
	 */
	dma_report_error_all_channels(pb, -ENOTRECOVERABLE);

	/* TODO:  Initiate a reset of the IPU and block new operations
	 * until the IPU comes out of reset.  Note it might be necessary to
	 * reset the whole Easel chip if the AXI bus is messed up.  As part of
	 * the reset process any MIPI or STP waiters should be released.
	 * b/34518459
	 */
}

/* This function must be called in an interrupt context */
static void paintbox_bif_mmu_bus_error_interrupt(struct paintbox_data *pb,
		bool overflow)
{
	if (!overflow) {
		unsigned int axi_id;
		uint64_t err_log;

		err_log = paintbox_readq(pb->dev, IPU_CSR_AXI_OFFSET +
				BIF_ERR_LOG);
		paintbox_writeq(pb->dev, err_log, IPU_CSR_AXI_OFFSET +
				BIF_ERR_LOG);

		axi_id = (unsigned int)((err_log &
				BIF_ERR_LOG_BUS_ERR_AXI_ID_MASK) >>
				BIF_ERR_LOG_BUS_ERR_AXI_ID_SHIFT);

		dev_err(pb->dev, "%s AXI ID 0x%02x\n", __func__, axi_id);
	} else {
		dev_err(pb->dev, "%s mmu bus error (interrupt overflow)\n",
				__func__);
	}

	/* An MMU bus error is a catastrophic error for the IPU.  Complete and
	 * report the error on all DMA channels.
	 */
	dma_report_error_all_channels(pb, -ENOTRECOVERABLE);

	/* TODO:  Initiate a reset of the IPU and block new operations
	 * until the IPU comes out of reset.  As part of the reset process any
	 * MIPI or STP waiters should be released.  b/34518459
	 */
}

static inline void paintbox_bif_dma_bus_error_interrupt_overflow(
		struct paintbox_data *pb)
{
	dev_err(pb->dev, "%s: dma bus error (interrupt overflow)\n", __func__);

	/* If the DMA bus error is in the interrupt overflow then there isn't
	 * much that can be done beyond logging the error and report error
	 * back.
	 */
	dma_report_error_all_channels(pb, -ENOTRECOVERABLE);
}

/* This function must be called in an interrupt context */
static void paintbox_bif_dma_bus_error_interrupt(struct paintbox_data *pb)
{
	unsigned int axi_id, channel_id;
	uint32_t err_paddr;
	uint64_t err_log;

	err_log = paintbox_readq(pb->dev, IPU_CSR_AXI_OFFSET + BIF_ERR_LOG);
	paintbox_writeq(pb->dev, err_log, IPU_CSR_AXI_OFFSET + BIF_ERR_LOG);

	err_paddr = paintbox_readl(pb->dev, IPU_CSR_AXI_OFFSET +
			BIF_ERR_LOG_BUS_ADDR);
	paintbox_writel(pb->dev, err_paddr, IPU_CSR_AXI_OFFSET +
			BIF_ERR_LOG_BUS_ADDR);

	channel_id = (unsigned int)((err_log &
			BIF_ERR_LOG_BUS_ERR_CHAN_MASK) >>
			BIF_ERR_LOG_BUS_ERR_CHAN_SHIFT);
	axi_id = (unsigned int)((err_log & BIF_ERR_LOG_BUS_ERR_AXI_ID_MASK) >>
			BIF_ERR_LOG_BUS_ERR_AXI_ID_SHIFT);

	if (err_log & BIF_ERR_LOG_BUS_ERR_CHAN_WRITE_MASK) {
		unsigned int dma_tid = (unsigned int)((err_log &
				BIF_ERR_LOG_BUS_ERR_AXI_ID_MASK) >>
				BIF_ERR_LOG_BUS_ERR_AXI_ID_SHIFT);

		dev_err(pb->dev, "%s: dma channel%u bus error on write\n",
				__func__, channel_id);
		dev_err(pb->dev, "\tphys 0x%08x AIX ID 0x%02x DMA TID 0x%02x\n",
				err_paddr, axi_id, dma_tid);
	} else {
		dev_err(pb->dev, "%s: dma channel%u bus error on read\n",
				__func__, channel_id);
		dev_err(pb->dev, "\tphys 0x%08x AIX ID 0x%02x\n", err_paddr,
				axi_id);
	}

	dma_report_channel_error(pb, channel_id, -ENOTRECOVERABLE);
}

/* This function must be called in an interrupt context */
void paintbox_bif_interrupt(struct paintbox_data *pb)
{
	uint32_t status;

	status = paintbox_readl(pb->dev, IPU_CSR_AXI_OFFSET + BIF_ISR);
	paintbox_writel(pb->dev, status, IPU_CSR_AXI_OFFSET + BIF_ISR);

	if (status & BIF_ISR_TO_ERR_CHAN_WR_MASK)
		paintbox_bif_dma_timeout_interrupt(pb, "write", false);

	if (status & BIF_ISR_TO_ERR_CHAN_RD_MASK)
		paintbox_bif_dma_timeout_interrupt(pb, "read", false);

	if (status & BIF_ISR_TO_ERR_MMU_RD_MASK)
		paintbox_bif_mmu_timeout_interrupt(pb, false);

	if (status & BIF_ISR_BUS_ERR_MMU_MASK)
		paintbox_bif_mmu_bus_error_interrupt(pb, false);

	if (status & BIF_ISR_TO_ERR_CHAN_RD_MASK)
		paintbox_bif_dma_bus_error_interrupt(pb);

	/* Check to see if an interrupt overflow occurred. */
	status = paintbox_readl(pb->dev, IPU_CSR_AXI_OFFSET + BIF_ISR_OVF);
	paintbox_writel(pb->dev, status, IPU_CSR_AXI_OFFSET + BIF_ISR_OVF);

	if (status & BIF_ISR_OVF_TO_ERR_CHAN_WR_MASK)
		paintbox_bif_dma_timeout_interrupt(pb, "write", true);

	if (status & BIF_ISR_OVF_TO_ERR_CHAN_RD_MASK)
		paintbox_bif_dma_timeout_interrupt(pb, "read", true);

	if (status & BIF_ISR_OVF_TO_ERR_MMU_RD_MASK)
		paintbox_bif_mmu_timeout_interrupt(pb, true);

	if (status & BIF_ISR_OVF_BUS_ERR_MMU_MASK)
		paintbox_bif_mmu_bus_error_interrupt(pb, true);

	if (status & BIF_ISR_OVF_TO_ERR_CHAN_WR_MASK)
		paintbox_bif_dma_bus_error_interrupt_overflow(pb);
}

int paintbox_bif_init(struct paintbox_data *pb)
{
#ifdef CONFIG_PAINTBOX_DEBUG
	paintbox_debug_create_entry(pb, &pb->io.axi_debug,
			pb->debug_root, "bif", -1,
			paintbox_dump_bif_registers, NULL, &pb->io);

	paintbox_debug_create_reg_entries(pb, &pb->io.axi_debug,
			bif_reg_names, IO_AXI_NUM_REGS,
			paintbox_bif_reg_entry_write,
			paintbox_bif_reg_entry_read);
#endif
	dev_dbg(pb->dev, "bif: base 0x%08x len %lu\n", IPU_CSR_AXI_OFFSET,
			IO_AXI_BLOCK_LEN);

	return 0;
}

/* All sessions must be released before remove can be called. */
void paintbox_bif_remove(struct paintbox_data *pb)
{
	paintbox_writel(pb->dev, 0, IPU_CSR_AXI_OFFSET + BIF_IMR);

#ifdef CONFIG_PAINTBOX_DEBUG
	paintbox_debug_free_reg_entries(&pb->io.axi_debug);
	paintbox_debug_free_entry(&pb->io.axi_debug);
#endif
}
