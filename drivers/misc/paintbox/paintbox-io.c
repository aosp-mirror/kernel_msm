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

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-bif.h"
#include "paintbox-bus.h"
#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-mmu.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

#ifdef CONFIG_PAINTBOX_DEBUG
static uint64_t paintbox_io_apb_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	return paintbox_readq(pb->dev, IPU_CSR_APB_OFFSET +
			reg_entry->reg_offset);
}

static void paintbox_io_apb_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	paintbox_writeq(pb->dev, val, IPU_CSR_APB_OFFSET +
			reg_entry->reg_offset);
}

static const char *io_apb_reg_names[IO_APB_NUM_REGS] = {
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

static inline int paintbox_dump_io_apb_reg(struct paintbox_data *pb,
		uint32_t reg_offset, char *buf, int *written, size_t len)
{
	const char *reg_name = io_apb_reg_names[REG_INDEX(reg_offset)];

	return dump_ipu_register(pb, IPU_CSR_APB_OFFSET, reg_offset, reg_name,
			buf, written, len);
}

int paintbox_dump_io_apb_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_data *pb = debug->pb;
	unsigned int i;
	int ret, written = 0;

	for (i = 0; i < IO_APB_NUM_REGS; i++) {
		if (io_apb_reg_names[i] != NULL) {
			ret = paintbox_dump_io_apb_reg(pb, i * IPU_REG_WIDTH,
					buf, &written, len);
			if (ret < 0)
				goto err_exit;
		}
	}

	return written;

err_exit:
	dev_err(pb->dev, "%s: register dump error, err = %d", __func__, ret);
	return ret;
}
#endif

static irqreturn_t paintbox_io_interrupt(struct device *dev, uint32_t status)
{
	struct paintbox_data *pb = dev_get_drvdata(dev);
	ktime_t timestamp;

#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time;

	start_time = pb->stats.ioctl_time_enabled ? ktime_get_boottime() :
			ktime_set(0, 0);
#endif

	pb->io.irq_activations++;

	timestamp = ktime_get_boottime();

	pb->io.ipu_interrupts++;

	if (status & IPU_ISR_BIF_INTR_MASK)
		paintbox_bif_interrupt(pb);

	if (status & IPU_ISR_MMU_INTR_MASK)
		paintbox_mmu_interrupt(pb);

	if (status & IPU_ISR_DMA_CHAN_INTR_MASK)
		paintbox_dma_channel_interrupt(pb, timestamp);

	if (status & IPU_ISR_DMA_ERR_INTR_MASK)
		paintbox_dma_channel_error_interrupt(pb, timestamp);

	if (status & IPU_ISR_STP_INTR_MASK) {
		uint32_t stp_status = paintbox_readl(pb->dev,
				IPU_CSR_APB_OFFSET + IPU_STP_ISR);
		paintbox_stp_interrupt(pb, stp_status, timestamp);
	}

	if (status & IPU_ISR_STP_ERR_INTR_MASK) {
		uint32_t error_status = paintbox_readl(pb->dev,
				IPU_CSR_APB_OFFSET + STP_ERR_ISR);
		paintbox_stp_error_interrupt(pb, error_status, timestamp);
	}

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb,
				PB_STATS_IO_INTERRUPT_HANDLE, start_time,
				ktime_get_boottime(), 0);
#endif

	return IRQ_HANDLED;
}

void paintbox_enable_dma_channel_interrupt(struct paintbox_data *pb,
		unsigned int channel_id)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);
	pb->io.regs.dma_chan_imr |= 1UL << channel_id;
	paintbox_writel(pb->dev, pb->io.regs.dma_chan_imr,
			IPU_CSR_APB_OFFSET + DMA_CHAN_IMR);
	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_disable_dma_channel_interrupt(struct paintbox_data *pb,
		unsigned int channel_id)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);
	pb->io.regs.dma_chan_imr &= ~(1UL << channel_id);
	paintbox_writel(pb->dev, pb->io.regs.dma_chan_imr,
			IPU_CSR_APB_OFFSET + DMA_CHAN_IMR);
	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_enable_dma_channel_error_interrupt(struct paintbox_data *pb,
		unsigned int channel_id)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);
	pb->io.regs.dma_err_imr |= 1UL << channel_id;
	paintbox_writel(pb->dev, pb->io.regs.dma_err_imr,
			IPU_CSR_APB_OFFSET + DMA_ERR_IMR);
	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_disable_dma_channel_error_interrupt(struct paintbox_data *pb,
		unsigned int channel_id)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);
	pb->io.regs.dma_err_imr &= ~(1UL << channel_id);
	paintbox_writel(pb->dev, pb->io.regs.dma_err_imr,
			IPU_CSR_APB_OFFSET + DMA_ERR_IMR);
	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_enable_stp_interrupt(struct paintbox_data *pb,
		unsigned int stp_id)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);
	pb->io.regs.ipu_stp_imr |= 1UL << stp_id_to_index(stp_id);
	paintbox_writel(pb->dev, pb->io.regs.ipu_stp_imr,
			IPU_CSR_APB_OFFSET + IPU_STP_IMR);
	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_disable_stp_interrupt(struct paintbox_data *pb,
		unsigned int stp_id)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);
	pb->io.regs.ipu_stp_imr &= ~(1UL << stp_id_to_index(stp_id));
	paintbox_writel(pb->dev, pb->io.regs.ipu_stp_imr,
			IPU_CSR_APB_OFFSET + IPU_STP_IMR);
	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_enable_stp_error_interrupt(struct paintbox_data *pb,
		unsigned int stp_id)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);
	pb->io.regs.stp_err_imr |= 1UL << stp_id_to_index(stp_id);
	paintbox_writel(pb->dev, pb->io.regs.stp_err_imr,
			IPU_CSR_APB_OFFSET + STP_ERR_IMR);
	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_disable_stp_error_interrupt(struct paintbox_data *pb,
		unsigned int stp_id)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);
	pb->io.regs.stp_err_imr &= ~(1UL << stp_id_to_index(stp_id));
	paintbox_writel(pb->dev, pb->io.regs.stp_err_imr,
			IPU_CSR_APB_OFFSET + STP_ERR_IMR);
	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

/* All sessions must be released before remove can be called. */
void paintbox_io_apb_remove(struct paintbox_data *pb)
{
	paintbox_release_irq(pb->dev);

#ifdef CONFIG_PAINTBOX_DEBUG
	paintbox_debug_free_reg_entries(&pb->io.apb_debug);
	paintbox_debug_free_entry(&pb->io.apb_debug);
#endif
}

/* Resets shadows. */
void paintbox_io_apb_post_ipu_reset(struct paintbox_data *pb)
{
	pb->io.regs.dma_chan_imr = DMA_CHAN_IMR_DEF;
	pb->io.regs.dma_err_imr = DMA_ERR_IMR_DEF;
	pb->io.regs.ipu_stp_imr = IPU_STP_IMR_DEF;
	pb->io.regs.stp_err_imr = STP_ERR_IMR_DEF;
	pb->io.regs.dma_chan_en = IPU_DMA_CHAN_EN_DEF;
}

int paintbox_io_apb_init(struct paintbox_data *pb)
{
	spin_lock_init(&pb->io.io_lock);

	pb->io.regs.dma_chan_imr = DMA_CHAN_IMR_DEF;
	pb->io.regs.dma_err_imr = DMA_ERR_IMR_DEF;
	pb->io.regs.ipu_stp_imr = IPU_STP_IMR_DEF;
	pb->io.regs.stp_err_imr = STP_ERR_IMR_DEF;
	pb->io.regs.dma_chan_en = IPU_DMA_CHAN_EN_DEF;

#ifdef CONFIG_PAINTBOX_DEBUG
	paintbox_debug_create_entry(pb, &pb->io.apb_debug,
			pb->debug_root, "apb", -1,
			paintbox_dump_io_apb_registers, NULL, &pb->io);

	paintbox_debug_create_reg_entries(pb, &pb->io.apb_debug,
			io_apb_reg_names, IO_APB_NUM_REGS,
			paintbox_io_apb_reg_entry_write,
			paintbox_io_apb_reg_entry_read);
#endif

	/* TODO(ahampson):  MMU interrupts are routed to the IPU driver right
	 * now but the processing of MMU interrupts should be moved to the the
	 * IOMMU driver.
	 */
	paintbox_request_irq(pb->dev, &paintbox_io_interrupt,
			IPU_ISR_BIF_INTR_MASK | IPU_ISR_MMU_INTR_MASK |
			IPU_ISR_DMA_CHAN_INTR_MASK | IPU_ISR_DMA_ERR_INTR_MASK |
			IPU_ISR_STP_INTR_MASK | IPU_ISR_STP_ERR_INTR_MASK);

	/* Update the number of available interrupts reported to the user space.
	 * This value is also used to allocate the number of IRQ waiter objects.
	 */
	pb->io.num_interrupts = pb->dma.num_channels + pb->stp.num_stps
			+ NUM_BIF_INTERRUPTS + NUM_MMU_INTERRUPTS;
	pb->io.available_interrupt_mask = (1ULL << pb->io.num_interrupts) - 1;

	dev_dbg(pb->dev, "io_apb: base 0x%08x len %lu\n", IPU_CSR_APB_OFFSET,
			IO_APB_BLOCK_LEN);

	return 0;
}
