/*
 * DMA support V2 for the Paintbox programmable IPU
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

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>

#include <uapi/paintbox.h>
#include <uapi/paintbox_v2.h>

#include "paintbox-bus.h"
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-dma-debug.h"
#include "paintbox-dma-dram.h"
#include "paintbox-dma-lbp.h"
#include "paintbox-dma-stp.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-lbp.h"
#include "paintbox-power.h"
#include "paintbox-regs.h"

#define MAX_DMA_STOP_ATTEMPTS 2

#define DMA_RESET_HOLD_PERIOD 10 /* us */

static inline bool paintbox_dma_src_is_mipi(uint32_t mode)
{
	return ((mode & DMA_CHAN_MODE_SRC_MASK) >> DMA_CHAN_MODE_SRC_SHIFT) ==
			DMA_CHAN_MODE_SRC_MIPI_IN;
}

static inline bool paintbox_dma_src_is_lbp(uint32_t mode)
{
	return ((mode & DMA_CHAN_MODE_SRC_MASK) >> DMA_CHAN_MODE_SRC_SHIFT) ==
			DMA_CHAN_MODE_SRC_LBP;
}

static inline bool paintbox_dma_dst_is_mipi(uint32_t mode)
{
	return ((mode & DMA_CHAN_MODE_DST_MASK) >> DMA_CHAN_MODE_DST_SHIFT) ==
			DMA_CHAN_MODE_DST_MIPI_OUT;
}

static inline bool paintbox_dma_dst_is_lbp(uint32_t mode)
{
	return ((mode & DMA_CHAN_MODE_DST_MASK) >> DMA_CHAN_MODE_DST_SHIFT) ==
			DMA_CHAN_MODE_DST_LBP;
}

static inline bool paintbox_dma_is_gather(uint32_t mode)
{
	return mode & DMA_CHAN_MODE_GATHER_MASK;
}

static inline bool paintbox_dma_is_gather_channel(
		struct paintbox_dma_channel *channel)
{
	return channel->session->dma_gather_channel_mask &
			(1 << channel->channel_id);
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void paintbox_dma_enable_channel_interrupts(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	if (channel->interrupts_enabled)
		return;

	channel->interrupts_enabled = true;

	pb->dma.regs.irq_imr |= 1 << channel->channel_id;
	paintbox_writel(pb->dev, pb->dma.regs.irq_imr,
			IPU_CSR_DMA_TOP_OFFSET + DMA_IRQ_IMR);

	/* Enable both VA_ERR and MS_ERR (MIPI Overflow) error interrupts for
	 * the channel.
	 */
	pb->dma.regs.irq_va_err_imr |= 1 << channel->channel_id;
	paintbox_writel(pb->dev, pb->dma.regs.irq_va_err_imr,
			IPU_CSR_DMA_TOP_OFFSET + DMA_IRQ_VA_ERR_IMR);

	/* Enable the DMA channel interrupt in the top-level IPU_IMR */
	paintbox_enable_dma_channel_interrupt(pb, channel->channel_id);

	/* Enable the DMA channel error interrupt in the top-level IPU_IMR */
	paintbox_enable_dma_channel_error_interrupt(pb, channel->channel_id);

	dev_dbg(pb->dev, "dma channel%u: interrupts enabled\n",
			channel->channel_id);
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void paintbox_dma_disable_channel_interrupts(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	if (!channel->interrupts_enabled)
		return;

	channel->interrupts_enabled = false;

	pb->dma.regs.irq_imr &= ~(1 << channel->channel_id);
	paintbox_writel(pb->dev, pb->dma.regs.irq_imr,
			IPU_CSR_DMA_TOP_OFFSET + DMA_IRQ_IMR);

	/* Disable both VA_ERR and MS_ERR (MIPI Overflow) error interrupts for
	 * the channel.
	 */
	pb->dma.regs.irq_va_err_imr &= ~(1 << channel->channel_id);
	paintbox_writel(pb->dev, pb->dma.regs.irq_va_err_imr,
			IPU_CSR_DMA_TOP_OFFSET + DMA_IRQ_VA_ERR_IMR);

	paintbox_disable_dma_channel_interrupt(pb, channel->channel_id);
	paintbox_disable_dma_channel_error_interrupt(pb, channel->channel_id);

	dev_dbg(pb->dev, "dma channel%u: interrupts disabled\n",
			channel->channel_id);
}

/* The caller to this function must hold pb->lock. */
static struct paintbox_dma_transfer *paintbox_allocate_transfer(
		struct paintbox_data *pb)
{
	struct paintbox_dma_transfer *transfer;

	if (pb->dma.free_count == 0) {
		transfer = kzalloc(sizeof(struct paintbox_dma_transfer),
				GFP_KERNEL);
	} else {
		transfer = list_entry(pb->dma.free_list.next,
				struct paintbox_dma_transfer, entry);
		list_del(&transfer->entry);
		pb->dma.free_count--;
		memset(transfer, 0, sizeof(struct paintbox_dma_transfer));
	}

	return transfer;
}

/* The caller to this function must hold pb->lock. */
static void paintbox_free_transfer(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer)
{
	list_add_tail(&transfer->entry, &pb->dma.free_list);
	pb->dma.free_count++;
}

/* The caller to this function must hold pb->lock */
int validate_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int channel_id)
{
	if (channel_id >= pb->dma.num_channels) {
		dev_err(pb->dev, "%s: invalid dma channel id %d\n", __func__,
				channel_id);
		return -EINVAL;
	}

	if (pb->dma.channels[channel_id].session != session) {
		dev_err(pb->dev, "%s: access error, dma channel id %d\n",
				__func__, channel_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
struct paintbox_dma_channel *get_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int channel_id,
		int *err)
{
	int ret;

	ret = validate_dma_channel(pb, session, channel_id);
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	*err = 0;
	return &pb->dma.channels[channel_id];
}

/* The caller to this function must hold pb->lock. */
static void drain_queue(struct paintbox_data *pb,
		struct list_head *transfer_list, unsigned int *count)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	while (!list_empty(transfer_list)) {
		struct paintbox_dma_transfer *transfer;

		transfer = list_entry(transfer_list->next,
				struct paintbox_dma_transfer, entry);
		list_del(&transfer->entry);
		(*count)--;
		WARN_ON(*count < 0);

		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

		ipu_dma_release_buffer(pb, transfer);

		paintbox_free_transfer(pb, transfer);

		spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);
	}

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

static void paintbox_dma_discard_queue_work(struct work_struct *work)
{
	struct paintbox_dma *dma = container_of(work, struct paintbox_dma,
			discard_queue_work);
	struct paintbox_data *pb = container_of(dma, struct paintbox_data, dma);

	mutex_lock(&pb->lock);
	drain_queue(pb, &dma->discard_list, &dma->discard_count);
	mutex_unlock(&pb->lock);
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void paintbox_dma_discard_transfer(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer)
{
	list_add_tail(&transfer->entry, &pb->dma.discard_list);
	pb->dma.discard_count++;
	queue_work(system_wq, &pb->dma.discard_queue_work);
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void paintbox_dma_enqueue_pending_read(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	list_add_tail(&transfer->entry, &channel->completed_list);
	channel->completed_count++;
	channel->stats.reported_completions++;
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void paintbox_dma_enqueue_pending_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	list_add_tail(&transfer->entry, &channel->pending_list);
	channel->pending_count++;
}

static struct paintbox_dma_transfer *paintbox_dequeue_active_transfer(
		struct paintbox_data *pb, struct paintbox_dma_channel *channel)
{
	struct paintbox_dma_transfer *transfer;

	if (list_empty(&channel->active_list) || channel->active_count == 0) {
		dev_warn(pb->dev,
				"%s: channel%u no active transfer\n", __func__,
				channel->channel_id);
		return NULL;
	}

	transfer = list_entry(channel->active_list.next,
				struct paintbox_dma_transfer, entry);
	list_del(&transfer->entry);
	channel->active_count--;

	return transfer;
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void dma_report_completion(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer, ktime_t timestamp,
		int err)
{
	dev_dbg(pb->dev, "dma channel%u: transfer %p completed err %d\n",
			channel->channel_id, transfer, err);

	if (!transfer->notify_on_completion) {
		paintbox_dma_discard_transfer(pb, transfer);
		channel->stats.reported_discards++;
		return;
	}

	/* Buffers that need to be copied back to user space go on a separate
	 * read queue that is emptied by read_dma_transfer_ioctl().  All other
	 * transfers go on a discard queue and are cleaned up by a work queue.
	 *
	 * TODO(ahampson): Remove the read queue support when b/62371806 is
	 * fixed.
	 */
	if (transfer->buffer_type == DMA_DRAM_BUFFER_USER &&
			transfer->dir == DMA_FROM_DEVICE) {
		paintbox_dma_enqueue_pending_read(pb, channel, transfer);
	} else {
		paintbox_dma_discard_transfer(pb, transfer);
		channel->stats.reported_discards++;
	}

	paintbox_irq_waiter_signal(pb, channel->irq, timestamp, 0 /* data */,
			err);
}

int bind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_interrupt_config __user *user_req;
	struct dma_interrupt_config req;
	struct paintbox_dma_channel *channel;
	int ret;

	user_req = (struct dma_interrupt_config __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, req.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(pb->dev, "dma channel%u bind irq%u\n", req.channel_id,
			req.interrupt_id);

	ret = bind_dma_interrupt(pb, session, channel, req.interrupt_id);

	mutex_unlock(&pb->lock);

	return ret;
}

int unbind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = unbind_dma_interrupt(pb, session, channel);
	if (ret < 0) {
		dev_err(pb->dev, "dma channel%u unbind irq failed, err %d\n",
				channel_id, ret);
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(pb->dev, "dma channel%u unbind irq\n", channel_id);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock and pb->dma.dma_lock */
static void dma_reset_channel_locked(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, bool force)
{
	struct paintbox_dma_transfer *transfer;

	/* If there are no active transfers and a reset is not being forced then
	 * there is no need to reset the channel.
	 */
	if (!force && channel->active_count == 0) {
		paintbox_dma_disable_channel_interrupts(pb, channel);
		paintbox_pm_disable_dma_channel(pb, channel);
		return;
	}

	paintbox_dma_select_channel(pb, channel->channel_id);

	paintbox_writeq(pb->dev, DMA_CHAN_CTRL_CHAN_RESET_MASK,
			IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_CTRL);

	/* Reset bits are not self-clearing so clear it here. */
	paintbox_writeq(pb->dev, 0, IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_CTRL);

	if (channel->active_count > 0) {
		channel->active_count--;

		transfer = list_entry(channel->active_list.next,
				struct paintbox_dma_transfer, entry);
		if (!WARN_ON(transfer == NULL)) {
			list_del(&transfer->entry);
			dma_report_completion(pb, channel, transfer,
					ktime_get_boottime(), -ECANCELED);
		}
	}
}

/* The caller to this function must hold pb->lock */
void dma_reset_channel(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	dma_reset_channel_locked(pb, channel, false /* force */);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
void dma_stop_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;
	uint32_t mode;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	/* If the DMA channel is powered down then there is nothing to stop. */
	if (!channel->pm_enabled) {
		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
		return;
	}

	paintbox_dma_select_channel(pb, channel->channel_id);

	/* Disable the channel to prevent any pending transfers from starting.
	 */
	mode = paintbox_readl(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_MODE);
	if (mode & DMA_CHAN_MODE_CHAN_ENA_MASK) {
		mode &= ~DMA_CHAN_MODE_CHAN_ENA_MASK;
		paintbox_writel(pb->dev, mode, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_MODE);
	}

	/* If there is no active transfer then disable the interrupt and
	 * power down the channel.
	 */
	if (channel->active_count == 0) {
		paintbox_dma_disable_channel_interrupts(pb, channel);
		paintbox_pm_disable_dma_channel(pb, channel);

		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
		return;
	}

	mode = paintbox_readl(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_MODE_RO);

	/* If there is an active transfer then issue a stop request. */
	paintbox_writeq(pb->dev, DMA_CHAN_CTRL_STOP_MASK,
			IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_CTRL);

	reinit_completion(&channel->stop_completion);

	channel->stop_request = true;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	dev_dbg(pb->dev, "dma channel%u stopping\n", channel->channel_id);

	/* Wait for the EOF frame interrupt to occur.  If we timeout waiting
	 * for the interrupt or the completion is interrupted by a signal then
	 * the DMA channel will be reset below.
	 */
	wait_for_completion_interruptible_timeout(&channel->stop_completion,
			usecs_to_jiffies(max(DMA_STOP_TIMEOUT_US,
			MIPI_CLEANUP_TIMEOUT_US)));

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	/* Re-select the channel, the DMA_SEL field might have changed while the
	 * pb->dma.dma_lock spinlock was not held.
	 */
	paintbox_dma_select_channel(pb, channel->channel_id);

	/* Stop bits are not self-clearing so clear it here. */
	paintbox_writeq(pb->dev, 0, IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_CTRL);

	paintbox_dma_disable_channel_interrupts(pb, channel);

	/* If the transfer was to or from a line buffer then reset the
	 * line buffer too.
	 */
	if (paintbox_dma_src_is_lbp(mode) || paintbox_dma_dst_is_lbp(mode)) {
		uint64_t chan_node = paintbox_readl(pb->dev,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_NODE_RO);

		reset_lb(pb, chan_node & DMA_CHAN_NODE_RO_CORE_ID_MASK,
				(chan_node & DMA_CHAN_NODE_RO_LB_ID_MASK) >>
				DMA_CHAN_NODE_RO_LB_ID_SHIFT);
	}

	/* If the channel didn't stop within the timeout then reset it.  This
	 * situation can occur if there are no outstanding requests between the
	 * src and dst or if the transfer has not started.
	 */
	if (channel->stop_request) {
		channel->stop_request = false;

		dma_reset_channel_locked(pb, channel, false /* force */);
	}

	/* If there are no active transfers then power down the DMA channel. */
	if (channel->active_count == 0)
		paintbox_pm_disable_dma_channel(pb, channel);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	dev_dbg(pb->dev, "dma channel%u stopped\n", channel->channel_id);
}

/* The caller to this function must hold pb->lock */
void release_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;

	dev_dbg(pb->dev, "dma channel%u release\n", channel->channel_id);

	dma_stop_transfer(pb, channel);

	drain_queue(pb, &channel->pending_list, &channel->pending_count);
	drain_queue(pb, &channel->active_list, &channel->active_count);
	drain_queue(pb, &channel->completed_list, &channel->completed_count);
	drain_queue(pb, &pb->dma.discard_list, &pb->dma.discard_count);

	/* Remove the DMA channel from the session. */
	list_del(&channel->session_entry);

	channel->session = NULL;
	pb->dma.available_channel_mask |= 1ULL << channel->channel_id;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	/* Make sure the DMA channel is powered down. */
	if (!WARN_ON(channel->active_count != 0))
		paintbox_pm_disable_dma_channel(pb, channel);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

int release_dma_channel_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* Unbind any associated interrupt */
	unbind_dma_interrupt(pb, session, channel);

	release_dma_channel(pb, session, channel);
	signal_completion_on_first_alloc_waiter(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

int flush_dma_transfers_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_transfer_flush __user *user_req;
	struct dma_transfer_flush req;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	user_req = (struct dma_transfer_flush __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	channel = get_dma_channel(pb, session, req.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(pb->dev, "dma channel%u flush transfers\n", req.channel_id);

	if (req.flush_pending)
		drain_queue(pb, &channel->pending_list,
				&channel->pending_count);

	if (req.flush_active) {
		/* Stop any running transfers before flushing the active queue.
		 */
		dma_stop_transfer(pb, channel);
		drain_queue(pb, &channel->active_list, &channel->active_count);
	}

	if (req.flush_completed)
		drain_queue(pb, &channel->completed_list,
				&channel->completed_count);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock */
int allocate_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int channel_id)
{
	struct paintbox_dma_channel *channel;

	channel = &pb->dma.channels[channel_id];
	if (channel->session) {
		dev_err(pb->dev, "%s: access error, dma channel id %d\n",
				__func__, channel_id);
		return -EACCES;
	}

	dev_dbg(pb->dev, "dma channel%u allocated\n", channel_id);

	channel->session = session;
	list_add_tail(&channel->session_entry, &session->dma_list);
	pb->dma.available_channel_mask &= ~(1ULL << channel_id);

	return 0;
}

int allocate_dma_channel_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret;
	unsigned int channel_id = (unsigned int)arg;

	if (channel_id >= pb->dma.num_channels) {
		dev_err(pb->dev, "%s: invalid dma channel id %d\n", __func__,
				channel_id);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	ret = allocate_dma_channel(pb, session, channel_id);
	if (ret < 0)
		dev_err(pb->dev,
				"%s: allocate dma channel id %d error %d\n",
				__func__, channel_id, ret);
	mutex_unlock(&pb->lock);

	return ret;
}

/* The caller to this function must hold pb->dma.dma_lock and DMA_CHAN_SEL must
 * be set.
 */
static inline void paintbox_dma_load_channel_regs(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	if (transfer->chan_img_format != channel->regs.chan_img_format) {
		channel->regs.chan_img_format = transfer->chan_img_format;
		paintbox_writel(pb->dev, transfer->chan_img_format,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_IMG_FORMAT);
	}

	if (transfer->chan_img_size != channel->regs.chan_img_size) {
		channel->regs.chan_img_size = transfer->chan_img_size;
		paintbox_writel(pb->dev, transfer->chan_img_size,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_IMG_SIZE);
	}

	if (transfer->chan_img_pos != channel->regs.chan_img_pos) {
		channel->regs.chan_img_pos = transfer->chan_img_pos;
		paintbox_writeq(pb->dev, transfer->chan_img_pos,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_IMG_POS);
	}

	if (transfer->chan_img_layout != channel->regs.chan_img_layout) {
		channel->regs.chan_img_layout = transfer->chan_img_layout;
		paintbox_writeq(pb->dev, transfer->chan_img_layout,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_IMG_LAYOUT);
	}

	if (transfer->chan_bif_xfer != channel->regs.chan_bif_xfer) {
		channel->regs.chan_bif_xfer = transfer->chan_bif_xfer;
		paintbox_writel(pb->dev, transfer->chan_bif_xfer,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_BIF_XFER);
	}

	/* TODO(ahampson):  The trace_combiner used in register trace generation
	 * relies on the DMA_CHAN_VA and DMA_CHAN_VA_BDRY registers being
	 * present in register traces for DMA transfers.  When running on the
	 * Simulator the driver needs to always write these registers
	 * until the trace combiner is fixed.  b/37688363
	 */
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	channel->regs.chan_va = transfer->chan_va;
	paintbox_writeq(pb->dev, transfer->chan_va, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_VA);

	channel->regs.chan_va_bdry = transfer->chan_va_bdry;
	paintbox_writeq(pb->dev, transfer->chan_va_bdry,
			IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_VA_BDRY);
#else
	if (transfer->chan_va != channel->regs.chan_va) {
		channel->regs.chan_va = transfer->chan_va;
		paintbox_writeq(pb->dev, transfer->chan_va,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_VA);
	}

	if (transfer->chan_va_bdry != channel->regs.chan_va_bdry) {
		channel->regs.chan_va_bdry = transfer->chan_va_bdry;
		paintbox_writeq(pb->dev, transfer->chan_va_bdry,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_VA_BDRY);
	}
#endif

	if (transfer->chan_noc_xfer != channel->regs.chan_noc_xfer) {
		channel->regs.chan_noc_xfer = transfer->chan_noc_xfer;
		paintbox_writeq(pb->dev, transfer->chan_noc_xfer,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_NOC_XFER);
	}

	if (transfer->chan_ssp_config != channel->regs.chan_ssp_config) {
		channel->regs.chan_ssp_config = transfer->chan_ssp_config;
		paintbox_writel(pb->dev, transfer->chan_ssp_config,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_SSP_CFG);
	}

	if (transfer->chan_node != channel->regs.chan_node) {
		channel->regs.chan_node = transfer->chan_node;
		paintbox_writeq(pb->dev, transfer->chan_node,
				IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_NODE);
	}
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void commit_transfer_to_hardware(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	list_add_tail(&transfer->entry, &channel->active_list);
	channel->active_count++;

	paintbox_dma_select_channel(pb, channel->channel_id);
	paintbox_dma_load_channel_regs(pb, channel, transfer);
	paintbox_dma_enable_channel_interrupts(pb, channel);

	if (channel->stats.time_stats_enabled)
		transfer->start_time = ktime_get_boottime();

	/* Write the channel mode register last as this will enqueue the
	 * transfer into the hardware.
	 */
	paintbox_writel(pb->dev, transfer->chan_mode,
			IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_MODE);

	LOG_DMA_REGISTERS(pb, channel);
}

static int paintbox_dma_setup_transfer(struct paintbox_data *pb,
	struct paintbox_session *session,
	struct dma_transfer_config_v2 *config)
{
	struct paintbox_dma_channel *channel;
	struct paintbox_dma_transfer *transfer;
	unsigned long irq_flags;
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t enq_start;
	ktime_t setup_start;
#endif
	int ret = 0;

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled) {
		setup_start = ktime_get_boottime();
	} else {
		enq_start = ktime_set(0, 0);
		setup_start = ktime_set(0, 0);
	}
#endif

	mutex_lock(&pb->lock);

	channel = get_dma_channel(pb, session, config->channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	if (channel->stats.time_stats_enabled)
		channel->stats.setup_start_time = ktime_get_boottime();

	transfer = paintbox_allocate_transfer(pb);
	if (!transfer) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev, "%s: allocation failure\n", __func__);
		return -ENOMEM;
	}

	switch (config->transfer_type) {
	case DMA_DRAM_TO_LBP:
		ret = dma_setup_dram_to_lbp_transfer(pb, session, channel,
				transfer, config);
		break;
	case DMA_DRAM_TO_STP:
		ret = dma_setup_dram_to_stp_transfer(pb, session, channel,
				transfer, config);
		break;
	case DMA_LBP_TO_DRAM:
		ret = dma_setup_lbp_to_dram_transfer(pb, session, channel,
				transfer, config);
		break;
	default:
		dev_err(pb->dev, "dma: invalid transfer type %u\n",
				config->transfer_type);
		ret = -EINVAL;
	}

	if (ret < 0) {
		paintbox_free_transfer(pb, transfer);
		mutex_unlock(&pb->lock);
		return ret;
	}

	transfer->notify_on_completion = config->notify_on_completion;
	transfer->auto_start_transfer = config->auto_start_transfer;

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled) {
		enq_start = ktime_get_boottime();
		paintbox_debug_log_non_ioctl_stats(pb, PB_STATS_DMA_SETUP,
				setup_start, enq_start, 0);
	}
#endif

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	/* If the transfer is marked for auto start then check to see if there
	 * are any transfers in the pending queue and that there is space in the
	 * active queue before.  If these are conditions are met then the
	 * transfer can be enqueued in the active queue, otherwise it goes to
	 * the pending queue.
	 */
	if (transfer->auto_start_transfer && channel->pending_count == 0 &&
			channel->active_count < MAX_ACTIVE_TRANSFERS) {
		paintbox_pm_enable_dma_channel(pb, channel);
		commit_transfer_to_hardware(pb, channel, transfer);
	} else {
		paintbox_dma_enqueue_pending_transfer(pb, channel, transfer);
	}

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb,  PB_STATS_DMA_ENQ,
				enq_start, ktime_get_boottime(), 0);
#endif

	if (channel->stats.time_stats_enabled)
		channel->stats.setup_finish_time = ktime_get_boottime();

	mutex_unlock(&pb->lock);

	return 0;
}

int setup_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_transfer_config __user *user_config;
	struct dma_transfer_config config;
	struct dma_transfer_config_v2 v2_config;

	user_config = (struct dma_transfer_config __user *)arg;
	if (copy_from_user(&config, user_config, sizeof(config)))
		return -EFAULT;

	memset(&v2_config, 0, sizeof(v2_config));

	/* Translate the v1 structure into the V2 structure */
	v2_config.channel_id = config.channel_id;
	v2_config.transfer_type = config.transfer_type;
	v2_config.sheet_width = config.sheet_width;
	v2_config.sheet_height = config.sheet_height;
	v2_config.stripe_height = config.stripe_height;
	v2_config.noc_outstanding = config.noc_outstanding;
	v2_config.retry_interval = config.retry_interval;
	v2_config.notify_on_completion = config.notify_on_completion;
	v2_config.auto_start_transfer = config.auto_start_transfer;
	memcpy(&v2_config.img, &config.img, sizeof(config.img));

	switch (config.transfer_type) {
	case DMA_DRAM_TO_LBP:
		memcpy(&v2_config.src.dram, &config.src.dram,
				sizeof(config.src.dram));
		memcpy(&v2_config.dst.lbp.base, &config.dst.lbp,
				sizeof(config.dst.lbp));
		break;
	case DMA_DRAM_TO_STP:
		memcpy(&v2_config.src.dram, &config.src.dram,
				sizeof(config.src.dram));
		memcpy(&v2_config.dst.stp.base, &config.dst.stp,
				sizeof(config.dst.stp));
		break;
	case DMA_LBP_TO_DRAM:
		memcpy(&v2_config.src.lbp.base, &config.src.lbp,
				sizeof(config.src.lbp));
		memcpy(&v2_config.dst.dram, &config.dst.dram,
				sizeof(config.dst.dram));
		break;
	default:
		return -EINVAL;
	}

	return paintbox_dma_setup_transfer(pb, session, &v2_config);
}

int setup_dma_transfer_ioctl_v2(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_transfer_config_v2 __user *user_config;
	struct dma_transfer_config_v2 config;

	user_config = (struct dma_transfer_config_v2 __user *)arg;
	if (copy_from_user(&config, user_config, sizeof(config)))
		return -EFAULT;

	return paintbox_dma_setup_transfer(pb, session, &config);
}

int read_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_transfer_read __user *user_req;
	struct dma_transfer_read req;
	struct paintbox_dma_channel *channel;
	struct paintbox_dma_transfer *transfer;
	unsigned long irq_flags;
	int ret;

	user_req = (struct dma_transfer_read __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	channel = get_dma_channel(pb, session, req.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	if (!access_ok(VERIFY_READ, req.host_vaddr, req.len_bytes)) {
		mutex_unlock(&pb->lock);
		return -EFAULT;
	}

	dev_dbg(pb->dev, "dma channel%u read transfer\n", channel->channel_id);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	if (list_empty(&channel->completed_list)) {
		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
		mutex_unlock(&pb->lock);
		return -ENOENT;
	}

	channel->completed_count--;
	WARN_ON(channel->completed_count < 0);

	transfer = list_entry(channel->completed_list.next,
			struct paintbox_dma_transfer, entry);
	list_del(&transfer->entry);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	ret = ipu_dma_release_and_copy_buffer(pb, transfer, req.host_vaddr,
			req.len_bytes);

	paintbox_free_transfer(pb, transfer);

	mutex_unlock(&pb->lock);

	return ret;
}

int stop_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dma_stop_transfer(pb, channel);

	mutex_unlock(&pb->lock);

	return 0;
}

int start_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_transfer *transfer;
	struct paintbox_dma_channel *channel;
	unsigned long irq_flags;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(pb->dev, "dma channel%u start\n", channel_id);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	if (list_empty(&channel->pending_list)) {
		dev_err(pb->dev, "dma channel%u no pending transfers\n",
				channel->channel_id);
		ret = -ENOENT;
		goto err_exit;
	}

	if (channel->active_count >= MAX_ACTIVE_TRANSFERS) {
		dev_err(pb->dev, "dma channel%u too many transfers\n",
				channel->channel_id);
		ret = -EAGAIN;
		goto err_exit;
	}

	transfer = list_entry(channel->pending_list.next,
			struct paintbox_dma_transfer, entry);
	list_del(&transfer->entry);
	channel->pending_count--;
	WARN_ON(channel->pending_count < 0);

	paintbox_pm_enable_dma_channel(pb, channel);

	commit_transfer_to_hardware(pb, channel, transfer);

err_exit:
	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return ret;
}

/* Returns the number of DMA transfers that have been completed and are ready
 * to be read out by the HAL.
 */
int get_completed_transfer_count_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	unsigned long irq_flags;
	int ret;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	ret = channel->completed_count;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);

	dev_dbg(pb->dev, "dma channel%u get completed transfer count %d\n",
			channel_id, ret);

	return ret;
}

/* The caller to this function must hold pb->dma.dma_lock */
static void dma_report_channel_error_locked(struct paintbox_data *pb,
		unsigned int channel_id, int err)
{
	struct paintbox_dma_channel *channel;
	struct paintbox_dma_transfer *transfer;
	uint32_t mode;

	channel = &pb->dma.channels[channel_id];

	paintbox_dma_select_channel(pb, channel->channel_id);

	/* Disable the channel to prevent any pending transfers from starting.
	 */
	mode = paintbox_readl(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_MODE);
	if (mode & DMA_CHAN_MODE_CHAN_ENA_MASK) {
		mode &= ~DMA_CHAN_MODE_CHAN_ENA_MASK;
		paintbox_writel(pb->dev, mode, IPU_CSR_DMA_GRP_OFFSET +
				DMA_CHAN_MODE);
	}

	/* If there is an active stop request on this channel then cancel it. */
	if (channel->stop_request) {
		channel->stop_request = false;
		complete(&channel->stop_completion);
	}

	/* If there is no active transfer then there isn't much more we can do.
	 */
	if (!channel->active_count)
		return;

	/* If there is an active request then then complete it and report the
	 * error to the client.
	 */
	channel->active_count--;
	WARN_ON(channel->active_count < 0);

	transfer = list_entry(channel->active_list.next,
			struct paintbox_dma_transfer, entry);
	if (!WARN_ON(transfer == NULL)) {
		list_del(&transfer->entry);
		dma_report_completion(pb, channel, transfer,
				ktime_get_boottime(), err);
	}
}

/* This function must be called in an interrupt context */
void dma_report_channel_error(struct paintbox_data *pb,
		unsigned int channel_id, int err)
{
	spin_lock(&pb->dma.dma_lock);

	dma_report_channel_error_locked(pb, channel_id, err);

	spin_unlock(&pb->dma.dma_lock);
}

/* This function must be called in an interrupt context */
void dma_report_error_all_channels(struct paintbox_data *pb, int err)
{
	unsigned int channel_id;

	spin_lock(&pb->dma.dma_lock);

	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++)
		dma_report_channel_error_locked(pb, channel_id, err);

	spin_unlock(&pb->dma.dma_lock);
}

/* The caller to this function must hold pb->dma.dma_lock */
static void paintbox_dma_reset_locked(struct paintbox_data *pb)
{
	unsigned int channel_id;

	paintbox_writel(pb->dev, DMA_CTRL_DMA_RESET_MASK,
			IPU_CSR_DMA_TOP_OFFSET + DMA_CTRL);

	/* TODO(ahampson):  There should be no need to hold the DMA reset
	 * register high for a minimum period but the FPGA will lockup if the
	 * reset register is cleared immediately following a VA Error Interrupt.
	 * This needs to be evaluated on the real hardware.  b/35779292
	 *
	 */
	udelay(DMA_RESET_HOLD_PERIOD);
	paintbox_writel(pb->dev, 0, IPU_CSR_DMA_TOP_OFFSET + DMA_CTRL);

	/* Notify all channels that there has been a DMA block reset */
	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++)
		dma_report_channel_error_locked(pb, channel_id, -EIO);
}

#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
int dma_test_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned long irq_flags;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	paintbox_dma_reset_locked(pb);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int dma_test_channel_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret >= 0)
		dma_reset_channel(pb, channel);

	mutex_unlock(&pb->lock);

	return ret;
}
#endif

/* This function must be called in an interrupt context */
void dma_report_mipi_output_completed(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	spin_lock(&pb->dma.dma_lock);

	if (channel->active_count == 0)
		paintbox_pm_disable_dma_channel(pb, channel);

	spin_unlock(&pb->dma.dma_lock);
}

/* This function must be called in an interrupt context and the caller must
 * hold pb->dma.dma_lock.
 */
void paintbox_dma_disable_gather_channels(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_dma_channel *channel, *channel_next;

	/* Gather DMA channels can only be powered down when both DMA channels
	 * in the gather have completed.  The driver does not have information
	 * about the relationship between DMA channels so the driver tracks
	 * all gather channels within the session.  Once all gather transfers
	 * for the session have completed the gather DMA channels can be
	 * powered down.
	 */
	list_for_each_entry_safe(channel, channel_next, &session->dma_list,
			session_entry) {
		if (!(session->dma_gather_channel_mask &
				(1 << channel->channel_id)))
			continue;

		/* If this channel has other active transfers then skip it. */
		if (channel->active_count > 0)
			continue;

		paintbox_pm_disable_dma_channel(pb, channel);
	}

	session->dma_gather_channel_mask = 0;
}

/* This function must be called in an interrupt context and the caller must
 * hold pb->dma.dma_lock.
 */
void paintbox_dma_eof_interrupt(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer, ktime_t timestamp)
{
	struct paintbox_dma_transfer *next_transfer;
	int err = transfer->error;

	channel->stats.eof_interrupts++;

	if (channel->stats.time_stats_enabled)
		channel->stats.last_transfer_time_us = ktime_to_us(ktime_sub(
				timestamp, transfer->start_time));

	/* If there was a stop request then clear it. */
	if (channel->stop_request) {
		channel->stop_request = false;
		complete(&channel->stop_completion);
		/* In the case of a MIPI error and a cancel error then the
		 * cancel will take precedence.
		 */
		err = -ECANCELED;
	}

	dev_dbg(pb->dev, "dma channel%u EOF err %d\n", channel->channel_id,
			err);

	dma_report_completion(pb, channel, transfer, timestamp, err);

	/* If there are no pending transfers or the channel has been stopped
	 * then return now.
	 */
	if (list_empty(&channel->pending_list) || err == -ECANCELED)
		return;

	/* There should be space in the active queue if not then there is a
	 * bug.
	 */
	if (WARN_ON(channel->active_count >= MAX_ACTIVE_TRANSFERS))
		return;

	next_transfer = list_entry(channel->pending_list.next,
			struct paintbox_dma_transfer, entry);

	if (next_transfer->auto_start_transfer) {
		list_del(&next_transfer->entry);
		channel->pending_count--;
		WARN_ON(channel->pending_count < 0);

		commit_transfer_to_hardware(pb, channel, next_transfer);
	}
}

/* This function must be called in an interrupt context and the caller must
 * hold pb->dma.dma_lock.
 */
void paintbox_dma_va_interrupt(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer, ktime_t timestamp)
{
	channel->stats.va_interrupts++;

	/* If there was a stop request then clear it.  The VA error takes
	 * precedence over the stop request.
	 */
	if (channel->stop_request) {
		channel->stop_request = false;
		complete(&channel->stop_completion);
	}

	dev_dbg(pb->dev, "dma channel%u VA ERR %d\n", channel->channel_id,
			-EIO);

	if (!transfer) {
		dev_warn(pb->dev, "%s: channel%u no active transfer\n",
				__func__, channel->channel_id);
		return;
	}

	/* Reporting non-recoverable to all channel since DMA VA error cannot
	 * be recovered. This will trigger a full IPU reset.
	 */
	dma_report_error_all_channels(pb, -ENOTRECOVERABLE);

	paintbox_dma_reset_locked(pb);
}

static struct paintbox_dma_transfer *paintbox_dma_interrupt_common(
		struct paintbox_data *pb, struct paintbox_dma_channel *channel)
{
	struct paintbox_dma_transfer *transfer;

	transfer = paintbox_dequeue_active_transfer(pb, channel);
	if (!transfer)
		return NULL;

	if (paintbox_dma_is_gather(transfer->chan_mode))
		channel->session->dma_gather_transfer_count--;

	return transfer;
}

static void paintbox_dma_pm_disable_if_idle(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	/* If there is an active transfer then the channel is not idle and can
	 * not be powered down.
	 */
	if (channel->active_count > 0)
		return;

	/* If the last transfer was a MIPI out transfer then the channel will
	 * not be idle until the MIPI EOF interrupt.  The MIPI code will notify
	 * the DMA code when the MIPI EOF has occurred.
	 */
	if (paintbox_dma_dst_is_mipi(transfer->chan_mode))
		return;

	/* If this is a gather channel then verify that all gather transfers for
	 * the session have completed.  Gather DMA transfers are performed using
	 * DMA channel pairs.  The DMA channels involved can not be powered down
	 * until both DMA transfers have completed.
	 *
	 * The driver does not know the relationship between the DMA channels so
	 * the driver tracks all gather transfers and DMA channels on a per
	 * session basis.  Gather channels will be powered down when all gather
	 * transfers for the session have completed.
	 */
	if (paintbox_dma_is_gather_channel(channel)) {
		if (channel->session->dma_gather_transfer_count == 0)
			paintbox_dma_disable_gather_channels(pb,
					channel->session);
		return;
	}

	/* The DMA channel is idle, power it down. */
	paintbox_pm_disable_dma_channel(pb, channel);
}

/* This function must be called in an interrupt context and pb->dma.dma_lock
 * must be held.
 */
static void paintbox_dma_process_ms_err_and_eof_interrupts(
		struct paintbox_data *pb, uint32_t status, ktime_t timestamp,
		bool process_ms_err)
{
	unsigned int channel_id;

	for (channel_id = 0; channel_id < pb->dma.num_channels && status;
			channel_id++, status >>= 1) {
		struct paintbox_dma_transfer *transfer;
		struct paintbox_dma_channel *channel;

		if (!(status & 0x01))
			continue;

		channel = &pb->dma.channels[channel_id];

		transfer = paintbox_dma_interrupt_common(pb, channel);
		if (!transfer) {
			/* If there is no active transfer associated with this
			 * interrupt then mark the channel as idle and move on
			 * to the next one.
			 */
			paintbox_pm_disable_dma_channel(pb, channel);
			continue;
		}

		if (process_ms_err) {
			/* In the case of a MIPI overflow error we set the
			 * transfer error and then use the EOF handler to
			 * finalize the transfer.
			 */
			transfer->error = -EIO;
		}

		paintbox_dma_eof_interrupt(pb, channel, transfer, timestamp);

		/* Determine if the channel is idle and power it down if
		 * possible.
		 */
		paintbox_dma_pm_disable_if_idle(pb, channel, transfer);
	}
}

/* This function must be called in an interrupt context and pb->dma.dma_lock
 * must be held.
 */
static void paintbox_dma_process_va_err_interrupts(struct paintbox_data *pb,
		uint32_t status, ktime_t timestamp)
{
	unsigned int channel_id;

	for (channel_id = 0; channel_id < pb->dma.num_channels && status;
			channel_id++, status >>= 1) {
		struct paintbox_dma_transfer *transfer;
		struct paintbox_dma_channel *channel;

		if (!(status & 0x01))
			continue;

		channel = &pb->dma.channels[channel_id];

		transfer = paintbox_dma_interrupt_common(pb, channel);
		if (!transfer) {
			/* If there is no active transfer associated with this
			 * interrupt then mark the channel as idle and move on
			 * to the next one.
			 */
			paintbox_pm_disable_dma_channel(pb, channel);
			continue;
		}

		paintbox_dma_va_interrupt(pb, channel, transfer, timestamp);

		/* Determine if the channel is idle and power it down if
		 * possible.
		 */
		paintbox_dma_pm_disable_if_idle(pb, channel, transfer);
	}
}

irqreturn_t paintbox_dma_channel_interrupt(struct paintbox_data *pb,
		ktime_t timestamp)
{
	uint32_t status;

	status = paintbox_readl(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
			DMA_IRQ_ISR);
	if (status) {
		paintbox_writel(pb->dev, status, IPU_CSR_DMA_TOP_OFFSET +
				DMA_IRQ_ISR);
		paintbox_dma_process_ms_err_and_eof_interrupts(pb, status,
				timestamp, false /* process_ms_err */);
	}

	status = paintbox_readl(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
			DMA_IRQ_ISR_OVF);
	if (status) {
		paintbox_writel(pb->dev, status, IPU_CSR_DMA_TOP_OFFSET +
				DMA_IRQ_ISR_OVF);
		paintbox_dma_process_ms_err_and_eof_interrupts(pb, status,
				timestamp, false /* process_ms_err */);
	}

	return IRQ_HANDLED;
}

irqreturn_t paintbox_dma_channel_error_interrupt(struct paintbox_data *pb,
		ktime_t timestamp)
{
	uint32_t status;

	status = paintbox_readl(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
			DMA_IRQ_VA_ERR_ISR);
	if (status) {
		paintbox_writel(pb->dev, status, IPU_CSR_DMA_TOP_OFFSET +
				DMA_IRQ_VA_ERR_ISR);

		if (status & DMA_IRQ_VA_ERR_ISR_VA_ERR_MASK)
			paintbox_dma_process_va_err_interrupts(pb, status &
					DMA_IRQ_VA_ERR_ISR_VA_ERR_MASK,
					timestamp);
	}

	status = paintbox_readl(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
			DMA_IRQ_VA_ERR_ISR_OVF);
	if (status) {
		paintbox_writel(pb->dev, status, IPU_CSR_DMA_TOP_OFFSET +
				DMA_IRQ_VA_ERR_ISR_OVF);

		if (status & DMA_IRQ_VA_ERR_ISR_VA_ERR_MASK)
			paintbox_dma_process_va_err_interrupts(pb, status &
					DMA_IRQ_VA_ERR_ISR_VA_ERR_MASK,
					timestamp);
	}

	return IRQ_HANDLED;
}

/* Outside of init the caller to this function must hold pb->dma.dma_lock */
static void paintbox_dma_channel_register_init(
		struct paintbox_dma_channel *channel)
{
	channel->regs.chan_img_format = DMA_CHAN_IMG_FORMAT_DEF;
	channel->regs.chan_img_size = DMA_CHAN_IMG_SIZE_DEF;
	channel->regs.chan_img_pos = DMA_CHAN_IMG_POS_DEF;
	channel->regs.chan_img_layout = DMA_CHAN_IMG_LAYOUT_DEF;
	channel->regs.chan_bif_xfer = DMA_CHAN_BIF_XFER_DEF;
	channel->regs.chan_va = DMA_CHAN_VA_DEF;
	channel->regs.chan_va_bdry = DMA_CHAN_VA_BDRY_DEF;
	channel->regs.chan_noc_xfer = DMA_CHAN_NOC_XFER_DEF;
	channel->regs.chan_ssp_config = DMA_CHAN_SSP_CFG_DEF;
	channel->regs.chan_node = DMA_CHAN_NODE_DEF;
}

/* The caller to this function must hold pb->lock */
void paintbox_dma_release(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_dma_channel *channel, *channel_next;

	/* Disable any dma channels associated with the channel */
	list_for_each_entry_safe(channel, channel_next, &session->dma_list,
			session_entry) {
		unbind_dma_interrupt(pb, session, channel);
		release_dma_channel(pb, session, channel);
	}
}

/* All sessions must be released before remove can be called. */
void paintbox_dma_remove(struct paintbox_data *pb)
{
	unsigned int channel_id;

	flush_work(&pb->dma.discard_queue_work);

	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++) {
		struct paintbox_dma_channel *channel =
				&pb->dma.channels[channel_id];
		paintbox_dma_channel_debug_remove(pb, channel);
	}

	kfree(pb->dma.channels);

	paintbox_dma_debug_remove(pb);
}

/* Resets shadows and state and cancels active and pending DMAs.
 * pb->lock is held, so no stops are pending.
 */
void paintbox_dma_post_ipu_reset(struct paintbox_data *pb)
{
	unsigned int channel_id;

	dev_info(pb->dev, "%s\n", __func__);

	pb->dma.bif_outstanding = ((DMA_CHAN_BIF_XFER_DEF &
			DMA_CHAN_BIF_XFER_OUTSTANDING_MASK) >>
			DMA_CHAN_BIF_XFER_OUTSTANDING_SHIFT) + 1;

	pb->dma.regs.irq_imr = DMA_IRQ_IMR_DEF;
	pb->dma.regs.irq_va_err_imr = DMA_IRQ_VA_ERR_IMR_DEF;
	pb->dma.selected_dma_channel_id = DMA_SEL_DEF;

	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++) {
		struct paintbox_dma_channel *channel =
				&pb->dma.channels[channel_id];

		channel->pm_enabled = false;
		channel->interrupts_enabled = false;
		paintbox_dma_channel_register_init(channel);

		/* TODO(ahampson):  Need to cancel any inprogress transfers. */
	}
}

int paintbox_dma_init(struct paintbox_data *pb)
{
	unsigned int channel_id;

	pb->dma.bif_outstanding = ((DMA_CHAN_BIF_XFER_DEF &
			DMA_CHAN_BIF_XFER_OUTSTANDING_MASK) >>
			DMA_CHAN_BIF_XFER_OUTSTANDING_SHIFT) + 1;

	pb->dma.selected_dma_channel_id = DMA_SEL_DEF;
	pb->dma.regs.irq_imr = DMA_IRQ_IMR_DEF;
	pb->dma.regs.irq_va_err_imr = DMA_IRQ_VA_ERR_IMR_DEF;

	spin_lock_init(&pb->dma.dma_lock);
	INIT_LIST_HEAD(&pb->dma.discard_list);
	INIT_LIST_HEAD(&pb->dma.free_list);

	paintbox_dma_debug_init(pb);

	pb->dma.channels = kcalloc(pb->dma.num_channels,
			sizeof(struct paintbox_dma_channel), GFP_KERNEL);
	if (!pb->dma.channels)
		return -ENOMEM;

	/* Store channel id with object as a convenience to avoid doing a
	 * lookup later on.
	 */
	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++) {
		struct paintbox_dma_channel *channel =
				&pb->dma.channels[channel_id];
		channel->channel_id = channel_id;

		paintbox_dma_channel_register_init(channel);

		INIT_LIST_HEAD(&channel->pending_list);
		INIT_LIST_HEAD(&channel->active_list);
		INIT_LIST_HEAD(&channel->completed_list);
		init_completion(&channel->stop_completion);
		paintbox_dma_channel_debug_init(pb, channel);
	}

	INIT_WORK(&pb->dma.discard_queue_work, paintbox_dma_discard_queue_work);

	dev_dbg(pb->dev,
			"dma: top base 0x%08x len %lu grp base 0x%08x len %lu dma channels %u\n",
			IPU_CSR_DMA_TOP_OFFSET, DMA_TOP_BLOCK_LEN,
			IPU_CSR_DMA_GRP_OFFSET, DMA_GRP_BLOCK_LEN,
			pb->dma.num_channels);

	return 0;
}
