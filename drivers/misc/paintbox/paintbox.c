/*
 * Core driver for the Paintbox programmable IPU
 *
 * Copyright (C) 2015 Google, Inc.
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
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <uapi/paintbox.h>
#include <uapi/paintbox_v2.h>

#include "paintbox-bif.h"
#include "paintbox-bus.h"
#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-lbp.h"
#include "paintbox-mmu.h"
#include "paintbox-pmon.h"
#include "paintbox-power.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"
#include "paintbox-stp-pc-histogram.h"
#include "paintbox-stp-sram.h"

typedef int (*resource_allocator_t)(struct paintbox_data *,
		struct paintbox_session *, unsigned int);

static int paintbox_open(struct inode *ip, struct file *fp)
{
	struct paintbox_session *session;
	struct paintbox_data *pb;
	struct miscdevice *m = fp->private_data;
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time;
#endif

	pb = container_of(m, struct paintbox_data, misc_device);
#ifdef CONFIG_PAINTBOX_DEBUG
	start_time = pb->stats.ioctl_time_enabled ? ktime_get_boottime() :
			ktime_set(0, 0);
#endif

	session = kzalloc(sizeof(struct paintbox_session), GFP_KERNEL);
	if (!session)
		return -ENOMEM;

	session->dev = pb;
	INIT_LIST_HEAD(&session->irq_list);
	INIT_LIST_HEAD(&session->dma_list);
	INIT_LIST_HEAD(&session->stp_list);
	INIT_LIST_HEAD(&session->lbp_list);
	INIT_LIST_HEAD(&session->wait_list);

	init_completion(&session->bulk_alloc_completion);
	init_completion(&session->release_completion);

	fp->private_data = session;

	mutex_lock(&pb->lock);

	pb->session_count++;

	mutex_unlock(&pb->lock);

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb, PB_STATS_OPEN,
				start_time, ktime_get_boottime(), 0);
#endif
	return 0;
}

/* The caller to this function must hold pb lock */
void signal_completion_on_first_alloc_waiter(struct paintbox_data *pb)
{
	struct paintbox_session *first_entry;

	if (list_empty(&pb->bulk_alloc_waiting_list))
		return;

	first_entry = list_first_entry(&pb->bulk_alloc_waiting_list,
			struct paintbox_session, alloc_wait_list_entry);

	complete(&first_entry->bulk_alloc_completion);
}

/* The caller to this function must hold pb lock */
static void remove_session_from_alloc_wait_list(
		struct paintbox_session *session)
{
	if (session->waiting_alloc) {
		list_del(&session->alloc_wait_list_entry);
		session->waiting_alloc = false;
	}

	/* Signal completion on first entry to avoid
	 * starvation.
	 */
	signal_completion_on_first_alloc_waiter(session->dev);
}

static int paintbox_release(struct inode *ip, struct file *fp)
{
	struct paintbox_session *session = fp->private_data;
	struct paintbox_data *pb = session->dev;
	struct paintbox_irq *irq, *irq_next;
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time;

	start_time = pb->stats.ioctl_time_enabled ? ktime_get_boottime() :
			ktime_set(0, 0);
#endif

	mutex_lock(&pb->lock);

	/* TODO(ahampson): Cleanup release sequence.  b/62372748 */
	paintbox_dma_release(pb, session);
	paintbox_stp_release(pb, session);

	/* Disable any interrupts associated with the session */
	list_for_each_entry_safe(irq, irq_next, &session->irq_list,
			session_entry)
		release_interrupt(pb, session, irq);

	paintbox_lbp_release(pb, session);

	paintbox_irq_wait_for_release_complete(pb, session);

	remove_session_from_alloc_wait_list(session);

	/* free any pmon allocations */
	if (pb->bif.pmon_session == session)
		pb->bif.pmon_session = NULL;

	if (pb->mmu.pmon_session == session)
		pb->mmu.pmon_session = NULL;

	if (pb->dma.pmon_session == session)
		pb->dma.pmon_session = NULL;

	if (WARN_ON(--pb->session_count < 0))
		pb->session_count = 0;

	mutex_unlock(&pb->lock);

	kfree(session);

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb, PB_STATS_CLOSE,
				start_time, ktime_get_boottime(), 0);
#endif

	return 0;
}

static long paintbox_get_caps_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct paintbox_pdata *pdata = pb->dev->platform_data;
	struct ipu_capabilities caps;
	uint32_t version;

	memset(&caps, 0, sizeof(caps));

	caps.num_lbps = pb->lbp.num_lbps;
	caps.num_stps = pb->stp.num_stps;
	caps.num_dma_channels = pb->dma.num_channels;
	caps.num_interrupts = pb->io.num_interrupts;

	version = paintbox_readl(pb->dev, IPU_VERSION);
	caps.version_major = (version & IPU_VERSION_MAJOR_MASK) >>
			IPU_VERSION_MAJOR_SHIFT;
	caps.version_minor = (version & IPU_VERSION_MINOR_MASK) >>
			IPU_VERSION_MAJOR_SHIFT;
	caps.version_build = version & IPU_VERSION_INCR_MASK;
	caps.is_fpga = !!(version & IPU_VERSION_FPGA_BUILD_MASK);

#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	caps.is_simulator = true;
#endif

#ifdef CONFIG_PAINTBOX_IOMMU
	caps.iommu_enabled = iommu_present(&paintbox_bus_type);
#endif

	caps.hardware_id = pdata->hardware_id;

	if (copy_to_user((void __user *)arg, &caps, sizeof(caps)))
		return -EFAULT;

	return 0;
}

/* The caller to this function must hold pb->lock */
static void paintbox_ipu_bulk_release_resources_internal
	(struct paintbox_data *pb, struct paintbox_session *session)
{
	struct paintbox_irq *irq, *irq_next;

	paintbox_stp_release(pb, session);
	paintbox_lbp_release(pb, session);
	paintbox_dma_release(pb, session);
	/* Disable any interrupts associated with the session */
	list_for_each_entry_safe(irq, irq_next, &session->irq_list,
			session_entry)
		release_interrupt(pb, session, irq);
	paintbox_irq_wait_for_release_complete(pb, session);
}

static int paintbox_ipu_bulk_release_resources_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	mutex_lock(&pb->lock);
	paintbox_ipu_bulk_release_resources_internal(pb, session);
	signal_completion_on_first_alloc_waiter(pb);
	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock */
static int allocate_requested_resources_internal(struct paintbox_data *pb,
		struct paintbox_session *session, uint64_t req_mask,
		resource_allocator_t allocator)
{
	int ret;
	int index;
	uint64_t iterator;

	iterator = req_mask;
	index = 0;

	for (index = 0; iterator > 0; iterator >>= 1, index++) {
		if (!(iterator & 0x1))
			continue;

		ret = allocator(pb, session, index);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* The caller to this function must hold pb->lock
 * The caller need to release resource on failure (non-zero return)
 */
static int allocate_requested_resources(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct ipu_bulk_allocation_request req)
{
	int ret;

	ret = allocate_requested_resources_internal(pb, session, req.stp_mask,
			&allocate_stp);
	if (ret)
		return ret;

	ret = allocate_requested_resources_internal(pb, session, req.lbp_mask,
			&allocate_lbp);
	if (ret)
		return ret;

	ret = allocate_requested_resources_internal(pb, session,
			req.dma_channel_mask, &allocate_dma_channel);
	if (ret)
		return ret;

	ret = allocate_requested_resources_internal(pb, session,
			req.interrupt_mask, &allocate_interrupt);
	if (ret)
		return ret;

	return 0;
}

static int validate_request_resource_mask(struct paintbox_data *pb,
		struct ipu_bulk_allocation_request req)
{
	if (req.stp_mask >> pb->stp.num_stps) {
		dev_warn(pb->dev, "%s: STP request invalid\n",
				__func__);
		return -EINVAL;
	}

	if (req.lbp_mask >> pb->lbp.num_lbps) {
		dev_warn(pb->dev, "%s: LBP request invalid\n",
				__func__);
		return -EINVAL;
	}

	if (req.dma_channel_mask >> pb->dma.num_channels) {
		dev_warn(pb->dev, "%s: DMA request invalid\n",
				__func__);
		return -EINVAL;
	}

	if (req.interrupt_mask >> pb->io.num_interrupts) {
		dev_warn(pb->dev, "%s: IRQ request invalid\n",
				__func__);
		return -EINVAL;
	}

	return 0;
}

/* The caller to this function must hold pb lock */
static bool check_requested_resource_availability(struct paintbox_data *pb,
		struct ipu_bulk_allocation_request req)
{
	uint64_t busy_mask;
	bool is_available = true;

	busy_mask = req.stp_mask & ~pb->stp.available_stp_mask;
	if (busy_mask) {
		dev_warn(pb->dev,
				"%s: Some or all requested STPs are busy, busy mask: 0x%016llx\n",
				__func__, busy_mask);
		is_available = false;
	}

	busy_mask = req.lbp_mask & ~pb->lbp.available_lbp_mask;
	if (busy_mask) {
		dev_warn(pb->dev,
				"%s: Some or all requested LBPs are busy, busy mask: 0x%016llx\n",
				__func__, busy_mask);
		is_available = false;
	}

	busy_mask = req.dma_channel_mask & ~pb->dma.available_channel_mask;
	if (busy_mask) {
		dev_warn(pb->dev,
				"%s: Some or all requested DMA Channels are busy, busy mask: 0x%016llx\n",
				__func__, busy_mask);
		is_available = false;
	}

	busy_mask = req.interrupt_mask & ~pb->io.available_interrupt_mask;
	if (busy_mask) {
		dev_warn(pb->dev,
				"%s: Some or all requested Interrupts are busy, busy mask: 0x%016llx\n",
				__func__, busy_mask);
		is_available = false;
	}

	return is_available;
}

static int paintbox_ipu_bulk_allocate_resources_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret;
	struct ipu_bulk_allocation_request __user *user_req;
	struct ipu_bulk_allocation_request req;
	long time_remaining = LONG_MAX;
	uint64_t timeout_remaining_ns;
	bool available = false;

	user_req = (struct ipu_bulk_allocation_request __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	ret = validate_request_resource_mask(pb, req);
	if (ret < 0)
		return ret;

	timeout_remaining_ns = req.timeout_ns;

	mutex_lock(&pb->lock);
	do {
		struct paintbox_session *first_entry;

		/* Adding session to the end of waiting list */
		if (!session->waiting_alloc) {
			list_add_tail(&session->alloc_wait_list_entry,
					&pb->bulk_alloc_waiting_list);
			session->waiting_alloc = true;
		}

		first_entry = list_first_entry(&pb->bulk_alloc_waiting_list,
				struct paintbox_session, alloc_wait_list_entry);

		if (first_entry == session) {
			available = check_requested_resource_availability(pb,
					req);

			/* If requested resources are available, skip to
			 * allocation
			 */
			if (available) {
				remove_session_from_alloc_wait_list(session);
				break;
			}
		}

		reinit_completion(&session->bulk_alloc_completion);
		mutex_unlock(&pb->lock);

		if (timeout_remaining_ns == LONG_MAX) {
			ret = wait_for_completion_interruptible
				(&session->bulk_alloc_completion);
			if (ret) {
				mutex_lock(&pb->lock);
				goto err_exit;
			}
			time_remaining = LONG_MAX;
		} else {
			time_remaining =
				wait_for_completion_interruptible_timeout
				(&session->bulk_alloc_completion,
				 nsecs_to_jiffies64(timeout_remaining_ns));

			if (time_remaining > 0)
				timeout_remaining_ns =
					jiffies_to_nsecs(time_remaining);
		}
		mutex_lock(&pb->lock);
	} while (time_remaining > 0);

	if (time_remaining == 0 && !available) {
		ret = -ETIMEDOUT;
		goto err_exit;
	} else if (time_remaining < 0) {
		ret = time_remaining;
		goto err_exit;
	}

	ret = allocate_requested_resources(pb, session, req);
	if (ret < 0) {
		dev_err(pb->dev, "%s: bulk resource allocation failed\n",
				__func__);

		/* Release allocated resources */
		paintbox_ipu_bulk_release_resources_internal(pb, session);
	}

	mutex_unlock(&pb->lock);

	return ret;

err_exit:
	remove_session_from_alloc_wait_list(session);
	mutex_unlock(&pb->lock);
	return ret;
}

static long paintbox_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	struct paintbox_session *session = fp->private_data;
	struct paintbox_data *pb = session->dev;
	int ret;
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time;

	if (pb->stats.ioctl_time_enabled)
		start_time = ktime_get_boottime();
#endif

	switch (cmd) {
	case PB_GET_IPU_CAPABILITIES:
		ret = paintbox_get_caps_ioctl(pb, session, arg);
		break;
	case PB_ALLOCATE_INTERRUPT:
		ret = allocate_interrupt_ioctl(pb, session, arg);
		break;
	case PB_WAIT_FOR_INTERRUPT:
		ret = wait_for_interrupt_ioctl(pb, session, arg);
		break;
	case PB_RELEASE_INTERRUPT:
		ret = release_interrupt_ioctl(pb, session, arg);
		break;
	case PB_FLUSH_INTERRUPTS:
		ret = paintbox_flush_interrupt_ioctl(pb, session, arg);
		break;
	case PB_FLUSH_ALL_INTERRUPTS:
		ret = paintbox_flush_all_interrupts_ioctl(pb, session, arg);
		break;
	case PB_ALLOCATE_DMA_CHANNEL:
		ret = allocate_dma_channel_ioctl(pb, session, arg);
		break;
	case PB_BIND_DMA_INTERRUPT:
		ret = bind_dma_interrupt_ioctl(pb, session, arg);
		break;
	case PB_UNBIND_DMA_INTERRUPT:
		ret = unbind_dma_interrupt_ioctl(pb, session, arg);
		break;
	case PB_START_DMA_TRANSFER:
		ret = start_dma_transfer_ioctl(pb, session, arg);
		break;
	case PB_STOP_DMA_TRANSFER:
		ret = stop_dma_transfer_ioctl(pb, session, arg);
		break;
	case PB_RELEASE_DMA_CHANNEL:
		ret = release_dma_channel_ioctl(pb, session, arg);
		break;
	case PB_GET_COMPLETED_UNREAD_COUNT:
		ret = get_completed_transfer_count_ioctl(pb, session, arg);
		break;
	case PB_FLUSH_DMA_TRANSFERS:
		ret = flush_dma_transfers_ioctl(pb, session, arg);
		break;
	case PB_ALLOCATE_LINE_BUFFER_POOL:
		ret = allocate_lbp_ioctl(pb, session, arg);
		break;
	case PB_SETUP_LINE_BUFFER:
		ret = paintbox_setup_lb_ioctl_v0(pb, session, arg);
		break;
	case PB_SETUP_LINE_BUFFER_V2:
		ret = paintbox_setup_lb_ioctl_v2(pb, session, arg);
		break;
	case PB_RELEASE_LINE_BUFFER_POOL:
		ret = release_lbp_ioctl(pb, session, arg);
		break;
	case PB_RESET_LINE_BUFFER_POOL:
		ret = reset_lbp_ioctl(pb, session, arg);
		break;
	case PB_RESET_LINE_BUFFER:
		ret = reset_lb_ioctl(pb, session, arg);
		break;
	case PB_WRITE_LBP_MEMORY:
		ret = write_lbp_memory_ioctl(pb, session, arg);
		break;
	case PB_READ_LBP_MEMORY:
		ret = read_lbp_memory_ioctl(pb, session, arg);
		break;
	case PB_ALLOCATE_PROCESSOR:
		ret = allocate_stp_ioctl(pb, session, arg);
		break;
	case PB_SETUP_PROCESSOR:
		ret = setup_stp_ioctl(pb, session, arg);
		break;
	case PB_WRITE_STP_MEMORY:
		ret = write_stp_scalar_sram_ioctl(pb, session, arg);
		break;
	case PB_READ_STP_MEMORY:
		ret = read_stp_scalar_sram_ioctl(pb, session, arg);
		break;
	case PB_WRITE_VECTOR_SRAM_COORDINATES:
		ret = write_stp_vector_sram_coordinates_ioctl(pb, session, arg);
		break;
	case PB_WRITE_VECTOR_SRAM_REPLICATE:
		ret = write_stp_vector_sram_replicate_ioctl(pb, session, arg);
		break;
	case PB_READ_VECTOR_SRAM_COORDINATES:
		ret = read_stp_vector_sram_coordinates_ioctl(pb, session, arg);
		break;
	case PB_START_PROCESSOR:
		ret = start_stp_ioctl(pb, session, arg);
		break;
	case PB_STOP_PROCESSOR:
		ret = stop_stp_ioctl(pb, session, arg);
		break;
	case PB_RESUME_PROCESSOR:
		ret = resume_stp_ioctl(pb, session, arg);
		break;
	case PB_RESET_PROCESSOR:
		ret = reset_stp_ioctl(pb, session, arg);
		break;
	case PB_RESET_ALL_PROCESSORS:
		ret = paintbox_reset_all_stp_ioctl(pb, session);
		break;
	case PB_GET_PROGRAM_STATE:
		ret = get_program_state_ioctl(pb, session, arg);
		break;
	case PB_GET_ALL_PROCESSOR_STATES:
		ret = paintbox_get_all_processor_states(pb, session, arg);
		break;
	case PB_RELEASE_PROCESSOR:
		ret = release_stp_ioctl(pb, session, arg);
		break;
	case PB_WAIT_FOR_ALL_PROCESSOR_IDLE:
		/* The simulator requires additional processing after the DMA
		 * interrupt before the processor goes idle.  Starting with the
		 * V2 register model this idle wait is handled completely with
		 * in the Simulator.
		 *
		 * This processing is fast enough on the actual hardware that we
		 * do not need to poll for idle.
		 */
		ret = 0;
		break;
	case PB_GET_PROCESSOR_IDLE:
		/* Idle processing is handled completely within the Simulator in
		 * the V2 register model.  The actual hardware does not have an
		 * idle check.
		 */
		ret = -EINVAL;
		break;
	case PB_BIND_STP_INTERRUPT:
		ret = bind_stp_interrupt_ioctl(pb, session, arg);
		break;
	case PB_UNBIND_STP_INTERRUPT:
		ret = unbind_stp_interrupt_ioctl(pb, session, arg);
		break;
	case PB_STP_PC_HISTOGRAM_CLEAR:
		ret = stp_pc_histogram_clear_ioctl(pb, session, arg);
		break;
	case PB_STP_PC_HISTOGRAM_ENABLE:
		ret = stp_pc_histogram_enable_ioctl(pb, session, arg);
		break;
	case PB_STP_PC_HISTOGRAM_READ:
		ret = stp_pc_histogram_read_ioctl(pb, session, arg);
		break;
	case PB_SETUP_DMA_TRANSFER:
		ret = setup_dma_transfer_ioctl(pb, session, arg);
		break;
	case PB_SETUP_DMA_TRANSFER_V2:
		ret = setup_dma_transfer_ioctl_v2(pb, session, arg);
		break;
	case PB_READ_DMA_TRANSFER:
		ret = read_dma_transfer_ioctl(pb, session, arg);
		break;
	case PB_BULK_ALLOCATE_IPU_RESOURCES:
		ret = paintbox_ipu_bulk_allocate_resources_ioctl(pb, session,
				arg);
		break;
	case PB_BULK_RELEASE_IPU_RESOURCES:
		ret = paintbox_ipu_bulk_release_resources_ioctl(pb, session,
				arg);
		break;
	case PB_PMON_ALLOCATE:
		ret = pmon_allocate_ioctl(pb, session, arg);
		break;
	case PB_PMON_RELEASE:
		ret = pmon_release_ioctl(pb, session, arg);
		break;
	case PB_PMON_CONFIG_WRITE:
		ret = pmon_config_write_ioctl(pb, session, arg);
		break;
	case PB_PMON_DATA_READ:
		ret = pmon_data_read_ioctl(pb, session, arg);
		break;
	case PB_PMON_DATA_WRITE:
		ret = pmon_data_write_ioctl(pb, session, arg);
		break;
	case PB_PMON_ENABLE:
		ret = pmon_enable_ioctl(pb, session, arg);
		break;
#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
	case PB_TEST_DMA_RESET:
		ret = dma_test_reset_ioctl(pb, session, arg);
		break;
	case PB_TEST_DMA_CHANNEL_RESET:
		ret = dma_test_channel_reset_ioctl(pb, session, arg);
		break;
	case PB_TEST_LBP_BROADCAST_WRITE_MEMORY:
		ret = lbp_test_broadcast_write_memory_ioctl(pb, session, arg);
		break;
#else
	case PB_TEST_DMA_RESET:
	case PB_TEST_DMA_CHANNEL_RESET:
	case PB_TEST_MIPI_IN_RESET_STREAM:
	case PB_TEST_MIPI_OUT_RESET_STREAM:
	case PB_TEST_LBP_BROADCAST_WRITE_MEMORY:
		ret = -EINVAL;
		break;
#endif
	default:
		dev_err(pb->dev, "%s: unknown ioctl 0x%0x\n", __func__, cmd);
		return -EINVAL;
	}

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_ioctl_stats(pb, cmd, start_time,
				ktime_get_boottime());
#endif

	return ret;
}

static const struct file_operations paintbox_fops = {
	.owner = THIS_MODULE,
	.open = paintbox_open,
	.release = paintbox_release,
	.unlocked_ioctl = paintbox_ioctl,
};

static int paintbox_get_capabilities(struct paintbox_data *pb)
{
	struct paintbox_pdata *pdata = pb->dev->platform_data;
	uint8_t major, minor, build;
	uint32_t val;
	bool is_fpga;

	val = paintbox_readl(pb->dev, IPU_VERSION);
	major = (val & IPU_VERSION_MAJOR_MASK) >> IPU_VERSION_MAJOR_SHIFT;
	minor = (val & IPU_VERSION_MINOR_MASK) >> IPU_VERSION_MAJOR_SHIFT;
	build = val & IPU_VERSION_INCR_MASK;
	is_fpga = !!(val & IPU_VERSION_FPGA_BUILD_MASK);

	val = paintbox_readl(pb->dev, IPU_CAP);
	pb->stp.num_stps = val & IPU_CAP_NUM_STP_MASK;
	pb->stp.available_stp_mask = (1ULL << pb->stp.num_stps) - 1;
	pb->lbp.num_lbps = (val & IPU_CAP_NUM_LBP_MASK) >>
		IPU_CAP_NUM_LBP_SHIFT;
	pb->lbp.available_lbp_mask = (1ULL << pb->lbp.num_lbps) - 1;
	pb->dma.num_channels = (val & IPU_CAP_NUM_DMA_CHAN_MASK) >>
			IPU_CAP_NUM_DMA_CHAN_SHIFT;
	pb->dma.available_channel_mask = (1ULL << pb->dma.num_channels) - 1;

#if defined(CONFIG_PAINTBOX_SIMULATOR_SUPPORT)
	dev_dbg(pb->dev,
			"Paintbox IPU Version %u.%u.%u Simulator Hardware ID %u\n",
			major, minor, build, pdata->hardware_id);
#else
	dev_dbg(pb->dev, "Paintbox IPU Version %u.%u.%u %s Hardware ID %u\n",
			major, minor, build, is_fpga ? "FPGA" : "",
			pdata->hardware_id);
#endif
	dev_dbg(pb->dev,
			"STPs %u LBPs %u DMA Channels %u\n",
			pb->stp.num_stps, pb->lbp.num_lbps,
			pb->dma.num_channels);
	return 0;
}

static void paintbox_ipu_firmware_down(struct device *dev)
{
	struct paintbox_data *pb = dev_get_drvdata(dev);

	dev_info(pb->dev, "JQS firmware is going down\n");

	/* TODO(ahampson):  This needs to handle a reset notification in the
	 * middle of a job.  Right now this will only work when the device is
	 * quiescent.
	 */
}

static void paintbox_ipu_firmware_up(struct device *dev)
{
	struct paintbox_data *pb = dev_get_drvdata(dev);

	dev_info(pb->dev, "JQS firmware is ready\n");

	/* TODO(ahampson):  This needs to handle a reset notification in the
	 * middle of a job.  Right now this will only work when the device is
	 * quiescent.
	 */

	paintbox_io_apb_post_ipu_reset(pb);
	paintbox_dma_post_ipu_reset(pb);
	paintbox_lbp_post_ipu_reset(pb);
	paintbox_stp_post_ipu_reset(pb);
}

static const struct paintbox_device_ops paintbox_dev_ops = {
	.firmware_up = paintbox_ipu_firmware_up,
	.firmware_down = paintbox_ipu_firmware_down,
};

static int paintbox_ipu_probe(struct device *dev)
{
	struct paintbox_data *pb;
	int ret;

	pb = devm_kzalloc(dev, sizeof(*pb), GFP_KERNEL);
	if (pb == NULL)
		return -ENOMEM;

	pb->dev = dev;

	dev_set_drvdata(dev, pb);

	mutex_init(&pb->lock);

	spin_lock_init(&pb->irq_lock);

	paintbox_set_device_ops(dev, &paintbox_dev_ops);

	paintbox_debug_init(pb);

	ret = paintbox_get_capabilities(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_dma_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_lbp_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_stp_init(pb);
	if (ret < 0)
		return ret;

	/* Initialize the IO APB block after the blocks that can generate
	 * interrupts.  All interrupt sources need to be initialized first.
	 */
	ret = paintbox_io_apb_init(pb);
	if (ret < 0)
		return ret;

	ret = paintbox_pm_init(pb);
	if (ret < 0)
		return ret;

	paintbox_bif_init(pb);

	ret = paintbox_mmu_init(pb);
	if (ret < 0)
		return ret;

	/* Initialize the IRQ waiters after IO APB so the IRQ waiter code knows
	 * how many interrupts to allocate.
	 */
	ret = paintbox_irq_init(pb);
	if (ret < 0)
		return ret;

	/* register the misc device */
	pb->misc_device.minor = MISC_DYNAMIC_MINOR,
	pb->misc_device.name  = "paintbox",
	pb->misc_device.fops  = &paintbox_fops,

	INIT_LIST_HEAD(&pb->bulk_alloc_waiting_list);

	ret = misc_register(&pb->misc_device);
	if (ret) {
		pr_err("Failed to register misc device node (ret = %d)", ret);
		return ret;
	}

	return 0;
}

static int paintbox_ipu_remove(struct device *dev)
{
	struct paintbox_data *pb = dev_get_drvdata(dev);

	misc_deregister(&pb->misc_device);
	paintbox_irq_remove(pb);
	paintbox_mmu_remove(pb);
	paintbox_bif_remove(pb);
	paintbox_lbp_remove(pb);
	paintbox_stp_remove(pb);
	paintbox_pm_remove(pb);
	paintbox_io_apb_remove(pb);
	paintbox_dma_remove(pb);

	paintbox_debug_remove(pb);

	mutex_destroy(&pb->lock);

	return 0;
}

static struct device_driver paintbox_ipu_driver = {
	.name	= "paintbox-ipu",
	.bus	= &paintbox_bus_type,
	.probe	= paintbox_ipu_probe,
	.remove	= paintbox_ipu_remove,
};

int paintbox_ipu_init(void)
{
	return driver_register(&paintbox_ipu_driver);
}

static void __exit paintbox_ipu_exit(void)
{
	driver_unregister(&paintbox_ipu_driver);
}

subsys_initcall(paintbox_ipu_init);
module_exit(paintbox_ipu_exit);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Paintbox IPU Driver");
