/*
 * JQS message transport for the Paintbox programmable IPU
 *
 * Copyright (C) 2018 Google, Inc.
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

#include <linux/err.h>
#include <linux/ipu-core.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sizes.h>

#include "ipu-core-internal.h"
#include "ipu-core-jqs.h"
#include "ipu-core-jqs-cbuf.h"
#include "ipu-core-jqs-msg-transport.h"
#include "ipu-core-jqs-structs.h"
#include "ipu-regs.h"

#define USER_READ_TIMEOUT_MS (300 * 1000)

/*
 * The following pieces of data (as well as a couple of shared
 * registers) are used to facilite host <-> JQS communication.
 *
 * 1 jqs_sys_buffer
 * 2 sys_jqs_buffer
 * 3 jqs_sys_buffer backing data
 * 4 sys_jqs_buffer backing data
 *
 * The host can address these data structures at addresses (from
 * jqs_message_transport_shared_state)
 * 1 shared_state->jqs_sys_buffer
 * 2 shared_state->sys_jqs_buffer
 * 3 host_jqs_sys_buffer->host_data
 * 4 host_sys_jqs_buffer->host_data
 *
 * and JQS can address them at
 * 1 shared_state +
 *	offsetof(struct jqs_message_transport_shared_buffer, jqs_sys_buffer)
 * 2 shared_state +
 *	offsetof(struct jqs_message_transport_shared_buffer, sys_jqs_buffer)
 * 3 shared_state->jqs_sys_buffer.data
 * 4 shared_state->sys_jqs_buffer.data
 *
 * Synchronizing access:
 *
 * On some implementations, the data will be directly mapped
 * to the device, and all memory accesses will be immediately visible
 * to the other side. However, in some cases this is not the case. For that,
 * it is required to explicitly sync shared state.
 *
 * The circular buffers are accessed in a single-reader, single-writer
 * fashion. So lots of care needs to be taken when syncing these
 * data structures.
 *
 * Specifically,
 * 1 Data structures are initialized by the host.
 * 2 For sys_jqs_buffer,
 *	a host writes to data, and bytes_written
 *	b JQS writes to bytes_read
 * 3 For jqs_sys_buffer,
 *	a JQS writes to data, and bytes_written
 *	b host writes to bytes_read
 *
 * As an example, when trying to write to sys_jqs_buffer, we first
 * (possibly) sync sys_jqs_buffer->bytes_read by reading from device to
 * determine how much space is available for writing.  Then we write
 * into the local copy of this data. Finally, we sync the data
 * structures again by writing the newly written data first, and then
 * writing the bytes_written address to make those writes visible to
 * JQS.
 *
 * Cache coherence:
 *
 * In addition to synchronizing access, we need to make sure that the effects
 * of the JQS data cache do not interfere with this protocol. Specifically,
 * we cannot have write access by both the AP and JQS on a single (64-byte)
 * cache line.
 *
 * The specific memory layout of the shared state between the AP and JQS is
 *
 * jqs_message_transport_shared_state
 * JQS_CACHE_LINE_SIZE
 * JQS_SYS_BUFFER_SIZE    <-- backing data for jqs_sys circular buffer
 * JQS_CACHE_LINE_SIZE
 * SYS_JQS_BUFFER_SIZE    <-- backing data for sys_jqs circular buffer
 *
 * Data alignment:
 *
 */

#define JQS_SYS_BUFFER_SIZE		SZ_8K
#define SYS_JQS_BUFFER_SIZE		SZ_128K

static struct paintbox_jqs_msg_transport *ipu_core_get_jqs_transport(
		struct paintbox_bus *bus)
{
	if (!bus->jqs_msg_transport || !ipu_core_jqs_is_ready(bus))
		return ERR_PTR(-ENETDOWN);

	return bus->jqs_msg_transport;
}

/* Must be called with bus->transport_lock held. */
static inline struct host_jqs_queue *ipu_core_get_kernel_queue(
		struct paintbox_jqs_msg_transport *trans)
{
	return &trans->queues[JQS_TRANSPORT_KERNEL_QUEUE_ID];
}

/* Must be called with bus->transport_lock held. */
static inline struct host_jqs_queue_waiter *ipu_core_get_kernel_waiter(
		struct paintbox_jqs_msg_transport *trans)
{
	return &trans->queues[JQS_TRANSPORT_KERNEL_QUEUE_ID].waiter;
}

/* Must be called with bus->transport_lock held. */
static void ipu_core_jqs_signal_queue(struct paintbox_bus *bus, uint32_t q_id)
{
	uint32_t dbl;

	/* Notify the JQS that data has been written into the queue.
	 * Note that it is possible that the JQS could do a read clear of the
	 * doorbell register after the AP has read it.  This will result in the
	 * JQS rechecking the queue which should be beign.
	 */
	dbl = ipu_core_readl(bus, IPU_CSR_JQS_OFFSET + SYS_JQS_DBL);
	dbl |= (1 << q_id);
	ipu_core_writel(bus, dbl, IPU_CSR_JQS_OFFSET + SYS_JQS_DBL);
}

/* Called in a threaded interrupt context, bus->transport_lock must be held. */
static void ipu_core_jqs_msg_process_ack_message(struct paintbox_bus *bus,
		struct paintbox_jqs_msg_transport *trans,
		struct jqs_message *jqs_msg)
{
	struct host_jqs_queue_waiter *waiter =
			ipu_core_get_kernel_waiter(trans);
	int ret = 0;

	if (!waiter->enabled) {
		/* No one is waiting for this response */
		dev_err(bus->parent_dev, "%s: unexpected response", __func__);
		return;
	}

	if (waiter->size >= jqs_msg->size) {
		memcpy(waiter->buf, jqs_msg, jqs_msg->size);
		ret = jqs_msg->size;
	} else {
		/* Not enough room, the entire response must be stored */
		ret = -EINVAL;
		dev_err(bus->parent_dev, "%s: response too large", __func__);
	}

	waiter->ret = ret;
	complete(&waiter->completion);
}

static void ipu_core_jqs_msg_process_log_message(struct paintbox_bus *bus,
		struct jqs_message *jqs_msg)
{
	struct jqs_message_log *log_msg = (struct jqs_message_log *)jqs_msg;
	char buf[MAX_LOG_SIZE];

	scnprintf(buf, MAX_LOG_SIZE, "JQS: %s", log_msg->data);

	switch (log_msg->log_level) {
	case JQS_LOG_LEVEL_FATAL:
	case JQS_LOG_LEVEL_ERROR:
		dev_err(bus->parent_dev, "%s\n", buf);
		break;
	case JQS_LOG_LEVEL_WARNING:
		dev_warn(bus->parent_dev, "%s\n", buf);
		break;
	case JQS_LOG_LEVEL_INFO:
		dev_info(bus->parent_dev, "%s\n", buf);
		break;
	case JQS_LOG_LEVEL_NONE:
		break;
	};
}

static void ipu_core_jqs_msg_process_error_message(struct paintbox_bus *bus,
		struct jqs_message *jqs_msg)
{
	struct jqs_message_error *err_msg = (struct jqs_message_error *)jqs_msg;

	if (err_msg->error == JQS_ERROR_ASSERTION) {
		dev_err(bus->parent_dev, "JQS: assert at %s:%d\n",
				err_msg->data.assertion.file,
				err_msg->data.assertion.line);
		/* TODO(b/114760293): JQS assertion should trigger catastrophic
		 * error here.
		 */
	} else {
		dev_err(bus->parent_dev, "JQS: error %d reported\n",
				err_msg->error);
		/* TODO(b/116061358): proper error should be triggered. */
	}
}

/* Called in a threaded interrupt context, bus->transport_lock must be held. */
static void ipu_core_jqs_msg_process_kernel_message(struct paintbox_bus *bus,
		struct paintbox_jqs_msg_transport *trans,
		struct jqs_message *jqs_msg)
{
	switch (jqs_msg->type) {
	/* These are the response types -- there should be a waiter on the
	 * kernel queue
	 */
	case JQS_MESSAGE_TYPE_ACK:
		ipu_core_jqs_msg_process_ack_message(bus, trans, jqs_msg);
		break;
	case JQS_MESSAGE_TYPE_LOG:
		ipu_core_jqs_msg_process_log_message(bus, jqs_msg);
		break;
	case JQS_MESSAGE_TYPE_ERROR:
		ipu_core_jqs_msg_process_error_message(bus, jqs_msg);
		break;
	case JQS_MESSAGE_TYPE_IPU_REG_VALUES:
		/* same logic as ack message */
		ipu_core_jqs_msg_process_ack_message(bus, trans, jqs_msg);
		break;
	default:
		dev_err(bus->parent_dev,
			"JQS: error unexpected response message type was received %d\n",
				jqs_msg->type);
		break;
	}
}

#define MIN_MSG_BUFFERED 2
#define MSG_BUFFER_SIZE (JQS_TRANSPORT_MAX_MESSAGE_SIZE * MIN_MSG_BUFFERED)

/* Called in a threaded interrupt context, bus->transport_lock must be held. */
static void ipu_core_jqs_msg_transport_process_kernel_queue(
		struct paintbox_bus *bus,
		struct paintbox_jqs_msg_transport *trans)
{
	struct host_jqs_queue *host_q = ipu_core_get_kernel_queue(trans);
	struct host_jqs_cbuf *host_cbuf = &host_q->host_jqs_sys_cbuf;
	uint8_t buf_msg[MSG_BUFFER_SIZE];
	uint32_t bytes_read;
	uint32_t bytes_buffered = 0;
	struct jqs_message *jqs_msg = (struct jqs_message *)buf_msg;
	const size_t hdr_size = sizeof(struct jqs_message);

	ipu_core_jqs_cbuf_sync(bus, host_cbuf, DMA_FROM_DEVICE);

	for (;;) {
		bytes_read = ipu_core_jqs_cbuf_read(bus, host_cbuf,
				&buf_msg[bytes_buffered],
				sizeof(buf_msg) - bytes_buffered,
				ipu_core_jqs_cbus_memcpy);

		bytes_buffered += bytes_read;

		/* If there are no bytes buffered for processing then exit. */
		if (bytes_buffered == 0)
			return;

		if (unlikely(bytes_buffered < hdr_size ||
				bytes_buffered < jqs_msg->size)) {
			/* TODO(b/114760293): Fatal error. Partial message
			 * reads are not supported on the kernel queue.
			 */
			dev_err(bus->parent_dev, "%s: partial read", __func__);
			return;
		}

		while (bytes_buffered != 0) {
			/* If there is a partial message left in buffer then
			 * return to outer loop to read the rest.
			 */
			if (bytes_buffered < hdr_size ||
					bytes_buffered < jqs_msg->size)
				break;

			ipu_core_jqs_msg_process_kernel_message(bus, trans,
					jqs_msg);
			/* Shift remaining bytes following the processed
			 * message to the beginning of the message buffer.
			 */
			bytes_buffered -= jqs_msg->size;
			memcpy(buf_msg, &buf_msg[jqs_msg->size],
					bytes_buffered);
		}
	}
}


/* Must be called with bus->transport_lock held. */
static int ipu_core_jqs_msg_transport_setup_queue(struct paintbox_bus *bus,
		struct paintbox_jqs_msg_transport *trans, uint32_t q_id)
{
	size_t size = JQS_SYS_BUFFER_SIZE + JQS_CACHE_LINE_SIZE +
			SYS_JQS_BUFFER_SIZE;
	struct host_jqs_queue *host_q = &trans->queues[q_id];
	int data_offset = 0;
	int cbuf_offset;

	cbuf_offset = offsetof(struct jqs_msg_transport_shared_state, queues) +
			q_id * sizeof(trans->jqs_shared_state->queues[0]);


	host_q->shared_buf_data = ipu_core_alloc_shared_buffer(bus, size);

	if (IS_ERR_OR_NULL(host_q->shared_buf_data))
		return PTR_ERR(host_q->shared_buf_data);

	memset(&host_q->waiter, 0, sizeof(struct host_jqs_queue_waiter));

	ipu_core_jqs_cbuf_init(&host_q->host_jqs_sys_cbuf,
			trans->shared_buf,
			cbuf_offset + offsetof(struct jqs_msg_transport_queue,
			jqs_sys_cbuf),
			host_q->shared_buf_data, data_offset,
			JQS_SYS_BUFFER_SIZE, false);

	data_offset += JQS_SYS_BUFFER_SIZE + JQS_CACHE_LINE_SIZE;

	ipu_core_jqs_cbuf_init(&host_q->host_sys_jqs_cbuf,
			trans->shared_buf,
			cbuf_offset + offsetof(struct jqs_msg_transport_queue,
			sys_jqs_cbuf),
			host_q->shared_buf_data, data_offset,
			SYS_JQS_BUFFER_SIZE, true);

	/* Write initial state to JQS */
	ipu_core_sync_shared_memory(bus, trans->shared_buf, cbuf_offset,
			sizeof(struct jqs_msg_transport_queue),
			DMA_TO_DEVICE);

	return 0;
}

/* API exposed to the rest of the kernel driver */
int ipu_core_jqs_msg_transport_init(struct paintbox_bus *bus)
{
	struct paintbox_jqs_msg_transport *trans;
	int ret;

	trans = kzalloc(sizeof(*trans), GFP_KERNEL);
	if (!trans)
		return -ENOMEM;

	trans->shared_buf = ipu_core_alloc_shared_buffer(bus,
			sizeof(struct jqs_msg_transport_shared_state));

	if (IS_ERR_OR_NULL(trans->shared_buf)) {
		ret = PTR_ERR(trans->shared_buf);
		goto free_local_dram;
	}

	trans->jqs_shared_state = (struct jqs_msg_transport_shared_state *)
		trans->shared_buf->host_vaddr;
	WARN_ON(bus->jqs_msg_transport);
	bus->jqs_msg_transport = trans;

	/* The free_queue_bits field in the transport structure is set up to
	 * match the bits in the JQS_SYS_DBL register.  Bits 0 and 1 in the
	 * JQS_SYS_DBL are used to trigger the two MSI interrupts assigned to
	 * the JQS and are excluded from the free queue number space.  MSI0 will
	 * be used to indicate data on the kernel queue and MSI1 will be used to
	 * indicate data on one of the application queues.
	 */
	trans->free_queue_ids = ~(JQS_SYS_DBL_MSI0 | JQS_SYS_DBL_MSI1);

	return 0;

free_local_dram:
	kfree(trans);

	return ret;
}

void ipu_core_jqs_msg_transport_shutdown(struct paintbox_bus *bus)
{
	struct paintbox_jqs_msg_transport *trans;

	mutex_lock(&bus->transport_lock);

	if (!bus->jqs_msg_transport) {
		mutex_unlock(&bus->transport_lock);
		return;
	}
	trans = bus->jqs_msg_transport;

	ipu_core_free_shared_memory(bus, trans->shared_buf);

	kfree(bus->jqs_msg_transport);

	bus->jqs_msg_transport = NULL;

	mutex_unlock(&bus->transport_lock);
}

/* Called in a threaded interrupt context, bus->transport_lock must be held. */
static void ipu_core_jqs_msg_transport_process_app_queue(
		struct paintbox_bus *bus,
		struct paintbox_jqs_msg_transport *trans, uint32_t q_id)
{
	struct host_jqs_queue *host_q = &trans->queues[q_id];

	/* Make sure data is accessible to the AP*I */
	ipu_core_jqs_cbuf_sync(bus, &host_q->host_jqs_sys_cbuf,
			DMA_FROM_DEVICE);

	if (host_q->waiter.enabled) {
		host_q->waiter.ret = 0;
		complete(&host_q->waiter.completion);
	}
}

/* Called in a threaded interrupt context */
irqreturn_t ipu_core_jqs_msg_transport_interrupt(struct paintbox_bus *bus)
{
	struct paintbox_jqs_msg_transport *trans;
	uint32_t q_ids;

	q_ids = ipu_core_readl(bus, IPU_CSR_JQS_OFFSET + JQS_SYS_DBL);
#if IS_ENABLED(CONFIG_IPU_DEBUG)
	bus->jqs.shadow_reg_jqs_sys_dbl = q_ids;
#endif
	if (!q_ids)
		return IRQ_NONE;

	/* Remove JQS_SYS_DBL_MSI1 from the q_ids mask.  This bit is used to
	 * signal data available in one or more of the application queues.
	 */
	q_ids &= ~JQS_SYS_DBL_MSI1;

	mutex_lock(&bus->transport_lock);

	trans = ipu_core_get_jqs_transport(bus);
	if (IS_ERR(trans)) {
		mutex_unlock(&bus->transport_lock);
		return IRQ_HANDLED;
	}

	while (q_ids) {
		uint32_t q_id;

		/* Pick out the next q_id to process */
		q_id = __builtin_ctz(q_ids);
		q_ids &= ~(1 << q_id);

		if (q_id == JQS_TRANSPORT_KERNEL_QUEUE_ID)
			ipu_core_jqs_msg_transport_process_kernel_queue(bus,
					trans);
		else if (!(trans->free_queue_ids & (1 << q_id)))
			ipu_core_jqs_msg_transport_process_app_queue(bus,
					trans, q_id);
		else {
			dev_err(bus->parent_dev,
				"%s: JQS received an interrupt for a free queue\n",
				__func__);
			ipu_bus_notify_fatal_error(bus);
			break;
		}
	}

	mutex_unlock(&bus->transport_lock);

	return IRQ_HANDLED;
}

int ipu_core_jqs_msg_transport_alloc_queue(struct paintbox_bus *bus)
{
	struct paintbox_jqs_msg_transport *trans;
	uint32_t q_id;
	int ret;

	mutex_lock(&bus->transport_lock);

	trans = ipu_core_get_jqs_transport(bus);
	if (IS_ERR(trans)) {
		mutex_unlock(&bus->transport_lock);
		dev_err(bus->parent_dev, "%s: JQS is not ready\n", __func__);
		return PTR_ERR(trans);
	}

	if (trans->free_queue_ids == 0) {
		mutex_unlock(&bus->transport_lock);
		return -ENOMEM;
	}

	q_id = __builtin_ctz(trans->free_queue_ids);
	trans->free_queue_ids &= ~(1 << q_id);

	ret = ipu_core_jqs_msg_transport_setup_queue(bus, trans, q_id);
	if (ret < 0) {
		mutex_unlock(&bus->transport_lock);
		return ret;
	}

	mutex_unlock(&bus->transport_lock);

	return q_id;
}

int ipu_core_jqs_msg_transport_alloc_kernel_queue(struct paintbox_bus *bus)
{
	int ret;

	mutex_lock(&bus->transport_lock);

	if (!bus->jqs_msg_transport) {
		mutex_unlock(&bus->transport_lock);
		dev_err(bus->parent_dev, "%s: JQS is not ready\n", __func__);
		return -ENETDOWN;
	}

	ret = ipu_core_jqs_msg_transport_setup_queue(bus,
			bus->jqs_msg_transport, JQS_TRANSPORT_KERNEL_QUEUE_ID);

	mutex_unlock(&bus->transport_lock);

	return ret;
}

void ipu_core_jqs_msg_transport_free_queue(struct paintbox_bus *bus,
		uint32_t q_id, int queue_err)
{
	struct paintbox_jqs_msg_transport *trans;
	struct host_jqs_queue *host_q;
	struct host_jqs_queue_waiter *waiter;

	mutex_lock(&bus->transport_lock);

	if (!bus->jqs_msg_transport) {
		mutex_unlock(&bus->transport_lock);
		return;
	}

	trans = bus->jqs_msg_transport;

	host_q = &trans->queues[q_id];
	waiter = &host_q->waiter;

	if (waiter->enabled) {
		/* Release the waiting thread, the queue is disappearing */
		waiter->ret = queue_err;
		complete(&waiter->completion);
	}

	ipu_core_free_shared_memory(bus, host_q->shared_buf_data);

	trans->free_queue_ids |= (1 << q_id);

	mutex_unlock(&bus->transport_lock);
}

void ipu_core_jqs_msg_transport_free_kernel_queue(struct paintbox_bus *bus,
		int queue_err)
{
	ipu_core_jqs_msg_transport_free_queue(bus,
			JQS_TRANSPORT_KERNEL_QUEUE_ID, queue_err);
}

void ipu_core_jqs_msg_transport_complete_queue(struct paintbox_bus *bus,
		uint32_t q_id, int queue_err)
{
	struct paintbox_jqs_msg_transport *trans;
	struct host_jqs_queue *host_q;
	struct host_jqs_queue_waiter *waiter;

	mutex_lock(&bus->transport_lock);

	if (!bus->jqs_msg_transport) {
		mutex_unlock(&bus->transport_lock);
		return;
	}

	trans = bus->jqs_msg_transport;

	host_q = &trans->queues[q_id];
	waiter = &host_q->waiter;

	if (waiter->enabled) {
		/* Release the waiting thread, the queue is disappearing */
		waiter->ret = queue_err;
		complete(&waiter->completion);
	}

	mutex_unlock(&bus->transport_lock);
}

void ipu_core_jqs_msg_transport_complete_kernel_queue(struct paintbox_bus *bus,
		int queue_err)
{
	ipu_core_jqs_msg_transport_complete_queue(bus,
			JQS_TRANSPORT_KERNEL_QUEUE_ID, queue_err);
}

ssize_t ipu_core_jqs_msg_transport_user_read(struct paintbox_bus *bus,
		uint32_t q_id, void __user *buf, size_t size)
{
	struct paintbox_jqs_msg_transport *trans;
	struct host_jqs_queue *host_q;
	unsigned long timeout_ms = USER_READ_TIMEOUT_MS;
	struct host_jqs_queue_waiter *waiter;
	ssize_t ret;

	mutex_lock(&bus->transport_lock);

	trans = ipu_core_get_jqs_transport(bus);
	if (IS_ERR(trans)) {
		mutex_unlock(&bus->transport_lock);
		return PTR_ERR(trans);
	}

	host_q = &trans->queues[q_id];
	waiter = &host_q->waiter;

	/* If the queue was freed before the trans->lock was acquired for the
	 * read then return a disconnect error.
	 */
	if (trans->free_queue_ids & (1 << q_id)) {
		mutex_unlock(&bus->transport_lock);
		return -ECONNABORTED;
	}

	/* Two concurrent calls to read is not supported */
	if (waiter->enabled) {
		mutex_unlock(&bus->transport_lock);
		dev_err(bus->parent_dev, "%s: queue busy", __func__);
		return -EBUSY;
	}

	memset(waiter, 0, sizeof(struct host_jqs_queue_waiter));
	waiter->enabled = true;
	init_completion(&waiter->completion);

	while (true) {
		long time_remaining;

		ret = ipu_core_jqs_cbuf_read(bus, &host_q->host_jqs_sys_cbuf,
				buf, size, ipu_core_jqs_cbus_copy_to_user);
		if (ret > 0) {
			break;
		} else if (ret < 0) {
			dev_err(bus->parent_dev,
					"%s: error reading from JQS circular buffer, ret %zd\n",
					__func__, ret);
			break;
		}

		mutex_unlock(&bus->transport_lock);

		time_remaining = wait_for_completion_interruptible_timeout(
				&waiter->completion,
				msecs_to_jiffies(timeout_ms));

		mutex_lock(&bus->transport_lock);

		trans = ipu_core_get_jqs_transport(bus);
		if (IS_ERR(trans)) {
			mutex_unlock(&bus->transport_lock);
			dev_err(bus->parent_dev, "%s: JQS is not ready\n",
					__func__);
			return PTR_ERR(trans);
		}

		host_q = &trans->queues[q_id];
		waiter = &host_q->waiter;

		/* An error reported by the queue takes precedence over other
		 * errors.
		 */
		if (waiter->ret != 0) {
			ret = waiter->ret;
			break;
		}

		if (time_remaining < 0) {
			ret = time_remaining; /* -ERESTARTSYS */
			break;
		} else if (time_remaining == 0) {
			dev_err(bus->parent_dev, "%s: wait timeout\n",
						__func__);
			ret = -ETIMEDOUT;
			break;
		}

		timeout_ms = jiffies_to_msecs(time_remaining);
		reinit_completion(&waiter->completion);
	}

	memset(waiter, 0, sizeof(struct host_jqs_queue_waiter));

	mutex_unlock(&bus->transport_lock);

	return ret;
}

ssize_t ipu_core_jqs_msg_transport_user_write(struct paintbox_bus *bus,
		uint32_t q_id, const void __user *buf, size_t size)
{
	struct paintbox_jqs_msg_transport *trans;
	struct host_jqs_queue *host_q;
	ssize_t bytes_written;

	mutex_lock(&bus->transport_lock);

	trans = ipu_core_get_jqs_transport(bus);
	if (IS_ERR(trans)) {
		mutex_unlock(&bus->transport_lock);
		dev_err(bus->parent_dev, "%s: JQS is not ready\n", __func__);
		return PTR_ERR(trans);
	}

	host_q = &trans->queues[q_id];

	bytes_written = ipu_core_jqs_cbuf_write(bus, &host_q->host_sys_jqs_cbuf,
			buf, size, ipu_core_jqs_cbus_copy_from_user);

	ipu_core_jqs_signal_queue(bus, q_id);

	mutex_unlock(&bus->transport_lock);

	return bytes_written;
}

/* Must be called with bus->transport_lock held. */
static ssize_t ipu_core_jqs_msg_transport_kernel_write_locked(
	struct paintbox_bus *bus, struct paintbox_jqs_msg_transport *trans,
	const struct jqs_message *msg)
{
	struct host_jqs_queue *host_q = ipu_core_get_kernel_queue(trans);
	ssize_t bytes_written;

	bytes_written = ipu_core_jqs_cbuf_write(bus, &host_q->host_sys_jqs_cbuf,
			msg, msg->size, ipu_core_jqs_cbus_memcpy);
	if (bytes_written != msg->size) {
		dev_err(bus->parent_dev,
				"%s: message size mismatch, expected %u, got %u\n",
				__func__, msg->size, bytes_written);
		return -EFAULT;
	}

	ipu_core_jqs_signal_queue(bus, JQS_TRANSPORT_KERNEL_QUEUE_ID);

	return bytes_written;
}

ssize_t ipu_core_jqs_msg_transport_kernel_write(struct paintbox_bus *bus,
	const struct jqs_message *msg)
{
	struct paintbox_jqs_msg_transport *trans;
	ssize_t bytes_written;

	mutex_lock(&bus->transport_lock);

	trans = ipu_core_get_jqs_transport(bus);
	if (IS_ERR(trans)) {
		mutex_unlock(&bus->transport_lock);
		return PTR_ERR(trans);
	}

	bytes_written = ipu_core_jqs_msg_transport_kernel_write_locked(bus,
			trans, msg);

	mutex_unlock(&bus->transport_lock);

	return bytes_written;
}

/* TODO(b/116817730):  This timeout should be revaluated.  Once JQS watchdog and
 * PCIe link status change handling is in place then the need for a timeout may
 * go away.
 */
#define IPU_JQS_KERNEL_QUEUE_WRITE_SYNC_TIMEOUT_MS 10000

ssize_t ipu_core_jqs_msg_transport_kernel_write_sync(struct paintbox_bus *bus,
		const struct jqs_message *msg, struct jqs_message *rsp,
		size_t rsp_size)
{
	struct paintbox_jqs_msg_transport *trans;
	struct host_jqs_queue_waiter *waiter;
	unsigned long timeout;
	ssize_t ret;

	timeout = msecs_to_jiffies(IPU_JQS_KERNEL_QUEUE_WRITE_SYNC_TIMEOUT_MS);

	mutex_lock(&bus->transport_lock);

	trans = ipu_core_get_jqs_transport(bus);
	if (IS_ERR(trans)) {
		mutex_unlock(&bus->transport_lock);
		return PTR_ERR(trans);
	}

	waiter = ipu_core_get_kernel_waiter(trans);

	/* Two concurrent calls to write sync are not supported */
	if (waiter->enabled) {
		mutex_unlock(&bus->transport_lock);
		dev_err(bus->parent_dev, "%s: write in progress\n", __func__);
		return -EBUSY;
	}

	memset(waiter, 0, sizeof(struct host_jqs_queue_waiter));
	waiter->enabled = true;
	waiter->buf = rsp;
	waiter->size = rsp_size;
	init_completion(&waiter->completion);

	ret = ipu_core_jqs_msg_transport_kernel_write_locked(bus, trans, msg);
	if (ret < 0) {
		memset(waiter, 0, sizeof(struct host_jqs_queue_waiter));
		mutex_unlock(&bus->transport_lock);
		return ret;
	}

	mutex_unlock(&bus->transport_lock);

	/* Wait for the response */
	timeout = wait_for_completion_timeout(&waiter->completion, timeout);

	/* A message error has higher priority than a timeout error. */
	if (waiter->ret)
		ret = waiter->ret;
	else if (timeout == 0)
		ret = -ETIMEDOUT;

	mutex_lock(&bus->transport_lock);

	trans = ipu_core_get_jqs_transport(bus);
	if (IS_ERR(trans)) {
		mutex_unlock(&bus->transport_lock);
		return PTR_ERR(trans);
	}

	waiter = ipu_core_get_kernel_waiter(trans);
	memset(waiter, 0, sizeof(struct host_jqs_queue_waiter));

	mutex_unlock(&bus->transport_lock);

	return ret;
}
