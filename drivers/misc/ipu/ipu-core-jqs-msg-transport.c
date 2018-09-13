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
#define SYS_JQS_BUFFER_SIZE		SZ_32K

static uint32_t write_core(struct paintbox_bus *bus, uint32_t q_id,
		const void *buf, size_t size)
{
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;
	struct host_jqs_queue *host_q = &trans->queues[q_id];
	struct host_jqs_cbuf *host_cbuf = &host_q->host_sys_jqs_cbuf;
	uint32_t bytes_written, dbl;
	jqs_cbuf_copy cpy = (q_id == JQS_TRANSPORT_KERNEL_QUEUE_ID) ?
			ipu_core_jqs_cbus_memcpy :
			ipu_core_jqs_cbus_copy_from_user;

	mutex_lock(&trans->lock);
	bytes_written = ipu_core_jqs_cbuf_write(bus, host_cbuf, buf, size, cpy);

	/* Notify the JQS that data has been written into the queue.
	 * Note that it is possible that the JQS could do a read clear of the
	 * doorbell register after the AP has read it.  This will result in the
	 * JQS rechecking the queue which should be beign.
	 */
	dbl = ipu_core_readl(bus, IPU_CSR_JQS_OFFSET + SYS_JQS_DBL);
	dbl |= (1 << q_id);
	ipu_core_writel(bus, dbl, IPU_CSR_JQS_OFFSET + SYS_JQS_DBL);

	mutex_unlock(&trans->lock);

	return bytes_written;
}

/* Must hold trans->lock */
static int init_waiter(struct host_jqs_queue *host_q, void *buf, size_t size,
		struct host_jqs_queue_waiter *waiter)
{
	/* Two concurrent calls to read is not supported */
	if (host_q->waiter)
		return -EBUSY;

	init_completion(&waiter->completion);
	waiter->ret = 0;
	waiter->buf = buf;
	waiter->size = size;

	host_q->waiter = waiter;
	return 0;
}

/* Must hold trans->lock && host_q->waiter != nullptr */
static void signal_waiter(struct host_jqs_queue *host_q, int ret)
{
	struct host_jqs_queue_waiter *waiter = host_q->waiter;

	waiter->ret = ret;
	complete(&waiter->completion);
	host_q->waiter = NULL;
}

/* Must hold trans->lock */
static void remove_waiter(struct host_jqs_queue *host_q,
		struct host_jqs_queue_waiter *waiter)
{
	/* A waiter can be removed in three circumstances.
	 * 1. Data arrives on the queue
	 * 2. A timeout occurs
	 * 3. The queue is deallocated
	 *
	 * Ideally, the only thread that should remove a waiter should be
	 * itself. However, I didn't want to require another synchronization
	 * step on #3, where the queue was deallocated, but there was still a
	 * waiter on it.  Since the host_q are statically allocated, we could
	 * avoid that synchronization, and just leave this 'dangling' waiter to
	 * wake up, realize it had been signaled due to a broken queue, remove
	 * itself and then return, but perhaps even worse than the dangling
	 * waiter, if that queue got reallocated, the waiter could cause a
	 * failure for a read on a newly created queue, perhaps even from a
	 * different session.
	 *
	 * Slightly cleaner, I think, is this approach, where the waiter is
	 * removed from the host_q as soon as it is signalled. In this case,
	 * there are a couple of subtle race conditions where a waiter could
	 * may be trying to remove itself from #2, say, while #1 has also
	 * occurred, and now host_q->waiter is either NULL already, or even
	 * worse, has a new waiter waiting on it. This check should suffice.
	 *
	 * All that being said, I'd certainly be interested in a cleaner
	 * solution to this.
	 */
	if (host_q->waiter == waiter)
		host_q->waiter = NULL;
}

/* Must hold trans->lock and have set up waiter */
static int wait_for_data(struct paintbox_bus *bus,
		struct host_jqs_queue *host_q, int timeout_ms)
{
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	struct host_jqs_queue_waiter *waiter = host_q->waiter;
	int ret;

	mutex_unlock(&trans->lock);
	timeout = wait_for_completion_timeout(&waiter->completion, timeout);

	ret = (timeout == 0) ? -ETIMEDOUT : waiter->ret;
	mutex_lock(&trans->lock);
	remove_waiter(host_q, waiter);

	return ret;
}

static void process_kernel_response(struct paintbox_bus *bus,
		struct jqs_message *jqs_msg)
{
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;
	struct host_jqs_queue *host_q =
			&trans->queues[JQS_TRANSPORT_KERNEL_QUEUE_ID];

	int ret = 0;

	mutex_lock(&trans->lock);

	if (!host_q->waiter) {
		/* No one is waiting for this response */
		dev_err(bus->parent_dev, "%s: unexpected response", __func__);
		goto exit;
	}

	if (host_q->waiter->size < jqs_msg->size) {
		/* Not enough room, the entire response must be stored */
		ret = -EINVAL;
		dev_err(bus->parent_dev, "%s: response too large", __func__);
		goto signal_waiter;
	}

	memcpy(host_q->waiter->buf, jqs_msg, jqs_msg->size);
	ret = jqs_msg->size;

signal_waiter:
	signal_waiter(host_q, ret);

exit:
	mutex_unlock(&trans->lock);
}

static void ipu_core_jqs_msg_process_error_message(struct paintbox_bus *bus,
		struct jqs_message *jqs_msg)
{
	struct jqs_message_error *err_msg = (struct jqs_message_error *)jqs_msg;

	if (err_msg->error == JQS_ERROR_ASSERTION) {
		dev_err(bus->parent_dev, "%s: JQS: assert at %s:%d\n",
				__func__, err_msg->data.assertion.file,
				err_msg->data.assertion.line);
		/* TODO(b/114760293): JQS assertion should trigger catastrophic
		 * error here.
		 */
	} else {
		dev_err(bus->parent_dev, "%s: error %d sent by JQS.\n",
				__func__, err_msg->error);
		/* TODO(b/116061358): proper error should be triggered. */
	}
}

static void process_kernel_message(struct paintbox_bus *bus,
		struct jqs_message *jqs_msg)
{
	switch (jqs_msg->type) {
	/* These are the response types -- there should be a waiter on the
	 * kernel queue
	 */
	case JQS_MESSAGE_TYPE_ACK:
		process_kernel_response(bus, jqs_msg);
		break;
	case JQS_MESSAGE_TYPE_ERROR:
		ipu_core_jqs_msg_process_error_message(bus, jqs_msg);
		break;
	default:
		break;
	}
}

#define MIN_MSG_BUFFERED 2
#define MSG_BUFFER_SIZE (JQS_TRANSPORT_MAX_MESSAGE_SIZE * MIN_MSG_BUFFERED)

static void process_kernel_queue(struct paintbox_bus *bus)
{
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;
	struct host_jqs_queue *host_q =
			&trans->queues[JQS_TRANSPORT_KERNEL_QUEUE_ID];
	struct host_jqs_cbuf *host_cbuf = &host_q->host_jqs_sys_cbuf;
	uint8_t buf_msg[MSG_BUFFER_SIZE];
	uint32_t bytes_read;
	uint32_t bytes_buffered = 0;
	struct jqs_message *jqs_msg = (struct jqs_message *)buf_msg;
	const size_t hdr_size = sizeof(struct jqs_message);

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

			process_kernel_message(bus, jqs_msg);
			/* Shift remaining bytes following the processed
			 * message to the beginning of the message buffer.
			 */
			bytes_buffered -= jqs_msg->size;
			memcpy(buf_msg, &buf_msg[jqs_msg->size],
					bytes_buffered);
		}
	}
}

static void process_queues(struct paintbox_bus *bus, uint32_t q_ids)
{
	uint32_t q_id;
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;
	struct host_jqs_queue *host_q;
	struct host_jqs_cbuf *host_cbuf;

	while (q_ids) {
		/* Pick out the next q_id to process */
		q_id = __builtin_ctz(q_ids);
		q_ids &= ~(1 << q_id);

		host_q = &trans->queues[q_id];
		host_cbuf = &host_q->host_jqs_sys_cbuf;
		/* Make sure data is accessible to the AP*I */
		ipu_core_jqs_cbuf_sync(bus, host_cbuf, DMA_FROM_DEVICE);

		/* Process away */
		if (q_id == JQS_TRANSPORT_KERNEL_QUEUE_ID)
			process_kernel_queue(bus);
		else {
			mutex_lock(&trans->lock);

			if (host_q->waiter) {
				host_q->waiter->ret = 0;
				complete(&host_q->waiter->completion);
			}

			mutex_unlock(&trans->lock);
		}
	}
}

static int setup_queue(struct paintbox_bus *bus, uint32_t q_id)
{
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;
	size_t size = JQS_SYS_BUFFER_SIZE + JQS_CACHE_LINE_SIZE +
			SYS_JQS_BUFFER_SIZE;
	struct host_jqs_queue *host_q = &trans->queues[q_id];
	int data_offset = 0;
	int cbuf_offset;

	cbuf_offset = offsetof(struct jqs_msg_transport_shared_state, queues) +
			q_id * sizeof(trans->jqs_shared_state->queues[0]);

	if (ipu_core_memory_alloc(bus, size, &host_q->shared_buf_data) < 0)
		return -ENOMEM;

	host_q->waiter = NULL;

	ipu_core_jqs_cbuf_init(&host_q->host_jqs_sys_cbuf,
			&trans->shared_buf,
			cbuf_offset + offsetof(struct jqs_msg_transport_queue,
			jqs_sys_cbuf),
			&host_q->shared_buf_data, data_offset,
			JQS_SYS_BUFFER_SIZE, false);

	data_offset += JQS_SYS_BUFFER_SIZE + JQS_CACHE_LINE_SIZE;

	ipu_core_jqs_cbuf_init(&host_q->host_sys_jqs_cbuf,
			&trans->shared_buf,
			cbuf_offset + offsetof(struct jqs_msg_transport_queue,
			sys_jqs_cbuf),
			&host_q->shared_buf_data, data_offset,
			SYS_JQS_BUFFER_SIZE, true);

	/* Write initial state to JQS */
	ipu_core_sync(bus, &trans->shared_buf, cbuf_offset,
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

	ret = ipu_core_memory_alloc(bus, sizeof(*trans), &trans->shared_buf);
	if (ret < 0)
		goto free_local_dram;

	trans->jqs_shared_state = (struct jqs_msg_transport_shared_state *)
		trans->shared_buf.host_vaddr;
	bus->jqs_msg_transport = trans;
	mutex_init(&trans->lock);

	/* Preallocate the kernel queue */
	ret = setup_queue(bus, JQS_TRANSPORT_KERNEL_QUEUE_ID);
	if (ret < 0)
		goto free_remote_dram;

	/* The free_queue_bits field in the transport structure is set up to
	 * match the bits in the JQS_SYS_DBL register.  Bits 0 and 1 in the
	 * JQS_SYS_DBL are used to trigger the two MSI interrupts assigned to
	 * the JQS and are excluded from the free queue number space.  MSI0 will
	 * be used to indicate data on the kernel queue and MSI1 will be used to
	 * indicate data on one of the application queues.
	 */
	trans->free_queue_ids = ~(JQS_SYS_DBL_MSI0 | JQS_SYS_DBL_MSI1);

	return 0;

free_remote_dram:
	ipu_core_memory_free(bus, &trans->shared_buf);
free_local_dram:
	kfree(trans);

	return ret;
}

void ipu_core_jqs_msg_transport_shutdown(struct paintbox_bus *bus)
{
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;

	ipu_core_memory_free(bus, &trans->shared_buf);
	kfree(bus->jqs_msg_transport);

	bus->jqs_msg_transport = NULL;
}

irqreturn_t ipu_core_jqs_msg_transport_interrupt(struct paintbox_bus *bus)
{
	uint32_t q_ids;

	q_ids = ipu_core_readl(bus, IPU_CSR_JQS_OFFSET + JQS_SYS_DBL);
	if (!q_ids)
		return IRQ_NONE;

	/* Remove JQS_SYS_DBL_MSI1 from the q_ids mask.  This bit is used to
	 * signal data available in one or more of the application queues.
	 */
	q_ids &= ~JQS_SYS_DBL_MSI1;

	ipu_core_jqs_msg_transport_process_queues(bus, q_ids);

	return IRQ_HANDLED;
}

int ipu_core_jqs_msg_transport_alloc_queue(struct paintbox_bus *bus)
{
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;
	uint32_t q_id = -1;
	uint32_t ret;

	mutex_lock(&trans->lock);
	if (trans->free_queue_ids != 0) {
		q_id = __builtin_ctz(trans->free_queue_ids);
		trans->free_queue_ids &= ~(1 << q_id);
	}
	mutex_unlock(&trans->lock);

	if (q_id == -1)
		return -ENOMEM;

	ret = setup_queue(bus, q_id);

	if (ret < 0)
		return ret;

	return q_id;
}

void ipu_core_jqs_msg_transport_free_queue(struct paintbox_bus *bus,
		uint32_t q_id)
{
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;
	struct host_jqs_queue *host_q = &trans->queues[q_id];

	mutex_lock(&trans->lock);

	if (host_q->waiter) {
		/* Release the waiting thread, the queue is disappearing */
		host_q->waiter->ret = -ECONNRESET;
		complete(&host_q->waiter->completion);
	}

	ipu_core_memory_free(bus, &host_q->shared_buf_data);

	trans->free_queue_ids |= (1 << q_id);

	mutex_unlock(&trans->lock);
}

ssize_t ipu_core_jqs_msg_transport_user_read(struct paintbox_bus *bus,
		uint32_t q_id, void __user *buf, size_t size)
{
	struct host_jqs_queue_waiter waiter;
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;
	struct host_jqs_queue *host_q = &trans->queues[q_id];
	struct host_jqs_cbuf *host_cbuf = &host_q->host_jqs_sys_cbuf;
	unsigned long timeout_ms = USER_READ_TIMEOUT_MS;
	ssize_t ret;

	mutex_lock(&trans->lock);

	/* If the queue was freed before the trans->lock was acquired for the
	 * read then return a disconnect error.
	 */
	if (trans->free_queue_ids & (1 << q_id)) {
		mutex_unlock(&trans->lock);
		return -ECONNRESET;
	}

	/* Two concurrent calls to read is not supported */
	if (host_q->waiter) {
		mutex_unlock(&trans->lock);
		dev_err(bus->parent_dev, "%s: queue busy", __func__);
		return -EBUSY;
	}

	memset(&waiter, 0, sizeof(waiter));
	init_completion(&waiter.completion);

	host_q->waiter = &waiter;

	while (true) {
		long time_remaining;

		ret = ipu_core_jqs_cbuf_read(bus, host_cbuf, buf, size,
				ipu_core_jqs_cbus_copy_to_user);
		if (ret > 0) {
			break;
		} else if (ret < 0) {
			dev_err(bus->parent_dev,
					"%s: error reading from JQS circular buffer, ret %zd\n",
					__func__, ret);
			break;
		}

		mutex_unlock(&trans->lock);

		time_remaining = wait_for_completion_interruptible_timeout(
				&waiter.completion,
				msecs_to_jiffies(timeout_ms));

		mutex_lock(&trans->lock);

		/* An error reported by the queue takes precedence over other
		 * errors.
		 */
		if (waiter.ret != 0) {
			ret = waiter.ret;
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
		reinit_completion(&waiter.completion);
	}

	host_q->waiter = NULL;

	mutex_unlock(&trans->lock);

	return ret;
}

int ipu_core_jqs_msg_transport_user_write(struct paintbox_bus *bus,
		uint32_t q_id, const void __user *p, size_t size)
{
	return write_core(bus, q_id, p, size);
}

int ipu_core_jqs_msg_transport_kernel_write(struct paintbox_bus *bus,
	const struct jqs_message *msg)
{
	int ret;

	ret = write_core(bus, JQS_TRANSPORT_KERNEL_QUEUE_ID, msg,
			msg->size);
	if (ret != msg->size)
		return -EFAULT;

	return 0;
}

int ipu_core_jqs_msg_transport_kernel_write_sync(struct paintbox_bus *bus,
	const struct jqs_message *msg, struct jqs_message *rsp, size_t rsp_size)
{
	struct host_jqs_queue_waiter waiter;
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;
	struct host_jqs_queue *host_q =
			&trans->queues[JQS_TRANSPORT_KERNEL_QUEUE_ID];
	int ret;

	/* Schedule the waiter to store the response data */
	mutex_lock(&trans->lock);
	ret = init_waiter(host_q, rsp, rsp_size, &waiter);
	mutex_unlock(&trans->lock);

	if (ret < 0)
		return ret;

	/* Do the write */
	ret = ipu_core_jqs_msg_transport_kernel_write(bus, msg);
	if (ret < 0)
		return ret;

	 /* Wait for the response */
	mutex_lock(&trans->lock);
	ret = wait_for_data(bus, host_q, 10000);
	mutex_unlock(&trans->lock);

	return ret;
}

/* Process queues based on bitmask of q_ids */
void ipu_core_jqs_msg_transport_process_queues(struct paintbox_bus *bus,
		uint32_t q_ids)
{
	struct paintbox_jqs_msg_transport *trans = bus->jqs_msg_transport;

	mutex_lock(&trans->lock);
	if (trans->processing_queues) {
		/* Queues are already being processed. Mark down the request and
		 * exit.  Typically, this should rarely occur.
		 */
		trans->process_queues_request |= q_ids;
		mutex_unlock(&trans->lock);
		return;
	}

	/* This thread has claimed the privelege of dispatcher. Iterate until
	 * there have been no additional requests.
	 */
	trans->processing_queues = true;
	while (q_ids) {
		mutex_unlock(&trans->lock);
		process_queues(bus, q_ids);
		mutex_lock(&trans->lock);

		q_ids = trans->process_queues_request;
		trans->process_queues_request = 0;
	}

	trans->processing_queues = false;
	mutex_unlock(&trans->lock);
}
