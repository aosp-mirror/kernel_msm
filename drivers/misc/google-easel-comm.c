/*
 * Android/Easel coprocessor communication.
 *
 * Copyright 2016 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* #define DEBUG */

#include <uapi/linux/google-easel-comm.h>
#include "google-easel-comm-shared.h"
#include "google-easel-comm-private.h"

#include <linux/compat.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>

/* retry delay in msec for retrying flush of a message being processed */
#define MSG_FLUSH_RETRY_DELAY 10

/* Service for link status update */
static struct easelcomm_service link_service;

/* Mutex to guard easelcomm_service_list */
struct mutex service_mutex;

/* Per-Easel-service state */
static struct easelcomm_service *easelcomm_service_list[
		EASELCOMM_SERVICE_COUNT];

/* Local command channel local state */
struct easelcomm_cmd_channel_local {
	char *buffer; /* start of command buffer */
	char *readp; /* ptr to next entry to read */
	uint64_t consumer_seqnbr_next; /* next cmd seq# to be consumed */
};
static struct easelcomm_cmd_channel_local cmd_channel_local;

/* Remote command channel local state */
struct easelcomm_cmd_channel_remote {
	/* Protects access to all mutable fields below */
	struct mutex mutex;
	/* offset of next entry to write */
	uint64_t write_offset;
	/* next command sequence number to write */
	uint64_t write_seqnbr;
	/* remote consumer has caught up, ready to wrap channel buffer */
	struct completion wrap_ready;
	/* remote channel is initialized */
	bool initialized;
	/* channel initialization or link shutdown received from remote */
	struct completion init_state_changed;
};
static struct easelcomm_cmd_channel_remote cmd_channel_remote;

/* Per-file-descriptor (user) state */
struct easelcomm_user_state {
	/* Which Easel service is registered for this fd, or NULL if none */
	struct easelcomm_service *service;
};

/* max delay in msec waiting for remote to ack link shutdown */
#define LINK_SHUTDOWN_ACK_TIMEOUT 500
/* max delay in msec waiting for remote to return flush done */
#define SERVICE_FLUSH_DONE_TIMEOUT 5000

static bool easelcomm_up; /* is easelcomm up and running? */
/* used to wait for remote peer to ack link shutdown */
static DECLARE_COMPLETION(easelcomm_link_peer_shutdown);

static int easelcomm_open(struct inode *inode, struct file *file);
static int easelcomm_release(struct inode *inode, struct file *file);
static long easelcomm_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg);

#ifdef CONFIG_COMPAT
/* 32-bit userspace on 64-bit OS conversion defines */
struct easelcomm_compat_kbuf_desc {
	easelcomm_msgid_t message_id;  /* ID of message for this transfer */
	compat_uptr_t __user buf;      /* local buffer source or dest */
	int dma_buf_fd;                /* fd of local dma_buf */
	int buf_type;                  /* e.g. EASELCOMM_DMA_BUFFER_DMA_BUF */
	uint32_t buf_size;             /* size of the local buffer */
};

static long easelcomm_compat_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg);
#else
#define easelcomm_compat_ioctl NULL
#endif

static const struct file_operations easelcomm_fops = {
	.owner		= THIS_MODULE,
	.open		= easelcomm_open,
	.release	= easelcomm_release,
	.unlocked_ioctl = easelcomm_ioctl,
	.compat_ioctl	= easelcomm_compat_ioctl,
	.llseek		= no_llseek,
};

struct miscdevice easelcomm_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
#ifdef CONFIG_GOOGLE_EASEL_AP
	.name = "easelcomm-client",
#else
	.name = "easelcomm-server",
#endif
	.fops = &easelcomm_fops,
};
EXPORT_SYMBOL(easelcomm_miscdev);

/* Dump message info for debugging. */
static void easelcomm_dump_message(
	struct easelcomm_message_metadata *msg_metadata)
{
	struct easelcomm_kmsg *msg = msg_metadata->msg;

	dev_dbg(easelcomm_miscdev.this_device, "msg: id=%s%llu msgsize=%u dmasize=%u needreply=%d inreplyto=%llu replycode=%d\n",
		easelcomm_msgid_prefix(msg_metadata),
		msg->desc.message_id, msg->desc.message_size,
		msg->desc.dma_buf_size, msg->desc.need_reply,
		msg->desc.in_reply_to, msg->desc.replycode);
}

/*
 * Add an additional refcount bump on a message for which a refcount is
 * already held.  This function is for use by callers that hold a valid
 * message pointer for which a refcount is already held, in order to reflect
 * another reference being created, which will be dropped by another context
 * later.  At present this is used only to hand off incoming reply messages to
 * waiters, where the additional refcount reflects the fact that the original
 * message contains a pointer to the reply message.
 */
static void easelcomm_grab_reference(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata)
{
	if (!msg_metadata)
		return;
	spin_lock(&service->lock);
	msg_metadata->reference_count++;
	spin_unlock(&service->lock);
}

/*
 * Find a local message (for the specified service) by ID.
 * Returns a pointer to the metadata with the reference count bumped.
 */
struct easelcomm_message_metadata *easelcomm_find_local_message(
	struct easelcomm_service *service, easelcomm_msgid_t message_id)
{
	struct easelcomm_message_metadata *msg_metadata = NULL;
	struct easelcomm_message_metadata *msg_cursor;

	spin_lock(&service->lock);
	list_for_each_entry(msg_cursor, &service->local_list,
			list) {
		if (msg_cursor->msg->desc.message_id == message_id) {
			msg_metadata = msg_cursor;
			msg_metadata->reference_count++;
			break;
		}
	}
	spin_unlock(&service->lock);
	return msg_metadata;
}
EXPORT_SYMBOL(easelcomm_find_local_message);

/*
 * Find a remote message (for the specified service) by ID.
 * Returns a pointer to the metadata with the reference count bumped.
 */
struct easelcomm_message_metadata *easelcomm_find_remote_message(
	struct easelcomm_service *service, easelcomm_msgid_t message_id)
{
	struct easelcomm_message_metadata *msg_metadata = NULL;
	struct easelcomm_message_metadata *msg_cursor;

	spin_lock(&service->lock);
	list_for_each_entry(msg_cursor, &service->remote_list, list) {
		if (msg_cursor->msg->desc.message_id == message_id) {
			msg_metadata = msg_cursor;
			msg_metadata->reference_count++;
			break;
		}
	}
	spin_unlock(&service->lock);
	return msg_metadata;
}
EXPORT_SYMBOL(easelcomm_find_remote_message);

/*
 * Return a pointer to the reply message for the specified original message.
 * Responsibility for the existing reference count held for the reply message
 * (due to the linkage from the original message) now transfers to the caller,
 * which must drop the reference when appropriate.
 */
static struct easelcomm_message_metadata *easelcomm_grab_reply_message(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *orig_msg_metadata)
{
	struct easelcomm_message_metadata *reply_msg_metadata;

	if (!orig_msg_metadata)
		return NULL;
	spin_lock(&service->lock);
	reply_msg_metadata = orig_msg_metadata->reply_metadata;
	orig_msg_metadata->reply_metadata = NULL;
	spin_unlock(&service->lock);
	return reply_msg_metadata;
}

/*
 * Add a new message and metadata for the specified service.  Depending on the
 * message type, the message will be placed in the local or remote list and
 * potentially the receiveMessage queue.  The reference count for the message
 * starts at 1 due to the returned pointer.
 */
static struct easelcomm_message_metadata *easelcomm_add_metadata(
	struct easelcomm_service *service,
	enum easelcomm_msg_type msg_type, struct easelcomm_kmsg *msg)
{
	struct easelcomm_message_metadata *msg_metadata =
		kmalloc(sizeof(struct easelcomm_message_metadata), GFP_KERNEL);
	if (!msg_metadata)
		return NULL;
	msg_metadata->msg_type = msg_type;
	INIT_LIST_HEAD(&msg_metadata->list);
	msg_metadata->dma_xfer.sg_local = NULL;
	msg_metadata->dma_xfer.sg_local_size = 0;
	msg_metadata->dma_xfer.sg_remote = NULL;
	msg_metadata->dma_xfer.sg_remote_size = 0;
	init_completion(&msg_metadata->dma_xfer.sg_remote_ready);
	init_completion(&msg_metadata->dma_xfer.xfer_ready);
	init_completion(&msg_metadata->dma_xfer.xfer_done);
	msg_metadata->dma_xfer.aborting = false;
	msg_metadata->msg = msg;
	msg_metadata->reference_count = 1;
	msg_metadata->queued = false;
	msg_metadata->flushing = false;
	msg_metadata->free_message = false;
	init_completion(&msg_metadata->reply_received);
	msg_metadata->reply_metadata = NULL;

	spin_lock(&service->lock);
	switch (msg_metadata->msg_type) {
	case TYPE_LOCAL:
		/* add to the local list */
		list_add_tail(&msg_metadata->list, &service->local_list);
		break;
	case TYPE_REMOTE_REPLY:
	case TYPE_REMOTE_NONREPLY:
		/* add to the remote list */
		list_add_tail(&msg_metadata->list, &service->remote_list);

		if (msg_metadata->msg_type == TYPE_REMOTE_NONREPLY) {
			/*
			 * Also add to the receiveMessage queue and wakeup
			 * waiter.
			 */
			list_add_tail(&msg_metadata->rcvq_list,
				&service->receivemsg_queue);
			msg_metadata->queued = true;
			complete(&service->receivemsg_queue_new);
		}
		break;
	default:
		WARN_ON(1);
	}
	spin_unlock(&service->lock);
	return msg_metadata;
}

/*
 * Free message, remove from lists and receiveMessage queue.  Called when
 * message reference count drops to zero, and either: the message is marked
 * for freeing when no longer referenced, or: the message is being flushed.
 * The lock for the containing service must be held.  This method is only for
 * use by reference counting and message flushing routines, not for general use.
 */
static void easelcomm_free_message(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata)
{
	if (!msg_metadata)
		return;

	WARN_ON(msg_metadata->reference_count);
	/* remove from local or remote list */
	list_del(&msg_metadata->list);
	/* If queued remove from receive queue */
	if (msg_metadata->queued)
		list_del(&msg_metadata->rcvq_list);
	kfree(msg_metadata->msg);
	vfree(msg_metadata->dma_xfer.sg_local);  /* was allocated by vmalloc */
	kfree(msg_metadata->dma_xfer.sg_remote);
	kfree(msg_metadata);
}

/*
 * Decrement message refcount, optionally mark for freeing.  The message is
 * freed if it is now, or has previously been, marked for freeing when
 * unreferenced and the reference count is dropped to zero.  Once this method
 * is called the supplied message pointer is no longer valid and must not be
 * dereferenced.  Any code that needs to reference the same message again
 * needs to find it by message ID.
 *
 * A reference count must be held whenever a pointer to the message is created
 * and valid (apart from local or remote message list linkage, which is always
 * present until the message is freed).	 (And except briefly while maintaining
 * message state and the service lock is held.)	 A reference count of zero
 * indicates the message is not actively being manipulated by a kernel code
 * path and no pointers to it existing apart from the local or remote message
 * list for its associated service -- if not in the process of being freed then
 * it is expected something will later grab a reference to the message and
 * resume handling it.
 */
void easelcomm_drop_reference(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata, bool mark_free)
{
	if (!msg_metadata)
		return;

	spin_lock(&service->lock);
	if (!WARN_ON(!msg_metadata->reference_count))
		msg_metadata->reference_count--;

	if (mark_free)
		msg_metadata->free_message = true;

	if (!msg_metadata->reference_count && msg_metadata->free_message)
		easelcomm_free_message(service, msg_metadata);
	spin_unlock(&service->lock);
}
EXPORT_SYMBOL(easelcomm_drop_reference);

/*
 * Attempt to flush a message.	If a reference count is held on the message or
 * the DMA transfer for the message is in progress then the message is not
 * flushed. If a DMA transfer is active then a DMA abort is initiated.
 * Function returns true if need to retry flushing the message later due to
 * the above, or false if now flushed.
 */
static bool easelcomm_flush_message(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata)
{
	easelcomm_msgid_t msgid;
	bool ret = false;

	if (!msg_metadata || !msg_metadata->msg)
		return false;

	msgid = msg_metadata->msg->desc.message_id;
	/* Mark message for flushing in case message is still in progress */
	msg_metadata->flushing = true;

	/*
	 * If we may be working on a DMA transfer then trigger an abort.  If
	 * a local process is working on it then a refcount will be held, and
	 * we'll need to retry after dropping the spinlock, letting the
	 * process handle the DMA abort and then drop the refcount.
	 */
	if (msg_metadata->msg->desc.dma_buf_size &&
		!msg_metadata->dma_xfer.aborting) {
		dev_dbg(easelcomm_miscdev.this_device,
			"flush abort DMA msg %u:%s%llu\n",
			service->service_id,
			easelcomm_msgid_prefix(msg_metadata), msgid);
		msg_metadata->dma_xfer.aborting = true;
		complete(&msg_metadata->dma_xfer.sg_remote_ready);
		complete(&msg_metadata->dma_xfer.xfer_ready);
		complete(&msg_metadata->dma_xfer.xfer_done);
	}

	if (msg_metadata->reference_count) {
		dev_dbg(easelcomm_miscdev.this_device,
			"not flushing msg %u:%s%llu refcnt=%d\n",
			service->service_id,
			easelcomm_msgid_prefix(msg_metadata), msgid,
			msg_metadata->reference_count);
		ret = true;
	} else {
		easelcomm_free_message(service, msg_metadata);
		dev_dbg(easelcomm_miscdev.this_device,
			"flushed msg %u:%s%llu\n",
			service->service_id,
			easelcomm_msgid_prefix(msg_metadata), msgid);
		ret = false;
	}

	return ret;
}

/* Flush all messages (local or remote) kept locally for the service. */
static void easelcomm_flush_local_service(struct easelcomm_service *service)
{
	struct easelcomm_message_metadata *msg_cursor;
	struct easelcomm_message_metadata *msg_temp;
	int i;
	bool need_retry = true;

#define MAX_FLUSH_TRIES 4

	for (i = 0; need_retry && i < MAX_FLUSH_TRIES; i++) {
		spin_lock(&service->lock);
		need_retry = false;
		list_for_each_entry_safe(
			msg_cursor, msg_temp, &service->local_list, list) {
			need_retry |= easelcomm_flush_message(
				service, msg_cursor);
		}
		list_for_each_entry_safe(
			msg_cursor, msg_temp, &service->remote_list, list) {
			need_retry |= easelcomm_flush_message(
				service, msg_cursor);
		}
		spin_unlock(&service->lock);

		if (!need_retry)
			break;

		/* sleep, let refcount holders abort DMA/reply wait */
		msleep(MSG_FLUSH_RETRY_DELAY);
	}

	if (need_retry)
		dev_err(easelcomm_miscdev.this_device,
			"svc %u failed to flush messages after %d tries\n",
			service->service_id, MAX_FLUSH_TRIES);
}

/*
 * Wake up any message of a service waiting for reply.
 */
static void __force_complete_reply_waiter(struct easelcomm_service *service)
{
	struct easelcomm_message_metadata *msg_cursor;

	spin_lock(&service->lock);
	list_for_each_entry(msg_cursor, &service->local_list, list) {
		if (msg_cursor->msg->desc.need_reply)
			complete(&msg_cursor->reply_received);
	}
	spin_unlock(&service->lock);
}

/*
 * Handle local-side processing of shutting down an Easel service.  Easel
 * services are shutdown when the local side handling process closes its fd
 * or issues the shutdown ioctl, or the remote side sends a CLOSE_SERVICE
 * command (meaning the remote side is doing one of those things).
 *
 * Flush local state if local side initiated the shutdown, wakeup the
 * receiveMessage() waiter.
 */
static void easelcomm_handle_service_shutdown(
	struct easelcomm_service *service, bool shutdown_local)
{
	dev_dbg(easelcomm_miscdev.this_device, "svc %u shutdown from %s\n",
		service->service_id, shutdown_local ? "local" : "remote");

	/*
	 * If local side shutting down then flush local state, just in case.
	 * If remote is closing then local side may still have messages in
	 * progress, just signal the receivemessage() caller to return
	 * shutdown status.
	 */
	if (shutdown_local)
		easelcomm_flush_local_service(service);

	spin_lock(&service->lock);
	if (shutdown_local)
		service->shutdown_local = true;
	else
		service->shutdown_remote = true;
	spin_unlock(&service->lock);
	/* Wakeup any receiveMessage() waiter so they can return to user */
	complete(&service->receivemsg_queue_new);

	/* Wakeup flush_done waiter so they can return to user */
	complete(&service->flush_done);

	/* Wakeup any sendMessageReceiveReply() waiter to return to user */
	__force_complete_reply_waiter(service);
}

/*
 * Shutdown easelcomm local activity, mark link down.
 */
static void easelcomm_stop_local(void)
{
	int i;

	if (!easelcomm_up)
		return;

	dev_dbg(easelcomm_miscdev.this_device, "stopping\n");
	/* Set local shutdown flag, disallow further activity */
	easelcomm_up = false;

	for (i = 0; i < EASELCOMM_SERVICE_COUNT; i++) {
		struct easelcomm_service *service = easelcomm_service_list[i];

		if (service != NULL)
			easelcomm_handle_service_shutdown(service, true);
	}
}

/*
 * Shutdown easelcomm communications.  async is true if emergency shutdown (on
 * panic), don't wait for sync from other side.
 */
static void easelcomm_stop(bool async)
{
	int ret;

	if (!easelcomm_up)
		return;

	easelcomm_send_cmd_noargs(
		&link_service,
		EASELCOMM_CMD_LINK_SHUTDOWN);
	if (!async) {
		ret = wait_for_completion_interruptible_timeout(
			&easelcomm_link_peer_shutdown,
			msecs_to_jiffies(LINK_SHUTDOWN_ACK_TIMEOUT));
		if (ret <= 0)
			dev_warn(easelcomm_miscdev.this_device,
				"error or timeout on peer link shutdown ack\n");
	}
	easelcomm_stop_local();
}

/*
 * LINK_SHUTDOWN command received from remote, stop local side and send
 * ACK_SHUTDOWN back to remote.
 */
static void easelcomm_handle_cmd_link_shutdown(void)
{
	dev_dbg(easelcomm_miscdev.this_device,
		"recv cmd LINK_SHUTDOWN\n");
	easelcomm_stop_local();
	dev_dbg(easelcomm_miscdev.this_device,
		"send cmd ACK_SHUTDOWN\n");
	easelcomm_send_cmd_noargs(
		&link_service,
		EASELCOMM_CMD_ACK_SHUTDOWN);
}

/* ACK_SHUTDOWN command received from remote, wakeup waiter. */
static void easelcomm_handle_cmd_ack_shutdown(void)
{
	dev_dbg(easelcomm_miscdev.this_device,
		"recv cmd ACK_SHUTDOWN\n");
	complete(&easelcomm_link_peer_shutdown);
}

/*
 * Handle SEND_MSG command from remote.  Add a remote message, link a reply
 * to original message and wakeup waiter.
 */
static void easelcomm_handle_cmd_send_msg(
	struct easelcomm_service *service, char *command_args,
	int command_arg_len)
{
	struct easelcomm_kmsg *cmd_msg;
	struct easelcomm_kmsg *new_msg;
	enum easelcomm_msg_type msg_type;
	struct easelcomm_message_metadata *msg_metadata;
	bool discard_message = false;

	if (WARN_ON(command_arg_len < sizeof(struct easelcomm_kmsg_desc)))
		return;
	cmd_msg = (struct easelcomm_kmsg *) command_args;
	if (WARN_ON(command_arg_len != sizeof(struct easelcomm_kmsg_desc) +
			cmd_msg->desc.message_size))
		return;

	dev_dbg(easelcomm_miscdev.this_device,
		"recv cmd SEND_MSG msg %u:r%llu\n",
		service->service_id, cmd_msg->desc.message_id);

	new_msg = kmalloc(command_arg_len, GFP_KERNEL);
	if (WARN_ON(!new_msg))
		return;
	memcpy(new_msg, cmd_msg, command_arg_len);
	msg_type = new_msg->desc.in_reply_to ?
		TYPE_REMOTE_REPLY : TYPE_REMOTE_NONREPLY;
	msg_metadata = easelcomm_add_metadata(service, msg_type, new_msg);
	if (WARN_ON(!msg_metadata)) {
		kfree(new_msg);
		return;
	}
	easelcomm_dump_message(msg_metadata);

	/* If a reply then link to the original message and wakeup waiter. */
	if (msg_type == TYPE_REMOTE_REPLY) {
		struct easelcomm_message_metadata *orig_msg_metadata =
			easelcomm_find_local_message(
				service, msg_metadata->msg->desc.in_reply_to);

		if (orig_msg_metadata) {
			orig_msg_metadata->reply_metadata =
				msg_metadata;
			/*
			 * Bump reference count for this new ref, waiter will
			 * drop.
			 */
			easelcomm_grab_reference(service, msg_metadata);
			complete(&orig_msg_metadata->reply_received);
			easelcomm_drop_reference(
				service, orig_msg_metadata, false);
		} else {
			dev_dbg(easelcomm_miscdev.this_device,
				"reply msg %u:r%llu reply-to msg %u:l%llu not found\n",
				service->service_id,
				msg_metadata->msg->desc.message_id,
				service->service_id,
				msg_metadata->msg->desc.in_reply_to);
			discard_message = true;
		}
	}

	easelcomm_drop_reference(service, msg_metadata, discard_message);
}

/*
 * Remote side of link indicates command channel ready.  Both server and
 * client call this when handling any command.
 * In addition, server calls this when a LINK_INIT command is received from
 * the client.
 *
 * Init stuff and wakeup anybody waiting for the channel to be ready (to send
 * a command).
 */
static void easelcomm_cmd_channel_remote_set_ready(void)
{
	struct easelcomm_cmd_channel_remote *channel =
		&cmd_channel_remote;

	mutex_lock(&channel->mutex);
	/* Return right away if channel is already initialized */
	if (channel->initialized) {
		mutex_unlock(&channel->mutex);
		return;
	}
	/* Next write offset starts at top after header, cmd seq# zero */
	channel->write_offset =
		(sizeof(struct easelcomm_cmd_channel_header) + 7) & ~0x7;
	channel->write_seqnbr = 0;
	/* mark channel initialized, wakeup waiters. */
	channel->initialized = true;
	complete_all(&channel->init_state_changed);
	easelcomm_up = true;
	mutex_unlock(&channel->mutex);
}

/*
 * LINK_INIT command received from client, init local state for remote command
 * channel.
 */
static void easelcomm_handle_cmd_link_init(
	char *command_args, int command_arg_len)
{
	dev_dbg(easelcomm_miscdev.this_device, "recv cmd LINK_INIT\n");
	easelcomm_cmd_channel_remote_set_ready();
}

/*
 * Wait for remote command channel ready and initialized, prior to sending a
 * command.  Returns 0 if command channel ready and channel mutex is held, or
 * non-zero if signal received while waiting, channel mutex not held.
 */
static int easelcomm_wait_channel_initialized(
	struct easelcomm_cmd_channel_remote *channel)
{
	int ret = 0;

	mutex_lock(&channel->mutex);
	while (!channel->initialized) {
		mutex_unlock(&channel->mutex);
		ret = wait_for_completion_interruptible(
			&channel->init_state_changed);
		if (ret)
			return ret;
		mutex_lock(&channel->mutex);
	}

	return ret;
}

/* CLOSE_SERVICE command received from remote, shut down service. */
static void easelcomm_handle_cmd_close_service(
	struct easelcomm_service *service)
{
	dev_dbg(easelcomm_miscdev.this_device,
		"recv cmd CLOSE_SERVICE svc %u\n",
		service->service_id);
	easelcomm_handle_service_shutdown(service, false);
}

/*
 * FLUSH_SERVICE command received from remote, flush local state for service
 * and send FLUSH_SERVICE_DONE back to remote.
 */
static void easelcomm_handle_cmd_flush_service(
	struct easelcomm_service *service)
{
	dev_dbg(easelcomm_miscdev.this_device,
		"recv cmd FLUSH_SERVICE svc %u\n",
		service->service_id);
	easelcomm_flush_local_service(service);
	dev_dbg(easelcomm_miscdev.this_device,
		"send cmd FLUSH_SERVICE_DONE svc %u\n",
		service->service_id);
	easelcomm_send_cmd_noargs(service, EASELCOMM_CMD_FLUSH_SERVICE_DONE);
}

/* FLUSH_SERVICE_DONE command received from remote, wakeup waiter. */
static void easelcomm_handle_cmd_flush_service_done(
	struct easelcomm_service *service)
{
	dev_dbg(easelcomm_miscdev.this_device,
		"recv cmd FLUSH_SERVICE_DONE svc %u\n", service->service_id);
	complete(&service->flush_done);
}

static void easelcomm_init_service(struct easelcomm_service *service, int id)
{
	if (service == NULL) {
		dev_err(easelcomm_miscdev.this_device, "service is null\n");
		return;
	}
	service->service_id = id;
	service->user = NULL;
	service->shutdown_local = false;
	service->shutdown_remote = false;
	spin_lock_init(&service->lock);
	INIT_LIST_HEAD(&service->local_list);
	INIT_LIST_HEAD(&service->receivemsg_queue);
	init_completion(&service->receivemsg_queue_new);
	INIT_LIST_HEAD(&service->remote_list);
	init_completion(&service->flush_done);
}

/* Caller of this function must hold service_mutex. */
static struct easelcomm_service *easelcomm_create_service(int id)
{
	struct easelcomm_service *service;

	if (easelcomm_service_list[id] != NULL)
		return easelcomm_service_list[id];

	service = kzalloc(sizeof(struct easelcomm_service), GFP_KERNEL);
	if (service == NULL) {
		dev_err(easelcomm_miscdev.this_device,
				"could not allocate service %u\n", id);
		return NULL;
	}
	easelcomm_init_service(service, id);
	easelcomm_service_list[id] = service;
	return service;
}

/* Command received from remote, dispatch. */
static void easelcomm_handle_command(struct easelcomm_cmd_header *cmdhdr)
{
	struct easelcomm_service *service;
	char *cmdargs = (char *)cmdhdr + sizeof(struct easelcomm_cmd_header);

	/*
	 * Any command can inform the server that remote is ready, not just
	 * LINK_INIT.
	 */
	if (!easelcomm_is_client())
		easelcomm_cmd_channel_remote_set_ready();

	switch(cmdhdr->command_code) {
	case EASELCOMM_CMD_LINK_INIT:
		easelcomm_handle_cmd_link_init(
			cmdargs, cmdhdr->command_arg_len);
		return;
	case EASELCOMM_CMD_LINK_SHUTDOWN:
		easelcomm_handle_cmd_link_shutdown();
		return;
	case EASELCOMM_CMD_ACK_SHUTDOWN:
		easelcomm_handle_cmd_ack_shutdown();
		return;
	default:
		break;
	}

	if (cmdhdr->service_id >= EASELCOMM_SERVICE_COUNT) {
		dev_err(easelcomm_miscdev.this_device,
			"invalid service ID %u received\n",
			cmdhdr->service_id);
		return;
	}
	service = easelcomm_service_list[cmdhdr->service_id];

	/*
	 * If service has not been initialized,
	 * initialize the service here without userspace ownership.
	 * Userspace ownership could be claimed later.
	 */
	if (service == NULL) {
		mutex_lock(&service_mutex);
		service = easelcomm_create_service(cmdhdr->service_id);
		mutex_unlock(&service_mutex);
		if (service == NULL) {
			dev_err(easelcomm_miscdev.this_device,
				"could not handle cmd %d for service %u\n",
				cmdhdr->command_code, cmdhdr->service_id);
			return;
		}
	}

	switch (cmdhdr->command_code) {
	case EASELCOMM_CMD_SEND_MSG:
		easelcomm_handle_cmd_send_msg(
			service, cmdargs, cmdhdr->command_arg_len);
		break;
	case EASELCOMM_CMD_DMA_SG:
		easelcomm_handle_cmd_dma_sg(
			service, cmdargs, cmdhdr->command_arg_len);
		break;
	case EASELCOMM_CMD_DMA_XFER:
		easelcomm_handle_cmd_dma_xfer(
			service, cmdargs, cmdhdr->command_arg_len);
		break;
	case EASELCOMM_CMD_DMA_DONE:
		easelcomm_handle_cmd_dma_done(
			service, cmdargs, cmdhdr->command_arg_len);
		break;
	case EASELCOMM_CMD_FLUSH_SERVICE:
		easelcomm_handle_cmd_flush_service(service);
		break;
	case EASELCOMM_CMD_FLUSH_SERVICE_DONE:
		easelcomm_handle_cmd_flush_service_done(service);
		break;
	case EASELCOMM_CMD_CLOSE_SERVICE:
		easelcomm_handle_cmd_close_service(service);
		break;
	default:
		dev_err(easelcomm_miscdev.this_device,
			"svc %u invalid command code %u received\n",
			cmdhdr->service_id, cmdhdr->command_code);
	}
}

/*
 * Bump the consumer sequence number in the local command channel state,
 * indicating that we've consumed messages prior to the new number in
 * sequence.  The sequence number cannot be the reserved "command buffer
 * wrapped" marker; bump twice if needed to avoid.
 *
 * Not locked, but there's only one thread handling local command channel
 * data, in worker context.
 */
static void easelcomm_cmd_channel_bump_consumer_seqnbr(
	struct easelcomm_cmd_channel_local *channel)
{
	channel->consumer_seqnbr_next++;

	if (channel->consumer_seqnbr_next == CMD_BUFFER_WRAP_MARKER)
		channel->consumer_seqnbr_next++;
}

/*
 * Interrupt for new local command channel data ready was received.  This
 * function is called on a workqueue worker.  Process any new command channel
 * data found.
 *
 * Not locked, but there's only one thread handling local command channel
 * data, in worker context.
 */
void easelcomm_cmd_channel_data_handler(void)
{
	struct easelcomm_cmd_channel_local *channel =
		&cmd_channel_local;
	struct easelcomm_cmd_channel_header *channel_buf_hdr =
		(struct easelcomm_cmd_channel_header *)
		channel->buffer;

	/* While we haven't caught up to producer in sequence #s processed. */
	while (channel->consumer_seqnbr_next !=
		channel_buf_hdr->producer_seqnbr_next) {
		struct easelcomm_cmd_header *cmdhdr =
			(struct easelcomm_cmd_header *)channel->readp;
		uint32_t saved_cmd_len = READ_ONCE(cmdhdr->command_arg_len);

		dev_dbg(easelcomm_miscdev.this_device, "cmdchan consumer loop prodseq=%llu consseq=%llu off=%lx\n",
			channel_buf_hdr->producer_seqnbr_next,
			channel->consumer_seqnbr_next,
			channel->readp - channel->buffer);

		/*
		 * If producer dropped a wrap marker at the current position
		 * then wrap our read pointer and let the producer know we
		 * wrapped and both sides are ready to continue at the top of
		 * the buffer.
		 */
		if (cmdhdr->sequence_nbr == CMD_BUFFER_WRAP_MARKER) {
			dev_dbg(easelcomm_miscdev.this_device,
				"cmdchan consumer wrap at off=%lx\n",
				channel->readp - channel->buffer);
			channel->readp =
				channel->buffer +
				sizeof(struct easelcomm_cmd_channel_header);
			/* Send consumer wrapped IRQ to remote */
			easelcomm_hw_send_wrap_interrupt();
			/* Wrapping consumes a seqnbr */
			easelcomm_cmd_channel_bump_consumer_seqnbr(channel);
			continue;
		}

		dev_dbg(easelcomm_miscdev.this_device,
			"cmdchan recv cmd seq=%llu svc=%u cmd=%u len=%u off=%lx\n",
			cmdhdr->sequence_nbr, cmdhdr->service_id,
			cmdhdr->command_code, saved_cmd_len,
			channel->readp - channel->buffer);
		if (sizeof(struct easelcomm_cmd_header) +
			saved_cmd_len >
			EASELCOMM_CMD_CHANNEL_SIZE) {
			dev_err(easelcomm_miscdev.this_device,
				"command channel corruption detected: seq=%llu svc=%u cmd=%u len=%u off=%lx\n",
				cmdhdr->sequence_nbr, cmdhdr->service_id,
				cmdhdr->command_code, saved_cmd_len,
				channel->readp - channel->buffer);
			break;
		}

		if (cmdhdr->sequence_nbr !=
			channel->consumer_seqnbr_next) {
			dev_err(easelcomm_miscdev.this_device,
				"command channel corruption detected: expected seq# %llu, got %llu\n",
				channel->consumer_seqnbr_next,
				cmdhdr->sequence_nbr);
			break;
		}

		/* Process the command. */
		easelcomm_handle_command(cmdhdr);

		/* Post-process double check */
		if (saved_cmd_len != cmdhdr->command_arg_len) {
			dev_err(easelcomm_miscdev.this_device,
				"command channel corruption detected: off=%lx expected len %u got %u\n",
				channel->readp - channel->buffer,
				saved_cmd_len, cmdhdr->command_arg_len);
			break;
		}

		channel->readp += sizeof(struct easelcomm_cmd_header) +
			saved_cmd_len;

		/* 8-byte-align next entry pointer */
		if ((uintptr_t)channel->readp & 0x7)
			channel->readp += 8 - ((uintptr_t)channel->readp & 0x7);
		easelcomm_cmd_channel_bump_consumer_seqnbr(channel);
	}
	dev_dbg(easelcomm_miscdev.this_device, "cmdchan consumer exiting prodseq=%llu consseq=%llu off=%lx\n",
		channel_buf_hdr->producer_seqnbr_next,
		channel->consumer_seqnbr_next,
		channel->readp - channel->buffer);
}
EXPORT_SYMBOL(easelcomm_cmd_channel_data_handler);

/*
 * Interrupt from remote indicating remote has followed our channel buffer
 * wrap received.  This function called in workqueue worker context.  Wakeup
 * the command producer that is waiting on the remote to follow the wrap.
 */
void easelcomm_cmd_channel_wrap_handler(void)
{
	struct easelcomm_cmd_channel_remote *channel =
		&cmd_channel_remote;
	dev_dbg(easelcomm_miscdev.this_device,
		"cmdchan remote wrap IRQ received\n");
	complete(&channel->wrap_ready);
}
EXPORT_SYMBOL(easelcomm_cmd_channel_wrap_handler);

/* Device file open.  Allocate a user state structure. */
static int easelcomm_open(struct inode *inode, struct file *file)
{
	struct easelcomm_user_state *user_state;

	if (!easelcomm_up)
		return -ENODEV;

	user_state = kzalloc(sizeof(struct easelcomm_user_state), GFP_KERNEL);
	if (!user_state)
		return -ENOMEM;
	file->private_data = user_state;
	return 0;
}

/* Initiate service shutdown from fd release or ioctl. */
static void easelcomm_initiate_service_shutdown(
	struct easelcomm_service *service)
{
	bool already_shutdown;

	spin_lock(&service->lock);
	already_shutdown = service->shutdown_local;
	spin_unlock(&service->lock);

	if (already_shutdown) {
		dev_dbg(easelcomm_miscdev.this_device,
			"svc %u already shutdown\n",
			service->service_id);
		return;
	}

	dev_dbg(easelcomm_miscdev.this_device, "svc %u initiate shutdown\n",
		service->service_id);
	easelcomm_handle_service_shutdown(service, true);
	dev_dbg(easelcomm_miscdev.this_device,
		"svc %u send cmd CLOSE_SERVICE\n", service->service_id);
	easelcomm_send_cmd_noargs(service, EASELCOMM_CMD_CLOSE_SERVICE);
}

/* Device file descriptor release, initiate service shutdown and cleanup */
static int easelcomm_release(struct inode *inode, struct file *file)
{
	struct easelcomm_user_state *user_state = file->private_data;

	if (user_state) {
		struct easelcomm_service *service = user_state->service;

		if (service) {
			dev_dbg(easelcomm_miscdev.this_device,
				"svc %u release\n", service->service_id);

			if (easelcomm_up)
				easelcomm_initiate_service_shutdown(service);
			spin_lock(&service->lock);
			service->user = NULL;
			spin_unlock(&service->lock);
		}

		kfree(file->private_data);
		file->private_data = NULL;
	}
	return 0;
}

/*
 * Bump the producer next sequence number in the remote command channel buffer
 * header.  After this, the remote side can then see that entries prior to
 * that sequence number are present.  The sequence number cannot be the
 * reserved "command buffer wrapped" marker; bump twice to avoid if needed.
 *
 * Call with remote channel producer mutex held.
 */
static int easelcomm_cmd_channel_bump_producer_seqnbr(
	struct easelcomm_cmd_channel_remote *channel)
{
	int ret;

	channel->write_seqnbr++;
	if (channel->write_seqnbr == CMD_BUFFER_WRAP_MARKER)
		channel->write_seqnbr++;
	ret = easelcomm_hw_remote_write(
		&channel->write_seqnbr, sizeof(uint64_t),
		(uint64_t)EASELCOMM_CMDCHAN_PRODUCER_SEQNBR);
	return ret;
}

/*
 * Start the process of writing a new command to the remote command channel.
 * Wait for channel init'ed and grab the mutex.	 Wraparound the buffer if
 * needed.  Write the command header.
 * If no error then return zero and channel mutex is locked on return.
 * If error then return non-zero and channel mutex is unlocked.
 */
int easelcomm_start_cmd(
	struct easelcomm_service *service, int command_code,
	int command_arg_len)
{
	struct easelcomm_cmd_channel_remote *channel =
		&cmd_channel_remote;
	struct easelcomm_cmd_header cmdhdr;
	unsigned int cmdbuf_size =
	    sizeof(struct easelcomm_cmd_header) + command_arg_len;
	int ret;

	/*
	 * If never enough room for the command plus an 8-byte wrap marker,
	 * bail.
	 */
	if (cmdbuf_size > EASELCOMM_CMD_CHANNEL_SIZE - 8 -
		sizeof(struct easelcomm_cmd_channel_header))
		return -EINVAL;

	ret = easelcomm_wait_channel_initialized(channel);
	/* If no error then channel->mutex is locked */
	if (ret)
		return ret;

	/* Choose a spot for the new entry, wrap around if needed. */
	if (channel->write_offset + cmdbuf_size >
		EASELCOMM_CMD_CHANNEL_SIZE - 8 -
		sizeof(struct easelcomm_cmd_channel_header)) {
		uint64_t wrap_marker = CMD_BUFFER_WRAP_MARKER;

		/* Write the "buffer wrapped" marker in place of sequence # */
		dev_dbg(easelcomm_miscdev.this_device, "cmdchan producer wrap at off=%llx seq=%llu\n",
			channel->write_offset, channel->write_seqnbr);
		ret = easelcomm_hw_remote_write(
			&wrap_marker, sizeof(wrap_marker),
			channel->write_offset);
		if (ret)
			goto error;

		/*
		 * Bump the producer seqnbr for the wrap marker (so consumer
		 * knows its there) and send IRQ to remote to consume the new
		 * data.
		 *
		 * Consumer will process entries up to and including the wrap
		 * marker and then send a "wrapped" interrupt that wakes up
		 * the wrap_ready completion.
		 */
		ret = easelcomm_cmd_channel_bump_producer_seqnbr(channel);
		if (ret)
			goto error;
		ret = easelcomm_hw_send_data_interrupt();
		if (ret)
			goto error;

		channel->write_offset =
			(sizeof(struct easelcomm_cmd_channel_header) + 7)
			& ~0x7;
		/* Wait for remote to catch up.	 IRQ from remote wakes this. */
		ret = wait_for_completion_interruptible(&channel->wrap_ready);
		if (ret)
			goto error;
	}

	dev_dbg(easelcomm_miscdev.this_device, "cmdchan producer sending cmd seq#%llu svc=%u cmd=%u arglen=%u off=%llx\n",
		channel->write_seqnbr, service->service_id, command_code,
		command_arg_len, channel->write_offset);
	cmdhdr.service_id = service->service_id;
	cmdhdr.sequence_nbr = channel->write_seqnbr;
	cmdhdr.command_code = command_code;
	cmdhdr.command_arg_len = command_arg_len;

	/*
	 * Send the command header. Subsequent calls to
	 * easelcomm_append_cmd_args() and easelcomm_send_cmd() will finish
	 * the in-progress command.
	 */
	ret = easelcomm_hw_remote_write(
		&cmdhdr, sizeof(cmdhdr), channel->write_offset);
	if (ret)
		goto error;

	/* Advance write offset */
	channel->write_offset += sizeof(cmdhdr);
	return 0;

error:
	mutex_unlock(&channel->mutex);
	return ret;
}
EXPORT_SYMBOL(easelcomm_start_cmd);

/*
 * Append arguments to the in-progress command being sent.
 * Call with remote channel producer mutex held.
 */
int easelcomm_append_cmd_args(
	struct easelcomm_service *service, void *cmd_args,
	size_t cmd_args_len)
{
	struct easelcomm_cmd_channel_remote *channel =
		&cmd_channel_remote;
	int ret = easelcomm_hw_remote_write(
		cmd_args, cmd_args_len, channel->write_offset);
	if (ret) {
		mutex_unlock(&channel->mutex);
		return ret;
	}
	channel->write_offset += cmd_args_len;
	return 0;
}
EXPORT_SYMBOL(easelcomm_append_cmd_args);

/* Start and send a command with no arguments. */
int easelcomm_send_cmd_noargs(
	struct easelcomm_service *service, int command_code)
{
	int ret;

	ret = easelcomm_start_cmd(service, command_code, 0);
	if (ret)
		return ret;
	return easelcomm_send_cmd(service);
}

/*
 * Finish sending an in-progress command to the remote.
 */
int easelcomm_send_cmd(struct easelcomm_service *service)
{
	struct easelcomm_cmd_channel_remote *channel =
		&cmd_channel_remote;
	int ret;

	/*
	 * Bump the producer sequence number.  Remote may now see and process
	 * the new entry as soon as this value is visible to it.
	 */
	ret = easelcomm_cmd_channel_bump_producer_seqnbr(channel);
	if (ret)
		goto out;
	/* Send data ready IRQ to remote */
	ret = easelcomm_hw_send_data_interrupt();
	if (ret)
		goto out;

	/* Bump write offset for next command, 8-byte-integer-aligned */
	if (channel->write_offset & 0x7)
		channel->write_offset += 8 - (channel->write_offset & 0x7);

out:
	mutex_unlock(&channel->mutex);
	return ret;
}
EXPORT_SYMBOL(easelcomm_send_cmd);

/*
 * Return next message ID in sequence for this service.  Message ID zero is
 * not used, bump twice to avoid if needed.
 */
static easelcomm_msgid_t easelcomm_next_msgid(
	struct easelcomm_service *service)
{
	easelcomm_msgid_t next_id;

	spin_lock(&service->lock);
	next_id = ++service->next_id;
	if (!next_id)
		next_id = ++service->next_id;
	spin_unlock(&service->lock);
	return next_id;
}

/*
 * Handle SENDMSG ioctl, the message descriptor send part of easelcomm
 * sendMessage*() and sendReply() calls.
 */
static int easelcomm_send_message_ioctl(
	struct easelcomm_service *service,
	struct easelcomm_kmsg_desc __user *pmsg_desc)
{
	struct easelcomm_kmsg_desc *msg_desc;
	struct easelcomm_message_metadata *msg_metadata = NULL;
	int ret = 0;

	msg_desc = kmalloc(sizeof(struct easelcomm_kmsg_desc), GFP_KERNEL);
	if (!msg_desc)
		return -ENOMEM;
	if (copy_from_user(msg_desc, (void __user *) pmsg_desc,
			   sizeof(struct easelcomm_kmsg_desc))) {
		ret = -EFAULT;
		goto out_freedesc;
	}
	if (msg_desc->message_size > EASELCOMM_MAX_MESSAGE_SIZE) {
		ret = -EINVAL;
		goto out_freedesc;
	}

	/* Assign a message ID to the new message and add to local list. */
	msg_desc->message_id = easelcomm_next_msgid(service);
	msg_metadata = easelcomm_add_metadata(service, TYPE_LOCAL,
					(struct easelcomm_kmsg *)msg_desc);
	if (!msg_metadata) {
		ret = -ENOMEM;
		goto out_freedesc;
	}
	/*
	 * After a successful call to the above, msg_desc is pointed to by
	 * msg_metadata and the memory is managed along with the other
	 * metadata.
	 */

	dev_dbg(easelcomm_miscdev.this_device, "SENDMSG msg %u:l%llu\n",
		service->service_id, msg_desc->message_id);
	easelcomm_dump_message(msg_metadata);

	/* Copy the updated msg desc with message ID to user. */
	if (copy_to_user((void __user *) pmsg_desc, msg_desc,
				sizeof(struct easelcomm_kmsg_desc))) {
		easelcomm_drop_reference(service, msg_metadata, true);
		return -EFAULT;
	}

	/*
	 * If this is a reply then we are done with the replied-to remote
	 * message.
	 */
	if (msg_metadata->msg->desc.in_reply_to) {
		struct easelcomm_message_metadata *orig_msg_metadata =
			easelcomm_find_remote_message(
				service, msg_metadata->msg->desc.in_reply_to);

		if (!orig_msg_metadata) {
			dev_dbg(easelcomm_miscdev.this_device,
				"msg %u:l%llu replied-to msg r%llu not found\n",
				service->service_id, msg_desc->message_id,
				msg_metadata->msg->desc.in_reply_to);
		} else {
			dev_dbg(easelcomm_miscdev.this_device,
				"msg %u:l%llu is reply to msg r%llu\n",
				service->service_id, msg_desc->message_id,
				msg_metadata->msg->desc.in_reply_to);
			easelcomm_drop_reference(
				service, orig_msg_metadata, true);
		}
	}
	easelcomm_drop_reference(service, msg_metadata, false);
	return 0;

out_freedesc:
	kfree(msg_desc);
	return ret;
}

/* Handle RECVMSG ioctl / easelcomm receiveMessage() call. */
static int easelcomm_wait_message(
	struct easelcomm_service *service,
	struct easelcomm_kmsg_desc __user *msgp)
{
	struct easelcomm_message_metadata *msg_metadata = NULL;
	struct easelcomm_kmsg_desc msg_temp;
	int ret = 0;
	bool has_timeout = false;
	long remaining = 0;

	dev_dbg(easelcomm_miscdev.this_device, "WAITMSG svc %u\n",
		service->service_id);

	if (copy_from_user(&msg_temp, (void __user *) msgp,
				sizeof(struct easelcomm_kmsg_desc)))
		return -EFAULT;

	dev_dbg(easelcomm_miscdev.this_device,
		"%s: timeout_ms=%d\n", __func__, msg_temp.wait.timeout_ms);

	if (msg_temp.wait.timeout_ms >= 0) {
		has_timeout = true;
		remaining = (long)msecs_to_jiffies(
				(unsigned int)msg_temp.wait.timeout_ms);
		dev_dbg(easelcomm_miscdev.this_device, "%s: remaining=%ld\n",
			__func__, remaining);
	}

	while (!msg_metadata) {
		spin_lock(&service->lock);
		if (service->shutdown_local ||
		    (easelcomm_is_client() && service->shutdown_remote)) {
			/* Service shutdown initiated locally or remotely. */
			dev_dbg(easelcomm_miscdev.this_device,
				"WAITMSG svc %u returning shutdown local=%d remote=%d\n",
				service->service_id, service->shutdown_local,
				service->shutdown_remote);
			service->shutdown_remote = false;
			spin_unlock(&service->lock);
			return -ESHUTDOWN;
		}
		/* grab first entry off receiveMessage queue. */
		msg_metadata = list_first_entry_or_null(
			&service->receivemsg_queue,
			struct easelcomm_message_metadata, rcvq_list);
		if (msg_metadata)
			break;
		spin_unlock(&service->lock);
		if (has_timeout) {
			remaining = wait_for_completion_interruptible_timeout(
					&service->receivemsg_queue_new,
					(unsigned long)remaining);
			if (remaining == 0)
				ret = -ETIMEDOUT;
			if (remaining < 0)
				ret = remaining;
		} else {
			ret = wait_for_completion_interruptible(
				&service->receivemsg_queue_new);
		}
		if (ret)
			return ret;
	}

	dev_dbg(easelcomm_miscdev.this_device,
		"WAITMSG svc %u returning msg %u:r%llu\n",
		service->service_id, service->service_id,
		msg_metadata->msg->desc.message_id);
	/* remove from receive queue, grab a ref while copy to user */
	list_del(&msg_metadata->rcvq_list);
	msg_metadata->queued = false;
	msg_metadata->reference_count++;
	spin_unlock(&service->lock);
	/* Copy the message desc to the caller */
	if (copy_to_user(msgp, msg_metadata->msg,
				sizeof(struct easelcomm_kmsg_desc)))
		ret = -EFAULT;

	easelcomm_drop_reference(service, msg_metadata, false);
	return ret;
}

/*
 * Handle WAITREPLY ioctl / reply waiting part of easelcomm
 * sendMessageReceiveReply() call.
 */
static int easelcomm_wait_reply(
	struct easelcomm_service *service,
	struct easelcomm_kmsg_desc __user *pmsg_desc)
{
	struct easelcomm_kmsg_desc orig_msg_desc;
	struct easelcomm_message_metadata *orig_msg_metadata;
	struct easelcomm_message_metadata *msg_metadata = NULL;
	int ret;
	bool has_timeout = false;
	long remaining = 0;

	if (copy_from_user(&orig_msg_desc, (void __user *) pmsg_desc,
				sizeof(struct easelcomm_kmsg_desc)))
		return -EFAULT;

	dev_dbg(easelcomm_miscdev.this_device, "%s: timeout_ms=%d\n",
		__func__, orig_msg_desc.wait.timeout_ms);

	if (orig_msg_desc.wait.timeout_ms >= 0) {
		has_timeout = true;
		remaining = (long)msecs_to_jiffies(
				(unsigned int)orig_msg_desc.wait.timeout_ms);
		dev_dbg(easelcomm_miscdev.this_device, "%s: remaining=%ld\n",
			__func__, remaining);
	}

	/* Grab a reference to the original message. */
	orig_msg_metadata = easelcomm_find_local_message(
		service, orig_msg_desc.message_id);
	if (!orig_msg_metadata) {
		dev_dbg(easelcomm_miscdev.this_device,
			"WAITREPLY msg %u:l%llu not found\n",
			service->service_id, orig_msg_desc.message_id);
		return -EINVAL;
	}

	dev_dbg(easelcomm_miscdev.this_device,
		"WAITREPLY msg %u:l%llu\n",
		service->service_id, orig_msg_desc.message_id);

	if (has_timeout) {
		remaining = wait_for_completion_interruptible_timeout(
				&orig_msg_metadata->reply_received,
				(unsigned long)remaining);
		if (remaining == 0)
			ret = -ETIMEDOUT;
		if (remaining < 0)
			ret = remaining;
	} else {
		ret = wait_for_completion_interruptible(
			&orig_msg_metadata->reply_received);
	}
	if (ret)
		goto out;
	if (orig_msg_metadata->flushing) {
		dev_dbg(easelcomm_miscdev.this_device,
			"WAITREPLY msg %u:l%llu flushed\n",
			service->service_id, orig_msg_desc.message_id);
		goto out;
	}

	msg_metadata = easelcomm_grab_reply_message(
		service, orig_msg_metadata);

	/*
	 * If completion was waken up on receiving the reply,
	 * msg_metadata really shouldn't be null.
	 * If completion was waken up on receiving shutdown, we should
	 * return to user immediately.
	 */
	if (!msg_metadata) {
		dev_err(easelcomm_miscdev.this_device,
			"WAITREPLY msg %u:l%llu no matching reply\n",
			service->service_id, orig_msg_desc.message_id);
		/*
		 * Here we return -ESHUTDOWN.  See above comment and
		 * easelcomm_handle_service_shutdown().
		 */
		ret = -ESHUTDOWN;
		goto out;
	}

	/* Copy the reply message descriptor to the caller */
	if (copy_to_user(pmsg_desc, &msg_metadata->msg->desc,
				sizeof(struct easelcomm_kmsg_desc))) {
		ret = -EFAULT;
		goto out;
	}
	dev_dbg(easelcomm_miscdev.this_device,
		"WAITREPLY msg %u:l%llu returning msg r%llu\n",
		service->service_id, orig_msg_desc.message_id,
		msg_metadata->msg->desc.message_id);
	ret = 0;

out:
	/* Discard reply message if error retrieving it. */
	if (msg_metadata)
		easelcomm_drop_reference(service, msg_metadata, ret);
	/* Always discard the original message, we're done with it. */
	easelcomm_drop_reference(service, orig_msg_metadata, true);
	return ret;
}

/* Initiate a flush of service on both sides of the link. */
static void easelcomm_flush_service(struct easelcomm_service *service)
{
	int ret;

	easelcomm_flush_local_service(service);
	dev_dbg(easelcomm_miscdev.this_device,
		"send cmd FLUSH_SERVICE svc %u\n",
		service->service_id);
	ret = easelcomm_send_cmd_noargs(service, EASELCOMM_CMD_FLUSH_SERVICE);
	if (ret)
		return;
	ret = wait_for_completion_interruptible_timeout(
			&service->flush_done,
			msecs_to_jiffies(SERVICE_FLUSH_DONE_TIMEOUT));
	if (ret <= 0)
		dev_err(easelcomm_miscdev.this_device,
			"service flush done wait aborted svc %u\n",
			service->service_id);
}

/* Handle REGISTER ioctl, register an open fd for an Easel service. */
static int easelcomm_register(struct easelcomm_user_state *user_state,
			unsigned int service_id) {
	struct easelcomm_service *service;

	if (service_id >= EASELCOMM_SERVICE_COUNT)
		return -EINVAL;

	mutex_lock(&service_mutex);
	service = easelcomm_service_list[service_id];
	if (service == NULL) {
		service = easelcomm_create_service(service_id);
		if (service == NULL) {
			mutex_unlock(&service_mutex);
			dev_err(easelcomm_miscdev.this_device,
					"could not create service %u\n",
					service_id);
			return -ENOMEM;
		}
	}
	mutex_unlock(&service_mutex);

	spin_lock(&service->lock);
	if (service->user) {
		spin_unlock(&service->lock);
		return -EBUSY;
	}
	user_state->service = service;
	service->user = user_state;
	service->shutdown_local = false;
	service->shutdown_remote = false;
	spin_unlock(&service->lock);
	dev_dbg(easelcomm_miscdev.this_device, "REGISTER svc %u\n",
		service_id);

	/*
	 * New client flushes any messages generated from activity by previous
	 * clients.
	 *
	 * New server handles any client messages that were waiting for the
	 * server to startup and process, does not flush at start/restart.
	 * Server can explicitly call the flush ioctl if wanted, but with the
	 * understanding any clients starting at the same time may have their
	 * traffic dropped.
	 */
	if (easelcomm_is_client())
		easelcomm_flush_service(service);
	return 0;
}

/*
 * Handle READDATA ioctl, the read message data part of easelcomm
 * receiveMessage() and sendMessageReceiveReply() calls.
 */
static int easelcomm_read_msgdata(
	struct easelcomm_service *service,
	struct easelcomm_kbuf_desc *buf_desc)
{
	struct easelcomm_message_metadata *msg_metadata;
	int ret = 0;

	dev_dbg(easelcomm_miscdev.this_device,
		"READDATA msg %u:r%llu buf=%p\n",
		service->service_id, buf_desc->message_id, buf_desc->buf);
	msg_metadata =
		easelcomm_find_remote_message(service, buf_desc->message_id);
	if (!msg_metadata) {
		dev_err(easelcomm_miscdev.this_device,
			"READDATA msg %u:r%llu not found\n",
			service->service_id, buf_desc->message_id);
		return -EINVAL;
	}

	if (buf_desc->buf_size &&
		buf_desc->buf_size != msg_metadata->msg->desc.message_size) {
		dev_err(easelcomm_miscdev.this_device,
			"READDATA descriptor buffer size %u doesn't match message %u:r%llu size %u\n",
			buf_desc->buf_size, service->service_id,
			buf_desc->message_id,
			msg_metadata->msg->desc.message_size);
		ret = -EINVAL;
		goto out;
	}

	if (buf_desc->buf_size) {
		if (copy_to_user(buf_desc->buf,
					&msg_metadata->msg->message_data,
					buf_desc->buf_size)) {
			ret = -EFAULT;
			goto out;
		}
	}

	/*
	 * If no DMA transfer and no reply needed then we're done with this
	 * message.
	 */
	if (!msg_metadata->msg->desc.dma_buf_size &&
		!msg_metadata->msg->desc.need_reply) {
		easelcomm_drop_reference(service, msg_metadata, true);
		return 0;
	}

out:
	easelcomm_drop_reference(service, msg_metadata, false);
	return ret;
}

/*
 * Handle WRITEDATA ioctl, the message data write portion of easelcomm
 * sendMessage*() and sendReply() calls.
 */
static int easelcomm_write_msgdata(
	struct easelcomm_service *service, struct easelcomm_kbuf_desc *buf_desc)
{
	struct easelcomm_message_metadata *msg_metadata;
	char *databuf = NULL;
	bool discard_message = true;
	int ret;

	dev_dbg(easelcomm_miscdev.this_device,
		"WRITEDATA msg %u:l%llu buf=%p\n",
		service->service_id, buf_desc->message_id, buf_desc->buf);

	msg_metadata =
		easelcomm_find_local_message(service, buf_desc->message_id);
	if (!msg_metadata) {
		dev_err(easelcomm_miscdev.this_device,
			"WRITEDATA msg %u:l%llu not found\n",
			service->service_id, buf_desc->message_id);
		return -EINVAL;
	}

	if (buf_desc->buf_size &&
	    buf_desc->buf_size != msg_metadata->msg->desc.message_size) {
		dev_err(easelcomm_miscdev.this_device,
			"WRITEDATA descriptor buffer size %u doesn't match message %u:l%llu size %u\n",
			buf_desc->buf_size, service->service_id,
			buf_desc->message_id,
			msg_metadata->msg->desc.message_size);
		ret = -EINVAL;
		goto out;
	}

	if (buf_desc->buf_size) {
		databuf = kmalloc(buf_desc->buf_size, GFP_KERNEL);
		if (!databuf) {
			ret = -ENOMEM;
			goto out;
		}

		if (copy_from_user(
				databuf, buf_desc->buf, buf_desc->buf_size)) {
			ret = -EFAULT;
			goto out;
		}
	}

	dev_dbg(easelcomm_miscdev.this_device,
		"send cmd SEND_MSG msg %u:l%llu\n",
		service->service_id, msg_metadata->msg->desc.message_id);
	ret = easelcomm_start_cmd(
		service, EASELCOMM_CMD_SEND_MSG,
		sizeof(struct easelcomm_kmsg_desc) + buf_desc->buf_size);
	if (ret)
		goto out;
	ret = easelcomm_append_cmd_args(
		service, &msg_metadata->msg->desc,
		sizeof(struct easelcomm_kmsg_desc));
	if (ret)
		goto out;
	if (buf_desc->buf_size) {
		ret = easelcomm_append_cmd_args(
			service, databuf, buf_desc->buf_size);
		if (ret)
			goto out;
	}
	ret = easelcomm_send_cmd(service);
	if (ret)
		goto out;

	/*
	 * If no DMA transfer and no reply needed then we're done with this
	 * message.
	 */
	discard_message = !msg_metadata->msg->desc.dma_buf_size &&
		!msg_metadata->msg->desc.need_reply;

out:
	/*
	 * If success and no DMA transfer and no reply needed, or if error,
	 * then free the message.
	 */
	easelcomm_drop_reference(service, msg_metadata, discard_message);
	kfree(databuf);
	return ret;
}

/*
 * Client calls this upon receipt of BOOTSTRAP_READY IRQ
 * from server. Call MNH host driver to setup iATU mappings for the command
 * channels and send LINK_INIT.
 */
int easelcomm_client_remote_cmdchan_ready_handler(void)
{
	int ret;

	/* Sync up client and server sides using MNH host driver APIs */
	ret = easelcomm_hw_ap_setup_cmdchans();
	if (ret)
		return ret;
	easelcomm_cmd_channel_remote_set_ready();
	/* Client can now send link init to server */
	dev_dbg(easelcomm_miscdev.this_device, "send cmd LINK_INIT\n");
	return easelcomm_send_cmd_noargs(
		&link_service,
		EASELCOMM_CMD_LINK_INIT);
}
EXPORT_SYMBOL(easelcomm_client_remote_cmdchan_ready_handler);

/*
 * Handle ioctls that take a struct easelcomm_kbuf_desc * argument, convert
 * field layout and buf field pointer for 32-bit compat if needed.
 */
static int easelcomm_ioctl_kbuf_desc(
	struct easelcomm_service *service, unsigned int cmd,
	unsigned long arg, bool compat)
{
	struct easelcomm_kbuf_desc kbuf_desc;
	int ret;

	if (compat) {
#ifdef CONFIG_COMPAT
		struct easelcomm_compat_kbuf_desc compat_kbuf_desc;

		if (copy_from_user(
				&compat_kbuf_desc, (void __user *) arg,
				sizeof(struct easelcomm_compat_kbuf_desc)))
			return -EFAULT;

		kbuf_desc.message_id = compat_kbuf_desc.message_id;
		kbuf_desc.buf = compat_ptr(compat_kbuf_desc.buf);
		kbuf_desc.buf_size = compat_kbuf_desc.buf_size;
		kbuf_desc.buf_type = compat_kbuf_desc.buf_type;
		kbuf_desc.dma_buf_fd = compat_kbuf_desc.dma_buf_fd;
#else
		return -EINVAL;
#endif
	} else {
		if (copy_from_user(
				&kbuf_desc, (void __user *) arg,
				sizeof(struct easelcomm_kbuf_desc)))
			return -EFAULT;
	}

	switch (cmd) {
	case EASELCOMM_IOC_WRITEDATA:
		ret = easelcomm_write_msgdata(service, &kbuf_desc);
		break;
	case EASELCOMM_IOC_READDATA:
		ret = easelcomm_read_msgdata(service, &kbuf_desc);
		break;
	case EASELCOMM_IOC_SENDDMA:
		ret = easelcomm_send_dma(service, &kbuf_desc);
		break;
	case EASELCOMM_IOC_RECVDMA:
		ret = easelcomm_receive_dma(service, &kbuf_desc);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/* 32-bit and 64-bit userspace ioctl handling, dispatch. */
static long easelcomm_ioctl_common(
	struct file *file, unsigned int cmd, unsigned long arg, bool compat)
{
	int ret = 0;
	struct easelcomm_user_state *user_state = file->private_data;
	struct easelcomm_service *service;

	if (_IOC_TYPE(cmd) != EASELCOMM_IOC_MAGIC)
		return -EINVAL;

	if (WARN_ON(!user_state))
		return -EINVAL;

	if (!easelcomm_up)
		return -ESHUTDOWN;

	/* REGISTER is the only ioctl that doesn't need a service registered. */
	if (cmd == EASELCOMM_IOC_REGISTER)
		return easelcomm_register(user_state, (unsigned int) arg);

	service = user_state->service;
	if (!service) {
		dev_err(easelcomm_miscdev.this_device,
			"user has not registered a service for the file\n");
		return -EINVAL;
	}

	switch (cmd) {
	case EASELCOMM_IOC_SENDMSG:
		ret = easelcomm_send_message_ioctl(
			service, (struct easelcomm_kmsg_desc *)arg);
		break;
	case EASELCOMM_IOC_WRITEDATA:
	case EASELCOMM_IOC_READDATA:
	case EASELCOMM_IOC_SENDDMA:
	case EASELCOMM_IOC_RECVDMA:
		/* Convert 32-bit kbuf_desc argument if needed and dispatch. */
		ret = easelcomm_ioctl_kbuf_desc(service, cmd, arg, compat);
		break;
	case EASELCOMM_IOC_WAITMSG:
		ret = easelcomm_wait_message(
			service, (struct easelcomm_kmsg_desc *)arg);
		break;
	case EASELCOMM_IOC_WAITREPLY:
		ret = easelcomm_wait_reply(
			service, (struct easelcomm_kmsg_desc *)arg);
		break;
	case EASELCOMM_IOC_SHUTDOWN:
		easelcomm_initiate_service_shutdown(service);
		break;
	case EASELCOMM_IOC_FLUSH:
		easelcomm_flush_service(service);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

/* 64-bit ioctl handling. */
static long easelcomm_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg)
{
	return easelcomm_ioctl_common(file, cmd, arg, false);
}

#ifdef CONFIG_COMPAT
/* 32-bit ioctl handling. */
static long easelcomm_compat_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int cmd_nr = _IOC_NR(cmd);
	int ret;

	/* Fixup pointer argument size for ioctls 1..7 */
	if ((cmd_nr >= 1 && cmd_nr <= 7) &&
		_IOC_SIZE(cmd) == sizeof(compat_uptr_t)) {
		cmd &= ~(_IOC_SIZEMASK << _IOC_SIZESHIFT);
		cmd |= sizeof(void *) << _IOC_SIZESHIFT;
	}

	switch (cmd) {
	case EASELCOMM_IOC_SENDMSG:
	case EASELCOMM_IOC_WAITMSG:
	case EASELCOMM_IOC_WAITREPLY:
	case EASELCOMM_IOC_WRITEDATA:
	case EASELCOMM_IOC_READDATA:
	case EASELCOMM_IOC_SENDDMA:
	case EASELCOMM_IOC_RECVDMA:
		/* pointer argument, fixup */
		ret = easelcomm_ioctl_common(file, cmd,
					(unsigned long)compat_ptr(arg), true);
		break;

	case EASELCOMM_IOC_REGISTER:
	case EASELCOMM_IOC_SHUTDOWN:
	case EASELCOMM_IOC_FLUSH:
		/* no ioctl argument conversion needed */
		ret = easelcomm_ioctl_common(file, cmd, arg, true);
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif /* CONFIG_COMPAT */

/* Init local state for remote command channel. */
static void easelcomm_init_cmd_channel_remote(
	struct easelcomm_cmd_channel_remote *channel)
{
	mutex_init(&channel->mutex);
	channel->initialized = false;
	init_completion(&channel->init_state_changed);
	init_completion(&channel->wrap_ready);
	/* Write pointer starts after the channel header */
	channel->write_offset =
		(sizeof(struct easelcomm_cmd_channel_header) + 7) & ~0x7;
	channel->write_seqnbr = 0;
}

/* Init local command channel. */
static int easelcomm_init_cmd_channel_local(
	struct easelcomm_cmd_channel_local *channel, void *cmdchan_buffer)
{
	channel->buffer = cmdchan_buffer;
	if (!channel->buffer)
		return -ENOMEM;
	channel->readp = channel->buffer +
		sizeof(struct easelcomm_cmd_channel_header);

	/* next sequence # to be produced/consumed starts at zero */
	channel->consumer_seqnbr_next = 0;
	((struct easelcomm_cmd_channel_header *)
		channel->buffer)->producer_seqnbr_next = 0;
	return 0;
}

/*
 * Callback from h/w layer when PCIe ready (which is immediate for server,
 * happens on first PCIe host driver probe on client).	The h/w layer has
 * allocated the local command channel buffer (which needs the host PCIe
 * driver on client), the pointer to which is passed as an argument.  Can now
 * init our local command channel state, and do remote too.
 */
int easelcomm_init_pcie_ready(void *local_cmdchan_buffer)
{
	int ret;

	ret = easelcomm_init_cmd_channel_local(
		&cmd_channel_local, local_cmdchan_buffer);
	easelcomm_init_cmd_channel_remote(&cmd_channel_remote);
	return ret;
}
EXPORT_SYMBOL(easelcomm_init_pcie_ready);

/*
 * Callback for AP/client from h/w layer when PCIe EP is off.
 */
int easelcomm_pcie_hotplug_out(void)
{
	struct easelcomm_cmd_channel_remote *channel = &cmd_channel_remote;

	/*
	 * PCIe link is already down, so no need to call easelcomm_stop() to
	 * notify remote, but only call easelcomm_stop_local().
	 */
	easelcomm_stop_local();

	mutex_lock(&channel->mutex);
	channel->initialized = false;
	mutex_unlock(&channel->mutex);
	return 0;
}
EXPORT_SYMBOL(easelcomm_pcie_hotplug_out);

static int easelcomm_notify_sys(
	struct notifier_block *this, unsigned long code, void *unused)
{
	if (code == SYS_RESTART || code == SYS_HALT || code == SYS_POWER_OFF)
		easelcomm_stop(false);
	return NOTIFY_DONE;
}

static struct notifier_block easelcomm_notifier = {
	.notifier_call	= easelcomm_notify_sys,
};

static int __init easelcomm_init(void)
{
	int ret;
	int i;

	ret = misc_register(&easelcomm_miscdev);
	if (ret) {
		dev_err(easelcomm_miscdev.this_device,
			"misc_register failed\n");
		return ret;
	}
	dev_info(easelcomm_miscdev.this_device,
		"registered at misc device minor %d\n",
		easelcomm_miscdev.minor);

	mutex_init(&service_mutex);

	for (i = 0; i < EASELCOMM_SERVICE_COUNT; i++) {
		easelcomm_service_list[i] = NULL;
	}

	/* Initializes link_service. */
	easelcomm_init_service(&link_service, EASELCOMM_SERVICE_COUNT);

	easelcomm_hw_init();

	/* Shut down easelcomm on reboot */
	ret = register_reboot_notifier(&easelcomm_notifier);
	if (ret)
		dev_warn(easelcomm_miscdev.this_device,
			"register reboot notifier failed\n");

	return 0;
}

static void __exit easelcomm_exit(void)
{
	int i;
	easelcomm_stop(false);
	kfree(cmd_channel_local.buffer);
	misc_deregister(&easelcomm_miscdev);
	/*
	 * TODO(cjluo): Some of these service struct
	 * could be freed when unregistered.
	 */
	for (i = 0; i < EASELCOMM_SERVICE_COUNT; i++) {
		if (easelcomm_service_list[i] != NULL) {
			kfree(easelcomm_service_list[i]);
			easelcomm_service_list[i] = NULL;
		}
	}
}

module_init(easelcomm_init);
module_exit(easelcomm_exit);

MODULE_AUTHOR("Google Inc.");
MODULE_DESCRIPTION("Easel coprocessor communication driver");
MODULE_LICENSE("GPL");
