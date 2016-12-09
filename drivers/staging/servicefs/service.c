/*
 *  service.c - part of servicefs, a small filesystem for service namespaces.
 *
 *  Copyright (C) 2015 Corey Tabaka <eieio@google.com>
 *  Copyright (C) 2015 Google, Inc.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/file.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/poison.h>
#include <linux/poll.h>
#include <linux/namei.h>
#include <linux/atomic.h>
#include <uapi/linux/servicefs.h>

#include "servicefs_private.h"

/*
 * Lock ordering rules:
 *  1. s_mutex
 *  2. s_channel_lock OR m_mutex (never held together)
 *
 * The mutex s_mutex protects most members of struct service, except for the
 * channel list and channel id sweeper. This lock orders access to the message
 * queue and service context pointer.
 *
 * The mutex s_channel_lock protects the channel list, channel id sweeper, and
 * orders writes to per-channel context pointers. Reads from per-channel context
 * pointers are not synchronized with respect to writes and should be
 * synchronized in userspace in multithreaded services. This is deemed
 * acceptable becuase changing context pointers requires userspace
 * synchronization between setting the pointer and receiving messages,
 * regardless of whether the kernel synchronizes reads and updates internally.
 *
 * The mutex m_mutex protects most per-message members of struct message. This
 * lock serializes access to the message payload iovecs and pending fd list.
 * This lock also ensures that message APIs are serialized with respect to
 * service tear down.
 */

#define THIS_SERVICE_FD (-1)

/**
 * servicefs_atomic_update_mask - update a mask in an atomic variable
 * @v: pointer to atomic variable to update
 * @clr: bits to clear in the atomic variable
 * @set: bits to set in the atomic variable
 *
 * Atomically updates the bits in an atomic variable by first clearing
 * the bits given by |clr| and then setting the bits given by |set|.
 *
 * Returns the previous value of the mask.
 */
static inline int servicefs_atomic_update_mask(atomic_t *v, int clr, int set)
{
	int c, old;

	c = atomic_read(v);
	while ((old = atomic_cmpxchg(v, c, (c & ~clr) | set)) != c)
		c = old;

	return old;
}

#define servicefs_atomic_mask_set(v, set) \
	servicefs_atomic_update_mask((v), 0, (set))

#define servicefs_atomic_mask_clear(v, clr) \
	servicefs_atomic_update_mask((v), (clr), 0)

static inline bool is_close_message(struct message *m)
{
	return m->m_task == NULL && m->m_op == SERVICEFS_OP_UNIX_CLOSE;
}

/**
 * servicefs_service_cache - cache for service structs.
 */
static struct kmem_cache *servicefs_service_cache;

/**
 * servicefs_channel_cache - cache for channel structs.
 */
static struct kmem_cache *servicefs_channel_cache;

/**
 * servicefs_message_cache - cache for message structs.
 */
static struct kmem_cache *servicefs_message_cache;

/**
 * servicefs_impulse_cache - cache for impulse structs.
 */
static struct kmem_cache *servicefs_impulse_cache;

/**
 * servicefs_pending_fd_cache - cache for pending_fd structs.
 */
static struct kmem_cache *servicefs_pending_fd_cache;

int servicefs_cache_init(void)
{
	unsigned long flags = 0;
#ifdef CONFIG_SERVICEFS_USE_POISON
	flags |= SLAB_POISON;
#endif

	servicefs_service_cache = KMEM_CACHE(service, flags);
	if (!servicefs_service_cache)
		goto error;

	servicefs_channel_cache = KMEM_CACHE(channel, flags);
	if (!servicefs_channel_cache)
		goto error;

	servicefs_message_cache = KMEM_CACHE(message, flags);
	if (!servicefs_message_cache)
		goto error;

	servicefs_impulse_cache = KMEM_CACHE(impulse, flags);
	if (!servicefs_impulse_cache)
		goto error;

	servicefs_pending_fd_cache = KMEM_CACHE(pending_fd, flags);
	if (!servicefs_pending_fd_cache)
		goto error;

	return 0;

error:
	if (servicefs_service_cache) {
		kmem_cache_destroy(servicefs_service_cache);
		servicefs_service_cache = NULL;
	}
	if (servicefs_channel_cache) {
		kmem_cache_destroy(servicefs_channel_cache);
		servicefs_channel_cache = NULL;
	}
	if (servicefs_message_cache) {
		kmem_cache_destroy(servicefs_message_cache);
		servicefs_message_cache = NULL;
	}
	if (servicefs_impulse_cache) {
		kmem_cache_destroy(servicefs_impulse_cache);
		servicefs_impulse_cache = NULL;
	}
	if (servicefs_pending_fd_cache) {
		kmem_cache_destroy(servicefs_pending_fd_cache);
		servicefs_pending_fd_cache = NULL;
	}

	return -ENOMEM;
}

/*
 * Forward declarations.
 */
static struct message *message_new(struct service *svc, struct channel *c,
		int op, const struct iovec *svec, size_t scnt,
		const struct iovec *rvec, size_t rcnt, const __s32 *fds, size_t fdcnt,
		struct task_struct *task);
static void __message_get(struct message *msg);
static void __message_put(struct message *msg);
static void __message_cancel(struct message *msg);
static void __message_complete(struct message *msg, int retcode);
static void __impulse_cancel(struct impulse *impulse);
static void channel_put(struct channel *c);
static bool channel_cancel(struct channel *c);

static void channel_free(struct channel *c)
{
	kmem_cache_free(servicefs_channel_cache, c);
}

static void message_free(struct message *m)
{
	pr_debug("free: msg=%p m_op=%d cid=%d\n", m, m->m_op, m->m_channel->c_id);

	if (m->m_task)
		put_task_struct(m->m_task);
	channel_put(m->m_channel);
	kmem_cache_free(servicefs_message_cache, m);
}

static void impulse_free(struct impulse *i)
{
	pr_debug("i=%p cid=%d\n", i, i->i_channel->c_id);

	channel_put(i->i_channel);
	kmem_cache_free(servicefs_impulse_cache, i);
}

static void pending_fd_free(struct pending_fd *pfd)
{
	kmem_cache_free(servicefs_pending_fd_cache, pfd);
}

/**
 * service_free - free an unused service struct
 * @svc: pointer to the service struct to free
 *
 * Performs sanity checks to make sure the service is properly torn down before
 * freeing its memory.
 */
void service_free(struct service *svc)
{
	pr_debug("freeing service: svc=%p\n", svc);

	/* make sure everything is already properly cleaned up */
	BUG_ON(!list_empty(&svc->s_impulses));
	BUG_ON(!list_empty(&svc->s_messages));
	BUG_ON(waitqueue_active(&svc->s_wqreceivers));
	BUG_ON(waitqueue_active(&svc->s_wqselect));

	/* free allocators */
	idr_destroy(&svc->s_channel_idr);
	idr_destroy(&svc->s_message_idr);

	kmem_cache_free(servicefs_service_cache, svc);
}

/**
 * service_new - allocate and initialize a new service
 *
 * Returns a pointer to the new service.
 */
struct service *service_new(void)
{
	struct service *svc = kmem_cache_alloc(servicefs_service_cache, GFP_KERNEL);
	if (!svc) {
		pr_err("failed to allocate service struct\n");
		goto out;
	}

	pr_debug("creating new service: svc=%p\n", svc);

	svc->s_count = (atomic_t) ATOMIC_INIT(0);

	mutex_init(&svc->s_mutex);
	mutex_init(&svc->s_channel_lock);

	idr_init(&svc->s_message_idr);
	idr_init(&svc->s_channel_idr);

	// Disable RCU free.
	svc->s_message_idr.no_rcu_free = true;
	svc->s_channel_idr.no_rcu_free = true;

	svc->s_message_start = 0;
	svc->s_channel_start = 0;

	INIT_LIST_HEAD(&svc->s_impulses);
	INIT_LIST_HEAD(&svc->s_messages);

	init_waitqueue_head(&svc->s_wqreceivers);
	init_waitqueue_head(&svc->s_wqselect);

	svc->s_flags = (atomic_t) ATOMIC_INIT(SERVICE_FLAGS_DEFAULT);

	svc->s_context = 0;
	svc->s_filp = NULL;

out:
	return svc;
}

int service_cancel(struct service *svc)
{
	int ret;
	struct channel *c;
	struct impulse *i, *in;
	struct message *m, *mn;
	int cid, mid;
	int previous_mask;

	previous_mask = servicefs_atomic_mask_set(&svc->s_flags,
			SERVICE_FLAGS_CANCELED);

	if ((previous_mask & SERVICE_FLAGS_CANCELED) == 0) {
		pr_debug("canceling service=%p\n", svc);

		pr_debug("removing dentry service=%p\n", svc);
		servicefs_remove_dentry(svc->s_filp->f_path.dentry);

		mutex_lock(&svc->s_mutex);
		list_for_each_entry_safe(i, in, &svc->s_impulses, i_impulses_node) {
			__impulse_cancel(i);
		}
		list_for_each_entry_safe(m, mn, &svc->s_messages, m_messages_node) {
			__message_cancel(m);
		}
		idr_for_each_entry(&svc->s_message_idr, m, mid) {
			__message_cancel(m);
		}
		mutex_unlock(&svc->s_mutex);

		mutex_lock(&svc->s_channel_lock);
		idr_for_each_entry(&svc->s_channel_idr, c, cid) {
			channel_cancel(c);
		}
		mutex_unlock(&svc->s_channel_lock);

		wake_up_all(&svc->s_wqreceivers);
		wake_up_poll(&svc->s_wqselect, POLLHUP | POLLFREE);

		ret = 0;
	} else {
		pr_debug("already canceled service=%p\n", svc);
		ret = -ESHUTDOWN;
	}

	return ret;
}

/**
 * alloc_channel_id - allocate an unused channel id for a channel
 * @c: a pointer to the channel to associate the id with.
 *
 * This function allocates an id for a channel on its service's channel id
 * allocator. The given channel's c_id member is filled in with the id, and
 * the channel pointer is associated with the id, for future lookup.
 *
 * The c_service member of the channel struct must be set to the associated
 * service pointer prior to calling this function.
 */
static int alloc_channel_id(struct channel *c)
{
	int ret;
	struct service *svc = c->c_service;
	BUG_ON(svc == NULL);

	mutex_lock(&svc->s_channel_lock);

	ret = idr_alloc(&svc->s_channel_idr, c, svc->s_channel_start, -1, GFP_KERNEL);
	if (ret < 0) {
		mutex_unlock(&svc->s_channel_lock);
		return ret;
	} else {
		svc->s_channel_start = (svc->s_channel_start + 1) & 0x7fffffff;
	}

	c->c_id = ret;
	mutex_unlock(&svc->s_channel_lock);
	return 0;
}

/**
 * free_channel_id - free the channel id for a channel
 * @c: a pointer to the channel to free the id for.
 */
static void free_channel_id(struct channel *c)
{
	struct service *svc = c->c_service;
	BUG_ON(svc == NULL);

	mutex_lock(&svc->s_channel_lock);
	idr_remove(&svc->s_channel_idr, c->c_id);
	mutex_unlock(&svc->s_channel_lock);
}

/**
 * alloc_message_id - allocates an unused message for a message
 * @svc: pointer to the service to allocate the message id on.
 * @m: pointer to the message to associate the id with.
 *
 * This function allocates an id for a message on its service's message id
 * allocator. The given message's m_id member is filled in with the id and
 * the message pointer is associated with the id for future lookup.
 *
 * The message's m_channel member must be set before calling this function.
 *
 * This function must be called with the service's s_mutex held.
 */
static int alloc_message_id(struct message *m)
{
	int ret;
	struct service *svc = m->m_channel->c_service;
	BUG_ON(svc == NULL);

	lockdep_assert_held(&svc->s_mutex);

	ret = idr_alloc(&svc->s_message_idr, m, svc->s_message_start, -1, GFP_KERNEL);
	if (ret < 0)
		return ret;
	else
		svc->s_message_start = (svc->s_message_start + 1) & 0x7fffffff;

	m->m_id = ret;
	return 0;
}

/**
 * free_message_id - free the message id for a message
 * @m: a pointer to the message to free the id for.
 *
 * This function must be called with the service's s_mutex held.
 */
static void free_message_id(struct message *m)
{
	struct service *svc = m->m_channel->c_service;
	BUG_ON(svc == NULL);

	lockdep_assert_held(&svc->s_mutex);

	if (m->m_id != MESSAGE_NO_ID) {
		idr_remove(&svc->s_message_idr, m->m_id);
		m->m_id = MESSAGE_NO_ID;
	}
}

/**
 * channel_new - allocates a new channel to a service
 * @svc: a pointer to the service to allocate the channel for.
 *
 * This function allocates a new channel struct and associates it with the
 * given service. It handles allocating an id for the channel and adding
 * it to the list of open channels.
 */
struct channel *channel_new(struct service *svc)
{
	int ret;
	struct channel *c = kmem_cache_alloc(servicefs_channel_cache, GFP_KERNEL);
	if (!c)
		return NULL;

	kref_init(&c->c_ref);
	c->c_service = svc;
	c->c_context = 0;
	c->c_flags = (atomic_t) ATOMIC_INIT(0);
	c->c_events = (atomic_t) ATOMIC_INIT(0);
	init_waitqueue_head(&c->c_waitqueue);

	ret = alloc_channel_id(c);
	if (ret) {
		channel_free(c);
		return NULL;
	} else {
		pr_debug("added channel id=%d to service=%p\n", c->c_id, svc);
		return c;
	}
}

static void channel_release_kref(struct kref *ref)
{
	struct channel *c = container_of(ref, struct channel, c_ref);

	pr_debug("c=%p c_id=%d\n", c, c->c_id);

	free_channel_id(c);
	BUG_ON(waitqueue_active(&c->c_waitqueue));
	channel_free(c);
}

static void channel_put(struct channel *c)
{
	kref_put(&c->c_ref, channel_release_kref);
}

static bool channel_get(struct channel *c)
{
	return !!kref_get_unless_zero(&c->c_ref);
}

/**
 * channel_cancel - cancel a channel and wake up clients
 * @c: pointer to the channel to cancel
 *
 * Puts the channel into canceled state and wakes up any
 * clients that are blocked waiting for channel events.
 * POLLHUP is guaranteed by the kernel to always be in the
 * client event set, and is used to signal that the file
 * descriptor is in a canceled state.
 *
 * Returns true if this call canceled the channel, false if the
 * channel was already canceled.
 */
static bool channel_cancel(struct channel *c)
{
	int previous_mask;
	struct service *svc = c->c_service;
	BUG_ON(!svc);

	previous_mask = servicefs_atomic_mask_set(&c->c_flags,
			CHANNEL_FLAGS_CANCELED);

	if ((previous_mask & CHANNEL_FLAGS_CANCELED) == 0) {
		servicefs_atomic_mask_set(&c->c_events, POLLHUP | POLLFREE);
		wake_up_poll(&c->c_waitqueue, POLLHUP | POLLFREE);
		return true;
	} else {
		return false;
	}
}

/*
 * channel_remove - remove a channel from a service
 * @c: pointer to the channel to remove from its service
 *
 * This should only be called by client_release() when the
 * file object associated with this channel is being
 * released.
 */
void channel_remove(struct channel *c)
{
	struct service *svc = c->c_service;
	BUG_ON(!svc);

	pr_debug("removing channel id=%d from service=%p\n", c->c_id, c->c_service);

	if (channel_cancel(c)) {
		/*
		 * Generate a special close message to signal to the service that the
		 * channel has closed. This message is only sent if the channel is not
		 * already canceled by the service explicitly closing it or the service
		 * shutting down.
		 */
		struct message *msg = message_new(svc, c, SERVICEFS_OP_UNIX_CLOSE,
				NULL, 0, NULL, 0, NULL, 0, NULL);
		if (!msg) {
			pr_err("Failed to allocate close message!\n");
			goto done;
		}

		/* Enqueue the close message unless the service is canceled. */
		mutex_lock(&svc->s_mutex);

		if (is_service_canceled(svc)) {
			pr_debug("cid=%d service_canceled=%d\n", c->c_id,
					is_service_canceled(svc));
			__message_put(msg);
		} else {
			list_add_tail(&msg->m_messages_node, &svc->s_messages);

			if (waitqueue_active(&svc->s_wqreceivers))
				wake_up(&svc->s_wqreceivers);
			else
				wake_up_poll(&svc->s_wqselect, POLLIN | POLLRDNORM);
		}

		mutex_unlock(&svc->s_mutex);
	}

done:
	/* Drop the channel ref. The close message maintains its own ref. */
	channel_put(c);
}

/**
 * __message_cancel - cancel a pending message
 * @msg: pointer to the message to cancel
 *
 * Completes a message with a return code of -ESHUTDOWN.
 *
 * Must be called with the service's s_mutex held.
 */
static void __message_cancel(struct message *msg)
{
	pr_debug("msg=%p\n", msg);
	__message_complete(msg, -ESHUTDOWN);
}

static void __impulse_cancel(struct impulse *impulse)
{
	list_del_init(&impulse->i_impulses_node);
	impulse_free(impulse);
}

static struct channel *__lookup_channel(struct service *svc, int cid)
{
	return idr_find(&svc->s_channel_idr, cid);
}

static struct message *message_new(struct service *svc, struct channel *c,
		int op, const struct iovec *svec, size_t scnt,
		const struct iovec *rvec, size_t rcnt, const __s32 *fds, size_t fdcnt,
		struct task_struct *task)
{
	kuid_t kuid;
	kgid_t kgid;
	struct message *msg = kmem_cache_alloc(
			servicefs_message_cache, GFP_KERNEL);
	if (!msg)
		return NULL;

	INIT_LIST_HEAD(&msg->m_messages_node);
	INIT_LIST_HEAD(&msg->m_pending_fds);
	kref_init(&msg->m_ref);
	mutex_init(&msg->m_mutex);

	msg->m_id = MESSAGE_NO_ID;
	msg->m_op = op;
	msg->m_flags = (atomic_t) ATOMIC_INIT(MESSAGE_FLAGS_PENDING);

	init_waitqueue_head(&msg->m_waitqueue);

	/*
	 * Get a ref to the channel. This ref is dropped when the message is freed.
	 */
	channel_get(c);
	msg->m_channel = c;
	msg->m_priority = task ? task_nice(task) : -1;
	msg->m_status = 0;

	iov_buffer_uinit(&msg->m_sbuf, svec, scnt);
	iov_buffer_uinit(&msg->m_rbuf, rvec, rcnt);

	msg->m_fds = fds;
	msg->m_fdcnt = fdcnt;

	/*
	 * Get a ref to the task and stash the task struct for later use. The task
	 * struct ref is dropped when the message is freed.
	 */
	if (task)
		get_task_struct(task);
	msg->m_task = task;

	/* Credentials always come from the current task. */
	current_euid_egid(&kuid, &kgid);
	msg->m_euid = __kuid_val(kuid);
	msg->m_egid = __kgid_val(kgid);
	msg->m_pid = pid_vnr(task_tgid(current));
	msg->m_tid = pid_vnr(task_pid(current));

	return msg;
}

static struct message *__lookup_message(struct service *svc, int mid)
{
	return idr_find(&svc->s_message_idr, mid);
}

/**
 * lookup_and_lock_message - looks up a message, acquiring its m_mutex
 * @svc: pointer to the service in which to look up the message
 * @mid: message id of the message to lookup
 * @check_interrupted: whether to check the interrupted status of the message
 *  and bail out, leaving the message unlocked
 * Returns a message on success, with its m_mutex acquired. On error
 * an error code is returned as an error pointer.
 *
 * This function properly interlocks the release of svc->s_mutex with
 * the acquisition of the message's m_mutex.
 */
static struct message *lookup_and_lock_message(struct service *svc, int mid,
		bool check_interrupted)
{
	struct message *msg;

	mutex_lock(&svc->s_mutex);

	msg = __lookup_message(svc, mid);
	if (!msg) {
		mutex_unlock(&svc->s_mutex);
		return ERR_PTR(-ENOENT);
	}

	mutex_lock(&msg->m_mutex);

	if (check_interrupted && is_message_interrupted(msg)) {
		mutex_unlock(&msg->m_mutex);
		mutex_unlock(&svc->s_mutex);
		return ERR_PTR(-EINTR);
	}

	// unlock s_mutex, leaving m_mutex locked
	mutex_unlock(&svc->s_mutex);
	return msg;
}

static struct message *__peek_message(struct service *svc)
{
	lockdep_assert_held(&svc->s_mutex);
	return list_first_entry_or_null(&svc->s_messages, struct message, m_messages_node);
}

static int __activate_message(struct message *msg)
{
	int ret;
	struct service *svc = msg->m_channel->c_service;
	BUG_ON(svc == NULL);

	lockdep_assert_held(&svc->s_mutex);

	ret = alloc_message_id(msg);
	if (ret)
		goto error;

	list_del_init(&msg->m_messages_node);

	/* If the message is a close message inherit the ref. */
	if (!is_close_message(msg))
		__message_get(msg);

error:
	return ret;
}

struct impulse *__dequeue_impulse(struct service *svc)
{
	struct impulse *impulse;
	lockdep_assert_held(&svc->s_mutex);
	impulse = list_first_entry_or_null(&svc->s_impulses, struct impulse, i_impulses_node);
	if (impulse)
		list_del_init(&impulse->i_impulses_node);
	return impulse;
}

/**
 * message_release_kref - clean up a message after the last kref is dropped
 * @ref: pointer to the messages m_ref member
 *
 * Must be called with the service's s_mutex held.
 */
static void message_release_kref(struct kref *ref)
{
	struct message *msg = container_of(ref, struct message, m_ref);

	pr_debug("release: msg=%p m_id=%d m_op=%d m_flags=0x%x m_status=%zd\n", msg,
			msg->m_id, msg->m_op, atomic_read(&msg->m_flags), msg->m_status);

	/* remove the message in case it's queued */
	list_del_init(&msg->m_messages_node);
	message_free(msg);
}

/**
 * __message_put - drop a reference to a message
 * @msg: pointer to the message to drop the reference to
 *
 * Must be called with the service's s_mutex held.
 */
static void __message_put(struct message *msg)
{
	kref_put(&msg->m_ref, message_release_kref);
}

static void message_put(struct service *svc, struct message *msg)
{
	mutex_lock(&svc->s_mutex);
	__message_put(msg);
	mutex_unlock(&svc->s_mutex);
}

/**
 * __message_get - get a reference to a message
 * @msg: pointer to the message to get a reference to
 *
 * Must be called with the service's s_mutex held.
 */
static void __message_get(struct message *msg)
{
	BUG_ON(!kref_get_unless_zero(&msg->m_ref));
}

/**
 * __message_complete - complete a message and wake up the sender
 * @msg: pointer to the message to complete
 * @retcode: the return code to return to the sender
 *
 * Must be called with the service's s_mutex held.
 */
static void __message_complete(struct message *msg, int retcode)
{
	struct pending_fd *pfd, *pfdn;

	/*
	 * An active message has a ref taken by msg_recv and similarly a close
	 * message is created with a ref isn't owned by the sender. In both cases
	 * this ref must be dropped when we are done here.
	 */
	bool drop_ref = is_message_active(msg) || is_close_message(msg);

	free_message_id(msg);
	list_del_init(&msg->m_messages_node);

	mutex_lock(&msg->m_mutex);

	/* Install pending fds or roll back fd allocation depending on retcode. */
	list_for_each_entry_safe(pfd, pfdn,
			&msg->m_pending_fds, p_pending_fds_node) {
		if (retcode >= 0) {
			servicefs_fd_install(msg->m_task, pfd->p_fd, pfd->p_filp);
		} else {
			servicefs_put_unused_fd(msg->m_task, pfd->p_fd);
			fput(pfd->p_filp);
		}

		list_del(&pfd->p_pending_fds_node);
		pending_fd_free(pfd);
	}

	msg->m_status = retcode;
	servicefs_atomic_mask_set(&msg->m_flags, MESSAGE_FLAGS_COMPLETED);

	mutex_unlock(&msg->m_mutex);

	wake_up(&msg->m_waitqueue);

	if (drop_ref)
		__message_put(msg);
}

/**
 * servicefs_check_channel - check if an fd in a message is a channel
 * @svc: pointer to the service owning the message
 * @svcfd: fd of the service to check the fd against
 * @mgsid: id of the message that the fd is in
 * @index: index of the fd to check in the message's fd array
 * @cid: optional user pointer to store the channel id
 * @ctx: optional user pointer to store the channel context pointer
 *
 * Checks whether the fd at the given index within the message's fd
 * array is a channel to the service described by svcfd. If the fd is
 * a channel to the service, optionally return the channel's id and
 * context pointer before returning success.
 *
 * svcfd may be -1, in which case the channel is checked against svc.
 */
int servicefs_check_channel(struct service *svc, int svcfd, int msgid,
		int index, __s32 __user *cid, __u64 __user *ctx)
{
	struct service *check_svc;
	struct message *msg;
	struct channel *c;
	struct file *filp = NULL, *svc_filp = NULL;
	int ret = 0;

	/* determine which service to check the channel against */
	if (svcfd == THIS_SERVICE_FD) {
		check_svc = svc;
	} else {
		svc_filp = fget(svcfd);
		if (!svc_filp) {
			ret = -EBADF;
			goto error;
		}

		check_svc = servicefs_get_service_from_file(svc_filp);
		if (!check_svc) {
			ret = -EINVAL;
			goto error;
		}
	}

	/* validate arguments and retrieve the message struct */
	if (cid && !access_ok(VERIFY_WRITE, cid, sizeof(*cid))) {
		ret = -EFAULT;
		goto error;
	}

	if (ctx && !access_ok(VERIFY_WRITE, ctx, sizeof(*ctx))) {
		ret = -EFAULT;
		goto error;
	}

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg)) {
		ret = PTR_ERR(msg);
		goto error;
	}

	if (is_close_message(msg) || index < 0 || index >= msg->m_fdcnt) {
		mutex_unlock(&msg->m_mutex);
		ret = -EINVAL;
		goto error;
	}

	/* get a reference to the file object for the fd */
	filp = servicefs_fget(msg->m_task, msg->m_fds[index]);
	if (!filp) {
		mutex_unlock(&msg->m_mutex);
		ret = -EBADF;
		goto error;
	}

	/* allow other threads the access the message now */
	mutex_unlock(&msg->m_mutex);

	/* get the channel for this file object if it's a channel */
	c = servicefs_get_channel_from_file(filp);
	if (!c || c->c_service != check_svc) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	/* return channel id and context if requested */
	if (cid && __put_user(c->c_id, cid)) {
		ret = -EFAULT;
		goto error;
	}

	if (ctx && __put_user(c->c_context, ctx)) {
		ret = -EFAULT;
		goto error;
	}

error:
	if (svc_filp)
		fput(svc_filp);
	if (filp)
		fput(filp);
	return ret;
}

int servicefs_push_channel(struct service *svc, int svcfd, int msgid,
		int flags, __s32 __user *cid, __u64 ctx)
{
	struct pending_fd *pfd;
	struct message *msg;
	struct channel *c;
	struct file *file, *svc_file, *put_file;
	int fd;

	/* filter user flags and force RDONLY */
	flags &= O_NONBLOCK | O_CLOEXEC;
	flags |= O_RDONLY;

	/*
	 * The new channel is added either to this service or the one
	 * specified by svcfd.
	 */
	if (svcfd == THIS_SERVICE_FD) {
		svc_file = svc->s_filp;
		put_file = NULL;
	} else {
		svc_file = fget(svcfd);
		if (!svc_file)
			return -EBADF;
		if (!servicefs_get_service_from_file(svc_file)) {
			fput(svc_file);
			return -EINVAL;
		}
		put_file = svc_file;
	}

	if (cid && !access_ok(VERIFY_WRITE, cid, sizeof(*cid)))
		return -EFAULT;

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg)) {
		fd = PTR_ERR(msg);
		goto error_unlocked;
	}

	if (is_close_message(msg)) {
		fd = -EINVAL;
		goto error_locked;
	}

	/* msg is valid and msg->m_mutex is locked at this point */
	fd = servicefs_get_unused_fd_flags(msg->m_task, flags);
	if (unlikely(fd < 0))
		goto error_locked;

	file = servicefs_create_channel(svc_file, flags);
	if (IS_ERR(file)) {
		servicefs_put_unused_fd(msg->m_task, fd);
		fd = PTR_ERR(file);
		goto error_locked;
	}

	c = file->private_data;
	BUG_ON(!c);

	c->c_context = ctx;

	pfd = kmem_cache_alloc(servicefs_pending_fd_cache, GFP_KERNEL);
	if (!pfd) {
		fput(file);
		servicefs_put_unused_fd(msg->m_task, fd);
		fd = -ENOMEM;
		goto error_locked;
	}

	/* write the new channel id to the user, if cid is not NULL */
	if (cid && __put_user(c->c_id, cid)) {
		fput(file);
		pending_fd_free(pfd);
		servicefs_put_unused_fd(msg->m_task, fd);
		fd = -EFAULT;
		goto error_locked;
	}

	/* no more failures can occur, complete the channel setup */
	servicefs_complete_channel_setup(file);

	/* Add the pending fd to the message. */
	pfd->p_fd = fd;
	pfd->p_filp = file;
	list_add_tail(&pfd->p_pending_fds_node, &msg->m_pending_fds);

error_locked:
	mutex_unlock(&msg->m_mutex);
error_unlocked:
	if (put_file)
		fput(put_file);
	return fd;
}

int servicefs_close_channel(struct service *svc, int cid)
{
	struct channel *c;
	int ret = 0;

	pr_debug("cid=%d\n", cid);

	mutex_lock(&svc->s_channel_lock);

	c = __lookup_channel(svc, cid);
	if (!c) {
		ret = -ENOENT;
		goto done;
	}

	channel_cancel(c);

done:
	mutex_unlock(&svc->s_channel_lock);
	return ret;
}

int servicefs_set_service_context(struct service *svc, __u64 ctx)
{
	mutex_lock(&svc->s_mutex);
	svc->s_context = ctx;
	mutex_unlock(&svc->s_mutex);

	return 0;
}

int servicefs_set_channel_context(struct service *svc, int cid, __u64 ctx)
{
	struct channel *c;
	int ret = 0;

	pr_debug("cid=%d ctx=%p\n", cid, (void *) ctx);

	mutex_lock(&svc->s_channel_lock);

	c = __lookup_channel(svc, cid);
	if (!c) {
		ret = -ENOENT;
		goto done;
	}

	c->c_context = ctx;

done:
	mutex_unlock(&svc->s_channel_lock);
	return ret;
}

static inline bool is_message_pending(struct service *svc)
{
	bool pending;

	pending = is_service_canceled(svc)
			|| !list_empty_careful(&svc->s_impulses)
			|| !list_empty_careful(&svc->s_messages);

	return pending;
}

int servicefs_msg_recv(struct service *svc,
		struct servicefs_msg_info_struct __user *msg_info, long timeout)
{
	struct message *msg;
	struct impulse *impulse;
	__u64 s_context;
	__u64 c_context;
	int c_id;
	struct servicefs_msg_info_struct info_out;
	int ret;

	mutex_lock(&svc->s_mutex);

	do {
		if (is_service_canceled(svc)) {
			pr_debug("canceled_path\n");
			mutex_unlock(&svc->s_mutex);
			return -ESHUTDOWN;
		}

		/* impulses take priority over messages */
		if (!list_empty(&svc->s_impulses))
			goto impulse_path;

		if (!list_empty(&svc->s_messages))
			goto message_path;

		if (signal_pending(current)) {
			pr_debug("interrupted_path\n");
			mutex_unlock(&svc->s_mutex);
			return -ERESTARTSYS;
		}

		if (timeout <= 0) {
			pr_debug("timeout_path\n");
			mutex_unlock(&svc->s_mutex);
			return -ETIMEDOUT;
		}

		mutex_unlock(&svc->s_mutex);

		pr_debug("waiting for message: svc=%p timeout=%ld\n", svc, timeout);

		timeout = wait_event_interruptible_timeout(svc->s_wqreceivers,
				is_message_pending(svc), timeout);

		mutex_lock(&svc->s_mutex);
	} while (1);

message_path:
	pr_debug("message_path\n");
	msg = __peek_message(svc);
	BUG_ON(!msg);

	ret = __activate_message(msg);
	if (ret) {
		pr_debug("error: %d\n", ret);
		mutex_unlock(&svc->s_mutex);
		return ret;
	}

	/* grab the service/channel context and channel id before releasing s_mutex */
	s_context = svc->s_context;
	c_context = msg->m_channel->c_context;
	c_id = msg->m_channel->c_id;

	mutex_lock(&msg->m_mutex);
	mutex_unlock(&svc->s_mutex);

	pr_debug("m_id=%d m_op=%d m_sbuf.i_len=%zu m_rbuf.i_len=%zu msg_info=%p\n",
			msg->m_id, msg->m_op, msg->m_sbuf.i_len, msg->m_rbuf.i_len, msg_info);

	info_out.pid = msg->m_pid;
	info_out.tid = msg->m_tid;
	info_out.cid = c_id;
	info_out.mid = msg->m_id;
	info_out.euid = msg->m_euid;
	info_out.egid = msg->m_egid;
	info_out.service_private = s_context;
	info_out.channel_private = c_context;
	info_out.op = msg->m_op;
	info_out.flags = 0;
	info_out.send_len = msg->m_sbuf.i_len;
	info_out.recv_len = msg->m_rbuf.i_len;
	info_out.fd_count = msg->m_fdcnt;

	/* clear the impulse payload to prevent leaking stack contents */
	memset(info_out.impulse, 0, sizeof(info_out.impulse));

	if (copy_to_user(msg_info, &info_out, sizeof(info_out))) {
		pr_debug("Fault transferring msg info: pid=%d\n",
				pid_vnr(task_tgid(current)));
		mutex_unlock(&msg->m_mutex);
		message_put(svc, msg);
		return -EFAULT;
	}

	mutex_unlock(&msg->m_mutex);
	return 0;

impulse_path:
	pr_debug("impulse_path\n");
	impulse = __dequeue_impulse(svc);
	BUG_ON(!impulse);

	/* grab the service/channel context and channel id before releasing s_mutex */
	s_context = svc->s_context;
	c_context = impulse->i_channel->c_context;
	c_id = impulse->i_channel->c_id;

	mutex_unlock(&svc->s_mutex);

	pr_debug("i_op=%d i_len=%u\n", impulse->i_op, impulse->i_len);

	info_out.pid = impulse->i_pid;
	info_out.tid = impulse->i_tid;
	info_out.cid = c_id;
	info_out.mid = MESSAGE_NO_ID; // impulses have no message id.
	info_out.euid = impulse->i_euid;
	info_out.egid = impulse->i_egid;
	info_out.service_private = s_context;
	info_out.channel_private = c_context;
	info_out.op = impulse->i_op;
	info_out.flags = 0;
	info_out.send_len = impulse->i_len;
	info_out.recv_len = 0;
	info_out.fd_count = 0;

	/* copy the impulse payload and clear the remaining bytes */
	memcpy(info_out.impulse, impulse->i_data, impulse->i_len);
	memset(((uint8_t *) info_out.impulse) + impulse->i_len, 0,
			sizeof(info_out.impulse) - impulse->i_len);

	impulse_free(impulse);

	if (copy_to_user(msg_info, &info_out, sizeof(info_out))) {
		pr_debug("Fault transferring msg info: pid=%d\n",
				pid_vnr(task_tgid(current)));
		return -EFAULT;
	}

	return 0;
}

ssize_t servicefs_msg_readv(struct service *svc, int msgid,
		const struct iovec *vec, size_t cnt)
{
	struct message *msg;
	ssize_t ret;
	struct iov_buffer buf;
	bool remote_fault;

	iov_buffer_uinit(&buf, vec, cnt);

	pr_debug("svc=%p msgid=%d length=%zu\n", svc, msgid, buf.i_len);

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	ret = vm_transfer_from_remote(&msg->m_sbuf, &buf, msg->m_task,
			&remote_fault);

	mutex_unlock(&msg->m_mutex);

	/*
	 * use EACCESS to communicate that the remote process faulted during
	 * the payload transfer.
	 */
	if (ret == -EFAULT && remote_fault)
		ret = -EACCES;

	return ret;
}

ssize_t servicefs_msg_writev(struct service *svc, int msgid,
		const struct iovec *vec, size_t cnt)
{
	struct message *msg;
	ssize_t ret;
	struct iov_buffer buf;
	bool remote_fault;

	iov_buffer_uinit(&buf, vec, cnt);

	pr_debug("svc=%p msgid=%d length=%zu\n", svc, msgid, buf.i_len);

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	ret = vm_transfer_to_remote(&msg->m_rbuf, &buf, msg->m_task,
			&remote_fault);

	mutex_unlock(&msg->m_mutex);

	/*
	 * use EACCES to communicate that the remote process faulted during
	 * the payload transfer.
	 */
	if (ret == -EFAULT && remote_fault)
		ret = -EACCES;

	return ret;
}

int servicefs_msg_seek(struct service *svc, int msgid, long offset,
		int whence)
{
	return -EINVAL;
}

ssize_t servicefs_msg_busv(struct service *svc, int dst_msgid, long dst_off,
		int src_msgid, long src_off, size_t len)
{
	return -EINVAL;
}

int servicefs_msg_reply(struct service *svc, int msgid, ssize_t retcode)
{
	struct message *msg;
	int ret = 0;

	pr_debug("svc=%p msgid=%d retcode=%zd\n", svc, msgid, retcode);

	/*
	 * Make sure the return code is in the valid range [-MAX_ERRNO, MAX_INT].
	 * TODO: block special return values, such as ERESTARTSYS.
	 */
	if (retcode < -MAX_ERRNO)
		return -EINVAL;

	mutex_lock(&svc->s_mutex);

	msg = __lookup_message(svc, msgid);
	if (!msg) {
		ret = -ENOENT;
		goto error;
	}

	__message_complete(msg, retcode);

error:
	mutex_unlock(&svc->s_mutex);
	return ret;
}

int servicefs_msg_reply_fd(struct service *svc, int msgid, unsigned int pushfd)
{
	struct message *msg;
	int ret = 0;
	int newfd;
	struct file *filp;

	pr_debug("svc=%p msgid=%d pushfd=%u\n", svc, msgid, pushfd);

	mutex_lock(&svc->s_mutex);

	msg = __lookup_message(svc, msgid);
	if (!msg) {
		ret = -ENOENT;
		goto error_svc_unlock;
	}

	mutex_lock(&msg->m_mutex);

	if (is_message_interrupted(msg)) {
		mutex_unlock(&msg->m_mutex);
		__message_put(msg);
		ret = -EINTR;
		goto error_svc_unlock;
	}

	if (is_close_message(msg))
		goto close_message;

	filp = fget(pushfd);
	if (!filp) {
		ret = -EINVAL;
		goto error_msg_unlock;
	}

	// TODO: add selinux check for FD__USE and other relevant checks
	// look in security/selinux/hooks.c

	newfd = servicefs_get_unused_fd_flags(msg->m_task, 0);
	if (newfd < 0) {
		fput(filp);
		ret = newfd;
		goto error_msg_unlock;
	}

	servicefs_fd_install(msg->m_task, newfd, filp);

close_message:
	mutex_unlock(&msg->m_mutex);

	__message_complete(msg, newfd);

	mutex_unlock(&svc->s_mutex);
	return 0;

error_msg_unlock:
	mutex_unlock(&msg->m_mutex);
error_svc_unlock:
	mutex_unlock(&svc->s_mutex);
	return ret;
}

int servicefs_msg_push_fd(struct service *svc, int msgid, unsigned int pushfd)
{
	struct message *msg;
	struct pending_fd *pfd;
	int ret = 0;
	int newfd;
	struct file *filp;

	pr_debug("svc=%p msgid=%d pushfd=%u\n", svc, msgid, pushfd);

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	if (is_close_message(msg)) {
		ret = -EINVAL;
		goto error;
	}

	filp = fget(pushfd);
	if (!filp) {
		ret = -EINVAL;
		goto error;
	}

	// TODO: add selinux check for FD__USE and other relevant checks
	// look in security/selinux/hooks.c

	newfd = servicefs_get_unused_fd_flags(msg->m_task, 0);
	if (newfd < 0) {
		fput(filp);
		ret = newfd;
		goto error;
	}

	pfd = kmem_cache_alloc(servicefs_pending_fd_cache, GFP_KERNEL);
	if (!pfd) {
		fput(filp);
		servicefs_put_unused_fd(msg->m_task, newfd);
		ret = -ENOMEM;
		goto error;
	}

	/* Add the pending fd to the message. */
	pfd->p_fd = newfd;
	pfd->p_filp = filp;
	list_add_tail(&pfd->p_pending_fds_node, &msg->m_pending_fds);

	ret = newfd;

error:
	mutex_unlock(&msg->m_mutex);
	return ret;
}

int servicefs_msg_get_fd(struct service *svc, int msgid, unsigned int index)
{
	struct message *msg;
	int ret = 0;
	int newfd;
	int getfd;
	struct file *filp;

	pr_debug("svc=%p msgid=%d index=%u\n", svc, msgid, index);

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	if (is_close_message(msg) || index >= msg->m_fdcnt) {
		ret = -EINVAL;
		goto error;
	}
	getfd = msg->m_fds[index];

	filp = servicefs_fget(msg->m_task, getfd);
	if (!filp) {
		ret = -EBADF;
		goto error;
	}

	// TODO: add selinux check for FD__USE and other relevant checks
	// look in security/selinux/hooks.c

	newfd = servicefs_get_unused_fd_flags(current, 0);
	if (newfd < 0) {
		fput(filp);
		ret = newfd;
		goto error;
	}

	servicefs_fd_install(current, newfd, filp);
	ret = newfd;

error:
	mutex_unlock(&msg->m_mutex);
	return ret;
}

int servicefs_mod_channel_events(struct service *svc, int cid,
		int clr, int set)
{
	int ret = 0;
	struct channel *c;

	mutex_lock(&svc->s_channel_lock);

	c = __lookup_channel(svc, cid);
	if (!c) {
		ret = -ENOENT;
		goto done;
	}

	servicefs_atomic_update_mask(&c->c_events, clr, set);
	wake_up_poll(&c->c_waitqueue, (long) atomic_read(&c->c_events));

done:
	mutex_unlock(&svc->s_channel_lock);
	return ret;

}

ssize_t servicefs_msg_sendv(struct channel *c, int op,
		const struct iovec *svec, size_t scnt,
		const struct iovec *rvec, size_t rcnt,
		const __s32 *fds, size_t fdcnt, long task_state)
{
	struct message *msg;
	struct service *svc;
	int ret;

	svc = c->c_service;
	BUG_ON(!svc);

	msg = message_new(svc, c, op, svec, scnt, rvec, rcnt, fds, fdcnt, current);
	if (!msg)
		return -ENOMEM;

	pr_debug("msg=%p op=%d scnt=%zu slen=%zu rcnt=%zu rlen=%zu fdcnt=%zu\n",
			msg, op, msg->m_sbuf.i_cnt, msg->m_sbuf.i_len,
			msg->m_rbuf.i_cnt, msg->m_rbuf.i_len, msg->m_fdcnt);

	mutex_lock(&svc->s_mutex);

	if (is_channel_canceled(c) || is_service_canceled(svc)) {
		pr_debug("cid=%d channel_canceled=%d service_canceled=%d\n",
				c->c_id, is_channel_canceled(c), is_service_canceled(svc));
		__message_put(msg);
		mutex_unlock(&svc->s_mutex);
		return -ESHUTDOWN;
	}

	list_add_tail(&msg->m_messages_node, &svc->s_messages);

	pr_debug("svc=%p s_wqreceivers=%s s_wqselect=%s\n", svc,
		waitqueue_active(&svc->s_wqreceivers) ? "active" : "empty",
		waitqueue_active(&svc->s_wqselect) ? "active" : "empty");

	if (waitqueue_active(&svc->s_wqreceivers))
		wake_up(&svc->s_wqreceivers);
	else
		wake_up_poll(&svc->s_wqselect, POLLIN | POLLRDNORM);

	mutex_unlock(&svc->s_mutex);

	/* wait for message reply/cancel or a signal if interruptible */
	if (task_state == TASK_INTERRUPTIBLE)
		wait_event_interruptible(msg->m_waitqueue, is_message_completed(msg));
	else
		wait_event(msg->m_waitqueue, is_message_completed(msg));

	mutex_lock(&msg->m_mutex);

	if (!is_message_completed(msg)) {
		servicefs_atomic_mask_set(&msg->m_flags, MESSAGE_FLAGS_INTERRUPTED);
		msg->m_status = -EINTR;
	}

	ret = msg->m_status;

	mutex_unlock(&msg->m_mutex);

	/* release the message now that we're done with it */
	message_put(svc, msg);

	pr_debug("ret=%d\n", ret);
	return ret;
}

int servicefs_msg_send_impulse(struct channel *c, int op,
		void __user *buf, size_t len)
{
	struct impulse *impulse;
	struct service *svc;
	kuid_t kuid;
	kgid_t kgid;

	svc = c->c_service;
	BUG_ON(!svc);

	if (len > sizeof(impulse->i_data))
		return -EINVAL;

	impulse = kmem_cache_alloc(servicefs_impulse_cache, GFP_KERNEL);
	if (!impulse)
		return -ENOMEM;

	INIT_LIST_HEAD(&impulse->i_impulses_node);
	impulse->i_op = op;
	impulse->i_len = len;

	/*
	 * The impulse holds a ref to the channel. The channel file object also
	 * holds a ref to the channel, which is guaranteed to be maintained for
	 * the duration of this function because the ioctl holds a ref to the
	 * channel file object. The channel can be canceled however, which is
	 * handled below.
	 */
	channel_get(c);
	impulse->i_channel = c;

	current_euid_egid(&kuid, &kgid);
	impulse->i_euid = __kuid_val(kuid);
	impulse->i_egid = __kgid_val(kgid);

	impulse->i_pid = pid_vnr(task_tgid(current));
	impulse->i_tid = pid_vnr(task_pid(current));

	if (len > 0 && copy_from_user(&impulse->i_data, buf, len)) {
		pr_debug("Fault transferring impulse payload: pid=%d\n",
			pid_vnr(task_tgid(current)));
		impulse_free(impulse);
		return -EFAULT;
	}

	mutex_lock(&svc->s_mutex);

	if (is_channel_canceled(c) || is_service_canceled(svc)) {
		pr_debug("cid=%d channel_canceled=%d service_canceled=%d\n",
				c->c_id, is_channel_canceled(c), is_service_canceled(svc));
		mutex_unlock(&svc->s_mutex);
		impulse_free(impulse);
		return -ESHUTDOWN;
	}

	list_add_tail(&impulse->i_impulses_node, &svc->s_impulses);

	pr_debug("svc=%p s_wqreceivers=%s s_wqselect=%s\n", svc,
		waitqueue_active(&svc->s_wqreceivers) ? "active" : "empty",
		waitqueue_active(&svc->s_wqselect) ? "active" : "empty");

	if (waitqueue_active(&svc->s_wqreceivers))
		wake_up(&svc->s_wqreceivers);
	else
		wake_up_poll(&svc->s_wqselect, POLLIN | POLLRDNORM);

	mutex_unlock(&svc->s_mutex);

	return 0;
}
